"""Regression tests for kanban task ``t_5ef52557`` (gap G-933-B).

Background
----------
PR #933 (``fix(voice/tts): per-provider max chunk + retry-halve``) was
architecturally accepted but its side-effects leaked into the user
experience: when the primary Yandex gRPC TTS fails mid-dialog, the
fallback path lazy-loads Silero v5 *synchronously*, triggering a
**2-3 second pause** during which the user hears only silence.  The
architect verdict ``06-ARCHITECT-REVIEW-V3.md`` §2 (gap G-933-B,
HIGH severity) classified this as a UX illusion of hang.

The fix moves the cold-load into a daemon ``Thread`` started from
``TTSNode.__init__`` whenever Silero is *only* a fallback (i.e. the
primary provider is yandex or minimax).  The synchronous synth path
waits up to 1.5 s on a ``threading.Event`` (``self._silero_loaded``);
on timeout it skips playback for that one chunk (publishing a
``silero_warming`` marker on ``/voice/tts/finished``) so the user
hears the *next* fallback at full speed instead of a hang.

This module proves the runtime + structural invariants of the fix:

  1. When ``provider=yandex``, ``TTSNode.__init__`` launches a daemon
     ``Thread`` named ``silero-warm-load`` that calls
     ``_load_silero_model`` in the background.
  2. The ``_silero_loaded`` ``threading.Event`` is **always** set at
     the end of the warm-load — success OR failure — so the hot-path
     wait never blocks forever.
  3. The hot-path wait timeout is < typical cold-load (1.5 s vs ~2.7 s
     observed in voice_v3_postdeploy.log) — verified by AST.
  4. ``provider=silero`` does NOT spawn the warm-load thread; the
     synchronous load already covers it and the event is set inline.
  5. Repeated invocations of ``_start_silero_warm_load`` are no-ops
     (idempotency).
  6. On a successful warm-load the hot-path ``_synthesize_and_play``
     fallback branch doesn't pay the cold-load latency (the model
     attribute is already populated when we enter the branch).
  7. The hot-path skip-on-timeout publishes ``silero_warming:<id>``
     on ``/voice/tts/finished`` and clears ``processing_dialogue_id``
     so barge-in state stays consistent.

The tests use ``conftest.py``'s rclpy/grpc/torch stubs so
``rob_box_voice.tts_node`` is importable in a developer environment
without those heavy deps.  Where we need a real ``Thread`` (for
race-free assertions on the warm-load event) we use ``threading.Event``
on the live module object instead of patching the threading primitive.
"""

from __future__ import annotations

import ast
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest


_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TTS_NODE_SRC = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"


# ── Module-level helpers ────────────────────────────────────────────────────


def _parse_module(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def _find_class(tree: ast.Module, name: str) -> ast.ClassDef:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    raise AssertionError(f"Class {name!r} not found")


def _find_method(cls: ast.ClassDef, name: str) -> ast.FunctionDef:
    for node in cls.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"Method {name!r} not found in {cls.name}")


@pytest.fixture
def tts_node_module():
    """Import ``rob_box_voice.tts_node`` with the heavy-import stubs.

    The conftest at ``test/unit/tts/conftest.py`` patches rclpy,
    audio_common_msgs, sounddevice, etc. so the module can be imported
    in a developer environment without those deps.
    """
    import sys
    sys.path.insert(0, str(_PACKAGE_ROOT))
    from rob_box_voice import tts_node  # noqa: PLC0415 — test fixture
    return tts_node


def _destroy_node(node) -> None:
    """Tear down a FakeNode TTSNode in the conftest test environment.

    The conftest's ``rclpy.node.Node`` stub doesn't implement
    ``destroy_node``; calling it directly would ``AttributeError``.
    Instead we just clear heavy state so the GC can reclaim memory
    before the next test runs.
    """
    # Daemon thread will be killed when the test process exits; no
    # explicit join needed (and unsafe — the thread is intentionally
    # daemon=True so it never blocks shutdown).
    for attr in (
        "_synthesis_executor", "audio_pub", "state_pub", "finished_pub",
        "dialogue_sub", "tts_request_sub", "control_sub",
        "_new_dialogue_id_sub", "playback_manager", "yandex_stub",
    ):
        if hasattr(node, attr):
            try:
                setattr(node, attr, None)
            except Exception:  # noqa: BLE001
                pass


# ── Structural: source-level invariants ─────────────────────────────────────


def test_tts_node_defines_warm_load_helpers() -> None:
    """Structural: TTSNode exposes _silero_loaded Event + helpers.

    Catches accidental rename / deletion of the warm-load API surface.
    Uses AST so comments / string literals don't trip up the match.
    """
    tree = _parse_module(_TTS_NODE_SRC)
    cls = _find_class(tree, "TTSNode")

    method_names = {
        n.name for n in cls.body if isinstance(n, ast.FunctionDef)
    }
    required = {
        "_silero_loaded",          # threading.Event
        "_silero_warm_thread",     # Thread reference (daemon)
        "_silero_load_outcome",    # "ok" | "fail"
        "_silero_load_lock",       # locks writes to outcome
        "_start_silero_warm_load", # public-facing kick-off
        "_silero_warm_loader",     # thread entry point
    }
    # AST-level attribute assignments (in __init__ / methods) for the
    # threading primitives — to ensure they aren't only mentioned in
    # a docstring.
    init_src = ast.unparse(_find_method(cls, "__init__"))
    for attr in ("_silero_loaded", "_silero_warm_thread", "_silero_load_outcome", "_silero_load_lock"):
        assert f"self.{attr}" in init_src, (
            f"TTSNode.__init__ must assign self.{attr} "
            f"(gap G-933-B warm-load state)"
        )
    for method in ("_start_silero_warm_load", "_silero_warm_loader"):
        assert method in method_names, (
            f"TTSNode must define {method}() (gap G-933-B warm-load API)"
        )


def test_warm_load_thread_is_daemon() -> None:
    """Structural: the warm-load thread is daemon=True.

    A non-daemon thread would block process shutdown while the
    torch.package import is in flight; on ROS teardown that would
    manifest as a hang at exit.  Belt-and-suspenders guard.
    """
    tree = _parse_module(_TTS_NODE_SRC)
    cls = _find_class(tree, "TTSNode")
    start = _find_method(cls, "_start_silero_warm_load")
    src = ast.unparse(start)
    assert "daemon=True" in src, (
        "TTSNode._start_silero_warm_load must spawn a daemon thread "
        "so ROS node teardown doesn't block on a slow torch.package import"
    )
    # ``ast.unparse`` emits single quotes around string literals; the
    # actual source uses double quotes.  Accept either.
    assert (
        "name='silero-warm-load'" in src
        or 'name="silero-warm-load"' in src
    ), (
        "TTSNode._start_silero_warm_load must name its thread "
        "'silero-warm-load' for stack-trace clarity"
    )


def test_hot_path_wait_timeout_below_cold_load() -> None:
    """Structural: hot-path wait is < typical cold-load duration.

    voice_v3_postdeploy.log shows Silero cold-load ≈2.7 s.  The
    hot-path wait timeout must be strictly less than this so the
    skip-on-timeout path actually triggers (otherwise the user
    still hears the hang we're trying to eliminate).
    """
    tree = _parse_module(_TTS_NODE_SRC)
    cls = _find_class(tree, "TTSNode")
    synth = _find_method(cls, "_synthesize_and_play")
    src = ast.unparse(synth)
    # We accept either a literal 1.5 or a named constant; the literal
    # is fine because the body of the docstring documents the choice.
    assert "1.5" in src, (
        "_synthesize_and_play must explicitly wait 1.5 s on _silero_loaded "
        "(< observed cold-load of 2.7 s in voice_v3_postdeploy.log)"
    )
    assert "self._silero_loaded.wait(timeout=" in src, (
        "_synthesize_and_play must wait on self._silero_loaded (threading.Event) "
        "with a timeout, not block indefinitely"
    )


def test_warm_loader_always_sets_event() -> None:
    """Structural: warm-loader sets the event on both success and failure.

    If the event is only set on success, a load failure (e.g. no
    network for torch.hub fallback) would deadlock the hot-path
    waiting forever.  Both branches must call ``self._silero_loaded.set()``
    — once via the ``finally`` block.
    """
    tree = _parse_module(_TTS_NODE_SRC)
    cls = _find_class(tree, "TTSNode")
    loader = _find_method(cls, "_silero_warm_loader")
    src = ast.unparse(loader)
    # The contract is "always set" — done via `finally` so we accept
    # either explicit `finally:` clause + .set() inside it.
    assert "finally" in src, (
        "_silero_warm_loader must wrap its load attempt in try/finally "
        "so self._silero_loaded.set() is called even on exceptions"
    )
    assert "self._silero_loaded.set()" in src, (
        "_silero_warm_loader must call self._silero_loaded.set() "
        "(without this the hot-path blocks indefinitely on failure)"
    )


# ── Runtime: warm-load lifecycle ────────────────────────────────────────────


def _make_node(tts_node_module, provider: str = "yandex") -> "object":
    """Construct a TTSNode for the given provider, with torch stubbed.

    The conftest's torch mock is a bare MagicMock — any real
    torch.package call would explode.  We also stub the synchronous
    ``_load_silero_model`` to a no-op so the warm-load thread doesn't
    try to materialise a real model.  The daemon thread itself is
    real, just executing a stubbed ``_load_silero_model``.
    """
    # Patch the *method* on the class so both __init__ (sync path when
    # provider=silero) and the warm-load thread (provider=yandex path)
    # route through the stub.  ``autospec`` keeps the ``self`` arg
    # semantics so ``side_effect`` can mutate ``self.silero_model``.
    cls = tts_node_module.TTSNode

    def _fake_load(self):
        self.silero_model = MagicMock(name="FakeSileroModel")

    with patch.object(cls, "_load_silero_model", autospec=True, side_effect=_fake_load):
        node = cls()
    return node


def test_warm_load_spawns_daemon_thread_for_yandex_primary(tts_node_module) -> None:
    """provider=yandex → background warm-load thread is spawned at init.

    Without this, the first Yandex→Silero fallback pays the 2-3 s
    cold-load cost in the user-facing thread — exactly the bug we're
    fixing.
    """
    node = _make_node(tts_node_module, provider="yandex")

    try:
        # TTSNode.__init__ must have kicked off the warm-load.
        assert node._silero_warm_requested is True, (
            "TTSNode(provider='yandex') must request warm-load at __init__ "
            "so the first fallback doesn't pay the cold-load latency"
        )
        # Give the daemon thread a moment to finish its (stubbed) load.
        warmed = node._silero_loaded.wait(timeout=2.0)
        assert warmed, (
            "background warm-load thread did not finish within 2 s; "
            "daemon thread setup is broken"
        )
        # Outcome must be 'ok' (the stubbed load returns immediately).
        assert node._silero_load_outcome == "ok", (
            f"warm-load outcome expected 'ok', got {node._silero_load_outcome!r}"
        )
        # silero_model is populated — no need to lazy-load on first fallback.
        assert node.silero_model is not None, (
            "warm-loaded silero_model must be populated before first fallback"
        )
    finally:
        _destroy_node(node)


def test_no_warm_load_thread_for_silero_primary(tts_node_module) -> None:
    """provider=silero → synchronous load, no extra background thread.

    When Silero IS the primary, the constructor already loads it
    synchronously (so the very first dialogue works).  Spawning an
    additional background thread would be wasteful and slightly racy
    (one extra daemon thread, two places that could clobber the
    silero_model attribute).
    """
    cls = tts_node_module.TTSNode

    def _fake_load(self):
        self.silero_model = MagicMock(name="FakeSileroModel")

    # Patch the FakeNode's declare_parameter to coerce provider='silero'
    # for the duration of __init__.
    rclpy_node_mod = __import__("sys").modules["rclpy.node"]
    orig_declare = rclpy_node_mod.Node.declare_parameter

    def _patched_declare(self, name, default=None):
        if name == "provider":
            default = "silero"
        return orig_declare(self, name, default=default)

    with patch.object(
        cls, "_load_silero_model", autospec=True, side_effect=_fake_load
    ) as mock_load, patch.object(
        rclpy_node_mod.Node, "declare_parameter", _patched_declare
    ):
        node = cls()

    try:
        # The synchronous path ran the (stubbed) load exactly once.
        assert mock_load.call_count == 1, (
            f"provider=silero must load exactly once (sync path); "
            f"got {mock_load.call_count} calls"
        )
        # The warm-load thread flag is NOT set — we skipped that path.
        assert node._silero_warm_requested is False, (
            "provider=silero must NOT spawn a redundant warm-load thread "
            "(synchronous load already covered it)"
        )
        # Event is still set (so any caller that still calls
        # .wait(timeout=...) returns immediately).
        assert node._silero_loaded.is_set(), (
            "_silero_loaded must be set even on the sync path so "
            "hot-path wait() returns instantly"
        )
        assert node._silero_load_outcome == "ok"
    finally:
        _destroy_node(node)


def test_warm_load_is_idempotent(tts_node_module) -> None:
    """Repeated _start_silero_warm_load calls must be no-ops.

    Defensive against accidental double-init from a refactor that
    calls _start_silero_warm_load from both __init__ and a parameter
    callback (e.g. set_parameters for provider='silero'→'yandex').
    """
    node = _make_node(tts_node_module, provider="yandex")

    try:
        # Wait for the first warm-load to complete so the thread slot
        # is reusable for the assertion below.
        node._silero_loaded.wait(timeout=2.0)

        # Capture the original thread reference.
        first_thread = node._silero_warm_thread
        assert first_thread is not None

        # Calling _start_silero_warm_load again must NOT spawn a
        # second thread.
        node._start_silero_warm_load()
        assert node._silero_warm_thread is first_thread, (
            "_start_silero_warm_load must be idempotent (single "
            "warm-load thread per node lifetime)"
        )
    finally:
        _destroy_node(node)


# ── Issue #929: silero_warm_load=false (OOM mitigation) ─────────────────────


def test_warm_load_disabled_by_parameter(tts_node_module) -> None:
    """``silero_warm_load=false`` must NOT spawn the warm-load thread.

    Issue #929: the background warm-load keeps ~700 MB-1 GB RSS
    (PyTorch + Silero model) alive for the whole node lifetime, even
    when the primary provider (yandex/minimax) works fine.  In the
    2-4 GB container hosting 9 nodes the constant RSS contributed to
    the OOM kill of tts_node.  With ``silero_warm_load=false`` the
    model is loaded lazily on the first real fallback instead.
    """
    cls = tts_node_module.TTSNode

    def _fake_load(self):
        self.silero_model = MagicMock(name="FakeSileroModel")

    rclpy_node_mod = __import__("sys").modules["rclpy.node"]
    orig_declare = rclpy_node_mod.Node.declare_parameter

    def _patched_declare(self, name, default=None):
        if name == "silero_warm_load":
            default = False
        return orig_declare(self, name, default=default)

    with patch.object(
        cls, "_load_silero_model", autospec=True, side_effect=_fake_load
    ) as mock_load, patch.object(
        rclpy_node_mod.Node, "declare_parameter", _patched_declare
    ):
        node = cls()

    try:
        # No warm-load requested → no daemon thread, no background load.
        assert node._silero_warm_requested is False, (
            "silero_warm_load=false must NOT request the background warm-load"
        )
        assert mock_load.call_count == 0, (
            "silero_warm_load=false must not load the model in __init__ "
            f"(lazy path only); got {mock_load.call_count} calls"
        )
        assert node.silero_model is None, (
            "silero_warm_load=false must leave silero_model=None until "
            "the first real fallback"
        )
        # Flag is readable for hot-path branching.
        assert node.silero_warm_load_enabled is False, (
            "silero_warm_load_enabled must mirror the parameter"
        )
    finally:
        _destroy_node(node)


def test_warm_load_disabled_hot_path_lazy_loads(tts_node_module) -> None:
    """With warm-load disabled, hot-path fallback lazy-loads synchronously.

    Issue #929: the skip-on-timeout branch (G-933-B) exists for the
    *warm-load* case — an in-flight background load.  When
    ``silero_warm_load=false`` there is no background load, so the
    hot-path must NOT skip the chunk: it loads the model synchronously
    (first fallback pays the 2-3 s cold-load — acceptable for the
    emergency path).
    """
    cls = tts_node_module.TTSNode
    load_calls = []

    def _fake_load(self):
        load_calls.append(1)
        self.silero_model = MagicMock(name="FakeSileroModel")

    rclpy_node_mod = __import__("sys").modules["rclpy.node"]
    orig_declare = rclpy_node_mod.Node.declare_parameter

    def _patched_declare(self, name, default=None):
        if name == "silero_warm_load":
            default = False
        if name == "provider":
            default = "yandex"
        return orig_declare(self, name, default=default)

    node = None
    try:
        with patch.object(
            cls, "_load_silero_model", autospec=True, side_effect=_fake_load
        ) as mock_load, patch.object(
            rclpy_node_mod.Node, "declare_parameter", _patched_declare
        ):
            node = cls()

            # Hot-path: Yandex fails → Silero branch must lazy-load.
            # NOTE: _load_silero_model patch stays ACTIVE for the whole
            # with-block — the call below must hit the fake, not the real
            # torch.package loader.
            node.yandex_stub = MagicMock()
            synth_mock = MagicMock(
                side_effect=RuntimeError("simulated Yandex gRPC fail")
            )
            with patch.object(node, "_synthesize_yandex", synth_mock):
                t0 = time.monotonic()
                node._synthesize_and_play(
                    ssml="<speak>test</speak>",
                    text="test",
                    dialogue_id="dlg-lazy-load",
                )
                elapsed = time.monotonic() - t0

            # Sanity: no warm-load was started.
            assert node._silero_warm_requested is False

            # Lazy-load happens synchronously (mock returns instantly, so
            # elapsed is small — well below the 1.5 s skip threshold).
            assert load_calls, (
                "hot-path must call _load_silero_model when warm-load is disabled"
            )
            assert elapsed < 1.0, (
                f"lazy-load path must be synchronous and fast with stubbed model; "
                f"got {elapsed:.2f}s"
            )
            # No silero_warming skip marker — the chunk actually played.
            published = []
            for call in node.finished_pub.publish.call_args_list:
                try:
                    published.append(call.args[0].data)
                except Exception:  # noqa: BLE001
                    pass
            assert not any("silero_warming" in p for p in published), (
                f"warm-load-disabled path must NOT publish silero_warming skip "
                f"marker; got publishes: {published}"
            )
            # NOTE: processing_dialogue_id is intentionally NOT asserted
            # here — it is cleared at the very end of _synthesize_and_play
            # (playback completion / barge-in bookkeeping), which is
            # orthogonal to the warm-load-disabled lazy-load path.
    finally:
        if node is not None:
            _destroy_node(node)


def test_warm_load_failure_still_sets_event(tts_node_module) -> None:
    """If warm-load raises, the event must still fire.

    Without this, a failing warm-load would deadlock the hot-path
    forever (Event.wait(timeout=...) would always time out and we'd
    retry on every fallback).
    """
    # Stub _load_silero_model to raise (simulates torch.package
    # failure, network down for torch.hub, etc.).
    cls = tts_node_module.TTSNode
    with patch.object(
        cls, "_load_silero_model", autospec=True,
        side_effect=RuntimeError("simulated torch.package load failure"),
    ):
        node = cls()

    try:
        # Wait — the warm-load thread's _silero_warm_loader catches
        # the exception in its try/except and still calls .set() in
        # the finally block.
        warmed = node._silero_loaded.wait(timeout=2.0)
        assert warmed, (
            "_silero_loaded Event must be set even when warm-load fails; "
            "otherwise hot-path wait() would block forever"
        )
        assert node._silero_load_outcome == "fail", (
            f"outcome must be 'fail' on exception, got {node._silero_load_outcome!r}"
        )
        # silero_model stays None — the legacy sync path on hot-path
        # fallback will try once more (synchronously) before giving up.
        assert node.silero_model is None
    finally:
        _destroy_node(node)


def test_hot_path_skips_on_timeout_and_publishes_marker(tts_node_module) -> None:
    """If warm-load is slow (>1.5 s), hot-path skips playback for that chunk.

    Acceptance criterion: "TTSNode init не блокирует основной поток >0.5 с"
    AND "первый Yandex→Silero fallback имеет <0.5 с дополнительной
    задержки".  The trade-off: if warm-load takes longer than 1.5 s
    (e.g. cold cache, slow disk), we skip the *current* chunk so the
    user doesn't hear 2-3 s of silence; the *next* fallback will hit
    the (by then) warm model.
    """
    cls = tts_node_module.TTSNode
    slow_done = threading.Event()

    def _slow_load(self):
        # Simulate the real cold-load latency (~2.7 s in production).
        # We use a short sleep for the test (3 s > 1.5 s timeout)
        # so the assertion below is deterministic.
        time.sleep(3.0)
        self.silero_model = MagicMock(name="FakeSileroModel")
        slow_done.set()

    with patch.object(cls, "_load_silero_model", autospec=True, side_effect=_slow_load):
        node = cls()

    try:
        # Sanity: warm-load is in flight, event NOT yet set.
        assert not node._silero_loaded.is_set(), (
            "warm-load must still be in flight when slow_load is sleeping"
        )

        # Make the hot-path see Yandex fail so we go to the Silero branch.
        node.yandex_stub = MagicMock()  # so the Yandex branch is taken

        # Stub the Yandex synth to raise immediately so we fall through
        # to the Silero branch.
        with patch.object(
            node, "_synthesize_yandex",
            side_effect=RuntimeError("simulated Yandex gRPC fail"),
        ):
            t0 = time.monotonic()
            node._synthesize_and_play(
                ssml="<speak>test</speak>",
                text="test",
                dialogue_id="dlg-skip-test",
            )
            elapsed = time.monotonic() - t0

        # The hot-path should have waited ~1.5 s and then bailed out.
        assert 1.0 <= elapsed < 2.5, (
            f"hot-path wait must be ~1.5 s on warm-load timeout; "
            f"got {elapsed:.2f}s — either timeout missing or not bounded"
        )

        # finished_pub should have published 'silero_warming:<id>'.
        # The conftest's create_publisher returns a MagicMock, so
        # finished_pub.publish is a MagicMock we can inspect.
        published = []
        for call in node.finished_pub.publish.call_args_list:
            try:
                published.append(call.args[0].data)
            except Exception:  # noqa: BLE001
                pass
        assert any("silero_warming:dlg-skip-test" in p for p in published), (
            f"hot-path skip-on-timeout must publish "
            f"'silero_warming:<dialogue_id>' on /voice/tts/finished; "
            f"got publishes: {published}"
        )

        # processing_dialogue_id is cleared so barge-in bookkeeping
        # stays consistent (next dialogue can start cleanly).
        assert node.processing_dialogue_id is None, (
            "hot-path skip-on-timeout must clear processing_dialogue_id "
            "so downstream barge-in state isn't poisoned"
        )
    finally:
        # Wait for the slow thread to finish (or daemonize out).
        slow_done.wait(timeout=5.0)
        _destroy_node(node)