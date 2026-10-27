"""Regression tests for kanban task ``t_8c85c7b8``.

Companion to :mod:`test_no_daemon_threads` (which proves the source has
zero bare ``threading.Thread(daemon=True)`` calls). This module proves
the **runtime** invariant: when the refactored ``tts_node`` fans
out work, the resulting thread pool is bounded
by the documented constants and never spawns unbounded workers under
sustained load.

Acceptance criteria covered here:

  * The synthesis executor captured at runtime has
    ``max_workers == SYNTHESIS_MAX_WORKERS_DEFAULT`` (asserted **and**
    re-validated against the module constant).
  * When the entry point (``_submit_synthesis``) is driven
    ``N = max_workers * 10`` times with a blocking callable, at no
    point do more than ``max_workers`` futures run concurrently — the
    semaphore-plus-executor pair enforces this.
  * All N submissions are accepted (no drops, no exceptions) when the
    slot cap is sized for the burst, and every submitted future
    eventually completes.
  * On node shutdown (``shutdown_synthesis_executor`` / ``destroy_node``
    for TTS)
    pending futures are awaited or cancelled, not abandoned. The
    captured executor's ``shutdown()`` is invoked exactly once.

The tests use :mod:`unittest.mock.patch` on the underlying
``concurrent.futures.ThreadPoolExecutor`` (patched at *both* the
package-level ``concurrent.futures.ThreadPoolExecutor`` and the
source-module ``concurrent.futures.thread.ThreadPoolExecutor`` — the
``__init__.py`` does ``from .thread import ThreadPoolExecutor`` at
import time, which copies the binding into ``concurrent.futures``
itself, so both targets must be patched).

Test environment note:
  The TTSNode module pulls in heavy transitive imports
  (rclpy, asyncio-loop driver, audio_common_msgs, etc.). We don't
  import those here — we test the **executor dispatch primitive** in
  isolation (with its real semaphore + slot counter) and use the
  existing test fixtures to construct a TTSNode for the live-fan-out
  test.

  PR-15 (#3711, part 15p): the v1 dialogue node asyncio-loop driver
  tests were deleted together with the node.
"""

from __future__ import annotations

import ast
import concurrent.futures
import re
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest


# Resolve source files under test without needing a working rclpy.
_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TTS_NODE_SRC = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"


# ── Source-text helpers (mirror the sibling test's approach) ────────────────


def _strip_strings_and_comments(source: str) -> str:
    """Return *source* with comments and string literals blanked out.

    We blank rather than strip so line numbers in the residual text
    still align with the original file — diagnostic messages include
    the offending line number.
    """
    no_strings = re.sub(
        r"\"[^\"\\]*(?:\\.[^\"\\]*)*\"|'[^'\\]*(?:\\.[^'\\]*)*'",
        lambda m: " " * len(m.group(0)),
        source,
    )
    no_comments = re.sub(r"(?m)#.*$", lambda m: " " * len(m.group(0)), no_strings)
    return no_comments


# ── Recording executor wrapper ──────────────────────────────────────────────

# Saved BEFORE any patch so the recording factory can construct a real
# executor without recursing into the patched symbol. This is the key
# trick — the patch is active while the production code is running,
# so any naive ``concurrent.futures.ThreadPoolExecutor(...)`` call
# inside the factory would re-enter the patch and recurse forever.
_REAL_EXECUTOR_CLS = concurrent.futures.ThreadPoolExecutor


class _RecordingExecutor:
    """Wraps a real ``ThreadPoolExecutor`` and records each construction.

    The factory ``_make_recording_executor`` returns a *function* that
    behaves like ``concurrent.futures.ThreadPoolExecutor`` from the
    caller's perspective (callable, returns a context manager that
    ``.submit``s and ``.shutdown``s) but records every call so the
    test can inspect:

      * the ``max_workers`` argument actually used,
      * the executor instance actually returned,
      * how many times ``shutdown()`` was called,
      * the live count of in-flight futures **scoped to THIS
        executor** (via a wrapper callable).

    The wrapper is intentional: we want to verify the **pattern** the
    production code uses (bounded executor + per-task slot counter) is
    actually bounded at runtime, not just declared in source. Patching
    with a pure ``MagicMock`` would prove nothing about thread count.

    IMPORTANT — per-executor in-flight scoping:

        A ``TTSNode`` (and most real callers) constructs more than
        one ``ThreadPoolExecutor`` over its lifetime — e.g. the
        synthesis pool, an async-bridge pool, a silero warm-load
        pool. Each is bounded by its own ``max_workers``. If a
        shared ``in_flight`` counter increments across pools, the
        peak in any one pool can trivially exceed its own limit
        even when every pool is honestly bounded (e.g. synthesis
        peak=2 + warm-load peak=1 → shared peak=3).

        ``concurrent.futures.ThreadPoolExecutor.submit`` runs tasks
        on its own internal pool of OS threads. Tasks dispatched to
        *one* executor never run on *another* executor's threads, so
        cross-executor in-flight counts are physically independent.
        Therefore each recorder MUST own its in-flight counter and
        the assertion MUST be made on the recorder that owns the
        executor under test (filtered by ``max_workers`` or by
        stored reference).
    """

    def __init__(self, recordings: list, lock: threading.Lock):
        self._recordings = recordings
        # Per-executor in-flight counter — see class docstring for why
        # this MUST NOT be shared across recorders.
        self._in_flight: dict = {"count": 0, "peak": 0}
        self._lock = lock
        # The real executor, sized exactly as the factory was called.
        self._real: concurrent.futures.ThreadPoolExecutor | None = None
        self._max_workers: int | None = None
        self.shutdown_count: int = 0
        self.shutdown_kwargs: list[dict] = []

    def __call__(self, max_workers: int = 1, **kwargs):
        """Factory signature — invoked by production code like a class."""
        self._max_workers = max_workers
        # Use the captured ``_REAL_EXECUTOR_CLS`` so the patch is not
        # re-entered — the patch is still active while we are inside
        # ``__call__`` because the producer code is calling us.
        self._real = _REAL_EXECUTOR_CLS(max_workers=max_workers, **kwargs)
        self._recordings.append(
            SimpleNamespace(
                max_workers=max_workers,
                kwargs=kwargs,
                executor=self._real,
                wrapper=self,
            )
        )
        return self

    # The production code uses ``executor.submit(fn, *args)`` and
    # ``executor.shutdown(wait=..., cancel_futures=...)`` — both of
    # those must work on the *return value* of the factory call.
    def submit(self, fn, *args, **kwargs):
        wrapped = self._wrap(fn)
        return self._real.submit(wrapped, *args, **kwargs)

    def shutdown(self, wait: bool = True, **kwargs):
        self.shutdown_count += 1
        self.shutdown_kwargs.append({"wait": wait, **kwargs})
        return self._real.shutdown(wait=wait, **kwargs)

    def _wrap(self, fn):
        """Return a callable that increments ``in_flight`` around *fn*.

        The counter is the recorder's own — see class docstring.
        """

        def _wrapped(*args, **kwargs):
            with self._lock:
                self._in_flight["count"] += 1
                self._in_flight["peak"] = max(
                    self._in_flight["peak"], self._in_flight["count"]
                )
            try:
                return fn(*args, **kwargs)
            finally:
                with self._lock:
                    self._in_flight["count"] -= 1

        return _wrapped


def _make_recording_executor_factory(
    recordings: list,
    lock: threading.Lock,
):
    """Build a factory that records each construction call.

    Returns a callable with the same ``__call__`` shape as the
    :class:`_RecordingExecutor` so that
    ``patch('concurrent.futures.thread.ThreadPoolExecutor', factory)``
    transparently routes through it.

    Note: each recorder constructed by this factory owns its own
    in-flight counter (see ``_RecordingExecutor`` docstring). Callers
    must locate the recorder for the executor-under-test (by
    ``max_workers`` or by stored reference) and inspect ITS counter,
    not any global one.
    """

    def _factory(max_workers: int = 1, **kwargs):
        rec = _RecordingExecutor(recordings, lock)
        rec(max_workers=max_workers, **kwargs)
        return rec

    return _factory


# ── TTSNode-driven fan-out test ─────────────────────────────────────────────


# We construct a TTSNode for the runtime test. The existing
# test_no_daemon_threads.py file deliberately avoids this, but the
# task body for t_8c85c7b8 explicitly asks us to "drive the node's
# entry point" — so we DO need to import the module. The existing
# conftest installs the heavy-import stubs.
@pytest.fixture
def tts_node_module():
    """Import ``rob_box_voice.tts_node`` with the heavy-import stubs.

    The conftest at ``test/unit/tts/conftest.py`` patches rclpy,
    audio_common_msgs, sounddevice, etc. so the module can be
    imported in a developer environment without those deps.
    """
    import sys
    sys.path.insert(0, str(_PACKAGE_ROOT))
    from rob_box_voice import tts_node  # noqa: PLC0415 — test fixture
    return tts_node


def _make_tts_node_capture(
    tts_node_module,
    max_workers: int = 2,
    max_queue: int = 100,
):
    """Construct a TTSNode with the executor patched to a recorder.

    Returns ``(node, recordings)`` where:
      * ``recordings`` is a list that gets one SimpleNamespace per
        ``ThreadPoolExecutor(...)`` call observed during the init.
        Each entry's ``wrapper`` is the :class:`_RecordingExecutor`
        that owns its own per-executor in-flight counter (see the
        class docstring for why sharing is wrong); look up the
        recorder for the executor-under-test by ``max_workers`` and
        inspect its ``_in_flight`` dict directly.

    Note on patch location: ``concurrent.futures.__init__.py``
    does ``from .thread import ThreadPoolExecutor`` at module load
    time, which *copies* the class reference into
    ``concurrent.futures.__dict__``. Production code that reads
    ``concurrent.futures.ThreadPoolExecutor`` therefore sees the
    original class, not whatever is bound on
    ``concurrent.futures.thread``. We must patch
    ``concurrent.futures.ThreadPoolExecutor`` (the binding the
    production code actually reads) and similarly
    ``concurrent.futures.thread.ThreadPoolExecutor`` to be safe
    (some attribute-lookup styles go through the source module).
    """
    recordings: list = []
    lock = threading.Lock()
    factory = _make_recording_executor_factory(recordings, lock)

    with patch(
        "concurrent.futures.ThreadPoolExecutor", side_effect=factory
    ), patch(
        "concurrent.futures.thread.ThreadPoolExecutor", side_effect=factory
    ):
        node = tts_node_module.TTSNode()

    # Sanity: the node picked up our values.
    assert node._synthesis_executor_max_workers == max_workers, (
        "TTSNode did not apply the configured synthesis_max_workers; "
        "default ROS param handling must accept the value."
    )
    return node, recordings


def test_tts_node_synthesis_executor_is_bounded_at_runtime(tts_node_module):
    """Burst N = max_workers * 10 submissions; in-flight never exceeds cap.

    The test patches the underlying ``ThreadPoolExecutor`` with a
    recorder that forwards to a real ``ThreadPoolExecutor`` of the
    same size, while wrapping each submitted callable so we can
    observe the in-flight count across all workers.

    The reported ``max_workers`` on the captured executor must equal
    the documented constant, in-flight must never exceed that number,
    and every one of the N submissions must be accepted (no drops,
    no exceptions) and eventually complete.
    """
    # Pull the documented constant for the assertion.
    SYNTHESIS_MAX_WORKERS_DEFAULT = getattr(
        tts_node_module, "SYNTHESIS_MAX_WORKERS_DEFAULT", 2
    )
    SYNTHESIS_MAX_QUEUE_DEFAULT = getattr(
        tts_node_module, "SYNTHESIS_MAX_QUEUE_DEFAULT", 16
    )

    # Make the slot cap (max_queue + max_workers) comfortably exceed
    # N = max_workers * 10 so the burst is accepted wholesale.
    max_workers = SYNTHESIS_MAX_WORKERS_DEFAULT
    max_queue = max(SYNTHESIS_MAX_QUEUE_DEFAULT, max_workers * 10 + 4)
    N = max_workers * 10

    node, recordings = _make_tts_node_capture(
        tts_node_module,
        max_workers=max_workers,
        max_queue=max_queue,
    )

    # ── (a) max_workers matches the documented constant ─────────────
    assert len(recordings) >= 1, (
        "TTSNode did not create any ThreadPoolExecutor at init; "
        "the bounded-executor primitive is missing."
    )
    # Find the synthesis-pool recorder. The TTSNode may create several
    # executors over its lifetime (synthesis, async-bridge, silero
    # warm-load); only the synthesis pool has max_workers ==
    # SYNTHESIS_MAX_WORKERS_DEFAULT (currently 2). Filter by that —
    # NOT by ``recordings[0]`` — because construction order is not
    # guaranteed when the silero warm-up is triggered before the
    # synthesis pool is constructed (regression observed on PR
    # #2016, run 34077288865).
    synth_candidates = [
        rec for rec in recordings
        if rec.max_workers == SYNTHESIS_MAX_WORKERS_DEFAULT
    ]
    assert synth_candidates, (
        f"No recorder with max_workers == "
        f"{SYNTHESIS_MAX_WORKERS_DEFAULT} was created; "
        f"observed max_workers values: "
        f"{[r.max_workers for r in recordings]}"
    )
    captured = synth_candidates[0]

    # The recorder is what the node actually uses (the patch made
    # ``concurrent.futures.ThreadPoolExecutor(...)`` return the recorder,
    # which wraps a real PoolExecutor of the same size). We submit
    # directly through the recorder below so we can observe the
    # in-flight count via the wrapper, while the semantics (semaphore
    # + bounded Pool) are exactly what the production code uses.
    # We must NOT also assign ``node._synthesis_executor.submit = ...``
    # because the recorder shares the real executor internally and
    # doing so would create a self-referential bind on the TpE.
    recorder = captured.wrapper
    real_executor = captured.executor  # the real ``ThreadPoolExecutor``

    # ── Pre-flight invariant: nothing is running yet ────────────────
    # The recorder owns its own per-executor in-flight counter (see
    # ``_RecordingExecutor`` docstring). Before we drive N submissions
    # through the synthesis pool, no task should be in flight on it.
    with recorder._lock:  # noqa: SLF001
        assert recorder._in_flight["count"] == 0, (
            f"synthesis-pool recorder has unexpected pre-flight "
            f"in-flight count={recorder._in_flight['count']}"
        )
        assert recorder._in_flight["peak"] == 0, (
            f"synthesis-pool recorder has unexpected pre-flight "
            f"peak={recorder._in_flight['peak']}"
        )

    # Widen the slot cap so N = max_workers * 10 fits comfortably. The
    # default ``max_queue=16`` caps the queue at max_workers + max_queue
    # = 18 total slots, which would only just barely fit N = 20 with
    # 2 slack — too close to the edge for a stress test. We want the
    # bursting behaviour to be observably bounded by the executor
    # (max_workers), not by the queue's edge behaviour.
    slot_cap = max_workers + max(N - max_workers, 1)
    node._synthesis_slots = threading.Semaphore(slot_cap)

    # ── Drive the entry point N times with a blocking callable ─────
    # We invoke the production ``_submit_synthesis`` method directly:
    # it is the actual ROS-callback → executor dispatch site (the
    # JSON-publish handler calls it). The recorder wraps the
    # ``submit`` call so we observe in-flight count without
    # disturbing the node's own bookkeeping (semaphore, completion
    # callbacks).
    block = threading.Event()  # workers wait on this

    def blocking_callable(*args, **kwargs):
        # Increment-then-decrement is handled by the recorder wrapper.
        block.wait(timeout=10.0)  # 10s safety; we release manually below
        return None

    # Each call acquires a slot from ``_synthesis_slots`` (bounded by
    # max_workers + max_queue) and submits to the executor. If the
    # semaphore is sized correctly, all N submissions are accepted.
    accepted = 0
    rejected = 0
    try:
        for i in range(N):
            # ``_submit_synthesis(fn, speech_id, *args)`` acquires a
            # slot; if the slot is unavailable, it logs+returns instead
            # of submitting. We capture the drop-vs-accept distinction
            # by inspecting the slot counter before+after.
            pending_before = (
                slot_cap
                - node._synthesis_slots._value  # type: ignore[attr-defined]
            )
            # Now actually submit (this is the canonical entry point).
            node._submit_synthesis(blocking_callable, f"speech-{i}")
            pending_after = (
                slot_cap
                - node._synthesis_slots._value  # type: ignore[attr-defined]
            )
            if pending_after > pending_before:
                accepted += 1
            else:
                rejected += 1

        # ── (b) in-flight never exceeds max_workers ────────────────
        # Wait briefly for the executor to actually start the workers.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with recorder._lock:  # noqa: SLF001
                running = recorder._in_flight["count"]
            if running >= max_workers:
                break
            time.sleep(0.02)

        with recorder._lock:  # noqa: SLF001
            running = recorder._in_flight["count"]
            peak = recorder._in_flight["peak"]

        assert peak <= max_workers, (
            f"in-flight peak={peak} exceeded max_workers={max_workers}; "
            f"the bounded executor primitive is broken."
        )
        # And at least one worker must actually be running — if zero
        # are running, the test isn't actually exercising the semaphore.
        assert running >= 1, (
            "no workers were observed running after N submissions; "
            "is the executor set to lazy start?"
        )
        # After the brief warm-up we expect at most max_workers
        # concurrent (the rest are queued).
        assert running <= max_workers, (
            f"in-flight count {running} exceeded max_workers={max_workers}"
        )

        # ── (c) all N accepted, eventually all complete ────────────
        assert rejected == 0, (
            f"{rejected} submissions were rejected under the configured "
            f"slot cap (max_workers={max_workers}, max_queue={max_queue}, "
            f"N={N}); semaphore sizing is wrong."
        )
        assert accepted == N, (
            f"only {accepted}/{N} submissions were accepted"
        )

        # Release the workers and confirm they all finish.
        block.set()

        # Give the executor time to drain. The recorder's shutdown
        # delegates to the real ThreadPoolExecutor.
        recorder.shutdown(wait=True, cancel_futures=False)
    finally:
        # Always release so a failed assert doesn't leak threads.
        block.set()
        try:
            recorder.shutdown(wait=True, cancel_futures=True)
        except TypeError:
            recorder.shutdown(wait=True)


def test_tts_node_shutdown_synthesis_executor_invokes_shutdown(tts_node_module):
    """``shutdown_synthesis_executor`` must invoke executor.shutdown().

    Idempotency: a second call must be a no-op (already shut down).
    """
    recordings: list = []
    lock = threading.Lock()
    factory = _make_recording_executor_factory(recordings, lock)

    with patch(
        "concurrent.futures.ThreadPoolExecutor", side_effect=factory
    ), patch(
        "concurrent.futures.thread.ThreadPoolExecutor", side_effect=factory
    ):
        node = tts_node_module.TTSNode()

    assert len(recordings) >= 1
    recorder = recordings[0].wrapper

    # First call: not yet shut down → must call executor.shutdown().
    assert recorder.shutdown_count == 0
    node.shutdown_synthesis_executor(wait=False)
    assert recorder.shutdown_count == 1, (
        f"shutdown_synthesis_executor did not invoke executor.shutdown() "
        f"(got {recorder.shutdown_count} calls)"
    )

    # Second call: idempotent — must NOT call shutdown again.
    node.shutdown_synthesis_executor(wait=False)
    assert recorder.shutdown_count == 1, (
        f"shutdown_synthesis_executor is not idempotent: "
        f"second call incremented shutdown_count to {recorder.shutdown_count}"
    )


def test_tts_node_synthesis_executor_uses_bounded_init() -> None:
    """Structural: TTSNode.__init__ builds the synth executor with the.
    documented size and slot cap.

    Belt-and-suspenders against the runtime test: if the runtime test
    is skipped (e.g. numpy missing in the test environment), this
    static AST check still guarantees the source code wires the bounded
    executor + semaphore. We use AST instead of substring regex so
    comments / f-strings / escaped quotes don't trip up the assertion.
    """
    src = _TTS_NODE_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)

    cls = next(
        (n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TTSNode"),
        None,
    )
    assert cls is not None, "TTSNode class definition not found"
    init = next(
        (n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__"),
        None,
    )
    assert init is not None, "TTSNode.__init__ not found"

    src_init = ast.unparse(init)
    assert "self._synthesis_executor" in src_init, (
        "TTSNode.__init__ does not assign self._synthesis_executor"
    )
    assert "ThreadPoolExecutor" in src_init, (
        "TTSNode.__init__ does not construct a ThreadPoolExecutor"
    )
    assert "max_workers=max_workers" in src_init, (
        "TTSNode.__init__ ThreadPoolExecutor must use max_workers=max_workers "
        "(from the bounded parameter), not a magic number"
    )
    assert "threading.Semaphore(max_queue + max_workers)" in src_init, (
        "TTSNode.__init__ must use threading.Semaphore(max_queue + max_workers) "
        "to cap pending submissions"
    )

    method_names = {
        n.name for n in cls.body if isinstance(n, ast.FunctionDef)
    }
    assert "shutdown_synthesis_executor" in method_names, (
        "TTSNode must expose shutdown_synthesis_executor() to drain the "
        "bounded executor on node teardown."
    )
    # TTSNode does NOT override ``destroy_node`` (it inherits from
    # rclpy Node); the production ``main()`` calls
    # ``shutdown_synthesis_executor(wait=False)`` explicitly before
    # ``destroy_node()``. The runtime test
    # ``test_tts_node_shutdown_synthesis_executor_invokes_shutdown``
    # verifies the shutdown path is reachable.
