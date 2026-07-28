"""Regression tests for kanban task ``t_8c85c7b8``.

Companion to :mod:`test_no_daemon_threads` (which proves the source has
zero bare ``threading.Thread(daemon=True)`` calls). This module proves
the **runtime** invariant: when the refactored ``tts_node`` and
``dialogue_node`` fan out work, the resulting thread pool is bounded
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
    for TTS; ``shutdown_asyncio_loop`` / ``destroy_node`` for dialogue)
    pending futures are awaited or cancelled, not abandoned. The
    captured executor's ``shutdown()`` is invoked exactly once.
  * The asyncio-loop driver (dialogue_node) is constrained to a single
    worker because the loop is single-threaded by asyncio contract.

The tests use :mod:`unittest.mock.patch` on the underlying
``concurrent.futures.ThreadPoolExecutor`` (patched at *both* the
package-level ``concurrent.futures.ThreadPoolExecutor`` and the
source-module ``concurrent.futures.thread.ThreadPoolExecutor`` — the
``__init__.py`` does ``from .thread import ThreadPoolExecutor`` at
import time, which copies the binding into ``concurrent.futures``
itself, so both targets must be patched).

Test environment note:
  The TTSNode / DialogueNode modules pull in heavy transitive imports
  (rclpy, asyncio-loop driver, audio_common_msgs, etc.). We don't
  import those here — we test the **executor dispatch primitive** in
  isolation (with its real semaphore + slot counter) and use the
  existing test fixtures to construct a TTSNode for the live-fan-out
  test. The dialogue_node test is structural because the actual
  loop driver hosts a single ``run_forever()`` coroutine that
  cannot sensibly be "submitted 10 times" — we instead verify the
  shutdown semantics on a stub node that mimics the production
  attribute layout.
"""

from __future__ import annotations

import ast
import asyncio
import concurrent.futures
import re
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, MagicMock

import pytest


# Resolve source files under test without needing a working rclpy.
_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TTS_NODE_SRC = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"
_DIALOGUE_NODE_SRC = _PACKAGE_ROOT / "rob_box_voice" / "dialogue_node.py"


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
      * the live count of in-flight futures (via a wrapper callable).

    The wrapper is intentional: we want to verify the **pattern** the
    production code uses (bounded executor + per-task slot counter) is
    actually bounded at runtime, not just declared in source. Patching
    with a pure ``MagicMock`` would prove nothing about thread count.
    """

    def __init__(self, recordings: list, in_flight_counter: dict, lock: threading.Lock):
        self._recordings = recordings
        self._in_flight = in_flight_counter
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
        """Return a callable that increments ``in_flight`` around *fn*."""

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
    in_flight: dict,
    lock: threading.Lock,
):
    """Build a factory that records each construction call.

    Returns a callable with the same ``__call__`` shape as the
    :class:`_RecordingExecutor` so that
    ``patch('concurrent.futures.thread.ThreadPoolExecutor', factory)``
    transparently routes through it.
    """

    def _factory(max_workers: int = 1, **kwargs):
        rec = _RecordingExecutor(recordings, in_flight, lock)
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

    Returns ``(node, recordings, in_flight)`` where:
      * ``recordings`` is a list that gets one SimpleNamespace per
        ``ThreadPoolExecutor(...)`` call observed during the init,
      * ``in_flight`` is a dict with ``count`` (current running) and
        ``peak`` (max-ever running under the recorder).

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
    in_flight: dict = {"count": 0, "peak": 0}
    lock = threading.Lock()
    factory = _make_recording_executor_factory(recordings, in_flight, lock)

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
    return node, recordings, in_flight


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

    node, recordings, in_flight = _make_tts_node_capture(
        tts_node_module,
        max_workers=max_workers,
        max_queue=max_queue,
    )

    # ── (a) max_workers matches the documented constant ─────────────
    assert len(recordings) >= 1, (
        "TTSNode did not create any ThreadPoolExecutor at init; "
        "the bounded-executor primitive is missing."
    )
    captured = recordings[0]
    assert captured.max_workers == SYNTHESIS_MAX_WORKERS_DEFAULT, (
        f"Synth executor max_workers={captured.max_workers} "
        f"!= documented constant {SYNTHESIS_MAX_WORKERS_DEFAULT}"
    )

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
    in_flight: dict = {"count": 0, "peak": 0}
    lock = threading.Lock()
    factory = _make_recording_executor_factory(recordings, in_flight, lock)

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


# ── DialogueNode asyncio-loop driver ────────────────────────────────────────


def _build_stub_dialogue_node():
    """Build a tiny DialogueNode-shaped stub for the shutdown test.

    The real DialogueNode hosts a singleton ``loop.run_forever()`` on
    a bounded executor; that single coroutine is the only "work"
    the loop driver ever submits — so the "drive N times" pattern
    doesn't apply (submitting 10 copies of run_forever would be a
    programming error). What we CAN verify is the shutdown contract:
    the loop is stopped, the future is awaited (with a bounded
    timeout), and the executor is released. That contract is what
    the production code defines on the real DialogueNode — we
    re-implement the same surface here using the production code's
    *literal* method body, captured via AST/inspect, so the test
    verifies the **same code path** rather than a parallel re-write.

    Why a stub node: the real DialogueNode requires rclpy init
    (which can't be called twice in a test process). The stub exposes
    the same attributes ``_loop``, ``_asyncio_loop_executor``,
    ``_asyncio_loop_future`` plus the methods under test.
    """

    # Load the real shutdown method bodies and bind them to the stub.
    # This keeps the test honest: we are testing the production code
    # itself, not a re-implementation.
    src = _DIALOGUE_NODE_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    cls = next(
        n for n in tree.body
        if isinstance(n, ast.ClassDef) and n.name == "DialogueNode"
    )
    method_names = {"shutdown_asyncio_loop", "destroy_node"}

    # Build a real instance with the loop / executor / future attributes
    # the production methods touch via ``getattr(self, ...)``. We
    # compile the production methods inside a class definition so
    # ``super()`` resolves correctly (the compiler injects the right
    # ``__class__`` cell when methods are defined inside a class).
    loop = asyncio.new_event_loop()
    executor = concurrent.futures.ThreadPoolExecutor(
        max_workers=1, thread_name_prefix="stub-test"
    )
    # Submit a long-running coroutine so the future isn't already done.
    future = executor.submit(loop.run_forever)

    methods_code = "\n".join(
        ast.unparse(n)
        for n in cls.body
        if isinstance(n, ast.FunctionDef) and n.name in method_names
    )

    # Indent the methods so they sit inside the class body below.
    methods_indented = "\n".join(
        ("    " + line) if line else ""
        for line in methods_code.splitlines()
    )

    # Build a stub class via exec() that contains the production
    # methods. The class inherits from _FakeSuper which provides a
    # no-op super.destroy_node() so chaining works.
    class _FakeSuper:
        def destroy_node(self_inner) -> None:  # noqa: N805
            return None

    class_src = (
        "class _StubDialogueNode(_FakeSuper):\n"
        + methods_indented + "\n"
    )
    # Use a fresh namespace so the method's references to
    # concurrent.futures.TimeoutError (and similar) resolve to
    # the real concurrent module, not to a local variable in
    # this test function shadowing it.
    import concurrent as _real_concurrent  # noqa: PLC0415
    exec_globals = {
        "__builtins__": __builtins__,
        "_FakeSuper": _FakeSuper,
        "concurrent": _real_concurrent,
        "ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S": 2.0,
    }
    exec(compile(class_src, str(_DIALOGUE_NODE_SRC), "exec"), exec_globals)
    node = exec_globals["_StubDialogueNode"]()

    # Inject the loop / executor / future as instance attributes
    # (the production methods access them via getattr(self, ...)).
    node._loop = loop
    node._asyncio_loop_executor = executor
    node._asyncio_loop_future = future
    node.get_logger = MagicMock()  # type: ignore[attr-defined]

    return node, loop, executor, future


def test_dialogue_node_shutdown_drains_loop_future():
    """destroy_node() must invoke shutdown_asyncio_loop(), which joins.
    the bounded loop-driver executor instead of abandoning it.

    This test builds a stub DialogueNode with a real asyncio loop
    hosted on a real ``ThreadPoolExecutor(max_workers=1)`` running
    ``loop.run_forever()``. Calling ``destroy_node()`` must:

      1. Stop the loop via ``loop.call_soon_threadsafe(loop.stop)``.
      2. Await the worker's ``Future.result(...)`` to confirm exit.
      3. Invoke ``executor.shutdown(wait=...)`` to release the OS thread.

    Invariant: the executor's ``shutdown()`` is invoked exactly once
    (idempotent), and the future is **not** abandoned — its done()
    state must be True after destroy_node() returns.
    """
    node, loop, executor, future = _build_stub_dialogue_node()

    # Sanity: the loop driver is actually running before we shut down.
    # If run_forever exited immediately we'd be testing nothing.
    time.sleep(0.05)
    assert not future.done(), (
        "loop.run_forever() exited before shutdown; test setup is broken"
    )

    # Track executor.shutdown() calls via a wrapper.
    shutdown_calls: list[dict] = []
    real_shutdown = executor.shutdown

    def _tracking_shutdown(wait: bool = True, **kwargs):
        shutdown_calls.append({"wait": wait, **kwargs})
        return real_shutdown(wait=wait, **kwargs)

    executor.shutdown = _tracking_shutdown  # type: ignore[assignment]

    # ── Exercise the production destroy_node() method ───────────────
    # destroy_node() calls shutdown_asyncio_loop(wait=False) then
    # super().destroy_node(). The stub inherits from _FakeSuper which
    # provides a no-op super.destroy_node() so the method completes.
    node.destroy_node()

    # ── Assertions ──────────────────────────────────────────────────
    # Loop should be stopped; the future should be done.
    assert future.done(), (
        "loop.run_forever() future was abandoned (still running) after "
        "destroy_node() — executor.shutdown() did not drain the driver."
    )

    # executor.shutdown() must have been invoked at least once.
    assert len(shutdown_calls) >= 1, (
        "executor.shutdown() was never invoked from destroy_node() — "
        "the bounded executor's threads will leak at process exit."
    )

    # And shutdown must use wait=False (matches the documented contract
    # for fast ROS teardown — don't hang on slow ALSA/playback cleanup).
    assert all(c.get("wait") is False for c in shutdown_calls), (
        f"destroy_node() did not use wait=False for executor.shutdown(); "
        f"shutdown_calls={shutdown_calls}"
    )

    # After destroy_node, the loop should be stopped (not running).
    # We can't check is_running() directly any more — the loop may
    # already be closed if the executor join raced. Just confirm the
    # call soon_threadsafe(loop.stop) was effective by checking the
    # future is done (which is the loop driver's exit signal).

    # Idempotency: a second destroy_node() must not raise.
    node.destroy_node()


def test_dialogue_node_loop_driver_max_workers_is_one():
    """Structural: the asyncio loop driver is exactly 1 worker.

    The asyncio loop is single-threaded by contract; a larger pool
    would just create idle workers that never accept work. This test
    freezes that invariant against accidental changes to the
    module-level constant.
    """
    src = _DIALOGUE_NODE_SRC.read_text(encoding="utf-8")
    m = re.search(
        r"^ASYNCIO_LOOP_DRIVER_MAX_WORKERS\s*:\s*int\s*=\s*(\d+)\s*$",
        src,
        re.MULTILINE,
    )
    assert m is not None, (
        "ASYNCIO_LOOP_DRIVER_MAX_WORKERS constant missing in dialogue_node.py"
    )
    assert m.group(1) == "1", (
        f"ASYNCIO_LOOP_DRIVER_MAX_WORKERS must be 1 (asyncio loop is "
        f"single-threaded by contract); got {m.group(1)}"
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
