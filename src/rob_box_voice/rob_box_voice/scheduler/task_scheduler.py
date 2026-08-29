"""task_scheduler.py — Phase 1 MVP TaskScheduler (issue #968 §11.1).

Minimal viable scheduler that

1. accepts :class:`SchedulerTask` submissions,
2. routes each task to the right FIFO channel
   (voice / music / anim — see :class:`ChannelKind`),
3. runs the tasks on each channel **strictly sequentially** via an
   ``asyncio.Lock`` so two TTS requests never collide on the audio
   device and ``stop_music`` cannot outrun the TTS chunk that the
   scheduler just queued,
4. exposes a small ``wait_all`` / ``cancel`` / ``channel_status``
   surface for the LLM loop and integration tests.

The MVP deliberately keeps the executor interface trivial: callers
pass a :class:`TaskExecutor` — a coroutine that performs the actual
side-effect (publish on ``/voice/tts/control``, etc.) and returns a
:class:`TaskResult`. The MVP does NOT route through the
:class:`~rob_box_harness.core.acceptance.AcceptanceGate`; that is
Phase 1.5 (acceptance tool-calling, §11.2).

Threading / concurrency
-----------------------

The scheduler is built around ``asyncio``. All public methods are
coroutines (or coroutine-returning). A single ``threading.Lock``
guards ``self._tasks`` so the snapshot views are coherent when read
from a non-asyncio thread (e.g. a ROS2 callback). The per-channel
execution lock is an ``asyncio.Lock``; it MUST be awaited on the
loop that owns the scheduler (callers that need to drive it from a
worker thread should use ``asyncio.run_coroutine_threadsafe``).

Out of scope (Phase 1.5+)
-------------------------

* ``SchedulerEventBus`` (Phase 2)
* Reflex / priority queue (Phase 1.5+)
* Two-tier quick-decide (Phase 2)
* ``SegmentEstimator`` / speculative pre-gen (Phase 3)
* Persistent timers / timeouts (Phase 1.5)
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Dict, List, Mapping, Optional, Protocol, runtime_checkable

from .delta import DeltaOp, DeltaOpKind, TaskDelta
from .event_bus import EventEnvelope

_LOG = logging.getLogger(__name__)

#: S10 (scheduler-segments-merge, issue #968, §4.5) — events at this
#: priority never satisfy the auto-trigger's "unapplied event" condition
#: (anti-pattern: "При ``priority=low`` events (IGNORE / шум)").
_LOW_PRIORITY = "low"


# ---------------------------------------------------------------------------
# Public enums & dataclasses
# ---------------------------------------------------------------------------


class ChannelKind(str, Enum):
    """Scheduler channels for Phase 1 MVP.

    Mirrors the §3.1 trio (voice / music / anim). ``UNKNOWN`` is
    intentionally absent — the MVP does not have to absorb
    nav/mapping/mutate, those land in Phase 1.5 via
    :class:`~rob_box_harness.core.acceptance.AcceptanceGate`. Tasks
    whose :attr:`SchedulerTask.channel` is not a known
    :class:`ChannelKind` are rejected with :class:`TaskSubmitError`.
    """

    VOICE = "voice"
    MUSIC = "music"
    ANIM = "anim"


class TaskStatus(str, Enum):
    """Lifecycle states a :class:`SchedulerTask` walks through.

    The transitions are:

    * ``QUEUED``     — set by :meth:`TaskScheduler.submit` after the
      task is appended to a channel queue.
    * ``SCHEDULED``  — set by :meth:`Channel._pump` once the task is
      dequeued and the channel lock is acquired (i.e. it is the next
      task to run).
    * ``RUNNING``    — set right before the executor is awaited.
    * ``COMPLETED``  — terminal: executor returned a successful
      :class:`TaskResult`.
    * ``FAILED``     — terminal: executor raised.
    * ``CANCELLED``  — terminal: removed from the queue (before
      ``SCHEDULED``) or cancelled mid-flight via
      :meth:`TaskScheduler.cancel`.

    The MVP does NOT have a ``SKIPPED`` or ``REJECTED`` status —
    those belong to Phase 1.5's awaiting-confirmation flow.
    """

    QUEUED = "QUEUED"
    SCHEDULED = "SCHEDULED"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class TaskOutcome(str, Enum):
    """Subset of :class:`TaskStatus` values that signal «done».

    Used by :class:`TaskScheduler.wait_all` so callers can branch
    on success/failure without inspecting :class:`TaskStatus`.
    """

    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class TaskResult:
    """What the executor returns on success.

    Carries the executor's payload (typically the structured
    response the LLM tool decorator would have produced) plus an
    optional human-readable ``message`` for logging / LLM
    feedback. ``None`` payload means "tool produced no
    response" — e.g. ``stop_music``.
    """

    payload: Any = None
    message: str = ""
    error: Optional[str] = None

    @property
    def is_success(self) -> bool:
        """True when the task completed without an ``error`` set."""
        return self.error is None


@dataclass
class SchedulerTask:
    """A single unit of work submitted to the scheduler.

    Attributes:
        task_id: Stable identifier (uuid4 hex). Generated by
            :meth:`TaskScheduler.submit` when not supplied.
        tool: Tool name (e.g. ``"speak_text"``). Echoed in logs and
            in the §7 ``[CHANNELS]`` feedback block — Phase 3 will
            surface it through the LLM system prompt.
        channel: Which :class:`ChannelKind` owns this task.
        executor: Coroutine invoked when the task reaches the head
            of its channel. It must return a :class:`TaskResult`;
            raising ends the task with :attr:`TaskStatus.FAILED`.
        args: Original argument dict — kept verbatim for the
            executor and for log introspection.
        status: Current :class:`TaskStatus`. Mutated under the
            scheduler's lock.
        created_at: ``time.monotonic()`` at submission.
        started_at: ``time.monotonic()`` at executor entry, or
            ``None`` until then.
        finished_at: ``time.monotonic()`` at terminal transition.
        result: :class:`TaskResult` returned by the executor, or
            ``None`` until then.
        error: Exception text captured on :attr:`TaskStatus.FAILED`,
            or ``None`` until then.
        group_id: S2 (scheduler-segments-merge, issue #968) —
            identifies the multi-segment task this task belongs to
            (e.g. all ``speak_text`` calls of one song). ``None``
            means an ungrouped, standalone task — the default, so
            existing callers are unaffected.
        seg_idx: Position within :attr:`group_id`, 0-based. ``None``
            when :attr:`group_id` is ``None``.
    """

    task_id: str
    tool: str
    channel: ChannelKind
    executor: Callable[["SchedulerTask"], Awaitable[TaskResult]]
    args: Dict[str, Any] = field(default_factory=dict)
    status: TaskStatus = TaskStatus.QUEUED
    created_at: float = field(default_factory=time.monotonic)
    started_at: Optional[float] = None
    finished_at: Optional[float] = None
    result: Optional[TaskResult] = None
    error: Optional[str] = None
    group_id: Optional[str] = None
    seg_idx: Optional[int] = None

    def snapshot(self) -> Dict[str, Any]:
        """Return a frozen, log-friendly view of the task.

        The executor callable is NOT included — coroutine objects
        are not JSON-serializable. Callers that need the body
        should keep a separate handle.
        """
        return {
            "task_id": self.task_id,
            "tool": self.tool,
            "channel": self.channel.value,
            "status": self.status.value,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "args_keys": sorted(self.args.keys()),
            "error": self.error,
            "group_id": self.group_id,
            "seg_idx": self.seg_idx,
        }


@runtime_checkable
class TaskExecutor(Protocol):
    """Protocol describing a single executor callable.

    Concrete implementations are usually inline lambdas created by
    the LLM-tool adapter — e.g. ``lambda task: _publish_tts(task)``.
    The protocol keeps the surface explicit for unit tests that
    build fakes.
    """

    def __call__(self, task: SchedulerTask) -> Awaitable[TaskResult]:
        ...


# ---------------------------------------------------------------------------
# Channel
# ---------------------------------------------------------------------------


@dataclass
class ChannelStatus:
    """Snapshot of a single channel — used by the §7 ``[CHANNELS]`` block.

    The MVP exposes the three numbers LLM feedback needs in Phase 3
    (queue depth, current task id, ETA). ETA is ``None`` because
    the MVP has no :class:`SegmentEstimator` yet — Phase 3 wires
    it up via :meth:`TaskScheduler.set_eta_provider`.
    """

    kind: ChannelKind
    queue_depth: int
    current_task_id: Optional[str]
    current_tool: Optional[str]
    eta_s: Optional[float] = None


class _Channel:
    """A single FIFO channel.

    Each channel owns

    * an ``asyncio.Queue`` (dequeuing is driven by
      :meth:`_pump`),
    * an ``asyncio.Lock`` that serialises executor awaits, and
    * a small bookkeeping table that mirrors the queue head so
      :meth:`TaskScheduler.channel_status` does not have to peek
      at the queue (which would race with ``_pump``).
    """

    def __init__(
        self,
        kind: ChannelKind,
        *,
        loop: asyncio.AbstractEventLoop,
        on_event: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> None:
        self.kind = kind
        self._queue: "asyncio.Queue[SchedulerTask]" = asyncio.Queue()
        self._lock = asyncio.Lock()
        self._current_task_id: Optional[str] = None
        self._current_tool: Optional[str] = None
        self._pump_task: Optional[asyncio.Task[None]] = None
        # W7c observer — forwarded from :class:`TaskScheduler`.
        self._on_event = on_event
        # Loop is bound so the lock/queue are constructed on the
        # right event loop. ``loop`` is unused at runtime but kept
        # to fail fast when the scheduler is mis-initialised from
        # inside a non-asyncio thread.
        self._loop = loop

    def _emit(self, event: str, task: SchedulerTask, **extra: Any) -> None:
        if self._on_event is None:
            return
        payload: Dict[str, Any] = {
            "task_id": task.task_id,
            "tool": task.tool,
            "channel": self.kind.value,
            "status": task.status.value,
        }
        payload.update(extra)
        try:
            self._on_event(event, payload)
        except Exception:  # noqa: BLE001 — observer must not break the pump
            _LOG.exception("scheduler: channel on_event(%s) failed", event)

    def submit(self, task: SchedulerTask) -> None:
        """Enqueue *task* on this channel (sync, called from :meth:`TaskScheduler.submit`)."""
        self._queue.put_nowait(task)

    def start(self) -> None:
        """Spawn the background pump task. Idempotent."""
        if self._pump_task is None or self._pump_task.done():
            self._pump_task = asyncio.create_task(
                self._pump(), name=f"scheduler-{self.kind.value}-pump"
            )

    async def drain(self) -> None:
        """Wait until the channel has finished every queued/current task.

        Used by tests and by :meth:`TaskScheduler.wait_all` so the
        caller can sync on the side-effects settling.
        """
        if self._pump_task is None:
            return
        await self._pump_task

    def status(self) -> ChannelStatus:
        """Snapshot view (sync, lock-free reads)."""
        return ChannelStatus(
            kind=self.kind,
            queue_depth=self._queue.qsize(),
            current_task_id=self._current_task_id,
            current_tool=self._current_tool,
        )

    # ----- internals ----------------------------------------------------

    async def _pump(self) -> None:
        """Drain the queue forever; one executor at a time per channel."""
        while True:
            task: SchedulerTask = await self._queue.get()
            try:
                # Lock guarantees that two tasks on the SAME channel
                # never run concurrently. Different channels are
                # independent — Phase 3 introduces priority that
                # spans channels.
                async with self._lock:
                    self._current_task_id = task.task_id
                    self._current_tool = task.tool
                    task.status = TaskStatus.SCHEDULED
                    task.status = TaskStatus.RUNNING
                    task.started_at = time.monotonic()
                    self._emit("task.started", task)
                    try:
                        task.result = await task.executor(task)
                        task.status = TaskStatus.COMPLETED
                        self._emit("task.completed", task)
                    except asyncio.CancelledError:
                        task.status = TaskStatus.CANCELLED
                        task.error = "cancelled"
                        self._emit("task.cancelled", task, reason="cancelled")
                        raise
                    except Exception as exc:  # noqa: BLE001 — MVP guards the boundary
                        task.status = TaskStatus.FAILED
                        task.error = f"{type(exc).__name__}: {exc}"
                        self._emit("task.failed", task, error=task.error)
                        _LOG.exception(
                            "scheduler: task %s (%s) failed",
                            task.task_id, task.tool,
                        )
                    finally:
                        task.finished_at = time.monotonic()
                        self._current_task_id = None
                        self._current_tool = None
            finally:
                self._queue.task_done()

    def remove(self, task_id: str) -> Optional[SchedulerTask]:
        """Remove *task_id* from the queue if it has not started yet.

        The asyncio.Queue API does not support O(1) removal, so we
        rebuild the queue from a snapshot. Cheap at MVP scale (≤
        a few dozen tasks); Phase 3 will swap in a deque.
        """
        kept: list[SchedulerTask] = []
        removed: Optional[SchedulerTask] = None
        while True:
            try:
                t = self._queue.get_nowait()
            except asyncio.QueueEmpty:
                break
            if t.task_id == task_id and removed is None:
                removed = t
                t.status = TaskStatus.CANCELLED
                t.error = "cancelled before start"
                t.finished_at = time.monotonic()
                self._emit(
                    "task.cancelled", t, reason="cancelled before start"
                )
                # We swallowed one ``put``; mark it done so
                # ``join`` accounting stays sane.
                self._queue.task_done()
                continue
            kept.append(t)
        for t in kept:
            self._queue.put_nowait(t)
            # 🔴 FIX (S3.2, scheduler-segments-merge) — put_nowait() here
            # is a *re*-queue of an item whose original put() already
            # counted toward asyncio.Queue's unfinished-task total.
            # Without this compensating task_done(), every kept item
            # permanently inflates that counter by one and wait_all()
            # (Queue.join()) never returns once a snapshot rebuild has
            # touched a queue with 2+ items. _pump's own task_done() at
            # completion still fires later and brings the count back to
            # zero — this just cancels the redundant +1 from our re-put.
            self._queue.task_done()
        return removed

    def replace_args(self, task_id: str, args: Dict[str, Any]) -> Optional[SchedulerTask]:
        """Rewrite the ``args`` of *task_id* while it is still queued.

        S3.2 (scheduler-segments-merge, §2.3 invariant / R2) — same
        rebuild-from-snapshot technique as :meth:`remove`, which is
        what makes it race-safe: ``get_nowait``/``put_nowait`` run with
        no ``await`` in between, so nothing can interleave mid-rebuild.
        If ``_pump`` already dequeued *task_id* (about to go RUNNING or
        already terminal), it simply will not be found in this
        snapshot — returns ``None`` rather than mutating a task that is
        no longer safely ours to touch.
        """
        kept: list[SchedulerTask] = []
        found: Optional[SchedulerTask] = None
        while True:
            try:
                t = self._queue.get_nowait()
            except asyncio.QueueEmpty:
                break
            if t.task_id == task_id and found is None:
                t.args = dict(args)
                found = t
            kept.append(t)
        for t in kept:
            self._queue.put_nowait(t)
            # Same Queue.join() accounting fix as remove() above.
            self._queue.task_done()
        if found is not None:
            self._emit("task.updated", found)
        return found


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------


class TaskSubmitError(ValueError):
    """Raised when :meth:`TaskScheduler.submit` rejects a task."""


class TaskNotFoundError(KeyError):
    """Raised when ``cancel`` / ``wait`` references an unknown task id."""


class ChannelBusyError(RuntimeError):
    """Raised when a single-task channel already has a current task."""


# ---------------------------------------------------------------------------
# S3.1/S3.2 — MERGE delta application result (scheduler-segments-merge)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class UpdateOpOutcome:
    """What happened to one :class:`~rob_box_voice.scheduler.delta.DeltaOp`.

    ``applied=False`` is the normal, expected outcome for an op that
    targets a RUNNING or already-terminal segment — the §2.3 invariant
    means "ignored", not "error". ``reason`` explains why for logging.

    ``frozen`` (S9.1, §6.5): ``True`` when the op *was* applied but the
    target segment was PENDING_FROZEN (past the group's pre-gen
    boundary) at the time — the op still succeeded (FROZEN, unlike
    RUNNING, is not a hard block), but the caller should treat this as
    a signal to cancel/regenerate that segment's speculative pre-gen.
    Always ``False`` for ``applied=False`` outcomes and for ``append``.
    """

    op: DeltaOp
    applied: bool
    task_id: Optional[str] = None
    reason: str = ""
    frozen: bool = False


@dataclass(frozen=True)
class UpdateReport:
    """Result of :meth:`TaskScheduler.update` — one outcome per op, in order."""

    group_id: str
    outcomes: tuple[UpdateOpOutcome, ...]


# ---------------------------------------------------------------------------
# S10 — llm_continue_hook (scheduler-segments-merge, issue #968, §4.5)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LlmContinueContext:
    """Snapshot handed to the §4.5 ``llm_continue_hook`` when it fires.

    Built by :meth:`TaskScheduler._maybe_trigger_continue` once all
    three §4.5 conditions hold. Carries exactly what the hook needs to
    build a ``[SEGMENT PLAN]``/``[PENDING EVENTS]``-style prompt and,
    eventually, call :meth:`TaskScheduler.update` with the resulting
    ``task_delta`` — the scheduler itself never builds that prompt or
    calls an LLM; that is the hook's job (see :class:`LlmContinueHook`).
    """

    group_id: str
    channel: ChannelKind
    pending_segments: tuple[SchedulerTask, ...]
    events: tuple[EventEnvelope, ...]


#: §4.5 extension point. The scheduler calls ``hook(context)`` as a
#: fire-and-forget task on its own loop when the auto-trigger fires; it
#: does **not** know or care what the hook does with it (per revision
#: v5 / §4.7, the intended implementation calls the MAIN LLM — never a
#: second/light model — but that decision lives entirely on the caller
#: side, e.g. ``dialogue_node.py``, not here).
LlmContinueHook = Callable[[LlmContinueContext], Awaitable[None]]


# ---------------------------------------------------------------------------
# Scheduler façade
# ---------------------------------------------------------------------------


class TaskScheduler:
    """Phase 1 MVP task scheduler.

    Lifecycle:

    1. Construct on the asyncio loop that will own the channels.
    2. Call :meth:`start` once (idempotent) to spawn the channel
       pumps.
    3. :meth:`submit` tasks; await :meth:`wait_all` if you need to
       synchronise on completion (tests, end-of-dialogue cleanup).
    4. :meth:`shutdown` to drain and stop the pumps (rarely needed
       in production — the dialogue node owns the loop).

    Threading:

    * :meth:`start`, :meth:`shutdown`, :meth:`submit`, :meth:`cancel`
      are **not** coroutines — they are safe to call from a ROS2
      callback running on a worker thread, provided the loop that
      owns the channels is reachable via
      ``asyncio.run_coroutine_threadsafe``.
    * :meth:`wait_all`, :meth:`wait`, :meth:`channel_status` are
      synchronous and lock-free — they only read state mutated
      under the internal ``threading.Lock``.

    Example (test)::

        async def _run() -> None:
            sched = TaskScheduler()
            sched.start()
            task = sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_fake_speak, args={"text": "hello"},
            ))
            await sched.wait_all()
            assert task.status is TaskStatus.COMPLETED
            sched.shutdown()
    """

    #: Channels created by default — :class:`ChannelKind.VOICE`,
    #: :class:`ChannelKind.MUSIC`, :class:`ChannelKind.ANIM`.
    DEFAULT_CHANNELS: tuple[ChannelKind, ...] = (
        ChannelKind.VOICE,
        ChannelKind.MUSIC,
        ChannelKind.ANIM,
    )

    def __init__(
        self,
        *,
        channels: tuple[ChannelKind, ...] = DEFAULT_CHANNELS,
        loop: Optional[asyncio.AbstractEventLoop] = None,
        on_event: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> None:
        if not channels:
            raise TaskSubmitError("TaskScheduler requires at least one channel")
        try:
            running_loop = asyncio.get_running_loop()
        except RuntimeError:
            running_loop = None
        self._loop = loop or running_loop
        if self._loop is None:
            raise RuntimeError(
                "TaskScheduler must be constructed inside an asyncio loop "
                "or be given an explicit loop= argument"
            )
        self._lock = threading.Lock()
        self._tasks: Dict[str, SchedulerTask] = {}
        # S2.2 (scheduler-segments-merge) — group_id → ordered task_ids.
        # Populated in submit(); cleared lazily in segments() once every
        # segment of the group has reached a terminal status (§2.2).
        self._groups: Dict[str, list[str]] = {}
        # S9.1 (scheduler-segments-merge, §6.5) — group_id → boundary_idx,
        # mirroring the latest PreGenPlan.boundary_idx the caller computed
        # for that group. See set_group_boundary()/is_frozen().
        self._boundaries: Dict[str, int] = {}
        # S9.1 — optional callback fired from update() when a FROZEN
        # segment is edited (see set_frozen_touch_hook()).
        self._on_frozen_touch: Optional[Callable[[SchedulerTask], None]] = None
        # W7c (issue #968): optional lifecycle callback — receives
        # ``(event, payload)`` for task.created / task.started /
        # task.completed / task.failed / task.cancelled. The dialogue
        # node uses it to publish on /harness/task_events and to feed
        # the [ACTIVE TASKS] LLM-context block.
        self._on_event = on_event
        # S10 (scheduler-segments-merge, issue #968, §4.5) — auto-trigger
        # state. ``_pending_events`` holds unapplied EventEnvelopes per
        # group_id (condition 2); ``_channel_running`` is a tri-state
        # per channel: ``None`` = never started a task yet, ``True`` =
        # currently RUNNING, ``False`` = previously RUNNING and now
        # idle. The tri-state (not a plain bool) is what keeps the
        # trigger from firing on a channel that simply hasn't started
        # anything yet — only a real ACTIVE→idle transition counts as
        # "voice: speaking → silence после ACTIVE".
        self._pending_events: Dict[str, list[EventEnvelope]] = {}
        self._channel_running: Dict[ChannelKind, Optional[bool]] = {
            kind: None for kind in channels
        }
        self._llm_continue_hook: Optional[LlmContinueHook] = None
        self._channels: Dict[ChannelKind, _Channel] = {
            kind: _Channel(kind, loop=self._loop, on_event=self._dispatch_channel_event)
            for kind in channels
        }
        self._started: bool = False
        self._shutdown: bool = False
        # Optional Phase 3 hook — left ``None`` for the MVP. When
        # set, :meth:`channel_status` reads ETA from the provider.
        self._eta_provider: Optional[Callable[[SchedulerTask], Optional[float]]] = None

    def _emit(self, event: str, payload: Dict[str, Any]) -> None:
        """Dispatch a lifecycle event to the optional ``on_event`` callback."""
        if self._on_event is None:
            return
        try:
            self._on_event(event, payload)
        except Exception:  # noqa: BLE001 — observer must not break the scheduler
            _LOG.exception("scheduler: on_event(%s) callback failed", event)

    # ----- S10: llm_continue_hook auto-trigger (§4.5) --------------------

    _CHANNEL_LIFECYCLE_EVENTS = frozenset(
        {"task.started", "task.completed", "task.failed", "task.cancelled"}
    )

    def _dispatch_channel_event(self, event: str, payload: Dict[str, Any]) -> None:
        """``_Channel``'s ``on_event`` — updates §4.5 state, then forwards.

        Runs synchronously, in-line with ``_Channel._pump``'s own
        processing of the just-finished/just-started task, which is
        exactly what makes the auto-trigger race-free: by the time this
        function returns, nothing else on this event loop has had a
        chance to dequeue the *next* segment out from under
        :meth:`segments`'s snapshot (single-threaded asyncio — no
        ``await`` happens in this call chain).
        """
        if event in self._CHANNEL_LIFECYCLE_EVENTS:
            channel_value = payload.get("channel")
            try:
                kind = ChannelKind(channel_value)
            except ValueError:
                kind = None
            if kind is not None:
                self._channel_running[kind] = event == "task.started"
                task_id = payload.get("task_id")
                task = self.get_task(task_id) if task_id else None
                if task is not None and task.group_id is not None:
                    self._maybe_trigger_continue(task.group_id)
        if self._on_event is not None:
            try:
                self._on_event(event, payload)
            except Exception:  # noqa: BLE001 — observer must not break the pump
                _LOG.exception("scheduler: on_event(%s) callback failed", event)

    def _channel_needs_continuation(self, kind: ChannelKind) -> bool:
        """§4.5 condition (3) — "канал выглядит требующим продолжения".

        * ``VOICE`` — the ACTIVE segment just ended (state flipped
          ``True`` → ``False``). ``None`` (nothing has run yet on this
          channel) deliberately does NOT count — otherwise a MERGE
          event registered before the group's first segment even
          starts would fire immediately, which is not what §4.5 means
          by "speaking → silence после ACTIVE".
        * ``MUSIC``/``ANIM`` — §4.5's music branch fires WHILE the
          segment is still playing (see the "комар+енот" §4.6
          sequence: MERGE lands mid-verse-1). The full elapsed/eta
          refinement ("_last_segment_user_initiated=false AND
          elapsed > 0.5×eta") needs a per-task "user initiated" flag
          and the Phase-3 :class:`SegmentEstimator` wired through
          :meth:`set_eta_provider` — neither exists on
          :class:`SchedulerTask` yet, so S10 deliberately ships the
          simpler "channel is currently RUNNING" heuristic and leaves
          the eta-ratio refinement as a follow-up (see final report).
        """
        state = self._channel_running.get(kind)
        if kind is ChannelKind.VOICE:
            return state is False
        return state is True

    def _maybe_trigger_continue(self, group_id: str) -> None:
        """Re-evaluate the §4.5 three-condition auto-trigger for *group_id*.

        Called after every VOICE/MUSIC/ANIM lifecycle transition and
        every :meth:`notify_event` call. Cheap no-op when the hook is
        not installed or any condition is unmet — see
        :meth:`_channel_needs_continuation` for condition (3) and the
        module docstring / §4.5 for the full three-condition contract.
        """
        hook = self._llm_continue_hook
        if hook is None:
            return
        events = self._pending_events.get(group_id) or []
        unapplied = tuple(e for e in events if e.priority != _LOW_PRIORITY)
        if not unapplied:
            return  # condition (2) unmet
        pending_segments = tuple(
            s for s in self.segments(group_id)
            if s.status in (TaskStatus.QUEUED, TaskStatus.SCHEDULED)
        )
        if not pending_segments:
            return  # condition (1) unmet
        channel_kind = pending_segments[0].channel
        if not self._channel_needs_continuation(channel_kind):
            return  # condition (3) unmet
        # All three hold — consume the events now so a second lifecycle
        # transition (e.g. the next segment's task.started, fired right
        # after this call returns) cannot fan the same events out twice.
        self._pending_events.pop(group_id, None)
        context = LlmContinueContext(
            group_id=group_id,
            channel=channel_kind,
            pending_segments=pending_segments,
            events=unapplied,
        )
        self._fire_llm_continue_hook(hook, context)

    def _fire_llm_continue_hook(self, hook: LlmContinueHook, context: LlmContinueContext) -> None:
        """Schedule *hook* as a fire-and-forget task on the scheduler's loop.

        The scheduler never awaits the hook itself — §4.5 is explicit
        that the scheduler "self-drives" this turn without blocking its
        own channels on the LLM round-trip. A failing hook is logged,
        not raised, matching every other observer boundary in this
        module (:meth:`_emit`, ``_Channel._emit``).
        """

        async def _run() -> None:
            try:
                await hook(context)
            except Exception:  # noqa: BLE001 — hook boundary must not break the scheduler
                _LOG.exception(
                    "scheduler: llm_continue_hook failed for group %s", context.group_id
                )

        self._loop.create_task(_run(), name=f"scheduler-llm-continue-{context.group_id}")

    def notify_event(self, group_id: str, event: EventEnvelope) -> None:
        """Register *event* as an unapplied §4.5 continuation signal.

        Called by the quick_decide/EventBus integration layer (outside
        this module) whenever a decision lands that could need a
        follow-up ``task_delta`` — MERGE, ``battery_critical``, etc. —
        but no fresh user turn is going to carry it. Immediately
        re-evaluates the auto-trigger for *group_id*; if the other two
        §4.5 conditions already hold, this call is what fires the
        installed :meth:`set_llm_continue_hook` hook. A *group_id* the
        scheduler has never seen (or has already fully drained) is a
        harmless no-op — :meth:`segments` returns ``[]`` for it, so
        condition (1) simply stays unmet.
        """
        self._pending_events.setdefault(group_id, []).append(event)
        self._maybe_trigger_continue(group_id)

    def set_llm_continue_hook(self, hook: Optional[LlmContinueHook]) -> None:
        """Install (or clear, with ``None``) the §4.5 auto-trigger hook.

        Mirrors :meth:`set_eta_provider`'s "Phase 3 hook" shape — the
        scheduler exposes a plain extension point and stays ignorant of
        what the hook does. Per revision v5 (§4.7, "Одна LLM, без
        уровня 2"), the intended production hook calls the MAIN LLM
        turn, not a second/light model; that decision is entirely the
        caller's (``dialogue_node.py``/MCP integration), not enforced
        here.
        """
        self._llm_continue_hook = hook

    # ----- lifecycle -----------------------------------------------------

    def start(self) -> None:
        """Spawn the per-channel pump tasks. Idempotent."""
        with self._lock:
            if self._started:
                return
            self._started = True
        for channel in self._channels.values():
            channel.start()

    def shutdown(self) -> None:
        """Cancel pump tasks. Does NOT cancel in-flight executor coroutines.

        The MVP is designed to live as long as the dialogue node;
        calling :meth:`shutdown` from a test is the typical use.
        Operators that need a hard stop should ``await`` the pump
        tasks explicitly (exposed via :attr:`_channels`).
        """
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
        for channel in self._channels.values():
            pump = channel._pump_task  # noqa: SLF001 — test-only escape hatch
            if pump is not None and not pump.done():
                pump.cancel()

    # ----- public surface ------------------------------------------------

    def submit(self, task: SchedulerTask) -> SchedulerTask:
        """Enqueue *task* on its channel.

        The task's :attr:`~SchedulerTask.task_id` is rewritten to a
        uuid4 hex if empty. The returned object is the live
        instance the scheduler stores internally — keep a
        reference if you need to call :meth:`wait`.

        Raises:
            TaskSubmitError: If the channel is unknown, the
                scheduler is shut down, or the executor is missing.
        """
        if task.channel not in self._channels:
            raise TaskSubmitError(
                f"unknown channel {task.channel!r}; "
                f"valid channels: {[c.value for c in self._channels]}"
            )
        if task.executor is None:
            raise TaskSubmitError("task.executor is required")
        if not callable(task.executor):
            raise TaskSubmitError("task.executor must be callable")
        if self._shutdown:
            raise TaskSubmitError("scheduler is shut down")
        if not task.task_id:
            task.task_id = uuid.uuid4().hex
        with self._lock:
            self._tasks[task.task_id] = task
            if task.group_id is not None:
                self._groups.setdefault(task.group_id, []).append(task.task_id)
        channel = self._channels[task.channel]
        # ``submit`` runs on the loop that owns the channel's
        # queue; it is a sync helper (``put_nowait``).
        channel.submit(task)
        self._emit(
            "task.created",
            {
                "task_id": task.task_id,
                "tool": task.tool,
                "channel": task.channel.value,
                "status": task.status.value,
                "args_keys": sorted(task.args.keys()),
                # Сегментные координаты задачи. Без них по
                # ``/harness/task_events`` нельзя посчитать, приезжает
                # выступление одним батчем или по куску за итерацию
                # тул-цикла: ``dialog_core`` зовёт ``begin_group()`` на
                # КАЖДЫЙ батч, поэтому число разных ``group_id`` за тёрн
                # равно числу итераций, а ``seg_idx`` внутри группы —
                # размеру батча. Оба поля у задачи были всегда, в
                # событие не попадали. ``None`` — задача вне группы
                # (bypass-тул или submit без ``begin_group``).
                "group_id": task.group_id,
                "seg_idx": task.seg_idx,
            },
        )
        _LOG.debug(
            "scheduler: enqueued task %s (%s) on %s",
            task.task_id, task.tool, task.channel.value,
        )
        return task

    def cancel(self, task_id: str) -> bool:
        """Cancel *task_id*.

        Returns ``True`` if the task was found and either
        cancelled before start or marked for cancellation after
        start. The MVP does not interrupt a running executor
        coroutine mid-flight — that lands in Phase 2 with
        :class:`SchedulerEventBus`.

        Tasks already in :attr:`TaskStatus.COMPLETED` /
        :attr:`TaskStatus.FAILED` are reported as not found
        because their lifecycle is closed.
        """
        with self._lock:
            task = self._tasks.get(task_id)
        if task is None:
            return False
        if task.status in (TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED):
            return False
        channel = self._channels[task.channel]
        removed = channel.remove(task_id)
        if removed is not None:
            _LOG.info("scheduler: task %s cancelled before start", task_id)
            return True
        # The task is already past SCHEDULED — we cannot preempt
        # it in the MVP. Surface a best-effort log line and leave
        # the executor to finish naturally.
        _LOG.info(
            "scheduler: task %s already running; "
            "MVP cannot preempt (Phase 2 will add EventBus cancel)",
            task_id,
        )
        return False

    def get_task(self, task_id: str) -> Optional[SchedulerTask]:
        """Return the live task record for *task_id*, or ``None``."""
        with self._lock:
            return self._tasks.get(task_id)

    _TERMINAL_STATUSES = (TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED)

    def segments(self, group_id: str) -> list[SchedulerTask]:
        """Return every segment of *group_id*, ordered by :attr:`SchedulerTask.seg_idx`.

        S2.2 (scheduler-segments-merge, issue #968). Unknown or already-
        cleared groups return ``[]`` — this is a read-only query used by
        the future ``[SEGMENT PLAN]`` context block (S5), not an error
        path like :meth:`TaskScheduler.update` (S3).

        Completed/failed/cancelled segments stay in the returned list
        (so the LLM/log can see "verse 1 already played") until every
        segment in the group has reached a terminal status — at that
        point the group is cleared from the registry as a side effect
        of this call.
        """
        with self._lock:
            task_ids = list(self._groups.get(group_id, ()))
            tasks = [self._tasks[tid] for tid in task_ids if tid in self._tasks]
            if tasks and all(t.status in self._TERMINAL_STATUSES for t in tasks):
                self._groups.pop(group_id, None)
        tasks.sort(key=lambda t: (t.seg_idx is None, t.seg_idx))
        return tasks

    def set_group_boundary(self, group_id: str, boundary_idx: Optional[int]) -> None:
        """S9.1 (§6.5, scheduler-segments-merge) — record the PENDING_FROZEN /
        PENDING_LIVE split for *group_id*.

        ``boundary_idx`` mirrors :attr:`~.pre_gen.PreGenPlan.boundary_idx` —
        the caller (whoever owns the ``SpeculativePreGenerator`` /
        ``SpeculativeStepExecutor`` bridge, §6.5) is expected to call this
        every time it recomputes ``build_plan`` for the group: when a
        segment starts, after ``LLMEstimator.record`` recalibrates, and
        right before :meth:`update`. This module deliberately does not
        import ``pre_gen``/``speculative_executor`` itself — they already
        import *this* module, so the dependency only runs one way — it
        just stores whatever value the caller last computed.

        :meth:`is_frozen` reads the stored boundary fresh on every call;
        there is no separate cache to go stale, so "recompute at the
        three triggers" simply means "call this setter again there".

        ``None`` clears the boundary — every PENDING segment of the group
        is then LIVE (matches pre-S9 behaviour, and a ``PreGenPlan`` with
        no reachable boundary, e.g. an enormous LLM ETA).
        """
        with self._lock:
            if boundary_idx is None:
                self._boundaries.pop(group_id, None)
            else:
                self._boundaries[group_id] = boundary_idx

    def is_frozen(self, task: SchedulerTask) -> bool:
        """True when *task* is PENDING_FROZEN (§6.5).

        FROZEN is deliberately **not** a :class:`TaskStatus` value — it
        is a derived property of a PENDING (``QUEUED``/``SCHEDULED``)
        segment: still queued, but at or past the pre-gen boundary index
        recorded via :meth:`set_group_boundary` for its group. RUNNING
        and terminal segments are never FROZEN — RUNNING is already
        covered by the harder §2.3 invariant enforced in :meth:`update`.
        """
        if task.status not in (TaskStatus.QUEUED, TaskStatus.SCHEDULED):
            return False
        if task.group_id is None or task.seg_idx is None:
            return False
        with self._lock:
            boundary = self._boundaries.get(task.group_id)
        if boundary is None:
            return False
        return task.seg_idx < boundary

    def _notify_frozen_touch(self, task: SchedulerTask) -> None:
        """S9.1: fire the optional frozen-touch hook (see
        :meth:`set_frozen_touch_hook`).

        The hook lets the integration layer cancel that segment's
        in-flight speculative pre-gen (with
        ``speculative_executor.CANCEL_REASON_MERGE_TOUCHED_FROZEN``) and
        let it regenerate — this module only notifies, it does not own
        the pre-gen cache (§6.5 boundary rule, see
        ``speculative_executor.py``'s module docstring).
        """
        if self._on_frozen_touch is None:
            return
        try:
            self._on_frozen_touch(task)
        except Exception:  # noqa: BLE001 — a hook bug must not corrupt update()
            _LOG.exception(
                "scheduler: on_frozen_touch hook failed for task %s", task.task_id
            )

    def set_frozen_touch_hook(
        self, hook: Optional[Callable[[SchedulerTask], None]],
    ) -> None:
        """Install the S9.1 callback fired when :meth:`update` edits a
        FROZEN segment (§6.5). ``None`` (the MVP default) disables it —
        ``update`` still applies the edit either way; only the
        notification is skipped.
        """
        self._on_frozen_touch = hook

    def update(
        self,
        group_id: str,
        delta: TaskDelta,
        *,
        executor_factory: Optional[Callable[[DeltaOp], "TaskExecutor"]] = None,
    ) -> UpdateReport:
        """Apply *delta* to *group_id*, honouring the §2.3 ACTIVE invariant.

        S3.2 (scheduler-segments-merge, issue #968) — the whole point of
        this plan: a RUNNING segment is **never** rewritten or cancelled.
        ``rewrite``/``replace`` mutate a PENDING (``QUEUED``/``SCHEDULED``)
        segment's args in place, preserving FIFO order; ``drop`` removes a
        PENDING segment (reuses :meth:`_Channel.remove`); ``append`` adds
        a brand new segment to the tail via the normal :meth:`submit`
        path. An op that targets a RUNNING or already-terminal segment is
        *ignored*, not an error — see :class:`UpdateOpOutcome`.

        Race safety (R2): the RUNNING/terminal check below is only an
        optimisation. The actual safety comes from
        :meth:`_Channel.replace_args` / :meth:`_Channel.remove`, which
        operate on the live queue snapshot — if ``_pump`` already
        dequeued the segment (even if ``status`` has not flipped to
        SCHEDULED yet), they simply will not find it.

        Raises:
            TaskNotFoundError: *group_id* has no tracked segments (never
                submitted, or fully cleared by a prior :meth:`segments`
                call after every segment went terminal).
            TaskSubmitError: *delta* contains an ``append`` op but no
                *executor_factory* was given — the scheduler has no way
                to build the new segment's executor on its own (that
                requires tool-specific knowledge that lives in
                :mod:`rob_box_voice.scheduler.tool_executor`, not here).
        """
        with self._lock:
            task_ids = list(self._groups.get(group_id, ()))
            by_seg_idx: Dict[int, SchedulerTask] = {
                t.seg_idx: t
                for t in (self._tasks.get(tid) for tid in task_ids)
                if t is not None and t.seg_idx is not None
            }
            sample_task = next(
                (self._tasks[tid] for tid in task_ids if tid in self._tasks), None
            )
        if sample_task is None:
            raise TaskNotFoundError(group_id)
        if delta.group_id != group_id:
            raise ValueError(
                f"delta.group_id={delta.group_id!r} does not match group_id={group_id!r}"
            )
        if executor_factory is None and any(
            op.kind is DeltaOpKind.APPEND for op in delta.ops
        ):
            raise TaskSubmitError(
                "delta contains an append op but no executor_factory was given"
            )

        channel = self._channels[sample_task.channel]
        outcomes: List[UpdateOpOutcome] = []
        for op in delta.ops:
            if op.kind is DeltaOpKind.APPEND:
                next_idx = (max(by_seg_idx) + 1) if by_seg_idx else 0
                new_task = SchedulerTask(
                    task_id="",
                    tool=sample_task.tool,
                    channel=sample_task.channel,
                    executor=executor_factory(op),  # type: ignore[misc]
                    args=dict(op.args or {}),
                    group_id=group_id,
                    seg_idx=next_idx,
                )
                self.submit(new_task)
                by_seg_idx[next_idx] = new_task
                outcomes.append(UpdateOpOutcome(op=op, applied=True, task_id=new_task.task_id))
                continue

            target = by_seg_idx.get(op.seg_idx)
            if target is None:
                outcomes.append(UpdateOpOutcome(op=op, applied=False, reason="segment not found"))
                continue
            if target.status not in (TaskStatus.QUEUED, TaskStatus.SCHEDULED):
                reason = (
                    "segment RUNNING (invariant)"
                    if target.status is TaskStatus.RUNNING
                    else "segment terminal"
                )
                outcomes.append(
                    UpdateOpOutcome(op=op, applied=False, task_id=target.task_id, reason=reason)
                )
                continue

            # S9.1 (§6.5): FROZEN is not a hard block like RUNNING — the
            # edit still goes through — but the caller needs to know so
            # it can cancel/regenerate that segment's speculative pre-gen.
            frozen = self.is_frozen(target)
            if op.kind in (DeltaOpKind.REWRITE, DeltaOpKind.REPLACE):
                mutated = channel.replace_args(target.task_id, dict(op.args or {}))
            else:  # DROP
                mutated = channel.remove(target.task_id)
            applied = mutated is not None
            if applied and frozen:
                self._notify_frozen_touch(target)
            outcomes.append(UpdateOpOutcome(
                op=op, applied=applied, task_id=target.task_id,
                reason="" if applied else "segment started running before update landed",
                frozen=applied and frozen,
            ))
        return UpdateReport(group_id=group_id, outcomes=tuple(outcomes))

    def wait(self, task_id: str, *, timeout: Optional[float] = None) -> SchedulerTask:
        """Block until *task_id* reaches a terminal status.

        Implemented as a polling loop on the task's :attr:`status`
        field rather than a per-task ``asyncio.Event`` so the MVP
        keeps a minimal API surface. Polling cadence is 5 ms —
        fine for human-scale TTS (sub-second to multi-second) but
        Phase 3 will swap in proper events.

        Raises:
            TaskNotFoundError: If *task_id* is not tracked.
            asyncio.TimeoutError: If *timeout* (in seconds) elapses
                before the task terminates.
        """
        task = self.get_task(task_id)
        if task is None:
            raise TaskNotFoundError(task_id)
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if task.status in (
                TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED,
            ):
                return task
            if deadline is not None and time.monotonic() >= deadline:
                raise asyncio.TimeoutError(f"task {task_id} did not finish within {timeout}s")
            time.sleep(0.005)

    async def wait_all(self, timeout: Optional[float] = None) -> None:
        """Await every channel's queue and current task.

        Returns when every channel has finished every task that
        was queued at the moment of the call. New submissions
        after :meth:`wait_all` started are ignored — this matches
        the typical use case (end-of-dialogue cleanup) where the
        caller wants to know «everything that was in flight is
        done». The pump tasks themselves keep running; they are
        cancelled only by :meth:`shutdown`.

        Raises:
            asyncio.TimeoutError: If *timeout* (in seconds)
                expires before every channel settles.
        """
        # Capture the queues we want to join BEFORE awaiting — a
        # late submit() that lands during wait_all() must NOT be
        # included (caller has not enqueued it yet).
        queues = list(self._channels.values())
        if timeout is None:
            await asyncio.gather(*(ch._queue.join() for ch in queues))  # noqa: SLF001
        else:
            await asyncio.wait_for(
                asyncio.gather(*(ch._queue.join() for ch in queues)),  # noqa: SLF001
                timeout=timeout,
            )

    def channel_status(self, kind: ChannelKind) -> ChannelStatus:
        """Return the §7 ``[CHANNELS]`` snapshot for *kind*."""
        channel = self._channels.get(kind)
        if channel is None:
            raise TaskSubmitError(f"unknown channel {kind!r}")
        status = channel.status()
        if self._eta_provider is not None:
            with self._lock:
                current = self._tasks.get(status.current_task_id) if status.current_task_id else None
            if current is not None:
                status.eta_s = self._eta_provider(current)
        return status

    def all_statuses(self) -> Mapping[ChannelKind, ChannelStatus]:
        """Return a snapshot of every channel's status."""
        return {kind: ch.status() for kind, ch in self._channels.items()}

    async def wait_until_idle(self, kind: ChannelKind) -> None:
        """Wait until channel *kind* drains (empty queue, no current task).

        W7b (issue #968): destructive tools such as ``stop_music`` that
        arrive while the voice channel still has queued/current TTS wait
        here, so the side effect fires only after the speech finishes —
        ``stop_music`` can no longer outrun the TTS chunk it was meant
        to follow (e2e v36).

        Polls at 20 ms — human-scale TTS is seconds long, so the
        polling overhead is negligible at MVP scale (Phase 3 will swap
        in proper events).
        """
        channel = self._channels.get(kind)
        if channel is None:
            raise TaskSubmitError(f"unknown channel {kind!r}")
        while True:
            status = channel.status()
            if status.queue_depth == 0 and status.current_task_id is None:
                return
            await asyncio.sleep(0.02)

    # ----- Phase 3 hooks (no-op in MVP) ---------------------------------

    def set_eta_provider(
        self,
        provider: Optional[Callable[[SchedulerTask], Optional[float]]],
    ) -> None:
        """Install the Phase 3 :class:`SegmentEstimator` callback.

        The MVP leaves ETA as ``None`` until §11.3 ships. The hook
        is exposed now so callers can wire the estimator later
        without touching the scheduler API.
        """
        self._eta_provider = provider

    # ----- diagnostics ---------------------------------------------------

    def tasks_snapshot(self) -> Dict[str, Dict[str, Any]]:
        """Return a log-friendly view of every tracked task.

        Used by tests to assert ordering without poking at the
        internals.
        """
        with self._lock:
            return {tid: t.snapshot() for tid, t in self._tasks.items()}