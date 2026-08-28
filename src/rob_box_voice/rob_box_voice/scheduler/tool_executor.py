"""tool_executor.py — W7b SchedulerToolExecutor (issue #968).

Routes LLM tool calls through the :class:`TaskScheduler` so channel
ownership is enforced between the LLM and the ROS side-effect layer:

* ``speak_text``            → VOICE channel (FIFO, strictly sequential)
* ``stop_music``            → MUSIC channel (deferred until VOICE drains)
* ``play_animation``        → ANIM channel
* music starters (``execute_music_code`` / ``set_vibe_preset`` /
  ``load_track``) and everything else (memory / search / get_* /
  estimate_* / nav …)       → executed directly, blocking (bypass)

The key contract (W7_INTEGRATION_PLAN.md §W7b): :meth:`execute` does NOT
await completion for queued tools — it returns ``{"status": "queued",
"task_id": ...}`` immediately so the LLM stays free (fire-and-forget is
preserved for the VOICE / ANIM channels). Music starters BYPASS the
scheduler on purpose (live 19.08 party regression): the LLM must see the
real result («Код выполнен успешно») to keep a DJ set going. Destructive
tools (``stop_music``) are additionally deferred until the VOICE channel
drains, so music cannot be killed while TTS is still speaking (e2e v36).

Safety: the executor is fail-open. Any scheduler misbehaviour
(submit error, unknown channel, loop problems) falls back to direct
execution of the underlying provider — a scheduler bug must never
silence the robot.
"""

from __future__ import annotations

import json
import logging
import uuid
from typing import Any, Callable, Optional

from rob_box_llm.provider import ToolCall, ToolResult

from rob_box_voice.scheduler.task_scheduler import (
    ChannelKind,
    SchedulerTask,
    TaskScheduler,
)

_LOG = logging.getLogger(__name__)

#: Tools that own the VOICE channel (FIFO, strictly sequential).
_VOICE_TOOLS: frozenset[str] = frozenset({"speak_text"})

#: Tools that own the MUSIC channel.
#
# 🔴 FIX (party regression, live 19.08): the music *starters*
# (``execute_music_code`` / ``set_vibe_preset`` / ``load_track``) must run
# BLOCKING so the LLM sees the real result («Код выполнен успешно») instead
# of a fire-and-forget «queued». The queued contract blinded the DJ LLM —
# it never got confirmation that music actually started and kept skipping
# ``execute_music_code`` across transitions. Only ``stop_music`` stays on the
# scheduler, so it can still be deferred until the VOICE channel drains
# (e2e v36: stop_music must not outrun the TTS chunk it follows).
_MUSIC_TOOLS: frozenset[str] = frozenset({"stop_music"})

#: Tools that own the ANIM channel.
_ANIM_TOOLS: frozenset[str] = frozenset({"play_animation"})

#: Destructive tools whose side effect must wait until the VOICE channel
#: has drained (no queued/current TTS). Fixes issue #968 — stop_music used
#: to land on the robot milliseconds after speak_text, killing music
#: mid-phrase.
_DEFERRED_DESTRUCTIVE_TOOLS: frozenset[str] = frozenset({"stop_music"})


def channel_for_tool(tool: str) -> Optional[ChannelKind]:
    """Return the scheduler channel for *tool*, or ``None`` for bypass.

    Tools not in the explicit channel sets (memory / search / get_* /
    estimate_* / navigation …) return ``None`` — they are executed
    directly without queueing, preserving the pre-W7b behaviour.
    """
    if tool in _VOICE_TOOLS:
        return ChannelKind.VOICE
    if tool in _MUSIC_TOOLS:
        return ChannelKind.MUSIC
    if tool in _ANIM_TOOLS:
        return ChannelKind.ANIM
    return None


class SchedulerToolExecutor:
    """Legacy ``discover/execute`` provider wrapper backed by a scheduler.

    Implements the same structural port as
    :class:`rob_box_harness.tools.ToolProvider` (``discover`` /
    ``execute`` / ``aclose``) so it can be dropped into ``DialogCore``
    in place of the plain adapter. The underlying provider is invoked
    verbatim for the actual ROS side effects.
    """

    name = "scheduler"

    def __init__(
        self,
        underlying: Any,
        scheduler: Optional[TaskScheduler] = None,
        *,
        on_event: Optional[Callable[[str, dict[str, Any]], None]] = None,
    ) -> None:
        self._underlying = underlying
        self._scheduler = scheduler
        self._on_event = on_event
        self._scheduler_attempted = False
        # S2.3 (scheduler-segments-merge) — group_id/seg_idx assigned to
        # every channel-routed task submitted while a group is open.
        # None until the first begin_group() call (backward compat:
        # ungrouped tasks keep group_id=None like before this feature).
        self._current_group_id: Optional[str] = None
        self._current_seg_idx: int = 0

    def begin_group(self) -> str:
        """Start a new segment group (issue #968, S2.3).

        Called by ``dialog_core`` right before it processes one LLM
        batch of tool_calls (the same re-ordering point W7a already
        hooks into). Every channel-routed task :meth:`execute` submits
        afterwards gets this call's ``group_id`` and a ``seg_idx``
        counting up from 0, until the next ``begin_group()`` call
        starts a fresh group.
        """
        self._current_group_id = uuid.uuid4().hex
        self._current_seg_idx = 0
        return self._current_group_id

    # ----- port surface --------------------------------------------------

    async def discover(self) -> Any:
        """Delegate discovery to the underlying provider."""
        return await self._underlying.discover()

    async def aclose(self) -> None:
        """Release resources of the underlying provider."""
        return await self._underlying.aclose()

    async def execute(self, call: ToolCall) -> ToolResult:
        """Route *call* through the scheduler, or execute it directly.

        Returns immediately for queued tools (``status=queued``);
        the real side effect runs asynchronously on the scheduler's
        channel pump. Bypass tools return the underlying result.
        """
        channel = channel_for_tool(call.name)
        if channel is None:
            return await self._underlying.execute(call)

        scheduler = self._ensure_scheduler()
        if scheduler is None:
            return await self._underlying.execute(call)

        deferred = call.name in _DEFERRED_DESTRUCTIVE_TOOLS
        group_id = self._current_group_id
        seg_idx: Optional[int] = None
        if group_id is not None:
            seg_idx = self._current_seg_idx
            self._current_seg_idx += 1
        try:
            task = scheduler.submit(
                SchedulerTask(
                    task_id="",
                    tool=call.name,
                    channel=channel,
                    executor=self._make_executor(call, deferred),
                    args=dict(call.arguments or {}),
                    group_id=group_id,
                    seg_idx=seg_idx,
                )
            )
        except Exception as exc:  # noqa: BLE001 — fail-open: never break voice
            _LOG.warning(
                "scheduler submit failed for %s (%s); executing directly",
                call.name,
                exc,
            )
            return await self._underlying.execute(call)
        if not getattr(task, "task_id", None):
            _LOG.warning(
                "scheduler submit returned no task_id for %s; "
                "executing directly",
                call.name,
            )
            return await self._underlying.execute(call)

        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps(
                {
                    "status": "queued",
                    "task_id": task.task_id,
                    "tool": call.name,
                    "channel": channel.value,
                    "deferred_until_voice_drained": deferred,
                },
                ensure_ascii=False,
            ),
            is_error=False,
        )

    # ----- internals -----------------------------------------------------

    def _ensure_scheduler(self) -> Optional[TaskScheduler]:
        """Lazy-create the scheduler on first use (running loop available).

        ``_build_tool_provider`` runs in the ROS2 node constructor where
        there is no running asyncio loop; the scheduler is therefore
        created on the first ``execute`` (which runs inside the dialogue
        turn's async context). Idempotent and fail-soft: if creation
        fails, ``None`` is returned and the caller executes directly.
        """
        if self._scheduler is not None or self._scheduler_attempted:
            return self._scheduler
        self._scheduler_attempted = True
        try:
            scheduler = TaskScheduler(on_event=self._on_event)
            scheduler.start()
            self._scheduler = scheduler
        except Exception as exc:  # noqa: BLE001 — fail-open
            _LOG.warning(
                "TaskScheduler init failed (%s); tool calls bypass the "
                "scheduler",
                exc,
            )
            self._scheduler = None
        return self._scheduler

    def _make_executor(
        self,
        call: ToolCall,
        deferred: bool,
    ) -> Callable[[SchedulerTask], Any]:
        """Build the per-channel executor for *call*."""

        async def _run(_task: SchedulerTask) -> Any:
            if deferred:
                # stop_music must not fire while speech is still queued
                # or playing (issue #968). Wait for the voice channel to
                # drain, then stop.
                scheduler = self._scheduler
                if scheduler is not None:
                    await scheduler.wait_until_idle(ChannelKind.VOICE)
            return await self._underlying.execute(call)

        return _run

    # ----- feedback for W7c ----------------------------------------------

    def active_tasks_block(self) -> str:
        """Return the ``[ACTIVE TASKS]`` block for the LLM system context.

        Empty string when the scheduler has no in-flight/queued work on
        any channel. Format matches W7_INTEGRATION_PLAN.md §W7c.
        """
        scheduler = self._scheduler
        if scheduler is None:
            return ""
        try:
            statuses = scheduler.all_statuses()
        except Exception:  # noqa: BLE001 — context must never crash
            return ""
        lines: list[str] = []
        for kind in (ChannelKind.VOICE, ChannelKind.MUSIC, ChannelKind.ANIM):
            st = statuses.get(kind)
            if st is None:
                continue
            if st.queue_depth == 0 and st.current_task_id is None:
                continue
            current = st.current_tool or "idle"
            lines.append(
                f"- channel={kind.value}, current={current}, "
                f"queued={st.queue_depth}, eta={st.eta_s if st.eta_s is not None else '?'}s"
            )
        if not lines:
            return ""
        return "[ACTIVE TASKS]\n" + "\n".join(lines)
