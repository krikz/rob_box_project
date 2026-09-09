"""Async action server primitives with cancellation and graceful shutdown."""
from __future__ import annotations

import asyncio
import inspect
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Mapping


class ActionError(RuntimeError):
    """Invalid action transition or handler failure."""


class ActionState(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    CANCELLED = "cancelled"
    FAILED = "failed"


@dataclass(frozen=True)
class Health:
    ok: bool
    active_goals: int
    shutting_down: bool


@dataclass
class ActionHandle:
    goal_id: str
    task: asyncio.Task[Any]
    state: ActionState = ActionState.ACCEPTED
    feedback: list[Mapping[str, Any]] = field(default_factory=list)
    result: Any = None
    error: str | None = None

    async def wait(self) -> Any:
        return await self.task

    def cancel(self) -> bool:
        if self.task.done():
            return False
        return self.task.cancel()


Handler = Callable[[Mapping[str, Any], Callable[[Mapping[str, Any]], None], asyncio.Event], Any]


class ActionServer:
    """Owns long-running actions and provides a deterministic lifecycle.

    A handler receives ``goal``, a synchronous feedback callback and a
    cancellation event.  It may be sync or async.  Cancellation is delivered
    both by task cancellation and the cooperative event, allowing providers
    to stop network/audio work promptly.
    """

    def __init__(self, handler: Handler):
        self._handler = handler
        self._handles: dict[str, ActionHandle] = {}
        self._cancel_events: dict[str, asyncio.Event] = {}
        self._shutting_down = False

    @property
    def active_goals(self) -> int:
        return sum(not h.task.done() for h in self._handles.values())

    def health(self) -> Health:
        return Health(not self._shutting_down, self.active_goals, self._shutting_down)

    def submit(self, goal: Mapping[str, Any]) -> ActionHandle:
        if self._shutting_down:
            raise ActionError("action server is shutting down")
        goal_id = str(goal.get("goal_id") or uuid.uuid4())
        cancel_event = asyncio.Event()
        task = asyncio.create_task(self._run(goal_id, goal, cancel_event), name=f"action-{goal_id}")
        handle = ActionHandle(goal_id, task)
        self._handles[goal_id] = handle
        self._cancel_events[goal_id] = cancel_event
        return handle

    async def _run(self, goal_id: str, goal: Mapping[str, Any], cancel_event: asyncio.Event) -> Any:
        handle = self._handles[goal_id]
        handle.state = ActionState.RUNNING
        def feedback(value: Mapping[str, Any]) -> None:
            if not handle.task.done():
                handle.feedback.append(dict(value))
        try:
            value = self._handler(goal, feedback, cancel_event)
            value = await value if inspect.isawaitable(value) else value
            if cancel_event.is_set():
                handle.state = ActionState.CANCELLED
                return None
            handle.result, handle.state = value, ActionState.SUCCEEDED
            return value
        except asyncio.CancelledError:
            cancel_event.set()
            handle.state = ActionState.CANCELLED
            raise
        except Exception as exc:  # retain failure for the caller and health diagnostics
            handle.error, handle.state = str(exc), ActionState.FAILED
            raise

    def cancel(self, goal_id: str) -> bool:
        handle = self._handles.get(goal_id)
        if not handle or handle.task.done():
            return False
        self._cancel_events[goal_id].set()
        return handle.cancel()

    async def shutdown(self, timeout: float = 2.0) -> None:
        self._shutting_down = True
        active = [h for h in self._handles.values() if not h.task.done()]
        for handle in active:
            self._cancel_events[handle.goal_id].set()
            handle.cancel()
        if active:
            await asyncio.wait([h.task for h in active], timeout=timeout)
