"""PASTE-style shadow queue: speculative work is committed or discarded."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Mapping


@dataclass
class ShadowAction:
    action_id: str
    goal: Mapping[str, Any]
    task: asyncio.Task[Any]


class PastePlanner:
    """Prefetch N+1 actions without allowing speculative side effects."""

    def __init__(self, executor: Callable[[Mapping[str, Any]], Awaitable[Any] | Any]):
        self._executor = executor
        self._shadow: dict[str, ShadowAction] = {}

    def speculate(self, action_id: str, goal: Mapping[str, Any]) -> ShadowAction:
        if action_id in self._shadow:
            return self._shadow[action_id]
        task = asyncio.create_task(self._run(goal), name=f"paste-shadow-{action_id}")
        item = ShadowAction(action_id, dict(goal), task)
        self._shadow[action_id] = item
        return item

    async def _run(self, goal: Mapping[str, Any]) -> Any:
        value = self._executor(goal)
        return await value if asyncio.iscoroutine(value) else value

    async def commit(self, action_id: str) -> Any:
        item = self._shadow.pop(action_id, None)
        if item is None:
            raise KeyError(action_id)
        return await item.task

    def discard(self, action_id: str) -> bool:
        item = self._shadow.pop(action_id, None)
        if item is None:
            return False
        item.task.cancel()
        return True

    async def discard_all(self) -> None:
        for action_id in list(self._shadow):
            self.discard(action_id)
        await asyncio.gather(*(i.task for i in self._shadow.values()), return_exceptions=True)
