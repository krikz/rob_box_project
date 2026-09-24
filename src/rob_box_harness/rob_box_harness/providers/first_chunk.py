"""Ожидание с дедлайном, которое не ждёт признания отмены (issue #2939).

``asyncio.wait_for`` по таймауту отменяет внутреннюю корутину и затем
ЖДЁТ, пока та отмену признает. Если отмена застряла в транспорте SDK
(ретрай внутри ``AsyncOpenAI``, httpx/anyio), ``wait_for`` висит вместе
с ней — дедлайн first-chunk-гуарда #2718 превращается в «сколько SDK
захочет». Живой ход #2939 так провисел 3 минуты вместо 10 секунд.

:func:`await_with_deadline` отменяет ожидание и сразу бросает
``asyncio.TimeoutError``; брошенная задача дотлевает в фоне, её
исключение поглощается колбэком (без «Task exception was never
retrieved» в логе).
"""

from __future__ import annotations

import asyncio
from typing import Any, Awaitable

__all__ = ["await_with_deadline"]


def _swallow_result(task: "asyncio.Future[Any]") -> None:
    if not task.cancelled():
        task.exception()


async def await_with_deadline(awaitable: Awaitable[Any], timeout_s: float) -> Any:
    """Дождаться ``awaitable`` не дольше ``timeout_s`` секунд.

    По дедлайну — ``asyncio.TimeoutError`` сразу, без ожидания, пока
    отменённая задача остановится. Отмена самого вызывающего тоже
    отменяет задачу (без ожидания).
    """
    task = asyncio.ensure_future(awaitable)
    try:
        done, _pending = await asyncio.wait({task}, timeout=timeout_s)
    except BaseException:
        task.cancel()
        task.add_done_callback(_swallow_result)
        raise
    if task in done:
        return task.result()
    task.cancel()
    task.add_done_callback(_swallow_result)
    raise asyncio.TimeoutError(f"no result within {timeout_s:.1f}s")
