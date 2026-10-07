"""Исполнение плана медиароутера: тулы по порядку и фраза (issue #3134; ADR-0149 PR-6).

Вынесено из ``DialogueNode._execute_media_plan`` — нода только передаёт исполнителя тулов,
лог событий плеера и говорит итоговую фразу.

ADR-0149 §5.1, A14: при ``MediaPlan.confirm_started`` фраза об успехе — только после события
``started`` (``/voice/music/event``) с ``track_id`` из ответа первого тула. ``rejected`` —
честный отказ (``say_fail``); события нет за :data:`STARTED_WAIT_S` — :data:`NOT_STARTED_TEXT`.
Никакого разбора текста LLM: решает событие владельца плеера.

Модуль без ROS: юнит-тесты гоняют его на фейковом исполнителе.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any, Awaitable, Callable, Dict, List, Optional, Tuple

from .media_router import NOT_STARTED_TEXT, MediaPlan
from .music_player_state import MusicEventLog
from .music_turn import next_turn_id
from .named_play import tool_data
from .set_length_words import heard_set_length

#: Ждать ``started`` после ответа тула: A2 p100 ≤ 6 с (ADR-0149 §7.2).
STARTED_WAIT_S = 6.0

#: ``call -> (успех, текст результата)`` — исполнитель тула роутера.
CallTool = Callable[[Any], Awaitable[Tuple[bool, str]]]


def command_turn_context(plan: MediaPlan, text: str) -> Dict[str, Any]:
    """Контекст хода команды для скрытых аргументов MCP-тулов (``DialogueNode._mcp_turn_context``): атрибут ноды →
    значение.

    Команда, заменившая ход LLM (``cancel_inflight``), — свой ход: новый ``turn_id``, реплика ``heard_text`` (тему
    сета из неё выделяет ``dj_set``) и длина сета из её слов (``heard_tracks``). Без этого тулы роутера получали
    контекст ПРОШЛОГО хода LLM: тема чужой реплики подменяла тему команды, её длина — длину. Команда поверх хода
    (громкость) — ``{}``: идущий ход LLM не трогаем.
    """
    if not plan.cancel_inflight:
        return {}
    return {
        "_turn_id": next_turn_id(None),
        "_turn_heard_text": text,
        "_turn_set_tracks": heard_set_length(text),
    }


async def run_media_plan(
    plan: MediaPlan,
    call_tool: CallTool,
    events: Optional[MusicEventLog] = None,
    log: Callable[[str], None] = lambda _msg: None,
    begin_turn: Optional[Callable[[], None]] = None,
) -> Tuple[bool, str, List[str]]:
    """Тулы плана по порядку → ``(успех, фраза, исполненные тулы)``.

    ``begin_turn`` — граница хода исполнителя тулов (``SchedulerToolExecutor.begin_turn``: ``MusicTurn`` и лимит
    трека): зовётся, только если команда заменила ход LLM (:func:`command_turn_context`).
    """
    if plan.cancel_inflight and callable(begin_turn):
        begin_turn()
    ok, phrase, done, contents = True, "", [], {}
    for call in plan.tool_calls:
        call_ok, content = await call_tool(call)
        if call_ok:
            done.append(call.name)
            contents[call.name] = content
        else:
            ok = False
    if ok and plan.confirm_started:
        ok, phrase = await _confirm_started(plan, contents, events, log)
    return ok, phrase or (plan.say_ok if ok else plan.say_fail), done


async def _confirm_started(
    plan: MediaPlan, contents: Dict[str, str], events: Optional[MusicEventLog],
    log: Callable[[str], None],
) -> Tuple[bool, str]:
    first = plan.tool_calls[0].name
    track_id = (tool_data(contents.get(first, "")) or {}).get("track_id")
    begin = time.monotonic()
    event = await asyncio.to_thread(events.wait, track_id, STARTED_WAIT_S) if events and track_id else None
    waited = time.monotonic() - begin
    kind = event.event if event is not None else "none"
    log(f"🎛️ [media-router v2] {first} track_id={track_id} событие={kind} ждали={waited:.2f}с")
    if kind == "started":
        return True, plan.say_ok
    if kind == "rejected":
        return False, plan.say_fail
    return False, NOT_STARTED_TEXT


__all__ = ["STARTED_WAIT_S", "command_turn_context", "run_media_plan"]
