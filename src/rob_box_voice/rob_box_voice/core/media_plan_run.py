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

from rob_box_music.dj_line import SET_NOT_FOUND

from .media_phrases import dj_started_text
from .media_router import DJ_SET_TOOL, NOT_STARTED_TEXT, MediaPlan
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

    Заказ по имени (``play_name``, #3176) — тоже своя реплика, хотя ``cancel_inflight`` у него ``False`` до находки
    мелодии: без нового ``turn_id`` он нёс id прошлой команды, и ``request_music`` счёл сет прошлой реплики «своим
    ходом» (#3525).
    """
    if not (plan.cancel_inflight or plan.play_name):
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
            at = (content or "").find(SET_NOT_FOUND)  # отказ сета «названное не нашлось» — фраза кода (#3493)
            phrase = phrase or (content[at:].strip() if at >= 0 else "")
    if ok and plan.confirm_started:
        ok, phrase = await _confirm_started(plan, contents, events, log)
    return ok, phrase or (plan.say_ok if ok else plan.say_fail), done


async def _confirm_started(
    plan: MediaPlan, contents: Dict[str, str], events: Optional[MusicEventLog],
    log: Callable[[str], None],
) -> Tuple[bool, str]:
    first = plan.tool_calls[0].name
    data = tool_data(contents.get(first, "")) or {}
    track_id = data.get("track_id")
    begin = time.monotonic()
    event = await asyncio.to_thread(events.wait, track_id, STARTED_WAIT_S) if events and track_id else None
    waited = time.monotonic() - begin
    kind = event.event if event is not None else "none"
    log(f"🎛️ [media-router v2] {first} track_id={track_id} событие={kind} ждали={waited:.2f}с")
    if kind == "started":
        return True, _started_text(plan, data)
    if kind == "rejected":
        return False, plan.say_fail
    return False, NOT_STARTED_TEXT


def _started_text(plan: MediaPlan, data: Dict[str, Any]) -> str:
    """Фраза после ``started``: у ``dj_set`` без темы в команде тему назовёт результат тула — её выделил ``dj_set``
    из слов реплики (``theme_grounding.heard_theme``, 07.10), фразу строит код из результата (ADR-0148)."""
    call = plan.tool_calls[0]
    if call.name != DJ_SET_TOOL or call.arguments.get("theme") or not data.get("theme"):
        return plan.say_ok
    return dj_started_text(str(call.arguments.get("persona") or ""), str(data["theme"]))


__all__ = ["STARTED_WAIT_S", "command_turn_context", "run_media_plan"]
