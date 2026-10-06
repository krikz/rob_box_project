"""music_turn.py — речь о музыке в ходе LLM решает код по результату тулов (ADR-0148).

Живой прогон 06.10 (set98207, 14:50 UTC): в одном ходе ``dj_set`` → ``request_music('1812 Overture')``
(заказ погасил сет и не заиграл) → ``speak_text('Сет запустил…')`` — неправда. 14:38 (set97492):
``dj_set`` вернул ``not_started``, а модель сказала «Сет собрался и стартовал».

:class:`MusicTurn` — запись запусков музыки за один ход (``dj_set(start)`` / ``request_music``) с их
итогом. Её ведёт ``SchedulerToolExecutor`` — единственная точка, через которую идут тулы хода:

* :meth:`MusicTurn.speech_allowed` — пока ПОСЛЕДНИЙ запуск хода не удался, ``speak_text`` модели не
  исполняется (заявить «запустил» нечем). Успешный запуск речь не запрещает: «рэп под бит» —
  ``request_music`` и потом ``speak_text`` × N (скилл composer).
* :meth:`MusicTurn.phrase` — что сказать по итогу хода, шаблоны :mod:`.media_phrases`; ``None`` — запусков
  в ходе не было.
* :func:`turn_reply` — фраза хода о музыке, которую нода говорит ВМЕСТО текста модели. 06.10 15:30 UTC:
  «ретро 8-бит … замути сэт на 30 минут» → ``tools=[]`` и «Сет идёт — 30 минут ретро-ностальгии!». Просьбу
  о запуске узнаёт код по словам человека (грамматика медиакоманд и длина сета, :func:`launch_requested`),
  а не по тексту модели: просили, а запуска в ходе не было — честная :data:`.media_phrases.NOT_LAUNCHED_TEXT`.

Итог тула — :func:`.media_router.media_tool_succeeded` (то же правило, что у роутера медиакоманд):
``request_music``/``dj_set`` отвечают успехом только по событию ``started`` плеера (A14).
"""

from __future__ import annotations

import ast
import json
import uuid
from dataclasses import dataclass, field
from typing import Any, List, Mapping, Optional, Sequence

from .media_command_grammar import MediaIntent, parse_media_command
from .media_phrases import (
    DJ_FAIL_TEXT,
    NOT_LAUNCHED_TEXT,
    REQUEST_FAIL_TEXT,
    SET_KEPT_TEXT,
    SET_PLAYING_REASON,
    dj_started_text,
    play_ok_text,
    request_ok_text,
)
from .media_router import DJ_SET_TOOL, REQUEST_MUSIC_TOOL, media_tool_succeeded

#: Код отказа ``speak_text`` в ходе, где запуск музыки не удался.
SPEECH_REFUSAL_CODE = "music_phrase_by_code"
#: Что видит модель вместо исполнения ``speak_text``.
SPEECH_REFUSAL_MESSAGE = (
    "speak_text не исполнен: музыка в этом ходе не запустилась (смотри результат dj_set / "
    "request_music). Что сказать человеку о музыке, робот решит сам по результату тулов. "
    "Заверши ход словом done."
)


@dataclass(frozen=True)
class Launch:
    """Один запуск музыки в ходе: тул, его аргументы и итог."""

    tool: str
    args: Mapping[str, Any]
    ok: bool
    content: str


def _data(content: str) -> Mapping[str, Any]:
    """Тело успешного результата MCP-тула: ``repr(dict)`` (``core_adapter``) или JSON; иначе ``{}``."""
    for parse in (ast.literal_eval, json.loads):
        try:
            body = parse(content)
        except (ValueError, SyntaxError, TypeError, MemoryError, RecursionError):
            continue
        if isinstance(body, dict):
            return body
    return {}


def is_launch(tool: str, args: Mapping[str, Any]) -> bool:
    """Запуск музыки: ``request_music`` или ``dj_set`` с ``action=start`` (стоп сета — не запуск)."""
    if tool == REQUEST_MUSIC_TOOL:
        return True
    return tool == DJ_SET_TOOL and str(args.get("action") or "start") == "start"


def ok_text(launch: Launch) -> str:
    """Фраза об удавшемся запуске — те же шаблоны, что у роутера медиакоманд."""
    args = launch.args
    if launch.tool == DJ_SET_TOOL:
        return dj_started_text(str(args.get("persona") or ""), str(args.get("theme") or ""))
    title = str(_data(launch.content).get("title") or "")
    if args.get("intent") == "melody" and title:
        return play_ok_text(title)
    return request_ok_text("")


def fail_text(launch: Launch) -> str:
    """Фраза о запуске, который не удался."""
    return DJ_FAIL_TEXT if launch.tool == DJ_SET_TOOL else REQUEST_FAIL_TEXT


@dataclass
class MusicTurn:
    """Запуски музыки за один ход LLM и речь, которую они разрешают (ADR-0148)."""

    launches: List[Launch] = field(default_factory=list)

    def reset(self) -> None:
        """Граница хода."""
        self.launches.clear()

    def record(self, tool: str, args: Optional[Mapping[str, Any]], *, is_error: bool, content: str) -> None:
        """Учесть исполненный вызов тула; не запуск музыки — ничего."""
        args = dict(args or {})
        if is_launch(tool, args):
            self.launches.append(Launch(tool, args, media_tool_succeeded(is_error, content), content))

    def speech_allowed(self) -> bool:
        """``speak_text`` модели можно исполнить: запусков не было или последний удался."""
        return not self.launches or self.launches[-1].ok

    def phrase(self) -> Optional[str]:
        """Что сказать о музыке по итогу хода; ``None`` — запусков музыки в ходе не было."""
        if not self.launches:
            return None
        last = self.launches[-1]
        if last.ok:
            return ok_text(last)
        playing = next((launch for launch in reversed(self.launches) if launch.ok), None)
        if playing is not None and SET_PLAYING_REASON in last.content:
            return f"{ok_text(playing)} {SET_KEPT_TEXT}"
        return fail_text(last)


#: Интенты грамматики, которые просят ЗАПУСТИТЬ музыку. ``PLAY_NAMED`` сюда не входит: его промах по базе
#: уходит в LLM, и «такой мелодии нет» без тула запуска — честный ответ.
LAUNCH_INTENTS: frozenset = frozenset({MediaIntent.DJ, MediaIntent.REQUEST_MUSIC})


def launch_requested(user_text: str, heard_tracks: Optional[int] = None) -> bool:
    """Человек просил запустить музыку: длина сета в словах (``heard_set_length``) или интент запуска."""
    return heard_tracks is not None or parse_media_command(user_text or "").intent in LAUNCH_INTENTS


def turn_reply(turn: MusicTurn, *, requested: bool, voiced: int) -> Optional[str]:
    """Фраза о музыке, которую код говорит вместо текста модели; ``None`` — речь хода не трогать.

    * запусков не было: просили запуск — :data:`NOT_LAUNCHED_TEXT`, иначе ``None``;
    * последний запуск удался и модель уже сказала своё через ``speak_text`` (``voiced`` > 0, «рэп под бит») —
      ``None``; не сказала — фраза об успехе (:meth:`MusicTurn.phrase`);
    * последний запуск не удался — честная фраза :meth:`MusicTurn.phrase` всегда.
    """
    if not turn.launches:
        return NOT_LAUNCHED_TEXT if requested else None
    if turn.launches[-1].ok and voiced > 0:
        return None
    return turn.phrase()


def executor_turn_reply(
    executor: Any, user_texts: Sequence[Optional[str]], heard_tracks: Optional[int], voiced: int
) -> Optional[str]:
    """:func:`turn_reply` по ``MusicTurn`` исполнителя тулов (``SchedulerToolExecutor.music_turn``); исполнителя
    нет — ``None``. ``user_texts`` — слова человека по приоритету (сырая команда ретрая, затем реплика хода)."""
    turn = getattr(executor, "music_turn", None)
    if turn is None:
        return None
    text = next((t for t in user_texts if t), "")
    return turn_reply(turn, requested=launch_requested(text, heard_tracks), voiced=voiced)


def next_turn_id(current: Optional[str], *retry_flags: bool) -> str:
    """Ход для скрытого ``turn_id`` MCP-тулов: новый на реплику человека; синтетический ретрай (любой из
    ``retry_flags``) продолжает ход, который его вызвал."""
    return current if current and any(retry_flags) else uuid.uuid4().hex


def speech_refusal_content() -> str:
    """JSON-тело отказа ``speak_text`` для ``ToolResult.content``."""
    return json.dumps(
        {"success": False, "error": SPEECH_REFUSAL_CODE, "message": SPEECH_REFUSAL_MESSAGE},
        ensure_ascii=False,
    )


__all__ = [
    "LAUNCH_INTENTS",
    "Launch",
    "MusicTurn",
    "SPEECH_REFUSAL_CODE",
    "SPEECH_REFUSAL_MESSAGE",
    "executor_turn_reply",
    "fail_text",
    "is_launch",
    "launch_requested",
    "next_turn_id",
    "ok_text",
    "speech_refusal_content",
    "turn_reply",
]
