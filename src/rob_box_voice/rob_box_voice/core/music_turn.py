"""music_turn.py — речь о музыке в ходе LLM решает код по результату тулов (ADR-0148).

Живой прогон 06.10 (set98207, 14:50 UTC): в одном ходе ``dj_set`` → ``request_music('1812 Overture')``
(заказ погасил сет и не заиграл) → ``speak_text('Сет запустил…')`` — неправда. 14:38 (set97492):
``dj_set`` вернул ``not_started``, а модель сказала «Сет собрался и стартовал».

:class:`MusicTurn` — запись запусков музыки за один ход (``dj_set(start)`` / ``request_music``) с их
итогом. Её ведёт ``SchedulerToolExecutor`` — единственная точка, через которую идут тулы хода:

* :meth:`MusicTurn.speech_allowed` — пока ПОСЛЕДНИЙ запуск хода не удался, ``speak_text`` модели не
  исполняется (заявить «запустил» нечем). Успешный запуск речь не запрещает: «рэп под бит» —
  ``request_music`` и потом ``speak_text`` × N (скилл composer). Исключение — заиграла пьеса (поле ``played``
  результата, ADR-0157 R6): что играет и чем заменили, говорит код (прогон 12.10: модель пересказала
  «Toccata And Fugue» своими словами через ``speak_text``).
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

from rob_box_core.music_tools import MUSIC_AUDIBLE_LYRICS_TOOLS as _LEGACY_MUSIC_TOOLS

from .media_command_grammar import MediaIntent, parse_media_command
from .media_phrases import (
    DJ_FAIL_TEXT,
    NOT_LAUNCHED_TEXT,
    NOT_PLAYED_TEXT,
    LIBRARY_FAIL_TEXT,
    LIBRARY_NO_TRACK_TEXT,
    REQUEST_FAIL_TEXT,
    SET_KEPT_TEXT,
    SET_PLAYING_REASON,
    dj_started_text,
    launch_text,
    play_ok_text,
    request_ok_text,
    source_phrase,
)
from .media_router import DJ_SET_TOOL, GENERATE_MUSIC_TOOL, REQUEST_MUSIC_TOOL, media_tool_succeeded

#: Тул библиотеки генераций (ADR-0158 PR-9) и его действия, итог которых говорит код (фраза — первая строка
#: результата, ``MCPToolResult.message``); ``search`` / ``info`` модель пересказывает по полям результата сама.
LIBRARY_TOOL = "music_library"
LIBRARY_SPOKEN_ACTIONS = ("list", "play", "rate", "update", "delete")
#: Звуковой эффект (ADR-0150 PR-7b, #3698): его итог - ``SoundEvent`` владельца звука, фразу об исходе строит тул.
#: Ход, где эффект исполнился (``done`` или честный отказ), - про звук, а не про пьесу: «Ничего не заиграло» музыки
#: о нём не говорится (живой прогон 21.10 15:39Z: звук играл, а робот сказал, что ничего не заиграло, #3794).
SOUND_TOOL = "play_sound"
#: Код отказа ``speak_text`` в ходе, где запуск музыки не удался.
SPEECH_REFUSAL_CODE = "music_phrase_by_code"
#: Что видит модель вместо исполнения ``speak_text``.
SPEECH_REFUSAL_MESSAGE = (
    "speak_text не исполнен: музыка в этом ходе не запустилась (смотри результат dj_set / "
    "request_music). Что сказать человеку о музыке, робот решит сам по результату тулов. "
    "Заверши ход словом done."
)
#: То же, когда итог библиотеки уже сказал код (``music_library``: список, играет, оценка, удаление).
SPEECH_LIBRARY_MESSAGE = (
    "speak_text не исполнен: итог music_library робот скажет сам, фразой из результата. Заверши ход словом done."
)
#: То же, когда пьеса заиграла: фразу о ней робот уже строит из ``played`` результата ``request_music``.
SPEECH_PIECE_MESSAGE = (
    "speak_text не исполнен: пьеса играет, что именно играет — робот скажет сам фразой из результата "
    "request_music. Заверши ход словом done."
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
    if tool in (REQUEST_MUSIC_TOOL, GENERATE_MUSIC_TOOL):
        return True
    return tool == DJ_SET_TOOL and str(args.get("action") or "start") == "start"


def library_result_phrase(is_error: bool, content: str) -> str:
    """Фраза итога ``music_library``: успех — ``message`` результата (первая строка), отказ — честная фраза кода."""
    if not is_error:
        return content.split("\n", 1)[0].strip() or LIBRARY_FAIL_TEXT
    return content if content == LIBRARY_NO_TRACK_TEXT else LIBRARY_FAIL_TEXT


def ok_text(launch: Launch) -> str:
    """Фраза об удавшемся запуске — те же шаблоны, что у роутера медиакоманд."""
    args = launch.args
    if launch.tool == DJ_SET_TOOL:
        return dj_started_text(str(args.get("persona") or ""), str(args.get("theme") or ""))
    data = _data(launch.content)
    title = str(data.get("title") or "")
    if args.get("intent") == "melody" and title:
        return launch_text(play_ok_text(title), data)
    return launch_text(request_ok_text(""), data)


def fail_text(launch: Launch) -> str:
    """Фраза о запуске, который не удался; отказ чойсера пути (ADR-0158) — его фраза кода."""
    return DJ_FAIL_TEXT if launch.tool == DJ_SET_TOOL else source_phrase(launch.content) or REQUEST_FAIL_TEXT


@dataclass
class MusicTurn:
    """Запуски музыки за один ход LLM и речь, которую они разрешают (ADR-0148)."""

    launches: List[Launch] = field(default_factory=list)
    #: Фраза итога ``music_library`` хода (список / играет / оценка / удаление) или ``None``.
    library_phrase: Optional[str] = None
    #: Исходы ``play_sound`` хода: ``True`` - звук заиграл (``SoundEvent started``), ``False`` - отказ/таймаут.
    sounds: List[bool] = field(default_factory=list)

    def reset(self) -> None:
        """Граница хода."""
        self.launches.clear()
        self.library_phrase = None
        self.sounds.clear()

    def library_answered(self) -> bool:
        """Итог библиотеки хода говорит код, запусков v2 в ходе не было."""
        return self.library_phrase is not None and not self.launches

    def record(self, tool: str, args: Optional[Mapping[str, Any]], *, is_error: bool, content: str) -> None:
        """Учесть исполненный вызов тула; не запуск музыки — ничего."""
        args = dict(args or {})
        if is_launch(tool, args):
            self.launches.append(Launch(tool, args, media_tool_succeeded(is_error, content), content))
        elif tool == LIBRARY_TOOL and str(args.get("action") or "") in LIBRARY_SPOKEN_ACTIONS:
            self.library_phrase = library_result_phrase(is_error, content)
        elif tool == SOUND_TOOL:
            self.sounds.append(not is_error)

    def speech_allowed(self) -> bool:
        """``speak_text`` модели можно исполнить: запусков не было или последний удался и это не пьеса
        (:meth:`piece_playing`)."""
        if self.library_answered():
            return False
        return not self.launches or (self.launches[-1].ok and not self.piece_playing())

    def piece_playing(self) -> bool:
        """Последний запуск хода удался и это пьеса (``played`` в результате): о ней говорит код."""
        return bool(self.launches) and self.launches[-1].ok and "played" in _data(self.launches[-1].content)

    def phrase(self) -> Optional[str]:
        """Что сказать о музыке по итогу хода; ``None`` — запусков музыки в ходе не было."""
        if not self.launches:
            return self.library_phrase
        last = self.launches[-1]
        if last.ok:
            return ok_text(last)
        playing = next((launch for launch in reversed(self.launches) if launch.ok), None)
        if playing is not None and SET_PLAYING_REASON in last.content:
            return f"{ok_text(playing)} {SET_KEPT_TEXT}"
        return fail_text(last)


#: Интенты грамматики, которые просят ЗАПУСТИТЬ музыку. ``PLAY_NAMED`` сюда не входит: его промах по базе
#: уходит в LLM, и «такой мелодии нет» без тула запуска — честный ответ.
LAUNCH_INTENTS: frozenset = frozenset({MediaIntent.DJ, MediaIntent.REQUEST_MUSIC, MediaIntent.GENERATE})


#: Интенты, где роутер сам запускает музыку и говорит фразу запуска (сет, заказ, мелодия по имени): ход в историю
#: модели фразой не кладётся (#3565). ``dj_set`` у ``STOP`` («выключи») запуском не является.
ROUTER_LAUNCH_INTENTS: frozenset = LAUNCH_INTENTS | {MediaIntent.PLAY_NAMED, MediaIntent.LIBRARY}


def history_recorder(core: Any, intent: MediaIntent) -> Optional[Any]:
    """``AgentCore.record_external_turn`` для хода роутера или ``None``: ход с запуском (:data:`ROUTER_LAUNCH_INTENTS`)
    в окно модели не пишется — фраза запуска стала бы образцом, который модель повторяет с ``tools=[]`` (#3461)."""
    record = getattr(core, "record_external_turn", None)
    return record if callable(record) and intent not in ROUTER_LAUNCH_INTENTS else None


def launch_requested(user_text: str, heard_tracks: Optional[int] = None) -> bool:
    """Человек просил запустить музыку: длина сета в словах (``heard_set_length``) или интент запуска."""
    return heard_tracks is not None or parse_media_command(user_text or "").intent in LAUNCH_INTENTS


def piece_requested(user_text: str) -> bool:
    """Человек назвал мелодию/пьесу («сыграй Токкату ре минор», «сыграй Баха на трамбоне») — интент ``PLAY_NAMED``.
    Её запускает роутер (:mod:`.named_play`); дошла до модели — значит, роутер промахнулся, и запуск — тул модели."""
    return parse_media_command(user_text or "").intent is MediaIntent.PLAY_NAMED


def turn_reply(
    turn: MusicTurn, *, requested: bool, voiced: int, piece: bool = False, other_launch: bool = False
) -> Optional[str]:
    """Фраза о музыке, которую код говорит вместо текста модели; ``None`` — речь хода не трогать.

    * запусков не было: просили сет — :data:`NOT_LAUNCHED_TEXT`; просили пьесу (``piece``) и никакой другой
      музыкальный тул не сработал (``other_launch``) — :data:`NOT_PLAYED_TEXT` (прогон 12.10 16:48: «Токкату ре
      минор Баха ставлю мелодией.» с ``tools=[]``); иначе ``None``. Ход со звуковым эффектом с исходом
      (``turn.sounds``: «включи звук радостный» грамматика тоже читает как пьесу) — про звук: об исходе говорит
      фраза тула ``play_sound``, «Ничего не заиграло» не звучит;
    * последний запуск удался и модель уже сказала своё через ``speak_text`` (``voiced`` > 0, «рэп под бит») —
      ``None``; не сказала — фраза об успехе (:meth:`MusicTurn.phrase`);
    * последний запуск не удался — честная фраза :meth:`MusicTurn.phrase` всегда.
    """
    if turn.library_answered():
        return turn.library_phrase
    if not turn.launches:
        if requested:
            return NOT_LAUNCHED_TEXT
        return NOT_PLAYED_TEXT if piece and not other_launch and not turn.sounds else None
    if turn.launches[-1].ok and voiced > 0:
        return None
    return turn.phrase()


def executor_turn_reply(
    executor: Any,
    user_texts: Sequence[Optional[str]],
    heard_tracks: Optional[int],
    voiced: int,
    tools_called: Sequence[str] = (),
) -> Optional[str]:
    """:func:`turn_reply` по ``MusicTurn`` исполнителя тулов (``SchedulerToolExecutor.music_turn``); исполнителя
    нет — ``None``. ``user_texts`` — слова человека по приоритету (сырая команда ретрая, затем реплика хода);
    ``tools_called`` — тулы хода: музыку мог запустить не v2-тул (``music_library`` и др.)."""
    turn = getattr(executor, "music_turn", None)
    if turn is None:
        return None
    text = next((t for t in user_texts if t), "")
    other = bool(set(tools_called or ()) & _LEGACY_MUSIC_TOOLS)
    return turn_reply(
        turn, requested=launch_requested(text, heard_tracks), voiced=voiced,
        piece=piece_requested(text), other_launch=other,
    )


def next_turn_id(current: Optional[str], *retry_flags: bool) -> str:
    """Ход для скрытого ``turn_id`` MCP-тулов: новый на реплику человека; синтетический ретрай (любой из
    ``retry_flags``) продолжает ход, который его вызвал."""
    return current if current and any(retry_flags) else uuid.uuid4().hex


def turn_heard_text(current: Optional[str], user_input: str, is_synthetic: bool) -> Optional[str]:
    """Реплика хода для скрытого ``heard_text`` MCP-тулов: реплика человека; синтетический ретрай продолжает ход,
    его служебный текст репликой не становится."""
    return current if is_synthetic else user_input


def speech_refusal_content(piece: bool = False, library: bool = False) -> str:
    """JSON-тело отказа ``speak_text`` для ``ToolResult.content``; ``piece`` — пьеса играет
    (:data:`SPEECH_PIECE_MESSAGE`), ``library`` — итог библиотеки говорит код (:data:`SPEECH_LIBRARY_MESSAGE`)."""
    message = SPEECH_LIBRARY_MESSAGE if library else SPEECH_PIECE_MESSAGE if piece else SPEECH_REFUSAL_MESSAGE
    return json.dumps({"success": False, "error": SPEECH_REFUSAL_CODE, "message": message}, ensure_ascii=False)


__all__ = [
    "LAUNCH_INTENTS",
    "ROUTER_LAUNCH_INTENTS",
    "Launch",
    "MusicTurn",
    "SPEECH_REFUSAL_CODE",
    "LIBRARY_SPOKEN_ACTIONS",
    "LIBRARY_TOOL",
    "SPEECH_LIBRARY_MESSAGE",
    "SPEECH_PIECE_MESSAGE",
    "SPEECH_REFUSAL_MESSAGE",
    "executor_turn_reply",
    "history_recorder",
    "fail_text",
    "is_launch",
    "launch_requested",
    "piece_requested",
    "next_turn_id",
    "turn_heard_text",
    "ok_text",
    "speech_refusal_content",
    "turn_reply",
]
