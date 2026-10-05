"""media_router.py — детерминированный роутер медиакоманд ДО LLM (issue #3134).

Шаг Ш2 из ``docs/design/2026-09-28-music-dj-systemic-analysis.md``: команды
«громче / тише / на максимум / стоп / ты диджей X» и заказ мелодии по имени
(«поставь к элизе», issue #3176) исполняет код, модель их
не видит (как HA Assist «Prefer handling commands locally», OVOS OCP).
Причина — ADR-0143: MiniMax игнорирует ``tool_choice``, поэтому «заставить
LLM вызвать тул» нельзя, а ретраи-гуарды вокруг неё рождают новые баги.

Два шага:

1. **Классификация** — за портом ``DecisionProvider``
   (:mod:`rob_box_harness.decision`): вопрос ``media_intent`` —
   ``ChoiceQuestion`` с закрытым набором :data:`MEDIA_INTENT_OPTIONS`.
   По умолчанию — :class:`DeterministicProvider` поверх грамматики
   :mod:`.media_command_grammar`. Jev не подключён (NO-GO,
   ``docs/research/jev/README.md``). Аргументы (персона, тема) достаёт
   грамматика: провайдер решает только интент.
2. **План** — :func:`plan_media_command`: какие MCP-тулы вызвать и какую
   фиксированную фразу сказать при данном состоянии плеера
   (:class:`MediaState`). Состояние роутер НЕ хранит — его приносит
   вызывающий из одного аксессора (``DialogueNode._media_state``).

ADR-0149 PR-6/PR-13a — музыку ведёт движок v2: «включи диджей сет на тему X» /
«ты диджей X» → ``dj_set(start)``, «поставь клубный трек» → ``request_music``, стоп →
``dj_set(stop)`` + ``stop_music``; заказ по имени (classic, PR-11) — поиск
``lookup_melody``, играет ``request_music``. Старый путь (``compose_music`` /
``set_dj_mode`` / DJ-превью) удалён в PR-13a. Фраза об успехе
запуска — только по событию ``started`` плеера (``MediaPlan.confirm_started``,
исполняет :mod:`.media_plan_run`), при ``rejected`` — честный отказ.

Модуль чистый: без ROS и без I/O. Исполнение — в ``DialogueNode``.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

from rob_box_harness.decision import (
    ChoiceAnswer,
    ChoiceQuestion,
    DecisionProvider,
    DeterministicProvider,
)

from .media_command_grammar import (
    MEDIA_INTENT_OPTIONS,
    NO_COMMAND,
    MediaCommand,
    MediaIntent,
    extract_user_utterance,
    parse_media_command,
)

_LOG = logging.getLogger(__name__)

#: Id единственного вопроса к провайдеру.
MEDIA_INTENT_QUESTION = "media_intent"


@dataclass(frozen=True)
class MediaState:
    """Снимок состояния плеера на момент реплики.

    Attributes:
        music_playing: играет ли музыка — снимок плеера
            (``_music_playing_now``, #3133 / ADR-0141).
        dj_enabled: идёт ли DJ-сет (поле ``dj.enabled`` снимка плеера v2).
        track_name: название играющего трека, если известно.
    """

    music_playing: bool = False
    dj_enabled: bool = False
    track_name: Optional[str] = None


@dataclass(frozen=True)
class MediaToolCall:
    """Один MCP-тул плана."""

    name: str
    arguments: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MediaPlan:
    """Что сделать с репликой (реплика закрыта роутером — в LLM не идёт).

    Attributes:
        command: разобранная команда (для логов).
        tool_calls: MCP-тулы по порядку. Пусто — тулы не нужны.
        say_ok: фраза, если все тулы успешны (или тулов нет). ``""`` —
            ничего не говорить.
        say_fail: фраза, если какой-то тул не сработал.
        cancel_inflight: отменить идущий ход LLM (команда его заменяет).
        play_name: issue #3176 — заказ по имени: название мелодии словами
            юзера. Непусто — нода исполняет не ``tool_calls``, а поток
            :mod:`.named_play` (``lookup_melody`` → ``request_music``):
            нашлась — играет и говорит :func:`.named_play.play_ok_text`;
            не нашлась — реплика уходит в LLM, роутер молчит.
        confirm_started: ADR-0149 PR-6 — ``say_ok`` только после события
            ``started`` с ``track_id`` из ответа первого тула; ``rejected`` →
            ``say_fail``; события нет — :data:`NOT_STARTED_TEXT` (A14).
    """

    command: MediaCommand
    tool_calls: Tuple[MediaToolCall, ...] = ()
    say_ok: str = ""
    say_fail: str = ""
    cancel_inflight: bool = False
    play_name: str = ""
    confirm_started: bool = False


NOTHING_PLAYING_TEXT = "Сейчас ничего не играет."
VOLUME_FAIL_TEXT = "Не получилось поменять громкость музыки."
STOP_OK_TEXT = "Выключил музыку."
STOP_IDLE_TEXT = "Сейчас ничего не играет — на всякий случай всё остановил."
STOP_FAIL_TEXT = "Не получилось выключить музыку."
#: ADR-0149 PR-6: плеер отказал (``rejected``) или тул не прошёл.
DJ_FAIL_TEXT = "Не получилось включить диджей-сет — музыка не заиграла."
REQUEST_FAIL_TEXT = "Не получилось включить музыку."
#: Тул ответил, а ``started`` так и не пришло: об успехе не говорим (A14).
NOT_STARTED_TEXT = "Музыка пока не заиграла."

#: Имена тулов движка v2 (``rob_box_mcp_tools.engine.tools_v2``).
DJ_SET_TOOL = "dj_set"
REQUEST_MUSIC_TOOL = "request_music"

_VOLUME_ACTIONS: Mapping[MediaIntent, Tuple[str, str]] = {
    MediaIntent.VOLUME_UP: ("louder", "Сделал музыку громче."),
    MediaIntent.VOLUME_DOWN: ("quieter", "Сделал музыку тише."),
    MediaIntent.VOLUME_MAX: ("max", "Музыка на максимуме."),
}


# ---------------------------------------------------------------------------
# Классификатор за портом DecisionProvider
# ---------------------------------------------------------------------------


def _state_track(state: Any) -> Optional[str]:
    if isinstance(state, Mapping):
        track = state.get("track_name")
        return track if isinstance(track, str) and track else None
    return None


def _state_utterance(state: Any) -> str:
    if isinstance(state, Mapping):
        return str(state.get("utterance") or "")
    return str(state or "")


def media_tool_succeeded(is_error: bool, content: str) -> bool:
    """Issue #3323 — удался ли тул роутера: транспорт И тело результата.

    Отказ исполнителя (гард, MCP) приходит как ``is_error=False`` с JSON
    ``{"success": false, ...}``; одного ``is_error`` мало — роутер писал
    ``ok=True`` и говорил «остановил» поверх отказа. Не-JSON тело
    (``'DJ-режим включён ...'``) — успех, как раньше.
    """
    if is_error:
        return False
    try:
        body = json.loads(content)
    except (TypeError, ValueError):
        return True
    return not (isinstance(body, dict) and body.get("success") is False)


def media_intent_rules(
    state: Any, questions: Mapping[str, Any]
) -> Dict[str, ChoiceAnswer]:
    """``RulesFn`` для :class:`DeterministicProvider`: интент по грамматике."""
    intent = parse_media_command(
        _state_utterance(state), track_name=_state_track(state)
    ).intent.value
    return {
        qid: ChoiceAnswer(answer=intent, probabilities={intent: 1.0}, confidence=1.0)
        for qid in questions
    }


def default_media_provider() -> DecisionProvider:
    """Провайдер по умолчанию — регекс-грамматика, без сети."""
    return DeterministicProvider(media_intent_rules)


def _decide_now(provider: DecisionProvider, state: Mapping[str, Any]) -> Optional[str]:
    """Прогнать ``provider.decide`` синхронно, если он не ждёт I/O.

    Роутер стоит в ``_on_stt`` (колбэк ROS, синхронный). Детерминированный
    провайдер отвечает без ``await`` — корутина завершается на первом
    ``send``. Провайдер, который реально ждёт (сеть), здесь не допускается:
    корутина закрывается, возвращается ``None`` — вызывающий откатывается
    на грамматику. Удалённому провайдеру нужен ``DecisionOrchestrator`` с
    дедлайном — это отдельная карточка (Jev в проде — NO-GO).
    """
    questions = {
        MEDIA_INTENT_QUESTION: ChoiceQuestion(
            text="Какую медиакоманду дал юзер?", options=MEDIA_INTENT_OPTIONS
        )
    }
    coro = provider.decide(state, questions)
    try:
        coro.send(None)
    except StopIteration as done:
        answer = done.value.answers.get(MEDIA_INTENT_QUESTION)
        return getattr(answer, "answer", None)
    except Exception as exc:  # noqa: BLE001 — нет loop'а / ошибка провайдера
        _LOG.warning("media_router: provider failed (%s) — grammar fallback", exc)
        return None
    coro.close()
    _LOG.warning(
        "media_router: provider %r awaited I/O — synchronous path refuses it, "
        "falling back to grammar", getattr(provider, "name", provider)
    )
    return None


class MediaRouter:
    """Классификатор + планировщик медиакоманд."""

    def __init__(self, provider: Optional[DecisionProvider] = None) -> None:
        self._provider = provider or default_media_provider()

    @property
    def provider_name(self) -> str:
        return str(getattr(self._provider, "name", "unknown"))

    def classify(self, user_input: str, media: MediaState) -> MediaCommand:
        """Интент — от провайдера, аргументы — от грамматики."""
        track = media.track_name if media.music_playing else None
        state = {"utterance": user_input, "track_name": track}
        parsed = parse_media_command(user_input, track_name=track)
        intent = _decide_now(self._provider, state)
        if intent is None or intent == parsed.intent.value:
            return parsed
        if intent not in MEDIA_INTENT_OPTIONS or intent == MediaIntent.NONE.value:
            return NO_COMMAND
        # Провайдер (не регекс) нашёл команду, которую грамматика не
        # разобрала: аргументов нет — DJ без персоны уходит в LLM, заказ
        # без названия (#3176) — не заказ.
        chosen = MediaIntent(intent)
        if chosen is MediaIntent.PLAY_NAMED:
            return NO_COMMAND
        return MediaCommand(intent=chosen, closed=chosen is not MediaIntent.DJ)

    def route(self, user_input: str, media: MediaState) -> Optional[MediaPlan]:
        """План для реплики или ``None`` — не медиакоманда, реплика в LLM."""
        if not extract_user_utterance(user_input):
            return None
        command = self.classify(user_input, media)
        if command.intent is MediaIntent.NONE:
            return None
        return plan_media_command(command, media, extract_user_utterance(user_input))


# ---------------------------------------------------------------------------
# План по состоянию
# ---------------------------------------------------------------------------


def _volume_plan(command: MediaCommand, media: MediaState) -> MediaPlan:
    if not media.music_playing:
        # Живой прогон 28.09: «играй громче» в тишине → LLM «Клубный трек
        # на полную» без единого тула. Честно и без LLM.
        return MediaPlan(command=command, say_ok=NOTHING_PLAYING_TEXT)
    action, ok_text = _VOLUME_ACTIONS[command.intent]
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall("set_music_volume", {"action": action}),),
        say_ok=ok_text,
        say_fail=VOLUME_FAIL_TEXT,
    )


def _stop_plan(command: MediaCommand, media: MediaState) -> MediaPlan:
    """Стоп: дека движка v2 (``dj_set(stop)``) и ``stop_music`` (mp3 из библиотеки).

    ``stop_music`` идемпотентен — зовём и в «тишине»: mp3 из библиотеки и
    опоздавший снимок плеера не должны оставить музыку недовыключенной.
    """
    active = media.music_playing or media.dj_enabled
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall(DJ_SET_TOOL, {"action": "stop"}), MediaToolCall("stop_music", {})),
        say_ok=STOP_OK_TEXT if active else STOP_IDLE_TEXT,
        say_fail=STOP_FAIL_TEXT,
        cancel_inflight=True,
    )


def _play_named_plan(command: MediaCommand) -> MediaPlan:
    """Issue #3176 — заказ по имени: исполняет нода, решает база мелодий.

    Состояние плеера плану не нужно: заказ играет и в тишине, и поверх
    трека. ``cancel_inflight`` — ``False``: идущий ход LLM отменяется только
    когда мелодия нашлась; не нашлась — реплика проходит оставшиеся шаги
    приёма (barge-in) как обычная. Играет ``request_music`` движка v2 (PR-11).
    """
    return MediaPlan(command=command, play_name=command.name)


def dj_started_text(persona: str, theme: str) -> str:
    """Фраза после ``started`` трека 1 сета (шаблон кода, I5)."""
    about = f" Тема — {theme}." if theme else ""  # «тема — X»: падеж слов человека не ломается
    return f"Я {persona}, включаю сет.{about}" if persona else f"Включаю диджей-сет.{about}"


def _dj_plan(command: MediaCommand) -> Optional[MediaPlan]:
    """«включи диджей сет на тему X» / «ты диджей X» → ``dj_set(start)`` без LLM.

    Реплика с чем-то сверх персоны/темы («…и поставь Still Dre») — не
    закрыта: её разбирает LLM, у которой есть ``dj_set``.
    """
    theme = command.theme or command.set_theme
    if not (command.closed or command.set_theme):
        return None
    persona = command.set_persona if command.set_theme else command.persona
    args: Dict[str, Any] = {"action": "start"}
    if theme:
        args["theme"] = theme
    if persona:
        args["persona"] = persona
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall(DJ_SET_TOOL, args),),
        say_ok=dj_started_text(persona, theme),
        say_fail=DJ_FAIL_TEXT,
        cancel_inflight=True,
        confirm_started=True,
    )


def _request_plan(command: MediaCommand, text: str) -> MediaPlan:
    """«поставь клубный трек» → ``request_music`` (club v2); слова человека — дословно."""
    args = {"intent": "track", "text": text}
    if command.mood:  # «музыку для танцев» → настроение из таблицы грамматики
        args["mood"] = command.mood
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall(REQUEST_MUSIC_TOOL, args),),
        say_ok=f"Включаю {command.name}." if command.name else "Включаю музыку.",
        say_fail=REQUEST_FAIL_TEXT,
        cancel_inflight=True,
        confirm_started=True,
    )


def plan_media_command(
    command: MediaCommand, media: MediaState, text: str = ""
) -> Optional[MediaPlan]:
    """Тулы и фраза для команды при данном состоянии плеера; ``text`` — слова человека."""
    if command.intent in _VOLUME_ACTIONS:
        return _volume_plan(command, media)
    if command.intent is MediaIntent.STOP:
        return _stop_plan(command, media)
    if command.intent is MediaIntent.DJ:
        return _dj_plan(command)
    if command.intent is MediaIntent.REQUEST_MUSIC:
        return _request_plan(command, text)
    if command.intent is MediaIntent.PLAY_NAMED and command.name:
        return _play_named_plan(command)
    return None


__all__ = [
    "DJ_FAIL_TEXT",
    "DJ_SET_TOOL",
    "MEDIA_INTENT_QUESTION",
    "MediaPlan",
    "MediaRouter",
    "MediaState",
    "MediaToolCall",
    "NOTHING_PLAYING_TEXT",
    "NOT_STARTED_TEXT",
    "REQUEST_FAIL_TEXT",
    "REQUEST_MUSIC_TOOL",
    "default_media_provider",
    "dj_started_text",
    "media_intent_rules",
    "plan_media_command",
]
