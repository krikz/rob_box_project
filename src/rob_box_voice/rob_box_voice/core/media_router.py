"""media_router.py — детерминированный роутер медиакоманд ДО LLM (issue #3134).

Шаг Ш2 из ``docs/design/2026-09-28-music-dj-systemic-analysis.md``: команды
«громче / тише / на максимум / стоп / ты диджей X» исполняет код, модель их
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

Модуль чистый: без ROS и без I/O. Исполнение — в ``DialogueNode``.
"""

from __future__ import annotations

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

#: Через сколько секунд после ``set_dj_mode`` DJ-тикер делает переход #1
#: («СТАРТ ВЕЧЕРИНКИ»), если музыка не играет. 15 — нижний клэмп
#: ``DJModeController`` / ``SetDjModeTool``.
DJ_START_TRANSITION_SEC = 15


@dataclass(frozen=True)
class MediaState:
    """Снимок состояния плеера на момент реплики.

    Attributes:
        music_playing: играет ли музыка — снимок плеера
            (``_music_playing_now``, #3133 / ADR-0141).
        dj_enabled: идёт ли DJ-сет.
        track_name: название играющего трека, если известно.
    """

    music_playing: bool = False
    dj_enabled: bool = False
    track_name: Optional[str] = None


@dataclass(frozen=True)
class MediaToolCall:
    name: str
    arguments: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MediaPlan:
    """Что сделать с репликой.

    Attributes:
        command: разобранная команда (для логов).
        tool_calls: MCP-тулы по порядку. Пусто — тулы не нужны.
        say_ok: фраза, если все тулы успешны (или тулов нет). ``""`` —
            ничего не говорить.
        say_fail: фраза, если какой-то тул не сработал.
        to_llm: после тулов реплика всё равно идёт в LLM (DJ-запрос с
            содержимым вне грамматики — план, треки, лимиты).
        dj_off: стоп — выключить DJ-режим в коде (как #2897).
        cancel_inflight: отменить идущий ход LLM (команда его заменяет).
    """

    command: MediaCommand
    tool_calls: Tuple[MediaToolCall, ...] = ()
    say_ok: str = ""
    say_fail: str = ""
    to_llm: bool = False
    dj_off: bool = False
    cancel_inflight: bool = False

    @property
    def handled(self) -> bool:
        """Реплика закрыта роутером — в LLM не идёт."""
        return not self.to_llm


NOTHING_PLAYING_TEXT = "Сейчас ничего не играет."
VOLUME_FAIL_TEXT = "Не получилось поменять громкость музыки."
STOP_OK_TEXT = "Выключил музыку."
STOP_IDLE_TEXT = "Сейчас ничего не играет — на всякий случай всё остановил."
STOP_FAIL_TEXT = "Не получилось выключить музыку."
DJ_FAIL_TEXT = "Не получилось включить диджей-сет."
DJ_ALREADY_TEXT = "Сет уже идёт."
DJ_START_TEXT = "Запускаю диджей-сет."

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
        # разобрала: аргументов нет — DJ без персоны уходит в LLM.
        chosen = MediaIntent(intent)
        return MediaCommand(intent=chosen, closed=chosen is not MediaIntent.DJ)

    def route(self, user_input: str, media: MediaState) -> Optional[MediaPlan]:
        """План для реплики или ``None`` — не медиакоманда, реплика в LLM."""
        if not extract_user_utterance(user_input):
            return None
        command = self.classify(user_input, media)
        if command.intent is MediaIntent.NONE:
            return None
        return plan_media_command(command, media)


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
    # stop_music идемпотентен — зовём и в «тишине»: mp3 из библиотеки и
    # опоздавший снимок плеера не должны оставить музыку недовыключенной.
    active = media.music_playing or media.dj_enabled
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall("stop_music", {}),),
        say_ok=STOP_OK_TEXT if active else STOP_IDLE_TEXT,
        say_fail=STOP_FAIL_TEXT,
        dj_off=True,
        cancel_inflight=True,
    )


def _dj_plan(command: MediaCommand, media: MediaState) -> MediaPlan:
    args: Dict[str, Any] = {"enabled": True}
    if command.persona:
        args["persona"] = command.persona
    if command.theme:
        args["theme"] = command.theme
    if not media.dj_enabled:
        # Сет стартует переходом #1 DJ-тикера (штатный путь старта сета:
        # «СТАРТ ВЕЧЕРИНКИ», композитор, ретраи Bug B). На играющем треке
        # переход всё равно ждёт конца формы (``form_ends_at``).
        args["next_transition_sec"] = DJ_START_TRANSITION_SEC
    who = command.persona
    if not media.dj_enabled:
        ok_text = f"Я {who}, запускаю сет." if who else DJ_START_TEXT
    elif who:
        # Сет идёт: персона/тема меняются без остановки, темп сохраняется
        # (bpm не передаём — #3113), со следующего перехода — новый диджей.
        ok_text = f"Теперь я {who}. Следующий трек — уже мой."
    else:
        ok_text = DJ_ALREADY_TEXT
    closed = command.closed
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall("set_dj_mode", args),),
        say_ok=ok_text if closed else "",
        say_fail=DJ_FAIL_TEXT if closed else "",
        to_llm=not closed,
        cancel_inflight=closed,
    )


def plan_media_command(command: MediaCommand, media: MediaState) -> Optional[MediaPlan]:
    """Тулы и фраза для команды при данном состоянии плеера."""
    if command.intent in _VOLUME_ACTIONS:
        return _volume_plan(command, media)
    if command.intent is MediaIntent.STOP:
        return _stop_plan(command, media)
    if command.intent is MediaIntent.DJ:
        return _dj_plan(command, media)
    return None


__all__ = [
    "DJ_START_TRANSITION_SEC",
    "MEDIA_INTENT_QUESTION",
    "MediaPlan",
    "MediaRouter",
    "MediaState",
    "MediaToolCall",
    "NOTHING_PLAYING_TEXT",
    "default_media_provider",
    "media_intent_rules",
    "plan_media_command",
]
