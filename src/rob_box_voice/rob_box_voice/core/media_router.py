"""media_router.py — детерминированный роутер медиакоманд ДО LLM (issue #3134).

Шаг Ш2 из ``docs/design/2026-09-28-music-dj-systemic-analysis.md``: команды
«стоп / ты диджей X» и заказ мелодии по имени (громкость — Tier-1 ``rob_box_dialog.grammar``, ADR-0150 PR-6)
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
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

from rob_box_harness.decision import (
    ChoiceAnswer,
    ChoiceQuestion,
    DecisionProvider,
    DeterministicProvider,
)
from rob_box_dialog.phrases import text as _phrase
from rob_box_music.dj_line import now_playing_text, plain_fold, when_text

from .media_command_grammar import (
    MEDIA_INTENT_OPTIONS,
    NO_COMMAND,
    MediaCommand,
    MediaIntent,
    extract_user_utterance,
    parse_media_command,
)
from .media_phrases import (
    DJ_FAIL_TEXT,
    NOTHING_PLAYING_TEXT,
    REQUEST_FAIL_TEXT,
    dj_started_text,
    now_playing_track_text,
    request_ok_text,
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
        dj_persona: имя диджея идущего сета (поле ``dj.persona`` снимка), если есть.
    """

    music_playing: bool = False
    dj_enabled: bool = False
    track_name: Optional[str] = None
    dj_persona: str = ""
    set_facts: Mapping[str, Any] = field(default_factory=dict)
    #: Факты одиночного трека деки (поле ``dj`` снимка + ``elapsed_s``/``remaining_s``) — «что играет» (#3613).
    track_facts: Mapping[str, Any] = field(default_factory=dict)


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
        library: #3696 — ``play``/``list`` из библиотеки генераций: исполняет
            :mod:`.library_command` (поиск/список → трек выбирает код → ``play``).
    """

    command: MediaCommand
    tool_calls: Tuple[MediaToolCall, ...] = ()
    say_ok: str = ""
    say_fail: str = ""
    cancel_inflight: bool = False
    play_name: str = ""
    confirm_started: bool = False
    library: str = ""

    @property
    def needs_tools(self) -> bool:
        """Плану нужен MCP-исполнитель: тулы по порядку или поток библиотеки (#3696)."""
        return bool(self.tool_calls or self.library)


#: Тексты — ``rob_box_dialog/phrases/ru.yaml`` (ключи ``music.*``, ADR-0150 PR-1).
STOP_OK_TEXT = _phrase("music.stop_ok")
STOP_IDLE_TEXT = _phrase("music.stop_idle")
STOP_FAIL_TEXT = _phrase("music.stop_fail")
#: Тул ответил, а ``started`` так и не пришло: об успехе не говорим (A14).
NOT_STARTED_TEXT = _phrase("music.not_started")

#: Имена тулов движка v2 (``rob_box_mcp_tools.engine.tools_v2``).
DJ_SET_TOOL = "dj_set"
REQUEST_MUSIC_TOOL = "request_music"
#: Просьба сгенерировать (ADR-0158 PR-3): тот же чойсер пути, что у ``request_music``.
GENERATE_MUSIC_TOOL = "generate_music"

# ---------------------------------------------------------------------------
# Классификатор за портом DecisionProvider
# ---------------------------------------------------------------------------


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
    intent = parse_media_command(_state_utterance(state)).intent.value
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
        state = {"utterance": user_input}
        parsed = parse_media_command(user_input)
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


def _dj_plan(command: MediaCommand) -> Optional[MediaPlan]:
    """«включи диджей сет на тему X» / «ты диджей X» → ``dj_set(start)`` без LLM.

    Реплика с чем-то сверх персоны/темы («…и поставь Still Dre») — не
    закрыта: её разбирает LLM, у которой есть ``dj_set``.
    """
    theme = command.set_theme  # тему из свободных слов реплики выделяет dj_set из heard_text (theme_grounding)
    if not (command.closed or command.set_theme):
        return None
    persona = command.set_persona if command.set_theme else command.persona
    args: Dict[str, Any] = {"action": "start"}
    if theme:
        args["theme"] = theme
    if persona:
        args["persona"] = persona
    if command.style:  # «рейв на тему X» — стиль решила грамматика (ADR-0153 S1)
        args["style"] = command.style
    if command.tracks:  # «сет на 3 трека» — длину решила грамматика (06.10), без числа — тул: 10 или остаток сета
        args["tracks"] = command.tracks
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall(DJ_SET_TOOL, args),),
        say_ok=dj_started_text(persona, theme),
        say_fail=DJ_FAIL_TEXT,
        cancel_inflight=True,
        confirm_started=True,
    )


#: Факты играющего трека сета в снимке (``dj``), которыми код отвечает на «что играет / когда будет X».
SET_FACTS = ("track_no", "tracks", "theme", "melody", "played", "next_melodies", "next_known", "not_found")


def _fold() -> Any:
    """Свёртка названий движка (транслит: «Марио» ~ «Super Mario»); без пакета движка — простая."""
    try:
        from rob_box_mcp_tools.engine.dj_lines import latin_fold
    except ImportError:
        return plain_fold
    return latin_fold


def _now_playing_plan(command: MediaCommand, media: MediaState) -> Optional[MediaPlan]:
    """Ответ о сете из фактов снимка (ADR-0148: фразу строит код). Код не знает ответа (нет фактов, X не совпал ни
    с одной мелодией сета) — ``None``: вопрос уходит LLM как раньше."""
    facts = media.set_facts
    if not command.name:
        if not media.music_playing:
            return MediaPlan(command=command, say_ok=NOTHING_PLAYING_TEXT)
        if media.dj_enabled and facts.get("track_no"):
            return MediaPlan(command=command, say_ok=now_playing_text(facts))
        if media.track_facts.get("source") or media.track_name:  # одиночный трек деки: факты владельца плеера
            return MediaPlan(command=command, say_ok=now_playing_track_text(media.track_facts))
        return None
    if not (media.music_playing and media.dj_enabled and facts.get("track_no")):
        return None
    text = when_text(facts, command.name, _fold())
    return MediaPlan(command=command, say_ok=text) if text else None


def _track_facts(playing: bool, info: Mapping[str, Any], now: float) -> Dict[str, Any]:
    """Факты трека из снимка: сколько играет и сколько осталось считает код по ``started_at``/``length_s``."""
    facts: Dict[str, Any] = {**info, "playing": playing}
    started = info.get("started_at")
    if playing and isinstance(started, (int, float)):
        facts["elapsed_s"] = max(0.0, now - float(started))
        if isinstance(info.get("length_s"), (int, float)):
            facts["remaining_s"] = max(0.0, float(info["length_s"]) - facts["elapsed_s"])
    return facts


def media_state_from_snapshot(playing: bool, snapshot: Any, now: Optional[float] = None) -> MediaState:
    """Снимок плеера (``MusicPlayerState`` или ``None``) → :class:`MediaState`."""
    info = snapshot.dj_info if snapshot is not None else {}
    title = str(info.get("title") or "")
    return MediaState(
        track_facts=_track_facts(playing, info, time.time() if now is None else now),
        music_playing=playing,
        dj_enabled=snapshot is not None and bool(snapshot.dj),
        track_name=title if playing and title else None,
        dj_persona=str(info.get("persona") or ""),
        set_facts={k: info[k] for k in SET_FACTS if k in info},
    )


def dj_theme_switch_plan(plan: MediaPlan, media: MediaState) -> Optional[MediaPlan]:
    """Issue #3400 — заказ «X» мимо базы мелодий посреди сета, значит тема X: ``dj_set(start, theme=X)``, без LLM.

    ``None`` — сета нет: реплика уходит в LLM, как раньше. Мелодия из базы
    сет не трогает (#3176) — сюда доходит только промах поиска.
    Персона идущего сета сохраняется. Фразу об успехе строит код по событию
    ``started`` (как у «включи сет на тему X»), отказ — честная ``DJ_FAIL_TEXT``.
    """
    if not (media.dj_enabled and plan.play_name):
        return None
    command = MediaCommand(intent=MediaIntent.DJ, closed=True, set_theme=plan.play_name)
    persona = media.dj_persona
    args: Dict[str, Any] = {"action": "start", "theme": plan.play_name}
    if persona:
        args["persona"] = persona
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall(DJ_SET_TOOL, args),),
        say_ok=dj_started_text(persona, plan.play_name),
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
        say_ok=request_ok_text(command.name),
        say_fail=REQUEST_FAIL_TEXT,
        cancel_inflight=True,
        confirm_started=True,
    )


def _generate_plan(command: MediaCommand) -> MediaPlan:
    """«спой песенку про X» → ``generate_music(kind, theme)`` кодом (ADR-0150 §9.1, #3696). Песня пишется десятки
    секунд: фраза — ``source_message`` результата (ожидание/мост/отказ кода), об успехе — по ``started`` моста."""
    args: Dict[str, Any] = {"kind": command.kind}
    if command.name:
        args["theme"] = command.name
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall(GENERATE_MUSIC_TOOL, args),),
        say_ok=request_ok_text(""),
        say_fail=REQUEST_FAIL_TEXT,
        cancel_inflight=True,
        confirm_started=True,
    )


def plan_media_command(
    command: MediaCommand, media: MediaState, text: str = ""
) -> Optional[MediaPlan]:
    """Тулы и фраза для команды при данном состоянии плеера; ``text`` — слова человека."""
    if command.intent is MediaIntent.STOP:
        return _stop_plan(command, media)
    if command.intent is MediaIntent.DJ:
        return _dj_plan(command)
    if command.intent is MediaIntent.REQUEST_MUSIC:
        return _request_plan(command, text)
    if command.intent is MediaIntent.NOW_PLAYING:
        return _now_playing_plan(command, media)
    if command.intent is MediaIntent.GENERATE:
        return _generate_plan(command)
    if command.intent is MediaIntent.LIBRARY:
        return MediaPlan(command=command, library=command.kind, cancel_inflight=True)
    if command.intent is MediaIntent.PLAY_NAMED and command.name:
        plan = _play_named_plan(command)
        if command.themed:  # #3410: «давай тему X» посреди сета — смена темы, без lookup
            return dj_theme_switch_plan(plan, media) or plan
        return plan
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
    "GENERATE_MUSIC_TOOL",
    "default_media_provider",
    "dj_theme_switch_plan",
    "media_state_from_snapshot",
    "dj_started_text",
    "media_intent_rules",
    "plan_media_command",
]
