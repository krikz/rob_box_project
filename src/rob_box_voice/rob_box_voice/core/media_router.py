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

Модуль чистый: без ROS и без I/O. Исполнение — в ``DialogueNode``.
"""

from __future__ import annotations

import logging
import random
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

from rob_box_harness.decision import (
    ChoiceAnswer,
    ChoiceQuestion,
    DecisionProvider,
    DeterministicProvider,
)

from .dj_mode import CLUB_ROOTS, DJ_SET_DEFAULT_BPM
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
#: («СТАРТ ВЕЧЕРИНКИ»), когда роутер не запускает превью (над играющим
#: треком — переход всё равно ждёт конца его формы; открытый DJ-запрос —
#: музыку ставит LLM). 15 — нижний клэмп ``DJModeController`` /
#: ``SetDjModeTool``.
DJ_START_TRANSITION_SEC = 15

#: Issue #3153 (ADR-0142 «мгновенное превью») — длина одного прохода
#: клубной формы превью, с: все шаблоны ``style="club"`` — 32 такта,
#: при темпе сета 124 BPM это 32·4·60/124 ≈ 61,9 с
#: (``club_arranger.club_duration_seconds``). Переход #1 назначается на
#: конец формы превью; ``DJModeController.tick`` дополнительно гейтит его
#: реальным ``form_ends_at`` из ``/voice/music/form``.
DJ_PREVIEW_FORM_SEC = 62

#: Диапазон сида превью: ``seed=0`` — эталонный каркас club
#: (``club_arranger.REFERENCE_KIT``), его не берём — каждый сет свой.
DJ_PREVIEW_SEED_RANGE = (1, 9_999_999)


@dataclass(frozen=True)
class MediaState:
    """Снимок состояния плеера на момент реплики.

    Attributes:
        music_playing: играет ли музыка — снимок плеера
            (``_music_playing_now``, #3133 / ADR-0141).
        dj_enabled: идёт ли DJ-сет.
        track_name: название играющего трека, если известно.
        form_ends_at: issue #3153 (доп.) — конец текущего прохода формы
            играющего трека, epoch (``/voice/music/state`` → снимок плеера,
            то же поле, что ``DJState.form_ends_at``). ``None`` — данных
            нет (форма ещё не сыграна ни разу / топик не пришёл) или
            музыка не играет.
    """

    music_playing: bool = False
    dj_enabled: bool = False
    track_name: Optional[str] = None
    form_ends_at: Optional[float] = None


@dataclass(frozen=True)
class MediaToolCall:
    """Один MCP-тул плана.

    Attributes:
        fail_text: issue #3153 — если непусто и тул не сработал, план
            ОСТАНАВЛИВАЕТСЯ на этом туле и говорится эта фраза (вместо
            ``MediaPlan.say_fail``): следующий тул без предыдущего не имеет
            смысла, а общая фраза соврала бы о том, что успело случиться.
    """

    name: str
    arguments: Dict[str, Any] = field(default_factory=dict)
    fail_text: str = ""


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
        preview_root: issue #3153 — план запускает DJ-сет мгновенным
            club-превью в этой тонике (``""`` — превью нет или тоника
            неизвестна). Нода заранее сообщает DJ-контроллеру, что превью
            — трек #1 сета.
        claim_track_one: issue #3153 (доп.) — заявка «трек #1 сета»
            (``DJModeController.claim_preview``) нужна ДАЖЕ без своего
            ``compose_music``: «ты диджей X» поверх уже играющего
            обычного трека тоже считает его треком #1 (роутер тоник
            играющего трека не знает — заявка уходит с ``preview_root=""``,
            контроллер тогда просто не трогает тонику сета). ``False`` —
            план не связан с DJ-стартом, заявку звать не нужно.
        play_name: issue #3176 — заказ по имени: название мелодии словами
            юзера. Непусто — нода исполняет не ``tool_calls``, а поток
            :mod:`.named_play` (``lookup_melody`` → ``compose_music``):
            нашлась — играет и говорит :func:`.named_play.play_ok_text`;
            не нашлась — реплика уходит в LLM, роутер молчит.
    """

    command: MediaCommand
    tool_calls: Tuple[MediaToolCall, ...] = ()
    say_ok: str = ""
    say_fail: str = ""
    to_llm: bool = False
    dj_off: bool = False
    cancel_inflight: bool = False
    preview_root: str = ""
    claim_track_one: bool = False
    play_name: str = ""

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
#: Issue #3153 — превью не запустилось: DJ не включаем, говорим как есть.
DJ_PREVIEW_FAIL_TEXT = "Не получилось запустить музыку — диджей-сет не включил."
#: Превью играет, а ``set_dj_mode`` не прошёл: музыка есть, сета нет.
DJ_MODE_FAIL_AFTER_PREVIEW_TEXT = "Музыку включил, а диджей-сет не запустился."

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

    def __init__(
        self,
        provider: Optional[DecisionProvider] = None,
        rng: Optional[random.Random] = None,
    ) -> None:
        self._provider = provider or default_media_provider()
        # Issue #3153 — сид и тоника превью: свои на каждый сет. Тесты
        # подают сидированный ГСЧ.
        self._rng = rng or random.Random()

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
        return plan_media_command(command, media, preview=self._preview_pick())

    def _preview_pick(self) -> "DJPreview":
        low, high = DJ_PREVIEW_SEED_RANGE
        return DJPreview(
            seed=self._rng.randint(low, high),
            root=self._rng.choice(CLUB_ROOTS),
        )


@dataclass(frozen=True)
class DJPreview:
    """Issue #3153 — сид и тоника мгновенного club-превью DJ-сета."""

    seed: int
    root: str

    def compose_call(self) -> MediaToolCall:
        """``compose_music`` превью: зациклен, в темпе сета по умолчанию.

        ``transition="fade"`` (issue #3154): превью — трек #1 сета и входит
        так же, как любой DJ-переход (#3166): с первого блока полной бочки
        шаблона, а не с тихого интро. Из тишины ``wrap_with_fade`` фейда не
        делает — трек стартует сразу (``core/club_transition``). Живой
        прогон 29.09: превью ``long_build_32`` из тишины начиналось одной
        нотой пэда и 3,4 с цифровой тишины.
        """
        return MediaToolCall(
            "compose_music",
            {
                "style": "club",
                "repeat": True,
                "seed": self.seed,
                "root": self.root,
                "bpm": DJ_SET_DEFAULT_BPM,
                "transition": "fade",
            },
            fail_text=DJ_PREVIEW_FAIL_TEXT,
        )


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


def _dj_args(command: MediaCommand) -> Dict[str, Any]:
    args: Dict[str, Any] = {"enabled": True}
    if command.persona:
        args["persona"] = command.persona
    if command.theme:
        args["theme"] = command.theme
    return args


def _dj_preview_plan(
    command: MediaCommand, media: MediaState, preview: Optional[DJPreview]
) -> Optional[MediaPlan]:
    """Issue #3153 — «ты диджей X» в ТИШИНЕ: звук сразу, без хода LLM.

    Живой прогон 28.09: роутер включал DJ с переходом через 15 с, переход
    #1 («СТАРТ ВЕЧЕРИНКИ») гнал модель в search_web / search_samples /
    gen_search_library и план — первый звук через ~50 с после «запускаю
    сет». Теперь (ADR-0142 §7, «мгновенное превью»):

    1. ``compose_music(style="club", repeat=True, seed=<свой>)`` — первым:
       звук не ждёт ничего, кроме этого вызова. Не сработал — DJ не
       включаем и честно говорим, что музыки нет.
    2. ``set_dj_mode(..., next_transition_sec=DJ_PREVIEW_FORM_SEC)`` —
       переход #1 на конце формы превью. Не сработал — музыка играет, а
       сета нет, так и говорим.

    Превью — трек #1 сета: нода до вызовов сообщает об этом
    DJ-контроллеру (``preview_root``), и переход #1 играет трек #2 без
    исследования (``DJModeController.build_auto_prompt``). Порядок
    «compose, потом set_dj_mode» на счёт треков не влияет — счёт ставит
    не ход LLM, а заявка ноды, которую забирает генуинный старт сета.

    Только закрытая команда: открытый DJ-запрос («…сыграй Still Dre и
    Next Episode») идёт в LLM, и музыку запускает она — превью поверх
    заказа юзера было бы лишним треком.
    """
    if preview is None or media.music_playing or media.dj_enabled or not command.closed:
        return None
    who = command.persona
    args = _dj_args(command)
    args["next_transition_sec"] = DJ_PREVIEW_FORM_SEC
    return MediaPlan(
        command=command,
        tool_calls=(
            preview.compose_call(),
            MediaToolCall(
                "set_dj_mode", args, fail_text=DJ_MODE_FAIL_AFTER_PREVIEW_TEXT
            ),
        ),
        say_ok=f"Я {who}, запускаю сет." if who else DJ_START_TEXT,
        say_fail=DJ_FAIL_TEXT,
        cancel_inflight=True,
        preview_root=preview.root,
        claim_track_one=True,
    )


def _over_playing_transition_sec(media: MediaState, *, now: Optional[float] = None) -> int:
    """Issue #3153 (доп.) — секунды до перехода #1 над уже играющим треком.

    До конца текущей формы (``media.form_ends_at``), если она известна —
    тикер (``DJModeController.tick``) всё равно гейтит переход реальным
    ``form_ends_at``, так что более раннее число здесь только НАЗНАЧАЕТ
    будильник, а не укорачивает играющий трек. Форма ещё не известна (топик
    ``/voice/music/state`` не донёс ``form_ends_at``) или уже в прошлом —
    прежний дефолт :data:`DJ_START_TRANSITION_SEC`.
    """
    if media.form_ends_at is None:
        return DJ_START_TRANSITION_SEC
    moment = time.time() if now is None else now
    remaining = media.form_ends_at - moment
    if remaining <= 0:
        return DJ_START_TRANSITION_SEC
    return int(round(remaining))


def _dj_over_playing_plan(
    command: MediaCommand, media: MediaState
) -> Optional[MediaPlan]:
    """Issue #3153 (доп.) — «ты диджей X» поверх уже играющего ОБЫЧНОГО трека.

    Живой прогон 28.09.2026 22:30 (деплой develop ``8888d6f02``): музыка уже
    играла — превью роутера (:func:`_dj_preview_plan`) не запускается
    (``media.music_playing``), роутер ставил только
    ``set_dj_mode(next_transition_sec=DJ_START_TRANSITION_SEC)``, а
    стартовый переход #1 DJ-тикера всё равно шёл по ветке «СТАРТ
    ВЕЧЕРИНКИ» (``search_web`` / ``search_samples`` / ``gen_search_library``)
    — LLM ответила текстом, сработал Bug B retry.

    Играющий трек засчитывается треком #1 сета — тем же механизмом, что и
    мгновенное превью (:meth:`DJModeController.claim_preview`), только без
    своего ``compose_music`` (трек уже звучит): переход #1 идёт через
    ``_after_preview_prompt`` (без исследования). Тонику играющего трека
    роутер не знает (только имя, не тональность) — заявка уходит с
    ``preview_root=""``, контроллер тогда просто не трогает тонику сета
    (``related_root`` останется от дефолта). ``next_transition_sec`` — до
    конца текущей формы, если она известна (:func:`_over_playing_transition_sec`).

    Идущий DJ-сет (``media.dj_enabled``) сюда не попадает — там прежний
    путь (смена персоны/темы без остановки). Открытый DJ-запрос
    (``not command.closed``) тоже — музыку в этом случае ставит LLM, а не
    код, и заявка на «уже играющий трек» была бы лишней.
    """
    if media.dj_enabled or not media.music_playing or not command.closed:
        return None
    args = _dj_args(command)
    args["next_transition_sec"] = _over_playing_transition_sec(media)
    who = command.persona
    ok_text = f"Я {who}, подхватываю сет." if who else DJ_START_TEXT
    return MediaPlan(
        command=command,
        tool_calls=(MediaToolCall("set_dj_mode", args),),
        say_ok=ok_text,
        say_fail=DJ_FAIL_TEXT,
        cancel_inflight=True,
        preview_root="",
        claim_track_one=True,
    )


def _dj_plan(
    command: MediaCommand, media: MediaState, preview: Optional[DJPreview] = None
) -> MediaPlan:
    preview_plan = _dj_preview_plan(command, media, preview)
    if preview_plan is not None:
        return preview_plan
    over_playing_plan = _dj_over_playing_plan(command, media)
    if over_playing_plan is not None:
        return over_playing_plan
    args = _dj_args(command)
    if not media.dj_enabled:
        # Сюда доходит только тишина без превью (``preview is None`` —
        # тесты/деградация) и открытый DJ-запрос (музыку ставит LLM):
        # играющий обычный трек и закрытое превью в тишине разобраны выше
        # (:func:`_dj_over_playing_plan`, :func:`_dj_preview_plan`). Сет
        # стартует переходом #1 DJ-тикера через прежний дефолт.
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


def _play_named_plan(command: MediaCommand) -> MediaPlan:
    """Issue #3176 — заказ по имени: исполняет нода, решает база мелодий.

    Состояние плеера плану не нужно: заказ играет и в тишине, и поверх
    трека, и посреди DJ-сета (там — как заказ гостя, сет не гасится; см.
    ``DialogueNode._execute_play_named``). ``cancel_inflight`` — ``False``:
    идущий ход LLM отменяется только когда мелодия нашлась; не нашлась —
    реплика проходит оставшиеся шаги приёма (barge-in) как обычная.
    """
    return MediaPlan(command=command, play_name=command.name)


def plan_media_command(
    command: MediaCommand,
    media: MediaState,
    preview: Optional[DJPreview] = None,
) -> Optional[MediaPlan]:
    """Тулы и фраза для команды при данном состоянии плеера.

    ``preview`` — сид/тоника club-превью для старта DJ-сета в тишине
    (issue #3153); ``None`` — превью не запускать (старый путь).
    """
    if command.intent in _VOLUME_ACTIONS:
        return _volume_plan(command, media)
    if command.intent is MediaIntent.STOP:
        return _stop_plan(command, media)
    if command.intent is MediaIntent.DJ:
        return _dj_plan(command, media, preview)
    if command.intent is MediaIntent.PLAY_NAMED and command.name:
        return _play_named_plan(command)
    return None


__all__ = [
    "DJ_MODE_FAIL_AFTER_PREVIEW_TEXT",
    "DJ_PREVIEW_FAIL_TEXT",
    "DJ_PREVIEW_FORM_SEC",
    "DJ_START_TRANSITION_SEC",
    "DJPreview",
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
