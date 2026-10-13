"""Модельные провайдеры выбора произведения за ``rob_box_music.choice.Chooser`` (ADR-0157 §2.2, §3.3, §4; PR-7 #3564).

``rob_box_music`` чистый (без сети), поэтому адаптер живёт здесь, рядом со сборкой :class:`PieceChain`:
:class:`ModelChooser` переводит music-``ChoiceQuestion`` (слоты заказа, недавнее, кандидаты с признаками) в вопрос
decision layer (спайк #3084: оркестратор — один дедлайн, circuit breaker, метрики) к провайдеру ``jev``
(``/v1/systemone``, ``criteria`` = описания кандидатов) или ``llm_harness`` (``chat/completions``, строгая схема с
``enum`` id) и возвращает ``Choice``. Ответ вне перечня, таймаут, ошибка, открытый circuit — :class:`ModelUnavailable`;
:class:`DeterministicChooser` ловит и отвечает сам: модель список кандидатов не расширяет.

Режим выбора - параметр ноды ``mcp_server.music_chooser`` (``deterministic``/``jev``/``chain``, умолчание ``chain``),
как цепочки провайдеров LLM/STT/TTS в yaml. Цепочка (решение Шифу 13.10 «Jev > LLM со спец-промптом > код с
коэффициентом и кубиком»): выбор ИСПОЛНЯЕТ :class:`ChainModel` — ``jev`` → ``llm_harness`` (тот же промпт V1) → код ``choice.dice``
(вероятность ∝ ``exp(score/T)``). У каждой модельной ступени свой дедлайн (Jev ``CHOOSER_DEADLINE_MS`` 900 мс, LLM
``CHOOSER_LLM_DEADLINE_MS`` 2 с) и свой circuit breaker (свой оркестратор); таймаут, ошибка, ответ вне кандидатов — ступень провалилась, дальше следующая. Одна строка
на выбор: ``[chooser] q=… stage=jev|llm|code choice=… ms_jev=… ms_llm=… reason_fallback=…``. Однозначный вопрос
(разрыв ≥ ``CHOOSER_CLEAR_GAP``) цепочка не видит — отвечает ``DeterministicChooser`` сразу (§3.3).

Shadow (``MUSIC_CHOOSER_SHADOW=jev|llm_harness``): исполняется ДЕТЕРМИНИРОВАННЫЙ выбор, модель спрашивается на тех же
неоднозначных вопросах и пишется в лог ``[chooser-shadow] q=… det=… model=… agree=… ms=…``. Без ключа — выключено с
одной строкой в лог. Повод включить — сравнение «модель против кода» на живых заказах до решения Go (§4).
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import logging
import random
import time
from typing import Any, Callable, List, Mapping, Optional, Sequence, Tuple

from rob_box_harness.decision import (
    ChoiceAnswer,
    ClassRoute,
    DecisionClass,
    DecisionOrchestrator,
    DecisionProvider,
    RoutingConfig,
    jev_from_env,
)
from rob_box_harness.decision import ChoiceQuestion as ModelQuestion
from rob_box_harness.decision.chat_choice import chat_choice_from_env
from rob_box_music.choice import (Choice, ChoiceQuestion, Chooser, DeterministicChooser, Option, dice, penalty, points,
                                  score)
from rob_box_music.knowledge import CHOOSER_DEADLINE_MS, CHOOSER_LLM_DEADLINE_MS, CHOOSER_WEIGHTS, HOOK_FRESH_SETS

_LOG = logging.getLogger(__name__)
SHADOW_ENV = "MUSIC_CHOOSER_SHADOW"
#: Режим выбора - параметр ноды ``mcp_server.music_chooser`` (yaml, как ``llm_providers`` у диалога и
#: ``stt_provider_chain`` у STT); env для него нет. Единая таблица: режим → провайдеры ступеней по порядку.
DETERMINISTIC_MODE = "deterministic"
JEV_MODE = "jev"
CHAIN_MODE = "chain"
CHOOSER_MODES: Mapping[str, Tuple[str, ...]] = {DETERMINISTIC_MODE: (), JEV_MODE: ("jev",),
                                                CHAIN_MODE: ("jev", "llm_harness")}
DEFAULT_MODE = CHAIN_MODE
#: Провайдер → имя ступени в строке лога и её дедлайн (Jev 900 мс, LLM 2 с - решения Шифу 13.10).
STAGE_LABELS = {"jev": "jev", "llm_harness": "llm"}
STAGE_DEADLINE_MS = {"jev": CHOOSER_DEADLINE_MS, "llm_harness": CHOOSER_LLM_DEADLINE_MS}
PROVIDERS = ("jev", "llm_harness")
QUESTION_ID = "pick"
_W = CHOOSER_WEIGHTS
#: Промпт V1 (замер координатора на живом Jev 12.10: 6/6 случаев, у V0 «Токкату играли 2 раза» проваливался): очки
#: кода в одной шкале, лучший - с наибольшим итогом; числа - из ``CHOOSER_WEIGHTS``.
INSTRUCTIONS = (f"Выбери, что сыграть роботу. У кандидатов - очки кода в одной шкале: совпадение названия "
                f"+{_W['title_match']:g}, композитора +{_W['composer_match']:g}, штраф за недавний повтор "
                f"(до −{abs(_W['recent']):g}). Обычно лучший - с наибольшим итогом; отклоняйся, только если явно "
                f"видишь, что другой кандидат лучше подходит к заказу. Выбери один id.")
#: Подписи очков кода для модели (ключи ``CHOOSER_WEIGHTS``).
_POINTS = {"title_match": "название", "composer_match": "композитор", "fit": "подходит",
           "rating": "рейтинг", "recent": "повтор", "known": "известное", "long": "длинная форма"}


class ModelUnavailable(Exception):
    """Модель не дала годный выбор за дедлайн (причина — в тексте); вызывающий отвечает детерминированно."""


def _times(count: int) -> str:
    return f"{count} раз" if count % 100 in range(11, 15) or count % 10 not in (2, 3, 4) else f"{count} раза"


def played_text(option: Option) -> str:
    """Что модели известно о повторах кандидата - человеческим текстом (штраф - тот, что вычитает код, из
    ``HOOK_FRESH_SETS`` последних исполнений; счётчик - за ``PLAY_COUNT_WINDOW_SEC``); пусто - истории нет."""
    plays = option.plays
    if plays is None:
        return ""
    if not plays.count:
        return "ещё не играли"
    if penalty(option.features):
        return f"уже играли {_times(plays.count)}, последний раз {plays.last_ago_min} мин назад"
    return (f"играли {_times(plays.count)} за сутки, но не среди последних {HOOK_FRESH_SETS} исполнений - "
            f"штрафа нет")


def points_text(option: Option) -> str:
    """Очки кода кандидата в одной шкале: «очки кода: название +3, композитор +2, повтор −2; итого 3»."""
    parts = [f"{_POINTS.get(name, name)} {'+' if value > 0 else '−'}{round(abs(value), 3):g}"
             for name, value in sorted(points(option.features).items(), key=lambda nv: nv[0] == "recent") if value]
    return f"очки кода: {', '.join(parts) or '0'}; итого {round(score(option.features), 3):g}"


def describe(option: Option) -> str:
    """Подпись кандидата для модели: «название - композитор; очки кода: …; итого N; сыгранное»."""
    return "; ".join(p for p in (option.label, points_text(option), played_text(option)) if p)


def session_text(question: ChoiceQuestion) -> str:
    """Сыгранное за сессию по кандидатам вопроса: «название ×N, …» или «ничего»."""
    seen = [f"{o.label or o.id} ×{o.plays.count}" for o in question.options if o.plays and o.plays.count]
    return ", ".join(seen) or "ничего"


def plays_note(question: ChoiceQuestion) -> str:
    """Хвост строки shadow: ``id=сыграно/штраф`` для сыгранных кандидатов вопроса (что видела модель)."""
    seen = [f"{o.id}={o.plays.count}/" + (f"-{penalty(o.features):g}" if penalty(o.features) else "0")
            for o in question.options if o.plays and o.plays.count]
    return f" plays=[{' '.join(seen)}]" if seen else " plays=[]"


def to_model_state(question: ChoiceQuestion) -> str:
    return f"Заказ: «{question.text}». За сессию сыграно: {session_text(question)}"


def to_model_question(question: ChoiceQuestion) -> ModelQuestion:
    ids = question.ids()
    return ModelQuestion(INSTRUCTIONS, ids, {o.id: describe(o) for o in question.options})


def build_provider(kind: str, env: Optional[Mapping[str, str]] = None, *,
                   timeout_s: Optional[float] = None, **kwargs: Any) -> Optional[DecisionProvider]:
    """Провайдер по имени из env; нет ключа/конфига — ``None``. HTTP-таймаут — дедлайн ступени (короче — ступень
    обрывалась бы раньше своего дедлайна)."""
    if kind in STAGE_DEADLINE_MS and timeout_s is None:
        timeout_s = STAGE_DEADLINE_MS[kind] / 1000.0
    if kind == "jev":
        return jev_from_env(env, timeout_s=timeout_s, **kwargs)
    if kind == "llm_harness":
        return chat_choice_from_env(env, timeout_s=timeout_s, **kwargs)
    raise ValueError(f"неизвестный провайдер выбора {kind!r}; есть {PROVIDERS}")


def run_sync(coro: Any) -> Any:
    """Корутину — в синхронный вызов: своя петля; если в потоке уже крутится петля — отдельный поток."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(coro)
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
        return pool.submit(asyncio.run, coro).result()


class ModelChooser:
    """``Chooser`` над провайдером decision layer: дедлайн, circuit breaker и метрики — оркестратора."""

    def __init__(self, provider: DecisionProvider, *, deadline_ms: int = CHOOSER_DEADLINE_MS,
                 orchestrator: Optional[DecisionOrchestrator] = None) -> None:
        self.name = provider.name
        self._deadline_ms = deadline_ms
        self.orchestrator = orchestrator or DecisionOrchestrator({provider.name: provider},
                                                                 routing=self._routing(deadline_ms))

    def _routing(self, deadline_ms: int) -> RoutingConfig:
        route = ClassRoute((self.name,), deadline_ms, 0.0)
        return RoutingConfig({c: route for c in DecisionClass})

    def choose(self, question: ChoiceQuestion, *, deadline_ms: int = CHOOSER_DEADLINE_MS) -> Choice:
        asked = to_model_question(question)
        if deadline_ms != self._deadline_ms:
            self._deadline_ms = deadline_ms
            self.orchestrator.set_routing(self._routing(deadline_ms))
        ids = question.ids()
        decision = run_sync(self.orchestrator.decide(
            DecisionClass.QUALITY_CRITICAL, to_model_state(question), {QUESTION_ID: asked},
            fallback=lambda: {QUESTION_ID: ChoiceAnswer(ids[0], {ids[0]: 1.0}, 0.0)}))
        if decision.fallback_used:
            raise ModelUnavailable(decision.fallback_reason or "fallback")
        answer = decision.answers[QUESTION_ID]
        return Choice(answer.answer, float(answer.confidence), self.name)


class ShadowModel:
    """Модель «в тени»: спрашивает ``model``, пишет сравнение в лог, а исполняется детерминированный ответ."""

    def __init__(self, model: Chooser, log: Callable[[], Any] = lambda: _LOG,
                 clock: Callable[[], float] = time.perf_counter) -> None:
        self._model = model
        self._log = log
        self._clock = clock
        self._det = DeterministicChooser()

    def choose(self, question: ChoiceQuestion, *, deadline_ms: int = CHOOSER_DEADLINE_MS) -> Choice:
        det = self._det.choose(question)
        started = self._clock()
        try:
            said = self._model.choose(question, deadline_ms=deadline_ms).id
            seen = said if said in question.ids() else f"OUT({said})"
        except Exception as exc:  # noqa: BLE001 — тень не должна влиять на ответ
            seen = f"ERR({type(exc).__name__}: {exc})"
        ms = (self._clock() - started) * 1000.0
        self._log().info(f"🪞 [chooser-shadow] q={question.text!r} det={det.id} model={seen} "
                         f"agree={'yes' if seen == det.id else 'no'} ms={ms:.0f}{plays_note(question)}")
        return det


class ChainModel:
    """Цепочка исполнения: модельные ступени по порядку, затем кубик кода (всегда отвечает, не бросает)."""

    def __init__(self, stages: Sequence[Tuple[str, Chooser]], log: Callable[[], Any] = lambda: _LOG,
                 clock: Callable[[], float] = time.perf_counter, rng: Any = None,
                 deadlines: Optional[Mapping[str, int]] = None) -> None:
        self._stages = list(stages)
        self._deadlines = dict(deadlines or {})
        self._log = log
        self._clock = clock
        self._rng = rng if rng is not None else random.Random()

    def choose(self, question: ChoiceQuestion, *, deadline_ms: int = CHOOSER_DEADLINE_MS) -> Choice:
        timings: List[str] = []
        reasons: List[str] = []
        for label, stage in self._stages:
            started = self._clock()
            answer, why = self._ask(stage, question, self._deadlines.get(label, deadline_ms))
            timings.append(f"ms_{label}={(self._clock() - started) * 1000.0:.0f}")
            if answer is not None:
                return self._note(question, label, answer, timings, reasons)
            reasons.append(f"{label}:{why}")
        return self._note(question, "code", dice(question, self._rng), timings, reasons)

    @staticmethod
    def _ask(stage: Chooser, question: ChoiceQuestion, deadline_ms: int) -> Tuple[Optional[Choice], str]:
        try:
            answer = stage.choose(question, deadline_ms=deadline_ms)
        except Exception as exc:  # noqa: BLE001 — ступень не держит выбор: дальше следующая
            return None, str(exc) or type(exc).__name__
        if answer is None or answer.id not in question.ids():
            return None, f"out({answer.id if answer else None})"
        return answer, ""

    def _note(self, question: ChoiceQuestion, stage: str, answer: Choice, timings: List[str],
              reasons: List[str]) -> Choice:
        self._log().info(f"🎲 [chooser] q={question.text!r} stage={stage} choice={answer.id} "
                         f"conf={answer.confidence:.2f} {' '.join(timings) or 'ms=0'} "
                         f"reason_fallback={','.join(reasons) or '-'}{plays_note(question)}")
        return answer


def chain_from_env(mode: str, env: Mapping[str, str], log: Callable[[], Any],
                   factory: Callable[..., Optional[DecisionProvider]]) -> DeterministicChooser:
    """Режим ``jev``/``chain``: ступени, для которых в env есть ключ/конфиг; нет ни одной — один кубик кода."""
    stages, missing = [], []
    for kind in CHOOSER_MODES[mode]:
        provider = factory(kind, env)
        if provider is None:
            missing.append(kind)
        else:
            stages.append((STAGE_LABELS[kind], ModelChooser(provider, deadline_ms=STAGE_DEADLINE_MS[kind])))
    names = " → ".join([label for label, _s in stages] + ["code"])
    deadlines = {STAGE_LABELS[k]: STAGE_DEADLINE_MS[k] for k in CHOOSER_MODES[mode]}
    note = f"; без ключа/конфига: {', '.join(missing)}" if missing else ""
    (log().warning if missing else log().info)(f"🎲 [chooser] режим {mode}: {names}{note}")
    return DeterministicChooser(ChainModel(stages, log, deadlines=deadlines))


def chooser_from_env(mode: str = DEFAULT_MODE, env: Optional[Mapping[str, str]] = None,
                     log: Callable[[], Any] = lambda: _LOG,
                     factory: Callable[..., Optional[DecisionProvider]] = build_provider) -> DeterministicChooser:
    """Выбиратель сборки по режиму ``mcp_server.music_chooser`` (``deterministic``/``jev``/``chain``; по умолчанию
    цепочка Jev → LLM → кубик). Ключи провайдеров - из env. ``MUSIC_CHOOSER_SHADOW=<провайдер>`` - тень модели при
    детерминированном режиме (отладка). Неизвестный режим / нет ключа - детерминированный (или ступень пропущена)
    и одна строка в лог."""
    import os

    env = os.environ if env is None else env
    mode = (mode or "").strip().lower() or DEFAULT_MODE
    if mode in (JEV_MODE, CHAIN_MODE):
        return chain_from_env(mode, env, log, factory)
    if mode != DETERMINISTIC_MODE:
        log().warning(f"🎲 [chooser] music_chooser={mode!r} неизвестен (есть {', '.join(CHOOSER_MODES)}) — "
                      f"детерминированный")
    kind = env.get(SHADOW_ENV, "").strip().lower()
    if not kind:
        return DeterministicChooser()
    try:
        provider = factory(kind, env)
    except ValueError as exc:
        log().warning(f"🪞 [chooser-shadow] выключен: {exc}")
        return DeterministicChooser()
    if provider is None:
        log().warning(f"🪞 [chooser-shadow] выключен: для {kind} не задан ключ/конфиг в env")
        return DeterministicChooser()
    log().info(f"🪞 [chooser-shadow] включён: {kind} (исполняется детерминированный выбор)")
    return DeterministicChooser(ShadowModel(ModelChooser(provider), log))


__all__ = ["CHAIN_MODE", "CHOOSER_MODES", "ChainModel", "DEFAULT_MODE", "DETERMINISTIC_MODE", "JEV_MODE",
           "STAGE_DEADLINE_MS", "STAGE_LABELS", "ModelChooser",
           "ModelUnavailable", "PROVIDERS", "SHADOW_ENV", "ShadowModel", "build_provider", "chain_from_env",
           "chooser_from_env", "describe", "played_text", "plays_note", "points_text", "run_sync", "session_text",
           "to_model_question",
           "to_model_state"]
