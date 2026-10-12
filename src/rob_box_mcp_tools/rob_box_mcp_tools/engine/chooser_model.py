"""Модельные провайдеры выбора произведения за ``rob_box_music.choice.Chooser`` (ADR-0157 §2.2, §3.3, §4; PR-7 #3564).

``rob_box_music`` чистый (без сети), поэтому адаптер живёт здесь, рядом со сборкой :class:`PieceChain`:
:class:`ModelChooser` переводит music-``ChoiceQuestion`` (слоты заказа, недавнее, кандидаты с признаками) в вопрос
decision layer (спайк #3084: оркестратор — один дедлайн, circuit breaker, метрики) к провайдеру ``jev``
(``/v1/systemone``, ``criteria`` = описания кандидатов) или ``llm_harness`` (``chat/completions``, строгая схема с
``enum`` id) и возвращает ``Choice``. Ответ вне перечня, таймаут, ошибка, открытый circuit — :class:`ModelUnavailable`;
:class:`DeterministicChooser` ловит и отвечает сам: модель список кандидатов не расширяет.

Shadow (``MUSIC_CHOOSER_SHADOW=jev|llm_harness``): исполняется ДЕТЕРМИНИРОВАННЫЙ выбор, модель спрашивается на тех же
неоднозначных вопросах и пишется в лог ``[chooser-shadow] q=… det=… model=… agree=… ms=…``. Без ключа — выключено с
одной строкой в лог. Повод включить — сравнение «модель против кода» на живых заказах до решения Go (§4).
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import logging
import time
from typing import Any, Callable, Mapping, Optional

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
from rob_box_music.choice import Choice, ChoiceQuestion, Chooser, DeterministicChooser, Option, penalty, points, score
from rob_box_music.knowledge import CHOOSER_DEADLINE_MS, CHOOSER_WEIGHTS, HOOK_FRESH_SETS

_LOG = logging.getLogger(__name__)
SHADOW_ENV = "MUSIC_CHOOSER_SHADOW"
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
           "rating": "рейтинг", "recent": "повтор"}


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


def build_provider(kind: str, env: Optional[Mapping[str, str]] = None, *, timeout_s: float = 0.6,
                   **kwargs: Any) -> Optional[DecisionProvider]:
    """Провайдер по имени из env; нет ключа/конфига — ``None``."""
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


def chooser_from_env(env: Optional[Mapping[str, str]] = None, log: Callable[[], Any] = lambda: _LOG,
                     factory: Callable[..., Optional[DecisionProvider]] = build_provider) -> DeterministicChooser:
    """Выбиратель сборки: по умолчанию детерминированный; ``MUSIC_CHOOSER_SHADOW=<провайдер>`` — с тенью модели.
    Нет ключа / неизвестное имя — детерминированный и одна строка в лог."""
    import os

    env = os.environ if env is None else env
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


__all__ = ["ModelChooser", "ModelUnavailable", "PROVIDERS", "SHADOW_ENV", "ShadowModel", "build_provider",
           "chooser_from_env", "describe", "played_text", "plays_note", "points_text", "run_sync", "session_text",
           "to_model_question",
           "to_model_state"]
