"""Спорный стоп (ADR-0150 §18.3 В19, решение Шифу 20.10 «гибрид»): Chooser ``{stop, part_of_command}``.

Ясный стоп («робот, стоп», «всё, хватит, спой другую») решает код — :func:`rob_box_dialog.grammar.match`, 0 вызовов
LLM. Спорный — стоп-слово внутри другой просьбы («спой песню про стоп», «покажи анимацию стоп»,
:func:`rob_box_dialog.words.stop_verdict` → ``disputed``) — решает модель за ``STOP_CHOOSER_DEADLINE_MS``; таймаут,
ошибка, нет ключа — ``part_of_command``: робот не замолкает от слова внутри просьбы, речь до решения не рвётся.

Модель — общий ``ModelChooser`` выбора музыки (``rob_box_mcp_tools.engine.chooser_model``: дедлайн, circuit breaker
и метрики оркестратора decision layer), провайдер — ``llm_harness`` из env (по умолчанию DeepSeek). Одна строка на
решение: ``🤔 [stop-chooser] verdict=… stage=model|timeout|off ms=… text=…`` (доля спорных — ``turn_metrics.py``).
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Mapping, Optional, Tuple

from rob_box_dialog.knowledge import STOP_CHOICES, STOP_CHOOSER_DEADLINE_MS
from rob_box_music.choice import Chooser, ChoiceQuestion, Option

_LOG = logging.getLogger(__name__)
STOP, PART_OF_COMMAND = STOP_CHOICES
INSTRUCTIONS = (
    "Человек сказал роботу фразу со словом остановки («стоп», «стой», «хватит», «замолчи»). Реши, что это: приказ "
    "роботу остановиться или замолчать (stop) — или слово внутри другой просьбы: название, тема, цитата "
    "(part_of_command). «Спой песню про стоп», «покажи анимацию стоп», «найди песню Stop» — part_of_command; "
    "«стоп, хватит петь», «замолчи уже» — stop. Выбери один id."
)
_LABELS: Mapping[str, str] = {
    STOP: "приказ роботу остановиться или замолчать",
    PART_OF_COMMAND: "слово остановки — часть другой просьбы (название, тема, цитата), робот выполняет просьбу",
}


def question(text: str) -> ChoiceQuestion:
    """Вопрос модели: фраза человека и два варианта; весов нет — это не выбор пьесы."""
    options = tuple(Option(choice, {}, _LABELS[choice]) for choice in STOP_CHOICES)
    return ChoiceQuestion(text, options, weights={}, instructions=INSTRUCTIONS)


def state_text(q: ChoiceQuestion) -> str:
    return f"Фраза человека роботу: «{q.text}»"


class StopChooser:
    """``decide(text) -> (stop|part_of_command, stage)``; без модели — всегда ``part_of_command`` (stage ``off``)."""

    def __init__(self, model: Optional[Chooser], deadline_ms: int = STOP_CHOOSER_DEADLINE_MS,
                 log: Callable[[], Any] = lambda: _LOG, clock: Callable[[], float] = time.perf_counter) -> None:
        self._model, self._deadline_ms, self._log, self._clock = model, deadline_ms, log, clock

    def decide(self, text: str) -> Tuple[str, str]:
        started = self._clock()
        verdict, stage = PART_OF_COMMAND, "off"
        if self._model is not None:
            try:
                answer = self._model.choose(question(text), deadline_ms=self._deadline_ms)
                verdict, stage = (answer.id, "model") if answer.id in STOP_CHOICES else (PART_OF_COMMAND, "out")
            except Exception as exc:  # noqa: BLE001 — таймаут/ошибка модели: робот не замолкает
                stage = f"timeout:{type(exc).__name__}"
        ms = (self._clock() - started) * 1000.0
        self._log().info(f"🤔 [stop-chooser] verdict={verdict} stage={stage} ms={ms:.0f} text={text[:60]!r}")
        return verdict, stage


def stop_chooser_from_env(env: Optional[Mapping[str, str]] = None,
                          log: Callable[[], Any] = lambda: _LOG) -> StopChooser:
    """Chooser над ``llm_harness`` (ключ из env); нет ключа — без модели и одна строка в лог.

    ``rob_box_mcp_tools`` — объявленная зависимость (``package.xml``); импорт здесь, а не наверху модуля, чтобы
    импорт ``dialogue_node`` не тянул сетевой слой decision (httpx) — его зовут только при сборке ноды.
    """
    from rob_box_mcp_tools.engine.chooser_model import ModelChooser, build_provider  # noqa: PLC0415

    provider = build_provider("llm_harness", env, timeout_s=STOP_CHOOSER_DEADLINE_MS / 1000.0)
    if provider is None:
        log().warning("🤔 [stop-chooser] нет ключа/конфига llm_harness — спорный стоп = part_of_command")
        return StopChooser(None, log=log)
    log().info(f"🤔 [stop-chooser] модель llm_harness, дедлайн {STOP_CHOOSER_DEADLINE_MS} мс")
    return StopChooser(ModelChooser(provider, deadline_ms=STOP_CHOOSER_DEADLINE_MS, state=state_text), log=log)


__all__ = ["INSTRUCTIONS", "PART_OF_COMMAND", "STOP", "StopChooser", "question", "state_text",
           "stop_chooser_from_env"]
