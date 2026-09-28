"""Scheduler PoC (issue #3084 §C) в shadow-режиме.

Почему shadow, а не «Jev решает»: ``docs/design/SCHEDULER_DESIGN.md §4.7``
(ревизия v5, решение owner'а) явно убрал «уровень 2» — второй модельный
вызов на пути классификации ввода (доп. RTT, двойная стоимость,
рассинхрон с основной LLM). Пока Шифу не пересмотрит §4.7, модель здесь
может только ОТВЕТИТЬ, а исполняется всегда детерминированный вердикт
``quick_decide``. Паттерн взят из ``jev-skill-router`` (awesome-jev):
«starts in a shadow mode that only logs the decision».

Модуль не импортирует ``rob_box_voice``: вердикт правил передаётся
строкой, варианты — закрытый набор ``QUICK_VERDICTS`` (значения
``rob_box_voice.scheduler.quick_decide.QuickVerdict``). Никаких
side-effect'ов: функция возвращает данные, action не вызывает.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .orchestrator import DecisionOrchestrator
from .types import ChoiceAnswer, ChoiceQuestion, Decision, DecisionClass

QUICK_VERDICTS: tuple[str, ...] = ("IGNORE", "REPLACE", "PENDING_LLM")

VERDICT_QUESTION = ChoiceQuestion(
    "Как планировщику обработать новую фразу пользователя? IGNORE — шум, "
    "междометие, эхо; REPLACE — явная команда прервать текущее; "
    "PENDING_LLM — всё остальное, решает основная LLM.",
    QUICK_VERDICTS,
)


@dataclass(frozen=True)
class ShadowVerdict:
    executed: str  # всегда вердикт правил
    model_verdict: str | None  # None — модель не ответила (fallback)
    agreed: bool | None
    decision: Decision


async def shadow_quick_verdict(
    orchestrator: DecisionOrchestrator,
    *,
    utterance: str,
    baseline_verdict: str,
    context: Mapping[str, Any] | None = None,
) -> ShadowVerdict:
    """Спросить модель о вердикте, но исполнить ``baseline_verdict``."""
    if baseline_verdict not in QUICK_VERDICTS:
        raise ValueError(f"unknown verdict {baseline_verdict!r}")
    state = {"utterance": utterance, **dict(context or {})}

    def fallback() -> Mapping[str, ChoiceAnswer]:
        return {"verdict": ChoiceAnswer(baseline_verdict, {baseline_verdict: 1.0}, 1.0)}

    decision = await orchestrator.decide(
        DecisionClass.LATENCY_CRITICAL, state, {"verdict": VERDICT_QUESTION}, fallback=fallback
    )
    if decision.fallback_used:
        return ShadowVerdict(baseline_verdict, None, None, decision)
    model_verdict = decision.answers["verdict"].answer  # type: ignore[union-attr]
    return ShadowVerdict(baseline_verdict, model_verdict, model_verdict == baseline_verdict, decision)
