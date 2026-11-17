"""Acceptance PoC (issue #3084 §D): модельный сигнал может только УЖЕСТОЧИТЬ.

Чистая функция поверх результата :meth:`ToolConfirmationPolicy.classify`.
Правила, которые нельзя ослабить никаким ответом модели (или его
отсутствием):

1. ``EMERGENCY_TOOLS`` (``stop_navigation``) возвращаются как есть —
   ноль зависимости от Jev: функция даже не смотрит на сигнал.
2. Решение ``require`` никогда не понижается до ``notify``/``pass_through``.
3. Нет сигнала (Jev недоступен, fallback) → исходное решение политики.
4. Повышение до ``require`` — только при ``needs_review >= threshold``.

Функция синхронная и не ходит в сеть: :class:`AcceptanceGate.submit`
синхронный, и удалённый вызов в нём на горячем пути недопустим. Сигнал
считается заранее (асинхронно, с дедлайном) и передаётся сюда числом.
"""

from __future__ import annotations

from ..core.confirmation_policy import EMERGENCY_TOOLS, AcceptanceDecision, ConfirmationKind

#: Консервативный порог «нужен human review». Как и все пороги — до
#: калибровки на наших размеченных сценариях это догадка, не измерение.
DEFAULT_REVIEW_THRESHOLD = 0.5

_RANK = {
    ConfirmationKind.PASS_THROUGH: 0,
    ConfirmationKind.NOTIFY: 1,
    ConfirmationKind.REQUIRE: 2,
}


def escalate_acceptance(
    decision: AcceptanceDecision,
    needs_review_probability: float | None,
    *,
    threshold: float = DEFAULT_REVIEW_THRESHOLD,
    plan_text: str | None = None,
) -> AcceptanceDecision:
    """Вернуть решение не мягче ``decision`` с учётом модельного сигнала."""
    if decision.tool in EMERGENCY_TOOLS:
        return decision
    if needs_review_probability is None or needs_review_probability < threshold:
        return decision
    if _RANK[decision.kind] >= _RANK[ConfirmationKind.REQUIRE]:
        return decision
    return AcceptanceDecision(
        kind=ConfirmationKind.REQUIRE,
        tool=decision.tool,
        reason=(f"{decision.reason}; escalated: needs_review=" f"{needs_review_probability:.2f} >= {threshold:.2f}"),
        plan_text=plan_text or decision.plan_text or f"Выполнить {decision.tool}? Подтверждаешь?",
    )
