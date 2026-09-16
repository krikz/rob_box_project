"""Confirmation-gate policy extracted from ``AgentCore._run_with_tools``.

Issue #2630 PR-A — выделить ~10 CC из основного метода.

Когда :class:`rob_box_harness.core.acceptance.AcceptanceGate` классифицирует
вызов как :attr:`ConfirmationKind.REQUIRE`, мы не зовём executor — мы
возвращаем LLM сентинель-tool-message ``awaiting_user_confirmation``,
а сегмент уходит в ``AWAITING_CONFIRMATION`` через scheduler
(фаза 2, см. ADR-0051). Цикл LLM не блокируется (§4.4 — MERGE/QUEUE/
AWAITING не отменяют LLM-цикл).

Этот модуль содержит **только** решение «REQUIRE → sentinel result».
Вызов ``gate.submit`` и формирование payload — здесь; всё остальное
(выполнение, retry, counters) — в основном цикле :meth:`AgentCore._run_with_tools`.
"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from rob_box_llm.provider import ToolCall, ToolResult

if TYPE_CHECKING:
    from rob_box_harness.core.acceptance import AcceptanceGate, Segment


def build_awaiting_confirmation_result(
    call: ToolCall,
    segment: "Segment",
    gate: "AcceptanceGate",
) -> ToolResult:
    """Build the sentinel :class:`ToolResult` for a ``REQUIRE`` call.

    The result is fed back to the LLM so it learns the call is pending
    user confirmation. The actual execution happens later, after the
    user confirms through the channel the scheduler wires up.

    Parameters
    ----------
    call:
        The tool call that requires confirmation. Used only for the
        ``tool_call_id`` on the result.
    segment:
        The segment created by ``gate.submit`` — its ``decision`` carries
        the ``plan_text`` rendered for the user.
    gate:
        The acceptance gate itself — used to read
        ``config.confirmation_timeout_ms``. Kept as an object (not a
        plain int) so unit tests can stub the config without monkey-
        patching module globals.

    Returns
    -------
    ToolResult
        ``is_error=False``, content is a JSON payload with the fields
        the LLM downstream expects: ``status``, ``segment_id``,
        ``tool``, ``plan_text``, ``confirmation_timeout_ms``.
    """
    payload = {
        "status": "awaiting_user_confirmation",
        "segment_id": segment.segment_id,
        "tool": call.name,
        "plan_text": segment.decision.plan_text,
        "confirmation_timeout_ms": int(gate.config.confirmation_timeout_ms),
    }
    return ToolResult(
        tool_call_id=call.id,
        content=json.dumps(payload, ensure_ascii=False),
        is_error=False,
    )