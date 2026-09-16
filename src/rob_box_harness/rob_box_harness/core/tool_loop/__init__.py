"""Tool-loop policy helpers extracted from :class:`AgentCore._run_with_tools`.

This package breaks down the 422-line ``_run_with_tools`` body into three
narrow, independently-testable modules:

* :mod:`.confirmation` — user-confirmation gate policy (issue #968 §8).
  When ``AcceptanceGate.submit`` returns ``REQUIRE``, the orchestrator
  feeds the LLM a sentinel ``tool`` message instead of executing the
  call, so the cycle is not blocked and the user gets to confirm via
  a separate channel.

* :mod:`.retry` — corrective retry helpers (issues #1217 / #1899).
  Two single-shot retries: truncated tool-call arguments (mid-JSON
  cut-off) and silent responses (empty / pseudo-call payload).

* :mod:`.output_format` — final-response shaping (issue #1253).
  Babble filter (tool error + word-only answer → suppress) and the
  ``_ToolLoopOutcome`` builder.

The goal is CC≤15 in :meth:`AgentCore._run_with_tools` (ADR-0021 R1).
The three helpers here carry their own complexity, but each one is
straight-line, single-responsibility, and 100% unit-testable without
ROS2, an LLM, or an executor.
"""

from __future__ import annotations

from rob_box_harness.core.tool_loop.confirmation import (
    build_awaiting_confirmation_result,
)
from rob_box_harness.core.tool_loop.output_format import (
    apply_babble_filter,
    build_outcome_from_response,
)
from rob_box_harness.core.tool_loop.retry import (
    build_silent_response_correction,
    build_truncated_args_correction,
    is_truncated_args_candidate,
    request_silent_response_retry,
    request_truncated_args_retry,
)

__all__ = [
    "apply_babble_filter",
    "build_awaiting_confirmation_result",
    "build_outcome_from_response",
    "build_silent_response_correction",
    "build_truncated_args_correction",
    "is_truncated_args_candidate",
    "request_silent_response_retry",
    "request_truncated_args_retry",
]