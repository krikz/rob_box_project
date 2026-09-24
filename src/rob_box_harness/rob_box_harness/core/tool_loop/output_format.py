"""Output-formatting and error-classification helpers for the tool loop.

Issue #2630 PR-C — выделить ~12 CC из :meth:`AgentCore._run_with_tools`.

Две вещи, которые раньше сидели в хвосте основного метода:

1. **Babble filter** (issue #1253). Если в этом ходу tool вернул
   ``is_error=True`` И модель в финальном ответе НЕ звала retry-tool
   И ``speak_text`` ни разу не вызывался — это babble, не ответ.
   Проговаривание такого текста даёт «дан» / «бит не получился» при
   нулевом действии. Возвращаем ``_ToolLoopOutcome(spoken_text="")``
   чтобы нода перешла к следующему раунду.

2. **Outcome shaping** — единая точка сборки :class:`_ToolLoopOutcome`
   из последнего :class:`LLMResponse` + накопленных счётчиков. Раньше
   эта dataclass собиралась в двух местах (babble-suppressed return +
   happy-path return) с риском разъезжания полей.
"""

from __future__ import annotations

import logging
from typing import Callable

from rob_box_harness.core.tool_loop.outcomes import _ToolLoopOutcome
from rob_box_llm.provider import LLMResponse

_LOG = logging.getLogger(__name__)


def apply_babble_filter(
    *,
    tool_error_occurred: bool,
    response: LLMResponse,
    seen_tool_names: set[str],
    is_silent_response_fn: Callable[[LLMResponse], bool],
    tools_called: list[str],
    speak_text_count: int,
    speak_text_real_count: int,
    spoken_texts: list[str],
    track_name: str | None = None,
) -> _ToolLoopOutcome | None:
    """Return a suppressing outcome for the babble-filter, else ``None``.

    Returns ``None`` when the current response is NOT babble — caller
    falls through to the happy-path outcome builder.

    The babble shape (issue #1253): a tool failed ``is_error=True`` AND
    the LLM's final response has NO tool calls AND ``speak_text`` was
    never named AND the content is silent (per ``is_silent_response_fn``).
    Voicing that would make the robot say «дан» / «бит не получился»
    while nothing actually happened.

    The babble-suppressed outcome keeps the live counters (issue #992 /
    #1343 / #1899 fields must be propagated so downstream consumers
    don't lose visibility into what actually happened this turn).
    """
    is_babble = (
        tool_error_occurred
        and not response.tool_calls
        and "speak_text" not in seen_tool_names
        and bool(response.content)
        and is_silent_response_fn(response)
    )
    if not is_babble:
        return None
    _LOG.warning(
        "AgentCore: tool error + babble-only final answer — "
        f"suppressing spoken text {response.content[:80]!r} "
        "(system transition)"
    )
    return _ToolLoopOutcome(
        spoken_text="",
        tools_called=list(tools_called),
        finish_reason=response.finish_reason,
        raw_response=response.raw,
        speak_text_count=speak_text_count,
        speak_text_real_count=speak_text_real_count,
        spoken_via_tool="\n".join(spoken_texts),
        truncated_tool_args=response.truncated_tool_args,
        track_name=track_name,
        tool_error_occurred=tool_error_occurred,
    )


def build_outcome_from_response(
    *,
    response: LLMResponse,
    tools_called: list[str],
    speak_text_count: int,
    speak_text_real_count: int,
    spoken_texts: list[str],
    track_name: str | None = None,
    tool_error_occurred: bool = False,
) -> _ToolLoopOutcome:
    """Assemble the happy-path :class:`_ToolLoopOutcome` from the last response.

    Centralising this avoids the «семь полей, аннотация обещает шесть»
    drift we had before — there's exactly ONE place that builds the
    dataclass now.
    """
    return _ToolLoopOutcome(
        spoken_text=response.content,
        tools_called=list(tools_called),
        finish_reason=response.finish_reason,
        raw_response=response.raw,
        speak_text_count=speak_text_count,
        speak_text_real_count=speak_text_real_count,
        spoken_via_tool="\n".join(spoken_texts),
        truncated_tool_args=response.truncated_tool_args,
        track_name=track_name,
        tool_error_occurred=tool_error_occurred,
    )
