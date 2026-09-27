"""Unit tests for tool-loop policy helpers.

Issue #2630 — каждый выделенный из ``AgentCore._run_with_tools``
политический модуль покрывается юнит-тестами (ADR-0021 R1 + ADR-0018).
Тесты не зависят от ROS2, LLM, executor'а — только от dataclass'ов
и helper'ов в чистом виде.

Структура:

* :class:`TestBuildAwaitingConfirmationResult` — ``confirmation.py``
* :class:`TestPseudoToolCallClassifier` — ``text_classify.py``
* :class:`TestIsTruncatedArgsCandidate` — ``retry.py`` (детектор)
* :class:`TestTruncatedArgsCorrection` — ``retry.py`` (текст correction)
* :class:`TestSilentResponseCorrection` — ``retry.py`` (текст correction)
* :class:`TestApplyBabbleFilter` — ``output_format.py``
* :class:`TestBuildOutcomeFromResponse` — ``output_format.py``
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

import pytest

from rob_box_harness.core.tool_loop import (
    apply_babble_filter,
    build_awaiting_confirmation_result,
    build_outcome_from_response,
    build_silent_response_correction,
    build_truncated_args_correction,
    is_truncated_args_candidate,
)
from rob_box_harness.core.tool_loop.outcomes import _ToolLoopOutcome
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall, ToolResult


# ---------------------------------------------------------------------------
# Test fixtures — minimal stand-ins for AcceptanceGate / Segment.
# ---------------------------------------------------------------------------


@dataclass
class _FakeSegment:
    """Stand-in for ``rob_box_harness.core.acceptance.Segment``."""

    segment_id: str = "seg-abc123"
    decision: Any = field(
        default_factory=lambda: _FakeDecision(plan_text="нужно подтверждение")
    )


@dataclass
class _FakeDecision:
    plan_text: str = "подтвердите выполнение"


@dataclass
class _FakeGateConfig:
    confirmation_timeout_ms: int = 30000


@dataclass
class _FakeGate:
    config: _FakeGateConfig = field(default_factory=_FakeGateConfig)


# ---------------------------------------------------------------------------
# confirmation.build_awaiting_confirmation_result
# ---------------------------------------------------------------------------


class TestBuildAwaitingConfirmationResult:
    """Sentinel ``ToolResult`` returned to the LLM on a REQUIRE decision."""

    def test_returns_tool_result_with_call_id(self) -> None:
        call = ToolCall(id="call-42", name="navigate_to_waypoint", arguments={})
        seg = _FakeSegment()
        gate = _FakeGate()

        result = build_awaiting_confirmation_result(call, seg, gate)

        assert isinstance(result, ToolResult)
        assert result.tool_call_id == "call-42"
        assert result.is_error is False

    def test_payload_contains_required_fields(self) -> None:
        call = ToolCall(id="c1", name="start_mapping", arguments={"area": "kitchen"})
        seg = _FakeSegment(segment_id="seg-xyz")
        gate = _FakeGate()

        result = build_awaiting_confirmation_result(call, seg, gate)

        payload = json.loads(result.content)
        assert payload["status"] == "awaiting_user_confirmation"
        assert payload["segment_id"] == "seg-xyz"
        assert payload["tool"] == "start_mapping"
        assert payload["plan_text"] == "нужно подтверждение"
        assert payload["confirmation_timeout_ms"] == 30000

    def test_payload_uses_unicode_plan_text_unescaped(self) -> None:
        """``ensure_ascii=False`` so the LLM sees readable Cyrillic."""
        call = ToolCall(id="c1", name="start_mapping", arguments={})
        seg = _FakeSegment(
            decision=_FakeDecision(plan_text="Переместиться на кухню?")
        )
        gate = _FakeGate()

        result = build_awaiting_confirmation_result(call, seg, gate)

        # The raw JSON should contain the Cyrillic chars directly, not \uXXXX.
        assert "Переместиться на кухню?" in result.content

    def test_timeout_is_int_cast(self) -> None:
        """Even if config returns a non-int, we cast to int (defensive)."""
        call = ToolCall(id="c1", name="t", arguments={})
        seg = _FakeSegment()

        @dataclass
        class _Config:
            confirmation_timeout_ms: Any = "15000"

        @dataclass
        class _Gate:
            config: Any = field(default_factory=_Config)

        result = build_awaiting_confirmation_result(call, seg, _Gate())
        payload = json.loads(result.content)
        assert payload["confirmation_timeout_ms"] == 15000
        assert isinstance(payload["confirmation_timeout_ms"], int)


# ---------------------------------------------------------------------------
# text_classify.is_pseudo_tool_call
# ---------------------------------------------------------------------------


class TestPseudoToolCallClassifier:
    """Model wrote ``<single_token>`` instead of calling."""

    @pytest.mark.parametrize(
        "text",
        [
            "<speak_text>",
            "<navigate>",
            "<set_dj_mode>",
        ],
    )
    def test_detects_single_token_pseudo(self, text: str) -> None:
        from rob_box_harness.core.tool_loop.text_classify import (
            is_pseudo_tool_call,
        )

        assert is_pseudo_tool_call(text) is True

    @pytest.mark.parametrize(
        "text",
        [
            "",
            "просто текст без вызова",
            "<speak_text>{}",  # has braces — regex only matches single tokens
            "<navigate>(a, b)",  # parens — not allowed by [^<>]
            "<>",  # empty tag
            "<<>>",  # nested tags — has < inside
        ],
    )
    def test_rejects_non_pseudo(self, text: str) -> None:
        from rob_box_harness.core.tool_loop.text_classify import (
            is_pseudo_tool_call,
        )

        # The helper itself calls .strip() internally — test the raw input.
        assert is_pseudo_tool_call(text) is False


# ---------------------------------------------------------------------------
# retry.is_truncated_args_candidate
# ---------------------------------------------------------------------------


class TestIsTruncatedArgsCandidate:
    """Issue #1899 — detect truncated JSON BEFORE execution."""

    def test_true_when_truncated_args_with_tool_calls(self) -> None:
        response = _make_response(
            tool_calls=[ToolCall(id="t1", name="navigate", arguments={"x": 1})],
            truncated_tool_args=True,
        )
        assert is_truncated_args_candidate(response) is True

    def test_false_when_not_truncated(self) -> None:
        response = _make_response(
            tool_calls=[ToolCall(id="t1", name="navigate", arguments={"x": 1})],
            truncated_tool_args=False,
        )
        assert is_truncated_args_candidate(response) is False

    def test_false_when_no_tool_calls(self) -> None:
        """No tool-call → no broken JSON to retry."""
        response = _make_response(
            tool_calls=[],
            truncated_tool_args=True,
        )
        assert is_truncated_args_candidate(response) is False


# ---------------------------------------------------------------------------
# retry.build_truncated_args_correction
# ---------------------------------------------------------------------------


class TestTruncatedArgsCorrection:
    """Correction message asks the model to redo with shorter args."""

    def test_returns_user_role_message(self) -> None:
        response = _make_response(
            tool_calls=[ToolCall(id="t1", name="navigate", arguments={})],
            finish_reason="length",
        )
        msg = build_truncated_args_correction(response)
        assert isinstance(msg, LLMMessage)
        assert msg.role == "user"

    def test_lists_tool_names_sorted(self) -> None:
        response = _make_response(
            tool_calls=[
                ToolCall(id="t1", name="z_tool", arguments={}),
                ToolCall(id="t2", name="a_tool", arguments={}),
            ],
        )
        msg = build_truncated_args_correction(response)
        assert "a_tool" in msg.content
        assert "z_tool" in msg.content
        # Names appear sorted in the message (a_tool before z_tool).
        assert msg.content.index("a_tool") < msg.content.index("z_tool")

    def test_message_mentions_finish_reason_context(self) -> None:
        """Operator can grep for the retry trigger without log-diving."""
        response = _make_response(
            tool_calls=[ToolCall(id="t1", name="navigate", arguments={})],
            finish_reason="length",
        )
        msg = build_truncated_args_correction(response)
        # The Russian instruction should be present.
        assert "ОБРЕЗАН" in msg.content
        assert "max_tokens" in msg.content


# ---------------------------------------------------------------------------
# retry.build_silent_response_correction
# ---------------------------------------------------------------------------


class TestSilentResponseCorrection:
    """Correction message for empty payload / pseudo-call."""

    def test_pseudo_call_uses_specific_message(self) -> None:
        """``<single_token>`` triggers the «Ты НАПИСАЛ» branch (issue 01.09)."""
        response = _make_response(
            content="<speak_text>", tool_calls=[]
        )
        msg = build_silent_response_correction(response)
        assert "НАПИСАЛ" in msg.content
        assert "function" in msg.content
        assert "ПО-НАСТОЯЩЕМУ" in msg.content

    def test_empty_payload_uses_empty_message(self) -> None:
        response = _make_response(content="", tool_calls=[])
        msg = build_silent_response_correction(response)
        assert "был пустым" in msg.content
        assert "ОБЯЗАТЕЛЬНО" in msg.content

    def test_pseudo_call_includes_quoted_text(self) -> None:
        """The correction echoes the bad text so the model sees what it wrote."""
        response = _make_response(
            content="<speak_text>", tool_calls=[]
        )
        msg = build_silent_response_correction(response)
        # The pseudo-text appears inside the correction verbatim.
        assert "<speak_text>" in msg.content

    def test_returns_user_role(self) -> None:
        msg = build_silent_response_correction(_make_response(content=""))
        assert msg.role == "user"


# ---------------------------------------------------------------------------
# output_format.apply_babble_filter
# ---------------------------------------------------------------------------


class TestApplyBabbleFilter:
    """Issue #1253 — babble filter on tool error + word-only final answer."""

    def _tool_error_response(self, content: str = "готово") -> LLMResponse:
        return _make_response(content=content, tool_calls=[])

    def test_no_tool_error_returns_none(self) -> None:
        """Happy path → caller falls through to ``build_outcome_from_response``."""
        response = self._tool_error_response()
        outcome = apply_babble_filter(
            tool_error_occurred=False,
            response=response,
            seen_tool_names=set(),
            is_silent_response_fn=_is_silent_response_stub,
            tools_called=[],
            speak_text_count=0,
            speak_text_real_count=0,
            spoken_texts=[],
        )
        assert outcome is None

    def test_tool_error_with_speak_text_returns_none(self) -> None:
        """Tool failed BUT ``speak_text`` was called → real speech happened."""
        outcome = apply_babble_filter(
            tool_error_occurred=True,
            response=self._tool_error_response(),
            seen_tool_names={"speak_text"},
            is_silent_response_fn=_is_silent_response_stub,
            tools_called=["speak_text"],
            speak_text_count=1,
            speak_text_real_count=1,
            spoken_texts=["привет"],
        )
        assert outcome is None

    def test_tool_error_with_tool_calls_returns_none(self) -> None:
        """Tool failed BUT model called another tool in the final response."""
        response = _make_response(
            content="дан",
            tool_calls=[ToolCall(id="t1", name="set_dj_mode", arguments={})],
        )
        outcome = apply_babble_filter(
            tool_error_occurred=True,
            response=response,
            seen_tool_names=set(),
            is_silent_response_fn=_is_silent_response_stub,
            tools_called=["set_dj_mode"],
            speak_text_count=0,
            speak_text_real_count=0,
            spoken_texts=[],
        )
        assert outcome is None

    def test_tool_error_babble_suppresses_spoken_text(self) -> None:
        """Tool failed, no tool calls, no speak_text, silent content → babble."""
        # Use a content that the stub recognises as silent (pseudo-call).
        response = self._tool_error_response("<speak_text>")
        outcome = apply_babble_filter(
            tool_error_occurred=True,
            response=response,
            seen_tool_names=set(),
            is_silent_response_fn=_is_silent_response_stub,
            tools_called=["execute_music_code"],
            speak_text_count=0,
            speak_text_real_count=0,
            spoken_texts=[],
        )
        assert isinstance(outcome, _ToolLoopOutcome)
        assert outcome.spoken_text == ""
        assert outcome.tools_called == ["execute_music_code"]
        assert outcome.speak_text_count == 0

    def test_non_silent_content_is_not_babble(self) -> None:
        """If the model produced a real sentence, voice it."""
        outcome = apply_babble_filter(
            tool_error_occurred=True,
            response=self._tool_error_response("Простите, не получилось"),
            seen_tool_names=set(),
            is_silent_response_fn=lambda r: False,  # never silent
            tools_called=["execute_music_code"],
            speak_text_count=0,
            speak_text_real_count=0,
            spoken_texts=[],
        )
        assert outcome is None


# ---------------------------------------------------------------------------
# output_format.build_outcome_from_response
# ---------------------------------------------------------------------------


class TestBuildOutcomeFromResponse:
    """Happy-path outcome assembly."""

    def test_basic_shape(self) -> None:
        response = _make_response(
            content="привет",
            tool_calls=[],
            finish_reason="stop",
            truncated_tool_args=False,
        )
        outcome = build_outcome_from_response(
            response=response,
            tools_called=["speak_text"],
            speak_text_count=1,
            speak_text_real_count=1,
            spoken_texts=["привет"],
        )
        assert outcome.spoken_text == "привет"
        assert outcome.tools_called == ["speak_text"]
        assert outcome.finish_reason == "stop"
        assert outcome.speak_text_count == 1
        assert outcome.speak_text_real_count == 1
        assert outcome.spoken_via_tool == "привет"
        assert outcome.truncated_tool_args is False

    def test_spoken_texts_joined_with_newline(self) -> None:
        response = _make_response(content="x", tool_calls=[])
        outcome = build_outcome_from_response(
            response=response,
            tools_called=["speak_text"],
            speak_text_count=2,
            speak_text_real_count=2,
            spoken_texts=["phrase one", "phrase two"],
        )
        assert outcome.spoken_via_tool == "phrase one\nphrase two"

    def test_tools_called_list_is_copied(self) -> None:
        """The output list is independent of the input — caller mutations don't leak."""
        original = ["speak_text"]
        response = _make_response(content="x", tool_calls=[])
        outcome = build_outcome_from_response(
            response=response,
            tools_called=original,
            speak_text_count=1,
            speak_text_real_count=1,
            spoken_texts=["x"],
        )
        original.append("set_dj_mode")
        assert outcome.tools_called == ["speak_text"]


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _is_silent_response_stub(response: LLMResponse) -> bool:
    """Stub of :meth:`AgentCore._is_silent_response` for unit tests.

    Returns True for content that is empty / pseudo-call / the «done»
    marker (mirrors the real implementation's defaults).
    """
    from rob_box_harness.core.tool_loop.text_classify import (
        is_pseudo_tool_call,
    )

    content = response.content or ""
    stripped = content.strip()
    if not stripped:
        return True
    if stripped == "done":
        return True
    if is_pseudo_tool_call(content):
        return True
    return False


def _make_response(
    *,
    content: str = "",
    tool_calls: list[ToolCall] | None = None,
    finish_reason: str = "stop",
    truncated_tool_args: bool = False,
) -> LLMResponse:
    """Build a minimal :class:`LLMResponse` for unit tests.

    Mirrors the structure produced by real LLM providers but skips the
    heavy provider-specific ``raw`` payload — only ``finish_reason``,
    ``truncated_tool_args``, ``content``, and ``tool_calls`` are needed
    by the helpers under test.
    """
    return LLMResponse(
        content=content,
        tool_calls=list(tool_calls or []),
        finish_reason=finish_reason,
        raw={"stub": True},
        truncated_tool_args=truncated_tool_args,
    )