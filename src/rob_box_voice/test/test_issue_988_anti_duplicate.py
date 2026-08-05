"""test_issue_988_anti_duplicate.py — dialogue_node must NOT double-voice
content the LLM already spoke via ``speak_text`` (issue #988).

E2E v44 regression: the LLM called ``speak_text('Жил да был енот
весёлый...')`` → the song was voiced through the MCP tool, BUT
``dialogue_node._handle_result`` then auto-published the final
``result.spoken_text`` to ``/voice/dialogue/response`` — the song was
read a SECOND time (log: ``[dialogue_node] 📦 TTS batch: 3 chunks``
right after the tool already voiced batch 1/1).

The fix: when ``result.tools_called`` contains ``speak_text``, skip the
auto-TTS publish of the final text (the master prompt tells the LLM to
return ``"done"`` after the LAST speak_text anyway).

These tests drive the real W5 shell (``_TestableDialogueNode`` from
``test_dialogue_shell.py``) with a scripted LLM + a ``FakeToolProvider``
that knows ``speak_text``, and assert on the published response topic.

Run with::

    python3 -m pytest src/rob_box_voice/test/test_issue_988_anti_duplicate.py
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

# Reuse the shell test harness (rclpy shims + testable node).
from test_dialogue_shell import (
    _make_string,
    _response_text,
    _ScriptedLLMProvider,
    _TestableDialogueNode,
)
from rob_box_harness.core.dialogue_state_machine import DialogueEvent
from rob_box_harness.core.tool_registry import ToolSpec
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMResponse, ToolCall


def _speak_text_tools() -> FakeToolProvider:
    """FakeToolProvider with a capturing ``speak_text`` tool."""

    spec = ToolSpec(
        name="speak_text",
        description="Произнести текст голосом через TTS.",
        parameters={
            "type": "object",
            "properties": {
                "text": {"type": "string", "description": "Текст для произнесения."},
            },
            "required": ["text"],
        },
    )

    async def _handler(args):
        return "TTS запрос отправлен: ok"

    tools = FakeToolProvider()
    tools.register(spec, _handler)
    return tools


class TestIssue988AntiDuplicate:
    def _drive(self, node: _TestableDialogueNode) -> None:
        node._dsm.on_event(DialogueEvent.WAKE_WORD)
        node._on_stt(_make_string("роббокс спой песенку про енотика"))
        node.drive_one_turn()

    def test_speak_text_then_done_does_not_auto_voice(self):
        """LLM: speak_text(песня) → 'done'. NO auto-TTS of final text."""
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-1",
                        name="speak_text",
                        arguments={"text": "Жил да был енот весёлый, полосатый, озорной."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_speak_text_tools())
        try:
            self._drive(node)
            # The response topic must NOT contain the song / final text.
            responses = [p.data for p in node._response_pub.published]
            assert responses == [], (
                f"auto-TTS must be skipped when speak_text was called; "
                f"got {len(responses)} publish(es): {responses[:2]!r}"
            )
        finally:
            node.close()

    def test_speak_text_then_full_song_does_not_auto_voice(self):
        """LLM misbehaves: speak_text(песня) + repeats song as final text.

        Even then we must not voice the duplicate — the fix ignores the
        final text whenever ``speak_text`` ran in the cycle.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-1",
                        name="speak_text",
                        arguments={"text": "Жил да был енот весёлый, полосатый, озорной."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(
                content=(
                    "А вот и песенка про енотика! 🎶🦝\n\n"
                    "*Жил да был енот весёлый,*\n"
                    "*полосатый, озорной.*"
                ),
                finish_reason="stop",
            ),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_speak_text_tools())
        try:
            self._drive(node)
            responses = [p.data for p in node._response_pub.published]
            assert responses == [], (
                "final text after speak_text must not be auto-voiced (issue #988)"
            )
        finally:
            node.close()

    def test_plain_text_still_auto_voices(self):
        """No speak_text in the cycle → final text IS published (unchanged)."""
        llm = _ScriptedLLMProvider([LLMResponse(content="Привет!", finish_reason="stop")])
        node = _TestableDialogueNode(llm=llm, tools=_speak_text_tools())
        try:
            self._drive(node)
            responses = [p.data for p in node._response_pub.published]
            assert len(responses) >= 1, "plain-text turn must publish a response"
            text = _response_text(responses[-1])
            assert "Привет!" in text
        finally:
            node.close()


if __name__ == "__main__":  # pragma: no cover
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
