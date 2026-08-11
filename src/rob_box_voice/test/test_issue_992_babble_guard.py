#!/usr/bin/env python3
"""test_issue_992_babble_guard.py — metalanguage / babble detector for issue #992 Bug D.

LLM providers (esp. DeepSeek) sometimes answer a performance command
("роббокс зачитай рэп про космос", "роббокс расскажи стих про осень")
with meta-talk instead of actually performing it::

    speak_text("Зачитаю рэпчик про космос!")
    speak_text("Могу бит добавить, хочешь?")
    → "done"

The user hears the meta-phrase ("Зачитаю рэпчик про космос! Могу бит
добавить, хочешь?") and then silence — no music, no rap, no poem.
The LLM promised but never performed.

Two layers of defense were added for issue #992:

1. **Prompt-level (master_prompt_compact.txt)** — Rule #0 NO
   METALANGUAGE with explicit anti-examples (see the file for the
   full list). Tested here only indirectly (we test the code
   detector, which is the second layer).

2. **Code-level (dialogue_node._check_babble_and_retry)** — when the
   LLM final text starts with a known meta-opener AND ``speak_text``
   was NOT called this cycle, force ONE synchronous retry with a
   CRITICAL prompt reminder. The retry is capped at one per turn to
   avoid an infinite LLM ping-pong.

These tests drive the real W5 shell with scripted LLM responses,
the same pattern as ``test_issue_992_dj_mode.py``. Run with::

    python3 -m pytest src/rob_box_voice/test/test_issue_992_babble_guard.py
"""

from __future__ import annotations

import json
import unittest
from typing import List

import pytest

# Reuse the test scaffolding from the existing 992 suites — the
# ``_TestableDialogueNode`` knows how to run a turn on a private
# asyncio loop and capture publishers.
from test_dialogue_shell import (  # noqa: E402
    _response_text,
    _make_string,
    _ScriptedLLMProvider,
    _TestableDialogueNode,
)
from rob_box_harness.core.dialogue_state_machine import DialogueEvent  # noqa: E402
from rob_box_harness.core.dialog_core import DialogResult  # noqa: E402
from rob_box_harness.core.tool_registry import ToolSpec  # noqa: E402
from rob_box_harness.tools import FakeToolProvider  # noqa: E402
from rob_box_llm.provider import LLMResponse  # noqa: E402


def _published_texts(node: _TestableDialogueNode) -> List[str]:
    """Decode every message published to /voice/dialogue/response."""
    pub = node._publishers.get("/voice/dialogue/response")
    if pub is None:
        return []
    return [_response_text(p) for p in getattr(pub, "published", [])]


# ── Bug D — pure detector unit tests (no LLM round-trip) ─────────────


class TestBabbleDetectorPure(unittest.TestCase):
    """Direct unit tests for ``_is_metalanguage_babble``.

    These pin down the substring matching rules without spinning up
    a full dialogue_node, so the failure modes are easy to read.
    """

    def _babble(self, text: str, node: _TestableDialogueNode) -> bool:
        return node._is_metalanguage_babble(text)

    def _make_node(self) -> _TestableDialogueNode:
        llm = _ScriptedLLMProvider([])
        node = _TestableDialogueNode(llm=llm)
        return node

    def test_detects_zachitayu_rap(self):
        node = self._make_node()
        try:
            self.assertTrue(self._babble("Зачитаю рэпчик про космос!", node))
            self.assertTrue(self._babble("Зачитаю стих про дождь", node))
            self.assertTrue(self._babble("зачитаем песенку про кота", node))
        finally:
            node.close()

    def test_detects_mogu_bit(self):
        node = self._make_node()
        try:
            self.assertTrue(self._babble("Могу бит добавить, хочешь?", node))
            self.assertTrue(self._babble("могу спеть колыбельную", node))
        finally:
            node.close()

    def test_detects_pognali(self):
        node = self._make_node()
        try:
            self.assertTrue(self._babble("Погнали!", node))
            self.assertTrue(self._babble("Ну что, погнали?", node))
        finally:
            node.close()

    def test_detects_slushai_seichas(self):
        node = self._make_node()
        try:
            self.assertTrue(self._babble("Слушай, сейчас расскажу сказку.", node))
            self.assertTrue(self._babble("Слушай, у меня есть идея", node))
        finally:
            node.close()

    def test_detects_pereklyuch(self):
        node = self._make_node()
        try:
            self.assertTrue(self._babble("Переключаюсь на Баха!", node))
            self.assertTrue(self._babble("Переключусь на джаз", node))
        finally:
            node.close()

    def test_normal_answer_does_not_trigger(self):
        node = self._make_node()
        try:
            self.assertFalse(self._babble("Привет! Как дела?", node))
            self.assertFalse(self._babble("Жил-был енотик, полоски на спинке!", node))
            self.assertFalse(self._babble("Ракета мчится через тьму!", node))
            self.assertFalse(self._babble("", node))
        finally:
            node.close()

    def test_mid_sentence_keyword_does_not_trigger(self):
        """A real answer can mention «слушай» in the middle — that's safe."""
        node = self._make_node()
        try:
            text = (
                "Сначала робот едет вперёд, потом поворачивает. "
                "Если хочешь, могу остановиться — просто скажи. "
                "А сейчас продолжу маршрут."
            )
            self.assertFalse(self._babble(text, node))
        finally:
            node.close()

    def test_markdown_prefix_stripped(self):
        """``strip_markdown`` runs before the detector; ensure prefix cleanup."""
        node = self._make_node()
        try:
            # ``strip_markdown`` in core.speak_helpers collapses leading
            # bold/italic markers. After that the detector should still
            # fire on the bare opener.
            self.assertTrue(self._babble("**Зачитаю рэп!**", node))
        finally:
            node.close()


# ── Bug D — performance keyword heuristic ────────────────────────────


class TestPerformanceKeywordHeuristic(unittest.TestCase):
    """``_user_wants_performance`` decides whether babble is hard-fatal."""

    def _wants(self, user_input: str, node: _TestableDialogueNode) -> bool:
        return node._user_wants_performance(user_input)

    def _make_node(self) -> _TestableDialogueNode:
        llm = _ScriptedLLMProvider([])
        node = _TestableDialogueNode(llm=llm)
        return node

    def test_rap_request_is_performance(self):
        node = self._make_node()
        try:
            self.assertTrue(self._wants("роббокс зачитай рэп про космос", node))
            self.assertTrue(self._wants("спой песенку про енотика", node))
            self.assertTrue(self._wants("роббокс расскажи стих про осень", node))
            self.assertTrue(self._wants("сыграй бит", node))
        finally:
            node.close()

    def test_normal_chat_is_not_performance(self):
        node = self._make_node()
        try:
            self.assertFalse(self._wants("как дела?", node))
            self.assertFalse(self._wants("расскажи про РОББОКС", node))
            self.assertFalse(self._wants("", node))
        finally:
            node.close()


# ── Bug D — end-to-end retry behaviour ───────────────────────────────


class TestBabbleRetryE2E(unittest.TestCase):
    """Drive the real ``_run_turn`` path with a scripted LLM."""

    def test_babble_response_triggers_one_retry(self):
        """LLM answers with meta-talk → 1 retry scheduled, babble swallowed.

        This test pins down the core invariants of the bug-D guard:

        1. The original babble text "Зачитаю рэпчик про космос!" MUST
           NOT reach the TTS publisher.
        2. The LLM provider is hit exactly twice (initial + retry).
        3. The ``_babble_retry_used`` flag is set after the retry
           fires.

        The retry's final-answer content is intentionally NOT pinned
        here — the scripted second response is plain text without
        ``execute_music_code``, so the music-guard (Bug C) will then
        speak a recovery nudge. End-to-end coverage of the user's
        audio path is provided by ``test_issue_992_dj_mode.py`` with
        a real tool-call scripted response; this test focuses on the
        babble layer exclusively.
        """
        from rob_box_harness.core.tool_registry import ToolSpec
        from rob_box_harness.tools import FakeToolProvider

        # First call: babble (no speak_text). Second call: clean
        # plain-text answer (would pass the babble detector, but the
        # music-guard catches it because no execute_music_code).
        llm = _ScriptedLLMProvider([
            LLMResponse(content="Зачитаю рэпчик про космос!", finish_reason="stop"),
            LLMResponse(content="Ракета мчится через тьму и пустоту!", finish_reason="stop"),
        ])
        # Register a no-op execute_music_code so the babble-retry
        # prompt's mention of the tool doesn't surprise the harness.
        tools = FakeToolProvider()
        tools.register(
            ToolSpec(
                name="execute_music_code",
                description="Сыграть Renardo / SuperCollider code.",
                parameters={"type": "object", "properties": {"code": {"type": "string"}}},
            ),
            lambda args: json.dumps({"ok": True}),
        )

        node = _TestableDialogueNode(llm=llm, tools=tools)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс зачитай рэп про космос"))
            node.drive_one_turn()

            texts = _published_texts(node)
            joined = " ".join(texts)
            # Bug D assertion: the babble text "Зачитаю рэпчик про
            # космос!" MUST NOT be in the published TTS stream.
            self.assertNotIn(
                "Зачитаю рэпчик про космос", joined,
                "babble text must be swallowed by the retry, not voiced; "
                f"published={texts!r}",
            )
            # Babble retry budget: exactly one dispatch per turn.
            # ``llm.call_count`` records how many times the provider
            # was hit; we expect 2 (original + retry).
            self.assertEqual(
                llm.call_count, 2,
                f"expected 2 LLM calls (initial + retry), got {llm.call_count}",
            )
            self.assertTrue(
                node._babble_retry_used,
                "babble-retry flag must be set after a retry fires",
            )
        finally:
            node.close()

    def test_no_babble_no_retry(self):
        """Normal answer → no retry, exactly 1 LLM call."""
        llm = _ScriptedLLMProvider([
            LLMResponse(content="Привет! Как дела?", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс привет"))
            node.drive_one_turn()

            self.assertEqual(llm.call_count, 1)
            self.assertFalse(node._babble_retry_used)
            texts = _published_texts(node)
            self.assertTrue(
                any("Привет" in t for t in texts),
                f"normal answer should be published verbatim; got {texts!r}",
            )
        finally:
            node.close()

    def test_pure_promise_opener_retries_without_performance_keyword(self):
        """«Зачитаю», «Погнали», «Устроим» always retry — even on a non-perf turn.

        These openers are NEVER valid answers (they're pure promises).
        The detector should fire a retry regardless of the user_input
        classification. We verify by scripting a clean second reply
        and asserting the original babble was swallowed.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(content="Погнали!", finish_reason="stop"),
            LLMResponse(content="Готов к работе.", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс расскажи про себя"))
            node.drive_one_turn()

            texts = _published_texts(node)
            joined = " ".join(texts)
            self.assertNotIn("Погнали", joined)
            self.assertIn("Готов к работе", joined)
            self.assertEqual(llm.call_count, 2)
        finally:
            node.close()

    def test_retry_only_fires_once_even_if_retry_also_babbles(self):
        """Babble-retry budget is one-shot per turn.

        The second LLM call also babbles («Слушай, сейчас...»). Even
        though it's still meta-talk, the detector must NOT fire a
        second retry (that would loop forever). The second babble
        text is published verbatim and we log a warning so the
        operator sees it.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(content="Зачитаю рэп про космос!", finish_reason="stop"),
            LLMResponse(content="Слушай, сейчас всё будет.", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс зачитай рэп про космос"))
            node.drive_one_turn()

            self.assertEqual(
                llm.call_count, 2,
                "babble-retry is one-shot; second babble must not trigger "
                f"a third call; got {llm.call_count}",
            )
            self.assertTrue(node._babble_retry_used)
            # The second babble is published verbatim (no further retry).
            texts = _published_texts(node)
            joined = " ".join(texts)
            self.assertIn("Слушай", joined)
        finally:
            node.close()

    def test_speak_text_call_suppresses_babble_retry(self):
        """If the LLM called ``speak_text``, the anti-duplicate path wins.

        Even if the final text starts with «зачит», we MUST NOT retry
        — the LLM already produced audio via the MCP tool and the
        auto-TTS publish was correctly skipped by issue #988.
        """
        # We can't easily inject ``tools_called`` into the scripted
        # LLM provider, so we test the detector directly. End-to-end
        # coverage of this path lives in test_issue_988_anti_duplicate
        # — the babble detector shares its anti-duplicate guard.
        llm = _ScriptedLLMProvider([])
        node = _TestableDialogueNode(llm=llm)
        try:
            result = DialogResult(
                spoken_text="Зачитаю рэп про космос!",
                tools_called=("speak_text",),
            )
            decision = node._check_babble_and_retry(
                spoken=result.spoken_text,
                user_input="роббокс зачитай рэп",
                tools_called=result.tools_called,
            )
            self.assertFalse(
                decision,
                "babble retry must NOT fire when speak_text was already called",
            )
        finally:
            node.close()


if __name__ == "__main__":
    unittest.main()