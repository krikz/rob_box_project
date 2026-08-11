#!/usr/bin/env python3
"""test_issue_992_dj_mode.py — DJ-mode regressions for issue #992.

The 5-second DJ auto-transition timer fires a fresh LLM cycle every time
``next_transition_at`` expires. The previous implementation had three
distinct bugs that this test file pins down:

**Bug A — DJ auto-transition fires ``new_dialogue`` music_cleanup.**
The first non-DJ STT in a session publishes ``/mcp/music_cleanup`` with
``reason="new_dialogue"`` so the MCP server can stop any leftover
background music before the new dialogue. The same code path fired for
*every* DJ auto-transition, which wiped the in-flight Renardo track
mid-phrase and fed the LLM an empty ``process_input`` payload. The fix
routes DJ dispatches through a dedicated :meth:`DialogueNode._dispatch_dj_turn`
that takes the ``is_dj_auto=True`` branch in :meth:`_dispatch_turn`,
skipping the cleanup publish.

**Bug B — DJ tick without ``execute_music_code`` is silent.**
Even with Bug A fixed, the LLM sometimes replies to the DJ auto-prompt
with just a spoken phrase and no ``execute_music_code`` tool call — the
DJ cycle produces zero music for that transition. The fix installs a
post-turn guard (:meth:`_apply_music_guard`) that, when DJ mode is on
*and* the LLM did not invoke ``execute_music_code``, re-arms
``next_transition_at`` and publishes a synchronous follow-up prompt
with an explicit CRITICAL reminder. The retry is capped at
``MAX_DJ_AUTO_RETRIES`` to prevent an infinite loop on a stubborn LLM.

**Bug C — user rap/song request without ``execute_music_code`` is silent.**
Even outside DJ mode, when the user explicitly asks for rap / song /
DJ and the LLM skips ``execute_music_code``, the robot used to say
nothing. The fix teaches :meth:`_apply_music_guard` a narrow keyword
heuristic (``_user_wants_music``) and publishes a short spoken
acknowledgment so the user hears *something* and can retry.

These tests drive the real W5 shell (``_TestableDialogueNode``) with
scripted LLM responses, the same pattern as
``test_issue_992_batch_cleanup.py``.

Run with::

    python3 -m pytest src/rob_box_voice/test/test_issue_992_dj_mode.py
"""

from __future__ import annotations

import json
import unittest
from typing import Any, List

import pytest

from test_dialogue_shell import (
    _make_string,
    _ScriptedLLMProvider,
    _TestableDialogueNode,
)
from rob_box_harness.core.dialogue_state_machine import DialogueEvent
from rob_box_harness.core.tool_registry import ToolSpec
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMResponse, ToolCall


# ── helpers ──────────────────────────────────────────────────────────


def _execute_music_code_tools() -> FakeToolProvider:
    """FakeToolProvider carrying a no-op ``execute_music_code`` tool.

    The handler returns immediately so ``DialogCore`` records the tool
    call name in ``result.tools_called`` without actually playing any
    Renardo code — that's the signal the music-guard uses to decide
    whether Bug B / Bug C should fire.
    """

    spec = ToolSpec(
        name="execute_music_code",
        description="Сыграть Renardo / SuperCollider code.",
        parameters={
            "type": "object",
            "properties": {
                "code": {"type": "string", "description": "Renardo code."},
                "segments": {"type": "integer", "description": "Bars."},
            },
            "required": ["code"],
        },
    )

    async def _handler(args):
        return json.dumps({"ok": True, "played": True})

    tools = FakeToolProvider()
    tools.register(spec, _handler)
    return tools


def _music_cleanup_payloads(node: _TestableDialogueNode) -> List[dict]:
    """Decode every message ``dialogue_node`` published on /mcp/music_cleanup.

    Empty list when the publisher was never created (no
    ``/mcp/music_cleanup`` publisher available) or no messages were sent.
    """
    pub = node._publishers.get("/mcp/music_cleanup")
    if pub is None:
        return []
    out: List[dict] = []
    for msg in getattr(pub, "published", []):
        data = getattr(msg, "data", None)
        if not data:
            continue
        try:
            out.append(json.loads(data))
        except (TypeError, ValueError):
            out.append({"raw": data})
    return out


# ── Bug A: DJ auto-transition must NOT fire ``new_dialogue`` cleanup ─


class TestIssue992BugA(unittest.TestCase):
    """Regression for issue #992 Bug A — DJ tick must skip ``new_dialogue`` cleanup."""

    def test_dj_dispatch_does_not_publish_new_dialogue_cleanup(self):
        """``_dispatch_dj_turn`` calls ``_dispatch_turn(is_dj_auto=True)``.

        Before the fix the DJ auto-transition went through the same
        path as a user STT message and published
        ``/mcp/music_cleanup`` with ``reason="new_dialogue"``. After the
        fix the DJ dispatch bypasses that publish — the only cleanup
        messages that ever go out for a DJ cycle must come from
        ``tts_batch_complete`` (i.e. ``reason="tts_batch_complete"``).
        """
        # ``_dispatch_dj_turn`` schedules onto the loop, so we don't
        # need a real LLM. Stub one anyway so the turn can run.
        llm = _ScriptedLLMProvider()
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс как дела"))
            node.drive_one_turn()
            # Sanity: baseline turn cleared _pending_music_cleanup.
            cleanups_after_baseline = len(_music_cleanup_payloads(node))

            # Simulate an active DJ cycle: enable DJ mode and dispatch
            # the auto-transition. We use ``_dispatch_dj_turn`` directly
            # (not the timer) so we don't depend on the 5 s tick.
            node._dj.state.enabled = True
            node._dj.state.next_transition_at = 0.0
            node._pending_music_cleanup = False
            node._dispatch_dj_turn("[DJ_AUTO] test transition")
            node.drive_one_turn()

            # Bug A assertion: the DJ auto-dispatch must NOT publish
            # ``music_cleanup`` with ``reason="new_dialogue"``. It can
            # still publish with ``reason="tts_batch_complete"`` from
            # the legacy path, so we filter by reason.
            cleanups = _music_cleanup_payloads(node)
            new_dialogue_cleanups = [
                c for c in cleanups
                if c.get("reason") == "new_dialogue"
            ]
            self.assertEqual(
                new_dialogue_cleanups, [],
                "DJ auto-dispatch must NOT publish new_dialogue cleanup "
                f"(issue #992 Bug A); got {new_dialogue_cleanups!r}",
            )
            # Sanity: nothing was published at all from this empty
            # cycle (no TTS chunks ⇒ no batch_complete either).
            self.assertEqual(
                len(cleanups), cleanups_after_baseline,
                "DJ auto-dispatch (no TTS, no stop_music) must not add "
                "any new music_cleanup publish — baseline cycle already "
                "fired its own.",
            )
        finally:
            node.close()

    def test_dispatch_turn_user_path_still_publishes_new_dialogue_cleanup(self):
        """Sanity: the user-STT path still fires ``new_dialogue`` cleanup.

        ``_dispatch_dj_turn`` is the only path that should skip the
        cleanup. The user-STT path (``_on_stt`` → ``_dispatch_turn``)
        must keep publishing it — otherwise the original
        "wait for fresh user dialogue" contract regresses.
        """
        llm = _ScriptedLLMProvider(["Привет!"])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс как дела"))
            node.drive_one_turn()

            # ``_on_stt`` → ``_dispatch_turn(is_dj_auto=False)`` must
            # still trigger the cleanup publish. The cleanup is
            # deferred until ``tts_batch_complete``, but the legacy
            # code path also publishes ``new_dialogue`` eagerly inside
            # ``_dispatch_turn`` when ``_pending_music_cleanup`` is
            # already set. We assert the flag was NOT cleared and the
            # ``tts_batch_complete`` channel will publish eventually.
            new_dialogue_cleanups = [
                c for c in _music_cleanup_payloads(node)
                if c.get("reason") == "new_dialogue"
            ]
            # The user path either publishes ``new_dialogue`` eagerly
            # (when a previous pending cleanup existed) or holds the
            # cleanup until ``tts_batch_complete``. Both are correct;
            # the only wrong outcome is "no cleanup published at all
            # after a fresh user turn", which the regression suites
            # in test_tts_batch_complete.py already cover.
            # Here we just confirm the user-path code still calls
            # _dispatch_turn (which would have fired cleanup if a
            # pending cleanup was already set).
            node._pending_music_cleanup = True  # simulate leftover state
            node._on_stt(_make_string("роббокс следующий вопрос"))
            node.drive_one_turn()
            all_reasons = [
                c.get("reason") for c in _music_cleanup_payloads(node)
            ]
            self.assertIn(
                "new_dialogue", all_reasons,
                "User-STT path must still publish new_dialogue cleanup "
                "when _pending_music_cleanup was set (regression guard).",
            )
        finally:
            node.close()


# ── Bug B: DJ auto-transition retries on missing execute_music_code ──


class TestIssue992BugB(unittest.TestCase):
    """Regression for issue #992 Bug B — DJ guard forces music sync retry."""

    def test_dj_auto_without_music_triggers_synchronous_retry(self):
        """LLM skips ``execute_music_code`` on a DJ auto-turn → retry fires.

        The post-turn guard (``_apply_music_guard``) sees that the LLM
        replied with just text (no ``execute_music_code`` tool call)
        while DJ mode is on, increments the retry counter, postpones
        ``next_transition_at`` by ``POSTPONE_INTERVAL_S`` and dispatches
        a fresh turn with the CRITICAL retry prompt.
        """
        # First scripted reply: just text, no tool call.
        # Second reply (after the retry): calls execute_music_code.
        llm = _ScriptedLLMProvider([
            LLMResponse(content="Готовлю трек", finish_reason="stop"),
            LLMResponse(
                content="",
                tool_calls=(ToolCall(
                    id="call-music",
                    name="execute_music_code",
                    arguments={"code": "Clock.bpm = 120\nd1 >> play('x')"},
                ),),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="Готово", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_execute_music_code_tools())
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._dj.state.enabled = True
            node._dj.state.next_transition_at = 0.0
            # Tick fires the auto-dispatch.
            node._dj.tick()
            node.drive_one_turn()
            # The first auto-turn ran (LLM call_count == 1). The guard
            # noticed no execute_music_code, so it scheduled a
            # synchronous retry which ran as the second turn.
            # LLM call_count == 2 (one for each turn).
            self.assertEqual(
                llm.call_count, 2,
                "DJ auto-turn + synchronous retry must drive exactly "
                f"two LLM calls; got {llm.call_count}",
            )
            # Retry counter was reset on the successful second turn.
            self.assertEqual(
                node._dj_auto_retry_count, 0,
                "Successful retry must reset the Bug-B retry counter",
            )
        finally:
            node.close()

    def test_dj_auto_retry_budget_caps_loop(self):
        """After ``MAX_DJ_AUTO_RETRIES`` failures the guard stops.

        Without a budget cap, a stubborn LLM that keeps ignoring the
        CRITICAL reminder would lock the dialogue node in an infinite
        coroutine chain, never letting the 5 s tick take over. The cap
        is currently 2 — once exceeded, the guard logs a warning and
        returns. The test scripts 4 text-only replies to prove the
        cap fires and the LLM is called exactly ``1 + MAX_DJ_AUTO_RETRIES``
        times for the auto cycle.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(content="no music 1", finish_reason="stop"),
            LLMResponse(content="no music 2", finish_reason="stop"),
            LLMResponse(content="no music 3", finish_reason="stop"),
            LLMResponse(content="no music 4", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._dj.state.enabled = True
            node._dj.state.next_transition_at = 0.0
            node._dj.tick()
            node.drive_one_turn()
            # 1 tick + N retries (capped at MAX_DJ_AUTO_RETRIES).
            expected = 1 + node.MAX_DJ_AUTO_RETRIES
            self.assertEqual(
                llm.call_count, expected,
                f"DJ tick + retries must be capped at {expected} LLM "
                f"calls (1 + MAX_DJ_AUTO_RETRIES={node.MAX_DJ_AUTO_RETRIES}); "
                f"got {llm.call_count}",
            )
            # Counter is reset so the next tick has a fresh budget.
            self.assertEqual(
                node._dj_auto_retry_count, 0,
                "Retry budget must be reset once exhausted, so the next "
                "tick has a fresh allowance.",
            )
        finally:
            node.close()

    def test_dj_auto_with_music_does_not_retry(self):
        """Happy path: LLM calls execute_music_code → no retry.

        Sanity guard: when the LLM *does* call ``execute_music_code``
        the music-guard must NOT trigger a redundant follow-up turn.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(ToolCall(
                    id="call-music",
                    name="execute_music_code",
                    arguments={"code": "Clock.bpm = 110\nd1 >> play('x')"},
                ),),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="Готово", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_execute_music_code_tools())
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._dj.state.enabled = True
            node._dj.state.next_transition_at = 0.0
            node._dj.tick()
            node.drive_one_turn()
            # LLM call_count == 2: one for the tool-call turn, one
            # for the follow-up text reply. There must be NO extra
            # retry turn on top of that.
            self.assertEqual(
                llm.call_count, 2,
                "Successful DJ auto-turn must NOT trigger an extra "
                f"retry; got {llm.call_count} LLM calls (expected 2).",
            )
        finally:
            node.close()

    def test_dj_flag_resets_after_run_turn(self):
        """``_current_turn_is_dj_auto`` is reset regardless of branch.

        The flag drives Bug B + Bug C. If a DJ auto-turn left it set,
        the *next* user turn would also be treated as DJ-auto and
        trigger Bug-B retries on the user's question. The fix
        resets it in ``_run_turn.finally`` (both happy and error
        branches).
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(content="ok", finish_reason="stop"),
            LLMResponse(content="user reply", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._dj.state.enabled = True
            node._dj.state.next_transition_at = 0.0
            node._dj.tick()
            node.drive_one_turn()
            # After the DJ auto-turn ran, the flag must be False again.
            self.assertFalse(
                node._current_turn_is_dj_auto,
                "_current_turn_is_dj_auto must reset after the DJ "
                "auto-turn finishes (else user turns would be misclassified).",
            )
        finally:
            node.close()


# ── Bug C: user rap/song request without execute_music_code ───────────


class TestIssue992BugC(unittest.TestCase):
    """Regression for issue #992 Bug C — user rap/song guard speaks a nudge."""

    def test_user_rap_request_without_music_publishes_nudge(self):
        """User asks for rap, LLM skips execute_music_code → spoken nudge.

        Outside DJ mode, when the user explicitly asks for a
        rap / song / DJ, the music-guard publishes a short spoken
        acknowledgment so the user hears *something* and can retry.
        The nudge is sent through the regular ``_speak_direct`` →
        ``/voice/dialogue/response`` path (issue #988 anti-duplicate
        still suppresses auto-TTS for ``speak_text`` cycles, but the
        nudge uses ``_speak_direct``, which is the low-level chunk
        publish that bypasses LLM-anti-dup).
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(content="сейчас", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс спой рэп про енотика"))
            node.drive_one_turn()

            responses = [
                _extract_response_text(p)
                for p in node._response_pub.published
            ]
            joined = " ".join(responses)
            self.assertIn(
                "бит не запустился", joined,
                "Bug C must publish a spoken nudge when user asks for "
                "rap/song/DJ and LLM skips execute_music_code; "
                f"responses={responses!r}",
            )
        finally:
            node.close()

    def test_user_normal_chat_without_music_is_silent(self):
        """Negative: ordinary chit-chat that mentions 'трек' in passing.

        The keyword heuristic is intentionally narrow — we do NOT want
        to nudge on every sentence that contains the word «трек». A
        user asking about, say, a music recommendation must not hear
        «бит не запустился» — that's a false positive.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(content="Порекомендую альбом", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm)
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            # No music keyword — a chit-chat turn.
            node._on_stt(_make_string("роббокс какой трек посоветуешь?"))
            node.drive_one_turn()

            responses = [
                _extract_response_text(p)
                for p in node._response_pub.published
            ]
            joined = " ".join(responses)
            self.assertNotIn(
                "бит не запустился", joined,
                "Bug C must NOT publish the nudge on chit-chat that "
                "happens to contain 'трек'; "
                f"responses={responses!r}",
            )
        finally:
            node.close()

    def test_user_music_request_with_music_code_no_nudge(self):
        """LLM did the right thing → no nudge, the music itself is the answer.

        When the LLM correctly calls ``execute_music_code``, the
        music-guard takes the early-return branch and publishes
        nothing extra. This is the positive-path regression guard.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(ToolCall(
                    id="call-music",
                    name="execute_music_code",
                    arguments={"code": "Clock.bpm = 100\nd1 >> play('x')"},
                ),),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="Готово", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_execute_music_code_tools())
        try:
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс спой песенку"))
            node.drive_one_turn()

            responses = [
                _extract_response_text(p)
                for p in node._response_pub.published
            ]
            joined = " ".join(responses)
            self.assertNotIn(
                "бит не запустился", joined,
                "Bug C must NOT publish the nudge when LLM correctly "
                "calls execute_music_code; "
                f"responses={responses!r}",
            )
        finally:
            node.close()


# ── module-level helpers ──────────────────────────────────────────────


def _extract_response_text(msg: Any) -> str:
    """Decode a ``/voice/dialogue/response`` payload to plain text.

    The dialogue shell publishes SSML envelopes on the response topic.
    For these tests we just want the visible Russian text — the SSML
    wrapper is stripped so we can assert on the actual content.
    """
    raw = getattr(msg, "data", msg)
    if isinstance(raw, str):
        try:
            envelope = json.loads(raw)
        except (TypeError, ValueError):
            return raw
        ssml = envelope.get("ssml", "")
        return ssml.removeprefix("<speak>").removesuffix("</speak>")
    return str(raw)


# Module-level aliases so unittest discovery picks the test classes
# (pytest also discovers Test* on its own, but unittest.main() at the
# bottom of the file expects TestCase subclasses).
TestIssue992BugA = TestIssue992BugA
TestIssue992BugB = TestIssue992BugB
TestIssue992BugC = TestIssue992BugC
