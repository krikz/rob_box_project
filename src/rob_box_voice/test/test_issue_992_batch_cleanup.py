#!/usr/bin/env python3
"""test_issue_992_batch_cleanup.py — issue #992 batch-tracking regression.

Issue #992 root cause: ``dialogue_node`` fires ``/mcp/music_cleanup`` on
the *first* ``/voice/tts/batch_complete`` event for a turn, even when
the LLM still has more ``speak_text`` chunks queued (e.g. the LLM
called ``speak_text`` twice in a row — the second batch's TTS is
silenced because the first batch's ``batch_complete`` already wiped
the music).

The fix introduces an ``_active_batches`` registry in
:class:`DialogueNode` that tracks every in-flight TTS ``batch_id``.
``_on_tts_finished`` registers a batch the first time it sees a
``batch_id``; ``_on_tts_batch_complete`` unregisters it. Cleanup is
published only when the registry is empty AND ``_pending_music_cleanup``
is set. Bonus: ``stop_music`` from a follow-up LLM turn is ignored
when the cleanup flag is already True (idempotent guard).

These tests drive the real W5 shell (``_TestableDialogueNode``) with
scripted LLM responses and simulate the ``/voice/tts/finished`` /
``/voice/tts/batch_complete`` traffic that ``tts_node`` would emit in
production.

Run with::

    python3 -m pytest src/rob_box_voice/test/test_issue_992_batch_cleanup.py
"""

from __future__ import annotations

import json
import sys
from typing import List

import pytest

# Reuse the shell test harness (rclpy shims + testable node).
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


def _speak_text_tools() -> FakeToolProvider:
    """FakeToolProvider with a capturing ``speak_text`` tool.

    The handler returns immediately (no TTS playback) so we can simulate
    the TTS lifecycle manually in tests by publishing
    ``/voice/tts/finished`` and ``/voice/tts/batch_complete`` events.
    """

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
        # Simulate the SpeakTextTool flow: a real call would push N TTS
        # requests via /voice/tts/request and register a batch_id. The
        # test publishes the matching tts_finished / batch_complete
        # events by hand below.
        return json.dumps({"ok": True, "batch_id": "fake-batch"})

    tools = FakeToolProvider()
    tools.register(spec, _handler)
    return tools


def _stop_music_tools() -> FakeToolProvider:
    """FakeToolProvider with a no-op ``stop_music`` tool."""

    stop_spec = ToolSpec(
        name="stop_music",
        description="Stop all music.",
        parameters={"type": "object", "properties": {}},
    )

    async def _stop_handler(_args):
        return json.dumps({"ok": True})

    tools = FakeToolProvider()
    tools.register(stop_spec, _stop_handler)
    return tools


def _simulate_batch(node: _TestableDialogueNode, batch_id: str,
                    chunks_total: int, success: bool = True) -> None:
    """End-to-end replay: register the batch and then drive it to completion.

    Combines :func:`_register_batch` and :func:`_complete_batch` for the
    simple single-batch tests below.
    """
    _register_batch(node, batch_id, chunks_total)
    _complete_batch(node, batch_id, chunks_total, success=success)


def _register_batch(node: _TestableDialogueNode, batch_id: str,
                    chunks_total: int) -> None:
    """Publish the ``batch_registered`` prelude that SpeakTextTool emits."""
    reg_cb = node._subs.get("/voice/tts/batch_registered")
    if reg_cb is None:
        raise AssertionError(
            "/voice/tts/batch_registered subscription is not registered; "
            "did dialogue_node lose the new subscription?"
        )
    reg_cb(_make_string(json.dumps({
        "batch_id": batch_id,
        "chunks_total": chunks_total,
    })))


def _complete_batch(node: _TestableDialogueNode, batch_id: str,
                    chunks_total: int, success: bool = True) -> None:
    """Replay tts_node's per-chunk ``tts_finished`` + final ``batch_complete``."""
    finished_cb = node._subs["/voice/tts/finished"]
    batch_cb = node._subs.get("/voice/tts/batch_complete")
    for i in range(1, chunks_total + 1):
        finished_msg = _make_string(json.dumps({
            "speech_id": f"speech-{batch_id}-{i}",
            "success": success,
            "batch_id": batch_id,
            "batch_index": i,
            "batch_total": chunks_total,
        }))
        finished_cb(finished_msg)
        if i == chunks_total and batch_cb is not None:
            batch_msg = _make_string(json.dumps({
                "batch_id": batch_id,
                "chunks_total": chunks_total,
                "batch_duration_ms": 1234,
                "success": success,
            }))
            batch_cb(batch_msg)


def _music_cleanup_payloads(node: _TestableDialogueNode) -> List[dict]:
    """Decode every message dialogue_node published on /mcp/music_cleanup."""
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


# ── tests ────────────────────────────────────────────────────────────


class TestIssue992BatchCleanup:
    """Regression coverage for issue #992 batch-tracking."""

    def _drive(self, node: _TestableDialogueNode) -> None:
        node._dsm.on_event(DialogueEvent.WAKE_WORD)
        node._on_stt(_make_string("роббокс спой куплет и ещё куплет"))
        node.drive_one_turn()

    def test_two_speak_text_batches_hold_cleanup_until_last(self):
        """LLM вызвал speak_text дважды — music_cleanup только после второго.

        Repro issue #992: LLM returns two ``speak_text`` tool calls in
        one cycle. The SpeakTextTool creates a separate ``batch_id`` per
        call, so the tts_node publishes TWO ``batch_complete`` events.
        Before the fix the first ``batch_complete`` fired
        ``/mcp/music_cleanup`` while the second batch's TTS was still
        playing — music cut off mid-phrase. With the fix, the cleanup
        fires only after the second batch's ``batch_complete``.

        In production both ``speak_text`` calls execute back-to-back
        inside the same LLM cycle, so both batch_registered preludes
        land before either batch's tts_finished. We mirror that here
        by registering both batches first, then draining the finished
        events for each in turn.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-1",
                        name="speak_text",
                        arguments={"text": "Первый куплет, длинный текст для TTS."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-2",
                        name="speak_text",
                        arguments={"text": "Второй куплет, тоже не короткий."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_speak_text_tools())
        try:
            self._drive(node)

            # Step 1 — both speak_text calls fire and register their
            # batches (mirrors back-to-back SpeakTextTool.execute()).
            _register_batch(node, "batch-1", chunks_total=1)
            _register_batch(node, "batch-2", chunks_total=1)
            assert len(node._active_batches) == 2, (
                f"both batches must be registered as active: "
                f"{list(node._active_batches)!r}"
            )

            # Step 2 — batch 1 finishes (finished + batch_complete).
            # Cleanup MUST NOT fire because batch 2 is still active.
            _complete_batch(node, "batch-1", chunks_total=1)
            cleanups_after_first = _music_cleanup_payloads(node)
            assert cleanups_after_first == [], (
                "music_cleanup fired after first batch_complete while "
                "second batch is still in flight (issue #992): "
                f"{cleanups_after_first!r}"
            )
            assert len(node._active_batches) == 1, (
                "second batch must still be registered as active after "
                f"first batch_complete: {list(node._active_batches)!r}"
            )

            # Step 3 — batch 2 finishes. NOW cleanup should fire.
            _complete_batch(node, "batch-2", chunks_total=1)
            cleanups_after_second = _music_cleanup_payloads(node)
            assert len(cleanups_after_second) == 1, (
                "music_cleanup must fire exactly once after the LAST "
                "batch_complete; got: "
                f"{cleanups_after_second!r}"
            )
            assert cleanups_after_second[0].get("reason") == "tts_batch_complete"
            assert len(node._active_batches) == 0, (
                "active_batches must be empty after the last "
                "batch_complete: "
                f"{list(node._active_batches)!r}"
            )
        finally:
            node.close()

    def test_single_speak_text_batch_fires_cleanup_normally(self):
        """One speak_text call → batch_complete → cleanup (back-compat).

        The fix must NOT regress the single-batch case: with exactly one
        batch in flight, ``music_cleanup`` must fire after its
        ``batch_complete`` (as in issue #980's behaviour).
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-1",
                        name="speak_text",
                        arguments={"text": "Один куплет."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_speak_text_tools())
        try:
            self._drive(node)
            _simulate_batch(node, "batch-only", chunks_total=1)
            cleanups = _music_cleanup_payloads(node)
            assert len(cleanups) == 1, (
                f"single speak_text must trigger exactly one cleanup; "
                f"got: {cleanups!r}"
            )
            assert cleanups[0].get("reason") == "tts_batch_complete"
        finally:
            node.close()

    def test_multi_chunk_single_batch_fires_cleanup_after_last_chunk(self):
        """One speak_text → 3 chunks → cleanup only after chunk 3/3.

        Sanity check that the chunk count doesn't break the registry:
        the FIRST chunk's tts_finished registers the batch, the
        batch_complete on chunk 3 unregisters it. Even with 3 chunks
        inside ONE batch, cleanup fires once at the end.
        """
        llm = _ScriptedLLMProvider([
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-1",
                        name="speak_text",
                        arguments={"text": "Длинный текст на 3 чанка."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_speak_text_tools())
        try:
            self._drive(node)
            _simulate_batch(node, "multi-chunk", chunks_total=3)
            cleanups = _music_cleanup_payloads(node)
            assert len(cleanups) == 1, (
                f"single multi-chunk batch must fire exactly one cleanup; "
                f"got: {cleanups!r}"
            )
        finally:
            node.close()

    def test_stop_music_when_already_pending_is_ignored(self):
        """Bonus: stop_music от LLM игнорируется если cleanup уже pending.

        A second LLM turn arrives while a previous turn's cleanup is
        still pending (e.g. its TTS batch hasn't finished yet). The LLM
        calls ``stop_music`` again — we must NOT log it as a fresh
        "deferred" event, because the flag is already set and the
        original cleanup will fire as soon as the batches drain.
        """
        llm = _ScriptedLLMProvider([
            # First turn: stop_music deferred
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-1",
                        name="stop_music",
                        arguments={},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="ok", finish_reason="stop"),
        ])
        node = _TestableDialogueNode(llm=llm, tools=_stop_music_tools())
        try:
            # Drive first turn — this sets _pending_music_cleanup=True
            # and the test logs would normally emit "stop_music deferred".
            node._dsm.on_event(DialogueEvent.WAKE_WORD)
            node._on_stt(_make_string("роббокс выключи музыку"))
            node.drive_one_turn()
            assert node._pending_music_cleanup is True

            # Drive a second turn with stop_music again. The second
            # invocation must NOT log a fresh "stop_music deferred"
            # message — the cleanup is already pending. We assert via
            # the public state: the flag stays True, and the warn/info
            # log "stop_music deferred" is logged only once.
            deferred_logs_before = sum(
                1 for call in node._logger.info.call_args_list
                if call.args and "stop_music deferred" in str(call.args[0])
            )
            llm.push(LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call-2",
                        name="stop_music",
                        arguments={},
                    ),
                ),
                finish_reason="tool_calls",
            ))
            llm.push(LLMResponse(content="ok", finish_reason="stop"))
            node._on_stt(_make_string("роббокс выключи музыку ещё раз"))
            node.drive_one_turn()

            deferred_logs_after = sum(
                1 for call in node._logger.info.call_args_list
                if call.args and "stop_music deferred" in str(call.args[0])
            )
            assert deferred_logs_after == deferred_logs_before, (
                "stop_music deferred log must not fire a second time "
                "while _pending_music_cleanup is already True"
            )
            assert node._pending_music_cleanup is True
        finally:
            node.close()


if __name__ == "__main__":  # pragma: no cover
    sys.exit(pytest.main([__file__, "-v"]))
