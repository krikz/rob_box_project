"""test_voice_persistence_auto_voice.py — voice persistence in the auto-voice path.

Regression guard for the bug where ``set_voice('Russian_ReliableMan')`` was
persisted only inside mcp_server's ``VoiceStateStore`` (used by the
``speak_text`` MCP tool) but **dropped** by the dialogue_node auto-voice path.

When the LLM answers with plain text (no ``speak_text`` tool call),
``dialogue_node`` voices the reply itself through
``_publish_response()`` / ``_publish_response_batch()`` -> ``build_ssml_payload()``.
That path never carried ``voice``, so ``tts_node`` received ``voice=None`` and
fell back to the provider default (``male-qn-qingse``) — while the LLM kept
claiming it speaks ``Russian_ReliableMan`` (its ``[TTS] current_voice`` context).

Checks:

1. ``build_ssml_payload`` includes ``voice`` when provided and omits it otherwise
   (back-compat).
2. ``_publish_response`` carries ``self._current_tts_voice`` into the payload.
3. ``_publish_response_batch`` carries it into every chunk of the batch.
"""

from __future__ import annotations

import json
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.speak_helpers import build_ssml_payload
from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  build_ssml_payload: voice in / out
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildSsmlPayloadVoice:
    def test_includes_voice_when_provided(self):
        payload = json.loads(
            build_ssml_payload("привет", voice="Russian_ReliableMan")
        )
        assert payload["voice"] == "Russian_ReliableMan"

    def test_omits_voice_by_default(self):
        payload = json.loads(build_ssml_payload("привет"))
        assert "voice" not in payload


# ─────────────────────────────────────────────────────────────────────────────
#  _publish_response / _publish_response_batch: current_voice propagation
# ─────────────────────────────────────────────────────────────────────────────

def _bare_node(**attrs):
    n = object.__new__(DialogueNode)
    n._active_tg_chat_id = None
    n._current_tts_voice = None
    n._response_pub = MagicMock()
    n._register_active_batch = MagicMock()
    for key, value in attrs.items():
        setattr(n, key, value)
    return n


class TestPublishResponseVoice:
    def test_single_chunk_carries_current_tts_voice(self):
        n = _bare_node(_current_tts_voice="Russian_ReliableMan")
        n._publish_response("привет")
        payload = json.loads(n._response_pub.publish.call_args.args[0].data)
        assert payload["voice"] == "Russian_ReliableMan"

    def test_single_chunk_without_current_voice_omits_field(self):
        n = _bare_node(_current_tts_voice=None)
        n._publish_response("привет")
        payload = json.loads(n._response_pub.publish.call_args.args[0].data)
        assert "voice" not in payload

    def test_batch_carries_current_tts_voice_on_every_chunk(self):
        n = _bare_node(_current_tts_voice="Russian_ReliableMan")
        n._publish_response_batch(["один", "два", "три"])
        # 3 chunks published, each must carry the persisted voice.
        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        assert len(published) == 3
        for raw in published:
            assert json.loads(raw)["voice"] == "Russian_ReliableMan"

    def test_batch_without_current_voice_omits_field(self):
        n = _bare_node(_current_tts_voice=None)
        n._publish_response_batch(["один", "два"])
        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        assert len(published) == 2
        for raw in published:
            assert "voice" not in json.loads(raw)
