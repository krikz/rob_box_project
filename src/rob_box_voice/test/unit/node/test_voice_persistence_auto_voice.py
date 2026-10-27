"""test_voice_persistence_auto_voice.py — голос в полезной нагрузке SSML.

``build_ssml_payload`` несёт ``voice``, когда он задан, и не несёт без него (обратная совместимость).

PR-15 (#3711, часть 15p): проверки публикации ответа нодой диалога v1 (``_publish_response`` /
``_publish_response_batch``) удалены вместе с ней; текущий голос ведёт tts_node (VoiceOwner, PR-4 ч.3).
"""

from __future__ import annotations

import json

from rob_box_voice.core.speak_helpers import build_ssml_payload


class TestBuildSsmlPayloadVoice:
    def test_includes_voice_when_provided(self):
        payload = json.loads(
            build_ssml_payload("привет", voice="Russian_ReliableMan")
        )
        assert payload["voice"] == "Russian_ReliableMan"

    def test_omits_voice_by_default(self):
        payload = json.loads(build_ssml_payload("привет"))
        assert "voice" not in payload
