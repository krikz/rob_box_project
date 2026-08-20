"""test_strip_speaker_tag.py — Unit tests for ``strip_speaker_tag``.

Regression for the 20.08 DJ incident: the LLM copied the voice-channel
routing marker ``[Spkr:<имя>]`` into its final reply, so the internal
``[CRITICAL]`` retry prompt started with ``[Spkr:Эйджик]`` and slipped
past the service-text guard — the robot spoke the internal instruction
aloud. ``strip_speaker_tag`` removes the leading routing marker so the
guard can recognise what follows.

Pure Python — no ROS, no rclpy, no heavy deps.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_strip_speaker_tag.py
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.speak_helpers import strip_speaker_tag


class TestStripSpeakerTag:
    def test_leading_tag_removed(self):
        assert strip_speaker_tag("[Spkr:Эйджик] Привет!") == "Привет!"

    def test_unmasks_service_marker(self):
        # The exact incident shape: speaker tag + [CRITICAL] retry prompt.
        text = (
            "[Spkr:Эйджик] [CRITICAL] В прошлом цикле ты НЕ вызвал "
            "ни один музыкальный тул."
        )
        assert strip_speaker_tag(text) == (
            "[CRITICAL] В прошлом цикле ты НЕ вызвал ни один музыкальный тул."
        )

    def test_repeated_leading_tags_removed(self):
        assert strip_speaker_tag("[Spkr:a] [Spkr:b] [CRITICAL] x") == "[CRITICAL] x"

    def test_case_insensitive(self):
        assert strip_speaker_tag("[spkr:эйджик] Привет") == "Привет"

    def test_marker_inside_text_kept(self):
        text = "Привет [Spkr:Эйджик] как дела"
        assert strip_speaker_tag(text) == text

    def test_no_tag_unchanged(self):
        assert strip_speaker_tag("Привет, как дела?") == "Привет, как дела?"

    def test_only_tag_becomes_empty(self):
        assert strip_speaker_tag("[Spkr:Эйджик]") == ""

    def test_empty_and_non_string_passthrough(self):
        assert strip_speaker_tag("") == ""
        assert strip_speaker_tag(None) is None  # type: ignore[arg-type]
        assert strip_speaker_tag(42) == 42  # type: ignore[arg-type]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
