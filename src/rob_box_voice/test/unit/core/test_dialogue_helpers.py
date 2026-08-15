"""Unit tests for :mod:`rob_box_voice.core.dialogue_helpers` (TD-1 extraction).

Covers the pure helper functions extracted from ``dialogue_node.py``:

* emotion → animation mapping,
* offline fallback replies,
* volume / pitch intent detection.

These are pure-Python functions — no ROS2 node required.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.dialogue_helpers import (
    EMOTION_TO_ANIMATION,
    detect_pitch_intent,
    detect_volume_intent,
    generate_fallback_response,
    map_emotion_to_animation,
)


# ---------------------------------------------------------------------------
# map_emotion_to_animation
# ---------------------------------------------------------------------------


class TestMapEmotionToAnimation:
    def test_known_emotions_mapped_correctly(self) -> None:
        assert map_emotion_to_animation("happy") == "happy"
        assert map_emotion_to_animation("sad") == "sad"
        assert map_emotion_to_animation("angry") == "angry"
        assert map_emotion_to_animation("surprised") == "surprised"
        assert map_emotion_to_animation("thinking") == "thinking"
        assert map_emotion_to_animation("excited") == "victory"
        assert map_emotion_to_animation("confused") == "thinking"
        assert map_emotion_to_animation("worried") == "sad"

    def test_neutral_and_calm_map_to_idle(self) -> None:
        assert map_emotion_to_animation("neutral") == "idle"
        assert map_emotion_to_animation("calm") == "idle"

    def test_case_insensitive(self) -> None:
        assert map_emotion_to_animation("HAPPY") == "happy"
        assert map_emotion_to_animation("Excited") == "victory"

    def test_unknown_and_empty(self) -> None:
        assert map_emotion_to_animation("bogus") == "idle"
        assert map_emotion_to_animation("") == "idle"
        assert map_emotion_to_animation(None) == "idle"  # type: ignore[arg-type]

    def test_table_smoke(self) -> None:
        assert EMOTION_TO_ANIMATION


# ---------------------------------------------------------------------------
# generate_fallback_response
# ---------------------------------------------------------------------------


class TestGenerateFallbackResponse:
    def test_greetings(self) -> None:
        out = generate_fallback_response("привет")
        assert "Привет!" in out
        assert "интернет" in out.lower() or "недоступен" in out.lower()

    def test_how_are_you(self) -> None:
        out = generate_fallback_response("как дела")
        assert "хорошо" in out.lower()

    def test_thanks(self) -> None:
        assert generate_fallback_response("спасибо") == "Пожалуйста!"
        assert generate_fallback_response("благодарю") == "Пожалуйста!"

    def test_farewells(self) -> None:
        assert generate_fallback_response("пока") == "До свидания!"
        assert generate_fallback_response("до свидания") == "До свидания!"

    def test_unknown(self) -> None:
        out = generate_fallback_response("расскажи про космос")
        assert "недоступен" in out.lower()

    def test_empty(self) -> None:
        assert "недоступен" in generate_fallback_response("").lower()
        assert "недоступен" in generate_fallback_response(None).lower()  # type: ignore[arg-type]

    def test_case_insensitive(self) -> None:
        assert generate_fallback_response("СПАСИБО") == "Пожалуйста!"


# ---------------------------------------------------------------------------
# detect_volume_intent
# ---------------------------------------------------------------------------


class TestDetectVolumeIntent:
    def test_max(self) -> None:
        assert detect_volume_intent("максимальная громкость") == "max"
        assert detect_volume_intent("максимум громкости") == "max"
        assert detect_volume_intent("говори громко") == "max"

    def test_normal(self) -> None:
        assert detect_volume_intent("нормальная громкость") == "normal"
        assert detect_volume_intent("стандартная громкость") == "normal"
        assert detect_volume_intent("обычная громкость") == "normal"

    def test_louder_quieter(self) -> None:
        assert detect_volume_intent("говори громче") == "louder"
        assert detect_volume_intent("громко") == "louder"
        assert detect_volume_intent("тише") == "quieter"
        assert detect_volume_intent("потише") == "quieter"

    def test_no_intent(self) -> None:
        assert detect_volume_intent("расскажи стих") is None
        assert detect_volume_intent("") is None
        assert detect_volume_intent(None) is None  # type: ignore[arg-type]

    def test_case_insensitive(self) -> None:
        assert detect_volume_intent("ГРОМЧЕ") == "louder"


# ---------------------------------------------------------------------------
# detect_pitch_intent
# ---------------------------------------------------------------------------


class TestDetectPitchIntent:
    def test_normal(self) -> None:
        assert detect_pitch_intent("нормальный голос") == "normal"
        assert detect_pitch_intent("говори нормально") == "normal"
        assert detect_pitch_intent("обычный голос") == "normal"

    def test_higher_lower(self) -> None:
        assert detect_pitch_intent("говори выше") == "higher"
        assert detect_pitch_intent("повысь голос") == "higher"
        assert detect_pitch_intent("ниже") == "lower"

    def test_no_intent(self) -> None:
        assert detect_pitch_intent("расскажи стих") is None
        assert detect_pitch_intent("") is None
        assert detect_pitch_intent(None) is None  # type: ignore[arg-type]

    def test_case_insensitive(self) -> None:
        assert detect_pitch_intent("ВЫШЕ") == "higher"
