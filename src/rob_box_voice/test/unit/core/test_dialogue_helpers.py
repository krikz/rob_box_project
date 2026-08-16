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
    format_llm_skipped_summary,
    generate_fallback_response,
    map_emotion_to_animation,
    sanitize_speaker_name,
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


# ---------------------------------------------------------------------------
# sanitize_speaker_name (issue #1077 / #1101 — junk speaker names)
# ---------------------------------------------------------------------------


class TestSanitizeSpeakerName:
    def test_clean_name_passthrough(self) -> None:
        assert sanitize_speaker_name("Антон") == "Антон"
        assert sanitize_speaker_name("  Антон  ") == "Антон"

    def test_junk_names_normalized_to_empty(self) -> None:
        for junk in ("null", "None", "UNDEFINED", "Unknown", "", "  null  "):
            assert sanitize_speaker_name(junk) == "", junk

    def test_none_normalized_to_empty(self) -> None:
        assert sanitize_speaker_name(None) == ""

    def test_real_names_not_mangled(self) -> None:
        # "NullPointer" contains "null" as prefix but is a real name.
        assert sanitize_speaker_name("NullPointer") == "NullPointer"
        assert sanitize_speaker_name("Никто") == "Никто"


# ---------------------------------------------------------------------------
# format_llm_skipped_summary (issue #1101 — diagnostics)
# ---------------------------------------------------------------------------


class TestFormatLlmSkippedSummary:
    def test_empty_counter_returns_none(self) -> None:
        assert format_llm_skipped_summary({}) is None

    def test_all_zero_returns_none(self) -> None:
        counter = {"no_wake_word": 0, "silenced": 0}
        assert format_llm_skipped_summary(counter) is None

    def test_nonzero_counter_returns_summary(self) -> None:
        summary = format_llm_skipped_summary(
            {"no_wake_word": 3, "silenced": 0, "stt_rejected": 2},
            window_s=300.0,
        )
        assert summary is not None
        assert "llm_skipped_total=5" in summary
        assert "no_wake_word=3" in summary
        assert "stt_rejected=2" in summary
        assert "silenced" not in summary  # zero-count reasons omitted
        assert "last 300s" in summary

    def test_window_seconds_rendered(self) -> None:
        summary = format_llm_skipped_summary({"silenced": 1}, window_s=60.0)
        assert summary is not None
        assert "last 60s" in summary

    def test_unicode_prefix_present(self) -> None:
        summary = format_llm_skipped_summary({"silenced": 1})
        assert summary is not None
        assert summary.startswith("📊 [diagnostics]")

