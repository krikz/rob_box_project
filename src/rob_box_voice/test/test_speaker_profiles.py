#!/usr/bin/env python3
"""
test_speaker_profiles.py — Pure-Python тесты профилей спикеров (issue #1077).

Проверяет:
- SpeakerTracker: подтверждение tag после 2+ фраз подряд, защита от
  нестабильных tags (0/1 чередование), короткие фразы (<0.8с) не создают
  профиль, смена tag сбрасывает streak.
- extract_speaker_name: «меня зовут X» / «зовут меня X» / «моё имя X».
- format_speaker_context: профиль + факты → LLM-контекст.

Модуль НЕ импортирует rclpy/vosk/grpc — только stdlib.
"""

from __future__ import annotations

from rob_box_voice.speaker_profiles import (
    DEFAULT_MIN_DURATION_S,
    DEFAULT_MIN_PHRASES,
    SpeakerTracker,
    extract_speaker_name,
    format_speaker_context,
)


class TestSpeakerTracker:
    def test_defaults(self):
        t = SpeakerTracker()
        assert t.min_phrases == DEFAULT_MIN_PHRASES == 2
        assert t.min_duration_s == DEFAULT_MIN_DURATION_S == 0.8

    def test_no_tag_never_confirms(self):
        t = SpeakerTracker()
        assert t.note_phrase(None, 2.0) is False
        assert t.note_phrase("", 2.0) is False
        assert t.is_confirmed(None) is False

    def test_two_consecutive_phrases_confirm(self):
        t = SpeakerTracker()
        assert t.note_phrase("0", 1.5) is False  # первая фраза — streak=1
        assert t.is_confirmed("0") is False
        assert t.note_phrase("0", 1.2) is True  # вторая — подтверждение
        assert t.is_confirmed("0") is True
        # Третья фраза того же tag — уже подтверждён, False.
        assert t.note_phrase("0", 1.0) is False
        assert t.is_confirmed("0") is True

    def test_short_phrase_does_not_confirm(self):
        t = SpeakerTracker()
        assert t.note_phrase("0", 0.5) is False  # < 0.8с
        assert t.is_confirmed("0") is False
        # Короткая фраза не считается: ещё две длинные — подтверждение.
        assert t.note_phrase("0", 1.0) is False
        assert t.note_phrase("0", 1.0) is True
        assert t.is_confirmed("0") is True

    def test_short_phrase_does_not_reset_streak(self):
        t = SpeakerTracker()
        t.note_phrase("0", 1.5)
        # Короткая фраза того же tag не сбрасывает streak.
        assert t.note_phrase("0", 0.4) is False
        assert t.note_phrase("0", 1.5) is True  # streak=2 → подтверждён

    def test_unstable_tag_never_confirms(self):
        """Edge case #1: Yandex разбивает один голос на tag='0' и '1'."""
        t = SpeakerTracker()
        # Чередование 0/1 — ни один tag не набирает 2 подряд.
        assert t.note_phrase("0", 1.0) is False
        assert t.note_phrase("1", 1.0) is False
        assert t.note_phrase("0", 1.0) is False
        assert t.note_phrase("1", 1.0) is False
        assert t.is_confirmed("0") is False
        assert t.is_confirmed("1") is False

    def test_speaker_change_resets_previous_streak(self):
        """Edge case #3: смена спикера — профили не смешиваются."""
        t = SpeakerTracker()
        t.note_phrase("0", 1.5)
        t.note_phrase("1", 1.5)
        t.note_phrase("1", 1.5)  # спикер 1 подтверждён
        assert t.is_confirmed("1") is True
        # Спикер 0 сброшен (был 1, потом 1 → 0).
        assert t.is_confirmed("0") is False
        assert t.note_phrase("0", 1.5) is False  # streak=1 заново
        assert t.note_phrase("0", 1.5) is True  # подтверждён


class TestExtractSpeakerName:
    def test_menya_zovut(self):
        assert extract_speaker_name("меня зовут Саша") == "Саша"

    def test_zovut_menya(self):
        assert extract_speaker_name("зовут меня саша") == "Саша"

    def test_moe_imya(self):
        assert extract_speaker_name("моё имя Алекс") == "Алекс"

    def test_no_name_pattern(self):
        assert extract_speaker_name("привет робот") is None

    def test_empty_text(self):
        assert extract_speaker_name("") is None
        assert extract_speaker_name(None) is None

    def test_wake_word_prefix(self):
        # wake word уже срезан dialogue_node, но и с ним работает.
        assert extract_speaker_name("робот меня зовут Лена") == "Лена"

    def test_name_in_sentence(self):
        assert extract_speaker_name(
            "привет, меня зовут Игорь, расскажи анекдот"
        ) == "Игорь"


class TestFormatSpeakerContext:
    def test_profile_with_name(self):
        profile = {"name": "Саша", "dialog_count": 3}
        ctx = format_speaker_context(profile, [])
        assert ctx is not None
        assert "Саша" in ctx
        assert "3-й диалог" in ctx

    def test_is_new_flag(self):
        profile = {"dialog_count": 1}
        ctx = format_speaker_context(profile, [], is_new=True)
        assert "НОВЫЙ собеседник" in ctx

    def test_facts_rendered(self):
        from rob_box_harness.memory import Fact

        profile = {"dialog_count": 2}
        facts = [
            Fact(key="music", value="джаз", tags=("speaker",)),
            Fact(key="name", value="Пётр", tags=("speaker",)),
        ]
        ctx = format_speaker_context(profile, facts)
        assert "джаз" in ctx
        assert "Пётр" in ctx

    def test_empty_returns_none(self):
        assert format_speaker_context({}, []) is None
        assert format_speaker_context(None, None) is None
