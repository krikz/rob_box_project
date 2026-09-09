#!/usr/bin/env python3
"""
Unit tests for SpeechFormatter

Tests text formatting, accent placement, SSML processing,
and text cleanup for TTS without ROS dependencies.
"""

import pytest
from rob_box_voice.core.speech_formatter import SpeechFormatter


class TestSpeechFormatterBasics:
    """Test basic formatter initialization and stats."""

    def test_init_default(self):
        """Test default initialization."""
        formatter = SpeechFormatter()

        assert formatter.enable_accents is True

    def test_init_without_accents(self):
        """Test initialization with accents disabled."""
        formatter = SpeechFormatter(enable_accents=False)

        assert formatter.enable_accents is False
        assert formatter.accent_replacer is None

    def test_get_stats(self):
        """Test stats retrieval."""
        formatter = SpeechFormatter()
        stats = formatter.get_stats()

        assert 'accents_enabled' in stats
        assert 'accent_replacer_loaded' in stats


class TestSpeechFormatterCleaning:
    """Test text cleaning functionality."""

    def test_clean_extra_whitespace(self):
        """Test removal of extra whitespace."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.clean_for_speech('Привет   мир')
        assert result == 'Привет мир'

        result = formatter.clean_for_speech('  Текст  с  пробелами  ')
        assert result == 'Текст с пробелами'

    def test_clean_markdown_bold(self):
        """Test removal of markdown bold."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.clean_for_speech('**Важно**: сообщение')
        assert result == 'Важно: сообщение'

        result = formatter.clean_for_speech('__Также важно__')
        assert result == 'Также важно'

    def test_clean_markdown_italic(self):
        """Test removal of markdown italic."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.clean_for_speech('*курсив* текст')
        assert result == 'курсив текст'

        result = formatter.clean_for_speech('_курсив_')
        assert result == 'курсив'

    def test_clean_urls(self):
        """Test removal of URLs."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.clean_for_speech('Смотри https://example.com тут')
        assert result == 'Смотри тут'

        result = formatter.clean_for_speech('Проверь http://test.ru ссылку')
        assert result == 'Проверь ссылку'

    def test_clean_multiple_punctuation(self):
        """Test cleanup of multiple punctuation marks."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.clean_for_speech('Точки.........тут')
        assert result == 'Точки...тут'

        result = formatter.clean_for_speech('Wow!!!!!!')
        assert result == 'Wow!'

        result = formatter.clean_for_speech('Что???')
        assert result == 'Что?'


class TestSpeechFormatterSSML:
    """Test SSML processing."""

    def test_is_ssml_true(self):
        """Test SSML detection for valid SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        assert formatter.is_ssml('<speak>Привет</speak>') is True
        assert formatter.is_ssml('<speak><prosody rate="fast">Быстро</prosody></speak>') is True

    def test_is_ssml_false(self):
        """Test SSML detection for plain text."""
        formatter = SpeechFormatter(enable_accents=False)

        assert formatter.is_ssml('Привет') is False
        assert formatter.is_ssml('Текст без тегов') is False
        assert formatter.is_ssml('<speak>Только открывающий') is False

    def test_wrap_in_ssml(self):
        """Test wrapping text in SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.wrap_in_ssml('Привет')
        assert result == '<speak>Привет</speak>'

    def test_wrap_in_ssml_already_wrapped(self):
        """Test wrapping already wrapped text."""
        formatter = SpeechFormatter(enable_accents=False)

        input_text = '<speak>Уже обернут</speak>'
        result = formatter.wrap_in_ssml(input_text)
        assert result == input_text  # Should not double-wrap

    def test_extract_from_ssml(self):
        """Test extracting text from SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.extract_from_ssml('<speak>Привет</speak>')
        assert result == 'Привет'

    def test_extract_from_ssml_with_tags(self):
        """Test extracting from SSML with inner tags."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.extract_from_ssml('<speak><prosody rate="fast">Быстро</prosody></speak>')
        assert result == 'Быстро'

    def test_extract_from_plain_text(self):
        """Test extracting from plain text."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.extract_from_ssml('Просто текст')
        assert result == 'Просто текст'


class TestSpeechFormatterFormatting:
    """Test main formatting method."""

    def test_format_for_tts_basic(self):
        """Test basic formatting without SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('  Привет   мир  ')
        assert result == 'Привет мир'

    def test_format_for_tts_with_ssml(self):
        """Test formatting with SSML wrapping."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('Привет', add_ssml=True)
        assert result == '<speak>Привет</speak>'

    def test_format_for_tts_clean_and_wrap(self):
        """Test formatting with cleaning and SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('**Важно**: привет', add_ssml=True)
        assert result == '<speak>Важно: привет</speak>'

    def test_format_for_tts_preserves_ssml(self):
        """Test that existing SSML is preserved."""
        formatter = SpeechFormatter(enable_accents=False)

        input_text = '<speak>Уже готово</speak>'
        result = formatter.format_for_tts(input_text, add_ssml=True)
        assert result == input_text


class TestSpeechFormatterAccents:
    """Test accent placement (if available)."""

    def test_add_accents_without_replacer(self):
        """Test accent addition when replacer not available."""
        formatter = SpeechFormatter(enable_accents=False)

        text = 'Привет мир'
        result = formatter.add_accents(text)
        assert result == text  # Should return unchanged

    def test_add_accents_with_ssml(self):
        """Test accent addition with SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        # Even without accent replacer, SSML structure should be preserved
        input_text = '<speak>Текст</speak>'
        result = formatter.add_accents(input_text)

        # Should still be valid SSML
        assert formatter.is_ssml(result)

    def test_format_for_tts_with_accents_enabled(self):
        """Test full formatting with accents enabled."""
        formatter = SpeechFormatter(enable_accents=True)

        # Test that it doesn't crash even if accent_replacer fails to load
        result = formatter.format_for_tts('Привет мир')
        assert isinstance(result, str)
        assert len(result) > 0


class TestSpeechFormatterEdgeCases:
    """Test edge cases and error handling."""

    def test_empty_string(self):
        """Test with empty string."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('')
        assert result == ''

    def test_only_whitespace(self):
        """Test with only whitespace."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('   ')
        assert result == ''

    def test_special_characters(self):
        """Test with special characters."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('Тест @#$ символы')
        assert 'Тест' in result
        assert 'символы' in result

    def test_mixed_languages(self):
        """Test with mixed language text."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('Hello мир')
        assert result == 'Hello мир'

    def test_numbers(self):
        """Test with numbers."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('Температура 25 градусов')
        assert '25' in result
        assert 'Температура' in result


class TestSpeechFormatterIntegration:
    """Integration tests combining multiple features."""

    def test_full_pipeline_without_ssml(self):
        """Test complete formatting pipeline."""
        formatter = SpeechFormatter(enable_accents=False)

        input_text = '  **Важно**:  проверь  https://test.com  сейчас!!!  '
        result = formatter.format_for_tts(input_text)

        assert result == 'Важно: проверь сейчас!'

    def test_full_pipeline_with_ssml(self):
        """Test complete pipeline with SSML."""
        formatter = SpeechFormatter(enable_accents=False)

        input_text = '**Важно**: сообщение'
        result = formatter.format_for_tts(input_text, add_ssml=True)

        assert result == '<speak>Важно: сообщение</speak>'

    def test_preserve_important_punctuation(self):
        """Test that important punctuation is preserved."""
        formatter = SpeechFormatter(enable_accents=False)

        result = formatter.format_for_tts('Вопрос? Да! Может быть...')
        assert '?' in result
        assert '!' in result
        assert '...' in result


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
