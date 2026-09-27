"""Unit-тесты SpeechAccumulator — аккумулятор фоновой речи без wake-слова."""

import time

from rob_box_voice.core.speech_accumulator import (
    SpeechAccumulator,
    format_ago_s,
)


class TestFormatAgoS:
    def test_seconds(self):
        assert format_ago_s(0) == "0с"
        assert format_ago_s(12) == "12с"
        assert format_ago_s(59) == "59с"

    def test_minutes(self):
        assert format_ago_s(60) == "1м0с"
        assert format_ago_s(185) == "3м5с"

    def test_hours(self):
        assert format_ago_s(3700) == "1ч1м"


class TestSpeechAccumulator:
    def test_empty_block_returns_none(self):
        acc = SpeechAccumulator()
        assert acc.format_block() is None

    def test_add_and_format(self):
        acc = SpeechAccumulator()
        acc.add("расскажи про погоду", speaker_tag="0", speaker_name="Антон")
        block = acc.format_block()
        assert block is not None
        assert "<speech_backlog>" in block
        assert 'speaker="Антон"' in block
        assert "расскажи про погоду" in block

    def test_unknown_speaker_defaults_to_neznakomets(self):
        acc = SpeechAccumulator()
        acc.add("привет", speaker_tag=None, speaker_name=None)
        block = acc.format_block()
        assert 'speaker="незнакомец"' in block

    def test_empty_text_ignored(self):
        acc = SpeechAccumulator()
        acc.add("   ", speaker_tag="0", speaker_name="Антон")
        assert acc.is_empty()

    def test_prune_expired_entries(self):
        acc = SpeechAccumulator(window_sec=60.0)
        now = time.time()
        acc.add("старая фраза", speaker_tag="0", speaker_name="Антон")
        # подменяем timestamp записи в прошлое
        acc._entries[0]["ts"] = now - 120.0
        acc.prune(now=now)
        assert acc.is_empty()

    def test_max_entries_cap(self):
        acc = SpeechAccumulator(max_entries=2)
        for i in range(5):
            acc.add(f"фраза {i}", speaker_tag="0", speaker_name="Антон")
        assert len(acc._entries) == 2
        assert acc._entries[0]["text"] == "фраза 3"

    def test_clear(self):
        acc = SpeechAccumulator()
        acc.add("привет", speaker_tag="0", speaker_name="Антон")
        acc.clear()
        assert acc.is_empty()

    def test_xml_escape(self):
        acc = SpeechAccumulator()
        acc.add('а < b & c > "d"', speaker_tag="0", speaker_name="Антон")
        block = acc.format_block()
        assert "&lt;" in block and "&gt;" in block and "&amp;" in block

    def test_format_user_hint(self):
        acc = SpeechAccumulator()
        acc.add("напомни позвонить маме", speaker_tag="0", speaker_name="Антон")
        hint = acc.format_user_hint()
        assert hint is not None
        assert "ФОНОВЫЙ ЗАПРОС" in hint
        assert "напомни позвонить маме" in hint
        assert "Антон" in hint
        # Issue #1766 — усиленная инструкция: priority над историей + LRU.
        assert "ВАЖНО: backlog ИМЕЕТ ПРИОРИТЕТ над историей диалога" in hint
        assert "ВЫПОЛНИ последнюю явную команду" in hint
        assert "выполни ПОСЛЕДНЮЮ (LRU)" in hint
        assert "[URGENT_BACKLOG]" in hint

    def test_format_user_hint_empty_returns_none(self):
        acc = SpeechAccumulator()
        assert acc.format_user_hint() is None

    # Issue #1766 — инструкция для LLM в <system_context> требует
    # явного приоритета backlog над историей + [URGENT_BACKLOG] маркер.
    # Без этого LLM «застревает» на старой теме диалога и игнорирует
    # backlog, в котором лежит явная команда («включи трек про весну»).
    def test_format_block_has_urgent_marker(self):
        acc = SpeechAccumulator()
        acc.add("включи трек про весну", speaker_tag="0", speaker_name="Денчик")
        block = acc.format_block()
        assert block is not None
        assert "[URGENT_BACKLOG]" in block
        assert "включи трек про весну" in block

    def test_format_block_instructs_priority_over_history(self):
        acc = SpeechAccumulator()
        acc.add("включи трек про весну", speaker_tag="0", speaker_name="Денчик")
        block = acc.format_block()
        # Приоритет backlog над историей — главная мотивация issue #1766.
        assert "backlog ИМЕЕТ ПРИОРИТЕТ над историей диалога" in block

    def test_format_block_instructs_lru_for_multiple_commands(self):
        """Если в backlog несколько явных команд — выполни последнюю (LRU)."""
        acc = SpeechAccumulator()
        acc.add("включи джаз", speaker_tag="0", speaker_name="Денчик")
        acc.add("поставь рок", speaker_tag="0", speaker_name="Денчик")
        acc.add("вруби техно", speaker_tag="0", speaker_name="Денчик")
        block = acc.format_block()
        assert "выполни ПОСЛЕДНЮЮ (LRU)" in block
        # Все три записи в block присутствуют (порядок LRU = insertion order).
        assert "включи джаз" in block
        assert "поставь рок" in block
        assert "вруби техно" in block

    def test_format_user_hint_instructs_priority_over_history(self):
        acc = SpeechAccumulator()
        acc.add("включи трек про весну", speaker_tag="0", speaker_name="Денчик")
        hint = acc.format_user_hint()
        assert hint is not None
        assert "backlog ИМЕЕТ ПРИОРИТЕТ над историей диалога" in hint
        assert "выполни ПОСЛЕДНЮЮ (LRU)" in hint

    def test_format_user_hint_lists_bare_wake_examples(self):
        """Документирует в user-hint «голые wake» паттерны, чтобы LLM
        не трактовала их как новую тему диалога (live-баг #1766:
        «давай зарегистрируем» сбивал LLM на тему идентификации)."""
        acc = SpeechAccumulator()
        acc.add("включи трек про весну", speaker_tag="0", speaker_name="Денчик")
        hint = acc.format_user_hint()
        assert "робот" in hint
        assert "повтори" in hint
        assert "что думаешь" in hint
        assert "давай зарегистрируем" in hint
