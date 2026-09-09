"""test_strip_markdown.py — Unit tests for ``strip_markdown`` (issue #988).

The LLM wraps poems / rap / emphasis in Markdown (``*Жил да был енот
весёлый,*``). TTS reads the literal ``*`` as «звёздочка». These tests
pin the sanitisation rules so the robot sings the words, not the
punctuation.

Pure Python — no ROS, no rclpy, no heavy deps.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_strip_markdown.py
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.speak_helpers import strip_markdown


class TestStripMarkdownEmphasis:
    def test_italic_asterisks_removed(self):
        assert strip_markdown("*Жил да был енот весёлый,*") == "Жил да был енот весёлый,"

    def test_bold_asterisks_removed(self):
        assert strip_markdown("**Повторю слова:**") == "Повторю слова:"

    def test_underscore_italic_removed(self):
        assert strip_markdown("_полосатый, озорной._") == "полосатый, озорной."

    def test_underscore_bold_removed(self):
        assert strip_markdown("__важно__") == "важно"

    def test_multi_line_poem(self):
        text = (
            "А вот и песенка про енотика! 🎶🦝\n\n"
            "*Жил да был енот весёлый,*\n"
            "*полосатый, озорной.*"
        )
        cleaned = strip_markdown(text)
        assert "*" not in cleaned
        assert "Жил да был енот весёлый," in cleaned
        assert "полосатый, озорной." in cleaned
        assert "песенка про енотика" in cleaned

    def test_lone_asterisk_preserved(self):
        # Multiplication is not emphasis — keep it.
        assert strip_markdown("2 * 3 = 6") == "2 * 3 = 6"

    def test_lone_underscore_preserved(self):
        # Underscore inside a word is not emphasis.
        assert strip_markdown("под_черкивание") == "под_черкивание"


class TestStripMarkdownBlocks:
    def test_heading_removed(self):
        assert strip_markdown("# Заголовок") == "Заголовок"
        assert strip_markdown("### Третий уровень") == "Третий уровень"

    def test_blockquote_removed(self):
        assert strip_markdown("> Жил-был енотик") == "Жил-был енотик"

    def test_list_bullets_removed(self):
        assert strip_markdown("- пункт один\n- пункт два") == "пункт один\nпункт два"

    def test_numbered_list_removed(self):
        assert strip_markdown("1. первый\n2. второй") == "первый\nвторой"


class TestStripMarkdownInline:
    def test_inline_code_keeps_text(self):
        assert strip_markdown("Используй `speak_text` для ответа") == "Используй speak_text для ответа"

    def test_link_keeps_label(self):
        assert strip_markdown("[текст](https://example.com)") == "текст"

    def test_strikethrough_removed(self):
        assert strip_markdown("~~зачёркнуто~~") == "зачёркнуто"

    def test_non_string_passthrough(self):
        assert strip_markdown(None) is None  # type: ignore[arg-type]
        assert strip_markdown(42) == 42  # type: ignore[arg-type]

    def test_plain_text_unchanged(self):
        text = "Привет! Как дела?"
        assert strip_markdown(text) == text
