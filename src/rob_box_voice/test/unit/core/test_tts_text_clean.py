"""test_tts_text_clean.py — очистка текста и отбраковка служебного ввода перед синтезом (``rob_box_voice.tts_text_clean``).

* ``strip_markdown`` (issue #988): LLM оборачивает стихи и рэп в Markdown (``*Жил да был енот весёлый,*``), а TTS читает
  ``*`` как «звёздочка» — тесты закрепляют правила, чтобы робот пел слова, а не знаки препинания;
* ``strip_bracket_asides`` (PR #3474): вставки ``[...]`` не озвучиваются, где бы они ни стояли — в том числе реплика
  окна ``[ответ действием: …]`` (``knowledge.WINDOW_ACTIONS_REPLY``), которую код ставит в историю хода;
* ``is_system_template_regurgitated_in_ssml`` (issue #2175): ответ целиком из ``<system>…</system>`` — не речь.

PR-15 часть 15-core (#3711): модуль заменил ``core/speak_helpers.py`` и ``core/dialogue_guards.py``; чистка ответа старой
ноды диалога (метки ``[Spkr:…]``, маркер ``done``, ``<think>``) ушла вместе с ней.

Pure Python — no ROS, no rclpy, no heavy deps.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_tts_text_clean.py
"""

from __future__ import annotations

from typing import Optional

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_voice.tts_text_clean import (
    is_system_template_regurgitated_in_ssml,
    strip_bracket_asides,
    strip_markdown,
)


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


class TestStripBracketAsides:
    def test_aside_in_middle_removed(self):
        assert strip_bracket_asides("Играю Баха [§2 ANTI-DUP: не повторять] сейчас") == "Играю Баха сейчас"

    def test_aside_at_end_removed(self):
        assert strip_bracket_asides("Привет! [думаю, что ответить]") == "Привет!"

    def test_multiline_reasoning_only_leaves_empty(self):
        assert strip_bracket_asides("[долгое\nрассуждение §2]") == ""

    def test_leading_aside_removed(self):
        assert strip_bracket_asides("[Мнение ассистента] Привет!") == "Привет!"

    def test_unclosed_tail_dropped(self):
        assert strip_bracket_asides("Ответ. [рассуждение оборвалось") == "Ответ."

    def test_nested_brackets(self):
        assert strip_bracket_asides("a [b [c] d] e") == "a e"

    def test_service_markers_are_removed_too(self):
        # tts_node — последний рубеж: служебные вставки тоже не звучат (раньше их оставлял гард старой ноды).
        assert strip_bracket_asides("[CRITICAL] привет") == "привет"
        assert strip_bracket_asides("текст [SYSTEM note]") == "текст"

    def test_markdown_link_survives_for_strip_markdown(self):
        assert strip_bracket_asides("см. [сайт](http://x)") == "см. [сайт](http://x)"
        assert strip_markdown(strip_bracket_asides("см. [сайт](http://x)")) == "см. сайт"

    def test_plain_text_and_non_string_untouched(self):
        assert strip_bracket_asides("Просто ответ.") == "Просто ответ."
        assert strip_bracket_asides(None) is None  # type: ignore[arg-type]

    def test_window_reply_of_the_action_only_turn_is_not_voiced(self):
        """Реплика окна «ответ действием» (``knowledge.WINDOW_ACTIONS_REPLY``) — служебная вставка в скобках: если
        модель скопирует её в речь, TTS её не озвучит."""
        reply = kn.WINDOW_ACTIONS_REPLY.format(actions="play_animation(happy)")
        assert reply.startswith("[") and reply.endswith("]")
        assert strip_bracket_asides(reply) == ""


class TestSystemTemplateRegurgitateInSsml:
    """Defence-in-depth для tts_node: детектор на СЫРОМ SSML (до снятия тегов в ``_extract_text_from_ssml``).

    Узкий: только полный ``<speak>``-обрамлённый блок ИЛИ plain ``<system>...</system>``.
    """

    @pytest.mark.parametrize(
        "ssml",
        [
            # SSML-обёртка с regurgitates внутри
            "<speak><system>foo</system></speak>",
            "<speak><system>\n[получатель ответа забыл указать антропоморфные "
            "атрибуты]\n</system></speak>",
            # Без SSML-обёртки (edge-case)
            "<system>x</system>",
            # Whitespace внутри тегов (после ``<speak>`` / перед ``</speak>``)
            "<speak>\n  <system>foo</system>  \n</speak>",
        ],
    )
    def test_ssml_regurgitate_is_detected(self, ssml: str) -> None:
        assert is_system_template_regurgitated_in_ssml(ssml) is True

    @pytest.mark.parametrize(
        "ssml",
        [
            # Нормальная фраза с серединным тегом — НЕ regurgitates
            "<speak>Согласно <system>инструкции</system>, отвечу.</speak>",
            # Обычная русская речь
            "<speak>Привет! Как дела?</speak>",
            # Пустая строка / None
            "",
            None,  # type: ignore[list-item]
            # Неполные теги
            "<speak><system>foo</speak>",
            "<speak>foo</system></speak>",
        ],
    )
    def test_normal_ssml_passes(self, ssml: Optional[str]) -> None:
        """Серединные ссылки, нормальная речь, неполные теги."""
        assert is_system_template_regurgitated_in_ssml(ssml) is False
