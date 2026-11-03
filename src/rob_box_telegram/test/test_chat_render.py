"""#4243: оформитель чата Telegram - голый текст -> сообщения с HTML-разметкой; чистая функция, без telegram/ROS."""

from __future__ import annotations

import re

import pytest

from rob_box_telegram import chat_render as cr

SPECIAL = 'Ток 5 < 7 & 8 > 3, тег <b>не жирный</b>, "кавычки", \\ слеш, _ * [ ] ( ) ~ ` # + - = | { } . !'


def _visible(html: str) -> str:
    """Что увидит человек: без тегов, с раскрытыми &lt; &gt; &amp;."""
    text = re.sub(r"</?(b|i|blockquote)[^>]*>", "", html)
    return text.replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")


def test_escape_covers_exactly_the_three_html_characters():
    assert cr.escape("a & b < c > d") == "a &amp; b &lt; c &gt; d"
    assert cr.escape("«ёлка» \"x\" 'y' \\ _ *") == "«ёлка» \"x\" 'y' \\ _ *"  # остальное API не требует
    assert cr.escape("&lt;") == "&amp;lt;"  # текст модели голый: уже «экранированное» - тоже просто текст


def test_short_narrator_reply_is_plain_text_with_digits_untouched():
    [msg] = cr.render("Заряд 87%, до дома 2,5 км.")
    assert msg.text == msg.plain == "Заряд 87%, до дома 2,5 км."
    assert msg.parse_mode == "HTML"


def test_special_characters_never_become_markup_in_any_role():
    for role in ("", "Ермил <script>"):
        for msg in cr.render(SPECIAL, role):
            assert "<b>не жирный</b>" not in msg.text
            assert SPECIAL in _visible(msg.text)
            assert msg.plain.endswith(SPECIAL)


def test_hero_gets_bold_name_and_the_line_in_a_quote():
    [msg] = cr.render("Ты делал бэкап?", "Ермил")
    assert msg.text == "<b>Ермил</b>\n<blockquote>Ты делал бэкап?</blockquote>"
    assert msg.plain == "Ермил\nТы делал бэкап?"


def test_long_hero_line_is_an_expandable_quote():
    [msg] = cr.render("слово " * 120, "Ермил")
    assert "<blockquote expandable>" in msg.text


def test_long_narrator_shows_first_paragraph_and_hides_the_rest_in_expandable_quote():
    text = "Вот анекдот.\n\n" + "Строка стиха.\n" * 10
    [msg] = cr.render(text)
    assert msg.text.startswith("Вот анекдот.\n<blockquote expandable>Строка стиха.")
    assert msg.text.endswith("</blockquote>")
    assert msg.plain == text.strip()


def test_long_single_paragraph_goes_wholly_into_expandable_quote():
    [msg] = cr.render("слово " * 120)
    assert msg.text.startswith("<blockquote expandable>") and msg.text.endswith("</blockquote>")


def test_few_short_lines_stay_plain_but_a_poem_of_many_lines_is_quoted():
    assert cr.render("а\nб\nв")[0].text == "а\nб\nв"
    assert cr.render("\n".join("стих" for _ in range(8)))[0].text.startswith("<blockquote expandable>")


def test_empty_text_renders_nothing():
    assert cr.render("") == [] and cr.render("  \n ") == []


@pytest.mark.parametrize("role", ["", "Ермил"])
def test_text_over_the_limit_is_cut_into_messages_each_within_the_limit(role):
    text = "\n\n".join(f"Абзац {n}. " + "слово " * 150 for n in range(12))
    parts = cr.render(text, role)
    assert len(parts) > 1
    assert all(len(p.text) <= cr.MESSAGE_LIMIT and len(p.plain) <= cr.MESSAGE_LIMIT for p in parts)
    joined = " ".join(p.plain.removeprefix(f"{role}\n") if role else p.plain for p in parts)
    assert re.sub(r"\s+", " ", joined) == re.sub(r"\s+", " ", text).strip()
    if role:
        assert parts[0].text.startswith("<b>Ермил</b>") and not any("<b>" in p.text for p in parts[1:])


def test_a_word_longer_than_the_limit_is_cut_hard_not_lost():
    parts = cr.render("я" * 10000)
    assert len(parts) >= 3 and "".join(p.plain for p in parts) == "я" * 10000


def test_markup_is_always_balanced_in_every_part():
    for msg in cr.render("\n\n".join("абзац <" + "слово & " * 100 for _ in range(10)), "Роль"):
        for tag in ("b", "blockquote"):
            assert len(re.findall(rf"<{tag}[ >]", msg.text)) == len(re.findall(rf"</{tag}>", msg.text))


def test_progress_line_is_italic_with_seconds():
    item = cr.progress("Думаю…", 7.9)
    assert item.text == "<i>⏳ Думаю… 7 с</i>" and item.plain == "⏳ Думаю… 7 с"


def test_progress_steps_past_ones_are_dots_current_one_has_the_counter_and_html_is_escaped():
    item = cr.progress(["Думаю…", "Ищу <в сети>…", "Ищу <в сети> - готово"], 12.2)
    assert item.plain == "• Думаю…\n• Ищу <в сети>…\n⏳ Ищу <в сети> - готово 12 с"
    assert item.text == "<i>• Думаю…\n• Ищу &lt;в сети&gt;…\n⏳ Ищу &lt;в сети&gt; - готово 12 с</i>"


def test_progress_shows_only_the_last_steps():
    item = cr.progress([f"шаг {n}" for n in range(20)], 1)
    assert item.plain.count("\n") + 1 == cr.PROGRESS_STEPS_SHOWN and item.plain.startswith("• шаг 14")
