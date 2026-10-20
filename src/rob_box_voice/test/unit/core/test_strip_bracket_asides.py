"""Скобочные вставки ``[...]`` не озвучиваются, где бы они ни стояли (PR #3474 разбор)."""
from rob_box_voice.core.speak_helpers import (
    EMPTY_REPLY_PHRASE, strip_bracket_asides, strip_meta_markers,
)


def test_aside_in_middle_removed():
    assert strip_meta_markers("Играю Баха [§2 ANTI-DUP: не повторять] сейчас") == "Играю Баха сейчас"


def test_aside_at_end_removed():
    assert strip_meta_markers("Привет! [думаю, что ответить]") == "Привет!"


def test_multiline_reasoning_only_leaves_empty():
    assert strip_meta_markers("[долгое\nрассуждение §2]") == ""


def test_prefix_still_stripped():
    assert strip_meta_markers("[Мнение ассистента] Привет!") == "Привет!"


def test_unclosed_tail_dropped():
    assert strip_meta_markers("Ответ. [рассуждение оборвалось") == "Ответ."


def test_nested_brackets():
    assert strip_bracket_asides("a [b [c] d] e") == "a e"


def test_service_markers_kept_for_guard():
    assert strip_bracket_asides("[CRITICAL] do x") == "[CRITICAL] do x"
    assert strip_bracket_asides("текст [SYSTEM note]") == "текст [SYSTEM note]"


def test_service_markers_removed_by_last_line_of_defence():
    assert strip_bracket_asides("[CRITICAL] привет", keep_service=False) == "привет"


def test_markdown_link_survives():
    assert strip_bracket_asides("см. [сайт](http://x)") == "см. [сайт](http://x)"


def test_plain_text_untouched():
    assert strip_meta_markers("Просто ответ.") == "Просто ответ."


def test_empty_reply_phrase_does_not_claim_acceptance():
    assert "принял" not in EMPTY_REPLY_PHRASE.lower()
