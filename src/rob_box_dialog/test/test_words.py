"""Сверка текста со словами знания — по границам слов (ADR-0150 §4.2, §6.3; приёмка A15 — корпус вейков)."""

import pytest

from rob_box_dialog.knowledge import DEFAULT_WAKE_WORDS, SILENCE_WORDS, TIER1_POLICY, UNSILENCE_WORDS
from rob_box_dialog.words import has_wake_word, has_word, strip_wake_word

#: Фразы, обращённые к роботу (живые и e2e-искажения STT).
WAKES = [
    "робот, включи музыку",
    "Робот понял хватит останавливай",
    "денчик ой фу робот меня зовут саша",
    "Робота, про меня что помнишь?",
    "Робби, добрый вечер",
    "роб бокс расскажи анекдот",
    "rob box, hello",
    "рома покажи анимацию ускорения",
    "РОББОКС стоп",
]
#: Фон: слово-вейк внутри другого слова или в составном слове через дефис — не обращение.
NOT_WAKES = [
    "у меня всё работает",
    "системный промт для робота-диджея",
    "он заработал денег",
    "это ботинок",
    "роботы захватят мир",
    "робототехника — наука",
    "супер-робот на экране",
    "",
]


@pytest.mark.parametrize("text", WAKES)
def test_wake_corpus_wakes(text):
    assert has_wake_word(text)


@pytest.mark.parametrize("text", NOT_WAKES)
def test_false_wake_corpus_does_not_wake(text):
    assert not has_wake_word(text)


def test_empty_wake_list_is_bypass_mode():
    assert has_wake_word("что угодно", [])


def test_strip_removes_first_whole_word_wake_with_punctuation():
    assert strip_wake_word("Робот, включи музыку") == "включи музыку"
    assert strip_wake_word("роб бокс расскажи") == "расскажи"
    assert strip_wake_word("денчик ой фу робот меня зовут") == "денчик ой фу меня зовут"
    # «роботы» — не вейк «робот»: снимается настоящее слово дальше по фразе
    assert strip_wake_word("роботы, робот иди") == "роботы, иди"
    assert strip_wake_word("ничего тут нет") == "ничего тут нет"
    assert strip_wake_word("  текст ", []) == "текст"


@pytest.mark.parametrize("text", [
    "Робот, понял, хватит, останавливай",
    "помолчи",
    "замолчите все",
    "робот хватит",
    "ХВАТИТ",
])
def test_silence_words_match_on_word_start(text):
    assert has_word(text, SILENCE_WORDS)


@pytest.mark.parametrize("text", ["нехватка денег", "он захватит флаг", "молчаливый", "ничего"])
def test_silence_words_do_not_match_inside_words(text):
    assert not has_word(text, SILENCE_WORDS)


def test_unsilence_words():
    assert has_word("говори снова", UNSILENCE_WORDS)
    assert has_word("отвечай", UNSILENCE_WORDS)
    assert not has_word("заработает", UNSILENCE_WORDS)


def test_empty_stems_match_nothing():
    assert not has_word("хватит", ())


def test_tier1_stop_words():
    assert has_word("стоп", TIER1_POLICY["stop"].words)
    assert has_word("когда закончишь спой", TIER1_POLICY["later"].words)


def test_every_wake_word_is_found_alone_and_in_a_phrase():
    for word in DEFAULT_WAKE_WORDS:
        assert has_wake_word(word), word
        assert has_wake_word(f"{word}, расскажи анекдот"), word
