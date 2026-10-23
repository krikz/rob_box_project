"""Сверка текста со словами знания — по границам слов (ADR-0150 §4.2, §6.3; приёмка A15 — корпус вейков)."""

import pytest

from rob_box_dialog.knowledge import (
    DEFAULT_WAKE_WORDS, SILENCE_WORDS, STOP_LEAD_WORDS, STOP_WORDS, TIER1_POLICY, UNSILENCE_WORDS,
)
from rob_box_dialog.words import (
    STOP_CLEAR, STOP_DISPUTED, STOP_NONE, has_form, has_wake_word, has_word, introduces_self, stop_verdict, strip_wake_word,
)

#: Фразы, обращённые к роботу (живые и e2e-искажения STT).
WAKES = [
    "робот, включи музыку",
    "Робот понял хватит останавливай",
    "денчик ой фу робот меня зовут саша",
    "Робота, про меня что помнишь?",
    "Робби, добрый вечер",
    "роб бокс расскажи анекдот",
    "rob box, hello",
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
    # В18: рискованные вейки выключены по умолчанию (включаются risky_enabled в wake_words.yaml)
    "бот в телеграме молчит",
    "",
]


@pytest.mark.parametrize("text", WAKES)
def test_wake_corpus_wakes(text):
    assert has_wake_word(text)


@pytest.mark.parametrize("text", ["рома покажи анимацию ускорения", "Роман, включи музыку", "роман расскажи анекдот"])
def test_roma_and_roman_wake(text):
    """23.10 Шифу: «рома»/«роман» — обычные вейки (DEFAULT_WAKE_WORDS), не корпус ложных вейков."""
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
    "Робот, понял, замолчи, останавливай",
    "помолчи",
    "замолчите все",
    "робот замолчи",
    "ПОМОЛЧИ",
])
def test_silence_words_match_on_word_start(text):
    assert has_word(text, SILENCE_WORDS)


@pytest.mark.parametrize("text", ["нехватка денег", "он захватит флаг", "молчаливый", "ничего"])
def test_silence_words_do_not_match_inside_words(text):
    assert not has_word(text, SILENCE_WORDS)


@pytest.mark.parametrize("text", ["хватит", "Робот, хватит", "ХВАТИТ", "стоп", "отставить"])
def test_enough_is_not_a_silence_word(text):
    """Решение Шифу 22.10 (#3700): «хватит» останавливает звуки, режим тишины не включает."""
    assert not has_word(text, SILENCE_WORDS)
    assert has_word(text, TIER1_POLICY["stop"].words)


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


# -- В19 (Шифу 20.10): стоп-слово как команда против стоп-слова внутри другой команды ----------------------------
#: Пары «рвёт речь / не рвёт»: слева ясная стоп-команда, справа то же стоп-слово внутри другой команды или без него.
STOP_PAIRS = [
    ("Робот, стоп", "Робот, покажи анимацию стоп"),
    ("Робот, хватит", "Робот, спой песню про хватит"),
    ("робот стой", "робот не стой на месте"),
    ("Робот, замолчи", "Робот, расскажи сказку про то как ёж сказал замолчи"),
    ("Робот, всё, хватит, спой другую", "Робот, спой другую и хватит"),
    ("робот ну хватит уже", "робот найди песню стоп"),
    ("Роббокс, стоп-стоп", "Роббокс, включи песню хватит"),
    ("хватит робот", "робот у меня стопка книг"),
    ("Робот, помолчи", "Робот, это стоит дорого"),
    ("Робби, отставить", "Робби, найди песню Stop"),
    ("робот да хватит", "робот хватит ли нам сил"),
    ("Робот, стоп. Включи музыку", "Робот, включи стоп-кран"),
    ("эй робот стоп", "робот захватит флаг"),
    ("Робот, замолчите", "Робот, покажи анимацию «стой»"),
    ("Робот, стойте", "Робот, как будет по-английски хватит"),
    ("РОББОКС СТОП", "робот работает"),
    # живой лог 21.10: хвост речи робота («…у корней старой сосны») склеился с фразой человека
    ("старой робот хватит", "спой песню про стоп робот"),
]


@pytest.mark.parametrize("clear, other", STOP_PAIRS)
def test_stop_corpus_clear_command_versus_stop_word_inside_another(clear, other):
    assert stop_verdict(clear) == STOP_CLEAR, clear
    assert stop_verdict(other) != STOP_CLEAR, other


@pytest.mark.parametrize("text", [
    "Робот, покажи анимацию стоп",
    "Робот, спой песню про хватит",
    "Робот, расскажи сказку про то как ёж сказал замолчи",
    "робот не стой на месте",
])
def test_stop_word_after_another_word_is_disputed(text):
    """Спорный класс — его решает Chooser (PR-6), речь до решения не рвётся."""
    assert stop_verdict(text) == STOP_DISPUTED


@pytest.mark.parametrize("text", ["Робот, найди песню Stop", "робот у меня стопка книг", "робот захватит флаг",
                                  "робот", "", "робот работает"])
def test_no_stop_word_is_none(text):
    assert stop_verdict(text) == STOP_NONE


def test_stop_verdict_uses_given_wake_words():
    assert stop_verdict("тарс стоп", ["тарс"]) == STOP_CLEAR
    assert stop_verdict("тарс стоп", ["робот"]) == STOP_DISPUTED


def test_tier1_stop_words_are_stop_words():
    for policy in ("stop", "halt"):
        for word in TIER1_POLICY[policy].words:
            assert word in STOP_WORDS, word
    for word in STOP_WORDS:
        assert word in TIER1_POLICY["stop"].words + TIER1_POLICY["halt"].words + ("стойте",) or has_word(
            word, SILENCE_WORDS), word
    assert not set(STOP_LEAD_WORDS) & set(STOP_WORDS)


@pytest.mark.parametrize("text", [
    "Робот, здравствуй, я Саша, чиню тут технику", "привет, я Борис", "робот это я, Валера", "меня зовут Денис",
    "привет я Аскар", "зови меня Шифу",
])
def test_introduces_self(text):
    assert introduces_self(text)


@pytest.mark.parametrize("text", [
    "я думаю, что это правда", "я пью зелёный чай", "я устал", "я не Саша", "привет, я дома", "привет, я голодный",
    "привет", "робот, включи музыку", "как меня зовут", "я Борису сказал", "я Аскар",
])
def test_does_not_introduce_self(text):
    """«я Аскар» без приветствия и вне словаря - честная граница: словарь даёт точность, а не полноту."""
    assert not introduces_self(text)


def test_has_form_is_a_whole_word():
    assert has_form("добавь бас", ("бас",)) and not has_form("расскажи басню", ("бас",))
