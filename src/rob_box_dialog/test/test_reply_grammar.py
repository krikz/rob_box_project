"""Грамматика ответа на вопрос робота (#3695): что похоже на ответ, а что — чужой разговор."""

import pytest

from rob_box_dialog.reply_grammar import NAME, NO, REFUSAL, YES, Reply, classify_reply, classify_same


@pytest.mark.parametrize("text, expected", [
    ("меня зовут Саша", Reply(NAME, "Саша")),
    ("Меня зовут Кракозябра.", Reply(NAME, "Кракозябра")),
    ("зови меня Гриша", Reply(NAME, "Гриша")),
    ("я Гриша", Reply(NAME, "Гриша")),
    ("Гриша", Reply(NAME, "Гриша")),
    ("Гриша Иванов", Reply(NAME, "Гриша")),
    ("робот меня зовут Борис", Reply(NAME, "Борис")),
    ("не скажу", Reply(REFUSAL)), ("не хочу говорить", Reply(REFUSAL)), ("секрет", Reply(REFUSAL)),
    ("потом", Reply(REFUSAL)),
    ("да", Reply(YES)), ("это я", Reply(YES)), ("Да, это я!", Reply(YES)),
    ("нет", Reply(NO)), ("не я", Reply(NO)), ("это не я", Reply(NO)),
])
def test_looks_like_an_answer(text, expected):
    assert classify_reply(text) == expected


@pytest.mark.parametrize("text", [
    "поселения происходили давайте я устрою наше время", "только что основали", "Гриша пришёл вчера", "я не Гриша",
    "расскажи анекдот", "потом я расскажу тебе длинную историю про море", "", "привет", "включи музыку",
])
def test_not_an_answer(text):
    assert classify_reply(text) is None


def test_bare_name_is_not_an_answer_to_yes_no_question():
    assert classify_reply("Гриша", asks_name=False) is None
    assert classify_reply("да", asks_name=False) == Reply(YES)


# ── #2828/#3765: ответ на вопрос склейки после регистрации ───────────────────────────────────────────────────────────
@pytest.mark.parametrize("text, expected", [
    ("мы разные люди", False),
    ("нет, я другой человек", False),
    ("ты обознался", False),
    ("это не я", False),
    ("да, это я под другим именем", True),
    ("я тот самый Дэнчик", True),
    ("Робот, один человек", True),
    ("не тот самый", None),          # оба сигнала — не понять
    ("да", None),                    # голое да/нет — не про «один или двое»
    ("это японец", None),            # «это я» — по границам слов
    ("это я вчера включал тебе эту музыку", None),  # длинная фраза — не ответ
    ("пицца будет в пятницу", None),
])
def test_classify_same(text, expected):
    assert classify_same(text) is expected


@pytest.mark.parametrize("text, expected", [
    ("это я", True), ("да это я", True), ("да, я", True), ("Робот, да, это я", True), ("я", True),
    ("да я не знаю", None),  # «да я» внутри другой фразы — не ответ
])
def test_classify_same_it_is_me_corpus(text, expected):
    """Живой робот 21.10 (координатор): на «Вы разные люди или это ты?» ответы «это я / да это я / да, я» — один
    человек."""
    assert classify_same(text) is expected
