"""#4159 п.3 (ревью PR #4153): справка со словом из ``CHAT_ROBOT_ASK_FORMS`` - ответ в чат, а не «ничего не сделал».

Решение Шифу 28.10: Telegram - пульт робота, но справки (время, поиск, вопросы) - ответом в чат. ``robot_owed`` требует
повтора ТОЛЬКО когда реплика просит действие робота: слово из ``CHAT_ROBOT_ASK_FORMS`` («роботом», «вслух», «народ»,
«толпу»…), не названное темой, и не справка. Вопрос («что значит вслух?», «сколько народу было?») - справка с #4153
(``words.asks_question``); сюда добавлены справка-просьба («найди, что пишет народ»: ``CHAT_REFERENCE_LEADS``) и слово,
названное темой («расскажи про народ», «переведи слово вслух»: ``CHAT_TOPIC_MARKERS``).
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command
from rob_box_dialog.deadline import TurnDeadline
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import robot_asked
from rob_box_dialog.turn import _Turn, robot_owed
from rob_box_dialog.words import asks_reference, asks_robot_to_act, name_tokens

CLAIM = "Покекал немного: глазами поводил, повертелся."

#: Справки со словом из ``CHAT_ROBOT_ASK_FORMS``: вопросы (Шифу 28.10) и справки-просьбы, и слово как тема.
REFERENCES = (
    # вопросы - решение Шифу 28.10 и #4153
    "что значит вслух?", "сколько народу было?", "Как управлять роботом?", "что народ пишет про айфон",
    # справка-просьба: найти, узнать, перевести, научить
    "Найди, что пишет народ про айфон", "Поищи в интернете сколько народу было на митинге",
    "Найди статью про толпу", "Узнай, что люди пишут про народ", "Научи меня читать вслух",
    "Посоветуй, как читать вслух", "Робот, найди в интернете, что думает народ о погоде",
    # слово названо темой
    "Расскажи про народ", "Переведи слово вслух на английский", "Объясни слово роботом", "Расскажи о толпе",
    "Объясни выражение через робота", "Скажи, что значит слово озвучь",
)

#: Просьбы сделать на роботе: глагол действия и слово из таблицы.
ACTIONS = (
    "Покекай роботом", "Именно через робота чтобы хороший кек", "развлекай народ, говори о том, какой прекрасный день",
    "скажи всем вокруг привет", "Скажи вслух, который час", "агитируй толпу под Калинку-Малинку",
    "Можешь покекать роботом?", "А можно вслух прочитать стих?", "прочитай вслух",
    "Расскажи народу анекдот", "Спой песню роботом", "включи в колонки музыку",
    "Робот, посчитай вслух от одного до двадцати не торопясь",
    # справка-просьба, после которой робота просят сказать: поиск и действие
    "Найди анекдот и прочитай вслух", "Найди в интернете анекдот и расскажи его народу",
    "Поищи, что сегодня за праздник, и скажи вслух",
    # слово-тема и слово-адресат в одной реплике: действие остаётся
    "Расскажи про космос вслух", "Спой про народ вслух", "Расскажи народу про космос",
)


@pytest.mark.parametrize("text", REFERENCES)
def test_reference_is_not_a_robot_request(text):
    assert not robot_asked(text), text
    assert not asks_robot_to_act(text)


@pytest.mark.parametrize("text", ACTIONS)
def test_request_to_act_stays_a_robot_request(text):
    assert robot_asked(text), text


def test_robot_owed_follows_the_words_of_the_person_not_the_reply():
    chat = Session("telegram").snapshot
    speech = Command(speech=CLAIM)
    for text in REFERENCES:
        turn = _Turn("t1", 0, text, "telegram", TurnDeadline(30.0))
        assert not robot_owed(chat, turn, speech), text
    for text in ("покекай роботом", "Найди анекдот и прочитай вслух", "Расскажи народу анекдот"):
        turn = _Turn("t2", 0, text, "telegram", TurnDeadline(30.0))
        assert robot_owed(chat, turn, speech), text


def test_a_lookup_lead_without_a_robot_verb_is_a_reference_and_with_one_is_not():
    assert asks_reference("Найди, что пишет народ")
    assert not asks_reference("Найди анекдот и прочитай вслух")
    assert not asks_reference("покекай роботом")
    assert asks_reference("Робот, поищи, сколько народу было")


def test_topic_marker_unnames_only_the_form_right_after_it():
    assert not asks_robot_to_act("Расскажи про народ")
    assert asks_robot_to_act("Расскажи про космос народу")
    assert asks_robot_to_act("Расскажи про космос вслух")
    assert asks_robot_to_act("расскажи про народ вслух"), "«вслух» - канал: он остаётся даже когда «народ» - тема"


def test_reference_tables_are_whole_lowercase_words_and_do_not_overlap():
    for table in (kn.CHAT_REFERENCE_LEADS, kn.CHAT_ROBOT_ACT_VERBS, kn.CHAT_TOPIC_MARKERS):
        for word in table:
            assert word == word.lower() and "ё" not in word and " " not in word, word
            assert name_tokens(word) == (word,), word
        assert len(set(table)) == len(table)
    assert not set(kn.CHAT_REFERENCE_LEADS) & set(kn.CHAT_ROBOT_ACT_VERBS), "поиск - не глагол, поручающий действие"
    assert not set(kn.CHAT_REFERENCE_LEADS) & set(kn.ASK_LEAD_SKIP), "вводное слово не может быть и справочным зачином"
