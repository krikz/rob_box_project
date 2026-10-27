"""Сверка текста со словами знания — по границам слов (ADR-0150 §4.2, §6.3; приёмка A15 — корпус вейков)."""

import pytest

from rob_box_dialog.knowledge import (
    DEFAULT_WAKE_WORDS, SILENCE_WORDS, STOP_LEAD_WORDS, STOP_WORDS, TIER1_POLICY, UNSILENCE_WORDS, WAKE_SLIP_ADDRESSEE,
    WAKE_SLIP_AMBIGUOUS, WAKE_SLIP_COMMAND_LEADS, WAKE_SLIP_GATES, WAKE_SLIP_MORE_LEADS, WAKE_SLIP_NEVER,
    WAKE_SLIP_PLURAL, WAKE_SLIP_QUESTION_CUED_LEADS, WAKE_SLIP_QUESTION_LEADS, WAKE_SLIP_VARIANTS,
)
from rob_box_dialog.words import (
    NO_SCOPE, STOP_CLEAR, STOP_DISPUTED, STOP_NONE, StopScope, has_form, has_wake_word, has_word, introduces_self,
    slipped_wake_rest, stop_command, stop_scope, stop_verdict, strip_wake_word, wake_command,
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
    # #4020 n709: «останови» — стоп; «остановка» в обычной речи — нет
    ("Робот, останови, Борис уже уснул под неё", "Робот, где остановка автобуса"),
    ("робот остановись", "робот на остановке холодно"),
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


# --- #3996: искажённый вейк «роботы/работа/рада» - ворота WAKE_SLIP_GATES (поведение slipped_wake_rest) ------------

@pytest.mark.parametrize("text,rest", [
    ("роботы еще куплет добавь про комара который еноту мешал", "еще куплет добавь про комара который еноту мешал"),
    ("РОБОТЫ ЕЩЁ КУПЛЕТ ДОБАВЬ", "ЕЩЁ КУПЛЕТ ДОБАВЬ"),
    ("работа как меня зовут", "как меня зовут"),
    ("Работа, как меня зовут?", "как меня зовут"),
    ("роботы что ты умеешь", "что ты умеешь"),
    ("рада какое у тебя имя", "какое у тебя имя"),
    ("работа сколько у тебя заряда", "сколько у тебя заряда"),
    ("роботы где ты", "где ты"),
    ("рада что сейчас звучит", "что сейчас звучит"),
    ("рада что сейчас крутится", "что сейчас крутится"),
    ("роботы дальше музыку потише", "дальше музыку потише"),
    ("работа тишина", "тишина"),
    ("роботу дали команду", "дали команду"),
    ("работа как а б в г д ты", "как а б в г д ты"),
])
def test_slipped_wake_rest_gate_opens(text, rest):
    assert slipped_wake_rest(text) == rest


@pytest.mark.parametrize("text", [
    "работ много накопилось", "работ на неделю", "работа дальше пойдёт", "роботе не хватает памяти",
    "роботы это сложно", "роботы такие умные", "роботы ещё не умеют летать", "работа как работа", "работа что надо",
    "роботы что-то делают", "работа какая-то странная", "рада тебя видеть", "роботы захватят мир",
    "роботы", "работа", "работа, ",
    "привет работа как меня зовут",
    "роботы как а б в г д е ты",
])
def test_slipped_wake_rest_gate_stays_closed(text):
    """Отрицательные из проверки #3975, союзное/сравнительное «как/что», «ещё» без музыки, не первый токен и знак
    дальше окна ``WAKE_SLIP_CUE_WINDOW`` (шестое слово после ведущего - ещё внутри, седьмое - уже нет)."""
    assert slipped_wake_rest(text) is None


@pytest.mark.parametrize("lead", WAKE_SLIP_COMMAND_LEADS + WAKE_SLIP_QUESTION_LEADS)
@pytest.mark.parametrize("slip", ["работа", "рады", "роботы"])
def test_slip_lead_without_condition_opens_by_itself(slip, lead):
    assert slipped_wake_rest(f"{slip} {lead} просто так") == f"{lead} просто так"


@pytest.mark.parametrize("lead", WAKE_SLIP_QUESTION_CUED_LEADS + WAKE_SLIP_MORE_LEADS)
@pytest.mark.parametrize("slip", ["работа", "рады", "роботы"])
def test_slip_lead_with_condition_needs_its_cue(slip, lead):
    """«как/что/…» и «ещё/дальше/…» одни ничего не решают: без знака обращения или темы ворота закрыты."""
    assert slipped_wake_rest(f"{slip} {lead} просто так") is None


@pytest.mark.parametrize("cue", WAKE_SLIP_ADDRESSEE)
def test_every_addressee_word_opens_the_question_gate(cue):
    assert slipped_wake_rest(f"работа как {cue} думаешь") == f"как {cue} думаешь"


def test_slipped_wake_rest_needs_a_robot_wake_word_in_the_table():
    assert slipped_wake_rest("роботы еще куплет добавь", ["тарс"]) is None
    assert slipped_wake_rest("роботы еще куплет добавь", ["робот"]) == "еще куплет добавь"


def test_slip_tables_do_not_contradict_each_other():
    leads = [lead for gate in WAKE_SLIP_GATES for lead in gate.leads]
    assert len(leads) == len(set(leads)), "ведущее слово принадлежит одним воротам"
    for word in (*leads, *WAKE_SLIP_ADDRESSEE, *WAKE_SLIP_AMBIGUOUS, *WAKE_SLIP_PLURAL, *WAKE_SLIP_NEVER):
        assert word == word.lower() and "ё" not in word and " " not in word, word
    gated = set(WAKE_SLIP_AMBIGUOUS) | set(WAKE_SLIP_PLURAL)
    assert not gated & set(WAKE_SLIP_NEVER), "слово либо за воротами, либо не будит никогда"
    assert not gated & set(WAKE_SLIP_VARIANTS), "слово либо за воротами, либо будит без условий"
    assert not set(WAKE_SLIP_AMBIGUOUS) & set(WAKE_SLIP_PLURAL)


# --- #3994: охват ясного стопа (STOP_SCOPE, words.stop_scope) ------------------------------------------------------

@pytest.mark.parametrize("tail, scope, rest", [
    # n811 (акт 8, 23.10): «Робот, хватит читать, а музыку оставь» — со знаками и без (STT запятых не ставит)
    ("читать, а музыку оставь", StopScope(speech=True, keep_music=True), ""),
    ("читать а музыку оставь", StopScope(speech=True, keep_music=True), ""),
    ("читать", StopScope(speech=True), ""),
    ("говорить", StopScope(speech=True), ""),
    ("рассказывать, пожалуйста", StopScope(speech=True), ""),
    ("петь", StopScope(speech=True), ""),
    ("зачитывать", StopScope(speech=True), ""),
    ("болтать и петь", StopScope(speech=True), ""),
    ("музыку оставь", StopScope(keep_music=True), ""),
    ("оставь музыку", StopScope(keep_music=True), ""),
    ("а музыка пусть играет", StopScope(keep_music=True), ""),
    ("читать, музыку не выключай", StopScope(speech=True, keep_music=True), ""),
    ("читать и бит не трогай", StopScope(speech=True, keep_music=True), ""),
    ("читать, а музыку не глуши", StopScope(speech=True, keep_music=True), ""),
    ("музыки", StopScope(music=True), ""),
    ("играть", StopScope(music=True), ""),
    ("читать и играть", StopScope(speech=True, music=True), ""),
    # перефразы n811 (ревью PR #4035): объект речи, «тебе», «не останавливай/не отключай»
    ("читать рэп, а музыку оставь", StopScope(speech=True, keep_music=True), ""),
    ("рэп читать", StopScope(speech=True), ""),
    ("читать стих, а музыку оставь", StopScope(speech=True, keep_music=True), ""),
    ("читать вслух", StopScope(speech=True), ""),
    ("сказки рассказывать", StopScope(speech=True), ""),
    ("тебе читать, а музыку оставь", StopScope(speech=True, keep_music=True), ""),
    ("читать, музыку не останавливай", StopScope(speech=True, keep_music=True), ""),
    ("читать, музыку не отключай", StopScope(speech=True, keep_music=True), ""),
    # «рэп» без глагола речи — не охват («хватит рэпа» — может быть трек плеера)
    ("рэпа", NO_SCOPE, "рэпа"),
    # оговорка после новой просьбы не теряется; остаток — просьба целиком
    ("спой другую, а музыку оставь", StopScope(keep_music=True), "спой другую, а музыку оставь"),
    ("спой другую, и не трогай кота", NO_SCOPE, "спой другую, и не трогай кота"),
    # не покрыто — остаток (новая просьба), как было
    ("спой другую", NO_SCOPE, "спой другую"),
    ("сыграй другую", NO_SCOPE, "сыграй другую"),  # повелительное «сыграй» — заказ, не объект стопа
    ("играй дальше", NO_SCOPE, "играй дальше"),
    ("говори про котов", NO_SCOPE, "говори про котов"),
    ("читай другое", NO_SCOPE, "читай другое"),
    ("и включи свет", NO_SCOPE, "и включи свет"),
    ("Борис уже уснул под неё", NO_SCOPE, "Борис уже уснул под неё"),
    # покрытая часть съедается, остаток начинается с первой непокрытой части (союз перед ней отброшен)
    ("читать, а музыку сделай потише", StopScope(speech=True), "музыку сделай потише"),
    ("рассказывать и спой песню", StopScope(speech=True), "спой песню"),
    ("читать а музыку оставь и расскажи анекдот", StopScope(speech=True, keep_music=True), "расскажи анекдот"),
    ("", NO_SCOPE, ""),
])
def test_stop_scope_closed_grammar(tail, scope, rest):
    assert stop_scope(tail) == (scope, rest)


def test_stop_command_n811_scope_and_no_rest_for_the_llm():
    """n811: ясный стоп, охват — речь и «музыку оставь», остатка нет — новой просьбы в LLM нет."""
    call = stop_command("робот хватит читать а музыку оставь")
    assert call.verdict == STOP_CLEAR and call.word == "хватит"
    assert call.scope == StopScope(speech=True, keep_music=True) and call.rest == ""


def test_stop_command_keeps_the_rest_of_a_new_request():
    call = stop_command("Робот, хватит, спой другую")
    assert (call.verdict, call.scope, call.rest) == (STOP_CLEAR, NO_SCOPE, "спой другую")


@pytest.mark.parametrize("text", ["Робот, останови", "робот остановись", "Робот, остановите", "робот остановить",
                                  "Робот, останови, Борис уже уснул под неё"])
def test_4020_stop_forms_are_clear_stops(text):
    assert stop_verdict(text) == STOP_CLEAR


@pytest.mark.parametrize("text", ["Робот, где остановка автобуса", "робот останова двигателя", "робот без остановки",
                                  "робот остановкой больше", "остановка автобуса", "робот остановился сам"])
def test_4020_stop_lookalikes_are_not_stops(text):
    assert stop_verdict(text) == STOP_NONE


@pytest.mark.parametrize("text, command", [
    # n811 27.10: эхо рэпа робота склеилось с фразой человека перед вейком — команда только после вейка
    ("четыре недели робот хватит читать а музыку оставь", "хватит читать а музыку оставь"),
    ("старой робот хватит", "хватит"),
    ("Робот, хватит читать, а музыку оставь.", "хватит читать, а музыку оставь"),
    ("хватит читать, робот!", "хватит читать"),  # вейк в конце — команда перед ним
    ("хватит читать", "хватит читать"),  # без вейка — вся фраза
    ("работает робот-пылесос", "работает робот-пылесос"),  # не вейк
])
def test_4085_wake_command_is_what_follows_the_wake(text, command):
    assert wake_command(text) == command


def test_4085_wake_command_follows_the_same_rule_as_stop_command():
    """Одно правило «команда — после вейка» у Tier-1 реплики и у разбора стопа: что ясный стоп с эхом перед вейком,
    то ясный стоп и по команде без вейка."""
    text = "четыре недели робот хватит читать а музыку оставь"
    assert stop_verdict(text) == STOP_CLEAR
    assert stop_verdict(wake_command(text)) == STOP_CLEAR
    assert stop_verdict(strip_wake_word(text)) == STOP_DISPUTED  # так было: вейк снят, эхо осталось — спорный стоп
