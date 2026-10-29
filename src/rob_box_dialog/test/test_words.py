"""Сверка текста со словами знания — по границам слов (ADR-0150 §4.2, §6.3; приёмка A15 — корпус вейков)."""

import pytest

from rob_box_dialog.knowledge import (
    DEFAULT_WAKE_WORDS, MUSIC_AMBIGUOUS_STEMS, MUSIC_AMBIGUOUS_WORDS, MUSIC_OFF_WORDS, REACH_NEXT, SILENCE_WORDS,
    STOP_LEAD_WORDS, STOP_WORDS, TIER1_POLICY, UNSILENCE_WORDS, VOLUME_CHANNEL_STEMS, VOLUME_LEVEL_WORDS,
    VOLUME_DEGREE_WORDS, VOLUME_NOUN_STEMS, VOLUME_STEMS, WAKE_SLIP_ADDRESSEE, WAKE_SLIP_AMBIGUOUS, WAKE_SLIP_COMMAND_LEADS,
    WAKE_SLIP_CUE_WINDOW, WAKE_SLIP_GATES, WAKE_SLIP_MORE_LEADS, WAKE_SLIP_NEVER, WAKE_SLIP_PLURAL, WAKE_SLIP_POLITE,
    WAKE_SLIP_QUESTION_CUED_LEADS, WAKE_SLIP_QUESTION_LEADS, WAKE_SLIP_STOP_FORMS, WAKE_SLIP_STOP_WINDOW,
    WAKE_SLIP_TWEAK_COMPARATIVE_ENDINGS, WAKE_SLIP_TWEAK_CUE_STEMS, WAKE_SLIP_TWEAK_ENDINGS, WAKE_SLIP_TWEAK_FORMS,
    WAKE_SLIP_TWEAK_IMPERATIVE_ENDINGS, WAKE_SLIP_TWEAK_NOUN_STEMS, WAKE_SLIP_TWEAK_REQUEST_STEMS,
    WAKE_SLIP_TWEAK_STEMS, WAKE_SLIP_VARIANTS,
)
from rob_box_dialog.words import (
    NO_SCOPE, STOP_CLEAR, STOP_DISPUTED, STOP_NONE, StopScope, asked_level, has_form, has_wake_word, has_word,
    introduces_self, level_phrases, level_word, slipped_wake_rest, stop_command, stop_scope, stop_verdict, strip_wake_word,
    wake_command,
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
    for under_music in (False, True):
        mine = [lead for gate in WAKE_SLIP_GATES if gate.under_music is under_music for lead in gate.leads]
        assert len(mine) == len(set(mine)), "ведущее слово принадлежит одним воротам (при том же under_music)"
    for word in (*leads, *WAKE_SLIP_ADDRESSEE, *WAKE_SLIP_AMBIGUOUS, *WAKE_SLIP_PLURAL, *WAKE_SLIP_NEVER):
        assert word == word.lower() and "ё" not in word and " " not in word, word
    gated = set(WAKE_SLIP_AMBIGUOUS) | set(WAKE_SLIP_PLURAL)
    assert not gated & set(WAKE_SLIP_NEVER), "слово либо за воротами, либо не будит никогда"
    assert not gated & set(WAKE_SLIP_VARIANTS), "слово либо за воротами, либо будит без условий"
    assert not set(WAKE_SLIP_AMBIGUOUS) & set(WAKE_SLIP_PLURAL)


# --- #3975: правка играющей музыки после искажённого вейка («роботы ускорь темп») - ворота ``under_music`` ---------

#: Фраза и остаток после вейка: под играющей музыкой будят. Первые три - живые фразы 27.10 (14:49 «роботы ускорь темп»
#: под диджей-сетом; 09:26 n506 «роботы ускорь там потом мы тут засыпаем оба»), остальные - те же основы таблиц.
TWEAK_UNDER_MUSIC = [
    ("роботы ускорь темп", "ускорь темп"),                                                    # лог: 27.10 14:49
    ("роботы ускорь там потом мы тут засыпаем оба", "ускорь там потом мы тут засыпаем оба"),  # лог: n506, 27.10 09:26
    ("Роботы, ускорь темп!", "ускорь темп"),
    ("работа замедли", "замедли"),
    ("рада замедлите немного", "замедлите немного"),
    ("роботы громче", "громче"),
    ("работа погромче пожалуйста", "погромче пожалуйста"),
    ("рада тише", "тише"),
    ("роботы потише", "потише"),
    ("роботы убавь", "убавь"),
    ("работа прибавь громкости", "прибавь громкости"),
    ("роботы быстрее", "быстрее"),
    ("роботы помедленнее", "помедленнее"),
    ("роботы жёстче", "жёстче"),
    ("рады мягче", "мягче"),
    ("роботы темп побыстрее", "темп побыстрее"),
    ("роботы бас потише", "бас потише"),
    ("рады басы погромче", "басы погромче"),
    ("роботы темп помедленнее", "темп помедленнее"),
    ("роботы громче пожалуйста", "громче пожалуйста"),
    # ревью PR #4123: естественное «ещё быстрее/громче» и «сделай тише»
    ("роботы ещё быстрее", "ещё быстрее"),
    ("роботы ещё громче", "ещё громче"),
    ("рада еще тише", "еще тише"),
    ("роботы ещё чуть быстрее", "ещё чуть быстрее"),
    ("роботы сделай тише", "сделай тише"),
    ("работа сделайте погромче", "сделайте погромче"),
    ("рада сделай музыку потише", "сделай музыку потише"),
    ("роботы сделай бит потяжелее", "сделай бит потяжелее"),
    ("роботы добавь бас", "добавь бас"),
    ("работа добавь бит", "добавь бит"),
    ("рада добавьте куплет про комара", "добавьте куплет про комара"),
]
#: Те же слова и обычная речь с теми же основами: даже под музыкой не будят (не просьба или не про музыку).
TWEAK_NOT_WAKE_UNDER_MUSIC = [
    "роботы захватят мир", "роботы ускоряются", "роботы быстро учатся", "роботы ускоряют производство",
    "работа темпы растут", "рада тебя видеть", "роботы это сложно", "работа добавь соли", "рада добавь воды",
    "роботы скоро будут", "роботы бассейн", "работа мягкий плед", "роботы громкие",
    # ревью PR #4123: слово правки в начале рассказа - не просьба (за ним не конец фразы и не знак «про музыку»)
    "роботы темп задают", "рады скорости не боятся", "работа громче всех", "роботы быстрее нас считают",
    "роботы скорость набирают", "работа тише воды", "роботы тише воды ниже травы",
    # «бас» без слова правки и голые существительные не будят
    "роботы бас", "роботы басы", "роботы бас играет", "роботы басы играют", "роботы темп", "рады скорость",
    "рады скорость растёт",
    # «ещё» и «сделай» без слова правки или музыки рядом
    "роботы еще быстро учатся", "роботы ещё не умеют быстрее", "роботы еще", "роботы сделай чай",
    "работа сделайте шаг назад", "роботы сделай это сам",
]


@pytest.mark.parametrize("text,rest", TWEAK_UNDER_MUSIC)
def test_tweak_gate_wakes_under_music_and_only_under_music(text, rest):
    """Поведение ворот: под играющей музыкой правка будит, без музыки слово остаётся за закрытыми воротами (прежнее
    поведение: не вейк, фраза уходит в бэклог)."""
    assert slipped_wake_rest(text, music_playing=True) == rest
    assert slipped_wake_rest(text) is None
    assert slipped_wake_rest(text, music_playing=False) is None


@pytest.mark.parametrize("text", TWEAK_NOT_WAKE_UNDER_MUSIC)
def test_tweak_gate_does_not_wake_ordinary_speech_under_music(text):
    assert slipped_wake_rest(text, music_playing=True) is None, text


@pytest.mark.parametrize("text", ["роботы ускорь темп", "работа замедли", "рада громче", "роботы добавь бит"])
def test_tweak_gate_needs_a_robot_wake_word_in_the_table(text):
    assert slipped_wake_rest(text, ["тарс"], music_playing=True) is None


def test_music_does_not_change_the_unconditional_gates():
    """Музыка открывает только свои ворота: остальные решают одинаково с ней и без неё."""
    for text in ("роботы еще куплет добавь", "работа как меня зовут", "работа тишина", "роботы захватят мир",
                 "работ много накопилось", "роботы ещё не умеют летать", "рада что пришли"):
        assert slipped_wake_rest(text, music_playing=True) == slipped_wake_rest(text), text


def test_tweak_gate_stems_come_from_the_existing_tables_not_a_copy():
    """Одна таблица знания (ADR-0148): основы ведущих слов правки - объединение основ громкости и темпа, своих слов нет.
    Основы сверяются после ё → е («жёстч» из таблицы темпа стоит здесь как «жестч»: с «ё» она не совпала бы ни с одним
    словом фразы)."""
    known = {s.replace("ё", "е") for s in (*VOLUME_STEMS["louder"], *VOLUME_STEMS["quieter"], *MUSIC_AMBIGUOUS_STEMS)}
    assert set(WAKE_SLIP_TWEAK_STEMS) == known and len(WAKE_SLIP_TWEAK_STEMS) == len(known)
    assert "жестч" in WAKE_SLIP_TWEAK_STEMS and "жёстч" not in WAKE_SLIP_TWEAK_STEMS
    assert set(WAKE_SLIP_TWEAK_NOUN_STEMS) <= set(MUSIC_AMBIGUOUS_STEMS), "существительные - из таблицы темпа"
    assert set(WAKE_SLIP_TWEAK_REQUEST_STEMS) | set(WAKE_SLIP_TWEAK_NOUN_STEMS) == set(WAKE_SLIP_TWEAK_STEMS)
    assert not set(WAKE_SLIP_TWEAK_REQUEST_STEMS) & set(WAKE_SLIP_TWEAK_NOUN_STEMS)
    assert not any("ё" in form for form in WAKE_SLIP_TWEAK_FORMS), "формы сверяются после ё → е"
    assert {stem + ending for stem in WAKE_SLIP_TWEAK_REQUEST_STEMS for ending in WAKE_SLIP_TWEAK_ENDINGS} == set(
        WAKE_SLIP_TWEAK_FORMS), "слова просьбы - основы из таблиц × окончания, своих слов нет"
    tweak_gates = [gate for gate in WAKE_SLIP_GATES if gate.under_music]
    assert tweak_gates, "ворота правки музыки есть"
    by_ending = {gate.lead_endings for gate in tweak_gates if gate.lead_stems == WAKE_SLIP_TWEAK_REQUEST_STEMS}
    assert by_ending == {WAKE_SLIP_TWEAK_IMPERATIVE_ENDINGS, WAKE_SLIP_TWEAK_COMPARATIVE_ENDINGS}
    assert any(gate.lead_stems == WAKE_SLIP_TWEAK_NOUN_STEMS and set(MUSIC_AMBIGUOUS_WORDS) <= set(gate.leads)
               for gate in tweak_gates), "существительные (темп, бас) - свои ворота"
    assert not any(gate.under_music for gate in WAKE_SLIP_GATES[:4]), "прежние ворота музыки не требуют"


def test_tweak_endings_table_is_lowercase_and_unique():
    assert len(WAKE_SLIP_TWEAK_ENDINGS) == len(set(WAKE_SLIP_TWEAK_ENDINGS))
    assert set(WAKE_SLIP_TWEAK_ENDINGS) == set(WAKE_SLIP_TWEAK_IMPERATIVE_ENDINGS) | set(
        WAKE_SLIP_TWEAK_COMPARATIVE_ENDINGS)
    assert not set(WAKE_SLIP_TWEAK_IMPERATIVE_ENDINGS) & set(WAKE_SLIP_TWEAK_COMPARATIVE_ENDINGS)
    for part in (*WAKE_SLIP_TWEAK_ENDINGS, *WAKE_SLIP_TWEAK_STEMS, *WAKE_SLIP_POLITE):
        assert part == part.lower() and " " not in part, part


#: Ревью PR #4123: слово правки сравнительной степенью (громче, тише, быстрее, жёстче) - просьба, если фраза на нём
#: кончилась или сразу за ним слово «про музыку»; любое другое продолжение - уже рассказ.
COMPARATIVES = ("громче", "погромче", "тише", "потише", "быстрее", "быстрей", "побыстрее", "помедленнее", "жёстче",
                "мягче")


@pytest.mark.parametrize("word", COMPARATIVES)
def test_comparative_wakes_at_the_end_of_the_phrase_or_before_a_music_word_only(word):
    for tail, wakes in (("", True), (" пожалуйста", True), (" музыку", True), (" басы", True), (" быстрее", True),
                        (" всех", False), (" нас считают", False), (" воды", False), (" чем вчера", False)):
        text = f"роботы {word}{tail}"
        got = slipped_wake_rest(text, music_playing=True)
        assert (got is not None) is wakes, text
        assert slipped_wake_rest(text) is None, f"{text}: без музыки ворота закрыты"


@pytest.mark.parametrize("word", ("ускорь", "ускорьте", "замедли", "замедлите", "убавь", "прибавь", "подкрути", "приглуши"))
def test_imperative_wakes_whatever_follows(word):
    """Повелительное («ускорь», «убавь») - просьба по форме: живая n506 27.10 («ускорь там потом мы тут засыпаем оба»)
    будит и тогда, когда STT исказил слово после него."""
    for tail in ("", " темп", " там потом мы тут засыпаем оба", " всех"):
        assert slipped_wake_rest(f"роботы {word}{tail}", music_playing=True) == f"{word}{tail}"
        assert slipped_wake_rest(f"роботы {word}{tail}") is None


@pytest.mark.parametrize("noun", ("темп", "скорость", "скорости", "бас", "басы", "баса"))
def test_noun_leads_only_before_a_tweak_word(noun):
    """«темп/скорость/бас» называют, что правят: будят только перед словом правки («темп побыстрее», «бас потише»)."""
    assert slipped_wake_rest(f"роботы {noun}", music_playing=True) is None
    assert slipped_wake_rest(f"роботы {noun} задают", music_playing=True) is None
    assert slipped_wake_rest(f"роботы {noun} играет", music_playing=True) is None
    if noun in ("темп", "бас", "басы", "баса"):
        assert slipped_wake_rest(f"роботы {noun} потише", music_playing=True) == f"{noun} потише"
        assert slipped_wake_rest(f"роботы {noun} чуть быстрее", music_playing=True) == f"{noun} чуть быстрее"


def test_the_politeness_words_do_not_make_a_request_of_a_story():
    assert slipped_wake_rest("роботы чуть быстрее", music_playing=True) is None, "«чуть» - не ведущее слово"
    assert slipped_wake_rest("роботы ещё чуть", music_playing=True) is None
    assert slipped_wake_rest("роботы ещё чуть быстрее", music_playing=True) == "ещё чуть быстрее"


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


# --- #4136 п.2: громкость в заказе музыки словами (n809 «только не громко») --------------------------------------

@pytest.mark.parametrize("text, level", [
    ("поставь фоном ровный ритм на синтезаторе только не громко", "quieter"),
    ("Робот, поставь ровный ритм, только негромко.", "quieter"),
    ("поставь потише", "quieter"), ("убавь", "quieter"), ("поставь тихо", "quieter"),
    ("поставь громко", "louder"), ("поставь погромче", "louder"), ("только не тихо", "louder"),
    ("не тише", "louder"), ("не громче", "quieter"),
    ("поставь ритм", ""), ("громкость поменьше", ""), ("поставь «Тихий Дон»", ""),
    ("потише, нет, погромче", ""),
])
def test_4136_asked_level_reads_direction_and_negation(text, level):
    assert asked_level(text) == level


def test_4136_level_words_table_is_lowercase_whole_adverbs_not_stems():
    """Целые наречия, без ё и без основ: по основе «громк» открылась бы «громкость», по «тих» - «Тихий Дон»."""
    for intent, words in VOLUME_LEVEL_WORDS.items():
        assert intent in VOLUME_STEMS
        for word in words:
            assert word == word.lower() and "ё" not in word and " " not in word, word
            assert level_word(word) == intent
    assert not set(VOLUME_LEVEL_WORDS["louder"]) & set(VOLUME_LEVEL_WORDS["quieter"])
    assert level_word("громкость") == "" and level_word("тихий") == "" and level_word("") == ""


#: #4159 п.2 (ревью PR #4150): «не» + степень + слово громкости - то же обратное направление, что «не» вплотную. Было:
#: «не слишком громко» / «не очень громко» → louder (противоположное просьбе), «не слишком тихо» → quieter,
#: «потише, не очень громко» → просьба терялась (оба направления).
@pytest.mark.parametrize("text, level", [
    ("включи сет про космос не слишком громко", "quieter"),
    ("поставь музыку не очень громко", "quieter"),
    ("поставь ритм не так уж громко", "quieter"),
    ("поставь ритм не сильно громко", "quieter"),
    ("поставь ритм не особо громко", "quieter"),
    ("поставь ритм не особенно громко", "quieter"),
    ("поставь ритм не очень-то громко", "quieter"),
    ("не слишком тихо", "louder"),
    ("поставь ритм не очень тихо", "louder"),
    ("поставь ритм не так уж тихо", "louder"),
    ("потише, не очень громко", "quieter"),
    ("не очень громко и потише", "quieter"),
    ("погромче, не слишком тихо", "louder"),
    ("не слишком громко, но и не тихо", ""),  # «не громко» и «не тихо» вместе: середина - выбирать нечем
    # без «не» степень направления не меняет: «очень тихо» - тихо, «так громко» - громко
    ("поставь ритм очень тихо", "quieter"), ("поставь ритм так громко", "louder"),
    # «не» не около слова громкости (другая часть фразы) направление не переворачивает
    ("не включай танго, поставь громко", "louder"),
    ("поставь ритм очень", ""), ("не слишком, не очень", ""),
])
def test_4159_degree_between_not_and_level_word_still_negates(text, level):
    assert asked_level(text) == level


@pytest.mark.parametrize("words, phrases", [
    (["сет", "про", "космос", "не", "слишком", "громко"], [(3, 5, "quieter")]),
    (["не", "так", "уж", "громко"], [(0, 3, "quieter")]),
    (["потише", "не", "очень", "громко"], [(0, 0, "quieter"), (1, 3, "quieter")]),
    (["очень", "ё"], []),
    (["Не", "СЛИШКОМ", "Тихо"], [(0, 2, "louder")]),
    (["очень", "тихо"], [(0, 1, "quieter")]),
], ids=["theme-and-level", "so-already", "both-say-quieter", "no-level-word", "case-insensitive", "degree-without-not"])
def test_4159_level_phrases_cover_not_degree_and_the_level_word(words, phrases):
    assert list(level_phrases(words)) == phrases


def test_4159_degree_words_are_whole_lowercase_words_and_none_names_a_direction():
    """Слова степени - обычная речь («так», «очень»): сами направления не называют, иначе «так» открыло бы громкость."""
    for word in VOLUME_DEGREE_WORDS:
        assert word == word.lower() and "ё" not in word and " " not in word, word
        assert level_word(word) == "", word
    assert len(set(VOLUME_DEGREE_WORDS)) == len(VOLUME_DEGREE_WORDS)


# --- #4136 п.3 (n812): стоп-слово под музыкой будит искажённый вейк ----------------------------------------------

#: Фраза и остаток после вейка: под играющей музыкой стоп-слово будит. Первая - живая n812 28.10 07:03:19 («Робот, теперь
#: и музыку выключай.» → «роботы пере музыку выключай»), остальные - те же формы из таблиц.
STOP_UNDER_MUSIC = [
    ("роботы пере музыку выключай", "пере музыку выключай"),  # лог: n812 28.10
    ("роботы музыку выключай", "музыку выключай"),
    ("роботы выключай музыку", "выключай музыку"),
    ("работа всю музыку отключи", "всю музыку отключи"),
    ("рада музыка стоп", "музыка стоп"),
    ("роботы замолчите", "замолчите"),
    ("роботы останови музыку", "останови музыку"),
    ("роботы музыку глуши", "музыку глуши"),
    ("роботы бит помолчите", "бит помолчите"),
]
#: Те же слова вне просьбы: даже под музыкой не будят (формы не те, слово далеко от «роботы» или не к роботу).
STOP_NOT_WAKE_UNDER_MUSIC = [
    "роботы стоят", "роботы выключаются", "роботы всегда останавливаются", "роботы остановка", "роботы стоп-кран",
    "роботы захватят мир", "роботы очень любят музыку",
    # стоп-слово дальше окна WAKE_SLIP_STOP_WINDOW: речь о роботах, а не просьба
    "роботы любят музыку а потом сразу выключай",
]


@pytest.mark.parametrize("text,rest", STOP_UNDER_MUSIC)
def test_stop_word_wakes_under_music_and_only_under_music(text, rest):
    """«Музыку выключай» под играющей музыкой - просьба к роботу; без музыки та же фраза - разговор о чём-то другом."""
    assert slipped_wake_rest(text, music_playing=True) == rest
    assert slipped_wake_rest(text) is None, text


@pytest.mark.parametrize("text", STOP_NOT_WAKE_UNDER_MUSIC)
def test_stop_gate_does_not_wake_ordinary_speech_under_music(text):
    assert slipped_wake_rest(text, music_playing=True) is None, text


def test_stop_gate_words_are_the_stop_tables_not_a_copy():
    """Одна таблица знания (ADR-0148): формы стоп-ворот = точные стоп-слова Tier-1 плюс «выключи/отключи музыку»; своих
    слов нет, ё нет (сверка после ё → е), «выключи свет» в стоп-слова Tier-1 не попал."""
    assert set(WAKE_SLIP_STOP_FORMS) == set(STOP_WORDS) | set(MUSIC_OFF_WORDS)
    assert set(TIER1_POLICY["stop"].words) <= set(WAKE_SLIP_STOP_FORMS)
    assert not set(MUSIC_OFF_WORDS) & set(STOP_WORDS), "выключи - не стоп Tier-1: «выключи свет»"
    for word in WAKE_SLIP_STOP_FORMS:
        assert word == word.lower() and "ё" not in word and " " not in word, word
    stop_gates = [gate for gate in WAKE_SLIP_GATES if set(gate.leads) == set(WAKE_SLIP_STOP_FORMS)
                  or set(gate.forms) == set(WAKE_SLIP_STOP_FORMS)]
    assert len(stop_gates) == 2 and all(gate.under_music for gate in stop_gates), "стоп-ворота - только под музыкой"
    assert any(gate.open_lead and gate.window == WAKE_SLIP_STOP_WINDOW for gate in stop_gates)
    assert WAKE_SLIP_STOP_WINDOW < WAKE_SLIP_CUE_WINDOW


@pytest.mark.parametrize("word", ["выключи", "выключай", "отключи", "стоп", "хватит", "замолчи", "глуши"])
def test_stop_word_first_opens_the_gate_by_itself_under_music(word):
    assert slipped_wake_rest(f"роботы {word}", music_playing=True) == word


# --- хвосты ревью PR #4132 (#3975, пачка мелочи 9) ---------------------------------------------------------------

#: Канал голоса робота после слова правки: основы ``VOLUME_CHANNEL_STEMS["tts"]`` - знак просьбы, как музыка и громкость.
TTS_CHANNEL_UNDER_MUSIC = [
    ("роботы потише говори", "потише говори"),
    ("роботы громче говори", "громче говори"),
    ("роботы тише голос", "тише голос"),
    ("работа погромче говорить", "погромче говорить"),
    ("рада тише разговаривай", "тише разговаривай"),
    ("роботы громче голосом", "громче голосом"),
]


@pytest.mark.parametrize("text,rest", TTS_CHANNEL_UNDER_MUSIC)
def test_tts_channel_after_a_tweak_word_wakes_under_music_and_only_under_music(text, rest):
    assert slipped_wake_rest(text, music_playing=True) == rest
    assert slipped_wake_rest(text) is None, text


def test_tweak_cue_stems_take_the_tts_channel_from_the_volume_table_not_a_copy():
    """Одна таблица знания (ADR-0148): основы канала голоса - те, которыми Tier-1 громкость узнаёт «говори тише»."""
    assert set(VOLUME_CHANNEL_STEMS["tts"]) <= set(WAKE_SLIP_TWEAK_CUE_STEMS)
    assert set(VOLUME_NOUN_STEMS) <= set(WAKE_SLIP_TWEAK_CUE_STEMS)


#: «ещё/снова/опять» сами ничего не решают: дальше те же ворота правки, что и без них. Просьбы.
MORE_TWEAK_WAKES = [
    ("роботы ещё быстрее", "ещё быстрее"),
    ("роботы ещё громче", "ещё громче"),
    ("рада еще тише", "еще тише"),
    ("роботы ещё чуть быстрее", "ещё чуть быстрее"),
    ("роботы снова потише", "снова потише"),
    ("роботы опять быстрее пожалуйста", "опять быстрее пожалуйста"),
    ("роботы ещё ускорь темп", "ещё ускорь темп"),
    ("роботы ещё бас потише", "ещё бас потише"),
]
#: Рассказ: «ещё» + слово правки + любое продолжение не из знаков (хвост ревью: REACH смотрел одно слово после «ещё»).
MORE_TWEAK_STORIES = [
    "роботы ещё быстрее нас считают", "роботы ещё громче всех", "роботы снова тише воды", "роботы ещё чуть громче всех",
    "роботы опять быстрее людей", "роботы ещё быстро учатся", "роботы ещё не умеют быстрее", "роботы ещё", "роботы ещё чуть",
    "роботы ещё темп", "роботы ещё скорость растёт",
]


@pytest.mark.parametrize("text,rest", MORE_TWEAK_WAKES)
def test_more_before_a_tweak_request_wakes_under_music_only(text, rest):
    assert slipped_wake_rest(text, music_playing=True) == rest
    assert slipped_wake_rest(text) is None, f"{text}: без музыки ворота правки закрыты"


@pytest.mark.parametrize("text", MORE_TWEAK_STORIES)
def test_more_before_a_tweak_word_in_a_story_does_not_wake_even_under_music(text):
    assert slipped_wake_rest(text, music_playing=True) is None, text


@pytest.mark.parametrize("word", COMPARATIVES)
def test_more_does_not_change_what_the_comparative_decides(word):
    """«ещё» + сравнительная степень решается так же, как сама степень (кончилась фраза / знак дальше / рассказ)."""
    for tail in ("", " пожалуйста", " музыку", " говори", " всех", " нас считают", " чем вчера"):
        bare = slipped_wake_rest(f"роботы {word}{tail}", music_playing=True) is not None
        more = slipped_wake_rest(f"роботы ещё {word}{tail}", music_playing=True) is not None
        assert more is bare, f"{word}{tail}"


def test_more_prefix_gate_is_the_only_music_gate_with_more_leads_and_is_a_prefix():
    gates = [gate for gate in WAKE_SLIP_GATES if gate.under_music and set(WAKE_SLIP_MORE_LEADS) & set(gate.leads)]
    assert len(gates) == 1 and gates[0].prefix and not gates[0].forms and not gates[0].stems
    assert not any(gate.prefix for gate in WAKE_SLIP_GATES if not gate.under_music), "приставка - только про музыку"


#: Хвост ревью: достижимость существительного («темп», «бас») - СРАЗУ за ним, а не где-то в окне; мутация reach=window
#: не краснела: все прежние негативные фразы не содержали слова правки вообще.
NOUN_REACH_NEXT_ONLY = ["роботы темп мы задаём потише", "роботы бас играет потише", "роботы скорость тут растёт быстрее"]


@pytest.mark.parametrize("text", NOUN_REACH_NEXT_ONLY)
def test_noun_lead_needs_the_tweak_word_right_after_it_not_anywhere_in_the_window(text):
    assert slipped_wake_rest(text, music_playing=True) is None, text


def test_noun_gate_reaches_only_the_next_word():
    gates = [gate for gate in WAKE_SLIP_GATES if gate.under_music and gate.lead_stems == WAKE_SLIP_TWEAK_NOUN_STEMS]
    assert len(gates) == 1 and gates[0].reach == REACH_NEXT and set(MUSIC_AMBIGUOUS_WORDS) <= set(gates[0].leads)
