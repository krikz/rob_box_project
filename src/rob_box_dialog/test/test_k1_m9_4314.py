"""#4314 класс К1 марафона М9 (фраза или обещание не по исходу), решает код (ADR-0148).

* n907: ``search_web`` исполнен, а ответ «Поищу ещё раз, уточню запрос.» - второго поиска не было. Обещание поискать без
  тула поиска - нарушение ``search_promise`` (таблица ``knowledge.SEARCH_PROMISES``, как ``VOICE_PROMISES`` #4020);
  после повтора говорит код: поиск в ходе был - по его данным, не было - «поиск я не запускал».
* n802: «посчитай вслух от одного до сорока» → «Считаю, Саша. Один… Десять.» - модель молча урезала счёт. Сколько
  просили, код знает из слов человека; не досчитала - числа строит код (``counting.full_count``).
* n211: «голос той женщины я не запомнил», хотя профиль без имени «Тихий» заведён (#3858): реплика в истории модели шла
  с меткой «незнакомый голос». Свой профиль без имени - метка «голос без имени, запомнен».
"""

import asyncio

import pytest

from rob_box_dialog import counting
from rob_box_dialog import foreign_name
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, Performance, ToolCall, explain, parse, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import phrase_speaker, voice_tag
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.turn_log import phrase_label, speaker_label
from rob_box_dialog.words import search_promises

TOOLS = ("search_web", "memory_search", "music_library", "get_current_time", "play_animation")
#: Живой лог М9 00:16:49 (n907): ответ модели после ``search_web``.
N907_SPEECH = ("Саша, поиск выдал только обзорные страницы новостей, без конкретных событий недели. Поищу ещё раз,"
               " уточню запрос.")
N907_TEXT = "робот поищи в интернете что нового в космосе за последнюю неделю"
#: Живой лог М9 00:04:32-35 (n802): реплика и ответ модели.
N802_TEXT = "робот посчитай вслух от одного до сорока не торопясь с выражением"
N802_SPEECH = "Считаю, Саша. Один. Два. Три. Четыре. Пять. Шесть. Семь. Восемь. Девять. Десять."


class Ports:
    """Транспорт хода: ответы LLM по очереди, исходы действий - ``done`` с данными ``outcomes``."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.said, self.performed = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             "done", None, {**dict(a.args), **self.outcomes.get(a.tool, {})})
                for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        self.performed.append(perf)
        return None


def _run(ports, text):
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


# -- n907: обещание поискать без тула поиска ----------------------------------------------------------------------


@pytest.mark.parametrize("text, kinds", [
    (N907_SPEECH, ("search",)),
    ("Посмотрю в интернете и скажу.", ("search",)),
    ("Сейчас погуглю.", ("search",)),
    ("Уточню запрос и вернусь.", ("search",)),
    ("Найду ещё раз, не переживай.", ("again",)),
])
def test_search_promises_are_found_by_words(text, kinds):
    assert search_promises(text) == kinds


@pytest.mark.parametrize("text", [
    "Поискал: конкретики нет.",          # прошедшее - правда после поиска
    "Нашёл три статьи про Марс.",
    "Проверю ещё раз батарею.",           # «ещё раз» не с «найду» - не про поиск
    "Попробую ещё раз спеть.",
    "Посмотрю на тебя внимательно.",
    "",
])
def test_not_a_search_promise(text):
    assert search_promises(text) == ()


def test_search_table_is_one_knowledge():
    assert set(kn.SEARCH_PROMISES) == {"search", "again"}
    assert all(set(p.tools) == set(kn.SEARCH_PROMISE_TOOLS) for p in kn.SEARCH_PROMISES.values())
    assert "search_web" in kn.SEARCH_PROMISE_TOOLS


def test_n907_search_promise_without_search_tool_is_invalid_with_hint():
    """00:16:49 М9: ответ после поиска обещает искать снова, а ``actions`` пуст - нарушение с подсказкой."""
    result = validate(parse({"speech": N907_SPEECH}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_SEARCH_PROMISE
    assert kn.SEARCH_PROMISE_HINT in explain(result)


def test_search_promise_with_search_tool_is_kept():
    cmd = Command(actions=(ToolCall("search_web", {"query": "новости космоса", "max_results": 6}),),
                  speech=None)
    result = validate(cmd, TOOLS)
    assert not (isinstance(result, Invalid) and kn.VIOLATION_SEARCH_PROMISE in {v.reason for v in result.all})


def test_n907_after_retry_code_answers_by_the_search_data_not_the_promise():
    """Поиск в ходе был, модель и после повтора обещает искать снова: обещание не звучит, код говорит по данным
    поиска (``turn.query_fallback``), а не «поищу ещё»."""
    search = Command(actions=(ToolCall("search_web", {"query": "новости космоса за неделю", "max_results": 6}),))
    broken = Invalid(kn.VIOLATION_SEARCH_PROMISE, "speech обещает search без тула поиска")
    data = {"results": [{"title": "Новости космоса", "body": "Обзор", "url": "https://example.org"}]}
    ports = Ports([Answer(search, "deepseek"), Answer(broken, "deepseek", calls=2)], {"search_web": data})
    _run(ports, N907_TEXT)
    assert ports.executed == [["search_web"]]
    assert ports.said and not any("Поищу" in s for s in ports.said)
    assert ports.said[-1] != phrases.text(kn.SEARCH_PROMISE_PHRASE)


def test_search_promise_without_any_search_says_it_did_not_search():
    broken = Invalid(kn.VIOLATION_SEARCH_PROMISE, "speech обещает search без тула поиска")
    ports = Ports([Answer(broken, "deepseek", calls=2)])
    _run(ports, N907_TEXT)
    assert ports.said == [phrases.text(kn.SEARCH_PROMISE_PHRASE)]


def test_search_promise_phrase_in_catalog():
    assert kn.SEARCH_PROMISE_PHRASE in kn.PHRASE_KEYS and phrases.text(kn.SEARCH_PROMISE_PHRASE)


# -- n802: счёт, урезанный моделью, строит код ---------------------------------------------------------------------


@pytest.mark.parametrize("text, asked", [
    (N802_TEXT, (1, 40)),
    ("посчитай до двадцати", (1, 20)),
    ("сосчитай от пяти до тридцати пяти", (5, 35)),
    ("считай от десяти до одного", (10, 1)),
    ("досчитай до 15", (1, 15)),
    ("расскажи сказку до утра", None),         # «до» без счёта
    ("посчитай, сколько нас", None),           # счёт без «до N»
])
def test_asked_range_from_the_person_words(text, asked):
    assert counting.asked_range(text) == asked


def test_n802_short_count_is_built_by_code_to_forty():
    """00:04:35 М9: модель досчитала до десяти - числа до сорока строит код, выступлением по десятку."""
    full = counting.full_count(Command(speech=N802_SPEECH), N802_TEXT)
    assert full is not None and full.speech is None and full.perform is not None
    assert full.perform.kind == "poem" and len(full.perform.segments) == 4
    said = [n for segment in full.perform.segments for n in counting.said_numbers(segment)]
    assert said == list(range(1, 41))
    assert all(len(s) <= kn.SEGMENT_MAX_CHARS for s in full.perform.segments)
    assert len(full.perform.segments) <= kn.SEGMENTS_MAX


def test_short_count_that_fits_speech_stays_speech():
    full = counting.full_count(Command(speech="Один, два, три."), "посчитай до двенадцати")
    assert full.perform is None and counting.said_numbers(full.speech) == tuple(range(1, 13))
    assert len(full.speech) <= kn.SPEECH_MAX_CHARS


@pytest.mark.parametrize("cmd", [
    Command(speech="Раз, два, три… тридцать девять и сорок!"),                     # досчитала
    Command(perform=Performance("poem", ("Один. Два. Двадцать.", "Тридцать девять. Сорок."))),
    Command(speech="До сорока? Это долго, точно считать?"),                        # не начала - переспрос
    Command(speech="Один, два", edit_pending=True,
            perform=Performance("poem", ("Один, два.", "Три."))),                  # правка идущего - не новый счёт
])
def test_model_count_stays_when_complete_or_not_started(cmd):
    assert counting.full_count(cmd, N802_TEXT) is None


def test_count_above_the_ceiling_is_capped_with_an_honest_code_phrase():
    full = counting.full_count(Command(speech="Один, два."), "посчитай до 150")
    said = [n for segment in full.perform.segments for n in counting.said_numbers(segment)]
    assert said[-1] == kn.COUNT_MAX and len(full.perform.segments) <= kn.SEGMENTS_MAX
    assert full.perform.segments[0].startswith(phrases.text(kn.COUNT_CAPPED_PHRASE, top="ста"))


def test_n802_turn_performs_the_code_count():
    """Ход целиком: ответ модели «Один… Десять.» на «до сорока» - выступление кода до сорока, речь модели не звучит."""
    ports = Ports([Answer(Command(speech=N802_SPEECH), "deepseek")])
    _run(ports, N802_TEXT)
    assert N802_SPEECH not in ports.said
    spoken = " ".join(ports.said + [s for p in ports.performed for s in p.segments])
    assert counting.said_numbers(spoken)[-1] == 40


# -- n211: свой профиль голоса без имени - не «незнакомый голос» ---------------------------------------------------


def test_n211_nameless_profile_is_labelled_remembered_not_unknown():
    """22:59:14 М9: «незнакомый голос → профиль без имени id=3d41f655 кличка 'Тихий'» - метка автора реплики в
    истории модели говорит, что голос запомнен; голос без профиля - по-прежнему незнакомый."""
    assert speaker_label(SpeakerRef(person_ref="3d41f655", name=None)) == kn.NAMELESS_SPEAKER_LABEL
    assert speaker_label(SpeakerRef(person_ref=None, name=None)) == kn.UNKNOWN_SPEAKER_LABEL
    assert speaker_label(SpeakerRef(person_ref="51a5a6e7", name="Саша", name_state="confirmed")) == "Саша"
    assert speaker_label(None) == kn.UNKNOWN_SPEAKER_LABEL


def test_n211_backlog_phrase_of_a_nameless_profile_is_remembered():
    nameless = {"is_known": True, "speaker_id": "3d41f655", "name": None, "epithet": "Тихий", "source": "enroll"}
    assert phrase_label(phrase_speaker(voice_tag(nameless))) == kn.NAMELESS_SPEAKER_LABEL
    unknown = {"is_known": False, "speaker_id": None}
    assert phrase_label(phrase_speaker(voice_tag(unknown))) == kn.UNKNOWN_SPEAKER_LABEL
    tentative = {"is_known": True, "speaker_id": "x", "tentative_name": "Борис", "confidence": 0.6}
    assert phrase_label(phrase_speaker(voice_tag(tentative))) == kn.UNKNOWN_SPEAKER_LABEL  # гипотеза - не профиль


def test_nameless_label_is_not_a_name_for_foreign_name_guard():
    assert foreign_name.name_keys([kn.NAMELESS_SPEAKER_LABEL, kn.UNKNOWN_SPEAKER_LABEL]) == frozenset()
