"""#3609 — фраза заказа одиночной пьесы → слоты (композитор, произведение, инструмент, стиль, настроение, режим),
каждое поле проверено каталогом; механизм — как у theme-links (LLM даёт узкую схему, код проверяет, кэш).

Решение Шифу 13.10: «фразу надо кормить ЛЛМке, чтоб она делала разбор на поисковые фразы». Код-разбор первым (все
слова известны — без LLM: «сыграй к Элизе»), LLM — только при непонятых словах, ждём ≤ 1.5 с, кэш разборов. Тесты —
на настоящем архиве RTTTL и фейковой LLM с детерминированными ответами.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.piece import levels
from rob_box_mcp_tools.engine.request_slots import Catalog, HarnessAsk, RequestSlotParser
from rob_box_mcp_tools.engine.search import terms
from rob_box_music import knowledge as kn
from rob_box_music import request_slots as rs
from rob_box_music.theme_queries import QueriesInvalid
from rob_box_music.works import RTTTL, SCORE, WorkCandidate, WorkRef, WorkWord
from rob_box_voice.core.media_command_grammar import parse_media_command

pytestmark = pytest.mark.unit

EMPTY = {"composer": "", "work": "", "instrument": "", "style": "", "mood": "", "mode": "piece"}
#: Что «знает» фейковая LLM: фраза → слоты. Ложное произведение и чужой композитор — нарочно.
ANSWERS = {
    "давай вагнера полет волкирий ебанем": {"composer": "Richard Wagner", "work": "Ride of the Valkyries"},
    "давай вагнера полет волкирий врубай": {"composer": "Richard Wagner", "work": "Ride of the Valkyries"},
    "сыграй танец маленьких утят": {"work": "Chicken Dance"},
    "сыграй полет шмеля на ксилофоне": {"composer": "Rimsky-Korsakov", "work": "Flight of the Bumblebee",
                                        "instrument": "xylophone"},
    "сыграй абырвалг шмабырвалг": {"composer": "Zzyzx Qwertyson", "work": "Abyrvalg Opus 999"},
}


class FakeLLM:
    def __init__(self, answers=ANSWERS, delay=0.0, outcome="ok"):
        self.answers, self.delay, self.outcome, self.calls = answers, delay, outcome, []

    def __call__(self, system, user, tool, deadline_s, max_tokens):
        text = user.split("«", 1)[1].split("»", 1)[0]
        self.calls.append(text)
        time.sleep(self.delay)
        if self.outcome != "ok":
            return self.outcome, None, "фейк"
        args = {**EMPTY, **self.answers.get(text, {})}
        call = SimpleNamespace(name=tool["function"]["name"], arguments=args)
        return "ok", SimpleNamespace(tool_calls=[call], truncated_tool_args=False), ""


class NoScores:
    """Индекса партитур нет (как на машине без ``score_index.db``): каталог — только RTTTL."""

    def composer_role(self, word):
        return None

    def work_candidates(self, ref, limit=8, hooks=()):
        return []


@pytest.fixture(scope="module")
def archive(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("slots") / "voice_memory.db"))


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    warning = info


def _parser(archive, llm=None, spawn=lambda fn: fn(), wait_s=1.5, log=None):
    log = log or Log()
    parser = RequestSlotParser(lambda text: terms(archive, text), Catalog(lambda: archive, NoScores()), ask=llm,
                               wait_s=wait_s, spawn=spawn, log=lambda: log)
    return parser, log


def _parse(parser, text):
    command = parse_media_command(text)
    return parser.parse(text, command.name, command)


# ── набор фраз из issue ─────────────────────────────────────────────────────────────────────────────────────────

def test_wagner_with_typo_and_junk_becomes_wagner_ride_of_the_valkyries(archive):
    llm = FakeLLM()
    parser, log = _parser(archive, llm)
    slots = _parse(parser, "давай вагнера полет волкирий ебанем")
    assert llm.calls == ["давай вагнера полет волкирий ебанем"]
    assert slots.source == "llm" and slots.composer == "Wagner" and slots.work == "Ride of the Valkyries"
    assert slots.work_confirmed and slots.query == "Wagner Ride of the Valkyries"
    assert set(slots.unknown) == {"полет", "ебанем"} and slots.rejected == ()
    line = log.lines[-1]
    assert "[request-slots]" in line and "source=llm" in line and " ms=" in line and "llm=1/1" in line


def test_chicken_dance_by_its_russian_name(archive):
    slots = _parse(_parser(archive, FakeLLM())[0], "сыграй танец маленьких утят")
    assert slots.work == "Chicken Dance" and slots.work_confirmed and slots.query == "Chicken Dance"


def test_bach_toccata_on_organ_needs_no_llm(archive):
    llm = FakeLLM()
    slots = _parse(_parser(archive, llm)[0], "врубай токкату баха на органе")
    assert llm.calls == [] and slots.source == "code"
    assert slots.instrument == "organ" and slots.query == "токкату баха"


def test_something_by_mozart_needs_no_llm(archive):
    llm = FakeLLM()
    slots = _parse(_parser(archive, llm)[0], "сыграй что-нибудь из Моцарта")
    assert llm.calls == [] and slots.source == "code" and slots.query == "из моцарта"


def test_valkyries_on_sitar_is_work_plus_instrument_from_code(archive):
    llm = FakeLLM()
    slots = _parse(_parser(archive, llm)[0], "сыграй валькирии на ситаре")
    assert llm.calls == [] and slots.instrument == "sitar" and slots.query == "валькирии"


def test_fur_elise_is_parsed_by_code_without_llm(archive):
    """«сыграй к Элизе» — без LLM вообще (``ask=None``): семя реестра знает фразу."""
    slots = _parse(_parser(archive, None)[0], "сыграй к Элизе")
    assert slots.source == "code" and slots.unknown == () and slots.query == "к элизе"


# ── проверка каталогом ──────────────────────────────────────────────────────────────────────────────────────────

def test_unconfirmed_fields_are_dropped_and_logged(archive):
    """Выдуманные композитор и произведение — пусто, в лог «отброшено»; строка поиска — известные слова кода."""
    parser, log = _parser(archive, FakeLLM())
    slots = _parse(parser, "сыграй абырвалг шмабырвалг")
    assert slots.composer == "" and not slots.work_confirmed
    assert slots.rejected == ("composer=Zzyzx Qwertyson", "work=Abyrvalg Opus 999")
    assert "отброшено=['composer=Zzyzx Qwertyson', 'work=Abyrvalg Opus 999']" in log.lines[-1]
    assert slots.query == "абырвалг шмабырвалг"  # известных слов нет — фраза как была


def test_instrument_from_llm_must_be_from_the_table():
    with pytest.raises(QueriesInvalid):
        rs.validate({**EMPTY, "instrument": "theremin"})
    assert rs.validate({**EMPTY, "instrument": "Xylophone "})["instrument"] == "xylophone"


def test_schema_enums_are_the_knowledge_tables():
    props = rs.schema()["properties"]
    assert props["instrument"]["enum"] == ["", *kn.INSTRUMENTS]
    assert props["style"]["enum"] == ["", *kn.STYLES]
    assert props["mood"]["enum"] == ["", *kn.MOOD_ENERGY]
    assert props["mode"]["enum"] == list(rs.MODES)
    assert rs.schema()["additionalProperties"] is False and set(rs.schema()["required"]) == set(rs.FIELDS)


def test_grammar_instrument_wins_over_llm(archive):
    """«на ксилофоне» назвала грамматика — слот кода; LLM его не переписывает."""
    answers = {"сыграй полет шмеля на ксилофоне": {"work": "Flight of the Bumblebee", "instrument": "harp"}}
    slots = _parse(_parser(archive, FakeLLM(answers))[0], "сыграй полет шмеля на ксилофоне")
    assert slots.instrument == "xylophone"


# ── скорость: дедлайн, кэш, опоздавший ответ ────────────────────────────────────────────────────────────────────

def test_second_same_phrase_comes_from_the_cache(archive):
    llm = FakeLLM()
    parser, log = _parser(archive, llm)
    _parse(parser, "давай вагнера полет волкирий ебанем")
    slots = _parse(parser, "давай вагнера полет волкирий ебанем")
    assert llm.calls == ["давай вагнера полет волкирий ебанем"] and slots.source == "cache"
    assert slots.query == "Wagner Ride of the Valkyries" and "cache=1" in log.lines[-1]


def test_late_llm_falls_back_to_code_and_fills_the_cache(archive):
    """LLM опоздала — код-разбор: непонятые слова не в строке поиска (#3608: на них theme_search тратил 1.8 с);
    опоздавший ответ ложится в кэш — следующий заказ без LLM."""
    jobs = []
    parser, log = _parser(archive, FakeLLM(), spawn=jobs.append, wait_s=0.01)
    slots = _parse(parser, "давай вагнера полет волкирий ебанем")
    assert slots.source == "code" and slots.query == "вагнера валкирий"
    assert any("не ответила за" in line for line in log.lines)
    jobs[0]()  # ответ пришёл в фоне
    again = _parse(parser, "давай вагнера полет волкирий ебанем")
    assert again.source == "cache" and again.query == "Wagner Ride of the Valkyries"


def test_llm_error_is_code_parse(archive):
    slots = _parse(_parser(archive, FakeLLM(outcome="error"))[0], "давай вагнера полет волкирий ебанем")
    assert slots.source == "code" and slots.query == "вагнера валкирий"


def test_wait_is_the_knowledge_constant():
    assert kn.REQUEST_SLOTS_WAIT_MS <= 1500
    parser = RequestSlotParser(lambda t: (), Catalog(lambda: None, NoScores()))
    assert parser._wait_s == kn.REQUEST_SLOTS_WAIT_MS / 1000.0


# ── провайдер: та же LLM-ступень, что у выбора ──────────────────────────────────────────────────────────────────

class FakeHarness:
    model = "fake"

    def __init__(self, answer=None, delay=0.0, error=None):
        self.answer, self.delay, self.error, self.seen = answer, delay, error, []

    async def complete_json(self, system, user, schema):
        import asyncio

        self.seen.append(schema)
        await asyncio.sleep(self.delay)
        if self.error:
            raise self.error
        return self.answer


def test_harness_ask_returns_the_tool_call_contract():
    ask = HarnessAsk(FakeHarness({**EMPTY, "composer": "Wagner"}))
    outcome, response, _detail = ask("s", "u", rs.tool(), 1.0)
    assert outcome == "ok" and response.tool_calls[0].name == rs.SUBMIT_TOOL
    assert response.tool_calls[0].arguments["composer"] == "Wagner"


def test_harness_ask_late_and_breaker():
    ask = HarnessAsk(FakeHarness({}, delay=0.2))
    assert ask("s", "u", rs.tool(), 0.01)[0] == "late"
    failing = HarnessAsk(FakeHarness(error=RuntimeError("boom")))
    assert [failing("s", "u", rs.tool(), 1.0)[0] for _ in range(4)] == ["error", "error", "error", "circuit_open"]


# ── выход — в кандидатов: названное произведение не вытесняет свежесть ──────────────────────────────────────────

def test_confirmed_work_keeps_only_the_exact_title_in_the_first_level():
    """Решение Шифу 13.10 (живой случай #3608: «Wedding March» вместо недавно игравшей «Ride Of The Valkyries»):
    произведение подтверждено — первый уровень только оно, частичное совпадение — в «остальное»."""
    ref = WorkRef((WorkWord(("wagner",)),), (WorkWord(("ride",)), WorkWord(("valkyries",))))
    ride = WorkCandidate("rideofth", RTTTL, "Ride Of The Valkyries", "Wagner", 1.0, 1.0)
    march = WorkCandidate("weddingm", RTTTL, "Wedding March", "Wagner", 0.5, 1.0)
    score = WorkCandidate("pdmx:1", SCORE, "Lohengrin", "Wagner", 0.0, 1.0)
    assert levels(ref, [march, ride, score], confirmed=True) == [[], [ride], [march, score]]
    assert levels(ref, [march, ride, score]) == [[], [march, ride], [score]]  # без подтверждения — как было


def _request(archive, llm, history):
    """``request_music`` на настоящем владельце плеера: находки — поиск по строке слотов на настоящем архиве."""
    from rob_box_mcp_tools.engine.search import ThemeHits, ThemeQuery, part_query
    from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool

    from .test_engine_session import _rig
    from .test_piece_fail_open import Scores, _records
    from melodies import LONG, SHORT  # noqa: E402  (путь добавлен test_piece_fail_open)

    table = {"rideofth": ("Ride Of The Valkyries", LONG, "Wagner"), "theride": ("The Ride", SHORT, "Wagner"),
             "weddingm": ("Wedding March", SHORT, "Wagner")}
    asked = []

    def finder(theme):
        asked.append(theme)
        return ThemeHits(tuple(table), query=ThemeQuery(theme, (part_query(archive, theme),)))

    rig = _rig()
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=finder, seed=lambda: 4242, scores=Scores())
    parser, log = _parser(archive, llm)
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, history=history,
                           records=_records(table), words=lambda word: ThemeHits(), slots=parser)
    return req, asked, log


def test_named_and_confirmed_work_plays_even_when_it_just_played(archive):
    """«давай вагнера полет волкирий врубай»: LLM → Wagner / Ride of the Valkyries (каталог подтвердил); «Ride Of The
    Valkyries» недавно играла дважды, «The Ride» Вагнера свежа и совпадает частично — играет названное."""
    from rob_box_music.diversity import MusicHistory

    history = MusicHistory(":memory:")
    for _ in range(2):
        history.record(kit=kn.CLASSIC_HISTORY_KIT, melody_name="rideofth")
    req, asked, _log = _request(archive, FakeLLM(), history)
    data = req.execute(intent="melody", text="давай вагнера полет волкирий врубай").data
    assert asked == ["Wagner Ride of the Valkyries"]  # непонятые слова в поиск не ушли
    assert data["ok"] and data["melody_id"] == "rideofth"
    assert "врубай" not in data["message"] and "полет" not in data["message"]


def test_instrument_of_the_turn_is_the_code_slot(archive):
    """Живой прогон 13.10: LLM диалога передала в ``text`` только «валькирии», «на ситаре» — в реплике хода; слот
    инструмента — из неё (грамматика), LLM разбора не зовётся."""
    llm = FakeLLM()
    req, asked, log = _request(archive, llm, None)
    req.execute(intent="melody", text="валькирии", heard_text="Робот сыграй валькирии на ситаре")
    line = next(x for x in log.lines if "[request-slots] «валькирии»" in x)
    assert "instrument=sitar" in line and "source=code" in line and llm.calls == []
