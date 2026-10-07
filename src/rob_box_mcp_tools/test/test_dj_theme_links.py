"""Живой случай 07.10 ~15:10 UTC (товарищ Шифу): «танец маленьких утят, лебединое озеро, … лайон кинг» играл тетрис.

Прямой поиск по русским словам не находит ни одной части (архив англоязычный), а в архиве есть «Chicken Dance»
(``chickend``), «Swan Lake» (``swanlake``, ``swanlake_2``), записи «Lion King». Общий путь #3493: часть без находок →
LLM предлагает строки поиска → код проверяет каждую каталогом → проверенное кешируется в реестре (``works.theme_links``)
→ повтор без LLM. Ничего не нашлось у названного — сет не стартует пулом. Тесты — на настоящем архиве и фейковой LLM
с детерминированными предложениями.
"""

from __future__ import annotations

import sqlite3
import threading
import time
from contextlib import closing
from datetime import datetime, timedelta
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import ThemeHits, theme_search
from rob_box_mcp_tools.engine.theme_links import ThemeLinks, verify
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, library_melodies, theme_finder
from rob_box_music import theme_queries as tq
from rob_box_music.dj_line import SET_NOT_FOUND
from rob_box_music.works import ThemeLink, put_theme_link, theme_links_report

from .test_engine_session import _rig, _started

pytestmark = pytest.mark.unit

DUCKS = "танец маленьких утят"
LIST = "танец маленьких утят, лебединое озеро, король лев"
LION_KING = {"can_twai", "cantwait", "hakunama", "hakunama_2", "hakunama_3", "canyoufe", "canyoufe_2"}
#: Что «знает» фейковая LLM: часть → (вид, строки поиска). Ложные и общие строки — нарочно.
ANSWERS = {
    DUCKS: ("work", ["Chicken Dance", "Vogeltanz", "Birdie Song", "Dance"]),
    "лебединое озеро": ("work", ["Swan Lake", "Tchaikovsky Swan Lake Op. 20"]),
    "король лев": ("work", ["Lion King", "Hakuna Matata", "Can You Feel the Love Tonight", "Circle of Life"]),
    "животных": ("concept", ["The Lion Sleeps Tonight", "Baby Elephant Walk", "Pink Panther",
                             "Flight of the Bumblebee", "Zebra Dance"]),
    "кукушка": ("work", ["Cuckoo Waltz"]),
    "ламповое": ("not_theme", []),
}


class FakeLLM:
    def __init__(self, answers=ANSWERS, delay=0.0):
        self.answers, self.delay, self.calls = answers, delay, []

    def __call__(self, system, user, tool, deadline_s, max_tokens):
        parts = tool["function"]["parameters"]["properties"]["suggestions"]["items"]["properties"]["part"]["enum"]
        self.calls.append(list(parts))
        time.sleep(self.delay)
        args = {"suggestions": [{"part": p, "kind": self.answers[p][0], "queries": self.answers[p][1]}
                                for p in parts if p in self.answers]}
        call = SimpleNamespace(name=tq.SUBMIT_TOOL, arguments=args)
        return "ok", SimpleNamespace(tool_calls=[call], truncated_tool_args=False), ""


@pytest.fixture(scope="module")
def archive(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("links") / "voice_memory.db"))


@pytest.fixture()
def library(archive, tmp_path):
    """Архив модуля, но своё хранилище связей на тест: кеш одного теста не помогает другому."""
    db = str(tmp_path / "links.db")
    return SimpleNamespace(**{k: getattr(archive, k) for k in ("search", "get", "vocabulary")}, db_path=db,
                           _db_path=db)


def _links(llm=None, **kw):
    return ThemeLinks(ask=llm, spawn=lambda fn: fn(), **kw)


_DIRECT = {}


def _direct(library, theme):
    """Прямой поиск по архиву (секунды на тему) — один раз на модуль: хранилище связей он не трогает."""
    if theme not in _DIRECT:
        _DIRECT[theme] = theme_search(library, theme)
    return _DIRECT[theme]


def _expand(links, library, theme):
    return links.expand(library, theme, _direct(library, theme))


def test_direct_search_finds_nothing_for_the_live_list(library):
    hits = _direct(library, LIST)
    assert hits.names == () and hits.named and set(hits.missing) == {DUCKS, "лебединое озеро", "король лев"}


def test_llm_queries_are_checked_by_the_catalog_and_only_found_records_play(library):
    llm = FakeLLM()
    hits = _expand(_links(llm), library, DUCKS)
    assert hits.names == ("chickend",) and hits.missing == ()
    with closing(sqlite3.connect(library.db_path)) as conn:
        queries, rejected, source = conn.execute(
            "SELECT queries, rejected, source FROM theme_links WHERE phrase=?", (DUCKS,)).fetchone()
    assert queries == '["Chicken Dance"]' and source == "llm"
    assert set(eval(rejected)) == {"Vogeltanz", "Birdie Song", "Dance"}  # нет в каталоге / слишком общая строка
    again = _links(llm).expand(library, "Танец маленьких утят!", _direct(library, DUCKS))  # повтор — из реестра
    assert again.names == ("chickend",) and len(llm.calls) == 1


def test_live_list_plays_every_named_work_and_dj_set_starts_with_them(library):
    llm = FakeLLM()
    hits = _expand(_links(llm), library, LIST)
    assert hits.missing == () and "chickend" in hits.names
    assert {"swanlake", "swanlake_2"} <= set(hits.names) and set(hits.names) & LION_KING
    assert [set(p) for p in hits.parts][0] == {"chickend"}  # части в порядке названного
    rig = _rig()  # тот же заказ через dj_set: связи уже в реестре — без LLM
    seen = []
    finder = theme_finder(lambda: library, _links(llm))
    dj = DjSetTool(None, rig.owner, melodies=library_melodies(lambda: library), seed=lambda: 4242,
                   finder=lambda theme: seen.append(finder(theme)) or seen[-1])
    data = dj.execute(action="start", heard_text=f"Робот, включи сет: {LIST}").data
    assert data["theme_source"] == "theme" and len(llm.calls) == 1 and set(seen[0].names) == set(hits.names)
    rig.clock.run_until(rig.clock.beat + 2)
    assert _started(rig)[0]["track_id"] == data["track_id"]


def test_concept_becomes_concrete_works_from_the_catalog(library):
    hits = _expand(_links(FakeLLM()), library, "животных")
    assert {"babyelep", "flightof"} & set(hits.names) and any(n.startswith("pinkpant") for n in hits.names)
    assert "zebra" not in " ".join(hits.names)


def test_not_found_named_work_is_missing_and_marked_named(library):
    hits = _expand(_links(FakeLLM()), library, "кукушка")
    assert hits.names == () and hits.missing == ("кукушка",) and hits.named


def test_not_theme_part_is_dropped_from_missing(library):
    hits = _expand(_links(FakeLLM()), library, "ламповое, лебединое озеро")
    assert hits.missing == () and set(hits.names) <= {"swanlake", "swanlake_2"} and hits.names


def test_exception_added_as_data_changes_the_search_without_code(library):
    assert _expand(_links(), library, "танец утят").names == ()
    with closing(sqlite3.connect(library.db_path)) as conn:
        put_theme_link(conn, ThemeLink("танец утят", "found", "manual", "work", ("chicken dance",)), datetime.now())
    assert _expand(_links(), library, "танец утят").names == ("chickend",)


def test_generic_query_is_not_a_work_name(library):
    assert verify(library, "dance") == () and verify(library, "chicken dance") == ("chickend",)


def test_late_llm_does_not_hold_the_set_and_its_answer_is_cached(library):
    llm = FakeLLM(delay=0.3)
    spawned = []

    def spawn(fn):
        spawned.append(threading.Thread(target=fn))
        spawned[-1].start()

    links = ThemeLinks(ask=llm, wait_s=0.05, spawn=spawn)
    direct = theme_search(library, DUCKS)
    started = time.perf_counter()
    assert links.expand(library, DUCKS, direct).names == ()
    assert time.perf_counter() - started < 0.25
    spawned[0].join()
    assert _expand(links, library, DUCKS).names == ("chickend",) and len(llm.calls) == 1


def test_unknown_phrases_pile_up_in_the_journal_report(library):
    links = _links()  # LLM выключена: вердикта нет — в журнал непонятого
    for _ in range(3):
        _expand(links, library, "зебра")
    with closing(sqlite3.connect(library.db_path)) as conn:
        report = theme_links_report(conn, 30, datetime.now() + timedelta(seconds=1))
    assert "3  missed    «зебра»" in report


def _dj(library, llm):
    rig = _rig()
    finder = theme_finder(lambda: library, _links(llm))
    dj = DjSetTool(None, rig.owner, melodies=library_melodies(lambda: library), finder=finder, seed=lambda: 4242)
    return rig, dj


def test_named_list_with_nothing_found_does_not_start_a_random_pool(library):
    rig, dj = _dj(library, None)
    result = dj.execute(action="start", heard_text=f"Робот, включи сет: {LIST}")
    assert not result.success and result.data["reason"] == "not_found"
    assert result.error.startswith(SET_NOT_FOUND) and "«лебединое озеро»" in result.error
    rig.clock.run_until(rig.clock.beat + 2)
    assert _started(rig) == [] and dj._session is None


def test_concept_without_llm_still_plays_the_pool_as_before(library):
    rig, dj = _dj(library, None)
    data = dj.execute(action="start", heard_text="Робот, включи сет на тему животных").data
    assert data["ok"] and data["theme_source"] == "pool"


def test_empty_theme_is_untouched(library):
    assert _links(FakeLLM()).expand(library, "", ThemeHits()) == ThemeHits()
