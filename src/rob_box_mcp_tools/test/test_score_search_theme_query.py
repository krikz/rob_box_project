"""#3512: тема → строки поиска одной функцией (``search.part_query``) для мелодий RTTTL и партитур.

Живой прогон 07.10 на паке 11 915 партитур: «Бах и Моцарт», «классическая музыка», «музыка из фильмов» →
``материалы=[]`` (поиск партитур требовал все слова темы в названии и не знал семян реестра и жанра), «Bach» → 5+.
Индекс партитур — синтетический (строки ``score_index`` как у PDMX: латинские названия, композитор, жанры через «-»).
"""

from __future__ import annotations

import json
import logging
from types import SimpleNamespace

import pytest

from rob_box_music import works

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.score_library import ScoreIndex
from rob_box_mcp_tools.engine.search import ThemeHits, ThemeQuery, part_query, theme_search
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool

from .test_engine_session import _rig

pytestmark = pytest.mark.unit

ROWS = (
    {"material_id": "pdmx:bach1", "title": "Minuet in G BWV Anh. 114", "composer": "Johann Sebastian Bach",
     "genres": "classical", "rating": 4.9, "n_ratings": 300},
    {"material_id": "pdmx:bach2", "title": "Bach - Prelude in C", "composer": "J. S. Bach", "genres": "classical",
     "rating": 4.8, "n_ratings": 90},
    {"material_id": "pdmx:moz1", "title": "Eine kleine Nachtmusik", "composer": "Wolfgang Amadeus Mozart",
     "genres": "classical", "rating": 4.85, "n_ratings": 200},
    {"material_id": "pdmx:moz2", "title": "Rondo alla Turca - Mozart", "composer": "W. A. Mozart",
     "genres": "classical-soundtrack", "rating": 4.7, "n_ratings": 80},
    {"material_id": "pdmx:bachelor", "title": "Bachelor Party", "composer": "Bacharach", "genres": "pop",
     "rating": 5.0, "n_ratings": 10},
    {"material_id": "pdmx:hp", "title": "Hedwig's Theme (Harry Potter)", "composer": "John Williams",
     "genres": "soundtrack", "rating": 4.95, "n_ratings": 500},
    {"material_id": "pdmx:rock", "title": "Smoke on the Water", "composer": "Deep Purple", "genres": "rock",
     "rating": 4.99, "n_ratings": 900},
    # название без единого читаемого слова (кракозябры PDMX): у жанра не «точное название», а по рейтингу
    {"material_id": "pdmx:mojibake", "title": "ÐÐ ÑÐ", "composer": "", "genres": "classical",
     "rating": 4.0, "n_ratings": 3},
)


@pytest.fixture(scope="module")
def archive(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("s3512") / "voice_memory.db"))


@pytest.fixture(scope="module")
def index():
    return ScoreIndex(ROWS)


def test_bach_and_mozart_finds_scores_of_both_composers_in_turn(archive, index):
    hits = theme_search(archive, "Бах и Моцарт")
    assert [p.text for p in hits.query.parts] == ["Бах", "Моцарт"]
    found = index.search(hits.query)
    assert set(found) == {"pdmx:bach1", "pdmx:bach2", "pdmx:moz1", "pdmx:moz2"}  # «Bachelor» — не Бах
    assert found[:2] in (("pdmx:bach1", "pdmx:moz1"), ("pdmx:moz1", "pdmx:bach1"))  # части по кругу


@pytest.mark.parametrize("theme, expected", [
    ("классическая музыка", ("pdmx:bach1", "pdmx:moz1", "pdmx:bach2", "pdmx:moz2", "pdmx:mojibake")),
    ("музыка из фильмов", ("pdmx:hp", "pdmx:moz2")),
])
def test_genre_theme_finds_scores_by_pdmx_genre_best_rated_first(archive, index, theme, expected):
    hits = theme_search(archive, theme)
    assert index.search(hits.query) == expected


def test_rtttl_and_scores_search_by_the_same_strings(archive, index):
    """Одна функция: строки, по которым искались мелодии, — те же, что уходят в поиск партитур."""
    hits = theme_search(archive, "Бах и Моцарт")
    assert hits.names  # мелодии RTTTL нашлись
    assert hits.query.parts == (part_query(archive, "Бах"), part_query(archive, "Моцарт"))
    assert hits.query.parts[0].terms[0].alts == ("bach",) and hits.query.parts[1].terms[0].alts == ("mozart",)


@pytest.fixture()
def seeds(tmp_path, monkeypatch):
    """Свой файл семян реестра: исключение — данные, код поиска не правится."""
    data = json.loads(works.SEEDS_FILE.read_text(encoding="utf-8"))
    data["seeds"].append({"phrase": "зюзюк*", "queries": ["bach"]})
    path = tmp_path / "seeds.json"
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    monkeypatch.setattr(works, "SEEDS_FILE", path)
    works.theme_seeds.cache_clear()
    yield
    works.theme_seeds.cache_clear()


def test_seed_data_changes_both_catalogs_without_code(archive, index, seeds):
    hits = theme_search(archive, "зюзюки")
    assert hits.names and hits.query.parts[0].terms[0].alts == ("bach",)
    assert set(index.search(hits.query)) == {"pdmx:bach1", "pdmx:bach2"}


def test_registry_link_strings_search_scores_too(index):
    """Часть, найденная связью реестра (``engine.theme_links``), ищет партитуры её проверенными строками."""
    query = ThemeQuery("путешествие по хогвардсу", (part_query(None, "путешествие по хогвардсу"),))
    assert index.search(query) == ()
    linked = query.with_links({"путешествие по хогвардсу": ("harry potter",)})
    assert index.search(linked) == ("pdmx:hp",)


def test_zero_materials_line_names_the_searched_strings(tmp_path, caplog):
    lib = SimpleNamespace(state="партитур 0", search=lambda query, limit=8: (), titles=lambda ids: {})
    dj = DjSetTool(None, _rig().owner, melodies=lambda ids: {}, scores=lib, lines=False,
                   finder=lambda theme: ThemeHits(query=ThemeQuery(theme, (part_query(None, theme),))))
    with caplog.at_level(logging.INFO):
        dj.theme_profile("Бах")
    line = next(r.getMessage() for r in caplog.records if "тема «Бах»" in r.getMessage())
    assert "материалы=[]" in line and "искали: «Бах» ['bach']" in line
