"""ADR-0155 K-2: реестр произведений на настоящем архиве RTTTL; поиск библиотеки читает алиасы из реестра."""

from __future__ import annotations

import gzip
import json
from importlib.resources import files

import pytest

from rob_box_mcp_tools.core import rtttl_library as lib_mod
from rob_box_music import knowledge as kn
from rob_box_music import works as w

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def records():
    path = files("rob_box_mcp_tools.data") / "rtttl_melodies.jsonl.gz"
    with gzip.open(str(path), "rt", encoding="utf-8") as fh:
        return [json.loads(line) for line in fh if line.strip()]


@pytest.fixture(scope="module")
def built(records):
    return w.build_works(records)


def test_search_reads_aliases_from_the_registry_not_from_a_second_table():
    assert lib_mod._ALIAS_SORTED == w.alias_pairs()
    assert lib_mod._ALIAS_CANONICAL_TO_RU_PHRASE == w.ru_phrase_by_query()
    assert not hasattr(lib_mod, "_ALIASES")  # старая таблица удалена, а не продублирована
    assert lib_mod._alias_normalize("гимн ссср") == "soviet anthem"
    assert lib_mod._alias_normalize("Супер Марио") == "mario"


def test_every_archive_record_is_exactly_one_source_of_exactly_one_work(records, built):
    ids = [s.material_id for work in built for s in work.sources]
    assert len(ids) == len(set(ids)) == len(records)
    assert len({work.work_id for work in built}) == len(built)  # sha8 без коллизий на 10 461 записи


def test_category_artists_are_gone_from_works_and_became_types(built):
    categories = set(kn.CATEGORY_ARTISTS) - {""}
    assert not [x.title for x in built if x.artist.lower() in categories]
    assert not [x.title for x in built if x.title.lower() in kn.EMPTY_TITLES]
    types = {x.work_type.value for x in built if x.work_type}
    assert {"tv_theme", "film_theme", "game_theme", "anthem"} <= types
    assert types <= set(kn.TAG_WORK_TYPE.values()) | set(kn.CATEGORY_ARTISTS.values())


def test_theme_records_with_the_name_in_artist_got_a_title(records, built):
    fox = next(x for x in built if x.title == "20th Century Fox")
    assert [s.material_id for s in fox.sources] == ["rtttl:theme"] and fox.work_type.value == "tv_theme"


def test_aliases_reach_the_real_archive(built):
    tetris = [x for x in built if "тетрис" in x.aliases]
    assert tetris and all("tetris" in f"{x.title} {x.artist}".lower() for x in tetris)
    assert any(x.title == "Tetris" for x in tetris)


def test_gate_is_closed_for_every_field_today(records, built):
    working = w.working_ids(records)
    assert working
    assert {f: w.gate(built, f, working).verdict.split(":")[0] for f in w.FIELDS} == {f: "closed" for f in w.FIELDS}
