"""#3399 (ADR-0149 §3.3, I16): ``engine.search.find`` по словам человека — на настоящем архиве RTTTL.

Живой прогон 05.10: сет «вечеринка по темам терминатора» играл axelf/popcorn/tetris — ``get('терминатора')`` → None,
``search('вечеринка по темам терминатора')`` → мусор по «по/темам». Тест проверяет исход поиска (запись, доля
покрытых слов, ``found``), а не текст промпта.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.classic import find_record
from rob_box_mcp_tools.engine.search import find, sound_key, stem

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("search") / "voice_memory.db"))


def _names(found):
    return {found.record["name"], *(r["name"] for r in found.alternatives)}


@pytest.mark.parametrize("text", ["вечеринка по темам терминатора", "давай тему терминатора", "терминатора",
                                  "Терминатор"])
def test_terminator_by_case_form_and_party_words(library, text):
    found = find(library, text)
    assert found.found and found.confidence == 1.0
    assert found.record["name"] == "terminat"
    assert {"theme_177", "theme_178"} <= _names(found)


@pytest.mark.parametrize("text", ["тему инспектор гаджет", "инспектор гаджет", "Inspector Gadget"])
def test_inspector_gadget_by_transliteration(library, text):
    found = find(library, text)
    assert found.found and found.confidence == 1.0
    assert all("inspector gadget" in f"{r['title']} {r['artist']}".lower()
               for r in (found.record, *found.alternatives))


@pytest.mark.parametrize("text,expected", [
    ("пираты", {"pirateso", "theme_141"}),
    ("космос", {"spacecha", "spaceque"}),
    ("денди", {"zelda", "contra", "supermar_4"}),
    ("калинку", {"kalinkav_2"}),
])
def test_concepts_and_plural_find_meaningful_melodies(library, text, expected):
    found = find(library, text)
    assert found.found and expected & _names(found), (text, _names(found) if found.record else None)


@pytest.mark.parametrize("text", ["Angine de Poitrine", "привет как дела", "бухгалтерский отчёт", "новый год",
                                  "охотники за привидениями", "вечеринка по темам", "ты диджей", ""])
def test_garbage_and_unknown_are_honest_misses(library, text):
    found = find(library, text)
    assert not found.found and found.record is None and found.alternatives == ()


def test_known_queries_keep_the_record_get_finds(library):
    """Запрос, который ``RtttlLibrary.get`` уже покрывает, даёт ту же запись (A15: не хуже v1)."""
    for text in ("super mario", "russian anthem", "гимн ссср", "имперский марш", "калинка", "tetris"):
        found = find(library, text)
        assert found.found and found.query == text and found.record["name"] == library.get(text)["name"], text


def test_classic_order_by_case_form_plays_the_melody(library):
    record, reason = find_record(library, "терминатора")
    assert record is not None and record["name"] == "terminat", reason
    assert find_record(library, "гимн германии")[0] is None  # «германии» не та запись — честный промах


def test_stem_and_sound_key():
    assert stem("терминатора") == "терминатор" and stem("темам") == "тем" and stem("tetris") == "tetris"
    assert sound_key("gadzhet") == sound_key("gadget") and sound_key("inspektor") == sound_key("inspector")
