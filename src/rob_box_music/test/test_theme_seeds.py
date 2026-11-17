"""Семена связей реестра (#3493 PR-B): бывшие ``knowledge.THEME_CONCEPTS``/``RU_ALIASES`` — данные
``data/theme_link_seeds.json``, записи ``ThemeLink(source="seed")``, читает их один модуль ``works``."""

from __future__ import annotations

import json

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import works as w
from rob_box_music.theme import match_row


@pytest.fixture()
def seeds_file(tmp_path, monkeypatch):
    """Свой файл семян: ``write(seeds)`` подменяет данные, кеш загрузки сбрасывается."""
    path = tmp_path / "seeds.json"

    def write(seeds):
        path.write_text(json.dumps({"seeds": seeds}, ensure_ascii=False), encoding="utf-8")
        w.theme_seeds.cache_clear()

    monkeypatch.setattr(w, "SEEDS_FILE", path)
    yield write
    w.theme_seeds.cache_clear()


def test_seeds_are_registry_records():
    seeds = w.theme_seeds()
    assert seeds and all(s.source == "seed" and s.status == "found" and s.queries for s in seeds)
    by = {s.phrase: s for s in seeds}
    assert by["хогвар*"].queries == ("potter",) and by["хогвар*"].kind == "concept"
    assert by["денди*"].queries == ("mario", "contra", "zelda") and by["гимн ссср"].kind == "work"


def test_old_tables_are_gone_from_code():
    assert not hasattr(kn, "THEME_CONCEPTS") and not hasattr(kn, "RU_ALIASES")


def test_prefix_seed_covers_word_forms():
    assert w.word_links("хогвардсу") == ("potter",) and w.word_links("интерстеллара") == ("space",)
    assert w.word_links("тетрис") == ()  # целые слова — не начало: это замена фразы в запросе (alias_pairs)
    assert match_row("интерстеллар") == "space"


def test_adding_a_seed_as_data_changes_lookup_without_code(seeds_file):
    seeds_file([])
    assert w.word_links("пингвинов") == () and match_row("пингвины") is None
    seeds_file([{"phrase": "пингвин*", "queries": ["pingu"]}, {"phrase": "утиные истории", "queries": ["ducktales"]}])
    assert w.word_links("пингвинов") == ("pingu",)
    assert w.alias_pairs() == [("утиные истории", "ducktales")]
    assert w.ru_phrase_by_query() == {"ducktales": "утиные истории"}


@pytest.mark.parametrize("bad", [{"phrase": "два слова*", "queries": ["x"]},
                                 {"phrase": "целые слова", "queries": ["a", "b"]},
                                 {"phrase": "пусто", "queries": []}])
def test_malformed_seed_is_rejected(seeds_file, bad):
    seeds_file([bad])
    with pytest.raises(ValueError):
        w.theme_seeds()
