"""Хранилище связей «фраза темы → строки поиска архива» (#3493, ``works.theme_links``) и схема ответа LLM
(``theme_queries``): одно место исключений с источником, сроком и счётчиком; журнал непонятого."""

from __future__ import annotations

import sqlite3
from datetime import datetime, timedelta

import pytest

from rob_box_music import theme_queries as tq
from rob_box_music.works import (LOOKUP_TTL_DAYS, ThemeLink, get_theme_link, main, put_theme_link, theme_links_report,
                                 theme_phrase)

NOW = datetime(2026, 10, 7, 15, 0)
PARTS = ("танец маленьких утят", "зебра")


@pytest.fixture()
def conn():
    return sqlite3.connect(":memory:")


def _llm(phrase="танец маленьких утят", queries=("Chicken Dance",)):
    return ThemeLink(theme_phrase(phrase), "found", "llm", "work", queries, ("chickend",), ("Vogeltanz",))


def test_phrase_key_ignores_case_punctuation_and_yo():
    assert theme_phrase("  Танец маленьких УТЯТ!") == "танец маленьких утят"
    assert theme_phrase("Ёжик") == theme_phrase("ежик")


def test_link_is_read_back_and_counts_hits(conn):
    put_theme_link(conn, _llm(), NOW)
    link = get_theme_link(conn, "Танец маленьких утят", NOW)
    assert link.queries == ("Chicken Dance",) and link.rejected == ("Vogeltanz",) and link.source == "llm"
    get_theme_link(conn, "танец маленьких утят", NOW)
    assert get_theme_link(conn, "танец маленьких утят", NOW).hits == 2  # два прошлых срабатывания


def test_llm_link_expires_but_manual_and_seed_do_not(conn):
    put_theme_link(conn, _llm(), NOW)
    put_theme_link(conn, ThemeLink("зебра", "not_theme", "manual", "not_theme"), NOW)
    later = NOW + timedelta(days=LOOKUP_TTL_DAYS + 1)
    assert get_theme_link(conn, "танец маленьких утят", later) is None
    assert get_theme_link(conn, "зебра", later).status == "not_theme"


def test_stale_rules_version_is_asked_again(conn):
    put_theme_link(conn, ThemeLink("танец утят", "found", "llm", "work", ("Chicken Dance",), rules_version="old"), NOW)
    assert get_theme_link(conn, "танец утят", NOW) is None


def test_miss_piles_up_without_hiding_a_verdict(conn):
    for _ in range(3):
        put_theme_link(conn, ThemeLink("кукушка", "missed", "miss"), NOW)
    assert get_theme_link(conn, "кукушка", NOW) is None  # непонятое не мешает спросить LLM
    put_theme_link(conn, _llm(), NOW)
    put_theme_link(conn, ThemeLink(_llm().phrase, "missed", "miss"), NOW)
    assert get_theme_link(conn, "танец маленьких утят", NOW).status == "found"
    report = theme_links_report(conn, 7, NOW)
    assert "3  missed    «кукушка»" in report and "llm    found     «танец маленьких утят» → Chicken Dance" in report


def test_found_without_queries_is_rejected():
    with pytest.raises(ValueError):
        ThemeLink("x", "found", "llm")
    with pytest.raises(ValueError):
        ThemeLink("x", "found", "wiki", queries=("a",))


def test_manual_link_from_cli(tmp_path, capsys):
    db = str(tmp_path / "v.db")
    assert main(["--db", db, "--add-theme-link", "Танец утят", "chicken dance"]) == 0
    assert "manual found     «танец утят» → chicken dance" in capsys.readouterr().out
    link = get_theme_link(sqlite3.connect(db), "танец утят", datetime.now() + timedelta(seconds=1))
    assert link.source == "manual" and link.queries == ("chicken dance",)


def test_llm_answer_schema_is_narrow():
    schema = tq.schema(PARTS)["properties"]["suggestions"]["items"]["properties"]
    assert schema["part"]["enum"] == list(PARTS) and schema["kind"]["enum"] == list(tq.KINDS)
    assert schema["queries"]["maxItems"] == tq.MAX_QUERIES


def test_validator_cleans_queries_and_rejects_foreign_parts():
    payload = {"suggestions": [{"part": PARTS[0], "kind": "work",
                                "queries": ["Chicken  Dance", "Chicken Dance", "", "x" * 99, *map(str, range(20))]}]}
    (s,) = tq.validate(payload, PARTS)
    assert s.queries[0] == "Chicken Dance" and len(s.queries) == tq.MAX_QUERIES and "" not in s.queries
    for bad in ({"suggestions": [{"part": "тетрис", "kind": "work", "queries": []}]},
                {"suggestions": [{"part": PARTS[0], "kind": "hook", "queries": []}]},
                {"suggestions": "Chicken Dance"}, None):
        with pytest.raises(tq.QueriesInvalid):
            tq.validate(bad, PARTS)


def test_prompt_names_every_part():
    system, user = tq.prompt("звери", PARTS)
    assert tq.SUBMIT_TOOL in system and all(p in user for p in PARTS)
