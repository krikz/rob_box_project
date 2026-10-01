"""issue #3264 — русское название ищется транслитом, без словаря под песни."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.rtttl_library import (
    RtttlLibrary,
    human_track_title,
    match_info,
)
from rob_box_mcp_tools.core.translit_ru import strip_version_tail, transliterate_ru

_RTTTL = "X:d=4,o=5,b={bpm}:c,d,e,f,g,a,b,c6,d6,e6,f6,g6"

_RECORDS = [
    ("kalinkav", "Kalinka V1.0"),
    ("kalinkav_2", "Kalinka V2.0"),
    ("we_rock_47", "polka"),
    ("polkka", "polkka"),
    ("tetris", "Tetris"),
    ("mario", "Super Mario Brothers 1"),
    ("stranger_2", "Strangers In The Night"),
    ("polaris", "Polaris Theme"),
    ("kalimba", "Kalimba Tune"),
    ("imperial", "Star Wars - Imperial March 1"),
]


@pytest.fixture()
def lib(tmp_path: Path) -> RtttlLibrary:
    archive = tmp_path / "m.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for i, (name, title) in enumerate(_RECORDS):
            fh.write(json.dumps({
                "name": name, "title": title, "artist": "", "source": "t",
                "tags": [], "rtttl": _RTTTL.format(bpm=60 + i),
            }) + "\n")
    return RtttlLibrary(db_path=str(tmp_path / "m.db"), archive_path=str(archive))


def test_transliterate_basic():
    assert transliterate_ru("калинка") == "kalinka"
    assert transliterate_ru("полька") == "polka"
    assert transliterate_ru("щёлк super") == "schyolk super"


def test_strip_version_tail():
    assert strip_version_tail("Kalinka V1.0") == "Kalinka"
    assert strip_version_tail("kalinka_v2") == "kalinka"
    assert strip_version_tail("Version Two") == "Version Two"


@pytest.mark.parametrize("query", ["Калинка", "калинка", "Калинка v1.0", "Калинка-малинка"])
def test_kalinka_found_by_russian_name(lib, query):
    rec = lib.get(query)
    assert rec is not None
    assert rec["name"].startswith("kalinkav")
    assert {r["name"] for r in lib.search(query)} == {"kalinkav", "kalinkav_2"}


def test_polka_found_by_russian_name(lib):
    rec = lib.get("Полька")
    assert rec is not None and rec["title"] == "polka"


def test_unknown_russian_title_is_honest_not_found(lib):
    assert lib.get("Катюша") is None
    assert lib.search("Катюша") == []


def test_russian_query_does_not_match_unrelated(lib):
    names = {r["name"] for r in lib.search("Калинка")}
    assert "kalimba" not in names and "polaris" not in names


def test_existing_aliases_and_regressions_unchanged(lib):
    assert lib.get("тетрис")["name"] == "tetris"
    assert lib.get("super mario")["name"] == "mario"
    assert lib.get("imperial march")["name"] == "imperial"
    # #2877: токены запроса не вылечиваются транслитом — решает match_info
    rec = lib.get("stranger things")
    info = match_info(lib, rec, "stranger things") if rec else None
    assert info is None or "things" in info["unmatched"]


def test_match_info_for_russian_query_has_no_ignored_words(lib):
    rec = lib.get("Калинка")
    info = match_info(lib, rec, "Калинка")
    assert info["ignored"] == []
    assert info["coverage"] == 1.0


def test_human_track_title_strips_version(lib):
    rec = lib.get("Калинка")
    assert human_track_title(lib, rec).startswith("Kalinka")
    assert "V" not in human_track_title(lib, rec).replace("Kalinka", "")
