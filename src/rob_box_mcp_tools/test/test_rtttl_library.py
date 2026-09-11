"""Tests for ``core.rtttl_library`` — archive migration into SQLite + SQL search."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary


def _make_archive(tmp_path: Path) -> Path:
    records = [
        {
            "name": "nokiatun",
            "title": "Nokia Tune",
            "artist": "",
            "source": "mixed3",
            "tags": ["nokia"],
            "rtttl": "NokiaTun:d=4,o=5,b=225:8e6,8d6,f#,g#,8c#6,8b,d,e,8b,8a,c#,e,2a",
        },
        {
            "name": "mario",
            "title": "Super Mario",
            "artist": "Nintendo",
            "source": "mixed3",
            "tags": ["game"],
            "rtttl": "MarioBro:d=4,o=6,b=80:16g5,32c5,16g.5",
        },
        {
            "name": "tetris",
            "title": "Tetris",
            "artist": "",
            "source": "mixed3",
            "tags": ["game"],
            "rtttl": "Tetris:d=4,o=6,b=80:8f7,16c7,16c_7",
        },
    ]
    path = tmp_path / "melodies.jsonl.gz"
    with gzip.open(path, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    return path


def _make_lib(tmp_path: Path) -> RtttlLibrary:
    archive = _make_archive(tmp_path)
    db = tmp_path / "melodies.db"
    return RtttlLibrary(db_path=str(db), archive_path=str(archive))


def test_migration_and_total(tmp_path):
    lib = _make_lib(tmp_path)
    assert lib.total() == 3


def test_get(tmp_path):
    lib = _make_lib(tmp_path)
    assert lib.get("mario")["title"] == "Super Mario"
    assert lib.get("MARIO")["title"] == "Super Mario"  # регистронезависимо
    assert lib.get("nokia")["rtttl"].startswith("NokiaTun")
    assert lib.get("nope") is None


def test_get_falls_back_to_title_substring(tmp_path):
    lib = _make_lib(tmp_path)
    # точного имени нет — совпадение по title "Super Mario".
    assert lib.get("super mario")["name"] == "mario"


def test_search_ranked_and_metadata_only(tmp_path):
    lib = _make_lib(tmp_path)
    hits = lib.search("game")
    assert [h["name"] for h in hits] == ["mario", "tetris"]
    assert "rtttl" not in hits[0]  # search отдаёт метаданные, не ноты
    assert lib.search("nintendo", limit=1)[0]["name"] == "mario"


def test_empty_query_returns_nothing(tmp_path):
    lib = _make_lib(tmp_path)
    assert lib.search("") == []
    assert lib.get("") is None


def test_migration_is_idempotent(tmp_path):
    """Повторный init с той же БД не задваивает записи."""
    archive = _make_archive(tmp_path)
    db = str(tmp_path / "melodies.db")
    RtttlLibrary(db_path=db, archive_path=str(archive))
    RtttlLibrary(db_path=db, archive_path=str(archive))
    lib = RtttlLibrary(db_path=db, archive_path=str(archive))
    assert lib.total() == 3


def test_multiword_and_russian_alias(tmp_path):
    """«гимн ссср» / «soviet anthem» / «ussr» должны находить Soviet Hymne."""
    records = [
        {
            "name": "soviethy",
            "title": "Soviet Hymne",
            "artist": "",
            "source": "mixed3",
            "tags": ["anthem"],
            "rtttl": "SovietHy:d=4,o=6,b=225:f,2a_",
        },
        {
            "name": "unknown_115",
            "title": "Unknown",
            "artist": "",
            "source": "mixed3",
            "tags": ["ussr"],
            "rtttl": "x:d=4,o=6,b=100:c",
        },
    ]
    archive = tmp_path / "alias.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "alias.db"), archive_path=str(archive))

    assert lib.get("soviet anthem")["name"] == "soviethy"  # мультислово → токены
    assert lib.get("гимн ссср")["name"] == "soviethy"      # русский alias
    assert lib.get("ussr")["name"] == "soviethy"           # аббревиатура → не мусор
    assert lib.get("imperial march") is None               # честно None, нет такой


def test_migration_repairs_unknown_title_from_artist(tmp_path):
    """title='Unknown' → artist (там реальное имя, потерянное при сборке архива)."""
    records = [
        {
            "name": "unknown_16",
            "title": "Unknown",
            "artist": "Batman V1.0",
            "source": "mixed3",
            "tags": ["movie"],
            "rtttl": "Unknown:d=4,o=5,b=100:16d#6",
        },
    ]
    archive = tmp_path / "repair.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "repair.db"), archive_path=str(archive))

    rec = lib.get("batman")
    assert rec is not None
    assert rec["title"] == "Batman V1.0"
    assert rec["rtttl_name"] == "Unknown"  # имя внутри формата RTTTL


def test_rtttl_name_is_searchable(tmp_path):
    """Поиск должен матчить и имя внутри формата RTTTL (префикс до ':')."""
    records = [
        {
            "name": "unknown_x",
            "title": "Unknown",
            "artist": "",
            "source": "mixed3",
            "tags": [],
            "rtttl": "NokiaTun:d=4,o=5,b=225:8e6,8d6",
        },
    ]
    archive = tmp_path / "rtname.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "rtname.db"), archive_path=str(archive))

    assert lib.get("nokiatun")["rtttl_name"] == "NokiaTun"
    assert lib.search("nokiatun")[0]["rtttl_name"] == "NokiaTun"


def test_multiword_query_prefers_full_title_over_name_token(tmp_path):
    """«happy birthday» → «Happy Birthday To You», а не Ashanti «Happy»."""
    records = [
        {
            "name": "happy",
            "title": "Happy",
            "artist": "Ashanti",
            "source": "mixed3",
            "tags": [],
            "rtttl": "Happy:d=4,o=5,b=100:c",
        },
        {
            "name": "happybir_3",
            "title": "Happy Birthday To You",
            "artist": "Mildred J Hill",
            "source": "mixed3",
            "tags": [],
            "rtttl": "HappyBir:d=8,o=5,b=100:16c,16c,d,c,f,e",
        },
    ]
    archive = tmp_path / "hb.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "hb.db"), archive_path=str(archive))

    rec = lib.get("happy birthday")
    assert rec is not None
    assert rec["name"] == "happybir_3"

