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

