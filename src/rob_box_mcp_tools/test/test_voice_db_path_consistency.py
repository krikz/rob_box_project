"""Один источник пути БД инструментов (#4082): хранилища ``mcp_server`` открывают ``rob_box_core.tools_db``.

Раньше умолчание ``/data/...db`` было вписано в каждом хранилище (``WaypointStore``, ``FAQStore``, ``VoiceMemory``
у ``mcp_server``, ``RtttlLibrary``, ``MusicHistory``, параметры ``speaker_id_node``/``dialog_node``), а этот тест
сверял литералы регулярным выражением. Теперь путь знает один модуль. Тест проверяет поведение: хранилища без
явного пути пишут в файл, который даёт ``tools_db_path()``; и что в коде пакетов не появилось второго места с
именем файла или чтения env в обход модуля (критерий карточки #4082).

Почему не ``harness_voice.db`` - докстринг ``rob_box_core.tools_db`` (схемы ``waypoints``/``faq_items`` там
другие); это проверяет ``rob_box_core/test/test_tools_db.py``.
"""

from __future__ import annotations

import gzip
import json
import sqlite3
from pathlib import Path

import pytest

from rob_box_core import tools_db
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.waypoint_store import WaypointStore

REPO = Path(__file__).resolve().parents[3]
MODULE = REPO / "src" / "rob_box_core" / "rob_box_core" / "tools_db.py"


@pytest.fixture()
def tools_file(monkeypatch, tmp_path):
    path = tmp_path / "tools.db"
    monkeypatch.setenv(tools_db.TOOLS_DB_ENV, str(path))
    return path


def _rows(path: Path, sql: str) -> list:
    conn = sqlite3.connect(path)
    try:
        return conn.execute(sql).fetchall()
    finally:
        conn.close()


def test_waypoint_store_without_path_writes_to_the_tools_db(tools_file):
    store = WaypointStore()
    try:
        assert store.save_waypoint("кухня", 1.0, 2.0, 0.0)
    finally:
        store.close()
    assert _rows(tools_file, "SELECT name FROM waypoints") == [("кухня",)]


def test_rtttl_library_without_path_lands_in_the_tools_db(tools_file, tmp_path):
    archive = tmp_path / "melodies.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        fh.write(
            json.dumps(
                {
                    "name": "nokiatun",
                    "title": "Nokia Tune",
                    "artist": "",
                    "source": "t",
                    "tags": [],
                    "rtttl": "NokiaTun:d=4,o=5,b=225:8e6,8d6",
                }
            )
            + "\n"
        )
    RtttlLibrary(archive_path=str(archive))
    assert _rows(tools_file, "SELECT name FROM rtttl_melodies") == [("nokiatun",)]


def _package_sources():
    for path in sorted((REPO / "src").rglob("*.py")):
        rel = path.relative_to(REPO).as_posix()
        if "/test/" in rel or "/tests/" in rel or path == MODULE:
            continue
        yield rel, path.read_text(encoding="utf-8", errors="replace")


def test_no_second_place_knows_the_file_name_or_reads_the_env():
    """Имя файла и чтение env - только в ``rob_box_core/tools_db.py``; остальные зовут ``tools_db_path()``."""
    file_name = tools_db.TOOLS_DB_DEFAULT.rsplit("/", 1)[1]
    e2e_name = tools_db.TOOLS_E2E_DB_DEFAULT.rsplit("/", 1)[1]
    leaks = [
        rel
        for rel, text in _package_sources()
        if file_name in text or e2e_name in text or f'"{tools_db.TOOLS_DB_ENV}"' in text
    ]
    assert leaks == []
