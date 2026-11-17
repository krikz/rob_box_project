"""test_voice_facts_marathon_inventory.py — issue #2781 cleanup CLI.

Проверяет ``scripts/maintenance/voice_facts_marathon_inventory.py`` ТОЛЬКО
на синтетическом SQLite-файле во временном каталоге — ни один тест здесь
не трогает и не может тронуть боевую ``/data/voice_memory.db`` (см.
docstring самого скрипта: инвентаризация/чистка живой базы — решение
владельца робота, не автоматики, этот PR только даёт инструмент).

Run:
  python -m pytest tests/unit/maintenance/test_voice_facts_marathon_inventory.py -q --no-cov
"""

from __future__ import annotations

import importlib.util
import sqlite3
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "maintenance" / "voice_facts_marathon_inventory.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "voice_facts_marathon_inventory", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    # dataclasses' ``@dataclass`` needs the module registered in
    # ``sys.modules`` under its own name before ``exec_module`` runs (it
    # looks itself up via ``sys.modules.get(cls.__module__)``) — otherwise
    # ``Fact`` fails to decorate with a confusing AttributeError.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def _make_db(path: Path) -> None:
    conn = sqlite3.connect(path)
    try:
        conn.execute(
            "CREATE TABLE voice_facts ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, fact TEXT NOT NULL, "
            "category TEXT NOT NULL DEFAULT 'general', speaker_id TEXT, "
            "created_at REAL NOT NULL, updated_at REAL NOT NULL)"
        )
        now = time.time()
        rows = [
            ("Саша не ест лук, даже жареный", "habit", None, now, now),
            ("Борис — друг Саши, приходит раз в неделю с пиццей", "general", None, now, now),
            ("Владелец робота пьёт кофе без сахара", "preference", "real-speaker-uuid", now, now),
            ("Денчик любит играть на гитаре", "habit", "real-speaker-uuid-2", now, now),
        ]
        conn.executemany(
            "INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
            "VALUES (?, ?, ?, ?, ?)",
            rows,
        )
        conn.commit()
    finally:
        conn.close()


@pytest.fixture()
def db_path(tmp_path: Path) -> Path:
    path = tmp_path / "voice_memory.db"
    _make_db(path)
    return path


def test_find_matches_only_matches_cast_keywords(db_path: Path):
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        matches = mod.find_matches(conn, ["Саша", "Борис"])
    finally:
        conn.close()

    assert {m.fact for m in matches} == {
        "Саша не ест лук, даже жареный",
        "Борис — друг Саши, приходит раз в неделю с пиццей",
    }


def test_find_matches_is_case_insensitive(db_path: Path):
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        matches = mod.find_matches(conn, ["саша"])
    finally:
        conn.close()
    assert len(matches) == 1


def test_find_matches_empty_keywords_matches_nothing(db_path: Path):
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        matches = mod.find_matches(conn, [])
    finally:
        conn.close()
    assert matches == []


def test_cli_list_is_always_dry_run_and_never_writes(db_path: Path, capsys):
    before = db_path.read_bytes()

    rc = mod.main(["--db", str(db_path), "list"])

    assert rc == 0
    assert db_path.read_bytes() == before, "list не должен писать в файл вообще"
    out = capsys.readouterr().out
    assert "Всего фактов" in out
    assert "Саша не ест лук" in out


def test_cli_tag_without_apply_is_dry_run(db_path: Path, capsys):
    before = db_path.read_bytes()

    rc = mod.main(["--db", str(db_path), "tag"])

    assert rc == 0
    assert db_path.read_bytes() == before, "tag без --apply --yes не должен писать в файл"
    assert "DRY-RUN" in capsys.readouterr().out

    conn = sqlite3.connect(db_path)
    try:
        categories = {r[0] for r in conn.execute("SELECT category FROM voice_facts")}
    finally:
        conn.close()
    assert mod.TAG_CATEGORY not in categories


def test_cli_tag_without_yes_is_still_dry_run_even_with_apply(db_path: Path):
    """Обе защёлки (--apply И --yes) обязаны быть выставлены — одного
    --apply недостаточно, иначе опечатка в CI/CLI могла бы применить
    изменения без явного подтверждения оператора."""
    before = db_path.read_bytes()

    rc = mod.main(["--db", str(db_path), "tag", "--apply"])

    assert rc == 0
    assert db_path.read_bytes() == before


def test_cli_tag_with_apply_and_yes_marks_matches_and_backs_up(db_path: Path, tmp_path: Path):
    rc = mod.main(["--db", str(db_path), "tag", "--apply", "--yes"])
    assert rc == 0

    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute(
            "SELECT fact, category FROM voice_facts ORDER BY id"
        ).fetchall()
    finally:
        conn.close()

    tagged = {fact for fact, category in rows if category == mod.TAG_CATEGORY}
    assert tagged == {
        "Саша не ест лук, даже жареный",
        "Борис — друг Саши, приходит раз в неделю с пиццей",
    }
    # Живые факты не тронуты.
    untouched = {fact for fact, category in rows if category != mod.TAG_CATEGORY}
    assert "Владелец робота пьёт кофе без сахара" in untouched

    backups = list(tmp_path.glob("voice_memory.db.bak-*"))
    assert backups, "tag --apply --yes обязан снять бэкап перед записью (issue #2750/#2781)"


def test_cli_delete_without_apply_yes_is_dry_run_and_keeps_all_rows(db_path: Path):
    conn = sqlite3.connect(db_path)
    try:
        before_count = conn.execute("SELECT COUNT(*) FROM voice_facts").fetchone()[0]
    finally:
        conn.close()

    rc = mod.main(["--db", str(db_path), "delete"])
    assert rc == 0

    conn = sqlite3.connect(db_path)
    try:
        after_count = conn.execute("SELECT COUNT(*) FROM voice_facts").fetchone()[0]
    finally:
        conn.close()
    assert after_count == before_count


def test_cli_delete_with_apply_and_yes_removes_only_matches_and_backs_up(
    db_path: Path, tmp_path: Path
):
    rc = mod.main(["--db", str(db_path), "delete", "--apply", "--yes"])
    assert rc == 0

    conn = sqlite3.connect(db_path)
    try:
        remaining = {r[0] for r in conn.execute("SELECT fact FROM voice_facts")}
    finally:
        conn.close()

    assert remaining == {
        "Владелец робота пьёт кофе без сахара",
        "Денчик любит играть на гитаре",
    }
    backups = list(tmp_path.glob("voice_memory.db.bak-*"))
    assert backups, "delete --apply --yes обязан снять бэкап перед удалением"


def test_missing_db_file_is_a_clean_error_not_a_traceback(tmp_path: Path):
    missing = tmp_path / "nope.db"
    rc = mod.main(["--db", str(missing), "list"])
    assert rc == 1


def test_custom_keywords_override_default_cast_names(db_path: Path, capsys):
    rc = mod.main(["--db", str(db_path), "--keyword", "Денчик", "list"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "Денчик любит играть на гитаре" in out
    assert "Саша не ест лук" not in out
