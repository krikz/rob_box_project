"""Поток миграций SQLite - данные пакета ядра ``rob_box_core.migrations`` (ADR-0167 §2.4, шаг Ш1а)."""

from __future__ import annotations

import ast
import re
import sqlite3
import threading
from importlib import resources
from pathlib import Path

import pytest

from rob_box_core import migrations

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]


def _tables(conn):
    return {r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}


def test_stream_is_every_numbered_sql_file_in_order():
    names = sorted(f.name for f in resources.files("rob_box_core.migrations").iterdir() if f.name.endswith(".sql"))
    steps = migrations.stream()
    assert [s.name for s in steps] == [n for n in names if re.match(r"\d+_", n)]
    versions = [s.version for s in steps]
    assert versions == sorted(set(versions)), "версии потока растут и не повторяются"
    assert steps[0].name == "001_init.sql" and all(s.sql.strip() for s in steps)


def test_script_by_name_and_honest_error_for_unknown():
    assert "CREATE TABLE IF NOT EXISTS waypoints" in migrations.script("003_waypoints.sql")
    with pytest.raises(FileNotFoundError):
        migrations.script("999_nope.sql")


def test_apply_runs_the_stream_once_and_raises_user_version():
    conn = sqlite3.connect(":memory:")
    steps = migrations.stream()
    migrations.apply(conn, steps, threading.Lock())
    assert conn.execute("PRAGMA user_version").fetchone()[0] == steps[-1].version
    assert {"voice_facts", "maps", "waypoints", "faq_items"} <= _tables(conn)
    migrations.apply(conn, steps)  # второй раз - ни одного шага: ALTER из 006 не повторяется
    assert conn.execute("PRAGMA user_version").fetchone()[0] == steps[-1].version


def test_no_stream_left_at_the_checkout_root():
    """Один владелец у файла: поток живёт только в пакете (каталог ``migrations/`` в корне удалён)."""
    assert not (REPO / "migrations").exists()


def test_colcon_setup_ships_the_sql_as_package_data():
    """Прод-образ ставит пакет без --symlink-install: без ``package_data`` *.sql в install-дерево не попадут, и
    FAQStore/WaypointStore робота упадут на старте (честно, но упадут)."""
    tree = ast.parse((PKG / "setup.py").read_text(encoding="utf-8"))
    call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "setup")
    data = ast.literal_eval(next(k.value for k in call.keywords if k.arg == "package_data"))
    assert data.get("rob_box_core.migrations") == ["*.sql"]


PYPROJECT = REPO / "packaging" / "dialog-core" / "pyproject.toml"


@pytest.mark.skipif(not PYPROJECT.is_file(), reason="нет packaging/ (colcon test_ws G-Run Tests копирует только src/)")
def test_core_package_ships_the_sql_as_package_data():
    text = PYPROJECT.read_text(encoding="utf-8")
    assert '"rob_box_core.migrations" = ["*.sql"]' in text
