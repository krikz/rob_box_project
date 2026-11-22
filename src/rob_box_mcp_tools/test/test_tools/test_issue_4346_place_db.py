"""#4346 инкремент 1: имя места → безопасный путь базы, колонка ``maps.db_path``.

Тулы ``load_map`` / ``start_mapping`` путь ещё не читают (это инкремент 2).
Здесь только чистые функции ``map_places`` и запись/чтение колонки в
``WaypointStore``. Файлов на диске модуль не создаёт. Корень
``/maps/rtabmap.db`` — наследие одной базы (#4392), не путь места.
"""

import sqlite3

import pytest

from rob_box_core import migrations as core_migrations
from rob_box_mcp_tools.map_places import (
    LEGACY_DB_PATH,
    PlacePathError,
    is_legacy_db_path,
    is_place_db_path,
    legacy_db_path,
    place_db_path,
    place_slug,
    same_place_path,
)
from rob_box_mcp_tools.waypoint_store import WaypointStore

_DDL = core_migrations.script("003_waypoints.sql")


def _open(path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    return conn


def _columns(path: str, table: str) -> set:
    with _open(path) as conn:
        return {row["name"] for row in conn.execute(f"PRAGMA table_info({table})")}


@pytest.mark.unit
@pytest.mark.parametrize(
    "name, slug",
    [
        ("Мастерская", "masterskaja"),
        ("  Квартира  ", "kvartira"),
        ("хил-офис", "hil-ofis"),
        ("Hall 2", "hall-2"),
        ("workshop_a", "workshop_a"),
    ],
)
def test_place_slug_is_lowercase_safe_alphabet(name, slug):
    got = place_slug(name)
    assert got == slug
    assert got == got.lower()
    assert all(ch in "abcdefghijklmnopqrstuvwxyz0123456789_-" for ch in got)


@pytest.mark.unit
def test_place_slug_is_deterministic():
    assert (
        place_slug("Мастерская") == place_slug("мастерская") == place_slug("МАСТЕРСКАЯ")
    )


@pytest.mark.unit
def test_place_db_path_is_under_maps_not_the_legacy_root():
    path = place_db_path("мастерская")
    assert path == "/maps/masterskaja/rtabmap.db"
    assert path != LEGACY_DB_PATH
    assert not is_legacy_db_path(path)
    assert legacy_db_path() == LEGACY_DB_PATH == "/maps/rtabmap.db"


@pytest.mark.unit
@pytest.mark.parametrize(
    "bad",
    [
        "",
        "   ",
        "..",
        "../etc",
        "maps/../../etc",
        "a/b",
        "a\\b",
        ".",
        "..hidden",
        "???",
        "---",
    ],
)
def test_place_slug_rejects_empty_slash_and_dotdot(bad):
    with pytest.raises(PlacePathError):
        place_slug(bad)
    with pytest.raises(PlacePathError):
        place_db_path(bad)


@pytest.mark.unit
@pytest.mark.parametrize("bad", [None, 12, b"ceh", ["цех"]])
def test_place_slug_rejects_non_string(bad):
    with pytest.raises(PlacePathError):
        place_slug(bad)


@pytest.mark.unit
def test_legacy_root_is_not_a_place_path():
    """Функция места корень не возвращает. Наследие видно отдельной константой."""
    assert is_legacy_db_path(legacy_db_path())
    assert all(
        not is_legacy_db_path(place_db_path(name))
        for name in ("квартира", "офис", "default")
    )
    assert "/maps/rtabmap.db" not in {
        place_db_path("квартира"),
        place_db_path("rtabmap"),
    }


@pytest.mark.unit
def test_db_path_column_added_on_empty_db(tmp_path):
    db = str(tmp_path / "empty.db")
    store = WaypointStore(db_path=db)
    assert "db_path" in _columns(db, "maps")
    assert store.get_active_map() is None
    store.create_map("кухня")
    assert store.get_active_map()["db_path"] == "/maps/kuhnja/rtabmap.db"


@pytest.mark.unit
def test_db_path_migration_is_idempotent_on_already_migrated_db(tmp_path):
    db = str(tmp_path / "twice.db")
    WaypointStore(db_path=db)
    again = WaypointStore(db_path=db)
    again.create_map("цех")
    assert again.get_active_map()["db_path"] == place_db_path("цех")
    assert _columns(db, "maps").issuperset(
        {"map_id", "name", "created_at", "is_active", "db_path"}
    )


@pytest.mark.unit
def test_create_map_writes_path_and_get_active_map_returns_it(tmp_path):
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    store.create_map("мастерская")
    active = store.get_active_map()
    assert active["name"] == "мастерская"
    assert active["db_path"] == "/maps/masterskaja/rtabmap.db"
    listed = store.list_maps()
    assert listed[0]["db_path"] == active["db_path"]


@pytest.mark.unit
def test_create_map_keeps_explicit_path(tmp_path):
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    store.create_map("цех", db_path="/maps/ceh/rtabmap.db")
    assert store.get_active_map()["db_path"] == "/maps/ceh/rtabmap.db"


@pytest.mark.unit
def test_create_map_without_safe_name_stores_null_path(tmp_path):
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    store.create_map()
    assert store.get_active_map()["db_path"] is None
    store.create_map("../secret")
    assert store.get_active_map()["name"] == "../secret"
    assert store.get_active_map()["db_path"] is None


@pytest.mark.unit
def test_old_rows_without_path_still_read(tmp_path):
    """Строка, записанная до колонки, читается с ``db_path is None`` и не падает."""
    db = str(tmp_path / "legacy.db")
    with _open(db) as conn:
        conn.executescript(_DDL)
        conn.execute(
            "INSERT INTO maps (map_id, name, created_at, is_active) VALUES (?, ?, ?, 1)",
            ("old-id", "двор", 1.0),
        )
        conn.commit()
    assert "db_path" not in _columns(db, "maps")

    store = WaypointStore(db_path=db)
    active = store.get_active_map()
    assert active["map_id"] == "old-id"
    assert active["name"] == "двор"
    assert active["db_path"] is None
    assert store.list_maps()[0]["db_path"] is None

    store.create_map("квартира")
    rows = {row["name"]: row["db_path"] for row in store.list_maps()}
    assert rows["двор"] is None
    assert rows["квартира"] == place_db_path("квартира")


@pytest.mark.unit
def test_slug_collision_stores_null_on_the_second_map(tmp_path):
    """«хил-офис» и «hil-ofis» — один slug. Второй путь не пишем, файл не трогаем."""
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    store.create_map("хил-офис")
    first = store.get_active_map()
    assert first["db_path"] == "/maps/hil-ofis/rtabmap.db"
    assert same_place_path("хил-офис", "hil-ofis")

    store.create_map("hil-ofis")
    second = store.get_active_map()
    assert second["name"] == "hil-ofis"
    assert second["db_path"] is None
    rows = {row["name"]: row["db_path"] for row in store.list_maps()}
    assert rows["хил-офис"] == "/maps/hil-ofis/rtabmap.db"
    assert rows["hil-ofis"] is None


@pytest.mark.unit
def test_explicit_legacy_root_is_stored_as_null(tmp_path):
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    store.create_map("цех", db_path="/maps/rtabmap.db")
    assert store.get_active_map()["name"] == "цех"
    assert store.get_active_map()["db_path"] is None


@pytest.mark.unit
def test_explicit_valid_place_path_is_still_written(tmp_path):
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    store.create_map("цех", db_path="/maps/ceh/rtabmap.db")
    assert store.get_active_map()["db_path"] == "/maps/ceh/rtabmap.db"


@pytest.mark.unit
def test_rename_map_does_not_move_db_path(tmp_path):
    """Шаг 1 путь при переименовании не чинит. Чинка — шаг 2."""
    store = WaypointStore(db_path=str(tmp_path / "maps.db"))
    map_id = store.create_map("цех")
    assert store.rename_map(map_id, "хил-офис") is True
    active = store.get_active_map()
    assert active["name"] == "хил-офис"
    assert active["db_path"] == "/maps/ceh/rtabmap.db"


@pytest.mark.unit
@pytest.mark.parametrize(
    "bad",
    [
        "/maps/rtabmap.db",
        "/maps/../rtabmap.db",
        "/maps/ceh/../rtabmap.db",
        "/maps/foo/../ceh/rtabmap.db",
        "/other/ceh/rtabmap.db",
        "/maps/ceh/other.db",
        "/maps//rtabmap.db",
        "",
        None,
        12,
        b"/maps/ceh/rtabmap.db",
    ],
)
def test_is_place_db_path_rejects_root_dotdot_foreign_empty_nonstr(bad):
    assert is_place_db_path(bad) is False


@pytest.mark.unit
def test_is_place_db_path_accepts_only_a_place_file():
    assert is_place_db_path("/maps/ceh/rtabmap.db") is True
    assert is_place_db_path(place_db_path("мастерская")) is True
