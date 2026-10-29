"""#4082: путь к БД инструментов голоса знает один модуль - ``rob_box_core.tools_db``."""

from __future__ import annotations

import pytest

from rob_box_core import tools_db


def test_default_path_without_env(monkeypatch):
    monkeypatch.delenv(tools_db.TOOLS_DB_ENV, raising=False)
    assert tools_db.tools_db_path() == tools_db.TOOLS_DB_DEFAULT


def test_env_overrides_the_default(monkeypatch, tmp_path):
    monkeypatch.setenv(tools_db.TOOLS_DB_ENV, str(tmp_path / "tools.db"))
    assert tools_db.tools_db_path() == str(tmp_path / "tools.db")


@pytest.mark.parametrize("blank", ["", "   "])
def test_blank_env_falls_back_to_the_default_not_to_cwd(monkeypatch, blank):
    """Пустой env раньше доходил до ``WaypointStore(db_path="")`` - SQLite открыл бы файл в текущем каталоге."""
    monkeypatch.setenv(tools_db.TOOLS_DB_ENV, blank)
    assert tools_db.tools_db_path() == tools_db.TOOLS_DB_DEFAULT


def test_tools_db_is_not_the_harness_profiles_db():
    """``waypoints``/``faq_items`` у ``SQLiteVoiceMemory`` в ``harness_voice.db`` с другой схемой (ADR-0055 §1.3):
    указать БД инструментов туда - вставки вейпоинтов и FAQ упадут."""
    for path in (tools_db.TOOLS_DB_DEFAULT, tools_db.TOOLS_E2E_DB_DEFAULT):
        assert "harness_voice" not in path


def test_e2e_file_is_a_different_file_on_the_same_volume():
    """Вход в e2e стирает e2e-файл (``mcp_server._wipe_e2e_facts_db_file``): совпади он с боевым - стёрлись бы
    факты живых людей."""
    assert tools_db.TOOLS_E2E_DB_DEFAULT != tools_db.TOOLS_DB_DEFAULT
    assert (
        tools_db.TOOLS_E2E_DB_DEFAULT.rsplit("/", 1)[0]
        == tools_db.TOOLS_DB_DEFAULT.rsplit("/", 1)[0]
    )
