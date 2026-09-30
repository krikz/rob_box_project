"""Тул add_music_material (issue #3227): библиотека, дубликат, честная ошибка, путь до club-хука и истории."""

import gzip
from pathlib import Path
from unittest.mock import patch

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.core.music_diversity import MusicHistory
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
    from rob_box_mcp_tools.tools.music import ComposeMusicTool
    from rob_box_mcp_tools.tools.music_material import AddMusicMaterialTool
_ros_stubs = _ros.fixture()

from .test_music import _make_manager  # noqa: E402

FIXTURE = Path(__file__).parent.parent / "fixtures" / "strudel_stranger_things.txt"


@pytest.fixture
def library(tmp_path):
    archive = tmp_path / "empty.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        fh.write("")
    return RtttlLibrary(db_path=str(tmp_path / "lib.db"), archive_path=str(archive))


def test_add_then_duplicate_then_garbage(mock_node, library):
    tool = AddMusicMaterialTool(mock_node, library)
    text = FIXTURE.read_text(encoding="utf-8")
    first = tool.execute(text=text)
    assert first.success is True and first.data["added"] is True
    assert first.data["bpm"] == 83 and first.data["notes"] == 16 and first.data["source_format"] == "strudel"
    assert "16 нот, 83 bpm" in first.message and first.data["name"] in first.message
    rec = library.get(first.data["name"])
    assert rec["name"] == first.data["name"] and rec["source"] == "user"
    assert rec["tags"] == ["user", "dj-material"] and rec["title"] == "Stranger Things Intro Theme"

    again = tool.execute(text=text)  # тот же материал: имя то же, записи не дублируются
    assert again.success is True and again.data["added"] is False and again.data["name"] == first.data["name"]
    assert "уже был" in again.message

    bad = tool.execute(text="вот тебе для материала: привет как дела")
    assert bad.success is False and "не распознал" in bad.error and library.total() == 1


def test_material_flows_into_club_hook_and_music_history(mock_node, library):
    added = AddMusicMaterialTool(mock_node, library).execute(text=FIXTURE.read_text(encoding="utf-8"))
    name = added.data["name"]
    history = MusicHistory(":memory:")
    tool = ComposeMusicTool(
        mock_node, _make_manager(sc_running=True, renardo_available=True),
        rtttl_library=library, music_history=history,
    )
    with patch("builtins.exec"):
        result = tool.execute(style="club", name=name, bpm=124, root="A#", scale="minor", seed=6261504)
    assert result.success is True, result.error
    info = result.data["club_hook"]
    assert info is not None and info["id"] == name  # хук — из присланного материала, не пентатоника
    row = history.recent()[0]
    assert row["melody_name"] == name and row["hook_fingerprint"]
