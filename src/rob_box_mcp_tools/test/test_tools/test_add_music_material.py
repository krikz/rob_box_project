"""Тул add_music_material (issue #3227): библиотека, дубликат, честная ошибка, мелодия в каталоге."""

import gzip
from pathlib import Path

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
    from rob_box_mcp_tools.tools.music_material import AddMusicMaterialTool
_ros_stubs = _ros.fixture()

from rob_box_mcp_tools.core.music_material import _hook_like  # noqa: E402
from rob_box_mcp_tools.core.rtttl import parse_rtttl  # noqa: E402

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


def test_material_lands_in_the_catalog_as_a_hook_like_melody(mock_node, library):
    """Присланный материал — мелодия каталога по имени (её играет ``request_music``), начало годится как хук."""
    added = AddMusicMaterialTool(mock_node, library).execute(text=FIXTURE.read_text(encoding="utf-8"))
    rec = library.get(added.data["name"])
    _title, bpm, notes = parse_rtttl(rec["rtttl"])
    assert bpm == 83 and _hook_like(notes)
