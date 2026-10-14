"""Таблица музыкальных тулов не расходится с каталогом (ADR-0158 §7)."""

from rob_box_core import music_tools
from rob_box_core.tool_catalog import TOOL_CATALOG


def test_every_name_is_a_catalog_tool():
    names = {entry.name for entry in TOOL_CATALOG}
    assert music_tools.ALL_MUSIC_TOOL_NAMES <= names


def test_roles_keep_their_meaning():
    assert music_tools.MUSIC_LAUNCH_TOOLS == {"dj_set", "request_music", "generate_music"}
    assert music_tools.MUSIC_AUDIBLE_LYRICS_TOOLS == {"generate_music", "music_library"}
    assert music_tools.MUSIC_STOP_TOOLS == {"stop_music"}
    assert music_tools.MUSIC_CLAIM_TOOLS >= music_tools.MUSIC_LAUNCH_TOOLS | music_tools.MUSIC_STOP_TOOLS
    assert "lookup_melody" in music_tools.MUSIC_CLAIM_TOOLS
