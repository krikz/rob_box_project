"""Issue #3136 — в DJ-режиме club-трек зациклен независимо от ``repeat`` модели."""

from unittest.mock import patch

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.core.club_arranger import render_club
    from rob_box_mcp_tools.core.club_transition import club_repeat_requested
    from rob_box_mcp_tools.tools.music import ComposeMusicTool
_ros_stubs = _ros.fixture()

from .test_music import _make_manager  # noqa: E402


def _tool(mock_node, dj):
    mgr = _make_manager(sc_running=True, renardo_available=True)
    mgr.set_dj_mode(dj)
    return ComposeMusicTool(mock_node, mgr), mgr


def test_dj_on_forces_repeat_for_club_track_without_repeat_arg(mock_node):
    tool, mgr = _tool(mock_node, True)
    with patch("builtins.exec") as fake_exec:
        result = tool.execute(style="club", bpm=124, root="C", seed=3)
    assert result.success is True, result.error
    assert fake_exec.call_args[0][0] == render_club(bpm=124, root="C", scale="minor", seed=3, repeat=True)
    assert mgr._music_form_deadline_at is None  # зацикленный: дедлайна остановки нет


def test_dj_off_keeps_model_choice(mock_node):
    tool, _ = _tool(mock_node, False)
    with patch("builtins.exec") as fake_exec:
        tool.execute(style="club", bpm=124, root="C", seed=3)
    assert fake_exec.call_args[0][0] == render_club(bpm=124, root="C", scale="minor", seed=3, repeat=False)


def test_helper_truth_table():
    class M:
        dj_mode_enabled = True

    class N:
        dj_mode_enabled = False

    assert club_repeat_requested({}, M()) is True
    assert club_repeat_requested({"repeat": True}, N()) is True
    assert club_repeat_requested({}, N()) is False
    assert club_repeat_requested({}, object()) is False
