"""compose_music / lookup_melody отдают партитуру и анализ (ADR-0132, PR-1).

* ``compose_music`` отдаёт модели ОДИН раз компактный текст партитуры —
  ``data["score"]`` (строка); структурный dict — ``tool.last_score`` и лог,
  в ответ модели не идёт (ADR-0132 PR-4, бюджет контекста);
  ``data["alternatives"]`` (#2896) остаётся.
* Предупреждения санитайзера (``execute_code`` → ``warnings``) больше не
  теряются: попадают в партитуру и в сообщение.
* ``lookup_melody`` отдаёт блок анализа: тональность+альтернативы,
  диапазон, такты, плотность.
"""

from __future__ import annotations

import json
import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

for _mod in [
    "rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs",
    "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg", "nav2_msgs",
    "nav2_msgs.action", "action_msgs", "action_msgs.srv", "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.music import (  # noqa: E402
    ComposeMusicTool,
    LookupMelodyTool,
)

_ARR = dict(lead_synth="blip", bass_synth="moogbass", pad_synth="strings")


@pytest.fixture(scope="module")
def rtttl_library(tmp_path_factory):
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary

    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("score") / "lib.db"))


def _compose_tool(mock_node, rtttl_library, exec_result=None):
    mgr = Mock()
    mgr.execute_code = Mock(return_value=exec_result or {"success": True})
    return ComposeMusicTool(mock_node, mgr, rtttl_library), mgr


def test_compose_by_name_returns_score_and_keeps_alternatives(mock_node, rtttl_library):
    tool, _mgr = _compose_tool(mock_node, rtttl_library)
    result = tool.execute(name="national anthem of russia", **_ARR)
    assert result.success is True
    score = tool.last_score
    assert score["title"] == result.data["title"]
    assert len(score["chords"]) == score["theme"]["bars"]
    assert {"lead", "bass", "pad"} <= set(score["parts"])
    assert "density" in score["decisions"]
    assert score["text"] and result.data["score"] == score["text"]
    assert "Партитура" in result.data["score"]
    assert "alternatives" in result.data  # #2896 сосуществует с партитурой
    json.dumps(score, ensure_ascii=False)  # уходит в лог JSON-ом


def test_sanitizer_warnings_are_forwarded(mock_node, rtttl_library):
    tool, _mgr = _compose_tool(
        mock_node, rtttl_library,
        {"success": True, "message": "Код выполнен успешно. ⚠️ W1",
         "code": "d1 >> play('x')", "warnings": ["W1 предупреждение санитайзера"]},
    )
    result = tool.execute(name="tetris", **_ARR)
    assert result.success is True
    assert "W1 предупреждение санитайзера" in tool.last_score["warnings"]
    assert "W1 предупреждение санитайзера" in result.data["score"]


def test_long_release_lead_warning_is_visible_in_score(mock_node, rtttl_library):
    """ADR-0132 PR-6: вместо тихого safety net — предупреждение в партитуре."""
    tool, _mgr = _compose_tool(mock_node, rtttl_library)
    result = tool.execute(
        name="national anthem of russia", lead_synth="imperialbrass",
        bass_synth="moogbass", pad_synth="strings",
    )
    score = tool.last_score
    assert not any("safety net" in w for w in score["warnings"])
    assert not score["decisions"]["counter"].startswith("auto→off")
    tail = [w for w in score["warnings"] if "долгий релиз" in w]
    voices = ("counter" in score["parts"]) + bool(score["parts"]["lead"]["octave_doubled"])
    assert bool(tail) == (voices == 2)
    if tail:
        assert "counter=off или theme_octaves=off" in tail[0]
        assert "долгий релиз" in result.data["score"]


def test_explicit_root_scale_with_name_reach_the_score(mock_node, rtttl_library):
    """ADR-0132 PR-2: root/scale при name= больше не игнорируются —
    аккомпанемент построен в заданной тональности, партитура это называет."""
    tool, _mgr = _compose_tool(mock_node, rtttl_library)
    tool.execute(name="tetris", root="F#", scale="major", **_ARR)
    score = tool.last_score
    assert (score["key"]["root"], score["key"]["scale"]) == ("F#", "major")
    assert score["key"]["source"] == "задана вызовом"
    assert score["decisions"]["key"].startswith("explicit→F# major (auto ")
    assert not any("расходятся" in w for w in score["warnings"])


def test_composed_track_without_name_has_score(mock_node, rtttl_library):
    tool, _mgr = _compose_tool(mock_node, rtttl_library)
    result = tool.execute(
        bpm=110, root="A", scale="minor", drums="X...o...",
        lead_synth="pluck", lead_notes="0, 2, 4, 7", pad_synth="warmpad",
        pad_notes="0, 2, 4", bass_synth="dub", bass_notes="0, 4",
    )
    assert result.success is True
    assert tool.last_score["theme"] is None
    assert result.data["score"] == tool.last_score["text"]


def test_score_failure_does_not_break_playing_track(mock_node, rtttl_library):
    tool, _mgr = _compose_tool(mock_node, rtttl_library)
    with patch(
        "rob_box_mcp_tools.tools.music.describe", side_effect=RuntimeError("boom")
    ):
        result = tool.execute(name="tetris", **_ARR)
    assert result.success is True
    assert "boom" in tool.last_score["error"]
    assert result.data["score"].startswith("Партитура не собрана") and "boom" in result.data["score"]


def test_lookup_melody_returns_analysis(mock_node, rtttl_library):
    tool = LookupMelodyTool(mock_node, Mock(), Mock(), rtttl_library)
    result = tool.execute("national anthem of russia")
    assert result.success is True
    analysis = result.data["analysis"]
    for field in ("root", "scale", "key_gap", "key_alternatives", "bars", "lo", "hi", "density"):
        assert field in analysis, field
    assert analysis["text"] in result.message
    assert "alternatives" in result.data


def test_lookup_melody_bad_rtttl_is_honest(mock_node):
    rtttl_library = Mock()
    rtttl_library.get.return_value = {"name": "x", "title": "X", "rtttl": "garbage"}
    rtttl_library.search.return_value = []
    tool = LookupMelodyTool(mock_node, Mock(), Mock(), rtttl_library)
    result = tool.execute("x")
    assert result.success is True
    assert "error" in result.data["analysis"]
    assert "Анализ недоступен" in result.message


def test_execute_code_exposes_sanitizer_warnings():
    """``execute_code`` отдаёт мягкие предупреждения отдельным полем."""
    from .test_music import _make_manager

    mgr = _make_manager(sc_running=True, renardo_available=True)
    with patch("builtins.exec"):
        result = mgr.execute_code("p1 >> pluck([0,2,4])")
    assert result["success"] is True
    assert result["warnings"]
    assert all(w in result["message"] for w in result["warnings"])
