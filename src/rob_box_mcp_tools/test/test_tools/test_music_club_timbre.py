"""Issue #3268 — ``compose_music(style="club")``: пентатоника и тембр от модели.

Вызовы — из живого лога 01.10.2026 (voice-assistant, 10.1.1.21): славянский
сет ``lead_synth='brass', pad_synth='strings'`` получал «Проигнорировано в
club: …lead_synth, pad_synth», азиатский ``scale='minorPentatonic'`` —
«не поддержан клубным режимом».
"""

from unittest.mock import patch

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.core.club_arranger import render_club
    from rob_box_mcp_tools.tools.music import ComposeMusicTool
_ros_stubs = _ros.fixture()

from .test_music import _make_manager  # noqa: E402


def _tool(mock_node):
    mgr = _make_manager(sc_running=True, renardo_available=True)
    return ComposeMusicTool(mock_node, mgr), mgr


def test_minor_pentatonic_is_accepted_and_named(mock_node):
    tool, mgr = _tool(mock_node)
    with patch("builtins.exec") as fake_exec:
        result = tool.execute(style="club", bpm=96, root="D", scale="minorPentatonic", seed=3)
    assert result.success is True, result.error
    code = fake_exec.call_args[0][0]
    assert code == render_club(bpm=96, root="D", scale="minorPentatonic", seed=3)
    assert ", D minorPentatonic, pmin:" in code
    assert mgr.current_track_name == "клубный трек, 96 BPM, ре минорная пентатоника"


def test_lead_and_pad_from_the_call_play_or_are_refused_with_reason(mock_node):
    """Живой вызов славянского сета: brass — применён, strings — честный отказ."""
    tool, _ = _tool(mock_node)
    with patch("builtins.exec") as fake_exec:
        result = tool.execute(
            style="club", bpm=132, root="C#", scale="major", seed=4220803,
            lead_synth="brass", pad_synth="strings", bass_synth="retrobass", form="buildup",
        )
    assert result.success is True, result.error
    code = fake_exec.call_args[0][0]
    kit = result.data["club_kit"]
    assert kit["lead"] == "brass" and "p1 >> brass(" in code
    assert kit["pad"] in ("sinepad", "warmpad", "space") and f"p3 >> {kit['pad']}(" in code
    assert result.data["club_timbre"]["applied"] == {"lead": "brass"}
    assert result.data["ignored_params"] == ["bass_synth", "form"]
    msg = result.message
    assert msg.startswith("⚠️ pad_synth='strings' не применён: strings в club слишком тихий пэд")
    assert "Проигнорировано в club (трек звучит БЕЗ них): bass_synth, form." in msg
    assert "Тембр из вызова: lead brass" in msg
    assert f"тембры brass/{kit['bass']}/{kit['pad']}" in msg


def test_without_timbre_call_code_is_unchanged(mock_node):
    tool, _ = _tool(mock_node)
    with patch("builtins.exec") as fake_exec:
        result = tool.execute(style="club", root="C", seed=3)
    assert result.success is True, result.error
    assert fake_exec.call_args[0][0] == render_club(bpm=124, root="C", scale="minor", seed=3)
    assert result.data["club_timbre"] == {"applied": {}, "refused": []}
    assert "Тембр из вызова" not in result.message


def test_unknown_scale_is_still_an_honest_error(mock_node):
    tool, _ = _tool(mock_node)
    with patch("builtins.exec"):
        result = tool.execute(style="club", scale="harmonicMinor", seed=3)
    assert result.success is False
    assert "harmonicMinor" in result.error and "minorPentatonic" in result.error
