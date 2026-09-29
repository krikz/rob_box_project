"""Issue #3125 — ``set_music_volume``: громкость МУЗЫКИ, а не голоса.

Живой сет 28.09.2026: «играй громче» во время DJ-сета → LLM звала
``set_volume`` (``/tts_node volume_db`` — ГОЛОС), музыка не менялась:

    [mcp_server]: 📥 Запрос выполнения: set_volume с параметрами {'action': 'louder'}
    [mcp_server]: [set_volume] Громкость: -3.0 → 0.0 dB

Уровень музыки — только мастер-фейдер ``masterlimiter`` (``/n_set 999 gain``),
его и должен двигать новый тул. Менеджер — реальный ``MusicManager`` без
SuperCollider: OSC-отправка перехвачена ``patch.object(_send_osc_raw)``.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, patch

import pytest

for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.music import MusicManager, SetMusicVolumeTool  # noqa: E402
from rob_box_mcp_tools.tools.system import SetVolumeTool  # noqa: E402


def _manager(gain: float = 0.5) -> MusicManager:
    mgr = MusicManager.__new__(MusicManager)
    mgr._master_gain = gain
    return mgr


def _tool(gain: float = 0.5):
    mgr = _manager(gain)
    return SetMusicVolumeTool(None, mgr), mgr


def _sent_gains(send: MagicMock) -> list:
    return [c.args[3] for c in send.call_args_list]


class TestRelativeSteps:
    def test_louder_is_plus_3db_on_master_fader(self) -> None:
        tool, mgr = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw") as send:
            res = tool.execute(action="louder")
        assert res.success, res.error
        assert mgr.master_gain == pytest.approx(0.5 * 10 ** (3 / 20), rel=1e-6)
        send.assert_called_once()
        address, node, control, value = send.call_args.args
        assert (address, node, control) == ("/n_set", MusicManager.MASTER_LIMITER_NODE, "gain")
        assert value == pytest.approx(0.7063, abs=1e-3)
        assert res.data["old_gain"] == 0.5
        assert res.data["new_gain"] == pytest.approx(0.706, abs=1e-3)

    def test_quieter_is_minus_3db(self) -> None:
        tool, mgr = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw"):
            assert tool.execute(action="quieter").success
        assert mgr.master_gain == pytest.approx(0.5 / 10 ** (3 / 20), rel=1e-6)

    def test_louder_clamps_at_one(self) -> None:
        tool, mgr = _tool(0.9)
        with patch.object(MusicManager, "_send_osc_raw") as send:
            first = tool.execute(action="louder")
            second = tool.execute(action="louder")
        assert mgr.master_gain == 1.0
        assert _sent_gains(send) == [1.0, 1.0]
        assert first.success and second.success
        assert "не изменилась" in second.message

    def test_quieter_steps_never_mute(self) -> None:
        tool, mgr = _tool(0.06)
        with patch.object(MusicManager, "_send_osc_raw"):
            for _ in range(5):
                tool.execute(action="quieter")
        assert mgr.master_gain == pytest.approx(SetMusicVolumeTool.MIN_STEP_GAIN)


class TestAbsoluteLevels:
    def test_max(self) -> None:
        tool, mgr = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw"):
            assert tool.execute(action="max").success
        assert mgr.master_gain == 1.0

    def test_normal_returns_to_startup_gain(self) -> None:
        # normal = значение ROS-параметра music_master_gain при старте (0.5).
        tool, mgr = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw"):
            tool.execute(action="max")
            assert tool.execute(action="normal").success
        assert mgr.master_gain == 0.5

    @pytest.mark.parametrize(
        "level, expected",
        [(70, 0.7), (0, 0.0), (100, 1.0), (250, 1.0), (-10, 0.0)],
    )
    def test_set_percent_clamped(self, level, expected) -> None:
        tool, mgr = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw"):
            res = tool.execute(action="set", level=level)
        assert res.success
        assert mgr.master_gain == pytest.approx(expected)
        assert res.data["percent"] == round(expected * 100)

    def test_set_without_level_fails_honestly(self) -> None:
        tool, mgr = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw") as send:
            res = tool.execute(action="set")
        assert not res.success
        assert "level" in res.error
        send.assert_not_called()
        assert mgr.master_gain == 0.5

    def test_unknown_action_fails(self) -> None:
        tool, _ = _tool(0.5)
        with patch.object(MusicManager, "_send_osc_raw") as send:
            res = tool.execute(action="boost")
        assert not res.success
        send.assert_not_called()


class TestContract:
    def test_name_and_schema(self) -> None:
        tool, _ = _tool()
        assert tool.name == "set_music_volume"
        params = {p.name: p for p in tool.parameters}
        assert params["action"].enum == ["louder", "quieter", "max", "normal", "set"]
        assert params["action"].required is True
        assert params["level"].required is False

    def test_descriptions_distinguish_voice_and_music(self) -> None:
        tool, _ = _tool()
        assert "МУЗЫКИ" in tool.description
        assert "set_volume" in tool.description
        voice = SetVolumeTool.description.fget(None)
        assert "ГОЛОСА" in voice
        assert "НЕ музыки" in voice
        assert "set_music_volume" in voice

    def test_music_manager_master_gain_property(self) -> None:
        mgr = _manager(0.42)
        assert mgr.master_gain == 0.42
        # class-level fallback (MusicManager.__new__ без __init__)
        assert MusicManager.__new__(MusicManager).master_gain == MusicManager.DEFAULT_MASTER_GAIN


class TestCatalogAndSlices:
    """Тул доезжает до LLM: каталог, срез personality, скиллы (Move B)."""

    def test_in_generated_catalog(self) -> None:
        from rob_box_core.tool_catalog import TOOL_CATALOG

        entry = next(e for e in TOOL_CATALOG if e.name == "set_music_volume")
        assert entry.starts_music is False
        assert entry.satisfies_user_music is False
        # «громче» роутится в voice-tts (skill_router) — тул обязан быть там.
        assert {"voice-tts", "dj", "composer"} <= set(entry.skill)

    def test_slice_policy_allows_dialogue_node(self) -> None:
        from rob_box_mcp_tools.slice_authority import load_default_authority

        auth = load_default_authority()
        assert auth.is_allowed("dialogue_node", "set_music_volume").allowed
        assert auth.is_allowed("dialogue_node", "compose_music").allowed
