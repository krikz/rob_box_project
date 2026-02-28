"""
test_music.py - Unit тесты для инструментов управления музыкой

Тестирует:
- MusicManager: фильтрация кода, проверка SC, execute_code, stop_pattern, stop_all,
  set_vibe_preset, get_state
- ExecuteMusicCodeTool, StopMusicTool, SetVibePresetTool, GetMusicStateTool
"""

import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 modules before importing anything from rob_box_mcp_tools
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

from rob_box_mcp_tools.tools.music import (  # noqa: E402
    MusicManager,
    ExecuteMusicCodeTool,
    StopMusicTool,
    SetVibePresetTool,
    GetMusicStateTool,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_manager(*, sc_running: bool = False, renardo_available: bool = False) -> MusicManager:
    """Create a MusicManager with patched infrastructure."""
    mgr = MusicManager.__new__(MusicManager)
    mgr._max_amp = 0.7
    mgr._pattern_history = {}
    mgr._active_patterns = set()
    mgr._current_preset = None
    mgr._renardo_available = renardo_available
    mgr._renardo_context = {}
    mgr._check_supercollider = Mock(return_value=sc_running)
    return mgr


# ---------------------------------------------------------------------------
# MusicManager — code safety filter
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerFilter:
    """Тесты фильтра безопасности кода."""

    def setup_method(self):
        self.mgr = _make_manager()

    def test_safe_renardo_code_passes(self):
        ok, err = self.mgr._filter_code("p1 >> pluck([0, 2, 4], dur=0.5)")
        assert ok is True
        assert err == ""

    def test_import_is_blocked(self):
        ok, err = self.mgr._filter_code("import os")
        assert ok is False
        assert "import" in err

    def test_os_is_blocked(self):
        ok, err = self.mgr._filter_code("os.system('rm -rf /')")
        assert ok is False
        assert "os" in err

    def test_subprocess_is_blocked(self):
        ok, err = self.mgr._filter_code("subprocess.run(['ls'])")
        assert ok is False
        assert "subprocess" in err

    def test_open_is_blocked(self):
        ok, err = self.mgr._filter_code("open('/etc/passwd').read()")
        assert ok is False
        assert "open" in err

    def test_eval_is_blocked(self):
        ok, err = self.mgr._filter_code("eval('1+1')")
        assert ok is False
        assert "eval" in err

    def test_exec_is_blocked(self):
        ok, err = self.mgr._filter_code("exec('print(1)')")
        assert ok is False
        assert "exec" in err

    def test_dunder_import_is_blocked(self):
        ok, err = self.mgr._filter_code("__import__('os')")
        assert ok is False
        assert "__import__" in err

    def test_builtins_is_blocked(self):
        ok, err = self.mgr._filter_code("__builtins__['eval']('1')")
        assert ok is False

    def test_setattr_is_allowed(self):
        """setattr must pass — needed for Clock.future() BPM/Scale changes."""
        ok, err = self.mgr._filter_code("Clock.future(8, lambda: setattr(Clock, 'bpm', 170))")
        assert ok is True
        assert err == ""

    def test_getattr_is_allowed(self):
        """getattr must pass — needed for pattern introspection."""
        ok, err = self.mgr._filter_code("getattr(p1, 'degree')")
        assert ok is True
        assert err == ""

    def test_clock_future_setattr_scale_passes(self):
        """Clock.future with setattr for Scale/Root changes must pass."""
        ok, err = self.mgr._filter_code(
            "Clock.future(16, lambda: setattr(Scale, 'default', 'minor'))\n"
            "Clock.future(16, lambda: setattr(Root, 'default', 4))"
        )
        assert ok is True
        assert err == ""

    def test_scale_code_passes(self):
        ok, err = self.mgr._filter_code("Scale.default = Scale.major\nClock.bpm = 120")
        assert ok is True

    def test_clock_clear_passes(self):
        ok, err = self.mgr._filter_code("Clock.clear()")
        assert ok is True


# ---------------------------------------------------------------------------
# MusicManager — SuperCollider check
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerSCCheck:
    """Тесты проверки SuperCollider."""

    def _make_raw_manager(self):
        mgr = MusicManager.__new__(MusicManager)
        mgr._pattern_history = {}
        mgr._active_patterns = set()
        mgr._current_preset = None
        mgr._renardo_available = False
        mgr._renardo_context = {}
        return mgr

    def test_sc_running_returns_true(self):
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.return_value = (b"\x00" * 16, ("127.0.0.1", 57110))
            result = mgr._check_supercollider()
        assert result is True

    def test_sc_not_running_returns_false(self):
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.side_effect = OSError("timeout")
            result = mgr._check_supercollider()
        assert result is False

    def test_sc_check_sends_osc_status_message(self):
        """Verify the correct OSC /status message is sent to scsynth."""
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.return_value = (b"\x00" * 16, ("127.0.0.1", 57110))
            mgr._check_supercollider()
        # Проверяем что отправлен правильный OSC /status
        call_args = mock_sock.sendto.call_args
        assert call_args is not None
        sent_data, (host, port) = call_args[0]
        assert sent_data == b"/status\x00,\x00\x00\x00"
        assert port == 57110

    def test_sc_check_uses_udp_socket(self):
        """Verify UDP socket (SOCK_DGRAM) is used, not TCP."""
        import socket as socket_module

        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.side_effect = OSError
            mgr._check_supercollider()
        mock_sock_class.assert_called_once_with(socket_module.AF_INET, socket_module.SOCK_DGRAM)


# ---------------------------------------------------------------------------
# MusicManager — execute_code
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerExecuteCode:
    """Тесты выполнения кода."""

    def test_execute_fails_if_sc_not_running(self):
        mgr = _make_manager(sc_running=False, renardo_available=True)
        result = mgr.execute_code("p1 >> pluck([0])")
        assert result["success"] is False
        assert "SuperCollider" in result["error"]

    def test_execute_fails_if_renardo_not_available(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)
        result = mgr.execute_code("p1 >> pluck([0])")
        assert result["success"] is False
        assert "Renardo" in result["error"]

    def test_execute_fails_if_code_is_dangerous(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        result = mgr.execute_code("import os; os.system('ls')")
        assert result["success"] is False
        assert "Запрещённый" in result["error"]

    def test_execute_success_stores_pattern_in_history(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0, 2])", pattern_name="p1")
        assert result["success"] is True
        assert "p1" in mgr._pattern_history
        assert "p1" in mgr._active_patterns

    def test_execute_success_without_pattern_name(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("Clock.bpm = 120")
        assert result["success"] is True
        assert len(mgr._pattern_history) == 0

    def test_execute_returns_code_in_result(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        code = "p2 >> bass([0, -2])"
        with patch("builtins.exec"):
            result = mgr.execute_code(code, pattern_name="p2")
        assert result["code"] == code

    def test_execute_runtime_error_returns_failure(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec", side_effect=RuntimeError("boom")):
            result = mgr.execute_code("p1 >> bad()")
        assert result["success"] is False
        assert "boom" in result["error"]


# ---------------------------------------------------------------------------
# MusicManager — stop_pattern / stop_all
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerStop:
    """Тесты остановки паттернов."""

    def test_stop_unknown_pattern_succeeds_without_sc(self):
        """stop_pattern always succeeds (discards from active set) even for unknown patterns."""
        mgr = _make_manager()
        result = mgr.stop_pattern("unknown")
        assert result["success"] is True

    def test_stop_known_pattern_removes_from_active(self):
        mgr = _make_manager(sc_running=False, renardo_available=False)
        mgr._pattern_history["p1"] = "p1 >> pluck([])"
        mgr._active_patterns.add("p1")
        result = mgr.stop_pattern("p1")
        assert result["success"] is True
        assert "p1" not in mgr._active_patterns

    def test_stop_pattern_with_sc_and_renardo(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._pattern_history["p1"] = "p1 >> pluck([])"
        mgr._active_patterns.add("p1")
        with patch("builtins.exec"):
            result = mgr.stop_pattern("p1")
        assert result["success"] is True
        assert "p1" not in mgr._active_patterns

    def test_stop_all_clears_active_patterns(self):
        mgr = _make_manager(sc_running=False, renardo_available=False)
        mgr._active_patterns = {"p1", "p2", "p3"}
        result = mgr.stop_all()
        assert result["success"] is True
        assert len(mgr._active_patterns) == 0

    def test_stop_all_with_sc_and_renardo(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns = {"p1", "p2"}
        with patch("builtins.exec"):
            result = mgr.stop_all()
        assert result["success"] is True
        assert len(mgr._active_patterns) == 0


# ---------------------------------------------------------------------------
# MusicManager — set_vibe_preset
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerPreset:
    """Тесты вайб-пресетов."""

    @pytest.mark.parametrize("preset", list(MusicManager.VIBE_PRESETS.keys()))
    def test_set_valid_preset_without_sc(self, preset):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset(preset)
        assert result["success"] is True
        assert mgr._current_preset == preset
        assert result["preset"] == MusicManager.VIBE_PRESETS[preset]

    def test_set_invalid_preset_fails(self):
        mgr = _make_manager()
        result = mgr.set_vibe_preset("nonexistent")
        assert result["success"] is False
        assert "nonexistent" in result["error"]

    def test_set_preset_with_sc_applies_renardo_code(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec") as mock_exec:
            result = mgr.set_vibe_preset("chill")
        assert result["success"] is True
        mock_exec.assert_called_once()
        executed_code = mock_exec.call_args[0][0]
        assert "Scale.default" in executed_code
        assert "Clock.bpm" in executed_code

    def test_set_preset_without_sc_remembers_name(self):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset("energetic")
        assert result["success"] is True
        assert mgr._current_preset == "energetic"

    def test_chill_preset_values(self):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset("chill")
        assert result["preset"]["scale"] == "major"
        assert result["preset"]["bpm"] == 85
        assert result["preset"]["root"] == 0  # C = 0 semitones from C

    def test_dark_preset_values(self):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset("dark")
        assert result["preset"]["scale"] == "phrygian"
        assert result["preset"]["bpm"] == 100
        assert result["preset"]["root"] == 4  # E = 4 semitones from C

    @pytest.mark.parametrize("preset", ["rock", "latin", "electronic", "cinematic", "funk", "reggae", "classical"])
    def test_new_presets_execute_successfully(self, preset):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset(preset)
        assert result["success"] is True
        assert mgr._current_preset == preset
        assert "scale" in result["preset"]
        assert "bpm" in result["preset"]
        assert "root" in result["preset"]


# ---------------------------------------------------------------------------
# MusicManager — get_state
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerGetState:
    """Тесты получения состояния."""

    def test_get_state_returns_all_fields(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._pattern_history = {"p1": "code1"}
        mgr._active_patterns = {"p1"}
        mgr._current_preset = "chill"
        state = mgr.get_state()
        assert state["renardo_available"] is True
        assert state["supercollider_running"] is True
        assert state["current_preset"] == "chill"
        assert "p1" in state["pattern_history"]
        assert "p1" in state["active_patterns"]

    def test_initial_state_is_empty(self):
        mgr = _make_manager()
        state = mgr.get_state()
        assert state["current_preset"] is None
        assert state["pattern_history"] == {}
        assert state["active_patterns"] == []


# ---------------------------------------------------------------------------
# ExecuteMusicCodeTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestExecuteMusicCodeTool:
    """Тесты MCPTool для выполнения кода."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return ExecuteMusicCodeTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "execute_music_code"

    def test_tool_description_mentions_renardo(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert "Renardo" in tool.description

    def test_tool_has_code_parameter(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        param_names = [p.name for p in tool.parameters]
        assert "code" in param_names

    def test_tool_has_pattern_name_parameter(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        param_names = [p.name for p in tool.parameters]
        assert "pattern_name" in param_names

    def test_execute_success(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = tool.execute(code="p1 >> pluck([0])", pattern_name="p1")
        assert result.success is True

    def test_execute_failure_propagates(self, mock_node):
        tool, _ = self._make_tool(mock_node, sc_running=False)
        result = tool.execute(code="p1 >> pluck([0])")
        assert result.success is False
        assert result.error is not None

    def test_execute_blocked_code(self, mock_node):
        tool, _ = self._make_tool(mock_node, sc_running=True, renardo_available=True)
        result = tool.execute(code="import os")
        assert result.success is False


# ---------------------------------------------------------------------------
# StopMusicTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestStopMusicTool:
    """Тесты MCPTool для остановки музыки."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return StopMusicTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "stop_music"

    def test_stop_all_when_no_pattern_name(self, mock_node):
        tool, mgr = self._make_tool(mock_node)
        mgr._active_patterns = {"p1", "p2"}
        result = tool.execute()
        assert result.success is True
        assert len(mgr._active_patterns) == 0

    def test_stop_all_when_pattern_name_is_all(self, mock_node):
        tool, mgr = self._make_tool(mock_node)
        mgr._active_patterns = {"p1"}
        result = tool.execute(pattern_name="all")
        assert result.success is True
        assert len(mgr._active_patterns) == 0

    def test_stop_specific_pattern(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=False, renardo_available=False)
        mgr._pattern_history["p1"] = "code"
        mgr._active_patterns.add("p1")
        result = tool.execute(pattern_name="p1")
        assert result.success is True
        assert "p1" not in mgr._active_patterns

    def test_stop_unknown_pattern_succeeds(self, mock_node):
        """stop_pattern always succeeds — LLM can stop any player name."""
        tool, _ = self._make_tool(mock_node)
        result = tool.execute(pattern_name="nonexistent")
        assert result.success is True


# ---------------------------------------------------------------------------
# SetVibePresetTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSetVibePresetTool:
    """Тесты MCPTool для вайб-пресетов."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return SetVibePresetTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "set_vibe_preset"

    def test_preset_name_enum_contains_all_presets(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        param = next(p for p in tool.parameters if p.name == "preset_name")
        for preset_key in MusicManager.VIBE_PRESETS:
            assert preset_key in param.enum

    def test_execute_valid_preset(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=False)
        result = tool.execute(preset_name="chill")
        assert result.success is True

    def test_execute_invalid_preset(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        result = tool.execute(preset_name="nonexistent")
        assert result.success is False

    @pytest.mark.parametrize("preset", list(MusicManager.VIBE_PRESETS.keys()))
    def test_all_presets_execute_successfully(self, mock_node, preset):
        tool, _ = self._make_tool(mock_node, sc_running=False)
        result = tool.execute(preset_name=preset)
        assert result.success is True


# ---------------------------------------------------------------------------
# GetMusicStateTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestGetMusicStateTool:
    """Тесты MCPTool для получения состояния музыки."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return GetMusicStateTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "get_music_state"

    def test_tool_has_no_required_parameters(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert len(tool.parameters) == 0

    def test_tool_is_read_only(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.read_only is True

    def test_execute_returns_state(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=False, renardo_available=False)
        mgr._pattern_history = {"bass": "code"}
        mgr._active_patterns = {"bass"}
        mgr._current_preset = "jazz"
        result = tool.execute()
        assert result.success is True
        assert result.data["current_preset"] == "jazz"
        assert "bass" in result.data["pattern_history"]

    def test_execute_message_contains_status(self, mock_node):
        tool, _ = self._make_tool(mock_node, sc_running=False)
        result = tool.execute()
        assert result.success is True
        assert "SuperCollider" in result.message
        assert "Renardo" in result.message
