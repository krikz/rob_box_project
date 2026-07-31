"""
test_music.py - Unit тесты для инструментов управления музыкой

Тестирует:
- MusicManager: фильтрация кода, проверка SC, execute_code, stop_pattern, stop_all,
  set_vibe_preset, get_state
- ExecuteMusicCodeTool, StopMusicTool, SetVibePresetTool, GetMusicStateTool
"""

import sys
import time
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
    from rob_box_voice.core.music_stack_validation import MusicStackStatus

    mgr = MusicManager.__new__(MusicManager)
    mgr._max_amp = 0.7
    mgr._pattern_history = {}
    mgr._active_patterns = set()
    mgr._current_preset = None
    mgr._renardo_available = renardo_available
    mgr._renardo_last_error = None
    mgr._renardo_context = {}
    # issue G-MUSIC — music-stack health (must match __init__ defaults).
    # Tests that need a degraded manager should overwrite _music_stack_status
    # and/or _require_healthy directly.
    mgr._music_stack_status = MusicStackStatus(
        is_healthy=True,
        oscdef_registered=True,
        missing_synths=(),
        fatal_errors=(),
    )
    mgr._require_healthy = True
    mgr._critical_synths = MusicManager.DEFAULT_CRITICAL_SYNTHS
    # issue #935 — music session lifecycle (must match __init__ defaults
    # to keep tests faithful; see ``MusicManager.__init__``)
    mgr._auto_stop_ttl_seconds = 300
    mgr._music_session_active_since = None
    mgr._last_music_activity_at = None
    mgr._last_stop_at = None
    mgr._auto_stop_count = 0
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
        # issue #935 — music session lifecycle defaults
        mgr._auto_stop_ttl_seconds = 300
        mgr._music_session_active_since = None
        mgr._last_music_activity_at = None
        mgr._last_stop_at = None
        mgr._auto_stop_count = 0
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
        # Must mock _ensure_renardo_available — the real method calls
        # _initialize_renardo() which may succeed on machines where
        # Renardo is installed, overriding the manual False setting.
        mgr._ensure_renardo_available = Mock(return_value=False)
        result = mgr.execute_code("p1 >> pluck([0])")
        assert result["success"] is False
        assert "Renardo" in result["error"]

    def test_execute_retries_renardo_initialization_before_failing(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)

        def _recover() -> None:
            mgr._renardo_available = True
            mgr._renardo_context = {}

        mgr._initialize_renardo = Mock(side_effect=_recover)

        with patch("builtins.exec"):
            result = mgr.execute_code("Clock.bpm = 83")

        assert result["success"] is True
        mgr._initialize_renardo.assert_called_once()

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

    def test_get_state_retries_renardo_initialization_when_sc_is_running(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)

        def _recover() -> None:
            mgr._renardo_available = True
            mgr._renardo_context = {}

        mgr._initialize_renardo = Mock(side_effect=_recover)

        state = mgr.get_state()

        assert state["renardo_available"] is True
        mgr._initialize_renardo.assert_called_once()


# ---------------------------------------------------------------------------
# MusicManager — degraded sclang runtime (issue G-MUSIC)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerDegradedStack:
    """Tests for the G-MUSIC fail-fast behaviour when sclang reports
    syntax errors during startup. The acceptance criterion is: mcp_server
    must answer "music unavailable" instead of letting the LLM fight a
    broken Renardo stack.
    """

    @staticmethod
    def _mark_degraded(mgr, *, require_healthy: bool = True) -> None:
        """Simulate a degraded sclang startup log snapshot.

        Note: we leave ``_renardo_available`` alone unless the caller
        explicitly overrides it — that lets us test the degraded-guard
        independently from the renardo-init guard.
        """
        from rob_box_voice.core.music_stack_validation import MusicStackStatus

        mgr._music_stack_status = MusicStackStatus(
            is_healthy=False,
            oscdef_registered=True,
            missing_synths=("strings",),
            fatal_errors=(
                "ERROR: syntax error, unexpected '.', expecting '}'",
            ),
        )
        mgr._require_healthy = require_healthy

    def test_execute_code_is_blocked_when_stack_is_degraded(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        self._mark_degraded(mgr)

        with patch("builtins.exec") as exec_mock:
            result = mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")

        assert result["success"] is False
        assert "Музыка недоступна" in result["error"]
        assert "syntax error" in result["error"]
        # exec must not be called — we never want to talk to Renardo
        # while sclang itself is in degraded mode.
        exec_mock.assert_not_called()

    def test_set_vibe_preset_is_blocked_when_stack_is_degraded(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        self._mark_degraded(mgr)

        with patch("builtins.exec") as exec_mock:
            result = mgr.set_vibe_preset("chill")

        assert result["success"] is False
        assert "Музыка недоступна" in result["error"]
        assert "warning" in result
        exec_mock.assert_not_called()

    def test_stop_pattern_returns_unavailable_but_clears_active_set(self):
        """In degraded mode we still drop the pattern from active_patterns
        so the safety-net watchdog (issue #935) sees an empty session."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns = {"p1"}
        self._mark_degraded(mgr)

        result = mgr.stop_pattern("p1")

        assert "p1" not in mgr._active_patterns
        assert result["success"] is False
        assert "degraded" in result["error"].lower() or "Музыка недоступна" in result["error"]

    def test_stop_all_returns_unavailable_but_clears_active_set(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns = {"p1", "p2"}
        self._mark_degraded(mgr)

        result = mgr.stop_all()

        assert mgr._active_patterns == set()
        assert result["success"] is False
        assert "Музыка недоступна" in result["error"]

    def test_require_healthy_false_lets_degraded_stack_execute(self):
        """Operator opt-out: ROB_BOX_MUSIC_REQUIRE_HEALTHY=0 → degraded
        mode does NOT block the LLM from calling execute_code. This is
        useful for debugging a known-broken stack or running with a
        patched Renardo.
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        self._mark_degraded(mgr, require_healthy=False)

        with patch("builtins.exec") as exec_mock:
            result = mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")

        # Guard explicitly disabled → degraded-guard does NOT short-circuit.
        # exec runs (mocked); we only care that the error message is NOT
        # the G-MUSIC one.
        assert result["success"] is True
        assert "Музыка недоступна" not in result.get("error", "")
        exec_mock.assert_called_once()

    def test_get_state_surfaces_music_stack_health(self):
        """The LLM can read get_music_state and see the failure reason."""
        mgr = _make_manager(sc_running=False, renardo_available=False)
        self._mark_degraded(mgr)

        state = mgr.get_state()

        assert state["music_stack_healthy"] is False
        assert state["music_stack_require_healthy"] is True
        assert any("syntax error" in err for err in state["music_stack_fatal_errors"])
        assert "strings" in state["music_stack_missing_synths"]

    def test_evaluate_music_stack_health_clears_renardo_when_degraded(self, tmp_path, monkeypatch):
        """Integration: end-to-end degraded snapshot from a real log file."""
        log_path = tmp_path / "sclang.log"
        log_path.write_text(
            "\n".join([
                "FoxDot OSCdef registered. Ready to compile SynthDefs.",
                "ERROR: syntax error, unexpected '.', expecting '}'",
                "ERROR: Command line parse failed",
            ]),
            encoding="utf-8",
        )
        monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

        mgr = _make_manager(sc_running=True, renardo_available=True)
        status = mgr._evaluate_music_stack_health()

        assert status.is_healthy is False
        assert mgr._renardo_available is False
        assert not mgr.is_music_stack_healthy()

    def test_evaluate_music_stack_health_keeps_renardo_when_healthy(self, tmp_path, monkeypatch):
        # Narrow the critical-synths list so a 2-synth log counts as healthy.
        log_path = tmp_path / "sclang.log"
        log_path.write_text(
            "\n".join([
                "FoxDot OSCdef registered. Ready to compile SynthDefs.",
                "SynthDef preload ok: strings",
                "SynthDef preload ok: wobblebass",
            ]),
            encoding="utf-8",
        )
        monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._critical_synths = ("strings", "wobblebass")
        status = mgr._evaluate_music_stack_health()

        assert status.is_healthy is True
        # We don't promise renardo_available stays True — that flag is
        # the result of the heavier _initialize_renardo() flow — but
        # the degraded-mode path must not flip it to False.
        assert mgr._renardo_available is True


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


# ---------------------------------------------------------------------------
# MusicManager — music session lifecycle / safety-net (issue #935)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicSessionLifecycle:
    """Тесты session lifecycle + safety-net (issue #935):

    * ``get_state`` exposes lifecycle fields (timestamps, ttl, counter)
    * ``execute_code`` stamps ``_last_music_activity_at`` (and starts a session
      on the first pattern)
    * ``stop_pattern`` / ``stop_all`` reset / close the session correctly
    * ``auto_stop_idle_music`` no-ops when nothing is active, no-ops when the
      idle interval is below the configured TTL, and **does** call
      ``stop_all`` (incrementing the counter) once the TTL is exceeded
    * ``stop_music_on_session_end`` is idempotent and reports what was active
    """

    # ----- get_state lifecycle fields ---------------------------------------

    def test_get_state_exposes_lifecycle_fields(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)
        state = mgr.get_state()
        # default-from-__init__ values
        assert state["music_session_active_since"] is None
        assert state["last_music_activity_at"] is None
        assert state["last_stop_at"] is None
        assert state["auto_stop_ttl_seconds"] == 300
        assert state["auto_stop_count"] == 0
        # nothing is active yet → no idle window
        assert state["idle_seconds"] is None
        assert state["active_patterns"] == []

    # ----- execute_code stamps lifecycle ------------------------------------

    def test_execute_code_stamps_last_activity(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # No session yet → after execute_code, both timestamps populated.
        assert mgr._music_session_active_since is None
        assert mgr._last_music_activity_at is None
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert result["success"] is True
        assert mgr._music_session_active_since is not None
        assert mgr._last_music_activity_at is not None
        # session start == first activity (monotonic)
        assert (
            mgr._music_session_active_since <= mgr._last_music_activity_at
        )

    def test_execute_code_without_pattern_opens_session(self):
        """Any successful code execution (even without pattern_name) stamps
        lifecycle timestamps so safety nets can stop unnamed music (issue #935 fix)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("Clock.bpm = 120")
        assert result["success"] is True
        # Session IS opened — safety nets need the timestamps even for unnamed code.
        assert mgr._music_session_active_since is not None
        assert mgr._last_music_activity_at is not None
        # But _active_patterns stays empty (no pattern_name given).
        assert len(mgr._active_patterns) == 0

    def test_execute_code_continues_existing_session(self):
        """Second execute keeps session_open but bumps last_activity."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
            first_session = mgr._music_session_active_since
            first_activity = mgr._last_music_activity_at
            # Execute again — same session, but activity bumped.
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        assert mgr._music_session_active_since == first_session
        assert mgr._last_music_activity_at >= first_activity

    # ----- stop_all resets session ------------------------------------------

    def test_stop_all_resets_session(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert mgr._music_session_active_since is not None
        mgr.stop_all()
        assert mgr._music_session_active_since is None
        assert mgr._last_music_activity_at is None
        assert mgr._last_stop_at is not None

    def test_stop_pattern_partial_does_not_reset_session(self):
        """stop_pattern leaves lifecycle intact until ALL patterns are gone."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        session_before = mgr._music_session_active_since
        activity_before = mgr._last_music_activity_at
        mgr.stop_pattern("p1")
        # session & activity should still be set — p2 is still active
        assert mgr._music_session_active_since == session_before
        assert mgr._last_music_activity_at == activity_before
        assert "p1" not in mgr._active_patterns
        assert "p2" in mgr._active_patterns

    # ----- auto_stop_idle_music --------------------------------------------

    def test_auto_stop_is_noop_when_inactive(self):
        mgr = _make_manager()
        # Nothing was ever executed → _last_music_activity_at is None → no-op.
        result = mgr.auto_stop_idle_music()
        assert result["stopped"] is False
        assert result["active_patterns"] == []
        assert result["auto_stop_count"] == 0
        assert result["idle_seconds"] is None

    def test_auto_stop_works_with_unnamed_patterns(self):
        """Issue #935 regression: auto_stop must work even when the LLM
        executed music code without a pattern_name (active_patterns empty)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # Simulate: music was executed, but without pattern_name.
        mgr._last_music_activity_at = 100.0
        mgr._music_session_active_since = 100.0
        # _active_patterns is empty — the exact bug scenario.
        assert len(mgr._active_patterns) == 0
        # Idle for 400 s, TTL=300 s → should auto-stop.
        result = mgr.auto_stop_idle_music(now=500.0)
        assert result["stopped"] is True
        assert result["auto_stop_count"] == 1

    def test_stop_music_on_session_end_always_calls_stop_all(self):
        """Issue #935 regression: stop_music_on_session_end must call stop_all
        even when _active_patterns is empty (unnamed patterns)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # Simulate: music was executed without pattern_name.
        mgr._last_music_activity_at = 100.0
        mgr._music_session_active_since = 100.0
        assert len(mgr._active_patterns) == 0
        # Should still report was_active=True and call stop_all().
        result = mgr.stop_music_on_session_end()
        assert result["was_active"] is True
        # After stop_all, session is reset.
        assert mgr._music_session_active_since is None
        assert mgr._last_music_activity_at is None
        assert mgr._last_stop_at is not None

    def test_auto_stop_is_noop_when_within_ttl(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        # 10 s idle, ttl 300 s → no auto-stop.
        result = mgr.auto_stop_idle_music(ttl_seconds=300, now=time.monotonic() + 10)
        assert result["stopped"] is False
        assert mgr._auto_stop_count == 0

    def test_auto_stop_stops_when_ttl_exceeded(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert "p1" in mgr._active_patterns
        # Inject a future "now" so idle = ttl+10.
        result = mgr.auto_stop_idle_music(
            ttl_seconds=300, now=time.monotonic() + 310
        )
        assert result["stopped"] is True
        assert mgr._auto_stop_count == 1
        assert mgr._active_patterns == set()
        assert mgr._music_session_active_since is None

    def test_auto_stop_count_increments_on_repeat(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # Round 1: trigger an auto-stop, then re-arm by adding a pattern.
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        now_offset = 0.0
        t0 = time.monotonic() + now_offset
        mgr._last_music_activity_at = t0
        result1 = mgr.auto_stop_idle_music(ttl_seconds=1, now=t0 + 10)
        assert result1["stopped"] is True
        assert mgr._auto_stop_count == 1
        # Round 2: re-create a session and trigger again.
        with patch("builtins.exec"):
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        t1 = time.monotonic()
        result2 = mgr.auto_stop_idle_music(ttl_seconds=1, now=t1 + 10)
        assert result2["stopped"] is True
        assert mgr._auto_stop_count == 2

    def test_auto_stop_default_ttl_matches_env_or_300(self):
        """The default TTL constant must be 300 seconds when env unset."""
        mgr = _make_manager()
        mgr._auto_stop_ttl_seconds = 300
        assert mgr._auto_stop_ttl_seconds == 300

    def test_auto_stop_returns_diagnostic_fields(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        result = mgr.auto_stop_idle_music(
            ttl_seconds=300, now=time.monotonic() + 999
        )
        # Returning all keys makes it easy for DialogueCore to log without
        # touching internals.
        for key in (
            "stopped", "idle_seconds", "ttl_seconds",
            "active_patterns", "auto_stop_count", "stop_result",
        ):
            assert key in result, f"missing key: {key}"
        assert result["stopped"] is True
        assert isinstance(result["idle_seconds"], float)
        assert result["idle_seconds"] >= 300.0

    # ----- stop_music_on_session_end ---------------------------------------

    def test_session_end_hook_noop_when_silent(self):
        mgr = _make_manager()
        result = mgr.stop_music_on_session_end()
        assert result["was_active"] is False
        assert result["stopped_patterns"] == []
        # Always calls stop_all (prophylactic), message reflects that.
        assert "профилактически" in result["message"]

    def test_session_end_hook_stops_active_music(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        result = mgr.stop_music_on_session_end()
        assert result["was_active"] is True
        assert set(result["stopped_patterns"]) == {"p1", "p2"}
        # Message describes auto-stop (mixed RU/EN keywords).
        msg_lower = result["message"].lower()
        assert "stop_music" in msg_lower or "стоп" in msg_lower
        # Lifecycle is fully closed.
        assert mgr._active_patterns == set()
        assert mgr._music_session_active_since is None
        assert mgr._last_stop_at is not None

    def test_session_end_hook_is_idempotent(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        first = mgr.stop_music_on_session_end()
        second = mgr.stop_music_on_session_end()
        assert first["was_active"] is True
        # Second call: session already closed, but stop_all still called.
        assert second["was_active"] is False
        assert second["stopped_patterns"] == []
        assert "профилактически" in second["message"]
