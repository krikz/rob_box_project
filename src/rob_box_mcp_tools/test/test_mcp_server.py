"""Unit tests for MCP server startup behavior."""

import importlib.util
import json
import os
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


class _FakePublisher:
    """Captures every ``.publish(msg)`` call — no real rclpy publisher."""

    def __init__(self):
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg.data)


class _FakeRegistry:
    def __init__(self):
        self.tools = []

    def register(self, tool):
        self.tools.append(tool.name)


class _FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []
        self.warning_messages = []
        self.debug_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)

    def debug(self, message):
        self.debug_messages.append(message)


class _FakeParameter:
    def __init__(self, value):
        self.value = value


class _FakeServer:
    def __init__(self):
        self.registry = _FakeRegistry()
        self.waypoint_store = object()
        self.mapping_state = object()
        self._logger = _FakeLogger()
        self.stop_generated_track_playback_calls = 0

    def get_parameter(self, name):
        # ``_register_music_tools`` reads both — the second (music_master_gain)
        # was added after this fixture was first written (same drift as the
        # tool-import fallback above).
        assert name in ("music_max_amp", "music_master_gain")
        return _FakeParameter(0.7)

    def get_logger(self):
        return self._logger

    def get_current_pose_snapshot(self):
        return None

    def stop_generated_track_playback(self):
        self.stop_generated_track_playback_calls += 1

    def _arm_form_end_timer(self, remaining_s):
        # Issue #3133 — записываем, на что взводился бы таймер конца формы.
        self.__dict__.setdefault("armed_form_end", []).append(remaining_s)


def _make_tool_class(tool_name):
    class _Tool:
        def __init__(self, *args, **kwargs):
            self.name = tool_name

    return _Tool


def _install_fake_mcp_server_dependencies(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")
    rclpy_qos = types.ModuleType("rclpy.qos")
    # Issue #2781 — mcp_server.py gained ``from rcl_interfaces.msg import
    # SetParametersResult`` for the voice_memory e2e_mode parameters
    # callback (same pattern speaker_id_node already uses for its own
    # e2e_mode). Real ``rcl_interfaces`` needs a live ROS2 install, so it
    # gets the same module-stub treatment as ``rclpy``/``std_msgs`` here.
    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    registry_module = types.ModuleType("rob_box_mcp_tools.registry")
    tools_module = types.ModuleType("rob_box_mcp_tools.tools")
    waypoint_store_module = types.ModuleType("rob_box_mcp_tools.waypoint_store")
    mapping_state_module = types.ModuleType("rob_box_mcp_tools.mapping_state")

    class Node:
        pass

    class SetParametersResult:
        def __init__(self, successful: bool = True, reason: str = ""):
            self.successful = successful
            self.reason = reason

    class ReentrantCallbackGroup:
        pass

    class QoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    class ReliabilityPolicy:
        RELIABLE = 1

    class HistoryPolicy:
        KEEP_LAST = 1

    class DurabilityPolicy:
        # Issue #1812: mcp_server.py's ``publish_tools`` QoS (line ~203)
        # gained a ``durability=DurabilityPolicy.TRANSIENT_LOCAL`` at some
        # point after this fake ``rclpy.qos`` stub was written, so every
        # test using ``_load_mcp_server_module`` started failing at import
        # time with ``ImportError: cannot import name 'DurabilityPolicy'``
        # — an infra gap, unrelated to any one feature, that happened to
        # surface again with the watchdog tests added here.
        TRANSIENT_LOCAL = 1

    class String:
        def __init__(self):
            self.data = ""

    class MCPToolRegistry:
        pass

    class WaypointStore:
        pass

    class MappingState:
        pass

    tool_names = {
        "NavigateToWaypointTool": "navigate_to_waypoint",
        "NavigateToCoordinatesTool": "navigate_to_coordinates",
        "MoveDirectionTool": "move_direction",
        "StopNavigationTool": "stop_navigation",
        "ListWaypointsTool": "list_waypoints",
        "SaveWaypointTool": "save_waypoint",
        "DeleteWaypointTool": "delete_waypoint",
        "ClearWaypointsTool": "clear_waypoints",
        "GetCurrentPoseTool": "get_current_pose",
        "SetVolumeTool": "set_volume",
        "SetPitchTool": "set_pitch",
        "SetSpeedTool": "set_speed",
        "GetRobotStatusTool": "get_robot_status",
        "GetCurrentTimeTool": "get_current_time",
        "GetPerceptionContextTool": "get_perception_context",
        "GetBatteryLevelTool": "get_battery_level",
        "StartMappingTool": "start_mapping",
        "ContinueMappingTool": "continue_mapping",
        "FinishMappingTool": "finish_mapping",
        "OptimizeMapTool": "optimize_map",
        "LoadMapTool": "load_map",
        "PlayAnimationTool": "play_animation",
        "PlaySoundTool": "play_sound",
        "GetSoundInfoTool": "get_sound_info",
        "SpeakTextTool": "speak_text",
        "ListenForResponseTool": "listen_for_response",
        "EstimateTtsDurationTool": "estimate_tts_duration",
        "RegisterSpeakerTool": "register_speaker",
        "SetVoiceTool": "set_voice",
        "MemorySaveTool": "memory_save",
        "MemorySearchTool": "memory_search",
        "MemoryContextTool": "memory_context",
        "ExecuteMusicCodeTool": "execute_music_code",
        "StopMusicTool": "stop_music",
        "GetMusicStateTool": "get_music_state",
        "FaqSearchTool": "faq_search",
        "SearchWebTool": "search_web",
    }

    for class_name, tool_name in tool_names.items():
        setattr(tools_module, class_name, _make_tool_class(tool_name))

    def _tools_module_fallback(name):
        """Issue #1812: ``mcp_server.py`` has grown far more tool imports
        (compose_music, generate_music, set_tts_provider, task_delta, ...)
        than this fixture's hand-curated ``tool_names`` map tracks, so the
        whole module failed to import — one missing name at a time — every
        time a new tool was added upstream. PEP 562 module ``__getattr__``:
        anything not explicitly listed above gets a generic stub instead of
        an ``ImportError``; only tests that actually care about a tool's
        identity/behaviour need to list it in ``tool_names``.

        Must raise ``AttributeError`` (not synthesize a stub) for dunder
        names like ``__path__``/``__all__`` — the import machinery probes
        those to decide whether this is a package, and handing back a
        class instead of ``None``/a list breaks it with a confusing
        ``TypeError: 'type' object is not iterable``.
        """
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)
        return _make_tool_class(name)

    tools_module.__getattr__ = _tools_module_fallback

    class MusicManager:
        def __init__(self, *args, **kwargs):
            pass

    class TrackLibrary:
        def __init__(self, *args, **kwargs):
            raise FileNotFoundError("missing 004_music_library.sql")

    tools_module.MusicManager = MusicManager
    tools_module.TrackLibrary = TrackLibrary

    rclpy_node.Node = Node
    rclpy_callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy
    rclpy_qos.HistoryPolicy = HistoryPolicy
    rclpy_qos.DurabilityPolicy = DurabilityPolicy
    rcl_interfaces_msg.SetParametersResult = SetParametersResult
    std_msgs_msg.String = String
    registry_module.MCPToolRegistry = MCPToolRegistry
    waypoint_store_module.WaypointStore = WaypointStore
    mapping_state_module.MappingState = MappingState

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", rclpy_callback_groups)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_qos)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", rcl_interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", rcl_interfaces_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.registry", registry_module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.tools", tools_module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.waypoint_store", waypoint_store_module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.mapping_state", mapping_state_module)


def _load_mcp_server_module(monkeypatch):
    _install_fake_mcp_server_dependencies(monkeypatch)
    module_path = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools" / "mcp_server.py"
    spec = importlib.util.spec_from_file_location("rob_box_mcp_tools.mcp_server", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.unit
def test_register_tools_skips_track_library_failures_without_crashing(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server._register_music_tools = lambda: module.MCPServer._register_music_tools(server)

    module.MCPServer._register_tools(server)

    assert "start_mapping" in server.registry.tools
    assert "execute_music_code" in server.registry.tools
    assert "lookup_melody" not in server.registry.tools  # нужна библиотека треков
    assert any("Music library disabled" in msg for msg in server.get_logger().error_messages)


@pytest.mark.unit
def test_recommended_executor_threads_never_returns_less_than_two(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)

    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: {0}, raising=False)

    assert module._recommended_executor_threads() == 2


@pytest.mark.unit
def test_recommended_executor_threads_uses_affinity_when_available(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)

    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: {0, 1, 2, 3}, raising=False)

    assert module._recommended_executor_threads() == 4


# ---------------------------------------------------------------------------
# Issue #1229 — /voice/tts/provider_state (фактический провайдер TTS)
# ---------------------------------------------------------------------------


@pytest.mark.unit
def test_tts_provider_state_updates_actual_provider(monkeypatch):
    """tts_node публикует фактического провайдера после фолбека → mcp_server
    запоминает его для валидации голосов в speak_text/set_voice."""
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server.actual_tts_provider = None

    msg = module.String()
    msg.data = '{"provider": "yandex", "voice": "anton", "reason": "provider_dead"}'
    module.MCPServer._on_tts_provider_state(server, msg)

    assert server.actual_tts_provider == "yandex"


@pytest.mark.unit
def test_tts_provider_state_ignores_empty_payload(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server.actual_tts_provider = None

    msg = module.String()
    msg.data = "not-json"
    module.MCPServer._on_tts_provider_state(server, msg)
    assert server.actual_tts_provider is None

    msg2 = module.String()
    msg2.data = ""
    module.MCPServer._on_tts_provider_state(server, msg2)
    assert server.actual_tts_provider is None


# ---------------------------------------------------------------------------
# Issue #1812 — music watchdog idle-TTL parameter + form-end protection
# ---------------------------------------------------------------------------


@pytest.mark.unit
def test_music_watchdog_idle_ttl_defaults_to_1800(monkeypatch):
    """300s ("слушаю музыку" читалось как простой) → 1800s (30 min).

    Same env-var-backed pattern as ``_music_watchdog_period_s`` /
    ``_music_watchdog_enabled``: read once at __init__ time, default when
    unset.
    """
    monkeypatch.delenv("MUSIC_WATCHDOG_IDLE_TTL_S", raising=False)
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    # Exercise only the parsing snippet that __init__ runs — constructing
    # the full Node is out of scope for this fake-module harness (see
    # test_register_tools_* above for the same pattern with music_max_amp).
    try:
        server._music_watchdog_idle_ttl_s = float(
            module.os.environ.get("MUSIC_WATCHDOG_IDLE_TTL_S", "1800.0")
        )
    except (TypeError, ValueError):
        server._music_watchdog_idle_ttl_s = 1800.0
    assert server._music_watchdog_idle_ttl_s == 1800.0


@pytest.mark.unit
def test_music_watchdog_idle_ttl_honors_env_override(monkeypatch):
    monkeypatch.setenv("MUSIC_WATCHDOG_IDLE_TTL_S", "42")
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    try:
        server._music_watchdog_idle_ttl_s = float(
            module.os.environ.get("MUSIC_WATCHDOG_IDLE_TTL_S", "1800.0")
        )
    except (TypeError, ValueError):
        server._music_watchdog_idle_ttl_s = 1800.0
    assert server._music_watchdog_idle_ttl_s == 42.0


@pytest.mark.unit
def test_run_music_watchdog_passes_the_configured_ttl_to_the_manager(monkeypatch):
    """The watchdog timer callback must forward its own TTL explicitly —
    it must not rely on whatever default MusicManager picked up."""
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server._music_watchdog_idle_ttl_s = 1800.0
    manager = MagicMock()
    manager.auto_stop_idle_music.return_value = {"stopped": False}
    server._music_manager = manager

    module.MCPServer._run_music_watchdog(server)

    manager.auto_stop_idle_music.assert_called_once_with(ttl_seconds=1800.0)


@pytest.mark.unit
def test_run_music_watchdog_logs_stop_with_reason_as_before(monkeypatch):
    """Regression guard: the stop_reason logging (#935/#990) must keep working."""
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server._music_watchdog_idle_ttl_s = 1800.0
    manager = MagicMock()
    manager.auto_stop_idle_music.return_value = {
        "stopped": True,
        "active_patterns": ["p1"],
        "idle_seconds": 1801.0,
        "ttl_seconds": 1800.0,
        "stop_reason": "idle_ttl",
    }
    server._music_manager = manager

    module.MCPServer._run_music_watchdog(server)

    assert server.stop_generated_track_playback_calls == 1
    assert any(
        "reason=idle_ttl" in m for m in server.get_logger().warning_messages
    )


@pytest.mark.unit
def test_run_music_watchdog_survives_a_manager_exception(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server._music_watchdog_idle_ttl_s = 1800.0
    manager = MagicMock()
    manager.auto_stop_idle_music.side_effect = RuntimeError("boom")
    server._music_manager = manager

    module.MCPServer._run_music_watchdog(server)

    assert any(
        "Music watchdog failed" in m for m in server.get_logger().warning_messages
    )
    assert server.stop_generated_track_playback_calls == 0


# ── Issue #3174 / ADR-0149 — music_cleanup и дека движка ──────────────────


class _Deck:
    """Владелец деки: играет / не играет, помнит причину стопа."""

    def __init__(self, playing):
        self.playing = playing
        self.stopped = []

    def is_playing(self):
        return self.playing

    def stop(self, reason="user_stop"):
        self.stopped.append(reason)
        self.playing = False
        return {"ok": True, "track_id": "t1"}


class _DjSet:
    def __init__(self):
        self.closed = []
        self.running = False

    def close_set(self, reason):
        self.closed.append(reason)


def _cleanup_server(module, *, deck_playing):
    manager = MagicMock()
    manager.stop_music_on_session_end.return_value = {
        "was_active": True, "stopped_patterns": [], "message": "ok",
    }
    server = _FakeServer()
    server._music_manager = manager
    server._player_owner = _Deck(deck_playing)
    server._dj_set_tool = _DjSet()
    server._cleanup_spares_deck = lambda reason: module.MCPServer._cleanup_spares_deck(server, reason)
    server._stop_deck = lambda reason: module.MCPServer._stop_deck(server, reason)
    return server, manager


def _send_cleanup(module, server, reason):
    msg = module.String()
    msg.data = json.dumps({"reason": reason})
    module.MCPServer._on_music_cleanup(server, msg)


@pytest.mark.unit
@pytest.mark.parametrize("reason", ["tts_batch_complete", "dialogue_end", "new_dialogue"])
def test_soft_cleanup_spares_playing_deck(monkeypatch, reason):
    """Живой прогон 29.09 05:00: сет роутера, ход без тулов → тишина 38 с."""
    module = _load_mcp_server_module(monkeypatch)
    server, manager = _cleanup_server(module, deck_playing=True)

    _send_cleanup(module, server, reason)

    manager.stop_music_on_session_end.assert_not_called()
    assert server._player_owner.stopped == [] and server._dj_set_tool.closed == []
    assert server.stop_generated_track_playback_calls == 0


@pytest.mark.unit
@pytest.mark.parametrize(
    "reason",
    ["user_stop_command", "stop_command_guard", "new_session", "shutdown"],
)
def test_explicit_cleanup_stops_deck_through_its_owner(monkeypatch, reason):
    """Жёсткий стоп закрывает сет и снимает деку у владельца, а не за его спиной."""
    module = _load_mcp_server_module(monkeypatch)
    server, manager = _cleanup_server(module, deck_playing=True)

    _send_cleanup(module, server, reason)

    assert server._dj_set_tool.closed == [reason]
    assert server._player_owner.stopped == [reason]
    manager.stop_music_on_session_end.assert_called_once()
    assert server.stop_generated_track_playback_calls == 1


@pytest.mark.unit
def test_soft_cleanup_spares_a_running_set_even_if_the_deck_looks_idle(monkeypatch):
    """06.10 15:30 UTC: ход ``tools=[]`` → ``tts_batch_complete`` погасил идущий сет. Идёт ли сет — знает его
    владелец ``DjSetTool.running``, а не только отметка ``started`` деки."""
    module = _load_mcp_server_module(monkeypatch)
    server, manager = _cleanup_server(module, deck_playing=False)
    server._dj_set_tool.running = True

    _send_cleanup(module, server, "tts_batch_complete")

    manager.stop_music_on_session_end.assert_not_called()
    assert server._dj_set_tool.closed == [] and server._player_owner.stopped == []


@pytest.mark.unit
def test_soft_cleanup_with_idle_deck_still_stops_leftovers(monkeypatch):
    """Дека молчит — мягкий cleanup гасит то, что могло остаться (mp3, код харнесса)."""
    module = _load_mcp_server_module(monkeypatch)
    server, manager = _cleanup_server(module, deck_playing=False)

    _send_cleanup(module, server, "tts_batch_complete")

    assert server._player_owner.stopped == []
    manager.stop_music_on_session_end.assert_called_once()
    assert server.stop_generated_track_playback_calls == 1
