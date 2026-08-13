"""Unit tests for MCP server startup behavior."""

import importlib.util
import os
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


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

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)


class _FakeParameter:
    def __init__(self, value):
        self.value = value


class _FakeServer:
    def __init__(self):
        self.registry = _FakeRegistry()
        self.waypoint_store = object()
        self.tf_buffer = object()
        self.mapping_state = object()
        self._logger = _FakeLogger()

    def get_parameter(self, name):
        assert name == "music_max_amp"
        return _FakeParameter(0.7)

    def get_logger(self):
        return self._logger


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
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    registry_module = types.ModuleType("rob_box_mcp_tools.registry")
    tools_module = types.ModuleType("rob_box_mcp_tools.tools")
    waypoint_store_module = types.ModuleType("rob_box_mcp_tools.waypoint_store")
    mapping_state_module = types.ModuleType("rob_box_mcp_tools.mapping_state")

    class Node:
        pass

    class ReentrantCallbackGroup:
        pass

    class QoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    class ReliabilityPolicy:
        RELIABLE = 1

    class HistoryPolicy:
        KEEP_LAST = 1

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
        "MemorySaveTool": "memory_save",
        "MemorySearchTool": "memory_search",
        "MemoryContextTool": "memory_context",
        "ExecuteMusicCodeTool": "execute_music_code",
        "StopMusicTool": "stop_music",
        "SetVibePresetTool": "set_vibe_preset",
        "GetMusicStateTool": "get_music_state",
        "SaveTrackTool": "save_track",
        "ListTracksTool": "list_tracks",
        "LoadTrackTool": "load_track",
        "DeleteTrackTool": "delete_track",
        "SetDjModeTool": "set_dj_mode",
        "SearchSamplesTool": "search_samples",
        "FaqSearchTool": "faq_search",
        "SearchWebTool": "search_web",
    }

    for class_name, tool_name in tool_names.items():
        setattr(tools_module, class_name, _make_tool_class(tool_name))

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
    std_msgs_msg.String = String
    registry_module.MCPToolRegistry = MCPToolRegistry
    waypoint_store_module.WaypointStore = WaypointStore
    mapping_state_module.MappingState = MappingState

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", rclpy_callback_groups)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_qos)
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
    assert "set_dj_mode" in server.registry.tools
    assert "save_track" not in server.registry.tools
    assert "list_tracks" not in server.registry.tools
    assert any("Music library disabled" in msg for msg in server.get_logger().error_messages)


@pytest.mark.unit
def test_recommended_executor_threads_never_returns_less_than_two(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)

    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: {0})

    assert module._recommended_executor_threads() == 2


@pytest.mark.unit
def test_recommended_executor_threads_uses_affinity_when_available(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)

    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: {0, 1, 2, 3})

    assert module._recommended_executor_threads() == 4


# ---------------------------------------------------------------------------
# Issue #1016 — empty-response music fallback (/mcp/music_fallback)
# ---------------------------------------------------------------------------


class _FakeLibrary:
    def __init__(self, tracks):
        self._tracks = tracks

    def list_tracks(self, min_rating=0):
        return {"success": True, "tracks": self._tracks, "total": len(self._tracks)}

    def load_track(self, name):
        for t in self._tracks:
            if t["name"] == name:
                return {"success": True, "code": f"# {name} code", "track": t}
        return {"success": False, "error": f"Трек '{name}' не найден"}


@pytest.mark.unit
def test_music_fallback_plays_top_rated_track(monkeypatch):
    """LLM пустой ответ → /mcp/music_fallback → играет топ-трек (rating DESC)."""
    module = _load_mcp_server_module(monkeypatch)
    manager = MagicMock()
    manager.execute_code.return_value = {"success": True, "message": "ok"}
    library = _FakeLibrary([
        {"name": "top_track", "rating": 5, "title": "Top"},
        {"name": "ok_track", "rating": 3, "title": "Ok"},
    ])
    server = _FakeServer()
    server._music_manager = manager
    server._track_library = library

    msg = module.String()
    msg.data = '{"reason": "empty_response"}'
    module.MCPServer._on_music_fallback(server, msg)

    manager.execute_code.assert_called_once()
    # Первый в списке = с самым высоким rating (ORDER BY rating DESC).
    args = manager.execute_code.call_args
    assert args.kwargs["pattern_name"] == "top_track"
    assert "# top_track code" in args.args[0]


@pytest.mark.unit
def test_music_fallback_skips_when_manager_missing(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    server = _FakeServer()
    server._music_manager = None
    server._track_library = _FakeLibrary([])
    msg = module.String()
    msg.data = ""
    # Не падает и не играет.
    module.MCPServer._on_music_fallback(server, msg)
    assert any(
        "unavailable" in m for m in server.get_logger().warning_messages
    )


@pytest.mark.unit
def test_music_fallback_skips_when_library_empty(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    manager = MagicMock()
    server = _FakeServer()
    server._music_manager = manager
    server._track_library = _FakeLibrary([])
    msg = module.String()
    msg.data = ""
    module.MCPServer._on_music_fallback(server, msg)
    manager.execute_code.assert_not_called()
    assert any("пуста" in m for m in server.get_logger().warning_messages)
