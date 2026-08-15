#!/usr/bin/env python3
"""Audit script: measure MCP tool definitions token overhead (BUG-14 / issue #813).

Instantiates every tool the MCPServer registers, serializes them in the
OpenAI Tool Calls format (exactly what gets published on /mcp/tools and
sent to the LLM on every request), and reports per-tool token counts.

Usage:
    PYTHONPATH=src/rob_box_mcp_tools python3 scripts/audit_mcp_tools.py

Requires: tiktoken (pip install tiktoken) for token counting.
"""
from __future__ import annotations

import json
import sys
import types

# ---------------------------------------------------------------------------
# Stub out ROS2 modules so the tools can be imported and instantiated
# without a running ROS environment (pure token-overhead measurement).
# ---------------------------------------------------------------------------
_STUB_MODULES = [
    "rclpy", "rclpy.node", "rclpy.callback_groups", "rclpy.qos",
    "rclpy.action", "std_msgs", "std_msgs.msg", "geometry_msgs",
    "geometry_msgs.msg", "nav2_msgs", "nav2_msgs.action", "action_msgs",
    "action_msgs.srv", "action_msgs.msg", "rcl_interfaces",
    "rcl_interfaces.srv", "rcl_interfaces.msg", "std_srvs", "std_srvs.srv",
    "rtabmap_msgs", "rtabmap_msgs.srv", "rob_box_voice", "rob_box_voice.core",
    "rob_box_voice.core.music_stack_validation",
    "rob_box_voice.core.sc_only_custom_synthdefs",
    "rob_box_perception_msgs", "rob_box_perception_msgs.msg",
]
for _name in _STUB_MODULES:
    sys.modules.setdefault(_name, types.ModuleType(_name))

def _stub_srv(name: str):
    """Create a stub srv type with .Request/.Response inner classes."""
    req = type(f"{name}.Request", (), {})
    resp = type(f"{name}.Response", (), {})
    return type(name, (), {"Request": req, "Response": resp})


sys.modules["rclpy.action"].ActionClient = type(
    "ActionClient", (), {"__init__": lambda self, *a, **k: None}
)
sys.modules["std_msgs.msg"].String = type("String", (), {})
sys.modules["geometry_msgs.msg"].PoseStamped = type("PoseStamped", (), {})
sys.modules["nav2_msgs.action"].NavigateToPose = type("NavigateToPose", (), {})
sys.modules["action_msgs.srv"].CancelGoal = _stub_srv("CancelGoal")
sys.modules["action_msgs.msg"].GoalInfo = type("GoalInfo", (), {})
sys.modules["rcl_interfaces.srv"].GetParameters = _stub_srv("GetParameters")
sys.modules["rcl_interfaces.srv"].SetParameters = _stub_srv("SetParameters")
sys.modules["rcl_interfaces.msg"].Parameter = type("Parameter", (), {})
sys.modules["rcl_interfaces.msg"].ParameterType = type("ParameterType", (), {"INTEGER": 0, "DOUBLE": 0})
sys.modules["rcl_interfaces.msg"].ParameterValue = type("ParameterValue", (), {})
sys.modules["std_srvs.srv"].Empty = _stub_srv("Empty")
sys.modules["rtabmap_msgs.srv"].LoadDatabase = _stub_srv("LoadDatabase")
sys.modules["rtabmap_msgs.srv"].PublishMap = _stub_srv("PublishMap")
sys.modules["rtabmap_msgs.srv"].CleanupLocalGrids = _stub_srv("CleanupLocalGrids")
sys.modules["rtabmap_msgs.srv"].DetectMoreLoopClosures = _stub_srv("DetectMoreLoopClosures")
sys.modules["rtabmap_msgs.srv"].GlobalBundleAdjustment = _stub_srv("GlobalBundleAdjustment")
_music_status_stub = type(
    "MusicStackStatus", (), {"__init__": lambda self, *a, **k: setattr(self, "is_healthy", True)}
)
sys.modules["rob_box_voice.core.music_stack_validation"].MusicStackStatus = _music_status_stub
sys.modules["rob_box_voice.core.music_stack_validation"].load_sclang_health = lambda *a, **k: _music_status_stub()
sys.modules["rob_box_voice.core.sc_only_custom_synthdefs"].register_sc_only_custom_synthdefs = lambda: None

from rob_box_mcp_tools.tools import (  # noqa: E402
    NavigateToWaypointTool, NavigateToCoordinatesTool, MoveDirectionTool,
    StopNavigationTool, ListWaypointsTool, SaveWaypointTool, DeleteWaypointTool,
    ClearWaypointsTool, GetCurrentPoseTool, SetVolumeTool, SetPitchTool,
    SetSpeedTool, GetRobotStatusTool, GetCurrentTimeTool,
    GetPerceptionContextTool, GetBatteryLevelTool, StartMappingTool,
    ContinueMappingTool, FinishMappingTool, OptimizeMapTool, LoadMapTool,
    PlayAnimationTool, PlaySoundTool, GetSoundInfoTool, FaqSearchTool,
    SpeakTextTool, EstimateTtsDurationTool, ListenForResponseTool,
    RegisterSpeakerTool, SearchWebTool, MemorySaveTool, MemorySearchTool,
    MemoryContextTool, MusicManager, ExecuteMusicCodeTool, StopMusicTool,
    SetVibePresetTool, GetMusicStateTool, SetDjModeTool, SearchSamplesTool,
    SaveTrackTool, ListTracksTool, LoadTrackTool, DeleteTrackTool,
)
from rob_box_mcp_tools.registry import MCPToolRegistry


class _FakeLogger:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass
    def debug(self, *a, **k): pass


class _FakeParam:
    def __init__(self, value):
        self.value = value


class FakeNode:
    """Minimal node satisfying the __init__ side effects of every tool."""

    def __init__(self):
        self._logger = _FakeLogger()
        self._params = {"music_max_amp": 0.42}

    def get_logger(self):
        return self._logger

    def declare_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        return _FakeParam(self._params.get(name))

    def create_publisher(self, *a, **k):
        return _FakePub()

    def create_subscription(self, *a, **k):
        return None

    def create_client(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def has_parameter(self, name):
        return name in self._params


class _FakePub:
    def publish(self, *a, **k):
        pass


def build_registry() -> MCPToolRegistry:
    node = FakeNode()
    registry = MCPToolRegistry()
    wp_store = object()
    tf_buffer = object()
    mapping_state = object()

    regs = [
        NavigateToWaypointTool(node, wp_store),
        NavigateToCoordinatesTool(node),
        MoveDirectionTool(node),
        StopNavigationTool(node),
        ListWaypointsTool(node, wp_store),
        SaveWaypointTool(node, wp_store, tf_buffer, mapping_state),
        DeleteWaypointTool(node, wp_store),
        ClearWaypointsTool(node, wp_store),
        GetCurrentPoseTool(node, tf_buffer),
        SetVolumeTool(node),
        SetPitchTool(node),
        SetSpeedTool(node),
        GetRobotStatusTool(node),
        GetCurrentTimeTool(node),
        GetPerceptionContextTool(node),
        GetBatteryLevelTool(node),
        StartMappingTool(node, wp_store, mapping_state),
        ContinueMappingTool(node),
        FinishMappingTool(node, wp_store, mapping_state),
        OptimizeMapTool(node),
        LoadMapTool(node, wp_store, mapping_state),
        PlayAnimationTool(node),
        PlaySoundTool(node),
        GetSoundInfoTool(node),
        FaqSearchTool(node),
        SpeakTextTool(node),
        EstimateTtsDurationTool(node),
        ListenForResponseTool(node),
        RegisterSpeakerTool(node),
        SearchWebTool(node),
        MemorySaveTool(node),
        MemorySearchTool(node),
        MemoryContextTool(node),
    ]
    for t in regs:
        registry.register(t)

    music_manager = MusicManager(max_amp=0.42)
    registry.register(ExecuteMusicCodeTool(node, music_manager))
    registry.register(StopMusicTool(node, music_manager))
    registry.register(SetVibePresetTool(node, music_manager))
    registry.register(GetMusicStateTool(node, music_manager))
    registry.register(SetDjModeTool(node))
    registry.register(SearchSamplesTool(node))
    track_library = music_manager.track_library if hasattr(music_manager, "track_library") else None
    if track_library is None:
        from rob_box_mcp_tools.tools import TrackLibrary
        track_library = TrackLibrary()
    registry.register(SaveTrackTool(node, track_library, music_manager))
    registry.register(ListTracksTool(node, track_library))
    registry.register(LoadTrackTool(node, track_library, music_manager))
    registry.register(DeleteTrackTool(node, track_library))
    return registry


def main() -> None:
    import os
    import tempfile

    # TrackLibrary/VoiceMemory persist to VOICE_MEMORY_DB_PATH; point it to a
    # temp file so the audit does not touch /data (permission denied in CI/dev).
    if not os.getenv("VOICE_MEMORY_DB_PATH"):
        os.environ["VOICE_MEMORY_DB_PATH"] = os.path.join(
            tempfile.gettempdir(), "audit_mcp_tools_voice_memory.db"
        )
    import tiktoken

    enc = tiktoken.get_encoding("cl100k_base")
    registry = build_registry()
    tools = registry.get_openai_tools()

    print(f"=== MCP tools audit (issue #813, BUG-14) ===")
    print(f"Total tools: {len(tools)}")
    print()

    rows = []
    for t in sorted(tools, key=lambda x: len(json.dumps(x, ensure_ascii=False))):
        name = t["function"]["name"]
        text = json.dumps(t, ensure_ascii=False)
        chars = len(text)
        tokens = len(enc.encode(text))
        rows.append((name, chars, tokens, len(t["function"].get("description", ""))))

    total_chars = sum(r[1] for r in rows)
    total_tokens = sum(r[2] for r in rows)
    for name, chars, tokens, desc_len in rows:
        print(f"{tokens:5d} tok {chars:6d} ch  desc={desc_len:5d}  {name}")
    print()
    print(f"TOTAL: {total_tokens} tokens, {total_chars} chars, {len(rows)} tools")
    print(f"Average per tool: {total_tokens // max(len(rows), 1)} tokens")

    # Overlap analysis helpers
    print()
    print("=== Overlap candidates (issue #813) ===")
    by_prefix = {}
    for name, _, _, _ in rows:
        prefix = name.split("_")[0] if "_" in name else name
        by_prefix.setdefault(prefix, []).append(name)
    for prefix, names in sorted(by_prefix.items()):
        if len(names) > 1:
            print(f"  {prefix}: {', '.join(names)}")


if __name__ == "__main__":
    main()
