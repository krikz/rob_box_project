"""Tests for MCPServer._on_speaker_result (issue #1770).

Wires the server's `/voice/speaker/result` handler so that the memory
tools' `node.current_speaker_id` fallback gets the live speaker from
``speaker_id_node``. Without this, every tool call (save/search/context)
sees ``speaker_id=None`` and returns the global fact pool — Денчик
gets Саша's facts and vice versa.

These tests bypass the rclpy-aware MCPServer class and exercise the
handler directly against a stub server — same approach as
``test_mcp_server.py`` for unrelated startup hooks.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Bootstrap: load ``mcp_server.py`` source bypassing the rclpy init.
# ---------------------------------------------------------------------------

_PKG_NAME = "rob_box_mcp_tools"
_MCP_NAME = _PKG_NAME


def _install_rclpy_stub() -> None:
    """Provide a minimal rclpy / std_msgs shim.

    ``mcp_server.py`` imports ``rclpy``, ``rclpy.node``, ``rclpy.qos``,
    and ``std_msgs.msg.String`` at module load time. We provide minimal
    stand-ins so the import succeeds in CI (no ROS2 install).
    """
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **kw: None
    rclpy.shutdown = lambda *a, **kw: None
    rclpy.ok = lambda: True

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = type(
        "QoSProfile",
        (),
        {},
    )
    rclpy_qos.ReliabilityPolicy = type(
        "ReliabilityPolicy", (), {"RELIABLE": "reliable", "BEST_EFFORT": "b_e"}
    )
    rclpy_qos.HistoryPolicy = type(
        "HistoryPolicy", (), {"KEEP_LAST": "k_last"}
    )
    rclpy_qos.DurabilityPolicy = type(
        "DurabilityPolicy", (), {"VOLATILE": "volatile"}
    )

    rclpy_cb = types.ModuleType("rclpy.callback_groups")
    rclpy_cb.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    std_msgs.msg = std_msgs_msg

    for mod in [rclpy, rclpy_node, rclpy_qos, rclpy_cb, std_msgs, std_msgs_msg]:
        sys.modules[mod.__name__] = mod
    sys.modules["std_msgs.msg"] = std_msgs_msg


def _load_mcp_server_module():
    """Load ``mcp_server.py`` as ``rob_box_mcp_tools.mcp_server`` module.

    Use ``importlib.util.spec_from_file_location`` with a synthesized
    package so relative imports (``from .registry``) work without
    triggering the package ``__init__.py`` (which pulls in rclpy).
    """
    src_path = (
        Path(__file__).resolve().parent.parent
        / "rob_box_mcp_tools"
        / "mcp_server.py"
    )
    pkg_root = src_path.parent

    # Stub the registry module that ``from .registry import MCPToolRegistry`` needs.
    fake_registry = types.ModuleType(f"{_PKG_NAME}.registry")
    fake_registry.MCPToolRegistry = type(
        "MCPToolRegistry",
        (),
        {
            "register": lambda self, t: None,
            "__len__": lambda self: 0,
            "list_tools": lambda self: [],
        },
    )
    sys.modules[fake_registry.__name__] = fake_registry

    # Tools package: empty namespace + a dummy attribute for each name
    # imported in mcp_server.py (we never call any of them).
    fake_tools_pkg = types.ModuleType(f"{_PKG_NAME}.tools")
    fake_tools_pkg.__path__ = [str(pkg_root / "tools")]
    sys.modules[fake_tools_pkg.__name__] = fake_tools_pkg
    for name in [
        "NavigateToWaypointTool", "NavigateToCoordinatesTool", "MoveDirectionTool",
        "StopNavigationTool", "ListWaypointsTool", "SaveWaypointTool",
        "DeleteWaypointTool", "ClearWaypointsTool", "GetCurrentPoseTool",
        "SetVolumeTool", "SetPitchTool", "SetSpeedTool", "GetRobotStatusTool",
        "GetCurrentTimeTool", "GetPerceptionContextTool", "GetBatteryLevelTool",
        "StartMappingTool", "ContinueMappingTool", "FinishMappingTool",
        "OptimizeMapTool", "LoadMapTool", "PlayAnimationTool", "PlaySoundTool",
        "GetSoundInfoTool", "SpeakTextTool", "ListenForResponseTool",
        "EstimateTtsDurationTool", "RegisterSpeakerTool", "SetVoiceTool",
        "MemorySaveTool", "MemorySearchTool", "MemoryContextTool",
        "MusicManager", "TrackLibrary", "ExecuteMusicCodeTool",
        "StopMusicTool", "SetVibePresetTool", "GetMusicStateTool",
        "SaveTrackTool", "ListTracksTool", "LoadTrackTool", "DeleteTrackTool",
        "SetDjModeTool", "SearchSamplesTool", "FaqSearchTool", "SearchWebTool",
    ]:
        setattr(fake_tools_pkg, name, type(name, (), {}))

    # Stub the other sub-modules mcp_server imports.
    class _Auth:
        @classmethod
        def from_env(cls, **_kw):
            return None

    class _MappingState:
        def get(self):
            return {"mode": "idle", "map_name": None}

    for mod_name, attrs in [
        (f"{_PKG_NAME}.mcp_auth", {"RequestAuthenticator": _Auth}),
        (f"{_PKG_NAME}.waypoint_store", {"WaypointStore": type("WaypointStore", (), {})}),
        (f"{_PKG_NAME}.mapping_state", {"MappingState": _MappingState}),
        (f"{_PKG_NAME}.voice_state", {"VoiceStateStore": type("VoiceStateStore", (), {})}),
        ("rob_box_voice.core.voice_memory", {"VoiceMemory": type("VoiceMemory", (), {})}),
        ("rob_box_voice.core.faq_store", {"FAQStore": type("FAQStore", (), {})}),
        ("rob_box_voice.core.event_profile", {"load_event_profile": lambda *a, **kw: None}),
    ]:
        m = types.ModuleType(mod_name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[mod_name] = m

    # Synthesize the parent package.
    pkg = types.ModuleType(_PKG_NAME)
    pkg.__path__ = [str(pkg_root)]
    sys.modules[_PKG_NAME] = pkg

    # Now load mcp_server via importlib so the package context is correct.
    spec = importlib.util.spec_from_file_location(
        f"{_PKG_NAME}.mcp_server", str(src_path)
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load spec for {src_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# Bootstrap rclpy + module ONCE at import time so each test can grab
# ``MCPServer`` directly. This mirrors the conftest pattern in
# ``test_memory_speaker_id.py``.
_install_rclpy_stub()
_module_ns = _load_mcp_server_module()
MCPServer = _module_ns.MCPServer


class _StubLogger:
    """Capture info/warning/error messages for assertions."""

    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(str(msg))

    def warning(self, msg: str) -> None:
        self.warnings.append(str(msg))

    def error(self, msg: str) -> None:
        self.errors.append(str(msg))


class _StubNode:
    """Minimal MCPServer stand-in.

    We don't call ``MCPServer.__init__`` (it pulls in the whole
    registry, ROS publishers, etc.). Instead we bind the handler onto a
    stub that exposes the same ``self`` namespace — what we actually
    test is the pure-logic of the callback.
    """

    def __init__(self) -> None:
        self.logger = _StubLogger()
        self.current_speaker_id = None

    def get_logger(self):
        return self.logger


def _bind_handler(node: _StubNode):
    """Attach ``_on_speaker_result`` to ``node`` via a wrapper.

    The method accesses ``self.get_logger()`` and ``json.loads``; both
    are easy to provide. ``msg.data`` is the JSON string.
    """
    handler = MCPServer._on_speaker_result  # unbound method

    def wrapped(self, msg):
        self.logger = self.logger  # identity
        return handler(self, msg)

    # Bind directly: handler doesn't touch any rclpy internals.
    return handler.__get__(node, type(node))


def _msg(data: str):
    """Build a stub std_msgs.msg.String with ``.data``."""

    class _Msg:
        pass

    m = _Msg()
    m.data = data
    return m


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestOnSpeakerResult:
    def test_known_speaker_sets_current_speaker_id(self) -> None:
        node = _StubNode()
        handler = _bind_handler(node)
        handler(
            _msg(json.dumps(
                {"is_known": True, "speaker_id": "uuid-den-1234", "name": "Денчик", "confidence": 0.92}
            ))
        )
        assert node.current_speaker_id == "uuid-den-1234"

    def test_unknown_speaker_clears_current_speaker_id(self) -> None:
        node = _StubNode()
        # First prime with a known speaker …
        handler = _bind_handler(node)
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "uuid-1", "name": "A"})))
        assert node.current_speaker_id == "uuid-1"
        # … then send an unknown-speaker event.
        handler(_msg(json.dumps({"is_known": False})))
        assert node.current_speaker_id is None

    def test_empty_speaker_id_treated_as_unknown(self) -> None:
        """``is_known=True`` but missing/empty speaker_id ⇒ clear the cache.

        Defensive: speaker_id_node could publish a malformed message
        where is_known=true but the UUID is empty (race during
        registration). We must not leak a falsy value that downstream
        code might treat as a real id.
        """
        node = _StubNode()
        handler = _bind_handler(node)
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "", "name": "ghost"})))
        assert node.current_speaker_id is None

    def test_registered_event_updates_cache(self) -> None:
        """``{event: 'registered', speaker_id: ...}`` acks set the cache too.

        After registering a new speaker the next utterance from that
        user should hit their row immediately, even before the next
        ``is_known=true`` event lands.
        """
        node = _StubNode()
        handler = _bind_handler(node)
        handler(_msg(json.dumps({"event": "registered", "speaker_id": "uuid-new", "name": "Саша"})))
        assert node.current_speaker_id == "uuid-new"

    def test_malformed_json_does_not_raise(self) -> None:
        node = _StubNode()
        handler = _bind_handler(node)
        # Bad JSON: silently drop, do not change cache.
        handler(_msg("not-json-at-all"))
        assert node.current_speaker_id is None
        # Handler must not log an error (just return): JSON parse failures
        # from a noisy speaker_id_node would otherwise spam the operator.
        assert node.logger.errors == []

    def test_empty_msg_does_not_raise(self) -> None:
        node = _StubNode()
        handler = _bind_handler(node)
        handler(_msg(""))
        assert node.current_speaker_id is None

    def test_repeat_same_id_does_not_re_log(self) -> None:
        """Identical speaker_id → silent return (avoid log spam)."""
        node = _StubNode()
        handler = _bind_handler(node)
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "uuid-a", "name": "A"})))
        first_log_count = sum(1 for m in node.logger.infos if "current_speaker_id" in m)
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "uuid-a", "name": "A"})))
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "uuid-a", "name": "A"})))
        second_log_count = sum(1 for m in node.logger.infos if "current_speaker_id" in m)
        # Only the first transition should have been logged.
        assert second_log_count == first_log_count
        assert node.current_speaker_id == "uuid-a"

    def test_transition_logged_once(self) -> None:
        node = _StubNode()
        handler = _bind_handler(node)
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "uuid-a", "name": "A"})))
        # Switch speaker
        handler(_msg(json.dumps({"is_known": True, "speaker_id": "uuid-b", "name": "B"})))
        transition_logs = [
            m for m in node.logger.infos
            if "current_speaker_id" in m and "→" in m
        ]
        assert len(transition_logs) == 2  # init + transition
        # First log mentions new id, second log mentions both.
        assert "uuid-a" in transition_logs[0]
        assert "uuid-b" in transition_logs[1]
        assert node.current_speaker_id == "uuid-b"
