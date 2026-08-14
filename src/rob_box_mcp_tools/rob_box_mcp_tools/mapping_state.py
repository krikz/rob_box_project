#!/usr/bin/env python3
"""
mapping_state.py — In-memory FSM state for the mapping lifecycle.

State machine::

    STARTUP ──────────────────────────────────────► localization (default)
                                                         │
                               "начни картировать"       │
                  ◄──────────────────────────────────────┘
              mapping  (blocks nav, waypoints, music)
                  │
                  │ "завершить картографирование"
                  ▼
              localization  (everything unlocked)

Purely in-memory — no file I/O, no cross-Pi sync issues.
Defaults to localization on every voice-assistant startup (safe default).
rtabmap mode is switched via ROS2 services (set_mode_mapping /
set_mode_localization) by the MCP tools — the actual source of truth
is the running rtabmap node, not a file.

Thread-safe: every public method acquires ``_lock``.
"""

import threading
import time
from typing import Optional


# Tools that are allowed to execute even during active mapping
MAPPING_WHITELIST: frozenset = frozenset({
    "finish_mapping",
    "continue_mapping",
    "optimize_map",
    "stop_navigation",
    "get_robot_status",
    "get_current_time",
    "get_battery_level",
    "get_perception_context",
    "speak_text",
    "listen_for_response",
    "play_animation",
    "play_sound",
    "set_volume",
    "set_pitch",
    "set_speed",
    "set_voice",
    "move_direction",   # allow manual driving while mapping
})


class MappingState:
    """In-memory FSM for the mapping lifecycle.

    State is lost on voice-assistant restart — that's intentional.
    A restarted container defaults to localization (safe), and the user
    can issue a new "start mapping" command if needed.
    """

    def __init__(self, path: Optional[str] = None) -> None:
        # path argument kept for backward compat with tests — ignored
        self._lock = threading.Lock()
        self._state: dict = {
            "mode": "localization",
            "map_name": None,
            "map_id": None,
            "updated_at": time.time(),
        }

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_mapping(self) -> bool:
        """Return True if the robot is currently in active mapping mode."""
        with self._lock:
            return self._state["mode"] == "mapping"

    def is_localization(self) -> bool:
        """Return True if the robot is in localization/navigation mode."""
        with self._lock:
            return self._state["mode"] != "mapping"

    def get(self) -> dict:
        """Return a copy of the current state dict."""
        with self._lock:
            return dict(self._state)

    def set_mapping(self, map_name: Optional[str] = None, map_id: Optional[str] = None) -> None:
        """Transition to mapping mode (called by StartMappingTool)."""
        with self._lock:
            self._state = {
                "mode": "mapping",
                "map_name": map_name,
                "map_id": map_id,
                "updated_at": time.time(),
            }

    def set_localization(self, map_name: Optional[str] = None, map_id: Optional[str] = None) -> None:
        """Transition to localization mode (called by FinishMappingTool / LoadMapTool)."""
        with self._lock:
            self._state = {
                "mode": "localization",
                "map_name": map_name or self._state.get("map_name"),
                "map_id": map_id or self._state.get("map_id"),
                "updated_at": time.time(),
            }

    def is_tool_allowed(self, tool_name: str) -> bool:
        """Return True if *tool_name* may execute in the current mode."""
        if not self.is_mapping():
            return True  # localization mode — everything allowed
        return tool_name in MAPPING_WHITELIST
