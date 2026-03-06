#!/usr/bin/env python3
"""
mapping_state.py — Persisted FSM state for the mapping lifecycle.

The state is stored in ``/maps/mapping_state.json`` (same volume as
``rtabmap.db``).  Both the bash startup script and Python MCP tools read/write
this file so that rtabmap always boots in the correct mode.

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

Thread-safe: every public method acquires ``_lock``.
"""

import json
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Optional

# Default path — overridable via env var MAPPING_STATE_PATH
_DEFAULT_STATE_PATH = "/maps/mapping_state.json"

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
    "move_direction",   # allow manual driving while mapping
})


class MappingState:
    """Read/write the mapping lifecycle state to a JSON file.

    Parameters
    ----------
    path:
        Override the default state file path (useful in tests).
    """

    def __init__(self, path: Optional[str] = None) -> None:
        self._path = Path(path or os.getenv("MAPPING_STATE_PATH", _DEFAULT_STATE_PATH))
        self._lock = threading.Lock()

        # Ensure parent directory exists
        self._path.parent.mkdir(parents=True, exist_ok=True)

        # Bootstrap default state if file doesn't exist
        if not self._path.exists():
            self._write_locked({
                "mode": "localization",
                "map_name": None,
                "map_id": None,
                "updated_at": time.time(),
            })

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_mapping(self) -> bool:
        """Return True if the robot is currently in active mapping mode."""
        return self.get().get("mode") == "mapping"

    def is_localization(self) -> bool:
        """Return True if the robot is in localization/navigation mode."""
        return self.get().get("mode") != "mapping"

    def get(self) -> dict:
        """Return a copy of the current state dict.

        Returns a safe default if the file can't be read.
        """
        with self._lock:
            try:
                return json.loads(self._path.read_text(encoding="utf-8"))
            except Exception:
                return {"mode": "localization", "map_name": None, "map_id": None}

    def set_mapping(self, map_name: Optional[str] = None, map_id: Optional[str] = None) -> None:
        """Transition to mapping mode (called by StartMappingTool)."""
        self._write_locked({
            "mode": "mapping",
            "map_name": map_name,
            "map_id": map_id,
            "updated_at": time.time(),
        })

    def set_localization(self, map_name: Optional[str] = None, map_id: Optional[str] = None) -> None:
        """Transition to localization mode (called by FinishMappingTool / LoadMapTool)."""
        # Preserve existing map info if not overridden
        current = self.get()
        self._write_locked({
            "mode": "localization",
            "map_name": map_name or current.get("map_name"),
            "map_id": map_id or current.get("map_id"),
            "updated_at": time.time(),
        })

    def is_tool_allowed(self, tool_name: str) -> bool:
        """Return True if *tool_name* may execute in the current mode."""
        if not self.is_mapping():
            return True  # localization mode — everything allowed
        return tool_name in MAPPING_WHITELIST

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _write_locked(self, data: dict) -> None:
        """Atomically write *data* to the state file (tmp → rename)."""
        with self._lock:
            tmp = self._path.with_suffix(".tmp")
            try:
                tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
                tmp.replace(self._path)
            except Exception:
                tmp.unlink(missing_ok=True)
                raise
