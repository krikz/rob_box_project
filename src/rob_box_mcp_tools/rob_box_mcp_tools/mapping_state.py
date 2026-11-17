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

Имя и id карты здесь НЕ хранятся (#4392): одно место правды об активной карте -
строка ``maps.is_active`` в БД инструментов (``WaypointStore``), FSM читает её
через ``active_map`` (вызываемое, обычно ``waypoint_store.get_active_map``).
Раньше имя жило ещё и тут (с ``None`` после рестарта) и в ``/maps/mapping_state.json``
на Main Pi (не читается с ``4f5baba18``) - три места, три разных ответа.

Thread-safe: every public method acquires ``_lock``.
"""

import threading
import time
from typing import Callable, Dict, Optional


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

    def __init__(self, path: Optional[str] = None,
                 active_map: Optional[Callable[[], Optional[Dict]]] = None) -> None:
        # path argument kept for backward compat with tests — ignored
        self._lock = threading.Lock()
        self._active_map = active_map
        self._mode = "localization"
        self._updated_at = time.time()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_mapping(self) -> bool:
        """Return True if the robot is currently in active mapping mode."""
        with self._lock:
            return self._mode == "mapping"

    def is_localization(self) -> bool:
        """Return True if the robot is in localization/navigation mode."""
        with self._lock:
            return self._mode != "mapping"

    def _map_row(self) -> Dict:
        """Строка активной карты владельца (``maps``); нет источника или он упал - пусто, имени не выдумываем."""
        if self._active_map is None:
            return {}
        try:
            return dict(self._active_map() or {})
        except Exception:  # noqa: BLE001 - БД карт недоступна: имя неизвестно, режим FSM от этого не зависит
            return {}

    def get(self) -> dict:
        """Режим FSM и активная карта (имя и id - из владельца карт, не копия)."""
        with self._lock:
            mode, updated_at = self._mode, self._updated_at
        row = self._map_row()
        return {"mode": mode, "map_name": row.get("name"), "map_id": row.get("map_id"), "updated_at": updated_at}

    def set_mapping(self) -> None:
        """Transition to mapping mode (called by StartMappingTool)."""
        with self._lock:
            self._mode, self._updated_at = "mapping", time.time()

    def set_localization(self) -> None:
        """Transition to localization mode (called by FinishMappingTool / LoadMapTool)."""
        with self._lock:
            self._mode, self._updated_at = "localization", time.time()

    def is_tool_allowed(self, tool_name: str) -> bool:
        """Return True if *tool_name* may execute in the current mode."""
        if not self.is_mapping():
            return True  # localization mode — everything allowed
        return tool_name in MAPPING_WHITELIST

    def block_data(self) -> dict:
        """Данные отказа FSM-гарда (#4363): структурная причина и имя пишущейся карты - фразу строит диалог
        (``knowledge.MAP_FAILED``), а не парсинг текста ошибки."""
        from rob_box_dialog import knowledge as kn

        return {"reason": kn.MAP_ACTIVE_REASON, kn.MAP_ACTIVE_FIELD: self.get().get("map_name")}
