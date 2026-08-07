"""``ToolRegistry`` — manifest-only registry of all dialogue tools.

This module is the **catalog** of every tool that ``dialogue_node``
exposes to the LLM. It is intentionally **pure Python** — no
``rclpy``, no ``openai-agents``, no ROS2 transport. The actual tool
implementations live in :class:`rob_box_harness.executors.ros_mcp
.ROSMCPToolProvider` (which bridges ROS2 topics), but the *specs*
(names, descriptions, JSON schemas) are owned here so that:

1. The catalog is unit-testable without ROS2.
2. The LLM-side ``complete()`` / ``stream()`` calls always have a
   consistent view of what tools exist, regardless of which handlers
   are wired up.
3. New tools can be added in one place without touching the dialogue
   shell.

The inventory mirrors ``dialogue_node._make_tools`` /
``_make_output_tools`` / ``_build_skills`` from the legacy code path,
decomposed into 29 *flat* tools and 5 *skill* sub-agents (per
``06-01-PLAN.md`` §W2).

Tool specs use :class:`rob_box_harness.tools.ToolSpec` so they can be
passed straight to :class:`FakeToolProvider` for unit tests and to
:class:`ROSMCPToolProvider` for production.
"""

from __future__ import annotations

from typing import Any, Callable

from rob_box_harness.tools import ToolHandler, ToolSpec


# ---------------------------------------------------------------------------
# JSON-Schema fragments reused across multiple tools
# ---------------------------------------------------------------------------

_TEXT_PROPERTY: dict[str, Any] = {
    "type": "string",
    "description": "Text string.",
}

_INT_PROPERTY: dict[str, Any] = {
    "type": "integer",
    "description": "Integer.",
}

_FLOAT_PROPERTY: dict[str, Any] = {
    "type": "number",
    "description": "Floating-point number.",
}

_BOOL_PROPERTY: dict[str, Any] = {
    "type": "boolean",
    "description": "Boolean flag.",
}


# ---------------------------------------------------------------------------
# Default handler — every tool's handler can be replaced via register().
# ---------------------------------------------------------------------------


async def _default_handler(args: dict[str, Any]) -> dict[str, Any]:
    """Default no-op handler used at registry-construction time.

    Real handlers (ROS2 bridges, MCP adapters) are wired in by the
    caller. The default returns the arguments dict unchanged so
    callers can still ``discover()`` the registry without crashing.
    """
    return dict(args)


# ---------------------------------------------------------------------------
# The 29 flat tools (audio / memory / status / navigation / music / tracks)
# ---------------------------------------------------------------------------


def _build_flat_specs() -> tuple[ToolSpec, ...]:
    """Build the 29 flat tool specs.

    Names and descriptions follow the legacy ``dialogue_node`` so the
    LLM-facing contract is stable across the harness migration.
    Parameter schemas are minimal but valid JSON Schema — enough for
    the LLM to call correctly.
    """
    return (
        ToolSpec(
            name="speak_text",
            description="Speak ``text`` via the TTS pipeline with the given animation.",
            parameters={
                "type": "object",
                "properties": {
                    "text": _TEXT_PROPERTY,
                    "animation": {
                        "type": "string",
                        "description": (
                            "LED matrix animation to play while speaking. "
                            "Choose one that matches the emotional tone or context. "
                            "Aliases are auto-normalized: neutral→idle, excited→happy, confused→thinking."
                        ),
                        "enum": [
                            "idle", "talking", "wakeup", "sleep",
                            "happy", "sad", "angry", "surprised", "thinking", "victory",
                            "error", "low_battery", "charging",
                            "police_lights", "ambulance", "fire_truck", "road_service",
                            "turn_left", "turn_right", "accelerating", "braking",
                            "neutral", "excited", "confused",
                        ],
                    },
                },
                "required": ["text"],
            },
        ),
        ToolSpec(
            name="estimate_tts_duration",
            description=(
                "Estimate TTS playback duration in seconds for a given text. "
                "Use BEFORE speak_text to plan music arrangement timing (#949). "
                "Returns estimate_sec (float) — approximate playback time with chipmunk 2x speedup."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "text": _TEXT_PROPERTY,
                    "chars_per_second": {
                        "type": "number",
                        "description": (
                            "Speech rate in chars/second. Default 30 — calibrated "
                            "for Russian TTS with ROBBOX chipmunk 2x effect."
                        ),
                    },
                },
                "required": ["text"],
            },
        ),
        ToolSpec(
            name="play_sound",
            description="Play a sound effect from the sound pack.",
            parameters={
                "type": "object",
                "properties": {"sound": _TEXT_PROPERTY},
                "required": ["sound"],
            },
        ),
        ToolSpec(
            name="play_animation",
            description="Trigger a robot animation for ``duration`` seconds.",
            parameters={
                "type": "object",
                "properties": {
                    "animation": _TEXT_PROPERTY,
                    "duration": _FLOAT_PROPERTY,
                },
                "required": ["animation"],
            },
        ),
        ToolSpec(
            name="memory_context",
            description="Load recent conversation context from long-term memory.",
            parameters={
                "type": "object",
                "properties": {"limit": _INT_PROPERTY},
            },
        ),
        ToolSpec(
            name="memory_save",
            description="Save a fact to long-term memory under a category.",
            parameters={
                "type": "object",
                "properties": {
                    "fact": _TEXT_PROPERTY,
                    "category": _TEXT_PROPERTY,
                },
                "required": ["fact"],
            },
        ),
        ToolSpec(
            name="memory_search",
            description="Search long-term memory for relevant facts.",
            parameters={
                "type": "object",
                "properties": {
                    "query": _TEXT_PROPERTY,
                    "limit": _INT_PROPERTY,
                },
                "required": ["query"],
            },
        ),
        ToolSpec(
            name="faq_search",
            description="Search the active event's FAQ for an answer.",
            parameters={
                "type": "object",
                "properties": {
                    "query": _TEXT_PROPERTY,
                    "limit": _INT_PROPERTY,
                },
                "required": ["query"],
            },
        ),
        ToolSpec(
            name="get_current_time",
            description="Return the current local time as an ISO-8601 string.",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="get_robot_status",
            description="Return the robot's current status (idle/moving/silent).",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="get_battery_level",
            description="Return the current battery percentage.",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="navigate_to_waypoint",
            description="Drive to a previously-saved waypoint by name.",
            parameters={
                "type": "object",
                "properties": {"name": _TEXT_PROPERTY},
                "required": ["name"],
            },
        ),
        ToolSpec(
            name="navigate_to_coordinates",
            description="Drive to an absolute (x, y, theta) pose.",
            parameters={
                "type": "object",
                "properties": {
                    "x": _FLOAT_PROPERTY,
                    "y": _FLOAT_PROPERTY,
                    "theta": _FLOAT_PROPERTY,
                },
                "required": ["x", "y", "theta"],
            },
        ),
        ToolSpec(
            name="move_direction",
            description="Move in a direction (``forward``/``back``/``left``/``right``) for a duration.",
            parameters={
                "type": "object",
                "properties": {
                    "direction": _TEXT_PROPERTY,
                    "duration": _FLOAT_PROPERTY,
                },
                "required": ["direction"],
            },
        ),
        ToolSpec(
            name="list_waypoints",
            description="List all saved waypoints.",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="save_waypoint",
            description="Save the current pose as a named waypoint.",
            parameters={
                "type": "object",
                "properties": {"name": _TEXT_PROPERTY},
                "required": ["name"],
            },
        ),
        ToolSpec(
            name="delete_waypoint",
            description="Delete a saved waypoint by name.",
            parameters={
                "type": "object",
                "properties": {"name": _TEXT_PROPERTY},
                "required": ["name"],
            },
        ),
        ToolSpec(
            name="clear_waypoints",
            description="Delete every saved waypoint.",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="get_current_pose",
            description="Return the robot's current pose as (x, y, theta).",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="voice_settings",
            description="Adjust voice / TTS settings (rate, pitch, voice id).",
            parameters={
                "type": "object",
                "properties": {
                    "rate": _FLOAT_PROPERTY,
                    "pitch": _FLOAT_PROPERTY,
                    "voice_id": _TEXT_PROPERTY,
                },
            },
        ),
        ToolSpec(
            name="search_samples",
            description="Search Renardo sample packs by keyword in filename. Returns letter, sample_index, and ready-to-use play_code.",
            parameters={
                "type": "object",
                "properties": {
                    "query": {"type": "string", "description": "Keyword: kick, snare, hat, bass, synth, vocal, glitch. Use '*' for overview."},
                    "pack": {"type": "string", "description": "Sample pack: 0_foxdot_default (standard) or 1_pitchglitch_samples (extended)."},
                    "case": {"type": "string", "description": "Letter case: lower or upper."},
                },
                "required": ["query"],
            },
        ),
        ToolSpec(
            name="execute_music_code",
            description=(
                "Execute FoxDot / music code in the runtime. Pass `segments` "
                "(number of bars, 4 beats per bar) as a safety net only — the "
                "system stops the music itself at tts_batch_complete (issue #990). "
                "Do NOT pass duration_sec (deprecated, ignored)."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "code": _TEXT_PROPERTY,
                    "segments": {
                        "type": "integer",
                        "description": (
                            "Number of bars (4 beats per bar) the background music "
                            "should play as a backstop. The system stops music at "
                            "tts_batch_complete; segments only caps playback if TTS "
                            "hangs. Typical song background: 8-16 bars. Omit if "
                            "unsure (default: until tts_batch_complete)."
                        ),
                    },
                    "duration_sec": {
                        "type": "number",
                        "description": (
                            "DEPRECATED (issue #990) — ignored for stopping. "
                            "Backward compatibility only; do not use."
                        ),
                    },
                },
                "required": ["code"],
            },
        ),
        ToolSpec(
            name="stop_music",
            description="Stop all music / DJ playback.",
            parameters={"type": "object", "properties": {}},
        ),
        # Canonical name is ``preset_name`` to match the MCP ``SetVibePresetTool``
        # schema (see rob_box_mcp_tools/tools/music.py) — the previous ``preset``
        # alias caused `mcp_server` validation errors in issue #935.
        ToolSpec(
            name="set_vibe_preset",
            description=(
                "Apply a named vibe preset (tempo, scale, root key). "
                "Preset values: chill (85bpm), energetic (140bpm), ambient (70bpm), "
                "jazz (120bpm), dark (100bpm), rock (120bpm), latin (105bpm), "
                "electronic (128bpm), cinematic (90bpm), funk (110bpm), "
                "reggae (75bpm), classical (100bpm)."
            ),
            parameters={
                "type": "object",
                "properties": {"preset_name": _TEXT_PROPERTY},
                "required": ["preset_name"],
            },
        ),
        ToolSpec(
            name="get_music_state",
            description="Return the current music runtime state.",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="set_dj_mode",
            description="Enable/disable DJ mode and configure transitions.",
            parameters={
                "type": "object",
                "properties": {
                    "enabled": _BOOL_PROPERTY,
                    "theme": _TEXT_PROPERTY,
                    "transition_seconds": _INT_PROPERTY,
                },
                "required": ["enabled"],
            },
        ),
        ToolSpec(
            name="list_tracks",
            description="List saved music tracks.",
            parameters={"type": "object", "properties": {}},
        ),
        ToolSpec(
            name="save_track",
            description="Save the current music runtime state as a named track.",
            parameters={
                "type": "object",
                "properties": {"name": _TEXT_PROPERTY},
                "required": ["name"],
            },
        ),
        ToolSpec(
            name="load_track",
            description="Load a previously-saved music track by name.",
            parameters={
                "type": "object",
                "properties": {"name": _TEXT_PROPERTY},
                "required": ["name"],
            },
        ),
        ToolSpec(
            name="delete_track",
            description="Delete a saved music track by name.",
            parameters={
                "type": "object",
                "properties": {"name": _TEXT_PROPERTY},
                "required": ["name"],
            },
        ),
    )


# ---------------------------------------------------------------------------
# The 5 skill sub-agents (compositor mode)
# ---------------------------------------------------------------------------


def _build_skill_specs() -> tuple[ToolSpec, ...]:
    """Build the 5 skill sub-agent specs (compositor mode).

    Each skill is a thin facade that wraps a group of flat tools and
    exposes them through a single high-level entry point.
    """
    return (
        ToolSpec(
            name="handle_music",
            description="Skill: handle music-related requests (search, play, stop, DJ).",
            parameters={
                "type": "object",
                "properties": {
                    "intent": {
                        "type": "string",
                        "description": "Music intent (play/stop/set_dj/save/load/…).",
                    },
                    "args": {
                        "type": "object",
                        "description": "Intent-specific arguments.",
                    },
                },
                "required": ["intent"],
            },
        ),
        ToolSpec(
            name="handle_navigation",
            description="Skill: handle navigation requests (goto waypoint, move direction).",
            parameters={
                "type": "object",
                "properties": {
                    "intent": {
                        "type": "string",
                        "description": "Navigation intent.",
                    },
                    "args": {
                        "type": "object",
                        "description": "Intent-specific arguments.",
                    },
                },
                "required": ["intent"],
            },
        ),
        ToolSpec(
            name="handle_memory",
            description="Skill: handle memory operations (save/search/load context).",
            parameters={
                "type": "object",
                "properties": {
                    "intent": {
                        "type": "string",
                        "description": "Memory intent.",
                    },
                    "args": {
                        "type": "object",
                        "description": "Intent-specific arguments.",
                    },
                },
                "required": ["intent"],
            },
        ),
        ToolSpec(
            name="handle_status",
            description="Skill: handle status queries (time, battery, pose, robot state).",
            parameters={
                "type": "object",
                "properties": {
                    "intent": {
                        "type": "string",
                        "description": "Status intent.",
                    },
                    "args": {
                        "type": "object",
                        "description": "Intent-specific arguments.",
                    },
                },
                "required": ["intent"],
            },
        ),
        ToolSpec(
            name="handle_faq",
            description="Skill: handle FAQ lookup against the active event profile.",
            parameters={
                "type": "object",
                "properties": {
                    "query": _TEXT_PROPERTY,
                    "limit": _INT_PROPERTY,
                },
                "required": ["query"],
            },
        ),
    )


# ---------------------------------------------------------------------------
# The registry
# ---------------------------------------------------------------------------


class ToolRegistry:
    """Manifest-only registry of all dialogue tools.

    Each tool is a (spec, handler) pair. The handler can be replaced
    via :meth:`register` (with ``override=True``) — production wires in
    :class:`ROSMCPToolProvider` handlers, tests register mocks.

    The registry is intentionally a plain Python class (not a
    :class:`ToolProvider` subclass) so it can be reused as a manifest
    source by *any* :class:`ToolProvider` implementation.
    """

    name = "tool_registry"

    def __init__(self) -> None:
        self._tools: dict[str, tuple[ToolSpec, ToolHandler]] = {}
        # Pre-register all 34 tools with the default no-op handler.
        for spec in _build_flat_specs() + _build_skill_specs():
            self._tools[spec.name] = (spec, _default_handler)

    # ---- read API -------------------------------------------------------

    def list_tools(self) -> tuple[ToolSpec, ...]:
        """Return every registered tool's spec."""
        return tuple(spec for spec, _ in self._tools.values())

    def get(self, name: str) -> ToolSpec:
        """Return the spec for ``name``.

        :raises KeyError: if ``name`` is not registered.
        """
        entry = self._tools.get(name)
        if entry is None:
            raise KeyError(f"tool {name!r} is not registered")
        return entry[0]

    def get_handler(self, name: str) -> ToolHandler:
        """Return the handler for ``name``.

        :raises KeyError: if ``name`` is not registered.
        """
        entry = self._tools.get(name)
        if entry is None:
            raise KeyError(f"tool {name!r} is not registered")
        return entry[1]

    # ---- write API ------------------------------------------------------

    def register(
        self,
        spec: ToolSpec,
        handler: ToolHandler,
        *,
        override: bool = False,
    ) -> None:
        """Register a new (or replace an existing) tool.

        :raises ValueError: if ``name`` is already registered and
            ``override=False`` (the default — prevents accidental
            clobbering of the pre-registered manifests).
        """
        if spec.name in self._tools and not override:
            raise ValueError(
                f"tool {spec.name!r} is already registered; "
                "pass override=True to replace it"
            )
        self._tools[spec.name] = (spec, handler)


__all__ = ["ToolRegistry", "ToolSpec", "ToolHandler"]