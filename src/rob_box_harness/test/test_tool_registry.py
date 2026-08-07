"""Tests for the harness-side ``ToolRegistry``.

The registry is a **manifest-only** ToolProvider — it owns the
``ToolSpec`` for each of the 34 tools that ``dialogue_node`` exposes
(29 flat + 5 skill sub-agents). The actual handlers are registered
separately by the ``ROSMCPToolProvider`` which bridges ROS2 topics,
so the registry stays ROS2-free and unit-testable.

Coverage:
* All 34 tools pre-registered with non-empty descriptions
* list_tools() returns tuple of ToolSpec
* ToolSpec names are unique (no duplicates)
* get(name) / get_handler(name) work
* register() adds a new tool; raises on duplicate name with override=False
* Pre-defined list matches the spec from 06-01-PLAN.md §W2
"""

from __future__ import annotations

import pytest

from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.tools import ToolHandler, ToolSpec


# ---------------------------------------------------------------------------
# Expected tool inventory (from .planning/06-01-PLAN.md §W2)
# ---------------------------------------------------------------------------

FLAT_TOOL_NAMES: tuple[str, ...] = (
    "speak_text",
    "play_sound",
    "play_animation",
    "memory_context",
    "memory_save",
    "memory_search",
    "faq_search",
    "get_current_time",
    "get_robot_status",
    "get_battery_level",
    "navigate_to_waypoint",
    "navigate_to_coordinates",
    "move_direction",
    "list_waypoints",
    "save_waypoint",
    "delete_waypoint",
    "clear_waypoints",
    "get_current_pose",
    "voice_settings",
    "search_samples",
    "execute_music_code",
    "stop_music",
    "set_vibe_preset",
    "get_music_state",
    "set_dj_mode",
    "list_tracks",
    "save_track",
    "load_track",
    "delete_track",
)

SKILL_TOOL_NAMES: tuple[str, ...] = (
    "handle_music",
    "handle_navigation",
    "handle_memory",
    "handle_status",
    "handle_faq",
)

EXPECTED_TOOL_NAMES: frozenset[str] = frozenset(FLAT_TOOL_NAMES) | frozenset(
    SKILL_TOOL_NAMES
)


# ---------------------------------------------------------------------------
# Construction & inventory
# ---------------------------------------------------------------------------


def test_default_registry_contains_all_34_tools() -> None:
    """The default ToolRegistry must pre-register all 34 tools."""
    registry = ToolRegistry()
    names = {spec.name for spec in registry.list_tools()}
    assert len(names) == 34
    assert names == EXPECTED_TOOL_NAMES


def test_every_tool_has_a_non_empty_description() -> None:
    """Every ToolSpec must have a non-empty description and a name."""
    registry = ToolRegistry()
    for spec in registry.list_tools():
        assert spec.name, "ToolSpec.name must not be empty"
        assert spec.description, f"{spec.name}: description must not be empty"


def test_tool_spec_parameters_is_mapping() -> None:
    """Every ToolSpec.parameters must be a Mapping (JSON-serialisable)."""
    registry = ToolRegistry()
    for spec in registry.list_tools():
        assert isinstance(spec.parameters, dict) or spec.parameters is None


def test_no_duplicate_tool_names() -> None:
    """ToolRegistry.list_tools() must return unique names."""
    registry = ToolRegistry()
    names = [spec.name for spec in registry.list_tools()]
    assert len(names) == len(set(names))


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def test_register_adds_new_tool() -> None:
    """register() must add a new tool to the registry."""
    registry = ToolRegistry()
    spec = ToolSpec(
        name="custom_tool",
        description="A custom tool.",
        parameters={"type": "object"},
    )

    async def _handler(args):
        return "ok"

    registry.register(spec, _handler)
    assert "custom_tool" in {s.name for s in registry.list_tools()}


def test_register_rejects_duplicate_by_default() -> None:
    """register() must raise when a name is already taken."""
    registry = ToolRegistry()
    spec = ToolSpec(name="speak_text", description="override", parameters={})

    async def _handler(args):
        return "ok"

    with pytest.raises(ValueError, match="already registered"):
        registry.register(spec, _handler)


def test_register_allows_override_with_flag() -> None:
    """register(spec, handler, override=True) must replace the existing entry."""
    registry = ToolRegistry()

    async def _old(args):
        return "old"

    async def _new(args):
        return "new"

    original = ToolSpec(name="custom", description="old", parameters={})
    registry.register(original, _old)
    replacement = ToolSpec(name="custom", description="new", parameters={})
    registry.register(replacement, _new, override=True)
    assert registry.get("custom").description == "new"


def test_get_returns_spec_by_name() -> None:
    """ToolRegistry.get(name) returns the ToolSpec for ``name``."""
    registry = ToolRegistry()
    spec = registry.get("speak_text")
    assert spec.name == "speak_text"


def test_get_unknown_tool_raises() -> None:
    """ToolRegistry.get(unknown) must raise KeyError."""
    registry = ToolRegistry()
    with pytest.raises(KeyError):
        registry.get("does_not_exist")


def test_get_handler_returns_callable() -> None:
    """ToolRegistry.get_handler(name) returns the registered handler."""
    registry = ToolRegistry()

    async def _handler(args):
        return "x"

    registry.register(
        ToolSpec(name="custom", description="d", parameters={}), _handler
    )
    handler = registry.get_handler("custom")
    assert handler is _handler


def test_get_handler_unknown_raises() -> None:
    """ToolRegistry.get_handler(unknown) must raise KeyError."""
    registry = ToolRegistry()
    with pytest.raises(KeyError):
        registry.get_handler("does_not_exist")


# ---------------------------------------------------------------------------
# Provider integration
# ---------------------------------------------------------------------------


def test_registry_adapts_to_fake_tool_provider() -> None:
    """A ToolRegistry can populate a FakeToolProvider with all 34 specs."""
    import asyncio

    from rob_box_harness.tools import FakeToolProvider

    registry = ToolRegistry()
    provider = FakeToolProvider()
    for spec in registry.list_tools():
        handler = registry.get_handler(spec.name)

        async def _default_handler(args, _h=handler):
            if hasattr(_h, "__call__"):
                result = _h(args)
                if hasattr(result, "__await__"):
                    return await result
                return result
            return None

        provider.register(spec, _default_handler)

    discovered = asyncio.run(provider.discover())
    names = {s.name for s in discovered}
    assert EXPECTED_TOOL_NAMES.issubset(names)