"""``ToolRegistry`` — the dialogue tools the LLM is offered.

This module is the harness-side **view** of the tool catalog. It stays
intentionally pure Python — no ``rclpy``, no ``openai-agents``, no ROS2
transport — so the catalog is unit-testable without a ROS2 runtime, which
is why it exists as a separate layer at all.

What changed
------------
The specs used to be *hand-written here*, in parallel with the ``MCPTool``
classes in ``rob_box_mcp_tools`` that actually implement ``execute()``. Two
declarations of the same contract drifted, badly and silently:

* ``navigate_to_waypoint`` advertised ``name`` while ``execute()`` took
  ``waypoint`` — every LLM-driven waypoint navigation failed validation.
* ``move_direction`` advertised ``duration`` against a ``distance``
  parameter — ``TypeError`` on every call that used it.
* 13 tools registered by ``mcp_server`` were missing here entirely, so the
  LLM could not call them at all (``task_delta``, ``stop_navigation``, the
  whole mapping FSM, ``set_volume``/``set_pitch``/``set_speed``, …).
* 29 of 38 shared descriptions had decayed into one-line stubs
  (``play_sound``: 782 characters of guidance reduced to 40).

Specs are now derived from :mod:`rob_box_core.tool_catalog`, which is
generated from the tool classes by ``tools/gen_tool_catalog.py`` and kept
honest by ``test_tool_catalog_is_current``. Adding a tool means writing one
``MCPTool`` subclass and regenerating — there is no second list to update.
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, Mapping

from rob_box_core.tool_catalog import (
    ToolCatalogEntry,
    llm_visible_tools,
    tools_for_skill,
)
from rob_box_harness.tools import ToolHandler, ToolSpec

__all__ = ["ToolRegistry", "ToolSpec", "ToolHandler", "spec_from_catalog"]


# ---------------------------------------------------------------------------
# Default handler — every tool's handler can be replaced via register().
# ---------------------------------------------------------------------------


async def _default_handler(args: Mapping[str, Any]) -> dict[str, Any]:
    """Default no-op handler used at registry-construction time.

    Real handlers (ROS2 bridges, MCP adapters) are wired in by the caller.
    The default returns the arguments dict unchanged so callers can still
    ``discover()`` the registry without crashing.
    """
    return dict(args)


def spec_from_catalog(entry: ToolCatalogEntry) -> ToolSpec:
    """Convert a catalog entry into the harness's :class:`ToolSpec`."""
    return ToolSpec(
        name=entry.name,
        description=entry.description,
        parameters=dict(entry.parameters),
    )


class ToolRegistry:
    """Manifest registry of the dialogue tools offered to the LLM.

    Each tool is a (spec, handler) pair. The handler can be replaced via
    :meth:`register` (with ``override=True``) — production wires in
    :class:`ROSMCPToolProvider` handlers, tests register mocks.

    The registry is a plain Python class (not a :class:`ToolProvider`
    subclass) so it can be reused as a manifest source by *any*
    :class:`ToolProvider` implementation.

    Only ``llm_visible`` catalog entries are pre-registered. A tool hidden
    from the LLM stays executable over ``/mcp/execute`` but must never
    appear in ``tools=`` — the LLM picking a dead tool is a user-visible
    failure (see ``generate_music``, whose MiniMax backend returns 410).
    """

    name = "tool_registry"

    def __init__(self) -> None:
        self._tools: dict[str, tuple[ToolSpec, ToolHandler]] = {
            entry.name: (spec_from_catalog(entry), _default_handler)
            for entry in llm_visible_tools()
        }

    # ---- read API -------------------------------------------------------

    def list_tools(
        self,
        *,
        skills: "tuple[str, ...] | None" = None,
    ) -> tuple[ToolSpec, ...]:
        """Return registered tool specs.

        ``skills=None`` (default) returns everything, which is today's
        behaviour and what the LLM sees while tool-narrowing is off.

        Passing ``skills`` narrows the result to the tools of those
        domain skills plus ``core`` — the Move B path. Only tools that are
        actually registered here are returned, so a handler replaced via
        :meth:`register` keeps working.

        :raises KeyError: on an unknown skill name (never silently empty —
            an empty tool list reaches the LLM as "нет такой функции").
        """
        if skills is None:
            return tuple(spec for spec, _ in self._tools.values())
        wanted = {entry.name for entry in tools_for_skill(*skills)}
        return tuple(
            spec
            for name, (spec, _) in self._tools.items()
            if name in wanted
        )

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
