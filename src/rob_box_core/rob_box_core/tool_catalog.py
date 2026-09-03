"""Typed access to the shared dialogue-tool catalog.

The catalog is the **single declaration** of every tool the robot exposes:
name, LLM-facing description, JSON-Schema parameters, and the behaviour
hints (``read_only`` / ``destructive`` / ``idempotent``). It is derived from
the ``MCPTool`` subclasses that actually implement ``execute()`` — see
``tools/gen_tool_catalog.py`` — so the schema the LLM is shown and the code
that runs cannot drift apart.

This module lives in ``rob_box_core`` because both sides need it and neither
may reach for the other's dependencies: ``rob_box_harness`` must stay free of
ROS2, and ``rob_box_mcp_tools`` must not depend on the harness. ``rob_box_core``
has no dependencies at all.

History: before this existed, the catalog was written twice — once as tool
classes, once as hand-maintained manifests in
``rob_box_harness/core/tool_registry.py``. They disagreed on parameter
*names* for ``navigate_to_waypoint`` and ``move_direction`` (every LLM-driven
waypoint navigation failed), omitted 13 registered tools from the LLM's view
entirely, and had degraded 29 of 38 shared descriptions to one-line stubs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

from rob_box_core._tool_catalog_data import TOOL_CATALOG_DATA

__all__ = [
    "ToolCatalogEntry",
    "TOOL_CATALOG",
    "CORE_SKILL",
    "get_tool",
    "llm_visible_tools",
    "skill_names",
    "tool_names",
    "tools_for_skill",
]

#: Скилл, который предъявляется ВСЕГДА, независимо от активного домена.
#: Речь и базовый статус робота нужны в любом ходу.
CORE_SKILL: str = "core"


@dataclass(frozen=True)
class ToolCatalogEntry:
    """One tool's contract, shared by the LLM catalog and the MCP server."""

    name: str
    description: str
    #: OpenAI-style JSON Schema (``{"type": "object", "properties": {...}}``).
    parameters: Mapping[str, Any]
    #: Whether the tool is offered to the LLM. ``False`` keeps it executable
    #: over ``/mcp/execute`` while hiding it from the model — used for tools
    #: whose backend is gone (``generate_music``: MiniMax API 410 Gone).
    llm_visible: bool = True
    read_only: bool = False
    destructive: bool = True
    idempotent: bool = False
    execution_type: str = "medium"
    #: Доменные скиллы, в которые входит инструмент. Инструмент может
    #: входить в несколько (``stop_music`` — в composer, dj и player);
    #: описание при этом одно, оно здесь же, поэтому копии контракта,
    #: которая могла бы разойтись, не существует. Пусто у инструментов,
    #: скрытых от LLM. Проставляется генератором из ``SKILL_TOOLS``.
    skill: tuple[str, ...] = ()
    #: What ``execute()`` accepts, recorded so tests can prove the advertised
    #: schema and the runtime signature still agree.
    signature: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))

    def to_openai_tool(self) -> dict[str, Any]:
        """Render this entry in OpenAI / DeepSeek / Qwen tool-call format."""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": dict(self.parameters),
            },
            "annotations": {
                "readOnlyHint": self.read_only,
                "destructiveHint": self.destructive,
                "idempotentHint": self.idempotent,
            },
        }


def _build() -> tuple[ToolCatalogEntry, ...]:
    return tuple(
        ToolCatalogEntry(
            name=raw["name"],
            description=raw["description"],
            parameters=MappingProxyType(dict(raw["parameters"])),
            llm_visible=raw.get("llm_visible", True),
            read_only=raw.get("read_only", False),
            destructive=raw.get("destructive", True),
            idempotent=raw.get("idempotent", False),
            execution_type=raw.get("execution_type", "medium"),
            skill=tuple(raw.get("skill", ())),
            signature=MappingProxyType(dict(raw.get("signature", {}))),
        )
        for raw in TOOL_CATALOG_DATA
    )


#: Every tool, sorted by name — including ones hidden from the LLM.
TOOL_CATALOG: tuple[ToolCatalogEntry, ...] = _build()

_BY_NAME: Mapping[str, ToolCatalogEntry] = MappingProxyType(
    {entry.name: entry for entry in TOOL_CATALOG}
)


def get_tool(name: str) -> ToolCatalogEntry:
    """Return the entry for *name*.

    :raises KeyError: if no such tool is declared.
    """
    try:
        return _BY_NAME[name]
    except KeyError:
        raise KeyError(f"tool {name!r} is not in the catalog") from None


def llm_visible_tools() -> tuple[ToolCatalogEntry, ...]:
    """Return the tools that should be offered to the LLM."""
    return tuple(entry for entry in TOOL_CATALOG if entry.llm_visible)


def skill_names() -> tuple[str, ...]:
    """Вернуть имена всех объявленных скиллов, отсортированные."""
    seen: set[str] = set()
    for entry in TOOL_CATALOG:
        seen.update(entry.skill)
    return tuple(sorted(seen))


def tools_for_skill(
    *skills: str,
    include_core: bool = True,
) -> tuple[ToolCatalogEntry, ...]:
    """Вернуть llm_visible инструменты перечисленных скиллов.

    ``include_core`` добавляет :data:`CORE_SKILL` — он нужен в любом ходу
    (речь, статус, время), поэтому по умолчанию входит всегда.

    Пустой ``skills`` при ``include_core=True`` даёт ровно core: это
    состояние «скилл не активирован».

    :raises KeyError: если запрошен скилл, которого нет ни у одного
        инструмента — молча вернуть пустой набор значит предъявить LLM
        каталог без нужных инструментов и получить «нет такой функции».
    """
    requested = set(skills)
    if include_core:
        requested.add(CORE_SKILL)
    known = set(skill_names())
    unknown = sorted(requested - known)
    if unknown:
        raise KeyError(
            f"unknown skill(s): {', '.join(unknown)}; "
            f"known: {', '.join(sorted(known))}"
        )
    return tuple(
        entry
        for entry in llm_visible_tools()
        if requested.intersection(entry.skill)
    )


def tool_names(*, llm_visible_only: bool = False) -> tuple[str, ...]:
    """Return catalog tool names, optionally restricted to LLM-visible ones."""
    source = llm_visible_tools() if llm_visible_only else TOOL_CATALOG
    return tuple(entry.name for entry in source)
