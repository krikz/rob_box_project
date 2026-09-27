"""Tests for the harness-side ``ToolRegistry``.

The registry is a **manifest-only** ToolProvider: it exposes one
``ToolSpec`` per LLM-visible tool in :mod:`rob_box_core.tool_catalog`.
Handlers are registered separately by ``ROSMCPToolProvider``, which bridges
ROS2 topics, so the registry stays ROS2-free and unit-testable.

These tests deliberately assert *against the catalog* rather than against a
copied list of names. The copied list was itself part of the problem this
module was cleaned up for: the inventory lived in four places (tool classes,
harness manifests, this test, and the voice-side skills), and every addition
had to be mirrored by hand — which is exactly what stopped happening.

Coverage:
* Every LLM-visible catalog tool is pre-registered, and nothing else is
* Hidden tools (``llm_visible=False``) never reach the LLM
* Specs carry non-empty descriptions and JSON-Schema parameters
* register() adds / rejects duplicates / honours override=True
"""

from __future__ import annotations

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG, llm_visible_tools
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.tools import ToolHandler, ToolSpec


# ---------------------------------------------------------------------------
# Construction & inventory
# ---------------------------------------------------------------------------


def test_default_registry_matches_the_llm_visible_catalog() -> None:
    """The registry must expose exactly the LLM-visible catalog tools.

    Both directions matter. A *missing* tool is the original bug — 13 tools
    that ``mcp_server`` registered were absent here, so the LLM could not
    call them at all (``task_delta``, ``stop_navigation``, the whole mapping
    FSM, the volume/pitch/speed controls). An *extra* tool is the mirror
    failure: the LLM picks a tool the MCP server cannot execute and the user
    hears «инструмент не найден» (this happened with the ``handle_*`` skill
    facades).
    """
    registry = ToolRegistry()
    names = {spec.name for spec in registry.list_tools()}
    expected = {entry.name for entry in llm_visible_tools()}
    assert not expected - names, f"missing tools in registry: {sorted(expected - names)}"
    assert not names - expected, f"unexpected tools in registry: {sorted(names - expected)}"


def test_hidden_tools_are_never_offered_to_the_llm() -> None:
    """``llm_visible=False`` tools stay executable but out of ``tools=``.

    ``generate_music`` is the live case: the MiniMax Music API returns 410
    Gone for new users, and an LLM that can see the schema will keep
    choosing it for «сыграй что-нибудь» instead of the library tools.
    """
    hidden = {entry.name for entry in TOOL_CATALOG if not entry.llm_visible}
    registered = {spec.name for spec in ToolRegistry().list_tools()}
    assert hidden, "expected at least one hidden tool to guard this contract"
    assert not (hidden & registered), (
        f"hidden tools leaked into the LLM catalog: {sorted(hidden & registered)}"
    )


def test_register_speaker_tool_is_exposed_to_llm() -> None:
    """Issue #1101 — register_speaker must be visible to the LLM.

    Regression guard: prior to this fix, RegisterSpeakerTool existed only
    in the MCP server-side registry (used for runtime dispatch via
    /mcp/execute → /mcp/result), but was NEVER added to the harness-side
    ``ToolRegistry`` that ``dialogue_node._build_tool_provider`` feeds
    into the chat-completion ``tools=`` argument. Result: LLM had no
    schema, never called it, voice-bio embeddings for new users were
    never saved.

    The spec must be JSON-Schema valid and expose ``name`` as optional
    string (LLM passes None to ask, real name to register).
    """
    registry = ToolRegistry()
    spec = registry.get("register_speaker")
    # Spec sanity
    assert spec.description, "register_speaker description must not be empty"
    params = spec.parameters or {}
    props = params.get("properties") or {}
    assert "name" in props, (
        "register_speaker.parameters.properties must expose 'name' (string)"
    )
    name_schema = props["name"]
    assert name_schema.get("type") == "string", (
        "register_speaker.name must be typed as 'string' so JSON-Schema "
        "validation accepts Cyrillic values from the LLM"
    )


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
    assert {entry.name for entry in llm_visible_tools()}.issubset(names)