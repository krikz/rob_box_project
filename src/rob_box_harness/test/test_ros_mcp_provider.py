"""Tests for the DialogueNode ROS MCP ToolProvider adapter."""

from __future__ import annotations

import asyncio
from typing import Any, Mapping

import pytest

from rob_box_core.ports import ToolContext, ToolDescriptor, ToolNotFound
from rob_box_harness.executors import ROSMCPToolProvider
from rob_box_harness.executors.ros_mcp import _validate_json_schema


class _Bridge:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any], float | None]] = []

    def execute_tool_call_sync(
        self,
        tool_name: str,
        parameters: dict[str, Any],
        timeout: float | None = None,
    ) -> Mapping[str, Any]:
        self.calls.append((tool_name, parameters, timeout))
        return {"success": True, "message": "ok", "data": {"name": tool_name}}


def _provider() -> tuple[ROSMCPToolProvider, _Bridge]:
    bridge = _Bridge()
    provider = ROSMCPToolProvider(bridge)
    provider.register_tool(
        ToolDescriptor(
            name="status",
            description="status",
            parameters={
                "type": "object",
                "properties": {},
                "additionalProperties": False,
            },
            idempotent=True,
        )
    )
    return provider, bridge


async def test_ros_adapter_executes_existing_bridge_off_event_loop() -> None:
    provider, bridge = _provider()

    result = await provider.invoke(
        "status", {}, ToolContext(timeout=2.5, idempotency_key="status-1")
    )

    assert result.value == {"name": "status"}
    assert bridge.calls == [("status", {}, 2.5)]


async def test_ros_adapter_uses_typed_unknown_tool_error() -> None:
    provider, bridge = _provider()

    with pytest.raises(ToolNotFound):
        await provider.invoke("missing", {})

    assert bridge.calls == []


def test_ros_adapter_catalogue_is_dynamic_and_fresh() -> None:
    provider, _ = _provider()

    first = provider.list_tools()
    second = provider.list_tools()

    assert first == second
    assert first is not second
    assert first[0].name == "status"


# -- JSON Schema type-check coverage -----------------------------------------
#
# ``_validate_json_schema`` is the only line of defence against type-confused
# tool calls reaching the legacy ROS bridge. Each primitive JSON Schema type
# gets a passing call plus the obvious rejection (e.g. ``text=42`` for a
# string schema). ``validate_args`` delegates here, so the provider contract
# is exercised transitively via ``_validate_json_schema``.


@pytest.mark.parametrize(
    "json_type, value, valid",
    [
        ("string", "hello", True),
        ("string", "", True),
        ("string", 42, False),
        ("string", None, False),
        ("string", ["hello"], False),
        ("integer", 0, True),
        ("integer", 42, True),
        ("integer", True, False),       # bool subclass of int — must reject
        ("integer", 1.5, False),
        ("integer", "42", False),
        ("number", 0, True),
        ("number", 1.5, True),
        ("number", True, False),        # bool subclass gotcha
        ("number", "1.5", False),
        ("boolean", True, True),
        ("boolean", False, True),
        ("boolean", 0, False),
        ("boolean", "true", False),
        ("array", [1, 2, 3], True),
        ("array", [], True),
        ("array", (1, 2), False),
        ("array", {"a": 1}, False),
        ("object", {"a": 1}, True),
        ("object", {}, True),
        ("object", [1, 2], False),
        ("object", "value", False),
    ],
)
def test_validate_json_schema_type_check(
    json_type: str, value: Any, valid: bool
) -> None:
    schema = {
        "type": "object",
        "properties": {"payload": {"type": json_type}},
        "required": ["payload"],
    }
    result = _validate_json_schema(schema, {"payload": value})
    assert result.valid is valid, (json_type, value, result.errors)


def test_validate_json_schema_example_from_bug_report() -> None:
    # invoke("speak_text", {"text": 42}) must be rejected at validation
    # time, not leak into the legacy bridge. This is the concrete failure
    # scenario described in t_167edca7.
    schema = {
        "type": "object",
        "properties": {"text": {"type": "string"}},
        "required": ["text"],
        "additionalProperties": False,
    }
    result = _validate_json_schema(schema, {"text": 42})
    assert result.valid is False
    assert "text must be string" in result.errors


def test_validate_json_schema_unknown_type_is_permissive() -> None:
    # Forward-compat: schemas that reference JSON Schema draft types this
    # adapter doesn't implement yet must not falsely reject valid calls.
    schema = {
        "type": "object",
        "properties": {"when": {"type": "string", "format": "date-time"}},
    }
    assert _validate_json_schema(schema, {"when": "2026-01-02T03:04:05Z"}).valid


def test_validate_json_schema_preserves_required_and_extra_keys() -> None:
    # Type-check must compose with the existing required / additionalProperties
    # checks — both branches should still fire when violated together.
    schema = {
        "type": "object",
        "properties": {"text": {"type": "string"}},
        "required": ["text"],
        "additionalProperties": False,
    }
    result = _validate_json_schema(schema, {})
    assert not result.valid
    assert "missing required argument: text" in result.errors

    result = _validate_json_schema(
        schema, {"text": "ok", "unexpected": True}
    )
    assert not result.valid
    assert "unexpected argument: unexpected" in result.errors


def test_ros_adapter_rejects_mismatched_types_before_dispatch() -> None:
    # End-to-end: ``invoke`` with a wrong-type argument must raise a typed
    # ``ToolValidationError`` and never reach the bridge. We drive the
    # coroutine through ``asyncio.run`` so the test stays sync-only and
    # follows the pattern used in ``test_harness_lifecycle`` and
    # ``test_add_telegram_handlers`` for this repo.
    provider, bridge = _provider()
    provider.register_tool(
        ToolDescriptor(
            name="speak_text",
            description="speak",
            parameters={
                "type": "object",
                "properties": {"text": {"type": "string"}},
                "required": ["text"],
                "additionalProperties": False,
            },
            idempotent=True,
        )
    )

    with pytest.raises(Exception) as excinfo:
        asyncio.run(provider.invoke("speak_text", {"text": 42}))

    assert bridge.calls == []
    assert excinfo.value.__class__.__name__ == "ToolValidationError"
