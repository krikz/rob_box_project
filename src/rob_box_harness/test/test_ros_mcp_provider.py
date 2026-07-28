"""Tests for the DialogueNode ROS MCP ToolProvider adapter."""

from __future__ import annotations

from typing import Any, Mapping

import pytest

from rob_box_core.ports import ToolContext, ToolDescriptor, ToolNotFound
from rob_box_harness.executors import ROSMCPToolProvider


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
