"""Tests for the canonical MCP bridge executor."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any, Mapping

import pytest

from rob_box_core.ports import ToolNotFound, ToolTimeout, ToolValidationError
from rob_box_harness.executors import (
    MCPBridgeExecutor,
    MCPRateLimit,
    MCPRetryPolicy,
)


class FakeTool:
    def __init__(
        self,
        name: str,
        *,
        required: tuple[str, ...] = (),
        idempotent: bool = False,
    ) -> None:
        self.name = name
        self.description = f"{name} description"
        self.idempotent = idempotent
        self.required = required

    def to_openai_tool_format(self) -> dict[str, Any]:
        properties = {key: {"type": "string"} for key in self.required}
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": properties,
                    "required": list(self.required),
                    "additionalProperties": False,
                },
            },
            "annotations": {"idempotentHint": self.idempotent},
        }

    def validate_parameters(self, **kwargs: Any) -> tuple[bool, str | None]:
        for key in self.required:
            if key not in kwargs:
                return False, f"missing required argument: {key}"
        unexpected = set(kwargs) - set(self.required)
        if unexpected:
            return False, f"unexpected argument: {sorted(unexpected)[0]}"
        return True, None


class FakeRegistry:
    def __init__(
        self,
        tools: Mapping[str, FakeTool],
        *,
        failures: int = 0,
        delay: float = 0.0,
    ) -> None:
        self.tools = dict(tools)
        self.failures = failures
        self.delay = delay
        self.calls: list[tuple[str, dict[str, Any]]] = []

    def list_tools(self) -> list[str]:
        return list(self.tools)

    def get_tool(self, name: str) -> FakeTool | None:
        return self.tools.get(name)

    async def execute(self, name: str, **kwargs: Any) -> Any:
        self.calls.append((name, dict(kwargs)))
        if self.delay:
            await asyncio.sleep(self.delay)
        if len(self.calls) <= self.failures:
            raise ConnectionError("temporary MCP failure")
        return SimpleNamespace(
            success=True,
            data={"tool": name, "arguments": kwargs},
            message=None,
            error=None,
        )


def test_list_tools_migrates_every_registry_entry() -> None:
    registry = FakeRegistry(
        {f"tool_{index}": FakeTool(f"tool_{index}") for index in range(24)}
    )
    provider = MCPBridgeExecutor(registry)

    tools = provider.list_tools()

    assert len(tools) == 24
    assert {tool.name for tool in tools} == set(registry.tools)
    assert tools is not provider.list_tools()


def test_list_tools_preserves_schema_and_idempotency() -> None:
    provider = MCPBridgeExecutor(
        FakeRegistry(
            {"echo": FakeTool("echo", required=("text",), idempotent=True)}
        )
    )

    descriptor = provider.list_tools()[0]

    assert descriptor.description == "echo description"
    assert descriptor.parameters["required"] == ["text"]
    assert descriptor.idempotent is True


def test_validate_args_uses_existing_mcp_validator() -> None:
    provider = MCPBridgeExecutor(
        FakeRegistry({"echo": FakeTool("echo", required=("text",))})
    )

    assert provider.validate_args("echo", {"text": "ok"}).valid is True
    invalid = provider.validate_args("echo", {})
    assert invalid.valid is False
    assert "missing required argument" in invalid.errors[0]


async def test_invoke_maps_llm_call_to_registry_and_result() -> None:
    registry = FakeRegistry(
        {"echo": FakeTool("echo", required=("text",))}
    )
    provider = MCPBridgeExecutor(registry)

    result = await provider.invoke("echo", {"text": "hello"})

    assert result.is_error is False
    assert result.value == {
        "tool": "echo",
        "arguments": {"text": "hello"},
    }
    assert registry.calls == [("echo", {"text": "hello"})]


async def test_unknown_tool_raises_typed_error() -> None:
    provider = MCPBridgeExecutor(FakeRegistry({}))

    with pytest.raises(ToolNotFound):
        await provider.invoke("missing", {})


async def test_invalid_arguments_raise_typed_error_before_dispatch() -> None:
    registry = FakeRegistry(
        {"echo": FakeTool("echo", required=("text",))}
    )
    provider = MCPBridgeExecutor(registry)

    with pytest.raises(ToolValidationError) as exc_info:
        await provider.invoke("echo", {})

    assert "missing required argument" in exc_info.value.errors[0]
    assert registry.calls == []


async def test_timeout_is_mapped_to_typed_error() -> None:
    registry = FakeRegistry({"slow": FakeTool("slow")}, delay=0.05)
    provider = MCPBridgeExecutor(registry, default_timeout=0.01)

    with pytest.raises(ToolTimeout, match="timed out"):
        await provider.invoke("slow", {})


def test_policy_rejects_invalid_values() -> None:
    with pytest.raises(ValueError, match="attempts"):
        MCPRetryPolicy(attempts=0)
    with pytest.raises(ValueError, match="base_delay"):
        MCPRetryPolicy(base_delay=-1)
    with pytest.raises(ValueError, match="max_delay"):
        MCPRetryPolicy(base_delay=2, max_delay=1)
    with pytest.raises(ValueError, match="max_per_second"):
        MCPRateLimit(max_per_second=0)
