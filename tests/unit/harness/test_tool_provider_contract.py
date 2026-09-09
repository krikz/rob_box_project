"""Cross-implementation contract tests for canonical tool providers."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Mapping

import pytest

from rob_box_core.ports import ToolDescriptor, ToolProvider, ToolResult
from rob_box_harness.executors import LocalToolProvider, MCPBridgeExecutor


class _MCPTool:
    name = "mcp.status"
    description = "Return status"
    idempotent = True

    def to_openai_tool_format(self) -> dict[str, Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            "annotations": {"idempotentHint": True},
        }

    def validate_parameters(self, **kwargs: Any) -> tuple[bool, str | None]:
        return (not kwargs, None if not kwargs else "arguments are not accepted")


class _Registry:
    def __init__(self) -> None:
        self.tool = _MCPTool()

    def list_tools(self) -> list[str]:
        return [self.tool.name]

    def get_tool(self, name: str) -> _MCPTool | None:
        return self.tool if name == self.tool.name else None

    async def execute(self, name: str, **kwargs: Any) -> Any:
        return SimpleNamespace(success=True, data={"status": "ok"}, error=None)


@pytest.fixture(params=["local", "mcp"])
def provider(request: pytest.FixtureRequest) -> ToolProvider:
    if request.param == "local":
        return LocalToolProvider()
    return MCPBridgeExecutor(_Registry())


def test_provider_catalogue_contract(provider: ToolProvider) -> None:
    first = provider.list_tools()
    second = provider.list_tools()

    assert first
    assert first == second
    assert first is not second
    assert all(isinstance(item, ToolDescriptor) for item in first)
    assert all(item.name for item in first)
    assert all(item.parameters.get("type") == "object" for item in first)


def test_provider_validate_contract(provider: ToolProvider) -> None:
    descriptor = provider.list_tools()[0]

    validation = provider.validate_args(descriptor.name, {})

    assert validation.valid is True
    assert validation.errors == ()


async def test_provider_invoke_contract(provider: ToolProvider) -> None:
    descriptor = provider.list_tools()[0]

    result = await provider.invoke(descriptor.name, {})

    assert isinstance(result, ToolResult)
    assert result.is_error is False


async def test_provider_aclose_contract(provider: ToolProvider) -> None:
    assert await provider.aclose() is None
    assert await provider.aclose() is None
