"""Retry and idempotency guarantees for ``MCPBridgeExecutor``."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any

import pytest

from rob_box_core.ports import ToolContext
from rob_box_harness.executors import (
    MCPBridgeExecutor,
    MCPRetryPolicy,
    MCPTransportError,
)


class Tool:
    def __init__(self, name: str, *, idempotent: bool) -> None:
        self.name = name
        self.idempotent = idempotent
        self.description = name

    def validate_parameters(self, **kwargs: Any) -> tuple[bool, str | None]:
        return True, None

    def to_openai_tool_format(self) -> dict[str, Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.name,
                "parameters": {"type": "object", "properties": {}},
            },
        }


class FlakyRegistry:
    def __init__(self, tool: Tool, *, fail_n: int = 0) -> None:
        self.tool = tool
        self.fail_n = fail_n
        self.calls = 0

    def list_tools(self) -> list[str]:
        return [self.tool.name]

    def get_tool(self, name: str) -> Tool | None:
        return self.tool if name == self.tool.name else None

    async def execute(self, name: str, **kwargs: Any) -> Any:
        self.calls += 1
        if self.calls <= self.fail_n:
            raise ConnectionError("temporary")
        return SimpleNamespace(
            success=True,
            data={"attempt": self.calls},
            error=None,
        )


async def test_idempotent_tool_retries_until_success() -> None:
    registry = FlakyRegistry(Tool("status", idempotent=True), fail_n=2)
    provider = MCPBridgeExecutor(
        registry,
        retry_policy=MCPRetryPolicy(attempts=4, base_delay=0),
    )

    result = await provider.invoke("status", {})

    assert result.value == {"attempt": 3}
    assert registry.calls == 3


async def test_non_idempotent_tool_never_retries() -> None:
    registry = FlakyRegistry(Tool("move", idempotent=False), fail_n=10)
    provider = MCPBridgeExecutor(
        registry,
        retry_policy=MCPRetryPolicy(attempts=4, base_delay=0),
    )

    with pytest.raises(MCPTransportError):
        await provider.invoke("move", {})

    assert registry.calls == 1


async def test_same_idempotency_key_returns_cached_result() -> None:
    registry = FlakyRegistry(Tool("status", idempotent=True))
    provider = MCPBridgeExecutor(registry)
    context = ToolContext(idempotency_key="request-42")

    first = await provider.invoke("status", {}, context)
    second = await provider.invoke("status", {}, context)

    assert first == second
    assert registry.calls == 1


async def test_concurrent_duplicate_call_is_coalesced() -> None:
    registry = FlakyRegistry(Tool("status", idempotent=True))
    original_execute = registry.execute

    async def slow_execute(name: str, **kwargs: Any) -> Any:
        await asyncio.sleep(0.01)
        return await original_execute(name, **kwargs)

    registry.execute = slow_execute  # type: ignore[method-assign]
    provider = MCPBridgeExecutor(registry)
    context = ToolContext(idempotency_key="concurrent")

    first, second = await asyncio.gather(
        provider.invoke("status", {}, context),
        provider.invoke("status", {}, context),
    )

    assert first == second
    assert registry.calls == 1


async def test_different_keys_execute_independently() -> None:
    registry = FlakyRegistry(Tool("status", idempotent=True))
    provider = MCPBridgeExecutor(registry)

    await provider.invoke(
        "status", {}, ToolContext(idempotency_key="one")
    )
    await provider.invoke(
        "status", {}, ToolContext(idempotency_key="two")
    )

    assert registry.calls == 2
