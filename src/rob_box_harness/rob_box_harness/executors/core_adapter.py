"""Backward-compatible adapters for legacy harness ``ToolCall`` consumers.

New code should inject :class:`rob_box_core.ports.ToolProvider` directly. The
adapter below keeps P0/P1.1 harness code working while migration proceeds.
"""

from __future__ import annotations

from rob_box_core.ports import (
    ToolContext,
    ToolProvider as CoreToolProvider,
    ToolResult as CoreToolResult,
)
from rob_box_harness.tools import ToolProvider as LegacyToolProvider
from rob_box_harness.tools import ToolSpec
from rob_box_llm.provider import ToolCall, ToolResult as LLMToolResult


class LegacyToolProviderAdapter(LegacyToolProvider):
    """Expose a core provider through P0 ``discover/execute`` methods."""

    def __init__(self, provider: CoreToolProvider) -> None:
        self._provider = provider
        self.name = provider.name

    async def discover(self) -> tuple[ToolSpec, ...]:
        return tuple(
            ToolSpec(
                name=descriptor.name,
                description=descriptor.description,
                parameters=dict(descriptor.parameters),
            )
            for descriptor in self._provider.list_tools()
        )

    async def execute(self, call: ToolCall) -> LLMToolResult:
        result = await self._provider.invoke(
            call.name,
            call.arguments,
            ToolContext(metadata={"tool_call_id": call.id}),
        )
        return LLMToolResult(
            tool_call_id=call.id,
            content=_result_content(result),
            is_error=result.is_error,
        )

    async def aclose(self) -> None:
        await self._provider.aclose()


def adapt_tool_provider(provider: CoreToolProvider) -> LegacyToolProviderAdapter:
    return LegacyToolProviderAdapter(provider)


def _result_content(result: CoreToolResult) -> str:
    if result.error is not None:
        return result.error
    if isinstance(result.value, str):
        return result.value
    return repr(result.value)


MCPBridgeProviderAdapter = LegacyToolProviderAdapter
LocalSkillProviderAdapter = LegacyToolProviderAdapter


__all__ = [
    "LegacyToolProviderAdapter",
    "LocalSkillProviderAdapter",
    "MCPBridgeProviderAdapter",
    "adapt_tool_provider",
]
