"""Backward-compatibility tests for the P0 harness tool contract."""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_core.ports import (
    ToolContext,
    ToolDescriptor,
    ToolNotFound,
    ToolProvider,
    ToolResult,
    ValidationResult,
)
from rob_box_harness.executors import LegacyToolProviderAdapter, adapt_tool_provider
from rob_box_harness.tools import ToolProvider as LegacyToolProvider
from rob_box_llm.provider import ToolCall


class _Provider(ToolProvider):
    name = "core-provider"

    def __init__(self) -> None:
        self.contexts: list[ToolContext | None] = []
        self.closed = 0

    def list_tools(self) -> list[ToolDescriptor]:
        return [
            ToolDescriptor(
                name="status",
                description="Read status",
                parameters={"type": "object", "properties": {}},
                idempotent=True,
            )
        ]

    def validate_args(self, name: str, args: Mapping[str, Any]) -> ValidationResult:
        return ValidationResult(valid=name == "status" and not args)

    async def invoke(
        self,
        name: str,
        args: Mapping[str, Any],
        ctx: ToolContext | None = None,
    ) -> ToolResult:
        self.contexts.append(ctx)
        if name != "status":
            raise ToolNotFound(name)
        return ToolResult(value={"healthy": True})

    async def aclose(self) -> None:
        self.closed += 1


async def test_adapter_exposes_core_catalogue_to_legacy_harness() -> None:
    adapter = adapt_tool_provider(_Provider())

    specs = await adapter.discover()

    assert isinstance(adapter, LegacyToolProvider)
    assert isinstance(adapter, LegacyToolProviderAdapter)
    assert len(specs) == 1
    assert specs[0].name == "status"
    assert specs[0].description == "Read status"
    assert specs[0].parameters == {"type": "object", "properties": {}}


async def test_adapter_maps_legacy_tool_call_and_preserves_call_id() -> None:
    provider = _Provider()
    adapter = adapt_tool_provider(provider)

    result = await adapter.execute(
        ToolCall(id="call-42", name="status", arguments={})
    )

    assert result.tool_call_id == "call-42"
    assert result.is_error is False
    assert result.content == "{'healthy': True}"
    assert provider.contexts[0] is not None
    assert provider.contexts[0].metadata == {"tool_call_id": "call-42"}


async def test_adapter_delegates_close() -> None:
    provider = _Provider()
    adapter = adapt_tool_provider(provider)
    await adapter.aclose()
    assert provider.closed == 1


def test_adapter_wraps_invoke_exception_into_error_tool_result() -> None:
    """ToolValidationError/любое исключение invoke НЕ убивает цикл —
    оборачивается в ToolResult(is_error=True), LLM получает причину."""

    import asyncio

    provider = _Provider()
    adapter = adapt_tool_provider(provider)
    result = asyncio.run(
        adapter.execute(
            ToolCall(
                id="call_x",
                name="speak_text",
                arguments={"text": ""},
            )
        )
    )
    assert result.is_error is True
    assert "ToolNotFound" in result.content
    assert result.tool_call_id == "call_x"
