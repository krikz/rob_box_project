"""Contract tests for the canonical :mod:`rob_box_core.ports` API."""

from __future__ import annotations

import inspect
from typing import Any, Mapping

import pytest

from rob_box_core.ports import (
    ToolContext,
    ToolDescriptor,
    ToolNotFound,
    ToolProvider,
    ToolResult,
    ValidationResult,
)


class _EchoProvider(ToolProvider):
    name = "echo"

    def list_tools(self) -> list[ToolDescriptor]:
        return [
            ToolDescriptor(
                name="echo",
                description="Echo text",
                parameters={
                    "type": "object",
                    "properties": {"text": {"type": "string"}},
                    "required": ["text"],
                    "additionalProperties": False,
                },
                idempotent=True,
            )
        ]

    async def invoke(
        self,
        name: str,
        args: Mapping[str, Any],
        ctx: ToolContext | None = None,
    ) -> ToolResult:
        if name != "echo":
            raise ToolNotFound(name, provider=self.name, tool_name=name)
        return ToolResult(
            value=args["text"],
            metadata={"caller": ctx.caller_id if ctx else None},
        )

    def validate_args(self, name: str, args: Mapping[str, Any]) -> ValidationResult:
        if name != "echo":
            return ValidationResult(valid=False, errors=(f"unknown tool: {name}",))
        if not isinstance(args.get("text"), str):
            return ValidationResult(valid=False, errors=("text must be a string",))
        return ValidationResult(valid=True)


def test_tool_provider_is_an_abstract_contract() -> None:
    assert inspect.isabstract(ToolProvider)
    assert ToolProvider.__abstractmethods__ == frozenset(
        {"list_tools", "invoke", "validate_args"}
    )


def test_list_tools_is_synchronous_and_returns_a_fresh_list() -> None:
    provider = _EchoProvider()

    first = provider.list_tools()
    second = provider.list_tools()

    assert not inspect.isawaitable(first)
    assert first == second
    assert first is not second
    assert first[0].idempotent is True


async def test_invoke_round_trips_context_and_value() -> None:
    provider = _EchoProvider()

    result = await provider.invoke(
        "echo",
        {"text": "hello"},
        ToolContext(caller_id="contract-test", idempotency_key="same-call"),
    )

    assert result == ToolResult(
        value="hello", metadata={"caller": "contract-test"}
    )
    assert result.is_error is False


def test_validation_result_exposes_valid_and_legacy_ok() -> None:
    result = _EchoProvider().validate_args("echo", {"text": 42})

    assert result.valid is False
    assert result.ok is False
    assert result.errors == ("text must be a string",)


async def test_aclose_is_idempotent_by_default() -> None:
    provider = _EchoProvider()

    assert await provider.aclose() is None
    assert await provider.aclose() is None


async def test_unknown_tool_uses_canonical_error() -> None:
    with pytest.raises(ToolNotFound) as exc_info:
        await _EchoProvider().invoke("missing", {})

    assert exc_info.value.provider == "echo"
    assert exc_info.value.tool_name == "missing"
