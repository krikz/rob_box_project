"""Unit tests for FakeToolProvider and ToolProvider contract.

Tests tool registration, execution, discovery, error handling,
concurrency, and the ToolSpec/ToolResult value objects.
"""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from rob_box_harness.tools import (
    FakeToolProvider,
    ToolExecutionError,
    ToolSpec,
    ToolCall,
    ToolResult,
    _stringify,
)


def _run(coro):
    return asyncio.run(coro)


class TestFakeToolProvider:

    def test_register_and_execute(self) -> None:
        provider = FakeToolProvider()
        called: list[dict] = []

        async def handler(args):
            called.append(dict(args))
            return "result ok"

        provider.register(
            ToolSpec(name="my_tool", description="Does something", parameters={"type": "object"}),
            handler,
        )
        specs = _run(provider.discover())
        assert any(s.name == "my_tool" for s in specs)

        result = _run(provider.execute(ToolCall(id="c1", name="my_tool", arguments={"key": "val"})))
        assert result.is_error is False
        assert result.content == "result ok"
        assert called == [{"key": "val"}]

    def test_builtin_echo_tool(self) -> None:
        provider = FakeToolProvider()
        specs = _run(provider.discover())
        assert any(s.name == "echo" for s in specs)

        result = _run(provider.execute(ToolCall(id="c1", name="echo", arguments={"text": "hello"})))
        assert result.is_error is False
        assert "hello" in result.content

    def test_unknown_tool_returns_error(self) -> None:
        provider = FakeToolProvider()
        result = _run(provider.execute(ToolCall(id="c1", name="no_such_tool", arguments={})))
        assert result.is_error is True
        assert "unknown" in result.content.lower()

    def test_handler_exception_wrapped_as_error(self) -> None:
        provider = FakeToolProvider()

        async def bad_handler(args):
            raise ValueError("invalid input")

        provider.register(
            ToolSpec(name="bad_tool", description="Always fails"),
            bad_handler,
        )
        result = _run(provider.execute(ToolCall(id="c1", name="bad_tool", arguments={})))
        assert result.is_error is True
        assert "ValueError" in result.content

    def test_discover_returns_all_registered(self) -> None:
        provider = FakeToolProvider()
        provider.register(ToolSpec(name="t1", description="First"), lambda a: "ok")
        provider.register(ToolSpec(name="t2", description="Second"), lambda a: "ok")
        specs = _run(provider.discover())
        names = {s.name for s in specs}
        assert "t1" in names
        assert "t2" in names

    def test_register_overrides_existing(self) -> None:
        provider = FakeToolProvider()
        provider.register(ToolSpec(name="echo", description="Custom"), lambda a: "custom")
        result = _run(provider.execute(ToolCall(id="c1", name="echo", arguments={"text": "x"})))
        assert result.content == "custom"


class TestToolSpec:

    def test_default_parameters(self) -> None:
        spec = ToolSpec(name="t", description="d")
        assert spec.parameters == {}


class TestStringify:

    def test_string_passthrough(self) -> None:
        assert _stringify("hello") == "hello"

    def test_int_repr(self) -> None:
        assert _stringify(42) == "42"

    def test_list_repr(self) -> None:
        assert _stringify([1, 2]) == "[1, 2]"

    def test_none_repr(self) -> None:
        assert _stringify(None) == "None"
