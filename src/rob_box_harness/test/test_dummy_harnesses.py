"""Tests for the two built-in dummy harnesses: EchoHarness and UpperHarness.

Covers the user-facing behaviour of the smoke test fixtures:

* ``EchoHarness`` round-trips the LLM's response.
* ``UpperHarness`` uppercases it.
* Both persist turns to the memory store.
* Both dispatch an EchoEffect on the side-effect bus.
* Both decorate the snapshot with ``last_assistant_text``.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.effects import EchoEffect, NoopBus, RecordingBus
from rob_box_harness.harnesses.echo import EchoHarness
from rob_box_harness.harnesses.upper import UpperHarness
from rob_box_harness.providers.dummy import DummyLLMProvider


@pytest.mark.asyncio
async def test_echo_harness_returns_response() -> None:
    """``EchoHarness`` returns the LLM's response verbatim."""
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    harness = EchoHarness(config)
    await harness.init()
    result = await harness.run("hello world")
    assert result.output == "echo: hello world"


@pytest.mark.asyncio
async def test_echo_harness_persists_turns() -> None:
    """Both user and assistant turns are appended to memory."""
    config = HarnessConfig.from_dict(
        {"harness": {"kind": "echo", "name": "echo_session"}}
    )
    harness = EchoHarness(config)
    await harness.init()
    await harness.run("hi there")
    turns = await harness.memory.load_recent("echo_session", limit=10)
    assert [t.role for t in turns] == ["assistant", "user"]
    assert turns[0].content == "echo: hi there"
    assert turns[1].content == "hi there"


@pytest.mark.asyncio
async def test_echo_harness_dispatches_effect() -> None:
    """An EchoEffect is dispatched on the side-effect bus."""
    bus = RecordingBus()
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    harness = EchoHarness(config)
    harness.effects = bus
    await harness.init()
    await harness.run("hi")
    assert len(bus.effects) == 1
    assert isinstance(bus.effects[0], EchoEffect)
    assert bus.effects[0].text == "echo: hi"


@pytest.mark.asyncio
async def test_upper_harness_uppercases_response() -> None:
    """``UpperHarness`` uppercases the LLM's response."""
    config = HarnessConfig.from_dict({"harness": {"kind": "upper"}})
    harness = UpperHarness(config)
    await harness.init()
    result = await harness.run("hello")
    assert result.output == "ECHO: HELLO"


@pytest.mark.asyncio
async def test_upper_harness_dispatches_uppercase_effect() -> None:
    """The dispatched EchoEffect also carries the uppercased text."""
    bus = RecordingBus()
    config = HarnessConfig.from_dict({"harness": {"kind": "upper"}})
    harness = UpperHarness(config)
    harness.effects = bus
    await harness.init()
    await harness.run("hi")
    assert bus.effects[0].text == "ECHO: HI"


@pytest.mark.asyncio
async def test_ping_triggers_tool_call_path() -> None:
    """``ping`` triggers a tool call on the dummy LLM; the harness runs it."""
    from rob_box_harness.tools import ToolSpec

    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    harness = EchoHarness(config)
    await harness.init()
    # Wire the echo tool so the dummy LLM's ``tool_calls`` path is exercised.
    received: list[str] = []

    async def _echo_handler(args: dict) -> str:
        received.append(args.get("text", ""))
        return args.get("text", "")

    harness.tools.register(
        ToolSpec(name="echo", description="echo", parameters={}),
        _echo_handler,
    )
    await harness.run("ping")
    assert received == ["ping"], "the dummy LLM should have called the echo tool"


@pytest.mark.asyncio
async def test_harness_accepts_mapping_input() -> None:
    """A dict with ``text`` / ``message`` / ``prompt`` / ``input`` is accepted."""
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    harness = EchoHarness(config)
    await harness.init()
    result = await harness.run({"text": "from dict"})
    assert result.output == "echo: from dict"
