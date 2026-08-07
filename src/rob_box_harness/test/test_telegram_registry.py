"""Tests for the new declarative (ctx, args) → Effect handler API.

Per host decision H.1, P1.4 introduces a new ``@command`` decorator
that registers handlers returning :class:`Effect` instances. The
legacy ``registry.register(name, (args, state) -> str)`` API stays
working for back-compat with the 9 placeholder handlers shipped
in ``telegram.py`` and the 28 tests that exercise them.

These tests pin the new contract; the registry itself is
implemented in the same module as the existing
``TelegramCommandRegistry`` to avoid splitting one logical class
across two files.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any

import pytest

from rob_box_harness.effects import (
    LogEffect,
    NoopBus,
    RecordingBus,
    SendReplyEffect,
    SpeakEffect,
)
from rob_box_harness.harnesses.telegram import (
    AuthMiddleware,
    SnapshotStore,
    TelegramCommandContext,
    TelegramCommandRegistry,
    TelegramHarness,
    TelegramState,
)


def _run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# RED-1: @command decorator + declarative signature
# ---------------------------------------------------------------------------


def test_command_decorator_registers_handler() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("/echo")
    async def _echo(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text=f"echo: {args}")

    assert "/echo" in reg
    result = _run(reg.dispatch("/echo", "ping", _ctx()))
    assert isinstance(result, SendReplyEffect)
    assert result.text == "echo: ping"
    assert result.channel == "chat-42"


def test_command_decorator_supports_sync_handler() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("/sync")
    def _sync(ctx: TelegramCommandContext, args: str) -> LogEffect:
        return LogEffect(message=f"sync: {args}")

    result = _run(reg.dispatch("/sync", "x", _ctx()))
    assert isinstance(result, LogEffect)
    assert result.message == "sync: x"


def test_command_decorator_records_description() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("/help", description="Show help text")
    async def _help(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text="help")

    info = reg.metadata("/help")
    assert info["description"] == "Show help text"


def test_command_decorator_without_slash_normalises() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("ping")
    async def _ping(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text="pong")

    assert "/ping" in reg


# ---------------------------------------------------------------------------
# RED-2: dispatch returns the Effect for the caller (harness) to dispatch
# ---------------------------------------------------------------------------


def test_dispatch_returns_effect_not_string() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("/reply")
    async def _reply(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text=args or "default")

    eff = _run(reg.dispatch("/reply", "hi", _ctx()))
    assert isinstance(eff, SendReplyEffect)
    assert eff.text == "hi"


def test_dispatch_returns_speak_effect_when_handler_emits_it() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("/say")
    async def _say(ctx: TelegramCommandContext, args: str) -> SpeakEffect:
        return SpeakEffect(text=args, voice="ru-RU")

    eff = _run(reg.dispatch("/say", "Привет", _ctx()))
    assert isinstance(eff, SpeakEffect)
    assert eff.text == "Привет"


def test_dispatch_unknown_command_returns_string_for_legacy() -> None:
    """Unknown commands return a plain string so the existing 28.
    regression tests asserting on substrings continue to work.
    The harness collapses that string into a SendReplyEffect at
    the bus boundary.
    """
    reg = TelegramCommandRegistry()
    outcome = _run(reg.dispatch("/nope", "", _ctx()))
    assert isinstance(outcome, str)
    assert "Неизвестная команда" in outcome


def test_dispatch_handler_exception_returns_friendly_effect() -> None:
    reg = TelegramCommandRegistry()

    @reg.command("/boom")
    async def _boom(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        raise RuntimeError("kaboom")

    eff = _run(reg.dispatch("/boom", "", _ctx()))
    assert isinstance(eff, SendReplyEffect)
    assert "ошибк" in eff.text.lower()


# ---------------------------------------------------------------------------
# RED-3: Legacy (args, state) -> str handlers still work for regression.
# Per H.4 we DO NOT delete the legacy 9 placeholders in telegram.py.
# ---------------------------------------------------------------------------


def test_legacy_str_handler_still_works() -> None:
    reg = TelegramCommandRegistry()

    def legacy(args: str, state: TelegramState) -> str:
        return f"legacy: {args}"

    reg.register("/legacy", legacy)
    out = _run(reg.dispatch("/legacy", "x", TelegramState()))
    assert out == "legacy: x"


# ---------------------------------------------------------------------------
# RED-4: TelegramHarness dispatches Effects through self.effects bus.
# This is the actual P1.4 acceptance behaviour.
# ---------------------------------------------------------------------------


@dataclass
class _MockResp:
    content: str = "mocked"
    tool_calls: list = field(default_factory=list)


class _MockLLM:
    name = "mock"
    def __init__(self, resp: _MockResp | None = None) -> None:
        self.resp = resp or _MockResp()
    async def complete(self, messages, **kw):
        return self.resp
    async def aclose(self): pass


def _make_harness(reg: TelegramCommandRegistry | None = None) -> tuple[TelegramHarness, RecordingBus]:
    from rob_box_harness.config import HarnessConfig
    from rob_box_harness.memory import InMemoryStore
    from rob_box_harness.tools import FakeToolProvider
    from rob_box_harness.transport import FakeTransport

    cfg = HarnessConfig.from_dict({"harness": {"kind": "telegram", "name": "tg"}})
    bus = RecordingBus()
    harness = TelegramHarness(
        config=cfg,
        llm=_MockLLM(),
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        transport=FakeTransport(),
        effects=bus,
    )
    if reg is not None:
        harness._registry = reg  # type: ignore[attr-defined]
    return harness, bus


def test_harness_dispatches_send_reply_through_bus_for_command() -> None:
    """Use the harness after wiring the registry — ``init()`` would.
    otherwise re-populate it with the 9 default placeholders.
    """
    from rob_box_harness.config import HarnessConfig
    from rob_box_harness.memory import InMemoryStore
    from rob_box_harness.tools import FakeToolProvider
    from rob_box_harness.transport import FakeTransport

    reg = TelegramCommandRegistry()

    @reg.command("/hello")
    async def _hello(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text="hello there")

    cfg = HarnessConfig.from_dict({"harness": {"kind": "telegram", "name": "tg"}})
    bus = RecordingBus()
    harness = TelegramHarness(
        config=cfg,
        llm=_MockLLM(),
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        transport=FakeTransport(),
        effects=bus,
    )
    # Replace the default registry BEFORE init() so init()'s placeholder
    # registration does not stomp on our declarative handler.
    harness._registry = reg  # type: ignore[attr-defined]
    _run(harness.init())
    result = _run(harness.step({"chat_id": "chat-42", "user_id": "u1", "command": "/hello"}))
    # Result is the harness's resolved string (for legacy callers)
    assert result == "hello there"
    # And the Effect was dispatched through the bus exactly once.
    assert len(bus.effects) == 1
    assert isinstance(bus.effects[0], SendReplyEffect)
    assert bus.effects[0].channel == "chat-42"


def test_harness_dispatches_unknown_command_through_bus() -> None:
    harness, bus = _make_harness(reg=TelegramCommandRegistry())
    _run(harness.init())
    result = _run(harness.step({"chat_id": "chat-7", "user_id": "u1", "command": "/ghost"}))
    assert "Неизвестная команда" in result
    assert any(isinstance(e, SendReplyEffect) and e.channel == "chat-7" for e in bus.effects)


def test_harness_dispatches_text_message_response_through_bus() -> None:
    harness, bus = _make_harness(reg=TelegramCommandRegistry())
    _run(harness.init())
    result = _run(harness.step({"chat_id": "chat-9", "user_id": "u1", "text": "ping"}))
    assert result == "mocked"  # mock LLM response
    assert any(
        isinstance(e, SendReplyEffect) and e.channel == "chat-9" and e.text == "mocked"
        for e in bus.effects
    )


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------


def _ctx(chat_id: str = "chat-42") -> TelegramCommandContext:
    return TelegramCommandContext(
        command="/test",
        args="",
        chat_id=chat_id,
        user_id="u1",
        state=TelegramState(chat_id=chat_id, user_id="u1"),
    )
