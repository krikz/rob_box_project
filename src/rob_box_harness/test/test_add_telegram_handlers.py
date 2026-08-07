"""Tests for :func:`add_telegram_handlers` — wires a registry onto a.
python-telegram-bot ``Application``.

A5 in the P1.4 checklist: ``add_telegram_handlers(registry, app)`` must
register one ``CommandHandler`` per command in the registry with the
``python-telegram-bot`` ``Application``.

Per host decision H.4 the test uses a stub Application that records
``add_handler`` calls — we don't pull in ``python-telegram-bot`` for
this. The signature is the only thing the harness depends on.
"""

from __future__ import annotations

import asyncio
from typing import Any, Callable

import pytest

from rob_box_harness import SendReplyEffect
from rob_box_harness.harnesses.telegram import (
    TelegramCommandContext,
    TelegramCommandRegistry,
    add_telegram_handlers,
)


def _run(coro):  # noqa: ANN001
    return asyncio.run(coro)


class _FakeApp:
    """In-memory stand-in for ``telegram.ext.Application``."""

    def __init__(self) -> None:
        self.handlers: list[tuple[Any, int]] = []

    def add_handler(self, handler: Any, group: int = 0) -> None:
        self.handlers.append((handler, group))


class _FakeCommandHandler:
    """Captures the command name it was registered for."""

    def __init__(self, command: str, callback: Callable[..., Any]) -> None:
        self.command = command
        self.callback = callback


@pytest.fixture
def patched_handler_cls(monkeypatch: pytest.MonkeyPatch) -> None:
    """Inject a fake CommandHandler class into the add_telegram_handlers.
    module's namespace so we don't depend on python-telegram-bot."""

    import rob_box_harness.harnesses.telegram as harness_telegram_mod

    monkeypatch.setattr(
        harness_telegram_mod,
        "_PTB_COMMAND_HANDLER",
        _FakeCommandHandler,
        raising=False,
    )


def test_add_telegram_handlers_registers_all_commands(
    patched_handler_cls: None,
) -> None:
    registry = TelegramCommandRegistry()

    @registry.command("/start")
    async def _start(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text="hi")

    @registry.command("/help")
    async def _help(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text="help")

    app = _FakeApp()
    added = add_telegram_handlers(registry, app)  # type: ignore[arg-type]
    assert added == 2
    assert len(app.handlers) == 2
    commands_seen = sorted(handler.command for handler, _ in app.handlers)
    # python-telegram-bot CommandHandler is registered with the
    # command name WITHOUT the leading slash. The adapter strips
    # the slash; tests assert on the stripped form so the
    # implementation is exercised end-to-end.
    assert commands_seen == ["help", "start"]


def test_add_telegram_handlers_handles_empty_registry(
    patched_handler_cls: None,
) -> None:
    registry = TelegramCommandRegistry()
    app = _FakeApp()
    added = add_telegram_handlers(registry, app)  # type: ignore[arg-type]
    assert added == 0
    assert app.handlers == []


def test_add_telegram_handlers_strips_leading_slash(
    patched_handler_cls: None,
) -> None:
    registry = TelegramCommandRegistry()

    @registry.command("ping")  # no leading slash
    async def _ping(ctx: TelegramCommandContext, args: str) -> SendReplyEffect:
        return SendReplyEffect(channel=ctx.chat_id, text="pong")

    app = _FakeApp()
    added = add_telegram_handlers(registry, app)  # type: ignore[arg-type]
    assert added == 1
    handler, _ = app.handlers[0]
    # python-telegram-bot CommandHandler expects command WITHOUT the
    # leading slash. The adapter must strip it.
    assert handler.command == "ping"
