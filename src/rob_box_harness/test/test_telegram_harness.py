"""Unit tests for :class:`TelegramHarness` — Telegram bot harness.

Tests command dispatch, authentication, text message processing,
memory persistence, snapshot store, and lifecycle. Uses ONLY
fake ports — no network, no real Telegram API.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.harnesses.telegram import (
    AuthMiddleware,
    SnapshotStore,
    TelegramCommandRegistry,
    TelegramHarness,
    TelegramState,
)
from rob_box_harness.memory import InMemoryStore
from rob_box_harness.tools import FakeToolProvider
from rob_box_harness.transport import FakeTransport
from rob_box_llm.provider import LLMMessage


@dataclass
class MockResponse:
    content: str = "Hello from bot!"
    tool_calls: list = field(default_factory=list)


class MockLLMProvider:
    name = "mock"

    def __init__(self, response: MockResponse | None = None) -> None:
        self.response = response or MockResponse()
        self.calls: list[list] = []

    async def complete(self, messages, **kwargs):
        self.calls.append(messages)
        return self.response

    async def aclose(self):
        pass


def _make_config(**overrides: Any) -> HarnessConfig:
    cfg: dict[str, Any] = {"harness": {"kind": "telegram", "name": "test_tg"}}
    cfg["harness"].update(overrides)
    return HarnessConfig.from_dict(cfg)


def _make_harness(**overrides: Any) -> TelegramHarness:
    return TelegramHarness(
        config=_make_config(**overrides),
        llm=MockLLMProvider(),
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        transport=FakeTransport(),
    )


def _run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestCommandDispatch:

    def test_start_command(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "456", "command": "/start"}))
        assert "РОББОКС" in result

    def test_help_command(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "456", "command": "/help"}))
        assert "Команды" in result or "/start" in result

    def test_status_command(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "456", "command": "/status"}))
        assert "Статус" in result

    def test_stop_command(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "456", "command": "/stop"}))
        assert "остановк" in result.lower()

    def test_command_with_args(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({
            "chat_id": "123", "user_id": "456",
            "command": "/navigate", "args": "кухня",
        }))
        assert isinstance(result, str)
        assert len(result) > 0

    def test_unknown_command(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({
            "chat_id": "123", "user_id": "456", "command": "/nonexistent",
        }))
        assert "Неизвестная команда" in result

    def test_last_command_tracked(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step({"chat_id": "123", "user_id": "456", "command": "/start"}))
        assert harness.state.last_command == "/start"


class TestTextMessage:

    def test_basic_text_message(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "456", "text": "hello"}))
        assert isinstance(result, str)
        assert len(result) > 0

    def test_text_message_count_increments(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        assert harness.state.message_count == 0
        _run(harness.step({"chat_id": "123", "user_id": "456", "text": "msg1"}))
        assert harness.state.message_count == 1
        _run(harness.step({"chat_id": "123", "user_id": "456", "text": "msg2"}))
        assert harness.state.message_count == 2

    def test_text_state_tracking(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step({"chat_id": "999", "user_id": "789", "username": "testuser", "text": "hi"}))
        assert harness.state.chat_id == "999"
        assert harness.state.user_id == "789"
        assert harness.state.username == "testuser"


class TestAuth:

    def test_auth_anonymous_blocked(self) -> None:
        config = _make_config()
        object.__setattr__(config, "telegram_allowed_users", ["111", "222"])
        harness = TelegramHarness(
            config=config, llm=MockLLMProvider(),
            tools=FakeToolProvider(), memory=InMemoryStore(), transport=FakeTransport(),
        )
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "999", "text": "hello"}))
        assert "запрещ" in result.lower()

    def test_auth_allowed_user(self) -> None:
        config = _make_config()
        object.__setattr__(config, "telegram_allowed_users", ["111", "222"])
        harness = TelegramHarness(
            config=config, llm=MockLLMProvider(),
            tools=FakeToolProvider(), memory=InMemoryStore(), transport=FakeTransport(),
        )
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "111", "text": "hello"}))
        assert "запрещ" not in result.lower()

    def test_auth_open_by_default(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step({"chat_id": "123", "user_id": "999", "text": "hello"}))
        assert "запрещ" not in result.lower()


class TestMemoryPersistence:

    def test_turns_saved(self) -> None:
        memory = InMemoryStore()
        harness = TelegramHarness(
            config=_make_config(), llm=MockLLMProvider(),
            tools=FakeToolProvider(), memory=memory, transport=FakeTransport(),
        )
        _run(harness.init())
        _run(harness.step({"chat_id": "123", "user_id": "456", "text": "hello"}))
        turns = _run(memory.load_recent("tg:123"))
        assert len(turns) >= 2

    def test_multiple_messages_accumulate(self) -> None:
        memory = InMemoryStore()
        harness = TelegramHarness(
            config=_make_config(), llm=MockLLMProvider(),
            tools=FakeToolProvider(), memory=memory, transport=FakeTransport(),
        )
        _run(harness.init())
        _run(harness.step({"chat_id": "123", "user_id": "456", "text": "msg1"}))
        _run(harness.step({"chat_id": "123", "user_id": "456", "text": "msg2"}))
        turns = _run(memory.load_recent("tg:123"))
        assert len(turns) >= 4


class TestSnapshotStore:

    def test_store_and_retrieve(self) -> None:
        store = SnapshotStore()
        key = store.store("123", b"fake_image_data")
        assert store.retrieve(key) == b"fake_image_data"

    def test_retrieve_missing(self) -> None:
        store = SnapshotStore()
        assert store.retrieve("nonexistent") is None

    def test_expire_removes_old(self) -> None:
        store = SnapshotStore()
        key = store.store("123", b"data")
        assert len(store) == 1
        store.expire(max_age_seconds=-1)  # expire everything
        assert len(store) == 0


class TestLifecycle:

    def test_init_idempotent(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        cmds_before = len(harness._registry)
        _run(harness.init())
        assert len(harness._registry) == cmds_before

    def test_teardown_idempotent(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.teardown())
        _run(harness.teardown())


class TestCommandRegistry:

    def test_register_and_dispatch(self) -> None:
        reg = TelegramCommandRegistry()
        called: list[str] = []

        def handler(args, state):
            called.append(args)
            return "OK"

        reg.register("/test", handler)
        result = _run(reg.dispatch("/test", "arg1 arg2", TelegramState()))
        assert result == "OK"
        assert called == ["arg1 arg2"]

    def test_register_without_slash(self) -> None:
        reg = TelegramCommandRegistry()
        reg.register("testcmd", lambda a, s: "ok")
        assert "/testcmd" in reg

    def test_unknown_command(self) -> None:
        reg = TelegramCommandRegistry()
        result = _run(reg.dispatch("/unknown", "", TelegramState()))
        assert "Неизвестная команда" in result

    def test_handler_exception_graceful(self) -> None:
        reg = TelegramCommandRegistry()

        def bad_handler(args, state):
            raise RuntimeError("boom")

        reg.register("/bad", bad_handler)
        result = _run(reg.dispatch("/bad", "", TelegramState()))
        assert "ошибк" in result.lower()


class TestAuthMiddleware:

    def test_empty_allows_all(self) -> None:
        auth = AuthMiddleware()
        assert auth.check("anyone") is True

    def test_specific_user_allowed(self) -> None:
        auth = AuthMiddleware(allowed_users=["111"])
        assert auth.check("111") is True

    def test_specific_user_blocked(self) -> None:
        auth = AuthMiddleware(allowed_users=["111"])
        assert auth.check("999") is False

    def test_add_remove_user(self) -> None:
        auth = AuthMiddleware(allowed_users=["111"])
        assert auth.check("111") is True
        assert auth.check("123") is False
        auth.add_user("123")
        assert auth.check("123") is True
        auth.remove_user("123")
        # After removal, the set no longer contains "123",
        # AND it's not empty → check returns False
        assert auth.check("123") is False
