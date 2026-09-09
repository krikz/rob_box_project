"""Scenario tests for the ``tools`` and ``transport`` ports.

These two modules are the public-facing "ports" harnesses plug into
(ADR-0001 §2.4.2 and §2.4.5). They were not covered by the in-package
unit suite (only ``isinstance`` checks via ``test_harness_lifecycle``),
so the per-module coverage report showed:

* ``tools.py``     : 83% (missing: ToolExecutionError ctor + echo
                     registration + handler success/exception paths)
* ``transport.py`` : 75% (missing: bind/unbind + dispatch + the four
                     FakeTransport event entry points + the
                     test-only ``feed_*`` helpers)

The tests below are minimal and behaviour-only: they don't poke at
attributes that aren't part of the documented contract, and they
prefer fakes over mocks so the assertions read like the spec.

Why these tests live in ``tests/unit/harness/`` (not
``src/rob_box_harness/test/``):

The in-package suite already imports ``FakeToolProvider`` and
``FakeTransport`` purely as type assertions inside
``test_harness_lifecycle.py``. Adding scenario tests alongside
implementation tests would muddy the layering ("is this testing the
fake, or the port?"). This tree keeps port-contract tests grouped
under ``tests/unit/harness/`` per the documented convention.

Implementation notes:

* ``BaseTransport`` is itself abstract (it inherits the abstract
  ``on_*`` methods from :class:`Transport`), so the bind/unbind/
  dispatch tests use :class:`FakeTransport` instead, which is the
  concrete subclass that ships with the framework.
* The asyncio-mode is ``auto`` (set in ``pytest.ini``), so we don't
  decorate every ``async def`` test with ``@pytest.mark.asyncio``.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import pytest

from rob_box_harness.tools import (
    FakeToolProvider,
    ToolExecutionError,
    ToolSpec,
)
from rob_box_harness.transport import (
    EventHandler,
    FakeTransport,
    KeyEvent,
    TelegramUpdate,
    VadEvent,
)
from rob_box_llm.provider import ToolCall, ToolResult


# ===========================================================================
# tools.py — FakeToolProvider
# ===========================================================================


class TestFakeToolProviderEcho:
    """The built-in ``echo`` tool is the smoke-test workhorse."""

    async def test_echo_tool_is_registered_by_default(self) -> None:
        """A fresh provider has a single ``echo`` tool ready to call."""
        provider = FakeToolProvider()
        specs = await provider.discover()
        assert [s.name for s in specs] == ["echo"]

    async def test_echo_tool_honors_text_argument(self) -> None:
        """The default echo handler returns ``f"echo: {text}"``."""
        provider = FakeToolProvider()
        result = await provider.execute(
            ToolCall(id="call-1", name="echo", arguments={"text": "hello"})
        )
        assert result.tool_call_id == "call-1"
        assert result.content == "echo: hello"
        assert result.is_error is False

    async def test_echo_handler_tolerates_missing_text(self) -> None:
        """Missing ``text`` argument yields ``"echo: "`` (not a KeyError)."""
        provider = FakeToolProvider()
        result = await provider.execute(
            ToolCall(id="call-2", name="echo", arguments={})
        )
        assert result.content == "echo: "
        assert result.is_error is False

    async def test_user_can_pre_register_echo_and_skip_builtin(self) -> None:
        """A pre-registered ``echo`` blocks the default installation.

        The "_register_builtin_echo" guard prevents the provider from
        silently overwriting a user-supplied echo with the default.
        We assert this by verifying that a custom handler is the one
        that survives after construction.
        """
        seen: list[Mapping[str, Any]] = []

        def custom_handler(args: Mapping[str, Any]) -> str:
            seen.append(dict(args))
            return "custom"

        provider = FakeToolProvider(
            tools={
                "echo": (
                    ToolSpec(
                        name="echo",
                        description="custom echo",
                        parameters={"type": "object"},
                    ),
                    custom_handler,
                )
            }
        )

        result = await provider.execute(
            ToolCall(id="call-3", name="echo", arguments={"text": "x"})
        )
        assert result.content == "custom"
        assert seen == [{"text": "x"}]


class TestFakeToolProviderRegistration:
    """The `register` method and the discover/execute lifecycle."""

    async def test_register_adds_new_tool(self) -> None:
        """Registering a tool makes it discoverable and callable."""
        provider = FakeToolProvider()

        def add(args: Mapping[str, Any]) -> int:
            return int(args["a"]) + int(args["b"])

        provider.register(
            ToolSpec(
                name="add",
                description="Add two numbers.",
                parameters={
                    "type": "object",
                    "properties": {
                        "a": {"type": "integer"},
                        "b": {"type": "integer"},
                    },
                },
            ),
            add,
        )

        specs = await provider.discover()
        names = {s.name for s in specs}
        assert "add" in names
        assert "echo" in names  # default still present

        result = await provider.execute(
            ToolCall(id="call-add", name="add", arguments={"a": "2", "b": "3"})
        )
        assert result.content == "5"
        assert result.is_error is False

    async def test_register_overrides_existing_name(self) -> None:
        """Re-registering the same name silently replaces the previous entry."""
        provider = FakeToolProvider()

        def first(args: Mapping[str, Any]) -> str:
            return "first"

        def second(args: Mapping[str, Any]) -> str:
            return "second"

        spec = ToolSpec(name="echo", description="overridden", parameters={})
        provider.register(spec, first)
        provider.register(spec, second)

        result = await provider.execute(
            ToolCall(id="call-ovr", name="echo", arguments={"text": "any"})
        )
        assert result.content == "second"

    async def test_execute_async_handler_is_awaited(self) -> None:
        """When the handler returns an awaitable, the provider awaits it."""
        provider = FakeToolProvider()

        async def async_double(args: Mapping[str, Any]) -> int:
            return int(args["n"]) * 2

        provider.register(
            ToolSpec(name="double", description="", parameters={}),
            async_double,
        )

        result = await provider.execute(
            ToolCall(id="call-async", name="double", arguments={"n": "21"})
        )
        assert result.content == "42"
        assert result.is_error is False

    async def test_execute_unknown_tool_returns_error_result(self) -> None:
        """Unknown tool name returns a ToolResult with is_error=True.

        This is *not* the same as ToolExecutionError — tool-level
        errors are surfaced as a result so the LLM can re-think.
        Transport-level errors (separate exception) are tested below.
        """
        provider = FakeToolProvider()
        result = await provider.execute(
            ToolCall(id="call-404", name="nope", arguments={})
        )
        assert result.tool_call_id == "call-404"
        assert result.content == "unknown tool: nope"
        assert result.is_error is True

    async def test_execute_handler_exception_is_wrapped_in_result(self) -> None:
        """A raising handler yields a ToolResult(is_error=True) with type+msg."""
        provider = FakeToolProvider()

        def boom(args: Mapping[str, Any]) -> str:
            raise ValueError("kaboom")

        provider.register(
            ToolSpec(name="boom", description="", parameters={}), boom
        )

        result = await provider.execute(
            ToolCall(id="call-boom", name="boom", arguments={})
        )
        assert result.is_error is True
        assert result.content == "ValueError: kaboom"

    async def test_execute_stringifies_non_string_results(self) -> None:
        """Non-string return values are passed through ``repr``."""
        provider = FakeToolProvider()

        def numeric(args: Mapping[str, Any]) -> int:
            return 7

        provider.register(
            ToolSpec(name="seven", description="", parameters={}), numeric
        )

        result = await provider.execute(
            ToolCall(id="call-7", name="seven", arguments={})
        )
        assert result.content == "7"
        assert result.is_error is False


class TestToolExecutionError:
    """The transport-level exception."""

    def test_carries_message_only_by_default(self) -> None:
        """The minimal ctor uses None for both optional fields."""
        err = ToolExecutionError("something went wrong")
        assert str(err) == "something went wrong"
        assert err.provider is None
        assert err.call is None

    def test_carries_provider_and_call_when_supplied(self) -> None:
        """Optional ``provider`` and ``call`` kwargs are stored verbatim."""
        call = ToolCall(id="x", name="boom", arguments={})
        err = ToolExecutionError("nope", provider="minimax", call=call)
        assert err.provider == "minimax"
        assert err.call is call

    def test_is_an_exception_subclass(self) -> None:
        """``ToolExecutionError`` is catchable as a generic Exception."""
        err: Exception = ToolExecutionError("x")
        assert isinstance(err, Exception)


# ===========================================================================
# transport.py — FakeTransport + BaseTransport helpers
# ===========================================================================


class _RecordingHandler:
    """Reusable handler that records every (kind, payload) it receives.

    Used for both happy-path (handler bound) and "no handler"
    (dispatch returns silently) testing.
    """

    def __init__(self) -> None:
        self.received: list[tuple[str, Any]] = []

    async def __call__(self, kind: str, payload: Any) -> None:
        self.received.append((kind, payload))


class TestFakeTransportBindUnbind:
    """The bind/unbind/dispatch machinery, exercised on the concrete fake.

    ``BaseTransport`` is abstract (inherits ``Transport.on_*`` ABCs), so we
    exercise the same machinery through :class:`FakeTransport`, which is
    what concrete harnesses are expected to subclass anyway. The
    ``bind``/``unbind``/``_dispatch`` methods are inherited unchanged.
    """

    async def test_handler_is_none_unless_bound(self) -> None:
        """A fresh FakeTransport has no handler attached."""
        transport = FakeTransport(name="t1")
        assert transport._handler is None  # noqa: SLF001 — internal check

    async def test_bind_attaches_handler(self) -> None:
        """``bind`` stores the handler so ``_dispatch`` can call it."""
        transport = FakeTransport(name="t1")
        handler: EventHandler = _RecordingHandler()
        transport.bind(handler)
        assert transport._handler is handler  # noqa: SLF001

    async def test_unbind_clears_handler(self) -> None:
        """``unbind`` resets the handler to None."""
        transport = FakeTransport(name="t1")
        transport.bind(_RecordingHandler())
        transport.unbind()
        assert transport._handler is None  # noqa: SLF001

    async def test_unbind_is_idempotent(self) -> None:
        """Calling unbind on a never-bound transport is a no-op."""
        transport = FakeTransport(name="t1")
        transport.unbind()
        transport.unbind()  # twice, just to be sure
        assert transport._handler is None  # noqa: SLF001

    async def test_dispatch_skips_when_no_handler(self) -> None:
        """With no handler, ``_dispatch`` returns silently (no exception)."""
        transport = FakeTransport(name="t1")
        # The whole point: this must not raise.
        await transport._dispatch("anything", {"k": "v"})  # noqa: SLF001

    async def test_dispatch_calls_bound_handler(self) -> None:
        """A bound handler is invoked with (kind, payload)."""
        transport = FakeTransport(name="t1")
        handler = _RecordingHandler()
        transport.bind(handler)
        await transport._dispatch("ping", {"value": 42})  # noqa: SLF001
        assert handler.received == [("ping", {"value": 42})]


class TestFakeTransportHistory:
    """``FakeTransport`` records every event it sees."""

    def test_default_name_is_fake(self) -> None:
        """The fake's default name lets the test-helper tests stay terse."""
        transport = FakeTransport()
        assert transport.name == "fake"

    async def test_history_starts_empty(self) -> None:
        """No events have been recorded on a fresh fake."""
        transport = FakeTransport()
        assert transport.history == []

    async def test_on_stt_result_records_and_dispatches(self) -> None:
        """``on_stt_result`` records the payload and forwards to the handler."""
        transport = FakeTransport()
        handler = _RecordingHandler()
        transport.bind(handler)
        await transport.on_stt_result("hi", confidence=0.9)
        assert transport.history == [
            ("stt_result", {"text": "hi", "confidence": 0.9})
        ]
        assert handler.received == [
            ("stt_result", {"text": "hi", "confidence": 0.9})
        ]

    async def test_on_vad_records_and_dispatches(self) -> None:
        """``on_vad`` records the event verbatim and forwards to the handler."""
        transport = FakeTransport()
        handler = _RecordingHandler()
        transport.bind(handler)
        event = VadEvent(is_speech=True, confidence=0.7)
        await transport.on_vad(event)
        assert transport.history == [("vad", event)]
        assert handler.received == [("vad", event)]

    async def test_on_telegram_update_records_and_dispatches(self) -> None:
        """``on_telegram_update`` records the value object and forwards it."""
        transport = FakeTransport()
        handler = _RecordingHandler()
        transport.bind(handler)
        update = TelegramUpdate(
            update_id=1, kind="message", payload={"text": "hi"}
        )
        await transport.on_telegram_update(update)
        assert transport.history == [("telegram_update", update)]
        assert handler.received == [("telegram_update", update)]

    async def test_on_key_event_records_and_dispatches(self) -> None:
        """``on_key_event`` records the event and forwards it."""
        transport = FakeTransport()
        handler = _RecordingHandler()
        transport.bind(handler)
        event = KeyEvent(code="space", pressed=True)
        await transport.on_key_event(event)
        assert transport.history == [("key_event", event)]
        assert handler.received == [("key_event", event)]


class TestFakeTransportFeedHelpers:
    """The ``feed_*`` helpers are the ergonomic API for tests."""

    async def test_feed_stt_calls_on_stt_result(self) -> None:
        """``feed_stt`` delegates to ``on_stt_result`` with the same args."""
        transport = FakeTransport()
        await transport.feed_stt("hello", confidence=0.5)
        assert transport.history == [
            ("stt_result", {"text": "hello", "confidence": 0.5})
        ]

    async def test_feed_stt_default_confidence_is_one(self) -> None:
        """The confidence default must be 1.0 (full trust)."""
        transport = FakeTransport()
        await transport.feed_stt("hi")
        assert transport.history == [
            ("stt_result", {"text": "hi", "confidence": 1.0})
        ]

    async def test_feed_vad_wraps_into_event(self) -> None:
        """``feed_vad`` builds a ``VadEvent`` and forwards it."""
        transport = FakeTransport()
        await transport.feed_vad(is_speech=True, confidence=0.6)
        assert transport.history == [
            ("vad", VadEvent(is_speech=True, confidence=0.6))
        ]

    async def test_feed_vad_default_confidence_is_one(self) -> None:
        """The VAD confidence default is 1.0."""
        transport = FakeTransport()
        await transport.feed_vad(is_speech=False)
        assert transport.history == [
            ("vad", VadEvent(is_speech=False, confidence=1.0))
        ]

    async def test_feed_key_wraps_into_event(self) -> None:
        """``feed_key`` builds a ``KeyEvent`` and forwards it."""
        transport = FakeTransport()
        await transport.feed_key("enter", pressed=False)
        assert transport.history == [
            ("key_event", KeyEvent(code="enter", pressed=False))
        ]

    async def test_feed_key_default_pressed_is_true(self) -> None:
        """The default is a key-press (not a release)."""
        transport = FakeTransport()
        await transport.feed_key("a")
        assert transport.history == [
            ("key_event", KeyEvent(code="a", pressed=True))
        ]

    async def test_feed_telegram_forwards_update(self) -> None:
        """``feed_telegram`` forwards the pre-built ``TelegramUpdate``."""
        transport = FakeTransport()
        update = TelegramUpdate(
            update_id=42, kind="callback", payload={"data": "ok"}
        )
        await transport.feed_telegram(update)
        assert transport.history == [("telegram_update", update)]


class TestFakeTransportNoHandlerBound:
    """Even without a bound handler, recording continues."""

    async def test_no_handler_does_not_break_recording(self) -> None:
        """Calling ``on_*`` with no handler must still record the event."""
        transport = FakeTransport()
        await transport.feed_stt("alone")
        assert transport.history == [
            ("stt_result", {"text": "alone", "confidence": 1.0})
        ]

    async def test_mixed_events_keep_history_in_order(self) -> None:
        """A sequence of mixed events must be recorded in arrival order."""
        transport = FakeTransport()
        await transport.feed_stt("one")
        await transport.feed_vad(is_speech=True)
        await transport.feed_key("space")
        await transport.feed_telegram(
            TelegramUpdate(update_id=1, kind="message", payload={})
        )
        assert [kind for kind, _ in transport.history] == [
            "stt_result",
            "vad",
            "key_event",
            "telegram_update",
        ]

    async def test_aclose_is_inherited_noop(self) -> None:
        """``aclose`` on a fake transport is a no-op (no resources)."""
        transport = FakeTransport()
        # Must not raise, must return None.
        result = await transport.aclose()
        assert result is None