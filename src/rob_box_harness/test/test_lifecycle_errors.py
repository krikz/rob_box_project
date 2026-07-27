"""Contract tests for lifecycle hooks and provider failure propagation."""

from __future__ import annotations

from collections.abc import Iterable

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.errors import HarnessStateError, HookError
from rob_box_harness.harnesses.echo import EchoHarness
from rob_box_harness.lifecycle import LifecycleHooks
from rob_box_harness.providers.dummy import DummyLLMProvider
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMMessage, LLMResponse, LLMSettings


class _FailingProvider(DummyLLMProvider):
    """Offline provider double that fails before producing a response."""

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[dict] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        raise ProviderError("provider unavailable", provider="failing")


def _config() -> HarnessConfig:
    return HarnessConfig.from_dict({"harness": {"kind": "echo"}})


@pytest.mark.asyncio
async def test_sync_and_async_hooks_are_invoked_with_event_arguments() -> None:
    events: list[tuple[str, object | None]] = []

    def on_start() -> None:
        events.append(("start", None))

    async def on_turn_begin(value: object) -> None:
        events.append(("turn", value))

    hooks = LifecycleHooks(on_start=on_start, on_turn_begin=on_turn_begin)

    await hooks.invoke("on_start", "echo")
    await hooks.invoke("on_turn_begin", "echo", "hello")

    assert events == [("start", None), ("turn", "hello")]


def test_iter_hooks_returns_only_registered_hooks_in_lifecycle_order() -> None:
    def on_start() -> None:
        pass

    def on_stop() -> None:
        pass

    hooks = LifecycleHooks(on_stop=on_stop, on_start=on_start)

    assert list(hooks.iter_hooks()) == ["on_start", "on_stop"]


@pytest.mark.asyncio
async def test_harness_lifecycle_invokes_each_hook_once_in_order() -> None:
    events: list[str] = []

    def on_start() -> None:
        events.append("start")

    def on_turn_begin(_input: object) -> None:
        events.append("turn")

    def on_stop() -> None:
        events.append("stop")

    harness = EchoHarness(
        _config(),
        hooks=LifecycleHooks(
            on_start=on_start,
            on_turn_begin=on_turn_begin,
            on_stop=on_stop,
        ),
    )

    await harness.init()
    await harness.run("hello")
    await harness.teardown()

    assert events == ["start", "turn", "stop"]


@pytest.mark.asyncio
async def test_hook_exception_is_wrapped_with_context_and_cause() -> None:
    def broken_hook() -> None:
        raise ValueError("bad hook")

    hooks = LifecycleHooks(on_start=broken_hook)

    with pytest.raises(HookError, match="bad hook") as caught:
        await hooks.invoke("on_start", "echo")

    assert caught.value.harness == "echo"
    assert caught.value.hook == "on_start"
    assert isinstance(caught.value.__cause__, ValueError)


@pytest.mark.asyncio
async def test_existing_hook_error_is_not_double_wrapped() -> None:
    original = HookError("veto", harness="echo", hook="on_tool_call")

    def veto() -> None:
        raise original

    hooks = LifecycleHooks(on_tool_call=veto)

    with pytest.raises(HookError) as caught:
        await hooks.invoke("on_tool_call", "echo")

    assert caught.value is original


@pytest.mark.asyncio
async def test_missing_hook_is_a_noop() -> None:
    await LifecycleHooks().invoke("on_stop", "echo")


@pytest.mark.asyncio
async def test_init_rejects_an_active_run() -> None:
    harness = EchoHarness(_config())
    await harness.init()
    harness._running = True  # type: ignore[attr-defined]

    with pytest.raises(HarnessStateError, match="run.*progress"):
        await harness.init()

    harness._running = False  # type: ignore[attr-defined]
    await harness.teardown()


@pytest.mark.asyncio
async def test_teardown_rejects_an_active_run() -> None:
    harness = EchoHarness(_config())
    await harness.init()
    harness._running = True  # type: ignore[attr-defined]

    with pytest.raises(HarnessStateError, match="run.*progress"):
        await harness.teardown()

    harness._running = False  # type: ignore[attr-defined]
    await harness.teardown()


@pytest.mark.asyncio
async def test_provider_error_propagates_and_run_state_is_reset() -> None:
    provider = _FailingProvider()
    harness = EchoHarness(_config(), llm=provider)
    await harness.init()

    with pytest.raises(ProviderError, match="provider unavailable"):
        await harness.run("hello")

    assert harness.is_initialized is True
    assert harness.is_running is False
    assert harness.llm is provider


@pytest.mark.asyncio
async def test_context_manager_reports_provider_error_and_tears_down() -> None:
    errors: list[BaseException] = []

    async def on_error(error: BaseException) -> None:
        errors.append(error)

    provider = _FailingProvider()
    harness = EchoHarness(
        _config(),
        llm=provider,
        hooks=LifecycleHooks(on_error=on_error),
    )

    with pytest.raises(ProviderError) as caught:
        async with harness:
            await harness.run("hello")

    assert errors == [caught.value]
    assert harness.is_initialized is False
    assert harness.is_running is False


@pytest.mark.asyncio
async def test_teardown_remains_consistent_when_stop_hook_fails() -> None:
    def broken_stop() -> None:
        raise RuntimeError("cannot stop hook")

    harness = EchoHarness(
        _config(),
        hooks=LifecycleHooks(on_stop=broken_stop),
    )
    await harness.init()

    with pytest.raises(HookError, match="cannot stop hook"):
        await harness.teardown()

    assert harness.is_initialized is False
    assert harness.is_running is False


@pytest.mark.asyncio
async def test_on_error_hook_propagates_after_teardown() -> None:
    """A ``HookError`` raised by ``on_error`` must surface AFTER teardown.

    Contract (harness.py docstring + ``lifecycle.py:99-101``):
    hook errors are observable to the caller of ``async with``,
    not silently swallowed. ``__aexit__`` must still run
    ``teardown()`` so ports are closed before the exception is
    re-raised — that ordering is the bug this test pins down.

    We trigger the error path by giving the harness a provider
    that raises ``ProviderError`` on ``complete`` — the same
    shape ``test_context_manager_reports_provider_error_and_tears_down``
    uses. The DIFFERENCE here is that ``on_error`` itself blows up,
    which is exactly what the old code silently swallowed.
    """
    teardown_ran: list[bool] = []

    def broken_on_error(_error: BaseException) -> None:
        raise HookError("on_error broke", harness="echo", hook="on_error")

    # Subclass ``EchoHarness`` to record (without monkeypatching)
    # that ``teardown()`` runs before the ``HookError`` propagates.
    # We keep the real lifecycle by ``super()``-into ``teardown``.
    class _ObservingHarness(EchoHarness):
        async def teardown(self) -> None:  # type: ignore[override]
            await super().teardown()
            teardown_ran.append(True)

    provider = _FailingProvider()
    harness = _ObservingHarness(
        _config(),
        llm=provider,
        hooks=LifecycleHooks(on_error=broken_on_error),
    )

    with pytest.raises(HookError, match="on_error broke") as caught:
        async with harness:
            await harness.run("hello")

    # 1. The original HookError from on_error reached the caller.
    assert isinstance(caught.value, HookError)
    assert caught.value.hook == "on_error"
    # 2. teardown() ran BEFORE the raise — no resource leak.
    assert teardown_ran == [True], (
        "teardown() must run before HookError is re-raised "
        f"(got teardown_ran={teardown_ran!r})"
    )
    # 3. The original ``on_error`` raised HookError directly, which
    #    ``lifecycle.invoke`` re-raises without wrapping — so
    #    ``__cause__`` is None and the same instance surfaces to
    #    the caller (see ``test_existing_hook_error_is_not_double_wrapped``).
    assert caught.value.__cause__ is None
