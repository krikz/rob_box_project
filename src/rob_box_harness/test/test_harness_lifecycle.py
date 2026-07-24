"""Harness lifecycle tests: init / run / teardown / async-with.

Exercises the contract from ADR-0001 §2.3:

* ``__init__`` does no I/O.
* ``init`` is idempotent and creates default ports.
* ``run`` requires ``init`` and refuses to re-enter.
* ``teardown`` is idempotent and closes ports.
* ``async with`` guarantees ``teardown`` even on exception.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.effects import NoopBus
from rob_box_harness.errors import HarnessStateError
from rob_box_harness.harness import Harness
from rob_box_harness.harnesses.echo import EchoHarness
from rob_box_harness.memory import InMemoryStore
from rob_box_harness.providers.dummy import DummyLLMProvider
from rob_box_harness.tools import FakeToolProvider
from rob_box_harness.transport import FakeTransport


def _make_harness(config: HarnessConfig | None = None) -> EchoHarness:
    """Helper: build an EchoHarness with default ports (no I/O)."""
    return EchoHarness(config or HarnessConfig.from_dict({"harness": {"kind": "echo"}}))


def test_init_does_no_io() -> None:
    """``__init__`` only stores config; no default ports are constructed."""
    harness = _make_harness()
    # llm/tools/memory/transport are None until init() is called.
    assert harness.llm is None
    assert harness.tools is None
    assert harness.memory is None
    assert harness.transport is None
    assert harness.is_initialized is False


def test_init_creates_default_ports() -> None:
    """``init`` constructs default ports when none were supplied."""
    import asyncio

    harness = _make_harness()
    asyncio.run(harness.init())
    assert harness.is_initialized is True
    assert isinstance(harness.llm, DummyLLMProvider)
    assert isinstance(harness.tools, FakeToolProvider)
    assert isinstance(harness.memory, InMemoryStore)
    assert isinstance(harness.transport, FakeTransport)
    assert isinstance(harness.effects, NoopBus)


def test_init_is_idempotent() -> None:
    """Calling ``init`` twice is a no-op (idempotency per ADR-0001 §2.3)."""
    import asyncio

    harness = _make_harness()
    asyncio.run(harness.init())
    llm = harness.llm
    asyncio.run(harness.init())
    assert harness.llm is llm  # same instance, not re-created


def test_run_requires_init() -> None:
    """``run`` raises if ``init`` was not called."""
    import asyncio

    harness = _make_harness()
    with pytest.raises(HarnessStateError, match="before init"):
        asyncio.run(harness.run("hello"))


@pytest.mark.asyncio
async def test_run_rejects_overlap_via_state_guard() -> None:
    """``run`` raises ``HarnessStateError`` when invoked while another is active.

    We use a direct guard flip instead of an async re-entrant step
    call (an async re-entrancy would naturally propagate the inner
    exception via ``await`` before the outer can complete the check).
    """

    class _GuardedHarness(EchoHarness):
        pass

    harness = _GuardedHarness(HarnessConfig.from_dict({"harness": {"kind": "echo"}}))
    await harness.init()
    harness._running = True  # type: ignore[attr-defined]
    with pytest.raises(HarnessStateError, match="re-entered"):
        await harness.run("hello")


def test_teardown_is_idempotent() -> None:
    """``teardown`` can be called multiple times safely."""
    import asyncio

    harness = _make_harness()
    asyncio.run(harness.init())
    asyncio.run(harness.teardown())
    asyncio.run(harness.teardown())  # no exception
    assert harness.is_initialized is False


@pytest.mark.asyncio
async def test_teardown_closes_ports() -> None:
    """``teardown`` calls ``aclose`` on every port that has it."""

    class _TrackingBus(NoopBus):
        def __init__(self) -> None:
            super().__init__()
            self.closed = False

        async def aclose(self) -> None:
            self.closed = True

    bus = _TrackingBus()
    harness = _make_harness()
    harness.effects = bus
    await harness.init()
    await harness.teardown()
    assert bus.closed is True


@pytest.mark.asyncio
async def test_async_with_teardown_on_exception() -> None:
    """``async with`` guarantees ``teardown`` even when ``step`` raises."""

    class _BoomHarness(EchoHarness):
        async def step(self, input_data: str) -> str:  # type: ignore[override]
            raise RuntimeError("boom")

    class _TrackingBus(NoopBus):
        def __init__(self) -> None:
            super().__init__()
            self.closed = False

        async def aclose(self) -> None:
            self.closed = True

    bus = _TrackingBus()
    harness = _BoomHarness(HarnessConfig.from_dict({"harness": {"kind": "echo"}}))
    harness.effects = bus
    with pytest.raises(RuntimeError, match="boom"):
        async with harness:
            await harness.run("hello")
    assert harness.is_initialized is False
    assert bus.closed is True


@pytest.mark.asyncio
async def test_extensions_recorded_after_run() -> None:
    """The harness stores ``last_input`` / ``last_output`` in extensions."""
    harness = _make_harness()
    await harness.init()
    result = await harness.run("hello")
    assert result.output == "echo: hello"
    snapshot = harness.snapshot()
    assert snapshot.extensions["last_input"] == "hello"
    assert snapshot.extensions["last_output"] == "echo: hello"


@pytest.mark.asyncio
async def test_state_is_initialised_from_config() -> None:
    """``_initial_state`` defaults to a copy of ``config.state``."""
    config = HarnessConfig.from_dict(
        {
            "harness": {
                "kind": "echo",
                "state": {"wake_active": True, "count": 3},
            }
        }
    )
    harness = _make_harness(config)
    await harness.init()
    assert harness.state["wake_active"] is True
    assert harness.state["count"] == 3


def test_init_validates_config_type() -> None:
    """Passing a non-HarnessConfig raises ``HarnessError``."""
    with pytest.raises(Exception):  # HarnessError or TypeError
        EchoHarness("not a config")  # type: ignore[arg-type]
