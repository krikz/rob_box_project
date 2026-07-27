"""Tests for the runner entry point and side-effect ports.

Covers:

* ``run_harness`` validates ``name`` (unknown harness raises).
* ``run_harness`` returns a :class:`HarnessRunResult`.
* ``run_harness_sync`` works without an event loop.
* The composite bus fans out to multiple downstream buses.
* NoopBus counts dispatched effects.
* RecordingBus records every effect for later assertion.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.effects import (
    CompositeBus,
    EchoEffect,
    NoopBus,
    RecordingBus,
    SideEffectBus,
)
from rob_box_harness.errors import HarnessNotFoundError
from rob_box_harness.runner import (
    get_default_registry,
    reset_default_registry,
    run_harness,
    run_harness_sync,
)


@pytest.mark.asyncio
async def test_run_harness_returns_result() -> None:
    """``run_harness`` returns a ``HarnessRunResult`` with the LLM output."""
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    result = await run_harness("echo", "hello", config)
    assert result.output == "echo: hello"
    assert result.metadata["harness"] == "echo"


def test_run_harness_sync_works() -> None:
    """``run_harness_sync`` is the synchronous wrapper for scripts."""
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    result = run_harness_sync("echo", "world", config)
    assert result.output == "echo: world"


@pytest.mark.asyncio
async def test_run_harness_unknown_raises() -> None:
    """An unknown harness name raises ``HarnessNotFoundError``."""
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    with pytest.raises(HarnessNotFoundError, match="nonexistent"):
        await run_harness("nonexistent", "hello", config)


@pytest.mark.asyncio
async def test_run_harness_uses_default_registry() -> None:
    """``run_harness`` resolves the default registry when none is passed."""
    registry = get_default_registry()
    assert "echo" in registry.names()


@pytest.mark.asyncio
async def test_run_harness_accepts_explicit_registry() -> None:
    """A custom registry can be passed in for test isolation."""
    from rob_box_harness.harnesses.echo import EchoHarness
    from rob_box_harness.registry import HarnessRegistry

    registry = HarnessRegistry()
    registry.register("echo", lambda config: EchoHarness(config))
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    result = await run_harness("echo", "hi", config, registry=registry)
    assert result.output == "echo: hi"


def test_reset_default_registry() -> None:
    """``reset_default_registry`` clears the default registry."""
    registry = get_default_registry()
    assert "echo" in registry.names()
    reset_default_registry()
    # After reset, the next call reconstructs the registry.
    registry = get_default_registry()
    assert "echo" in registry.names()


# ---------------------------------------------------------------------------
# Side-effect bus tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_noop_bus_counts_dispatches() -> None:
    """NoopBus increments its counter on every dispatch."""
    bus = NoopBus()
    assert bus.count == 0
    await bus.dispatch(EchoEffect(text="one"))
    await bus.dispatch(EchoEffect(text="two"))
    assert bus.count == 2


@pytest.mark.asyncio
async def test_recording_bus_records_effects() -> None:
    """RecordingBus appends every effect to its history."""
    bus = RecordingBus()
    await bus.dispatch(EchoEffect(text="a"))
    await bus.dispatch(EchoEffect(text="b"))
    assert [e.text for e in bus.effects] == ["a", "b"]


@pytest.mark.asyncio
async def test_recording_bus_reset() -> None:
    """``reset`` clears the history between tests."""
    bus = RecordingBus()
    await bus.dispatch(EchoEffect(text="x"))
    bus.reset()
    assert bus.effects == []


@pytest.mark.asyncio
async def test_composite_bus_fans_out() -> None:
    """CompositeBus dispatches to every downstream bus."""
    a, b = RecordingBus(), RecordingBus()
    composite = CompositeBus([a, b])
    await composite.dispatch(EchoEffect(text="hi"))
    assert [e.text for e in a.effects] == ["hi"]
    assert [e.text for e in b.effects] == ["hi"]


def test_composite_bus_requires_at_least_one_bus() -> None:
    """An empty composite raises ``ValueError`` at construction."""
    with pytest.raises(ValueError, match="at least one"):
        CompositeBus([])


@pytest.mark.asyncio
async def test_composite_bus_propagates_first_error() -> None:
    """If a downstream bus raises, the composite propagates the error."""

    class _RaisingBus(SideEffectBus):
        name = "raising"

        async def dispatch(self, effect: EchoEffect) -> None:
            raise RuntimeError("downstream failed")

    class _SilentBus(SideEffectBus):
        name = "silent"

        async def dispatch(self, effect: EchoEffect) -> None:
            self.received = effect  # type: ignore[attr-defined]

    silent = _SilentBus()
    composite = CompositeBus([_RaisingBus(), silent])
    with pytest.raises(RuntimeError, match="downstream"):
        await composite.dispatch(EchoEffect(text="hi"))
    # The silent bus never saw the effect because the first one raised.
    assert not hasattr(silent, "received")
