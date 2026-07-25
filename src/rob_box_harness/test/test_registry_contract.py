"""Additional provider-registry behaviour tests."""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig, LoggingConfig
from rob_box_harness.harnesses.echo import EchoHarness
from rob_box_harness.registry import HarnessFactory, HarnessRegistry


@pytest.fixture
def echo_registry() -> HarnessRegistry:
    registry = HarnessRegistry()
    registry.register("echo", EchoHarness)
    return registry


def _config(*, name: str = "session", count: int = 0) -> HarnessConfig:
    return HarnessConfig.from_dict(
        {
            "harness": {
                "kind": "echo",
                "name": name,
                "state": {"count": count},
            }
        }
    )


def test_lookup_returns_the_registered_builder(echo_registry: HarnessRegistry) -> None:
    assert echo_registry.resolve("echo") is EchoHarness


def test_unregistered_error_lists_available_provider_names(
    echo_registry: HarnessRegistry,
) -> None:
    with pytest.raises(Exception) as caught:
        echo_registry.resolve("missing")

    message = str(caught.value)
    assert "missing" in message
    assert "echo" in message


def test_unregister_removes_provider_from_lookup(echo_registry: HarnessRegistry) -> None:
    echo_registry.unregister("echo")

    with pytest.raises(Exception, match="unknown harness"):
        echo_registry.resolve("echo")


def test_factory_uses_separate_instances_for_different_state(
    echo_registry: HarnessRegistry,
) -> None:
    first = HarnessFactory.create("echo", _config(count=1), echo_registry)
    second = HarnessFactory.create("echo", _config(count=2), echo_registry)

    assert first is not second


def test_logging_changes_do_not_invalidate_cached_instance(
    echo_registry: HarnessRegistry,
) -> None:
    first_config = _config()
    second_config = _config()
    object.__setattr__(
        second_config,
        "logging",
        LoggingConfig(level="DEBUG"),
    )

    first = HarnessFactory.create("echo", first_config, echo_registry)
    second = HarnessFactory.create("echo", second_config, echo_registry)

    assert first is second


def test_builder_exception_propagates_without_cache_pollution() -> None:
    calls = 0

    def failing_builder(config: HarnessConfig) -> EchoHarness:
        nonlocal calls
        calls += 1
        raise RuntimeError("builder failed")

    registry = HarnessRegistry()
    registry.register("broken", failing_builder)

    for _ in range(2):
        with pytest.raises(RuntimeError, match="builder failed"):
            HarnessFactory.create("broken", _config(), registry)

    assert calls == 2
