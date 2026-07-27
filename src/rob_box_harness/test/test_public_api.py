"""Tests for the supported top-level ``rob_box_harness`` API."""

from __future__ import annotations

import pytest

import rob_box_harness as harness_api
from rob_box_harness.harnesses.echo import EchoHarness


@pytest.mark.parametrize(
    "name",
    [
        "Harness",
        "HarnessConfig",
        "LifecycleHooks",
        "HarnessRegistry",
        "HarnessFactory",
        "run_harness",
        "run_harness_sync",
        "DummyLLMProvider",
        "HarnessError",
        "HarnessNotFoundError",
        "ProviderNotFoundError",
    ],
)
def test_documented_symbols_are_exported(name: str) -> None:
    assert name in harness_api.__all__
    assert getattr(harness_api, name) is not None


@pytest.mark.asyncio
async def test_public_api_can_register_build_and_run_custom_registry() -> None:
    registry = harness_api.HarnessRegistry()
    registry.register("custom", lambda config: EchoHarness(config))
    config = harness_api.HarnessConfig.from_dict(
        {"harness": {"kind": "echo", "name": "public_api"}}
    )

    result = await harness_api.run_harness(
        "custom",
        {"message": "hello"},
        config,
        registry=registry,
    )

    assert result.output == "echo: hello"
    assert result.metadata == {"harness": "echo"}


def test_framework_version_is_public_and_parseable() -> None:
    parts = harness_api.__version__.split(".")

    assert len(parts) == 3
    assert all(part.isdigit() for part in parts)
