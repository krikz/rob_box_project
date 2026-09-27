"""Tests for the supported top-level ``rob_box_harness`` API."""

from __future__ import annotations

import pytest

import rob_box_harness as harness_api


@pytest.mark.parametrize(
    "name",
    [
        "HarnessConfig",
        "LifecycleHooks",
        "DummyLLMProvider",
        "HarnessError",
        "HarnessNotFoundError",
        "ProviderNotFoundError",
    ],
)
def test_documented_symbols_are_exported(name: str) -> None:
    assert name in harness_api.__all__
    assert getattr(harness_api, name) is not None


def test_framework_version_is_public_and_parseable() -> None:
    parts = harness_api.__version__.split(".")

    assert len(parts) == 3
    assert all(part.isdigit() for part in parts)
