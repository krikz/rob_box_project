"""Pytest fixtures shared across the harness test suite.

Every test in this package builds on these:

* ``base_config`` — a minimal :class:`HarnessConfig` for an
  ``echo`` harness.
* ``reset_runner_singletons`` — clears the runner's default
  registry and the factory's instance cache between tests so a
  test that mutates either doesn't leak state into the next one.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.registry import HarnessFactory
from rob_box_harness.runner import reset_default_registry


@pytest.fixture
def base_config() -> HarnessConfig:
    """A minimal valid config for the echo harness."""
    return HarnessConfig.from_dict(
        {"harness": {"kind": "echo", "name": "test_echo"}}
    )


@pytest.fixture(autouse=True)
def reset_runner_singletons() -> None:
    """Reset the runner's default registry + factory cache between tests."""
    reset_default_registry()
    HarnessFactory.reset_cache()
    yield
    reset_default_registry()
    HarnessFactory.reset_cache()
