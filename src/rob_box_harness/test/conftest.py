"""Pytest fixtures shared across the harness test suite.

Every test in this package builds on these:

* ``base_config`` — a minimal :class:`HarnessConfig` for an
  ``echo`` harness.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig


@pytest.fixture
def base_config() -> HarnessConfig:
    """A minimal valid config for the echo harness."""
    return HarnessConfig.from_dict(
        {"harness": {"kind": "echo", "name": "test_echo"}}
    )
