"""Pytest configuration for the top-level ``tests/unit/harness`` suite.

Why this directory exists
-------------------------

The project's canonical pytest root is ``src/rob_box_harness/test``
(see ``pytest.ini``'s ``testpaths``). That suite covers the framework
*implementation* (Echo / Upper harnesses, MiniMax wrappers, etc.).

This sibling tree — ``tests/unit/harness/`` — is reserved for
**scenario-driven** tests that exercise the public surface from an
outsider's perspective:

* tests use fake harnesses defined inline, not the bundled ones;
* tests assert behaviour contracts ("calling X raises Y", "after
  unregister, lookup must fail with the same exception type as before
  registration");
* tests are intentionally network-free — no MiniMax key, no socket.

The conftest below provides three reusable pieces:

* ``FakeHarness`` — a tiny harness class that records every
  ``run`` invocation, used as the "provider class" in registration
  tests. Subclassing :class:`Harness` is intentional: the registry
  contract says builders must return ``Harness[Any]``, so fakes must
  satisfy the same type.
* ``fake_builder`` — a free function matching
  :data:`rob_box_harness.registry.HarnessBuilder`, used to verify the
  registry works with any callable, not just classes.
* ``clean_registry`` — a fixture that gives each test a fresh
  ``HarnessRegistry`` AND wipes the ``HarnessFactory`` cache so a
  builder-call leak from one test cannot poison another.
* ``base_config`` — a minimal :class:`HarnessConfig` for the echo
  harness name (the ``kind`` field is purely cosmetic here — fakes
  don't actually dispatch to it).
"""

from __future__ import annotations

from typing import Any

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.harness import Harness, HarnessRunResult
from rob_box_harness.registry import HarnessFactory, HarnessRegistry


class FakeHarness(Harness[Any]):
    """Tiny in-memory harness used as a fake "provider" in registry tests.

    The base :meth:`Harness.run` requires the harness to be
    ``init()``-ed first and returns a :class:`HarnessRunResult`. We
    keep that contract so the fake fits into the type system, but we
    don't drive ``run`` from registry tests directly — the registry
    tests only care that the builder was invoked and produced a
    properly-typed instance.

    :meth:`step` is abstract on :class:`Harness`; we implement it as
    a no-op because registry tests don't exercise the runtime, only
    construction + identity.
    """

    def __init__(self, config: HarnessConfig) -> None:
        super().__init__(config)

    async def step(self, input_data: Any) -> Any:
        # Body is intentionally trivial: registry tests assert on
        # construction, not on runtime semantics. Anything that
        # runs against this fake would have to be a separate test.
        return None


def fake_builder(config: HarnessConfig) -> FakeHarness:
    """Plain-function builder (NOT a class) — proves registry accepts callables."""
    return FakeHarness(config)


@pytest.fixture
def clean_registry() -> HarnessRegistry:
    """Return a fresh registry AND wipe the factory cache.

    Using this fixture isolates each test from prior runs: the registry
    is empty on entry and exits, and the process-wide
    ``HarnessFactory._cache`` is reset both before and after the test,
    so cached instances from a previous test cannot leak in.
    """
    HarnessFactory.reset_cache()
    yield HarnessRegistry()
    HarnessFactory.reset_cache()


@pytest.fixture
def fake_harness_class() -> type[FakeHarness]:
    """Expose :class:`FakeHarness` as a fixture for tests that prefer it."""
    return FakeHarness


@pytest.fixture
def base_config() -> HarnessConfig:
    """A minimal valid config; the ``kind`` is irrelevant to fake harnesses."""
    return HarnessConfig.from_dict({"harness": {"kind": "fake", "name": "test_fake"}})
