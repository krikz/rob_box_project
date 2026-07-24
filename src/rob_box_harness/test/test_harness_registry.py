"""Tests for the harness registry and factory.

Covers:

* ``register`` / ``resolve`` / ``unregister`` semantics.
* ``register_builtin_harnesses`` registers the two built-ins.
* ``HarnessFactory`` caches instances per ``(name, config_hash)``.
* Unknown names raise ``HarnessNotFoundError``.
* Duplicate names raise ``ValueError``.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.errors import HarnessNotFoundError
from rob_box_harness.harness import Harness
from rob_box_harness.harnesses.echo import EchoHarness
from rob_box_harness.harnesses.upper import UpperHarness
from rob_box_harness.registry import (
    HarnessFactory,
    HarnessRegistry,
    register_builtin_harnesses,
)


def test_register_and_resolve() -> None:
    """A registered builder is resolvable by name."""
    registry = HarnessRegistry()
    registry.register("echo", lambda config: EchoHarness(config))
    builder = registry.resolve("echo")
    assert callable(builder)


def test_register_duplicate_raises() -> None:
    """Re-registering a name raises ``ValueError``."""
    registry = HarnessRegistry()
    registry.register("echo", lambda config: EchoHarness(config))
    with pytest.raises(ValueError, match="already registered"):
        registry.register("echo", lambda config: EchoHarness(config))


def test_resolve_unknown_raises() -> None:
    """Unknown names raise ``HarnessNotFoundError``."""
    registry = HarnessRegistry()
    with pytest.raises(HarnessNotFoundError, match="unknown harness"):
        registry.resolve("nope")


def test_unregister_is_no_op_for_unknown() -> None:
    """Unregistering an unknown name is a silent no-op."""
    registry = HarnessRegistry()
    registry.unregister("nope")  # no exception
    assert registry.names() == []


def test_names_returns_sorted_list() -> None:
    """``names()`` returns a sorted list of registered names."""
    registry = HarnessRegistry()
    registry.register("zeta", lambda config: EchoHarness(config))
    registry.register("alpha", lambda config: EchoHarness(config))
    assert registry.names() == ["alpha", "zeta"]


def test_register_builtin_harnesses() -> None:
    """The built-in registration adds ``echo`` and ``upper``."""
    registry = register_builtin_harnesses()
    assert "echo" in registry.names()
    assert "upper" in registry.names()


def test_register_builtin_harnesses_returns_registry() -> None:
    """Calling without a registry returns a fresh one."""
    registry = register_builtin_harnesses()
    assert isinstance(registry, HarnessRegistry)


def test_harness_factory_caches_instances() -> None:
    """Calling ``create`` twice with the same config returns the same instance."""
    registry = register_builtin_harnesses()
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    first = HarnessFactory.create("echo", config, registry)
    second = HarnessFactory.create("echo", config, registry)
    assert first is second


def test_harness_factory_reset_cache() -> None:
    """``reset_cache`` forces a fresh instance on the next call."""
    registry = register_builtin_harnesses()
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    first = HarnessFactory.create("echo", config, registry)
    HarnessFactory.reset_cache()
    second = HarnessFactory.create("echo", config, registry)
    assert first is not second


def test_harness_factory_unknown_raises() -> None:
    """``create`` with an unknown name raises ``HarnessNotFoundError``."""
    registry = register_builtin_harnesses()
    config = HarnessConfig.from_dict({"harness": {"kind": "echo"}})
    with pytest.raises(HarnessNotFoundError, match="nonexistent"):
        HarnessFactory.create("nonexistent", config, registry)


def test_factory_resolves_to_concrete_subclass() -> None:
    """``create`` returns an instance of the registered harness class."""
    registry = register_builtin_harnesses()
    config = HarnessConfig.from_dict({"harness": {"kind": "upper"}})
    instance = HarnessFactory.create("upper", config, registry)
    assert isinstance(instance, UpperHarness)
    assert isinstance(instance, Harness)
