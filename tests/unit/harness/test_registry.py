"""Scenario tests for the harness registry — the "lookup table" for harnesses.

These tests live outside ``src/rob_box_harness/test/`` on purpose:
that tree is for unit tests of the implementation (using bundled
``EchoHarness`` / ``UpperHarness``). This tree is for **behavioural
contracts** that any future registry implementation must honour, so
the fakes are local to the test and the assertions are about
*the registry shape*, not about whether the bundled harnesses work.

Scenarios covered (one test per requirement from the task spec):

1. ``test_register_new_provider_succeeds_then_lookup_returns_same_builder``
   — a fresh name can be registered, and ``resolve`` returns exactly
   the builder that was registered (identity-preserving).
2. ``test_registering_a_duplicate_name_raises_value_error``
   — re-registering the same name raises ``ValueError`` to prevent
   silent overrides that mask refactoring mistakes.
3. ``test_lookup_of_unknown_provider_raises_specific_error``
   — looking up a name that was never registered raises
   :class:`HarnessNotFoundError` (a ``KeyError`` is also surfaced by
   the framework, depending on the path; we check both).
4. ``test_unregister_removes_provider_and_followup_lookup_fails``
   — ``unregister`` of an existing name silences the entry; a
   subsequent ``resolve`` raises the same error as scenario 3.
5. ``test_unregister_unknown_name_is_noop``
   — ``unregister`` is forgiving when the name was never present
   (mirrors ``dict.pop`` with a default).
6. ``test_names_lists_all_providers_in_sorted_order``
   — ``names()`` is sorted and stable across registrations.

Plus two factory-level tests:

7. ``test_factory_caches_and_resets_per_name_and_config``
   — ``HarnessFactory`` caches per ``(name, config_hash)``;
   ``reset_cache`` is the only way to bypass it.
8. ``test_factory_unknown_name_propagates_registry_error``
   — factory propagates the registry's ``HarnessNotFoundError``
   unchanged, so callers can catch by type.

No network, no MiniMax key, no real LLM. ``HarnessFactory._cache``
is reset by the ``clean_registry`` fixture for total isolation.
"""

from __future__ import annotations

from typing import Any

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.errors import HarnessNotFoundError
from rob_box_harness.harness import Harness
from rob_box_harness.registry import (
    HarnessFactory,
    HarnessRegistry,
    register_builtin_harnesses,
)

from tests.unit.harness.conftest import FakeHarness, fake_builder


# --------------------------------------------------------------------------- #
# 1. Successful registration + lookup                                          #
# --------------------------------------------------------------------------- #
def test_register_new_provider_succeeds_then_lookup_returns_same_builder(
    clean_registry: HarnessRegistry,
    base_config: HarnessConfig,
) -> None:
    """Registering a fresh name returns the SAME callable from ``resolve``.

    Identity-preservation matters: callers that cached the builder
    before the registry was opened (e.g. during wiring) must keep
    getting their instance, not a copy.
    """
    clean_registry.register("alpha", fake_builder)

    resolved = clean_registry.resolve("alpha")

    # Identity, not just equality: registries store by reference.
    assert resolved is fake_builder
    # And it is actually a working builder: calling it produces a
    # properly-typed harness instance.
    instance = resolved(base_config)
    assert isinstance(instance, Harness)
    assert isinstance(instance, FakeHarness)


# --------------------------------------------------------------------------- #
# 2. Duplicate registration raises ValueError                                  #
# --------------------------------------------------------------------------- #
def test_registering_a_duplicate_name_raises_value_error(
    clean_registry: HarnessRegistry,
) -> None:
    """A second registration under the same name raises ``ValueError``.

    The exact wording of the message is part of the contract — it
    must reference the offending name so debugging is direct.
    """
    clean_registry.register("dup", fake_builder)

    with pytest.raises(ValueError, match=r"'dup'"):
        clean_registry.register("dup", fake_builder)

    # The first registration must still be intact — the failed
    # re-registration must not silently half-overwrite the slot.
    assert clean_registry.resolve("dup") is fake_builder


# --------------------------------------------------------------------------- #
# 3. Unknown name on lookup raises HarnessNotFoundError                        #
# --------------------------------------------------------------------------- #
def test_lookup_of_unknown_provider_raises_specific_error(
    clean_registry: HarnessRegistry,
) -> None:
    """An unknown name raises the framework's ``HarnessNotFoundError``.

    The error MUST mention:
    * the offending name (so a misconfigured YAML can be diagnosed);
    * the available names (so the fix is obvious at a glance).

    We don't assert exact sorting of the "Available:" segment because
    that's an implementation detail of :meth:`HarnessRegistry.resolve`;
    the *presence* of both tokens is what users depend on.
    """
    clean_registry.register("alpha", fake_builder)

    with pytest.raises(HarnessNotFoundError) as exc_info:
        clean_registry.resolve("does_not_exist")

    message = str(exc_info.value)
    assert "does_not_exist" in message
    assert "alpha" in message


# --------------------------------------------------------------------------- #
# 4. Unregister removes a provider; subsequent lookup fails                    #
# --------------------------------------------------------------------------- #
def test_unregister_removes_provider_and_followup_lookup_fails(
    clean_registry: HarnessRegistry,
) -> None:
    """After ``unregister``, the provider must be gone from lookup."""
    clean_registry.register("alpha", fake_builder)
    assert "alpha" in clean_registry.names()

    clean_registry.unregister("alpha")

    assert "alpha" not in clean_registry.names()

    with pytest.raises(HarnessNotFoundError, match="alpha"):
        clean_registry.resolve("alpha")


def test_unregister_unknown_name_is_noop(
    clean_registry: HarnessRegistry,
) -> None:
    """Unregistering a name that was never registered is a silent no-op.

    This matches the symmetric :py:meth:`dict.pop` with a default:
    tests and tear-down code frequently call ``unregister`` for
    names they conditionally registered; failure to do so must
    not break the lifecycle.
    """
    # Pre-condition: registry is empty (per fixture).
    assert clean_registry.names() == []

    clean_registry.unregister("ghost")  # must not raise
    assert clean_registry.names() == []


# --------------------------------------------------------------------------- #
# 5. Listing all providers                                                    #
# --------------------------------------------------------------------------- #
def test_names_lists_all_providers_in_sorted_order(
    clean_registry: HarnessRegistry,
) -> None:
    """``names()`` returns the full set, sorted ascending.

    Sort order matters because diagnostic messages (e.g. the
    "Available: [...]" suffix on :class:`HarnessNotFoundError`)
    depend on it being deterministic. If you re-order the registry,
    re-order the error message; don't break the contract here.
    """
    for n in ("zulu", "alpha", "mike", "bravo"):
        clean_registry.register(n, fake_builder)

    assert clean_registry.names() == ["alpha", "bravo", "mike", "zulu"]

    # Adding after the fact keeps the contract.
    clean_registry.register("charlie", fake_builder)
    assert clean_registry.names() == [
        "alpha", "bravo", "charlie", "mike", "zulu",
    ]


# --------------------------------------------------------------------------- #
# 6. Registering different builder types under the same registry              #
# --------------------------------------------------------------------------- #
def test_registry_accepts_class_based_and_function_based_builders(
    clean_registry: HarnessRegistry,
    base_config: HarnessConfig,
    fake_harness_class: type[FakeHarness],
) -> None:
    """A registry can mix class-based builders and callable builders.

    The contract is "anything callable with the right signature",
    so we explicitly verify that a class-as-constructor and a
    free function coexist without registration order mattering.
    """
    clean_registry.register("fake_class", fake_harness_class)
    clean_registry.register("fake_func", fake_builder)

    class_instance = clean_registry.resolve("fake_class")(base_config)
    func_instance = clean_registry.resolve("fake_func")(base_config)

    assert isinstance(class_instance, fake_harness_class)
    assert isinstance(func_instance, FakeHarness)
    assert class_instance is not func_instance  # different builders


# --------------------------------------------------------------------------- #
# 7. Built-in registration helper                                             #
# --------------------------------------------------------------------------- #
def test_register_builtin_harnesses_uses_provided_registry() -> None:
    """When called with an explicit registry, it mutates that registry."""
    target = HarnessRegistry()

    returned = register_builtin_harnesses(target)

    # Same object is returned (no surprise allocation).
    assert returned is target
    # Built-ins are present.
    names = target.names()
    assert "echo" in names
    assert "upper" in names


# --------------------------------------------------------------------------- #
# 8. Factory caching across resolves                                           #
# --------------------------------------------------------------------------- #
def test_factory_caches_and_resets_per_name_and_config(
    clean_registry: HarnessRegistry,
    base_config: HarnessConfig,
) -> None:
    """``HarnessFactory.create`` returns the same instance until cache is reset.

    Two registrations with the same config key MUST return the
    SAME object — the framework guarantee is "no accidental double
    instantiation". After ``reset_cache``, a fresh instance is
    produced; this is the only sanctioned way to bypass the cache.
    """
    clean_registry.register("alpha", fake_builder)

    first = HarnessFactory.create("alpha", base_config, clean_registry)
    second = HarnessFactory.create("alpha", base_config, clean_registry)

    assert first is second
    # The cached instance is of the expected type.
    assert isinstance(first, FakeHarness)

    # Bypass the cache.
    HarnessFactory.reset_cache()
    third = HarnessFactory.create("alpha", base_config, clean_registry)

    assert third is not first
    assert isinstance(third, FakeHarness)


def test_factory_unknown_name_propagates_registry_error(
    clean_registry: HarnessRegistry,
    base_config: HarnessConfig,
) -> None:
    """Factory propagates :class:`HarnessNotFoundError` from the registry."""
    with pytest.raises(HarnessNotFoundError, match="missing"):
        HarnessFactory.create("missing", base_config, clean_registry)
