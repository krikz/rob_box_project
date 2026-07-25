"""Harness registry — name → builder map.

A :class:`HarnessRegistry` is the entry point for "I want a
harness named ``dialog``", so the same composition root can
power :func:`run_harness` and the CLI ``run_harness`` command.

Design notes:

* Build on top of the existing TTS registry pattern
  (:class:`rob_box_llm.tts_provider_registry.TTSProviderRegistry`)
  for consistency: explicit registration, no auto-discovery, builders
  are callables that take a :class:`HarnessConfig` and return a
  :class:`Harness`.
* Built-in harnesses (``echo``, ``upper``) are registered via
  :func:`register_builtin_harnesses`, mirroring
  :func:`rob_box_llm.tts_provider_registry.register_builtin_tts_providers`.
* The factory caches instances per ``(name, config_hash)`` so
  long-running processes don't accidentally open two HTTP pools
  for the same harness.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Callable, Mapping

from rob_box_harness.config import HarnessConfig
from rob_box_harness.errors import HarnessNotFoundError

if TYPE_CHECKING:
    from rob_box_harness.harness import Harness

logger = logging.getLogger(__name__)


# A builder takes a HarnessConfig and returns a Harness. The
# framework treats the return value as opaque after construction.
HarnessBuilder = Callable[[HarnessConfig], "Harness[Any]"]


class HarnessRegistry:
    """In-memory ``name → builder`` registry.

    NOT thread-safe for concurrent writes. Construct once at
    process start, then read-only for the rest of the lifetime.
    """

    def __init__(self) -> None:
        self._builders: dict[str, HarnessBuilder] = {}

    def register(self, name: str, builder: HarnessBuilder) -> None:
        """Register ``builder`` under ``name``.

        Raises ``ValueError`` if the name is already taken — silent
        overrides would mask refactoring mistakes.
        """
        if name in self._builders:
            raise ValueError(
                f"harness {name!r} already registered; "
                "duplicate registration is forbidden"
            )
        self._builders[name] = builder

    def resolve(self, name: str) -> HarnessBuilder:
        """Return the builder for ``name``. Raises :class:`HarnessNotFoundError`."""
        if name not in self._builders:
            raise HarnessNotFoundError(
                f"unknown harness: {name!r}. "
                f"Available: {sorted(self._builders)}",
                harness=name,
            )
        return self._builders[name]

    def unregister(self, name: str) -> None:
        """Remove a builder from the registry. Test-only helper."""
        self._builders.pop(name, None)

    def names(self) -> list[str]:
        """Return all registered harness names (sorted)."""
        return sorted(self._builders)


class HarnessFactory:
    """Cached harness factory.

    Instances are reused per ``(name, config_hash)`` so accidental
    double-instantiation in long-running processes doesn't leak
    ports. Tests can call :meth:`reset_cache` to start fresh.
    """

    _cache: dict[tuple[str, str], "Harness[Any]"] = {}

    @classmethod
    def create(
        cls,
        name: str,
        config: HarnessConfig,
        registry: HarnessRegistry,
    ) -> "Harness[Any]":
        """Resolve ``name`` in ``registry``, invoke the builder, cache, return.

        Raises :class:`HarnessNotFoundError` if ``name`` is unknown.
        Raises ``ValueError`` if the builder rejects ``config``.
        """
        config_hash = repr(_stable_config_hash(config))
        key = (name, config_hash)
        if key in cls._cache:
            return cls._cache[key]
        builder = registry.resolve(name)
        instance = builder(config)
        cls._cache[key] = instance
        return instance

    @classmethod
    def reset_cache(cls) -> None:
        """Clear the instance cache. Test-only helper."""
        cls._cache.clear()


def register_builtin_harnesses(registry: HarnessRegistry | None = None) -> HarnessRegistry:
    """Register the built-in dummy harnesses under their canonical names.

    Currently registers:

    * ``"echo"``  — Round-trips the input through the LLM and
      returns the LLM's response verbatim.
    * ``"upper"`` — Same as ``echo`` but uppercases the response.

    Real harnesses (Dialog / Persistent / Telegram) will be added
    here as their implementations land in ``rob_box_harness.harnesses``.
    """
    if registry is None:
        registry = HarnessRegistry()

    def _echo_builder(config: HarnessConfig) -> "Harness[Any]":
        from rob_box_harness.harnesses.echo import EchoHarness

        return EchoHarness(config)

    def _upper_builder(config: HarnessConfig) -> "Harness[Any]":
        from rob_box_harness.harnesses.upper import UpperHarness

        return UpperHarness(config)

    registry.register("echo", _echo_builder)
    registry.register("upper", _upper_builder)

    return registry


def _stable_config_hash(config: HarnessConfig) -> Mapping[str, Any]:
    """Return a JSON-serialisable dict with ``config``'s meaningful fields.

    The factory uses this to know when two configs are "the same"
    for caching purposes. We deliberately exclude ``LoggingConfig``:
    changing the log level should not invalidate a cached harness.
    """
    return {
        "harness": config.harness,
        "name": config.name,
        "state": dict(config.state),
        "llm": config.llm.__dict__ if config.llm is not None else None,
        "tools": config.tools.__dict__ if config.tools is not None else None,
        "memory": config.memory.__dict__ if config.memory is not None else None,
        "effects": config.effects.__dict__ if config.effects is not None else None,
        "transport": config.transport.__dict__ if config.transport is not None else None,
        "tts": config.tts.__dict__ if config.tts is not None else None,
    }


__all__ = [
    "HarnessBuilder",
    "HarnessRegistry",
    "HarnessFactory",
    "register_builtin_harnesses",
]
