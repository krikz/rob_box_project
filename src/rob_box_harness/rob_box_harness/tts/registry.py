"""Harness-side TTS provider registry.

Mirrors the structure of :mod:`rob_box_harness.registry` (harness
registry) and the upstream :class:`rob_box_llm.tts_provider_registry.TTSProviderRegistry`,
but lives in the harness so the P0 framework can wire a
:class:`TTSProvider` into harnesses (Dialog/Persistent/Telegram) that
publish audio side-effects.

Design notes:

* In-memory ``name → builder`` map, like the upstream registry.
* Built-in providers are registered via
  :func:`register_builtin_tts_providers`, mirroring
  :func:`rob_box_llm.tts_provider_registry.register_builtin_tts_providers`
  and :func:`rob_box_harness.registry.register_builtin_harnesses`.
* The factory caches instances per ``(name, config_hash)`` so
  long-running processes don't accidentally open two HTTP pools
  for the same provider.
* No auto-discovery via ``importlib.metadata.entry_points()`` —
  ADR-0004 §2.3 explicitly rejects this for the same reasons as the
  upstream registry: implicit side-effects, hard to test, hidden
  dependencies at import time.
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Mapping

from rob_box_harness.config import TTSConfig
from rob_box_harness.errors import ProviderNotFoundError

_log = logging.getLogger(__name__)


# A builder takes a :class:`TTSConfig` and returns a TTSProvider
# (any subclass of :class:`rob_box_llm.tts.TTSProvider`). The
# framework treats the return value as opaque after construction.
TTSBuilder = Callable[[TTSConfig], Any]


class TTSProviderRegistry:
    """In-memory ``name → builder`` registry for TTS providers.

    NOT thread-safe for concurrent writes. Construct once at process
    start, then read-only for the rest of the lifetime.
    """

    def __init__(self) -> None:
        self._builders: dict[str, TTSBuilder] = {}

    def register(self, name: str, builder: TTSBuilder) -> None:
        """Register ``builder`` under ``name``.

        Raises ``ValueError`` if the name is already taken — silent
        overrides would mask refactoring mistakes.
        """
        if name in self._builders:
            raise ValueError(
                f"harness TTS provider {name!r} already registered; "
                "duplicate registration is forbidden"
            )
        self._builders[name] = builder

    def resolve(self, name: str) -> TTSBuilder:
        """Return the builder for ``name``.

        Raises :class:`ProviderNotFoundError` if the name is unknown.
        """
        if name not in self._builders:
            raise ProviderNotFoundError(
                f"unknown harness TTS provider: {name!r}. "
                f"Available: {sorted(self._builders)}",
                port="tts",
            )
        return self._builders[name]

    def unregister(self, name: str) -> None:
        """Remove a builder from the registry. Test-only helper."""
        self._builders.pop(name, None)

    def names(self) -> list[str]:
        """Return all registered provider names (sorted)."""
        return sorted(self._builders)


class TTSProviderFactory:
    """Cached TTS provider factory.

    Instances are reused per ``(name, config_hash)`` so accidental
    double-instantiation in long-running processes doesn't leak
    ports. Tests can call :meth:`reset_cache` to start fresh.
    """

    _cache: dict[tuple[str, str], Any] = {}

    @classmethod
    def create(
        cls,
        name: str,
        config: TTSConfig,
        registry: TTSProviderRegistry,
    ) -> Any:
        """Resolve ``name`` in ``registry``, invoke the builder with.
        ``config``, cache and return the result.

        Raises :class:`ProviderNotFoundError` if ``name`` is unknown.
        Raises :class:`ConfigError` if the builder rejects
        ``config``.
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


def register_builtin_tts_providers(
    registry: TTSProviderRegistry | None = None,
) -> TTSProviderRegistry:
    """Register the built-in harness-side TTS providers.

    Currently registers:

    * ``"minimax"`` — the harness-side ``MiniMaxTTSProvider`` wrapper
      (see :mod:`rob_box_harness.tts.minimax_tts`). Adds env-based
      auth, exponential-backoff retries, and an optional content-hash
      cache on top of the upstream
      :class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider`.

    Real-world TTS providers (ElevenLabs, Google, local Piper) will be
    added here as their harness-side wrappers land.
    """
    if registry is None:
        registry = TTSProviderRegistry()

    def _minimax_builder(config: TTSConfig) -> Any:
        # Local import — keeps the registry import-cheap and lets
        # tests substitute the upstream module via monkeypatch.
        from rob_box_harness.tts.minimax_tts import build_minimax_tts_provider

        return build_minimax_tts_provider(config)

    registry.register("minimax", _minimax_builder)
    return registry


def _stable_config_hash(config: TTSConfig) -> Mapping[str, Any]:
    """Return a JSON-serialisable dict with ``config``'s meaningful fields.

    The factory uses this to know when two configs are "the same"
    for caching purposes. Mirrors the LLM-side helper in
    :mod:`rob_box_harness.registry`.
    """
    return {
        "provider": config.provider,
        "model": config.model,
        "voice": config.voice,
        "language": config.language,
        "sample_rate": config.sample_rate,
        "format": config.format,
        "timeout_s": config.timeout_s,
        "cache": config.cache,
        "rate_limit_per_min": config.rate_limit_per_min,
    }


__all__ = [
    "TTSBuilder",
    "TTSProviderRegistry",
    "TTSProviderFactory",
    "register_builtin_tts_providers",
]
