"""TTS Provider registry + factory.

Landed in P0.5 / ADR-0008. Provides:

* :class:`TTSProviderRegistry`     — in-memory ``name → builder`` map
* :class:`TTSProviderFactory`      — single entry point for constructing
  a provider; caches instances per ``(name, config_hash)``
* :func:`register_builtin_tts_providers` — registers ``"minimax"`` (and
  future built-ins like ``"elevenlabs"``, ``"google"``, ``"local-piper"``)

Why a separate registry module:

* Single source of truth for ``provider_name → builder`` mapping.
* Lets 3rd-party packages register their own providers without
  touching ``rob_box_llm`` (callers add to a passed-in registry).
* Gives tests a way to inject mock providers under any name.

Composition-root contract:

* :func:`register_builtin_tts_providers` is the ONLY place built-in
  providers get registered. Called once at process start.
* No auto-discovery via ``importlib.metadata.entry_points()`` —
  ADR-0004 §2.3 explicitly rejects this (implicit side-effects,
  hard to test, hidden dependencies at import time).
* :meth:`TTSProviderFactory.create` is the only entry point for both
  ROS path (``tts_node._synthesize_and_play``) and CLI path (future).

See also:

* ``docs/architecture/tts-extension-points.md`` — full design doc
* ``docs/adr/0004-minimax-tts-integration-design.md`` §2.3, §2.8
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Mapping

if TYPE_CHECKING:
    from rob_box_llm.tts_provider_base import BaseTTSProvider, ProviderBuilder


class TTSProviderRegistry:
    """In-memory ``name → builder`` registry.

    NOT thread-safe for concurrent registration. Construct once at
    process start, then read-only for the rest of the lifetime.
    """

    def __init__(self) -> None:
        self._builders: dict[str, "ProviderBuilder"] = {}

    def register(self, name: str, builder: "ProviderBuilder") -> None:
        """Register a builder under ``name``.

        Raises ``ValueError`` if the name is already taken (no silent
        override — refactor the registration call instead).
        """
        if name in self._builders:
            raise ValueError(
                f"TTS provider {name!r} already registered; "
                "duplicate registration is forbidden"
            )
        self._builders[name] = builder

    def resolve(self, name: str) -> "ProviderBuilder":
        """Return the builder for ``name``. Raises ``KeyError`` if missing."""
        if name not in self._builders:
            raise KeyError(
                f"Unknown TTS provider: {name!r}. "
                f"Available: {sorted(self._builders)}"
            )
        return self._builders[name]

    def unregister(self, name: str) -> None:
        """Remove a builder from the registry. Test-only helper.

        Production code should never unregister — composition root is
        supposed to be set once at process start.
        """
        self._builders.pop(name, None)

    def names(self) -> list[str]:
        """Return all registered provider names (sorted)."""
        return sorted(self._builders)


class TTSProviderFactory:
    """Single entry point for constructing a TTS provider.

    Both the ROS path (``tts_node._synthesize_and_play``) and the future
    CLI path (``rob_box_llm.tts_cli``) call :meth:`create`. Built
    instances are cached per ``(name, config_hash)`` so accidental
    double-construction in long-running processes doesn't open two
    HTTP pools.
    """

    _cache: dict[tuple[str, str], "BaseTTSProvider"] = {}

    @classmethod
    def create(
        cls,
        name: str,
        config: Mapping[str, Any],
        registry: "TTSProviderRegistry",
    ) -> "BaseTTSProvider":
        """Resolve ``name`` in ``registry``, invoke builder with ``config``,.
        cache and return the result.

        Raises ``KeyError`` if ``name`` is unknown. Raises ``ValueError``
        if ``config`` fails provider-specific validation (builders may
        defer validation to first call — that's allowed but discouraged).
        """
        config_hash = repr(sorted(config.items()))
        cache_key = (name, config_hash)
        if cache_key in cls._cache:
            return cls._cache[cache_key]
        builder = registry.resolve(name)
        instance = builder(config)
        cls._cache[cache_key] = instance
        return instance

    @classmethod
    def reset_cache(cls) -> None:
        """Clear the instance cache. Test-only helper."""
        cls._cache.clear()


def register_builtin_tts_providers(
    registry: "TTSProviderRegistry | None" = None,
) -> "TTSProviderRegistry":
    """Register MiniMax (and future built-ins) under their canonical names.

    Called once at process start from the composition root
    (``tts_node.on_init`` for ROS, ``tts_cli`` entry-point for CLI).
    Returns the (possibly fresh) registry for convenience.

    Currently registers only ``"minimax"``. Future built-ins
    (``"elevenlabs"``, ``"google"``, ``"local-piper"``) will be added
    here as their providers land in ``providers/``.
    """
    if registry is None:
        registry = TTSProviderRegistry()

    def _minimax_builder(config: Mapping[str, Any]) -> "BaseTTSProvider":
        from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider

        return MiniMaxTTSProvider(**config)

    registry.register("minimax", _minimax_builder)

    # Future built-ins registered here as they land:
    # registry.register("elevenlabs", _elevenlabs_builder)
    # registry.register("google",      _google_builder)
    # registry.register("local-piper", _local_piper_builder)

    return registry


__all__ = [
    "TTSProviderRegistry",
    "TTSProviderFactory",
    "register_builtin_tts_providers",
]
