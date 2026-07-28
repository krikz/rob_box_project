"""LLM provider configuration registry and resolvers.

Pure helpers extracted from :class:`rob_box_voice.dialogue_node.DialogueNode`
(ADR-0001 §2.7 — "core modules own pure logic; nodes own side-effects").

The functions here do NOT import ``rclpy`` or any ROS2 dependency so
they are trivially unit-testable.

Usage::

    cfg = resolve_provider_config(
        provider_name="deepseek",
        overrides=ProviderOverrides(api_key="", base_url="", model=""),
        env=os.environ,
    )
    print(cfg.api_key, cfg.base_url, cfg.model, cfg.fallback_model)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Callable, Mapping, Optional, Sequence

# Default fallback model when neither the per-provider entry nor the
# ROS2 parameter supplies one. Mirrors the legacy value in
# ``DialogueNode._resolve_model``.
DEFAULT_FALLBACK_MODEL = "deepseek-v4-flash"

# ── Provider registry ──────────────────────────────────────────────────────
# Each entry is a static catalog of the default API surface for a named
# provider. Override any field via ROS2 parameter (see ProviderOverrides).
# The dialog node reads ``PROVIDERS`` and falls back to these defaults.

PROVIDERS: dict[str, dict[str, object]] = {
    "deepseek": {
        "base_url": "https://api.deepseek.com/v1",
        "model": "deepseek-v4-flash",
        "fallback_model": "deepseek-v4-flash",
        "env_vars": ["DEEPSEEK_API_KEY", "LLM_API_KEY"],
    },
    "mimo": {
        "base_url": "https://api.xiaomimimo.com/v1",
        "model": "mimo-v2.5-pro",
        "fallback_model": "mimo-v2.5",
        "env_vars": ["MIMO_API_KEY", "LLM_API_KEY"],
    },
}


@dataclass(frozen=True)
class ProviderOverrides:
    """Optional caller-supplied overrides for a provider.

    An empty string means "use the registry default". ``None`` means
    "fall through to the registry without warning". Both are valid;
    callers should pick the one that matches their semantics.
    """

    api_key: str = ""
    base_url: str = ""
    model: str = ""
    fallback_model: str = ""


@dataclass(frozen=True)
class ProviderConfig:
    """Fully resolved provider settings, ready for the LLM client."""

    provider: str
    api_key: str
    base_url: str
    model: str
    fallback_model: str
    env_vars: Sequence[str] = field(default_factory=tuple)

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-friendly dict (matches the legacy shape)."""
        return {
            "provider": self.provider,
            "api_key": self.api_key,
            "base_url": self.base_url,
            "model": self.model,
            "fallback_model": self.fallback_model,
            "env_vars": list(self.env_vars),
        }


def _get_env(name: str, env: Mapping[str, str]) -> str:
    """Read from the provided env mapping (testable substitute for ``os.environ``)."""
    return env.get(name, "")


def resolve_api_key(
    provider: str,
    override: str = "",
    env: Optional[Mapping[str, str]] = None,
    registry: Optional[Mapping[str, Mapping[str, object]]] = None,
) -> str:
    """Resolve the API key for ``provider``.

    Resolution order:
        1. ``override`` (if non-empty)
        2. First non-empty value among the registry's ``env_vars``
        3. ``RuntimeError`` listing the candidate env var names.

    Args:
        provider: Provider key (e.g. ``"deepseek"``).
        override: Explicit API key from a higher-priority source (ROS param).
        env: Environment mapping. Defaults to ``os.environ`` at call time.
        registry: Provider catalog. Defaults to :data:`PROVIDERS`.

    Returns:
        The resolved API key string (never empty).

    Raises:
        RuntimeError: When no key is found anywhere.
        KeyError: When ``provider`` is unknown AND ``override`` is empty.
    """
    if override:
        return override
    if env is None:
        env = os.environ
    if registry is None:
        registry = PROVIDERS
    entry = registry.get(provider)
    if entry is None:
        raise KeyError(
            f"Unknown provider '{provider}'. "
            f"Known: {sorted(registry.keys())}"
        )
    env_vars = entry.get("env_vars", [])  # type: ignore[arg-type]
    for name in env_vars:  # type: ignore[union-attr]
        val = _get_env(name, env)
        if val:
            return val
    raise RuntimeError(
        f"API key not found for provider '{provider}'. "
        f"Set one of: {list(env_vars)}"  # type: ignore[arg-type]
    )


def resolve_base_url(
    provider: str,
    override: str = "",
    registry: Optional[Mapping[str, Mapping[str, object]]] = None,
) -> str:
    """Resolve the base URL for ``provider`` (override > registry)."""
    if override:
        return override
    if registry is None:
        registry = PROVIDERS
    entry = registry.get(provider, {})
    return str(entry.get("base_url", ""))


def resolve_model(
    provider: str,
    override: str = "",
    field_name: str = "model",
    registry: Optional[Mapping[str, Mapping[str, object]]] = None,
) -> str:
    """Resolve the (fallback) model for ``provider`` (override > registry).

    Args:
        provider: Provider key.
        override: Explicit value from a higher-priority source.
        field_name: ``"model"`` or ``"fallback_model"``.
        registry: Provider catalog. Defaults to :data:`PROVIDERS`.
    """
    if override:
        return override
    if registry is None:
        registry = PROVIDERS
    entry = registry.get(provider, {})
    value = entry.get(field_name)
    if isinstance(value, str) and value:
        return value
    return DEFAULT_FALLBACK_MODEL


def resolve_provider_config(
    provider: str,
    overrides: Optional[ProviderOverrides] = None,
    env: Optional[Mapping[str, str]] = None,
    registry: Optional[Mapping[str, Mapping[str, object]]] = None,
    *,
    error_on_missing_key: bool = True,
) -> ProviderConfig:
    """Resolve all four provider fields in one shot.

    Args:
        provider: Provider key.
        overrides: Optional explicit overrides for any of the four fields.
        env: Environment mapping (defaults to ``os.environ``).
        registry: Provider catalog (defaults to :data:`PROVIDERS`).
        error_on_missing_key: If ``False``, return an empty ``api_key``
            instead of raising when no key is found. Useful for
            "dry-run" setups (e.g. CI without secrets).

    Returns:
        A :class:`ProviderConfig` with all four fields populated.

    Raises:
        KeyError: When ``provider`` is unknown.
        RuntimeError: When ``api_key`` cannot be resolved and
            ``error_on_missing_key`` is True.
    """
    if overrides is None:
        overrides = ProviderOverrides()
    if registry is None:
        registry = PROVIDERS
    entry = registry.get(provider)
    if entry is None:
        raise KeyError(
            f"Unknown provider '{provider}'. "
            f"Known: {sorted(registry.keys())}"
        )
    env_vars = tuple(entry.get("env_vars", []) or ())  # type: ignore[arg-type]
    base_url = resolve_base_url(provider, overrides.base_url, registry)
    model = resolve_model(provider, overrides.model, "model", registry)
    fallback_model = resolve_model(
        provider, overrides.fallback_model, "fallback_model", registry
    )
    if error_on_missing_key:
        api_key = resolve_api_key(provider, overrides.api_key, env, registry)
    else:
        try:
            api_key = resolve_api_key(provider, overrides.api_key, env, registry)
        except RuntimeError:
            api_key = ""
    return ProviderConfig(
        provider=provider,
        api_key=api_key,
        base_url=base_url,
        model=model,
        fallback_model=fallback_model,
        env_vars=env_vars,
    )


def known_providers() -> tuple[str, ...]:
    """Return the registered provider names (immutable snapshot)."""
    return tuple(PROVIDERS.keys())


__all__ = [
    "DEFAULT_FALLBACK_MODEL",
    "PROVIDERS",
    "ProviderConfig",
    "ProviderOverrides",
    "known_providers",
    "resolve_api_key",
    "resolve_base_url",
    "resolve_model",
    "resolve_provider_config",
]