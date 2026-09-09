"""Well-known LLM providers: defaults + a ROS2-free factory.

This is the single declaration of "which provider name means what":
display name, default ``base_url`` / ``model``, the env var that carries
its API key, and whether it exposes a balance API worth probing before
the first call.

The table used to live as ``DialogueNode._LLM_PROVIDER_REGISTRY`` inside
``rob_box_voice.dialogue_node`` — a module that imports ``rclpy`` at the
top and therefore cannot be touched by anything outside a ROS2 runtime.
Every other caller that wanted the same defaults (the local text-chat
entry point in ``scripts/dialogue``, tests, future adapters) had to copy
them, which is exactly the drift that cost this project issues #1252 and
#1734 with ``wake_words``. One table, here, in the ROS2-free package
that already owns the provider builders.

``build_provider`` is the pure half of ``DialogueNode._build_single_provider``:
given a name and the (already resolved) overrides it returns a provider
instance. Reading ROS parameters and logging stays in the node; reading
``.env`` and printing stays in the CLI.
"""

from __future__ import annotations

import os
from typing import Any, Mapping

from rob_box_harness.config import LLMConfig
from rob_box_harness.providers.deepseek import (
    DEFAULT_BASE_URL as DEEPSEEK_DEFAULT_BASE_URL,
    DEFAULT_MODEL as DEEPSEEK_DEFAULT_MODEL,
    build_deepseek_provider,
)
from rob_box_harness.providers.minimax import (
    DEFAULT_BASE_URL as MINIMAX_DEFAULT_BASE_URL,
    DEFAULT_MODEL as MINIMAX_DEFAULT_MODEL,
    build_minimax_provider,
)

__all__ = [
    "LLM_PROVIDER_REGISTRY",
    "known_provider_names",
    "resolve_api_key",
    "build_provider",
]


#: name → metadata. ``default_base_url`` / ``default_model`` are the
#: well-known values used when neither the YAML section nor the CLI
#: passes an override. ``env_key_var`` is the environment variable the
#: API key is read from. ``has_balance_api`` marks providers whose
#: balance can be probed before the first call (see
#: :class:`rob_box_harness.health.HealthAwareFallbackLLM`).
LLM_PROVIDER_REGISTRY: dict[str, dict[str, Any]] = {
    "minimax": {
        "display_name": "MiniMax",
        "has_balance_api": False,
        "default_base_url": MINIMAX_DEFAULT_BASE_URL,
        "default_model": MINIMAX_DEFAULT_MODEL,
        "env_key_var": "MINIMAX_API_KEY",
    },
    "deepseek": {
        "display_name": "DeepSeek",
        "has_balance_api": True,
        "default_base_url": DEEPSEEK_DEFAULT_BASE_URL,
        "default_model": DEEPSEEK_DEFAULT_MODEL,
        "env_key_var": "DEEPSEEK_API_KEY",
    },
    "mimo": {
        "display_name": "MiMo",
        "has_balance_api": False,
        "default_base_url": "https://api.xiaomimimo.com/v1",
        "default_model": "mimo-v2.5",
        "env_key_var": "MIMO_API_KEY",
    },
    "qwen": {
        "display_name": "Qwen",
        "has_balance_api": False,
        "default_base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "default_model": "qwen-turbo",
        "env_key_var": "DASHSCOPE_API_KEY",
    },
}


def known_provider_names() -> tuple[str, ...]:
    """Return every provider name the registry knows, sorted."""
    return tuple(sorted(LLM_PROVIDER_REGISTRY))


def resolve_api_key(
    name: str,
    *,
    explicit: str | None = None,
    env: Mapping[str, str] | None = None,
) -> str | None:
    """Resolve the API key for ``name``: explicit → env var → ``None``.

    ``None`` is a legitimate outcome — the provider builder decides
    whether a missing key is fatal (MiniMax raises ``ConfigError``,
    DeepSeek defers to the OpenAI SDK).
    """
    if explicit:
        return explicit
    entry = LLM_PROVIDER_REGISTRY.get(name.strip().lower())
    if entry is None:
        return None
    env_var = entry.get("env_key_var", "")
    if not env_var:
        return None
    env_map: Mapping[str, str] = env if env is not None else os.environ
    return env_map.get(env_var) or None


def build_provider(
    name: str,
    *,
    api_key: str | None = None,
    base_url: str = "",
    model: str = "",
    timeout_s: float = 0.0,
) -> Any:
    """Build one provider by well-known name.

    Args:
        name: Registry key (``minimax`` / ``deepseek`` / ``mimo`` / ``qwen``).
        api_key: Already-resolved key, or ``None`` to let
            :func:`resolve_api_key` read the registry's env var.
        base_url: Override; empty falls back to the registry default.
            Ignored for MiniMax, whose builder pins its own base URL.
        model: Override; empty falls back to the registry default.
        timeout_s: Override; ``0`` falls back to the provider default.

    Raises:
        KeyError: if ``name`` is not in the registry.
        rob_box_harness.errors.ConfigError: if the provider needs an API
            key and none was found.
    """
    key = name.strip().lower()
    entry = LLM_PROVIDER_REGISTRY.get(key)
    if entry is None:
        raise KeyError(
            f"unknown LLM provider {name!r}; known: {known_provider_names()!r}"
        )

    resolved_key = resolve_api_key(key, explicit=api_key)
    resolved_model = model or entry.get("default_model", "")
    resolved_base_url = base_url or entry.get("default_base_url", "")

    if key == "minimax":
        return build_minimax_provider(
            LLMConfig(
                provider="minimax",
                model=resolved_model or MINIMAX_DEFAULT_MODEL,
                api_key=resolved_key,
                timeout_s=timeout_s or 90.0,
            )
        )
    # Everything else is OpenAI-compatible (deepseek, mimo, qwen) and
    # goes through the same chat-completions client. NB: the timeout is
    # deliberately NOT forwarded here — ``build_deepseek_provider``'s own
    # 30s default is what the robot has always run with, and raising it
    # silently would change live latency behaviour, not just local runs.
    return build_deepseek_provider(
        api_key=resolved_key,
        base_url=resolved_base_url or DEEPSEEK_DEFAULT_BASE_URL,
        model=resolved_model or DEEPSEEK_DEFAULT_MODEL,
    )
