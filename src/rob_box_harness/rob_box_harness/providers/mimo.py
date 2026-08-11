"""``MiMoProvider`` — the harness-side Xiaomi MiMo LLM-provider.

This module is the **integration layer** between the
:class:`rob_box_harness.config.HarnessConfig` schema and the
production-grade :class:`rob_box_llm.providers.mimo.MiMoProvider` that
already implements the full :class:`rob_box_llm.LLMProvider` contract.

The class provided here mirrors :class:`HarnessDeepSeekProvider`:

1. **Authentication via env only** — the API key is read from
   ``MIMO_API_KEY`` (or supplied explicitly via ``api_key=``).
2. **A convenience ``chat(messages, **kwargs)`` method.**
3. **Optional retry with exponential backoff** — transient errors are
   retried with ``delay = base * (2 ** attempt) + jitter``.

The class is async-end-to-end (``asyncio``) and exposes the canonical
``name = "mimo"`` so the harness registry's fallback chain
(``[minimax, deepseek, mimo]``) can pick it by name.
"""

from __future__ import annotations

import os
from typing import Any, Iterable, Mapping

from openai import AsyncOpenAI

from rob_box_harness.errors import ConfigError
from rob_box_harness.providers.deepseek import HarnessDeepSeekProvider, RetryPolicy
from rob_box_llm.providers.mimo import MiMoProvider as _UpstreamMiMoProvider
from rob_box_llm.provider import (
    LLMMessage,
    LLMResponse,
    LLMSettings,
)

DEFAULT_BASE_URL: str = _UpstreamMiMoProvider.DEFAULT_BASE_URL
DEFAULT_MODEL: str = _UpstreamMiMoProvider.DEFAULT_MODEL

__all__ = [
    "MimoProvider",
    "HarnessMiMoProvider",
    "build_mimo_provider",
    "MIMO_API_KEY_ENV",
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
]


#: Canonical env var that holds the MiMo API key. ADR-0001 §2.5.2
#: mandates that every secret enters the system through the OS env.
MIMO_API_KEY_ENV: str = "MIMO_API_KEY"


class HarnessMiMoProvider(HarnessDeepSeekProvider):
    """Harness-friendly wrapper around :class:`MiMoProvider`.

    Inherits the complete retry / chat / aclose behaviour from
    :class:`HarnessDeepSeekProvider` and overrides only the auth /
    endpoint defaults.

    Parameters
    ----------
    api_key:
        The MiMo API key. If ``None``, reads ``MIMO_API_KEY`` from
        ``os.environ``.
    model:
        Default model. Defaults to ``"mimo-v2.5-pro"``.
    base_url:
        Endpoint URL. Defaults to ``https://api.xiaomimimo.com/v1``.
    """

    name = "mimo"

    def __init__(
        self,
        *,
        api_key: str | None = None,
        model: str = DEFAULT_MODEL,
        base_url: str = DEFAULT_BASE_URL,
        timeout: float = 30.0,
        retry: RetryPolicy | None = None,
        client: AsyncOpenAI | None = None,
    ) -> None:
        # We deliberately bypass the parent's __init__ so we can use
        # the MIMO-specific env var. The parent's __init__ would
        # otherwise look up DEEPSEEK_API_KEY.
        resolved_key = api_key or os.environ.get(MIMO_API_KEY_ENV)
        if not resolved_key:
            raise ConfigError(
                f"mimo: missing API key; set the {MIMO_API_KEY_ENV} "
                "env var or pass api_key= explicitly",
                section="llm.api_key",
            )

        self._inner: _UpstreamMiMoProvider = _UpstreamMiMoProvider(
            base_url=base_url,
            api_key=resolved_key,
            model=model,
            timeout=timeout,
            client=client,
        )
        self._retry: RetryPolicy = retry or RetryPolicy()
        self._closed: bool = False


def build_mimo_provider(
    *,
    api_key: str | None = None,
    model: str = DEFAULT_MODEL,
    base_url: str = DEFAULT_BASE_URL,
    timeout: float = 30.0,
    retry: RetryPolicy | None = None,
    client: AsyncOpenAI | None = None,
) -> HarnessMiMoProvider:
    """Build a :class:`HarnessMiMoProvider` from explicit kwargs."""
    return HarnessMiMoProvider(
        api_key=api_key,
        model=model,
        base_url=base_url,
        timeout=timeout,
        retry=retry,
        client=client,
    )


#: Backwards-compatible alias.
MimoProvider = HarnessMiMoProvider