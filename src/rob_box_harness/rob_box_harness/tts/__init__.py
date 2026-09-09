"""Harness-side TTS providers (ADR-0001 P0 + ADR-0008 / t_8cbf9995).

This package exposes the harness-side **wrappers** that bind the
upstream :class:`rob_box_llm.tts.TTSProvider` contract to the
:class:`rob_box_harness.config.TTSConfig` schema.

Sub-modules:

* :mod:`rob_box_harness.tts.minimax_tts` — the harness-side
  ``MiniMaxTTSProvider`` wrapper. Adds env-based auth, exponential-
  backoff retries on transient errors, and an optional content-hash
  cache for repeated phrases. Delegates to the
  :class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider`
  upstream provider for the actual HTTP transport.

* :mod:`rob_box_harness.tts.registry` — the harness-side
  ``TTSProviderRegistry`` + cached ``TTSProviderFactory`` +
  :func:`register_builtin_tts_providers` composition-root helper.

Future TTS providers (ElevenLabs, Google, local Piper) will be added
here as their harness-side wrappers land.
"""

from __future__ import annotations

from rob_box_harness.tts.minimax_tts import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    DEFAULT_TIMEOUT,
    DEFAULT_VOICE,
    MINIMAX_API_KEY_ENV,
    MINIMAX_GROUP_ID_ENV,
    HarnessMiniMaxTTSProvider,
    MiniMaxTTSProvider,
    RetryPolicy,
    build_minimax_tts_provider,
)
from rob_box_harness.tts.registry import (
    TTSBuilder,
    TTSProviderFactory,
    TTSProviderRegistry,
    register_builtin_tts_providers,
)

__all__ = [
    # MiniMax (harness-side)
    "MiniMaxTTSProvider",
    "HarnessMiniMaxTTSProvider",
    "build_minimax_tts_provider",
    "RetryPolicy",
    "MINIMAX_API_KEY_ENV",
    "MINIMAX_GROUP_ID_ENV",
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
    "DEFAULT_VOICE",
    "DEFAULT_TIMEOUT",
    # Registry
    "TTSBuilder",
    "TTSProviderRegistry",
    "TTSProviderFactory",
    "register_builtin_tts_providers",
]
