"""Concrete providers shipped with the harness framework.

This package stays intentionally small: only the implementations that
the dummy harnesses and the smoke test need at the framework level,
plus the harness-side **wrappers** that bind
:class:`rob_box_llm.providers` providers to the
:class:`rob_box_harness.config.LLMConfig` schema.

Real-world network providers (DeepSeek, MiniMax, MiMo, ROS2Transport,
…) live in their own packages and are wired in via the
:class:`HarnessRegistry`.

Sub-modules
-----------

* :mod:`rob_box_harness.providers.dummy` — the deterministic
  ``DummyLLMProvider`` used by the dummy harnesses.
* :mod:`rob_box_harness.providers.fake_llm` — thin re-export of
  ``rob_box_llm.providers.fake.FakeLLMProvider`` under the harness
  name.
* :mod:`rob_box_harness.providers.minimax` — the harness-side
  ``MiniMaxProvider`` wrapper. Adds env-based auth, a
  ``chat(messages, **kwargs)`` convenience method, and exponential-
  backoff retries on transient errors. Delegates to the
  ``rob_box_llm.providers.minimax`` upstream provider for the actual
  HTTP transport.
"""

from __future__ import annotations

from rob_box_harness.providers.dummy import DummyLLMProvider
from rob_box_harness.providers.fake_llm import HarnessFakeLLMProvider
from rob_box_harness.providers.minimax import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    DEFAULT_THINKING_POLICY,
    MINIMAX_API_KEY_ENV,
    MINIMAX_MAX_IMAGE_BYTES,
    HarnessMiniMaxProvider,
    MiniMaxProvider,
    MiniMaxRedactedLogFilter,
    RetryPolicy,
    build_minimax_provider,
)

__all__ = [
    "DummyLLMProvider",
    "HarnessFakeLLMProvider",
    # MiniMax (harness-side)
    "MiniMaxProvider",
    "HarnessMiniMaxProvider",
    "build_minimax_provider",
    "RetryPolicy",
    "MINIMAX_API_KEY_ENV",
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
    "DEFAULT_THINKING_POLICY",
    "MINIMAX_MAX_IMAGE_BYTES",
    "MiniMaxRedactedLogFilter",
]
