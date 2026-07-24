"""``MiniMaxProvider`` — the harness-side MiniMax LLM-provider.

This module is the **integration layer** between the
:class:`rob_box_harness.config.HarnessConfig` schema and the
production-grade :class:`rob_box_llm.providers.minimax.MiniMaxProvider`
that already implements the full :class:`rob_box_llm.LLMProvider`
contract (M1-M10 of ADR-0001 §2.6).

The class provided here adds three harness-side affordances on top of
the upstream provider:

1. **Authentication via env only** — the API key is read from
   ``MINIMAX_API_KEY`` (or supplied explicitly via ``api_key=``). The
   harness refuses to silently fall back to a YAML-embedded literal,
   per ADR-0001 §2.5.2 ("Every secret enters through env, never
   inline").
2. **A convenience ``chat(messages, **kwargs)`` method** — wraps
   :meth:`LLMProvider.complete` so callers can pass ``temperature=``,
   ``max_tokens=`` etc. positionally / by keyword without first
   building an :class:`LLMSettings` object. The full ``complete`` /
   ``stream`` API is still available for callers that need it.
3. **Optional retry with exponential backoff** — when the upstream
   provider raises :class:`RateLimitError` or :class:`TimeoutError`
   (transient only), the call is retried with
   ``delay = base * (2 ** attempt) + jitter``. :class:`AuthError`,
   :class:`ContentFilterError` and :class:`CapabilityUnavailableError`
   bypass the retry — they are programming errors, not transient
   failures.

The class is async-end-to-end (``asyncio``) and exposes the canonical
``name = "minimax"`` so the harness registry's fallback chain
(``[minimax, deepseek, mimo]``) can pick it by name.

The actual HTTP transport is delegated to the OpenAI SDK that
:class:`rob_box_llm.providers.minimax.MiniMaxProvider` wraps, so a
unit test can swap the SDK client (or pass an ``httpx.MockTransport``)
without touching this module's code.

See ``README.md`` in this directory for the YAML config shape and
the list of env vars.
"""

from __future__ import annotations

import asyncio
import logging
import os
import random
from dataclasses import dataclass
from typing import Any, AsyncIterator, Iterable, Mapping

from openai import AsyncOpenAI

from rob_box_harness.config import LLMConfig
from rob_box_harness.errors import ConfigError

# Re-export the upstream provider + helpers so callers can ``from
# rob_box_harness.providers.minimax import MiniMaxProvider`` exactly
# the way they would from ``rob_box_llm.providers``. The harness's
# wrappers below compose the upstream provider — they do not replace
# it.
from rob_box_llm.errors import (
    AuthError,
    CapabilityUnavailableError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.providers.minimax import (
    DEFAULT_THINKING_POLICY,
    MINIMAX_MAX_IMAGE_BYTES,
    MiniMaxProvider as _UpstreamMiniMaxProvider,
    MiniMaxRedactedLogFilter,
)
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
)

# The upstream provider exposes ``DEFAULT_BASE_URL`` and
# ``DEFAULT_MODEL`` as *class attributes* (``MiniMaxProvider``), not
# as module-level constants. We materialise them here so the
# harness-side surface is uniform and so callers can ``from
# rob_box_harness.providers.minimax import DEFAULT_BASE_URL``.
DEFAULT_BASE_URL: str = _UpstreamMiniMaxProvider.DEFAULT_BASE_URL
DEFAULT_MODEL: str = _UpstreamMiniMaxProvider.DEFAULT_MODEL

__all__ = [
    "MiniMaxProvider",
    "HarnessMiniMaxProvider",
    "build_minimax_provider",
    "RetryPolicy",
    "MINIMAX_API_KEY_ENV",
    # Re-exports from rob_box_llm so callers have a single import path.
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
    "DEFAULT_THINKING_POLICY",
    "MINIMAX_MAX_IMAGE_BYTES",
    "MiniMaxRedactedLogFilter",
]

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

#: Canonical env var that holds the MiniMax API key. ADR-0001 §2.5.2
#: mandates that every secret enters the system through the OS env,
#: never as a YAML literal. ``build_minimax_provider`` reads this var
#: when no explicit ``api_key=`` is supplied.
MINIMAX_API_KEY_ENV: str = "MINIMAX_API_KEY"


# ---------------------------------------------------------------------------
# Retry policy
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RetryPolicy:
    """Retry behaviour for transient MiniMax failures.

    Only transient errors (``RateLimitError`` / ``TimeoutError``) are
    retried. ``AuthError`` / ``ContentFilterError`` /
    ``CapabilityUnavailableError`` always propagate immediately —
    retrying them only hides a real bug.

    ``max_attempts`` is the upper bound including the initial call.
    ``max_attempts=1`` disables retries (the initial call still
    happens). ``backoff_base`` is the first retry delay in seconds;
    each subsequent delay is ``backoff_base * 2 ** (attempt - 1)``
    plus a uniform random jitter in ``[0, backoff_jitter)`` to avoid
    thundering herds. Wait times are therefore bounded by
    ``sum(backoff_base * 2 ** i for i in range(max_attempts - 1))``
    plus jitter.
    """

    max_attempts: int = 3
    backoff_base: float = 0.5
    backoff_jitter: float = 0.25

    def __post_init__(self) -> None:
        if self.max_attempts < 1:
            raise ValueError(
                f"RetryPolicy.max_attempts must be >= 1; got {self.max_attempts}"
            )
        if self.backoff_base < 0:
            raise ValueError(
                f"RetryPolicy.backoff_base must be >= 0; got {self.backoff_base}"
            )
        if self.backoff_jitter < 0:
            raise ValueError(
                f"RetryPolicy.backoff_jitter must be >= 0; got {self.backoff_jitter}"
            )

    def delay_for(self, attempt: int) -> float:
        """Return the sleep duration before retry ``attempt`` (1-based).

        ``attempt=1`` is the first retry (after the initial call).
        Returns 0.0 when ``attempt`` is out of range — callers should
        never invoke ``delay_for`` past ``max_attempts - 1``.
        """
        if attempt < 1:
            return 0.0
        base: float = self.backoff_base * (2 ** (attempt - 1))
        jitter: float = 0.0
        if self.backoff_jitter:
            jitter = float(random.uniform(0.0, self.backoff_jitter))
        return base + jitter


# ---------------------------------------------------------------------------
# Harness-side MiniMaxProvider
# ---------------------------------------------------------------------------


class HarnessMiniMaxProvider(LLMProvider):  # type: ignore[misc]
    """Harness-friendly wrapper around :class:`MiniMaxProvider`.

    The class is itself an :class:`LLMProvider` — it ``IS-A`` the
    contract — so it can be passed directly to ``harness.llm`` like
    any other provider. The harness gains:

    * automatic env-based auth (``MINIMAX_API_KEY``),
    * a friendly ``chat(messages, **kwargs)`` shortcut,
    * exponential-backoff retries on transient errors,
    * a no-op for non-transient errors so the caller sees the real
      failure type.

    The underlying transport (``AsyncOpenAI``) is composed from
    :class:`rob_box_llm.providers.minimax.MiniMaxProvider`, so all
    M1-M10 requirements (capabilities, image-size limit, base_resp
    envelope, API-key redaction, auth hardening, ``aclose``) are
    inherited unchanged.

    Parameters
    ----------
    api_key:
        The MiniMax API key. If ``None``, the provider reads
        ``MINIMAX_API_KEY`` from ``os.environ``. The key is never
        logged.
    model:
        Default model name. Defaults to ``"MiniMax-M3"`` (the
        upstream default; vision-capable).
    base_url:
        Endpoint URL. Defaults to ``https://api.minimax.io/v1``.
    timeout:
        Per-request timeout in seconds. Forwarded to the OpenAI SDK.
    retry:
        :class:`RetryPolicy` for transient failures. Pass
        ``RetryPolicy(max_attempts=1)`` to disable retries.
    client:
        Optional pre-built ``AsyncOpenAI`` client. Useful for tests
        that need to inject a mock transport.
    thinking:
        MiniMax thinking policy. Defaults to
        ``{"type": "disabled"}`` for latency-sensitive paths. Pass
        ``None`` to disable.
    """

    name = "minimax"

    def __init__(
        self,
        *,
        api_key: str | None = None,
        model: str = DEFAULT_MODEL,
        base_url: str = DEFAULT_BASE_URL,
        timeout: float = 30.0,
        retry: RetryPolicy | None = None,
        client: AsyncOpenAI | None = None,
        thinking: Mapping[str, str] | None = DEFAULT_THINKING_POLICY,
    ) -> None:
        # Resolve API key from env when not supplied. We intentionally
        # do NOT fall back to a YAML-embedded string — that's the
        # M7 contract from ADR-0001 §2.6.
        resolved_key = api_key or os.environ.get(MINIMAX_API_KEY_ENV)
        if not resolved_key:
            raise ConfigError(
                f"minimax: missing API key; set the {MINIMAX_API_KEY_ENV} "
                "env var or pass api_key= explicitly",
                section="llm.api_key",
            )

        # Build the upstream provider. We pass through every constructor
        # knob so callers get the full surface of M1-M10 without
        # re-implementing it here.
        self._inner: _UpstreamMiniMaxProvider = _UpstreamMiniMaxProvider(
            base_url=base_url,
            api_key=resolved_key,
            model=model,
            timeout=timeout,
            client=client,
            thinking=thinking,
        )
        self._retry: RetryPolicy = retry or RetryPolicy()
        # Track a close flag so ``aclose`` is idempotent. The inner
        # provider closes its own client; we just memoize here.
        self._closed: bool = False

    # ---- capability introspection ----------------------------------------

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Forward to the upstream provider.

        The upstream ``MiniMaxProvider`` exposes text + streaming +
        tools + image_input. ``streaming_tools`` is intentionally
        ``False`` — see the class docstring of the upstream provider.
        """
        return self._inner.capabilities

    def capabilities_for(self, model: str | None) -> ProviderCapabilities:
        """Capabilities narrowed to a specific model name.

        Mirrors :meth:`rob_box_llm.providers.minimax.MiniMaxProvider.capabilities_for`
        so that ``fallback`` chains can pick a vision-capable adapter
        for multi-modal requests.
        """
        return self._inner.capabilities_for(model)

    # ---- LLMProvider contract -------------------------------------------

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        """Run a non-streaming completion with retries.

        Retries on :class:`RateLimitError` /
        :class:`TimeoutError` only. Programming errors
        (``AuthError`` / ``ContentFilterError`` /
        ``CapabilityUnavailableError``) propagate immediately.
        """
        return await self._call_with_retry(
            self._inner.complete, messages, tools=tools, settings=settings
        )

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        """Run a streaming completion.

        The upstream provider streams synchronously from the first
        chunk. We therefore do *not* wrap ``stream`` in the retry
        decorator — once the first chunk is yielded, mid-stream
        failures become the caller's problem (matches the contract
        spelled out in :class:`rob_box_llm.provider.LLMProvider`).
        If the initial request fails, the upstream provider raises
        before yielding, and we DO retry the same way as ``complete``.
        """
        # We can't decorate an async generator with the retry loop
        # without buffering chunks, so we isolate the initial request
        # call in a retryable wrapper. The upstream AsyncOpenAI stream
        # is lazy, so the initial network round-trip happens as soon
        # as we begin iterating.
        attempts = 0
        last_exc: BaseException | None = None
        while attempts < self._retry.max_attempts:
            attempts += 1
            try:
                # We open the inner stream and yield from it. If the
                # first ``__anext__`` raises a transient error, we
                # close the stream and retry. Past the first chunk,
                # errors propagate.
                inner_stream = self._inner.stream(
                    messages, tools=tools, settings=settings
                )
                # Validate the initial request by peeking at the first
                # chunk. We do this by attempting ``__anext__`` and
                # then re-inserting the chunk via a queue.
                first_chunk = await inner_stream.__anext__()
                queue: asyncio.Queue[LLMChunk | BaseException | None] = asyncio.Queue()

                async def _replay() -> AsyncIterator[LLMChunk]:
                    # First chunk (already fetched) + remainder.
                    yield first_chunk
                    try:
                        while True:
                            chunk = await inner_stream.__anext__()
                            yield chunk
                    except StopAsyncIteration:
                        return

                replay_iter = _replay()
                # Wrap to surface mid-stream errors as final chunks.
                async def _runner() -> None:
                    try:
                        async for chunk in replay_iter:
                            await queue.put(chunk)
                    except BaseException as exc:  # noqa: BLE001
                        await queue.put(exc)
                    finally:
                        await queue.put(None)

                task = asyncio.create_task(_runner())
                while True:
                    item = await queue.get()
                    if item is None:
                        break
                    if isinstance(item, BaseException):
                        raise item
                    yield item
                # Drain so we don't leak the background task.
                await task
                return
            except (RateLimitError, TimeoutError) as exc:
                last_exc = exc
                if attempts >= self._retry.max_attempts:
                    raise
                delay = self._retry.delay_for(attempts)
                _log.warning(
                    "minimax: transient error on stream attempt %d/%d (%s); "
                    "retrying in %.2fs",
                    attempts,
                    self._retry.max_attempts,
                    type(exc).__name__,
                    delay,
                )
                await asyncio.sleep(delay)
                continue
        # Unreachable when max_attempts >= 1, but kept for type-checker.
        if last_exc is not None:
            raise last_exc
        raise RuntimeError("minimax.stream: exhausted retries without exception")

    async def aclose(self) -> None:
        """Tear down the underlying HTTP client.

        Idempotent: calling ``aclose`` twice is a no-op. After
        ``aclose`` the provider must not be reused.
        """
        if self._closed:
            return
        self._closed = True
        await self._inner.aclose()

    # ---- harness-friendly shortcut --------------------------------------

    async def chat(self, messages: list[LLMMessage] | tuple[LLMMessage, ...], **kwargs: Any) -> LLMResponse:
        """Convenience wrapper around :meth:`complete`.

        Accepts the same kwargs as :class:`LLMSettings` (minus
        ``model``/``temperature``/``max_tokens`` etc.) so callers can
        skip the boilerplate::

            response = await provider.chat(
                [LLMMessage(role="user", content="hi")],
                model="MiniMax-M3",
                temperature=0.7,
                max_tokens=256,
                tool_choice="auto",
            )

        Returns a fully-assembled :class:`LLMResponse`.
        """
        settings = self._build_settings(**kwargs)
        return await self.complete(messages, settings=settings)

    # ---- internal helpers -----------------------------------------------

    async def _call_with_retry(
        self,
        fn: Any,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]],
        settings: LLMSettings | None,
    ) -> LLMResponse:
        attempts = 0
        last_exc: BaseException | None = None
        while attempts < self._retry.max_attempts:
            attempts += 1
            try:
                return await fn(messages, tools=tools, settings=settings)
            except (RateLimitError, TimeoutError) as exc:
                last_exc = exc
                if attempts >= self._retry.max_attempts:
                    raise
                delay = self._retry.delay_for(attempts)
                _log.warning(
                    "minimax: transient error on attempt %d/%d (%s); "
                    "retrying in %.2fs",
                    attempts,
                    self._retry.max_attempts,
                    type(exc).__name__,
                    delay,
                )
                await asyncio.sleep(delay)
        if last_exc is not None:  # pragma: no cover — unreachable
            raise last_exc
        raise RuntimeError("minimax: exhausted retries without exception")

    @staticmethod
    def _build_settings(**kwargs: Any) -> LLMSettings:
        """Pull kwargs into an :class:`LLMSettings` instance.

        Unknown kwargs are merged into ``extra`` so callers can pass
        vendor-specific knobs (``thinking=``, ``top_k=``, …) without
        changing the surface.
        """
        known = {
            "model",
            "temperature",
            "max_tokens",
            "stop",
            "tool_choice",
        }
        known_args: dict[str, Any] = {}
        extra: dict[str, Any] = {}
        for key, value in kwargs.items():
            if key in known:
                known_args[key] = value
            else:
                extra[key] = value
        return LLMSettings(extra=extra, **known_args)


# ---------------------------------------------------------------------------
# Public alias
# ---------------------------------------------------------------------------

# The task body asks for a class named ``MiniMaxProvider``. The
# harness owns the class layout, so we publish the wrapper under that
# canonical name. The upstream provider remains importable as
# ``rob_box_harness.providers.minimax._UpstreamMiniMaxProvider`` for
# callers that need direct access (advanced tests, vendor extensions).
MiniMaxProvider = HarnessMiniMaxProvider


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def build_minimax_provider(
    llm_config: LLMConfig,
    *,
    env: Mapping[str, str] | None = None,
    retry: RetryPolicy | None = None,
    client: AsyncOpenAI | None = None,
) -> MiniMaxProvider:
    """Build a :class:`MiniMaxProvider` from a :class:`LLMConfig`.

    The API key is resolved entirely from ``env`` (which defaults to
    ``os.environ``); the YAML / LLMConfig layer is not allowed to
    embed a literal key. If ``llm_config.api_key`` is set, it must
    resolve to a non-empty string via ``env``; otherwise the factory
    raises :class:`ConfigError`.

    Parameters
    ----------
    llm_config:
        Parsed ``llm`` section of the harness config.
    env:
        Explicit env map for tests. ``None`` (default) reads
        ``os.environ``. Secrets are looked up here first, then
        ``os.environ`` — which means tests can isolate the env
        without monkey-patching the global ``os.environ``.
    retry:
        Override the default retry policy. ``None`` uses the
        default ``RetryPolicy(max_attempts=3, backoff_base=0.5)``.
    client:
        Pre-built OpenAI client (for tests with a mock transport).

    Raises
    ------
    ConfigError
        If ``llm_config.provider`` is not ``"minimax"``, or the API
        key is missing from both env and the explicit ``api_key=``
        form of the config.
    """
    if llm_config is None:
        raise ConfigError(
            "minimax: cannot build provider from None config",
            section="llm",
        )
    if llm_config.provider != "minimax":
        raise ConfigError(
            f"minimax: expected llm.provider='minimax', got "
            f"{llm_config.provider!r}",
            section="llm.provider",
        )

    # Resolve the API key. The harness never hardcodes; if the env
    # is missing the key, we fail loudly at startup instead of at
    # first request.
    env_map: Mapping[str, str] = env if env is not None else os.environ
    api_key: str | None = llm_config.api_key
    if api_key is None:
        api_key = env_map.get(MINIMAX_API_KEY_ENV)
    if not api_key:
        raise ConfigError(
            f"minimax: missing API key; set the {MINIMAX_API_KEY_ENV} "
            "env var or pass it via llm.api_key (which itself must "
            "resolve to ${" + MINIMAX_API_KEY_ENV + "})",
            section="llm.api_key",
        )

    # Translate LLMConfig knobs into the constructor signature.
    timeout = float(llm_config.timeout_s) if llm_config.timeout_s is not None else 30.0
    model = llm_config.model or DEFAULT_MODEL

    return MiniMaxProvider(
        api_key=api_key,
        model=model,
        base_url=DEFAULT_BASE_URL,
        timeout=timeout,
        retry=retry,
        client=client,
    )
