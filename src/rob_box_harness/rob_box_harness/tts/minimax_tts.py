"""``MiniMaxTTSProvider`` — the harness-side MiniMax TTS provider.

This module is the **integration layer** between the
:class:`rob_box_harness.config.TTSConfig` schema and the
production-grade :class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider`
that already implements the full :class:`rob_box_llm.tts.TTSProvider`
contract (5 extension points per ADR-0008 + the t_8cbf9995 series:
``synthesize`` / ``stream`` / ``aclose``, ``capabilities``,
``list_voices``, ``healthcheck``, ``_build_request_payload``,
``_http_client_factory``).

The class provided here adds three harness-side affordances on top of
the upstream provider:

1. **Authentication via env only** — the API key (and the
   ``MINIMAX_GROUP_ID`` required by the MiniMax T2A v2 query string)
   are read from environment variables, never from a YAML literal
   (ADR-0001 §2.5.2 — "Every secret enters through env, never inline").
   The factory :func:`build_minimax_tts_provider` raises
   :class:`ConfigError` when either is missing.
2. **Optional retry with exponential backoff** — when the upstream
   provider raises :class:`TTSRateLimitError` or
   :class:`TTSTimeoutError` (transient only), the call is retried with
   ``delay = base * (2 ** attempt) + jitter``. :class:`TTSAuthError`
   and :class:`TTSBadRequestError` bypass the retry — they are
   programming errors, not transient failures.
3. **Optional content-hash cache** — when ``cache=True`` the wrapper
   memoises ``synthesize`` results by ``hash(text + voice + format +
   sample_rate)`` so repeated phrases ("OK", wake-word ack, error
   stubs) don't burn quota. Disabled by default.

The class is async-end-to-end (``asyncio``) and exposes the canonical
``name = "minimax"`` so the harness-side
:class:`TTSProviderRegistry` (see :mod:`rob_box_harness.tts.registry`)
can pick it by name.

The actual HTTP transport is delegated to the upstream
:class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider`, which
uses :class:`httpx.AsyncClient` directly. Unit tests can swap the
client (or pass an :class:`httpx.MockTransport`) without touching this
module's code.

See ``README.md`` in this directory for the YAML config shape and
the list of env vars.
"""

from __future__ import annotations

import asyncio
import hashlib
import logging
import os
import random
import time
from dataclasses import dataclass, field
from typing import Any, AsyncIterator, Mapping, Optional

import httpx

from rob_box_harness.config import TTSConfig
from rob_box_harness.errors import ConfigError
from rob_box_harness.providers.minimax import (
    RetryPolicy as _LLMRetryPolicy,  # re-use the LLM retry policy
)

# Re-export the upstream provider + helpers so callers can ``from
# rob_box_harness.tts.minimax_tts import MiniMaxTTSProvider`` exactly
# the way they would from ``rob_box_llm.providers``. The harness's
# wrapper below composes the upstream provider — it does not replace
# it.
from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.providers.minimax_tts import (
    MiniMaxTTSProvider as _UpstreamMiniMaxTTSProvider,
)
from rob_box_llm.tts import (
    TTSAudio,
    TTSChunk,
    TTSFormat,
    TTSProvider,
    TTSSettings,
)
from rob_box_llm.tts_provider_base import (
    TTSCapabilities,
    TTSHealth,
    TTSVoice,
)

# Materialise upstream defaults as module-level constants so the
# harness-side surface is uniform. Callers can ``from
# rob_box_harness.tts.minimax_tts import DEFAULT_BASE_URL``. The
# upstream class exposes these as class attributes; we hoist them
# out so ``import CONST`` is symmetric with the LLM-side surface.
DEFAULT_BASE_URL: str = _UpstreamMiniMaxTTSProvider.DEFAULT_BASE_URL
DEFAULT_MODEL: str = _UpstreamMiniMaxTTSProvider.DEFAULT_MODEL
DEFAULT_VOICE: str = _UpstreamMiniMaxTTSProvider.DEFAULT_VOICE
DEFAULT_TIMEOUT: float = _UpstreamMiniMaxTTSProvider.DEFAULT_TIMEOUT

__all__ = [
    "MiniMaxTTSProvider",
    "HarnessMiniMaxTTSProvider",
    "build_minimax_tts_provider",
    "RetryPolicy",
    "TTSSettings",
    "TTSAudio",
    "TTSChunk",
    "TTSFormat",
    "MINIMAX_API_KEY_ENV",
    "MINIMAX_GROUP_ID_ENV",
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
    "DEFAULT_VOICE",
    "DEFAULT_TIMEOUT",
]

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

#: Canonical env var that holds the MiniMax API key. ADR-0001 §2.5.2
#: mandates that every secret enters the system through the OS env,
#: never as a YAML literal. :func:`build_minimax_tts_provider` reads
#: this var when no explicit ``api_key=`` is supplied.
MINIMAX_API_KEY_ENV: str = "MINIMAX_API_KEY"

#: MiniMax T2A v2 also requires a ``GroupId`` query parameter. Listed
#: separately because the upstream provider has historically read it
#: as ``MINIMAX_GROUP_ID`` and we want the harness to surface the
#: same convention to operators.
MINIMAX_GROUP_ID_ENV: str = "MINIMAX_GROUP_ID"


# ---------------------------------------------------------------------------
# Retry policy (re-exported for the harness-side TTS surface)
# ---------------------------------------------------------------------------

#: Re-export of the LLM provider's :class:`RetryPolicy` so callers
#: only need to import a single class for the harness. The semantics
#: are identical — exponential backoff with uniform jitter, applied
#: to transient errors only.
RetryPolicy = _LLMRetryPolicy


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _resolve_format(fmt: str) -> TTSFormat:
    """Convert a YAML ``format`` string to a :class:`TTSFormat`.

    The validation is also done in :func:`rob_box_harness.config._parse_tts`,
    but the wrapper is callable directly (without going through the
    config loader), so we re-validate here for defence in depth.
    """
    try:
        return TTSFormat(fmt)
    except ValueError as exc:
        raise ConfigError(
            f"minimax_tts: unsupported format {fmt!r}; "
            f"expected one of {[f.value for f in TTSFormat]}",
            section="tts.format",
        ) from exc


def _hash_audio_key(
    text: str,
    voice: str | None,
    fmt: TTSFormat,
    sample_rate: int | None,
    model: str | None,
) -> str:
    """Stable cache key for a (text, voice, format, sample_rate, model) tuple.

    The key is a hex SHA-256 of the joined string. We use SHA-256
    rather than ``hash()`` because ``hash()`` is randomised per
    process (PYTHONHASHSEED) and would invalidate the cache across
    restarts.
    """
    parts = (
        text,
        voice or "",
        fmt.value,
        str(sample_rate or 0),
        model or "",
    )
    raw = "|".join(parts).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


# ---------------------------------------------------------------------------
# Harness-side TTS provider
# ---------------------------------------------------------------------------


class HarnessMiniMaxTTSProvider(TTSProvider):  # type: ignore[misc]
    """Harness-friendly wrapper around :class:`MiniMaxTTSProvider`.

    The class is itself a :class:`TTSProvider` — it ``IS-A`` the
    contract — so it can be passed to any consumer that type-annotates
    the upstream port. The harness gains:

    * automatic env-based auth (``MINIMAX_API_KEY`` +
      ``MINIMAX_GROUP_ID``),
    * exponential-backoff retries on transient errors,
    * an optional content-hash cache that suppresses duplicate
      ``synthesize`` calls.

    The underlying transport (``httpx.AsyncClient``) is composed from
    :class:`rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider`, so
    all of the upstream contract (capabilities, voice catalogue,
    health check, payload builder, HTTP factory, API-key redaction,
    ``aclose``) is inherited unchanged.

    Parameters
    ----------
    api_key:
        MiniMax API key. If ``None``, the provider reads
        ``MINIMAX_API_KEY`` from the env. The key is never logged.
    group_id:
        MiniMax ``GroupId`` query parameter. If ``None``, the
        provider reads ``MINIMAX_GROUP_ID`` from the env.
    model:
        Default model name (overridable per call via
        :class:`TTSSettings`). Defaults to ``"speech-02-hd"``.
    base_url:
        Endpoint root. Defaults to ``https://api.minimax.io``.
    default_voice:
        Voice id used when ``TTSSettings.voice`` is ``None``.
    timeout:
        Per-request timeout in seconds. Forwarded to ``httpx``.
    retry:
        :class:`RetryPolicy` for transient failures. Pass
        ``RetryPolicy(max_attempts=1)`` to disable retries.
    client:
        Optional pre-built :class:`httpx.AsyncClient`. Useful for
        tests that need to inject a mock transport.
    cache:
        When ``True``, repeated ``synthesize`` calls with identical
        ``(text, voice, format, sample_rate, model)`` return the
        cached :class:`TTSAudio` without hitting the API. Caching is
        best-effort: there is no eviction policy, and the cache is
        held in-process. Use it for short-lived responses (acks,
        error stubs) where a few hundred entries are expected; do
        not use it for arbitrary user speech.
    rate_limit_per_min:
        Soft cap on requests per minute. ``None`` (default) disables
        throttling. When set, the wrapper sleeps before the request
        if the rolling-window counter would exceed the cap. The
        counter resets every 60 seconds; resets are *not* aligned
        to wall-clock minutes — they are relative to the first call.
    """

    name = "minimax"

    def __init__(
        self,
        *,
        api_key: str | None = None,
        group_id: str | None = None,
        model: str = DEFAULT_MODEL,
        base_url: str = DEFAULT_BASE_URL,
        default_voice: str = DEFAULT_VOICE,
        timeout: float = DEFAULT_TIMEOUT,
        retry: RetryPolicy | None = None,
        client: httpx.AsyncClient | None = None,
        cache: bool = False,
        rate_limit_per_min: int | None = None,
    ) -> None:
        # Resolve credentials from env. The harness refuses to
        # silently fall back to a YAML-embedded string — that's
        # the M7 contract from ADR-0001 §2.5 + the parallel TTS
        # rule from the task body.
        resolved_key = api_key or os.environ.get(MINIMAX_API_KEY_ENV)
        if not resolved_key:
            raise ConfigError(
                f"minimax_tts: missing API key; set the {MINIMAX_API_KEY_ENV} "
                "env var or pass api_key= explicitly",
                section="tts.api_key",
            )
        resolved_group = group_id or os.environ.get(MINIMAX_GROUP_ID_ENV)
        if not resolved_group:
            raise ConfigError(
                f"minimax_tts: missing GroupId; set the {MINIMAX_GROUP_ID_ENV} "
                "env var or pass group_id= explicitly",
                section="tts.group_id",
            )

        # Build the upstream provider. We pass through every
        # constructor knob so callers get the full surface without
        # re-implementing it here.
        self._inner: _UpstreamMiniMaxTTSProvider = _UpstreamMiniMaxTTSProvider(
            base_url=base_url,
            api_key=resolved_key,
            group_id=resolved_group,
            default_voice=default_voice,
            default_model=model,
            timeout=timeout,
            client=client,
        )
        self._retry: RetryPolicy = retry or RetryPolicy()
        # Track a close flag so ``aclose`` is idempotent. The inner
        # provider closes its own client; we just memoize here.
        self._closed: bool = False

        # Optional content-hash cache. The dict maps SHA-256 hex to
        # :class:`TTSAudio`. We deliberately keep it simple (no
        # eviction, no LRU) — the cache is for short-lived responses
        # only, and tests can clear it via ``_clear_cache()``.
        self._cache_enabled: bool = bool(cache)
        self._cache: dict[str, TTSAudio] = {}

        # Soft rate limiter. ``_rate_limit_window_start`` is the
        # monotonic timestamp of the current 60-second window;
        # ``_rate_limit_calls`` is the count so far. When the
        # window rolls over, the counter resets.
        self._rate_limit_per_min: int | None = (
            int(rate_limit_per_min) if rate_limit_per_min is not None else None
        )
        self._rate_limit_window_start: float = 0.0
        self._rate_limit_calls: int = 0

    # ---- capability introspection ----------------------------------------

    @property
    def capabilities(self) -> TTSCapabilities:
        """Forward to the upstream provider.

        The upstream ``MiniMaxTTSProvider`` exposes streaming (SSE),
        voice cloning (via ``timbre_weights``), audio_format_pcm +
        audio_format_mp3. ``ssml`` and ``audio_format_ogg`` are
        intentionally ``False`` — see the upstream class docstring.
        """
        return self._inner.capabilities()

    @property
    def default_voice(self) -> str:
        """Return the default voice id (used when ``TTSSettings.voice`` is ``None``)."""
        voice: str = self._inner._default_voice  # noqa: SLF001 — internal but stable
        return voice

    @property
    def default_model(self) -> str:
        """Return the default model name."""
        model: str = self._inner._default_model  # noqa: SLF001 — internal but stable
        return model

    async def list_voices(self) -> list[TTSVoice]:
        """Return the MiniMax built-in voice catalogue.

        Forwarded to the upstream provider; the catalogue is a static
        list (no upstream call) until MiniMax ships a public
        ``/v1/voices`` endpoint.
        """
        voices: list[TTSVoice] = await self._inner.list_voices()
        return voices

    async def healthcheck(self) -> TTSHealth:
        """Cheap pre-flight: verify both credentials are configured.

        Does NOT call upstream — same contract as the upstream
        provider's :meth:`healthcheck`.
        """
        return await self._inner.healthcheck()

    # ---- TTSProvider contract -------------------------------------------

    async def synthesize(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> TTSAudio:
        """Run a non-streaming synthesis with retries + optional cache.

        Retries on :class:`TTSRateLimitError` /
        :class:`TTSTimeoutError` only. Programming errors
        (:class:`TTSAuthError` / :class:`TTSBadRequestError`)
        propagate immediately.
        """
        effective = settings or TTSSettings()
        cache_key: str | None = None
        if self._cache_enabled:
            cache_key = _hash_audio_key(
                text,
                effective.voice,
                effective.format,
                effective.sample_rate,
                effective.model,
            )
            cached = self._cache.get(cache_key)
            if cached is not None:
                _log.debug(
                    "minimax_tts: cache hit for key=%s (voice=%s, format=%s)",
                    cache_key[:8],
                    effective.voice,
                    effective.format.value,
                )
                return cached

        await self._rate_limit_acquire()

        async def _call() -> TTSAudio:
            return await self._inner.synthesize(text, settings=effective)

        audio = await self._call_with_retry(_call)

        if cache_key is not None:
            self._cache[cache_key] = audio
        return audio

    async def stream(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> AsyncIterator[TTSChunk]:
        """Run a streaming synthesis.

        Like :meth:`LLMProvider.stream`, the upstream contract raises
        a :class:`TTSError` BEFORE yielding anything if the initial
        request fails. We therefore retry on the initial call only;
        past the first chunk, errors propagate (mirrors the LLM
        contract).
        """
        effective = settings or TTSSettings()
        await self._rate_limit_acquire()

        attempts = 0
        last_exc: BaseException | None = None
        while attempts < self._retry.max_attempts:
            attempts += 1
            try:
                inner_stream = self._inner.stream(text, settings=effective)
                # Probe the first chunk to surface a transient
                # initial-request error as a retriable exception.
                first_chunk = await inner_stream.__anext__()
                queue: asyncio.Queue[TTSChunk | BaseException | None] = asyncio.Queue()

                async def _replay() -> AsyncIterator[TTSChunk]:
                    yield first_chunk
                    try:
                        while True:
                            chunk = await inner_stream.__anext__()
                            yield chunk
                    except StopAsyncIteration:
                        return

                replay_iter = _replay()

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
                await task
                return
            except (TTSRateLimitError, TTSTimeoutError) as exc:
                last_exc = exc
                if attempts >= self._retry.max_attempts:
                    raise
                delay = self._retry.delay_for(attempts)
                _log.warning(
                    "minimax_tts: transient error on stream attempt %d/%d (%s); "
                    "retrying in %.2fs",
                    attempts,
                    self._retry.max_attempts,
                    type(exc).__name__,
                    delay,
                )
                await asyncio.sleep(delay)
                continue
        if last_exc is not None:  # pragma: no cover — unreachable
            raise last_exc
        raise RuntimeError("minimax_tts.stream: exhausted retries without exception")

    async def aclose(self) -> None:
        """Tear down the underlying HTTP client.

        Idempotent: calling ``aclose`` twice is a no-op. After
        ``aclose`` the provider must not be reused.
        """
        if self._closed:
            return
        self._closed = True
        await self._inner.aclose()

    # ---- cache helpers (test-only) --------------------------------------

    def _clear_cache(self) -> None:
        """Empty the in-memory synthesis cache. Test-only helper.

        Production code should not call this — the cache is
        self-managing by virtue of being short-lived.
        """
        self._cache.clear()

    def _cache_size(self) -> int:
        """Return the number of cached synthesis results."""
        return len(self._cache)

    # ---- rate limiter ----------------------------------------------------

    async def _rate_limit_acquire(self) -> None:
        """Sleep if the rolling 60-second window is full.

        When ``self._rate_limit_per_min`` is ``None`` this is a no-op.
        When set, the call blocks for the remainder of the current
        window if the counter has reached the cap, then resets the
        counter.
        """
        if self._rate_limit_per_min is None:
            return
        now = time.monotonic()
        if self._rate_limit_window_start == 0.0:
            self._rate_limit_window_start = now
        elapsed = now - self._rate_limit_window_start
        if elapsed >= 60.0:
            # Window rolled over — reset.
            self._rate_limit_window_start = now
            self._rate_limit_calls = 0
        if self._rate_limit_calls >= self._rate_limit_per_min:
            sleep_s = 60.0 - elapsed
            if sleep_s > 0:
                _log.debug(
                    "minimax_tts: rate limit %d/min reached; sleeping %.2fs",
                    self._rate_limit_per_min,
                    sleep_s,
                )
                await asyncio.sleep(sleep_s)
            self._rate_limit_window_start = time.monotonic()
            self._rate_limit_calls = 0
        self._rate_limit_calls += 1

    # ---- retry helper ---------------------------------------------------

    async def _call_with_retry(self, fn: Any) -> TTSAudio:
        attempts = 0
        last_exc: BaseException | None = None
        while attempts < self._retry.max_attempts:
            attempts += 1
            try:
                return await fn()
            except (TTSRateLimitError, TTSTimeoutError) as exc:
                last_exc = exc
                if attempts >= self._retry.max_attempts:
                    raise
                delay = self._retry.delay_for(attempts)
                _log.warning(
                    "minimax_tts: transient error on attempt %d/%d (%s); "
                    "retrying in %.2fs",
                    attempts,
                    self._retry.max_attempts,
                    type(exc).__name__,
                    delay,
                )
                await asyncio.sleep(delay)
        if last_exc is not None:  # pragma: no cover — unreachable
            raise last_exc
        raise RuntimeError("minimax_tts: exhausted retries without exception")


# ---------------------------------------------------------------------------
# Public alias
# ---------------------------------------------------------------------------

# The task body asks for a class named ``MiniMaxTTSProvider``. The
# harness owns the class layout, so we publish the wrapper under that
# canonical name. The upstream provider remains importable as
# ``rob_box_harness.tts.minimax_tts._UpstreamMiniMaxTTSProvider`` for
# callers that need direct access (advanced tests, vendor extensions).
MiniMaxTTSProvider = HarnessMiniMaxTTSProvider


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def build_minimax_tts_provider(
    tts_config: TTSConfig,
    *,
    env: Mapping[str, str] | None = None,
    retry: RetryPolicy | None = None,
    client: httpx.AsyncClient | None = None,
) -> MiniMaxTTSProvider:
    """Build a :class:`MiniMaxTTSProvider` from a :class:`TTSConfig`.

    The API key and ``GroupId`` are resolved entirely from ``env``
    (which defaults to ``os.environ``); the YAML / TTSConfig layer is
    not allowed to embed a literal. If ``tts_config.api_key`` is set,
    it must resolve to a non-empty string via ``env``; otherwise the
    factory raises :class:`ConfigError`.

    Parameters
    ----------
    tts_config:
        Parsed ``tts`` section of the harness config.
    env:
        Explicit env map for tests. ``None`` (default) reads
        ``os.environ``. Secrets are looked up here first, then
        ``os.environ`` — which means tests can isolate the env
        without monkey-patching the global ``os.environ``.
    retry:
        Override the default retry policy. ``None`` uses the
        default ``RetryPolicy(max_attempts=3, backoff_base=0.5)``.
    client:
        Pre-built ``httpx.AsyncClient`` (for tests with a mock
        transport).

    Raises
    ------
    ConfigError
        If ``tts_config.provider`` is not ``"minimax"``, or the API
        key / GroupId is missing from both env and the explicit
        ``api_key=`` / ``group_id=`` form of the config.
    """
    if tts_config is None:
        raise ConfigError(
            "minimax_tts: cannot build provider from None config",
            section="tts",
        )
    if tts_config.provider != "minimax":
        raise ConfigError(
            f"minimax_tts: expected tts.provider='minimax', got "
            f"{tts_config.provider!r}",
            section="tts.provider",
        )

    # Resolve credentials. The harness never hardcodes; if the env
    # is missing the key, we fail loudly at startup instead of at
    # first request.
    env_map: Mapping[str, str] = env if env is not None else os.environ
    api_key: str | None = tts_config.api_key
    if api_key is None:
        api_key = env_map.get(MINIMAX_API_KEY_ENV)
    if not api_key:
        raise ConfigError(
            f"minimax_tts: missing API key; set the {MINIMAX_API_KEY_ENV} "
            "env var or pass it via tts.api_key (which itself must "
            "resolve to ${" + MINIMAX_API_KEY_ENV + "})",
            section="tts.api_key",
        )

    group_id: str | None = env_map.get(MINIMAX_GROUP_ID_ENV)
    if not group_id:
        raise ConfigError(
            f"minimax_tts: missing GroupId; set the {MINIMAX_GROUP_ID_ENV} "
            "env var",
            section="tts.group_id",
        )

    # Translate TTSConfig knobs into the constructor signature.
    timeout = (
        float(tts_config.timeout_s)
        if tts_config.timeout_s is not None
        else DEFAULT_TIMEOUT
    )
    model = tts_config.model or DEFAULT_MODEL
    default_voice = tts_config.voice or DEFAULT_VOICE

    return MiniMaxTTSProvider(
        api_key=api_key,
        group_id=group_id,
        model=model,
        default_voice=default_voice,
        timeout=timeout,
        retry=retry,
        client=client,
        cache=tts_config.cache,
        rate_limit_per_min=tts_config.rate_limit_per_min,
    )
