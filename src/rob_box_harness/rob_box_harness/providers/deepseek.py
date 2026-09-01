"""``DeepSeekProvider`` — the harness-side DeepSeek LLM-provider.

This module is the **integration layer** between the
:class:`rob_box_harness.config.HarnessConfig` schema and the
production-grade :class:`rob_box_llm.providers.deepseek.DeepSeekProvider`
that already implements the full :class:`rob_box_llm.LLMProvider`
contract.

The class provided here adds three harness-side affordances on top of
the upstream provider, mirroring :class:`HarnessMiniMaxProvider`:

1. **Authentication via env only** — the API key is read from
   ``DEEPSEEK_API_KEY`` (or supplied explicitly via ``api_key=``). The
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
``name = "deepseek"`` so the harness registry's fallback chain
(``[deepseek, minimax, mimo]``) can pick it by name.
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Any, AsyncIterator, Iterable, Mapping

from openai import AsyncOpenAI

from rob_box_harness.errors import ConfigError
# Ретрай-политика — одна на харнес (providers/retry.py). Раньше здесь
# лежала своя копия класса, дословно совпадавшая с копией в minimax.py,
# но БУДУЧИ другим объектом (карточка W6-1). Имя оставлено в модуле:
# по нему ходят providers/__init__.py, providers/mimo.py и тесты.
from rob_box_harness.providers.retry import RetryPolicy
from rob_box_llm.errors import (
    AuthError,
    CapabilityUnavailableError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.providers.deepseek import DeepSeekProvider as _UpstreamDeepSeekProvider
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
)

DEFAULT_BASE_URL: str = _UpstreamDeepSeekProvider.DEFAULT_BASE_URL
DEFAULT_MODEL: str = _UpstreamDeepSeekProvider.DEFAULT_MODEL

__all__ = [
    "DeepSeekProvider",
    "HarnessDeepSeekProvider",
    "build_deepseek_provider",
    "RetryPolicy",
    "DEEPSEEK_API_KEY_ENV",
    # Re-exports
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
]

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

#: Canonical env var that holds the DeepSeek API key. ADR-0001 §2.5.2
#: mandates that every secret enters the system through the OS env,
#: never as a YAML literal. ``build_deepseek_provider`` reads this var
#: when no explicit ``api_key=`` is supplied.
DEEPSEEK_API_KEY_ENV: str = "DEEPSEEK_API_KEY"


# ---------------------------------------------------------------------------
# Harness-side DeepSeekProvider
# ---------------------------------------------------------------------------


class HarnessDeepSeekProvider(LLMProvider):  # type: ignore[misc]
    """Harness-friendly wrapper around :class:`DeepSeekProvider`.

    The class is itself an :class:`LLMProvider` — it ``IS-A`` the
    contract — so it can be passed directly to ``harness.llm`` like
    any other provider. The harness gains:

    * automatic env-based auth (``DEEPSEEK_API_KEY``),
    * a friendly ``chat(messages, **kwargs)`` shortcut,
    * exponential-backoff retries on transient errors,
    * idempotent ``aclose()``.

    Parameters
    ----------
    api_key:
        The DeepSeek API key. If ``None``, the provider reads
        ``DEEPSEEK_API_KEY`` from ``os.environ``. The key is never
        logged.
    model:
        Default model name. Defaults to ``"deepseek-chat"``.
    base_url:
        Endpoint URL. Defaults to ``https://api.deepseek.com``.
    timeout:
        Per-request timeout in seconds. Forwarded to the OpenAI SDK.
    retry:
        :class:`RetryPolicy` for transient failures. Pass
        ``RetryPolicy(max_attempts=1)`` to disable retries.
    client:
        Optional pre-built ``AsyncOpenAI`` client. Useful for tests
        that need to inject a mock transport.
    """

    name = "deepseek"

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
        resolved_key = api_key or os.environ.get(DEEPSEEK_API_KEY_ENV)
        if not resolved_key:
            raise ConfigError(
                f"deepseek: missing API key; set the {DEEPSEEK_API_KEY_ENV} "
                "env var or pass api_key= explicitly",
                section="llm.api_key",
            )

        self._inner: _UpstreamDeepSeekProvider = _UpstreamDeepSeekProvider(
            base_url=base_url,
            api_key=resolved_key,
            model=model,
            timeout=timeout,
            client=client,
        )
        self._retry: RetryPolicy = retry or RetryPolicy()
        self._closed: bool = False

    # ---- capability introspection ----------------------------------------

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Forward to the upstream provider."""
        return self._inner.capabilities

    def capabilities_for(self, model: str | None) -> ProviderCapabilities:
        """Capabilities narrowed to a specific model name."""
        return self._inner.capabilities_for(model)

    # ---- LLMProvider contract -------------------------------------------

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        """Run a non-streaming completion with retries."""
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
        """Run a streaming completion with retry on the initial request."""
        attempts = 0
        last_exc: BaseException | None = None
        while attempts < self._retry.max_attempts:
            attempts += 1
            try:
                inner_stream = self._inner.stream(
                    messages, tools=tools, settings=settings
                )
                first_chunk = await inner_stream.__anext__()
                queue: asyncio.Queue[LLMChunk | BaseException | None] = asyncio.Queue()
                queue.put_nowait(first_chunk)

                async def _replay() -> AsyncIterator[LLMChunk]:
                    while True:
                        item = await queue.get()
                        if item is None:
                            return
                        if isinstance(item, BaseException):
                            raise item
                        yield item

                async def _drain() -> None:
                    try:
                        async for chunk in inner_stream:
                            await queue.put(chunk)
                    except BaseException as exc:  # noqa: BLE001
                        await queue.put(exc)
                    finally:
                        await queue.put(None)

                drain_task = asyncio.create_task(_drain())
                try:
                    async for chunk in _replay():
                        yield chunk
                finally:
                    # 🔴 FIX (issue #1280): barge-in — consumer отменён
                    # (CancelledError) или генератор закрыт (aclose).
                    # Раньше drain_task только cancel()'ился без await:
                    # если он в этот момент был между чанками
                    # (await queue.put), CancelledError не доходил до
                    # inner_stream — HTTP-запрос к провайдеру жил до
                    # конца генерации (трата квоты на старую тему,
                    # «робот добивает старую тему после смены»).
                    # Теперь дожидаемся drain_task и принудительно
                    # закрываем inner_stream — соединение рвётся сразу.
                    if not drain_task.done():
                        drain_task.cancel()
                    try:
                        await drain_task
                    except BaseException:
                        pass
                    aclose = getattr(inner_stream, "aclose", None)
                    if aclose is not None:
                        try:
                            await aclose()
                        except Exception:
                            pass
                return
            except (RateLimitError, TimeoutError) as exc:
                last_exc = exc
                if attempts >= self._retry.max_attempts:
                    raise
                await asyncio.sleep(self._retry.delay_for(attempts))
                continue
        # Defensive — loop should have returned or raised.
        if last_exc is not None:
            raise last_exc
        raise ProviderError("deepseek: stream() exited unexpectedly")

    # ---- convenience chat() shortcut ------------------------------------

    async def chat(
        self,
        messages: Iterable[LLMMessage],
        *,
        temperature: float | None = None,
        max_tokens: int | None = None,
        top_p: float | None = None,
        tools: Iterable[Mapping[str, Any]] = (),
    ) -> LLMResponse:
        """Single-turn chat shortcut.

        Builds an :class:`LLMSettings` from the kwargs and forwards to
        :meth:`complete`. Equivalent to the upstream convenience.
        """
        settings_kwargs: dict[str, Any] = {}
        if temperature is not None:
            settings_kwargs["temperature"] = temperature
        if max_tokens is not None:
            settings_kwargs["max_tokens"] = max_tokens
        if top_p is not None:
            settings_kwargs["top_p"] = top_p
        settings = LLMSettings(**settings_kwargs) if settings_kwargs else None
        return await self.complete(messages, tools=tools, settings=settings)

    # ---- internal helpers -----------------------------------------------

    async def _call_with_retry(
        self,
        method: Any,
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
                return await method(messages, tools=tools, settings=settings)
            except (RateLimitError, TimeoutError) as exc:
                last_exc = exc
                if attempts >= self._retry.max_attempts:
                    raise
                await asyncio.sleep(self._retry.delay_for(attempts))
                continue
        if last_exc is not None:
            raise last_exc
        raise ProviderError("deepseek: retry loop exited unexpectedly")

    # ---- lifecycle ------------------------------------------------------

    async def aclose(self) -> None:
        """Close the underlying client. Idempotent."""
        if self._closed:
            return
        await self._inner.aclose()
        self._closed = True


# ---------------------------------------------------------------------------
# Factory and aliases
# ---------------------------------------------------------------------------


def build_deepseek_provider(
    *,
    api_key: str | None = None,
    model: str = DEFAULT_MODEL,
    base_url: str = DEFAULT_BASE_URL,
    timeout: float = 30.0,
    retry: RetryPolicy | None = None,
    client: AsyncOpenAI | None = None,
) -> HarnessDeepSeekProvider:
    """Build a :class:`HarnessDeepSeekProvider` from explicit kwargs.

    All kwargs are optional — ``api_key`` falls back to the
    ``DEEPSEEK_API_KEY`` env var, and the rest default to upstream
    constants.
    """
    return HarnessDeepSeekProvider(
        api_key=api_key,
        model=model,
        base_url=base_url,
        timeout=timeout,
        retry=retry,
        client=client,
    )


#: Backwards-compatible alias. Earlier code refers to ``DeepSeekProvider``
#: as the public harness-side class; the upstream is now imported as
#: ``_UpstreamDeepSeekProvider`` for clarity.
DeepSeekProvider = HarnessDeepSeekProvider