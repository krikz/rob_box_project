"""Concrete providers for the OpenAI-compatible APIs rob_box talks to.

Both DeepSeek and MiMo speak the OpenAI Chat Completions protocol, so the
implementation is shared via ``_OpenAICompatibleProvider`` below. Each public
class is just a thin wrapper that locks in the right `base_url`, `default_model`
and `name`.

We deliberately do NOT touch the existing `ProviderManager` in
``rob_box_voice/llm/provider_manager.py`` — the new module is additive per the
P0 plan ("Что НЕ делаем: не трогаем существующих потребителей, только добавляем
новый модуль"). Migration of existing callers happens in P1.
"""

from __future__ import annotations

import logging
from typing import Any, AsyncIterator, Iterable, Mapping, Optional

from openai import (
    APIConnectionError,
    APIStatusError,
    APITimeoutError,
    AsyncOpenAI,
    AuthenticationError,
)

from rob_box_llm.errors import (
    AuthError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ToolCall,
)


_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _to_openai_messages(messages: Iterable[LLMMessage]) -> list[dict[str, Any]]:
    """Translate our value objects into OpenAI Chat-Completions dicts."""
    out: list[dict[str, Any]] = []
    for msg in messages:
        if msg.role == "tool" and msg.tool_result is not None:
            out.append(
                {
                    "role": "tool",
                    "tool_call_id": msg.tool_result.tool_call_id,
                    "content": msg.tool_result.content,
                }
            )
            continue

        m: dict[str, Any] = {"role": msg.role, "content": msg.content}
        if msg.name:
            m["name"] = msg.name
        if msg.tool_calls:
            m["tool_calls"] = [
                {
                    "id": tc.id,
                    "type": "function",
                    "function": {
                        "name": tc.name,
                        "arguments": _json_dumps(tc.arguments),
                    },
                }
                for tc in msg.tool_calls
            ]
        out.append(m)
    return out


def _json_dumps(obj: Any) -> str:  # local to avoid hard json dep at module load
    import json

    return json.dumps(obj, ensure_ascii=False)


def _map_exception(exc: Exception, *, provider: str) -> ProviderError:
    """Map the openai SDK exception tree onto our domain errors."""
    if isinstance(exc, AuthenticationError):
        return AuthError(str(exc), provider=provider)
    if isinstance(exc, APITimeoutError):
        return TimeoutError(str(exc), provider=provider)
    if isinstance(exc, APIConnectionError):
        # Connection-level failure — also a "try again later" condition.
        return TimeoutError(str(exc), provider=provider)
    if isinstance(exc, APIStatusError):
        status = getattr(exc, "status_code", None)
        body = getattr(exc, "body", None) or {}
        # OpenAI uses 429 for rate-limit; some providers also surface quota
        # limits as 403. We treat both as RateLimitError.
        if status in (429, 403):
            return RateLimitError(f"{status}: {body}", provider=provider)
        # 400 with content_filter / content_policy is the OpenAI shape; some
        # providers put it on 422 / 451. Heuristic: look for the keywords.
        text = (body.get("error", {}).get("message", "") if isinstance(body, dict) else str(body)).lower()
        if "content" in text and ("filter" in text or "policy" in text or "safety" in text):
            return ContentFilterError(f"{status}: {body}", provider=provider)
        return ProviderError(f"{status}: {body}", provider=provider)
    return ProviderError(str(exc), provider=provider)


# ---------------------------------------------------------------------------
# Shared base
# ---------------------------------------------------------------------------


class _OpenAICompatibleProvider(LLMProvider):
    """Shared implementation for any OpenAI Chat-Completions compatible API."""

    def __init__(
        self,
        *,
        name: str,
        base_url: str,
        default_model: str,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        client: Optional[AsyncOpenAI] = None,
    ) -> None:
        self.name = name
        self._base_url = base_url
        self._default_model = default_model
        if client is not None:
            self._client = client
        else:
            self._client = AsyncOpenAI(
                base_url=base_url,
                api_key=api_key or "no-key-configured",
                timeout=timeout,
            )

    # -- helpers -----------------------------------------------------------

    def _build_kwargs(
        self,
        messages: Iterable[LLMMessage],
        tools: Iterable[Mapping[str, Any]],
        settings: LLMSettings | None,
        *,
        stream: bool,
    ) -> dict[str, Any]:
        s = settings or LLMSettings()
        kwargs: dict[str, Any] = {
            "model": s.model or self._default_model,
            "messages": _to_openai_messages(messages),
            "stream": stream,
        }
        if s.temperature is not None:
            kwargs["temperature"] = s.temperature
        if s.max_tokens is not None:
            kwargs["max_tokens"] = s.max_tokens
        if s.stop:
            kwargs["stop"] = list(s.stop)
        if s.tool_choice is not None:
            kwargs["tool_choice"] = s.tool_choice
        if s.extra:
            kwargs.update(s.extra)
        if tools:
            kwargs["tools"] = [dict(t) for t in tools]
        return kwargs

    @staticmethod
    def _usage_from(resp: Any) -> dict[str, int]:
        u = getattr(resp, "usage", None)
        if u is None:
            return {}
        out: dict[str, int] = {}
        for attr in ("prompt_tokens", "completion_tokens", "total_tokens"):
            v = getattr(u, attr, None)
            if isinstance(v, int):
                out[attr] = v
        return out

    # -- complete ----------------------------------------------------------

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        kwargs = self._build_kwargs(messages, tools, settings, stream=False)
        try:
            resp = await self._client.chat.completions.create(**kwargs)
        except Exception as exc:  # noqa: BLE001 — convert to our domain errors
            raise _map_exception(exc, provider=self.name) from exc

        choice = resp.choices[0] if resp.choices else None
        content = choice.message.content if choice and choice.message else ""
        tool_calls: tuple[ToolCall, ...] = ()
        if choice and choice.message and choice.message.tool_calls:
            tool_calls = tuple(
                ToolCall(
                    id=tc.id,
                    name=tc.function.name,
                    arguments=_safe_json(tc.function.arguments),
                )
                for tc in choice.message.tool_calls
                if tc.function
            )
        finish = choice.finish_reason if choice else None
        return LLMResponse(
            content=content or "",
            tool_calls=tool_calls,
            finish_reason=finish,
            usage=self._usage_from(resp),
            raw=resp,
        )

    # -- stream ------------------------------------------------------------

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        kwargs = self._build_kwargs(messages, tools, settings, stream=True)
        try:
            stream_obj = await self._client.chat.completions.create(**kwargs)
        except Exception as exc:  # noqa: BLE001
            raise _map_exception(exc, provider=self.name) from exc

        async for event in stream_obj:
            choice = event.choices[0] if event.choices else None
            delta = choice.delta if choice else None
            chunk = LLMChunk(
                content_delta=getattr(delta, "content", "") or "",
                finish_reason=getattr(choice, "finish_reason", None),
            )
            yield chunk
            if chunk.finish_reason:
                return

    async def aclose(self) -> None:
        await self._client.close()


def _safe_json(raw: Any) -> dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    if not isinstance(raw, str) or not raw:
        return {}
    import json

    try:
        return json.loads(raw)
    except Exception:  # noqa: BLE001
        _log.warning("Could not parse tool-call arguments JSON: %r", raw)
        return {}


# ---------------------------------------------------------------------------
# Public provider classes
# ---------------------------------------------------------------------------


class DeepSeekProvider(_OpenAICompatibleProvider):
    """DeepSeek Chat (OpenAI-compatible).

    Default base URL: ``https://api.deepseek.com``.
    Default model:    ``deepseek-chat``.
    """

    DEFAULT_BASE_URL = "https://api.deepseek.com"
    DEFAULT_MODEL = "deepseek-chat"

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_BASE_URL,
        api_key: Optional[str] = None,
        model: str = DEFAULT_MODEL,
        timeout: float = 30.0,
        client: Optional[AsyncOpenAI] = None,
    ) -> None:
        super().__init__(
            name="deepseek",
            base_url=base_url,
            default_model=model,
            api_key=api_key,
            timeout=timeout,
            client=client,
        )
