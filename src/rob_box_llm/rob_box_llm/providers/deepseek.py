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

import base64
import logging
import os
from typing import Any, AsyncIterator, Iterable, Mapping, Optional, Union

from openai import (
    APIConnectionError,
    APIStatusError,
    APITimeoutError,
    AsyncOpenAI,
    AuthenticationError,
)

from rob_box_llm.errors import (
    AuthError,
    CapabilityUnavailableError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.provider import (
    ImagePart,
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
    TextPart,
    ToolCall,
)

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _image_part_to_openai(part: ImagePart) -> dict[str, Any]:
    """Render an ``ImagePart`` as an OpenAI ``image_url`` content block.

    Bytes are base64-encoded into a ``data:`` URL. String sources are passed
    through unchanged (assumed to already be ``http(s)://`` or ``data:...``).
    ``detail`` is forwarded only when it is not the default — this keeps the
    wire format minimal for callers that don't care.
    """
    if isinstance(part.source, bytes):
        b64 = base64.b64encode(part.source).decode("ascii")
        url = f"data:{part.media_type};base64,{b64}"
    else:
        url = part.source

    image_url: dict[str, Any] = {"url": url}
    if part.detail != "default":
        image_url["detail"] = part.detail
    return {"type": "image_url", "image_url": image_url}


def _content_to_openai(content: Any) -> Any:
    """Translate our ``MessageContent`` into the OpenAI wire format.

    Plain strings pass through. Tuples of ``TextPart``/``ImagePart`` become
    the OpenAI content-parts list. ``tool`` messages that go through this
    function still arrive as plain strings (their payload lives in
    ``tool_result.content``), so this branch is not exercised for them.
    """
    if isinstance(content, str):
        return content
    out: list[dict[str, Any]] = []
    for part in content:
        if isinstance(part, TextPart):
            out.append({"type": "text", "text": part.text})
        elif isinstance(part, ImagePart):
            out.append(_image_part_to_openai(part))
        else:  # pragma: no cover — guarded by the type alias
            raise TypeError(f"Unsupported message part: {part!r}")
    return out


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

        m: dict[str, Any] = {
            "role": msg.role,
            "content": _content_to_openai(msg.content),
        }
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
    from types import MappingProxyType

    # ``ToolCall.arguments`` is frozen into a ``MappingProxyType`` in
    # ``provider.ToolCall.__post_init__`` so the dataclass stays truly
    # immutable. ``json.dumps`` does not know how to serialise proxy
    # views (it raises ``TypeError: Object of type MappingProxyType is
    # not JSON serializable``), so unwrap to a plain ``dict`` first.
    # Same applies to any other read-only mapping view that might arrive
    # (e.g. ``types.MappingProxyType`` returned by ``Mapping`` subclasses
    # in tests). Nested proxies are handled by ``json.dumps`` itself
    # because the unwrap creates a fully mutable plain-dict tree.
    if isinstance(obj, MappingProxyType):
        obj = dict(obj)
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

    # Subclasses set ``_CAPABILITIES`` to advertise what they support.
    _CAPABILITIES: ProviderCapabilities = ProviderCapabilities(
        text=True,
        streaming_text=True,
        tools=True,
        # 🔴 FIX (live 06.08): streaming_tools=True — OpenAI-совместимый API
        # DeepSeek/MiniMax стримит tool-call deltas (id/name/arguments
        # фрагментами), stream() теперь агрегирует их в ToolCall.
        streaming_tools=True,
        image_input=False,
    )

    def __init__(
        self,
        *,
        name: str,
        base_url: str,
        default_model: str,
        api_key: Optional[str] = None,
        # Accept either a bare ``float`` (all phases) or a per-phase
        # :class:`httpx.Timeout` — see ``MiniMaxProvider.DEFAULT_TIMEOUT``
        # for the BLK-5 rationale (per-phase defaults are needed so a
        # DNS/TLS hang on the connect phase doesn't burn the whole
        # 30 s budget). OpenAI's ``AsyncOpenAI(timeout=...)`` accepts
        # both shapes natively, so we pass through unchanged.
        timeout: Union[float, "httpx.Timeout", None] = 30.0,
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

    # -- capability introspection -----------------------------------------

    @property
    def capabilities(self) -> ProviderCapabilities:
        return self._CAPABILITIES

    # -- helpers -----------------------------------------------------------

    def _require_capability_for_messages(
        self,
        messages: Iterable[LLMMessage],
        settings: LLMSettings | None,
        tools: Iterable[Mapping[str, Any]],
        *,
        stream: bool,
    ) -> None:
        """Refuse the request early if the provider / model can't fulfil it.

        Image input is checked against ``capabilities_for(model)``. Tool-call
        support is checked at the adapter level (always True today, kept here
        so a future text-only cheap model can opt out). Streaming tool calls
        are not yet implemented in the wire-format adapter and are gated
        explicitly so callers don't silently lose tool calls mid-stream.
        """
        caps = self.capabilities_for(settings.model if settings is not None else None)
        # Image input.
        needs_image = any(
            isinstance(part, ImagePart) for msg in messages if not isinstance(msg.content, str) for part in msg.content
        )
        if needs_image and not caps.image_input:
            raise CapabilityUnavailableError(
                f"{self.name}: image input not supported for "
                f"model={settings.model if settings else self._default_model!r}",
                provider=self.name,
            )
        # Tools.
        if tools and not caps.tools:
            raise CapabilityUnavailableError(
                f"{self.name}: tool calling not supported by this adapter",
                provider=self.name,
            )
        # Streaming.
        if stream and not caps.streaming_text:
            raise CapabilityUnavailableError(
                f"{self.name}: streaming text not supported by this adapter",
                provider=self.name,
            )
        if stream and tools and not caps.streaming_tools:
            raise CapabilityUnavailableError(
                f"{self.name}: streaming tool calls not supported yet — " "use complete() for tool requests",
                provider=self.name,
            )

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

    def _post_process_response(self, resp: Any) -> Any:
        """Hook for provider-specific response-level error mapping.

        Default: identity. MiniMax uses this to surface the in-body
        ``base_resp.status_code`` envelope; other providers can override.
        MUST raise a typed ``ProviderError`` subclass when the call should be
        considered failed, even though the HTTP layer returned 200.
        """
        return resp

    # -- complete ----------------------------------------------------------

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        # BLK-4: freeze iterables on entry. ``_require_capability_for_messages``
        # iterates ``messages`` (any() over image parts) and ``_build_kwargs``
        # iterates both again. A one-shot generator would be empty on the
        # second pass, sending messages=[] to the SDK → 400 / empty reply.
        # tuple() is O(n) memory and removes the whole class of bugs.
        messages = tuple(messages)
        tools = tuple(tools)
        self._require_capability_for_messages(messages, settings, tools, stream=False)
        kwargs = self._build_kwargs(messages, tools, settings, stream=False)
        # 🐞 VERBOSE LLM TRACE: полный контекст, который видит модель.
        # Пишем в stderr (как rclpy) — python logging не виден в docker logs.
        # Включается env ROBOT_LLM_VERBOSE (по умолчанию 1 = всегда).
        if os.environ.get("ROBOT_LLM_VERBOSE", "1") == "1":
            import sys as _sys
            _sys.stderr.write("🔎 LLM REQUEST START\n"); _sys.stderr.flush()
            for i, m in enumerate(messages):
                role = getattr(m, "role", "?")
                content = getattr(m, "content", "")
                if isinstance(content, list):
                    content = json.dumps(content, ensure_ascii=False)[:500]
                else:
                    content = str(content)[:500]
                tc = getattr(m, "tool_calls", None)
                tc_str = ""
                if tc:
                    try:
                        tc_str = " tool_calls=" + json.dumps(
                            [{"name": t.name, "args": t.arguments} for t in tc],
                            ensure_ascii=False)[:400]
                    except Exception:
                        tc_str = f" tool_calls={tc!r}"[:400]
                _sys.stderr.write(f"  [{i}] {role}: {content!r}{tc_str}\n")
            if tools:
                _sys.stderr.write("  tools(" + str(len(tools)) + "): " + ", ".join(
                    t.get("function", {}).get("name", "?") for t in tools) + "\n")
            _sys.stderr.write("🔎 LLM REQUEST END\n"); _sys.stderr.flush()
        try:
            resp = await self._client.chat.completions.create(**kwargs)
        except Exception as exc:  # noqa: BLE001 — convert to our domain errors
            raise _map_exception(exc, provider=self.name) from exc
        self._post_process_response(resp)
        if os.environ.get("ROBOT_LLM_VERBOSE", "1") == "1":
            import sys as _sys
            try:
                ch = resp.choices[0] if resp.choices else None
                msg = ch.message if ch else None
                content = msg.content if msg else ""
                _sys.stderr.write(
                    f"🔎 LLM RESPONSE: content={str(content)[:500]!r} "
                    f"finish={ch.finish_reason if ch else None} "
                    f"tool_calls={[(tc.function.name, str(tc.function.arguments)[:200]) for tc in (msg.tool_calls if msg and msg.tool_calls else [])]!r}\n"
                )
                _sys.stderr.flush()
            except Exception as exc:  # noqa: BLE001
                _sys.stderr.write(f"🔎 LLM RESPONSE (log failed): {exc}\n")

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
        # BLK-4: see note in ``complete()`` — iterables are consumed twice
        # (_require_capability_for_messages then _build_kwargs), so a
        # generator would be empty on the second pass.
        messages = tuple(messages)
        tools = tuple(tools)
        self._require_capability_for_messages(messages, settings, tools, stream=True)
        kwargs = self._build_kwargs(messages, tools, settings, stream=True)
        try:
            stream_obj = await self._client.chat.completions.create(**kwargs)
        except Exception as exc:  # noqa: BLE001
            raise _map_exception(exc, provider=self.name) from exc

        # 🔴 FIX (live 06.08): streaming tool-call aggregation (DeepSeek/
        # MiniMax OpenAI-compatible APIs stream tool arguments token-by-token,
        # e.g. '{"lo' + 'cat' + 'ion": "Tok' + 'yo"} — MUST concatenate before
        # json.loads). Also ignore delta.reasoning_content (thinking models
        # stream inner thoughts first). Previously streaming_tools=False —
        # callers had to use complete() and lost ~20s latency on the first
        # turn (MiniMax cold start + full non-streamed response).
        pending: dict[int, dict] = {}  # index -> {id, name, arguments}
        for event in stream_obj:
            self._post_process_response(event)
            choice = event.choices[0] if event.choices else None
            delta = choice.delta if choice else None
            finish = getattr(choice, "finish_reason", None)
            if delta is not None:
                dcontent = getattr(delta, "content", None)
                if dcontent:
                    yield LLMChunk(content_delta=dcontent, finish_reason=None)
                # Aggregate tool-call deltas by index (OpenAI wire format).
                dtool_calls = getattr(delta, "tool_calls", None)
                if dtool_calls:
                    for tc in dtool_calls:
                        idx = getattr(tc, "index", 0) or 0
                        slot = pending.setdefault(idx, {"id": "", "name": "", "arguments": ""})
                        if getattr(tc, "id", None):
                            slot["id"] = tc.id
                        fn = getattr(tc, "function", None)
                        if fn is not None:
                            if getattr(fn, "name", None):
                                slot["name"] += fn.name
                            if getattr(fn, "arguments", None):
                                slot["arguments"] += fn.arguments
            if finish:
                # Emit fully-assembled tool calls, then the terminal chunk.
                for idx in sorted(pending):
                    slot = pending[idx]
                    name = slot["name"]
                    if not name:
                        continue
                    args = _safe_json(slot["arguments"])
                    yield LLMChunk(
                        content_delta="",
                        tool_call_delta=ToolCall(id=slot["id"] or f"call_{idx}", name=name, arguments=args),
                        finish_reason=None,
                    )
                yield LLMChunk(content_delta="", finish_reason=finish)
                return

    async def aclose(self) -> None:
        # Idempotent — safe to call multiple times from ``finally`` blocks.
        # The underlying client exposes either ``is_closed`` (real
        # AsyncOpenAI) or ``closed`` (fake / test doubles). Without the
        # guard, a second ``aclose()`` after the client has already been
        # closed raises RuntimeError, which masks the real exception in
        # teardown paths.
        already_closed: bool = (
            getattr(self._client, "is_closed", False)
            or getattr(self._client, "closed", False)
        )
        if not already_closed:
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
