"""MiniMax provider (MiniMax-M3, OpenAI-compatible).

Adds ``MiniMaxProvider`` to the ``rob_box_llm`` shared layer. The endpoint is
``https://api.minimax.io/v1`` (global), the default model is ``MiniMax-M3``.

MiniMax is NOT the same as Xiaomi MiMo — the latter is already shipped as
``MiMoProvider`` against ``api.xiaomimimo.com``. Use this adapter only when
you want MiniMax.

Design constraints (see ``architecture/minimax-provider.md`` and
``docs/adr/0002-minimax-provider.md``):

* Inherits from ``_OpenAICompatibleProvider`` so we reuse the OpenAI Chat
  Completions wire format and the existing ``openai`` SDK.
* ``streaming_tools=False`` — streaming tool-call delta aggregation is not yet
  implemented in the wire-format adapter; callers must use ``complete()`` for
  tool requests.
* Image input is gated per-model: only MiniMax vision-capable models (today:
  ``MiniMax-M3`` and any other ``*M3*`` / ``*vision*`` names) accept
  ``ImagePart``. Other models raise ``CapabilityUnavailableError`` BEFORE
  hitting the network.
* Image payloads are bounded at 10 MB per frame — an engineering default that
  matches MiniMax's published guidance (the documented value may move; this
  constant lives here so a single edit updates the limit).
* The API key is never logged. ``MiniMaxProvider`` automatically attaches a
  ``MiniMaxRedactedLogFilter`` to its own and httpx's loggers as belt-and-
  braces redaction of accidental ``Authorization`` leaks.
* ``_post_process_response`` translates MiniMax's in-body ``base_resp.status_code``
  envelope into our typed ``ProviderError`` hierarchy so callers can branch
  on quota / safety / auth errors that arrive on HTTP 200.
"""

from __future__ import annotations

import dataclasses
import logging
import os
from typing import Any, Iterable, Mapping, Optional, Union

import httpx
from openai import AsyncOpenAI

from rob_box_llm.errors import (
    AuthError,
    CapabilityUnavailableError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
)
from rob_box_llm.provider import (
    ImagePart,
    LLMMessage,
    LLMSettings,
    ProviderCapabilities,
)
from rob_box_llm.providers.deepseek import _OpenAICompatibleProvider

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------


#: Substrings that mark a model as vision-capable (lowercase comparison).
#: Conservative default: only MiniMax-M3 and MiniMax-M2 vision variants.
_MINIMAX_VISION_MODEL_TOKENS: tuple[str, ...] = (
    "minimax-m3",
    "minimax-m2-vision",
    "minimax-vision",
)

#: Engineering-default upper bound for a single image payload (10 MB).
#: Matches MiniMax's published guidance as of writing; if the official limit
#: moves, change this constant and re-run the unit tests.
MINIMAX_MAX_IMAGE_BYTES: int = 10 * 1024 * 1024

#: Default thinking policy for latency-sensitive paths (voice, perception).
#: Pushed through ``settings.extra`` so callers can override it for agentic /
#: tool-call scenarios where reasoning is desirable.
DEFAULT_THINKING_POLICY: dict[str, str] = {"type": "disabled"}

#: Per-phase HTTP timeout used by the MiniMax LLM client.
#:
#: A bare ``float`` would mean "use this many seconds for every phase
#: (connect/read/write/pool)" — so a DNS or TLS hang on the connect phase
#: would block for the full 30 s, which is the symptom BLK-5 set out to
#: fix. We split the budget:
#:
#: * ``connect=5.0``  — TCP+TLS handshake; a slow connect usually means
#:   the host is unreachable and the user should hear about it fast.
#: * ``read=20.0``    — covers LLM streaming; MiniMax-M3 long-context
#:   replies can take ~10–15 s and we want headroom.
#: * ``write=10.0``   — request body upload; image frames are 10 MB but
#:   the wire is usually fast, so 10 s is comfortable.
#: * ``pool=5.0``     — waiting for a free connection from the SDK's
#:   internal pool; should be near-instant.
#:
#: Pass an :class:`httpx.Timeout` (or a plain float, which is treated as
#: a single "all phases" value) to override per-instance.
DEFAULT_TIMEOUT: httpx.Timeout = httpx.Timeout(
    connect=5.0, read=20.0, write=10.0, pool=5.0
)


def _coerce_timeout(value: Union[float, httpx.Timeout, None]) -> httpx.Timeout:
    """Normalise the ``timeout`` constructor argument to :class:`httpx.Timeout`.

    Accepts:

    * ``None``  — falls back to :data:`DEFAULT_TIMEOUT`.
    * ``float`` — applied to every phase (httpx semantics). Useful for
      quick overrides like ``timeout=60.0`` in tests.
    * :class:`httpx.Timeout` — used as-is.

    The OpenAI SDK accepts ``httpx.Timeout`` natively for its ``timeout=``
    kwarg, so the returned value passes straight through to
    ``AsyncOpenAI(timeout=...)`` without further conversion.
    """
    if value is None:
        return DEFAULT_TIMEOUT
    if isinstance(value, httpx.Timeout):
        return value
    # Numeric (int/float). httpx already normalises a single number to
    # "all phases", so we just construct the Timeout explicitly to keep
    # the type predictable.
    return httpx.Timeout(timeout=float(value))


# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------


class MiniMaxRedactedLogFilter(logging.Filter):
    """Strip the MiniMax API key from any log record that mentions it.

    Attach to a logger / handler::

        logger = logging.getLogger("rob_box_llm.providers.minimax")
        logger.addFilter(MiniMaxRedactedLogFilter(api_key_env="MINIMAX_API_KEY"))

    The filter replaces any occurrence of the resolved API key with ``"***"``.
    Use ``api_key_env`` in production (read once at startup) so the key never
    lands in code or logs. ``api_key`` is also accepted for tests.
    """

    REDACTED = "***"

    def __init__(
        self,
        *,
        api_key: Optional[str] = None,
        api_key_env: Optional[str] = None,
    ) -> None:
        super().__init__()
        resolved: Optional[str] = api_key
        if resolved is None and api_key_env:
            resolved = os.environ.get(api_key_env)
        self._needle: Optional[str] = resolved or None

    def filter(self, record: logging.LogRecord) -> bool:  # noqa: A003
        if not self._needle:
            return True
        msg = record.getMessage()
        if self._needle in msg:
            record.msg = msg.replace(self._needle, self.REDACTED)
            record.args = ()
        return True


# ---------------------------------------------------------------------------
# Helpers (used by the adapter and the unit tests)
# ---------------------------------------------------------------------------


def _validate_image_bytes(messages: Iterable[Any]) -> None:
    """Enforce the per-image size limit on every ``ImagePart`` in ``messages``.

    Raises ``CapabilityUnavailableError`` (with the bytes count) so the caller
    can short-circuit before paying for an upload that will be rejected.
    """
    for msg in messages:
        content = getattr(msg, "content", None)
        if isinstance(content, str) or content is None:
            continue
        for part in content:
            if not isinstance(part, ImagePart):
                continue
            src = part.source
            if isinstance(src, bytes) and len(src) > MINIMAX_MAX_IMAGE_BYTES:
                raise CapabilityUnavailableError(
                    f"minimax: image payload {len(src)} bytes exceeds " f"limit {MINIMAX_MAX_IMAGE_BYTES}",
                    provider="minimax",
                )


def _model_supports_vision(model: str | None) -> bool:
    """Return True if ``model`` advertises image-input support."""
    if not model:
        return False
    lowered = model.lower()
    return any(tok in lowered for tok in _MINIMAX_VISION_MODEL_TOKENS)


def _raise_for_base_resp(resp: Any) -> None:
    """Translate ``base_resp.status_code`` into a typed ``ProviderError``.

    MiniMax returns HTTP 200 with an application-level error envelope on
    quota / billing / safety / auth failures. We map the ``status_msg`` to
    our domain errors; otherwise the caller sees a ``ProviderError`` with
    the raw ``status_code`` for diagnostics.
    """
    base_resp = getattr(resp, "base_resp", None)
    if base_resp is None and isinstance(resp, Mapping):
        base_resp = resp.get("base_resp")
    if not isinstance(base_resp, Mapping):
        return
    status = base_resp.get("status_code")
    if not status or status == 0:
        return
    msg = base_resp.get("status_msg", "minimax: base_resp error")
    text = msg.lower()
    if "auth" in text or "key" in text or "token" in text:
        raise AuthError(f"minimax: {status} {msg}", provider="minimax")
    if "quota" in text or "balance" in text or "billing" in text:
        raise RateLimitError(f"minimax: {status} {msg}", provider="minimax")
    if "safe" in text or "content" in text or "policy" in text:
        raise ContentFilterError(f"minimax: {status} {msg}", provider="minimax")
    raise ProviderError(f"minimax: {status} {msg}", provider="minimax")


# ---------------------------------------------------------------------------
# MiniMaxProvider
# ---------------------------------------------------------------------------


class MiniMaxProvider(_OpenAICompatibleProvider):
    """MiniMax (global) chat-completions adapter.

    Default base URL: ``https://api.minimax.io/v1``.
    Default model:    ``MiniMax-M3`` (vision-capable).

    Pass ``api_key`` directly in tests; in production set the
    ``MINIMAX_API_KEY`` env var and pass it through the composition root /
    factory (M2).
    """

    DEFAULT_BASE_URL = "https://api.minimax.io/v1"
    DEFAULT_MODEL = "MiniMax-M3"

    # ``MiniMax-M3`` supports image input, tools and streaming text.
    # Streaming tool calls are deliberately False — the OpenAI-compatible
    # wire-format adapter doesn't aggregate streaming tool-call deltas.
    _CAPABILITIES = ProviderCapabilities(
        text=True,
        streaming_text=True,
        tools=True,
        # 🔴 FIX (live 06.08): streaming_tools=True — MiniMax OpenAI-совместимый
        # API стримит tool-call deltas; база (_OpenAICompatibleProvider.stream)
        # теперь агрегирует их в ToolCall.
        streaming_tools=True,
        image_input=True,
    )

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_BASE_URL,
        api_key: Optional[str] = None,
        model: str = DEFAULT_MODEL,
        timeout: Union[float, httpx.Timeout, None] = DEFAULT_TIMEOUT,
        client: Optional[AsyncOpenAI] = None,
        # Extra knobs the composition root may want to bind without subclassing:
        # Defaults to ``DEFAULT_THINKING_POLICY`` ("disabled") for latency-
        # sensitive paths (voice, perception). Pass ``thinking=None`` to opt
        # out of the default; pass your own mapping to override.
        thinking: Optional[Mapping[str, str]] = DEFAULT_THINKING_POLICY,
    ) -> None:
        # Install redaction on the originating loggers before any SDK/httpx
        # record can reach application handlers. Logger filters run before
        # propagation, so this also protects handlers attached later.
        for logger in (_log, logging.getLogger("httpx")):
            if not any(
                isinstance(item, MiniMaxRedactedLogFilter)
                for item in logger.filters
            ):
                logger.addFilter(
                    MiniMaxRedactedLogFilter(api_key_env="MINIMAX_API_KEY")
                )

        # Normalise to ``httpx.Timeout`` so the OpenAI SDK gets a per-phase
        # configuration by default (see ``_coerce_timeout`` and the BLK-5
        # review note for the rationale). A bare ``float`` keeps working as
        # "all phases" — older call-sites that pass ``timeout=30.0`` do not
        # need to change.
        resolved_timeout = _coerce_timeout(timeout)
        super().__init__(
            name="minimax",
            base_url=base_url,
            default_model=model,
            api_key=api_key,
            timeout=resolved_timeout,
            client=client,
        )
        # The thinking policy is applied via ``settings.extra`` for each call;
        # we keep it as an instance attribute so tests / callers can override.
        self._thinking: Optional[dict[str, str]] = dict(thinking) if thinking is not None else None

    # -- capability introspection -----------------------------------------

    def capabilities_for(self, model: str | None) -> ProviderCapabilities:
        """Narrow capabilities to a specific model.

        Image input is only available on MiniMax vision-capable models; for
        everything else we return the base capabilities with ``image_input``
        set to ``False``. This lets the registry / fallback decorator pick
        the right adapter for a multimodal request.
        """
        base = self._CAPABILITIES
        if base.image_input and not _model_supports_vision(model or self._default_model):
            return ProviderCapabilities(
                text=base.text,
                streaming_text=base.streaming_text,
                tools=base.tools,
                streaming_tools=base.streaming_tools,
                image_input=False,
            )
        return base

    # -- pre-flight validation --------------------------------------------

    def _require_capability_for_messages(
        self,
        messages: Iterable[LLMMessage],
        settings: LLMSettings | None,
        tools: Iterable[Mapping[str, Any]],
        *,
        stream: bool,
    ) -> None:
        # Validate image bytes size BEFORE the network call.
        _validate_image_bytes(messages)
        # Delegate to the shared OpenAI-compatible capability check.
        super()._require_capability_for_messages(messages, settings, tools, stream=stream)

    # -- response post-processing -----------------------------------------

    def _post_process_response(self, resp: Any) -> Any:
        """Surface MiniMax's in-body error envelope, if present."""
        _raise_for_base_resp(resp)
        return resp

    # -- request shaping --------------------------------------------------

    def _build_kwargs(
        self,
        messages: Iterable[LLMMessage],
        tools: Iterable[Mapping[str, Any]],
        settings: LLMSettings | None,
        *,
        stream: bool,
    ) -> dict[str, Any]:
        """Apply the instance thinking policy before delegating to the base."""
        settings = self._apply_thinking_policy(settings)
        return super()._build_kwargs(messages, tools, settings, stream=stream)

    # -- thinking policy helper ------------------------------------------

    def _apply_thinking_policy(self, settings: LLMSettings | None) -> LLMSettings:
        """Merge the instance thinking policy into ``settings.extra``.

        Callers can override per-call by passing their own ``extra`` mapping;
        their value wins. We deliberately keep thinking as an opt-in surface
        rather than promoting it to a typed ``MiniMaxOptions`` — that's a M3
        concern once we have multiple agents using different policies.
        """
        if settings is None:
            settings = LLMSettings()
        if self._thinking is None:
            return settings
        # LLMSettings is frozen — rebuild it with merged ``extra``.
        merged_extra: dict[str, Any] = dict(settings.extra)
        if "thinking" not in merged_extra:
            merged_extra["thinking"] = dict(self._thinking)
        return dataclasses.replace(settings, extra=merged_extra)


__all__ = [
    "MiniMaxProvider",
    "MINIMAX_MAX_IMAGE_BYTES",
    "DEFAULT_THINKING_POLICY",
    "DEFAULT_TIMEOUT",
    "MiniMaxRedactedLogFilter",
]
