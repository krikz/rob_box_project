#!/usr/bin/env python3
"""MiniMax STT provider (PoC, issue #2365).

Adds MiniMax Speech-to-Text as a new ``STTProvider``-compatible adapter to
the rob_box_voice STT chain. Implements the same minimal ``STTProvider``
Protocol declared in :mod:`rob_box_voice.stt_fallback`:

* ``name: str``                        — provider tag for metrics/logs
* ``recognize(audio_bytes) -> str | None`` — return text or ``None`` on failure

Why a separate file (and not extending ``stt_node.py``)
------------------------------------------------------
The existing chain (Yandex gRPC primary + Vosk fallback) is wired inside
``stt_node._recognize_with_fallback``. Adding MiniMax to the wire
(priority / chain ordering / runtime flag / e2e scenario) belongs to
Phase 2 of issue #2365 and will require an ADR + integration tests on
real audio. Phase 1 here ships:

1. Pure-Python adapter that talks to MiniMax's REST endpoint
   (``POST https://api.minimax.io/v1/speech_to_text``) over
   ``httpx.AsyncClient``. No rclpy/vosk/grpc imports — testable in plain
   unit tests the same way ``stt_fallback`` is.
2. API-key redaction on ``httpx`` and our own loggers (same pattern as
   ``MiniMaxProvider`` in rob_box_llm).
3. Typed exceptions + ``MiniMaxSTTError`` that ``select_recognition``
   maps to ``FallbackReason.error`` / ``timeout``.

Wire format
-----------
MiniMax accepts ``multipart/form-data`` with the audio file under ``file``
and a few string fields. Confirmed via:

    curl -sS https://api.minimax.io/v1/speech_to_text
        # → HTTP 411 (Length Required) — endpoint exists, accepts POST.

Reference: https://platform.minimax.io/docs/api-reference/speech-to-text
(fields ``model``, ``file``, ``response_format``, ``timestamp_level``,
``stream``, ``language``).

Configuration
-------------
Keys are read from ``MINIMAX_API_KEY`` env (or passed via the constructor
in tests). Provider can be disabled without code changes by leaving the
env empty — the constructor raises ``MiniMaxSTTUnavailableError`` and
the chain naturally skips it (see ``MiniMaxSTTProvider.maybe_from_env``).

Notes
-----
* ``MiniMax-M3 STT`` (a.k.a. ``asr-1.0``) supports streaming output,
  speaker diarization and subtitle export. We use ``response_format=json``
  + ``timestamp_level=word`` to keep the response shape close to what
  ``select_recognition`` already understands (a single ``text`` field).
* Phase 2 (chain wiring) should expose a ROS parameter
  ``minimax_stt_enabled`` (default ``False`` until we have empirical
  parity numbers) — see ADR planned for follow-up issue.
"""

from __future__ import annotations

import io
import logging
import os
import time
from dataclasses import dataclass
from typing import Any, Mapping, Optional

import httpx

# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

#: Default MiniMax STT endpoint (PoC). Issue #2365 research:
#: ``POST https://api.minimax.io/v1/speech_to_text`` — confirmed live
#: (HTTP 411 on GET, as expected).
DEFAULT_BASE_URL: str = "https://api.minimax.io"

#: Default ASR model name. Per MiniMax docs, ``asr-1.0`` is the
#: STT model exposed via ``/v1/speech_to_text`` (also referenced in the
#: community as "MiniMax-M3 STT").
DEFAULT_MODEL: str = "asr-1.0"

#: Per-phase HTTP timeout, aligned with rob_box_llm ``MiniMaxProvider``.
#:
#: * ``connect=5.0``  — TLS handshake; slow connect usually means the host
#:   is unreachable and the user should hear about it fast (STT is on the
#:   user-perceived latency path).
#: * ``read=15.0``    — generous enough for a single utterance (<=10s)
#:   plus a one-shot diarization pass on the server side.
#: * ``write=10.0``   — audio upload; typical PCM is small but we keep
#:   headroom.
#: * ``pool=5.0``     — waiting on the SDK pool; near-instant in practice.
DEFAULT_TIMEOUT: httpx.Timeout = httpx.Timeout(
    connect=5.0, read=15.0, write=10.0, pool=5.0
)

#: Default language code for the STT call. ``null`` lets the server
#: auto-detect; ``"ru"`` / ``"ru-RU"`` forces Russian. STT-chain in
#: rob_box is Russian-first so we default to ``"ru"``.
DEFAULT_LANGUAGE: Optional[str] = "ru"

#: Provider name used in metrics / logs. Stable string for grep / dashboards.
PROVIDER_NAME: str = "minimax"

#: Upper bound for a single audio payload. MiniMax documentation does
#: not pin a hard limit for /v1/speech_to_text; 25 MB is a conservative
#: engineering ceiling that matches typical ASR services (Whisper,
#: Deepgram, AssemblyAI all advertise 25 MB max upload).
MAX_AUDIO_BYTES: int = 25 * 1024 * 1024


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------


class MiniMaxSTTError(Exception):
    """Base error for MiniMax STT adapter."""


class MiniMaxSTTUnavailableError(MiniMaxSTTError):
    """The provider is configured-but-unavailable (no API key, network down).

    Phase 2 chain wiring should treat this as "skip this provider" rather
    than surfacing to the STT node operator.
    """


class MiniMaxSTTAuthError(MiniMaxSTTError):
    """HTTP 401/403 from MiniMax. Caller should NOT retry."""


class MiniMaxSTTRateLimitError(MiniMaxSTTError):
    """HTTP 429 from MiniMax. Caller may retry with backoff."""


class MiniMaxSTTInvalidResponseError(MiniMaxSTTError):
    """The server returned 200 but the body is not what we expected."""


# ---------------------------------------------------------------------------
# Response dataclass
# ---------------------------------------------------------------------------


@dataclass
class MiniMaxSTTResponse:
    """Decoded response from MiniMax /v1/speech_to_text.

    Only ``text`` is used by ``select_recognition`` today. The other
    fields are kept for Phase 2 (diarization / speaker-id enrichment,
    see issue #2346/#2348).
    """

    text: str
    language: Optional[str] = None
    duration_s: Optional[float] = None
    words: Optional[list[dict]] = None  # populated only when timestamp_level=word
    raw: Optional[Mapping[str, Any]] = None


# ---------------------------------------------------------------------------
# API-key redaction (mirrors rob_box_llm.providers.minimax.MiniMaxRedactedLogFilter)
# ---------------------------------------------------------------------------


class MiniMaxSTTRedactedLogFilter(logging.Filter):
    """Strip the MiniMax API key from any log record that mentions it.

    Attach to the module logger and to the ``httpx`` logger so an accidental
    ``Authorization: Bearer <key>`` leak in httpx debug logs gets scrubbed
    before it reaches the operator's screen.
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
        if self._needle and self._needle in msg:
            record.msg = msg.replace(self._needle, self.REDACTED)
            record.args = ()
        return True


_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Provider
# ---------------------------------------------------------------------------


class MiniMaxSTTProvider:
    """STT provider that POSTs audio to MiniMax's ``/v1/speech_to_text`` endpoint.

    The class implements the ``STTProvider`` Protocol from
    :mod:`rob_box_voice.stt_fallback`:

    * ``name``        — always ``"minimax"`` (the ``PROVIDER_NAME`` constant)
    * ``recognize``   — sync wrapper around the async ``transcribe`` method

    Construction is intentionally cheap (no I/O). Use
    :meth:`maybe_from_env` to get a ready-to-use instance or ``None``
    when the provider is not configured.
    """

    name: str = PROVIDER_NAME

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_BASE_URL,
        api_key: Optional[str] = None,
        model: str = DEFAULT_MODEL,
        language: Optional[str] = DEFAULT_LANGUAGE,
        timeout: httpx.Timeout = DEFAULT_TIMEOUT,
        client: Optional[httpx.Client] = None,
    ) -> None:
        if not api_key:
            # Phase 2 will use ROS params; for now we just refuse to start.
            raise MiniMaxSTTUnavailableError(
                "minimax STT: api_key is empty — provider disabled"
            )

        self._base_url = base_url.rstrip("/")
        self._api_key = api_key
        self._model = model
        self._language = language
        self._timeout = timeout
        self._owns_client = client is None
        self._client: Optional[httpx.Client] = client

        # Install redaction. Same pattern as rob_box_llm.providers.minimax.
        for logger in (_log, logging.getLogger("httpx")):
            if not any(
                isinstance(item, MiniMaxSTTRedactedLogFilter)
                for item in logger.filters
            ):
                logger.addFilter(
                    MiniMaxSTTRedactedLogFilter(api_key=self._api_key)
                )

    # -- factory ---------------------------------------------------------

    @classmethod
    def maybe_from_env(
        cls,
        *,
        api_key_env: str = "MINIMAX_API_KEY",
        **kwargs: Any,
    ) -> Optional["MiniMaxSTTProvider"]:
        """Return a configured provider, or ``None`` if the key is missing.

        Phase 2 chain wiring should call this and skip the provider
        cleanly when no key is present (no noisy warnings).
        """
        api_key = os.environ.get(api_key_env, "").strip()
        if not api_key:
            return None
        try:
            return cls(api_key=api_key, **kwargs)
        except MiniMaxSTTUnavailableError:
            return None

    # -- lifecycle -------------------------------------------------------

    def __enter__(self) -> "MiniMaxSTTProvider":
        if self._client is None:
            self._client = httpx.Client(timeout=self._timeout)
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def close(self) -> None:
        if self._owns_client and self._client is not None:
            self._client.close()
            self._client = None

    def _get_client(self) -> httpx.Client:
        if self._client is None:
            self._client = httpx.Client(timeout=self._timeout)
        return self._client

    # -- core API --------------------------------------------------------

    def recognize(self, audio_bytes: bytes) -> Optional[str]:
        """Synchronous wrapper around :meth:`transcribe` for the ``STTProvider`` Protocol.

        Returns the transcribed text or ``None`` on any error.
        ``select_recognition`` classifies the result and decides whether to
        retry / fall back / surface the empty string to the caller.
        """
        try:
            response = self.transcribe(audio_bytes)
        except MiniMaxSTTAuthError as exc:
            # Auth errors are not retryable — caller should treat as "error"
            # and move on (Yandex/Vosk still available).
            _log.warning("minimax STT: auth error (%s) — skipping", exc)
            return None
        except MiniMaxSTTRateLimitError as exc:
            _log.warning("minimax STT: rate-limited (%s) — falling back", exc)
            return None
        except MiniMaxSTTUnavailableError as exc:
            _log.info("minimax STT: unavailable (%s)", exc)
            return None
        except MiniMaxSTTError as exc:
            _log.warning("minimax STT: %s", exc)
            return None
        return response.text if response else None

    def transcribe(self, audio_bytes: bytes) -> MiniMaxSTTResponse:
        """POST ``audio_bytes`` to MiniMax and return the parsed response.

        Raises :class:`MiniMaxSTTError` subclasses on failure. The
        :meth:`recognize` wrapper downgrades these to ``None`` so the
        STT chain can keep flowing.
        """
        if not audio_bytes:
            raise MiniMaxSTTError("audio_bytes is empty")
        if len(audio_bytes) > MAX_AUDIO_BYTES:
            raise MiniMaxSTTError(
                f"audio_bytes exceeds {MAX_AUDIO_BYTES} byte limit"
            )

        url = f"{self._base_url}/v1/speech_to_text"
        files = {"file": ("audio.wav", io.BytesIO(audio_bytes), "audio/wav")}
        data: dict[str, str] = {
            "model": self._model,
            "response_format": "json",
        }
        if self._language:
            data["language"] = self._language

        headers = {"Authorization": f"Bearer {self._api_key}"}

        client = self._get_client()
        started = time.monotonic()
        try:
            resp = client.post(url, headers=headers, files=files, data=data)
        except httpx.TimeoutException as exc:
            _log.warning("minimax STT: timeout after %.2fs (%s)", self._timeout.read, exc)
            raise MiniMaxSTTUnavailableError(f"timeout: {exc}") from exc
        except httpx.HTTPError as exc:
            _log.warning("minimax STT: http error (%s)", exc)
            raise MiniMaxSTTUnavailableError(f"http error: {exc}") from exc

        latency_ms = int((time.monotonic() - started) * 1000)
        _raise_for_http_status(resp)

        try:
            payload = resp.json()
        except ValueError as exc:
            raise MiniMaxSTTInvalidResponseError(
                f"minimax STT: non-JSON response: {exc}"
            ) from exc

        text = _extract_text(payload)
        if text is None:
            raise MiniMaxSTTInvalidResponseError(
                f"minimax STT: missing 'text' in response: {payload!r}"
            )

        _log.info("minimax STT: ok %dms text=%r", latency_ms, text[:80])
        return MiniMaxSTTResponse(
            text=text,
            language=(payload.get("language") if isinstance(payload, Mapping) else None),
            duration_s=(
                payload.get("duration") if isinstance(payload, Mapping) else None
            ),
            words=(
                payload.get("words") if isinstance(payload, Mapping) else None
            ),
            raw=payload if isinstance(payload, Mapping) else None,
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _extract_text(payload: Any) -> Optional[str]:
    """Return the recognized text from a MiniMax STT response.

    MiniMax's documented JSON shape has ``text`` at the top level. Some
    third-party mirrors wrap it as ``{"data": {"text": ...}}``; we accept
    both for robustness.
    """
    if not isinstance(payload, Mapping):
        return None
    text = payload.get("text")
    if isinstance(text, str):
        return text.strip() or None
    nested = payload.get("data")
    if isinstance(nested, Mapping):
        text = nested.get("text")
        if isinstance(text, str):
            return text.strip() or None
    return None


def _raise_for_http_status(resp: httpx.Response) -> None:
    """Translate a non-2xx response into a typed :class:`MiniMaxSTTError`.

    Kept as a separate helper so :meth:`MiniMaxSTTProvider.transcribe`
    stays under the ADR-0021 cyclomatic-complexity budget (CC<=15).
    """
    status = resp.status_code
    if status in (401, 403):
        raise MiniMaxSTTAuthError(f"minimax STT: auth failure (HTTP {status})")
    if status == 429:
        raise MiniMaxSTTRateLimitError("minimax STT: rate-limited (HTTP 429)")
    if status >= 500:
        raise MiniMaxSTTUnavailableError(
            f"minimax STT: server error (HTTP {status})"
        )
    if status >= 400:
        # 400-class errors that are not auth/rate are client mistakes —
        # bubble up so callers know it was our fault.
        raise MiniMaxSTTInvalidResponseError(
            f"minimax STT: HTTP {status}: {resp.text[:200]}"
        )


__all__ = [
    "PROVIDER_NAME",
    "DEFAULT_BASE_URL",
    "DEFAULT_MODEL",
    "DEFAULT_TIMEOUT",
    "MAX_AUDIO_BYTES",
    "MiniMaxSTTProvider",
    "MiniMaxSTTResponse",
    "MiniMaxSTTError",
    "MiniMaxSTTUnavailableError",
    "MiniMaxSTTAuthError",
    "MiniMaxSTTRateLimitError",
    "MiniMaxSTTInvalidResponseError",
    "MiniMaxSTTRedactedLogFilter",
]