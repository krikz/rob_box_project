"""MiniMax TTS provider (T2A v2 HTTP API).

Implements :class:`rob_box_llm.tts.TTSProvider` against MiniMax's
``POST /v1/t2a_v2`` synchronous HTTP endpoint. The provider does NOT need an
async SDK — it uses :mod:`httpx.AsyncClient` directly so we can mock it with
``httpx.MockTransport`` in unit tests (no network, no sleep, no flakiness).

Endpoint reference (from MiniMax T2A v2 HTTP docs):

* URL:    ``https://api.minimax.io/v1/t2a_v2``
* Auth:   ``Authorization: Bearer <MINIMAX_API_KEY>``  (NOT OpenAI-compatible)
* Group:  ``MINIMAX_GROUP_ID`` query parameter
* Body:   ``{"model": "speech-02-hd", "text": "...", "voice_setting": {...}, "audio_setting": {...}, "stream": false}``
* Return: JSON with ``{"data": {"audio": "<hex-encoded pcm>", ...}, "base_resp": {"status_code": 0, "status_msg": "success"}}``
          status_code != 0  → error.

PCM payload is little-endian 16-bit mono. We decode the hex string to raw
bytes and wrap it in a ``TTSAudio`` with the reported sample rate. When OGG
is requested, MiniMax's documented MP3 fallback is surfaced as ``MP3`` in
the returned value so downstream decoders never dispatch on a false marker.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, AsyncIterator, Mapping, Optional, cast

import httpx

from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.tts import (
    TTSAudio,
    TTSChunk,
    TTSFormat,
    TTSProvider,
    TTSSettings,
)
from rob_box_llm.tts_provider_base import (
    BaseTTSProvider,
    DEFAULT_MAX_CONNECTIONS,
    DEFAULT_MAX_CONTENT_SIZE,
    DEFAULT_MAX_KEEPALIVE_CONNECTIONS,
    TTSCapabilities,
    TTSHealth,
    TTSVoice,
)

_log = logging.getLogger(__name__)


# MiniMax-specific BCP-47 → human-readable language name. MiniMax's
# ``language_boost`` field takes full English names ("Russian", "English", …);
# the common short codes ("ru", "en") are mapped below. Anything we don't
# recognise is passed through as-is so the API can reject it explicitly.
_LANGUAGE_ALIASES: dict[str, str] = {
    "ru": "Russian",
    "en": "English",
    "zh": "Chinese",
    "ja": "Japanese",
    "ko": "Korean",
    "es": "Spanish",
    "fr": "French",
    "de": "German",
    "pt": "Portuguese",
    "it": "Italian",
    "ar": "Arabic",
    "hi": "Hindi",
}


def _map_language(lang: str | None) -> str | None:
    if lang is None:
        return None
    lang = lang.strip()
    if not lang:
        return None
    if lang.lower() in _LANGUAGE_ALIASES:
        return _LANGUAGE_ALIASES[lang.lower()]
    # Already a full name (e.g. "Russian") — pass through.
    return lang


def _redact_sensitive_text(text: str, *, secrets: tuple[str, ...]) -> str:
    """Remove configured credentials from untrusted transport diagnostics."""
    for secret in secrets:
        if secret:
            text = text.replace(secret, "<redacted>")
    return text


class _ContentSizeExceeded(TTSError):
    """Sentinel subclass of :class:`TTSError` for the BLK-6 size guard.

    The streaming ``stream()`` path wraps its body in a generic
    ``except Exception`` that converts recoverable transport errors
    into an in-band ``TTSChunk(finish_reason="error")`` so the caller
    can drain partial audio. The size guard, however, is a hard
    policy ceiling — it must ALWAYS propagate to the caller, even if
    audio has already been streamed. We use this marker so the outer
    ``except`` block can re-raise it unchanged instead of converting
    it to an in-band error chunk.
    """

    pass


def _enforce_content_size(
    content_length: int | str | None,
    actual_len: int | None,
    *,
    limit: int = DEFAULT_MAX_CONTENT_SIZE,
    provider: str,
) -> None:
    """Raise :class:`_ContentSizeExceeded` if a response exceeds ``limit`` bytes.

    Called from the MiniMax TTS provider at every response boundary:

    * :meth:`MiniMaxTTSProvider._post` — after reading the JSON body
      for the non-streaming T2A v2 endpoint.
    * :meth:`MiniMaxTTSProvider.stream` — on the SSE byte stream, once
      the upstream has sent :rfc:`7230` ``Content-Length`` (early bail)
      or after each ``aiter_bytes`` chunk for chunked / no-length
      responses.

    BLK-6 / PR-907: httpx's :class:`httpx.Limits` does NOT expose
    ``max_content_size`` — that knob belongs to other HTTP libraries
    (e.g. aiohttp). We enforce the ceiling here instead. The check is
    intentionally cheap (one integer comparison) so it can be invoked
    on every chunk without measurable overhead.

    Parameters
    ----------
    content_length:
        The ``Content-Length`` header value from the upstream response,
        or ``None`` if the server did not advertise one. Passed as the
        raw header value so callers don't need to coerce it themselves.
    actual_len:
        The number of bytes actually read so far (or the final length
        when the body is buffered). ``None`` means "unknown" — the
        function falls back to checking only the header.
    limit:
        Maximum allowed size in bytes. Defaults to
        :data:`rob_box_llm.tts_provider_base.DEFAULT_MAX_CONTENT_SIZE`
        (50 MiB). Tests override to drive the failure path cheaply.
    provider:
        Provider name threaded into the exception for diagnostics.

    Raises
    ------
    _ContentSizeExceeded
        If either the advertised ``Content-Length`` OR the actual bytes
        read exceed ``limit``. Subclassing :class:`TTSError` keeps the
        exception catchable as the domain error everywhere downstream,
        while the sentinel class lets the streaming path's outer
        ``except`` block re-raise it unchanged instead of converting it
        to an in-band error chunk.
    """
    if content_length is not None:
        try:
            advertised = int(content_length)
        except (TypeError, ValueError):
            advertised = -1
        if advertised > limit:
            raise _ContentSizeExceeded(
                f"minimax response too large: Content-Length={advertised} "
                f"exceeds limit={limit} bytes (provider={provider})",
                provider=provider,
            )
    if actual_len is not None and actual_len > limit:
        raise _ContentSizeExceeded(
            f"minimax response too large: read {actual_len} bytes "
            f"exceeds limit={limit} bytes (provider={provider})",
            provider=provider,
        )


def _map_exception(
    exc: Exception,
    *,
    provider: str,
    secrets: tuple[str, ...] = (),
) -> TTSError:
    """Map httpx / MiniMax-shaped errors onto our domain errors.

    If ``exc`` is already a :class:`TTSError` subclass, return it unchanged —
    we don't want to lose specificity (e.g. ``TTSAuthError`` raised by
    ``_headers()`` should not be downgraded to a generic ``TTSError``).

    HTTP response bodies and transport messages are untrusted. Some upstream
    proxies echo the Authorization header in an error body, so credentials
    known to this provider are redacted before an exception reaches callers or
    their exception loggers.
    """
    if isinstance(exc, TTSError):
        return exc
    if isinstance(exc, httpx.TimeoutException):
        return TTSTimeoutError(
            _redact_sensitive_text(str(exc), secrets=secrets), provider=provider
        )
    if isinstance(exc, httpx.HTTPStatusError):
        status = exc.response.status_code
        body = _redact_sensitive_text(exc.response.text, secrets=secrets)
        if status in (401, 403):
            return TTSAuthError(f"{status}: {body}", provider=provider)
        if status == 429:
            return TTSRateLimitError(f"{status}: {body}", provider=provider)
        if 400 <= status < 500:
            return TTSBadRequestError(f"{status}: {body}", provider=provider)
        return TTSError(f"{status}: {body}", provider=provider)
    if isinstance(exc, httpx.HTTPError):
        return TTSTimeoutError(
            _redact_sensitive_text(str(exc), secrets=secrets), provider=provider
        )
    return TTSError(
        _redact_sensitive_text(str(exc), secrets=secrets), provider=provider
    )


def _build_payload(
    text: str,
    settings: TTSSettings,
    *,
    stream: bool,
    default_voice: str,
    default_model: str,
) -> dict[str, Any]:
    """Translate our :class:`TTSSettings` into a MiniMax T2A v2 body.

    We split the body into two nested objects to match the documented schema:

    * ``voice_setting`` — voice, speed, volume, pitch, emotion
    * ``audio_setting``  — sample_rate, bitrate, format, channel

    Fields the caller didn't set fall back to provider defaults — we DON'T
    drop them from the body, otherwise the API applies its own defaults
    which may surprise callers (e.g. "hd" vs "turbo" model).
    """
    voice = settings.voice or default_voice
    model = settings.model or default_model
    sample_rate = settings.sample_rate or 32000
    fmt = settings.format

    voice_setting: dict[str, Any] = {"voice_id": voice}
    if settings.speed is not None:
        voice_setting["speed"] = float(settings.speed)
    if settings.volume is not None:
        # MiniMax T2A v2 documents vol as a number in [0.0, 10.0]. We
        # reject out-of-range values here so the API doesn't return
        # 400 — fail-fast with a typed error instead.
        vol = float(settings.volume)
        if not (0.0 <= vol <= 10.0):
            from rob_box_llm.errors import TTSBadRequestError  # local import

            raise TTSBadRequestError(
                f"volume={vol} out of range [0.0, 10.0] (MiniMax T2A v2 spec)",
                provider="minimax",
            )
        voice_setting["vol"] = vol
    if settings.pitch is not None:
        voice_setting["pitch"] = int(settings.pitch)
    if settings.emotion is not None:
        voice_setting["emotion"] = settings.emotion
    lang = _map_language(settings.language)
    if lang is not None:
        voice_setting["language"] = lang

    # MiniMax audio_setting accepts: sample_rate, bitrate, format ("mp3"|"pcm"|"wav"),
    # channel (1=mono, 2=stereo). We default to mono PCM at 32 kHz — matches
    # the default documented in MiniMax's T2A v2 reference.
    audio_setting: dict[str, Any] = {
        "sample_rate": sample_rate,
        "bitrate": 128000,
        "format": (
            fmt.value if fmt != TTSFormat.OGG else "mp3"
        ),  # OGG unsupported, fall back
        "channel": 1,
    }

    payload: dict[str, Any] = {
        "model": model,
        "text": text,
        "stream": stream,
        "voice_setting": voice_setting,
        "audio_setting": audio_setting,
    }
    if settings.text_normalization is not None:
        payload["text_normalization"] = bool(settings.text_normalization)

    # Whitelist `extra` to the documented top-level keys so a caller (e.g.
    # dialogue_node pulling user-supplied JSON) can't silently overwrite the
    # nested ``voice_setting`` / ``audio_setting`` objects above and inject
    # arbitrary fields like ``__proto__``-style payloads.
    #
    # Anything not on this allowlist is logged at WARNING and dropped — we
    # choose "drop + log" over "raise" because ``extra`` is meant for
    # forward-compat with MiniMax API additions, and raising would force
    # every caller to audit keys against this list.
    if settings.extra:
        from rob_box_llm.errors import TTSBadRequestError  # local import — see tts.py

        for key, value in settings.extra.items():
            if key in _ALLOWED_EXTRA_KEYS:
                payload[key] = value
            else:
                _log.warning(
                    "minimax_tts: dropping unknown extra key %r "
                    "(not in allowlist %s)",
                    key,
                    sorted(_ALLOWED_EXTRA_KEYS),
                )
        # Validate against reserved keys defensively — if the user somehow
        # passes one of our top-level field names we'd silently overwrite
        # it. Show the error loudly.
        overlap = set(settings.extra.keys()) & _RESERVED_PAYLOAD_KEYS
        if overlap:
            raise TTSBadRequestError(
                f"settings.extra contains reserved top-level payload keys: "
                f"{sorted(overlap)}",
                provider="minimax",
            )
    return payload


# Top-level payload keys we always build ourselves; ``extra`` MUST NOT
# touch these — exceptions raise TTSBadRequestError because allowing them
# would let callers either silently overwrite a typed field (security
# hazard: e.g. re-pointing ``voice_setting`` at an attacker-controlled
# dict) or set a contradictory top-level field like ``model``.
_RESERVED_PAYLOAD_KEYS: frozenset[str] = frozenset(
    {
        "model",
        "text",
        "stream",
        "voice_setting",
        "audio_setting",
        "text_normalization",
    }
)

# Allow-list for ``settings.extra``. Only documented top-level MiniMax T2A v2
# fields we don't model ourselves go here. Overlap with the reserved set
# triggers an exception above; the lists are kept separate so future
# reserved additions don't accidentally widen the allow-list.
_ALLOWED_EXTRA_KEYS: frozenset[str] = frozenset(
    {
        "voice_id",  # top-level shortcut, equivalent to voice_setting.voice_id
        "speed",  # top-level shortcut
        "pitch",  # top-level shortcut
        "vol",  # top-level shortcut (note: we build voice_setting.vol ourselves)
        "emotion",  # top-level shortcut
        "subtitle_timestamp",  # request word-level timing
        "pronunciation_dict",  # custom pronunciation overrides
        "timbre_weights",  # custom voice timbre mix
        "audio_output_format",  # alt name for output container
    }
)


class _RedactGroupIdFilter(logging.Filter):
    """Redact MiniMax's credential-like GroupId from httpx access records.

    httpx defers interpolation of ``record.args`` until handlers format the
    record.  Replacing URL arguments here protects every downstream handler,
    even if another component re-enables the global ``httpx`` logger at INFO.
    """

    _REDACTED = "<redacted>"

    def filter(self, record: logging.LogRecord) -> bool:
        args = record.args
        if isinstance(args, tuple):
            record.args = tuple(self._redact(value) for value in args)
        elif isinstance(args, dict):
            record.args = {key: self._redact(value) for key, value in args.items()}
        return True

    @classmethod
    def _redact(cls, value: Any) -> Any:
        if isinstance(value, httpx.URL) and "GroupId" in value.params:
            return value.copy_set_param("GroupId", cls._REDACTED)
        return value


_HTTPX_GROUP_ID_FILTER = _RedactGroupIdFilter()


# Built-in voice catalogue for ``list_voices()``.
#
# MiniMax T2A v2 does NOT expose a public ``/v1/voices`` endpoint as of
# 2026-07-22 (ADR-0003 §4). Until that endpoint lands we serve a static
# catalogue extracted from MiniMax's documented voice list. When the
# endpoint becomes public this constant is replaced by an HTTP call.
_BUILTIN_VOICES: tuple[TTSVoice, ...] = (
    TTSVoice(
        id="male-qn-qingse",
        name="Qingse (male, Chinese-leaning)",
        language="zh",
        gender="male",
        supports_cloning=True,
    ),
    TTSVoice(
        id="female-shaonv",
        name="Shaonv (female, youthful)",
        language="zh",
        gender="female",
        supports_cloning=True,
    ),
    TTSVoice(
        id="Calm_Woman",
        name="Calm Woman (female, English)",
        language="en",
        gender="female",
        supports_cloning=True,
    ),
    TTSVoice(
        id="English_PassionateWarrior",
        name="English Passionate Warrior",
        language="en",
        gender="male",
        supports_cloning=True,
    ),
    TTSVoice(
        id="Russian_Husky_Man",
        name="Russian Husky Man",
        language="ru",
        gender="male",
        supports_cloning=True,
    ),
    TTSVoice(
        id="Russian_Calm_Woman",
        name="Russian Calm Woman",
        language="ru",
        gender="female",
        supports_cloning=True,
    ),
)


class MiniMaxTTSProvider(BaseTTSProvider):
    """MiniMax TTS via the T2A v2 HTTP endpoint.

    Subclasses :class:`rob_box_llm.tts_provider_base.BaseTTSProvider`,
    which itself IS-A :class:`rob_box_llm.tts.TTSProvider`. The 5
    extension points are filled in as follows:

    1. ``capabilities()``              → streaming=True (SSE), voice_cloning=True
                                         (via ``timbre_weights``), audio_format_pcm=True,
                                         audio_format_mp3=True; ssml=False,
                                         pronunciation_dict=False (available via
                                         ``settings.extra``, not first-class),
                                         audio_format_ogg=False (API doesn't support
                                         — falls back to MP3 in ``synthesize``),
                                         custom_endpoint=False.
    2. ``list_voices()``               → returns a static catalogue from
                                         :data:`_BUILTIN_VOICES`. MiniMax's T2A v2
                                         has no public ``/v1/voices`` endpoint as of
                                         2026-07-22 (ADR-0003 §4).
    3. ``healthcheck()``               → cheap pre-flight: validates auth
                                         credentials are configured (no upstream call).
                                         Heavy health is reserved for explicit
                                         ``ping_minimax.py`` script.
    4. ``_build_request_payload``      → pure mapping (TTSSettings → T2A v2 body);
                                         override-friendly.
    5. ``_http_client_factory``        → ``httpx.AsyncClient(timeout=self._timeout)``;
                                         default is sufficient (no custom headers,
                                         no proxy, no OAuth).

    Parameters
    ----------
    api_key:
        MiniMax API key (``MINIMAX_API_KEY`` env var). Falls back to env at
        construction time if not provided.
    group_id:
        MiniMax account/group id (``MINIMAX_GROUP_ID`` env var). Required by
        the API as a query parameter.
    base_url:
        Override the API root. Useful for the China endpoint and for tests.
    default_voice:
        Voice id used when ``TTSSettings.voice`` is ``None``. MiniMax default:
        ``"male-qn-qingse"`` (per MiniMax's T2A v2 docs).
    default_model:
        Model used when ``TTSSettings.model`` is ``None``. Default:
        ``"speech-02-hd"`` — high-quality speech, supports Russian + emotion.
        Use ``"speech-02-turbo"`` for lower latency.
    timeout:
        httpx timeout in seconds.
    client:
        Inject a pre-built :class:`httpx.AsyncClient` (handy for tests with a
        ``MockTransport``). The provider does NOT close an injected client on
        ``aclose()`` — caller owns it.

    Migration note (ADR-0008):
        Before t_25b8e221 this class inherited :class:`TTSProvider` directly.
        Migrating to :class:`BaseTTSProvider` is a single-line change to the
        class declaration and adds 3 small override methods. The public
        contract (synthesize / stream / aclose) is unchanged, so existing
        ROS callers in ``tts_node`` keep working without modification.
    """

    DEFAULT_BASE_URL = "https://api.minimax.io"
    DEFAULT_VOICE = "male-qn-qingse"
    DEFAULT_MODEL = "speech-02-hd"
    DEFAULT_TIMEOUT = 30.0

    def __init__(
        self,
        *,
        api_key: Optional[str] = None,
        group_id: Optional[str] = None,
        base_url: str = DEFAULT_BASE_URL,
        default_voice: str = DEFAULT_VOICE,
        default_model: str = DEFAULT_MODEL,
        timeout: float = DEFAULT_TIMEOUT,
        client: Optional[httpx.AsyncClient] = None,
    ) -> None:
        import os

        self.name = "minimax"
        self._base_url = base_url.rstrip("/")
        self._api_key = api_key or os.getenv("MINIMAX_API_KEY") or ""
        self._group_id = group_id or os.getenv("MINIMAX_GROUP_ID") or ""
        self._default_voice = default_voice
        self._default_model = default_model
        self._timeout = timeout
        self._owns_client = client is None
        # Route through the extension hook so subclasses can customise
        # transport / TLS / OAuth headers — see BaseTTSProvider for the
        # default ``httpx.AsyncClient(timeout=self._timeout)`` factory.
        self._client = client or self._http_client_factory()
        # httpx's default INFO-level access log echoes the full URL —
        # including the ``GroupId`` query parameter — to the ``httpx``
        # logger every request. Attach a filter to the originating logger so
        # the URL argument is redacted before current or future handlers format
        # it. This remains safe if another library later re-enables the global
        # logger at INFO; relying on logger level alone would be undone by
        # normal application logging configuration.
        _httpx_logger = logging.getLogger("httpx")
        if not any(
            isinstance(item, _RedactGroupIdFilter)
            for item in _httpx_logger.filters
        ):
            _httpx_logger.addFilter(_HTTPX_GROUP_ID_FILTER)

    # ------------------------------------------------------------------
    # Extension surface — overrides of BaseTTSProvider hooks
    # ------------------------------------------------------------------

    def capabilities(self) -> TTSCapabilities:
        """Static capability declaration for MiniMax.

        MiniMax's T2A v2 endpoint advertises:

        * streaming (SSE under ``stream=true``)
        * voice cloning (via ``timbre_weights`` in ``settings.extra``)
        * audio_format_pcm + audio_format_mp3 (documented)
        * audio_format_ogg (NOT documented; we transparently fall back
          to MP3 — see :meth:`synthesize` / :meth:`stream`)

        We do NOT claim ``ssml`` (MiniMax has no SSML parser) or
        ``pronunciation_dict`` (the field exists but is not surfaced in
        the public ``TTSSettings`` shape yet — leave the flag off until
        the value-object adds the field, otherwise tests would silently
        start using a capability the provider doesn't actually expose).
        """
        return TTSCapabilities(
            streaming=True,
            voice_cloning=True,
            ssml=False,
            pronunciation_dict=False,
            audio_format_pcm=True,
            audio_format_mp3=True,
            audio_format_ogg=False,
            custom_endpoint=False,
        )

    async def list_voices(self) -> list[TTSVoice]:
        """Return the built-in voice catalogue.

        The 6 voices documented as MiniMax's pre-built set (see the T2A
        v2 reference) are returned without an upstream call — they're
        stable catalogue entries, not per-account customisations. If
        the product later needs account-specific voice lists, this is
        the override point: add an HTTP call, keep the return type.
        """
        return [
            TTSVoice(
                id="male-qn-qingse",
                name="Qn Qingse",
                language="zh",
                gender="male",
                supports_cloning=False,
            ),
            TTSVoice(
                id="female-shaonv",
                name="Shaonv",
                language="zh",
                gender="female",
                supports_cloning=False,
            ),
            TTSVoice(
                id="Calm_Woman",
                name="Calm Woman",
                language="en",
                gender="female",
                supports_cloning=False,
            ),
            TTSVoice(
                id="English_PassionateWarrior",
                name="English Passionate Warrior",
                language="en",
                gender="male",
                supports_cloning=False,
            ),
            TTSVoice(
                id="Russian_DeepVoice",
                name="Russian Deep Voice",
                language="ru",
                gender="male",
                supports_cloning=False,
            ),
            TTSVoice(
                id="Russian_CalmWoman",
                name="Russian Calm Woman",
                language="ru",
                gender="female",
                supports_cloning=False,
            ),
        ]

    async def healthcheck(self) -> TTSHealth:
        """Cheap pre-flight: verify both credentials are configured.

        Does NOT call upstream — that would defeat the point of a
        pre-flight (and would burn quota). If credentials are missing
        we return ``ok=False`` with a short reason; otherwise the
        snapshot says ``ok=True`` and ``latency_ms=0.0`` because we
        haven't actually done a network round-trip.

        ``time.perf_counter`` is unused here because the check is
        pure-validation; if a future implementation adds an HTTP probe,
        it should wrap the call with ``perf_counter()`` and populate
        ``latency_ms``.
        """
        if not self._api_key:
            return TTSHealth(
                ok=False, provider=self.name, reason="MINIMAX_API_KEY missing"
            )
        if not self._group_id:
            return TTSHealth(
                ok=False, provider=self.name, reason="MINIMAX_GROUP_ID missing"
            )
        return TTSHealth(ok=True, provider=self.name)

    def _http_client_factory(self) -> httpx.AsyncClient:
        """Build the default ``httpx.AsyncClient`` for MiniMax.

        Overrides :meth:`BaseTTSProvider._http_client_factory` only to
        document the intent; the implementation matches the base
        default exactly. Subclasses (e.g. a China-endpoint variant)
        can override to set ``base_url=`` and per-request headers.

        BLK-6 / PR-907: the pool is tightened to
        :data:`DEFAULT_MAX_CONNECTIONS` /
        :data:`DEFAULT_MAX_KEEPALIVE_CONNECTIONS` — a single-tenant
        voice pipeline never bursts past a handful of concurrent
        sockets, and httpx's default of 100/20 invites an OOM via
        runaway response bodies. Response-body size is bounded
        separately by :func:`_enforce_content_size` at the
        :meth:`_post` / :meth:`stream` boundary because httpx's
        :class:`httpx.Limits` does NOT expose a ``max_content_size``
        parameter (verified against httpx 0.27–0.28; not on the
        upstream roadmap).
        """
        return httpx.AsyncClient(
            timeout=self._timeout,
            limits=httpx.Limits(
                max_connections=DEFAULT_MAX_CONNECTIONS,
                max_keepalive_connections=DEFAULT_MAX_KEEPALIVE_CONNECTIONS,
            ),
        )

    def _build_request_payload(
        self,
        text: str,
        settings: TTSSettings,
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        """Pure mapping ``TTSSettings → MiniMax T2A v2 body``.

        Subclasses / tests can override this hook to inspect a
        ``voice_meta`` catalogue entry (e.g. to swap to a cloned voice
        id when ``supports_cloning=True``). The default implementation
        delegates to the module-level :func:`_build_payload` helper,
        which is the same function used by the non-extended public
        ``synthesize`` / ``stream`` paths — keeping a single mapping
        function avoids the "two payload builders drift apart" bug.

        ``voice_meta`` is currently advisory: the public
        :meth:`synthesize` / :meth:`stream`` already resolve the
        ``settings.voice`` themselves before calling the module-level
        helper, so by the time we get here ``voice_meta`` is for
        logging / future cloning logic, not for mutation.
        """
        # Note: callers in synthesize() / stream() need the non-stream
        # / stream flag respectively. They don't pass it through this
        # hook — that's why the public methods still call _build_payload
        # directly. This hook exists for the registry / future-providers
        # contract and for unit-testing the mapping in isolation.
        return _build_payload(
            text,
            settings,
            stream=False,  # overridden by synthesize / stream paths
            default_voice=self._default_voice,
            default_model=self._default_model,
        )

    # ------------------------------------------------------------------
    # HTTP plumbing
    # ------------------------------------------------------------------

    def _headers(self) -> dict[str, str]:
        if not self._api_key:
            raise TTSAuthError(
                "MINIMAX_API_KEY is not configured (set env or pass api_key=)",
                provider=self.name,
            )
        return {
            "Authorization": f"Bearer {self._api_key}",
            "Content-Type": "application/json",
        }

    def _params(self) -> dict[str, str]:
        if not self._group_id:
            raise TTSAuthError(
                "MINIMAX_GROUP_ID is not configured (set env or pass group_id=)",
                provider=self.name,
            )
        return {"GroupId": self._group_id}

    async def _post(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        url = f"{self._base_url}/v1/t2a_v2"
        try:
            resp = await self._client.post(
                url,
                params=self._params(),
                headers=self._headers(),
                content=json.dumps(payload),
            )
        except Exception as exc:  # noqa: BLE001 — map to domain errors
            raise _map_exception(
                exc,
                provider=self.name,
                secrets=(self._api_key, self._group_id),
            ) from exc

        if resp.status_code >= 400:
            # Use raise_for_status to get the HTTPStatusError path.
            try:
                resp.raise_for_status()
            except httpx.HTTPStatusError as exc:
                raise _map_exception(
                    exc,
                    provider=self.name,
                    secrets=(self._api_key, self._group_id),
                ) from exc

        # BLK-6: enforce response-size ceiling BEFORE decoding the body.
        # httpx has no ``max_content_size`` on :class:`httpx.Limits`, so we
        # guard at the application boundary. ``resp.aread()`` is the
        # canonical way to drain the body once for both the size check
        # and the JSON parse below — calling ``resp.json()`` would
        # re-read the buffer for non-streaming bodies.
        body_bytes = await resp.aread()
        _enforce_content_size(
            resp.headers.get("Content-Length"),
            len(body_bytes),
            provider=self.name,
        )

        try:
            data = json.loads(body_bytes)
        except json.JSONDecodeError as exc:
            response_text = _redact_sensitive_text(
                body_bytes[:200].decode("utf-8", errors="replace"),
                secrets=(self._api_key, self._group_id),
            )
            raise TTSError(
                f"Non-JSON response: {response_text}", provider=self.name
            ) from exc

        # MiniMax's error envelope: base_resp.status_code != 0 → API-level error.
        base_resp = data.get("base_resp") or {}
        status_code = base_resp.get("status_code", 0)
        if status_code != 0:
            raw_status_msg = str(base_resp.get("status_msg", "unknown"))
            status_msg = _redact_sensitive_text(
                raw_status_msg,
                secrets=(self._api_key, self._group_id),
            )
            message = f"minimax API error {status_code}: {status_msg}"
            # Heuristic mapping — we don't have a documented taxonomy beyond
            # status_code so we use the original status text to pick a category;
            # redaction may replace short test credentials inside words such as
            # ``key`` and must not alter exception classification.
            msg_lower = raw_status_msg.lower()
            if "auth" in msg_lower or "key" in msg_lower or "token" in msg_lower:
                raise TTSAuthError(message, provider=self.name)
            if "quota" in msg_lower or "rate" in msg_lower or "limit" in msg_lower:
                raise TTSRateLimitError(message, provider=self.name)
            if "invalid" in msg_lower or "param" in msg_lower or "voice" in msg_lower:
                raise TTSBadRequestError(message, provider=self.name)
            raise TTSError(message, provider=self.name)

        # `resp.json()` is typed `Any` by httpx; the documented MiniMax
        # envelope is always a JSON object so narrowing here is safe.
        # `cast` keeps ``mypy --strict --no-any-return`` quiet — same
        # idiom used in ``rob_box_llm/providers/deepseek.py``.
        return cast("dict[str, Any]", data)

    def _decode_audio(self, data: dict[str, Any], fmt: TTSFormat) -> tuple[bytes, int]:
        """Extract raw audio bytes from the MiniMax response.

        MiniMax returns hex-encoded PCM in ``data.audio`` when format=pcm,
        or a hex-encoded MP3/WAV blob when those formats are requested.
        We assume hex because that's what every documented sample shows.
        """
        payload = data.get("data") or {}
        audio_hex = payload.get("audio")
        if not audio_hex:
            raise TTSError("minimax response missing 'data.audio'", provider=self.name)
        try:
            return bytes.fromhex(audio_hex), int(
                payload.get("audio_sample_rate", 32000)
            )
        except ValueError as exc:
            raise TTSError(
                f"minimax returned non-hex audio payload: {exc}", provider=self.name
            ) from exc

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    async def synthesize(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> TTSAudio:
        if not text or not text.strip():
            raise TTSBadRequestError("text is empty", provider=self.name)

        s = settings or TTSSettings()
        # Force non-streaming for synthesize() regardless of the user's format
        # preference — we always get the full payload back and decode it once.
        payload = _build_payload(
            text,
            s,
            stream=False,
            default_voice=self._default_voice,
            default_model=self._default_model,
        )
        data = await self._post(payload)
        samples, sample_rate = self._decode_audio(data, s.format)
        actual_format = TTSFormat.MP3 if s.format == TTSFormat.OGG else s.format
        _log.info(
            "minimax TTS: %d samples @ %d Hz (model=%s, voice=%s, fmt=%s)",
            len(samples) // 2,
            sample_rate,
            payload["model"],
            payload["voice_setting"]["voice_id"],
            actual_format.value,
        )
        return TTSAudio(
            samples=samples, sample_rate=sample_rate, format=actual_format, raw=data
        )

    async def stream(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> AsyncIterator[TTSChunk]:
        """Stream the MiniMax TTS response as :class:`TTSChunk` frames.

        .. note::

           The provider emits each SSE audio event as a ``TTSChunk`` as soon
           as it arrives, followed by an empty terminal chunk with
           ``finish_reason="stop"``. This is HTTP/SSE event streaming; true
           fixed-size PCM frame streaming would require switching to
           MiniMax's T2A WebSocket endpoint (out of scope here; tracked in the
           parent ADR "Future work").

        Mid-stream errors (events delivered after the first audio chunk
        would have been yielded) are converted to a terminal
        ``TTSChunk(finish_reason="error")`` per the contract; pre-yield
        errors raise :class:`TTSError` directly.
        """
        if not text or not text.strip():
            raise TTSBadRequestError("text is empty", provider=self.name)

        s = settings or TTSSettings()
        # MiniMax supports SSE streaming under stream=true — same payload
        # shape, but the response is a series of JSON objects rather than a
        # single one. Emit each audio event as soon as it arrives so callers
        # can begin playback before the complete utterance is buffered.
        payload = _build_payload(
            text,
            s,
            stream=True,
            default_voice=self._default_voice,
            default_model=self._default_model,
        )
        yielded_audio = False
        collected_sr = 0
        # BLK-6: cumulative byte counter for the SSE stream. ``aiter_bytes``
        # yields raw network bytes (which include the SSE framing + JSON
        # envelope, not just the decoded audio), so this naturally fires
        # BEFORE the audio payload itself blows the budget — we get a
        # cheap, single-comparison OOM guard per chunk.
        bytes_seen = 0
        # BLK-6: set when the upstream signals end-of-stream via
        # ``data: [DONE]``. The outer ``aiter_bytes`` loop must keep
        # draining the socket so we don't strand half-consumed bytes
        # in the connection pool; once the inner ``for line`` loop is
        # done, we break out of the outer loop and fall through to the
        # terminal ``stop`` chunk yield below.
        saw_done = False
        try:
            url = f"{self._base_url}/v1/t2a_v2"
            async with self._client.stream(
                "POST",
                url,
                params=self._params(),
                headers=self._headers(),
                content=json.dumps(payload),
            ) as resp:
                # BLK-6: early bail on an oversized Content-Length. If the
                # upstream advertised a body larger than our policy ceiling,
                # we fail BEFORE buffering any bytes — the same guard as
                # ``_post`` runs at the streaming boundary too.
                _enforce_content_size(
                    resp.headers.get("Content-Length"),
                    None,
                    provider=self.name,
                )
                if resp.status_code >= 400:
                    await resp.aread()
                    try:
                        resp.raise_for_status()
                    except httpx.HTTPStatusError as exc:
                        raise _map_exception(
                            exc,
                            provider=self.name,
                            secrets=(self._api_key, self._group_id),
                        ) from exc
                async for chunk in resp.aiter_bytes():
                    bytes_seen += len(chunk)
                    if bytes_seen > DEFAULT_MAX_CONTENT_SIZE:
                        # BLK-6: this is a HARD POLICY ceiling. The outer
                        # ``except Exception`` (below) converts recoverable
                        # transport errors into an in-band ``error`` chunk,
                        # but partial audio + a silent swallow is the wrong
                        # outcome here — the caller asked for a bounded
                        # response and we must ALWAYS propagate. ``_ContentSizeExceeded``
                        # is a :class:`TTSError` subclass so the outer
                        # handler recognises it and re-raises unchanged.
                        raise _ContentSizeExceeded(
                            f"minimax stream too large: read {bytes_seen} bytes "
                            f"exceeds limit={DEFAULT_MAX_CONTENT_SIZE} bytes "
                            f"(provider={self.name})",
                            provider=self.name,
                        )
                    try:
                        text = chunk.decode("utf-8", errors="replace")
                    except Exception as exc:  # noqa: BLE001
                        raise TTSError(
                            f"minimax stream decode error: {exc}",
                            provider=self.name,
                        ) from exc
                    # SSE events are separated by blank lines; ``aiter_bytes``
                    # hands us raw network bytes, so we split on those. This
                    # is equivalent to aiter_lines() but interleaved with the
                    # byte-count guard above.
                    for line in text.splitlines():
                        if not line:
                            continue
                        # MiniMax SSE payload: "data:{json}\n\n"
                        if line.startswith("data:"):
                            line = line[5:].strip()
                        if line == "[DONE]":
                            saw_done = True
                            continue
                        try:
                            evt = json.loads(line)
                        except json.JSONDecodeError:
                            diagnostic = _redact_sensitive_text(
                                line[:80], secrets=(self._api_key, self._group_id)
                            )
                            _log.debug("ignoring non-JSON SSE line: %r", diagnostic)
                            continue
                        base_resp = evt.get("base_resp") or {}
                        if base_resp.get("status_code", 0) != 0:
                            raw_status_msg = str(base_resp.get("status_msg", "unknown"))
                            status_msg = _redact_sensitive_text(
                                raw_status_msg,
                                secrets=(self._api_key, self._group_id),
                            )
                            message = f"minimax stream error {base_resp.get('status_code')}: {status_msg}"
                            msg_lower = raw_status_msg.lower()
                            if "auth" in msg_lower or "key" in msg_lower or "token" in msg_lower:
                                api_error: TTSError = TTSAuthError(message, provider=self.name)
                            elif "quota" in msg_lower or "rate" in msg_lower or "limit" in msg_lower:
                                api_error = TTSRateLimitError(message, provider=self.name)
                            elif "invalid" in msg_lower or "param" in msg_lower or "voice" in msg_lower:
                                api_error = TTSBadRequestError(message, provider=self.name)
                            else:
                                api_error = TTSError(message, provider=self.name)
                            if yielded_audio:
                                yield TTSChunk(finish_reason="error")
                                return
                            raise api_error
                        payload_data = evt.get("data") or {}
                        chunk_hex = payload_data.get("audio")
                        if chunk_hex:
                            try:
                                chunk_samples = bytes.fromhex(chunk_hex)
                            except ValueError as exc:
                                raise TTSError(
                                    f"minimax returned non-hex audio payload: {exc}",
                                    provider=self.name,
                                ) from exc
                            collected_sr = int(
                                payload_data.get("audio_sample_rate", collected_sr or 32000)
                            )
                            yielded_audio = True
                            actual_format = TTSFormat.MP3 if s.format == TTSFormat.OGG else s.format
                            yield TTSChunk(
                                samples=chunk_samples,
                                sample_rate=collected_sr,
                                format=actual_format,
                            )
                    if saw_done:
                        # Drain the rest of the socket (if any) so the
                        # connection returns cleanly to the keep-alive pool,
                        # then exit the outer byte loop.
                        break
        except Exception as exc:  # noqa: BLE001
            # Transport/decoder failures after the first audio frame are
            # represented in-band; before that point they must raise.
            # The size guard is the exception — it must ALWAYS propagate
            # because partial audio + a silent swallow is the wrong
            # outcome (the caller asked for a bounded response).
            if isinstance(exc, _ContentSizeExceeded):
                raise
            if isinstance(exc, TTSError):
                if yielded_audio:
                    yield TTSChunk(finish_reason="error")
                    return
                raise
            mapped_error = _map_exception(
                exc,
                provider=self.name,
                secrets=(self._api_key, self._group_id),
            )
            if yielded_audio:
                yield TTSChunk(finish_reason="error")
                return
            raise mapped_error from exc

        if not yielded_audio:
            # No audio delivered → pre-yield "no data" failure: raise.
            raise TTSError(
                "minimax stream returned no audio chunks", provider=self.name
            )

        # Empty terminal chunk makes end-of-stream unambiguous without adding
        # latency to the audio frames above.
        yield TTSChunk(
            samples=b"",
            sample_rate=collected_sr,
            format=TTSFormat.MP3 if s.format == TTSFormat.OGG else s.format,
            finish_reason="stop",
        )

    async def aclose(self) -> None:
        # Idempotent — safe to call multiple times from ``finally`` blocks.
        # We only ever close a client we ourselves created (caller owns
        # injected clients and is responsible for closing them). The
        # ``is_closed`` guard handles the case where someone closed a
        # previously-injected client out from under us.
        if self._owns_client and not self._client.is_closed:
            await self._client.aclose()


__all__ = ["MiniMaxTTSProvider"]
