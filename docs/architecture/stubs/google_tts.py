"""Google Cloud Text-to-Speech provider — DESIGN-ONLY STUB (t_8d714ff0).

**STATUS: design-only. NOT imported by production code.**

This file exists to demonstrate how the 5 extension points of
:class:`BaseTTSProvider` are filled in for Google Cloud TTS. Real
implementation lands as a separate task once ``BaseTTSProvider`` and
``TTSProviderRegistry`` (ADR-0007) are Accepted.

Reference: https://cloud.google.com/text-to-speech/docs/reference/rest

Extension points filled in:

1. ``capabilities()``  → streaming=True, ssml=True,
                         audio_format_pcm/mp3/ogg=True; voice_cloning=False.
2. ``list_voices()``    → voices.list with OAuth2 token.
3. ``healthcheck()``    → cheap OAuth token introspection.
4. ``_build_request_payload`` → ``{"input": {"ssml": ...},
   "voice": {"name": ..., "languageCode": ...}, "audioConfig": ...}``.
5. ``_http_client_factory``  → httpx.AsyncClient with OAuth Bearer header
   (token refreshed in background via ``google.auth``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import httpx

    from rob_box_llm.tts import TTSSettings
    from rob_box_llm.tts_provider_base import TTSHealth, TTSVoice


class GoogleTTSProvider:
    """Google Cloud TTS adapter — STUB ONLY (no implementation).

    Subclasses :class:`BaseTTSProvider` once the ABC lands. Until then,
    calling any method raises ``NotImplementedError`` (correct contract
    for an unconfigured stub).
    """

    name = "google"
    DEFAULT_BASE_URL = "https://texttospeech.googleapis.com"
    DEFAULT_VOICE = "en-US-Wavenet-A"
    DEFAULT_LANGUAGE = "en-US"

    def __init__(
        self,
        *,
        credentials_path: str | None = None,  # service-account JSON path
        base_url: str = DEFAULT_BASE_URL,
        default_voice: str = DEFAULT_VOICE,
        default_language: str = DEFAULT_LANGUAGE,
        timeout: float = 30.0,
        client: "httpx.AsyncClient | None" = None,
    ) -> None:
        """Construct a Google Cloud TTS adapter. STUB — body lands later."""
        raise NotImplementedError(
            "GoogleTTSProvider is a design-only stub (t_8d714ff0). "
            "Implementation lands as a separate task once BaseTTSProvider "
            "is Accepted (ADR-0007)."
        )

    # ---- capability metadata ----

    def capabilities(self) -> Any:
        """Google supports streaming, SSML, PCM/MP3/OGG; no cloning."""
        raise NotImplementedError("stub")

    async def list_voices(self) -> list["TTSVoice"]:
        """voices.list with OAuth2 → list[TTSVoice]."""
        raise NotImplementedError("stub")

    async def healthcheck(self) -> "TTSHealth":
        """OAuth token introspection → TTSHealth(ok, latency_ms)."""
        raise NotImplementedError("stub")

    # ---- mandatory extension points ----

    def _build_request_payload(
        self,
        text: str,
        settings: "TTSSettings",
        voice_meta: "TTSVoice | None",
    ) -> dict[str, Any]:
        """Pure mapping → Google texttospeech body shape.

        Example (illustrative — exact field names per Google docs)::

            {
                "input": {"ssml": text} if settings.ssml else {"text": text},
                "voice": {
                    "name": voice_meta.id if voice_meta else "en-US-Wavenet-A",
                    "languageCode": voice_meta.language if voice_meta else "en-US",
                },
                "audioConfig": {
                    "audioEncoding": "MP3",
                    "sampleRateHertz": settings.sample_rate or 24000,
                },
            }
        """
        raise NotImplementedError("stub")

    def _http_client_factory(self) -> "httpx.AsyncClient":
        """httpx.AsyncClient with OAuth Bearer header (refreshed in bg)."""
        raise NotImplementedError("stub")

    # ---- inherited TTSProvider contract (synthesize / stream / aclose) ----

    async def synthesize(
        self,
        text: str,
        *,
        settings: "TTSSettings | None" = None,
    ) -> Any:
        """POST /v1/text:synthesize → JSON with ``audioContent`` (base64). STUB."""
        raise NotImplementedError("stub")

    async def stream(
        self,
        text: str,
        *,
        settings: "TTSSettings | None" = None,
    ) -> Any:
        """Not natively streaming — degrades to single-chunk synthesize
        with finish_reason='stop' per ADR-0003 §2.4 contract. STUB."""
        raise NotImplementedError("stub")

    async def aclose(self) -> None:
        """Close owned httpx.AsyncClient (idempotent). STUB."""
        raise NotImplementedError("stub")


__all__ = ["GoogleTTSProvider"]