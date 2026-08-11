"""ElevenLabs TTS provider — DESIGN-ONLY STUB (t_8d714ff0).

**STATUS: design-only. NOT imported by production code.**

This file exists to demonstrate how the 5 extension points of
:class:`BaseTTSProvider` are filled in for ElevenLabs. Real implementation
lands as a separate task once ``BaseTTSProvider`` and
``TTSProviderRegistry`` (ADR-0007) are Accepted.

Reference: https://docs.elevenlabs.io/api-reference/text-to-speech

Extension points filled in:

1. ``capabilities()``  → streaming=True, voice_cloning=True,
                         audio_format_mp3=True; rest False.
2. ``list_voices()``    → HTTP GET ``/v1/voices`` → list[TTSVoice].
3. ``healthcheck()``    → HTTP GET ``/v1/user`` (cheap, auth-only).
4. ``_build_request_payload`` → ElevenLabs-specific body
   (text, voice_settings, model_id, output_format).
5. ``_http_client_factory``  → httpx.AsyncClient with ``xi-api-key``
   header and ``https://api.elevenlabs.io`` base URL.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import httpx

    from rob_box_llm.tts import TTSSettings
    from rob_box_llm.tts_provider_base import TTSHealth, TTSVoice


class ElevenLabsTTSProvider:
    """ElevenLabs TTS adapter — STUB ONLY (no implementation).

    Subclasses :class:`BaseTTSProvider` once the ABC lands. Until then,
    the methods below have type-annotated signatures + docstrings, but
    calling them raises ``NotImplementedError`` (which is the correct
    contract for an unconfigured stub).
    """

    name = "elevenlabs"
    DEFAULT_BASE_URL = "https://api.elevenlabs.io"
    DEFAULT_VOICE = "21m00Tcm4TlvDq8ikWAM"  # "Rachel"
    DEFAULT_MODEL = "eleven_monolingual_v1"

    def __init__(
        self,
        *,
        api_key: str | None = None,
        base_url: str = DEFAULT_BASE_URL,
        default_voice: str = DEFAULT_VOICE,
        default_model: str = DEFAULT_MODEL,
        timeout: float = 30.0,
        client: "httpx.AsyncClient | None" = None,
    ) -> None:
        """Construct an ElevenLabs adapter. STUB — body lands later."""
        raise NotImplementedError(
            "ElevenLabsTTSProvider is a design-only stub (t_8d714ff0). "
            "Implementation lands as a separate task once BaseTTSProvider "
            "is Accepted (ADR-0007)."
        )

    # ---- capability metadata ----

    def capabilities(self) -> Any:
        """ElevenLabs supports streaming, voice-cloning, MP3."""
        raise NotImplementedError("stub")

    async def list_voices(self) -> list["TTSVoice"]:
        """HTTP GET /v1/voices → list[TTSVoice]."""
        raise NotImplementedError("stub")

    async def healthcheck(self) -> "TTSHealth":
        """HTTP GET /v1/user → TTSHealth(ok, latency_ms)."""
        raise NotImplementedError("stub")

    # ---- mandatory extension points ----

    def _build_request_payload(
        self,
        text: str,
        settings: "TTSSettings",
        voice_meta: "TTSVoice | None",
    ) -> dict[str, Any]:
        """Pure mapping → ElevenLabs body shape.

        Example (illustrative — exact field names per ElevenLabs docs)::

            {
                "text": text,
                "voice_settings": {"stability": 0.5, "similarity_boost": 0.75},
                "model_id": settings.model or "eleven_monolingual_v1",
                "output_format": "mp3_44100_128",
            }
        """
        raise NotImplementedError("stub")

    def _http_client_factory(self) -> "httpx.AsyncClient":
        """httpx.AsyncClient with xi-api-key header + base URL."""
        raise NotImplementedError("stub")

    # ---- inherited TTSProvider contract (synthesize / stream / aclose) ----

    async def synthesize(
        self,
        text: str,
        *,
        settings: "TTSSettings | None" = None,
    ) -> Any:
        """Non-streaming synthesis. STUB."""
        raise NotImplementedError("stub")

    async def stream(
        self,
        text: str,
        *,
        settings: "TTSSettings | None" = None,
    ) -> Any:
        """Streaming synthesis via ElevenLabs SSE. STUB."""
        raise NotImplementedError("stub")

    async def aclose(self) -> None:
        """Close owned httpx.AsyncClient (idempotent). STUB."""
        raise NotImplementedError("stub")


__all__ = ["ElevenLabsTTSProvider"]