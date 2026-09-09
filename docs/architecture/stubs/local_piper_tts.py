"""Local Piper TTS provider — DESIGN-ONLY STUB (t_8d714ff0).

**STATUS: design-only. NOT imported by production code.**

This file exists to demonstrate how the 5 extension points of
:class:`BaseTTSProvider` are filled in for a fully-local, offline
Piper (https://github.com/rhasspy/piper) adapter. Real implementation
lands as a separate task.

Use case: privacy-sensitive scenarios, no internet, ROS deployment
without external API quotas. ``custom_endpoint=True`` marks it as
non-cloud; capability flags are all False (no streaming, no cloning,
no SSML) — but availability is 100% local.

Extension points filled in:

1. ``capabilities()``  → custom_endpoint=True; all flags False
                         (Piper is local, single-voice per ONNX,
                          no streaming, no cloning, no SSML).
2. ``list_voices()``    → list ``.onnx`` files in
                         ``/usr/share/piper/voices/`` → list[TTSVoice].
3. ``healthcheck()``    → ``subprocess.run(["piper", "--version"])``.
4. ``_build_request_payload`` → CLI arg-list shape (NOT JSON),
   since Piper is a subprocess not an HTTP endpoint.
5. ``_http_client_factory``  → NOT applicable; default returns
   httpx.AsyncClient (unused) so the type contract holds; runtime
   uses subprocess instead. A future ADR may add a
   ``_transport_factory()`` hook for non-HTTP providers.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rob_box_llm.tts import TTSSettings
    from rob_box_llm.tts_provider_base import TTSHealth, TTSVoice


class LocalPiperTTSProvider:
    """Local Piper TTS adapter — STUB ONLY (no implementation).

    Subclasses :class:`BaseTTSProvider` once the ABC lands. Until then,
    calling any method raises ``NotImplementedError`` (correct contract
    for an unconfigured stub).
    """

    name = "local-piper"
    DEFAULT_VOICES_DIR = "/usr/share/piper/voices"
    DEFAULT_VOICE = "ru_RU-irina-medium"

    def __init__(
        self,
        *,
        piper_binary: str = "piper",
        voices_dir: str = DEFAULT_VOICES_DIR,
        default_voice: str = DEFAULT_VOICE,
        timeout: float = 30.0,
    ) -> None:
        """Construct a local Piper adapter. STUB — body lands later."""
        raise NotImplementedError(
            "LocalPiperTTSProvider is a design-only stub (t_8d714ff0). "
            "Implementation lands as a separate task once BaseTTSProvider "
            "is Accepted (ADR-0007)."
        )

    # ---- capability metadata ----

    def capabilities(self) -> Any:
        """Piper is single-shot per ONNX model; no streaming / cloning / SSML."""
        raise NotImplementedError("stub")

    async def list_voices(self) -> list["TTSVoice"]:
        """Glob voices_dir/*.onnx → list[TTSVoice]."""
        raise NotImplementedError("stub")

    async def healthcheck(self) -> "TTSHealth":
        """subprocess.run([piper_binary, '--version']) → TTSHealth."""
        raise NotImplementedError("stub")

    # ---- mandatory extension points ----

    def _build_request_payload(
        self,
        text: str,
        settings: "TTSSettings",
        voice_meta: "TTSVoice | None",
    ) -> dict[str, Any]:
        """Pure mapping → Piper CLI shape (NOT a real HTTP payload).

        Example (illustrative)::

            {
                "text_file": "<tmpfile>",
                "voice_model": f"{voice_meta.id}.onnx",
                "output_file": "<tmpfile>",
            }

        The subprocess wrapper reads this dict and shells out.
        """
        raise NotImplementedError("stub")

    # NOTE: `_http_client_factory` is inherited unchanged from BaseTTSProvider
    # for this provider (it does not use HTTP). A future ADR may introduce
    # `_transport_factory()` for non-HTTP providers; YAGNI until then.

    # ---- inherited TTSProvider contract (synthesize / stream / aclose) ----

    async def synthesize(
        self,
        text: str,
        *,
        settings: "TTSSettings | None" = None,
    ) -> Any:
        """Write text to tmpfile, shell out to ``piper`` with
        ``--model {voice}.onnx --output_file out.wav``, read WAV, wrap
        in TTSAudio(sample_rate=22050, format=TTSFormat.WAV). STUB."""
        raise NotImplementedError("stub")

    async def stream(
        self,
        text: str,
        *,
        settings: "TTSSettings | None" = None,
    ) -> Any:
        """Piper is single-shot — stream() degrades to single-chunk
        synthesize with finish_reason='stop' per ADR-0003 §2.4 contract.
        Explicitly NOT a streaming provider. STUB."""
        raise NotImplementedError("stub")

    async def aclose(self) -> None:
        """No HTTP client to close; default no-op idempotent. STUB."""
        raise NotImplementedError("stub")


__all__ = ["LocalPiperTTSProvider"]