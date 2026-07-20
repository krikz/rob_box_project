"""TTS provider contract — `TTSProvider` ABC + value objects.

Mirrors the shape of :mod:`rob_box_llm.provider` (LLM) so the rest of the
codebase can reason about side-effect providers in the same way: a small,
async-only contract with frozen dataclasses for inputs and outputs.

Contract::

    synthesize(text, *, voice=..., settings=...) -> TTSAudio
    stream(text,    *, voice=..., settings=...) -> AsyncIterator[TTSChunk]

Value objects:
    * `TTSAudio` — full PCM/int16 buffer + sample rate, ready for a sink.
    * `TTSChunk` — a single streaming frame; last chunk carries `finish_reason`.
    * `TTSSettings` — voice, model, language, speed, volume, pitch, format, …
    * `TTSFormat` — enum for the audio container the provider returns
      (the `tts_node` then transcodes to whatever the ROS playback sink wants).

Providers MUST raise `TTSError` BEFORE yielding anything if the initial
request fails — once the first chunk is yielded, mid-stream failures become
a final chunk with ``finish_reason="error"`` (mirrors the LLM contract).
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, AsyncIterator, Mapping, Optional


class TTSFormat(str, Enum):
    """Audio container requested from the provider.

    The provider may ignore this hint if it only supports a single format
    (e.g. raw PCM); in that case it MUST still set ``TTSAudio.format`` to the
    format it actually returned.
    """

    PCM = "pcm"  # raw 16-bit little-endian samples, no header
    WAV = "wav"  # PCM wrapped in a RIFF/WAVE container
    MP3 = "mp3"  # MPEG-1 Layer III
    OGG = "ogg"  # Ogg container (typically Opus or Vorbis)


# ---------------------------------------------------------------------------
# Value objects
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSSettings:
    """Per-call knobs. All optional; the provider chooses sensible defaults.

    Field semantics follow MiniMax's T2A v2 schema where applicable so we
    can map straight to the HTTP request body. For providers that don't
    support a given field the value is silently dropped.
    """

    model: Optional[str] = None  # e.g. "speech-02-hd", "speech-02-turbo"
    voice: Optional[str] = None  # provider-specific voice id (e.g. "Calm_Woman")
    language: Optional[str] = (
        None  # BCP-47 ("ru", "en") — MiniMax uses "Russian", "English", …
    )
    speed: Optional[float] = None  # 0.5 – 2.0 typical, 1.0 = normal
    volume: Optional[float] = None  # 0.0 – 10.0 (provider-specific scale)
    pitch: Optional[int] = None  # semitone offset (provider-specific range)
    emotion: Optional[str] = None  # e.g. "happy", "neutral", "sad"
    sample_rate: Optional[int] = None  # Hz — provider default if None
    format: TTSFormat = TTSFormat.PCM  # desired output container
    text_normalization: Optional[bool] = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if isinstance(self.extra, dict):
            object.__setattr__(self, "extra", MappingProxyType(self.extra))


@dataclass(frozen=True)
class TTSAudio:
    """Non-streaming result. `samples` are int16 little-endian PCM unless
    `format` says otherwise; `sample_rate` is what the caller should feed
    into its playback sink."""

    samples: bytes
    sample_rate: int
    format: TTSFormat = TTSFormat.PCM
    raw: Any | None = None  # the original provider response, kept for diagnostics

    @property
    def duration_s(self) -> float:
        """Estimated duration in seconds (assumes 16-bit mono PCM)."""
        if self.format != TTSFormat.PCM:
            # Bytes-per-second differs for compressed formats; callers who care
            # can decode the container themselves using `raw` / soundfile.
            return 0.0
        bytes_per_sample = 2  # int16
        return len(self.samples) / (self.sample_rate * bytes_per_sample)


@dataclass(frozen=True)
class TTSChunk:
    """A single streaming frame.

    Providers MUST emit at least one final chunk with `finish_reason` set so
    callers can detect end-of-stream deterministically.
    """

    samples: bytes = b""
    sample_rate: int = 0
    format: TTSFormat = TTSFormat.PCM
    finish_reason: Optional[str] = None  # "stop" | "error"


# ---------------------------------------------------------------------------
# ABC
# ---------------------------------------------------------------------------


class TTSProvider(abc.ABC):
    """Async-only contract for text-to-speech providers.

    Implementations:
        - MiniMaxTTSProvider   (MiniMax T2A v2 HTTP)
        - FakeTTSProvider      (deterministic, for tests)
    """

    name: str = "abstract"

    @abc.abstractmethod
    async def synthesize(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> TTSAudio:
        """Run a non-streaming synthesis. Raises `TTSError` on failure."""

    @abc.abstractmethod
    async def stream(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> AsyncIterator[TTSChunk]:
        """Run a streaming synthesis.

        Implementation contract:

        * Providers MUST raise :class:`TTSError` BEFORE yielding anything if
          the initial request fails — once the first chunk is yielded,
          mid-stream failures become ``TTSChunk(finish_reason="error")``
          instead, mirroring the LLM contract.
        * Providers MUST always emit at least one final chunk with
          ``finish_reason`` set (``"stop"`` on success, ``"error"`` on
          mid-stream failure) so callers can detect end-of-stream
          deterministically.

        .. note::

           The current MiniMax implementation returns 0 or 1 chunks
           (it buffers the full SSE stream and yields one terminal chunk
           with ``finish_reason="stop"``). Code that needs true
           chunk-per-frame streaming should plan for this — the contract
           accepts a single terminal chunk as a valid v1 implementation.
        """
        raise NotImplementedError

    async def aclose(self) -> None:  # noqa: D401 — async context-manager hook
        """Release resources. Default impl is no-op for stateless providers.

        Implementations SHOULD make ``aclose()`` idempotent so callers can
        use it in ``finally`` blocks without a separate guard.
        """
        return None


# ---------------------------------------------------------------------------
# Built-in fakes for tests + local development
# ---------------------------------------------------------------------------


class FakeTTSProvider(TTSProvider):
    """Deterministic in-memory TTS provider for tests and offline dev.

    Echoes ``text`` as a synthetic PCM buffer (the text repeated, encoded
    as 16-bit mono samples). Always emits ``finish_reason="stop"`` for
    streaming. ``aclose()`` is a no-op and idempotent.

    Use this wherever a unit test needs a ``TTSProvider`` instance but the
    test is not about TTS itself (e.g. exercising downstream consumers in
    ``tts_node`` or ``dialogue_node``).
    """

    name = "fake"

    def __init__(self, *, sample_rate: int = 32_000) -> None:
        self._sample_rate = sample_rate

    async def synthesize(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> TTSAudio:
        if not text or not text.strip():
            # Lazy import — keep `tts.py` import-cheap so downstream
            # packages that only need the dataclasses don't pay for the
            # full errors module on import time.
            from rob_box_llm.errors import TTSBadRequestError

            raise TTSBadRequestError("text is empty", provider=self.name)
        s = settings or TTSSettings()
        sample_rate = s.sample_rate or self._sample_rate
        # Deterministic fake: encode len(text) 16-bit samples with
        # constant value 1. Same input → same bytes → stable assertions.
        import struct as _struct

        n_samples = max(len(text.encode("utf-8")), 1)
        samples = _struct.pack(f"<{n_samples}h", *([1] * n_samples))
        return TTSAudio(
            samples=samples,
            sample_rate=sample_rate,
            format=s.format,
            raw={"fake": True, "text": text},
        )

    async def stream(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> AsyncIterator[TTSChunk]:
        audio = await self.synthesize(text, settings=settings)
        yield TTSChunk(
            samples=audio.samples,
            sample_rate=audio.sample_rate,
            format=audio.format,
            finish_reason="stop",
        )

    async def aclose(self) -> None:
        return None


__all__ = [
    "TTSProvider",
    "TTSAudio",
    "TTSChunk",
    "TTSSettings",
    "TTSFormat",
    "FakeTTSProvider",
]
