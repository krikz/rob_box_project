"""Audio transcode helpers for the ROS voice stack.

Used by ``tts_node`` to bridge the gap between the byte payload returned by
a :class:`rob_box_llm.tts.TTSProvider` and the format the ROS playback sink
expects (``audio_common_msgs/AudioData`` carries raw int16 little-endian PCM
bytes; the ReSpeaker sink additionally wants 16 kHz mono).

The helpers are intentionally **dependency-light**: stdlib ``wave`` handles
WAV; a lazy import is used for ``pydub`` (preferred) and a lazy subprocess
fallback for ``ffmpeg`` (when neither pydub nor soundfile are installed).
That way ``rob_box_voice`` stays importable on minimal containers (no audio
encoder binaries in the bench stack).

Contract::

    pcm_bytes, sample_rate, channels = to_pcm_int16(samples, fmt)

* For ``PCM`` input the bytes are returned as-is.
* For ``WAV`` input we extract the data chunk and meta (sample rate,
  channels) via the stdlib.
* For ``MP3`` / ``OGG`` we delegate to pydub (or the ``ffmpeg`` CLI).
* Channels > 1 are downmixed to mono via simple averaging — the rob_box
  pipeline downstream upsamples mono→stereo for ReSpeaker anyway, so we
  drop extra channels at the earliest point to keep the buffers small.

All conversions land on **little-endian int16 PCM** so the result is ready
to wrap into an :class:`audio_common_msgs.msg.AudioData`.

See ADR-0003 §2.3 (output contract) — these helpers ensure the
``/voice/audio/speech`` topic format stays stable regardless of provider.
"""

from __future__ import annotations

import io
import logging
import shutil
import subprocess
import wave
from dataclasses import dataclass
from typing import Tuple

from rob_box_llm.tts import TTSFormat

_log = logging.getLogger(__name__)


class AudioTranscodeError(Exception):
    """Raised when transcoding fails for technical reasons (bad input bytes,
    missing encoder, etc.). Distinct from provider-side errors so the caller
    can categorize "the network was fine, but we can't play the result"
    separately from "the network is broken"."""

    def __init__(self, message: str, *, fmt: str, reason: str) -> None:
        super().__init__(message)
        self.fmt = fmt
        self.reason = reason


@dataclass(frozen=True)
class DecodedAudio:
    """Result of transcoding a provider audio blob into ROS-ready PCM.

    Attributes:
        pcm: raw little-endian int16 PCM bytes (no header).
        sample_rate: Hz, e.g. 16000 / 32000.
        channels: 1 (mono) — multi-channel inputs are downmixed.
        source_format: ``TTSFormat`` the input was carrying (for logging).
    """

    pcm: bytes
    sample_rate: int
    channels: int
    source_format: TTSFormat


def to_pcm_int16(
    samples: bytes,
    fmt: TTSFormat,
    *,
    default_sample_rate: int = 32000,
) -> DecodedAudio:
    """Transcode an audio blob from a TTS provider into PCM int16 mono.

    Args:
        samples: bytes returned by ``TTSAudio.samples``. Container depends
            on ``fmt``: ``PCM`` → raw int16 LE; ``WAV`` → RIFF/WAVE blob;
            ``MP3`` / ``OGG`` → compressed blob.
        fmt: the container format the provider said it returned.
        default_sample_rate: Hz assumed when ``fmt`` is ``PCM`` (no
            container to inspect) and the provider didn't surface a rate
            elsewhere. MiniMax defaults to 32000 Hz.

    Returns:
        :class:`DecodedAudio` ready to wrap into ``AudioData``.

    Raises:
        AudioTranscodeError: on bad bytes, missing encoder, or unsupported
            format combination. The error carries ``fmt`` and ``reason``
            so callers can branch (e.g. log vs. user-visible message).
    """
    if not samples:
        raise AudioTranscodeError("empty audio payload from provider", fmt=fmt.value, reason="empty")

    if fmt == TTSFormat.PCM:
        # No container to inspect; provider's reported sample_rate is the
        # authoritative one. We don't channel-strip here because raw PCM
        # has no channel marker — caller is responsible for treating this
        # as mono (MiniMax always returns mono PCM, per docs).
        if len(samples) % 2:
            raise AudioTranscodeError(
                "raw PCM payload is not aligned to 16-bit samples",
                fmt=TTSFormat.PCM.value,
                reason="unaligned_pcm",
            )
        return DecodedAudio(
            pcm=samples,
            sample_rate=default_sample_rate,
            channels=1,
            source_format=TTSFormat.PCM,
        )

    if fmt == TTSFormat.WAV:
        pcm, sr, ch = _decode_wav(samples)
        return DecodedAudio(pcm=pcm, sample_rate=sr, channels=ch, source_format=TTSFormat.WAV)

    if fmt in (TTSFormat.MP3, TTSFormat.OGG):
        pcm, sr, ch = _decode_compressed(samples, fmt)
        return DecodedAudio(pcm=pcm, sample_rate=sr, channels=ch, source_format=fmt)

    # Defensive: in case a future provider adds an enum value we don't know.
    raise AudioTranscodeError(f"unsupported TTSFormat: {fmt!r}", fmt=fmt.value, reason="unsupported_format")


def _decode_wav(samples: bytes) -> Tuple[bytes, int, int]:
    """Decode a WAV blob using stdlib ``wave``.

    Always returns 16-bit PCM (we bit-convert if the source is e.g. 24-bit
    or 32-bit-float — MiniMax uses 16-bit, so this branch is rarely hit
    but keeps the contract honest).
    """
    try:
        with wave.open(io.BytesIO(samples), "rb") as w:
            sr = w.getframerate()
            ch = w.getnchannels()
            sampwidth = w.getsampwidth()
            nframes = w.getnframes()
            raw = w.readframes(nframes)
    except wave.Error as exc:
        raise AudioTranscodeError(f"WAV decode failed: {exc}", fmt=TTSFormat.WAV.value, reason="bad_wav") from exc

    if sampwidth == 2:
        pcm = raw  # already int16 LE
    elif sampwidth == 1:
        # unsigned 8-bit → signed 16-bit (centered at 0)
        import numpy as np  # local import — keep module-light

        arr = np.frombuffer(raw, dtype=np.uint8).astype(np.int16) - 128
        arr = (arr << 8).astype(np.int16)
        pcm = arr.tobytes()
    elif sampwidth == 3:
        # 24-bit signed LE → 16-bit. We truncate the low byte.
        import numpy as np

        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        # LE: b[:,0]=L, b[:,1]=M, b[:,2]=H
        s24 = b[:, 0].astype(np.int32) | (b[:, 1].astype(np.int32) << 8) | (b[:, 2].astype(np.int32) << 16)
        # sign-extension from 24-bit
        s24 = (s24 ^ 0x800000) - 0x800000
        s16 = (s24 >> 8).astype(np.int16)
        pcm = s16.tobytes()
    elif sampwidth == 4:
        # 32-bit signed → 16-bit (high 16 bits)
        import numpy as np

        s32 = np.frombuffer(raw, dtype=np.int32)
        s16 = (s32 >> 16).astype(np.int16)
        pcm = s16.tobytes()
    else:
        raise AudioTranscodeError(
            f"WAV sample width {sampwidth} bytes not supported",
            fmt=TTSFormat.WAV.value,
            reason="unsupported_sampwidth",
        )

    if ch > 1:
        pcm = _downmix_int16(pcm, ch, fmt=TTSFormat.WAV)
        ch = 1

    return pcm, sr, ch


def _downmix_int16(pcm: bytes, channels: int, *, fmt: TTSFormat) -> bytes:
    """Downmix interleaved int16 LE PCM to mono with overflow-safe averaging."""
    if channels <= 1:
        return pcm
    if len(pcm) % (2 * channels) != 0:
        raise AudioTranscodeError(
            "PCM frame is not aligned to the channel count",
            fmt=fmt.value,
            reason="unaligned_channels",
        )

    import numpy as np

    frames = np.frombuffer(pcm, dtype="<i2").reshape(-1, channels)
    return frames.astype(np.int32).mean(axis=1).astype("<i2").tobytes()


def _decode_compressed(samples: bytes, fmt: TTSFormat) -> Tuple[bytes, int, int]:
    """Decode MP3 / OGG via pydub (preferred) or ffmpeg subprocess.

    Both libraries are **optional** — ``rob_box_voice`` doesn't hard-depend
    on them so a minimal install (no audio codecs) still imports. When
    neither is available we raise ``AudioTranscodeError``; the
    ``tts_node`` caller surfaces a typed transcode failure to the operator.

    The production ``voice-base`` image and ``package.xml`` install ffmpeg;
    minimal developer environments may only exercise PCM/WAV paths.
    """
    # Strategy 1: pydub (preferred — pure-Python wrapper, no subprocess)
    try:
        from pydub import AudioSegment  # type: ignore[import-not-found]

        seg = AudioSegment.from_file(io.BytesIO(samples), format=fmt.value)
        seg = seg.set_channels(1).set_sample_width(2)
        return seg.raw_data, seg.frame_rate, 1
    except ImportError:
        pass  # pydub not installed — try ffmpeg
    except Exception as exc:  # noqa: BLE001 — fall through to ffmpeg
        _log.debug("pydub decode failed (%s); falling back to ffmpeg", exc)

    # Strategy 2: ffmpeg subprocess
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise AudioTranscodeError(
            f"cannot decode {fmt.value}: pydub and ffmpeg both unavailable",
            fmt=fmt.value,
            reason="no_decoder",
        )

    try:
        proc = subprocess.run(  # noqa: S603 — input is bytes from TTS provider, no shell
            [
                ffmpeg,
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                fmt.value,
                "-i",
                "pipe:0",
                "-f",
                "s16le",
                "-acodec",
                "pcm_s16le",
                "-ac",
                "1",
                "-ar",
                "44100",
                "pipe:1",
            ],
            input=samples,
            capture_output=True,
            check=True,
            timeout=30,
        )
    except subprocess.CalledProcessError as exc:
        raise AudioTranscodeError(
            f"ffmpeg decode failed: {exc.stderr.decode(errors='replace')[:200]}",
            fmt=fmt.value,
            reason="ffmpeg_failed",
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise AudioTranscodeError(
            "ffmpeg decode timeout after 30s",
            fmt=fmt.value,
            reason="ffmpeg_timeout",
        ) from exc

    return proc.stdout, 44100, 1
