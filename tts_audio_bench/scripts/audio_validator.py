"""Audio validation helpers for the TTS audio test bench.

These checks turn the bench's raw AudioData captures into pass/fail
verdicts:

* :func:`check_joints` — concatenate published PCM chunks and report
  any chunk-boundary discontinuity. The TTS pipeline produces one
  continuous audio stream from a provider response; if concatenation
  produces a click, either a chunk was dropped or the sample rate was
  mis-aligned (e.g. WAV header leaked into the PCM stream).

* :func:`duration_report` — assert the published audio duration is in
  a sensible range. The bench's fixture is a 1.0 s sine wave; we allow
  some slack on each side to account for resampling rounding.

* :func:`validate_wav_header` — cross-check a written WAV's header
  matches the expected sample rate / mono / int16 contract.

The thresholds are deliberately conservative — the goal is to catch
real regressions, not to chase sample-perfect equality with the
fixture (which would be impossible across resampling/format
round-trips).
"""
from __future__ import annotations

import struct
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


# Discontinuity threshold: in int16 units. The fixture is a 100 Hz sine
# at amplitude 0.6 — the max adjacent-sample jump for that signal is
# ~775 int16 units. The threshold leaves a comfortable safety margin so
# resampling artefacts (linear interpolation rounding) don't trip the
# check, while still flagging real breaks (a chunk dropped or a WAV
# header leaking into the PCM stream produces jumps in the 5k-15k range
# easily). 4000 is the conservative midpoint.
JOINT_THRESHOLD = 4000


@dataclass(frozen=True)
class JointReport:
    ok: bool
    max_jump: int
    samples: int

    def describe(self) -> str:
        return f"joints: ok={self.ok} max_jump={self.max_jump} samples={self.samples}"


def check_joints(pcm: bytes) -> JointReport:
    """Walk ``pcm`` as int16 LE samples and flag the largest adjacent jump.

    The check is intra-stream — we look at every adjacent pair, so
    chunk boundaries are tested just like any other transition.
    """
    n = len(pcm) // 2
    if n < 2:
        return JointReport(ok=True, max_jump=0, samples=n)
    samples = struct.unpack(f"<{n}h", pcm[: n * 2])
    max_jump = 0
    for prev, curr in zip(samples[:-1], samples[1:]):
        jump = abs(int(curr) - int(prev))
        if jump > max_jump:
            max_jump = jump
    return JointReport(
        ok=max_jump <= JOINT_THRESHOLD,
        max_jump=max_jump,
        samples=n,
    )


@dataclass(frozen=True)
class DurationReport:
    ok: bool
    duration_s: float
    expected_min_s: float
    expected_max_s: float

    def describe(self) -> str:
        return (
            f"duration: {self.duration_s:.3f}s in "
            f"[{self.expected_min_s:.2f}, {self.expected_max_s:.2f}] "
            f"({'OK' if self.ok else 'OUT-OF-RANGE'})"
        )


def duration_report(
    duration_s: float,
    *,
    expected_min_s: float,
    expected_max_s: float,
) -> DurationReport:
    return DurationReport(
        ok=expected_min_s <= duration_s <= expected_max_s,
        duration_s=duration_s,
        expected_min_s=expected_min_s,
        expected_max_s=expected_max_s,
    )


@dataclass(frozen=True)
class WavHeaderReport:
    ok: bool
    sample_rate: int
    channels: int
    sample_width: int
    frames: int
    reason: Optional[str] = None

    def describe(self) -> str:
        if self.ok:
            return (
                f"wav: sr={self.sample_rate} ch={self.channels} "
                f"sw={self.sample_width} frames={self.frames}"
            )
        return f"wav: BAD ({self.reason})"


def validate_wav_header(
    path: Path,
    *,
    expected_sample_rate: Optional[int] = None,
    expected_channels: int = 1,
    expected_sample_width: int = 2,
) -> WavHeaderReport:
    """Open ``path`` and cross-check the RIFF/WAVE header."""
    try:
        with wave.open(str(path), "rb") as w:
            sr = w.getframerate()
            ch = w.getnchannels()
            sw = w.getsampwidth()
            frames = w.getnframes()
    except (wave.Error, FileNotFoundError, EOFError) as exc:
        return WavHeaderReport(
            ok=False, sample_rate=0, channels=0, sample_width=0, frames=0,
            reason=str(exc),
        )

    problems = []
    if expected_sample_rate is not None and sr != expected_sample_rate:
        problems.append(f"sr={sr} != {expected_sample_rate}")
    if ch != expected_channels:
        problems.append(f"ch={ch} != {expected_channels}")
    if sw != expected_sample_width:
        problems.append(f"sw={sw} != {expected_sample_width}")

    return WavHeaderReport(
        ok=not problems,
        sample_rate=sr,
        channels=ch,
        sample_width=sw,
        frames=frames,
        reason="; ".join(problems) if problems else None,
    )


@dataclass(frozen=True)
class AudioDataReport:
    """Structural validation of a raw AudioData message.

    The ``tts_audio_bench`` bench uses this to confirm that anything
    flowing through ``/voice/audio/speech`` matches the contract:
    int16 little-endian, mono, configured output sample rate.
    """

    ok: bool
    sample_rate: int = 0
    channels: int = 0
    sample_width: int = 0  # bytes — int16 == 2
    layout: str = ""  # "little_endian" | "big_endian" | "unknown"
    samples: int = 0
    expected_sample_rate: Optional[int] = None
    reason: Optional[str] = None

    def describe(self) -> str:
        if self.ok:
            return (
                f"audio: sr={self.sample_rate} ch={self.channels} "
                f"sw={self.sample_width} layout={self.layout} samples={self.samples}"
            )
        return f"audio: BAD ({self.reason})"


def validate_audio_data_message(
    msg: Any,
    *,
    expected_sample_rate: Optional[int] = 16000,
) -> AudioDataReport:
    """Validate a single ``audio_common_msgs/AudioData`` message.

    Accepts anything with ``.data`` (bytes) and ``.info`` (object with
    ``sample_rate``, ``channels``, ``layout``) — the canonical
    AudioData message from ROS, plus the bench's stub.
    """
    try:
        raw = bytes(msg.data)
    except Exception as exc:  # noqa: BLE001
        return AudioDataReport(
            ok=False, reason=f"cannot read .data as bytes: {exc!r}"
        )
    info = getattr(msg, "info", None)
    sr = int(getattr(info, "sample_rate", 0)) if info is not None else 0
    ch = int(getattr(info, "channels", 1)) if info is not None else 1
    layout = str(getattr(info, "layout", "") or "") if info is not None else ""

    # The bench's stub AudioData does not carry .info; in that case we
    # fall back to the configured expected_sample_rate and assume mono
    # int16 little-endian — the production tts_node contract on
    # /voice/audio/speech. Override via the keyword argument when
    # validation requires different parameters.
    if info is None:
        if expected_sample_rate is not None:
            sr = expected_sample_rate
        ch = 1
        layout = "little_endian"

    if len(raw) % 2 != 0:
        return AudioDataReport(
            ok=False,
            sample_rate=sr,
            channels=ch,
            sample_width=0,
            layout=layout,
            samples=0,
            expected_sample_rate=expected_sample_rate,
            reason=f"AudioData bytes not aligned to int16: {len(raw)} bytes",
        )

    sw = 2  # int16
    # Big-endian sample layout would imply MSB first (>, not <).
    if layout and layout not in ("little_endian", "iec60958"):
        if layout in ("big_endian",):
            # Big-endian is not what tts_node publishes; flag it.
            samples_be = struct.unpack(f">{len(raw) // sw}h", raw)
            peak = max(abs(int(s)) for s in samples_be) if samples_be else 0
            return AudioDataReport(
                ok=False,
                sample_rate=sr,
                channels=ch,
                sample_width=sw,
                layout=layout,
                samples=len(samples_be),
                expected_sample_rate=expected_sample_rate,
                reason=(
                    f"big-endian samples unexpected "
                    f"(peak={peak})"
                ),
            )

    samples_le = struct.unpack(f"<{len(raw) // sw}h", raw)
    samples = len(samples_le)
    peak = max(abs(int(s)) for s in samples_le) if samples else 0
    if samples == 0:
        return AudioDataReport(
            ok=False,
            sample_rate=sr,
            channels=ch,
            sample_width=sw,
            layout="little_endian",
            samples=0,
            expected_sample_rate=expected_sample_rate,
            reason="empty AudioData",
        )

    problems: list[str] = []
    if ch != 1:
        problems.append(f"ch={ch} != 1 (mono required)")
    if expected_sample_rate is not None and sr != expected_sample_rate:
        problems.append(f"sr={sr} != {expected_sample_rate}")
    if peak > 32767:
        problems.append(f"peak={peak} > int16 range")
    if layout and layout not in ("little_endian", "iec60958"):
        # Empty / unknown layout is treated as unknown — pass through with
        # a soft note so the downstream decoder still works.
        pass

    return AudioDataReport(
        ok=not problems,
        sample_rate=sr,
        channels=ch,
        sample_width=sw,
        layout="little_endian" if not layout or layout in ("little_endian", "iec60958") else layout,
        samples=samples,
        expected_sample_rate=expected_sample_rate,
        reason="; ".join(problems) if problems else None,
    )