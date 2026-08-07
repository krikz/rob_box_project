#!/usr/bin/env python3
"""Generate deterministic audio fixtures for the TTS audio test bench.

Each fixture is a 16-bit little-endian mono PCM reference that the bench
uses to validate end-to-end output. We synthesise a sine sweep with a
recognisable fingerprint (known RMS, zero crossings, exact byte count) so
that downstream checks can:

* assert the byte length matches expectations;
* assert the playback duration (samples / sample_rate);
* assert "joints" between concatenated chunks are smooth (no clicks /
  discontinuities) — a zero-crossing test on the concatenated stream.

Three sample rates are produced because the bench covers PCM/WAV/MP3/OGG
output formats and each provider-format round-trip may resample.

Usage:
    python make_fixture.py [--out-dir fixtures] [--duration-s 1.0]
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import wave
from pathlib import Path


def synth_sine_pcm(
    *,
    duration_s: float,
    sample_rate: int,
    freq_hz: float = 440.0,
    amplitude: float = 0.6,
) -> bytes:
    """Synthesise a mono sine wave as 16-bit LE PCM bytes.

    Amplitude is kept below 1.0 so the bench can still apply its volume
    gain without saturating. A 440 Hz tone gives a recognisable
    zero-crossing pattern that the joints checker validates.
    """
    n_samples = int(round(duration_s * sample_rate))
    out = bytearray()
    # Use struct.pack instead of numpy for portability in minimal envs.
    # n_samples of 1.0s @ 16 kHz = 16000 iters; fast enough.
    for n in range(n_samples):
        v = amplitude * math.sin(2.0 * math.pi * freq_hz * n / sample_rate)
        s16 = int(round(v * 32767.0))
        # Clamp defensively (should never trip with amplitude < 1).
        if s16 > 32767:
            s16 = 32767
        elif s16 < -32768:
            s16 = -32768
        out += struct.pack("<h", s16)
    return bytes(out)


def write_wav(path: Path, pcm: bytes, sample_rate: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(pcm)


def write_pcm(path: Path, pcm: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(pcm)


def write_mp3(path: Path, pcm: bytes, sample_rate: int) -> None:
    """Encode a WAV-equivalent PCM stream to MP3 using ffmpeg."""
    import subprocess
    import tempfile

    path.parent.mkdir(parents=True, exist_ok=True)
    # ffmpeg can ingest raw PCM via -f s16le; pipe through stdin.
    proc = subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "s16le",
            "-ar", str(sample_rate),
            "-ac", "1",
            "-i", "pipe:0",
            "-codec:a", "libmp3lame",
            "-b:a", "64k",
            "-y", str(path),
        ],
        input=pcm,
        check=True,
    )


def write_ogg(path: Path, pcm: bytes, sample_rate: int) -> None:
    """Encode to OGG/Vorbis via ffmpeg."""
    import subprocess

    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "s16le",
            "-ar", str(sample_rate),
            "-ac", "1",
            "-i", "pipe:0",
            "-codec:a", "libvorbis",
            "-b:a", "64k",
            "-y", str(path),
        ],
        input=pcm,
        check=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="tts_audio_bench/fixtures")
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--sample-rates", default="16000,24000,32000")
    parser.add_argument("--freq-hz", type=float, default=100.0)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    sample_rates = [int(s) for s in args.sample_rates.split(",") if s.strip()]

    for sr in sample_rates:
        pcm = synth_sine_pcm(
            duration_s=args.duration_s,
            sample_rate=sr,
            freq_hz=args.freq_hz,
        )
        base = f"sine_{args.freq_hz:.0f}hz_{args.duration_s:.2f}s_{sr}"
        # Raw PCM (the "PCM" container).
        write_pcm(out_dir / "pcm" / f"{base}.pcm", pcm)
        # WAV container (the "WAV" container).
        write_wav(out_dir / "wav" / f"{base}.wav", pcm, sr)
        # MP3 container (compressed).
        write_mp3(out_dir / "mp3" / f"{base}.mp3", pcm, sr)
        # OGG container (compressed).
        write_ogg(out_dir / "ogg" / f"{base}.ogg", pcm, sr)
        # Reference WAV for joints comparison (decode-and-verify source).
        write_wav(out_dir / "reference" / f"{base}.wav", pcm, sr)

    print(f"Wrote fixtures to {out_dir}/")
    for fmt in ("pcm", "wav", "mp3", "ogg", "reference"):
        d = out_dir / fmt
        if d.exists():
            files = sorted(d.iterdir())
            print(f"  {fmt}/: {len(files)} file(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())