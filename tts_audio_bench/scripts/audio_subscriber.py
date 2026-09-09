#!/usr/bin/env python3
"""Standalone ROS AudioData subscriber used as a verifier.

In production this is a normal ROS2 subscriber node that listens on
``/voice/audio/speech`` and forwards frames to the audio sink (typically
``audio_play`` or ``sound_node``). For the bench we provide two
verifier paths:

* this module's :class:`WavRecorder` — fast in-process recorder used
  for the in-process checks (joints, duration, header);
* ``scripts/real_subscriber.py`` — the bench's *real subprocess
  subscriber*. It runs as a separate Python process and consumes the
  AudioData frames over a stdio pipe. Its WAV file + JSON report are
  the canonical "what a real ROS subscriber would record" artifacts
  the bench commits to disk under
  ``artifacts/<scenario>_real_subscriber.wav``.

Run via :mod:`tts_audio_bench.scripts.run_bench` — this module is
imported by ``run_bench`` to build the in-process verifier. The
real-subprocess path is wired in ``run_bench.run_audio_data_subscriber``.
The CLI form below is provided for ad-hoc runs against a hand-rolled
publisher.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
import wave
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any, List, Optional


@dataclass
class SubscriberReport:
    """A bundle of subscriber-level statistics."""

    first_frame_at_s: Optional[float] = None
    last_frame_at_s: Optional[float] = None
    frames: int = 0
    bytes_total: int = 0
    sample_rate: int = 0
    channels: int = 1
    sample_width: int = 2
    artifact: Optional[str] = None
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "first_frame_at_s": self.first_frame_at_s,
            "last_frame_at_s": self.last_frame_at_s,
            "frames": self.frames,
            "bytes_total": self.bytes_total,
            "sample_rate": self.sample_rate,
            "channels": self.channels,
            "sample_width": self.sample_width,
            "artifact": self.artifact,
            "warnings": self.warnings,
        }


class WavRecorder:
    """Records ``AudioData`` messages into a WAV file."""

    def __init__(self, *, sample_rate: int, channels: int = 1, sample_width: int = 2) -> None:
        self.sample_rate = sample_rate
        self.channels = channels
        self.sample_width = sample_width
        self._t0: Optional[float] = None
        self._last_t: Optional[float] = None
        self._frames: List[Any] = []
        self._bytes = 0

    def __call__(self, msg: Any) -> None:
        """``AudioData`` callback shim — invoked from the bench mock publisher."""
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        self._last_t = now
        self._frames.append(msg)
        # AudioData exposes ``data`` as a list[int] in the bench stub.
        # Production AudioData uses uint8[] — same bytes.
        if hasattr(msg, "data") and isinstance(msg.data, (list, bytes, bytearray)):
            self._bytes += len(msg.data)
        else:
            raise TypeError(f"unexpected AudioData payload type: {type(msg.data)!r}")

    @property
    def first_arrival_s(self) -> Optional[float]:
        return self._t0

    @property
    def elapsed_s(self) -> float:
        if self._t0 is None or self._last_t is None:
            return 0.0
        return self._last_t - self._t0

    def to_wav(self, out_path: Path) -> SubscriberReport:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if not self._frames:
            return SubscriberReport(artifact=str(out_path), warnings=["no frames captured"])

        with wave.open(str(out_path), "wb") as w:
            w.setnchannels(self.channels)
            w.setsampwidth(self.sample_width)
            w.setframerate(self.sample_rate)
            for msg in self._frames:
                w.writeframes(bytes(msg.data))

        report = SubscriberReport(
            first_frame_at_s=self._t0,
            last_frame_at_s=self._last_t,
            frames=len(self._frames),
            bytes_total=self._bytes,
            sample_rate=self.sample_rate,
            channels=self.channels,
            sample_width=self.sample_width,
            artifact=str(out_path),
        )
        # Cross-check via ffprobe if available — otherwise trust wave.
        if shutil.which("ffprobe") is not None:
            try:
                proc = subprocess.run(
                    [
                        "ffprobe", "-v", "error", "-show_streams",
                        "-of", "json", str(out_path),
                    ],
                    check=True, capture_output=True, text=True, timeout=10,
                )
                info = json.loads(proc.stdout)
                streams = info.get("streams") or []
                if streams:
                    s = streams[0]
                    if s.get("sample_rate") != str(self.sample_rate):
                        report.warnings.append(
                            f"ffprobe sr={s.get('sample_rate')} != expected {self.sample_rate}"
                        )
                    if s.get("channels") != self.channels:
                        report.warnings.append(
                            f"ffprobe ch={s.get('channels')} != expected {self.channels}"
                        )
            except (subprocess.CalledProcessError, json.JSONDecodeError) as exc:
                report.warnings.append(f"ffprobe cross-check failed: {exc}")
        return report


def _cli() -> int:
    """CLI: run as a standalone recorder, feeding from stdin (JSON lines).

    Each stdin line is a JSON object with a base64-encoded ``data``
    payload, e.g. ``{"sample_rate": 16000, "data_b64": "..."}``. Lines
    without ``data_b64`` are ignored (useful for heartbeat events).
    """
    import base64

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    args = parser.parse_args()

    recorder = WavRecorder(sample_rate=args.sample_rate)

    deadline = time.monotonic() + args.timeout_s
    for line in sys.stdin:
        if time.monotonic() > deadline:
            break
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        b64 = obj.get("data_b64")
        if not b64:
            continue
        payload = base64.b64decode(b64)

        # Wrap the raw bytes into a duck-typed AudioData.
        msg = SimpleNamespace(data=list(payload))
        recorder(msg)
    report = recorder.to_wav(args.out)
    json.dump(report.to_dict(), sys.stdout, indent=2)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(_cli())