#!/usr/bin/env python3
"""Real subprocess subscriber for the TTS audio test bench.

This module is the bench's *real subscriber* — a standalone process that
mirrors what a production ``ros2 topic echo /voice/audio/speech`` consumer
does, just over a stdio pipe instead of DDS. It accepts AudioData frames
serialised as JSON-lines on stdin and writes them to a WAV file plus a
JSON report on stdout.

Why a subprocess and not an in-process hook?

A real ROS subscriber lives in a separate process from the publisher.
Running the bench's verifier in a subprocess:

* exercises the same kind of process boundary a real ROS subscriber has;
* proves the AudioData payload is self-contained — i.e. nothing in the
  bench process leaks into the verifier beyond what was actually put on
  the "wire" (here: a pipe);
* gives the bench a strict, machine-checkable handoff for the WAV file
  the subprocess produces — that file is what we assert on, not an
  in-memory object.

Wire format
-----------

Each line on stdin is a JSON object with the shape::

    {
        "frame_index": 0,
        "publish_t_s": 0.123,
        "sample_rate": 16000,
        "channels": 1,
        "sample_width": 2,
        "layout": "little_endian",
        "data_b64": "<base64-encoded int16 LE bytes>"
    }

A line with no ``data_b64`` (e.g. ``{"event": "end_of_stream"}``) is
ignored — useful for heartbeats.

Output
------

On success the process prints a single JSON object to stdout, e.g.::

    {
        "frames": 4,
        "bytes_total": 32000,
        "samples": 16000,
        "duration_s": 1.0,
        "sample_rate": 16000,
        "channels": 1,
        "sample_width": 2,
        "wav_path": "/abs/path/to/out.wav",
        "first_frame_at_s": 0.123,
        "last_frame_at_s": 0.789,
        "ffprobe_ok": true
    }

Exit codes::

    0 — subscriber received at least one frame, WAV written, header OK
    2 — no frames received (subscriber never got data on stdin)
    3 — frame payload invalid (e.g. wrong sample_width)
    4 — WAV header validation failed
    5 — ffprobe cross-check failed when ffprobe was available

CLI usage::

    python -m tts_audio_bench.scripts.real_subscriber \
        --out /tmp/audio.wav \
        --expected-sample-rate 16000 \
        --ffprobe-check

When invoked from ``run_bench``, the bench orchestrates the subprocess
(see :func:`run_audio_data_subscriber`).
"""
from __future__ import annotations

import argparse
import base64
import json
import shutil
import subprocess
import sys
import time
import wave
from pathlib import Path
from typing import Any, Dict, List, Optional


def _read_stdin_lines(deadline_s: Optional[float] = None) -> List[str]:
    """Read stdin until EOF or deadline.

    A deadline keeps the subscriber from hanging forever if the publisher
    process crashes mid-flight. The bench always closes the pipe on
    success, so EOF is the normal termination signal.
    """
    lines: List[str] = []
    for line in sys.stdin:
        if deadline_s is not None and time.monotonic() > deadline_s:
            break
        # Strip newline but preserve empty lines (we'll filter below).
        lines.append(line.rstrip("\n"))
    return lines


def _decode_frame(obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Decode one JSON-line frame into a frame record or None on heartbeat."""
    b64 = obj.get("data_b64")
    if not b64:
        return None
    try:
        payload = base64.b64decode(b64, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"invalid base64 data_b64: {exc}") from exc
    return {
        "frame_index": int(obj.get("frame_index", -1)),
        "publish_t_s": float(obj.get("publish_t_s", 0.0)),
        "sample_rate": int(obj.get("sample_rate", 0)),
        "channels": int(obj.get("channels", 1)),
        "sample_width": int(obj.get("sample_width", 2)),
        "layout": str(obj.get("layout", "little_endian")),
        "raw": payload,
    }


def _write_wav(
    frames: List[Dict[str, Any]],
    out_path: Path,
    *,
    expected_sample_rate: Optional[int] = None,
) -> Dict[str, Any]:
    """Serialise ``frames`` to a WAV file and return a header report dict."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not frames:
        return {
            "wav_path": str(out_path),
            "ok": False,
            "reason": "no frames captured",
        }

    sample_rate = frames[0]["sample_rate"] or (expected_sample_rate or 0)
    channels = frames[0]["channels"] or 1
    sample_width = frames[0]["sample_width"] or 2

    if not sample_rate:
        return {
            "wav_path": str(out_path),
            "ok": False,
            "reason": "frame has sample_rate=0",
        }

    total_bytes = 0
    with wave.open(str(out_path), "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(sample_width)
        w.setframerate(sample_rate)
        for frame in frames:
            w.writeframes(frame["raw"])
            total_bytes += len(frame["raw"])

    # Re-open for cross-check + duration.
    report: Dict[str, Any] = {
        "wav_path": str(out_path),
        "ok": True,
        "bytes_total": total_bytes,
        "samples": total_bytes // max(1, sample_width * channels),
        "duration_s": (total_bytes / max(1, sample_width * channels)) / sample_rate,
        "sample_rate": sample_rate,
        "channels": channels,
        "sample_width": sample_width,
        "frames": len(frames),
    }

    with wave.open(str(out_path), "rb") as r:
        rsr = r.getframerate()
        rch = r.getnchannels()
        rsw = r.getsampwidth()
        rnframes = r.getnframes()
    if (
        rsr != sample_rate
        or rch != channels
        or rsw != sample_width
        or rnframes * sample_width * channels != total_bytes
    ):
        report["ok"] = False
        report["reason"] = (
            f"header mismatch: file=(sr={rsr},ch={rch},sw={rsw},n={rnframes}) "
            f"!= expected=(sr={sample_rate},ch={channels},sw={sample_width})"
        )
    return report


def _ffprobe_check(wav_path: Path) -> Dict[str, Any]:
    """Run ffprobe and report what it sees. Returns ``{"ffprobe_ok": bool, ...}``."""
    if shutil.which("ffprobe") is None:
        return {"ffprobe_ok": None, "ffprobe_skip_reason": "ffprobe not installed"}
    try:
        proc = subprocess.run(
            [
                "ffprobe", "-v", "error", "-show_streams",
                "-of", "json", str(wav_path),
            ],
            check=True, capture_output=True, text=True, timeout=10,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        return {"ffprobe_ok": False, "ffprobe_error": str(exc)}
    try:
        info = json.loads(proc.stdout)
    except json.JSONDecodeError as exc:
        return {"ffprobe_ok": False, "ffprobe_error": f"json: {exc}"}
    streams = info.get("streams") or []
    if not streams:
        return {"ffprobe_ok": False, "ffprobe_error": "no streams"}
    s = streams[0]
    return {
        "ffprobe_ok": True,
        "ffprobe_sample_rate": s.get("sample_rate"),
        "ffprobe_channels": s.get("channels"),
        "ffprobe_codec_name": s.get("codec_name"),
    }


def _cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", required=True, type=Path,
                        help="Output WAV path (overwritten).")
    parser.add_argument("--expected-sample-rate", type=int, default=16000,
                        help="Sample rate to assert against if a frame is silent.")
    parser.add_argument("--deadline-s", type=float, default=30.0,
                        help="Stop reading stdin after this many seconds.")
    parser.add_argument("--ffprobe-check", action="store_true",
                        help="Cross-check the WAV via ffprobe (requires ffprobe).")
    parser.add_argument("--strict-layout", action="store_true",
                        help="Reject any layout other than little_endian / iec60958.")
    args = parser.parse_args()

    deadline = time.monotonic() + args.deadline_s
    raw_lines = _read_stdin_lines(deadline_s=deadline)

    frames: List[Dict[str, Any]] = []
    first_t: Optional[float] = None
    last_t: Optional[float] = None
    for line in raw_lines:
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        try:
            frame = _decode_frame(obj)
        except ValueError as exc:
            print(json.dumps({"ok": False, "reason": str(exc)}))
            return 3
        if frame is None:
            continue
        if first_t is None:
            first_t = frame["publish_t_s"]
        last_t = frame["publish_t_s"]
        if args.strict_layout and frame["layout"] not in ("little_endian", "iec60958"):
            print(json.dumps({
                "ok": False,
                "reason": f"non-little-endian layout: {frame['layout']!r}",
            }))
            return 3
        if frame["sample_width"] != 2:
            print(json.dumps({
                "ok": False,
                "reason": f"unexpected sample_width={frame['sample_width']} (int16 required)",
            }))
            return 3
        frames.append(frame)

    if not frames:
        print(json.dumps({
            "ok": False,
            "reason": "no AudioData frames received on stdin within deadline",
            "lines_seen": len(raw_lines),
        }))
        return 2

    header = _write_wav(
        frames, args.out,
        expected_sample_rate=args.expected_sample_rate,
    )
    if not header["ok"]:
        print(json.dumps({"ok": False, **header}))
        return 4

    report: Dict[str, Any] = {
        "ok": True,
        "frames": len(frames),
        "bytes_total": header["bytes_total"],
        "samples": header["samples"],
        "duration_s": header["duration_s"],
        "sample_rate": header["sample_rate"],
        "channels": header["channels"],
        "sample_width": header["sample_width"],
        "wav_path": header["wav_path"],
        "first_frame_at_s": first_t,
        "last_frame_at_s": last_t,
    }

    if args.ffprobe_check:
        ff = _ffprobe_check(args.out)
        if ff.get("ffprobe_ok") is False:
            print(json.dumps({"ok": False, **header, **ff}))
            return 5
        report.update(ff)

    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    sys.exit(_cli())