#!/usr/bin/env python3
"""Unit tests for the real_subscriber subprocess entry point.

These tests run the subscriber in-process via ``runpy`` so we don't
shell out to ``python3`` — that keeps the tests fast and removes the
dependency on the host Python in the PATH. The subscriber is still
exercised end-to-end (stdin -> WAV -> JSON report), so any breakage in
the wire format or the WAV header validation surfaces here.
"""
from __future__ import annotations

import base64
import io
import json
import os
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import Any, Dict, List

# Run subscriber as __main__ without spawning a subprocess.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tts_audio_bench.scripts import real_subscriber as rs  # noqa: E402


def _frame_bytes(sample_rate: int, duration_s: float = 1.0, freq: float = 440.0) -> bytes:
    """Synthesise a 1 s sine wave at int16 LE mono — same shape as a fixture."""
    import math
    n = int(sample_rate * duration_s)
    raw = bytearray()
    for i in range(n):
        v = int(0.6 * 32767 * math.sin(2 * math.pi * freq * i / sample_rate))
        raw.extend(struct.pack("<h", v))
    return bytes(raw)


def _drive_subscriber(stdin_payload: str) -> Dict[str, Any]:
    """Drive ``_cli`` with a captured stdin and return the parsed report."""
    saved_stdin = sys.stdin
    saved_argv = sys.argv
    saved_stdout = sys.stdout
    saved_stderr = sys.stderr
    try:
        sys.stdin = io.StringIO(stdin_payload)
        sys.stdout = io.StringIO()
        sys.stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpd:
            tmp_path = Path(tmpd) / "out.wav"
            sys.argv = [
                "real_subscriber.py",
                "--out", str(tmp_path),
                "--expected-sample-rate", "16000",
                "--ffprobe-check",
            ]
            rc = rs._cli()
            out_text = sys.stdout.getvalue()
            try:
                report = json.loads(out_text.strip().splitlines()[-1])
            except (json.JSONDecodeError, IndexError):
                report = {"ok": False, "raw_stdout": out_text, "rc": rc}
            report["_rc"] = rc
            report["_wav_path"] = str(tmp_path)
            return report
    finally:
        sys.stdin = saved_stdin
        sys.argv = saved_argv
        sys.stdout = saved_stdout
        sys.stderr = saved_stderr


class RealSubscriberTests(unittest.TestCase):
    def test_happy_path(self) -> None:
        """A single frame → WAV with right header, ffprobe confirms, JSON OK."""
        raw = _frame_bytes(16000)
        payload = json.dumps({
            "frame_index": 0,
            "publish_t_s": 0.0,
            "sample_rate": 16000,
            "channels": 1,
            "sample_width": 2,
            "layout": "little_endian",
            "data_b64": base64.b64encode(raw).decode("ascii"),
        }) + "\n"
        report = _drive_subscriber(payload)
        self.assertTrue(report.get("ok"), msg=f"subscriber said not-ok: {report}")
        self.assertEqual(report["frames"], 1)
        self.assertEqual(report["bytes_total"], len(raw))
        self.assertEqual(report["sample_rate"], 16000)
        self.assertEqual(report["channels"], 1)
        self.assertEqual(report["sample_width"], 2)
        self.assertEqual(report.get("ffprobe_ok"), True)
        self.assertEqual(report["_rc"], 0)

    def test_no_frames(self) -> None:
        """Empty stdin → exit code 2 + ok=False."""
        report = _drive_subscriber("")
        self.assertFalse(report.get("ok"))
        self.assertEqual(report["_rc"], 2)

    def test_heartbeat_only(self) -> None:
        """Frames without data_b64 (heartbeats) are ignored."""
        line = json.dumps({"event": "heartbeat"}) + "\n"
        report = _drive_subscriber(line)
        self.assertFalse(report.get("ok"))
        self.assertEqual(report["_rc"], 2)

    def test_bad_base64(self) -> None:
        """Invalid base64 → exit code 3."""
        line = json.dumps({"data_b64": "!!!not base64!!!"}) + "\n"
        report = _drive_subscriber(line)
        self.assertFalse(report.get("ok"))
        self.assertEqual(report["_rc"], 3)

    def test_wrong_sample_width(self) -> None:
        """sample_width=4 (int32) is rejected — tts_node contract is int16."""
        line = json.dumps({
            "data_b64": base64.b64encode(b"\x00" * 4).decode("ascii"),
            "sample_width": 4,
        }) + "\n"
        report = _drive_subscriber(line)
        self.assertFalse(report.get("ok"))
        self.assertEqual(report["_rc"], 3)

    def test_multi_frame_concatenation(self) -> None:
        """Multiple frames concatenate into one WAV with duration = sum."""
        chunk = _frame_bytes(16000, duration_s=0.25)
        lines = []
        for i in range(4):
            lines.append(json.dumps({
                "frame_index": i,
                "publish_t_s": i * 0.25,
                "sample_rate": 16000,
                "channels": 1,
                "sample_width": 2,
                "layout": "little_endian",
                "data_b64": base64.b64encode(chunk).decode("ascii"),
            }))
        report = _drive_subscriber("\n".join(lines) + "\n")
        self.assertTrue(report.get("ok"), msg=f"subscriber said not-ok: {report}")
        self.assertEqual(report["frames"], 4)
        self.assertAlmostEqual(report["duration_s"], 1.0, places=3)


if __name__ == "__main__":
    unittest.main()