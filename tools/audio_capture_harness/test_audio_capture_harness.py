"""Unit tests for the ``tools.audio_capture_harness`` capture/validation logic.

These tests run without ROS2, without ffmpeg, and without launching
``tts_node`` — they cover the deterministic parts of the harness:

* ``validate_audio_chunk`` — int16 LE mono contract;
* ``check_joints`` — chunk-boundary discontinuity check;
* ``write_wav_from_chunks`` — header round-trip;
* the end-to-end ``stdin`` transport, fed a synthesized JSON-lines
  frame stream similar to what the bench's ``real_subscriber`` would
  produce.

The acceptance scenario — running the harness end-to-end against a
real ``tts_node`` over ROS2 — is documented in the harness README and
runs inside the project's Docker image, where ROS2 + audio_common_msgs
are available.
"""
from __future__ import annotations

import base64
import io
import json
import os
import shutil
import struct
import subprocess
import sys
import unittest
import wave
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List, Optional

HARNESS_DIR = Path(__file__).resolve().parent
# test file lives at <worktree>/tools/audio_capture_harness/<file>.
# Walk upward to find the worktree root (the directory that contains
# ``tools/__init__.py``). This is more robust than ``parents[2]``
# because the worktree may live at an arbitrary depth (e.g. nested
# inside ``.worktrees/<name>/``), and because some test runners
# resolve ``__file__`` through different symlink paths.
def _find_worktree_root(start: Path) -> Path:
    cur = start
    for _ in range(8):  # unlikely to need more than a handful
        if (cur / "tools" / "__init__.py").exists():
            return cur
        if cur.parent == cur:
            return start
        cur = cur.parent
    return start


PROJECT_ROOT = _find_worktree_root(HARNESS_DIR)
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(HARNESS_DIR))

from tools.audio_capture_harness.audio_capture_harness import (  # noqa: E402
    CaptureHeader,
    FormatViolation,
    JOINT_THRESHOLD_DEFAULT,
    JointReport,
    check_joints,
    validate_audio_chunk,
    write_wav_from_chunks,
)


def _int16_bytes(samples: Iterable[int]) -> bytes:
    """Pack an iterable of int16 samples into little-endian bytes."""
    return struct.pack(f"<{len(list(samples))}h", *samples)


def _frame_line(*, frame_index: int, samples: List[int], sample_rate: int = 16000,
                layout: str = "little_endian", publish_t_s: float = 0.0,
                sample_width: int = 2, channels: int = 1) -> str:
    payload = _int16_bytes(samples)
    return json.dumps({
        "frame_index": frame_index,
        "publish_t_s": publish_t_s,
        "sample_rate": sample_rate,
        "channels": channels,
        "sample_width": sample_width,
        "layout": layout,
        "data_b64": base64.b64encode(payload).decode("ascii"),
    })


# ---------------------------------------------------------------------------
# validate_audio_chunk
# ---------------------------------------------------------------------------


class ValidateAudioChunkTests(unittest.TestCase):
    def test_int16_le_mono_payload_is_valid(self) -> None:
        samples = [0, 100, -100, 32000, -32000, 0]
        ok, violation, peak = validate_audio_chunk(
            _int16_bytes(samples),
            expected_sample_rate=16000,
            expected_channels=1,
            expected_sample_width=2,
            layout="little_endian",
            frame_index=0,
        )
        self.assertTrue(ok, violation)
        self.assertIsNone(violation)
        self.assertEqual(peak, 32000)

    def test_big_endian_layout_is_rejected(self) -> None:
        samples = [0, 100, -100, 32000, -32000]
        ok, violation, _ = validate_audio_chunk(
            struct.pack(f">{len(samples)}h", *samples),
            expected_sample_rate=16000,
            layout="big_endian",
            frame_index=1,
        )
        self.assertFalse(ok)
        assert violation is not None
        self.assertEqual(violation.layout, "big_endian")
        self.assertIn("big_endian", violation.reason)

    def test_odd_byte_count_is_rejected(self) -> None:
        ok, violation, _ = validate_audio_chunk(
            b"\x00\x01\x02",  # 3 bytes — not int16 aligned
            expected_sample_rate=16000,
            frame_index=2,
        )
        self.assertFalse(ok)
        assert violation is not None
        self.assertIn("not aligned", violation.reason)

    def test_empty_payload_is_rejected(self) -> None:
        ok, violation, _ = validate_audio_chunk(
            b"",
            expected_sample_rate=16000,
            frame_index=3,
        )
        self.assertFalse(ok)
        assert violation is not None
        self.assertEqual(violation.reason, "empty payload")

    def test_unknown_layout_is_rejected(self) -> None:
        ok, violation, _ = validate_audio_chunk(
            _int16_bytes([0, 1]),
            expected_sample_rate=16000,
            layout="midi-junk",
            frame_index=4,
        )
        self.assertFalse(ok)
        assert violation is not None
        self.assertIn("unsupported layout", violation.reason)

    def test_iec60958_layout_is_accepted(self) -> None:
        ok, violation, peak = validate_audio_chunk(
            _int16_bytes([0, 32767, -32768, 0]),
            expected_sample_rate=16000,
            layout="iec60958",
            frame_index=5,
        )
        self.assertTrue(ok, violation)
        self.assertIsNone(violation)
        # 32768 == |int16 min| (-32768) is valid; the largest absolute
        # sample in the payload.
        self.assertEqual(peak, 32768)

    def test_peaks_exceeding_int16_are_rejected(self) -> None:
        # Pack a sample that's deliberately out of int16 range by using
        # the unsigned 32k range boundary (32767 == OK, anything bigger
        # values-wise would overflow the struct).
        ok, violation, peak = validate_audio_chunk(
            _int16_bytes([32000, 32000, 32000]),
            expected_sample_rate=16000,
            frame_index=6,
        )
        self.assertTrue(ok, violation)  # 32000 < 32767
        self.assertEqual(peak, 32000)


# ---------------------------------------------------------------------------
# check_joints
# ---------------------------------------------------------------------------


class CheckJointsTests(unittest.TestCase):
    def test_continuous_stream_has_zero_jumps(self) -> None:
        # A flat DC signal has zero adjacent-sample jumps.
        pcm = _int16_bytes([0] * 100)
        report = check_joints(pcm)
        self.assertTrue(report.ok)
        self.assertEqual(report.max_jump, 0)
        self.assertEqual(report.samples, 100)

    def test_small_sine_wave_below_threshold(self) -> None:
        # 100 Hz sine at amplitude 0.6 → max int16 ≈ 19660,
        # adjacent-sample jump ≈ 775 (below the 4000 threshold).
        import math
        sr = 16000
        f = 100
        amp = int(0.6 * 32767)
        n = 160
        samples = [
            int(amp * math.sin(2 * math.pi * f * i / sr))
            for i in range(n)
        ]
        report = check_joints(_int16_bytes(samples))
        self.assertTrue(report.ok, f"max_jump={report.max_jump}")
        self.assertLessEqual(report.max_jump, JOINT_THRESHOLD_DEFAULT)

    def test_chunk_boundary_click_is_detected(self) -> None:
        # First chunk ends on -30000, second starts on +30000 → Δ ≈ 60000.
        chunk1 = _int16_bytes([100] * 50 + [-30000])
        chunk2 = _int16_bytes([+30000] + [100] * 50)
        pcm = chunk1 + chunk2
        report = check_joints(pcm)
        self.assertFalse(report.ok)
        self.assertGreaterEqual(report.max_jump, 60000)

    def test_short_stream_is_ok(self) -> None:
        self.assertTrue(check_joints(_int16_bytes([0])).ok)
        self.assertTrue(check_joints(b"").ok)


# ---------------------------------------------------------------------------
# write_wav_from_chunks
# ---------------------------------------------------------------------------


class WriteWavFromChunksTests(unittest.TestCase):
    def test_round_trip_header_matches(self) -> None:
        chunks = [
            _int16_bytes([100, 200, 300]),
            _int16_bytes([400, 500, -100, 0]),
        ]
        path = self._wav_path()
        self._tmp_paths.append(path)
        header = write_wav_from_chunks(
            chunks, path,
            sample_rate=16000, channels=1, sample_width=2,
        )
        self.assertTrue(header.ok, header.reason)
        self.assertEqual(header.sample_rate, 16000)
        self.assertEqual(header.channels, 1)
        self.assertEqual(header.sample_width, 2)
        # frames * sample_width * channels == total bytes
        expected_frames = sum(len(c) // 2 for c in chunks)
        self.assertEqual(header.frames, expected_frames)

    def test_empty_chunks_yields_no_wav(self) -> None:
        path = self._wav_path()
        self._tmp_paths.append(path)
        header = write_wav_from_chunks([], path, sample_rate=16000)
        self.assertFalse(header.ok)
        self.assertEqual(header.reason, "no chunks captured")

    def test_blank_chunks_are_skipped(self) -> None:
        chunks = [_int16_bytes([]), _int16_bytes([100, 200])]
        path = self._wav_path()
        self._tmp_paths.append(path)
        header = write_wav_from_chunks(chunks, path, sample_rate=16000)
        self.assertTrue(header.ok, header.reason)
        # Only the second chunk contributes samples (the first is empty).
        self.assertEqual(header.frames, 2)
        # Total bytes from non-empty chunks: int16 2 samples → 4 bytes.
        self.assertEqual(sum(len(c) for c in chunks), 4)

    @staticmethod
    def _wav_path():
        from tempfile import NamedTemporaryFile
        # NamedTemporaryFile(delete=False) leaves the file on disk so
        # the WAV reader can re-open it. The test fixture (cleanup
        # path) is on the class itself.
        f = NamedTemporaryFile(suffix=".wav", delete=False)
        f.close()
        return Path(f.name)

    def setUp(self) -> None:
        from tempfile import TemporaryDirectory
        self._tmp = TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self._tmp_paths: List[Path] = []

    def tearDown(self) -> None:
        for p in self._tmp_paths:
            try:
                p.unlink()
            except OSError:
                pass


# ---------------------------------------------------------------------------
# End-to-end stdin transport via the public CLI
# ---------------------------------------------------------------------------


class CliStdinTransportTests(unittest.TestCase):
    """Drive ``audio_capture_harness.py`` as a black box through stdin-pipe."""

    def setUp(self) -> None:
        from tempfile import TemporaryDirectory
        self._tmp = TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.out_wav = Path(self._tmp.name) / "cap.wav"
        self.out_json = Path(self._tmp.name) / "cap.json"

    def _run(self, stdin_text: str) -> Dict[str, Any]:
        cmd = [
            sys.executable,
            "-m",
            "tools.audio_capture_harness.audio_capture_harness",
            "--transport", "stdin",
            "--wav-out", str(self.out_wav),
            "--report-out", str(self.out_json),
            "--expected-sample-rate", "16000",
            "--deadline-s", "5",
        ]
        # Pass the worktree path explicitly via PYTHONPATH so the
        # subprocess can find ``tools.audio_capture_harness`` even if
        # pytest's cwd manipulation strips the trailing worktree path.
        env = os.environ.copy()
        py_path = str(PROJECT_ROOT)
        if env.get("PYTHONPATH"):
            env["PYTHONPATH"] = f"{py_path}:{env['PYTHONPATH']}"
        else:
            env["PYTHONPATH"] = py_path
        proc = subprocess.run(
            cmd,
            cwd=str(PROJECT_ROOT),
            env=env,
            input=stdin_text,
            capture_output=True,
            text=True,
            timeout=20,
        )
        # The harness prints the report on stdout. Fall back to the
        # report file if stdout is empty (e.g. the harness crashed
        # before printing).
        last_json_line = ""
        for line in proc.stdout.strip().splitlines()[::-1]:
            line = line.strip()
            if line.startswith("{") and line.endswith("}"):
                last_json_line = line
                break
        if not last_json_line and self.out_json.exists():
            last_json_line = self.out_json.read_text().strip().splitlines()[-1]
        try:
            parsed = json.loads(last_json_line) if last_json_line else {}
        except json.JSONDecodeError:
            parsed = {}
        if not parsed:
            parsed = {"ok": False, "stdout": proc.stdout, "stderr": proc.stderr}
        parsed["_exit_code"] = proc.returncode
        parsed["_stderr_tail"] = "\n".join(proc.stderr.strip().splitlines()[-5:])
        parsed["_stdout_tail"] = "\n".join(proc.stdout.strip().splitlines()[-3:])
        return parsed

    def test_continuous_two_chunk_stream_passes(self) -> None:
        # Two 0.5 s chunks at 16 kHz, mono, int16 LE. The fixture is
        # a flat-zero DC stream — no chunk-boundary click.
        chunk1 = _int16_bytes([0] * 8000)
        chunk2 = _int16_bytes([0] * 8000)
        stdin_text = "\n".join([
            _frame_line(frame_index=0, samples=[0] * 8000, publish_t_s=0.10),
            _frame_line(frame_index=1, samples=[0] * 8000, publish_t_s=0.60),
            "",
        ])
        report = self._run(stdin_text)
        self.assertEqual(report.get("_exit_code"), 0, report)
        self.assertTrue(report.get("ok"), report)
        self.assertEqual(report["frames"], 2)
        self.assertEqual(report["valid_chunks"], 2)
        self.assertEqual(report["samples"], 16000)
        self.assertAlmostEqual(report["duration_s"], 1.0, places=3)
        self.assertEqual(report["ttfa_s"], 0.10)
        self.assertTrue(report["joints"]["ok"])
        self.assertTrue(report["header_ok"])
        # WAV exists and is openable.
        with wave.open(str(self.out_wav), "rb") as r:
            self.assertEqual(r.getframerate(), 16000)
            self.assertEqual(r.getnchannels(), 1)
            self.assertEqual(r.getsampwidth(), 2)
            self.assertEqual(r.getnframes(), 16000)

    def test_duration_window_passes(self) -> None:
        # 1.0 s of audio, expected 1.0 ± 0.05 → in-range.
        stdin_text = "\n".join([
            _frame_line(frame_index=0, samples=[0] * 16000, publish_t_s=0.10),
            "",
        ])
        cmd = [
            sys.executable,
            "-m",
            "tools.audio_capture_harness.audio_capture_harness",
            "--transport", "stdin",
            "--wav-out", str(self.out_wav),
            "--report-out", str(self.out_json),
            "--expected-sample-rate", "16000",
            "--expected-duration-s", "1.0",
            "--expected-duration-tol-s", "0.05",
            "--deadline-s", "5",
        ]
        env = os.environ.copy()
        if env.get("PYTHONPATH"):
            env["PYTHONPATH"] = f"{PROJECT_ROOT}:{env['PYTHONPATH']}"
        else:
            env["PYTHONPATH"] = str(PROJECT_ROOT)
        proc = subprocess.run(
            cmd, cwd=str(PROJECT_ROOT), env=env,
            input=stdin_text, capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = json.loads(self.out_json.read_text())
        self.assertTrue(report["ok"], report)
        self.assertTrue(report["duration_ok"])
        self.assertAlmostEqual(report["duration_s"], 1.0, places=3)

    def test_duration_window_out_of_range_fails(self) -> None:
        # 1.0 s of audio, expected 0.5 ± 0.05 → out-of-range, exit 1.
        stdin_text = "\n".join([
            _frame_line(frame_index=0, samples=[0] * 16000, publish_t_s=0.10),
            "",
        ])
        cmd = [
            sys.executable,
            "-m",
            "tools.audio_capture_harness.audio_capture_harness",
            "--transport", "stdin",
            "--wav-out", str(self.out_wav),
            "--report-out", str(self.out_json),
            "--expected-sample-rate", "16000",
            "--expected-duration-s", "0.5",
            "--expected-duration-tol-s", "0.05",
            "--deadline-s", "5",
        ]
        env = os.environ.copy()
        if env.get("PYTHONPATH"):
            env["PYTHONPATH"] = f"{PROJECT_ROOT}:{env['PYTHONPATH']}"
        else:
            env["PYTHONPATH"] = str(PROJECT_ROOT)
        proc = subprocess.run(
            cmd, cwd=str(PROJECT_ROOT), env=env,
            input=stdin_text, capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(proc.returncode, 1)
        report = json.loads(self.out_json.read_text())
        self.assertFalse(report["ok"])
        self.assertFalse(report["duration_ok"])

    def test_two_chunks_with_click_fail_joints(self) -> None:
        # First chunk ends on +32767, second starts on -32768 → Δ = 65535.
        chunk1_samples = [100] * 50 + [32767]
        chunk2_samples = [-32768] + [100] * 50
        stdin_text = "\n".join([
            _frame_line(frame_index=0, samples=chunk1_samples, publish_t_s=0.1),
            _frame_line(frame_index=1, samples=chunk2_samples, publish_t_s=0.2),
            "",
        ])
        report = self._run(stdin_text)
        self.assertEqual(report.get("_exit_code"), 1, report)
        self.assertFalse(report.get("ok"), report)
        self.assertFalse(report["joints"]["ok"])
        self.assertGreaterEqual(report["joints"]["max_jump"], 60000)

    def test_big_endian_layout_chunk_is_rejected(self) -> None:
        # We manufacture a big-endian payload and pipe it through the
        # stdin transport — the harness should record a format
        # violation and exit non-zero.
        payload = struct.pack(">4h", 0, 100, -100, 0)
        b64 = base64.b64encode(payload).decode("ascii")
        bad_line = json.dumps({
            "frame_index": 0,
            "publish_t_s": 0.05,
            "sample_rate": 16000,
            "channels": 1,
            "sample_width": 2,
            "layout": "big_endian",
            "data_b64": b64,
        })
        stdin_text = "\n".join([bad_line, _frame_line(frame_index=1, samples=[0] * 100, publish_t_s=0.1), ""])
        report = self._run(stdin_text)
        self.assertEqual(report.get("_exit_code"), 1, report)
        self.assertFalse(report.get("ok"))
        self.assertEqual(len(report["format_violations"]), 1)

    def test_ttfa_is_recorded_from_publish_t_s(self) -> None:
        # Publisher reports its first frame at publish_t_s=0.42 → TTFA = 0.42.
        stdin_text = "\n".join([
            _frame_line(frame_index=0, samples=[0] * 100, publish_t_s=0.42),
            _frame_line(frame_index=1, samples=[0] * 100, publish_t_s=0.50),
            "",
        ])
        report = self._run(stdin_text)
        self.assertEqual(report["ttfa_s"], 0.42)
        self.assertEqual(report["first_frame_at_s"], 0.42)
        self.assertEqual(report["last_frame_at_s"], 0.50)

    def test_command_log_is_present(self) -> None:
        stdin_text = "\n".join([
            _frame_line(frame_index=0, samples=[0] * 100, publish_t_s=0.1),
            "",
        ])
        report = self._run(stdin_text)
        self.assertIn("commands", report)
        self.assertIsInstance(report["commands"], list)
        # In stdin mode the harness does NOT spawn any subprocess, so
        # the command log is empty — but the field is reserved for
        # ros2 mode where every launched subprocess is logged.
        self.assertEqual(report["commands"], [])

    def test_heartbeat_lines_are_skipped(self) -> None:
        stdin_text = "\n".join([
            json.dumps({"event": "end_of_stream"}),
            json.dumps({"event": "heartbeat"}),
            _frame_line(frame_index=0, samples=[0] * 100, publish_t_s=0.1),
            "",
        ])
        report = self._run(stdin_text)
        self.assertEqual(report.get("_exit_code"), 0, report)
        self.assertEqual(report["frames"], 1)


# ---------------------------------------------------------------------------
# End-to-end ros2 transport: stubbed
# ---------------------------------------------------------------------------


class Ros2TransportSmokeTests(unittest.TestCase):
    """Check that the ros2 transport returns RuntimeError when rclpy is unavailable.

    On hosts without ROS2 (Debian trixie without the jammy .deb ABI match),
    importing rclpy fails. The harness must surface this as a RuntimeError
    carrying the underlying cause, with an exit code that lets operators
    distinguish 'rclpy missing' from 'tts_node crashed' from 'capture
    rejected'.
    """

    def test_ros2_transport_raises_when_rclpy_missing(self) -> None:
        from tools.audio_capture_harness import audio_capture_harness as mod
        # Force-stub rclpy out of sys.modules so the harness cannot import it.
        saved = sys.modules.get("rclpy")
        sys.modules["rclpy"] = None  # type: ignore[assignment]
        try:
            class _Args:
                transport = "ros2"
                tts_node_cmd = None
                spawn_warmup_s = 0.0
                publish_after_s = 0.0
                publish_topic = None
                publish_payload = None
            args = _Args()
            import logging
            with self.assertRaises(RuntimeError) as ctx:
                mod.run_ros2_transport(args, logging.getLogger("test"))
            self.assertIn("rclpy", str(ctx.exception).lower())
        finally:
            if saved is not None:
                sys.modules["rclpy"] = saved
            else:
                sys.modules.pop("rclpy", None)


if __name__ == "__main__":
    unittest.main(verbosity=2)


# Acceptance: end-to-end through the production TTSNode._publish_audio
# (run via tools/audio_capture_harness/acceptance.py). See that file's
# docstring for the gates it asserts and the environment notes.


if __name__ == "__main__":
    unittest.main(verbosity=2)
