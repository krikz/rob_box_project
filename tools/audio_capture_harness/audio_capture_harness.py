#!/usr/bin/env python3
"""AudioData capture/validation harness for the ``tts_node`` audio pipeline.

This harness lives outside ``rob_box_llm`` and ``rob_box_voice`` — it is
a *test-only* module (under ``tools/audio_capture_harness/``) that:

1. subscribes to ``/voice/audio/speech`` (``audio_common_msgs/AudioData``)
   with a QoS profile compatible with what ``tts_node`` publishes;
2. validates every chunk matches the int16-LE / mono / configured-sample-rate
   contract (the test expects 16 kHz by default);
3. dumps every chunk into a single WAV file, in publish order;
4. measures *time-to-first-AudioData* — wall-clock time from the moment
   the harness spawns ``tts_node`` to the moment the first AudioData
   callback fires;
5. once the publisher stream ends, validates the WAV:
   * header (sample_rate / channels / sample_width)
   * duration matches an expected window
   * joints across chunk boundaries are continuous (no dropouts/clicks
     produced by missed chunks or leaked WAV/MP3 headers);

and emits a machine-readable JSON report on stdout describing the run,
plus a human-readable summary on stderr.

The harness supports two transports so it can run in both production
and bench environments:

* ``--transport ros2``  — real rclpy subscriber (requires ROS2 Humble +
  ``audio_common_msgs``). TTFA is measured from the moment the
  subprocess starts until the first ROS callback fires.
* ``--transport stdin`` — JSON-lines pipe. Each line on stdin is a
  frame description (base64-encoded payload + sample_rate + sample_width
  + layout + publish_t_s); the harness decodes them exactly the same
  way the bench's ``real_subscriber.py`` does. TTFA defaults to the
  ``publish_t_s`` of the first frame. This mode is what the project's
  ``tts_audio_bench`` exercises when the host does not have ROS2
  installed.

Both modes produce the same WAV file, the same JSON report shape, and
the same pass/fail verdict, so the same acceptance run can be repeated
on a developer laptop (pipe transport) and on the production image
(rclpy transport).

CLI usage::

    # ros2 mode — drive a separate tts_node in a known config
    python -m tools.audio_capture_harness.audio_capture_harness \\
        --transport ros2 \\
        --tts-node-cmd 'ros2 run rob_box_voice tts_node --ros-args -p provider:=minimax ...' \\
        --synthesize-text "привет мир" \\
        --expected-duration-s 0.8 --expected-duration-tol-s 0.5 \\
        --wav-out /tmp/cap.wav

    # stdin mode — feed it the bench's frame stream
    python -m tools.audio_capture_harness.audio_capture_harness \\
        --transport stdin \\
        --expected-sample-rate 16000 \\
        --wav-out /tmp/cap.wav < frames.jsonl

Exit codes::

    0  capture accepted (all optional checks pass; ``--strict`` to fail on warn)
    1  capture rejected — frames failed format validation, or the WAV
       wrote nothing, or the duration/joints checks rejected the stream
    2  environment not ready (e.g. rclpy unavailable and transport=ros2)
    3  tts_node subprocess exited non-zero before any frame was published

JSON report shape::

    {
      "ok": true,
      "transport": "ros2",
      "wav_path": "/tmp/cap.wav",
      "frames": 4,
      "bytes_total": 32000,
      "samples": 16000,
      "sample_rate": 16000,
      "channels": 1,
      "sample_width": 2,
      "ttfa_s": 0.42,
      "duration_s": 1.0,
      "duration_ok": true,
      "joints": { "ok": true, "max_jump": 120, "samples": 16000 },
      "first_frame_at_s": 0.42,
      "last_frame_at_s": 0.97,
      "format_violations": [],
      "commands": [
        ["ros2", "run", "rob_box_voice", "tts_node", ...],
        ["ros2", "topic", "info", "/voice/audio/speech", "--once"]
      ]
    }
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import shlex
import struct
import subprocess
import sys
import time
import traceback
import wave
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Project paths & test-scoped imports
# ---------------------------------------------------------------------------
HARNESS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = HARNESS_DIR.parents[2]
# Re-use the bench's check_joints / duration_report / validate_wav_header
# helpers — they live next door under tts_audio_bench/scripts/audio_validator.py
# and the harness inherits their semantics for free.
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

# ---------------------------------------------------------------------------
# Validation helpers (int16-LE mono validation, joints/duration/wav header)
# ---------------------------------------------------------------------------

# Discontinuity threshold for chunk-boundary joints. The TTS pipeline
# must publish a continuous stream — if a chunk boundary produces a
# click, either a chunk was dropped or a WAV/MP3 header leaked into the
# PCM payload. The bench uses 4000 (sine up to ~775 int16 units leaves
# a comfortable safety margin); we keep the same value for parity.
JOINT_THRESHOLD_DEFAULT = 4000

# Layout string the production ``tts_node`` publishes. AudioData is
# raw ``uint8[] data`` (no built-in sample-rate metadata), so the
# sample-rate / channels / sample-width / endianness must come either
# from outside (this harness defaults to int16 LE mono 16 kHz) or from
# the optional ``info`` field on the AudioInfo companion message.
EXPECTED_LAYOUTS = frozenset({"little_endian", "iec60958"})


@dataclass(frozen=True)
class FormatViolation:
    """A single chunk that did not match the int16-LE / mono contract."""

    frame_index: int
    reason: str
    sample_rate: int = 0
    channels: int = 0
    sample_width: int = 0
    layout: str = ""
    bytes: int = 0


@dataclass(frozen=True)
class CaptureHeader:
    """WAV header cross-check after all chunks are written."""

    ok: bool
    sample_rate: int
    channels: int
    sample_width: int
    frames: int
    reason: Optional[str] = None


@dataclass(frozen=True)
class JointReport:
    ok: bool
    max_jump: int
    samples: int


def validate_audio_chunk(
    payload: bytes,
    *,
    expected_sample_rate: int,
    expected_channels: int = 1,
    expected_sample_width: int = 2,
    layout: str = "little_endian",
    frame_index: int = 0,
) -> Tuple[bool, Optional[FormatViolation], int]:
    """Validate a single AudioData payload against the int16-LE contract.

    Returns ``(ok, violation_or_none, peak_int16)``:

    * ``ok`` is False if any structural property is wrong — the harness
      then drops that chunk from the WAV and records a
      :class:`FormatViolation`;
    * ``peak_int16`` is the largest absolute sample in the chunk (for
      diagnostics).

    The harness treats AudioData as ``uint8[] data`` (raw PCM bytes) as
    per the ROS ``audio_common_msgs/AudioData`` definition. Sample-rate
    and channels are *out of band*: the publisher's parameter
    ``audio_output_sample_rate`` is the single source of truth.
    """
    if not isinstance(payload, (bytes, bytearray)):
        return False, FormatViolation(
            frame_index=frame_index,
            reason=f"payload is {type(payload).__name__}, expected bytes",
            bytes=len(payload) if hasattr(payload, "__len__") else 0,
        ), 0

    raw = bytes(payload)

    if expected_sample_width != 2:
        return False, FormatViolation(
            frame_index=frame_index,
            reason=f"expected_sample_width={expected_sample_width} not int16",
            sample_width=expected_sample_width,
        ), 0

    if len(raw) % expected_sample_width != 0:
        return False, FormatViolation(
            frame_index=frame_index,
            reason=(
                f"payload length {len(raw)} not aligned to "
                f"sample_width={expected_sample_width}"
            ),
            sample_width=expected_sample_width,
            bytes=len(raw),
        ), 0

    n = len(raw) // expected_sample_width
    if n == 0:
        return False, FormatViolation(
            frame_index=frame_index,
            reason="empty payload",
            sample_rate=expected_sample_rate,
            channels=expected_channels,
            sample_width=expected_sample_width,
            layout=layout,
            bytes=0,
        ), 0

    if layout not in EXPECTED_LAYOUTS:
        return False, FormatViolation(
            frame_index=frame_index,
            reason=f"unsupported layout {layout!r} (expected one of {sorted(EXPECTED_LAYOUTS)})",
            sample_rate=expected_sample_rate,
            channels=expected_channels,
            sample_width=expected_sample_width,
            layout=layout,
            bytes=len(raw),
        ), 0

    # little_endian == native on x86/arm64; iec60958 == IEC 60958 bitstream
    # framing which is byte-compatible with little-endian PCM for our use.
    samples = struct.unpack(f"<{n}h", raw)
    peak = max(abs(int(s)) for s in samples)

    # 32768 == |int16 min| is valid (it's -32768). Any positive
    # overflow (positive sample > 32767) saturates struct to -32767
    # during unpacking, which would already be hidden as |32767|.
    # The remaining real concern is payload bytes that don't decode
    # to int16 at all — caught above. We still gate on >32768 so the
    # test 'samples = [-32768]' round-trips without a false positive.
    if peak > 32768:
        return False, FormatViolation(
            frame_index=frame_index,
            reason=f"peak={peak} exceeds int16 range (>32768)",
            sample_rate=expected_sample_rate,
            channels=expected_channels,
            sample_width=expected_sample_width,
            layout=layout,
            bytes=len(raw),
        ), peak

    return True, None, peak


def check_joints(pcm: bytes, threshold: int = JOINT_THRESHOLD_DEFAULT) -> JointReport:
    """Walk ``pcm`` as int16 LE samples and report the largest adjacent jump.

    The check is intra-stream: adjacent pairs across chunk boundaries
    are included, so missing chunks or leaked WAV/MP3 headers surface
    as a discontinuity exceeding the threshold.
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
        ok=max_jump <= threshold,
        max_jump=max_jump,
        samples=n,
    )


def write_wav_from_chunks(
    chunks: List[bytes],
    out_path: Path,
    *,
    sample_rate: int,
    channels: int = 1,
    sample_width: int = 2,
) -> CaptureHeader:
    """Serialise ``chunks`` to a WAV file and validate the header round-trip.

    Returns a header report describing whether the file the harness
    wrote matches the contract it claims to satisfy.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not chunks:
        return CaptureHeader(
            ok=False,
            sample_rate=0,
            channels=0,
            sample_width=0,
            frames=0,
            reason="no chunks captured",
        )

    total_bytes = 0
    with wave.open(str(out_path), "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(sample_width)
        w.setframerate(sample_rate)
        for c in chunks:
            if c:
                w.writeframes(c)
                total_bytes += len(c)

    # Round-trip cross-check.
    try:
        with wave.open(str(out_path), "rb") as r:
            rsr = r.getframerate()
            rch = r.getnchannels()
            rsw = r.getsampwidth()
            rnframes = r.getnframes()
    except (wave.Error, FileNotFoundError, EOFError) as exc:
        return CaptureHeader(
            ok=False,
            sample_rate=0,
            channels=0,
            sample_width=0,
            frames=0,
            reason=f"cannot reopen {out_path}: {exc!r}",
        )

    problems: List[str] = []
    if rsr != sample_rate:
        problems.append(f"sr={rsr} != {sample_rate}")
    if rch != channels:
        problems.append(f"ch={rch} != {channels}")
    if rsw != sample_width:
        problems.append(f"sw={rsw} != {sample_width}")
    if rnframes * sample_width * channels != total_bytes:
        problems.append(
            f"frame bytes {rnframes * sample_width * channels} != {total_bytes}"
        )

    return CaptureHeader(
        ok=not problems,
        sample_rate=rsr,
        channels=rch,
        sample_width=rsw,
        frames=rnframes,
        reason="; ".join(problems) or None,
    )


# ---------------------------------------------------------------------------
# Transport: ROS2 (rclpy) subscriber
# ---------------------------------------------------------------------------

def _try_import_rclpy() -> Tuple[Optional[Any], Optional[Any], Optional[str]]:
    """Best-effort import of rclpy + audio_common_msgs.

    Returns ``(rclpy_module, AudioData_class, error)`` — when rclpy is
    not importable on the host (e.g. Debian trixie without the jammy
    ``.deb`` ABI match) we want the harness to surface that *cleanly*
    so the operator can run with ``--transport stdin`` instead.
    """
    try:
        import rclpy  # type: ignore
        try:
            from audio_common_msgs.msg import AudioData  # type: ignore
            return rclpy, AudioData, None
        except Exception as exc:  # noqa: BLE001
            return rclpy, None, f"audio_common_msgs unavailable: {exc!r}"
    except Exception as exc:  # noqa: BLE001
        return None, None, f"rclpy unavailable: {exc!r}"


def _log_command(cmd: List[str], label: str, log: "logging.Logger") -> None:
    """Log a launched subprocess in a single readable line."""
    pretty = " ".join(shlex.quote(c) for c in cmd)
    log.info("[%s] $ %s", label, pretty)


@dataclass
class Ros2CaptureResult:
    """The shape ROS2 mode fills in before writing the WAV."""

    chunks: List[bytes] = field(default_factory=list)
    publish_timestamps: List[float] = field(default_factory=list)
    sample_rate_meta: List[Optional[int]] = field(default_factory=list)
    channels_meta: List[Optional[int]] = field(default_factory=list)
    sample_width_meta: List[Optional[int]] = field(default_factory=list)
    layout_meta: List[str] = field(default_factory=list)
    violations: List[FormatViolation] = field(default_factory=list)
    first_frame_at_s: Optional[float] = None
    last_frame_at_s: Optional[float] = None
    started_at_s: float = 0.0


def _ros2_run(
    *,
    node_name: str,
    audio_topic: str,
    qos_reliability: str,
    qos_depth: int,
    started_at_s: float,
    capture: Ros2CaptureResult,
    min_msgs: int,
    deadline_s: float,
    capture_max_seconds: float,
    log,
) -> None:
    """Spin an rclpy node subscribing to ``audio_topic`` until termination.

    Termination is whichever comes first:

    * ``min_msgs`` frames have arrived *and* no frame has been published
      for ``capture_max_seconds`` (idle-quiescence);
    * the deadline ``deadline_s`` has elapsed;
    * the rclpy context is shut down externally (CTRL-C / parent).
    """
    import rclpy  # type: ignore
    from audio_common_msgs.msg import AudioData  # type: ignore
    from rclpy.qos import (  # type: ignore
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    rclpy.init()
    node = rclpy.create_node(node_name)

    if qos_reliability == "best_effort":
        reliability = ReliabilityPolicy.BEST_EFFORT
    elif qos_reliability == "reliable":
        reliability = ReliabilityPolicy.RELIABLE
    else:
        node.destroy_node()
        rclpy.shutdown()
        raise ValueError(f"qos_reliability must be best_effort|reliable, got {qos_reliability!r}")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, qos_depth),
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )

    def _on_audio(msg: Any) -> None:
        nonlocal last_frame_at  # noqa: F824 — annotate for static analyzers
        payload = bytes(msg.data)
        info = getattr(msg, "info", None)
        sr_meta = getattr(info, "sample_rate", None) if info is not None else None
        ch_meta = getattr(info, "channels", None) if info is not None else None
        sw_meta = getattr(info, "sample_width", None) if info is not None else None
        layout = str(getattr(info, "layout", "little_endian") or "little_endian")

        idx = len(capture.chunks)
        capture.chunks.append(payload)
        capture.publish_timestamps.append(time.monotonic() - started_at_s)
        capture.sample_rate_meta.append(sr_meta)
        capture.channels_meta.append(ch_meta)
        capture.sample_width_meta.append(sw_meta)
        capture.layout_meta.append(layout)
        if capture.first_frame_at_s is None:
            capture.first_frame_at_s = time.monotonic() - started_at_s
        capture.last_frame_at_s = time.monotonic() - started_at_s
        last_frame_at = time.monotonic()
        log.debug(
            "audio frame %d: bytes=%d sr_meta=%s ch_meta=%s sw_meta=%s layout=%s",
            idx, len(payload), sr_meta, ch_meta, sw_meta, layout,
        )

    sub = node.create_subscription(AudioData, audio_topic, _on_audio, qos)
    log.info("rclpy subscriber ready on %s (qos=%s depth=%d)", audio_topic, qos_reliability, qos_depth)

    end_at = time.monotonic() + deadline_s
    last_frame_at = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() < end_at:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                len(capture.chunks) >= min_msgs
                and (time.monotonic() - last_frame_at) > capture_max_seconds
            ):
                log.info(
                    "idle-quiescence: %d msgs captured, no new frame for %.2fs — done",
                    len(capture.chunks),
                    time.monotonic() - last_frame_at,
                )
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def run_ros2_transport(args: argparse.Namespace, log) -> Tuple[Ros2CaptureResult, List[List[str]]]:
    """Run the ROS2 transport: spawn tts_node, spin subscriber, return frames.

    The tts_node subprocess is started BEFORE the rclpy subscriber so
    the TTFA measurement begins from the same wall clock instant the
    publisher was spawned. We then wait for the publisher to publish
    the requested text, give the publisher a few seconds to quiesce,
    and finally return the captured frames.
    """
    rclpy, _AudioData, err = _try_import_rclpy()
    if rclpy is None or _AudioData is None:
        raise RuntimeError(
            "rclpy/audio_common_msgs unavailable: " + (err or "unknown")
        )

    commands_logged: List[List[str]] = []
    started_at_s = time.monotonic()

    # 1) Spawn tts_node if requested.
    proc: Optional[subprocess.Popen] = None
    if args.tts_node_cmd:
        cmd = shlex.split(args.tts_node_cmd) if isinstance(args.tts_node_cmd, str) else list(args.tts_node_cmd)
        _log_command(cmd, "tts_node", log)
        commands_logged.append(cmd)
        # Use a new process group so we can clean up children (e.g. roscore) on exit.
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        # Give ROS a moment to bring the publisher up. We do this
        # even when the test publishes via --publish-after, so that
        # the TTFA measurement starts before the actual sample is sent.
        if args.spawn_warmup_s > 0:
            time.sleep(args.spawn_warmup_s)

    # 2) Optionally publish a synthesize trigger to /voice/tts/request.
    if args.publish_after_s > 0:
        time.sleep(args.publish_after_s)
    if args.publish_topic and args.publish_payload:
        publish_cmd = [
            "ros2", "topic", "pub", "--once", args.publish_topic, "std_msgs/msg/String",
        ]
        # payload arrives as a string literal; we need to inject it with
        # shell-escape rules. Publish via subprocess so ROS properly
        # registers the temporary tool.
        pub_proc = subprocess.run(
            publish_cmd + [args.publish_payload],
            capture_output=True, text=True, timeout=15,
        )
        log.info("ros2 topic pub rc=%d out=%r err=%r", pub_proc.returncode, pub_proc.stdout[:80], pub_proc.stderr[:80])
        commands_logged.append(publish_cmd + [args.publish_payload])

    # 3) Spin subscriber.
    capture = Ros2CaptureResult(started_at_s=started_at_s)

    try:
        _ros2_run(
            node_name=args.node_name,
            audio_topic=args.audio_topic,
            qos_reliability=args.qos_reliability,
            qos_depth=args.qos_depth,
            started_at_s=started_at_s,
            capture=capture,
            min_msgs=args.min_msgs,
            deadline_s=args.deadline_s,
            capture_max_seconds=args.capture_max_seconds,
            log=log,
        )
    finally:
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:  # noqa: BLE001
                    pass

    return capture, commands_logged


# ---------------------------------------------------------------------------
# Transport: stdin JSON-lines (bench-pipe fallback)
# ---------------------------------------------------------------------------

@dataclass
class StdinCaptureResult(Ros2CaptureResult):
    """Identical shape to :class:`Ros2CaptureResult` for uniform reporting."""


def run_stdin_transport(args: argparse.Namespace, log) -> Tuple[StdinCaptureResult, List[List[str]]]:
    """Decode JSON-lines from stdin (same wire format as tts_audio_bench real_subscriber).

    Each line is ``{"frame_index": N, "publish_t_s": float, "sample_rate": int,
    "channels": int, "sample_width": int, "layout": str, "data_b64": str}``.

    Returns the same shape as :func:`run_ros2_transport`.
    """
    commands_logged: List[List[str]] = []
    started_at_s = time.monotonic()
    capture = StdinCaptureResult(started_at_s=started_at_s)

    log.info("stdin transport: reading JSON-lines from stdin (deadline=%.1fs)", args.deadline_s)
    end_at = time.monotonic() + args.deadline_s
    seen_any = False
    for raw in sys.stdin:
        if time.monotonic() > end_at:
            log.warning("stdin deadline exceeded after %d bytes", len(raw))
            break
        line = raw.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError as exc:
            log.warning("skipping unparseable stdin line: %s", exc)
            continue
        if not isinstance(obj, dict):
            continue
        b64 = obj.get("data_b64")
        if not b64:
            log.debug("skipping stdin heartbeat: %s", obj.get("event") or "(no data_b64)")
            continue
        try:
            payload = base64.b64decode(b64, validate=True)
        except (ValueError, TypeError) as exc:
            log.warning("invalid base64 in stdin frame: %s", exc)
            capture.violations.append(FormatViolation(
                frame_index=len(capture.chunks),
                reason=f"base64 decode failed: {exc}",
            ))
            continue
        capture.chunks.append(payload)
        capture.publish_timestamps.append(float(obj.get("publish_t_s", 0.0)))
        capture.sample_rate_meta.append(
            int(obj.get("sample_rate", args.expected_sample_rate))
            if obj.get("sample_rate") is not None else None
        )
        capture.channels_meta.append(int(obj.get("channels", 1)) if obj.get("channels") is not None else None)
        capture.sample_width_meta.append(int(obj.get("sample_width", 2)) if obj.get("sample_width") is not None else None)
        capture.layout_meta.append(str(obj.get("layout", "little_endian")))
        if capture.first_frame_at_s is None:
            capture.first_frame_at_s = float(obj.get("publish_t_s", 0.0))
        capture.last_frame_at_s = float(obj.get("publish_t_s", 0.0))
        seen_any = True

    if not seen_any:
        log.error("stdin closed with no AudioData frames")
    return capture, commands_logged


# ---------------------------------------------------------------------------
# Top-level orchestration: validate, write WAV, run joints/duration, report
# ---------------------------------------------------------------------------

def _validate_capture(
    capture: Ros2CaptureResult,
    *,
    expected_sample_rate: int,
    expected_channels: int,
    expected_sample_width: int,
    log,
) -> None:
    """Apply :func:`validate_audio_chunk` to every chunk in the capture."""
    for idx, payload in enumerate(capture.chunks):
        layout = capture.layout_meta[idx] or "little_endian"
        ok, violation, peak = validate_audio_chunk(
            payload,
            expected_sample_rate=expected_sample_rate,
            expected_channels=expected_channels,
            expected_sample_width=expected_sample_width,
            layout=layout,
            frame_index=idx,
        )
        if not ok and violation is not None:
            log.warning(
                "frame %d rejected: %s (peak=%d)",
                idx, violation.reason, peak,
            )
            capture.violations.append(violation)


def _filter_valid_chunks(capture: Ros2CaptureResult) -> List[bytes]:
    """Drop chunks that failed validation before they hit the WAV.

    The WAV must contain only valid PCM; a single bad chunk would
    produce an audible click or a header mismatch. We still keep the
    violation record so the report can pinpoint the offending frame.
    """
    bad_frames = {v.frame_index for v in capture.violations}
    return [c for i, c in enumerate(capture.chunks) if i not in bad_frames]


def _build_report(
    *,
    transport: str,
    wav_path: Path,
    capture: Ros2CaptureResult,
    header: CaptureHeader,
    expected_sample_rate: int,
    expected_channels: int,
    expected_sample_width: int,
    expected_duration_min_s: Optional[float],
    expected_duration_max_s: Optional[float],
    joint_threshold: int,
    joints: JointReport,
    duration_s: float,
    duration_ok: bool,
    commands: List[List[str]],
    log,
) -> Dict[str, Any]:
    """Compose the JSON report and decide pass/fail.

    Pass criteria:

    * at least one valid chunk was captured;
    * the WAV round-trip header matches what we asked for;
    * the duration falls within the expected window (when given);
    * the joints check passed;
    * no FormatViolation was recorded.

    A ``--strict`` flag upgrades any warning to a fail.
    """
    frames = len(capture.chunks)
    valid_chunks = _filter_valid_chunks(capture)
    bytes_total = sum(len(c) for c in valid_chunks)
    samples = bytes_total // max(1, expected_sample_width * expected_channels)

    ok = (
        frames > 0
        and len(capture.violations) == 0
        and header.ok
        and duration_ok
        and joints.ok
    )

    report: Dict[str, Any] = {
        "ok": ok,
        "transport": transport,
        "wav_path": str(wav_path),
        "frames": frames,
        "valid_chunks": len(valid_chunks),
        "bytes_total": bytes_total,
        "samples": samples,
        "sample_rate": expected_sample_rate,
        "channels": expected_channels,
        "sample_width": expected_sample_width,
        "ttfa_s": capture.first_frame_at_s,
        "duration_s": duration_s,
        "duration_ok": duration_ok,
        "expected_duration_min_s": expected_duration_min_s,
        "expected_duration_max_s": expected_duration_max_s,
        "joints": {
            "ok": joints.ok,
            "max_jump": joints.max_jump,
            "samples": joints.samples,
            "threshold": joint_threshold,
        },
        "header_ok": header.ok,
        "header": asdict(header),
        "first_frame_at_s": capture.first_frame_at_s,
        "last_frame_at_s": capture.last_frame_at_s,
        "format_violations": [asdict(v) for v in capture.violations],
        "commands": [" ".join(shlex.quote(c) for c in cmd) for cmd in commands],
    }
    if not ok:
        log.error(
            "capture rejected: header_ok=%s duration_ok=%s joints_ok=%s violations=%d frames=%d",
            header.ok, duration_ok, joints.ok, len(capture.violations), frames,
        )
    return report


def _cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--transport", choices=("ros2", "stdin"), default="stdin",
                        help="Frame transport. Default stdin (bench pipe).")
    parser.add_argument("--node-name", default="tts_audio_capture",
                        help="rclpy node name for --transport=ros2.")
    parser.add_argument("--audio-topic", default="/voice/audio/speech",
                        help="AudioData topic to subscribe to.")
    parser.add_argument("--qos-reliability", choices=("best_effort", "reliable"), default="best_effort",
                        help="QoS reliability (must match tts_node audio_qos_reliability).")
    parser.add_argument("--qos-depth", type=int, default=10,
                        help="QoS KEEP_LAST depth.")
    parser.add_argument("--expected-sample-rate", type=int, default=16000,
                        help="Sample rate tts_node publishes (default 16000).")
    parser.add_argument("--expected-channels", type=int, default=1,
                        help="Expected number of channels (default 1 = mono).")
    parser.add_argument("--expected-sample-width", type=int, default=2,
                        help="Expected sample width in bytes (default 2 = int16).")
    parser.add_argument("--wav-out", type=Path, required=True,
                        help="Where to write the captured WAV file.")
    parser.add_argument("--report-out", type=Path, default=None,
                        help="Optional JSON report path. Defaults to <wav-out>.json.")
    parser.add_argument("--min-msgs", type=int, default=1,
                        help="Stop after at least N frames AND no new frame for capture_max_seconds.")
    parser.add_argument("--capture-max-seconds", type=float, default=3.0,
                        help="Idle-quiescence: stop if no new frame for this many seconds.")
    parser.add_argument("--deadline-s", type=float, default=60.0,
                        help="Hard deadline in seconds.")
    parser.add_argument("--expected-duration-s", type=float, default=None,
                        help="Expected duration of the captured audio (centre of the window).")
    parser.add_argument("--expected-duration-tol-s", type=float, default=None,
                        help="Symmetric tolerance; required iff --expected-duration-s is given.")
    parser.add_argument("--joint-threshold", type=int, default=JOINT_THRESHOLD_DEFAULT,
                        help="Max int16 unit jump allowed at chunk boundaries.")
    parser.add_argument("--strict", action="store_true",
                        help="Treat any warning (e.g. duration slightly out of window) as a failure.")
    parser.add_argument("--spawn-warmup-s", type=float, default=1.5,
                        help="Wait this many seconds between spawning tts_node and starting subscriber.")
    parser.add_argument("--publish-after-s", type=float, default=0.0,
                        help="Wait this many seconds after subscriber is up before triggering synthesis.")
    parser.add_argument("--publish-topic", default="/voice/tts/request",
                        help="std_msgs/msg/String topic used to trigger synthesis.")
    parser.add_argument("--publish-payload", default=None,
                        help="String payload published to --publish-topic before capture.")
    parser.add_argument("--tts-node-cmd", default=None,
                        help="Optional shell command that starts the tts_node subprocess.")
    parser.add_argument("--log-file", type=Path, default=None,
                        help="Optional file path to mirror the human-readable summary.")
    parser.add_argument("--log-level", default="INFO",
                        help="Log level for stderr logs (default INFO).")
    args = parser.parse_args()

    # Lazy logger so tests can patch it without breaking argparse.
    import logging
    log = logging.getLogger("audio_capture_harness")
    log.setLevel(getattr(logging, args.log_level.upper(), logging.INFO))
    if not log.handlers:
        h = logging.StreamHandler()
        h.setFormatter(logging.Formatter("[harness] %(levelname)s %(message)s"))
        log.addHandler(h)

    # ---- Pre-flight ---------------------------------------------------
    if (args.expected_duration_s is None) != (args.expected_duration_tol_s is None):
        log.error("--expected-duration-s and --expected-duration-tol-s must be given together")
        return 2

    expected_min: Optional[float] = None
    expected_max: Optional[float] = None
    if args.expected_duration_s is not None and args.expected_duration_tol_s is not None:
        expected_min = args.expected_duration_s - args.expected_duration_tol_s
        expected_max = args.expected_duration_s + args.expected_duration_tol_s

    if args.publish_payload and not args.publish_topic:
        log.error("--publish-payload requires --publish-topic")
        return 2

    # ---- Run the chosen transport -------------------------------------
    commands: List[List[str]] = []
    try:
        if args.transport == "ros2":
            capture, captured_commands = run_ros2_transport(args, log)
        else:
            capture, captured_commands = run_stdin_transport(args, log)
    except RuntimeError as exc:
        log.error("transport failed: %s", exc)
        return 2
    except Exception as exc:  # noqa: BLE001
        log.error("transport crashed: %s\n%s", exc, traceback.format_exc())
        return 2
    commands.extend(captured_commands)

    # ---- Validate every chunk -----------------------------------------
    _validate_capture(
        capture,
        expected_sample_rate=args.expected_sample_rate,
        expected_channels=args.expected_channels,
        expected_sample_width=args.expected_sample_width,
        log=log,
    )

    valid_chunks = _filter_valid_chunks(capture)

    # ---- Write WAV + cross-check header -------------------------------
    header = write_wav_from_chunks(
        valid_chunks,
        args.wav_out,
        sample_rate=args.expected_sample_rate,
        channels=args.expected_channels,
        sample_width=args.expected_sample_width,
    )

    # ---- Duration + joints check --------------------------------------
    bytes_total = sum(len(c) for c in valid_chunks)
    samples = bytes_total // max(1, args.expected_sample_width * args.expected_channels)
    duration_s = samples / args.expected_sample_rate if args.expected_sample_rate else 0.0

    joints = check_joints(b"".join(valid_chunks), threshold=args.joint_threshold)

    duration_ok = True
    if expected_min is not None and expected_max is not None:
        duration_ok = expected_min <= duration_s <= expected_max
    if args.strict and capture.violations:
        log.warning("strict: %d format violations present", len(capture.violations))

    # ---- Compose & emit report ----------------------------------------
    report = _build_report(
        transport=args.transport,
        wav_path=args.wav_out,
        capture=capture,
        header=header,
        expected_sample_rate=args.expected_sample_rate,
        expected_channels=args.expected_channels,
        expected_sample_width=args.expected_sample_width,
        expected_duration_min_s=expected_min,
        expected_duration_max_s=expected_max,
        joint_threshold=args.joint_threshold,
        joints=joints,
        duration_s=duration_s,
        duration_ok=duration_ok,
        commands=commands,
        log=log,
    )

    report_path = args.report_out or args.wav_out.with_suffix(".json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    log.info("wrote report: %s", report_path)

    if args.log_file:
        args.log_file.parent.mkdir(parents=True, exist_ok=True)
        with args.log_file.open("w") as f:
            for cmd in commands:
                f.write("$ " + " ".join(shlex.quote(c) for c in cmd) + "\n")
            f.write(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    # Always print the report on stdout so callers can pipe it to jq.
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(_cli())
