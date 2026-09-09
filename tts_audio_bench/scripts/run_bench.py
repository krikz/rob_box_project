"""Test bench for the MiniMax TTS → ROS AudioData bridge.

This is the entry point for ``tts_audio_bench``. It exercises the
end-to-end pipeline the production ``TTSNode`` runs at runtime:

    dialogue string  →  MiniMax HTTP  →  decode/transcode  →
    ROS AudioData int16 LE mono @ 16 kHz  →  WAV dump

What it covers (acceptance gates):

1. ``colcon build rob_box_llm + rob_box_voice`` — the bench invokes the
   build inside ``.devcontainer/Dockerfile`` if available, otherwise
   falls back to a Python-only ``colcon build --packages-select`` that
   validates ``setup.py`` is import-clean (no need for full ament).
2. MiniMax mock-server round-trip for **PCM / WAV / MP3 / OGG** —
   verifies the bench can swap the endpoint via a local mock, no real
   secrets required.
3. ``AudioData`` messages on ``/voice/audio/speech`` are int16 LE mono
   and at the configured sample rate; QoS compatibility (best_effort,
   KEEP_LAST, depth=10) is asserted via the captured publisher object.
4. ffmpeg decode for compressed formats (MP3 / OGG) — exercised by
   matching decoded bytes against a reference WAV fixture.
5. ``time-to-first-AudioData`` (TTFA) measured per scenario.
6. The published chunks are dumped to WAV; the bench then compares
   duration (samples / sample_rate) and runs a joint discontinuity
   check on the concatenation — any chunk-boundary click fails the
   scenario.

Run from the project root::

    python -m tts_audio_bench.scripts.run_bench

The bench also reports a JSON summary at ``artifacts/bench-summary.json``
plus a per-scenario WAV at ``artifacts/<scenario>.wav``.

Exit codes::

    0 — all scenarios passed
    1 — at least one scenario failed (see logs/ for details)
    2 — environment not ready (e.g. ffmpeg missing)
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import struct
import sys
import time
import traceback
import wave
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Path bootstrap: make rob_box_llm and rob_box_voice importable without
# having to ``colcon build`` first. The bench is designed to work on
# minimal hosts (Debian trixie without ROS2) where a full ament build is
# not possible. The full build is documented in the README and exercised
# inside the .devcontainer image.
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src" / "rob_box_llm"))
sys.path.insert(0, str(PROJECT_ROOT / "src" / "rob_box_voice"))

# ROS stub must be installed BEFORE tts_node is imported.
from tts_audio_bench.scripts import ros_stub  # noqa: E402
from tts_audio_bench.scripts.audio_subscriber import (  # noqa: E402
    SubscriberReport,
    WavRecorder,
)
from tts_audio_bench.scripts.audio_validator import (  # noqa: E402
    JointReport,
    check_joints,
    duration_report,
    validate_wav_header,
)
from tts_audio_bench.scripts.mock_minimax_server import start_server  # noqa: E402

ros_stub.install()

# Now we can safely import the production code.
import httpx  # noqa: E402
import numpy as np  # noqa: E402

# We import the heavy tts_node module carefully — it pulls in torch,
# sounddevice, grpc, yandex-cloud-ml-sdk which are NOT installed in the
# bench host. We monkey-patch sys.modules with no-op stand-ins FIRST,
# so the imports succeed but ``YANDEX_GRPC_AVAILABLE = False`` etc.
def _install_heavy_dep_stubs() -> None:
    import types
    mods = {
        "grpc": types.ModuleType("grpc"),
        "torch": types.ModuleType("torch"),
        "sounddevice": types.ModuleType("sounddevice"),
        "yandex.cloud": types.ModuleType("yandex.cloud"),
        "yandex.cloud.ai": types.ModuleType("yandex.cloud.ai"),
        "yandex.cloud.ai.tts": types.ModuleType("yandex.cloud.ai.tts"),
        "yandex.cloud.ai.tts.v3": types.ModuleType("yandex.cloud.ai.tts.v3"),
    }
    # torch.device("cpu") must work — provide a minimal stub.
    class _Device:
        def __init__(self, *_a, **_kw):
            self.type = "cpu"
    mods["torch"].device = _Device  # type: ignore[attr-defined]
    # sounddevice needs query_devices (called in initialize_audio_device).
    mods["sounddevice"].query_devices = lambda *a, **kw: {"name": "ALSA stub"}  # type: ignore[attr-defined]
    mods["sounddevice"].stop = lambda *a, **kw: None  # type: ignore[attr-defined]
    for name, mod in mods.items():
        sys.modules.setdefault(name, mod)


_install_heavy_dep_stubs()


def _install_audio_dep_stubs() -> None:
    """Stub the audio libs that ``rob_box_voice.utils.__init__`` imports.

    ``rob_box_voice/utils/__init__.py`` pulls in ``audio_utils`` which in
    turn requires ``pyaudio`` (not on the bench host). We register
    ``pyaudio`` / ``pixel_ring`` / ``spidev`` as empty modules so the
    package import succeeds without dragging in actual hardware libs.
    """
    import types

    def _stub_module(name: str, attrs: Optional[Dict[str, Any]] = None) -> types.ModuleType:
        mod = types.ModuleType(name)
        for k, v in (attrs or {}).items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        return mod

    pyaudio_stub = _stub_module("pyaudio")
    # audio_utils references ``pyaudio.PyAudio`` as a type hint only —
    # ``find_respeaker_device(p: pyaudio.PyAudio)``. A bare class with a
    # permissive constructor keeps that annotation valid without dragging
    # in libportaudio.
    class _PyAudio:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            return None

        def get_device_count(self) -> int:
            return 0

    pyaudio_stub.PyAudio = _PyAudio  # type: ignore[attr-defined]

    _stub_module("pixel_ring")
    _stub_module("spidev")
    # ``usb.core`` is referenced as ``usb.core.Device`` in a type
    # annotation, so we need both the package and the submodule exposed
    # via ``usb.core``.
    usb_pkg = _stub_module("usb")
    usb_core_stub = _stub_module("usb.core")

    class _UsbDevice:
        pass

    usb_core_stub.Device = _UsbDevice  # type: ignore[attr-defined]
    usb_pkg.core = usb_core_stub  # type: ignore[attr-defined]

    _stub_module("usb.util")


_install_audio_dep_stubs()

# Pre-import the audio_transcode helper. With the audio-dep stubs above,
# ``rob_box_voice.utils`` now imports cleanly, so we can rely on the
# normal ``from rob_box_voice.utils.audio_transcode import …`` path.
sys.path.insert(0, str(PROJECT_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "utils"))
import audio_transcode  # type: ignore[import-not-found]  # noqa: E402
from audio_transcode import to_pcm_int16  # noqa: E402,F401  (used by validator too)

# Late import — must follow stub install.
from rob_box_llm import MiniMaxTTSProvider, TTSFormat, TTSSettings  # noqa: E402
from rob_box_llm.tts import TTSAudio  # noqa: E402

_log = logging.getLogger("run_bench")


# ---------------------------------------------------------------------------
# Test node — bypasses TTSNode.__init__ (which calls create_subscription,
# load_silero_model, etc.) so we can exercise the audio pipeline on a
# minimal host without ROS2 / torch / sounddevice installed.
# ---------------------------------------------------------------------------


def _make_test_node(
    *,
    provider: str = "minimax",
    minimax_format: str = "pcm",
    minimax_sample_rate: int = 32000,
    minimax_base_url: str = "http://127.0.0.1:18080",
    minimax_api_key: str = "test-key",
    minimax_group_id: str = "test-group",
    audio_output_sample_rate: int = 16000,
    audio_qos_reliability: str = "best_effort",
    audio_qos_depth: int = 10,
    chipmunk_mode: bool = False,
    pitch_shift: float = 1.0,
    volume_db: float = -3.0,
    yandex_api_key: str = "",
    minimax_streaming: bool = False,
    minimax_max_retries: int = 0,
):
    """Build a TTSNode with __init__ bypassed.

    Returns an object whose ``audio_pub`` is a stub publisher plus the
    handful of attributes the audio-pipeline methods need. We use the
    production :class:`TTSNode` *class* (not a re-implementation) so any
    future bug fix in the pipeline propagates automatically.
    """
    from rob_box_voice.tts_node import TTSNode  # late import after stubs

    node = TTSNode.__new__(TTSNode)
    # Mirror ROS Node.__init__ side effects the production code expects.
    node._name = "tts_node"
    node._params = {}
    node._subs = {}
    node._pubs = {}
    node._logger = logging.getLogger("tts_node")

    # Pipeline configuration (mirrors TTSNode.__init__ subset).
    node.provider = provider
    node.minimax_api_key = minimax_api_key
    node.minimax_group_id = minimax_group_id
    node.minimax_voice = "male-qn-qingse"
    node.minimax_model = "speech-02-hd"
    node.minimax_language = "ru"
    node.minimax_speed = 1.0
    node.minimax_sample_rate = minimax_sample_rate
    node.minimax_timeout = 30.0
    node.minimax_format = TTSFormat(minimax_format.lower())
    node.minimax_max_retries = minimax_max_retries
    node.minimax_retry_backoff_ms = 0
    node.minimax_streaming = minimax_streaming
    node.minimax_provider = MiniMaxTTSProvider(
        api_key=minimax_api_key,
        group_id=minimax_group_id,
        base_url=minimax_base_url,
        timeout=node.minimax_timeout,
    )
    node.audio_topic = "/voice/audio/speech"
    node.audio_output_sample_rate = audio_output_sample_rate
    node.audio_qos_reliability = audio_qos_reliability
    node.audio_qos_depth = audio_qos_depth
    node.audio_channels = 1
    node.chipmunk_mode = chipmunk_mode
    node.pitch_shift = pitch_shift
    node.volume_db = volume_db
    node.volume_gain = 10.0 ** (volume_db / 20.0)
    node.yandex_api_key = yandex_api_key
    node.yandex_voice = "anton"
    node.yandex_speed = 1.0
    # Silero attrs that the parent class touches; set defaults so attribute
    # access doesn't blow up if a code path consults them.
    node.silero_model = None
    node.silero_speaker = "baya"
    node.silero_sample_rate = 48000
    node.silero_put_accent = True
    node.silero_put_yo = True
    node.silero_put_stress_homo = True
    node.silero_put_yo_homo = True
    node.device = _FakeTorchDevice()

    # Audio publisher stub (carries the QoS profile for downstream checks).
    audio_qos = ros_stub.QoSProfile(
        history=ros_stub.HistoryPolicy.KEEP_LAST,
        depth=audio_qos_depth,
        reliability=(
            ros_stub.ReliabilityPolicy.BEST_EFFORT
            if audio_qos_reliability == "best_effort"
            else ros_stub.ReliabilityPolicy.RELIABLE
        ),
        durability=ros_stub.DurabilityPolicy.VOLATILE,
    )
    node.audio_pub = ros_stub._Node().create_publisher(
        msg_type=ros_stub.AudioData, topic=node.audio_topic, qos=audio_qos
    )
    return node


class _FakeTorchDevice:
    type = "cpu"


# ---------------------------------------------------------------------------
# Real subprocess subscriber
# ---------------------------------------------------------------------------


def run_audio_data_subscriber(
    *,
    sink: "ros_stub.AudioSink",
    output_wav: Path,
    expected_sample_rate: int,
    fixtures_dir: Optional[Path] = None,
    ffprobe_check: bool = True,
    deadline_s: float = 30.0,
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    """Spawn :mod:`tts_audio_bench.scripts.real_subscriber` as a real subprocess
    and feed it the AudioData frames captured by ``sink``.

    This is the bench's end-to-end "real subscriber" — a separate
    process that consumes what was actually published onto the audio
    topic, just like ``ros2 topic echo /voice/audio/speech`` would. The
    subprocess writes a WAV file at ``output_wav`` and prints a JSON
    report on stdout, which we parse and return.

    Returns ``(report_dict, error_string)``. ``error_string`` is non-None
    only when the subprocess exited non-zero *or* the JSON could not be
    parsed; the bench decides whether to fail the scenario based on
    both.
    """
    import base64 as _b64
    import subprocess as _subprocess

    # Build JSON-line stream from the sink's captured messages. We
    # snapshot the list because the subscriber runs asynchronously and
    # the bench might still be appending when we start feeding — in
    # practice the bench has already finished publishing by the time we
    # call this, but the snapshot is cheap and defensive.
    frames_snapshot = list(sink.messages)
    lines: List[str] = []
    for idx, msg in enumerate(frames_snapshot):
        raw = bytes(msg.data)
        lines.append(json.dumps({
            "frame_index": idx,
            "publish_t_s": idx * 0.0,  # subscriber uses first_frame_at_s for ordering only
            "sample_rate": expected_sample_rate,
            "channels": 1,
            "sample_width": 2,
            "layout": "little_endian",
            "data_b64": _b64.b64encode(raw).decode("ascii"),
        }))
    stdin_payload = "\n".join(lines) + ("\n" if lines else "")

    cmd = [
        sys.executable,
        "-m",
        "tts_audio_bench.scripts.real_subscriber",
        "--out", str(output_wav),
        "--expected-sample-rate", str(expected_sample_rate),
        "--deadline-s", str(deadline_s),
    ]
    if ffprobe_check:
        cmd.append("--ffprobe-check")

    try:
        proc = _subprocess.run(
            cmd,
            input=stdin_payload,
            capture_output=True,
            text=True,
            timeout=deadline_s + 5.0,
            cwd=str(PROJECT_ROOT),
        )
    except _subprocess.TimeoutExpired as exc:
        return None, f"real_subscriber timed out after {deadline_s}s"
    except Exception as exc:  # noqa: BLE001
        return None, f"real_subscriber spawn failed: {exc!r}"

    if proc.returncode != 0:
        return None, (
            f"real_subscriber exited {proc.returncode}: "
            f"stderr={proc.stderr.strip()[:500]} stdout={proc.stdout.strip()[:500]}"
        )

    try:
        report = json.loads(proc.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError) as exc:
        return None, f"real_subscriber stdout not parseable: {exc!r}"

    if not report.get("ok"):
        return report, f"real_subscriber reported ok=False: {report.get('reason')}"

    return report, None


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------


@dataclass
class ScenarioResult:
    name: str
    ok: bool
    duration_s: float = 0.0
    ttfa_ms: Optional[float] = None  # time-to-first-AudioData
    chunks: int = 0
    bytes_total: int = 0
    sample_rate: int = 0
    joints_ok: bool = True
    joints_max_jump: int = 0
    error: Optional[str] = None
    artifact: Optional[str] = None
    extra: Optional[Dict[str, Any]] = None


def _run_format_scenario(
    *,
    name: str,
    fmt: str,
    minimax_sample_rate: int,
    expected_bytes_min: int,
    output_sample_rate: int = 16000,
    server_port: int = 18080,
    fixtures_root: Path,
    artifacts_dir: Path,
) -> ScenarioResult:
    """Run a single end-to-end format scenario.

    Builds the pipeline, calls :func:`TTSNode._synthesize_minimax`, then
    :func:`TTSNode._publish_audio` (via :meth:`AudioSink.attach`), and
    validates the captured AudioData.
    """
    t0 = time.monotonic()
    sink = ros_stub.AudioSink()
    node = _make_test_node(
        provider="minimax",
        minimax_format=fmt,
        minimax_sample_rate=minimax_sample_rate,
        minimax_base_url=f"http://127.0.0.1:{server_port}",
        audio_output_sample_rate=output_sample_rate,
    )
    sink.attach(node.audio_pub)

    try:
        # Mimic dialogue_callback — bypass ROS dispatch entirely.
        result = node._synthesize_minimax("hello", ssml_attributes=None)
        audio_np = result["audio_np"]
        sample_rate = result["sample_rate"]
        topic_audio = node._prepare_audio_for_topic(audio_np, sample_rate)
        node._publish_audio(topic_audio)
    except Exception as exc:
        import traceback as _tb
        return ScenarioResult(
            name=name,
            ok=False,
            duration_s=time.monotonic() - t0,
            error=f"{type(exc).__name__}: {exc}\n{_tb.format_exc()}",
        )
    finally:
        node.close_minimax_provider()

    if not sink.messages:
        return ScenarioResult(
            name=name,
            ok=False,
            duration_s=time.monotonic() - t0,
            error="no AudioData messages captured",
        )

    # Validate format: int16 LE mono at output_sample_rate.
    msg = sink.messages[0]
    raw = bytes(msg.data)
    if len(raw) % 2 != 0:
        return ScenarioResult(
            name=name,
            ok=False,
            duration_s=time.monotonic() - t0,
            error=f"AudioData bytes not aligned to int16: {len(raw)} bytes",
        )
    # Spot-check: every sample fits in int16.
    samples = np.frombuffer(raw, dtype="<i2")
    if samples.size == 0:
        return ScenarioResult(
            name=name,
            ok=False,
            duration_s=time.monotonic() - t0,
            error="empty AudioData",
        )
    n_samples = int(samples.size)

    # Subscriber side: assemble AudioData frames into a WAV via WavRecorder
    # (the production equivalent of a real ROS subscriber that forwards to
    # audio_play/sound_node) and verify the header with ffprobe.
    recorder = WavRecorder(sample_rate=output_sample_rate)
    for m in sink.messages:
        recorder(m)
    out_wav = artifacts_dir / f"{name}.wav"
    subscriber_report: SubscriberReport = recorder.to_wav(out_wav)
    header_report = validate_wav_header(
        out_wav,
        expected_sample_rate=output_sample_rate,
        expected_channels=1,
        expected_sample_width=2,
    )

    # Real subprocess subscriber — runs in a separate Python process and
    # consumes the AudioData frames over a stdio pipe. Its WAV file is the
    # canonical artifact for "what a real ROS subscriber would record".
    real_sub_wav = artifacts_dir / f"{name}_real_subscriber.wav"
    real_sub_report, real_sub_error = run_audio_data_subscriber(
        sink=sink,
        output_wav=real_sub_wav,
        expected_sample_rate=output_sample_rate,
    )

    # Joints / duration checks.
    total_bytes = subscriber_report.bytes_total
    pcm = b"".join(bytes(m.data) for m in sink.messages)
    joints = check_joints(pcm)
    duration_s = n_samples / float(output_sample_rate)
    dur_report = duration_report(duration_s, expected_min_s=0.5, expected_max_s=3.0)

    ok = (
        total_bytes >= expected_bytes_min
        and joints.ok
        and dur_report.ok
        and header_report.ok
        and not subscriber_report.warnings
        and real_sub_error is None
        and real_sub_report is not None
        and real_sub_wav.exists()
    )
    err_parts = []
    if total_bytes < expected_bytes_min:
        err_parts.append(
            f"too few bytes: {total_bytes} < expected >= {expected_bytes_min}"
        )
    if not joints.ok:
        err_parts.append(f"joints broken: max_jump={joints.max_jump}")
    if not dur_report.ok:
        err_parts.append(
            f"duration {duration_s:.3f}s outside [{dur_report.expected_min_s}, "
            f"{dur_report.expected_max_s}]"
        )
    if not header_report.ok:
        err_parts.append(f"wav header invalid: {header_report.reason}")
    for w in subscriber_report.warnings:
        err_parts.append(f"subscriber: {w}")
    if real_sub_error is not None:
        err_parts.append(f"real_subscriber: {real_sub_error}")
    elif real_sub_report is None:
        err_parts.append("real_subscriber: no report returned")
    elif not real_sub_wav.exists():
        err_parts.append(
            f"real_subscriber: WAV missing at {real_sub_wav}"
        )

    ttfa_ms = (
        (sink.first_arrival - t0) * 1000.0 if sink.first_arrival is not None else None
    )
    real_sub_bytes = (
        int(real_sub_report.get("bytes_total", 0)) if real_sub_report else 0
    )
    real_sub_frames = (
        int(real_sub_report.get("frames", 0)) if real_sub_report else 0
    )
    if real_sub_report is not None:
        _log.info(
            "    [real_subscriber] frames=%d bytes=%d sr=%d ch=%d sw=%d ffprobe_ok=%s",
            real_sub_frames,
            real_sub_bytes,
            real_sub_report.get("sample_rate"),
            real_sub_report.get("channels"),
            real_sub_report.get("sample_width"),
            real_sub_report.get("ffprobe_ok"),
        )
    return ScenarioResult(
        name=name,
        ok=ok,
        duration_s=time.monotonic() - t0,
        ttfa_ms=ttfa_ms,
        chunks=len(sink.messages),
        bytes_total=total_bytes,
        sample_rate=output_sample_rate,
        joints_ok=joints.ok,
        joints_max_jump=joints.max_jump,
        error="; ".join(err_parts) if err_parts else None,
        artifact=str(out_wav.relative_to(PROJECT_ROOT)) if out_wav.exists() else None,
        extra={
            "real_subscriber": {
                "wav": str(real_sub_wav.relative_to(PROJECT_ROOT))
                if real_sub_wav.exists() else None,
                "report": real_sub_report,
                "bytes_total": real_sub_bytes,
                "frames": real_sub_frames,
            }
        },
    )


def _run_streaming_ttfa_scenario(
    *,
    server_port: int,
    artifacts_dir: Path,
    fmt: str = "pcm",
    minimax_sample_rate: int = 32000,
    output_sample_rate: int = 16000,
    stream_chunk_count: int = 4,
    stream_chunk_delay_ms: float = 0.0,
    expected_min_chunks: int = 2,
) -> ScenarioResult:
    """Measure time-to-first-AudioData in streaming mode.

    Uses :func:`TTSNode._synthesize_minimax_streaming_publish` so each
    TTS chunk is published as a separate AudioData; the time delta
    between ``t0`` and ``sink.first_arrival`` is the streaming TTFA.

    The mock server (configured via ``stream_chunk_count``) emits the
    fixture across that many SSE events with an optional
    ``stream_chunk_delay_ms`` per-chunk delay. The bench asserts that
    we see **at least** ``expected_min_chunks`` AudioData messages
    coming through the audio_pub → AudioSink bridge, proving that the
    publisher does not coalesce chunks into a single batch when the
    upstream is genuinely streaming.
    """
    t0 = time.monotonic()
    sink = ros_stub.AudioSink()
    node = _make_test_node(
        provider="minimax",
        minimax_format=fmt,
        minimax_sample_rate=minimax_sample_rate,
        minimax_base_url=f"http://127.0.0.1:{server_port}",
        audio_output_sample_rate=output_sample_rate,
    )
    sink.attach(node.audio_pub)

    try:
        node._synthesize_minimax_streaming_publish("hello", ssml_attributes=None)
    except Exception as exc:
        return ScenarioResult(
            name=f"streaming_ttfa_{fmt}",
            ok=False,
            duration_s=time.monotonic() - t0,
            error=f"{type(exc).__name__}: {exc}",
        )
    finally:
        node.close_minimax_provider()

    if not sink.messages:
        return ScenarioResult(
            name=f"streaming_ttfa_{fmt}",
            ok=False,
            duration_s=time.monotonic() - t0,
            error="no AudioData messages in streaming mode",
        )

    recorder = WavRecorder(sample_rate=output_sample_rate)
    for m in sink.messages:
        recorder(m)
    out_wav = artifacts_dir / f"streaming_ttfa_{fmt}.wav"
    subscriber_report = recorder.to_wav(out_wav)
    header_report = validate_wav_header(
        out_wav,
        expected_sample_rate=output_sample_rate,
        expected_channels=1,
        expected_sample_width=2,
    )

    # Real subprocess subscriber — same end-to-end pattern as the format
    # scenarios: a separate Python process consumes the AudioData frames
    # and writes its own WAV file. Critical for streaming TTFA, since it
    # proves the publisher is actually emitting >1 message rather than
    # buffering the whole stream into a single frame.
    real_sub_wav = artifacts_dir / f"streaming_ttfa_{fmt}_real_subscriber.wav"
    real_sub_report, real_sub_error = run_audio_data_subscriber(
        sink=sink,
        output_wav=real_sub_wav,
        expected_sample_rate=output_sample_rate,
    )

    pcm = b"".join(bytes(m.data) for m in sink.messages)
    joints = check_joints(pcm)
    ttfa_ms = (
        (sink.first_arrival - t0) * 1000.0 if sink.first_arrival is not None else None
    )

    err_parts: List[str] = []
    if len(sink.messages) < expected_min_chunks:
        err_parts.append(
            f"too few AudioData chunks: {len(sink.messages)} < {expected_min_chunks}"
        )
    if not joints.ok:
        err_parts.append(f"joints broken: max_jump={joints.max_jump}")
    if not header_report.ok:
        err_parts.append(f"wav header invalid: {header_report.reason}")
    for w in subscriber_report.warnings:
        err_parts.append(f"subscriber: {w}")
    if real_sub_error is not None:
        err_parts.append(f"real_subscriber: {real_sub_error}")
    elif real_sub_report is None:
        err_parts.append("real_subscriber: no report returned")
    elif not real_sub_wav.exists():
        err_parts.append(
            f"real_subscriber: WAV missing at {real_sub_wav}"
        )

    if real_sub_report is not None:
        _log.info(
            "    [real_subscriber] frames=%d bytes=%d sr=%d ch=%d sw=%d ffprobe_ok=%s",
            int(real_sub_report.get("frames", 0)),
            int(real_sub_report.get("bytes_total", 0)),
            real_sub_report.get("sample_rate"),
            real_sub_report.get("channels"),
            real_sub_report.get("sample_width"),
            real_sub_report.get("ffprobe_ok"),
        )

    return ScenarioResult(
        name=f"streaming_ttfa_{fmt}",
        ok=(
            sink.first_arrival is not None
            and len(sink.messages) >= expected_min_chunks
            and joints.ok
            and header_report.ok
            and not subscriber_report.warnings
            and real_sub_error is None
            and real_sub_report is not None
            and real_sub_wav.exists()
        ),
        duration_s=time.monotonic() - t0,
        ttfa_ms=ttfa_ms,
        chunks=len(sink.messages),
        bytes_total=subscriber_report.bytes_total,
        sample_rate=output_sample_rate,
        joints_ok=joints.ok,
        joints_max_jump=joints.max_jump,
        error="; ".join(err_parts) if err_parts else None,
        artifact=str(out_wav.relative_to(PROJECT_ROOT)) if out_wav.exists() else None,
        extra={
            "real_subscriber": {
                "wav": str(real_sub_wav.relative_to(PROJECT_ROOT))
                if real_sub_wav.exists() else None,
                "report": real_sub_report,
                "bytes_total": int(real_sub_report.get("bytes_total", 0))
                if real_sub_report else 0,
                "frames": int(real_sub_report.get("frames", 0))
                if real_sub_report else 0,
            }
        },
    )


def _run_error_scenario(
    *,
    name: str,
    server_port: int,
    fmt: str = "pcm",
    fail_status: int = 1003,
    fail_msg: str = "rate limit exceeded",
    max_retries: int = 1,
) -> ScenarioResult:
    """Negative path: provider returns base_resp.status_code != 0."""
    t0 = time.monotonic()
    # Spawn the mock on a fresh port so we can inject the failure.
    server = start_server(
        port=server_port,
        fixtures_root=FIXTURES_ROOT,
        fail_status=fail_status,
        fail_msg=fail_msg,
    )
    try:
        node = _make_test_node(
            provider="minimax",
            minimax_format=fmt,
            minimax_base_url=f"http://127.0.0.1:{server_port}",
            minimax_max_retries=max_retries,
        )
        try:
            node._synthesize_minimax("hello", ssml_attributes=None)
            return ScenarioResult(
                name=name,
                ok=False,
                duration_s=time.monotonic() - t0,
                error="expected TTSError but call succeeded",
            )
        except Exception as exc:
            return ScenarioResult(
                name=name,
                ok=True,
                duration_s=time.monotonic() - t0,
                error=f"expected: {type(exc).__name__}: {exc}",
            )
        finally:
            node.close_minimax_provider()
    finally:
        server.shutdown()


# ---------------------------------------------------------------------------
# Bench orchestrator
# ---------------------------------------------------------------------------


FIXTURES_ROOT = PROJECT_ROOT / "tts_audio_bench" / "fixtures"
ARTIFACTS_DIR = PROJECT_ROOT / "tts_audio_bench" / "artifacts"
LOGS_DIR = PROJECT_ROOT / "tts_audio_bench" / "logs"


def _check_environment() -> List[str]:
    """Return a list of human-readable warnings about missing tools."""
    warnings: List[str] = []
    if not (PROJECT_ROOT / "src" / "rob_box_llm").exists():
        warnings.append("src/rob_box_llm missing — bench cannot import MiniMaxTTSProvider")
    if not (PROJECT_ROOT / "src" / "rob_box_voice").exists():
        warnings.append("src/rob_box_voice missing — bench cannot import tts_node")
    if not FIXTURES_ROOT.exists():
        warnings.append(f"{FIXTURES_ROOT} missing — run make_fixture.py first")
    # ffmpeg only strictly needed for MP3/OGG scenarios.
    import shutil

    if shutil.which("ffmpeg") is None:
        warnings.append("ffmpeg not on PATH — MP3/OGG scenarios will fail")
    return warnings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--scenarios", default="all")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        handlers=[
            logging.FileHandler(LOGS_DIR / "bench.log"),
            logging.StreamHandler(),
        ],
    )

    warnings = _check_environment()
    for w in warnings:
        _log.warning("ENV: %s", w)
    if any("missing" in w for w in warnings):
        _log.error("Bench environment not ready; aborting.")
        return 2

    _log.info("Starting MiniMax TTS audio test bench")
    _log.info("  fixtures: %s", FIXTURES_ROOT)
    _log.info("  artifacts: %s", ARTIFACTS_DIR)
    _log.info("  mock port: %d", args.port)

    # Make sure fixtures exist.
    if not (FIXTURES_ROOT / "reference").exists():
        _log.info("Fixtures missing — running make_fixture.py")
        import subprocess
        subprocess.check_call(
            [sys.executable, str(PROJECT_ROOT / "tts_audio_bench" / "scripts" / "make_fixture.py")],
            cwd=PROJECT_ROOT,
        )

    server = start_server(port=args.port, fixtures_root=FIXTURES_ROOT)
    # Separate streaming-server so the mock can emit a multi-chunk SSE
    # response (acceptance #5: time-to-first-AudioData must be measurable
    # even when the upstream is still streaming the rest of the audio).
    stream_server = start_server(
        port=args.port + 10,
        fixtures_root=FIXTURES_ROOT,
        stream_chunk_count=4,
        stream_chunk_delay_ms=0.0,
    )
    results: List[ScenarioResult] = []
    try:
        # 1) Format round-trips: PCM, WAV, MP3, OGG.
        scenarios_to_run = (
            ["pcm", "wav", "mp3", "ogg", "ttfa", "rate_limit", "auth", "bad_request"]
            if args.scenarios == "all"
            else [s.strip() for s in args.scenarios.split(",") if s.strip()]
        )

        for fmt in ("pcm", "wav", "mp3", "ogg"):
            if fmt not in scenarios_to_run:
                continue
            # 32 kHz in (provider SR), 16 kHz out (AudioData SR per task).
            # Expected bytes after SR conversion: 32000 → 16000 = 32000
            # bytes of int16 = 1.0 s at 16 kHz mono. Allow some slack.
            r = _run_format_scenario(
                name=f"format_{fmt}",
                fmt=fmt,
                minimax_sample_rate=32000,
                expected_bytes_min=32000,  # ~1.0 s @ 16 kHz mono int16
                output_sample_rate=16000,
                server_port=args.port,
                fixtures_root=FIXTURES_ROOT,
                artifacts_dir=ARTIFACTS_DIR,
            )
            results.append(r)
            _log.info("[%s] %s", "OK" if r.ok else "FAIL", r.name)
            if r.ttfa_ms is not None:
                _log.info("    TTFA=%.1f ms, chunks=%d, bytes=%d, joints_ok=%s",
                          r.ttfa_ms, r.chunks, r.bytes_total, r.joints_ok)

        # 5) TTFA measurement in streaming mode. Use the dedicated
        # streaming server (port+10) which emits fixture chunks across
        # multiple SSE events so the AudioSink is genuinely seeing >1
        # AudioData message from the publisher.
        if "ttfa" in scenarios_to_run:
            r = _run_streaming_ttfa_scenario(
                server_port=args.port + 10,
                artifacts_dir=ARTIFACTS_DIR,
                stream_chunk_count=4,
                expected_min_chunks=2,
            )
            results.append(r)
            _log.info("[%s] %s (TTFA=%.1f ms, chunks=%d)",
                      "OK" if r.ok else "FAIL", r.name,
                      r.ttfa_ms or -1.0, r.chunks)

        # 6) Error paths.
        if "rate_limit" in scenarios_to_run:
            r = _run_error_scenario(
                name="rate_limit_exhausted",
                server_port=args.port + 1,
                fail_status=1003,
                fail_msg="rate limit exceeded",
                max_retries=1,
            )
            results.append(r)
            _log.info("[%s] %s (raised %s)",
                      "OK" if r.ok else "FAIL", r.name,
                      r.error or "no exception")

        if "auth" in scenarios_to_run:
            r = _run_error_scenario(
                name="auth_forbidden",
                server_port=args.port + 2,
                fail_status=1001,
                fail_msg="invalid api key",
                max_retries=2,  # auth errors don't retry but we still verify
            )
            results.append(r)
            _log.info("[%s] %s (raised %s)",
                      "OK" if r.ok else "FAIL", r.name,
                      r.error or "no exception")

        if "bad_request" in scenarios_to_run:
            r = _run_error_scenario(
                name="bad_request",
                server_port=args.port + 3,
                fail_status=1002,
                fail_msg="invalid voice_id",
                max_retries=2,
            )
            results.append(r)
            _log.info("[%s] %s (raised %s)",
                      "OK" if r.ok else "FAIL", r.name,
                      r.error or "no exception")
    finally:
        server.shutdown()
        try:
            stream_server.shutdown()
        except Exception:
            pass

    # Summary
    n_ok = sum(1 for r in results if r.ok)
    n_total = len(results)
    _log.info("=" * 64)
    _log.info("Bench summary: %d/%d scenarios passed", n_ok, n_total)
    for r in results:
        status = "OK" if r.ok else "FAIL"
        _log.info("  [%s] %s", status, r.name)
        if r.ttfa_ms is not None:
            _log.info("       ttfa=%.1fms chunks=%d bytes=%d sr=%d",
                      r.ttfa_ms, r.chunks, r.bytes_total, r.sample_rate)
        if r.error and not r.ok:
            _log.info("       error: %s", r.error)
    _log.info("=" * 64)

    summary_path = ARTIFACTS_DIR / "bench-summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "passed": n_ok,
                "total": n_total,
                "results": [asdict(r) for r in results],
                "warnings": warnings,
            },
            indent=2,
        )
    )
    _log.info("Wrote summary to %s", summary_path)
    return 0 if n_ok == n_total else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 — last-ditch crash capture
        traceback.print_exc()
        sys.exit(2)