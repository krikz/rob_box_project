"""Issue #4693 — измеренные +12 dB только для English_Whispering_girl."""

from __future__ import annotations

import math
import shutil
import struct
import subprocess
import wave
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / "scripts" / "ci" / "e2e" / "e2e_voice_test.sh"
SCRIPT_TEXT = E2E_SCRIPT.read_text(encoding="utf-8")
SCRIPT_LINES = SCRIPT_TEXT.replace("\r\n", "\n").split("\n")
BASE_FILTER = "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1"
WHISPER_FILTER = "highpass=f=100,volume=3.0,volume=12dB,"
WHISPER_FILTER += "alimiter=limit=0.98,adelay=1500|all=1"


def extract_function(name: str) -> str:
    def is_function_start(line: str) -> bool:
        return line.startswith(name + "() {")

    start = -1
    for i, line in enumerate(SCRIPT_LINES):
        if is_function_start(line):
            start = i
            break
    if start < 0:
        raise AssertionError(name)
    for i in range(start + 1, len(SCRIPT_LINES)):
        if SCRIPT_LINES[i] == "}":
            return "\n".join(SCRIPT_LINES[start : i + 1])  # noqa: E203
    raise AssertionError(name)


def filter_for_voice(tmp_path: Path, voice: str) -> str:
    script = "\n".join(
        [
            "set -eu",
            "ffmpeg() { printf '%s\\n' \"$@\"; }",
            extract_function("eq_command_wav"),
            f'eq_command_wav input.wav output.wav "{voice}"',
        ]
    )
    result = subprocess.run(
        ["bash", "-s"], input=script.encode(), capture_output=True, timeout=20
    )
    assert result.returncode == 0, result.stderr.decode("utf-8", "replace")
    args = result.stdout.decode("utf-8").splitlines()
    return args[args.index("-af") + 1]


@pytest.mark.parametrize("voice", ["filipp", "kostya"])
def test_whisper_gets_12_db_before_limiter(tmp_path, voice):
    assert filter_for_voice(tmp_path, voice) == WHISPER_FILTER


@pytest.mark.parametrize(
    "voice", ["anton", "ermil", "zahar", "", "Russian_ReliableMan"]
)
def test_other_voices_keep_the_original_filter(tmp_path, voice):
    assert filter_for_voice(tmp_path, voice) == BASE_FILTER


def audio_metrics(path: Path) -> tuple[float, float, float]:
    with wave.open(str(path), "rb") as wav:
        rate = wav.getframerate()
        frames = wav.getnframes()
        samples = struct.unpack(f"<{frames}h", wav.readframes(frames))
    energy = sum(sample * sample for sample in samples)
    rms = math.sqrt(energy / len(samples)) / 32767
    peak = max(abs(sample) for sample in samples) / 32767
    return rms, peak, frames / rate


def run_audio_eq(
    tmp_path: Path, voice: str, amplitude: float
) -> tuple[float, float, float, float]:
    raw = tmp_path / "raw.wav"
    out = tmp_path / "out.wav"
    rate = 32000
    samples = [
        int(amplitude * 32767 * math.sin(2 * math.pi * 440 * i / rate))
        for i in range(rate // 5)
    ]
    with wave.open(str(raw), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(rate)
        wav.writeframes(struct.pack(f"<{len(samples)}h", *samples))

    script = "\n".join(
        [
            "set -eu",
            "RAW=raw.wav",
            "OUT=out.wav",
            extract_function("eq_command_wav"),
            f'eq_command_wav "$RAW" "$OUT" "{voice}"',
        ]
    )
    result = subprocess.run(
        ["bash", "-s"],
        input=script.encode(),
        capture_output=True,
        timeout=30,
        cwd=tmp_path,
    )
    assert result.returncode == 0, result.stderr.decode("utf-8", "replace")
    return (*audio_metrics(out), audio_metrics(raw)[0])


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="нужен ffmpeg")
def test_whisper_gain_preserves_preroll_duration_and_clip_budget(tmp_path):
    rms, peak, duration, raw_rms = run_audio_eq(tmp_path, "filipp", 0.001)
    assert rms == pytest.approx(raw_rms * 10 ** (12 / 20), rel=0.08)
    assert duration == pytest.approx(1.7, abs=0.05)
    assert peak <= 0.9801


def test_vad_budget_is_not_changed():
    text = E2E_SCRIPT.read_text(encoding="utf-8")
    assert 'E2E_VAD_MAX_DURATION="${E2E_VAD_MAX_DURATION:-15.0}"' in text
    assert 'E2E_VAD_OVERHEAD="${E2E_VAD_OVERHEAD:-2.5}"' in text
