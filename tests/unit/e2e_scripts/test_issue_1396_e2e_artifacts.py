"""Юнит-тесты для e2e_voice_test.sh supporting scripts (issue #1396).

Сценарий: реализация дёргается через subprocess CLI (как из workflow),
без mock'ов и без сети. Все файлы (test wav / log) — синтетические.

Скрипты:
  - .github/workflows/scripts/e2e_audio_metrics.py
  - .github/workflows/scripts/e2e_baseline_diff.py
  - .github/workflows/scripts/e2e_timing.py
"""
from __future__ import annotations

import json
import re
import struct
import subprocess
import sys
import wave
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[3] / ".github/workflows/scripts"


# ─────────────────────────── helpers ───────────────────────────
def _write_wav(
    path: Path,
    samples: list[int],
    sr: int = 16000,
) -> None:
    """Записать моно int16 wav."""
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(struct.pack(f"<{len(samples)}h", *samples))


def _sine_samples(sr: int, duration_s: float, freq_hz: float = 440.0,
                  amplitude_dbfs: float = -20.0) -> list[int]:
    """Генерирует массив int16 сэмплов с синусом нужной амплитуды."""
    import math
    n = int(sr * duration_s)
    amp = 32767 * (10 ** (amplitude_dbfs / 20.0))
    return [int(amp * math.sin(2 * math.pi * freq_hz * i / sr)) for i in range(n)]


def _script_path(name: str) -> Path:
    p = SCRIPTS_DIR / name
    if not p.exists():
        pytest.skip(f"script not found: {p}")
    return p


def _run(args: list[str], **kw) -> subprocess.CompletedProcess:
    """Запустить CLI-скрипт, вернуть CompletedProcess."""
    cmd = [sys.executable, str(_script_path(args[0])), *args[1:]]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=30, **kw)


# ─────────────────────────── e2e_audio_metrics.py ───────────────────────────
class TestAudioMetrics:
    def test_sine_minus20dbfs_runs_produces_json(self, tmp_path: Path):
        """-20dB sine 1s: пик -20dB FS, RMS ≈ -23dB FS, mic_working=true, silence_ratio=0."""
        wav = tmp_path / "rec.wav"
        _write_wav(wav, _sine_samples(16000, 1.0, amplitude_dbfs=-20.0))
        result = _run(["e2e_audio_metrics.py", str(wav)])
        assert result.returncode == 0, result.stderr
        d = json.loads(result.stdout)
        assert d["wav_path"] == str(wav)
        assert d["duration_s"] == 1.0
        assert d["sample_rate_hz"] == 16000
        assert d["channels"] == 1
        assert d["peak_dbfs"] == -20.0
        assert abs(d["rms_dbfs"] - (-20.0 - 3.01)) < 0.1   # sine: RMS = peak - 3dB
        assert d["silence_ratio"] == 0.0
        assert d["mic_working"] is True

    def test_silence_window_detected(self, tmp_path: Path):
        """0.5s silence (-inf RMS) + 0.5s loud (-6dB) → silence_ratio = 0.5."""
        wav = tmp_path / "rec.wav"
        sr = 16000
        # 0.5s zeros + 0.5s @ -6dB (peak)
        silence = [0] * (sr // 2)
        loud = _sine_samples(sr, 0.5, amplitude_dbfs=-6.0)
        _write_wav(wav, silence + loud)
        d = json.loads(_run(["e2e_audio_metrics.py", str(wav)]).stdout)
        assert d["silence_ratio"] == 0.5
        assert d["peak_dbfs"] == -6.0

    def test_missing_file_returns_error_dict(self, tmp_path: Path):
        d = json.loads(_run(["e2e_audio_metrics.py", str(tmp_path / "absent.wav")]).stdout)
        assert "error" in d
        assert "not found" in d["error"].lower()

    def test_empty_wav(self, tmp_path: Path):
        """0-sample wav → meters в -inf, не падает."""
        wav = tmp_path / "empty.wav"
        _write_wav(wav, [])
        d = json.loads(_run(["e2e_audio_metrics.py", str(wav)]).stdout)
        # 0 windows → silence_ratio=0 by definition (no data)
        assert d["samples_analyzed"] == 0
        assert d["duration_s"] == 0.0
        # RMS/peak могут быть -inf


# ─────────────────────────── e2e_baseline_diff.py ───────────────────────────
class TestBaselineDiff:
    def test_recording_within_tolerance_passes(self, tmp_path: Path):
        """Recording -23dB vs Golden -23dB → diff=0, pass."""
        rec = tmp_path / "rec.wav"
        gold = tmp_path / "gold.wav"
        # -23dB sine 1s. RMS ≈ -26dB FS для sine (peak -23 → RMS -26).
        _write_wav(gold, _sine_samples(16000, 1.0, amplitude_dbfs=-23.0))
        _write_wav(rec, _sine_samples(16000, 1.0, amplitude_dbfs=-23.0))
        d = json.loads(_run(["e2e_baseline_diff.py", str(rec), str(gold),
                              "Робот, привет"]).stdout)
        assert d["pass"] is True
        assert d["rms_diff_db"] == pytest.approx(0.0, abs=0.2)
        assert d["duration_diff_pct"] == pytest.approx(0.0, abs=0.1)

    def test_recording_too_quiet_fails(self, tmp_path: Path):
        """Recording -50dB vs Golden -20dB → 30dB diff → FAIL."""
        rec = tmp_path / "rec.wav"
        gold = tmp_path / "gold.wav"
        _write_wav(gold, _sine_samples(16000, 1.0, amplitude_dbfs=-20.0))
        _write_wav(rec, _sine_samples(16000, 1.0, amplitude_dbfs=-50.0))
        d = json.loads(_run(["e2e_baseline_diff.py", str(rec), str(gold),
                              "voice test"]).stdout)
        assert d["pass"] is False
        assert "rms_diff" in d["reason"].lower() or "expected_tool_calls" not in d

    def test_no_golden_uses_synthetic_baseline(self, tmp_path: Path):
        """Если golden=None, должен быть reason=no_golden и pass=mic_working."""
        rec = tmp_path / "rec.wav"
        _write_wav(rec, _sine_samples(16000, 1.0, amplitude_dbfs=-30.0))
        d = json.loads(_run(["e2e_baseline_diff.py", str(rec), "", "voice"]).stdout)
        assert d["reason"] == "no_golden"
        assert d["pass"] is True   # RMS -30 > -50
        assert d["golden_wav"] is None

    def test_duration_diff_fails(self, tmp_path: Path):
        """3s recording vs 1s golden → 200% diff → FAIL."""
        rec = tmp_path / "rec.wav"
        gold = tmp_path / "gold.wav"
        _write_wav(gold, _sine_samples(16000, 1.0, amplitude_dbfs=-20.0))
        _write_wav(rec, _sine_samples(16000, 3.0, amplitude_dbfs=-20.0))
        d = json.loads(_run(["e2e_baseline_diff.py", str(rec), str(gold),
                              "v"]).stdout)
        assert d["duration_diff_pct"] == pytest.approx(200.0, abs=1.0)
        assert d["pass"] is False
        assert "duration_diff" in d["reason"]

    def test_keyword_match_evaluates_transcript(self, tmp_path: Path):
        """Если в transcript.json есть expected_keywords → проверяет contains."""
        rec = tmp_path / "rec.wav"
        gold = tmp_path / "gold.wav"
        _write_wav(gold, _sine_samples(16000, 1.0, amplitude_dbfs=-23.0))
        _write_wav(rec, _sine_samples(16000, 1.0, amplitude_dbfs=-23.0))
        tr = tmp_path / "transcript.json"
        tr.write_text(
            json.dumps({
                "text": "робот привет меня зовут саша",
                "acceptance": {"expected_keywords": ["привет", "саша", "иван"]}
            }),
            encoding="utf-8",
        )
        d = json.loads(_run(["e2e_baseline_diff.py", str(rec), str(gold),
                              "v", str(tr)]).stdout)
        # 2/3 keywords matched (66.7%) < 80% → FAIL
        assert d["pass"] is False
        assert d["keyword_match_pct"] == pytest.approx(66.67, abs=0.1)
        assert "missing in recognized" in d["reason"] or "keyword" in d["reason"]

    def test_find_golden_by_slug(self, tmp_path: Path):
        """Если golden не задан вторым аргументом, ищем по slug в .github/e2e/golden."""
        # Это требует наличия golden-файла рядом с проектом — пропускаем если нет.
        golden_dir = Path(__file__).resolve().parents[3] / ".github/e2e/golden"
        if not golden_dir.is_dir():
            pytest.skip(".github/e2e/golden отсутствует (пока пустой)")
        # Иначе — мы не можем протестировать без заранее созданного golden.
        pytest.skip("нужен golden/<slug>.wav для проверки авто-поиска")


# ─────────────────────────── e2e_timing.py ───────────────────────────
SAMPLE_DOCKER_LOG = """\
[1000.0] 🎤 Получена фраза: 2.00с (32000 bytes)
[1000.8] ✅ ПРИНЯТО: Робот, привет как дела
[1000.85] 📥 LLM INPUT: ...
[1002.5] 🔊 Синтез через tts_node
[1003.5] [INFO] TTS finished success=True duration=1.20s
[1004.7] ✅ Воспроизведение завершено
"""


class TestTiming:
    def test_text_mode_human_readable(self, tmp_path: Path):
        log = tmp_path / "voice.log"
        log.write_text(SAMPLE_DOCKER_LOG, encoding="utf-8")
        result = _run(["e2e_timing.py", str(log), "Робот, привет как дела"])
        assert result.returncode == 0
        assert "T_total" in result.stdout
        assert "COMMAND:" in result.stdout
        assert "Робот, привет как дела" in result.stdout

    def test_json_mode_machine_readable(self, tmp_path: Path):
        log = tmp_path / "voice.log"
        log.write_text(SAMPLE_DOCKER_LOG, encoding="utf-8")
        result = _run(["e2e_timing.py", str(log), "Робот, привет как дела", "--json"])
        assert result.returncode == 0
        d = json.loads(result.stdout)
        assert d["voice_text"] == "Робот, привет как дела"
        assert d["recognized"] == "Робот, привет как дела"
        # Проверяем, что ключи из issue #1396 есть в JSON
        m = d["metrics_ms"]
        for key in ("wake_to_stt_end_ms", "stt_to_llm_ms", "llm_to_tts_start_ms",
                    "tts_to_playback_end_ms", "total_latency_ms"):
            assert key in m, f"missing metric: {key}"

        # Конкретные значения по синтетическому логу
        # wake (1000.0) → accept (1000.8) = 800ms
        assert m["wake_to_stt_end_ms"] == 800.0
        # accept (1000.8) → llm_in (1000.85) = 50ms
        assert m["stt_to_llm_ms"] == 50.0
        # wake (1000.0) → tts_ok (1003.5) = 3500ms
        assert m["total_latency_ms"] == 3500.0

    def test_partial_log_marks_partial(self, tmp_path: Path):
        log = tmp_path / "empty.log"
        log.write_text("", encoding="utf-8")
        result = _run(["e2e_timing.py", str(log), "x", "--json"])
        d = json.loads(result.stdout)
        assert d.get("partial_data") is True

    def test_text_mode_compatible_signature(self, tmp_path: Path):
        """Старый вызов (без --json) из L-E2E Voice Test.yml всё ещё работает."""
        log = tmp_path / "v.log"
        log.write_text(SAMPLE_DOCKER_LOG, encoding="utf-8")
        # Прямой вызов — тот, что в старом workflow
        result = _run(["e2e_timing.py", str(log), "voice"])
        assert result.returncode == 0
        assert "SUMMARY_MARKER T_total_ms" in result.stdout
