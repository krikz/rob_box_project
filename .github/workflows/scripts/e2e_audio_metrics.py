#!/usr/bin/env python3
"""e2e_audio_metrics.py — вычисляет RMS/peak/silence_ratio для recording.wav.

Вызывается после e2e-цикла чтобы ретро-инженер видел:
  - работал ли микрофон вообще (RMS > -50 dB FS = да)
  - насколько громкая запись (peak в dB FS)
  - silence_ratio: доля секунд с RMS < -40 dB FS (молчание/артефакты)

Использует stdlib `wave` + `struct` (soundfile на билдовой машине может
не быть, не хочу плодить зависимости). Двухканал/моно — оба поддерживаются;
проценты рассчитываются по СЕКУНДАМ (окно 100ms).

Usage:
  python3 e2e_audio_metrics.py /path/to/recording.wav > audio_metrics.json
  # путь опционально: если не задан — ищет последний /tmp/e2e_v2_*/recording.wav
  # или /tmp/dialog_e2e_*.wav

Output: JSON {"rms_dbfs":..., "peak_dbfs":..., "silence_ratio":...,
             "duration_s":..., "mic_working":..., "samples_analyzed":N,
             "ffmpeg_ebur128":{...опц...}}

Stdlib-only: PEP 668 не нарушается.
"""
from __future__ import annotations

import json
import os
import re
import struct
import subprocess
import sys
import wave
from pathlib import Path

# Константы калибровки (документированы в ADR-0015 / VOICE_COMMANDS_RESEARCH.md):
MIC_WORKING_THRESHOLD_DB = -50.0   # RMS > -50dB FS → микрофон работал
SILENCE_THRESHOLD_DB = -40.0       # RMS < -40dB FS в этом окне → тишина
WINDOW_MS = 100                     # анализ по 100ms окнам (10 окон/сек)


def _latest_wav() -> Path | None:
    """Best-effort: найти последний recording.wav."""
    candidates: list[Path] = []
    for pattern in ("/tmp/e2e_v2_*/recording.wav", "/tmp/dialog_e2e_*.wav",
                    "/tmp/recording.wav", "/tmp/voice_e2e_*.wav"):
        for p in Path("/tmp").glob(pattern):
            candidates.append(p)
    # Также ищем под симлинками (часто dialog_e2e_X.wav → e2e_v2_X/recording.wav)
    if not candidates:
        return None
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def _rms_dbfs(samples: list[int]) -> float:
    """RMS → dB FS (full scale). int16 диапазон ±32767 → 0 dB FS."""
    import math
    if not samples:
        return float("-inf")
    sq_sum = sum(s * s for s in samples)
    rms = (sq_sum / len(samples)) ** 0.5
    if rms <= 0:
        return float("-inf")
    return 20.0 * math.log10(rms / 32767.0)


def _peak_dbfs(samples: list[int]) -> float:
    import math
    if not samples:
        return float("-inf")
    peak = max(abs(s) for s in samples)
    if peak <= 0:
        return float("-inf")
    return 20.0 * math.log10(peak / 32767.0)


def compute_metrics(wav_path: Path) -> dict:
    """Читает wav, считает RMS+peak по 100ms окнам → метрики."""
    if not wav_path.exists():
        return {"error": f"file not found: {wav_path}", "wav_path": str(wav_path)}

    try:
        with wave.open(str(wav_path), "rb") as w:
            n_ch = w.getnchannels()
            sr = w.getframerate()
            sw = w.getsampwidth()
            n_frames = w.getnframes()
            raw = w.readframes(n_frames)
    except Exception as exc:
        return {"error": f"wave open failed: {exc}", "wav_path": str(wav_path)}

    duration_s = n_frames / float(sr) if sr > 0 else 0.0
    if sw != 2:
        # Расширение для 32-bit float и пр. — пока int16 (микрофон робота —
        # 16-bit, проверял в e2e_remote.sh: parec --format=s16le).
        return {
            "error": f"unsupported sample width {sw} bytes (only int16 supported)",
            "wav_path": str(wav_path),
        }

    fmt = f"<{n_ch * (n_frames)}h"
    if len(raw) < struct.calcsize(fmt):
        return {"error": f"truncated wav ({len(raw)} bytes)", "wav_path": str(wav_path)}
    all_samples = struct.unpack(fmt, raw[:struct.calcsize(fmt)])
    if n_ch > 1:
        mono = [all_samples[i] for i in range(0, len(all_samples), n_ch)]
    else:
        mono = list(all_samples)

    win = int(sr * WINDOW_MS / 1000)
    if win <= 0:
        return {"error": f"window too small (sr={sr})", "wav_path": str(wav_path)}

    silent_windows = 0
    total_windows = 0
    rms_list = []
    peak_db = float("-inf")
    for i in range(0, len(mono), win):
        chunk = mono[i : i + win]
        if not chunk:
            break
        total_windows += 1
        rms_db = _rms_dbfs(chunk)
        rms_list.append(rms_db)
        if rms_db < SILENCE_THRESHOLD_DB:
            silent_windows += 1
        chunk_peak = _peak_dbfs(chunk)
        if chunk_peak > peak_db:
            peak_db = chunk_peak

    overall_rms_db = _rms_dbfs(mono)
    if rms_list:
        loud_rms_db = max(rms_list)
        quiet_rms_db = min(r for r in rms_list if r != float("-inf"))
    else:
        loud_rms_db = quiet_rms_db = overall_rms_db

    silence_ratio = silent_windows / total_windows if total_windows > 0 else 0.0
    mic_working = overall_rms_db > MIC_WORKING_THRESHOLD_DB

    return {
        "wav_path": str(wav_path),
        "duration_s": round(duration_s, 3),
        "sample_rate_hz": sr,
        "channels": n_ch,
        "samples_analyzed": len(mono),
        "windows_analyzed": total_windows,
        "rms_dbfs": round(overall_rms_db, 2),
        "loud_window_rms_dbfs": round(loud_rms_db, 2) if loud_rms_db != float("-inf") else None,
        "quiet_window_rms_dbfs": round(quiet_rms_db, 2) if quiet_rms_db != float("-inf") else None,
        "peak_dbfs": round(peak_db, 2) if peak_db != float("-inf") else None,
        "silence_ratio": round(silence_ratio, 4),
        "silence_threshold_db": SILENCE_THRESHOLD_DB,
        "window_ms": WINDOW_MS,
        "mic_working": mic_working,
        "mic_threshold_db": MIC_WORKING_THRESHOLD_DB,
    }


def ffmpeg_ebur128(wav_path: Path) -> dict | None:
    """Опционально: ffmpeg ebur128 (Loudness Units относительно FS)."""
    if not wav_path.exists():
        return None
    try:
        # Подавляем вывод ffmpeg, нам нужен только парсинг stderr (ebur128 туда пишет)
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-i", str(wav_path),
             "-af", "ebur128=peak=true", "-f", "null", "-"],
            capture_output=True, text=True, timeout=30,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    out = (result.stderr or "") + (result.stdout or "")
    summ = re.search(r"Integrated loudness:\s+I:\s*(-?[\d.]+)\s*LUFS", out)
    range_ = re.search(r"Loudness range:\s*LRA:\s*([\d.]+)\s*LU", out)
    peak = re.search(r"Peak level:\s*(-?[\d.]+)\s*dBFS", out)
    if not summ:
        return None
    return {
        "lufs": float(summ.group(1)) if summ else None,
        "lra_lu": float(range_.group(1)) if range_ else None,
        "peak_dbfs": float(peak.group(1)) if peak else None,
        "tool": "ffmpeg-ebur128",
    }


def main(argv: list[str]) -> int:
    if len(argv) > 1 and not argv[1].startswith("-"):
        path = Path(argv[1])
    else:
        path = _latest_wav()
        if path is None:
            print(json.dumps({"error": "no recording.wav found in /tmp/e2e_v2_* or /tmp/dialog_e2e_*"}),
                  file=sys.stdout)
            return 0  # НЕ падаем — это просто диагностика, без wav метрик нет

    metrics = compute_metrics(path)
    if "error" not in metrics and path is not None:
        ebur = ffmpeg_ebur128(path)
        if ebur is not None:
            metrics["ffmpeg_ebur128"] = ebur
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
