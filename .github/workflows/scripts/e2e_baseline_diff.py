#!/usr/bin/env python3
"""e2e_baseline_diff.py — сравнивает recording.wav с golden-эталоном.

Зачем (issue #1396): ретро-инженер должен видеть, отличается ли запись e2e
от «золотой» записи (когда-то хороший прогон). Если diff > 3dB по RMS или
> 30% по длительности — это регрессия (микрофон отвалился, громкость
упала, что-то застряло в начале/конце).

Golden-файлы: .github/e2e/golden/<voice_text_slug>.wav (если положили).
Сравнение идёт по RMS и duration; если wav не задан — recording сравнивается
с baseline по synthetic_tone (-20dB 1s) чтобы хотя бы валидировать что
recording вообще не мёртвый.

Usage:
  python3 e2e_baseline_diff.py <recording.wav> <golden.wav> "voice text"
  # → выводит JSON {"rms_diff_db":..., "duration_diff_pct":..., "pass":bool, "reason":...}

Stdlib-only (wave/struct/math). Принимает golden=опц. — тогда сравнивается
c синтетическим baseline (1s -20dB sine) и reason="no_golden".

Подсчёт keyword-match: e2e_voice_test.sh рядом пишет transcript.json с
распознанной фразой. Опциональный 4-й параметр — путь к transcript.json —
тогда добавится keyword_match (простая проверка contains expected keyword).
"""
from __future__ import annotations

import json
import math
import re
import struct
import sys
import wave
from pathlib import Path

RMS_DIFF_FAIL_DB = 3.0           # > 3dB RMS diff → FAIL
DURATION_DIFF_FAIL_PCT = 30.0    # > 30% по длительности → FAIL
KEYWORD_MATCH_MIN_PCT = 80.0     # < 80% keywords found → FAIL

# Дефолтный «baseline» если golden не задан: синтетический -20dB sine 1s.
# Сравнение — информативное (recording должен быть ГРОМЧЕ -20dB в норме),
# но pass=True если recording не пустой (rms > -50dB FS — mic_working).
DEFAULT_BASELINE_RMS_DBFS = -23.0
DEFAULT_BASELINE_DURATION_S = 1.0


def _read_mono(wav_path: Path) -> tuple[list[int], int, int]:
    """(samples_int16_mono, sample_rate, n_channels). Raises on error."""
    with wave.open(str(wav_path), "rb") as w:
        sr = w.getframerate()
        n_ch = w.getnchannels()
        sw = w.getsampwidth()
        n = w.getnframes()
        raw = w.readframes(n)
    if sw != 2:
        raise ValueError(f"only int16 supported (got {sw} bytes/sample)")
    fmt = f"<{n_ch * n}h"
    all_s = struct.unpack(fmt, raw[:struct.calcsize(fmt)])
    if n_ch > 1:
        return [all_s[i] for i in range(0, len(all_s), n_ch)], sr, n_ch
    return list(all_s), sr, n_ch


def _rms_dbfs(samples: list[int]) -> float:
    if not samples:
        return float("-inf")
    sq = sum(s * s for s in samples)
    rms = (sq / len(samples)) ** 0.5
    if rms <= 0:
        return float("-inf")
    return 20.0 * math.log10(rms / 32767.0)


def _duration_s(wav_path: Path) -> float:
    with wave.open(str(wav_path), "rb") as w:
        n = w.getnframes()
        sr = w.getframerate()
    return n / float(sr) if sr > 0 else 0.0


def _keyword_match_pct(expected: list[str], recognized: str) -> float:
    if not expected:
        return 100.0
    text = (recognized or "").lower()
    hits = sum(1 for kw in expected if kw and kw.lower() in text)
    return 100.0 * hits / len(expected)


def _slug_from_voice_text(voice_text: str) -> str:
    """Русский текст → ascii-safe slug для поиска golden-файла."""
    s = voice_text.lower().strip()
    # Простейший транслит для кириллицы (без зависимостей)
    table = str.maketrans({
        "а": "a", "б": "b", "в": "v", "г": "g", "д": "d", "е": "e", "ё": "yo",
        "ж": "zh", "з": "z", "и": "i", "й": "y", "к": "k", "л": "l", "м": "m",
        "н": "n", "о": "o", "п": "p", "р": "r", "с": "s", "т": "t", "у": "u",
        "ф": "f", "х": "h", "ц": "ts", "ч": "ch", "ш": "sh", "щ": "sch",
        "ъ": "", "ы": "y", "ь": "", "э": "e", "ю": "yu", "я": "ya", " ": "_",
    })
    slug = s.translate(table)
    slug = re.sub(r"[^a-z0-9_]+", "", slug)[:80] or "default"
    return slug


def find_golden(voice_text: str, golden_dir: Path) -> Path | None:
    """Ищет golden .wav для данной voice_text. Slug матчится *.wav."""
    slug = _slug_from_voice_text(voice_text)
    # Точное совпадение <slug>.wav или с префиксом индекса
    for candidate in [golden_dir / f"{slug}.wav"]:
        if candidate.exists():
            return candidate
    # Любой .wav (fallback)
    wavs = sorted(golden_dir.glob("*.wav"))
    return wavs[0] if wavs else None


def compare(
    recording: Path,
    golden: Path | None,
    voice_text: str,
    expected_keywords: list[str] | None = None,
    recognized: str = "",
) -> dict:
    """Главный вход: вернёт dict с rms_diff_db, duration_diff_pct, pass, reason."""
    if not recording.exists():
        return {"pass": False, "reason": f"recording not found: {recording}",
                "recording_wav": str(recording)}

    rec_rms = _rms_dbfs(_read_mono(recording)[0])
    rec_dur = _duration_s(recording)

    if golden is None or not golden.exists():
        # «Synthetic» baseline: recording должен быть громче -50dB FS
        # (mic_working). Pass если хоть не мёртвый.
        return {
            "recording_wav": str(recording),
            "golden_wav": None,
            "reason": "no_golden",
            "recording_rms_dbfs": round(rec_rms, 2) if rec_rms != float("-inf") else None,
            "recording_duration_s": round(rec_dur, 3),
            "baseline_rms_dbfs": DEFAULT_BASELINE_RMS_DBFS,
            "baseline_duration_s": DEFAULT_BASELINE_DURATION_S,
            "rms_diff_db": None,
            "duration_diff_pct": None,
            "pass": rec_rms > -50.0,
            "pass_detail": "no_golden: mic_working threshold check only",
        }

    gold_rms = _rms_dbfs(_read_mono(golden)[0])
    gold_dur = _duration_s(golden)

    rms_diff = abs(rec_rms - gold_rms) if (rec_rms != float("-inf") and gold_rms != float("-inf")) else None
    dur_diff = abs(rec_dur - gold_dur) / gold_dur * 100 if gold_dur > 0 else None

    failures = []
    if rms_diff is not None and rms_diff > RMS_DIFF_FAIL_DB:
        failures.append(f"rms_diff {rms_diff:.2f}dB > {RMS_DIFF_FAIL_DB}dB")
    if dur_diff is not None and dur_diff > DURATION_DIFF_FAIL_PCT:
        failures.append(f"duration_diff {dur_diff:.1f}% > {DURATION_DIFF_FAIL_PCT}%")

    keyword_match = _keyword_match_pct(expected_keywords or [], recognized)
    if expected_keywords and keyword_match < KEYWORD_MATCH_MIN_PCT:
        failures.append(
            f"keyword_match {keyword_match:.1f}% < {KEYWORD_MATCH_MIN_PCT}% "
            f"(expected={expected_keywords}, recognized='{recognized}')"
        )

    return {
        "recording_wav": str(recording),
        "golden_wav": str(golden),
        "recording_rms_dbfs": round(rec_rms, 2) if rec_rms != float("-inf") else None,
        "golden_rms_dbfs": round(gold_rms, 2) if gold_rms != float("-inf") else None,
        "rms_diff_db": round(rms_diff, 2) if rms_diff is not None else None,
        "rms_diff_threshold_db": RMS_DIFF_FAIL_DB,
        "recording_duration_s": round(rec_dur, 3),
        "golden_duration_s": round(gold_dur, 3),
        "duration_diff_pct": round(dur_diff, 2) if dur_diff is not None else None,
        "duration_diff_threshold_pct": DURATION_DIFF_FAIL_PCT,
        "keyword_match_pct": round(keyword_match, 2) if expected_keywords else None,
        "keyword_match_min_pct": KEYWORD_MATCH_MIN_PCT if expected_keywords else None,
        "expected_keywords": expected_keywords,
        "recognized": recognized,
        "pass": not failures,
        "reason": "; ".join(failures) if failures else "all checks passed",
    }


def main(argv: list[str]) -> int:
    # argv: [recording.wav, golden.wav(opt), voice_text, transcript.json(opt)]
    if len(argv) < 2:
        print(json.dumps({"error": "usage: e2e_baseline_diff.py <recording.wav> [golden.wav] 'voice text' [transcript.json]"}))
        return 0  # диагностика, не fail

    recording = Path(argv[1])
    golden: Path | None = Path(argv[2]) if len(argv) > 2 and argv[2] else None
    voice_text = argv[3] if len(argv) > 3 else ""

    expected_keywords: list[str] = []
    recognized = ""
    if len(argv) > 4 and argv[4]:
        tp = Path(argv[4])
        if tp.exists():
            try:
                tj = json.loads(tp.read_text(encoding="utf-8"))
                expected_keywords = tj.get("acceptance", {}).get("expected_keywords", []) \
                    or tj.get("expected_keywords", [])
                recognized = tj.get("recognized", "") or tj.get("text", "")
            except Exception:
                pass

    # Если golden не задан — попробуем найти в golden/ по slug голосовой команды
    if golden is None and voice_text:
        for cand in (Path(".github/e2e/golden"),
                     Path("/tmp/e2e_golden"),
                     Path("./golden")):
            if cand.is_dir():
                gold_found = find_golden(voice_text, cand)
                if gold_found:
                    golden = gold_found
                    break

    result = compare(recording, golden, voice_text, expected_keywords, recognized)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
