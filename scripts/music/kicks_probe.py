#!/usr/bin/env python3
"""kicks_probe.py — разбор записей kicks_probe.sh (ADR-0152 PR-4): по каждому wav доли энергии < 250 и < 120 Гц,
> 2 кГц (щелчок), RMS и пик, поправка unit-уровня (громче эталона — больше) против ``kick_12.wav``. Хост с numpy.

    python scripts/music/kicks_probe.py kicks_probe/kick_*.wav [--ref kick_12.wav]
"""
from __future__ import annotations

import argparse
import math
import sys
import wave
from pathlib import Path

import numpy as np

#: Поправка эталона ``X:12`` из ``knowledge.KICK_SOUNDS`` (замер 02.10): поправки кандидатов считаются от неё.
REF_OFFSET_DB = 0.98


def load(path: Path):
    with wave.open(str(path)) as w:
        raw = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64) / 32768.0
        return raw.reshape(-1, w.getnchannels()).mean(axis=1), w.getframerate()


def band_shares(x: np.ndarray, rate: int):
    power = np.abs(np.fft.rfft(x)) ** 2
    freq = np.fft.rfftfreq(len(x), 1 / rate)
    total = power.sum() or 1.0
    return power[freq < 250].sum() / total, power[freq < 120].sum() / total, power[freq > 2000].sum() / total


def db(v: float) -> float:
    return -180.0 if v <= 0 else round(20 * math.log10(v), 1)


def measure(path: Path) -> dict:
    x, rate = load(path)
    low, sub, click = band_shares(x, rate)
    return {"file": path.name, "low250": round(float(low), 3), "sub120": round(float(sub), 3),
            "hi2k": round(float(click), 3), "rms_db": db(float(np.sqrt(np.mean(x ** 2)))),
            "peak_db": db(float(np.abs(x).max()))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("wavs", nargs="+", type=Path)
    ap.add_argument("--ref", default="kick_12.wav")
    args = ap.parse_args(argv)
    rows = [measure(p) for p in args.wavs]
    ref = next((r for r in rows if r["file"] == args.ref), None)
    print(f"{'file':14} {'<250':>6} {'<120':>6} {'>2k':>6} {'rms':>7} {'peak':>7} {'offset_db':>9}")
    for r in rows:
        offset = None if ref is None else round(REF_OFFSET_DB + r["rms_db"] - ref["rms_db"], 2)
        print(f"{r['file']:14} {r['low250']:6.3f} {r['sub120']:6.3f} {r['hi2k']:6.3f} {r['rms_db']:7.1f} "
              f"{r['peak_db']:7.1f} {offset!s:>9}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
