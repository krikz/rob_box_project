#!/usr/bin/env python3
"""Спайк PR-S1 #3709 «речь в такт»: 4 куплета × 3 голоса × подходы → WAV/OGG 16 кГц, замер A19, слепые A/B.

Запуск (Windows, Git Bash; нужны numpy, scipy, soundfile, librosa, httpx, ffmpeg с rubberband)::

    PYTHONUTF8=1 python scripts/dialog/rap_sync/run_spike.py --out <каталог> --env-file scripts/dialog/.env

Синтезы кэшируются в ``<out>/cache`` (``ledger.jsonl`` — каждый настоящий вызов API). Результат: ``renders/`` —
все сведения, ``ab/pairN_{A,B}.{wav,ogg}`` + ``ab_key.json`` (ключ — рядом, не в ``ab/``: Шифу слушает вслепую), ``metrics.json``.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import approaches as ap  # noqa: E402
from beat import engine_loop, render_beat  # noqa: E402
from common import SR, VERSES, VOICES, read_mono, syllables, tts_text, write_audio  # noqa: E402
from metric import a19, detector_check, flux_onsets, quality, syllable_onsets  # noqa: E402
from tts_minimax import MiniMaxTTS, load_key  # noqa: E402

ENGINE_SEED = 3
FILES = ((0, 1), (2, 3))
#: Слепые пары: каждая альтернатива против текущего кода (1); голос и куплеты разные.
PAIRS = (("q", "q_lines", 0, 0), ("q", "fit", 1, 1), ("q", "stretch", 2, 0), ("q", "markup", 0, 1))
AB_MAX_S = 30.0
APPROACHES = ("q", "q_lines", "fit", "stretch", "stretch_rb", "markup1", "markup")


def pool(rows: List[Dict]) -> Dict[str, float]:
    keys = ("p50_ms", "p95_ms", "p50_norm", "p95_norm", "R16", "R8", "C16", "bias_ms", "line_p50_ms", "line_p95_ms",
            "seconds")
    return {k: round(float(np.median([r[k] for r in rows])), 3) for k in keys}


def null_fit(counts: List[int], durs: List[float], rng: np.random.Generator, trials: int = 20) -> Dict[str, float]:
    """Случайные онсеты той же плотности → подбор темпа (2) → R16 тех же онсетов с дрожью детектора 35 мс."""
    rs, r_fixed = [], []
    for _ in range(trials):
        ons = [np.sort(rng.uniform(0.15, d, n)) for n, d in zip(counts, durs)]
        bpm, _ = ap.fit_bpm(ons)
        placed = ap.beat_to_line([np.zeros(int(d * SR)) for d in durs], ons, bpm)
        allon = np.concatenate([s + o + rng.normal(0, 0.035, len(o)) for (s, _), o in zip(placed, ons)])
        rs.append(a19(allon, 0.0, bpm)["R16"])
        r_fixed.append(a19(np.concatenate([s + o for (s, _), o in zip(ap.quantize(
            [np.zeros(int(d * SR)) for d in durs], 131.0), ons)]), 0.0, 131.0)["R16"])
    return {"fit_R16_mean": round(float(np.mean(rs)), 3), "fit_R16_p95": round(float(np.percentile(rs, 95)), 3),
            "fixed_R16_mean": round(float(np.mean(r_fixed)), 3)}


def synth_markup(tts: MiniMaxTTS, plain: Dict, bpm: float, prev: Dict = None) -> Tuple[Dict, Dict]:
    """Подход 4: синтез с паузами между строками; ``prev`` — замер прошлого прохода (калибровка пауз)."""
    marked, info = {}, {}
    for v in VOICES:
        for i in range(len(VERSES)):
            fix: List[float] = []
            if prev:  # поправка копится: прошлая поправка + ошибка шага строки прошлого прохода
                last = prev[f"{v}:{i}"]
                old = last["fix"] or [0.0] * len(last["line_step_err_ms"])
                fix = [f + e / 1000.0 for f, e in zip(old, last["line_step_err_ms"])]
            text, speed, n = ap.markup_text(VERSES[i], plain[(v, i)]["words"], bpm, fix=fix)
            marked[(v, i)] = tts.synth(v, text, speed)
            got = ap.line_cuts(marked[(v, i)]["words"], VERSES[i])
            err = [round((b - a - n * ap.bar_s(bpm)) * 1000) for a, b in zip(got[:-1], got[1:])]
            info[f"{v}:{i}"] = {"text": text, "speed": speed, "bars_per_line": n, "line_step_err_ms": err,
                                "fix": list(fix)}
    return marked, info


def build(tts: MiniMaxTTS, out: Path) -> Dict:
    eng_bpm, loop = engine_loop(ENGINE_SEED)
    plain = {(v, i): tts.synth(v, tts_text(VERSES[i])) for v in VOICES for i in range(len(VERSES))}
    audio = {k: read_mono(Path(r["wav"])) for k, r in plain.items()}
    ons = {k: syllable_onsets(x) for k, x in audio.items()}
    det = [dict(detector_check(ons[k], ap.vowel_times(plain[k]["words"])), syll=syllables(plain[k]["text"]),
                flux=len(flux_onsets(audio[k])), key=f"{k[0]}:{k[1]}") for k in plain]
    marked1, markup_info = synth_markup(tts, plain, eng_bpm)
    marked, markup_info2 = synth_markup(tts, plain, eng_bpm, markup_info)
    markup_info = {"pass1": markup_info, "pass2": markup_info2}
    rows: Dict[str, List[Dict]] = {a: [] for a in APPROACHES}
    extra: Dict[str, List] = {"stretch": [], "stretch_rb": [], "markup": [], "markup1": [], "fit_bpm": [], "prep_s": {}}
    prep: Dict[str, List[float]] = {a: [] for a in APPROACHES}
    for vi, v in enumerate(VOICES):
        for fi, idx in enumerate(FILES):
            xs = [audio[(v, i)] for i in idx]
            os_ = [ons[(v, i)] for i in idx]
            cuts = [ap.line_cuts(plain[(v, i)]["words"], VERSES[i]) for i in idx]
            for a in APPROACHES:
                t0, bpm, maps, lcuts = time.perf_counter(), eng_bpm, (), cuts
                if a == "q":
                    placed = ap.quantize(xs, bpm)
                elif a == "q_lines":
                    placed = ap.quantize_lines(xs, cuts, os_, bpm)
                elif a == "fit":
                    bpm, r = ap.fit_bpm(os_)
                    placed = ap.beat_to_line(xs, os_, bpm)
                    extra["fit_bpm"].append({"file": f"{v}:{idx}", "bpm": bpm, "R16_fit": round(r, 3)})
                elif a.startswith("stretch"):
                    placed, st = ap.stretch(xs, os_, bpm, "wsola" if a == "stretch" else "rubberband")
                    q = [quality(x, y) for x, (_, y) in zip(xs, placed)]
                    maps = st.pop("maps")
                    extra[a].append(dict(st, file=f"{v}:{idx}", quality=q))
                else:
                    src = marked if a == "markup" else marked1
                    ms = [read_mono(Path(src[(v, i)]["wav"])) for i in idx]
                    lcuts = [ap.line_cuts(src[(v, i)]["words"], VERSES[i]) for i in idx]
                    mo = [syllable_onsets(m) for m in ms]
                    placed = ap.quantize_lines(ms, [[0.0]] * len(ms), mo, bpm)
                    extra[a].append({"file": f"{v}:{idx}", "quality": [quality(x, m) for x, m in zip(xs, ms)]})
                prep[a].append(time.perf_counter() - t0)
                end = max(s + len(x) / SR for s, x in placed)
                beat = render_beat(loop, bpm, ap.next_bar(end, bpm) + ap.bar_s(bpm))
                mixed, voc = ap.mix(placed, beat)
                lines_at = ap.expected_lines(a, placed, lcuts, maps)
                m = dict(a19(flux_onsets(voc), 0.0, bpm, lines_at), voice=v, file=fi, seconds=round(len(mixed) / SR, 1))
                rows[a].append(m)
                write_audio(out / "renders" / f"{a}_{vi}_{fi}.wav", mixed, ogg=False)
    extra["prep_s"] = {a: round(float(np.median(p)), 3) for a, p in prep.items()}
    extra["synth_s_median"] = round(float(np.median([r["synth_s"] for r in plain.values()])), 2)
    counts = [len(ons[(v, i)]) for v in VOICES for i in (0, 1)]
    durs = [len(audio[(v, i)]) / SR for v in VOICES for i in (0, 1)]
    null = null_fit(counts[:2], durs[:2], np.random.default_rng(0))
    return {"engine_bpm": eng_bpm, "engine_seed": ENGINE_SEED, "detector": det, "markup": markup_info,
            "rows": rows, "pooled": {a: pool(r) for a, r in rows.items()}, "extra": extra, "null": null,
            "synth_calls": tts.calls()}


def blind_pairs(out: Path) -> List[Dict]:
    from common import read_mono as rd

    flips = [True, True, False, False]  # текущий код (1) — «A» ровно в двух парах из четырёх, какие — случайно
    random.Random(3709).shuffle(flips)
    key = []
    for n, ((a, b, vi, fi), flip) in enumerate(zip(PAIRS, flips), 1):
        order = [a, b] if flip else [b, a]
        for side, appr in zip("AB", order):
            x = rd(out / "renders" / f"{appr}_{vi}_{fi}.wav")[:int(AB_MAX_S * SR)]
            x[-SR // 2:] *= np.linspace(1, 0, SR // 2, dtype=np.float32)  # конец ≤ 30 с — затухание 0.5 с
            write_audio(out / "ab" / f"pair{n}_{side}.wav", x)
        key.append({"pair": n, "A": order[0], "B": order[1], "voice": VOICES[vi], "verses": list(FILES[fi])})
    (out / "ab_key.json").write_text(json.dumps(key, ensure_ascii=False, indent=1), encoding="utf-8")
    return key


def main() -> int:
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--out", required=True)
    ap_.add_argument("--env-file")
    args = ap_.parse_args()
    out = Path(args.out)
    tts = MiniMaxTTS(out / "cache", load_key(args.env_file))
    res = build(tts, out)
    res["pairs"] = blind_pairs(out)
    (out / "metrics.json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({"pooled": res["pooled"], "markup": res["markup"], "prep_s": res["extra"]["prep_s"], "null": res["null"],
                      "fit_bpm": res["extra"]["fit_bpm"], "synth_calls": res["synth_calls"]},
                     ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
