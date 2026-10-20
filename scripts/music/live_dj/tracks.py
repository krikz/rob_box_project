"""Потрековая сводка сета: лог + запись.

python tracks.py setN.full.log rN.wav "2026-10-01T12:20:00Z"
(третий аргумент — старт записи из index.txt)

Лог движка v2 (ADR-0149): трек — строка ``[music v2] started track_id=… bpm=… deck=…`` (время — ROS-штамп),
источник (колонка «сэмпл») — ``[set v2] … started … source=…``, хуки/тон — ``[reasoner] … план применён``;
не вызов ``compose_music``; при блэнде (PR-8) отрезок трека — от его входа до входа следующего.
"""
import calendar
import re
import sys
import time

import numpy as np

import audit_wav as A
import v2log as V

TS = re.compile(r"\[(\d{10})\.(\d+)\]")


def grid_R(flux, fs, bpm, s, e, step=4.0):
    P = 60 / bpm / 4
    out = []
    from scipy.signal import find_peaks
    for t in np.arange(s, e - step + 0.01, step):
        x = flux[int(t * fs):int((t + step) * fs)]
        if len(x) < 10:
            continue
        p, _ = find_peaks(x, height=np.percentile(x, 85), distance=int(fs * 0.05))
        if len(p) < 6:
            continue
        tt = p / fs + t
        out.append(abs(np.exp(2j * np.pi * tt / P).mean()))
    return np.median(out) if out else float("nan")


def v2_started(line, t_rec):
    """Трек движка v2 из строки ``[music v2] started`` или ``None``."""
    r = V.parse_started(line)
    if not r:
        return None
    return {"t": r["t_abs"] - t_rec, "bpm": r["bpm"], "sample": "—", "hook": "—", "id": r["track_id"],
            "kit": f"v2 {r['track_id']} {r['deck']}", "key": ""}


def main():
    log, wav, start = sys.argv[1:4]
    t_rec = calendar.timegm(time.strptime(start, "%Y-%m-%dT%H:%M:%SZ"))
    tracks, cur, plan, sources = [], None, None, {}
    for line in open(log, encoding="utf-8", errors="replace"):
        m = TS.search(line)
        t = float(m.group(1)) if m else None
        sv = V.parse_set_started(line)
        if sv:
            sources[sv[0]] = sv[1]
            for tr in tracks:
                if tr.get("id") == sv[0]:
                    tr["sample"] = sv[1]
            continue
        pl = V.parse_plan(line)
        if pl:
            plan = pl
            continue
        v2 = v2_started(line, t_rec)
        if v2:
            v2["sample"] = sources.get(v2["id"], "—")
            if plan:
                v2["key"] = f"{plan['row']} {plan['mode']}"
                v2["hook"] = plan["hooks"][:22]
            cur = v2
            tracks.append(cur)
            continue
        if "Запрос выполнения: compose_music" in line and t:
            cur = {"t": t - t_rec, "bpm": None, "sample": "—", "hook": "—", "kit": "", "key": ""}
            tracks.append(cur)
            k = re.search(r"'root': '([^']+)', 'scale': '([^']+)'", line)
            if k:
                cur["key"] = f"{k.group(1)} {k.group(2)}"
        if cur is None:
            continue
        m = re.search(r"Clock\.bpm = (\d+)", line)
        if m and cur["bpm"] is None:
            cur["bpm"] = int(m.group(1))
        m = re.search(r"club хук: (\S+ \S+)", line)
        if m:
            cur["hook"] = m.group(1)
        m = re.search(r"club выбор: .*?template=(\S+), kick=(\S+), hats=(\S+),.*?sample=(\S+?)[;,]", line)
        if m:
            cur["kit"] = f"{m.group(1)}/{m.group(2)}/{m.group(3)}"
            cur["sample"] = m.group(4)
    a, sr = A.load(wav)
    dur = len(a) / sr
    flux, fs = A.onsets(a, sr)
    print(f"{'старт,с':>7} {'bpm':>4} {'тон.':9} {'каркас':34} {'сэмпл':14} {'хук':22} {'R':>5} {'вне-лада':>8}")
    for i, tr in enumerate(tracks):
        s = tr["t"] + 3
        e = min(tracks[i + 1]["t"] if i + 1 < len(tracks) else dur, dur)
        if e - s < 8 or not tr["bpm"]:
            print(f"{tr['t']:7.0f} {tr['bpm'] or '?':>4} (слишком коротко в записи / нет bpm)")
            continue
        R = grid_R(flux, fs, tr["bpm"], s, e)
        oos = np.median([A.out_of_scale(A.chroma(a[int(x * sr):int((x + 8) * sr)], sr))
                         for x in np.arange(s, e - 8 + 0.01, 8)] or [np.nan])
        print(f"{tr['t']:7.0f} {tr['bpm']:>4} {tr['key']:9} {tr['kit']:34} {tr['sample']:14} "
              f"{tr['hook']:22} {R:5.2f} {oos:8.2f}")


if __name__ == "__main__":
    main()
