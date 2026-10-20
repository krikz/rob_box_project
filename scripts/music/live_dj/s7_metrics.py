#!/usr/bin/env python3
"""ADR-0153 S7: метрики записей стилей ТЕМ ЖЕ методом, что профили эталонов (07.10, ``style_profile.py``).

Запись ``s7_<ключ>.wav`` (``s7_styles.sh``) режется с начала трека 1 на ``--seconds`` (старт — первый звук записи,
сверяется с ROS-штампом ``[music v2] started`` в ``<ключ>.full.log``), декодируется ffmpeg в 16 кГц стерео s16 (как
``prep.sh`` эталонов), окна 20 с считаются функцией ``win_metrics`` эталонов (полосы низ <150 / середина / верх,
crest, LR, side, онсеты, темп тремя методами), свинг — ``swing_from`` эталонов, но с темпом из лога робота (ADR-0153
§5 S2, вывод отчёта эталонов: авто-темп ненадёжен). Громкость: σ по минутам, ebur128 (LUFS/LRA), пик.

Код метода эталонов лежит вне git (``artifacts_live_check_2026-10-07/style_refs``) — путь ``--method``; без него
скрипт не считает, а падает (иначе «тот же метод» — неправда).

    python scripts/music/live_dj/s7_metrics.py <каталог записей> --method <style_refs> --json out.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import wave

import numpy as np

#: Эталон стиля (ключ в ``summary.json`` профилей); нет — None (честно «эталона нет»).
REFS = {
    "club": "club_ref_dj",
    "rock": "rock70-90",
    "grunge": "nirvana",
    "jazz": "jazz_cafe",
    "lofi": "dark_trip",
}
#: Оси различимости: (ключ, подпись).
AXES = (
    ("bpm", "темп лога"),
    ("swing_q", "свинг четвертью"),
    ("low", "низ"),
    ("mid", "середина"),
    ("high", "верх"),
    ("crest", "crest"),
    ("lr", "LR"),
    ("ons_s", "онсеты/с"),
    ("cent", "центроид"),
)


def read_wav(path):
    with wave.open(path) as w:
        sr, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
        x = np.frombuffer(w.readframes(n), dtype="<i2").astype(np.float32) / 32768
    return sr, x.reshape(-1, ch)


def first_sound(x, sr, thr_db=-60.0):
    """Секунда первого звука: первое окно 10 мс громче ``thr_db`` dBFS."""
    m = x.mean(1)
    hop = sr // 100
    for i in range(0, len(m) - hop, hop):
        if 20 * math.log10(np.sqrt(np.mean(m[i : i + hop] ** 2)) + 1e-12) > thr_db:
            return i / sr
    return None


def parse_log(path):
    txt = open(path, encoding="utf-8", errors="replace").read()
    out = {"tracks": []}
    started = (
        r"\[[A-Z]+\] \[(\d+\.\d+)\] \[mcp_server\]: \S+ \[music v2\] started (track_id=\S+)"
        r".*?late_beats=([\d.]+) bpm=([\d.]+)"
    )
    for m in re.finditer(started, txt):
        out["tracks"].append(
            dict(
                stamp=float(m.group(1)),
                track_id=m.group(2)[9:],
                late_beats=float(m.group(3)),
                bpm=float(m.group(4)),
            )
        )
    comps = re.findall(r"трек (\d+)(?:/\d+)? started .*?composition=(\{.*\})", txt)
    for no, js in comps:
        i = int(no) - 1
        if i < len(out["tracks"]):
            try:
                out["tracks"][i]["composition"] = json.loads(js)
            except ValueError:
                out["tracks"][i]["composition_raw"] = js[:400]
    out["stt"] = re.findall(r"STT: (.*)", txt)[:4]
    out["rejected"] = len(re.findall(r"\[music v2\] rejected", txt))
    out["dj_auto"] = txt.count("DJ_AUTO")
    return out


def ebur(path):
    r = subprocess.run(
        ["ffmpeg", "-nostats", "-i", path, "-af", "ebur128", "-f", "null", "-"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    s = r.stderr.split("Summary:")[-1]
    i = re.search(r"I:\s+(-?[\d.]+) LUFS", s)
    lra = re.search(r"LRA:\s+([\d.]+) LU", s)
    return (float(i.group(1)) if i else None), (float(lra.group(1)) if lra else None)


def profile(sp, wav16, bpm, win=20.0):
    import librosa  # noqa: WPS433 (тяжёлый импорт только здесь)

    sr, x = read_wav(wav16)
    assert sr == 16000, sr
    m = x.mean(1)
    nsec = len(m) // sr
    sec_db = 10 * np.log10((m[: nsec * sr].reshape(nsec, sr) ** 2).mean(1) + 1e-12)
    minutes = [
        float(10 * np.log10((10 ** (sec_db[i : i + 60] / 10)).mean()))
        for i in range(0, nsec - 59, 60)
    ]
    act = sec_db[sec_db > -55]
    rows = []
    for s in np.arange(0, nsec - win + 1e-9, win):
        xs = x[int(s * sr) : int((s + win) * sr)]
        w = sp.win_metrics(xs, sr)
        # онсеты как в win_metrics, свинг с темпом лога: div=1 — пульс как у эталонов (сложен в 90–180),
        # div=2 — вдвое медленнее (для bpm < 90 — четверть, иначе половинная)
        mm = xs.mean(1)
        hop = int(sr * 0.008)
        env = librosa.onset.onset_strength(
            y=mm.astype(np.float32), sr=sr, hop_length=hop
        )
        on = librosa.onset.onset_detect(
            onset_envelope=env, sr=sr, hop_length=hop, units="frames"
        )
        tl, wl = on / (sr / hop), (env[on] if len(on) else np.array([]))
        for div, key in ((1, "swing_log"), (2, "swing_log_half")):
            r = sp.swing_from(tl, wl, bpm, div=div)
            if r:
                w[key], w[key + "_m"] = r[0], r[1]
        w["swing_q"] = (
            w.get("swing_log_half") if bpm < 90 else w.get("swing_log")
        )  # пульс = четверть по логу
        w["t"] = float(s)
        rows.append(w)
    peak = float(20 * np.log10(np.abs(x).max() + 1e-12))
    lufs, lra = ebur(wav16)

    def med(k):
        v = [r[k] for r in rows if r.get(k) is not None and r[k] == r[k]]
        return (
            (float(np.median(v)), float(np.min(v)), float(np.max(v)), len(v))
            if v
            else None
        )

    agg = {
        k: med(k)
        for k in (
            "low",
            "mid",
            "high",
            "sub",
            "crest",
            "lr",
            "side_db",
            "ons_s",
            "cent",
            "db",
            "t_cons_fold",
            "swing",
            "swing_log",
            "swing_log_half",
            "swing_q",
        )
    }
    agg.update(
        peak_dbfs=peak,
        clips=int((np.abs(x) >= 0.999).sum()),
        lufs=lufs,
        lra=lra,
        minutes_db=minutes,
        min_sigma=float(np.std(minutes)) if minutes else None,
        min_range=float(max(minutes) - min(minutes)) if minutes else None,
        sec_sigma=float(act.std()) if len(act) else None,
        silent_s=int((sec_db <= -55).sum()),
        tempo_agree=float(np.mean([r["t_raw_agree"] for r in rows])),
        n_windows=len(rows),
    )
    return agg, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument(
        "--method",
        required=True,
        help="каталог style_refs (style_profile.py, summary.json)",
    )
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--json")
    a = ap.parse_args()
    sys.path.insert(0, a.method)
    import style_profile as sp  # noqa: E402  — метод эталонов как есть

    refs = json.load(open(os.path.join(a.method, "summary.json"), encoding="utf-8"))
    res = {}
    for name in sorted(os.listdir(a.folder)):
        mt = re.fullmatch(r"s7_(\w+)\.wav", name)
        if not mt:
            continue
        key = mt.group(1)
        path = os.path.join(a.folder, name)
        log = parse_log(os.path.join(a.folder, f"{key}.full.log"))
        if not log["tracks"]:
            res[key] = dict(error="нет [music v2] started в логе", log=log)
            print(key, "нет трека в логе", flush=True)
            continue
        sr, x = read_wav(path)
        t0 = first_sound(x, sr)
        bpm = log["tracks"][0]["bpm"]
        with tempfile.TemporaryDirectory() as td:
            w16 = os.path.join(td, f"{key}_16k.wav")
            subprocess.run(
                [
                    "ffmpeg",
                    "-v",
                    "error",
                    "-y",
                    "-ss",
                    f"{t0:.3f}",
                    "-t",
                    f"{a.seconds}",
                    "-i",
                    path,
                    "-ac",
                    "2",
                    "-ar",
                    "16000",
                    "-c:a",
                    "pcm_s16le",
                    w16,
                ],
                check=True,
            )
            agg, rows = profile(sp, w16, bpm)
        res[key] = dict(
            sr_rec=sr, rec_s=len(x) / sr, t0=t0, bpm=bpm, log=log, agg=agg, windows=rows
        )
        print(
            key,
            f"sr={sr} t0={t0:.2f} bpm={bpm} low={agg['low'][0]:.2f} crest={agg['crest'][0]:.1f} "
            f"swing_q={agg['swing_q'] and round(agg['swing_q'][0], 2)} lufs={agg['lufs']}",
            flush=True,
        )
    # различимость: z-оценка осей по записям робота, евклид; ближайший эталон в тех же осях (без темпа/свинга лога)
    keys = [k for k in res if "agg" in res[k]]

    def val(k, ax):
        if ax == "bpm":
            return res[k]["bpm"]
        v = res[k]["agg"].get(ax)
        return v[0] if v else float("nan")

    M = np.array([[val(k, ax) for ax, _ in AXES] for k in keys], float)
    mu, sd = np.nanmean(M, 0), np.nanstd(M, 0) + 1e-9
    Z = np.nan_to_num((M - mu) / sd)
    D = np.sqrt(((Z[:, None, :] - Z[None, :, :]) ** 2).sum(2))
    # шум внутри стиля: окна 20 с одной записи в том же z-пространстве; различимость пары = медиана расстояний
    # окно×окно между стилями / большая из двух медиан внутри стиля (> 1 — стили дальше друг от друга, чем окна
    # одного стиля между собой)
    W = {}
    for i, k in enumerate(keys):
        rows = []
        for w in res[k]["windows"]:
            rows.append(
                [
                    (
                        res[k]["bpm"]
                        if ax == "bpm"
                        else (w.get(ax) if w.get(ax) is not None else M[i, j])
                    )
                    for j, (ax, _) in enumerate(AXES)
                ]
            )
        W[k] = np.nan_to_num((np.array(rows, float) - mu) / sd)

    def mdist(a, b, same):
        d = np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(2))
        return float(np.median(d[np.triu_indices(len(a), 1)] if same else d))

    intra = {k: mdist(W[k], W[k], True) for k in keys}
    sep = [
        [
            mdist(W[a_], W[b_], False) / max(intra[a_], intra[b_]) if a_ != b_ else 0.0
            for b_ in keys
        ]
        for a_ in keys
    ]
    ref_axes = ("low", "mid", "high", "crest", "lr", "ons_s", "cent")
    ref_cols = [i for i, (ax, _) in enumerate(AXES) if ax in ref_axes]
    near = {}
    for i, k in enumerate(keys):
        d = {}
        for rn, r in refs.items():
            if rn == "angine_frag":
                continue
            rv = np.array([r[f"{ax}@16"]["med"] for ax in ref_axes])
            d[rn] = float(np.sqrt((((M[i, ref_cols] - rv) / sd[ref_cols]) ** 2).sum()))
        near[k] = dict(sorted(d.items(), key=lambda kv: kv[1]))
    print(
        "\nразличимость окон: межстилевая медиана / внутристилевая (intra: "
        + ", ".join(f"{k}={intra[k]:.2f}" for k in keys)
        + ")"
    )
    print("        " + " ".join(f"{k[:7]:>7}" for k in keys))
    for i, k in enumerate(keys):
        print(f"{k[:7]:>7} " + " ".join(f"{sep[i][j]:7.2f}" for j in range(len(keys))))
    out = dict(
        results=res,
        axes=[ax for ax, _ in AXES],
        keys=keys,
        matrix=M.tolist(),
        dist=D.tolist(),
        nearest_ref=near,
        intra=intra,
        separation=sep,
        refs={
            rn: {
                ax: refs[rn][f"{ax}@16"]["med"]
                for ax in ref_axes + ("swing", "swing_half")
            }
            for rn in refs
        },
    )
    print("\nрасстояния (z по осям " + ", ".join(ax for ax, _ in AXES) + ")")
    print("        " + " ".join(f"{k[:7]:>7}" for k in keys))
    for i, k in enumerate(keys):
        print(f"{k[:7]:>7} " + " ".join(f"{D[i, j]:7.2f}" for j in range(len(keys))))
    print("\nближайший эталон по осям", ref_axes)
    for k in keys:
        print(k, " ".join(f"{rn}={d:.2f}" for rn, d in near[k].items()))
    if a.json:
        json.dump(
            out,
            open(a.json, "w", encoding="utf-8"),
            ensure_ascii=False,
            indent=1,
            default=float,
        )


if __name__ == "__main__":
    main()
