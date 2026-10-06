#!/usr/bin/env python3
"""Разнообразие треков сета (ADR-0149 §7.2, A16): по звуку (librosa) и по составу (синты, бочка, форма, ...).

Три режима (запускать из этой папки; на Windows ``PYTHONUTF8=1``):

* ``audio``: запись + лог -> признаки трека (MFCC среднее/ст.откл., spectral contrast, onset-rate, centroid),
  межтрековое расстояние (медиана/минимум по соседним и по всем парам). Границы треков - строки
  ``[music v2] started`` лога (старт записи - из ``index.txt``); нет лога - окна по 60 с. Признаки
  z-нормируются по статистике ЭТАЛОНА (если он дан), поэтому числа робота и эталона сравнимы:
  расстояние = евклид / sqrt(размерность) = среднеквадратичная разница в сигмах эталона.
* ``--dir <каталог>``: серия (``index.txt``, ``<prefix>N.wav``, ``setN.full.log``) целиком.
* ``--offline N``: N случайных мелодий из RTTTL-архива -> seeded-профиль -> сет из 5 треков -> вектор состава
  (без звука и ROS): сколько разных синтов пэда/лида/баса, бочек, форм, каркасов хэтов.

Состав по логу (ADR-0152 §2.3): строка ``[set v2] ... started`` несёт ``composition={...}`` - синты, бочку, форму,
прогрессию и т.д. читает ``v2log.parse_composition``, офлайн-реконструкция не нужна. Лог старого движка без
``composition=`` даёт только bpm/лад/хуки плана. ``--offline`` строит тот же вектор через ``track_composition``.
"""
from __future__ import annotations

import argparse
import calendar
import json
import os
import random
import re
import time
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

import v2log as V

SR = 16000
WINDOW_S = 60.0
#: Оси состава трека (``composition``): значение оси - hashable.
AXES = ("pad", "pad_figure", "lead", "bass", "bass_figure", "kick", "kit", "form", "bpm", "mode", "root", "hook",
        "prog", "sample", "perc", "fx", "energy")
SYNTH_AXES = ("pad", "pad_figure", "lead", "bass", "bass_figure", "kick", "form", "kit")


# ── звук ────────────────────────────────────────────────────────────────────────────────────────────────

def track_features(y: np.ndarray, sr: int = SR) -> np.ndarray:
    """Вектор признаков трека: MFCC(13) среднее+ст.откл., spectral contrast(7) среднее, onset-rate, centroid."""
    import librosa

    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
    contrast = librosa.feature.spectral_contrast(y=y, sr=sr, fmin=100.0, n_bands=6)
    onsets = librosa.onset.onset_detect(y=y, sr=sr, units="time")
    centroid = librosa.feature.spectral_centroid(y=y, sr=sr)
    rate = len(onsets) / max(len(y) / sr, 1e-9)
    return np.concatenate([mfcc.mean(axis=1), mfcc.std(axis=1), contrast.mean(axis=1), [rate], [centroid.mean()]])


def normalizer(reference: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(среднее, сигма) признаков по строкам ``reference``; нулевая сигма -> 1 (константный признак не взрывает)."""
    sigma = reference.std(axis=0)
    return reference.mean(axis=0), np.where(sigma > 1e-9, sigma, 1.0)


def pair_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Среднеквадратичная разница векторов (евклид / sqrt(dim)); одинаковые -> 0."""
    return float(np.linalg.norm(a - b) / np.sqrt(len(a)))


def distance_stats(z: np.ndarray) -> Dict[str, float]:
    """Медиана/минимум расстояния по соседним и по всем парам строк ``z`` (нормированные признаки)."""
    n = len(z)
    if n < 2:
        return {"n": n, "adj_median": float("nan"), "adj_min": float("nan"),
                "all_median": float("nan"), "all_min": float("nan")}
    adj = [pair_distance(z[i], z[i + 1]) for i in range(n - 1)]
    allp = [pair_distance(z[i], z[j]) for i in range(n) for j in range(i + 1, n)]
    return {"n": n, "adj_median": float(np.median(adj)), "adj_min": float(np.min(adj)),
            "all_median": float(np.median(allp)), "all_min": float(np.min(allp))}


def read_window(path: str, start: float, dur: float, sr: int = SR) -> np.ndarray:
    import librosa

    y, _ = librosa.load(path, sr=sr, mono=True, offset=max(start, 0.0), duration=dur)
    return y


def wav_seconds(path: str) -> float:
    import soundfile

    return float(soundfile.info(path).duration)


def fixed_windows(total: float, length: float = WINDOW_S, limit: int = 0) -> List[Tuple[float, float]]:
    """Окна ``(start, dur)`` подряд по ``length`` секунд (неполный хвост отбрасывается); ``limit`` > 0 - не больше."""
    out = [(i * length, length) for i in range(int(total // length))]
    return out[:limit] if limit > 0 else out


def log_segments(log_path: str, rec_start_epoch: float, total: float) -> List[Tuple[float, float]]:
    """Окна треков ``(start, dur)`` по строкам ``[music v2] started``: от старта трека до старта следующего."""
    starts = []
    with open(log_path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            rec = V.parse_started(line)
            if rec:
                starts.append(rec["t_abs"] - rec_start_epoch)
    starts = [t for t in starts if 0 <= t < total]
    ends = starts[1:] + [total]
    return [(s, e - s) for s, e in zip(starts, ends) if e - s >= 20.0]


def iso_epoch(text: str) -> float:
    return float(calendar.timegm(time.strptime(text, "%Y-%m-%dT%H:%M:%SZ")))


def set_features(wav: str, windows: Sequence[Tuple[float, float]]) -> np.ndarray:
    return np.array([track_features(read_window(wav, s, d)) for s, d in windows])


def parse_index(path: str) -> Dict[int, Tuple[str, float]]:
    """``index.txt`` серии -> ``{N: (тема, старт epoch)}``."""
    out = {}
    pat = re.compile(r"=== SET (\d+) '(.*)' start (\S+)")
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            m = pat.match(line)
            if m:
                out[int(m.group(1))] = (m.group(2), iso_epoch(m.group(3)))
    return out


def reference_features(ref_wav: str, length: float, limit: int) -> np.ndarray:
    return set_features(ref_wav, fixed_windows(wav_seconds(ref_wav), length, limit))


def run_audio(args: argparse.Namespace) -> int:
    sets: Dict[str, np.ndarray] = {}
    lens: List[float] = []
    logs: List[List[Dict[str, object]]] = []
    if args.dir:
        index = parse_index(os.path.join(args.dir, "index.txt"))
        for no, (theme, t0) in sorted(index.items()):
            wav = os.path.join(args.dir, f"{args.prefix}{no}.wav")
            log = os.path.join(args.dir, f"set{no}.full.log")
            total = wav_seconds(wav)
            win = log_segments(log, t0, total) if os.path.exists(log) else []
            win = win or fixed_windows(total, WINDOW_S)
            print(f"set{no} «{theme}»: {len(win)} окон, средняя длина {np.mean([d for _, d in win]):.0f} с")
            lens += [d for _, d in win]
            if os.path.exists(log):
                logs.append(log_composition(log))
            sets[f"set{no}"] = set_features(wav, win)
    else:
        total = wav_seconds(args.wav)
        win = log_segments(args.log, iso_epoch(args.start), total) if args.log else fixed_windows(total, WINDOW_S)
        lens += [d for _, d in win]
        sets[os.path.basename(args.wav)] = set_features(args.wav, win)
    mean_len = args.window or float(np.mean(lens))
    if args.ref:
        ref = reference_features(args.ref, mean_len, args.ref_windows)
        mu, sigma = normalizer(ref)
        ref_stats = distance_stats((ref - mu) / sigma)
        print(f"\nЭТАЛОН {os.path.basename(args.ref)}: окна {mean_len:.0f} с")
        print_stats("эталон", ref_stats)
    else:
        mu, sigma = normalizer(np.vstack(list(sets.values())))
        ref_stats = None
    print("\nРОБОТ (признаки нормированы по " + ("эталону)" if args.ref else "самой записи)"))
    medians = []
    for name, feats in sets.items():
        st = distance_stats((feats - mu) / sigma)
        print_stats(name, st, ref_stats)
        medians.append(st)
    if len(medians) > 1:
        keys = ("adj_median", "adj_min", "all_median", "all_min")
        agg = {k: float(np.nanmedian([m[k] for m in medians])) for k in keys}
        print_stats("медиана по сетам", {"n": len(medians), **agg}, ref_stats)
    if logs:
        flat = [v for vecs in logs for v in vecs]
        idx = diversity_index(flat, ("bpm", "mode", "hook"))
        print(f"\nСостав по логам (в логе только bpm/лад/хуки плана, синтов нет): треков {idx['n']}, "
              f"разных {idx['distinct']}")
    return 0


def print_log_composition(logs: Sequence[Sequence[Mapping[str, object]]]) -> None:
    """Состав по логам серии: полный (``composition=`` в ``started``) или только bpm/лад/хуки плана (старый лог)."""
    flat = [v for vecs in logs for v in vecs]
    full = [v for v in flat if all(a in v for a in AXES)]
    if full:
        idx = diversity_index(full)
        print(f"\nСостав по логам (composition= в started): треков {idx['n']} из {len(flat)}, разных по осям "
              f"{idx['distinct']}, индекс {idx['index']:.3f}")
        return
    idx = diversity_index(flat, ("bpm", "mode", "hook"))
    print(f"\nСостав по логам (composition= нет - только bpm/лад/хуки плана): треков {idx['n']}, "
          f"разных {idx['distinct']}")


def print_stats(name: str, st: Mapping[str, float], ref: Optional[Mapping[str, float]] = None) -> None:
    line = (f"  {name:<18} n={int(st['n']):>3}  соседние: медиана {st['adj_median']:.3f} мин {st['adj_min']:.3f}  "
            f"все пары: медиана {st['all_median']:.3f} мин {st['all_min']:.3f}")
    if ref:
        line += f"  | к эталону: медиана(все) {100 * st['all_median'] / ref['all_median']:.0f}%"
    print(line)


# ── состав ──────────────────────────────────────────────────────────────────────────────────────────────

def composition(track) -> Dict[str, object]:
    """Вектор состава ``Track`` v2 по осям :data:`AXES` - тот же ``track_composition``, что пишет лог движка."""
    from rob_box_music.diversity import track_composition

    return track_composition(track)


def diversity_index(vectors: Sequence[Mapping[str, object]], axes: Sequence[str] = AXES) -> Dict[str, object]:
    """Число разных значений и доля уникальных (разных / треков) по каждой оси; ``index`` - средняя доля по осям."""
    n = len(vectors)
    distinct = {a: len({v[a] for v in vectors}) for a in axes}
    share = {a: (distinct[a] / n if n else 0.0) for a in axes}
    return {"n": n, "distinct": distinct, "share": share, "index": float(np.mean(list(share.values()))) if n else 0.0}


def log_composition(log_path: str) -> List[Dict[str, object]]:
    """Состав треков по логу: ``composition={...}`` строки ``started`` (все оси), иначе bpm/лад/хуки плана."""
    out: List[Dict[str, object]] = []
    plan: Dict[str, str] = {}
    with open(log_path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            p = V.parse_plan(line)
            if p:
                plan = p
            comp = V.parse_composition(line)
            if comp:
                out.append(comp)
                continue
            s = V.parse_started(line)
            if s:
                out.append({"bpm": s["bpm"], "mode": plan.get("mode", "-"), "hook": plan.get("hooks", "-"),
                            "row": plan.get("row", "-")})
    return out


def _archive_path() -> str:
    from importlib.resources import files

    return str(files("rob_box_mcp_tools.data").joinpath("rtttl_melodies.jsonl.gz"))


def load_archive(path: Optional[str] = None) -> Dict[str, Dict[str, str]]:
    """RTTTL-архив ``{name: {title, rtttl}}`` (те же записи, что ``RtttlLibrary``; без БД)."""
    import gzip

    out = {}
    with gzip.open(path or _archive_path(), "rt", encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                rec = json.loads(line)
                if rec.get("name") and rec.get("rtttl") and len(rec["rtttl"]) >= 60:
                    out[rec["name"]] = {"title": rec.get("title") or rec["name"], "rtttl": rec["rtttl"]}
    return out


def offline_set(record: Mapping[str, str], name: str, seed: int, tracks: int = 5) -> List[Dict[str, object]]:
    """Сет из ``tracks`` треков на мелодии ``name``: ``seeded_profile`` -> ``seeded_plan`` -> ``compose``."""
    from rob_box_music.arrange.compose import compose
    from rob_box_music.diversity import track_history
    from rob_box_music.set_plan import seeded_plan
    from rob_box_music.theme import seeded_profile

    profile = seeded_profile(record["title"], found=(name,))
    plan = seeded_plan(profile, seed, tracks, set_id=f"off{seed}")
    melodies = {name: record["rtttl"]}
    history: List[Dict[str, object]] = []
    out = []
    for no in range(1, tracks + 1):
        track = compose(plan, no, melodies=melodies, history=history)
        history.insert(0, track_history(track, plan.set_id))
        out.append(composition(track))
    return out


def pick_names(archive: Mapping[str, object], n: int, seed: int) -> List[str]:
    """``n`` случайных названий архива по сиду: один выбор для ``--offline`` и ``--pick`` (серия на роботе)."""
    return random.Random(seed).sample(sorted(archive), n)


def theme_text(title: str) -> str:
    """Название как тема для фразы: только буквы/цифры/пробелы (кавычки ломают инъекцию в ``/voice/stt/result``)."""
    return " ".join(re.sub(r"[^\w ]", " ", title).split())


def run_pick(args: argparse.Namespace) -> int:
    archive = load_archive(args.archive)
    for name in pick_names(archive, args.pick, args.seed):
        print(theme_text(archive[name]["title"]) or name)
    return 0


def run_offline(args: argparse.Namespace) -> int:
    archive = load_archive(args.archive)
    names = pick_names(archive, args.offline, args.seed)
    print(f"seed={args.seed}  мелодий={args.offline}  треков в сете={args.tracks}  архив={len(archive)} мелодий")
    every: List[Dict[str, object]] = []
    per_set = []
    for i, name in enumerate(names, 1):
        try:
            vecs = offline_set(archive[name], name, args.seed * 1000 + i, args.tracks)
        except Exception as exc:  # noqa: BLE001 — мелодия, которую compose не принял, не должна ронять отчёт
            print(f"  пропуск {name}: {type(exc).__name__}: {exc}")
            continue
        every += vecs
        per_set.append(diversity_index(vecs))
        title = archive[name]["title"][:28]
        print(f"  {i:>2}. {title:<28} pad={vecs[0]['pad']:<8} lead={sorted({v['lead'] for v in vecs})} "
              f"bass={sorted({v['bass'] for v in vecs})} kit={len({v['kit'] for v in vecs})} bpm={vecs[0]['bpm']}")
    total = diversity_index(every)
    print(f"\nВсего треков {total['n']} (сетов {len(per_set)}). Разных значений по оси / доля уникальных:")
    for axis in AXES:
        inside = float(np.mean([s["distinct"][axis] for s in per_set])) if per_set else 0.0
        print(f"  {axis:<5} {total['distinct'][axis]:>3} / {total['share'][axis]:.2f}   "
              f"внутри сета в среднем {inside:.1f} из {args.tracks}")
    print(f"Индекс разнообразия состава (средняя доля по осям): {total['index']:.3f}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wav", help="запись одного сета")
    ap.add_argument("--log", help="лог сета (границы треков); нет - окна по 60 с")
    ap.add_argument("--start", help="старт записи UTC из index.txt (с --log)")
    ap.add_argument("--dir", help="каталог серии: index.txt, <prefix>N.wav, setN.full.log")
    ap.add_argument("--prefix", default="r", help="префикс wav в каталоге серии (r или t)")
    ap.add_argument("--ref", help="эталон (wav 16 кГц): база нормировки и порога")
    ap.add_argument("--window", type=float, default=0.0,
                    help="длина окна эталона, с (по умолчанию - средний трек робота)")
    ap.add_argument("--ref-windows", type=int, default=0, help="не больше окон эталона (0 - все)")
    ap.add_argument("--offline", type=int, default=0, metavar="N", help="состав: N случайных мелодий из RTTTL-архива")
    ap.add_argument("--pick", type=int, default=0, metavar="N",
                    help="напечатать N случайных тем (названий) из архива по --seed и выйти (для runs_random.sh)")
    ap.add_argument("--tracks", type=int, default=5, help="треков в офлайн-сете")
    ap.add_argument("--seed", type=int, default=1, help="сид выбора мелодий (публиковать вместе с результатом)")
    ap.add_argument("--archive", help="путь к rtttl_melodies.jsonl.gz (по умолчанию - из rob_box_mcp_tools)")
    args = ap.parse_args(argv)
    if args.pick:
        return run_pick(args)
    if args.offline:
        return run_offline(args)
    if not (args.dir or args.wav):
        ap.error("нужен --dir, --wav или --offline N")
    if args.log and not args.start and not args.dir:
        ap.error("--log требует --start (старт записи из index.txt)")
    return run_audio(args)


if __name__ == "__main__":
    raise SystemExit(main())
