"""Видео картографии: правда мира, растущая карта rtabmap и две позы робота.

Читает запись ``rob_box_sim.map_recorder`` (каталог с ``poses.csv``, ``maps/*.npz``,
``meta.json``) и коробки мира (SDF), рисует кадры вида сверху по sim time и
склеивает их ffmpeg в mp4 и gif::

    python3 -m rob_box_sim.map_video --rec /results/maprec --world /sim_worlds/polygon.sdf \\
        --out /results/polygon_mapping

Кадр:
  * правда мира — коробки SDF (серые);
  * карта rtabmap ``/rtabmap/map`` полупрозрачным слоем: свободно — голубое,
    занято — синее; на кадре t — последняя карта, пришедшая не позже t;
  * истинная поза Gazebo (зелёная) и оценка ``map → base_footprint`` (красная),
    со следами;
  * подпись: sim time, ошибка позы, совпадение карты с миром (``map_agreement``).

Система ``map`` в начале прогона совпадает с миром: спавн в (0, 0, 0) во всех мирах.
Метрики и выбор кадров — чистые функции на numpy (tests/unit/sim/test_sim_map_video.py),
рисование — PIL, склейка — ffmpeg (оба есть в образе docker/sim).
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from rob_box_sim.geometry2d import Box

OCCUPIED = 65  # порог «занято», как map_check и map_saver
# Препятствия ниже плоскости лидара (на роботе 0.477 м над base_link) на карту не
# попадают и в «правду» для сравнения не идут (low_obstacle: коробка 0.3 м).
LIDAR_MIN_HEIGHT = 0.4


@dataclass(frozen=True)
class GridSnap:
    """Снимок OccupancyGrid: t — sim time приёма, data[r, c] — строка r от origin по +Y."""

    t: float
    resolution: float
    origin_x: float
    origin_y: float
    data: np.ndarray  # int8/int16, -1 неизвестно, 0..100

    @property
    def shape(self) -> tuple[int, int]:
        return self.data.shape


# ── Метрики ──────────────────────────────────────────────────────────────────


def world_mask(boxes: Sequence[Box], snap: GridSnap, min_height: float = LIDAR_MIN_HEIGHT) -> np.ndarray:
    """Клетки сетки снимка, центр которых внутри коробки мира (высотой от ``min_height``)."""
    h, w = snap.shape
    xs = snap.origin_x + (np.arange(w) + 0.5) * snap.resolution
    ys = snap.origin_y + (np.arange(h) + 0.5) * snap.resolution
    gx, gy = np.meshgrid(xs, ys)
    mask = np.zeros((h, w), dtype=bool)
    for b in boxes:
        if b.height < min_height:
            continue
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        dx, dy = gx - b.cx, gy - b.cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        # половина клетки допуска: стена 0.1 м при клетке 0.05 даёт 2 клетки, а не 0–1 по сдвигу сетки
        half = snap.resolution / 2
        mask |= (np.abs(lx) <= b.sx / 2 + half) & (np.abs(ly) <= b.sy / 2 + half)
    return mask


def dilate(mask: np.ndarray, cells: int) -> np.ndarray:
    """Расширение маски квадратом (2·cells+1)² сдвигами, без scipy."""
    if cells <= 0:
        return mask.copy()
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx == 0 and dy == 0:
                continue
            src = mask[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)]
            out[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)] |= src
    return out


def map_agreement(snap: GridSnap, boxes: Sequence[Box], tol: float = 0.15) -> dict:
    """Совпадение занятых клеток карты со стенами мира с допуском ``tol``.

    * precision — доля занятых клеток карты, лежащих не дальше tol от стены
      (как ``map_check.compare``: нет ли на карте лишних стен, не съехала ли она);
    * recall — доля клеток стен мира *в пределах сетки*, рядом с которыми (tol)
      есть занятая клетка карты: сколько видимых стен уже нанесено;
    * iou — 1 / (1/precision + 1/recall − 1): это TP / (TP + FP + FN), где доли
      precision и recall считаются каждая в своих клетках. Прямой подсчёт клеток
      смешал бы единицы: стена мира 0.1 м — 2–4 клетки толщиной, а rtabmap рисует
      её линией в одну клетку.

    Стены за краем сетки rtabmap (ещё не виденные) в recall и iou не входят —
    они растут вместе с картой только внутри её границ.
    """
    occ = snap.data >= OCCUPIED
    wall = world_mask(boxes, snap)
    k = max(0, int(math.ceil(tol / snap.resolution - 1e-9)))
    near_wall = dilate(wall, k)
    near_occ = dilate(occ, k)
    n_occ, n_wall = int(np.count_nonzero(occ)), int(np.count_nonzero(wall))
    precision = np.count_nonzero(occ & near_wall) / n_occ if n_occ else None
    recall = np.count_nonzero(wall & near_occ) / n_wall if n_wall else None
    if precision is None or recall is None:
        iou = None if precision is None and recall is None else 0.0
    elif precision == 0 or recall == 0:
        iou = 0.0
    else:
        iou = 1 / (1 / precision + 1 / recall - 1)
    return {
        "occupied": n_occ,
        "wall_cells": n_wall,
        "precision": None if precision is None else round(float(precision), 4),
        "recall": None if recall is None else round(float(recall), 4),
        "iou": None if iou is None else round(float(iou), 4),
        "tol_m": tol,
    }


def interp_pose(samples: Sequence[tuple[float, float, float, float]], t: float, max_gap: float = 1.0):
    """Поза (x, y, yaw) в момент t линейной интерполяцией; None — нет соседей ближе max_gap."""
    if not samples:
        return None
    ts = [s[0] for s in samples]
    i = bisect.bisect_left(ts, t)
    if i < len(ts) and abs(ts[i] - t) < 1e-9:
        return samples[i][1:]
    if i == 0 or i == len(ts):
        j = 0 if i == 0 else len(ts) - 1
        return samples[j][1:] if abs(ts[j] - t) <= max_gap else None
    a, b = samples[i - 1], samples[i]
    if b[0] - a[0] > max_gap:
        return None
    u = (t - a[0]) / (b[0] - a[0])
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return (a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * dyaw)


def pose_error(truth, estimate) -> float | None:
    if truth is None or estimate is None:
        return None
    return math.hypot(truth[0] - estimate[0], truth[1] - estimate[1])


def frame_times(t0: float, t1: float, step: float) -> list[float]:
    """Моменты кадров по sim time: t0, t0+step, ..., не позже t1 (t1 всегда последний)."""
    if t1 < t0 or step <= 0:
        return []
    n = int(math.floor((t1 - t0) / step + 1e-9))
    times = [t0 + i * step for i in range(n + 1)]
    if t1 - times[-1] > 1e-6:
        times.append(t1)
    return times


def snap_at(snaps: Sequence[GridSnap], t: float) -> GridSnap | None:
    """Последний снимок карты с t_snap ≤ t (снимки по возрастанию t)."""
    i = bisect.bisect_right([s.t for s in snaps], t)
    return snaps[i - 1] if i else None


# ── Чтение записи ────────────────────────────────────────────────────────────


def load_poses(path: Path):
    """poses.csv → (truth, estimate): списки (t, x, y, yaw); пустые ячейки оценки пропускаются."""
    truth, est = [], []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t = float(row["t"])
            if row["tx"]:
                truth.append((t, float(row["tx"]), float(row["ty"]), float(row["tyaw"])))
            if row["ex"]:
                est.append((t, float(row["ex"]), float(row["ey"]), float(row["eyaw"])))
    return truth, est


def load_snaps(maps_dir: Path) -> list[GridSnap]:
    snaps = []
    for p in sorted(maps_dir.glob("map_*.npz")):
        z = np.load(p)
        snaps.append(GridSnap(float(z["t"]), float(z["resolution"]), float(z["origin_x"]),
                              float(z["origin_y"]), z["data"]))
    snaps.sort(key=lambda s: s.t)
    return snaps


# ── Рисование ────────────────────────────────────────────────────────────────

COLORS = {
    "bg": (250, 250, 250),
    "wall": (110, 110, 110),
    "free": (120, 190, 255),
    "occ": (20, 40, 200),
    "truth": (0, 160, 60),
    "est": (220, 30, 30),
    "text": (20, 20, 20),
}


@dataclass(frozen=True)
class View:
    x0: float
    y0: float
    x1: float
    y1: float
    px_per_m: float
    header_px: int = 96

    @property
    def size(self) -> tuple[int, int]:
        w = int(round((self.x1 - self.x0) * self.px_per_m))
        h = int(round((self.y1 - self.y0) * self.px_per_m)) + self.header_px
        return w + w % 2, h + h % 2  # yuv420p: чётные стороны

    def px(self, x: float, y: float) -> tuple[float, float]:
        return ((x - self.x0) * self.px_per_m, self.header_px + (self.y1 - y) * self.px_per_m)


def view_for(
    boxes: Sequence[Box], truth, margin: float = 0.8, width_px: int = 1280, crop: float | None = None
) -> View:
    """Вид на мир целиком; ``crop`` — только путь робота ± crop м (длинный коридор)."""
    xs = [b.cx + s * b.sx / 2 for b in boxes for s in (-1, 1)] + [p[1] for p in truth]
    ys = [b.cy + s * b.sy / 2 for b in boxes for s in (-1, 1)] + [p[2] for p in truth]
    x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
    if crop is not None:
        tx, ty = [p[1] for p in truth], [p[2] for p in truth]
        x0, x1 = max(x0, min(tx) - crop), min(x1, max(tx) + crop)
        y0, y1 = max(y0, min(ty) - crop), min(y1, max(ty) + crop)
    ppm = width_px / (x1 - x0)
    # Высокие миры (turn90): не выше 900 px по карте.
    ppm = min(ppm, 900 / (y1 - y0))
    return View(x0, y0, x1, y1, ppm)


def _font(size: int):
    from PIL import ImageFont

    for name in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _map_layer(snap: GridSnap, view: View, alpha: int):
    """RGBA-слой карты в пикселях вида: свободно / занято, неизвестно прозрачно."""
    from PIL import Image

    h, w = snap.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    free = (snap.data >= 0) & (snap.data < OCCUPIED)
    occ = snap.data >= OCCUPIED
    rgba[free] = (*COLORS["free"], alpha)
    rgba[occ] = (*COLORS["occ"], min(255, alpha + 90))
    img = Image.fromarray(rgba[::-1], "RGBA")  # строка 0 сетки — низ (+Y вверх)
    size = (max(1, round(w * snap.resolution * view.px_per_m)), max(1, round(h * snap.resolution * view.px_per_m)))
    img = img.resize(size, Image.NEAREST)
    left, top = view.px(snap.origin_x, snap.origin_y + h * snap.resolution)
    return img, (int(round(left)), int(round(top)))


def _robot(draw, view: View, pose, color, r_m: float = 0.28, width: int = 3):
    x, y = view.px(pose[0], pose[1])
    r = r_m * view.px_per_m
    draw.ellipse([x - r, y - r, x + r, y + r], outline=color, width=width)
    hx, hy = view.px(pose[0] + math.cos(pose[2]) * r_m * 1.6, pose[1] + math.sin(pose[2]) * r_m * 1.6)
    draw.line([x, y, hx, hy], fill=color, width=width)


def _legend(draw, x: int, y: int, font) -> None:
    """Подписи цветов: квадрат или отрезок того же цвета, что на кадре."""
    items = (
        ("box", COLORS["wall"], "стены мира"),
        ("box", COLORS["free"], "карта: свободно"),
        ("box", COLORS["occ"], "карта: занято"),
        ("line", COLORS["truth"], "правда Gazebo"),
        ("line", COLORS["est"], "оценка map→base"),
    )
    for kind, color, text in items:
        if kind == "box":
            draw.rectangle([x, y + 2, x + 14, y + 16], fill=color)
        else:
            draw.line([x, y + 9, x + 22, y + 9], fill=color, width=5 if color == COLORS["truth"] else 2)
        x += 28
        draw.text((x, y), text, fill=(70, 70, 70), font=font)
        x += int(draw.textlength(text, font=font)) + 22


def render_frame(view: View, boxes, snap, truth_trail, est_trail, truth_pose, est_pose, lines, alpha=110):
    from PIL import Image, ImageDraw

    img = Image.new("RGB", view.size, COLORS["bg"])
    draw = ImageDraw.Draw(img)
    for b in boxes:
        draw.polygon([view.px(*p) for p in b.corners()], fill=COLORS["wall"])
    if snap is not None:
        layer, pos = _map_layer(snap, view, alpha)
        img.paste(layer, pos, layer)
        draw = ImageDraw.Draw(img)
    # Правда — широкой линией снизу, оценка — тонкой сверху: при ошибке в сантиметры видны обе.
    for trail, color, width in ((truth_trail, COLORS["truth"], 6), (est_trail, COLORS["est"], 2)):
        if len(trail) >= 2:
            draw.line([view.px(p[1], p[2]) for p in trail], fill=color, width=width)
    if truth_pose is not None:
        _robot(draw, view, truth_pose, COLORS["truth"], r_m=0.32, width=5)
    if est_pose is not None:
        _robot(draw, view, est_pose, COLORS["est"], r_m=0.26, width=3)
    draw.rectangle([0, 0, view.size[0], view.header_px - 4], fill=(255, 255, 255))
    font = _font(24)
    small = _font(18)
    draw.text((12, 8), lines[0], fill=COLORS["text"], font=font)
    draw.text((12, 44), lines[1], fill=COLORS["text"], font=small)
    _legend(draw, 12, 70, _font(15))
    return img


# ── Сборка видео ─────────────────────────────────────────────────────────────


def _ffmpeg(args: list[str]) -> None:
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *args], check=True)


def encode(frames_dir: Path, fps: int, mp4: Path, gif: Path, gif_mb: float) -> list[str]:
    pattern = str(frames_dir / "f_%05d.png")
    log = []
    for crf in (23, 28, 33):
        _ffmpeg(["-framerate", str(fps), "-i", pattern, "-c:v", "libx264", "-preset", "medium",
                 "-crf", str(crf), "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)])
        log.append(f"mp4 crf {crf}: {mp4.stat().st_size // 1024} КБ")
        if mp4.stat().st_size <= gif_mb * 1024 * 1024:
            break
    for w in (960, 800, 640, 480):
        for gfps in (8, 6, 4):
            vf = (f"fps={gfps},scale={w}:-1:flags=lanczos,split[s0][s1];"
                  "[s0]palettegen=max_colors=64:stats_mode=diff[p];"
                  "[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle")
            _ffmpeg(["-i", str(mp4), "-vf", vf, str(gif)])
            size = gif.stat().st_size
            log.append(f"gif {w}px {gfps} fps: {size // 1024} КБ")
            if size <= gif_mb * 1024 * 1024:
                return log
    return log


def main(argv: list[str] | None = None) -> int:
    from rob_box_sim.world_sdf import load_world_boxes

    ap = argparse.ArgumentParser(description="Видео картографии: мир | карта rtabmap | позы")
    ap.add_argument("--rec", type=Path, required=True, help="каталог записи map_recorder")
    ap.add_argument("--world", type=Path, required=True, help="SDF мира")
    ap.add_argument("--out", type=Path, required=True, help="префикс выходных файлов (без расширения)")
    ap.add_argument("--title", default="")
    ap.add_argument("--step", type=float, default=0.5, help="шаг кадров, с sim time")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--trail-s", type=float, default=1e9, help="длина следа, с (по умолчанию весь)")
    ap.add_argument("--tol", type=float, default=0.15)
    ap.add_argument("--max-mb", type=float, default=10.0)
    ap.add_argument("--crop", type=float, default=None, help="вид: путь робота ± crop м, а не весь мир")
    args = ap.parse_args(argv)

    boxes = load_world_boxes(args.world)
    truth, est = load_poses(args.rec / "poses.csv")
    snaps = load_snaps(args.rec / "maps")
    meta = json.loads((args.rec / "meta.json").read_text(encoding="utf-8"))
    if not truth:
        print("[map_video] нет истинных поз", file=sys.stderr)
        return 1
    view = view_for(boxes, truth, crop=args.crop)
    t0 = truth[0][0]
    times = frame_times(t0, truth[-1][0], args.step)
    frames = args.out.parent / (args.out.name + "_frames")
    if frames.exists():
        shutil.rmtree(frames)
    frames.mkdir(parents=True)
    agreement_cache: dict[float, dict] = {}
    series = []
    for i, t in enumerate(times):
        snap = snap_at(snaps, t)
        tp, ep = interp_pose(truth, t), interp_pose(est, t)
        err = pose_error(tp, ep)
        if snap is not None and snap.t not in agreement_cache:
            agreement_cache[snap.t] = map_agreement(snap, boxes, args.tol)
        agr = agreement_cache.get(snap.t) if snap is not None else None
        ttrail = [p for p in truth if t - args.trail_s <= p[0] <= t]
        etrail = [p for p in est if t - args.trail_s <= p[0] <= t]
        err_s = f"{err:.2f} м" if err is not None else "нет оценки"
        if agr and agr["iou"] is not None:
            map_s = (f"IoU {agr['iou']:.2f}  precision {agr['precision']:.2f}  recall {agr['recall']:.2f}"
                     f"  (допуск {args.tol:.2f} м)")
        else:
            map_s = "карты ещё нет"
        lines = [
            f"{args.title}  t = {t - t0:6.1f} с (sim)   ошибка позы {err_s}",
            f"карта rtabmap vs стены мира: {map_s}",
        ]
        render_frame(view, boxes, snap, ttrail, etrail, tp, ep, lines).save(frames / f"f_{i:05d}.png")
        series.append({"t": round(t - t0, 2), "pose_err_m": None if err is None else round(err, 4),
                       **({k: agr[k] for k in ("iou", "precision", "recall", "occupied")} if agr else {})})
    mp4, gif = args.out.with_suffix(".mp4"), args.out.with_suffix(".gif")
    log = encode(frames, args.fps, mp4, gif, args.max_mb)
    shutil.rmtree(frames)
    final = series[-1]
    summary = {
        "frames": len(times),
        "sim_s": round(times[-1] - t0, 1),
        "fps": args.fps,
        "speedup": round(args.step * args.fps, 2),
        "final": final,
        "pose_err_max_m": max((s["pose_err_m"] for s in series if s["pose_err_m"] is not None), default=None),
        "map_snapshots": len(snaps),
        "recorder": meta,
        "encode": log,
    }
    args.out.with_suffix(".json").write_text(json.dumps({"summary": summary, "series": series},
                                                        ensure_ascii=False, indent=1), encoding="utf-8")
    print("[map_video] " + json.dumps(summary, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
