#!/usr/bin/env python3
"""Монтаж из настоящих кадров прогона: вид сверху (+ карта rtabmap, поза, след) и 3D-камера в углу.

Только данные прогона: кадры камер Gazebo, снимки /rtabmap/map, истинная поза gt, лог route.log.
Запуск в контейнере rob_box_sim:hil (PIL, numpy, ffmpeg):
  compose.py OUT_DIR SPEED FPS T0 T1 OUTDIR_FRAMES [--map-only]
"""
import bisect
import csv
import glob
import math
import os
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pip_placement import PIP_H, PIP_W, PipPlacer  # noqa: E402

out = Path(sys.argv[1])
speed = float(sys.argv[2])
fps = float(sys.argv[3])
t0 = float(sys.argv[4])
t1 = float(sys.argv[5])
dst = Path(sys.argv[6])
dst.mkdir(parents=True, exist_ok=True)

CX, CY, PXM = 10.5, 0.0, 40.0  # центр кадра сверху в мире, пикселей на метр
W, H = 1280, 720
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
fsm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)

# --- лог маршрута
goals, pairs = [], []
cur = None
for line in open(out / "route.log", encoding="utf-8"):
    m = re.search(r"GOAL_START (\w+) target=\(([-\d.]+),([-\d.]+),([-\d.]+)\) sim=([\d.]+) frame=(-?\d+)", line)
    if m:
        cur = dict(name=m.group(1), x=float(m.group(2)), y=float(m.group(3)), s=float(m.group(5)))
        pairs.append((cur["s"], int(m.group(6))))
        goals.append(cur)
    m = re.search(r"GOAL_END (\w+) result=(\w+) sim=([\d.]+) frame=(-?\d+)", line)
    if m and cur:
        cur["e"] = float(m.group(3))
        cur["res"] = m.group(2)
        pairs.append((cur["e"], int(m.group(4))))
A = np.array(pairs)
a, b = np.polyfit(A[:, 0], A[:, 1], 1)
print("frame = %.4f * sim + %.1f (10 Hz ожидается 10)" % (a, b))

# --- позы и карты
poses = []
for r in csv.reader(open(out / "maprec" / "poses.csv", encoding="utf-8")):
    try:
        poses.append((float(r[0]), float(r[1]), float(r[2]), float(r[3])))
    except ValueError:
        pass
pt = [p[0] for p in poses]
mapfiles = sorted(glob.glob(str(out / "maprec" / "maps" / "map_*.npz")))
mapt = [int(Path(f).stem.split("_")[1]) / 1000 for f in mapfiles]
cache = {}


def getmap(t):
    i = bisect.bisect_right(mapt, t) - 1
    if i < 0:
        return None
    if i not in cache:
        cache.clear()
        z = np.load(mapfiles[i])
        cache[i] = (float(z["resolution"]), float(z["origin_x"]), float(z["origin_y"]), z["data"])
    return cache[i]


def frame_path(cam, idx):
    return glob.glob(str(out / "frames" / cam / f"*_{idx}.png"))[0]


def w2p(x, y):
    return 640 + (x - CX) * PXM, 360 - (y - CY) * PXM


def label_for(t):
    for g in goals:
        if g["s"] <= t <= g.get("e", 1e9):
            return g
    return None


nout = int((t1 - t0) / (speed / fps))
placer = PipPlacer(fps)
for k in range(nout):
    t = t0 + k * speed / fps
    fi = int(round(a * t + b))
    try:
        top = Image.open(frame_path("video_top", fi)).convert("RGBA").resize((W, H))
        ch = Image.open(frame_path("video_chase", fi)).convert("RGB")
    except IndexError:
        continue
    ov = Image.new("RGBA", top.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    mp = getmap(t)
    if mp is not None:
        res, ox, oy, data = mp
        rr, cc = np.nonzero(data >= 0)
        # свободные клетки редкой сеткой, занятые — ярче
        for sel, col, step in ((data == 0, (60, 180, 255, 40), 1), (data >= 65, (255, 60, 60, 230), 1)):
            rr, cc = np.nonzero(sel)
            if len(rr) == 0:
                continue
            xs = ox + (cc + 0.5) * res
            ys = oy + (rr + 0.5) * res
            px = 640 + (xs - CX) * PXM
            py = 360 - (ys - CY) * PXM
            s = max(1, int(res * PXM / 2))
            for x_, y_ in zip(px, py):
                d.rectangle([x_ - s, y_ - s, x_ + s, y_ + s], fill=col)
    for g in goals:
        if not g["name"].startswith(os.environ.get("VIDEO_GOALS", "M")):
            continue
        gx, gy = w2p(g["x"], g["y"])
        col = (80, 255, 120, 255) if g["name"].startswith("R") else (255, 220, 60, 255)
        d.ellipse([gx - 9, gy - 9, gx + 9, gy + 9], outline=col, width=3)
        d.text((gx + 11, gy - 24), g["name"], fill=col, font=fsm)
    j = bisect.bisect_right(pt, t) - 1
    robot_px = w2p(poses[j][1], poses[j][2]) if j >= 0 else (-1e4, -1e4)
    pip_x, pip_y, pip_a = placer.update(*robot_px)
    if j >= 0:
        trail = [w2p(p[1], p[2]) for p in poses[max(0, j - 600) : j + 1 : 3]]  # noqa: E203
        if len(trail) > 1:
            d.line(trail, fill=(255, 255, 255, 220), width=3)
        rx, ry = w2p(poses[j][1], poses[j][2])
        d.ellipse([rx - 8, ry - 8, rx + 8, ry + 8], fill=(0, 255, 80, 255), outline=(0, 0, 0, 255), width=2)
        hx, hy = rx + 18 * math.cos(poses[j][3]), ry - 18 * math.sin(poses[j][3])
        d.line([rx, ry, hx, hy], fill=(0, 0, 0, 255), width=3)
    img = Image.alpha_composite(top, ov).convert("RGB")
    # 3D-вид в углу
    pip = ch.resize((PIP_W, PIP_H))
    frame = Image.new("RGBA", (PIP_W + 4, PIP_H + 4), (255, 255, 255, 255))  # белая окантовка 2 пикс.
    frame.paste(pip, (2, 2))
    if pip_a < 1.0:
        frame.putalpha(int(round(255 * pip_a)))
    if pip_a > 0:
        img.paste(frame, (pip_x - 2, pip_y - 2), frame)
    dr = ImageDraw.Draw(img)
    # плашка
    dr.rectangle([0, H - 48, W, H], fill=(0, 0, 0))
    g = label_for(t)
    phase = "КАРТОГРАФИЯ" if (g and g["name"].startswith("M")) else ("МАРШРУТ ПО КАРТЕ" if g else "")
    txt = f"sim {t - t0 + 0:6.1f} с   x{speed:g}   {phase}"
    if g:
        txt += f"   цель {g['name']}"
    dr.text((14, H - 40), txt, fill=(255, 255, 255), font=font)
    img.save(dst / f"f{k:05d}.png")
print("frames", nout)
