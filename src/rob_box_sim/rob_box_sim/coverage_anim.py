"""Анимация обхода покрытием (#4560): карта + след корпуса по правде Gazebo + стопы защиты. Без ROS.

Чистая функция «данные → кадры» (``render_frames``) и сборка GIF с лимитом размера (``save_gif``).
Покрытие в подписи и цвет клеток берутся из ``coverage_metric`` (``covered_mask`` / ``coverage_result``): на
последнем кадре число совпадает с ``COVERAGE_DONE``. Зелёное — пол, прошедший под корпусом; красное —
покрываемое, но ещё не покрытое; оранжевое — свободно, но корпусу недоступно.

    python -m rob_box_sim.coverage_anim plan_map.npz coverage.log out.gif [--plan plan.json] [--max-mb 2]
"""

from __future__ import annotations

import argparse
import io
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from rob_box_sim.coverage_metric import TrackPoint, coverage_result, coverage_rgb, covered_mask, parse_track_log
from rob_box_sim.coverage_plan import CoverageSpace, coverage_space, load_grid_npz
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT

_EM_RE = re.compile(r"\bEM_ON sim=(\S+)")
_FONT_PATHS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    "C:/Windows/Fonts/arial.ttf",
)
PLAN_COLOR = (90, 90, 230)
STOP_COLOR = (230, 30, 30)
ROBOT_COLOR = (20, 20, 20)
MAX_GIF_BYTES = 2 * 1024 * 1024  # лимит Telegram для анимации, с запасом
HOLD_MS = 1500  # последний кадр держим, чтобы итог успели прочитать


def parse_em_times(lines: Sequence[str]) -> list[float]:
    """Время sim (с) стопов защиты: строки ``EM_ON sim=…``."""
    return [float(m.group(1)) for m in map(_EM_RE.search, lines) if m]


def _font(size: int):
    for path in _FONT_PATHS:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size)
    except TypeError:  # Pillow < 10.1
        return ImageFont.load_default()


def frame_ends(n: int, frames: int) -> list[int]:
    """Правые границы (исключая) срезов трека на кадрах: не больше ``frames`` значений, последний — ``n``."""
    k = max(1, min(frames, n))
    return sorted({max(1, round(n * (i + 1) / k)) for i in range(k)})


def render_frames(
    space: CoverageSpace,
    known: np.ndarray,
    track: Sequence[TrackPoint],
    plan_xy: Sequence[tuple[float, float]] = (),
    em_times: Sequence[float] = (),
    frames: int = 60,
    scale: int = 1,
    pad_cells: int = 20,
    title: str = "HIL",
) -> list[tuple[Image.Image, float]]:
    """Кадры (RGB) и покрытый м² на каждом. Покрытие копится по кускам трека через ``covered_mask``."""
    if not track:
        raise ValueError("пустой трек")
    h, w = space.free.shape
    ys, xs = np.nonzero(known & ~space.free)
    if len(xs) == 0:
        ys, xs = np.nonzero(space.free)
    r0, r1 = max(int(ys.min()) - pad_cells, 0), min(int(ys.max()) + pad_cells + 1, h)
    c0, c1 = max(int(xs.min()) - pad_cells, 0), min(int(xs.max()) + pad_cells + 1, w)
    res = space.resolution

    def px(x: float, y: float) -> tuple[float, float]:  # мир → пиксель кадра (верх — +Y)
        return ((x - space.origin_x) / res - c0 + 0.5) * scale, (r1 - 1 - (y - space.origin_y) / res + 0.5) * scale

    font = _font(max(13, 5 * scale))
    fp = ROBOT_FOOTPRINT
    corners = [(fp.x_max, fp.y_max), (fp.x_max, fp.y_min), (fp.x_min, fp.y_min), (fp.x_min, fp.y_max)]
    covered = np.zeros_like(space.free)
    out, j = [], 0
    t_start = track[0][0]
    for k in frame_ends(len(track), frames):
        covered |= covered_mask(space, track[max(j - 1, 0) : k])  # перехлёст в одну точку: звено не теряется
        j = k
        rgb = coverage_rgb(space, covered, known)  # строка 0 — верх (+Y)
        img = Image.fromarray(np.ascontiguousarray(rgb[h - r1 : h - r0, c0:c1])).resize(
            ((c1 - c0) * scale, (r1 - r0) * scale), Image.NEAREST
        )
        dr = ImageDraw.Draw(img)
        for x, y in plan_xy:
            dr.point(px(x, y), fill=PLAN_COLOR)
        t_now = track[k - 1][0]
        for te in em_times:
            if te <= t_now:
                near = min(track[:k], key=lambda p: abs(p[0] - te))
                u, v = px(near[1], near[2])
                r = 3 * scale
                dr.ellipse((u - r, v - r, u + r, v + r), outline=STOP_COLOR, width=max(1, scale // 2))
        _, x, y, yaw = track[k - 1]
        c, s = math.cos(yaw), math.sin(yaw)
        dr.polygon([px(x + a * c - b * s, y + a * s + b * c) for a, b in corners], outline=ROBOT_COLOR)
        res_now = coverage_result(space, covered)
        pct = "n/a" if res_now.pct is None else f"{res_now.pct:.1f}%"
        label = (
            f"{title}  t={(t_now - t_start) / 60:4.1f} min  "
            f"covered {res_now.covered_m2:.0f}/{res_now.coverable_m2:.0f} m2 ({pct})"
        )
        dr.rectangle((0, 0, img.width, font.size + 8), fill=(255, 255, 255))
        dr.text((6, 3), label, fill=(0, 0, 0), font=font)
        out.append((img, res_now.covered_m2))
    return out


def save_gif(
    frames: Sequence[Image.Image], path: Path, max_bytes: int = MAX_GIF_BYTES, fps: float = 8.0, colors: int = 32
) -> tuple[int, int, int]:
    """GIF ≤ ``max_bytes``: палитра адаптивная; при превышении прореживаем кадры, потом сжимаем.

    Возвращает (байт, кадров, ширина). Последний кадр (итог) остаётся всегда.
    """
    if not frames:
        raise ValueError("нет кадров")
    cur, width = list(frames), frames[0].width
    while True:
        k = width / frames[0].width
        pal = [f.resize((width, round(f.height * k))) if k != 1 else f for f in cur]
        pal = [f.convert("P", palette=Image.ADAPTIVE, colors=colors) for f in pal]
        buf = io.BytesIO()
        dur = [round(1000 / fps)] * (len(pal) - 1) + [HOLD_MS]
        pal[0].save(buf, "GIF", save_all=True, append_images=pal[1:], duration=dur, loop=0, optimize=True)
        if buf.tell() <= max_bytes or (len(cur) <= 8 and width <= 240):
            break
        if len(cur) > 8:
            cur = cur[:-1:2] + [cur[-1]]  # каждый второй, последний (итог) остаётся всегда
        else:
            width = int(width * 0.8)
    path.write_bytes(buf.getvalue())
    return buf.tell(), len(pal), width


def save_mp4(frames: Sequence[Image.Image], path: Path, fps: float = 8.0) -> bool:
    """MP4 через ffmpeg (если есть в PATH): кадры — PNG по пайпу, чётные стороны. False — ffmpeg нет."""
    exe = shutil.which("ffmpeg")
    if not exe:
        return False
    w, h = frames[0].width // 2 * 2, frames[0].height // 2 * 2
    cmd = [exe, "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", str(fps), "-i", "-"]
    cmd += ["-vf", f"crop={w}:{h}", "-pix_fmt", "yuv420p", str(path)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in list(frames) + [frames[-1]] * int(fps * 1.5):
        f.save(p.stdin, "PNG")
    p.stdin.close()
    return p.wait() == 0


def build(
    map_npz: Path, log: Path, plan_json: Path | None = None, frames: int = 60, scale: int = 1, title: str = "HIL"
) -> tuple[list[Image.Image], list[float]]:
    """Файлы прогона → кадры и покрытый м² по кадрам. Пространство — от первой точки трека, как в метрике."""
    lines = log.read_text(encoding="utf-8", errors="replace").splitlines()
    track = parse_track_log(lines)
    if not track:
        raise ValueError(f"нет строк TRACK в {log}")
    grid = load_grid_npz(map_npz)
    space = coverage_space(grid, track[0][1:3])
    known = np.asarray(grid.data).reshape(grid.height, grid.width) >= 0
    plan_xy = []
    if plan_json:
        plan_xy = [(p["x"], p["y"]) for p in json.loads(plan_json.read_text(encoding="utf-8")).get("poses", [])]
    rendered = render_frames(space, known, track, plan_xy, parse_em_times(lines), frames, scale, title=title)
    return [f for f, _ in rendered], [m2 for _, m2 in rendered]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="GIF/MP4 обхода покрытием: карта npz + лог с TRACK/EM_ON")
    ap.add_argument("map", type=Path)
    ap.add_argument("log", type=Path)
    ap.add_argument("out", type=Path, help="*.gif или *.mp4")
    ap.add_argument("--plan", type=Path, default=None, help="plan.json: точки поз плана")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--scale", type=int, default=1, help="пикселей на клетку карты")
    ap.add_argument("--max-mb", type=float, default=MAX_GIF_BYTES / 1048576, help="лимит GIF, МБ (Telegram — 2)")
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--title", default="Depot HIL")
    a = ap.parse_args(argv)
    imgs, covered = build(a.map, a.log, a.plan, a.frames, a.scale, a.title)
    if a.out.suffix.lower() == ".mp4":
        if not save_mp4(imgs, a.out, a.fps):
            print("ffmpeg не найден или упал", file=sys.stderr)
            return 2
        print(f"{a.out} {len(imgs)} frames covered_m2={covered[-1]:.1f}")
        return 0
    size, n, width = save_gif(imgs, a.out, int(a.max_mb * 1048576), a.fps)
    print(f"{a.out} {size} bytes {n} frames width={width} covered_m2={covered[-1]:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
