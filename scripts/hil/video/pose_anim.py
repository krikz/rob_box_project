#!/usr/bin/env python3
"""Анимация позы по данным прогона HIL: вид сверху, «где робот на самом деле» против «где он думает».

Перенос `hil_anim.py` координатора (#4513) в репо. Рисует PIL, без matplotlib: на машине студии (builder)
matplotlib нет, а PIL и numpy есть. Источников данных два, оба — настоящие записи прогона, ничего не дорисовано:

  * ``maprec/poses.csv`` (``t,tx,ty,tyaw,ex,ey,eyaw``): правда Gazebo (``tx…``) и оценка rtabmap (``ex…``,
    пусто, пока оценки нет). Это прогоны дрейфа локализации на складе Depot (#4495);
  * TRACK-строки логов hp2 (``HH:MM:SS.s TRACK gt=(x,y,yaw°) … em=0|1``) — полигон с проёмами, как у
    исходного ``hil_anim.py``: зелёный — едет, красный — стоп защиты.

Модуль только рисует кадры (PIL.Image). Сборку в MP4/GIF делает вызывающий (studio_render, ffmpeg).

    pose_anim.py track LOG_DIR OUT_DIR name=заголовок …   # GIF по TRACK-логам, как hil_anim.py
"""

from __future__ import annotations

import bisect
import csv
import math
import re
import sys
from pathlib import Path
from typing import Iterator, Optional

from PIL import Image, ImageDraw, ImageFont

# --------------------------------------------------------------------------- данные

TRACK_RE = re.compile(r"^(\d\d):(\d\d):(\d\d\.\d+) TRACK gt=\(([-\d.]+),([-\d.]+),(-?\d+)°\).*em=(\d)")
RESULT_RE = re.compile(r"RESULT (\S+) wall=([\d.]+)s")
MOVE_EPS = 0.05  # м: с какого сдвига правды считаем, что робот поехал
LEAD_S = 2.0  # с до начала движения, которые оставляем в ролике


class Row:
    """Одна строка позы: время, правда (x, y, yaw) и оценка (может быть None) или флаг стопа."""

    __slots__ = ("t", "tx", "ty", "tyaw", "ex", "ey", "eyaw", "stop")

    def __init__(self, t, tx, ty, tyaw, ex=None, ey=None, eyaw=None, stop=False):
        self.t, self.tx, self.ty, self.tyaw = t, tx, ty, tyaw
        self.ex, self.ey, self.eyaw, self.stop = ex, ey, eyaw, stop

    @property
    def has_estimate(self) -> bool:
        return self.ex is not None and self.ey is not None

    @property
    def error(self) -> Optional[float]:
        """Ошибка позы, м: расстояние между правдой и оценкой. None — оценки ещё нет."""
        if not self.has_estimate:
            return None
        return math.hypot(self.ex - self.tx, self.ey - self.ty)


def _num(text: str) -> Optional[float]:
    text = (text or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def load_poses_csv(path: Path) -> list:
    """Строки maprec/poses.csv. Строки без правды пропускаются; оценка может быть пустой."""
    rows = []
    with open(path, encoding="utf-8", errors="replace") as handle:
        for rec in csv.DictReader(handle):
            t, tx, ty = _num(rec.get("t")), _num(rec.get("tx")), _num(rec.get("ty"))
            if t is None or tx is None or ty is None:
                continue
            rows.append(
                Row(
                    t,
                    tx,
                    ty,
                    _num(rec.get("tyaw")) or 0.0,
                    _num(rec.get("ex")),
                    _num(rec.get("ey")),
                    _num(rec.get("eyaw")),
                )
            )
    return rows


def load_track_log(path: Path) -> tuple:
    """TRACK-строки лога hp2 (как в hil_anim.py): (строки, итог прогона строкой)."""
    t0, rows, result = None, [], ""
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        match = TRACK_RE.match(line)
        if match:
            t = int(match.group(1)) * 3600 + int(match.group(2)) * 60 + float(match.group(3))
            t0 = t if t0 is None else t0
            yaw = math.radians(int(match.group(6)))
            rows.append(Row(t - t0, float(match.group(4)), float(match.group(5)), yaw, stop=match.group(7) != "0"))
        found = RESULT_RE.search(line)
        if found:
            result = f"{found.group(1).replace('STATUS_', '')} за {float(found.group(2)):.1f} с"
    return rows, result


def motion_start(rows: list) -> float:
    """Время, когда правда сдвинулась от старта больше MOVE_EPS (минус LEAD_S). Пусто — 0."""
    if not rows:
        return 0.0
    x0, y0 = rows[0].tx, rows[0].ty
    for row in rows:
        if math.hypot(row.tx - x0, row.ty - y0) > MOVE_EPS:
            return max(rows[0].t, row.t - LEAD_S)
    return rows[0].t


def summary(rows: list) -> dict:
    """Цифры прогона для подписей: их считает код из данных, а не модель."""
    errors = [(r.t, r.error) for r in rows if r.error is not None]
    start = motion_start(rows)
    out = {"duration": (rows[-1].t - start) if rows else 0.0, "samples": len(rows), "with_estimate": len(errors)}
    if errors:
        t_max, e_max = max(errors, key=lambda item: item[1])
        out.update(max_error=e_max, t_max=t_max - start, final_error=errors[-1][1])
    return out


# --------------------------------------------------------------------------- геометрия кадра


class Viewport:
    """Мир (м) → пиксели панели. Одинаковый масштаб по осям, ось y вверх."""

    def __init__(self, bounds: tuple, box: tuple):
        x0, y0, x1, y1 = bounds
        bx0, by0, bx1, by1 = box
        self.box = box
        scale = min((bx1 - bx0) / max(x1 - x0, 1e-6), (by1 - by0) / max(y1 - y0, 1e-6))
        self.scale = scale
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.ox = (bx0 + bx1) / 2 - cx * scale
        self.oy = (by0 + by1) / 2 + cy * scale

    def px(self, x: float, y: float) -> tuple:
        return self.ox + x * self.scale, self.oy - y * self.scale


def bounds(runs: list, pad: float = 1.0) -> tuple:
    """Общий прямоугольник мира для нескольких прогонов: один масштаб — честное сравнение."""
    xs, ys = [], []
    for rows in runs:
        for r in rows:
            xs.append(r.tx)
            ys.append(r.ty)
            if r.has_estimate:
                xs.append(r.ex)
                ys.append(r.ey)
    if not xs:
        return (-1.0, -1.0, 1.0, 1.0)
    return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)


def index_at(rows: list, t: float) -> int:
    """Последняя строка не позже t (по времени прогона). -1 — ещё ничего."""
    return bisect.bisect_right([r.t for r in rows], t) - 1


# --------------------------------------------------------------------------- рисование

FONT_PATHS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    "C:/Windows/Fonts/arial.ttf",
)
BOLD_PATHS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
)
_FONTS: dict = {}

BG = (14, 16, 22)
PANEL = (26, 29, 38)
GRID = (40, 44, 56)
WALL = (120, 126, 140)
TRUTH = (60, 220, 120)
ESTIMATE = (255, 90, 80)
TEXT = (235, 238, 245)
DIM = (150, 156, 170)
TRAIL_POINTS = 700  # след прорежен до стольких точек: кадр рисуется быстро и на длинном прогоне


def font(size: int, bold: bool = False):
    """Шрифт с кириллицей; нет TTF — встроенный (текст будет латиницей-квадратиками, но кадр соберётся)."""
    key = (size, bold)
    if key not in _FONTS:
        loaded = None
        for path in BOLD_PATHS if bold else FONT_PATHS:
            try:
                loaded = ImageFont.truetype(path, size)
                break
            except OSError:
                continue
        _FONTS[key] = loaded or ImageFont.load_default()
    return _FONTS[key]


def map_layer(snapshot: dict, view: Viewport, size: tuple) -> Image.Image:
    """Слой карты rtabmap (занятые клетки серым) — один раз на снимок, потом вставляется в кадры.

    Клетки вне панели не рисуются: карта «до» уплывшая и шире мира прогона, без обрезки она
    залезала на заголовок и на соседнюю панель (живая сборка 06.11, #4513).
    """
    layer = Image.new("RGBA", size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    res, ox, oy, data = snapshot["resolution"], snapshot["origin_x"], snapshot["origin_y"], snapshot["data"]
    half = max(1.0, res * view.scale / 2)
    bx0, by0, bx1, by1 = view.box
    for row_i, col_i in occupied_cells(data):
        x, y = view.px(ox + (col_i + 0.5) * res, oy + (row_i + 0.5) * res)
        if not (bx0 + half <= x <= bx1 - half and by0 + half <= y <= by1 - half):
            continue
        draw.rectangle([x - half, y - half, x + half, y + half], fill=WALL + (200,))
    return layer


def occupied_cells(data, threshold: int = 65) -> list:
    """(строка, столбец) занятых клеток OccupancyGrid: numpy, если есть, иначе списки."""
    if hasattr(data, "shape"):
        import numpy as np

        rows, cols = np.nonzero(data >= threshold)
        return list(zip(rows.tolist(), cols.tolist()))
    return [(r, c) for r, line in enumerate(data) for c, value in enumerate(line) if value >= threshold]


def _robot(draw, view: Viewport, x: float, y: float, yaw: float, color: tuple, radius: int = 9) -> None:
    px, py = view.px(x, y)
    draw.ellipse([px - radius, py - radius, px + radius, py + radius], fill=color, outline=(0, 0, 0), width=2)
    hx, hy = px + 2.2 * radius * math.cos(yaw), py - 2.2 * radius * math.sin(yaw)
    draw.line([px, py, hx, hy], fill=(0, 0, 0), width=3)


def _trail(rows: list, upto: int, attr: tuple) -> list:
    step = max(1, (upto + 1) // TRAIL_POINTS)
    points = []
    for r in rows[: upto + 1 : step]:  # noqa: E203
        x, y = getattr(r, attr[0]), getattr(r, attr[1])
        if x is not None and y is not None:
            points.append((x, y))
    return points


def draw_panel(img: Image.Image, view: Viewport, rows: list, upto: int, layer: Optional[Image.Image] = None) -> None:
    """Вид сверху одного прогона до строки upto: карта, два следа, две позы, линия ошибки."""
    x0, y0, x1, y1 = view.box
    draw = ImageDraw.Draw(img, "RGBA")
    draw.rectangle([x0, y0, x1, y1], fill=PANEL)
    if layer is not None:
        img.paste(layer, (0, 0), layer)
    if upto < 0:
        return
    # Правда — толще и снизу, оценка — тоньше сверху: когда они совпадают, видны обе.
    for attr, color, width in ((("tx", "ty"), TRUTH, 5), (("ex", "ey"), ESTIMATE, 2)):
        points = [view.px(x, y) for x, y in _trail(rows, upto, attr)]
        if len(points) > 1:
            draw.line(points, fill=color + (190,), width=width)
    row = rows[upto]
    if row.has_estimate:
        draw.line([view.px(row.tx, row.ty), view.px(row.ex, row.ey)], fill=(255, 220, 80, 220), width=2)
        _robot(draw, view, row.ex, row.ey, row.eyaw or 0.0, ESTIMATE)
    _robot(draw, view, row.tx, row.ty, row.tyaw, (210, 60, 60) if row.stop else TRUTH)


def draw_error_strip(img: Image.Image, box: tuple, rows: list, start: float, span: float, t_rel: float, top: float):
    """Полоса «ошибка позы во времени» под панелью: общий масштаб по y для всех панелей."""
    x0, y0, x1, y1 = box
    draw = ImageDraw.Draw(img)
    draw.rectangle([x0, y0, x1, y1], fill=PANEL)
    top = max(top, 0.5)
    points = []
    for r in rows:
        err = r.error
        rel = r.t - start
        if err is None or rel < 0 or rel > t_rel:
            continue
        points.append((x0 + (x1 - x0) * rel / span, y1 - 4 - (y1 - y0 - 8) * min(err, top) / top))
    if len(points) > 1:
        draw.line(points, fill=ESTIMATE, width=2)
    cursor = x0 + (x1 - x0) * min(t_rel, span) / span
    draw.line([cursor, y0, cursor, y1], fill=DIM, width=1)
    draw.text((x0 + 6, y0 + 2), f"ошибка, до {top:.0f} м", font=font(14), fill=DIM)


# --------------------------------------------------------------------------- «до/после» бок о бок


class Run:
    """Прогон для монтажа: строки позы, подпись и (необязательно) снимки карты по времени."""

    def __init__(self, rows: list, label: str, maps: Optional[list] = None):
        self.rows = rows
        self.label = label
        self.maps = sorted(maps or [], key=lambda m: m["t"])  # [{"t", "resolution", "origin_x", …, "data"}]
        self.start = motion_start(rows)
        self.stats = summary(rows)
        self._layers: dict = {}
        self.peak = []  # максимум ошибки к строке i: «худшее до сих пор» в подписи кадра
        worst = None
        for row in rows:
            err = row.error
            worst = err if worst is None or (err is not None and err > worst) else worst
            self.peak.append(worst)

    @property
    def span(self) -> float:
        return max(1.0, self.stats["duration"])

    def layer(self, t_abs: float, view: Viewport, size: tuple) -> Optional[Image.Image]:
        stamps = [m["t"] for m in self.maps]
        i = bisect.bisect_right(stamps, t_abs) - 1
        if i < 0:
            return None
        if i not in self._layers:
            self._layers = {i: map_layer(self.maps[i], view, size)}
        return self._layers[i]


def side_by_side_frames(
    runs: list, title: str, frames: int, size: tuple = (1280, 720), hold: int = 0
) -> Iterator[Image.Image]:
    """Кадры ролика «до/после»: прогоны рядом в одном масштабе и с общим временем от начала движения.

    hold — сколько последних кадров стоять на финале (итоговые цифры видны дольше).
    """
    w, h = size
    span = max(run.span for run in runs)
    world = bounds([run.rows for run in runs])
    top_err = max([run.stats.get("max_error", 0.0) for run in runs] + [1.0])
    col = (w - 30 * (len(runs) + 1)) // len(runs)
    head, strip = int(h * 0.15), int(h * 0.12)
    views = [
        Viewport(world, (30 + i * (col + 30), head, 30 + i * (col + 30) + col, h - strip - 46))
        for i in range(len(runs))
    ]
    moving = max(1, frames - hold)
    for k in range(frames):
        t_rel = span * min(k, moving - 1) / max(1, moving - 1)
        img = Image.new("RGB", size, BG)
        draw = ImageDraw.Draw(img)
        draw.text((30, 14), title, font=font(max(18, h // 26), bold=True), fill=TEXT)
        legend = "зелёный — где робот на самом деле (Gazebo)   красный — где он думает (rtabmap)"
        draw.text((30, 14 + h // 20), legend, font=font(max(12, h // 48)), fill=DIM)
        for run, view in zip(runs, views):
            _draw_run(img, run, view, t_rel, (w, h), span, top_err, strip)
        draw.text((30, h - 34), f"t = {t_rel:5.0f} с от начала движения", font=font(max(12, h // 40)), fill=DIM)
        yield img


def stacked_frames(
    runs: list, title: str, frames: int, size: tuple = (720, 1280), hold: int = 0, font_hint: Optional[int] = None
) -> Iterator[Image.Image]:
    """Кадры ролика «до/после» стопкой: «до» сверху, «после» снизу.

    Просьба Шифу (07.11): «видео вертикальным, чтобы до было сверху, а
    после снизу». Кадр вертикальный (720×1280), прогоны друг под другом, у
    обоих один масштаб по осям — как в горизонтальном варианте.
    """
    w, h = size
    span = max(run.span for run in runs)
    world = bounds([run.rows for run in runs])
    top_err = max([run.stats.get("max_error", 0.0) for run in runs] + [1.0])
    count = len(runs)
    head = int(h * 0.13)
    strip = int(h * 0.075)
    gap = int(h * 0.02)
    body = h - head - strip - 46 - gap * (count + 1)
    panel_h = max(60, (body - count * strip) // count)
    views = []
    for i in range(count):
        top = head + gap + i * (panel_h + strip + gap)
        views.append(Viewport(world, (30, top, w - 30, top + panel_h)))
    moving = max(1, frames - hold)
    for k in range(frames):
        t_rel = span * min(k, moving - 1) / max(1, moving - 1)
        img = Image.new("RGB", size, BG)
        draw = ImageDraw.Draw(img)
        draw.text((30, 16), title, font=font(max(18, w // 26), bold=True), fill=TEXT)
        legend = "зелёный — где робот на самом деле (Gazebo), красный — оценка"
        draw.text((30, 16 + h // 40), legend, font=font(max(12, w // 46)), fill=DIM)
        for run, view in zip(runs, views):
            _draw_run(img, run, view, t_rel, (w, h), span, top_err, strip, font_hint=font_hint or w)
        draw.text((30, h - 34), f"t = {t_rel:5.0f} с от начала движения", font=font(max(12, w // 46)), fill=DIM)
        yield img


def meters(value: Optional[float]) -> str:
    """«11.9 м» / «0.08 м»: меньше метра — с сотыми, чтобы «после» не читалось как ровный ноль."""
    if value is None:
        return "—"
    return f"{value:.1f} м" if value >= 1.0 else f"{value:.2f} м"


def _draw_run(
    img, run: Run, view: Viewport, t_rel: float, size: tuple, span: float, top_err: float, strip: int, font_hint=None
):
    t_abs = run.start + t_rel
    upto = index_at(run.rows, t_abs)
    draw_panel(img, view, run.rows, upto, run.layer(t_abs, view, size))
    x0, y0, x1, y1 = view.box
    draw = ImageDraw.Draw(img)
    # Шрифты — по ширине панели: в вертикальном кадре высота большая, а текст
    # всё равно должен помещаться по ширине (Шифу, 07.11).
    hint = font_hint or size[1]
    small = font(max(12, hint // 40))
    draw.text((x0 + 8, y0 + 6), run.label, font=font(max(14, hint // 34), bold=True), fill=TEXT)
    row = run.rows[upto] if upto >= 0 else None
    err = row.error if row is not None else None
    if err is None:
        status = "оценки позы ещё нет"
    else:
        status = f"ошибка позы {meters(err)} · худшая {meters(run.peak[upto])}"
    color = ESTIMATE if (err or 0) > 1.0 else TRUTH
    if t_rel > run.span:
        # Отдельной строкой: в одну с ошибкой не влезала в панель 1280×720 (живая сборка 06.11).
        draw.text((x0 + 8, y1 - 52), f"прогон закончился ({run.span:.0f} с)", font=small, fill=DIM)
    draw.text((x0 + 8, y1 - 28), status, font=small, fill=color)
    draw_error_strip(img, (x0, y1 + 8, x1, y1 + 8 + strip - 12), run.rows, run.start, span, t_rel, top_err)


# --------------------------------------------------------------------------- полигон (как hil_anim.py)

WALL_T = 0.1
GAPS = ((2.0, 1.2), (4.0, 1.0), (6.0, 0.9), (8.0, 0.8))
BODY_L, BODY_W = 0.61, 0.605


def polygon_walls() -> list:
    """Стены полигона hp2 (x, y, ширина, высота), м — те же, что в hil_anim.py."""
    walls = [
        (-1.55, 3.0 - WALL_T / 2, 12.1, WALL_T),
        (-1.55, -3.0 - WALL_T / 2, 12.1, WALL_T),
        (-1.5 - WALL_T / 2, -3.0, WALL_T, 6.0),
        (10.5 - WALL_T / 2, -3.0, WALL_T, 6.0),
    ]
    for x, gap in GAPS:
        walls.append((x - WALL_T / 2, gap / 2, WALL_T, 3.0 - gap / 2))
        walls.append((x - WALL_T / 2, -3.0, WALL_T, 3.0 - gap / 2))
    return walls


def body(x: float, y: float, yaw: float) -> list:
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(BODY_L / 2, BODY_W / 2), (-BODY_L / 2, BODY_W / 2), (-BODY_L / 2, -BODY_W / 2), (BODY_L / 2, -BODY_W / 2)]
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in pts]


def track_frames(rows: list, title: str, result: str, max_frames: int = 160, size: tuple = (640, 520)):
    """Кадры полигона по TRACK-логу: стены, след, стопы защиты красным, корпус робота."""
    if not rows:
        return
    step = max(1, math.ceil(len(rows) / max_frames))
    idx = list(range(0, len(rows), step))
    if idx[-1] != len(rows) - 1:
        idx.append(len(rows) - 1)
    view = Viewport(bounds([rows], pad=1.0), (10, 60, size[0] - 10, size[1] - 10))
    for i in idx:
        img = Image.new("RGB", size, (250, 250, 250))
        draw = ImageDraw.Draw(img)
        for wx, wy, ww, wh in polygon_walls():
            a, b = view.px(wx, wy + wh), view.px(wx + ww, wy)
            draw.rectangle([a, b], fill=(51, 51, 51))
        trail = [view.px(r.tx, r.ty) for r in rows[: i + 1]]
        if len(trail) > 1:
            draw.line(trail, fill=(31, 119, 180), width=2)
        for r in rows[: i + 1]:
            if r.stop:
                sx, sy = view.px(r.tx, r.ty)
                draw.ellipse([sx - 2, sy - 2, sx + 2, sy + 2], fill=(214, 39, 40))
        row = rows[i]
        draw.polygon(
            [view.px(*p) for p in body(row.tx, row.ty, row.tyaw)], fill=(214, 39, 40) if row.stop else (44, 160, 44)
        )
        draw.text((10, 6), f"{title}  {result}", font=font(14, bold=True), fill=(20, 20, 20))
        draw.text((10, 30), f"t = {row.t:5.1f} с" + ("   СТОП" if row.stop else ""), font=font(13), fill=(20, 20, 20))
        yield img


def save_gif(frames: list, out: Path, fps: int = 10, colors: int = 64) -> int:
    """Кадры → GIF одной палитрой (палитра по последнему кадру). Возвращает размер файла."""
    if not frames:
        raise ValueError("нет кадров для GIF")
    palette = frames[-1].quantize(colors=colors, method=Image.MEDIANCUT)
    quant = [f.quantize(palette=palette, dither=Image.Dither.NONE if hasattr(Image, "Dither") else 0) for f in frames]
    durations = [int(1000 / fps)] * (len(quant) - 1) + [2500]
    quant[0].save(out, save_all=True, append_images=quant[1:], duration=durations, loop=0, optimize=True, disposal=1)
    return out.stat().st_size


def main(argv: list) -> int:
    if len(argv) < 4 or argv[1] != "track":
        print(__doc__)
        return 2
    src, dst = Path(argv[2]), Path(argv[3])
    dst.mkdir(parents=True, exist_ok=True)
    for spec in argv[4:]:
        name, title = spec.split("=", 1)
        rows, result = load_track_log(src / f"{name}.log")
        frames = list(track_frames(rows, title, result))
        if not frames:
            print(name, "no track")
            continue
        print(name, "ok", save_gif(frames, dst / f"{name}.gif"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
