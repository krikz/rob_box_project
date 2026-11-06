#!/usr/bin/env python3
"""Генератор обложек канала @robbox_project (docs/channel/media/).

Рисует кадры в Pillow по собственному таймлайну (подход vizavi-motion: управляемый таймлайн, без внешних
движков): GIF с набором светодиодов до 381 и две статичные карточки. Никаких выдуманных данных: числа на
карточках взяты из репо (381 — `docker/vision/config/led_matrix/led_matrix_driver.yaml`, 131 шаг и 12 актов —
`docs/e2e/night-voice-marathon.md`).

    python -X utf8 scripts/telegram/make_channel_media.py [--out docs/channel/media]
"""
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont

BG = (10, 10, 15)
PANEL = (24, 26, 33)
GREEN = (74, 222, 128)
CYAN = (34, 211, 238)
DIM = (120, 128, 140)
RAINBOW = [(168, 0, 255), (255, 0, 140), (255, 70, 20), (255, 200, 0), (120, 255, 0), (0, 230, 200), (20, 90, 255)]

FONT_CANDIDATES = [
    "C:/Windows/Fonts/segoeuib.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]


def font(size: int) -> ImageFont.FreeTypeFont:
    for path in FONT_CANDIDATES:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    raise SystemExit("нет шрифта с кириллицей: поставь DejaVu или Segoe UI")


def ease(t: float) -> float:
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def led_frame(i: int, total: int, hold: int, w: int = 640, h: int = 360) -> Image.Image:
    """Кадр i: решётка светодиодов загорается волной, счётчик бежит до 381, камера чуть плывёт."""
    ss = 2  # суперсэмплинг для гладких точек
    img = Image.new("RGB", (w * ss, h * ss), BG)
    d = ImageDraw.Draw(img)
    run = total - hold
    p = ease(i / max(run - 1, 1)) if i < run else 1.0
    # параллакс: сетка дрейфует чуть быстрее текста
    drift = (i / total) * 8
    cols, rows = 24, 8
    step = 21 * ss
    x0 = (w * ss - (cols - 1) * step) // 2 - int(drift * ss)
    y0 = 24 * ss
    shown = 0
    glow = Image.new("RGB", img.size, (0, 0, 0))
    gd = ImageDraw.Draw(glow)
    for r in range(rows):
        for c in range(cols):
            idx = r * cols + c
            x, y = x0 + c * step, y0 + r * step
            lit = idx / (cols * rows) <= p
            if lit:
                shown += 1
                col = RAINBOW[(c + r) % len(RAINBOW)]
                # фронт волны чуть ярче
                edge = 1.0 if idx / (cols * rows) > p - 0.06 else 0.82
                col = tuple(int(v * edge) for v in col)
                rad = 6.2 * ss
                gd.ellipse(
                    [x - rad * 1.6, y - rad * 1.6, x + rad * 1.6, y + rad * 1.6], fill=tuple(v // 3 for v in col)
                )
            else:
                col = (34, 36, 44)
                rad = 5.2 * ss
            d.ellipse([x - rad, y - rad, x + rad, y + rad], fill=col)
    glow = glow.filter(ImageFilter.GaussianBlur(9 * ss))
    img = ImageChops.add(img, glow)
    d = ImageDraw.Draw(img)
    count = round(381 * p)
    big = font(92 * ss)
    txt = str(count)
    tw = d.textlength(txt, font=big)
    ty = 300 * ss - 100 * ss
    d.text(((w * ss - tw) / 2, ty), txt, font=big, fill=GREEN)
    sub = font(24 * ss)
    line = "светодиодов в лице РОББОКСА"
    sw = d.textlength(line, font=sub)
    alpha = ease((p - 0.5) / 0.5)
    col = tuple(int(BG[k] + (DIM[k] + 70 - BG[k]) * alpha) for k in range(3))
    d.text(((w * ss - sw) / 2, 318 * ss), line, font=sub, fill=col)
    return img.resize((w, h), Image.LANCZOS)


def make_led_gif(out: Path) -> None:
    total, hold = 44, 14
    frames = [led_frame(i, total, hold) for i in range(total)]
    pal = frames[-1].quantize(colors=96, method=Image.MEDIANCUT)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    durs = [60] * (total - hold) + [100] * (hold - 1) + [1800]
    q[0].save(out, save_all=True, append_images=q[1:], duration=durs, loop=0, optimize=True, disposal=1)


def card(out: Path, big_text: list[str], small: str, accent=GREEN, acts: int = 0, w: int = 1000, h: int = 560) -> None:
    img = Image.new("RGB", (w, h), BG)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([40, 40, w - 40, h - 40], radius=28, fill=PANEL)
    # полоска светодиодов сверху: тот же приём, что и в лице
    for c in range(30):
        x = 90 + c * 27
        d.ellipse([x, 78, x + 14, 92], fill=RAINBOW[c % len(RAINBOW)] if c % 3 else (40, 42, 50))
    f_big, f_small = font(74), font(30)
    y = 150
    for ln in big_text:
        d.text((90, y), ln, font=f_big, fill=accent if ln is big_text[0] else (235, 238, 245))
        y += 92
    for k in range(acts):  # по квадрату на акт; цвет нейтральный: итог актов тут не заявляется
        cx, cy = w - 330 + (k % 4) * 56, 170 + (k // 4) * 56
        d.rounded_rectangle([cx, cy, cx + 40, cy + 40], radius=8, fill=(52, 58, 70))
    d.text((90, h - 120), small, font=f_small, fill=DIM)
    d.text((w - 330, h - 120), "РОББОКС", font=f_small, fill=accent)
    img.save(out, optimize=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/channel/media")
    out = Path(ap.parse_args().out)
    out.mkdir(parents=True, exist_ok=True)
    make_led_gif(out / "led381.gif")
    card(out / "marathon.png", ["131 шаг,", "12 актов,", "одна ночь"], "Ночной голосовой марафон", acts=12)
    card(
        out / "code-decides.png",
        ["Код решает,", "LLM говорит"],
        "ADR-0148 · честный FAIL лучше красивого PASS",
        accent=CYAN,
    )


if __name__ == "__main__":
    main()
