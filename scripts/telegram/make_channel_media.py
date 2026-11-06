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


BADGE_TEXT = "РОББОКС"
EDGE = 90  # поле слева и справа до текста
CAPTION_GAP = 30  # зазор между нижней подписью и знаком


class LayoutError(ValueError):
    """Текст не помещается в отведённое место даже самым мелким шрифтом: честная ошибка вместо наложения."""


class RecordingDraw(ImageDraw.ImageDraw):
    """ImageDraw, который запоминает рамки (textbbox) всего нарисованного текста: для проверки наложений."""

    def __init__(self, im, boxes: list | None = None):
        super().__init__(im)
        self.boxes = boxes

    def text(self, xy, text, fill=None, font=None, *args, **kwargs):
        if self.boxes is not None and str(text).strip():
            self.boxes.append((str(text), self.textbbox(xy, str(text), font=font)))
        return super().text(xy, text, fill, font, *args, **kwargs)


def wrap(d: ImageDraw.ImageDraw, text: str, fnt, width: int) -> list:
    lines, cur = [], ""
    for word in text.split():
        trial = (cur + " " + word).strip()
        if d.textlength(trial, font=fnt) <= width or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    return lines + ([cur] if cur else [])


def fit_caption(d: ImageDraw.ImageDraw, text: str, sizes: tuple, max_w: int, max_lines: int = 2) -> tuple:
    """(шрифт, строки): одна строка самым крупным шрифтом, который влезает; иначе перенос на вторую строку
    (тоже от крупного к мелкому). Не влезло и в минимальном размере — LayoutError, а не наложение."""
    for lines_allowed in range(1, max_lines + 1):
        for size in sizes:
            fnt = font(size)
            lines = wrap(d, text, fnt, max_w)
            if len(lines) <= lines_allowed and all(d.textlength(ln, font=fnt) <= max_w for ln in lines):
                return fnt, lines
    raise LayoutError(f"подпись {text!r} не помещается в {max_w} px за {max_lines} строки при шрифте {sizes[-1]}")


def draw_footer(d: ImageDraw.ImageDraw, text: str, size: tuple, sizes: tuple, badge_size: int, y: int, accent) -> None:
    """Нижняя подпись и знак РОББОКС: знак прижат вправо и резервирует свою зону, подпись в неё не заходит."""
    w, _ = size
    badge_font = font(badge_size)
    badge_x = w - EDGE - int(d.textlength(BADGE_TEXT, font=badge_font))
    fnt, lines = fit_caption(d, text, sizes, badge_x - EDGE - CAPTION_GAP)
    step = int(fnt.size * 1.3)
    top = y - (step // 2 if len(lines) > 1 else 0)  # две строки: центр блока остаётся на месте подписи
    for i, ln in enumerate(lines):
        d.text((EDGE, top + i * step), ln, font=fnt, fill=DIM)
    d.text((badge_x, y), BADGE_TEXT, font=badge_font, fill=accent)


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


def card(
    out: Path | None,
    big_text: list[str],
    small: str,
    accent=GREEN,
    acts: int = 0,
    w: int = 1000,
    h: int = 560,
    boxes: list | None = None,
    bg: Image.Image | None = None,
) -> Image.Image:
    img = Image.new("RGB", (w, h), BG)
    d = RecordingDraw(img, boxes)
    d.rounded_rectangle([40, 40, w - 40, h - 40], radius=28, fill=PANEL)
    if bg is not None:
        import channel_bg

        channel_bg.apply_background(img, bg)
    # полоска светодиодов сверху: тот же приём, что и в лице
    for c in range(30):
        x = 90 + c * 27
        d.ellipse([x, 78, x + 14, 92], fill=RAINBOW[c % len(RAINBOW)] if c % 3 else (40, 42, 50))
    f_big = font(74)
    y = 150
    for ln in big_text:
        d.text((90, y), ln, font=f_big, fill=accent if ln is big_text[0] else (235, 238, 245))
        y += 92
    for k in range(acts):  # по квадрату на акт; цвет нейтральный: итог актов тут не заявляется
        cx, cy = w - 330 + (k % 4) * 56, 170 + (k // 4) * 56
        d.rounded_rectangle([cx, cy, cx + 40, cy + 40], radius=8, fill=(52, 58, 70))
    draw_footer(d, small, (w, h), (30, 27, 24, 21), 30, h - 120, accent)
    if out is not None:
        img.save(out, optimize=True)
    return img


CARD_BACKGROUNDS = {  # декоративные подложки карточек (channel_bg): абстракция по теме поста, не «фото робота»
    "marathon": {
        "slug": "bg-night",
        "prompt": "Abstract calm night scene, deep indigo sky, soft concentric sound ripples, faint moon glow",
        "seed": 8,
    },
    "code-decides": {
        "slug": "bg-code",
        "prompt": "Abstract geometric bracket shapes balanced against flowing speech waves, dark cyan and teal glow",
        "seed": 9,
    },
}


def main() -> None:
    import channel_bg

    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/channel/media")
    ap.add_argument("--no-bg", action="store_true", help="без подложек MiniMax (только кэш не используется)")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    def bg_for(name):
        return None if args.no_bg else channel_bg.load_background(CARD_BACKGROUNDS[name])

    make_led_gif(out / "led381.gif")
    card(
        out / "marathon.png",
        ["131 шаг,", "12 актов,", "одна ночь"],
        "Ночной голосовой марафон",
        acts=12,
        bg=bg_for("marathon"),
    )
    card(
        out / "code-decides.png",
        ["Код решает,", "LLM говорит"],
        "ADR-0148 · честный FAIL лучше красивого PASS",
        accent=CYAN,
        bg=bg_for("code-decides"),
    )


if __name__ == "__main__":
    main()
