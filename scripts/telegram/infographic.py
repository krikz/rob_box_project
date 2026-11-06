#!/usr/bin/env python3
"""Инфографика канала @robbox_project в одном стиле: палитра, шрифт и подложка как у аватара и обложек.

Шаблоны: numbers (цифры), table (таблица результатов), nodes (схема узлов), before_after (было → стало),
timeline (таймлайн). Вход — JSON или YAML, выход — PNG (и GIF с поэлементным появлением, `--gif`).
Правило честности: у каждой цифры есть источник (файл или issue), без него генератор отказывается рисовать.
Стиль и формат данных описаны в docs/process/telegram-channel.md; примеры — docs/channel/infographics/.

    python -X utf8 scripts/telegram/infographic.py docs/channel/infographics/doorway.json docs/channel/media/doorway.png
    python -X utf8 scripts/telegram/infographic.py data.yaml out.png --gif
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_channel_media import BG, CYAN, DIM, GREEN, PANEL, RAINBOW, font  # noqa: E402

SIZES = {"card": (1000, 560), "wide": (1280, 720)}
RED = (248, 113, 113)
TEXT = (235, 238, 245)
MARGIN = 90
LIMITS = {"numbers": (2, 4), "nodes": (2, 4), "before_after": (1, 4), "timeline": (2, 6), "table": (1, 7)}
TEMPLATES = tuple(LIMITS)


class SpecError(ValueError):
    pass


def load_spec(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in (".yaml", ".yml"):
        import yaml

        return yaml.safe_load(text)
    return json.loads(text)


def validate_spec(spec: dict) -> None:
    """Каждая цифра обязана иметь источник; число элементов в пределах шаблона."""
    kind = spec.get("template")
    if kind not in TEMPLATES:
        raise SpecError(f"template должен быть из {TEMPLATES}, а не {kind!r}")
    if not spec.get("title"):
        raise SpecError("нет title")
    key = {"numbers": "items", "nodes": "nodes", "before_after": "pairs", "timeline": "events", "table": "rows"}[kind]
    items = spec.get(key) or []
    lo, hi = LIMITS[kind]
    if not lo <= len(items) <= hi:
        raise SpecError(f"{kind}: {key} должно быть {lo}–{hi}, а их {len(items)}")
    if kind == "table":
        if not spec.get("source"):
            raise SpecError("table: нужен source (файл или issue) для таблицы")
        if not spec.get("columns"):
            raise SpecError("table: нужны columns")
    elif kind == "nodes":
        if not spec.get("source"):
            raise SpecError("nodes: нужен source")
    else:
        for i, it in enumerate(items):
            if not it.get("source"):
                raise SpecError(f"{kind}[{i}]: нет source — без источника цифру не рисуем")


def sources_of(spec: dict) -> list:
    out = []
    if spec.get("source"):
        out.append(spec["source"])
    key = {"numbers": "items", "before_after": "pairs", "timeline": "events"}.get(spec["template"])
    for it in spec.get(key, []) if key else []:
        if it["source"] not in out:
            out.append(it["source"])
    return out


def mix(color, alpha: float, base=PANEL):
    alpha = min(max(alpha, 0.0), 1.0)
    return tuple(int(base[k] + (color[k] - base[k]) * alpha) for k in range(3))


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


def base_canvas(spec: dict, size: tuple):
    w, h = size
    img = Image.new("RGB", size, BG)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([40, 40, w - 40, h - 40], radius=28, fill=PANEL)
    for c in range((w - 2 * MARGIN) // 27):  # полоска светодиодов — тот же приём, что у обложек
        x = MARGIN + c * 27
        d.ellipse([x, 78, x + 14, 92], fill=RAINBOW[c % len(RAINBOW)] if c % 3 else (40, 42, 50))
    d.text((MARGIN, 118), spec["title"], font=font(40), fill=TEXT)
    if spec.get("subtitle"):
        d.text((MARGIN, 174), spec["subtitle"], font=font(24), fill=DIM)
    src = "Источник: " + "; ".join(sources_of(spec))
    sf = font(17)
    lines = wrap(d, src, sf, w - 2 * MARGIN - 150)[:2]
    for i, ln in enumerate(lines):
        d.text((MARGIN, h - 112 + i * 24), ln, font=sf, fill=DIM)
    d.text((w - MARGIN - 130, h - 112), "РОББОКС", font=font(24), fill=GREEN)
    return img, d


def draw_numbers(d, spec, size, reveal):
    items, w = spec["items"], size[0]
    col = (w - 2 * MARGIN) // len(items)
    for i, it in enumerate(items):
        a = reveal - i
        x = MARGIN + i * col
        d.text((x, 230), str(it["value"]), font=font(84 if len(items) < 4 else 68), fill=mix(GREEN, a))
        for j, ln in enumerate(wrap(d, it["label"], font(22), col - 30)[:4]):
            d.text((x, 335 + j * 28), ln, font=font(22), fill=mix(TEXT, a))


def draw_table(d, spec, size, reveal):
    cols, rows, w = spec["columns"], spec["rows"], size[0]
    cw = (w - 2 * MARGIN) / len(cols)
    f = font(24)
    y0 = 225
    for k, name in enumerate(cols):
        d.text((MARGIN + k * cw, y0), name, font=f, fill=DIM)
    for r, row in enumerate(rows):
        a = reveal - r
        y = y0 + 46 + r * 40
        for k, cell in enumerate(row):
            text, status = (cell["text"], cell.get("status")) if isinstance(cell, dict) else (str(cell), None)
            color = {"ok": GREEN, "fail": RED}.get(status, TEXT)
            d.text((MARGIN + k * cw, y), text, font=f, fill=mix(color, a))


def draw_nodes(d, spec, size, reveal):
    nodes, w = spec["nodes"], size[0]
    gap = 24
    bw = (w - 2 * MARGIN - gap * (len(nodes) - 1)) // len(nodes)
    for i, node in enumerate(nodes):
        a = reveal - i
        x = MARGIN + i * (bw + gap)
        d.rounded_rectangle([x, 215, x + bw, 440], radius=16, fill=mix((52, 58, 70), a), outline=mix(CYAN, a), width=2)
        d.text((x + 18, 228), node["name"], font=font(30), fill=mix(CYAN, a))
        if node.get("role"):
            d.text((x + 18, 268), node["role"], font=font(18), fill=mix(DIM, a))
        y = 300
        for it in node.get("items", []):
            for ln in wrap(d, it, font(20), bw - 36):
                if y < 430:
                    d.text((x + 18, y), ln, font=font(20), fill=mix(TEXT, a))
                    y += 26


def draw_before_after(d, spec, size, reveal):
    for i, p in enumerate(spec["pairs"]):
        a = reveal - i
        y = 225 + i * 72
        label = p["label"] + (" · " + p["note"] if p.get("note") else "")
        d.text((MARGIN, y), label, font=font(22), fill=mix(DIM, a))
        d.text((MARGIN, y + 28), str(p["before"]), font=font(34), fill=mix(TEXT, a))
        d.text((MARGIN + 300, y + 22), "→", font=font(40), fill=mix(DIM, a))
        after = RED if p.get("status") == "fail" else GREEN
        d.text((MARGIN + 380, y + 28), str(p["after"]), font=font(34), fill=mix(after, a))


def draw_timeline(d, spec, size, reveal):
    events = spec["events"]
    d.line([MARGIN + 8, 235, MARGIN + 8, 235 + 50 * (len(events) - 1)], fill=(52, 58, 70), width=3)
    for i, ev in enumerate(events):
        a = reveal - i
        y = 225 + i * 50
        d.ellipse([MARGIN, y + 2, MARGIN + 16, y + 18], fill=mix(RAINBOW[i % len(RAINBOW)], a))
        d.text((MARGIN + 40, y - 2), ev["date"], font=font(24), fill=mix(GREEN, a))
        d.text((MARGIN + 190, y - 2), ev["text"], font=font(24), fill=mix(TEXT, a))


DRAWERS = {
    "numbers": draw_numbers,
    "table": draw_table,
    "nodes": draw_nodes,
    "before_after": draw_before_after,
    "timeline": draw_timeline,
}


def render(spec: dict, size_name: str = "card", reveal: float = 99.0) -> Image.Image:
    validate_spec(spec)
    size = SIZES[size_name]
    img, d = base_canvas(spec, size)
    DRAWERS[spec["template"]](d, spec, size, reveal)
    return img


def render_gif(spec: dict, out: Path, size_name: str = "card") -> None:
    n = len(
        spec[
            {"numbers": "items", "nodes": "nodes", "before_after": "pairs", "timeline": "events", "table": "rows"}[
                spec["template"]
            ]
        ]
    )
    steps = 4  # кадров на появление одного элемента
    frames = [render(spec, size_name, k / steps) for k in range(n * steps + 1)]
    pal = frames[-1].quantize(colors=64, method=Image.MEDIANCUT)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    durs = [70] * (len(q) - 1) + [2500]
    q[0].save(out, save_all=True, append_images=q[1:], duration=durs, loop=0, optimize=True, disposal=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("spec", type=Path, help="YAML/JSON с данными")
    ap.add_argument("out", type=Path, help="PNG (или GIF с --gif)")
    ap.add_argument("--size", choices=SIZES, default="card", help="card 1000x560, wide 1280x720")
    ap.add_argument("--gif", action="store_true", help="анимированный вариант: элементы появляются по очереди")
    args = ap.parse_args()
    spec = load_spec(args.spec)
    try:
        if args.gif:
            render_gif(spec, args.out, args.size)
        else:
            render(spec, args.size).save(args.out, optimize=True)
    except SpecError as e:
        print(f"{args.spec}: {e}")
        return 1
    print(f"{args.out}: {args.out.stat().st_size} байт")
    return 0


if __name__ == "__main__":
    sys.exit(main())
