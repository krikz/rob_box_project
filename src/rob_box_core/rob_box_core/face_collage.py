"""Shared ROS-free face collage renderer for Telegram and Quest.

The module intentionally knows nothing about ROS, Telegram, Quest, FaceStore,
or filesystem layout. Callers provide already-read image bytes and labels.
ADR-0135 requires one implementation shared by both operator surfaces.
"""

from __future__ import annotations

import io
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class FaceCollageTile:
    """One already-read image used by the shared collage renderer."""

    label: str
    width: int
    height: int
    jpeg_bytes: bytes


def build_face_collage(
    tiles: Sequence[FaceCollageTile],
    *,
    name_label: str,
    columns: int = 4,
    rows: int = 3,
    tile_size: Tuple[int, int] = (240, 320),
    pad: int = 6,
    caption_height: int = 22,
) -> Optional[bytes]:
    """Compose a fixed grid and return JPEG bytes, or None for no tiles.

    Image data is supplied by the caller so the shared module remains usable
    by both Telegram and Quest without depending on their record models.
    """
    if not tiles:
        return None
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None

    cols = max(1, columns)
    row_n = max(1, rows)
    tile_w, tile_h = tile_size
    cell_w = tile_w + 2 * pad
    cell_h = tile_h + caption_height + 2 * pad
    canvas = Image.new("RGB", (cols * cell_w, row_n * cell_h), color=(255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    font = _pick_font(caption_height - 4)
    safe_name = name_label.replace("\n", " ")

    positions = [(c, r) for r in range(row_n) for c in range(cols)]
    for idx, (col, row) in enumerate(positions):
        x0 = col * cell_w + pad
        y0 = row * cell_h + pad
        if idx >= len(tiles):
            _draw_empty_cell(draw, x0, y0, tile_w, tile_h + caption_height)
            continue

        tile = tiles[idx]
        try:
            tile_img = Image.open(io.BytesIO(tile.jpeg_bytes)).convert("RGB")
            tile_img = _cover_fit(tile_img, tile_w, tile_h)
            canvas.paste(tile_img, (x0, y0))
        except Exception:
            _draw_empty_cell(draw, x0, y0, tile_w, tile_h + caption_height)
        caption = f"{safe_name} {tile.width}x{tile.height}"
        _draw_caption(draw, x0, y0 + tile_h, tile_w, caption_height, caption, font)

    buf = io.BytesIO()
    canvas.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _pick_font(size: int):
    try:
        from PIL import ImageFont
        for candidate in (
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/TTF/DejaVuSans.ttf",
        ):
            try:
                return ImageFont.truetype(candidate, size=size)
            except OSError:
                continue
        return ImageFont.load_default()
    except Exception:
        return None


def _draw_caption(draw, x: int, y: int, w: int, h: int, text: str, font) -> None:
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255))
    try:
        draw.text((x + 4, y + 2), text, fill=(20, 20, 20), font=font)
    except Exception:
        pass


def _draw_empty_cell(draw, x: int, y: int, w: int, h: int) -> None:
    draw.rectangle(
        [x, y, x + w, y + h],
        fill=(255, 255, 255),
        outline=(220, 220, 220),
    )


def _cover_fit(img, w: int, h: int):
    iw, ih = img.size
    if iw == 0 or ih == 0:
        return img
    target_ratio = w / h
    src_ratio = iw / ih
    if src_ratio > target_ratio:
        new_w = int(ih * target_ratio)
        left = (iw - new_w) // 2
        img = img.crop((left, 0, left + new_w, ih))
    elif src_ratio < target_ratio:
        new_h = int(iw / target_ratio)
        top = (ih - new_h) // 2
        img = img.crop((0, top, iw, top + new_h))
    return img.resize((w, h))


__all__ = ["FaceCollageTile", "build_face_collage"]
