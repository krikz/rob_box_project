#!/usr/bin/env python3
"""face_collage.py - коллаж снимков для «найди похожих» (issue #4469).

Собирает владелец данных (Vision Pi), как и одиночный снимок: сетка плиток, на каждой крупный номер ``1..k`` (плитка
искомого лица - буква ``T``), под плиткой короткий id и сходство в процентах. Текст только латиницей и цифрами:
шрифты OpenCV кириллицу не рисуют, а имя в подписи и не нужно - его говорит тул. Нет снимка у записи - серая плитка
с номером: номер всё равно значим, а решение «тот же человек» принимает человек по остальным плиткам.

Возвращает JPEG; потолок размера (:data:`people_service.PHOTO_MAX_BYTES`) обеспечивает вызывающий через ``fit_jpeg``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

#: Плитка: квадрат ``CELL`` пикселей под снимок плюс полоса подписи.
CELL = 200
CAPTION_H = 28
#: Столбцов в сетке: 12 кандидатов + искомое лицо = 13 плиток -> 4 ряда по 4.
COLUMNS = 4
JPEG_QUALITY = 80
TARGET_MARK = "T"


@dataclass(frozen=True)
class Tile:
    """Плитка коллажа: метка номера (``"1"``..., ``"T"`` у искомого), id (8 знаков), сходство 0..1 (``None`` у T)."""

    mark: str
    short_id: str
    similarity: Optional[float]
    jpeg: Optional[bytes]


def _caption(tile: Tile) -> str:
    return tile.short_id if tile.similarity is None else f"{tile.short_id} {round(tile.similarity * 100)}%"


def _paint(tile: Tile):
    """Одна плитка (BGR ``ndarray``): снимок по центру квадрата, номер в углу, подпись снизу."""
    import cv2  # noqa: PLC0415 - тяжёлая зависимость только для сборки коллажа
    import numpy as np  # noqa: PLC0415

    cell = np.full((CELL + CAPTION_H, CELL, 3), 40, np.uint8)
    img = cv2.imdecode(np.frombuffer(tile.jpeg, np.uint8), cv2.IMREAD_COLOR) if tile.jpeg else None
    if img is not None:
        scale = min(CELL / img.shape[1], CELL / img.shape[0])
        size = (max(1, int(img.shape[1] * scale)), max(1, int(img.shape[0] * scale)))
        img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
        top, left = (CELL - size[1]) // 2, (CELL - size[0]) // 2
        cell[top : top + size[1], left : left + size[0]] = img
    color = (0, 140, 255) if tile.mark == TARGET_MARK else (0, 0, 0)
    cv2.rectangle(cell, (0, 0), (62 if len(tile.mark) < 2 else 90, 58), color, -1)
    cv2.putText(cell, tile.mark, (6, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.putText(cell, _caption(tile), (6, CELL + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return cell


def build_collage(tiles: Sequence[Tile]) -> Optional[bytes]:
    """JPEG-коллаж из плиток по порядку (слева направо, сверху вниз) или ``None``: плиток нет или нет cv2."""
    if not tiles:
        return None
    try:
        import cv2  # noqa: PLC0415
        import numpy as np  # noqa: PLC0415

        painted = [_paint(t) for t in tiles]
        rows: List = []
        for start in range(0, len(painted), COLUMNS):
            row = painted[start : start + COLUMNS]
            row += [np.zeros_like(painted[0])] * (COLUMNS - len(row))
            rows.append(np.hstack(row))
        ok, buf = cv2.imencode(".jpg", np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return bytes(buf) if ok else None
    except Exception:  # noqa: BLE001 - нет cv2 или битый снимок: честный отказ, не пустая картинка
        return None


__all__ = ["CELL", "COLUMNS", "TARGET_MARK", "Tile", "build_collage"]
