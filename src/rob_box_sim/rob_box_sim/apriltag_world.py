"""AprilTag tag36h11 в мирах сима: текстура тега, модель на стене, мир с тегами (#4346, вариант D).

Зачем: проверить в Gazebo, как rtabmap локализуется после перезапуска, с тегом в карте и без,
и как тег влияет на карту (docs/research/2026-11-09-sim-localization-restart.md).

Текстура генерируется здесь из открытого формата tag36h11, чужих картинок в репо нет
(решение Шифу 05.11 про сторонние текстуры). Коды и раскладка битов — из
AprilRobotics/apriltag ``tag36h11.c`` (BSD-2, (C) 2013-2016 The Regents of The University
of Michigan): ``codedata[id]``, ``bit_x``/``bit_y``, ``width_at_border=8``, ``total_width=10``.
Рисование — как ``apriltag_to_image()`` (apriltag.c): кольцо белого, кольцо чёрного,
36 бит внутри (1 — белая клетка).

Размер — как у тегов Шифу (#4346, 04.11): чёрный квадрат 0.119 м (это ``size`` в
docker/vision/config/apriltag/apriltag_config.yaml), с белой рамкой 0.179 м. Клетка
0.119 / 8 = 14.875 мм, рамка — 12 клеток = 0.1785 м.

Перегенерировать мир и текстуры::

    python3 -m rob_box_sim.apriltag_world src/rob_box_sim/worlds

Тест tests/unit/sim/test_sim_apriltag_world.py сверяет файлы с этим модулем.
"""

from __future__ import annotations

import math
import struct
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path

# ── формат tag36h11 (AprilRobotics/apriltag tag36h11.c) ───────────────────────
TAG36H11_BIT_X = (
    1,
    2,
    3,
    4,
    5,
    2,
    3,
    4,
    3,
    6,
    6,
    6,
    6,
    6,
    5,
    5,
    5,
    4,
    6,
    5,
    4,
    3,
    2,
    5,
    4,
    3,
    4,
    1,
    1,
    1,
    1,
    1,
    2,
    2,
    2,
    3,
)
TAG36H11_BIT_Y = (
    1,
    1,
    1,
    1,
    1,
    2,
    2,
    2,
    3,
    1,
    2,
    3,
    4,
    5,
    2,
    3,
    4,
    3,
    6,
    6,
    6,
    6,
    6,
    5,
    5,
    5,
    4,
    6,
    5,
    4,
    3,
    2,
    5,
    4,
    3,
    4,
)
TAG36H11_NBITS = 36
TAG36H11_WIDTH_AT_BORDER = 8
TAG36H11_TOTAL_WIDTH = 10
# codedata[id] для id, которые стоят в мирах сима (тег 21 — тот, что видит робот в #4346).
TAG36H11_CODES = {
    0: 0x0000000D7E00984B,
    21: 0x000000086CC4A5C5,
    22: 0x00000008B64DF90F,
    23: 0x00000009C577B611,
}

# ── размеры тега Шифу ─────────────────────────────────────────────────────────
TAG_BLACK_M = 0.119
CELL_M = TAG_BLACK_M / TAG36H11_WIDTH_AT_BORDER
MARGIN_CELLS = 1  # белая рамка шире формата на клетку с каждой стороны: 12 клеток ≈ 0.179 м
TAG_TOTAL_CELLS = TAG36H11_TOTAL_WIDTH + 2 * MARGIN_CELLS
TAG_PLATE_M = TAG_TOTAL_CELLS * CELL_M
TAG_THICK_M = 0.002
PX_PER_CELL = 20
TEXTURE_DIR = "textures"
SIM_WORLDS_MOUNT = "/sim_worlds"  # docker/sim/compose.yaml: src/rob_box_sim/worlds → /sim_worlds


def tag_cells(tag_id: int) -> list[list[int]]:
    """Клетки tag36h11 ``tag_id`` 10×10, строки сверху вниз: 1 — белое, 0 — чёрное.

    То же, что ``apriltag_to_image()``: внешнее кольцо белое, следующее чёрное, биты кода
    от старшего к младшему в клетки ``(bit_x + 1, bit_y + 1)``.
    """
    if tag_id not in TAG36H11_CODES:
        raise KeyError(f"кода tag36h11 id={tag_id} нет в TAG36H11_CODES: {sorted(TAG36H11_CODES)}")
    code = TAG36H11_CODES[tag_id]
    n = TAG36H11_TOTAL_WIDTH
    cells = [[0] * n for _ in range(n)]
    for i in range(n):
        for row, col in ((0, i), (n - 1, i), (i, 0), (i, n - 1)):
            cells[row][col] = 1
    start = (n - TAG36H11_WIDTH_AT_BORDER) // 2
    for i in range(TAG36H11_NBITS):
        if code & (1 << (TAG36H11_NBITS - i - 1)):
            cells[TAG36H11_BIT_Y[i] + start][TAG36H11_BIT_X[i] + start] = 1
    return cells


def with_margin(cells: list[list[int]], margin: int = MARGIN_CELLS) -> list[list[int]]:
    """Добавить белую рамку ``margin`` клеток."""
    width = len(cells[0]) + 2 * margin
    pad = [[1] * width for _ in range(margin)]
    return pad + [[1] * margin + row + [1] * margin for row in cells] + [row[:] for row in pad]


def mirror_columns(cells: list[list[int]]) -> list[list[int]]:
    """Зеркало слева направо (если текстура на грани ложится зеркально)."""
    return [row[::-1] for row in cells]


def png_gray(cells: list[list[int]], px_per_cell: int = PX_PER_CELL) -> bytes:
    """PNG 8-бит серый без зависимостей: клетка → квадрат ``px_per_cell``; 1 → 255, 0 → 0."""
    rows = []
    for row in cells:
        line = b"\x00" + b"".join((b"\xff" if v else b"\x00") * px_per_cell for v in row)
        rows.extend([line] * px_per_cell)
    height, width = len(rows), len(cells[0]) * px_per_cell

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(b"".join(rows), 9))
        + chunk(b"IEND", b"")
    )


@dataclass(frozen=True)
class TagPlacement:
    """Тег на стене: центр (x, y, z) в мире и ``yaw`` — куда смотрит лицо тега (нормаль)."""

    tag_id: int
    x: float
    y: float
    z: float
    yaw: float

    @property
    def name(self) -> str:
        return f"tag36h11_{self.tag_id}"

    @property
    def texture(self) -> str:
        return f"{TEXTURE_DIR}/{self.name}.png"


def tag_model_sdf(tag: TagPlacement, texture_uri: str) -> str:
    """Статическая модель-пластина без collision (лидар её почти не видит: 2 мм от стены).

    Текстура и в albedo, и в emissive: тег светится сам, тень на грани не гасит контраст.
    """
    c, h = TAG_THICK_M, TAG_PLATE_M
    pose = f"{tag.x:.4f} {tag.y:.4f} {tag.z:.4f} 0 0 {tag.yaw:.6f}"
    return f"""    <model name="{tag.name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{c:.3f} {h:.4f} {h:.4f}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
            <pbr><metal>
              <albedo_map>{texture_uri}</albedo_map>
              <emissive_map>{texture_uri}</emissive_map>
              <metalness>0</metalness><roughness>1</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""


def add_tags(world_sdf: str, old_name: str, new_name: str, tags: list[TagPlacement], uri_root: str) -> str:
    """Мир ``old_name`` → ``new_name`` с моделями тегов перед ``</world>``."""
    head = f'<world name="{old_name}">'
    if world_sdf.count(head) != 1 or world_sdf.count("</world>") != 1:
        raise ValueError(f"в SDF нет ровно одного мира {old_name!r}")
    ids = [t.tag_id for t in tags]
    if len(set(ids)) != len(ids):
        raise ValueError(f"id тегов повторяются: {ids}")
    models = "".join(tag_model_sdf(t, f"{uri_root}/{t.texture}") for t in tags)
    note = f"<!-- Теги tag36h11 (rob_box_sim.apriltag_world, #4346): {', '.join(map(str, ids))} -->\n"
    return world_sdf.replace(head, f'<world name="{new_name}">', 1).replace(
        "  </world>", f"    {note}{models}  </world>", 1
    )


# ── теги в мирах ──────────────────────────────────────────────────────────────
# polygon (worlds_catalog.polygon): комната x -1.5..10.5, y -3..3, перегородки x=2,4,6,8.
# Высота центра тега 0.55 м — примерно высота камеры OAK-D (base_link z 0.4595 + колесо).
#   21 — западная грань перегородки x=2 (y=+1.0), лицом на запад: видна со спавна по ходу +X;
#   22 — восточная грань западной стены x=-1.5 (y=-1.0), лицом на восток: видна на пути домой;
#   23 — западная грань восточной стены x=10.5 (y=+1.0): видна у дальней цели маршрута.
_FACE = 0.05 + TAG_THICK_M / 2  # полстены + полпластины
POLYGON_TAGS = (
    TagPlacement(21, 2.0 - _FACE, 1.0, 0.55, math.pi),
    TagPlacement(22, -1.5 + _FACE, -1.0, 0.55, 0.0),
    TagPlacement(23, 10.5 - _FACE, 1.0, 0.55, math.pi),
)
TAG_WORLDS = {"polygon_tags": ("polygon", POLYGON_TAGS)}


def write_tag_world(worlds_dir: Path, name: str) -> list[Path]:
    """Записать ``<name>.sdf`` и текстуры его тегов в ``worlds_dir``."""
    base, tags = TAG_WORLDS[name]
    text = (worlds_dir / f"{base}.sdf").read_text(encoding="utf-8").replace("\r\n", "\n")
    out = [worlds_dir / f"{name}.sdf"]
    out[0].write_text(add_tags(text, base, name, list(tags), SIM_WORLDS_MOUNT), encoding="utf-8", newline="\n")
    (worlds_dir / TEXTURE_DIR).mkdir(exist_ok=True)
    for t in tags:
        path = worlds_dir / t.texture
        path.write_bytes(png_gray(with_margin(tag_cells(t.tag_id))))
        out.append(path)
    return out


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: python3 -m rob_box_sim.apriltag_world <каталог worlds>", file=sys.stderr)
        return 2
    for name in TAG_WORLDS:
        for path in write_tag_world(Path(args[0]), name):
            print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
