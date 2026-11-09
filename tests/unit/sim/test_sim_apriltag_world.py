"""Теги tag36h11 в мирах сима (#4346, вариант D): формат, текстура, мир.

Проверяется то, что увидит детектор и Gazebo: клетки тега по правилам формата, PNG,
который читается обратно в те же клетки, позы тегов на гранях стен мира, и что файлы
в worlds/ совпадают с генератором. Декодирование настоящим детектором — на стенде
(apriltag_node в docker/sim), здесь его нет.
"""

from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import pytest
from rob_box_sim.world_sdf import boxes_from_sdf

from rob_box_sim import apriltag_world as aw

REPO = Path(__file__).resolve().parents[3]
WORLDS = REPO / "src" / "rob_box_sim" / "worlds"


def _read_png_gray(data: bytes) -> list[list[int]]:
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat, w = 8, b"", 0
    while pos < len(data):
        (n,), kind = struct.unpack(">I", data[pos : pos + 4]), data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + n]
        assert zlib.crc32(kind + body) & 0xFFFFFFFF == struct.unpack(">I", data[pos + 8 + n : pos + 12 + n])[0]
        if kind == b"IHDR":
            w, h, depth, color = struct.unpack(">IIBB", body[:10])
            assert (depth, color) == (8, 0)
        elif kind == b"IDAT":
            idat += body
        pos += 12 + n
    raw = zlib.decompress(idat)
    rows = [raw[i * (w + 1) : (i + 1) * (w + 1)] for i in range(len(raw) // (w + 1))]
    assert all(r[0] == 0 for r in rows)
    return [list(r[1:]) for r in rows]


@pytest.mark.parametrize("tag_id", sorted(aw.TAG36H11_CODES))
def test_tag_has_white_ring_black_border_and_36_code_bits(tag_id):
    cells = aw.tag_cells(tag_id)
    n = aw.TAG36H11_TOTAL_WIDTH
    assert len(cells) == n and all(len(r) == n for r in cells)
    ring = [cells[0][i] for i in range(n)] + [cells[n - 1][i] for i in range(n)]
    ring += [cells[i][0] for i in range(n)] + [cells[i][n - 1] for i in range(n)]
    assert set(ring) == {1}
    border = [cells[1][i] for i in range(1, n - 1)] + [cells[n - 2][i] for i in range(1, n - 1)]
    border += [cells[i][1] for i in range(1, n - 1)] + [cells[i][n - 2] for i in range(1, n - 1)]
    assert set(border) == {0}
    inner = [cells[r][c] for r in range(2, n - 2) for c in range(2, n - 2)]
    assert sum(inner) == bin(aw.TAG36H11_CODES[tag_id]).count("1")


def test_bit_layout_covers_inner_6x6_once():
    assert sorted(zip(aw.TAG36H11_BIT_X, aw.TAG36H11_BIT_Y)) == [(x, y) for x in range(1, 7) for y in range(1, 7)]


def test_first_bit_is_msb_at_top_left():
    """apriltag_to_image: бит i = (code >> (35 - i)) & 1 в клетке (bit_x[i]+1, bit_y[i]+1); бит 0 — (1, 1)."""
    cells = aw.tag_cells(21)
    assert cells[2][2] == (aw.TAG36H11_CODES[21] >> 35) & 1


def test_codes_are_far_apart():
    """Пара тегов на стенде не должна путаться: расстояние Хэмминга tag36h11 ≥ 11."""
    codes = list(aw.TAG36H11_CODES.values())
    for i, a in enumerate(codes):
        for b in codes[i + 1 :]:
            assert bin(a ^ b).count("1") >= 11


def test_unknown_id_fails_loudly():
    with pytest.raises(KeyError, match="id=5"):
        aw.tag_cells(5)


def test_png_reads_back_as_the_same_cells_with_margin():
    cells = aw.with_margin(aw.tag_cells(21))
    assert len(cells) == aw.TAG_TOTAL_CELLS == 12
    px = _read_png_gray(aw.png_gray(cells, px_per_cell=3))
    assert len(px) == 36 and len(px[0]) == 36
    back = [[1 if px[r * 3 + 1][c * 3 + 1] == 255 else 0 for c in range(12)] for r in range(12)]
    assert back == cells


def test_tag_size_is_shifu_tag():
    """Чёрный квадрат 0.119 м (= size apriltag_config.yaml робота), с рамкой ≈ 0.179 м."""
    assert aw.CELL_M * aw.TAG36H11_WIDTH_AT_BORDER == pytest.approx(0.119)
    assert aw.TAG_PLATE_M == pytest.approx(0.179, abs=0.001)
    cfg = (REPO / "docker" / "vision" / "config" / "apriltag" / "apriltag_config.yaml").read_text(encoding="utf-8")
    assert "size: 0.119" in cfg


def test_mirror_flips_columns():
    assert aw.mirror_columns([[1, 0, 0], [0, 1, 1]]) == [[0, 0, 1], [1, 1, 0]]


def _tag_world() -> ET.Element:
    return ET.fromstring((WORLDS / "polygon_tags.sdf").read_text(encoding="utf-8").split("?>", 1)[1])


def test_committed_tag_world_and_textures_equal_generator(tmp_path):
    (tmp_path / "polygon.sdf").write_text((WORLDS / "polygon.sdf").read_text(encoding="utf-8"), encoding="utf-8")
    for path in aw.write_tag_world(tmp_path, "polygon_tags"):
        committed = (WORLDS / path.relative_to(tmp_path)).read_bytes()
        if path.suffix == ".sdf":  # текст: на Windows чекаут бывает с CRLF
            committed = committed.replace(b"\r\n", b"\n")
        assert committed == path.read_bytes(), f"перегенерируй: python3 -m rob_box_sim.apriltag_world {WORLDS}"


def test_tag_world_keeps_polygon_walls():
    """Стены те же, что у polygon: карта мира и сверка map_check не меняются."""
    walls = boxes_from_sdf((WORLDS / "polygon.sdf").read_text(encoding="utf-8"))
    tagged = boxes_from_sdf((WORLDS / "polygon_tags.sdf").read_text(encoding="utf-8"))
    assert tagged == walls  # у тегов нет collision


@pytest.mark.parametrize("tag", aw.POLYGON_TAGS, ids=lambda t: t.name)
def test_tag_sits_on_wall_face_and_looks_into_room(tag):
    walls = boxes_from_sdf((WORLDS / "polygon.sdf").read_text(encoding="utf-8"))
    nx, ny = math.cos(tag.yaw), math.sin(tag.yaw)
    behind = (tag.x - nx * (aw.TAG_THICK_M / 2 + 0.01), tag.y - ny * (aw.TAG_THICK_M / 2 + 0.01))
    front = (tag.x + nx * 0.05, tag.y + ny * 0.05)

    def inside(p, b):
        return abs(p[0] - b.cx) <= b.sx / 2 + 1e-6 and abs(p[1] - b.cy) <= b.sy / 2 + 1e-6

    assert any(inside(behind, b) for b in walls), "за тегом нет стены"
    assert not any(inside(front, b) for b in walls), "перед тегом стена"
    # сторона тега целиком на стене (не висит над проёмом)
    for s in (-aw.TAG_PLATE_M / 2, aw.TAG_PLATE_M / 2):
        edge = (behind[0] - ny * s, behind[1] + nx * s)
        assert any(inside(edge, b) for b in walls)
    assert 0.0 < tag.z - aw.TAG_PLATE_M / 2 and tag.z + aw.TAG_PLATE_M / 2 < 1.0


def test_tag_models_use_generated_textures_inside_mount():
    root = _tag_world()
    world = root.find("world")
    assert world.get("name") == "polygon_tags"
    models = {m.get("name"): m for m in world.findall("model") if m.get("name").startswith("tag36h11")}
    assert sorted(models) == sorted(t.name for t in aw.POLYGON_TAGS)
    for t in aw.POLYGON_TAGS:
        mat = models[t.name].find("link/visual/material/pbr/metal")
        uri = mat.findtext("albedo_map")
        assert uri == f"/sim_worlds/textures/{t.name}.png" == mat.findtext("emissive_map")
        assert (WORLDS / "textures" / f"{t.name}.png").is_file()
        assert models[t.name].find("link/collision") is None


def test_add_tags_refuses_duplicate_ids_and_wrong_world():
    text = (WORLDS / "polygon.sdf").read_text(encoding="utf-8")
    t = aw.POLYGON_TAGS[0]
    with pytest.raises(ValueError, match="повторяются"):
        aw.add_tags(text, "polygon", "x", [t, t], "/w")
    with pytest.raises(ValueError, match="turn90"):
        aw.add_tags(text, "turn90", "x", [t], "/w")
