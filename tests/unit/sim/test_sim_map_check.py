"""rob_box_sim.map_check: сверка карты rtabmap с коробками мира, без ROS."""

from __future__ import annotations

import pytest

from rob_box_sim.geometry2d import Box
from rob_box_sim.map_check import OccupancyGrid, ascii_map, compare, write_pgm


def _grid():
    # 1 м × 1 м по 0.1 м: занят столбец у x≈0.95 (стена мира) и одна ложная клетка в центре
    data = [0] * 100
    for r in range(10):
        data[r * 10 + 9] = 100
    data[5 * 10 + 5] = 100
    data[0] = -1
    return OccupancyGrid(10, 10, 0.1, 0.0, 0.0, data)


def test_precision_counts_cells_near_world_boxes():
    wall = Box("w", 1.05, 0.5, 0.1, 1.0, 1.0)
    stats = compare(_grid(), [wall], tol=0.15)
    assert stats["cells_occupied"] == 11
    assert stats["occupied_near_world_box"] == 10
    assert stats["precision"] == pytest.approx(10 / 11, abs=1e-4)
    assert stats["cells_unknown"] == 1
    assert stats["cells_free"] == 88


def test_shifted_map_loses_precision():
    wall = Box("w", 1.05 + 0.5, 0.5, 0.1, 1.0, 1.0)  # мир на 0.5 м дальше, чем на карте
    assert compare(_grid(), [wall], tol=0.15)["precision"] == 0.0


def test_ascii_top_is_plus_y():
    art = ascii_map(_grid(), step=1)
    assert art[0].endswith("#") and art[4][5] == "#" and art[-1][0] == " "


def test_write_pgm_flips_rows(tmp_path):
    write_pgm(_grid(), tmp_path / "m.pgm")
    data = (tmp_path / "m.pgm").read_bytes()
    assert data.startswith(b"P5\n10 10\n255\n")
    px = data[len(b"P5\n10 10\n255\n"):]
    assert px[9 * 10 + 0] == 205  # нижняя строка картинки = строка 0 карты, клетка 0 неизвестна
    assert px[9] == 0 and px[0] == 254


def test_value_at_and_raw_pgm_keep_cost_levels(tmp_path):
    from rob_box_sim.map_check import write_raw_pgm

    g = OccupancyGrid(3, 2, 0.5, -1.0, 0.0, [0, 50, 99, 100, -1, 7])
    assert g.value_at(-0.9, 0.1) == 0
    assert g.value_at(0.1, 0.1) == 99
    assert g.value_at(-0.4, 0.6) == -1
    assert g.value_at(5.0, 0.0) is None and g.value_at(-1.1, 0.0) is None
    write_raw_pgm(g, tmp_path / "c.pgm")
    raw = (tmp_path / "c.pgm").read_bytes()
    assert raw.startswith(b"P5\n3 2\n255\n")
    assert list(raw[-6:]) == [100, 255, 7, 0, 50, 99]  # верх — +Y: сначала строка r=1
