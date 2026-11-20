"""Тесты генератора зон скорости (rob_box_sim.speed_zones).

Эталоны считаются в лоб (brute force) — EDT и лучи не переиспользуют проверяемый код.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from rob_box_sim.geometry2d import Box, Pose2D
from rob_box_sim.speed_zones import (
    FREE_THRESH,
    OCC_THRESH,
    ZoneParams,
    cell_to_pixel,
    corridor_width_m,
    distance_to_occupied_m,
    grid_from_boxes,
    grid_from_map,
    zones_mask,
)
from rob_box_sim.map_check import OccupancyGrid


def brute_distance_m(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """Евклидово расстояние до ближайшей занятой, в лоб по всем занятым."""
    occ_idx = np.argwhere(occupied)
    h, w = occupied.shape
    d = np.full((h, w), np.inf)
    for r in range(h):
        for c in range(w):
            if occupied[r, c]:
                d[r, c] = 0.0
                continue
            dists = np.hypot(occ_idx[:, 0] - r, occ_idx[:, 1] - c)
            d[r, c] = dists.min() * resolution
    return d


class TestDistanceToOccupied:
    def test_single_cell_matches_brute_force(self):
        occupied = np.zeros((21, 21), dtype=bool)
        occupied[10, 10] = True
        d = distance_to_occupied_m(occupied, 0.05)
        ref = brute_distance_m(occupied, 0.05)
        np.testing.assert_allclose(d, ref, atol=1e-9)

    def test_two_obstacles_corner(self):
        occupied = np.zeros((31, 31), dtype=bool)
        occupied[5, 5] = True
        occupied[5, 25] = True
        occupied[25, 5] = True
        d = distance_to_occupied_m(occupied, 0.1)
        ref = brute_distance_m(occupied, 0.1)
        np.testing.assert_allclose(d, ref, atol=1e-9)

    def test_no_occupied_raises(self):
        with pytest.raises(ValueError):
            distance_to_occupied_m(np.zeros((5, 5), dtype=bool), 0.05)


class TestCorridorWidth:
    def test_open_room_wide_corridor(self):
        # Комната 4×4 м: коридор в центре шире 2 м.
        occupied = np.zeros((100, 100), dtype=bool)
        occupied[0, :] = occupied[-1, :] = occupied[:, 0] = occupied[:, -1] = True
        width = corridor_width_m(occupied, 0.05)
        assert width[50, 50] > 4.8  # центр: почти вся ширина комнаты (~4.95 м)
        assert width[50, 50] <= 5.0

    def test_corridor_one_meter(self):
        # Коридор 1 м между двумя стенами по y=10..30 (внутренние грани), идёт вдоль x.
        occupied = np.zeros((60, 120), dtype=bool)
        occupied[:10, :] = True
        occupied[30:, :] = True
        width = corridor_width_m(occupied, 0.05)
        # центр коридора: ширина ≈ (30-10)*0.05 = 1.0 м
        assert 0.9 <= width[20, 60] <= 1.05

    def test_near_wall_of_big_room_is_not_narrow(self):
        # У стены большой комнаты ширина по вертикали остаётся шириной комнаты.
        occupied = np.zeros((120, 120), dtype=bool)
        occupied[0, :] = occupied[-1, :] = occupied[:, 0] = occupied[:, -1] = True
        width = corridor_width_m(occupied, 0.05)
        assert width[3, 60] > 5.0  # у стены, но комната ~6 м


class TestZonesMask:
    def _room_with_doorway(self):
        """Комната 4×4 м со стеной посередине и проёмом 0.9 м (res 0.05)."""
        res = 0.05
        wall = np.zeros((100, 100), dtype=bool)
        wall[:, :5] = True
        wall[:, -5:] = True
        wall[:5, :] = True
        wall[-5:, :] = True
        # перегородка x=48..52 кроме проёма y=45..63 (0.9 м)
        wall[45:63, 48:53] = False
        wall[:, 48:53] = True
        wall[45:63, 48:53] = False
        return wall, res

    def test_doorway_tight_room_free(self):
        wall, res = self._room_with_doorway()
        mask = zones_mask(wall, res)
        # проём — теснота 0.10
        assert mask[54, 50] == 10
        # у стены большой половины — 0.15
        assert mask[10, 20] == 15
        # центр большой половины — без ограничения
        assert mask[25, 25] == 0

    def test_params_validation(self):
        with pytest.raises(ValueError):
            ZoneParams(tight_speed=0.2, near_speed=0.15).validate()
        with pytest.raises(ValueError):
            ZoneParams(near_dist=0.5, tight_width=1.5).validate()
        with pytest.raises(ValueError):
            ZoneParams(near_speed=0.0).validate()


class TestGridFromBoxes:
    def test_box_becomes_occupied(self):
        occ, res, ox, oy = grid_from_boxes(
            [Box("b", 0.0, 0.0, 1.0, 1.0, 1.0)], resolution=0.1, margin_m=1.0
        )
        assert occ.any()
        cx, cy = int((0.0 - ox) / res), int((0.0 - oy) / res)
        assert occ[cy, cx]

    def test_map_pose_shifts_grid(self):
        # map в мире сдвинута: стена мира x=2..3 должна попасть в map при x≈-2..-3.
        occ, res, ox, oy = grid_from_boxes(
            [Box("w", 2.5, 0.0, 1.0, 10.0, 1.0)],
            resolution=0.1,
            margin_m=1.0,
            map_pose=Pose2D(5.0, 0.0, 0.0),
        )
        assert occ.any()
        rows, cols = np.argwhere(occ)[:, 0], np.argwhere(occ)[:, 1]
        xs = (cols + 0.5) * res + ox
        ys = (rows + 0.5) * res + oy
        assert xs.max() < -1.9
        assert xs.min() > -3.1
        assert ys.min() > -5.1
        assert ys.max() < 5.1


class TestGridFromMap:
    def test_unknown_is_occupied(self):
        grid = OccupancyGrid(2, 1, 0.05, 0.0, 0.0, [0, -1])
        occ, res, ox, oy = grid_from_map(grid)
        assert occ.tolist() == [[False, True]]

    def test_occupied_threshold(self):
        grid = OccupancyGrid(3, 1, 0.05, 0.0, 0.0, [0, 64, 100])
        occ, _, _, _ = grid_from_map(grid)
        assert occ.tolist() == [[False, False, True]]


class TestCellToPixel:
    def test_zero_is_free_white(self):
        assert cell_to_pixel(0) == 255

    def test_roundtrip_nav2_scale_formula(self):
        # Формула map_io (Scale): cell = rint((occ - free)/(occ_free) * 100), occ = 1 - pixel/255.
        for cell in (10, 15, 50, 100):
            pixel = cell_to_pixel(cell)
            occ = 1.0 - pixel / 255.0
            got = int(round((occ - FREE_THRESH) / (OCC_THRESH - FREE_THRESH) * 100.0))
            assert got == cell, f"cell {cell} -> pixel {pixel} -> {got}"
        # и в промежутках не должно быть неизвестного: pixel 255..191 -> free
        for pixel in (255, 220, 192):
            occ = 1.0 - pixel / 255.0
            assert occ < FREE_THRESH or pixel == 255
