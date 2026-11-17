"""Видео картографии (rob_box_sim.map_video): метрики карты и поз, выбор кадров.

Рисование (PIL) и склейка (ffmpeg) не проверяются — их нет в CI; проверяются
числа на кадре: совпадение карты со стенами мира, ошибка позы, моменты кадров.
"""

import math

import numpy as np
import pytest
from rob_box_sim.geometry2d import Box
from rob_box_sim.map_video import (
    GridSnap,
    dilate,
    frame_times,
    interp_pose,
    map_agreement,
    pose_error,
    snap_at,
    view_for,
    world_mask,
)

RES = 0.05
# Стена вдоль X: x 0..2, y 0.95..1.05 (ось y=1.0, толщина 0.1).
WALL = Box("w", 1.0, 1.0, 2.0, 0.1, 1.0)
LOW = Box("low", 1.0, 0.5, 0.4, 0.4, 0.3)


def _empty(w=60, h=40, ox=-0.5, oy=0.0, t=0.0, fill=0):
    return GridSnap(t, RES, ox, oy, np.full((h, w), fill, dtype=np.int16))


def _row(snap, y):
    return int(math.floor((y - snap.origin_y) / RES))


def _col(snap, x):
    return int(math.floor((x - snap.origin_x) / RES))


def _draw_wall(snap, y, x0=0.0, x1=2.0):
    data = snap.data.copy()
    data[_row(snap, y), _col(snap, x0):_col(snap, x1)] = 100
    return GridSnap(snap.t, snap.resolution, snap.origin_x, snap.origin_y, data)


def test_world_mask_marks_wall_cells_and_skips_low_boxes():
    snap = _empty()
    mask = world_mask([WALL, LOW], snap)
    rows = sorted({r for r, _ in zip(*np.nonzero(mask))})
    # 0.1 м стены + по полклетки допуска: строки y 0.90..1.10
    assert rows == [_row(snap, 0.93), _row(snap, 0.97), _row(snap, 1.02), _row(snap, 1.07)]
    assert not mask[_row(snap, 0.5), _col(snap, 1.0)]  # коробка 0.3 м ниже лидара


def test_dilate_grows_by_cells_and_keeps_shape():
    m = np.zeros((7, 7), dtype=bool)
    m[3, 3] = True
    d = dilate(m, 2)
    assert d.shape == m.shape and d.sum() == 25 and d[1, 1] and d[5, 5] and not d[0, 3]
    assert dilate(m, 0).sum() == 1


def test_perfect_map_scores_one():
    snap = _draw_wall(_empty(), 1.0)
    agr = map_agreement(snap, [WALL])
    assert agr["precision"] == 1.0 and agr["recall"] == 1.0 and agr["iou"] == 1.0


def test_shifted_wall_beyond_tolerance_is_false_positive():
    snap = _draw_wall(_empty(), 1.4)  # стена нарисована на 0.4 м выше настоящей
    agr = map_agreement(snap, [WALL], tol=0.15)
    assert agr["precision"] == 0.0 and agr["recall"] == 0.0 and agr["iou"] == 0.0


def test_half_seen_wall_halves_recall_not_precision():
    snap = _draw_wall(_empty(), 1.0, 0.0, 1.0)
    agr = map_agreement(snap, [WALL], tol=0.1)
    assert agr["precision"] == 1.0
    assert agr["recall"] == pytest.approx(0.55, abs=0.06)
    assert 0.4 < agr["iou"] < 0.7


def test_empty_map_has_no_score():
    agr = map_agreement(_empty(fill=-1), [WALL])
    assert agr["occupied"] == 0 and agr["precision"] is None and agr["recall"] == 0.0 and agr["iou"] == 0.0


def test_wall_outside_grid_is_not_counted():
    snap = _draw_wall(_empty(), 1.0)
    far = Box("far", 20.0, 20.0, 1.0, 0.1, 1.0)
    assert map_agreement(snap, [WALL, far])["iou"] == 1.0


def test_interp_pose_linear_and_wraps_yaw():
    s = [(0.0, 0.0, 0.0, 3.0), (1.0, 1.0, 2.0, -3.0)]
    x, y, yaw = interp_pose(s, 0.5)
    assert (x, y) == (0.5, 1.0)
    # 3.0 → -3.0 — короткой дугой через π, а не через 0
    assert abs(math.cos(yaw) - math.cos(math.pi)) < 1e-3


def test_interp_pose_gaps_and_edges():
    s = [(0.0, 0.0, 0.0, 0.0), (5.0, 1.0, 0.0, 0.0)]
    assert interp_pose(s, 2.0, max_gap=1.0) is None  # дыра в оценке — честно «нет оценки»
    assert interp_pose(s, 5.5, max_gap=1.0) == (1.0, 0.0, 0.0)
    assert interp_pose(s, 7.0, max_gap=1.0) is None
    assert interp_pose([], 1.0) is None


def test_pose_error():
    assert pose_error((0, 0, 0), (3, 4, 1)) == 5.0
    assert pose_error(None, (0, 0, 0)) is None


def test_frame_times_cover_run_and_end_on_last_sample():
    assert frame_times(10.0, 11.2, 0.5) == [10.0, 10.5, 11.0, 11.2]
    assert frame_times(0.0, 1.0, 0.5) == [0.0, 0.5, 1.0]
    assert frame_times(1.0, 0.0, 0.5) == []


def test_view_crop_keeps_robot_path_and_even_frame():
    long_wall = Box("long", 6.0, 1.0, 36.0, 0.1, 1.0)
    truth = [(0.0, 0.0, 0.0, 0.0), (1.0, 10.0, 0.0, 0.0)]
    full, cropped = view_for([long_wall], truth), view_for([long_wall], truth, crop=3.0)
    assert full.x0 < -11 and full.x1 > 23
    assert (cropped.x0, cropped.x1) == (-3.0, 13.0)
    assert cropped.size[0] % 2 == 0 and cropped.size[1] % 2 == 0
    px, py = cropped.px(0.0, 0.0)
    assert 0 <= px <= cropped.size[0] and cropped.header_px <= py <= cropped.size[1]


def test_snap_at_takes_last_map_not_after_t():
    snaps = [_empty(t=1.0), _empty(t=2.0), _empty(t=3.0)]
    assert snap_at(snaps, 0.5) is None
    assert snap_at(snaps, 2.0).t == 2.0
    assert snap_at(snaps, 2.9).t == 2.0
    assert snap_at(snaps, 10.0).t == 3.0
