"""Обход покрытием (#4560): план полос (coverage_plan) и метрика (coverage_metric) без ROS."""

import math

import numpy as np
import pytest
from rob_box_sim.coverage_metric import (
    GoalOutcome,
    coverage_result,
    covered_mask,
    densify,
    parse_track_log,
    plan_track,
    run_summary,
)
from rob_box_sim.coverage_plan import (
    coverage_space,
    decompose,
    default_erosion,
    erode,
    plan_coverage,
    plan_summary,
    row_runs,
)
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Box, Footprint, signed_distance
from rob_box_sim.map_check import OccupancyGrid

RES = 0.05


def grid_from_boxes(x0, y0, x1, y1, boxes, res=RES, unknown_border=False):
    """Зал [x0..x1]×[y0..y1] со стенами по краю (одна клетка) и коробками-препятствиями (центр, размеры)."""
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    a = np.zeros((h, w), dtype=np.int16)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = -1 if unknown_border else 100
    cx = x0 + (np.arange(w) + 0.5) * res
    cy = y0 + (np.arange(h) + 0.5) * res
    for b in boxes:
        cols = (cx >= b.cx - b.sx / 2) & (cx <= b.cx + b.sx / 2)
        rows = (cy >= b.cy - b.sy / 2) & (cy <= b.cy + b.sy / 2)
        a[np.ix_(rows, cols)] = 100
    return OccupancyGrid(w, h, res, x0, y0, a.reshape(-1).tolist())


def box(name, cx, cy, sx, sy):
    return Box(name, cx, cy, sx, sy, 2.0)


def walls(x0, y0, x1, y1, t=RES):
    return [
        box("s", (x0 + x1) / 2, y0 + t / 2, x1 - x0, t),
        box("n", (x0 + x1) / 2, y1 - t / 2, x1 - x0, t),
        box("w", x0 + t / 2, (y0 + y1) / 2, t, y1 - y0),
        box("e", x1 - t / 2, (y0 + y1) / 2, t, y1 - y0),
    ]


def depot_like():
    """Похоже на Depot (docs/runbooks/hil-video.md §7): зал 30×15 м, запад свободен, на востоке ряды
    стеллажей вдоль x с проходами 1.5–2.3 м, колонны 0.4×0.4 через 6 м, несколько ящиков."""
    x0, y0, x1, y1 = -10.5, -7.5, 19.5, 7.5
    obs = []
    y = -7.5 + 1.5
    for k, aisle in enumerate((1.5, 1.9, 2.3, 1.7, 1.1)):  # последний — зазор до северной стены
        obs.append(box(f"rack{k}", 11.0, y + 0.5, 14.0, 1.0))
        y += 1.0 + aisle
    for k, (cx, cy) in enumerate([(-4.5, 1.5), (-4.5, -1.5), (1.5, 1.5), (1.5, -1.5), (-4.5, 4.5), (1.5, -4.5)]):
        obs.append(box(f"col{k}", cx, cy, 0.4, 0.4))
    obs += [box("crate0", -7.0, -4.0, 1.2, 1.2), box("crate1", -2.0, 5.5, 2.0, 1.0)]
    return grid_from_boxes(x0, y0, x1, y1, obs), obs + walls(x0, y0, x1, y1)


def _cell(space, x, y):
    return math.floor((y - space.origin_y) / space.resolution), math.floor((x - space.origin_x) / space.resolution)


def _lane_clearance(plan, world):
    """Наименьший зазор от прямоугольника, который корпус заметает на каждой полосе (вдоль оси), до мира."""
    half = max(ROBOT_FOOTPRINT.x_max, -ROBOT_FOOTPRINT.x_min)
    side = ROBOT_FOOTPRINT.y_max
    best = math.inf
    for a, b in zip(plan.poses, plan.poses[1:]):
        if (a.cell, a.lane) != (b.cell, b.lane):
            continue
        along_x = abs(math.cos(a.yaw)) > 0.5
        hx, hy = (half, side) if along_x else (side, half)
        x0, x1, y0, y1 = min(a.x, b.x) - hx, max(a.x, b.x) + hx, min(a.y, b.y) - hy, max(a.y, b.y) + hy
        swept = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        best = min([best] + [signed_distance(swept, w.corners()) for w in world])
    return best


def _plan_coverage_pct(space, plan):
    return coverage_result(space, covered_mask(space, plan_track(plan.poses))).pct


# ── маски ────────────────────────────────────────────────────────────────────


def test_row_runs():
    assert row_runs(np.array([0, 1, 1, 0, 1, 0, 1, 1, 1], bool)) == [(1, 2), (4, 4), (6, 8)]
    assert row_runs(np.zeros(4, bool)) == []


def test_erode_keeps_centers_beyond_radius():
    free = np.ones((40, 40), bool)
    free[:, 0] = False  # стена слева
    centers = erode(free, 0.336, RES)
    # стена — клетка 0; до центра клетки c расстояние от края стены (c − 0.5) × RES
    first = int(np.argmax(centers[20]))
    assert (first - 0.5) * RES >= 0.336
    assert (first - 1.5) * RES < 0.336 + RES


def test_default_erosion_is_half_footprint_plus_padding():
    assert default_erosion() == pytest.approx(0.3064 + 0.03)


def test_decompose_splits_around_obstacle():
    space = np.ones((30, 30), bool)
    space[10:20, 10:20] = False
    dec = decompose(space)
    assert len(dec.rows) == 4  # под, слева, справа, над
    assert all(len(adj) >= 2 for adj in dec.adjacency)


# ── план ─────────────────────────────────────────────────────────────────────


def test_empty_room_plan_covers_95_percent_of_coverable():
    grid = grid_from_boxes(0, 0, 10, 8, [])
    space = coverage_space(grid, (1.0, 1.0))
    plan = plan_coverage(space)
    assert not plan.unreachable
    assert plan.lanes_dropped_short == 0
    assert _plan_coverage_pct(space, plan) >= 95.0
    for p in plan.poses:
        assert space.centers[_cell(space, p.x, p.y)]
    # шаг полос не больше заданного: соседние полосы ячейки не дальше 0.55 м
    ys = sorted({round(p.y, 3) for p in plan.poses})
    assert max(b - a for a, b in zip(ys, ys[1:])) <= 0.55 + 1e-9


def test_racks_room_lanes_stay_clear_of_obstacles():
    racks = [box("r0", 5.0, 2.5, 6.0, 1.0), box("r1", 5.0, 5.0, 6.0, 1.0), box("c", 1.5, 6.5, 0.6, 0.6)]
    grid = grid_from_boxes(0, 0, 10, 8, racks)
    world = racks + walls(0, 0, 10, 8)
    space = coverage_space(grid, (1.0, 1.0))
    plan = plan_coverage(space)
    assert len(plan.cells) > 1
    assert not plan.unreachable
    for p in plan.poses:
        assert space.centers[_cell(space, p.x, p.y)], p
    # корпус на полосах не задевает ни стеллаж, ни стену (с запасом padding минус клетка)
    assert _lane_clearance(plan, world) >= 0.03 - RES
    assert _plan_coverage_pct(space, plan) >= 90.0


def test_narrow_passage_marks_room_behind_unreachable():
    # две комнаты 5×5, между ними стена с проёмом 0.5 м (уже габарита 0.605)
    wall = [box("wall_s", 5.0, 1.0, 0.2, 2.0), box("wall_n", 5.0, 3.75, 0.2, 2.5 + 2.0)]
    grid = grid_from_boxes(0, 0, 10, 5, wall)
    space = coverage_space(grid, (1.0, 1.0))
    plan = plan_coverage(space)
    assert plan.unreachable, "комната за узким проходом должна быть в отчёте"
    assert sum(c.centers_m2 for c in plan.unreachable) > 5.0
    assert all(p.x < 5.0 for p in plan.poses)
    assert "unreachable_cells=" in plan_summary(plan, space)
    assert space.area(space.free) - space.area(space.coverable) > 15.0


def test_doorway_0_90_stays_reachable():
    # минимальный проём (CONTEXT.md): 0.90 м робот проходит — после сужения проход должен остаться
    wall = [box("wall_s", 5.0, 1.0, 0.2, 2.0), box("wall_n", 5.0, 3.95, 0.2, 2.1)]
    grid = grid_from_boxes(0, 0, 10, 5, wall)
    space = coverage_space(grid, (1.0, 1.0))
    plan = plan_coverage(space)
    assert not plan.unreachable
    assert any(p.x > 5.5 for p in plan.poses)


def test_unknown_is_not_free_and_start_snaps_inside():
    grid = grid_from_boxes(0, 0, 6, 4, [], unknown_border=True)
    space = coverage_space(grid, (0.05, 0.05))  # старт у самой стены
    assert space.start_snap_m is not None and space.start_snap_m > 0.3
    assert plan_coverage(space).poses


def test_auto_axis_picks_faster_plan():
    grid = grid_from_boxes(0, 0, 10, 2, [])
    space = coverage_space(grid, (1.0, 1.0))
    auto = plan_coverage(space)
    across = plan_coverage(space, axis="y")
    assert auto.axis == "x"
    assert auto.lanes < across.lanes
    assert auto.time_estimate_s() < across.time_estimate_s()


def test_short_lanes_dropped_and_counted():
    grid = grid_from_boxes(0, 0, 0.95, 4, [])  # центры по x — около 0.2 м: полоса вдоль x короче 0.3
    space = coverage_space(grid, (0.45, 1.0))
    plan = plan_coverage(space, axis="x")
    assert plan.lanes == 0 and plan.lanes_dropped_short > 0


def test_waypoint_step_adds_mid_poses():
    grid = grid_from_boxes(0, 0, 10, 2, [])
    space = coverage_space(grid, (1.0, 1.0))
    plain = plan_coverage(space, axis="x")
    dense = plan_coverage(space, axis="x", waypoint_step_m=2.0)
    assert {p.kind for p in plain.poses} == {"start", "end"}
    assert "mid" in {p.kind for p in dense.poses}
    assert len(dense.poses) > len(plain.poses)


def test_depot_like_plan():
    grid, world = depot_like()
    space = coverage_space(grid, (0.0, 0.0))
    plan = plan_coverage(space)
    assert not plan.unreachable
    assert _plan_coverage_pct(space, plan) >= 90.0
    assert _lane_clearance(plan, world) >= 0.03 - RES


# ── метрика ──────────────────────────────────────────────────────────────────


def test_metric_straight_track_known_answer():
    res = 0.1
    grid = OccupancyGrid(60, 40, res, 0.0, 0.0, [0] * 2400)
    fp = Footprint(-0.3, 0.3, -0.3, 0.3)
    space = coverage_space(grid, (2.0, 2.0), footprint=fp, erosion_m=0.3)
    track = [(t, 1.0 + 0.5 * t, 2.0, 0.0) for t in range(5)]  # 1.0 → 3.0 м по 0.5 м
    covered = covered_mask(space, track, footprint=fp)
    # центры клеток 0.05 + 0.1k: по x в [0.7, 3.3] — 26 столбцов, по y в [1.7, 2.3] — 6 строк
    assert int(covered.sum()) == 26 * 6
    res_ = coverage_result(space, covered)
    assert res_.free_covered_m2 == pytest.approx(156 * res * res)


def test_rotated_footprint_stays_inside_circumscribed_circle():
    grid = OccupancyGrid(40, 40, 0.05, 0.0, 0.0, [0] * 1600)
    space = coverage_space(grid, (1.0, 1.0))
    covered = covered_mask(space, [(0.0, 1.0, 1.0, math.pi / 4)])
    rr, cc = np.nonzero(covered)
    d = np.hypot((cc + 0.5) * 0.05 - 1.0, (rr + 0.5) * 0.05 - 1.0)
    assert d.max() <= math.hypot(0.3064, 0.3025) + 1e-9
    assert covered.sum() * 0.0025 == pytest.approx(0.61 * 0.605, rel=0.1)


def test_densify_does_not_bridge_log_gaps():
    track = [(0.0, 0.0, 0.0, 0.0), (1.0, 0.5, 0.0, 0.0), (2.0, 3.0, 0.0, 0.0), (30.0, 3.2, 0.0, 0.0)]
    out = densify(track, step_m=0.05, max_gap_m=1.0, max_dt_s=5.0)
    xs = [p[1] for p in out]
    assert len([x for x in xs if 0.0 < x < 0.5]) == 9  # 0.5 м через 0.05 м
    assert not [x for x in xs if 0.5 < x < 3.0]  # прыжок 2.5 м не связан
    assert not [x for x in xs if 3.0 < x < 3.2]  # пауза 28 с не связана


def test_parse_track_log_route_format():
    lines = [
        "12:00:00.000 GOAL_START c0l0s target=(1,2,0) sim=10.00 frame=-1 gt=(0, 0, 0)",
        "12:00:00.200 TRACK sim=10.20 x=1.000 y=-2.500 yaw=90.0",
        "12:00:00.300 EM_ON sim=10.30",
        "12:00:00.400 TRACK sim=10.40 x=1.100 y=-2.500 yaw=-180.0",
    ]
    assert parse_track_log(lines) == [
        (10.2, 1.0, -2.5, pytest.approx(math.pi / 2)),
        (10.4, 1.1, -2.5, pytest.approx(-math.pi)),
    ]


def test_run_summary_is_built_from_outcomes():
    grid = grid_from_boxes(0, 0, 4, 4, [])
    space = coverage_space(grid, (1.0, 1.0))
    cov = coverage_result(space, space.coverable.copy())
    outcomes = [
        GoalOutcome("a", "SUCCEEDED", 1),
        GoalOutcome("b", "ABORTED"),
        GoalOutcome("c", "TIMEOUT", 2),
        GoalOutcome("d", "SUCCEEDED"),
        GoalOutcome("e", "NOT_SENT"),
    ]
    line = run_summary(outcomes, cov, 123.45, "5_failures_in_row")
    assert line.startswith("COVERAGE_DONE poses=5 succeeded=2 skipped=2 (ABORTED=1, TIMEOUT=1) not_sent=1 ")
    assert "stop=5_failures_in_row" in line
    assert "em_episodes=3" in line and "sim_s=123.5" in line and "coverage=100.0%" in line
