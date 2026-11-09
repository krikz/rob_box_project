"""Места разворота в плане обхода (#4560 п.1): концы полос там, где робот крутится на месте без стопа защиты.

HIL Depot 09.11: концы полос в 0.34 м от стены, разворот требует ≥ 0.43 м (описанный радиус) плюс зона стопа
защиты 0.20 м — 58 стопов за 10 мин, 2 цели из 8. Здесь проверяется геометрия плана на синтетике.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from rob_box_sim.coverage_metric import coverage_result, covered_mask, plan_track
from rob_box_sim.coverage_plan import (
    GUARD_STOP_M,
    NAV2_PADDING_M,
    circumscribed_radius,
    clear_disc,
    coverage_space,
    default_turn_clearance,
    plan_coverage,
    plan_summary,
    turn_pose_clearance,
)
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT
from test_sim_coverage import RES, box, depot_like, grid_from_boxes

REPO = Path(__file__).resolve().parents[3]


def _grid_turn_clearance(space, pose):
    """Независимо от модуля: перебор всех несвободных клеток, расстояние от позы до квадрата клетки."""
    rr, cc = np.nonzero(~space.free)
    x0 = space.origin_x + cc * space.resolution
    y0 = space.origin_y + rr * space.resolution
    dx = np.maximum(np.maximum(x0 - pose.x, pose.x - (x0 + space.resolution)), 0.0)
    dy = np.maximum(np.maximum(y0 - pose.y, pose.y - (y0 + space.resolution)), 0.0)
    return float(np.hypot(dx, dy).min())


def _turn_poses(plan):
    return [p for p in plan.poses if p.kind in ("start", "end")]


def _strip_along_long_sides(space, racks, width=0.35, end_margin=0.7):
    """Покрываемый пол в полосе ``width`` вдоль длинных сторон стеллажей, без торцов (там полосы подрезаны)."""
    r, c = np.indices(space.free.shape)
    x = space.origin_x + (c + 0.5) * space.resolution
    y = space.origin_y + (r + 0.5) * space.resolution
    m = np.zeros_like(space.free)
    for b in racks:
        along = (x >= b.cx - b.sx / 2 + end_margin) & (x <= b.cx + b.sx / 2 - end_margin)
        above = (y > b.cy + b.sy / 2) & (y < b.cy + b.sy / 2 + width)
        below = (y < b.cy - b.sy / 2) & (y > b.cy - b.sy / 2 - width)
        m |= along & (above | below)
    return m & space.coverable


# ── одно место правды ───────────────────────────────────────────────────────


def test_turn_clearance_is_built_from_footprint_guard_and_padding():
    assert circumscribed_radius() == pytest.approx(math.hypot(0.3064, 0.3025))
    assert default_turn_clearance() == pytest.approx(math.hypot(0.3064, 0.3025) + 0.20 + 0.03)
    assert default_turn_clearance() == pytest.approx(circumscribed_radius() + GUARD_STOP_M + NAV2_PADDING_M)


def test_guard_stop_mirrors_proximity_thresholds():
    sys.path.insert(0, str(REPO / "src" / "rob_box_teleop"))
    from rob_box_teleop.proximity import Thresholds

    assert GUARD_STOP_M == Thresholds().stop_m


def test_padding_mirrors_nav2_params():
    doc = yaml.safe_load((REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml").read_text("utf-8"))
    for costmap in ("local_costmap", "global_costmap"):
        assert doc[costmap][costmap]["ros__parameters"]["footprint_padding"] == NAV2_PADDING_M


# ── круг свободы ─────────────────────────────────────────────────────────────


def test_clear_disc_matches_brute_force():
    rng = np.random.default_rng(4560)
    free = rng.random((40, 50)) > 0.03
    r_m, res = 0.33, 0.05
    got = clear_disc(free, r_m, res)
    occ = np.argwhere(~free)
    for i in range(free.shape[0]):
        for j in range(free.shape[1]):
            edge = min(i + 0.5, free.shape[0] - i - 0.5, j + 0.5, free.shape[1] - j - 0.5)
            d = np.hypot(np.maximum(np.abs(occ[:, 0] - i) - 0.5, 0), np.maximum(np.abs(occ[:, 1] - j) - 0.5, 0))
            expect = min(edge, d.min()) * res >= r_m
            assert got[i, j] == expect, (i, j)


# ── план ─────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("axis", ["x", "y", "auto"])
def test_no_turn_pose_closer_than_turn_clearance(axis):
    grid, world = depot_like()
    space = coverage_space(grid, (0.0, 0.0))
    plan = plan_coverage(space, axis=axis)
    poses = _turn_poses(plan)
    assert poses
    tc = default_turn_clearance()
    worst = min(_grid_turn_clearance(space, p) for p in poses)
    assert worst >= tc, worst
    assert turn_pose_clearance(space, plan.poses) == pytest.approx(worst)
    # и по коробкам мира: растр сдвигает границу не больше чем на клетку
    for p in poses:
        d = min(math.hypot(max(abs(p.x - b.cx) - b.sx / 2, 0), max(abs(p.y - b.cy) - b.sy / 2, 0)) for b in world)
        assert d >= tc - RES, (p, d)
    assert "min_turn_clear_m=" in plan_summary(plan, space)


def test_lanes_keep_0336_along_racks_and_beat_uniform_erosion():
    grid, world = depot_like()
    racks = [b for b in world if b.name.startswith("rack")]
    old = coverage_space(grid, (0.0, 0.0), turn_clearance_m=0.0)
    new = coverage_space(grid, (0.0, 0.0))
    wide = coverage_space(grid, (0.0, 0.0), erosion_m=0.65, turn_clearance_m=0.0)
    cov = {k: covered_mask(s, plan_track(plan_coverage(s).poses)) for k, s in (("old", old), ("new", new))}
    strip = _strip_along_long_sides(new, racks)
    assert strip.sum() * RES * RES > 20.0
    # у стеллажей вдоль полос — не хуже сужения 0.336 без мест разворота
    assert (strip & cov["new"]).sum() >= (strip & cov["old"]).sum()
    assert (strip & cov["new"]).sum() == strip.sum()
    # а целиком — больше, чем сужение 0.65 (попытка 2 на Depot)
    wide_cov = coverage_result(wide, covered_mask(wide, plan_track(plan_coverage(wide).poses)))
    assert coverage_result(new, cov["new"]).covered_m2 > wide_cov.covered_m2 + 30.0


def test_wall_rows_without_turn_room_are_dropped_and_counted():
    grid = grid_from_boxes(0, 0, 10, 8, [])
    space = coverage_space(grid, (1.0, 1.0))
    plan = plan_coverage(space, axis="x", cross=False)
    assert plan.lanes_dropped_no_turn > 0
    ys = sorted({round(p.y, 3) for p in plan.poses})
    # крайняя полоса — первая строка с местом разворота, а не 0.336 у стены
    assert ys[0] >= default_turn_clearance() and ys[-1] <= 8 - default_turn_clearance()
    assert max(b - a for a, b in zip(ys, ys[1:])) <= 0.55 + 1e-9
    pct = coverage_result(space, covered_mask(space, plan_track(plan.poses))).pct
    assert 80.0 < pct < 95.0  # полоса у стен честно не покрыта


def test_dead_end_aisle_narrower_than_two_turn_radii_is_not_planned():
    # тупик 1.1 × 4 м с запада от зала 6 × 6: развернуться в нём негде, задом план не выезжает
    obs = [box("n", 2.0, 4.15, 4.0, 3.7), box("s", 2.0, 0.6, 4.0, 1.2)]
    grid = grid_from_boxes(0, 0, 10, 6, obs)
    space = coverage_space(grid, (7.0, 3.0))
    plan = plan_coverage(space)
    assert all(p.x > 3.0 for p in plan.poses), "в тупик полосы не заводятся"
    zero = coverage_space(grid, (7.0, 3.0), turn_clearance_m=0.0)
    assert any(p.x < 3.0 for p in plan_coverage(zero).poses), "без мест разворота тупик был бы в плане"


def test_cross_pass_covers_ends_at_columns():
    cols = [box(f"c{k}", 2.0 + 2.0 * k, 4.0, 0.4, 0.4) for k in range(4)]
    grid = grid_from_boxes(0, 0, 10, 8, cols)
    space = coverage_space(grid, (1.0, 1.0))
    plain = plan_coverage(space, axis="x", cross=False)
    crossed = plan_coverage(space, axis="x")
    assert crossed.cross_lanes > 0
    pct = [coverage_result(space, covered_mask(space, plan_track(p.poses))).pct for p in (plain, crossed)]
    assert pct[1] > pct[0]
    assert turn_pose_clearance(space, crossed.poses) >= default_turn_clearance()


def test_zero_turn_clearance_keeps_old_plan():
    grid, _ = depot_like()
    space = coverage_space(grid, (0.0, 0.0), turn_clearance_m=0.0)
    assert (space.turn == space.centers).all()
    plan = plan_coverage(space, cross=False)
    assert plan.lanes_dropped_no_turn == 0 and plan.clipped_m == 0.0
    assert turn_pose_clearance(space, plan.poses) < default_erosion_plus_cell()


def default_erosion_plus_cell():
    return ROBOT_FOOTPRINT.x_max + NAV2_PADDING_M + 2 * RES
