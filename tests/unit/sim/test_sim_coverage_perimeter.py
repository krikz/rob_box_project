"""Проход по периметру (#4607): нырки полос к стенам без разворота у стены и без заднего хода.

Безопасность проверяется настоящей функцией защиты ``rob_box_teleop.proximity.nearest_threat``: точки стен —
поверхность коробок мира (не растр карты), робот на каждой позе пути едет вперёд 0.15 м/с.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from rob_box_sim.coverage_metric import CoverageResult, GoalOutcome, coverage_result, covered_mask, plan_track
from rob_box_sim.coverage_perimeter import (
    GUARD_CLOSING_MPS,
    GUARD_CORRIDOR_MARGIN_M,
    GUARD_MIN_POINTS,
    GUARD_SIDE_STOP_M,
    RAMP_GUARD_SHARE,
    Obstacles,
    RampRule,
    default_wall_offset,
    dip_depth,
    dip_envelope,
    dip_profile,
    dip_spans,
    erode_rect,
    footprint_hits,
    guard_band,
    guard_reach,
    guard_stops,
    perimeter_done,
    perimeter_summary,
    perimeter_track,
    plan_perimeter,
    reverse_m,
)
from rob_box_sim.coverage_plan import GUARD_STOP_M, NAV2_PADDING_M, SPEED_MPS, coverage_space, plan_coverage
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT
from test_sim_coverage import RES, box, depot_like, grid_from_boxes, walls

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_teleop"))
from rob_box_teleop import proximity as px  # noqa: E402

BODY = px.Body(ROBOT_FOOTPRINT.x_min, ROBOT_FOOTPRINT.x_max, ROBOT_FOOTPRINT.y_min, ROBOT_FOOTPRINT.y_max)
TH = px.Thresholds()


def _surface(boxes, step=0.01):
    """Точки по контуру коробок через ``step`` м — как плотный скан лидара по поверхности."""
    pts = []
    for b in boxes:
        x0, x1, y0, y1 = b.cx - b.sx / 2, b.cx + b.sx / 2, b.cy - b.sy / 2, b.cy + b.sy / 2
        xs = np.arange(x0, x1 + 1e-9, step)
        ys = np.arange(y0, y1 + 1e-9, step)
        for y in (y0, y1):
            pts.append(np.column_stack((xs, np.full_like(xs, y))))
        for x in (x0, x1):
            pts.append(np.column_stack((np.full_like(ys, x), ys)))
    return np.concatenate(pts)


def _to_body(points, pose):
    x, y, yaw = pose
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.column_stack((math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy))


def _real_zone(points, pose, v=SPEED_MPS):
    near = points[np.hypot(points[:, 0] - pose[0], points[:, 1] - pose[1]) < 1.5]
    return px.classify(px.nearest_threat(BODY, _to_body(near, pose), v, 0.0, TH), TH)


# ── одно место правды ───────────────────────────────────────────────────────


def test_guard_constants_mirror_proximity_thresholds():
    assert GUARD_SIDE_STOP_M == TH.side_stop_m
    assert GUARD_CORRIDOR_MARGIN_M == TH.corridor_margin_m
    assert GUARD_CLOSING_MPS == TH.closing_mps
    assert GUARD_MIN_POINTS == TH.min_points
    assert GUARD_STOP_M == TH.stop_m


def test_wall_offset_is_built_from_footprint_side_stop_and_padding():
    assert default_wall_offset() == pytest.approx(0.3025 + 0.08 + 0.03)
    assert default_wall_offset() == pytest.approx(ROBOT_FOOTPRINT.y_max + GUARD_SIDE_STOP_M + NAV2_PADDING_M)
    assert guard_band() == pytest.approx(0.3025 + 0.05)
    assert guard_reach() == pytest.approx(0.3064 + 0.20)


def test_real_guard_stops_along_a_wall_at_0336_and_not_at_wall_offset():
    """Почему не 0.336 (сужение змейки): стена ближе 0.05 м к борту — в коридоре хода, стоп с 0.20 м."""
    wall = _surface([box("w", 0.0, -1.0, 6.0, 0.05)])
    face = -1.0 + 0.025
    assert _real_zone(wall, (0.0, face + 0.336, 0.0)) == px.ZONE_STOP
    for yaw_deg in (0.0, -2.0, -4.0):
        assert _real_zone(wall, (0.0, face + default_wall_offset(), math.radians(yaw_deg))) != px.ZONE_STOP


def test_grid_guard_is_the_proximity_rule():
    """``guard_stops`` по тем же точкам и позе решает так же, как ``proximity.nearest_threat`` (ход вперёд, wz = 0)."""
    rng = np.random.default_rng(4607)
    disagree = 0
    for _ in range(300):
        pts = rng.uniform(-0.9, 0.9, size=(rng.integers(2, 12), 2))
        pose = (0.0, 0.0, 0.0)
        mine = bool(guard_stops(Obstacles(pts, pts), np.array([pose]))[0])
        real = px.classify(px.nearest_threat(BODY, pts, SPEED_MPS, 0.0, TH), TH) == px.ZONE_STOP
        disagree += mine != real
    assert disagree == 0


def test_footprint_hits_counts_padding():
    pts = np.array([[ROBOT_FOOTPRINT.x_max + NAV2_PADDING_M - 0.005, 0.0]])
    assert footprint_hits(Obstacles(pts, pts), np.array([[0.0, 0.0, 0.0]]))[0]
    assert not footprint_hits(Obstacles(pts, pts), np.array([[-0.02, 0.0, 0.0]]))[0]


# ── геометрия нырка ─────────────────────────────────────────────────────────


def test_erode_rect_matches_brute_force():
    rng = np.random.default_rng(1)
    free = rng.random((30, 40)) > 0.04
    rows_m, cols_m = 0.4125, 0.336
    got = erode_rect(free, rows_m, cols_m, RES)
    occ = np.argwhere(~free)
    h, w = free.shape
    for i in range(h):
        for j in range(w):
            near = (np.maximum(np.abs(occ[:, 0] - i) - 0.5, 0) * RES < rows_m) & (
                np.maximum(np.abs(occ[:, 1] - j) - 0.5, 0) * RES < cols_m
            )
            edge = min(i + 0.5, h - i - 0.5) * RES < rows_m or min(j + 0.5, w - j - 0.5) * RES < cols_m
            assert got[i, j] == (not near.any() and not edge), (i, j)


def test_dip_depth_only_toward_a_close_wall():
    strip = np.ones((20, 10), dtype=bool)
    strip[:3] = False  # стена снизу: строки 0..2 вне пространства прохода
    assert dip_depth(strip, (8, 0, 9), -1, 6).tolist() == [5] * 10  # 7, 6, 5, 4, 3 — внутри, 2 — край
    assert dip_depth(strip, (8, 0, 9), +1, 6).tolist() == [0] * 10  # сверху края нет в 6 строках
    strip[3, 4] = False  # выступ стены
    assert dip_depth(strip, (8, 0, 9), -1, 6)[4] == 4


def test_envelope_spreads_an_obstacle_over_nose_and_stop_zone():
    depth = np.full(40, 5)
    depth[20] = 1  # пилястра
    reach = math.ceil((guard_reach() + NAV2_PADDING_M) / RES)
    env = dip_envelope(depth, reach)
    assert reach == 11
    assert env[20 - reach : 20 + reach + 1].tolist() == [1] * (2 * reach + 1)
    assert env[20 - reach - 1] == 5 and env[20 + reach + 1] == 5


def test_dip_profile_keeps_envelope_ends_and_ramp():
    rule = RampRule(RES, default_wall_offset(), guard_band(), guard_reach())
    env = np.array([0] + [5] * 60 + [2] * 10 + [5] * 60 + [0], dtype=float)
    m = dip_profile(env, rule)
    assert m[0] == 0 and m[-1] == 0
    assert np.all(m <= env + 1e-9)
    for j in range(1, len(m)):  # склон в обе стороны не круче правила
        assert m[j] - m[j - 1] <= rule.step(env[j], m[j - 1]) + 1e-9
        assert m[j - 1] - m[j] <= rule.step(env[j - 1], m[j]) + 1e-9
    assert m.max() == pytest.approx(5.0)


def test_ramp_step_is_a_share_of_what_the_guard_allows():
    rule = RampRule(RES, default_wall_offset(), guard_band(), guard_reach())
    # у самой стены (зазор = отступ): tg θ ≤ доля × (0.4125 − 0.3525) / 0.5064
    assert rule.step(5, 5) == pytest.approx(RAMP_GUARD_SHARE * (default_wall_offset() - 0.3525) / 0.5064)
    assert rule.step(5, 0) > rule.step(5, 5)  # дальше от стены — круче можно
    assert RampRule(RES, 0.30, guard_band(), guard_reach()).step(0, 0) == 0.0  # в коридоре — никак


def test_dip_spans_end_at_turn_places_or_are_skipped():
    m = np.zeros(30)
    m[5:10] = 1.0
    m[20:25] = 1.0
    turn = np.zeros(30, dtype=bool)
    turn[[2, 12, 27]] = True
    assert dip_spans(m, turn) == [(2, 27)]  # общее место разворота — один путь, без лишней остановки
    turn[15] = True
    assert dip_spans(m, turn) == [(2, 12), (15, 27)]
    turn[2] = False  # слева от первого нырка места разворота нет — только задом
    assert dip_spans(m, turn) == [(15, 27)]


# ── план на зале ────────────────────────────────────────────────────────────


def _hall():
    """Зал 12×6 м, на южной стене две пилястры 0.3×0.2 м (выступ в зал)."""
    x0, y0, x1, y1 = 0.0, 0.0, 12.0, 6.0
    obs = [box("p0", 4.0, 0.15, 0.3, 0.2), box("p1", 8.0, 0.15, 0.3, 0.2)]
    return grid_from_boxes(x0, y0, x1, y1, obs), obs + walls(x0, y0, x1, y1)


@pytest.fixture(scope="module")
def hall_plan():
    grid, world = _hall()
    space = coverage_space(grid, (6.0, 3.0))
    snake = plan_coverage(space)
    return space, snake, plan_perimeter(space, snake), world


def test_dips_reach_all_four_walls_and_add_floor(hall_plan):
    space, snake, plan, _ = hall_plan
    assert len(plan.strips) >= 4
    assert {(s.axis, s.side) for s in plan.strips} >= {("x", 1), ("x", -1), ("y", 1), ("y", -1)}
    base = covered_mask(space, plan_track(snake.poses))
    both = base | covered_mask(space, perimeter_track(plan))
    gain = coverage_result(space, both).covered_m2 - coverage_result(space, base).covered_m2
    assert gain > 4.0, gain
    # полоса у северной стены 0.11–0.36 м (змейка туда не дотягивается) теперь почти вся под корпусом
    r, c = np.indices(space.free.shape)
    y = space.origin_y + (r + 0.5) * RES
    x = space.origin_x + (c + 0.5) * RES
    band = (y > 6.0 - 0.05 - 0.36) & (y < 6.0 - 0.05 - 0.11) & (x > 3.0) & (x < 9.0) & space.coverable
    assert (band & base).sum() < 0.1 * band.sum()
    assert (band & both).sum() > 0.9 * band.sum()


def test_dip_ends_are_turn_places_and_nothing_is_reversed(hall_plan):
    space, _, plan, _ = hall_plan
    for k, path in plan.driven():
        for x, y, _ in (path[0], path[-1]):
            assert space.turn[math.floor((y - space.origin_y) / RES), math.floor((x - space.origin_x) / RES)]
        # ход вперёд: каждый следующий шаг по курсу позы, без заднего хода
        t = [(0.0, *p) for p in path]
        assert reverse_m(t) == 0.0


def test_no_pose_would_stop_the_real_guard_or_touch_a_wall(hall_plan):
    """Каждая поза пути в направлении проезда: настоящая защита по поверхности коробок мира — не «стоп»; корпус
    с padding до коробок не достаёт."""
    _, _, plan, world = hall_plan
    surface = _surface(world)
    fp = ROBOT_FOOTPRINT
    for _, path in plan.driven():
        for pose in path:
            assert _real_zone(surface, pose) != px.ZONE_STOP, pose
            b = _to_body(surface[np.hypot(*(surface - pose[:2]).T) < 1.0], pose)
            inside = (b[:, 0] > fp.x_min - NAV2_PADDING_M) & (b[:, 0] < fp.x_max + NAV2_PADDING_M)
            inside &= (b[:, 1] > fp.y_min - NAV2_PADDING_M) & (b[:, 1] < fp.y_max + NAV2_PADDING_M)
            assert not inside.any(), pose


def test_pilaster_is_passed_outside_the_guard_corridor(hall_plan):
    """У пилястр (выступ 0.2 м) нырок отходит от стены заранее: перед пилястрой корпус не в коридоре хода."""
    _, _, plan, world = hall_plan
    south = [s for s in plan.strips if s.axis == "x" and s.side == -1]
    assert south
    pil = _surface([b for b in world if b.name.startswith("p")])
    for s in south:
        for pose in s.path:
            assert _real_zone(pil, pose) != px.ZONE_STOP
            assert _real_zone(pil, (pose[0], pose[1], pose[2] + math.pi)) != px.ZONE_STOP


def test_summary_line_is_built_from_plan_and_metric(hall_plan):
    space, snake, plan, _ = hall_plan
    line = perimeter_summary(plan, space, snake)
    assert line.startswith(f"PERIMETER_PLAN wall_offset=0.4125 strips={len(plan.strips)} ")
    for key in ("dropped_guard=", "dropped_footprint=", "gain_m2=", "coverage=", "coverage_free=", "time_est_s="):
        assert key in line


def test_depot_like_gain():
    grid, _ = depot_like()
    space = coverage_space(grid, (0.0, 0.0))
    snake = plan_coverage(space)
    plan = plan_perimeter(space, snake)
    base = covered_mask(space, plan_track(snake.poses))
    gain = coverage_result(space, base | covered_mask(space, perimeter_track(plan))).covered_m2
    assert gain - coverage_result(space, base).covered_m2 > 10.0
    assert plan.dropped_guard == 0 and plan.dropped_footprint == 0


def test_zero_offset_room_gives_no_dips():
    """Отступ шире, чем место у стены, — нырков нет, план пустой, а не опасный."""
    grid, _ = _hall()
    space = coverage_space(grid, (6.0, 3.0))
    plan = plan_perimeter(space, plan_coverage(space), wall_offset_m=0.70)
    assert plan.strips == [] and plan.order == []


# ── итог прогона ────────────────────────────────────────────────────────────


def test_perimeter_done_counts_strips_and_coverage_delta():
    outs = [
        GoalOutcome("P0000a", "SUCCEEDED"),
        GoalOutcome("P0000f", "SUCCEEDED", em_episodes=1),
        GoalOutcome("P0001a", "ABORTED"),
        GoalOutcome.not_sent("P0001f"),
        GoalOutcome.not_sent("P0002a"),
        GoalOutcome.not_sent("P0002f"),
    ]
    before, after = CoverageResult(400.0, 356.1, 465.0, 357.0), CoverageResult(400.0, 370.0, 465.0, 371.0)
    line = perimeter_done(outs, before, after, back_m=0.25)
    assert line.startswith("PERIMETER_DONE strips=3 approach_ok=1 path_ok=1 not_sent=2 failed=(ABORTED=1) ")
    assert "em_episodes=1 reverse_m=0.25 gain_m2=13.9 covered_m2=356.1->370.0" in line


def test_reverse_m_counts_only_motion_against_heading():
    fwd = [(float(k), 0.1 * k, 0.0, 0.0) for k in range(5)]
    back = [(5.0 + k, 0.4 - 0.1 * k, 0.0, 0.0) for k in range(1, 4)]
    assert reverse_m(fwd) == 0.0
    assert reverse_m(fwd + back) == pytest.approx(0.3)
