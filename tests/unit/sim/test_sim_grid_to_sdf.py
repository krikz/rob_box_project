"""rob_box_sim.grid_to_sdf: карта rtabmap (pgm+yaml) → мир стен."""

from __future__ import annotations

import math
import random

import pytest
import yaml

from rob_box_sim.geometry2d import Pose2D, clearance
from rob_box_sim.grid_to_sdf import build, load_grid, read_pgm, rectangles
from rob_box_sim.world_sdf import load_world_boxes

FREE, OCC, UNKNOWN = 254, 0, 205  # значения rtabmap-export --map


def _write_map(tmp_path, rows, res=0.1, origin=(0.0, 0.0)):
    h, w = len(rows), len(rows[0])
    px = bytes(OCC if c == "#" else UNKNOWN if c == "?" else FREE for r in rows for c in r)
    (tmp_path / "m.pgm").write_bytes(b"P5\n# rtabmap\n%d %d\n255\n" % (w, h) + px)
    (tmp_path / "m.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "m.pgm",
                "resolution": res,
                "origin": [origin[0], origin[1], 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        )
    )
    return tmp_path / "m.yaml"


def _write_poses(tmp_path, poses):
    lines = ["#timestamp x y z qx qy qz qw id"]
    for i, (x, y, yaw) in enumerate(poses):
        lines.append(f"{i}.0 {x} {y} 0 0 0 {math.sin(yaw / 2)} {math.cos(yaw / 2)} {i + 1}")
    (tmp_path / "p.txt").write_text("\n".join(lines) + "\n")
    return tmp_path / "p.txt"


def test_read_pgm_skips_comment():
    w, h, maxval, px = read_pgm(b"P5\n# c\n2 1\n255\n\x00\xfe")
    assert (w, h, maxval, px) == (2, 1, 255, b"\x00\xfe")


def test_rectangles_cover_exactly_occupied_cells():
    rnd = random.Random(7)
    occ = [[rnd.random() < 0.3 for _ in range(23)] for _ in range(17)]
    covered = [[0] * 23 for _ in range(17)]
    for r0, r1, c0, c1 in rectangles(occ):
        for r in range(r0, r1):
            for c in range(c0, c1):
                covered[r][c] += 1
    assert covered == [[1 if v else 0 for v in row] for row in occ]


def test_rectangles_merge_solid_wall_into_one_box():
    occ = [[False, True, True, False]] * 5
    assert rectangles(occ) == [(0, 5, 1, 3)]


def test_unknown_is_not_wall(tmp_path):
    grid = load_grid(_write_map(tmp_path, ["??", "#."]))
    assert grid.occupied == [[False, False], [True, False]]


def test_build_moves_first_robot_pose_to_origin(tmp_path):
    # карта 4×3 м, 0.1 м/клетка: стена по верху (y ∈ [2.9, 3.0]) и по правому краю
    rows = ["#" * 40] + ["." * 39 + "#"] * 29
    y = _write_map(tmp_path, rows, origin=(-1.0, 0.0))
    # робот стартовал в (1, 1) лицом на +Y (к верхней стене), потом поехал к правой
    poses = _write_poses(tmp_path, [(1.0, 1.0, math.pi / 2), (1.5, 1.0, 0.0), (2.0, 1.5, 0.0)])
    # шаг 0.4, не 0.5: расстояние 0.5 до первой цели на границе float (в CI цель пропускалась)
    meta = build(y, poses, tmp_path / "out", "t", goal_step=0.4, max_goals=5)
    boxes = load_world_boxes(tmp_path / "out" / "t.sdf")
    assert meta["spawn_in_map"] == pytest.approx([1.0, 1.0, math.pi / 2], abs=1e-4)
    # верхняя стена в мире — прямо по курсу робота: её ближняя грань на 1.9 м впереди
    # правая стена карты (x 2.9) — справа от робота в 1.9 м, верхняя — впереди в 1.9 м
    assert clearance(Pose2D(0, 0, 0), boxes) == pytest.approx(min(1.9 - 0.3064, 1.9 - 0.27), abs=1e-3)
    ahead = [b for b in boxes if abs(b.cy) < 0.5 and b.cx > 1.0]
    assert ahead and min(x for b in ahead for x, _ in b.corners()) == pytest.approx(1.9, abs=1e-4)
    right = [b for b in boxes if b.cy < -1.0]
    assert right and max(y for b in right for _, y in b.corners()) == pytest.approx(-1.9, abs=1e-4)
    # цели — в мировой системе: (1.5, 1.0) карты = 0.5 м справа от робота
    g = meta["goals"][0]
    assert (g["x"], g["y"]) == pytest.approx((0.0, -0.5), abs=1e-3)


def test_spawn_inside_wall_fails_loudly(tmp_path):
    y = _write_map(tmp_path, ["#####"] * 5)
    poses = _write_poses(tmp_path, [(0.25, 0.25, 0.0)])
    with pytest.raises(ValueError, match="спавна"):
        build(y, poses, tmp_path / "out", "t")


def test_spawn_skips_poses_too_close_for_nav2(tmp_path):
    """Первая поза вплотную к стене (Nav2 с robot_radius 0.45 там «в стене») — спавн со следующей."""
    rows = ["#" * 40] + ["." * 40] * 29  # стена по верху, y ∈ [2.9, 3.0]
    y = _write_map(tmp_path, rows)
    poses = _write_poses(tmp_path, [(1.0, 2.5, 0.0), (1.0, 1.0, 0.0), (3.5, 1.0, 0.0)])
    meta = build(y, poses, tmp_path / "out", "t", goal_step=1.0)
    assert meta["spawn_pose_index"] == 1
    assert meta["spawn_in_map"] == pytest.approx([1.0, 1.0, 0.0], abs=1e-4)
    assert meta["spawn_center_clearance_m"] == pytest.approx(1.9, abs=1e-3)
    assert [(g["x"], g["y"]) for g in meta["goals"]] == [pytest.approx((2.5, 0.0), abs=1e-3)]
