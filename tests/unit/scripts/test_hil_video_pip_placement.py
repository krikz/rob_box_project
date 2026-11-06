"""Врезка 3D-камеры в ролике HIL не закрывает робота: решение принимает код (scripts/hil/video/pip_placement.py)."""

import csv
import importlib.util
import math
import sys
from pathlib import Path

import pytest

_MOD = Path(__file__).resolve().parents[3] / "scripts" / "hil" / "video" / "pip_placement.py"
_spec = importlib.util.spec_from_file_location("pip_placement", _MOD)
pp = importlib.util.module_from_spec(_spec)
sys.modules["pip_placement"] = pp
_spec.loader.exec_module(pp)

FPS = 20
MIN_GAP = 40  # запас между рамкой врезки и коробкой робота, пикселей


def _run(path):
    """Прогнать позы (пиксели) через PipPlacer, вернуть [(поза, левый верхний угол врезки)]."""
    placer = pp.PipPlacer(FPS)
    res = []
    for rx, ry in path:
        x, y, a = placer.update(rx, ry)
        res.append(((rx, ry), (x, y), a))
    return res


def _gap(rob, pos):
    return pp.gap(pp.rect_at(*pos), pp.robot_box(*rob))


def _line(a, b, n):
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def test_robot_in_center_keeps_default_corner():
    out = _run([(640, 360)] * 60)
    assert {pos for _, pos, _ in out} == {pp.CORNERS[pp.DEFAULT_CORNER]}


def test_robot_far_from_pip_keeps_corner():
    out = _run(_line((100, 100), (500, 300), 100))
    assert {pos for _, pos, _ in out} == {pp.CORNERS["br"]}


def test_robot_drives_into_bottom_right_pip_leaves():
    out = _run(_line((640, 360), (960, 480), 200) + [(960, 480)] * 40)
    assert out[-1][1] != pp.CORNERS["br"]
    assert all(_gap(rob, pos) >= MIN_GAP for rob, pos, a in out if a > 0.05)


@pytest.mark.parametrize(
    "target", [(960, 520), (1100, 560), (300, 560), (200, 120), (1100, 100), (640, 560), (640, 100)]
)
def test_final_position_clear_of_robot(target):
    out = _run(_line((640, 360), target, 300) + [target] * 60)
    rob, pos, _ = out[-1]
    assert _gap(rob, pos) >= MIN_GAP


def test_every_synthetic_stationary_pose_has_a_clear_corner():
    for x in range(0, pp.W + 1, 40):
        for y in range(0, pp.H + 1, 40):
            best = max(pp.corner_gap(c, x, y) for c in pp.CORNERS)
            assert best >= MIN_GAP, (x, y)


def test_hysteresis_no_flicker_on_boundary():
    """Робот колеблется у границы LEAVE_DIST: врезка не мельтешит, максимум одно переключение."""
    rx = 788 - pp.ROBOT_HALF - pp.LEAVE_DIST  # на границе от правой нижней врезки
    path = [(rx + 6 * math.sin(i / 3), 500 + 4 * math.cos(i / 5)) for i in range(400)]
    out = _run(path)
    corners = []
    for _, pos, _ in out:
        if pos in pp.CORNERS.values() and (not corners or corners[-1] != pos):
            corners.append(pos)
    assert len(corners) <= 2, corners


def test_transition_takes_half_second_and_is_smooth():
    placer = pp.PipPlacer(FPS)
    xs = [placer.update(700, 500)[1] for _ in range(40)]  # робот слева от врезки: она уезжает вверх по правому краю
    moving = [i for i in range(1, len(xs)) if xs[i] != xs[i - 1]]
    assert 8 <= len(moving) <= 12  # 0.5 с при 20 к/с
    steps = [abs(xs[i] - xs[i - 1]) for i in moving]
    assert max(steps) < 0.4 * abs(xs[-1] - xs[0])  # без скачка


def test_real_route_poses_never_hidden():
    """Поза робота по кадрам реального прогона (если csv есть рядом): врезка и робот не пересекаются."""
    csv_path = Path(__file__).resolve().parent / "fixtures" / "hil_run1_poses.csv"
    if not csv_path.exists():
        pytest.skip("нет fixtures/hil_run1_poses.csv")
    cx, cy, pxm = 10.5, 0.0, 40.0
    rows = [(float(r["t"]), float(r["tx"]), float(r["ty"])) for r in csv.DictReader(csv_path.open(encoding="utf-8"))]
    placer = pp.PipPlacer(FPS)
    worst = 1e9
    t = 31.0
    while t < 240.0:
        _, x, y = min(rows, key=lambda r: abs(r[0] - t))
        rx, ry = 640 + (x - cx) * pxm, 360 - (y - cy) * pxm
        x_, y_, a = placer.update(rx, ry)
        if a > 0.05:
            worst = min(worst, _gap((rx, ry), (x_, y_)))
        t += 0.25
    assert worst >= MIN_GAP


def test_fade_when_straight_path_would_cross_robot():
    """Робот у врезки справа: ехать через него нельзя, врезка гаснет и проявляется в другом углу."""
    placer = pp.PipPlacer(FPS, corner="tr")
    out = []
    for _ in range(40):
        x, y, a = placer.update(1000, 330)
        out.append(((1000, 330), (x, y), a))
    assert placer.mode == "fade" and placer.corner in ("tl", "bl")
    assert min(a for _, _, a in out) < 0.05 and out[-1][2] == 1.0
    new = pp.CORNERS[placer.corner]
    assert all(_gap(rob, pos) >= MIN_GAP for rob, pos, a in out if a > 0.05 and pos == new)
    assert not any(pos not in (new, pp.CORNERS["tr"]) for _, pos, _ in out)  # по пути робота не пересекает
