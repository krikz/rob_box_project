"""TF потолочной камеры: оптическая ось смотрит вверх, а не вперёд.

Камера стоит вертикально вверх (docs/architecture/HARDWARE.md §3.3). До 04.11.2026
оптический кадр ``ceiling_camera_optical_frame`` в URDF смотрел вперёд (+X
base_link): поза тега от потолочного AprilTag легла бы в TF повёрнутой на 90°.

Тест читает эталон ``golden/rob_box_real.urdf`` — он обязан совпадать с выводом
xacro (test_xacro_sim_arg.py), поэтому xacro здесь не нужен — и сводит цепочку
fixed-джойнтов base_link → оптический кадр.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

GOLDEN = Path(__file__).resolve().parent / "golden" / "rob_box_real.urdf"

Mat = list  # 3×3, строки


def _rpy(r: float, p: float, y: float) -> Mat:
    """URDF rpy → матрица поворота (Rz(y)·Ry(p)·Rx(r)), как в urdfdom."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mul(a: Mat, b: Mat) -> Mat:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(m: Mat, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _vec(text: str):
    return [float(x) for x in text.split()]


@pytest.fixture(scope="module")
def joints() -> dict:
    root = ET.parse(GOLDEN).getroot()
    out = {}
    # Только джойнты URDF верхнего уровня: <joint> внутри <ros2_control> — другое.
    for j in root.findall("joint"):
        origin = j.find("origin")
        if origin is None:
            origin = ET.Element("origin")
        out[j.find("child").get("link")] = (
            j.find("parent").get("link"),
            _vec(origin.get("xyz", "0 0 0")),
            _rpy(*_vec(origin.get("rpy", "0 0 0"))),
        )
    return out


def _pose_in_base(joints: dict, link: str):
    """Поворот и смещение ``link`` относительно base_link."""
    rot = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    pos = [0.0, 0.0, 0.0]
    chain = []
    while link != "base_link":
        parent, xyz, r = joints[link]
        chain.append((xyz, r))
        link = parent
    for xyz, r in reversed(chain):
        step = _apply(rot, xyz)
        pos = [pos[i] + step[i] for i in range(3)]
        rot = _mul(rot, r)
    return rot, pos


def _assert_dir(actual, expected, what: str):
    assert all(abs(a - e) < 1e-3 for a, e in zip(actual, expected)), f"{what}: {actual} ≠ {expected}"


def test_ceiling_optical_axis_points_up(joints):
    rot, _ = _pose_in_base(joints, "ceiling_camera_optical_frame")
    _assert_dir(_apply(rot, [0, 0, 1]), [0, 0, 1], "оптическая ось Z")


def test_ceiling_image_top_points_forward_and_right_edge_left(joints):
    """Поворот кадра вокруг оси — допущение, не замер (см. комментарий в xacro)."""
    rot, _ = _pose_in_base(joints, "ceiling_camera_optical_frame")
    _assert_dir(_apply(rot, [0, -1, 0]), [1, 0, 0], "верх кадра (−Y оптики)")
    _assert_dir(_apply(rot, [1, 0, 0]), [0, 1, 0], "правый край кадра (+X оптики)")


def test_ceiling_camera_position_unchanged(joints):
    """Позиция — из экспорта Fusion 360; рулеткой не мерили."""
    _, pos = _pose_in_base(joints, "ceiling_camera_optical_frame")
    _assert_dir(pos, [-0.170663, 0.065, 0.4615], "позиция оптического кадра")


def test_oak_d_optical_axis_still_forward(joints):
    """Правка макроса usb_camera не задела OAK-D: его оптика смотрит вперёд."""
    rot, _ = _pose_in_base(joints, "camera_color_optical_frame")
    _assert_dir(_apply(rot, [0, 0, 1]), [1, 0, 0], "оптическая ось OAK-D")
