"""Поза по тегу с известной позой (#4346, вариант B): геометрия и решение «поправить / согласен»."""

from __future__ import annotations

import math

import pytest

from rob_box_perception.tag_relocalization import (
    RelocalizerConfig,
    TagRelocalizer,
    Transform,
    base_pose_from_tag,
    parse_tags,
    tag_pose_on_wall,
)


def _yaw_tf(x, y, yaw, z=0.0):
    return Transform(x, y, z, (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


def _base_tag(robot, map_tag):
    """Что увидит робот в позе ``robot`` (x, y, yaw): T_base_tag = (T_map_base)⁻¹ · T_map_tag."""
    return _yaw_tf(*robot).inverse().compose(map_tag)


def test_tag_frame_on_west_facing_wall():
    """Лицо тега на запад (курс π): Z тега — на восток (в стену), Y — вниз, X — на юг (вправо смотрящему)."""
    t = tag_pose_on_wall(2.0, 1.0, 0.55, math.pi)
    assert t.rotate((0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-9)
    assert t.rotate((0, 1, 0)) == pytest.approx((0, 0, -1), abs=1e-9)
    assert t.rotate((1, 0, 0)) == pytest.approx((0, -1, 0), abs=1e-9)


def test_robot_facing_tag_sees_it_straight_ahead_in_camera_axes():
    """Робот в (0, 1) смотрит на восток на тег в (2, 1): тег в 2 м впереди, Z тега — от робота."""
    tag = tag_pose_on_wall(2.0, 1.0, 0.55, math.pi)
    bt = _base_tag((0.0, 1.0, 0.0), tag)
    assert (bt.x, bt.y, bt.z) == pytest.approx((2.0, 0.0, 0.55), abs=1e-9)
    assert bt.rotate((0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-9)


@pytest.mark.parametrize("robot", [(0.0, 0.0, 0.0), (0.5, -0.3, 0.4), (-1.0, 2.0, -2.5), (1.2, 0.8, math.pi)])
def test_base_pose_from_tag_inverts_observation(robot):
    tag = tag_pose_on_wall(2.0, 1.0, 0.55, math.pi)
    x, y, yaw = base_pose_from_tag(tag, _base_tag(robot, tag))
    assert (x, y) == pytest.approx(robot[:2], abs=1e-9)
    assert math.atan2(math.sin(yaw - robot[2]), math.cos(yaw - robot[2])) == pytest.approx(0.0, abs=1e-9)


def test_transform_inverse_roundtrip():
    t = Transform(1.0, -2.0, 0.3, (0.1, 0.2, 0.3, math.sqrt(1 - 0.14)))
    i = t.compose(t.inverse())
    assert (i.x, i.y, i.z) == pytest.approx((0, 0, 0), abs=1e-9)
    assert abs(i.q[3]) == pytest.approx(1.0, abs=1e-9)


TAG = tag_pose_on_wall(2.0, 1.0, 0.55, math.pi)


def _reloc(**kw):
    return TagRelocalizer(RelocalizerConfig(tags={21: TAG}, **kw))


def _feed(r, robot, estimate, n=5, t0=0.0, dt=0.2, tag_id=21):
    out = None
    for i in range(n):
        out = r.observe(t0 + i * dt, tag_id, _base_tag(robot, TAG), estimate)
    return out


def test_wrong_estimate_after_restart_is_corrected():
    """Робот в (0.5, 0, π), rtabmap думает (0, 0, π) — как «дрейф на полметра» после перезапуска."""
    d = _feed(_reloc(), (0.5, 0.0, 0.3), (0.0, 0.0, 0.3))
    assert d is not None and d.kind == "correct"
    assert d.pose[:2] == pytest.approx((0.5, 0.0), abs=1e-6)
    assert d.error_m == pytest.approx(0.5, abs=1e-6)


def test_agrees_when_rtabmap_is_right():
    d = _feed(_reloc(), (0.5, 0.0, 0.3), (0.52, 0.01, 0.31))
    assert d is not None and d.kind == "agree"


def test_needs_consistent_streak():
    r = _reloc()
    assert _feed(r, (0.5, 0.0, 0.3), (0.0, 0.0, 0.3), n=4) is None
    # разброс больше 5 см — не решаем
    r = _reloc()
    for i in range(6):
        jitter = 0.1 if i % 2 else 0.0
        d = r.observe(i * 0.2, 21, _base_tag((0.5 + jitter, 0.0, 0.3), TAG), (0.0, 0.0, 0.3))
        assert d is None


def test_unknown_tag_and_far_tag_ignored():
    assert _feed(_reloc(), (0.5, 0.0, 0.3), None, tag_id=7) is None
    assert _feed(_reloc(max_range_m=1.0), (0.0, 1.0, 0.0), None) is None  # тег в 2 м


def test_cooldown_after_correction():
    r = _reloc(cooldown_s=10.0)
    assert _feed(r, (0.5, 0.0, 0.3), (0.0, 0.0, 0.3)).kind == "correct"
    assert _feed(r, (0.5, 0.0, 0.3), (0.0, 0.0, 0.3), t0=2.0) is None
    assert _feed(r, (0.5, 0.0, 0.3), (0.0, 0.0, 0.3), t0=12.0).kind == "correct"


def test_no_estimate_yet_means_correct():
    assert _feed(_reloc(), (0.5, 0.0, 0.3), None).kind == "correct"


def test_parse_tags_from_tape_measure_rows():
    tags = parse_tags([[21, 1.95, 1.0, 0.55, math.pi], [22, -1.45, -1.0, 0.55, 0.0]])
    assert sorted(tags) == [21, 22]
    with pytest.raises(ValueError, match="дважды"):
        parse_tags([[21, 0, 0, 0, 0], [21, 1, 1, 1, 0]])
    with pytest.raises(ValueError, match="id, x, y, z"):
        parse_tags([[21, 0, 0, 0]])
