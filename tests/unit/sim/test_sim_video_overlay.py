"""Видео из Gazebo: мир с камерами (rob_box_sim.video_overlay).

Проверяется то, что увидит Gazebo и стенд: камеры — сенсоры, которые пишут
кадры; область «спавн + цели» целиком в кадре вида сверху; препятствия мира
для физики и метрик не изменились; карту квартиры снимать нельзя.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from rob_box_sim import video_overlay as vo
from rob_box_sim.geometry2d import Pose2D
from rob_box_sim.world_sdf import boxes_from_sdf, load_world_boxes

REPO = Path(__file__).resolve().parents[3]
SIM = REPO / "src" / "rob_box_sim"
WORLDS = SIM / "worlds"
VIDEO_SCENARIOS = SIM / "scenarios" / "video.yaml"
BASELINE = SIM / "scenarios" / "baseline.yaml"
SCENES = ("polygon", "turn90")


def _built(name: str) -> str:
    text = (WORLDS / f"{name}.sdf").read_text(encoding="utf-8")
    return vo.build(text, vo.scenario_goals(VIDEO_SCENARIOS, name), "/sim_video/frames")


def _sensors(text: str) -> dict[str, ET.Element]:
    world = ET.fromstring(text).find("world")
    return {s.get("name"): s for s in world.iter("sensor")}


@pytest.mark.parametrize("name", SCENES)
def test_cameras_are_saving_sensors(name):
    sensors = _sensors(_built(name))
    assert set(sensors) == {"video_top", "video_chase"}
    for cam, s in sensors.items():
        assert s.get("type") == "camera"
        save = s.find("camera/save")
        assert save is not None and save.get("enabled") == "true"
        assert save.findtext("path") == f"/sim_video/frames/{cam}"
        w = int(s.findtext("camera/image/width"))
        h = int(s.findtext("camera/image/height"))
        assert h % 2 == 0 and w % 2 == 0  # yuv420p
        assert float(s.findtext("update_rate")) == vo.DEFAULT_RATE_HZ


@pytest.mark.parametrize("name", SCENES)
def test_obstacles_unchanged_for_physics_and_metrics(name):
    assert boxes_from_sdf(_built(name)) == load_world_boxes(WORLDS / f"{name}.sdf")


@pytest.mark.parametrize("name", SCENES)
def test_video_goals_are_bench_goals(name):
    base = dict(vo.scenario_goals(BASELINE, name))
    for goal_name, goal in vo.scenario_goals(VIDEO_SCENARIOS, name):
        assert base[goal_name] == goal


def _project_top(cam: vo.Camera, x: float, y: float) -> tuple[float, float]:
    """Точка пола в нормированных координатах кадра вида сверху (−1..1, +u вправо, +v вверх)."""
    tan_h = math.tan(cam.hfov / 2)
    tan_v = tan_h / vo.ASPECT
    return (x - cam.x) / (cam.z * tan_h), (y - cam.y) / (cam.z * tan_v)


@pytest.mark.parametrize("name", SCENES)
def test_start_and_goals_inside_top_view(name):
    goals = [g for _, g in vo.scenario_goals(VIDEO_SCENARIOS, name)]
    roi = vo.roi_for([Pose2D(0, 0), *goals])
    cam = vo.top_camera(roi)
    for p in [Pose2D(0, 0), *goals]:
        u, v = _project_top(cam, p.x, p.y)
        assert abs(u) < 1 and abs(v) < 1, (p, u, v)


def test_chase_camera_looks_at_roi_center_from_behind():
    roi = vo.Roi(-1, 4, -1, 1)
    cam = vo.chase_camera(roi)
    assert cam.x < roi.x0 and cam.y < roi.y0
    dx, dy = roi.cx - cam.x, roi.cy - cam.y
    assert cam.yaw == pytest.approx(math.atan2(dy, dx))
    # ось взгляда попадает в центр области на полу
    assert cam.z / math.tan(cam.pitch) == pytest.approx(math.hypot(dx, dy))


def test_world_without_sensors_system_is_rejected():
    with pytest.raises(ValueError):
        vo.overlay("<sdf><world name='w'></world></sdf>", [], [], "/f")


def test_private_flat_map_is_refused(tmp_path):
    with pytest.raises(SystemExit):
        vo.main(["--world", str(WORLDS / "polygon.sdf"), "--name", "local/robot_map",
                 "--scenarios", str(VIDEO_SCENARIOS), "--out", str(tmp_path / "w.sdf")])
