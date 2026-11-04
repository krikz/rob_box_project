"""Миры сима: файлы worlds/*.sdf = генератор, и размеры те, что заявлены.

План docs/research/2026-11-04-gazebo-sim-navigation.md §5, §6 шаг 3. Проверяется
геометрия, которую увидит Gazebo (коробки, прочитанные обратно из SDF), а не текст.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from rob_box_sim import worlds_catalog as wc
from rob_box_sim.geometry2d import ROBOT_BODY_BOTTOM_Z, Pose2D, clearance
from rob_box_sim.world_sdf import boxes_from_sdf, gap_between, load_world_boxes

REPO = Path(__file__).resolve().parents[3]
WORLDS = REPO / "src" / "rob_box_sim" / "worlds"
LIDAR_RANGE_MAX = 10.0  # rob_box.xacro, сим-ветка gpu_lidar


@pytest.mark.parametrize("name", sorted(wc.CATALOG))
def test_committed_world_equals_generator(name):
    committed = (WORLDS / f"{name}.sdf").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert committed == wc.CATALOG[name].sdf(), f"перегенерируй: python3 -m rob_box_sim.worlds_catalog {WORLDS}"


@pytest.mark.parametrize("name", sorted(wc.CATALOG))
def test_spawn_point_is_free(name):
    boxes = load_world_boxes(WORLDS / f"{name}.sdf")
    assert clearance(Pose2D(0, 0, 0), boxes) > 0.1


def test_polygon_gaps_are_as_declared():
    boxes = load_world_boxes(WORLDS / "polygon.sdf")
    for x, gap in wc.POLYGON_GAPS:
        assert gap_between(boxes, x) == [pytest.approx(gap)], x


def test_turn90_corridor_width():
    boxes = load_world_boxes(WORLDS / "turn90.sdf")
    # ветка A: поперёк по Y на x=1; ветка B: поперёк по X на y=3
    a = [b for b in boxes if b.sx > b.sy and b.cx - b.sx / 2 <= 1.0 <= b.cx + b.sx / 2]
    a_edges = sorted(b.cy for b in a)
    assert a_edges[1] - a_edges[0] - 0.1 == pytest.approx(wc.TURN90_WIDTH)
    b = [b for b in boxes if b.sy > b.sx and b.cy - b.sy / 2 <= 3.0 <= b.cy + b.sy / 2]
    b_edges = sorted(x.cx for x in b)
    assert b_edges[1] - b_edges[0] - 0.1 == pytest.approx(wc.TURN90_WIDTH)


def test_corridor_ends_out_of_lidar_range_on_drive_segment():
    boxes = load_world_boxes(WORLDS / "corridor.sdf")
    ends = [b for b in boxes if b.sy < 2 and b.sx < 0.2]
    assert len(ends) == 2
    for x in (0.0, 10.0):  # участок езды сценариев
        for e in ends:
            assert abs(e.cx - x) > LIDAR_RANGE_MAX


def test_low_obstacle_is_below_lidar_but_hits_body():
    boxes = load_world_boxes(WORLDS / "low_obstacle.sdf")
    low = [b for b in boxes if b.height < 0.5]
    assert len(low) == 1
    # лидар сима: base_link z = r колеса 0.1143, лидар над base_link 0.4765 (rob_box.xacro)
    lidar_z = 0.1143 + 0.4765
    assert ROBOT_BODY_BOTTOM_Z < low[0].height < lidar_z
    # и стоит на прямой от спавна к цели (5, 0)
    assert abs(low[0].cy) < low[0].sy / 2 and 0 < low[0].cx < 5


def test_sdf_parser_composes_model_link_collision_poses():
    text = """<sdf version="1.8"><world name="w">
      <model name="m"><pose>1 0 0 0 0 1.5707963</pose>
        <link name="l"><pose>2 0 0 0 0 0</pose>
          <collision name="c"><pose>0 0 0.25 0 0 0</pose>
            <geometry><box><size>1 2 0.5</size></box></geometry></collision>
          <collision name="p"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
        </link></model></world></sdf>"""
    (box,) = boxes_from_sdf(text)
    assert (box.cx, box.cy) == pytest.approx((1.0, 2.0), abs=1e-6)
    assert box.yaw == pytest.approx(math.pi / 2, abs=1e-6)
    assert box.height == pytest.approx(0.5)


def test_compose_selects_world_by_env_and_mounts_worlds():
    class _L(yaml.SafeLoader):
        pass

    _L.add_constructor("!override", lambda loader, node: loader.construct_sequence(node))
    compose = yaml.load((REPO / "docker" / "sim" / "compose.yaml").read_text(encoding="utf-8"), Loader=_L)  # noqa: S506
    gz = compose["services"]["gazebo"]
    assert "world:=/sim_worlds/${SIM_WORLD:-polygon}.sdf" in gz["command"]
    assert "../../src/rob_box_sim/worlds:/sim_worlds:ro" in gz["volumes"]
