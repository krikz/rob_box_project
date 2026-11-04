"""SDF-мир из коробок и обратно.

``render_world`` пишет мир Gazebo Fortress: физика 1 мс, системы Physics,
UserCommands, SceneBroadcaster, Sensors (ogre2 — нужен gpu_lidar), Imu, солнце,
пол и одна статическая модель ``<world>_walls`` со звеном на каждую коробку.

``load_world_boxes`` читает коробки из любого SDF-мира (модели, звенья,
коллизии ``<box>``; поворот только по yaw) — стенд метрик считает по ним
зазор до препятствий по истинной позе робота.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, Sequence

from rob_box_sim.geometry2d import Box, Pose2D

WALL_COLOR = "0.7 0.7 0.7 1"
LOW_COLOR = "0.8 0.3 0.1 1"  # препятствия ниже лидара — оранжевые, чтобы видеть глазами


def _f(v: float, digits: int = 3) -> str:
    """Миллиметры для размеров; для угла — 6 знаков (на 20 м от начала 1e-3 рад — уже 2 см)."""
    s = f"{v:.{digits}f}"
    return s[1:] if s.startswith("-") and float(s) == 0 else s


def _box_link(box: Box) -> str:
    size = f"{_f(box.sx)} {_f(box.sy)} {_f(box.height)}"
    color = WALL_COLOR if box.height >= 0.8 else LOW_COLOR
    return (
        f'      <link name="{box.name}">\n'
        f"        <pose>{_f(box.cx)} {_f(box.cy)} {_f(box.height / 2)} 0 0 {_f(box.yaw, 6)}</pose>\n"
        f'        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>\n'
        f'        <visual name="visual">\n'
        f"          <geometry><box><size>{size}</size></box></geometry>\n"
        f"          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>\n"
        f"        </visual>\n"
        f"      </link>\n"
    )


def render_world(name: str, boxes: Sequence[Box], description: str) -> str:
    """Текст SDF-мира. ``description`` уходит в шапку-комментарий (без ``--``)."""
    names = [b.name for b in boxes]
    if len(set(names)) != len(names):
        raise ValueError(f"имена коробок в мире {name} повторяются")
    if "--" in description:
        raise ValueError("в XML-комментарии нельзя «--»")
    links = "".join(_box_link(b) for b in boxes)
    return f"""<?xml version="1.0"?>
<!--
{description.rstrip()}

  Файл сгенерирован: python3 -m rob_box_sim.worlds_catalog (или grid_to_sdf). Руками не править.
  Системы мира: Sensors (ogre2) нужен gpu_lidar, Imu — IMU-сенсору робота.
-->
<sdf version="1.8">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="{name}_walls">
      <static>true</static>
{links}    </model>
  </world>
</sdf>
"""


def _pose_of(elem: ET.Element) -> tuple[Pose2D, float]:
    """(поза в плане, z) из ``<pose>x y z roll pitch yaw</pose>``; крен/тангаж должны быть 0."""
    p = elem.find("pose")
    if p is None or not (p.text or "").strip():
        return Pose2D(0.0, 0.0, 0.0), 0.0
    vals = [float(v) for v in p.text.split()]
    if len(vals) != 6:
        raise ValueError(f"pose: ждал 6 чисел, получил {p.text!r}")
    if abs(vals[3]) > 1e-6 or abs(vals[4]) > 1e-6:
        raise ValueError(f"pose с креном/тангажом не поддерживается: {p.text!r}")
    return Pose2D(vals[0], vals[1], vals[5]), vals[2]


def boxes_from_sdf(text: str) -> list[Box]:
    """Все коллизии-коробки статических и нестатических моделей мира, в мировой системе."""
    root = ET.fromstring(text)
    world = root.find("world")
    if world is None:
        raise ValueError("в SDF нет <world>")
    out: list[Box] = []
    for model in world.findall("model"):
        m_pose, m_z = _pose_of(model)
        for link in model.findall("link"):
            l_pose, l_z = _pose_of(link)
            for col in link.findall("collision"):
                size = col.find("geometry/box/size")
                if size is None:
                    continue
                c_pose, c_z = _pose_of(col)
                sx, sy, sz = (float(v) for v in size.text.split())
                p = m_pose.compose(l_pose).compose(c_pose)
                zc = m_z + l_z + c_z
                top = zc + sz / 2
                name = f"{model.get('name')}::{link.get('name')}::{col.get('name')}"
                out.append(Box(name, p.x, p.y, sx, sy, top, p.yaw))
    return out


def load_world_boxes(path: str | Path) -> list[Box]:
    return boxes_from_sdf(Path(path).read_text(encoding="utf-8"))


def gap_between(boxes: Iterable[Box], x: float, eps: float = 1e-6) -> list[float]:
    """Ширины просветов по Y в перегородке x=const (для коробок без поворота).

    Нужна тестам и сценариям: «проход 0.9 м» должен быть 0.9 м и в SDF.
    """
    spans = sorted(
        (b.cy - b.sy / 2, b.cy + b.sy / 2)
        for b in boxes
        if abs(math.sin(b.yaw)) < eps and b.cx - b.sx / 2 - eps <= x <= b.cx + b.sx / 2 + eps
    )
    gaps = []
    for (_, hi), (lo, _) in zip(spans, spans[1:]):
        if lo - hi > eps:
            gaps.append(round(lo - hi, 6))
    return gaps
