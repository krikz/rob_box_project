"""Видео из Gazebo: мир + камеры-сенсоры, которые сами пишут кадры на диск.

Запись — docker/sim/record_video.sh. Камеры — обычные сенсоры Gazebo Fortress
(``type="camera"``, рендер ogre2 той же системой Sensors, что gpu_lidar, через
EGL на /dev/dri). Кадры пишет сам сенсор (``<save enabled="true">``) в каталог
``<frames>/<камера>/``, по одному на каждый тик ``update_rate`` в sim time:
видео — картинка самого симулятора, без схем.

Что добавляется в мир (копия, исходный worlds/*.sdf не трогается):
  * ``video_top`` — вид сверху на область «старт + цели» (картинка как карта:
    +X вправо, +Y вверх);
  * ``video_chase`` — ракурс сзади-слева-сверху на ту же область;
  * зелёные метки целей сценария — только visual, без collision: физика,
    лидар (метка ниже луча) и метрики стенда их не видят.

Модели камер стоят с нулевыми креном и тангажом (наклон — у позы сенсора), а
коллизий у них нет: ``world_sdf.load_world_boxes`` читает мир с камерами как
исходный.

  python3 -m rob_box_sim.video_overlay --world /sim_worlds/polygon.sdf \\
      --name polygon --scenarios scenarios/video.yaml \\
      --out /sim_video/world.sdf --frames /sim_video/frames
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import yaml

from rob_box_sim.geometry2d import Pose2D

ASPECT = 16 / 9
DEFAULT_WIDTH = 1280
DEFAULT_RATE_HZ = 10.0
ROI_MARGIN_M = 1.2
TOP_FOV = 1.0          # рад по горизонтали
CHASE_FOV = 1.2
CHASE_HEIGHT_M = 3.0
CHASE_BACK_M = 1.2     # насколько камера отнесена от области назад (−X) и вправо (−Y)
GOAL_MARKER_R = 0.25
GOAL_COLOR = "0.1 0.8 0.2 1"


@dataclass(frozen=True)
class Roi:
    """Область кадра в плане: прямоугольник по осям мира."""

    x0: float
    x1: float
    y0: float
    y1: float

    @property
    def cx(self) -> float:
        return (self.x0 + self.x1) / 2

    @property
    def cy(self) -> float:
        return (self.y0 + self.y1) / 2


@dataclass(frozen=True)
class Camera:
    name: str
    x: float
    y: float
    z: float
    pitch: float   # > 0 — смотрит вниз (ось X камеры уходит к −Z)
    yaw: float
    hfov: float
    width: int = DEFAULT_WIDTH
    rate_hz: float = DEFAULT_RATE_HZ

    @property
    def height(self) -> int:
        return int(round(self.width / ASPECT / 2)) * 2  # чётная — для yuv420p


def roi_for(points: Sequence[Pose2D], margin: float = ROI_MARGIN_M) -> Roi:
    if not points:
        raise ValueError("нет точек для области кадра")
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return Roi(min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)


def top_camera(roi: Roi, hfov: float = TOP_FOV, **kw) -> Camera:
    """Камера над центром области, смотрит вниз; ширина кадра — вдоль X мира.

    pitch = π/2 опускает ось взгляда вниз, yaw = π/2 разворачивает кадр так, что
    +X мира — вправо, +Y — вверх (как на карте).
    """
    tan_h = math.tan(hfov / 2)
    tan_v = tan_h / ASPECT
    z = max((roi.x1 - roi.x0) / 2 / tan_h, (roi.y1 - roi.y0) / 2 / tan_v)
    return Camera("video_top", roi.cx, roi.cy, z, math.pi / 2, math.pi / 2, hfov, **kw)


def chase_camera(roi: Roi, hfov: float = CHASE_FOV, height: float = CHASE_HEIGHT_M,
                 back: float = CHASE_BACK_M, **kw) -> Camera:
    """Камера сзади-справа-сверху от области (−X, −Y), взгляд — в центр области."""
    x, y = roi.x0 - back, roi.y0 - back
    dx, dy = roi.cx - x, roi.cy - y
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(height, math.hypot(dx, dy))
    return Camera("video_chase", x, y, height, pitch, yaw, hfov, **kw)


def _f(v: float) -> str:
    s = f"{v:.4f}"
    return s[1:] if s.startswith("-") and float(s) == 0 else s


def camera_model(cam: Camera, frames_dir: str) -> str:
    far = cam.z * 3 + 20
    return f"""
    <model name="{cam.name}">
      <static>true</static>
      <pose>{_f(cam.x)} {_f(cam.y)} {_f(cam.z)} 0 0 0</pose>
      <link name="link">
        <sensor name="{cam.name}" type="camera">
          <pose>0 0 0 0 {_f(cam.pitch)} {_f(cam.yaw)}</pose>
          <always_on>true</always_on>
          <update_rate>{cam.rate_hz:g}</update_rate>
          <topic>/sim/video/{cam.name}</topic>
          <camera>
            <horizontal_fov>{_f(cam.hfov)}</horizontal_fov>
            <image><width>{cam.width}</width><height>{cam.height}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>{far:.1f}</far></clip>
            <save enabled="true"><path>{frames_dir.rstrip('/')}/{cam.name}</path></save>
          </camera>
        </sensor>
      </link>
    </model>
"""


def goal_marker(name: str, goal: Pose2D) -> str:
    """Плоский диск на полу + стрелка курса; без collision (физика и лидар его не видят)."""
    ax = goal.x + math.cos(goal.yaw) * GOAL_MARKER_R
    ay = goal.y + math.sin(goal.yaw) * GOAL_MARKER_R
    mat = f"<material><ambient>{GOAL_COLOR}</ambient><diffuse>{GOAL_COLOR}</diffuse></material>"
    return f"""
    <model name="video_goal_{name}">
      <static>true</static>
      <link name="disc">
        <pose>{_f(goal.x)} {_f(goal.y)} 0.003 0 0 0</pose>
        <visual name="visual">
          <geometry><cylinder><radius>{GOAL_MARKER_R}</radius><length>0.004</length></cylinder></geometry>
          {mat}
        </visual>
      </link>
      <link name="heading">
        <pose>{_f(ax)} {_f(ay)} 0.006 0 0 {_f(goal.yaw)}</pose>
        <visual name="visual">
          <geometry><box><size>0.2 0.05 0.004</size></box></geometry>
          {mat}
        </visual>
      </link>
    </model>
"""


def overlay(world_text: str, cameras: Sequence[Camera], goals: Sequence[tuple[str, Pose2D]],
            frames_dir: str) -> str:
    """Мир с камерами и метками целей, вставленными перед последним ``</world>``."""
    idx = world_text.rfind("</world>")
    if idx < 0:
        raise ValueError("в SDF нет </world>")
    if "ignition::gazebo::systems::Sensors" not in world_text:
        raise ValueError("в мире нет системы Sensors: камере нечем рендерить")
    extra = "".join(camera_model(c, frames_dir) for c in cameras)
    extra += "".join(goal_marker(n, g) for n, g in goals)
    return world_text[:idx] + extra.lstrip("\n") + "  " + world_text[idx:].lstrip()


def scenario_goals(scenarios: Path, name: str) -> list[tuple[str, Pose2D]]:
    doc = yaml.safe_load(scenarios.read_text(encoding="utf-8"))
    if name not in doc.get("worlds", {}):
        raise SystemExit(f"мира {name!r} нет в {scenarios}")
    goals = doc["worlds"][name].get("goals") or []
    return [(str(g["name"]), Pose2D(float(g["x"]), float(g["y"]), float(g.get("yaw", 0.0)))) for g in goals]


def build(world_text: str, goals: Sequence[tuple[str, Pose2D]], frames_dir: str,
          width: int = DEFAULT_WIDTH, rate_hz: float = DEFAULT_RATE_HZ) -> str:
    roi = roi_for([Pose2D(0.0, 0.0, 0.0), *(g for _, g in goals)])  # спавн во всех мирах — (0, 0)
    cams = [top_camera(roi, width=width, rate_hz=rate_hz), chase_camera(roi, width=width, rate_hz=rate_hz)]
    return overlay(world_text, cams, goals, frames_dir)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Мир сима с камерами для видео")
    ap.add_argument("--world", type=Path, required=True, help="исходный SDF-мир")
    ap.add_argument("--name", required=True, help="имя мира в сценариях (polygon, turn90 ...)")
    ap.add_argument("--scenarios", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--frames", default="/sim_video/frames", help="куда сенсоры пишут кадры (в контейнере)")
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="кадров в секунду sim time")
    args = ap.parse_args(argv)
    if args.name.startswith("local"):
        raise SystemExit("миры из карты квартиры (local/) не снимаем: приватно")
    text = build(args.world.read_text(encoding="utf-8"), scenario_goals(args.scenarios, args.name),
                 args.frames, args.width, args.rate)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")
    print(f"[video] {args.out}: камеры video_top, video_chase → {args.frames}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
