"""Мир Depot для HIL-видео: заголовок polygon.sdf (физика, Sensors, ground_plane) + Depot_vid + камеры.
Параметры камер — переменными окружения DEPOT_TOP="x0,x1,y0,y1", DEPOT_CHASE="cx,cy,z,tx,ty,fov"."""

import math
import os
from pathlib import Path

from rob_box_sim import video_overlay as v

src = Path("/sim_worlds/polygon.sdf").read_text(encoding="utf-8")
cut = src.index('<model name="polygon_walls">')
head = src[:cut]
pose = os.environ.get("DEPOT_POSE", "0 0 0 0 0 0")
inc = f"""    <include>
      <name>depot</name>
      <pose>{pose}</pose>
      <uri>file:///fuel/Depot_vid</uri>
    </include>

"""
x0, x1, y0, y1 = map(float, os.environ.get("DEPOT_TOP", "-16,16,-9,9").split(","))
top = v.top_camera(v.Roi(x0, x1, y0, y1), hfov=1.0, width=1280, rate_hz=10)
cx, cy, z, tx, ty, fov = map(float, os.environ.get("DEPOT_CHASE", "-18,-12,12,0,0,1.0").split(","))
yaw = math.atan2(ty - cy, tx - cx)
pitch = math.atan2(z, math.hypot(tx - cx, ty - cy))
chase = v.Camera("video_chase", cx, cy, z, pitch, yaw, fov, width=1280, rate_hz=10)
extra = "".join(v.camera_model(c, "/sim_video/frames") for c in (top, chase))
Path("/sim_video/world.sdf").write_text(head + inc + extra + "  </world>\n</sdf>\n", encoding="utf-8")
print("[depot] world ok; top z=%.1f" % top.z)
