#!/usr/bin/env python3
"""Правит КОПИЮ docker/sim/compose.hil.yaml на песочнице для видео: gazebo грузит мир Depot
(mkdepot.py), спавн в (0, 0), кадры камер пишутся в <dir>/out. Репозиторий не трогает.

  patch_compose.py /home/ros2/rob_box_hilvideo
"""
import sys
from pathlib import Path

root = Path(sys.argv[1])
p = root / "docker/sim/compose.hil.yaml"
s = p.read_text(encoding="utf-8")
old = """    volumes:
      - ./hil_run.sh:/hil_run.sh:ro
    command:
      - /ros_scripts/ros_with_namespace.sh
      - /hil_run.sh
      - world:=/sim_worlds/${SIM_WORLD:-polygon}.sdf
"""
new = f"""    volumes:
      - ./hil_run.sh:/hil_run.sh:ro
      - {root}/mkdepot.py:/mkdepot.py:ro
      - {root}/fuel:/fuel:ro
      - {root}/out:/sim_video
    command:
      - /ros_scripts/ros_with_namespace.sh
      - bash
      - -c
      - python3 /mkdepot.py && exec /hil_run.sh world:=/sim_video/world.sdf x:=0.0 y:=0.0 yaw:=0.0
"""
if old not in s:
    sys.exit("compose.hil.yaml изменился: блок gazebo.volumes/command не найден, правь patch_compose.py")
env_old = "      HIL_GUARD_NO_LINK: ${HIL_GUARD_NO_LINK:-0}\n"
env_new = env_old + (
    "      # Depot лежит в своём кадре; спавн (0,0) = точка (-10.5, 0) здания: мир = карта rtabmap\n"
    '      DEPOT_POSE: "10.5 0 0 0 0 0"\n'
    '      DEPOT_TOP: "-5.5,26.5,-9,9"\n'
    '      DEPOT_CHASE: "-3.7,-5.0,7.0,8.5,1,1.35"\n'
)
if env_old not in s:
    sys.exit("compose.hil.yaml изменился: HIL_GUARD_NO_LINK не найден")
p.write_text(s.replace(old, new).replace(env_old, env_new), encoding="utf-8")
print("patched", p)
