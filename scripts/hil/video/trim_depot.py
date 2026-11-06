#!/usr/bin/env python3
"""Depot без крыши, вентиляторов и окон (сверху виден пол), без collision и без прочих звеньев.

Копия Depot_vid в кэше песочницы; мир Depot (Open Robotics, Fuel, CC-BY 4.0) не правится.
Меши и текстуры — жёсткие ссылки (cp -al): симлинки на хост внутри контейнера не видны.

  trim_depot.py /home/ros2/rob_box_hilvideo/fuel
"""
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

fuel = Path(sys.argv[1])
src, dst = fuel / "Depot", fuel / "Depot_vid"
shutil.rmtree(dst, ignore_errors=True)
dst.mkdir(parents=True)
for d in ("meshes", "materials"):
    subprocess.run(["cp", "-al", str(src / d), str(dst / d)], check=True)
shutil.copy(src / "model.config", dst)
tree = ET.parse(src / "model.sdf")
model = tree.getroot().find("model")
main = model.find("link[@name='main']")
drop = {"ROOF_visual", "FANS_visual", "WINDOW_visual", "FanBack_visual"}
for v in list(main.findall("visual")):
    if v.get("name") in drop:
        main.remove(v)
for c in list(main.findall("collision")):
    main.remove(c)
for el in list(model):
    if el.tag not in ("static", "link") or (el.tag == "link" and el.get("name") != "main"):
        model.remove(el)
model.find("static").text = "true"
tree.write(dst / "model.sdf")
print("visuals:", [v.get("name") for v in main.findall("visual")])
