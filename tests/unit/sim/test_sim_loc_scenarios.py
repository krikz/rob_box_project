"""Сценарии стенда локализации (#4346): scenarios/localization.yaml читается стендом и годится миру.

Проверяется то, что выполнит стенд: шаги пробы, позы спавна и «похищения» в свободном месте
полигона (робот не спавнится в стену), и что маршрут картирования мира с тегами — тот же,
что у polygon.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from rob_box_sim import loc_bench
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Pose2D, clearance
from rob_box_sim.world_sdf import load_world_boxes

REPO = Path(__file__).resolve().parents[3]
SCEN = REPO / "src" / "rob_box_sim" / "scenarios"
LOC = SCEN / "localization.yaml"
WORLDS = REPO / "src" / "rob_box_sim" / "worlds"
DOC = yaml.safe_load(LOC.read_text(encoding="utf-8"))


@pytest.mark.parametrize("name", sorted(DOC["scenarios"]))
def test_every_scenario_loads_with_valid_probe(name):
    sc = loc_bench.load_scenario(LOC, name)
    assert ("spawn_abs" in sc) != ("spawn_offset" in sc)
    assert sc["event"] in ("restart", "kidnap")
    kinds = [next(iter(s)) for s in sc["probe"]]
    assert ("kidnap" in kinds) == (sc["event"] == "kidnap")


def test_bad_step_fails_loudly(tmp_path):
    bad = dict(DOC, scenarios={"x": {"spawn_abs": [0, 0, 0], "probe": [{"drive": 1}]}})
    path = tmp_path / "loc.yaml"
    path.write_text(yaml.safe_dump(bad), encoding="utf-8")
    with pytest.raises(SystemExit, match="wait"):
        loc_bench.load_scenario(path, "x")


def _free_poses():
    """Абсолютные позы сценариев: спавны spawn_abs и цели похищения."""
    poses = [("spawn_abs:" + n, sc["spawn_abs"]) for n, sc in DOC["scenarios"].items() if "spawn_abs" in sc]
    for name, steps in DOC["probes"].items():
        poses += [(f"kidnap:{name}", s["kidnap"]) for s in steps if "kidnap" in s]
    # Сохранённая поза — дом маршрута картирования (0, 0, π); спавны со сдвигом от неё.
    for n, sc in DOC["scenarios"].items():
        if "spawn_offset" in sc:
            dx, dy, dyaw = sc["spawn_offset"]
            poses.append((f"offset:{n}", [dx, dy, 3.1416 + dyaw]))
    return poses


@pytest.mark.parametrize("world", ["polygon", "polygon_tags"])
@pytest.mark.parametrize("label,pose", _free_poses(), ids=lambda v: v if isinstance(v, str) else None)
def test_spawn_and_kidnap_poses_are_free(world, label, pose):
    boxes = load_world_boxes(WORLDS / f"{world}.sdf")
    gap = clearance(Pose2D(*pose), boxes, ROBOT_FOOTPRINT)
    assert gap > 0.1, f"{label}: до стены {gap:.2f} м"


def test_mapping_route_of_tag_world_is_polygon_route():
    doc = yaml.safe_load((SCEN / "mapping.yaml").read_text(encoding="utf-8"))
    assert doc["worlds"]["polygon_tags"] == doc["worlds"]["polygon"]
    assert doc["worlds"]["polygon"]["goals"][-1]["name"] == "home"
