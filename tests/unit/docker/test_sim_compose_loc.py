"""Guard: стенд локализации в docker/sim (#4346, вариант D) — камера, AprilTag, база на хосте.

* apriltag в симе — тот же узел и тот же конфиг, что в контейнере oak-d робота, и пишет
  в топик, который слушает боевой rtabmap (``tag_topic:=/detections``);
* камера и теги включаются только флагами (SIM_CAMERA, профиль tags), по умолчанию сим прежний;
* compose.loc.yaml меняет только место базы и добавляет стенд locbench;
* на роботе сим-переключателей нет.

Docker не нужен: проверяется то, что запустит docker compose.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from test_sim_compose import MAIN_COMPOSE, SIM, SIM_COMPOSE, _env, _interpolate, _load

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_sim"))
VISION = REPO / "docker" / "vision"
LOC = _load(SIM / "compose.loc.yaml")["services"]


def _launch_remaps(path: Path) -> dict[str, str]:
    return dict(re.findall(r"\('([\w/]+)',\s*'([\w/]+)'\)", path.read_text(encoding="utf-8")))


def _ros_args(cmd: list[str]) -> tuple[dict[str, str], dict[str, str], list[str]]:
    remaps, params, files = {}, {}, []
    for flag, value in zip(cmd, cmd[1:]):
        if flag == "-r":
            k, v = value.split(":=", 1)
            remaps[k] = v
        elif flag == "-p":
            k, v = value.split(":=", 1)
            params[k] = v
        elif flag == "--params-file":
            files.append(value)
    return remaps, params, files


def test_apriltag_is_robot_node_with_robot_config():
    svc = SIM_COMPOSE["apriltag"]
    assert svc["profiles"] == ["tags"]
    cmd = svc["command"]
    assert cmd[:5] == ["/ros_scripts/ros_with_namespace.sh", "ros2", "run", "apriltag_ros", "apriltag_node"]
    remaps, params, files = _ros_args(cmd)
    assert remaps["__node"] == "apriltag"  # ключ apriltag: в apriltag_config.yaml
    assert files == ["/vision_config/apriltag/apriltag_config.yaml"]
    assert "../vision/config/apriltag:/vision_config/apriltag:ro" in svc["volumes"]
    assert (VISION / "config" / "apriltag" / "apriltag_config.yaml").is_file()
    # те же входы и выход, что у apriltag в контейнере oak-d робота
    robot = _launch_remaps(VISION / "config" / "oak-d" / "launch" / "oakd_with_apriltag.launch.py")
    for name in ("image_rect", "camera_info", "detections"):
        assert remaps[name] == robot[name], name
    # и это тот топик, что слушает боевой rtabmap
    assert "tag_topic:=/detections" in MAIN_COMPOSE["rtabmap"]["command"]
    # отличия сима — только эти аргументы
    assert set(params) == {"image_transport", "detector.decimate", "size", "use_sim_time"}
    assert params["image_transport"] == "raw"
    assert _interpolate(params["size"], {}) == "0.119"


def test_camera_and_spawn_are_flags_with_old_defaults():
    cmd = [_interpolate(a, {}) for a in SIM_COMPOSE["gazebo"]["command"]]
    assert "camera:=false" in cmd
    assert {"x:=0.0", "y:=0.0", "yaw:=0.0"} <= set(cmd)
    cmd = [_interpolate(a, {"SIM_CAMERA": "true", "SIM_SPAWN_X": "0.5"}) for a in SIM_COMPOSE["gazebo"]["command"]]
    assert "camera:=true" in cmd and "x:=0.5" in cmd
    # URDF строит robot-state-publisher: флаг камеры должен дойти и до него
    assert _interpolate(_env(SIM_COMPOSE["robot-state-publisher"])["SIM_CAMERA"], {}) == "false"


def test_tag_variance_flag_only_in_sim():
    raw = _env(SIM_COMPOSE["rtabmap"])["RTABMAP_SIM_TAG_LINEAR_VARIANCE"]
    assert _interpolate(raw, {}) == ""  # пусто = как на роботе
    assert _interpolate(raw, {"SIM_TAG_LINEAR_VARIANCE": "0.01"}) == "0.01"
    for name, svc in MAIN_COMPOSE.items():
        assert "RTABMAP_SIM_TAG_LINEAR_VARIANCE" not in _env(svc), name
        assert "SIM_CAMERA" not in _env(svc), name


def test_loc_overlay_moves_only_maps_and_adds_locbench():
    assert set(LOC) == {"rtabmap", "nav2", "locbench", "tagreloc"}
    for name in ("rtabmap", "nav2"):
        assert set(LOC[name]) <= {"volumes", "stop_grace_period"}, name
        (vol,) = LOC[name]["volumes"]
        assert vol.startswith("${SIM_MAPS_DIR:?") and vol.split("}:")[1].startswith("/maps"), vol
    bench = LOC["locbench"]
    assert bench["extends"] == {"file": "compose.yaml", "service": "bench"}
    assert bench["profiles"] == ["bench"]
    assert bench["command"][1:4] == ["python3", "-m", "rob_box_sim.loc_bench"]


def test_loc_runner_is_lf_and_uses_only_its_project():
    text = (SIM / "run_loc_bench.sh").read_bytes()
    assert b"\r\n" not in text
    src = text.decode("utf-8")
    assert 'project="${SIM_PROJECT:-rob_box_sim_loc}"' in src
    assert 'SIM_PREFIX="${SIM_PREFIX:-rob-box-sim-loc}"' in src
    # гасит только свой compose-проект
    assert "docker rm" not in src and "docker kill" not in src


def test_tagreloc_prototype_is_opt_in_and_uses_world_tag_faces():
    """Вариант B — только профиль reloc; позы тегов = лицевые грани тегов мира polygon_tags."""
    import math

    from rob_box_sim.apriltag_world import POLYGON_TAGS, TAG_THICK_M  # noqa: E402

    svc = LOC["tagreloc"]
    assert svc["profiles"] == ["reloc"]
    cmd = svc["command"]
    assert cmd[1:4] == ["python3", "-m", "rob_box_perception.tag_relocalization_node"]
    params = dict(a.split(":=", 1) for a in cmd if ":=" in a)
    assert params["enabled"] == "true"
    flat = [float(v) for v in params["tags"].strip("[]").split(",")]
    rows = [flat[i:i + 5] for i in range(0, len(flat), 5)]
    assert [int(r[0]) for r in rows] == [t.tag_id for t in POLYGON_TAGS]
    for row, tag in zip(rows, POLYGON_TAGS):
        face = (tag.x + math.cos(tag.yaw) * TAG_THICK_M / 2, tag.y + math.sin(tag.yaw) * TAG_THICK_M / 2)
        assert row[1:3] == [round(face[0], 3), round(face[1], 3)], tag.name
        assert row[3] == tag.z and abs(row[4] - tag.yaw) < 1e-5
