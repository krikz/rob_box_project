"""Повторный M1 (#4694): список топиков bag и фиксация условий до цели.

Живой HIL не нужен. Проверяется код scripts/hil/repeat/m1_topics.py и то, что
сценарий вызывает гейт мира как есть и копирует bag на хост до hil off.
"""

from __future__ import annotations

import importlib.util
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
REPEAT = ROOT / "scripts" / "hil" / "repeat"
_spec = importlib.util.spec_from_file_location("hil_m1_topics", REPEAT / "m1_topics.py")
topics = importlib.util.module_from_spec(_spec)
sys.modules["hil_m1_topics"] = topics
_spec.loader.exec_module(topics)

REQUIRED = (
    "/cmd_vel_yaw",
    "/cmd_vel_nav",
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
    "/rosout",
    "/evaluation",
    "/trajectories",
    "/local_plan",
)


def test_repeat_topics_add_yaw_costmap_and_controller_before_mux():
    names = topics.repeat_bag_topics()
    for name in REQUIRED:
        assert name in names, name
    # База прошлого bag (/tmp/hil_m1_stall) не выкинута.
    for name in ("/cmd_vel", "/plan", "/sim/ground_truth/odom", "/local_costmap/costmap_raw"):
        assert name in names
    assert names.count("/cmd_vel_yaw") == 1
    assert topics.CONTROLLER_BEFORE_MUX == "/cmd_vel_nav"
    assert "/cmd_vel_nav" != "/cmd_vel"


def test_record_argv_refuses_list_without_required():
    argv = topics.record_argv("/tmp/bag")
    assert argv[:5] == ["ros2", "bag", "record", "-o", "/tmp/bag"]
    assert "/cmd_vel_yaw" in argv
    with pytest.raises(ValueError, match="cmd_vel_yaw"):
        topics.record_argv("/tmp/bag", topics.BASE_BAG_TOPICS)


def test_start_map_gt_does_not_invent_zero():
    text = topics.format_start_map_gt(None, {"x": 0.0, "y": 1.0})
    assert text == "map none\ngt none\n"
    full = topics.format_start_map_gt(
        {"x": -0.001, "y": 0.002, "yaw": 0.01},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    assert full.startswith("map x=-0.001000 y=0.002000 yaw=0.010000\n")
    assert "gt x=0.000000" in full


def test_parse_tf_and_ground_truth_raw():
    tf = (
        "At time 1.0\n"
        "- Translation: [0.010000, -0.020000, 0.000000]\n"
        "- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]\n"
        "- Rotation: in RPY (radian) [0.000, 0.000, 0.100]\n"
    )
    gt = (
        "pose:\n"
        "  pose:\n"
        "    position:\n"
        "      x: -1.3e-05\n"
        "      y: 6.0e-10\n"
        "      z: 0.0\n"
        "    orientation:\n"
        "      x: 0.0\n"
        "      y: 0.0\n"
        "      z: 0.0\n"
        "      w: 1.0\n"
    )
    parsed_tf = topics.parse_tf2_echo(tf)
    assert parsed_tf["x"] == pytest.approx(0.01)
    assert parsed_tf["yaw"] == pytest.approx(0.1)
    parsed_gt = topics.parse_ground_truth_echo(gt)
    assert parsed_gt["x"] == pytest.approx(-1.3e-05)
    assert parsed_gt["yaw"] == pytest.approx(0.0)
    assert topics.parse_tf2_echo("no frame") is None
    assert topics.parse_ground_truth_echo("position:\n  x: 1\n") is None


def test_write_start_and_assert_ready(tmp_path):
    body = topics.write_start_map_gt(tmp_path, "garbage", "garbage")
    assert body == "map none\ngt none\n"
    assert (tmp_path / "start_map_gt.txt").read_text(encoding="utf-8") == body
    for name in topics.BEFORE_GOAL_FILES:
        if name == "start_map_gt.txt":
            continue
        (tmp_path / name).write_text("x\n", encoding="utf-8")
    topics.assert_ready(tmp_path, "before_goal")
    (tmp_path / "sdf.sha256").write_text("", encoding="utf-8")
    with pytest.raises(FileNotFoundError, match="sdf.sha256"):
        topics.assert_ready(tmp_path, "before_goal")


def test_params_file_names_each_value():
    text = topics.format_params_file(
        [
            ("FollowPath.max_vel_x", "Double value is: 0.15"),
            ("FollowPath.PathAlign.forward_point_distance", "Double value is: 0.2"),
            ("progress_checker.movement_time_allowance", "Double value is: 30.0"),
            ("FollowPath.min_speed_xy", "Double value is: 0.05"),
        ]
    )
    assert "FollowPath.max_vel_x=0.15" in text
    assert "FollowPath.PathAlign.forward_point_distance=0.2" in text
    assert "progress_checker.movement_time_allowance=30.0" in text
    assert "FollowPath.min_speed_xy=0.05" in text
    assert "Double value is" not in text
    assert topics.params_missing_names(text) == []
    # Безымянный ответ без вызова param get — имя missing, ноль не подставляем.
    bare = topics.format_params_file([])
    assert "FollowPath.max_vel_x=missing" in bare
    assert topics.params_missing_names(bare) == list(topics.NAMED_PARAMS)
    assert topics.format_named_param("FollowPath.max_vel_x", "") == "FollowPath.max_vel_x=missing"
    assert math.isclose(topics.MAX_VEL_X, 0.15)
    assert math.isclose(topics.PATHALIGN_FORWARD_M, 0.20)


def test_recovery_hits_name_the_marker_not_invent_a_type():
    log = "Failed to make progress\nClearing global costmap\n"
    assert topics.recovery_hits(log) == ["progress_fail", "clear_global"]
    assert topics.recovery_hits("") == []
    assert "spin" not in topics.recovery_hits(log)


def test_rotate_latch_markers_and_dwb_overlay_do_not_touch_speed():
    log = "IllegalTrajectoryException: Nonrotation command near goal\nNot slowing down near goal\n"
    hits = topics.recovery_hits(log)
    assert hits == ["rotate_nonrotation", "rotate_not_slowing"]
    overlay = topics.dwb_publish_overlay_yaml()
    assert "publish_evaluation: true" in overlay
    assert "publish_trajectories: true" in overlay
    assert "publish_local_plan: true" in overlay
    assert "max_vel_x" not in overlay
    assert "lookahead_time" not in overlay
    assert topics.EVALUATION_TYPE == "dwb_msgs/msg/LocalPlanEvaluation"
    assert topics.TRAJECTORIES_TYPE == "visualization_msgs/msg/MarkerArray"
    assert "/evaluation" in topics.repeat_bag_topics()
    assert "/local_plan" in topics.repeat_bag_topics()


def test_script_uses_world_gate_and_pulls_before_off():
    script = (REPEAT / "run_m1_repeat.sh").read_text(encoding="utf-8")
    assert "HIL_EXPECT_WORLD=depot" in script
    assert "world_check.py" in script or "WORLD_CHECK" in script
    # Артефакты на хост до off: pull вызывается из cleanup раньше hil off.
    pull_at = script.index("pull_before_off || bad=1")
    off_at = script.index('bash "$HIL_CMD" off', pull_at)
    assert pull_at < off_at
    assert "ros2 bag record" in script
    assert "topics_line" in script
    assert "FollowPath.min_speed_xy" in script
    assert "movement_time_allowance" in script
    assert "nav2.log" in script
    assert "recovery.txt" in script
    assert "dwb_publish.yaml" in script
    assert "nav2tune.compose.yaml" in script
    assert "publish_evaluation" in script
    code = "\n".join(line for line in script.splitlines() if not line.lstrip().startswith("#"))
    assert "nav2_params.yaml" not in code
    assert "CAMERAS=0 ./runvideo.sh M1" in script
    assert "/tmp/hil.lock" in script
    assert "NAV2_PARAMS_OVERLAYS" in script
    # Гейт не переписываем: сценарий только вызывает существующий world_check.
    assert "Depot_vid" not in script


@pytest.mark.skipif(subprocess.call(["bash", "-n", str(REPEAT / "run_m1_repeat.sh")]) != 0, reason="bash -n")
def test_shell_syntax_ok():
    subprocess.run(["bash", "-n", str(REPEAT / "run_m1_repeat.sh")], check=True)
