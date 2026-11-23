"""Nav2 1.1.20 isGoalReached ignores failed goal TF → false "Reached the goal" (#4682).

Upstream (commit a0970867, nav2_controller/src/controller_server.cpp isGoalReached):
transformPose return value is discarded; on failure transformed_end_pose stays
default-constructed (0,0,0). Goal checker then compares the robot to the origin.

This module does NOT build controller_server and does NOT run a live route.
It locks: (1) the decision must fail closed on TF error; (2) the Docker patch
must apply to the pinned source and be wired into the nav2 image.
"""

from __future__ import annotations

import math
import subprocess
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "controller_goal_tf_fail.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM_URL = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/src/controller_server.cpp"
)
XY_TOL = 0.25  # general_goal_checker.xy_goal_tolerance in nav2_params.yaml


def _xy_reached(
    robot_xy: tuple[float, float], goal_xy: tuple[float, float], tol: float = XY_TOL
) -> bool:
    dx = robot_xy[0] - goal_xy[0]
    dy = robot_xy[1] - goal_xy[1]
    return dx * dx + dy * dy <= tol * tol


def is_goal_reached_v1_1_20(
    robot_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    transform_ok: bool,
) -> bool:
    """Model upstream isGoalReached: transformPose bool ignored."""
    transformed_xy = (0.0, 0.0)  # default PoseStamped on failure
    if transform_ok:
        transformed_xy = end_xy
    # BUG: no `if not transform_ok: return False`
    return _xy_reached(robot_xy, transformed_xy)


def is_goal_reached_fixed(
    robot_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    transform_ok: bool,
) -> bool:
    """Model patched isGoalReached: TF failure ⇒ not reached."""
    if not transform_ok:
        return False
    return _xy_reached(robot_xy, end_xy)


def test_upstream_model_false_positive_near_origin_on_tf_fail():
    """M1 geometry: goal (-6, 4), robot near origin ≈7.21 m away; TF fail → false success."""
    robot = (0.05, 0.02)
    goal = (-6.0, 4.0)
    dist = math.hypot(robot[0] - goal[0], robot[1] - goal[1])
    assert dist == pytest.approx(7.21, abs=0.05)

    assert not is_goal_reached_v1_1_20(robot, goal, transform_ok=True)
    # RED without fix: default goal (0,0) sits next to the robot
    assert is_goal_reached_v1_1_20(robot, goal, transform_ok=False)


def test_fixed_model_rejects_tf_fail():
    robot = (0.05, 0.02)
    goal = (-6.0, 4.0)
    assert not is_goal_reached_fixed(robot, goal, transform_ok=False)
    assert not is_goal_reached_fixed(robot, goal, transform_ok=True)
    assert is_goal_reached_fixed(goal, goal, transform_ok=True)


def test_patch_file_checks_transform_pose_return():
    text = PATCH.read_text(encoding="utf-8")
    assert "if (!nav_2d_utils::transformPose(" in text
    assert "return false;" in text
    assert "nav2_controller/src/controller_server.cpp" in text
    # Must not weaken goal checker tolerances
    assert "xy_goal_tolerance" not in text
    assert "yaw_goal_tolerance" not in text


def test_dockerfile_builds_pinned_controller_with_patch():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "controller_goal_tf_fail.patch" in df
    assert PINNED_COMMIT in df
    assert "libcontroller_server_core.so" in df
    assert "--packages-select nav2_controller" in df


def test_patch_applies_to_pinned_upstream_controller_server(tmp_path: Path):
    """Fetch pinned controller_server.cpp and prove patch applies (needs network)."""
    src_root = tmp_path / "navigation2"
    target = src_root / "nav2_controller" / "src" / "controller_server.cpp"
    target.parent.mkdir(parents=True)
    try:
        with urllib.request.urlopen(UPSTREAM_URL, timeout=30) as resp:
            body = resp.read()
    except Exception as exc:  # noqa: BLE001 — honest fail if upstream unreachable
        pytest.fail(f"could not fetch pinned Nav2 source: {exc}")
    target.write_bytes(body)

    before = target.read_text(encoding="utf-8")
    assert "nav_2d_utils::transformPose(" in before
    # Unpatched: call is not under `if (!...transformPose`
    unpatched_block = (
        "  nav_2d_utils::transformPose(\n"
        "    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),\n"
        "    end_pose_, transformed_end_pose, tolerance);"
    )
    assert unpatched_block in before

    result = subprocess.run(
        ["patch", "-p1", "--forward", "--batch"],
        cwd=src_root,
        input=PATCH.read_text(encoding="utf-8"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    after = target.read_text(encoding="utf-8")
    assert unpatched_block not in after
    assert (
        "  if (!nav_2d_utils::transformPose(\n"
        "      costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),\n"
        "      end_pose_, transformed_end_pose, tolerance))\n"
        "  {\n"
        "    return false;\n"
        "  }"
    ) in after
