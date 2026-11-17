"""Сторож сирот FollowPath (#4458): сервис всегда в стеке main, запускает ноду, имена остановки общие с тулами."""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
COMPOSE = REPO / "docker" / "main" / "docker-compose.yaml"
SCRIPT = REPO / "docker" / "main" / "scripts" / "teleop" / "start_followpath_watchdog.sh"
NAVIGATION = REPO / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "tools" / "navigation.py"
NODE = REPO / "src" / "rob_box_teleop" / "rob_box_teleop" / "followpath_watchdog_node.py"


def _service():
    return yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]["followpath-watchdog"]


def test_service_is_always_in_the_stack_and_runs_the_watchdog_script():
    svc = _service()
    assert "profiles" not in svc
    assert svc["restart"] == "unless-stopped" and svc["healthcheck"]["test"]
    assert "./scripts/teleop:/scripts:ro" in svc["volumes"]
    assert svc["command"][-1] == "/scripts/start_followpath_watchdog.sh"
    assert "followpath_watchdog_node" in SCRIPT.read_text(encoding="utf-8")


def test_watchdog_and_navigation_tools_share_one_halt_path():
    assert "rob_box_core.follow_path_halt" in NODE.read_text(encoding="utf-8")
    assert "rob_box_core.follow_path_halt" in NAVIGATION.read_text(encoding="utf-8")


def test_declare_parameter_names_are_unique_in_the_node():
    text = NODE.read_text(encoding="utf-8")
    assert text.count("self.declare_parameter(") == 1  # цикл по _DEFAULTS, имена — ключи словаря


def test_watchdog_subscribes_to_every_parent_action_that_calls_follow_path():
    """#4458: голос/квест/HIL (NavigateToPose) и полоса HIL (NavigateThroughPoses) — одно правило."""
    text = NODE.read_text(encoding="utf-8")
    assert '"navigate_to_pose"' in text and '"/navigate_to_pose/_action/status"' in text
    assert '"navigate_through_poses"' in text and '"/navigate_through_poses/_action/status"' in text
    assert "NAV_STATUS_TOPIC" not in text  # один топик больше не считается «всеми источниками»


def test_watchdog_mounts_nav2_config_and_turns_at_goal_on_its_own_topic():
    """#4694: разворот читает yaw_goal_tolerance из того же файла, что Nav2, и не пишет в /cmd_vel контроллера."""
    svc = _service()
    assert "./config:/config:ro" in svc["volumes"]
    node = NODE.read_text(encoding="utf-8")
    assert "YawAtGoal" in node
    assert "YAW_AT_GOAL_CMD_VEL_TOPIC" in (REPO / "src/rob_box_core/rob_box_core/follow_path_halt.py").read_text(
        encoding="utf-8"
    )
    mux = yaml.safe_load((REPO / "docker/main/config/twist_mux/twist_mux.yaml").read_text(encoding="utf-8"))
    topics = mux["twist_mux"]["ros__parameters"]["topics"]
    yaw = topics["yaw_at_goal"]
    assert yaw["topic"] == "cmd_vel_yaw"
    assert topics["navigation"]["priority"] < yaw["priority"] < topics["voice"]["priority"]
    assert yaw["priority"] < mux["twist_mux"]["ros__parameters"]["locks"]["joystick_lock"]["priority"]
