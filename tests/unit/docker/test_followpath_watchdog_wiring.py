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
