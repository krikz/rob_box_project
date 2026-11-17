"""Дерево NavigateToPose вставляет разворот до свободного курса перед FollowPath (#4475).

Угол считает ``turn_in_place.free_spin`` (pytest рядом). Узел ``WouldSpinOnStall``
только вызывает его и пишет угол в доску. Нет угла — доска получает 0: Humble Spin
на 0 рад сразу успешен и колёс не трогает. Фиксированные 1.57 рад в эту ветку не ставятся.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
TREE = REPO / "docker" / "main" / "config" / "nav2" / "behavior_trees" / "navigate_to_pose_w_replanning.xml"
PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
DOCKERFILE = REPO / "docker" / "main" / "nav2" / "Dockerfile"
PLUGIN = REPO / "docker" / "main" / "nav2" / "would_spin_on_stall.cpp"
BRIDGE = REPO / "docker" / "main" / "nav2" / "turn_in_place_py.hpp"


def _root() -> ET.Element:
    return ET.parse(TREE).getroot()


def test_follow_path_is_preceded_by_a_computed_spin():
    recovery = _root().find("BehaviorTree/RecoveryNode")
    follow = recovery.find("PipelineSequence/RecoveryNode[@name='FollowPath']")
    sequence = follow.find("Sequence[@name='TurnThenFollow']")
    assert [child.tag for child in sequence] == ["Fallback", "Spin", "FollowPath"]
    stall = sequence.find("Fallback/WouldSpinOnStall")
    assert stall.get("path") == "{path}"
    assert stall.get("turn_spin_dist") == "{turn_spin_dist}"
    assert sequence.find("Spin").get("spin_dist") == "{turn_spin_dist}"
    # Нет угла — 0, а не фиксированные 1.57 рад recovery.
    blackboard = sequence.find("Fallback/SetBlackboard")
    assert blackboard.get("output_key") == "turn_spin_dist"
    assert blackboard.get("value") == "0.0"
    assert sequence.find(".//Spin").get("spin_dist") != "1.57"


def test_recovery_spin_stays_a_fixed_recovery():
    """Отказ FollowPath по-прежнему крутит recovery 1.57, это не разворот до курса."""
    recovery = _root().find("BehaviorTree/RecoveryNode")
    spin = recovery.find("ReactiveFallback/RoundRobin/Fallback/Spin")
    assert spin.get("spin_dist") == "1.57"
    assert spin.get("if") == "corner_escape != 'backup'"


def test_plugin_is_loaded_and_calls_free_spin():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    libs = params["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    assert "rob_box_would_spin_on_stall_bt_node" in libs
    text = PLUGIN.read_text(encoding="utf-8")
    assert "free_heading_spin_rad" in text
    assert 'registerNodeType<rob_box::WouldSpinOnStall>("WouldSpinOnStall")' in text
    bridge = BRIDGE.read_text(encoding="utf-8")
    assert "from rob_box_teleop.turn_in_place import free_spin" in bridge
    assert "body_from_nav2_params" in bridge
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")
    assert "librob_box_would_spin_on_stall_bt_node.so" in dockerfile
    assert "nav_msgs__rosidl_typesupport_cpp" in dockerfile
    assert "-lpython3.10" in dockerfile
