"""Дерево NavigateToPose не пересчитывает годный путь каждую секунду.

HIL Depot 16.11 (#4682): у стены x≈−3.5 план раз в секунду менял обход
(север / юг), робот метался и не доезжал M4, R3, R4. Плагин IsPathValid
уже подключён. Контракт — дерево Humble
navigate_w_recovery_and_replanning_only_if_path_becomes_invalid:
новый план только если цель обновилась или текущий путь не годен.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
TREE = (
    REPO
    / "docker"
    / "main"
    / "config"
    / "nav2"
    / "behavior_trees"
    / "navigate_to_pose_w_replanning.xml"
)


def _root() -> ET.Element:
    return ET.parse(TREE).getroot()


def test_params_load_this_tree_and_the_path_check_plugin():
    doc = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    bt = doc["bt_navigator"]["ros__parameters"]
    assert bt["default_nav_to_pose_bt_xml"].endswith(
        "navigate_to_pose_w_replanning.xml"
    )
    assert "nav2_is_path_valid_condition_bt_node" in bt["plugin_lib_names"]


def test_compute_path_runs_only_when_goal_changed_or_path_invalid():
    rate = _root().find(".//RateController")
    assert rate is not None
    recovery = rate.find("RecoveryNode")
    assert recovery is not None and recovery.get("name") == "ComputePathToPose"
    fallback = recovery.find("Fallback")
    assert (
        fallback is not None
    ), "годный путь не должен попадать сразу в ComputePathToPose"
    gate = fallback.find("ReactiveSequence")
    assert gate is not None
    inverter = gate.find("Inverter")
    assert inverter is not None and inverter.find("GlobalUpdatedGoal") is not None
    valid = gate.find("IsPathValid")
    assert valid is not None and valid.get("path") == "{path}"
    compute = fallback.find("ComputePathToPose")
    assert compute is not None
    assert compute.get("goal") == "{goal}" and compute.get("path") == "{path}"
    assert recovery.find("ComputePathToPose") is None


def test_stuck_recovery_stays():
    recovery = _root().find("BehaviorTree/RecoveryNode")
    assert recovery is not None and recovery.get("number_of_retries") == "6"
    assert recovery.find(".//FollowPath") is not None
    assert recovery.find(".//Spin") is not None
    # Прогон 18:47Z: Script (BT.CPP v4) Humble не грузит. BackUp без него
    # невыбираем — в RoundRobin снова прямой Spin, как до #4785.
    assert recovery.find(".//Script") is None
    assert recovery.find(".//BackUp") is None
    actions = recovery.find("./ReactiveFallback/RoundRobin")
    assert actions is not None
    assert actions.find("Spin") is not None
    assert actions.find("Fallback") is None
    assert recovery.find(".//Wait") is not None
