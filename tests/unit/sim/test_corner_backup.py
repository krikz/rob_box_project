"""Угол/тупик (#4607): впереди ближе порога защиты, сзади свободно — ``backup``, иначе ``spin``.

Порог не новый: ``corner_backup.STOP_M`` равен ``proximity.Thresholds.stop_m``
и ``coverage_plan.GUARD_STOP_M``. В дерево NavigateToPose формула не вшита:
Humble Nav2 1.1.20 / BehaviorTree.CPP v3 не знает узел Script (прогон 18:47Z),
а без него ветка BackUp невыбираема. Recovery — прежний Spin, как до #4785.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from rob_box_sim.corner_backup import BACKUP, SPIN, STOP_M, corner_escape
from rob_box_sim.coverage_plan import GUARD_STOP_M

REPO = Path(__file__).resolve().parents[3]
BT_DIR = REPO / "docker/main/config/nav2/behavior_trees"
TREE = BT_DIR / "navigate_to_pose_w_replanning.xml"
THROUGH = BT_DIR / "navigate_through_poses.xml"
PARAMS = REPO / "docker/main/config/nav2/nav2_params.yaml"


def test_stop_m_is_the_existing_proximity_threshold_not_a_new_corner_constant():
    import sys

    sys.path.insert(0, str(REPO / "src/rob_box_teleop"))
    from rob_box_teleop.proximity import Thresholds

    assert STOP_M == Thresholds().stop_m == GUARD_STOP_M == 0.20


def test_corner_with_wall_ahead_and_clear_behind_backs_up_not_spin():
    assert corner_escape(ahead_m=0.19, behind_m=1.0) == BACKUP
    assert corner_escape(ahead_m=0.0, behind_m=STOP_M) == BACKUP
    assert corner_escape(ahead_m=-0.01, behind_m=2.0) == BACKUP


def test_open_floor_and_rear_dead_end_still_spin():
    assert corner_escape(ahead_m=STOP_M, behind_m=1.0) == SPIN
    assert corner_escape(ahead_m=1.0, behind_m=1.0) == SPIN
    assert corner_escape(ahead_m=0.05, behind_m=0.05) == SPIN
    assert corner_escape(ahead_m=0.05, behind_m=STOP_M - 0.01) == SPIN


def test_missing_range_does_not_invent_a_backup():
    assert corner_escape(ahead_m=float("nan"), behind_m=1.0) == SPIN
    assert corner_escape(ahead_m=0.05, behind_m=float("nan")) == SPIN
    assert corner_escape(ahead_m=None, behind_m=1.0) == SPIN  # type: ignore[arg-type]


def _loaded_bt_paths() -> list[Path]:
    """Деревья, которые bt_navigator реально грузит (default_*_bt_xml)."""
    bt = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))["bt_navigator"][
        "ros__parameters"
    ]
    loaded = []
    for key, path in (
        ("default_nav_to_pose_bt_xml", TREE),
        ("default_nav_through_poses_bt_xml", THROUGH),
    ):
        assert bt[key].endswith(path.name)
        loaded.append(path)
    return loaded


def test_loaded_behavior_trees_have_no_script_node():
    """Humble не знает Script. Узел под другим именем тоже не оставляем."""
    for path in _loaded_bt_paths():
        root = ET.parse(path).getroot()
        tags = {el.tag for el in root.iter()}
        assert "Script" not in tags, path.name
        assert not any(tag.lower() == "script" for tag in tags), path.name
        text = path.read_text(encoding="utf-8")
        assert "<Script" not in text and "bt_script" not in text


def test_navigate_recovery_is_the_pre_4785_spin_not_an_always_backup_stub():
    """Без Script ветка BackUp невыбираема — в RoundRobin прежний Spin, не BackUp."""
    recovery = ET.parse(TREE).getroot().find("BehaviorTree/RecoveryNode")
    assert recovery is not None
    actions = recovery.find("./ReactiveFallback/RoundRobin")
    assert actions is not None and actions.get("name") == "RecoveryActions"
    names = [child.tag for child in list(actions)]
    assert names == ["Sequence", "Spin", "Wait"]
    spin = actions.find("Spin")
    assert spin is not None and spin.get("spin_dist") == "1.57"
    assert spin.get("if") is None
    assert list(spin) == []
    assert actions.find("BackUp") is None
    assert recovery.find(".//BackUp") is None
    assert actions.find("Fallback") is None


def test_backup_stop_m_stays_the_proximity_threshold():
    """Порог формулы не выдуман и не сдвинут. В дерево он не вшит."""
    bt = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))["bt_navigator"][
        "ros__parameters"
    ]
    assert float(bt["backup_stop_m"]) == STOP_M
    assert "nav2_back_up_action_bt_node" in bt["plugin_lib_names"]
    body = "".join(
        el.text or "" for el in ET.parse(TREE).getroot().iter()
    )
    assert "corner_escape" not in body


@pytest.mark.parametrize(
    "ahead,behind,want",
    [
        (0.199, 0.20, BACKUP),
        (0.20, 0.20, SPIN),
        (0.10, float("inf"), BACKUP),
    ],
)
def test_threshold_is_strictly_closer_than_stop_m(ahead, behind, want):
    assert corner_escape(ahead, behind) == want
