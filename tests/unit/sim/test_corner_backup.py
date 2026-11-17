"""Угол/тупик (#4607): впереди ближе порога защиты, сзади свободно — задний ход, не Spin.

Порог не новый: ``corner_backup.STOP_M`` равен ``proximity.Thresholds.stop_m``
и ``coverage_plan.GUARD_STOP_M``. Дерево NavigateToPose вызывает ту же функцию.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from rob_box_sim.corner_backup import BACKUP, SPIN, STOP_M, bt_script, corner_escape
from rob_box_sim.coverage_plan import GUARD_STOP_M

REPO = Path(__file__).resolve().parents[3]
TREE = REPO / "docker/main/config/nav2/behavior_trees/navigate_to_pose_w_replanning.xml"
PARAMS = REPO / "docker/main/config/nav2/nav2_params.yaml"
CORNER_ESCAPE_SCRIPT = bt_script()


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


def test_navigate_recovery_backs_up_before_spin_when_ahead_is_closer_than_stop_m():
    """Дерево вызывает ту же функцию: впереди < stop_m и сзади не ближе — BackUp, иначе Spin."""
    recovery = ET.parse(TREE).getroot().find("BehaviorTree/RecoveryNode")
    assert recovery is not None
    actions = recovery.find("./ReactiveFallback/RoundRobin")
    assert actions is not None and actions.get("name") == "RecoveryActions"
    names = [child.tag for child in list(actions)]
    assert "Spin" not in names
    gate = actions.find("Fallback")
    assert gate is not None and names.index("Fallback") < names.index("Wait")
    script = gate.find(".//Script")
    assert script is not None and script.get("code") == CORNER_ESCAPE_SCRIPT
    backup = gate.find(".//BackUp")
    assert backup is not None
    assert backup.get("backup_dist") == "0.30"
    assert backup.get("backup_speed") == "0.05"
    assert backup.get("if") == "corner_escape == 'backup'"
    spin = gate.find("Spin")
    assert spin is not None and spin.get("if") == "corner_escape != 'backup'"
    assert list(spin) == []
    assert actions.find("BackUp") is None


def test_script_is_the_function_and_threshold_is_stop_m():
    bt = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))["bt_navigator"][
        "ros__parameters"
    ]
    assert float(bt["backup_stop_m"]) == STOP_M
    assert "nav2_back_up_action_bt_node" in bt["plugin_lib_names"]
    assert "corner_escape" in CORNER_ESCAPE_SCRIPT
    assert "'backup'" in CORNER_ESCAPE_SCRIPT and "'spin'" in CORNER_ESCAPE_SCRIPT
    assert str(STOP_M) not in CORNER_ESCAPE_SCRIPT


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
