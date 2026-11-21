"""BackUp достижим в каждом внешнем recovery-цикле NavigateToPose.

RoundRobin (Humble nav2_behavior_tree) на первом SUCCESS делает haltChildren
и возвращает SUCCESS; RecoveryNode зовёт haltChild(1), а RoundRobin::halt
сбрасывает индекс в 0. Дети после первого недостижимы: первый ребёнок —
единственный, кто при успехе реально выполняется. Раунд 3 Gate 2 (21.11,
дерево a898292, стенд Depot, 2×10 целей): «Running backup» = 0,
«Running spin 1.57» = 0; единственный global-clear в R1-B 04:33:24.811Z =
единственный внешний цикл за 200 с. Поэтому отъезд стоит в первом ребёнке,
вместе с очисткой карт. BackUp — штатный узел BehaviorTree.CPP v3 (плагин
``backup`` уже в ``behavior_server``), без Script и без ``if``: Humble
BT.CPP v3 узел Script не знает (прогон 18:47Z, #4797).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
TREE = (
    REPO
    / "docker"
    / "main"
    / "config"
    / "nav2"
    / "behavior_trees"
    / "navigate_to_pose_w_replanning.xml"
)
PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

# Порт по умолчанию BackUpAction (Humble back_up_action.hpp). Меньше stop_m
# guard (0.20): отъезд сам не заходит за порог стопа в коридоре хода.
BACKUP_DIST_M = 0.15
BACKUP_SPEED_M_S = 0.025


def _recovery_actions() -> ET.Element:
    root = ET.parse(TREE).getroot()
    actions = root.find("BehaviorTree/RecoveryNode/ReactiveFallback/RoundRobin")
    assert actions is not None and actions.get("name") == "RecoveryActions"
    return actions


def test_recovery_round_robin_backs_up_before_spin():
    """Отъезд — в первом ребёнке RoundRobin, до Spin, без Script и if.

    Первый ребёнок при успехе — единственный, кто реально выполняется:
    SUCCESS → haltChild(1) → RoundRobin::halt сбрасывает индекс в 0.
    """
    actions = _recovery_actions()
    assert [child.tag for child in actions] == [
        "Sequence",
        "Spin",
        "Wait",
    ]
    first = actions.find("Sequence")
    assert first.get("name") == "ClearThenBackUp"
    assert [child.tag for child in first] == [
        "ClearEntireCostmap",
        "ClearEntireCostmap",
        "BackUp",
    ]
    local, global_map = first.findall("ClearEntireCostmap")
    assert local.get("service_name") == "local_costmap/clear_entirely_local_costmap"
    assert (
        global_map.get("service_name") == "global_costmap/clear_entirely_global_costmap"
    )

    backup = first.find("BackUp")
    assert backup.get("backup_dist") == f"{BACKUP_DIST_M:.2f}"
    assert backup.get("backup_speed") == f"{BACKUP_SPEED_M_S:.3f}"
    # Условие выбора — атрибут if или узел Script; Humble v3 Script не грузит.
    # Отъезд безусловный и стоит в первом ребёнке: сзади lethal своя проверка
    # столкновения роняет Sequence, RoundRobin идёт к Spin.
    assert backup.get("if") is None
    assert actions.find(".//Script") is None

    spin = actions.find("Spin")
    assert spin.get("spin_dist") == "1.57"
    assert spin.get("is_recovery") in (None, "true")
    wait = actions.find("Wait")
    assert wait.get("wait_duration") == "5"


def test_backup_plugin_is_loaded_and_checks_collision():
    """Плагин backup объявлен, а проверка столкновения — на уровне сервера.

    Humble BackUp читает simulate_ahead_time без префикса плагина
    (drive_on_heading.hpp). Ключ backup.simulate_ahead_time Nav2 не читает.
    """
    doc = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    bt = doc["bt_navigator"]["ros__parameters"]
    assert "nav2_back_up_action_bt_node" in bt["plugin_lib_names"]

    behavior = doc["behavior_server"]["ros__parameters"]
    assert "backup" in behavior["behavior_plugins"]
    assert behavior["backup"]["plugin"] == "nav2_behaviors/BackUp"
    assert behavior["simulate_ahead_time"] == 2.0
    assert "simulate_ahead_time" not in behavior["backup"]
    # Spin/BackUp ищут позу в той же costmap, что и проверка столкновения.
    assert behavior["costmap_topic"] == "local_costmap/costmap_raw"
    assert behavior["global_frame"] == "odom"
