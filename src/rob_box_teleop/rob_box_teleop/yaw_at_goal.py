"""Разворот на месте у цели, когда xy уже внутри допуска, а yaw нет (#4694).

DWB ``RotateToGoal`` обязан крутить на месте внутри ``xy_goal_tolerance``. На Depot 17.11
(M2, R3) он этого не сделал: 180 с yaw болтался в 14–57° при err 0.23 м, progress checker
(только xy, 0.08 м за 30 с) сдавался, recovery Spin крутил фиксированные 90° не к цели,
робот досиживал 200 с. Допуск yaw не ослабляем: берём ``yaw_goal_tolerance`` goal checker'а
из того же ``nav2_params.yaml``.

Решение — чистая функция. Нода подаёт сюда позу и конец живого ``/plan`` и публикует
решённый ``Twist`` в ``/cmd_vel_yaw`` (twist_mux priority 15, выше Nav2) только пока
разворот идёт. ``None`` и ``done`` нода не льёт нулями: timeout 0.2 с должен истечь.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional

# Как SimpleGoalChecker (Humble): stateful, цель принята только если |dyaw| ≤ допуск.
CHECKER_PLUGIN = "nav2_controller::SimpleGoalChecker"
# DWB: max_vel_theta боевого FollowPath. Не выдумываем скорость — читаем из того же файла.
PLANNER_PLUGIN = "dwb_core::DWBLocalPlanner"

#: Поза: x, y, yaw (рад, любой обёртки).
Pose = tuple


def shortest_yaw(current: float, goal: float) -> float:
    """Кратчайший поворот current → goal, рад, в (−π, π]."""
    return math.atan2(math.sin(goal - current), math.cos(goal - current))


def yaw_outside(current: float, goal: float, tolerance: float) -> bool:
    """Курс вне допуска goal checker'а. Допуск ≤ 0 — вне всегда (дырявый допуск не «уже на курсе»)."""
    if tolerance <= 0.0:
        return True
    return abs(shortest_yaw(current, goal)) > tolerance


def xy_inside(pose: Optional[Pose], goal: Pose, xy_tolerance_m: float) -> bool:
    """Поза внутри xy-допуска цели. Нет позы или допуск ≤ 0 — нет."""
    if pose is None or xy_tolerance_m <= 0.0:
        return False
    dx = float(pose[0]) - float(goal[0])
    dy = float(pose[1]) - float(goal[1])
    return dx * dx + dy * dy <= xy_tolerance_m * xy_tolerance_m


def _plugin(section: Mapping, name: str) -> Mapping:
    block = section.get(name)
    return block if isinstance(block, Mapping) else {}


def tolerances_from_nav2(doc: Mapping) -> tuple:
    """``(xy_m, yaw_rad, w_rad_s)`` из боевого nav2_params.

    xy и yaw — у ``general_goal_checker`` (SimpleGoalChecker). Скорость разворота —
    ``FollowPath.max_vel_theta`` (та же, что у DWB, не отдельная константа).
    Нет блока, чужой плагин или нечисло — ``ValueError``, не тихий дефолт.
    """
    params = doc.get("controller_server", {}).get("ros__parameters", {})
    if not isinstance(params, Mapping):
        raise ValueError("controller_server.ros__parameters нет")
    checker_name = "general_goal_checker"
    names = params.get("goal_checker_plugins")
    if isinstance(names, list) and names:
        checker_name = str(names[0])
    checker = _plugin(params, checker_name)
    if checker.get("plugin") != CHECKER_PLUGIN:
        raise ValueError(f"{checker_name}.plugin={checker.get('plugin')!r}, ждали {CHECKER_PLUGIN}")
    planner = _plugin(params, "FollowPath")
    if planner.get("plugin") != PLANNER_PLUGIN:
        raise ValueError(f"FollowPath.plugin={planner.get('plugin')!r}, ждали {PLANNER_PLUGIN}")
    try:
        xy = float(checker["xy_goal_tolerance"])
        yaw = float(checker["yaw_goal_tolerance"])
        omega = float(planner["max_vel_theta"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"допуски цели не числа: {exc}") from exc
    if xy <= 0.0 or yaw <= 0.0 or omega <= 0.0:
        raise ValueError(f"допуски цели неположительны: xy={xy} yaw={yaw} w={omega}")
    return xy, yaw, omega


@dataclass(frozen=True)
class Turn:
    """Один такт разворота. ``omega`` — рад/с в ``cmd_vel.angular.z``; 0 и ``done`` — стоп."""

    omega: float
    done: bool
    reason: str


def turn_at_goal(
    pose: Pose,
    goal: Pose,
    xy_tolerance_m: float,
    yaw_tolerance_rad: float,
    max_omega: float,
) -> Optional[Turn]:
    """Разворот, только если xy уже внутри допуска, а yaw нет.

    Снаружи xy-окна — ``None``: это ещё FollowPath, не наш разворот. Внутри окна и
    внутри допуска yaw — стоп (``done``): цель принял бы goal checker. Иначе крутим
    к цели со знаком кратчайшего угла, не быстрее ``max_omega``.
    """
    if max_omega <= 0.0 or not xy_inside(pose, goal, xy_tolerance_m):
        return None
    dyaw = shortest_yaw(float(pose[2]), float(goal[2]))
    if not yaw_outside(float(pose[2]), float(goal[2]), yaw_tolerance_rad):
        return Turn(0.0, True, "yaw внутри допуска")
    omega = math.copysign(min(abs(dyaw), max_omega), dyaw)
    return Turn(omega, False, "xy внутри допуска, yaw вне — разворот на месте")
