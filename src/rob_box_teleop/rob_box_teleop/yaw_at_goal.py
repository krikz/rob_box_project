"""Финальный подъезд к цели, когда xy уже внутри допуска, а yaw нет (#4694).

DWB ``RotateToGoal`` обязан крутить на месте внутри ``xy_goal_tolerance``. На Depot он этого
не делает, и «разворот на месте» по истине идёт по окружности: Gate 2 21.11, проход 1, M3 —
``base_footprint`` описывает круг радиуса ``ROTATION_SWEEP_M`` (0.088 м). Один импульс курса
на границе 0.25 м сам выносит позу из окна, ``goal_from_plan`` при замороженном плане
(``PLAN_STALE_S``) возвращает ``None``, и разворот отпускает пол. Дальше DWB защёлкнут
(``rotating_`` снимается только ``setPlan``) и везти не может, а progress checker (только
трансляция) рвёт FollowPath.

Поэтому нода — не «только разворот», а детерминированный финальный подъезд: держит цель,
пока сама не доедет, даже если план замер; сначала доворачивает нос на точку и едет к ней;
у точки доворачивает на курс цели; отдаёт пол mux одним нулём и молчит. Допуск goal checker'а
(0.25 м / 0.30 рад) и допуск приёмки (0.32 м / 0.35 рад) не ослабляем: точка подъезда
``APPROACH_TARGET_M`` выбрана так, что даже после сноса разворотом поза остаётся внутри окна.

Решение — чистая функция. Нода подаёт сюда позу и удержанную цель и публикует решённый
``Twist`` (и ``linear.x``, и ``angular.z``) в ``/cmd_vel_yaw`` (twist_mux priority 15, выше
Nav2) только пока подъезд идёт. ``None`` и ``done`` нода не льёт нулями: timeout 0.2 с
должен истечь.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional

# Как SimpleGoalChecker (Humble): stateful, цель принята только если |dyaw| ≤ допуск.
CHECKER_PLUGIN = "nav2_controller::SimpleGoalChecker"
# DWB: скорости боевого FollowPath. Не выдумываем — читаем из того же файла.
PLANNER_PLUGIN = "dwb_core::DWBLocalPlanner"

#: Поза: x, y, yaw (рад, любой обёртки).
Pose = tuple

# Gate 2 Depot 21.11, проход 1, M3 (develop 1898f12bd): «разворот на месте» по истине
# идёт по окружности этого радиуса вокруг точки, не совпадающей с целью. При yaw +0.2°
# дистанция до цели была 0.396 м при vx=0. Запас ниже считается от этого замера.
ROTATION_SWEEP_M = 0.088
# Куда подъезжаем, прежде чем крутить курс. 0.10 + 0.088 = 0.188 ≤ 0.25 − 0.062:
# даже если доворот к курсу цели снова снесёт корпус по окружности, поза останется
# внутри xy_goal_tolerance. Инвариант проверяет тест по боевому yaml.
APPROACH_TARGET_M = 0.10
# Нос дальше этого от направления на точку — сначала доворот, потом ход. 0.15 рад ≈ 8.6°.
BEARING_ALIGN_RAD = 0.15
# Потолок нашей трансляции. Реальный предел — min(это, FollowPath.max_speed_xy):
# быстрее контроллера не едем, даже если yaml поднимут.
APPROACH_MAX_VX_M_S = 0.10
# Потолок нашего курса на подъезде (не на чистом довороте: там предел — max_vel_theta).
APPROACH_MAX_W_RAD_S = 0.30
# Доворачиваем курс чуть глубже допуска checker'а, чтобы done не сел ровно на границу.
FINISH_YAW_MARGIN_RAD = 0.05
# Поза ушла дальше этого от удержанной цели — это уже не финал, пол обратно Nav2.
ESCAPE_RADIUS_M = 0.60
# Дольше этого одну цель не держим: замороженный хвост не должен крутить вечно.
ENGAGE_TIME_CAP_S = 25.0
# После потолка столько тишины, прежде чем снова взять тот же хвост.
REENGAGE_COOLDOWN_S = 5.0
# Свежий хвост /plan отличается от удержанного — checker сравнивает с новым, берём его.
# Меньше этого — тот же замороженный план, часы захвата не сбрасываем.
GOAL_CHANGE_EPS_M = 0.005
GOAL_CHANGE_EPS_RAD = 0.01


def shortest_yaw(current: float, goal: float) -> float:
    """Кратчайший поворот current → goal, рад, в (−π, π]."""
    return math.atan2(math.sin(goal - current), math.cos(goal - current))


def yaw_outside(current: float, goal: float, tolerance: float) -> bool:
    """Курс вне допуска goal checker'а. Допуск ≤ 0 — вне всегда (дырявый допуск не «уже на курсе»)."""
    if tolerance <= 0.0:
        return True
    return abs(shortest_yaw(current, goal)) > tolerance


def pose_distance(pose: Optional[Pose], goal: Optional[Pose]) -> float:
    """Расстояние до точки цели, м. Нет позы или цели — бесконечность (не «уже на месте»)."""
    if pose is None or goal is None:
        return math.inf
    return math.hypot(float(pose[0]) - float(goal[0]), float(pose[1]) - float(goal[1]))


def xy_inside(pose: Optional[Pose], goal: Pose, xy_tolerance_m: float) -> bool:
    """Поза внутри xy-допуска цели. Нет позы или допуск ≤ 0 — нет."""
    if pose is None or xy_tolerance_m <= 0.0:
        return False
    return pose_distance(pose, goal) <= xy_tolerance_m


def bearing_to(pose: Pose, goal: Pose) -> float:
    """Кратчайший угол от курса робота на точку цели, рад, в (−π, π]."""
    toward = math.atan2(
        float(goal[1]) - float(pose[1]), float(goal[0]) - float(pose[0])
    )
    return shortest_yaw(float(pose[2]), toward)


def goal_differs(
    a: Optional[Pose], b: Optional[Pose], eps_m: float, eps_rad: float
) -> bool:
    """Хвосты разные: точка дальше ``eps_m`` или курс дальше ``eps_rad``. Нет одного — разные."""
    if a is None or b is None:
        return a is not b
    if pose_distance(a, b) > eps_m:
        return True
    return abs(shortest_yaw(float(a[2]), float(b[2]))) > eps_rad


def _plugin(section: Mapping, name: str) -> Mapping:
    block = section.get(name)
    return block if isinstance(block, Mapping) else {}


def tolerances_from_nav2(doc: Mapping) -> tuple:
    """``(xy_m, yaw_rad, w_rad_s, v_m_s)`` из боевого nav2_params.

    xy и yaw — у ``general_goal_checker`` (SimpleGoalChecker). Скорость разворота —
    ``FollowPath.max_vel_theta``, скорость хода — ``FollowPath.max_speed_xy``
    (те же, что у DWB, не отдельные константы).
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
        raise ValueError(
            f"{checker_name}.plugin={checker.get('plugin')!r}, ждали {CHECKER_PLUGIN}"
        )
    planner = _plugin(params, "FollowPath")
    if planner.get("plugin") != PLANNER_PLUGIN:
        raise ValueError(
            f"FollowPath.plugin={planner.get('plugin')!r}, ждали {PLANNER_PLUGIN}"
        )
    try:
        xy = float(checker["xy_goal_tolerance"])
        yaw = float(checker["yaw_goal_tolerance"])
        omega = float(planner["max_vel_theta"])
        speed = float(planner["max_speed_xy"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"допуски цели не числа: {exc}") from exc
    if xy <= 0.0 or yaw <= 0.0 or omega <= 0.0 or speed <= 0.0:
        raise ValueError(
            f"допуски цели неположительны: xy={xy} yaw={yaw} w={omega} v={speed}"
        )
    return xy, yaw, omega, speed


@dataclass(frozen=True)
class Approach:
    """Один такт подъезда. ``vx``/``omega`` — в ``Twist``; 0 и ``done`` — стоп, пол у Nav2."""

    vx: float
    omega: float
    done: bool
    reason: str


def _align(angle: float, max_w: float, reason: str) -> Approach:
    """Чистый доворот: vx=0, курс к знаку угла, не быстрее ``max_w``."""
    omega = math.copysign(min(abs(angle), max_w), angle)
    return Approach(0.0, omega, False, reason)


def approach_at_goal(
    pose: Optional[Pose],
    goal: Optional[Pose],
    xy_tol: float,
    yaw_tol: float,
    max_v: float,
    max_w: float,
) -> Optional[Approach]:
    """Финальный подъезд: доворот на точку, ход, доворот на курс цели.

    ``max_v``/``max_w`` ≤ 0 или нет позы/цели — ``None`` (нечего решать).
    Дальше ``APPROACH_TARGET_M`` нос не на точке — доворот на месте; нос на точке —
    ход не быстрее ``min(max_v, APPROACH_MAX_VX_M_S)`` с подруливанием.
    Ближе — доворот на курс цели чуть глубже ``yaw_tol``. Оба в допуске — ``done``.
    ``xy_tol`` здесь не решает такт (окно захвата — у ноды), но это допуск checker'а:
    точка подъезда обязана оставить после сноса разворотом запас внутри него.
    """
    del xy_tol  # допуск checker'а; окно захвата проверяет нода, не такт
    if max_v <= 0.0 or max_w <= 0.0 or pose is None or goal is None:
        return None
    dist = pose_distance(pose, goal)
    if dist > APPROACH_TARGET_M:
        return _toward_point(pose, goal, max_v, max_w)
    return _finish_yaw(pose, goal, yaw_tol, max_w)


def _finish_yaw(pose: Pose, goal: Pose, yaw_tol: float, max_w: float) -> Approach:
    """Заключительный этап: только курс цели, без возврата к курсу-на-точку.

    Ход здесь снова открыл бы границу, а заметание разворота само качает dist
    через любой порог меньше ``ROTATION_SWEEP_M`` (модель: порог 0.088 м, знак
    wz скачет 35 раз за 8 с). Вход на 0.10 м даёт худший радиус 0.188 м — ниже
    checker 0.25 и планки приёмки 0.32.
    """
    return _align_goal_yaw(pose, goal, yaw_tol, max_w)


def _align_goal_yaw(pose: Pose, goal: Pose, yaw_tol: float, max_w: float) -> Approach:
    """Курс цели, когда точка уже ближе заметания. Иначе стоп."""
    dyaw = shortest_yaw(float(pose[2]), float(goal[2]))
    if abs(dyaw) > yaw_tol - FINISH_YAW_MARGIN_RAD:
        return _align(dyaw, max_w, "доворот к курсу цели")
    return Approach(0.0, 0.0, True, "цель в допуске")


class ApproachStage:
    """Защёлка заключительного этапа. Один объект на одну удержанную цель.

    ``approach_at_goal`` сам по себе stateless и на границе ``APPROACH_TARGET_M``
    переключает режим. Разворот на месте двигает ``base_footprint`` по окружности
    ``ROTATION_SWEEP_M``, dist скачет через 0.10 м, знак ``wz`` перекидывается
    (Gate 2 21.11, passA_yaw.log 283–322). Защёлка входит при dist ≤ 0.10 и не
    отпускает этап, пока цель та же и поза внутри радиуса побега. Внутри этапа
    сначала подъезд к точке, затем доворот на курс цели — без возврата наружу.

    Сброс — явные события ноды: новая цель (``goal_differs``), побег
    (dist > ``ESCAPE_RADIUS_M``), потолок времени и done (``forget``). Гистерезис
    по dist не нужен: побег и новая цель уже есть в контракте удержания.
    """

    def __init__(self) -> None:
        self._goal: Optional[Pose] = None

    @property
    def finishing(self) -> bool:
        return self._goal is not None

    def forget(self) -> None:
        """Потолок, done, Nav погас, нет позы-удержания — этап снят."""
        self._goal = None

    def step(
        self,
        pose: Optional[Pose],
        goal: Optional[Pose],
        xy_tol: float,
        yaw_tol: float,
        max_v: float,
        max_w: float,
    ) -> Optional[Approach]:
        """Такт с защёлкой. Контракт ``Approach`` тот же, что у ``approach_at_goal``."""
        if max_v <= 0.0 or max_w <= 0.0 or pose is None or goal is None:
            return None
        if self._goal is None or goal_differs(
            self._goal, goal, GOAL_CHANGE_EPS_M, GOAL_CHANGE_EPS_RAD
        ):
            self._goal = None
        elif pose_distance(pose, self._goal) > ESCAPE_RADIUS_M:
            self._goal = None
        if self._goal is None and pose_distance(pose, goal) <= APPROACH_TARGET_M:
            self._goal = goal
        if self._goal is not None:
            step = _finish_yaw(pose, goal, yaw_tol, max_w)
            if step.done:
                self.forget()
            return step
        return approach_at_goal(pose, goal, xy_tol, yaw_tol, max_v, max_w)


def _toward_point(pose: Pose, goal: Pose, max_v: float, max_w: float) -> Approach:
    """Дальше точки подъезда: нос не туда — доворот, нос туда — ход с подруливанием."""
    bearing = bearing_to(pose, goal)
    if abs(bearing) > BEARING_ALIGN_RAD:
        return _align(bearing, max_w, "доворот на точку")
    speed = min(max_v, APPROACH_MAX_VX_M_S)
    omega = max(-APPROACH_MAX_W_RAD_S, min(APPROACH_MAX_W_RAD_S, bearing))
    return Approach(speed, omega, False, "подъезд к точке")
