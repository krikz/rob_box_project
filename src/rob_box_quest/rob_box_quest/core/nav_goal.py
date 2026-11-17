"""Nav-цель из VR: чистая логика (issue #3151, Captain Bridge волна 2).

Без ROS и без сокетов — разбор ``JSON_CMD{cmd:"nav_goal"}``, таблица
статусов action_msgs → wire-состояние и сборка ``nav_status``. ROS-обвязка
(ActionClient ``navigate_to_pose``) живёт в :mod:`rob_box_quest.nav2_goal`,
сокетная — в ``server/ws_server.py``.

Контракт wire (meta-quest-api.md §5/§6, bridge_protocol.COMMANDS/EVENTS):

* ``nav_goal {seq, x, y, yaw, frame:"map", ts_ms}`` — цель уже в ``map``.
  Сервер не знает, какой позой робота клиент пересчитывал точку с пола,
  поэтому frame обязан быть ``map`` явно, иначе честный отказ.
* ``nav_status {state, seq, x, y, yaw, distance_remaining?, reason?, ts_ms}``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

#: Единственный кадр, который принимает мост (см. шапку).
NAV_GOAL_FRAME: str = "map"

# Причины nav_goal_nack (перечислены в bridge_protocol EventSpec).
NACK_BAD_PAYLOAD = "bad_payload"
NACK_BAD_FRAME = "bad_frame"
NACK_FLOOR_HELD = "floor_held"
NACK_EMERGENCY = "emergency_active"
NACK_NAV2_UNAVAILABLE = "nav2_unavailable"

# Wire-состояния nav_status.
STATE_ACCEPTED = "accepted"
STATE_REJECTED = "rejected"
STATE_ACTIVE = "active"
STATE_SUCCEEDED = "succeeded"
STATE_ABORTED = "aborted"
STATE_CANCELED = "canceled"

#: Состояния, после которых цель закрыта (путь гасим, пин снимаем).
TERMINAL_STATES = frozenset({STATE_REJECTED, STATE_SUCCEEDED, STATE_ABORTED, STATE_CANCELED})

# action_msgs/msg/GoalStatus (числа из IDL, чтобы не тянуть action_msgs в
# чистый модуль): 1 ACCEPTED, 2 EXECUTING, 3 CANCELING, 4 SUCCEEDED,
# 5 CANCELED, 6 ABORTED.
_GOAL_STATUS_TO_STATE: dict[int, str] = {
    1: STATE_ACCEPTED,
    2: STATE_ACTIVE,
    3: STATE_ACTIVE,  # CANCELING — ещё едет, пока BT не остановился
    4: STATE_SUCCEEDED,
    5: STATE_CANCELED,
    6: STATE_ABORTED,
}


@dataclass(frozen=True)
class NavGoalRequest:
    """Проверенная цель в кадре ``map``."""

    seq: int
    x: float
    y: float
    yaw: float


def _finite(value: Any) -> Optional[float]:
    # bool — подкласс int; true/false в поле координаты — битый payload.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    f = float(value)
    return f if math.isfinite(f) else None


def parse_nav_goal(payload: dict[str, Any]) -> NavGoalRequest | str:
    """Разобрать ``nav_goal``. Возвращает запрос или причину nack.

    ``yaw`` нормализуется в (−π, π] — Nav2 всё равно строит кватернион, а
    клиенту в nav_status удобнее видеть ту же нормальную форму.
    """
    if payload.get("frame") != NAV_GOAL_FRAME:
        return NACK_BAD_FRAME
    seq = payload.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int):
        return NACK_BAD_PAYLOAD
    x, y, yaw = (_finite(payload.get(k)) for k in ("x", "y", "yaw"))
    if x is None or y is None or yaw is None:
        return NACK_BAD_PAYLOAD
    return NavGoalRequest(seq=seq, x=x, y=y, yaw=normalize_angle(yaw))


def normalize_angle(a: float) -> float:
    """Угол в (−π, π]."""
    wrapped = math.atan2(math.sin(a), math.cos(a))
    return math.pi if wrapped == -math.pi else wrapped


def state_from_goal_status(code: int) -> Optional[str]:
    """action_msgs GoalStatus → wire-состояние. ``None`` — UNKNOWN (0) и мусор."""
    return _GOAL_STATUS_TO_STATE.get(int(code))


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    """Поворот вокруг Z → (qz, qw) — как в rob_box_mcp_tools _send_nav_goal."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def nav_status_event(
    req: NavGoalRequest,
    state: str,
    *,
    ts_ms: int,
    distance_remaining: Optional[float] = None,
    reason: Optional[str] = None,
) -> dict[str, Any]:
    """Собрать ``JSON_EVENT{type:"nav_status"}``.

    Опциональные поля кладутся только когда они есть — «нет данных» и
    «0 м до цели» для оператора разные вещи (ADR-0018).
    """
    event: dict[str, Any] = {
        "type": "nav_status",
        "state": state,
        "seq": req.seq,
        "x": req.x,
        "y": req.y,
        "yaw": req.yaw,
        "ts_ms": int(ts_ms),
    }
    if distance_remaining is not None and math.isfinite(distance_remaining):
        event["distance_remaining"] = float(distance_remaining)
    if reason:
        event["reason"] = str(reason)
    return event


class RateGate:
    """Пропускать не чаще раза в ``min_period_s`` (feedback Nav2 идёт ~100 Гц).

    Первое событие проходит сразу — оператор должен увидеть «едет», как
    только пошёл feedback.
    """

    def __init__(self, min_period_s: float) -> None:
        self._min_period_s = float(min_period_s)
        self._last: Optional[float] = None

    def admit(self, now: float) -> bool:
        if self._last is not None and now - self._last < self._min_period_s:
            return False
        self._last = now
        return True

    def reset(self) -> None:
        self._last = None
