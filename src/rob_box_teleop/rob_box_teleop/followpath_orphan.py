"""Сторож «сирот» FollowPath (#4458): решение «гасить ли цель контроллера» — чистая функция без ROS.

«Сирота» — цель ``controller_server`` (FollowPath), которая ведёт робота, хотя последняя цель NavigateToPose
уже в терминальном статусе (#4435: ``bt_navigator`` не дождался подтверждения цели и не может её отменить).
Нода-сторож (``followpath_watchdog_node``) подаёт сюда списки статусов action и время; отмену и нулевую
скорость выполняет она же.

Правило: последняя (по ``stamp``) цель NavigateToPose терминальна (SUCCEEDED/CANCELED/ABORTED), нет ни одной
живой цели NavigateToPose, а цель FollowPath ACCEPTED/EXECUTING дольше ``orphan_timeout_s`` — отменить её.
Одну цель отменяем один раз. Если NavigateToPose не было вовсе, FollowPath ведёт чужой клиент — не трогаем.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping, Tuple

# action_msgs/GoalStatus
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

TERMINAL = frozenset({STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED})
#: Живая цель NavigateToPose — любая, что ещё не завершилась (включая «отменяется»).
NAV_LIVE = frozenset({STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING})
#: FollowPath, который ведёт робота.
FOLLOW_ACTIVE = frozenset({STATUS_ACCEPTED, STATUS_EXECUTING})

STATUS_NAMES = {
    STATUS_ACCEPTED: "ACCEPTED",
    STATUS_EXECUTING: "EXECUTING",
    STATUS_CANCELING: "CANCELING",
    STATUS_SUCCEEDED: "SUCCEEDED",
    STATUS_CANCELED: "CANCELED",
    STATUS_ABORTED: "ABORTED",
}

#: Цель action: ``(goal_id, stamp в секундах, статус)``.
Goal = Tuple[str, float, int]


@dataclass(frozen=True)
class WatchState:
    """Состояние сторожа между вызовами: когда увидели сироту и каких уже отменили."""

    orphan_since: Mapping[str, float] = None  # type: ignore[assignment]  # goal_id -> время первого наблюдения
    cancelled: FrozenSet[str] = frozenset()

    def __post_init__(self) -> None:
        if self.orphan_since is None:
            object.__setattr__(self, "orphan_since", {})


@dataclass(frozen=True)
class Decision:
    """Исход одного такта: что отменить сейчас и новое состояние."""

    cancel: Tuple[str, ...]
    state: WatchState
    nav_goal: str = ""  # последняя цель NavigateToPose и её статус — для журнала
    nav_status: int = 0


def decide(state: WatchState, now: float, nav_goals, follow_goals, orphan_timeout_s: float) -> Decision:
    """Один такт сторожа. ``nav_goals`` и ``follow_goals`` — списки ``Goal`` из ``GoalStatusArray``."""
    if not nav_goals:
        return Decision((), WatchState(cancelled=state.cancelled))
    last_id, _, last_status = max(nav_goals, key=lambda g: g[1])
    nav_alive = any(status in NAV_LIVE for _, _, status in nav_goals)
    if nav_alive or last_status not in TERMINAL:
        return Decision((), WatchState(cancelled=state.cancelled), last_id, last_status)

    active = {gid for gid, _, status in follow_goals if status in FOLLOW_ACTIVE}
    since = {gid: state.orphan_since.get(gid, now) for gid in active if gid not in state.cancelled}
    due = tuple(sorted(gid for gid, t in since.items() if now - t >= orphan_timeout_s))
    # отменённых помним, пока цель есть в статусах (память не растёт)
    listed = frozenset(gid for gid, _, _ in follow_goals)
    cancelled = (state.cancelled & listed) | frozenset(due)
    remaining = {gid: t for gid, t in since.items() if gid not in due}
    return Decision(due, WatchState(remaining, cancelled), last_id, last_status)
