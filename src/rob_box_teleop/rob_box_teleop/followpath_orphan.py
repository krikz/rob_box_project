"""Сторож «сирот» FollowPath (#4458): решение «гасить ли цель контроллера» — чистая функция без ROS.

«Сирота» — цель ``controller_server`` (FollowPath), которая ведёт робота, хотя родительская цель
bt_navigator уже терминальна (#4435: навигатор не дождался подтверждения и не может её отменить).
Родитель — любая цель, из которой дерево зовёт FollowPath: NavigateToPose (голос, Quest, HIL-скрипт)
и NavigateThroughPoses (полоса HIL). Источник цели правилу не важен: код смотрит статусы, не клиента.

Нода-сторож (``followpath_watchdog_node``) подаёт сюда списки статусов action и время; отмену и
нулевую скорость выполняет она же.

Правило: последняя (по ``stamp``) родительская цель терминальна (SUCCEEDED/CANCELED/ABORTED), нет
ни одной живой родительской цели, а FollowPath ACCEPTED/EXECUTING дольше ``orphan_timeout_s`` —
отменить её. Одну цель отменяем один раз. Если родительской цели не было вовсе, FollowPath ведёт
чужой клиент — не трогаем.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping, Sequence, Tuple

# action_msgs/GoalStatus
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

TERMINAL = frozenset({STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED})
#: Живая родительская цель — любая, что ещё не завершилась (включая «отменяется»).
PARENT_LIVE = frozenset({STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING})
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
#: Родительские цели одним списком: ``((источник, цели), ...)``. Источник — только для журнала.
ParentGoals = Sequence[Tuple[str, Sequence[Goal]]]


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
    parent_source: str = (
        ""  # какой action дал последнюю родительскую цель — для журнала
    )
    parent_goal: str = ""
    parent_status: int = 0


def _flatten(parent_goals: ParentGoals) -> Tuple[Tuple[str, Goal], ...]:
    return tuple((source, goal) for source, goals in parent_goals for goal in goals)


def decide(
    state: WatchState,
    now: float,
    parent_goals: ParentGoals,
    follow_goals: Sequence[Goal],
    orphan_timeout_s: float,
) -> Decision:
    """Один такт сторожа.

    ``parent_goals`` — статусы всех action, из которых дерево зовёт FollowPath.
    ``follow_goals`` — статусы ``/follow_path``. Оба списка — ``Goal`` из ``GoalStatusArray``.
    """
    parents = _flatten(parent_goals)
    if not parents:
        return Decision((), WatchState(cancelled=state.cancelled))
    source, (last_id, _, last_status) = max(parents, key=lambda item: item[1][1])
    parent_alive = any(goal[2] in PARENT_LIVE for _, goal in parents)
    if parent_alive or last_status not in TERMINAL:
        return Decision(
            (), WatchState(cancelled=state.cancelled), source, last_id, last_status
        )

    active = {gid for gid, _, status in follow_goals if status in FOLLOW_ACTIVE}
    since = {
        gid: state.orphan_since.get(gid, now)
        for gid in active
        if gid not in state.cancelled
    }
    due = tuple(sorted(gid for gid, t in since.items() if now - t >= orphan_timeout_s))
    # отменённых помним, пока цель есть в статусах (память не растёт)
    listed = frozenset(gid for gid, _, _ in follow_goals)
    cancelled = (state.cancelled & listed) | frozenset(due)
    remaining = {gid: t for gid, t in since.items() if gid not in due}
    return Decision(due, WatchState(remaining, cancelled), source, last_id, last_status)
