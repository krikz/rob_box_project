"""Отказ прогресса «упёрся в препятствие» (#5003). Решение — код, не лог.

Внешний recovery NavigateToPose (ClearThenBackUp, #5000) тикается только на
FAILURE ветки, а она падает лишь после двух подряд отказов FollowPath.
Humble ``SimpleProgressChecker`` считает прогрессом любое смещение центра
больше ``required_movement_radius`` (0.08 м) за ``movement_time_allowance``
(30 с). У стены DWB даёт это смещение вдоль стены, а упёртый нос при
ненулевой команде центр почти не двигает — до внешнего цикла не доходит
(Gate 2 21.11: «Running backup» = 0, таймаут 200 с).

Здесь отдельный признак, не замена радиуса. Отказ засчитывается, когда за
короткое окно одновременно:

* команда на движение была (ненулевой twist набрался дольше порога);
* центр сместился меньше радиуса прогресса;
* по курсу команды в локальной costmap препятствие (lethal/inscribed)
  ближе порога касания защиты (``side_stop_m``).

Движение вдоль стены (Δy ≈ +0.85 м за то же окно) радиус превышает — отказ
не засчитывается, даже если сбоку стена.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

from rob_box_teleop.proximity import Body, Thresholds

Cell = Tuple[int, int, int]
Pose = Tuple[float, float, float]

# Те же байты, что nav2_costmap_2d::LETHAL_OBSTACLE / INSCRIBED_INFLATED_OBSTACLE.
# Свой порог не вводим: это шкала costmap, а не дистанция.
LETHAL_COST = 254
INSCRIBED_COST = 253

# Окно короче movement_time_allowance (30 с): за 30 с DWB у стены набирает
# радиус 0.08 м вдоль стены, и признак никогда не сошёлся бы. 2 с — 40 тиков
# контроллера (20 Гц), меньше failure_tolerance (0.3 с) не берём: одиночный
# тик «нет траектории» — не упор.
STUCK_WINDOW_S = 2.0
# Команда должна держаться дольше trans_stopped_velocity-окна DWB, иначе
# нулевой twist между траекториями выглядит как «команды не было».
CMD_HOLD_S = 0.3
# |twist| ниже этого — не команда. Тот же порог, что у защиты (moving_v):
# нового числа модуль не вводит.
CMD_EPS = Thresholds().moving_v
# Смещение центра меньше required_movement_radius (nav2_params.yaml) — не
# прогресс. Тот же радиус, не новый.
PROGRESS_RADIUS_M = 0.08
# Клетка по курсу ближе этого — упор. Тот же side_stop_m защиты (касание
# борта/носа), не новый порог. stop_m (0.20) сюда нельзя: инфляцию 0.55 м
# он принял бы за стену на ровном месте.
BLOCK_GAP_M = Thresholds().side_stop_m


class StuckAgainst:
    """Копит окно и говорит, засчитан ли отказ «упёрся»."""

    def __init__(
        self,
        window_s: float = STUCK_WINDOW_S,
        cmd_hold_s: float = CMD_HOLD_S,
        cmd_eps: float = CMD_EPS,
        progress_radius_m: float = PROGRESS_RADIUS_M,
        block_gap_m: float = BLOCK_GAP_M,
    ) -> None:
        self.window_s = window_s
        self.cmd_hold_s = cmd_hold_s
        self.cmd_eps = cmd_eps
        self.progress_radius_m = progress_radius_m
        self.block_gap_m = block_gap_m
        self._samples: list[tuple[float, Pose, bool]] = []
        self._cmd_since: Optional[float] = None

    def reset(self) -> None:
        self._samples.clear()
        self._cmd_since = None

    def update(self, now_s: float, pose: Pose, commanded: bool) -> bool:
        """Один тик контроллера. ``True`` — отказ прогресса засчитан."""
        if commanded:
            if self._cmd_since is None:
                self._cmd_since = now_s
        else:
            self._cmd_since = None
        self._samples.append((now_s, pose, commanded and self._cmd_held(now_s)))
        cutoff = now_s - self.window_s
        self._samples = [s for s in self._samples if s[0] >= cutoff]
        return self.refused()

    def refused(self) -> bool:
        if len(self._samples) < 2:
            return False
        start_t, start_pose, _ = self._samples[0]
        end_t = self._samples[-1][0]
        if end_t - start_t < self.window_s:
            return False
        if not any(held for _, _, held in self._samples):
            return False
        if self._moved_enough(start_pose):
            return False
        return True

    def note_blocked(self, blocked: bool) -> bool:
        """Отказ только вместе с препятствием по курсу. Без него — нет."""
        return blocked and self.refused()

    def _cmd_held(self, now_s: float) -> bool:
        return (
            self._cmd_since is not None and now_s - self._cmd_since >= self.cmd_hold_s
        )

    def _moved_enough(self, start: Pose) -> bool:
        sx, sy, _ = start
        for _, (x, y, _), _ in self._samples:
            if math.hypot(x - sx, y - sy) > self.progress_radius_m:
                return True
        return False


def controller_stuck(
    now_s: float,
    x: float,
    y: float,
    vx: float,
    wz: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    size_x: int,
    size_y: int,
    costs: Sequence[int],
    body: Body = Body(-0.3036, 0.3064, -0.3025, 0.3025),
    state: Optional[StuckAgainst] = None,
) -> bool:
    """Один тик из controller_server (#5003). ``True`` — отказ прогресса.

    ``body`` — footprint local_costmap (nav2_params.yaml, замер 04.11).
    Контроллер зовёт раз за тик и держит ``state`` между тиками; без него
    окно копится в модуле, потому что C++ своего состояния не хранит.
    """
    held = command_is_motion(vx, wz)
    checker = _CONTROLLER_STATE if state is None else state
    checker.update(now_s, (x, y, 0.0), held)
    blocked = course_blocked(
        body, origin_x, origin_y, resolution, size_x, size_y, costs, vx
    )
    return checker.note_blocked(blocked)


_CONTROLLER_STATE = StuckAgainst()


def reset_controller_state() -> None:
    """Новая цель FollowPath: окно упора начинается заново."""
    _CONTROLLER_STATE.reset()


def command_is_motion(vx: float, wz: float, cmd_eps: float = CMD_EPS) -> bool:
    """Ненулевая команда: |v| или |w| выше порога «стоим»."""
    return abs(vx) > cmd_eps or abs(wz) > cmd_eps


def course_blocked(
    body: Body,
    origin_x: float,
    origin_y: float,
    resolution: float,
    size_x: int,
    size_y: int,
    costs: Sequence[int],
    vx: float,
    gap_m: float = BLOCK_GAP_M,
    lethal: int = INSCRIBED_COST,
) -> bool:
    """По курсу команды в costmap есть препятствие ближе ``gap_m`` от корпуса.

    ``costs`` — строка за строкой, как ``Costmap2D::getCharMap`` (index =
    my * size_x + mx). Курс — знак ``vx`` (вперёд/назад в ``base_footprint``);
    нулевой ``vx`` курса не имеет, отказ по карте не ставится.
    """
    if resolution <= 0.0 or abs(vx) <= CMD_EPS:
        return False
    if len(costs) < size_x * size_y:
        return False
    for mx, my, _cost in _cells_ahead(
        body, origin_x, origin_y, resolution, size_x, size_y, vx, gap_m
    ):
        if costs[my * size_x + mx] >= lethal:
            return True
    return False


def _cells_ahead(
    body: Body,
    origin_x: float,
    origin_y: float,
    resolution: float,
    size_x: int,
    size_y: int,
    vx: float,
    gap_m: float,
) -> Iterable[Cell]:
    """Клетки полосы корпуса по курсу, от края корпуса до ``gap_m``, без самого корпуса."""
    forward = vx > 0.0
    edge = body.x_max if forward else body.x_min
    reach = edge + gap_m if forward else edge - gap_m
    x0, x1 = (edge, reach) if forward else (reach, edge)
    y0, y1 = body.y_min, body.y_max
    mx0 = max(0, math.floor((x0 - origin_x) / resolution))
    mx1 = min(size_x - 1, math.floor((x1 - origin_x) / resolution))
    my0 = max(0, math.floor((y0 - origin_y) / resolution))
    my1 = min(size_y - 1, math.floor((y1 - origin_y) / resolution))
    for mx in range(mx0, mx1 + 1):
        cx = origin_x + (mx + 0.5) * resolution
        if forward and cx <= edge:
            continue
        if not forward and cx >= edge:
            continue
        if abs(cx - edge) > gap_m:
            continue
        for my in range(my0, my1 + 1):
            yield mx, my, 0
