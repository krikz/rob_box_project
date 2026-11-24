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

* в окне была ненулевая команда (twist выше порога дольше
  ``cmd_hold``). Ноль прямо сейчас не снимает признак: DWB при полном
  блоке бросает ``NoLegalTrajectoriesException`` и ничего не публикует,
  а ``controller_server`` в ``catch`` пишет нулевой twist на время
  ``failure_tolerance`` (0.3 с) и потом сам абортит. Если требовать
  команду на этом тике, лобовой упор не засчитается;
* центр сместился меньше радиуса прогресса;
* по курсу последней ненулевой команды в локальной costmap препятствие
  (lethal/inscribed) ближе порога стопа защиты в коридоре хода
  (``stop_m``). Касание борта (``side_stop_m``, 0.08 м) тут молчит:
  защита останавливает корпус на 0.20 м, и полоса 0.08 м перед носом
  пуста (HIL 22.11: команда была, смещения нет, отказ не ставился).

Движение вдоль стены (Δy ≈ +0.85 м за то же окно) радиус превышает — отказ
не засчитывается, даже если сбоку стена.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

from rob_box_teleop.proximity import Body, Thresholds

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
# Клетка по курсу ближе этого — упор. Тот же stop_m защиты (коридор хода,
# proximity.py), не новый порог. side_stop_m (0.08) тут молчит: защита
# держит стоп, пока команда толкает в препятствие ближе 0.20 м, и полоса
# 0.08 м перед носом пуста — отказ не ставится, BackUp не тикается.
# Инфляция 0.55 м сама по себе не lethal/inscribed: порог смотрит только
# эти две стоимости, не серую инфляцию.
BLOCK_GAP_M = Thresholds().stop_m


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
        self._last_motion_course: float = 0.0
        self._last_course_time = -math.inf

    def reset(self) -> None:
        self._samples.clear()
        self._cmd_since = None
        self._last_motion_course = 0.0
        self._last_course_time = -math.inf

    def update(self, now_s: float, pose: Pose, commanded: bool, vx: float = 0.0) -> bool:
        """Один тик контроллера. ``True`` — отказ прогресса засчитан."""
        if self._samples:
            dt = now_s - self._samples[-1][0]
            if dt == 0:
                return False
            if dt < 0 or dt > self.cmd_hold_s:
                self.reset()
        if abs(vx) > self.cmd_eps:
            self._last_motion_course = vx
            self._last_course_time = now_s
        if commanded:
            if self._cmd_since is None:
                self._cmd_since = now_s
        else:
            self._cmd_since = None
        held = commanded and self._cmd_held(now_s)
        self._samples.append((now_s, pose, held))
        cutoff = now_s - self.window_s
        # Keep the last pose at or before the boundary, even between controller ticks.
        anchor = 0
        while anchor + 1 < len(self._samples) and self._samples[anchor + 1][0] <= cutoff:
            anchor += 1
        del self._samples[:anchor]
        return self.refused()

    def course(self, now_s: float) -> float:
        """Курс для карты: знак последней ненулевой ``vx`` в окне.

        Ноль на этом тике курс не стирает. DWB при лобовом упоре публикует
        не ноль, а исключение; ноль пишет controller_server в catch. Курс
        берётся из команды, которая была до этого нуля.
        """
        if self._last_course_time < now_s - self.window_s:
            return 0.0
        return self._last_motion_course

    def refused(self) -> bool:
        if len(self._samples) < 2:
            return False
        start_t, start_pose, _ = self._samples[0]
        cutoff = self._samples[-1][0] - self.window_s
        if start_t > cutoff:
            return False
        if not any(held and t >= cutoff for t, _, held in self._samples):
            return False
        if self._moved_enough(start_pose):
            return False
        return True

    def note_blocked(self, blocked: bool) -> bool:
        """Отказ только вместе с препятствием по курсу. Без него — нет."""
        return blocked and self.refused()

    def _cmd_held(self, now_s: float) -> bool:
        return self._cmd_since is not None and now_s - self._cmd_since >= self.cmd_hold_s

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
    yaw: float = 0.0,
) -> bool:
    """Один тик из controller_server (#5003). ``True`` — отказ прогресса.

    ``body`` — footprint local_costmap (nav2_params.yaml, замер 04.11).
    ``(x, y, yaw)`` — поза ``base_footprint`` в кадре карты (у контроллера
    это ``odom``). Полоса курса ищется от этой позы, не от начала карты.
    Контроллер зовёт раз за тик и держит ``state`` между тиками; без него
    окно копится в модуле, потому что C++ своего состояния не хранит.
    """
    held = command_is_motion(vx, wz)
    checker = _CONTROLLER_STATE if state is None else state
    if not checker.update(now_s, (x, y, yaw), held, vx):
        return False
    # Курс — последняя ненулевая vx в окне, не vx этого тика: после
    # NoLegalTrajectoriesException controller_server пишет ноль.
    return course_blocked(
        body,
        origin_x,
        origin_y,
        resolution,
        size_x,
        size_y,
        costs,
        checker.course(now_s),
        robot_x=x,
        robot_y=y,
        robot_yaw=yaw,
    )


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
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_yaw: float = 0.0,
) -> bool:
    """По курсу команды в costmap есть препятствие ближе ``gap_m`` от корпуса.

    ``costs`` — строка за строкой, как ``Costmap2D::getCharMap`` (index =
    my * size_x + mx). Курс — знак ``vx`` (вперёд/назад в ``base_footprint``);
    нулевой ``vx`` курса не имеет, отказ по карте не ставится.

    Полоса ищется в кадре корпуса, затем переводится в кадр карты. Локальная
    карта контроллера живёт в ``odom`` (nav2_params.yaml ``global_frame``), а
    ``(robot_x, robot_y, robot_yaw)`` — поза ``base_footprint`` в этом кадре.
    Ноль по умолчанию оставляет старый вызов: карта, чьё начало совпало с
    корпусом. Мировые координаты корпуса как координаты клетки — не то: у
    стены Depot корпус стоит на метрах, а полоса 0.08 м от края ищется у
    начала карты и стену не видит (#5003).
    """
    if resolution <= 0.0 or abs(vx) <= CMD_EPS:
        return False
    if len(costs) < size_x * size_y:
        return False
    for mx, my in cells_ahead_in_map(
        body,
        origin_x,
        origin_y,
        resolution,
        size_x,
        size_y,
        vx,
        gap_m,
        robot_x,
        robot_y,
        robot_yaw,
    ):
        if costs[my * size_x + mx] >= lethal:
            return True
    return False


def cells_ahead_in_map(
    body: Body,
    origin_x: float,
    origin_y: float,
    resolution: float,
    size_x: int,
    size_y: int,
    vx: float,
    gap_m: float = BLOCK_GAP_M,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_yaw: float = 0.0,
) -> Iterable[Tuple[int, int]]:
    """Клетки полосы по курсу в индексах карты.

    Полоса задана в ``base_footprint`` (от края корпуса до ``gap_m``, без
    самого корпуса). Клетка попадает, если её центр, переведённый в корпус,
    лежит в этой полосе. Так стена впереди корпуса находится и когда начало
    карты — в ``odom``, а не в корпусе.
    """
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    forward = vx > 0.0
    seen: set[Tuple[int, int]] = set()
    for bx, by in _body_strip(body, vx, gap_m, resolution):
        wx = robot_x + bx * cos_yaw - by * sin_yaw
        wy = robot_y + bx * sin_yaw + by * cos_yaw
        mx = math.floor((wx - origin_x) / resolution)
        my = math.floor((wy - origin_y) / resolution)
        if mx < 0 or my < 0 or mx >= size_x or my >= size_y:
            continue
        cx = origin_x + (mx + 0.5) * resolution
        cy = origin_y + (my + 0.5) * resolution
        # Центр клетки обратно в корпус: полоса режется по нему, не по углу.
        dx, dy = cx - robot_x, cy - robot_y
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        if not _in_body_strip(body, local_x, local_y, forward, gap_m):
            continue
        cell = (mx, my)
        if cell not in seen:
            seen.add(cell)
            yield cell


def _body_strip(body: Body, vx: float, gap_m: float, resolution: float) -> Iterable[Tuple[float, float]]:
    """Точки полосы в кадре корпуса, шаг мельче клетки: центр клетки не пропустим."""
    forward = vx > 0.0
    edge = body.x_max if forward else body.x_min
    reach = edge + gap_m if forward else edge - gap_m
    x0, x1 = (edge, reach) if forward else (reach, edge)
    step = resolution / 2.0 if resolution > 0.0 else gap_m
    x = x0
    while x <= x1 + 1e-9:
        y = body.y_min
        while y <= body.y_max + 1e-9:
            yield x, y
            y += step
        x += step


def _in_body_strip(body: Body, local_x: float, local_y: float, forward: bool, gap_m: float) -> bool:
    """Центр клетки в полосе курса: за краем корпуса, не дальше ``gap_m``, в ширине корпуса."""
    edge = body.x_max if forward else body.x_min
    if forward and local_x <= edge:
        return False
    if not forward and local_x >= edge:
        return False
    if abs(local_x - edge) > gap_m:
        return False
    return body.y_min <= local_y <= body.y_max
