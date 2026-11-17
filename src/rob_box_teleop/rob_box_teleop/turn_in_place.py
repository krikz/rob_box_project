"""Разворот на месте до свободного курса (#4475). Решение — код, не контроллер и не LLM.

HIL 06.11 / 09.11 (`docs/research/2026-11-05-nav2-hil-tuning.md` §8, §10): путь к цели есть,
робот стоит лицом в перегородку проёма или в торец тупика, прогресс нулевой. DWB в этой позе
не выбирает разворот (`path=0`), а recovery `Spin` на фиксированные 1.57 рад в тупике 1.1 м
честно падает «Collision Ahead»: вокруг задней оси полный оборот не влезает.

Здесь только выбор угла. «Свободен» — тот же зазор, который уже измеряет защита
(`proximity.gap` и её порог, вызывающий передаёт его сам): нового числа модуль не вводит.
Вызывающий шлёт `Spin` на возвращённый угол и только потом `FollowPath`. Угол `None` —
Spin не слать.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from rob_box_teleop.proximity import Body

Point = Tuple[float, float]
Pose = Tuple[float, float, float]

#: Шаг перебора курса, рад. На носу корпуса (~0.31 м) это ~1.5 см — мельче клетки costmap 0.05 м.
HEADING_STEP_RAD = 0.05
#: Сколько точек должны подтвердить стену по курсу: один луч-призрак не разворачивает.
WALL_MIN_POINTS = 2
#: Дуга разворота не должна пересекать точки скана. 0 — контур не задевает стену.
#: Это не порог стопа защиты (spin_stop_m): тот стопит уже крутящийся корпус, а здесь робот
#: ещё стоит и только выбирает, влезет ли короткий поворот. Ослаблять spin_stop_m нельзя.
ARC_CLEARANCE_M = 0.0
#: Смещение центра вращения от base_footprint назад по x, м. HIL 09.11 §10: в Gazebo
#: разворот идёт вокруг задней оси (−0.144, 0), base_footprint ходит по кругу R = 0.144 м.
#: Центр живого робота не мерен (там же) — пока берётся измеренный сим, не новое число.
#: None — крутить вокруг базы (проверка «влезает ли контур на месте»).
REAR_AXLE_X_M = -0.144
#: Первая точка пути дальше этого — уже «куда ехать», а не сама поза робота.
PATH_STEP_M = 0.05


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _world_to_body(pose: Pose, x: float, y: float) -> Point:
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - px, y - py
    return c * dx + s * dy, -s * dx + c * dy


def body_gap(body: Body, pose: Pose, points: Sequence[Point]) -> float:
    """Наименьший зазор корпуса в позе ``pose`` до точек, м. Точек нет — ``inf``."""
    best = math.inf
    for x, y in points:
        bx, by = _world_to_body(pose, x, y)
        best = min(best, body.gap(bx, by))
    return best


def facing_wall(
    body: Body,
    pose: Pose,
    points: Sequence[Point],
    clearance_m: float,
    min_points: int = WALL_MIN_POINTS,
) -> bool:
    """Нос упирается в стену: в коридоре хода ближе ``clearance_m`` не меньше ``min_points`` точек.

    Коридор — полоса корпуса по ширине впереди носа. Боковая стена тупика в коридор не входит:
    стоя лицом вдоль неё, робот не «лицом в стену».
    """
    if clearance_m <= 0.0 or min_points < 1:
        raise ValueError(f"нужно clearance_m > 0 и min_points ≥ 1, получено {clearance_m}, {min_points}")
    hits = 0
    for x, y in points:
        bx, by = _world_to_body(pose, x, y)
        if bx > body.x_max and body.y_min <= by <= body.y_max and body.gap(bx, by) < clearance_m:
            hits += 1
            if hits >= min_points:
                return True
    return False


def _first_ahead(pose: Pose, path: Sequence[Point]) -> Optional[Point]:
    """Первая точка пути, которая ещё впереди носа. Точка сзади — разворот её не догоняет."""
    px, py, yaw = pose
    fx, fy = math.cos(yaw), math.sin(yaw)
    for x, y in path:
        dx, dy = float(x) - px, float(y) - py
        if dx * dx + dy * dy < PATH_STEP_M * PATH_STEP_M:
            continue
        if dx * fx + dy * fy > 0.0:
            return float(x), float(y)
    return None


def _bearing(pose: Pose, point: Point) -> float:
    """Угол от курса до точки пути, рад. > 0 — точка слева."""
    _px, _py, yaw = pose
    return wrap_angle(math.atan2(point[1] - pose[1], point[0] - pose[0]) - yaw)


def _swept_pose(pose: Pose, spin: float, pivot_x: float) -> Pose:
    """Поза базы после поворота на ``spin`` вокруг точки корпуса ``pivot_x`` (0 — вокруг базы)."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    # Точка вращения в мире, затем база относительно неё на новом курсе.
    ox, oy = px + c * pivot_x, py + s * pivot_x
    ny = yaw + spin
    nc, ns = math.cos(ny), math.sin(ny)
    return ox - nc * pivot_x, oy - ns * pivot_x, ny


def _arc_holds(
    body: Body,
    pose: Pose,
    points: Sequence[Point],
    spin: float,
    clearance_m: float,
    step_rad: float,
    pivot_x: float,
) -> bool:
    """Каждый шаг поворота до ``spin`` держит зазор корпуса не меньше ``clearance_m``."""
    n = max(1, int(math.ceil(abs(spin) / step_rad)))
    for i in range(1, n + 1):
        if body_gap(body, _swept_pose(pose, spin * i / n, pivot_x), points) < clearance_m:
            return False
    return True


def free_spin(
    body: Body,
    pose: Pose,
    points: Sequence[Point],
    path: Sequence[Point],
    clearance_m: float,
    *,
    min_points: int = WALL_MIN_POINTS,
    step_rad: float = HEADING_STEP_RAD,
    pivot_x: float = REAR_AXLE_X_M,
) -> Optional[float]:
    """Кратчайший разворот на месте до курса, где нос свободен и путь впереди. ``None`` — не крутить.

    Все условия сразу: путь не пуст; нос упёрт в стену (зазор коридора хода меньше
    ``clearance_m``); есть поворот не длиннее 180°, на котором контур не пересекает
    точки скана, и после него первая точка пути лежит впереди носа (±90°). Бросок сразу
    на азимут пути не годится: нос ещё в перегородке, а путь уходит в щель сбоку —
    крутим только до щели. ``clearance_m`` на дугу не переносится: это порог «нос в стене»,
    не запас крутящегося корпуса (его держит защита, и здесь он не ослабляется).

    Уже свободный курс, пустой путь и дуга, которая задевает стену (тупик 1.1 м, полный
    оборот), возвращают ``None``. Фиксированный Spin туда не шлётся.
    """
    if step_rad <= 0.0:
        raise ValueError(f"step_rad > 0, получено {step_rad}")
    pts = tuple((float(x), float(y)) for x, y in points)
    goal = _first_ahead(pose, path)
    if goal is None or not facing_wall(body, pose, pts, clearance_m, min_points):
        return None
    toward = _bearing(pose, goal)
    sign = 1.0 if toward >= 0.0 else -1.0
    n = int(math.ceil(abs(toward) / step_rad))
    for k in range(1, n + 1):
        spin = wrap_angle(sign * k * step_rad)
        turned = _swept_pose(pose, spin, pivot_x)
        if not _arc_holds(body, pose, pts, spin, ARC_CLEARANCE_M, step_rad, pivot_x):
            return None
        if facing_wall(body, turned, pts, clearance_m, min_points):
            continue
        if abs(_bearing(turned, goal)) > math.pi / 2:
            continue
        return spin
    return None
