"""Поза робота по видимому тегу с известной позой на карте (#4346, вариант B; прототип).

Чистый Python, без rclpy. Тонкая нода — ``tag_relocalization_node`` (выключена по умолчанию).

Задача (#4346): после перезапуска rtabmap ставит робота в последнюю позу из базы
(``RGBD/StartAtOrigin=false``). Если робота сдвинули или база чужая, исправиться ему нечем.
Тег с измеренной позой на карте даёт независимую оценку:

    T_map_base = T_map_tag · (T_base_tag)⁻¹

где T_base_tag — поза тега в ``base_footprint`` (TF ``base_footprint → tag36h11:<id>``, её
публикует apriltag_node), T_map_tag — известная поза тега на карте (конфиг). Результат
проецируется на плоскость (x, y, курс).

Решение — в коде (ADR-0148), не в LLM: :class:`TagRelocalizer` копит ``min_consistent``
согласных оценок подряд и только тогда говорит
  * ``correct`` — оценка rtabmap расходится с тегом больше ``correct_if_m``/``correct_if_deg``:
    послать ``/rtabmap/initialpose`` с позой по тегу;
  * ``agree`` — rtabmap и тег согласны (это и есть проверка «правильно ли локализовался»).

Кадр тега apriltag_ros: X вправо, Y вниз, Z — в тег (от камеры), если смотреть на тег
анфас [проверено в Gazebo: docs/research/2026-11-09-sim-localization-restart.md].
:func:`tag_pose_on_wall` строит T_map_tag из того, что можно измерить рулеткой:
центр тега (x, y, z) и куда смотрит лицо тега (``normal_yaw``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

Quat = Tuple[float, float, float, float]  # x, y, z, w
Pose2 = Tuple[float, float, float]  # x, y, yaw


@dataclass(frozen=True)
class Transform:
    """Жёсткое преобразование: перенос (x, y, z) и кватернион (qx, qy, qz, qw)."""

    x: float
    y: float
    z: float
    q: Quat = (0.0, 0.0, 0.0, 1.0)

    def rotate(self, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        qx, qy, qz, qw = self.q
        # v' = q v q*, через t = 2 q×v
        tx = 2 * (qy * v[2] - qz * v[1])
        ty = 2 * (qz * v[0] - qx * v[2])
        tz = 2 * (qx * v[1] - qy * v[0])
        return (
            v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx),
        )

    def compose(self, other: "Transform") -> "Transform":
        """self · other."""
        ox, oy, oz = self.rotate((other.x, other.y, other.z))
        return Transform(self.x + ox, self.y + oy, self.z + oz, _qmul(self.q, other.q))

    def inverse(self) -> "Transform":
        qi = (-self.q[0], -self.q[1], -self.q[2], self.q[3])
        inv_rot = Transform(0.0, 0.0, 0.0, qi)
        px, py, pz = inv_rot.rotate((self.x, self.y, self.z))
        return Transform(-px, -py, -pz, qi)

    def yaw(self) -> float:
        qx, qy, qz, qw = self.q
        return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _qmul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_from_matrix(m: Sequence[Sequence[float]]) -> Quat:
    """Кватернион из матрицы поворота 3×3 (столбцы — оси дочернего кадра в родителе)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def tag_pose_on_wall(x: float, y: float, z: float, normal_yaw: float) -> Transform:
    """T_map_tag тега на вертикальной стене: центр (x, y, z), лицо смотрит в курс ``normal_yaw``.

    Оси кадра тега (apriltag_ros, смотрим на тег анфас): Z — в тег (−нормаль), Y — вниз,
    X — вправо для смотрящего (= Y × Z).
    """
    nx, ny = math.cos(normal_yaw), math.sin(normal_yaw)
    ez = (-nx, -ny, 0.0)
    ey = (0.0, 0.0, -1.0)
    ex = (ey[1] * ez[2] - ey[2] * ez[1], ey[2] * ez[0] - ey[0] * ez[2], ey[0] * ez[1] - ey[1] * ez[0])
    m = [[ex[i], ey[i], ez[i]] for i in range(3)]
    return Transform(x, y, z, _quat_from_matrix(m))


def base_pose_from_tag(map_tag: Transform, base_tag: Transform) -> Pose2:
    """(x, y, курс) ``base_footprint`` в ``map`` по одной детекции."""
    t = map_tag.compose(base_tag.inverse())
    return (t.x, t.y, t.yaw())


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class RelocalizerConfig:
    tags: Dict[int, Transform]
    max_range_m: float = 2.5  # дальше детекция шумит (тег 0.119 м)
    min_consistent: int = 5  # подряд согласных оценок (~1 с при 5 Гц)
    window_s: float = 3.0
    spread_m: float = 0.05
    spread_deg: float = 3.0
    correct_if_m: float = 0.2
    correct_if_deg: float = 8.0
    cooldown_s: float = 10.0


@dataclass(frozen=True)
class Decision:
    kind: str  # correct | agree
    tag_id: int
    pose: Pose2
    error_m: Optional[float]  # расхождение с оценкой rtabmap (None — оценки нет)
    error_deg: Optional[float]


@dataclass
class TagRelocalizer:
    config: RelocalizerConfig
    _recent: List[Tuple[float, int, Pose2]] = field(default_factory=list)
    _last_correct_t: Optional[float] = None

    def observe(self, t: float, tag_id: int, base_tag: Transform, estimate: Optional[Pose2]) -> Optional[Decision]:
        """Одна детекция в момент ``t``. Возвращает решение, когда набралось согласных оценок."""
        cfg = self.config
        if tag_id not in cfg.tags:
            return None
        if math.hypot(base_tag.x, base_tag.y) > cfg.max_range_m:
            return None
        pose = base_pose_from_tag(cfg.tags[tag_id], base_tag)
        self._recent = [r for r in self._recent if t - r[0] <= cfg.window_s] + [(t, tag_id, pose)]
        mean = self._consistent_mean()
        if mean is None:
            return None
        err_m, err_deg = _discrepancy(estimate, mean)
        if err_m is not None and err_m <= cfg.correct_if_m and err_deg <= cfg.correct_if_deg:
            return Decision("agree", tag_id, mean, err_m, err_deg)
        if self._last_correct_t is not None and t - self._last_correct_t < cfg.cooldown_s:
            return None
        self._last_correct_t = t
        self._recent.clear()
        return Decision("correct", tag_id, mean, err_m, err_deg)

    def _consistent_mean(self) -> Optional[Pose2]:
        """Средняя поза последних ``min_consistent`` оценок, если они согласны; иначе None."""
        cfg = self.config
        if len(self._recent) < cfg.min_consistent:
            return None
        last = [p for _, _, p in self._recent[-cfg.min_consistent :]]
        mean = _mean_pose(last)
        if any(math.hypot(p[0] - mean[0], p[1] - mean[1]) > cfg.spread_m for p in last):
            return None
        if any(abs(math.degrees(_wrap(p[2] - mean[2]))) > cfg.spread_deg for p in last):
            return None
        return mean


def _discrepancy(estimate: Optional[Pose2], pose: Pose2) -> Tuple[Optional[float], Optional[float]]:
    """(м, градусы) между оценкой rtabmap и позой по тегу; (None, None) — оценки нет."""
    if estimate is None:
        return None, None
    return (
        math.hypot(estimate[0] - pose[0], estimate[1] - pose[1]),
        abs(math.degrees(_wrap(estimate[2] - pose[2]))),
    )


def _mean_pose(poses: Sequence[Pose2]) -> Pose2:
    n = len(poses)
    s = sum(math.sin(p[2]) for p in poses)
    c = sum(math.cos(p[2]) for p in poses)
    return (sum(p[0] for p in poses) / n, sum(p[1] for p in poses) / n, math.atan2(s, c))


def parse_tags(spec: Sequence[Sequence[float]]) -> Dict[int, Transform]:
    """Конфиг тегов: строки ``[id, x, y, z, normal_yaw]`` (рулетка + направление стены)."""
    out: Dict[int, Transform] = {}
    for row in spec:
        if len(row) != 5:
            raise ValueError(f"тег {row!r}: нужно [id, x, y, z, normal_yaw]")
        tag_id = int(row[0])
        if tag_id in out:
            raise ValueError(f"тег {tag_id} указан дважды")
        out[tag_id] = tag_pose_on_wall(float(row[1]), float(row[2]), float(row[3]), float(row[4]))
    return out
