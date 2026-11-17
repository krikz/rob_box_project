"""motion_state.py - едет ли робот: по скорости одометрии, а не по одному сэмплу (issue #4352).

Раньше ``context_aggregator_node`` ставил ``is_moving = |vx| > 0.01 or |wz| > 0.01`` по последнему
сообщению ``/odom``. Одометрию на роботе публикует ``rtabmap/icp_odometry``, и у стоящего робота
она шумит (замер 04.11.2026, робот на кирпичах): ``linear.x`` от -0.044 до +0.039 м/с,
``angular.z`` от -0.021 до +0.019 рад/с. Порог 0.01 этот шум не отсекал, и стоящий робот
«ехал».

Здесь скорость усредняется по окну со знаком. Шум стоящего робота меняет знак и в среднем гасится,
настоящее движение (езда или разворот на месте) держит знак и остаётся. Пороги выше шума и ниже
медленной езды.

Чистый Python, без rclpy: тестируется без ROS.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

#: Окно усреднения скорости, секунды.
WINDOW_S = 1.0
#: Порог линейной скорости, м/с. Шум ICP-одометрии стоящего робота: до 0.044 по модулю, среднее ~0.
LINEAR_THRESHOLD_MPS = 0.05
#: Порог угловой скорости, рад/с. Шум стоящего робота: до 0.021 по модулю.
ANGULAR_THRESHOLD_RPS = 0.05


class MotionEstimator:
    """Скользящее окно сэмплов скорости из ``/odom``."""

    def __init__(self, window_s: float = WINDOW_S) -> None:
        """Создать пустое окно длиной ``window_s`` секунд."""
        self._window_s = window_s
        self._samples: Deque[Tuple[float, float, float, float]] = deque()

    def add(self, t: float, vx: float, vy: float, wz: float) -> None:
        """Добавить сэмпл скорости (``twist.twist`` одометрии) на момент ``t``."""
        self._samples.append((t, vx, vy, wz))
        self._prune(t)

    def is_moving(self, now: float) -> Optional[bool]:
        """Едет ли робот. ``None`` - одометрии за окно не было, ответа нет."""
        self._prune(now)
        if not self._samples:
            return None
        n = len(self._samples)
        vx = sum(s[1] for s in self._samples) / n
        vy = sum(s[2] for s in self._samples) / n
        wz = sum(s[3] for s in self._samples) / n
        return math.hypot(vx, vy) > LINEAR_THRESHOLD_MPS or abs(wz) > ANGULAR_THRESHOLD_RPS

    def _prune(self, now: float) -> None:
        cutoff = now - self._window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
