"""Правдоподобие локализации перед словом «приехал» (#4495, ADR-0148).

HIL Depot 06.11: Nav2 вернул ``SUCCEEDED``, а робот стоял в 3.5–7.5 м от цели. Nav2 судит
о прибытии по позе в кадре ``map``, а поза эта уже не опиралась на лидар:

* ``icp_odometry`` (rtabmap) на быстром развороте не сошлась и ушла в ``lost``, дальше поза
  ехала на колёсной одометрии. Потерянный кадр rtabmap публикует в ``/odom`` с ковариацией
  9999, кадр сброса одометрии — 19998 (замер bag 06.11: без сброса 8309 кадров ``lost`` из 8946
  подряд; со сбросом ``Odom/ResetCountdown=1`` — пары «потерян + сброс», 2 кадра на случай);
* ``map → icp_odom`` прыгал на 0.5–2.7 м за секунду (``/rtabmap/info``, ``Loop/MapToOdom_*``):
  rtabmap поправлял граф по уже уехавшим позам. В здоровой части прогона скачков больше 0.2 м
  не было.

Здесь решение «можно ли верить позе» принимает код по этим двум признакам; LLM только
произносит готовую фразу. Модуль без ROS: адаптер подписок — ``rob_box_mcp_tools.localization_watch``.
Пороги — только здесь (:data:`LIMITS`).
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

#: Поза надёжна: за поездку ни потерь одометрии сверх допуска, ни скачков карты.
LOC_CONFIDENT = "confident"
#: Сигналов за поездку не пришло (поездка короче кадра одометрии или топики не дошли): не проверено.
LOC_UNCHECKED = "unchecked"
#: ICP-одометрия теряла лидарную привязку дольше допуска.
LOC_ODOM_LOST = "odom_lost"
#: Поправка карты ``map → odom`` прыгнула больше допуска: поза до скачка была неверной.
LOC_MAP_JUMP = "map_jump"


@dataclass(frozen=True)
class LocalizationLimits:
    """Пороги правдоподобия. Значения — из замеров HIL Depot 06.11 (#4495), на живом роботе не мерены."""

    #: rtabmap публикует потерянный кадр одометрии с ковариацией позы 9999, кадр сброса — 19998 [bag 06.11].
    odom_lost_covariance: float = 9999.0
    #: Самая длинная серия таких кадров подряд, которую терпим (10 Гц: 0.5 с). Со сбросом одометрии
    #: серия — 2 кадра (12 целей 06.11, ошибка по правде 0.17–0.34 м); без сброса — 295–5996 кадров.
    max_lost_run: int = 5
    #: Доля таких кадров за поездку. Здоровые цели 06.11 — до 0.7 %, потерянные — 98–100 %.
    max_lost_share: float = 0.05
    #: Скачок поправки карты между соседними ``/rtabmap/info`` (1 Гц). В здоровых прогонах 06.11 —
    #: меньше 0.2 м, после потери одометрии — 0.5–2.7 м.
    max_map_jump_m: float = 1.0
    max_map_jump_deg: float = 10.0


LIMITS = LocalizationLimits()


@dataclass(frozen=True)
class LocalizationVerdict:
    """Итог проверки поездки: ``status`` — машинный признак, ``detail`` — числа для логов и данных тула."""

    status: str
    detail: Dict[str, Any] = field(default_factory=dict)

    @property
    def trusted(self) -> bool:
        """Можно ли сказать «приехал». ``unchecked`` не блокирует: проверить было нечем (см. модуль)."""
        return self.status in (LOC_CONFIDENT, LOC_UNCHECKED)


def _angle_deg(a: float, b: float) -> float:
    return abs((b - a + 180.0) % 360.0 - 180.0)


class LocalizationTrack:
    """Сигналы локализации за одну поездку. Колбэки подписок приходят из потоков исполнителя — под замком."""

    def __init__(self, limits: LocalizationLimits = LIMITS) -> None:
        self._limits = limits
        self._lock = threading.Lock()
        self._odom_frames = 0
        self._lost_frames = 0
        self._run = 0
        self._longest_run = 0
        self._last_map_to_odom: Optional[Tuple[float, float, float]] = None
        self._max_jump_m = 0.0
        self._max_jump_deg = 0.0
        self._jumps = 0

    def on_odom_covariance(self, covariance_xx: float) -> None:
        """Кадр ICP-одометрии: ``pose.covariance[0]`` из ``/odom``."""
        lost = not math.isfinite(covariance_xx) or covariance_xx >= self._limits.odom_lost_covariance
        with self._lock:
            self._odom_frames += 1
            self._lost_frames += int(lost)
            self._run = self._run + 1 if lost else 0
            self._longest_run = max(self._longest_run, self._run)

    def on_map_to_odom(self, x: float, y: float, yaw_deg: float) -> None:
        """Поправка карты ``map → odom`` из ``/rtabmap/info`` (``Loop/MapToOdom_x/m``, ``_y/m``, ``_yaw/deg``)."""
        with self._lock:
            prev = self._last_map_to_odom
            self._last_map_to_odom = (x, y, yaw_deg)
            if prev is None:
                return
            jump_m = math.hypot(x - prev[0], y - prev[1])
            jump_deg = _angle_deg(prev[2], yaw_deg)
            self._max_jump_m = max(self._max_jump_m, jump_m)
            self._max_jump_deg = max(self._max_jump_deg, jump_deg)
            if jump_m > self._limits.max_map_jump_m or jump_deg > self._limits.max_map_jump_deg:
                self._jumps += 1

    def _odom_lost(self) -> bool:
        share = self._lost_frames / self._odom_frames if self._odom_frames else 0.0
        return self._longest_run > self._limits.max_lost_run or share > self._limits.max_lost_share

    def verdict(self) -> LocalizationVerdict:
        """Итог поездки. Потеря одометрии важнее скачка: она и есть причина ухода позы."""
        with self._lock:
            detail = {
                "odom_frames": self._odom_frames,
                "lost_frames": self._lost_frames,
                "longest_lost_run": self._longest_run,
                "map_jumps": self._jumps,
                "max_map_jump_m": round(self._max_jump_m, 3),
                "max_map_jump_deg": round(self._max_jump_deg, 1),
            }
            if self._odom_frames == 0 and self._last_map_to_odom is None:
                return LocalizationVerdict(LOC_UNCHECKED, detail)
            if self._odom_lost():
                return LocalizationVerdict(LOC_ODOM_LOST, detail)
            if self._jumps:
                return LocalizationVerdict(LOC_MAP_JUMP, detail)
            return LocalizationVerdict(LOC_CONFIDENT, detail)


def map_to_odom_from_stats(keys, values) -> Optional[Tuple[float, float, float]]:
    """``(x, y, yaw_deg)`` поправки карты из статистики ``rtabmap_msgs/Info`` или ``None``, если её нет."""
    stats = dict(zip(keys, values))
    try:
        return (
            float(stats["Loop/MapToOdom_x/m"]),
            float(stats["Loop/MapToOdom_y/m"]),
            float(stats["Loop/MapToOdom_yaw/deg"]),
        )
    except (KeyError, TypeError, ValueError):
        return None
