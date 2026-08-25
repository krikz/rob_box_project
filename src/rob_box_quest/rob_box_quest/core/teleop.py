"""TeleopController — чистая логика teleop с dead-man и emergency-stop.

Источник истины: docs/architecture/meta-quest-api.md §5 (teleop_twist),
docs/adr/0027 §3.3 (dead-man, watchdog, emergency B).

Контракт:
- consume(linear, angular, deadman, now) → Twist | None
  - deadman=False → None (отпустил grip → стоп)
  - (now - last_input_at) > DEADMAN_TIMEOUT_S → None (тишина → стоп)
  - emergency_stop() → все consume до reset() возвращают None
- reset() — снимает emergency lock

Используется в quest_node.py: TeleopController → publish cmd_vel_quest.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# Dead-man timeout (meta-quest-api.md §5 throttle + ADR §3.3):
# "если нет па > 500 мс → safe stop".
DEADMAN_TIMEOUT_S: float = 0.5

# Верхний предел скорости — safety belt. По архитектуре twist_mux сам
# делает timeout, но это защита от клиента, который шлёт запредельные
# значения (или unit-mismatch в JSON).
MAX_LINEAR_M_S: float = 2.0
MAX_ANGULAR_RAD_S: float = 3.0


@dataclass
class _TwistLike:
    """Минимальный Twist-контейнер, не зависящий от geometry_msgs.

    Phase 1.3: quest_node.py конвертирует в geometry_msgs.msg.Twist на publish.
    Тесты проверяют структуру без rclpy.
    """

    linear_x: float = 0.0
    angular_z: float = 0.0


class TeleopController:
    """Состояние одного teleop-источника (WebXR/Meta Quest клиента).

    Не thread-safe — все мутации из event-loop rclpy.
    """

    __slots__ = (
        "_last_input_monotonic",
        "_emergency",
        "_twist",
    )

    def __init__(self) -> None:
        self._last_input_monotonic: Optional[float] = None
        self._emergency: bool = False
        self._twist = _TwistLike()

    @property
    def is_emergency(self) -> bool:
        return self._emergency

    @property
    def last_twist(self) -> _TwistLike:
        return _TwistLike(
            linear_x=self._twist.linear_x,
            angular_z=self._twist.angular_z,
        )

    def consume(
        self,
        linear: float,
        angular: float,
        deadman: bool,
        now_monotonic: float,
    ) -> Optional[_TwistLike]:
        """Обработать teleop-фрейм. Возвращает Twist для публикации или None.

        None означает "не публикуй ничего" — robot safe stop.
        """
        if self._emergency:
            return None
        if not deadman:
            # Grip отпущен → safe stop, last_input НЕ обновляем (тишина тоже стоп).
            self._last_input_monotonic = None
            self._twist = _TwistLike()
            return None
        # Обновить last_input.
        self._last_input_monotonic = now_monotonic
        # Clamp к safety belt.
        linear_c = max(-MAX_LINEAR_M_S, min(MAX_LINEAR_M_S, float(linear)))
        angular_c = max(-MAX_ANGULAR_RAD_S, min(MAX_ANGULAR_RAD_S, float(angular)))
        self._twist = _TwistLike(linear_x=linear_c, angular_z=angular_c)
        return _TwistLike(linear_x=linear_c, angular_z=angular_c)

    def tick(self, now_monotonic: float) -> Optional[_TwistLike]:
        """Periodic tick — повторяем последний Twist пока свежо,
        иначе None (dead-man). Вызывается из quest_node publishing loop."""
        if self._emergency:
            return None
        if self._last_input_monotonic is None:
            return None
        age = now_monotonic - self._last_input_monotonic
        if age > DEADMAN_TIMEOUT_S:
            # Тишина → стоп. last_input остаётся, чтобы grace-период
            # не сбрасывался, но возвращаем None.
            return None
        return _TwistLike(
            linear_x=self._twist.linear_x,
            angular_z=self._twist.angular_z,
        )

    def emergency_stop(self) -> None:
        """Зафиксировать emergency-стоп. consume/tick → None до reset()."""
        self._emergency = True
        self._last_input_monotonic = None
        self._twist = _TwistLike()

    def reset(self) -> None:
        """Снять emergency lock (после operator ack)."""
        self._emergency = False
        self._last_input_monotonic = None
        self._twist = _TwistLike()
