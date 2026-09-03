"""StateAggregator — собирает ``/avatar/state`` из потоков (AV-6, ADR-0028 §4.3).

Чисто-Python класс, без зависимости от rclpy — это позволяет гонять
юнит-тесты на CI без ROS 2 в окружении. ROS-нода supervisor_node.py
держит ссылку на :class:`StateAggregator` и в колбэках подписок
пробрасывает свежие данные через ``update_*`` методы; публикация
``/avatar/state`` сериализует текущий snapshot в msgpack поверх
``std_msgs/String`` (см. ADR-0028 §4.3 и AV-5 — там будет полный IDL).

Источники истины:
- ADR-0028 §4.3 (подписки: ``/odom``, ``/device/snapshot``,
  ``/voice/dialogue/state`` — НЕ ``/voice/state``, расхождение ADR-0027 #2).
- SYSTEM_OVERVIEW.md §5.4 (контракт ``/avatar/state``).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class AvatarState:
    """Snapshot, который публикуется в ``/avatar/state`` (msgpack-encoded).

    Структура intentionally совпадает с ADR-0028 §6 Q4 — поле
    ``last_event`` несёт ``dead_man_trips_total{client_id}`` счётчик.
    Полный IDL появится в AV-5; пока Phase 1 мониторинга хватает
    msgpack-dict, чтобы не плодить IDL до Phase 2.
    """

    ts_ms: int
    pose_xy: Optional[tuple] = None  # (x, y) из /odom
    battery_pct: Optional[float] = None  # из /device/snapshot
    voice_state: Optional[str] = None  # из /voice/dialogue/state
    last_event: Dict[str, Any] = field(default_factory=dict)
    dead_man_trips_total: Dict[str, int] = field(default_factory=dict)

    def to_msgpack_dict(self) -> Dict[str, Any]:
        """Сериализация в dict для msgpack (Phase 1)."""
        return {
            "ts_ms": self.ts_ms,
            "pose_xy": list(self.pose_xy) if self.pose_xy is not None else None,
            "battery_pct": self.battery_pct,
            "voice_state": self.voice_state,
            "last_event": self.last_event,
            "dead_man_trips_total": dict(self.dead_man_trips_total),
        }


class StateAggregator:
    """Склеивает ``/avatar/state`` из подписок.

    Пример использования в supervisor_node::

        agg = StateAggregator()
        agg.update_odom(x=1.0, y=2.0)
        agg.update_device_snapshot(battery_pct=87.5)
        agg.update_voice_state("listening")
        snapshot = agg.snapshot()
        publisher.publish(String(data=msgpack.packb(snapshot.to_msgpack_dict())))

    Дизайн:
    - Каждый ``update_*`` принимает сырые поля (не ROS-сообщения) — это
      держит класс переносимым между ROS 2 (rclpy), Zenoh-bridge и
      голыми юнит-тестами.
    - :py:meth:`snapshot` возвращает неизменяемый :class:`AvatarState`,
      который можно сериализовать и опубликовать.
    """

    def __init__(self) -> None:
        self._pose_xy: Optional[tuple] = None
        self._battery_pct: Optional[float] = None
        self._voice_state: Optional[str] = None
        self._last_event: Dict[str, Any] = {}
        self._dead_man: Dict[str, int] = {}

    # ── inputs ────────────────────────────────────────────────────────
    def update_odom(self, x: float, y: float) -> None:
        """Обработать ``/odom`` (только x, y — остальное Phase 2)."""
        self._pose_xy = (float(x), float(y))

    def update_device_snapshot(self, battery_pct: Optional[float] = None) -> None:
        """Обработать ``/device/snapshot`` (только battery_pct — Phase 1)."""
        if battery_pct is not None:
            self._battery_pct = float(battery_pct)

    def update_voice_state(self, voice_state: Optional[str]) -> None:
        """Обработать ``/voice/dialogue/state`` (см. ADR-0027 #2)."""
        self._voice_state = voice_state

    # ── dead-man / events ─────────────────────────────────────────────
    def record_dead_man_trip(self, client_id: str) -> int:
        """Инкрементировать ``dead_man_trips_total{client_id}``.

        Возвращает новое значение счётчика — удобно для логов и тестов.
        Попадает в :py:attr:`AvatarState.last_event` и в отдельное поле
        ``dead_man_trips_total`` (ADR-0028 §6 Q4, Phase 1 metric).
        """
        cid = str(client_id)
        new_value = self._dead_man.get(cid, 0) + 1
        self._dead_man[cid] = new_value
        self._last_event = {
            "kind": "dead_man_trip",
            "client_id": cid,
            "trip_count": new_value,
        }
        return new_value

    def last_event_as_avatar_event(self, now_ms: int) -> Optional[Any]:
        """Return the last meaningful event as a wire-format ``AvatarEvent``.

        AV-14 (issue #1906) — aggregator stays a Phase-1 metric holder
        (``pose_xy`` / ``battery_pct`` / ``voice_state`` /
        ``dead_man_trips_total``), but it no longer drives the
        ``/avatar/state`` schema — ``encode_for_ros_string`` builds that
        from ``core.state.AvatarState``. This helper bridges the two
        layers cleanly: a single ``AvatarEvent`` if there is recorded
        activity (right now: only ``dead_man_trip`` events), ``None``
        otherwise.

        Imported as ``Any`` to avoid a hard coupling between this module
        and :class:`core.state.AvatarEvent` (which lives in a sibling
        module imported by ``supervisor_node``, never by ``aggregator``).
        """
        if not self._last_event:
            return None
        if self._last_event.get("kind") != "dead_man_trip":
            return None
        cid = str(self._last_event.get("client_id", ""))
        # Адресный контракт живёт в core.state.AvatarEvent (см. модуль
        # core.state): kind/args — общие поля, которые мы заполняем из
        # нашей dict-формы. Импортируем лениво, чтобы core/aggregator.py
        # оставался импортируем без побочных эффектов (для unit-тестов
        # в test_aggregator.py, где AvatarEvent не нужен).
        from rob_box_supervisor.core.state import AvatarEvent as _AE  # noqa: PLC0415

        return _AE(
            timestamp_ms=int(now_ms),
            client_id=cid,
            kind="dead_man_trip",
            args={"trip_count": int(self._last_event.get("trip_count", 0))},
        )

    def dead_man_count(self, client_id: str) -> int:
        """Текущее значение счётчика (0 если ещё не было трипов)."""
        return self._dead_man.get(str(client_id), 0)

    # ── snapshot ──────────────────────────────────────────────────────
    def snapshot(self) -> AvatarState:
        """Собрать текущий :class:`AvatarState` (для публикации)."""
        return AvatarState(
            ts_ms=int(time.time() * 1000),
            pose_xy=self._pose_xy,
            battery_pct=self._battery_pct,
            voice_state=self._voice_state,
            last_event=dict(self._last_event),
            dead_man_trips_total=dict(self._dead_man),
        )
