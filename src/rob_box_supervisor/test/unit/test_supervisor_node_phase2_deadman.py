"""TDD-тесты для Phase 2 active-dead-man watchdog (t_6fa213cc, AV-6 Phase 2).

ADR-0028:
- §4.4 S10: dead-man 500 ms (клиент, держащий teleop_floor, шлёт
  heartbeat ≥10 Гц; без heartbeat > 500 ms — супервизор снимает floor).
- §6 Q4: «Phase 1 метрика: собирать dead_man_trips_total{client_id}».

Acceptance criteria для этой карточки (issue t_6fa213cc):
  A. self._dead_man_timer создан в __init__ (period=0.1 s).
  B. Heartbeat subscription на /avatar/heartbeat (msgpack-style JSON
     fallback для CI без rclpy).
  C. twist_mux lock side-effect:
     - при acquire teleop_floor (active) → publish True на /teleop_lock;
     - при release teleop_floor         → publish False на /teleop_lock;
     - voice_floor НЕ трогает /teleop_lock (только teleop).
  D. _dead_man_tick():
     - heartbeat-fresh → no-op (LockManager.holder() ещё жив);
     - heartbeat-stale > DEAD_MAN_TIMEOUT_MS → release FSM-event,
       DeadManCounter.trip(client_id) инкрементируется, /avatar/state
       публикуется со свежим mode=off.
  E. Twist_mux lock side-effect ВЫКЛЮЧЕН в monitor mode (S12).
  F. Backward-compat: passive dead-man (через next-acquire) продолжает
     работать (не ломаем существующий TestPhase2Acceptance6DeadMan).

Эти тесты — RED-фаза TDD. Они падают до тех пор, пока в
supervisor_node.py не появится:

  - ``self._dead_man_timer = self.create_timer(0.1, self._dead_man_tick)``
  - ``self._heartbeat_subscription`` + ``self._on_heartbeat_msg(...)``
  - ``self._teleop_lock_pub = self.create_publisher(Bool, "/teleop_lock", ...)``
  - методы ``_apply_teleop_lock(acquire: bool)`` + ``_dead_man_tick()``

См. также:
  - core/dead_man.py — DeadManCounter (метрика dead_man_trips_total).
  - core/locks.py — DEAD_MAN_TIMEOUT_MS = 500.
  - core/fsm.py — EVENT_FORCE_OFF (escape hatch ``* → off``).
"""

from __future__ import annotations

import json
import unittest
from typing import Any, Dict
from unittest.mock import MagicMock

from rob_box_supervisor.core.fsm import Mode, ModeManager
from rob_box_supervisor.core.locks import (
    DEAD_MAN_TIMEOUT_MS,
    FLOOR_TELEOP,
    FLOOR_VOICE,
    LockManager,
)
from rob_box_supervisor.supervisor_node import (
    MONITOR_MODE_REASON,
    TELEOP_LOCK_TOPIC,
    AvatarSupervisor,
)


# === helpers =========================================================


def _activate(node: AvatarSupervisor) -> None:
    """Перевести ноду в active-режим (Phase 2)."""
    node._mode = "active"


def _make_request(client_id: str = "", floor: str = "") -> MagicMock:
    """Создать fake Trigger.Request с convention-полем .data."""
    req = MagicMock()
    req.data = json.dumps({"client_id": client_id, "floor": floor})
    return req


def _make_response() -> MagicMock:
    resp = MagicMock()
    resp.success = False
    resp.message = ""
    return resp


def _call_active_service(
    node: AvatarSupervisor,
    svc_name: str,
    client_id: str,
    floor: str = "",
) -> Dict[str, Any]:
    """Вызвать AcquireFloor/ReleaseFloor и распарсить response.message."""
    svc = next(s for s in node._services if s.name == svc_name)
    req = _make_request(client_id=client_id, floor=floor)
    resp = _make_response()
    svc.callback(req, resp)
    body: Dict[str, Any] = {"success": bool(resp.success)}
    if resp.message:
        try:
            body.update(json.loads(resp.message))
        except (ValueError, TypeError):
            body["raw"] = resp.message
    return body


def _install_fake_clock(node: AvatarSupervisor, start_ms: int = 1_000_000) -> Any:
    """Заменить clock у LockManager/FSM И node._now_ms() на ручной (для детерминизма)."""
    clock_ref = [start_ms]

    def fake_clock() -> int:
        return clock_ref[0]

    # Инициируем manager-ов (если ещё не) и подменяем clock.
    node._ensure_active_handlers()
    node._lock_manager = LockManager(clock=fake_clock)
    node._mode_manager = ModeManager(clock=fake_clock)
    # Также подменяем wall-clock ``_now_ms`` чтобы supervisor_node видел
    # то же «виртуальное время», что и LockManager (в проде оба бегут от
    # time.time(); в тесте мы контролируем оба через clock_ref).
    node._now_ms = lambda: clock_ref[0]  # type: ignore[method-assign]
    return clock_ref


# === Acceptance A: timer создан ======================================


class TestDeadManTimerCreation(unittest.TestCase):
    """Acceptance A: нода в __init__ создаёт dead_man timer (period ~100 ms)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_dead_man_timer_is_registered(self) -> None:
        """rclpy.create_timer(0.1, self._dead_man_tick) был вызван."""
        timer_periods = [t.period for t in self.node._timers]
        self.assertTrue(
            any(abs(p - 0.1) < 1e-6 for p in timer_periods),
            msg=f"ожидаем timer с period=0.1, есть {timer_periods}",
        )


# === Acceptance B: heartbeat subscription ============================


class TestHeartbeatSubscription(unittest.TestCase):
    """Acceptance B: подписка на /avatar/heartbeat в __init__."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_heartbeat_subscription_topic(self) -> None:
        """Есть subscription на /avatar/heartbeat."""
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(
            "/avatar/heartbeat",
            topics,
            msg=f"/avatar/heartbeat subscription отсутствует; есть {topics}",
        )


# === Acceptance C: twist_mux lock side-effect ========================


class TestTwistMuxLockSideEffect(unittest.TestCase):
    """Acceptance C: при acquire/release teleop_floor публикуется /teleop_lock Bool."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        _activate(self.node)
        # Инициируем active-handlers явно (lazy init работает и на
        # первый service call; для теста publisher_exists нужно
        # триггернуть это сразу — вызываем ``_ensure_active_handlers``
        # через любой active service call).
        _call_active_service(self.node, "acquire_floor", "telegram", FLOOR_VOICE)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_teleop_lock_publisher_exists(self) -> None:
        """Publisher на TELEOP_LOCK_TOPIC зарегистрирован в active-режиме."""
        self.assertIn(
            TELEOP_LOCK_TOPIC,
            self.node._publishers,
            msg=f"Publisher на {TELEOP_LOCK_TOPIC!r} должен быть создан",
        )

    def test_acquire_teleop_publishes_true(self) -> None:
        """AcquireFloor(teleop) → publish True на /teleop_lock."""
        pub = self.node._publishers.get(TELEOP_LOCK_TOPIC)
        self.assertIsNotNone(pub, "teleop_lock publisher missing")
        before = len(pub.published)
        _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        self.assertGreater(
            len(pub.published),
            before,
            msg="AcquireFloor(teleop) должен публиковать в /teleop_lock",
        )
        last = pub.published[-1]
        self.assertTrue(
            bool(last.data),
            msg=f"После acquire teleop должен быть True, got {last.data!r}",
        )

    def test_release_teleop_publishes_false(self) -> None:
        """ReleaseFloor(teleop) после acquire → publish False."""
        pub = self.node._publishers.get(TELEOP_LOCK_TOPIC)
        self.assertIsNotNone(pub)
        _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        before = len(pub.published)
        _call_active_service(self.node, "release_floor", "quest", FLOOR_TELEOP)
        self.assertGreater(
            len(pub.published),
            before,
            msg="ReleaseFloor(teleop) должен публиковать в /teleop_lock",
        )
        last = pub.published[-1]
        self.assertFalse(
            bool(last.data),
            msg=f"После release teleop должен быть False, got {last.data!r}",
        )

    def test_voice_floor_does_not_publish_teleop_lock(self) -> None:
        """AcquireFloor(voice) НЕ трогает /teleop_lock (только teleop влияет)."""
        pub = self.node._publishers.get(TELEOP_LOCK_TOPIC)
        self.assertIsNotNone(pub)
        before = len(pub.published)
        # Acquire telegram/voice (уже был в setUp, но acquire 2-й раз — no-op).
        _call_active_service(self.node, "acquire_floor", "telegram", FLOOR_VOICE)
        self.assertEqual(
            len(pub.published),
            before,
            msg="AcquireFloor(voice) НЕ должен публиковать /teleop_lock",
        )


# === Acceptance D: _dead_man_tick() ===================================


class TestDeadManTick(unittest.TestCase):
    """Acceptance D: watchdog-timer явно снимает floor при timeout и
    инкрементирует DeadManCounter."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        _activate(self.node)
        self._clock_ms = _install_fake_clock(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_fresh_heartbeat_no_op(self) -> None:
        """Если floor не expired — tick не меняет state."""
        _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        before_mode = self.node._mode_manager.mode
        before_holder = self.node._lock_manager.holder(FLOOR_TELEOP)
        # Прокрутим < DEAD_MAN_TIMEOUT_MS — tick должен быть no-op.
        self._clock_ms[0] += DEAD_MAN_TIMEOUT_MS - 10
        self.node._dead_man_tick()
        self.assertEqual(self.node._mode_manager.mode, before_mode)
        self.assertEqual(self.node._lock_manager.holder(FLOOR_TELEOP), before_holder)

    def test_stale_heartbeat_releases_and_trips_counter(self) -> None:
        """Heartbeat > DEAD_MAN_TIMEOUT_MS → release FSM-event + counter++."""
        _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        before_count = self.node._dead_man.count("quest")
        # Программируем time-travel за пределы DEAD_MAN_TIMEOUT_MS.
        self._clock_ms[0] += DEAD_MAN_TIMEOUT_MS + 1
        self.node._dead_man_tick()

        # Holder должен быть освобождён.
        self.assertIsNone(
            self.node._lock_manager.holder(FLOOR_TELEOP),
            msg="dead-man tick должен снять teleop_floor",
        )
        # FSM переведён в off (escape hatch EVENT_FORCE_OFF).
        self.assertEqual(
            self.node._mode_manager.mode,
            Mode.OFF,
            msg=f"FSM должен быть off, got {self.node._mode_manager.mode}",
        )
        # DeadManCounter увеличился.
        self.assertEqual(
            self.node._dead_man.count("quest"),
            before_count + 1,
            msg="DeadManCounter.trip(quest) должен инкрементироваться",
        )

    def test_stale_tick_publishes_avatar_state(self) -> None:
        """После dead-man tick /avatar/state публикуется со свежим mode=off."""
        _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        pub = self.node._publishers["/avatar/state"]
        before = len(pub.published)
        self._clock_ms[0] += DEAD_MAN_TIMEOUT_MS + 1
        self.node._dead_man_tick()
        self.assertGreater(
            len(pub.published),
            before,
            msg="dead-man tick должен публиковать /avatar/state после release",
        )


# === Acceptance E: monitor mode — no twist_mux, no active timers =====


class TestMonitorModeBackwardCompat(unittest.TestCase):
    """Acceptance E: в monitor mode нода НЕ трогает /teleop_lock
    (S12, ADR-0028 §4.5 — minimize blast radius)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        # НЕ активируем — default = monitor.

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_monitor_mode_no_teleop_lock_publisher(self) -> None:
        """В monitor publisher /teleop_lock НЕ создаётся (или не используется)."""
        # В monitor нода не вмешивается в twist_mux (S12). Если publisher
        # не создан — service call в monitor не должен иметь side-effect.
        # Допускаем оба варианта: (а) publisher отсутствует; (б) publisher
        # создан лениво, но service call не публикует. Здесь тестируем (а)
        # через service call — после monitor-response в pub должно быть
        # столько же сообщений, сколько было до (0).
        result = _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        self.assertEqual(result["reason"], MONITOR_MODE_REASON)
        # Если publisher был создан (lazy), он НЕ должен был ничего
        # опубликовать.
        pub = self.node._publishers.get(TELEOP_LOCK_TOPIC)
        if pub is not None:
            self.assertEqual(
                len(pub.published),
                0,
                msg=("monitor-mode service call НЕ должен публиковать " "/teleop_lock"),
            )


# === Acceptance F: backward-compat с passive dead-man =================


class TestBackwardCompatWithPassiveDeadMan(unittest.TestCase):
    """Acceptance F: passive dead-man (через next-acquire) продолжает
    работать — мы НЕ сломали TestPhase2Acceptance6DeadMan.

    Суть: после acquire (heartbeat-fresh) и time-travel > DEAD_MAN_TIMEOUT_MS,
    следующий acquire от того же клиента должен:
      - либо auto-release expired floor → acquire succeeds (r.applied=True);
      - либо applied=False reason содержит "deadman".
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        _activate(self.node)
        self._clock_ms = _install_fake_clock(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_passive_deadman_still_works_after_active_watchdog(self) -> None:
        _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        self._clock_ms[0] += DEAD_MAN_TIMEOUT_MS + 1
        # Не вызываем _dead_man_tick() — имитируем «никто не дёрнул watchdog».
        # Вместо этого сразу делаем service call (passive detection).
        r = _call_active_service(self.node, "acquire_floor", "quest", FLOOR_TELEOP)
        # Passive dead-man: LockManager сам авто-releases expired floor,
        # acquire от того же клиента успешен.
        self.assertTrue(
            r["success"],
            msg=f"service call должен вернуть success, got {r!r}",
        )


if __name__ == "__main__":
    unittest.main()
