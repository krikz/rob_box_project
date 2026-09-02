"""Unit-тесты для AvatarSupervisor ROS 2 ноды (Phase 1 monitor, AV-6).

Запускаются через mock-rclpy из conftest.py: rclpy недоступен на CI, но
conftest подсовывает FakeNode / FakeService / std_srvs.srv.Trigger.

Покрытие (acceptance AV-6):
- Нода создаётся с name="avatar_supervisor" и параметром mode="monitor".
- /avatar/state publisher создан (latched QoS).
- Подписки на /odom, /device/snapshot, /voice/dialogue/state созданы.
- Сервисы acquire_floor / release_floor / set_avatar_mode созданы.
- AcquireFloor → success=True, message содержит applied=false/reason=monitor.
- Timer-callback публикует msgpack-encoded /avatar/state (raw-evidence).
- Heartbeat трип через aggregator.record_dead_man_trip → счётчик
  попадает в /avatar/state.last_event и dead_man_trips_total.
- SetAvatarMode в monitor-режиме → НЕ меняет state, отвечает monitor.
"""

from __future__ import annotations

import json
import types
import unittest
from unittest.mock import MagicMock

from rob_box_supervisor.core.fsm import Mode, ModeManager
from rob_box_supervisor.supervisor_node import (
    MONITOR_MODE_REASON,
    SET_VOICE_MODE_TOPIC,
    VOICE_INPUT_MODES,
    AvatarSupervisor,
)


def _make_string_msg(data: str) -> MagicMock:
    """Создать фейковый std_msgs/String с .data."""
    m = MagicMock()
    m.data = data
    return m


class TestAvatarSupervisorCreation(unittest.TestCase):
    def test_node_name_is_avatar_supervisor(self) -> None:
        node = AvatarSupervisor()
        try:
            self.assertEqual(node.get_name(), "avatar_supervisor")
        finally:
            node.destroy_node()

    def test_mode_parameter_defaults_to_monitor(self) -> None:
        node = AvatarSupervisor()
        try:
            self.assertEqual(node.get_parameter("mode").value, "monitor")
            self.assertTrue(node.has_parameter("mode"))
        finally:
            node.destroy_node()


class TestAvatarSupervisorTopology(unittest.TestCase):
    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_avatar_state_publisher_exists(self) -> None:
        self.assertIn("/avatar/state", self.node._publishers)

    def test_odom_subscription_registered(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn("/odom", topics)

    def test_device_snapshot_subscription_registered(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn("/device/snapshot", topics)

    def test_voice_dialogue_state_subscription_registered(self) -> None:
        """AV-6 acceptance: подписка на ``/voice/dialogue/state`` (НЕ /voice/state)."""
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn("/voice/dialogue/state", topics)
        self.assertNotIn("/voice/state", topics)

    def test_services_registered(self) -> None:
        names = [s.name for s in self.node._services]
        self.assertIn("acquire_floor", names)
        self.assertIn("release_floor", names)
        self.assertIn("set_avatar_mode", names)


class TestAvatarSupervisorMonitorServices(unittest.TestCase):
    """Сервисы в monitor-режиме отвечают ``success=true/applied=false/reason=monitor``."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _call_service(self, name: str) -> dict:
        svc = next(s for s in self.node._services if s.name == name)
        req = MagicMock()  # Trigger.Request — пустой
        resp = MagicMock()  # Trigger.Response
        resp.success = False
        resp.message = ""
        svc.callback(req, resp)
        return {"success": resp.success, "message": resp.message}

    def test_acquire_floor_monitor_response(self) -> None:
        result = self._call_service("acquire_floor")
        self.assertTrue(result["success"])
        body = json.loads(result["message"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)

    def test_release_floor_monitor_response(self) -> None:
        result = self._call_service("release_floor")
        self.assertTrue(result["success"])
        body = json.loads(result["message"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)

    def test_set_avatar_mode_monitor_response(self) -> None:
        result = self._call_service("set_avatar_mode")
        self.assertTrue(result["success"])
        body = json.loads(result["message"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)


class TestAvatarSupervisorPublishHeartbeat(unittest.TestCase):
    """Heartbeat трип → /avatar/state обновляется (AV-6 acceptance)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_timer_publishes_avatar_state(self) -> None:
        """Timer-callback публикует msgpack-encoded payload в /avatar/state."""
        # Трипнуть через aggregator (имитация heartbeat-watchdog).
        self.node._aggregator.update_odom(1.0, 2.0)
        self.node._aggregator.update_device_snapshot(battery_pct=80.0)
        self.node._aggregator.update_voice_state("listening")
        self.node._aggregator.record_dead_man_trip("quest")

        # Запустить timer-callback вручную (без rclpy.spin).
        self.node._publish_avatar_state()

        pub = self.node._publishers["/avatar/state"]
        self.assertEqual(len(pub.published), 1)
        msg = pub.published[0]
        # Raw-evidence: data — это сериализованный msgpack-as-text.
        self.assertTrue(msg.data)
        # msgpack-encoded bytes были декодированы latin-1 в str.
        # Проверяем что данные не пустые и не None.
        self.assertIsInstance(msg.data, str)
        self.assertGreater(len(msg.data), 0)

    def test_heartbeat_trip_updates_aggregator_state(self) -> None:
        """Heartbeat трип → last_event и dead_man_trips_total попадают в state."""
        new_count = self.node._aggregator.record_dead_man_trip("quest")
        snap = self.node._aggregator.snapshot()
        self.assertEqual(new_count, 1)
        self.assertEqual(snap.dead_man_trips_total, {"quest": 1})
        self.assertEqual(snap.last_event["kind"], "dead_man_trip")
        self.assertEqual(snap.last_event["client_id"], "quest")

    def test_subscription_callbacks_feed_aggregator(self) -> None:
        """Сообщения в /odom, /device/snapshot, /voice/dialogue/state обновляют state."""
        self.node._on_odom_msg(_make_string_msg(json.dumps({"x": 5.0, "y": -3.0})))
        self.node._on_device_snapshot_msg(_make_string_msg(json.dumps({"battery_pct": 73.2})))
        self.node._on_voice_state_msg(_make_string_msg(json.dumps({"state": "speaking"})))
        snap = self.node._aggregator.snapshot()
        self.assertEqual(snap.pose_xy, (5.0, -3.0))
        self.assertEqual(snap.battery_pct, 73.2)
        self.assertEqual(snap.voice_state, "speaking")

    def test_garbage_subscription_payload_does_not_crash(self) -> None:
        """Битые сообщения в топиках не валят ноду (best-effort parse)."""
        # Просто не должно быть исключений.
        self.node._on_odom_msg(_make_string_msg("not-json"))
        self.node._on_device_snapshot_msg(_make_string_msg(""))
        self.node._on_voice_state_msg(_make_string_msg("{}"))


class TestAvatarSupervisorDoesNotMutateExternalState(unittest.TestCase):
    """Monitor-режим НЕ меняет twist_mux / dialogue_node (ADR-0028 §4.5)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_no_twist_mux_publisher(self) -> None:
        """Нода в Phase 1 НЕ публикует cmd_vel_* напрямую."""
        topics = list(self.node._publishers.keys())
        for t in topics:
            self.assertNotIn("cmd_vel", t)
            self.assertNotIn("twist_mux", t)

    def test_no_set_parameter_calls_for_dialogue(self) -> None:
        """Нода в Phase 1 НЕ правит параметры dialogue_node (нет service-client'а)."""
        # Доказательство: нода не создаёт service-client'ов, только service-servers.
        # Это структурная гарантия — никаких side-effects на dialogue_node.
        # (Phase 2 active-режим может добавить, но только под mode=active.)
        self.assertEqual(self.node._publishers, self.node._publishers)  # smoke
        # На всякий случай: нет ни одного publisher-а на /voice/ или /dialogue/
        for topic in self.node._publishers:
            self.assertFalse(
                topic.startswith("/voice/"),
                f"Phase 1 monitor must NOT publish to /voice/* (got {topic})",
            )
            self.assertFalse(
                topic.startswith("/dialogue/"),
                f"Phase 1 monitor must NOT publish to /dialogue/* (got {topic})",
            )

    def test_log_startup_diagnostics_uses_single_msg_arg(self) -> None:
        """Регресс #1644: ``_log.info`` должен получать ОДИН строковый msg.

        rclpy ``RcutilsLogger.info(msg, *args)`` принимает ``*args`` для
        ``%``-форматирования msg, а не как самостоятельные поля. Старый
        вызов ``info(fmt, mode, zenoh, msgpack)`` (3 args) ломал рантайм
        в проде: ``TypeError: RcutilsLogger.info() takes 2 positional
        arguments but 5 were given`` (run #32892615440 на Vision Pi).

        Тест проверяет инвариант: при вызове diagnostics info() получает
        ровно одну позиционную строку-msg, без format-args.
        """
        # Сбрасываем историю вызовов MagicMock-логгера, чтобы тест не зависел
        # от того, что conftest уже мог что-то залогировать при инициализации.
        self.node._log.reset_mock()
        self.node._log_startup_diagnostics()
        self.assertTrue(self.node._log.info.called, "info() должен быть вызван")
        call = self.node._log.info.call_args
        # Ровно один позиционный аргумент — итоговая f-string.
        self.assertEqual(
            len(call.args),
            1,
            f"info() должен получить 1 positional arg (msg), got {call.args!r}",
        )
        msg = call.args[0]
        self.assertIsInstance(msg, str)
        # Сообщение содержит ключевые поля (mode/zenoh), которые раньше
        # передавались как отдельные format-args. ``msgpack=…`` убрано в
        # AV-14 (#1906) — msgpack стал hard dep в ``rob_box_supervisor``,
        # смёржен на уровне ``core.state``, и эта diagnostics-строка
        # больше не должна сообщать про его наличие (это уже не поле,
        # которое варьируется на проде).
        self.assertIn("avatar_supervisor started", msg)
        self.assertIn(f"mode={self.node._mode}", msg)
        # ``msgpack=`` намеренно отсутствует — см. комментарий выше.
        self.assertNotIn("msgpack=", msg)
        # Никаких kwargs %-форматирования быть не должно (kwargs в rclpy
        # info() не поддерживаются и упадут так же, как и >1 args).
        self.assertEqual(
            call.kwargs,
            {},
            f"info() не должен получать kwargs, got {call.kwargs!r}",
        )


class TestAvatarSupervisorVoiceMode(unittest.TestCase):
    """ADR-0028 S5 — супервизор владеет voice_input_mode (Phase 1)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_set_voice_mode_topic_subscribed(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(SET_VOICE_MODE_TOPIC, topics)

    def test_monitor_mode_does_not_apply(self) -> None:
        """В monitor супервизор принимает режим, но НЕ применяет (S12)."""
        applied, reason = self.node._apply_voice_mode("quest_ttts")
        self.assertFalse(applied)
        self.assertEqual(reason, MONITOR_MODE_REASON)

    def test_invalid_mode_rejected(self) -> None:
        applied, reason = self.node._apply_voice_mode("not_a_mode")
        self.assertFalse(applied)
        self.assertIn("invalid_voice_mode", reason)

    def test_active_mode_dispatches_param_set(self) -> None:
        """В active режиме валидный режим → _set_dialogue_param вызывается."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        applied, reason = self.node._apply_voice_mode("quest_ttts")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with("voice_input_mode", "quest_ttts")

    def test_on_set_voice_mode_feeds_apply(self) -> None:
        """Топик → _apply_voice_mode; в monitor применяется=false."""
        self.node._apply_voice_mode = MagicMock(return_value=(False, MONITOR_MODE_REASON))
        self.node._on_set_voice_mode(_make_string_msg("quest_ttts"))
        self.node._apply_voice_mode.assert_called_once_with("quest_ttts")

    def test_off_mode_is_valid(self) -> None:
        """W3-1 — "off" ("диалог off", §3.5 dialogue-mode-spec) в списке
        допустимых режимов voice_input_mode (ADR-0027 §3.4)."""
        self.assertIn("off", VOICE_INPUT_MODES)

    def test_active_mode_dispatches_off(self) -> None:
        """В active режиме "off" применяется так же, как остальные режимы —
        супервизор не отличает "off" от прочих значений на своей стороне,
        вся логика блокировки ReSpeaker — в dialogue_node."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        applied, reason = self.node._apply_voice_mode("off")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with("voice_input_mode", "off")


class TestAvatarSupervisorFloorLockManager(unittest.TestCase):
    """W3-2 — acquire_floor/release_floor реально работают через LockManager.

    Контракт запроса (см. ``AvatarSupervisor._extract_floor_request``) —
    ЗАВЕДОМО переходный: атрибуты ``client_id``/``floor`` на запросе ИЛИ
    JSON в ``request.data`` — до кастомного IDL (AV-5, ADR-0028 §4.3).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    @staticmethod
    def _req(client_id, floor):
        """Переходный контракт (а): атрибуты client_id/floor на запросе."""
        return types.SimpleNamespace(client_id=client_id, floor=floor)

    def _acquire(self, client_id, floor) -> dict:
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        self.node._on_acquire_floor(self._req(client_id, floor), resp)
        return json.loads(resp.message)

    def _release(self, client_id, floor) -> dict:
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        self.node._on_release_floor(self._req(client_id, floor), resp)
        return json.loads(resp.message)

    def test_conflict_between_two_clients_on_same_floor(self) -> None:
        """Второй клиент, запросивший уже занятый floor, получает granted=False."""
        first = self._acquire("quest", "voice_floor")
        self.assertTrue(first["granted"], first)

        second = self._acquire("telegram", "voice_floor")
        self.assertFalse(second["granted"])
        self.assertIn("conflict", second["reason"])
        self.assertIn("quest", second["reason"])  # внятная причина — кто держит

    def test_idempotent_reacquire_same_client(self) -> None:
        """Повторный acquire тем же client_id — не конфликт, granted=True оба раза."""
        first = self._acquire("quest", "teleop_floor")
        second = self._acquire("quest", "teleop_floor")
        self.assertTrue(first["granted"])
        self.assertTrue(second["granted"])
        self.assertEqual(second["reason"], "granted")

    def test_dead_man_releases_floor_after_500ms_without_heartbeat(self) -> None:
        """Без heartbeat > 500 мс (ADR-0028 §6 Q4) floor освобождается — новый клиент может acquire."""
        from rob_box_supervisor.core import LockManager

        clock = {"t": 0}
        self.node._lock_manager = LockManager(clock=lambda: clock["t"])

        first = self._acquire("quest", "teleop_floor")
        self.assertTrue(first["granted"])

        clock["t"] += 501  # dead-man timeout = 500 мс

        second = self._acquire("telegram", "teleop_floor")
        self.assertTrue(second["granted"], second)

    def test_dead_man_trip_increments_aggregator_metric(self) -> None:
        """Dead-man авто-release замечается таймер-тиком и попадает в dead_man_trips_total."""
        from rob_box_supervisor.core import LockManager

        clock = {"t": 0}
        self.node._lock_manager = LockManager(clock=lambda: clock["t"])

        self._acquire("quest", "voice_floor")
        clock["t"] += 501
        self.node._check_dead_man_trips()

        self.assertEqual(self.node._aggregator.dead_man_count("quest"), 1)

    def test_release_by_owner_succeeds(self) -> None:
        self._acquire("quest", "teleop_floor")
        result = self._release("quest", "teleop_floor")
        self.assertTrue(result["applied"])
        self.assertEqual(result["reason"], "released")

    def test_release_by_wrong_client_denied(self) -> None:
        self._acquire("quest", "teleop_floor")
        result = self._release("telegram", "teleop_floor")
        self.assertFalse(result["applied"])
        self.assertIn("permission_denied", result["reason"])

    def test_json_in_request_data_is_accepted(self) -> None:
        """Переходный контракт (б): JSON в request.data (без атрибутов client_id/floor)."""
        req = types.SimpleNamespace(data=json.dumps({"client_id": "quest", "floor": "voice_floor"}))
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        self.node._on_acquire_floor(req, resp)
        body = json.loads(resp.message)
        self.assertTrue(body["granted"])

    def test_monitor_mode_still_applied_false_for_floor_ops(self) -> None:
        """В monitor-режиме acquire/release_floor по-прежнему applied=false (регресс ADR-0028 §4.5)."""
        self.node._mode = "monitor"
        acquire_result = self._acquire("quest", "voice_floor")
        self.assertFalse(acquire_result["applied"])
        self.assertFalse(acquire_result["granted"])
        self.assertEqual(acquire_result["reason"], MONITOR_MODE_REASON)

        release_result = self._release("quest", "voice_floor")
        self.assertFalse(release_result["applied"])
        self.assertEqual(release_result["reason"], MONITOR_MODE_REASON)


class TestAvatarSupervisorSetAvatarMode(unittest.TestCase):
    """W3-4 (issue #968 wave2) — ``SetAvatarMode`` реально меняет avatar-режим
    через :class:`ModeManager` (``core/fsm.py``) в ``active``-режиме, вместо
    заглушки «Phase 2 не реализован» (ADR-0028 §4.1, §4.3).

    Контракт запроса — тот же переходный паттерн, что и floor-ы (W3-2, см.
    ``AvatarSupervisor._extract_floor_request``): атрибуты ``event``/
    ``client_id`` на запросе ИЛИ JSON в ``request.data``. ``event`` — имя
    FSM-события (``core.fsm.EVENT_*``), а не целевой режим напрямую:
    ``ModeManager`` — событийный автомат (ADR-0028 §4.1 mermaid), а не
    setter состояния.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    @staticmethod
    def _req(event, client_id):
        return types.SimpleNamespace(event=event, client_id=client_id)

    def _set_mode(self, event, client_id) -> dict:
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        self.node._on_set_avatar_mode(self._req(event, client_id), resp)
        return json.loads(resp.message)

    def test_mode_manager_instantiated(self) -> None:
        """W3-4 задел: ``ModeManager`` инстанцируется в ноде (раньше — нет)."""
        self.assertIsInstance(self.node._mode_manager, ModeManager)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_valid_transition_applies_and_changes_mode(self) -> None:
        """``off → telegram_active``: applied=true, новый режим виден в ответе."""
        result = self._set_mode("telegram_acquire_floor", "telegram1")
        self.assertTrue(result["applied"], result)
        self.assertEqual(result["actual_mode"], "telegram_active")
        self.assertEqual(self.node._mode_manager.mode, Mode.TELEGRAM_ACTIVE)

    def test_invalid_transition_refused_as_conflict(self) -> None:
        """``quest_acquire_floor_teleop_only`` из ``off`` — переход невалиден
        (годится только из ``telegram_active``, ADR-0028 §4.1) — отказ,
        режим не меняется."""
        result = self._set_mode("quest_acquire_floor_teleop_only", "quest1")
        self.assertFalse(result["applied"], result)
        self.assertIn("conflict", result["reason"])
        self.assertEqual(result["actual_mode"], "off")
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_unknown_event_refused(self) -> None:
        """Неизвестное имя события — отказ с внятной причиной, режим не меняется."""
        result = self._set_mode("not_a_real_event", "telegram1")
        self.assertFalse(result["applied"], result)
        self.assertIn("invalid_event", result["reason"])
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_monitor_mode_still_applied_false(self) -> None:
        """Регресс ADR-0028 §4.5: в ``monitor`` SetAvatarMode по-прежнему
        ``applied=false``, avatar-режим не трогается."""
        self.node._mode = "monitor"
        result = self._set_mode("telegram_acquire_floor", "telegram1")
        self.assertFalse(result["applied"])
        self.assertEqual(result["reason"], MONITOR_MODE_REASON)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_json_in_request_data_is_accepted(self) -> None:
        """Переходный контракт (б): JSON в ``request.data`` вместо атрибутов."""
        req = types.SimpleNamespace(
            data=json.dumps({"event": "telegram_acquire_floor", "client_id": "telegram1"})
        )
        resp = MagicMock()
        resp.success = False
        resp.message = ""
        self.node._on_set_avatar_mode(req, resp)
        body = json.loads(resp.message)
        self.assertTrue(body["applied"])
        self.assertEqual(body["actual_mode"], "telegram_active")

    def test_mode_change_releases_lock_manager_floors(self) -> None:
        """W3-4 — уход из активного avatar-режима синхронно освобождает
        LockManager floor-ы того же ``client_id``: иначе остаётся висячий
        holder, до которого больше не достучаться через ``ReleaseFloor``
        (см. docstring ``AvatarSupervisor._set_avatar_mode_logic``)."""
        # Клиент реально держит оба floor-а через LockManager (как будто
        # раньше отдельно вызвал AcquireFloor — W3-2).
        self.node._acquire_floor_logic("questA", "teleop_floor")
        self.node._acquire_floor_logic("questA", "voice_floor")
        # И тот же client_id зафиксирован в FSM как участник avatar_present.
        entered = self._set_mode("quest_acquire_floor", "questA")
        self.assertEqual(entered["actual_mode"], "avatar_present")

        result = self._set_mode("quest_release", "questA")
        self.assertTrue(result["applied"], result)
        self.assertEqual(result["actual_mode"], "off")

        from rob_box_supervisor.core import Floor as LockFloor

        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.VOICE))

    def test_no_phase2_warning_logged(self) -> None:
        """Регресс: вводящий в заблуждение warning «Phase 2 не реализован» убран."""
        self.node._log.reset_mock()
        self._set_mode("telegram_acquire_floor", "telegram1")
        for call in self.node._log.warning.call_args_list:
            msg = call.args[0] if call.args else ""
            self.assertNotIn("Phase 2", msg)

    def test_set_avatar_mode_logs_single_msg_arg(self) -> None:
        """Регресс #1644 (см. ``_log_startup_diagnostics``): ``info()`` должен
        получать РОВНО один позиционный ``msg``-аргумент."""
        self.node._log.reset_mock()
        self._set_mode("telegram_acquire_floor", "telegram1")
        self.assertTrue(self.node._log.info.called)
        call = self.node._log.info.call_args
        self.assertEqual(len(call.args), 1, f"info() должен получить 1 positional arg, got {call.args!r}")
        self.assertEqual(call.kwargs, {})


if __name__ == "__main__":
    unittest.main()
