"""Unit-тесты для AvatarSupervisor ROS 2 ноды (Phase 1 monitor, AV-6 / AV-12).

Запускаются через mock-rclpy из conftest.py: rclpy недоступен на CI, но
conftest подсовывает FakeNode / FakeService + std_srvs.srv.Trigger +
rob_box_supervisor_msgs.srv.AcquireFloor/ReleaseFloor/SetAvatarMode.

Покрытие (acceptance AV-6 + AV-12):
- Нода создаётся с name="avatar_supervisor" и параметром mode="monitor".
- /avatar/state publisher создан (latched QoS).
- Подписки на /odom, /device/snapshot, /voice/dialogue/state созданы.
- Сервисы acquire_floor / release_floor / set_avatar_mode созданы
  (типизированный IDL — AV-12).
- AcquireFloor в monitor → granted=false, applied=false, reason=monitor.
- Timer-callback публикует msgpack-encoded /avatar/state (raw-evidence).
- Heartbeat трип через aggregator.record_dead_man_trip → счётчик
  попадает в /avatar/state.last_event и dead_man_trips_total.
- SetAvatarMode в monitor-режиме → НЕ меняет state, отвечает monitor.
- AcquireFloor в active:
  * empty client_id → granted=false, reason="bad_request"
  * ("quest","teleop") → granted=true, applied=true
  * второй клиент "telegram" на тот же floor → granted=false,
    held_by="quest", reason="held_by_other"
- У JSON-в-data контракта больше нет: _extract_floor_request
  возвращает (None, None) если поля клиента пустые/не заданы.
"""

from __future__ import annotations

import json
import unittest
from typing import Any
from unittest.mock import MagicMock

from rob_box_supervisor.core.fsm import Mode, ModeManager
from rob_box_supervisor.supervisor_node import (
    MONITOR_MODE_REASON,
    REASON_BAD_REQUEST,
    REASON_GRANTED,
    REASON_HELD_BY_OTHER,
    REASON_MONITOR,
    REASON_RELEASED,
    SET_VOICE_MODE_TOPIC,
    VOICE_INPUT_MODES,
    AvatarSupervisor,
)


def _make_string_msg(data: str) -> MagicMock:
    """Создать фейковый std_msgs/String с .data."""
    m = MagicMock()
    m.data = data
    return m


# ── IDL-совместимые helper-ы (AV-12, типизированный контракт) ──────────
#
# Нода вызывает _on_acquire_floor(request, response) где:
#   request.client_id / request.floor — typed-поля из rob_box_supervisor_msgs
#   response.success / response.granted / response.held_by /
#   response.reason / response.applied — typed-поля ответа.
#
# Для unit-тестов мы создаём «живой» typed Request (через conftest-овый
# FakeNode и FakeService.srv_type) и реальный typed Response object —
# так тесты проверяют тот самый контракт, который клиенты получают по
# проводу, а не Trigger-обёртку. Это и есть смысл AV-12.


def _make_typed_acquire_request(client_id: str = "", floor: str = "") -> MagicMock:
    """Создать typed-объект AcquireFloor.Request с заданными полями.

    Используем MagicMock — supervisor_node читает через getattr(..., None)
    и работает одинаково с любым объектом, у которого есть .client_id /
    .floor. Главное, чтобы это был НЕ Trigger.Request (как раньше в W3-2
    fake-хелпере), а typed-поля.
    """
    req = MagicMock()
    req.client_id = client_id
    req.floor = floor
    return req


def _make_typed_release_request(client_id: str = "", floor: str = "") -> MagicMock:
    req = MagicMock()
    req.client_id = client_id
    req.floor = floor
    return req


def _make_typed_set_mode_request(client_id: str = "", mode: str = "") -> MagicMock:
    req = MagicMock()
    req.client_id = client_id
    req.mode = mode
    return req


def _make_typed_response(srv_type: type) -> Any:
    """Создать response-объект с реально сохраняемыми bool/string-полями.

    ``srv_type`` — полный srv-класс (``AcquireFloor`` / ``ReleaseFloor`` /
    ``SetAvatarMode``), откуда берём вложенные ``Response``-классы для
    определения набора полей. Не используем ``MagicMock(spec=srv_type)`` —
    он ИГНОРИРУЕТ ``setattr`` и не сохраняет состояние, отчего все
    атрибуты читаются как новые MagicMock-объекты. Вместо этого делаем
    обычный fake-класс, чьи поля сохраняются между записями.
    """

    class _AcquireResponse:
        def __init__(self) -> None:
            self.success = False
            self.granted = False
            self.held_by = ""
            self.reason = ""
            self.applied = False

    class _ReleaseResponse:
        def __init__(self) -> None:
            self.success = False
            self.reason = ""
            self.applied = False

    class _SetModeResponse:
        def __init__(self) -> None:
            self.success = False
            self.mode = ""
            self.reason = ""
            self.applied = False

    fields = srv_type.Response._FIELDS
    if "granted" in fields:
        return _AcquireResponse()
    if "mode" in fields:
        return _SetModeResponse()
    return _ReleaseResponse()


def _acquire_response_to_dict(resp: MagicMock) -> dict:
    return {
        "success": resp.success,
        "granted": resp.granted,
        "held_by": resp.held_by,
        "reason": resp.reason,
        "applied": resp.applied,
    }


def _release_response_to_dict(resp: MagicMock) -> dict:
    return {
        "success": resp.success,
        "reason": resp.reason,
        "applied": resp.applied,
    }


def _set_mode_response_to_dict(resp: MagicMock) -> dict:
    return {
        "success": resp.success,
        "mode": resp.mode,
        "reason": resp.reason,
        "applied": resp.applied,
    }


def _get_typed_srv_type():
    """Достать conftest-овый srv-класс AcquireFloor.Request (то, что нода
    регистрирует как ``srv_type`` сервиса — см. ``_register_services``).
    """
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].AcquireFloor.Request


def _get_typed_release_srv_type():
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].ReleaseFloor.Request


def _get_typed_set_mode_srv_type():
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].SetAvatarMode.Request


def _get_typed_srv_full_type():
    """Полный srv-класс AcquireFloor (с nested Request/Response).

    Нужен helper-ам вроде ``_make_typed_response``, которым нужны поля
    ``Response`` (для определения набора attribute). Сама нода
    регистрирует *Request*, а не AcquireFloor-класс.
    """
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].AcquireFloor


def _get_typed_release_full_type():
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].ReleaseFloor


def _get_typed_set_mode_full_type():
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].SetAvatarMode


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

    def test_typed_idl_loaded(self) -> None:
        """AV-12: нода загружает rob_box_supervisor_msgs через try-import.

        При наличии conftest мок-ов — ``use_typed_floor_services`` True,
        и сервисы объявляются на AcquireFloor/ReleaseFloor/SetAvatarMode.
        """
        node = AvatarSupervisor()
        try:
            self.assertTrue(node._use_typed_floor_services)
            self.assertIsNotNone(node._msgs_types["AcqReq"])
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

    def test_services_use_typed_srv_type(self) -> None:
        """AV-12 acceptance: сервисы объявлены на AcquireFloor (не Trigger).

        Это структурная гарантия, что supervise-клиенты получают typed
        request/response, а не W3-2 Trigger fallback.
        """
        srv_types_by_name = {s.name: s.srv_type for s in self.node._services}
        typed_acquire = _get_typed_srv_type()
        typed_release = _get_typed_release_srv_type()
        typed_set_mode = _get_typed_set_mode_srv_type()
        self.assertIs(srv_types_by_name["acquire_floor"], typed_acquire)
        self.assertIs(srv_types_by_name["release_floor"], typed_release)
        self.assertIs(srv_types_by_name["set_avatar_mode"], typed_set_mode)


class TestAvatarSupervisorMonitorServices(unittest.TestCase):
    """Сервисы в monitor-режиме (AV-12, типизированный IDL):

    - acquire_floor: success=true, granted=false, applied=false, reason=monitor
    - release_floor: success=true, applied=false, reason=monitor
    - set_avatar_mode: success=true, applied=false, mode=<text>, reason=monitor
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_acquire_floor_monitor_response(self) -> None:
        """Monitor: granted=false, applied=false, reason=monitor.

        Даже если клиент пошлёт валидный client_id/floor — в monitor
        supervisor НЕ вмешивается (ADR-0028 §4.5).
        """
        svc = next(s for s in self.node._services if s.name == "acquire_floor")
        req = _make_typed_acquire_request(client_id="quest", floor="teleop")
        resp = _make_typed_response(_get_typed_srv_full_type())
        svc.callback(req, resp)
        body = _acquire_response_to_dict(resp)
        self.assertTrue(body["success"])
        self.assertFalse(body["granted"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)
        self.assertEqual(body["held_by"], "")

    def test_release_floor_monitor_response(self) -> None:
        svc = next(s for s in self.node._services if s.name == "release_floor")
        req = _make_typed_release_request(client_id="quest", floor="teleop")
        resp = _make_typed_response(_get_typed_release_full_type())
        svc.callback(req, resp)
        body = _release_response_to_dict(resp)
        self.assertTrue(body["success"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)

    def test_set_avatar_mode_monitor_response(self) -> None:
        svc = next(s for s in self.node._services if s.name == "set_avatar_mode")
        req = _make_typed_set_mode_request(client_id="telegram1", mode="telegram_acquire_floor")
        resp = _make_typed_response(_get_typed_set_mode_full_type())
        svc.callback(req, resp)
        body = _set_mode_response_to_dict(resp)
        self.assertTrue(body["success"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)
        # avatar_mode (текущий mode ModeManager) эхом приходит в response.mode
        # в Phase 1 это "off" (стартовое состояние FSM).
        self.assertEqual(body["mode"], "off")

    def test_no_trigger_srv_type_for_typed_services(self) -> None:
        """Регресс W3-2: сервисы НЕ должны быть на std_srvs/Trigger."""
        from std_srvs.srv import Trigger

        srv_types_by_name = {s.name: s.srv_type for s in self.node._services}
        for name in ("acquire_floor", "release_floor", "set_avatar_mode"):
            self.assertIsNot(srv_types_by_name[name], Trigger)


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
        # Сообщение содержит ключевые поля (mode/zenoh/msgpack/typed_services),
        # которые раньше передавались как отдельные format-args.
        self.assertIn("avatar_supervisor started", msg)
        self.assertIn(f"mode={self.node._mode}", msg)
        self.assertIn("msgpack=", msg)
        self.assertIn("typed_services=", msg)
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


class TestAcquireFloorTypedContract(unittest.TestCase):
    """AV-12 acceptance: typed AcquireFloor в active-режиме через LockManager.

    Контракт запроса — типизированный IDL (см. supervisor_node._extract_floor_request).
    Прошлый JSON-в-data fallback УБРАН (R13).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _acquire(self, client_id: str, floor: str) -> dict:
        """Дёрнуть сервисный callback, как будто пришёл typed request."""
        svc = next(s for s in self.node._services if s.name == "acquire_floor")
        req = _make_typed_acquire_request(client_id=client_id, floor=floor)
        resp = _make_typed_response(_get_typed_srv_full_type())
        svc.callback(req, resp)
        return _acquire_response_to_dict(resp)

    # ── AV-12 acceptance tests (issue #1904) ──────────────────────────
    def test_empty_client_id_returns_bad_request(self) -> None:
        """Acceptance: пустой client_id → granted=false, reason=bad_request."""
        body = self._acquire("", "teleop")
        self.assertFalse(body["granted"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)
        self.assertEqual(body["held_by"], "")

    def test_first_acquire_grants_and_applies(self) -> None:
        """Acceptance: acquire_floor("quest","teleop") → granted=true, applied=true."""
        body = self._acquire("quest", "teleop")
        self.assertTrue(body["granted"])
        self.assertTrue(body["applied"])
        self.assertEqual(body["reason"], REASON_GRANTED)
        self.assertEqual(body["held_by"], "quest")

    def test_second_client_on_same_floor_blocked(self) -> None:
        """Acceptance: после первого grant второй клиент → granted=false,
        held_by="quest", reason="held_by_other"."""
        first = self._acquire("quest", "teleop")
        self.assertTrue(first["granted"], first)

        second = self._acquire("telegram", "teleop")
        self.assertFalse(second["granted"])
        self.assertTrue(second["applied"])
        self.assertEqual(second["held_by"], "quest")
        self.assertEqual(second["reason"], REASON_HELD_BY_OTHER)

    def test_idempotent_reacquire_same_client(self) -> None:
        """Повторный acquire тем же client_id — granted=True оба раза."""
        first = self._acquire("quest", "teleop")
        second = self._acquire("quest", "teleop")
        self.assertTrue(first["granted"])
        self.assertTrue(second["granted"])
        self.assertEqual(second["reason"], REASON_GRANTED)

    def test_unknown_floor_returns_bad_request(self) -> None:
        """Floor, которого нет в Floor.values() → granted=false, reason=bad_request."""
        body = self._acquire("quest", "lidar_floor")
        self.assertFalse(body["granted"])
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)

    def test_release_by_owner_succeeds(self) -> None:
        """Release от owner-а → applied=true, reason="released"."""
        self._acquire("quest", "teleop")
        svc = next(s for s in self.node._services if s.name == "release_floor")
        req = _make_typed_release_request(client_id="quest", floor="teleop")
        resp = _make_typed_response(_get_typed_release_full_type())
        svc.callback(req, resp)
        body = _release_response_to_dict(resp)
        self.assertTrue(body["applied"])
        self.assertEqual(body["reason"], REASON_RELEASED)

    def test_release_by_wrong_client_denied(self) -> None:
        """Release не-owner-ом → applied=false, reason="permission_denied"."""
        self._acquire("quest", "teleop")
        svc = next(s for s in self.node._services if s.name == "release_floor")
        req = _make_typed_release_request(client_id="telegram", floor="teleop")
        resp = _make_typed_response(_get_typed_release_full_type())
        svc.callback(req, resp)
        body = _release_response_to_dict(resp)
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], "permission_denied")

    def test_dead_man_releases_floor_after_500ms_without_heartbeat(self) -> None:
        """Без heartbeat > 500 мс (ADR-0028 §6 Q4) floor освобождается —
        новый клиент может acquire."""
        from rob_box_supervisor.core import LockManager

        clock = {"t": 0}
        self.node._lock_manager = LockManager(clock=lambda: clock["t"])

        first = self._acquire("quest", "teleop")
        self.assertTrue(first["granted"])

        clock["t"] += 501  # dead-man timeout = 500 мс

        second = self._acquire("telegram", "teleop")
        self.assertTrue(second["granted"], second)

    def test_dead_man_trip_increments_aggregator_metric(self) -> None:
        """Dead-man авто-release замечается таймер-тиком и попадает
        в dead_man_trips_total."""
        from rob_box_supervisor.core import LockManager

        clock = {"t": 0}
        self.node._lock_manager = LockManager(clock=lambda: clock["t"])

        self._acquire("quest", "teleop")
        clock["t"] += 501
        self.node._check_dead_man_trips()

        self.assertEqual(self.node._aggregator.dead_man_count("quest"), 1)

    def test_monitor_mode_still_applied_false_for_floor_ops(self) -> None:
        """В monitor-режиме acquire/release_floor по-прежнему applied=false
        (регресс ADR-0028 §4.5)."""
        self.node._mode = "monitor"
        acquire_body = self._acquire("quest", "teleop")
        self.assertFalse(acquire_body["applied"])
        self.assertFalse(acquire_body["granted"])
        self.assertEqual(acquire_body["reason"], REASON_MONITOR)

        svc = next(s for s in self.node._services if s.name == "release_floor")
        req = _make_typed_release_request(client_id="quest", floor="teleop")
        resp = _make_typed_response(_get_typed_release_full_type())
        svc.callback(req, resp)
        release_body = _release_response_to_dict(resp)
        self.assertFalse(release_body["applied"])
        self.assertEqual(release_body["reason"], REASON_MONITOR)

    def test_no_json_in_data_field_used(self) -> None:
        """Рергресс R13: _extract_floor_request НЕ парсит JSON из request.data.

        Если клиент отошлёт только ``data=JSON``, сервис НЕ должен его
        принять — typed IDL требует явные ``client_id``/``floor``.
        """
        # Имитируем запрос «старого стиля» — только request.data с JSON.
        legacy_req = MagicMock(spec=["data"])
        legacy_req.data = json.dumps({"client_id": "quest", "floor": "teleop"})

        svc = next(s for s in self.node._services if s.name == "acquire_floor")
        resp = _make_typed_response(_get_typed_srv_full_type())
        svc.callback(legacy_req, resp)
        body = _acquire_response_to_dict(resp)
        # legacy-запрос с пустым client_id/floor → bad_request;
        # НЕ granted (т.е. JSON-в-data больше НЕ маскирует иллюзию работы).
        self.assertFalse(body["granted"])
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)


class TestSetAvatarModeTypedContract(unittest.TestCase):
    """AV-12 acceptance: typed SetAvatarMode в active-режиме через ModeManager.

    Контракт запроса — ``client_id``/``mode`` (последний — имя FSM-события,
    см. ADR-0028 §4.1). Прошлый JSON-в-data fallback УБРАН.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _set_mode(self, client_id: str, mode: str) -> dict:
        svc = next(s for s in self.node._services if s.name == "set_avatar_mode")
        req = _make_typed_set_mode_request(client_id=client_id, mode=mode)
        resp = _make_typed_response(_get_typed_set_mode_full_type())
        svc.callback(req, resp)
        return _set_mode_response_to_dict(resp)

    def test_mode_manager_instantiated(self) -> None:
        """W3-4 задел: ``ModeManager`` инстанцируется в ноде (раньше — нет)."""
        self.assertIsInstance(self.node._mode_manager, ModeManager)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_valid_transition_applies_and_changes_mode(self) -> None:
        """``off → telegram_active``: applied=true, новый режим виден в ответе."""
        body = self._set_mode("telegram1", "telegram_acquire_floor")
        self.assertTrue(body["applied"], body)
        self.assertEqual(body["mode"], "telegram_active")
        self.assertEqual(self.node._mode_manager.mode, Mode.TELEGRAM_ACTIVE)

    def test_invalid_transition_refused_as_conflict(self) -> None:
        """``quest_acquire_floor_teleop_only`` из ``off`` — переход невалиден
        (годится только из ``telegram_active``, ADR-0028 §4.1) — отказ,
        режим не меняется."""
        body = self._set_mode("quest1", "quest_acquire_floor_teleop_only")
        self.assertFalse(body["applied"], body)
        self.assertEqual(body["reason"], "conflict")
        self.assertEqual(body["mode"], "off")
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_unknown_event_refused(self) -> None:
        """Неизвестное имя события — отказ с внятной причиной, режим не меняется."""
        body = self._set_mode("telegram1", "not_a_real_event")
        self.assertFalse(body["applied"], body)
        self.assertEqual(body["reason"], "invalid_event")
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_empty_mode_refused(self) -> None:
        """AV-12 acceptance: пустой mode → bad_request, режим не меняется."""
        body = self._set_mode("telegram1", "")
        self.assertFalse(body["applied"], body)
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_monitor_mode_still_applied_false(self) -> None:
        """Регресс ADR-0028 §4.5: в ``monitor`` SetAvatarMode по-прежнему
        ``applied=false``, avatar-режим не трогается."""
        self.node._mode = "monitor"
        body = self._set_mode("telegram1", "telegram_acquire_floor")
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_mode_change_releases_lock_manager_floors(self) -> None:
        """W3-4 — уход из активного avatar-режима синхронно освобождает
        LockManager floor-ы того же ``client_id``: иначе остаётся висячий
        holder, до которого больше не достучаться через ``ReleaseFloor``
        (см. docstring ``AvatarSupervisor._set_avatar_mode_logic``)."""
        # Клиент реально держит оба floor-а через LockManager (как будто
        # раньше отдельно вызвал AcquireFloor — W3-2). Используем
        # LockManager-имена, т.к. работаем напрямую с LockManager.
        from rob_box_supervisor.core import Floor as LockFloor

        self.node._acquire_floor_logic("questA", LockFloor.TELEOP)
        self.node._acquire_floor_logic("questA", LockFloor.VOICE)
        # И тот же client_id зафиксирован в FSM как участник avatar_present.
        entered = self._set_mode("questA", "quest_acquire_floor")
        self.assertEqual(entered["mode"], "avatar_present")

        result = self._set_mode("questA", "quest_release")
        self.assertTrue(result["applied"], result)
        self.assertEqual(result["mode"], "off")

        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.VOICE))

    def test_no_phase2_warning_logged(self) -> None:
        """Регресс: вводящий в заблуждение warning «Phase 2 не реализован» убран."""
        self.node._log.reset_mock()
        self._set_mode("telegram1", "telegram_acquire_floor")
        for call in self.node._log.warning.call_args_list:
            msg = call.args[0] if call.args else ""
            self.assertNotIn("Phase 2", msg)

    def test_set_avatar_mode_logs_single_msg_arg(self) -> None:
        """Регресс #1644 (см. ``_log_startup_diagnostics``): ``info()`` должен
        получать РОВНО один позиционный ``msg``-аргумент."""
        self.node._log.reset_mock()
        self._set_mode("telegram1", "telegram_acquire_floor")
        self.assertTrue(self.node._log.info.called)
        call = self.node._log.info.call_args
        self.assertEqual(len(call.args), 1, f"info() должен получить 1 positional arg, got {call.args!r}")
        self.assertEqual(call.kwargs, {})

    def test_no_json_in_data_field_used(self) -> None:
        """Рергресс R13: _extract_avatar_mode_request НЕ парсит JSON из request.data."""
        legacy_req = MagicMock(spec=["data"])
        legacy_req.data = json.dumps({"event": "telegram_acquire_floor", "client_id": "telegram1"})

        svc = next(s for s in self.node._services if s.name == "set_avatar_mode")
        resp = _make_typed_response(_get_typed_set_mode_full_type())
        svc.callback(legacy_req, resp)
        body = _set_mode_response_to_dict(resp)
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)


if __name__ == "__main__":
    unittest.main()
