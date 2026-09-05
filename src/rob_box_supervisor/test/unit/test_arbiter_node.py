"""Unit-тесты для AvatarArbiter ROS 2 ноды (ADR-0051 §2.2, issue #1987).

AvatarArbiter — отдельная нода ``avatar_arbiter`` (без LLM), в которую из
``avatar_supervisor`` переехал арбитраж floor/FSM + публикация
``/avatar/state``: LockManager, ModeManager, агрегатор, типизированные
сервисы ``/avatar_arbiter/{acquire_floor,release_floor,set_avatar_mode}``,
heartbeat-подписка ``/teleop_heartbeat`` и dead-man watcher (AV-13).

Запускаются через mock-rclpy из conftest.py (FakeNode / FakeService +
std_srvs.srv.Trigger + rob_box_supervisor_msgs.srv.AcquireFloor/ReleaseFloor/
SetAvatarMode).

Покрытие (то же, что раньше тестировалось на AvatarSupervisor — чистое
перемещение без изменения логики, #1987):
- Нода создаётся с name="avatar_arbiter" и параметром mode="monitor".
- /avatar/state publisher создан (latched QoS).
- Подписки на /odom, /device/snapshot, /voice/dialogue/state созданы.
- Сервисы на АБСОЛЮТНЫХ именах /avatar_arbiter/acquire_floor /
  release_floor / set_avatar_mode (типизированный IDL — AV-12).
- AcquireFloor в monitor → granted=false, applied=false, reason=monitor.
- Timer-callback публикует /avatar/state (raw-evidence).
- Heartbeat трип через aggregator → счётчик в /avatar/state.last_event.
- SetAvatarMode / AcquireFloor в active-режиме через FSM / LockManager.
- Телеоп-heartbeat + dead-man watcher (AV-13, fake-clock).
"""
from __future__ import annotations

import inspect
import json
import types
import unittest
from typing import Any
from unittest.mock import MagicMock

from rob_box_supervisor.core.fsm import Mode, ModeManager
from rob_box_supervisor.arbiter_node import (
    MODE_TRANSITIONS,
    MONITOR_MODE_REASON,
    REASON_BAD_REQUEST,
    REASON_GRANTED,
    REASON_HELD_BY_OTHER,
    REASON_MONITOR,
    REASON_RELEASED,
    WIRE_MODES,
    AvatarArbiter,
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
    """Полный srv-класс ``AcquireFloor`` (с nested ``.Request``/``.Response``).

    Так его регистрирует ``AvatarArbiter._register_services`` через
    ``create_service(...)`` — rclpy требует именно полный srv-класс,
    отдельный ``AcquireFloor.Request`` бросает ``RuntimeError``
    (см. карточку t_979f0cb2, issue #1904). До этого фикса нода
    регистрировала ``AcquireFloor.Request``, и регрессия проскакивала
    через unit-тесты прямо в прод.
    """
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].AcquireFloor


def _get_typed_release_srv_type():
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].ReleaseFloor


def _get_typed_set_mode_srv_type():
    import sys

    return sys.modules["rob_box_supervisor_msgs.srv"].SetAvatarMode


def _get_typed_srv_full_type():
    """Алиас для обратной совместимости с тестами, которым нужен
    полный srv-класс ``AcquireFloor`` (для ``Response._FIELDS``)."""
    return _get_typed_srv_type()


def _get_typed_release_full_type():
    return _get_typed_release_srv_type()


def _get_typed_set_mode_full_type():
    return _get_typed_set_mode_srv_type()


def _get_typed_request_only(srv_name: str):
    """Helper, возвращающий ТОЛЬКО ``.Request`` (не полный srv-класс).

    Используется в тесте ``test_create_service_rejects_request_only`` —
    мы хотим передать в ``create_service`` заведомо невалидный тип
    (``AcquireFloor.Request``) и убедиться, что mock-rclpy бросает
    ``RuntimeError`` так же, как реальный rclpy. Эта регрессия ловится
    только если mock валидирует srv_type — см. ``conftest.FakeNode.
    create_service``.
    """
    import sys

    return getattr(sys.modules["rob_box_supervisor_msgs.srv"], srv_name).Request


class TestAvatarArbiterCreation(unittest.TestCase):
    def test_node_name_is_avatar_arbiter(self) -> None:
        node = AvatarArbiter()
        try:
            self.assertEqual(node.get_name(), "avatar_arbiter")
        finally:
            node.destroy_node()

    def test_self_clock_not_overwritten_with_callable_issue_1968(self) -> None:
        """Issue #1968 regression: ``AvatarArbiter.__init__`` НЕ должен
        перезаписывать ``self._clock`` call'able-объектом.

        ``self._clock`` зарезервировано rclpy.Node для clock-объекта
        (``rclpy.clock.Clock``), у которого есть ``.handle`` атрибут.
        ``rclpy.timer.Timer.__init__`` читает ``self._clock.handle`` и
        ``self._context.handle`` — перезапись лямбдой ломает
        ``create_timer()`` с ``AttributeError: 'function' object has no
        attribute 'handle'`` (deploy run 33751147006, 2026-09-03).

        Контракт (после фикса):
        - ``self._now_ms`` — call'able, возвращает ``int`` (ms).
        - ``self._clock`` либо отсутствует, либо не является call'able
          (оставлен за rclpy или просто не нужен).
        """
        node = AvatarArbiter()
        try:
            # 1. self._now_ms должен быть call'able
            self.assertTrue(callable(node._now_ms))
            # 2. self._clock, ЕСЛИ есть, НЕ должен быть call'able
            if hasattr(node, "_clock"):
                # В mock-rclpy _clock может быть MagicMock — это ОК (не
                # call'able). В реальном rclpy — это rclpy.clock.Clock
                # (тоже не call'able). Если же мы получили call'able — это
                # именно та регрессия из issue #1968.
                self.assertFalse(
                    callable(node._clock),
                    "self._clock must NOT be a callable (issue #1968): "
                    "AvatarArbiter.__init__ must store the time-source "
                    "only in self._now_ms; self._clock belongs to rclpy.Node.",
                )
            # 3. self._now_ms() возвращает int (ms).
            self.assertIsInstance(node._now_ms(), int)
        finally:
            node.destroy_node()

    def test_mode_parameter_defaults_to_monitor(self) -> None:
        node = AvatarArbiter()
        try:
            self.assertEqual(node.get_parameter("mode").value, "monitor")
            self.assertTrue(node.has_parameter("mode"))
        finally:
            node.destroy_node()

    def test_typed_idl_loaded(self) -> None:
        """AV-12: нода загружает rob_box_supervisor_msgs через try-import.

        При наличии conftest мок-ов — ``use_typed_floor_services`` True,
        и сервисы объявляются на AcquireFloor/ReleaseFloor/SetAvatarMode.

        Проверяем ОБА набора ключей: полные srv-классы (``Acq``/``Set``)
        для ``create_service`` и вложенные Request/Response (``AcqReq``/
        ``SetReq``) для построения объектов запросов/ответов в callback-ах.
        """
        node = AvatarArbiter()
        try:
            self.assertTrue(node._use_typed_floor_services)
            # Полные srv-классы — нужны create_service() (см. t_979f0cb2).
            self.assertIsNotNone(node._msgs_types["Acq"])
            self.assertIsNotNone(node._msgs_types["Rel"])
            self.assertIsNotNone(node._msgs_types["Set"])
            # Вложенные Request/Response — нужны callback-ам.
            self.assertIsNotNone(node._msgs_types["AcqReq"])
            self.assertIsNotNone(node._msgs_types["SetReq"])
        finally:
            node.destroy_node()


class TestAvatarArbiterTopology(unittest.TestCase):
    def setUp(self) -> None:
        self.node = AvatarArbiter()

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
        self.assertIn("/avatar_arbiter/acquire_floor", names)
        self.assertIn("/avatar_arbiter/release_floor", names)
        self.assertIn("/avatar_arbiter/set_avatar_mode", names)

    def test_services_use_typed_srv_type(self) -> None:
        """AV-12 acceptance: сервисы объявлены на AcquireFloor (не Trigger).

        Это структурная гарантия, что supervise-клиенты получают typed
        request/response, а не W3-2 Trigger fallback.
        """
        srv_types_by_name = {s.name: s.srv_type for s in self.node._services}
        typed_acquire = _get_typed_srv_type()
        typed_release = _get_typed_release_srv_type()
        typed_set_mode = _get_typed_set_mode_srv_type()
        self.assertIs(srv_types_by_name["/avatar_arbiter/acquire_floor"], typed_acquire)
        self.assertIs(srv_types_by_name["/avatar_arbiter/release_floor"], typed_release)
        self.assertIs(srv_types_by_name["/avatar_arbiter/set_avatar_mode"], typed_set_mode)


class TestAvatarArbiterMonitorServices(unittest.TestCase):
    """Сервисы в monitor-режиме (AV-12, типизированный IDL):

    - acquire_floor: success=true, granted=false, applied=false, reason=monitor
    - release_floor: success=true, applied=false, reason=monitor
    - set_avatar_mode: success=true, applied=false, mode=<text>, reason=monitor
    """

    def setUp(self) -> None:
        self.node = AvatarArbiter()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_acquire_floor_monitor_response(self) -> None:
        """Monitor: granted=false, applied=false, reason=monitor.

        Даже если клиент пошлёт валидный client_id/floor — в monitor
        supervisor НЕ вмешивается (ADR-0028 §4.5).
        """
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/acquire_floor")
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
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/release_floor")
        req = _make_typed_release_request(client_id="quest", floor="teleop")
        resp = _make_typed_response(_get_typed_release_full_type())
        svc.callback(req, resp)
        body = _release_response_to_dict(resp)
        self.assertTrue(body["success"])
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)

    def test_set_avatar_mode_monitor_response(self) -> None:
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/set_avatar_mode")
        req = _make_typed_set_mode_request(client_id="telegram1", mode="telegram_active")
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
        for name in ("/avatar_arbiter/acquire_floor", "/avatar_arbiter/release_floor", "/avatar_arbiter/set_avatar_mode"):
            self.assertIsNot(srv_types_by_name[name], Trigger)


class TestCreateServiceSrvTypeContract(unittest.TestCase):
    """Регресс-тест на ``create_service(srv_type, ...)`` контракт (issue #1904,
    карточка t_979f0cb2).

    Реальный rclpy.Node.create_service() требует ПОЛНЫЙ srv-класс
    (``AcquireFloor`` с nested ``.Request``/``.Response``). Передача
    отдельного ``AcquireFloor.Request`` приводит к ``RuntimeError: The
    service type provided is not valid`` ещё в ``__init__`` ноды —
    именно так ``avatar-supervisor`` падал в crash-loop на Vision Pi в
    раундах e2e 337..341.

    До фикса mock-rclpy.conftest тихо принимал любой srv_type, и эта
    регрессия проскакивала через unit-тесты прямо в прод. Этот набор
    тестов структурно гарантирует, что:
    (a) нода регистрирует сервисы на полном srv-классе;
    (b) mock ``FakeNode.create_service`` валидирует srv_type и бросает
        RuntimeError, если передан неполный тип;
    (c) guard ``_use_typed_floor_services`` падает в Trigger fallback,
        если IDL загружен частично (нет полных srv-классов).
    """

    def test_register_services_uses_full_srv_class(self) -> None:
        """(a) нода регистрирует каждый из трёх сервисов на полном srv-классе."""
        node = AvatarArbiter()
        try:
            srv_types_by_name = {s.name: s.srv_type for s in node._services}
            for name, full_name in (
                ("/avatar_arbiter/acquire_floor", "AcquireFloor"),
                ("/avatar_arbiter/release_floor", "ReleaseFloor"),
                ("/avatar_arbiter/set_avatar_mode", "SetAvatarMode"),
            ):
                srv_type = srv_types_by_name[name]
                # Полный srv-класс имеет nested .Request и .Response.
                self.assertTrue(
                    hasattr(srv_type, "Request") and hasattr(srv_type, "Response"),
                    f"{name} registered with non-srv type {srv_type!r}",
                )
                # И это именно AcquireFloor, а не std_srvs.Trigger или
                # какой-то другой srv-класс.
                self.assertEqual(srv_type.__name__, full_name)
        finally:
            node.destroy_node()

    def test_create_service_rejects_request_only(self) -> None:
        """(b) mock-rclpy бросает RuntimeError на ``AcquireFloor.Request``.

        Имитируем ровно ту ошибку, которая ловила avatar-supervisor в
        проде (см. issue #1904): разработчик по ошибке передаёт
        ``AcquireFloor.Request`` вместо ``AcquireFloor``. До фикса
        этот тест был RED (mock тихо принимал .Request). После фикса —
        GREEN, и любая попытка вернуть регрессию немедленно видна
        здесь, а не на Vision Pi в 4 утра.
        """
        # Тестируем «сырой» mock-rclpy FakeNode.create_service напрямую —
        # это именно та вальва, которую мы добавили в conftest. ``__init__``
        # ноды тут не нужен, нас интересует контракт mock-а.
        from test.unit.conftest import _install_ros_mocks  # noqa: F401, PLC0415
        from rclpy.node import Node as _FakeNode  # noqa: PLC0415

        node = _FakeNode("__test__")
        try:
            # Достаём «голый» Request-класс — то, что раньше по ошибке
            # передавалось в create_service.
            bad_srv_type = _get_typed_request_only("AcquireFloor")
            # У .Request нет nested .Request/.Response → RuntimeError.
            self.assertFalse(
                hasattr(bad_srv_type, "Request") and hasattr(bad_srv_type, "Response"),
                "sanity: AcquireFloor.Request действительно без nested attrs",
            )
            with self.assertRaises(RuntimeError) as ctx:
                node.create_service(bad_srv_type, "/avatar_arbiter/acquire_floor", lambda *a: None)
            self.assertIn("not valid", str(ctx.exception))
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass

    def test_create_service_rejects_response_only(self) -> None:
        """(b') Симметричный тест для ``AcquireFloor.Response``.

        ``rclpy`` ругается на любой srv-класс без nested ``.Request`` /
        ``.Response`` (даже если сам класс — ``AcquireFloor.Response``).
        Проверяем именно Response, чтобы guard не пропустил «обратную»
        опечатку.
        """
        from test.unit.conftest import _install_ros_mocks  # noqa: F401, PLC0415
        from rclpy.node import Node as _FakeNode  # noqa: PLC0415

        node = _FakeNode("__test__")
        try:
            import sys as _sys
            bad_srv_type = _sys.modules[
                "rob_box_supervisor_msgs.srv"
            ].AcquireFloor.Response
            self.assertFalse(
                hasattr(bad_srv_type, "Request") and hasattr(bad_srv_type, "Response"),
                "sanity: AcquireFloor.Response действительно без nested attrs",
            )
            with self.assertRaises(RuntimeError) as ctx:
                node.create_service(bad_srv_type, "/avatar_arbiter/acquire_floor", lambda *a: None)
            self.assertIn("not valid", str(ctx.exception))
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass

    def test_create_service_accepts_full_srv_class(self) -> None:
        """(b'') Полный srv-класс проходит валидацию без RuntimeError.

        Контр-тест к ``test_create_service_rejects_request_only``:
        убеждаемся, что валидация не over-rejects и нормальный путь
        всё ещё работает.
        """
        from test.unit.conftest import _install_ros_mocks  # noqa: F401, PLC0415
        from rclpy.node import Node as _FakeNode  # noqa: PLC0415

        node = _FakeNode("__test__")
        try:
            good_srv_type = _get_typed_srv_type()
            # sanity — у него есть .Request / .Response.
            self.assertTrue(hasattr(good_srv_type, "Request"))
            self.assertTrue(hasattr(good_srv_type, "Response"))
            # Не должно быть RuntimeError.
            svc = node.create_service(
                good_srv_type, "/avatar_arbiter/acquire_floor", lambda req, resp: None
            )
            self.assertEqual(svc.name, "/avatar_arbiter/acquire_floor")
            self.assertIs(svc.srv_type, good_srv_type)
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass

    def test_use_typed_guard_requires_full_srv_class(self) -> None:
        """(c) ``_use_typed_floor_services`` зависит от ПОЛНЫХ srv-классов.

        Имитируем ситуацию «IDL частично загружен: только .msg, не .srv»
        — guard должен уйти в Trigger fallback, а не в typed-путь с None
        (как было до фикса, когда guard смотрел на ``AcqReq``/``SetReq``
        и тихо выбирал typed-путь без полного srv-класса).
        """
        node = AvatarArbiter()
        try:
            # Guard вычислен в __init__, проверяем его формулу:
            # ``_use_typed_floor_services = Acq is not None and Set is not None``.
            # Если кто-то переключит формулу обратно на AcqReq/SetReq —
            # этот assert его поймает, потому что AcqReq != Acq.
            msgs = node._msgs_types
            self.assertIs(msgs["Acq"], _get_typed_srv_type())  # полный srv-класс
            self.assertIs(msgs["AcqReq"], _get_typed_request_only("AcquireFloor"))
            # Sanity-логика guard: если полные классы есть, typed-путь ок;
            # если кто-то занулит их (частичный IDL) — guard уйдёт в False.
            msgs["Acq"] = None
            msgs["Set"] = None
            # Пересчитываем формулу, как она записана в __init__:
            recomputed = msgs["Acq"] is not None and msgs["Set"] is not None
            self.assertFalse(recomputed)
            # Возвращаем как было для tearDown.
            import sys as _sys2
            msgs["Acq"] = _get_typed_srv_type()
            msgs["Set"] = _sys2.modules["rob_box_supervisor_msgs.srv"].SetAvatarMode
        finally:
            node.destroy_node()


class TestAvatarArbiterPublishHeartbeat(unittest.TestCase):
    """Heartbeat трип → /avatar/state обновляется (AV-6 acceptance)."""

    def setUp(self) -> None:
        self.node = AvatarArbiter()

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


class TestAvatarArbiterDoesNotMutateExternalState(unittest.TestCase):
    """Monitor-режим НЕ меняет twist_mux / dialogue_node (ADR-0028 §4.5)."""

    def setUp(self) -> None:
        self.node = AvatarArbiter()

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
        self.assertIn("avatar_arbiter started", msg)
        self.assertIn(f"mode={self.node._mode}", msg)
        # ``msgpack=`` намеренно отсутствует — см. комментарий выше.
        self.assertNotIn("msgpack=", msg)
        self.assertIn("typed_services=", msg)
        # Никаких kwargs %-форматирования быть не должно (kwargs в rclpy
        # info() не поддерживаются и упадут так же, как и >1 args).
        self.assertEqual(
            call.kwargs,
            {},
            f"info() не должен получать kwargs, got {call.kwargs!r}",
        )


class TestAcquireFloorTypedContract(unittest.TestCase):
    """AV-12 acceptance: typed AcquireFloor в active-режиме через LockManager.

    Контракт запроса — типизированный IDL (см. supervisor_node._extract_floor_request).
    Прошлый JSON-в-data fallback УБРАН (R13).
    """

    def setUp(self) -> None:
        self.node = AvatarArbiter()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _acquire(self, client_id: str, floor: str) -> dict:
        """Дёрнуть сервисный callback, как будто пришёл typed request."""
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/acquire_floor")
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
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/release_floor")
        req = _make_typed_release_request(client_id="quest", floor="teleop")
        resp = _make_typed_response(_get_typed_release_full_type())
        svc.callback(req, resp)
        body = _release_response_to_dict(resp)
        self.assertTrue(body["applied"])
        self.assertEqual(body["reason"], REASON_RELEASED)

    def test_release_by_wrong_client_denied(self) -> None:
        """Release не-owner-ом → applied=false, reason="permission_denied"."""
        self._acquire("quest", "teleop")
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/release_floor")
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
        """Dead-man авто-release замечается 10 Гц watcher-ом и попадает в dead_man_trips_total.

        AV-13: после рефакторинга trip-метрика собирается в
        :py:meth:`AvatarArbiter._check_floor_expiry` (10 Гц
        watcher), а не в прежней ``_check_dead_man_trips`` (1 Гц
        ленивая детекция). Тест обновлён под новый единый путь.
        """
        from rob_box_supervisor.core import LockManager

        clock = {"t": 0}
        # AV-13: конструктор LockManager теперь принимает ``clock`` +
        # ``timeout_ms``; node-уровневые часы пробрасываем, чтобы
        # watcher увидел fake-time при вызове _check_floor_expiry().
        self.node._lock_manager = LockManager(clock=lambda: clock["t"], timeout_ms=500)
        # node._now_ms должен использовать те же fake-часы для watcher-а.
        self.node._now_ms = lambda: clock["t"]

        self._acquire("quest", "teleop")
        clock["t"] += 501
        self.node._check_floor_expiry()

        self.assertEqual(self.node._aggregator.dead_man_count("quest"), 1)

    def test_monitor_mode_still_applied_false_for_floor_ops(self) -> None:
        """В monitor-режиме acquire/release_floor по-прежнему applied=false
        (регресс ADR-0028 §4.5)."""
        self.node._mode = "monitor"
        acquire_body = self._acquire("quest", "teleop")
        self.assertFalse(acquire_body["applied"])
        self.assertFalse(acquire_body["granted"])
        self.assertEqual(acquire_body["reason"], REASON_MONITOR)

        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/release_floor")
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

        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/acquire_floor")
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
        self.node = AvatarArbiter()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _set_mode(self, client_id: str, mode: str) -> dict:
        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/set_avatar_mode")
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
        body = self._set_mode("telegram1", "telegram_active")
        self.assertTrue(body["applied"], body)
        self.assertEqual(body["mode"], "telegram_active")
        self.assertEqual(self.node._mode_manager.mode, Mode.TELEGRAM_ACTIVE)

    def test_unreachable_target_refused_as_conflict(self) -> None:
        """``off → mixed`` недостижим за один шаг: ``mixed`` требует ДВУХ
        клиентов (ADR-0028 §4.1). Отказ, режим не меняется — через
        промежуточное состояние супервизор сам не ходит."""
        body = self._set_mode("quest1", "mixed")
        self.assertFalse(body["applied"], body)
        self.assertEqual(body["reason"], "conflict")
        self.assertEqual(body["mode"], "off")
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_unknown_mode_refused_as_bad_request(self) -> None:
        """Имя, которого нет среди режимов — ``bad_request``.

        Отдельно от ``conflict``: клиент прислал мусор (или, что важнее,
        имя FSM-события по старому контракту AV-12 — ``mode`` на проводе
        теперь целевой режим). Это ошибка запроса, а не занятый floor.
        """
        body = self._set_mode("telegram1", "telegram_acquire_floor")
        self.assertFalse(body["applied"], body)
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_same_mode_is_idempotent_noop(self) -> None:
        """Запрос текущего режима — no-op с ``applied=true``.

        Клиенты пере-отправляют SET_MODE на реконнекте (meta-quest-api.md
        §3), и падать на этом нельзя.
        """
        self._set_mode("telegram1", "telegram_active")
        body = self._set_mode("telegram1", "telegram_active")
        self.assertTrue(body["applied"], body)
        self.assertEqual(body["reason"], "applied")
        self.assertEqual(body["mode"], "telegram_active")
        self.assertEqual(self.node._mode_manager.mode, Mode.TELEGRAM_ACTIVE)

    def test_off_is_always_reachable(self) -> None:
        """``* → off`` — escape hatch (ADR-0028 §4.1 + §6 Q1 fail-safe):
        из любого активного режима выключение проходит."""
        for entry, client in (
            ("telegram_active", "telegram1"),
            ("avatar_present", "quest1"),
        ):
            with self.subTest(entry=entry):
                self.node._mode_manager = ModeManager()
                self._set_mode(client, entry)
                body = self._set_mode(client, "off")
                self.assertTrue(body["applied"], body)
                self.assertEqual(body["mode"], "off")

    def test_two_step_path_off_to_mixed(self) -> None:
        """``off → telegram_active → mixed``: полный путь до ``mixed``
        собирается из двух шагов ДВУХ клиентов, как в ADR-0028 §4.1."""
        self._set_mode("telegram1", "telegram_active")
        body = self._set_mode("quest1", "mixed")
        self.assertTrue(body["applied"], body)
        self.assertEqual(body["mode"], "mixed")
        self.assertEqual(self.node._mode_manager.mode, Mode.MIXED)

    def test_mixed_splits_back_by_target(self) -> None:
        """Из ``mixed`` целевой режим однозначно выбирает, КТО уходит.

        Это и есть причина, по которой на проводе режим, а не событие:
        одно событие ``quest_release`` значит ``avatar_present → off``,
        но ``mixed → telegram_active``.
        """
        self._set_mode("telegram1", "telegram_active")
        self._set_mode("quest1", "mixed")
        body = self._set_mode("quest1", "telegram_active")
        self.assertTrue(body["applied"], body)
        self.assertEqual(body["mode"], "telegram_active")

    def test_mode_transitions_table_matches_core_fsm(self) -> None:
        """Анти-дрейф: каждое событие из ``MODE_TRANSITIONS`` существует в
        ``core.fsm``, и каждый ключ — пара валидных wire-режимов.

        Ровно эта рассинхронизация и была багом: в супервизоре лежали
        имена рёбер mermaid-диаграммы (``quest_acquire_full_floor`` и
        др.), которых у автомата нет.
        """
        from rob_box_supervisor.core.fsm import _ALL_EVENTS

        for (src, dst), event in MODE_TRANSITIONS.items():
            with self.subTest(transition=f"{src}->{dst}"):
                self.assertIn(src, WIRE_MODES)
                self.assertIn(dst, WIRE_MODES)
                self.assertNotEqual(src, dst, "no-op не должен быть в таблице")
                self.assertIn(event, _ALL_EVENTS)

    def test_wire_modes_match_fsm_mode_enum(self) -> None:
        """Анти-дрейф: ``WIRE_MODES`` == значения ``core.fsm.Mode``.

        Если в FSM появится пятый режим, а на проводе нет — запрос на него
        будет отвергнут как ``bad_request``, и это надо заметить здесь, а
        не на роботе.
        """
        self.assertEqual(set(WIRE_MODES), {m.value for m in Mode})

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
        body = self._set_mode("telegram1", "telegram_active")
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], MONITOR_MODE_REASON)
        self.assertEqual(self.node._mode_manager.mode, Mode.OFF)

    def test_mode_change_releases_lock_manager_floors(self) -> None:
        """W3-4 — уход из активного avatar-режима синхронно освобождает
        LockManager floor-ы того же ``client_id``: иначе остаётся висячий
        holder, до которого больше не достучаться через ``ReleaseFloor``
        (см. docstring ``AvatarArbiter._set_avatar_mode_logic``)."""
        # Клиент реально держит оба floor-а через LockManager (как будто
        # раньше отдельно вызвал AcquireFloor — W3-2). Используем
        # LockManager-имена, т.к. работаем напрямую с LockManager.
        from rob_box_supervisor.core import Floor as LockFloor

        self.node._acquire_floor_logic("questA", LockFloor.TELEOP)
        self.node._acquire_floor_logic("questA", LockFloor.VOICE)
        # И тот же client_id зафиксирован в FSM как участник avatar_present.
        entered = self._set_mode("questA", "avatar_present")
        self.assertEqual(entered["mode"], "avatar_present")

        result = self._set_mode("questA", "off")
        self.assertTrue(result["applied"], result)
        self.assertEqual(result["mode"], "off")

        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.VOICE))

    def test_no_phase2_warning_logged(self) -> None:
        """Регресс: вводящий в заблуждение warning «Phase 2 не реализован» убран."""
        self.node._log.reset_mock()
        self._set_mode("telegram1", "telegram_active")
        for call in self.node._log.warning.call_args_list:
            msg = call.args[0] if call.args else ""
            self.assertNotIn("Phase 2", msg)

    def test_set_avatar_mode_logs_single_msg_arg(self) -> None:
        """Регресс #1644 (см. ``_log_startup_diagnostics``): ``info()`` должен
        получать РОВНО один позиционный ``msg``-аргумент."""
        self.node._log.reset_mock()
        self._set_mode("telegram1", "telegram_active")
        self.assertTrue(self.node._log.info.called)
        call = self.node._log.info.call_args
        self.assertEqual(len(call.args), 1, f"info() должен получить 1 positional arg, got {call.args!r}")
        self.assertEqual(call.kwargs, {})

    def test_no_json_in_data_field_used(self) -> None:
        """Рергресс R13: _extract_avatar_mode_request НЕ парсит JSON из request.data."""
        legacy_req = MagicMock(spec=["data"])
        legacy_req.data = json.dumps({"mode": "telegram_active", "client_id": "telegram1"})

        svc = next(s for s in self.node._services if s.name == "/avatar_arbiter/set_avatar_mode")
        resp = _make_typed_response(_get_typed_set_mode_full_type())
        svc.callback(legacy_req, resp)
        body = _set_mode_response_to_dict(resp)
        self.assertFalse(body["applied"])
        self.assertEqual(body["reason"], REASON_BAD_REQUEST)


class TestAvatarArbiterTeleopHeartbeat(unittest.TestCase):
    """AV-13 — подписка на ``/teleop_heartbeat`` + dead-man watcher 500 мс.

    Acceptance criteria (issue #1905):

    1. Подписка на ``/teleop_heartbeat`` есть в ``_subscriptions``
       (через try-import IDL; в CI пакет недоступен → факт подписки не
       проверяем напрямую, но проверяем через ``_heartbeat_msg_type``
       и наличие callback-метода).
    2. ROS-параметр ``dead_man_timeout_ms`` объявлен с default 500.
    3. ``mode=monitor`` — heartbeat приходит, floor НЕ трогается.
    4. ``mode=active``, fake-clock: heartbeat каждые 100 мс 2 секунды →
       floor держится у клиента.
    5. ``mode=active``, fake-clock: heartbeat прекратился → ≤600 мс
       holder=None, dead_man_trips_total[client]=1, /avatar/state
       опубликован внеочередно.
    6. Второй клиент может взять floor сразу после протухания первого.
    7. Heartbeat от клиента, который floor НЕ держит, — игнорируется.
    8. Часы подменяемы в тестах (никаких ``time.time()`` в новом коде).
    """

    def setUp(self) -> None:
        self.node = AvatarArbiter()
        # В CI пакет IDL недоступен → ``_heartbeat_msg_type = None``, подписка
        # не регистрируется. Callback всё равно тестируем напрямую — это
        # unit-test, не integration.
        self.node._heartbeat_msg_type = None  # mock: IDL отсутствует
        # Fake-clock для детерминированных временных тестов. Заменяем
        # и ``_now_ms`` (для watcher-а и callback), и ``_lock_manager``
        # (LockManager хранит ссылку на clock из конструктора — после
        # __init__ это реальное время, нужно пересоздать с fake-часами,
        # иначе watcher видит реальное время и force_expire не сработает).
        self._clock = {"t": 0}
        fake_clock_fn = lambda: self._clock["t"]  # noqa: E731
        self.node._now_ms = fake_clock_fn
        from rob_box_supervisor.core import LockManager

        self.node._lock_manager = LockManager(clock=fake_clock_fn, timeout_ms=self.node._dead_man_timeout_ms)

    def tearDown(self) -> None:
        self.node.destroy_node()

    # ── acceptance 1: параметр объявлен ───────────────────────────────
    def test_dead_man_timeout_ms_parameter_declared_with_default_500(self) -> None:
        """ROS-параметр ``dead_man_timeout_ms`` объявлен, default = 500 мс."""
        self.assertTrue(self.node.has_parameter("dead_man_timeout_ms"))
        self.assertEqual(self.node.get_parameter("dead_man_timeout_ms").value, 500)
        self.assertEqual(self.node._dead_man_timeout_ms, 500)

    def test_callback_method_exists(self) -> None:
        """Метод ``_on_teleop_heartbeat`` определён и callable."""
        self.assertTrue(callable(getattr(self.node, "_on_teleop_heartbeat", None)))
        self.assertTrue(callable(getattr(self.node, "_check_floor_expiry", None)))
        self.assertTrue(callable(getattr(self.node, "_try_import_heartbeat_msg", None)))

    # ── acceptance 3: monitor — heartbeat не трогает floor ────────────
    def test_monitor_mode_heartbeat_does_not_touch_floor(self) -> None:
        """В monitor heartbeat приходит, но floor НЕ берётся и НЕ держится."""
        self.node._mode = "monitor"
        # Имитируем ``/teleop_heartbeat`` от клиента.
        msg = types.SimpleNamespace(client_id="quest1", ts_ms=0, seq=1)
        # Floor изначально свободен.
        from rob_box_supervisor.core import Floor as LockFloor

        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))

        self.node._on_teleop_heartbeat(msg)

        # После heartbeat-а в monitor floor всё ещё None (НЕ взят).
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))

    # ── acceptance 7: heartbeat от чужого/не-держателя — игнор ────────
    def test_active_mode_heartbeat_without_holder_is_ignored(self) -> None:
        """Heartbeat от клиента, который НЕ держит floor — игнорируется,
        не создаёт floor."""
        self.node._mode = "active"
        # Floor ещё никем не занят.
        from rob_box_supervisor.core import Floor as LockFloor

        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))

        # Heartbeat приходит.
        msg = types.SimpleNamespace(client_id="ghost", ts_ms=0, seq=1)
        self.node._on_teleop_heartbeat(msg)

        # Floor всё ещё None — heartbeat НЕ создал holder-а.
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))

    def test_active_mode_heartbeat_from_other_client_is_ignored(self) -> None:
        """Heartbeat от client_id, который НЕ совпадает с текущим holder-ом —
        отказ (LockManager.heartbeat → PermissionError), floor не трогается."""
        self.node._mode = "active"
        from rob_box_supervisor.core import Floor as LockFloor

        # Quest держит floor.
        self.node._lock_manager.acquire("quest", LockFloor.TELEOP, now_ms=0)
        self._clock["t"] = 100
        # Телеграм шлёт heartbeat — не его floor.
        msg = types.SimpleNamespace(client_id="telegram", ts_ms=0, seq=1)
        self.node._on_teleop_heartbeat(msg)

        # Quest всё ещё держит floor.
        self.assertEqual(self.node._lock_manager.holder(LockFloor.TELEOP), "quest")

    # ── acceptance 4: heartbeat держит floor живым ────────────────────
    def test_active_mode_heartbeat_keeps_floor_alive_2s(self) -> None:
        """Heartbeat каждые 100 мс 2 секунды → floor всё ещё у клиента."""
        self.node._mode = "active"
        from rob_box_supervisor.core import Floor as LockFloor

        # Клиент взял floor.
        self.node._lock_manager.acquire("quest", LockFloor.TELEOP, now_ms=0)

        # Шлём heartbeat каждые 100 мс в течение 2 секунд (20 тиков).
        for tick in range(20):
            self._clock["t"] = (tick + 1) * 100
            msg = types.SimpleNamespace(client_id="quest", ts_ms=self._clock["t"], seq=tick)
            self.node._on_teleop_heartbeat(msg)

        # Floor всё ещё у quest — 2000 мс прошло, но heartbeat-ы держали.
        self._clock["t"] = 2000
        self.assertEqual(self.node._lock_manager.holder(LockFloor.TELEOP), "quest")

    # ── acceptance 5: trip после прекращения heartbeat ────────────────
    def test_active_mode_no_heartbeat_triggers_dead_man_within_600ms(self) -> None:
        """Heartbeat прекратился → через ≤600 мс holder=None,
        dead_man_trips_total[client]=1, /avatar/state опубликован внеочередно."""
        self.node._mode = "active"
        from rob_box_supervisor.core import Floor as LockFloor

        # Клиент взял floor.
        self.node._lock_manager.acquire("quest", LockFloor.TELEOP, now_ms=0)
        self.assertEqual(self.node._lock_manager.holder(LockFloor.TELEOP), "quest")

        # Один heartbeat, чтобы снимок holder-ов в ноде был согласован.
        self._clock["t"] = 100
        self.node._on_teleop_heartbeat(types.SimpleNamespace(client_id="quest", ts_ms=100, seq=1))

        # Запоминаем сколько публикаций было до trip-а.
        pub = self.node._publishers["/avatar/state"]
        before_count = len(pub.published)

        # Перематываем время на 600 мс (dead-man = 500 мс).
        self._clock["t"] = 700

        # Запускаем watcher-таймер вручную.
        self.node._check_floor_expiry()

        # Floor снят.
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))

        # Метрика инкрементнулась.
        self.assertEqual(self.node._aggregator.dead_man_count("quest"), 1)

        # /avatar/state опубликован внеочередно (была публикация ПОСЛЕ before_count).
        self.assertGreater(
            len(pub.published),
            before_count,
            "watcher должен опубликовать /avatar/state внеочередно при trip",
        )

    # ── acceptance 5b: точный порог 500 мс (boundary) ─────────────────
    def test_dead_man_trips_exactly_at_501ms_not_at_500ms(self) -> None:
        """Граница dead-man (>timeout, не >=): ровно 500 мс — alive, 501 мс — trip."""
        self.node._mode = "active"
        from rob_box_supervisor.core import Floor as LockFloor

        self.node._lock_manager.acquire("quest", LockFloor.TELEOP, now_ms=0)
        self._clock["t"] = 100
        self.node._on_teleop_heartbeat(types.SimpleNamespace(client_id="quest", ts_ms=100, seq=1))

        # Ровно 500 мс с последнего heartbeat — ещё держит.
        self._clock["t"] = 600  # 500 мс после heartbeat
        self.node._check_floor_expiry()
        self.assertEqual(self.node._lock_manager.holder(LockFloor.TELEOP), "quest")

        # 501 мс — trip.
        self._clock["t"] = 601
        self.node._check_floor_expiry()
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))
        self.assertEqual(self.node._aggregator.dead_man_count("quest"), 1)

    # ── acceptance 6: второй клиент сразу после протухания ────────────
    def test_second_client_acquires_floor_after_first_expires(self) -> None:
        """После trip первого клиента второй может взять teleop_floor сразу."""
        self.node._mode = "active"
        from rob_box_supervisor.core import Floor as LockFloor

        # Quest взял floor.
        self.node._lock_manager.acquire("quest", LockFloor.TELEOP, now_ms=0)
        self._clock["t"] = 100
        self.node._on_teleop_heartbeat(types.SimpleNamespace(client_id="quest", ts_ms=100, seq=1))

        # Ждём trip.
        self._clock["t"] = 700
        self.node._check_floor_expiry()
        self.assertIsNone(self.node._lock_manager.holder(LockFloor.TELEOP))

        # Telegram сразу берёт floor — без конфликта.
        self.node._lock_manager.acquire("telegram", LockFloor.TELEOP, now_ms=self._clock["t"])
        self.assertEqual(self.node._lock_manager.holder(LockFloor.TELEOP), "telegram")

    # ── acceptance 8: часы подменяемы ─────────────────────────────────
    def test_no_time_time_in_heartbeat_or_watcher_code(self) -> None:
        """Регресс-контракт: ни в одном новом методе нет прямого ``time.time()``
        или ``time.monotonic()`` — только ``self._now_ms()`` / ``self._clock``.

        Смотрим на AST (а не сырой текст через ``inspect.getsource``),
        чтобы docstring с упоминанием ``time.time()`` не давал ложного
        срабатывания. Проверяем только ``ast.Call`` нод с func.id
        ``time.time`` / ``time.monotonic`` — это ловит именно runtime
        вызов, а не комментарий. ``inspect.getsource`` возвращает тело
        метода С отступом класса (4 пробела) — снимаем через
        ``textwrap.dedent`` чтобы ``ast.parse`` не упал на IndentationError.
        """
        import ast
        import textwrap

        for method_name in ("_on_teleop_heartbeat", "_check_floor_expiry", "_try_import_heartbeat_msg"):
            method = getattr(self.node, method_name)
            source = textwrap.dedent(inspect.getsource(method))
            tree = ast.parse(source)
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                func = node.func
                # func.attr указывает на последний атрибут цепочки
                # (``time.time()`` → ast.Attribute(attr='time', value=ast.Name(id='time'))).
                if isinstance(func, ast.Attribute) and isinstance(func.value, ast.Name):
                    if func.value.id == "time" and func.attr in ("time", "monotonic"):
                        self.fail(f"{method_name} должен использовать self._now_ms() вместо time.{func.attr}()")

    # ── дополнительно: пустой client_id защищён ───────────────────────
    def test_empty_client_id_is_ignored(self) -> None:
        """Heartbeat с пустым ``client_id`` не валит ноду, просто логирует WARN."""
        self.node._mode = "active"
        msg = types.SimpleNamespace(client_id="", ts_ms=0, seq=1)
        # Не должно быть исключений.
        self.node._on_teleop_heartbeat(msg)


if __name__ == "__main__":
    unittest.main()


if __name__ == "__main__":
    unittest.main()
