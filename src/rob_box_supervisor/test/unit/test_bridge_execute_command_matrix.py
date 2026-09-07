"""Матрица тестов для ``AvatarSupervisor.execute(Command)`` (ADR-0051 §2.2).

Цель: зафиксировать контракт ``Bridge.execute(Command) → Response``
**одним** файлом тестов, чтобы любая регрессия в маршрутизации
``Command.kind`` ловилась сразу, без необходимости читать legacy-тесты
по ``_acquire_floor_logic``/``_set_avatar_mode_logic`` и т.п.

Карточка: t_2da4b2b4 / issue #2002 / ADR-0051 §6 / Шаг 12
``target-operator-agent-and-dialogue.md``.

Что покрываем
-------------

1. **Service topology** — ``/supervisor/execute`` зарегистрирован
   параллельно с legacy (``acquire_floor``, ``release_floor``,
   ``set_avatar_mode``). Никаких breaking changes в Phase 1.

2. **Маршрутизация по ``kind`` (monitor-режим)** — для каждой команды
   ``KIND_ACQUIRE_FLOOR``/``KIND_RELEASE_FLOOR``/``KIND_SET_AVATAR_MODE``/
   ``KIND_SET_VOICE_MODE``/``KIND_EMERGENCY_STOP``/``KIND_HEARTBEAT``
   ответ соответствует существующим ``_*_logic`` методам.

3. **Маршрутизация по ``kind`` (active-режим)** — то же, но с реальным
   применением (LockManager / ModeManager / dialogue-параметр).

4. **Edge cases** — неизвестный ``kind``, пустой ``client_id``, пустой
   ``floor``, невалидный ``voice_mode`` — все отдают внятный ``reason``
   (ADR-0018 — не молчим на отказе).

5. **Сервис-callback** — ``_on_execute_command`` корректно копирует
   поля ``Response`` в ``response.response`` (или прямо в ``response``
   для упрощённых mock-объектов).

Эти тесты **НЕ** проверяют, что ``quest_node`` или ``telegram_node``
вызывают ``Bridge.execute`` — это Phase 2 (отдельные карточки по
ADR-0013 incremental delivery).
"""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    EXECUTE_COMMAND_SERVICE,
    KIND_ACQUIRE_FLOOR,
    KIND_EMERGENCY_STOP,
    KIND_HEARTBEAT,
    KIND_RELEASE_FLOOR,
    KIND_SET_AVATAR_MODE,
    KIND_SET_VOICE_MODE,
    KIND_UNKNOWN,
    MONITOR_MODE_REASON,
    AvatarSupervisor,
)


def _cmd(kind: int, **fields) -> SimpleNamespace:
    """Собрать ``Command``-объект (duck-typed) с заданными полями."""
    defaults = {
        "kind": kind,
        "client_id": "",
        "floor": "",
        "avatar_event": "",
        "voice_mode": "",
        "emergency": False,
    }
    defaults.update(fields)
    return SimpleNamespace(**defaults)


class TestExecuteCommandTopology(unittest.TestCase):
    """/supervisor/execute зарегистрирован в ноде (ADR-0051 §2.2)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_execute_service_registered(self) -> None:
        """Сервис ``/supervisor/execute`` появляется в ``self._services``."""
        names = [s.name for s in self.node._services]
        self.assertIn(EXECUTE_COMMAND_SERVICE, names)

    def test_legacy_services_still_registered(self) -> None:
        """Legacy-сервисы НЕ удалены в Phase 1 (ADR-0013, никаких breaking changes)."""
        names = [s.name for s in self.node._services]
        self.assertIn("acquire_floor", names)
        self.assertIn("release_floor", names)
        self.assertIn("set_avatar_mode", names)


class TestExecuteMonitorMatrix(unittest.TestCase):
    """Матрица ``kind → Response`` в monitor-режиме (default).

    В monitor всё принимается, ничего не применяется (ADR-0028 §4.5,
    S12). ``accepted=true, applied=false, reason=monitor, contacted_service=False``.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _execute(self, kind: int, **fields) -> SimpleNamespace:
        return self.node.execute(_cmd(kind, **fields))

    def test_acquire_floor_monitor(self) -> None:
        resp = self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="voice_floor")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, MONITOR_MODE_REASON)
        self.assertFalse(resp.contacted_service)

    def test_release_floor_monitor(self) -> None:
        resp = self._execute(KIND_RELEASE_FLOOR, client_id="quest", floor="voice_floor")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, MONITOR_MODE_REASON)

    def test_set_avatar_mode_monitor(self) -> None:
        resp = self._execute(
            KIND_SET_AVATAR_MODE,
            client_id="quest",
            avatar_event="quest_acquire_floor",
        )
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, MONITOR_MODE_REASON)
        # actual_mode всё равно отдаётся (для UI/телеметрии), даже
        # когда ничего не применилось (это ModeManager.mode до запроса).
        self.assertEqual(resp.actual_mode, "off")

    def test_set_voice_mode_monitor(self) -> None:
        resp = self._execute(KIND_SET_VOICE_MODE, client_id="quest", voice_mode="quest_ttts")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, MONITOR_MODE_REASON)
        self.assertFalse(resp.contacted_service)

    def test_emergency_stop_monitor(self) -> None:
        """Emergency в monitor: accepted=true, applied=false (Phase 1)."""
        resp = self._execute(KIND_EMERGENCY_STOP, client_id="quest")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, MONITOR_MODE_REASON)

    def test_heartbeat_monitor_no_op(self) -> None:
        resp = self._execute(KIND_HEARTBEAT, client_id="quest", floor="teleop_floor")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, MONITOR_MODE_REASON)


class TestExecuteActiveMatrix(unittest.TestCase):
    """Матрица ``kind → Response`` в active-режиме (LockManager / ModeManager).

    Здесь ``applied=true`` для валидных команд и ``contacted_service=True``
    (т.е. решение принималось с реальным эффектом).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _execute(self, kind: int, **fields) -> SimpleNamespace:
        return self.node.execute(_cmd(kind, **fields))

    def test_acquire_floor_active_granted(self) -> None:
        resp = self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="voice_floor")
        self.assertTrue(resp.accepted)
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, "granted")
        self.assertEqual(resp.held_by, "quest")
        self.assertTrue(resp.contacted_service)

    def test_acquire_floor_active_conflict(self) -> None:
        """Второй клиент получает ``applied=true, granted=false, held_by=<first>``."""
        self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="voice_floor")
        resp = self._execute(KIND_ACQUIRE_FLOOR, client_id="telegram", floor="voice_floor")
        self.assertTrue(resp.accepted)
        self.assertTrue(resp.applied)  # applied=true даже при conflict (мы реально попытались)
        self.assertIn("conflict", resp.reason)
        self.assertIn("quest", resp.reason)
        self.assertEqual(resp.held_by, "quest")

    def test_release_floor_active_releases(self) -> None:
        self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="voice_floor")
        resp = self._execute(KIND_RELEASE_FLOOR, client_id="quest", floor="voice_floor")
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, "released")
        self.assertEqual(resp.held_by, "")  # floor свободен

    def test_release_floor_by_wrong_client(self) -> None:
        self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="voice_floor")
        resp = self._execute(KIND_RELEASE_FLOOR, client_id="telegram", floor="voice_floor")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertIn("permission_denied", resp.reason)

    def test_set_avatar_mode_active_transition(self) -> None:
        resp = self._execute(
            KIND_SET_AVATAR_MODE,
            client_id="telegram1",
            avatar_event="telegram_acquire_floor",
        )
        self.assertTrue(resp.applied)
        self.assertEqual(resp.actual_mode, "telegram_active")

    def test_set_voice_mode_active_dispatches(self) -> None:
        """В active voice_mode реально отправляется в ``_set_dialogue_param``."""
        self.node._set_dialogue_param = MagicMock()
        resp = self._execute(KIND_SET_VOICE_MODE, client_id="quest", voice_mode="quest_ttts")
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with("voice_input_mode", "quest_ttts")

    def test_set_voice_mode_invalid_mode(self) -> None:
        """Невалидный режим → applied=false, reason с 'invalid_voice_mode' (ADR-0018)."""
        resp = self._execute(KIND_SET_VOICE_MODE, client_id="quest", voice_mode="foo_bar")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertIn("invalid_voice_mode", resp.reason)

    def test_emergency_stop_active_applied(self) -> None:
        """В active emergency всегда applied=true."""
        resp = self._execute(KIND_EMERGENCY_STOP, client_id="quest")
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, "applied")
        self.assertTrue(resp.contacted_service)

    def test_heartbeat_active_refreshes(self) -> None:
        """Heartbeat с заполненным floor → applied=true, LockManager обновляется."""
        self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="teleop_floor")
        resp = self._execute(KIND_HEARTBEAT, client_id="quest", floor="teleop_floor")
        self.assertTrue(resp.accepted)
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, "heartbeat_refreshed")
        self.assertEqual(resp.held_by, "quest")

    def test_heartbeat_active_wrong_client(self) -> None:
        """Heartbeat от НЕ-владельца → applied=false, reason содержит 'heartbeat_rejected'."""
        self._execute(KIND_ACQUIRE_FLOOR, client_id="quest", floor="teleop_floor")
        resp = self._execute(KIND_HEARTBEAT, client_id="telegram", floor="teleop_floor")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertIn("heartbeat_rejected", resp.reason)

    def test_heartbeat_active_no_floor_no_op(self) -> None:
        """Heartbeat без floor в active → no_op (легковесный маркер)."""
        resp = self._execute(KIND_HEARTBEAT, client_id="quest")
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, "no_op")


class TestExecuteEdgeCases(unittest.TestCase):
    """Граничные случаи — accepted=False с понятным reason (ADR-0018)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._mode = "active"

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_unknown_kind_rejected(self) -> None:
        resp = self.node.execute(_cmd(KIND_UNKNOWN))
        self.assertFalse(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertIn("unknown_kind", resp.reason)

    def test_future_kind_value_rejected(self) -> None:
        """Любой uint8, не входящий в 1..6, отвергается с 'unknown_kind'."""
        resp = self.node.execute(_cmd(99))
        self.assertFalse(resp.accepted)
        self.assertIn("unknown_kind", resp.reason)

    def test_acquire_floor_missing_client_id(self) -> None:
        resp = self.node.execute(_cmd(KIND_ACQUIRE_FLOOR, floor="voice_floor"))
        # Допустимое поведение: applied=false (reject), но accepted=true
        # (запрос валидный по форме, невалидный по содержимому — это
        # разные отказы; в обоих случаях мы честно отвечаем, не молчим).
        self.assertFalse(resp.applied)
        self.assertIn("invalid_request", resp.reason)

    def test_acquire_floor_missing_floor(self) -> None:
        resp = self.node.execute(_cmd(KIND_ACQUIRE_FLOOR, client_id="quest"))
        self.assertFalse(resp.applied)
        self.assertIn("invalid_request", resp.reason)

    def test_set_avatar_mode_unknown_event(self) -> None:
        resp = self.node.execute(_cmd(KIND_SET_AVATAR_MODE, client_id="quest", avatar_event="bogus"))
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertIn("invalid_event", resp.reason)

    def test_object_without_kind_attribute(self) -> None:
        """Передали «не-Command» (нет атрибута kind) → unknown_kind, без exception."""
        garbage = SimpleNamespace()
        resp = self.node.execute(garbage)
        self.assertFalse(resp.accepted)
        self.assertIn("unknown_kind", resp.reason)


class TestExecuteServiceCallback(unittest.TestCase):
    """_on_execute_command корректно копирует поля в service-response."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_callback_fills_response_fields(self) -> None:
        """Smoke: реальный ``ExecuteCommand``-stype (mock из conftest)."""
        from rob_box_supervisor_msgs.srv import ExecuteCommand  # noqa: PLC0415

        svc = next(s for s in self.node._services if s.name == EXECUTE_COMMAND_SERVICE)
        self.assertIs(svc.srv_type, ExecuteCommand)

        req = ExecuteCommand.Request()
        req.command.kind = KIND_ACQUIRE_FLOOR
        req.command.client_id = "quest"
        req.command.floor = "voice_floor"

        resp = ExecuteCommand.Response()
        svc.callback(req, resp)

        # ``Response`` (внутренний msg) заполнен ``execute``-результатом.
        self.assertTrue(resp.response.accepted)
        self.assertFalse(resp.response.applied)  # monitor → applied=false
        self.assertEqual(resp.response.reason, MONITOR_MODE_REASON)

    def test_callback_logs_single_msg_arg(self) -> None:
        """Issue #1644: ``_log.info()`` получает ровно один позиционный msg."""
        from rob_box_supervisor_msgs.srv import ExecuteCommand  # noqa: PLC0415

        svc = next(s for s in self.node._services if s.name == EXECUTE_COMMAND_SERVICE)
        req = ExecuteCommand.Request()
        req.command.kind = KIND_EMERGENCY_STOP
        req.command.client_id = "tester"

        self.node._log.reset_mock()
        svc.callback(req, ExecuteCommand.Response())

        # Хотя бы один info() вызван с результатом.
        info_calls = [c for c in self.node._log.info.call_args_list if "ExecuteCommand" in str(c)]
        self.assertTrue(info_calls, "должен быть info() с префиксом ExecuteCommand")
        for call in info_calls:
            self.assertEqual(
                len(call.args),
                1,
                f"ExecuteCommand info() должен получить 1 positional arg, got {call.args!r}",
            )
            self.assertEqual(call.kwargs, {})


if __name__ == "__main__":
    unittest.main()
