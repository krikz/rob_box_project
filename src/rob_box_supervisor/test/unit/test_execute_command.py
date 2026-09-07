"""Unit-тесты Bridge.execute(Command) — ADR-0051 §2.1, issue #2002.

Покрывает:

* Сервис ``/supervisor/execute`` зарегистрирован в ``__init__`` AvatarSupervisor
  через srv-тип с nested ``.Request``/``.Response`` (issue #1904).
* ``AvatarSupervisor.execute(Command)`` корректно маршрутизирует по ``kind``:
  ACQUIRE/RELEASE_FLOOR/SET_AVATAR_MODE → facade_only/monitor_mode;
  SET_VOICE_MODE → локально через ``_apply_voice_mode``;
  EMERGENCY_STOP → not_implemented/emergency_off;
  HEARTBEAT → noop (с client_id);
  UNKNOWN (включая 0 и 99) → unknown_kind.
* ``_on_execute_command`` корректно заполняет ``response.response``.

Матрица: 6 команд × {accepted, applied, reason, held_by, actual_mode}
+ 1 unknown + 3 edge-кейса (empty client_id, magicmock kind, empty payload).
Цель по DoD карточки #2002: ``Bridge.execute(Command)`` обрабатывает все
команды, покрытые прежними 25 методами (Phase 1: facade-only для floor/mode).
"""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    EXEC_REASON_BAD_REQUEST,
    EXEC_REASON_EMERGENCY_OFF,
    EXEC_REASON_HEARTBEAT_NOOP,
    EXEC_REASON_MONITOR_MODE,
    EXEC_REASON_NOT_IMPLEMENTED,
    EXEC_REASON_UNKNOWN_KIND,
    EXEC_REASON_VOICE_MODE_REJECTED,
    EXECUTE_COMMAND_SERVICE,
    KIND_ACQUIRE_FLOOR,
    KIND_EMERGENCY_STOP,
    KIND_HEARTBEAT,
    KIND_RELEASE_FLOOR,
    KIND_SET_AVATAR_MODE,
    KIND_SET_VOICE_MODE,
    KIND_UNKNOWN,
    AvatarSupervisor,
)


def _make_command(**kwargs: object) -> SimpleNamespace:
    """Собрать Command-подобный объект с разумными дефолтами."""
    base = {
        "kind": 0,
        "client_id": "",
        "floor": "",
        "avatar_event": "",
        "voice_mode": "",
        "emergency": False,
    }
    base.update(kwargs)
    return SimpleNamespace(**base)


class TestExecuteServiceRegistered(unittest.TestCase):
    """Сервис ``/supervisor/execute`` зарегистрирован (ADR-0051 §2.1)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_execute_service_registered(self) -> None:
        names = [s.name for s in self.node._services]
        self.assertIn(EXECUTE_COMMAND_SERVICE, names)

    def test_execute_service_has_full_srv_type(self) -> None:
        """Регресс #1904: create_service должен получить ПОЛНЫЙ srv-класс,
        иначе RuntimeError в __init__. Этот тест ловит, если кто-то
        по ошибке передал ``ExecuteCommand.Request``.
        """
        names = [s.name for s in self.node._services]
        idx = names.index(EXECUTE_COMMAND_SERVICE)
        srv_obj = self.node._services[idx]
        srv_type = srv_obj.srv_type
        self.assertTrue(hasattr(srv_type, "Request"))
        self.assertTrue(hasattr(srv_type, "Response"))

    def test_execute_service_callback_wired(self) -> None:
        """Callback сервиса — :py:meth:`AvatarSupervisor._on_execute_command`."""
        names = [s.name for s in self.node._services]
        idx = names.index(EXECUTE_COMMAND_SERVICE)
        srv_obj = self.node._services[idx]
        # callback bound к self.node, не свободная функция
        self.assertTrue(callable(srv_obj.callback))


class TestExecuteAcquisitionCommands(unittest.TestCase):
    """ACQUIRE/RELEASE_FLOOR и SET_AVATAR_MODE → facade_only (Phase 1)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _assert_facade_only(self, resp: object) -> None:
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, "facade_only")

    def test_acquire_floor_active_returns_facade_only(self) -> None:
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_ACQUIRE_FLOOR, client_id="quest")
        resp = self.node.execute(cmd)
        self._assert_facade_only(resp)
        self.assertEqual(resp.held_by, "quest")

    def test_release_floor_active_returns_facade_only(self) -> None:
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_RELEASE_FLOOR, client_id="telegram")
        resp = self.node.execute(cmd)
        self._assert_facade_only(resp)
        self.assertEqual(resp.held_by, "telegram")

    def test_set_avatar_mode_active_returns_facade_only(self) -> None:
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_SET_AVATAR_MODE, client_id="quest")
        resp = self.node.execute(cmd)
        self._assert_facade_only(resp)
        self.assertEqual(resp.held_by, "quest")

    def test_acquire_floor_monitor_returns_monitor_mode(self) -> None:
        """В monitor supervisor принимает, но не делает (S12)."""
        cmd = _make_command(kind=KIND_ACQUIRE_FLOOR, client_id="quest")
        resp = self.node.execute(cmd)
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_MONITOR_MODE)

    def test_release_floor_monitor_returns_monitor_mode(self) -> None:
        cmd = _make_command(kind=KIND_RELEASE_FLOOR, client_id="telegram")
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_MONITOR_MODE)

    def test_set_avatar_mode_monitor_returns_monitor_mode(self) -> None:
        cmd = _make_command(kind=KIND_SET_AVATAR_MODE, client_id="quest")
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_MONITOR_MODE)

    def test_acquire_floor_empty_client_id_rejected(self) -> None:
        """Пустой client_id — bad_request, даже в active."""
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_ACQUIRE_FLOOR, client_id="")
        resp = self.node.execute(cmd)
        self.assertFalse(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_BAD_REQUEST)

    def test_set_avatar_mode_empty_client_id_rejected(self) -> None:
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_SET_AVATAR_MODE, client_id="")
        resp = self.node.execute(cmd)
        self.assertFalse(resp.accepted)
        self.assertEqual(resp.reason, EXEC_REASON_BAD_REQUEST)

    def test_facade_only_does_not_leak_avatar_event(self) -> None:
        """applied=False → actual_mode=\"\" (честный FAIL, ADR-0018).

        Клиент НЕ должен думать, что мы применили avatar_event, если
        реально только проксируем в arbiter (ещё не подключён).
        """
        self.node._mode = "active"
        cmd = _make_command(
            kind=KIND_SET_AVATAR_MODE,
            client_id="quest",
            avatar_event="avatar_present",
        )
        resp = self.node.execute(cmd)
        self.assertEqual(resp.actual_mode, "")


class TestExecuteVoiceMode(unittest.TestCase):
    """SET_VOICE_MODE → локально через :py:meth:`_apply_voice_mode`."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_set_voice_mode_active_applied(self) -> None:
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        cmd = _make_command(kind=KIND_SET_VOICE_MODE, voice_mode="quest_ttts")
        resp = self.node.execute(cmd)
        self.assertTrue(resp.accepted)
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, "applied")
        self.assertEqual(resp.actual_mode, "quest_ttts")

    def test_set_voice_mode_monitor_rejected_monitor_reason(self) -> None:
        cmd = _make_command(kind=KIND_SET_VOICE_MODE, voice_mode="quest_ttts")
        resp = self.node.execute(cmd)
        self.assertFalse(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_MONITOR_MODE)

    def test_set_voice_mode_invalid_mode_rejected(self) -> None:
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_SET_VOICE_MODE, voice_mode="not_a_mode")
        resp = self.node.execute(cmd)
        self.assertFalse(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_VOICE_MODE_REJECTED)

    def test_set_voice_mode_empty_string_rejected(self) -> None:
        self.node._mode = "active"
        cmd = _make_command(kind=KIND_SET_VOICE_MODE, voice_mode="")
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_VOICE_MODE_REJECTED)


class TestExecuteEmergencyStop(unittest.TestCase):
    """EMERGENCY_STOP → not_implemented в Phase 1 (ADR-0018)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_emergency_stop_true_not_implemented(self) -> None:
        cmd = _make_command(kind=KIND_EMERGENCY_STOP, emergency=True)
        resp = self.node.execute(cmd)
        # Честный FAIL: приняли, но не сделали (ADR-0018).
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_NOT_IMPLEMENTED)

    def test_emergency_stop_false_emergency_off(self) -> None:
        """emergency=false («снять стоп») — не ошибка, просто ничего
        делать не нужно (стопа, видимо, и не было)."""
        cmd = _make_command(kind=KIND_EMERGENCY_STOP, emergency=False)
        resp = self.node.execute(cmd)
        self.assertTrue(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_EMERGENCY_OFF)

    def test_emergency_stop_default_false_treated_as_off(self) -> None:
        """Если клиент забыл выставить emergency — трактуем как off."""
        cmd = _make_command(kind=KIND_EMERGENCY_STOP)
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_EMERGENCY_OFF)


class TestExecuteHeartbeat(unittest.TestCase):
    """HEARTBEAT → noop в Phase 1 (Phase 2 заменит на floor-refresh)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_heartbeat_with_client_id_noop(self) -> None:
        cmd = _make_command(kind=KIND_HEARTBEAT, client_id="telegram")
        resp = self.node.execute(cmd)
        self.assertTrue(resp.accepted)
        self.assertTrue(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_HEARTBEAT_NOOP)
        self.assertEqual(resp.held_by, "telegram")

    def test_heartbeat_empty_client_id_rejected(self) -> None:
        cmd = _make_command(kind=KIND_HEARTBEAT, client_id="")
        resp = self.node.execute(cmd)
        self.assertFalse(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_BAD_REQUEST)


class TestExecuteUnknownKind(unittest.TestCase):
    """UNKNOWN kind → unknown_kind (явный FAIL)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_kind_zero_is_unknown(self) -> None:
        """KIND_UNKNOWN=0 → unknown_kind."""
        cmd = _make_command(kind=KIND_UNKNOWN)
        resp = self.node.execute(cmd)
        self.assertFalse(resp.accepted)
        self.assertFalse(resp.applied)
        self.assertEqual(resp.reason, EXEC_REASON_UNKNOWN_KIND)

    def test_kind_out_of_range_is_unknown(self) -> None:
        """kind=99 (за пределами [1..6]) → unknown_kind."""
        cmd = _make_command(kind=99)
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_UNKNOWN_KIND)

    def test_kind_negative_is_unknown(self) -> None:
        cmd = _make_command(kind=-1)
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_UNKNOWN_KIND)

    def test_kind_missing_attribute_is_unknown(self) -> None:
        """Если у command вообще нет атрибута ``kind`` → unknown_kind."""
        cmd = SimpleNamespace(client_id="x")  # без kind
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_UNKNOWN_KIND)


class TestExecuteRobustness(unittest.TestCase):
    """execute() не валится на битом payload (MagicMock и пр.)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_magicmock_kind_falls_back_to_unknown(self) -> None:
        """Битый ``kind`` (строка без int-значения) → UNKNOWN.

        MagicMock сам по себе поддерживает ``__int__`` (возвращает 1),
        поэтому как payload он проходит как ``KIND_ACQUIRE_FLOOR``. Чтобы
        действительно пробить fallback ``_coerce_kind``, подсунем объект,
        который не конвертится в ``int`` (строковый литерал).
        """
        cmd = MagicMock()
        cmd.kind = "not-a-number"  # int() → ValueError → _coerce_kind → UNKNOWN
        cmd.client_id = "x"
        resp = self.node.execute(cmd)
        self.assertEqual(resp.reason, EXEC_REASON_UNKNOWN_KIND)

    def test_voice_mode_non_string_normalized(self) -> None:
        """Не-строковый voice_mode (например, MagicMock или None)
        нормализуется и не валит ноду."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        cmd = _make_command(kind=KIND_SET_VOICE_MODE, voice_mode=None)
        resp = self.node.execute(cmd)
        # None нормализуется в "", это невалидный mode → voice_mode_rejected
        self.assertEqual(resp.reason, EXEC_REASON_VOICE_MODE_REJECTED)


class TestOnExecuteCommandCallback(unittest.TestCase):
    """Callback ``/supervisor/execute`` заполняет ``response.response``."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _make_request(self, kind: int, client_id: str = "test") -> SimpleNamespace:
        return SimpleNamespace(
            command=_make_command(kind=kind, client_id=client_id)
        )

    def _make_response(self) -> SimpleNamespace:
        # srv-тип из conftest: ExecuteCommand.Response имеет вложенный response
        return SimpleNamespace(response=SimpleNamespace(
            accepted=False,
            applied=False,
            reason="",
            held_by="",
            actual_mode="",
            contacted_service="",
        ))

    def test_callback_unrecognized_kind_populates_unknown(self) -> None:
        request = self._make_request(kind=99)
        response = self._make_response()
        self.node._on_execute_command(request, response)
        self.assertFalse(response.response.accepted)
        self.assertEqual(response.response.reason, EXEC_REASON_UNKNOWN_KIND)

    def test_callback_voice_mode_applied_propagates(self) -> None:
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        request = self._make_request(
            kind=KIND_SET_VOICE_MODE, client_id="ignored"
        )
        request.command.voice_mode = "respeaker"
        response = self._make_response()
        self.node._on_execute_command(request, response)
        self.assertTrue(response.response.accepted)
        self.assertTrue(response.response.applied)
        self.assertEqual(response.response.actual_mode, "respeaker")


if __name__ == "__main__":
    unittest.main()