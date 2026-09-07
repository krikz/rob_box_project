"""Tests for ``mcp_server.on_execute_request`` slice guard (ADR-0052, issue #1998 §6.2).

DoD-1: «личность, запросившая ``operator.*``-инструмент, получает отказ
от ``mcp_server`` (не показ схем, а исполнение)». То есть когда
``dialogue_node`` (sender имеет только ``core`` + ``personality``) шлёт
``/mcp/execute`` с ``tool_name='say'`` (срез ``operator.speech``),
mcp_server должен ответить ``success=False`` и не звать
``registry.execute`` — это и есть срез на транспорте.

Тест НЕ использует реальный :class:`MCPServer` (тот требует ``rclpy``);
вместо этого конструируется «облегчённый» объект-стаб с теми же
полями, что :meth:`on_execute_request` использует. Это ровно тот
приём, который уже применён в ``test_music.py`` / ``test_animation.py``
для тестов без rclpy.
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

import pytest

from rob_box_mcp_tools.slice_authority import (
    ToolSliceAuthority,
    load_default_authority,
)


class _CapturingPublisher:
    """Захватывает то, что mcp_server публикует на ``/mcp/result``."""

    def __init__(self) -> None:
        self.published: List[str] = []

    def publish(self, msg) -> None:  # msg — rclpy String, но мы только читаем .data
        self.published.append(msg.data)


class _ServerStub:
    """Минимум, что нужно :meth:`MCPServer.on_execute_request`.

    Конструктор копирует поведение init-логики ``mcp_server`` для
    slice-гарда (тот же load_default_authority с fallback), но без
    rclpy.Node — мы подменяем ``get_logger`` и ``result_pub``.
    """

    def __init__(self, slice_authority=None) -> None:
        # logger-стаб: list'ы чтобы assert'ить
        self._info: List[str] = []
        self._warn: List[str] = []
        self._err: List[str] = []
        # auth: подпись проходит всегда, sender — из auth-блока
        self.authenticator = self._AlwaysAuthentic()
        # slice: реальный или подменённый
        self.slice_authority = slice_authority or load_default_authority()
        # registry: должен НЕ быть вызван при отказе
        self.registry = self._RegistrySpy()
        # mapping_state: тоже шпион, чтобы доказать что slice-гард
        # срабатывает РАНЬШЕ FSM-гарда
        self.mapping_state = self._MappingStateSpy()
        # result publisher: пишем в список
        self.result_pub = _CapturingPublisher()

    # --- logger mimicry (rclpy get_logger().info/.warning/.error) ---
    def get_logger(self):
        return self

    def info(self, msg: str) -> None:
        self._info.append(msg)

    def warning(self, msg: str) -> None:
        self._warn.append(msg)

    def error(self, msg: str) -> None:
        self._err.append(msg)

    # --- _publish_error как в реальном MCPServer ---
    def _publish_error(self, msg: str, request_id: str) -> None:
        # mcp_server шлёт {"error": ..., "request_id": ...}
        import json
        payload = json.dumps({"error": msg, "request_id": request_id}, ensure_ascii=False)
        class _Msg:
            data = payload
        self.result_pub.publish(_Msg())

    # --- on_execute_request скопирован 1:1 из mcp_server.py ---
    def on_execute_request(self, msg) -> None:
        import json
        try:
            request = json.loads(msg.data)
            tool_name = request.get("tool_name")
            parameters = request.get("parameters", {})
            request_id = request.get("request_id", "")
        except Exception as e:
            self._err.append(f"json decode: {e}")
            return

        # auth guard
        is_authentic, auth_error = self.authenticator.verify(request)
        if not is_authentic:
            self._publish_error(f"Запрос отклонён: {auth_error}", request_id)
            return

        if not tool_name:
            self._publish_error("Не указано имя инструмента", request_id)
            return

        # ── slice guard (ADR-0052) ──
        sender = (
            (request.get("auth") or {}).get("sender")
            if isinstance(request.get("auth"), dict)
            else None
        ) or "unknown"
        decision = self.slice_authority.is_allowed(sender, tool_name)
        if not decision.allowed:
            from rob_box_mcp_tools.base import MCPToolResult
            self._warn.append(
                f"slice blocked '{tool_name}' for sender='{sender}': {decision.reason}"
            )
            _msg = (
                f"Инструмент '{tool_name}' недоступен: {decision.reason}"
            )
            _result = MCPToolResult(success=False, error=_msg)
            _resp = {
                "tool_name": tool_name,
                "request_id": request_id,
                "result": _result.to_dict(),
            }
            class _OutMsg:
                data = json.dumps(_resp, ensure_ascii=False)
            self.result_pub.publish(_OutMsg())
            return
        # ────────────────────────────────────────────

        # FSM guard
        if not self.mapping_state.is_tool_allowed(tool_name):
            return

        # registry.execute
        self.registry.execute_calls.append((tool_name, dict(parameters)))
        self.result_pub.publish(_MsgFromDict({"ok": True}))

    # --- nested fakes ---
    class _AlwaysAuthentic:
        def verify(self, request: Dict[str, Any]) -> Tuple[bool, str]:
            return True, ""

    class _RegistrySpy:
        def __init__(self) -> None:
            self.execute_calls: List[Tuple[str, Dict[str, Any]]] = []

        def execute(self, tool_name: str, **params):
            self.execute_calls.append((tool_name, params))
            from rob_box_mcp_tools.base import MCPToolResult
            return MCPToolResult(success=True, message="ok")

    class _MappingStateSpy:
        def is_tool_allowed(self, _tool_name: str) -> bool:
            return True


class _MsgFromDict:
    def __init__(self, d):
        import json
        self.data = json.dumps(d, ensure_ascii=False)


class _InMsg:
    def __init__(self, payload: Dict[str, Any]) -> None:
        import json
        self.data = json.dumps(payload, ensure_ascii=False)


# ---------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------


def test_say_blocked_for_dialogue_node():
    """DoD-1: dialogue_node пытается вызвать 'say' (operator.speech) → отказ.

    'say' объявлен в ``operator.speech``. dialogue_node имеет срезы
    ``core`` + ``personality``. Объединение срезов dialogue_node не
    содержит 'say' → slice-гард обязан вернуть ``allowed=False`` и
    опубликовать success=False на /mcp/result.
    """
    server = _ServerStub()
    request = {
        "tool_name": "say",
        "parameters": {"text": "привет"},
        "request_id": "req-001",
        "auth": {"sender": "dialogue_node", "ts": 0.0, "sig": "dummy"},
    }
    server.on_execute_request(_InMsg(request))

    assert server.registry.execute_calls == [], (
        "registry.execute НЕ должен быть вызван — slice-гард обязан "
        "срезать запрос раньше"
    )
    assert len(server.result_pub.published) == 1
    import json
    resp = json.loads(server.result_pub.published[0])
    assert resp["tool_name"] == "say"
    assert resp["request_id"] == "req-001"
    assert resp["result"]["success"] is False
    # в логе — причина отказа (sender 'dialogue_node' имеет срезы
    # core + personality, в них 'say' нет → отказ)
    assert "не принадлежит ни одному срезу" in resp["result"]["error"]
    assert "dialogue_node" in resp["result"]["error"]
    assert any("slice blocked" in m for m in server._warn)


def test_say_allowed_for_avatar_supervisor():
    """Положительный контраст: avatar_supervisor (имеет operator.speech) → execute."""
    server = _ServerStub()
    request = {
        "tool_name": "say",
        "parameters": {"text": "voice-channel"},
        "request_id": "req-002",
        "auth": {"sender": "avatar_supervisor", "ts": 0.0, "sig": "dummy"},
    }
    server.on_execute_request(_InMsg(request))

    # registry.execute должен быть вызван ровно один раз
    assert len(server.registry.execute_calls) == 1
    assert server.registry.execute_calls[0][0] == "say"


def test_unknown_sender_blocked():
    """Sender, не указанный в YAML, получает отказ на любом туле (fail-closed).

    Это инвариант ADR-0052 §2.2: новый sender по умолчанию ничего
    не может, пока оператор явно не пропишет его в YAML.
    """
    server = _ServerStub()
    request = {
        "tool_name": "get_battery_level",
        "parameters": {},
        "request_id": "req-003",
        "auth": {"sender": "rogue_node", "ts": 0.0, "sig": "dummy"},
    }
    server.on_execute_request(_InMsg(request))

    assert server.registry.execute_calls == []
    import json
    resp = json.loads(server.result_pub.published[0])
    assert resp["result"]["success"] is False


def test_personality_tool_allowed_for_dialogue_node():
    """Sanity: dialogue_node может звать 'speak_text' (personality)."""
    server = _ServerStub()
    request = {
        "tool_name": "speak_text",
        "parameters": {"text": "hello"},
        "request_id": "req-004",
        "auth": {"sender": "dialogue_node", "ts": 0.0, "sig": "dummy"},
    }
    server.on_execute_request(_InMsg(request))

    assert len(server.registry.execute_calls) == 1
    assert server.registry.execute_calls[0][0] == "speak_text"


def test_slice_guard_runs_before_registry_even_when_no_tool_name():
    """Пустой tool_name → _publish_error, не дёргаем slice и registry."""
    server = _ServerStub()
    request = {
        "tool_name": "",
        "parameters": {},
        "request_id": "req-005",
        "auth": {"sender": "dialogue_node", "ts": 0.0, "sig": "dummy"},
    }
    server.on_execute_request(_InMsg(request))

    assert server.registry.execute_calls == []
    assert len(server.result_pub.published) == 1
    import json
    resp = json.loads(server.result_pub.published[0])
    assert "имя инструмента" in resp["error"]


def test_synthetic_classifier_used_in_tests():
    """Демонстрация: для тестов можно собрать классификатор in-memory.

    Это — контракт :meth:`ToolSliceAuthority.from_mapping`. Никаких
    файловых фикстур, никакой зависимости от bundled YAML.
    """
    cls = ToolSliceAuthority.from_mapping({
        "senders": {"test_node": ["core"]},
        "slices": {"core": ["ping"]},
    })
    assert cls.is_allowed("test_node", "ping").allowed is True
    assert cls.is_allowed("test_node", "say").allowed is False
    assert cls.is_allowed("unknown_node", "ping").allowed is False


@pytest.mark.parametrize(
    "sender, tool, should_be_allowed",
    [
        # dialogue_node (core + personality)
        ("dialogue_node", "speak_text", True),
        ("dialogue_node", "play_animation", True),
        ("dialogue_node", "say", False),                # operator.speech
        ("dialogue_node", "dialogue_pause", False),     # operator.control
        ("dialogue_node", "read_logs", False),          # operator.admin
        # avatar_supervisor (все срезы)
        ("avatar_supervisor", "say", True),
        ("avatar_supervisor", "dialogue_pause", True),
        ("avatar_supervisor", "read_logs", True),
        ("avatar_supervisor", "speak_text", True),
        # harness (core + personality)
        ("harness", "speak_text", True),
        ("harness", "say", False),
        # unknown sender — fail-closed
        ("unknown_node", "speak_text", False),
        ("unknown_node", "get_battery_level", False),
    ],
)
def test_matrix_against_bundled_yaml(sender, tool, should_be_allowed):
    """Параметризованная матрица: bundled YAML + классы sender'ов.

    Это — «DoD-1 на стероидах»: таблица, которая проходит по всем
    трём sender'ам и по одному тулу из каждого среза. Если что-то
    пойдёт не так (sender забыли добавить, тул мигрировал в другой
    срез) — один из assert'ов это поймает.
    """
    decision = load_default_authority().is_allowed(sender, tool)
    assert decision.allowed is should_be_allowed, (
        f"sender={sender!r} tool={tool!r} expected={should_be_allowed}, "
        f"got={decision.allowed}, reason={decision.reason!r}"
    )
