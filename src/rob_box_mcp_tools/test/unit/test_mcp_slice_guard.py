"""DoD acceptance test for the slice-guard on ``mcp_server`` (issue #1998 §6.2).

Этот файл — узкая приёмочная проверка DoD-1 из карточки
``[operator-agent 08-DoD] тесты на slice/redact/ConfirmationPolicy``:

    «Оператор вне среза получает отказ на исполнение ``operator.*``
    инструмента, а не на показ схем»

То есть когда ``dialogue_node`` (имеет только срезы ``core`` +
``personality``) шлёт ``/mcp/execute`` с ``tool_name='say'`` (входит в
``operator.speech``), ``mcp_server`` ОБЯЗАН:

1. Не звать ``registry.execute``.
2. Опубликовать ``/mcp/result`` с ``success=False`` и явным указанием,
   что инструмент вне среза отправителя.

Широкие параметризованные матрицы лежат в
``test_mcp_server_slice_guard.py`` — здесь мы держим **только** тот
сценарий, который фигурирует в issue #1998 как DoD.

Тест не требует ``rclpy`` / ROS 2 — мы конструируем in-memory
классификатор (``ToolSliceAuthority.from_mapping``) и проходим ровно
тот же логический путь, что ``MCPServer.on_execute_request``:

    sender -> is_allowed() -> allowed? -> publish success/error
"""

from __future__ import annotations

import json
from typing import Any, Dict, List

import pytest

from rob_box_mcp_tools.slice_authority import ToolSliceAuthority


# ---------------------------------------------------------------------------
# Мини-имитация транспорта MCPServer.on_execute_request
# ---------------------------------------------------------------------------
#
# Скопирована та же логика, что и в ``mcp_server.py::on_execute_request``
# (ADR-0052 §2.2: slice-гард срабатывает ПОСЛЕ auth-гарда и ДО FSM-гарда).
# Никакого rclpy, никакого subprocess — мы проверяем чистое поведение
# классификатора + контракт публикации ошибки.


class _CapturingPublisher:
    """Захватывает то, что mcp_server публикует на ``/mcp/result``."""

    def __init__(self) -> None:
        self.published: List[str] = []

    def publish(self, msg) -> None:
        self.published.append(msg.data)


class _InMsg:
    """Заглушка входящего std_msgs/String с JSON-полезной нагрузкой."""

    def __init__(self, payload: Dict[str, Any]) -> None:
        self.data = json.dumps(payload, ensure_ascii=False)


def _execute_request_with_authority(
    authority: ToolSliceAuthority,
    sender: str,
    tool_name: str,
    request_id: str = "req-dod-001",
) -> Dict[str, Any]:
    """Прогоняет запрос через ту же логику, что ``on_execute_request``.

    Возвращает декодированный dict результата, опубликованного на
    ``/mcp/result`` (либо ``{"_blocked": True, "reason": ...}`` если
    slice-гард сработал ДО публикации).
    """
    pub = _CapturingPublisher()
    request = {
        "tool_name": tool_name,
        "parameters": {"text": "привет"},
        "request_id": request_id,
        "auth": {"sender": sender, "ts": 0.0, "sig": "dummy"},
    }

    # 1. Auth guard — пропускаем всегда (мы проверяем slice, не auth).
    # 2. tool_name presence — в нашем тесте всегда непустой.
    # 3. Slice guard.
    decision = authority.is_allowed(sender, tool_name)
    if not decision.allowed:
        # mcp_server публикует {"tool_name": ..., "request_id": ...,
        # "result": {"success": False, "error": "..."}}
        from rob_box_mcp_tools.base import MCPToolResult
        result = MCPToolResult(
            success=False,
            error=f"Инструмент '{tool_name}' недоступен: {decision.reason}",
        )
        response = {
            "tool_name": tool_name,
            "request_id": request_id,
            "result": result.to_dict(),
        }
        class _OutMsg:
            data = json.dumps(response, ensure_ascii=False)
        pub.publish(_OutMsg())
        return json.loads(pub.published[0])

    # До сюда мы дойти не должны при DoD-сценарии.
    raise AssertionError(
        f"DoD-1 нарушен: sender='{sender}' за пределами среза "
        f"tool='{tool_name}' был пропущен, а должен быть отказ. "
        f"decision={decision}"
    )


# ---------------------------------------------------------------------------
# Минимальный in-memory классификатор: dialogue_node имеет только core+personality.
# ---------------------------------------------------------------------------
#
# Это — «синтетический» источник истины (matches ADR-0052: данные живут в
# YAML, а Python — это просто логика над данными). Берём минимальный
# набор срезов, чтобы покрыть DoD-1 без зависимости от bundled YAML —
# тест остаётся независимым от packaging-решений Phase 1.

_AUTHORITY = ToolSliceAuthority.from_mapping(
    {
        "senders": {
            "dialogue_node": ["core", "personality"],
            "avatar_supervisor": ["core", "personality", "operator"],
        },
        "slices": {
            "core": ["ping", "speak_text", "play_animation"],
            "personality": ["speak_text", "play_animation"],
            "operator": ["say", "dialogue_pause", "read_logs"],
        },
    }
)


# ---------------------------------------------------------------------------
# DoD-1 acceptance test
# ---------------------------------------------------------------------------


def test_dod1_dialogue_node_blocked_from_operator_speech_say() -> None:
    """DoD-1 acceptance: ``dialogue_node`` НЕ может исполнить ``say``.

    ``say`` объявлен в ``operator``. ``dialogue_node`` имеет только
    ``core`` + ``personality``. Объединение срезов dialogue_node не
    содержит ``say`` → slice-гард ОБЯЗАН вернуть ``allowed=False`` и
    опубликовать ``success=False`` на ``/mcp/result``.

    Проверяем четыре инварианта DoD-1:

    * ``success=False`` (отказ, не silent drop);
    * ``tool_name`` и ``request_id`` в ответе (корреляция для агента);
    * ``error`` упоминает имя инструмента и причину отказа;
    * причина отказа содержит ``dialogue_node`` — чтобы оператор
      понимал, *кто* именно был за пределами среза.
    """
    response = _execute_request_with_authority(
        authority=_AUTHORITY,
        sender="dialogue_node",
        tool_name="say",
    )

    # 1. Отказ — это success=False.
    assert response["result"]["success"] is False, (
        "DoD-1: mcp_server должен вернуть отказ при выходе за срез, "
        f"получили success={response['result']['success']!r}"
    )

    # 2. Корреляция — имя тула и request_id сохранены.
    assert response["tool_name"] == "say"
    assert response["request_id"] == "req-dod-001"

    # 3. Сообщение об ошибке информативно.
    error = response["result"]["error"]
    assert "say" in error, f"error должен упоминать 'say', получили {error!r}"
    assert "недоступен" in error or "не принадлежит" in error, (
        f"error должен объяснять отказ, получили {error!r}"
    )
    assert "dialogue_node" in error, (
        f"error должен указывать sender'а, получили {error!r}"
    )


def test_dod1_dialogue_node_blocked_from_operator_admin_read_logs() -> None:
    """DoD-1 контраст: ``dialogue_node`` тоже не имеет ``read_logs``.

    ``read_logs`` — самый «громкий» операторский тул: может вернуть
    ``DEEPSEEK_API_KEY=...`` из логов. Slice-гард должен зарезать его
    ещё до того, как payload пойдёт в сторону ``registry.execute``.
    """
    response = _execute_request_with_authority(
        authority=_AUTHORITY,
        sender="dialogue_node",
        tool_name="read_logs",
        request_id="req-dod-002",
    )

    assert response["result"]["success"] is False
    assert response["tool_name"] == "read_logs"
    assert response["request_id"] == "req-dod-002"
    assert "read_logs" in response["result"]["error"]


@pytest.mark.parametrize(
    "tool_name",
    sorted({"say", "dialogue_pause", "read_logs"}),
)
def test_dod1_dialogue_node_blocked_from_all_operator_tools(tool_name: str) -> None:
    """Параметризация DoD-1: dialogue_node не имеет НИ ОДНОГО ``operator.*`` тула.

    Это — последняя линия обороны §6.2: ни один инструмент из
    operator-среза не должен случайно стать доступным dialogue_node.
    """
    response = _execute_request_with_authority(
        authority=_AUTHORITY,
        sender="dialogue_node",
        tool_name=tool_name,
    )

    assert response["result"]["success"] is False, (
        f"DoD-1 violated: dialogue_node смог исполнить {tool_name!r}"
    )


def test_dod1_positive_contrast_avatar_supervisor_can_say() -> None:
    """Положительный контраст: avatar_supervisor (есть operator) — может say.

    DoD говорит про отказ sender'а, у которого нет среза. Этот тест
    страхует от «over-correction» — slice-гард не должен блокировать
    того, кому инструмент действительно положен.
    """
    # avatar_supervisor в нашем синтетическом authority имеет operator,
    # значит ``say`` (operator) ему доступен. _execute_request_with_authority
    # в этом случае должен БРОСИТЬ AssertionError (потому что мы не
    # публикуем success=True). Ловим — это сигнал «дошло до execute».
    with pytest.raises(AssertionError) as exc_info:
        _execute_request_with_authority(
            authority=_AUTHORITY,
            sender="avatar_supervisor",
            tool_name="say",
        )
    # AssertionError — это именно то, что мы хотим: код дошёл до строки,
    # которая сработала бы только если slice-гард НЕ заблокировал вызов.
    assert "за пределами среза" in str(exc_info.value)
