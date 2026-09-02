"""avatar_command.py — единый контракт ``/avatar/command`` ↔ ``/avatar/command_result``.

Контракт заморожен в ``docs/plans/2026-09-02-avatar-worker-brief.md`` §3.3.
Используется:

* AV-22 (эта карточка) — **producer** в ``dialogue_node`` (режим
  ``quest_command``) и в ``rob_box_telegram`` (``/cmd`` + ``/operator on``).
* AV-21 (follow-up, ещё не сделан) — **consumer**: супервизор-агент
  подписан на ``/avatar/command`` и публикует ``/avatar/command_result``.

Контракт топиков
================

``/avatar/command`` (``std_msgs/String``, JSON, RELIABLE depth 10):

.. code-block:: python

    {
        "request_id": "<uuid4>",        # уникален на каждую команду
        "source":     "quest" | "telegram",  # откуда пришла команда
        "client_id":  "<session_id>" | "telegram:<chat_id>",
        "text":       "<строка команды>",
        "ts_ms":      <uint64 epoch ms>,
    }

``/avatar/command_result`` (``std_msgs/String``, JSON, RELIABLE depth 10):

.. code-block:: python

    {
        "request_id":  "<uuid4>",       # коррелирует с command
        "ok":          True | False,
        "summary":     "<короткий текст результата>",
        "tool_calls":  ["<имя tool-а>", ...],
    }

Инварианты
==========

* ``request_id`` — UUIDv4; producer генерирует, потребитель пробрасывает
  в ответ. Producer **никогда** не использует ``request_id``, выданный
  клиентом — иначе клиент-в-очках сможет подделать корреляцию.
* ``client_id`` формирует **серверная сторона** (краткий бриф §1.3):
  ``quest:<session_id>`` формирует quest-сервер, ``telegram:<chat_id>``
  формирует telegram-нода. Значение, присланное клиентом — не источник
  истины; расхождение логируется, но не используется.
* ``ts_ms`` — монотонный ``time.time_ns() // 1_000_000`` (или
  инжектированные часы в тестах).
* Никакого тихого fallback на другой кодек (worker-brief §6.2): не
  удалось декодировать — шуми, не публикуй.

Тут нет ROS-зависимостей — этот модуль чистый Python, чтобы его могли
импортировать и unit-тесты без rclpy.
"""

from __future__ import annotations

import json
import time
import uuid
from typing import Any, Optional


# ─────────────────────────────────────────────────────────────────────────
#  Замороженные константы
# ─────────────────────────────────────────────────────────────────────────

#: ``std_msgs/String`` топик, куда producers шлют команды.
AVATAR_COMMAND_TOPIC: str = "/avatar/command"

#: ``std_msgs/String`` топик, куда agent публикует результат.
AVATAR_COMMAND_RESULT_TOPIC: str = "/avatar/command_result"

#: Допустимые источники команды (см. краткий бриф §1.2, §3.3).
SOURCES: tuple[str, ...] = ("quest", "telegram")


# ─────────────────────────────────────────────────────────────────────────
#  Producers
# ─────────────────────────────────────────────────────────────────────────


def new_request_id() -> str:
    """Уникальный UUIDv4 для корреляции command ↔ command_result.

    Используем ``uuid.uuid4`` (без clock_seq/node) — простой и
    воспроизводимо-тестируемый.
    """
    return str(uuid.uuid4())


def now_ts_ms(clock_ns: Optional[int] = None) -> int:
    """Текущий epoch в миллисекундах.

    ``clock_ns`` — инжектор часов для тестов (worker-brief §6.7):
    ``clock_ns=time.time_ns()``. По умолчанию берём реальное время.
    """
    ns = clock_ns if clock_ns is not None else time.time_ns()
    return int(ns // 1_000_000)


def build_command(
    *,
    source: str,
    client_id: str,
    text: str,
    ts_ms: Optional[int] = None,
    request_id: Optional[str] = None,
    clock_ns: Optional[int] = None,
) -> dict[str, Any]:
    """Собрать dict для ``/avatar/command``.

    Args:
        source: ``"quest"`` или ``"telegram"`` (см. :data:`SOURCES`).
        client_id: серверная ``client_id`` (``quest:<sid>`` или
            ``telegram:<chat_id>``). Без префикса ``telegram:`` —
            выкидываем ``ValueError``.
        text: команда оператора (уже очищенная от префиксов).
        ts_ms: явный timestamp (для тестов). По умолчанию —
            ``now_ts_ms()``.
        request_id: явный UUID (для тестов). По умолчанию —
            :func:`new_request_id`.
        clock_ns: инжектор часов (только для ``ts_ms``); если ``ts_ms``
            задан явно — игнорируется.

    Raises:
        ValueError: ``source`` или ``client_id`` некорректны.
    """
    if source not in SOURCES:
        raise ValueError(
            f"avatar_command: source={source!r} не в {SOURCES!r}"
        )
    if not client_id or not isinstance(client_id, str):
        raise ValueError(
            f"avatar_command: client_id={client_id!r} должен быть непустой строкой"
        )
    if not text or not isinstance(text, str):
        raise ValueError(
            f"avatar_command: text={text!r} должен быть непустой строкой"
        )
    return {
        "request_id": request_id or new_request_id(),
        "source": source,
        "client_id": client_id,
        "text": text,
        "ts_ms": ts_ms if ts_ms is not None else now_ts_ms(clock_ns),
    }


def build_command_result(
    *,
    request_id: str,
    ok: bool,
    summary: str,
    tool_calls: Optional[list[str]] = None,
) -> dict[str, Any]:
    """Собрать dict для ``/avatar/command_result``.

    Args:
        request_id: UUID, выданный producer'ом (см. :func:`build_command`).
        ok: ``True`` если команда выполнена, ``False`` если ошибка.
        summary: короткое человеко-читаемое описание результата.
        tool_calls: список имён инструментов, которые agent вызвал
            (для логов и observability). Может быть пустым.

    Raises:
        ValueError: ``request_id`` пустой или ``summary`` пустой.
    """
    if not request_id:
        raise ValueError("avatar_command: request_id обязателен")
    if not isinstance(summary, str):
        raise ValueError("avatar_command: summary должен быть строкой")
    return {
        "request_id": request_id,
        "ok": bool(ok),
        "summary": summary,
        "tool_calls": list(tool_calls or []),
    }


# ─────────────────────────────────────────────────────────────────────────
#  Wire — encode/decode JSON payload
# ─────────────────────────────────────────────────────────────────────────


def encode_command(payload: dict[str, Any]) -> str:
    """Сериализовать dict команды в JSON для ``std_msgs/String.data``.

    Args:
        payload: dict из :func:`build_command`.

    Raises:
        ValueError: payload не проходит минимальную валидацию.
    """
    _validate_command(payload)
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def decode_command(raw: str) -> dict[str, Any]:
    """Десериализовать JSON ``/avatar/command`` в dict.

    Raises:
        ValueError: пусто, не JSON, или нет обязательных полей.
    """
    if not raw or not isinstance(raw, str):
        raise ValueError("avatar_command: пустой payload")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"avatar_command: невалидный JSON: {exc.msg} (line {exc.lineno})"
        ) from exc
    if not isinstance(data, dict):
        raise ValueError("avatar_command: payload не dict")
    _validate_command(data)
    return data


def encode_command_result(payload: dict[str, Any]) -> str:
    """Сериализовать dict результата в JSON."""
    _validate_command_result(payload)
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def decode_command_result(raw: str) -> dict[str, Any]:
    """Десериализовать JSON ``/avatar/command_result`` в dict.

    Raises:
        ValueError: пусто, не JSON, или нет обязательных полей.
    """
    if not raw or not isinstance(raw, str):
        raise ValueError("avatar_command_result: пустой payload")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"avatar_command_result: невалидный JSON: {exc.msg} (line {exc.lineno})"
        ) from exc
    if not isinstance(data, dict):
        raise ValueError("avatar_command_result: payload не dict")
    _validate_command_result(data)
    return data


# ─────────────────────────────────────────────────────────────────────────
#  Helpers для producers
# ─────────────────────────────────────────────────────────────────────────


def make_quest_client_id(session_id: str) -> str:
    """Сформировать ``client_id`` для Quest-сессии.

    Args:
        session_id: идентификатор WSS-сессии на стороне quest-сервера.

    Raises:
        ValueError: ``session_id`` пустой.
    """
    if not session_id:
        raise ValueError("avatar_command: quest session_id пустой")
    return f"quest:{session_id}"


def make_telegram_client_id(chat_id: int) -> str:
    """Сформировать ``client_id`` для Telegram-чата.

    Args:
        chat_id: int из ``update.effective_chat.id``.
    """
    if chat_id is None:
        raise ValueError("avatar_command: telegram chat_id отсутствует")
    try:
        chat_id_int = int(chat_id)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"avatar_command: telegram chat_id={chat_id!r} не int"
        ) from exc
    return f"telegram:{chat_id_int}"


# ─────────────────────────────────────────────────────────────────────────
#  Внутренние валидаторы
# ─────────────────────────────────────────────────────────────────────────


def _validate_command(payload: dict[str, Any]) -> None:
    required = ("request_id", "source", "client_id", "text", "ts_ms")
    missing = [k for k in required if k not in payload]
    if missing:
        raise ValueError(
            f"avatar_command: отсутствуют поля {missing!r}"
        )
    if payload["source"] not in SOURCES:
        raise ValueError(
            f"avatar_command: source={payload['source']!r} не в {SOURCES!r}"
        )
    if not isinstance(payload["text"], str) or not payload["text"]:
        raise ValueError("avatar_command: text должен быть непустой строкой")
    if not isinstance(payload["client_id"], str) or not payload["client_id"]:
        raise ValueError(
            "avatar_command: client_id должен быть непустой строкой"
        )
    if not isinstance(payload["request_id"], str) or not payload["request_id"]:
        raise ValueError(
            "avatar_command: request_id должен быть непустой строкой"
        )
    if not isinstance(payload["ts_ms"], int):
        raise ValueError("avatar_command: ts_ms должен быть int")


def _validate_command_result(payload: dict[str, Any]) -> None:
    required = ("request_id", "ok", "summary", "tool_calls")
    missing = [k for k in required if k not in payload]
    if missing:
        raise ValueError(
            f"avatar_command_result: отсутствуют поля {missing!r}"
        )
    if not isinstance(payload["ok"], bool):
        raise ValueError("avatar_command_result: ok должен быть bool")
    if not isinstance(payload["summary"], str):
        raise ValueError("avatar_command_result: summary должен быть строкой")
    if not isinstance(payload["tool_calls"], list):
        raise ValueError(
            "avatar_command_result: tool_calls должен быть list[str]"
        )
    if not all(isinstance(x, str) for x in payload["tool_calls"]):
        raise ValueError(
            "avatar_command_result: tool_calls должен быть list[str]"
        )
    if not isinstance(payload["request_id"], str) or not payload["request_id"]:
        raise ValueError(
            "avatar_command_result: request_id должен быть непустой строкой"
        )
