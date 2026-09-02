"""ClientSession — состояние одной WS-сессии rob_box_quest.

Чистая логика, без зависимостей от aiohttp / ROS / Zenoh.
Тестируется прямо в pytest без rclpy.

Источник истины: docs/architecture/meta-quest-api.md §1/§3/§7/§8,
docs/adr/0027-meta-quest-ar-control.md §3.3 (dead-man + watchdog).
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional


class SessionState(str, Enum):
    """FSM сессии (см. meta-quest-api.md §3 handshake)."""

    AWAITING_HELLO = "awaiting_hello"
    AUTHENTICATED = "authenticated"
    CLOSED = "closed"


# Error-коды из meta-quest-api.md §8.
class ErrorCode:
    AUTH_FAIL = "AUTH_FAIL"
    BAD_PAYLOAD = "BAD_PAYLOAD"
    TOPIC_UNKNOWN = "TOPIC_UNKNOWN"
    RATE_LIMIT = "RATE_LIMIT"
    PROTOCOL_VERSION = "PROTOCOL_VERSION"  # AV-16: subprotocol mismatch (§11)
    FLOOR_HELD = "FLOOR_HELD"  # AV-16: ACQUIRE_FLOOR / RELEASE_FLOOR (§8)
    MODE_CONFLICT = "MODE_CONFLICT"  # AV-16: SET_MODE отвергнут FSM (§8)
    INTERNAL = "INTERNAL"


# Поддерживаемые wire-subprotocol-версии (AV-16, docs §11.1).
# Порядок объявления важен: aiohttp выбирает первый совпавший, поэтому
# v2 объявлен первым и сервер его предпочитает при наличии.
SUPPORTED_SUBPROTOCOLS_V2: tuple[str, ...] = ("robbox-quest-v2", "robbox-quest-v1")
SUPPORTED_SUBPROTOCOLS_V1: tuple[str, ...] = ("robbox-quest-v1",)


def _subprotocol_to_version(subprotocol: Optional[str]) -> int:
    """Превратить ``ws.ws_protocol`` в наш внутренний subprotocol-version.

    v2 → 2, v1 → 1, None/unknown → 1 (по умолчанию, обратная совместимость
    с прежними клиентами, которые не объявляли subprotocol).
    """
    if subprotocol == "robbox-quest-v2":
        return 2
    return 1


def server_client_id(session_id: str) -> str:
    """Серверный ``client_id`` для ``Bridge.supervisor_*`` вызовов.

    Источник истины для client_id — сервер. По карточке AV-16/§11 требование:
    «клиент не должен уметь представиться Telegram-ом» → клиентский
    payload-``client_id`` игнорируется; сервер квантует по собственному
    ``session_id``. Формат — ``"quest:<uuid>"``: подтип (``quest``) явно
    отличает от telegram-клиентов в логах/метриках супервизора и при
    extensions на других клиентов (admin-panel, curl).
    """
    return f"quest:{session_id}"


# Heartbeat/watchdog тайминги (meta-quest-api.md §7 + ADR-0027 §3.3).
HEARTBEAT_INTERVAL_S = 0.2  # server → client (200 ms)
CLIENT_PING_INTERVAL_S = 5.0  # client → server ожидаемая частота
WATCHDOG_TIMEOUT_S = 0.6  # 3× heartbeat = 600 ms нет ping → close


@dataclass
class ClientSession:
    """Состояние одной WS-сессии. Создаётся на connect, мутируется по ходу.

    Потокобезопасность НЕ требуется — мутации только из event-loop aiohttp.
    """

    session_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    state: SessionState = SessionState.AWAITING_HELLO
    client_version: Optional[str] = None
    capabilities: list[str] = field(default_factory=list)
    # Subprotocol-version: 1 (Phase 1, robbox-quest-v1) или 2 (Phase 2,
    # robbox-quest-v2 + supervisor API). Заполняется через
    # ``apply_subprotocol`` ПОСЛЕ ``ws.prepare`` — aiohttp согласует
    # версию в WS-handshake, до этого момента поле = None. По нему
    # handler-ы 0x30..0x33 решают, можно ли слать STATE_UPDATE этому
    # клиенту и принимать от него supervisor-команды.
    protocol_version: Optional[int] = None
    # subscribed: topic_ui_name -> server-initiated stream_id (0x1000..0xFFFF).
    subscribed: Dict[str, int] = field(default_factory=dict)
    # timestamps для watchdog (monotonic, seconds).
    last_ping_monotonic: Optional[float] = None
    last_heartbeat_monotonic: Optional[float] = None
    created_monotonic: float = field(default_factory=time.monotonic)
    # Серверный client_id для supervisor API («quest:<session_id>»). Заполняется
    # в ``mark_authenticated``; используется в Bridge.supervisor_* вызовах
    # вместо client-supplied client_id из payload (см. AV-16/§11).
    server_client_id: Optional[str] = None

    def is_open(self) -> bool:
        return self.state != SessionState.CLOSED

    def mark_authenticated(self, client_version: str, capabilities: list[str]) -> None:
        """Перевод AWAITING_HELLO → AUTHENTICATED. Raises если уже не в начальном."""
        if self.state != SessionState.AWAITING_HELLO:
            raise RuntimeError(f"cannot authenticate: state={self.state.value}")
        self.state = SessionState.AUTHENTICATED
        self.client_version = client_version
        self.capabilities = list(capabilities)
        # Первый ping-таймер — момент рукопожатия.
        self.last_ping_monotonic = time.monotonic()
        # Серверный client_id, готов к supervisor_* вызовам (см. AV-16/§11).
        self.server_client_id = server_client_id(self.session_id)

    def apply_subprotocol(self, negotiated: Optional[str]) -> int:
        """Зафиксировать согласованный subprotocol-уровень сессии.

        Вызывается после ``aiohttp.WSResponse.prepare(request)`` —
        ``negotiated`` берётся из ``ws.ws_protocol``. None/unknown →
        subprotocol v1 (обратная совместимость со старыми клиентами,
        которые Sec-WebSocket-Protocol не объявляли).
        """
        self.protocol_version = _subprotocol_to_version(negotiated)
        return self.protocol_version

    def feed_ping(self, now_monotonic: Optional[float] = None) -> None:
        """Клиент прислал JSON_EVENT{type:"ping"} → сбрасываем watchdog."""
        self.last_ping_monotonic = now_monotonic if now_monotonic is not None else time.monotonic()

    def feed_heartbeat(self, now_monotonic: Optional[float] = None) -> None:
        """Сервер отправил heartbeat клиенту → фиксируем момент."""
        self.last_heartbeat_monotonic = now_monotonic if now_monotonic is not None else time.monotonic()

    def watchdog_tripped(self, now_monotonic: Optional[float] = None) -> bool:
        """True если клиент молчит дольше WATCHDOG_TIMEOUT_S.

        До аутентификации watchdog не считается (могут быть сетевые задержки).
        """
        if self.state != SessionState.AUTHENTICATED:
            return False
        if self.last_ping_monotonic is None:
            return False
        now = now_monotonic if now_monotonic is not None else time.monotonic()
        return (now - self.last_ping_monotonic) > WATCHDOG_TIMEOUT_S

    def allocate_stream_id(self, stream_ids_in_use: set[int]) -> int:
        """Выдать новый server-initiated stream_id из 0x1000..0xFFFF.

        Аргумент stream_ids_in_use — set всех уже занятых ID (включая
        stream_id'ы из других сессий на тот же subscription topic).
        Raises RuntimeError если пул исчерпан.
        """
        for sid in range(0x1000, 0x10000):
            if sid not in stream_ids_in_use:
                return sid
        raise RuntimeError("server-initiated stream_id pool exhausted")

    def close(self) -> None:
        self.state = SessionState.CLOSED


def generate_pin() -> str:
    """6-значный PIN (ADR-0027 §4.5). Без ведущих нулей-исключений,
    любая 6-значная последовательность цифр подходит (000000..999999).
    """
    import secrets

    return f"{secrets.randbelow(1_000_000):06d}"
