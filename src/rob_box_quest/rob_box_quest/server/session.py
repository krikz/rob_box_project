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
    INTERNAL = "INTERNAL"


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
    # subscribed: topic_ui_name -> server-initiated stream_id (0x1000..0xFFFF).
    subscribed: Dict[str, int] = field(default_factory=dict)
    # timestamps для watchdog (monotonic, seconds).
    last_ping_monotonic: Optional[float] = None
    last_heartbeat_monotonic: Optional[float] = None
    created_monotonic: float = field(default_factory=time.monotonic)
    # Phase 2.2 telemetry (ADR-0032 §3.5): rate-limit для 0x40 TELEMETRY_PERF.
    # Client шлёт 1 Hz; если чаще 5 Hz (анти-спам) → дроп после 10 быстрых.
    # last_telemetry_ms — wall-clock (ms), чтобы сравнивать с int(time.time()*1000).
    last_telemetry_ms: Optional[int] = None
    telemetry_fast_count: int = 0

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
