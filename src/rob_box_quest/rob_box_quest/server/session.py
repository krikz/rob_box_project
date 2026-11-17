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
# [voice-vr 07] / issue #2192: ``ErrorCode`` — read-only façade над
# :data:`rob_box_core.bridge_protocol.ERRORS`. Раньше жил в этом модуле
# обычным классом с дублями ``FLOOR_HELD``/``MODE_CONFLICT`` (см. issue
# #2192, ADR-0080 §2.2 инвариант 3). Теперь — единый источник истины
# в rob_box_core.bridge_protocol; legacy-импортеры
# (``from rob_box_quest.server.session import ErrorCode``) получают
# тот же набор атрибутов + два guard'а:
#
#   * ``ErrorCode.NOT_A_REAL_CODE`` → AttributeError с «не существует в
#     каноне» (а не silent ``AttributeError: 'ErrorCode' has no attribute``).
#   * ``ErrorCode.AUTH_FAIL = "HACKED"`` → AttributeError с «неизменяем»
#     (раньше присваивание проходило без ошибки — баг voice-vr 02).
class _ErrorCodeMeta(type):
    """Мета-класс для :class:`ErrorCode`.

    Канонический набор кодов подгружается лениво из
    :mod:`rob_box_core.bridge_protocol` и кладётся как class-level
    атрибуты на самом метаклассе; при ``ErrorCode.AUTH_FAIL`` Python
    сначала ищет атрибут в ``type(ErrorCode)`` (= ``_ErrorCodeMeta``),
    а не в самом ``ErrorCode``. Это даёт два эффекта:

      * ``__getattr__`` срабатывает как fallback для имён, которых нет
        в каноне (а не просто дефолтный ``AttributeError``).
      * ``__setattr__`` отвергает любое присваивание не-дандер имени.
    """

    def __getattr__(cls, name: str) -> str:  # noqa: D401
        from rob_box_core.bridge_protocol import is_known_error

        if name.startswith("_"):
            raise AttributeError(name)
        if is_known_error(name):
            # На случай гонки: код добавлен в канон, но bind-цикл
            # ещё не отработал — отдаём строку напрямую.
            return name
        raise AttributeError(
            f"ErrorCode.{name!r} не существует в каноне "
            "(см. rob_box_core.bridge_protocol.ERRORS)"
        )

    def __setattr__(cls, name: str, value: object) -> None:
        if name.startswith("_"):
            type.__setattr__(cls, name, value)
            return
        raise AttributeError(
            f"ErrorCode неизменяем: попытка присвоить {name!r}={value!r}; "
            f"для нового кода добавьте его в rob_box_core.bridge_protocol.ERRORS"
        )


class ErrorCode(metaclass=_ErrorCodeMeta):
    """Канонические коды ``ERROR.code`` (meta-quest-api.md §8).

    Используется как ``ErrorCode.BAD_PAYLOAD`` (строковая константа).
    Атрибуты хранятся на метаклассе (см. :class:`_ErrorCodeMeta`).
    Пустое тело класса — это нормально: bind выполняется сразу после
    определения класса (см. ниже).
    """


# Bind канонических кодов как атрибутов метакласса. Прямое присваивание
# в теле класса запрещено нашим ``__setattr__``, поэтому используем
# ``type.__setattr__`` на метаклассе.
from rob_box_core.bridge_protocol import ERRORS as _CANON_ERRORS  # noqa: E402

for _code in _CANON_ERRORS:
    type.__setattr__(_ErrorCodeMeta, _code, _code)
del _code
# AV-19 (issue #1911, ADR-0028 §4.4, meta-quest-api.md §5/§8):
# запрошенный teleop_floor уже держит другой client_id. Сервер
# отдаёт эту ошибку только при ``require_teleop_floor=true`` и
# rate-limited (≤ 1 Гц на сессию), чтобы не заливать сокет.
#
# [voice-vr 10] UNKNOWN_COMMAND (issue #2195): клиент прислал JSON_CMD
# с неизвестным ``cmd``. Терминальный dispatcher-fallback возвращает
# явный отказ вместо молчаливого drop. Канон живёт в
# rob_box_core.bridge_protocol.ERRORS; bind-цикл выше подхватит код
# автоматически (для legacy-импортеров через ``ErrorCode.UNKNOWN_COMMAND``).



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
