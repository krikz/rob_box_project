"""Catalog of the rob_box bridge wire-protocol.

Источник истины для имён команд, событий, стримов, режимов аватара и
кодов ошибок, которыми обмениваются клиенты шлема/моста (``rob_box_quest``
webxr_client, TS-сторона) и сервисы робота (``rob_box_quest.ws_server``,
``rob_box_quest.quest_node``, ``rob_box_supervisor``, ``rob_box_telegram``
как второй клиент).

Зачем (ADR-0080 §2.2 инвариант 3, CONTEXT.md «каталог протокола»,
issue #2192):

    До этого модуля имена собирались вручную в ПЯТИ местах:

      1. ``src/rob_box_quest/rob_box_quest/protocol/topics.py``
         — ``TOPIC_IDS`` (8 записей)
      2. ``src/rob_box_quest/rob_box_quest/streams/registry.py``
         — ``STREAM_CATALOG`` (9 записей, частично расходится с (1))
      3. ``src/rob_box_quest/rob_box_quest/server/ws_server.py``
         — ``VOICE_PRESET_IDS``, ``VOICE_LANGUAGES``,
         ``VALID_FLOORS_V2``, ``VALID_MODES_V2``,
         ``VOICE_PIPELINE_DEFAULT_LANGUAGE`` (разбросаны по модулю)
      4. ``src/rob_box_quest/rob_box_quest/server/session.py``
         — ``ErrorCode`` (9 кодов, дубль ``FLOOR_HELD``/``MODE_CONFLICT``
         + ``UNKNOWN_COMMAND``)
      5. ``src/rob_box_quest/webxr_client/src/wire/messages.ts``
         — ``JsonCmd`` / ``JsonEvent`` union-типы
         (часть имён сервер НЕ шлёт: ``ui_button``, ``set_panel_topic``,
         ``admin_logs`` — ``avatar_set_mode``/``avatar_acquire_floor``/
         ``avatar_release_floor`` сервер обрабатывает с закрытия
         voice-vr 09; ``voice_pipeline`` живёт в отдельном файле
         ``wire/voice_pipeline_cmd.ts``)

    Плюс шестое место, где контракт переписывался человеческим языком:

      6. ``docs/architecture/meta-quest-api.md`` §3–§8 — wire-протокол.

Что здесь:

    * ``COMMANDS`` — имена ``JSON_CMD.cmd`` (подмножество, которое
      сервер реально диспатчит; помечено ``server_dispatched=True``,
      для остальных сервер вернёт ``ERROR{BAD_PAYLOAD}``).
    * ``EVENTS`` — имена ``JSON_EVENT.type`` (включая события,
      которые сервер только ШЛЁТ, но не обрабатывает в ``_on_json_event``).
    * ``STREAMS`` — сервер-инициируемые BINARY_FRAME-стримы (topic_id + ui_name).
    * ``MODES`` — режимы аватара ``avatar_mode`` (FSM в avatar_supervisor).
    * ``FLOORS`` — два независимых ресурса блокировки.
    * ``ERRORS`` — коды ``ERROR.code`` (строковые константы).
    * ``VOICE_PRESETS`` / ``VOICE_LANGUAGES`` — whitelist для
      AV-28 §P7 (стиль речи + язык вывода LLM-формализации).
    * ``FRAME_TYPES`` — имена/числа из meta-quest-api.md §3 (для
      reference и conformance-тестов).

Этот модуль — чистая декларация, без зависимостей от ROS, msgpack,
aiohttp. Импорт стоит ``0`` времени и безопасен в conftest.

Кто импортирует:

    * ``rob_box_quest.server.session`` — ``ErrorCode`` (через тонкую
      обёртку для backward-compat с существующими ``from rob_box_quest.server.session
      import ErrorCode`` в тестах и в коде).
    * ``rob_box_quest.protocol.topics`` — ``TOPIC_IDS`` (тонкая обёртка).
    * ``rob_box_quest.streams.registry`` — ``STREAM_CATALOG``.
    * ``rob_box_quest.server.ws_server`` — ``VOICE_PRESET_IDS``,
      ``VOICE_LANGUAGES``, ``VALID_FLOORS_V2``, ``VALID_MODES_V2``.
    * Conformance-тесты — напрямую из этого модуля, без грепа по
      ``ws_server.py`` (как требует voice-vr 02 и ADR-0080 §2.2).

Out of scope для этой карточки (см. issue #2192 / ADR-0080):

    * Runtime-валидация payload'ов (JSON Schema, msgpack-схемы) — здесь
      только «shape» (имя + обязательные поля + версия subprotocol).
    * Генерация TS-типов из Python — отдельная карточка (voice-vr 08).
    * Диспетчер по таблице (табличный switch вместо if-цепочки в
      ``_on_json_cmd``) — отдельная карточка (voice-vr 10).
    * Amendment к docs/architecture/meta-quest-api.md по итогам
      каталога — отдельная карточка (voice-vr 11).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Optional


# === Wire payload value grammar (voice-vr 08) ================================
#
# Single source of truth for the JSON payload shape that travels over each
# ``cmd`` / ``type`` discriminant. ``tools/gen_bridge_protocol_ts.py`` reads
# this from :class:`CommandSpec.payload` / :class:`EventSpec.payload` to emit
# ``webxr_client/src/wire/protocol_generated.ts``; CI guards drift via
# ``.github/workflows/G-Bridge-Protocol-Drift.yml``.
#
# Grammar (mirror of ``_bridge_protocol_data._is_valid_payload_value``;
# that module was removed in voice-vr 08 — payload now lives on the
# canonical spec, not in a parallel data dict):
#
#   * ``str`` — atomic TS type (``"str"→string``, ``"int"→number``,
#     ``"float"→number``, ``"bool"→boolean``, ``"unknown"→unknown``).
#   * ``"type":"literal"`` dict — string-literal union
#     (``{"a" | "b" | "c"}`` in TS). Optional via ``"optional": True``.
#   * inline-object dict — TS inline ``{ x: number; y: number }``.
#
# ``?`` suffix on atomic / composite values marks the field optional — this is
# the same convention the old ``_bridge_protocol_data.py`` used, kept here
# so re-generated TS keeps identical field marks. Optimality is therefore
# written ONCE (on the payload value) and ``required_fields`` /
# ``optional_fields`` are derived properties — the convention that prevents
# the "two truths about one field" drift this card existed to eliminate.
PayloadValue = str | dict[str, Any]
Payload = Mapping[str, PayloadValue]


def _p(d: dict[str, PayloadValue]) -> Payload:
    """Freeze a payload mapping so callers can't mutate it post-declare.

    Returned as :class:`types.MappingProxyType` — read-only view; the
    generator and tests can still iterate it.
    """
    return MappingProxyType(d)


# === Frame types (meta-quest-api.md §3) =====================================
#
# Server/client обмениваются бинарными фреймами:
#   [1 byte: type][4 bytes: stream_id (LE)][varint: payload_len][payload]
#
# Числа зафиксированы в meta-quest-api.md и в
# ``src/rob_box_quest/rob_box_quest/protocol/frame.py::FrameType``.
# Здесь — reference для документации и conformance-тестов.


class FrameTypeId:
    """Имена frame-type из meta-quest-api.md §3.

    Числа совпадают с ``protocol.frame.FrameType`` (IntEnum).
    Поле ``value`` — это 1-байтовый идентификатор, второй столбец
    в таблице §3.
    """

    HELLO = 0x01
    WELCOME = 0x02
    SUBSCRIBE = 0x03
    UNSUBSCRIBE = 0x04
    BINARY_FRAME = 0x10
    JSON_CMD = 0x11
    JSON_EVENT = 0x12
    GOODBYE = 0x20
    VOICE_AUDIO = 0x21
    ERROR = 0xFF
    # AV-16 / issue #1906 (ADR-0028 §4.4): supervisor API (v2 only).
    SET_MODE = 0x30
    ACQUIRE_FLOOR = 0x31
    RELEASE_FLOOR = 0x32
    STATE_UPDATE = 0x33


# === Helpers =================================================================


@dataclass(frozen=True)
class CommandSpec:
    """Описание одной ``JSON_CMD.cmd``, которую клиент может слать.

    Attributes:
        name: имя команды (``cmd: "<name>"`` в payload).
        payload: полный JSON-payload с типами полей (``int`` / ``str`` /
            ``bool`` / ``{"type":"literal", "values":[…]}`` / inline-object).
            Источник истины для ``required_fields`` / ``optional_fields``
            (derived properties ниже) и для TS-зеркала через
            ``tools/gen_bridge_protocol_ts.py``. ``payload["cmd"]``
            ОБЯЗАН совпадать с ``name`` — conformance-тест это проверяет.
        subprotocol: минимальная версия subprotocol, на которой команда
            доступна (``"v1"``, ``"v2"``, или ``"any"`` для обеих).
        server_dispatched: сервер реально имеет обработчик ветки в
            ``_on_json_cmd``. ``False`` → сервер вернёт ``ERROR{BAD_PAYLOAD}``;
            такие команды помечены для visibility (TS-тип обявлен,
            но сервер не обрабатывает) и для будущих карточек (voice-vr 09).
        description: человеко-описание (для документации/UI).
    """

    name: str
    payload: Payload = field(default_factory=lambda: _p({}))
    subprotocol: str = "any"  # "v1" | "v2" | "any"
    server_dispatched: bool = True
    description: str = ""

    @property
    def required_fields(self) -> tuple[str, ...]:
        """Имена обязательных полей (derived от ``payload``).

        Поле требуется ⇔ в payload оно либо atomic/composite без суффикса
        ``?``, либо literal-словарь без ``"optional": True``. Суффикс
        ``?`` на atomic/composite — единственный способ пометить поле
        опциональным; ``required_fields`` и ``optional_fields`` всегда
        согласованы, потому что оба вычисляются из одного источника.
        """
        out: list[str] = []
        for fname, ftype in self.payload.items():
            if isinstance(ftype, str):
                if not ftype.endswith("?"):
                    out.append(fname)
                continue
            if isinstance(ftype, dict):
                if ftype.get("type") == "literal":
                    if not ftype.get("optional", False):
                        out.append(fname)
                    continue
                # inline-object: все поля inline-объекта обязательны.
                out.append(fname)
        return tuple(out)

    @property
    def optional_fields(self) -> tuple[str, ...]:
        """Имена опциональных полей (derived от ``payload``)."""
        out: list[str] = []
        for fname, ftype in self.payload.items():
            if isinstance(ftype, str):
                if ftype.endswith("?"):
                    out.append(fname)
                continue
            if isinstance(ftype, dict):
                if ftype.get("type") == "literal" and ftype.get("optional", False):
                    out.append(fname)
        return tuple(out)


@dataclass(frozen=True)
class EventSpec:
    """Описание одного ``JSON_EVENT.type``.

    Атрибуты симметричны :class:`CommandSpec`. ``server_emitted`` —
    сервер ШЛЁТ это событие (через ``_send`` из разных хендлеров).
    ``server_handled`` — сервер ОБРАБАТЫВАЕТ входящий ``_on_json_event``
    (на сегодня только ``ping``).
    """

    name: str
    payload: Payload = field(default_factory=lambda: _p({}))
    subprotocol: str = "any"
    server_emitted: bool = True
    server_handled: bool = False
    description: str = ""

    @property
    def required_fields(self) -> tuple[str, ...]:
        out: list[str] = []
        for fname, ftype in self.payload.items():
            if isinstance(ftype, str):
                if not ftype.endswith("?"):
                    out.append(fname)
                continue
            if isinstance(ftype, dict):
                if ftype.get("type") == "literal":
                    if not ftype.get("optional", False):
                        out.append(fname)
                    continue
                out.append(fname)
        return tuple(out)

    @property
    def optional_fields(self) -> tuple[str, ...]:
        out: list[str] = []
        for fname, ftype in self.payload.items():
            if isinstance(ftype, str):
                if ftype.endswith("?"):
                    out.append(fname)
                continue
            if isinstance(ftype, dict):
                if ftype.get("type") == "literal" and ftype.get("optional", False):
                    out.append(fname)
        return tuple(out)


class StreamKind(str, Enum):
    """Источник данных для :class:`StreamSpec` (server-initiated streams).

    ``str``-родитель даёт ``StreamKind.ROS_TOPIC == "ros_topic"`` (для
    обратной совместимости со старыми потребителями, которые сравнивают
    ``spec.kind`` со строкой-литералом).
    """

    ROS_TOPIC = "ros_topic"
    CAMERA_DIRECT = "camera_direct"


@dataclass(frozen=True)
class StreamSpec:
    """Сервер-инициируемый стрим (meta-quest-api.md §4).

    Attributes:
        ui_name: имя, которое клиент видит в ``SUBSCRIBE{topic: ...}``.
        topic_id: 4-байтовый little-endian uint32 в payload
            ``BINARY_FRAME`` (для клиентского парсера, см. §4).
        kind: источник данных (``ros_topic`` или ``camera_direct``).
        source: для ``ros_topic`` — имя ROS-топика; для ``camera_direct``
            — device-id (``"oak:color"``, ``"oak:depth"``, ``"ceiling"``).
        default_quality: human-hint для UI (``"low"|"med"|"high"``).
        description: человеко-описание.
    """

    ui_name: str
    topic_id: int
    kind: StreamKind
    source: str
    default_quality: str = "med"
    description: str = ""


# === COMMANDS (JSON_CMD.cmd, meta-quest-api.md §5 + §5.1) ===================
#
# Полный список 1:1 с тем, что сервер реально диспатчит в ``_on_json_cmd``
# или в ``_handle_supervisor_command`` (для supervisor_* frame-types 0x30-0x33;
# они НЕ идут через JSON_CMD.cmd — это отдельный frame-type с msgpack).
#
# Команды, которые сервер НЕ обрабатывает (ui_button, set_panel_topic,
# admin_logs*) — перечислены здесь с ``server_dispatched=False`` для
# visibility (TS-типы объявлены), но помечены явно. Их серверная
# поддержка — отдельные карточки. avatar_set_mode/avatar_acquire_floor/
# avatar_release_floor раньше тоже были в этом списке — voice-vr 09
# закрыт, сервер их диспатчит, поэтому server_dispatched=True (default).


COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(
        name="ping",
        # АНОМАЛИЯ (voice-vr 08): оригинальный _bridge_protocol_data.py
        # добавлял сюда ``nonce: str?`` со ссылкой на «echo в pong», но
        # ни _on_json_cmd (ws_server.py:1876), ни ws_handler.py не читают
        # ``nonce`` из ping. Удалено, чтобы derived ``required_fields``
        # совпадал с канон-контрактом (этот spec до voice-vr 08
        # имел required_fields=("cmd","ts_ms"), opt=()). Если клиент
        # действительно шлёт ``nonce`` — это отдельная карточка.
        payload=_p({"cmd": "ping",
            "ts_ms": "int",}),
        subprotocol="any",
        description="Watchdog keepalive (meta-quest-api.md §7).",
    ),
    CommandSpec(
        name="stream_list",
        payload=_p({"cmd": "stream_list",
            "ts_ms": "int",}),
        subprotocol="v1",
        description="Запрос списка доступных стримов (R10, §5).",
    ),
    CommandSpec(
        name="stream_select",
        payload=_p({"cmd": "stream_select",
            "ts_ms": "int",
            "topic": "str",}),
        subprotocol="v1",
        description="Переключение активного стрима для UI-панели (§6.2).",
    ),
    CommandSpec(
        name="teleop_twist",
        # АНОМАЛИЯ (voice-vr 08): _bridge_protocol_data.py помечал
        # ``seq: int`` (required). ws_server.py:1914 (``relay_teleop_heartbeat``
        # branch) default'ит seq=0 если его нет. Канон до voice-vr 08
        # держал ``seq`` в optional_fields. Возвращено ``int?`` — клиент
        # может слать без seq. Если relay обязан получать seq — это
        # отдельная карточка (issue для ws_server, не для каталога).
        payload=_p({"cmd": "teleop_twist",
            "ts_ms": "int",
            "seq": "int?",
            "linear": {"x": "float", "y": "float", "z": "float"},
            "angular": {"x": "float", "y": "float", "z": "float"},
            "deadman": "bool",}),
        subprotocol="v1",
        description="Телеоп: linear/angular + deadman-grip (§5, AV-19).",
    ),
    CommandSpec(
        name="teleop_heartbeat",
        payload=_p({"cmd": "teleop_heartbeat",
            "ts_ms": "int",
            "seq": "int",}),
        subprotocol="v1",
        description="Heartbeat клиента пока держит teleop_floor (AV-19).",
    ),
    CommandSpec(
        name="stop_emergency",
        # АНОМАЛИЯ (voice-vr 08): _bridge_protocol_data.py помечал
        # ``source`` как обязательный literal. ws_server.py на stop_emergency
        # вообще не читает source (только публикует в /safety/emergency_stop).
        # Канон до voice-vr 08 имел source в optional_fields. Возвращён
        # optional=True — если клиент реально шлёт source, выяснить это
        # по ws_server выходит за скоуп voice-vr 08.
        payload=_p({"cmd": "stop_emergency",
            "ts_ms": "int",
            "source": {"type": "literal", "values": ["controller_b", "ui_button", "client_lost"], "optional": True},}),
        subprotocol="any",
        description="Аварийная остановка — всегда в обход гейта floor (§5).",
    ),
    CommandSpec(
        name="voice_ptt_start",
        # АНОМАЛИЯ (voice-vr 08): _bridge_protocol_data.py не имел
        # ``client_id``. Канон до voice-vr 08 держал его в optional_fields
        # (см. ws_server.py: обработка client_id для multi-client routing).
        # Добавлен как ``str?``.
        payload=_p({"cmd": "voice_ptt_start",
            "ts_ms": "int",
            "client_id": "str?",
            "mode": {"type": "literal", "values": ["radio", "robot_voice"], "optional": True},}),
        subprotocol="v1",
        description="Push-to-talk start; mode='radio'|'robot_voice' (§5).",
    ),
    CommandSpec(
        name="voice_ptt_stop",
        payload=_p({"cmd": "voice_ptt_stop",
            "ts_ms": "int",
            "mode": {"type": "literal", "values": ["radio", "robot_voice"], "optional": True},}),
        subprotocol="v1",
        description="Push-to-talk stop (§5).",
    ),
    CommandSpec(
        name="voice_mode",
        payload=_p({"cmd": "voice_mode",
            "ts_ms": "int",
            "mode": {"type": "literal",
                     "values": ["off", "passthrough", "ttts_proxy", "stt_llm", "llm_formalize"]},}),
        subprotocol="v1",
        description="Смена voice_input_mode (ADR-0027 §3.4).",
    ),
    # ADR-0071 step 5a: панельный тумблер «всегда слушать». Старые имена,
    # оставлены для обратной совместимости (см. ws_server.py:1864).
    CommandSpec(
        name="voice_listen_start",
        payload=_p({"cmd": "voice_listen_start",
            "ts_ms": "int",}),
        subprotocol="v1",
        description="(deprecated) wake-stream ON; используйте wake-API напрямую.",
    ),
    CommandSpec(
        name="voice_listen_stop",
        payload=_p({"cmd": "voice_listen_stop",
            "ts_ms": "int",}),
        subprotocol="v1",
        description="(deprecated) wake-stream OFF.",
    ),
    # ── AV-16 / ADR-0028 §4.4: supervisor commands (JSON-эквиваленты, §5.1)
    CommandSpec(
        name="supervisor_set_mode",
        # АНОМАЛИЯ (voice-vr 08): _bridge_protocol_data.py добавлял
        # ``seq: int?``. Канон до voice-vr 08 НЕ имел seq (req=
        # ``(cmd, ts_ms, client_id, mode)``, opt=()). Возвращено к
        # канону — seq удалён, чтобы derived соответствовал контракту.
        # Если клиент реально шлёт seq (для корреляции с STATE_UPDATE)
        # — это отдельная карточка.
        payload=_p({"cmd": "supervisor_set_mode",
            "ts_ms": "int",
            "client_id": "str",
            "mode": {"type": "literal",
                     "values": ["off", "telegram_active", "avatar_present", "mixed",
                                "teleop_only", "voice_only"]},}),
        subprotocol="v2",
        description="Смена avatar_mode (FSM супервизора, §3/§5.1).",
    ),
    CommandSpec(
        name="supervisor_acquire_floor",
        # АНОМАЛИЯ (voice-vr 08): см. supervisor_set_mode.
        payload=_p({"cmd": "supervisor_acquire_floor",
            "ts_ms": "int",
            "client_id": "str",
            "floor": {"type": "literal", "values": ["teleop", "voice"]},}),
        subprotocol="v2",
        description="Запрос floor: teleop|voice (§5.1).",
    ),
    CommandSpec(
        name="supervisor_release_floor",
        # АНОМАЛИЯ (voice-vr 08): см. supervisor_set_mode.
        payload=_p({"cmd": "supervisor_release_floor",
            "ts_ms": "int",
            "client_id": "str",
            "floor": {"type": "literal", "values": ["teleop", "voice"]},}),
        subprotocol="v2",
        description="Освобождение floor (§5.1).",
    ),
    CommandSpec(
        name="supervisor_get_state",
        payload=_p({"cmd": "supervisor_get_state",
            "ts_ms": "int",}),
        subprotocol="v2",
        description="Poll-эквивалент STATE_UPDATE (§5.1).",
    ),
    # ── AV-27 / issue #1919 — TTS picker (§4.1+§4.3)
    CommandSpec(
        name="list_voices",
        payload=_p({"cmd": "list_voices",
            "ts_ms": "int",}),
        subprotocol="v1",
        description="Запрос списка доступных голосов (§4.1).",
    ),
    CommandSpec(
        name="set_voice",
        # АНОМАЛИЯ (voice-vr 08): _bridge_protocol_data.py помечал
        # ``voice_id`` как required. Канон до voice-vr 08 держал его
        # в optional_fields (preset/language — это РАЗНЫЕ ветки
        # dispatch в ws_server.py:2156, не одновременные). Возвращён
        # ``str?``.
        payload=_p({"cmd": "set_voice",
            "ts_ms": "int",
            "voice_id": "str?",
            "preset": {"type": "literal",
                       "values": ["standard", "friendly", "authoritative", "whisper",
                                  "technical", "street", "caveman", "business",
                                  "philosopher", "lenin"],
                       "optional": True},
            "language": {"type": "literal", "values": ["ru", "en"], "optional": True},}),
        subprotocol="v1",
        description=(
            "Установка voice_id (AV-27) или preset/language (AV-28 §P7). "
            "Диспетчер по ЗНАЧЕНИЮ preset'а, см. ws_server.py:2156."
        ),
    ),
    CommandSpec(
        name="voice_pipeline",
        # АНОМАЛИЯ (voice-vr 08): _bridge_protocol_data.py требовал
        # llm_enabled/preset/language как обязательные поля. Канон до
        # voice-vr 08 имел их все в optional_fields (preset/language
        # валидируются ТОЛЬКО если заданы, см. ws_server.py:
        # _validate_voice_pipeline_payload → ``if preset not in
        # VOICE_PRESET_IDS: raise``). Возвращены в ``str?``/``bool?``/
        # ``literal[optional]``. Если клиент реально шлёт все три —
        # отдельная карточка (handshake-уровень).
        payload=_p({"cmd": "voice_pipeline",
            "ts_ms": "int",
            "llm_enabled": "bool?",
            "preset": "str?",
            "language": {"type": "literal", "values": ["ru", "en", "fr", "de", "zh", "hi"], "optional": True},}),
        subprotocol="v2",
        description=(
            "Конфиг grip-пайплайна (трансформация STT→LLM→TTS), "
            "см. supervisor_node.py:_on_grip_voice_pipeline."
        ),
    ),
    CommandSpec(
        name="preview_voice",
        payload=_p({"cmd": "preview_voice",
            "ts_ms": "int",
            "voice_id": "str",
            "text": "str",
            "request_id": "str",}),
        subprotocol="v2",
        description="Синтез превью голоса (§4.2).",
    ),
    # ── [voice-vr 09 / issue #2194] Канонические имена (ADR-0080 §1.2).
    # Раньше здесь стоял server_dispatched=False (сервер игнорировал —
    # кнопки режима/floor на мостике молча не работали). Карточка
    # voice-vr 09 закрыта: ws_server._on_json_cmd теперь диспатчит эти
    # три имени рядом с legacy supervisor_* алиасом (см. ws_server.py,
    # блок «[voice-vr 09] канонические имена»).
    CommandSpec(
        name="avatar_set_mode",
        # canonical required_fields=("cmd", "ts_ms", "mode"),
        # optional_fields=("reason",).
        payload=_p({"cmd": "avatar_set_mode",
            "ts_ms": "int",
            "mode": "str",
            "reason": "str?"}),
        subprotocol="any",
        description=(
            "Канонический алиас supervisor_set_mode (ADR-0080 §1.2). "
            "Шлётся webxr_client (main.ts:154); voice-vr 09 закрыт — "
            "сервер обрабатывает наравне с legacy supervisor_set_mode."
        ),
    ),
    CommandSpec(
        name="avatar_acquire_floor",
        # canonical required_fields=("cmd", "ts_ms", "kind").
        payload=_p({"cmd": "avatar_acquire_floor",
            "ts_ms": "int",
            "kind": "str"}),
        subprotocol="any",
        description=(
            "Канонический алиас supervisor_acquire_floor (ADR-0080 §1.2). "
            "Шлётся webxr_client (main.ts:172); voice-vr 09 закрыт."
        ),
    ),
    CommandSpec(
        name="avatar_release_floor",
        # canonical required_fields=("cmd", "ts_ms", "kind").
        payload=_p({"cmd": "avatar_release_floor",
            "ts_ms": "int",
            "kind": "str"}),
        subprotocol="any",
        description=(
            "Канонический алиас supervisor_release_floor (ADR-0080 §1.2). "
            "Шлётся webxr_client (main.ts:175); voice-vr 09 закрыт."
        ),
    ),
    CommandSpec(
        name="set_panel_topic",
        payload=_p({"cmd": "set_panel_topic",
            "ts_ms": "int",
            "panel_id": "str",
            "topic": "str",}),
        subprotocol="any",
        server_dispatched=False,
        description=(
            "Описан в meta-quest-api.md §6.2 и TS messages.ts:154, "
            "но обработчик на сервере не подключён (Phase 2, "
            "отдельная карточка)."
        ),
    ),
    CommandSpec(
        name="ui_button",
        payload=_p({"cmd": "ui_button",
            "ts_ms": "int",
            "button": "str",
            "press": "bool",}),
        subprotocol="any",
        server_dispatched=False,
        description=(
            "(Phase 2, R14) Описан в meta-quest-api.md §5 "
            "(sound_play/sound_stop/light_*), TS-тип отсутствует. "
            "Отдельная карточка voice-vr (ещё не создана) — "
            "перевод в Phase 2, registry из rob_box_voice/command_node.py."
        ),
    ),
    CommandSpec(
        name="admin_logs",
        payload=_p({"cmd": "admin_logs",
            "ts_ms": "int",
            "service": {"type": "literal", "values": ["dialogue_node", "rob_box_quest", "all"]},
            "tail": "int?",
            "follow": "bool?",}),
        subprotocol="any",
        server_dispatched=False,
        description=(
            "(Phase 2, R14) Описан в §5, обработчик не подключён."
        ),
    ),
    CommandSpec(
        name="admin_logs_stop",
        payload=_p({"cmd": "admin_logs_stop",
            "ts_ms": "int",}),
        subprotocol="any",
        server_dispatched=False,
        description=(
            "(Phase 2, R14) Описан в §5; см. admin_logs. "
            "Обработчик не подключён."
        ),
    ),
)


# === EVENTS (JSON_EVENT.type, meta-quest-api.md §6 + §8) ===================


EVENTS: tuple[EventSpec, ...] = (
    EventSpec(
        name="subscribe_ack",
        payload=_p({"type": "subscribe_ack",
            "topic": "str",
            "stream_id": "int",
            "quality": "str",
            "kind": "str?",}),
        subprotocol="any",
        description="Подтверждение SUBSCRIBE (§6).",
    ),
    EventSpec(
        name="subscribe_nack",
        payload=_p({"type": "subscribe_nack",
            "topic": "str",
            "reason": "str",}),
        subprotocol="any",
        description="Отказ SUBSCRIBE (§6).",
    ),
    EventSpec(
        name="heartbeat",
        payload=_p({"type": "heartbeat",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="200 мс keepalive от сервера (§7).",
    ),
    EventSpec(
        name="ping",
        payload=_p({"type": "ping",
            "ts_ms": "int",
            "nonce": "str?",}),
        subprotocol="any",
        server_handled=True,
        description="Клиентский keepalive; триггерит pong (§7).",
    ),
    EventSpec(
        name="pong",
        payload=_p({"type": "pong",
            "ts_ms": "int",
            "nonce": "str?",}),
        subprotocol="any",
        server_emitted=True,
        description="Ответ сервера на ping (§6/§7).",
    ),
    EventSpec(
        name="stream_list",
        payload=_p({"type": "stream_list",
            "topics": "list[str]",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Ответ на stream_list cmd (§6).",
    ),
    EventSpec(
        name="stream_select_ack",
        payload=_p({"type": "stream_select_ack",
            "topic": "str",
            "stream_id": "int?",
            "kind": "str?",}),
        subprotocol="any",
        server_emitted=True,
        description="Подтверждение stream_select (§6.2).",
    ),
    EventSpec(
        name="voice_state",
        payload=_p({"type": "voice_state",
            "state": {"type": "literal",
                      "values": ["idle", "listening", "thinking", "speaking", "denied"]},
            "ts_ms": "int",
            "utterance_id": "str?",
            "holder_id": "str?",
            "detail": "str?",}),
        subprotocol="any",
        server_emitted=True,
        description=(
            "idle|listening|thinking|speaking|denied — FSM-стейт голоса (§6). "
            "Также идёт через BINARY_FRAME topic_id=0x1202."
        ),
    ),
    EventSpec(
        name="voice_mode_ack",
        payload=_p({"type": "voice_mode_ack",
            "mode": "str",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="ACK на voice_mode cmd (§6).",
    ),
    EventSpec(
        name="voice_listen_ack",
        # canonical required_fields=("type", "active", "ts_ms").
        payload=_p({"type": "voice_listen_ack",
            "active": "bool",
            "ts_ms": "int"}),
        subprotocol="any",
        server_emitted=True,
        description="ACK на voice_listen_start/stop (ADR-0071).",
    ),
    EventSpec(
        name="voice_list",
        payload=_p({"type": "voice_list",
            "voices": "list[VoiceInfo]",
            "active_provider": "str?",
            "active_voice": "str?",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Список голосов в ответ на list_voices (§4.1).",
    ),
    EventSpec(
        name="voice_set_ack",
        payload=_p({"type": "voice_set_ack",
            "voice_id": "str",
            "preset": "str",
            "language": "str",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="ACK на set_voice (§4.3).",
    ),
    EventSpec(
        name="voice_set_nack",
        payload=_p({"type": "voice_set_nack",
            "voice_id": "str?",
            "preset": "str?",
            "language": "str?",
            "reason": "str",
            "available": "list[str]?",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="NACK на set_voice (§4.3).",
    ),
    EventSpec(
        name="voice_pipeline_ack",
        payload=_p({"type": "voice_pipeline_ack",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="ACK на voice_pipeline cmd.",
    ),
    EventSpec(
        name="voice_pipeline_nack",
        payload=_p({"type": "voice_pipeline_nack",
            "reason": "str",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="NACK на voice_pipeline cmd.",
    ),
    EventSpec(
        name="preview_voice_audio",
        payload=_p({"type": "preview_voice_audio",
            "request_id": "str",
            "format": {"type": "literal", "values": ["mp3", "opus", "wav"]},
            "content_type": "str",
            "seq": "int",
            "total": "int",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Чанк аудио превью (§4.2).",
    ),
    EventSpec(
        name="preview_voice_done",
        payload=_p({"type": "preview_voice_done",
            "request_id": "str",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Финал превью (§4.2).",
    ),
    EventSpec(
        name="preview_voice_error",
        payload=_p({"type": "preview_voice_error",
            "request_id": "str",
            "reason": "str",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Ошибка превью (§4.2).",
    ),
    EventSpec(
        name="supervisor_state",
        payload=_p({"type": "supervisor_state",
            "state": "Record<string, unknown>",
            "ts_ms": "int",}),
        subprotocol="v2",
        server_emitted=True,
        description="Snapshot FSM супервизора (§5.1, STATE_UPDATE-эквивалент).",
    ),
    EventSpec(
        name="safety_stop",
        payload=_p({"type": "safety_stop",
            "reason": {"type": "literal", "values": ["controller_b", "client_lost"]},
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Аварийная остановка (§6).",
    ),
    EventSpec(
        name="robot_alert",
        payload=_p({"type": "robot_alert",
            "code": "str",
            "level": {"type": "literal", "values": ["warn", "error", "info"]},
            "active": "bool?",
            "args": "Record<string, unknown>?",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Алёрт робота (battery/wifi/stuck/...) (§6).",
    ),
    EventSpec(
        name="floor_lost",
        payload=_p({"type": "floor_lost",
            "floor": {"type": "literal", "values": ["teleop", "voice"]},
            "reason": "str?",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Tэта сессия больше не держит teleop_floor/voice_floor (§6, AV-19).",
    ),
    EventSpec(
        name="admin_logs_chunk",
        payload=_p({"type": "admin_logs_chunk",
            "service": {"type": "literal", "values": ["dialogue_node", "rob_box_quest", "all"]},
            "lines": "list[str]",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Стрим логов (R14, §6).",
    ),
    EventSpec(
        name="admin_logs_end",
        payload=_p({"type": "admin_logs_end",
            "service": "str",
            "ts_ms": "int",}),
        subprotocol="any",
        server_emitted=True,
        description="Финал стрима логов (R14, §6).",
    ),
)


# === STREAMS (BINARY_FRAME topic, meta-quest-api.md §4) =====================
#
# Источник истины для id'ов стримов. Раньше жили в
# ``protocol/topics.py::TOPIC_IDS`` (только topic_id, без source/kind)
# и в ``streams/registry.py::STREAM_CATALOG`` (полные spec, но без
# единой нумерации). Здесь объединено.

STREAMS: tuple[StreamSpec, ...] = (
    StreamSpec(
        ui_name="camera_rear",
        topic_id=0x1001,
        kind=StreamKind.ROS_TOPIC,
        source="/camera/camera/color/image_raw",
        default_quality="med",
        description="OAK-D color (Phase 1 fallback через ROS).",
    ),
    StreamSpec(
        ui_name="camera_front",
        topic_id=0x1002,
        kind=StreamKind.ROS_TOPIC,
        source="/camera/front/image_raw",
        default_quality="med",
        description="(Phase 2) панорамная передняя камера.",
    ),
    StreamSpec(
        ui_name="camera_oak_color",
        topic_id=0x1003,
        kind=StreamKind.CAMERA_DIRECT,
        source="oak:color",
        default_quality="med",
        description="OAK-D color (depthai SDK, in-process).",
    ),
    StreamSpec(
        ui_name="camera_oak_depth",
        topic_id=0x1004,
        kind=StreamKind.ROS_TOPIC,
        source="/camera/camera/depth/image_rect_raw/compressedDepth",
        default_quality="med",
        description=(
            "OAK-D depth (oak-d ROS topic, перекодировано в цветной JPEG)."
        ),
    ),
    StreamSpec(
        ui_name="camera_ceiling",
        topic_id=0x1005,
        kind=StreamKind.ROS_TOPIC,
        source="/ceiling_camera/image_raw/compressed",
        default_quality="med",
        description="USB ceiling camera (usb_cam → image_transport JPEG).",
    ),
    StreamSpec(
        ui_name="lidar_2d",
        topic_id=0x1101,
        kind=StreamKind.ROS_TOPIC,
        source="/scan",
        default_quality="high",
        description="2D LiDAR (sensor_msgs/LaserScan, 10 Hz).",
    ),
    StreamSpec(
        ui_name="lidar_3d",
        topic_id=0x1102,
        kind=StreamKind.ROS_TOPIC,
        source="/rtabmap/cloud_map",
        default_quality="low",
        description="3D point cloud (zstd+msgpack, 2 Hz).",
    ),
    StreamSpec(
        ui_name="map_2d",
        topic_id=0x1103,
        kind=StreamKind.ROS_TOPIC,
        source="/rtabmap/map",
        default_quality="low",
        description="SLAM occupancy grid (RGBA PNG + поза робота).",
    ),
    StreamSpec(
        ui_name="robot_status",
        topic_id=0x1201,
        kind=StreamKind.ROS_TOPIC,
        source="aggregation",
        default_quality="med",
        description="1 Hz battery/wifi/mode/vel.",
    ),
    StreamSpec(
        ui_name="voice_state",
        topic_id=0x1202,
        kind=StreamKind.ROS_TOPIC,
        source="/voice/dialogue/state",
        default_quality="med",
        description="Dialogue FSM state (event-driven).",
    ),
    StreamSpec(
        ui_name="person_detections",
        topic_id=0x1301,
        kind=StreamKind.ROS_TOPIC,
        source="phase2_source",
        default_quality="med",
        description="(Phase 2 R11) детекция людей в стриме.",
    ),
)


# === MODES (avatar_mode, ADR-0028 §4.1) =====================================


class Mode(str, Enum):
    """Режим аватара — FSM в ``avatar_supervisor``.

    Значения совпадают с ``AVATAR_MODES`` в
    ``src/rob_box_supervisor/.../core/fsm.py`` (single source of truth
    FSM). Здесь — reference для серверной валидации и для
    conformance-тестов между client (TS) и server (Python).
    """

    OFF = "off"
    TELEGRAM_ACTIVE = "telegram_active"
    AVATAR_PRESENT = "avatar_present"
    MIXED = "mixed"
    TELEOP_ONLY = "teleop_only"
    VOICE_ONLY = "voice_only"


MODES: tuple[str, ...] = tuple(m.value for m in Mode)


# === FLOORS (avatar_floor, ADR-0028 §4.2) ====================================


class Floor(str, Enum):
    """Два независимых ресурса блокировки (LockManager в арбитре)."""

    TELEOP = "teleop"
    VOICE = "voice"


FLOORS: tuple[str, ...] = tuple(f.value for f in Floor)


# === ERRORS (ERROR.code, meta-quest-api.md §8) ===============================
#
# До этого модуля ``ErrorCode`` жил в ``server/session.py`` классом с
# дублями ``FLOOR_HELD``/``MODE_CONFLICT`` (строки 34 и 41; см. issue
# #2192). Здесь — единый набор строк-констант и dataclass-обёртка
# ``ErrorCodeSpec`` для документации.

ERRORS: tuple[str, ...] = (
    # AUTH_FAIL — неверный PIN или сессия не аутентифицирована.
    "AUTH_FAIL",
    # BAD_PAYLOAD — синтаксически невалидный JSON или unknown cmd/frame.
    "BAD_PAYLOAD",
    # TOPIC_UNKNOWN — SUBSCRIBE на topic, которого нет в STREAMS.
    "TOPIC_UNKNOWN",
    # RATE_LIMIT — старый код; сервер НЕ шлёт его (нет реализации),
    # оставлен в декларации для обратной совместимости клиентов.
    # Решение по реализации — отдельная карточка (ADR-0080 §7 вопрос 2).
    "RATE_LIMIT",
    # PROTOCOL_VERSION — subprotocol mismatch (AV-16, §11).
    "PROTOCOL_VERSION",
    # FLOOR_HELD — запрашиваемый floor уже держит другой client_id.
    "FLOOR_HELD",
    # MODE_CONFLICT — FSM супервизора отклонила смену режима (Phase 2).
    "MODE_CONFLICT",
    # INTERNAL — необработанное исключение в server-side handler'е.
    "INTERNAL",
    # UNKNOWN_COMMAND — клиент прислал JSON_CMD с неизвестным ``cmd``
    # (issue #2194, voice-vr 09). Раньше сервер молча ронял хвост функции
    # без ERROR/WARNING; теперь — единый код для всех неизвестных команд.
    # Терминальная ветка dispatcher-таблицы (issue #2195, voice-vr 10)
    # тоже возвращает этот код вместо молчаливого drop.
    "UNKNOWN_COMMAND",
)


@dataclass(frozen=True)
class ErrorCodeSpec:
    """Метаданные одного кода ошибки."""

    code: str
    description: str
    server_emitted: bool = True
    implemented: bool = True


ERROR_SPECS: tuple[ErrorCodeSpec, ...] = (
    ErrorCodeSpec("AUTH_FAIL", "Неверный PIN или сессия не аутентифицирована (§8)."),
    ErrorCodeSpec(
        "BAD_PAYLOAD",
        "Синтаксически невалидный JSON, unknown cmd/frame, или "
        "неподдерживаемое значение поля (§8).",
    ),
    ErrorCodeSpec(
        "TOPIC_UNKNOWN",
        "SUBSCRIBE на topic, которого нет в STREAMS (§8).",
    ),
    ErrorCodeSpec(
        "RATE_LIMIT",
        "Превышен rate-limit (legacy §8).",
        server_emitted=False,  # исторически задекларирован, но НЕ шлётся
        implemented=False,     # отложено (ADR-0080 §7 вопрос 2)
    ),
    ErrorCodeSpec(
        "PROTOCOL_VERSION",
        "Subprotocol mismatch (v1 ↔ v2, AV-16/§11).",
    ),
    ErrorCodeSpec(
        "FLOOR_HELD",
        "Запрашиваемый floor держит другой client_id (ADR-0028 §4.2/§8).",
    ),
    ErrorCodeSpec(
        "MODE_CONFLICT",
        "FSM супервизора отклонила смену режима (ADR-0028 §4.1/§8).",
    ),
    ErrorCodeSpec(
        "INTERNAL",
        "Необработанное исключение в server-side handler'е (§8).",
    ),
    ErrorCodeSpec(
        "UNKNOWN_COMMAND",
        "JSON_CMD с неизвестным ``cmd`` (issue #2194, voice-vr 09 §8). "
        "Терминальная ветка dispatcher-таблицы (issue #2195, voice-vr 10) "
        "тоже возвращает этот код вместо молчаливого drop.",
    ),
)


# === AV-28 §P7: voice presets and languages ================================


# Синхронизирован с ``src/rob_box_voice/config/voice_presets.yaml`` и
# meta-quest-api.md §P7. Расширение пресетов = правка YAML; voice-vr 21
# / ADR-0080 §2.7 — Python-тупла ниже служит FALLBACK'ом на случай
# недоступности yaml (CI-минимум без зависимости на пакет rob_box_voice),
# а живой список загружается из yaml через ``_load_voice_lists_from_yaml``
# ниже. Тот же приём для VOICE_LANGUAGES.
_VOICE_PRESET_IDS_FALLBACK: tuple[str, ...] = (
    "technical",
    "street",
    "caveman",
    "business",
    "philosopher",
    "lenin",
    # Нейтральный пресет: не стилизует, только чистит оговорки
    # и переводит на выбранный язык.
    "translate",
)

# Языки вывода LLM-формализатора (аудит панели пайплайна, AV-28 §P7).
# Ключи languages: в voice_presets.yaml — источник истины.
_VOICE_LANGUAGES_FALLBACK: tuple[str, ...] = ("ru", "en", "fr", "de", "zh", "hi")


def _resolve_voice_presets_yaml_path() -> Optional[str]:
    """Найти ``voice_presets.yaml`` пакета ``rob_box_voice``.

    Порядок (одинаков с :func:`rob_box_supervisor.grip_pipeline.resolve_voice_presets_path`,
    чтобы константор и grip-пайплайн читали один файл):

      0. env ``ROB_BOX_VOICE_PRESETS_YAML`` — override для тестов /
         staging (если задан, используется как есть, без проверки
         существования через ``isfile``);
      1. ament share ``rob_box_voice/config/voice_presets.yaml`` (робот);
      2. source-tree ``<repo>/src/rob_box_voice/config/voice_presets.yaml``
         (colcon symlink / unit-тесты).

    ``None`` — файл нигде не найден → останется FALLBACK.
    """
    env_override = os.environ.get("ROB_BOX_VOICE_PRESETS_YAML")
    if env_override:
        return env_override
    try:
        from ament_index_python.packages import (  # noqa: PLC0415
            get_package_share_directory,
        )

        share = os.path.join(
            get_package_share_directory("rob_box_voice"),
            "config",
            "voice_presets.yaml",
        )
        if os.path.isfile(share):
            return share
    except Exception:  # noqa: BLE001 — нет ament (CI / unit-тесты / не-colcon)
        pass
    # Source-tree fallback. bridge_protocol живёт в
    # ``<repo>/src/rob_box_core/rob_box_core/bridge_protocol.py``,
    # поэтому идём через parents[3] (== ``src/``) + rob_box_voice/config/.
    source = (
        Path(__file__).resolve().parents[3]
        / "rob_box_voice"
        / "config"
        / "voice_presets.yaml"
    )
    return str(source) if source.is_file() else None


def _load_voice_lists_from_yaml() -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Прочитать ключи ``presets`` и ``languages`` из voice_presets.yaml.

    Возвращает ``(preset_ids, language_ids)``. На любой ошибке (нет
    файла, битый yaml, не-dict, отсутствует секция) — возвращает
    FALLBACK-туплы. Без side-effects (warning не пишем — модуль импортится
    часто, и шум в conftest не нужен; регрессию ловит DoD-тест
    ``TestVoiceConfigYamlDrivesWhitelist``).
    """
    path = _resolve_voice_presets_yaml_path()
    if path is None:
        return _VOICE_PRESET_IDS_FALLBACK, _VOICE_LANGUAGES_FALLBACK
    try:
        import yaml  # noqa: PLC0415 — лениво: bridge_protocol импортится без yaml

        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception:  # noqa: BLE001
        return _VOICE_PRESET_IDS_FALLBACK, _VOICE_LANGUAGES_FALLBACK
    if not isinstance(data, dict):
        return _VOICE_PRESET_IDS_FALLBACK, _VOICE_LANGUAGES_FALLBACK
    raw_presets = data.get("presets")
    if isinstance(raw_presets, dict) and raw_presets:
        preset_ids = tuple(str(k) for k in raw_presets.keys())
    else:
        preset_ids = _VOICE_PRESET_IDS_FALLBACK
    raw_languages = data.get("languages")
    if isinstance(raw_languages, dict) and raw_languages:
        language_ids = tuple(str(k).lower() for k in raw_languages.keys())
    elif isinstance(raw_languages, list) and raw_languages:
        # Совместимость со старым списочным форматом (см. коммент в yaml).
        language_ids = tuple(str(x).lower() for x in raw_languages)
    else:
        language_ids = _VOICE_LANGUAGES_FALLBACK
    return preset_ids, language_ids


# Финальные публичные константы: при импорте модуля пытаемся
# прочитать voice_presets.yaml; на неудаче — FALLBACK. Это и есть
# ADR-0080 §2.7 «один белый список»: yaml → константор → ws_server.
# (Python-тупла перестала быть SoT — она страховка на случай CI без yaml.)
VOICE_PRESET_IDS: tuple[str, ...]
VOICE_LANGUAGES: tuple[str, ...]
_VOICE_PRESET_IDS_RESOLVED, _VOICE_LANGUAGES_RESOLVED = _load_voice_lists_from_yaml()
VOICE_PRESET_IDS = _VOICE_PRESET_IDS_RESOLVED
VOICE_LANGUAGES = _VOICE_LANGUAGES_RESOLVED
del _VOICE_PRESET_IDS_RESOLVED, _VOICE_LANGUAGES_RESOLVED


# Дефолтный язык grip-пайплайна до первой синхронизации с панели
# (должен совпадать с GRIP_DEFAULT_LANGUAGE в supervisor_node.py:362 —
# иначе оптимистичная подсветка клиента разъедется с тем, что реально
# применилось).
VOICE_PIPELINE_DEFAULT_LANGUAGE: str = "ru"


# === Voice input modes (ADR-0027 §3.4) ======================================


class VoiceInputMode(str, Enum):
    """Параметр ``voice_input_mode`` на ``dialogue_node``.

    Значения совпадают с ADR-0027 §3.4 + AV-22 (issue #1914, режим
    ``quest_command``). Сервер валидирует через
    ``VOICE_WIRE_MODES`` при приёме ``voice_mode`` cmd.
    """

    OFF = "off"
    RESPEAKER = "respeaker"
    PASSTHROUGH = "passthrough"
    TTTS_PROXY = "ttts_proxy"
    STT_LLM = "stt_llm"
    LLM_FORMALIZE = "llm_formalize"
    QUEST_COMMAND = "quest_command"


# Подмножество, которое клиент может прислать в ``voice_mode`` cmd
# (Phase 2 уже на сервере, см. ws_server.py:1847). Внутренний
# ``respeaker`` клиент НЕ ставит через wire — это default dialogue_node.
VOICE_WIRE_MODES: tuple[str, ...] = (
    "off",
    "passthrough",
    "ttts_proxy",
    "stt_llm",
    "llm_formalize",
)


# === Lookups ================================================================
#
# Поисковые структуры, построенные из «сырых» объявлений выше.
# Используются в runtime + в conformance-тестах.


_COMMANDS_BY_NAME: Mapping[str, CommandSpec] = MappingProxyType(
    {c.name: c for c in COMMANDS}
)
_EVENTS_BY_NAME: Mapping[str, EventSpec] = MappingProxyType(
    {e.name: e for e in EVENTS}
)
_STREAMS_BY_UI_NAME: Mapping[str, StreamSpec] = MappingProxyType(
    {s.ui_name: s for s in STREAMS}
)
_STREAMS_BY_TOPIC_ID: Mapping[int, StreamSpec] = MappingProxyType(
    {s.topic_id: s for s in STREAMS}
)
_ERRORS_SET: frozenset[str] = frozenset(ERRORS)


# Public read-only dict-views над :data:`STREAMS` (для backward-compat
# с кодом, который импортировал ``STREAM_CATALOG`` / ``TOPIC_IDS`` из
# ``rob_box_quest.streams.registry`` / ``rob_box_quest.protocol.topics``;
# voice-vr 07 / issue #2192 — re-export, см. обёртки там).
STREAM_CATALOG: Mapping[str, StreamSpec] = _STREAMS_BY_UI_NAME
TOPIC_IDS: Mapping[str, int] = MappingProxyType(
    {s.ui_name: s.topic_id for s in STREAMS}
)


def get_command(name: str) -> CommandSpec | None:
    """Lookup команды по имени (``None`` если нет в каталоге)."""
    return _COMMANDS_BY_NAME.get(name)


def get_event(name: str) -> EventSpec | None:
    """Lookup события по имени (``None`` если нет в каталоге)."""
    return _EVENTS_BY_NAME.get(name)


def get_stream(ui_name: str) -> StreamSpec | None:
    """Lookup стрима по UI-имени (``None`` если нет в каталоге)."""
    return _STREAMS_BY_UI_NAME.get(ui_name)


def get_stream_by_topic_id(topic_id: int) -> StreamSpec | None:
    """Lookup стрима по topic_id (uint32)."""
    return _STREAMS_BY_TOPIC_ID.get(topic_id)


def is_known_error(code: str) -> bool:
    """True если ``code`` присутствует в :data:`ERRORS_AS_SET`."""
    return code in _ERRORS_SET


def client_dispatched_commands() -> tuple[str, ...]:
    """Команды, которые клиент РЕАЛЬНО шлёт (= диспатчатся сервером).

    Возвращает имена в порядке объявления в :data:`COMMANDS`.
    Используется в conformance-тестах вместо grep по ``ws_server.py``.
    """
    return tuple(c.name for c in COMMANDS if c.server_dispatched)


def deprecated_commands() -> tuple[str, ...]:
    """Команды с ``server_dispatched=False`` (TS-типы есть, сервер игнорирует).

    Используется для отчёта и для отслеживания «что ещё не закрыто»
    (voice-vr 09 — про ``avatar_*``; ``ui_button``/``set_panel_topic``/
    ``admin_logs*`` — Phase 2).
    """
    return tuple(c.name for c in COMMANDS if not c.server_dispatched)


__all__ = (
    "FrameTypeId",
    "CommandSpec",
    "EventSpec",
    "StreamKind",
    "StreamSpec",
    "ErrorCodeSpec",
    "Mode",
    "Floor",
    "VoiceInputMode",
    "COMMANDS",
    "EVENTS",
    "STREAMS",
    "STREAM_CATALOG",
    "MODES",
    "FLOORS",
    "ERRORS",
    "ERROR_SPECS",
    "VOICE_PRESET_IDS",
    "VOICE_LANGUAGES",
    "VOICE_PIPELINE_DEFAULT_LANGUAGE",
    "VOICE_WIRE_MODES",
    "TOPIC_IDS",
    "get_command",
    "get_event",
    "get_stream",
    "get_stream_by_topic_id",
    "is_known_error",
    "client_dispatched_commands",
    "deprecated_commands",
)
