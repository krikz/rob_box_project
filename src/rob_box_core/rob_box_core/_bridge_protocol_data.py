"""Bridge protocol catalog — GENERATED, do not edit by hand.

Source of truth for the **wire contract** between the WebXR / REST clients
and the rob_box_quest server (and from there the avatar_supervisor for
``v2`` subprotocol frames). One declaration here is read by:

* ``tools/gen_bridge_protocol_ts.py`` to emit
  ``src/rob_box_quest/webxr_client/src/wire/protocol_generated.ts``
  (TS types + runtime string-literal unions) — guarded in CI by
  ``.github/workflows/G-Bridge-Protocol-Drift.yml``.
* The future ``rob_box_core.bridge_protocol`` typed-access wrapper
  (``voice-vr 07``) — same data, frozen dataclasses over this tuple.
* The conformance test from ``voice-vr 02`` — every JSON_CMD handler in
  ``ws_server._on_json_cmd`` must be present here, and vice versa.

What lives here vs hand-written files
-------------------------------------

| Layer                  | Source                                                           |
|------------------------|------------------------------------------------------------------|
| Frame byte codes       | ``protocol/frame.py:FrameType`` (codec) — *not* duplicated here. |
| Topic IDs & payloads   | ``protocol/topics.py`` (codec) — *not* duplicated here.          |
| **JSON_CMD names**     | **here** ``COMMANDS[].name`` (drives client+server dispatch).    |
| **JSON_CMD payloads**  | **here** ``COMMANDS[].payload`` (drives TS ``XxxCmd`` interfaces).|
| **JSON_EVENT names**   | **here** ``EVENTS[].name`` (server-side dispatcher + client UI). |
| **JSON_EVENT payloads**| **here** ``EVENTS[].payload`` (drives TS ``JsonEvent`` union).   |
| **Stream catalog**     | **here** ``STREAMS[]`` (drives stream picker; topic_ids match    |
|                        | ``protocol/topics.py`` for the 8 wire-stream IDs, with extra    |
|                        | derived streams registered only in quest-side ``streams/``).    |
| **Error codes**        | **here** ``ERRORS[]`` (single source; was duplicated in         |
|                        | ``server/session.py`` before, see ``voice-vr 07``).              |
| **Supervisor modes**   | **here** ``MODES[]`` (``off`` / ``avatar_present`` / ``mixed``). |
| **Floor names**        | **here** ``FLOORS[]`` (``teleop`` / ``voice``).                  |
| **Voice presets/lang** | **here** ``VOICE_PRESETS[]`` / ``VOICE_LANGUAGES[]`` (whitelist, |
|                        | must match ws_server.VOICE_PRESET_IDS / VOICE_LANGUAGES).       |

Schema convention for ``payload`` values
----------------------------------------

A payload value is **always** one of:

* A string type (the TS generator emits these verbatim):

      "str"      -> ``string``
      "int"      -> ``number``
      "float"    -> ``number``
      "bool"     -> ``boolean``
      "unknown"  -> ``unknown``

* A composite type:

      "list[int]"           -> ``Array<number>``
      "list[str]"           -> ``Array<string>``
      "list[Xxx]"           -> ``Array<Xxx>`` (Xxx must be a top-level TS interface)
      "Record<string, X>"   -> ``Record<string, X>``

* A trailing ``?`` marks the field optional:

      "str?"          -> ``field?: string``
      "list[int]?"    -> ``field?: Array<number>``

* A string-literal union (rendered as a TS ``type`` alias):

      {"type": "literal", "values": ["a", "b", "c"]}              -> ``"a" | "b" | "c"``
      {"type": "literal", "values": ["a"], "optional": True}      -> ``field?: "a"``

  The first value of a literal is always the discriminant (``cmd``/``type``)
  in this catalog; the generator reuses it as the TS-discriminant hint.

Regenerate the TS mirror with::

    python tools/gen_bridge_protocol_ts.py

Detect drift in CI with::

    python tools/gen_bridge_protocol_ts.py --check

History: before this existed, the wire contract was hand-written in three
places — ``src/rob_box_quest/rob_box_quest/protocol/topics.py`` (8
stream IDs, hand-rolled), ``server/session.py:28-43`` (``ErrorCode``
with duplicates for ``FLOOR_HELD``/``MODE_CONFLICT`` and an unreachable
``RATE_LIMIT``), and ``src/rob_box_quest/webxr_client/src/wire/messages.ts``
(321 hand-rolled TS interfaces; already drifted — server announces
``voice_pipeline``, ``voice_listen_start/stop``, ``ping`` that the
client never declared). This file is the convergence point — see
``voice-vr 07`` and ``ADR-0080`` §2.2 for the rationale.
"""

from __future__ import annotations

from typing import Any

# ---------------------------------------------------------------------------
# Subprotocol matrix (meta-quest-api.md §11.1)
# ---------------------------------------------------------------------------

#: Subprotocols the server advertises in the WS handshake. Order matters:
#: aiohttp picks the first match from the client's offered list. ``v2``
#: MUST be first so v2-capable clients (preferred in Phase 2) win
#: negotiation; ``v1`` stays for the read-only ``monitor`` rollout mode
#: (ADR-0028 §4.5).
SUBPROTOCOLS: tuple[str, ...] = (
    "robbox-quest-v2",
    "robbox-quest-v1",
)

# ---------------------------------------------------------------------------
# JSON_CMD catalog — client → server (meta-quest-api.md §5 + §5.1)
# ---------------------------------------------------------------------------

#: One entry per ``cmd`` value. ``payload`` is the *full* JSON shape with
#: the schema convention described in the module docstring. The generator
#: emits one ``interface XxxCmd`` per entry. ``subprotocol`` defaults to
#: ``"v1"``; supervisor-only commands (``supervisor_*``) are tagged
#: ``"v2"``. ``rate_limit`` is the per-row rate from meta-quest-api.md §9
#: (``None`` = no documented limit, server-side token bucket unenforced).
#:
#: Order matters for the deterministic generator output: commands are
#: rendered in the same order they appear here.
COMMANDS_DATA: tuple[dict[str, Any], ...] = (
    {
        "name": "admin_logs",
        "subprotocol": "v1",
        "rate_limit": None,
        "description": "Phase 2 R14. Сервер читает docker logs / journald и шлёт admin_logs_chunk.",
        "payload": {
            "cmd": "admin_logs",
            "ts_ms": "int",
            "service": {"type": "literal", "values": ["dialogue_node", "rob_box_quest", "all"]},
            "tail": "int?",
            "follow": "bool?",
        },
    },
    {
        "name": "admin_logs_stop",
        "subprotocol": "v1",
        "rate_limit": None,
        "description": "Останавливает follow-стриминг admin_logs.",
        "payload": {
            "cmd": "admin_logs_stop",
            "ts_ms": "int",
        },
    },
    {
        "name": "list_voices",
        "subprotocol": "v1",
        "rate_limit": "1 per 10s",
        "description": "Phase 2 §4.1. Запрос списка доступных голосов → JSON_EVENT voice_list.",
        "payload": {
            "cmd": "list_voices",
            "ts_ms": "int",
        },
    },
    {
        "name": "ping",
        "subprotocol": "v1",
        "rate_limit": "1 per 5s",
        "description": "Latency probe (§7). Клиент шлёт JSON_EVENT ping (см. EVENTS), JSON_CMD ping — "
                       "отступление от контракта, но ws_server принимает оба.",
        "payload": {
            "cmd": "ping",
            "ts_ms": "int",
            "nonce": "str?",
        },
    },
    {
        "name": "preview_voice",
        "subprotocol": "v1",
        "rate_limit": "1 per 5s, max 3 concurrent",
        "description": "Phase 2 §4.2. Синтез фразы голосом → BINARY_FRAME + preview_voice_audio/_done/_error.",
        "payload": {
            "cmd": "preview_voice",
            "ts_ms": "int",
            "voice_id": "str",
            "text": "str",
            "request_id": "str",
        },
    },
    {
        "name": "set_panel_topic",
        "subprotocol": "v1",
        "rate_limit": "5 Hz per panel",
        "description": "Phase 2 §6.2. Меняет топик, рендеримый в данной panel.",
        "payload": {
            "cmd": "set_panel_topic",
            "ts_ms": "int",
            "panel_id": "str",
            "topic": "str",
        },
    },
    {
        "name": "set_voice",
        "subprotocol": "v1",
        "rate_limit": "voice_id 1/2s; preset/language 1/0.5s",
        "description": "Phase 2 §4.3 + AV-28 §P7. Меняет голос/preset/language TTS-пайплайна.",
        "payload": {
            "cmd": "set_voice",
            "ts_ms": "int",
            "voice_id": "str",
            "preset": {"type": "literal",
                       "values": ["standard", "friendly", "authoritative", "whisper",
                                  "technical", "street", "caveman", "business",
                                  "philosopher", "lenin"],
                       "optional": True},
            "language": {"type": "literal", "values": ["ru", "en"], "optional": True},
        },
    },
    {
        "name": "stop_emergency",
        "subprotocol": "v1",
        "rate_limit": "1 per 100ms",
        "description": "Публикует в /safety/emergency_stop. Всегда в обход teleop_floor-гейта (AV-19).",
        "payload": {
            "cmd": "stop_emergency",
            "ts_ms": "int",
            "source": {"type": "literal", "values": ["controller_b", "ui_button", "client_lost"]},
        },
    },
    {
        "name": "stream_list",
        "subprotocol": "v1",
        "rate_limit": None,
        "description": "Phase 2 R10. Сервер отвечает JSON_EVENT stream_list со списком доступных топиков.",
        "payload": {
            "cmd": "stream_list",
            "ts_ms": "int",
        },
    },
    {
        "name": "stream_select",
        "subprotocol": "v1",
        "rate_limit": None,
        "description": "Phase 2 §6.2 (companion to set_panel_topic).",
        "payload": {
            "cmd": "stream_select",
            "ts_ms": "int",
            "topic": "str",
        },
    },
    {
        "name": "supervisor_acquire_floor",
        "subprotocol": "v2",
        "rate_limit": None,
        "description": "§5.1 — JSON-эквивалент 0x31 ACQUIRE_FLOOR (msgpack). v1 сессия → ERROR PROTOCOL_VERSION.",
        "payload": {
            "cmd": "supervisor_acquire_floor",
            "ts_ms": "int",
            "seq": "int?",
            "client_id": "str",
            "floor": {"type": "literal", "values": ["teleop", "voice"]},
        },
    },
    {
        "name": "supervisor_get_state",
        "subprotocol": "v2",
        "rate_limit": None,
        "description": "§5.1 — poll-эквивалент 0x33 STATE_UPDATE.",
        "payload": {
            "cmd": "supervisor_get_state",
            "ts_ms": "int",
        },
    },
    {
        "name": "supervisor_release_floor",
        "subprotocol": "v2",
        "rate_limit": None,
        "description": "§5.1 — JSON-эквивалент 0x32 RELEASE_FLOOR (msgpack).",
        "payload": {
            "cmd": "supervisor_release_floor",
            "ts_ms": "int",
            "seq": "int?",
            "client_id": "str",
            "floor": {"type": "literal", "values": ["teleop", "voice"]},
        },
    },
    {
        "name": "supervisor_set_mode",
        "subprotocol": "v2",
        "rate_limit": None,
        "description": "§5.1 — JSON-эквивалент 0x30 SET_MODE (msgpack).",
        "payload": {
            "cmd": "supervisor_set_mode",
            "ts_ms": "int",
            "seq": "int?",
            "client_id": "str",
            "mode": {"type": "literal",
                     "values": ["off", "telegram_active", "avatar_present", "mixed",
                                "teleop_only", "voice_only"]},
        },
    },
    {
        "name": "teleop_heartbeat",
        "subprotocol": "v1",
        "rate_limit": "10 Hz",
        "description": "AV-19 / ADR-0028 §4.4 S10. Шлётся клиентом 10 Гц пока ARM + teleop_floor.",
        "payload": {
            "cmd": "teleop_heartbeat",
            "ts_ms": "int",
            "seq": "int",
        },
    },
    {
        "name": "teleop_twist",
        "subprotocol": "v1",
        "rate_limit": "30 Hz",
        "description": "Twist-команда (Twist.linear/angular + deadman). AV-19: при require_teleop_floor=true "
                       "без своего floor → ERROR FLOOR_HELD ≤1 Гц.",
        "payload": {
            "cmd": "teleop_twist",
            "ts_ms": "int",
            "seq": "int",
            "linear": {"x": "float", "y": "float", "z": "float"},
            "angular": {"x": "float", "y": "float", "z": "float"},
            "deadman": "bool",
        },
    },
    {
        "name": "ui_button",
        "subprotocol": "v1",
        "rate_limit": "5 Hz",
        "description": "Phase 1/2. Кнопка UI → ROS-сервис через registry (phase-2).",
        "payload": {
            "cmd": "ui_button",
            "ts_ms": "int",
            "button": "str",
            "press": "bool",
        },
    },
    {
        "name": "voice_listen_start",
        "subprotocol": "v1",
        "rate_limit": "edge-triggered, max 2 start/s",
        "description": "Включает постоянное прослушивание микрофона шлема (для wake-gate). "
                       "Раньше был частью voice_ptt_start; вынесен отдельно (voice-vr 09).",
        "payload": {
            "cmd": "voice_listen_start",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_listen_stop",
        "subprotocol": "v1",
        "rate_limit": "edge-triggered",
        "description": "Выключает прослушивание микрофона шлема (voice-vr 09).",
        "payload": {
            "cmd": "voice_listen_stop",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_mode",
        "subprotocol": "v1",
        "rate_limit": "1 per 5s",
        "description": "Переключает voice_input_mode в dialogue_node (ADR-0028 §5).",
        "payload": {
            "cmd": "voice_mode",
            "ts_ms": "int",
            "mode": {"type": "literal",
                     "values": ["off", "passthrough", "ttts_proxy", "stt_llm", "llm_formalize"]},
        },
    },
    {
        "name": "voice_pipeline",
        "subprotocol": "v1",
        "rate_limit": "per-call whitelist",
        "description": "Конфиг пайплайна грипа (issue #1989) → /avatar/voice_pipeline. "
                       "Whitelist (_validate_voice_pipeline_payload): preset ∈ VOICE_PRESET_IDS, "
                       "language ∈ {ru, en, fr, de, zh, hi}, llm_enabled ∈ {true,false}.",
        "payload": {
            "cmd": "voice_pipeline",
            "ts_ms": "int",
            "llm_enabled": "bool",
            "preset": "str",
            "language": {"type": "literal", "values": ["ru", "en", "fr", "de", "zh", "hi"]},
        },
    },
    {
        "name": "voice_ptt_start",
        "subprotocol": "v1",
        "rate_limit": "edge-triggered, max 2 start/s",
        "description": "Phase 2.1+ (§11.3). Заменяет старый voice_ptt{state:start|stop}. mode определяет "
                       "маршрут аудио (radio=passthrough, robot_voice=STT→LLM→TTS).",
        "payload": {
            "cmd": "voice_ptt_start",
            "ts_ms": "int",
            "mode": {"type": "literal", "values": ["radio", "robot_voice"], "optional": True},
        },
    },
    {
        "name": "voice_ptt_stop",
        "subprotocol": "v1",
        "rate_limit": "edge-triggered",
        "description": "§11.3. Edge-triggered stop. mode обязан совпадать с voice_ptt_start.",
        "payload": {
            "cmd": "voice_ptt_stop",
            "ts_ms": "int",
            "mode": {"type": "literal", "values": ["radio", "robot_voice"], "optional": True},
        },
    },
)


# ---------------------------------------------------------------------------
# JSON_EVENT catalog — server → client (meta-quest-api.md §6 + §5.1, §7)
# ---------------------------------------------------------------------------

EVENTS_DATA: tuple[dict[str, Any], ...] = (
    {
        "name": "admin_logs_chunk",
        "subprotocol": "v1",
        "description": "Чанк docker logs / journald в ответ на admin_logs (follow=true).",
        "payload": {
            "type": "admin_logs_chunk",
            "service": {"type": "literal", "values": ["dialogue_node", "rob_box_quest", "all"]},
            "lines": "list[str]",
            "ts_ms": "int",
        },
    },
    {
        "name": "admin_logs_end",
        "subprotocol": "v1",
        "description": "Финал стриминга admin_logs.",
        "payload": {
            "type": "admin_logs_end",
            "service": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "avatar_state_ack",
        "subprotocol": "v1",
        "description": "Phase 2. Ответ на supervisor_get_state — текущий AvatarState.",
        "payload": {
            "type": "avatar_state_ack",
            "state": "Record<string, unknown>",
            "ts_ms": "int",
        },
    },
    {
        "name": "avatar_state_nack",
        "subprotocol": "v1",
        "description": "Phase 2. Ошибка получения state.",
        "payload": {
            "type": "avatar_state_nack",
            "reason": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "floor_lost",
        "subprotocol": "v1",
        "description": "AV-19. Сервер сообщает, что наша сессия больше не держит teleop_floor. "
                       "Клиент обязан DISARM-нуться (teleop_fsm.setHasFloor(false)).",
        "payload": {
            "type": "floor_lost",
            "floor": {"type": "literal", "values": ["teleop", "voice"]},
            "reason": "str?",
            "ts_ms": "int",
        },
    },
    {
        "name": "heartbeat",
        "subprotocol": "v1",
        "description": "Server → client heartbeat каждые 200 мс (§7). 3 пропуска → CONNECTION LOST.",
        "payload": {
            "type": "heartbeat",
            "ts_ms": "int",
        },
    },
    {
        "name": "operator_tts_audio",
        "subprotocol": "v1",
        "description": "ADR-0055 / issue #1993 — обратный канал ТАРС в шлем через BINARY_FRAME + JSON_EVENT.",
        "payload": {
            "type": "operator_tts_audio",
            "request_id": "str",
            "format": {"type": "literal", "values": ["pcm_s16le", "mp3", "opus", "wav"]},
            "content_type": "str",
            "seq": "int",
            "total": "int",
            "ts_ms": "int",
        },
    },
    {
        "name": "operator_tts_done",
        "subprotocol": "v1",
        "description": "ADR-0055 — финал обратного канала.",
        "payload": {
            "type": "operator_tts_done",
            "request_id": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "operator_tts_error",
        "subprotocol": "v1",
        "description": "ADR-0055 — ошибка обратного канала.",
        "payload": {
            "type": "operator_tts_error",
            "request_id": "str",
            "reason": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "ping",
        "subprotocol": "v1",
        "description": "Latency probe (§7). Сервер шлёт в ответ на JSON_EVENT ping клиента. "
                       "JSON_CMD ping (см. COMMANDS) — отступление от контракта.",
        "payload": {
            "type": "ping",
            "ts_ms": "int",
            "nonce": "str?",
        },
    },
    {
        "name": "pong",
        "subprotocol": "v1",
        "description": "Ответ сервера на ping (§7).",
        "payload": {
            "type": "pong",
            "ts_ms": "int",
            "nonce": "str?",
        },
    },
    {
        "name": "preview_voice_audio",
        "subprotocol": "v1",
        "description": "Phase 2 §4.2. Первый чанк preview_voice (BINARY_FRAME несёт сам звук).",
        "payload": {
            "type": "preview_voice_audio",
            "request_id": "str",
            "format": {"type": "literal", "values": ["mp3", "opus", "wav"]},
            "content_type": "str",
            "seq": "int",
            "total": "int",
            "ts_ms": "int",
        },
    },
    {
        "name": "preview_voice_done",
        "subprotocol": "v1",
        "description": "Phase 2 §4.2. Финал preview_voice.",
        "payload": {
            "type": "preview_voice_done",
            "request_id": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "preview_voice_error",
        "subprotocol": "v1",
        "description": "Phase 2 §4.2. Ошибка preview_voice.",
        "payload": {
            "type": "preview_voice_error",
            "request_id": "str",
            "reason": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "robot_alert",
        "subprotocol": "v1",
        "description": "AV-26 R7 — алёрт на изменении (edge semantics, гистерезис, выдержка 10 с).",
        "payload": {
            "type": "robot_alert",
            "code": "str",
            "level": {"type": "literal", "values": ["warn", "error", "info"]},
            "active": "bool?",
            "args": "Record<string, unknown>?",
            "ts_ms": "int",
        },
    },
    {
        "name": "safety_stop",
        "subprotocol": "v1",
        "description": "Сервер принудительно остановил движение (controller_b или client_lost).",
        "payload": {
            "type": "safety_stop",
            "reason": {"type": "literal", "values": ["controller_b", "client_lost"]},
            "ts_ms": "int",
        },
    },
    {
        "name": "stream_list",
        "subprotocol": "v1",
        "description": "Phase 2 R10. Список доступных топиков в ответ на cmd=stream_list.",
        "payload": {
            "type": "stream_list",
            "topics": "list[str]",
            "ts_ms": "int",
        },
    },
    {
        "name": "stream_select_ack",
        "subprotocol": "v1",
        "description": "Phase 2 §6.2. Подтверждение stream_select: новый (или null) stream_id.",
        "payload": {
            "type": "stream_select_ack",
            "topic": "str",
            "stream_id": "int?",
            "kind": "str?",
        },
    },
    {
        "name": "subscribe_ack",
        "subprotocol": "v1",
        "description": "Подтверждение SUBSCRIBE с назначенным server-side stream_id.",
        "payload": {
            "type": "subscribe_ack",
            "topic": "str",
            "stream_id": "int",
            "quality": "str",
            "kind": "str?",
        },
    },
    {
        "name": "subscribe_nack",
        "subprotocol": "v1",
        "description": "Отказ SUBSCRIBE (TOPIC_UNKNOWN / TOPIC_NOT_AVAILABLE_YET / etc).",
        "payload": {
            "type": "subscribe_nack",
            "topic": "str",
            "reason": "str",
        },
    },
    {
        "name": "supervisor_state",
        "subprotocol": "v2",
        "description": "§5.1 / §11.2. Ответ на supervisor_get_state (poll) либо broadcast.",
        "payload": {
            "type": "supervisor_state",
            "state": "Record<string, unknown>",
            "ts_ms": "int",
        },
    },
    {
        "name": "tars1_text",
        "subprotocol": "v1",
        "description": "issue #2113 / #2112 — Captain Bridge. TARS 1 text panel echo.",
        "payload": {
            "type": "tars1_text",
            "request_id": "str",
            "text": "str",
            "streaming": "bool",
            "done": "bool",
            "ts_ms": "int",
        },
    },
    {
        "name": "tars_panel_url",
        "subprotocol": "v1",
        "description": "issue #2113 — TARS 2 metrics panel: URL Grafana от avatar_supervisor.",
        "payload": {
            "type": "tars_panel_url",
            "request_id": "str",
            "url": "str",
            "status": "str",
            "error": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_list",
        "subprotocol": "v1",
        "description": "Phase 2 §4.1. Список VoiceInfo в ответ на cmd=list_voices. AV-27 добавил provider.",
        "payload": {
            "type": "voice_list",
            "voices": "list[VoiceInfo]",
            "active_provider": "str?",
            "active_voice": "str?",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_mode_ack",
        "subprotocol": "v1",
        "description": "Подтверждение voice_mode.",
        "payload": {
            "type": "voice_mode_ack",
            "mode": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_pipeline_ack",
        "subprotocol": "v1",
        "description": "Подтверждение voice_pipeline.",
        "payload": {
            "type": "voice_pipeline_ack",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_pipeline_nack",
        "subprotocol": "v1",
        "description": "Отказ voice_pipeline (rate_limited / invalid_preset / invalid_language / unknown_field).",
        "payload": {
            "type": "voice_pipeline_nack",
            "reason": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_presets",
        "subprotocol": "v1",
        "description": "Список пресетов/языков, доступных dialogue_node (boot-time UI seed).",
        "payload": {
            "type": "voice_presets",
            "presets": "list[VoicePresetInfo]",
            "languages": "list[str]",
            "default_preset": "str",
            "default_language": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_set_ack",
        "subprotocol": "v1",
        "description": "Подтверждение set_voice.",
        "payload": {
            "type": "voice_set_ack",
            "voice_id": "str",
            "preset": "str",
            "language": "str",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_set_nack",
        "subprotocol": "v1",
        "description": "Отказ set_voice (voice_unavailable / rate_limited / invalid_*). AV-27: available[] подсказка.",
        "payload": {
            "type": "voice_set_nack",
            "voice_id": "str?",
            "preset": "str?",
            "language": "str?",
            "reason": "str",
            "available": "list[str]?",
            "ts_ms": "int",
        },
    },
    {
        "name": "voice_state",
        "subprotocol": "v1",
        "description": "§6 — состояние голосового канала робота.",
        "payload": {
            "type": "voice_state",
            "state": {"type": "literal",
                      "values": ["idle", "listening", "thinking", "speaking", "denied"]},
            "ts_ms": "int",
            "utterance_id": "str?",
            "holder_id": "str?",
            "detail": "str?",
        },
    },
)


# ---------------------------------------------------------------------------
# HELLO / WELCOME / SUBSCRIBE / UNSUBSCRIBE / GOODBYE (control frames)
# ---------------------------------------------------------------------------
#
# These don't fit cleanly into COMMANDS / EVENTS — they have their own
# JSON shape on the wire and aren't dispatched through _on_json_cmd.
# Hand-rolled interfaces live in messages.ts; the catalog carries only
# the minimum the conformance test needs (which control frames exist).

CONTROL_FRAMES_DATA: tuple[dict[str, Any], ...] = (
    {
        "name": "HELLO",
        "direction": "client→server",
        "description": "Initial handshake from client (meta-quest-api.md §3, 0x01).",
        "payload": {
            "client_version": "str",
            "capabilities": "list[str]",
            "session_pin": "str",
        },
    },
    {
        "name": "WELCOME",
        "direction": "server→client",
        "description": "Server ack of HELLO (0x02). teleop_floor_held_by добавлен AV-19.",
        "payload": {
            "server_version": "str",
            "session_id": "str",
            "server_time_ms": "int",
            "robot_status": "Record<string, unknown>?",
            "teleop_floor_held_by": "str?",
        },
    },
    {
        "name": "SUBSCRIBE",
        "direction": "client→server",
        "description": "Subscribe to a stream (0x03). topic ∈ STREAMS[].",
        "payload": {
            "topic": "str",
            "quality": "str?",
        },
    },
    {
        "name": "UNSUBSCRIBE",
        "direction": "client→server",
        "description": "Unsubscribe from a stream (0x04).",
        "payload": {
            "topic": "str",
        },
    },
    {
        "name": "GOODBYE",
        "direction": "both",
        "description": "Close handshake (0x20).",
        "payload": {
            "reason": {"type": "literal",
                       "values": ["user_logout", "shutdown", "timeout", "auth_fail"]},
        },
    },
)


# ---------------------------------------------------------------------------
# Stream catalog (meta-quest-api.md §4 — BINARY_FRAME; SUBSCRIBE topics)
# ---------------------------------------------------------------------------

#: Сервер-инициируемые стримы. ``topic_id`` обязан совпадать с
#: ``protocol/topics.py:TOPIC_IDS`` (codec-уровень); этот каталог несёт
#: описание/назначение/default_quality, которых в codec не было. Не
#: дублируйте topic_id — конвергенция через ``voice-vr 02`` conformance.
STREAMS_DATA: tuple[dict[str, Any], ...] = (
    {"topic": "camera_rear", "topic_id": 0x1001, "kind": "camera_direct",
     "description": "Задняя камера робота, H.264 NAL (Phase 1).",
     "default_quality": "med"},
    {"topic": "camera_front", "topic_id": 0x1002, "kind": "camera_direct",
     "description": "Передняя камера робота, H.264 NAL (Phase 1).",
     "default_quality": "med"},
    {"topic": "lidar_2d", "topic_id": 0x1101, "kind": "ros_topic",
     "description": "2D LaserScan, 360 точек float32 ranges+intensities.",
     "default_quality": "high"},
    {"topic": "lidar_3d", "topic_id": 0x1102, "kind": "ros_topic",
     "description": "3D pointcloud (Phase 3 — bandwidth-limited).",
     "default_quality": "low"},
    {"topic": "map_2d", "topic_id": 0x1103, "kind": "ros_topic",
     "description": "2D occupancy grid (для AR-overlay).",
     "default_quality": "low"},
    {"topic": "robot_status", "topic_id": 0x1201, "kind": "ros_topic",
     "description": "1 Hz статус: battery, wifi, mode, velocity.",
     "default_quality": "high"},
    {"topic": "voice_state", "topic_id": 0x1202, "kind": "ros_topic",
     "description": "Голосовое состояние робота (idle/listening/thinking/speaking/denied).",
     "default_quality": "high"},
    {"topic": "person_detections", "topic_id": 0x1301, "kind": "ros_topic",
     "description": "Phase 2 R11 — детекции людей (OAK-D / YOLO).",
     "default_quality": "med"},
)


# ---------------------------------------------------------------------------
# Error codes (meta-quest-api.md §8) — ERROR-frame payload.code
# ---------------------------------------------------------------------------

ERRORS_DATA: tuple[dict[str, Any], ...] = (
    {"code": "AUTH_FAIL",
     "description": "Неверный PIN.",
     "client_action": "показать «Wrong PIN, ask operator for current one»."},
    {"code": "BAD_PAYLOAD",
     "description": "payload не парсится / не проходит schema.",
     "client_action": "drop frame, лог в UI-console."},
    {"code": "RATE_LIMIT",
     "description": "teleop_twist чаще 30 Hz.",
     "client_action": "drop frame, throttle на клиенте."},
    {"code": "TOPIC_UNKNOWN",
     "description": "subscribe на topic, которого нет в registry.",
     "client_action": "показать «Topic not available»."},
    {"code": "TOPIC_NOT_AVAILABLE_YET",
     "description": "source-данные ещё не пришли (lidar не запущен).",
     "client_action": "автоматический retry через 1 с."},
    {"code": "INTERNAL",
     "description": "неожиданная ошибка сервера.",
     "client_action": "UI показывает «Server error, reconnect»."},
    {"code": "PROTOCOL_VERSION",
     "description": "subprotocol не совпадает с поддерживаемой версией (§11).",
     "client_action": "close socket, UI «Update client»."},
    {"code": "FLOOR_HELD",
     "description": "ACQUIRE_FLOOR / RELEASE_FLOOR — floor занят чужим client_id. "
                    "AV-19: также шлётся при teleop_twist без своего teleop_floor "
                    "(rate-limited ≤ 1 Гц).",
     "client_action": "показать «Floor held by another operator» или тост «возьми руль»."},
    {"code": "MODE_CONFLICT",
     "description": "SET_MODE / supervisor_set_mode — FSM супервизора отклонила переход.",
     "client_action": "показать «Mode not allowed now»."},
)


# ---------------------------------------------------------------------------
# Supervisor FSM modes (ADR-0028 §4.1 + meta-quest-api.md §5.1)
# ---------------------------------------------------------------------------

MODES_DATA: tuple[str, ...] = (
    "off",
    "telegram_active",
    "avatar_present",
    "mixed",
    "teleop_only",
    "voice_only",
)


# ---------------------------------------------------------------------------
# Floors (ADR-0028 §4.2 + meta-quest-api.md §5.1)
# ---------------------------------------------------------------------------

FLOORS_DATA: tuple[str, ...] = (
    "teleop",
    "voice",
)


# ---------------------------------------------------------------------------
# Voice presets (meta-quest-api.md §P7 / AV-27 / voice_presets.yaml)
# ---------------------------------------------------------------------------

#: Legacy пресеты (Phase 1/2.0), плюс AV-28 §P7 стили речи из
#: voice_presets.yaml. Whitelist сервера (ws_server._validate_voice_pipeline_payload
#: → VOICE_PRESET_IDS) должен совпадать с этим списком (conformance).
VOICE_PRESETS_DATA: tuple[str, ...] = (
    # Legacy (Phase 1/2.0)
    "standard",
    "friendly",
    "authoritative",
    "whisper",
    # AV-28 §P7 — стили речи из voice_presets.yaml.
    "technical",
    "street",
    "caveman",
    "business",
    "philosopher",
    "lenin",
)


# ---------------------------------------------------------------------------
# Voice languages (ws_server.VOICE_LANGUAGES whitelist)
# ---------------------------------------------------------------------------

VOICE_LANGUAGES_DATA: tuple[str, ...] = (
    "ru",
    "en",
    "fr",
    "de",
    "zh",
    "hi",
)


# ---------------------------------------------------------------------------
# Quality levels for SUBSCRIBE
# ---------------------------------------------------------------------------

QUALITY_LEVELS_DATA: tuple[str, ...] = (
    "low",
    "med",
    "high",
)


# ---------------------------------------------------------------------------
# Aliases (mirror `_tool_catalog_data.py` style — one tuple per section).
# ---------------------------------------------------------------------------

COMMANDS: tuple[dict[str, Any], ...] = COMMANDS_DATA
EVENTS: tuple[dict[str, Any], ...] = EVENTS_DATA
CONTROL_FRAMES: tuple[dict[str, Any], ...] = CONTROL_FRAMES_DATA
STREAMS: tuple[dict[str, Any], ...] = STREAMS_DATA
ERRORS: tuple[dict[str, Any], ...] = ERRORS_DATA
MODES: tuple[str, ...] = MODES_DATA
FLOORS: tuple[str, ...] = FLOORS_DATA
VOICE_PRESETS: tuple[str, ...] = VOICE_PRESETS_DATA
VOICE_LANGUAGES: tuple[str, ...] = VOICE_LANGUAGES_DATA
QUALITY_LEVELS: tuple[str, ...] = QUALITY_LEVELS_DATA