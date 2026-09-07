# Command.msg — typed union-команда для avatar_supervisor (ADR-0051 §2.1, issue #2002).
#
# ROS 2 msg-синтаксис НЕ поддерживает настоящие enum-union / oneof,
# поэтому используем ``uint8 kind`` + «пустые» поля под каждый вариант.
# По образцу ``robot_sensor_hub_msg/msg/DeviceCommand.msg``.
#
# Каждое новое поле = новая команда; добавлять новые сервисы
# НЕ требуется (Phase 2+). См. docs/architecture/target-operator-agent-and-dialogue.md
# §13 Шаг 12 + ADR-0051 §2.1.
#
# Значения ``kind`` обязаны совпадать с константами в
# ``rob_box_supervisor.supervisor_node`` (KIND_*): фиксируем здесь
# единый источник истины, чтобы и unit-тесты, и реальные сервисные
# ответы матчились без магических литералов.

# ── kind enum ────────────────────────────────────────────────────────
uint8 KIND_UNKNOWN          = 0
uint8 KIND_ACQUIRE_FLOOR    = 1
uint8 KIND_RELEASE_FLOOR    = 2
uint8 KIND_SET_AVATAR_MODE  = 3
uint8 KIND_SET_VOICE_MODE   = 4
uint8 KIND_EMERGENCY_STOP   = 5
uint8 KIND_HEARTBEAT        = 6

uint8 kind

# ── payload (поля не используются вне своего ``kind``) ──────────────
# KIND_ACQUIRE_FLOOR / KIND_RELEASE_FLOOR:
string client_id            # кто отправил (для floor-логики)
string floor                # "teleop" | "voice" | "" для других команд

# KIND_SET_AVATAR_MODE:
string avatar_event         # off | telegram_active | avatar_present | mixed | ...

# KIND_SET_VOICE_MODE (допустимые значения см. ADR-0027 §3.4):
string voice_mode           # respeaker | quest_passthrough | quest_ttts | quest_stt | quest_llm_formalize | off

# KIND_EMERGENCY_STOP:
bool   emergency            # true → стоп, false → снять стоп