# Response.msg — ответ на ``Command`` через сервис ``ExecuteCommand`` (ADR-0051 §2.1).
#
# ``accepted`` отделяет «приняли в обработку» от «сделали что-то
# осмысленное» (ADR-0018 — не молчим на отказе). В monitor-режиме
# supervisor принимает команду, но не применяет (applied=false,
# reason=monitor_mode), чтобы клиент мог отличить «принято» от «сделано».
#
# ``reason`` — машинно-читаемыйый код причины (пусто на успехе):
#   - "unknown_kind"         — kind за пределами [1..6]
#   - "monitor_mode"         — supervisor в monitor (applied=false)
#   - "held_by_other"        — floor занят другим клиентом
#   - "mode_conflict"        — FSM отверг из-за avatar-mode
#   - "bad_request"          — пустой client_id, нераспознанный floor/mode
#   - "not_implemented"      — emergency_stop ещё не задеплоен (Phase 1)
#   - "voice_mode_rejected"  — недопустимый voice_mode
#   - "emergency_off"        — emergency=false (снять стоп — accepted=true, applied=false)
#   - "heartbeat_refreshed"  — heartbeat принят (always applied=true)
#   - "wrong_client"         — release/heartbeat от не-держателя floor
#   - "no_op"                — heartbeat от клиента без floor (no-op)
#
# Поля ``held_by`` / ``actual_mode`` / ``contacted_service`` —
# контекстные (заполняются соответствующей командой, иначе пустые).

bool   accepted
bool   applied
string reason

# Для ACQUIRE_FLOOR / RELEASE_FLOOR: текущий держатель (если применимо).
string held_by

# Для SET_AVATAR_MODE / SET_VOICE_MODE: итоговый режим (если применимо).
string actual_mode

# Имя сервиса arbiter, который был вызван (для debug). Пусто, если
# команда обработана локально (voice_mode / emergency_stop / heartbeat).
string contacted_service