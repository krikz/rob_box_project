# Снимок состояния сессии (ADR-0150 §10.1, §7.5), latched.
# Заменяет /voice/dialogue/state и /voice/dialogue/barge_in_policy.

# Агент.
string agent
# Идентификатор сессии.
string session_id
# Текущая эпоха.
uint32 epoch
# Стек сцен, верхняя последняя.
SceneFrame[] scenes
# Голос, который просили для сессии.
string voice_requested
# Голос, который реально звучит.
string voice_applied
# SpeakerRef.person_ref: идентификатор собеседника ("" - неизвестен).
string speaker_id
# SpeakerRef.name: имя собеседника ("" - нет).
string speaker_name
# SpeakerRef.name_state: unknown | tentative | confirmed.
string speaker_name_state
# SpeakerRef.azimuth_deg; NaN - неизвестен.
float32 speaker_azimuth_deg
# Идентификатор вопроса, на который робот ждёт ответ ("" - нет).
string pending_ask
# Время снимка.
builtin_interfaces/Time stamp
