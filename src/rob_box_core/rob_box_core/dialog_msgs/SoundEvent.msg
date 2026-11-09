# Ответ звукового узла на триггер (ADR-0150 §10.1, V33): вместо молчаливого return.

# Идентификатор запроса звука.
string request_id
uint8 KIND_STARTED=0
uint8 KIND_FINISHED=1
uint8 KIND_SKIPPED=2
# Что произошло.
uint8 kind
# Причина пропуска (busy, voice_stream, not_found, error, superseded); "" иначе.
string reason
# Имя звука.
string sound
