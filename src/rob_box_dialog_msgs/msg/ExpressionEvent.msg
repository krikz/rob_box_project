# Ответ исполнителя экспрессии (ADR-0150 §10.1).

# Кто исполнял (face, ring, sound).
string executor
uint8 KIND_STARTED=0
uint8 KIND_SKIPPED=1
# started или skipped.
uint8 kind
# Причина пропуска (invalid_face, busy); "" для started.
string reason
# Ход диалога.
string turn_id
