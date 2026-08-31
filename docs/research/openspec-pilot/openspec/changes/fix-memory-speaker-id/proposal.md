## Why

Голосовой ассистент путает speaker-id между пользователями после
перезагрузки ROS-ноды: сессии разных дикторов сливаются в одну
`speaker_id=0` запись, что ломает long-term memory и voice profile.
Нужен надёжный способ различать дикторов и сохранять привязку
сессия→диктор между рестартами.

## What Changes

- Добавить детерминированный speaker-id на основе acoustic embedding
  (resemblyzer / voice embedding) вместо инкрементального счётчика.
- Сохранять speaker embeddings в SQLite (`memory/speakers.db`) с TTL
  на 30 дней.
- При reconnect ROS-ноды подтягивать embeddings и матчить новые
  голоса по cosine similarity ≥ 0.75.
- Логировать каждое распознавание speaker с evidence (score, top-3
  candidates).

## Capabilities

### New Capabilities
- `voice-speaker-recognition`: детерминированная идентификация диктора
  по acoustic embedding с persistent storage.

### Modified Capabilities
- (нет на уровне требований; `dialogue_node` поведение не меняется
  снаружи — только internal speaker-id resolution.)

## Impact

- `rob_box_voice/dialogue_node.py`: новый модуль `speaker_recognizer.py`.
- `rob_box_voice/memory/`: новая SQLite-схема `speakers(id, embedding,
  label, last_seen)`.
- ROS-топики: `/voice/speaker_id` (новый, String) + `/voice/speaker_confidence`
  (новый, Float32).
- Docker: новый Python-deps (`resemblyzer>=0.1.0`).
