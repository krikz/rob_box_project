# Spec: voice-speaker-recognition

## Purpose

Детерминированная идентификация диктора голосового ассистента по
acoustic embedding с persistent storage между рестартами ROS-ноды.

## ADDED Requirements

### Requirement: Speaker embedding extraction

Система MUST извлекать 256-dimensional embedding из 1.5-секундного окна
voice audio, sampled at 16 kHz, mono.

#### Scenario: clean audio with sufficient duration
- **WHEN** dialogue_node получает audio chunk длиной ≥ 1.5s с RMS > 0.01
- **THEN** speaker_recognizer публикует embedding в `/voice/speaker_embedding`
  (Float32MultiArray, length=256) в течение 200 ms

#### Scenario: short or silent audio
- **WHEN** audio chunk короче 1.5s или RMS ≤ 0.01
- **THEN** speaker_recognizer публикует пустой embedding и confidence=0.0

### Requirement: Persistent speaker database

Система MUST хранить speaker embeddings в SQLite (`memory/speakers.db`)
со схемой:

```sql
CREATE TABLE speakers (
  id INTEGER PRIMARY KEY,
  embedding BLOB NOT NULL,    -- 256 × float32, ~1 KB
  label TEXT,
  first_seen TEXT NOT NULL,   -- ISO 8601
  last_seen TEXT NOT NULL
);
```

#### Scenario: known speaker matches existing record
- **WHEN** cosine similarity нового embedding с stored embedding ≥ 0.75
- **THEN** speaker_recognizer возвращает existing `id` и обновляет
  `last_seen`

#### Scenario: unknown speaker
- **WHEN** cosine similarity нового embedding со всеми stored embeddings < 0.75
- **THEN** speaker_recognizer создаёт новую запись и возвращает новый `id`

### Requirement: Speaker-id publication

Система MUST публиковать `/voice/speaker_id` (String) и
`/voice/speaker_confidence` (Float32) на каждом voice turn.

#### Scenario: confident recognition
- **WHEN** best match similarity ≥ 0.75
- **THEN** `/voice/speaker_id` = matched id, `/voice/speaker_confidence` = score

#### Scenario: low confidence
- **WHEN** best match similarity ∈ [0.5, 0.75)
- **THEN** `/voice/speaker_id` = "unknown", `/voice/speaker_confidence` = score

### Requirement: TTL-based cleanup

Система MUST удалять speaker-записи, которые не были seen > 30 дней.

#### Scenario: stale speaker
- **WHEN** nightly cron (04:00 local) находит запись с `last_seen` старше 30 дней
- **THEN** запись удаляется и логируется INFO с id
