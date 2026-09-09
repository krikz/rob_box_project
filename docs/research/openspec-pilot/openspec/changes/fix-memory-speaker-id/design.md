## Context

Текущая реализация в `rob_box_voice/dialogue_node.py` использует
инкрементальный счётчик: первый услышанный диктор получает `id=0`,
второй — `id=1` и т.д. Счётчик живёт в RAM ноды; при рестарте он
обнуляется. После рестарта тот же диктор получает **новый** id,
поэтому его long-term memory (факты, привычки) теряется: в
SQLite-запросах `WHERE speaker_id = 0` находит старые факты, но
`/voice/speaker_id` теперь стримит новое число.

OpenSpec даёт нам формальный контракт (spec) для нового поведения +
чеклист (tasks) для реализации. Этот design описывает конкретный
тех-подход.

## Goals / Non-Goals

**Goals:**
- Deterministic speaker-id (один и тот же голос → один id после рестарта).
- Persistent storage embeddings, не чувствительный к рестарту ROS.
- Backwards-compat: старые `speaker_id=0` записи остаются в БД,
  но помечаются как legacy (используются только для read-only fallback).

**Non-Goals:**
- Multi-language speaker recognition (только русский).
- Real-time re-training embeddings (модель зафиксирована в Docker image).
- Anti-spoofing / liveness detection.

## Decisions

1. **Модель: resemblyzer (VoiceEncoder)**, 256-dim embedding, ONNX-export
   для CPU-only RPi5. Альтернатива: pyannote (слишком тяжёлая),
   speechbrain (более новый, но тяжелее). Resemblyzer — проверенный
   компромисс.

2. **Threshold 0.75 cosine similarity** — из публичных бенчмарков
   resemblyzer на VoxCeleb (clean, single-speaker). Будем A/B-тестить
   на наших записях.

3. **SQLite vs PostgreSQL** — SQLite, потому что voice stack работает
   на RPi5 без сервера БД. Postgres overkill.

4. **TTL = 30 дней** — компромисс между privacy и удобством.
   Записи без active use — удаляются.

5. **Single global DB path** — `~/.ros/speakers.db` (XDG convention),
   чтобы все процессы видели одну БД.

## Risks / Trade-offs

- **RPi5 CPU**: 1.5s audio → embedding ≈ 200-400 ms на RPi5 8GB. Может
  тормозить wake-word pipeline. Митигация: запускаем recognizer в
  отдельном thread (ThreadPoolExecutor, как в dialogue_node уже есть).
- **Cold start**: новый диктор → новая запись, требуется ≥ 1.5s
  чистой речи для первого embedding. Для шумных сред — fallback
  на "unknown" + ручное подтверждение через Telegram bot.
- **False positive при threshold 0.75**: близнецы / очень похожие
  голоса могут сливаться. Допустимый trade-off для MVP.

## Alternatives Considered

- **Whisper embeddings**: уже есть whisper_node, но Whisper embeddings
  не специализированы на speaker-id (transcript-oriented). Отвергли.
- **PIP-install speaker-recognition от SpeechBrain**: работает, но
  Docker image вырастет на ~800 MB. Отвергли.
- **Хранить только label, без embeddings**: не работает — нужен
  similarity для новых записей.
