## 1. Research & prototype

- [ ] 1.1 Запустить resemblyzer на 10 наших записях (5 разных дикторов
      × 2 сессии), замерить cosine similarity same-speaker vs
      cross-speaker, подтвердить threshold 0.75.
- [ ] 1.2 Benchmark CPU latency на RPi5 8GB: 1.5s audio → embedding ms.

## 2. Implementation

- [ ] 2.1 Добавить `resemblyzer>=0.1.0,<0.2` в `rob_box_voice/setup.py`.
- [ ] 2.2 Создать `dialogue_node/speaker_recognizer.py` (Python class
      с методами `extract_embedding(audio)`, `match_or_create(embedding)`).
- [ ] 2.3 Создать SQLite schema migration `migrations/004_speakers.sql`
      с TTL cleanup function.
- [ ] 2.4 Подключить speaker_recognizer в `dialogue_node` через
      ThreadPoolExecutor (не блокировать wake-word pipeline).

## 3. ROS interface

- [ ] 3.1 Добавить publishers `/voice/speaker_id` (String) и
      `/voice/speaker_confidence` (Float32).
- [ ] 3.2 Обновить URDF / launch-файлы не требуется (топики
      публикуются нодой).

## 4. Tests

- [ ] 4.1 Unit test `test_speaker_recognizer.py`: 5 mock embeddings,
      проверка match-or-create логики с threshold 0.75.
- [ ] 4.2 Integration test: симулировать voice turn → проверить
      публикацию `/voice/speaker_id` и запись в SQLite.
- [ ] 4.3 E2E voice test (L-E2E Voice Test): реальный диктор говорит
      фразу → проверить, что id стабилен между двумя рестартами
      dialogue_node.

## 5. Deployment

- [ ] 5.1 Rebuild Docker image `rob_box_voice` с новыми deps.
- [ ] 5.2 Deploy на vision-pi, прогнать smoke test (10-минутная
      сессия без crash).
- [ ] 5.3 Закрыть change через `openspec archive fix-memory-speaker-id`.
