# Changelog

Все заметные изменения в этом проекте будут документироваться в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [Unreleased]

### Added
- **Query Queue System** — система накопления запросов для пакетной обработки
  - Накопление нескольких быстрых запросов в очереди
  - Пакетная обработка всех накопленных запросов одним запросом к LLM
  - Параметр конфигурации `query_accumulation_timeout` (по умолчанию 2.5 секунды)
  - Автоматическая цепочка обработки после завершения LLM
  - Очистка очереди при команде молчания
  - Unit тесты для новой функциональности
  - Документация `docs/QUERY_QUEUE_SYSTEM.md`

### Changed
- Улучшен `dialogue_node.py` с механизмом накопления запросов
- Обновлён конфиг `voice_assistant.yaml` с параметром `query_accumulation_timeout`
- Обновлён `README.md` с примерами использования Query Queue System

### Fixed
- Исправлена проблема накопления неактуальных ответов при быстрых запросах
- **Issue #1389**: KeyError 'e2e_busy' на первом STT после старта voice-assistant.
  Counter `_llm_skipped_counter` теперь строится из единой константы `_LLM_SKIP_REASONS`
  (single source of truth) — невозможно «забыть» ключ в `__init__`, если он есть
  в increment site. Регрессионный тест `test_counter_includes_e2e_busy_key` ловит
  расхождение константы и increment-сайтов.

## [1.0.0] - 2025-10-XX

### Added
- Оригинальный голос ROBBOX (нормальная скорость, без эффекта "бурундука")
- SSML управление pitch и speed из dialogue_node
- Голосовые команды управления громкостью ("громче", "тише", "громко")
- Опциональный эффект "бурундука" через параметры

### Initial Release
- ReSpeaker Mic Array v2.0 интеграция
- STT с Vosk/Whisper/Yandex
- TTS с Yandex/Silero
- Dialogue Node с DeepSeek/Local LLM
- LED индикация состояний
- Command Node для управления роботом
- Sound Node для звуковых эффектов
- 7 модульных ROS2 нод

[Unreleased]: https://github.com/krikz/rob_box_project/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/krikz/rob_box_project/releases/tag/v1.0.0
