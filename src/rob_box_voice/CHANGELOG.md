# Changelog

Все заметные изменения в этом проекте будут документироваться в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [Unreleased]

### Added
- **STT provider chain — Phase 2: `minimax → yandex → vosk`** (issue
  [#2365](https://github.com/krikz/rob_box_project/issues/2365),
  [ADR-0124](../../docs/adr/0124-stt-provider-chain-priority.md),
  заменяет ADR-0091 §2.2/§2.3/§5).
  - MiniMax STT наконец **подключён** в `stt_node._recognize_with_fallback`.
    До этого Phase 1 поставила готовый, оттестированный и полностью
    выключенный провайдер: цепочка собирала только `yandex → vosk`.
  - Порядок задаётся ROS-параметром `stt_provider_chain` и
    нормализуется (`_normalize_provider_chain`) по тем же инвариантам,
    что цепочка TTS: только известные провайдеры, без дублей,
    **`vosk` всегда последний**, битая цепочка → дефолт.
  - `ProviderPolicy` — per-provider таймаут/повторы. Раньше бюджет был
    один на всю цепочку, а повторы доставались только первому
    провайдеру.
  - `ProviderDeadCache` — кэш «мёртвых» провайдеров с TTL
    (квота/ключ → 300с, сеть/таймаут → 30с), персистентный между
    рестартами (`provider_state_file`, формат общий с
    `tts_provider_state.json`). Пропуск виден в логе попыток как
    `reason="dead"`. Успешный ответ снимает отметку; если мертвы все —
    кэш игнорируется.
  - Типизированные `STTAuthError` / `STTQuotaError` рядом с
    `STTTimeoutError`; gRPC-коды Yandex и исключения MiniMax
    маппятся на них (`_map_grpc_error`, `_recognize_minimax`) — без
    этого «кончились деньги» и «моргнула сеть» неотличимы.
  - Фактический провайдер после фолбека фиксируется в логе при смене
    (`🎧 STT provider → ...`) и в `provider_state_file`. Топика
    `/voice/stt/provider_state` намеренно нет: потребителя у него пока
    ноль, а это ровно то, что ловит сторож issue #2118 (ADR-0124 §2.5).
  - **Фикс:** `MiniMaxSTTProvider` отправлял headerless PCM в поле
    `audio.wav` с типом `audio/wav` — сервер не мог узнать sample rate.
    Добавлен `ensure_wav_container()` (готовый WAV пропускается как есть).
  - `config/stt_chain.yaml` **удалён**: второй YAML-источник для того же
    значения — класс ошибки issue #1252 / #1734. Конфигурация цепочки —
    `declare_parameter` + `config/stt_node.yaml` (issue #1004).
  - `select_recognition` разложена на `_classify_attempt` / `_policy_for`
    / `_live_providers` / `_run_provider`: CC 18 → 6 (ADR-0021).
  - Тесты: новый `test/test_stt_dead_cache.py` (31) + 32 теста цепочки,
    `provider_state` и маппинга ошибок в `test/test_stt_node_fallback.py`.
- **MiniMax STT provider — Phase 1 PoC** (issue
  [#2365](https://github.com/krikz/rob_box_project/issues/2365), PR
  [#2369](https://github.com/krikz/rob_box_project/pull/2369),
  [ADR-0091](../../docs/adr/0091-minimax-stt-provider.md)).
  - `MiniMaxSTTProvider` (`src/rob_box_voice/rob_box_voice/stt_providers/minimax_provider.py`)
    — sync/async обёртка над `POST https://api.minimax.io/v1/speech_to_text`,
    реализует `STTProvider` Protocol из `stt_fallback.py`.
    Bearer-авторизация через `MINIMAX_API_KEY`, `response_format=json`,
    `timestamp_level=word`, типизированные
    `MiniMaxSTTAuthError` / `MiniMaxSTTRateLimitError` /
    `MiniMaxSTTUnavailableError` / `MiniMaxSTTInvalidResponseError`.
    API-key redaction filter (`MiniMaxSTTRedactedLogFilter`) на
    module + `httpx` логгерах. Фабрика
    `MiniMaxSTTProvider.maybe_from_env(api_key_env="MINIMAX_API_KEY")`
    возвращает `None`, если ключ не задан — цепочка пропускает
    провайдер без warning.
  - Документационный SSoT `config/stt_chain.yaml` —
    описывал предполагаемый порядок Phase 2: `vosk → minimax → yandex`.
    (Файл удалён в Phase 2, см. ADR-0124 §2.6.)
  - Operator-гайд
    [`docs/architecture/minimax-stt-provider.md`](../../docs/architecture/minimax-stt-provider.md)
    — env-var (`MINIMAX_API_KEY`), chain order, toggle on/off,
    pytest-команды, troubleshooting.
  - Обновлён `README.md` (этот пакет): блок STT-провайдеров и
    подсекция «MiniMax STT (Phase 1 PoC)».
  - Расширен class docstring
    `MiniMaxSTTProvider` — секции «When to prefer this provider» и
    «Configuration» (cloud + diarization trade-offs).
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
- **Issue #1389**: защитный паттерн против «забытого ключа» в
  `_llm_skipped_counter` dict-init. Counter теперь строится из
  единой константы `_LLM_SKIP_REASONS` (single source of truth),
  тестовый fixture использует ту же константу. Регрессионный тест
  `test_counter_keys_match_constant` сканирует increment-сайты в
  `dialogue_node.py` и проверяет, что все они покрыты константой
  → следующий worker, добавляющий новый skip-reason, не сможет
  «забыть» ключ в `__init__`. (Конкретный bug `e2e_busy` откатан
  в 4742a390 — revert #1386.)

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
