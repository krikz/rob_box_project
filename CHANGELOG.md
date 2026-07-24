# Changelog

Все значимые изменения в проекте Rob Box документируются в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [Unreleased]

### Harness P0: pytest-конфиг и CI-джобы (Kanban `t_b13cf256`)

> Готовит почву для ветки `feature/harness-p0-foundation` →
> `develop` мерджа `wt/t_35cfe938` (harness framework) и
> `wt/t_2bf98118` (MiniMax LLM provider wrapper). Без этих файлов
> harness-тесты не имели единой точки запуска и coverage-гейта.

#### Добавлено

* **Корневой `pytest.ini`** — единая точка входа для тестов
  `rob_box_harness`. Маркеры: `unit`, `harness`, `integration`,
  `network`, `slow`. Coverage gate 85% для
  `rob_box_harness` (включая `providers/minimax.py`).
  `asyncio_mode = auto`, `addopts = --strict-markers`, явный
  `testpaths = src/rob_box_harness/test` чтобы не тащить в выборку
  весь `src/*` моно-репо.
* **Корневой `conftest.py`** — auto-skip `network`/`integration`
  тестов при отсутствии `MINIMAX_API_KEY` (default — replay,
  без интернета); sanity-чек в `pytest_report_header`.
* **CI-workflow `.github/workflows/G-Harness-Tests.yml`** —
  две джобы:
  * `unit-tests` (`Harness unit tests (no network)`) — default
    PR-чек, обязателен для merge в `develop`/`main`. Кэш pip по
    `src/rob_box_harness/requirements-dev.txt`. Coverage
    в Codecov (если задан `CODECOV_TOKEN`) и как артефакт.
  * `integration-tests` (`Harness integration tests`) —
    ручной/по расписанию, `replay` по умолчанию; `live`-режим
    включается только через `workflow_dispatch` +
    `secrets.MINIMAX_API_KEY`.
  * `test-summary` — пишет итог в GitHub Step Summary,
    fail-fast если `unit-tests` красные.
* **Документация `docs/development/HARNESS_TESTING.md`** — TL;DR,
  таблица маркеров, инструкция по branch protection
  (`gh api … required_status_checks.contexts`) и порядок добавления
  нового теста.

#### Acceptance (ТЗ)

* `pytest --collect-only` показывает все 5 маркеров (`unit`,
  `harness`, `integration`, `network`, `slow`).
* `pytest -m 'not network'` запускается локально без сети и
  без секретов; coverage ≥85% по `rob_box_harness` (включая
  `rob_box_harness/providers/minimax.py`) или fail с указанием
  непокрытых строк.
* Default-PR-чек — `Harness unit tests (no network)`; блокирует
  merge. `integration-tests` — manual/opt-in.

### [PR #907] — MiniMax LLM-интеграция в `rob_box_llm` (text + tools + vision)

> Ветка `feature/harness-p0-foundation` → `develop`. Один feature branch,
> внутри несколько фаз (M0 + M1 + M4, см. `architecture/minimax-provider.md`).
> TTS и image generation намеренно не входят — это отдельные адаптеры
> (TTS-фаза уже смержена через PR #907/ADR-0007).

#### Добавлено

* **`MiniMaxProvider`** — OpenAI-compatible адаптер существующего
  `LLMProvider` (`MiniMax-M3`, `https://api.minimax.io/v1`). Наследуется
  от общего `_OpenAICompatibleProvider` и переиспользует маппинг SDK
  исключений на типизированный `errors.ProviderError`.
* **Мультимодальный `LLMMessage.content`**: backward-compatible расширение
  до `str | tuple[MessagePart, ...]`. Новые value objects
  `TextPart`/`ImagePart` сериализуются в OpenAI `image_url` content
  blocks (URL pass-through / bytes → base64-data-URL).
* **`ProviderCapabilities`** + `capabilities_for(model)` —
  capability introspection для безопасного fallback и fail-fast gate
  до сетевого вызова. `image_input` сужается до vision-capable моделей
  (`*M3*`, `*M2-vision*`, `*vision*`).
* **`MINIMAX_MAX_IMAGE_BYTES = 10 MB`** — инженерный default для image
  payload; единая точка правки, юнит-тесты на превышение лимита.
* **`MiniMaxRedactedLogFilter`** — utility для гарантированного
  вычёркивания `MINIMAX_API_KEY` из log records.
* **Маппинг `base_resp.status_code`** — HTTP 200 c прикладной ошибкой
  MiniMax превращается в `AuthError`/`RateLimitError`/`ContentFilterError`/
  `ProviderError` через общий `_post_process_response` hook.
* **`MiniMaxProvider.thinking` (per-call override)** — default
  `{type: disabled}` (latency-sensitive); переопределяется через
  `settings.extra` для agent mode.
* **35 новых unit-тестов** (`test_minimax_provider.py`): fake SDK,
  text/tool/error/thinking/image-validation, vision off для не-vision моделей.
  85 зелёных в `rob_box_llm` итого.
* **Документация:**
  * `docs/guides/MINIMAX.md` — пользовательский гайд по text+vision
    провайдеру (API key, env, factory YAML, capabilities, troubleshooting).
  * `docs/guides/examples/minimax_llm.yaml` — копируемый шаблон
    `llm.providers` для registry/factory.
  * `src/rob_box_llm/README.md` — обновлён: добавлена таблица
    text+vision провайдеров с явными `name` / `base_url` / capabilities.
  * `.env.example` — секция `LLM ПРОВАЙДЕРЫ` (отдельно от TTS-блока).
  * `architecture/minimax-provider.md` — обзорный проектный документ.
  * `docs/adr/0002-minimax-provider.md` — capability-segregated ADR.

#### Изменено

* `src/rob_box_llm/rob_box_llm/__init__.py` — публичные экспорты
  `MiniMaxProvider`, `TextPart`, `ImagePart`, `MessagePart`,
  `MessageContent`, `ProviderCapabilities` (semver-minor: 0.1.0 → 0.2.1).
* `src/rob_box_llm/rob_box_llm/errors.py` — добавлены
  `CapabilityUnavailableError` и обновлён docstring иерархии.

#### Не входит в PR #907

* Реестр провайдеров / factory / fallback decorator — фаза M2, отдельная
  Kanban-задача (после PR #907).
* Миграция `DialogueNode` и Telegram `LLMChat` на новый `LLMProvider` —
  фаза M3, отдельная Kanban-задача. Текущие legacy-пути сохранены.
* Image generation через MiniMax — отложено (YAGNI), до подтверждённого
  consumer (потенциально Telegram media tool).

## [Март 2026] — PR #572: Integrate MCP tools, enhance documentation, and improve test coverage

> Ветка `feature/agent-skills` → `develop` | 566 коммитов | +68840 / -2670 строк

### 🎉 Добавлено

#### Voice Assistant — переписан на OpenAI Agents SDK
- **Полный рефакторинг `dialogue_node`** на OpenAI Agents SDK: Compositor + 4 суб-агента (Navigation, Memory, Music, Status)
- **Compositor паттерн** — главный агент делегирует задачи специализированным скилам через `FunctionTool`
- **Streaming + barge-in** — `run_streamed()` с мгновенным прерыванием через VAD-прерывание
- **Generation guard** — защита от stale потоков, публикующих TTS в новый диалог
- **Параметр `verbose_llm`** — полное логирование входа/выхода LLM через ROS 2 параметр
- **`get_current_time` MCP tool** — время через инструмент вместо инъекции в промт (KV cache)
- **Sentence splitting** — Yandex TTS: длинные тексты режутся по предложениям и озвучиваются чанками
- **SSML-only** — унификация формата ответа TTS, убран plain text
- **Бэрдж-ин grace период** — 5с после STT для предотвращения отклика на комнатное эхо
- **Многослойная музыкальная стратегия** — instant drums первым, добавление слоёв чанками

#### Telegram Bot (новый сервис)
- **`rob_box_telegram`** — операторский интерфейс через Telegram: команды, фото с потолочной камеры, TTS через бота
- CI/CD: telegram-bot добавлен во все Vision Pi workflows (build, deploy, single-service)
- Docker: Dockerfile без `--symlink-install`, корректный `chmod +x` стартового скрипта

#### Навигация
- **Динамический CRUD вейпоинтов** через SQLite — добавление/удаление/просмотр точек на лету
- **Go-speak-return миссии** — робот едет к вейпоинту, произносит фразу, возвращается
- **Fail-fast на ошибки Nav2** — голосовое сообщение об ошибке вместо молчания
- **Anti-hallucination tool markers** + защита от ложных навигационных команд в промте
- **Замена `rclpy.spin_until_future_complete` → `threading.Event`** в NavigationSkill (устранение дедлока)
- **`ReentrantCallbackGroup`** для Nav2 action client — устранение дедлока

#### Музыка (Renardo + SuperCollider)
- **Renardo MCP tool** — real-time генерация музыки через FoxDot/SuperCollider
- **SuperCollider в Docker Compose** — отдельный сервис с JACK/ALSA dmix
- **`music_max_amp` параметр** — ограничение амплитуды в LLM-генерируемом коде
- **Загрузка сэмплов Renardo** в Dockerfile (pitchglitch vocal)
- **Трёхшаговый стоп музыки** — корректная очистка SuperCollider synths
- **Гайдлайны Imperial March, Christmas Tree, waltz** — точные MIDI ноты и BPM
- **Monkey-patch для `spack`** в `getBufferFromSymbol`

#### Калибровка моторов
- **Серия калибровок `wheel_separation`**: 0.39 → 0.81 → 0.97 → 1.05 → 1.09 → 1.11 (на основе тестов вращения на 360°)
- **Калибровки `gear_ratio`**: 2.3 → 2.16 → 2.26 → 2.17
- **Circular footprint** для Nav2 (skid-steer вращение), `robot_radius` 0.35 → 0.45
- **`max_rps` калибровка**: 6.5 → 10 → 12.2 (через `move_test.py`)
- Документация: процедуры тестирования и калибровки моторов Rob Box

#### Телеуправление
- **Миграция BLE → SBUS serial protocol** для ExpressLRS джойстика
- **Авто-определение порта SBUS** при USB-реподключении
- Кнопки: остановка публикации нулей при дизарме (twist_mux timeout)

#### Тестирование
- **Docker-compose интеграционная тест-среда** для `dialogue_node` (scenario runner)
- **Mock MCP сервер** в scenario-runner с реальными инструментами
- **5 интеграционных сценариев**: barge-in A/B/C, rapid_messages, memory_context
- **`wait_for_idle()` через state topic** (замена `wait_for_quiet()` на sleep)
- **`voice_memory.db` фикстура** (85 turns, 1 fact) из SQL seed через python3
- **Переход с Ollama → DeepSeek API** в тестах
- **Self-hosted Pi runner** для нативных arm64 интеграционных тестов
- Юнит-тесты: `health_monitor` 19%→84%, `sound_node` 53%→85%, `led_node` 62%→90%, `reflection_node` 28%→75%, `context_aggregator` 43%→69%, `command_node` 48%→75%
- LED animation синхронизация и тесты (`led_node`)

#### Agent Skills & Документация
- `.agents/skills/context-engineering/SKILL.md` — методология Research→Design→Plan→Implement
- `.agents/skills/github-actions-runner/SKILL.md` — запуск CI/CD через UI и CLI
- `.agents/skills/docker-expert/SKILL.md` — Docker best practices, оптимизация, безопасность
- `.agents/skills/mcp-builder/SKILL.md` — создание MCP серверов (FastMCP/SDK)
- `.agents/skills/skill-creator/SKILL.md` — руководство по созданию skills
- `.agents/skills/motor-testing/SKILL.md` — тестирование моторов и одометрия
- `.agents/skills/zenoh-dev-setup/SKILL.md` — Zenoh для dev-машины
- `.agents/github-copilot/code-review-specialist/SKILL.md` — code review методология
- `.agents/github-copilot/python-testing-patterns/SKILL.md` — pytest best practices
- VS Code prompt files (`.github/copilot-instructions.md`, `.claude/commands/`) для Context Engineering
- ROADMAP.md + ROADMAP_SIMPLE.md — реализованные и планируемые фичи
- Zenoh dev-машина: setup документация + калибровочный скрипт

#### CI/CD
- Integration Tests workflow на self-hosted Pi runner (native arm64)
- `feat/*` ветки добавлены во все Single Service workflows
- SSH hardening в deploy workflow (предотвращение connection hangs)
- Cleanup stale `*.db` директорий перед checkout

### 🐛 Исправлено

#### Критические баги Voice Assistant
- **BUG-17** — silent retry при LLM timeout до порога счётчика ошибок
- **BUG-18** — `pending_queries` не обрабатываются после interrupt
- **BUG-19** — deadlock в `_recreate_llm_client` при вызове `client.close()` (py-spy диагноз)
- **BUG-20** — stale `_continue_after_tool_calls` озвучивает ответ в новый диалог
- **`Message.get()` bug** в non-streaming пути
- **`MaxTurnsExceeded`** обработка из OpenAI Agents SDK
- **Orphaned tool messages** — очистка истории от осиротевших tool_calls (HTTP 400)
- **Stale `pending_queries`** — очистка перед `listen_for_response`
- **`httpx.Client` пересоздание** после каждого LLM timeout
- **Параллельный `speak_text` стоп** при cancel run
- **Конкурентность `speak_text`** — `asyncio.Lock` для сериализации

#### MCP / Navigation
- **Nav2 action client deadlock** — `ReentrantCallbackGroup`
- **`self._node → self.node`** в mapping skill
- **Non-blocking service calls** — убран `spin_until_future_complete`
- **`set_vibe_preset`** — параметр `preset` → `preset_name` (MCP API)
- **`memory_save`** — параметр `content` → `fact`
- **`registry.execute`** — параметр `name` → `tool_name`
- **`GetCurrentTimeTool`** не был зарегистрирован в mcp_server
- **BEST_EFFORT QoS** для `/mcp/*` топиков (снижение задержки Zenoh)

#### Audio / SuperCollider
- **ALSA `dmix`** вместо `hw` device index — параллельное воспроизведение
- **JACK shm cleanup** — удаление stale SHM файлов при старте (предотвращение "default server already active")
- **jackd2 debconf hang** при установке supercollider-language
- **`shm_size: 256m`** для JACK Bus error
- **`jack_connect`** — подключение scsynth outputs к ALSA playback

#### Telegram
- **Ceiling camera topic** исправлен на `/ceiling_camera/image_raw/compressed`
- **f-strings для RcutilsLogger** (нет поддержки printf `%s`)
- **JSON с SSML** в tts_node вместо plain text

#### Конфигурация
- `docker-compose.yaml` — YAML multi-line python oneliners исправлены
- `wake_words: [""]` вместо `[]` — ROS 2 не может парсить пустой список

### ♻️ Рефакторинг
- **`_continue_after_tool_calls`**: рекурсия → итеративный while
- **`ConversationHistory`** вынесен в core слой
- **`VoiceCommandHandler`, `CommandParser`, `SpeechFormatter`, `DialogueManager`** — extraction в core
- **`ProviderManager`, `ToolCallExecutor`, `StreamingHandler`** — extraction в LLM layer
- **`MemoryManager`, `PromptFormatter`, `EventDetector`** — extraction в perception core
- **`ToolCallAccumulator`** вынесен в core layer
- `master_prompt_compact.txt` компактизация (555→128 линий, reverт до рабочей версии)
- Реорганизация `scripts/` и `tools/` в логическую структуру
- `.gitignore` — игнорировать `.agents/` skills директории

### 📚 Документация
- BUG-11, BUG-12..BUG-16 описания с анализом и fix-кодом
- BUG-18 — pending_queries hang after interrupt
- BUG-19 — deadlock с py-spy диагнозом
- Транскрипция видео для анализа
- AI HAT+ 26 TOPS анализ и актуализация ROADMAP
- CONTRIBUTING.md — таблица именования веток + SemVer правила

---

## [Февраль 2026]

### Добавлено
- PRD.md — Product Requirements Document с 34 задачами, milestones и acceptance criteria (19 февраля 2026)
- 11 специализированных AI-агентов в `docs/development/agents/`: navigation, backend, voice, frontend, devops, docs, git, security, scenarios, structure, **diagnostics** (19 февраля 2026)
- `docs/development/agents/diagnostics-agent.md` — агент удалённой диагностики: SSH-диагностика контейнеров, ROS 2 топиков, здоровья сервисов (19 февраля 2026)
- tasks.json — структурированный список задач с приоритетами и test_steps (19 февраля 2026)
- progress.md — лог выполнения задач агентами (19 февраля 2026)
- `docs/architecture/NETWORK_TOPOLOGY.md` — отдельный документ сетевой топологии (19 февраля 2026)
- SKILLS-001: добавлены `When to Apply` секции во все 12 agent guides (19 февраля 2026)
- SKILLS-002: оценка 4 пакетов skills.sh — ROS2 ниша пуста, внешние skills не установлены (19 февраля 2026)
- SKILLS-003: установлены 2 quality skills — `python-testing-patterns` и `code-review-specialist` (20 февраля 2026)

### Исправлено
- **TASK-048**: Double timeout hang (BUG-10 + BUG-15) — `_continue_after_tool_calls` переписан с рекурсии на итеративный while-цикл, устранено зависание 120с (20 февраля 2026)
- **TASK-035**: Серия фиксов стабильности agent mode (20 февраля 2026):
  - dmix `asound.conf` — параллельный TTS+sound через ALSA dmix
  - PlaySoundTool INSTANT — убран `time.sleep()`, fire-and-forget
  - ThreadPoolExecutor hang — `shutdown(wait=False)` в `_ask_llm_streaming` и `_continue_after_tool_calls`
  - interrupt_agent_loop — STT STOP прерывает цикл на итерации 2+
  - GetCurrentTimeTool — убрана инъекция времени в system_prompt (KV cache miss)
  - Убран preload past_turns — устранена "каша" с предыдущими сессиями
  - `conversation_history.clear()` при wake word из IDLE
  - TIME_CONTEXT_MARKER — исправлен поиск маркера для вставки времени
  - speak_text animation — добавлены псевдонимы neutral/excited/confused в enum

### Изменено
- Реорганизация документации: 10 файлов из корня `docs/` перемещены в соответствующие категории (27 февраля 2026)
- Архивировано 19 устаревших/дублирующих файлов документации (27 февраля 2026)
- Исправлены все ссылки на "Raspberry Pi 4" → "Raspberry Pi 5" (11 файлов) (27 февраля 2026)
- Консолидированы мелкие категории документации: `operations/` → `guides/`, `optimization/` → `development/` (27 февраля 2026)

## [Январь 2026]

### Добавлено
- `sound_catalog.json` — каталог из 51+ звуковых эффектов с метаданными (тип, теги, длительность)
- Загрузка звуков из `sound_catalog.json` в dialogue_node с логикой выбора по контексту (30 января 2026)
- `GetSoundInfoTool` — MCP инструмент для получения информации о доступных звуках (30 января 2026)
- Логирование использования токенов LLM API (документ: [TOKEN_USAGE_LOGGING.md](docs/fixes/TOKEN_USAGE_LOGGING.md), [TOKEN_USAGE_RU.md](docs/fixes/TOKEN_USAGE_RU.md)) (29 января 2026)
- MCP инструменты для провайдера DeepSeek (29 января 2026)

### Исправлено
- **Бесконечный цикл анимаций** — race condition в `llm_adapter.py`: Event теперь регистрируется ДО publish + добавлен лимит MAX_ITERATIONS=10 ([документ](docs/fixes/ANIMATION_LOOP_FIX.md)) (29 января 2026)
- **Синхронизация TTS и анимаций** — предотвращение смешивания анимаций между сеансами диалога ([документ](docs/fixes/ANIMATION_TTS_FIX.md)) (28 января 2026)
- **Scary story + системные звуки** — robot не молчит после "расскажу историю", устранено смешивание контекстов ([документ](docs/fixes/SCARY_STORY_FIX.md)) (29 января 2026)
- **QoS mismatch в deepseek_adapter** — несоответствие QoS настроек вызывало timeout ошибки ([документ](docs/fixes/QOS_MISMATCH_FIX.md)) (29 января 2026)
- **DeepSeek connection pool** — отключён httpx connection pooling для предотвращения idle timeout ([документ](docs/fixes/DEEPSEEK_CONNECTION_POOL_FIX.md)) (30 января 2026)
- **DeepSeek reasoner** — исправлен режим reasoner для корректной работы стриминга ([документ](docs/fixes/DEEPSEEK_REASONER_FIX.md)) (29 января 2026)
- **Stream timeout в dialogue_node** — добавлен таймаут для API stream соединения, устранено зависание (30 января 2026)
- **ThreadPoolExecutor deadlock** — устранён вложенный deadlock при создании stream (30 января 2026)
- **Повторение ответов LLM** — добавлены ограничения на повторения в промте ([документ](docs/fixes/PROMPT_REPETITION_FIX.md)) (29 января 2026)
- **Остановка agent cycle** — улучшены stopping conditions после tool calls, увеличен max_iterations с таймаутом (30 января 2026)
- **TTS ошибка устройства** — publish error state и сообщение при недоступности аудио устройства (30 января 2026)
- Удалён `set_emotion`, заменён на `play_animation` в системных промтах (30 января 2026)
- Fallback для Qwen отключён в конфигурации dialogue_node (30 января 2026)

## [Октябрь–Декабрь 2025]

### Добавлено
- Документация ICP Odometry: `docs/architecture/ICP_ODOMETRY.md` (декабрь 2025)
- GUI интерфейс управления роботом `tools/robot_control_gui_simple.py` (ноябрь 2025)
- Параметр `enable_search` для Qwen API web-поиска в dialogue_node и reflection_node (ноябрь 2025)
- Голосовой ассистент rob_box_voice с DeepSeek, Vosk STT, Silero TTS
- LED анимации rob_box_animations для WS2812B матриц
- Интеграция Zenoh для распределённой связи между Vision Pi и Main Pi
- Docker контейнеры для всех сервисов
- RTAB-Map SLAM с OAK-D Lite камерой
- AprilTag детекция на Vision Pi
- Nav2 навигация с командным управлением
- Документация в docs/ по стандартам ROS 2
- Система мониторинга с Grafana, Prometheus, Loki (24 октября 2025)
  - Легковесный мониторинг на отдельной машине
  - cAdvisor и Promtail на обоих Raspberry Pi
  - Красивые Grafana дашборды с 20 панелями
  - Скрипты enable/disable для управления мониторингом
- Полная документация по Zenoh namespace и облачному подключению (23 октября 2025)
- Исследование практик маппинга для RTAB-Map (24 октября 2025)
- Time awareness в dialogue_node - робот теперь знает текущее время (24 октября 2025)
- dialogue_id для синхронизации TTS чанков между сеансами диалога (24 октября 2025)

### Изменено
- Миграция с ROS 2 topics на Zenoh pub/sub
- Переход на offline-first стратегию для STT/TTS
- Реорганизация Docker структуры по стандартам проекта
- Оптимизация сборки для Raspberry Pi 5
- Система накопления запросов в dialogue_node — все запросы отправляются одним пакетом в DeepSeek (таймаут 2.5с) (4 ноября 2025)
- Автоматический fallback между Qwen и DeepSeek в dialogue_node и reflection_node (ноябрь 2025)
- Провайдер LLM по умолчанию изменён на DeepSeek (ноябрь 2025)
- Перемещение perception и lslidar контейнеров с Vision Pi на Main Pi (24 октября 2025)
- Рефакторинг системы мониторинга — агенты на Pi, центральный стек на отдельной машине (24 октября 2025)
- Изменена стратегия CI/CD — создание PR вместо прямого auto-merge (23 октября 2025)
- Реорганизация скриптов и конфигов согласно DOCKER_STANDARDS.md (24 октября 2025)

### Исправлено
- USB питание на Vision Pi для OAK-D камеры
- Проблемы с контейнерами Vision Pi (config volumes, network_mode)
- Ошибки компиляции apriltag и lslidar драйверов в Docker
- TF трансформации — robot-state-publisher теперь использует Zenoh namespace wrapper (24 октября 2025)
- Порядок TTS чанков — предотвращение смешивания между сеансами диалога (24 октября 2025)
- Дублирование запусков тестов и линтинга в CI/CD (23 октября 2025)
- Предупреждение 'PerceptionEvent не найден' в voice-assistant (24 октября 2025)
- Orphaned workflow build-all-local.yml — добавлен placeholder с deprecation notice (28 октября 2025)
- Инвалидация кэша Docker образа robot-state-publisher (19 ноября 2025)
- Поворот лидара на 180° — корректная ориентация LSLIDAR N10 (20 ноября 2025)
- Ориентация колёс и осей (20 ноября 2025)
- Парсинг JSON для ответов Qwen и DeepSeek (ноябрь 2025)
- Chipmunk эффект TTS — восстановлен оригинальный голос ([документ](docs/fixes/CHIPMUNK_VOICE_FIX_SUMMARY.md))

## [0.1.0] - 2025-10-04

### Добавлено
- Первый релиз базовой системы
- URDF модель робота rob_box_description
- Базовые launch файлы rob_box_bringup
- Интеграция VESC моторных контроллеров vesc_nexus
- ESP32 сенсорный хаб robot_sensor_hub_msg
- LED драйверы ros2leds и led_matrix_driver

---

**Навигация:** [← Назад в README](README.md) | [📚 Документация](docs/README.md)
