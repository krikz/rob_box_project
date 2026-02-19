# Changelog

Все значимые изменения в проекте Rob Box документируются в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [Unreleased]

### Добавлено
- PRD.md — Product Requirements Document с 34 задачами, milestones и acceptance criteria (19 февраля 2026)
- 11 специализированных AI-агентов в `docs/development/agents/`: navigation, backend, voice, frontend, devops, docs, git, security, scenarios, structure, **diagnostics** (19 февраля 2026)
- `docs/development/agents/diagnostics-agent.md` — агент удалённой диагностики: SSH-диагностика контейнеров, ROS 2 топиков, здоровья сервисов (19 февраля 2026)
- tasks.json — структурированный список задач с приоритетами и test_steps (19 февраля 2026)
- progress.md — лог выполнения задач агентами (19 февраля 2026)
- `docs/architecture/NETWORK_TOPOLOGY.md` — отдельный документ сетевой топологии (19 февраля 2026)

## [Январь 2026]

### Добавлено
- `sound_catalog.json` — каталог из 51+ звуковых эффектов с метаданными (тип, теги, длительность)
- Загрузка звуков из `sound_catalog.json` в dialogue_node с логикой выбора по контексту (30 января 2026)
- `GetSoundInfoTool` — MCP инструмент для получения информации о доступных звуках (30 января 2026)
- Логирование использования токенов LLM API (документ: [TOKEN_USAGE_LOGGING.md](TOKEN_USAGE_LOGGING.md), [TOKEN_USAGE_RU.md](TOKEN_USAGE_RU.md)) (29 января 2026)
- MCP инструменты для провайдера DeepSeek (29 января 2026)

### Исправлено
- **Бесконечный цикл анимаций** — race condition в `llm_adapter.py`: Event теперь регистрируется ДО publish + добавлен лимит MAX_ITERATIONS=10 ([документ](ANIMATION_LOOP_FIX.md)) (29 января 2026)
- **Синхронизация TTS и анимаций** — предотвращение смешивания анимаций между сеансами диалога ([документ](ANIMATION_TTS_FIX.md)) (28 января 2026)
- **Scary story + системные звуки** — robot не молчит после "расскажу историю", устранено смешивание контекстов ([документ](SCARY_STORY_FIX.md)) (29 января 2026)
- **QoS mismatch в deepseek_adapter** — несоответствие QoS настроек вызывало timeout ошибки ([документ](QOS_MISMATCH_FIX.md)) (29 января 2026)
- **DeepSeek connection pool** — отключён httpx connection pooling для предотвращения idle timeout ([документ](DEEPSEEK_CONNECTION_POOL_FIX.md)) (30 января 2026)
- **DeepSeek reasoner** — исправлен режим reasoner для корректной работы стриминга ([документ](DEEPSEEK_REASONER_FIX.md)) (29 января 2026)
- **Stream timeout в dialogue_node** — добавлен таймаут для API stream соединения, устранено зависание (30 января 2026)
- **ThreadPoolExecutor deadlock** — устранён вложенный deadlock при создании stream (30 января 2026)
- **Повторение ответов LLM** — добавлены ограничения на повторения в промте ([документ](PROMPT_REPETITION_FIX.md)) (29 января 2026)
- **Остановка agent cycle** — улучшены stopping conditions после tool calls, увеличен max_iterations с таймаутом (30 января 2026)
- **TTS ошибка устройства** — publish error state и сообщение при недоступности аудио устройства (30 января 2026)
- Удалён `set_emotion`, заменён на `play_animation` в системных промтах (30 января 2026)
- Fallback для Qwen отключён в конфигурации dialogue_node (30 января 2026)
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
- Оптимизация сборки для Raspberry Pi 4
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
- Chipmunk эффект TTS — восстановлен оригинальный голос ([документ](docs/CHIPMUNK_VOICE_FIX_SUMMARY.md))

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
