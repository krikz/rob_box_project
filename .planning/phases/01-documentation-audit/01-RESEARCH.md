# Phase 1: Аудит документации — Research

**Researched:** 2026-05-15
**Phase:** 1 — Аудит документации
**Requirements:** DOCS-01, DOCS-02, DOCS-03, DOCS-04, DOCS-05, DOCS-06

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| DOCS-01 | Все файлы в `docs/` актуальны — устаревшие разделы удалены или помечены | Audit reveals multiple stale files; removals and updates identified per file |
| DOCS-02 | `README.md` корректно описывает текущее состояние проекта (железо, стек, запуск) | SSH username wrong, missing recent services, links to stale sub-docs |
| DOCS-03 | `docs/architecture/` отражает реальную Docker-топологию (Main Pi / Vision Pi сервисы) | Both compose files read; gap list documented |
| DOCS-04 | `ROADMAP.md` синхронизирован с фактическим состоянием реализованных фич | Covered below — ROADMAP.md freshness assessed |
| DOCS-05 | Каждый ROS 2 пакет в `src/` имеет актуальный README с описанием нод, топиков, параметров | 5 packages missing README; 3 existing READMEs are stale |
| DOCS-06 | `docs/development/` содержит актуальные гайды по запуску, деплою и разработке | QUICK_START.md has critical errors; other guides partially stale |
</phase_requirements>

---

## Summary

Аудит выявил три класса проблем: **критические ошибки** (неверные данные, которые заведут нового разработчика в тупик), **устаревшие описания** (сервисы/контейнеры, которых больше нет или есть новые) и **пробелы** (пакеты без документации). Наиболее критичны: неверный SSH-username в `QUICK_START.md` (`ubuntu@` вместо `ros2@`), несуществующий контейнер `animation-player` в примерах команд, и отсутствие архитектурного описания 5 новых Vision Pi сервисов (`telegram-bot`, `supercollider`, `ceiling-camera`, `voice-resources-init`, `ollama`). Пять пакетов (`led_matrix_driver`, `rob_box_bringup`, `rob_box_description`, `rob_box_perception_msgs`, `rob_box_telegram`) не имеют README совсем. Три существующих README (`rob_box_voice`, `rob_box_perception`, `rob_box_mcp_tools`) содержат устаревшую информацию.

**Primary recommendation:** Исправить критические ошибки в QUICK_START.md и SSH-инструкциях немедленно; затем обновить `docs/architecture/SYSTEM_OVERVIEW.md` до полного списка 12+ сервисов; затем создать/обновить README для всех 13 пакетов по единому шаблону.

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Docker topology docs | docs/architecture/ | docker/*.yaml (source of truth) | Compose files are authoritative; docs reflect them |
| Package API docs (nodes/topics/params) | src/*/README.md | — | Node source is authoritative |
| Quick start / deployment | docs/guides/ + docs/deployment/ | README.md (entry point) | Guides are detailed; README.md links to them |
| Architecture diagrams | docs/architecture/SYSTEM_OVERVIEW.md | — | Single source for high-level diagrams |

---

## Current State Analysis

### docs/ Directory Inventory

#### architecture/ — СТALE (частично)

| Файл | Версия/Дата | Статус | Проблема |
|------|------------|--------|----------|
| `SYSTEM_OVERVIEW.md` | v1.1.0 / 2025-10-24 | **STALE** | Docker-секция пропускает 5 новых Vision Pi сервисов; депенденси-диаграмма Vision Pi неполная |
| `HARDWARE.md` | v1.0.0 / 2025-10-12 | **STALE** | "5× LED Matrix" (правильно: 381 LED NeoPixel, 5 панелей); BLE-джойстик заблокирован kernel bug но не помечен |
| `SOFTWARE.md` | v1.0.0 / 2025-10-12 | **STALE** | Не содержит: `rob_box_telegram`, `rob_box_mcp_tools`, `robot_sensor_hub_msg`; Docker секция устарела аналогично SYSTEM_OVERVIEW |
| `NETWORK_TOPOLOGY.md` | v1.0 / 2026-02-19 | **CURRENT** | Корректна, самая свежая в папке |
| `ICP_ODOMETRY.md` | — | **CURRENT** | Технически корректна для текущей реализации |
| `INTERNAL_DIALOGUE_VOICE_ASSISTANT.md` | — | CURRENT | — |
| `ZENOH_CLOUD_NAMESPACES.md` | — | CURRENT | — |

#### guides/ — STALE (критичные ошибки)

| Файл | Статус | Проблема |
|------|--------|----------|
| `QUICK_START.md` | **CRITICAL STALE** | SSH username `ubuntu@` вместо `ros2@`; упомянут контейнер `animation-player` (не существует, нет в compose); путь `~/rob_box_project/` vs `/opt/rob_box_project/` непоследователен |
| `TROUBLESHOOTING.md` | **STALE** | Основной кейс — "RTAB-Map не получает данные от камеры" — неактуален: RTAB-Map сейчас работает LiDAR-only (`depth:=false`); проблема с OAK-D для RTAB-Map неактуальна |
| `MONITORING_QUICK_REF.md` | **STALE** | Grafana показана на `10.1.1.10:3000` (Main Pi), но monitoring — отдельная машина с `docker/monitoring/`; команды путают где запускать |
| `VISION_PI_SETUP.md` | **STALE** | Username `pi` в примерах Raspberry Pi Imager, реальный username `ros2` |
| `NAV2_SETUP.md` | Неизвестно | Не читался (ниже приоритет) |
| `QUICK_START.md` ссылка на `animation-player` | **CRITICAL** | `docker exec -it animation-player ros2 service call ...` — контейнер называется `led-matrix` |

#### development/ — MIXED (много файлов, разный возраст)

| Файл | Статус | Заметка |
|------|--------|---------|
| `DOCKER_STANDARDS.md` | CURRENT | Правила подтверждены compose файлами |
| `PYTHON_STYLE_GUIDE.md` | CURRENT | — |
| `TESTING_GUIDE.md` | Неизвестно | Нет временной метки |
| `AGENT_GUIDE.md` | Неизвестно | Большой файл, не полностью прочитан |
| `VIBE_CODING_*.md` (несколько) | Вероятно STALE | Ссылаются на "Q1 2025" планы, сейчас май 2026 |
| `VOICE_AGENT_BEST_PRACTICES.md` | CURRENT | — |
| `LLM_REFACTORING_SUMMARY.md` | Неизвестно | — |

#### deployment/ — CURRENT (mostly)

| Файл | Статус | Заметка |
|------|--------|---------|
| `DEPLOYMENT_WORKFLOW.md` | CURRENT | GitHub Actions деплой хорошо документирован |
| `VOICE_ASSISTANT_DOCKER.md` | Вероятно CURRENT | — |
| `VISION_PI_DEPLOYMENT.md` | Вероятно CURRENT | — |
| `MONITORING_DEPLOYMENT.md` | Неизвестно | — |

#### Остальные файлы верхнего уровня в docs/

| Файл | Статус | Заметка |
|------|--------|---------|
| `CI_CD_PIPELINE.md` | Вероятно CURRENT | — |
| `DOCUMENTATION_STRUCTURE.md` | **STALE** | Структура docs/ изменилась, новые файлы не отражены |

---

### Root README.md Assessment

**Версия**: Нет явной версии. Последнее изменение: февраль 2026.

**Что корректно:**
- Общее описание проекта, badges CI/CD
- Список ссылок на документацию в целом верный
- Аппаратный стек (dual Pi 5) — корректен
- Цель проекта и этапы (навигация, голос, LED)

**Что стало неверным/устаревшим:**
- Ссылка на `docs/guides/QUICK_START.md` ведёт в документ с неверным SSH username
- Ссылка на `docs/guides/VISION_PI_SETUP.md` ведёт в документ с неверным username `pi`
- Раздел "⚡ Последние изменения" заканчивается февралём 2026 (PRD + AI агенты). Это не последнее изменение — после него появились: telegram-bot (в docker/vision), GSD планирование (май 2026)
- Ссылка `[Конфигурация RTAB-Map + LiDAR](docs/guides/VISUALIZATION.md)` — неверный файл (VISUALIZATION.md про RViz, не про RTAB-Map конфигурацию)
- Секция "Для разработчиков" ссылается на `PRD.md` и `tasks.json` — они частично заменены GSD системой
- Нет упоминания telegram-bot, ceiling-camera, supercollider, mcp-tools

**Что отсутствует:**
- SSH username явно не указан в README.md (только по ссылке в QUICK_START)
- Нет упоминания monitoring machine и Grafana endpoint
- Нет упоминания `rob_box_telegram` пакета

---

### src/ Package README Assessment

| Пакет | README есть | Ноды описаны | Топики описаны | Параметры описаны | Критические пробелы |
|-------|-------------|--------------|----------------|-------------------|---------------------|
| `rob_box_voice` | ✅ | ✅ (7 нод) | ⚠️ частично | ⚠️ частично | `ros-jazzy-audio-common-msgs` → должно быть `kilted`; установка описана для bare metal, не Docker; не упомянут `USE_SKILLS`, `VoiceMemory`, `MusicSkill`, `ollama` интеграция |
| `rob_box_perception` | ✅ | ✅ | ⚠️ частично | ❌ нет | "Рефакторинг Q1 2025" — устарело на год; `vision_stub_node` описан как TODO, но это рабочая реализация |
| `rob_box_animations` | ✅ | ✅ | ✅ | ✅ | Хорошая документация; `animation-player` container name неверен — в docker-compose нет такого container, он в `led-matrix` |
| `rob_box_mcp_tools` | ✅ | ⚠️ | ⚠️ | ❌ | "Рефакторинг Q1 2025" устарело; нет списка MCP tools и их назначений |
| `rob_box_teleop` | ✅ | ✅ | ✅ | ✅ | Хорошая документация. BLE джойстик заблокирован kernel bug 6.14.0-raspi — не отражено |
| `vesc_nexus` | ✅ | ✅ | ✅ | ✅ | Хорошая документация |
| `robot_sensor_hub_msg` | ✅ | N/A (только msgs) | ✅ | N/A | Хорошая документация |
| `ros2leds` | ✅ | ✅ | ✅ | ✅ | Хорошая документация |
| `led_matrix_driver` | ❌ | — | — | — | Нет README. `ros2leds/README.md` покрывает оба пакета, но это не в правильном месте |
| `rob_box_bringup` | ❌ | — | — | — | Нет README. Пакет содержит launch-файлы: `display.launch.py`, `rob_box_control.launch.py`, `complete_system.launch.py` |
| `rob_box_description` | ❌ | — | — | — | Нет README. Содержит URDF/Xacro, meshes (.stl), Gazebo worlds |
| `rob_box_perception_msgs` | ❌ | N/A (только msgs) | N/A | N/A | Нет README. Сообщения, используемые perception |
| `rob_box_telegram` | ❌ | — | — | — | Нет README. Telegram бот: `telegram_node.py`, `mcp_bridge.py`, `handlers/`; полноценный операторский интерфейс |

---

### Docker Architecture Reality

#### Main Pi (`docker/main/docker-compose.yaml`) — 11 сервисов

| Контейнер | Назначение |
|-----------|-----------|
| `zenoh-router` | Zenoh Router (peer mode), мост к облаку |
| `twist-mux` | Мультиплексирование cmd_vel с приоритетами |
| `micro-ros-agent` | UART↔ROS2 bridge для ESP32 Sensor Hub |
| `robot-state-publisher` | TF дерево из URDF |
| `rtabmap` | LiDAR-only SLAM + ICP odometry |
| `ros2-control` | ros2_control manager + VESC CAN драйвер |
| `lslidar` | LS LiDAR N10 driver |
| `perception` | Internal dialogue: reflection_node, context_aggregator, health_monitor |
| `nav2` | Nav2 navigation stack |
| `teleop` | Joystick teleoperation (SBUS/serial) |
| `cadvisor` (profile: monitoring) | Container metrics |
| `promtail` (profile: monitoring) | Log shipping |

#### Vision Pi (`docker/vision/docker-compose.yaml`) — 10 сервисов

| Контейнер | Назначение |
|-----------|-----------|
| `zenoh-router-vision` | Zenoh Router (client mode), подключается к Main Pi |
| `oak-d` | OAK-D Lite driver + **встроенный AprilTag detector** |
| `led-matrix` | LED matrix controller (381 NeoPixel LEDs) |
| `ceiling-camera` | Потолочная USB-камера 720p |
| `supercollider` | Audio synthesis server (JACK/ALSA) |
| `voice-resources-init` | One-shot: загрузка renardo samples в volume |
| `voice-assistant` | Voice assistant: Vosk STT, TTS, LLM dialogue |
| `telegram-bot` | Telegram operator interface |
| `cadvisor-vision` (profile: monitoring) | Container metrics |
| `promtail-vision` (profile: monitoring) | Log shipping |
| `ollama` (profile: ai) | Optional: локальный LLM для embeddings |

#### Monitoring Machine (`docker/monitoring/docker-compose.yaml`) — отдельная машина

Prometheus, Loki, Grafana — развёртываются на **отдельной машине**, НЕ на Main Pi. Агенты (cAdvisor, Promtail) на каждом Pi отправляют данные на неё.

---

### Orphaned Files/Directories

| Путь | Проблема |
|------|----------|
| `docker/vision/apriltag/Dockerfile` | AprilTag интегрирован в oak-d контейнер; отдельный Dockerfile больше не используется, но директория `docker/vision/apriltag/` с `Dockerfile` осталась |

---

### Key Gaps Found (по приоритету)

1. **[CRITICAL] QUICK_START.md — неверный SSH username**: `ubuntu@` вместо `ros2@`. Блокирует подключение нового разработчика с первого шага.

2. **[CRITICAL] QUICK_START.md — несуществующий контейнер `animation-player`**: Все ROS-команды через этот контейнер упадут. Правильное имя — `led-matrix`.

3. **[HIGH] SYSTEM_OVERVIEW.md Docker-секция**: Отсутствуют 5 Vision Pi сервисов (`ceiling-camera`, `supercollider`, `voice-resources-init`, `telegram-bot`, `ollama`) и 1 Main Pi сервис (`teleop`). Диаграмма зависимостей Vision Pi неполная.

4. **[HIGH] 5 пакетов без README**: `led_matrix_driver`, `rob_box_bringup`, `rob_box_description`, `rob_box_perception_msgs`, `rob_box_telegram`. Невозможно понять назначение без чтения кода.

5. **[HIGH] TROUBLESHOOTING.md — устаревший основной кейс**: Описывает "RTAB-Map не получает данные от камеры" — этот режим отключён (`depth:=false`). Нужны актуальные кейсы: Zenoh connectivity, LiDAR не публикует, VESC timeout.

6. **[MEDIUM] rob_box_voice README — stale dependencies**: `ros-jazzy-audio-common-msgs` должно быть `ros-kilted-*`. Установка описана для bare metal, а не Docker. Новые фичи (`USE_SKILLS`, `VoiceMemory`, `MusicSkill`, `Renardo/SuperCollider`) не описаны.

7. **[MEDIUM] rob_box_perception README — "Q1 2025" статус рефакторинга**: Прошёл год. Нужно либо пометить как DONE/changed либо обновить статус.

8. **[MEDIUM] rob_box_mcp_tools README — "Q1 2025" статус**: Аналогично.

9. **[MEDIUM] SOFTWARE.md / HARDWARE.md**: Не обновлялись с октября 2025. Отсутствуют: `rob_box_telegram`, `rob_box_mcp_tools`, `robot_sensor_hub_msg` в пакетах; ceiling-camera, supercollider, telegram-bot в сервисах.

10. **[LOW] Orphaned `docker/vision/apriltag/Dockerfile`**: Вводит в заблуждение — выглядит как активный сервис.

11. **[LOW] VISION_PI_SETUP.md — username `pi`**: Raspberry Pi Imager пример настраивает `pi`, но реальный username `ros2`.

---

## Implementation Approach

### Plan 01-01: Аудит и исправление docs/

**Файлы для обновления (ordered by priority):**

**1. `docs/guides/QUICK_START.md`** — Critical fixes:
- Заменить все `ubuntu@` → `ros2@`
- Убрать/заменить `animation-player` → `led-matrix`
- Добавить примеры для `telegram-bot` и `ceiling-camera` (или убрать лишние старые)
- Сверить список сервисов с актуальным compose

**2. `docs/guides/TROUBLESHOOTING.md`** — Restructure:
- Убрать/обновить раздел "RTAB-Map не получает данные от камеры" → заменить на LiDAR-only troubleshooting
- Добавить: Zenoh connection issues, VESC timeout, perception container unhealthy
- Ссылки на `docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md` уже есть в copilot-instructions

**3. `docs/guides/VISION_PI_SETUP.md`** — Minor fix:
- Username `pi` → `ros2` в примерах

**4. `docs/guides/MONITORING_QUICK_REF.md`** — Clarify:
- Явно написать что monitoring — отдельная машина, не Main Pi
- Указать где берётся мониторинг (docker/monitoring/)

**5. `docs/architecture/SYSTEM_OVERVIEW.md`** — Major update (covered в Plan 01-02)

**6. `docs/architecture/SOFTWARE.md`** — Medium update:
- Добавить пакеты: `rob_box_telegram`, `rob_box_mcp_tools`, `robot_sensor_hub_msg`
- Добавить сервисы аналогично SYSTEM_OVERVIEW

**7. `docs/architecture/HARDWARE.md`** — Minor fix:
- "5× LED Matrix" → "381 NeoPixel LED (5 панелей 5×5 + 1 панель)"
- Joystick: добавить NOTE о kernel bug 6.14.0-raspi

**8. `DOCUMENTATION_STRUCTURE.md`** — Update:
- Актуализировать список файлов

**Оценка трудоёмкости Plan 01-01**: 3-4 файла с хирургическими правками, 2 файла с medium переписью → ~1 день работы агента.

---

### Plan 01-02: Обновление docs/architecture/ (Docker-топология)

**Целевое состояние**: `docs/architecture/SYSTEM_OVERVIEW.md` раздел 6 (Docker инфраструктура) точно отражает оба compose файла.

**Конкретные изменения в SYSTEM_OVERVIEW.md:**

1. **Раздел 6.1 (Структура docker/)** — добавить в Vision Pi:
   - `ceiling-camera/` — USB потолочная камера
   - `supercollider/` — audio synthesis
   - `telegram_bot/` — Telegram operator interface
   - Убрать: отдельный `apriltag/` (интегрирован в oak-d)

2. **Раздел 6.2 (Зависимости контейнеров)** — Полная диаграмма:
   - Main Pi: полный список из 10 сервисов
   - Vision Pi: полный список из 8+ сервисов с правильными зависимостями (`voice-assistant` depends on `supercollider` + `voice-resources-init`)

3. **Добавить таблицу сервисов** (как выше в этом документе) — одна таблица Main Pi, одна Vision Pi, отдельная секция monitoring machine.

4. **Обновить "Дата" в заголовке**: с 2025-10-24 → 2026-05-15

**Оценка трудоёмкости Plan 01-02**: Средняя переработка одного большого файла (~811 строк). 2-3 раздела нуждаются в обновлении. ~0.5 дня.

---

### Plan 01-03: README для пакетов src/

**Шаблон README для каждого пакета:**

```markdown
# <package_name>

<Одна строка: назначение пакета>

## Описание

<2-3 предложения>

## Ноды

| Нода | Назначение |
|------|-----------|
| `node_name` | ... |

## Топики

### Публикует
| Топик | Тип | Частота | Описание |
|-------|-----|---------|---------|

### Подписывается
| Топик | Тип | Описание |
|-------|-----|---------|

## Параметры

| Параметр | По умолчанию | Описание |
|----------|-------------|---------|

## Запуск

<Как запустить в Docker контексте>
```

**Пакеты для создания README (с нуля):**

| Пакет | Что содержит | Приоритет |
|-------|-------------|-----------|
| `rob_box_telegram` | `telegram_node.py`, `mcp_bridge.py`, handlers; Telegram оператор interface с LLM чат, голосом, фото | HIGH (новый незадокументированный сервис) |
| `rob_box_bringup` | launch-файлы: `display.launch.py`, `rob_box_control.launch.py`, `complete_system.launch.py` | HIGH (точка входа в запуск системы) |
| `rob_box_description` | URDF/Xacro, meshes (.stl), Gazebo worlds; геометрия и TF дерево | MEDIUM |
| `led_matrix_driver` | Низкоуровневый SPI драйвер NeoPixel; публикует/подписывается на LED chain | MEDIUM (ros2leds README покрывает но не в правильном месте) |
| `rob_box_perception_msgs` | Custom ROS2 messages для perception (узнать список из source) | LOW (msgs-only пакет) |

**Пакеты для обновления существующего README:**

| Пакет | Что обновить |
|-------|-------------|
| `rob_box_voice` | Fix: `ros-kilted-*` (не jazzy); добавить: `USE_SKILLS`, `VoiceMemory`, Ollama, SuperCollider/Renardo интеграция; убрать bare metal install steps |
| `rob_box_perception` | Обновить статус рефакторинга; добавить параметры нод |
| `rob_box_mcp_tools` | Обновить статус рефакторинга; добавить список MCP tools |
| `rob_box_animations` | Fix: container name `animation-player` → `led-matrix` в примерах команд |

---

## Risk & Dependencies

### Риски

1. **rob_box_telegram README**: Пакет не задокументирован. Нужно читать код (`telegram_node.py`, `mcp_bridge.py`) чтобы выяснить топики/параметры. Умеренный effort.

2. **rob_box_perception_msgs**: Нужно прочитать `.msg` файлы в пакете. Низкий риск.

3. **rob_box_voice README**: Нужно проверить что `USE_SKILLS`, `VoiceMemory`, Ollama интеграции задокументированы корректно — нет reference docs, нужно читать код.

4. **SYSTEM_OVERVIEW.md обновление**: Файл большой (811 строк). Нужно обновить несколько секций без поломки Mermaid диаграмм. Средний риск.

5. **"Q1 2025 рефакторинг" в rob_box_perception / rob_box_mcp_tools**: Нужно либо (a) установить факт — рефакторинг завершён? не завершён? — либо просто убрать временные метки.

### Зависимости между планами

- Plan 01-01 и 01-02 независимы (разные файлы).
- Plan 01-03 независим от 01-01 и 01-02.
- Все три плана могут выполняться параллельно.

---

## Validation Architecture

### Как проверить критерии успеха

**Критерий 1**: Новый разработчик может запустить робота только по документации

Тест (ручной walkthrough):
```bash
# Следовать README.md → QUICK_START.md
sshpass -p 'open' ssh ros2@10.1.1.20   # должно работать
cd ~/rob_box_project/docker/main
docker compose up -d
docker compose ps    # все сервисы healthy
```
Автоматизированная проверка:
```bash
# Убедиться что SSH username ros2@ везде в docs/
grep -rn "ubuntu@" docs/ README.md   # должно вернуть 0 результатов
```

**Критерий 2**: docs/architecture/ содержит актуальную Docker-топологию

Тест:
```bash
# Извлечь список сервисов из compose файлов
grep "container_name:" docker/main/docker-compose.yaml | sort > /tmp/main_actual.txt
grep "container_name:" docker/vision/docker-compose.yaml | sort > /tmp/vision_actual.txt
# Сравнить с тем что написано в SYSTEM_OVERVIEW.md
# Не должно быть сервисов в compose которых нет в docs
```

**Критерий 3**: Каждый пакет src/ имеет README

Тест (автоматический):
```bash
for d in src/*/; do
  pkg=$(basename "$d")
  readme="$d/README.md"
  if [ -f "$readme" ]; then echo "OK: $pkg"
  else echo "MISSING: $pkg"; fi
done
# Все строки должны начинаться с "OK:"
```

**Критерий 4**: README.md корректно описывает аппаратный стек и порядок запуска

Тест (spot-check):
- SSH username корректен: `ros2@` (не `ubuntu@`, не `pi`)
- Контейнер `led-matrix` (не `animation-player`)
- Наличие telegram-bot и ceiling-camera в архитектурном описании
- LED: 381 NeoPixel (не просто "LED Matrix")

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | Мониторинг деплоится на **отдельной** машине (не Main Pi), Grafana на 10.1.1.10:3000 доступна через проброс портов или машина мониторинга = Main Pi | Docker Architecture Reality | Неверное описание deployment в docs |
| A2 | `rob_box_bringup` launch-файлы не используются в Docker (контейнеры запускаются напрямую через скрипты), но пакет нужен для bare metal / RViz | Plan 01-03 | Неверный приоритет описания |
| A3 | `led_matrix_driver` отличается от `ros2leds` (отдельный пакет в src/) | Package README table | Может быть дублирующаяся документация |

---

## Open Questions

1. **Мониторинг на Main Pi или отдельной машине?**
   - Grafana URL в документации: `http://10.1.1.10:3000` → это Main Pi IP
   - `docker/monitoring/` → отдельный compose для мониторинговой машины
   - Но Prometheus scrapes `10.1.1.10:8080` (cAdvisor на Main Pi) — Prometheus должен видеть Pi
   - **Рекомендация**: перед обновлением MONITORING_QUICK_REF.md — уточнить у разработчика где фактически запущен Grafana.

2. **Статус рефакторинга rob_box_perception и rob_box_mcp_tools (Q1 2025)?**
   - Прошёл год. Реализован ли он?
   - **Рекомендация**: быстрый `git log --oneline src/rob_box_perception/ | head -10` покажет свежие коммиты

3. **`docker/vision/apriltag/Dockerfile` — удалить или оставить?**
   - Это orphaned файл
   - **Рекомендация**: удалить при выполнении Plan 01-02 (или создать issue)

---

## Environment Availability

Step 2.6: SKIPPED — фаза документации, нет внешних зависимостей. Все изменения — это редактирование Markdown файлов в репозитории.

---

## Validation Architecture (Nyquist)

Фаза состоит исключительно из документационных изменений (Markdown, без кода). Автоматические тесты неприменимы для содержимого текстовых файлов.

**Проверка завершения:**
```bash
# Тест 1: Нет username ubuntu@ в docs
grep -rn "ubuntu@" docs/ README.md
# Expected: empty output (0 matches)

# Тест 2: Все src/ пакеты имеют README
for d in src/*/; do
  [ -f "$d/README.md" ] && echo "OK: $(basename $d)" || echo "MISSING: $(basename $d)"
done
# Expected: all OK

# Тест 3: Список контейнеров в SYSTEM_OVERVIEW совпадает с compose
# Manual spot-check достаточен для документации
```

---

## Sources

### Primary (HIGH confidence)
- `docker/main/docker-compose.yaml` — список всех Main Pi сервисов [VERIFIED: прочитан полностью]
- `docker/vision/docker-compose.yaml` — список всех Vision Pi сервисов [VERIFIED: прочитан полностью]
- `src/*/README.md` — состояние существующих README [VERIFIED: прочитаны ключевые файлы]
- `find src/ -name README.md` + проверка missing [VERIFIED: terminal output]

### Secondary (MEDIUM confidence)
- `docs/architecture/SYSTEM_OVERVIEW.md` — контент прочитан частично (811 строк, ключевые секции) [VERIFIED]
- `docs/guides/QUICK_START.md` — полностью прочитан [VERIFIED]
- `README.md` (root) — прочитан [VERIFIED]

### Tertiary (LOW confidence)
- `docs/development/*.md` — большинство только просмотрены по имени, не прочитаны полностью [ASSUMED — статус "STALE" для VIBE_CODING_* на основе "Q1 2025" ссылок]

---

## Metadata

**Confidence breakdown:**
- Current service list (docker): HIGH — прочитаны оба compose файла
- README gaps: HIGH — проверен shell loop
- Stale content in specific docs: HIGH — прочитаны ключевые файлы
- docs/development/ status: MEDIUM — большинство файлов не прочитаны полностью
- Monitoring machine topology: LOW — логика выведена, не верифицирована

**Research date:** 2026-05-15
**Valid until:** 2026-08-15 (стабильная архитектура, изменения возможны только при добавлении новых сервисов)
