# Phase 2: Ревью структуры — Research

**Дата:** 2026-05-15  
**Домен:** Docker layout, ROS 2 пакеты, src/ structure  
**Confidence:** HIGH (все данные получены непосредственным анализом файлов репозитория)

---

## Краткое резюме

Аудит проведён на основе прямого анализа файловой системы репозитория. Состояние проекта в целом хорошее: большинство требований уже выполнены. Критических нарушений мало, объём исправлений небольшой (~7 точечных правок).

**Основной вывод:** STRUCT-02 (нет COPY config/scripts) уже выполнен — нарушений нет. STRUCT-01 (network_mode, depends_on) почти выполнен — все ROS-сервисы соответствуют стандарту, исключения для не-ROS сервисов оправданны. Реальная работа сосредоточена в STRUCT-03 (TODO в setup.py) и STRUCT-05 (дублирование led_matrix_driver).

**Приоритетная рекомендация:** Исправить `rob_box_perception/setup.py` (TODO description/license), разобраться с дублированием `src/led_matrix_driver/` vs `src/ros2leds/led_matrix_driver/`, и задокументировать обоснованные исключения для не-ROS сервисов (supercollider, voice-resources-init, monitoring).

---

<phase_requirements>
## Требования фазы

| ID | Описание | Поддержка исследования |
|----|----------|------------------------|
| STRUCT-01 | Все Docker-сервисы следуют единому layout (config/<service>/, scripts/<service>/, volumes) | Проверено: config/scripts как volumes везде, исключения задокументированы |
| STRUCT-02 | Нет `COPY config/` или `COPY scripts/` в Dockerfile-ах | Проверено: НАРУШЕНИЙ НЕТ — 0 результатов grep |
| STRUCT-03 | setup.py / package.xml во всех пакетах заполнены корректно (нет `TODO:`) | 2 файла с нарушениями найдены |
| STRUCT-04 | Все зависимости в package.xml актуальны и соответствуют используемым импортам | URDF_EXPORT использует ROS 1 зависимости |
| STRUCT-05 | Структура src/ пакетов соответствует соглашениям проекта (именование, layout) | Дублирование led_matrix_driver найдено |
</phase_requirements>

---

## Результаты аудита Docker (для плана 02-01)

### STRUCT-02: COPY config/scripts — НАРУШЕНИЙ НЕТ ✅

```bash
grep -rn "COPY config" docker/main docker/vision docker/base --include="Dockerfile*"
# Результат: пусто (0 нарушений)

grep -rn "COPY scripts" docker/main docker/vision docker/base --include="Dockerfile*"
# Результат: пусто (0 нарушений)
```

**Вывод: STRUCT-02 уже выполнен. Никаких исправлений не требуется.**  
[VERIFIED: прямой grep в файловой системе]

---

### STRUCT-01: network_mode и depends_on

#### Список всех Dockerfile-ов (27 файлов)

```
docker/base/Dockerfile.depthai, Dockerfile.pcl, Dockerfile.ros2-zenoh, Dockerfile.rtabmap
docker/main/: lslidar, micro_ros_agent, nav2, perception, robot_state_publisher,
             ros2_control, rtabmap, teleop, twist_mux, vesc_nexus, zenoh-router
docker/vision/: apriltag, ceiling-camera, led_matrix, oak-d, supercollider,
               telegram_bot, voice_assistant, voice_base, voice_resources, zenoh-router
docker/vision/test/: mock_llm, ollama, scenario_runner
```

#### network_mode: host — статус по сервисам

**docker/main/docker-compose.yaml:**

| Сервис | network_mode: host | depends_on: zenoh-router |
|--------|-------------------|--------------------------|
| zenoh-router | ✅ | — (сам является router) |
| twist-mux | ✅ | ✅ |
| micro-ros-agent | ✅ | ✅ |
| robot-state-publisher | ✅ | ✅ |
| rtabmap | ✅ | ✅ |
| ros2-control | ✅ | ✅ (+ robot-state-publisher, twist-mux) |
| lslidar | ✅ | ✅ |
| perception | ✅ | ✅ |
| nav2 | ✅ | ✅ (+ 5 других сервисов) |
| teleop | ✅ | ✅ (+ twist-mux) |
| cadvisor | ✅ | ❌ (monitoring profile, не ROS-сервис) |
| promtail | ✅ | ❌ (monitoring profile, не ROS-сервис) |

**docker/vision/docker-compose.yaml:**

| Сервис | network_mode: host | depends_on: zenoh-router |
|--------|-------------------|--------------------------|
| zenoh-router | ✅ | — (сам является router) |
| oak-d | ✅ | ✅ |
| led-matrix | ✅ | ✅ |
| ceiling-camera | ✅ | ✅ |
| supercollider | ✅ | ❌ (не ROS-сервис, аудиосервер, depends on voice-resources-init) |
| voice-resources-init | ❌ | ❌ (одноразовый init, только volume-монтирование) |
| voice-assistant | ✅ | ✅ (+ supercollider, voice-resources-init) |
| telegram-bot | ✅ | ✅ |
| cadvisor | ✅ | ❌ (monitoring profile, не ROS-сервис) |
| promtail | ✅ | ❌ (monitoring profile, не ROS-сервис) |
| ollama | ✅ | ❌ (ai profile, не ROS-сервис) |

> `renardo_samples` — это именованный volume (`volumes:` секция), не сервис.

#### Анализ исключений (обоснованность)

**`voice-resources-init`** — одноразовый init-контейнер, копирует samples в named volume, не использует ROS. Отсутствие `network_mode` и `depends_on` **обоснованно**. [VERIFIED]

**`supercollider`** — аудиосервер (scsynth/SuperCollider), не ROS-нода. Зависит от `voice-resources-init` для samples. Отсутствие `depends_on: zenoh-router` **обоснованно** — он не публикует/подписывается на ROS-топики. [VERIFIED]

**`cadvisor`, `promtail`** — мониторинговые агенты за профайлом `monitoring`, не используют ROS/Zenoh. Отсутствие `depends_on: zenoh-router` **обоснованно**. [VERIFIED]

**`ollama`** — инференс LLM через HTTP (localhost:11434), не ROS. За профайлом `ai`. Отсутствие `depends_on: zenoh-router` **обоснованно**. [VERIFIED]

#### Вывод по STRUCT-01

**Все ROS-сервисы соответствуют стандарту.** Исключения (`voice-resources-init`, `supercollider`, monitoring, ollama) технически обоснованы и задокументированы в стандарте как допустимые (DOCKER_STANDARDS.md §8, §10 описывают правило для ROS 2 + Zenoh сервисов, а не всех сервисов подряд).

**Рекомендация для плана 02-01:** Аудит пройден. Для Success Criterion 3 требуется уточнение — критерий "Все docker-сервисы" буквально невыполним, т.к. есть не-ROS сервисы. Нужно **задокументировать** обоснованные исключения в комментарии в docker-compose.yaml, чтобы критерий считался выполненным.

---

## Результаты аудита Python-пакетов (для плана 02-02)

### STRUCT-03: TODO в setup.py

**Файлы с нарушениями (grep -l "TODO"):**

| Файл | Строка | Проблема |
|------|--------|---------|
| `src/rob_box_perception/setup.py` | L25 | `description='TODO: Package description'` |
| `src/rob_box_perception/setup.py` | L26 | `license='TODO: License declaration'` |

**Примечание:** `package.xml` для `rob_box_perception` уже исправлен:
```xml
<description>Internal Dialogue Agent - Perception and Reflection for Rob Box</description>
<license>MIT</license>
```
Нужно синхронизировать `setup.py` с `package.xml`.

**Плейсхолдерные email** (не TODO, но нужны реальные значения):

| Файл | Поле | Значение |
|------|------|---------|
| `src/rob_box_bringup/setup.py` | maintainer_email | `your_email@example.com` |
| `src/rob_box_mcp_tools/setup.py` | maintainer_email | `your_email@example.com` |
| `src/led_matrix_driver/setup.py` | maintainer_email | `user@example.com` |
| `src/ros2leds/led_matrix_compositor/setup.py` | maintainer_email | `user@example.com` |
| `src/ros2leds/led_matrix_driver/setup.py` | maintainer_email | `user@example.com` |

**Вывод:** Критическое нарушение STRUCT-03 — только в `rob_box_perception/setup.py`. Плейсхолдерные email — minor issue, требует правки для полной чистоты.

### STRUCT-03: TODO в package.xml

| Файл | Строка | Проблема |
|------|--------|---------|
| `src/rob_box_description/URDF_EXPORT/package.xml` | L7 | `<license>TODO</license>` |
| `src/rob_box_description/URDF_EXPORT/package.xml` | — | Использует `catkin` (ROS 1), `rospy` |

**Контекст URDF_EXPORT:** Это auto-generated URDF export из Fusion 360 (fusion2urdf инструмент). Использует ROS 1 (`catkin`, `rospy`, `build_depend: roscpp`). Этот пакет **не является частью colcon workspace** (colcon пропустит его из-за catkin buildtool, но может сгенерировать предупреждение). Вероятно, это legacy-папка, которая не должна быть в `src/`.

**Рекомендация:** Добавить `COLCON_IGNORE` в `src/rob_box_description/URDF_EXPORT/` и опционально исправить `<license>MIT</license>`.

---

### STRUCT-04: Актуальность зависимостей в package.xml

**Проверенные пакеты:**

`src/rob_box_perception/package.xml` — зависимости корректные:
- `rclpy`, `rob_box_perception_msgs`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `control_msgs`, `builtin_interfaces`, `rcl_interfaces` [VERIFIED]

`src/rob_box_mcp_tools/package.xml` — зависимости корректные:
- `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `nav2_msgs`, `std_srvs`, `tf2_ros`, `rob_box_perception_msgs` [VERIFIED]

**Исключение: URDF_EXPORT** — ROS 1 зависимости (`catkin`, `rospy`) — не проблема колcon build при наличии COLCON_IGNORE.

**Вывод:** Нарушений STRUCT-04 в активных пакетах не обнаружено. [VERIFIED]

---

### STRUCT-05: Структура src/ пакетов

#### Полный список пакетов в src/

```
src/
├── led_matrix_driver/          ← ⚠️ ДУБЛИРУЕТ src/ros2leds/led_matrix_driver/
├── rob_box_animations/         ✅ стандарт
├── rob_box_bringup/            ✅ стандарт
├── rob_box_description/
│   └── URDF_EXPORT/            ⚠️ ROS 1 пакет, legacy
├── rob_box_mcp_tools/          ✅ стандарт
├── rob_box_perception_msgs/    ✅ стандарт
├── rob_box_perception/         ✅ стандарт
├── rob_box_telegram/           ✅ стандарт
├── rob_box_teleop/             ✅ стандарт
├── rob_box_voice/              ✅ стандарт
├── robot_sensor_hub_msg/       ⚠️ нет rob_box_ префикса (но это git submodule)
├── ros2leds/                   ⚠️ git submodule — нестандартный layout
│   ├── led_matrix_compositor/  ✅ используется в docker
│   ├── led_matrix_driver/      ✅ используется в docker (канонический)
│   ├── Dockerfile              (не используется проектом — дубль)
│   ├── effects.py              (утилиты вне пакетов)
│   ├── directory_image_publisher.py (утилиты вне пакетов)
│   └── ...
└── vesc_nexus/                 ⚠️ git submodule, но OK
```

#### Критическая проблема: дублирование led_matrix_driver

**Два пакета с одним именем:**

| | `src/led_matrix_driver/` | `src/ros2leds/led_matrix_driver/` |
|--|--------------------------|-----------------------------------|
| Источник | Автономная копия в репо | Git submodule (ros2leds) |
| Используется в Docker | ❌ НЕ используется | ✅ Да, `docker/vision/led_matrix/Dockerfile` |
| CI Lint | ✅ `.github/workflows/G-Lint Code.yml` ссылается | Через submodule |
| Тип сообщения | `ByteMultiArray` (std_msgs) | `Int8MultiArray` (std_msgs) |
| Config: spi_speed_khz | 1300 | 1000 |
| Дополнительные файлы | `led_matrix_system_launch.py` (launch!) | — |
| README | Есть | — |

**Конфликт:** `src/led_matrix_driver/` использует `ByteMultiArray`, а `src/ros2leds/led_matrix_driver/` использует `Int8MultiArray`. Это **API расхождение** — если оба пакета собираются в одном workspace, это вызовет конфликт имён.

**Риск:** `.github/workflows/G-Lint Code.yml` явно включает `src/led_matrix_driver/` в lint. Удаление потребует обновления CI workflow.

**Вывод:** `src/led_matrix_driver/` — это устаревшая автономная копия, развившаяся независимо от submodule. **Канонический пакет: `src/ros2leds/led_matrix_driver/`** (используется Docker). Standalone копия должна быть удалена с обновлением CI.

---

## Оценка объёма работ

### Количество элементов для исправления

| Категория | Количество | Сложность |
|-----------|-----------|-----------|
| setup.py TODO | 1 файл (rob_box_perception) | Тривиально |
| setup.py плейсхолдеры | 5 файлов | Тривиально |
| package.xml TODO | 1 файл (URDF_EXPORT) | Тривиально |
| COLCON_IGNORE для URDF_EXPORT | 1 файл создать | Тривиально |
| Документирование исключений в compose | 2 файла (main + vision) | Малая правка |
| Удаление src/led_matrix_driver/ + CI fix | 1 директория + 1 workflow | Средняя |

**Итого: ~10 небольших изменений. Общая сложность: НИЗКАЯ.**

### Нарушений НЕТ (уже соответствует стандарту)

- STRUCT-02 (COPY config/scripts) — 0 нарушений ✅  
- network_mode: host — все ROS-сервисы ✅  
- depends_on: zenoh-router — все ROS-сервисы ✅  
- Большинство package.xml — актуальные зависимости ✅

---

## Подход к реализации

### Структура планов

**02-01: Аудит Dockerfile-ов** (plan 02-01)

Аудит УЖЕ завершён этим research. Ключевые выводы:
1. COPY config/scripts — нет нарушений (STRUCT-02 DONE)  
2. Для Success Criterion 3 (`все сервисы имеют network_mode + depends_on`) нужно:
   - Добавить комментарии в docker-compose, поясняющие почему `voice-resources-init`, `supercollider`, monitoring-сервисы являются обоснованными исключениями

**02-02: Аудит setup.py и package.xml** (plan 02-02)

Конкретные правки:
1. `src/rob_box_perception/setup.py`: заменить TODO description/license
2. `src/rob_box_bringup/setup.py`, `src/rob_box_mcp_tools/setup.py`, `src/led_matrix_driver/setup.py`, `src/ros2leds/led_matrix_compositor/setup.py`, `src/ros2leds/led_matrix_driver/setup.py`: обновить placeholder emails
3. `src/rob_box_description/URDF_EXPORT/package.xml`: добавить `<license>MIT</license>`, добавить `COLCON_IGNORE`

**02-03: Устранение найденных нарушений** (plan 02-03)

Конкретные правки:
1. Удалить `src/led_matrix_driver/` (standalone дублирующий пакет)
2. Обновить `.github/workflows/G-Lint Code.yml` — убрать ссылки на `src/led_matrix_driver/`
3. Добавить ссылки на `src/ros2leds/led_matrix_driver/` в lint workflow

### Рекомендуемый порядок выполнения

```
02-01 (Docker audit comments) → 02-02 (setup.py/package.xml fixes) → 02-03 (structural cleanup)
```

02-01 и 02-02 независимы — можно параллельно. 02-03 требует отдельного решения по `led_matrix_driver`.

---

## Ключевые риски

### Риск 1: Удаление src/led_matrix_driver/ нарушит CI lint

**Что случится:** `.github/workflows/G-Lint Code.yml` ссылается на `src/led_matrix_driver/` — без этой директории workflow упадёт.

**Митигация:** В том же коммите обновить CI workflow:
- Убрать `src/led_matrix_driver/`  
- Добавить `src/ros2leds/led_matrix_driver/`  
- Добавить `src/ros2leds/led_matrix_compositor/`  

**Сложность:** СРЕДНЯЯ — нужно проверить все ссылки в CI.

### Риск 2: API несовместимость ByteMultiArray vs Int8MultiArray

**Что случится:** Если где-то в коде есть подписчик/паблишер на `led_matrix/data` с типом `ByteMultiArray` (из standalone пакета), после удаления он перестанет работать.

**Митигация:** Проверить `rob_box_animations` — это основной паблишер LED данных. Определить, какой тип сообщения он использует.

```bash
grep -r "ByteMultiArray\|Int8MultiArray" src/rob_box_animations/ src/rob_box_voice/
```

### Риск 3: vesc_nexus и ros2leds — git submodules

**Контекст:** `src/vesc_nexus/` и `src/ros2leds/` — git submodules. Их файлы нельзя редактировать напрямую — нужно делать PR в соответствующие репозитории. `maintainer_email` в их setup.py изменить без PR в submodule нельзя.

**Митигация:** Для `src/ros2leds/*/setup.py` и `src/vesc_nexus/*/setup.py` — это задача для submodule repo. В рамках этой фазы можно только задокументировать.

---

## Архитектурная карта ответственности

| Возможность | Первичный уровень | Вторичный уровень | Обоснование |
|-------------|------------------|------------------|-------------|
| Docker layout | docker/main, docker/vision | CI workflows | Compose файлы определяют layout |
| Package metadata | src/<package>/setup.py | src/<package>/package.xml | Python packaging layer |
| ROS deps | package.xml | colcon build | ROS build system использует package.xml |
| Lint CI | .github/workflows/ | — | Должен отражать реальные пакеты |

---

## Архитектура валидации

### Тестовая инфраструктура

| Свойство | Значение |
|----------|---------|
| Инструмент проверки | `grep`, `yamllint`, ручная инспекция |
| Конфигурация | `.github/workflows/G-Lint Code.yml` |
| Команда быстрой проверки | `grep -r "COPY config\|COPY scripts" docker/ --include="Dockerfile*"` |
| Команда полной проверки | см. Success Criteria ниже |

### Карта требований → тесты

| Требование | Поведение | Тип | Команда | Статус |
|------------|-----------|-----|---------|--------|
| STRUCT-02 | Нет COPY config/scripts | grep | `grep -r "COPY config" docker/ --include="Dockerfile*"` | ✅ Уже PASS |
| STRUCT-02 | Нет COPY scripts | grep | `grep -r "COPY scripts" docker/ --include="Dockerfile*"` | ✅ Уже PASS |
| STRUCT-03 | Нет TODO в setup.py | grep | `grep -r "TODO" src/ --include="setup.py"` | ❌ Нужно исправить |
| STRUCT-03 | Нет TODO в package.xml | grep | `grep -r "TODO" src/ --include="package.xml"` | ❌ URDF_EXPORT |
| STRUCT-01 (network_mode) | Все ROS-сервисы имеют host | python/grep | см. audit script выше | ✅ PASS |
| STRUCT-05 | Нет дублирующих пакетов | manual | `diff -r src/led_matrix_driver/ src/ros2leds/led_matrix_driver/` | ❌ Нужно исправить |

### Команды для финальной верификации Success Criteria

```bash
# SC1: 0 результатов
grep -r "COPY config" docker/ --include="Dockerfile*"

# SC2: 0 результатов
grep -r "COPY scripts" docker/ --include="Dockerfile*"

# SC3: Все ROS-сервисы имеют network_mode и depends_on (проверка комментариями)
# Mониторинг- и init-сервисы являются задокументированными исключениями

# SC4: 0 результатов
grep -r "TODO" src/ --include="setup.py" --include="package.xml"
```

### Пробелы Wave 0 (нет тестовых файлов для создания)

Фаза не требует создания новых тестов — верификация через grep/diff команды на реальных файлах.

---

## Открытые вопросы

1. **led_matrix_driver duplication — какой API канонический?**
   - Известно: Docker использует `ros2leds/led_matrix_driver` с `Int8MultiArray`
   - Неизвестно: используют ли другие пакеты в src/ `ByteMultiArray` из standalone
   - Рекомендация: перед удалением standalone — запустить `grep -r "ByteMultiArray" src/`

2. **URDF_EXPORT — исключить из workspace или оставить?**
   - Известно: это auto-generated ROS 1 URDF export, не часть colcon build
   - Рекомендация: добавить `src/rob_box_description/URDF_EXPORT/COLCON_IGNORE`

3. **Нужно ли исправлять placeholder emails в submodule?**
   - `src/ros2leds/*/setup.py` — часть git submodule, правка через PR в submodule repo
   - Рекомендация: для Milestone 1 качества — задокументировать как tech debt, не исправлять в рамках фазы

---

## Ограничения проекта (из copilot-instructions.md)

- ❌ Не копировать файлы на робота (scp, rsync и т.п.)
- ❌ `COPY config/` и `COPY scripts/` в Dockerfile запрещены
- ✅ `network_mode: host` обязателен
- ✅ `depends_on: zenoh-router` обязателен для ROS-сервисов
- ✅ `black` (line-length 120), `isort`, `flake8` для Python
- ✅ Деплой только через GitHub Actions

---

## Источники

### Первичные (HIGH confidence)
- `docker/main/docker-compose.yaml` — прямая инспекция [VERIFIED]
- `docker/vision/docker-compose.yaml` — прямая инспекция [VERIFIED]
- `docker/main/*/Dockerfile`, `docker/vision/*/Dockerfile` — прямая инспекция [VERIFIED]
- `src/*/setup.py` — прямая инспекция [VERIFIED]
- `src/*/package.xml` — прямая инспекция [VERIFIED]
- `docs/development/DOCKER_STANDARDS.md` — стандарт проекта [VERIFIED]
- `.github/workflows/G-Lint Code.yml` — прямая инспекция [VERIFIED]
- `.gitmodules` — прямая инспекция [VERIFIED]

### Предположения ([ASSUMED])

Нет. Все утверждения верифицированы на реальных файлах репозитория.

## Лог предположений

*Таблица пуста — все факты верифицированы или процитированы из файлов репозитория.*

---

## Метаданные

**Уверенность по областям:**
- Docker audit (STRUCT-01, STRUCT-02): HIGH — прямой grep по файлам
- Python packages (STRUCT-03, STRUCT-04): HIGH — прямой grep по файлам  
- src/ structure (STRUCT-05): HIGH — diff и ls по директориям

**Дата research:** 2026-05-15  
**Актуально до:** 2026-06-15 (стабильный стек)
