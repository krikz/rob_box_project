# Phase 05: Миграция ROS 2 Humble → Lyrical Luth — Research

**Researched:** 2026-07-27
**Domain:** ROS 2 distro migration (Humble→Lyrical), Docker, CI/CD, embedded ARM64
**Confidence:** HIGH (verified via live container inspection)

## Summary

Миграция с ROS 2 Humble (Ubuntu 22.04) на ROS 2 Lyrical Luth (Ubuntu 26.04 Resolute) технически возможна, но имеет **один критический архитектурный сюрприз**: Navigation2 (Nav2) **полностью удалён** из дистрибутива Lyrical и заменён на EasyNavigation (NavMap). Это не просто смена названий пакетов — это смена навигационного фреймворка, требующая переписывания Dockerfile и адаптации конфигурации Nav2.

Остальные изменения ожидаемы: Python 3.10→3.14 с `--break-system-packages`, переименование `behaviortree-cpp-v3`→`behaviortree-cpp`, добавление `python3-pip` (отсутствует по умолчанию). DepthAI остаётся блокером (нужна сборка из исходников). Zenoh-архитектура не меняется.

**Primary recommendation:** Разбить миграцию на 2 волны: (1) всё кроме Nav2 и depthai — быстро, механически; (2) Nav2-переход на EasyNavigation + depthai-сборка — исследовательски, итеративно.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| ROS 2 base image | Docker base | CI/CD | `FROM ros:lyrical-ros-base` — единая точка входа |
| Navigation (Nav2→EasyNav) | Docker main | Source build | Nav2 удалён из Lyrical, EasyNav — apt-пакеты + source |
| DepthAI/OAK-D | Docker base (новый) | Source build | `luxonis/depthai-ros` не имеет lyrical-тегов |
| Zenoh middleware | Docker base + router | APT external | `ros-lyrical-rmw-zenoh-cpp` — отдельный пакет |
| RTAB-Map SLAM | Docker base | Upstream image | `introlab3it/rtabmap_ros:lyrical-latest` ✅ |
| Voice assistant | Docker vision | Python 3.14 pip | `--break-system-packages` для vosk, silero, openai |
| LED matrix | Docker vision | System SPI | Системные пакеты resolute |
| CI/CD pipelines | GitHub Actions | Workflow env | `ROS_DISTRO: lyrical` во всех workflow |

## User Constraints (from CONTEXT.md)

### Locked Decisions
- **D-01:** Humble→Lyrical напрямую от `develop` (не через Kilted)
- **D-02:** DepthAI — свой Dockerfile (сборка `depthai-core` из `main`)
- **D-03:** Zenoh — внешний пакет `ros-lyrical-rmw-zenoh-cpp` (v0.10.4), архитектура zenoh-router не меняется
- **D-04:** Lyrical подтверждён (Ubuntu 26.04 ✅, ROS ✅, 2154 пакета, rtabmap ✅)
- **D-05:** Приоритет — перед Milestone 2 (навигация должна разрабатываться на целевом дистрибутиве)
- **D-06:** База ветки — `develop` (Humble), не `feature/kilted`

### Deferred Ideas (OUT OF SCOPE)
- EventsCBGExecutor / AsyncNode (оптимизация после сборки)
- URDF 1.2 (отдельная задача)
- Bag circular recording (после миграции)
- Удаление zenoh-router (до нативной Zenoh-интеграции в ROS 2)

## 🚨 CRITICAL FINDING: Navigation2 Removed from Lyrical

**Verified:** `ros-lyrical` Docker image + rosdistro distribution.yaml + apt-cache

Navigation2 **отсутствует** в ROS 2 Lyrical. В rosdistro Lyrical 0 записей `navigation2` (против 7 в Humble). В apt только 3 minimal simulation пакета:
- `ros-lyrical-nav2-minimal-tb3-sim`
- `ros-lyrical-nav2-minimal-tb4-description`
- `ros-lyrical-nav2-minimal-tb4-sim`

**Замена:** EasyNavigation — новый навигационный фреймворк, включённый в Lyrical:
- `NavMap` (v0.4.0-3): `navmap_core`, `navmap_ros`, `navmap_ros_interfaces`, `navmap_rviz_plugin`
- `easynav` (v0.4.0): `easynav`, `easynav_common`, `easynav_controller`, `easynav_core`, `easynav_interfaces`, `easynav_localizer`, `easynav_maps_manager`
- `easynav_plugins`
- `SMACC2` — state machine для навигации

**Последствия для проекта:**
- `docker/main/nav2/Dockerfile` — **полностью переписать** (20+ `ros-humble-nav2-*` пакетов → EasyNav)
- Навигационные конфиги (`nav2_params.yaml`, etc.) — требуют адаптации под EasyNav API
- Запускные скрипты (`start_nav2_direct.sh`) — переписать
- **Решение EasyNav vs source-build nav2 требует обсуждения с пользователем**

**Варианты решения (на обсуждение):**

| Вариант | Плюсы | Минусы | Оценка |
|---------|-------|--------|--------|
| A: Перейти на EasyNavigation | Нативный для Lyrical, apt-пакеты, поддерживается | v0.4.0 — незрелый, другой API, риски стабильности | 🔴 Высокий риск |
| B: Собрать navigation2 из исходников | Знакомый API, зрелый код | Нет lyrical-ветки, ручная сборка, не тестировался на Resolute | 🟡 Средний риск |
| C: Временно оставить nav2 на humble-базе | Быстро, без изменений кода | Два дистрибутива, defeats purpose D-05 | 🔴 Технический долг |
| D: Гибрид: EasyNav для простого + nav2 source для сложного | Максимальная гибкость | Максимальная сложность | 🔴 Переусложнение |

**Рекомендация исследователя:** Вариант B (source-build navigation2). EasyNavigation v0.4.0 слишком незрел для production-робота. Navigation2 — зрелый фреймворк с известным поведением. Сборка из исходников на `ros:lyrical-ros-base` должна работать (зависимости: behaviortree-cpp, nav_msgs, tf2 — все доступны в Lyrical как apt-пакеты).

## Standard Stack

### Core — ROS 2 Lyrical

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| ROS 2 Lyrical Luth | May 2026 | Робототехнический фреймворк | LTS до 2031, целевой дистрибутив |
| Ubuntu Resolute | 26.04 | Базовая ОС | Требование Lyrical |
| Python | 3.14.4 | Runtime | Системный Python в Resolute |
| pip | 25.1.1 | Package manager | Устанавливается через `python3-pip` |
| GCC | 15.2.0 | C++ compiler | Системный компилятор Resolute |

### ROS 2 Package Name Changes (Verified)

| Humble Package | Lyrical Package | Status |
|---------------|-----------------|--------|
| `ros-humble-rmw-zenoh-cpp` | `ros-lyrical-rmw-zenoh-cpp` | ✅ Доступен (v0.10.4) |
| `ros-humble-behaviortree-cpp-v3` | `ros-lyrical-behaviortree-cpp` | ⚠️ Переименован (без -v3) |
| `ros-humble-theora-image-transport` | `ros-lyrical-theora-image-transport` | ✅ Доступен |
| `ros-humble-compressed-image-transport` | `ros-lyrical-compressed-image-transport` | ✅ Доступен |
| `ros-humble-image-transport-plugins` | `ros-lyrical-image-transport-plugins` | ✅ Доступен |
| `ros-humble-control-msgs` | `ros-lyrical-control-msgs` | ✅ Доступен |
| `ros-humble-apriltag-ros` | `ros-lyrical-apriltag-ros` | ✅ Доступен |
| `ros-humble-cv-bridge` | `ros-lyrical-cv-bridge` | ✅ Доступен |
| `ros-humble-pcl-conversions` | `ros-lyrical-pcl-conversions` | ✅ Доступен |
| `ros-humble-pcl-ros` | `ros-lyrical-pcl-ros` | ✅ Доступен |
| `ros-humble-twist-mux` | `ros-lyrical-twist-mux` | ✅ Доступен |
| `ros-humble-depthimage-to-laserscan` | `ros-lyrical-depthimage-to-laserscan` | ✅ Доступен |
| `ros-humble-robot-state-publisher` | `ros-lyrical-robot-state-publisher` | ✅ Доступен |
| `ros-humble-diagnostic-updater` | `ros-lyrical-diagnostic-updater` | ✅ Доступен |
| `ros-humble-rosidl-default-generators` | `ros-lyrical-rosidl-default-generators` | ✅ Доступен |
| `ros-humble-ament-cmake` | `ros-lyrical-ament-cmake` | ✅ Доступен |
| `ros-humble-nav2-*` (20+ пакетов) | **УДАЛЕНЫ** | 🚨 Заменены на EasyNavigation |

### Supporting — System Packages

| Package | Purpose | Status in Resolute |
|---------|---------|--------------------|
| `python3-pip` | pip installer | ⚠️ **НЕ предустановлен** — нужен явный `apt-get install` |
| `python3-dev` | Python headers | ✅ Доступен |
| `python3-colcon-common-extensions` | Build tool | ✅ Доступен |
| `libusb-1.0-0-dev` | USB (OAK-D, LIDAR) | ✅ Доступен |
| `libgpiod-dev` | GPIO | ✅ Доступен |
| `i2c-tools`, `libi2c-dev` | I2C | ✅ Доступен |
| `spi-tools` | SPI (LED) | ✅ Доступен |

### Alternatives Considered

| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| EasyNavigation (NavMap) | Source-build navigation2 | EasyNav — нативный, но v0.4.0 (незрелый); nav2 — зрелый, но без официальной поддержки Lyrical |
| `luxonis/depthai-ros:lyrical` | Свой Dockerfile.depthai | Официального образа нет; свой Dockerfile даёт контроль, но требует поддержки |
| `ros:lyrical-ros-base` | `ros:lyrical-perception` | perception включает OpenCV/PCL предустановленными — меньше apt-шагов |
| Прямая замена humble→lyrical в nav2 | Гибрид humble-base для nav2 | Временное решение, нарушает D-05 |

## Architecture Patterns

### System Architecture — Migration Impact Map

```
┌─────────────────────────────────────────────────────────────────┐
│                    DOCKER BASE IMAGES                            │
│  ┌──────────────┐  ┌──────────┐  ┌──────────┐  ┌────────────┐  │
│  │ ros2-zenoh   │  │ rtabmap  │  │ depthai  │  │    pcl     │  │
│  │ (humble→     │  │ (humble→ │  │ (БЛОКЕР) │  │ (humble→   │  │
│  │  lyrical) ✅ │  │ lyrical)✅│  │ 🚨       │  │  lyrical)✅│  │
│  └──────┬───────┘  └────┬─────┘  └────┬─────┘  └─────┬──────┘  │
│         │               │             │               │         │
├─────────┼───────────────┼─────────────┼───────────────┼─────────┤
│         ▼               ▼             ▼               ▼         │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              MAIN PI SERVICES (12 Dockerfiles)            │   │
│  │  nav2🚨 rtabmap perception vesc_nexus ros2_control       │   │
│  │  teleop twist_mux micro_ros_agent lslidar                │   │
│  │  robot_state_publisher zenoh-router                      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │             VISION PI SERVICES (9 Dockerfiles)            │   │
│  │  oak-d🚨 apriltag ceiling-camera led_matrix              │   │
│  │  voice_assistant voice_base voice_resources               │   │
│  │  telegram_bot supercollider zenoh-router                  │   │
│  └──────────────────────────────────────────────────────────┘   │
├──────────────────────────────────────────────────────────────────┤
│  CI/CD (19 workflows) + .env (9 files) + scripts (30+)          │
│  Все ссылаются на humble → заменить на lyrical                   │
└──────────────────────────────────────────────────────────────────┘
```

### Pattern 1: Mechanical Find-and-Replace (80% файлов)
**What:** Массовая замена `humble`→`lyrical`, `jammy`→`resolute`, `22.04`→`26.04` во всех Dockerfile'ах, .env, скриптах, CI/CD workflow.
**When to use:** Все сервисы КРОМЕ nav2 и depthai.
**Example:**
```dockerfile
# Before
FROM ros:humble-ros-base
RUN apt-get install -y ros-humble-rmw-zenoh-cpp
ENV LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib

# After
FROM ros:lyrical-ros-base
RUN apt-get install -y ros-lyrical-rmw-zenoh-cpp
ENV LD_LIBRARY_PATH=/opt/ros/lyrical/opt/zenoh_cpp_vendor/lib:/opt/ros/lyrical/lib/aarch64-linux-gnu:/opt/ros/lyrical/lib
```

### Pattern 2: Python 3.14 pip Adaptation
**What:** Добавить `python3-pip` в apt-зависимости, использовать `--break-system-packages` для всех `pip install`.
**When to use:** Все Dockerfile'ы с `pip install` (voice_assistant, voice_base, perception, telegram_bot, test).
**Example:**
```dockerfile
# Before (Humble)
RUN pip3 install numpy openai vosk

# After (Lyrical)
RUN apt-get install -y python3-pip && \
    pip3 install --break-system-packages numpy openai vosk
```

### Pattern 3: Source-Build Navigation2 (если выбран вариант B)
**What:** Клонировать `ros-navigation/navigation2`, собрать через colcon.
**When to use:** Замена `docker/main/nav2/Dockerfile`.
**Example:**
```dockerfile
FROM ros:lyrical-ros-base
RUN apt-get update && apt-get install -y \
    ros-lyrical-behaviortree-cpp \
    ros-lyrical-nav-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /ws/src && cd /ws/src && \
    git clone --branch 1.3.12 https://github.com/ros-navigation/navigation2.git
RUN cd /ws && rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/lyrical/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Pattern 4: Custom depthai-ros Image
**What:** Собрать `depthai-ros` из исходников на базе `ros:lyrical-ros-base`.
**When to use:** Замена `docker/base/Dockerfile.depthai` и `docker/vision/oak-d/Dockerfile`.
**Reference:** Полный Dockerfile в `docs/plans/lyrical-migration-plan.md` (секция 2.2, вариант C).

### Anti-Patterns to Avoid
- **Слепой sed humble→lyrical:** `behaviortree-cpp-v3` переименован в `behaviortree-cpp` (без -v3). Простой sed сломает имя пакета.
- **Копирование kilted-ветки:** 297 файлов, но для Lyrical другая структура пакетов (nav2 удалён).
- **pip install без --break-system-packages:** Python 3.14 в Resolute использует EXTERNALLY-MANAGED (PEP 668). Без флага pip откажется устанавливать пакеты.
- **Забыть про python3-pip:** В `ros:lyrical-ros-base` pip НЕ предустановлен (в отличие от humble).
- **Игнорировать Nav2 removal:** Просто заменить `ros-humble-nav2-*` на `ros-lyrical-nav2-*` не сработает — пакетов нет.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Навигация с нуля | Писать свой планировщик | EasyNavigation (Lyrical-native) или source-build nav2 | Годы разработки, нестабильность |
| DepthAI сборка | Ждать Luxonis | Свой Dockerfile на базе `ros:lyrical-ros-base` | Контроль над сборочным процессом |
| Python package management | pip в систему | `--break-system-packages` или venv | Resolute enforced PEP 668 |
| Поиск замены `theora-image-transport` | Переписывать image pipeline | Theora ВСЁ ЕЩЁ доступен в Lyrical | Не нужно менять |
| APT-cacher-ng | Переписывать конфиг с нуля | Добавить `resolute` в Remap-строки | Существующий конфиг покрывает 80% |

## Runtime State Inventory

> Миграция — это rename-фаза (humble→lyrical, jammy→resolute).

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | RTAB-Map maps в `docker/main/maps/` — не содержат humble-строк | ✅ Без изменений |
| Live service config | GitHub Actions secrets/variables — не содержат humble в именах | ✅ Без изменений |
| OS-registered state | systemd-сервисы на Pi: `docker-compose@*.service` — не содержат humble | ✅ Без изменений |
| Secrets/env vars | `.env.secrets` — только API-ключи, без humble-строк | ✅ Без изменений |
| Build artifacts | Docker images в GHCR: `*-humble-latest`, `*-humble-test`, `*-humble-dev` | 🔄 Пересборка с lyrical-тегами (старые humble-теги сохранить для отката) |
| Container names | `oak-d`, `voice-assistant`, etc. — не содержат humble | ✅ Без изменений |

**Nothing found in runtime state requiring data migration.** Все изменения — code-only (Dockerfile, скрипты, конфиги).

## Common Pitfalls

### Pitfall 1: Nav2 Removal — Незамеченный Блокер
**What goes wrong:** План предполагает простую замену `ros-humble-nav2-*`→`ros-lyrical-nav2-*`, но пакетов нет. CI падает с `E: Unable to locate package`.
**Why it happens:** Nav2 удалён из Lyrical rosdistro. Заменён на EasyNavigation.
**How to avoid:** Решить стратегию (EasyNav vs source-build) ДО начала массовых замен.
**Warning signs:** `apt-cache search ros-lyrical-nav2` возвращает только 3 simulation пакета.

### Pitfall 2: `behaviortree-cpp-v3` → `behaviortree-cpp`
**What goes wrong:** `sed 's/humble/lyrical/g'` создаст `ros-lyrical-behaviortree-cpp-v3`, которого нет.
**Why it happens:** Пакет переименован без суффикса `-v3` в Lyrical.
**How to avoid:** Отдельная замена для `behaviortree-cpp-v3`→`behaviortree-cpp`.

### Pitfall 3: pip --break-system-packages
**What goes wrong:** `pip install` падает с `error: externally-managed-environment`.
**Why it happens:** Python 3.14 в Resolute включает PEP 668 (EXTERNALLY-MANAGED).
**How to avoid:** Всегда использовать `--break-system-packages` или `pipx`. Проверено: флаг работает, numpy устанавливается.

### Pitfall 4: Отсутствие python3-pip
**What goes wrong:** `pip3: command not found` в Docker-сборке.
**Why it happens:** В `ros:lyrical-ros-base` pip НЕ предустановлен (в отличие от `ros:humble-ros-base`).
**How to avoid:** Добавить `python3-pip` в apt-зависимости ВСЕХ Dockerfile'ов, использующих pip.

### Pitfall 5: APT-cacher-ng и Resolute
**What goes wrong:** APT-cacher-ng не кэширует/блокирует запросы к resolute-репозиториям.
**Why it happens:** Конфиг `acng.conf` содержит `Remap-uburep: http://archive.ubuntu.com/ubuntu` — это работает для всех релизов Ubuntu. Resolute использует тот же `archive.ubuntu.com`.
**How to avoid:** **Проверено:** стандартный `archive.ubuntu.com` обслуживает Resolute. APT-cacher-ng должен работать без изменений. Дополнительно добавить явный `Remap-rosrep: http://packages.ros.org/ros2/ubuntu` для lyrical (уже есть).

### Pitfall 6: LD_LIBRARY_PATH с Zenoh
**What goes wrong:** `LD_LIBRARY_PATH` указывает на `/opt/ros/humble/...`.
**Why it happens:** Механическая замена humble→lyrical исправляет путь, но структура `zenoh_cpp_vendor` может измениться.
**How to avoid:** Проверить актуальный путь в `ros:lyrical-ros-base` после установки `ros-lyrical-rmw-zenoh-cpp`.

### Pitfall 7: `runs-on: ubuntu-22.04` в CI
**What goes wrong:** GitHub Actions тесты падают — `ubuntu-22.04` не имеет lyrical-пакетов.
**Why it happens:** `G-Run Tests.yml` использует `runs-on: ubuntu-22.04` + `setup-ros` с `required-ros-distributions: humble`.
**How to avoid:** Изменить на `ubuntu-26.04` (если доступен в GA) или `ubuntu-latest` + `required-ros-distributions: lyrical`.

## Code Examples

### Шаблон: Dockerfile для сервиса на Lyrical
```dockerfile
# Source: verified on ros:lyrical-ros-base (Docker Hub, updated 2026-07-16)
FROM ros:lyrical-ros-base

# pip НЕ предустановлен — добавляем явно
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-lyrical-rmw-zenoh-cpp \
    # ... другие apt-зависимости ...
    && rm -rf /var/lib/apt/lists/*

# pip install ВСЕГДА с --break-system-packages
RUN pip3 install --break-system-packages \
    numpy \
    openai \
    # ... другие pip-зависимости ...

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    LD_LIBRARY_PATH=/opt/ros/lyrical/opt/zenoh_cpp_vendor/lib:/opt/ros/lyrical/lib/aarch64-linux-gnu:/opt/ros/lyrical/lib
```

### Шаблон: .env для Lyrical
```bash
# Before
ROS_DISTRO=humble
BASE_IMAGE_TAG=humble-latest

# After
ROS_DISTRO=lyrical
BASE_IMAGE_TAG=lyrical-latest
```

### Шаблон: CI/CD workflow для Lyrical
```yaml
env:
  ROS_DISTRO: lyrical  # Было: humble

# В setup-ros:
- uses: ros-tooling/setup-ros@v0.7
  with:
    required-ros-distributions: lyrical  # Было: humble
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `ros-humble-nav2-*` (20+ apt пакетов) | EasyNavigation (NavMap/easynav) или source-build | May 2026 (Lyrical) | Полное переписывание nav2 Dockerfile |
| `ros-humble-behaviortree-cpp-v3` | `ros-lyrical-behaviortree-cpp` | May 2026 | Переименование, несовместимость sed |
| pip в системе без ограничений | PEP 668 EXTERNALLY-MANAGED | Python 3.14 / Resolute | `--break-system-packages` обязателен |
| pip предустановлен в ros-base | pip нужно ставить через apt | Lyrical | `python3-pip` в apt-зависимостях |
| `runs-on: ubuntu-22.04` | `runs-on: ubuntu-latest` (24.04/26.04) | Q2 2026 | CI runner OS upgrade |

**Deprecated/outdated:**
- `ros-humble-behaviortree-cpp-v3`: заменён на `ros-lyrical-behaviortree-cpp` (v4+)
- `ros-humble-nav2-*`: весь стек удалён из Lyrical
- `luxonis/depthai-ros:humble-*`: для Lyrical нужна сборка из исходников

## File Impact Analysis

### Категории файлов (на основе анализа kilted-ветки: 297 files, +1294/-1259)

| Категория | Кол-во | Тип изменений | Сложность |
|-----------|--------|---------------|-----------|
| **Dockerfile'ы** | 25 | `FROM ros:humble`→`ros:lyrical`, `ros-humble-*`→`ros-lyrical-*`, `LD_LIBRARY_PATH`, `pip install --break-system-packages`, `python3-pip` | 🟡 Средняя |
| **.env файлы** | 9 | `ROS_DISTRO=lyrical`, теги `*-lyrical-*` | 🟢 Низкая |
| **CI/CD workflows** | 19 | `ROS_DISTRO: lyrical`, `ros-humble-*`→`ros-lyrical-*`, `ubuntu-22.04`→`ubuntu-latest` | 🟡 Средняя |
| **Shell-скрипты** | ~30 | `source /opt/ros/humble/setup.bash`→`/opt/ros/lyrical/` | 🟢 Низкая |
| **Python-файлы (src/)** | ~5 | Комментарии "ROS 2 Humble", `setup.cfg` | 🟢 Низкая |
| **Конфиги** | ~10 | `humble` в описаниях, путях | 🟢 Низкая |
| **Документация** | ~20 | `humble`→`lyrical` в docs/ | 🟢 Низкая |
| **APT-cacher-ng** | 1 | Добавить `resolute` маппинг (опционально) | 🟢 Низкая |
| **docker-compose.yaml** | 2 | Теги образов | 🟢 Низкая |
| **package.xml** | ~8 | `<description>` обновления | 🟢 Низкая |

### Ключевые Dockerfile'ы, требующие особого внимания

| Dockerfile | Риск | Причина |
|-----------|------|--------|
| `docker/main/nav2/Dockerfile` | 🔴 КРИТИЧЕСКИЙ | Nav2 удалён из Lyrical — полное переписывание |
| `docker/base/Dockerfile.depthai` | 🔴 КРИТИЧЕСКИЙ | Сборка из исходников (vcpkg, depthai-core) |
| `docker/vision/oak-d/Dockerfile` | 🟡 Средний | Смена базового образа на свой depthai |
| `docker/vision/voice_base/Dockerfile` | 🟡 Средний | pip пакеты: vosk, silero, openai, numpy — совместимость с Python 3.14 |
| `docker/vision/voice_assistant/Dockerfile` | 🟡 Средний | `--break-system-packages`, `python3-pip` |
| `docker/base/Dockerfile.ros2-zenoh` | 🟢 Низкий | Механическая замена |
| `docker/base/Dockerfile.rtabmap` | 🟢 Низкий | `FROM introlab3it/rtabmap_ros:lyrical-latest` ✅ |
| `docker/base/Dockerfile.pcl` | 🟢 Низкий | Механическая замена |
| `docker/main/perception/Dockerfile` | 🟡 Средний | `control-msgs` (проверить доступность), pip |
| `docker/main/vesc_nexus/Dockerfile` | 🟢 Низкий | Механическая замена |
| `docker/vision/led_matrix/Dockerfile` | 🟢 Низкий | SPI-пакеты системные, не ROS-specific |

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Docker | Все сборки | ✓ | 24.0.7+ | — |
| `ros:lyrical-ros-base` | Все образы | ✓ | latest (16.07.2026) | — |
| `introlab3it/rtabmap_ros:lyrical-latest` | rtabmap | ✓ | lyrical-latest | — |
| `luxonis/depthai-ros:lyrical` | oak-d | ✗ | — | Свой Dockerfile (D-02) |
| GitHub Actions `ubuntu-latest` | CI/CD | ✓ | 24.04 | `ubuntu-24.04` явно |
| GitHub Actions ARM64 runner | CI/CD | ✓ | BuildJet / self-hosted | `self-hosted` label |
| APT-cacher-ng | Кэширование сборок | ✓ | 3.7.5 | Без кэша (медленнее) |
| python3-pip (в образе) | pip-зависимости | ✗ (по умолчанию) | 25.1.1 | `apt-get install python3-pip` |
| `ros-lyrical-rmw-zenoh-cpp` | Zenoh | ✓ | 0.10.4 | — |
| `ros-lyrical-navigation2` | Nav2 | ✗ | — | EasyNavigation или source-build |
| `ros-lyrical-behaviortree-cpp` | Nav2, perception | ✓ | latest | — |
| vcpkg | depthai-core сборка | ⚠️ Не проверен | — | Системные библиотеки вместо vcpkg |
| GitHub Actions `ubuntu-26.04` | CI тесты | ⚠️ Не проверен | — | `ubuntu-24.04` + docker |

**Missing dependencies with no fallback:**
- `luxonis/depthai-ros:lyrical` — **блокирует oak-d**, решается через D-02 (свой Dockerfile)

**Missing dependencies with fallback:**
- `ros-lyrical-navigation2` — **блокирует nav2**, решается через EasyNavigation или source-build
- `python3-pip` в образе — решается добавлением в apt-зависимости
- `ubuntu-26.04` в GitHub Actions — `ubuntu-24.04` + Docker для сборки, тесты внутри контейнера

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest (ROS 2 launch_testing + pytest) |
| Config file | `setup.cfg` (в каждом пакете) |
| Quick run command | `colcon test --packages-select rob_box_perception --ctest-args -V` |
| Full suite command | `colcon test --ctest-args -V` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| MIGRATE-01 | Все Docker-образы собираются | smoke | `docker compose -f docker/main/docker-compose.yaml build` | ✅ (wave 0: CI workflow) |
| MIGRATE-02 | ROS 2 пакеты импортируются | unit | `colcon test --packages-select rob_box_perception` | ✅ частично |
| MIGRATE-03 | Zenoh connect | integration | `ros2 topic list` в контейнере | ❌ Wave 0 |
| MIGRATE-04 | pip-пакеты устанавливаются | smoke | `pip3 install --break-system-packages --dry-run ...` | ❌ Wave 0 |
| MIGRATE-05 | Nav2/EasyNav запускается | integration | `ros2 launch nav2_bringup ...` (manual) | ❌ Wave 0 |
| MIGRATE-06 | depthai-ros собирается | smoke | `colcon build --packages-select depthai-ros` | ❌ Wave 0 |

### Sampling Rate
- **Per task commit:** CI build изменённого сервиса (`L-Build Single Service`)
- **Per wave merge:** `L-Build All Services` (локальный runner)
- **Phase gate:** `G-Build All Services` green + все образы в GHCR

### Wave 0 Gaps
- [ ] `.github/workflows/G-Build Lyrical Services.yml` — новый workflow для lyrical-сборки
- [ ] `tests/test_lyrical_migration.py` — smoke-тест: импорт ROS 2 пакетов под Lyrical
- [ ] `tests/test_pip_resolute.py` — smoke-тест: pip install с `--break-system-packages`

## Security Domain

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | No | — |
| V3 Session Management | No | — |
| V4 Access Control | No | — |
| V5 Input Validation | No (code/config migration only) | — |
| V6 Cryptography | No | — |

**Security note:** Миграция — это замена дистрибутива, не добавление новых функций. Стандартные ROS 2 security-паттерны сохраняются. Основной риск — устаревшие CVE в Resolute (новый дистрибутив, меньше исправлений безопасности чем в Jammy).

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `LD_LIBRARY_PATH` для Zenoh сохраняет структуру `/opt/ros/lyrical/opt/zenoh_cpp_vendor/lib` | Code Examples | Сборка падает на LD_PRELOAD ошибках (Medium) |
| A2 | vcpkg собирается на Ubuntu 26.04 (GCC 15.2.0) | Don't Hand-Roll | Блокирует depthai-ros сборку (High) |
| A3 | `ros-lyrical-compressed-depth-image-transport` существует (не проверен явно, но `compressed-image-transport` есть) | Standard Stack | Мелкий фикс в Dockerfile (Low) |
| A4 | GitHub Actions поддерживает `ubuntu-26.04` или `ubuntu-latest` достаточно для Docker-сборки | Environment Availability | Нужен self-hosted runner (Medium) |
| A5 | `navigation2` собирается из исходников на `ros:lyrical-ros-base` без патчей | Critical Finding | Нужен патч апстрим, задержка (High) |
| A6 | `introlab3it/rtabmap_ros:lyrical-latest` совместим с нашим rtabmap-конфигом | File Impact | Мелкие правки конфига (Low) |
| A7 | Python-пакеты (vosk, silero, openai, numpy) имеют wheels для Python 3.14 / arm64 | File Impact | Блокирует voice-сервисы (Medium) |

## Open Questions

1. **Navigation2 vs EasyNavigation — стратегический выбор**
   - What we know: Nav2 удалён из Lyrical, EasyNav v0.4.0 — замена
   - What's unclear: Готов ли EasyNav для production (v0.4.0), совместим ли API с нашими конфигами
   - Recommendation: Обсудить с пользователем; recommendation research — source-build nav2 как менее рискованный

2. **Сборка navigation2 из исходников на Lyrical**
   - What we know: Зависимости (behaviortree-cpp, nav_msgs, tf2) доступны в Lyrical
   - What's unclear: Соберётся ли без патчей, какие флаги cmake нужны
   - Recommendation: Первый же spike в feature/lyrical — попытка сборки `navigation2` через colcon

3. **vcpkg на Ubuntu 26.04 (GCC 15.2.0)**
   - What we know: depthai-core зависит от vcpkg для ARM64
   - What's unclear: Совместимость vcpkg с GCC 15.2.0, новыми системными библиотеками Resolute
   - Recommendation: Протестировать сборку depthai-core в `ros:lyrical-ros-base` контейнере

4. **Python 3.14 wheel availability для ARM64**
   - What we know: vosk, silero, openai, numpy — ключевые зависимости голосового ассистента
   - What's unclear: Есть ли pre-built wheels для Python 3.14 на linux/arm64
   - Recommendation: Проверить `pip install --break-system-packages --dry-run` для каждого пакета

5. **GitHub Actions ubuntu-26.04 runner**
   - What we know: Сейчас используется `ubuntu-22.04` + `setup-ros`
   - What's unclear: Доступен ли `ubuntu-26.04` в GitHub Actions (вышел 23.04.2026, прошло 3 месяца)
   - Recommendation: Проверить доступность; fallback — `ubuntu-24.04` + Docker для сборки

## Sources

### Primary (HIGH confidence — verified via live container inspection)
- `ros:lyrical-ros-base` (Docker Hub, updated 2026-07-16) — Python 3.14.4, pip 25.1.1, EXTERNALLY-MANAGED, package availability
- `ros:lyrical` (Docker Hub) — идентичен ros-base (200 packages in share)
- `https://raw.githubusercontent.com/ros/rosdistro/master/lyrical/distribution.yaml` — 701 repositories, 0 navigation2 entries, NavMap + easynav present
- `https://raw.githubusercontent.com/ros/rosdistro/master/humble/distribution.yaml` — 7 navigation2 entries (comparison)
- `https://github.com/ros-navigation/navigation2` — latest release 1.3.12 (April 2026), no lyrical branches

### Secondary (MEDIUM confidence — cross-referenced)
- `https://hub.docker.com/r/luxonis/depthai-ros` — no lyrical tags (confirmed via Docker Hub)
- `https://hub.docker.com/r/introlab3it/rtabmap_ros` — lyrical-latest tag exists (confirmed via Docker Hub)
- `docs/plans/lyrical-migration-plan.md` — existing analysis, corrected by this research
- `origin/feature/kilted` (13 commits, 297 files) — reference for scope of changes

### Tertiary (LOW confidence — needs validation)
- vcpkg compatibility with GCC 15.2.0 on ARM64 — not tested [ASSUMED A2]
- EasyNavigation v0.4.0 production readiness — not evaluated [ASSUMED]
- Python 3.14 wheel availability for vosk, silero on ARM64 — not tested [ASSUMED A7]

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — verified via live `ros:lyrical-ros-base` container + apt-cache + rosdistro
- Architecture: HIGH — nav2 removal confirmed via 3 independent sources (apt, rosdistro, container inspection)
- Pitfalls: HIGH — Python 3.14 pip behavior verified, package renames verified
- File impact: MEDIUM — based on kilted reference (297 files), exact count for Lyrical may vary

**Research date:** 2026-07-27
**Valid until:** 2026-08-27 (Lyrical stable, но возможны обновления пакетов)

**Key discovery:** Navigation2 removed from Lyrical — this is the single most impactful finding that reshapes the entire migration strategy.
