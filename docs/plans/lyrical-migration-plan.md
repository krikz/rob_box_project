# План миграции: ROS 2 Kilted Kaiju → ROS 2 Lyrical Luth

> **Статус:** 📋 План  
> **Ветка:** `feature/lyrical` ← `develop`  
> **Дата:** 2026-07-27  
> **Автор:** AI-assisted planning

---

## 1. Обзор Lyrical Luth

| Параметр | Kilted Kaiju | Lyrical Luth |
|----------|-------------|--------------|
| **Дата релиза** | Май 2025 | Май 2026 |
| **Ubuntu** | 24.04 Noble Numbat | **26.04 Resolute** ⚠️ |
| **LTS** | Нет (non-LTS) | **Да, до мая 2031** |
| **Python (системный)** | 3.12 | **3.14** (предположительно) |
| **Zenoh** | Внешний middleware (`rmw_zenoh_cpp`) | **Встроенная поддержка**, развивается |

### Ключевые новые фичи Lyrical
- **EventsCBGExecutor** — 10-15% меньше CPU чем MultithreadedExecutor
- **AsyncNode** — asyncio в rclpy (значительно меньше CPU)
- **rosidl::Buffer** — zero-copy publish/subscribe (поддержка Zenoh в разработке)
- **YAML type annotations** — `!!str`, `!!bool`, `!!int`, `!!float` в параметрах
- **URDF 1.2** — quaternions, capsule geometry, accel/decel/jerk limits
- **robot_state_publisher** — может читать `robot_description` из топика
- **Bag recording** — circular buffers, Python API, message-loss observability
- **Resource retriever service** — загрузка мешей по сети
- **Per-message log severity** в launch файлах
- **fish shell** поддержка

---

## 2. Анализ воздействия на проект

### 2.1 Критические изменения

| Область | Kilted | Lyrical | Сложность |
|--------|--------|---------|-----------|
| **ОС** | Ubuntu 24.04 Noble | Ubuntu 26.04 Resolute | 🔴 Высокая |
| **Python** | 3.12 | 3.14 (предп.) | 🔴 Высокая |
| **Системные пакеты** | `noble` | `resolute` (новые репы) | 🟡 Средняя |
| **ROS пакеты** | `ros-kilted-*` | `ros-lyrical-*` | 🟢 Низкая |
| **Docker base** | `ros:kilted-ros-base` | `ros:lyrical-ros-base` | 🟢 Низкая |
| **Zenoh** | Отдельный `rmw_zenoh_cpp` | Встроен? Нужно уточнить | 🟡 Средняя |

### 2.2 Доступность third-party образов

| Образ | Kilted | Lyrical |
|-------|--------|---------|
| `introlab3it/rtabmap_ros` | ✅ `kilted` (amd64+arm64) | ✅ `lyrical-latest` (amd64+arm64) |
| `luxonis/depthai-ros` | ✅ `kilted-arm64-latest` | ❌ **НЕТ** (ни amd64, ни arm64) |
| `ros:lyrical-ros-base` | — | ✅ (amd64+arm64) |

**🚨 БЛОКЕР: `luxonis/depthai-ros` не имеет lyrical тегов!**

Варианты решения:

#### Вариант A: Ждать (≈1-3 месяца)
Luxonis обычно добавляет новые дистрибутивы через 1-3 месяца после релиза ROS 2.
Плюсы: ноль усилий. Минусы: неопределённый срок.

#### Вариант B: Запросить у Luxonis
Создать Issue/PR в `luxonis/depthai-ros` с запросом lyrical сборки.
Плюсы: помогает сообществу. Минусы: нет гарантии сроков.

#### Вариант C: Свой Dockerfile (рекомендовано)
Собрать свой образ на базе `ros:lyrical-ros-base`, повторив логику официального.

**Анализ официального Dockerfile (`luxonis/depthai-ros`, ветка `kilted`):**
```dockerfile
ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}-ros-base
# Ставит build-зависимости (git, wget, colcon, libusb, etc.)
# Клонирует depthai-core + depthai-ros исходники
# rosdep install → colcon build --build-shared-libs
```

**Что нужно изменить для Lyrical (всего 2 строки):**
1. `ARG ROS_DISTRO=lyrical`
2. `git clone --branch lyrical` (или `main` если lyrical ветки нет)

**Наш `Dockerfile.depthai` для lyrical:**

```dockerfile
# Собственная сборка depthai-ros для ROS 2 Lyrical Luth
# На основе официального Dockerfile от Luxonis
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Build-зависимости (без zsh/ohmyzsh — не нужны для production)
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common git libusb-1.0-0-dev wget \
    python3-colcon-common-extensions zip unzip tar \
    python3-pip python3-dev build-essential \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
ENV WS=/ws
RUN mkdir -p $WS/src

# Клонируем depthai-ros и depthai-core
COPY ./ $WS/src/depthai-ros
RUN cd $WS/src && \
    git clone --branch main https://github.com/luxonis/depthai-core.git && \
    cd depthai-core && git submodule update --init --recursive

# Устанавливаем ROS зависимости
RUN cd $WS/ && rosdep install --from-paths src --ignore-src -r -y

# Собираем (только depthai-ros, без examples/tests для скорости)
RUN cd $WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select depthai-ros depthai_bridge depthai_ros_driver \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Устанавливаем Zenoh middleware
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN echo "source ${WS}/install/setup.bash" >> /root/.bashrc
```

**Оценка сложности:**

| Фактор | Оценка |
|--------|--------|
| Написание Dockerfile | 🟢 30 минут (20 строк) |
| Первая сборка (arm64) | 🔴 2-4 часа (vcpkg + C++ компиляция) |
| Размер образа | 🔴 ~2 GB (без сборочного мусора) |
| Кэширование слоёв | 🟢 Повторные сборки — минуты |
| Поддержка | 🟡 Нужно обновлять при новых версиях depthai |

**Риски:**
- `depthai-core` может не иметь `lyrical` ветки → использовать `main`
- `build.sh` может требовать правок под Ubuntu 26.04
- vcpkg может не собрать некоторые зависимости под Resolute

### 2.3 Пакеты, которые могут измениться/исчезнуть

На основе опыта миграции humble→kilted:

| Пакет (kilted) | Статус в Lyrical | Примечание |
|----------------|-----------------|------------|
| `ros-kilted-behaviortree-cpp-v3` | Вероятно `ros-lyrical-behaviortree-cpp` (v4+) | Переименован |
| `ros-kilted-control-msgs` | Может отсутствовать | Переименован/включён в другие |
| `ros-kilted-theora-image-transport` | Может быть удалён | Theora устарел |
| `ros-kilted-rmw-zenoh-cpp` | Может быть `ros-lyrical-rmw-zenoh-cpp` или встроен | Zenoh становится нативным |

---

## 3. План миграции (по шагам)

### Этап 1: Подготовка (безопасно, не ломает сборку)

- [ ] **1.1** Ветка `feature/lyrical` создана (✅ готово)
- [ ] **1.2** Проверить `ros:lyrical-ros-base` — базовые пакеты, Python, Ubuntu (✅ образ есть)
- [ ] **1.3** Проверить `pip` версию и поведение `--break-system-packages` в Ubuntu 26.04
- [ ] **1.4** Проверить APT репозитории для `resolute` (ports.ubuntu.com, packages.ros.org)
- [ ] **1.5** Уточнить статус Zenoh — встроен ли `rmw_zenoh_cpp` в базовый образ?

### Этап 2: Базовая миграция (аналог humble→kilted)

- [ ] **2.1** Массовая замена `kilted` → `lyrical` во всех файлах:
  - `ROS_DISTRO=lyrical` во всех `.env`
  - `ros:kilted-ros-base` → `ros:lyrical-ros-base` в Dockerfile'ах
  - `ros-kilted-*` → `ros-lyrical-*` в apt-get
  - `/opt/ros/kilted/` → `/opt/ros/lyrical/` в скриптах
  - `*-kilted-*` → `*-lyrical-*` в тегах образов
- [ ] **2.2** Обновить Dockerfile'ы:
  - `Dockerfile.ros2-zenoh`: `FROM ros:lyrical-ros-base`
  - `Dockerfile.rtabmap`: `FROM introlab3it/rtabmap_ros:lyrical-latest`
  - `Dockerfile.depthai`: ❌ ЗАБЛОКИРОВАНО (нет образа)
- [ ] **2.3** Обновить CI/CD:
  - `ROS_DISTRO: lyrical` во всех workflow
  - `python-version` (зависит от Ubuntu 26.04)

### Этап 3: Системные изменения Ubuntu 26.04

- [ ] **3.1** Python 3.14 — `externally-managed` всё ещё актуально
- [ ] **3.2** APT sources: `noble` → `resolute`, `ubuntu.com` → `old-releases`?
- [ ] **3.3** Системные пакеты могут изменить названия
- [ ] **3.4** `apt-cacher-ng` — будет ли работать с Resolute?

### Этап 4: Специфичные для проекта изменения

- [ ] **4.1** Zenoh: проверить, нужен ли отдельный `rmw_zenoh_cpp` или он встроен
- [ ] **4.2** Убрать `LD_LIBRARY_PATH=/opt/ros/lyrical/opt/zenoh_cpp_vendor/...` если Zenoh встроен
- [ ] **4.3** `docker/vision/zenoh-router/Dockerfile` — возможно больше не нужен
- [ ] **4.4** `docker/main/zenoh-router/Dockerfile` — возможно больше не нужен
- [ ] **4.5** Использовать новые фичи: `EventsCBGExecutor`, `AsyncNode`

### Этап 5: Решение блокера depthai

**Выбранный путь: Вариант C — свой Dockerfile**

- [ ] **5.1** Создать `docker/base/Dockerfile.depthai` для lyrical (на основе официального из `luxonis/depthai-ros`)
- [ ] **5.2** Клонировать `depthai-core` (ветка `main`) и `depthai-ros` исходники
- [ ] **5.3** Собрать через `colcon build --packages-select depthai-ros depthai_bridge depthai_ros_driver`
- [ ] **5.4** Добавить Zenoh (`ros-lyrical-rmw-zenoh-cpp`) в образ
- [ ] **5.5** Запушить в GHCR как `rob_box_base:depthai-lyrical-latest`
- [ ] **5.6** Обновить `docker/vision/oak-d/Dockerfile` → использовать новый базовый образ
- [ ] **5.7** Обновить `docker/vision/zenoh-router/Dockerfile` (если ещё нужен)
- [ ] **5.8** Запасной план: если сборка из исходников не удаётся → использовать `depthai-ros:kilted-arm64-latest` как временную базу (разные Ubuntu, но может сработать)
- [ ] **5.9** Создать Issue в `luxonis/depthai-ros` с запросом официальной lyrical сборки (параллельно)

### Этап 6: Тестирование и CI

- [ ] **6.1** Запустить `L: Build All Services` с `build_base_images=true`
- [ ] **6.2** Итеративно исправлять ошибки сборки (пакетов, Python, системных)
- [ ] **6.3** Все сервисы должны собраться ✅
- [ ] **6.4** Создать PR `feature/lyrical` → `develop`

---

## 4. Оценка рисков

| Риск | Вероятность | Влияние | Смягчение |
|------|-----------|--------|-----------|
| depthai-ros не выходит для lyrical | Высокая | Блокер | Билдить из исходников |
| Python 3.14 ломает pip пакеты | Средняя | Высокое | `--break-system-packages` + `--ignore-installed` |
| APT репы Resolute недоступны | Низкая | Блокер | Использовать `old-releases.ubuntu.com` |
| Zenoh integration меняет архитектуру | Средняя | Среднее | Проверить документацию, адаптировать |
| Системные пакеты переименованы | Средняя | Среднее | Итеративное исправление по логам CI |
| rtabmap_ros:lyrical теряет arm64 | Низкая | Среднее | Уже проверено — arm64 есть ✅ |

---

## 5. Ожидаемые выгоды от миграции

1. **LTS до 2031** — стабильность на 5 лет (vs Kilted — не LTS)
2. **Zenoh нативный** — упрощение архитектуры, меньше внешних зависимостей
3. **EventsCBGExecutor** — 10-15% экономия CPU на роботе
4. **AsyncNode** — возможность использовать asyncio для голосового ассистента
5. **URDF 1.2** — capsule geometry, quaternions для более точной модели робота
6. **Bag improvements** — circular recording для автономной работы

---

## 6. Ориентировочные трудозатраты

| Этап | Часы | Примечание |
|------|------|------------|
| Подготовка + исследование | 2-4 | Проверка Python, APT, Zenoh |
| Базовая замена kilted→lyrical | 1-2 | Массовый search-replace, как для kilted |
| Системные правки Ubuntu 26.04 | 4-8 | Самая непредсказуемая часть |
| depthai решение | 4-16 | Если билдить самим — дольше |
| Итеративные исправления CI | 8-16 | Как показал опыт kilted, много итераций |
| **Итого** | **20-45 часов** | Сильно зависит от depthai и системных сюрпризов |

---

## 7. Следующие шаги

1. 🔴 Решить вопрос с `depthai-ros` для lyrical (блокер)
2. 🟡 Начать поиск-replace `kilted`→`lyrical` (можно параллельно)
3. 🟡 Проверить Python 3.14 поведение с pip
4. 🟢 Итеративная сборка и исправление в CI

---

*План создан на основе успешного опыта миграции humble→kilted (16 коммитов, 200+ файлов).*
