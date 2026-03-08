# Docker Project Standards для rob_box_project

## Структура проекта (ОБНОВЛЕНО 2025-10-10)

```
docker/
├── base/                        # Базовые Docker образы (для ускорения сборки)
│   ├── Dockerfile.ros2-zenoh    # Базовый образ ROS 2 + Zenoh
│   ├── Dockerfile.rtabmap       # Базовый образ RTAB-Map SLAM
│   ├── Dockerfile.depthai       # Базовый образ для OAK-D camera
│   └── Dockerfile.pcl           # Базовый образ PCL для лидаров
│
├── main/                        # Main Pi (10.1.1.20) - обработка данных
│   ├── docker-compose.yaml      # Оркестрация контейнеров Main Pi
│   ├── config/                  # ✅ Конфиги (общие в корне, специфичные в подпапках)
│   │   ├── zenoh_router_config.json5        # Общий для всех
│   │   ├── zenoh_session_config.json5       # Общий для всех
│   │   ├── cyclonedds.xml                   # Общий для всех
│   │   ├── rtabmap/                         # Специфичные для rtabmap
│   │   │   ├── rtabmap.yaml
│   │   │   └── rtabmap_config.yaml
│   │   └── robot_state_publisher/           # Специфичные для RSP
│   │       └── (будущие конфиги)
│   ├── scripts/                 # ✅ Скрипты (утилиты в корне, запуск в подпапках)
│   │   ├── update_and_restart.sh            # Утилитарный
│   │   ├── build_base_images.sh             # Утилитарный
│   │   ├── rtabmap/                         # Скрипты запуска rtabmap
│   │   │   └── start_rtabmap.sh
│   │   └── robot_state_publisher/           # Скрипты запуска RSP
│   │       └── start_robot_state_publisher.sh
│   ├── maps/                    # Persistent данные для RTAB-Map
│   ├── robot_state_publisher/   # ✅ ТОЛЬКО Dockerfile
│   │   └── Dockerfile
│   ├── rtabmap/                 # ✅ ТОЛЬКО Dockerfile
│   │   └── Dockerfile
│   └── zenoh-router/            # ✅ ТОЛЬКО Dockerfile
│       └── Dockerfile
│
└── vision/                      # Vision Pi (10.1.1.21) - сенсоры
    ├── docker-compose.yaml      # Оркестрация контейнеров Vision Pi
    ├── config/                  # ✅ Конфиги (общие в корне, специфичные в подпапках)
    │   ├── zenoh_router_config.json5        # Общий для всех
    │   ├── zenoh_session_config.json5       # Общий для всех
    │   ├── cyclonedds.xml                   # Общий для всех
    │   ├── oak-d/                           # Специфичные для OAK-D
    │   │   └── (будущие конфиги)
    │   ├── apriltag/                        # Специфичные для AprilTag
    │   │   └── apriltag_config.yaml
    │   └── led_matrix/                      # Специфичные для LED Matrix
    │       └── led_matrix_config.yaml
    ├── scripts/                 # ✅ Скрипты (утилиты в корне, запуск в подпапках)
    │   ├── update_and_restart.sh            # Утилитарный
    │   ├── diagnose.sh                      # Утилитарный
    │   ├── monitor_camera_startup.sh        # Утилитарный
    │   ├── realtime_monitor.sh              # Утилитарный
    │   ├── setup_lidar.sh                   # Утилитарный
    │   ├── force_publish.sh                 # Утилитарный
    │   ├── build_base_images.sh             # Утилитарный
    │   ├── switch_version.sh                # Утилитарный
    │   ├── oak-d/                           # Скрипты запуска OAK-D
    │   │   └── start_oak_d.sh
    │   └── apriltag/                        # Скрипты запуска AprilTag
    │       └── start_apriltag.sh
    ├── oak-d/                   # ✅ ТОЛЬКО Dockerfile
    │   └── Dockerfile
    ├── apriltag/                # ✅ ТОЛЬКО Dockerfile
    │   └── Dockerfile
    └── zenoh-router/            # ✅ ТОЛЬКО Dockerfile
        └── Dockerfile
```

## Правила организации

### 0. Базовые Docker образы (Base Images)

Проект использует многоуровневую архитектуру базовых образов для ускорения сборки:

#### Иерархия образов:

```
docker/base/
├── Dockerfile.ros2-zenoh   → rob_box_base:ros2-zenoh (ROS 2 + Zenoh, общая база)
├── Dockerfile.rtabmap      → rob_box_base:rtabmap (RTAB-Map SLAM)
├── Dockerfile.depthai      → rob_box_base:depthai (OAK-D camera)
└── Dockerfile.pcl          → rob_box_base:pcl (Point Cloud Library для лидаров)
```

#### Принципы проектирования базовых образов:

1. **Наследование от upstream** - используем официальные образы как основу:
   - `rob_box_base:rtabmap` ← FROM `introlab3it/rtabmap_ros:humble-latest`
   - `rob_box_base:depthai` ← FROM `luxonis/depthai-ros:humble-latest`
   - `rob_box_base:pcl` ← FROM `rob_box_base:ros2-zenoh`

2. **Минимальное дополнение** - базовый образ добавляет только:
   - `ros-humble-rmw-zenoh-cpp` (Zenoh middleware)
   - Специфичные для функционала пакеты (PCL, image-transport, etc)
   - ENV переменные для Zenoh

3. **Один базовый образ = одна функция:**
   - ❌ НЕПРАВИЛЬНО: `base-main-pi`, `base-vision-pi` (привязка к железу)
   - ✅ ПРАВИЛЬНО: `base-rtabmap`, `base-depthai`, `base-pcl` (привязка к функционалу)

#### Использование в сервисах:

```dockerfile
# robot_state_publisher - простой сервис, использует общую базу
ARG BASE_IMAGE=rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}
RUN apt-get install -y ros-humble-robot-state-publisher ros-humble-xacro

# rtabmap - SLAM, использует специализированную базу
ARG BASE_IMAGE=rob_box_base:rtabmap
FROM ${BASE_IMAGE}
# Все зависимости уже есть!

# lslidar - лидар, использует PCL базу
ARG BASE_IMAGE=rob_box_base:pcl
FROM ${BASE_IMAGE}
# PCL, diagnostic-updater, libpcap уже установлены
```

#### Сборка базовых образов:

**Main Pi:**
```bash
cd ~/rob_box_project/docker/main
./scripts/build_base_images.sh
```

**Vision Pi:**
```bash
cd ~/rob_box_project/docker/vision
./scripts/build_base_images.sh
```

**Когда пересобирать базовые образы:**
- При обновлении upstream образов (rtabmap_ros, depthai-ros)
- При изменении версии ROS 2 или Zenoh
- При добавлении общих зависимостей
- НЕ нужно при изменении конфигов или launch файлов сервисов!

### 1. Структура папок сервисов (ОБНОВЛЕНО)

**Новая структура:** Каждый сервис имеет ТОЛЬКО Dockerfile, все конфиги и скрипты вынесены в общие папки

```
<service_name>/
└── Dockerfile           # ОБЯЗАТЕЛЬНО: только Dockerfile, ничего больше!
```

**Конфиги и скрипты:**
```
docker/<env>/
├── config/                          # Конфиги
│   ├── zenoh_router_config.json5    # ✅ Общие в корне
│   ├── zenoh_session_config.json5   # ✅ Общие в корне  
│   └── <service_name>/              # ✅ Специфичные в подпапке
│       ├── service_config.yaml
│       └── service_launch.py
│
└── scripts/                         # Скрипты
    ├── diagnose.sh                  # ✅ Утилитарные в корне
    ├── monitor.sh                   # ✅ Утилитарные в корне
    └── <service_name>/              # ✅ Скрипты запуска в подпапке
        └── start_service.sh
```

**Правила:**
- Для новых сервисов имя папки должно совпадать с именем сервиса в `docker-compose.yaml`
- В текущем репозитории есть legacy-исключения, где compose использует kebab-case, а папка — snake_case:
  - `robot-state-publisher` → `robot_state_publisher/`
  - `twist-mux` → `twist_mux/`
  - `micro-ros-agent` → `micro_ros_agent/`
  - `ros2-control` → `ros2_control/`
- Такие legacy-имена не переименовываем без отдельной задачи, потому что это затрагивает compose, workflows, docs и локальные скрипты
- В папке сервиса ТОЛЬКО `Dockerfile`
- Все конфиги в `config/` (общие в корне, специфичные в `config/<service>/`)
- Все скрипты в `scripts/` (утилиты в корне, запуска в `scripts/<service>/`)
- Startup-скрипты сервисов должны лежать в `scripts/<service>/start_<service>.sh`
- Shared-конфиги для нескольких сервисов должны жить в подпапке подсистемы, например `config/monitoring/`
- Shared helper scripts могут оставаться в корне `scripts/`, если они используются несколькими сервисами

**Преимущества новой структуры:**
1. ✅ Единообразные пути в Dockerfile: `config/<service>/file` и `scripts/<service>/file`
2. ✅ Работает одинаково в docker compose и GitHub Actions
3. ✅ Легко понять где общие файлы, где специфичные
4. ✅ Не нужно помнить относительные пути `../` или `../../`

### 1.1 Допустимые исключения

Ниже перечислены исключения, которые считаются корректными и не должны удаляться в рамках обычной нормализации структуры:

- `../../src/rob_box_description:/workspace/src/rob_box_description:ro` в `robot-state-publisher` — это source bind для URDF package, а не runtime-config и не runtime-script.
- `config/monitoring/...` — это shared subsystem config для monitoring-стека, а не нарушение правила `config/<service>/...`.
- Общие helper scripts вроде `ros_with_namespace.sh` и `patch_rtabmap_launch.py` могут оставаться в корне `scripts/`, потому что используются несколькими сервисами.

### 2. Правила volumes

#### Общее правило монтирования конфигов:

```yaml
volumes:
  - ./config:/config  # ВСЕГДА монтируем общую папку config
  - ./scripts:/scripts  # Опционально, если нужны скрипты в runtime
```

**НЕ дублируем** отдельные файлы из config:
```yaml
# ❌ ПЛОХО - дублирование
volumes:
  - ./config:/config
  - ./config/zenoh_session_config.json5:/config/zenoh_session_config.json5  # Избыточно!
  - ./config/rtabmap/rtabmap.yaml:/config/rtabmap.yaml  # Избыточно!

# ✅ ХОРОШО - чисто и просто
volumes:
  - ./config:/config
  - ./maps:/maps  # Дополнительные volume только для persistent данных
```

### 2.1 Правила COPY в Dockerfile (ОБНОВЛЕНО)

**Основное правило:** runtime-конфиги, runtime-скрипты и service-local launch-файлы не копируются в образ. Они монтируются через `docker-compose.yaml` из `config/` и `scripts/`.

#### Что НЕ копируем в образ:

```dockerfile
# ❌ НЕПРАВИЛЬНО: runtime-файлы не должны запекаться в образ
COPY config/lslidar/lsx10_custom.yaml /config/lsx10_custom.yaml
COPY scripts/lslidar/start_lslidar.sh /start_lslidar.sh
COPY config/oak-d/launch/oakd_with_apriltag.launch.py /launch/oakd_with_apriltag.launch.py
```

#### Что делаем вместо этого:

```dockerfile
# ✅ ПРАВИЛЬНО: Dockerfile содержит только неизменяемую инфраструктуру
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:pcl
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y <deps> && rm -rf /var/lib/apt/lists/*

# Runtime-файлы приходят через compose volumes:
# - ./config:/config:ro
# - ./scripts/lslidar:/scripts:ro

CMD ["/scripts/start_lslidar.sh"]
```

**Исключение:** если файл действительно нужен во время `docker build` или `colcon build`, его можно `COPY`-нуть в образ. Но это не относится к обычным runtime YAML, startup scripts и service-local launch-файлам.

### 3. Environment Variables - стандартный набор

Все ROS 2 + Zenoh сервисы должны иметь:

```yaml
environment:
  # ROS 2 базовые
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_zenoh_cpp
  
  # Zenoh конфигурация
  - ZENOH_CONFIG=/config/zenoh_session_config.json5
  - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  - ZENOH_ROUTER_CHECK_ATTEMPTS=10
  - RUST_LOG=zenoh=info
  
  # Дополнительные (для специфичных контейнеров)
  - LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib
  - QT_QPA_PLATFORM=offscreen  # Для headless режима
```

### 4. Naming Conventions

#### Container names:
- Простые, lowercase: `zenoh-router`, `oak-d`, `lslidar`, `apriltag`
- Суффикс `-vision` для Vision Pi если нужно различить: `zenoh-router-vision`

#### Image names:
- Формат: `<service>-custom:latest` для кастомных образов
- Пример: `robot-state-publisher-custom:latest`, `rtabmap-custom:latest`
- Официальные образы: `eclipse/zenoh:1.6.2`, `ros:humble-ros-base`

#### Volume paths:
- Всегда относительные пути от папки с docker-compose: `./config`, `./maps`, `./scripts`
- Внутри контейнера стандартные пути: `/config`, `/maps`, `/workspace`

### 5. Скрипты (ОБНОВЛЕНО)

#### Расположение:

```
docker/<env>/scripts/
├── diagnose.sh                    # ✅ Утилитарные скрипты в корне
├── monitor.sh
├── update_and_restart.sh
├── build_base_images.sh
└── <service>/                     # ✅ Скрипты запуска в подпапках
    └── start_service.sh
```

**Правила:**
- Утилитарные скрипты (диагностика, мониторинг) - в корне `scripts/`
- Скрипты запуска сервисов - в `scripts/<service>/start_*.sh`
- НЕ хранить скрипты в `docker/<env>/<service>/scripts/` (старая структура)

#### Структура скрипта запуска (start_*.sh):

```bash
#!/bin/bash
set -e

# 1. Ждём Zenoh Router
echo "Waiting for Zenoh Router..."
attempts=${ZENOH_ROUTER_CHECK_ATTEMPTS:-10}
for i in $(seq 1 $attempts); do
  if wget -qO- http://localhost:8000/@/local/router &>/dev/null; then
    echo "Zenoh Router is ready!"
    break
  fi
  sleep 2
done

# 2. Source ROS 2
source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# 3. Запуск основного процесса
exec ros2 launch <package> <launch_file>
```

#### Использование в Dockerfile:

```dockerfile
# Startup script приходит через volume mount из scripts/<service>/
CMD ["/scripts/start_<service>.sh"]
```

### 6. Device Access

Для доступа к железу (камеры, LiDAR, serial):

```yaml
# USB устройства (камеры)
privileged: true
volumes:
  - /dev/bus/usb:/dev/bus/usb

# Serial устройства (LiDAR)
privileged: true
devices:
  - /dev/ttyACM0:/dev/ttyACM0
```

### 7. Memory Limits (для Raspberry Pi)

Обязательно для тяжёлых сервисов на Pi:

```yaml
# Vision Pi: 8GB RAM total
mem_limit: 6g          # Оставляем 2GB для системы
memswap_limit: 7g      # 6GB RAM + 1GB swap max
```

### 8. Network Mode

Всегда `network_mode: host` для ROS 2 + Zenoh:

```yaml
network_mode: host  # Для multicast discovery и Zenoh
```

### 9. Restart Policy

```yaml
restart: unless-stopped  # Стандарт для production сервисов
```

### 10. Dependencies

```yaml
depends_on:
  - zenoh-router  # Все сервисы зависят от Zenoh Router
```

## Workflow для добавления нового сервиса

1. **Создать структуру:**
   ```bash
   cd docker/<env>
  mkdir -p my-service config/my-service scripts/my-service
  touch my-service/Dockerfile
  touch scripts/my-service/start_my_service.sh
   ```

2. **Dockerfile:**
   ```dockerfile
   FROM ros:humble-ros-base

   RUN apt-get update && apt-get install -y \
       ros-humble-rmw-zenoh-cpp \
       <other-deps> \
       && rm -rf /var/lib/apt/lists/*

   # Runtime-конфиги и startup scripts НЕ копируем.
   CMD ["/scripts/start_my_service.sh"]
   ```

3. **docker-compose.yaml:**
   ```yaml
   my-service:
     build:
       context: .
       dockerfile: my-service/Dockerfile
     container_name: my-service
     network_mode: host
     environment:
       - ROS_DOMAIN_ID=0
       - RMW_IMPLEMENTATION=rmw_zenoh_cpp
       - ZENOH_CONFIG=/config/zenoh_session_config.json5
       - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
       - ZENOH_ROUTER_CHECK_ATTEMPTS=10
       - RUST_LOG=zenoh=info
     volumes:
       - ./config:/config:ro
       - ./scripts/my-service:/scripts:ro
       - /dev/shm:/dev/shm
     depends_on:
       - zenoh-router
     restart: unless-stopped
   ```

4. **Добавить runtime-файлы:**
   ```bash
  vim config/my-service/my_config.yaml
  vim scripts/my-service/start_my_service.sh
  chmod +x scripts/my-service/start_my_service.sh
   ```

5. **Проверить compose и запустить:**
   ```bash
   docker compose config
   docker compose build my-service
   docker compose up -d my-service
   docker compose logs -f my-service
   ```

### Примеры из проекта

### Сервисная папка содержит только Dockerfile:
- `zenoh-router/` - только Dockerfile
- `apriltag/` - только Dockerfile
- `rtabmap/` - только Dockerfile

### Startup script в `scripts/<service>/`:
- `scripts/robot_state_publisher/start_robot_state_publisher.sh`
- `scripts/rtabmap/start_rtabmap.sh`
- `scripts/lslidar/start_lslidar.sh`

### Service-specific config в `config/<service>/`:
- `config/rtabmap/`
- `config/lslidar/`

### Shared subsystem config:
- `config/monitoring/promtail-config.yaml`

## Checklist для ревью

При добавлении/изменении сервиса проверить:

- [ ] Для новых сервисов папка сервиса называется как сервис в `docker-compose.yaml` (legacy-исключения проверяются отдельно)
- [ ] В папке сервиса ТОЛЬКО Dockerfile
- [ ] Конфиги: общие в `config/`, специфичные в `config/<service>/`
- [ ] Скрипты: утилитарные в `scripts/`, запуска в `scripts/<service>/`
- [ ] Volume монтирование: `./config:/config`, service-local script mount и без дублей
- [ ] В Dockerfile нет `COPY config/` и `COPY scripts/` для runtime-файлов
- [ ] Стандартный набор environment переменных
- [ ] Approved exceptions задокументированы, если сервис использует source bind или shared subsystem config
- [ ] `network_mode: host`
- [ ] `depends_on: zenoh-router`
- [ ] `restart: unless-stopped`
- [ ] Для Pi: memory limits если сервис тяжёлый

## Миграция на новую структуру (2025-10-10)

### Что изменилось:

**СТАРАЯ структура (НЕ работает в CI/CD):**
```
docker/vision/
├── config/                        # Общие конфиги
├── scripts/                       # Утилитарные скрипты
├── oak-d/
│   ├── Dockerfile
│   ├── config/                    # ❌ Специфичные конфиги здесь
│   └── scripts/                   # ❌ Скрипты запуска здесь
│       └── start_oak_d.sh
```

## Миграция существующих сервисов

Если нашли нарушения стандартов:

1. Переместить startup-скрипты в `scripts/<service>/`
2. Переместить service-specific runtime-конфиги в `config/<service>/`
3. Убрать дублирующие file-level bind mounts, если достаточно `./config:/config`
4. Оставить shared helper scripts в корне `scripts/`, только если они используются несколькими сервисами
5. Если есть source bind или shared subsystem config, явно задокументировать это как approved exception
6. Проверить `docker compose config`, затем запуск сервиса

---

**Версия:** 2.1  
**Дата:** 2026-03-08  
**Автор:** AI Agent (GitHub Copilot)  
**Изменения:** Актуализированы правила runtime layout, approved exceptions и service-local startup scripts
