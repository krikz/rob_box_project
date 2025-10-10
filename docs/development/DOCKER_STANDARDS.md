# Docker Project Standards для rob_box_project

## Структура проекта (ОБНОВЛЕНО 2025-10-10)

```
docker/
├── DOCKER_STANDARDS.md          # Этот файл - стандарты организации
├── AGENT_GUIDE.md               # Гайд для AI агентов
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
│   │   │   └── entrypoint.sh
│   │   └── robot_state_publisher/           # Скрипты запуска RSP
│   │       └── entrypoint.sh
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
    │   ├── lslidar/                         # Специфичные для LSLIDAR
    │   │   ├── lsx10_custom.yaml
    │   │   └── lslidar_headless_launch.py
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
    │   ├── lslidar/                         # Скрипты запуска LSLIDAR
    │   │   └── start_lslidar.sh
    │   └── apriltag/                        # Скрипты запуска AprilTag
    │       └── start_apriltag.sh
    ├── oak-d/                   # ✅ ТОЛЬКО Dockerfile
    │   └── Dockerfile
    ├── lslidar/                 # ✅ ТОЛЬКО Dockerfile
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
- Название папки сервиса = название сервиса в `docker-compose.yaml`
- В папке сервиса ТОЛЬКО `Dockerfile`
- Все конфиги в `config/` (общие в корне, специфичные в `config/<service>/`)
- Все скрипты в `scripts/` (утилиты в корне, запуска в `scripts/<service>/`)

**Преимущества новой структуры:**
1. ✅ Единообразные пути в Dockerfile: `config/<service>/file` и `scripts/<service>/file`
2. ✅ Работает одинаково в docker compose и GitHub Actions
3. ✅ Легко понять где общие файлы, где специфичные
4. ✅ Не нужно помнить относительные пути `../` или `../../`

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

**Новая структура путей:**
Docker context устанавливается в `docker/<env>/` (из docker-compose: `context: .`)

#### Копирование конфигов:

```dockerfile
# ✅ ПРАВИЛЬНО: копируем из подпапки сервиса в config/
COPY config/lslidar/lsx10_custom.yaml /config/lsx10_custom.yaml
COPY config/lslidar/lslidar_headless_launch.py /ws/src/lslidar_ros2_driver/lslidar_driver/launch/

# ✅ ПРАВИЛЬНО: копируем общий конфиг
COPY config/zenoh_session_config.json5 /config/zenoh_session_config.json5

# ❌ НЕПРАВИЛЬНО: старые пути не работают в CI/CD
COPY lslidar/config/lsx10_custom.yaml /config/
COPY ../../scripts/start.sh /start.sh
```

#### Копирование скриптов запуска:

```dockerfile
# ✅ ПРАВИЛЬНО: скрипт из подпапки сервиса в scripts/
COPY scripts/oak-d/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

# ✅ ПРАВИЛЬНО: для lslidar
COPY scripts/lslidar/start_lslidar.sh /start_lslidar.sh  
RUN chmod +x /start_lslidar.sh

# ❌ НЕПРАВИЛЬНО: старые пути
COPY oak-d/scripts/start_oak_d.sh /start_oak_d.sh
COPY ../../scripts/start_oak_d.sh /start_oak_d.sh
```

#### Примеры полных Dockerfile:

**Пример 1: OAK-D (простой сервис)**
```dockerfile
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:depthai
FROM ${BASE_IMAGE}

# Копируем скрипт запуска
COPY scripts/oak-d/start_oak_d.sh /start_oak_d.sh
RUN chmod +x /start_oak_d.sh

CMD ["/start_oak_d.sh"]
```

**Пример 2: LSLIDAR (со специфичными конфигами)**
```dockerfile
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:pcl
FROM ${BASE_IMAGE}

# ... apt-get install и git clone ...

# Копируем специфичные конфиги из config/lslidar/
COPY config/lslidar/lslidar_headless_launch.py /ws/src/lslidar_ros2_driver/lslidar_driver/launch/
COPY config/lslidar/lsx10_custom.yaml /config/lsx10_custom.yaml

# Собираем workspace
WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Копируем скрипт запуска
COPY scripts/lslidar/start_lslidar.sh /start_lslidar.sh
RUN chmod +x /start_lslidar.sh

ENTRYPOINT ["/start_lslidar.sh"]
```

**Правило выбора: volume vs COPY:**
- **COPY в образ** - если файл нужен при сборке или редко меняется (launch файлы, scripts)
- **Volume монтирование** - если файл часто редактируется или общий для нескольких сервисов

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
- Официальные образы: `eclipse/zenoh:latest`, `ros:humble-ros-base`

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
# Копируем скрипт запуска
COPY scripts/<service>/start_<service>.sh /start_<service>.sh
RUN chmod +x /start_<service>.sh

CMD ["/start_<service>.sh"]
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
   mkdir my_service
   cd my_service
   touch Dockerfile entrypoint.sh
   mkdir config  # Если нужны специфичные конфиги
   ```

2. **Dockerfile:**
   ```dockerfile
   FROM ros:humble-ros-base
   
   # Установить зависимости
   RUN apt-get update && apt-get install -y \
       ros-humble-rmw-zenoh-cpp \
       <other-deps> \
       && rm -rf /var/lib/apt/lists/*
   
   # Скопировать специфичные конфиги
   COPY my_service/config/my_config.yaml /config/my_config.yaml
   
   # Собрать пакеты
   WORKDIR /ws
   RUN source /opt/ros/humble/setup.bash && colcon build
   
   # Entrypoint
   COPY my_service/entrypoint.sh /entrypoint.sh
   RUN chmod +x /entrypoint.sh
   ENTRYPOINT ["/entrypoint.sh"]
   ```

3. **docker-compose.yaml:**
   ```yaml
   my-service:
     build:
       context: .
       dockerfile: my_service/Dockerfile
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
       - ./config:/config
       - /dev/shm:/dev/shm
     depends_on:
       - zenoh-router
     restart: unless-stopped
   ```

4. **Обновить конфиг (если нужен):**
   ```bash
   # Общий конфиг
   vim config/my_service_config.yaml
   
   # Или специфичный
   vim my_service/config/my_config.yaml
   ```

5. **Собрать и запустить:**
   ```bash
   docker compose build my-service
   docker compose up -d my-service
   docker compose logs -f my-service
   ```

## Примеры из проекта

### Простой сервис (только Dockerfile):
- `zenoh-router/` - только Dockerfile, использует официальный образ
- `apriltag/` - только Dockerfile

### Сервис с entrypoint:
- `robot_state_publisher/` - Dockerfile + entrypoint.sh для генерации YAML параметров
- `lslidar/` - Dockerfile + entrypoint.sh для проверки Zenoh Router

### Сервис с config/:
- `lslidar/` - имеет специфичные `lsx10_custom.yaml` и `lslidar_headless_launch.py`

## Checklist для ревью

При добавлении/изменении сервиса проверить:

- [ ] Папка сервиса называется как container_name
- [ ] В папке сервиса ТОЛЬКО Dockerfile
- [ ] Конфиги: общие в `config/`, специфичные в `config/<service>/`
- [ ] Скрипты: утилитарные в `scripts/`, запуска в `scripts/<service>/`
- [ ] Volume монтирование: только `./config:/config`, без дублей
- [ ] Пути COPY в Dockerfile: `config/<service>/file` и `scripts/<service>/file`
- [ ] Стандартный набор environment переменных
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

**НОВАЯ структура (✅ работает везде):**
```
docker/vision/
├── config/
│   └── oak-d/                     # ✅ Специфичные конфиги здесь
│       └── (конфиги)
├── scripts/
│   └── oak-d/                     # ✅ Скрипты запуска здесь
│       └── start_oak_d.sh
└── oak-d/
    └── Dockerfile                 # ✅ ТОЛЬКО Dockerfile
```

### Шаги миграции:

1. **Переместить специфичные конфиги:**
   ```bash
   # Из service/config/ в config/service/
   mv docker/vision/lslidar/config/* docker/vision/config/lslidar/
   rmdir docker/vision/lslidar/config
   ```

2. **Переместить скрипты запуска:**
   ```bash
   # Из service/scripts/ в scripts/service/
   mv docker/vision/oak-d/scripts/* docker/vision/scripts/oak-d/
   rmdir docker/vision/oak-d/scripts
   ```

3. **Обновить пути в Dockerfile:**
   ```dockerfile
   # ❌ БЫЛО:
   COPY oak-d/scripts/start_oak_d.sh /start_oak_d.sh
   
   # ✅ СТАЛО:
   COPY scripts/oak-d/start_oak_d.sh /start_oak_d.sh
   ```

4. **Протестировать сборку:**
   ```bash
   cd docker/vision
   docker compose build oak-d
   docker compose up -d oak-d
   docker compose logs -f oak-d
   ```

### Преимущества новой структуры:

1. ✅ **Единообразные пути** - всегда `config/<service>/` и `scripts/<service>/`
2. ✅ **Работает везде** - одинаково в docker compose и GitHub Actions
3. ✅ **Понятная организация** - общие файлы в корне, специфичные в подпапках
4. ✅ **Легко масштабировать** - просто добавить новую подпапку
5. ✅ **Минимум в папке сервиса** - только Dockerfile, всё остальное централизованно

---

**Версия:** 2.0  
**Дата:** 2025-10-10  
**Автор:** AI Agent (Claude)  
**Изменения:** Переработана структура config/ и scripts/ для совместимости с CI/CD
- [ ] `depends_on: zenoh-router`
- [ ] `restart: unless-stopped`
- [ ] Для Pi: memory limits если сервис тяжёлый
- [ ] entrypoint.sh проверяет доступность Zenoh Router

## Миграция существующих сервисов

Если нашли нарушения стандартов:

1. Переместить скрипты из корня в `scripts/`
2. Убрать дублирование volume монтирований
3. Переместить специфичные конфиги в `<service>/config/`
4. Добавить недостающие environment переменные
5. Обновить пути в Dockerfile после перемещений
6. Протестировать сборку и запуск

---

**Версия:** 1.0  
**Дата:** 2025-10-09  
**Автор:** AI Agent (Claude)
