# Docker Project Standards для rob_box_project

## Структура проекта

```
docker/
├── DOCKER_STANDARDS.md          # Этот файл - стандарты организации
├── AGENT_GUIDE.md               # Гайд для AI агентов
├── base/                        # Базовые Docker образы (для ускорения сборки)
│   ├── Dockerfile.ros2-zenoh    # Базовый образ ROS 2 + Zenoh
│   ├── Dockerfile.main-base     # Базовый образ для Main Pi (RTAB-Map)
│   └── Dockerfile.vision-base   # Базовый образ для Vision Pi (OAK-D, LSLIDAR)
├── main/                        # Main Pi (10.1.1.20) - обработка данных
│   ├── docker-compose.yaml      # Оркестрация контейнеров Main Pi
│   ├── config/                  # Общие конфиги для всех сервисов
│   │   ├── zenoh_router_config.json5
│   │   ├── zenoh_session_config.json5
│   │   ├── cyclonedds.xml
│   │   ├── rtabmap.yaml
│   │   └── rtabmap_config.yaml
│   ├── scripts/                 # Утилитарные скрипты
│   │   └── update_and_restart.sh
│   ├── maps/                    # Persistent данные для RTAB-Map
│   ├── <service_name>/          # Папки для кастомных сервисов
│   │   ├── Dockerfile           # Обязательно
│   │   ├── entrypoint.sh        # Опционально
│   │   └── config/              # Опционально: специфичные конфиги
│   ├── robot_state_publisher/
│   ├── rtabmap/
│   └── zenoh-router/
│
└── vision/                      # Vision Pi (10.1.1.21) - сенсоры
    ├── docker-compose.yaml      # Оркестрация контейнеров Vision Pi
    ├── config/                  # Общие конфиги для всех сервисов
    │   ├── zenoh_router_config.json5
    │   ├── zenoh_session_config.json5
    │   ├── cyclonedds.xml
    │   ├── oak_d_config.yaml
    │   ├── apriltag_config.yaml
    │   ├── lidar_config.yaml
    │   └── led_matrix_config.yaml
    ├── scripts/                 # Утилитарные скрипты
    │   ├── update_and_restart.sh
    │   ├── diagnose.sh
    │   ├── monitor_camera_startup.sh
    │   ├── realtime_monitor.sh
    │   ├── setup_lidar.sh
    │   ├── force_publish.sh
    │   ├── start_apriltag.sh
    │   └── start_oak_d.sh
    ├── <service_name>/          # Папки для кастомных сервисов
    │   ├── Dockerfile           # Обязательно
    │   ├── entrypoint.sh        # Опционально
    │   └── config/              # Опционально: специфичные конфиги/launch
    ├── oak-d/
    ├── lslidar/
    │   ├── Dockerfile
    │   ├── entrypoint.sh
    │   └── config/
    │       ├── lsx10_custom.yaml
    │       └── lslidar_headless_launch.py
    ├── apriltag/
    └── zenoh-router/
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

### 1. Структура папок сервисов

Каждый кастомный сервис (контейнер) должен находиться в отдельной папке:

```
<service_name>/
├── Dockerfile           # ОБЯЗАТЕЛЬНО
├── entrypoint.sh        # Опционально - если нужна логика при старте
└── config/              # Опционально - для специфичных конфигов
    ├── <service>.yaml
    ├── <service>_launch.py
    └── другие файлы
```

**Правила:**
- Название папки = название сервиса в `docker-compose.yaml`
- Один Dockerfile на сервис
- `entrypoint.sh` для логики инициализации (проверка Zenoh router, wait-for, etc)
- `config/` только для файлов, специфичных для этого сервиса
- Общие конфиги (Zenoh, CycloneDDS) всегда в `docker/<env>/config/`

### 2. Правила volumes

#### Общее правило монтирования конфигов:

```yaml
volumes:
  - ./config:/config  # ВСЕГДА монтируем общую папку config
```

**НЕ дублируем** отдельные файлы из config:
```yaml
# ❌ ПЛОХО - дублирование
volumes:
  - ./config:/config
  - ./config/zenoh_session_config.json5:/config/zenoh_session_config.json5  # Избыточно!
  - ./config/rtabmap.yaml:/config/rtabmap.yaml  # Избыточно!

# ✅ ХОРОШО - чисто и просто
volumes:
  - ./config:/config
  - ./maps:/maps  # Дополнительные volume только для persistent данных
```

#### Специфичные конфиги в образе:

Если сервису нужны специфичные конфиги/launch файлы, копируем их в образ при сборке:

```dockerfile
# Пример из lslidar/Dockerfile
COPY lslidar/config/lslidar_headless_launch.py /ws/src/lslidar_ros2_driver/lslidar_driver/launch/
COPY lslidar/config/lsx10_custom.yaml /config/lsx10_custom.yaml
```

**Правило выбора: volume vs COPY:**
- **COPY в образ** - если файл специфичен для сервиса и редко меняется (launch файлы, default configs)
- **Volume монтирование** - если файл общий для нескольких сервисов или часто редактируется

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

### 5. Скрипты

#### Расположение:
- `docker/<env>/scripts/` - утилиты для управления (update, monitor, diagnose)
- `docker/<env>/<service>/entrypoint.sh` - скрипт запуска контейнера
- `docker/<env>/config/start_*.sh` - НЕТ! Переносим в `scripts/`

#### Структура entrypoint.sh:

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
source /ws/install/setup.bash

# 3. Запуск основного процесса
exec ros2 launch <package> <launch_file>
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
- [ ] Dockerfile находится в папке сервиса
- [ ] Общие конфиги в `config/`, специфичные в `<service>/config/`
- [ ] Volume монтирование: только `./config:/config`, без дублей
- [ ] Стандартный набор environment переменных
- [ ] `network_mode: host`
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
