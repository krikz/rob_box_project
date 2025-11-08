# Программное обеспечение РОББОКС

<div align="center">
  <strong>Полная документация программных компонентов робота</strong>
  <br>
  <sub>Версия: 1.0.0 | Дата: 2025-10-12</sub>
</div>

---

## 📖 Содержание

1. [Обзор программной платформы](#1-обзор-программной-платформы)
2. [ROS 2 пакеты](#2-ros-2-пакеты)
3. [Docker сервисы](#3-docker-сервисы)
4. [Middleware и связь](#4-middleware-и-связь)
5. [Алгоритмы и библиотеки](#5-алгоритмы-и-библиотеки)
6. [Конфигурация и параметры](#6-конфигурация-и-параметры)

---

## 1. Обзор программной платформы

### 1.1. Технологический стек

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryColor':'#e8f4f8','primaryTextColor':'#000','primaryBorderColor':'#2c5282'}}}%%
graph TD
    subgraph App["📱 Application Layer"]
        Nav2[Nav2<br/>Navigation]
        RTAB[RTAB-Map<br/>SLAM]
        Anim[Animations<br/>Manager]
        Behav[Behaviors<br/>Future]
    end
    
    subgraph Middleware["⚙️ ROS 2 Middleware (Zenoh)"]
        Topics[Topics | Services | Actions | Parameters | TF]
    end
    
    subgraph Drivers["🔌 Drivers & Hardware Abstraction"]
        LiDARDrv[LiDAR<br/>Driver]
        OAKDrv[OAK-D<br/>Driver]
        VESCDrv[VESC<br/>Driver]
        LEDDrv[LED<br/>Matrix]
    end
    
    subgraph Hardware["🔩 Hardware Layer"]
        Sensors[Sensors | Cameras | Motors | LED Panels]
    end
    
    App --> Middleware
    Middleware --> Drivers
    Drivers --> Hardware
    
    style App fill:#e8f4f8,stroke:#2c5282,stroke-width:2px
    style Middleware fill:#fff3cd,stroke:#856404,stroke-width:2px
    style Drivers fill:#d4edda,stroke:#28a745,stroke-width:2px
    style Hardware fill:#cccccc,stroke:#666666,stroke-width:2px
```

### 1.2. Основные технологии

| Компонент | Технология | Версия | Назначение |
|-----------|------------|--------|------------|
| **OS** | Ubuntu Server | 22.04 ARM64 | Базовая ОС для Raspberry Pi |
| **ROS** | ROS 2 Humble | 2023 LTS | Робототехический фреймворк |
| **Middleware** | Zenoh | 0.11+ | Распределённая связь |
| **Контейнеризация** | Docker | 24.0+ | Изоляция сервисов |
| **Оркестрация** | Docker Compose | 2.20+ | Управление контейнерами |
| **SLAM** | RTAB-Map | 0.21+ | Картография и локализация |
| **Navigation** | Nav2 | Humble | Планирование траектории |
| **Vision** | DepthAI / OpenCV | Latest | Обработка изображений |
| **Build System** | colcon | Latest | Сборка ROS 2 пакетов |

---

## 2. ROS 2 пакеты

### 2.1. rob_box_bringup

**Описание**: Launch-файлы для запуска всей системы робота.

#### Структура пакета
```
rob_box_bringup/
├── launch/
│   ├── display.launch.py          # Визуализация URDF в RViz
│   ├── control.launch.py          # Запуск системы управления
│   └── full_system.launch.py      # Полный запуск всех компонентов
├── config/
│   └── rviz_config.rviz           # Конфигурация RViz
└── package.xml
```

#### Основные launch-файлы

**display.launch.py**
```python
# Запуск robot_state_publisher + joint_state_publisher + RViz
ros2 launch rob_box_bringup display.launch.py
```

**control.launch.py**
```python
# Запуск ros2_control контроллеров
ros2 launch rob_box_bringup control.launch.py
```

#### Зависимости
- `rob_box_description` (URDF модель)
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`

---

### 2.2. rob_box_description

**Описание**: URDF описание робота (геометрия, кинематика, сенсоры).

#### Структура пакета
```
rob_box_description/
├── urdf/
│   ├── rob_box.urdf.xacro         # Главный URDF файл
│   ├── robot.gazebo.xacro         # Gazebo плагины (future)
│   └── materials.xacro            # Материалы для визуализации
├── meshes/
│   ├── chassis.stl                # STL модели компонентов
│   ├── wheel_left.stl
│   └── wheel_right.stl
├── config/
│   └── ros2_control.yaml          # Конфигурация ros2_control
└── package.xml
```

#### Ключевые параметры URDF

| Параметр | Значение | Описание |
|----------|----------|----------|
| `wheelbase` | 0.40 m | Расстояние между колёсами |
| `wheel_radius` | 0.125 m | Радиус колеса |
| `wheel_width` | 0.05 m | Ширина колеса |
| `chassis_length` | 0.50 m | Длина корпуса |
| `chassis_width` | 0.40 m | Ширина корпуса |
| `chassis_height` | 0.30 m | Высота корпуса |

#### Фреймы (TF tree)
```
base_link (корневой фрейм)
├── base_footprint (проекция на землю)
├── left_wheel_link
├── right_wheel_link
├── lidar_link (сенсор LS LiDAR)
├── camera_link (OAK-D камера)
│   ├── camera_rgb_frame
│   └── camera_depth_frame
└── imu_link (IMU - future)
```

---

### 2.3. rob_box_animations

**Описание**: Система управления LED-анимациями для 381 NeoPixel LED (4× 8×8 + 5× 5×5).

#### Структура пакета
```
rob_box_animations/
├── rob_box_animations/               # Python модули
│   ├── animation_loader.py           # Загрузчик YAML манифестов
│   ├── animation_player.py           # Движок воспроизведения
│   └── frame_renderer.py             # Генератор кадров
├── scripts/
│   ├── animation_player_node.py      # ROS 2 нода
│   ├── generate_animation_frames.py  # Генератор PNG кадров
│   └── visualize_animations.py       # Превью анимаций
├── animations/
│   ├── manifests/                    # YAML описания (v2.5)
│   │   ├── emergency/
│   │   │   ├── police_lights.yaml
│   │   │   ├── ambulance.yaml
│   │   │   └── road_service.yaml
│   │   ├── emotions/
│   │   │   ├── happy.yaml
│   │   │   ├── sad.yaml
│   │   │   ├── angry.yaml
│   │   │   ├── thinking.yaml
│   │   │   └── talking.yaml
│   │   ├── navigation/
│   │   │   ├── turn_left.yaml
│   │   │   ├── turn_right.yaml
│   │   │   └── braking.yaml
│   │   └── system/
│   │       ├── idle.yaml
│   │       ├── charging.yaml
│   │       └── error.yaml
│   └── frames/                       # PNG кадры (8×8, 25×5)
│       ├── police/
│       ├── emotions/
│       └── ...
├── launch/
│   └── animation_player.launch.py
└── package.xml
```

#### Физическая конфигурация LED

**Wheel Panels (4× 8×8 = 256 LED)**:
- **FL/FR** (Front Left/Right) — Передние "глаза" / фары
- **RL/RR** (Rear Left/Right) — Задние стоп-сигналы

**Main Display (5× 5×5 = 125 LED)**:
- Объединены в 5×25 пикселей
- Используется для текста, эмоций, сообщений

**Всего: 381 адресуемых LED**

#### Доступные анимации (21+)

##### 🚨 Emergency (Экстренные службы)
| Анимация | Описание | Особенности |
|----------|----------|-------------|
| `police_lights` | Синие/красные мигалки + статичный текст "ПОЛИЦИЯ" | Custom cyrillic font 3×5px |
| `ambulance` | Красно-белая мигалка + крест | Высокая яркость |
| `fire_truck` | Красно-оранжевые огни + "FIRE" | Эффект пламени |
| `road_service` | Жёлтая синхронная пульсация | Яркость 100-255 синусоида |

##### 🧭 Navigation (Навигация)
| Анимация | Описание | FPS |
|----------|----------|-----|
| `turn_left` | Левый поворотник (FL/RL orange) + анимированная стрелка | 10 |
| `turn_right` | Правый поворотник (FR/RR orange) + анимированная стрелка | 10 |
| `braking` | Белые фары (расширение) + красные стоп-сигналы + "STOP" | 10 |
| `accelerating` | Нарастающие фары + оранжевое свечение | 10 |

##### 😊 Emotions (Эмоции)
| Анимация | Глаза (FL/FR) | Рот (Display) | Задние (RL/RR) |
|----------|---------------|---------------|----------------|
| `idle` | Эффект дыхания (0.5→1.0) | Нейтральное | Мягкое свечение |
| `happy` | Радужные глаза | Улыбка | Радужная пульсация |
| `sad` | Синие грустные | Грустный рот | Синий пульс |
| `angry` | Красная пульсация (255→150) | Сердитое лицо | Красная пульсация |
| `surprised` | Широко открытые | O-образный рот | Жёлтое мигание |
| `thinking` | Боковой взгляд | Точки "..." | Фиолетовая пульсация |
| `talking` | Моргание | Анимированный рот | Циановая пульсация |
| `victory` | Радужный цикл | Галочка ✓ | Радужный цикл |

##### ⚙️ System (Системные)
| Анимация | Описание | Использование |
|----------|----------|---------------|
| `sleep` | Закрытые глаза + ZZZ + тусклая пульсация | Режим ожидания |
| `wakeup` | Открывающиеся глаза + зевок | Выход из сна |
| `charging` | Нейтральные глаза + батарея | Процесс зарядки |
| `low_battery` | Тусклое красное мигание + критическая батарея | Предупреждение |
| `error` | X-глаза + "ERROR" + красное мигание | Системная ошибка |

#### Формат манифеста (v2.5)

```yaml
name: "police_lights"
version: "2.5"
duration_ms: 1000
loop: true
fps: 10

panels:
  - logical_group: "wheel_front_left"    # 8×8 LED
    offset_ms: 0
    frames:
      - image: "frames/police/fl_01.png"
        duration_ms: 100

  - logical_group: "main_display"        # 25×5 LED
    offset_ms: 0
    frames:
      - image: "frames/police/display_01.png"
        duration_ms: 100

metadata:
  priority: "high"
  category: "emergency"
  tags: ["police", "lights", "emergency"]
```

#### Генерация кадров

```bash
# Сгенерировать все анимации (600+ кадров)
python3 scripts/generate_animation_frames.py --animation all

# Конкретная анимация
python3 scripts/generate_animation_frames.py --animation happy

# Превью анимаций
python3 scripts/visualize_animations.py
```

**Cyrillic Font**: Custom 3×5 pixel шрифт для текста "ПОЛИЦИЯ"

#### ROS 2 интерфейс

**Публикуемые топики**:
```
/panel_image (sensor_msgs/Image)
  - frame_id: "wheel_front_left", "wheel_front_right", и т.д.
  - encoding: rgb8
  - Размеры: 8×8 (wheels) или 25×5 (display)
```

**Сервисы**:
```bash
# Список доступных анимаций
ros2 service call /animation_player/list_animations std_srvs/srv/Trigger

# Загрузить анимацию
ros2 service call /animation_player/load_animation std_msgs/srv/String \
  "data: 'police_lights'"

# Воспроизведение
ros2 service call /animation_player/play std_srvs/srv/Trigger

# Стоп
ros2 service call /animation_player/stop std_srvs/srv/Trigger
```

**Топики статуса**:
- `/animation_player/status` (std_msgs/String) — Текущее состояние плеера
- `/animation_player/current_animation` (std_msgs/String) — Активная анимация

---

### 2.4. robot_sensor_hub_msg

**Описание**: Кастомные ROS 2 сообщения для ESP32 сенсорного хаба.

#### Типы сообщений

**DeviceData.msg**
```
uint8 device_type      # 0=AHT30, 1=HX711, 2=FAN
uint8 device_id        # ID устройства
uint8 data_type        # 1=temp, 2=humidity, 3=weight, 4=speed, 5=rpm
float32 value          # Значение
uint8 error_code       # 0=OK, >0=ошибка
```

**DeviceCommand.msg**
```
uint8 device_type      # Тип устройства
uint8 device_id        # ID устройства
uint8 command_code     # 0=SET_SPEED, 1=TARE_SCALE
float32 param_1        # Параметр команды
float32 param_2        # Дополнительный параметр
```

**DeviceSnapshot.msg**
```
int64 timestamp        # Временная метка (ns)
DeviceData[] devices   # Массив данных от всех устройств
```

#### ROS 2 топики

**Публикуемые ESP32**:
- `/device/snapshot` (robot_sensor_hub_msg/DeviceSnapshot) @ 1 Hz

**Подписываемые ESP32**:
- `/device/command` (robot_sensor_hub_msg/DeviceCommand)

#### Примеры использования

```bash
# Просмотр всех данных сенсоров
ros2 topic echo /device/snapshot

# Установить скорость вентилятора 1 на 75%
ros2 topic pub --once /device/command robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 2, device_id: 0, command_code: 0, param_1: 0.75, param_2: 0.0}"

# Обнулить весы (tare)
ros2 topic pub --once /device/command robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 1, device_id: 0, command_code: 1, param_1: 0.0, param_2: 0.0}"
```

---

### 2.5. vesc_nexus (Git Submodule)

**Описание**: Драйвер для управления VESC контроллерами по CAN-шине.

#### Возможности
- ✅ Управление несколькими VESC (до 4+) по CAN
- ✅ Поддержка нескольких CAN-интерфейсов (can0, can1)
- ✅ Приём `/cmd_vel` → преобразование в RPM/Current
- ✅ Публикация одометрии `/odom`
- ✅ Мониторинг состояния моторов (ток, температура, напряжение)

#### Конфигурация (vesc_nexus_config.yaml)

```yaml
/**:
  ros__parameters:
    can_interface: "can0"
    vesc_ids: [0, 1]                # ID VESC на CAN шине
    wheel_labels: ["left", "right"] # Метки колёс
    publish_rate: 50.0              # Частота публикации состояния (Hz)
    
    # Параметры робота
    wheel_radius: 0.125             # Радиус колеса (м)
    wheelbase: 0.40                 # Расстояние между колёсами (м)
    gear_ratio: 1.0                 # Передаточное число редуктора
    
    # Лимиты
    speed_max: 23250.0              # Макс RPM
    current_max: 10.0               # Макс ток (А)
    brake_max: 200000.0             # Макс тормоз
```

#### ROS 2 интерфейс

**Подписываемые топики**:
- `/cmd_vel` (geometry_msgs/Twist) — Команды скорости

**Публикуемые топики**:
- `/odom` (nav_msgs/Odometry) @ 50 Hz — Одометрия
- `/vesc/sensors/core` (vesc_msgs/VescStateStamped) — Состояние моторов
- `/joint_states` (sensor_msgs/JointState) — Состояние сочленений

#### Запуск
```bash
# Поднять CAN интерфейс
sudo ip link set can0 up type can bitrate 500000

# Запуск драйвера
ros2 launch vesc_nexus vesc_driver.launch.py
```

---

### 2.6. ros2leds (Git Submodule)

**Описание**: Система управления NeoPixel LED матрицами через ROS 2 (Python/C).

**Репозиторий**: https://github.com/krikz/ros2leds

#### Возможности
- ✅ Управление 381 WS2812B LED (4× 8×8 + 5× 5×5)
- ✅ SPI интерфейс через GPIO 10 (MOSI)
- ✅ 24-bit цвет RGB (16.7 млн цветов)
- ✅ Композитор для объединения панелей в логические группы
- ✅ Поддержка snake/flip/arrangement трансформаций

#### Архитектура

```
[Applications] 
    │ /panel_image (sensor_msgs/Image)
    │ frame_id = логическая группа
    ↓
[led_matrix_compositor] 
    │ - Композитор логических групп
    │ - Поддержка snake/flip/arrangement
    │ - Объединение физических панелей
    │ /led_matrix/data (std_msgs/Int8MultiArray)
    ↓
[led_matrix_driver]
    │ - Низкоуровневый SPI драйвер
    │ - pi5neo библиотека (Raspberry Pi 5)
    ↓
[381 WS2812B LEDs]
```

#### Физическая конфигурация

**4× Headlight Panels (8×8) = 256 LEDs**
- Front Left, Front Right (передние фары)
- Rear Left, Rear Right (задние стоп-сигналы)

**5× Main Display Panels (5×5) = 125 LEDs**
- Объединены горизонтально: 5 строк × 25 колонок
- Эмоции, текст, анимации

**Итого: 381 LED**

#### ROS 2 интерфейс

**Input топики**:
```
/panel_image (sensor_msgs/Image, 8×8 или 5×25, RGB8)
  - frame_id определяет логическую группу
  - Примеры: "headlights_front", "main_display"
```

**Output топик**:
```
/led_matrix/data (std_msgs/Int8MultiArray)
  - Сырые RGB данные для всех 381 LED (1143 байта)
```

**Services**:
```bash
# Установить яркость (0-100%)
ros2 service call /led_matrix/set_brightness std_srvs/srv/SetInt "{data: 50}"

# Выключить все LED
ros2 service call /led_matrix/clear std_srvs/srv/Trigger
```

#### Библиотека
- **pi5neo**: Python библиотека для WS2812B на Raspberry Pi 5
- **SPI скорость**: 800 kHz (стандарт WS2812B)
- **Питание**: Внешний БП 5V/25A (НЕ от Raspberry Pi!)

---

## 3. Docker сервисы

### 3.1. Main Pi сервисы

#### zenoh-router (eclipse/zenoh:1.6.2)
- **Назначение**: Центральный роутер для связи между Pi
- **Режим**: `peer` (может работать автономно)
- **Порты**: 7447 (TCP/UDP), 8000 (HTTP REST API)
- **Конфигурация**: `config/zenoh_router_config.json5`
- **Healthcheck**: `wget http://localhost:8000/@/local/router`

#### twist-mux
- **Образ**: `ghcr.io/krikz/rob_box:twist-mux-humble-latest`
- **Назначение**: Мультиплексирование команд скорости с приоритетами
- **Приоритеты**: 1. Nav2, 2. Teleop, 3. Joystick
- **Конфигурация**: `config/twist_mux/twist_mux.yaml`

#### rtabmap
- **Образ**: `ghcr.io/krikz/rob_box:rtabmap-humble-latest`
- **Назначение**: SLAM картография и локализация
- **Входы**: `/scan`, `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/odom`
- **Выходы**: `/map`, `/rtabmap/localization_pose`, `/rtabmap/cloud_map`
- **Persistent данные**: `./maps:/root/.ros/rtabmap`

#### lslidar
- **Образ**: `ghcr.io/krikz/rob_box:lslidar-humble-latest`
- **Назначение**: Драйвер LS LiDAR N10
- **Выход**: `/scan` (sensor_msgs/LaserScan) @ 10 Hz
- **Параметры**: `config/lslidar/lslidar.yaml`

#### nav2 (future)
- **Образ**: `ghcr.io/krikz/rob_box:nav2-humble-latest`
- **Назначение**: Автономная навигация
- **Компоненты**: planner, controller, bt_navigator, lifecycle_manager
- **Параметры**: `config/nav2/nav2_params.yaml`

#### vesc-driver
- **Образ**: `ghcr.io/krikz/rob_box:vesc-humble-latest`
- **Назначение**: Управление VESC по CAN
- **Устройство**: `/dev/spidev0.0` (MCP2515 CAN hat)
- **Privileged**: Да (доступ к SPI)

#### micro-ros-agent
- **Образ**: `ghcr.io/krikz/rob_box:micro-ros-agent-humble-latest`
- **Назначение**: Мост между ESP32 (Micro-ROS) и ROS 2
- **Устройство**: `/dev/ttyUSB0` (ESP32 Serial)
- **Скорость**: 115200 baud

---

### 3.2. Vision Pi сервисы

#### zenoh-router (eclipse/zenoh:1.6.2)
- **Назначение**: Локальный роутер (client mode)
- **Режим**: `client` (подключается к Main Pi)
- **Конфигурация**: `config/zenoh_router_config.json5`
- **Connect**: `tcp/10.1.1.10:7447`

#### oak-d
- **Образ**: `ghcr.io/krikz/rob_box:oak-d-humble-latest`
- **Назначение**: Драйвер OAK-D Lite камеры
- **Выходы**: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, camera_info
- **Устройства**: `/dev/bus/usb` (privileged mode)
- **Конфигурация**: `config/oak-d/oak_d_config.yaml`

#### apriltag
- **Образ**: `ghcr.io/krikz/rob_box:apriltag-humble-latest`
- **Назначение**: Детекция AprilTag маркеров
- **Вход**: `/camera/color/image_raw`, `/camera/color/camera_info`
- **Выходы**: `/apriltag/detections`, `/tf` (трансформы маркеров)
- **Семейство тегов**: tag36h11

---

## 4. Middleware и связь

### 4.1. Zenoh

**Описание**: Высокопроизводительный pub/sub middleware для ROS 2.

#### Архитектура

```
Main Pi (peer mode)                Vision Pi (client mode)
┌─────────────────┐              ┌─────────────────┐
│ Zenoh Router    │◄──TCP/UDP────┤ Zenoh Router    │
│ (10.1.1.10:7447)│              │ (10.1.1.11:7447)│
└────────┬────────┘              └────────┬────────┘
         │                                │
    ┌────▼─────┐                     ┌────▼─────┐
    │ROS Nodes │                     │ROS Nodes │
    │(Zenoh RMW│                     │(Zenoh RMW│
    └──────────┘                     └──────────┘
         │
         │ TCP/TLS (опционально)
         ▼
┌──────────────────┐
│zenoh.robbox.online│ (Облако)
│      :7447        │
└──────────────────┘
```

#### Преимущества Zenoh
| Критерий | Zenoh | CycloneDDS |
|----------|-------|------------|
| **CPU overhead** | <5% | ~20% |
| **Bandwidth** | Сжатие встроено | Нет сжатия |
| **WAN** | Простая настройка | Сложная (multicast) |
| **Масштабируемость** | Сотни узлов | 10-20 узлов |

#### Конфигурация (zenoh_router_config.json5)

**Main Pi (peer mode)**:
```json5
{
  mode: "peer",
  listen: {
    endpoints: ["tcp/0.0.0.0:7447", "udp/0.0.0.0:7447"]
  },
  connect: {
    endpoints: ["tcp/zenoh.robbox.online:7447"]  // опционально
  },
  scouting: {
    multicast: { enabled: false }
  }
}
```

**Vision Pi (client mode)**:
```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/10.1.1.10:7447"]  // подключение к Main Pi
  }
}
```

---

### 4.2. rmw_zenoh_cpp

**Описание**: ROS 2 Middleware implementation для Zenoh.

#### Установка
```bash
sudo apt install ros-humble-rmw-zenoh-cpp
```

#### Использование
```bash
# Установить Zenoh как RMW
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Запустить ноду
ros2 run <package> <node>
```

#### Переменные окружения
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ZENOH_CONFIG=/config/zenoh_session_config.json5
```

---

## 5. Алгоритмы и библиотеки

### 5.1. RTAB-Map (SLAM)

**Описание**: Real-Time Appearance-Based Mapping — 3D SLAM с RGB-D и LiDAR.

#### Режим работы
- **RGBD Mode**: Использование RGB-D камеры + 2D LiDAR
- **Оптимизация**: Снижено разрешение и FPS для Raspberry Pi

#### Ключевые параметры оптимизации

```yaml
Mem/ImagePreDecimation: 2           # Уменьшение разрешения изображений
Mem/ImagePostDecimation: 1
Kp/MaxFeatures: 400                 # Визуальные фичи (было 1000)
Vis/MaxFeatures: 200                # Keypoints (было 400)
GFTT/MinDistance: 5                 # Расстояние между фичами
Vis/FeatureType: 6                  # GFTT (Fast, GPU-friendly)
Vis/CorType: 1                      # BRIEF дескриптор
Rtabmap/LoopThr: 0.11               # Порог loop closure
Rtabmap/TimeThr: 700                # Loop closure раз в 2 сек
Mem/STMSize: 10                     # Short-term memory (было unlimited)
Grid/RangeMax: 5.0                  # Максимальная дальность сетки
```

#### Входные топики
```
/scan                              (sensor_msgs/LaserScan)
/camera/color/image_raw            (sensor_msgs/Image)
/camera/depth/image_rect_raw       (sensor_msgs/Image)
/camera/color/camera_info          (sensor_msgs/CameraInfo)
/odom                              (nav_msgs/Odometry)
```

#### Выходные топики
```
/map                               (nav_msgs/OccupancyGrid) @ 1 Hz
/rtabmap/localization_pose         (geometry_msgs/PoseStamped)
/rtabmap/cloud_map                 (sensor_msgs/PointCloud2)
/rtabmap/odom                      (nav_msgs/Odometry)
```

---

### 5.2. Nav2 (Navigation Stack)

**Описание**: Стек автономной навигации для ROS 2.

#### Компоненты
- **Planner Server**: Глобальное планирование (A*, Dijkstra, Theta*)
- **Controller Server**: Локальное управление (DWB, TEB, RPP)
- **Behavior Server**: Базовые поведения (spin, backup, wait)
- **BT Navigator**: Behavior Tree навигация
- **Lifecycle Manager**: Управление жизненным циклом нод

#### Статус
🚧 **В разработке** — планируется в следующих релизах.

---

### 5.3. AprilTag Detection

**Описание**: Детекция визуальных маркеров AprilTag для точной локализации.

#### Семейство тегов
- **tag36h11** (по умолчанию) — 36-битные теги с высокой надёжностью
- Размер маркера: настраивается (обычно 0.15-0.20 м)

#### Использование
```bash
# Просмотр детекций
ros2 topic echo /apriltag/detections

# Просмотр TF трансформаций
ros2 run tf2_tools view_frames
```

#### Применение
- 🎯 **Точное позиционирование**: Dock станции, целевые точки
- 🗺️ **Калибровка карты**: Привязка SLAM карты к реальным координатам
- 🤝 **Human-robot interaction**: Метки для взаимодействия с пользователем

---

## 6. Конфигурация и параметры

### 6.1. Общие переменные окружения

```bash
# ROS 2
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG=/config/zenoh_session_config.json5

# Логирование
export RUST_LOG=zenoh=info
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

### 6.2. Структура конфигурационных файлов

```
docker/main/config/
├── zenoh_router_config.json5       # Zenoh router (Main Pi)
├── zenoh_session_config.json5      # Zenoh session для ROS нод
├── cyclonedds.xml                  # CycloneDDS (fallback)
├── twist_mux/
│   └── twist_mux.yaml
├── rtabmap/
│   └── rtabmap_config.yaml
├── lslidar/
│   └── lslidar.yaml
├── nav2/
│   └── nav2_params.yaml
└── vesc/
    └── vesc_nexus_config.yaml
```

---

## 📚 Связанные документы

- [Архитектура системы](SYSTEM_OVERVIEW.md)
- [Аппаратное обеспечение](HARDWARE.md)
- [Развёртывание](../deployment/README.md)
- [Стандарты Docker](../development/DOCKER_STANDARDS.md)
- [CI/CD Pipeline](../CI_CD_PIPELINE.md)

---

<div align="center">
  <sub>Версия: 1.0.0 | Дата: 2025-10-12 | Автор: КУКОРЕКЕН</sub>
</div>
