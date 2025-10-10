# Архитектура системы робота

```
┌─────────────────────────────────────────────────────────────────┐
│                         РОБОТ - ROS 2                           │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────────────┐         ┌──────────────────────────┐
│  Raspberry Pi #1         │         │  Raspberry Pi #2         │
│  (Камера OAK-D)          │◄───────►│  (SLAM RTAB-Map)         │
│                          │ Ethernet│                          │
│  IP: 10.1.1.11 (пример)  │   or    │  IP: 10.1.1.10 (пример)  │
│                          │  WiFi   │                          │
└──────────────────────────┘         └──────────────────────────┘
        │                                     │
        │                                     │
     [USB 3.0]                           [получает]
        │                                     │
        ▼                                     ▼
┌──────────────────┐              ┌────────────────────┐
│  OAK-D-Lite      │  публикует   │  Обработка SLAM    │
│  Camera          ├──────────────►│  Построение карты  │
│                  │   топики      │  Локализация       │
└──────────────────┘   ROS 2       └────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 ТОПИКИ (CycloneDDS)                    │
└─────────────────────────────────────────────────────────────────┘

Pi #1 публикует:                Pi #2 подписывается:
─────────────────               ────────────────────
/oak/rgb/image_raw/compressed   ──► RGB изображение (JPEG)
/oak/rgb/camera_info            ──► Калибровка RGB
/oak/stereo/image_raw/          ──► Карта глубины (PNG)
    compressedDepth
/oak/stereo/camera_info         ──► Калибровка depth

                                Pi #2 публикует:
                                ────────────────
                                /rtabmap/odom        ◄── Одометрия
                                /rtabmap/mapData     ◄── Карта
                                /rtabmap/grid_map    ◄── 2D карта
                                /rtabmap/cloud_map   ◄── 3D облако
```

## Поток данных

```
┌──────────┐
│ OAK-D    │ 5 FPS, 640x360 RGB + 640x400 Depth
│ Camera   │
└────┬─────┘
     │
     │ USB 3.0
     ▼
┌──────────────┐
│ DepthAI      │ Сжимает изображения:
│ ROS Driver   │ • RGB → JPEG (качество 80)
│ (Pi #1)      │ • Depth → PNG (уровень 3)
└────┬─────────┘
     │
     │ CycloneDDS через Ethernet
     │ ~8-15 Mbps (вместо 80-100 Mbps)
     ▼
┌──────────────┐
│ RTAB-Map     │ Обрабатывает:
│ SLAM         │ • Feature extraction (GFTT/BRIEF)
│ (Pi #2)      │ • Visual odometry
│              │ • Loop closure detection
│              │ • 3D mapping
└──────────────┘
     │
     │
     ▼
┌──────────────┐
│ База данных  │ SQLite: /maps/rtabmap.db
│ карты        │ Хранит граф, features, изображения
└──────────────┘
```

## Docker контейнеры

### Pi #1 - Vision System

```
docker/vision/
│
├── docker-compose.yaml          ► Конфигурация контейнеров
│
├── oak-d/
│   └── Dockerfile               ► Образ с DepthAI ROS driver
│
└── config/
    ├── oak_d_config.yaml        ► Настройки камеры
    ├── cyclonedds.xml           ► Настройки DDS
    └── start_oak_d.sh           ► Скрипт запуска
```

**Контейнер `oak-d`:**
- Базовый образ: `luxonis/depthai-ros:humble-latest`
- Привилегированный режим (доступ к USB)
- Host network (для CycloneDDS)
- Монтирует: USB устройства, конфиги

### Pi #2 - SLAM System

```
docker/main/
│
├── docker-compose.yaml          ► Конфигурация контейнеров
│
├── rtabmap/
│   └── Dockerfile               ► Образ с RTAB-Map
│
├── config/
│   ├── rtabmap_config.yaml      ► Параметры RTAB-Map
│   └── cyclonedds.xml           ► Настройки DDS
│
└── maps/                        ► Хранение карт
    └── rtabmap.db
```

**Контейнер `rtabmap`:**
- Базовый образ: `introlab3it/rtabmap_ros:humble-latest`
- Host network (для CycloneDDS)
- Монтирует: конфиги, maps
- QT_QPA_PLATFORM=offscreen (без GUI)

## Сетевая конфигурация

### CycloneDDS (RMW)

```
┌─────────────────────────────────────────────┐
│         CycloneDDS Discovery                │
│                                             │
│  Multicast: 239.255.0.1                     │
│  UDP Ports: 7400-7600                       │
│                                             │
│  ROS_DOMAIN_ID: 0 (на обоих Pi)             │
│                                             │
│  SPDPInterval: 5s (снижена нагрузка)        │
└─────────────────────────────────────────────┘
```

### Оптимизация буферов

**Pi #1 (передает больше):**
- SocketSendBufferSize: 1-2 MB
- SocketReceiveBufferSize: 2-4 MB
- Throttling: MaxBurst=20

**Pi #2 (принимает больше):**
- SocketSendBufferSize: 0.5-1 MB
- SocketReceiveBufferSize: 4-8 MB
- Throttling: MaxBurst=30

## Параметры производительности

### OAK-D камера (Pi #1)

| Параметр | Значение | Обоснование |
|----------|----------|-------------|
| FPS | 5 | Баланс качества/CPU |
| RGB разрешение | 640x360 | Достаточно для SLAM |
| Depth разрешение | 640x400 | Оптимально для Pi |
| JPEG качество | 80 | Хорошее качество |
| PNG сжатие | Level 3 | Быстрое сжатие |

**CPU нагрузка:** 30-45%  
**Memory:** ~600 MB  
**Bandwidth:** ~8-12 Mbps

### RTAB-Map (Pi #2)

| Параметр | Значение | Обоснование |
|----------|----------|-------------|
| Max Features | 400 | Достаточно для SLAM |
| Feature Type | GFTT/BRIEF | Быстрее SIFT/SURF |
| Detection Rate | 0.5 Hz | Раз в 2 секунды |
| STM Size | 10 | Ограничение памяти |
| Grid Range | 5m | Локальная карта |

**CPU нагрузка:** 40-60%  
**Memory:** ~800 MB  
**Loop closure:** каждые 2 сек

## Типичные проблемы и решения

### 1. Топики не видны между Pi

**Причина:** Проблема с CycloneDDS discovery

**Решение:**
```bash
# Проверить multicast
ping 239.255.0.1

# Проверить ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # должно быть 0 на обоих

# Перезапустить daemon
ros2 daemon stop && ros2 daemon start
```

### 2. Высокая задержка

**Причина:** Сеть перегружена или медленная

**Решение:**
- Использовать Gigabit Ethernet (не WiFi)
- Проверить качество кабеля
- Убедиться что switch/router Gigabit

### 3. RTAB-Map не получает изображения

**Причина:** Нет compressed_image_transport плагина

**Решение:**
```bash
# В контейнере rtabmap
ros2 pkg list | grep compressed

# Если нет - пересобрать с плагинами
```

### 4. CPU overload

**Причина:** Параметры слишком высокие для Pi

**Решение:**
- Снизить FPS до 3
- Уменьшить features до 300
- Отключить grid mapping

## Мониторинг системы

### Команды для проверки

```bash
# Топики
ros2 topic list

# Частота
ros2 topic hz /oak/rgb/image_raw/compressed

# Bandwidth
ros2 topic bw /oak/rgb/image_raw/compressed

# CPU
htop

# Температура
vcgencmd measure_temp

# Сеть
iftop -i eth0
```

### Логи

```bash
# Pi #1
docker logs -f oak-d

# Pi #2
docker logs -f rtabmap

# Статистика RTAB-Map
ros2 topic echo /rtabmap/info
```

## Дальнейшее развитие

### Возможные улучшения

1. **Добавить IMU** для лучшей одометрии
2. **Добавить LiDAR** для более точных карт
3. **Настроить Nav2** для навигации
4. **Добавить систему управления** (teleop)
5. **Настроить визуализацию** (RViz2 на отдельном компьютере)

### Масштабирование

- **Третья Pi** для навигации и управления
- **Zenoh** для bridge с облаком
- **Разделение SLAM**: локализация отдельно от mapping

---

**Документация:**
- Детали оптимизации: `OPTIMIZATION_README.md`
- Быстрый старт: `QUICK_START_RU.md`
- Чеклист запуска: `CHECKLIST.md`
- Сводка изменений: `SUMMARY.md`
