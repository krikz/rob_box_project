# Карта распределения нод по дашбордам

Визуальная схема распределения ROS 2 нод по четырем демонстрационным дашбордам.

## 📊 Схема распределения

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryColor':'#e8f4f8','primaryTextColor':'#000','primaryBorderColor':'#2c5282'}}}%%
graph TB
    subgraph Dashboard1["🖥️ Dashboard 1: Системный обзор и ошибки"]
        D1_CPU["CPU/Memory Gauges<br/>Main Pi + Vision Pi"]
        D1_GRAPHS["CPU по контейнерам<br/>Графики в реальном времени"]
        D1_ERRORS["Все ошибки<br/>Единый лог всех контейнеров"]
    end
    
    subgraph Dashboard2["🖥️ Dashboard 2: Навигация и движение"]
        D2_RTABMAP["RTAB-Map<br/>SLAM картографирование"]
        D2_NAV2["Nav2 Stack<br/>8 нод планирования"]
        D2_CONTROL["ROS2 Control<br/>3 ноды моторов"]
        D2_TWIST["Twist Mux<br/>Приоритизация команд"]
        D2_LIDAR["LSLIDAR<br/>2D сканер"]
        D2_TF["Robot State Publisher<br/>TF трансформации"]
    end
    
    subgraph Dashboard3["🖥️ Dashboard 3: Восприятие и сенсоры"]
        D3_OAKD["OAK-D Camera<br/>RGB-D + AprilTag"]
        D3_CEILING["Ceiling Camera<br/>USB обзор сверху"]
        D3_MICROROS["micro-ROS Agent<br/>ESP32 Sensor Hub"]
        D3_PERCEPTION["Perception System<br/>5 нод анализа"]
    end
    
    subgraph Dashboard4["🖥️ Dashboard 4: Голос и интерфейс"]
        D4_VOICE["Voice Assistant<br/>7 нод голоса"]
        D4_LED["LED Matrix<br/>NeoPixel дисплей"]
        D4_ZENOH["Zenoh Routers<br/>Main + Vision Pi"]
    end
    
    style Dashboard1 fill:#fff3cd,stroke:#856404,stroke-width:3px
    style Dashboard2 fill:#d4edda,stroke:#28a745,stroke-width:3px
    style Dashboard3 fill:#cce5ff,stroke:#004085,stroke-width:3px
    style Dashboard4 fill:#f8d7da,stroke:#721c24,stroke-width:3px
```

## 🎯 Детальная карта нод

### Dashboard 1: Системный обзор и ошибки

**Тип:** Мониторинг инфраструктуры  
**Обновление:** 10 секунд

| Компонент | Метрики | Источник |
|-----------|---------|----------|
| Main Pi CPU | Gauge (%) | Prometheus |
| Main Pi Memory | Gauge (%) | Prometheus |
| Vision Pi CPU | Gauge (%) | Prometheus |
| Vision Pi Memory | Gauge (%) | Prometheus |
| CPU по контейнерам | Timeseries | cAdvisor |
| Все ошибки | Logs | Loki |

---

### Dashboard 2: Навигация и движение

**Контейнеры:** 6  
**Ноды:** ~18

```
rtabmap [Main Pi]
├── rtabmap (SLAM core)
├── rtabmapviz (visualizer)
└── icp_odometry (ICP одометрия)

nav2 [Main Pi]
├── bt_navigator (behavior tree)
├── controller_server (DWB контроллер)
├── planner_server (планировщик пути)
├── smoother_server (сглаживание пути)
├── waypoint_follower (следование точкам)
├── lifecycle_manager (менеджер жизненного цикла)
├── velocity_smoother (сглаживание скорости)
└── behavior_server (поведенческие действия)

ros2-control [Main Pi]
├── ros2_control_node (main node)
├── diff_drive_controller (differential drive)
└── joint_state_broadcaster (публикация состояний)

twist-mux [Main Pi]
└── twist_mux (мультиплексор команд)

lslidar [Main Pi]
└── lslidar_node (N10 LiDAR драйвер)

robot-state-publisher [Main Pi]
└── robot_state_publisher (TF публикация)
```

---

### Dashboard 3: Восприятие и сенсоры

**Контейнеры:** 4  
**Ноды:** ~10

```
oak-d [Vision Pi]
├── oak (OAK-D camera driver)
├── oakd (camera publisher)
└── apriltag_node (AprilTag detector)

ceiling-camera [Vision Pi]
├── usb_cam (USB camera driver)
└── ceiling_camera (camera publisher)


perception [Main Pi]
├── health_monitor (система мониторинга)
├── context_aggregator (агрегатор контекста)
├── reflection_node (внутренний диалог)
├── vision_stub (обработка видео)
└── startup_greeting (приветствие)
```

---

### Dashboard 4: Голос и интерфейс

**Контейнеры:** 4  
**Ноды:** ~10

```
voice-assistant [Vision Pi]
├── audio_node (обработка аудио ReSpeaker)
├── led_node (LED индикаторы ReSpeaker)
├── dialogue_node (диалоговая система DeepSeek)
├── tts_node (синтез речи Silero v4)
├── stt_node (распознавание речи Vosk)
├── sound_node (звуковые эффекты)
└── command_node (команды навигации)

led-matrix [Vision Pi]
└── led_matrix_compositor (NeoPixel LED)

zenoh-router [Main Pi]
└── zenoh (DDS router)

zenoh-router-vision [Vision Pi]
└── zenoh (DDS router)
```

---

## 📍 Физическое расположение нод

### Main Pi (10.1.1.10)
- **Навигация:** rtabmap, nav2, robot-state-publisher, twist-mux, lslidar
- **Управление:** ros2-control
- **Восприятие:** perception (health, context, reflection)

### Vision Pi (10.1.1.11)
- **Камеры:** oak-d, ceiling-camera
- **Голос:** voice-assistant (7 nodes)
- **Интерфейс:** led-matrix
- **Связь:** zenoh-router-vision

---

## 🔄 Поток данных между дашбордами

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryColor':'#e8f4f8','primaryTextColor':'#000','primaryBorderColor':'#2c5282'}}}%%
graph LR
    D3[Dashboard 3<br/>Сенсоры] -->|RGB-D<br/>LaserScan| D2[Dashboard 2<br/>Навигация]
    D4[Dashboard 4<br/>Голос] -->|Команды| D2
    D2 -->|Статус| D3
    D2 -->|Ошибки| D1[Dashboard 1<br/>Обзор]
    D3 -->|Ошибки| D1
    D4 -->|Ошибки| D1
    
    style D1 fill:#fff3cd,stroke:#856404
    style D2 fill:#d4edda,stroke:#28a745
    style D3 fill:#cce5ff,stroke:#004085
    style D4 fill:#f8d7da,stroke:#721c24
```

**Легенда:**
- Dashboard 1 — агрегация метрик и ошибок со всех систем
- Dashboard 2 — потребляет данные сенсоров и голосовые команды
- Dashboard 3 — поставляет сенсорные данные для навигации
- Dashboard 4 — отправляет голосовые команды в навигацию

---

## 🎨 Цветовая кодировка

Для быстрого визуального различения дашбордов:

| Dashboard | Цвет границы | HEX | Назначение |
|-----------|-------------|-----|------------|
| Dashboard 1 | 🟨 Желтый | #856404 | Системный обзор |
| Dashboard 2 | 🟩 Зеленый | #28a745 | Навигация |
| Dashboard 3 | 🟦 Синий | #004085 | Восприятие |
| Dashboard 4 | 🟥 Красный | #721c24 | Голос |

---

## 📊 Статистика нод

| Категория | Контейнеров | Нод | Ключевые топики |
|-----------|-------------|-----|-----------------|
| **Навигация** | 6 | ~18 | /scan, /odom, /cmd_vel, /map |
| **Восприятие** | 4 | ~10 | /camera/*, /detections, /imu |
| **Голос** | 4 | ~10 | /audio/*, /dialogue/*, /led/* |
| **Всего** | 14 | ~38 | ~50+ топиков |

---

## 🔍 Использование

### Быстрый поиск ноды

**Навигация:**
```bash
# Найти ноду в Dashboard 2
ros2 node list | grep -E "(rtabmap|nav2|control|twist|lidar|state)"
```

**Восприятие:**
```bash
# Найти ноду в Dashboard 3
ros2 node list | grep -E "(oak|camera|health|context|reflection|vision)"
```

**Голос:**
```bash
# Найти ноду в Dashboard 4
ros2 node list | grep -E "(audio|led|dialogue|tts|stt|sound|command|zenoh)"
```

### Отладка по дашбордам

**Проблемы с движением?** → Смотрите Dashboard 2 (Навигация)  
**Проблемы с камерой?** → Смотрите Dashboard 3 (Восприятие)  
**Проблемы с голосом?** → Смотрите Dashboard 4 (Голос)  
**Общие ошибки?** → Смотрите Dashboard 1 (Обзор)

---

**Автор:** Rob Box Team  
**Дата:** 2025-11-01  
**Версия:** 1.0
