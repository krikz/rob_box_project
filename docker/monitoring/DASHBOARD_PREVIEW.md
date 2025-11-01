# Демонстрационные дашборды - Визуальное руководство

Визуализация 4 демонстрационных дашбордов для Rob Box.

## 🖥️ Расположение мониторов

Рекомендуемая конфигурация для 4 мониторов:

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃    Dashboard 1 (Желтый)    ┃    Dashboard 2 (Зеленый)   ┃
┃  Системный обзор & ошибки  ┃     Навигация & движение   ┃
┃         [Главный]          ┃                            ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃    Dashboard 3 (Синий)     ┃     Dashboard 4 (Красный)  ┃
┃   Восприятие & сенсоры     ┃      Голос & интерфейс     ┃
┃                            ┃                            ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

---

## 📊 Dashboard 1: Системный обзор и ошибки

**UID:** `rob_box_demo_1`  
**URL:** `http://10.1.1.10:3000/d/rob_box_demo_1?kiosk`  
**Цвет:** 🟨 Желтый (#856404)

### Макет панелей

```
┌─────────────────────────────────────────────────────────────┐
│                    Обзор системы                            │
├──────────────┬──────────────┬──────────────┬────────────────┤
│  Main Pi CPU │ Main Pi Mem  │ Vision Pi CPU│ Vision Pi Mem  │
│   [GAUGE]    │   [GAUGE]    │   [GAUGE]    │   [GAUGE]      │
│     45%      │     62%      │     38%      │     55%        │
└──────────────┴──────────────┴──────────────┴────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                  CPU по контейнерам                         │
├─────────────────────────────┬───────────────────────────────┤
│  Main Pi - CPU по           │  Vision Pi - CPU по           │
│  контейнерам [GRAPH]        │  контейнерам [GRAPH]          │
│  • rtabmap                  │  • oak-d                      │
│  • nav2                     │  • voice-assistant            │
│  • ros2-control             │  • led-matrix                 │
│  • lslidar                  │  • ceiling-camera             │
└─────────────────────────────┴───────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                       Все ошибки                            │
│  [LOGS PANEL - 16 rows]                                     │
│  2025-11-01 12:45:23  rtabmap       ERROR: Lost tracking    │
│  2025-11-01 12:45:20  oak-d         ERROR: Frame dropped    │
│  2025-11-01 12:45:15  nav2          WARN: Path blocked      │
│  ...                                                         │
└─────────────────────────────────────────────────────────────┘
```

**Ключевые метрики:**
- 4 Gauge панели (CPU/Memory для обоих Pi)
- 2 Timeseries графика (CPU по контейнерам)
- 1 Logs панель с фильтром ошибок

---

## 🗺️ Dashboard 2: Навигация и движение

**UID:** `rob_box_demo_2`  
**URL:** `http://10.1.1.10:3000/d/rob_box_demo_2?kiosk`  
**Цвет:** 🟩 Зеленый (#28a745)

### Макет панелей

```
┌─────────────────────────────────────────────────────────────┐
│                  Навигация и движение                       │
├─────────────────────────────────────────────────────────────┤
│  RTAB-Map [LOGS - 10 rows]                                  │
│  • rtabmap: Loop closure detected (12 -> 245)               │
│  • icp_odometry: Odometry updated                           │
│  • rtabmap: Map saved (3247 nodes)                          │
├─────────────────────────────────────────────────────────────┤
│  Nav2 Navigation Stack [LOGS - 10 rows]                     │
│  • bt_navigator: Following path to goal                     │
│  • controller_server: DWB controller active                 │
│  • planner_server: Planning trajectory                      │
├─────────────────────────────────────────────────────────────┤
│  ROS2 Control (Моторы) [LOGS - 10 rows]                     │
│  • ros2_control_node: Controllers loaded                    │
│  • diff_drive_controller: Velocity command received         │
│  • joint_state_broadcaster: Publishing joint states         │
├─────────────────────────────────────────────────────────────┤
│  Twist Mux [LOGS - 8 rows]       │  LSLIDAR [LOGS - 8 rows] │
│  • twist_mux: Input 2 active     │  • lslidar: 360° scan    │
│  • Priority: nav2 > joystick     │  • Range: 0.2-10m        │
├──────────────────────────────────┴──────────────────────────┤
│  Robot State Publisher (TF) [LOGS - 8 rows]                 │
│  • robot_state_publisher: TF tree published                 │
└─────────────────────────────────────────────────────────────┘
```

**Контейнеры:** rtabmap, nav2, ros2-control, twist-mux, lslidar, robot-state-publisher  
**Ноды:** ~18 ROS 2 нод

---

## 👁️ Dashboard 3: Восприятие и сенсоры

**UID:** `rob_box_demo_3`  
**URL:** `http://10.1.1.10:3000/d/rob_box_demo_3?kiosk`  
**Цвет:** 🟦 Синий (#004085)

### Макет панелей

```
┌─────────────────────────────────────────────────────────────┐
│                   Восприятие и сенсоры                      │
├─────────────────────────────────────────────────────────────┤
│  OAK-D Camera (с AprilTag) [LOGS - 10 rows]                 │
│  • oak: RGB-D stream @ 30fps                                │
│  • oakd: Depth estimation active                            │
│  • apriltag_node: Tags detected [12, 45]                    │
├─────────────────────────────────────────────────────────────┤
│  Ceiling Camera (USB) [LOGS - 8 rows]                       │
│  • usb_cam: Frame published @ 640x480                       │
│  • ceiling_camera: Compressed image transport               │
├─────────────────────────────────────────────────────────────┤
│  micro-ROS Agent (ESP32 Sensor Hub) [LOGS - 10 rows]        │
│  • micro_ros_agent: Serial connection @115200               │
│  • IMU data received: ax=0.1 ay=-0.05 az=9.8                │
│  • ToF sensors: front=1.2m, rear=0.8m                       │
├─────────────────────────────────────────────────────────────┤
│  Perception System [LOGS - 12 rows]                         │
│  • health_monitor: System status: HEALTHY                   │
│  • context_aggregator: Context updated @ 2Hz                │
│  • reflection_node: Internal dialogue active                │
│  • vision_stub: Processing camera frames                    │
│  • startup_greeting: "Привет! Я готов к работе"            │
└─────────────────────────────────────────────────────────────┘
```

**Контейнеры:** oak-d, ceiling-camera, micro-ros-agent, perception  
**Ноды:** ~10 ROS 2 нод

---

## 🎤 Dashboard 4: Голос и интерфейс

**UID:** `rob_box_demo_4`  
**URL:** `http://10.1.1.10:3000/d/rob_box_demo_4?kiosk`  
**Цвет:** 🟥 Красный (#721c24)

### Макет панелей

```
┌─────────────────────────────────────────────────────────────┐
│                    Голос и интерфейс                        │
├─────────────────────────────────────────────────────────────┤
│  Voice Assistant [LOGS - 14 rows]                           │
│  • audio_node: ReSpeaker audio @ 16kHz                      │
│  • led_node: LED pattern: listening                         │
│  • dialogue_node: User: "Поехали к кухне"                   │
│  • dialogue_node: Assistant: "Конечно, еду к кухне"         │
│  • tts_node: Silero TTS v4 synthesis                        │
│  • stt_node: Vosk recognition: "поехали к кухне"            │
│  • sound_node: Playing sound effect: beep_start             │
│  • command_node: Navigation command: go_to(kitchen)         │
├─────────────────────────────────────────────────────────────┤
│  LED Matrix (Дисплей) [LOGS - 8 rows]                       │
│  • led_matrix_compositor: Animation playing: smile          │
│  • NeoPixel: 256 LEDs updated @ 30fps                       │
├─────────────────────────────────────────────────────────────┤
│  Zenoh Router (Main Pi)          │  Zenoh Router (Vision Pi)│
│  [LOGS - 8 rows]                 │  [LOGS - 8 rows]         │
│  • zenoh: Router started         │  • zenoh: Router started │
│  • Connect: 10.1.1.11:7447       │  • Connect: 10.1.1.10    │
│  • Topics: 45 active             │  • Topics: 32 active     │
└──────────────────────────────────┴──────────────────────────┘
```

**Контейнеры:** voice-assistant, led-matrix, zenoh-router, zenoh-router-vision  
**Ноды:** ~10 ROS 2 нод

---

## 🎨 Цветовая кодировка панелей

### Dashboard 1 (Желтый)
- **Border:** `#856404`
- **Background:** `#fff3cd`
- **Назначение:** Системный обзор - первый взгляд на всю систему

### Dashboard 2 (Зеленый)
- **Border:** `#28a745`
- **Background:** `#d4edda`
- **Назначение:** Навигация - основная функция робота

### Dashboard 3 (Синий)
- **Border:** `#004085`
- **Background:** `#cce5ff`
- **Назначение:** Восприятие - сенсорный ввод

### Dashboard 4 (Красный)
- **Border:** `#721c24`
- **Background:** `#f8d7da`
- **Назначение:** Взаимодействие - голос и интерфейс

---

## 🚀 Быстрый запуск

### Шаг 1: Включить мониторинг на обоих Pi

```bash
# Main Pi
ssh ros2@10.1.1.20
cd ~/rob_box_project/docker/main
./scripts/enable_monitoring.sh

# Vision Pi
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

### Шаг 2: Открыть дашборды в браузере

**Полноэкранный (киоск) режим:**

```bash
# Monitor 1 (Top-Left) - Dashboard 1
http://10.1.1.10:3000/d/rob_box_demo_1?kiosk

# Monitor 2 (Top-Right) - Dashboard 2
http://10.1.1.10:3000/d/rob_box_demo_2?kiosk

# Monitor 3 (Bottom-Left) - Dashboard 3
http://10.1.1.10:3000/d/rob_box_demo_3?kiosk

# Monitor 4 (Bottom-Right) - Dashboard 4
http://10.1.1.10:3000/d/rob_box_demo_4?kiosk
```

**Обычный режим (с меню):**

```bash
http://10.1.1.10:3000/d/rob_box_demo_1
http://10.1.1.10:3000/d/rob_box_demo_2
http://10.1.1.10:3000/d/rob_box_demo_3
http://10.1.1.10:3000/d/rob_box_demo_4
```

### Шаг 3: Настроить параметры отображения

Для каждого дашборда:

1. **Временной диапазон:** "Last 15 minutes" (по умолчанию)
2. **Auto-refresh:** 10 секунд (по умолчанию)
3. **Theme:** Dark (по умолчанию)

Для смены темы: Settings (⚙️) → Preferences → Theme → Light/Dark

---

## 📐 Рекомендации по размещению

### Для 4 мониторов Full HD (1920x1080)

```
┌───────────────────┬───────────────────┐
│   Monitor 1       │   Monitor 2       │
│   Dashboard 1     │   Dashboard 2     │
│   1920x1080       │   1920x1080       │
│   (Main)          │   (Navigation)    │
├───────────────────┼───────────────────┤
│   Monitor 3       │   Monitor 4       │
│   Dashboard 3     │   Dashboard 4     │
│   1920x1080       │   1920x1080       │
│   (Perception)    │   (Voice)         │
└───────────────────┴───────────────────┘
```

**Общее разрешение:** 3840x2160 (4K)

### Для 2 мониторов Ultra-Wide (3440x1440)

```
┌───────────────────────────────────────┐
│         Monitor 1 (Top)               │
│  Dashboard 1 │ Dashboard 2            │
│  1720x1440   │ 1720x1440              │
├───────────────────────────────────────┤
│         Monitor 2 (Bottom)            │
│  Dashboard 3 │ Dashboard 4            │
│  1720x1440   │ 1720x1440              │
└───────────────────────────────────────┘
```

---

## 🔍 Интерактивные возможности

### Фильтрация логов

Для каждой logs панели можно применить дополнительные фильтры:

**Примеры LogQL запросов:**

```logql
# Только ошибки из конкретного контейнера
{container="rtabmap"} |~ "(?i)error"

# Исключить информационные сообщения
{container="nav2"} != "info"

# Только предупреждения и ошибки
{container="oak-d"} |~ "(?i)(warn|error)"

# Поиск по тексту
{container="voice-assistant"} |= "audio_node"

# Комбинированный фильтр
{container="perception"} |~ "(?i)(error|exception)" != "debug"
```

### Временные диапазоны

Быстрый выбор (правый верхний угол):
- Last 5 minutes
- Last 15 minutes (по умолчанию)
- Last 30 minutes
- Last 1 hour
- Last 3 hours

Кастомный диапазон:
- From: `2025-11-01 12:00:00`
- To: `2025-11-01 13:00:00`

### Zoom и navigation

- **Scroll wheel:** Прокрутка логов
- **Click & Drag (на графиках):** Zoom in на временной диапазон
- **Shift + Click & Drag:** Pan по временной оси
- **Double Click:** Zoom out

---

## 📊 Статистика по дашбордам

| Dashboard | Панелей | Gauge | Timeseries | Logs | Refresh | Data Source |
|-----------|---------|-------|------------|------|---------|-------------|
| 1. System | 8       | 4     | 2          | 2    | 10s     | Prom + Loki |
| 2. Navigation | 6   | 0     | 0          | 6    | 10s     | Loki        |
| 3. Perception | 4   | 0     | 0          | 4    | 10s     | Loki        |
| 4. Voice  | 4       | 0     | 0          | 4    | 10s     | Loki        |
| **TOTAL** | **22**  | **4** | **2**      | **16**| -      | -           |

---

## 🎬 Сценарий демонстрации

### Сценарий 1: Автономная навигация

**Фокус:** Dashboard 2 (Навигация)

1. Дать команду роботу: "Поехали к кухне"
2. **Dashboard 4** — показывает распознавание команды в voice-assistant
3. **Dashboard 2** — Nav2 планирует путь и начинает движение
4. **Dashboard 2** — RTAB-Map обновляет карту в реальном времени
5. **Dashboard 2** — ROS2 Control показывает команды моторам
6. **Dashboard 1** — CPU нагрузка увеличивается на графиках

### Сценарий 2: Обнаружение препятствия

**Фокус:** Dashboard 3 (Восприятие)

1. Поместить препятствие перед роботом
2. **Dashboard 3** — LSLIDAR обнаруживает препятствие
3. **Dashboard 3** — OAK-D камера видит объект
4. **Dashboard 2** — Nav2 перепланирует путь
5. **Dashboard 4** — Голосовое уведомление: "Объезжаю препятствие"

### Сценарий 3: Система мониторинга

**Фокус:** Dashboard 1 (Обзор)

1. Запустить все системы робота
2. **Dashboard 1** — Мониторы показывают рост CPU/Memory
3. **Dashboard 1** — Графики показывают нагрузку по контейнерам
4. Смоделировать ошибку (например, выключить камеру)
5. **Dashboard 1** — Панель ошибок показывает ERROR сообщения
6. **Dashboard 3** — OAK-D логи показывают connection lost

---

## 🛠️ Кастомизация

### Изменение размеров панелей

Панели расположены в grid system Grafana:
- **Ширина:** 0-24 units (полная ширина = 24)
- **Высота:** произвольная в units

Пример изменения высоты logs панели:
```json
"gridPos": {
  "h": 12,  // было 10, стало 12 (выше)
  "w": 24,
  "x": 0,
  "y": 10
}
```

### Добавление новых фильтров

В панели logs можно добавить переменные для динамической фильтрации:

1. Dashboard Settings → Variables
2. Add variable:
   - Name: `container`
   - Type: `Query`
   - Data source: `Loki`
   - Query: `label_values(container)`
3. Использовать в query: `{container="$container"}`

### Экспорт дашбордов

Для резервного копирования или переноса:

1. Dashboard Settings → JSON Model
2. Копировать JSON
3. Сохранить в файл `my_dashboard.json`
4. Импорт: Dashboards → Import → Upload JSON

---

## 📱 Мобильный доступ

Дашборды адаптивны и работают на мобильных устройствах:

**Смартфон (вертикальная ориентация):**
- Панели автоматически переставляются вертикально
- Рекомендуется Dashboard 1 для быстрого обзора

**Планшет (горизонтальная ориентация):**
- Полноценное отображение как на десктопе
- Можно использовать все 4 дашборда

**Мобильный URL:**
```
http://10.1.1.10:3000/d/rob_box_demo_1?kiosk&theme=light
```

Параметр `&theme=light` улучшает читаемость на мобильных экранах.

---

## 🔐 Безопасность

### Ограничение доступа

Для публичных демонстраций рекомендуется:

1. **Viewer Role:** Создать пользователя только для просмотра
   ```
   Email: demo@robbox.local
   Role: Viewer
   Password: demo123
   ```

2. **Anonymous Access:** Включить анонимный доступ (только просмотр)
   ```bash
   # В docker-compose.yaml для Grafana
   environment:
     - GF_AUTH_ANONYMOUS_ENABLED=true
     - GF_AUTH_ANONYMOUS_ORG_ROLE=Viewer
   ```

3. **Read-Only API Key:** Для программного доступа
   - Grafana → Configuration → API Keys
   - Add API Key (Role: Viewer)

---

**Автор:** Rob Box Team  
**Версия:** 1.0  
**Последнее обновление:** 2025-11-01
