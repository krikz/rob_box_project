# rob_box_perception

**Internal Dialogue Agent** - Система внутреннего диалога робота.

## Описание

Робот поддерживает непрерывный внутренний диалог на основе:
- 👁️ Камеры (1 fps или по событиям)
- 📡 Сенсоров (батарея, температура, IMU)
- 🗺️ Текущего состояния задачи (навигация, локализация)
- 🎤 Недавней речи + направление (азимут)

AI Agent (DeepSeek) будет:
- Анализировать визуальный контекст
- Интерпретировать контекст (включая динамику)
- Генерировать внутренние мысли (рефлексия, гипотезы, планы)
- Решать: говорить или молчать

## Архитектура

```
┌─────────────────────────────────────────────────────────────────┐
│                        Vision Pi                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐         ┌──────────────────────────────────┐  │
│  │ OAK-D Camera │────────▶│ vision_stub_node                 │  │
│  │ /oak/rgb/... │         │ (TODO: AI HAT + YOLO в будущем)  │  │
│  └──────────────┘         └─────────────┬────────────────────┘  │
│                                          │                        │
│  ┌──────────────┐                        │                        │
│  │ AprilTag     │────────────────────────┤                        │
│  │ /apriltag/.. │                        │                        │
│  └──────────────┘                        │                        │
│                                          ▼                        │
│                             /perception/vision_context            │
│                                          │                        │
└──────────────────────────────────────────┼────────────────────────┘
                                           │
┌──────────────────────────────────────────┼────────────────────────┐
│                        Main Pi           │                        │
├──────────────────────────────────────────┼────────────────────────┤
│                                          │                        │
│  ┌──────────────┐                        │                        │
│  │ RTAB-Map     │──────┐                 │                        │
│  │ /map, /pose  │      │                 │                        │
│  └──────────────┘      │                 │                        │
│                        │                 │                        │
│  ┌──────────────┐      │                 │                        │
│  │ VESC Nexus   │──────┤                 │                        │
│  │ /odom        │      │                 │                        │
│  └──────────────┘      │                 │                        │
│                        │                 │                        │
│  ┌──────────────┐      │                 │                        │
│  │ ESP32 Hub    │──────┤                 │                        │
│  │ /device/...  │      │                 │                        │
│  └──────────────┘      │                 │                        │
│                        ▼                 ▼                        │
│                 ┌────────────────────────────────┐                │
│                 │   reflection_node              │                │
│                 │   ┌────────────────────────┐   │                │
│                 │   │ 1. Собрать контекст    │   │                │
│                 │   │ 2. Проверить диалог    │   │                │
│                 │   │ 3. Размышление         │   │                │
│                 │   │ 4. Решение говорить    │   │                │
│                 │   └────────────────────────┘   │                │
│                 └────────────┬───────────────────┘                │
│                              │                                    │
│                              ├──▶ /reflection/internal_thought    │
│                              │                                    │
│                              └──▶ /voice/tts/request              │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

## Ноды

### vision_stub_node
**Статус:** ⚠️ ЗАГЛУШКА (TODO: AI HAT + YOLO)

Временная нода для публикации фейкового vision context.

**Подписывается:**
- `/oak/rgb/image_raw/compressed` (sensor_msgs/CompressedImage)

**Публикует:**
- `/perception/vision_context` (std_msgs/String) - JSON с семантическим контекстом

**В будущем:**
- Обработка на AI HAT 26 TOPS
- YOLO v8/v11 детекция объектов
- Поддержка нескольких камер (front stereo, up camera)

### reflection_node
**Статус:** ✅ Работает (с DeepSeek API)

Ядро внутреннего диалога робота.

**Подписывается:**
- `/perception/vision_context` (std_msgs/String) - семантический контекст
- `/rosout` (rcl_interfaces/Log) - **ВСЕ логи системы** для мониторинга
- `/voice/stt/result`, `/voice/dialogue/response` - диалог с пользователем
- `/rtabmap/localization_pose`, `/odom` - навигация
- `/device/snapshot` - сенсоры ESP32

**Публикует:**
- `/reflection/internal_thought` (String) - внутренние мысли
- `/voice/tts/request` (String) - команда на речь

**Мониторинг здоровья системы:**
- Собирает ERROR/WARN со всех нод через `/rosout`
- Проверяет активность важных топиков
- Сообщает пользователю о критичных проблемах
- Health status: `healthy` | `degraded` | `critical`

**System Prompt:**
- Загружается из файла: `prompts/reflection_prompt.txt`
- Можно кастомизировать через параметр `system_prompt_file`
- Fallback на встроенный промпт если файл не найден

**Параметры:**
- `reflection_rate` (1.0 Hz) - частота размышлений
- `dialogue_timeout` (10.0s) - пауза во время диалога
- `memory_window` (60s) - окно краткосрочной памяти
- `enable_speech` (true) - включить речь
- `system_prompt_file` ('reflection_prompt.txt') - файл с промптом

### health_monitor
**Статус:** ✅ Утилита для диагностики

Простой монитор для проверки здоровья системы.

**Использование:**
```bash
ros2 run rob_box_perception health_monitor
```

**Показывает:**
- Количество ошибок и предупреждений
- Последние ERROR/WARN от всех нод
- Статус системы (HEALTHY/DEGRADED/CRITICAL)
- Обновляется каждые 5 секунд
- `/apriltag/detections` (apriltag_msgs/AprilTagDetectionArray) - AprilTag маркеры
- `/voice/stt/result` (std_msgs/String) - что сказал пользователь
- `/voice/dialogue/response` (std_msgs/String) - что ответил робот
- `/rtabmap/localization_pose` (geometry_msgs/PoseStamped) - позиция на карте
- `/odom` (nav_msgs/Odometry) - одометрия
- `/device/snapshot` (robot_sensor_hub_msg/DeviceSnapshot) - сенсоры ESP32
- `/sensors/motor_state/*` (vesc_msgs/VescStateStamped) - состояние моторов

**Публикует:**
- `/reflection/internal_thought` (std_msgs/String) - внутренние мысли (для логов)
- `/voice/tts/request` (std_msgs/String) - когда решает сказать что-то вслух

## Запуск

```bash
# 1. Собрать пакет
cd ~/rob_box_project
colcon build --packages-select rob_box_perception --symlink-install

# 2. Source workspace
source install/setup.bash

# 3. Экспортировать API ключ (если есть)
export DEEPSEEK_API_KEY="your_key_here"

# 4. Запустить Internal Dialogue Agent
ros2 launch rob_box_perception internal_dialogue.launch.py
```

## Параметры

### vision_stub_node
- `publish_rate` (float, default: 1.0) - Частота публикации контекста (Hz)

### reflection_node
- `reflection_rate` (float, default: 1.0) - Частота размышлений (Hz)
- `dialogue_timeout` (float, default: 10.0) - Тайм-аут диалога (секунды)
- `memory_window` (int, default: 60) - Окно короткой памяти (секунды)
- `enable_speech` (bool, default: true) - Включить речь робота

## Топики

### Входящие (подписки)
| Топик | Тип | Источник | Описание |
|-------|-----|----------|----------|
| `/perception/vision_context` | String | vision_stub_node | Семантический контекст с камер |
| `/apriltag/detections` | AprilTagDetectionArray | apriltag_node | AprilTag маркеры |
| `/voice/stt/result` | String | stt_node | Распознанная речь |
| `/voice/dialogue/response` | String | dialogue_node | Ответы робота |
| `/rtabmap/localization_pose` | PoseStamped | rtabmap | Позиция на карте |
| `/odom` | Odometry | ros2_control | Одометрия |
| `/device/snapshot` | DeviceSnapshot | micro_ros_agent | Сенсоры ESP32 |
| `/rosout` | Log | ROS2 Core | **Логи всех нод (ERROR/WARN)** |

### Исходящие (публикации)
| Топик | Тип | Описание |
|-------|-----|----------|
| `/perception/vision_context` | String | Семантический контекст (от vision_stub) |
| `/reflection/internal_thought` | String | Внутренние мысли робота |
| `/voice/tts/request` | String | Команда на синтез речи |

## Мониторинг системы

### Health Check
**reflection_node** автоматически мониторит здоровье системы:

**Что проверяется:**
- ❌ **Ошибки**: Все ERROR/FATAL от нод через `/rosout`
- ⚠️ **Предупреждения**: WARN сообщения
- 📡 **Топики**: Активность важных топиков (камера, одометрия)
- 🕐 **Частота**: 5+ ошибок за минуту = degraded

**Статусы:**
- ✅ `healthy` - всё ОК
- ⚠️ `degraded` - есть проблемы (много ошибок, отсутствие данных)
- 🚨 `critical` - критичные ошибки (FATAL, crash, fail)

**Интеграция с диалогом:**
- Информация о здоровье попадает в контекст для DeepSeek
- При critical - робот **обязательно** сообщит пользователю
- При degraded - может предложить помощь

**Примеры:**
```
🚨 CRITICAL: "У меня проблемы с камерой, не могу видеть окружение"
⚠️  DEGRADED: "Много ошибок в навигации, возможно карта устарела"
```

### Утилита health_monitor

Для ручной диагностики используй:
```bash
ros2 run rob_box_perception health_monitor
```

**Показывает каждые 5 секунд:**
```
======================================================================
🏥 HEALTH REPORT
======================================================================
Status: ✅ HEALTHY
Total Errors: 0 (последние 0 за минуту)
Total Warnings: 2

--- Recent Warnings ---
  [WARN] oak_d_node (15s ago): Frame dropped due to high CPU usage
  [WARN] rtabmap (30s ago): No loop closure candidates found
======================================================================
```

## TODO

- [ ] **vision_stub_node**: Заменить на реальную обработку
  - [ ] Интеграция AI HAT 26 TOPS
  - [ ] YOLO v8/v11 детекция
  - [ ] Поддержка нескольких камер
  - [ ] Bounding boxes объектов
  
- [ ] **reflection_node**: DeepSeek API интеграция
  - [ ] Реальный вызов DeepSeek Vision API
  - [ ] Промпт инжиниринг для размышлений
  - [ ] Правила когда говорить/молчать
  
- [ ] **Память**:
  - [ ] Долговременная память (важные события)
  - [ ] Распознавание известных людей
  - [ ] База знаний о окружении

- [ ] **Интеграция**:
  - [ ] Пауза размышлений во время навигации
  - [ ] Контекст текущей задачи
  - [ ] LED анимации для "размышления"

## Тестирование

```bash
# Проверить топики
ros2 topic list | grep -E "perception|reflection"

# Посмотреть vision context
ros2 topic echo /perception/vision_context

# Посмотреть внутренние мысли
ros2 topic echo /reflection/internal_thought

# Проверить частоту
ros2 topic hz /perception/vision_context
ros2 topic hz /reflection/internal_thought
```

## Лицензия

MIT
