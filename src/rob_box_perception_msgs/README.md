# rob_box_perception_msgs

Кастомные ROS 2 сообщения для пакета `rob_box_perception`.

## Описание

Пакет определяет единственное сообщение `PerceptionEvent` — агрегированный контекст состояния робота и его окружения, публикуемый нодой `context_aggregator_node` и потребляемый `dialogue_node` (голосовой ассистент).

## Сообщения

### `PerceptionEvent`

Агрегированный контекст от всех источников данных системы.

| Поле | Тип | Описание |
|------|-----|---------|
| `stamp` | `builtin_interfaces/Time` | Временная метка |
| `vision_context` | `string` | JSON-контекст от Vision Pipeline |
| `pose` | `geometry_msgs/Pose` | Текущая позиция робота |
| `velocity` | `geometry_msgs/Twist` | Текущая скорость |
| `is_moving` | `bool` | Флаг движения |
| `battery_voltage` | `float32` | Напряжение батареи (В) |
| `temperature` | `float32` | Температура (°C) |
| `apriltag_ids` | `int32[]` | Идентификаторы обнаруженных AprilTag меток |
| `system_health_status` | `string` | Состояние системы: `"healthy"`, `"degraded"`, `"critical"` |
| `health_issues` | `string[]` | Список активных проблем |
| `current_time_human` | `string` | Время в читаемом формате: `"2025-10-21 12:30:00"` |
| `time_period` | `string` | Период суток: `"morning"`, `"day"`, `"evening"`, `"night"` |
| `time_context_json` | `string` | Полный контекст времени (JSON) |
| `internet_available` | `bool` | Доступность интернета |
| `active_nodes` | `string[]` | Список активных ROS 2 нод |
| `failed_nodes` | `string[]` | Список упавших нод |
| `missing_nodes` | `string[]` | Список ожидаемых, но отсутствующих нод |
| `equipment_summary_json` | `string` | Состояние оборудования (JSON) |
| `mapping_mode` | `string` | Режим карты: `"mapping"`, `"localization"`, `"unknown"` |
| `memory_summary` | `string` | Суммаризация из долгосрочной памяти |
| `speech_summaries` | `string` | Суммаризованные вопросы пользователя (JSON array) |
| `robot_response_summaries` | `string` | Суммаризованные ответы робота (JSON array) |
| `robot_thought_summaries` | `string` | Суммаризованные мысли робота (JSON array) |
| `vision_summaries` | `string` | Суммаризованные визуальные наблюдения (JSON array) |
| `system_summaries` | `string` | Суммаризованные системные события (JSON array) |

## Топик

Сообщение публикуется на топик `/perception/context` (пакет `rob_box_perception`).

```bash
# Просмотр последнего события
ros2 topic echo /perception/context --once
```

## Зависимости

- `builtin_interfaces` — временные метки
- `geometry_msgs` — Pose, Twist
- `rosidl_default_generators` — генерация кода сообщений
