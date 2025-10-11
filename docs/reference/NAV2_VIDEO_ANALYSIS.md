# Анализ стенограммы Articulated Robotics: Nav2 Tutorial

> **Источник**: Articulated Robotics - ROS2 Nav2 Tutorial  
> **Дата анализа**: 2025-10-11  
> **Цель**: Выявить best practices и улучшения для rob_box Nav2 integration

## 📋 Оглавление

- [Ключевые концепции](#ключевые-концепции)
- [Архитектурные решения](#архитектурные-решения)
- [Сравнение с rob_box](#сравнение-с-rob_box)
- [Рекомендации](#рекомендации)
- [Improvements для rob_box](#improvements-для-rob_box)

## 🎯 Ключевые концепции

### 1. Различие между SLAM map и Cost map

**Из видео**:
> "The cost map is different to the SLAM map. It might have been based on the SLAM map initially, but its values mean different things and it's getting updated independently."

**Суть**:
- **SLAM map**: Статическая карта окружения для локализации
  - Значения: occupied/free/unknown
  - Обновляется только во время SLAM
  - Используется для глобального планирования

- **Cost map**: Динамическая карта для навигации
  - Значения: cost (0-255, где 255 = препятствие)
  - Обновляется постоянно из LiDAR
  - Имеет inflation zones (зоны расширения препятствий)
  - Используется для path planning и collision avoidance

**Практическое значение**:
```
SLAM map (static) → Global Costmap (base layer)
                    + Obstacle Layer (from /scan)
                    + Inflation Layer
                    = Final Costmap for navigation
```

### 2. Два режима работы Nav2

#### Режим 1: SLAM + Navigation (Mapping Mode)
```bash
# Одновременно создается карта И навигация
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

**Плюсы**:
- Исследование неизвестного окружения
- Карта обновляется по мере движения

**Минусы**:
- Больше CPU/RAM
- Менее стабильная локализация

#### Режим 2: Localization + Navigation (Navigation Mode)
```bash
# Используется готовая карта
ros2 launch nav2_bringup localization_launch.py \
  map:=my_map.yaml \
  use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  map_subscribe_transient_local:=true
```

**Плюсы**:
- Меньше нагрузка на систему
- Более стабильная локализация (AMCL)
- Быстрый старт

**Минусы**:
- Нужна готовая карта
- Не адаптируется к изменениям окружения (только динамические препятствия)

### 3. Map Subscribe Transient Local

**Из видео**:
> "This is a flag that's going to tell Nav2 that it needs to subscribe to the map with transient local as well."

**Суть**: QoS parameter для `/map` topic
- **Volatile** (default): Пропускает старые сообщения, получает только новые
- **Transient Local**: Сохраняет последнее сообщение, новые subscribers получают его сразу

**Когда нужен**:
- При использовании `map_server` (публикует карту 1 раз)
- При использовании AMCL для локализации
- Nav2 должен получить карту даже если запустился позже

**Пример**:
```yaml
# nav2_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # ...
      static_layer:
        map_subscribe_transient_local: True  # ← ВАЖНО!
```

## 🏗️ Архитектурные решения

### 1. twist_mux для приоритизации команд

**Проблема**: Nav2 публикует на `/cmd_vel`, но у нас:
- Джойстик тоже публикует на `/cmd_vel`
- diff_drive_controller слушает на `/diff_cont/cmd_vel_unstamped`

**Решение из видео**: twist_mux

```yaml
# config/twist_mux.yaml
topics:
  - name: navigation
    topic: cmd_vel
    timeout: 0.5
    priority: 10  # Низкий приоритет
    
  - name: joystick
    topic: cmd_vel_joy
    timeout: 0.5
    priority: 100  # Высокий приоритет (джойстик важнее!)
```

**Launch**:
```python
Node(
    package='twist_mux',
    executable='twist_mux',
    parameters=[twist_mux_params],
    remappings=[('cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
)
```

**Логика работы**:
1. Джойстик активен → twist_mux пропускает только джойстик
2. Джойстик неактивен (timeout) → twist_mux пропускает Nav2
3. Оба неактивны → twist_mux публикует zero twist

**Преимущества**:
- ✅ Плавное переключение между ручным и авто режимом
- ✅ Приоритизация (безопасность важнее автономии)
- ✅ Lock/unlock механизмы
- ✅ Timeouts для безопасности

### 2. Альтернативы twist_mux

#### Вариант A: Прямое remapping
```python
# В navigation_launch.py
remappings=[('cmd_vel', '/diff_cont/cmd_vel_unstamped')]
```

**Проблема**: Джойстик и Nav2 оба публикуют → race condition

#### Вариант B: twist_stamper как multiplexer
```python
# Если используется twist_stamper
Node(
    package='twist_stamper',
    remappings=[
        ('cmd_vel_in', 'cmd_vel'),  # Nav2 + Joystick → здесь
        ('cmd_vel_out', '/diff_cont/cmd_vel')
    ]
)
```

**Проблема**: Нет приоритизации, нет timeouts

#### Вариант C: topic_tools relay
```bash
ros2 run topic_tools relay /cmd_vel /diff_cont/cmd_vel_unstamped
```

**Проблема**: Еще одна нода, нет логики

**Вывод**: twist_mux — самый правильный подход!

### 3. Параметры тюнинга из видео

**Robot geometry**:
```yaml
robot_radius: 0.17  # м (для rob_box это правильно!)
```

**Inflation parameters**:
```yaml
inflation_radius: 0.55  # robot_radius + safety_margin
cost_scaling_factor: 3.0
```

**DWB Controller samples** (для оптимизации):
```yaml
# Больше samples = точнее, но медленнее
vx_samples: 20  # можно уменьшить до 10-15 на слабом железе
vtheta_samples: 40  # можно уменьшить до 20-30
sim_time: 1.7  # время симуляции траектории
```

**Update frequencies** (для Raspberry Pi):
```yaml
# Local costmap
update_frequency: 5.0  # можно уменьшить до 3.0
publish_frequency: 2.0  # можно уменьшить до 1.0

# Global costmap
update_frequency: 1.0  # можно уменьшить до 0.5
publish_frequency: 1.0
```

## 📊 Сравнение с rob_box

| Аспект | Видео (Articulated Robotics) | rob_box (текущая реализация) | Статус |
|--------|------------------------------|-------------------------------|--------|
| **twist_mux** | ✅ Используется | ❌ Нет (прямое remapping) | 🔴 Нужно добавить |
| **Localization mode** | ✅ Отдельный launch | ❌ Только SLAM mode | 🔴 Нужно добавить |
| **map_subscribe_transient_local** | ✅ Настроен | ⚠️ Нужно проверить | 🟡 Проверить |
| **Waypoint navigation** | ✅ Показан в RViz | ❌ Только goal_pose | 🟡 Документировать |
| **Custom params** | ✅ Копирует в свой пакет | ✅ У нас есть nav2_params.yaml | ✅ OK |
| **Robot radius** | 0.22 (TurtleBot) | 0.17 (rob_box) | ✅ Правильно |
| **Inflation radius** | 0.55 | 0.55 | ✅ OK |
| **Controller frequency** | 20 Hz | 20 Hz | ✅ OK |
| **DWB samples** | vx:20, vtheta:40 | vx:20, vtheta:40 | ✅ OK |
| **Costmap resolution** | 0.05m | 0.05m | ✅ OK |

## 🎓 Рекомендации

### 1. КРИТИЧНО: Добавить twist_mux

**Почему**:
- Безопасность (джойстик должен иметь приоритет над Nav2)
- Smooth transition между режимами
- Prevents race conditions на `/cmd_vel`

**Как реализовать**:
```yaml
# docker/main/config/twist_mux/twist_mux.yaml
topics:
  - name: navigation
    topic: cmd_vel
    timeout: 0.5
    priority: 10
    
  - name: joystick
    topic: cmd_vel_joy
    timeout: 0.5
    priority: 100
    
  - name: emergency_stop
    topic: cmd_vel_emergency
    timeout: 0.1
    priority: 255  # Highest!
```

### 2. ВАЖНО: Режим локализации

**Создать** `localization.launch.py`:
```python
# Использует готовую карту вместо SLAM
# AMCL для локализации
# map_server для публикации карты
# Меньше нагрузка на CPU
```

**Use case**: Робот знает карту, просто навигирует

### 3. СРЕДНЕ: Waypoint navigation examples

**Добавить в документацию**:
- Как использовать Nav2 waypoint follower
- Python пример для waypoint mission
- RViz panel для waypoint mode

### 4. НИЗКО: Tuning guide

**Расширить NAV2_SETUP.md**:
- Секция про cost map vs SLAM map
- Таблица параметров для разных сценариев (точность vs скорость)
- Troubleshooting для AMCL drift issues

## 🔧 Improvements для rob_box

### Phase 1: Критичные (безопасность)

1. **twist_mux integration**
   - Файлы: `docker/main/config/twist_mux/twist_mux.yaml`
   - Изменения: `docker/main/docker-compose.yaml` (новый сервис)
   - Тестирование: Джойстик должен override Nav2

2. **Emergency stop topic**
   - Topic: `/cmd_vel_emergency`
   - Priority: 255 (максимальный)
   - Use case: Web UI или кнопка аварийной остановки

### Phase 2: Функциональность

3. **Localization mode launch**
   - `src/rob_box_bringup/launch/localization.launch.py`
   - Запускает: map_server + AMCL (без SLAM)
   - Use case: Повторная навигация по известной карте

4. **Waypoint navigation example**
   - `scripts/nav2/waypoint_navigation_example.py`
   - Использует Nav2 Action API
   - Демонстрирует patrol behavior

### Phase 3: Документация

5. **Обновить NAV2_SETUP.md**
   - Секция "Cost map vs SLAM map"
   - Секция "Localization vs SLAM mode"
   - Секция "twist_mux alternatives"
   - Troubleshooting для AMCL

6. **Создать TUNING_GUIDE.md**
   - Параметры для точности
   - Параметры для скорости
   - Параметры для экономии CPU
   - Real-world examples

### Phase 4: Оптимизация

7. **Performance tuning для Pi 4**
   - Уменьшить update frequencies при высокой нагрузке
   - Reduce DWB samples (vx: 20→15, vtheta: 40→25)
   - Dynamic reconfigure на основе load

## 📝 Примеры кода из видео

### twist_mux config
```yaml
topics:
  - name: navigation
    topic: cmd_vel
    timeout: 0.5
    priority: 10
    
  - name: joystick
    topic: cmd_vel_joy
    timeout: 0.5
    priority: 100
```

### Localization launch (упрощенно)
```bash
# Запуск AMCL + map_server
ros2 launch nav2_bringup localization_launch.py \
  map:=/maps/my_map.yaml \
  use_sim_time:=false

# Запуск Nav2 с transient_local
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map_subscribe_transient_local:=true
```

### Waypoint navigation (Python API)
```python
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

waypoints = [
    create_pose(1.0, 0.0, 0.0),
    create_pose(2.0, 1.0, 90.0),
    create_pose(1.0, 2.0, 180.0),
]

goal = NavigateThroughPoses.Goal()
goal.poses = waypoints

client.send_goal_async(goal)
```

## 🎯 Выводы

### Что мы делаем правильно:
✅ Параметры навигации (robot_radius, inflation, DWB)  
✅ Costmap конфигурация (local/global)  
✅ Behavior Trees для recovery  
✅ Comprehensive documentation  

### Что нужно улучшить:
🔴 **КРИТИЧНО**: twist_mux для безопасности  
🟡 **ВАЖНО**: Localization mode (AMCL без SLAM)  
🟡 **ПОЛЕЗНО**: Waypoint navigation examples  
🟢 **ОПЦИОНАЛЬНО**: Advanced tuning guide  

### Next steps:
1. Реализовать twist_mux (самое важное!)
2. Создать localization.launch.py
3. Добавить waypoint navigation example
4. Обновить документацию с insights из видео
5. Протестировать на реальном железе

---

**Источник анализа**: Articulated Robotics - "ROS2 Nav2 Tutorial"  
**Благодарность**: Josh Newans (@ArticulatedRobotics) за отличные туториалы!
