# Руководство по настройке Nav2 Navigation Stack

> **Дата создания**: 2025-10-11  
> **Статус**: ✅ Production Ready  
> **Платформа**: Main Pi (Raspberry Pi 5)

## 📋 Оглавление

- [Обзор](#обзор)
- [Архитектура](#архитектура)
- [Компоненты Nav2](#компоненты-nav2)
- [Конфигурация](#конфигурация)
- [Запуск](#запуск)
- [Использование](#использование)
- [Troubleshooting](#troubleshooting)
- [Оптимизация](#оптимизация)

## 🎯 Обзор

**Nav2 (Navigation2)** — это навигационный фреймворк для ROS2, обеспечивающий:
- 🗺️ **Глобальное планирование пути** (A*, Dijkstra, SMAC)
- 🎮 **Локальное следование траектории** (DWB, TEB, Regulated Pure Pursuit)
- 🚧 **Избежание препятствий** (Local/Global Costmaps)
- 🔄 **Recovery behaviors** (Spin, BackUp, Clear Costmaps)
- 🧭 **Waypoint navigation** (следование через точки)
- 🌳 **Behavior Trees** (гибкая логика навигации)

### Зачем нужен Nav2?

- **RTAB-Map** создает карту окружения (SLAM)
- **Nav2** использует эту карту для автономной навигации
- **ros2_control** выполняет команды скорости от Nav2

```
┌─────────┐      /map       ┌────────┐     /goal_pose    ┌──────┐
│ RTAB-Map│ ──────────────> │  Nav2  │ <──────────────── │ User │
└─────────┘                 └────────┘                    └──────┘
     ▲                           │
     │ /scan                     │ /cmd_vel
     │                           ▼
┌─────────┐                 ┌──────────────┐
│ LiDAR   │                 │ ros2_control │
└─────────┘                 └──────────────┘
                                    │
                                    ▼
                            ┌───────────────┐
                            │ VESC Motors   │
                            └───────────────┘
```

## 🏗️ Архитектура

### Компоненты системы

```
Nav2 Stack
├── Controller Server (path following)
│   ├── DWB Controller (Dynamic Window Approach)
│   ├── Progress Checker
│   └── Goal Checker
├── Planner Server (path planning)
│   └── NavFn Planner (A* на сетке)
├── Behavior Server (recovery behaviors)
│   ├── Spin
│   ├── BackUp
│   ├── DriveOnHeading
│   └── Wait
├── BT Navigator (behavior tree execution)
│   ├── navigate_to_pose_w_replanning.xml
│   └── navigate_through_poses.xml
├── Waypoint Follower
├── Velocity Smoother
└── Lifecycle Manager
```

### Зависимости

Nav2 требует:
1. **robot_state_publisher** → TF дерево робота
2. **rtabmap** → карта `/map` и локализация
3. **lslidar** (через Vision Pi) → сканы `/scan`
4. **ros2_control** → одометрия `/odom` и исполнение `/cmd_vel`

## 🔧 Компоненты Nav2

### 1. Controller Server (Следование по траектории)

**Задача**: Преобразовать глобальный путь в команды скорости `/cmd_vel`

**Алгоритм**: DWB (Dynamic Window Approach)
- Семплирует возможные скорости (vx, vy, vtheta)
- Симулирует траектории на `sim_time` вперед
- Оценивает траектории по критериям (obstacles, path alignment, goal distance)
- Выбирает лучшую траекторию

**Параметры rob_box**:
```yaml
max_vel_x: 0.5 м/с       # Максимальная скорость вперед
max_vel_theta: 1.2 рад/с # Максимальная угловая скорость (~68°/с)
min_vel_x: -0.3 м/с      # Максимальная скорость назад
acc_lim_x: 0.5 м/с²      # Ускорение
robot_radius: 0.17 м     # Радиус робота (диаметр 340мм)
```

### 2. Planner Server (Планирование пути)

**Задача**: Построить путь от текущей позиции до цели

**Алгоритм**: NavFn (Dijkstra/A*)
- A* с эвристикой для быстрого поиска
- Работает на глобальной costmap
- Избегает препятствия и неизвестные области

**Параметры**:
```yaml
tolerance: 0.5 м         # Допустимое отклонение от цели
use_astar: true          # A* вместо Dijkstra (быстрее)
allow_unknown: true      # Разрешить проходить через неизведанное
```

### 3. Costmaps (Карты стоимости)

#### Local Costmap (Локальная карта)

**Размер**: 3×3 метра вокруг робота  
**Фрейм**: `odom` (катится вместе с роботом)  
**Обновление**: 5 Hz

**Слои**:
- **Voxel Layer**: 3D препятствия из `/scan`
  - `max_obstacle_height: 2.0 м`
  - `obstacle_max_range: 2.5 м` (видимость препятствий)
  - `raytrace_max_range: 3.0 м` (очистка пространства)

- **Inflation Layer**: Расширение препятствий
  - `inflation_radius: 0.55 м` (robot_radius 0.17 + safety margin 0.38)
  - `cost_scaling_factor: 3.0`

#### Global Costmap (Глобальная карта)

**Размер**: 50×50 метров (rolling window)  
**Фрейм**: `map`  
**Обновление**: 1 Hz

**Режим**: Rolling window (вместо static map) для независимости от RTAB-Map  
**Обоснование**: Гарантирует работу Nav2 с самого старта, без ожидания карты от RTAB-Map

**Слои**:
- **Obstacle Layer**: Динамические препятствия из `/scan`
  - `obstacle_max_range: 2.5 м`
  - `raytrace_max_range: 3.0 м`
- **Inflation Layer**: Расширение препятствий
  - `inflation_radius: 0.55 м`

### 4. Behavior Server (Recovery Behaviors)

**Задача**: Восстановление при застревании

**Поведения**:
- **Spin**: Вращение на месте (поиск пути)
- **BackUp**: Движение назад (выход из тупика)
- **DriveOnHeading**: Движение по заданному направлению
- **Wait**: Ожидание (препятствие может уйти)

**Последовательность recovery** (из BT):
1. ClearCostmaps (локальная + глобальная)
2. Spin 1.57 рад (90°)
3. Wait 5 секунд
4. BackUp 0.30 метра

### 5. BT Navigator (Behavior Tree)

**Задача**: Управление логикой навигации

**Структура BT** (`navigate_to_pose_w_replanning.xml`):
```
RecoveryNode (6 retries)
├── PipelineSequence
│   ├── RateController (1 Hz replanning)
│   │   └── ComputePathToPose
│   └── FollowPath
└── ReactiveFallback (если застряли)
    ├── GoalUpdated (новая цель → прервать recovery)
    └── RoundRobin (перебор recovery actions)
        ├── ClearCostmaps
        ├── Spin
        ├── Wait
        └── BackUp
```

## ⚙️ Конфигурация

### Файлы конфигурации

```
docker/main/config/nav2/
├── nav2_params.yaml                # Основная конфигурация
├── local_costmap_params.yaml      # Локальная costmap
├── global_costmap_params.yaml     # Глобальная costmap
└── behavior_trees/
    ├── navigate_to_pose_w_replanning.xml
    └── navigate_through_poses.xml
```

### Настройка для rob_box

#### Геометрия робота
```yaml
robot_base_frame: base_link
robot_radius: 0.17  # диаметр 340мм → радиус 170мм
```

#### Скорости (ограничения VESC)
```yaml
max_vel_x: 0.5      # м/с (ограничено motors)
max_vel_theta: 1.2  # рад/с (~68°/с)
min_vel_x: -0.3     # м/с (обратное движение)
```

#### Точность навигации
```yaml
xy_goal_tolerance: 0.15   # 15 см точность позиции
yaw_goal_tolerance: 0.15  # ~8.6° точность угла
```

#### Безопасность
```yaml
inflation_radius: 0.55    # robot_radius (0.17) + margin (0.38)
cost_scaling_factor: 3.0  # Насколько быстро растет стоимость
```

## 🚀 Запуск

### Через Docker Compose

```bash
cd docker/main
docker-compose up -d nav2
```

**Проверка**:
```bash
# Логи
docker logs nav2 -f

# Список нод
ros2 node list | grep -E "controller_server|planner_server|bt_navigator"

# Топики
ros2 topic list | grep -E "/goal_pose|/cmd_vel|/local_costmap|/global_costmap"
```

### Вручную (для отладки)

```bash
docker run --rm -it \
  --network host \
  --name nav2 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -v $(pwd)/config:/config/shared:ro \
  -v $(pwd)/config/nav2:/config/nav2:ro \
  -v $(pwd)/maps:/maps:ro \
  ghcr.io/krikz/rob_box:nav2-kilted-latest \
  /scripts/start_nav2.sh
```

### Последовательность запуска

1. **zenoh-router** (центральный роутер)
2. **robot-state-publisher** (TF дерево)
3. **lslidar** (Vision Pi) → `/scan`
4. **rtabmap** → `/map` и локализация
5. **ros2-control** → `/odom` и выполнение `/cmd_vel`
6. **nav2** → автономная навигация

## 🎮 Использование

### Отправка цели (Goal Pose)

#### Через ROS2 CLI

```bash
# Простая цель (x=2.0, y=1.0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, 
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, 
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

# Цель с ориентацией (90° поворот)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, 
    pose: {position: {x: 3.0, y: 2.0, z: 0.0}, 
           orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}'
```

#### Через Nav2 Action

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

def send_goal():
    rclpy.init()
    node = rclpy.create_node('nav2_goal_sender')
    
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    client.wait_for_server()
    
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = 2.0
    goal.pose.pose.position.y = 1.0
    goal.pose.pose.orientation.w = 1.0
    
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    
    print(f"Navigation result: {result_future.result()}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    send_goal()
```

### Следование через waypoints

```bash
# Создать список точек
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}},
    {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}},
    {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}
  ]}"
```

### Мониторинг

#### Команды скорости
```bash
ros2 topic echo /cmd_vel
```

#### Статус навигации
```bash
ros2 topic echo /navigate_to_pose/_action/feedback
```

#### Costmaps (визуализация)
```bash
# Локальная costmap
ros2 topic echo /local_costmap/costmap

# Глобальная costmap
ros2 topic echo /global_costmap/costmap
```

#### Глобальный путь
```bash
ros2 topic echo /plan
```

#### Локальная траектория
```bash
ros2 topic echo /local_plan
```

### Отмена навигации

```bash
# Через action
ros2 action send_goal --cancel /navigate_to_pose

# Или остановка робота
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0}, angular: {z: 0}}"
```

## 🔍 Troubleshooting

### Проблема: Nav2 не запускается

**Симптомы**:
```
[ERROR] Failed to create controller_server
```

**Решение**:
1. Проверить зависимости:
   ```bash
   ros2 topic list | grep -E "/robot_description|/odom|/scan|/map"
   ```
2. Проверить конфигурацию:
   ```bash
   docker exec nav2 cat /config/nav2/nav2_params.yaml
   ```
3. Логи lifecycle_manager:
   ```bash
   docker logs nav2 | grep lifecycle
   ```

### Проблема: Нет глобального пути

**Симптомы**:
```
[WARN] [planner_server]: No path found
```

**Причины**:
- Нет карты `/map` (RTAB-Map не запущен)
- Цель в препятствии или за стеной
- Global costmap не обновляется

**Решение**:
```bash
# Проверить карту
ros2 topic echo /map --once

# Проверить global costmap
ros2 topic echo /global_costmap/costmap --once

# Очистить costmap
ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty
```

### Проблема: Робот застревает

**Симптомы**:
- Робот не двигается
- Постоянные recovery behaviors (Spin, BackUp)

**Причины**:
- Local costmap видит препятствие
- Inflation radius слишком большой
- DWB не находит допустимую траекторию

**Решение**:
```bash
# Проверить local costmap
ros2 topic echo /local_costmap/costmap --once

# Уменьшить inflation_radius в конфигурации
# docker/main/config/nav2/local_costmap_params.yaml
inflation_radius: 0.4  # было 0.55

# Очистить local costmap
ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty

# Перезапустить nav2
docker restart nav2
```

### Проблема: Робот не следует пути точно

**Симптомы**:
- Большое отклонение от пути
- Робот "срезает углы"

**Причины**:
- xy_goal_tolerance слишком большой
- PathAlign.scale слишком маленький

**Решение**: Увеличить weights в DWB
```yaml
# docker/main/config/nav2/nav2_params.yaml
PathAlign.scale: 48.0      # было 32.0 → больше следование пути
PathDist.scale: 48.0       # было 32.0
GoalDist.scale: 24.0       # оставить как было
```

### Проблема: Робот слишком медленный

**Симптомы**:
- Скорость намного ниже max_vel_x

**Причины**:
- sim_time слишком большой (слишком "осторожное" планирование)
- Слишком много семплов (vx_samples, vtheta_samples)

**Решение**:
```yaml
# docker/main/config/nav2/nav2_params.yaml
sim_time: 1.2              # было 1.7 → короче предсказание
vx_samples: 15             # было 20 → меньше семплов
vtheta_samples: 30         # было 40
```

### Проблема: "Transform timeout"

**Симптомы**:
```
[ERROR] [bt_navigator]: Transform timeout for frame 'base_link'
```

**Причины**:
- robot_state_publisher не публикует TF
- TF между `map` ↔ `odom` ↔ `base_link` не полный

**Решение**:
```bash
# Проверить TF дерево
ros2 run tf2_tools view_frames

# Проверить конкретный transform
ros2 run tf2_ros tf2_echo map base_link

# Увеличить wait_for_transform
# docker/main/config/nav2/nav2_params.yaml
wait_for_transform: 1.0    # было 0.5
```

## 📊 Оптимизация

### Производительность на Raspberry Pi 5

**Базовое использование**:
- CPU: ~15-20% (controller_server + planner_server)
- RAM: ~400-500 МБ
- Latency: ~50-100 мс (от `/scan` до `/cmd_vel`)

**Оптимизации**:

#### 1. Уменьшить частоту обновления

```yaml
# Local costmap
update_frequency: 3.0      # было 5.0
publish_frequency: 1.0     # было 2.0

# Global costmap
update_frequency: 0.5      # было 1.0
publish_frequency: 0.5     # было 1.0

# Controller
controller_frequency: 15.0  # было 20.0
```

#### 2. Уменьшить размер local costmap

```yaml
width: 2.5   # было 3.0
height: 2.5  # было 3.0
```

#### 3. Уменьшить количество семплов DWB

```yaml
vx_samples: 10       # было 20
vtheta_samples: 20   # было 40
```

#### 4. Упростить costmap слои

```yaml
# Отключить voxel_layer, использовать obstacle_layer
plugins: ["obstacle_layer", "inflation_layer"]
```

### Точность vs Скорость

**Режим "Точность"** (медленно, но точно):
```yaml
xy_goal_tolerance: 0.10       # 10 см
controller_frequency: 20.0    # 20 Hz
PathAlign.scale: 48.0         # Строго следовать пути
sim_time: 2.0                 # Дальше предсказание
```

**Режим "Скорость"** (быстро, но менее точно):
```yaml
xy_goal_tolerance: 0.25       # 25 см
controller_frequency: 10.0    # 10 Hz
PathAlign.scale: 16.0         # Меньше внимания к пути
sim_time: 1.0                 # Короткое предсказание
```

## 📚 Связанные файлы

- **Dockerfile**: `docker/main/nav2/Dockerfile`
- **Конфигурация**: `docker/main/config/nav2/nav2_params.yaml`
- **Costmaps**: `docker/main/config/nav2/*_costmap_params.yaml`
- **Behavior Trees**: `docker/main/config/nav2/behavior_trees/*.xml`
- **Startup скрипт**: `docker/main/scripts/nav2/start_nav2.sh`
- **Docker Compose**: `docker/main/docker-compose.yaml`

## 🔗 Ресурсы

- [Nav2 Official Documentation](https://navigation.ros.org/)
- [Nav2 GitHub](https://github.com/ros-planning/navigation2)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [Costmap2D](https://navigation.ros.org/configuration/packages/costmap-plugins/index.html)

## ✅ Checklist для запуска

- [ ] robot-state-publisher работает (`/robot_description` topic)
- [ ] lslidar публикует сканы (`/scan` topic)
- [ ] rtabmap публикует карту (`/map` topic)
- [ ] ros2_control публикует одометрию (`/odom` topic)
- [ ] TF дерево полное (`map` → `odom` → `base_link`)
- [ ] Nav2 ноды запущены (`ros2 node list`)
- [ ] Costmaps обновляются (`ros2 topic hz /local_costmap/costmap`)
- [ ] Можно отправить goal_pose (`ros2 topic pub /goal_pose ...`)
- [ ] Робот реагирует на `/cmd_vel` (двигается)

---

**Примечание**: Для полноценной работы Nav2 необходима готовая карта от RTAB-Map. Сначала создайте карту в режиме SLAM, затем используйте Nav2 для навигации по ней.
