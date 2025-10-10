# Конфигурация RTAB-Map для 2D LiDAR

> **Дата создания**: 2025-10-10  
> **Статус**: ✅ Решено и протестировано  
> **Автор**: AI Agent с помощью изучения исходников rtabmap_ros

## 🎯 Проблема

При запуске RTAB-Map с параметрами `scan_topic:=/scan` rtabmap всё равно подписывался на камерные топики:
```
/camera/rgb/image_rect_color
/camera/depth_registered/image_raw
/camera/rgb/camera_info
```

Несмотря на попытки установить `subscribe_depth:=false`, `subscribe_rgbd:=false`.

## 🔍 Корневая причина

Анализ исходного кода `/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py`:

```python
# Строка 45-46:
DeclareLaunchArgument('depth', 
    default_value=ConditionalText('false', 'true', 
        IfCondition(stereo == 'true')._predicate_func(context)))

DeclareLaunchArgument('subscribe_rgb', 
    default_value=LaunchConfiguration('depth'))
```

**Логика по умолчанию**:
1. Если `stereo=false` (по умолчанию) → `depth=true`
2. Если `depth=true` → `subscribe_rgb=true`
3. Node rtabmap получает параметры:
   ```python
   "subscribe_depth": LaunchConfiguration('depth'),      # true!
   "subscribe_rgb": LaunchConfiguration('subscribe_rgb'), # true!
   ```

**Дефолтные топики** (строки 455-457):
```python
DeclareLaunchArgument('rgb_topic', default_value='/camera/rgb/image_rect_color')
DeclareLaunchArgument('depth_topic', default_value='/camera/depth_registered/image_raw')
DeclareLaunchArgument('camera_info_topic', default_value='/camera/rgb/camera_info')
```

## ✅ Решение

### Вариант 1: Параметр `depth:=false` (РЕКОМЕНДУЕТСЯ)

**Для rtabmap.launch.py**:
```yaml
command: >
  ros2 launch rtabmap_launch rtabmap.launch.py
  depth:=false              # ← КЛЮЧЕВОЙ ПАРАМЕТР!
  subscribe_scan:=true
  scan_topic:=/scan
  icp_odometry:=true
  # ... остальные параметры
```

**Результат**:
- `depth=false` → `subscribe_rgb=false` → `subscribe_depth=false`
- rtabmap подписывается ТОЛЬКО на `/scan` и `/rtabmap/odom_info`

### Вариант 2: Явные параметры (для прямого запуска rtabmap node)

```python
parameters={
    'subscribe_depth': False,   # Явно отключаем depth
    'subscribe_rgb': False,     # Явно отключаем RGB
    'subscribe_rgbd': False,    # Явно отключаем RGBD
    'subscribe_scan': True,     # Включаем LaserScan
}
```

## 📚 Официальные примеры

### 1. TurtleBot3 Scan Only
**Файл**: `/opt/ros/humble/share/rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py`

```python
parameters={
    'frame_id':'base_footprint',
    'subscribe_depth':False,        # ← Явно FALSE
    'subscribe_rgb':False,          # ← Явно FALSE
    'subscribe_scan':True,
    'approx_sync':True,
    'use_action_for_goal':True,
    'Reg/Strategy':'1',             # ICP
    'Reg/Force3DoF':'true',         # 2D SLAM
    'RGBD/NeighborLinkRefining':'True',
    'Grid/RangeMin':'0.2',
    'Optimizer/GravitySigma':'0'
}

remappings=[('scan', '/scan')]
```

### 2. Husky 2D (Laser + RGB-D)
**Файл**: `/opt/ros/humble/share/rtabmap_demos/launch/husky/husky_slam2d.launch.py`

```python
rtabmap_parameters={
    'subscribe_rgbd':True,          # Используем RGBD
    'subscribe_scan':True,          # + LaserScan
    'use_action_for_goal':True,
    'odom_sensor_sync': True,
    'Mem/NotLinkedNodesKept':'false',
    'Grid/RangeMin':'0.7',
    'RGBD/OptimizeMaxError':'2',
}

# + отдельный rgbd_sync nodelet:
Node(
    package='rtabmap_sync', executable='rgbd_sync',
    parameters=[{'approx_sync':False}],
    remappings=[
        ('rgb/image', 'sensors/camera_0/color/image'),
        ('depth/image', 'sensors/camera_0/depth/image'),
        ('rgb/camera_info', 'sensors/camera_0/color/camera_info')
    ]
)
```

### 3. Demo Hector Mapping (ROS 1, но принцип тот же)
**Файл**: `rtabmap_demos/launch/demo_hector_mapping.launch`

```xml
<node name="rtabmap" pkg="rtabmap_slam" type="rtabmap">
    <param name="subscribe_rgb"   type="bool" value="false"/>
    <param name="subscribe_depth" type="bool" value="false"/>
    <param name="subscribe_rgbd"  type="bool" value="$(arg camera)"/>
    <param name="subscribe_scan"  type="bool" value="true"/>
    
    <remap from="scan" to="/jn0/base_scan"/>
    
    <!-- Параметры RTAB-Map -->
    <param name="Reg/Strategy"       value="1"/>    <!-- ICP -->
    <param name="Reg/Force3DoF"      value="true"/>
    <param name="RGBD/ProximityBySpace" value="true"/>
    <param name="Icp/VoxelSize"      value="0.05"/>
</node>
```

## 🎓 Рекомендации по конфигурации

### Базовая конфигурация (2D LiDAR без камеры)

```yaml
ros2 launch rtabmap_launch rtabmap.launch.py \
  depth:=false \                    # Отключает RGB/Depth подписку
  subscribe_scan:=true \            # Включает LaserScan
  scan_topic:=/scan \               # Топик лидара
  icp_odometry:=true \              # ICP одометрия из сканов
  frame_id:=base_link \             # База робота
  odom_frame_id:=odom \             # Фрейм одометрии
  qos:=1 \                          # Reliable QoS
  approx_sync:=false \              # Точная синхронизация
  rtabmap_viz:=false \              # Без GUI
  rviz:=false \                     # Без RVIZ
  visual_odometry:=false \          # Без визуальной одометрии
  database_path:=/maps/rtabmap.db \ # Путь к БД
  wait_for_transform:=0.5 \         # Таймаут TF
  args:="--delete_db_on_start"      # Удалить старую БД
```

### Дополнительные параметры RTAB-Map (через args:="-p ...")

```yaml
args:="-p Reg/Strategy:1 \              # 0=Visual, 1=ICP, 2=Visual+ICP
       -p Reg/Force3DoF:true \          # 2D SLAM (x,y,yaw)
       -p Icp/VoxelSize:0.05 \          # Фильтрация сканов (5см вокселы)
       -p Icp/MaxCorrespondenceDistance:0.1 \ # Макс расстояние для ICP
       -p RGBD/NeighborLinkRefining:true \    # Уточнение по сканам
       -p RGBD/ProximityBySpace:true \        # Детекция по расстоянию
       -p Grid/RangeMin:0.2 \           # Игнорировать точки ближе 20см
       -p Grid/Sensor:0 \               # Сетка из лидара (не из depth)
       -p Optimizer/GravitySigma:0"     # Отключить IMU constraints
```

## 🔧 Проверка конфигурации

### 1. Проверить параметры подписки

```bash
ros2 topic echo /rtabmap/info --field subscribe_scan,subscribe_rgb,subscribe_depth
```

**Ожидаемый результат**:
```
subscribe_scan: true
subscribe_rgb: false
subscribe_depth: false
```

### 2. Проверить активные топики

```bash
ros2 node info /rtabmap/rtabmap | grep -A20 "Subscribers:"
```

**Ожидаемый результат** (НЕ ДОЛЖНО быть camera топиков):
```
Subscribers:
  /scan: sensor_msgs/msg/LaserScan
  /rtabmap/odom_info: rtabmap_msgs/msg/OdomInfo
```

### 3. Проверить логи

```bash
ros2 launch ... | grep "subscribe_"
```

**Ожидаемый вывод**:
```
[rtabmap] rtabmap: subscribe_scan = true
[rtabmap] rtabmap: subscribe_rgb = false
[rtabmap] rtabmap: subscribe_depth = false
[rtabmap] rtabmap: subscribe_rgbd = false
```

## 📋 Таблица сравнения конфигураций

| Режим | depth | subscribe_rgb | subscribe_depth | subscribe_scan | Описание |
|-------|-------|---------------|-----------------|----------------|----------|
| **Laser Only** | `false` | `false` (авто) | `false` (авто) | `true` | Только 2D LiDAR |
| **RGB-D Only** | `true` (default) | `true` (авто) | `true` (авто) | `false` | Только камера |
| **RGB-D + Laser** | `true` | `true` | `true` | `true` | Камера + лидар |
| **Stereo** | `false` (авто) | `false` | `false` | опционально | Стерео камера |

## ⚠️ Частые ошибки

### ❌ Ошибка 1: Забыли `depth:=false`
```yaml
subscribe_scan:=true
scan_topic:=/scan
# ЗАБЫЛИ: depth:=false
```
**Результат**: rtabmap подпишется на `/camera/*` топики

### ❌ Ошибка 2: Попытка переопределить топики на `/null/*`
```yaml
rgb_topic:=/null/rgb
depth_topic:=/null/depth
```
**Проблема**: rtabmap всё равно создаст подписки, будет ждать данных

### ❌ Ошибка 3: Использовать `subscribe_depth:=false` без `depth:=false`
```yaml
subscribe_depth:=false  # НЕ РАБОТАЕТ!
subscribe_scan:=true
```
**Проблема**: Параметр `subscribe_depth` не существует в CLI, есть только `depth`

## ✅ Правильная конфигурация (итоговая)

```yaml
# docker/main/docker-compose.yaml
rtabmap:
  command: >
    ros2 launch rtabmap_launch rtabmap.launch.py
    args:="--delete_db_on_start"
    depth:=false                # ← КЛЮЧЕВОЙ ПАРАМЕТР
    subscribe_scan:=true
    scan_topic:=/scan
    icp_odometry:=true
    frame_id:=base_link
    odom_frame_id:=odom
    qos:=1
    approx_sync:=false
    rtabmap_viz:=false
    rviz:=false
    visual_odometry:=false
    odom_topic:=odom
    database_path:=/maps/rtabmap.db
    wait_for_transform:=0.5
```

## 📖 Источники

1. **Официальная документация**:
   - http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
   - Раздел "2D laser only"

2. **Исходный код**:
   - `/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py`
   - Строки 45-46 (логика depth → subscribe_rgb)

3. **Демо примеры**:
   - `turtlebot3_scan.launch.py` - Laser only
   - `husky_slam2d.launch.py` - Laser + RGB-D
   - `demo_hector_mapping.launch` - ROS 1 пример

4. **GitHub**:
   - https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py
   - https://github.com/introlab/rtabmap_ros/blob/master/rtabmap_demos/launch/demo_hector_mapping.launch

## 🧪 Тестирование

Созданы автотесты в `local_test/auto_test_v3.sh`:

```bash
# Тест с правильной конфигурацией
./local_test/auto_test_v3.sh 1

# Ожидаемый результат:
# subscribe_scan = true
# subscribe_rgb = false
# subscribe_depth = false
# rtabmap subscribed to:
#   /scan
#   /rtabmap/odom_info
```

## 📅 История изменений

- **2025-10-10**: Найдено решение через параметр `depth:=false`
- **2025-10-10**: Проанализированы все demo launch файлы
- **2025-10-10**: Создана документация с примерами
- **2025-10-10**: Обновлена конфигурация в `docker-compose.yaml`

---

**Статус**: ✅ Решено и протестировано локально  
**Следующий шаг**: Тестирование на Raspberry Pi (Main)
