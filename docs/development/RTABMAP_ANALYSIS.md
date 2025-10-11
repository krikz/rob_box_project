# RTABMap для rob_box_project - Анализ и Применение

## Обзор RTABMap (из видео микрокурса)

**RTABMap** = **R**eal-**T**ime **A**ppearance **B**ased **Map**ping

### Что это?
- RGB-D SLAM подход (Simultaneous Localization and Mapping)
- Работает с RGB-D сенсорами (Kinect, Asus Xtion, RealSense, и т.д.)
- Основан на **loop closure detector** (детектор замыкания петель)
- Есть ROS wrapper: `rtabmap_ros` package (создан Mathieu Labbé)

### Основная идея
RTABMap создаёт:
1. **2D карту** (occupancy grid map) - для навигации
2. **3D карту** (point cloud) - для визуализации и понимания окружения
3. **База данных изображений** с ключевыми точками для локализации

## Как работает Loop Closure Detector

### Процесс создания карты:

1. **Сбор изображений**
   - Робот движется и камера делает снимки
   - Каждое изображение сохраняется в базу данных

2. **Извлечение ключевых точек**
   - Каждое изображение анализируется
   - Находятся характерные элементы (углы, пересечения линий, уникальные объекты)
   - Отмечаются **жёлтыми дисками** в database viewer

3. **Сравнение изображений**
   - Когда робот возвращается в ранее посещенное место
   - Текущее изображение сравнивается с изображениями в БД
   - Совпадающие ключевые точки отмечаются **розовыми дисками**

4. **Loop Closure (замыкание петли)**
   - Если найдено совпадение → робот понимает, что он уже был здесь
   - Происходит корректировка карты (исправление ошибок одометрии)
   - Робот может локализоваться на карте

### Два режима работы:

**Mapping Mode (картографирование):**
```
mem_incremental_memory: true   # Строит новую карту
mem_init_wm_with_all_nodes: false
```

**Localization Mode (локализация):**
```
mem_incremental_memory: false  # Не добавляет новые узлы
mem_init_wm_with_all_nodes: true  # Загружает всю карту из БД
```

## Применение RTABMap к rob_box_project

### Текущая ситуация rob_box:

**У нас уже есть:**
- ✅ Nav2 navigation stack (картографирование с 2D лазером)
- ✅ ROS2 Humble
- ✅ Docker инфраструктура
- ⚠️ Нет RGB-D камеры (только 2D лидар в симуляции)

**RTABMap требует:**
- RGB-D камера (Kinect, RealSense D435, ZED, и т.д.)
- Опционально: 2D лазер + одометрия (улучшает результаты)

### Сценарии применения RTABMap

#### Сценарий 1: Дополнение к Nav2 (гибридный подход)

**Идея:** Использовать Nav2 для основной навигации, RTABMap для обогащения карты 3D данными.

**Преимущества:**
- 2D карта от Nav2 (надёжная, быстрая)
- 3D карта от RTABMap (понимание высоты объектов, лестниц, препятствий)
- Loop closure от RTABMap для коррекции долгосрочной одометрии

**Конфигурация:**
```yaml
# Nav2 создаёт 2D occupancy grid
nav2:
  local_costmap: ...
  global_costmap: ...

# RTABMap создаёт 3D point cloud
rtabmap:
  subscribe_depth: true
  subscribe_scan: true  # Используем тот же лазер что и Nav2
  rgbd_sync: true
```

**Топики:**
```
/scan              → Nav2 + RTABMap (общий)
/odom              → Nav2 + RTABMap (общий)
/camera/rgb/image  → RTABMap
/camera/depth/image → RTABMap
/map               → Nav2 (2D grid)
/rtabmap/cloud_map → RTABMap (3D point cloud)
```

#### Сценарий 2: RTABMap для indoor 3D mapping

**Идея:** Использовать RTABMap для создания детальных 3D карт помещений.

**Use cases:**
- Инспекция зданий
- Создание цифровых двойников
- Навигация в сложных 3D средах (многоэтажные здания)

**Особенности:**
- RTABMap как основной SLAM
- Nav2 использует карту от RTABMap
- 3D планировщик путей (если нужно)

#### Сценарий 3: Visual SLAM без лидара

**Идея:** Использовать только RGB-D камеру для SLAM (полезно, если лидар вышел из строя).

**Конфигурация:**
```yaml
rtabmap:
  subscribe_scan: false  # Не использовать лазер
  subscribe_depth: true
  visual_odometry: true  # Одометрия от камеры
  odom_frame_id: "camera_link"
```

**Преимущества:**
- Резервный режим навигации
- Удешевление робота (камера дешевле лидара)

**Недостатки:**
- Хуже работает в плохом освещении
- Медленнее чем лидар SLAM

### Интеграция с rob_box архитектурой

#### Добавление в Docker stack

**Новый сервис: `rtabmap-slam`**

```yaml
# docker/main/docker-compose.yaml
services:
  rtabmap-slam:
    image: ghcr.io/krikz/rob_box:rtabmap-humble-latest
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ./config/rtabmap:/config
      - rtabmap_db:/root/.ros/rtabmap.db  # Persistent database
    depends_on:
      - zenoh-router
    restart: unless-stopped

volumes:
  rtabmap_db:
```

**Dockerfile для RTABMap:**

```dockerfile
# docker/main/rtabmap/Dockerfile
ARG ROS_DISTRO=humble
FROM ghcr.io/krikz/rob_box_base:ros2-zenoh-${ROS_DISTRO}-latest

# Install RTABMap
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-rviz-plugins \
    ros-${ROS_DISTRO}-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY config/rtabmap_params.yaml /config/

CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && \
     ros2 launch rtabmap_ros rtabmap.launch.py \
     args:=\"--delete_db_on_start\" \
     params_file:=/config/rtabmap_params.yaml"]
```

#### Конфигурация параметров

**config/rtabmap_params.yaml:**
```yaml
rtabmap:
  ros__parameters:
    # Frame IDs
    frame_id: "base_link"
    odom_frame_id: "odom"
    map_frame_id: "map"
    
    # Subscribe topics
    subscribe_depth: true
    subscribe_scan: true
    subscribe_rgb: true
    
    # Database
    database_path: "~/.ros/rtabmap.db"
    
    # SLAM parameters
    mem_incremental_memory: true  # Mapping mode
    mem_init_wm_with_all_nodes: false
    
    # Grid map
    grid_size: 10.0
    grid_cell_size: 0.05
    
    # Optimization
    rgbd_optimize_from_graph_end: true
    
    # Loop closure
    kp_max_features: 400
    mem_use_odom_features: true
```

#### Launch файл для rob_box

**src/rob_box_bringup/launch/rtabmap_mapping.launch.py:**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Arguments
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Start in localization mode'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to RTABMap database'
    )
    
    # RTABMap node
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_rgb': True,
            'database_path': LaunchConfiguration('database_path'),
            'mem_incremental_memory': LaunchConfiguration('localization') == 'false',
            'mem_init_wm_with_all_nodes': LaunchConfiguration('localization') == 'true',
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('scan', '/scan'),
            ('odom', '/odom'),
        ]
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '$(find rob_box_bringup)/rviz/rtabmap.rviz']
    )
    
    return LaunchDescription([
        localization_arg,
        database_path_arg,
        rtabmap_node,
        rviz_node
    ])
```

### Workflow для использования RTABMap

#### 1. Mapping (создание карты)

```bash
# Запустить RTABMap в режиме картографирования
ros2 launch rob_box_bringup rtabmap_mapping.launch.py localization:=false

# Управлять роботом (телеопа или autonomous exploration)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Визуализация в RViz
# - Добавить /rtabmap/grid_map (OccupancyGrid)
# - Добавить /rtabmap/cloud_map (PointCloud2)
# - Добавить /rtabmap/mapData для loop closures

# Остановить картографирование
# Карта сохранена в ~/.ros/rtabmap.db
```

#### 2. Database Viewer (просмотр БД)

```bash
# Открыть database viewer
rtabmap-databaseViewer ~/.ros/rtabmap.db

# В viewer:
# - View → Constraints view (увидеть loop closures)
# - View → Graph view (увидеть граф позиций)
# - Scroll images → увидеть ключевые точки (жёлтые/розовые диски)
# - Edit → Export poses → сохранить траекторию
```

#### 3. Localization (локализация на готовой карте)

```bash
# Запустить RTABMap в режиме локализации
ros2 launch rob_box_bringup rtabmap_mapping.launch.py localization:=true

# Переместить робота в известное место
# RTABMap найдёт loop closure и загрузит карту

# Отправить navigation goal через RViz или Nav2
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...
```

### Рекомендуемая конфигурация для rob_box

#### Минимальная (только картографирование):

**Hardware:**
- RGB-D камера (Intel RealSense D435, Kinect Azure, ZED)
- Существующий 2D лидар
- Одометрия (уже есть от VESC)

**Software stack:**
```
┌─────────────────────────────────────┐
│         Navigation (Nav2)            │
│  - Local/Global Costmaps             │
│  - Path Planning                     │
└─────────────┬───────────────────────┘
              │
              ├──→ 2D map from Nav2 SLAM
              │
┌─────────────▼───────────────────────┐
│         RTABMap                      │
│  - 3D Mapping                        │
│  - Loop Closure Detection            │
│  - Visual Odometry (backup)          │
└─────────────────────────────────────┘
```

#### Расширенная (с визуальной одометрией):

**Hardware:**
- 2× RGB-D камеры (stereo для лучшей точности)
- 2D лидар
- IMU (для fusing)

**Software stack:**
```
┌──────────────────────────────────────┐
│    Sensor Fusion (robot_localization) │
│  - Fuse: odom + visual_odom + IMU     │
└──────────┬───────────────────────────┘
           │
           ├──→ Filtered odometry
           │
┌──────────▼───────────────────────────┐
│         RTABMap + Nav2                │
│  - RGB-D SLAM                         │
│  - Loop Closure                       │
│  - 2D + 3D Navigation                 │
└──────────────────────────────────────┘
```

## Сравнение подходов

### Nav2 SLAM vs RTABMap SLAM

| Аспект | Nav2 (slam_toolbox) | RTABMap |
|--------|---------------------|---------|
| **Вход** | 2D лазер | RGB-D камера (+лазер) |
| **Карта** | 2D occupancy grid | 2D grid + 3D point cloud |
| **Loop Closure** | Scan matching | Visual features |
| **Долговременная стабильность** | Средняя | Отличная (благодаря visual loop closure) |
| **Освещение** | Не важно | Критично |
| **Вычисления** | Низкие | Средние/Высокие |
| **Память** | Низкая | Высокая (БД изображений) |
| **3D понимание** | Нет | Да |

### Рекомендация для rob_box:

**Гибридный подход:**
1. **Nav2** - основная навигация (быстро, надёжно)
2. **RTABMap** - обогащение карты 3D данными + loop closure коррекция

**Когда использовать только RTABMap:**
- Indoor навигация с плохой видимостью для лидара (стеклянные стены)
- Нужна 3D карта (инспекция, мониторинг)
- Multi-floor навигация (лидар не видит этажи)

## Tools и Debug

### Полезные команды

**Просмотр топиков:**
```bash
ros2 topic list | grep rtabmap

# Основные топики:
# /rtabmap/grid_map         - 2D карта
# /rtabmap/cloud_map        - 3D облако точек
# /rtabmap/mapData          - Граф и loop closures
# /rtabmap/odom             - Визуальная одометрия
# /rtabmap/info             - Статистика SLAM
```

**Мониторинг производительности:**
```bash
ros2 topic hz /rtabmap/mapData  # Частота обновления
ros2 topic bw /rtabmap/mapData  # Bandwidth

# Смотреть статистику в real-time
ros2 topic echo /rtabmap/info
```

**Управление базой данных:**
```bash
# Экспорт карты в разные форматы
rtabmap-export --poses ~/.ros/rtabmap.db poses.txt
rtabmap-export --global_map ~/.ros/rtabmap.db map.pcd  # Point cloud

# Очистка БД
rm ~/.ros/rtabmap.db
```

### Визуализация в RViz

**Необходимые плагины:**
```bash
ros-humble-rtabmap-rviz-plugins
```

**Добавить в RViz:**
- **MapCloud** - 3D point cloud map
- **MapGraph** - граф позиций и loop closures
- **Info** - статистика SLAM
- **Grid** - 2D occupancy grid

### Troubleshooting

**Проблема 1: "No loop closures detected"**
- Увеличить `kp_max_features` (больше ключевых точек)
- Уменьшить скорость движения робота
- Улучшить освещение
- Проверить `mem_use_odom_features: true`

**Проблема 2: "Map drift (карта плывёт)"**
- Включить loop closure: `rgbd_proximity_by_space: true`
- Использовать лазер: `subscribe_scan: true`
- Добавить IMU для фьюжинга

**Проблема 3: "High CPU usage"**
- Уменьшить частоту камеры
- Уменьшить `kp_max_features`
- Использовать `mem_reduced_map: true`

**Проблема 4: "Database too large"**
```bash
# Очистить старые узлы
rtabmap-console
> vacuum

# Или установить лимит
rtabmap:
  mem_stm_size: 30  # Только последние 30 узлов в рабочей памяти
```

## Интеграция с существующими компонентами rob_box

### 1. Интеграция с Zenoh

RTABMap топики через Zenoh bridge:

```yaml
# config/zenoh_bridge_config.json
{
  "topics": [
    "/rtabmap/grid_map",
    "/rtabmap/cloud_map",
    "/rtabmap/mapData",
    "/rtabmap/info"
  ]
}
```

### 2. Интеграция с Nav2

RTABMap grid_map → Nav2 global_costmap:

```yaml
# config/nav2_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "rtabmap_layer", "inflation_layer"]
      rtabmap_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: "/rtabmap/grid_map"
```

### 3. Интеграция с VESC одометрией

Фьюжинг wheel odom + visual odom:

```yaml
# config/ekf_params.yaml
ekf_filter_node:
  ros__parameters:
    odom0: /vesc/odom          # Wheel odometry
    odom1: /rtabmap/odom       # Visual odometry
    
    odom0_config: [true, true, false, ...]   # x, y, yaw
    odom1_config: [true, true, false, ...]   # Backup when wheels slip
```

### 4. Интеграция с robot_sensor_hub

Использовать температурные данные для адаптивной навигации:

```python
# Если камера перегрелась → снизить resolution
def temp_callback(self, msg):
    camera_temp = [d.value for d in msg.devices if d.device_id == 2]
    if camera_temp[0] > 70.0:
        self.set_camera_resolution('640x480')  # Снизить с 1920x1080
```

## CI/CD для RTABMap

### GitHub Actions workflow

```yaml
# .github/workflows/build-rtabmap.yml
build-rtabmap:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v3
    
    - name: Build RTABMap Docker image
      run: |
        cd docker/main/rtabmap
        docker build -t ghcr.io/${{ github.repository }}:rtabmap-humble-latest .
    
    - name: Test RTABMap launch
      run: |
        docker run --rm ghcr.io/${{ github.repository }}:rtabmap-humble-latest \
          ros2 launch rtabmap_ros rtabmap.launch.py --help
    
    - name: Push to GHCR
      run: docker push ghcr.io/${{ github.repository }}:rtabmap-humble-latest
```

## Roadmap для внедрения

### Фаза 0: Подготовка (если нет RGB-D камеры)
- [ ] Закупить Intel RealSense D435i или ZED 2
- [ ] Установить на робота
- [ ] Калибровать камеру
- [ ] Протестировать в симуляции (Gazebo с RGB-D plugin)

**Приоритет:** HIGH (без камеры RTABMap бесполезен)
**Время:** 1-2 недели

### Фаза 1: Базовая интеграция (2-3 дня)
- [ ] Создать Docker образ с RTABMap
- [ ] Добавить в docker-compose.yaml
- [ ] Создать launch файл для картографирования
- [ ] Протестировать mapping mode

### Фаза 2: Интеграция с Nav2 (2-3 дня)
- [ ] Настроить remap топиков
- [ ] Интегрировать RTABMap grid_map в Nav2 costmap
- [ ] Протестировать localization mode
- [ ] Сравнить с Nav2 SLAM качество карт

### Фаза 3: Визуализация и tools (1-2 дня)
- [ ] Настроить RViz конфигурацию
- [ ] Документировать database viewer workflow
- [ ] Создать скрипты экспорта карт

### Фаза 4: Production optimization (2-3 дня)
- [ ] Настроить параметры под конкретное железо
- [ ] Оптимизировать CPU/Memory usage
- [ ] Persistent database storage (volume)
- [ ] Monitoring и logging

### Фаза 5: Advanced features (опционально)
- [ ] Multi-session mapping
- [ ] Cloud-based map storage
- [ ] Visual odometry fusion с wheel odom
- [ ] 3D path planning

## Выводы

### Преимущества RTABMap для rob_box:

1. **✅ 3D Understanding** - робот понимает высоту объектов
2. **✅ Loop Closure** - коррекция долгосрочного дрифта одометрии
3. **✅ Visual Localization** - резервный метод локализации
4. **✅ Database Persistence** - карты хранятся долгосрочно
5. **✅ ROS2 Native** - есть официальный пакет для Humble

### Недостатки / Ограничения:

1. **❌ Требует RGB-D камеру** - дополнительные расходы
2. **❌ Зависимость от освещения** - хуже работает в темноте
3. **❌ Вычислительная нагрузка** - нужен мощный CPU/GPU
4. **❌ Размер БД** - может вырасти до нескольких GB

### Рекомендация:

**Приоритет:** СРЕДНИЙ-ВЫСОКИЙ
- Если планируется indoor навигация → HIGH
- Если только outdoor с лидаром → MEDIUM-LOW

**Лучший подход для rob_box:**
- **Nav2** для основной навигации (надёжно, быстро)
- **RTABMap** для:
  - 3D картографирование
  - Loop closure коррекция
  - Multi-floor navigation
  - Инспекция и мониторинг

**Следующий шаг:**
1. Определить, нужна ли RGB-D камера для вашего use case
2. Если да → закупить RealSense D435i
3. Создать proof-of-concept: простое картографирование в Gazebo
4. Сравнить результаты с Nav2 SLAM
5. Принять решение о полной интеграции

## Ресурсы

### Официальная документация:
- RTABMap Wiki: http://wiki.ros.org/rtabmap_ros
- RTABMap GitHub: https://github.com/introlab/rtabmap
- RTABMap ROS2: https://github.com/introlab/rtabmap_ros

### Tutorials:
- Видео источник: "RTABMap ROS Microcourse" (Articulated Robotics style)
- ROS Answers: https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:rtabmap/

### Похожие проекты:
- ORB-SLAM3 (альтернатива, только visual SLAM)
- VINS-Fusion (visual-inertial SLAM)
- OpenVSLAM (deprecated, но исторически важный)

### Hardware recommendations:
- **Best:** Intel RealSense D435i (IMU + RGB-D, $200-300)
- **Good:** ZED 2 (stereo vision, $450)
- **Budget:** Kinect v2 (discontinued, но работает, ~$100 used)
- **Avoid:** Kinect v1 (устаревший, плохое качество depth)
