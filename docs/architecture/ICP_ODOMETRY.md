# ICP Одометрия и Wheel Odometry Fusion

[← Назад к архитектуре](README.md) | [Обзор системы](SYSTEM_OVERVIEW.md) | [Софт](SOFTWARE.md)

## 📋 Обзор

**ICP (Iterative Closest Point)** — алгоритм сопоставления облаков точек для оценки движения робота. В нашей системе ICP одометрия **корректирует** колёсную одометрию используя данные лидара.

### Почему нужна ICP одометрия?

| Источник | Преимущества | Недостатки |
|----------|--------------|------------|
| **Wheel Odometry** | Высокая частота (50 Hz), не зависит от окружения | Накапливает ошибку при проскальзывании, повороты неточные |
| **ICP Odometry** | Корректирует drift по лидару, точнее при поворотах | Может потерять трек в бедных feature средах (коридоры) |
| **Fusion (наш подход)** | Wheel как guess + ICP как коррекция | Лучшее из обоих миров |

## 🏗️ Архитектура

```
┌─────────────────────────────────────────────────────────────────────┐
│                           Main Pi (10.1.1.10)                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐         ┌──────────────────────────────────────┐ │
│  │   LSLidar    │ /scan   │              RTABMAP                 │ │
│  │   C16        │────────►│  ┌────────────┐   ┌──────────────┐   │ │
│  └──────────────┘         │  │icp_odometry│   │    rtabmap   │   │ │
│                           │  │            │   │    (SLAM)    │   │ │
│  ┌──────────────┐         │  │ Корректи-  │   │              │   │ │
│  │ ros2-control │ TF      │  │ рует wheel │──►│ Строит карту │   │ │
│  │  (VESC)      │────────►│  │ odometry   │   │ и локализует │   │ │
│  │              │ odom→   │  │ по лидару  │   │              │   │ │
│  │ Wheel Odom   │ base    │  └────────────┘   └──────────────┘   │ │
│  └──────────────┘         │        │                  │          │ │
│                           │        │ TF               │ /map     │ │
│                           │        ▼                  ▼          │ │
│                           │   icp_odom →        OccupancyGrid    │ │
│                           │   base_footprint                     │ │
│                           └──────────────────────────────────────┘ │
│                                                                     │
│  ┌──────────────┐                                                  │
│  │    Nav2      │◄─── Использует map + icp_odom для навигации     │
│  │  Navigation  │                                                  │
│  └──────────────┘                                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 🔧 Узлы и их роли

### 1. ros2-control (VESC Nexus)
**Контейнер:** `ros2-control`  
**Функция:** Управление моторами и публикация колёсной одометрии

```yaml
# Публикует:
- /odom                    # nav_msgs/Odometry (wheel odometry)
- TF: odom → base_footprint  # Трансформация на основе энкодеров

# Параметры:
wheel_radius: 0.138        # Калиброван по тесту 4.45м
wheel_separation: 0.390    # База робота
publish_rate: 50.0 Hz      # Частота публикации
```

**Как работает:**
1. Читает положение колёс через CAN (VESC контроллеры)
2. Считает пройденное расстояние: `distance = wheel_rotation × wheel_radius`
3. Использует дифференциальную кинематику для расчёта позиции
4. Публикует TF и /odom топик

### 2. icp_odometry (RTABMAP)
**Контейнер:** `rtabmap`  
**Функция:** Коррекция одометрии по лидарным сканам

```yaml
# Подписывается:
- /scan                    # sensor_msgs/LaserScan
- TF: odom → base_footprint  # Wheel odometry как guess

# Публикует:
- /rtabmap/odom            # nav_msgs/Odometry (скорректированная)
- TF: icp_odom → base_footprint  # Скорректированная трансформация

# Ключевые параметры:
guess_frame_id: odom       # Использовать wheel odom как начальную оценку
odom_frame_id: icp_odom    # Публиковать в свой frame
```

**Как работает ICP:**
1. Получает новый лидарный скан
2. Читает wheel odometry TF (`odom → base_footprint`) как **начальную оценку** (guess)
3. Сопоставляет текущий скан с предыдущим методом ICP:
   - Находит ближайшие точки между сканами
   - Итеративно минимизирует ошибку трансформации
   - Повторяет до сходимости (Icp/Iterations: 30)
4. Комбинирует guess с ICP результатом
5. Публикует скорректированную одометрию

**Алгоритм ICP (упрощённо):**
```
Input: Скан A (предыдущий), Скан B (текущий), Guess (от wheel odom)
Output: Трансформация T (rotation + translation)

1. Применить Guess к скану B
2. Repeat до сходимости:
   a. Для каждой точки в B найти ближайшую в A
   b. Вычислить оптимальную трансформацию T
   c. Применить T к B
   d. Если ошибка < epsilon, выход
3. Return T
```

### 3. rtabmap (SLAM)
**Контейнер:** `rtabmap`  
**Функция:** Построение карты и локализация

```yaml
# Подписывается:
- /scan                     # Для loop closure detection
- /rtabmap/odom            # От icp_odometry
- /camera/camera/imu       # IMU для graph optimization (OAK-D, namespace='camera')

# Публикует:
- /map                     # nav_msgs/OccupancyGrid
- /rtabmap/mapData         # Внутренние данные карты
- TF: map → icp_odom       # Map-to-odom коррекция

# Ключевые параметры:
Reg/Strategy: 1            # ICP регистрация
Reg/Force3DoF: true        # 2D SLAM (x, y, yaw)
RGBD/NeighborLinkRefining: true  # Уточнение связей
RGBD/ProximityBySpace: true      # Proximity detection
```

**Как работает:**
1. Получает одометрию от icp_odometry
2. Добавляет новые узлы в граф по критериям (время, расстояние)
3. Ищет loop closures (замыкания петель)
4. Оптимизирует граф при обнаружении loop closure
5. Публикует TF `map → icp_odom` для коррекции глобальной позиции

## 📊 TF Дерево

```
                    map
                     │
                     │ (rtabmap публикует, коррекция loop closure)
                     ▼
                 icp_odom
                     │
                     │ (icp_odometry публикует, скорректированная по лидару)
                     ▼
               base_footprint
                     │
                     │ (robot_state_publisher, статические TF)
                     ▼
    ┌────────────────┼────────────────┐
    │                │                │
 base_link      laser_link       camera_link
                                      │
                                  imu_link
```

**Параллельно существует:**
```
                    odom
                     │
                     │ (ros2-control публикует, wheel odometry)
                     ▼
               base_footprint
```

> **Важно:** Есть два пути к `base_footprint`:
> - `odom → base_footprint` (wheel odometry, используется как guess)
> - `icp_odom → base_footprint` (скорректированная, используется Nav2)

## ⚙️ Параметры ICP

### Файл: `docker-compose.yaml` (rtabmap сервис)

```yaml
icp_odometry:=true            # Включить ICP одометрию
odom_guess_frame_id:=odom     # Wheel odometry как guess
vo_frame_id:=icp_odom         # Frame для ICP odom
odom_frame_id:=icp_odom       # RTABMAP использует ICP odom
```

### Параметры ICP алгоритма (в args):

| Параметр | Значение | Описание |
|----------|----------|----------|
| `Icp/VoxelSize` | 0.05 | Размер воксела для downsampling (5 см) |
| `Icp/MaxCorrespondenceDistance` | 0.1 | Макс. расстояние для соответствия точек |
| `Icp/Iterations` | 30 | Количество итераций ICP |
| `Icp/Epsilon` | 0.001 | Порог сходимости |
| `Icp/MaxTranslation` | 0.3 | Макс. перемещение за итерацию (м) |
| `Icp/MaxRotation` | 0.78 | Макс. поворот за итерацию (~45°) |
| `Icp/CorrespondenceRatio` | 0.05 | Мин. доля точек с соответствием |

## 📚 Официальные значения RTAB-Map

Ниже собраны ключевые параметры из официальной документации RTAB-Map, которые важны для нашей текущей связки `LiDAR + ICP + landmarks`.

### Reg/Strategy

Официально:

```text
Reg/Strategy: 0=Vis, 1=Icp, 2=VisIcp
```

Что это значит у нас:
- `0=Vis` — регистрация по визуальным признакам камеры
- `1=Icp` — регистрация по scan/point cloud через ICP
- `2=VisIcp` — комбинированная регистрация: visual + ICP

В текущем стеке используется `Reg/Strategy=1`, то есть RTAB-Map делает регистрацию и loop closure через LiDAR/ICP, а AprilTag landmarks участвуют как дополнительные ограничения графа, а не как основной registration pipeline.

### Icp/Strategy

Официально:

```text
Icp/Strategy: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
```

Это внутренний выбор реализации самого ICP. Он не меняет архитектуру `Vis vs Icp`, а только backend ICP.

### Grid/Sensor

Официально:

```text
Grid/Sensor: 0=laser scan, 1=depth image(s), 2=both laser scan and depth image(s)
```

У нас используется `Grid/Sensor=0`, то есть occupancy grid строится только из LiDAR.

### Optimizer/Strategy

Официально:

```text
Optimizer/Strategy: 0=TORO, 1=g2o, 2=GTSAM, 3=Ceres
```

У нас зафиксирован `Optimizer/Strategy=1` (`g2o`) для более стабильной оптимизации графа в текущем режиме локализации.

### RGBD/OptimizeMaxError

Официально:

```text
Reject loop closures if optimization error ratio is greater than this value (0=disabled).
Ratio is computed as absolute error over standard deviation of each link.
```

Практический смысл:
- это не "вес AprilTag" и не "приоритет LiDAR"
- это предохранитель, который отбрасывает слишком плохие loop closures / localization constraints
- слишком маленькое значение делает локализацию чрезмерно консервативной

### Marker-параметры для landmarks

Официально важные для AprilTag/landmarks параметры:

```text
Marker/VarianceLinear
Marker/VarianceAngular
Marker/VarianceOrientationIgnored
```

Практический смысл:
- `Marker/VarianceLinear` — насколько жёстко доверять линейной позиции landmark
- `Marker/VarianceAngular` — насколько жёстко доверять ориентации landmark
- `Marker/VarianceOrientationIgnored=true` — оптимизировать только позицию landmark без ориентации, если yaw/pitch/roll у тега шумные

Для нашей задачи это ближе к "весу" AprilTag landmarks, чем `Reg/Strategy`.

### Вывод для Rob Box

С точки зрения RTAB-Map текущая архитектура уже жёстко LiDAR-first:
- `Reg/Strategy=1`
- `Grid/Sensor=0`
- `subscribe_scan=true`
- `icp_odometry=true`
- `subscribe_rgbd=false`

Поэтому вопрос "что сильнее, LiDAR или AprilTag?" в нашей конфигурации решается не одним параметром, а набором факторов:
- выбором registration pipeline (`Reg/Strategy`)
- параметрами ICP (`Icp/*`)
- правилами принятия loop closures (`RGBD/OptimizeMaxError`)
- дисперсиями landmark constraints (`Marker/*Variance*`)

## 🔄 Поток данных

```
[LSLidar C16]
     │
     │ /scan (10 Hz)
     ▼
┌────────────────────────────────────────────────────────┐
│                    icp_odometry                        │
│                                                        │
│  1. Получить /scan                                     │
│  2. Получить TF odom→base_footprint (wheel guess)     │
│  3. Сопоставить сканы (ICP алгоритм)                  │
│  4. Скорректировать guess                             │
│  5. Публиковать TF icp_odom→base_footprint            │
│  6. Публиковать /rtabmap/odom                         │
│                                                        │
└────────────────────────────────────────────────────────┘
     │
     │ TF + /rtabmap/odom
     ▼
┌────────────────────────────────────────────────────────┐
│                      rtabmap                           │
│                                                        │
│  1. Получить одометрию от icp_odometry                │
│  2. Добавить узел в граф                              │
│  3. Искать loop closures                              │
│  4. Оптимизировать граф                               │
│  5. Публиковать TF map→icp_odom                       │
│  6. Публиковать /map                                  │
│                                                        │
└────────────────────────────────────────────────────────┘
     │
     │ /map + TF
     ▼
┌────────────────────────────────────────────────────────┐
│                       Nav2                             │
│                                                        │
│  Использует:                                           │
│  - /map для планирования пути                         │
│  - TF map→icp_odom→base_footprint для локализации    │
│  - /cmd_vel для управления                            │
│                                                        │
└────────────────────────────────────────────────────────┘
```

## ❓ FAQ

### Почему не использовать только wheel odometry?
При проскальзывании колёс (например, при повороте на месте) накапливается ошибка. В тестах ошибка достигала ~32° при повороте.

### Почему не использовать только ICP?
ICP может "потерять трек" в бедных feature средах (длинные коридоры, пустые комнаты). Wheel odometry как guess делает ICP более робастным.

### Что если лидар временно закрыт?
ICP будет использовать wheel odometry guess пока не получит хороший скан.

### Как влияет частота сканирования?
Лидар работает на 10 Hz. ICP обрабатывает каждый скан. Wheel odometry на 50 Hz обеспечивает точный guess между сканами.

## 🔗 Связанные документы

- [Обзор системы](SYSTEM_OVERVIEW.md) — общая архитектура
- [Аппаратное обеспечение](HARDWARE.md) — LSLidar C16, VESC
- [Программное обеспечение](SOFTWARE.md) — ROS 2, RTABMAP
- [RTABMAP конфигурация](../../docker/main/config/rtabmap/rtabmap.yaml) — параметры

---
**Обновлено:** 15 декабря 2025  
**Автор:** AI Agent
