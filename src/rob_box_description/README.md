# rob_box_description

URDF-описание робота Rob Box: геометрия, кинематика, сенсоры и ros2_control интерфейс.

## Описание

Пакет содержит полное описание физической модели Rob Box: основной кузов, 4 колеса (skid-steer), установленные сенсоры (OAK-D, LiDAR, потолочная камера, IMU) и конфигурацию ros2_control.

## Файловая структура

```
rob_box_description/
├── urdf/
│   ├── rob_box.xacro          # Главный XACRO: собирает полную модель
│   ├── rob_box_ros2_control.xacro  # ros2_control интерфейс (VESC/CAN)
│   └── materials/             # Материалы для Gazebo/RViz
├── meshes/                    # STL-файлы компонентов
│   ├── base_link.stl          # Основной кузов
│   ├── body_cover.stl         # Крышка
│   ├── camera_oak.stl         # OAK-D камера
│   ├── camera_rpi.stl         # Raspberry Pi Camera
│   ├── lidar.stl              # LiDAR N10
│   ├── left_front_wheel.stl   # Колёса (4 шт)
│   ├── left_rear_wheel.stl
│   ├── right_front_wheel.stl
│   └── right_rear_wheel.stl
├── rviz/                      # Конфигурации RViz
├── worlds/                    # Worlds для Gazebo
└── models/                    # Дополнительные модели
```

## TF-дерево

```
base_footprint
└── base_link
    ├── left_front_wheel
    ├── left_rear_wheel
    ├── right_front_wheel
    ├── right_rear_wheel
    ├── camera_link        # OAK-D RGB камера
    │   └── camera_depth_optical_frame
    ├── lidar_link         # LSLIDAR N10
    └── imu_link
```

## ros2_control

Конфигурация `rob_box_ros2_control.xacro` описывает:
- 4 колеса: `left_front_wheel_joint`, `left_rear_wheel_joint`, `right_front_wheel_joint`, `right_rear_wheel_joint`
- Интерфейс управления: velocity command / velocity state
- Hardware plugin: `vesc_nexus/VescHardware` (CAN bus)

## Использование

В Docker контейнере `robot-state-publisher` на Main Pi:

```bash
# Публикует /robot_description и TF трансформации
docker logs robot-state-publisher -f

# Проверить TF
ros2 run tf2_tools view_frames

# RViz визуализация
ros2 launch rob_box_bringup display.launch.py
```

## Зависимости

- `urdf` — URDF parser
- `xacro` — XACRO preprocessor
- `robot_state_publisher` — TF publisher
- `ros2_control` — hardware abstraction layer
