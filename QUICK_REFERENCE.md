# Rob Box - Quick Reference
**Обновлено:** 2025-10-11

## ⚙️ Параметры робота

```yaml
# Геометрия
wheel_separation: 0.390 м  # База (left↔right по X)
wheel_radius: 0.115 м      # Радиус колеса
track: 0.289 м             # Колея (front↔rear по Y)

# Колеса (CAN IDs для VESC)
front_left:  49   # X= 0.195, Y=-0.146
front_right: 124  # X=-0.196, Y=-0.145
rear_left:   81   # X= 0.195, Y= 0.144
rear_right:  94   # X=-0.195, Y= 0.144

# Сенсоры (относительно base_link)
lidar:     X=-0.0003, Y=0.171, Z=0.477
oak_d:     X=-0.0002, Y=0.116, Z=0.460
rpi_cam:   X= 0.065,  Y=0.171, Z=0.462
```

## 🐳 Docker команды

```bash
# Main Robot (ros2_control + vesc_nexus)
cd docker/main
docker compose up -d

# Vision (cameras + lidar)
cd docker/vision
docker compose up -d

# Логи
docker compose logs -f ros2-control-manager
docker compose logs -f robot-state-publisher
```

## 🔧 ROS2 команды

```bash
# Проверить топики
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /odom

# Управление
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# TF дерево
ros2 run tf2_tools view_frames
evince frames.pdf

# Controller manager
ros2 control list_controllers
ros2 control load_controller diff_drive_controller
ros2 control switch_controllers --activate diff_drive_controller
```

## 📂 Важные файлы

```
FROM_FUSION_360/urdf/URDF_ROBBOX.xacro  # Fusion 360 экспорт ✅
src/rob_box_description/urdf/
  ├── rob_box_main.xacro                # Главный URDF
  ├── rob_box_ros2_control.xacro        # ros2_control блок
  └── sensors/rob_box_sensors.xacro     # Сенсоры

docker/main/config/controllers/
  └── controller_manager.yaml           # Конфиг контроллеров ✅

docker/main/ros2_control/Dockerfile     # Controller manager
docker/main/scripts/ros2_control/
  └── start_ros2_control.sh             # Startup script
```

## 🔌 CAN интерфейс

```bash
# Настройка CAN (Main Pi)
sudo docker/scripts/setup_can0.sh

# Проверка
ip link show can0
candump can0

# Отправка тестовой команды
cansend can0 031#1122334455667788
```

## 📊 Monitoring

```bash
# System resources
htop
docker stats

# CAN traffic
candump can0 -c  # с временными метками

# ROS2 graph
rqt_graph

# Параметры контроллера
ros2 param list /diff_drive_controller
ros2 param get /diff_drive_controller wheel_separation
```

## 🐛 Troubleshooting

```bash
# VESC не отвечает
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000

# Controller не загружается
ros2 control list_hardware_interfaces
cat /var/log/ros2_control.log

# TF проблемы
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

## 📝 Документация

- `FUSION360_FINAL_MEASUREMENTS.md` - Финальные измерения
- `ROS2_CONTROL_ARCHITECTURE.md` - Архитектура ros2_control
- `VESC_INTEGRATION_PROGRESS.md` - Прогресс интеграции
- `docs/guides/POWER_MANAGEMENT.md` - Управление питанием

## 🎯 Следующие шаги

1. Адаптировать rob_box_main.xacro под новые координаты
2. Добавить ros2-control-manager в docker-compose.yaml
3. Протестировать на железе с CAN Shield
4. Калибровка diff_drive_controller

---
**Создано:** GitHub Copilot
**Проект:** https://github.com/krikz/rob_box_project
