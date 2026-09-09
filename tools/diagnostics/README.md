# Diagnostics Tools

Набор скриптов для диагностики ROS 2 топиков и данных робота.

## Скрипты

### check_joints.py
Проверка публикации данных `/joint_states` от VESC моторов.
```bash
cd /home/ros2/rob_box_project
source install/setup.bash
python3 tools/diagnostics/check_joints.py
```

### check_odom.py
Проверка публикации одометрии `/odom` от колес.
```bash
python3 tools/diagnostics/check_odom.py
```

### compare_odom.py
Сравнение одометрии от VESC с другими источниками (RTAB-Map, IMU).
```bash
python3 tools/diagnostics/compare_odom.py
```

## Требования
- ROS 2 Humble
- Активное окружение: `source install/setup.bash`
