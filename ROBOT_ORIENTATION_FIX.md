# Исправление ориентации робота в RViz

**Дата:** 2025-11-19  
**Проблема:** Робот отображается повёрнутым на 90° в RViz, колёса в одном месте

## Проблемы

### 1. Неправильная ориентация
Робот экспортирован из Fusion 360 с системой координат:
- **Fusion 360:** Y → вперёд, X → влево, Z → вверх
- **ROS (REP-103):** X → вперёд, Y → влево, Z → вверх

**Результат:** Робот повёрнут на 90° против часовой стрелки в RViz.

### 2. Несовпадение имён джойнтов
Три разных системы именования в разных файлах:
- `rob_box.xacro`: `wheel_rear_left_1_joint`, `wheel_front_left_1_joint`...
- `robot_controller.yaml`: `front_left_wheel_joint`, `rear_left_wheel_joint`...
- `rob_box_ros2_control.xacro`: `left_front_wheel_joint`, `left_rear_wheel_joint`...

**Результат:** Колёса не отображались в правильных позициях.

## Решение

### 1. Добавлен base_footprint (REP-120)
```xml
<link name="base_footprint">
  <visual>
    <geometry>
      <sphere radius="0.001"/>
    </geometry>
  </visual>
</link>
```

### 2. Поворот base_link на -90° вокруг Z
```xml
<joint name="base_footprint_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 ${wheel_radius}" rpy="0 0 -1.5708"/>
</joint>
```

**Поворот:** `-1.5708` радиан = -90° = -π/2  
**Смещение:** `z="${wheel_radius}"` = 0.115м (base_footprint на уровне пола)

### 3. Унификация имён джойнтов

Все файлы приведены к единому стандарту из `robot_controller.yaml`:

| Колесо | Имя джойнта |
|--------|-------------|
| Переднее левое | `front_left_wheel_joint` |
| Заднее левое | `rear_left_wheel_joint` |
| Переднее правое | `front_right_wheel_joint` |
| Заднее правое | `rear_right_wheel_joint` |

**Изменённые файлы:**
- ✅ `src/rob_box_description/urdf/rob_box.xacro`
- ✅ `src/rob_box_description/urdf/rob_box_ros2_control.xacro`
- ✅ `src/rob_box_bringup/scripts/dummy_joint_state_publisher.py`
- ✅ `docker/main/config/vesc_nexus/robot_controller.yaml` (уже было правильно)

### 4. Меши колёс

Созданы меши с правильными именами:
- ✅ `front_left_wheel.stl` (скопирован из `wheel_front_left.stl`)
- ✅ `rear_left_wheel.stl` (скопирован из `wheel_rear_left.stl`)
- ✅ `front_right_wheel.stl` (скопирован из `wheel_front_right.stl`)
- ✅ `rear_right_wheel.stl` (скопирован из `wheel_rear_right.stl`)

**Примечание:** Файлы `wheel_*_1.stl` используются только в старом экспорте `URDF_EXPORT` и не удалены для совместимости.

## TF дерево (после исправления)

```
base_footprint (корневой фрейм, на уровне пола)
  └─ base_link (повёрнут -90° вокруг Z, поднят на 0.115м)
       ├─ front_left_wheel
       ├─ rear_left_wheel
       ├─ front_right_wheel
       ├─ rear_right_wheel
       ├─ lslidar_n10
       ├─ camera_link
       │    └─ camera → (rgb/stereo/imu frames)
       └─ ceiling_camera_link
            └─ ceiling_camera_optical_frame
```

## Проверка

### Визуализация в RViz
```bash
ros2 launch rob_box_bringup display.launch.py
```

**Ожидаемый результат:**
- ✅ Робот ориентирован правильно (X вперёд)
- ✅ Колёса в правильных позициях
- ✅ Все TF фреймы корректны

### Тестирование с реальными моторами
```bash
ros2 launch rob_box_bringup rob_box_control.launch.py
```

## Ссылки

- [REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP-120: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0120.html)
- [TF Naming Convention](http://wiki.ros.org/tf/Tutorials)
