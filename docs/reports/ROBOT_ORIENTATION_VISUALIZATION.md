# Визуальное объяснение исправления ориентации

## Проблема: Coordinate Frame Mismatch

### Fusion 360 координаты (экспорт):
```
        Y (вперёд)
        ↑
        |
        |
        └─────→ X (влево)
       /
      /
     Z (вверх)
```

### ROS REP-103 координаты (требуется):
```
        X (вперёд)
        ↑
        |
        |
        └─────→ Y (влево)
       /
      /
     Z (вверх)
```

## Решение: Rotation Transform

### Поворот на -90° вокруг оси Z

**До поворота** (Fusion 360):
```
      Front (Y+)
         ↑
         |
   Left  |  Right
   (X+)  |  (X-)
    ←────┼────→
         |
         |
         ↓
      Rear (Y-)
```

**После поворота** (ROS):
```
      Front (X+)
         ↑
         |
   Left  |  Right
   (Y+)  |  (Y-)
    ←────┼────→
         |
         |
         ↓
      Rear (X-)
```

## TF Transform

```xml
<joint name="base_footprint_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 0.115" rpy="0 0 -1.5708"/>
           ↑   ↑   ↑       ↑  ↑     ↑
           x   y   z      roll pitch yaw
```

**Параметры:**
- `xyz="0 0 0.115"` - поднять на высоту колеса (base_footprint на полу)
- `rpy="0 0 -1.5708"` - повернуть на -90° вокруг Z (-π/2 радиан)

## Позиции колёс

### В системе base_link (после поворота):

```
        front_left_wheel         front_right_wheel
             ●                          ●
             |                          |
    Y+ ←─────┼──────── X+ ──────────────┼─────→ Y-
             |                          |
             ●                          ●
        rear_left_wheel          rear_right_wheel
```

**Координаты колёс** (в системе base_link, ДО поворота):
```
Колесо              | X (left/right) | Y (front/rear) | Z (up)
--------------------|----------------|----------------|--------
front_left_wheel    |   +0.195       |   -0.145       | 0.029
rear_left_wheel     |   +0.195       |   +0.144       | 0.029
front_right_wheel   |   -0.196       |   -0.145       | 0.029
rear_right_wheel    |   -0.195       |   +0.144       | 0.029
```

**После поворота** координаты автоматически преобразуются в ROS convention!

## Проверка правильности

### Команда для проверки TF дерева:
```bash
ros2 run tf2_tools view_frames
```

### Ожидаемый вывод:
```
base_footprint
  └─ base_link (rot: 0, 0, -90°; trans: 0, 0, 0.115)
       ├─ front_left_wheel_joint
       ├─ rear_left_wheel_joint
       ├─ front_right_wheel_joint
       ├─ rear_right_wheel_joint
       ├─ lslidar_n10
       ├─ camera_link
       └─ ceiling_camera_link
```

### В RViz:
- **Ось X (красная)** должна указывать ВПЕРЁД ✅
- **Ось Y (зелёная)** должна указывать ВЛЕВО ✅
- **Ось Z (синяя)** должна указывать ВВЕРХ ✅
- Колёса должны быть по углам робота, не в одном месте ✅
