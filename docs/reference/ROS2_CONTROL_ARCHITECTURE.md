# Rob Box - Архитектура ROS2 Control и Robot State Publisher

## 🏗️ Общая архитектура

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Rob Box Main Pi                             │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────┐
│  1. URDF (rob_box_main.xacro)                                        │
│  ─────────────────────────────────────────────────────────────────── │
│  • Определяет geometry (links, joints)                               │
│  • Включает ros2_control блок (rob_box_ros2_control.xacro)          │
│  • Используется ОБОИМИ: robot_state_publisher и controller_manager   │
└──────────────────────────────────────────────────────────────────────┘
                                │
                                ├─────────────────────┐
                                ▼                     ▼
┌───────────────────────────────────────┐  ┌─────────────────────────────────┐
│  2. robot_state_publisher             │  │  3. controller_manager          │
│  ──────────────────────────────────── │  │  ──────────────────────────────│
│  ЧИТАЕТ из URDF:                      │  │  ЧИТАЕТ из URDF ros2_control:  │
│  • Links (base_link, wheels, sensors) │  │  • <hardware> плагин           │
│  • Joints (wheel_joints)              │  │  • <joint> конфигурация        │
│                                       │  │  • CAN IDs, poles, radius      │
│  ПОДПИСЫВАЕТСЯ на:                    │  │                                │
│  • /joint_states (от                  │  │  СОЗДАЕТ:                      │
│    joint_state_broadcaster)           │  │  • VescSystemHardwareInterface │
│                                       │  │  • joint_state_broadcaster     │
│  ПУБЛИКУЕТ:                           │  │  • diff_drive_controller       │
│  • TF: base_link → wheels             │  │                                │
│  • TF: base_link → lidar              │  │  ПОДПИСЫВАЕТСЯ на:             │
│  • TF: base_link → cameras            │  │  • /cmd_vel                    │
│  • TF: base_link → все статические    │  │                                │
│       компоненты                      │  │  ПУБЛИКУЕТ:                    │
│                                       │  │  • /joint_states (через        │
│  НЕ ПУБЛИКУЕТ:                        │  │    joint_state_broadcaster)    │
│  • /joint_states ❌                   │  │  • /odom (через                │
│  • /odom ❌                            │  │    diff_drive_controller)      │
│  • TF odom → base_link ❌             │  │  • TF: odom → base_link        │
│                                       │  │                                │
│  НЕ УПРАВЛЯЕТ:                        │  │  УПРАВЛЯЕТ:                    │
│  • Моторами ❌                         │  │  • VESC через CAN ✅            │
└───────────────────────────────────────┘  └─────────────────────────────────┘
                │                                          │
                │                                          │
                └──────────────► TF Tree ◄─────────────────┘
                               (объединенное)
```

## 📊 Детальное разделение обязанностей

### 1. URDF (`rob_box_main.xacro`)

**Используется:**
- ✅ `robot_state_publisher` - читает geometry (links, joints, визуализацию)
- ✅ `controller_manager` - читает `<ros2_control>` блок

**Содержит:**
```xml
<robot name="rob_box">
  <!-- Links и Joints для robot_state_publisher -->
  <link name="base_link">...</link>
  <link name="front_left_wheel">...</link>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="left_rocker"/>
    <child link="front_left_wheel"/>
  </joint>
  
  <!-- ros2_control блок для controller_manager -->
  <ros2_control name="VescSystem" type="system">
    <hardware>
      <plugin>vesc_nexus/VescSystemHardwareInterface</plugin>
      <param name="can_interface">can0</param>
    </hardware>
    
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
      <param name="can_id">49</param>
      <param name="poles">30</param>
    </joint>
    <!-- ... остальные колеса ... -->
  </ros2_control>
</robot>
```

**КРИТИЧНО:** Имена joints в URDF и ros2_control **ДОЛЖНЫ СОВПАДАТЬ**!

---

### 2. robot_state_publisher

**Запуск:**
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro rob_box_main.xacro)"
```

**Что делает:**

✅ **ПУБЛИКУЕТ TF трансформации:**
- **Статические:** base_link → lidar_link, camera_link, rocker_links, и т.д.
- **Динамические:** на основе `/joint_states`
  - base_link → left_rocker → front_left_wheel
  - base_link → right_rocker → rear_right_wheel
  - и т.д.

✅ **ПОДПИСЫВАЕТСЯ на:**
- `/joint_states` (sensor_msgs/JointState)
  - Получает положение и скорость каждого joint
  - Обновляет TF для подвижных joints

❌ **НЕ ДЕЛАЕТ:**
- Не публикует `/joint_states` (только читает!)
- Не управляет моторами
- Не вычисляет одометрию
- Не публикует TF `odom → base_link` (это делает diff_drive_controller)

**Topics:**
- Subscribe: `/joint_states`
- Publish: `/tf`, `/tf_static`

---

### 3. controller_manager (ros2_control_node)

**Запуск:**
```bash
ros2 launch controller_manager ros2_control_node.launch.py \
  robot_description:="$(xacro rob_box_main.xacro)" \
  controller_params_file:=controller_manager.yaml
```

**Что делает:**

✅ **Загружает hardware interface:**
- Читает `<ros2_control>` блок из URDF
- Создает экземпляр `VescSystemHardwareInterface`
- Передает параметры (CAN ID, poles, radius) в hardware interface

✅ **Управляет контроллерами:**
- Загружает и активирует контроллеры из конфига
- Вызывает `update()` каждого контроллера на каждом цикле

❌ **НЕ ДЕЛАЕТ:**
- Не публикует TF для статических частей робота (это делает robot_state_publisher)
- Не публикует `/joint_states` напрямую (это делает joint_state_broadcaster)

---

### 4. VescSystemHardwareInterface (плагин)

**Загружается:** controller_manager

**Что делает:**

✅ **Инициализация:**
- Открывает CAN интерфейс (`can0`)
- Подключается к VESC моторам по CAN ID
- Читает параметры из URDF (poles, wheel_radius, min_erpm)

✅ **Цикл update():**
```cpp
// Каждый цикл (50 Hz):
1. read() - читает состояние VESC через CAN:
   - RPM мотора
   - Положение (тахометр)
   - Ток
   - Температура
   - Напряжение
   
2. Преобразует в joint state:
   - RPM → velocity (rad/s)
   - Тахометр → position (rad)
   
3. write() - отправляет команды в VESC:
   - velocity command → RPM
   - Отправляет CAN фреймы
```

❌ **НЕ ДЕЛАЕТ:**
- Не публикует топики напрямую (это делают контроллеры)
- Не вычисляет одометрию (это делает diff_drive_controller)

---

### 5. joint_state_broadcaster (контроллер)

**Управляется:** controller_manager

**Что делает:**

✅ **Публикует `/joint_states`:**
```cpp
// Каждый цикл:
JointState msg;
msg.name = ["front_left_wheel_joint", "front_right_wheel_joint", ...];
msg.position = [...];  // от hardware interface
msg.velocity = [...];  // от hardware interface
msg.effort = [...];    // от hardware interface (если есть)
pub_joint_states.publish(msg);
```

**Topics:**
- Publish: `/joint_states` (sensor_msgs/JointState)

**Связь с robot_state_publisher:**
```
VescSystemHardwareInterface 
    → joint_state_broadcaster 
        → /joint_states 
            → robot_state_publisher 
                → TF (base_link → wheels)
```

---

### 6. diff_drive_controller (контроллер)

**Управляется:** controller_manager

**Что делает:**

✅ **Преобразует `/cmd_vel` → команды для колес:**
```cpp
// Входящая команда:
/cmd_vel: {linear.x: 1.0, angular.z: 0.5}

// Вычисляет скорости для каждого колеса:
left_velocity = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
right_velocity = (linear.x + angular.z * wheel_separation / 2) / wheel_radius

// Отправляет команды в hardware interface:
hardware_interface->set_command(front_left_wheel_joint, left_velocity)
hardware_interface->set_command(rear_left_wheel_joint, left_velocity)
hardware_interface->set_command(front_right_wheel_joint, right_velocity)
hardware_interface->set_command(rear_right_wheel_joint, right_velocity)
```

✅ **Вычисляет одометрию:**
```cpp
// Читает joint states от hardware interface:
left_pos = (front_left_pos + rear_left_pos) / 2
right_pos = (front_right_pos + rear_right_pos) / 2

// Вычисляет перемещение робота:
linear_x = (left_pos + right_pos) / 2 * wheel_radius
angular_z = (right_pos - left_pos) / wheel_separation * wheel_radius

// Публикует:
1. /odom (nav_msgs/Odometry)
2. TF: odom → base_link
```

**Topics:**
- Subscribe: `/cmd_vel` (geometry_msgs/Twist)
- Publish: `/odom` (nav_msgs/Odometry)
- Publish TF: `odom → base_link`

---

## 🔍 Проверка на дубли

### Нет дублей в TF:

| TF Transform         | Кто публикует            | Источник данных           |
|----------------------|--------------------------|---------------------------|
| `odom → base_link`   | diff_drive_controller    | Одометрия от колес        |
| `base_link → wheels` | robot_state_publisher    | /joint_states             |
| `base_link → lidar`  | robot_state_publisher    | URDF (статическая)        |
| `base_link → camera` | robot_state_publisher    | URDF (статическая)        |

✅ **Нет конфликтов!**

### Нет дублей в топиках:

| Topic           | Кто публикует            |
|-----------------|--------------------------|
| `/joint_states` | joint_state_broadcaster  |
| `/odom`         | diff_drive_controller    |
| `/cmd_vel`      | teleop / nav2 (внешние)  |
| `/tf`           | Оба, но разные фреймы    |

✅ **Нет конфликтов!**

---

## ⚙️ Конфигурационные файлы

### 1. URDF должен быть один:

**Используется:**
```bash
# robot_state_publisher
URDF_PATH=/ws/src/rob_box_description/urdf/rob_box_main.xacro

# controller_manager
URDF_PATH=/ws/src/rob_box_description/urdf/rob_box_main.xacro  # ТОТ ЖЕ!
```

✅ **Оба используют rob_box_main.xacro**

### 2. Контроллер конфиг:

```yaml
# controller_manager.yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    
diff_drive_controller:
  ros__parameters:
    left_wheel_names: [front_left_wheel_joint, rear_left_wheel_joint]
    right_wheel_names: [front_right_wheel_joint, rear_right_wheel_joint]
    wheel_separation: 0.380  # track_width
    wheel_radius: 0.115
    enable_odom_tf: true     # ✅ diff_drive публикует odom→base_link
```

---

## 🚀 Последовательность запуска

```bash
# 1. Zenoh router
docker compose up -d zenoh-router

# 2. Robot State Publisher
# Читает URDF, ждет /joint_states
docker compose up -d robot-state-publisher

# 3. Controller Manager (ros2_control)
# Загружает VescSystemHardwareInterface, spawns контроллеры
# joint_state_broadcaster начинает публиковать /joint_states
docker compose up -d ros2-control-manager

# Теперь:
# - robot_state_publisher получает /joint_states
# - TF дерево полное: odom → base_link → wheels → sensors
# - /cmd_vel работает через diff_drive_controller → VESC

# 4. RTAB-Map
docker compose up -d rtabmap
```

---

## 📝 Итоговая схема данных

```
        /cmd_vel
           │
           ▼
    ┌──────────────────────┐
    │ diff_drive_controller│
    │  (в controller_mgr)  │
    └──────────────────────┘
           │ │
           │ └────────────► /odom, TF(odom→base_link)
           ▼
    ┌──────────────────────┐
    │ VescSystemHardware   │
    │     Interface        │
    └──────────────────────┘
           │ │
           │ └────────► joint states (internal)
           ▼
    ┌──────────────────────┐
    │ joint_state_         │
    │   broadcaster        │
    └──────────────────────┘
           │
           └────────────► /joint_states
                              │
                              ▼
                      ┌──────────────────┐
                      │ robot_state_     │
                      │   publisher      │
                      └──────────────────┘
                              │
                              └──► TF (base_link → wheels, sensors)
```

**Вывод:** ✅ Нет дублей! Каждый компонент делает свою работу.
