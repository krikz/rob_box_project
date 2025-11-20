# Исправление base_frame_id для одометрии Rob Box

**Дата**: 2025-11-20  
**Проблема**: Расхождение между одометрией и визуальной моделью робота  
**Решение**: Изменить base_frame_id с base_link на base_footprint  
**Статус**: ✅ ИСПРАВЛЕНО

---

## 🔍 Корневая причина

### Найденная проблема

После PR #234, который восстановил поворот `base_footprint_joint` на -90°, возникла следующая ситуация:

1. **URDF модель**: `base_link` повернут на -90° относительно `base_footprint`
   ```xml
   <joint name="base_footprint_joint" type="fixed">
       <origin xyz="0 0 ${wheel_radius}" rpy="0 0 -1.5708"/>
   </joint>
   ```

2. **Одометрия**: публикуется с `base_frame_id: base_link`
   - Diff drive controller вычисляет одометрию относительно `base_link`
   - `base_link` уже повернут на -90° относительно `base_footprint`

3. **Результат**: 
   - Робот физически стоит в одном направлении
   - Одометрия показывает поворот на 90° влево (из-за поворота base_link)
   - Визуальная модель в RViz повернута на 90° вправо (относительно одометрии)

### Почему это проблема?

```
Physical Robot:    0°  (стоит прямо)
Odometry (base_link): -90° (повернут влево из-за base_footprint_joint)
Visual Model:     +90° (компенсация для отображения)

НЕСООТВЕТСТВИЕ! Одометрия должна быть относительно base_footprint, а не base_link!
```

---

## 🔧 Решение

### Концепция

По стандарту ROS (REP-120):
- **base_footprint**: корневой фрейм робота на уровне пола, ВСЕГДА без поворота
- **base_link**: физический фрейм робота, может быть повернут для соответствия модели
- **Одометрия**: должна публиковаться относительно `base_footprint`, а не `base_link`

### TF дерево (правильное)

```
odom (одометрия, публикуется diff_drive_controller)
  └─ base_footprint (без поворота, на уровне пола)
       └─ base_link (повернут на -90° вокруг Z через joint)
            ├─ колеса
            └─ сенсоры
```

### Изменения

**1. `docker/main/config/vesc_nexus/robot_controller.yaml` (строка 68):**
```diff
-    base_frame_id: base_link
+    base_frame_id: base_footprint
```

**2. `docker/main/config/vesc_nexus/vesc_config.yaml` (строка 89):**
```diff
-    base_frame_id: "base_link"
+    base_frame_id: "base_footprint"
```

---

## 📊 Физический смысл

### До исправления (НЕПРАВИЛЬНО)

```
odom → base_link (повернут на -90°)
       └─ колеса вращаются

Проблема: Одометрия публикуется в системе координат base_link,
которая повернута на -90° относительно физического робота.

Когда робот движется вперёд (физически):
- Колеса крутятся правильно
- НО одометрия считает движение в системе base_link (уже повернутой)
- Результат: одометрия показывает движение под углом 90°!
```

### После исправления (ПРАВИЛЬНО)

```
odom → base_footprint (БЕЗ ПОВОРОТА, на уровне пола)
       └─ base_link (повернут на -90° через fixed joint)
              └─ колеса

Решение: Одометрия публикуется в системе base_footprint,
которая НЕ повернута и соответствует физическому роботу.

Когда робот движется вперёд:
- Колеса крутятся правильно
- Одометрия считается в base_footprint (не повернут)
- base_link автоматически следует через fixed joint
- Результат: одометрия соответствует реальному движению! ✅
```

---

## ✅ Проверка

### Визуализация в RViz

```bash
# На Main Pi или через SSH
ros2 launch rob_box_bringup display.launch.py
```

**Ожидаемый результат:**
- ✅ Робот ориентирован правильно (X вперёд, Y влево)
- ✅ При движении вперёд одометрия движется вперёд (не под углом)
- ✅ TF трансформация `odom → base_footprint → base_link` корректна

### Проверка TF дерева

```bash
# Проверить структуру TF дерева
ros2 run tf2_tools view_frames
evince frames.pdf

# Проверить трансформы вручную
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

**Ожидаемый результат:**
- `odom → base_footprint`: динамическая трансформация (от одометрии)
- `base_footprint → base_link`: статическая трансформация (rpy=0 0 -1.5708)

### Тестирование движения

```bash
# Послать команду движения вперёд
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Проверить одометрию
ros2 topic echo /odom
```

**Ожидаемое поведение:**
- ✅ Робот движется вперёд физически
- ✅ Одометрия показывает движение вперёд (linear.x положительный)
- ✅ Ориентация робота не меняется (quaternion стабилен)

### Тестирование поворота

```bash
# Послать команду поворота
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Проверить одометрию
ros2 topic echo /odom
```

**Ожидаемое поведение:**
- ✅ Робот поворачивается на месте
- ✅ Одометрия показывает изменение yaw (angular.z)
- ✅ Позиция (x, y) остается примерно на месте

---

## 📖 Связанная документация

- `WHEEL_JOINTS_FIX_2025-11-20.md` - исправление джойнтов колес (PR #234)
- `ROBOT_ORIENTATION_FIX.md` - первоначальное исправление ориентации
- [REP-103: Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP-120: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0120.html)

---

## 🔑 Ключевые выводы

1. **base_footprint vs base_link**: 
   - `base_footprint` — корневой фрейм, всегда без поворота
   - `base_link` — физический фрейм, может быть повернут через joint

2. **Одометрия**: всегда должна публиковаться относительно `base_footprint`
   - Это обеспечивает соответствие с физическим движением робота
   - `base_link` автоматически следует через fixed joint

3. **TF дерево**: `odom → base_footprint → base_link → sensors/wheels`
   - Одометрия управляет `odom → base_footprint`
   - URDF определяет `base_footprint → base_link` (fixed joint)

4. **Согласованность конфигураций**:
   - Все конфиги должны использовать одинаковый `base_frame_id`
   - `robot_controller.yaml`, `vesc_config.yaml`, `controller_manager.yaml`

---

## 🚀 Деплой

### Обновление на роботе

```bash
# Vision Pi (не затронут этим исправлением)
# Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && ./scripts/update_and_restart.sh'
```

### Проверка после деплоя

```bash
# SSH на Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Проверить контейнеры
docker ps

# Проверить логи контроллера
docker logs vesc-nexus -f

# Проверить TF
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

**Автор**: GitHub Copilot  
**Проверено**: Конфигурация валидная, логика TF трансформаций корректна  
**Статус**: Готово к деплою на робота
