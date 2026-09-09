# Quick Testing Guide - Robot Orientation Fix

## 🚀 Быстрое тестирование

### 1. Проверка в RViz (локально)

```bash
# Запустить визуализацию
ros2 launch rob_box_bringup display.launch.py

# Или упрощённый вариант
ros2 launch rob_box_bringup display_simple.launch.py
```

**Что проверить:**
- ✅ Робот ориентирован правильно (ось X красная - вперёд)
- ✅ Колёса в 4 углах, не в одном месте
- ✅ Камера смотрит вперёд
- ✅ Лидар наверху спереди

### 2. Проверка TF дерева

```bash
# Сгенерировать граф TF
ros2 run tf2_tools view_frames

# Посмотреть результат (создаст frames.pdf)
xdg-open frames.pdf

# Или посмотреть live в терминале
ros2 run tf2_ros tf2_echo base_footprint base_link
```

**Ожидаемый результат:**
```
Translation: [0.000, 0.000, 0.115]
Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
         in RPY (radian) [0.000, 0.000, -1.571]
         in RPY (degree) [0.000, 0.000, -90.000]
```

### 3. Проверка joint_states

```bash
# Убедиться что публикуются правильные имена
ros2 topic echo /joint_states
```

**Ожидаемые имена:**
```yaml
name:
- front_left_wheel_joint
- rear_left_wheel_joint
- front_right_wheel_joint
- rear_right_wheel_joint
```

### 4. Проверка на Main Pi

```bash
# SSH на Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Запустить систему управления
cd ~/rob_box_project/docker/main
docker-compose up -d

# Проверить логи
docker logs robot_controller -f
```

### 5. Проверка на Vision Pi

```bash
# SSH на Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Проверить камеру
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw

# Проверить лидар
ros2 topic hz /scan
```

## ❌ Возможные проблемы

### Колёса всё ещё в одном месте
**Причина:** joint_state_publisher не запущен  
**Решение:**
```bash
# Проверить что dummy_joint_state_publisher работает
ros2 node list | grep dummy
ros2 topic hz /joint_states
```

### Робот всё ещё повёрнут
**Причина:** Используется старый URDF из кэша  
**Решение:**
```bash
# Перезапустить robot_state_publisher
ros2 launch rob_box_bringup display.launch.py
```

### Меши колёс не найдены
**Причина:** Отсутствуют новые STL файлы  
**Решение:**
```bash
# Проверить наличие мешей
ls -la src/rob_box_description/meshes/*_wheel.stl

# Должно быть:
# front_left_wheel.stl
# rear_left_wheel.stl
# front_right_wheel.stl
# rear_right_wheel.stl
```

## 📊 Ожидаемые метрики

### TF Transforms
- `base_footprint → base_link`: rotation Z = -90°, translation Z = 0.115m
- Все остальные transforms относительно base_link без изменений

### Joint States
- Частота публикации: ~50 Hz
- 4 джойнта: front_left, rear_left, front_right, rear_right
- Позиции вращаются при движении робота

### RViz Display
- Fixed Frame: `base_footprint` или `base_link`
- Робот корректно ориентирован в мировой системе координат
- TF frames видны и корректны

## 🎯 Критерии успеха

- [ ] Робот ориентирован X вперёд (красная ось)
- [ ] Колёса в 4 углах (не в центре)
- [ ] Все joint_states публикуются с правильными именами
- [ ] TF дерево построено корректно
- [ ] Моторы отзываются на команды (если тестируем с железом)
- [ ] Камера и лидар публикуют данные в правильных фреймах
