# Research: 4-колёсный скид-стир — проблемы поворота и переход на RPM-контроль

**Дата**: 2026-02-28  
**Статус**: Завершён  
**Область**: vesc_nexus, diff_drive_controller, ros2_control

---

## 1. Текущая конфигурация Rob Box

### Тип привода
4-колёсный **скид-стир** (differential drive), управляемый через `diff_drive_controller/DiffDriveController`.  
Левая сторона (front + rear) = одна группа, правая — вторая.

### Колёса и моторы

| Колесо | Joint | CAN ID | Реальный max_rps |
|--------|-------|--------|-----------------|
| Left Front | `left_front_wheel_joint` | 49 | 7.22 |
| Left Rear | `left_rear_wheel_joint` | 81 | 7.10 |
| Right Front | `right_front_wheel_joint` | 94 | 6.64 |
| Right Rear | `right_rear_wheel_joint` | 124 | 7.16 |

- **Радиус колеса**: 0.1143 м (9" с редуктором, замер 2026-02-23)
- **Wheel separation**: 0.390 м (controller_manager.yaml)
- **Масса**: ~62 кг (база) + 4×21.68 кг (колёса)
- **Моторы**: 30-полюсные (15 пар полюсов), 4× VESC 4.12 через CAN bus

### Текущий режим управления: Duty Cycle (открытый контур)

**Цепочка**: `cmd_vel` → `diff_drive_controller` → `rad/s` на каждое колесо → `VescHandler::sendSpeed()` → **duty cycle** через CAN.

```
linear_speed = cmd_velocities[i] × wheel_radius      // rad/s → m/s
target_duty = linear_speed / max_speed_mps            // m/s → duty [0,1]
scaled_duty = min_duty + |target_duty| × (1 - min_duty)  // deadzone compensation
→ createSetDutyCycleFrame(can_id, duty)               // CAN_PACKET_SET_DUTY = 0
```

---

## 2. Обнаруженные проблемы

### 2.1 Duty Cycle — открытый контур по скорости

**Суть**: VESC получает duty cycle (% напряжения), а не целевую скорость. Несмотря на `open_loop: false` в конфиге контроллера, реальное управление **открытое** — если колесо встречает сопротивление, оно замедляется без компенсации.

**Влияние на повороты**: При скид-стире внутренние колёса испытывают бóльшее сопротивление (трение + scrubbing), замедляются. Внешние — разгружены, крутятся быстрее. Робот поворачивает непредсказуемо.

**Файлы**: `src/vesc_nexus/src/vesc_nexus/src/vesc_handler.cpp` (sendSpeed, L120-186)

### 2.2 Разброс max_rps между колёсами

Все 4 колеса используют `max_rps = 6.5`, но реальные значения: 6.64–7.22. При одинаковом duty, колесо с max_rps=7.22 крутится на ~8.7% быстрее, чем 6.64. Вносит **систематическую асимметрию** — робот уводит в сторону при езде прямо.

**Файлы**: `src/rob_box_description/urdf/rob_box_ros2_control.xacro` (max_rps параметры)

### 2.3 Tire Scrubbing при повороте на месте

У 4-колёсного скид-стира оба колеса одной стороны получают одинаковую скорость, но передние и задние на разных осях. При повороте шины скребутся по поверхности → износ, высокий ток, непредсказуемость.

**Решение из индустрии**: `wheel_separation_multiplier` > 1.0 (типично 1.1–1.5 для 4WD). Clearpath Husky использует `wheels_per_side: 1` хотя колёс 2, т.к. оба управляются одним сигналом.

### 2.4 `wheel_separation_multiplier` не используется

Параметр `diff_drive_controller` закомментирован в `robot_controller.yaml`. Эффективное расстояние при повороте у скид-стира **больше** физического из-за scrubbing.

**Файлы**: `docker/main/config/controllers/controller_manager.yaml`, `docker/main/config/vesc_nexus/robot_controller.yaml`

### 2.5 Deadzone (`min_duty = 0.04`) создаёт рывок

При малейшем ненулевом сигнале duty скачком прыгает до 4%. Для 62 кг ровера это рывок при медленных маневрах. При RPM-контроле проблема исчезает (VESC PID сам плавно разгоняет).

### 2.6 Неконсистентные конфиги

| Параметр | controller_manager.yaml | robot_controller.yaml |
|---|---|---|
| `wheel_separation` | 0.390 | 0.380 |
| `base_frame_id` | base_footprint | base_link |
| `angular.z.max_velocity` | **15.0 rad/s** (860°/s!) | 3.0 rad/s |

| Параметр | xacro | vesc_config.yaml |
|---|---|---|
| `gear_ratio` | 2.46 | 5.0 |
| `min_duty` | 0.04 | 0.08 |

---

## 3. Существующая инфраструктура для RPM-контроля

### CAN-фрейм для ERPM уже реализован

`createSetSpeedFrame(can_id, rpm)` в `message_translator.cpp`:
- Использует `CAN_PACKET_SET_RPM = 3`
- Отправляет ERPM как int32, диапазон [-23250, 23250]
- **НЕ используется** в продакшн коде

### Тестовый инструмент существует

`src/vesc_nexus/src/vesc_nexus/tools/test_velocity_control.py` — отправляет CAN_PACKET_SET_RPM напрямую через python-can. Результаты тестов (из README_VELOCITY_TEST.md):
- Ожидаемая ошибка: < 5% (VESC PID)
- Стабильность: малый range колебаний
- Рекомендация: min_rpm ~30 RPM

---

## 4. План перехода на RPM-контроль

### Конвертация: rad/s → ERPM

```
linear_speed = cmd_velocities[i] × wheel_radius   // rad/s → m/s  (из write())
wheel_rps = linear_speed / (2π × wheel_radius)    // m/s → об/сек колеса
motor_rps = wheel_rps × gear_ratio                 // об/сек мотора
erpm = motor_rps × pole_pairs × 60                 // → ERPM
```

Упрощённо из rad/s:
```
motor_rad_s = cmd_velocities[i] × gear_ratio       // рад/с мотора
erpm = motor_rad_s × pole_pairs × 60 / (2π)        // → ERPM  
erpm = cmd_velocities[i] × gear_ratio × pole_pairs × 60 / (2π)
```

### Что нужно изменить

1. **xacro**: добавить `<param name="control_mode">rpm</param>`
2. **VescHandler**: новый метод `sendSpeedRpm(double linear_speed)` — конвертирует m/s → ERPM → `createSetSpeedFrame()`
3. **HW Interface**: читать `control_mode` параметр, вызывать `sendSpeedRpm()` или `sendSpeed()` в зависимости от режима
4. **controller_manager.yaml**: установить реалистичный `angular.z.max_velocity: 3.0`

### Преимущества

- Замкнутый контур по скорости на каждом колесе (VESC PID)
- Каждое колесо **точно** держит скорость независимо от нагрузки
- Не нужен min_duty (PID сам справляется с deadzone)
- Значительно точнее одометрия при поворотах
- Не зависит от заряда батареи

### Риски

- VESC PID может быть не настроен → осцилляции (нужна калибровка через VESC Tool)
- Диапазон ERPM [-23250, 23250] — при gear_ratio=2.46, pole_pairs=15: max_cmd ≈ 6.5 × 2.46 × 15 × 60 ≈ 14391 ERPM — в пределах лимита
- Обратная совместимость: параметр `control_mode: duty` оставляет старое поведение

---

## 5. Затронутые файлы

| Файл | Изменение |
|------|-----------|
| `src/rob_box_description/urdf/rob_box_ros2_control.xacro` | Добавить `control_mode` параметр |
| `src/vesc_nexus/.../vesc_handler.hpp` | Добавить `sendSpeedRpm()`, `control_mode_`, `setControlMode()` |
| `src/vesc_nexus/.../vesc_handler.cpp` | Реализовать `sendSpeedRpm()` |
| `src/vesc_nexus/.../vesc_system_hardware_interface.cpp` | Читать `control_mode`, вызывать правильный метод |
| `docker/main/config/controllers/controller_manager.yaml` | Уменьшить `angular.z.max_velocity` до 3.0 |

---

**Автор**: GitHub Copilot  
**Следующий шаг**: Реализация (этот же PR)
