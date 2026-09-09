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

## 6. Результаты реального тестирования RPM-режима (2026-02-28)

### 6.1 Реализованные изменения

Все изменения из плана (секция 4) были реализованы и задеплоены на робота:
- `control_mode=rpm` в xacro
- `sendSpeedRpm()` в VescHandler (m/s → ERPM конвертация)
- `sendCommand()` диспетчер в HW interface
- `angular.z.max_velocity: 3.0` в controller_manager.yaml

**Коммит vesc_nexus**: `fc655f0` на ветке `feat/rpm-control-mode`  
**Коммит main repo**: `dbb5598` на ветке `feature/agent-skills`

### 6.2 Инициализация — ОК

Логи подтверждают корректную инициализацию:
```
Control mode: RPM (closed-loop velocity via VESC PID)
Gear ratio configured: 2.5
[left_front_wheel_joint] can_id=49, max_rps=6.50, gear_ratio=2.5, max_speed=1.90 m/s, min_duty=0.040, mode=RPM
```

Все 4 колеса видны, CAN-интерфейс `can0` в состоянии `ERROR-ACTIVE` (нормально).

### 6.3 Проблема Zenoh маршрутизации (косметическая)

При тестировании обнаружена проблема маршрутизации cmd_vel:
- Основной процесс (`ros2_control_node`) использует `ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5`
- CLI-команды `ros2 topic pub` используют другой конфиг `/config/zenoh_session_config.json5`
- Из-за этого CLI-сессии не находили подписчиков контроллера

**Решение**: При работе внутри ros2-control контейнера необходимо использовать тот же конфиг:
```bash
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5
```

### 6.4 Тест движения: RPM режим работает, но с проблемами

Отправлена команда 0.1 м/с через `/diff_drive_controller/cmd_vel_unstamped`:

#### Выход (команды ERPM):
```
speed=0.0200 m/s → ERPM=62   (разгон, шаг 1)
speed=0.0400 m/s → ERPM=123  (разгон, шаг 2)
speed=0.0600 m/s → ERPM=185  (разгон, шаг 3)
speed=0.0800 m/s → ERPM=247  (разгон, шаг 4)
speed=0.1000 m/s → ERPM=308  (целевая скорость)
```

diff_drive_controller плавно разгоняет/замедляет через velocity ramp.

#### Обратная связь (реальные ERPM с left_front_wheel_joint):
```
Целевой: 308 ERPM
Реальный: 35 → 454 → 478 → 348 → 232 → 425 → -22 → 249 → 108 → 368
```

**ERPM скачет от -22 до 478 при целевых 308!** Хаотичные осцилляции ±200% от цели.

#### Наблюдения:
- Робот **поехал**, но дальше, чем нужно (10 секунд на 0.1 м/с ≈ 80 см, а не 10 см)
- Моторы **дребездят** (rattling sound) при движении
- После остановки один мотор показал ERPM=-1618 (PID overshoot при торможении)

### 6.5 Корневая причина дребезга: Sensorless FOC на низких ERPM

#### Исследование исходного кода VESC firmware

Функция `foc_run_pid_control_speed()` в `motor/foc_math.c` (bldc firmware):

```c
// Если целевой ERPM ниже порога — полностью отпустить мотор
if (fabsf(motor->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm) {
    motor->m_speed_i_term = 0.0;
    motor->m_iq_set = 0.0;  // ток = 0
    return;
}
```

**Ключевые параметры VESC Speed PID:**

| Параметр | Что делает |
|---|---|
| `s_pid_kp` | Пропорциональный коэффициент |
| `s_pid_ki` | Интегральный коэффициент |
| `s_pid_kd` | Дифференциальный коэффициент |
| `s_pid_kd_filter` | Фильтр D-компоненты |
| `s_pid_min_erpm` | Минимальный ERPM (ниже → мотор отпускается) |
| `s_pid_allow_braking` | Разрешить торможение через PID |
| `s_pid_ramp_erpms_s` | Скорость нарастания целевого ERPM |
| `s_pid_speed_source` | Источник: PLL (фильтр), Fast, Faster |

#### Vedder (автор VESC) о низких скоростях (GitHub Issue #139):

> "Very low speed operation (depending on how low) will always be a problem with hall sensors, even if they are perfectly spaced and/or perfectly calibrated. The reason is that there are no updates between the discrete 60 degree position updates. If you are operating on too low speeds there is no other choice than using some sort of encoder with higher resolution."

Для **sensorless** (наш случай) ситуация **ещё хуже**: observer теряет точность фазы при ERPM < ~500-1000, что делает PID-контроль нестабильным.

#### Issue #864 — аналогичный кейс (дифференциальный робот):

Пользователь: "I have a differential drive robot and don't want to go down the slope when speed is zero."  
Вывод: VESC speed mode при нулевой скорости **полностью отпускает мотор** (нет holding torque), в отличие от ODrive.

#### Issue #640 — PID использует фильтрованную скорость:

Speed PID по умолчанию использует `m_pll_speed` (фильтрованный с лагом). Vedder добавил опцию `s_pid_speed_source` для выбора `m_speed_est_fast` (меньше лаг, лучше PID).

#### Issue #884 — флуктуации скорости при любых оборотах:

Даже при средних скоростях наблюдаются колебания. Предлагается lowpass filter на выход PID.

### 6.6 Рекомендации по настройке VESC для RPM-режима

#### Через VESC Tool (требуется USB-подключение к каждому ESC):

1. **Speed PID Kp/Ki** — уменьшить от дефолтных значений для меньшего overshoot
2. **s_pid_min_erpm** — установить 500-1000 (чтобы VESC не пытался работать на недопустимо низких оборотах)
3. **s_pid_speed_source** — попробовать "Fast" вместо PLL для меньшего лага
4. **s_pid_allow_braking** — включить для робота (нужно торможение)
5. **FOC switching frequency** — увеличить до 25-30 кГц (лучше для низких скоростей)
6. **Current controller time constant** — попробовать 600-800µs (от дефолтных 1000µs, по рекомендации из Issue #139)
7. **Observer gain** — попробовать 2/3 от расчётного (по рекомендации Aquaharmonics)

#### Программные решения (без VESC Tool):

1. **Min ERPM порог в коде** — не отправлять |ERPM| < 500, использовать deadzone в `sendSpeedRpm()`
2. **Гибридный режим** — duty cycle для < 0.3 м/с, RPM для > 0.3 м/с
3. **Увеличить минимальную скорость** робота — не посылать cmd_vel < 0.2 м/с

#### Аппаратные решения:

1. **Hall sensors** — добавить датчики Холла на моторы (лучше для низких скоростей)
2. **Энкодеры** — AS5047P на каждый мотор (идеальная точность)
3. **HFI (High Frequency Injection)** — sensorless метод для низких скоростей (зависит от FW версии и мощности мотора)

---

## 7. Телеоп и маршрутизация топиков

### 7.1 Проверка спама нулями

Проверены все cmd_vel топики — **телеоп НЕ спамит**:
- `/cmd_vel_joy` — нет публикаций
- `/cmd_vel_web` — нет публикаций
- `/cmd_vel` (выход twist-mux) — нет публикаций

### 7.2 Маршрутизация cmd_vel

Топология (подтверждено `ros2 topic info -v`):
```
twist_mux (publisher) → /diff_drive_controller/cmd_vel_unstamped → diff_drive_controller (subscriber)
```

twist-mux приоритеты (из конфига):
- emergency (255) > joystick (100) > web (50) > voice (25) > nav2 (10)

Входные топики: `/cmd_vel_emergency`, `/cmd_vel_joy`, `/cmd_vel_web`, `/cmd_vel_voice`, `/cmd_vel`  
Выходной топик: `/diff_drive_controller/cmd_vel_unstamped`

---

**Автор**: GitHub Copilot  
**Статус**: Задокументированы находки первого реального тестирования RPM-режима  
**Следующий шаг**: Настройка VESC PID через VESC Tool + программный min ERPM порог
