# Резюме исправления одометрии VESC Nexus

## 🎯 Задача
Проверить и исправить расчёт одометрии от VESC регуляторов, т.к. при незначительном вращении колеса одометрия сильно меняется на карте.

## 🔍 Найденные проблемы

### Критическая ошибка #1: Неправильная конвертация RPM → скорость
**Файл**: `vesc_system_hardware_interface.cpp:146`

**Было**:
```cpp
hw_velocities_[i] = rpm * (2.0 * M_PI / 60.0);  // RPM → рад/с
```

**Проблема**: Конвертирует в **угловую скорость** (рад/с), а `diff_drive_controller` ожидает **линейную скорость** (м/с).

**Эффект**: Одометрия завышена в **8.7 раз** при `wheel_radius=0.115м`.

**Исправлено**:
```cpp
double angular_velocity = rpm * (2.0 * M_PI / 60.0);  // RPM → рад/с
double wheel_radius = vesc_handlers_[i]->getWheelRadius();
hw_velocities_[i] = angular_velocity * wheel_radius;  // рад/с → м/с
```

### Критическая ошибка #2: Позиция в метрах вместо радиан
**Файл**: `vesc_system_hardware_interface.cpp:147`

**Было**:
```cpp
hw_positions_[i] += hw_velocities_[i] * (1.0 / publish_rate_);
```

**Проблема**: После исправления ошибки #1, `hw_velocities_[i]` стала в м/с, но `hw_positions_[i]` должна быть в **радианах** (угловая позиция колеса), а не в метрах.

**Исправлено**:
```cpp
double period = 1.0 / publish_rate_;  // секунды
hw_positions_[i] += angular_velocity * period;  // радианы
```

### Критическая ошибка #3: Лишнее умножение команды
**Файл**: `vesc_system_hardware_interface.cpp:179`

**Было**:
```cpp
double linear_speed = cmd_velocities_[i] * wheel_radius_;  // rad/s → m/s
```

**Проблема**: `cmd_velocities_[i]` **уже** в м/с от `diff_drive_controller`, лишнее умножение на `wheel_radius` делает робот медленнее в **8.7 раз**.

**Исправлено**:
```cpp
double linear_speed = cmd_velocities_[i];  // уже в м/с
```

### Проблема #4: Управление через duty cycle (не исправлена)
**Файл**: `vesc_handler.cpp:103`

```cpp
const double max_linear_speed = 1.0;  // захардкожено!
double duty_cycle = linear_speed / max_linear_speed;
```

**Проблема**: 
- Нет обратной связи по скорости
- Реальная скорость зависит от батареи, нагрузки, поверхности
- Максимальная скорость не равна 1.0 м/с

**Рекомендация**: Переключиться на RPM control (требует доработки).

## 📊 Результаты

### Математика (пример)
**Параметры**: `wheel_radius = 0.115 м`, `wheel_poles = 30`, `RPM = 100`

**До исправления**:
```
hw_velocities = 100 × (2π / 60) = 10.47 рад/с  ← интерпретируется как м/с!
distance = 10.47 × 0.02 = 0.209 м = 209 мм  ← НЕПРАВИЛЬНО!
```

**После исправления**:
```
ω = 100 × (2π / 60) = 10.47 рад/с
v = 10.47 × 0.115 = 1.204 м/с  ← правильная линейная скорость
distance = 1.204 × 0.02 = 0.024 м = 24 мм  ← ПРАВИЛЬНО!
```

**Разница**: 209 / 24 = **8.7 раз**!

### Формула расчёта
```
ERPM → RPM: mechanical_rpm = erpm / (poles / 2)
RPM → рад/с: angular_velocity = rpm × 2π / 60
рад/с → м/с: linear_velocity = angular_velocity × wheel_radius

Полная формула:
v = (erpm / pole_pairs) / 60 × 2π × r
v = erpm × (2πr / 60 / pole_pairs)
v = erpm × 0.000802857 (при r=0.115м, poles=30)
```

## 📁 Созданные материалы

1. **VESC_ODOMETRY_PROBLEM_ANALYSIS.md** - полный технический анализ
2. **VESC_ODOMETRY_FIX_INSTRUCTIONS.md** - инструкция по применению
3. **vesc_nexus_odometry_fix.patch** - патч с исправлениями
4. **tools/odometry_diagnostic.py** - диагностический инструмент

## 🔧 Применение исправлений

```bash
# 1. Применить патч
cd ~/rob_box_project
git apply vesc_nexus_odometry_fix.patch

# 2. Пересобрать образ
cd docker/main/ros2_control
docker build --build-arg VESC_NEXUS_SHA=$(cd ../../../src/vesc_nexus && git rev-parse HEAD) \
  -t ghcr.io/krikz/rob_box:ros2-control-humble-dev -f Dockerfile ../..

# 3. Деплой
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && docker-compose up -d ros2-control'
```

## ✅ Проверка

```bash
# Тест движения (10 сек × 0.5 м/с = 5 метров)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}}" --rate 10 &
sleep 10 && killall ros2

# Проверить одометрию
ros2 topic echo /odom --once
# Ожидаемое: x ≈ 5.0 м (± 5%)

# Измерить реальное расстояние рулеткой
# Должно совпадать с одометрией!
```

## 🎓 Ключевые выводы

1. **Единицы измерения критичны** - ошибка в 1 символ (рад/с вместо м/с) привела к ошибке в 8.7 раз
2. **diff_drive_controller ожидает**:
   - `state_interface velocity` - линейная скорость в **м/с**
   - `state_interface position` - угловая позиция в **радианах**
   - `command_interface velocity` - линейная скорость в **м/с**
3. **Duty cycle control не подходит** для точной одометрии - нужен переход на RPM control
4. **Калибровка важна** - измерить реальный `wheel_radius` линейкой!

## 📌 Рекомендации

### Краткосрочные (сделать сразу):
1. ✅ Применить патч
2. ✅ Протестировать одометрию
3. ✅ Измерить точный wheel_radius
4. ✅ Обновить конфигурацию

### Долгосрочные (планировать):
1. Переключиться на RPM control вместо duty cycle
2. Добавить fusion одометрии с IMU (Kalman filter)
3. Использовать visual odometry для верификации
4. Мониторинг расхождений в Grafana

---

**Статус**: ✅ ГОТОВО К ПРИМЕНЕНИЮ  
**Дата**: 2025-11-20  
**Автор**: GitHub Copilot
