# Анализ проблемы одометрии VESC Nexus

**Дата**: 2025-11-20  
**Проблема**: При незначительном вращении колеса одометрия сильно меняется на карте  
**Статус**: 🔍 АНАЛИЗ ЗАВЕРШЁН, НАЙДЕНЫ КРИТИЧЕСКИЕ ОШИБКИ

---

## 🔍 Описание проблемы

При тестировании робота обнаружено:
- ✅ Колесо поворачивается на небольшой угол (визуально)
- ❌ Одометрия показывает значительное перемещение на карте
- ❌ Перемещение не соответствует реальному вращению колеса

Это указывает на **неправильный расчёт одометрии** из данных VESC регуляторов.

---

## 📊 Текущая архитектура управления

### Цепочка обработки команд

```
ROS 2 Navigation/Teleop
    ↓ /cmd_vel (Twist: linear.x м/с, angular.z рад/с)
    ↓
diff_drive_controller
    ↓ (вычисляет скорости для левых/правых колёс)
    ↓ command_interface velocity (м/с)
    ↓
VescSystemHardwareInterface::write()
    ↓ linear_speed = cmd_velocity * wheel_radius  ❌ ОШИБКА!
    ↓
VescHandler::sendSpeed(linear_speed м/с)
    ↓ duty_cycle = linear_speed / 1.0  ❌ ХАРДКОД!
    ↓
VESC (CAN) ← COMM_SET_DUTY_CYCLE
```

### Обратная связь (одометрия)

```
VESC (CAN) → COMM_GET_VALUES_SELECTIVE
    ↓ speed_rpm (механические обороты колеса)
    ↓
VescHandler::processCanFrame()
    ↓ сохраняет state.speed_rpm
    ↓
VescSystemHardwareInterface::read()
    ↓ hw_velocities_[i] = rpm * (2π / 60)  ❌ ОШИБКА!
    ↓ (конвертирует в рад/с вместо м/с!)
    ↓
diff_drive_controller
    ↓ вычисляет одометрию из hw_velocities
    ↓
/odom (nav_msgs/Odometry) ❌ НЕПРАВИЛЬНАЯ!
```

---

## 🐛 Найденные ошибки

### Ошибка #1: Неправильная конвертация RPM → скорость в read()

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp:146`

**Текущий код**:
```cpp
hw_velocities_[i] = rpm * (2.0 * M_PI / 60.0);  // RPM → rad/s
```

**Проблема**: 
- Код конвертирует RPM в **угловую скорость** (радианы/сек)
- `diff_drive_controller` ожидает **линейную скорость** (м/с)

**Правильная формула**:
```cpp
// RPM → линейная скорость (м/с)
// v = ω × r = (RPM × 2π / 60) × radius
hw_velocities_[i] = rpm * (2.0 * M_PI / 60.0) * wheel_radius_;
```

**Эффект ошибки**:
- При `wheel_radius = 0.115 м`:
  - RPM = 100 → текущий код выдаёт `10.47 рад/с` вместо `1.20 м/с`
  - **Ошибка в ~8.7 раз!** (10.47 / 1.20 ≈ 8.7)

---

### Ошибка #2: Неправильная конвертация команды в write()

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp:185`

**Текущий код**:
```cpp
double linear_speed = cmd_velocities_[i] * wheel_radius_;  // rad/s → m/s
```

**Проблема**:
- Комментарий говорит `rad/s → m/s`, но это неверно!
- `cmd_velocities_[i]` уже в **м/с** от `diff_drive_controller`
- Умножение на `wheel_radius` даёт **неправильное значение**

**Эффект ошибки**:
- Команда `1.0 м/с` → конвертируется в `1.0 × 0.115 = 0.115` м/с
- Робот движется медленнее в **8.7 раз**!

**Правильный код**:
```cpp
double linear_speed = cmd_velocities_[i];  // уже в м/с
```

---

### Ошибка #3: Хардкоженный max_linear_speed в sendSpeed()

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_handler.cpp:103`

**Текущий код**:
```cpp
const double max_linear_speed = 1.0;  // м/с, настраиваемое значение
double duty_cycle = linear_speed / max_linear_speed;
```

**Проблема**:
- Значение `1.0 м/с` **захардкожено** в коде
- Duty cycle = 100% не означает скорость 1.0 м/с!
- Реальная максимальная скорость зависит от:
  - Напряжения батареи (22-42V для 6S LiPo)
  - KV мотора
  - Редуктора (если есть)
  - Нагрузки
  - Радиуса колеса

**Последствия**:
- Нет обратной связи по скорости
- Одометрия не соответствует реальному движению
- Невозможно точное позиционирование

**Правильное решение**:
1. Использовать RPM control вместо duty cycle
2. Или калибровать соотношение duty cycle → реальная скорость

---

## 📐 Математика расчёта одометрии

### Правильная формула конвертации

**От VESC получаем**:
- `speed_rpm` - механические обороты колеса (RPM)

**Конвертация в линейную скорость**:
```
Угловая скорость (рад/с): ω = RPM × 2π / 60

Линейная скорость (м/с):  v = ω × r = (RPM × 2π / 60) × wheel_radius

Упрощённо:
v = RPM × wheel_radius × 0.10472
где 0.10472 = 2π / 60
```

**Перемещение за период Δt**:
```
distance = velocity × Δt
```

### Пример расчёта

**Параметры**:
- `wheel_radius = 0.115 м` (диаметр 230 мм)
- `RPM = 100` (от VESC)
- `Δt = 0.02 сек` (50 Hz)

**Правильный расчёт**:
```
ω = 100 × (2π / 60) = 10.47 рад/с
v = 10.47 × 0.115 = 1.204 м/с
distance = 1.204 × 0.02 = 0.024 м = 24 мм
```

**Текущий (неправильный) расчёт**:
```
v = 100 × (2π / 60) = 10.47 рад/с  ← интерпретируется как м/с!
distance = 10.47 × 0.02 = 0.209 м = 209 мм  ← В 8.7 РАЗ БОЛЬШЕ!
```

---

## 🔧 Исправления

### Исправление #1: Конвертация RPM в м/с

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`

**Строка 146**:
```diff
-hw_velocities_[i] = rpm * (2.0 * M_PI / 60.0);  // RPM → rad/s
+// RPM → линейная скорость (м/с)
+// v = ω × r = (RPM × 2π / 60) × wheel_radius
+double angular_velocity = rpm * (2.0 * M_PI / 60.0);  // RPM → рад/с
+double wheel_radius = vesc_handlers_[i]->getWheelRadius();
+hw_velocities_[i] = angular_velocity * wheel_radius;  // рад/с → м/с
```

### Исправление #2: Убрать лишнее умножение в write()

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`

**Строка 185**:
```diff
-double linear_speed = cmd_velocities_[i] * wheel_radius_;  // rad/s → m/s
+// cmd_velocities_[i] уже в м/с от diff_drive_controller
+double linear_speed = cmd_velocities_[i];  // м/с
```

### Исправление #3: Добавить геттер wheel_radius в VescHandler

**Файл**: `src/vesc_nexus/src/vesc_nexus/include/vesc_nexus/vesc_handler.hpp`

Добавить публичный метод:
```cpp
double getWheelRadius() const { return wheel_radius_; }
```

---

## ✅ Проверка исправлений

### Тестовый сценарий

1. **Запустить робота с исправлениями**
2. **Подать команду**: `linear.x = 0.5 м/с`
3. **Измерить**:
   - Время движения: 10 секунд
   - Реальное перемещение: линейкой/рулеткой
4. **Сравнить**:
   - Ожидаемое: `0.5 м/с × 10 сек = 5.0 м`
   - Одометрия: `/odom` должна показать ~5.0 м
   - Реальное: должно быть ~5.0 м ± 5%

### Диагностический инструмент

Использовать скрипт `tools/odometry_diagnostic.py`:

```bash
# Анализ конкретного RPM
python3 tools/odometry_diagnostic.py --erpm 1500 --wheel-radius 0.115 --wheel-poles 30

# Сравнение с реальным измерением
python3 tools/odometry_diagnostic.py --erpm 1500 --actual-distance 0.05
```

---

## 🎯 Рекомендации

### Краткосрочные (КРИТИЧНО)

1. ✅ **Исправить конвертацию RPM → м/с** в `read()`
2. ✅ **Убрать умножение на wheel_radius** в `write()`
3. ✅ **Добавить геттер wheel_radius** в VescHandler
4. ✅ **Протестировать** с диагностическим скриптом

### Долгосрочные (РЕКОМЕНДУЕТСЯ)

1. **Переключиться на RPM control вместо duty cycle**:
   - Позволит VESC поддерживать заданную скорость
   - Даст точную обратную связь
   - Уменьшит проскальзывание

2. **Калибровать параметры**:
   - Измерить точный `wheel_radius` (линейкой)
   - Проверить `wheel_poles` в спецификации мотора
   - Калибровать `wheel_separation` для поворотов

3. **Добавить фильтрацию одометрии**:
   - Kalman filter для объединения с IMU
   - Детекция проскальзывания
   - Fusion с visual odometry (OAK-D)

4. **Мониторинг**:
   - Логировать расхождения odometry vs visual
   - Алерты при больших ошибках
   - Графики в Grafana

---

## 📖 Связанные документы

- `docs/architecture/HARDWARE.md` - параметры колёс и моторов
- `tools/odometry_diagnostic.py` - диагностический скрипт
- `docker/main/config/vesc_nexus/vesc_config.yaml` - конфигурация VESC
- `src/rob_box_description/urdf/rob_box_ros2_control.xacro` - параметры ros2_control

---

## 🔑 Ключевые выводы

1. **Две критические ошибки** в конвертации единиц измерения:
   - В `read()`: RPM → рад/с вместо м/с (ошибка в 8.7 раз)
   - В `write()`: лишнее умножение на wheel_radius

2. **Duty cycle control** не подходит для точной одометрии:
   - Нет обратной связи по скорости
   - Зависит от батареи, нагрузки, поверхности
   - Рекомендуется переход на RPM control

3. **Исправления простые**, но требуют пересборки контейнера:
   - Изменения в vesc_nexus субмодуле
   - Rebuild образа `ros2-control`
   - Deployment на робота

4. **Калибровка критична**:
   - Точный wheel_radius (измерить!)
   - Правильный wheel_poles (из datasheet мотора)
   - Тестирование на реальном роботе

---

**Автор**: GitHub Copilot  
**Проверено**: Код vesc_nexus проанализирован, математика валидна  
**Статус**: Готово к исправлению
