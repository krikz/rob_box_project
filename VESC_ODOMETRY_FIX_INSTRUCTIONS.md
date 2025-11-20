# Инструкция по исправлению одометрии VESC Nexus

**Дата**: 2025-11-20  
**Проблема**: Одометрия завышена в ~8.7 раз при небольшом вращении колеса  
**Решение**: Патч для vesc_nexus с исправлением конвертации RPM → м/с

---

## 📋 Краткое описание

Найдены **две критические ошибки** в vesc_nexus hardware interface:

1. **В методе `read()`**: RPM конвертировался в рад/с вместо м/с → одометрия завышена в 8.7 раз
2. **В методе `write()`**: Лишнее умножение команды на wheel_radius → робот движется медленнее в 8.7 раз

---

## 🔧 Применение исправлений

### Вариант 1: Автоматическое применение патча (рекомендуется)

```bash
cd ~/rob_box_project

# Применить патч к субмодулю vesc_nexus
git apply vesc_nexus_odometry_fix.patch

# Или, если патч не применяется:
cd src/vesc_nexus
git apply ../../vesc_nexus_odometry_fix.patch
```

### Вариант 2: Ручное исправление

Внести изменения в 3 файла (см. раздел "Детали изменений" ниже).

---

## 📦 Пересборка и деплой

### 1. Пересборка Docker образа

```bash
cd ~/rob_box_project

# Обновить vesc_nexus SHA для инвалидации кэша
VESC_SHA=$(cd src/vesc_nexus && git rev-parse HEAD)

# Пересобрать ros2-control образ
cd docker/main/ros2_control
docker build \
  --build-arg VESC_NEXUS_SHA=$VESC_SHA \
  --build-arg BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap \
  -t ghcr.io/krikz/rob_box:ros2-control-humble-dev \
  -f Dockerfile \
  ../..  # build context = корень проекта
```

### 2. Деплой на робота

```bash
# Push образ в registry (если используется)
docker push ghcr.io/krikz/rob_box:ros2-control-humble-dev

# Обновить на Main Pi через SSH
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && \
   docker-compose pull ros2-control && \
   docker-compose up -d ros2-control'
```

### 3. Проверка

```bash
# SSH на Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Проверить логи
docker logs ros2-control -f

# Проверить контроллеры
ros2 control list_controllers

# Проверить одометрию
ros2 topic echo /odom --once
```

---

## 🧪 Тестирование

### Тест 1: Проверка одометрии в статике

```bash
# Запустить робота
ros2 launch rob_box_bringup robot.launch.py

# В другом терминале - слушать одометрию
ros2 topic echo /odom

# Вручную покрутить колесо на 1 полный оборот
# Ожидаемое перемещение: 2πr = 2 × 3.14159 × 0.115 = 0.722 м
```

### Тест 2: Движение вперёд

```bash
# Команда: 0.5 м/с на 10 секунд
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" \
  --rate 10 &

# Подождать 10 секунд
sleep 10

# Остановить публикацию
killall ros2

# Проверить одометрию
ros2 topic echo /odom --once

# Ожидаемое перемещение: 0.5 м/с × 10 сек = 5.0 м (± 5%)
# Реальное перемещение: измерить рулеткой
```

### Тест 3: Сравнение с визуальной одометрией

```bash
# Запустить RTAB-Map с visual odometry
ros2 launch rob_box_bringup mapping.launch.py

# Сравнить одометрии
ros2 run tf2_ros tf2_echo odom base_link  # От diff_drive
ros2 topic echo /rtabmap/odom --once      # От visual odometry

# Расхождение должно быть < 10% при движении по прямой
```

---

## 📊 Детали изменений

### Изменение 1: Добавить геттер wheel_radius

**Файл**: `src/vesc_nexus/src/vesc_nexus/include/vesc_nexus/vesc_handler.hpp`

```cpp
// После строки 33
double getWheelRadius() const;
```

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_handler.cpp`

```cpp
// После метода getLastState()
double VescHandler::getWheelRadius() const {
    return wheel_radius_;
}
```

### Изменение 2: Исправить конвертацию в read()

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`

**Было (строки 146-147)**:
```cpp
hw_velocities_[i] = rpm * (2.0 * M_PI / 60.0);  // RPM → rad/s
hw_positions_[i] += hw_velocities_[i] * (1.0 / publish_rate_);
```

**Стало**:
```cpp
// ИСПРАВЛЕНИЕ: Конвертация RPM → линейная скорость (м/с)
// Формула: v = ω × r = (RPM × 2π / 60) × wheel_radius
double angular_velocity = rpm * (2.0 * M_PI / 60.0);  // RPM → рад/с
double wheel_radius = vesc_handlers_[i]->getWheelRadius();
hw_velocities_[i] = angular_velocity * wheel_radius;  // рад/с → м/с

// ИСПРАВЛЕНИЕ: hw_positions должна быть в радианах (угловая позиция колеса)
// Интегрируем угловую скорость, а не линейную
double period = 1.0 / publish_rate_;  // секунды
hw_positions_[i] += angular_velocity * period;  // радианы
```

### Изменение 3: Убрать лишнее умножение в write()

**Файл**: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`

**Было (строка 179)**:
```cpp
double linear_speed = cmd_velocities_[i] * wheel_radius_;  // rad/s → m/s
```

**Стало**:
```cpp
// ИСПРАВЛЕНИЕ: cmd_velocities_[i] уже в м/с от diff_drive_controller
// НЕ нужно умножать на wheel_radius!
double linear_speed = cmd_velocities_[i];  // м/с
```

**Также обновить комментарий (строка 182)**:
```cpp
if (std::abs(cmd_velocities_[i]) > 0.001) {  // Порог ~0.001 м/с (игнорируем шум)
```

---

## 📈 Ожидаемый результат

### До исправления:
- ❌ Колесо поворачивается на 1 см → одометрия показывает 8.7 см
- ❌ Команда 1.0 м/с → робот движется со скоростью ~0.115 м/с
- ❌ Невозможно точное позиционирование
- ❌ Карта строится с искажениями

### После исправления:
- ✅ Колесо поворачивается на 1 см → одометрия показывает ~1 см (± 2%)
- ✅ Команда 1.0 м/с → робот движется со скоростью ~1.0 м/с (± 5%)
- ✅ Точное позиционирование возможно
- ✅ Карта строится правильно

---

## ⚠️ Важные замечания

### 1. Калибровка параметров после исправления

После применения патча **рекомендуется** откалибровать:

```yaml
# docker/main/config/vesc_nexus/robot_controller.yaml
wheel_radius: 0.115      # Измерить линейкой!
wheel_separation: 0.380  # Измерить между центрами колёс

# src/rob_box_description/urdf/rob_box_ros2_control.xacro
wheel_radius: 0.115      # Должно совпадать с robot_controller.yaml
poles: 30                # Проверить в datasheet мотора
```

**Как измерить wheel_radius**:
1. Измерить диаметр колеса линейкой (с грузом на роботе!)
2. `wheel_radius = diameter / 2`
3. Пример: диаметр 230 мм → radius = 0.115 м

### 2. Режим управления (duty cycle vs RPM)

Текущая реализация использует **duty cycle control**:
- ✅ Простота управления
- ❌ Нет обратной связи по скорости
- ❌ Зависит от батареи, нагрузки, поверхности

**Рекомендация**: В будущем переключиться на **RPM control** для точной одометрии.

### 3. Проверка после деплоя

Всегда проверяйте:
```bash
# Логи при запуске
docker logs ros2-control -f | grep -i "error\|warning\|vesc"

# CAN статус
ros2 topic echo /diagnostics | grep -i can

# Одометрия публикуется
ros2 topic hz /odom  # Должно быть ~50 Hz
```

---

## 🐛 Troubleshooting

### Проблема: Патч не применяется

```bash
# Проверить версию vesc_nexus
cd src/vesc_nexus
git log --oneline -5

# Если версия не та, обновить субмодуль
git checkout release/v1.0.0
git pull origin release/v1.0.0
```

### Проблема: Образ не собирается

```bash
# Проверить логи сборки
docker build --no-cache --progress=plain ...

# Проверить что vesc_nexus исходники скопировались
docker run -it <image-id> bash
ls -la /ws/src/vesc_nexus
```

### Проблема: Одометрия всё ещё неправильная

```bash
# Проверить параметры
ros2 param list /controller_manager
ros2 param get /controller_manager diff_drive_controller.wheel_radius

# Проверить TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Использовать диагностический инструмент
python3 tools/odometry_diagnostic.py --erpm 1000
```

---

## 📖 Связанные документы

- `VESC_ODOMETRY_PROBLEM_ANALYSIS.md` - детальный анализ проблемы
- `tools/odometry_diagnostic.py` - диагностический скрипт
- `vesc_nexus_odometry_fix.patch` - патч с изменениями
- `docs/architecture/HARDWARE.md` - параметры колёс

---

**Автор**: GitHub Copilot  
**Проверено**: Патч протестирован на vesc_nexus release/v1.0.0  
**Статус**: Готов к применению на роботе
