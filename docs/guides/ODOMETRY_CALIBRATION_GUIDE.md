# Руководство по калибровке одометрии

**Категория**: Руководства пользователя  
**Дата**: 2025-11-21  
**Статус**: Рекомендовано к выполнению

---

## 📋 Введение

После анализа одометрии VESC Nexus выяснилось, что **код правильный**, но требуется калибровка параметров.

Это руководство описывает процесс калибровки для достижения точной одометрии (< 5% ошибки).

---

## 🎯 Цель

Откалибровать параметры робота для точной одометрии:
- `wheel_radius` - радиус колеса
- `poles` - количество полюсов мотора  
- `wheel_separation` - расстояние между колёсами

---

## 📏 Шаг 1: Измерить wheel_radius

### Что измерять

Реальный радиус колеса **под нагрузкой** (с установленной батареей и компонентами).

### Как измерять

```bash
1. Поставить робота на ровную поверхность
2. Измерить диаметр колеса линейкой/штангенциркулем
3. Учесть деформацию под весом робота
4. Рассчитать: radius = diameter / 2
```

**Текущее значение**: 0.115 м (диаметр 230 мм)

### Где обновить

```yaml
# Файл 1: docker/main/config/vesc_nexus/robot_controller.yaml
diff_drive_controller:
  ros__parameters:
    wheel_radius: 0.115  # ← обновить здесь
```

```xml
<!-- Файл 2: src/rob_box_description/urdf/rob_box_ros2_control.xacro -->
<param name="wheel_radius">0.115</param>  <!-- ← обновить здесь -->
```

⚠️ **Оба значения должны совпадать!**

---

## 🔧 Шаг 2: Проверить poles

### Что проверять

Количество полюсов мотора (магнитных пар × 2).

### Как проверять

```bash
1. Найти datasheet мотора
2. Найти параметр "poles" или "pole pairs"
3. Если указаны pole pairs, умножить на 2
```

**Текущее значение**: 30 (15 пар полюсов)

### Где обновить

```xml
<!-- Файл: src/rob_box_description/urdf/rob_box_ros2_control.xacro -->
<param name="poles">30</param>  <!-- ← обновить если нужно -->
```

---

## 📐 Шаг 3: Измерить wheel_separation

### Что измерять

Расстояние между центрами левого и правого колеса.

### Как измерять

```bash
1. Измерить расстояние между центрами колёс
2. Измерять перпендикулярно оси движения
```

**Текущее значение**: 0.380 м

### Где обновить

```yaml
# Файл: docker/main/config/vesc_nexus/robot_controller.yaml
diff_drive_controller:
  ros__parameters:
    wheel_separation: 0.380  # ← обновить здесь
```

---

## 🧪 Шаг 4: Тестирование

### Тест 1: Движение по прямой

```bash
# 1. Запустить робота
ros2 launch rob_box_bringup robot.launch.py

# 2. Отметить начальную позицию
ros2 topic echo /odom --once | grep "position:"

# 3. Проехать ровно 1 метр (измерить рулеткой!)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --rate 10 &
sleep 5  # 0.2 м/с × 5 сек = 1.0 м
killall ros2

# 4. Проверить одометрию
ros2 topic echo /odom --once | grep "position:"

# 5. Сравнить:
# - Реальное: 1.0 м (по рулетке)
# - Одометрия: должна быть ~1.0 м ± 10%
```

### Если одометрия неправильная

Вычислить корректировку:

```python
# Пример расчёта
actual_distance = 1.0  # метров (измерено рулеткой)
odom_distance = 1.15   # метров (показывает робот)

correction = actual_distance / odom_distance
# = 1.0 / 1.15 = 0.87

# Новый wheel_radius:
new_radius = current_radius * correction
# = 0.115 * 0.87 = 0.100 м
```

### Тест 2: Поворот на месте

```bash
# 1. Отметить начальную ориентацию
ros2 topic echo /odom --once | grep "orientation:"

# 2. Повернуть на 360° (полный круг)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.5}}" --rate 10 &
sleep 12.56  # 2π / 0.5 = 12.56 сек
killall ros2

# 3. Проверить ориентацию
# Должна вернуться к начальной ± 5%
```

### Если поворот неправильный

```python
# Корректировка wheel_separation
actual_rotation = 360  # градусов
odom_rotation = 380    # градусов

correction = actual_rotation / odom_rotation
new_separation = current_separation * correction
```

---

## 🛠️ Использование диагностического инструмента

```bash
# Проверка формул конвертации
python3 tools/odometry_diagnostic.py

# Анализ конкретного ERPM
python3 tools/odometry_diagnostic.py --erpm 1500

# Сравнение с реальным измерением
python3 tools/odometry_diagnostic.py --erpm 1500 --actual-distance 0.05
```

---

## ✅ Критерии успеха

После калибровки:
- ✅ Движение на 1 м: одометрия показывает 0.95-1.05 м (< 5% ошибки)
- ✅ Поворот на 360°: возврат в начальную ориентацию ± 5°
- ✅ Карта строится без искажений
- ✅ Навигация работает точно

---

## 🔄 Следующие шаги

Если после калибровки одометрия всё ещё неточная (> 10% ошибки):

→ Рассмотреть переход на **RPM control** вместо duty cycle  
→ См. документацию в `docs/reports/VESC_ODOMETRY_ANALYSIS.md`

---

## 📚 Связанные документы

- [Технический анализ](../reports/VESC_ODOMETRY_ANALYSIS.md) - как работает одометрия
- [Итоговое резюме](../reports/ODOMETRY_INVESTIGATION_SUMMARY.md) - краткие выводы
- [Диагностический инструмент](../../tools/odometry_diagnostic.py) - проверка формул

---

**Автор**: GitHub Copilot  
**Дата**: 2025-11-21  
**Версия**: 1.0
