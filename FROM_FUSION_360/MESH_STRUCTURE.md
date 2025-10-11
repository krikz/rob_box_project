# Fusion 360 Export - Mesh Files Structure

## 📁 Структура файлов FROM_FUSION_360/meshes/

### ✅ Переименованные файлы (Русский → English)

| Оригинальное название          | Новое название           | Описание                           |
|--------------------------------|--------------------------|------------------------------------|
| `base_link.stl`                | `base_link.stl`          | ✓ Основное шасси (уже английское) |
| `Крышка_1_1.stl`               | `body_cover.stl`         | Крышка корпуса                     |
| `ЛеваяПара.stl`                | `left_assembly.stl`      | Левая сборка                       |
| `ПраваяПара_1.stl`             | `right_assembly.stl`     | Правая сборка                      |
| `Коромысло.stl`                | `rocker_left.stl`        | Левое коромысло (подвеска)         |
| `Коромысло(Mirror).stl`        | `rocker_right.stl`       | Правое коромысло (подвеска)        |
| `Колесо.stl`                   | `wheel_front_left.stl`   | Переднее левое колесо              |
| `Колесо(Mirror).stl`           | `wheel_front_right.stl`  | Переднее правое колесо             |
| `Колесо(Mirror)(Mirror).stl`   | `wheel_rear_left.stl`    | Заднее левое колесо                |
| `Колесо(Mirror) (1).stl`       | `wheel_rear_right.stl`   | Заднее правое колесо               |
| `Зарядка_1.stl`                | `charging_port.stl`      | Разъем зарядки                     |
| `HORN.stl`                     | `horn_speaker.stl`       | Динамик/гудок                      |
| `HEAD_LEAD_1.stl`              | `head_connector.stl`     | Головной разъем/провод             |

### ❓ Неопределенные компоненты (требуется уточнение)

| Файл                | Размер | Возможное назначение           | Действие                    |
|---------------------|--------|--------------------------------|-----------------------------|
| `Component10_1_1.stl` | 14KB   | Небольшой компонент (крепеж?)  | ⚠️ Нужна идентификация      |
| `Component16_1.stl`   | 45KB   | Средний компонент (кронштейн?) | ⚠️ Нужна идентификация      |

**Рекомендация:** Проверьте в Fusion 360 что это за компоненты и переименуйте соответственно.

## 📊 Категории компонентов

### 1. Chassis (Шасси)
- `base_link.stl` - основной корпус (49KB)
- `body_cover.stl` - крышка корпуса (119KB)

### 2. Suspension (Подвеска)
- `rocker_left.stl` - левое коромысло (39KB)
- `rocker_right.stl` - правое коромысло (31KB)

### 3. Wheels (Колеса)
- `wheel_front_left.stl` (157KB)
- `wheel_front_right.stl` (157KB)
- `wheel_rear_left.stl` (157KB)
- `wheel_rear_right.stl` (157KB)

**Примечание:** Все 4 колеса одинакового размера (~157KB), что подтверждает 10" колеса.

### 4. Assemblies (Сборки)
- `left_assembly.stl` - левая сборка (279KB) - возможно полная левая сторона с креплениями
- `right_assembly.stl` - правая сборка (29KB) - значительно меньше, проверить в Fusion 360

### 5. Accessories (Аксессуары)
- `charging_port.stl` - разъем зарядки (67KB)
- `horn_speaker.stl` - динамик (759KB - самый большой файл!)
- `head_connector.stl` - головной разъем (6.8KB)

## 🔧 Следующие шаги

### 1. Идентификация неизвестных компонентов

Откройте Fusion 360 и определите:
```
Component10_1_1.stl → ?
Component16_1.stl   → ?
```

Возможные варианты:
- Кронштейны для сенсоров (lidar, camera)
- Крепления для LED матриц
- Держатели для Raspberry Pi
- Кронштейны для проводки

### 2. Копирование в rob_box_description

После идентификации скопируйте нужные mesh в основной проект:

```bash
# Копируем структурированные mesh файлы
cp FROM_FUSION_360/meshes/base_link.stl src/rob_box_description/meshes/chassis/
cp FROM_FUSION_360/meshes/body_cover.stl src/rob_box_description/meshes/chassis/

cp FROM_FUSION_360/meshes/rocker_*.stl src/rob_box_description/meshes/suspension/

cp FROM_FUSION_360/meshes/wheel_*.stl src/rob_box_description/meshes/wheels/

cp FROM_FUSION_360/meshes/*_assembly.stl src/rob_box_description/meshes/assemblies/

cp FROM_FUSION_360/meshes/charging_port.stl src/rob_box_description/meshes/accessories/
cp FROM_FUSION_360/meshes/horn_speaker.stl src/rob_box_description/meshes/accessories/
cp FROM_FUSION_360/meshes/head_connector.stl src/rob_box_description/meshes/accessories/
```

### 3. Обновление URDF

Обновите пути к mesh файлам в `rob_box_main.xacro`:

```xml
<!-- Пример -->
<visual>
  <geometry>
    <mesh filename="package://rob_box_description/meshes/wheels/wheel_front_left.stl" 
          scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

### 4. Извлечение размеров из Fusion 360 URDF

Fusion 360 экспорт содержит точные параметры инерции и массы в `SIRIUSROVER_DX.xacro`:

```xml
<inertial>
  <origin xyz="-0.18 0.02255492508769995 0.08772092355261586" rpy="0 0 0"/>
  <mass value="7.782560572066453"/>
  <inertia ixx="0.24698" iyy="0.086974" izz="0.295958" 
           ixy="0.0" iyz="-0.000187" ixz="-0.0"/>
</inertial>
```

Эти значения можно использовать для обновления `rob_box_main.xacro`.

## 📝 Чеклист

- [x] Переименовать русские названия файлов
- [x] Структурировать по категориям
- [ ] Идентифицировать Component10_1_1.stl
- [ ] Идентифицировать Component16_1.stl
- [ ] Скопировать mesh в rob_box_description
- [ ] Обновить пути в URDF
- [ ] Извлечь точные размеры wheelbase/track_width
- [ ] Обновить FUSION360_MEASUREMENTS.md

## 🎯 Приоритетные данные для извлечения

Из Fusion 360 URDF нужно извлечь:

1. **Wheelbase и track_width** - расстояния между колесами
2. **Положение сенсоров** - координаты lidar, cameras
3. **Массы компонентов** - для точной физики
4. **Инерции** - для симуляции

Эти данные нужны для обновления:
- `src/rob_box_description/urdf/rob_box_main.xacro`
- `docker/main/config/controllers/controller_manager.yaml`
