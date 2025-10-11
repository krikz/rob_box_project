# Rob Box - Геометрические параметры из Fusion 360 STL Analysis

**Дата анализа:** 2025-10-11  
**Источник:** FROM_FUSION_360/meshes/*.stl  
**Метод:** Python script с numpy-stl

---

## 📐 Базовые размеры робота

### Base Link (основное шасси)
```yaml
size:
  length_x: 0.2300 m  # 230 mm
  width_y:  0.4757 m  # 475.7 mm
  height_z: 0.1550 m  # 155 mm

center_in_fusion:
  x: -0.1800 m
  y:  0.0224 m
  z:  0.0450 m
```

**Для URDF:** Эти размеры соответствуют текущим значениям в `rob_box.xacro`!

---

## 📡 Сенсоры - ТОЧНЫЕ КООРДИНАТЫ

### LSLIDAR N10 (2D LiDAR)

**Положение в Fusion 360:**
```yaml
position:
  x: -0.1803 m  # 180.3 mm назад от origin
  y:  0.1713 m  # 171.3 mm влево
  z:  0.4620 m  # 462 mm вверх

size:
  diameter: 0.0520 m  # 52 mm (соответствует спецификации)
  height:   0.0361 m  # 36.1 mm (соответствует спецификации)
```

**Для URDF (относительно base_link):**
```xml
<origin xyz="-0.1803 0.1713 0.4620" rpy="0 0 0"/>
```

---

### OAK-D-Lite (RGB + Stereo Depth Camera)

**Положение в Fusion 360:**
```yaml
position:
  x: -0.1802 m  # 180.2 mm назад от origin
  y:  0.1076 m  # 107.6 mm влево
  z:  0.4115 m  # 411.5 mm вверх

size:
  length_x: 0.0910 m  # 91 mm (соответствует спецификации 92mm)
  width_y:  0.0175 m  # 17.5 mm (соответствует спецификации 17mm)
  height_z: 0.0280 m  # 28 mm (соответствует спецификации)
```

**Для URDF (относительно base_link):**
```xml
<origin xyz="-0.1802 0.1076 0.4115" rpy="0 0 0"/>
```

---

### Raspberry Pi Camera (Ceiling Navigation)

**Положение в Fusion 360:**
```yaml
position:
  x: -0.1130 m  # 113 mm назад от origin
  y:  0.1712 m  # 171.2 mm влево (рядом с лидаром)
  z:  0.4390 m  # 439 mm вверх

size:
  width:  0.0280 m  # 28 mm
  length: 0.0280 m  # 28 mm
  height: 0.0200 m  # 20 mm
```

**Ориентация:** Смотрит ВВЕРХ (pitch = π/2 радиан)

**Для URDF (относительно base_link):**
```xml
<origin xyz="-0.1130 0.1712 0.4390" rpy="0 1.5708 0"/>
<!-- pitch = 1.5708 rad = 90° - смотрит вверх -->
```

---

## 🛞 Колеса

### Параметры колес

```yaml
radius: 0.115 m  # 115 mm (230 mm diameter = 10 inches)
width:  0.135 m  # Ширина колеса/рычага

# ⚠️ ВНИМАНИЕ: координаты колес в STL некорректные!
# Все 4 колеса имеют одинаковую позицию в экспортированных STL
# Нужно измерить wheelbase и track_width вручную
```

### Что нужно измерить в Fusion 360:

1. **Wheelbase** (расстояние между осями):
   - Измерь расстояние от центра переднего колеса до центра заднего по оси X
   - Текущее значение в URDF: 0.28934 м

2. **Track Width** (ширина колеи):
   - Измерь расстояние от центра левого колеса до центра правого по оси Y
   - Текущее значение в URDF: 0.360 м

### Положение колес (ТРЕБУЕТСЯ УТОЧНЕНИЕ):

```yaml
# На основе rockers (которые правильно экспортировались):
rocker_left_center:
  x:  0.0225 m
  y: -0.0014 m
  z:  0.0007 m

rocker_right_center:
  x:  0.0225 m
  y: -0.1452 m
  z:  0.0036 m

# Из этого можно вычислить track_width:
track_width ≈ 0.1438 m  # Расстояние между рокерами по Y

# ⚠️ Это НЕ совпадает с текущим значением 0.360 м!
# Нужна проверка в Fusion 360
```

---

## 🎯 Дополнительные компоненты

### Head Connector (крепление сенсоров?)
```yaml
position:
  x: -0.1550 m
  y:  0.1303 m
  z:  0.4080 m

size:
  x: 0.0140 m
  y: 0.0040 m
  z: 0.0700 m  # Вертикальный элемент 70mm
```

### Horn Speaker (динамик?)
```yaml
position:
  x: -0.1800 m
  y:  0.1863 m
  z:  0.3080 m

size:
  x: 0.4449 m
  y: 0.1897 m
  z: 0.2700 m
```

### Charging Port
```yaml
position:
  x: -0.0994 m
  y:  0.0226 m
  z: -0.0200 m  # Ниже base_link

size:
  x: 0.1429 m
  y: 0.0295 m
  z: 0.0260 m
```

---

## ✅ Проверка с текущим URDF

### Сравнение с rob_box_main.xacro:

| Параметр | Текущий URDF | STL Analysis | Статус |
|----------|--------------|--------------|--------|
| Base length | 0.4757 m | 0.4757 m | ✅ Совпадает |
| Base width | 0.238 m | 0.230 m | ⚠️ Различие |
| Base height | 0.1885 m | 0.155 m | ⚠️ Различие |
| Wheel radius | 0.115 m | 0.115 m | ✅ Совпадает |
| LSLIDAR position | TODO | x=-0.180, y=0.171, z=0.462 | 📝 Обновить |
| OAK-D position | TODO | x=-0.180, y=0.108, z=0.412 | 📝 Обновить |
| RPi Camera position | TODO | x=-0.113, y=0.171, z=0.439 | 📝 Обновить |

---

## 📝 Действия для обновления URDF

### 1. Обновить координаты сенсоров

Заменить в `src/rob_box_description/urdf/sensors/rob_box_sensors.xacro`:

```xml
<!-- LSLIDAR N10 -->
<xacro:lslidar_n10 parent_link="base_link" 
                   xyz="-0.1803 0.1713 0.4620"  <!-- ← ОБНОВЛЕНО -->
                   rpy="0 0 0"/>

<!-- OAK-D-Lite -->
<xacro:oak_d_lite parent_link="base_link" 
                  xyz="-0.1802 0.1076 0.4115"  <!-- ← ОБНОВЛЕНО -->
                  rpy="0 0 0"/>

<!-- RPi Camera (ceiling) -->
<xacro:rpi_camera_ceiling parent_link="base_link" 
                          xyz="-0.1130 0.1712 0.4390"/>  <!-- ← ОБНОВЛЕНО -->
                          <!-- rpy="0 1.5708 0" уже установлен -->
```

### 2. Уточнить wheelbase и track_width

**Нужно измерить в Fusion 360:**

1. Открой сборку в Fusion 360
2. Используй инструмент "Measure" (M)
3. Измерь:
   - **Wheelbase:** от центра переднего колеса до центра заднего (по X)
   - **Track Width:** от центра левого колеса до центра правого (по Y)

4. Обнови в `rob_box_main.xacro`:
```xml
<xacro:property name="wheelbase" value="???"/>      <!-- Измерить! -->
<xacro:property name="track_width" value="???"/>    <!-- Измерить! -->
```

5. Обнови в `controller_manager.yaml`:
```yaml
diff_drive_controller:
  ros__parameters:
    wheel_separation: ???  # = track_width
```

---

## 🔍 Замечания

### ✅ Хорошо:
- Размеры сенсоров совпадают со спецификациями
- Радиус колес правильный (0.115 м = 10")
- Координаты сенсоров извлечены успешно

### ⚠️ Требует внимания:
- STL колес не содержат правильные координаты (все в одной точке)
- Размеры base_link немного отличаются
- Wheelbase и track_width нужно измерить вручную в Fusion 360

### 📌 Следующие шаги:
1. Обновить координаты сенсоров в URDF ✅ (данные есть)
2. Измерить wheelbase и track_width в Fusion 360
3. Обновить параметры контроллера
4. Протестировать с реальным роботом

---

## 🛠️ Использованные инструменты

- **numpy-stl:** Анализ STL файлов
- **Python script:** `scripts/analyze_stl_meshes.py`
- **Fusion 360 Export:** STL meshes

**Команда для повторного анализа:**
```bash
python3 scripts/analyze_stl_meshes.py
```
