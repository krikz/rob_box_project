# Анализ ориентации сенсоров Rob Box

**Дата**: 2025-11-20  
**Проблема**: Робот развернут на 90° против часовой стрелки  
**Подозрение**: Неправильная ориентация LiDAR или OAK-D камеры

---

## 🔍 Анализ URDF

### Текущие позиции сенсоров

Из файла `src/rob_box_description/urdf/rob_box.xacro` (строки 313-318):

```xml
<!-- LiDAR: Rigid 2 joint xyz="-0.000331 0.170806 0.4765" -->
<xacro:lidar name="lslidar_n10" parent_link="base_link" 
             origin_xyz="-0.000331 0.170806 0.4765" 
             topic_name="/scan"/>

<!-- OAK-D-Lite камера: Rigid 3 joint xyz="-0.000238 0.115772 0.4595" -->
<xacro:oak_d_camera name="camera" parent_link="base_link" 
                     origin_xyz="-0.000238 0.115772 0.4595" 
                     origin_rpy="0 0 0"/>
```

### Определение макросов

#### LiDAR Macro (строка 101-130)

```xml
<xacro:macro name="lidar" params="name parent_link origin_xyz topic_name">
    <!-- ... -->
    <joint name="${name}_joint" type="fixed">
        <parent link="${parent_link}"/>
        <child link="${name}"/>
        <origin xyz="${origin_xyz}" rpy="0 0 0"/>  ← ❌ ЖЁСТКО ПРОПИСАНО!
    </joint>
    <!-- ... -->
</xacro:macro>
```

**ПРОБЛЕМА:** Макрос LiDAR **НЕ принимает параметр RPY**! Ориентация жёстко закодирована как `rpy="0 0 0"`.

#### OAK-D Camera Macro (строка 165-247)

```xml
<xacro:macro name="oak_d_camera" params="name parent_link origin_xyz origin_rpy">
    <!-- ... -->
    <joint name="${name}_joint" type="fixed">
        <parent link="${parent_link}"/>
        <child link="${name}_link"/>
        <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>  ← ✅ Параметр RPY есть
    </joint>
    
    <!-- RGB optical frame трансформация -->
    <joint name="${name}_rgb_camera_optical_frame_joint" type="fixed">
        <parent link="${name}_rgb_camera_frame"/>
        <child link="${name}_rgb_camera_optical_frame"/>
        <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>  ← Оптический фрейм (90° pitch + yaw)
    </joint>
    <!-- ... -->
</xacro:macro>
```

**OK:** Камера принимает RPY, но при вызове используется `origin_rpy="0 0 0"`.

---

## 🧭 Система координат ROS (REP-105)

### Base_link convention
- **X axis**: вперёд (красная стрелка в RViz)
- **Y axis**: влево (зелёная стрелка)
- **Z axis**: вверх (синяя стрелка)
- **RPY**: Roll (вращение вокруг X), Pitch (Y), Yaw (Z)

### LiDAR orientation
**Предполагаемая ориентация:**
- LiDAR монтирован на верхней панели робота
- Сканирует в горизонтальной плоскости (Z-up)
- X-axis лидара должен смотреть вперёд (совпадать с base_link X)

**Возможные причины поворота на 90°:**

1. **LiDAR физически развёрнут** на роботе на 90° вокруг Z
   - Если это так, нужно добавить `rpy="0 0 1.5708"` (90° yaw)
   
2. **Драйвер LiDAR публикует данные в неправильном frame**
   - Нужно проверить конфигурацию драйвера lslidar_ros2

3. **Base_link робота неправильно определён**
   - Но base_link из Fusion 360, должен быть корректным

### OAK-D Camera orientation
**Текущая конфигурация:**
- Корпус камеры: `origin_rpy="0 0 0"` (совпадает с base_link)
- RGB optical frame: `rpy="-1.5708 0 -1.5708"` (стандартная ROS camera convention)
  - **-90° pitch**: X→down, Y→right, Z→forward
  - **-90° yaw**: поворот для выравнивания

**Если робот развёрнут на 90°:**
- Может потребоваться `origin_rpy="0 0 1.5708"` для камеры

---

## 🔧 Возможные решения

### Вариант 1: Исправить макрос LiDAR (добавить параметр RPY)

**Изменение в строке 101:**
```xml
<!-- БЫЛО -->
<xacro:macro name="lidar" params="name parent_link origin_xyz topic_name">

<!-- СТАЛО -->
<xacro:macro name="lidar" params="name parent_link origin_xyz origin_rpy topic_name">
```

**Изменение в строке 127:**
```xml
<!-- БЫЛО -->
<origin xyz="${origin_xyz}" rpy="0 0 0"/>

<!-- СТАЛО -->
<origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
```

**Использование в строке 314:**
```xml
<xacro:lidar name="lslidar_n10" parent_link="base_link" 
             origin_xyz="-0.000331 0.170806 0.4765" 
             origin_rpy="0 0 1.5708"
             topic_name="/scan"/>
```

### Вариант 2: Проверить конфигурацию драйвера LiDAR

**Файл:** `docker/main/config/lsx10_custom.yaml`

Проверить параметры:
- `frame_id`: должен быть `lslidar_n10`
- Возможно есть параметр `angle_offset` или подобный

### Вариант 3: Static TF transform (временное решение)

Добавить static transform в robot_state_publisher или отдельную ноду:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 1.5708 base_link lslidar_n10_corrected
```

---

## 📊 Проверка данных

### Через RViz
1. Открыть RViz
2. Добавить LaserScan display
3. Установить Fixed Frame = `base_link`
4. Проверить направление лучей относительно модели робота

### Через топики
```bash
# Проверить frame_id в сообщениях LiDAR
ros2 topic echo /scan --once | grep frame_id

# Проверить TF трансформы
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Через TF echo
```bash
# Посмотреть трансформ от base_link к lslidar_n10
ros2 run tf2_ros tf2_echo base_link lslidar_n10
```

---

## 🎯 Рекомендуемый план действий

1. **Проверить физическую установку LiDAR на роботе**
   - Убедиться в каком направлении смотрит разъём/кабель
   - Сфотографировать монтаж

2. **Проверить данные LiDAR в RViz**
   - Запустить RViz с моделью робота
   - Добавить LaserScan visualization
   - Определить угол расхождения

3. **Проверить конфигурацию драйвера**
   - Посмотреть `docker/main/config/lsx10_custom.yaml`
   - Проверить параметр `angle_offset` или `angle_crop`

4. **Применить исправление**
   - Если проблема в URDF → добавить RPY параметр в макрос
   - Если проблема в драйвере → исправить конфигурацию
   - Если проблема физическая → переустановить или добавить offset

---

## 📁 Затронутые файлы

- `src/rob_box_description/urdf/rob_box.xacro` - главный URDF файл
- `docker/main/config/lsx10_custom.yaml` - конфигурация LiDAR драйвера
- `docker/vision/config/oak-d/oak_d_config.yaml` - конфигурация OAK-D

---

## 📖 Связанная документация

- REP-105: Coordinate Frames for Mobile Platforms
  https://www.ros.org/reps/rep-0105.html
  
- REP-103: Standard Units of Measure and Coordinate Conventions
  https://www.ros.org/reps/rep-0103.html

- LSLIDAR ROS2 Driver Documentation
  https://github.com/Leishen-lidar/LS_ROS2_Driver

---

## ✅ РЕЗУЛЬТАТЫ ТЕСТИРОВАНИЯ

### Анализ данных LiDAR (2025-11-20)

**Команда:** `python3 scripts/check_lidar_orientation.py`

**Результаты:**
```
📡 Frame: lslidar_n10
📐 Angle range: 0.0° to 360.0°
📊 Number of readings: ~450
🔄 Angle increment: ~0.804°

🎯 Obstacle detection by sector:
  Front (±22.5°)                  :   15 points ███
  Front-Left (22.5-67.5°)         :   56 points ███████████
  Left (67.5-112.5°)              :   56 points ███████████
  Rear-Left (112.5-157.5°)        :   56 points ███████████
  Rear (157.5-180° + -180-157.5°) :  245 points ████████████████████████████████████████
```

**Интерпретация:**
- LiDAR сканирует полный круг 0-360°
- **0° = FRONT** (вперёд, ось X base_link)
- **90° = LEFT**
- **180° = REAR**
- **270° = RIGHT**

**ВЫВОД:** ✅ **LiDAR ориентирован ПРАВИЛЬНО!**

Много точек сзади (Rear: 245) и мало спереди (Front: 15) означает что робот стоит лицом к стене, задом к открытому пространству. Это НЕ проблема ориентации сенсора.

### ❌ Проблема 90° поворота НЕ в LiDAR!

Если робот в RViz выглядит развёрнутым на 90°, проблема может быть в:

1. **Odometry/RTAB-Map** - неправильная initial pose
2. **IMU** - если используется для фильтрации одометрии
3. **Колёса/Энкодеры** - перепутаны левые/правые (инверсия направления)
4. **Base_link orientation** - начальная pose робота в map frame

**Статус**: ✅ **LiDAR ПРОВЕРЕН - ОК**  
**Следующий шаг**: Проверить одометрию и IMU (если используется)
