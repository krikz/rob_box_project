# LiDAR 180° Rotation Fix

**Дата**: 2025-11-20  
**Проблема**: Робот развёрнут на 180° - LiDAR смотрит назад вместо вперёд  
**Статус**: 🔴 **КРИТИЧЕСКАЯ ПРОБЛЕМА ОБНАРУЖЕНА**

---

## 🔍 Диагностика

### Тестирование LiDAR orientation

**Команда:**
```bash
python3 scripts/check_lidar_orientation.py
```

**Результаты:**
```
📡 Frame: lslidar_n10
📐 Angle range: 0.0° to 360.0°

🎯 Obstacle detection by sector:
  Front (±22.5°)                  :   15 points ███
  Front-Left (22.5-67.5°)         :   56 points ███████████
  Left (67.5-112.5°)              :   56 points ███████████
  Rear-Left (112.5-157.5°)        :   56 points ███████████
  Rear (157.5-180° + -180-157.5°) :  245 points ████████████████████████████████████████
```

### 🚨 Критическая находка

**Физическое положение робота**: Лицом к открытому пространству  
**Данные LiDAR показывают**: Rear sector имеет МНОГО препятствий (245 points)  
**Front sector**: Мало препятствий (15 points)

**ВЫВОД:** ❌ **LiDAR физически развёрнут на 180°!**

То, что LiDAR считает "вперёд" (0°), на самом деле смотрит **НАЗАД** на роботе!

---

## 🔧 Решение

### Вариант 1: Физическое переустановка LiDAR (РЕКОМЕНДУЕТСЯ)

**Действие:** Развернуть LiDAR на 180° на монтажной площадке

**Преимущества:**
- ✅ Правильная физическая ориентация
- ✅ Интуитивное понимание данных
- ✅ Не требует изменений в софте
- ✅ Соответствует URDF из Fusion 360

**Недостатки:**
- ⚠️ Требует физического доступа к роботу
- ⚠️ Нужно переподключить кабели

---

### Вариант 2: Исправление в URDF (ВРЕМЕННОЕ)

Если физическая переустановка невозможна сейчас, можно исправить в URDF.

#### Шаг 1: Добавить параметр RPY в макрос LiDAR

**Файл:** `src/rob_box_description/urdf/rob_box.xacro`

**Строка 101** - изменить сигнатуру макроса:

```xml
<!-- БЫЛО -->
<xacro:macro name="lidar" params="name parent_link origin_xyz topic_name">

<!-- СТАЛО -->
<xacro:macro name="lidar" params="name parent_link origin_xyz origin_rpy topic_name">
```

**Строка 127** - использовать параметр RPY:

```xml
<!-- БЫЛО -->
<origin xyz="${origin_xyz}" rpy="0 0 0"/>

<!-- СТАЛО -->
<origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
```

#### Шаг 2: Применить поворот 180° при вызове макроса

**Строка 314** - добавить 180° вокруг Z:

```xml
<!-- БЫЛО -->
<xacro:lidar name="lslidar_n10" parent_link="base_link" 
             origin_xyz="-0.000331 0.170806 0.4765" 
             topic_name="/scan"/>

<!-- СТАЛО -->
<xacro:lidar name="lslidar_n10" parent_link="base_link" 
             origin_xyz="-0.000331 0.170806 0.4765" 
             origin_rpy="0 0 3.14159"
             topic_name="/scan"/>
```

**Примечание:** `3.14159` радиан = 180° (π radians)

#### Шаг 3: Пересобрать и перезапустить

```bash
# На build machine
cd ~/rob_box_project
colcon build --packages-select rob_box_description

# На Main Pi - перезапустить robot_state_publisher
ssh ros2@10.1.1.20
cd ~/rob_box_project
docker-compose -f docker/main/docker-compose.yaml restart robot-state-publisher
```

---

### Вариант 3: Исправление в драйвере LiDAR (НЕ РЕКОМЕНДУЕТСЯ)

Некоторые драйверы LiDAR имеют параметр `angle_offset`, но:
- ❌ Не все драйверы поддерживают
- ❌ Может сломаться при обновлении драйвера
- ❌ Неочевидно для других разработчиков

---

## 📝 Изменения в коде

### Файл 1: `src/rob_box_description/urdf/rob_box.xacro`

```diff
--- a/src/rob_box_description/urdf/rob_box.xacro
+++ b/src/rob_box_description/urdf/rob_box.xacro
@@ -98,7 +98,7 @@
 	
 	<!-- Сенсоры -->
-	<xacro:macro name="lidar" params="name parent_link origin_xyz topic_name">
+	<xacro:macro name="lidar" params="name parent_link origin_xyz origin_rpy topic_name">
 		<link name="${name}">
 			<visual>
 				<origin xyz="0 0 0" rpy="0 0 0"/>
@@ -124,7 +124,7 @@
 		<joint name="${name}_joint" type="fixed">
 			<parent link="${parent_link}"/>
 			<child link="${name}"/>
-			<origin xyz="${origin_xyz}" rpy="0 0 0"/>
+			<origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
 		</joint>
 		
 		<gazebo reference="${name}">
@@ -311,7 +311,8 @@
 	<!-- Сенсоры - позиции из Fusion 360 Export (URDF_ROBBOX.xacro) -->
 	<!-- LiDAR: Rigid 2 joint xyz="-0.000331 0.170806 0.4765" -->
 	<xacro:lidar name="lslidar_n10" parent_link="base_link" 
-	             origin_xyz="-0.000331 0.170806 0.4765" 
+	             origin_xyz="-0.000331 0.170806 0.4765"
+	             origin_rpy="0 0 3.14159"
 	             topic_name="/scan"/>
 	<!-- OAK-D-Lite камера: Rigid 3 joint xyz="-0.000238 0.115772 0.4595" -->
```

---

## 🧪 Проверка после исправления

После применения исправления, запустить скрипт проверки снова:

```bash
cd ~/rob_box_project
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_rviz_config_RBXU100001.json5
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 scripts/check_lidar_orientation.py
```

**Ожидаемый результат:**
```
🎯 Obstacle detection by sector:
  Front (±22.5°)                  :  245 points ████████████████████████████████████████
  Rear (157.5-180° + -180-157.5°) :   15 points ███
```

Теперь Front должен иметь МНОГО точек (открытое пространство перед роботом), а Rear - мало.

---

## 📊 Проверка в RViz

После исправления, проверить в RViz:

1. Запустить RViz:
   ```bash
   scripts/start_rviz.sh
   ```

2. Добавить LaserScan display:
   - Add → By topic → /scan → LaserScan
   - Установить Fixed Frame = `base_link`

3. Проверить визуализацию:
   - ✅ Лучи LiDAR должны смотреть ВПЕРЁД (в направлении X axis base_link)
   - ✅ Препятствия впереди робота должны отображаться правильно

---

## 🎯 Корневая причина

### Почему это произошло?

**Вероятные причины:**

1. **Физическая установка:** LiDAR был установлен на роботе неправильно (развёрнут на 180°)

2. **Fusion 360 Export:** URDF из Fusion 360 может не учитывать ориентацию компонентов (только позицию XYZ)

3. **Отсутствие валидации:** При сборке робота не проверили соответствие физической ориентации и URDF

### Как избежать в будущем?

✅ **Всегда проверять ориентацию сенсоров после установки:**
```bash
# Запустить LiDAR и проверить что препятствие ВПЕРЕДИ показывается в секторе Front
python3 scripts/check_lidar_orientation.py
```

✅ **Добавить маркировку на LiDAR:** Стрелка "FORWARD" для визуального контроля

✅ **Документировать установку:** Фото монтажа с указанием направления

---

## 📖 Связанная документация

- `SENSOR_ORIENTATION_ANALYSIS_2025-11-20.md` - детальный анализ проблемы
- `docs/architecture/HARDWARE.md` - спецификация монтажа сенсоров
- `src/rob_box_description/urdf/rob_box.xacro` - URDF описание робота

---

## ✅ Чеклист исправления

- [ ] **ЛИБО** Физически развернуть LiDAR на 180°
- [ ] **ЛИБО** Применить исправление в URDF (добавить `origin_rpy="0 0 3.14159"`)
- [ ] Пересобрать rob_box_description пакет
- [ ] Перезапустить robot-state-publisher на Main Pi
- [ ] Проверить с помощью `check_lidar_orientation.py`
- [ ] Проверить в RViz что лучи смотрят вперёд
- [ ] Обновить фотодокументацию монтажа

---

**Приоритет**: 🔴 **ВЫСОКИЙ**  
**Влияние**: Навигация, картография, obstacle avoidance - всё работает неправильно  
**Рекомендация**: Исправить как можно скорее (физическая переустановка предпочтительнее)
