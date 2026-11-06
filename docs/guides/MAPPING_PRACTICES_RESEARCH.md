# Исследование практик картографирования для проекта Rob Box

> **Дата создания**: 24 октября 2025  
> **Статус**: ✅ Исследовательский документ  
> **Платформа**: Rob Box (Dual Raspberry Pi 5, RTABMAP, Nav2)

---

## 📋 Содержание

1. [Введение](#введение)
2. [Обзор текущей системы](#обзор-текущей-системы)
3. [Практики картографирования с RTABMAP](#практики-картографирования-с-rtabmap)
4. [Автономное исследование выбранных областей](#автономное-исследование-выбранных-областей)
5. [Использование AprilTag кубов для точности](#использование-apriltag-кубов-для-точности)
6. [Очистка временных маркеров](#очистка-временных-маркеров)
7. [Настройка постоянных точек навигации](#настройка-постоянных-точек-навигации)
8. [Интеграция с голосовым управлением](#интеграция-с-голосовым-управлением)
9. [Облачная конфигурация через Zenoh](#облачная-конфигурация-через-zenoh)
10. [Рекомендации по реализации](#рекомендации-по-реализации)

---

## 1. Введение

### 1.1. Цель исследования

Данное исследование направлено на разработку комплексного подхода к созданию и управлению картами для автономного робота Rob Box. Основные задачи:

- 🗺️ **Оптимизация процесса картографирования** - определение лучших практик создания карт с RTABMAP
- 🤖 **Автономное исследование** - разработка методов самостоятельного исследования выбранных областей
- 🎯 **Точность картографии** - использование AprilTag маркеров для повышения качества карт
- 🔧 **Управление конфигурацией** - облачные решения для управления точками навигации
- 🗣️ **Голосовое управление** - интеграция картографических команд с voice assistant

### 1.2. Контекст проекта

Rob Box - это автономный мобильный робот с распределенной архитектурой:

- **Main Pi (Raspberry Pi 5, 16GB)**: RTABMAP SLAM, Nav2 навигация, управление моторами (VESC)
- **Vision Pi (Raspberry Pi 5, 8GB)**: OAK-D камера, LSLIDAR N10, AprilTag детекция, голосовой ассистент
- **Middleware**: Zenoh DDS с облачной синхронизацией (zenoh.robbox.online:7447)
- **SLAM**: RTABMAP с 2D LiDAR (только лазерный сканер, без RGB-D)
- **Навигация**: Nav2 с DWB контроллером и NavFn планировщиком

---

## 2. Обзор текущей системы

### 2.1. Текущая конфигурация RTABMAP

**Местоположение**: `docker/main/config/rtabmap/rtabmap.yaml` (удалён в #4495; параметры — `docker/main/scripts/rtabmap/patch_rtabmap_launch.py`)

```yaml
rtabmap:
  ros__parameters:
    # Режим работы
    Mem/IncrementalMemory: true  # SLAM mode (mapping)
    
    # LaserScan параметры
    subscribe_scan: true
    subscribe_rgbd: false  # Не используем RGB-D
    
    # Grid mapping
    Grid/FromDepth: "false"  # Только LiDAR
    Grid/CellSize: "0.05"    # 5см разрешение
    Grid/RangeMax: "10.0"    # До 10 метров
    
    # ICP для 2D лидара
    Reg/Strategy: "1"        # ICP (Iterative Closest Point)
    Icp/PointToPlane: "false"  # 2D ICP
    Icp/Iterations: "30"
```

**Ключевые особенности**:
- ✅ Работает только с 2D LiDAR (LSLIDAR N10)
- ✅ База данных: `/maps/rtabmap.db`
- ✅ Поддержка режимов: Mapping (создание карты) и Localization (навигация)
- ✅ Loop closure detection для корректировки карты

### 2.2. Существующие голосовые команды

**Местоположение**: `src/rob_box_voice/rob_box_voice/command_node.py`

Уже реализованные команды для картографии:

```python
# Словарь waypoints (хардкодированный)
self.waypoints = {
    'дом': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
    'кухня': {'x': 2.0, 'y': 1.0, 'theta': 0.0},
    'гостиная': {'x': 3.0, 'y': 2.0, 'theta': 1.57},
    'точка 1': {'x': 1.0, 'y': 0.0, 'theta': 0.0},
}
```

**Документация**: `docs/packages/MAPPING_COMMANDS.md` содержит спецификацию команд:
- "Исследуй территорию" - создание новой карты
- "Продолжи исследование" - продолжение картографии
- "Закончи исследование" - переход в режим локализации


### 2.3. AprilTag система

**Местоположение**: `docker/vision/config/apriltag/apriltag_config.yaml`

```yaml
apriltag:
  ros__parameters:
    family: 36h11          # Семейство тегов
    size: 0.16             # Размер 16 см
    image_transport: raw
    detector:
      threads: 2
      decimate: 1.0
      refine: true
```

**Генерация тегов**: `scripts/generate_apriltags.py` - скрипт для печати тегов с ID

---

## 3. Практики картографирования с RTABMAP

### 3.1. Режимы работы RTABMAP

#### Режим Mapping (Картографирование)

**Параметры**:
```yaml
Mem/IncrementalMemory: true  # Добавление новых узлов в граф
```

**Когда использовать**:
- ✅ Первое исследование нового помещения
- ✅ Добавление новых областей к существующей карте
- ✅ Обновление карты после изменений в помещении

**Процесс**:
1. Запустить RTABMAP в режиме mapping
2. Медленно двигаться через помещение (0.2-0.3 м/с)
3. Обеспечить перекрытие сканов (не делать резких поворотов)
4. Периодически возвращаться к уже известным местам для loop closure

#### Режим Localization (Локализация)

**Параметры**:
```yaml
Mem/IncrementalMemory: false  # Не добавлять новые узлы
```

**Когда использовать**:
- ✅ Навигация по готовой карте
- ✅ Автономная доставка
- ✅ Патрулирование известных маршрутов

**Преимущества**:
- ⚡ Быстрее чем режим mapping (меньше вычислений)
- 💾 Не увеличивает размер базы данных
- 🎯 Более стабильная локализация

### 3.2. Процедура создания новой карты

#### Шаг 1: Подготовка

```bash
# На Main Pi
cd ~/rob_box_project/docker/main

# Создать backup текущей карты
docker exec rtabmap bash -c "
  if [ -f /maps/rtabmap.db ]; then
    cp /maps/rtabmap.db /maps/backups/rtabmap_backup_$(date +%Y-%m-%d_%H-%M-%S).db
    echo 'Backup created'
  fi
"
```

#### Шаг 2: Запуск в режиме mapping

**Голосовая команда**: "Исследуй территорию"

**Или вручную через ROS2**:
```bash
# Сброс памяти для новой карты
ros2 service call /rtabmap/reset_memory std_srvs/srv/Empty

# Переключение в mapping mode
ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty
```

#### Шаг 3: Исследование территории

**Рекомендации по движению**:

- 🐢 **Скорость**: 0.2-0.3 м/с (медленно)
- 🔄 **Повороты**: Плавные, не более 30°/с
- 📏 **Перекрытие**: 30-50% между последовательными сканами
- 🔁 **Loop closure**: Возвращаться к известным местам каждые 2-3 минуты
- 🚫 **Избегать**: Динамические объекты, зеркала, стекло

**Стратегии обхода**:

1. **Спиральное движение** (для открытых пространств):
   - Начать от центра
   - Двигаться по расширяющейся спирали
   - Возвращаться к центру для loop closure

2. **По периметру** (для комнат):
   - Начать у стены
   - Двигаться вдоль стен по часовой стрелке
   - Затем исследовать центральную часть

3. **Зигзаг** (для длинных коридоров):
   - Двигаться от одного конца к другому
   - Делать небольшие отклонения влево-вправо
   - Возвращаться по тому же маршруту

#### Шаг 4: Завершение картографии

**Голосовая команда**: "Закончи исследование"

**Или вручную**:
```bash
# Переключение в localization mode
ros2 service call /rtabmap/set_mode_localization std_srvs/srv/Empty

# Проверка режима
ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory
# Output: Boolean value is: false (localization mode)
```

### 3.3. Продолжение существующей карты

Если нужно добавить новые области к существующей карте:

**Голосовая команда**: "Продолжи исследование"

**Процесс**:
1. RTABMAP загружает существующую БД
2. Робот локализуется на карте
3. Переключение в mapping mode
4. Исследование новых областей
5. Автоматическое связывание с существующей картой через loop closure

**Важно**: Начинать с известной области для правильной локализации!

---

## 4. Автономное исследование выбранных областей

### 4.1. Проблема частичного картографирования

**Задача**: Исследовать только часть этажа (например, одно крыло здания)

**Традиционный подход** (ручное управление):
- ❌ Требует постоянного внимания оператора
- ❌ Неоптимальный маршрут
- ❌ Пропуск областей

**Предлагаемое решение** (автономное исследование):
- ✅ Автоматическое планирование маршрута
- ✅ Полное покрытие выбранной области
- ✅ Оператор задает только границы

### 4.2. Подходы к автономному исследованию

#### Подход 1: Frontier Exploration

**Концепция**: Робот автоматически находит и исследует границы между известным и неизвестным пространством

**Алгоритм**:
```
1. Анализировать текущую карту
2. Найти "frontiers" - границы между известным и unknown
3. Оценить каждый frontier:
   - Расстояние до робота
   - Размер неизведанной области
   - Информационная ценность
4. Выбрать лучший frontier
5. Построить путь до него (Nav2)
6. Двигаться к frontier
7. Повторить пока есть frontiers
```

**Реализация для ROS2**:
- Пакет: `explore_lite` или `frontier_exploration`
- Работает поверх Nav2
- Использует RTABMAP карту

**Пример запуска**:
```bash
ros2 launch explore_lite explore.launch.py
```

#### Подход 2: Coverage Path Planning

**Концепция**: Заранее спланировать путь который покроет всю выбранную область

**Алгоритмы**:
- **Boustrophedon (зигзаг)**: Движение параллельными линиями
- **Spiral**: Спиральное движение от центра
- **Grid-based**: Движение по сетке

**Преимущества**:
- ✅ Гарантированное полное покрытие
- ✅ Предсказуемое время выполнения
- ✅ Оптимальный маршрут

**Недостатки**:
- ❌ Требует знания границ заранее
- ❌ Не адаптируется к препятствиям

#### Подход 3: Гибридный (рекомендуется для Rob Box)

**Идея**: Комбинировать coverage planning с frontier exploration

**Процесс**:
1. Оператор задает область на плане помещения (интерфейс)
2. Генерация waypoints для coverage этой области
3. Робот следует по waypoints
4. Если встречает препятствие - использует frontier exploration для обхода
5. Возвращается к coverage plan после обхода

### 4.3. Интерфейс выбора области картографирования

#### Концепция UI

**Требования**:
- 📱 Веб-интерфейс (доступен через облако)
- 🗺️ Загрузка плана помещения (PNG/PDF)
- 📏 Масштабирование плана относительно реальных размеров
- ✏️ Рисование области для исследования (полигон)
- 🎯 Автоматическая генерация waypoints

**Технологии**:
- **Frontend**: React + Leaflet/OpenLayers для отображения карты
- **Backend**: FastAPI для обработки запросов
- **Связь с роботом**: Zenoh (публикация waypoints в топик)

#### Workflow использования

```
1. Оператор открывает веб-интерфейс
   URL: https://zenoh.robbox.online/mapping-ui

2. Загружает план этажа
   - Загрузка PNG/PDF
   - Установка масштаба (например, "10 метров = 500 пикселей")

3. Накладывает план на текущую позицию робота
   - Робот отображается как точка на UI
   - Оператор совмещает план с позицией робота
   - Поворачивает план если нужно

4. Рисует область для исследования
   - Инструмент "Polygon"
   - Клики по углам области
   - Замыкание полигона

5. Настраивает параметры
   - Шаг между waypoints (по умолчанию 1.5 метра)
   - Стратегия обхода (зигзаг / спираль)
   - Скорость движения (0.2-0.4 м/с)

6. Генерация и отправка
   - Кнопка "Создать маршрут"
   - UI генерирует список waypoints
   - Показывает предпросмотр маршрута
   - Кнопка "Отправить роботу"
   - Waypoints публикуются в Zenoh: robots/RBXU100001/mapping/waypoints

7. Робот выполняет маршрут
   - Получает waypoints через Zenoh
   - Начинает автономное движение
   - Отправляет прогресс в UI: robots/RBXU100001/mapping/progress
   - UI показывает текущее положение и пройденный путь
```

#### Пример генерации waypoints

**Python код для backend**:
```python
def generate_coverage_waypoints(polygon, step=1.5, strategy='boustrophedon'):
    """
    Генерация waypoints для покрытия полигона
    
    Args:
        polygon: List[(x, y)] - вершины полигона
        step: float - расстояние между линиями (метры)
        strategy: str - 'boustrophedon' или 'spiral'
    
    Returns:
        List[{x, y, theta}] - список waypoints
    """
    waypoints = []
    
    if strategy == 'boustrophedon':
        # Найти bounding box
        min_x = min(p[0] for p in polygon)
        max_x = max(p[0] for p in polygon)
        min_y = min(p[1] for p in polygon)
        max_y = max(p[1] for p in polygon)
        
        # Генерировать зигзаг
        y = min_y
        direction = 1  # 1 = вправо, -1 = влево
        
        while y <= max_y:
            if direction == 1:
                x_start, x_end = min_x, max_x
            else:
                x_start, x_end = max_x, min_x
            
            # Добавить waypoint только если внутри полигона
            if point_in_polygon((x_start, y), polygon):
                theta = 0 if direction == 1 else 3.14159
                waypoints.append({'x': x_start, 'y': y, 'theta': theta})
            
            if point_in_polygon((x_end, y), polygon):
                theta = 0 if direction == 1 else 3.14159
                waypoints.append({'x': x_end, 'y': y, 'theta': theta})
            
            y += step
            direction *= -1
    
    return waypoints
```


### 4.4. Реализация в Rob Box

**Новые ROS2 ноды**:

1. **mapping_planner_node** (Main Pi):
   - Подписка: `robots/{ROBOT_ID}/mapping/waypoints`
   - Публикация: `robots/{ROBOT_ID}/mapping/progress`
   - Функция: Управление автономным исследованием

2. **mapping_ui_bridge** (облачный сервер):
   - Zenoh роутер для связи UI ↔ робот
   - REST API для веб-интерфейса

**Интеграция с Nav2**:
```python
# В mapping_planner_node
from nav2_msgs.action import NavigateThroughPoses

def execute_mapping_mission(waypoints):
    """Выполнить маршрут картографии"""
    # Создать Nav2 action
    goal = NavigateThroughPoses.Goal()
    goal.poses = [self.create_pose_stamped(wp) for wp in waypoints]
    
    # Отправить goal
    self.nav_client.send_goal_async(goal, feedback_callback=self.progress_callback)
```

---

## 5. Использование AprilTag кубов для точности

### 5.1. Проблема точности SLAM с 2D LiDAR

**Ограничения только LiDAR**:
- ❌ Низкая точность в длинных коридорах (нет боковых ориентиров)
- ❌ Drift при движении по прямой
- ❌ Сложности с loop closure в однообразных помещениях
- ❌ Плохая работа в пустых комнатах

**Решение**: Добавить визуальные маркеры (AprilTags)

### 5.2. Концепция использования AprilTag кубов

**Идея**: Расставить кубы с AprilTag маркерами в ключевых местах во время картографии

**Типы маркеров**:
1. **Временные** - используются только во время картографии:
   - Расставляются перед началом mapping
   - Помогают в сложных местах (коридоры, пустые комнаты)
   - Удаляются из БД после завершения картографии

2. **Постоянные** - остаются в помещении навсегда:
   - Фиксированные ориентиры для локализации
   - Маркировка важных мест (кухня, зарядная станция)
   - Сохраняются в БД как landmarks

### 5.3. Изготовление AprilTag кубов

#### Материалы

- 📦 **Кубики**: 10×10×10 см (дерево, пластик, картон)
- 🖨️ **Печать**: Лазерный принтер, матовая бумага
- ✂️ **Наклейка**: Прозрачный скотч или ламинация
- 🎨 **Размер тега**: 16 см (как в конфиге: `size: 0.16`)

#### Процесс изготовления

1. **Генерация тегов**:
```bash
cd ~/rob_box_project
python3 scripts/generate_apriltags.py

# Создает: apriltags_printable/tag36h11_id000.png ... id586.png
```

2. **Печать**:
   - Размер: 180×180 мм (16см тег + 2см рамка)
   - Качество: 300+ DPI
   - Бумага: Матовая белая
   - Проверить размер линейкой после печати!

3. **Наклейка на кубы**:
   - По одному тегу на каждую сторону куба
   - Или один тег сверху (для потолочной камеры)
   - Защитить прозрачным скотчем

#### Рекомендуемые ID для разных целей

```python
# Временные маркеры (для картографии)
MAPPING_TAGS = range(100, 200)  # ID 100-199

# Постоянные ориентиры
PERMANENT_LANDMARKS = {
    0: "home",           # Домашняя позиция
    1: "kitchen",        # Кухня
    2: "living_room",    # Гостиная
    3: "charging",       # Зарядная станция
    # ... до ID 99
}
```

### 5.4. Размещение AprilTag кубов при картографии

#### Стратегия размещения временных маркеров

**Где ставить кубы**:

1. **Длинные коридоры**:
   - Каждые 3-5 метров по одному кубу
   - Чередовать стороны (слева-справа)
   - Высота: 30-50 см от пола

2. **Пустые комнаты**:
   - По одному кубу в каждом углу
   - Один в центре

3. **Перекрестки / T-образные перекрестки**:
   - По кубу с каждой стороны перекрестка
   - Помогает в локализации после поворота

4. **Начальная точка**:
   - Всегда ставить куб у стартовой позиции
   - Для loop closure в конце маршрута

#### Интеграция с RTABMAP

**Топики OAK-D камеры** (Vision Pi):
- `/camera/rgb/image_raw` - изображение
- `/camera/rgb/camera_info` - калибровка

**Топики AprilTag детекции**:
- `/apriltag/detections` - обнаруженные теги
- `/apriltag/tf` - TF трансформации для каждого тега

**RTABMAP может использовать AprilTag**:
```yaml
# docker/main/config/rtabmap/rtabmap.yaml
rtabmap:
  ros__parameters:
    # Добавить visual odometry
    subscribe_rgb: true
    subscribe_depth: true
    subscribe_rgbd: false
    
    # Включить landmarks
    Marker/DetectionRate: 1.0  # Проверять каждый кадр
    Marker/CornerRefinement: 1  # Уточнение углов
```

**Процесс**:
1. OAK-D камера детектирует AprilTag
2. AprilTag нода публикует TF: `camera_link` → `tag_N`
3. RTABMAP добавляет tag как landmark в граф
4. При повторном обнаружении тега - корректирует позицию (loop closure)

### 5.5. Улучшение точности с AprilTag

**Преимущества**:
- ✅ **Абсолютная позиция**: Тег дает точное положение в 6-DOF
- ✅ **Loop closure**: Мгновенное распознавание known place
- ✅ **Drift correction**: Исправление накопленной ошибки
- ✅ **Робастность**: Работает в однообразных помещениях

**Метрики улучшения** (по литературе):
- Позиционная ошибка: уменьшение с ~10cm до ~2cm
- Угловая ошибка: уменьшение с ~5° до ~1°
- Успешность loop closure: увеличение с 70% до 95%

---

## 6. Очистка временных маркеров

### 6.1. Необходимость очистки

**Проблема**: После картографии в БД остаются временные AprilTag маркеры

**Почему это плохо**:
- 📦 Увеличение размера БД
- 🐢 Замедление локализации (поиск среди лишних landmarks)
- ❌ Ложные loop closures если кубы убрали
- 🔄 Confusion при навигации

**Решение**: Удалить временные маркеры, оставить только постоянные

### 6.2. Классификация маркеров

**В базе данных RTABMAP**:

Маркеры хранятся как "Nodes" с типом "Landmark"

```sql
-- Структура таблицы Node в rtabmap.db
CREATE TABLE Node(
    id INTEGER PRIMARY KEY,
    map_id INTEGER,
    weight INTEGER,
    pose BLOB,
    stamp REAL,
    label TEXT,
    ground_truth_pose BLOB,
    velocity BLOB,
    gps BLOB,
    env_sensors BLOB
);

-- Landmarks имеют особый label
-- Например: "apriltag_100", "apriltag_1", etc.
```

**Схема меток**:
```python
TEMPORARY_TAG_PREFIX = "apriltag_1"  # ID 100-199
PERMANENT_TAG_PREFIX = "apriltag_0"  # ID 0-99
```

### 6.3. Процедура очистки

#### Метод 1: Через RTABMAP API (рекомендуется)

```python
#!/usr/bin/env python3
"""
Скрипт для очистки временных AprilTag маркеров из RTABMAP БД
"""

import sqlite3
import sys

def cleanup_temporary_markers(db_path, temp_prefix="apriltag_1"):
    """
    Удалить временные маркеры из БД
    
    Args:
        db_path: Путь к rtabmap.db
        temp_prefix: Префикс временных маркеров (по умолчанию apriltag_1)
    """
    # Подключиться к БД
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Найти все временные маркеры
    cursor.execute("""
        SELECT id, label FROM Node 
        WHERE label LIKE ? || '%'
    """, (temp_prefix,))
    
    temp_markers = cursor.fetchall()
    print(f"Найдено {len(temp_markers)} временных маркеров")
    
    # Удалить каждый маркер
    for marker_id, label in temp_markers:
        print(f"  Удаление: {label} (ID: {marker_id})")
        
        # Удалить node
        cursor.execute("DELETE FROM Node WHERE id = ?", (marker_id,))
        
        # Удалить связанные данные
        cursor.execute("DELETE FROM Link WHERE from_id = ? OR to_id = ?", 
                       (marker_id, marker_id))
        cursor.execute("DELETE FROM Data WHERE id = ?", (marker_id,))
    
    # Сохранить изменения
    conn.commit()
    conn.close()
    
    print(f"✅ Очистка завершена. Удалено {len(temp_markers)} маркеров")

if __name__ == "__main__":
    db_path = "/maps/rtabmap.db"
    cleanup_temporary_markers(db_path)
```

**Запуск на Main Pi**:
```bash
docker exec rtabmap python3 /scripts/cleanup_temp_markers.py
```

#### Метод 2: Через ROS2 сервис (если RTABMAP поддерживает)

```bash
# Удалить конкретный landmark
ros2 service call /rtabmap/remove_landmark \
  rtabmap_msgs/srv/RemoveLandmark \
  "{landmark_id: 100}"
```

### 6.4. Автоматическая очистка после картографии

**Интеграция с голосовым управлением**:

```python
# В dialogue_node.py
async def handle_finish_mapping(self):
    """Завершить картографию и очистить временные маркеры"""
    
    # 1. Переключить в localization mode
    await self.call_service(self.set_mode_localization_client)
    
    # 2. Подождать сохранения БД
    await asyncio.sleep(5.0)
    
    # 3. Запустить cleanup script
    result = subprocess.run([
        'docker', 'exec', 'rtabmap',
        'python3', '/scripts/cleanup_temp_markers.py'
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        return "Картографирование завершено. Временные маркеры очищены. Карта готова для навигации."
    else:
        return f"Картографирование завершено, но ошибка очистки маркеров: {result.stderr}"
```

**Голосовая команда**: "Закончи исследование"
- Автоматически переключает в localization
- Очищает временные AprilTag
- Сообщает результат

---

## 7. Настройка постоянных точек навигации

### 7.1. Типы точек навигации

**1. Именованные локации** (для голосовых команд):
```python
{
    'дом': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'description': 'Начальная позиция'},
    'кухня': {'x': 2.5, 'y': 1.2, 'theta': 0.0, 'description': 'Кухня'},
    'зарядная станция': {'x': -1.0, 'y': 0.5, 'theta': 3.14, 'description': 'Зарядка'}
}
```

**2. AprilTag landmarks** (постоянные маркеры):
```python
{
    'tag_0': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'type': 'apriltag', 'description': 'Home marker'},
    'tag_1': {'x': 2.5, 'y': 1.2, 'theta': 0.0, 'type': 'apriltag', 'description': 'Kitchen marker'}
}
```

**3. Зоны доставки** (для автономной доставки):
```python
{
    'delivery_zone_A': {'x': 3.0, 'y': 2.0, 'radius': 0.5, 'type': 'delivery'},
    'delivery_zone_B': {'x': 1.5, 'y': 3.0, 'radius': 0.5, 'type': 'delivery'}
}
```

### 7.2. Методы определения координат точек

#### Метод 1: Ручная навигация и запись

**Процесс**:
1. Управлять роботом вручную (джойстик/веб-интерфейс)
2. Подвести робота к нужному месту (например, кухня)
3. Записать текущую позицию

**Скрипт для записи**:
```python
#!/usr/bin/env python3
"""Сохранить текущую позицию робота как waypoint"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Подписаться на позицию робота
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.pose_callback,
            10
        )
        
        self.current_pose = None
    
    def pose_callback(self, msg):
        """Обновить текущую позицию"""
        self.current_pose = msg.pose.pose
    
    def save_waypoint(self, name, description=""):
        """Сохранить текущую позицию как waypoint"""
        if not self.current_pose:
            print("❌ Позиция робота неизвестна")
            return
        
        # Извлечь координаты
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Извлечь угол из quaternion
        from tf_transformations import euler_from_quaternion
        q = self.current_pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        waypoint = {
            'name': name,
            'x': round(x, 2),
            'y': round(y, 2),
            'theta': round(theta, 2),
            'description': description
        }
        
        print(f"✅ Waypoint '{name}': x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        
        # Сохранить в файл
        with open(f'/config/waypoints/{name}.json', 'w') as f:
            json.dump(waypoint, f, indent=2)

def main():
    rclpy.init()
    recorder = WaypointRecorder()
    
    # Подождать получения позиции
    print("Ожидание позиции робота...")
    while recorder.current_pose is None:
        rclpy.spin_once(recorder, timeout_sec=0.1)
    
    # Запросить имя waypoint
    name = input("Введите имя точки (например, 'кухня'): ")
    description = input("Описание (опционально): ")
    
    # Сохранить
    recorder.save_waypoint(name, description)
    
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Использование**:
```bash
# На Vision Pi
docker exec -it voice-assistant bash
ros2 run rob_box_voice waypoint_recorder

# Ввести: "кухня"
# Waypoint сохранен в /config/waypoints/кухня.json
```


#### Метод 2: Использование AprilTag для точной разметки

**Преимущество**: Можно заранее расставить постоянные AprilTag кубы и привязать к ним имена

**Процесс**:
1. Расставить постоянные AprilTag кубы (ID 0-99)
2. Во время картографии RTABMAP запомнит их позиции
3. Привязать имена к AprilTag ID

**Конфигурация**:
```yaml
# /config/waypoints/apriltag_mapping.yaml
permanent_landmarks:
  0:
    name: "дом"
    description: "Домашняя позиция"
    offset_x: 0.0  # Смещение от тега до точки навигации
    offset_y: 0.0
    
  1:
    name: "кухня"
    description: "Кухня"
    offset_x: 0.5  # Точка навигации на 0.5м от тега
    offset_y: 0.0
    
  3:
    name: "зарядная станция"
    description: "Charging dock"
    offset_x: -0.3  # Подъезжать задом к станции
    offset_y: 0.0
```

**Преобразование**: AprilTag позиция → Waypoint

```python
def apriltag_to_waypoint(tag_pose, offset_x, offset_y):
    """Вычислить позицию waypoint от позиции AprilTag"""
    import numpy as np
    from tf_transformations import quaternion_matrix
    
    # Матрица трансформации тега
    q = [tag_pose.orientation.x, tag_pose.orientation.y,
         tag_pose.orientation.z, tag_pose.orientation.w]
    T = quaternion_matrix(q)
    T[0:3, 3] = [tag_pose.position.x, tag_pose.position.y, tag_pose.position.z]
    
    # Смещение в системе координат тега
    offset = np.array([offset_x, offset_y, 0, 1])
    
    # Waypoint в мировых координатах
    waypoint_pos = T @ offset
    
    return {
        'x': waypoint_pos[0],
        'y': waypoint_pos[1],
        'theta': euler_from_quaternion(q)[2]
    }
```

### 7.3. Хранение конфигурации waypoints

#### Вариант 1: Локальный файл (текущая реализация)

**Местоположение**: `src/rob_box_voice/rob_box_voice/command_node.py`

**Недостатки**:
- ❌ Хардкод в коде Python
- ❌ Требует пересборки Docker образа для изменения
- ❌ Нет синхронизации между роботами
- ❌ Нет UI для редактирования

#### Вариант 2: YAML файл (улучшение)

**Местоположение**: `docker/vision/config/waypoints/waypoints.yaml`

```yaml
waypoints:
  дом:
    x: 0.0
    y: 0.0
    theta: 0.0
    description: "Домашняя позиция"
    type: "home"
  
  кухня:
    x: 2.5
    y: 1.2
    theta: 0.0
    description: "Кухня"
    type: "delivery"
  
  зарядная_станция:
    x: -1.0
    y: 0.5
    theta: 3.14
    description: "Charging dock"
    type: "charging"
```

**Загрузка в command_node.py**:
```python
import yaml

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        
        # Загрузить waypoints из файла
        with open('/config/waypoints/waypoints.yaml', 'r') as f:
            config = yaml.safe_load(f)
            self.waypoints = config['waypoints']
        
        self.get_logger().info(f'Загружено {len(self.waypoints)} waypoints')
```

**Преимущества**:
- ✅ Не требует пересборки образа
- ✅ Можно редактировать прямо на роботе
- ✅ Volume mounting в Docker

#### Вариант 3: Облачная база данных (рекомендуется для production)

**Архитектура**:
```
Облачный сервер (zenoh.robbox.online)
    ↓
  PostgreSQL БД
    ↓
Zenoh Topic: robots/{ROBOT_ID}/config/waypoints
    ↓
Робот (подписка на изменения)
```

**Схема БД**:
```sql
CREATE TABLE waypoints (
    id SERIAL PRIMARY KEY,
    robot_id VARCHAR(50),
    name VARCHAR(100),
    x FLOAT,
    y FLOAT,
    theta FLOAT,
    description TEXT,
    type VARCHAR(50),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_waypoints_robot ON waypoints(robot_id);
```

**Синхронизация**:
```python
class WaypointSync(Node):
    """Синхронизация waypoints с облаком через Zenoh"""
    
    def __init__(self):
        super().__init__('waypoint_sync')
        
        # Zenoh session
        import zenoh
        self.z_session = zenoh.open()
        
        # Подписка на обновления waypoints
        topic = f"robots/{os.getenv('ROBOT_ID')}/config/waypoints"
        self.z_sub = self.z_session.subscribe(topic, self.waypoint_update_callback)
        
        # Локальный кэш waypoints
        self.waypoints = {}
        
        # Запросить текущие waypoints при старте
        self.fetch_waypoints_from_cloud()
    
    def waypoint_update_callback(self, sample):
        """Обработка обновления waypoint из облака"""
        data = json.loads(sample.payload.decode('utf-8'))
        
        if data['action'] == 'add' or data['action'] == 'update':
            self.waypoints[data['name']] = {
                'x': data['x'],
                'y': data['y'],
                'theta': data['theta'],
                'description': data.get('description', ''),
                'type': data.get('type', 'general')
            }
            self.get_logger().info(f"✅ Waypoint '{data['name']}' обновлен")
        
        elif data['action'] == 'delete':
            if data['name'] in self.waypoints:
                del self.waypoints[data['name']]
                self.get_logger().info(f"🗑️ Waypoint '{data['name']}' удален")
        
        # Сохранить в локальный файл (кэш)
        self.save_waypoints_to_file()
```

---

## 8. Интеграция с голосовым управлением

### 8.1. Существующие команды

**Местоположение**: `src/rob_box_voice/rob_box_voice/command_node.py`

**Текущие паттерны навигации**:
```python
patterns = [
    (r'(двигайся|иди|поезжай|езжай|направляйся)\s+к\s+точке\s+(\d+)', 'waypoint_number'),
    (r'(двигайся|иди|поезжай|езжай)\s+к\s+(дом|кухня|гостиная)', 'waypoint_name'),
    (r'(стой|остановись|стоп)', 'stop'),
]
```

### 8.2. Расширенные команды для картографии

**Новые паттерны**:
```python
MAPPING_PATTERNS = {
    'start_mapping': [
        r'исследуй территорию',
        r'начни картографию',
        r'создай новую карту',
        r'начни исследование',
    ],
    
    'continue_mapping': [
        r'продолжи исследование',
        r'продолжи картографию',
        r'добавь к карте',
    ],
    
    'finish_mapping': [
        r'закончи исследование',
        r'завершить картографию',
        r'перейди в навигацию',
        r'карта готова',
    ],
    
    'save_waypoint': [
        r'сохрани точку (\w+)',
        r'запомни место (\w+)',
        r'добавь waypoint (\w+)',
    ],
    
    'go_to_waypoint': [
        r'поезжай (к|в|на) (\w+)',
        r'иди (к|в|на) (\w+)',
        r'направляйся (к|в|на) (\w+)',
    ],
}
```

### 8.3. Обработка команд картографии

**Workflow для "Исследуй территорию"**:
```python
async def handle_start_mapping(self, text):
    """Начать новую картографию"""
    
    # Запросить подтверждение
    self.pending_confirmation = {
        'action': 'start_mapping',
        'text': text
    }
    
    response = "Начать новое исследование? Текущая карта будет сохранена в резервную копию."
    
    # Опубликовать в TTS
    self.tts_pub.publish(String(data=response))
    
    # Ждать подтверждения ("да" / "начинай")
    return response

async def confirm_start_mapping(self):
    """Подтверждение: начать картографию"""
    
    # 1. Создать backup текущей карты
    await self.backup_rtabmap_db()
    
    # 2. Сбросить память RTABMAP
    await self.call_service('/rtabmap/reset_memory', Empty.Request())
    
    # 3. Переключить в mapping mode
    await self.call_service('/rtabmap/set_mode_mapping', Empty.Request())
    
    # 4. Подтвердить
    response = "Начинаю исследование. Старая карта сохранена."
    self.tts_pub.publish(String(data=response))
    
    self.pending_confirmation = None
```

**Workflow для "Сохрани точку"**:
```python
async def handle_save_waypoint(self, name):
    """Сохранить текущую позицию как waypoint"""
    
    # Получить текущую позицию из RTABMAP
    pose = await self.get_current_pose()
    
    if not pose:
        return "Не могу определить текущую позицию. Проверьте локализацию."
    
    # Создать waypoint
    waypoint = {
        'x': pose.position.x,
        'y': pose.position.y,
        'theta': self.quaternion_to_yaw(pose.orientation),
        'description': f"Сохранено голосовой командой",
        'timestamp': datetime.now().isoformat()
    }
    
    # Сохранить локально
    self.waypoints[name] = waypoint
    self.save_waypoints_to_file()
    
    # Опубликовать в облако (если включена синхронизация)
    if self.cloud_sync_enabled:
        await self.publish_waypoint_to_cloud(name, waypoint, action='add')
    
    response = f"Точка '{name}' сохранена. Координаты: x={waypoint['x']:.1f}, y={waypoint['y']:.1f}"
    return response
```

### 8.4. Диалоговые сценарии

**Сценарий 1: Создание карты с голосовым руководством**

```
Пользователь: "Исследуй территорию"
Робот: "Начать новое исследование? Текущая карта будет сохранена."
Пользователь: "Да"
Робот: "Начинаю исследование. Веди меня через помещение."
[Пользователь управляет роботом вручную или через waypoints]
Пользователь: "Сохрани точку кухня"
Робот: "Точка 'кухня' сохранена. Координаты: x=2.5, y=1.2"
[Продолжение исследования]
Пользователь: "Закончи исследование"
Робот: "Заканчиваю исследование. Временные маркеры очищены. Карта готова."
```

**Сценарий 2: Навигация по готовой карте**

```
Пользователь: "Поезжай на кухню"
Робот: "Строю маршрут на кухню."
[Робот движется]
Робот: "Прибыл на кухню."
```

---

## 9. Облачная конфигурация через Zenoh

### 9.1. Архитектура облачной системы

**Компоненты**:

```
┌─────────────────────────────────────────────────────────────┐
│  Веб-интерфейс (React SPA)                                  │
│  https://zenoh.robbox.online/                                │
│  - Управление waypoints                                      │
│  - Просмотр карт                                             │
│  - Мониторинг роботов                                        │
└──────────────────────────┬──────────────────────────────────┘
                           │ HTTPS/WebSocket
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Backend API (FastAPI)                                       │
│  - REST API для UI                                           │
│  - Zenoh bridge для робота                                   │
│  - PostgreSQL для хранения                                   │
└──────────────────────────┬──────────────────────────────────┘
                           │ Zenoh TCP
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Zenoh Cloud Router                                          │
│  zenoh.robbox.online:7447                                    │
│  - Центральный роутер                                        │
│  - Namespace: robots/{ROBOT_ID}                              │
└──────────────────────────┬──────────────────────────────────┘
                           │ Zenoh TCP (через WiFi)
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Rob Box Robot                                               │
│  Zenoh Session с namespace: robots/RBXU100001                │
│  - Подписка: robots/RBXU100001/config/**                     │
│  - Публикация: robots/RBXU100001/status/**                   │
└─────────────────────────────────────────────────────────────┘
```

### 9.2. Zenoh топики для конфигурации

**Waypoints управление**:
```
robots/{ROBOT_ID}/config/waypoints/add
robots/{ROBOT_ID}/config/waypoints/update
robots/{ROBOT_ID}/config/waypoints/delete
robots/{ROBOT_ID}/config/waypoints/list
```

**Mapping управление**:
```
robots/{ROBOT_ID}/mapping/start
robots/{ROBOT_ID}/mapping/stop
robots/{ROBOT_ID}/mapping/waypoints      # Coverage waypoints
robots/{ROBOT_ID}/mapping/progress       # Прогресс выполнения
```

**Статус робота**:
```
robots/{ROBOT_ID}/status/pose            # Текущая позиция
robots/{ROBOT_ID}/status/battery         # Уровень батареи
robots/{ROBOT_ID}/status/mapping_mode    # mapping/localization
```

### 9.3. Веб-интерфейс для управления waypoints

**Функции UI**:

1. **Список waypoints**:
   - Таблица всех waypoints
   - Колонки: Имя, X, Y, Theta, Описание, Тип
   - Сортировка, фильтрация, поиск

2. **Добавление waypoint**:
   - Форма: Имя, Описание, Тип
   - Кнопка "Записать текущую позицию робота"
   - Или ручной ввод координат

3. **Редактирование waypoint**:
   - Изменение координат
   - Изменение описания/типа
   - Синхронизация с роботом

4. **Удаление waypoint**:
   - Подтверждение удаления
   - Удаление из БД и с робота

5. **Визуализация на карте**:
   - Отображение всех waypoints на карте
   - Показать текущую позицию робота
   - Маршрут между waypoints

### 9.4. Backend API для waypoints

**FastAPI endpoints**:

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import zenoh

app = FastAPI()

class Waypoint(BaseModel):
    name: str
    x: float
    y: float
    theta: float
    description: str = ""
    type: str = "general"

# Zenoh session
z_session = zenoh.open(config={
    "connect": {"endpoints": ["tcp/0.0.0.0:7447"]},
    "mode": "client"
})

@app.post("/api/robots/{robot_id}/waypoints")
async def add_waypoint(robot_id: str, waypoint: Waypoint):
    """Добавить новый waypoint"""
    
    # Сохранить в БД
    db_waypoint = await db.save_waypoint(robot_id, waypoint)
    
    # Опубликовать в Zenoh
    topic = f"robots/{robot_id}/config/waypoints/add"
    payload = waypoint.json()
    z_session.put(topic, payload)
    
    return {"status": "success", "waypoint": db_waypoint}

@app.put("/api/robots/{robot_id}/waypoints/{name}")
async def update_waypoint(robot_id: str, name: str, waypoint: Waypoint):
    """Обновить waypoint"""
    
    # Обновить в БД
    db_waypoint = await db.update_waypoint(robot_id, name, waypoint)
    
    # Опубликовать в Zenoh
    topic = f"robots/{robot_id}/config/waypoints/update"
    payload = waypoint.json()
    z_session.put(topic, payload)
    
    return {"status": "success", "waypoint": db_waypoint}

@app.delete("/api/robots/{robot_id}/waypoints/{name}")
async def delete_waypoint(robot_id: str, name: str):
    """Удалить waypoint"""
    
    # Удалить из БД
    await db.delete_waypoint(robot_id, name)
    
    # Опубликовать в Zenoh
    topic = f"robots/{robot_id}/config/waypoints/delete"
    payload = json.dumps({"name": name})
    z_session.put(topic, payload)
    
    return {"status": "success"}

@app.get("/api/robots/{robot_id}/waypoints")
async def list_waypoints(robot_id: str):
    """Получить список всех waypoints"""
    waypoints = await db.get_waypoints(robot_id)
    return {"waypoints": waypoints}
```

---

## 10. Рекомендации по реализации

### 10.1. Приоритетность задач

**Фаза 1: Базовая функциональность** (2-3 недели)
- ✅ Уже реализовано: RTABMAP, Nav2, голосовые команды навигации
- [ ] Реализовать команды картографии ("Исследуй территорию", "Закончи исследование")
- [ ] YAML файл для waypoints
- [ ] Скрипт записи waypoints

**Фаза 2: AprilTag интеграция** (1-2 недели)
- [ ] Печать и изготовление AprilTag кубов (10-20 шт)
- [ ] Тестирование детекции AprilTag с OAK-D
- [ ] Интеграция AprilTag в RTABMAP
- [ ] Скрипт очистки временных маркеров

**Фаза 3: Автономное исследование** (3-4 недели)
- [ ] Исследовать пакеты: explore_lite, frontier_exploration
- [ ] Интеграция с Nav2
- [ ] Тестирование в реальных условиях
- [ ] Настройка параметров для Rob Box

**Фаза 4: Веб-интерфейс** (4-6 недель)
- [ ] Backend API (FastAPI + PostgreSQL)
- [ ] Frontend UI (React)
- [ ] Zenoh bridge
- [ ] Интерфейс управления waypoints
- [ ] Интерфейс выбора области картографии

### 10.2. Технические рекомендации

**RTABMAP**:
- Использовать режим "mapping" только при картографии
- Переключаться в "localization" для навигации
- Делать backup БД перед созданием новой карты
- Периодически оптимизировать БД (удалять старые сессии)

**AprilTag**:
- Размер тегов: 16 см (как в конфиге)
- Качество печати: 300+ DPI
- Освещение: избегать прямого солнечного света
- Высота размещения: 30-50 см от пола для OAK-D
- ID 0-99: постоянные, ID 100-199: временные

**Навигация**:
- Скорость при картографии: 0.2-0.3 м/с
- Скорость при навигации: 0.4-0.5 м/с
- Безопасное расстояние до препятствий: 20-30 см
- Точность достижения waypoint: 15 см (xy_goal_tolerance)

**Облачная система**:
- Использовать Zenoh namespace для изоляции роботов
- Синхронизировать waypoints через Zenoh
- Кэшировать конфигурацию локально на роботе
- Graceful fallback если облако недоступно

### 10.3. Метрики качества картографии

**Оценка качества карты**:

```python
def evaluate_map_quality(rtabmap_db_path):
    """Оценить качество созданной карты"""
    
    metrics = {
        'num_nodes': 0,          # Количество узлов в графе
        'num_loop_closures': 0,  # Количество loop closures
        'map_size_mb': 0,        # Размер БД
        'coverage_area_m2': 0,   # Покрытая площадь
        'avg_uncertainty': 0,    # Средняя неопределенность позиций
    }
    
    # Подключиться к БД
    conn = sqlite3.connect(rtabmap_db_path)
    cursor = conn.cursor()
    
    # Количество узлов
    cursor.execute("SELECT COUNT(*) FROM Node")
    metrics['num_nodes'] = cursor.fetchone()[0]
    
    # Количество loop closures (тип связи = 1)
    cursor.execute("SELECT COUNT(*) FROM Link WHERE type = 1")
    metrics['num_loop_closures'] = cursor.fetchone()[0]
    
    # Размер файла
    metrics['map_size_mb'] = os.path.getsize(rtabmap_db_path) / (1024 * 1024)
    
    conn.close()
    
    return metrics

# Пример вывода:
# {
#   'num_nodes': 1523,
#   'num_loop_closures': 87,
#   'map_size_mb': 245.3,
#   'coverage_area_m2': 120.5,
#   'avg_uncertainty': 0.05
# }
```

**Критерии хорошей карты**:
- ✅ Loop closures: минимум 1 на каждые 20-30 узлов
- ✅ Средняя неопределенность: < 0.1 метра
- ✅ Нет "разрывов" в карте
- ✅ Все области покрыты равномерно

---

## 11. Заключение

Данное исследование предоставляет комплексный подход к картографированию для робота Rob Box, охватывающий:

- 🗺️ **Оптимальные процедуры SLAM** с RTABMAP
- 🤖 **Автономное исследование** через frontier exploration и coverage planning
- 🎯 **Повышение точности** с помощью AprilTag маркеров
- 🔧 **Управление конфигурацией** через облачную систему
- 🗣️ **Голосовое управление** для удобного взаимодействия

Реализация предложенных решений позволит:
- Упростить процесс создания карт
- Повысить точность локализации
- Обеспечить гибкое управление waypoints
- Создать удобный пользовательский опыт

**Следующие шаги**:
1. Внедрение команд картографии в голосовой ассистент
2. Изготовление и тестирование AprilTag кубов
3. Разработка веб-интерфейса для управления
4. Тестирование в реальных условиях эксплуатации

---

**Ссылки на документацию проекта**:
- [MAPPING_COMMANDS.md](../packages/MAPPING_COMMANDS.md) - Спецификация голосовых команд
- [NAV2_SETUP.md](NAV2_SETUP.md) - Настройка навигационного стека
- [ZENOH_CLOUD_NAMESPACES.md](../architecture/ZENOH_CLOUD_NAMESPACES.md) - Облачная архитектура
- [SYSTEM_OVERVIEW.md](../architecture/SYSTEM_OVERVIEW.md) - Общая архитектура системы

---

**Дата завершения**: 24 октября 2025  
**Версия**: 1.0  
**Автор**: Rob Box Development Team

