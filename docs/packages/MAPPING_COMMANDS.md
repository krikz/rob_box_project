# Голосовые команды управления картографией РОББОКС

## 📋 Обзор

Система голосовых команд для управления процессом SLAM картографии через RTABMap. Робот может начинать новое исследование территории, продолжать существующую карту или переключаться в режим локализации для навигации.

## 🎯 Команды

### 1. "Исследуй территорию" (Start Mapping)

**Описание**: Начать новое исследование территории, создать новую карту

**Workflow**:
```
1. Пользователь: "Исследуй территорию"
2. Робот: "Начать новое исследование? Старая карта будет сохранена в резервную копию."
3. Пользователь: "Да" / "Начинай"
4. Робот:
   - Остановить rtabmap контейнер
   - Сохранить текущую БД: mv rtabmap.db → rtabmap_backup_<timestamp>.db
   - Запустить rtabmap с параметром --delete_db_on_start
   - Подтвердить: "Начинаю исследование. Старая карта сохранена."
```

**Технические детали**:
- **Режим**: SLAM (Mapping)
- **Параметр**: `Mem/IncrementalMemory: true` (включено по умолчанию)
- **База данных**: Создаётся новая `/maps/rtabmap.db`
- **Backup**: `/maps/backups/rtabmap_backup_YYYY-MM-DD_HH-MM-SS.db`

**ROS2 сервисы для управления**:
```bash
# Сброс БД и переход в SLAM mode
ros2 service call /rtabmap/reset_memory std_srvs/srv/Empty

# ИЛИ через Docker restart с --delete_db_on_start
docker stop rtabmap
mv /path/to/maps/rtabmap.db /path/to/maps/backups/rtabmap_backup_$(date +%Y-%m-%d_%H-%M-%S).db
docker start rtabmap  # с --delete_db_on_start в args
```

---

### 2. "Продолжи исследование" (Continue Mapping)

**Описание**: Продолжить картографию, открыть новую сессию с существующей картой

**Workflow**:
```
1. Пользователь: "Продолжи исследование"
2. Робот: "Продолжаю исследование территории. Добавляю новые области к существующей карте."
3. Робот:
   - Убедиться что rtabmap в SLAM mode
   - Убедиться что Mem/IncrementalMemory: true
   - Не удалять БД при старте (без --delete_db_on_start)
```

**Технические детали**:
- **Режим**: SLAM (Mapping) - продолжение
- **Параметр**: `Mem/IncrementalMemory: true`
- **База данных**: Используется существующая `/maps/rtabmap.db`
- **Сессия**: RTABMap автоматически создаёт новую сессию при перезапуске

**ROS2 команды**:
```bash
# Проверка режима
ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory
# Output: Boolean value is: true

# Если в localization mode - переключить в mapping
ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty
```

---

### 3. "Закончи исследование" (Finish Mapping → Localization)

**Описание**: Закончить картографию, переключиться в режим локализации для навигации

**Workflow**:
```
1. Пользователь: "Закончи исследование"
2. Робот: "Заканчиваю исследование. Переключаюсь в режим навигации."
3. Робот:
   - Переключить RTABMap в Localization mode
   - Сохранить текущую БД (автоматически)
   - Mem/IncrementalMemory: false
   - Робот будет локализоваться на существующей карте
```

**Технические детали**:
- **Режим**: Localization (только локализация, карта НЕ обновляется)
- **Параметр**: `Mem/IncrementalMemory: false`
- **База данных**: `/maps/rtabmap.db` (read-only для локализации)
- **Использование**: Для автономной навигации по готовой карте

**ROS2 сервисы**:
```bash
# Переключение в Localization mode
ros2 service call /rtabmap/set_mode_localization std_srvs/srv/Empty

# Проверка режима
ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory
# Output: Boolean value is: false (localization mode)

# Вернуться в Mapping mode
ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty
```

---

## 🔧 RTABMap режимы работы

### SLAM Mode (Mapping)
```yaml
Mem/IncrementalMemory: true
```
- **Назначение**: Создание и обновление карты
- **Поведение**: 
  - Добавляет новые узлы (nodes) в граф карты
  - Выполняет loop closure detection
  - Обновляет карту в реальном времени
  - Создаёт новые landmarks
- **База данных**: Read-Write

### Localization Mode
```yaml
Mem/IncrementalMemory: false
```
- **Назначение**: Локализация на готовой карте
- **Поведение**:
  - Использует существующую карту
  - НЕ добавляет новые узлы
  - Только определяет позицию робота
  - Быстрее чем SLAM mode
- **База данных**: Read-Only (для навигации)

---

## 📂 Структура базы данных

```
/maps/
├── rtabmap.db              # Текущая активная БД
└── backups/
    ├── rtabmap_backup_2025-10-18_10-30-00.db
    ├── rtabmap_backup_2025-10-18_14-20-15.db
    └── ...
```

---

## 🤖 Интеграция с Dialogue Node

### Добавить Intent Patterns:

```python
# В dialogue_node.py

MAPPING_INTENTS = {
    'start_mapping': [
        r'исследуй территорию',
        r'начни исследование',
        r'создай новую карту',
        r'начни картографию',
        r'новая карта',
    ],
    'continue_mapping': [
        r'продолжи исследование',
        r'продолжить картографию',
        r'продолжай карту',
        r'добавь к карте',
    ],
    'finish_mapping': [
        r'закончи исследование',
        r'завершить картографию',
        r'перейди в навигацию',
        r'режим локализации',
        r'карта готова',
    ],
}
```

### Добавить ROS2 Service Publishers:

```python
from std_srvs.srv import Empty
from example_interfaces.srv import SetBool

class DialogueNode(Node):
    def __init__(self):
        super().__init__('dialogue_node')
        
        # RTABMap control services
        self.reset_memory_client = self.create_client(Empty, '/rtabmap/reset_memory')
        self.set_mode_mapping_client = self.create_client(Empty, '/rtabmap/set_mode_mapping')
        self.set_mode_localization_client = self.create_client(Empty, '/rtabmap/set_mode_localization')
        
        # Для подтверждения операций
        self.pending_confirmation = None
```

### Handler для команд:

```python
async def handle_mapping_command(self, intent, text):
    """Обработка команд картографии"""
    
    if intent == 'start_mapping':
        # Запросить подтверждение
        self.pending_confirmation = 'start_mapping'
        return "Начать новое исследование? Старая карта будет сохранена в резервную копию."
    
    elif intent == 'continue_mapping':
        # Проверить режим
        is_mapping = await self.check_mapping_mode()
        if not is_mapping:
            await self.call_service(self.set_mode_mapping_client)
            return "Продолжаю исследование территории. Добавляю новые области к карте."
        else:
            return "Исследование уже активно. Продолжаю картографию."
    
    elif intent == 'finish_mapping':
        # Переключить в localization
        await self.call_service(self.set_mode_localization_client)
        return "Заканчиваю исследование. Переключаюсь в режим навигации по карте."
```

---

## 🔄 Workflow для backup

### Docker volume backup script:

```bash
#!/bin/bash
# /maps/backup_rtabmap.sh

BACKUP_DIR="/maps/backups"
mkdir -p "$BACKUP_DIR"

TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
DB_FILE="/maps/rtabmap.db"

if [ -f "$DB_FILE" ]; then
    cp "$DB_FILE" "$BACKUP_DIR/rtabmap_backup_${TIMESTAMP}.db"
    echo "✅ Backup created: rtabmap_backup_${TIMESTAMP}.db"
    
    # Удалить старые backup (старше 30 дней)
    find "$BACKUP_DIR" -name "rtabmap_backup_*.db" -mtime +30 -delete
    echo "✅ Old backups cleaned"
else
    echo "❌ Database file not found: $DB_FILE"
    exit 1
fi
```

### Вызов из Python:

```python
import subprocess
from datetime import datetime

async def backup_rtabmap_db(self):
    """Создать backup текущей БД RTABMap"""
    try:
        # Через Docker exec
        result = subprocess.run([
            'docker', 'exec', 'rtabmap',
            'bash', '-c',
            '/maps/backup_rtabmap.sh'
        ], capture_output=True, text=True, check=True)
        
        self.get_logger().info(f"✅ RTABMap backup: {result.stdout}")
        return True
    except subprocess.CalledProcessError as e:
        self.get_logger().error(f"❌ Backup failed: {e.stderr}")
        return False
```

---

## 🧪 Тестирование

### Проверка RTABMap сервисов:

```bash
# 1. Проверить доступные сервисы
ros2 service list | grep rtabmap

# Ожидаемые сервисы:
# /rtabmap/reset_memory
# /rtabmap/set_mode_mapping
# /rtabmap/set_mode_localization
# /rtabmap/pause
# /rtabmap/resume

# 2. Проверить текущий режим
ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory

# 3. Переключение режимов вручную
ros2 service call /rtabmap/set_mode_localization std_srvs/srv/Empty
ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty

# 4. Сброс памяти (новая карта)
ros2 service call /rtabmap/reset_memory std_srvs/srv/Empty
```

### Проверка базы данных:

```bash
# Размер БД
docker exec rtabmap ls -lh /maps/rtabmap.db

# Количество nodes в БД
docker exec rtabmap bash -c "sqlite3 /maps/rtabmap.db 'SELECT COUNT(*) FROM Node;'"

# Последняя модификация
docker exec rtabmap stat /maps/rtabmap.db
```

---

## 📊 Мониторинг режима

### Топик для статуса RTABMap:

```bash
# Подписаться на статистику
ros2 topic echo /rtabmap/info --once

# Проверить режим (Mem/IncrementalMemory в stats)
ros2 topic echo /rtabmap/info | grep -A5 "Mem/IncrementalMemory"
```

### Визуализация в Reflection Node:

```python
# В reflection_node.py - добавить подписку на /rtabmap/info

class ReflectionNode(Node):
    def __init__(self):
        super().__init__('reflection_node')
        
        self.rtabmap_info_sub = self.create_subscription(
            String,  # или rtabmap_msgs.msg.Info
            '/rtabmap/info',
            self.rtabmap_info_callback,
            10
        )
        
        self.current_mapping_mode = None  # 'mapping' or 'localization'
    
    def rtabmap_info_callback(self, msg):
        """Отслеживание режима RTABMap для контекста"""
        # Парсить info и определить режим
        # Сохранять в self.current_mapping_mode
        # Использовать в generate_thought() для контекста
        pass
```

---

## 🚀 Порядок реализации

### Этап 1: RTABMap Services (проверка доступности)
- [ ] Проверить наличие сервисов в rtabmap контейнере
- [ ] Протестировать переключение режимов вручную
- [ ] Убедиться что reset_memory работает

### Этап 2: Backup система
- [ ] Создать скрипт backup_rtabmap.sh
- [ ] Добавить в Docker volume: `./scripts/rtabmap:/scripts:ro`
- [ ] Протестировать backup через Docker exec

### Этап 3: Dialogue Node integration
- [ ] Добавить intent patterns для 3 команд
- [ ] Создать ROS2 service clients
- [ ] Реализовать handler для mapping commands
- [ ] Добавить систему подтверждения (для start_mapping)

### Этап 4: Reflection Node awareness
- [ ] Подписаться на /rtabmap/info
- [ ] Отслеживать текущий режим картографии
- [ ] Добавить в контекст для рефлексии

### Этап 5: Testing
- [ ] Локальное тестирование сервисов
- [ ] Проверка backup/restore
- [ ] Голосовые команды end-to-end
- [ ] Проверка переключения режимов

---

## 📝 Примеры использования

### Сценарий 1: Первое исследование
```
Пользователь: "Исследуй территорию"
Робот: "Начать новое исследование? Старая карта будет сохранена в резервную копию."
Пользователь: "Да"
Робот: "Начинаю исследование. Старая карта сохранена."
[Робот движется, создаёт карту]
```

### Сценарий 2: Продолжение после перезагрузки
```
Пользователь: "Продолжи исследование"
Робот: "Продолжаю исследование территории. Добавляю новые области к карте."
[RTABMap создаёт новую сессию, но использует существующую БД]
```

### Сценарий 3: Переход к навигации
```
Пользователь: "Закончи исследование"
Робот: "Заканчиваю исследование. Переключаюсь в режим навигации по карте."
[RTABMap переходит в localization mode]
Пользователь: "Поезжай к зарядной станции"
Робот: "Строю маршрут к зарядной станции."
[Nav2 использует готовую карту для навигации]
```

---

## 🔗 Дополнительные ресурсы

- [RTABMap ROS2 Wiki](http://wiki.ros.org/rtabmap_ros)
- [RTABMap Services Documentation](http://wiki.ros.org/rtabmap_ros#Services)
- [RTABMap Database Format](https://github.com/introlab/rtabmap/wiki/Database-structure)
- [Проект: docker/main/docker-compose.yaml](../../docker/main/docker-compose.yaml)
- [Проект: параметры rtabmap — docker/main/scripts/rtabmap/patch_rtabmap_launch.py](../../docker/main/scripts/rtabmap/patch_rtabmap_launch.py)

---

## ⚠️ Важные замечания

1. **Безопасность**: Всегда делать backup перед `reset_memory`
2. **Производительность**: Localization mode быстрее SLAM mode
3. **База данных**: При больших картах БД может занимать >1GB
4. **Backup retention**: Автоматически удалять backup старше 30 дней
5. **Подтверждение**: Обязательно запрашивать подтверждение для `start_mapping`

---

**Статус**: 📋 Документация для реализации  
**Приоритет**: СРЕДНИЙ (после завершения sound effects)  
**Сложность**: СРЕДНЯЯ (интеграция с RTABMap services)
