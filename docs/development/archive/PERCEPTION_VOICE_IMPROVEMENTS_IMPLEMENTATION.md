# Реализация Улучшений Perception и Voice Assistant
## Руководство по внедрению новых компонентов

**Дата:** 21 октября 2025  
**Версия:** 1.0  
**Связанный документ:** [PERCEPTION_VOICE_COMPREHENSIVE_REVIEW.md](./PERCEPTION_VOICE_COMPREHENSIVE_REVIEW.md)

---

## 📦 Созданные компоненты

### 1. Обновлённый PerceptionEvent.msg

**Файл:** `src/rob_box_perception_msgs/msg/PerceptionEvent.msg`

**Добавлены поля:**
```msg
# Time awareness (current time context)
string current_time_human     # Human-readable time: "2025-10-21 12:30:00"
string time_period            # "morning", "day", "evening", "night"
string time_context_json      # Full time context as JSON

# Internet connectivity
bool internet_available

# ROS2 nodes availability
string[] active_nodes
string[] failed_nodes
string[] missing_nodes

# Equipment health (cameras, LIDAR, motors, SLAM)
string equipment_summary_json
```

### 2. NodeAvailabilityMonitor

**Файл:** `src/rob_box_perception/rob_box_perception/utils/node_monitor.py`

**Описание:** Мониторинг доступности ROS2 нод через `ros2 node list`

**Использование:**
```python
from rob_box_perception.utils.node_monitor import NodeAvailabilityMonitor

# В конструкторе ноды
self.node_monitor = NodeAvailabilityMonitor(
    self,
    expected_nodes=[
        '/audio_node',
        '/stt_node',
        '/dialogue_node',
        '/reflection_node',
        '/context_aggregator',
        '/oak_d_node',
        '/lslidar_node',
        '/rtabmap'
    ]
)

# Получить статус
summary = self.node_monitor.get_status_summary()
active_nodes = self.node_monitor.get_active_nodes()
failed_nodes = self.node_monitor.get_failed_nodes()
missing_nodes = self.node_monitor.get_missing_nodes()
```

**Функции:**
- Автоматическая проверка каждые 5 секунд
- Детекция упавших нод (были active, стали failed)
- Детекция отсутствующих нод (missing с начала)
- Логирование изменений статуса

### 3. InternetConnectivityMonitor

**Файл:** `src/rob_box_perception/rob_box_perception/utils/internet_monitor.py`

**Описание:** Мониторинг доступности интернета через ping

**Использование:**
```python
from rob_box_perception.utils.internet_monitor import InternetConnectivityMonitor

# В конструкторе ноды
self.internet_monitor = InternetConnectivityMonitor(
    self,
    check_interval=30.0  # Проверка каждые 30 секунд
)

# Получить статус
status = self.internet_monitor.get_status()
is_online = status['is_online']
```

**Функции:**
- Ping к Google DNS (8.8.8.8) и Cloudflare DNS (1.1.1.1)
- Настраиваемый интервал проверки
- Автоматическое логирование изменений статуса
- Timeout-защита (3 секунды на проверку)

### 4. TimeAwarenessProvider

**Файл:** `src/rob_box_perception/rob_box_perception/utils/time_provider.py`

**Описание:** Провайдер осознания текущего времени и контекста времени суток

**Использование:**
```python
from rob_box_perception.utils.time_provider import TimeAwarenessProvider

# В конструкторе ноды
self.time_provider = TimeAwarenessProvider(timezone='Europe/Moscow')

# Получить контекст времени
time_context = self.time_provider.get_current_time_context()
# {
#   'timestamp': 1729509600.0,
#   'datetime': '2025-10-21T12:30:00+03:00',
#   'human_readable': '2025-10-21 12:30:00',
#   'time_only': '12:30',
#   'date_only': '2025-10-21',
#   'hour': 12,
#   'minute': 30,
#   'weekday': 'Monday',
#   'weekday_ru': 'Понедельник',
#   'period': 'day',
#   'period_ru': 'день',
#   'timezone': 'Europe/Moscow'
# }
```

**Функции:**
- Определение периода суток (утро/день/вечер/ночь)
- Human-readable форматирование
- Поддержка часовых поясов (через pytz)
- Русская локализация дней недели и периодов

---

## 🔧 Интеграция в context_aggregator_node

### Шаг 1: Импорты

Добавить в начало `context_aggregator_node.py`:

```python
from rob_box_perception.utils.node_monitor import NodeAvailabilityMonitor
from rob_box_perception.utils.internet_monitor import InternetConnectivityMonitor
from rob_box_perception.utils.time_provider import TimeAwarenessProvider
```

### Шаг 2: Инициализация в конструкторе

```python
class ContextAggregatorNode(Node):
    def __init__(self):
        super().__init__('context_aggregator')
        
        # ... существующий код ...
        
        # ============ Новые мониторы ============
        
        # Node availability monitor
        self.node_monitor = NodeAvailabilityMonitor(self)
        
        # Internet connectivity monitor
        self.internet_monitor = InternetConnectivityMonitor(self, check_interval=30.0)
        
        # Time awareness provider
        self.time_provider = TimeAwarenessProvider(timezone='Europe/Moscow')
        
        self.get_logger().info('✅ Monitoring components initialized')
```

### Шаг 3: Обновление publish_event()

```python
def publish_event(self):
    """Публикация PerceptionEvent с агрегированным контекстом"""
    if not self.event_pub:
        return
    
    # Проверка здоровья
    health_status, health_issues = self.check_system_health()
    
    # Создаём событие
    event = PerceptionEvent()
    event.stamp = self.get_clock().now().to_msg()
    
    # ... существующие поля ...
    
    # ============ НОВЫЕ ПОЛЯ ============
    
    # Time context
    time_context = self.time_provider.get_current_time_context()
    event.current_time_human = time_context['human_readable']
    event.time_period = time_context['period']
    event.time_context_json = json.dumps(time_context, ensure_ascii=False)
    
    # Internet connectivity
    event.internet_available = self.internet_monitor.get_status()['is_online']
    
    # Node availability
    node_summary = self.node_monitor.get_status_summary()
    event.active_nodes = node_summary['active_list']
    event.failed_nodes = node_summary['failed_list']
    event.missing_nodes = node_summary['missing_list']
    
    # Добавить в health_issues если есть проблемы с нодами
    if node_summary['failed']:
        health_issues.extend([f"Нода упала: {n}" for n in node_summary['failed_list']])
    if node_summary['missing']:
        health_issues.extend([f"Нода отсутствует: {n}" for n in node_summary['missing_list']])
    
    # Добавить в health_issues если нет интернета
    if not event.internet_available:
        health_issues.append("Нет подключения к интернету")
    
    # Equipment summary (пока заглушка, реализовать позже)
    event.equipment_summary_json = "{}"
    
    # System health (обновлённый с учётом новых проблем)
    event.system_health_status = health_status
    event.health_issues = health_issues
    
    # ... остальные поля ...
    
    # Публикуем
    self.event_pub.publish(event)
```

### Шаг 4: Обновление check_system_health()

```python
def check_system_health(self) -> tuple[str, List[str]]:
    """Проверка здоровья системы"""
    issues = []
    
    # Проверка ошибок
    recent_error_count = len([e for e in self.recent_errors if time.time() - e['time'] < 30])
    if recent_error_count >= 5:
        issues.append(f'Много ошибок: {recent_error_count} за 30 сек')
    
    # Проверка батареи
    battery = self.current_sensors.get('battery', 100.0)
    if battery > 0 and battery < 11.0:
        issues.append(f'Низкая батарея: {battery:.1f}V')
    
    # Проверка нод (добавлено)
    node_summary = self.node_monitor.get_status_summary()
    if node_summary['failed']:
        issues.append(f"Упавшие ноды: {node_summary['failed']}")
    if node_summary['missing'] and node_summary['missing'] > 2:
        issues.append(f"Отсутствуют ноды: {node_summary['missing']}")
    
    # Проверка интернета (добавлено)
    if not self.internet_monitor.get_status()['is_online']:
        issues.append("Нет интернета")
    
    # Определяем статус
    if len(issues) == 0:
        status = "healthy"
    elif len(issues) <= 2:
        status = "degraded"
    else:
        status = "critical"
    
    return status, issues
```

---

## 🧠 Интеграция в reflection_node

### Обновление промптов

Обновить `reflection_prompt.txt` для использования новых полей:

```
Ты - внутренний голос робота РобБокс.

ТЕКУЩЕЕ ВРЕМЯ И КОНТЕКСТ:
Время: {current_time}
Период суток: {time_period_ru} ({time_period})

СОСТОЯНИЕ ПОДКЛЮЧЕНИЙ:
Интернет: {internet_status}
Активных нод: {active_nodes_count}/{total_nodes_count}

СИСТЕМНАЯ ИНФОРМАЦИЯ:
[Используй новые поля из PerceptionEvent]

...
```

### Обновление _format_context_for_prompt()

```python
def _format_context_for_prompt(self, ctx: PerceptionEvent) -> str:
    """Полный контекст для обычного размышления"""
    lines = ["=== ТЕКУЩИЙ КОНТЕКСТ РОБОТА ===", ""]
    
    # ============ НОВЫЕ СЕКЦИИ ============
    
    # Time context
    if ctx.current_time_human:
        lines.append("=== ВРЕМЯ ===")
        lines.append(f"🕐 Время: {ctx.current_time_human}")
        lines.append(f"📅 Период: {ctx.time_period}")
        
        # Парсим JSON для более детальной информации
        try:
            time_data = json.loads(ctx.time_context_json)
            lines.append(f"   {time_data['weekday_ru']}, {time_data['period_ru']}")
        except:
            pass
        lines.append("")
    
    # Internet status
    internet_emoji = "✅" if ctx.internet_available else "❌"
    internet_text = "Доступен" if ctx.internet_available else "Недоступен"
    lines.append(f"🌐 Интернет: {internet_emoji} {internet_text}")
    lines.append("")
    
    # Node status
    if ctx.active_nodes or ctx.failed_nodes or ctx.missing_nodes:
        lines.append("=== СОСТОЯНИЕ НОД ===")
        lines.append(f"✅ Активных: {len(ctx.active_nodes)}")
        
        if ctx.failed_nodes:
            lines.append(f"❌ Упавших: {len(ctx.failed_nodes)}")
            for node in ctx.failed_nodes:
                lines.append(f"   • {node}")
        
        if ctx.missing_nodes:
            lines.append(f"⚠️  Отсутствуют: {len(ctx.missing_nodes)}")
            for node in ctx.missing_nodes:
                lines.append(f"   • {node}")
        
        lines.append("")
    
    # ... существующие секции (Vision, Pose, Movement, etc.) ...
    
    return '\n'.join(lines)
```

---

## 🗣️ Интеграция в dialogue_node

### Обновление system_prompt

Обновить `master_prompt_simple.txt` для учёта нового контекста:

```
Ты ROBBOX - мобильный робот-ассистент.

ВАЖНАЯ ИНФОРМАЦИЯ О СИСТЕМЕ:
- Если интернет недоступен, используй короткие ответы и избегай сложных запросов
- Учитывай время суток в ответах (утром - "Доброе утро!", вечером - "Добрый вечер!")
- Если есть проблемы с нодами, сообщи об этом пользователю

КОНТЕКСТ из perception system доступен через reflection_node.

...
```

### Fallback при отсутствии интернета

Добавить проверку в `stt_callback()`:

```python
def stt_callback(self, msg: String):
    """Обработка распознанной речи с State Machine"""
    user_message = msg.data.strip()
    if not user_message:
        return
    
    # ... существующая логика ...
    
    # ============ Проверка доступности интернета ============
    # Можно подписаться на /perception/context_update для получения статуса
    # Или создать service call к context_aggregator
    
    if not self._is_internet_available():
        # Fallback: локальные ответы
        self.get_logger().warn('⚠️ Интернет недоступен - используем fallback')
        fallback_response = self._generate_fallback_response(user_message)
        self._speak_simple(fallback_response)
        return
    
    # ... обычная обработка с DeepSeek ...
```

---

## 📊 Тестирование компонентов

### Тест NodeAvailabilityMonitor

```bash
# Запустить context_aggregator
ros2 run rob_box_perception context_aggregator

# В другом терминале убить какую-то ноду
ros2 lifecycle set /stt_node shutdown

# Проверить логи context_aggregator - должно появиться:
# "❌ Нода упала: /stt_node"

# Перезапустить ноду
ros2 run rob_box_voice stt_node

# Проверить логи - должно появиться:
# "✅ Нода восстановлена: /stt_node"
```

### Тест InternetConnectivityMonitor

```bash
# Запустить context_aggregator
ros2 run rob_box_perception context_aggregator

# Отключить интернет на роботе
sudo ip link set wlan0 down

# Через 30 секунд в логах должно появиться:
# "⚠️ Интернет недоступен"

# Включить интернет обратно
sudo ip link set wlan0 up

# Через 30 секунд:
# "✅ Интернет восстановлен"
```

### Тест TimeAwarenessProvider

```bash
# Запустить context_aggregator
ros2 run rob_box_perception context_aggregator

# Проверить published event
ros2 topic echo /perception/context_update --once

# Должны увидеть:
# current_time_human: "2025-10-21 12:30:00"
# time_period: "day"
# time_context_json: "{...}"
```

---

## 🚀 Следующие шаги

### Немедленно

1. ✅ Пересобрать пакет `rob_box_perception_msgs`:
   ```bash
   cd /workspace
   colcon build --packages-select rob_box_perception_msgs
   source install/setup.bash
   ```

2. ✅ Интегрировать мониторы в `context_aggregator_node`

3. ✅ Обновить `reflection_node` для использования новых полей

4. ✅ Тестирование на реальном роботе

### Скоро

5. ⚪ Создать `EquipmentHealthMonitor` (камера, LIDAR, моторы, SLAM)

6. ⚪ Создать `ProactiveReflectionTimer` для reflection_node

7. ⚪ Создать service `/perception/get_full_context` для доступа к контексту

### Опционально

8. ⚪ Долгосрочная память (LongTermMemory с SQLite)

9. ⚪ NLU для детекции интентов

10. ⚪ ML для прогнозирования проблем

---

## 📝 Обновление документации

После реализации обновить:

1. `docs/architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md` - добавить новые компоненты
2. `docs/packages/README.md` - описание rob_box_perception утилит
3. `README.md` - обновить описание возможностей

---

## ⚠️ Известные проблемы и ограничения

### NodeAvailabilityMonitor

- Требует доступ к `ros2` CLI (может быть медленно на Raspberry Pi)
- Не проверяет состояние managed nodes (lifecycle)
- Не проверяет QoS настройки топиков

**Решение:** В будущем использовать ROS2 API вместо subprocess

### InternetConnectivityMonitor

- Ping может быть заблокирован firewall
- Не различает "нет интернета" и "нет DNS"
- Не проверяет доступность конкретных API (DeepSeek, Yandex)

**Решение:** Добавить проверку HTTP к конкретным API

### TimeAwarenessProvider

- Требует pytz для часовых поясов
- Не синхронизирует время через NTP автоматически

**Решение:** Документировать зависимость от pytz, рекомендовать NTP

---

**Статус:** Готово к интеграции (Stage 1 - Критичные улучшения)  
**Следующий этап:** Equipment Health Monitor + Proactive Reflection
