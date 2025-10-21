# Комплексное Ревью Perception Nodes и Voice Assistant
## Анализ и Рекомендации по Улучшению Внутреннего Диалога и Системного Мониторинга

**Дата:** 21 октября 2025  
**Автор:** AI Agent Analysis  
**Версия:** 1.0

---

## 📋 Содержание

1. [Обзор текущей архитектуры](#обзор-текущей-архитектуры)
2. [Лучшие практики из индустрии](#лучшие-практики-из-индустрии)
3. [Детальный анализ компонентов](#детальный-анализ-компонентов)
4. [Выявленные проблемы](#выявленные-проблемы)
5. [Предложения по улучшению](#предложения-по-улучшению)
6. [План реализации](#план-реализации)

---

## 🏗️ Обзор текущей архитектуры

### Компоненты системы

```
┌─────────────────────────────────────────────────────────────────┐
│                    PERCEPTION LAYER                              │
│  ┌─────────────────────┐  ┌──────────────────┐                 │
│  │ context_aggregator  │  │ health_monitor   │                 │
│  │  (MPC lite)         │  │                  │                 │
│  └─────────────────────┘  └──────────────────┘                 │
└─────────────────────────────────────────────────────────────────┘
              │                           │
              │ /perception/context_update │ /rosout
              │ /perception/user_speech    │
              ↓                           ↓
┌─────────────────────────────────────────────────────────────────┐
│                   DIALOGUE LAYER                                 │
│  ┌─────────────────┐  ┌──────────────────┐                     │
│  │ reflection_node │  │ dialogue_node    │                     │
│  │  (internal AI)  │  │  (user facing)   │                     │
│  └─────────────────┘  └──────────────────┘                     │
└─────────────────────────────────────────────────────────────────┘
              │                           │
              │ /voice/tts/request        │
              ↓                           ↓
┌─────────────────────────────────────────────────────────────────┐
│                     VOICE LAYER                                  │
│  ┌─────────────┐  ┌─────────┐  ┌─────────┐  ┌──────────┐      │
│  │ audio_node  │  │ stt_node│  │ tts_node│  │ led_node │      │
│  └─────────────┘  └─────────┘  └─────────┘  └──────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

### Текущие возможности

**✅ Реализовано:**
- Event-driven архитектура для reflection_node
- Агрегация контекста из множественных источников
- Суммаризация истории по типам (speech, vision, system, thoughts, responses)
- Мониторинг системных логов (/rosout) для ошибок и предупреждений
- Управление диалогом через state machine (IDLE/LISTENING/DIALOGUE/SILENCED)
- Детекция личных вопросов для срочных ответов
- Базовая проверка здоровья системы (health_status, battery)

**❌ Отсутствует:**
- Мониторинг доступности ROS2 нод
- Детекция доступности интернета
- Осознание текущего времени и суток
- Мониторинг состояния оборудования (камеры, LIDAR, моторы)
- Историческая база знаний о системе
- Прогнозирование проблем

---

## 🌐 Лучшие практики из индустрии

### 1. Context Aggregation (Агрегация Контекста)

**Из исследований:**
- **Hierarchical/Modular Approach**: Разбиение задач восприятия на мелкие модули, которые можно оптимизировать независимо
- **Sensor Fusion**: Интеграция данных с множественных сенсоров (камеры, LIDAR, микрофоны, IMU)
- **Real-time Processing**: Обработка событий в реальном времени, а не по таймеру

**Текущее состояние проекта:**
- ✅ Модульный подход реализован (отдельные ноды для разных источников)
- ✅ Event-driven архитектура (публикация на каждое обновление)
- ⚠️ Sensor fusion частично реализован (не все сенсоры интегрированы)

### 2. System Health Monitoring (ROS2)

**Best Practices:**
- **Node Lifecycle Management**: Использование managed nodes с lifecycle states (unconfigured, inactive, active, finalized)
- **System Monitoring Tools**: Утилиты типа `ros2sysmon` для мониторинга CPU, памяти, сети и нод
- **Fault Detection**: Автоматическое обнаружение сбоев нод и перезапуск
- **Diagnostics Publishing**: Публикация диагностики для визуализации в runtime

**Текущее состояние проекта:**
- ✅ Мониторинг логов через /rosout
- ❌ НЕТ lifecycle management для критичных нод
- ❌ НЕТ автоматического обнаружения сбоев нод
- ❌ НЕТ dedicated diagnostics publishing

### 3. Context Awareness (Осознание Контекста)

**Ключевые аспекты:**
- **Internet Connectivity Detection**: Регулярная проверка доступности интернета (ping, fallback mode)
- **Time Awareness**: RTC модуль + NTP синхронизация для точного времени
- **State Monitoring**: Внутренние датчики (батарея, температура, диагностика) + внешние (камеры, motion)
- **Semantic Platform**: Семантическая платформа для персонализированных сервисов на основе контекста

**Текущее состояние проекта:**
- ❌ НЕТ проверки интернета
- ❌ НЕТ осознания текущего времени/суток
- ⚠️ State monitoring частично (только battery, temperature через ESP32)
- ❌ НЕТ семантической платформы

### 4. Internal Dialogue Systems

**Best Practices:**
- **Advanced NLP**: Обработка различных интентов с контекстно-релевантными ответами
- **AI Personalization**: Персонализация на основе истории взаимодействий
- **Real-time Feedback**: Непрерывное обучение на основе взаимодействий
- **Privacy & Ethics**: Безопасная и прозрачная обработка данных

**Текущее состояние проекта:**
- ✅ NLP через DeepSeek API
- ✅ Суммаризация истории для контекста
- ⚠️ Персонализация ограничена (нет долгосрочной памяти)
- ✅ Privacy: API ключи через env vars

---

## 🔍 Детальный анализ компонентов

### 1. context_aggregator_node.py

**Текущая реализация:**
```python
class ContextAggregatorNode(Node):
    # Подписывается на:
    # - /perception/vision_context
    # - /rtabmap/localization_pose
    # - /odom
    # - /rosout (ERROR/WARN)
    # - /voice/stt/result
    # - /voice/dialogue/response
    # - /reflection/internal_thought
    
    # Публикует:
    # - /perception/context_update (PerceptionEvent)
    # - /perception/user_speech (String)
```

**✅ Сильные стороны:**
1. **Event-driven публикация** - 2 Hz, настраиваемая частота
2. **Разделение событий по типам** - speech, robot_response, robot_thought, vision, system
3. **Суммаризация через DeepSeek** - автоматическое сжатие истории при достижении threshold
4. **Мониторинг логов** - сбор ERROR и WARN из /rosout
5. **Sliding window memory** - память событий в течение 60 секунд

**❌ Выявленные проблемы:**

#### Проблема 1: НЕТ мониторинга доступности ROS2 нод
**Описание:** 
- Агрегатор не знает, какие ноды активны, а какие упали
- Нет проверки подписки на топики (могут быть publishers, но нет данных)
- Невозможно определить, что камера или LIDAR перестали работать

**Влияние:** 
- Reflection node не знает о проблемах с нодами
- Нет автоматического реагирования на сбои
- Пользователь не получает уведомлений о проблемах оборудования

#### Проблема 2: НЕТ осознания времени
**Описание:**
- Агрегатор не включает текущее время в контекст
- Нет понимания времени суток (день/ночь/утро/вечер)
- Нет часового пояса и временных меток с human-readable форматом

**Влияние:**
- Reflection node не может формировать time-aware ответы
- Нельзя использовать время как контекст ("сейчас ночь, поэтому нужна тихая езда")
- История событий имеет только Unix timestamps

#### Проблема 3: НЕТ детекции интернета
**Описание:**
- Нет проверки доступности интернета
- DeepSeek API может фейлиться без предупреждения
- Нет fallback режима при отсутствии интернета

**Влияние:**
- Непредсказуемые ошибки при потере интернета
- Dialogue node и reflection node становятся неработоспособными
- Нет уведомления пользователя о проблеме

#### Проблема 4: Ограниченный мониторинг оборудования
**Описание:**
- Только battery и temperature через ESP32
- НЕТ мониторинга камеры (OAK-D), LIDAR (LSLIDAR N10), моторов (VESC)
- НЕТ проверки FPS камеры, качества SLAM, статуса навигации

**Влияние:**
- Невозможно определить деградацию работы оборудования
- Нет предупреждений о проблемах до полного сбоя

#### Проблема 5: Суммаризация без приоритетов
**Описание:**
- Все события суммаризируются одинаково
- Нет понятия "важных" vs "обычных" событий
- Threshold фиксированный (50 событий) для всех типов

**Влияние:**
- Важные события (ошибки, battery low) могут потеряться
- Излишняя суммаризация незначительных событий (обычная езда)

### 2. reflection_node.py

**Текущая реализация:**
```python
class ReflectionNode(Node):
    # Подписывается на:
    # - /perception/context_update (PerceptionEvent)
    # - /perception/user_speech (String)
    # - /voice/dialogue/response (String)
    
    # Публикует:
    # - /reflection/internal_thought (String)
    # - /voice/tts/request (String)
    # - /voice/sound/trigger (String)
```

**✅ Сильные стороны:**
1. **Event-driven размышления** - реагирует на события, а не таймер
2. **Urgent response hook** - детекция личных вопросов для быстрого ответа
3. **Silence mode** - команда "помолчи" для временного отключения
4. **Speech debouncing** - не говорит чаще 30 секунд
5. **Emotion-based sounds** - автоматические звуки на основе эмоций

**❌ Выявленные проблемы:**

#### Проблема 6: Отсутствие проактивности
**Описание:**
- Reflection node только реагирует на события, но не проактивен
- Нет периодической проверки критичных параметров (battery, temperature)
- Нет автоматического уведомления о деградации системы

**Влияние:**
- Пользователь не получает предупреждения о низком заряде до критического уровня
- Нет напоминаний о важных событиях

#### Проблема 7: Нет долгосрочной памяти
**Описание:**
- Память ограничена 10 последними мыслями (recent_thoughts)
- Нет базы знаний о системе (когда последний раз был mapping, где проблемные зоны)
- Нет обучения на исторических данных

**Влияние:**
- Робот не "помнит" прошлые взаимодействия после перезапуска
- Нет персонализации на основе истории
- Повторяющиеся ошибки не анализируются

#### Проблема 8: Простая детекция личных вопросов
**Описание:**
- Regex patterns для детекции личных вопросов ограничены
- Нет semantic understanding (NLU)
- Можно пропустить нестандартные формулировки

**Влияние:**
- Некоторые личные вопросы обрабатываются dialogue_node (медленнее)

#### Проблема 9: Нет прогнозирования проблем
**Описание:**
- Reflection node не анализирует тренды (растущие ошибки, падающий заряд)
- Нет машинного обучения для предсказания сбоев
- Нет аномали-детекции

**Влияние:**
- Реагирование только на уже случившиеся проблемы
- Нет превентивного обслуживания

### 3. dialogue_node.py

**Текущая реализация:**
```python
class DialogueNode(Node):
    # State Machine: IDLE -> LISTENING -> DIALOGUE -> SILENCED
    # Wake words, silence commands, mapping commands
```

**✅ Сильные стороны:**
1. **State Machine** - чёткое управление диалогом
2. **Wake word detection** - активация по ключевым словам
3. **Streaming LLM responses** - DeepSeek streaming для быстрых ответов
4. **Mapping commands** - управление RTABMap через голос
5. **Confirmation system** - подтверждение для критичных команд

**❌ Выявленные проблемы:**

#### Проблема 10: НЕТ контекста от reflection_node
**Описание:**
- dialogue_node не получает размышления от reflection_node
- Нет доступа к full_context() из context_aggregator
- LLM не знает о текущем состоянии робота

**Влияние:**
- Ответы dialogue_node могут быть некорректными (например, "всё хорошо" при низком заряде)
- Нет unified knowledge base

#### Проблема 11: Ограниченная история диалога
**Описание:**
- Только последние 10 сообщений в conversation_history
- История не сохраняется между перезапусками
- Нет суммаризации длинных диалогов

**Влияние:**
- Робот "забывает" начало длинного разговора
- Нет непрерывности после перезапуска

### 4. health_monitor.py

**Текущая реализация:**
```python
class HealthMonitor(Node):
    # Простой мониторинг /rosout
    # Отчёт каждые 5 секунд
```

**✅ Сильные стороны:**
1. **Простота** - легко понять и поддерживать
2. **Sound alerts** - звуковые сигналы при изменении статуса

**❌ Выявленные проблемы:**

#### Проблема 12: Дублирование функционала
**Описание:**
- health_monitor дублирует часть функционала context_aggregator
- Оба мониторят /rosout
- Нет интеграции между ними

**Влияние:**
- Избыточная обработка логов
- Запутанная архитектура

#### Проблема 13: НЕТ проверки нод
**Описание:**
- health_monitor не проверяет доступность ROS2 нод
- Нет использования `ros2 node list` или similar API

**Влияние:**
- Невозможно определить упавшие ноды

---

## 🚨 Выявленные проблемы (Сводка)

### Критичные (High Priority)

1. **НЕТ мониторинга доступности ROS2 нод** ⚠️
   - Невозможно определить упавшие ноды
   - Нет автоматического реагирования на сбои

2. **НЕТ детекции доступности интернета** ⚠️
   - DeepSeek API фейлится без предупреждения
   - Нет fallback режима

3. **НЕТ осознания текущего времени** ⚠️
   - Нет time-aware ответов
   - Нет понимания контекста времени суток

4. **Ограниченный мониторинг оборудования** ⚠️
   - Только battery и temperature
   - НЕТ мониторинга камеры, LIDAR, моторов

### Средние (Medium Priority)

5. **Отсутствие проактивности reflection_node**
   - Нет периодических проверок критичных параметров

6. **НЕТ долгосрочной памяти**
   - Робот "забывает" после перезапуска

7. **dialogue_node не использует контекст от reflection**
   - Некорректные ответы при проблемах

8. **Дублирование функционала health_monitor**
   - Избыточная обработка логов

### Низкие (Low Priority)

9. **Простая детекция личных вопросов**
   - Regex patterns вместо semantic understanding

10. **Нет прогнозирования проблем**
    - Нет ML для предсказания сбоев

11. **Суммаризация без приоритетов**
    - Важные события могут потеряться

---

## 💡 Предложения по улучшению

### Улучшение 1: Node Availability Monitor

**Цель:** Мониторинг доступности ROS2 нод в реальном времени

**Реализация:**
```python
class NodeAvailabilityMonitor:
    """Мониторинг доступности ROS2 нод"""
    
    def __init__(self, node: Node):
        self.node = node
        self.expected_nodes = [
            '/audio_node',
            '/stt_node',
            '/tts_node',
            '/dialogue_node',
            '/reflection_node',
            '/oak_d_node',
            '/lslidar_node',
            '/rtabmap',
            '/nav2',
            '/vesc_driver'
        ]
        self.node_status = {}  # node_name -> {'last_seen', 'status'}
        
        # Периодическая проверка
        self.check_timer = self.node.create_timer(5.0, self.check_nodes)
    
    def check_nodes(self):
        """Проверить доступность всех ожидаемых нод"""
        import subprocess
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=2.0
        )
        
        active_nodes = result.stdout.strip().split('\n')
        
        for expected_node in self.expected_nodes:
            if expected_node in active_nodes:
                self.node_status[expected_node] = {
                    'status': 'active',
                    'last_seen': time.time()
                }
            else:
                # Нода отсутствует
                if expected_node not in self.node_status:
                    self.node_status[expected_node] = {
                        'status': 'missing',
                        'last_seen': None
                    }
                else:
                    # Нода пропала
                    self.node_status[expected_node]['status'] = 'failed'
    
    def get_failed_nodes(self) -> List[str]:
        """Получить список упавших нод"""
        return [
            node for node, status in self.node_status.items()
            if status['status'] == 'failed'
        ]
    
    def get_missing_nodes(self) -> List[str]:
        """Получить список отсутствующих нод"""
        return [
            node for node, status in self.node_status.items()
            if status['status'] == 'missing'
        ]
```

**Интеграция:** Добавить в context_aggregator_node

**Результат:**
- Автоматическое обнаружение упавших нод
- Добавление в PerceptionEvent.health_issues
- Reflection node получает информацию о проблемах

### Улучшение 2: Internet Connectivity Monitor

**Цель:** Детекция доступности интернета и fallback режим

**Реализация:**
```python
class InternetConnectivityMonitor:
    """Мониторинг доступности интернета"""
    
    def __init__(self, node: Node):
        self.node = node
        self.is_online = None
        self.last_check_time = None
        self.check_interval = 30.0  # секунд
        
        # Список серверов для проверки
        self.test_hosts = [
            '8.8.8.8',  # Google DNS
            '1.1.1.1',  # Cloudflare DNS
        ]
        
        # Таймер проверки
        self.check_timer = self.node.create_timer(
            self.check_interval,
            self.check_connectivity
        )
    
    def check_connectivity(self) -> bool:
        """Проверить доступность интернета"""
        import subprocess
        
        for host in self.test_hosts:
            try:
                result = subprocess.run(
                    ['ping', '-c', '1', '-W', '2', host],
                    capture_output=True,
                    timeout=3.0
                )
                
                if result.returncode == 0:
                    # Интернет доступен
                    if self.is_online is False:
                        self.node.get_logger().info('✅ Интернет восстановлен')
                    
                    self.is_online = True
                    self.last_check_time = time.time()
                    return True
            
            except (subprocess.TimeoutExpired, Exception):
                continue
        
        # Интернет недоступен
        if self.is_online is True or self.is_online is None:
            self.node.get_logger().warn('⚠️ Интернет недоступен')
        
        self.is_online = False
        self.last_check_time = time.time()
        return False
    
    def get_status(self) -> Dict:
        """Получить статус подключения"""
        return {
            'is_online': self.is_online,
            'last_check': self.last_check_time,
            'check_interval': self.check_interval
        }
```

**Интеграция:**
- Добавить в context_aggregator_node
- Добавить поле `internet_available` в PerceptionEvent
- dialogue_node и reflection_node используют fallback при offline

**Результат:**
- Автоматическое уведомление о потере интернета
- Fallback режим для dialogue (локальные ответы)
- Пользователь знает о проблеме

### Улучшение 3: Time Awareness

**Цель:** Осознание текущего времени и контекста времени суток

**Реализация:**
```python
from datetime import datetime
import pytz

class TimeAwarenessProvider:
    """Провайдер осознания времени"""
    
    def __init__(self, timezone='Europe/Moscow'):
        self.timezone = pytz.timezone(timezone)
    
    def get_current_time_context(self) -> Dict:
        """Получить контекст текущего времени"""
        now = datetime.now(self.timezone)
        
        hour = now.hour
        
        # Определяем период суток
        if 5 <= hour < 12:
            period = 'morning'
            period_ru = 'утро'
        elif 12 <= hour < 17:
            period = 'day'
            period_ru = 'день'
        elif 17 <= hour < 22:
            period = 'evening'
            period_ru = 'вечер'
        else:
            period = 'night'
            period_ru = 'ночь'
        
        return {
            'timestamp': now.timestamp(),
            'datetime': now.isoformat(),
            'human_readable': now.strftime('%Y-%m-%d %H:%M:%S'),
            'time_only': now.strftime('%H:%M'),
            'date_only': now.strftime('%Y-%m-%d'),
            'hour': hour,
            'minute': now.minute,
            'weekday': now.strftime('%A'),
            'weekday_ru': self._get_weekday_ru(now.weekday()),
            'period': period,
            'period_ru': period_ru,
            'timezone': str(self.timezone)
        }
    
    def _get_weekday_ru(self, weekday: int) -> str:
        """Получить день недели на русском"""
        days = {
            0: 'Понедельник',
            1: 'Вторник',
            2: 'Среда',
            3: 'Четверг',
            4: 'Пятница',
            5: 'Суббота',
            6: 'Воскресенье'
        }
        return days.get(weekday, 'Неизвестно')
```

**Интеграция:**
- Добавить в context_aggregator_node
- Добавить поля в PerceptionEvent:
  - `current_time_human` (String)
  - `time_period` (String) - morning/day/evening/night
  - `time_context_json` (String) - полный JSON контекст

**Результат:**
- Reflection node может формировать time-aware ответы ("Сейчас вечер, скоро ночь")
- dialogue_node может учитывать время ("Доброе утро!")
- История событий с human-readable временем

### Улучшение 4: Equipment Health Monitor

**Цель:** Расширенный мониторинг состояния оборудования

**Реализация:**
```python
class EquipmentHealthMonitor:
    """Мониторинг состояния оборудования"""
    
    def __init__(self, node: Node):
        self.node = node
        
        # Состояние оборудования
        self.equipment_status = {
            'oak_d_camera': {
                'status': 'unknown',
                'fps': 0.0,
                'last_frame_time': None,
                'issues': []
            },
            'lslidar': {
                'status': 'unknown',
                'scan_rate': 0.0,
                'last_scan_time': None,
                'issues': []
            },
            'vesc_motors': {
                'status': 'unknown',
                'left_rpm': 0.0,
                'right_rpm': 0.0,
                'last_update': None,
                'issues': []
            },
            'rtabmap_slam': {
                'status': 'unknown',
                'mapping_mode': False,
                'loop_closures': 0,
                'last_update': None,
                'issues': []
            }
        }
        
        # Подписки на топики оборудования
        self._setup_subscriptions()
        
        # Таймер для проверки timeout-ов
        self.check_timer = self.node.create_timer(2.0, self.check_timeouts)
    
    def _setup_subscriptions(self):
        """Настроить подписки на топики оборудования"""
        # OAK-D Camera
        self.camera_sub = self.node.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.on_camera_frame,
            10
        )
        
        # LSLIDAR
        self.lidar_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.on_lidar_scan,
            10
        )
        
        # VESC (odometry как proxy для моторов)
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.on_motor_odom,
            10
        )
        
        # RTABMap info
        self.rtabmap_info_sub = self.node.create_subscription(
            MapData,
            '/rtabmap/mapData',
            self.on_rtabmap_info,
            10
        )
    
    def on_camera_frame(self, msg: Image):
        """Получен фрейм с камеры"""
        now = time.time()
        status = self.equipment_status['oak_d_camera']
        
        # Обновляем статус
        status['status'] = 'active'
        status['last_frame_time'] = now
        
        # Вычисляем FPS
        if hasattr(self, '_last_camera_time'):
            dt = now - self._last_camera_time
            status['fps'] = 1.0 / dt if dt > 0 else 0.0
        
        self._last_camera_time = now
        
        # Проверка FPS
        if status['fps'] < 15.0 and status['fps'] > 0:
            if 'low_fps' not in status['issues']:
                status['issues'].append('low_fps')
                self.node.get_logger().warn(f'⚠️ Камера: низкий FPS ({status["fps"]:.1f})')
        else:
            if 'low_fps' in status['issues']:
                status['issues'].remove('low_fps')
    
    def check_timeouts(self):
        """Проверить timeout-ы для оборудования"""
        now = time.time()
        timeout = 5.0  # секунд
        
        for equipment, status in self.equipment_status.items():
            last_update = status.get('last_frame_time') or status.get('last_scan_time') or status.get('last_update')
            
            if last_update and (now - last_update) > timeout:
                if status['status'] != 'timeout':
                    status['status'] = 'timeout'
                    status['issues'].append('no_data')
                    self.node.get_logger().warn(f'⚠️ {equipment}: timeout (нет данных {timeout}s)')
            elif last_update:
                if 'no_data' in status['issues']:
                    status['issues'].remove('no_data')
                    self.node.get_logger().info(f'✅ {equipment}: восстановлен')
    
    def get_summary(self) -> Dict:
        """Получить сводку состояния оборудования"""
        healthy_count = sum(1 for s in self.equipment_status.values() if s['status'] == 'active')
        total_count = len(self.equipment_status)
        
        all_issues = []
        for equipment, status in self.equipment_status.items():
            if status['issues']:
                all_issues.append(f"{equipment}: {', '.join(status['issues'])}")
        
        return {
            'healthy_count': healthy_count,
            'total_count': total_count,
            'overall_status': 'healthy' if healthy_count == total_count else 'degraded',
            'equipment_details': self.equipment_status,
            'issues': all_issues
        }
```

**Интеграция:**
- Добавить в context_aggregator_node
- Добавить поля в PerceptionEvent:
  - `equipment_summary` (String) - JSON сводка
  - Расширить `health_issues` информацией об оборудовании

**Результат:**
- Мониторинг камеры (FPS, наличие данных)
- Мониторинг LIDAR (scan rate, наличие данных)
- Мониторинг моторов (RPM, одометрия)
- Мониторинг SLAM (loop closures, mapping mode)
- Автоматические уведомления о проблемах

### Улучшение 5: Proactive Reflection

**Цель:** Проактивные размышления и уведомления

**Реализация:**
```python
class ProactiveReflectionTimer:
    """Проактивные периодические размышления"""
    
    def __init__(self, reflection_node):
        self.node = reflection_node
        
        # Правила для проактивных размышлений
        self.proactive_rules = [
            {
                'name': 'battery_check',
                'interval': 60.0,  # каждую минуту
                'condition': self.check_battery_level,
                'action': self.warn_low_battery
            },
            {
                'name': 'equipment_check',
                'interval': 300.0,  # каждые 5 минут
                'condition': self.check_equipment_health,
                'action': self.report_equipment_issues
            },
            {
                'name': 'idle_greeting',
                'interval': 600.0,  # каждые 10 минут
                'condition': self.check_idle_time,
                'action': self.idle_greeting
            }
        ]
        
        # Таймеры для каждого правила
        self.rule_timers = {}
        for rule in self.proactive_rules:
            timer = self.node.create_timer(
                rule['interval'],
                lambda r=rule: self.execute_rule(r)
            )
            self.rule_timers[rule['name']] = timer
    
    def execute_rule(self, rule: Dict):
        """Выполнить проактивное правило"""
        if rule['condition']():
            rule['action']()
    
    def check_battery_level(self) -> bool:
        """Проверить уровень батареи"""
        if not self.node.last_context:
            return False
        
        battery = self.node.last_context.battery_voltage
        
        # Критичный уровень: < 11.0V
        # Предупреждение: < 11.5V
        return battery > 0 and battery < 11.5
    
    def warn_low_battery(self):
        """Предупредить о низкой батарее"""
        battery = self.node.last_context.battery_voltage
        
        if battery < 11.0:
            speech = f"Внимание! Критически низкий заряд батареи: {battery:.1f} вольт. Требуется срочная зарядка."
            self.node._publish_speech(speech)
        else:
            speech = f"Батарея на низком уровне: {battery:.1f} вольт. Рекомендую зарядку."
            self.node._publish_speech(speech)
```

**Интеграция:**
- Добавить в reflection_node
- Настраиваемые правила через параметры

**Результат:**
- Периодические проверки критичных параметров
- Автоматические предупреждения о проблемах
- Idle greeting для вовлечения пользователя

### Улучшение 6: Long-term Memory (Optional)

**Цель:** Долгосрочная память для персонализации

**Реализация:**
```python
import sqlite3
from datetime import datetime

class LongTermMemory:
    """Долгосрочная память робота"""
    
    def __init__(self, db_path='/maps/robot_memory.db'):
        self.db_path = db_path
        self._init_database()
    
    def _init_database(self):
        """Инициализировать базу данных"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Таблица событий
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                event_type TEXT,
                content TEXT,
                importance INTEGER
            )
        ''')
        
        # Таблица пользовательских предпочтений
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS preferences (
                key TEXT PRIMARY KEY,
                value TEXT,
                last_updated REAL
            )
        ''')
        
        # Таблица мыслей робота
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS thoughts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                thought TEXT,
                context TEXT
            )
        ''')
        
        conn.commit()
        conn.close()
    
    def store_event(self, event_type: str, content: str, importance: int = 1):
        """Сохранить событие"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute(
            'INSERT INTO events (timestamp, event_type, content, importance) VALUES (?, ?, ?, ?)',
            (time.time(), event_type, content, importance)
        )
        
        conn.commit()
        conn.close()
    
    def get_recent_events(self, event_type: str = None, limit: int = 100) -> List[Dict]:
        """Получить недавние события"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        if event_type:
            cursor.execute(
                'SELECT * FROM events WHERE event_type = ? ORDER BY timestamp DESC LIMIT ?',
                (event_type, limit)
            )
        else:
            cursor.execute(
                'SELECT * FROM events ORDER BY timestamp DESC LIMIT ?',
                (limit,)
            )
        
        rows = cursor.fetchall()
        conn.close()
        
        return [
            {
                'id': row[0],
                'timestamp': row[1],
                'event_type': row[2],
                'content': row[3],
                'importance': row[4]
            }
            for row in rows
        ]
    
    def store_preference(self, key: str, value: str):
        """Сохранить пользовательское предпочтение"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute(
            'INSERT OR REPLACE INTO preferences (key, value, last_updated) VALUES (?, ?, ?)',
            (key, value, time.time())
        )
        
        conn.commit()
        conn.close()
    
    def get_preference(self, key: str) -> Optional[str]:
        """Получить пользовательское предпочтение"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('SELECT value FROM preferences WHERE key = ?', (key,))
        row = cursor.fetchone()
        
        conn.close()
        
        return row[0] if row else None
```

**Интеграция:**
- Опциональный модуль для context_aggregator и reflection_node
- SQLite база на /maps (persistent volume)

**Результат:**
- Сохранение истории между перезапусками
- Пользовательские предпочтения (любимые маршруты, предпочитаемый стиль общения)
- База знаний для обучения

### Улучшение 7: Unified Context Provider

**Цель:** Единый интерфейс для доступа к контексту

**Реализация:**
```python
class UnifiedContextProvider:
    """Единый провайдер контекста для всех нод"""
    
    def __init__(self, node: Node):
        self.node = node
        
        # Компоненты
        self.node_monitor = NodeAvailabilityMonitor(node)
        self.internet_monitor = InternetConnectivityMonitor(node)
        self.time_provider = TimeAwarenessProvider()
        self.equipment_monitor = EquipmentHealthMonitor(node)
    
    def get_full_context(self) -> Dict:
        """Получить полный контекст системы"""
        return {
            'timestamp': time.time(),
            'time_context': self.time_provider.get_current_time_context(),
            'internet': self.internet_monitor.get_status(),
            'nodes': {
                'active': [n for n, s in self.node_monitor.node_status.items() if s['status'] == 'active'],
                'failed': self.node_monitor.get_failed_nodes(),
                'missing': self.node_monitor.get_missing_nodes()
            },
            'equipment': self.equipment_monitor.get_summary(),
            'system_health': self._compute_overall_health()
        }
    
    def _compute_overall_health(self) -> str:
        """Вычислить общий статус здоровья"""
        issues_count = 0
        
        # Интернет
        if not self.internet_monitor.is_online:
            issues_count += 1
        
        # Ноды
        failed_nodes = self.node_monitor.get_failed_nodes()
        if failed_nodes:
            issues_count += len(failed_nodes)
        
        # Оборудование
        equipment_summary = self.equipment_monitor.get_summary()
        if equipment_summary['overall_status'] != 'healthy':
            issues_count += len(equipment_summary['issues'])
        
        if issues_count == 0:
            return 'healthy'
        elif issues_count <= 2:
            return 'degraded'
        else:
            return 'critical'
```

**Интеграция:**
- Заменить разрозненные компоненты в context_aggregator_node
- Использовать в dialogue_node и reflection_node

**Результат:**
- Единый источник истины для контекста
- Упрощённый доступ к данным
- Consistency across nodes

---

## 📝 План реализации

### Этап 1: Критичные улучшения (1-2 недели)

**Задачи:**
1. ✅ Создать `NodeAvailabilityMonitor` и интегрировать в `context_aggregator_node`
2. ✅ Создать `InternetConnectivityMonitor` и интегрировать
3. ✅ Создать `TimeAwarenessProvider` и добавить в `PerceptionEvent`
4. ✅ Расширить `PerceptionEvent` message:
   ```
   # Добавить поля:
   string current_time_human
   string time_period  # morning/day/evening/night
   string time_context_json
   bool internet_available
   string[] failed_nodes
   string[] missing_nodes
   ```
5. ✅ Обновить `reflection_node` для использования нового контекста
6. ✅ Обновить `dialogue_node` для использования нового контекста

**Результат:**
- Робот осознаёт время, доступность интернета и состояние нод
- Автоматические уведомления о критичных проблемах

### Этап 2: Расширенный мониторинг (1-2 недели)

**Задачи:**
1. ✅ Создать `EquipmentHealthMonitor`
2. ✅ Интегрировать в `context_aggregator_node`
3. ✅ Добавить поля в `PerceptionEvent`:
   ```
   string equipment_summary_json
   ```
4. ✅ Создать `ProactiveReflectionTimer` для `reflection_node`
5. ✅ Добавить правила для проактивных уведомлений

**Результат:**
- Мониторинг камеры, LIDAR, моторов, SLAM
- Проактивные предупреждения о проблемах

### Этап 3: Архитектурные улучшения (2-3 недели)

**Задачи:**
1. ✅ Создать `UnifiedContextProvider`
2. ✅ Рефакторинг `context_aggregator_node` для использования `UnifiedContextProvider`
3. ✅ Упростить или объединить `health_monitor` с `context_aggregator`
4. ✅ Добавить service для получения full context:
   ```python
   # ros2 service call /perception/get_full_context rob_box_perception_msgs/srv/GetFullContext
   ```
5. ✅ Обновить документацию

**Результат:**
- Чистая архитектура с единым источником контекста
- Service API для других нод

### Этап 4: Опциональные улучшения (опционально)

**Задачи:**
1. ⚪ Создать `LongTermMemory` с SQLite
2. ⚪ Интегрировать в `context_aggregator` и `reflection_node`
3. ⚪ Добавить NLU для детекции интентов (вместо regex)
4. ⚪ Добавить ML для прогнозирования проблем

**Результат:**
- Персонализация и обучение
- Предсказание сбоев

---

## 🎯 Ожидаемые результаты

После реализации предложенных улучшений:

### Функциональность

✅ **Осознание системы:**
- Робот знает текущее время и период суток
- Робот знает, доступен ли интернет
- Робот знает, какие ноды активны, а какие упали
- Робот знает состояние оборудования (камера, LIDAR, моторы, SLAM)

✅ **Проактивность:**
- Автоматические предупреждения о низком заряде
- Уведомления о падении нод
- Информирование о потере интернета
- Мониторинг деградации оборудования

✅ **Улучшенный диалог:**
- Time-aware ответы ("Доброе утро!", "Сейчас вечер")
- Context-aware ответы (учёт состояния системы)
- Fallback режим при отсутствии интернета

### Качество

✅ **Надёжность:**
- Автоматическое обнаружение сбоев
- Graceful degradation при проблемах
- Resilience к потере интернета

✅ **Observability:**
- Полный мониторинг состояния системы
- Единый источник истины для контекста
- Service API для доступа к контексту

✅ **Maintainability:**
- Модульная архитектура
- Единый `UnifiedContextProvider`
- Чистое разделение ответственности

---

## 📚 Ссылки и источники

### Лучшие практики

1. **Robot Context Awareness:**
   - [An IoT Platform with Monitoring Robot Applying CNN-Based Context Awareness](https://www.mdpi.com/1424-8220/19/11/2525)
   - [Semantics-based platform for context-aware robot](https://www.sciencedirect.com/science/article/pii/S0164121218302553)

2. **ROS2 System Monitoring:**
   - [ROS2 Node Lifecycle Management](https://design.ros2.org/articles/node_lifecycle.html)
   - [ros2sysmon - System Monitor Tool](https://github.com/ros2/ros2_tracing)

3. **AI Internal Dialogue:**
   - [AI in Internal Communications Best Practices](https://cerkl.com/blog/ai-in-internal-communications/)
   - [Real-time Feedback Systems](https://www.simpplr.com/blog/ai-in-internal-communications/)

### Текущая документация проекта

- `docs/architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md` - Актуальная архитектура
- `docs/development/AGENT_GUIDE.md` - Руководство для агентов
- `docs/development/PYTHON_STYLE_GUIDE.md` - Стандарты кода

---

## 🔚 Заключение

Текущая реализация perception nodes и voice assistant **функциональна и хорошо структурирована**, но имеет **критичные пробелы** в системном мониторинге и осознании контекста.

Предложенные улучшения основаны на **индустриальных best practices** и **актуальных исследованиях** в области робототехники и AI систем.

Реализация предложенных изменений **значительно повысит** надёжность, observability и пользовательский опыт взаимодействия с роботом.

**Рекомендуется** начать с **Этапа 1** (критичные улучшения) и постепенно продвигаться к более сложным функциям.

---

**Следующие шаги:**
1. Обсудить предложенные решения с командой
2. Выбрать приоритетные улучшения
3. Создать sub-PR для каждого этапа
4. Реализовать тесты для новых компонентов
5. Обновить документацию
