# Стратегия повышения покрытия тестами

**Цель:** Повысить покрытие Python кода с 19% до 80%+

**Дата старта:** 17 января 2026  
**Текущее состояние:** 19% (799/4118 строк)

## 📊 Baseline - Текущее покрытие

| Модуль | Строк | Покрытие | Покрыто строк | Статус |
|--------|-------|----------|---------------|--------|
| **dialogue_node.py** | 1111 | 80% | 888 | ✅ Хорошо |
| **startup_greeting_node.py** | 82 | 77% | 63 | ✅ Хорошо |
| **audio_node.py** | 216 | 64% | 138 | 🟡 Средне |
| **led_node.py** | 166 | 62% | 103 | 🟡 Средне |
| **sound_node.py** | 194 | 53% | 103 | 🟡 Средне |
| **command_node.py** | 267 | 48% | 128 | 🔴 Низко |
| **reflection_node.py** | 446 | 28% | 125 | 🔴 Низко |
| **health_monitor.py** | 80 | ~~19%~~ **84%** | ~~15~~ **67** | ✅ **УЛУЧШЕНО!** |
| **context_aggregator.py** | 342 | 15% | 51 | 🔴 Критично |
| **stt_node.py** | 160 | 0% | 0 | 🔴 Нет тестов |
| **tts_node.py** | 423 | 0% | 0 | 🔴 Нет тестов |

## 🎯 Фазы реализации

### ✅ Фаза 1: Quick Wins - Малые модули (ЗАВЕРШЕНА)

**Результат:** health_monitor.py: 19% → **84%** (+65%)

**Добавлено 9 новых тестов:**
1. `test_status_healthy` - статус "✅ HEALTHY"
2. `test_status_degraded` - статус "⚠️ DEGRADED" (5+ ошибок/мин)
3. `test_status_critical` - статус "🚨 CRITICAL" (FATAL ошибки)
4. `test_sound_trigger_on_status_change_to_critical` - звук 'angry_2'
5. `test_sound_trigger_on_status_change_to_degraded` - звук 'confused'
6. `test_sound_trigger_on_recovery` - звук 'cute' при восстановлении
7. `test_sound_disabled` - отключение звуков через параметр
8. `test_play_sound_exception_handling` - обработка ошибок публикации
9. `test_recent_errors_calculation` - подсчёт ошибок за 60 секунд

**Итого тестов:** 19 (было 10)

---

### 🚀 Фаза 2: Средние модули (В ПРОЦЕССЕ)

**Цель:** +40% общего покрытия

#### 2.1. startup_greeting_node.py (77% → 95%)
**Не хватает покрытия:**
- Edge cases в `check_readiness()`
- Тайм-аут при недоступных топиках
- Обработка некорректного JSON из context

**Оценка:** 2-3 новых теста, +18 строк

#### 2.2. sound_node.py (53% → 85%)
**Не хватает покрытия:**
- `play_sound_thread()` - полный цикл проигрывания
- Обработка отсутствующих файлов (sound_pack/)
- Регулировка громкости в реальном времени
- `find_respeaker_device_sounddevice()` - различные сценарии

**Оценка:** 6-8 новых тестов, +60 строк

#### 2.3. led_node.py (62% → 90%)
**Не хватает покрытия:**
- PixelRingLite - различные анимации
- Обработка ошибок USB (device not found)
- DoA (Direction of Arrival) визуализация
- Синхронизация с voice_state

**Оценка:** 8-10 новых тестов, +45 строк

#### 2.4. audio_node.py (64% → 90%)
**Проблема:** Падают текущие тесты (audio_common_msgs dependency)

**План:**
1. Исправить импорты (мокать audio_common_msgs)
2. Добавить тесты для audio callback обработки
3. Покрыть error handling (нет микрофона)

**Оценка:** Исправить 6 тестов + 4 новых, +55 строк

---

### 🔥 Фаза 3: Сложные модули

#### 3.1. reflection_node.py (28% → 75%)
**Сложность:** ⭐⭐⭐ (OpenAI API, async dialogue)

**Не хватает покрытия:**
- `_generate_reflection()` - полный цикл с OpenAI
- Обработка различных PerceptionEvent типов
- Timeout механизмы (dialogue_timeout, urgent_response_timeout)
- Silence mode ("помолчи" command)
- Error handling для API failures

**Требуется:**
- Mock OpenAI API responses (success, error, timeout)
- Тесты async операций
- Edge cases для sound debounce

**Оценка:** 15-20 новых тестов, +210 строк

#### 3.2. command_node.py (48% → 85%)
**Сложность:** ⭐⭐ (NLP parsing, command extraction)

**Проблема:** Падают существующие тесты

**Не хватает покрытия:**
- `parse_command()` - извлечение команд из текста
- Movement commands (вперед, назад, стоп)
- Turn commands (направо, налево)
- Non-command filtering
- Distance/angle extraction (regex parsing)

**Оценка:** Исправить 9 тестов + 8 новых, +100 строк

#### 3.3. context_aggregator_node.py (15% → 60%)
**Сложность:** ⭐⭐⭐⭐ (большой модуль, 342 строки, 12 subscribers)

**Не хватает покрытия:**
- Vision context callback
- Localization pose handling
- ESP32 sensor data processing
- AprilTag detection processing
- ROS log aggregation
- STT result transit
- Summarization logic (DeepSeek API)
- Node availability monitoring
- Internet connectivity checks
- Time awareness

**Требуется:**
- Mock для всех 12+ топиков
- Mock DeepSeek API
- Интеграционные тесты для агрегации

**Оценка:** 25-30 новых тестов, +145 строк

---

### 🔴 Фаза 4: Критичные пробелы

#### 4.1. stt_node.py (0% → 70%)
**Проблема:** audio_common_msgs dependency отсутствует

**План:**
1. Mock audio_common_msgs.msg.AudioData
2. Тесты Vosk STT engine
3. VAD (Voice Activity Detection)
4. Audio streaming и чанкование

**Оценка:** 10-12 новых тестов

#### 4.2. tts_node.py (0% → 70%)
**Проблема:** audio_common_msgs dependency

**План:**
1. Mock audio output
2. Тесты TTS API (OpenAI/local)
3. Audio queue management
4. Voice synthesis параметры

**Оценка:** 10-12 новых тестов

---

## 🛠️ Принципы рефакторинга для тестируемости

### 1. Выделение чистых функций

```python
# ❌ ПЛОХО - смешаны логика и I/O
def process_sensor_data(self):
    msg = self.subscription.get_message()
    if msg.value > 100:
        self.publisher.publish("Alert!")
    
# ✅ ХОРОШО - логика отдельно, легко тестируется
def should_alert(value: float, threshold: float = 100.0) -> bool:
    """Pure function"""
    return value > threshold

def process_sensor_data(self):
    msg = self.subscription.get_message()
    if self.should_alert(msg.value):
        self.publisher.publish("Alert!")
```

### 2. Dependency Injection

```python
# ❌ ПЛОХО - хардкод зависимостей
class Node:
    def __init__(self):
        self.api_client = OpenAI(api_key="...")
        
# ✅ ХОРОШО - инжектим зависимость
class Node:
    def __init__(self, api_client=None):
        self.api_client = api_client or OpenAI(api_key="...")
```

### 3. Разбиение больших методов

```python
# ❌ ПЛОХО - метод на 150 строк
def handle_context_update(self, msg):
    # 150 строк логики...
    
# ✅ ХОРОШО - разбито на функции
def handle_context_update(self, msg):
    data = self._parse_context(msg)
    summary = self._summarize_data(data)
    self._publish_event(summary)
    
def _parse_context(self, msg) -> dict:
    """Unit-testable parsing"""
    ...
    
def _summarize_data(self, data: dict) -> str:
    """Unit-testable summarization"""
    ...
```

### 4. Избегать глобального состояния

```python
# ❌ ПЛОХО - мутируемое глобальное состояние
global_cache = {}

def process(data):
    global_cache[data.id] = data.value
    
# ✅ ХОРОШО - явное управление состоянием
class Processor:
    def __init__(self):
        self.cache = {}
        
    def process(self, data):
        self.cache[data.id] = data.value
```

---

## 📈 Целевые метрики

| Фаза | Покрытие | Покрыто строк | Новых тестов | Дата |
|------|----------|---------------|--------------|------|
| Baseline | 19% | 799/4118 | - | 17.01.2026 |
| Фаза 1 ✅ | 22% | ~900/4118 | +9 | 17.01.2026 |
| Фаза 2 | 45% | ~1850/4118 | +30 | - |
| Фаза 3 | 65% | ~2670/4118 | +50 | - |
| Фаза 4 | 80%+ | ~3300/4118 | +30 | - |

**Итоговая цель:** 80%+ покрытие за 120+ новых тестов

---

## ✅ Чеклист качества тестов

### Хороший тест должен:
- ✅ Быть изолированным (не зависит от других тестов)
- ✅ Быть повторяемым (один и тот же результат)
- ✅ Быть быстрым (<100ms для unit-теста)
- ✅ Тестировать одну вещь (Single Responsibility)
- ✅ Иметь понятное имя (`test_status_changes_to_critical_when_fatal_error`)
- ✅ Использовать моки для внешних зависимостей (ROS topics, API, файлы)
- ✅ Проверять как success, так и error cases
- ✅ Покрывать edge cases (None, пустые строки, переполнение)

### Плохие практики:
- ❌ Тесты зависят от порядка выполнения
- ❌ Требуют реального оборудования (USB, камера, микрофон)
- ❌ Используют sleep() для синхронизации
- ❌ Тестируют несколько вещей одновременно
- ❌ Зависят от внешних сервисов (интернет, API без моков)

---

## 🔄 Workflow разработки тестов

1. **Анализ покрытия**
   ```bash
   pytest --cov=src/package --cov-report=term-missing
   ```

2. **Выбор модуля** (приоритет: малые → средние → большие)

3. **Изучение кода** (читаем исходный модуль целиком)

4. **Планирование** (что не покрыто? какие edge cases?)

5. **Написание тестов** (начинаем с простых)

6. **Запуск**
   ```bash
   pytest path/to/test_file.py -v
   ```

7. **Проверка покрытия**
   ```bash
   pytest --cov=module --cov-report=html
   firefox htmlcov/index.html  # Визуальный анализ
   ```

8. **Рефакторинг** (если код нетестируем → рефакторим)

9. **Commit**
   ```bash
   git commit -m "test: increase coverage for X from Y% to Z%"
   ```

---

## 📚 Полезные ресурсы

- **Python unittest**: [docs.python.org/3/library/unittest](https://docs.python.org/3/library/unittest.html)
- **pytest-cov**: [pytest-cov.readthedocs.io](https://pytest-cov.readthedocs.io/)
- **ROS 2 Testing**: [docs.ros.org/en/humble/Tutorials/Testing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- **Mock patterns**: [docs.python.org/3/library/unittest.mock](https://docs.python.org/3/library/unittest.mock.html)

---

**Последнее обновление:** 17 января 2026  
**Следующий шаг:** Фаза 2.1 - startup_greeting_node.py

