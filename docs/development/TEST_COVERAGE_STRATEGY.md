# Стратегия повышения покрытия тестами

**Цель:** Повысить покрытие Python кода с 19% до 80%+

**Дата старта:** 17 января 2026  
**Текущее состояние:** 19% (799/4118 строк)

## 📊 Baseline - Текущее покрытие

| Модуль | Строк | Покрытие | Покрыто строк | Статус |
|--------|-------|----------|---------------|--------|
| **dialogue_node.py** | 1111 | 80% | 888 | ✅ Хорошо |
| **health_monitor.py** | 80 | ~~19%~~ **84%** | ~~15~~ **67** | ✅ **УЛУЧШЕНО!** |
| **sound_node.py** | 194 | ~~53%~~ **83%** | ~~103~~ **161** | ✅ **УЛУЧШЕНО!** |
| **startup_greeting_node.py** | 82 | 77% | 63 | ✅ Хорошо |
| **audio_node.py** | 216 | 64% | 138 | 🟡 Средне |
| **led_node.py** | 166 | 62% | 103 | 🟡 Средне |
| **command_node.py** | 267 | 48% | 128 | 🔴 Низко |
| **context_aggregator.py** | 342 | 43% | 147 | 🔴 Низко |
| **reflection_node.py** | 446 | 28% | 125 | 🔴 Низко |
| **stt_node.py** | 160 | 0% | 0 | 🔴 Нет тестов |
| **tts_node.py** | 423 | 0% | 0 | 🔴 Нет тестов |

## 🎯 Фазы реализации

### ✅ Фаза 1: Quick Wins - Малые модули (ЗАВЕРШЕНА)

**Результаты:**
1. **health_monitor.py**: 19% → **84%** (+65%) - ✅ ЗАВЕРШЕНО
   - Добавлено 9 тестов (всего 19)
   - Покрыто: status calculation, sound triggers, error counting

2. **sound_node.py**: 53% → **83%** (+30%) - ✅ ЗАВЕРШЕНО
   - Добавлено 21 тест (всего 32)
   - Покрыто: sound selection, playback thread, resampling, animations, cleanup

**Коммиты:**
- `250d49c` - health_monitor coverage 19% → 84%
- `783f89f` - sound_node coverage 53% → 83%

---

### 🚀 Фаза 2: Средние модули (В ПРОЦЕССЕ → СЛЕДУЮЩАЯ)

**Цель:** +40% общего покрытия

#### 2.1. startup_greeting_node.py (77% → 90%) - SKIP (уже достаточно)
**Причина:** 77% покрытие уже хорошее, остались только main() и shutdown_node()

#### ⭐ 2.2. led_node.py (62% → 90%) - **СЛЕДУЮЩИЙ ПРИОРИТЕТ**
**Не хватает покрытия:**
- PixelRingLite - различные анимации (think, speak, off, wakeup)
- Обработка ошибок USB (device not found)
- DoA (Direction of Arrival) визуализация
- Синхронизация с voice_state
- SetBool service для auto_led

**Оценка:** 12-15 новых тестов, +47 строк (103→150 из 166)

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
| Фаза 1 ✅ | **~25%** | **~1030/4118** | **+30** | **17.01.2026** |
| Фаза 2 | 45% | ~1850/4118 | +30 | - |
| Фаза 3 | 65% | ~2670/4118 | +50 | - |
| Фаза 4 | 80%+ | ~3300/4118 | +30 | - |

**Прогресс Фазы 1:**
- ✅ health_monitor: +9 тестов (+52 строки покрытия)
- ✅ sound_node: +21 тест (+58 строк покрытия)
- **Итого:** +30 тестов, +110 строк покрытия

**Итоговая цель:** 80%+ покрытие за 140+ новых тестов

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

