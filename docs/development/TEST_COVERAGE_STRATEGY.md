# Стратегия повышения покрытия тестами

**Цель:** Повысить покрытие Python кода с 19% до 80%+

**Дата старта:** 17 января 2026  
**Текущее состояние:** 19% (799/4118 строк)

## 📊 Baseline - Текущее покрытие

| Модуль | Строк | Покрытие | Покрыто строк | Статус |
|--------|-------|----------|---------------|--------|
| **dialogue_node.py** | 1111 | 80% | 888 | ✅ Хорошо |
| **led_node.py** | 166 | ~~62%~~ **~90%** | ~~103~~ **~150** | ✅ **УЛУЧШЕНО!** |
| **health_monitor.py** | 80 | ~~19%~~ **84%** | ~~15~~ **67** | ✅ **УЛУЧШЕНО!** |
| **sound_node.py** | 194 | ~~53%~~ **83%** | ~~103~~ **161** | ✅ **УЛУЧШЕНО!** |
| **startup_greeting_node.py** | 82 | 77% | 63 | ✅ Хорошо |
| **reflection_node.py** | 446 | ~~28%~~ **~75%** | ~~125~~ **~330** | ✅ **УЛУЧШЕНО!** |
| **audio_node.py** | 216 | 64% | 138 | 🟡 Средне |
| **command_node.py** | 267 | 48% | 128 | 🔴 Низко |
| **context_aggregator.py** | 342 | 43% | 147 | 🔴 Низко |
| **stt_node.py** | 160 | 0% | 0 | 🔴 Нет тестов |
| **tts_node.py** | 423 | 0% | 0 | 🔴 Нет тестов |

## 🎯 Фазы реализации

### ✅ Фаза 1: Quick Wins - Малые модули (ЗАВЕРШЕНА)

**Результаты:**
1. **health_monitor.py**: 19% → **84%** (+65%) - ✅ ЗАВЕРШЕНО
   - Добавлено 9 тестов (всего 19)
   - Покрыто: status calculation, sound triggers, error counting

2. **sound_node.py**: 53% → **83%** (+30%) - ✅ ЗАВЕРШЕНО
   - Добавлено 17 тестов (всего 32)
   - Покрыто: sound selection, playback thread, resampling, animations, cleanup

3. **led_node.py**: 62% → **~90%** (+28%) - ✅ ЗАВЕРШЕНО
   - Добавлено 28 тестов (всего 51)
   - Покрыто: PixelRingLite USB, voice states, hardware init, manual modes

**Коммиты:**
- `250d49c` - health_monitor coverage 19% → 84%
- `783f89f` - sound_node coverage 53% → 83%
- `6f9bd60` - led_node coverage 62% → ~90%

**Итого Фазы 1:**
- Добавлено: **+54 теста** (9 + 17 + 28)
- Улучшено: **+157 строк покрытия** (52 + 58 + 47)

**Коммиты Фазы 2:**
- `90c6d79` - reflection_node coverage 28% → ~75%

**Итого Фазы 1+2.1:**
- Добавлено: **+92 теста** (54 + 38)
- Улучшено: **+362 строки покрытия** (157 + 205)
- Общее покрытие: **19% → ~32%** (+13%)

---

### ⭐ Фаза 2: Средние модули (В ПРОЦЕССЕ)

**Цель:** +20% общего покрытия

#### 2.1. reflection_node.py (28% → ~75%) - ✅ **ЗАВЕРШЕНО!**
**Сложность:** ⭐⭐⭐ (OpenAI API, async dialogue)

**Покрыто:**
- `_is_personal_question()` - все 6 regex patterns (7 тестов)
- `_is_silence_command()` - все 6 regex patterns (7 тестов)
- `_check_health_status_change()` - edge detection, periodic check, 3 states (6 тестов)
- `_publish_speech()` / `_publish_speech_ssml()` - silence mode, JSON formatting (4 теста)
- `_trigger_sound_for_thought()` / `_play_sound()` - 5 emotions, debounce (8 тестов)
- `on_context_update()` / `on_user_speech()` / `on_robot_response()` - callbacks (6 тестов)

**Результат:**
- Добавлено 38 тестов (20 → 58)
- Покрыто: ~330 строк из 446 (~75% coverage)
- Commit: `90c6d79`

**Также исправлено:**
- reflection_node.py: regex для 'настроение' (твоё? → тво[её])

#### 2.2. command_node.py (48% → 85%) - **СЛЕДУЮЩИЙ ПРИОРИТЕТ**
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
| Фаза 1 ✅ | **~27%** | **~1110/4118** | **+54** | **17.01.2026** |
| Фаза 2 | 45% | ~1850/4118 | +40 | - |
| Фаза 3 | 65% | ~2670/4118 | +50 | - |
| Фаза 4 | 80%+ | ~3300/4118 | +30 | - |

**Прогресс Фазы 1:**
- ✅ health_monitor: +9 тестов (+52 строки покрытия)
- ✅ sound_node: +17 тестов (+58 строк покрытия)
- ✅ led_node: +28 тестов (+47 строк покрытия)
- **Итого:** +54 теста, +157 строк покрытия

**Итоговая цель:** 80%+ покрытие за 174+ новых тестов

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

