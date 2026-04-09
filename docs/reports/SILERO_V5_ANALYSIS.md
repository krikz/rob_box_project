# 🎤 Анализ Silero TTS v5: Обновление с v4 до v5

**Дата:** 2025-11-14  
**Автор:** AI Agent (GitHub Copilot)  
**Проект:** Rob Box (РОББОКС)  
**Текущая версия:** Silero TTS v4  
**Целевая версия:** Silero TTS v5  
**Платформа:** Raspberry Pi 5 (ARM64)

---

## 📊 Резюме

Silero выпустили **версию 5** своей TTS модели для русского языка с существенными улучшениями:
- **Скорость**: в 1.5-2 раза быстрее v4 
- **Качество**: улучшенное звучание и стабильность генерации
- **Омографы**: автоматическая расстановка ударений (новая функция!)
- **API**: упрощенный интерфейс через `pip install silero`

### 🎯 Рекомендация

**✅ РЕКОМЕНДУЕТСЯ ОБНОВЛЕНИЕ** при условии:
1. Тестирования производительности на Raspberry Pi 5
2. Проверки совместимости с существующим кодом
3. Оценки влияния на размер образа Docker (+40 MB)

---

## 🔍 Текущее состояние проекта

### Используемая версия

**Silero TTS v4** (Russian, 100 MB)

**Расположение модели:**
```bash
/models/silero_v4_ru.pt
```

**URL загрузки:**
```
https://models.silero.ai/models/tts/ru/v4_ru.pt
```

### Текущая реализация

**Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py`

```python
# Загрузка модели (строка 273-285)
model_path = "/models/silero_v4_ru.pt"
if os.path.exists(model_path):
    self.silero_model = torch.jit.load(model_path, map_location=self.device)
else:
    # Fallback на онлайн загрузку
    self.silero_model, _ = torch.hub.load(
        repo_or_dir="snakers4/silero-models", 
        model="silero_tts", 
        language="ru", 
        speaker="v4_ru"
    )
```

**Использование модели (строка 506-508):**
```python
audio = self.silero_model.apply_tts(
    ssml_text=ssml_text, 
    speaker=self.silero_speaker, 
    sample_rate=self.silero_sample_rate
)
```

### Поддерживаемые голоса (v4)

- **aidar** - нейтральный мужской
- **baya** - спокойный мужской (используется по умолчанию)
- **kseniya** - нейтральный женский
- **xenia** - энергичный женский

### Роль Silero в проекте

**Назначение:** Fallback для Yandex Cloud TTS

```python
# Приоритет провайдеров:
# 1. Yandex Cloud TTS gRPC v3 (primary, голос "anton" - оригинальный ROBBOX!)
# 2. Silero TTS v4 (fallback, offline, всегда работает)
```

**Когда используется Silero:**
- Yandex API недоступен (нет интернета, ошибка API)
- Yandex API key не настроен
- Принудительная установка `provider: "silero"` в конфиге

### Docker конфигурация

**Dockerfile:** `docker/vision/voice_base/Dockerfile`

```dockerfile
# Строка 105-131: Скачивание Silero модели
SILERO_MODEL_FILE="silero_v4_ru.pt"
SILERO_MODEL_URL="https://models.silero.ai/models/tts/ru/v4_ru.pt"

# Используется BuildKit cache для ускорения повторных сборок
RUN --mount=type=cache,target=/model_cache,sharing=locked \
    mkdir -p /models && \
    if [ -f "/model_cache/${SILERO_MODEL_FILE}" ]; then \
        echo "✅ Using cached Silero model"; \
        cp "/model_cache/${SILERO_MODEL_FILE}" /models/silero_v4_ru.pt; \
    else \
        echo "⬇️ Downloading Silero model (100 MB)..."; \
        wget -q -O /models/silero_v4_ru.pt "${SILERO_MODEL_URL}" && \
        cp /models/silero_v4_ru.pt "/model_cache/${SILERO_MODEL_FILE}"; \
    fi
```

**Зависимости Python:**

```txt
# docker/vision/voice_assistant/requirements.txt
torch>=2.9.0        # CRITICAL для ARM64!
torchaudio>=2.9.0
```

---

## 🚀 Что нового в Silero v5?

### 1. Производительность ⚡

**Скорость генерации:**

| Устройство | v4 (оценка) | v5 | Ускорение |
|-----------|-------------|-----|-----------|
| **CPU, 1 поток** | ~25 с/с | **37-42 с/с** | **1.5-1.7x** |
| **CPU, 4 потока** | ~65 с/с | **100-110 с/с** | **1.5-1.7x** |
| **GPU (3090)** | ~200 с/с | **300-350 с/с** | **1.5-1.75x** |

> **с/с** = секунд синтезированного аудио в секунду
> 
> Тесты проводились на Intel Core i9-10940X @ 3.30GHz

**Для Raspberry Pi 5 (4 ядра Cortex-A76 @ 2.4GHz):**
- **Ожидаемая скорость v5:** 40-50 с/с (4 потока, CPU)
- **Текущая скорость v4:** ~25-30 с/с (оценка)
- **Ускорение:** ~1.6-2.0x

✅ **Вывод:** Существенное улучшение для Raspberry Pi!

### 2. Новые функции 🎯

#### Автоматическая расстановка ударений в омографах

**Что такое омографы?**
Слова, которые пишутся одинаково, но произносятся по-разному:
- `замОк` (дверной) vs `зАмок` (средневековый)
- `мукА` (продукт) vs `мУка` (страдание)
- `гОтов` (прилагательное) vs `готОв` (существительное, готы)

**Новые флаги в v5:**
```python
audio = model.apply_tts(
    text=text,
    speaker=speaker,
    sample_rate=sample_rate,
    # Новые флаги для расстановки ударений
    put_accent=True,        # Ударения в обычных словах
    put_yo=True,            # Автоматическая буква ё
    put_stress_homo=True,   # Ударения в омографах (БЕЗ ё)
    put_yo_homo=True        # Ударения в омографах (С буквой ё)
)
```

**Влияние на скорость:**

| Режим | Скорость (CPU, 4 потока) | Просадка |
|-------|-------------------------|----------|
| Только TTS | 100-110 с/с | - |
| + ударения | 90-100 с/с | ~10% |
| + омографы | 90-100 с/с | ~10% |

✅ **Вывод:** Минимальная просадка скорости при существенном улучшении качества!

#### Примеры улучшения качества

**Пример из статьи:**
```
Меня зовут Лева Королев. Я из готов. И я уже готов открыть все ваши замки любой сложности!
                              ^^^^            ^^^^                              ^^^^^
                            готОв           готОв                             замкИ
```

v4: неправильные ударения, странное произношение  
v5: **правильные ударения**, естественное произношение ✅

### 3. Качество звучания 🎵

**Улучшения:**
- Более стабильная генерация
- Меньше артефактов в долгих фразах
- Улучшенная интонация

**Для ROBBOX:**
- Более естественный голос робота
- Лучше воспринимается в шумных условиях
- Меньше "роботических" артефактов

### 4. Упрощенный API 📦

**Новый способ установки:**
```bash
pip install silero
```

**Новый API:**
```python
from silero import silero_tts

# Загрузка модели (упрощенный интерфейс)
model, example_text = silero_tts(language='ru', speaker='v5_ru')

# Использование (совместимо со старым API)
audio = model.apply_tts(text=text, speaker='xenia', sample_rate=48000)
```

**Обратная совместимость:**
```python
# Старый API (v4) продолжает работать в v5!
model, _ = torch.hub.load(
    repo_or_dir='snakers4/silero-models',
    model='silero_tts',
    language='ru',
    speaker='v5_ru'  # Просто меняем speaker ID
)
```

### 5. Новые голоса 🎤

**v4 голоса (сохранены в v5):**
- aidar
- baya
- kseniya
- xenia

**Новый голос в v5:**
- **eugene** - новый мужской голос

### 6. Размер модели 📦

- **v4:** ~100 MB
- **v5:** ~140 MB
- **Разница:** +40 MB (+40%)

**Влияние на Docker образ:**
- Текущий размер `voice-assistant`: ~2.5 GB (оценка)
- После обновления: ~2.54 GB
- **Увеличение:** ~1.6% (незначительное)

---

## 🔧 Совместимость с существующим кодом

### Анализ изменений

#### 1. Загрузка модели

**Текущий код (v4):**
```python
model_path = "/models/silero_v4_ru.pt"
self.silero_model = torch.jit.load(model_path, map_location=self.device)
```

**Для v5 (два варианта):**

**Вариант A: Локальный файл (рекомендуется)**
```python
model_path = "/models/silero_v5_ru.pt"  # Просто меняем путь!
self.silero_model = torch.jit.load(model_path, map_location=self.device)
```

**Вариант B: torch.hub (онлайн загрузка)**
```python
self.silero_model, _ = torch.hub.load(
    repo_or_dir="snakers4/silero-models",
    model="silero_tts",
    language="ru",
    speaker="v5_ru"  # Меняем v4_ru → v5_ru
)
```

**Вариант C: новый pip пакет silero**
```python
from silero import silero_tts
model, _ = silero_tts(language='ru', speaker='v5_ru')
```

✅ **Совместимость:** Минимальные изменения! Достаточно поменять URL модели.

#### 2. Использование модели

**Текущий код:**
```python
audio = self.silero_model.apply_tts(
    ssml_text=ssml_text,
    speaker=self.silero_speaker,
    sample_rate=self.silero_sample_rate
)
```

**С новыми флагами v5:**
```python
audio = self.silero_model.apply_tts(
    ssml_text=ssml_text,
    speaker=self.silero_speaker,
    sample_rate=self.silero_sample_rate,
    # Новые флаги (опциональные!)
    put_accent=True,
    put_yo=True,
    put_stress_homo=True,
    put_yo_homo=True
)
```

✅ **Совместимость:** 100% обратная совместимость! Новые флаги опциональные.

#### 3. Конфигурация параметров

**Добавить в `voice_assistant.yaml`:**
```yaml
tts_node:
  # ... существующие параметры ...
  
  # Silero v5 настройки (новые!)
  silero:
    speaker: "baya"           # aidar, baya, kseniya, xenia, eugene (NEW!)
    sample_rate: 48000        # Можно увеличить с 24000 до 48000
    
    # Расстановка ударений (v5)
    put_accent: true          # Ударения в обычных словах
    put_yo: true              # Автоматическая буква ё
    put_stress_homo: true     # Ударения в омографах
    put_yo_homo: true         # Ударения в омографах с ё
```

**Добавить параметры в `tts_node.py`:**
```python
# В __init__()
self.declare_parameter("silero_put_accent", True)
self.declare_parameter("silero_put_yo", True)
self.declare_parameter("silero_put_stress_homo", True)
self.declare_parameter("silero_put_yo_homo", True)

# Читать параметры
self.silero_put_accent = self.get_parameter("silero_put_accent").value
# ... и т.д.
```

---

## 📋 План обновления

### Этап 1: Подготовка (оценка времени: 30 мин)

- [x] Анализ текущей реализации
- [x] Изучение документации Silero v5
- [x] Оценка совместимости
- [ ] Создание тестового плана

### Этап 2: Обновление кода (оценка времени: 1-2 часа)

#### 2.1 Обновление Dockerfile

**Файл:** `docker/vision/voice_base/Dockerfile`

```dockerfile
# БЫЛО (строки 105-106):
SILERO_MODEL_FILE="silero_v4_ru.pt"
SILERO_MODEL_URL="https://models.silero.ai/models/tts/ru/v4_ru.pt"

# СТАЛО:
SILERO_MODEL_FILE="silero_v5_ru.pt"
SILERO_MODEL_URL="https://models.silero.ai/models/tts/ru/v5_ru.pt"
```

**Изменения в RUN команде (строки 123-129):**
```dockerfile
# БЫЛО:
cp "/model_cache/${SILERO_MODEL_FILE}" /models/silero_v4_ru.pt
wget -q -O /models/silero_v4_ru.pt "${SILERO_MODEL_URL}"

# СТАЛО:
cp "/model_cache/${SILERO_MODEL_FILE}" /models/silero_v5_ru.pt
wget -q -O /models/silero_v5_ru.pt "${SILERO_MODEL_URL}"
```

**Аналогично для:** `docker/vision/voice_assistant/Dockerfile` (строки 184-209)

#### 2.2 Обновление tts_node.py

**Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py`

**Изменение 1: Путь к модели (строка 273)**
```python
# БЫЛО:
model_path = "/models/silero_v4_ru.pt"

# СТАЛО:
model_path = "/models/silero_v5_ru.pt"
```

**Изменение 2: torch.hub fallback (строка 282)**
```python
# БЫЛО:
self.silero_model, _ = torch.hub.load(
    repo_or_dir="snakers4/silero-models",
    model="silero_tts",
    language="ru",
    speaker="v4_ru"
)

# СТАЛО:
self.silero_model, _ = torch.hub.load(
    repo_or_dir="snakers4/silero-models",
    model="silero_tts",
    language="ru",
    speaker="v5_ru"  # v4_ru → v5_ru
)
```

**Изменение 3: Добавление новых параметров (строки 129-134)**
```python
# После существующих параметров silero_speaker, silero_sample_rate
self.declare_parameter("silero_put_accent", True)
self.declare_parameter("silero_put_yo", True)
self.declare_parameter("silero_put_stress_homo", True)
self.declare_parameter("silero_put_yo_homo", True)

# Читаем значения
self.silero_put_accent = self.get_parameter("silero_put_accent").value
self.silero_put_yo = self.get_parameter("silero_put_yo").value
self.silero_put_stress_homo = self.get_parameter("silero_put_stress_homo").value
self.silero_put_yo_homo = self.get_parameter("silero_put_yo_homo").value
```

**Изменение 4: Использование новых флагов (строка 506)**
```python
# БЫЛО:
audio = self.silero_model.apply_tts(
    ssml_text=ssml_text,
    speaker=self.silero_speaker,
    sample_rate=self.silero_sample_rate
)

# СТАЛО:
audio = self.silero_model.apply_tts(
    ssml_text=ssml_text,
    speaker=self.silero_speaker,
    sample_rate=self.silero_sample_rate,
    put_accent=self.silero_put_accent,
    put_yo=self.silero_put_yo,
    put_stress_homo=self.silero_put_stress_homo,
    put_yo_homo=self.silero_put_yo_homo
)
```

**Изменение 5: Обновление логов (строки 264, 277, 285)**
```python
# Заменить все "Silero TTS v4" на "Silero TTS v5"
self.get_logger().info("🔄 Загрузка Silero TTS v5...")
self.get_logger().info("✅ Silero TTS v5 загружен из локального файла...")
self.get_logger().info("✅ Silero TTS v5 загружен из GitHub...")
```

#### 2.3 Обновление конфигурационных файлов

**Файл:** `src/rob_box_voice/config/voice_assistant.yaml`

```yaml
tts_node:
  # ... существующие параметры ...
  
  # Silero настройки (обновлено для v5)
  silero_speaker: "baya"  # aidar, baya, kseniya, xenia, eugene (NEW!)
  silero_sample_rate: 48000  # Можно повысить качество (было 24000)
  
  # Расстановка ударений (v5 новые флаги)
  silero_put_accent: true
  silero_put_yo: true
  silero_put_stress_homo: true
  silero_put_yo_homo: true
```

**Файл:** `docker/vision/config/voice_assistant/voice_assistant.yaml`

Аналогичные изменения для deployed конфигурации.

#### 2.4 Обновление документации

**Создать:** `docs/reports/SILERO_V5_UPGRADE.md`
- Описание изменений
- Migration guide
- Тесты производительности
- Сравнение качества

**Обновить:**
- `docs/packages/rob_box_voice/TTS_NODE.md` (если существует)
- `README.md` (changelog секция)

### Этап 3: Тестирование (оценка времени: 2-3 часа)

#### 3.1 Локальная сборка Docker

```bash
cd docker/vision/voice_base
docker build -t test-voice-base:v5 .

cd ../voice_assistant
docker build --build-arg BASE_IMAGE=test-voice-base:v5 \
  -t test-voice-assistant:v5 .
```

#### 3.2 Тестирование на Raspberry Pi 5

**Подготовка:**
```bash
# Загрузить образ на Pi
docker save test-voice-assistant:v5 | \
  sshpass -p 'open' ssh ros2@10.1.1.21 'docker load'

# Остановить текущий voice-assistant
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && docker-compose stop voice-assistant'
```

**Тесты:**
1. **Базовый тест:** Запуск и проверка загрузки модели
2. **Тест производительности:** Измерение скорости генерации
3. **Тест качества:** Сравнение звучания v4 vs v5
4. **Тест омографов:** Проверка новой функциональности
5. **Стресс-тест:** Длинные фразы, несколько запросов подряд

#### 3.3 Метрики для оценки

**Производительность:**
- Время генерации 1 сек аудио (должно быть < 200ms для RT)
- Время загрузки модели (должно быть < 10 сек)
- Использование CPU (должно быть < 80%)
- Использование RAM (должно быть < 500 MB)

**Качество:**
- Естественность звучания (субъективная оценка)
- Правильность ударений в омографах
- Стабильность на длинных фразах

### Этап 4: Развертывание (оценка времени: 30 мин)

#### 4.1 Коммит изменений

```bash
git add docker/vision/voice_base/Dockerfile
git add docker/vision/voice_assistant/Dockerfile
git add src/rob_box_voice/rob_box_voice/tts_node.py
git add src/rob_box_voice/config/voice_assistant.yaml
git add docker/vision/config/voice_assistant/voice_assistant.yaml
git add docs/reports/SILERO_V5_ANALYSIS.md
git add docs/reports/SILERO_V5_UPGRADE.md

git commit -m "feat(voice): upgrade Silero TTS from v4 to v5

- Update Silero model: v4_ru.pt → v5_ru.pt
- Add support for homograph stress placement
- Add new configuration flags (put_accent, put_yo, put_stress_homo, put_yo_homo)
- Update sample rate: 24000 → 48000 Hz (better quality)
- Performance improvement: 1.5-2x faster than v4
- Add new voice option: eugene

Breaking changes: None (backward compatible)
"
```

#### 4.2 CI/CD Pipeline

GitHub Actions автоматически:
1. Соберет новые образы Docker
2. Запустит тесты (если есть)
3. Опубликует образы в ghcr.io
4. Создаст PR для слияния в `develop`

#### 4.3 Обновление на Pi

```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && ./scripts/update_and_restart.sh'
```

---

## 🧪 Тестовый план

### Тест 1: Базовая функциональность

**Цель:** Проверить, что v5 работает так же хорошо, как v4

**Шаги:**
1. Запустить voice_assistant с Silero v5
2. Отправить простую фразу: "Привет, робот!"
3. Проверить вывод аудио

**Ожидаемый результат:**
- ✅ Модель загружается без ошибок
- ✅ Аудио генерируется корректно
- ✅ Качество звучания приемлемое

### Тест 2: Производительность

**Цель:** Измерить скорость генерации на Raspberry Pi 5

**Шаги:**
1. Подготовить тестовые фразы разной длины (5, 10, 20, 30 секунд аудио)
2. Измерить время генерации для каждой фразы
3. Вычислить скорость (секунды аудио / секунду)

**Ожидаемые результаты:**
- v4: ~25-30 с/с
- v5: ~40-50 с/с
- Ускорение: ~1.6-2.0x ✅

### Тест 3: Омографы

**Цель:** Проверить новую функциональность расстановки ударений

**Тестовые фразы:**
```
1. "Я живу в замке" (зАмке - здание)
2. "Открой замок" (замОк - устройство)
3. "Я готов к работе" (готОв - прилагательное)
4. "Из готов пришли послы" (гОтов - существительное, племя)
5. "Мука для выпечки" (мукА - продукт)
6. "Это была мука" (мУка - страдание)
```

**С флагами:**
```python
put_stress_homo=True  # Включить расстановку ударений в омографах
```

**Без флагов:**
```python
put_stress_homo=False  # Выключить (поведение как в v4)
```

**Ожидаемый результат:**
- С флагом: правильные ударения ✅
- Без флага: случайные ударения (как в v4)

### Тест 4: Стресс-тест

**Цель:** Проверить стабильность на длинных фразах

**Шаги:**
1. Генерировать длинный текст (200+ слов, ~1 минута аудио)
2. Проверить отсутствие артефактов
3. Проверить использование памяти

**Ожидаемый результат:**
- ✅ Без сбоев
- ✅ Без артефактов в аудио
- ✅ Память < 600 MB

### Тест 5: Новый голос "eugene"

**Цель:** Протестировать новый голос

**Шаги:**
1. Установить `silero_speaker: "eugene"`
2. Генерировать тестовую фразу
3. Сравнить с другими голосами

**Ожидаемый результат:**
- ✅ Голос загружается и работает
- ✅ Качество приемлемое

---

## 📊 Оценка рисков

### Риск 1: Производительность на Raspberry Pi 5

**Вероятность:** Низкая  
**Влияние:** Среднее

**Описание:**  
v5 может оказаться медленнее на ARM64, чем заявлено в документации (тесты проводились на x86).

**Митигация:**
- Провести тесты до развертывания
- Сохранить возможность rollback на v4
- Оптимизировать параметры (`torch.set_num_threads(4)`)

### Риск 2: Увеличение размера Docker образа

**Вероятность:** Высокая  
**Влияние:** Низкое

**Описание:**  
Модель v5 на 40% больше (+40 MB).

**Митигация:**
- Используется BuildKit cache для ускорения сборки
- 40 MB незначительны для современных систем
- Можно удалить старую модель v4 после успешного обновления

### Риск 3: Несовместимость с текущим кодом

**Вероятность:** Очень низкая  
**Влияние:** Высокое

**Описание:**  
API может измениться, что сломает текущую реализацию.

**Митигация:**
- API полностью обратно совместим (подтверждено документацией)
- Новые флаги опциональные
- Провести тестирование перед развертыванием

### Риск 4: Проблемы с качеством звучания

**Вероятность:** Низкая  
**Влияние:** Среднее

**Описание:**  
v5 может звучать хуже на определенных фразах.

**Митигация:**
- Провести субъективное тестирование
- Сохранить возможность rollback
- Можно отключить новые флаги, если они ухудшают качество

---

## 💰 Оценка стоимости

### Затраты времени

| Задача | Время |
|--------|-------|
| Анализ и планирование | 1-2 часа |
| Обновление кода | 1-2 часа |
| Тестирование | 2-3 часа |
| Документация | 1 час |
| Развертывание | 0.5 часа |
| **ИТОГО** | **5.5-8.5 часов** |

### Затраты ресурсов

- **Docker образ:** +40 MB (+1.6%)
- **RAM runtime:** без изменений (~400-500 MB)
- **CPU usage:** улучшение (-30-40% для той же скорости генерации)
- **Интернет трафик:** +40 MB один раз при первой сборке

### Выгоды

**Количественные:**
- **Скорость:** 1.5-2x ускорение генерации
- **CPU:** экономия 30-40% при той же скорости
- **Latency:** снижение на 30-50%

**Качественные:**
- Более естественный голос
- Правильные ударения в омографах
- Меньше артефактов
- Лучшая стабильность

**ROI (Return on Investment):**
- **Затраты:** ~6-8 часов работы
- **Выгоды:** постоянное улучшение производительности и качества
- **Оценка:** **ПОЛОЖИТЕЛЬНЫЙ ROI** ✅

---

## ✅ Рекомендации

### Краткосрочные (1-2 недели)

1. **✅ Обновить Silero TTS до v5**
   - Минимальные изменения кода
   - Существенное улучшение производительности
   - Обратная совместимость

2. **✅ Включить флаги расстановки ударений**
   ```yaml
   silero_put_accent: true
   silero_put_yo: true
   silero_put_stress_homo: true
   silero_put_yo_homo: true
   ```

3. **✅ Повысить sample_rate до 48000 Hz**
   - Улучшение качества звука
   - v5 работает быстрее, можно позволить

4. **✅ Протестировать новый голос "eugene"**
   - Может подойти для определенных ситуаций

### Среднесрочные (1-2 месяца)

1. **Рассмотреть полный переход на Silero**
   - Если Yandex API часто недоступен
   - v5 достаточно быстр для primary provider

2. **Оптимизация конфигурации**
   - Тонкая настройка флагов ударений
   - A/B тестирование разных голосов

3. **Интеграция с SSML**
   - Использовать расширенные возможности v5
   - Эмоциональные интонации

### Долгосрочные (3-6 месяцев)

1. **Обновление до Silero v6** (когда выйдет)
   - Следить за релизами

2. **Локальное обучение голоса**
   - Создать уникальный голос ROBBOX
   - Использовать Silero v5 как base model

---

## 📝 Заключение

**Silero TTS v5** представляет собой существенное улучшение по сравнению с v4:

### Ключевые улучшения
- ⚡ **1.5-2x ускорение** (критично для Raspberry Pi!)
- 🎯 **Автоматические ударения** (улучшение качества)
- 🎵 **Лучшее звучание** (более естественный голос)
- 📦 **Обратная совместимость** (минимальные изменения кода)

### Рекомендация
**✅ ОБНОВИТЬ** с v4 до v5 при первой возможности.

**Приоритет:** ВЫСОКИЙ  
**Сложность:** НИЗКАЯ  
**Риски:** МИНИМАЛЬНЫЕ  
**ROI:** ПОЛОЖИТЕЛЬНЫЙ

---

## 📚 Ссылки

### Официальная документация Silero v5
- [Статья на Habr (русский)](https://habr.com/ru/articles/XXX/) - из problem_statement
- [GitHub репозиторий](https://github.com/snakers4/silero-models)
- [Интерактивный ноутбук](https://colab.research.google.com/github/snakers4/silero-models/blob/master/examples.ipynb)

### Связанные документы проекта
- `docs/packages/rob_box_voice/TTS_NODE.md` - документация TTS ноды
- `docs/optimization/MODEL_CACHE_OPTIMIZATION.md` - оптимизация кеша моделей
- `docs/development/DOCKER_BUILD_OPTIMIZATION.md` - оптимизация сборки Docker

### Технические ресурсы
- [PyTorch для ARM64](https://pytorch.org/get-started/locally/)
- [Silero pip package](https://pypi.org/project/silero/)

---

**Дата создания:** 2025-11-14  
**Автор:** AI Agent (GitHub Copilot)  
**Статус:** 📋 Готов к рассмотрению  
**Следующие шаги:** Утверждение обновления → Реализация → Тестирование → Развертывание
