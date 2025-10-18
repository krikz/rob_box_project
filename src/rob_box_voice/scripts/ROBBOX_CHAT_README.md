# 🤖 ROBBOX Interactive Chat

Интерактивный чат с ROBBOX роботом, использующий DeepSeek API, Silero TTS и систему команд.

## 🎯 Возможности

- **Голосовые ответы** с эффектом "Бурундук" (как у настоящего ROBBOX)
- **SSML интонации** для выразительной речи (паузы, тон, скорость)
- **Команды роботу** (движение, сканирование, LED эмоции)
- **Нормализация текста** (числа → слова, латиница → кириллица)
- **JSON формат ответов** от DeepSeek

## 🚀 Быстрый старт

### 1. Установка зависимостей

```bash
cd ~/rob_box_project/src/rob_box_voice
python3 -m pip install torch sounddevice openai
```

### 2. Настройка API ключа

```bash
# Добавьте в .env.secrets:
export DEEPSEEK_API_KEY='your-deepseek-api-key'

# Загрузите переменные:
source .env.secrets
```

### 3. Запуск чата

```bash
cd scripts
python3 robbox_chat.py
```

## 💬 Использование

### Команды чата

- `/quit` - Выход из чата
- `/clear` - Очистить историю диалога
- `/history` - Показать историю

### Примеры вопросов

**Простые вопросы:**
```
👤 Вы: привет, как дела?
👤 Вы: расскажи о себе
👤 Вы: что ты умеешь?
```

**С командами:**
```
👤 Вы: поезжай вперёд
👤 Вы: стоп, остановись
👤 Вы: включи счастливые эмоции
👤 Вы: сделай круговое сканирование
```

**С интонациями:**
```
👤 Вы: расскажи страшную историю
👤 Вы: расскажи весёлую историю
👤 Вы: я расстроен
```

## 📋 Формат ответа DeepSeek

DeepSeek возвращает структурированный JSON:

```json
{
  "text": "Обычный текст (будет нормализован)",
  "ssml": "<speak><prosody rate='slow'>Текст с интонациями</prosody><break time='500ms'/>Продолжение</speak>",
  "commands": ["move_forward:0.3", "led_emotion:happy"],
  "emotion": "neutral"
}
```

### Поля

- **text** (обязательно) - основной текст ответа
- **ssml** (опционально) - SSML разметка для интонаций
- **commands** (опционально) - массив команд для выполнения
- **emotion** (опционально) - эмоция: `neutral`, `happy`, `sad`, `thinking`, `alert`

## 🎭 SSML теги

Silero TTS поддерживает:

```xml
<!-- Изменение тона -->
<prosody pitch="low|medium|high|x-high">текст</prosody>

<!-- Скорость речи -->
<prosody rate="x-slow|slow|medium|fast">текст</prosody>

<!-- Пауза -->
<break time="500ms"/>

<!-- Обёртка -->
<speak>текст с тегами</speak>
```

### Пример с интонациями

```json
{
  "text": "Ура! Задача выполнена.",
  "ssml": "<speak><prosody pitch='high' rate='fast'>Ура!</prosody><break time='300ms'/>Задача выполнена.</speak>",
  "emotion": "happy"
}
```

## 🤖 Команды робота

### Движение

- `move_forward:speed` - движение вперёд (speed: 0.1-1.0 м/с)
- `move_backward:speed` - движение назад
- `turn_left:angle` - поворот влево (градусы)
- `turn_right:angle` - поворот вправо
- `stop` - полная остановка

### Сенсоры

- `scan_360` - круговое сканирование лидаром
- `get_battery` - запрос уровня батареи
- `get_sensors` - запрос данных всех сенсоров

### Эмоции (LED матрица)

- `led_emotion:happy` - радость 😊
- `led_emotion:sad` - грусть 😢
- `led_emotion:thinking` - размышление 🤔
- `led_emotion:alert` - тревога ⚠️
- `led_emotion:neutral` - нейтральная 😐

### Пример с командами

```json
{
  "text": "Хорошо, еду вперёд.",
  "commands": ["move_forward:0.3", "led_emotion:neutral"],
  "emotion": "neutral"
}
```

## 🧪 Тестирование

### Text Normalizer v2

```bash
cd scripts
python3 text_normalizer_v2.py
```

Протестирует:
- Парсинг JSON
- Обработку SSML
- Выполнение команд
- Нормализацию текста

### Silero TTS GUI

```bash
cd scripts
python3 silero_tts_gui.py
```

Тестирование голоса с бурундуком:
- Включите чекбокс "🐿️ Эффект 'Бурундук'"
- Выберите speaker: `aidar`
- Выберите rate: `x-slow`
- Pitch: `medium`
- Sample rate: `24000`

## 📁 Структура

```
scripts/
├── robbox_chat.py           # Главный чат
├── text_normalizer.py       # Базовый нормализатор
├── text_normalizer_v2.py    # Расширенный (JSON, SSML, команды)
├── silero_tts_gui.py        # GUI для тестирования TTS
└── test_yandex_v3.py        # Тесты Yandex TTS

prompts/
└── master_prompt.txt        # Системный промпт для DeepSeek
```

## 🔧 Настройка

### Параметры TTS

В `robbox_chat.py`:

```python
self.tts = SileroTTS(
    sample_rate=24000,      # 24kHz (будет 48kHz с бурундуком)
    chipmunk_mode=True      # Эффект "бурундук"
)
```

### Параметры DeepSeek

```python
response = self.client.chat.completions.create(
    model="deepseek-chat",
    messages=messages,
    temperature=0.7,        # Креативность (0.0-1.0)
    max_tokens=500          # Длина ответа
)
```

## 🐛 Отладка

### DeepSeek не возвращает JSON

Проверьте `master_prompt.txt` - должны быть инструкции по формату JSON.

### Нет звука

```bash
# Проверьте устройства вывода:
python3 -c "import sounddevice as sd; print(sd.query_devices())"

# Проверьте Silero TTS:
python3 silero_tts_gui.py
```

### Команды не выполняются

В текущей версии команды только печатаются. Для реального выполнения нужна ROS2 интеграция:

```python
# В CommandExecutor.execute():
self.command_pub.publish(Command(name=command, param=param))
```

## 📚 Примеры диалогов

### Пример 1: Приветствие

```
👤 Вы: привет
🤖 ROBBOX [😊 happy]:
   🔊 Говорю: Привет!
   🐿️ Режим 'Бурундук': pitch shift 2.0x
   ⏸️ Пауза 200ms...
   🔊 Говорю: Я РОББОКС робот.
   🐿️ Режим 'Бурундук': pitch shift 2.0x
   🔊 Говорю: Рад вас видеть!
```

### Пример 2: Команда движения

```
👤 Вы: поезжай вперёд на скорости 0.5
🤖 ROBBOX [😐 neutral]:
   ⚙️ ВЫПОЛНЕНИЕ КОМАНД:
   🤖 КОМАНДА: move_forward [0.5]
   🤖 КОМАНДА: led_emotion [neutral]
   🔊 Говорю: Хорошо, еду вперёд.
```

### Пример 3: Эмоциональный ответ

```
👤 Вы: расскажи весёлую историю
🤖 ROBBOX [😊 happy]:
   🔊 Говорю: Хорошо!
   ⏸️ Пауза 300ms...
   🔊 Говорю: Слушайте...
   ⏸️ Пауза 500ms...
   🔊 Говорю: Однажды я решил научиться танцевать...
```

## 🎓 Технологии

- **DeepSeek Chat API** - генерация ответов
- **Silero TTS v4** - синтез речи (русский)
- **Text Normalization** - обработка текста
- **SSML** - интонации и паузы
- **Pitch Shift** - эффект "бурундук" (2.0x)

## 📝 TODO

- [ ] ROS2 интеграция для команд
- [ ] Поддержка voice input (STT)
- [ ] Визуализация эмоций в терминале
- [ ] Логирование диалогов
- [ ] Web интерфейс

## 📄 Лицензия

Часть проекта rob_box_project
