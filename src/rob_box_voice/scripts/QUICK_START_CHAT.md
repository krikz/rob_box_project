# 🚀 ROBBOX Chat - Быстрый старт

## Запуск чата

```bash
cd ~/rob_box_project
source src/rob_box_voice/.env.secrets
cd src/rob_box_voice/scripts
python3 robbox_chat.py
```

## Примеры вопросов

```
👤 Вы: привет расскажи о себе
👤 Вы: поезжай вперёд на скорости 0.5
👤 Вы: стоп остановись
👤 Вы: включи счастливые эмоции
👤 Вы: расскажи страшную историю
👤 Вы: что ты видишь вокруг?
👤 Вы: сделай круговое сканирование
```

## Команды чата

- `/quit` - выход
- `/clear` - очистить историю
- `/history` - показать историю

## Что происходит внутри

1. **DeepSeek генерирует JSON:**
```json
{
  "text": "Хорошо, еду вперёд.",
  "ssml": "<speak>Хорошо,<break time='300ms'/>еду вперёд.</speak>",
  "commands": ["move_forward:0.5", "led_emotion:neutral"],
  "emotion": "neutral"
}
```

2. **Text Normalizer обрабатывает:**
   - Убирает markdown
   - Конвертирует числа: `0.5` → `ноль точка пять`
   - Транслитерирует: `ROBBOX` → `РОББОКС`
   - Извлекает команды

3. **CommandExecutor выполняет:**
   - `🤖 КОМАНДА: move_forward [0.5]`
   - `🤖 КОМАНДА: led_emotion [neutral]`

4. **Silero TTS озвучивает:**
   - Синтез речи: `aidar` voice, `x-slow` rate
   - Pitch shift 2.0x (режим "Бурундук")
   - Воспроизведение: 24kHz → 48kHz

5. **SSML добавляет паузы:**
   - `🔊 Говорю: Хорошо,`
   - `⏸️ Пауза 300ms...`
   - `🔊 Говорю: еду вперёд.`

## Настройки

### Изменить голос

В `robbox_chat.py`:
```python
self.tts.synthesize_and_play(
    text,
    speaker='aidar',     # aidar, baya, kseniya, xenia
    rate='x-slow',       # x-slow, slow, medium, fast
    pitch='medium'       # low, medium, high, x-high
)
```

### Отключить "Бурундук"

```python
self.tts = SileroTTS(
    sample_rate=24000,
    chipmunk_mode=False  # Отключить pitch shift
)
```

### Изменить температуру DeepSeek

```python
response = self.client.chat.completions.create(
    model="deepseek-chat",
    temperature=0.7,  # 0.0 = строго, 1.0 = креативно
    max_tokens=500
)
```

## Полная документация

Смотри: `ROBBOX_CHAT_README.md`

## Тесты

```bash
# Тест нормализатора
python3 text_normalizer_v2.py

# Тест GUI с TTS
python3 silero_tts_gui.py
```
