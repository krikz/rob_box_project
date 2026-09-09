# 🤖 Robot Control GUI

Графический интерфейс для управления и мониторинга робота РОББОКС.

## 🎯 Возможности

### 📢 TTS Команды
- **Готовые фразы**: Привет, Еду вперед, Стоп, Думаю, Ошибка, Анекдот
- **Свой текст**: Отправка произвольного текста в TTS
- **SSML поддержка**: Расширенное форматирование речи

### 🎨 Управление анимациями
- `idle` - спокойное состояние
- `listening` - прослушивание
- `talking` - разговор
- `thinking` - размышление
- `happy` - радость
- `sad` - грусть
- `alert` - тревога

### 🔔 Звуки
- `cute` - милый звук
- `confused` - замешательство
- `sad` - грустный звук
- `thinking` - думает
- `positive` - позитивный

### 📊 Мониторинг
- **STT**: Сообщения от пользователя (распознанная речь)
- **DeepSeek**: JSON ответы от LLM
- **TTS**: Что робот говорит (чистый текст)
- **System Log**: Системные события

## 🚀 Запуск

```bash
# Простой запуск
./scripts/start_robot_gui.sh

# Или напрямую
python3 tools/robot_control_gui.py
```

## 📋 Требования

- Python 3.10+
- ROS2 Humble
- tkinter (обычно установлен по умолчанию)
- rclpy

Проверка tkinter:
```bash
python3 -c "import tkinter; print('✅ tkinter установлен')"
```

Если tkinter отсутствует:
```bash
sudo apt install python3-tk
```

## 🔌 ROS2 Topics

### Publishers (отправляет)
- `/voice/tts/request` - TTS команды
- `/voice/stt/result` - Имитация STT
- `/voice/animation/request` - Переключение анимаций
- `/voice/sound/trigger` - Воспроизведение звуков

### Subscribers (получает)
- `/voice/stt/result` - Сообщения от пользователя
- `/voice/dialogue/response` - Ответы DeepSeek
- `/voice/tts/request` - Мониторинг TTS

## 💡 Примеры использования

### Отправка TTS команды
Нажмите на кнопку "Привет" или введите свой текст и нажмите "Отправить текст".

### Переключение анимации
Нажмите на нужную анимацию (например, "Happy"), робот сменит выражение LED матрицы.

### Воспроизведение звука
Нажмите на звук (например, "Cute"), робот воспроизведёт соответствующий звуковой эффект.

### Мониторинг
Все события отображаются в соответствующих окнах:
- **🎤 STT** - что говорит пользователь
- **🤖 DeepSeek** - JSON ответы с SSML
- **🔊 TTS** - что произносит робот (без тегов)
- **📋 System Log** - системные события и ошибки

## 🎨 Интерфейс

```
┌──────────────────────────────────────────────────────────┐
│                   🤖 РОБОКС Control Panel                 │
├─────────────────────┬────────────────────────────────────┤
│  🎛️ Управление      │  🎤 STT Messages                   │
│                     │  (что говорит пользователь)        │
│  [Привет]           ├────────────────────────────────────┤
│  [Еду вперед]       │  🤖 DeepSeek Responses             │
│  [Стоп]             │  (JSON с SSML)                     │
│  [Думаю]            ├────────────────────────────────────┤
│  [Ошибка]           │  🔊 TTS Output                     │
│  [Анекдот]          │  (что произносит робот)            │
│                     ├────────────────────────────────────┤
│  ✏️ Свой текст:     │  📋 System Log                     │
│  [_______________]  │  (события и ошибки)                │
│  [Отправить]        │                                    │
│                     │                                    │
│  🎨 Анимации:       │                                    │
│  [Idle] [Happy]...  │                                    │
│                     │                                    │
│  🔔 Звуки:          │                                    │
│  [Cute] [Sad]...    │                                    │
└─────────────────────┴────────────────────────────────────┘
│         [🗑️ Очистить логи]  [❌ Выход]                   │
└──────────────────────────────────────────────────────────┘
```

## 🔧 Настройка

### Zenoh подключение
По умолчанию GUI подключается к роботу `RBXU100001`. Для другого робота измените в `start_robot_gui.sh`:

```bash
export ROBOT_ID=YOUR_ROBOT_ID
```

### Добавление своих TTS шаблонов
Отредактируйте `robot_control_gui.py`, секция `self.tts_templates`:

```python
self.tts_templates = {
    "Моя фраза": '{"ssml": "<speak>Текст.<break time=\'300ms\'/></speak>", "emotion": "neutral"}',
    # ...
}
```

## 🐛 Troubleshooting

### GUI не запускается
```bash
# Проверьте tkinter
python3 -c "import tkinter"

# Проверьте ROS2
ros2 topic list
```

### Нет связи с роботом
```bash
# Проверьте Zenoh connection
ros2 topic list | grep voice

# Проверьте IP робота
ping 10.1.1.21
```

### Логи не обновляются
Убедитесь, что:
1. Робот запущен и доступен
2. Topics существуют: `ros2 topic list`
3. GUI подключён к правильному namespace

## 📝 Формат TTS JSON

```json
{
  "ssml": "<speak>Текст с паузами.<break time='300ms'/></speak>",
  "emotion": "neutral",
  "commands": ["move_forward:0.3"]
}
```

### Поля:
- `ssml` (обязательно) - текст с SSML разметкой
- `emotion` (опционально) - neutral, happy, sad, thinking, alert
- `commands` (опционально) - массив команд движения

### SSML теги:
- `<break time="300ms"/>` - пауза
- `<prosody pitch="high">текст</prosody>` - высокий тон
- `<prosody rate="slow">текст</prosody>` - медленная скорость
- `<emphasis>текст</emphasis>` - эмфазис

## 📚 См. также

- [Voice Assistant Documentation](../docs/packages/rob_box_voice/)
- [SSML Guide](../docs/packages/rob_box_voice/SSML_GUIDE.md)
- [Animations](../docs/packages/rob_box_animations/)
