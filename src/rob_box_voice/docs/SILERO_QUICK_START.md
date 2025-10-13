# Silero TTS для ROBBOX - Быстрый старт

## Что изменилось

**✅ TTS движок заменён:** Piper → Silero v4

**Почему:**
- ⚡ **Быстрее realtime**: RTF 0.3-0.5 на Pi 5 (в 2-3 раза быстрее воспроизведения)
- 🎭 **SSML поддержка**: Можно менять pitch (тон), rate (скорость), паузы
- 🗣️ **4 голоса**: 2 мужских (aidar, baya), 2 женских (kseniya, xenia)

## Как использовать SSML

### Изменение pitch (тон голоса)

```python
# Низкий голос (серьёзный робот)
ssml = '<speak><prosody pitch="low">Внимание! Препятствие!</prosody></speak>'

# Высокий голос (дружелюбный робот)
ssml = '<speak><prosody pitch="high">Привет! Как дела?</prosody></speak>'

# Нормальный голос
ssml = '<speak><prosody pitch="medium">Обычная фраза</prosody></speak>'
```

**Значения pitch:**
- `x-low` - очень низкий (мощный робот)
- `low` - низкий (серьёзный)
- `medium` - нормальный (по умолчанию)
- `high` - высокий (дружелюбный)
- `x-high` - очень высокий (весёлый робот)

### Изменение rate (скорость речи)

```python
# Медленная речь (объяснение)
ssml = '<speak><prosody rate="slow">Сейчас объясню подробно</prosody></speak>'

# Быстрая речь (срочное сообщение)
ssml = '<speak><prosody rate="fast">Батарея разряжается!</prosody></speak>'
```

**Значения rate:**
- `x-slow` - очень медленно
- `slow` - медленно
- `medium` - нормально
- `fast` - быстро
- `x-fast` - очень быстро

### Комбинирование параметров

```python
# Низкий голос + медленная речь (предупреждение)
ssml = '<speak><prosody pitch="low" rate="slow">Предупреждение: препятствие впереди</prosody></speak>'

# Высокий голос + быстрая речь (радость)
ssml = '<speak><prosody pitch="high" rate="fast">Ура! Задача выполнена!</prosody></speak>'
```

### Паузы

```python
# Пауза между предложениями
ssml = '<speak>Первое предложение.<break time="1s"/>Второе предложение.</speak>'

# Пауза для размышления
ssml = '<speak>Дай подумаю...<break time="2s"/>Кажется, я знаю ответ.</speak>'
```

## Конфигурация

### Файл: `src/rob_box_voice/config/voice_assistant.yaml`

```yaml
tts_node:
  provider: "silero"
  
  silero:
    model_path: "/models/silero_v4_ru.pt"
    speaker: "aidar"          # Выбор голоса
    sample_rate: 48000        # Качество звука
    threads: 4                # CPU threads (для Pi 5)
    pitch: "medium"           # Базовый тон
    rate: "medium"            # Базовая скорость
    use_ssml: true            # Включить SSML
```

### Голоса

**Мужские:**
- **aidar** ✅ - нейтральный, чёткий, серьёзный (РЕКОМЕНДУЕТСЯ)
- **baya** - спокойный, мягкий, дружелюбный

**Женские:**
- **kseniya** - нейтральный, профессиональный
- **xenia** ✅ - энергичный, живой (РЕКОМЕНДУЕТСЯ для позитивного робота)

## Тестирование

```bash
cd src/rob_box_voice/scripts
python3 test_tts_voices.py --engine silero

# Тест конкретного голоса
python3 test_tts_voices.py --engine silero --voice aidar

# Тест SSML
python3 test_tts_voices.py --engine silero --text '<speak><prosody pitch="low">Низкий голос</prosody></speak>'
```

## Рекомендации для разных ситуаций

### 1. Серьёзный робот (охранник, помощник)
```yaml
silero:
  speaker: "aidar"
  pitch: "low"
  rate: "medium"
```

### 2. Дружелюбный робот (компаньон)
```yaml
silero:
  speaker: "xenia"
  pitch: "medium"
  rate: "medium"
```

### 3. Маленький робот (игрушка)
```yaml
silero:
  speaker: "xenia"
  pitch: "high"
  rate: "fast"
```

### 4. Эмоциональные фразы

```python
# Радость
ssml = '<speak><prosody pitch="high" rate="fast">Отлично! Выполнено!</prosody></speak>'

# Предупреждение
ssml = '<speak><prosody pitch="low" rate="slow">Внимание! Препятствие!</prosody></speak>'

# Размышление
ssml = '<speak>Дай подумаю...<break time="1s"/><prosody rate="slow">Кажется, я знаю ответ</prosody></speak>'

# Срочное сообщение
ssml = '<speak><prosody pitch="high" rate="x-fast">Батарея разряжается!</prosody></speak>'
```

## Производительность

**Raspberry Pi 5:**
- RTF: **0.3-0.5** (в 2-3 раза быстрее воспроизведения) ⚡
- Латентность: **300-500ms** для фразы "Привет!"
- RAM: **~200 MB**
- CPU: **60-80%** одного ядра

**Это быстрее чем realtime!** 🚀

## Полная документация

- **Полное руководство:** `docs/development/SILERO_SSML_CONFIGURATION.md`
- **Тестовый скрипт:** `src/rob_box_voice/scripts/test_tts_voices.py`
- **Исследование Pi 5:** `docs/development/SILERO_TTS_RASPBERRY_PI.md`
- **Официальная SSML Wiki:** https://github.com/snakers4/silero-models/wiki/SSML

## Что дальше?

1. ✅ Протестируй голоса: `python3 test_tts_voices.py --engine silero`
2. 🎨 Выбери голос и pitch для характера робота
3. 🎭 Используй SSML для эмоциональных фраз
4. 🚀 Запусти в Docker: образ обновлён, модель загружается автоматически

**Рекомендация:** Начни с **aidar + pitch="low"** для серьёзного робота или **xenia + pitch="medium"** для дружелюбного!
