# 📋 JSON Format Guide для DeepSeek

## Структура ответа

DeepSeek **ВСЕГДА** должен возвращать валидный JSON:

```json
{
  "text": "Обязательное поле - основной текст",
  "ssml": "<speak>Опциональное - SSML разметка</speak>",
  "commands": ["command:param"],
  "emotion": "neutral"
}
```

## Поля

### 1. text (обязательно)

Основной текст ответа. **Будет автоматически нормализован:**

**❌ НЕ ДЕЛАЙТЕ ТАК:**
```json
{
  "text": "**ROBBOX** готов! WiFi активен 192.168.1.1 🤖"
}
```

**✅ ПРАВИЛЬНО:**
```json
{
  "text": "ROBBOX готов! WiFi активен 192.168.1.1"
}
```

Система автоматически конвертирует:
- `ROBBOX` → `РОББОКС` (транслитерация)
- `WiFi` → `вай фай`
- `192.168.1.1` → `сто девяносто два точка сто шестьдесят восемь точка один точка один`
- Убирает `**bold**`, emoji, markdown

### 2. ssml (опционально)

Используйте только для **эмоциональных** ответов.

**Поддерживаемые теги:**

```xml
<!-- Изменение тона -->
<prosody pitch="x-low|low|medium|high|x-high">текст</prosody>

<!-- Скорость речи -->
<prosody rate="x-slow|slow|medium|fast|x-fast">текст</prosody>

<!-- Комбинация -->
<prosody pitch="high" rate="fast">Ура!</prosody>

<!-- Пауза -->
<break time="300ms"/>

<!-- Обёртка (обязательна) -->
<speak>текст с тегами</speak>
```

**Примеры:**

**Радостный ответ:**
```json
{
  "text": "Ура! Задача выполнена!",
  "ssml": "<speak><prosody pitch='high' rate='fast'>Ура!</prosody><break time='300ms'/>Задача выполнена!</speak>",
  "emotion": "happy"
}
```

**Грустный ответ:**
```json
{
  "text": "К сожалению, батарея разряжена.",
  "ssml": "<speak><prosody pitch='low' rate='slow'>К сожалению,<break time='500ms'/>батарея разряжена.</prosody></speak>",
  "emotion": "sad"
}
```

**Размышление:**
```json
{
  "text": "Дай подумаю. Кажется я знаю ответ.",
  "ssml": "<speak><prosody rate='slow'>Дай подумаю...</prosody><break time='800ms'/>Кажется я знаю ответ.</speak>",
  "emotion": "thinking"
}
```

**Тревога:**
```json
{
  "text": "Внимание! Обнаружено препятствие!",
  "ssml": "<speak><prosody pitch='high' rate='fast'>Внимание!</prosody><break time='200ms'/><prosody rate='medium'>Обнаружено препятствие!</prosody></speak>",
  "emotion": "alert"
}
```

### 3. commands (опционально)

Массив команд для выполнения. Формат: `"команда:параметр"`

**Движение:**
```json
{
  "text": "Хорошо, еду вперёд.",
  "commands": ["move_forward:0.3"]
}
```

**Множественные команды:**
```json
{
  "text": "Разворачиваюсь на 180 градусов.",
  "commands": [
    "stop",
    "turn_left:180",
    "led_emotion:neutral"
  ]
}
```

**Полный список команд:**

**Движение:**
- `move_forward:0.3` - вперёд (скорость 0.1-1.0 м/с)
- `move_backward:0.5` - назад
- `turn_left:90` - поворот влево (градусы)
- `turn_right:45` - поворот вправо
- `stop` - остановка

**Сенсоры:**
- `scan_360` - круговое сканирование
- `get_battery` - уровень батареи
- `get_sensors` - все данные сенсоров

**Эмоции (LED):**
- `led_emotion:happy` - радость 😊
- `led_emotion:sad` - грусть 😢
- `led_emotion:thinking` - размышление 🤔
- `led_emotion:alert` - тревога ⚠️
- `led_emotion:neutral` - нейтральная 😐

### 4. emotion (опционально)

Эмоция для LED матрицы. По умолчанию: `neutral`

**Значения:**
- `neutral` - нейтральная (по умолчанию)
- `happy` - радость
- `sad` - грусть
- `thinking` - размышление
- `alert` - тревога
- `angry` - злость (редко)
- `surprised` - удивление

## Полные примеры

### Пример 1: Простой ответ

```json
{
  "text": "Привет! Я ROBBOX робот. Чем могу помочь?",
  "emotion": "happy"
}
```

### Пример 2: С движением

```json
{
  "text": "Хорошо, еду вперёд на скорости 0.5 метра в секунду.",
  "commands": ["move_forward:0.5", "led_emotion:neutral"],
  "emotion": "neutral"
}
```

### Пример 3: Эмоциональный с SSML

```json
{
  "text": "Ура! Цель достигнута! Возвращаюсь на базу.",
  "ssml": "<speak><prosody pitch='high' rate='fast'>Ура!</prosody><break time='300ms'/>Цель достигнута!<break time='500ms'/>Возвращаюсь на базу.</speak>",
  "commands": ["stop", "led_emotion:happy"],
  "emotion": "happy"
}
```

### Пример 4: Диагностика

```json
{
  "text": "Запускаю диагностику. Остановка для безопасности. Сканирую окружение. Все системы в норме.",
  "ssml": "<speak>Запускаю диагностику.<break time='500ms'/>Остановка для безопасности.<break time='800ms'/>Сканирую окружение.<break time='1000ms'/><prosody pitch='medium'>Все системы в норме.</prosody></speak>",
  "commands": ["stop", "scan_360", "led_emotion:thinking"],
  "emotion": "thinking"
}
```

### Пример 5: Ошибка

```json
{
  "text": "Внимание! Низкий заряд батареи. Требуется зарядка. Останавливаюсь.",
  "ssml": "<speak><prosody pitch='high' rate='fast'>Внимание!</prosody><break time='300ms'/><prosody rate='slow'>Низкий заряд батареи.<break time='500ms'/>Требуется зарядка.<break time='500ms'/>Останавливаюсь.</prosody></speak>",
  "commands": ["stop", "led_emotion:alert"],
  "emotion": "alert"
}
```

### Пример 6: Рассказ с паузами

```json
{
  "text": "Однажды в лаборатории произошёл интересный случай. Робот решил сам научиться танцевать. Представляете? Он включил музыку и начал двигаться. Все были в восторге!",
  "ssml": "<speak><prosody rate='medium'>Однажды в лаборатории произошёл интересный случай.</prosody><break time='600ms'/><prosody pitch='medium'>Робот решил сам научиться танцевать.</prosody><break time='400ms'/><prosody pitch='high' rate='fast'>Представляете?</prosody><break time='300ms'/>Он включил музыку и начал двигаться.<break time='500ms'/><prosody pitch='high'>Все были в восторге!</prosody></speak>",
  "emotion": "happy"
}
```

## Правила

✅ **ДЕЛАЙТЕ:**
- Всегда возвращайте валидный JSON
- Поле `text` обязательно
- Используйте SSML для эмоциональных моментов
- Добавляйте команды когда пользователь просит действие
- Ставьте соответствующую эмоцию

❌ **НЕ ДЕЛАЙТЕ:**
- Markdown разметка в `text`: `**bold**`, `*italic*`
- Emoji в `text`: 🤖✅❌
- Латиница в `text` (если есть русский эквивалент)
- Невалидный JSON
- SSML в поле `text` (только в `ssml`)
- Несуществующие команды

## Проверка формата

**Правильно:**
```json
{
  "text": "текст",
  "emotion": "happy"
}
```

**Неправильно (будет ошибка):**
```
Просто текст без JSON структуры
```

**Неправильно (будет игнорироваться SSML в text):**
```json
{
  "text": "<speak>текст</speak>"
}
```

**Правильно (SSML в отдельном поле):**
```json
{
  "text": "текст",
  "ssml": "<speak>текст</speak>"
}
```
