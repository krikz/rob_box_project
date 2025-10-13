# Оптимизация JSON формата: удаление поля `text`

**Дата:** 13 октября 2025
**Ветка:** feature/voice-assistant

## Проблема

При streaming ответах DeepSeek возвращал JSON с двумя избыточными полями:
- `text` - простой текст без ударений
- `ssml` - SSML с ударениями и паузами

Поскольку система воспроизводит **только SSML**, поле `text` было полностью избыточным и увеличивало:
1. Размер каждого JSON chunk на ~38%
2. Время первого ответа (больше данных для передачи)
3. Нагрузку на DeepSeek (генерировать два представления текста)

## Решение

Убрали поле `text` из формата ответа. Теперь DeepSeek возвращает только:
- `ssml` - SSML с ударениями, паузами и интонациями (ОБЯЗАТЕЛЬНО)
- `emotion` - эмоция для LED матрицы (опционально)
- `commands` - команды управления роботом (опционально)

## Изменения

### 1. Промпт (`master_prompt.txt`)

**Было:**
```json
{
  "text": "Теорема Пифагора гласит: ...",
  "ssml": "<speak>Теор+ема Пиф+агора глас+ит: ...</speak>",
  "emotion": "thinking"
}
```

**Стало:**
```json
{
  "ssml": "<speak>Теор+ема Пиф+агора глас+ит: ...</speak>",
  "emotion": "thinking"
}
```

**Ключевые изменения:**
- Убрано требование поля `text` ("обязательно должно быть")
- Добавлено "НЕ добавляйте поле `text` - оно избыточно"
- Добавлено "Поле `ssml` ОБЯЗАТЕЛЬНО должно быть и содержать весь текст"
- Обновлены все примеры в промпте

### 2. Код парсинга (`robbox_chat_streaming.py`)

**Было:**
```python
def _speak_chunk(self, chunk_data: dict):
    text = chunk_data.get('text', '')
    normalized = self.voice_handler.parser.base_normalizer.normalize(text)
    
    ssml = chunk_data.get('ssml', None)
    if ssml:
        # Используем SSML
        ...
    else:
        # Озвучиваем text
        self.tts.synthesize_and_play(normalized, speaker='aidar')
```

**Стало:**
```python
def _speak_chunk(self, chunk_data: dict):
    ssml = chunk_data.get('ssml', '')
    
    if not ssml:
        print("⚠️  Пустой SSML в chunk, пропускаем")
        return
    
    # Парсим SSML и озвучиваем
    ssml_chunks = self.voice_handler.ssml_processor.parse_ssml_for_timing(ssml)
    for ssml_text, pause_ms in ssml_chunks:
        normalized = self.voice_handler.parser.base_normalizer.normalize(ssml_text)
        self.tts.synthesize_and_play(normalized, speaker='aidar')
        if pause_ms:
            time.sleep(pause_ms / 1000.0)
```

**Ключевые изменения:**
- Убран fallback на поле `text`
- SSML стал единственным источником текста
- Нормализация (латинские буквы) применяется к SSML
- Добавлена проверка на пустой SSML с предупреждением

### 3. Улучшен парсинг JSON

Также был улучшен парсинг для работы с многострочными JSON (DeepSeek иногда форматирует JSON с отступами):

**Было:**
- Парсинг построчно (\n-delimited JSON)
- Не работало с форматированным JSON

**Стало:**
- Парсинг с подсчётом фигурных скобок `{}`
- Работает и с компактным, и с форматированным JSON
- Автоматически игнорирует markdown блоки (\`\`\`json ... \`\`\`)

```python
# Новый алгоритм
brace_count = 0
in_json = False
current_chunk = ""

for token in stream:
    if '{' in token and not in_json:
        in_json = True
        current_chunk = token[token.index('{'):]
        brace_count = current_chunk.count('{') - current_chunk.count('}')
    
    elif in_json:
        current_chunk += token
        brace_count += token.count('{') - token.count('}')
        
        if brace_count == 0:  # Скобки сбалансированы = полный JSON
            chunk_data = json.loads(current_chunk)
            _speak_chunk(chunk_data)
            # Сброс для следующего chunk
            current_chunk = ""
            in_json = False
```

## Результаты

### Экономия трафика

**Пример chunk:**
- **Старый формат:** 510 байт
- **Новый формат:** 321 байт
- **Экономия:** 189 байт (**38% меньше**)

**Для типичного ответа (6 chunks):**
- **Старый:** ~3 КБ
- **Новый:** ~1.9 КБ
- **Экономия:** ~1.1 КБ (**38% меньше**)

### Улучшение времени отклика

Меньший JSON → быстрее передача → **быстрее первый chunk**

Для первого chunk (~300 байт):
- Экономия: ~110 байт
- На 4G (20 Мбит/с ≈ 2.5 МБ/с): **~0.04 мс** экономии
- На WiFi: незначительно

**Основная выгода:** Меньше нагрузка на DeepSeek (генерировать один текст вместо двух)

### Упрощение кода

- Убран ненужный fallback на `text`
- Единая логика обработки (только SSML)
- Меньше мест для ошибок

## Тестирование

### Тест 1: Короткий ответ
```bash
$ python3 -c "bot.ask_deepseek_streaming('Привет')"
📦 Chunk 1: {"ssml": "<speak><prosody pitch='high'>Прив+ет!</prosody>..."}
🔊 Говорю: Прив+ет!
```
✅ Работает, поле `text` отсутствует

### Тест 2: Длинный ответ (6 chunks)
```bash
$ python3 -c "bot.ask_deepseek_streaming('расскажи про Сочи подробно')"
📦 Chunk 1: {"chunk": 1, "ssml": "<speak>С+очи..."}
🔊 Говорю: С+очи— это кр+упный кур+ортный г+ород...
📦 Chunk 2: {"chunk": 2, "ssml": "<speak>Распол+ожен..."}
🔊 Говорю: Распол+ожен на ч+ерноморском...
...
📦 Chunk 6: {"chunk": 6, "ssml": "<speak>В С+очи нах+одятся..."}
```
✅ Работает, все 6 chunks озвучены

### Тест 3: Формулы и ударения
```bash
$ python3 -c "bot.ask_deepseek_streaming('теорема пифагора с формулой')"
📦 Chunk 1: {"ssml": "<speak>Теор+ема Пиф+агора глас+ит:..."}
🔊 Говорю: Теор+ема Пиф+агора глас+ит: в прямоуг+ольном...
```
✅ Работает, ударения правильные, формула текстом

## Выводы

✅ **Успешно убрали избыточное поле `text`**
✅ **Экономия 38% трафика на каждом chunk**
✅ **Упрощён код обработки**
✅ **Улучшен парсинг JSON (работает с форматированным)**
✅ **Все тесты пройдены**

## Файлы изменены

1. `src/rob_box_voice/prompts/master_prompt.txt`
   - Убрано требование поля `text`
   - Обновлены примеры
   - Добавлены примеры правильного/неправильного формата

2. `src/rob_box_voice/scripts/robbox_chat_streaming.py`
   - Метод `_speak_chunk()` - убран fallback на `text`
   - Улучшен парсинг JSON (подсчёт фигурных скобок)
   - Добавлен DEBUG режим для отладки

3. `src/rob_box_voice/scripts/test_streaming_ssml_only.py`
   - Новый тестовый скрипт для проверки

## Совместимость

⚠️ **Breaking change:** Старые ответы с полем `text` больше не будут использовать это поле.

Однако код **обратно совместим** - если придёт JSON с `text`, он просто будет проигнорирован, а SSML будет обработан как обычно.

## Следующие шаги

1. ✅ Протестировать на реальных запросах
2. ⏳ Убедиться что DeepSeek всегда возвращает компактный JSON в одну строку
3. ⏳ Мониторить качество ответов (все ударения на месте?)
4. ⏳ Commit и push изменений в ветку feature/voice-assistant
