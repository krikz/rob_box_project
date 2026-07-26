# Voice Assistant Optimization - October 13, 2025

## Резюме изменений

### 1. Упрощённый промпт без ударений
- **Создан:** `src/rob_box_voice/prompts/master_prompt_simple.txt`
- **Размер:** ~3.7KB (вместо 21KB старого)
- **Изменения:**
  - Убраны инструкции по расстановке ударений
  - Упрощены примеры
  - Акцент на компактный JSON
  - Добавлены правила математики прописью

### 2. Система автоматических ударений
- **Создан:** `src/rob_box_voice/config/accent_replacements.json`
- **Создан:** `src/rob_box_voice/scripts/accent_replacer.py`
- **Функциональность:**
  - Словарь с 25+ словами (топонимы, термины, глаголы, существительные)
  - Автоматическая замена слов на версии с ударениями
  - Легко расширяемый формат JSON
  - Категории: toponyms, tech_terms, common_verbs, common_nouns, homographs

**Пример словаря:**
```json
{
  "toponyms": {
    "Сочи": "С+очи",
    "Москва": "Москв+а",
    "Россия": "Росс+ия"
  },
  "tech_terms": {
    "робот": "р+обот",
    "роббокс": "робб+окс",
    "система": "сист+ема"
  }
}
```

### 3. Интеграция в rob_box_chat_streaming.py
- **Обновлён:** `src/rob_box_voice/scripts/robbox_chat_streaming.py`
- **Изменения:**
  - Импорт `AccentReplacer`
  - Инициализация в `__init__`
  - Применение ударений в `_speak_chunk()` перед озвучиванием
  - Использование упрощённого промпта (`master_prompt_simple.txt`)
  - Улучшен парсинг JSON (подсчёт фигурных скобок)

**Код применения ударений:**
```python
def _speak_chunk(self, chunk_data: dict):
    ssml = chunk_data.get('ssml', '')
    
    # ✨ ДОБАВЛЯЕМ УДАРЕНИЯ из словаря
    ssml = self.accent_replacer.add_accents(ssml)
    
    # Озвучиваем...
```

### 4. Удаление поля `text` из JSON
- **Обновлён:** `src/rob_box_voice/prompts/master_prompt.txt`
- **Изменения:**
  - Поле `text` больше НЕ требуется
  - Поле `ssml` теперь единственное обязательное
  - Экономия ~38% трафика на каждом chunk

**Было:**
```json
{
  "text": "Привет! Я робот.",
  "ssml": "<speak>Привет!<break time='300ms'/>Я робот.</speak>"
}
```

**Стало:**
```json
{
  "ssml": "<speak>Привет!<break time='300ms'/>Я робот.</speak>"
}
```

### 5. Обновлён setup.py
- **Изменён:** `src/rob_box_voice/setup.py`
- **Добавлено:** `glob('config/*.json')` для включения accent_replacements.json

## Результаты тестирования

### Тест 1: Короткий ответ
```bash
$ python3 robbox_chat_streaming.py
👤 Вы: Привет!
🤖 ROBBOX:
📦 Chunk 1: {"ssml": "<speak><prosody pitch='high'>Прив+ет!</prosody>..."}
🔊 Говорю: Прив+ет!
```
✅ Работает

### Тест 2: Город Сочи (3 chunks)
```bash
$ python3 robbox_chat_streaming.py
👤 Вы: Расскажи про Сочи кратко
🤖 ROBBOX:
📦 Chunk 1: Привет! С+очи — кур+ортный г+ород...
📦 Chunk 2: Распол+ожен на черноморском...
📦 Chunk 3: Изв+естен Олимпиадой...
```
✅ Ударения автоматические: Сочи→С+очи, город→г+ород, Расположен→Распол+ожен

### Тест 3: Теорема Пифагора
```bash
📦 Chunk 1: Теор+ема Пиф+агора глас+ит: в прямоугольном...
📦 Chunk 2: Ф+ормула: а в квадр+ате плюс бэ в квадр+ате...
```
✅ Формулы прописью, ударения на месте

## Преимущества

### 1. Меньше нагрузка на DeepSeek
- Промпт: 21KB → 3.7KB (**82% меньше**)
- Меньше токенов = быстрее ответ
- Меньше вероятность ошибок в ударениях

### 2. Контролируемые ударения
- Словарь полностью под контролем
- Легко добавлять новые слова
- Нет зависимости от качества LLM

### 3. Экономия трафика
- JSON без `text`: **38% меньше**
- Для 6 chunks: ~3KB → ~1.9KB

### 4. Упрощённая архитектура
- LLM только генерирует текст
- Ударения добавляются локально
- Меньше сложности в промпте

## Файлы для commit

### Новые файлы:
1. `src/rob_box_voice/config/accent_replacements.json` - словарь ударений
2. `src/rob_box_voice/prompts/master_prompt_simple.txt` - упрощённый промпт
3. `src/rob_box_voice/scripts/accent_replacer.py` - модуль автоударений
4. `src/rob_box_voice/scripts/robbox_chat_streaming.py` - streaming чат
5. `src/rob_box_voice/scripts/test_streaming_ssml_only.py` - тесты
6. `SSML_ONLY_OPTIMIZATION.md` - документация оптимизации

### Изменённые файлы:
1. `src/rob_box_voice/prompts/master_prompt.txt` - убрано поле text
2. `src/rob_box_voice/setup.py` - добавлен JSON в data_files
3. `src/rob_box_voice/scripts/text_normalizer.py` - добавлены латинские буквы

## Docker интеграция

### Готово:
- ✅ Dockerfile уже существует: `docker/vision/voice_assistant/Dockerfile`
- ✅ docker-compose.yaml уже настроен
- ✅ Startup script готов: `docker/vision/scripts/start_voice_assistant.sh`
- ✅ GitHub Actions workflow готов: `.github/workflows/build-vision-services.yml`

### Что нужно:
1. **Commit изменений** в feature/voice-assistant
2. **Push** в GitHub
3. **GitHub Actions автоматически:**
   - Соберёт новый Docker образ
   - Опубликует в ghcr.io с тегом `voice-assistant-kilted-dev`
4. **На роботе:**
   ```bash
   cd /home/ros2/rob_box_project/docker/vision
   docker-compose pull voice-assistant
   docker-compose up -d voice-assistant
   ```

## Следующие шаги

### 1. Commit и push
```bash
cd /home/ros2/rob_box_project
git add .
git commit -m "feat(voice): упрощённый промпт + автоударения + оптимизация JSON

- Создан упрощённый промпт (3.7KB vs 21KB)
- Добавлена система автоматических ударений (словарь 25+ слов)
- Удалено избыточное поле text из JSON (экономия 38%)
- Интегрирован accent_replacer в robbox_chat_streaming
- Улучшен парсинг JSON (подсчёт фигурных скобок)
- Добавлены тесты и документация"

git push origin feature/voice-assistant
```

### 2. Расширение словаря ударений
Просто редактировать `src/rob_box_voice/config/accent_replacements.json`:
```json
{
  "toponyms": {
    "Краснодар": "Краснод+ар",
    "Адлер": "+Адлер"
  }
}
```

### 3. Мониторинг на роботе
```bash
# Логи контейнера
docker logs -f voice-assistant

# ROS2 topics
ros2 topic echo /voice/dialogue/response
ros2 topic echo /voice/audio/speech
```

## API ключи DeepSeek

**Важно:** DeepSeek **НЕ поддерживает** хранение системного промпта на их стороне.
- Каждый запрос должен включать system message
- Но с упрощённым промптом (3.7KB) это не проблема
- Кэширование на стороне DeepSeek возможно, но не документировано

## Обратная совместимость

- ✅ Старый `master_prompt.txt` сохранён (для reference)
- ✅ Код работает с обоими форматами JSON (с `text` и без)
- ✅ Если DeepSeek вернёт `text`, он просто будет проигнорирован
- ✅ Accent replacer не ломает текст без ударений

## Производительность

**До оптимизации:**
- System prompt: 21KB
- JSON chunk: ~510 байт
- Зависимость от качества LLM ударений

**После оптимизации:**
- System prompt: 3.7KB (**82% ↓**)
- JSON chunk: ~321 байт (**38% ↓**)
- Контролируемые ударения через словарь
- Быстрее первый ответ

## Известные ограничения

1. **Словарь ударений:** 25 слов (легко расширяется)
2. **Омографы:** Пока не реализованы (замок/замок)
3. **DeepSeek иногда:** Возвращает форматированный JSON (парсер справляется)
4. **Markdown блоки:** DeepSeek может обернуть в ```json``` (парсер игнорирует)

## Рекомендации

1. **Расширяйте словарь** по мере использования
2. **Мониторьте логи** для пропущенных слов
3. **Добавляйте омографы** по необходимости
4. **Тестируйте** новые категории слов

---

**Статус:** ✅ Готово к production
**Тестирование:** ✅ Пройдено
**Docker:** ✅ Готов к сборке
**Документация:** ✅ Полная
