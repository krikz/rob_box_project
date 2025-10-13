# Генерация фраз для TTS через DeepSeek

Автоматическая генерация датасета для обучения голоса ROBBOX через DeepSeek API.

## Как это работает

1. **Генерация вопросов**: DeepSeek создаёт список разнообразных вопросов и команд для робота
2. **Получение ответов**: Каждый вопрос отправляется DeepSeek с master prompt ROBBOX
3. **Обработка**: Ответы очищаются от команд (`<CMD:.../>`) и разбиваются на предложения
4. **Дедупликация**: Удаляются повторяющиеся фразы
5. **Сохранение**: Финальный список предложений сохраняется для записи голоса

## Установка

```bash
# Установите OpenAI SDK (DeepSeek совместим с OpenAI API)
pip install openai
```

## Получение API ключа

1. Зарегистрируйтесь на https://platform.deepseek.com/
2. Перейдите в https://platform.deepseek.com/api_keys
3. Создайте новый API ключ
4. Скопируйте ключ (начинается с `sk-`)

**Стоимость:**
- DeepSeek-Chat: $0.14 / 1M входных токенов, $0.28 / 1M выходных
- 200 вопросов ≈ 100K токенов ≈ **$0.03-0.05** 💰
- 1000 вопросов ≈ 500K токенов ≈ **$0.15-0.25** 💰

Очень дёшево! 🎉

## Использование

### Базовый пример (200 вопросов для fine-tuning)

```bash
# Экспортируйте API ключ
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxx"

# Сгенерируйте датасет
python3 generate_phrases_deepseek.py \
  --master-prompt ../prompts/master_prompt.txt \
  --output ../dataset/robbox_sentences_deepseek.txt \
  --num-questions 200
```

### Расширенный пример (1000 вопросов для full training)

```bash
python3 generate_phrases_deepseek.py \
  --api-key sk-xxxxxxxxxxxxxxxxxx \
  --master-prompt ../prompts/master_prompt.txt \
  --output ../dataset/full_dataset_deepseek.txt \
  --num-questions 1000 \
  --delay 1.5 \
  --save-qa ../dataset/qa_pairs.json
```

### Параметры

| Параметр | Описание | По умолчанию |
|----------|----------|--------------|
| `--api-key` | DeepSeek API ключ | `$DEEPSEEK_API_KEY` |
| `--master-prompt` | Путь к master_prompt.txt | `../prompts/master_prompt.txt` |
| `--output` | Выходной файл с предложениями | Обязательный |
| `--num-questions` | Количество вопросов | 200 |
| `--delay` | Задержка между запросами (сек) | 1.0 |
| `--save-qa` | Сохранить Q&A пары в JSON | Опциональный |

## Примеры генерации

### Что генерируется

**Вопросы (примеры):**
```
Привет Роббокс
Поезжай вперёд
Какой заряд батареи
Остановись немедленно
Следуй за мной
Что ты видишь вокруг
Сканируй комнату
Вернись на базу
```

**Ответы робота (примеры с master prompt):**
```
Вопрос: "Привет Роббокс"
Ответ: "Привет! Я Роббокс, чем могу помочь?"

Вопрос: "Поезжай вперёд"
Ответ: "Еду вперёд. <CMD:move_forward speed=0.5/>"

Вопрос: "Какой заряд батареи"
Ответ: "Заряд батареи семьдесят восемь процентов."
```

**Финальный датасет (после обработки):**
```
Привет!
Я Роббокс, чем могу помочь?
Еду вперёд.
Заряд батареи семьдесят восемь процентов.
Останавливаюсь.
Поворачиваю налево на девяносто градусов.
...
```

Команды `<CMD:.../>` автоматически удаляются!

## Категории вопросов

Скрипт генерирует вопросы по категориям:

1. **Приветствия и диалог** (15%)
   - "Привет", "Как дела", "Спасибо", "До свидания"

2. **Команды движения** (20%)
   - "Поезжай вперёд", "Остановись", "Повернись налево"

3. **Запросы информации** (15%)
   - "Какой заряд батареи", "Где ты находишься", "Что видишь"

4. **Навигация** (15%)
   - "Поезжай к окну", "Вернись на базу", "Следуй за мной"

5. **Сенсоры** (10%)
   - "Покажи карту", "Сканируй комнату", "Есть ли препятствия"

6. **Эмоции и характер** (10%)
   - "Расскажи о себе", "Что ты умеешь", "Ты устал?"

7. **Безопасность** (10%)
   - "Всё в порядке?", "Можешь проехать?", "Остановись немедленно"

8. **Системные запросы** (5%)
   - "Какая температура", "Проверь систему", "Включи свет"

## Рекомендации по объёму

| Количество вопросов | Уникальных предложений | Аудио (примерно) | Подход | Стоимость DeepSeek |
|---------------------|------------------------|------------------|--------|---------------------|
| 50 | ~100-150 | 5-10 мин | Voice cloning | $0.01 |
| 200 | ~300-400 | 15-25 мин | Fine-tuning ⭐ | $0.03-0.05 |
| 500 | ~700-900 | 35-45 мин | Full training | $0.08-0.12 |
| 1000 | ~1200-1500 | 60-75 мин | Full training | $0.15-0.25 |

**Рекомендация: 200 вопросов** - оптимальный баланс качества и стоимости.

## Workflow

### Полный цикл создания кастомного голоса

```bash
# 1. Генерация фраз через DeepSeek
export DEEPSEEK_API_KEY="sk-xxx"
python3 generate_phrases_deepseek.py \
  --output ../dataset/robbox_sentences_deepseek.txt \
  --num-questions 200 \
  --save-qa ../dataset/qa_pairs.json

# 2. Проверка сгенерированных фраз
head -20 ../dataset/robbox_sentences_deepseek.txt
wc -l ../dataset/robbox_sentences_deepseek.txt

# 3. Запись голоса через Yandex TTS
python3 record_yandex_voice.py \
  --input ../dataset/robbox_sentences_deepseek.txt \
  --output ~/robbox_tts_training/datasets/robbox_voice_deepseek/ \
  --voice anton \
  --emotion neutral \
  --speed 1.0

# 4. Проверка датасета
ls -lh ~/robbox_tts_training/datasets/robbox_voice_deepseek/
python3 -c "import pandas as pd; df = pd.read_csv('~/robbox_tts_training/datasets/robbox_voice_deepseek/metadata.csv', sep='|'); print(f'Duration: {df.duration.sum()/60:.1f} min')"

# 5. Обучение модели
cd ../training
source ~/robbox_tts_training/activate.sh
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice_deepseek \
  --output ~/robbox_tts_training/models/robbox_piper_deepseek
```

## Сравнение с ручным корпусом

### Ручной корпус (robbox_sentences_example.txt)

**Преимущества:**
- ✅ Полный контроль над содержимым
- ✅ Гарантированная релевантность
- ✅ Бесплатно

**Недостатки:**
- ❌ Долго создавать (часы работы)
- ❌ Может быть однообразным
- ❌ Сложно придумать 200-300 уникальных фраз

### DeepSeek генерация (generate_phrases_deepseek.py)

**Преимущества:**
- ✅ Быстро (200 фраз за ~5 минут)
- ✅ Высокое разнообразие
- ✅ Естественный язык
- ✅ Соответствует master prompt
- ✅ Дёшево ($0.03-0.05)

**Недостатки:**
- ⚠️  Требует проверки (могут быть странные фразы)
- ⚠️  Нужен API ключ
- ⚠️  Зависимость от внешнего сервиса

**Рекомендация:** Комбинируйте оба подхода!
```bash
# Объедините ручной корпус + DeepSeek генерацию
cat robbox_sentences_example.txt robbox_sentences_deepseek.txt > combined_dataset.txt

# Удалите дубликаты
sort -u combined_dataset.txt > final_dataset.txt
```

## Проверка качества

После генерации проверьте:

```bash
# 1. Количество предложений
wc -l robbox_sentences_deepseek.txt

# 2. Средняя длина
awk '{print length}' robbox_sentences_deepseek.txt | awk '{sum+=$1; count++} END {print "Average:", sum/count, "chars"}'

# 3. Самые частые слова
cat robbox_sentences_deepseek.txt | tr ' ' '\n' | sort | uniq -c | sort -rn | head -20

# 4. Проверка на команды (должно быть пусто)
grep -E '<CMD:' robbox_sentences_deepseek.txt

# 5. Примеры предложений
shuf -n 20 robbox_sentences_deepseek.txt
```

## Ручная доработка

Если нужно отредактировать:

```bash
# Откройте в редакторе
nano robbox_sentences_deepseek.txt

# Удалите ненужные строки
# Добавьте свои фразы
# Исправьте ошибки
```

**Что искать:**
- ❌ Слишком длинные предложения (>100 символов)
- ❌ Технические термины без объяснения
- ❌ Повторяющиеся фразы
- ❌ Неестественные формулировки
- ✅ Разнообразие по длине и стилю

## Troubleshooting

### Ошибка "API key not provided"

```bash
# Проверьте переменную окружения
echo $DEEPSEEK_API_KEY

# Или укажите явно
python3 generate_phrases_deepseek.py --api-key sk-xxx ...
```

### Ошибка "Rate limit exceeded"

```bash
# Увеличьте задержку между запросами
python3 generate_phrases_deepseek.py --delay 2.0 ...
```

### Слишком мало уникальных предложений

```bash
# Увеличьте количество вопросов
python3 generate_phrases_deepseek.py --num-questions 300 ...

# Или запустите несколько раз и объедините
python3 generate_phrases_deepseek.py --output batch1.txt --num-questions 100
python3 generate_phrases_deepseek.py --output batch2.txt --num-questions 100
cat batch1.txt batch2.txt | sort -u > combined.txt
```

### Master prompt не найден

```bash
# Проверьте путь
ls -l ../prompts/master_prompt.txt

# Или укажите явно
python3 generate_phrases_deepseek.py \
  --master-prompt /path/to/rob_box_project/src/rob_box_voice/prompts/master_prompt.txt \
  ...
```

## Альтернативы

Если нет DeepSeek API:

1. **ChatGPT** (более дорого, ~$0.50 за 200 вопросов)
   ```bash
   # Используйте OpenAI API вместо DeepSeek
   # Измените base_url на https://api.openai.com
   ```

2. **Локальная LLM** (бесплатно, но медленнее)
   ```bash
   # Используйте Ollama + DeepSeek-R1:7B
   ollama pull deepseek-r1:7b
   # Адаптируйте скрипт под Ollama API
   ```

3. **Ручная генерация** (см. robbox_sentences_example.txt)

## Статистика

Типичные результаты для 200 вопросов:

```
Questions generated: 200
Responses generated: 198 (2 failed due to API errors)
Sentences extracted: 456
Unique sentences: 387
Average sentence length: 42 chars
Estimated audio duration: 19.4 minutes
Recommended approach: Fine-tuning ⭐
DeepSeek cost: $0.04
Total time: ~4 minutes
```

## Поддержка

Если возникли проблемы:
1. Проверьте API ключ: https://platform.deepseek.com/usage
2. Проверьте баланс: https://platform.deepseek.com/billing
3. Посмотрите логи ошибок в терминале
4. Попробуйте с меньшим количеством вопросов (--num-questions 50)

Удачи с генерацией! 🤖🎙️
