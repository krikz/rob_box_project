# 🚀 ROBBOX TTS - Быстрый старт

Полный цикл создания кастомного голоса ROBBOX от генерации фраз до обучения модели.

## 📋 Чек-лист

- [ ] Dell Precision 5540 с NVIDIA GPU
- [ ] Ubuntu 22.04+ на ноутбуке
- [ ] DeepSeek API ключ (https://platform.deepseek.com/)
- [ ] Yandex Cloud API ключ (https://cloud.yandex.ru/)
- [ ] 20-30 минут свободного времени
- [ ] 1-3 дня для обучения модели

## ⚡ Быстрый запуск (30 минут)

### 1️⃣ Генерация фраз через DeepSeek (5 минут)

```bash
# На вашем ноутбуке
cd /path/to/rob_box_project/src/rob_box_voice/scripts

# Экспортируйте API ключ
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxx"

# Генерируем 200 вопросов и ответов
python3 generate_phrases_deepseek.py \
  --master-prompt ../prompts/master_prompt.txt \
  --output ../dataset/robbox_sentences_deepseek.txt \
  --num-questions 200 \
  --save-qa ../dataset/qa_pairs.json

# ✅ Результат: ~300-400 уникальных фраз
# 💰 Стоимость: $0.03-0.05
```

### 2️⃣ Запись голоса через Yandex TTS (15 минут)

```bash
# Экспортируйте Yandex API ключи
export YANDEX_API_KEY="AQVNxxxxx"
export YANDEX_FOLDER_ID="b1gxxxxx"

# Записываем датасет
python3 record_yandex_voice.py \
  --input ../dataset/robbox_sentences_deepseek.txt \
  --output ~/robbox_tts_training/datasets/robbox_voice/ \
  --voice anton \
  --emotion neutral \
  --speed 1.0 \
  --delay 1.5

# ✅ Результат: ~300 WAV файлов + metadata.csv
# ⏱️  Время: ~8-15 минут (зависит от количества фраз)
# 💰 Стоимость: бесплатно (в пределах лимитов Yandex)
```

### 3️⃣ Проверка системы и установка окружения (5 минут)

```bash
cd ../training

# Проверяем GPU
python3 check_system.py

# Если GPU OK, устанавливаем окружение
chmod +x setup_training_env.sh
./setup_training_env.sh

# Выберите Piper (рекомендуется)
# ✅ Результат: Готовое окружение в ~/robbox_tts_training/
```

### 4️⃣ Обучение модели (1-3 дня)

```bash
# Активируем окружение
source ~/robbox_tts_training/activate.sh

# Запускаем обучение
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_piper

# ✅ Автоматически:
#    - Подготовка датасета
#    - Создание конфигурации
#    - Обучение с чекпоинтами
#    - TensorBoard логи

# ⏱️  Время обучения:
#    - 4-6 GB VRAM: 2-4 дня
#    - 8-12 GB VRAM: 1-2 дня
#    - 24+ GB VRAM: 12-24 часа
```

### 5️⃣ Интеграция в ROBBOX (5 минут)

```bash
# После завершения обучения
cp ~/robbox_tts_training/models/robbox_piper/checkpoints/best_model.pt \
   /path/to/rob_box_project/models/robbox_custom.pt

# Обновите конфигурацию
nano /path/to/rob_box_project/src/rob_box_voice/config/voice_assistant.yaml

# Измените:
# silero:
#   model_path: "/models/robbox_custom.pt"
#   speaker: "robbox"

# Тестируйте в GUI
cd /path/to/rob_box_project/src/rob_box_voice/scripts
python3 silero_tts_gui.py
```

## 🎯 Рекомендации по параметрам

### Для вашего Dell Precision 5540

| Параметр | Значение | Комментарий |
|----------|----------|-------------|
| **num_questions** | 200 | Оптимально для fine-tuning |
| **voice** | anton | Мужской голос, нейтральный |
| **emotion** | neutral | Для робота |
| **speed** | 1.0 | Нормальная скорость |
| **delay** | 1.5 | Чтобы не забанили API |
| **batch_size** | 8-16 | Зависит от VRAM (см. ниже) |

### Определение batch_size

```bash
# Запустите check_system.py чтобы узнать вашу GPU
python3 check_system.py

# Рекомендации:
# 4 GB VRAM → batch_size: 4
# 6 GB VRAM → batch_size: 8
# 8 GB VRAM → batch_size: 16  ⭐ рекомендуется
# 12+ GB VRAM → batch_size: 32
```

## 📊 Мониторинг обучения

### TensorBoard (в реальном времени)

```bash
# В отдельном терминале
source ~/robbox_tts_training/activate.sh
tensorboard --logdir ~/robbox_tts_training/models/robbox_piper/logs

# Откройте браузер: http://localhost:6006
```

### GPU мониторинг

```bash
# В отдельном терминале
watch -n 1 nvidia-smi
```

### Метрики качества

Смотрите в TensorBoard:
- **Train Loss**: должен плавно уменьшаться (стремится к 0)
- **Val Loss**: должен быть близок к Train Loss (если сильно выше → overfitting)
- **MOS (Mean Opinion Score)**: >3.5 хорошо, >4.0 отлично
- **RTF (Real-time Factor)**: <0.5 для Raspberry Pi 5

## 🛠️ Troubleshooting

### DeepSeek ошибки

```bash
# API key не найден
export DEEPSEEK_API_KEY="sk-xxx"

# Rate limit
python3 generate_phrases_deepseek.py --delay 2.0 ...

# Слишком мало фраз
python3 generate_phrases_deepseek.py --num-questions 300 ...
```

### Yandex ошибки

```bash
# API key не найден
export YANDEX_API_KEY="AQVNxxxxx"
export YANDEX_FOLDER_ID="b1gxxxxx"

# Превышен лимит
# Подождите 1 час или увеличьте --delay

# Битые аудио файлы
# Проверьте: aplay robbox_00000.wav
```

### GPU Out of Memory

```bash
# В config.json уменьшите batch_size
nano ~/robbox_tts_training/models/robbox_piper/config.json

# Измените:
# "batch_size": 8  # было 16

# Перезапустите обучение
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_piper \
  --resume-from ~/robbox_tts_training/models/robbox_piper/checkpoints/checkpoint_100.pt
```

### Плохое качество модели

1. **Увеличьте количество данных:**
   - Сгенерируйте ещё фразы (300-500)
   - Запишите новый датасет
   - Объедините: `cat dataset1.txt dataset2.txt > combined.txt`

2. **Увеличьте количество эпох:**
   - В config.json: `"num_epochs": 2000` (было 1000)

3. **Проверьте качество аудио:**
   - Прослушайте несколько файлов
   - Удалите битые/тихие записи
   - Перезапишите если нужно

## 💰 Стоимость

| Компонент | Стоимость | Комментарий |
|-----------|-----------|-------------|
| DeepSeek API (200 вопросов) | $0.03-0.05 | Очень дёшево! |
| Yandex TTS (300 фраз) | Бесплатно | В пределах лимитов |
| GPU обучение (локально) | Бесплатно | Ваш Dell Precision |
| **Итого** | **~$0.05** | 🎉 |

**Альтернатива (cloud GPU):**
- Google Colab Pro+: $50/month
- Vast.ai RTX 3090: $0.50-0.80/hour × 48h = $24-38

## 📚 Документация

- **Генерация фраз:** `scripts/README_DEEPSEEK_GENERATOR.md`
- **Запись голоса:** `scripts/record_yandex_voice.py --help`
- **Обучение:** `training/README.md`
- **Полный гайд:** `dataset/README_CUSTOM_VOICE.md`

## 🎉 Успех!

После завершения обучения у вас будет:
- ✅ Кастомная TTS модель с голосом ROBBOX
- ✅ Модель оптимизирована для русского языка
- ✅ Работает offline на Raspberry Pi 5
- ✅ Поддерживает SSML для выразительности
- ✅ RTF <0.5 для реального времени

**Сравните с готовыми голосами Silero:**
```bash
python3 scripts/silero_tts_gui.py
# Попробуйте aidar vs ваш robbox_custom
```

---

**Удачи! 🤖🎙️**

Если что-то пошло не так:
1. Запустите `python3 check_system.py`
2. Проверьте логи в `~/robbox_tts_training/models/*/logs/`
3. Посмотрите примеры в документации
4. Попробуйте с меньшим датасетом (50 фраз для теста)
