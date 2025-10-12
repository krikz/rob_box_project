# 🎯 Как запустить генерацию датасета (для вас)

## ✅ Ваши ключи уже настроены

Файл с ключами: `src/rob_box_voice/.env.secrets`

```bash
# Yandex Cloud API
export YANDEX_API_KEY="AQVN..."  # ваш полный ключ уже в файле!
export YANDEX_FOLDER_ID="ajeq..."

# DeepSeek API
export DEEPSEEK_API_KEY="sk-..."  # ваш полный ключ уже в файле!
```

**🔒 Этот файл защищён .gitignore и НЕ будет закоммичен!**
**📝 Полные ключи уже записаны в локальном файле!**

---

## 🚀 Быстрый запуск

### Весь процесс одной командой:

```bash
cd ~/rob_box_project/src/rob_box_voice/scripts
./generate_dataset.sh 200
```

**Что произойдёт:**
1. ✅ Загрузит ваши API ключи из `.env.secrets`
2. ✅ Сгенерирует 200 вопросов через DeepSeek (~5 мин, $0.05)
3. ✅ Получит ответы робота с вашим master_prompt.txt
4. ✅ Запишет ~300-400 фраз через Yandex TTS (~15 мин, бесплатно)
5. ✅ Покажет команды для обучения на Dell Precision

---

## 📋 Пошагово (если нужен контроль)

### Шаг 1: Генерация фраз

```bash
cd ~/rob_box_project/src/rob_box_voice/scripts

# Загрузите ключи
source ../.env.secrets

# Генерируйте фразы
python3 generate_phrases_deepseek.py \
  --master-prompt ../prompts/master_prompt.txt \
  --output ../dataset/my_sentences.txt \
  --num-questions 200 \
  --save-qa ../dataset/qa_pairs.json

# ✅ Результат: ../dataset/my_sentences.txt (~300-400 фраз)
# 💰 Стоимость: $0.03-0.05
```

### Шаг 2: Запись голоса

```bash
# Ключи уже загружены выше
python3 record_yandex_voice.py \
  --input ../dataset/my_sentences.txt \
  --output ~/robbox_tts_training/datasets/my_voice/ \
  --voice anton \
  --emotion neutral \
  --speed 1.0 \
  --delay 1.5

# ✅ Результат: ~/robbox_tts_training/datasets/my_voice/*.wav + metadata.csv
# ⏱️  Время: ~15 минут
```

### Шаг 3: Обучение (на Dell Precision)

```bash
# Проверьте GPU
cd ~/rob_box_project/src/rob_box_voice/training
python3 check_system.py

# Установите окружение (первый раз)
./setup_training_env.sh

# Активируйте
source ~/robbox_tts_training/activate.sh

# Обучайте
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/my_voice \
  --output ~/robbox_tts_training/models/my_model

# ⏱️  Время: 1-3 дня (зависит от GPU)
```

---

## 💡 Варианты объёма

| Вопросов | Фраз | Аудио | Подход | Время обучения | DeepSeek |
|----------|------|-------|--------|----------------|----------|
| 50 | ~100 | 5-10 мин | Voice cloning | Часы | $0.01 |
| **200** | **~350** | **20-30 мин** | **Fine-tuning** ⭐ | **1-3 дня** | **$0.05** |
| 500 | ~800 | 40-60 мин | Full training | 5-14 дней | $0.12 |
| 1000 | ~1400 | 70-90 мин | Full training | 5-14 дней | $0.25 |

**Рекомендую: 200 вопросов** для начала!

---

## 🔧 Параметры

### DeepSeek генерация

```bash
--num-questions 200    # Количество вопросов (100-1000)
--delay 1.0            # Задержка между запросами (сек)
--save-qa file.json    # Сохранить Q&A пары (опционально)
```

### Yandex запись

```bash
--voice anton          # anton (мужской), alena (женский), ermil (нейтральный)
--emotion neutral      # neutral, good, evil
--speed 1.0            # 0.1-3.0 (рекомендую 0.8-1.2)
--delay 1.5            # Задержка между запросами (чтобы не забанили)
```

### Piper обучение

```bash
--dataset /path        # Путь к датасету с metadata.csv
--output /path         # Путь для сохранения модели
--resume-from *.pt     # Продолжить с чекпоинта (если прервалось)
```

---

## 📊 Мониторинг обучения

### TensorBoard (графики в реальном времени)

```bash
# В отдельном терминале
source ~/robbox_tts_training/activate.sh
tensorboard --logdir ~/robbox_tts_training/models/my_model/logs

# Открыть: http://localhost:6006
```

### GPU утилизация

```bash
# В отдельном терминале
watch -n 1 nvidia-smi
```

---

## ❓ Частые вопросы

### Что делать если прервалось обучение?

```bash
# Найдите последний чекпоинт
ls ~/robbox_tts_training/models/my_model/checkpoints/

# Продолжите с него
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/my_voice \
  --output ~/robbox_tts_training/models/my_model \
  --resume-from ~/robbox_tts_training/models/my_model/checkpoints/checkpoint_500.pt
```

### Out of Memory на GPU?

```bash
# Уменьшите batch_size в config.json
nano ~/robbox_tts_training/models/my_model/config.json

# Измените:
# "batch_size": 8  # было 16

# Перезапустите обучение с --resume-from
```

### Хочу больше фраз?

```bash
# Сгенерируйте ещё один батч
./generate_dataset.sh 200

# Или объедините датасеты
cat dataset1.txt dataset2.txt | sort -u > combined.txt
python3 record_yandex_voice.py --input combined.txt ...
```

---

## 🎉 После обучения

```bash
# Скопируйте лучшую модель
cp ~/robbox_tts_training/models/my_model/checkpoints/best_model.pt \
   ~/rob_box_project/models/robbox_custom.pt

# Обновите конфиг
nano ~/rob_box_project/src/rob_box_voice/config/voice_assistant.yaml

# Измените:
# silero:
#   model_path: "/models/robbox_custom.pt"
#   speaker: "robbox"

# Тестируйте
cd ~/rob_box_project/src/rob_box_voice/scripts
python3 silero_tts_gui.py
```

---

## 📚 Полная документация

- Быстрый старт: `QUICKSTART_TTS_TRAINING.md`
- Безопасность: `SECRETS_GUIDE.md`
- DeepSeek генерация: `scripts/README_DEEPSEEK_GENERATOR.md`
- Обучение: `training/README.md`
- Запись голоса: `dataset/README_CUSTOM_VOICE.md`

---

**Удачи! 🤖🎙️**

Если нужна помощь - пишите! 😊
