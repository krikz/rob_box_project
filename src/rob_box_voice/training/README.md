# ROBBOX TTS Training

Обучение кастомной TTS модели для ROBBOX на Dell Precision 5540.

## Быстрый старт

### 1. Проверка системы

```bash
cd /path/to/rob_box_project/src/rob_box_voice/training
python3 check_system.py
```

Проверит:
- ✅ NVIDIA GPU и драйверы
- ✅ CUDA и PyTorch
- ✅ Доступную память GPU
- ✅ Установленные библиотеки

### 2. Установка окружения

```bash
chmod +x setup_training_env.sh
./setup_training_env.sh
```

Установит:
- NVIDIA драйверы (если нужно)
- PyTorch с CUDA 11.8
- Piper и/или Coqui TTS
- Все зависимости

### 3. Активация окружения

```bash
source ~/robbox_tts_training/activate.sh
```

### 4. Сбор данных

```bash
cd ../scripts

# Запись голоса через Yandex API
python3 record_yandex_voice.py \
  --input ../dataset/expanded_sentences.txt \
  --output ~/robbox_tts_training/datasets/robbox_voice/ \
  --voice anton \
  --emotion neutral \
  --speed 1.0 \
  --delay 1.5
```

**Требуется:**
- Yandex API key и Folder ID
- 200-300 предложений для fine-tuning (20-30 минут аудио)
- 3000-5000 предложений для полного обучения (3-5 часов аудио)

### 5. Обучение модели

#### Вариант A: Piper (рекомендуется) ⭐

```bash
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_piper
```

**Преимущества:**
- Быстрее обучается
- Меньше памяти
- Лучше для русского языка
- Production-ready

#### Вариант B: Coqui TTS

```bash
python3 train_coqui.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_coqui \
  --batch-size 16
```

**Преимущества:**
- Больше настроек
- Больше архитектур
- Активное коммьюнити

## Структура проекта

```
~/robbox_tts_training/
├── datasets/
│   └── robbox_voice/          # Записанные аудио + metadata.csv
│       ├── robbox_00000.wav
│       ├── robbox_00001.wav
│       ├── ...
│       └── metadata.csv
│
├── models/
│   ├── robbox_piper/          # Piper модель
│   │   ├── config.json
│   │   ├── checkpoints/
│   │   └── logs/
│   │
│   └── robbox_coqui/          # Coqui модель
│       ├── config.json
│       └── output/
│
├── checkpoints/               # Промежуточные чекпоинты
├── logs/                      # TensorBoard логи
└── activate.sh                # Скрипт активации окружения
```

## Требования к системе

### Минимальные (voice cloning)

- GPU: 6 GB VRAM (GTX 1660, RTX 3050)
- RAM: 16 GB
- Аудио: 5-10 минут
- Время: Несколько часов
- Качество: 3-4/5

### Рекомендуемые (fine-tuning) ⭐

- **GPU: 12+ GB VRAM (RTX 3060, RTX 4060)**
- **RAM: 32 GB**
- **Аудио: 20-30 минут**
- **Время: 1-3 дня**
- **Качество: 4/5**

### Оптимальные (full training)

- GPU: 24 GB VRAM (RTX 3090, RTX 4090)
- RAM: 64 GB
- Аудио: 3-5 часов
- Время: 5-14 дней
- Качество: 4-5/5

## Dell Precision 5540 - ваша конфигурация

```
CPU: Intel Core i7-9750H (6 cores, 12 threads)
RAM: 32 GB
GPU: NVIDIA Quadro T2000 (4 GB VRAM) или GTX/RTX?
```

**Проверьте вашу GPU:**
```bash
nvidia-smi
```

### Если у вас Quadro T2000 (4 GB):
- ✅ Voice cloning: да
- ⚠️  Fine-tuning: сложно (нужно batch_size=4-8)
- ❌ Full training: нет

### Если у вас GTX 1650/1660 (6 GB):
- ✅ Voice cloning: да
- ✅ Fine-tuning: да (batch_size=8)
- ⚠️  Full training: очень медленно

### Если у вас RTX 3050/3060 (8-12 GB):
- ✅ Voice cloning: отлично
- ✅ Fine-tuning: отлично (batch_size=16)
- ✅ Full training: медленно но можно

## Оптимизация для вашей GPU

### Если памяти мало (4-6 GB):

```bash
# Уменьшите batch size
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_piper

# Отредактируйте config.json:
# "batch_size": 4  # вместо 16
```

### Если памяти достаточно (8+ GB):

```bash
# Используйте стандартные настройки
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_piper

# batch_size: 16 (по умолчанию)
```

## Мониторинг обучения

### TensorBoard

```bash
# Запустите TensorBoard
tensorboard --logdir ~/robbox_tts_training/models/robbox_piper/logs

# Откройте в браузере
# http://localhost:6006
```

### nvidia-smi watch

```bash
# Мониторинг GPU в реальном времени
watch -n 1 nvidia-smi
```

### Метрики качества

Во время обучения смотрите на:
- **Loss (train/val):** должен уменьшаться
- **MOS (Mean Opinion Score):** должен расти (>3.5 хорошо, >4.0 отлично)
- **RTF (Real-time Factor):** <0.5 для Pi 5

## Прерывание и продолжение

### Сохранение прогресса

Обучение автоматически сохраняет чекпоинты каждые 100 эпох:
```
checkpoints/
├── checkpoint_100.pt
├── checkpoint_200.pt
├── ...
└── checkpoint_best.pt
```

### Продолжение обучения

```bash
# Piper
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_piper \
  --resume-from ~/robbox_tts_training/models/robbox_piper/checkpoints/checkpoint_500.pt

# Coqui
python3 train_coqui.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice \
  --output ~/robbox_tts_training/models/robbox_coqui \
  --resume-from ~/robbox_tts_training/models/robbox_coqui/output/checkpoint_500.pth
```

## Тестирование модели

### Во время обучения

```bash
# Piper
python3 -c "
import torch

model = torch.jit.load('checkpoints/checkpoint_500.pt')
audio = model('Привет, я ROBBOX!')

import soundfile as sf
sf.write('test.wav', audio.numpy(), 22050)
"

# Прослушать
aplay test.wav
```

### После обучения

```bash
# Используйте GUI для тестирования
cd ../scripts
python3 silero_tts_gui.py
```

## Интеграция в ROBBOX

После успешного обучения:

```bash
# 1. Копируем модель
cp ~/robbox_tts_training/models/robbox_piper/checkpoints/best_model.pt \
   /path/to/rob_box_project/models/robbox_custom.pt

# 2. Обновляем конфигурацию
nano /path/to/rob_box_project/src/rob_box_voice/config/tts_node.yaml

# Измените:
# silero:
#   model_path: "/models/robbox_custom.pt"
#   speaker: "robbox"

# 3. Пересоберите Docker
cd /path/to/rob_box_project
docker build -f docker/vision/voice_assistant/Dockerfile -t robbox-voice .

# 4. Тестируйте
docker run -it robbox-voice python3 test_voice.py
```

## Troubleshooting

### Out of memory (OOM)

```bash
# Уменьшите batch_size в config.json
"batch_size": 4  # было 16

# Уменьшите model size
"hidden_channels": 128  # было 192
```

### Плохое качество звука

- Увеличьте количество данных (>30 минут)
- Проверьте качество исходного аудио
- Увеличьте количество эпох (>500)
- Попробуйте другой voice (anton → ermil)

### Медленное обучение

- Проверьте что используется GPU: `nvidia-smi`
- Увеличьте batch_size (если есть память)
- Уменьшите num_workers (если CPU слабый)

### NaN loss

- Уменьшите learning_rate (0.0002 → 0.0001)
- Проверьте данные (нет ли битых аудио)
- Начните с меньшей модели

## Альтернатива: Cloud GPU

Если ваша GPU слишком слабая:

### Google Colab Pro+
```bash
# $50/month
# RTX A100 (40 GB) или V100 (16 GB)
# 100 compute units

# Загрузите dataset на Google Drive
# Используйте наш training скрипт в Colab
```

### Vast.ai
```bash
# $0.50-1.20/hour
# RTX 3090 (24 GB) или RTX 4090 (24 GB)

# Создайте instance
# Загрузите dataset
# Запустите training
```

## Рекомендации

### Для вашего Dell Precision 5540:

1. **Сначала проверьте GPU:**
   ```bash
   python3 check_system.py
   ```

2. **Если 4-6 GB VRAM:**
   - ✅ Voice cloning (5-10 мин данных)
   - ❌ Fine-tuning/Full training → используйте cloud

3. **Если 8-12 GB VRAM:**
   - ✅ Fine-tuning (20-30 мин данных) - рекомендуется!
   - ⚠️  Full training (медленно, но можно)

4. **Начните с fine-tuning:**
   - Запишите 200-300 фраз (20-30 минут)
   - Обучите на вашей GPU (1-3 дня)
   - Оцените качество
   - Если нужно лучше - full training на cloud

## Полезные ссылки

- [Piper GitHub](https://github.com/rhasspy/piper)
- [Coqui TTS GitHub](https://github.com/coqui-ai/TTS)
- [VITS Paper](https://arxiv.org/abs/2106.06103)
- [Vast.ai GPU Rental](https://vast.ai/)
- [Google Colab](https://colab.research.google.com/)

## Поддержка

Если возникли проблемы:
1. Запустите `python3 check_system.py`
2. Проверьте логи в `~/robbox_tts_training/models/*/logs/`
3. Создайте issue с выводом `nvidia-smi` и логами

Удачи с обучением! 🤖🎙️
