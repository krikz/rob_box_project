<!-- stale-paths: ignore-file (описывает удалённое или старую раскладку; кандидат в архив или переписывание, ADR-0163) -->
# Создание кастомного голоса ROBBOX

> **09.11.2026:** скрипты `generate_dataset.sh`, `generate_phrases_deepseek.py`, `silero_tts_gui.py` и чат `robbox_chat*.py` удалены (решение Шифу 09.11, ADR-0145 §4): их никто не вызывал (чат и генератор фраз к тому же читали давно удалённые промпты). Шаги с ними ниже не выполнимы; запись (`record_yandex_voice.py`) и обучение остались.

Инструкция по записи и обучению собственной TTS модели на базе голоса ROBBOX из Yandex SpeechKit.

## Обзор

Мы можем создать кастомный голос ROBBOX тремя способами:

| Метод | Аудио | Время | Сложность | Качество | Стоимость |
|-------|-------|-------|-----------|----------|-----------|
| **Voice Cloning** | 5-10 мин | Часы | Низкая | 3-4/5 | $50-100 |
| **Fine-tuning** | 20-30 мин | 1-3 дня | Средняя | 4/5 | $60-400 |
| **Полное обучение** | 3-5 часов | 5-14 дней | Высокая | 4-5/5 | $400-2000 |

**Рекомендация:** Начать с **Fine-tuning** (оптимальный баланс качества/усилий).

## Шаг 1: Сбор аудио данных

### 1.1 Подготовка текстового корпуса

Создайте файл с предложениями (одна фраза на строку):

```bash
# Пример корпуса уже есть
cat dataset/robbox_sentences_example.txt

# Или создайте свой
nano dataset/my_sentences.txt
```

**Рекомендации по объёму:**

- **Voice cloning:** 50-100 фраз (5-10 минут аудио)
- **Fine-tuning:** 200-300 фраз (20-30 минут) ⭐ рекомендуется
- **Полное обучение:** 3000-5000 фраз (3-5 часов)

**Источники текста:**

1. **DeepSeek автоматическая генерация** ⭐ РЕКОМЕНДУЕТСЯ:
   ```bash
   # Автоматически генерирует 200-300 фраз за 5 минут через DeepSeek API
   cd src/rob_box_voice/scripts
   export DEEPSEEK_API_KEY="sk-xxx"
   python3 generate_phrases_deepseek.py \
     --output ../dataset/robbox_sentences_deepseek.txt \
     --num-questions 200
   
   # Стоимость: $0.03-0.05 за 200 вопросов!
   # См. полную документацию: scripts/README_DEEPSEEK_GENERATOR.md
   ```

2. **Системные фразы ROBBOX:**
   - `src/rob_box_voice/config/voice_assistant.yaml` (system_phrases)
   - Команды движения, навигация, ошибки

3. **Ручная генерация через LLM:**
   ```bash
   # Prompt для ChatGPT/DeepSeek (если без API):
   "Сгенерируй 300 разнообразных фраз для автономного робота:
   - Системные сообщения
   - Команды движения
   - Диалоговые фразы
   - Ошибки и предупреждения
   Каждая фраза на новой строке."
   ```

4. **Публичные корпусы:**
   - [Tatoeba Russian](https://tatoeba.org/rus/sentences/search) - короткие фразы
   - [OpenCorpora](http://opencorpora.org/) - русский корпус

**💡 Совет:** Комбинируйте DeepSeek генерацию + ручной корпус для максимального разнообразия!

### 1.2 Настройка Yandex Cloud API

Получите API ключи:

1. Зарегистрируйтесь в [Yandex Cloud](https://cloud.yandex.ru/)
2. Создайте [API ключ](https://cloud.yandex.ru/docs/iam/operations/api-key/create)
3. Узнайте Folder ID из консоли

Экспортируйте переменные окружения:

```bash
export YANDEX_API_KEY="AQVNxxxxxxxxxxxxxxxxxxxxxxxx"
export YANDEX_FOLDER_ID="b1gxxxxxxxxxxxxxxxx"
```

### 1.3 Запись аудио через Yandex SpeechKit

Установите зависимости:

```bash
pip install requests soundfile numpy
```

Запустите запись:

```bash
# Базовый пример (тестовый корпус)
python3 record_yandex_voice.py \
  --input dataset/robbox_sentences_example.txt \
  --output dataset/robbox_voice_test

# Полная запись для fine-tuning (20-30 минут)
python3 record_yandex_voice.py \
  --input dataset/my_sentences.txt \
  --output dataset/robbox_voice_finetune \
  --voice anton \
  --emotion neutral \
  --speed 1.0 \
  --delay 1.5

# Продолжить с фразы 150 (если прервалось)
python3 record_yandex_voice.py \
  --input dataset/my_sentences.txt \
  --output dataset/robbox_voice_finetune \
  --resume-from 150
```

**Параметры:**

- `--voice`: anton (мужской), alena (женский), ermil (нейтральный)
- `--emotion`: neutral, good, evil
- `--speed`: 0.1-3.0 (рекомендуется 0.8-1.2)
- `--delay`: задержка между запросами (1-2 сек, чтобы не забанили API)

**Результат:**

```
dataset/robbox_voice_finetune/
├── metadata.csv              # Метаданные (filename|text|duration)
├── robbox_00000.wav
├── robbox_00001.wav
├── ...
└── robbox_00299.wav
```

### 1.4 Проверка качества аудио

Проверьте записи:

```bash
# Прослушать несколько файлов
aplay dataset/robbox_voice_finetune/robbox_00000.wav
aplay dataset/robbox_voice_finetune/robbox_00100.wav

# Статистика датасета
python3 << 'EOF'
import pandas as pd

df = pd.read_csv('dataset/robbox_voice_finetune/metadata.csv', sep='|')
print(f"Всего фраз: {len(df)}")
print(f"Общая длительность: {df['duration'].sum() / 60:.1f} минут")
print(f"Средняя длительность фразы: {df['duration'].mean():.1f}s")
print(f"Самая короткая: {df['duration'].min():.1f}s")
print(f"Самая длинная: {df['duration'].max():.1f}s")
EOF
```

**Требования к качеству:**

- ✅ Чистый звук без артефактов
- ✅ Ровная громкость
- ✅ Нет обрезанных фраз
- ✅ Соответствие текста и аудио

## Шаг 2: Обучение модели

### Вариант A: Fine-tuning Silero (рекомендуется) ⭐

**Преимущества:**
- Меньше данных (20-30 минут)
- Быстрее обучение (1-3 дня)
- Дешевле ($60-200)

**Процесс:**

1. **Клонируйте репозиторий Silero:**
   ```bash
   git clone https://github.com/snakers4/silero-models.git
   cd silero-models
   ```

2. **Подготовьте данные в формате Silero:**
   ```python
   # Скрипт convert_to_silero_format.py
   # TODO: Нужно адаптировать metadata.csv к формату Silero
   ```

3. **Обучение на GPU:**
   ```bash
   # Используйте Google Colab Pro+ ($50/month) или Vast.ai
   # Код обучения: https://github.com/snakers4/silero-models
   ```

**Стоимость:**
- Google Colab Pro+: $50/month (100 compute units)
- Vast.ai RTX 3090: $0.50-0.80/hour × 48 hours = $24-38
- Собственная GPU RTX 3090: ~$1200 (разовая покупка)

### Вариант B: Voice Cloning (быстрый тест)

**Преимущества:**
- Быстро (несколько часов)
- Мало данных (5-10 минут)
- Низкая сложность

**Недостатки:**
- Качество ниже (3-4 из 5)
- Возможна нестабильность

**Инструменты:**

1. **Coqui XTTS:**
   ```bash
   pip install TTS
   
   # Voice cloning
   tts --text "Привет, я робот ROBBOX" \
       --model_name tts_models/multilingual/multi-dataset/xtts_v2 \
       --speaker_wav dataset/robbox_voice_test/robbox_00000.wav \
       --language_idx ru \
       --out_path output.wav
   ```

2. **RVC (Retrieval-based Voice Conversion):**
   - Более сложен, но лучшее качество
   - https://github.com/RVC-Project/Retrieval-based-Voice-Conversion-WebUI

### Вариант C: Полное обучение с нуля

**Требования:**
- 3-5 часов аудио (3000-5000 фраз)
- RTX 3090/4090 или cloud GPU
- 5-14 дней обучения

**Когда использовать:**
- Когда нужно максимальное качество
- Есть большой бюджет ($400-2000)
- Есть время и экспертиза в ML

**Фреймворки:**

1. **Piper (VITS-based):**
   - https://github.com/rhasspy/piper
   - Хорошая документация
   - Производство-ready

2. **Coqui TTS:**
   - https://github.com/coqui-ai/TTS
   - Больше гибкости
   - Сложнее настройка

## Шаг 3: Интеграция в ROBBOX

После обучения модели, интегрируем её в проект:

### 3.1 Копируем модель

```bash
# Копируем обученную модель
cp trained_model/robbox_custom.pt /models/

# Или в Docker
docker cp trained_model/robbox_custom.pt voice-assistant:/models/
```

### 3.2 Обновляем конфигурацию

Файл: `src/rob_box_voice/config/voice_assistant.yaml`

```yaml
tts_node:
  provider: "silero"
  
  silero:
    model_path: "/models/robbox_custom.pt"  # Наша кастомная модель!
    speaker: "robbox"  # Имя нашего голоса
    sample_rate: 48000
    threads: 4
    pitch: "medium"
    rate: "medium"
    use_ssml: true
```

### 3.3 Тестируем

```bash
# Запускаем GUI для тестирования
cd src/rob_box_voice/scripts
python3 silero_tts_gui.py

# Или через командную строку
python3 << 'EOF'
import torch

model = torch.package.PackageImporter("/models/robbox_custom.pt").load_pickle("tts_models", "model")
model.to('cpu')

audio = model.apply_tts(
    text="Привет! Я ROBBOX с новым голосом!",
    speaker='robbox',
    sample_rate=48000
)

import sounddevice as sd
sd.play(audio.numpy(), 48000)
sd.wait()
EOF
```

## Рекомендации

### Для начала (тестирование идеи):

1. ✅ Запишите **50-100 фраз** (5-10 минут)
2. ✅ Попробуйте **voice cloning** через Coqui XTTS
3. ✅ Оцените качество и решите нужен ли fine-tuning

### Для production (качественный результат):

1. ✅ Запишите **200-300 фраз** (20-30 минут)
2. ✅ Сделайте **fine-tuning Silero** на cloud GPU
3. ✅ Интегрируйте в ROBBOX

### НЕ рекомендуется сейчас:

- ❌ Полное обучение с нуля (дорого и долго)
- ❌ Коммерческий сервис Silero ($5000-15000)

### Альтернатива (если не получится):

Используйте готовые голоса Silero (aidar, baya, kseniya, xenia) + SSML для кастомизации:

```python
# Низкий pitch = более серьёзный ROBBOX
ssml = '<speak><prosody pitch="low" rate="medium">Я робот ROBBOX</prosody></speak>'
```

## Полезные ссылки

- [Silero Models GitHub](https://github.com/snakers4/silero-models)
- [Coqui TTS](https://github.com/coqui-ai/TTS)
- [Piper TTS](https://github.com/rhasspy/piper)
- [RVC Voice Conversion](https://github.com/RVC-Project/Retrieval-based-Voice-Conversion-WebUI)
- [Google Colab Pro+](https://colab.research.google.com/signup)
- [Vast.ai GPU Rental](https://vast.ai/)

## Стоимость (итого)

| Компонент | Стоимость |
|-----------|-----------|
| Yandex API (запись 30 мин) | ~$2-5 |
| Cloud GPU (fine-tuning 48h) | $24-100 |
| USB микрофон (если записывать вживую) | $50-150 |
| **Итого (fine-tuning):** | **$76-255** |

## Заключение

**Создание кастомного голоса ROBBOX возможно!**

**Рекомендуемый путь:**
1. Записать 200-300 фраз через Yandex API (20-30 минут аудио)
2. Fine-tuning Silero на cloud GPU (1-3 дня, $60-200)
3. Интегрировать в ROBBOX

**Или проще:**
Использовать готовые голоса Silero (aidar с pitch="low") - они уже отличного качества!

Удачи! 🤖🎙️
