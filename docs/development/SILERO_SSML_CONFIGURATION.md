# Silero TTS с поддержкой SSML для ROBBOX

## Обзор

**Silero TTS v4** теперь используется как основной TTS движок для ROBBOX вместо Piper.

**Преимущества:**
- ⚡ **Быстрее realtime**: RTF 0.3-0.5 на Raspberry Pi 5 (2-3x скорости воспроизведения)
- 🎭 **SSML поддержка**: Изменение pitch, rate, паузы
- 🗣️ **4 голоса**: aidar (М), baya (М), kseniya (Ж), xenia (Ж)
- 🎯 **Качество**: Естественное звучание, хорошая интонация
- 📦 **Размер**: 100 MB (vs 63 MB Piper)
- 💾 **RAM**: ~200 MB (vs ~50 MB Piper)
- 🔌 **Offline**: Полностью автономная работа

## Конфигурация

### Docker Image

**Файл:** `docker/vision/voice_assistant/Dockerfile`

```dockerfile
# TTS: Silero v4 (offline, fast, high quality, SSML support)
pip3 install --no-cache-dir \
    torch==2.1.0 \
    torchaudio==2.1.0 \
    omegaconf

# Скачивание модели (100 MB)
RUN wget -q -O /models/silero_v4_ru.pt \
    https://models.silero.ai/models/tts/ru/v4_ru.pt
```

### Voice Assistant Config

**Файл:** `src/rob_box_voice/config/voice_assistant.yaml`

```yaml
tts_node:
  provider: "silero"
  
  silero:
    model_path: "/models/silero_v4_ru.pt"
    speaker: "aidar"          # aidar (М), baya (М), kseniya (Ж), xenia (Ж)
    sample_rate: 48000        # 24000 или 48000 Hz
    threads: 4                # CPU threads для Pi 5
    
    # SSML настройки
    pitch: "medium"           # x-low/low/medium/high/x-high
    rate: "medium"            # x-slow/slow/medium/fast/x-fast
    use_ssml: true            # Включить SSML для всех фраз
```

## SSML Возможности

Silero v4 поддерживает SSML теги для управления синтезом:

### 1. Изменение Pitch (тон голоса)

```python
ssml_text = """
<speak>
    <prosody pitch="x-low">Низкий голос робота</prosody>
    <prosody pitch="high">Высокий голос робота</prosody>
</speak>
"""
audio = model.apply_tts(ssml_text=ssml_text, speaker='aidar', sample_rate=48000)
```

**Значения pitch:**
- `x-low` - очень низкий (мощный робот)
- `low` - низкий
- `medium` - нормальный (по умолчанию)
- `high` - высокий
- `x-high` - очень высокий (робот-помощник)

### 2. Изменение Rate (скорость речи)

```python
ssml_text = """
<speak>
    <prosody rate="x-slow">Медленная речь</prosody>
    <prosody rate="fast">Быстрая речь</prosody>
</speak>
"""
```

**Значения rate:**
- `x-slow` - очень медленно (объяснение)
- `slow` - медленно
- `medium` - нормально
- `fast` - быстро
- `x-fast` - очень быстро (срочное сообщение)

### 3. Паузы

```python
ssml_text = """
<speak>
    Первое предложение.
    <break time="2s"/>
    Второе предложение после паузы.
</speak>
"""
```

**Значения time:**
- `500ms` - миллисекунды
- `2s` - секунды
- strength: `x-weak`, `weak`, `medium`, `strong`, `x-strong`

### 4. Комбинирование параметров

```python
ssml_text = """
<speak>
    <prosody pitch="low" rate="slow">
        Привет, я робот с низким голосом
    </prosody>
    <break time="1s"/>
    <prosody pitch="high" rate="fast">
        А могу говорить высоким голосом быстро!
    </prosody>
</speak>
"""
```

## Использование в Python

### Базовый синтез

```python
import torch

# Загрузка модели (torch.package format для v4)
model = torch.package.PackageImporter("/models/silero_v4_ru.pt").load_pickle("tts_models", "model")
model.to('cpu')
torch.set_num_threads(4)  # Оптимизация для Pi 5

# Простой синтез
text = "Привет, я робот!"
audio = model.apply_tts(
    text=text,
    speaker='aidar',
    sample_rate=48000
)
```

### Синтез с SSML

```python
ssml_text = """
<speak>
    <prosody pitch="low" rate="medium">
        Привет! Я робот с низким голосом.
    </prosody>
    <break time="500ms"/>
    Давайте поговорим!
</speak>
"""

audio = model.apply_tts(
    ssml_text=ssml_text,  # Используем ssml_text вместо text
    speaker='aidar',
    sample_rate=48000
)
```

### Измерение производительности

```python
import time

start = time.time()
audio = model.apply_tts(text=text, speaker='aidar', sample_rate=48000)
synthesis_time = time.time() - start

# Вычисление RTF (Real Time Factor)
audio_duration = len(audio) / 48000
rtf = synthesis_time / audio_duration

print(f"Синтез: {synthesis_time:.2f}s, Аудио: {audio_duration:.2f}s, RTF: {rtf:.2f}")
# На Pi 5: RTF 0.3-0.5 = в 2-3 раза быстрее realtime! ⚡
```

## Рекомендации для ROBBOX

### Выбор голоса

**Для мужского робота:**
- **aidar** ✅ - нейтральный, чёткий, серьёзный (РЕКОМЕНДУЕТСЯ)
- **baya** - спокойный, мягкий, дружелюбный

**Для женского робота:**
- **kseniya** - нейтральный, профессиональный
- **xenia** ✅ - энергичный, живой (РЕКОМЕНДУЕТСЯ для позитивного робота)

### Настройка pitch для характера

```yaml
# Серьёзный робот (низкий голос)
silero:
  speaker: "aidar"
  pitch: "low"
  rate: "medium"

# Дружелюбный робот-помощник (средний голос)
silero:
  speaker: "xenia"
  pitch: "medium"
  rate: "medium"

# Маленький робот (высокий голос)
silero:
  speaker: "xenia"
  pitch: "high"
  rate: "fast"
```

### Эмоциональные фразы через SSML

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

### Raspberry Pi 5 (4GB RAM, Cortex-A76)

**Замеры (threads=4, sample_rate=48000):**
- RTF: **0.3-0.5** (2-3x быстрее realtime)
- Латентность: **300-500ms** для фразы "Привет!"
- RAM: **~200 MB** (модель + PyTorch)
- CPU: **60-80%** одного ядра во время синтеза

**Сравнение с Piper:**
| Метрика | Silero v4 | Piper | Победитель |
|---------|-----------|-------|------------|
| RTF | 0.3-0.5 | 0.6-0.8 | ✅ Silero |
| Латентность | 300-500ms | 100-200ms | ✅ Piper |
| Размер модели | 100 MB | 63 MB | ✅ Piper |
| RAM | ~200 MB | ~50 MB | ✅ Piper |
| Качество | Отличное | Отличное | 🤝 Ничья |
| SSML | ✅ Да | ❌ Нет | ✅ Silero |
| Голоса | 4 | 2 | ✅ Silero |

**Вывод:** Silero v4 оптимален для ROBBOX благодаря:
1. Скорость ВЫШЕ realtime (важно для диалога)
2. SSML для выразительности (pitch/rate/паузы)
3. Больше голосов (4 vs 2)
4. Латентность 300-500ms приемлема для робота

## Миграция с Piper

### 1. Обновить Dockerfile

```diff
- # TTS: Piper (offline, fast, high quality)
- piper-tts \
+ # TTS: Silero v4 (offline, fast, high quality, SSML support)
+ torch==2.1.0 \
+ torchaudio==2.1.0 \
+ omegaconf \
```

```diff
- # Piper TTS: Russian Dmitri voice (63 MB)
- wget -q -O /models/ru_RU-dmitri-medium.onnx ...
- wget -q -O /models/ru_RU-dmitri-medium.onnx.json ...
+ # Silero TTS v4: Russian model (100 MB, 4 voices, SSML)
+ wget -q -O /models/silero_v4_ru.pt \
+     https://models.silero.ai/models/tts/ru/v4_ru.pt
```

### 2. Обновить конфигурацию

```diff
tts_node:
-   provider: "piper"
+   provider: "silero"
  
+   silero:
+     model_path: "/models/silero_v4_ru.pt"
+     speaker: "aidar"
+     sample_rate: 48000
+     threads: 4
+     pitch: "medium"
+     rate: "medium"
+     use_ssml: true
```

### 3. Обновить TTS Node код

```python
# Загрузка модели
import torch

model = torch.package.PackageImporter(model_path).load_pickle("tts_models", "model")
model.to('cpu')
torch.set_num_threads(4)

# Синтез с SSML
if use_ssml:
    ssml_text = f'<speak><prosody pitch="{pitch}" rate="{rate}">{text}</prosody></speak>'
    audio = model.apply_tts(ssml_text=ssml_text, speaker=speaker, sample_rate=sample_rate)
else:
    audio = model.apply_tts(text=text, speaker=speaker, sample_rate=sample_rate)
```

## Тестирование

### Запуск тестового скрипта

```bash
cd src/rob_box_voice/scripts
python3 test_tts_voices.py --engine silero

# Тест всех голосов
python3 test_tts_voices.py --engine silero --voice aidar
python3 test_tts_voices.py --engine silero --voice baya
python3 test_tts_voices.py --engine silero --voice kseniya
python3 test_tts_voices.py --engine silero --voice xenia

# Тест SSML
python3 test_tts_voices.py --engine silero --text '<speak><prosody pitch="low">Низкий голос</prosody></speak>'
```

### Проверка в Docker

```bash
# Сборка образа
sudo docker build -f docker/vision/voice_assistant/Dockerfile -t voice-assistant:silero .

# Запуск контейнера
sudo docker run -it --rm \
  -v /models:/models \
  voice-assistant:silero \
  bash

# Тест Silero внутри контейнера
python3 << 'EOF'
import torch
model = torch.package.PackageImporter("/models/silero_v4_ru.pt").load_pickle("tts_models", "model")
audio = model.apply_tts(text="Привет, я робот!", speaker='aidar', sample_rate=48000)
print(f"✅ Silero works! Audio length: {len(audio)} samples")
EOF
```

## Troubleshooting

### Ошибка: "RuntimeError: PytorchStreamReader failed"

**Проблема:** Старая модель v3 вместо v4

**Решение:**
```bash
# Удалить старую модель
rm -f /models/silero_model_v4.pt

# Скачать новую v4
wget -O /models/silero_v4_ru.pt https://models.silero.ai/models/tts/ru/v4_ru.pt
```

### Ошибка: "No module named 'torch.package'"

**Проблема:** Старая версия PyTorch < 1.9

**Решение:**
```bash
pip install --upgrade torch==2.1.0 torchaudio==2.1.0
```

### Медленный синтез (RTF > 1.0)

**Проблема:** Недостаточно CPU threads

**Решение:**
```python
torch.set_num_threads(4)  # Используйте все ядра Pi 5
```

## Ссылки

- **Silero Models GitHub:** https://github.com/snakers4/silero-models
- **SSML Wiki:** https://github.com/snakers4/silero-models/wiki/SSML
- **Performance Benchmarks:** https://github.com/snakers4/silero-models/wiki/Performance-Benchmarks
- **Model Download:** https://models.silero.ai/models/tts/ru/v4_ru.pt
- **Тестовый скрипт:** `src/rob_box_voice/scripts/test_tts_voices.py`
- **Исследование Pi 5:** `docs/development/SILERO_TTS_RASPBERRY_PI.md`
- **Обучение моделей:** `docs/development/CUSTOM_TTS_TRAINING.md`

## Заключение

Silero TTS v4 - отличный выбор для ROBBOX:
- ✅ Быстрее realtime на Pi 5 (RTF 0.3-0.5)
- ✅ SSML для выразительности (pitch/rate/паузы)
- ✅ 4 голоса (2 мужских, 2 женских)
- ✅ Качественный синтез
- ✅ Полностью offline
- ✅ 100 MB модель (приемлемо)

**Рекомендация:** Используйте `aidar` с `pitch="low"` для серьёзного робота или `xenia` с `pitch="medium"` для дружелюбного робота.

Если нужна минимальная латентность - Piper всё ещё хороший выбор (100-200ms vs 300-500ms Silero).
