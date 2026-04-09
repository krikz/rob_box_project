# STT/TTS Quick Reference для rob_box_voice

Быстрый справочник по выбору Speech-to-Text и Text-to-Speech решений для русского языка.

## 🎯 TL;DR - Рекомендации

| Компонент | Primary | Fallback | Зачем Fallback |
|-----------|---------|----------|----------------|
| **STT** | Vosk (offline) | Yandex STT (online) | Низкая уверенность (<70%) |
| **TTS** | Piper (offline) | Yandex TTS (online) | Важные сообщения |
| **LLM** | DeepSeek (online) | Ollama (offline) | Полная автономность |

**Memory Budget:** ~1.5 GB (audio + vosk + piper + llm) ✅ Fits в 2GB

---

## 📊 Сравнительная таблица

### STT (Speech-to-Text)

| Решение | Online | Точность | Latency | RAM | Best For |
|---------|--------|----------|---------|-----|----------|
| **Vosk small** ⭐ | ❌ | 75% | <0.5s | 500MB | Real-time, offline |
| Whisper base | ❌ | 80% | 1-2s | 1GB | Accuracy + offline |
| Whisper small | ❌ | 85% | 2-3s | 2GB | Max accuracy offline |
| Yandex STT | ✅ | 95% | <1s | 0 | Fallback, online |

**Рекомендация:** `Vosk small` для основной работы + `Yandex STT` как fallback

---

### TTS (Text-to-Speech)

| Решение | Online | Качество | Latency | RAM | Интонации | Best For |
|---------|--------|----------|---------|-----|-----------|----------|
| **Piper** ⭐ | ❌ | ⭐⭐⭐⭐ | <0.5s | 100MB | ⭐⭐ | Fast, quality, offline |
| Silero | ❌ | ⭐⭐⭐⭐ | 1-2s | 1.5GB | ⭐⭐⭐ | Multiple voices |
| RHVoice | ❌ | ⭐⭐ | <0.3s | 50MB | ⭐ | Very fast, low resource |
| Yandex TTS | ✅ | ⭐⭐⭐⭐⭐ | <0.5s | 0 | ⭐⭐⭐⭐ | Fallback, emotions |

**Рекомендация:** `Piper (dmitri)` для основной работы + `Yandex TTS` для важных сообщений

---

## 🚀 Быстрая настройка

### 1. Установка зависимостей

```bash
# В Dockerfile уже включено:
pip install vosk piper-tts faster-whisper
```

### 2. Скачивание моделей

```bash
# Vosk STT (45 MB)
wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
unzip vosk-model-small-ru-0.22.zip -d /models/

# Piper TTS (63 MB × 2)
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/ru_RU-dmitri-medium.onnx
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/ru_RU-irina-medium.onnx
```

### 3. Конфигурация

```yaml
# config/voice_assistant.yaml
stt_node:
  provider: "vosk"
  vosk:
    model_path: "/models/vosk-model-small-ru-0.22"
    confidence_threshold: 0.7
  fallback_provider: "yandex"

tts_node:
  provider: "piper"
  piper:
    model_path: "/models/ru_RU-dmitri-medium.onnx"
  fallback_provider: "yandex"
```

---

## 💡 Примеры использования

### STT Node (Vosk)

```python
from vosk import Model, KaldiRecognizer

# Инициализация
model = Model("/models/vosk-model-small-ru-0.22")
recognizer = KaldiRecognizer(model, 16000)

# Распознавание (streaming)
if recognizer.AcceptWaveform(audio_data):
    result = json.loads(recognizer.Result())
    text = result['text']
    confidence = result.get('confidence', 0.0)
    
    # Fallback на Yandex если уверенность низкая
    if confidence < 0.7:
        text = yandex_stt(audio_data)
```

### TTS Node (Piper)

```python
from piper import PiperVoice

# Инициализация
voice = PiperVoice.load("/models/ru_RU-dmitri-medium.onnx")

# Синтез
audio = voice.synthesize("Привет! Я голосовой ассистент робота.")
# audio - numpy array, ready to publish
```

---

## 🔍 Когда использовать что?

### Vosk vs Whisper

**Используй Vosk если:**
- ✅ Нужен real-time (streaming) STT
- ✅ Критичны ресурсы (<1GB RAM)
- ✅ Latency < 1s обязателен

**Используй Whisper если:**
- ✅ Точность важнее скорости
- ✅ Есть 2+ GB RAM
- ✅ Batch processing (не streaming)

### Piper vs Silero

**Используй Piper если:**
- ✅ Нужна скорость (<0.5s)
- ✅ Хватит 1-2 голосов
- ✅ Критична память (<200MB)

**Используй Silero если:**
- ✅ Нужно много голосов (4+)
- ✅ Есть 1.5GB RAM
- ✅ Нужны интонации

### Offline vs Online

**Используй Offline (Vosk/Piper) если:**
- ✅ Робот должен работать без интернета
- ✅ Нужна низкая latency
- ✅ Хочешь избежать API costs

**Используй Online (Yandex) как fallback если:**
- ✅ Vosk не уверен (<70%)
- ✅ Важное сообщение (эмоции нужны)
- ✅ Есть интернет

---

## 📦 Docker размеры

```
Base image:              1.2 GB
+ Vosk model:            + 45 MB
+ Piper models:          + 126 MB (2 voices)
+ Python deps:           + 150 MB
──────────────────────────────────
Total:                   ~1.52 GB
```

**Альтернатива (если нужна экономия):**
- Vosk only: 45 MB
- Piper 1 voice: 63 MB
- Total: ~1.4 GB

---

## 🎤 Голоса

### Piper (русский)

| Голос | Пол | Качество | Размер | Описание |
|-------|-----|----------|--------|----------|
| `dmitri` | M | ⭐⭐⭐⭐ | 63 MB | Чёткий, спокойный |
| `irina` | F | ⭐⭐⭐⭐ | 63 MB | Тёплый, дружелюбный |

### Silero (русский)

| Голос | Пол | Качество | Описание |
|-------|-----|----------|----------|
| `aidar` | M | ⭐⭐⭐⭐ | Мужской, нейтральный |
| `baya` | F | ⭐⭐⭐⭐ | Женский, тёплый |
| `kseniya` | F | ⭐⭐⭐⭐ | Женский, энергичный |
| `xenia` | F | ⭐⭐⭐⭐ | Молодой, дружелюбный |

### Yandex TTS (fallback)

| Голос | Пол | Эмоции | Описание |
|-------|-----|--------|----------|
| `anton` | M | ✅ | Мужской, нейтральный |
| `alena` | F | ✅ | Женский, приятный |

---

## 🛠️ Troubleshooting

### Vosk медленно работает

```bash
# Используй меньшую модель
wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip

# Или используй GPU (если есть)
# Vosk не поддерживает GPU, используй Whisper с CUDA
```

### Piper не хватает качества

```bash
# Используй Silero с лучшими интонациями
pip install torch torchaudio

# Или fallback на Yandex TTS для важных фраз
```

### Out of Memory

```yaml
# Уменьши memory footprint:
stt_node:
  provider: "vosk"  # Вместо whisper
  
tts_node:
  provider: "piper"  # Вместо silero

# Удали альтернативные модели из Docker image
```

---

## 📚 Дополнительная документация

- **Полный research:** `docs/development/STT_TTS_RESEARCH.md`
- **ROS2 package:** `src/rob_box_voice/README.md`
- **Конфигурация:** `docker/vision/config/voice_assistant/voice_assistant.yaml`

---

**Дата:** 2025-01-12  
**Статус:** Production Ready ✅
