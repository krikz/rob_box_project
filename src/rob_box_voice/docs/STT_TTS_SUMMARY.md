# 🎯 STT/TTS Research Summary для ROBBOX Voice Assistant

## Ключевые выводы

### ✅ Рекомендованное решение (Offline-First)

```
STT: Vosk (small) → Yandex (fallback)
TTS: Piper (dmitri) → Yandex (fallback)
```

**Почему:**
1. **Полная автономность** - работает без интернета
2. **Быстро** - <1s latency для STT+TTS
3. **Качественно** - neural TTS, хороший звук
4. **Легковесно** - 1.5GB RAM (укладывается в бюджет 2GB)
5. **Надёжно** - fallback на облако при проблемах

---

## 📊 Сравнение решений

### STT (Speech-to-Text)

| Решение | ❌ Julius | ❌ ros_speech_recognition | ✅ Vosk | ✅ Whisper | ☁️ Yandex |
|---------|-----------|---------------------------|---------|------------|-----------|
| **Русский** | ❌ Нет | ❌ Нет | ✅ Да | ✅ Да | ✅ Да |
| **ROS2** | ❌ ROS1 | ❌ ROS1 | ✅ Легко | ✅ Легко | ✅ Легко |
| **Offline** | ✅ | ✅ | ✅ | ✅ | ❌ |
| **Latency** | <0.5s | <1s | <0.5s | 1-3s | <1s |
| **Точность** | - | - | 75% | 80-90% | 95% |
| **RAM** | <500MB | <500MB | 500MB | 1-5GB | ~0 |
| **Вердикт** | ❌ Устарел | ❌ Устарел | ⭐ **Best** | ⚠️ Медленно | ☁️ Fallback |

**Вывод для проекта:**
- **Julius** и **ros_speech_recognition** - НЕ подходят (нет русского + устарели)
- **Vosk** - идеально для real-time offline STT ⭐
- **Whisper** - хорошо для batch processing или если нужна высокая точность
- **Yandex** - отличный fallback когда Vosk не уверен

---

### TTS (Text-to-Speech)

| Решение | ✅ Piper | ✅ Silero | ⚠️ RHVoice | ❌ Coqui | ☁️ Yandex |
|---------|----------|-----------|------------|----------|-----------|
| **Качество** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Интонации** | ⭐⭐ | ⭐⭐⭐ | ⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Latency** | <0.5s | 1-2s | <0.3s | 2-3s | <0.5s |
| **RAM** | 100MB | 1.5GB | 50MB | 2GB+ | ~0 |
| **Голосов** | 2 | 4 | 4 | Много | 4 |
| **Вердикт** | ⭐ **Best** | ⚠️ Тяжёлый | ⚠️ Робот | ❌ Мёртв | ☁️ Fallback |

**Вывод для проекта:**
- **Piper** - идеально для production (быстро + качественно) ⭐
- **Silero** - хорошая альтернатива если нужны разные голоса
- **RHVoice** - звучит как робот, не рекомендуется
- **Coqui TTS** - проект заморожен, НЕ использовать
- **Yandex TTS** - fallback для важных сообщений с эмоциями

---

## 🚀 Реализация

### Архитектура (Hybrid Offline-First)

```
┌─────────────────────────────────────────┐
│          Audio Input (ReSpeaker)        │
└──────────────┬──────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────┐
│           STT Node (Vosk)                │
│  ┌────────────────────────────────────┐  │
│  │  1. Stream audio to Vosk           │  │
│  │  2. Get recognition + confidence   │  │
│  │  3. If confidence < 70%:           │  │
│  │     → Fallback to Yandex STT       │  │
│  └────────────────────────────────────┘  │
└──────────────┬───────────────────────────┘
               │ text
               ▼
┌──────────────────────────────────────────┐
│         Dialogue Node (LLM)              │
│         DeepSeek / Local Ollama          │
└──────────────┬───────────────────────────┘
               │ response
               ▼
┌──────────────────────────────────────────┐
│          TTS Node (Piper)                │
│  ┌────────────────────────────────────┐  │
│  │  1. Synthesize with Piper          │  │
│  │  2. Cache result                   │  │
│  │  3. For important messages:        │  │
│  │     → Use Yandex TTS (emotions)    │  │
│  └────────────────────────────────────┘  │
└──────────────┬───────────────────────────┘
               │ audio
               ▼
┌──────────────────────────────────────────┐
│      Audio Output (Speaker/Jack)         │
└──────────────────────────────────────────┘
```

---

### Memory Budget

```
Component            RAM      Notes
─────────────────────────────────────────
Audio Node           200 MB   PyAudio + buffers
Vosk STT             500 MB   Model in memory
Piper TTS            100 MB   ONNX model
Dialogue Node        500 MB   LLM context
LED/Animations       200 MB   rob_box_animations
─────────────────────────────────────────
TOTAL               ~1.5 GB   ✅ Fits in 2GB budget

Optional alternatives:
- Whisper base      +500 MB   (if need accuracy)
- Silero TTS        +1.3 GB   (if need voices)
```

---

### Docker Image Size

```
Layer                     Size        Notes
──────────────────────────────────────────────
Base (ros2-zenoh)         1.2 GB      Already there
Audio libs                +50 MB      ALSA, PyAudio
ReSpeaker drivers         +20 MB      pixel_ring, tuning
Vosk + deps               +60 MB      Python package
Vosk model (small)        +45 MB      Russian small
Whisper + deps            +100 MB     Optional
Piper + deps              +30 MB      ONNX runtime
Piper models (×2)         +126 MB     dmitri + irina
Torch (for Silero)        +500 MB     Optional, if needed
──────────────────────────────────────────────
TOTAL (recommended)       ~1.65 GB    Vosk + Piper only
TOTAL (full)              ~2.15 GB    All alternatives
```

**Рекомендация:** Собирать только Vosk + Piper для экономии (~1.65 GB)

---

## 🎤 Выбор голосов

### Мужские голоса

| Голос | Engine | Качество | Характер | Для робота |
|-------|--------|----------|----------|------------|
| **dmitri** (Piper) | Neural | ⭐⭐⭐⭐ | Спокойный, чёткий | ✅ **Идеально** |
| aidar (Silero) | Neural | ⭐⭐⭐⭐ | Нейтральный | ✅ Хорошо |
| anton (Yandex) | Neural | ⭐⭐⭐⭐⭐ | Дружелюбный | ✅ Fallback |

### Женские голоса

| Голос | Engine | Качество | Характер | Для робота |
|-------|--------|----------|----------|------------|
| **irina** (Piper) | Neural | ⭐⭐⭐⭐ | Тёплый | ✅ Хорошо |
| baya (Silero) | Neural | ⭐⭐⭐⭐ | Приятный | ✅ Хорошо |
| alena (Yandex) | Neural | ⭐⭐⭐⭐⭐ | Мягкий | ✅ Fallback |

**Рекомендация для ROBBOX:** `dmitri` (Piper) - чёткий мужской голос, хорошо подходит для робота

---

## 📝 Конфигурация

### Минимальная (только offline)

```yaml
stt_node:
  provider: "vosk"
  vosk:
    model_path: "/models/vosk-model-small-ru-0.22"

tts_node:
  provider: "piper"
  piper:
    model_path: "/models/ru_RU-dmitri-medium.onnx"
```

### Рекомендуемая (hybrid)

```yaml
stt_node:
  provider: "vosk"
  vosk:
    model_path: "/models/vosk-model-small-ru-0.22"
    confidence_threshold: 0.7
  fallback_provider: "yandex"
  yandex:
    use_when_confidence_below: 0.7

tts_node:
  provider: "piper"
  piper:
    model_path: "/models/ru_RU-dmitri-medium.onnx"
  fallback_provider: "yandex"
  yandex:
    use_for_important: true
    emotion: "good"
```

---

## ⚡ Performance

### Latency Breakdown

```
Component          Latency     Notes
─────────────────────────────────────────
VAD detection      ~50ms       ReSpeaker hardware
Audio buffering    ~100ms      Chunk accumulation
Vosk STT           ~300ms      Real-time streaming
Piper TTS          ~200ms      ONNX inference
Audio playback     ~100ms      ALSA buffering
─────────────────────────────────────────
TOTAL              ~750ms      ✅ <1s target met

For comparison (online):
Yandex STT         ~500ms      Network + API
Yandex TTS         ~300ms      Network + API
─────────────────────────────────────────
Hybrid TOTAL       ~900ms      Still <1s!
```

---

## 🔧 Альтернативные сценарии

### Сценарий 1: Максимальная точность (медленнее)

```yaml
stt_node:
  provider: "whisper"
  whisper:
    model: "small"  # 85% accuracy, 2-3s latency

tts_node:
  provider: "silero"
  silero:
    speaker: "baya"  # Multiple voices
```

**Tradeoff:** +1-2s latency, +1.5GB RAM

### Сценарий 2: Минимальная память

```yaml
stt_node:
  provider: "vosk"
  vosk:
    model_path: "/models/vosk-model-small-ru-0.22"  # 45 MB

tts_node:
  provider: "rhvoice"  # 50 MB, но звучит как робот
```

**Tradeoff:** Хуже качество TTS (звучит роботизированно)

### Сценарий 3: Только облачное (не рекомендуется)

```yaml
stt_node:
  provider: "yandex"

tts_node:
  provider: "yandex"
```

**Tradeoff:** Требует постоянный интернет, платно, зависимость от облака

---

## 🎯 Финальное решение для ROBBOX

### Primary Setup

```yaml
STT: Vosk small (45 MB, 500 MB RAM, <0.5s)
TTS: Piper dmitri (63 MB, 100 MB RAM, <0.5s)
LLM: DeepSeek API (online, но можно Ollama для offline)

Fallbacks:
- Vosk confidence <70% → Yandex STT
- Important messages → Yandex TTS (emotions)

Total RAM: ~1.5 GB ✅
Total latency: ~750ms ✅
Offline: ✅ (с smart fallback)
```

### Benefits

✅ **Автономность** - работает без интернета
✅ **Скорость** - <1s от речи до ответа
✅ **Качество** - neural TTS, хороший голос
✅ **Память** - укладывается в 2GB budget
✅ **Fallback** - умное переключение на облако
✅ **Cost** - бесплатно (offline), дешёво (fallback)

---

## 📚 Ресурсы

- **Полный research:** `docs/development/STT_TTS_RESEARCH.md` (12+ страниц)
- **Quick reference:** `docs/development/STT_TTS_QUICK_REFERENCE.md`
- **Конфигурация:** `docker/vision/config/voice_assistant/voice_assistant.yaml`
- **Dockerfile:** `docker/vision/voice_assistant/Dockerfile`

### Ссылки

**STT:**
- Vosk: https://alphacephei.com/vosk/
- Whisper: https://github.com/openai/whisper
- Yandex STT: https://cloud.yandex.ru/docs/speechkit/stt/

**TTS:**
- Piper: https://github.com/rhasspy/piper
- Silero: https://github.com/snakers4/silero-models
- Yandex TTS: https://cloud.yandex.ru/docs/speechkit/tts/

**ROS2:**
- audio_common: https://github.com/ros2/audio_common

---

**Дата:** 2025-01-12  
**Статус:** Research Complete ✅  
**Следующий шаг:** Реализация STT/TTS nodes в rob_box_voice
