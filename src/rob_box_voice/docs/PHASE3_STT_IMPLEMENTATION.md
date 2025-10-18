# Phase 3: STT (Speech-to-Text) Implementation

## Обзор

**Статус**: ✅ Реализовано (Vosk)  
**Дата**: 2025-10-13  
**Выбранное решение**: Vosk offline recognition

## Архитектура

### STT Node

**Файл**: `rob_box_voice/stt_node.py` (217 строк)

**ROS Интерфейс**:
```
Subscribers:
  /audio/audio (AudioData)      - Аудио поток от audio_node
  /audio/vad (Bool)             - Voice Activity Detection

Publishers:
  /voice/stt/result (String)    - Финальный результат распознавания
  /voice/stt/partial (String)   - Частичный результат (во время речи)
  /voice/stt/state (String)     - Состояние ноды (ready/listening/error)
```

**Параметры**:
```yaml
model_path: /models/vosk-model-small-ru-0.22
sample_rate: 16000
vad_timeout: 1.5              # Секунды тишины → финальный результат
min_speech_duration: 0.5      # Минимальная длина речи
```

## Vosk Integration

### Выбор решения

Тестировались 3 варианта:
1. **Vosk** ✅ - Выбран
   - Скорость: 0.5-0.67s ⚡
   - Точность: 7-8/10 👍
   - Offline: ✅
   - CPU: Низкая нагрузка
   - Размер модели: 45 MB

2. **Whisper** ❌ - Отклонен
   - Скорость: 2-5s 🐌
   - Точность: 9/10 🌟
   - CPU: Высокая нагрузка
   - Проблемы с зависимостями

3. **Yandex** ⚠️ - Опционально
   - Скорость: 0.5-1s + сеть
   - Точность: 9/10
   - Требует интернет
   - API конфигурация сложная

### Vosk Model

**Модель**: `vosk-model-small-ru-0.22`  
**Размер**: 45 MB  
**Язык**: Русский  
**Установка**:
```bash
sudo mkdir -p /models
cd /tmp
wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
unzip vosk-model-small-ru-0.22.zip
sudo mv vosk-model-small-ru-0.22 /models/
```

**Альтернативы** (если нужна выше точность):
- `vosk-model-ru-0.42` - 1.9 GB, точность 9/10
- `vosk-model-ru-0.22` - 1.5 GB, точность 8.5/10

## Работа STT Node

### Алгоритм распознавания

1. **Ожидание речи**:
   - Слушает `/audio/vad` (Voice Activity Detection)
   - Когда VAD=True → начало речи

2. **Накопление аудио**:
   - Получает аудио поток из `/audio/audio`
   - Отправляет в Vosk `recognizer.AcceptWaveform()`
   - Публикует частичные результаты в `/voice/stt/partial`

3. **Детекция конца фразы**:
   - Когда VAD=False на 1.5s → конец речи
   - Или Vosk сам детектирует конец фразы

4. **Финальный результат**:
   - Вызов `recognizer.FinalResult()`
   - Публикация в `/voice/stt/result`
   - Сброс распознавателя для новой фразы

### Фильтрация шума

**Проблема**: Короткие звуки (шаги, стуки) могут вызвать ложные срабатывания.

**Решение**:
```python
if speech_duration < self.min_speech_duration:  # 0.5s
    self.get_logger().info('⚠️ Речь слишком короткая, игнорирую')
    self.reset_recognition()
```

## Интеграция с Dialogue Node

### Data Flow

```
Audio → STT → Dialogue → TTS → Speech
  ↓       ↓        ↓         ↓
ReSpeaker → Vosk → DeepSeek → Silero
```

**Топики**:
```
/audio/audio (AudioData)           → stt_node
/audio/vad (Bool)                  → stt_node
/voice/stt/result (String)         → dialogue_node
/voice/dialogue/response (String)  → tts_node
/voice/audio/speech (AudioData)    → audio_node/speakers
```

### Пример диалога

1. Пользователь: **"Привет робот как дела"**
   - `audio_node` → VAD=True
   - `stt_node` → "/voice/stt/result": "привет робот как дела"

2. Dialogue processing:
   - `dialogue_node` → DeepSeek API
   - Streaming response: "Привет! У меня всё отлично..."

3. TTS synthesis:
   - `tts_node` → Silero TTS
   - С accent_replacer: "Прив+ет! У мен+я всё отл+ично..."
   - Chipmunk mode (pitch 2.0x)

4. Audio playback:
   - `audio_node` → ReSpeaker 3.5mm jack

## Тестирование

### Unit Test Script

**Файл**: `scripts/test_stt_options.py`

**Возможности**:
- Интерактивное тестирование 3 STT движков
- Запись с ReSpeaker
- Сохранение WAV для отладки
- Замер времени распознавания

**Запуск**:
```bash
cd src/rob_box_voice/scripts
./quick_start_stt.sh
```

### Integration Test

**TODO**: Создать ROS2 integration test:
```bash
ros2 launch rob_box_voice voice_assistant.launch.py
# Говорить в микрофон и проверять:
# - /voice/stt/result - корректность распознавания
# - /voice/dialogue/response - ответ LLM
# - Аудио выход - синтез речи
```

## Performance

### Латентность (измерено на x86_64)

| Компонент | Время | Оптимизация |
|-----------|-------|-------------|
| STT (Vosk) | 0.5-0.7s | ✅ Offline |
| Dialogue (DeepSeek) | 1-2s | ✅ Streaming |
| TTS (Silero) | 0.3-0.5s | ✅ Local |
| **Total** | **1.8-3.2s** | 🚀 Real-time |

### На Raspberry Pi 5 (ожидаемо)

| Компонент | Время | Статус |
|-----------|-------|--------|
| STT (Vosk) | ~1s | ✅ OK |
| Dialogue | 1-2s | ✅ Network |
| TTS (Silero) | ~1s | ✅ 4 threads |
| **Total** | **3-4s** | ✅ Приемлемо |

### CPU/Memory

```
Vosk Model Load: ~500 MB RAM
Recognition: ~10-20% CPU (1 core)
```

## Troubleshooting

### Проблема: Vosk не распознаёт

**Причины**:
1. Модель не загружена → Проверить `/models/vosk-model-small-ru-0.22`
2. Неправильный sample_rate → Должен быть 16000 Hz
3. Плохое качество аудио → Проверить ReSpeaker

**Диагностика**:
```bash
# Проверить модель
ls -lh /models/vosk-model-small-ru-0.22

# Проверить sample rate
ros2 param get /stt_node sample_rate

# Записать тестовый WAV
ros2 topic echo /audio/audio > /tmp/test.raw
```

### Проблема: Ложные срабатывания

**Решение**: Увеличить `min_speech_duration`:
```yaml
stt_node:
  min_speech_duration: 0.8  # было 0.5
```

### Проблема: Медленное распознавание

**Причины**:
1. Большая модель → Использовать small модель
2. CPU перегружен → Проверить `top`
3. Swap активен → Увеличить RAM

**Оптимизация**:
- Использовать `vosk-model-small-ru-0.22` (45 MB)
- Закрыть лишние процессы
- Отключить SWAP на Pi 5

## Docker Integration

### Dockerfile Updates

**TODO**: Добавить в `docker/vision/Dockerfile`:
```dockerfile
# Vosk STT
RUN pip3 install vosk==0.3.45

# Download Vosk model
RUN mkdir -p /models && \
    cd /models && \
    wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip && \
    unzip vosk-model-small-ru-0.22.zip && \
    rm vosk-model-small-ru-0.22.zip
```

### Docker Compose

**Проверить** в `docker/vision/docker-compose.yaml`:
```yaml
voice-assistant:
  volumes:
    - /models:/models:ro  # Mount Vosk models
```

## Next Steps: Phase 4

**Sound Node** - звуковые эффекты:
- Загрузка `.mp3` из `sound_pack/`
- Триггеры на события (thinking, surprise, angry)
- Синхронизация с анимациями

**TODO**:
1. Создать `sound_node.py`
2. Интегрировать с `sound_pack/` (10 файлов)
3. Добавить trigger в dialogue_node
4. Обновить LED паттерны

## References

- [Vosk Documentation](https://alphacephei.com/vosk/)
- [Vosk Python API](https://github.com/alphacep/vosk-api/tree/master/python)
- [Vosk Models](https://alphacephei.com/vosk/models)
- [Phase 3 Testing Guide](PHASE3_STT_TESTING.md)

---

**Status**: ✅ Phase 3 Complete  
**Next**: Phase 4 - Sound Effects Node
