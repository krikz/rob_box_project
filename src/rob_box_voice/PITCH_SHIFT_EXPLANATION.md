# Эффект "Бурундука" в ROBBOX Voice - Техническое Объяснение

## Проблема: Голос звучит выше и быстрее на WM8960

### Что происходит

В вашем рабочем скрипте голос ROBBOX получается с характерным "эффектом бурундука" - звучит выше и быстрее чем в оригинале от Yandex TTS.

### Техническая причина

```python
def play_audio_segment(audio_data):
    # 1. Получаем частоту звуковой карты WM8960
    device_info = sd.query_devices(None, "output")
    target_rate = int(device_info["default_samplerate"])  # = 44100 Hz
    
    # 2. Читаем СЫРЫЕ байты WAV БЕЗ заголовка
    samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
    
    # 3. Воспроизводим на частоте устройства
    sd.play(samples, samplerate=target_rate, blocking=True)  # ⚠️ ИСКАЖЕНИЕ!
```

**Проблема:**
- Yandex TTS возвращает WAV с частотой **22050 Hz** (оригинальная частота антона)
- `np.frombuffer()` читает ТОЛЬКО PCM данные, **игнорируя заголовок WAV** с информацией о частоте
- `sd.play(samples, samplerate=44100)` воспроизводит на частоте устройства WM8960

**Результат:**
```
Pitch shift = 44100 / 22050 = 2.0x
Скорость = 2.0x быстрее
Тон = на октаву выше
```

### Почему это работает на Raspberry Pi с WM8960

На ROBOT001 звуковая карта WM8960:
```bash
ros2@ROBOT001:~$ aplay -l
card 0: wm8960soundcard [wm8960-soundcard], device 0: 3f203000.i2s-wm8960-hifi wm8960-hifi-0
```

Default samplerate = **44100 Hz** → ровно в 2 раза выше чем 22050 Hz от TTS.

### Правильное решение (без искажений)

```python
import soundfile as sf

def play_audio_segment_correct(audio_data):
    # 1. Читаем WAV С заголовком - получаем правильную частоту
    import io
    data, original_rate = sf.read(io.BytesIO(audio_data))  # original_rate = 22050
    
    # 2. Получаем частоту устройства
    device_info = sd.query_devices(None, "output")
    target_rate = int(device_info["default_samplerate"])  # = 44100
    
    # 3. Ресемплируем если нужно
    if original_rate != target_rate:
        import librosa
        data = librosa.resample(data, orig_sr=original_rate, target_sr=target_rate)
    
    # 4. Воспроизводим на правильной частоте
    sd.play(data, samplerate=target_rate, blocking=True)  # ✅ БЕЗ искажений
```

### Для датасета TTS training

**ВАЖНО:** Сохраняем файлы **БЕЗ ресемплинга** (оригинал 22050 Hz):

```python
def save_audio(self, audio_data: bytes, output_path: Path):
    # Сохраняем WAV как есть от Yandex (22050 Hz, PCM_16, mono)
    with open(output_path, 'wb') as f:
        f.write(audio_data)  # ✅ ОРИГИНАЛЬНОЕ качество
```

### Comparison

| Метод | Частота | Pitch | Звучание | Использование |
|-------|---------|-------|----------|---------------|
| `np.frombuffer()` + 44100 Hz | 22050→44100 | 2.0x выше | "Бурундук" | Ваш ROBBOX скрипт |
| `soundfile.read()` + resample | 22050→44100 | Нормальный | Как от Yandex | Тестирование |
| Сохранение оригинала | 22050 | Нормальный | Как от Yandex | TTS training |

### Вывод

Эффект "бурундука" - это **не баг, а особенность** вашей реализации, которая создаёт уникальный голос ROBBOX! 

Для обучения новой модели используем **оригинальные файлы 22050 Hz**, а воспроизведение на роботе будет с тем же искажением через `np.frombuffer()`.

## Frequency Analysis

```
Original Yandex TTS: 22050 Hz (стандарт для речи)
WM8960 default rate: 44100 Hz (аудио стандарт CD quality)

Ratio: 44100 / 22050 = 2.0

Effect:
- Playback speed: 2x faster  
- Pitch: +12 semitones (1 octave)
- Duration: 0.5x shorter
```

## Example

Фраза "Привет! Я робот" (9.53 сек при 0.4 speed):
- Оригинал 22050 Hz: нормальный голос антона
- Воспроизведение 44100 Hz: ~4.77 сек, голос выше, звучит как "детский робот"

Это и есть фирменный звук ROBBOX! 🤖
