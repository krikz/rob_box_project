# ROBBOX Оригинальный Голос - Реализация

## 📋 Обзор

Этот документ описывает реализацию оригинального голоса ROBBOX с поддержкой:
- Нормальной скорости воспроизведения (без эффекта "бурундука")
- SSML управления pitch и speed из dialogue_node
- Голосовых команд управления громкостью
- Опционального "эффекта бурундука" через параметры

**⚠️ ВАЖНОЕ ОБНОВЛЕНИЕ (Ноябрь 2024):**
После PR #20 была обнаружена проблема "обратного бурундука" (голос звучал слишком медленно и глубоко). 
Это было исправлено добавлением ресэмплинга аудио с оригинальной частоты (22050 Hz) на частоту ReSpeaker (16000 Hz).
См. [REVERSE_CHIPMUNK_FIX.md](REVERSE_CHIPMUNK_FIX.md) для деталей.

## 🎯 Проблема

### Оригинальный эффект "бурундука"

В оригинальном скрипте ROBBOX голос получался с характерным эффектом "бурундука":

```python
# Оригинальный код
def play_audio_segment(audio_data):
    device_info = sd.query_devices(None, "output")
    target_rate = int(device_info["default_samplerate"])  # = 44100 Hz
    
    # Читаем СЫРЫЕ байты WAV БЕЗ заголовка
    samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
    
    # Воспроизводим на частоте устройства
    sd.play(samples, samplerate=target_rate, blocking=True)  # ⚠️ ИСКАЖЕНИЕ!
```

**Причина искажения:**
- Yandex TTS возвращает WAV с частотой **22050 Hz**
- `np.frombuffer()` читает только PCM данные, **игнорируя заголовок WAV**
- `sd.play(samples, samplerate=44100)` воспроизводит на частоте устройства
- Результат: **Pitch shift = 44100 / 22050 = 2.0x** (голос в 2 раза выше и быстрее)

## ✅ Новая Реализация

### 1. Нормальная Скорость Воспроизведения

**Исправление sample rate (Ноябрь 2024):**

Добавлен ресэмплинг аудио перед воспроизведением для исправления "обратного бурундука":

```python
def resample_audio(audio: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    """Ресэмплинг аудио с оригинальной частоты на целевую"""
    if orig_sr == target_sr:
        return audio
    
    duration = len(audio) / orig_sr
    target_length = int(duration * target_sr)
    
    orig_indices = np.linspace(0, len(audio) - 1, len(audio))
    target_indices = np.linspace(0, len(audio) - 1, target_length)
    
    resampled = np.interp(target_indices, orig_indices, audio)
    return resampled

# В коде воспроизведения:
if sample_rate != target_rate:  # 22050 Hz → 16000 Hz
    audio_np = resample_audio(audio_np, sample_rate, target_rate)
```

**Параметры TTS Node изменены по умолчанию:**

```python
self.declare_parameter("chipmunk_mode", False)  # ❌ Эффект бурундука ВЫКЛЮЧЕН
self.declare_parameter("pitch_shift", 1.0)       # ✅ Нормальная скорость (1.0x)
```

**Код воспроизведения:**

```python
if self.chipmunk_mode and self.pitch_shift > 1.0:
    # Эффект ускорения (опционально)
    decimation_factor = int(self.pitch_shift)
    audio_processed = audio_np[::decimation_factor]
else:
    # ✅ Нормальное воспроизведение БЕЗ искажений
    audio_processed = audio_np
```

### 2. SSML Управление Pitch и Speed

#### Парсинг SSML атрибутов

Новый метод `_parse_ssml_attributes()` извлекает атрибуты из `<prosody>` тегов:

```python
def _parse_ssml_attributes(self, ssml: str) -> dict:
    """
    Извлекает атрибуты из SSML тегов (pitch, rate/speed)
    
    Примеры:
    - <prosody pitch="+10%" rate="1.2"> → {'pitch': 1.1, 'rate': 1.2}
    - <prosody pitch="high" rate="slow"> → {'pitch': 1.2, 'rate': 0.7}
    """
    # Поддерживаемые форматы:
    # - Проценты: pitch="+10%", rate="150%"
    # - Множители: pitch="1.2", rate="0.8"
    # - Ключевые слова: pitch="high"/"low", rate="fast"/"slow"
```

#### Применение к Yandex TTS

```python
def _synthesize_yandex(self, text: str, ssml_attributes: dict = None):
    # Скорость речи: берем из SSML или используем параметр ноды
    speech_rate = ssml_attributes.get('rate', self.yandex_speed)
    
    request = tts_pb2.UtteranceSynthesisRequest(
        text=text,
        hints=[
            tts_pb2.Hints(voice="anton"),    # Оригинальный голос ROBBOX
            tts_pb2.Hints(speed=speech_rate), # Динамическая скорость из SSML
        ],
        # ...
    )
```

**Примеры SSML:**

```xml
<!-- Быстрая речь -->
<speak><prosody rate="1.5">Я говорю быстро</prosody></speak>

<!-- Медленная речь -->
<speak><prosody rate="0.7">Я говорю медленно</prosody></speak>

<!-- Нормальная скорость (по умолчанию yandex_speed=1) -->
<speak>Обычная речь</speak>
```

### 3. Управление Громкостью

#### Голосовые Команды

Dialogue Node распознает команды:

| Команда | Intent | Изменение | Результат |
|---------|--------|-----------|-----------|
| "робот, громче" | `louder` | +3 dB | Увеличение громкости на ~40% |
| "робот, тише" | `quieter` | -3 dB | Уменьшение громкости на ~30% |
| "робот, говори громко" | `max` | +6 dB | Максимальная громкость (~2x) |
| "робот, нормальная громкость" | `normal` | -3 dB | Стандартная громкость (70%) |

#### Реализация

```python
def _detect_volume_intent(self, text: str):
    """Определить intent для команд управления громкостью"""
    volume_patterns = {
        'louder': [r'громче', r'громко', r'прибав\w* громкост'],
        'quieter': [r'тише', r'потише', r'убав\w* громкост'],
        'max': [r'говори громко', r'максимальн\w* громкост'],
        'normal': [r'нормальн\w* громкост', r'обычн\w* громкост'],
    }
    # ...

def _handle_volume_command(self, intent: str) -> str:
    """Изменение громкости через ROS2 параметры"""
    # Получаем текущий volume_db
    # Вычисляем новый volume_db
    # Устанавливаем параметры для tts_node и sound_node
```

#### Динамическое Обновление Параметров

TTS Node поддерживает изменение параметров на лету:

```python
def parameters_callback(self, params):
    for param in params:
        if param.name == 'volume_db':
            self.volume_db = param.value
            self.volume_gain = 10.0 ** (self.volume_db / 20.0)
            self.get_logger().info(f"🔊 Громкость изменена: {self.volume_db:.1f} dB")
```

Sound Node также обновляет громкость и перезагружает звуки:

```python
def parameters_callback(self, params):
    for param in params:
        if param.name == 'volume_db':
            self.volume_db = param.value
            self.load_sounds()  # Перезагрузка с новой громкостью
```

## 🔧 Конфигурация

### TTS Node Параметры

```yaml
tts_node:
  ros__parameters:
    # Yandex Cloud TTS
    yandex_voice: "anton"        # Оригинальный голос ROBBOX
    yandex_speed: 1.0            # Скорость по умолчанию (было 0.4 в оригинале)
    
    # Chipmunk mode (опционально)
    chipmunk_mode: false         # true = эффект бурундука
    pitch_shift: 1.0             # Множитель скорости (2.0 = 2x быстрее)
    
    # Громкость
    volume_db: -3.0              # -3 dB = 70% громкости
```

### Sound Node Параметры

```yaml
sound_node:
  ros__parameters:
    volume_db: -12.0             # -12 dB = 24% громкости (звуки тише голоса)
```

## 📊 Сравнение Режимов

| Параметр | Оригинал (эффект бурундука) | Новая реализация (нормальная) |
|----------|----------------------------|-------------------------------|
| `chipmunk_mode` | `true` | `false` |
| `pitch_shift` | `2.0` | `1.0` |
| `yandex_speed` | `0.4` | `1.0` |
| Частота воспроизведения | 44100 Hz | 16000 Hz (ReSpeaker) |
| Sample decimation | Каждый 2-й сэмпл | Без decimation |
| Результат | Голос выше и быстрее | Голос как от Yandex TTS |

## 🎛️ Использование

### Включение Эффекта Бурундука (Опционально)

Если нужен оригинальный эффект ROBBOX:

```bash
# Установить параметры через ROS2 CLI
ros2 param set /tts_node chipmunk_mode true
ros2 param set /tts_node pitch_shift 2.0
ros2 param set /tts_node yandex_speed 0.4
```

### Управление Громкостью Голосом

```
Пользователь: "Робот, громче"
ROBBOX: "Делаю громче"

Пользователь: "Робот, говори громко"
ROBBOX: "Максимальная громкость"

Пользователь: "Робот, тише"
ROBBOX: "Делаю тише"
```

### Управление Скоростью через SSML

Dialogue Node может генерировать SSML с атрибутами:

```json
{
  "ssml": "<speak><prosody rate=\"1.5\">Быстрая речь</prosody></speak>",
  "dialogue_id": "abc123..."
}
```

TTS Node автоматически применит `rate=1.5` к Yandex TTS.

## 🐛 Отладка

### Проверка Текущих Параметров

```bash
# TTS Node
ros2 param get /tts_node chipmunk_mode
ros2 param get /tts_node pitch_shift
ros2 param get /tts_node volume_db
ros2 param get /tts_node yandex_speed

# Sound Node
ros2 param get /sound_node volume_db
```

### Логи

TTS Node логирует:
- Применение SSML атрибутов: `🎵 SSML атрибуты: {'rate': 1.5}`
- Режим воспроизведения: `🎵 Нормальная скорость: 123456 samples`
- Изменение громкости: `🔊 Громкость изменена: -3.0 → 0.0 dB`

Dialogue Node логирует:
- Обнаружение volume команды: `🔊 Обнаружена volume команда: louder`
- Изменение громкости: `✅ Громкость изменена: -3.0 → 0.0 dB`

## 📝 TODO / Future Work

- [ ] Реализовать pitch control для Yandex TTS (пока не поддерживается API)
- [ ] Добавить pitch-shifting через librosa для локальной обработки
- [ ] Сохранить настройки громкости между перезапусками
- [ ] Добавить профили голоса (детский, взрослый, робот)
- [ ] UI для настройки параметров голоса

## 🔗 Связанные Документы

- [PITCH_SHIFT_EXPLANATION.md](../../src/rob_box_voice/PITCH_SHIFT_EXPLANATION.md) - Объяснение эффекта бурундука
- [TTS Node](../../src/rob_box_voice/rob_box_voice/tts_node.py) - Реализация TTS
- [Dialogue Node](../../src/rob_box_voice/rob_box_voice/dialogue_node.py) - Обработка команд
- [Sound Node](../../src/rob_box_voice/rob_box_voice/sound_node.py) - Звуковые эффекты

---

**Дата:** 2025-10-24  
**Автор:** AI Agent (GitHub Copilot)  
**Версия:** 1.0
