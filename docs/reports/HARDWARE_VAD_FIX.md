# Hardware VAD Fix Report
## Исправление аппаратного VAD с ReSpeaker Mic Array v2.0

**Дата:** 15 октября 2025  
**Автор:** AI Assistant (GitHub Copilot)  
**Статус:** ✅ РЕШЕНО

---

## 📋 Проблема

### Симптомы
```
[audio_node-1] Ошибка чтения параметра VOICEACTIVITY: [Errno 32] Pipe error
[audio_node-1] Ошибка чтения параметра VOICEACTIVITY: [Errno 32] Pipe error
...
```

- USB VAD (Voice Activity Detection) не работал
- Постоянные **Pipe errors** при попытке чтения параметра `VOICEACTIVITY`
- PyAudio streaming работал, но USB control transfer не мог читать данные
- Попытки с `dev.reset()` вызывали Segmentation fault

### Первоначальная гипотеза
- PyAudio блокирует USB устройство
- Невозможна одновременная работа USB VAD + PyAudio audio streaming
- Рассматривался переход на программный VAD (WebRTC, Silero)

---

## 🔍 Исследование

### 1. Изучение jsk-ros-pkg
**Репозиторий:** https://github.com/jsk-ros-pkg/jsk_3rdparty

Обнаружено что у них **работает** одновременная работа USB VAD + PyAudio:

```python
# respeaker_ros/src/respeaker_ros/__init__.py
def read(self, name):
    data = PARAMETERS[name]  # (id, offset, type, ...)
    id = data[0]
    cmd = 0x80 | data[1]  # ← КРИТИЧНО!
    if data[2] == 'int':
        cmd |= 0x40
    
    response = self.dev.ctrl_transfer(
        CTRL_IN | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE,
        0, cmd, id, 8, TIMEOUT)  # ← cmd и id в РАЗНЫХ местах!
```

### 2. Сравнение с нашим кодом

**Наш неправильный код:**
```python
data = self.dev.ctrl_transfer(
    CTRL_IN | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE,
    0,          # request = 0 ✓
    param_id,   # value = 19 ✗ НЕПРАВИЛЬНО!
    0x1C,       # index = 28 ✗ НЕПРАВИЛЬНО!
    64,         # length
    timeout=1000)
```

**Правильный формат (jsk):**
```python
cmd = 0x80 | 32 | 0x40  # 0x80 | offset | 0x40 для int
response = self.dev.ctrl_transfer(
    CTRL_IN | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE,
    0,        # request = 0
    cmd,      # value = 0xE0 (0x80 | 32 | 0x40)
    19,       # index = param_id
    8,        # length
    timeout=1000)
```

### 3. Тестирование гипотезы

Создан **test_jsk_simple.py** для проверки:
- Channels = 6 (RAW mode вместо processed channels=1)
- БЕЗ dev.reset() (jsk тоже не использует!)
- Правильный формат USB команд

**Результат:**
```
USB VAD ДО stream:      10/10  успешных
USB VAD ВО ВРЕМЯ stream: 20/20  успешных, 0 ошибок  ← 100%!
USB VAD ПОСЛЕ stream:    10/10  успешных

✅ ВЫВОД: USB VAD РАБОТАЕТ во время PyAudio stream!
   Процент успеха: 100%
```

---

## ✅ Решение

### 1. Исправлен формат USB команд

**respeaker_interface.py:**
```python
PARAMETERS = {
    'VOICEACTIVITY': (19, 32, 'int'),  # (param_id, offset, type)
    'DOAANGLE': (21, 0, 'int'),
}

def read_parameter(self, param_name: str) -> Optional[int]:
    param_data = self.PARAMETERS.get(param_name)
    param_id = param_data[0]    # 19
    offset = param_data[1]      # 32
    param_type = param_data[2]  # 'int'
    
    # Формируем cmd как в jsk-ros-pkg
    cmd = 0x80 | offset  # 0xA0
    if param_type == 'int':
        cmd |= 0x40      # 0xE0
    
    response = self.dev.ctrl_transfer(
        CTRL_IN | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE,
        0,        # request
        cmd,      # value (0xE0)
        param_id, # index (19)
        8,        # length
        timeout=1000)
    
    result = struct.unpack(b'ii', response.tobytes())
    return result[0]
```

### 2. Убран dev.reset()

**Проблема:** `dev.reset()` переподключает USB устройство, PyAudio теряет доступ к audio interface.

**Решение:** Убрали reset, добавили только sleep(5) для стабилизации.

```python
def connect(self) -> bool:
    self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    # НЕ делаем dev.reset()!
    
    # Sleep для стабилизации
    time.sleep(5)
    
    return True
```

### 3. Добавлена поддержка pixel_ring

**LED индикация статуса:**
```python
from pixel_ring import usb_pixel_ring_v2

# Инициализация
self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
self.pixel_ring.think()     # "Думающая" анимация
time.sleep(5)
self.pixel_ring.trace()     # DoA tracking mode
```

**Зависимости:**
```
pixel-ring>=0.1.0
spidev>=3.5
```

### 4. Исправлена синхронизация TTS/STT

**Проблема:** STT слышит себя → самовозбуждение

**Решение:**
- tts_node публикует `/voice/tts/state` (synthesizing/playing/ready)
- stt_node подписывается и отключает распознавание пока робот говорит

```python
# stt_node.py
def tts_state_callback(self, msg: String):
    if msg.data in ['synthesizing', 'playing']:
        self.is_robot_speaking = True
    elif msg.data in ['ready', 'idle']:
        self.is_robot_speaking = False

def speech_audio_callback(self, msg: AudioData):
    if self.is_robot_speaking:
        return  # Не слушаем пока говорим
    # ... распознавание
```

### 5. Multichannel audio processing

**audio_node.py:**
```python
# Channels = 6 (RAW mode)
if self.channels > 1:
    audio_data = np.frombuffer(in_data, dtype=np.int16)
    audio_data = audio_data.reshape(-1, self.channels)
    mono_data = audio_data.mean(axis=1).astype(np.int16)
    audio_bytes = mono_data.tobytes()
```

---

## 📊 Результаты

### До исправления
- ❌ Pipe errors каждые 100ms
- ❌ VAD не работает
- ❌ Рассматривался отказ от hardware VAD

### После исправления
- ✅ 0 Pipe errors
- ✅ VAD работает стабильно
- ✅ Pixel ring показывает статус
- ✅ Распознавание речи функционирует
- ✅ Нет самовозбуждения (TTS не мешает STT)

### Лог работающей системы
```
[audio_node-1] ✓ ReSpeaker USB подключен для VAD/DoA
[audio_node-1] ✓ Pixel ring инициализирован
[audio_node-1] Ожидание стабилизации USB (5 сек)...
[audio_node-1] ✓ Pixel ring в режиме трассировки
[audio_node-1] ✓ Аудио поток открыт: 16000Hz, 6ch
[audio_node-1] ▶ Захват аудио запущен
[audio_node-1] 🎙️  VAD: речь
[audio_node-1] 🗣️  Начало речи
[audio_node-1] 🎙️  VAD: тишина
[audio_node-1] ✅ Речь распознана: 2.23с
[stt_node-2] ✅ ПРИНЯТО: рома сколько сейчас время
```

---

## 📁 Изменённые файлы

### Критичные
1. `src/rob_box_voice/rob_box_voice/utils/respeaker_interface.py`
   - Формат PARAMETERS: `(param_id, offset, type)`
   - Правильный read_parameter()
   - pixel_ring поддержка
   - Убран dev.reset()

2. `src/rob_box_voice/rob_box_voice/audio_node.py`
   - ignore_stderr для ALSA
   - Channels=6 RAW mode
   - Multichannel → mono conversion
   - Speech prefetch буферы

3. `src/rob_box_voice/rob_box_voice/stt_node.py`
   - Слушает /audio/speech_audio
   - Подписка на /voice/tts/state
   - Фильтр коротких слов

4. `src/rob_box_voice/rob_box_voice/tts_node.py`
   - Публикует /voice/tts/state
   - Блокирующее воспроизведение
   - Тишина 200ms против white noise

### Зависимости
5. `src/rob_box_voice/requirements.txt`
   - pixel-ring>=0.1.0
   - spidev>=3.5
   - pydub>=0.25.1

6. `src/rob_box_voice/setup.py`
   - Добавлен scripts module

---

## 🔗 Ссылки

### Референсные репозитории
- jsk-ros-pkg: https://github.com/jsk-ros-pkg/jsk_3rdparty
  * respeaker_ros/__init__.py - USB протокол
  * respeaker_node.py - архитектура

### Тестовые скрипты (удалены после использования)
- `test_jsk_simple.py` - proof of concept (100% success)
- `test_usb_vad.py` - изолированный USB test
- `test_vosk_simple.py` - Vosk STT test

### Документация
- ReSpeaker 4 Mic Array: https://wiki.seeedstudio.com/ReSpeaker_4_Mic_Array_for_Raspberry_Pi/
- Tuning API: https://github.com/respeaker/usb_4_mic_array

---

## 💡 Ключевые выводы

### Что работает
1. **Одновременная работа USB VAD + PyAudio** - возможна с правильным форматом команд
2. **Channels=6 (RAW mode)** - критично для совместимости
3. **БЕЗ dev.reset()** - reset убивает audio interface
4. **pixel_ring** - полезен для визуальной индикации
5. **TTS/STT синхронизация** - необходима для предотвращения самовозбуждения

### Что НЕ работает
1. ❌ `dev.reset()` - ломает PyAudio
2. ❌ `channels=1` (processed mode) - USB конфликты
3. ❌ Неправильный формат USB команд - Pipe errors
4. ❌ Одновременное TTS + STT - самовозбуждение

### Уроки
- **Изучать рабочие реализации** - jsk-ros-pkg был ключом к решению
- **Создавать изолированные тесты** - test_jsk_simple.py доказал работоспособность
- **Не доверять документации** - официальный tuning.py не показывал полную картину
- **USB протокол имеет значение** - порядок параметров критичен

---

## 📝 Дальнейшие улучшения

### Краткосрочные (TODO)
- [ ] Интеграция Yandex SpeechKit (основной STT/TTS)
- [ ] DoA (Direction of Arrival) индикация на pixel_ring
- [ ] Адаптивный VAD threshold
- [ ] Noise gating для улучшения качества

### Долгосрочные
- [ ] Beam forming для улучшения SNR
- [ ] Echo cancellation
- [ ] Multi-language support
- [ ] Wake word detection

---

**Git Commit:** `830da4c`  
**Branch:** `feature/voice-assistant`  
**Status:** ✅ Ready for merge
