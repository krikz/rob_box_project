# Hardware Test Scripts

Тестовые скрипты для проверки железа Rob Box: LED матрицы, ReSpeaker микрофона, USB устройств.

## Скрипты

### 💡 LED тесты
- **test_led_direct.py** - прямой тест LED драйвера (pi5neo)
  - Бегущий огонек по всем 253 LED
  - Требует: `pip install pi5neo`

### 🎤 ReSpeaker тесты
- **test_jsk_respeaker.py** - тест JSK ReSpeaker node
- **test_jsk_simple.py** - упрощенный тест JSK
- **test_channels_fix.py** - тест исправления каналов микрофона
- **test_another_key.py** - тест альтернативного API ключа

### 🔌 USB/Audio тесты
- **test_usb_vad.py** - тест USB Voice Activity Detection
- **test_device.py** - общий тест USB устройств

## Запуск

### LED тест
```bash
cd /home/ros2/rob_box_project
python3 local_test/hardware/test_led_direct.py
```
⚠️ Требует `sudo` для доступа к `/dev/spidev0.0`

### ReSpeaker тест
```bash
source install/setup.bash
python3 local_test/hardware/test_jsk_respeaker.py
```

### USB VAD тест
```bash
python3 local_test/hardware/test_usb_vad.py
```

## Требования

### Для LED:
- pi5neo: `pip install pi5neo`
- SPI device: `/dev/spidev0.0`
- 253 WS2812B LEDs на GPIO10 (SPI0 MOSI)

### Для ReSpeaker:
- JSK audio common: `ros-humble-jsk-audio-common`
- ReSpeaker 4-Mic Array (USB)
- ALSA drivers

### Для USB VAD:
- PyAudio: `pip install pyaudio`
- webrtcvad: `pip install webrtcvad`
