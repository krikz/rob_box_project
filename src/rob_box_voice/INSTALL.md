# Установка и тестирование rob_box_voice

## Быстрый старт

### 1. Установка зависимостей

```bash
cd ~/rob_box_project/src/rob_box_voice

# Python зависимости
pip install -r requirements.txt

# Драйверы ReSpeaker
cd ~/
git clone https://github.com/respeaker/usb_4_mic_array.git
cd usb_4_mic_array
sudo pip install -r requirements.txt

# pixel_ring для LED
cd ~/
git clone https://github.com/respeaker/pixel_ring.git
cd pixel_ring
sudo python setup.py install
```

### 2. Настройка udev правил

```bash
cd ~/rob_box_project/src/rob_box_voice

# Создать udev правило
sudo tee /etc/udev/rules.d/60-respeaker.rules << EOF
# ReSpeaker Mic Array v2.0
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666", GROUP="plugdev"
EOF

# Перезагрузить udev
sudo udevadm control --reload-rules
sudo udevadm trigger

# Переподключить ReSpeaker или перезагрузить
```

### 3. Проверка подключения

```bash
# Проверить USB устройство
lsusb | grep 2886:0018
# Должно вывести: Bus XXX Device XXX: ID 2886:0018 Seeed Technology Inc.

# Проверить аудио устройство
arecord -l | grep ReSpeaker
# Должно вывести: card X: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)]
```

### 4. Загрузка прошивки (опционально)

Если нужно обновить прошивку на 1-channel версию:

```bash
cd ~/usb_4_mic_array

# Загрузить 1-channel прошивку (рекомендуется для AEC)
sudo python dfu.py --download 1_channel_firmware.bin

# Или 6-channels для отладки
# sudo python dfu.py --download 6_channels_firmware.bin
```

### 5. Тестирование без ROS2

```bash
cd ~/rob_box_project/src/rob_box_voice

# Запустить тестовый скрипт
chmod +x scripts/test_respeaker.py
python3 scripts/test_respeaker.py
```

Тест должен пройти все 4 проверки:
- ✓ USB подключение
- ✓ Аудио устройство
- ✓ VAD и DoA (говорите в микрофон)
- ✓ LED индикация (моргание разными цветами)

### 6. Сборка ROS2 пакета

```bash
cd ~/rob_box_project

# Сборка только rob_box_voice
colcon build --packages-select rob_box_voice

# Или полная сборка
colcon build

# Source
source install/setup.bash
```

### 7. Запуск ROS2 нод

#### Вариант A: Отдельные ноды

```bash
# Терминал 1: AudioNode
ros2 run rob_box_voice audio_node

# Терминал 2: LEDNode
ros2 run rob_box_voice led_node
```

#### Вариант B: Launch file

```bash
ros2 launch rob_box_voice basic_test.launch.py
```

### 8. Проверка топиков

```bash
# Список топиков
ros2 topic list

# Должны быть:
# /audio/audio
# /audio/vad
# /audio/direction
# /audio/speech_detected
# /audio/state

# Эхо аудио данных (проверка потока)
ros2 topic echo /audio/audio --once

# VAD (говорите в микрофон)
ros2 topic echo /audio/vad

# Направление на источник звука
ros2 topic echo /audio/direction
```

### 9. Тест LED управления

```bash
# Публиковать состояние вручную
ros2 topic pub --once /voice/state std_msgs/msg/String "data: 'listening'"
# LED должны загореться зелёным

ros2 topic pub --once /voice/state std_msgs/msg/String "data: 'thinking'"
# LED должны начать пульсировать синим

ros2 topic pub --once /voice/state std_msgs/msg/String "data: 'speaking'"
# LED должны вращаться голубым

ros2 topic pub --once /voice/state std_msgs/msg/String "data: 'idle'"
# LED должны выключиться
```

## Troubleshooting

### Проблема: "ReSpeaker device not found"

**Решение:**
1. Проверить USB подключение: `lsusb | grep 2886`
2. Проверить udev правила: `ls -l /etc/udev/rules.d/60-respeaker.rules`
3. Переподключить устройство или перезагрузить

### Проблема: "Permission denied" при доступе к USB

**Решение:**
```bash
# Добавить пользователя в группу plugdev
sudo usermod -a -G plugdev $USER

# Перелогиниться или выполнить
newgrp plugdev
```

### Проблема: Аудио устройство не найдено в PyAudio

**Решение:**
```bash
# Проверить доступные устройства
python3 -c "import pyaudio; p=pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)}') for i in range(p.get_device_count())]"

# Найти index с "ReSpeaker" в имени
```

### Проблема: LED не работают

**Решение:**
1. Проверить что используется libusb-1.0 (не 0.1)
2. Проверить python-usb: `pip show pyusb`
3. Попробовать с sudo: `sudo python3 scripts/test_respeaker.py`

### Проблема: Эхо (слышит сам себя)

**Решение:**
1. Убедиться что загружена 1-channel прошивка
2. Проверить AEC параметры:
```bash
cd ~/usb_4_mic_array
python tuning.py AECFREEZEONOFF 0  # Включить адаптацию
python tuning.py ECHOONOFF 1       # Включить подавление эха
```

### Проблема: Низкое качество звука

**Решение:**
```bash
cd ~/usb_4_mic_array

# Включить AGC
python tuning.py AGCONOFF 1
python tuning.py AGCMAXGAIN 30

# Включить подавление шума
python tuning.py STATNOISEONOFF 1
python tuning.py NONSTATNOISEONOFF 1
```

## Следующие шаги

После успешного тестирования AudioNode и LEDNode:

1. ✅ Реализовать **STTNode** (Speech-to-Text)
2. ✅ Реализовать **TTSNode** (Text-to-Speech)
3. ✅ Реализовать **DialogueNode** (State machine + LLM)
4. ✅ Реализовать **SoundNode** (воспроизведение)
5. ✅ Реализовать **CommandNode** (управление роботом)

См. `docs/development/VOICE_ASSISTANT_ARCHITECTURE.md` для деталей.
