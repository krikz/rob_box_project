# Audio-Reactive LED Animations

Система синхронизации LED анимаций с аудио выходом робота.

## Обзор

Аудио-реактивная система позволяет LED матрицам (особенно "рту") реагировать на звук в реальном времени. Это создаёт эффект "говорящего робота", где рот открывается и закрывается синхронно с речью или музыкой.

## Возможности

- **Реактивность в реальном времени**: Анимация синхронизируется с аудио выходом
- **Настраиваемые пороги**: Контроль чувствительности к громкости
- **Сглаживание**: Плавные переходы между кадрами
- **ROS2 интеграция**: Топики для мониторинга и управления

## Архитектура

```
┌─────────────────┐
│ System Audio    │
│ Output          │
└────────┬────────┘
         │
         ↓
┌─────────────────────────────┐
│ PyAudio Loopback/Stereo Mix │
│ (monitors audio output)     │
└────────┬────────────────────┘
         │
         ↓
┌──────────────────────────────────┐
│ audio_reactive_animation_node.py │
│ - Calculate RMS volume           │
│ - Apply smoothing                │
│ - Publish /audio/level           │
└────────┬─────────────────────────┘
         │
         ↓
┌─────────────────────────┐
│ animation_player_node   │
│ - Select frame based    │
│   on audio level        │
│ - Update LED display    │
└─────────────────────────┘
```

## Настройка

### 1. Установка зависимостей

```bash
# Python audio library
pip3 install pyaudio

# На Linux может потребоваться:
sudo apt-get install portaudio19-dev python3-pyaudio
```

### 2. Включение аудио мониторинга

#### Linux (PulseAudio)
```bash
# Создать loopback модуль для мониторинга аудио выхода
pactl load-module module-loopback latency_msec=1

# Или добавить в /etc/pulse/default.pa:
# load-module module-loopback latency_msec=1
```

#### Windows
1. Откройте "Звук" в Панели управления
2. Вкладка "Запись"
3. Включите "Стерео микшер" (Stereo Mix)
4. Установите как устройство по умолчанию

### 3. Запуск ноды

```bash
# Запустить аудио-реактивную ноду
ros2 run rob_box_animations audio_reactive_animation_node

# В другом терминале запустить плеер анимаций
ros2 run rob_box_animations animation_player_node

# Включить аудио-реактивную анимацию
ros2 topic pub --once /audio/enable_reactive std_msgs/String "data: 'talking'"
```

## Использование

### Включение аудио-реактивности

```bash
# Активировать анимацию "talking" с аудио синхронизацией
ros2 topic pub --once /audio/enable_reactive std_msgs/String "data: 'talking'"

# Воспроизвести звук
ros2 run audio_common audio_player_node sound_file.mp3

# Рот робота будет двигаться синхронно со звуком!
```

### Отключение

```bash
ros2 topic pub --once /audio/enable_reactive std_msgs/String "data: 'stop'"
```

### Мониторинг уровня звука

```bash
# Подписаться на топик уровня звука
ros2 topic echo /audio/level

# Визуализация (требует rqt)
rqt_plot /audio/level
```

## Создание аудио-реактивных анимаций

### Формат манифеста

```yaml
name: "talking"
audio_reactive: true
audio_source: "system_audio_output"
audio_threshold: 0.3  # Минимальная громкость для активации
audio_smoothing: 0.2  # Коэффициент сглаживания (0-1)

panels:
  - logical_group: "main_display"
    audio_controlled: true
    frames:
      - image: "frames/mouth_closed.png"
        duration_ms: 100
        audio_range: [0.0, 0.1]  # Показать при громкости 0-10%
      
      - image: "frames/mouth_open.png"
        duration_ms: 100
        audio_range: [0.1, 0.5]  # Показать при громкости 10-50%
      
      - image: "frames/mouth_wide.png"
        duration_ms: 100
        audio_range: [0.5, 1.0]  # Показать при громкости 50-100%
```

### Параметры

- **audio_reactive**: `true` - включает аудио-реактивность
- **audio_source**: Источник звука (`system_audio_output`)
- **audio_threshold**: Порог громкости (0.0-1.0)
- **audio_smoothing**: Сглаживание (0.0 = нет, 1.0 = максимальное)
- **audio_range**: `[min, max]` - диапазон громкости для кадра

## ROS2 API

### Топики

#### Publishers
- `/audio/level` (`std_msgs/Float32`) - Текущий уровень громкости (0.0-1.0)
- `/animation/audio_trigger` (`std_msgs/String`) - Триггер при превышении порога

#### Subscribers
- `/audio/enable_reactive` (`std_msgs/String`) - Включить аудио-реактивную анимацию

### Параметры

- `animations_dir` - Путь к директории анимаций
- `audio_device_index` - Индекс аудио устройства (-1 = по умолчанию)
- `sample_rate` - Частота дискретизации (44100 Hz)
- `chunk_size` - Размер буфера (1024 samples)

## Примеры анимаций

### 1. Talking (Говорение)
Рот робота в стиле Бендера, открывается синхронно с речью.

```bash
ros2 topic pub --once /audio/enable_reactive std_msgs/String "data: 'talking'"
espeak "Hello, I am a robot!" --stdout | aplay
```

### 2. Music Visualizer (будущее)
Создайте анимацию с разными цветами/паттернами в зависимости от частоты звука.

## Troubleshooting

### Проблема: "PyAudio not available"
```bash
pip3 install pyaudio
# Или на Linux:
sudo apt-get install python3-pyaudio
```

### Проблема: "No audio input detected"
- Убедитесь что Stereo Mix / Loopback включен
- Проверьте список устройств: `python3 -c "import pyaudio; p=pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)}') for i in range(p.get_device_count())]"`

### Проблема: "Mouth animation lags"
- Уменьшите `audio_smoothing` (меньше задержка, но больше дрожания)
- Уменьшите `chunk_size` (меньше латентность)
- Увеличьте приоритет ROS2 ноды

## Интеграция с TTS

```python
# Пример с gTTS (Google Text-to-Speech)
from gtts import gTTS
import os

# Создать речь
tts = gTTS(text='Hello, I am rob_box robot!', lang='en')
tts.save('speech.mp3')

# Воспроизвести (анимация будет синхронизирована автоматически)
os.system('mpg123 speech.mp3')
```

## Будущие улучшения

- [ ] Поддержка FFT для частотного анализа
- [ ] Визуализатор спектра на LED матрицах
- [ ] Предиктивная синхронизация (анализ TTS перед воспроизведением)
- [ ] Поддержка MIDI входа
- [ ] Эквалайзер LED (низкие частоты = нижние LED, высокие = верхние)

## Лицензия

MIT License - rob_box project
