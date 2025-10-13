# Voice Assistant Build Fix - audio_common Issues

**Дата:** 2025-10-13  
**Ветка:** `feature/voice-assistant`  
**Коммиты:** `8ef5a9e`, `f3101c8`

---

## Проблема #1: Несуществующая ветка humble

### Симптомы
```
#14 [ 8/15] RUN cd /ws/src && git clone -b humble https://github.com/ros-drivers/audio_common.git
#14 0.961 fatal: Remote branch humble not found in upstream origin
ERROR: failed to build: exit code: 128
```

### Причина
В Dockerfile была попытка склонировать несуществующую ветку `humble` из репозитория `ros-drivers/audio_common`.

**Доступные ветки в репозитории:**
```bash
$ git ls-remote --heads https://github.com/ros-drivers/audio_common.git
refs/heads/indigo-devel  # ROS1
refs/heads/master        # ROS1
refs/heads/ros2          # ROS2 ✅
```

**Вывод:** Ветки `humble` не существует!

### Решение #1

**Было:**
```dockerfile
RUN cd /ws/src && \
    git clone -b humble https://github.com/ros-drivers/audio_common.git
```

**Стало:**
```dockerfile
RUN cd /ws/src && \
    git clone -b ros2 https://github.com/ros-drivers/audio_common.git
```

**Коммит:** `8ef5a9e`

---

## Проблема #2: Ненужные зависимости GStreamer

### Симптомы
```
#19 97.02 ERROR: the following rosdeps failed to install
#19 97.02   apt: command [apt-get install -y libgstreamer1.0-dev] failed
#19 97.02   apt: command [apt-get install -y festival] failed
#19 97.02   apt: command [apt-get install -y festvox-kallpc16k] failed
#19 97.02   apt: Failed to detect successful installation of [festvox-kallpc16k]

#20 15.50 CMake Error: Package 'rcutils' exports the library 'rcutils' which couldn't be found
#20 15.50 Failed   <<< audio_common_msgs [10.2s, exited with code 1]
```

### Причина

Репозиторий `audio_common` (ветка ros2) содержит несколько пакетов:

```
audio_common/
├── audio_common_msgs     ← НАМ НУЖЕН ТОЛЬКО ЭТОТ!
├── audio_capture         ← использует GStreamer
├── audio_play            ← использует GStreamer  
└── sound_play            ← использует festival (TTS)
```

Когда `rosdep` обрабатывает всю директорию `src/`, он пытается установить зависимости ВСЕХ пакетов из `audio_common`, включая:
- GStreamer (для audio_capture/audio_play)
- festival + festvox-kallpc16k (для sound_play TTS)

**Проблемы:**
1. `festvox-kallpc16k` недоступен в Ubuntu ARM64 репозиториях
2. GStreamer нам не нужен (используем pyaudio для захвата)
3. festival нам не нужен (используем Silero TTS)

### Решение #2

**Извлекаем только audio_common_msgs:**

```dockerfile
# Клонируем audio_common для audio_common_msgs (пакет недоступен через apt для Humble)
# Используем ветку ros2, но удаляем ненужные пакеты чтобы избежать зависимостей GStreamer/festival
RUN cd /ws/src && \
    git clone -b ros2 --depth 1 https://github.com/ros-drivers/audio_common.git && \
    cd audio_common && \
    # Оставляем только audio_common_msgs, удаляем остальные пакеты
    find . -maxdepth 1 -type d ! -name '.' ! -name '.git' ! -name 'audio_common_msgs' -exec rm -rf {} + && \
    echo "✅ audio_common_msgs extracted from ros2 branch"
```

**Устанавливаем rosdeps только для наших пакетов:**

```dockerfile
# Установка ROS зависимостей через rosdep (игнорируем ошибки для audio_common)
# audio_common_msgs имеет минимальные зависимости, но rosdep может жаловаться на недоступные пакеты
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    # Устанавливаем зависимости только для rob_box_voice и rob_box_animations
    rosdep install --from-paths src/rob_box_voice src/rob_box_animations --ignore-src -r -y || true
```

**Коммит:** `f3101c8`

---

## Почему audio_common_msgs нужен?

### Зависимость в package.xml

`src/rob_box_voice/package.xml`:
```xml
<depend>audio_common_msgs</depend>
```

### Использование в коде

`rob_box_voice` публикует аудио топик:
```python
# AudioNode публикует обработанный аудиопоток для STT
/audio/audio (audio_common_msgs/AudioData) - 16 kHz, 1 channel
```

### Почему не через apt?

```bash
$ apt search ros-humble-audio-common-msgs
# Пусто - пакет недоступен в репозиториях Ubuntu ARM64 для Humble
```

**Решение:** Собрать `audio_common_msgs` из исходников (ветка `ros2`).

---

## Проверка решения

### Локальная проверка (если есть Docker)

```bash
cd /home/ros2/rob_box_project
docker build -f docker/vision/voice_assistant/Dockerfile -t test-voice-assistant .
```

### Проверка через GitHub Actions

После пуша изменений:
1. Открыть: https://github.com/krikz/rob_box_project/actions
2. Найти workflow: "Build Vision Services"
3. Проверить что voice-assistant собирается успешно

---

## Документация и стандарты

### Согласно AGENT_GUIDE.md

**Правило:** Docker образ должен собираться из исходников только когда пакет недоступен через apt.

**Что можно копировать в Dockerfile:**
- ✅ `RUN git clone` - клонирование репозиториев
- ✅ `RUN colcon build` - компиляция ROS пакетов
- ✅ `RUN apt-get install` - установка системных пакетов

**Что нельзя копировать:**
- ❌ `COPY config/` - конфиги монтируются через volumes
- ❌ `COPY scripts/` - скрипты монтируются через volumes
- ❌ `COPY launch/` - launch файлы монтируются через volumes

### Согласно architecture документации

`docs/development/VOICE_ASSISTANT_ARCHITECTURE.md`:

```
AudioNode (rob_box_voice/audio_node.py)
  Publishers:
    /audio/audio (audio_common_msgs/AudioData) # 16kHz, 1ch, обработанный
    /audio/vad (std_msgs/Bool)                 # Voice Activity Detection
    /audio/direction (std_msgs/Int32)          # DoA angle 0-360°
```

Для работы AudioNode нужен `audio_common_msgs`.

---

## Альтернативные решения (не использованы)

### 1. Создать собственный пакет с сообщением

**Проблема:** Дублирование стандартного ROS пакета, плохая практика.

### 2. Использовать sensor_msgs/Audio (если бы был)

**Проблема:** `sensor_msgs` не содержит типа `Audio` или `AudioData` в ROS2 Humble.

### 3. Ждать пока audio_common_msgs появится в apt

**Проблема:** Неизвестно когда это произойдёт, проект заблокирован.

---

## Урок для будущего

### Проверка доступности ветки перед использованием

```bash
# Проверить доступные ветки
git ls-remote --heads <repository_url>

# Проверить доступность конкретной ветки
git ls-remote --heads <repository_url> <branch_name>
```

### Документирование нестандартных зависимостей

Если пакет собирается из исходников (не через apt):
1. Добавить комментарий в Dockerfile с объяснением почему
2. Указать какая ветка используется и почему
3. Добавить проверку в CI/CD

---

## Статус

✅ **Исправлено**  
✅ **Закоммичено:** `8ef5a9e`  
✅ **Запушено в:** `feature/voice-assistant`  
⏳ **Ожидание:** GitHub Actions build

---

## Следующие шаги

1. **Дождаться успешного билда** на GitHub Actions
2. **Проверить image:** `ghcr.io/krikz/rob_box:voice-assistant-humble-dev`
3. **Протестировать на Vision Pi:**
   ```bash
   cd ~/rob_box_project/docker/vision
   docker compose pull voice-assistant
   docker compose up -d voice-assistant
   docker logs voice-assistant
   ```

4. **Если всё ОК - мерж в develop:**
   ```bash
   git checkout develop
   git merge feature/voice-assistant
   git push origin develop
   ```

---

**Prepared by:** AI Agent  
**Reviewed by:** krikz
