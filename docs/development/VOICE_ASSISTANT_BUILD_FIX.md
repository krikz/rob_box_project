# Voice Assistant Build Fix - audio_common Branch Issue

**Дата:** 2025-10-13  
**Ветка:** `feature/voice-assistant`  
**Коммит:** `8ef5a9e`

---

## Проблема

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

---

## Решение

### Исправление в Dockerfile

**Было:**
```dockerfile
RUN cd /ws/src && \
    git clone -b humble https://github.com/ros-drivers/audio_common.git && \
    echo "✅ audio_common cloned (audio_common_msgs included)"
```

**Стало:**
```dockerfile
# Клонируем audio_common для audio_common_msgs (пакет недоступен через apt для Humble)
# Используем ветку ros2 (ветки humble не существует в репозитории)
RUN cd /ws/src && \
    git clone -b ros2 https://github.com/ros-drivers/audio_common.git && \
    echo "✅ audio_common (ros2 branch) cloned - includes audio_common_msgs"
```

### Обновление команды сборки

**Было:**
```dockerfile
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_voice rob_box_animations \
    ...
```

**Стало:**
```dockerfile
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select audio_common_msgs rob_box_voice rob_box_animations \
    ...
```

Явно указываем `audio_common_msgs` чтобы убедиться что пакет собран перед `rob_box_voice`.

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
