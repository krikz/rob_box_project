# Base Dockerfile для Voice Assistant dependencies
# Содержит все Python библиотеки и системные зависимости
# Используется как база для rob_box_voice
#
# Цель: кеширование тяжелых зависимостей (PyTorch, Whisper, etc)
# для быстрой пересборки при изменениях в rob_box_voice коде

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap
# Issue #2378 (devops, arch review): добавили apt-зависимости voice-assistant
# (python3-httpx, python3-aiohttp) сюда, чтобы voice-assistant/Dockerfile мог
# УБРАТЬ `rosdep update && rosdep install || true` (Шаг 2) — это был
# единственный `rosdep install` в проекте после PR #2286 (9414bb3, issue #2279),
# и он проглатывал сетевые ошибки (`|| true`). Паттерн «build once, share with all»
# (как для rob_box_supervisor_msgs) — apt-зависимости резолвятся здесь,
# consumers (voice-assistant) переиспользуют. Bump при изменении apt-слоя —
# voice-assistant наследует через BASE_IMAGE=$LOCAL_PREFIX:voice-base-$ROS-$TAG
# (см. .github/workflows/L-Build Vision Pi Services.yml build-voice-assistant).
ARG BUILD_VERSION=2026-09-14-2378-httpx-aiohttp

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

# Метаданные
LABEL org.opencontainers.image.title="Voice Base Image"
LABEL org.opencontainers.image.description="Base image with Python audio/ML dependencies for Voice Assistant"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# ============================================================================
# Системные зависимости (аудио, USB, ROS2)
# ============================================================================
# ВАЖНО: DEBIAN_FRONTEND=noninteractive обязателен — jackd2 (зависимость
# supercollider-language) в postinst задаёт вопросы через debconf и может
# подвесить сборку без TTY.
RUN apt-get update && apt-get install -y debconf-utils \
    && echo "jackd2 jackd/tweak_rt_limits boolean false" | debconf-set-selections \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    python3-dev \
    libasound2-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    ffmpeg \
    alsa-utils \
    pulseaudio \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    git \
    wget \
    curl \
    unzip \
    usbutils \
    supercollider-language \
    sc3-plugins-language \
    ros-dev-tools \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    # Issue #2378: добавили python3-httpx + python3-aiohttp для voice-assistant,
    # чтобы он мог убрать `rosdep install || true` (см. Шаг 2 в
    # docker/vision/voice_assistant/Dockerfile). python3-yaml/pil/numpy уже
    # доступны из rob_box_base:rtabmap (introlab3it/rtabmap_ros:humble-latest
    # наследует Ubuntu jammy со всем python3-набором). python3-openai
    # НЕ существует в apt — rob_box_llm ставит openai через pip в setup.py
    # install_requires; <exec_depend>python3-openai</exec_depend> в его
    # package.xml — техдолг (не наша зона), но rosdep раньше его проглатывал
    # через || true; после удаления rosdep install — он просто не вызывается.
    python3-httpx \
    python3-aiohttp \
    && apt-get install -y --reinstall \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-rosidl-runtime-c \
    ros-${ROS_DISTRO}-rosidl-runtime-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Python базовые инструменты (отдельный слой, не зависит от requirements.txt)
# ============================================================================
RUN pip3 install --no-cache-dir --retries 5 --timeout 120 --upgrade pip "setuptools>=59.6.0,<68" wheel

# ============================================================================
# Python зависимости
# ============================================================================
COPY requirements.txt /tmp/requirements.txt

RUN --mount=type=cache,target=/root/.cache,sharing=locked \
    pip3 install --retries 5 --timeout 120 -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# ============================================================================
# ReSpeaker драйверы (usb_4_mic_array и pixel_ring)
# ============================================================================
RUN cd /tmp && \
    git clone --depth 1 https://github.com/respeaker/usb_4_mic_array.git && \
    cd usb_4_mic_array && \
    pip3 install --retries 5 --timeout 120 -r requirements.txt && \
    cp tuning.py /usr/local/bin/ && \
    cp dfu.py /usr/local/bin/ && \
    chmod +x /usr/local/bin/tuning.py /usr/local/bin/dfu.py && \
    cd /tmp && \
    git clone --depth 1 https://github.com/respeaker/pixel_ring.git && \
    cd pixel_ring && \
    python3 setup.py install && \
    cd / && rm -rf /tmp/usb_4_mic_array /tmp/pixel_ring

# ============================================================================
# udev правила для ReSpeaker USB device
# ============================================================================
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666", GROUP="audio"' \
    > /etc/udev/rules.d/60-respeaker.rules

# ============================================================================
# STT/TTS модели — БОЛЬШЕ НЕ ЗДЕСЬ (Ресурсный пак, Этап 3)
# ============================================================================
# Здесь была ступень скачивания Vosk (44 МиБ архив) + Silero v5 (139 МиБ —
# комментарий тут годами говорил «68 МБ», измерено 21.09.2026) + прогрев
# torch.hub. Удалена по docs/plans/2026-09-15-resource-pack.md §6.1/§7 Этап 3.
#
# Почему: сборка образа делала два сетевых запроса, и при недоступной сети
# падала целиком (цепочка `wget ... && unzip ... && test -f` без `|| true`),
# а ошибка тонула в стандартном выводе wget. Плюс ~227 МиБ моделей лежали в
# слое образа, который тянет каждый холодный `docker pull` на Vision Pi.
#
# Куда переехало: docker/vision/scripts/resource_pack/manifest.yaml, записи
# `vosk-ru-small` и `silero-tts-v5-ru`. Файлы кладёт на ХОСТ шаг деплоя
# «[Vision Pi] Ensure STT/TTS models», контейнер получает их bind-mount'ом
# `/opt/rob_box/models:/models:ro` (docker-compose.yaml, voice-assistant).
# Пути ВНУТРИ контейнера не изменились — stt_node.yaml:8 и tts_node.py:1853
# читают тот же `/models/...`, поэтому ни один Python-файл не правился.
#
# `mkdir -p /models` тоже не нужен: каталог создаёт bind-mount.
#
# Прогрев `torch.hub.load('snakers4/silero-models')` уехал вместе со ступенью
# и НЕ заменён ничем: это был fallback третьего уровня (tts_node.py:1874-1879)
# без pin на коммит, тянувший HEAD чужого репозитория. Если он сработает в
# рантайме — полезет в сеть на первом вызове. Судьба этого fallback'а —
# открытый вопрос плана §13.5, отдельным решением.

# ============================================================================
# Клонирование и сборка audio_common_msgs
# ============================================================================
RUN mkdir -p /ws/src && \
    cd /ws/src && \
    git clone -b ros2 --depth 1 https://github.com/ros-drivers/audio_common.git && \
    cd audio_common && \
    find . -maxdepth 1 -type d ! -name '.' ! -name '.git' ! -name 'audio_common_msgs' -exec rm -rf {} + && \
    echo "✅ audio_common_msgs prepared for building"

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths /ws/src/audio_common/audio_common_msgs --ignore-src -r -y || true

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /ws && \
    colcon build \
    --packages-select audio_common_msgs \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

ENV AUDIODEV=hw:ArrayUAC10,0
ENV ALSA_CARD=ArrayUAC10
ENV SOUND_PACK_DIR=/ws/sound_pack
ENV TTS_CACHE_DIR=/config/voice_assistant/tts_cache

WORKDIR /ws

RUN echo "Voice Base Image ${BUILD_VERSION}" > /voice_base_version.txt
