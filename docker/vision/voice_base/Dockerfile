# Base Dockerfile для Voice Assistant dependencies
# Содержит все Python библиотеки и системные зависимости
# Используется как база для rob_box_voice
#
# Цель: кеширование тяжелых зависимостей (PyTorch, Whisper, etc)
# для быстрой пересборки при изменениях в rob_box_voice коде

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap
ARG BUILD_VERSION=2025-10-14-v1

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

# Метаданные
LABEL org.opencontainers.image.title="Voice Base Image"
LABEL org.opencontainers.image.description="Base image with Python audio/ML dependencies for Voice Assistant"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# ============================================================================
# Системные зависимости (аудио, USB, ROS2)
# ============================================================================
RUN apt-get update && apt-get install -y \
    # Python runtime
    python3-pip \
    python3-dev \
    # Аудио система
    libasound2-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    ffmpeg \
    alsa-utils \
    pulseaudio \
    # USB библиотеки для ReSpeaker
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    # Утилиты
    git \
    wget \
    curl \
    unzip \
    # ROS2 message runtime (нужно для audio_common_msgs)
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Python зависимости (устанавливаем через requirements.txt для стабильности)
# ============================================================================

# Обновляем pip до последней версии
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# Копируем requirements.txt
COPY requirements.txt /tmp/requirements.txt

# Устанавливаем все зависимости одной командой
# Используем --no-cache-dir чтобы не раздувать образ
# Используем --timeout и --retries для стабильности на Raspberry Pi
RUN pip3 install --no-cache-dir \
    --timeout 120 \
    --retries 5 \
    -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# ============================================================================
# ReSpeaker драйверы (usb_4_mic_array и pixel_ring)
# ============================================================================
RUN cd /tmp && \
    git clone --depth 1 https://github.com/respeaker/usb_4_mic_array.git && \
    cd usb_4_mic_array && \
    pip3 install --no-cache-dir -r requirements.txt && \
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
# Скачивание STT/TTS моделей (offline inference)
# ============================================================================
RUN mkdir -p /models && \
    # Vosk STT: Russian small model (45 MB, быстрая инференция)
    wget -q -O /tmp/vosk-model.zip \
        https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip && \
    unzip -q /tmp/vosk-model.zip -d /models/ && \
    mv /models/vosk-model-small-ru-0.22 /models/vosk-model-ru && \
    rm /tmp/vosk-model.zip && \
    # Silero TTS v4: Russian model (100 MB, нейронная TTS)
    # Голоса: aidar, baya, kseniya, xenia
    wget -q -O /models/silero_v4_ru.pt \
        https://models.silero.ai/models/tts/ru/v4_ru.pt && \
    echo "✅ Downloaded STT/TTS models: Vosk (45 MB) + Silero v4 (100 MB)"

# ============================================================================
# Клонирование audio_common для audio_common_msgs
# ============================================================================
RUN mkdir -p /ws/src && \
    cd /ws/src && \
    git clone -b ros2 --depth 1 https://github.com/ros-drivers/audio_common.git && \
    cd audio_common && \
    # Оставляем только audio_common_msgs
    find . -maxdepth 1 -type d ! -name '.' ! -name '.git' ! -name 'audio_common_msgs' -exec rm -rf {} + && \
    echo "✅ audio_common_msgs prepared for building"

# Установка зависимостей audio_common_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths /ws/src/audio_common/audio_common_msgs --ignore-src -r -y || true

# Сборка audio_common_msgs (чтобы был доступен для rob_box_voice)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /ws && \
    colcon build \
    --packages-select audio_common_msgs \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release

# Source audio_common_msgs в bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Аудио environment variables
ENV AUDIODEV=hw:ArrayUAC10,0
ENV ALSA_CARD=ArrayUAC10

WORKDIR /ws

# Инфо о версии
RUN echo "Voice Base Image ${BUILD_VERSION}" > /voice_base_version.txt
