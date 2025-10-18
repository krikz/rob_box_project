# Dockerfile для Voice Assistant System
# AI голосовой ассистент с интеграцией LED анимаций
#
# Компоненты:
#   - rob_box_voice: голосовой ассистент (AudioNode, STTNode, TTSNode, DialogueNode, LEDNode, SoundNode)
#   - rob_box_animations: LED матричные анимации для эмоций и состояний
#
# Hardware:
#   - ReSpeaker Mic Array v2.0 (USB, 4 микрофона, 12× RGB LED)
#   - Sound card WM8960 (3.5mm jack для TTS output)
#   - NeoPixel LED матрицы (через led_matrix_compositor)
ARG BASE_IMAGE=ghcr.io/krikz/rob_box:nav2-humble-latest
# Версия сборки для инвалидации кеша при изменении зависимостей
ARG BUILD_VERSION=2025-10-12-rtabmap

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

# Метаданные
LABEL org.opencontainers.image.title="Voice Assistant System for Vision Pi"
LABEL org.opencontainers.image.description="AI voice assistant with ReSpeaker and LED animations"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    # Аудио система
    python3-pip \
    python3-dev \
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
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Установка ROS2 зависимостей для сборки пакетов (rob_box_animations требует sensor_msgs)
# КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: --reinstall для builtin-interfaces чинит сломанный CMake config
# Проблема: .so файл есть, но CMake не может его найти → нужен --reinstall
# ВАЖНО: apt-get install и --reinstall ДОЛЖНЫ быть в одном RUN (для доступа к apt cache!)
# Источник: docs/development/DOCKER_BUILD_FIXES.md (проблема #4)
RUN apt-get update && apt-get install -y \
    # ROS2 development tools (для colcon build с custom messages)
    ros-dev-tools \
    # ROS2 зависимости
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    # ROS2 message generation (КРИТИЧЕСКИ ВАЖНО для builtin_interfaces)
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    # ПЕРЕУСТАНОВКА в том же RUN (до rm -rf /var/lib/apt/lists)
    # --reinstall пересоздаёт CMake config файлы с правильными путями
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

# ОТЛАДКА: Проверяем что CMake find_library() находит библиотеки
RUN echo "=== ПОСЛЕ --reinstall: builtin_interfaces ===" && \
    ls -lh /opt/ros/humble/lib/libbuiltin_interfaces*.so* && \
    echo "=== Список без префикса lib: ===" && \
    cd /opt/ros/humble/lib && ls -1 libbuiltin_interfaces*.so | sed 's/^lib//' | sed 's/\.so$//'

# Установка Python библиотек для голосового ассистента
# Используем requirements.txt для лучшего кеширования и управления зависимостями
# BuildKit cache mount позволяет переиспользовать загруженные пакеты между сборками
# ВАЖНО: setuptools <80 требуется для совместимости с colcon-core и antlr4-python3-runtime

# Копируем requirements.txt ПЕРЕД остальными файлами для лучшего кэширования
COPY docker/vision/voice_assistant/requirements.txt /tmp/requirements.txt

# Устанавливаем pip, setuptools, wheel и все зависимости одной командой
# BuildKit cache mount сохраняет загруженные пакеты между сборками
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --no-cache-dir --upgrade pip "setuptools>=59.6.0,<68" wheel && \
    pip3 install -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src/rob_box_voice src/rob_box_animations

# ============================================================
# ОПТИМИЗАЦИЯ СЛОЕВ ДЛЯ КЕШИРОВАНИЯ (dependencies → config → code)
# ============================================================

# Шаг 1: Клонируем audio_common_msgs (редко меняется)
RUN cd /ws/src && \
    git clone -b ros2 --depth 1 https://github.com/ros-drivers/audio_common.git && \
    cd audio_common && \
    # Оставляем только audio_common_msgs, удаляем остальные пакеты
    find . -maxdepth 1 -type d ! -name '.' ! -name '.git' ! -name 'audio_common_msgs' -exec rm -rf {} + && \
    echo "✅ audio_common_msgs extracted from ros2 branch"

# Шаг 2: Копируем package.xml для rosdep (зависимости меняются редко)
COPY src/rob_box_voice/package.xml /ws/src/rob_box_voice/
COPY src/rob_box_animations/package.xml /ws/src/rob_box_animations/

# Шаг 3: Установка ROS зависимостей через rosdep
# Этот слой кешируется пока package.xml не изменяется
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src/rob_box_voice src/rob_box_animations --ignore-src -r -y || true

# Шаг 4: Копируем setup.py и CMakeLists.txt (структура пакета меняется редко)
COPY src/rob_box_voice/setup.py /ws/src/rob_box_voice/
COPY src/rob_box_voice/setup.cfg /ws/src/rob_box_voice/
COPY src/rob_box_animations/CMakeLists.txt /ws/src/rob_box_animations/

# Шаг 5: Копируем конфигурационные файлы и ресурсы
COPY src/rob_box_voice/resource /ws/src/rob_box_voice/resource
COPY sound_pack /ws/sound_pack

# Шаг 6: Копируем launch файлы и конфигурации
COPY src/rob_box_voice/launch /ws/src/rob_box_voice/launch
COPY src/rob_box_voice/config /ws/src/rob_box_voice/config

# Шаг 8: Копируем animations data
COPY src/rob_box_animations/animations /ws/src/rob_box_animations/animations

# Шаг 9: Копируем исходный код Python (меняется чаще всего - последний слой!)
COPY src/rob_box_voice/rob_box_voice /ws/src/rob_box_voice/rob_box_voice
COPY src/rob_box_animations/rob_box_animations /ws/src/rob_box_animations/rob_box_animations

# Шаг 10: Копируем scripts
COPY src/rob_box_voice/scripts /ws/src/rob_box_voice/scripts
COPY src/rob_box_animations/scripts /ws/src/rob_box_animations/scripts

# ============================================================
# ВНЕШНИЕ ЗАВИСИМОСТИ (ReSpeaker drivers, models)
# ============================================================

# Установка ReSpeaker драйверов (usb_4_mic_array и pixel_ring)
# Эти библиотеки меняются редко, поэтому ставим до сборки ROS пакетов
RUN cd /tmp && \
    git clone https://github.com/respeaker/usb_4_mic_array.git && \
    cd usb_4_mic_array && \
    pip3 install --no-cache-dir -r requirements.txt && \
    # Копируем tuning.py и dfu.py в системный путь
    cp tuning.py /usr/local/bin/ && \
    cp dfu.py /usr/local/bin/ && \
    chmod +x /usr/local/bin/tuning.py /usr/local/bin/dfu.py && \
    cd /tmp && \
    git clone https://github.com/respeaker/pixel_ring.git && \
    cd pixel_ring && \
    python3 setup.py install && \
    cd / && rm -rf /tmp/usb_4_mic_array /tmp/pixel_ring

# Настройка udev правил для ReSpeaker
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666", GROUP="audio"' \
    > /etc/udev/rules.d/60-respeaker.rules

# Скачивание и установка STT/TTS моделей (offline)
# Модели большие (145 MB) и никогда не меняются - кешируем
RUN mkdir -p /models && \
    # Vosk STT: Russian small model (45 MB, fast)
    wget -q -O /tmp/vosk-model.zip \
    https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip && \
    unzip -q /tmp/vosk-model.zip -d /models/ && \
    rm /tmp/vosk-model.zip && \
    # Silero TTS v4: Russian model (100 MB, neural TTS, 4 voices, SSML support)
    # Голоса: aidar (нейтральный М), baya (спокойный М), kseniya (нейтральный Ж), xenia (энергичный Ж)
    wget -q -O /models/silero_v4_ru.pt \
    https://models.silero.ai/models/tts/ru/v4_ru.pt && \
    echo "✅ STT/TTS models downloaded: Vosk (45 MB) + Silero v4 (100 MB)"

# ============================================
# 🚀 СБОРКА ROS2 ПАКЕТОВ
# ============================================

# ОТЛАДКА: Проверка состояния ПЕРЕД colcon build (не изменилось ли что-то между RUN-ами)
RUN echo "=== ПЕРЕД COLCON BUILD: проверка builtin_interfaces ===" && 
    ls -lh /opt/ros/humble/lib/libbuiltin_interfaces*.so* 2>/dev/null | head -3 && 
    echo "=== Проверка CMake config ===" && 
    grep "_exported_libraries" /opt/ros/humble/share/builtin_interfaces/cmake/ament_cmake_export_libraries-extras.cmake | head -1 && 
    echo "=== LD_LIBRARY_PATH ===" && 
    echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH" && 
    echo "=== CMAKE_PREFIX_PATH ===" && 
    echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH" && 
    echo "✅ Debug перед colcon build завершен"

# ИЗМЕНЕНО: Используем --symlink-install для избежания копирования Python файлов
# Это позволяет быстро пересобирать при изменении кода
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select audio_common_msgs rob_box_voice rob_box_animations \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# ROS2 и Zenoh environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_CONFIG=/config/shared/zenoh_session_config.json5

# Аудио environment variables
ENV AUDIODEV=hw:ArrayUAC10,0
ENV ALSA_CARD=ArrayUAC10

# Пути к данным
ENV SOUND_PACK_DIR=/ws/sound_pack
ENV TTS_CACHE_DIR=/config/voice/tts_cache

# Проверка доступности нод
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD ros2 node list | grep -q "audio_node\|animation_player" || exit 1

# Точка входа
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]

# По умолчанию запускаем через startup script
CMD ["/scripts/start_voice_assistant.sh"]
