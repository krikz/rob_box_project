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
ARG BASE_IMAGE=ghcr.io/krikz/rob_box:voice-base-humble-latest
# Версия сборки для инвалидации кеша при изменении зависимостей
ARG BUILD_VERSION=2026-03-08-app-layer

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

# Метаданные
LABEL org.opencontainers.image.title="Voice Assistant System for Vision Pi"
LABEL org.opencontainers.image.description="AI voice assistant with ReSpeaker and LED animations"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Тяжёлые зависимости, Python runtime, STT/TTS модели, ReSpeaker drivers и
# audio_common_msgs уже собраны в voice-base. Здесь остаётся только app-layer.

# Renardo sample packs вынесены в отдельный voice-resources image.
# Это не инвалидирует app-layer при изменениях исходников voice/animations.

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src/rob_box_core src/rob_box_voice src/rob_box_animations src/rob_box_mcp_tools src/rob_box_perception_msgs src/rob_box_harness src/rob_box_llm src/rob_box_supervisor_msgs

# ============================================================
# ОПТИМИЗАЦИЯ СЛОЕВ ДЛЯ КЕШИРОВАНИЯ (dependencies → config → code)
# ============================================================

# Шаг 1: Копируем package.xml для rosdep (зависимости меняются редко)
COPY src/rob_box_core/package.xml /ws/src/rob_box_core/
COPY src/rob_box_voice/package.xml /ws/src/rob_box_voice/
COPY src/rob_box_animations/package.xml /ws/src/rob_box_animations/
COPY src/rob_box_perception_msgs/package.xml /ws/src/rob_box_perception_msgs/
COPY src/rob_box_mcp_tools/package.xml /ws/src/rob_box_mcp_tools/
COPY src/rob_box_harness/package.xml /ws/src/rob_box_harness/
COPY src/rob_box_llm/package.xml /ws/src/rob_box_llm/
# IDL пакет для avatar_supervisor (AV-12 / AV-5): собирается раньше
# rob_box_supervisor, потому что тот на нём зависит.
COPY src/rob_box_supervisor_msgs/package.xml  /ws/src/rob_box_supervisor_msgs/
# Issue #2279 (arch review): публикуем согласованный colcon-workspace
# в voice-assistant — supervisor/quest больше НЕ пересобирают
# rob_box_supervisor_msgs/rob_box_core (это приводило к version skew
# между базой и потребителями, регрессии #2081 / #2089).
# COPY идёт ПЕРЕД colcon build — иначе голосасс build
# падает на ament_cmake (нужны CMakeLists.txt/srv/msg).
COPY src/rob_box_supervisor_msgs/CMakeLists.txt /ws/src/rob_box_supervisor_msgs/
COPY src/rob_box_supervisor_msgs/srv /ws/src/rob_box_supervisor_msgs/srv
COPY src/rob_box_supervisor_msgs/msg /ws/src/rob_box_supervisor_msgs/msg

# Шаг 2: Установка ROS зависимостей через rosdep
# Этот слой кешируется пока package.xml не изменяется
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src/rob_box_core src/rob_box_voice src/rob_box_animations src/rob_box_perception_msgs src/rob_box_mcp_tools src/rob_box_harness src/rob_box_llm src/rob_box_supervisor_msgs --ignore-src -r -y || true

# =============================================================================
# Шаг 2.5: APP-LAYER PYTHON ЗАВИСИМОСТИ (issue #2277)
# =============================================================================
# ВАЖНО: Этот слой СТАВИТСЯ ДО COPY Python-исходников (Шаг 8). Все pip-пакеты
# ниже не зависят от нашего кода — только от PyPI. Если перенести их ПОСЛЕ
# `COPY src/rob_box_*/rob_box_*`, любая правка .py инвалидирует ~5 минут
# компиляции resemblyzer/llvmlite на arm64 (run #34342854085 показал 8m25s
# вместо ожидаемых ~1m при тёплом кеше).
#
# Раньше было 7 отдельных RUN pip3 install ... ПОСЛЕ Step 8 + 2 ДУБЛЯ
# (resemblyzer x2, coverage+pyyaml x2). Дубли — артефакт параллельных
# hotfix-коммитов (#1101, #1104, #1160), которые никто не чистил.
# Все пакеты объединены в ОДИН RUN: один слой, одна транзакция pip,
# один cache-key, никаких дублей.
#
# Почему НЕ voice_base: ~10 GB pytorch/vosk/yandex/renardo cache
# инвалидируется для всех потребителей (voice_assistant, supervisor,
# quest). aiohttp/prometheus/resemblyzer/coverage/pyyaml/ddgs/
# duckduckgo-search — лёгкие, ставятся в app-layer точечно.
#
# Пакеты:
#   - aiohttp — voice-action-server / rob_box_quest ws-server
#   - prometheus_client — metrics endpoint audio_node (issue #1160)
#   - resemblyzer — speaker_id_node (issue #1077); на ARM64 компилирует
#     llvmlite ~3 min, на x86 — pre-built wheel
#   - coverage, pyyaml — runtime-pin для resemblyzer → librosa → numba
#     (issue #1101: coverage.types.Tracer, pyyaml C-парсер)
#   - ddgs — переименованный duckduckgo-search (issue #1104); search_web
#     и music_skill (search_artist_style) пробуют импорт ddgs ПЕРВЫМ
#   - duckduckgo-search — fallback в web_search.py (импорт-цепочка
#     ddgs → duckduckgo_search), нельзя убирать
RUN pip3 install --no-cache-dir --retries 5 --timeout 300 \
    "aiohttp>=3.9.1,<4.0" \
    "prometheus_client>=0.20.0" \
    "resemblyzer" \
    "coverage>=7.0" \
    "pyyaml>=6.0" \
    "ddgs>=9.0.0" \
    "duckduckgo-search>=8.0.0"

# Шаг 3: Копируем setup.py и CMakeLists.txt (структура пакета меняется редко)
COPY src/rob_box_core/setup.py /ws/src/rob_box_core/
COPY src/rob_box_voice/setup.py /ws/src/rob_box_voice/
COPY src/rob_box_voice/setup.cfg /ws/src/rob_box_voice/
COPY src/rob_box_animations/CMakeLists.txt /ws/src/rob_box_animations/
COPY src/rob_box_perception_msgs/CMakeLists.txt /ws/src/rob_box_perception_msgs/
COPY src/rob_box_mcp_tools/setup.py /ws/src/rob_box_mcp_tools/
COPY src/rob_box_mcp_tools/setup.cfg /ws/src/rob_box_mcp_tools/
COPY src/rob_box_harness/setup.py /ws/src/rob_box_harness/
COPY src/rob_box_harness/setup.cfg /ws/src/rob_box_harness/
COPY src/rob_box_llm/setup.py /ws/src/rob_box_llm/
COPY src/rob_box_llm/setup.cfg /ws/src/rob_box_llm/

# Шаг 4: Копируем конфигурационные файлы и ресурсы
COPY src/rob_box_core/resource /ws/src/rob_box_core/resource
# Issue #2250: rob_box_core/setup.py data_files включает config/speech_segmentation.yaml
# (issue #2199, PR #2211 voice-vr 14). Без COPY всей директории setup.py падает
# при install с "error: can't copy 'config/speech_segmentation.yaml': doesn't exist".
# Копируем директорию целиком (glob-стиль), а не конкретный файл — иначе любое
# новое config/* сломает билд тем же путём (issue #2250, ретро #2211).
COPY src/rob_box_core/config /ws/src/rob_box_core/config
COPY src/rob_box_voice/resource /ws/src/rob_box_voice/resource
COPY src/rob_box_mcp_tools/resource /ws/src/rob_box_mcp_tools/resource
COPY src/rob_box_harness/resource /ws/src/rob_box_harness/resource
COPY src/rob_box_llm/resource /ws/src/rob_box_llm/resource

# Шаг 5: Копируем launch файлы и конфигурации
COPY src/rob_box_voice/launch /ws/src/rob_box_voice/launch
COPY src/rob_box_voice/config /ws/src/rob_box_voice/config
COPY src/rob_box_voice/prompts /ws/src/rob_box_voice/prompts
COPY src/rob_box_animations/launch /ws/src/rob_box_animations/launch

# Шаг 6: Копируем message файлы для rob_box_perception_msgs
COPY src/rob_box_perception_msgs/msg /ws/src/rob_box_perception_msgs/msg

# Шаг 7: Копируем animations data
COPY src/rob_box_animations/animations /ws/src/rob_box_animations/animations

# ============================================================
# ИНВАЛИДАЦИЯ КЕША ДЛЯ PYTHON КОДА
# ============================================================
# BuildKit кеширует COPY layers по content hash файлов.
# Но на self-hosted runners кеш может быть stale.
# SOURCE_HASH передаётся из workflow и инвалидирует кеш при изменении кода.
ARG SOURCE_HASH=unknown
RUN echo "Source hash: ${SOURCE_HASH}" > /tmp/.source_hash

# Шаг 8: Копируем исходный код Python (меняется чаще всего - последний слой!)
COPY src/rob_box_core/rob_box_core /ws/src/rob_box_core/rob_box_core
COPY src/rob_box_voice/rob_box_voice /ws/src/rob_box_voice/rob_box_voice
COPY src/rob_box_animations/rob_box_animations /ws/src/rob_box_animations/rob_box_animations
COPY src/rob_box_mcp_tools/rob_box_mcp_tools /ws/src/rob_box_mcp_tools/rob_box_mcp_tools
COPY src/rob_box_harness/rob_box_harness /ws/src/rob_box_harness/rob_box_harness
COPY src/rob_box_llm/rob_box_llm /ws/src/rob_box_llm/rob_box_llm

# После копирования Python исходников можно переиспользовать patch helper для
# ремонта broken Renardo SynthDef-ов в установленном пакете.
COPY docker/vision/voice_assistant/fix_brass_scd.py /tmp/fix_brass_scd.py
RUN python3 /tmp/fix_brass_scd.py && rm /tmp/fix_brass_scd.py

# Live 13.08 — тихие ворнинги renardo при каждом старте контейнера:
#   "Could not read conf.json file..." (Settings/conf.py ищет свой conf.json
#   ВНУТРИ pip-пакета и создаёт его в writable-слое при первом импорте) и
#   ".../FoxDotEditor/themes/default.json color config file not found!"
#   (headless-образ не содержит GUI-редактор FoxDotEditor, renardo падает в
#   fallback-путь). Оба косметические, но замусоривают логи робота.
#   Создаём оба файла заранее — импорт перестаёт ворнить.
RUN python3 - <<'PY'
import json, os
conf = "/usr/local/lib/python3.10/dist-packages/renardo_lib/Settings/conf.json"
if not os.path.exists(conf):
    json.dump({
        "ADDRESS": "localhost", "PORT": 57110, "PORT2": 57120,
        "FONT": "Consolas", "SUPERCOLLIDER": "", "BOOT_ON_STARTUP": False,
        "CHECK_FOR_UPDATE": True, "SC3_PLUGINS": False, "MAX_CHANNELS": 2,
        "SAMPLES_DIR": "", "SAMPLES_PACK_NUMBER": 0, "GET_SC_INFO": True,
        "USE_ALPHA": True, "ALPHA_VALUE": 0.8, "MENU_ON_STARTUP": True,
        "CONSOLE_ON_STARTUP": True, "LINENUMBERS_ON_STARTUP": True,
        "TREEVIEW_ON_STARTUP": False, "TRANSPARENT_ON_STARTUP": False,
        "RECOVER_WORK": True, "LINE_NUMBER_MARKER_OFFSET": 0,
        "AUTO_COMPLETE_BRACKETS": True, "CPU_USAGE": 2, "CLOCK_LATENCY": 0,
        "FORWARD_ADDRESS": "", "FORWARD_PORT": 0, "COLOR_THEME": "cyborg",
        "TEXT_COLORS": "default",
    }, open(conf, "w"))
themes = "/usr/local/lib/python3.10/FoxDotEditor/FoxDotEditor/themes"
os.makedirs(themes, exist_ok=True)
theme = os.path.join(themes, "default.json")
if not os.path.exists(theme):
    # ВАЖНО: conf.py делает json_object[TEXT_COLORS]['plaintext'] и т.д.
    # Т.е. файл ДОЛЖЕН быть вложенным словарём {"default": {...}}.
    # Плоская схема ломает импорт renardo_lib.runtime:
    # KeyError: 'default' (live 13.08, round-2 hotfix).
    json.dump({
        "default": {
            "plaintext": "#ffffff", "background": "#1a1a1a",
            "functions": "#bf4acc", "key_types": "#29abe2",
            "user_defn": "#29abe2", "other_kws": "#49db8b",
            "comments": "#666666", "numbers": "#e89c18",
            "strings": "#eae02a", "dollar": "#ec4e20",
            "arrow": "#eae02a", "players": "#ec4e20",
            "prompt_fg": "#ffffff", "prompt_bg": "#1a1a1a",
            "console_text": "#ffffff", "console_bg": "#1a1a1a",
        },
    }, open(theme, "w"))
print("renardo conf.json + FoxDotEditor theme stub created")
PY

# (Шаги pip install, ранее жившие здесь, перенесены в Шаг 2.5 выше —
# до COPY Python-исходников, см. issue #2277. Дубли и комментарии к ним
# удалены: aiohttp/prometheus_client/resemblyzer/coverage/pyyaml/ddgs/
# duckduckgo-search теперь ставятся одним RUN.)

# Шаг 9: Копируем scripts
COPY src/rob_box_voice/scripts /ws/src/rob_box_voice/scripts
COPY src/rob_box_animations/scripts /ws/src/rob_box_animations/scripts

# Шаг 10: Копируем FoxDot SC init скрипт для sclang
# sclang читает этот файл при старте контейнера, регистрирует OSCdef /foxdot
# на порту 57120 для получения запросов на компиляцию SynthDef-ов от renardo.
COPY docker/vision/voice_assistant/foxdot_init.sc /ws/foxdot_init.sc
COPY docker/vision/voice_assistant/custom_synthdefs /ws/custom_synthdefs

# ============================================
# 🚀 СБОРКА ROS2 ПАКЕТОВ
# ============================================

# ИСПРАВЛЕНИЕ: Убираем проблемные CMAKE_LIBRARY_PATH и CMAKE_FIND_ROOT_PATH,
# которые мешают правильному поиску библиотек через CMake package config.
# Sourcing /opt/ros/humble/setup.sh достаточно для правильной настройки путей.
# В production image не используем --symlink-install: для ament_python это ведет
# к setup.py develop --editable, а поведение editable-режима зависит от версии setuptools.
# ДОПОЛНИТЕЛЬНО: очищаем CMake кеш перед сборкой для избежания проблем с base image
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    rm -rf /opt/ros/${ROS_DISTRO}/share/*/cmake/*.cmake~ && \
    ldconfig && \
    colcon build \
    --packages-select rob_box_core rob_box_supervisor_msgs rob_box_perception_msgs rob_box_mcp_tools rob_box_llm rob_box_harness rob_box_voice rob_box_animations \
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
ENV TTS_CACHE_DIR=/config/voice_assistant/tts_cache

# Проверка доступности нод
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD ros2 node list | grep -q "audio_node\|animation_player" || exit 1

# Точка входа
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]

# По умолчанию запускаем через startup script
CMD ["/scripts/start_voice_assistant.sh"]
