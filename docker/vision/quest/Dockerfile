# syntax=docker/dockerfile:1.7
# Dockerfile для Meta Quest WebXR-консоли (Phase 1.6, ADR-0027)
#
# Лёгкий runtime поверх ros2-zenoh базы (НЕ voice-assistant!):
#   - Stage 1 (builder): node:20-alpine собирает webxr_client (Vite).
#   - Stage 1.5: caddy:2-alpine даёт бинарь caddy (~45 MB, offline-safe).
#   - Stage 2 (runtime): rob_box_base:ros2-zenoh-humble + apt ros-* пакеты
#     (ament_cmake/rosidl/audio_common_msgs/std/sensor/geometry/nav-msgs)
#     + pip aiohttp/msgpack/opencv-python-headless + colcon build
#     rob_box_supervisor_msgs + rob_box_core + rob_box_quest.
#
# ARCH-quest (#2278): ранее наследовались от voice-assistant-humble-...
# (RUNTIME_IMAGE). Это тянуло ~10 GB pytorch/Renardo/vosk в образ,
# build-quest = 5m40s (run #34342854085) + deploy на Vision Pi тянул эти
# слои по сети. rob_box_quest нужен только rclpy + aiohttp + msgpack +
# opencv-python-headless + caddy-бинарь — всё это есть в ros2-zenoh
# базе либо ставится парой apt-get/pip3 строк. База rob_box_base:ros2-zenoh
# — это ros:humble-ros-base + rmw_zenoh_cpp (zenoh-роутинг, нужный
# quest_node для /cmd_vel_quest и /odom подписок). Итоговый образ ~200 MB
# вместо ~10 GB, build параллельно с voice-assistant (нет needs-зависимости).
#
# Зависимость от webxr_client: Phase 1.4/1.5 должны быть merged в
# текущую ветку ДО сборки этого образа (build контекст — корень репо,
# см. docker/vision/supervisor/Dockerfile как эталон по структуре).
# Когда Phase 1.5 приедет в develop — этот Dockerfile начнёт собираться
# «как есть». До тех пор CI на нём даст ошибку COPY про отсутствующий
# путь — это явный сигнал зависимости, не silent degradation (ADR-0018).
#
# Источники истины:
#   - docs/adr/0027-meta-quest-ar-control.md §4.1, §4.5
#   - docs/architecture/meta-quest-api.md §1, §3
#   - docker/base/Dockerfile.ros2-zenoh (база)
#   - docker/vision/supervisor/Dockerfile (паттерн COPY-этапов)
#   - docs/development/DOCKER_STANDARDS.md (нет COPY config/, нет COPY scripts/)

ARG BUILDER_IMAGE=node:20-alpine
# Caddy binary provider: caddy:2-alpine уже содержит /usr/sbin/caddy.
# Его COPY в runtime layer — не apk add (dl-cdn на Katana отрезан,
# см. raw-evidence #32894150667). Бинарь ~45 MB, кешируется отдельно.
ARG CADDY_BUILDER=caddy:2-alpine
# Runtime: rob_box_base:ros2-zenoh-{DISTRO} — лёгкая база (ros:humble-ros-base
# + rmw_zenoh_cpp). /opt/ros/humble/setup.sh и rclpy здесь уже есть
# (нужны quest_node для publish Twist / subscribe LaserScan, Odometry).
# ARCH-quest (#2278): раньше стоял voice-assistant-humble-... — это тянуло
# ~10 GB pytorch/Renardo/vosk; rob_box_base:ros2-zenoh — это ~250 MB.
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-${ROS_DISTRO:-humble}

# ============================================================
# Stage 1: Vite build webxr_client
# ============================================================
FROM ${BUILDER_IMAGE} AS webxr-builder

WORKDIR /build

# Сначала package*.json — слой зависимостей кешируется отдельно.
COPY src/rob_box_quest/webxr_client/package*.json ./

# Исходники клиента. Если на момент сборки путь отсутствует — build
# упадёт на этом COPY с понятной ошибкой; это сигнал зависимости от
# Phase 1.5 (см.комментарий выше), не silent degradation (ADR-0018).
COPY src/rob_box_quest/webxr_client/ ./

RUN npm ci --no-audit --no-fund \
 && npm run build

# ============================================================
# Stage 1.5: extract caddy binary from caddy:2-alpine (offline-safe)
# ============================================================
FROM ${CADDY_BUILDER} AS caddy-builder

# ============================================================
# Stage 2: Runtime — rob_box_base:ros2-zenoh + ros-* apt deps + pip aiohttp
# ============================================================
FROM ${BASE_IMAGE}

# ARG обязательно переобъявить в новом FROM — иначе ${ROS_DISTRO} пустой
# и '/opt/ros//setup.sh' не находится (raw-evidence #32899426873 step:25).
ARG ROS_DISTRO=humble

# Katana (build-машина): прямого доступа к apt-репозиториям нет — как и
# dl-cdn (alpine), прямые запросы дают "GPG error: ... At least one invalid
# signature" (run #34387765562). Поэтому apt идёт через локальный
# apt-cacher-ng (APT_PROXY=http://host.docker.internal:3142) — тот же паттерн,
# что в docker/base/Dockerfile.ros2-zenoh. Билд-аргумент APT_PROXY передаётся
# из workflow (build-quest / case quest). 02proxy удаляем после apt — не
# тащим прокси-конфиг в рантайм (как делает base).
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        echo "🔍 Checking proxy availability at $PROXY_HOST:$PROXY_PORT..."; \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi

# ARCH-quest (#2278): ставим ТОЛЬКО нужные apt-пакеты поверх
# ros2-zenoh базы. ros2-zenoh = ros:humble-ros-base + rmw_zenoh_cpp, поэтому
# rclpy + zenoh-транспорт уже есть. Не ставим:
#   - pytorch / renardo / vosk / silero / resemblyzer / supercollider / etc.
#     Всё это нужно только voice-assistant (TTS/STT/LED-анимации), но НЕ
#     rob_box_quest (чистый WSS-bridge с publish Twist / sub LaserScan).
#   - python3-opencv из apt (4.5.4): ломается на pip numpy 2.x
#     (`ImportError: numpy.core.multiarray failed to import` — issue, см. ниже).
#
# Что ставим:
#   1. openssl — для self-signed TLS в start_quest.sh.
#   2. curl, ca-certificates — healthcheck и HTTPS-запросы.
#   3. libgl1, libglib2.0-0 — opencv-python-headless wheel для arm64
#      (pip-cv2) линкуется с libGL.so.1; на минимальной Ubuntu-base без
#      этого imencode падает `libGL.so.1: cannot open shared object file`.
#   4. ros-humble-{ament-cmake,rosidl-default-generators,rosidl-default-runtime}
#      — нужны для colcon-сборки rob_box_supervisor_msgs (ament_cmake пакет,
#      IDL генерируется rosidl). Без них colcon build supervisor_msgs падает
#      `Could not find ament_cmake`.
#   5. ros-humble-{std-msgs,sensor-msgs,geometry-msgs,nav-msgs,builtin-interfaces}
#      — rosidl-зависимости rob_box_supervisor_msgs (CMakeLists.txt) +
#      exec_depend rob_box_quest. Без них find_package(...) в CMake или
#      rclpy-импорт в quest_node падают ModuleNotFoundError.
#   6. ros-humble-audio-common-msgs — quest_node импортирует
#      `from audio_common_msgs.msg import AudioData` (для /avatar/voice_in).
#      Есть в apt (поиск `apt-cache show ros-humble-audio-common-msgs`),
#      весия 0.4.0. Раньше собиралось из upstream в voice_base (см. там
#      коммент "пакет недоступен через apt для Humble" — это устарело,
#      в jammy-pockets он появился; apt быстрее git clone + colcon build).
RUN apt-get update -qq \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      openssl \
      curl \
      ca-certificates \
      libgl1 \
      libglib2.0-0 \
      ros-${ROS_DISTRO}-ament-cmake \
      ros-${ROS_DISTRO}-rosidl-default-generators \
      ros-${ROS_DISTRO}-rosidl-default-runtime \
      ros-${ROS_DISTRO}-builtin-interfaces \
      ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-audio-common-msgs \
 && rm -rf /var/lib/apt/lists/* \
 && rm -f /etc/apt/apt.conf.d/02proxy

# Python runtime-deps. ros2-zenoh база — Ubuntu 22.04 + PEP 668 (externally
# managed), поэтому pip install требует --break-system-packages.
#   - aiohttp>=3.9: WS-сервер в ws_server.py (использует asyncio + WebSocket).
#   - msgpack: rob_box_core.bridge_protocol + rob_box_quest.protocol.topics.
#   - opencv-python-headless (>=4.8, собран под numpy 2.x): camera_rear JPEG
#     (numpy → cv2.imencode) + depth_compressed_to_jpeg + occupancy.png.
#     На arm64 pip-wheel pre-built — не собираем из исходников (иначе
#     opencv contrib сборка = ~15 мин на Pi-раннере).
# pip3 --no-cache-dir — экономия места (apt уже почистил apt lists).
RUN pip3 install --no-cache-dir --break-system-packages --retries 5 --timeout 300 \
      "aiohttp>=3.9.1,<4.0" \
      "msgpack>=1.0.0" \
      "opencv-python-headless>=4.8.0"

# Копируем caddy binary из caddy:2-alpine (45 MB, оффлайн-safe).
# Положим в /usr/local/bin — в PATH у root в Ubuntu он есть.
COPY --from=caddy-builder /usr/bin/caddy /usr/local/bin/caddy

# Статика WebXR-клиента из node-builder
COPY --from=webxr-builder /build/dist /srv/quest_static

# ============================================================
# Сборка rob_box_quest (ament_python) — образец supervisor/Dockerfile
# ============================================================
# /ws/src в ros2-zenoh базе НЕ существует (это ros:humble-ros-base без
# colcon-workspace). Создаём заранее, иначе COPY на /ws/src/... упадёт
# "no such file or directory". voice_assistant/Dockerfile делает это в
# WORKDIR+RUN mkdir -p в самом начале — мы следуем тому же паттерну.
WORKDIR /ws
RUN mkdir -p \
      src/rob_box_supervisor_msgs \
      src/rob_box_core \
      src/rob_box_quest

# Issue #2081 (регрессия quest: ModuleNotFoundError rob_box_core.avatar_command):
# --packages-select был только rob_box_quest → rob_box_core брался из
# закешированного слоя базового voice-assistant image. Когда avatar_command.py
# добавили в rob_box_core, voice-assistant пересобрали ПОЗЖЕ quest → quest
# унаследовал старый rob_box_core без avatar_command.py → restart loop на
# /ws/install/rob_box_core/lib/.../avatar_command.py.
#
# Лечение:
#   1) COPY src/rob_box_core/* в /ws/src/rob_box_core/* (согласовано с
#      voice-assistant/Dockerfile шаги 1+3+8) — инвалидирует кеш слоя
#      при изменениях в rob_box_core.
#   2) colcon build теперь собирает rob_box_core + rob_box_quest.
#
# rob_box_core — лёгкий pure-Python (нет torch/Renardo), его пересборка
# в quest image НЕ инвалидирует тяжёлый кеш pytorch (тот остался в
# voice-assistant, а не в нашей базе).
#
# Issue #2089 (регрессия quest restart loop v2: ModuleNotFoundError
# rob_box_supervisor_msgs): rob_box_quest.quest_node импортирует
#   from rob_box_supervisor_msgs.srv import ExecuteCommand
#   from rob_box_supervisor_msgs.msg import Command
# (Bridge.execute(Command), issue #2002, ADR-0051 §2.1). Пакет — ament_cmake,
# IDL генерируется rosidl_default_generators на colcon build. Если он не
# скопирован и не пересобран в quest image — quest не получает свежие
# Python-типы (ExecuteCommand/Command) из локального colcon workspace,
# и падает на старте с ModuleNotFoundError → Restarting loop.
#
# ros2-zenoh база supervisor_msgs НЕ содержит (это не voice-assistant),
# поэтому ОБЯЗАТЕЛЬНО копируем пакет в образ + пересобираем его из
# локального src/ — никаких "унаследуется из base" трюков, всё явно.
# std_msgs и builtin_interfaces ставим через apt в /opt/ros — они нужны
# rob_box_supervisor_msgs как rosidl-зависимости (см. CMakeLists.txt).
# Шаг 0: IDL-пакет (ament_cmake) — структура пакета, кешируется отдельно.
COPY src/rob_box_supervisor_msgs/package.xml  /ws/src/rob_box_supervisor_msgs/
COPY src/rob_box_supervisor_msgs/CMakeLists.txt /ws/src/rob_box_supervisor_msgs/
COPY src/rob_box_supervisor_msgs/srv /ws/src/rob_box_supervisor_msgs/srv
COPY src/rob_box_supervisor_msgs/msg /ws/src/rob_box_supervisor_msgs/msg

# Шаг 1: package.xml + setup.py + setup.cfg (структура, кешируется)
COPY src/rob_box_quest/package.xml /ws/src/rob_box_quest/
COPY src/rob_box_quest/setup.py   /ws/src/rob_box_quest/
COPY src/rob_box_quest/setup.cfg  /ws/src/rob_box_quest/

# Issue #2081: rob_box_core package.xml/setup.py тоже нужны на этом слое,
# чтобы следующий COPY ресурсов нашёл директорию и setup.py.
COPY src/rob_box_core/package.xml /ws/src/rob_box_core/
COPY src/rob_box_core/setup.py   /ws/src/rob_box_core/
# Issue #2250: rob_box_core/setup.py data_files включает config/speech_segmentation.yaml
# (issue #2199, PR #2211 voice-vr 14). Без COPY всей директории setup.py падает
# при install с "error: can't copy 'config/speech_segmentation.yaml': doesn't exist".
# Копируем директорию целиком (glob-стиль), а не конкретный файл — иначе любое
# новое config/* сломает билд тем же путём (issue #2250, ретро #2211).
COPY src/rob_box_core/config /ws/src/rob_box_core/config

# Шаг 2: ament_index marker. setup.py ищет ФАЙЛ 'resource/<package_name>'
# (data_files copy), не директорию (raw-evidence #32900725289 step:25:
# error: can't copy 'resource/rob_box_quest': doesn't exist or not a regular file).
# Образец — src/rob_box_supervisor/resource/rob_box_supervisor (0 байт).
COPY src/rob_box_quest/resource/rob_box_quest /ws/src/rob_box_quest/resource/rob_box_quest
COPY src/rob_box_core/resource/rob_box_core   /ws/src/rob_box_core/resource/rob_box_core

# Шаг 3: исходный код Python (самый горячий слой, последний)
COPY src/rob_box_quest/rob_box_quest /ws/src/rob_box_quest/rob_box_quest
# Issue #2081: rob_box_core — иначе avatar_command.py не попадёт в образ.
COPY src/rob_box_core/rob_box_core /ws/src/rob_box_core/rob_box_core

# Шаг 4: пересборка rob_box_supervisor_msgs (IDL) + rob_box_core +
# rob_box_quest. ARCH-quest (#2278): теперь это лёгкая сборка поверх
# ros2-zenoh (~250 MB base), без тяжёлого pytorch/renardo/vosk-слоя —
# colcon собирает только pure-Python + IDL, время ~30-60 сек.
# WORKDIR=/ws обязателен — colcon ищет пакеты в $PWD/src.
# Порядок пакетов в --packages-select важен: supervisor_msgs собирается
# первым, чтобы его сгенерированные Python-типы были доступны при
# сборке rob_box_core/rob_box_quest (на практике colcon сам разруливает
# graph-build-order, но явный порядок оставляем для воспроизводимости).
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build \
        --packages-select rob_box_supervisor_msgs rob_box_core rob_box_quest \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Caddyfile + entry-script
COPY docker/vision/quest/Caddyfile       /etc/caddy/Caddyfile
COPY docker/vision/scripts/quest/start_quest.sh /scripts/start_quest.sh
RUN chmod +x /scripts/start_quest.sh

# /etc/caddy — rob_box_base:ros2-zenoh (ros:humble-ros-base) его НЕ
# содержит, но COPY /etc/caddy/Caddyfile автоматически создаст родительскую
# директорию. start_quest.sh пишет серты в /certs (volume).

# certs будут создаваться в /certs при первом старте контейнера.
# Pre-create чтобы том сразу был доступен и start_quest.sh не падал,
# если стартует первым.
RUN mkdir -p /certs

EXPOSE 8443

# ENTRYPOINT намеренно НЕ устанавливаем. compose.yaml даёт:
#   command: ["/ros_scripts/ros_with_namespace.sh", "/bin/bash",
#             "/scripts/start_quest.sh"]
# Docker правила: exec-form ENTRYPOINT делает его аргументами к command
# (т.е. start_quest.sh /ros_scripts/... — args игнорятся). Без ENTRYPOINT
# command работает напрямую: ros_with_namespace.sh → bash → start_quest.sh.
# Raw-evidence: Vision Pi #32900725289+ logs показали только start_quest.sh
# echo без единого "Robot ID:" от ros_with_namespace.sh.
