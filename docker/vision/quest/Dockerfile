# syntax=docker/dockerfile:1.7
# Dockerfile для Meta Quest WebXR-консоли (Phase 1.6, ADR-0027)
#
# Лёгкий runtime поверх caddy:2-alpine:
#   - Stage 1 (builder): node:20-alpine собирает webxr_client (Vite).
#   - Stage 2 (runtime): caddy:2-alpine + python3 + aiohttp/msgpack для
#     rob_box_quest. Всё через pip, без тяги voice-assistant/pytorch.
#
# Зависимость от webxr_client: Phase 1.4/1.5 должны быть merged в
# текущую ветку ДО сборки этого образа (build контекст — корень репо,
# см. docker/vision/supervisor/Dockerfile как эталон по структуре).
# Когда Phase 1.5 в feature/avatar попадёт в develop — этот Dockerfile
# начнёт собираться «как есть». До тех пор CI на нём даст ошибку COPY
# про отсутствующий путь — это явный сигнал зависимости, не silent
# degradation.
#
# Trade-off (см. kanban t_8539461e, вопрос Q1): могли бы наследоваться
# от voice-assistant (там уже собран rob_box_quest), но это утянет pytorch
# ~10GB и Renardo. Когда rob_box_quest войдёт в voice-assistant image
# как colcon-пакет — можно пересмотреть.
#
# Источники истины:
#   - docs/adr/0027-meta-quest-ar-control.md §4.1, §4.5
#   - docs/architecture/meta-quest-api.md §1, §3
#   - docker/vision/supervisor/Dockerfile (паттерн COPY-этапов)
#   - docs/development/DOCKER_STANDARDS.md (нет COPY config/, нет COPY scripts/)

ARG BUILDER_IMAGE=node:20-alpine
# Caddy binary provider: caddy:2-alpine уже содержит /usr/sbin/caddy.
# Его COPY в runtime layer — не apk add (dl-cdn на Katana отрезан,
# см. raw-evidence #32894150667). Бинарь ~45 MB, кешируется отдельно.
ARG CADDY_BUILDER=caddy:2-alpine
# Runtime: voice-assistant image — наследует ros2 humble-base, pip, aiohttp,
# msgpack уже установлены. Зачем: rob_box_quest.quest_node использует
# rclpy (publishes Twist, subscribes LaserScan/Odometry) — без
# /opt/ros/humble/setup.bash стартует WS-сервер, но teleop/мост
# не работают (raw-evidence Vision Pi: /opt/ros/humble/setup.bash not found).
ARG RUNTIME_IMAGE=ghcr.io/krikz/rob_box:voice-assistant-humble-${IMAGE_TAG:-latest}

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
# Stage 2: Runtime — voice-assistant base (ros2 + python + aiohttp) + caddy
# ============================================================
FROM ${RUNTIME_IMAGE}

# ARG обязательно переобъявить в новом FROM — иначе ${ROS_DISTRO} пустой
# и '/opt/ros//setup.sh' не находится (raw-evidence #32899426873 step:25).
ARG ROS_DISTRO=humble

# Базовые пакеты, которых нет в voice-assistant: openssl (для
# self-signed серта) + bash уже есть (Ubuntu base). curl — на всякий.
# aiohttp/msgpack уже стоят (install_requires voice-assistant), но pip3
# install здесь идёмпотентен.
#
# opencv-python-headless: apt python3-opencv (cv2 4.5.4) в base ломается на
# pip-numpy 2.x (`ImportError: numpy.core.multiarray failed to import`) —
# ставим pip-cv2, собранный под numpy 2.x. Нужен для camera_rear JPEG
# (numpy → cv2.imencode) в quest_node.
RUN apt-get update -qq \
 && apt-get install -y --no-install-recommends openssl curl ca-certificates \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir --break-system-packages aiohttp msgpack opencv-python-headless \
 || true

# Копируем caddy binary из caddy:2-alpine (45 MB, оффлайн-safe).
# Положим в /usr/local/bin — в PATH у root в Ubuntu он есть.
COPY --from=caddy-builder /usr/bin/caddy /usr/local/bin/caddy

# Статика WebXR-клиента из node-builder
COPY --from=webxr-builder /build/dist /srv/quest_static

# ============================================================
# Сборка rob_box_quest (ament_python) — образец supervisor/Dockerfile
# ============================================================
# Шаг 1: package.xml + setup.py + setup.cfg (структура, кешируется)
COPY src/rob_box_quest/package.xml /ws/src/rob_box_quest/
COPY src/rob_box_quest/setup.py   /ws/src/rob_box_quest/
COPY src/rob_box_quest/setup.cfg  /ws/src/rob_box_quest/

# Шаг 2: ament_index marker. setup.py ищет ФАЙЛ 'resource/<package_name>'
# (data_files copy), не директорию (raw-evidence #32900725289 step:25:
# error: can't copy 'resource/rob_box_quest': doesn't exist or not a regular file).
# Образец — src/rob_box_supervisor/resource/rob_box_supervisor (0 байт).
COPY src/rob_box_quest/resource/rob_box_quest /ws/src/rob_box_quest/resource/rob_box_quest

# Шаг 3: исходный код Python (самый горячий слой, последний)
COPY src/rob_box_quest/rob_box_quest /ws/src/rob_box_quest/rob_box_quest

# Шаг 4: пересборка ТОЛЬКО rob_box_quest (pytorch/renardo из base не трогаем).
# WORKDIR=/ws обязателен — colcon ищет пакеты в $PWD/src, а после
# 'source /opt/ros/.../setup.sh' WORKDIR не меняется, но нам нужно
# явно стартануть из /ws.
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    colcon build \
        --packages-select rob_box_quest \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Caddyfile + entry-script
COPY docker/vision/quest/Caddyfile       /etc/caddy/Caddyfile
COPY docker/vision/scripts/quest/start_quest.sh /scripts/start_quest.sh
RUN chmod +x /scripts/start_quest.sh

# /etc/caddy по умолчанию в voice-assistant есть (apt-installed в др.
# образах). Не пересоздаём — start_quest.sh пишет серты в /certs
# (volume), а Caddyfile лежит здесь.

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
