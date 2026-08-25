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
# Runtime: alpine:3.20 (LTS) с подключённым community-repo. caddy:2-alpine
# в build-контейнере на Katana отрезан от dl-cdn.alpinelinux.org (DNS
# transient, см. raw-evidence #32894150667 step:4:196) → apk add падает
# с "no such package". alpine:3.20 даёт community repo в /etc/apk/ и
# позволяет одной строкой поставить caddy + python3 + openssl + bash.
ARG RUNTIME_IMAGE=alpine:3.20

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
# Stage 2: Runtime — Caddy + Python aiohttp WSS-server (rob_box_quest)
# ============================================================
FROM ${RUNTIME_IMAGE}

# Caddy:2-alpine идёт с Caddy бинарём и пользователем caddy. Добавляем
# Caddy + Python в одном слое. alpine:3.20 community repo даёт и caddy, и
# python3 + py3-pip одной apk add. Параллельно тянем openssl (для
# self-signed серта), bash (entrypoint), curl (healthcheck Caddy), tini
# (PID 1 — корректный shutdown, вместо голого shell).
#
# --no-cache убирает /var/cache/apk → меньше размер слоя.
RUN apk add --no-cache \
        caddy \
        python3 \
        py3-pip \
        openssl \
        bash \
        curl \
        tini \
 && pip3 install --no-cache-dir --break-system-packages \
        aiohttp msgpack \
 || pip3 install --no-cache-dir aiohttp msgpack

# Статика WebXR-клиента из билдера
COPY --from=webxr-builder /build/dist /srv/quest_static

# Caddyfile + entry-script
COPY docker/vision/quest/Caddyfile       /etc/caddy/Caddyfile
COPY docker/vision/scripts/quest/start_quest.sh /scripts/start_quest.sh
RUN chmod +x /scripts/start_quest.sh

# /etc/caddy по умолчанию в alpine:3.20 уже создан (apk ставит туда
# caddy-config dir). Не пересоздаём — start_quest.sh пишет серты в /certs
# (volume), а Caddyfile лежит здесь.

# certs будут создаваться в /certs при первом старте контейнера.
# Pre-create чтобы том сразу был доступен и start_quest.sh не падал,
# если стартует первым.
RUN mkdir -p /certs

EXPOSE 8443

ENTRYPOINT ["/scripts/start_quest.sh"]
