#!/bin/bash
# Startup script для rob_box_quest (Vision Pi, ADR-0027, Phase 1.6)
#
# Ответственности:
#   1. Генерим self-signed TLS-сертификат, если отсутствует (DNS=quest.rob_box.local,
#      IP=10.1.1.11 — оба в SAN через subjectAltName).
#   2. Ждём Zenoh router (нужен для ROS2-стыка WSS-сервера).
#   3. Запускаем `caddy run` и `ros2 run rob_box_quest quest_node` оба в фоне.
#   4. PID 1 = этот shell; корректный shutdown на SIGTERM/SIGINT через trap
#      (вместо supervisord — вопрос Q2 карточки).
#
# PIN-генерация (ADR-0027 §4.5) живёт в rob_box_quest.quest_node (Python) —
# здесь только инфраструктурный слой, в логи контейнера будет уезжать строка
# PIN при рестарте.
#
# Источники истины:
#   - docs/adr/0027-meta-quest-ar-control.md §4.1, §4.5
#   - docker/vision/scripts/supervisor/start_supervisor.sh (паттерн Zenoh-wait)
#   - docker/vision/scripts/voice_assistant/start_voice_assistant.sh (env)

set -euo pipefail

echo "=========================================="
echo "  rob_box_quest Starting (Phase 1.6, ADR-0027)"
echo "=========================================="

# Source ROS2 (humble по умолчанию; переопределяется через ENV сборки)
# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

# Если rob_box_quest когда-то войдёт в общий workspace (вместо pip-пакета),
# подхватим его. Сейчас rob_box_quest — отдельный Python-пакет, ставится
# через pip; поэтому /ws/install/setup.bash может отсутствовать.
if [ -f /ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /ws/install/setup.bash
fi

# ----------------------------------------------------------
# 1. Self-signed certificate generation
# ----------------------------------------------------------
CERT_DIR="${CERT_DIR:-/certs}"
CERT_FILE="${CERT_DIR}/selfsigned.crt"
KEY_FILE="${CERT_DIR}/selfsigned.key"
QUEST_HOST="${QUEST_HOST:-quest.rob_box.local}"
QUEST_IP="${QUEST_IP:-10.1.1.11}"
CERT_DAYS="${CERT_DAYS:-365}"

mkdir -p "${CERT_DIR}"

if [ ! -f "${CERT_FILE}" ] || [ ! -f "${KEY_FILE}" ]; then
    echo "Генерируем self-signed TLS-сертификат: ${QUEST_HOST} (${QUEST_IP}), ${CERT_DAYS} дней"
    openssl req -x509 -newkey rsa:2048 -days "${CERT_DAYS}" -nodes \
        -subj "/CN=${QUEST_HOST}" \
        -addext "subjectAltName=DNS:${QUEST_HOST},IP:${QUEST_IP}" \
        -keyout "${KEY_FILE}" \
        -out    "${CERT_FILE}" \
        2>/dev/null
    chmod 600 "${KEY_FILE}"
    chmod 644 "${CERT_FILE}"
    echo "✓ Сертификат создан: ${CERT_FILE}"
    echo "  Импортируйте его в Meta Quest: Settings → Privacy → Security → Trusted Sources."
else
    echo "✓ Сертификат уже существует: ${CERT_FILE}"
fi

# ----------------------------------------------------------
# 2. Zenoh router wait
# ----------------------------------------------------------
echo "Ожидание Zenoh router..."
ZENOH_RETRIES=${ZENOH_ROUTER_CHECK_ATTEMPTS:-10}
ZENOH_RETRY=0

while [ $ZENOH_RETRY -lt $ZENOH_RETRIES ]; do
    if wget -qO- http://localhost:8000/@/local/router > /dev/null 2>&1; then
        echo "✓ Zenoh router доступен"
        break
    fi
    echo "Попытка $((ZENOH_RETRY + 1))/${ZENOH_RETRIES}..."
    sleep 2
    ZENOH_RETRY=$((ZENOH_RETRY + 1))
done

if [ $ZENOH_RETRY -eq $ZENOH_RETRIES ]; then
    echo "⚠ Zenoh router недоступен после ${ZENOH_RETRIES} попыток"
    echo "  quest_node запустится; Zenoh-сессия переподключится автоматически,"
    echo "  когда Zenoh поднимется (Phase 1.4 стрим-провайдер)."
fi

# ----------------------------------------------------------
# 3. Caddy в фоне
# ----------------------------------------------------------
echo ""
echo "Запуск Caddy (HTTPS-frontend, :8443)..."
CADDY_LOG=/var/log/caddy-quest.log
: > "${CADDY_LOG}"
caddy run --config /etc/caddy/Caddyfile >"${CADDY_LOG}" 2>&1 &
CADDY_PID=$!
echo "✓ Caddy запущен (pid=${CADDY_PID}, logs: ${CADDY_LOG})"

# Cleanup hook — вызывается на TERM/INT от compose `docker stop`.
cleanup() {
    echo ""
    echo "Получен сигнал shutdown, останавливаем процессы..."
    if kill -0 "${QUEST_PID:-0}" 2>/dev/null; then
        kill -TERM "${QUEST_PID}" 2>/dev/null || true
    fi
    if kill -0 "${CADDY_PID:-0}" 2>/dev/null; then
        kill -TERM "${CADDY_PID}" 2>/dev/null || true
    fi
    # Дадим процессам 5 сек на graceful shutdown, потом принудительно
    sleep 5
    kill -KILL "${QUEST_PID:-0}" 2>/dev/null || true
    kill -KILL "${CADDY_PID:-0}" 2>/dev/null || true
    wait 2>/dev/null || true
    echo "✓ rob_box_quest остановлен"
    exit 0
}
trap cleanup TERM INT

# ----------------------------------------------------------
# 4. quest_node в фоне (ждите как foreground-процесс PID 1)
# ----------------------------------------------------------
echo ""
echo "=========================================="
echo "  Запуск ros2 run rob_box_quest quest_node"
echo "=========================================="

ros2 run rob_box_quest quest_node \
    --ros-args \
    -p quest_host:="${QUEST_HOST}" \
    -p cert_file:="${CERT_FILE}" \
    &
QUEST_PID=$!

# Если quest_node умер сразу (например, пакет не установлен) — не держим
# контейнер с мёртвым PID 1, а уходим в cleanup.
if ! kill -0 "${QUEST_PID}" 2>/dev/null; then
    echo "✗ quest_node завершился сразу, см. логи ROS2"
    cleanup
fi

# Ждём quest_node как PID 1 (Caddy уже в фоне с CADDY_PID).
# Если quest_node умрёт — wait вернёт ненулевой код; cleanup сам погасит Caddy.
wait "${QUEST_PID}"
QUEST_EXIT=$?

echo "quest_node завершился с кодом ${QUEST_EXIT}"
cleanup
