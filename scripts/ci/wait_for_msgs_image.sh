#!/usr/bin/env bash
# Ждёт, пока общий образ ROS-сообщений появится в локальном registry.
#
# Образ (docker/msgs/Dockerfile) собирает ОДИН job — "L: Build Msgs Image" —
# параллельно со всеми сервисами. Ждут его только сервисы с
# `msgs_image: true` в docker/build-manifest.yaml (их Dockerfile делает
# COPY --from=${MSGS_IMAGE}); остальные начинают сборку сразу. Раньше на
# job msgs через needs: висели все Pi и вся волна 0 — узкое горлышко.
#
# Использование:
#   wait_for_msgs_image.sh <registry> <image>
#     registry — хост:порт (localhost:5000)
#     image    — полная ссылка (localhost:5000/krikz/rob_box:msgs-humble-<хэш>)
# Окружение:
#   MSGS_WAIT_TIMEOUT  — сколько ждать, с (по умолчанию 2700 = 45 мин:
#                        холодная сборка msgs под qemu — до ~25 мин)
#   MSGS_WAIT_INTERVAL — период опроса, с (по умолчанию 20)
#
# Коды возврата: 0 — образ в registry; 1 — не дождались (упала сборка msgs?).
set -euo pipefail

if [ "$#" -ne 2 ]; then
    echo "usage: $(basename "$0") <registry> <image>" >&2
    exit 1
fi

REGISTRY="$1"
IMAGE="$2"
TAG="${IMAGE##*:}"
REPO="${IMAGE#*/}"
REPO="${REPO%:*}"
TIMEOUT="${MSGS_WAIT_TIMEOUT:-2700}"
INTERVAL="${MSGS_WAIT_INTERVAL:-20}"
VERIFY="$(dirname "$0")/verify-image-in-registry.sh"

start=$SECONDS
while :; do
    if bash "$VERIFY" "$REGISTRY" "$TAG" "$REPO" >/dev/null 2>&1; then
        echo "✅ ${IMAGE} в registry (ждали $((SECONDS - start)) с)"
        exit 0
    fi
    if [ $((SECONDS - start)) -ge "$TIMEOUT" ]; then
        echo "::error::${IMAGE} так и не появился в registry за ${TIMEOUT} с — смотри job build-msgs (\"L: Build Msgs Image\") этого прогона" >&2
        exit 1
    fi
    echo "⏳ жду ${IMAGE} ($((SECONDS - start)) с из ${TIMEOUT})"
    sleep "$INTERVAL"
done
