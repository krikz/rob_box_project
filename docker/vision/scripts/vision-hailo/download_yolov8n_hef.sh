#!/usr/bin/env bash
# download_yolov8n_hef.sh — скачать yolov8n.hef из hailo_model_zoo (ADR-0104).
#
# ⚠️ ЗАМЕНЕНО РЕСУРСНЫМ ПАКОМ (Этап 2, docs/plans/2026-09-15-resource-pack.md).
#    Деплой (`L-Deploy and Verify.yml`, шаг «[Vision Pi] Ensure HEF models»)
#    этот скрипт больше НЕ вызывает — вместо него
#    docker/vision/scripts/resource_pack/apply_resource_pack.sh
#    --only yolov8n-hef,retinaface-hef (запись yolov8n-hef в manifest.yaml).
#    Скрипт оставлен на один релиз-цикл как быстрый откат (план §7 Этап 2 п.2)
#    и подлежит удалению Этапом 5 после N успешных прогонов деплоя.
#    Ручной откат: вызвать этот скрипт напрямую на Vision Pi.
#
# Issue #2531 acceptance #10: до этого фикса HEF lifecycle был полностью
# ручной ("если /opt/rob_box/models/yolov8n.hef не существует, compose
# упадёт"). Этот скрипт:
#
#   1. Проверяет наличие .hef в указанном пути (default
#      /opt/rob_box/models/yolov8n.hef).
#   2. Если файла нет — скачивает из hailo_model_zoo (Hailo официальный
#      репозиторий), проверяет SHA256, кладёт в нужный путь.
#   3. Если скачивание недоступно (нет сети, нет curl) — печатает
#      понятную инструкцию, не падает в тишине.
#
# Использование:
#   ./download_yolov8n_hef.sh                     # /opt/rob_box/models/yolov8n.hef
#   HEF_DIR=/tmp/hef ./download_yolov8n_hef.sh    # custom path
#   HEF_FORCE_DOWNLOAD=1 ./download_yolov8n_hef.sh  # always re-download
#
# Touchpoints:
# - ADR-0089 §3 touchpoint #11 (HEF lifecycle).
# - ADR-0104 (HEF lifecycle script, issue #2531 acceptance #10).
# - docker/vision/docker-compose.yaml:677-680 (монтирование /opt/rob_box/models).

set -eo pipefail

# ---------- defaults ----------
HEF_DIR="${HEF_DIR:-/opt/rob_box/models}"
HEF_FILE="${HEF_DIR}/yolov8n.hef"
HEF_URL="${HEF_URL:-https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/yolov8n.hef}"
HEF_SHA256="${HEF_SHA256:-}"  # Пусто = пропустить проверку. Hailo не публикует
                              # официальный SHA для compiled .hef; для CI можно
                              # проставить зафиксированный хеш в переменную.

# ---------- проверки окружения ----------
if [ -f "${HEF_FILE}" ] && [ "${HEF_FORCE_DOWNLOAD:-0}" != "1" ]; then
    echo "[download_hef] OK: ${HEF_FILE} уже существует ($(stat -c%s "${HEF_FILE}" 2>/dev/null || stat -f%z "${HEF_FILE}") байт)"
    exit 0
fi

if ! command -v curl > /dev/null 2>&1 && ! command -v wget > /dev/null 2>&1; then
    echo "[download_hef] ERROR: ни curl, ни wget не найдены." >&2
    echo "[download_hef] Установите HEF вручную:" >&2
    echo "[download_hef]   1) Скачайте ${HEF_URL}" >&2
    echo "[download_hef]   2) Положите в ${HEF_FILE}" >&2
    echo "[download_hef]   3) Убедитесь что docker-compose.yaml:677-680" >&2
    echo "[download_hef]      монтирует эту директорию в vision-hailo." >&2
    exit 1
fi

# ---------- download ----------
mkdir -p "${HEF_DIR}"
TMP_FILE=$(mktemp /tmp/yolov8n.hef.XXXXXX)
trap 'rm -f "${TMP_FILE}"' EXIT

echo "[download_hef] Скачиваю ${HEF_URL} → ${TMP_FILE}"
if command -v curl > /dev/null 2>&1; then
    curl -fL --retry 3 --retry-delay 2 -o "${TMP_FILE}" "${HEF_URL}" || {
        echo "[download_hef] ERROR: curl download failed" >&2
        exit 1
    }
else
    wget -O "${TMP_FILE}" "${HEF_URL}" || {
        echo "[download_hef] ERROR: wget download failed" >&2
        exit 1
    }
fi

# ---------- SHA256 check (опционально) ----------
if [ -n "${HEF_SHA256}" ]; then
    # Нижний регистр с обеих сторон — см. download_retinaface_hef.sh: там
    # заглавный прибитый хэш ронял скачивание при верном содержимом файла.
    ACTUAL_SHA=$(sha256sum "${TMP_FILE}" | cut -d' ' -f1 | tr 'A-Z' 'a-z')
    EXPECTED_SHA=$(printf '%s' "${HEF_SHA256}" | tr 'A-Z' 'a-z')
    if [ "${ACTUAL_SHA}" != "${EXPECTED_SHA}" ]; then
        echo "[download_hef] ERROR: SHA256 mismatch" >&2
        echo "[download_hef]   expected: ${HEF_SHA256}" >&2
        echo "[download_hef]   actual:   ${ACTUAL_SHA}" >&2
        exit 1
    fi
    echo "[download_hef] SHA256 OK"
fi

# ---------- move ----------
mv "${TMP_FILE}" "${HEF_FILE}"
trap - EXIT
echo "[download_hef] OK: ${HEF_FILE} готов ($(stat -c%s "${HEF_FILE}" 2>/dev/null || stat -f%z "${HEF_FILE}") байт)"
