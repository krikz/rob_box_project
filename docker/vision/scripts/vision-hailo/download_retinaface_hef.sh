#!/usr/bin/env bash
# download_retinaface_hef.sh — скачать retinaface_mobilenet_v1.hef (ADR-0089 Phase 2).
#
# ⚠️ ЗАМЕНЕНО РЕСУРСНЫМ ПАКОМ (Этап 2, docs/plans/2026-09-15-resource-pack.md).
#    Деплой (`L-Deploy and Verify.yml`, шаг «[Vision Pi] Ensure HEF models»)
#    этот скрипт больше НЕ вызывает — вместо него
#    docker/vision/scripts/resource_pack/apply_resource_pack.sh
#    --only yolov8n-hef,retinaface-hef (запись retinaface-hef в manifest.yaml,
#    тот же sha256, сверяется тестом tests/unit/docker/test_resource_pack.py).
#    Скрипт оставлен на один релиз-цикл как быстрый откат (план §7 Этап 2 п.2)
#    и подлежит удалению Этапом 5 после N успешных прогонов деплоя.
#    Ручной откат: вызвать этот скрипт напрямую на Vision Pi.
#
# Issue #2599 PR-A: лицевая детекция без идентификации. Скрипт — 1:1 по образцу
# download_yolov8n_hef.sh (ADR-0104, issue #2531 acceptance #10):
#
#   1. Проверяет наличие .hef в указанном пути (default
#      /opt/rob_box/models/retinaface_mobilenet_v1.hef).
#   2. Если файла нет — скачивает из hailo_model_zoo (Hailo официальный
#      репозиторий), проверяет SHA256, кладёт в нужный путь.
#   3. Если скачивание недоступно (нет сети, нет curl) — печатает
#      понятную инструкцию, не падает в тишине.
#
# SHA256 зафиксирован для compiled HEF v2.14.0/hailo8 (5993660 байт).
#
# Использование:
#   ./download_retinaface_hef.sh                     # /opt/rob_box/models/retinaface_mobilenet_v1.hef
#   HEF_DIR=/tmp/hef ./download_retinaface_hef.sh    # custom path
#   HEF_FORCE_DOWNLOAD=1 ./download_retinaface_hef.sh  # always re-download
#
# Touchpoints:
# - ADR-0089 §3 touchpoint #11 (HEF lifecycle).
# - Issue #2599 PR-A.

set -eo pipefail

# ---------- defaults ----------
HEF_DIR="${HEF_DIR:-/opt/rob_box/models}"
HEF_FILE="${HEF_DIR}/retinaface_mobilenet_v1.hef"
HEF_URL="${HEF_URL:-https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/retinaface_mobilenet_v1.hef}"
# SHA256 скомпилированного HEF v2.14.0/hailo8 (5993660 байт), зафиксирован
# для issue #2599 PR-A. В отличие от yolov8n-скрипта — НЕ пустой.
HEF_SHA256="${HEF_SHA256:-1FBC7BE2554CCEBA18986CEFA73983A59057AC5457BC75173FB2E09457BFF472}"

# ---------- проверки окружения ----------
if [ -f "${HEF_FILE}" ] && [ "${HEF_FORCE_DOWNLOAD:-0}" != "1" ]; then
    echo "[download_retinaface_hef] OK: ${HEF_FILE} уже существует ($(stat -c%s "${HEF_FILE}" 2>/dev/null || stat -f%z "${HEF_FILE}") байт)"
    exit 0
fi

if ! command -v curl > /dev/null 2>&1 && ! command -v wget > /dev/null 2>&1; then
    echo "[download_retinaface_hef] ERROR: ни curl, ни wget не найдены." >&2
    echo "[download_retinaface_hef] Установите HEF вручную:" >&2
    echo "[download_retinaface_hef]   1) Скачайте ${HEF_URL}" >&2
    echo "[download_retinaface_hef]   2) Положите в ${HEF_FILE}" >&2
    exit 1
fi

# ---------- download ----------
mkdir -p "${HEF_DIR}"
TMP_FILE=$(mktemp /tmp/retinaface_mobilenet_v1.hef.XXXXXX)
trap 'rm -f "${TMP_FILE}"' EXIT

echo "[download_retinaface_hef] Скачиваю ${HEF_URL} → ${TMP_FILE}"
if command -v curl > /dev/null 2>&1; then
    curl -fL --retry 3 --retry-delay 2 -o "${TMP_FILE}" "${HEF_URL}" || {
        echo "[download_retinaface_hef] ERROR: curl download failed" >&2
        exit 1
    }
else
    wget -O "${TMP_FILE}" "${HEF_URL}" || {
        echo "[download_retinaface_hef] ERROR: wget download failed" >&2
        exit 1
    }
fi

# ---------- SHA256 check ----------
# Сравниваем в нижнем регистре: sha256sum печатает hex строчными, а прибитый
# HEF_SHA256 записан заглавными — прямое сравнение строк не совпадало НИКОГДА,
# и скрипт удалял корректно скачанный файл с "SHA256 mismatch" (одинаковый hex,
# разный регистр). Норма для обеих сторон, чтобы формат источника не решал.
if [ -n "${HEF_SHA256}" ]; then
    ACTUAL_SHA=$(sha256sum "${TMP_FILE}" | cut -d' ' -f1 | tr 'A-Z' 'a-z')
    EXPECTED_SHA=$(printf '%s' "${HEF_SHA256}" | tr 'A-Z' 'a-z')
    if [ "${ACTUAL_SHA}" != "${EXPECTED_SHA}" ]; then
        echo "[download_retinaface_hef] ERROR: SHA256 mismatch" >&2
        echo "[download_retinaface_hef]   expected: ${HEF_SHA256}" >&2
        echo "[download_retinaface_hef]   actual:   ${ACTUAL_SHA}" >&2
        exit 1
    fi
    echo "[download_retinaface_hef] SHA256 OK"
fi

# ---------- move ----------
mv "${TMP_FILE}" "${HEF_FILE}"
trap - EXIT
echo "[download_retinaface_hef] OK: ${HEF_FILE} готов ($(stat -c%s "${HEF_FILE}" 2>/dev/null || stat -f%z "${HEF_FILE}") байт)"
