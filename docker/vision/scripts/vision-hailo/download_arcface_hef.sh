#!/usr/bin/env bash
# download_arcface_hef.sh — скачать arcface_mobilefacenet.hef (ADR-0123, issue #2599 PR-B).
#
# ⚠️ ЗАМЕНЕНО РЕСУРСНЫМ ПАКОМ (Этап 2, docs/plans/2026-09-15-resource-pack.md).
#    Деплой (`L-Deploy and Verify.yml`, шаг «[Vision Pi] Ensure HEF models»)
#    этот скрипт больше НЕ вызывает — вместо него
#    docker/vision/scripts/resource_pack/apply_resource_pack.sh
#    --only yolov8n-hef,retinaface-hef,arcface-hef (запись arcface-hef в
#    manifest.yaml, тот же sha256, сверяется тестом
#    tests/unit/docker/test_resource_pack.py).
#    Скрипт оставлен на один релиз-цикл как быстрый откат (план §7 Этап 2 п.2)
#    и подлежит удалению Этапом 5 после N успешных прогонов деплоя.
#    Ручной откат: вызвать этот скрипт напрямую на Vision Pi.
#
# Issue #2599 PR-B: ArcFace-эмбеддинги для узнавания лиц (ADR-0123). Скрипт —
# 1:1 по образцу download_retinaface_hef.sh (issue #2599 PR-A):
#
#   1. Проверяет наличие .hef в указанном пути (default
#      /opt/rob_box/models/arcface_mobilefacenet.hef).
#   2. Если файла нет — скачивает из hailo_model_zoo (Hailo официальный
#      репозиторий, модель публичная — аккаунт Developer Zone не нужен),
#      проверяет SHA256, кладёт в нужный путь.
#   3. Если скачивание недоступно (нет сети, нет curl) — печатает
#      понятную инструкцию, не падает в тишине.
#
# SHA256 и размер зафиксированы вручную на живом роботе 22.09.2026, после
# ручного копирования HEF в /opt/rob_box/models/ (issue #2599 PR-B). Реальные
# тензоры устройства: вход (112,112,3) UINT8, выход fc1 (512,) UINT8 —
# эмбеддинг 512-dim, а НЕ 128 (см. embedding_dim в
# docker/vision/config/hailo_models.yaml — 128 было ошибкой со страницы
# model zoo, унаследованной ADR-0089/ADR-0106/ADR-0123).
#
# Использование:
#   ./download_arcface_hef.sh                        # /opt/rob_box/models/arcface_mobilefacenet.hef
#   HEF_DIR=/tmp/hef ./download_arcface_hef.sh        # custom path
#   HEF_FORCE_DOWNLOAD=1 ./download_arcface_hef.sh    # always re-download
#
# Touchpoints:
# - ADR-0123 §3/§4 (эмбеддинг встречи, лицевой трекер).
# - Issue #2599 PR-B.

set -eo pipefail

# ---------- defaults ----------
HEF_DIR="${HEF_DIR:-/opt/rob_box/models}"
HEF_FILE="${HEF_DIR}/arcface_mobilefacenet.hef"
HEF_URL="${HEF_URL:-https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/arcface_mobilefacenet.hef}"
# SHA256 снят вручную на Vision Pi 22.09.2026 (issue #2599 PR-B, 3505142 байт).
HEF_SHA256="${HEF_SHA256:-c75fc63241383f7b346db54e3fa5d1cc89b85799152f5121ed0fcca9c057ddc7}"

# ---------- проверки окружения ----------
if [ -f "${HEF_FILE}" ] && [ "${HEF_FORCE_DOWNLOAD:-0}" != "1" ]; then
    echo "[download_arcface_hef] OK: ${HEF_FILE} уже существует ($(stat -c%s "${HEF_FILE}" 2>/dev/null || stat -f%z "${HEF_FILE}") байт)"
    exit 0
fi

if ! command -v curl > /dev/null 2>&1 && ! command -v wget > /dev/null 2>&1; then
    echo "[download_arcface_hef] ERROR: ни curl, ни wget не найдены." >&2
    echo "[download_arcface_hef] Установите HEF вручную:" >&2
    echo "[download_arcface_hef]   1) Скачайте ${HEF_URL}" >&2
    echo "[download_arcface_hef]   2) Положите в ${HEF_FILE}" >&2
    exit 1
fi

# ---------- download ----------
mkdir -p "${HEF_DIR}"
TMP_FILE=$(mktemp /tmp/arcface_mobilefacenet.hef.XXXXXX)
trap 'rm -f "${TMP_FILE}"' EXIT

echo "[download_arcface_hef] Скачиваю ${HEF_URL} → ${TMP_FILE}"
if command -v curl > /dev/null 2>&1; then
    curl -fL --retry 3 --retry-delay 2 -o "${TMP_FILE}" "${HEF_URL}" || {
        echo "[download_arcface_hef] ERROR: curl download failed" >&2
        exit 1
    }
else
    wget -O "${TMP_FILE}" "${HEF_URL}" || {
        echo "[download_arcface_hef] ERROR: wget download failed" >&2
        exit 1
    }
fi

# ---------- SHA256 check ----------
# Сравниваем в нижнем регистре: sha256sum печатает hex строчными, а ENV
# может прийти в любом регистре — на этом уже один раз сгорел
# download_retinaface_hef.sh (#2599 PR-A, заглавный HEF_SHA256 не совпадал
# НИКОГДА, и скрипт удалял корректно скачанный файл с "SHA256 mismatch").
# Норма для обеих сторон, чтобы формат источника не решал.
if [ -n "${HEF_SHA256}" ]; then
    ACTUAL_SHA=$(sha256sum "${TMP_FILE}" | cut -d' ' -f1 | tr 'A-Z' 'a-z')
    EXPECTED_SHA=$(printf '%s' "${HEF_SHA256}" | tr 'A-Z' 'a-z')
    if [ "${ACTUAL_SHA}" != "${EXPECTED_SHA}" ]; then
        echo "[download_arcface_hef] ERROR: SHA256 mismatch" >&2
        echo "[download_arcface_hef]   expected: ${HEF_SHA256}" >&2
        echo "[download_arcface_hef]   actual:   ${ACTUAL_SHA}" >&2
        exit 1
    fi
    echo "[download_arcface_hef] SHA256 OK"
fi

# ---------- move ----------
mv "${TMP_FILE}" "${HEF_FILE}"
trap - EXIT
echo "[download_arcface_hef] OK: ${HEF_FILE} готов ($(stat -c%s "${HEF_FILE}" 2>/dev/null || stat -f%z "${HEF_FILE}") байт)"
