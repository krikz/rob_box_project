#!/usr/bin/env bash
# pre_build-хук `fetch_hailort_wheel` (vision-hailo, ADR-0099 §2.2).
#
# docs/plans/2026-09-15-service-manifest.md §2.5: манифест
# docker/build-manifest.yaml хранит ТОЛЬКО факт «у сервиса есть предсборочный
# хук» (pre_build: fetch_hailort_wheel) — императивная логика (два пути,
# fallback, текст ошибки) остаётся кодом, а не данными. Раньше этот код жил
# прямо в build-vision-hailo job'е; после перехода на matrix шаблон job'а
# один на все сервисы, поэтому тело хука переехало в отдельный скрипт —
# дословно, без изменений в поведении.

set -e

mkdir -p docker/vision/vision-hailo/wheels
# Канонично — /opt/rob_box/vendor. Fallback — /tmp (host /tmp проброшен
# в runner'ы как /tmp:/tmp). Нужны ОБА: wheel (python) + libhailort.so.
for f in hailort-4.24.0-cp310-cp310-linux_aarch64.whl libhailort.so.4.24.0; do
  if [ -f "/opt/rob_box/vendor/${f}" ]; then
    cp "/opt/rob_box/vendor/${f}" docker/vision/vision-hailo/wheels/
  elif [ -f "/tmp/${f}" ]; then
    cp "/tmp/${f}" docker/vision/vision-hailo/wheels/
  else
    echo "::error::${f} не найден ни в /opt/rob_box/vendor, ни в /tmp — real inference build невозможен" >&2
    echo "::error::Это закрытый артефакт Hailo Developer Zone (нужен аккаунт), скриптом не качается." >&2
    echo "::error::Что делать: docs/deployment/hailo-vendor-artifacts.md — §2 скачать, §3 извлечь .so из .deb, §4.1 положить в /opt/rob_box/vendor на этом build-хосте." >&2
    exit 1
  fi
done
ls -la docker/vision/vision-hailo/wheels/
