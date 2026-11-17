#!/usr/bin/env bash
# hailo_device_smoke.sh — «видит ли контейнер vision-hailo чип Hailo?»
#
# Вызывается из start_vision_hailo.sh при HAILO_ENABLED=true.
#
# Почему не только `hailortcli scan` (#3090): образ vision-hailo НЕ содержит
# hailortcli и никогда не содержал. hailortcli живёт в hailort_X.Y.Z_arm64.deb,
# а Dockerfile (ADR-0099 §2.2) ставит только wheel `hailort-4.24.0-cp310` +
# libhailort.so.4.24.0 — Python-биндинг hailo_platform. Поэтому старая проверка
# «hailortcli есть?» в контейнере всегда уходила в ветку WARN
# «hailortcli не установлен» (деплой-лог run 36687135688, 08:09:55), и
# устройство фактически не проверялось. hailortcli стоит на ХОСТЕ Vision Pi —
# там его уже зовёт scripts/setup/ensure_hailo_driver.sh.
#
# Порядок:
#   1. hailortcli есть в PATH → `hailortcli scan` (прежний контракт).
#   2. иначе → hailo_platform.Device.scan() (то, что образ реально содержит).
#   3. иначе (биндинга нет — сборка с HAILO_INSTALL_BINDING=none) → WARN,
#      устройство проверить нечем; degraded-режим уже залогирован выше.
#
# Exit codes:
#   0 — устройство видно, ИЛИ проверить нечем / API биндинга другой (WARN).
#   1 — проверка отработала и устройства нет (тот же исход, что и у
#       упавшего `hailortcli scan` раньше: HAT не виден → контейнер не стартует).
#
# Env:
#   HAILO_SMOKE_PYTHON — интерпретатор для шага 2 (default python3);
#                        переопределяется в unit-тестах.

set -uo pipefail

PY="${HAILO_SMOKE_PYTHON:-python3}"
TAG="[start_vision_hailo]"

if command -v hailortcli >/dev/null 2>&1; then
    echo "$TAG running hailortcli scan (smoke)..."
    if ! hailortcli scan; then
        echo "$TAG ERROR: hailortcli scan failed — HAT не виден?" >&2
        exit 1
    fi
    exit 0
fi

echo "$TAG hailortcli в образе нет (ставится только wheel hailo_platform, ADR-0099) — scan через hailo_platform.Device.scan()"

out="$("$PY" - <<'PY' 2>&1
import sys
try:
    from hailo_platform import Device
except ImportError as exc:
    print(f"binding-missing: {exc}")
    sys.exit(3)
try:
    ids = list(Device.scan())
except Exception as exc:  # noqa: BLE001 — любая вариация API/runtime → WARN
    print(f"scan-error: {type(exc).__name__}: {exc}")
    sys.exit(4)
if not ids:
    print("no-devices")
    sys.exit(1)
print(" ".join(str(i) for i in ids))
sys.exit(0)
PY
)"
rc=$?

case "$rc" in
    0)
        echo "$TAG Hailo device(s) visible via hailo_platform: $out"
        exit 0
        ;;
    1)
        echo "$TAG ERROR: hailo_platform.Device.scan() не нашёл устройств — HAT не виден? (/dev/hailo0, hailo_pcie на хосте, #3090)" >&2
        exit 1
        ;;
    3)
        echo "$TAG WARN: устройство проверить нечем: нет ни hailortcli, ни hailo_platform ($out)" >&2
        exit 0
        ;;
    *)
        echo "$TAG WARN: hailo_platform.Device.scan() не отработал (rc=$rc): $out" >&2
        exit 0
        ;;
esac
