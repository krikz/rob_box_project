#!/bin/bash
# loudness_nrt_v2.sh — запуск scripts/music/loudness_nrt_v2.py на katana в одноразовом контейнере (issue #3422).
#
# Образ voice-assistant (arm64, на katana идёт через qemu-binfmt): в нём sclang + scsynth, numpy и renardo_lib
# 0.9.13 С ПАТЧАМИ робота (fix_brass_scd.py) — те же SynthDef-ы, что грузит робот. Образ supercollider
# sclang не содержит (только scsynth). Сэмплы 0_foxdot_default — в $WORK/samples (харнесс докачивает
# недостающие папки символов с collections.renardo.org).
#
#   scripts/music/loudness_nrt_v2.sh --regress
#   scripts/music/loudness_nrt_v2.sh --sweep lead sitar epiano
#   scripts/music/loudness_nrt_v2.sh --track 7
#
# REPO — корень чекаута (по умолчанию — этот), WORK — каталог рендеров и сэмплов (по умолчанию /tmp/nrt_v2).
set -euo pipefail
REPO=${REPO:-$(cd "$(dirname "$0")/../.." && pwd)}
WORK=${WORK:-/tmp/nrt_v2}
IMAGE=${IMAGE:-localhost:5000/krikz/rob_box:voice-assistant-humble-dev}
RENARDO=/usr/local/lib/python3.10/dist-packages/renardo_lib
mkdir -p "$WORK/out" "$WORK/samples"
exec docker run --rm --platform linux/arm64 --network host \
  -v "$REPO":/repo:ro -v "$WORK":/work -e PYTHONUTF8=1 -e QT_QPA_PLATFORM=offscreen \
  --entrypoint python3 "$IMAGE" /repo/scripts/music/loudness_nrt_v2.py \
  --renardo "$RENARDO" --samples /work/samples --out /work/out "$@"
