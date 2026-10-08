#!/bin/bash
# ADR-0153 S7: приёмка стилей на роботе — по одному DJ-сету на стиль, запись 200 с с момента фразы
# (трек 1 вырезается офлайн по ROS-штампу `[music v2] started`), полный лог voice-assistant и supercollider.
#
# Где: Vision Pi, тяжёлые контейнеры уже погашены, hotpatch status = чистый образ.
# Как: flock -w 60 /tmp/music_test.lock bash s7_styles.sh [каталог=/tmp/s7]     (ONLY="club jazz" — подмножество)
# Ответы робота уходят в Telegram Шифу: фразы только «Робот, включи <стиль> сет на тему <тема>» и «Робот, стоп диджей».
# Если Шифу сам пишет роботу (`STT: [TG:495039871]` без «Клод»), его сет не прерываем: ждём 3 мин без его реплик.
set -u
D=${1:-/tmp/s7}; mkdir -p "$D"
SETS=(
"club|клубный|космос"
"rave|рейв|в пещере горного короля"
"synthwave|синтвейв|Моцарт"
"chiptune|восьмибитный|в пещере горного короля"
"breaks|брейкбит|космос"
"dnb|драм-н-бейс|Моцарт"
"lofi|лоуфай|космос"
"rock|рок|в пещере горного короля"
"grunge|гранж|космос"
"jazz|джаз|Моцарт"
)
ONLY="${ONLY:-}"
inj(){ docker exec -e T="$1" voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: \"$T\"}" >/dev/null 2>&1'; }
shifu_busy(){ docker logs voice-assistant --since "$1" 2>&1 | grep 'STT: \[TG:495039871\]' | grep -v 'Клод'; }
LAST=$(date -u -d '-3 min' +%Y-%m-%dT%H:%M:%SZ)
echo "=== START $(date -u +%FT%TZ)" | tee -a "$D/index.txt"
for row in "${SETS[@]}"; do
  IFS='|' read -r key word theme <<< "$row"
  [ -n "$ONLY" ] && ! echo " $ONLY " | grep -q " $key " && continue
  n=0
  while true; do
    b=$(shifu_busy "$LAST")
    [ -z "$b" ] && break
    echo "SHIFU_ACTIVE $(date -u +%TZ): $b" | tee -a "$D/index.txt"
    LAST=$(date -u +%Y-%m-%dT%H:%M:%SZ); sleep 180; n=$((n + 1))
    [ $n -ge 10 ] && { echo "ABORT_SHIFU" | tee -a "$D/index.txt"; exit 3; }
  done
  inj "Робот, стоп диджей"; sleep 12
  S=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)
  echo "=== SET $key '$word' '$theme' start $S" | tee -a "$D/index.txt"
  docker exec supercollider sh -c "rm -f /tmp/s7_$key.wav; jack_rec -f /tmp/s7_$key.wav -d 200 -b 16 jack:out_1 jack:out_2 >/dev/null 2>&1" &
  REC=$!
  sleep 1; echo "inject $(date -u +%T.%NZ)" | tee -a "$D/index.txt"
  inj "Робот, включи $word сет на тему $theme"
  wait $REC
  docker cp supercollider:/tmp/s7_$key.wav "$D/s7_$key.wav"; docker exec supercollider rm -f /tmp/s7_$key.wav
  docker logs -t voice-assistant --since "$S" > "$D/$key.full.log" 2>&1
  docker logs -t supercollider --since "$S" > "$D/$key.sc.log" 2>&1
  echo "=== SET $key done $(date -u +%TZ) started=$(grep -c '\[music v2\] started' "$D/$key.full.log") late=$(grep -cE ' late [0-9]' "$D/$key.sc.log")" | tee -a "$D/index.txt"
  LAST=$S
done
inj "Робот, стоп диджей"; sleep 5
echo "=== ALL DONE $(date -u +%TZ)" | tee -a "$D/index.txt"
