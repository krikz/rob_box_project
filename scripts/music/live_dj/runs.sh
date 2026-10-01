#!/bin/bash
# на Vision Pi: nohup bash /tmp/cc_runs.sh &   — 5 DJ-сетов по 360 с, полная запись
D="${1:?usage: runs.sh <result_dir>}"; rm -rf "$D"; mkdir -p "$D"
THEMES=("славянская вечеринка" "космос" "славянская вечеринка" "киберпанк" "детский праздник")
inj(){ docker exec -e T="$1" voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 topic pub --once /voice/stt/result std_msgs/msg/String "{data: \"$T\"}" >/dev/null 2>&1'; }
i=0
for T in "${THEMES[@]}"; do
  i=$((i+1))
  inj "Робот останови музыку"; sleep 15
  S=$(date -u +%Y-%m-%dT%H:%M:%SZ); echo "=== SET $i '$T' start $S" >> "$D"/index.txt
  docker exec supercollider sh -c "rm -f /tmp/r$i.wav; jack_rec -f /tmp/r$i.wav -d 360 -b 16 jack:out_1 jack:out_2 >/dev/null 2>&1" &
  sleep 1; inj "Робот включи диджей сет на тему $T"
  wait
  docker cp supercollider:/tmp/r$i.wav "$D"/r$i.wav; docker exec supercollider rm -f /tmp/r$i.wav
  docker logs voice-assistant --since "$S" > "$D"/set$i.full.log 2>&1
  echo "=== SET $i done $(date -u +%H:%M:%SZ)" >> "$D"/index.txt
done
inj "Робот останови музыку"
echo "=== ALL DONE $(date -u +%H:%M:%SZ)" >> "$D"/index.txt
