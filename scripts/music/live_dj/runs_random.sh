#!/bin/bash
# на Vision Pi: nohup bash /tmp/cc_runs_random.sh <каталог> [seed] &
# Как runs.sh, но 5 СЛУЧАЙНЫХ мелодий из RTTTL-архива (ADR-0149 §7.1, A16): тема = название мелодии.
# Сид выбора печатается и пишется в index.txt первой строкой - публиковать вместе с результатом.
# Темы выбирает diversity.py --pick (хост или контейнер voice-assistant с rob_box_mcp_tools); можно задать вручную:
#   THEMES_FILE=/tmp/themes.txt (по теме на строку, 5 строк).
D="${1:?usage: runs_random.sh <result_dir> [seed]}"; SEED="${2:-$(date +%s)}"
rm -rf "$D"; mkdir -p "$D"
if [ -n "$THEMES_FILE" ]; then
  mapfile -t THEMES < "$THEMES_FILE"
else
  mapfile -t THEMES < <(docker exec -e PYTHONUTF8=1 voice-assistant bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/music/live_dj/diversity.py --pick 5 --seed $SEED")
fi
[ "${#THEMES[@]}" -eq 5 ] || { echo "нужно 5 тем, получено ${#THEMES[@]}" >&2; exit 1; }
echo "=== SEED $SEED themes: ${THEMES[*]}" | tee -a "$D"/index.txt
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
