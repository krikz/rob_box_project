#!/bin/bash
# kicks_probe.sh — замер бочек-кандидатов на роботе (ADR-0152 PR-4, §3.4). По образцу замера 02.10 (KickSound):
# одна бочка, 4/4 @130, запись jack_rec 8 с на каждый номер сэмпла X:N. Доли < 250 / < 120 Гц, RMS и пик считает
# kicks_probe.py на хосте по скачанным wav — на роботе нет numpy.
#
# Где: Vision Pi (ros2@10.1.1.21). Пишет ТОЛЬКО в /tmp хоста; в контейнеры файлы не копирует (скрипт
# live_check_mcp_call.py подаётся в voice-assistant через stdin, запись jack_rec лежит во /tmp supercollider и
# удаляется). Тяжёлые контейнеры гасит на время замера и поднимает обратно (политика стенда).
#
#   scp scripts/music/kicks_probe.sh scripts/music/live_check_mcp_call.py ros2@10.1.1.21:/tmp/
#   ssh ros2@10.1.1.21 'flock -w 900 /tmp/music_test.lock bash /tmp/kicks_probe.sh /tmp/kicks_probe 12 3 14 18 22 24 30 32'
#   scp -r ros2@10.1.1.21:/tmp/kicks_probe ./kicks_probe && python scripts/music/kicks_probe.py kicks_probe/kick_*.wav
#
# Эталон — X:12 (House_GhostFader): уровень остальных меряют против него.
set -u
OUT=${1:?каталог результата}; shift
SAMPLES=${*:?номера сэмплов X:N}
BPM=${BPM:-130}; SECS=${SECS:-8}
VAI="docker exec -i voice-assistant bash -c"
ROS='source /opt/ros/humble/setup.bash; source /ws/install/setup.bash'
mkdir -p "$OUT"

call() {  # call <tool> <json>
  $VAI "$ROS; python3 - $1 $(printf '%s' "$2" | base64 -w 0) 40" < /tmp/live_check_mcp_call.py
}

up=$(docker ps --format '{{.Names}}' | grep -xE 'oak-d|rob-box-quest|vision-face' || true)
[ -n "$up" ] && { echo "гашу: $up"; docker stop $up >/dev/null; }
restore() {
  [ -n "$up" ] && docker start $up >/dev/null
  sleep 25
  docker ps --format '{{.Names}} {{.Status}}' | grep -E 'oak-d|rob-box-quest|vision-face'
}
trap restore EXIT

call stop_music '{}' >/dev/null
for n in $SAMPLES; do
  code="Clock.bpm = $BPM\nk1 >> play('X...X...X...X...', dur=1/4, sample=$n)"
  docker exec supercollider sh -c "rm -f /tmp/kick_$n.wav; jack_rec -f /tmp/kick_$n.wav -d $SECS -b 16 jack:out_1 jack:out_2 >/dev/null 2>&1" &
  REC=$!
  sleep 1
  call execute_music_code "{\"code\":\"$code\",\"pattern_name\":\"kick_probe\"}" | tee "$OUT/call_$n.json" | cut -c1-200
  wait $REC
  call stop_music '{}' >/dev/null
  docker exec supercollider cat /tmp/kick_$n.wav > "$OUT/kick_$n.wav"
  docker exec supercollider rm -f /tmp/kick_$n.wav
  sleep 1
done
ls -l "$OUT"
