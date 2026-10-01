#!/bin/bash
# Приёмка PR-5 ADR-0149 (эпик #3312): DJ-сет движка v2 на роботе — A1 (тишина ≤ 5 с), A3 (DJ_AUTO = 0),
# A5 (один темп), переходы по nearly_finished, artifact_stale только при смене трека.
#
# Где: Vision Pi, ПОСЛЕ пересборки образа с PR-5 и с music_engine: "v2" (секция /** в
# docker/vision/config/voice_assistant/mcp_server.yaml → рестарт voice-assistant).
# Как: flock -w 900 /tmp/music_test.lock bash dj_set_v2.sh <каталог> [секунд=1200] [тема=космос]
# Нужны рядом: accept.py, compare.py, audit_wav.py (эта папка) и ../live_check_mcp_call.py.
# TG_CHAT_ID=<chat> — запись уйдёт Шифу голосовым через tgogg.sh.
set -u
OUT=${1:?каталог результата}; DUR=${2:-1200}; THEME=${3:-космос}
HERE=$(cd "$(dirname "$0")" && pwd)
VA="docker exec voice-assistant bash -c"
ROS='source /opt/ros/humble/setup.bash; source /ws/install/setup.bash'
mkdir -p "$OUT"
rms() { python3 - "$1" <<'PY'
import sys, wave, array, math
w = wave.open(sys.argv[1]); a = array.array("h", w.readframes(w.getnframes()))
s = sum(x * x for x in a) / max(1, len(a)); print(-180 if s == 0 else round(10 * math.log10(s / 32768 ** 2), 1))
PY
}
rec() { docker exec supercollider sh -c "rm -f /tmp/$1.wav; jack_rec -f /tmp/$1.wav -d $2 -b 16 jack:out_1 jack:out_2 >/dev/null 2>&1"
        docker cp supercollider:/tmp/"$1".wav "$OUT/$1.wav"; docker exec supercollider rm -f /tmp/"$1".wav; }
call() {  # call <json> — dj_set по /mcp/execute (подпись harness), ответ одной строкой JSON
  $VA "$ROS; python3 - dj_set $(printf '%s' "$1" | base64 -w 0) 60" < "$HERE/../live_check_mcp_call.py"
}

for node in "${MCP_NODE:-/mcp_server}" "${DIALOGUE_NODE:-/dialogue_node}"; do
  v=$($VA "$ROS; ros2 param get $node music_engine" 2>&1 | tail -1)
  echo "$node music_engine: $v" | tee -a "$OUT/summary.txt"
  case "$v" in *v2*) ;; *) echo "СТОП: $node не на v2 — прогон не имеет смысла" | tee -a "$OUT/summary.txt"; exit 2;; esac
done
up=$(docker ps --format '{{.Names}}' | grep -xE 'oak-d|rob-box-quest|vision-face' || true)
[ -n "$up" ] && { echo "гашу (политика стенда): $up"; docker stop $up >/dev/null; }

rec silence_before 3
echo "silence_before_dbfs $(rms "$OUT/silence_before.wav")" | tee -a "$OUT/summary.txt"
T0=$(date -u +%Y-%m-%dT%H:%M:%SZ); echo "T0 $T0 dur ${DUR}s theme $THEME" | tee -a "$OUT/summary.txt"
rec set $((DUR + 25)) &
REC=$!
sleep 1
call "{\"action\":\"start\",\"theme\":\"$THEME\"}" | tee "$OUT/start.json"
sleep "$DUR"
call '{"action":"stop"}' | tee "$OUT/stop.json"
wait $REC
rec silence_after 3
echo "silence_after_dbfs $(rms "$OUT/silence_after.wav")" | tee -a "$OUT/summary.txt"

docker logs -t voice-assistant --since "$T0" 2>&1 | grep -E "\[music v2\]|\[set v2\]" > "$OUT/engine.log"
{
  echo "started $(grep -c '\] started track_id' "$OUT/engine.log")"
  echo "started_phase_not_0 $(grep '\] started track_id' "$OUT/engine.log" | grep -vc 'phase_in_form=0.0 ')"
  echo "bpm_values $(grep -o '\] started track_id.* bpm=[0-9.]*' "$OUT/engine.log" | grep -o 'bpm=[0-9.]*' | sort | uniq -c | tr '\n' ' ')"
  echo "extended $(grep -c 'продлеваю' "$OUT/engine.log")"
  echo "rejected $(grep -c 'rejected track_id' "$OUT/engine.log")"
  echo "artifact_stale $(grep -c 'artifact_stale' "$OUT/engine.log")"
  echo "DJ_AUTO $(docker logs voice-assistant --since "$T0" 2>&1 | grep -c 'DJ_AUTO')"
  echo "late_lines $(docker logs -t supercollider --since "$T0" 2>&1 | grep -cE ' late [0-9]')"
} | tee -a "$OUT/summary.txt"

docker exec voice-assistant mkdir -p /tmp/dj_set_v2
docker cp "$HERE/." voice-assistant:/tmp/dj_set_v2/
docker cp "$OUT/set.wav" voice-assistant:/tmp/dj_set_v2/set.wav
$VA 'cd /tmp/dj_set_v2 && python3 accept.py set.wav && python3 compare.py set.wav --chunk 60' | tee "$OUT/accept.txt"
docker exec voice-assistant rm -rf /tmp/dj_set_v2
if [ -n "${TG_CHAT_ID:-}" ]; then
  A1=$(grep -m1 -oE 'A1[^|]*' "$OUT/accept.txt" | head -1)
  TG_CHAT_ID=$TG_CHAT_ID bash "$HERE/tgogg.sh" "$OUT/set.wav" \
    "PR-5 #3312, сет v2 ${DUR} с, тема «$THEME». $(tr '\n' ' ' < "$OUT/summary.txt" | cut -c1-700) | $A1"
fi
