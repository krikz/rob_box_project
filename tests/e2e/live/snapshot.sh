#!/bin/bash
# tests/e2e/live/snapshot.sh — v2 (дополнение v1 §9, см. E2E_TESTING_DESIGN_v2.md §F.2)
#
# Захватывает state «было/стало» перед/после e2e-прогона на живом роботе.
# v2 добавляет (см. acceptance A42, A43):
#   1. docker ps -a (не только running) — упавшие контейнеры видны
#   2. clock_drift_ms (docker exec voice-assistant date vs host)
#   3. voice-action-server crash count → DEGRADED если >5 рестартов за час
#   4. multi-model provenance: читает tests/e2e/_artifacts/<run_id>/model.json
#
# Использование:
#   bash tests/e2e/live/snapshot.sh <artifacts_dir> [run_id]
#
# Артефакты (в $ARTIFACTS):
#   docker_ps_all.tsv           # A43 — все контейнеры со статусом
#   docker_ps_running.tsv       # v1 — только running (back-compat)
#   uptime.txt                  # v1
#   git_rev.txt                 # v1
#   clock_drift_ms.txt          # v2 — drift внутри voice-assistant vs host
#   voice_action_server_crashes.txt   # v2 — ModuleNotFoundError aiohttp count
#   voice_action_server_restarts.txt  # v2 — restart count за последний час
#   build_wav_list.txt          # v2 — последние 20 wav на билдовой
#   build_clock.txt             # v1
#   build_git_rev.txt           # v2 — git rev на recorder
#   last_model.txt              # v2 — последняя MODEL: строка из recorder
#   model.json                  # v2 — скопированный из _artifacts/<run_id>
#   jack_lsp.txt                # v1
#   pytest_version.txt          # v1
#   python_version.txt          # v1
#   state_snapshot.json         # v1 — сводный json
#   verdict_helper.txt          # v2 — DEGRADED если crashes > 5
set -u

ARTIFACTS="${1:-tests/e2e/_artifacts/$(date -u +%Y%m%d_%H%M%S)}"
RUN_ID="${2:-}"
ROBOT="${ROBOT:-10.1.1.21}"
BUILD="${BUILD:-10.1.1.249}"
SSHPASS="${SSHPASS:-open}"

mkdir -p "$ARTIFACTS"
echo "[snapshot] writing to $ARTIFACTS (run_id=$RUN_ID)"

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=8)
SSH="sshpass -p $SSHPASS ssh ${SSH_OPTS[*]}"

# ---------------------------------------------------------------------------
# 1. ROBOT — все контейнеры (не только running, см. A43)
# ---------------------------------------------------------------------------
$SSH ros2@$ROBOT 'docker ps -a --format "{{.Names}}\t{{.Status}}\t{{.Image}}"' \
  > "$ARTIFACTS/docker_ps_all.tsv" 2>&1 || echo "docker_ps_all FAILED" > "$ARTIFACTS/docker_ps_all.tsv"

# Running-only — back-compat с v1 (для diff со старыми отчётами)
$SSH ros2@$ROBOT 'docker ps --format "{{.Names}}\t{{.Status}}\t{{.Image}}"' \
  > "$ARTIFACTS/docker_ps_running.tsv" 2>&1 || echo "docker_ps_running FAILED" > "$ARTIFACTS/docker_ps_running.tsv"

# ---------------------------------------------------------------------------
# 2. ROBOT — uptime + git rev (v1 §9)
# ---------------------------------------------------------------------------
$SSH ros2@$ROBOT 'uptime' > "$ARTIFACTS/uptime.txt" 2>&1 || true
$SSH ros2@$ROBOT \
  'docker exec voice-assistant bash -c "cd /ws && git rev-parse HEAD 2>/dev/null || echo unknown"' \
  > "$ARTIFACTS/git_rev.txt" 2>&1 || true

# ---------------------------------------------------------------------------
# 3. ROBOT — voice-action-server crash count (v2, A43)
#    Считаем:
#      a) ModuleNotFoundError строки в последних 50 строках лога
#         (текущий hot indicator aiohttp-бага, см. recovery t_4f546ead finding)
#      b) restart count за последний час (docker inspect)
# ---------------------------------------------------------------------------
$SSH ros2@$ROBOT \
  'docker logs --tail 50 voice-action-server 2>&1 | grep -c "ModuleNotFoundError" || echo 0' \
  > "$ARTIFACTS/voice_action_server_crashes.txt" 2>&1 || echo 0 > "$ARTIFACTS/voice_action_server_crashes.txt"

# restart count за последний час
RESTARTS_1H="$($SSH ros2@$ROBOT \
  'docker inspect --format="{{.State.RestartCount}}" voice-action-server 2>/dev/null || echo 0')"
echo "$RESTARTS_1H" > "$ARTIFACTS/voice_action_server_restarts.txt"

CRASHES=$(cat "$ARTIFACTS/voice_action_server_crashes.txt" 2>/dev/null | tr -dc '0-9' || echo 0)
DEGRADED_REASON=""
if [ "${CRASHES:-0}" -gt 5 ]; then
  DEGRADED_REASON="voice-action-server crashes=${CRASHES} (>5)"
fi
# доп. сигнал — Restarting loop (recovery t_4f546ead наблюдал такое)
VAS_STATUS="$($SSH ros2@$ROBOT 'docker inspect --format="{{.State.Status}}" voice-action-server 2>/dev/null || echo unknown')"
if echo "$VAS_STATUS" | grep -qi "restarting"; then
  DEGRADED_REASON="${DEGRADED_REASON:+$DEGRADED_REASON; }voice-action-server status=$VAS_STATUS"
fi
if [ -n "$DEGRADED_REASON" ]; then
  echo "DEGRADED $DEGRADED_REASON" > "$ARTIFACTS/verdict_helper.txt"
fi

# ---------------------------------------------------------------------------
# 4. ROBOT — clock drift (v2)
#    docker exec date vs host date — оба в epoch ms
# ---------------------------------------------------------------------------
ROBOT_TS="$($SSH ros2@$ROBOT 'docker exec voice-assistant date +%s%3N' 2>/dev/null | tr -dc '0-9-')"
HOST_TS="$(date +%s%3N)"
if [ -n "$ROBOT_TS" ] && [ -n "$HOST_TS" ]; then
  echo $((ROBOT_TS - HOST_TS)) > "$ARTIFACTS/clock_drift_ms.txt"
else
  echo "N/A" > "$ARTIFACTS/clock_drift_ms.txt"
fi

# ---------------------------------------------------------------------------
# 5. BUILD HOST — recorder artefacts
# ---------------------------------------------------------------------------
$SSH ros2@$BUILD 'ls -lt /tmp/dialog_e2e_*.wav 2>/dev/null | head -20' \
  > "$ARTIFACTS/build_wav_list.txt" 2>&1 || true
$SSH ros2@$BUILD 'date -u +%Y-%m-%dT%H:%M:%SZ' > "$ARTIFACTS/build_clock.txt" 2>&1 || true
$SSH ros2@$BUILD 'cd /home/ros2/rob_box_project 2>/dev/null && git rev-parse HEAD 2>/dev/null || echo no_git' \
  > "$ARTIFACTS/build_git_rev.txt" 2>&1 || true

# 6. multi-model provenance — model.json из recorder (_artifacts/<run_id>)
if [ -n "$RUN_ID" ]; then
  $SSH ros2@$BUILD \
    "cat /home/ros2/rob_box_project/tests/e2e/_artifacts/${RUN_ID}/model.json 2>/dev/null || echo \"{\\\"llm\\\":\\\"unknown\\\",\\\"tts\\\":\\\"unknown\\\",\\\"stt\\\":\\\"unknown\\\"}\"" \
    > "$ARTIFACTS/model.json" 2>&1 || true
  # last_model.txt — последняя MODEL: строка из любых логов recorder'а
  $SSH ros2@$BUILD \
    'grep -rh "MODEL:" /tmp/*.log /home/ros2/rob_box_project/tests/e2e/_artifacts/ 2>/dev/null | tail -1' \
    > "$ARTIFACTS/last_model.txt" 2>&1 || true
fi

# ---------------------------------------------------------------------------
# 7. ROBOT — jack_lsp (v1)
# ---------------------------------------------------------------------------
$SSH ros2@$ROBOT 'docker exec supercollider jack_lsp -c' > "$ARTIFACTS/jack_lsp.txt" 2>&1 || true

# ---------------------------------------------------------------------------
# 8. Локально — pytest/python (v1)
# ---------------------------------------------------------------------------
git rev-parse HEAD > "$ARTIFACTS/local_git_rev.txt" 2>&1 || echo unknown > "$ARTIFACTS/local_git_rev.txt"
pytest --version > "$ARTIFACTS/pytest_version.txt" 2>&1 || echo "pytest missing" > "$ARTIFACTS/pytest_version.txt"
python3 --version > "$ARTIFACTS/python_version.txt" 2>&1 || echo "python3 missing" > "$ARTIFACTS/python_version.txt"

# ---------------------------------------------------------------------------
# 9. Сводный state_snapshot.json + snapshot.json (для acceptance A43)
# ---------------------------------------------------------------------------
CLOCK_DRIFT=$(cat "$ARTIFACTS/clock_drift_ms.txt" 2>/dev/null || echo N/A)
RESTARTS=$(cat "$ARTIFACTS/voice_action_server_restarts.txt" 2>/dev/null || echo 0)
VAS_CRASHES=$(cat "$ARTIFACTS/voice_action_server_crashes.txt" 2>/dev/null | tr -dc '0-9')
VAS_CRASHES=${VAS_CRASHES:-0}
DEGRADED_FLAG="false"
[ -s "$ARTIFACTS/verdict_helper.txt" ] && DEGRADED_FLAG="true"

# Подсчёт unhealthy/exited контейнеров из docker_ps_all.tsv (A43)
UNHEALTHY=$(awk -F'\t' 'NR>0 && $2 !~ /^Up/ && $2 !~ /^Exited \(0\)/ {n++} END {print n+0}' "$ARTIFACTS/docker_ps_all.tsv" 2>/dev/null || echo 0)

MODEL_JSON_CONTENT="{}"
[ -s "$ARTIFACTS/model.json" ] && MODEL_JSON_CONTENT=$(cat "$ARTIFACTS/model.json")

python3 - "$ARTIFACTS" "$ROBOT" "$CLOCK_DRIFT" "$RESTARTS" "$VAS_CRASHES" "$DEGRADED_FLAG" "$UNHEALTHY" "$MODEL_JSON_CONTENT" <<'PY'
import json, sys, os, datetime
artifacts, robot, drift, restarts, crashes, degraded, unhealthy, model_raw = sys.argv[1:9]
try:
    model = json.loads(model_raw) if model_raw.strip().startswith("{") else {}
except Exception:
    model = {}
state = {
  "snapshot_at": datetime.datetime.utcnow().isoformat() + "Z",
  "schema": "v2",
  "robot": robot,
  "clock_drift_ms": drift,
  "voice_action_server": {
    "module_not_found_crashes": int(crashes or 0),
    "restart_count_total": int(restarts or 0),
    "degraded": degraded == "true",
  },
  "unhealthy_container_count": int(unhealthy or 0),
  "model": model,
}
with open(os.path.join(artifacts, "state_snapshot.json"), "w", encoding="utf-8") as fh:
    json.dump(state, fh, indent=2, ensure_ascii=False)
# snapshot.json — flat view для acceptance A43 (matching name)
with open(os.path.join(artifacts, "snapshot.json"), "w", encoding="utf-8") as fh:
    json.dump({
        "clock_drift_ms": drift,
        "voice_action_server_crashes": int(crashes or 0),
        "voice_action_server_restarts": int(restarts or 0),
        "unhealthy_container_count": int(unhealthy or 0),
        "degraded": degraded == "true",
        "model": model,
    }, fh, indent=2, ensure_ascii=False)
print("[snapshot] state_snapshot.json + snapshot.json written")
PY

# ---------------------------------------------------------------------------
# 10. Exit code — degraded (A43 acceptance)
# ---------------------------------------------------------------------------
if [ "$DEGRADED_FLAG" = "true" ]; then
  echo "[snapshot] DEGRADED — see verdict_helper.txt"
  cat "$ARTIFACTS/verdict_helper.txt" >&2
  # Не падаем — degraded != failed; это просто маркер для aggregate_verdicts.py
  exit 0
fi

echo "[snapshot] OK — artefacts in $ARTIFACTS"