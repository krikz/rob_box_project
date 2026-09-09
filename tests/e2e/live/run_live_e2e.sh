#!/bin/bash
# tests/e2e/live/run_live_e2e.sh — v2 (см. E2E_TESTING_DESIGN_v2.md §F.3)
#
# Multi-model e2e entry-point:
#   1. Создаёт tests/e2e/_artifacts/<run_id>/ с model.json + snapshot
#   2. Для каждого сценария диспатчит GH workflow с явными llm/tts/stt
#   3. Запускает watcher и ждёт verdict'ов
#
# Использование:
#   LLM=minimax-m2 TTS=minimax-male-qn-qingse STT=yandex \
#     SCENARIOS=R1,R2,R3 bash tests/e2e/live/run_live_e2e.sh
#
# ENV:
#   ROBOT (default 10.1.1.21)   — для snapshot
#   BUILD (default 10.1.1.249)  — recorder host
#   LLM (default minimax-m2)
#   TTS (default minimax-male-qn-qingse)
#   STT (default yandex)
#   SCENARIOS (default R1,R2,R3,R4,R5,R6,R7,R8)
#   WORKFLOW_REF (default feature/harness-p0-foundation)
#   REPORT (default tests/e2e/_artifacts/live_<UTC-ts>/)
set -u

ROBOT="${ROBOT:-10.1.1.21}"
BUILD="${BUILD:-10.1.1.249}"
LLM="${LLM:-minimax-m2}"
TTS="${TTS:-minimax-male-qn-qingse}"
STT="${STT:-yandex}"
SCENARIOS="${SCENARIOS:-R1,R2,R3,R4,R5,R6,R7,R8}"
WORKFLOW_REF="${WORKFLOW_REF:-feature/harness-p0-foundation}"
WORKFLOW_FILE="${WORKFLOW_FILE:-L-E2E Voice Test.yml}"
SSHPASS="${SSHPASS:-open}"

RUN_ID="$(date -u +%Y%m%d_%H%M%S)_$$"
REPORT="${REPORT:-tests/e2e/_artifacts/live_${RUN_ID}/}"

mkdir -p "$REPORT"
echo "[run_live_e2e] run_id=$RUN_ID  REPORT=$REPORT"
echo "[run_live_e2e] LLM=$LLM  TTS=$TTS  STT=$STT  SCENARIOS=$SCENARIOS"

# ---------------------------------------------------------------------------
# 1. Записать model.json в artifacts ДО запуска workflow
#    (workflow потом тоже его пишет — здесь для быстрого offline-доступа
#     к конфигурации прогона до того, как GH Actions отработает)
# ---------------------------------------------------------------------------
cat > "$REPORT/model.json" <<EOF
{
  "llm": "$LLM",
  "tts": "$TTS",
  "stt": "$STT",
  "ts":  "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "run_id": "$RUN_ID",
  "scenarios": [$(echo "$SCENARIOS" | awk -F',' '{for(i=1;i<=NF;i++) printf "\"%s\"%s", $i, (i<NF?",":""); print ""}')]
}
EOF
echo "[run_live_e2e] wrote $REPORT/model.json"
cat "$REPORT/model.json"

# ---------------------------------------------------------------------------
# 2. Snapshot v2 (§F.2)
# ---------------------------------------------------------------------------
echo "[run_live_e2e] running snapshot.sh..."
bash "$(dirname "$0")/snapshot.sh" "$REPORT" "$RUN_ID" || \
  echo "[run_live_e2e] WARNING: snapshot.sh returned non-zero (degraded?)"

# ---------------------------------------------------------------------------
# 3. Dispatch GH workflow для каждого сценария
# ---------------------------------------------------------------------------
if ! command -v gh >/dev/null 2>&1; then
  echo "[run_live_e2e] ERROR: gh CLI not found — cannot dispatch workflow" >&2
  exit 2
fi

for S in $(echo "$SCENARIOS" | tr ',' ' '); do
  SCEN_LC="$(echo "$S" | tr '[:upper:]' '[:lower:]')"
  VF=".github/e2e/voice_commands/rabot_${SCEN_LC}.ogg"
  echo "[run_live_e2e] dispatching $S (file=$VF) ..."
  gh workflow run "$WORKFLOW_FILE" \
    --ref "$WORKFLOW_REF" \
    -f environment=test \
    -f voice_file="$VF" \
    -f volume=150 \
    -f llm="$LLM" \
    -f tts="$TTS" \
    -f stt="$STT" \
    || echo "[run_live_e2e] WARNING: gh workflow run failed for $S"
  sleep 180
done

# ---------------------------------------------------------------------------
# 4. Watcher (если есть) — онлайн-слежение за verdict'ами
# ---------------------------------------------------------------------------
WATCHER="${WATCHER:-/tmp/e2e_watcher7.sh}"
if [ -x "$WATCHER" ]; then
  echo "[run_live_e2e] launching watcher: $WATCHER"
  nohup "$WATCHER" > "$REPORT/watcher.log" 2>&1 &
  echo $! > "$REPORT/watcher.pid"
fi

# ---------------------------------------------------------------------------
# 5. Verdict aggregation (если есть скрипт)
# ---------------------------------------------------------------------------
AGGREGATE="${AGGREGATE:-tests/e2e/live/aggregate_verdicts.py}"
if [ -f "$AGGREGATE" ]; then
  echo "[run_live_e2e] aggregating verdicts..."
  python3 "$AGGREGATE" "$REPORT" || \
    echo "[run_live_e2e] WARNING: aggregate_verdicts.py failed"
fi

# ---------------------------------------------------------------------------
# 6. Финальный сводный отчёт
# ---------------------------------------------------------------------------
{
  echo "# Live E2E report — run_id=$RUN_ID"
  echo ""
  echo "- started: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "- LLM/TTS/STT: \`$LLM\` / \`$TTS\` / \`$STT\`"
  echo "- scenarios: $SCENARIOS"
  echo "- robot: $ROBOT"
  echo "- recorder: $BUILD"
  echo ""
  echo "## Artefacts"
  ls -la "$REPORT" || true
} > "$REPORT/SUMMARY.md"

echo "[run_live_e2e] done — see $REPORT"