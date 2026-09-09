#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/cron-loop.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
# ============================================================================
# Простой цикл для cron-задач Hermes — пока нет gateway.
# Каждые 60с вызывает hermes cron tick, который запускает due jobs.
# Запускать вручную: nohup cron-loop.sh > /home/builder/.hermes/logs/cron-loop.log 2>&1 &
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
UVX="$HERMES_HOME/bin/uvx"
LOG="$HERMES_HOME/logs/cron-tick.log"
mkdir -p "$(dirname "$LOG")"

# Sentinel — если файла нет, останавливаемся
SENT="$HERMES_HOME/state/cron-loop.sentinel"
if [ ! -f "$SENT" ]; then
    echo "no sentinel — exit" >&2
    exit 1
fi

while [ -f "$SENT" ]; do
    echo "[cron-loop] tick at $(date -Iseconds)" >> "$LOG"
    cd "$HERMES_HOME" && $UVX --from hermes-agent hermes cron tick --accept-hooks >> "$LOG" 2>&1
    sleep 60
done
