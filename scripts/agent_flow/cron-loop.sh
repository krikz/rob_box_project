#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/cron-loop.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/cron-loop.sh
#   - ~/.hermes/profiles/architect/scripts/cron-loop.sh
#   - ~/.hermes/scripts/cron-loop.sh
# Правка: редактируем <repo>/scripts/agent_flow/cron-loop.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
#\!/usr/bin/env bash
# Простой цикл для cron-задач Hermes — пока нет gateway.
# Каждые 60с вызывает hermes cron tick, который запускает due jobs.
# Запускать вручную: nohup cron-loop.sh > /home/builder/.hermes/logs/cron-loop.log 2>&1 &
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
UVX="$HERMES_HOME/bin/uvx"
LOG="$HERMES_HOME/logs/cron-tick.log"
mkdir -p "$(dirname "$LOG")"

# Sentinel — если файла нет, останавливаемся
SENT="$HERMES_HOME/state/cron-loop.sentinel"
if [ \! -f "$SENT" ]; then
    echo "no sentinel — exit" >&2
    exit 1
fi

while [ -f "$SENT" ]; do
    echo "[cron-loop] tick at $(date -Iseconds)" >> "$LOG"
    cd "$HERMES_HOME" && $UVX --from hermes-agent hermes cron tick --accept-hooks >> "$LOG" 2>&1
    sleep 60
done
