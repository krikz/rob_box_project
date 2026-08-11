#!/bin/bash
# install.sh — раскладка процессных скриптов agent-flow по нужным директориям.
#
# Source of truth: <repo>/scripts/agent_flow/*.sh (эта папка).
# Копии (которые ищет cron при старте):
#   1. /home/builder/.hermes/profiles/agent-flow/scripts/   — каноническое место
#   2. /home/builder/.hermes/profiles/architect/scripts/    — где cron сейчас ищет
#   3. /home/builder/.hermes/scripts/                       — legacy (cron тоже стартует)
#
# Этот скрипт создаёт **символические ссылки** на каждом из этих путей,
# указывающие на канонические файлы в репо. Гарантии:
#   - Все 3 (или сколько есть) путей смотрят на одну и ту же копию
#   - Правка в репо (через PR/merge) автоматически расходится по всем путям
#     сразу при следующем запуске этого скрипта
#   - Нет дубликатов, нет drift (см. scripts/agent_flow/README.md)
#
# Запуск:
#   ./scripts/agent_flow/install.sh            # раскладка на хост
#   ./scripts/agent_flow/install.sh --dry-run  # только показать что сделает
#
# Идемпотентен — повторный запуск обновляет ссылки, ничего не ломает.

set -e
DRY_RUN=false
[ "${1:-}" = "--dry-run" ] && DRY_RUN=true

# Если передан явный REPO_DIR (например на build host, где нет
# /home/builder/hermes-share), используем его:
#   bash install.sh /tmp/install_af_1107
#   bash install.sh --dry-run /tmp/install_af_1107
#   REPO_DIR=/tmp/install_af_1107 bash install.sh
# По умолчанию — путь dev-машины.
if [ "${1:-}" = "--dry-run" ]; then
    REPO_DIR="${REPO_DIR:-${2:-/home/builder/hermes-share/rob_box_project}}"
else
    REPO_DIR="${REPO_DIR:-${1:-/home/builder/hermes-share/rob_box_project}}"
fi
SCRIPT_DIR="$REPO_DIR/scripts/agent_flow"

# Канонические пути (все должны стать симлинками на одну и ту же цель)
TARGET_DIRS=(
    "/home/builder/.hermes/profiles/agent-flow/scripts"
    "/home/builder/.hermes/profiles/architect/scripts"
    "/home/builder/.hermes/scripts"
)

run() {
    if $DRY_RUN; then
        echo "  [DRY] $*"
    else
        "$@"
    fi
}

echo "==> Source of truth: $SCRIPT_DIR"
if [ ! -d "$SCRIPT_DIR" ]; then
    echo "ERROR: source dir not found: $SCRIPT_DIR (clone the repo there?)"
    exit 1
fi

# sanity check — файлы на месте
EXPECTED=(
    agent-flow-triage.sh
    agent-flow-merge-gate.sh
    agent-flow-e2e-process.sh
    agent-flow-handoff.sh
    cron-loop.sh
    watchdog.sh
)
for f in "${EXPECTED[@]}"; do
    if [ ! -f "$SCRIPT_DIR/$f" ]; then
        echo "ERROR: missing canonical file $SCRIPT_DIR/$f"
        exit 2
    fi
done

for target_dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$target_dir" ]; then
        echo "  SKIP $target_dir (dir not present)"
        continue
    fi

    echo "==> $target_dir"
    run mkdir -p "$target_dir"
    for f in "${EXPECTED[@]}"; do
        src="$SCRIPT_DIR/$f"
        dst="$target_dir/$f"

        # Если существующий dst — НЕ симлинк или указывает не на наш src —
        # заменяем его (но сохраняем файл в .bak).
        if [ -L "$dst" ] && [ "$(readlink -f "$dst")" = "$(readlink -f "$src")" ]; then
            echo "  OK   $f (already linked)"
            continue
        fi

        if [ -e "$dst" ] && [ ! -L "$dst" ]; then
            bak="${dst}.bak.$(date -u +%Y%m%dT%H%M%SZ)"
            run mv "$dst" "$bak"
            echo "  BAK  $f (was real file — saved as $bak)"
        fi

        run ln -sf "$src" "$dst"
        echo "  LINK $f -> $src"
    done
done

echo
echo "==> Done. Verify:"
if ! $DRY_RUN; then
    for f in "${EXPECTED[@]}"; do
        f1="/home/builder/.hermes/profiles/agent-flow/scripts/$f"
        f2="/home/builder/.hermes/profiles/architect/scripts/$f"
        f3="/home/builder/.hermes/scripts/$f"
        for fp in "$f1" "$f2" "$f3"; do
            [ -e "$fp" ] && echo "  $(readlink -f "$fp"): $(md5sum "$fp" 2>/dev/null | cut -c1-10)"
        done
        echo "  ---"
    done
fi
