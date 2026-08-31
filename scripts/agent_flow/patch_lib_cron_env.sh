#!/bin/bash
# patch_lib_cron_env.sh — добавляет source lib_cron_env.sh в начало
# no-agent cron-скриптов (после SOT-баннера, до `set -euo pipefail`).
#
# Использование: cd scripts/agent_flow && bash patch_lib_cron_env.sh
# Идемпотентен: повторный запуск — no-op (проверка по маркеру
# `lib_cron_env preflight failed`).

set -euo pipefail

# Список скриптов, которым нужен env-bootstrap. Исключаем:
#   - lib_cron_env.sh — это сам модуль, не должен source'ить себя
#   - lib_user_unlabel_check.sh, lib_workflow_dedup.sh — это библиотеки,
#     их source'ат другие скрипты (которые УЖЕ получают env)
#   - hermes_github.sh — это helper для actor-marker, не standalone-скрипт
#   - push-via-gh-api.sh — ad-hoc helper
#   - validate_honesty.sh, round_ensure.sh, create_avatar_issues.sh —
#     manual/invoked scripts (не no-agent cron)
TARGETS=(
    agent-flow-cleanup-249.sh
    agent-flow-completion-check.sh
    agent-flow-deploy-sweep.sh
    agent-flow-e2e-drift-watchdog.sh
    agent-flow-e2e-process-launcher.sh
    agent-flow-e2e-process.sh
    agent-flow-handoff.sh
    agent-flow-merge-gate.sh
    agent-flow-post-merge-build.sh
    agent-flow-rotation-watchdog.sh
    agent-flow-triage.sh
    agents_sleep.sh
    cross-task-archive-sweeper.sh
    watchdog.sh
    # НЕ patched: agent-flow-unlabeled-sweep.sh уже имеет собственный
    # ENV_FILE-fallback (ретро t_9b0d60f7) — переключим на lib_cron_env
    # в этой карточке (замена, не дублирование).
    # НЕ patched: agent-flow-blocked-watchdog.sh — уже сделан вручную.
)

PATCH_BLOCK='set +e
# shellcheck source=lib_cron_env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/lib_cron_env.sh" || {
    printf "[%s] %s: lib_cron_env preflight failed — exit 1\n" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(basename "${BASH_SOURCE[0]:-$0}")" >&2
    exit 1
}
set -euo pipefail'

MARKER="lib_cron_env preflight failed"

for f in "${TARGETS[@]}"; do
    if [ ! -f "$f" ]; then
        echo "  SKIP $f (not found)"
        continue
    fi
    if grep -q "$MARKER" "$f"; then
        echo "  OK   $f (already patched)"
        continue
    fi
    # Ищем строку `set -euo pipefail` и вставляем перед ней
    if ! grep -q '^set -euo pipefail' "$f"; then
        echo "  WARN $f (no 'set -euo pipefail' line — skip)"
        continue
    fi
    # Используем awk для безопасной вставки перед target-строкой
    awk -v block="$PATCH_BLOCK" '
        /^set -euo pipefail$/ && !inserted {
            print block
            print ""
            inserted=1
        }
        { print }
    ' "$f" > "$f.tmp" && mv "$f.tmp" "$f"
    echo "  PATCHED $f"
done
