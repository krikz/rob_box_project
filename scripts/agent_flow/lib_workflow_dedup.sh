#!/bin/bash
# ============================================================================
# lib_workflow_dedup.sh — shared helpers for workflow-dispatch dedup.
#
# SOT: <repo>/scripts/agent_flow/lib_workflow_dedup.sh
# Копии раскладываются install.sh в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Назначение (issue #1540, ретро 22.08):
#   `gh workflow run` под нагрузкой плодит дубликаты (race condition eventual
#   consistency API после push). Был локальный verify_recent_run() в
#   agent-flow-post-merge-build.sh (issue #1538 / PR #1536), но e2e-process
#   НЕ имел pre-dispatch dedup → 2 build на develop за 9-23 секунды
#   (таблица в issue #1540).
#
#   Решение: вынести verify_recent_run() в общий lib + добавить pre-dispatch
#   guard в agent-flow-e2e-process.sh::_trigger_workflow_with_retry().
#
# Использование (после source):
#   if verify_recent_run "L-Build-All-Services.yml" "develop" 900; then
#       log "⏭️ recent build (≤900s) on develop — skip"
#       exit 0
#   fi
#   gh workflow run ...
#
#   # Или удобный wrapper, если переменные workflow/branch/repo уже в env:
#   if verify_recent_run_default 900; then
#       log "⏭️ recent run on $WORKFLOW/$BRANCH — skip"
#       return 0
#   fi
#
# Гарантии (ADR-0014 §4 req 4, conservative on uncertainty → fail-OPEN):
#   - gh run list сдох / пустой → return 1 (НЕ блокируем, идём как обычно —
#     свежий run мог быть старше окна или gh не доступен, лучше один
#     потенциальный дубль чем пропуск нужного build).
#   - jq НЕ используется (его может не быть в PATH на роботе) — парсим JSON
#     через python3 (всегда есть в hermes-agent venv).
# ============================================================================

# ---------------------------------------------------------------------------
# verify_recent_run <workflow> <branch> <repo> <window_seconds>
#
# Echo: 'ok' если свежий run для $workflow на $branch в $repo найден в
# `gh run list` за последние $window секунд. Иначе 'miss'.
#
# Возвращает exit 0 в ОБОИХ случаях (ok/miss), реальный результат — через
# stdout. Это позволяет: `if [ "$(verify_recent_run ...)" = "ok" ]; then ...`
#
# Параметры:
#   $1 — workflow (например "L-Build-All-Services.yml")
#   $2 — branch   (например "develop" или "z-{e2e}/test-round-7")
#   $3 — repo    (например "krikz/rob_box_project")
#   $4 — window_seconds (default 60)
#
# Использует python3 для парсинга JSON (jq может отсутствовать на роботе).
# ---------------------------------------------------------------------------
verify_recent_run() {
    local workflow="${1:-}"
    local branch="${2:-}"
    local repo="${3:-}"
    local window="${4:-60}"

    if [ -z "$workflow" ] || [ -z "$branch" ] || [ -z "$repo" ]; then
        echo "miss"
        return 0
    fi

    local now
    now="$(date -u +%s)"

    local runs_json
    runs_json="$(gh run list --workflow "$workflow" --repo "$repo" \
        --branch "$branch" --limit 1 --json databaseId,createdAt 2>/dev/null || true)"
    if [ -z "$runs_json" ]; then
        echo "miss"
        return 0
    fi

    local created_at
    created_at="$(printf '%s' "$runs_json" | python3 -c '
import sys, json
try:
    data = json.load(sys.stdin)
    if not data:
        sys.exit(0)
    print(data[0].get("createdAt", ""))
except Exception:
    sys.exit(0)
')"
    if [ -z "$created_at" ]; then
        echo "miss"
        return 0
    fi

    local created_epoch
    created_epoch="$(date -d "$created_at" +%s 2>/dev/null || echo 0)"
    if [ "$created_epoch" -eq 0 ]; then
        echo "miss"
        return 0
    fi

    local age=$((now - created_epoch))
    if [ "$age" -ge 0 ] && [ "$age" -le "$window" ]; then
        echo "ok"
    else
        echo "miss"
    fi
}

# ---------------------------------------------------------------------------
# verify_recent_run_default <window_seconds>
#
# Convenience wrapper для скриптов, где workflow/branch/repo уже в env
# (BUILD_WORKFLOW / PR_BASE / GH_REPO в post-merge-build, BRANCH / GH_REPO в
# e2e-process). Если нужные переменные не выставлены → 'miss' (fail-OPEN).
# ---------------------------------------------------------------------------
verify_recent_run_default() {
    local window="${1:-60}"
    local wf="${BUILD_WORKFLOW:-${WORKFLOW:-}}"
    local br="${PR_BASE:-${BRANCH:-}}"
    local repo="${GH_REPO:-}"
    verify_recent_run "$wf" "$br" "$repo" "$window"
}
