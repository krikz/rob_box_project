#!/usr/bin/env bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-post-merge-build.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические/жёсткие ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/agent-flow-post-merge-build.sh
#   - ~/.hermes/profiles/architect/scripts/agent-flow-post-merge-build.sh
#   - ~/.hermes/scripts/agent-flow-post-merge-build.sh
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-post-merge-build.sh,
# commit, merge. На хост: bash <repo>/scripts/agent_flow/install.sh
# (или вручную cp + ln -sf). Если правишь этот файл НА ХОСТЕ — синхронизируй
# обратно в репо.
# ============================================================================
# agent-flow-post-merge-build.sh — GATE-4 (ADR-0022 extension, issue #1475)
#
# Цель: после merge PR в develop (или main) — запустить L-Build-All-Services
# чтобы .image-versions.dev получил свежие SHA-теги (dev-<sha>) для
# deploy-цепочки. До этого фикса:
#   - L-Build-All-Services.yml триггерился ТОЛЬКО от workflow_dispatch
#     (e2e-process на round-ветке) И от workflow_call (G-Auto-merge to Main);
#   - push в develop триггерил G-Auto-merge to Main, который создавал
#     PR в main, но НЕ билдил develop HEAD;
#   - результат: после merge любого PR в develop develop-HEAD не
#     пересобирался, и deploy тащил старый .image-voices.dev тег
#     (issue #1475, evidence: PR #1434 merge 18.08 23:00 MSK →
#     на роботе до сих пор dev-ddd09e51 от 18.08 17:49 MSK, 5 часов stale).
#
# Этот скрипт — GATE-4 ADR-0022 extension. Вызывается из
# agent-flow-merge-gate.sh после успешного MERGED detection:
#   log "issue #${number}: PR #${pr_number} MERGED into ${pr_base}"
#   bash scripts/agent_flow/agent-flow-post-merge-build.sh "$pr_number" "$pr_base"
#
# Поведение:
#   1. Проверяет, что ветка — develop или main (не feature/copilot/test).
#   2. Запускает L-Build-All-Services через `gh workflow run` с retry.
#   3. НЕ блокирует merge-gate: ошибка триггера = warning + continue.
#   4. Идемпотентен: запуск с тем же (pr_number, branch) подряд — окей,
#      GitHub сериализует одинаковые workflow по workflow_name+head_branch.
#
# Pure bash. No LLM. Idempotent.
set -euo pipefail

# --- defaults (overridden by env / .env) -------------------------------------
HERMES_HOME=/home/builder/.hermes
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"
export HOME=/home/builder

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
BUILD_WORKFLOW="${BUILD_WORKFLOW:-L-Build-All-Services.yml}"
DEVELOP_BRANCH="${DEVELOP_BRANCH:-develop}"
MAIN_BRANCH="${MAIN_BRANCH:-main}"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-post-merge-build]}"
DRY_RUN="${DRY_RUN:-false}"

# --- args --------------------------------------------------------------------
# $1 = pr_number (для логирования и идемпотентности), $2 = pr_base (develop/main)
PR_NUMBER="${1:-}"
PR_BASE="${2:-}"

if [ -z "$PR_NUMBER" ] || [ -z "$PR_BASE" ]; then
    echo "❌ usage: agent-flow-post-merge-build.sh <pr_number> <pr_base>" >&2
    exit 64
fi

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
# DRY_RUN semantics: print the would-be command to stderr (NOT stdout, so
# `2>/dev/null` callers don't suppress it) and return success (0).
run() {
    if [ "$DRY_RUN" = "true" ]; then
        printf '%s DRY-RUN %s\n' "$LOG_PREFIX" "$*" >&2
        return 0
    else
        eval "$@"
    fi
}

# --- pre-condition: только develop/main --------------------------------------
case "$PR_BASE" in
    "$DEVELOP_BRANCH"|"$MAIN_BRANCH")
        log "PR #${PR_NUMBER} → ${PR_BASE}: eligible for post-merge build"
        ;;
    *)
        log "PR #${PR_NUMBER} → ${PR_BASE}: SKIP (only ${DEVELOP_BRANCH}/${MAIN_BRANCH} trigger build)"
        exit 0
        ;;
esac

# --- pre-flight: GH auth -----------------------------------------------------
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
    log "❌ gh auth not available — build trigger skipped (PR #${PR_NUMBER} → ${PR_BASE})"
    exit 0  # non-fatal: merge-gate continues
fi

# --- pre-flight: workflow exists --------------------------------------------
if ! gh workflow view "$BUILD_WORKFLOW" --repo "$GH_REPO" >/dev/null 2>&1; then
    log "❌ workflow ${BUILD_WORKFLOW} not found in ${GH_REPO} — build trigger skipped"
    exit 0
fi

# --- trigger with retry ------------------------------------------------------
# Ретро 10.08 #1: gh workflow run может вернуть non-zero exit сразу после
# свежего push (GitHub API rate-limit / eventual consistency). Retry 3 раза
# с backoff.
for attempt in 1 2 3; do
    # ВАЖНО: НЕ глушим stderr — `run()` пишет DRY-RUN маркер в stderr,
    # который тесты ловят. `gh workflow run` пишет прогресс в stderr сам —
    # он НЕ критичный для нас (мы не parse'им), но и не должен
    # блокировать. Поэтому `>/dev/null` только на stdout, а stderr пробрасываем.
    if run gh workflow run "$BUILD_WORKFLOW" --repo "$GH_REPO" \
            --ref "$PR_BASE" \
            --field push_to_registry=true \
            --field build_base_images=false >/dev/null; then
        if [ "$DRY_RUN" = "true" ]; then
            log "would-trigger ${BUILD_WORKFLOW} for ${PR_BASE} (PR #${PR_NUMBER}, attempt ${attempt})"
        else
            log "✅ ${BUILD_WORKFLOW} triggered for ${PR_BASE} (PR #${PR_NUMBER}, attempt ${attempt})"
        fi
        exit 0
    fi
    log "⚠️ gh workflow run attempt ${attempt}/3 failed — retry in $((attempt*5))s"
    sleep $((attempt*5))
done

log "❌ ${BUILD_WORKFLOW} trigger failed after 3 attempts (PR #${PR_NUMBER} → ${PR_BASE})"
log "   merge-gate continues; manual retry: gh workflow run ${BUILD_WORKFLOW} --repo ${GH_REPO} --ref ${PR_BASE}"
exit 0  # non-fatal: merge-gate не должен падать из-за build trigger
