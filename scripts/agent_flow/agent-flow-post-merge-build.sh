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
#   2. Запускает L-Build-All-Services через `gh workflow run` с retry+dedup.
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

# --- trigger with dedup + retry ---------------------------------------------
# Issue #1535 / ретро 10.08 #1: gh workflow run может вернуть non-zero exit
# СРАЗУ после старта (GitHub API rate-limit / eventual consistency / network
# race после push). Старый retry 3 раза спамил дубликаты workflow runs
# даже когда ПЕРВЫЙ уже стартанул — issue #1535: 2 одинаковых build на
# develop за 57 секунд.
#
# Новый подход (дедупликация через `gh run list`):
#   1. `gh workflow run` → если OK → success.
#   2. Если FAILED → проверяем `gh run list --workflow=... --limit=1`:
#      если самый свежий run — наш workflow на $PR_BASE и создан недавно
#      (≤ 60 сек назад) → это race: build на самом деле стартанул, а API
#      вернул ошибку. Считаем SUCCESS — retry НЕ нужен.
#   3. Только если run не найден → реальная ошибка API → 1 retry (было 3).
#   4. Если retry тоже failed — exit 0 (non-fatal), merge-gate продолжает.
#
# Это сокращает спам дублей с 3 до 1 и обрабатывает корневую причину:
# API ошибку после успешного старта workflow.
#
# Issue #1527 (df6131c9 в develop): пробрасываем трейсинг-поля
# triggered_by_*, чтобы в CI job logs было видно кто реально стартанул
# этот build (пост-merge из merge-gate). Значения:
#   - triggered_by_script = "agent-flow-post-merge-build"
#   - triggered_by_agent  = "merge-gate" (можно override через TRIGGERED_BY_AGENT)
#   - triggered_by_card   = пусто (merge-gate не знает про конкретную
#     карточку, но может быть подсказан через TRIGGERED_BY_CARD)
#   - triggered_by_reason = "post-merge PR #${PR_NUMBER} → ${PR_BASE}"
_TBS="${AGENT_FLOW_SCRIPT:-agent-flow-post-merge-build}"
_TBA="${TRIGGERED_BY_AGENT:-merge-gate}"
_TBC="${TRIGGERED_BY_CARD:-}"
_TBR="post-merge PR #${PR_NUMBER} → ${PR_BASE}"
verify_recent_run() {
    # $1 = window_seconds (default 60). Echo: 'ok' если свежий run найден,
    # иначе 'miss'. Использует $BUILD_WORKFLOW, $GH_REPO, $PR_BASE из env.
    #
    # Возвращает:
    #   ok   — созданный за последние $window сек run для $BUILD_WORKFLOW
    #          на $PR_BASE найден в `gh run list` (race condition detected)
    #   miss — иначе (нет run, run старый, или API ошибка)
    #
    # НЕ использует jq (его может не быть в PATH на роботе) — парсит JSON
    # через python3 (всегда есть в hermes-agent venv).
    local window="${1:-60}"
    local now
    now="$(date -u +%s)"
    # gh run list → JSON массив runs. createdAt в ISO 8601 (UTC).
    # Парсим через python3 (на роботе python3 — стандарт).
    local runs_json
    runs_json="$(gh run list --workflow "$BUILD_WORKFLOW" --repo "$GH_REPO" \
        --branch "$PR_BASE" --limit 1 --json databaseId,createdAt 2>/dev/null || true)"
    if [ -z "$runs_json" ]; then
        echo "miss"; return 0
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
        echo "miss"; return 0
    fi
    local created_epoch
    created_epoch="$(date -d "$created_at" +%s 2>/dev/null || echo 0)"
    if [ "$created_epoch" -eq 0 ]; then
        echo "miss"; return 0
    fi
    local age=$((now - created_epoch))
    if [ "$age" -ge 0 ] && [ "$age" -le "$window" ]; then
        echo "ok"
    else
        echo "miss"
    fi
}

MAX_ATTEMPTS="${POST_MERGE_BUILD_MAX_ATTEMPTS:-2}"  # 2 = 1 попытка + 1 retry
DEDUP_WINDOW="${POST_MERGE_BUILD_DEDUP_WINDOW:-60}"  # секунд
for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    # ВАЖНО: НЕ глушим stderr — `run()` пишет DRY-RUN маркер в stderr,
    # который тесты ловят. `gh workflow run` пишет прогресс в stderr сам —
    # он НЕ критичный для нас (мы не parse'им), но и не должен
    # блокировать. Поэтому `>/dev/null` только на stdout, а stderr пробрасываем.
    if run gh workflow run "$BUILD_WORKFLOW" --repo "$GH_REPO" \
            --ref "$PR_BASE" \
            --field push_to_registry=true \
            --field build_base_images=false \
            --field triggered_by_script="$_TBS" \
            --field triggered_by_agent="$_TBA" \
            --field triggered_by_card="$_TBC" \
            --field triggered_by_reason="$_TBR" >/dev/null; then
        if [ "$DRY_RUN" = "true" ]; then
            log "would-trigger ${BUILD_WORKFLOW} for ${PR_BASE} (PR #${PR_NUMBER}, attempt ${attempt})"
        else
            log "✅ ${BUILD_WORKFLOW} triggered for ${PR_BASE} (PR #${PR_NUMBER}, attempt ${attempt})"
        fi
        exit 0
    fi
    log "⚠️ gh workflow run attempt ${attempt}/${MAX_ATTEMPTS} failed — checking for race-condition run"
    if [ "$DRY_RUN" = "true" ]; then
        # DRY-RUN mode: skip dedup (gh run list would also hit real API).
        log "   (DRY-RUN: skipping gh run list dedup check)"
    else
        race_check="$(verify_recent_run "$DEDUP_WINDOW")"
        if [ "$race_check" = "ok" ]; then
            log "✅ recent run detected via gh run list — race condition; treating as success (attempt ${attempt})"
            exit 0
        fi
        log "   no recent run → real API failure"
    fi
    if [ "$attempt" -lt "$MAX_ATTEMPTS" ]; then
        sleep $((attempt*5))
    fi
done

log "❌ ${BUILD_WORKFLOW} trigger failed after ${MAX_ATTEMPTS} attempts (PR #${PR_NUMBER} → ${PR_BASE})"
log "   merge-gate continues; manual retry: gh workflow run ${BUILD_WORKFLOW} --repo ${GH_REPO} --ref ${PR_BASE} -f triggered_by_script=manual -f triggered_by_agent=human -f triggered_by_reason=\"manual retry\""
exit 0  # non-fatal: merge-gate не должен падать из-за build trigger
