#!/usr/bin/env bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-post-merge-build.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
# ============================================================================
# agent-flow-post-merge-build.sh — ADR-0022 extension (issue #1475),
# updated 25.08.2026 (issue #1625, Шифу)
#
# Цель: после merge PR в main — запустить L-Build-All-Services чтобы
# .image-versions.prod получил свежие SHA-теги. До этого фикса:
#   - L-Build-All-Services.yml триггерился ТОЛЬКО от workflow_dispatch
#     (e2e-process на round-ветке) И от workflow_call (G-Auto-merge to Main);
#   - push в develop триггерил G-Auto-merge to Main, который создавал
#     PR в main, но НЕ билдил develop HEAD;
#   - результат: после merge любого PR в develop develop-HEAD не
#     пересобирался, и deploy тащил старый .image-voices.dev тег
#     (issue #1475, evidence: PR #1434 merge 18.08 23:00 MSK →
#     на роботе до сих пор dev-ddd09e51 от 18.08 17:49 MSK, 5 часов stale).
#
# Этот скрипт — ADR-0022 extension. Вызывается из
# agent-flow-merge-gate.sh после успешного MERGED detection:
#   log "issue #${number}: PR #${pr_number} MERGED into ${pr_base}"
#   bash scripts/agent_flow/agent-flow-post-merge-build.sh "$pr_number" "$pr_base"
#
# Поведение (после фикса issue #1625, 25.08):
#   1. ENV kill-switch: DISABLE_POST_MERGE_BUILD=1 → exit 0 (hard skip,
#      не дёргает gh ни на какой ветке).
#   2. PR_BASE == develop → exit 0 (Шифу 25.08: develop build — ручной).
#   3. PR_BASE == main → запускает L-Build-All-Services через `gh workflow run`
#      с retry+dedup (production safety: первый push в main — там может быть
#      первая публикация, тег, deploy).
#   4. НЕ блокирует merge-gate: ошибка триггера = warning + continue.
#   5. Идемпотентен: запуск с тем же (pr_number, branch) подряд — окей,
#      GitHub сериализует одинаковые workflow по workflow_name+head_branch.
#
# Pure bash. No LLM. Idempotent.
set +e
# shellcheck source=lib_cron_env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/lib_cron_env.sh" || {
    printf "[%s] %s: lib_cron_env preflight failed — exit 1
" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(basename "${BASH_SOURCE[0]:-$0}")" >&2
    exit 1
}
set -euo pipefail

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

# --- pre-condition: ENV override (issue #1625, Шифу 25.08) ------------------
# Hard kill-switch: если DISABLE_POST_MERGE_BUILD=1 — НЕ дёргаем build ни для
# какой ветки (используется, чтобы вручную приглушить post-merge-build на
# время расследования, без правки merge-gate).
if [ "${DISABLE_POST_MERGE_BUILD:-0}" = "1" ]; then
    log "⏭️ post-merge build DISABLED by env (DISABLE_POST_MERGE_BUILD=1, PR #${PR_NUMBER} → ${PR_BASE})"
    exit 0
fi

# --- pre-condition: только develop/main --------------------------------------
# Issue #1625 (Шифу 25.08): develop build больше не триггерим автоматически —
# develop-HEAD собирается по push-триггеру L-Build-All-Services.yml (если он
# настроен) или вручную когда нужно. Авто-trigger после каждого merge в
# develop дёргал build без надобности (issue #1560). main — production safety,
# build обязателен (там может быть первая публикация/тег).
case "$PR_BASE" in
    "$DEVELOP_BRANCH")
        log "⏭️ skipped post-merge build for ${PR_BASE} (Шифу 25.08: develop builds only by hand)"
        exit 0
        ;;
    "$MAIN_BRANCH")
        log "PR #${PR_NUMBER} → ${PR_BASE}: eligible for post-merge build (production safety)"
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

# Issue #1540: shared dedup-библиотека. verify_recent_run() теперь
# живёт в lib_workflow_dedup.sh и используется обоими скриптами
# (post-merge-build + e2e-process), чтобы дедупликация была общим
# контрактом, а не копи-пастой.
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_workflow_dedup.sh
. "$_LIB_DIR_HERE/lib_workflow_dedup.sh"

MAX_ATTEMPTS="${POST_MERGE_BUILD_MAX_ATTEMPTS:-2}"  # 2 = 1 попытка + 1 retry
DEDUP_WINDOW="${POST_MERGE_BUILD_DEDUP_WINDOW:-60}"  # секунд
# --- pre-dispatch dedup (issue #1535 follow-up, 22.08) ----------------------
# merge-gate вызывает этот backup на КАЖДОМ тике (раз в ~5 мин), пока
# merged-issue не закрыта (нет e2e-done / no-e2e-required). GitHub НЕ
# дедуплицирует workflow_dispatch по workflow_name+branch — каждый вызов
# `gh workflow run` создаёт новый run. Поэтому: если на $PR_BASE уже есть
# свежий build-ран (запущен за последние $RECENT_WINDOW сек, любой статус) —
# push-триггер (или прошлый тик / ручной запуск) уже покрыл этот merge.
# Issue #1540: используем verify_recent_run из shared lib_workflow_dedup.sh
# (4-аргументный контракт) — общий SOT, чтобы e2e-process и post-merge-build
# дедуплицировали одинаково.
RECENT_WINDOW="${POST_MERGE_BUILD_RECENT_WINDOW:-900}"  # 15 мин
if [ "$DRY_RUN" != "true" ] && [ "$(verify_recent_run "$BUILD_WORKFLOW" "$PR_BASE" "$GH_REPO" "$RECENT_WINDOW")" = "ok" ]; then
    log "⏭️ recent build (≤${RECENT_WINDOW}s) already on ${PR_BASE} — skip (pre-dispatch dedup, PR #${PR_NUMBER}, issue #1540 shared lib)"
    exit 0
fi
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
        # Issue #1540: явный передача workflow/branch/repo вместо env-implicit,
        # чтобы контракт был одинаковый с e2e-process (shared lib).
        race_check="$(verify_recent_run "$BUILD_WORKFLOW" "$PR_BASE" "$GH_REPO" "$DEDUP_WINDOW")"
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
