#!/usr/bin/env bash
# ============================================================================
# pmb_merge_gate_probe.sh — прогон РЕАЛЬНОГО agent-flow-merge-gate.sh с
# mocked-GitHub фикстурой MERGED-PR, чтобы проверить: дёргает ли merge-gate
# agent-flow-post-merge-build.sh (issue #2294 / ADR-AF-0064).
#
# Зачем отдельный скрипт, а не функция внутри test_post_merge_build_skip.sh:
#   lib/mock_env.sh приносит СВОЙ счётчик тестов и свои assert_* — если
#   сорсить его в основной тест, он перетрёт локальные PASS/FAIL и assert'ы.
#   Здесь мы изолируем харнес в отдельном процессе: probe печатает результат
#   в машиночитаемом виде, вызывающий тест ассертит.
#
# Использование:
#   bash lib/pmb_merge_gate_probe.sh <pr_base>
#
# Печатает в stdout (по строке на факт):
#   PMB_CALLS=<N>          — сколько раз merge-gate вызвал post-merge-build.sh
#   SKIP_LOG=<0|1>         — есть ли лог-маркер "skipping post-merge build"
#   RECONCILE=<0|1>        — вошёл ли merge-gate в post-merge reconcile блок
# Exit 0 всегда (кроме внутренней ошибки харнеса → exit 1 + сообщение в stderr).
# ============================================================================
set -uo pipefail

PR_BASE="${1:?usage: pmb_merge_gate_probe.sh <pr_base>}"

PROBE_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=mock_env.sh
. "$PROBE_LIB_DIR/mock_env.sh"

new_test

# Позволяем указать альтернативный merge-gate (mutation control в тесте:
# «а поймает ли probe возврат вызова post-merge-build?»). По умолчанию —
# реальный скрипт репозитория, который выставил mock_env.sh.
MERGE_GATE="${PMB_MERGE_GATE_OVERRIDE:-$MERGE_GATE}"
if [ ! -f "$MERGE_GATE" ]; then
    printf 'pmb_merge_gate_probe: merge-gate не найден: %s\n' "$MERGE_GATE" >&2
    exit 1
fi

ISSUE=2294
PR=2295
TITLE="probe merge gate post merge build"
BRANCH="z-{agent}/${ISSUE}-$(printf '%s' "$TITLE" \
    | tr '[:upper:]' '[:lower:]' \
    | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
    | cut -c1-40)"

# --- Фикстура: MERGED PR с e2e-done, база = $PR_BASE ----------------------
set_state ISSUE_LIST_JSON "[{\"number\":${ISSUE},\"title\":\"${TITLE}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_ade2294\"}]"
set_state "ISSUE_${ISSUE}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
set_state "ISSUE_${ISSUE}_STATE_JSON" '{"state":"OPEN"}'
set_state "ISSUE_${ISSUE}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_ade2294\\n\"}]}"
set_state "ISSUE_${ISSUE}_COMMENTS_SINCE_JSON" '[]'
set_state "ISSUE_${ISSUE}_TIMELINE_JSON" '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-09-09T20:00:00Z"}]'
set_state "PR_HEAD_${BRANCH}_JSON" "[{\"number\":${PR},\"state\":\"MERGED\",\"baseRefName\":\"${PR_BASE}\",\"mergedAt\":\"2026-09-09T20:00:00Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] probe #${ISSUE}\",\"labels\":[{\"name\":\"agent:devops\"}]}]"
set_state "PR_${PR}_COMMITS_JSON" '[{"commit":{"committer":{"date":"2026-09-09T20:00:00Z"}}}]'
set_state PR_LIST_ALL_OPEN_JSON '[]'
set_state PR_FOLLOWUP_JSON '[]'
set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
set_state KANBAN_LIST_JSON "[{\"id\":\"t_ade2294\",\"status\":\"done\"}]"
set_state "BRANCH_PRESENT_${BRANCH}" 1

# --- REPO_DIR со стабом post-merge-build.sh -------------------------------
# Мёртвая ветка (до issue #2294) звала именно
# `bash "${REPO_DIR}/scripts/agent_flow/agent-flow-post-merge-build.sh"`.
# Если её вернут — стаб запишет вызов, и probe отдаст PMB_CALLS>0.
PROBE_REPO="$TEST_TMP/repo"
PMB_JOURNAL="$TEST_TMP/pmb.journal"
mkdir -p "$PROBE_REPO/scripts/agent_flow"
: >"$PMB_JOURNAL"
{
    echo '#!/bin/bash'
    printf 'echo "POST_MERGE_BUILD_CALLED pr=$1 base=$2" >> %q\n' "$PMB_JOURNAL"
    echo 'exit 0'
} >"$PROBE_REPO/scripts/agent_flow/agent-flow-post-merge-build.sh"
chmod +x "$PROBE_REPO/scripts/agent_flow/agent-flow-post-merge-build.sh"
export REPO_DIR="$PROBE_REPO"

run_merge_gate >/dev/null 2>&1 || true

STDERR_LOG="$TEST_TMP/stderr.log"
[ -f "$STDERR_LOG" ] || : >"$STDERR_LOG"

pmb_calls="$(grep -c 'POST_MERGE_BUILD_CALLED' "$PMB_JOURNAL" 2>/dev/null || true)"
skip_log="$(grep -c 'skipping post-merge build' "$STDERR_LOG" 2>/dev/null || true)"
reconcile="$(grep -c 'post-merge reconcile (ADR-0014)' "$STDERR_LOG" 2>/dev/null || true)"

printf 'PMB_CALLS=%s\n' "${pmb_calls:-0}"
printf 'SKIP_LOG=%s\n' "$([ "${skip_log:-0}" -gt 0 ] && echo 1 || echo 0)"
printf 'RECONCILE=%s\n' "$([ "${reconcile:-0}" -gt 0 ] && echo 1 || echo 0)"

# Отладка: PMB_PROBE_KEEP=1 оставляет TEST_TMP и печатает его путь в stderr
# (нужно, когда фикстура «не доезжает» до reconcile-блока).
if [ "${PMB_PROBE_KEEP:-0}" = "1" ]; then
    printf 'PMB_PROBE_TMP=%s\n' "$TEST_TMP" >&2
    exit 0
fi

rm -rf "$TEST_TMP" 2>/dev/null || true
exit 0
