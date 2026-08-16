#!/bin/bash
# ============================================================================
# test_merge_gate_deploy_reconcile.sh — ретро 15.08 t_238ff3f7 acceptance tests
#
# Verifies the deploy-issue label-less orphan backstop in
# agent-flow-merge-gate.sh (deploy_issue_reconcile_all):
#   Сценарий: L-Deploy and Verify создаёт deploy-issues с версией workflow-файла
#   С ВЕТКИ e2e-раунда (z-{e2e}/test-round-N). Если round-ветка ответвилась ДО
#   фикса #1263 (hermes+agent:devops при создании), issue получает только метку
#   `deployment` → агентский триаж (фильтр по hermes) карточку не создаёт →
#   issue висит open навсегда (#1276, round-116). Backstop: open
#   deployment-issue без process-меток старше DEPLOY_RECONCILE_MINUTES (30м) →
#   добавить hermes + agent:devops → триаж создаст карточку.
#
# Scenarios covered:
#   A. deployment issue без hermes-метки, старше 30м → hermes+agent:devops
#      добавлены + коммент.
#   B. deployment issue УЖЕ с hermes-меткой → skip (идемпотентность).
#   C. deployment issue свежий (< 30м) → skip (workflow ещё может метить сам).
#   D. коммент-дедупликация: в issue уже есть Авто-reconcile коммент за 24h →
#      метки ставятся, второй коммент НЕ постится.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_deploy_reconcile.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: minimal fixture for the deploy-issue reconcile.
# ISSUE_LIST_JSON = [] (нет hermes-issues → no-issues путь merge-gate).
# ISSUE_LIST_DEPLOYMENT_JSON = deployment-issues (то, что видит reconcile).
# ---------------------------------------------------------------------------
fixture_deploy_issue() {  # $1=issue $2=labels_json $3=updatedAt $4=comments_since_json
    local issue="$1" labels="$2" updated="$3" comments_since="${4:-[]}"
    set_state ISSUE_LIST_JSON '[]'
    set_state ISSUE_LIST_DEPLOYMENT_JSON "[{\"number\":${issue},\"title\":\"🚨 Deploy issues on z-{e2e}/test-round-116 (test) — 2026-08-15\",\"labels\":${labels},\"updatedAt\":\"${updated}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":${labels}}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" "$comments_since"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ===========================================================================
# A. deployment issue без hermes-метки, старше 30м → hermes+agent:devops
# ===========================================================================
test_A_deploy_issue_orphan_labeled() {
    new_test
    # updatedAt: 2 часа назад (> DEPLOY_RECONCILE_MINUTES=30м)
    local old_ts
    old_ts="$(date -u -d '2 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-15T05:00:00Z')"
    fixture_deploy_issue 1276 '[{"name":"deployment"}]' "$old_ts"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local add_hermes add_devops
    add_hermes="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1276 --add-label hermes' || true)"
    add_devops="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1276 --add-label agent:devops' || true)"
    assert_eq "1" "$add_hermes" "orphan deploy-issue → hermes label added" || return 1
    assert_eq "1" "$add_devops" "orphan deploy-issue → agent:devops label added" || return 1

    local comment
    comment="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1276' || true)"
    assert_eq "1" "$comment" "orphan deploy-issue → reconcile comment posted" || return 1
}

# ===========================================================================
# B. deployment issue уже с hermes → skip (идемпотентность)
# ===========================================================================
test_B_deploy_issue_already_labeled_skipped() {
    new_test
    local old_ts
    old_ts="$(date -u -d '2 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-15T05:00:00Z')"
    fixture_deploy_issue 1276 '[{"name":"deployment"},{"name":"hermes"},{"name":"agent:devops"}]' "$old_ts"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local edits
    edits="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1276 --add-label' || true)"
    assert_eq "0" "$edits" "deploy-issue с hermes → labels NOT re-added" || return 1
}

# ===========================================================================
# C. свежий deployment issue (< DEPLOY_RECONCILE_MINUTES) → skip
# ===========================================================================
test_C_deploy_issue_fresh_skipped() {
    new_test
    local fresh_ts
    fresh_ts="$(date -u -d '5 minutes ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo "$(date -u +%Y-%m-%dT%H:%M:%SZ)")"
    fixture_deploy_issue 1276 '[{"name":"deployment"}]' "$fresh_ts"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local edits
    edits="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1276 --add-label' || true)"
    assert_eq "0" "$edits" "свежий deploy-issue (< 30м) → labels NOT added" || return 1
}

# ===========================================================================
# D. коммент-дедупликация: Авто-reconcile коммент уже есть за 24h → метки
#    ставятся, но второй коммент НЕ постится.
# ===========================================================================
test_D_deploy_issue_comment_dedup() {
    new_test
    local old_ts
    old_ts="$(date -u -d '2 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-15T05:00:00Z')"
    local existing_comment
    existing_comment='[{"body":"🏷️ **Авто-reconcile** (merge-gate, ретро 15.08 t_238ff3f7): deployment-issue без hermes-метки"}]'
    fixture_deploy_issue 1276 '[{"name":"deployment"}]' "$old_ts" "$existing_comment"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local edits comment
    edits="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1276 --add-label' || true)"
    comment="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1276' || true)"
    assert_eq "2" "$edits" "dedup: метки всё равно добавляются (hermes+agent:devops)" || return 1
    assert_eq "0" "$comment" "dedup: повторный reconcile-коммент НЕ постится" || return 1
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "A. orphan deploy-issue → hermes+agent:devops" test_A_deploy_issue_orphan_labeled
run_test "B. deploy-issue с hermes → skip" test_B_deploy_issue_already_labeled_skipped
run_test "C. свежий deploy-issue → skip" test_C_deploy_issue_fresh_skipped
run_test "D. comment dedup 24h" test_D_deploy_issue_comment_dedup

summary
