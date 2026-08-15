#!/bin/bash
# ============================================================================
# test_merge_gate_duplicate_file.sh — ретро 15.08 t_20383d32 acceptance tests
#
# Verifies the duplicate-file guard in agent-flow-merge-gate.sh:
#   Сценарий: две ПАРАЛЛЕЛЬНЫЕ карточки пришли к одному корневому фиксу и
#   каждая добавила ОДИН И ТОТ ЖЕ файл с ИДЕНТИЧНЫМ содержимым (одинаковый
#   blob sha) — PR #1262 (t_392d6000 setup.cfg) и PR #1267 (issue #1266
#   setup.cfg) оба добавили src/rob_box_teleop/setup.cfg (blob 66dad822).
#   Guard: для open PR с needs-review/needs-e2e тянет pulls/N/files
#   (filename+sha), находит пару (filename, sha) в РАЗНЫХ PR → инфо-коммент
#   на оба PR (dedup 24h). НЕ блокирует CI и НЕ снимает needs-e2e.
#
# Scenarios covered:
#   A. Два open PR (needs-review) меняют ОДИН файл с ИДЕНТИЧНЫМ blob →
#      duplicate-file коммент постится на ОБА PR.
#   B. Два open PR меняют один файл с РАЗНЫМИ blob → НЕ дубль (нет коммента).
#   C. Один PR, уникальный файл → нет коммента.
#   D. PR без needs-review/needs-e2e меток → файл не сканируется (нет
#      ложного дубля с PR, у которого метки есть).
#   E. Dedup: 2 тика с одним дублем → коммент постится один раз за 24h.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: minimal open-PR fixture for the duplicate-file guard.
#   $1=pr $2=branch $3=labels_csv (нижний регистр) $4=files_json
#   $5=comments_since_json (для dedup-теста)
# ---------------------------------------------------------------------------
fixture_dup_pr() {  # $1=pr $2=branch $3=labels_csv $4=files_json $5=comments_since_json
    local pr="$1" branch="$2" labels_csv="$3" files_json="$4"
    local comments_since="${5:-[]}"
    set_state "PR_${pr}_FILES_JSON" "$files_json"
    set_state "PR_${pr}_COMMENTS_JSON" '[]'
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state "ISSUE_${pr}_COMMENTS_SINCE_JSON" "$comments_since"
    set_state "ISSUE_${pr}_LABELS_JSON" "{\"labels\":[{\"name\":\"hermes\"}]}"
    set_state "ISSUE_${pr}_STATE_JSON" '{"state":"OPEN"}'
    # shellcheck disable=SC2155
    local labels_json
    if [ -z "$labels_csv" ]; then
        labels_json='[]'
    else
        labels_json="[$(printf '%s' "$labels_csv" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | tr '\n' ',' | sed 's/,$//')]"
    fi
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${pr} demo\",\"labels\":${labels_json},\"additions\":10,\"deletions\":0,\"commits\":[{},{}]}]"
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    set_state "BRANCH_PRESENT_${branch}" 1
}

# Fixture: два open PR в PR_LIST_ALL_OPEN_JSON + пустая очередь issues
# (merge-gate уходит в scan-all-prs путь, где вызывается duplicate_file_scan_all).
# ВАЖНО: PR_LIST_ALL_OPEN_JSON обязан нести labels — python-фильтр duplicate-file
# скана отбрасывает PR без needs-review/needs-e2e.
fixture_dup_scan() {  # $1=pr1 $2=pr2 $3=labels1_csv $4=labels2_csv
    local l1 l2
    if [ -z "${3:-}" ]; then l1='[]'; else l1="[$(printf '%s' "$3" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | tr '\n' ',' | sed 's/,$//')]"; fi
    if [ -z "${4:-}" ]; then l2='[]'; else l2="[$(printf '%s' "$4" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | tr '\n' ',' | sed 's/,$//')]"; fi
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":$1,\"headRefName\":\"z-devops/t_dup${1}-a\",\"title\":\"fix #${1} demo\",\"additions\":10,\"deletions\":0,\"labels\":${l1}},{\"number\":$2,\"headRefName\":\"z-{agent}/dup${2}-b\",\"title\":\"fix #${2} demo\",\"additions\":10,\"deletions\":0,\"labels\":${l2}}]"
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "PR_EXISTS_${1}" 1
    set_state "PR_EXISTS_${2}" 1
}

# ===========================================================================
# A. Два open PR (needs-review), ОДИН файл с ИДЕНТИЧНЫМ blob → duplicate-file
#    коммент на ОБА PR.
# ===========================================================================
test_A_identical_blob_comments_both_prs() {
    new_test
    local blob="66dad82270ea249b801b375028dbf54397457319"
    fixture_dup_pr 3301 "z-devops/t_dup3301-a" "needs-review" "[{\"filename\":\"src/rob_box_teleop/setup.cfg\",\"sha\":\"${blob}\"}]"
    fixture_dup_pr 3302 "z-{agent}/dup3302-b" "needs-review" "[{\"filename\":\"src/rob_box_teleop/setup.cfg\",\"sha\":\"${blob}\"}]"
    fixture_dup_scan 3301 3302 "needs-review" "needs-review"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local dup_comments
    dup_comments="$(printf '%s\n' "$journal" | grep -c 'duplicate file detected' || true)"
    assert_eq "2" "$dup_comments" "идентичный blob → коммент на ОБА PR (2 штуки)"

    local pr1_comment pr2_comment
    pr1_comment="$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3301 .*duplicate file detected' || true)"
    pr2_comment="$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3302 .*duplicate file detected' || true)"
    assert_eq "1" "$pr1_comment" "коммент на PR #3301"
    assert_eq "1" "$pr2_comment" "коммент на PR #3302"
}

# ===========================================================================
# B. Два open PR меняют один файл с РАЗНЫМИ blob → НЕ дубль, нет коммента.
# ===========================================================================
test_B_different_blob_no_comment() {
    new_test
    fixture_dup_pr 3311 "z-devops/t_dup3311-a" "needs-review" '[{"filename":"src/rob_box_teleop/setup.cfg","sha":"1111111111111111111111111111111111111111"}]'
    fixture_dup_pr 3312 "z-{agent}/dup3312-b" "needs-review" '[{"filename":"src/rob_box_teleop/setup.cfg","sha":"2222222222222222222222222222222222222222"}]'
    fixture_dup_scan 3311 3312 "needs-review" "needs-review"

    run_merge_gate

    local dup_comments
    dup_comments="$(printf '%s\n' "$(cat "$GH_JOURNAL")" | grep -c 'duplicate file detected' || true)"
    assert_eq "0" "$dup_comments" "разные blob → нет duplicate-file коммента"
}

# ===========================================================================
# C. Один PR с уникальным файлом → нет коммента.
# ===========================================================================
test_C_single_pr_no_comment() {
    new_test
    fixture_dup_pr 3321 "z-devops/t_dup3321-a" "needs-review" '[{"filename":"src/rob_box_voice/setup.cfg","sha":"3333333333333333333333333333333333333333"}]'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[{"number":3321,"headRefName":"z-devops/t_dup3321-a","title":"fix #3321 demo","additions":10,"deletions":0}]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "PR_EXISTS_3321" 1

    run_merge_gate

    local dup_comments
    dup_comments="$(printf '%s\n' "$(cat "$GH_JOURNAL")" | grep -c 'duplicate file detected' || true)"
    assert_eq "0" "$dup_comments" "один PR → нет duplicate-file коммента"
}

# ===========================================================================
# D. PR без needs-review/needs-e2e меток → не сканируется: файл не тянется,
#    ложного дубля с PR-с-метками нет.
# ===========================================================================
test_D_unlabeled_pr_not_scanned() {
    new_test
    local blob="66dad82270ea249b801b375028dbf54397457319"
    # 3341 — с меткой needs-review; 3342 — БЕЗ меток, но с тем же файлом.
    fixture_dup_pr 3341 "z-devops/t_dup3341-a" "needs-review" "[{\"filename\":\"src/rob_box_teleop/setup.cfg\",\"sha\":\"${blob}\"}]"
    fixture_dup_pr 3342 "z-{agent}/dup3342-b" "" "[{\"filename\":\"src/rob_box_teleop/setup.cfg\",\"sha\":\"${blob}\"}]"
    fixture_dup_scan 3341 3342 "needs-review" ""

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Файлы PR #3342 (без меток) НЕ запрашиваются → нет ложного дубля.
    local files_3342
    files_3342="$(printf '%s\n' "$journal" | grep -c 'gh api .*pulls/3342/files' || true)"
    assert_eq "0" "$files_3342" "PR без needs-review/needs-e2e → файлы не тянутся"

    local dup_comments
    dup_comments="$(printf '%s\n' "$journal" | grep -c 'duplicate file detected' || true)"
    assert_eq "0" "$dup_comments" "немаркированный PR не участвует в duplicate-file детекте"
}

# ===========================================================================
# E. Dedup: 2 тика с одним дублем → коммент постится один раз за 24h.
# ===========================================================================
test_E_duplicate_comment_dedup() {
    new_test
    local blob="66dad82270ea249b801b375028dbf54397457319"
    fixture_dup_pr 3351 "z-devops/t_dup3351-a" "needs-review" "[{\"filename\":\"src/rob_box_teleop/setup.cfg\",\"sha\":\"${blob}\"}]"
    fixture_dup_pr 3352 "z-{agent}/dup3352-b" "needs-review" "[{\"filename\":\"src/rob_box_teleop/setup.cfg\",\"sha\":\"${blob}\"}]"
    fixture_dup_scan 3351 3352 "needs-review" "needs-review"

    run_merge_gate
    local first_journal
    first_journal="$(cat "$GH_JOURNAL")"

    # Второй тик: в COMMENTS_SINCE уже есть наш duplicate-file коммент.
    set_state "ISSUE_3351_COMMENTS_SINCE_JSON" '[{"body":"⚠️ **duplicate file detected** (merge-gate, ретро 15.08 t_20383d32)"}]'
    set_state "ISSUE_3352_COMMENTS_SINCE_JSON" '[{"body":"⚠️ **duplicate file detected** (merge-gate, ретро 15.08 t_20383d32)"}]'
    run_merge_gate
    local second_journal
    second_journal="$(cat "$GH_JOURNAL")"

    local first_comments second_comments
    first_comments="$(printf '%s\n' "$first_journal" | grep -c 'duplicate file detected' || true)"
    second_comments="$(printf '%s\n' "$second_journal" | grep -c 'duplicate file detected' || true)"
    assert_eq "2" "$first_comments" "tick 1: duplicate-file коммент на оба PR"
    assert_eq "2" "$second_comments" "tick 2: duplicate-file коммент НЕ повторяется (dedup)"
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "A. identical blob → comment on both PRs" test_A_identical_blob_comments_both_prs
run_test "B. different blob → no comment" test_B_different_blob_no_comment
run_test "C. single PR → no comment" test_C_single_pr_no_comment
run_test "D. unlabeled PR not scanned" test_D_unlabeled_pr_not_scanned
run_test "E. duplicate comment dedup" test_E_duplicate_comment_dedup

summary
