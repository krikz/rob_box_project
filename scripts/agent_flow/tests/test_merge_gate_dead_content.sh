#!/bin/bash
# ============================================================================
# test_merge_gate_dead_content.sh — ретро 22.08 t_e8d52cb7 acceptance tests
#
# Verifies the dead-content guard in agent-flow-merge-gate.sh:
#   Сценарий: после rebase develop ушёл вперёд, в PR остались ТОЛЬКО
#   binary/asset файлы (.ogg/.png/.bin/.wav/.jpg/...) — все meaningful
#   изменения (код/тесты/конфиги) уже в develop. PR висит OPEN +
#   MERGEABLE + CLEAN, но merge-ui показывает его как нормальный → merge
#   протащит только asset-мусор или пустой merge-commit. Кейс PR #1507:
#   6 voice .ogg vs develop = 0 meaningful файлов.
#
# Guard: если PR diff НЕ содержит meaningful файлов (.py|.json|.yaml|.yml|
# .toml|.md|.sh|.ts|.cpp|.h|.hpp|.launch.xml|.txt) И файлы вообще есть
# (не пустой PR — это регрессия-guard, см. L3) → помечаем PR меткой
# `dead-content` + comment-on-issue (24h dedup).
#
# Scenarios covered:
#   L1. PR с только .ogg/.png/.bin → dead-content label установлен, comment есть.
#   L2. PR с .ogg + .json/.md/.py (mixed) → НЕ dead-content, regression.
#   L3. PR пустой (0 файлов) → НЕ dead-content, regression-guard.
#   L4. PR с только .md → НЕ dead-content, regression.
#   L5. PR с dead-content, второй тик — идемпотентность (label/comment dedup).
#   L6. PR мёртвый по state (CLOSED) → dead-content guard НЕ срабатывает.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_dead_content.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Журнал для отладки — оставьте /tmp/dc-debug.* доступным после прогона.
TEST_TMP="${TEST_TMP:-/tmp/dc-debug.$$}"
mkdir -p "$TEST_TMP"
mkdir -p "/tmp/dc-debug"

# assert_ge — локальная копия из test_merge_gate_stale_rebase.sh
# (там file-private, не в lib/). Нужна для проверок "вызовов >= 1".
assert_ge() {  # $1=actual $2=expected $3=msg
    if [ "$1" -lt "$2" ] 2>/dev/null; then
        printf '  %sassert fail:%s %s\n    expected >= %s\n    actual:    %s\n' \
            "$RED" "$END" "$3" "$2" "$1" >&2
        return 1
    fi
}

# slugify_branch — локальная копия из test_merge_gate_stale_rebase.sh
# (file-private в lib/, не вынесен в общий). Используется для построения
# имени ветки, под которым mock_env отдаёт PR_HEAD_<branch>_JSON.
slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ---------------------------------------------------------------------------
# Helper: minimal valid issue+PR fixture for dead-content tests.
# Отличается от fixture_stale_rebase_pr тем, что:
#   - PR CLEAN + MERGEABLE (нормальный "всё зелёное" PR).
#   - PR_<n>_FILES_JSON хранит JSON-объект {"files":[{"path":"..."}]} —
#     merge-gate вызывает `gh pr view N --json files --jq '[.files[].path]'`,
#     mock_env применяет фильтр `\[.files[].path\]` к объекту с ключом
#     `files` (см. mock_env.sh:374). Top-level массив НЕ пройдёт — нужно
#     обернуть в `{"files": [...]}`.
#   - PR side labels не содержат dead-content по умолчанию (L1 проверит
#     установку метки).
# ---------------------------------------------------------------------------
fixture_dead_content_pr() {  # $1=issue $2=pr $3=branch $4=files_array_body (e.g. '[{"path":"a.ogg"}]')
    local issue="$1" pr="$2" branch="$3" files_array_body="$4"
    local title="fix #${issue} dead content demo"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dc${issue}\\n\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dc${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # PR CLEAN + MERGEABLE + green CI. Не содержит dead-content в labels —
    # guard должен поставить его сам (для L1) или НЕ ставить (для L2-L4).
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":50,\"deletions\":5,\"commits\":[{},{}]}]"
    # ОБЁРТКА в {"files": [...]} — mock_env применяет фильтр [.files[].path]
    # к объекту с ключом `files` (см. mock_env.sh line ~371-380).
    set_state "PR_${pr}_FILES_JSON" "{\"files\":${files_array_body}}"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_dc${issue}\",\"status\":\"running\"}]"
}

# ---------------------------------------------------------------------------
# L1. PR с только .ogg/.png/.bin → dead-content label + comment-on-issue.
# ---------------------------------------------------------------------------
test_L1_only_binary_files_labeled_dead_content() {
    new_test
    local branch
    branch="$(slugify_branch 3401 'fix #3401 dead content demo')"
    fixture_dead_content_pr 3401 3402 "$branch" '[{"path":"voice/rabbit.ogg"},{"path":"img/screenshot.png"},{"path":"data/blob.bin"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Dead-content log emitted. merge-gate `log()` идёт в stderr (см.
    # run_merge_gate в mock_env.sh:868), поэтому читаем stderr.log.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    local dc_logs
    dc_logs="$(printf '%s\n' "$stderr_log" | grep -c 'DEAD-CONTENT detected' || true)"
    assert_ge "$dc_logs" "1" "L1: only binary files → DEAD-CONTENT detected log emitted"

    # gh pr edit add-label dead-content вызван.
    local label_calls
    label_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3402.*dead-content' || true)"
    assert_ge "$label_calls" "1" "L1: dead-content label added to PR #3402"

    # gh issue comment с DEAD-CONTENT detected вызван.
    local issue_comments
    issue_comments="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3401 .*DEAD-CONTENT detected' || true)"
    assert_eq "1" "$issue_comments" "L1: dead-content comment-on-issue posted"

    # Guard НЕ должен пропустить PR дальше в needs-e2e.
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3401 --add-label needs-e2e' || true)"
    assert_eq "0" "$needs_e2e_calls" "L1: dead-content PR НЕ попадает в needs-e2e rotation"
}

# ---------------------------------------------------------------------------
# L2. PR с .ogg + .json/.md/.py (mixed) → НЕ dead-content, regression.
# ---------------------------------------------------------------------------
test_L2_mixed_files_not_dead_content() {
    new_test
    local branch
    branch="$(slugify_branch 3403 'fix #3403 dead content demo')"
    fixture_dead_content_pr 3403 3404 "$branch" '[{"path":"voice/rabbit.ogg"},{"path":"scripts/foo.py"},{"path":"tests/test_bar.json"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ должно быть DEAD-CONTENT detected log.
    local dc_logs
    dc_logs="$(printf '%s\n' "$journal" | grep -c 'DEAD-CONTENT detected' || true)"
    assert_eq "0" "$dc_logs" "L2: mixed files → НЕ dead-content"

    # НЕ должно быть dead-content label call.
    local label_calls
    label_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3404.*dead-content' || true)"
    assert_eq "0" "$label_calls" "L2: mixed files → НЕ added dead-content label"

    # Functional PR → needs-e2e должен быть поставлен (обычный флоу).
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3403 --add-label needs-e2e' || true)"
    assert_eq "1" "$needs_e2e_calls" "L2: mixed files → needs-e2e ставится (regression — нормальный флоу)"
}

# ---------------------------------------------------------------------------
# L3. PR пустой (0 файлов) → НЕ dead-content (регрессия-guard для merge-commits).
# ---------------------------------------------------------------------------
test_L3_empty_files_not_dead_content() {
    new_test
    local branch
    branch="$(slugify_branch 3405 'fix #3405 dead content demo')"
    fixture_dead_content_pr 3405 3406 "$branch" '[]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ должно быть DEAD-CONTENT detected log.
    local dc_logs
    dc_logs="$(printf '%s\n' "$journal" | grep -c 'DEAD-CONTENT detected' || true)"
    assert_eq "0" "$dc_logs" "L3: empty PR (0 files) → НЕ dead-content (regression-guard)"

    # НЕ должно быть dead-content label call.
    local label_calls
    label_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3406.*dead-content' || true)"
    assert_eq "0" "$label_calls" "L3: empty PR → НЕ added dead-content label"

    # Пустой PR после rebase (merge commit без изменений) — это нормально,
    # не должно блокироваться нашим guard'ом. Может ставиться needs-e2e.
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3405 --add-label needs-e2e' || true)"
    assert_eq "1" "$needs_e2e_calls" "L3: empty PR → needs-e2e ставится (merge-commit пустой diff)"
}

# ---------------------------------------------------------------------------
# L4. PR с только .md → НЕ dead-content (md = meaningful по acceptance).
# ---------------------------------------------------------------------------
test_L4_only_md_not_dead_content() {
    new_test
    local branch
    branch="$(slugify_branch 3407 'fix #3407 dead content demo')"
    fixture_dead_content_pr 3407 3408 "$branch" '[{"path":"docs/RETRO.md"},{"path":"README.md"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ должно быть DEAD-CONTENT detected log.
    local dc_logs
    dc_logs="$(printf '%s\n' "$journal" | grep -c 'DEAD-CONTENT detected' || true)"
    assert_eq "0" "$dc_logs" "L4: only .md → НЕ dead-content"

    # НЕ должно быть dead-content label call.
    local label_calls
    label_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3408.*dead-content' || true)"
    assert_eq "0" "$label_calls" "L4: only .md → НЕ added dead-content label"
}

# ---------------------------------------------------------------------------
# L5. Идемпотентность: dead-content уже стоит → 2-й тик НЕ дублирует label/comment.
# ---------------------------------------------------------------------------
test_L5_idempotent_no_duplicate_label_or_comment() {
    new_test
    local branch
    branch="$(slugify_branch 3409 'fix #3409 dead content demo')"
    # Override PR labels: dead-content уже стоит. Override comments-since:
    # предыдущий DEAD-CONTENT detected коммент уже есть в timeline.
    fixture_dead_content_pr 3409 3410 "$branch" '[{"path":"voice/rabbit.ogg"},{"path":"img/screenshot.png"}]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":3410,\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #3409 dead content demo\",\"labels\":[{\"name\":\"agent:devops\"},{\"name\":\"dead-content\"}],\"additions\":50,\"deletions\":5,\"commits\":[{},{}]}]"
    # Comment-on-issue 24h dedup: предыдущий DEAD-CONTENT detected уже есть.
    set_state "ISSUE_3409_COMMENTS_SINCE_JSON" '[{"body":"🪦 **PR #3410 DEAD-CONTENT detected** (merge-gate tick, ...)"},{"body":"🪦 **PR #3410 DEAD-CONTENT detected** (merge-gate tick, 00:00:00Z)"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Guard сработал (либо log DEAD-CONTENT detected, либо skip-add-label при
    # уже-стоящей метке — обе ситуации означают, что guard увидел dead-content).
    # merge-gate `log()` пишет в stderr (см. run_merge_gate в mock_env.sh).
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    local dc_logs
    dc_logs="$(printf '%s\n' "$stderr_log" | grep -cE 'DEAD-CONTENT detected|уже имеет dead-content' || true)"
    assert_ge "$dc_logs" "1" "L5: dead-content guard still runs on tick"

    # Метка уже стоит → НЕ должен быть gh pr edit --add-label dead-content.
    local label_calls
    label_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3410 .*--add-label dead-content' || true)"
    assert_eq "0" "$label_calls" "L5: метка dead-content уже есть → skip add-label (idempotent)"

    # Comment уже есть → НЕ должен быть новый gh issue comment DEAD-CONTENT detected.
    local issue_comments
    issue_comments="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3409 .*DEAD-CONTENT detected' || true)"
    assert_eq "0" "$issue_comments" "L5: comment уже есть → dedup skip (24h)"
}

# ---------------------------------------------------------------------------
# L6. PR CLOSED → dead-content guard НЕ срабатывает (state != OPEN).
# ---------------------------------------------------------------------------
test_L6_closed_pr_skipped() {
    new_test
    local branch
    branch="$(slugify_branch 3411 'fix #3411 dead content demo')"
    # Override: PR state=CLOSED — guard требует OPEN.
    set_state ISSUE_LIST_JSON "[{\"number\":3411,\"title\":\"fix #3411 dead content demo\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dc3411\\n\"}]"
    set_state "ISSUE_3411_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_3411_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_3411_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dc3411\\n\"}]}"
    set_state "ISSUE_3411_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_3411_TIMELINE_JSON" '[]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":3412,\"state\":\"CLOSED\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #3411 dead content demo\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":50,\"deletions\":5,\"commits\":[{},{}]}]"
    set_state "PR_3412_FILES_JSON" '{"files":[{"path":"voice/rabbit.ogg"},{"path":"img/screenshot.png"}]}'
    set_state "PR_3412_COMMITS_JSON" '[]'
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state KANBAN_LIST_JSON '[{"id":"t_dc3411","status":"running"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # CLOSED PR — guard не сработал, так как state != OPEN.
    local dc_logs
    dc_logs="$(printf '%s\n' "$journal" | grep -c 'DEAD-CONTENT detected' || true)"
    assert_eq "0" "$dc_logs" "L6: PR CLOSED → guard не срабатывает"

    # CLOSED PR — не должно быть label call.
    local label_calls
    label_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3412.*dead-content' || true)"
    assert_eq "0" "$label_calls" "L6: PR CLOSED → НЕ added dead-content label"
}

# ---------------------------------------------------------------------------
# Запуск всех тестов.
# ---------------------------------------------------------------------------
run_test "L1_only_binary_files_labeled_dead_content" test_L1_only_binary_files_labeled_dead_content
run_test "L2_mixed_files_not_dead_content"           test_L2_mixed_files_not_dead_content
run_test "L3_empty_files_not_dead_content"           test_L3_empty_files_not_dead_content
run_test "L4_only_md_not_dead_content"               test_L4_only_md_not_dead_content
run_test "L5_idempotent_no_duplicate_label_or_comment" test_L5_idempotent_no_duplicate_label_or_comment
run_test "L6_closed_pr_skipped"                      test_L6_closed_pr_skipped
summary
