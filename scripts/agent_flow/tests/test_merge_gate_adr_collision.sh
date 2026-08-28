#!/usr/bin/env bash
# ============================================================================
# test_merge_gate_adr_collision.sh — ретро 25.08 t_00ba0224
#
# Регресс-тест для guard'а check_adr_number_collision() в agent-flow-merge-gate.sh.
# Сценарий: 5 файлов под 3 номерами ADR (0027×3, 0028×2) на origin/develop —
# глобальная коллизия ломает обратные ссылки на ADR (документы ссылаются на
# «0027-foo», а в develop уже «0027-bar» → битая ссылка).
#
# Тест покрывает 8 кейсов acceptance:
#   A. PR добавляет 0031-foo.md (0031 свободен) → PASS (return 0, no-op).
#   B. PR добавляет 0027-bar.md при существующем в develop 0027-baz.md
#      → REJECT (return 1) + comment + label.
#   C. PR переименовывает 0028 → 0030 (delete 0028 + add 0030)
#      при существующем в develop 0028-baz.md → PASS (0028 в PR, не коллизия).
#   D. PR только правит текст существующего 0027-foo.md → PASS.
#   E. Override-метка adr-collision-override на issue → PASS (Шифу одобрил).
#   F. Comment dedup 24h: повторный tick не постит comment ещё раз.
#   G. Fail-open: PR_FILES_JSON пустой (gh flake) → return 0.
#   H. Fail-open: DEV_ADR_FILES пустой (tree упал) → return 0.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_adr_collision.sh
# ============================================================================
set -uo pipefail  # НЕ pipefail: подсчёт rc через `|| rc=$?` ломается с ним

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_LIB_DIR/.." && pwd)"

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Source guard + его прямые зависимости из agent-flow-merge-gate.sh.
#
# Стратегия: парсим awk'ом ТОЛЬКО функции, от которых зависит guard
# (log, has_label, check_adr_number_collision) и явно задаём нужные
# глобальные переменные. Так тест изолирован от main-блока merge-gate
# (не запускает весь gate) и автоматически подхватывает любые правки
# production-кода (без copy-paste).
#
# Если кто-то поправит guard в merge-gate, но не поправит тест — этот
# тест СРАЗУ покажет drift при первом прогоне.
# ---------------------------------------------------------------------------
extract_func() {  # $1=script_path $2=func_signature $3=out_var
    local script="$1" sig="$2" outvar="$3" body
    body="$(awk -v sig="$sig" '
        $0 ~ "^" sig "[[:space:]]*\\(\\)" {flag=1}
        flag {print}
        flag && /^}/ {flag=0; exit}
    ' "$script")"
    if [ -z "$body" ]; then
        printf 'FAIL: func %s не найдена в %s\n' "$sig" "$script" >&2
        return 1
    fi
    printf -v "$outvar" '%s' "$body"
}

extract_func "$REPO_ROOT/agent-flow-merge-gate.sh" "log"                          MG_LOG
extract_func "$REPO_ROOT/agent-flow-merge-gate.sh" "has_label"                    MG_HAS_LABEL
extract_func "$REPO_ROOT/agent-flow-merge-gate.sh" "check_adr_number_collision"   MG_GUARD

unset -f log has_label check_adr_number_collision 2>/dev/null || true
eval "$MG_LOG" >/dev/null
eval "$MG_HAS_LABEL" >/dev/null
eval "$MG_GUARD" >/dev/null

# Глобалы, на которые ссылается guard. Задаём те же defaults, что в merge-gate.
LOG_PREFIX="[merge-gate]"
DEVELOP_BRANCH="develop"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
NEEDS_E2E_LABEL="needs-e2e"
ADR_COLLISION_OVERRIDE_LABEL="adr-collision-override"
ADR_COLLISION_BLOCKED_LABEL="agent-flow:adr-collision"
ADR_COLLISION_COMMENT_DEDUP_HOURS="24"

# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------
install_mocks_for_test() {
    TEST_TMP="$(mktemp -d /tmp/agent-flow-adr-collision-tests.XXXXXX)"
    install_mocks
    : >"$TEST_TMP/stderr.log"
    export GH_STATE GH_JOURNAL
}

# Заполняем DEV_ADR_FILES в $GH_STATE (формат: имя без префикса
# docs/adr/, например «0027-baz.md»). Mock `git ls-tree` в lib/mock_env.sh
# автоматически оборачивает в «docs/adr/0027-baz.md» для совместимости с
# production grep'ом в guard'е (который ожидает формат реального
# `git ls-tree origin/develop`).

# Build PR_<n>_FILES_JSON (для gh pr view --json files).
# $1=pr $2=newline-list путей или JSON-массив.
set_pr_files() {
    local pr="$1" input="$2" built
    if [ "${input:0:1}" = "[" ]; then
        built="$(printf '%s' "$input" | python3 -c '
import json,sys
arr=json.load(sys.stdin)
print(json.dumps([{"path":p} for p in arr]))
')"
    else
        built="$(printf '%s\n' "$input" | python3 -c '
import json,sys
lines=[l.strip() for l in sys.stdin if l.strip()]
print(json.dumps([{"path":p} for p in lines]))
')"
    fi
    set_state "PR_${pr}_FILES_JSON" "{\"files\":$built}"
}

# Issue labels (comma-separated, lower-case) + comments-since JSON.
set_issue_state() {
    local issue="$1" labels="$2" comments="$3" labels_json='{"labels":[' first=1 l
    if [ -n "$labels" ]; then
        IFS=',' read -ra LARR <<< "$labels"
        for l in "${LARR[@]}"; do
            [ -z "$l" ] && continue
            [ "$first" -eq 0 ] && labels_json+=','
            labels_json+="{\"name\":\"$l\"}"
            first=0
        done
    fi
    labels_json+=']}'
    set_state "ISSUE_${issue}_LABELS_JSON" "$labels_json"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" "$comments"
}

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_eq() {
    local want="$1" got="$2" msg="$3"
    if [ "$want" != "$got" ]; then
        printf '    %sassert fail:%s want=%q got=%q (%s)\n' "$RED" "$END" "$want" "$got" "$msg" >&2
        return 1
    fi
}

assert_ge() {
    local actual="$1" want="$2" msg="$3"
    if [ "${actual:-0}" -lt "${want}" ] 2>/dev/null; then
        printf '    %sassert fail:%s want>=%s got=%s (%s)\n' "$RED" "$END" "$want" "$actual" "$msg" >&2
        return 1
    fi
}

# ---------------------------------------------------------------------------
# A. PR добавляет 0031-foo.md (0031 свободен) → PASS, no-op
# ---------------------------------------------------------------------------
test_A_new_adr_free_number() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md
0028-baz.md
0030-foo.md"
    set_pr_files 4002 '["docs/adr/0031-foo.md"]'
    set_issue_state 4001 "" "[]"

    local rc=0
    check_adr_number_collision 4002 4001 "" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (no collision)"
    local n_comments
    n_comments="$(grep -c 'gh issue comment 4001' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no comment posted"
    local n_labels
    n_labels="$(grep -c 'gh issue edit 4001 --add-label agent-flow:adr-collision' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_labels" "no block-label added"
}

# ---------------------------------------------------------------------------
# B. PR добавляет 0027-bar.md при существующем 0027-baz.md → REJECT
# ---------------------------------------------------------------------------
test_B_collision_blocks_pr() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md
0028-baz.md"
    set_pr_files 4012 '["docs/adr/0027-bar.md"]'
    set_issue_state 4011 "" "[]"

    local rc=0
    check_adr_number_collision 4012 4011 "" || rc=$?

    assert_eq "1" "$rc" "guard returns 1 (collision)"
    local comment_count comment_text has_0027 has_baz
    comment_count="$(grep -c 'gh issue comment 4011' "$GH_JOURNAL" || true)"
    comment_text="$(grep 'gh issue comment 4011' "$GH_JOURNAL" | grep -c 'ADR-COLLISION detected' || true)"
    has_0027="$(grep 'gh issue comment 4011' "$GH_JOURNAL" | grep -c '0027' || true)"
    has_baz="$(grep 'gh issue comment 4011' "$GH_JOURNAL" | grep -c '0027-baz.md' || true)"
    assert_ge "$comment_count" "1" "comment posted on issue"
    assert_ge "$comment_text" "1" "comment has ADR-COLLISION marker"
    assert_ge "$has_0027" "1" "comment mentions NNNN=0027"
    assert_ge "$has_baz" "1" "comment mentions existing 0027-baz.md"
    local n_label
    n_label="$(grep -c 'gh issue edit 4011 --add-label agent-flow:adr-collision' "$GH_JOURNAL" || true)"
    assert_eq "1" "$n_label" "block label added"
}

# ---------------------------------------------------------------------------
# C. Rename 0028 → 0030: PR трогает 0028-foo.md + добавляет 0030-foo.md.
#    В develop: 0027-baz.md, 0028-baz.md. 0028 занят, но файл в PR (0028-foo.md)
#    НЕ совпадает с develop (0028-baz.md) → guard видит что 0028 в PR и не
#    триггерит коллизию. 0030 свободен → no-op.
# ---------------------------------------------------------------------------
test_C_rename_collision_self_passes() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md
0028-baz.md"
    set_pr_files 4022 '["docs/adr/0028-foo.md","docs/adr/0030-foo.md"]'
    set_issue_state 4021 "" "[]"

    local rc=0
    check_adr_number_collision 4022 4021 "" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (rename: 0028 in PR, 0030 free)"
    local n_comments
    n_comments="$(grep -c 'gh issue comment 4021' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no comment posted"
}

# ---------------------------------------------------------------------------
# D. PR только правит существующий 0027-foo.md → PASS (правка не «новый номер»).
#    В develop: ТОЛЬКО 0027-foo.md (нет других файлов под этим номером → нет
#    коллизии). Если в develop есть 0027-baz.md — это коллизия (см. test_I).
# ---------------------------------------------------------------------------
test_D_only_text_edit_passes() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-foo.md
0028-baz.md"
    set_pr_files 4032 '["docs/adr/0027-foo.md"]'
    set_issue_state 4031 "" "[]"

    local rc=0
    check_adr_number_collision 4032 4031 "" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (existing edit, no other NNNN files)"
    local n_comments
    n_comments="$(grep -c 'gh issue comment 4031' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no comment posted"
}

# ---------------------------------------------------------------------------
# I. Edge-case: PR правит 0027-foo.md, в develop есть 0027-baz.md (другой
#    файл под тем же номером) → REJECT. После merge в develop будут оба
#    файла под 0027 (битые ссылки на «0027-foo» резолвятся в один из них).
# ---------------------------------------------------------------------------
test_I_edit_while_another_exists_blocks() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md
0027-foo.md"
    set_pr_files 4082 '["docs/adr/0027-foo.md"]'
    set_issue_state 4081 "" "[]"

    local rc=0
    check_adr_number_collision 4082 4081 "" || rc=$?

    assert_eq "1" "$rc" "guard returns 1 (0027-baz.md still in develop)"
    local n_label
    n_label="$(grep -c 'gh issue edit 4081 --add-label agent-flow:adr-collision' "$GH_JOURNAL" || true)"
    assert_eq "1" "$n_label" "block label added"
}

# ---------------------------------------------------------------------------
# E. Override-метка adr-collision-override на issue → PASS (Шифу одобрил).
# ---------------------------------------------------------------------------
test_E_override_label_passes() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md"
    set_pr_files 4042 '["docs/adr/0027-bar.md"]'
    set_issue_state 4041 "adr-collision-override" "[]"

    local rc=0
    check_adr_number_collision 4042 4041 "adr-collision-override" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (override)"
    local n_comments n_labels
    n_comments="$(grep -c 'gh issue comment 4041' "$GH_JOURNAL" || true)"
    n_labels="$(grep -c 'gh issue edit 4041 --add-label agent-flow:adr-collision' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no comment (override)"
    assert_eq "0" "$n_labels" "no block-label (override)"
}

# ---------------------------------------------------------------------------
# F. Comment dedup 24h: уже есть ADR-COLLISION comment → skip.
# ---------------------------------------------------------------------------
test_F_comment_dedup_24h() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md"
    set_pr_files 4052 '["docs/adr/0027-bar.md"]'
    set_issue_state 4051 "" '[{"body":"🚨 PR #4052 ADR-COLLISION detected (предыдущий тик)"}]'

    local rc=0
    check_adr_number_collision 4052 4051 "" || rc=$?

    assert_eq "1" "$rc" "guard still returns 1 (collision persists)"
    local n_comments n_label
    n_comments="$(grep -c 'gh issue comment 4051' "$GH_JOURNAL" || true)"
    n_label="$(grep -c 'gh issue edit 4051 --add-label agent-flow:adr-collision' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no duplicate comment (24h dedup)"
    assert_eq "1" "$n_label" "block label still added (idempotent)"
}

# ---------------------------------------------------------------------------
# G. Fail-open: PR_FILES_JSON пустой (gh flake) → return 0.
# ---------------------------------------------------------------------------
test_G_files_empty_fails_open() {
    install_mocks_for_test
    set_state DEV_ADR_FILES "0027-baz.md"
    set_state PR_4062_FILES_JSON ""
    set_issue_state 4061 "" "[]"

    local rc=0
    check_adr_number_collision 4062 4061 "" || rc=$?

    assert_eq "0" "$rc" "guard fails open (empty files → 0)"
    local n_comments
    n_comments="$(grep -c 'gh issue comment 4061' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no comment on fail-open"
}

# ---------------------------------------------------------------------------
# H. Fail-open: DEV_ADR_FILES пустой → return 0.
# ---------------------------------------------------------------------------
test_H_develop_empty_fails_open() {
    install_mocks_for_test
    set_pr_files 4072 '["docs/adr/0027-bar.md"]'
    set_issue_state 4071 "" "[]"

    local rc=0
    check_adr_number_collision 4072 4071 "" || rc=$?

    assert_eq "0" "$rc" "guard fails open (develop tree empty → 0)"
}

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
run_test "A_new_adr_free_number"            test_A_new_adr_free_number
run_test "B_collision_blocks_pr"            test_B_collision_blocks_pr
run_test "C_rename_collision_self_passes"   test_C_rename_collision_self_passes
run_test "D_only_text_edit_passes"          test_D_only_text_edit_passes
run_test "E_override_label_passes"          test_E_override_label_passes
run_test "F_comment_dedup_24h"              test_F_comment_dedup_24h
run_test "G_files_empty_fails_open"         test_G_files_empty_fails_open
run_test "H_develop_empty_fails_open"       test_H_develop_empty_fails_open
run_test "I_edit_while_another_exists_blocks" test_I_edit_while_another_exists_blocks

printf '\n%s==== Summary ====%s\n' "$YEL" "$END"
printf 'total:  %d\n' "$TESTS_TOTAL"
printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf '%sfailed: %d%s\n' "$RED" "$TESTS_FAILED" "$END"
    printf 'failures:\n'
    for n in "${FAILED_NAMES[@]}"; do printf '  - %s\n' "$n"; done
    exit 1
fi
exit 0
