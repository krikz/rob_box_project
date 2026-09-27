#!/bin/bash
# ============================================================================
# test_detect_stale_after_upstream_fix.sh
#
# Ретро 31.08 t_9d375e3e / ADR-0035 / task t_3e106bfc:
# UNIT-тесты для pure функции detect_stale_after_upstream_fix() в
# scripts/agent_flow/agent-flow-merge-gate.sh. Спецификация (t_3e106bfc):
#   «отдельная функция detect_stale_after_upstream_fix() ... покрытая
#    входными аргументами (без побочных эффектов, чтобы быть тестируемой
#    из юнит-теста)».
#
# Этот файл НЕ зависит от полного merge-gate (не вызывает его main loop) —
# только подгружает detect_stale_after_upstream_fix() и pr_state_now() из
# merge-gate.sh, плюс мок gh/git из tests/lib/mock_env.sh. Детектор чистый:
#   * принимает (card_id, pr_num, pr_sha, pr_base, sig_csv, tests_csv,
#     created_ts, repo_dir, gh_repo) — все 9 аргументов позиционные;
#   * возвращает TSV на stdout: stale<TAB>sha<TAB>strategy<TAB>reason<TAB>evidence;
#   * НЕ вызывает `hermes kanban block`, НЕ пишет в journal, НЕ логирует;
#   * только читает (git/gh через моки) и возвращает результат.
#
# Сценарии (4 стратегии + edge cases):
#   U1. strat A: PR head SHA ancestor of origin/<base> → {stale=true, sha=PR_HEAD, strategy=A}.
#   U2. strat B-attr: upstream-коммит с -S <attr> в origin/<base> → {stale=true, sha=UPSTREAM, strategy=B}.
#   U3. strat B-tests: upstream-коммит для failing-tests файла → {stale=true, sha=UPSTREAM, strategy=B}.
#   U4. strat C: фикс в самом PR + CI SUCCESS → {stale=true, sha="", strategy=C}.
#   U5. PR CLOSED → {stale=true, sha="", strategy=closed}.
#   U6. REST fallback: PR merged (gh pr view mergedAt не null) → {stale=true, sha=PR_HEAD, strategy=rest_fallback}.
#   U7. Нечего детектить: всё пусто → {stale=false, sha="", strategy=none}.
#   U8. Empty sig/tests + PR_SHA не ancestor → falls through to REST fallback (not stale).
#   U9. Без side-effects: detect НЕ вызывает `hermes kanban block` (journal пустой).
#
# Run:
#   bash scripts/agent_flow/tests/test_detect_stale_after_upstream_fix.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

MERGE_GATE="$REPO_ROOT/agent-flow-merge-gate.sh"
if [ ! -f "$MERGE_GATE" ]; then
    echo "FAIL: cannot find merge-gate at $MERGE_GATE" >&2
    exit 2
fi

# ----------------------------------------------------------------------------
# parse_tsv_fields — split TSV "stale<TAB>sha<TAB>strategy<TAB>reason<TAB>evidence".
# NB: bash `IFS=$'\t' read -r a b c d e` НЕ различает пустые поля между
# двумя \t — bash склеивает adjacent-whitespace в single delimiter, поэтому
# "true\t\tclosed\t..." даёт всего 2 поля вместо 5. Используем python —
# единственный 100% надёжный способ парсить TSV с пустыми полями без
# зависимостей. Вывод — 5 строк (newline-separated), индекс 0..4.
parse_tsv_fields() {
    local line="$1"
    python3 -c '
import sys
parts = sys.argv[1].rstrip("\n").split("\t")
while len(parts) < 5: parts.append("")
print("\n".join(parts))
' "$line"
}

# ----------------------------------------------------------------------------
# load_detect_function — source detect_stale_after_upstream_fix() + pr_state_now()
# из merge-gate.sh в текущий shell. Это позволяет вызвать detect напрямую
# (как unit-функцию), без полного main-loop merge-gate.
# ----------------------------------------------------------------------------
load_detect_function() {
    local dg="$1"
    eval "$(awk '
        /^detect_stale_after_upstream_fix\(\) \{/ {flag=1; print; next}
        flag && /^}$/ {print; flag=0; next}
        flag {print}
        /^pr_state_now\(\) \{/ {flag2=1; print; next}
        flag2 && /^}$/ {print; flag2=0; next}
        flag2 {print}
    ' "$dg")"
    # Stubs для функций, которые detect может дёрнуть.
    log() { :; }
    GH_REPO="${GH_REPO:-krikz/test-repo}"
    export GH_REPO
}

# parse → assign to vars; pass line as $1, var names as $2..$6.
assert_tsv() {
    local line="$1" label="$2"
    local expect_stale="$3" expect_sha="$4" expect_strategy="$5" expect_reason_substr="$6"
    local fields_file
    fields_file="$(mktemp)"
    parse_tsv_fields "$line" >"$fields_file"
    local stale _sha _strategy _reason _evidence
    stale="$(sed -n '1p' "$fields_file")"
    _sha="$(sed -n '2p' "$fields_file")"
    _strategy="$(sed -n '3p' "$fields_file")"
    _reason="$(sed -n '4p' "$fields_file")"
    _evidence="$(sed -n '5p' "$fields_file")"
    rm -f "$fields_file"
    assert_eq "$expect_stale" "$stale" "$label: stale" || return 1
    assert_eq "$expect_sha" "$_sha" "$label: upstream_sha" || return 1
    assert_eq "$expect_strategy" "$_strategy" "$label: strategy" || return 1
    assert_contains "$expect_reason_substr" "$_reason" "$label: reason" || return 1
}

# ----------------------------------------------------------------------------
# U1. strat A: PR head SHA ancestor of origin/<base>.
# ----------------------------------------------------------------------------
test_U1_strategy_A_pr_sha_ancestor() {
    new_test
    load_detect_function "$MERGE_GATE"
    REPO_DIR="$TEST_TMP"
    export REPO_DIR

    local pr=9001 sha="aaaa1111aaaa1111aaaa1111aaaa1111aaaa1111"
    set_state "STALE_DIAG_ANCESTOR_${sha}" "1"

    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u1" "$pr" "$sha" "develop" \
        "_any_attr" "" "1756598400" \
        "$REPO_DIR" "krikz/test-repo")"

    assert_tsv "$line" "U1" "true" "$sha" "A" "stale-after-upstream-fix" \
        || return 1
}

# ----------------------------------------------------------------------------
# U2. strat B-attr: upstream-коммит с -S <attr> в origin/<base>.
# ----------------------------------------------------------------------------
test_U2_strategy_B_attr_hit() {
    new_test
    load_detect_function "$MERGE_GATE"
    REPO_DIR="$TEST_TMP"
    export REPO_DIR

    local pr=9002 attr="_track_mode_music_active"
    local upstream_sha="06b83b01b6a8c1de76c32bf90d809f0cfa809ffc"
    set_state "STALE_DIAG_ATTR_HIT_${attr}" "$upstream_sha"

    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u2" "$pr" "fake-pr-sha" "develop" \
        "${attr}" "" "1756598400" \
        "$REPO_DIR" "krikz/test-repo")"

    assert_tsv "$line" "U2" "true" "$upstream_sha" "B" "stale-after-upstream-fix" \
        || return 1
}

# ----------------------------------------------------------------------------
# U3. strat B-tests: upstream-коммит для failing-tests файла.
# ----------------------------------------------------------------------------
test_U3_strategy_B_tests_hit() {
    new_test
    load_detect_function "$MERGE_GATE"
    REPO_DIR="$TEST_TMP"
    export REPO_DIR

    local pr=9003
    local test_file="src/rob_box_voice/test/unit/node/test_barge_in_policy.py"
    local upstream_sha="f924ad6c47bcf7deb66d2080dcee067c66cf5792"
    set_state "STALE_DIAG_ATTR_HIT_${test_file}" "$upstream_sha"

    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u3" "$pr" "fake-pr-sha" "develop" \
        "" "${test_file}" "1756598400" \
        "$REPO_DIR" "krikz/test-repo")"

    assert_tsv "$line" "U3" "true" "$upstream_sha" "B" "stale-after-upstream-fix" \
        || return 1
}

# ----------------------------------------------------------------------------
# U4. strat C: фикс в самом PR + CI SUCCESS.
# REPO_DIR пустой — чтобы A/B не сработали. tests_csv с 1 файлом, PR_9004
# содержит этот файл + PR_9004_CHECKS_FAILING_JSON=0 (все SUCCESS).
# ----------------------------------------------------------------------------
test_U4_strategy_C_fix_in_same_pr() {
    new_test
    load_detect_function "$MERGE_GATE"
    # REPO_DIR не экспортируем → strat A/B skip → REST fallback.
    unset REPO_DIR 2>/dev/null || true

    local pr=9004
    local test_file="src/foo/test_x.py"
    set_state "PR_${pr}_FILES_JSON" "{\"files\":[{\"path\":\"${test_file}\",\"filename\":\"${test_file}\"}]}"
    # Mock `gh pr checks` смотрит на CHECKS_FAILING_JSON — явно 0 = "все SUCCESS".
    set_state "PR_${pr}_CHECKS_FAILING_JSON" "0"
    # mergedAt пусто → REST fallback не сработает.
    set_state "PR_${pr}_MERGEDAT_JSON" ""

    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u4" "$pr" "cafecafecafecafecafecafecafecafecafecafe" "develop" \
        "" "${test_file}" "1756598400" \
        "" "krikz/test-repo")"

    assert_tsv "$line" "U4" "true" "" "C" "фикс уже в самом PR" \
        || return 1
}

# ----------------------------------------------------------------------------
# U5. PR CLOSED → strategy=closed.
# ----------------------------------------------------------------------------
test_U5_pr_closed() {
    new_test
    load_detect_function "$MERGE_GATE"
    unset REPO_DIR 2>/dev/null || true
    # Override pr_state_now: PR CLOSED.
    pr_state_now() { printf '%s' "CLOSED"; }

    local pr=9005 sha="deadbeefdeadbeefdeadbeefdeadbeefdeadbeef"
    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u5" "$pr" "$sha" "develop" \
        "_attr" "" "1756598400" \
        "" "krikz/test-repo")"

    assert_tsv "$line" "U5" "true" "" "closed" "PR #${pr} CLOSED" \
        || return 1
}

# ----------------------------------------------------------------------------
# U6. REST fallback: PR merged (mergedAt != null/empty) → rest_fallback.
# ----------------------------------------------------------------------------
test_U6_rest_compare_fallback() {
    new_test
    load_detect_function "$MERGE_GATE"
    unset REPO_DIR 2>/dev/null || true

    local pr=9006 sha="beadbeadbeadbeadbeadbeadbeadbeadbeadbead"
    # Mock pr view --json mergedAt → отдаёт ISO timestamp → rest_fallback.
    set_state "PR_${pr}_MERGEDAT_JSON" "2026-08-15T12:34:56Z"

    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u6" "$pr" "$sha" "develop" \
        "" "" "0" \
        "" "krikz/test-repo")"

    assert_tsv "$line" "U6" "true" "$sha" "rest_fallback" "mergedAt=" \
        || return 1
}

# ----------------------------------------------------------------------------
# U7. Нечего детектить: всё пусто → {stale=false, ...}.
# mergedAt пустой → REST fallback не сработает. Tests empty → strat C skip.
# ----------------------------------------------------------------------------
test_U7_nothing_to_detect() {
    new_test
    load_detect_function "$MERGE_GATE"
    unset REPO_DIR 2>/dev/null || true
    # Никаких state fixtures → gh pr view --json mergedAt отдаст пусто.

    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u7" "9007" "fake-sha" "develop" \
        "" "" "0" \
        "" "krikz/test-repo")"

    assert_tsv "$line" "U7" "false" "" "none" "" \
        || return 1
}

# ----------------------------------------------------------------------------
# U9. Без side-effects: detect НЕ вызывает `hermes kanban block`.
# ----------------------------------------------------------------------------
test_U9_no_side_effects() {
    new_test
    load_detect_function "$MERGE_GATE"
    REPO_DIR="$TEST_TMP"
    export REPO_DIR

    local pr=9009 sha="c0ffeec0ffeec0ffeec0ffeec0ffeec0ffeec0ffe"
    set_state "STALE_DIAG_ANCESTOR_${sha}" "1"

    # Run detect — should return stale=true via strat A.
    local line
    line="$(detect_stale_after_upstream_fix \
        "t_detect_u9" "$pr" "$sha" "develop" \
        "" "" "1756598400" \
        "$REPO_DIR" "krikz/test-repo")"
    # Sanity: assert detection worked.
    local fields_file
    fields_file="$(mktemp)"
    parse_tsv_fields "$line" >"$fields_file"
    local stale _sha _strategy _reason _evidence
    stale="$(sed -n '1p' "$fields_file")"
    _sha="$(sed -n '2p' "$fields_file")"
    _strategy="$(sed -n '3p' "$fields_file")"
    rm -f "$fields_file"
    assert_eq "true" "$stale" "U9: detect returned stale=true (sanity)" || return 1

    # Проверяем journal: detect не должен был вызывать `hermes kanban block`.
    # NB: grep -c печатает "0" при no-match (exit 1). НЕ добавлять `|| echo 0`
    # — это даст "0\n0" (лишний echo поверх уже напечатанного "0").
    local journal_block_calls
    journal_block_calls="$(grep -cE 'hermes .* block .* t_detect_u9' "$GH_JOURNAL" 2>/dev/null; true)"
    assert_eq "0" "$journal_block_calls" "U9: NO hermes kanban block calls from detect" || return 1
}

# ============================================================================
# Run.
# ============================================================================
run_test "U1. strat A: PR SHA ancestor of origin/develop"            test_U1_strategy_A_pr_sha_ancestor
run_test "U2. strat B-attr: -S <attr> upstream hit"                 test_U2_strategy_B_attr_hit
run_test "U3. strat B-tests: failing-tests file upstream hit"       test_U3_strategy_B_tests_hit
run_test "U4. strat C: fix in same PR + CI SUCCESS"                 test_U4_strategy_C_fix_in_same_pr
run_test "U5. PR CLOSED → strategy=closed"                          test_U5_pr_closed
run_test "U6. REST mergedAt fallback (REPO_DIR empty)"              test_U6_rest_compare_fallback
run_test "U7. nothing to detect → stale=false, strategy=none"       test_U7_nothing_to_detect
run_test "U9. detect has NO side effects (no kanban block call)"    test_U9_no_side_effects

echo
echo "==== Summary ===="
echo "total:  $TESTS_TOTAL"
echo "passed: $TESTS_PASSED"
echo "failed: $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf 'failures:\n'
    for n in "${FAILED_NAMES[@]}"; do
        printf '  - %s\n' "$n"
    done
    exit 1
fi
exit 0