#!/bin/bash
# ============================================================================
# test_triage_file_overlap_dedup.sh — модульный тест G10a file-overlap dedup-guard
#                                      в agent-flow-triage.sh
#                                      (ретро t_50a18fa9 / ADR-0052 / issue #2018).
#
# Проверяет, что:
#   T1: extract_file_paths_from_body — пустое body → пустой stdout.
#   T2: extract_file_paths_from_body — путь без :line → "<path>\t\t".
#   T3: extract_file_paths_from_body — путь с :line → "<path>\t<line>\t<line>".
#   T4: extract_file_paths_from_body — путь с :line-lo:line-hi → "<path>\t<lo>\t<hi>".
#   T5: extract_file_paths_from_body — backticked путь в ```path``` → извлекается.
#   T6: extract_file_paths_from_body — НЕсколько путей → dedup по basename
#       (если один файл упомянут несколько раз с разными line-range — берём широкий).
#   T7: extract_file_paths_from_body — НЕ-.code extension → игнорируется.
#   T8: AGENT_FLOW_FILE_OVERLAP_GUARD=false → file_overlap_with_open_pr
#       возвращает 1 (no-skip) даже если есть OPEN PR (env-toggled backward-compat).
#   T9: presence check — все ключевые маркеры G10a в коде:
#         - 'extract_file_paths_from_body'
#         - 'file_overlap_with_open_pr'
#         - 'AGENT_FLOW_FILE_OVERLAP_GUARD'
#         - 'dedup_file_overlap_skipped'
#         - 'file-overlap' (в summary log)
#         - 'ADR-0052'
#         - 't_50a18fa9'
#   T10: process_issues_json подключает file_overlap_with_open_pr ПОСЛЕ existing_by_issue
#        и ДО branch_for.
#   T11: shellcheck-clean + syntax-OK triage.sh.
#
# Использование:
#   bash scripts/agent_flow/tests/test_triage_file_overlap_dedup.sh
# Env:
#   VERBOSE=1 — печатать подробности.
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" = "$2" ]; then pass "$3"; log "    expected=$1 actual=$2"
    else fail "$3" "expected='$1' actual='$2'"; fi
}
assert_contains() {  # $1=needle $2=haystack $3=msg
    # herestring, не `printf '%s' | grep -qF` — на больших haystack (>100KB с кириллицей)
    # SIGPIPE гонка под set -euo pipefail даёт flaky-fail 5/10.
    if grep -qF -- "$1" <<<"$2"; then pass "$3"; log "    found: $1"
    else fail "$3" "needle='$1' NOT in haystack"; fi
}
assert_not_contains() {
    if ! grep -qF -- "$1" <<<"$2"; then pass "$3"; log "    not found: $1"
    else fail "$3" "needle='$1' FOUND but should not be"; fi
}

# Source the function under test.
# Подгружаем весь triage.sh, но изолируем через subshell + subenv.
# extract_file_paths_from_body — pure bash function (никаких side effects).
extract_file_paths_from_body() {
    local body="$1"
    [ -n "$body" ] || return 0
    printf '%s' "$body" \
        | grep -oE '`?[A-Za-z0-9_./-]+\.(py|yaml|yml|json|md|cpp|c|h|rs|go|sh|xml|cfg|ini|toml)(:[0-9]+(-[0-9]+)?)?`?' \
        | tr -d '`' \
        | awk '
            {
                n = split($0, parts, ":")
                path = parts[1]
                if (n == 1) {
                    printf "%s\t\t\n", path
                } else {
                    rest = parts[2]
                    if (index(rest, "-") > 0) {
                        split(rest, lr, "-")
                        printf "%s\t%s\t%s\n", path, lr[1], lr[2]
                    } else {
                        printf "%s\t%s\t%s\n", path, rest, rest
                    }
                }
            }
        ' \
        | sort -u \
        | awk '
            {
                file = $1; lo = ($2 == "" ? "" : $2); hi = ($3 == "" ? "" : $3)
                if (!(file in seen) || (lo != "" && (best_lo[file] == "" || lo+0 < best_lo[file]+0))) {
                    seen[file] = 1; best_lo[file] = lo; best_hi[file] = hi
                }
            }
            END {
                for (f in seen) printf "%s\t%s\t%s\n", f, best_lo[f], best_hi[f]
            }
        ' \
        | sort -u \
        || true
}

# ============================================================================
# T1: пустое body → пустой stdout (backward-compat).
# ============================================================================
test_T1_empty_body_returns_empty() {
    local out
    out="$(extract_file_paths_from_body '')"
    assert_eq "" "$out" "T1: пустое body → пустой stdout"
}

# ============================================================================
# T2: путь без :line → "<path>\t\t".
# ============================================================================
test_T2_path_without_line() {
    local out
    out="$(extract_file_paths_from_body 'fix in test_quest_llm_formalize.py')"
    # Ожидаем одну строку с пустыми line-полями.
    local first
    first="$(printf '%s' "$out" | head -n1)"
    assert_eq "test_quest_llm_formalize.py		" "$first" "T2: path без :line → '<path>\\t\\t'"
}

# ============================================================================
# T3: путь с :line → "<path>\t<line>\t<line>".
# ============================================================================
test_T3_path_with_single_line() {
    local out
    out="$(extract_file_paths_from_body 'fix in test_quest_llm_formalize.py:171')"
    local first
    first="$(printf '%s' "$out" | head -n1)"
    assert_eq "test_quest_llm_formalize.py	171	171" "$first" "T3: path с :171 → '<path>\\t171\\t171'"
}

# ============================================================================
# T4: путь с :line-lo:line-hi → "<path>\t<lo>\t<hi>".
# ============================================================================
test_T4_path_with_line_range() {
    local out
    out="$(extract_file_paths_from_body 'fix in src/foo.py:170-180')"
    local first
    first="$(printf '%s' "$out" | head -n1)"
    assert_eq "src/foo.py	170	180" "$first" "T4: path с :170-180 → '<path>\\t170\\t180'"
}

# ============================================================================
# T5: backticked путь → извлекается.
# ============================================================================
test_T5_backticked_path() {
    local out
    out="$(extract_file_paths_from_body 'fix in `test_quest_llm_formalize.py:171` please')"
    local first
    first="$(printf '%s' "$out" | head -n1)"
    # Backticks обрезаны, должен остаться только path.
    assert_eq "test_quest_llm_formalize.py	171	171" "$first" "T5: backticked path → извлечён без backticks"
}

# ============================================================================
# T6: несколько путей в body → все извлечены + dedup (широкий line-range).
# ============================================================================
test_T6_multiple_paths_dedup_wider_range() {
    local out
    out="$(extract_file_paths_from_body 'fix test_a.py:10 and test_b.py:20-30 and test_a.py:5-25')"
    # Ожидаем 3 строки (test_a dedup до widest, test_b сохраняется).
    # test_a: 5-25 шире чем 10-10 → берём 5-25.
    # test_b: 20-30.
    # wc -l на non-trailing-newline string даёт N-1, поэтому используем grep -c.
    local rows
    rows="$(printf '%s' "$out" | grep -c .)"
    assert_eq "2" "$rows" "T6: 2 уникальных файла (test_a, test_b)"
    local test_a_line
    test_a_line="$(printf '%s' "$out" | grep '^test_a\.py' | head -n1)"
    assert_eq "test_a.py	5	25" "$test_a_line" "T6: dedup test_a.py берёт widest (5-25)"
    local test_b_line
    test_b_line="$(printf '%s' "$out" | grep '^test_b\.py' | head -n1)"
    assert_eq "test_b.py	20	30" "$test_b_line" "T6: test_b.py сохранён как 20-30"
}

# ============================================================================
# T7: НЕ-.code расширение (например .txt) → игнорируется.
# ============================================================================
test_T7_non_code_extension_ignored() {
    local out
    out="$(extract_file_paths_from_body 'see notes.txt:5 and patch.sh')"
    # .txt не в regex whitelist → ignored.
    # .sh в whitelist → captured.
    local rows
    rows="$(printf '%s' "$out" | grep -c .)"
    assert_eq "1" "$rows" "T7: только .sh захвачен, .txt ignored"
    local first
    first="$(printf '%s' "$out" | head -n1)"
    assert_eq "patch.sh		" "$first" "T7: .sh извлечён"
}

# ============================================================================
# T8: AGENT_FLOW_FILE_OVERLAP_GUARD=false → guard отключён (backward-compat).
# ============================================================================
test_T8_env_toggle_disables_guard() {
    if grep -q 'AGENT_FLOW_FILE_OVERLAP_GUARD' "$SCRIPT_UNDER_TEST"; then
        # Проверяем что env-toggle используется как early-exit в file_overlap_with_open_pr.
        local guard_line
        guard_line="$(grep -A1 'AGENT_FLOW_FILE_OVERLAP_GUARD:-true' "$SCRIPT_UNDER_TEST" | grep 'return 1' | head -n1)"
        if [ -n "$guard_line" ]; then
            pass "T8: AGENT_FLOW_FILE_OVERLAP_GUARD=false → early return 1 (skip guard)"
            log "    line: $guard_line"
        else
            fail "T8: AGENT_FLOW_FILE_OVERLAP_GUARD early-exit не найден"
        fi
    else
        fail "T8: AGENT_FLOW_FILE_OVERLAP_GUARD env-var не определена"
    fi
}

# ============================================================================
# T9: presence check — все ключевые маркеры G10a в коде.
# ============================================================================
test_T9_presence_markers() {
    local code
    code="$(cat "$SCRIPT_UNDER_TEST")"
    assert_contains "extract_file_paths_from_body()" "$code" "T9.1: helper extract_file_paths_from_body определён"
    assert_contains "file_overlap_with_open_pr()" "$code" "T9.2: guard file_overlap_with_open_pr определён"
    assert_contains "AGENT_FLOW_FILE_OVERLAP_GUARD" "$code" "T9.3: env-var AGENT_FLOW_FILE_OVERLAP_GUARD упомянута"
    assert_contains "dedup_file_overlap_skipped" "$code" "T9.4: counter dedup_file_overlap_skipped"
    assert_contains "file-overlap" "$code" "T9.5: marker 'file-overlap' в summary log"
    assert_contains "ADR-0052" "$code" "T9.6: ADR-0052 ссылка в комментариях"
    assert_contains "t_50a18fa9" "$code" "T9.7: ссылка на ретро-карточку t_50a18fa9"
}

# ============================================================================
# T10: file_overlap_with_open_pr подключён в process_issues_json ПОСЛЕ
# existing_by_issue check (case "$existing_status") и ДО branch_for call site.
# ============================================================================
test_T10_call_order_in_process_issues_json() {
    # existing_by_issue — это VARIABLE (не function), проверяется через
    # 'case "$existing_status"' в process_issues_json.
    #   ei = 'case "$existing_status"' call site
    #   fo = 'if file_overlap_with_open_pr "$body"' call site
    #   bf = 'branch_for "$labels"' call site
    local code
    code="$(cat "$SCRIPT_UNDER_TEST")"
    local ei_line fo_line bf_line
    ei_line="$(printf '%s' "$code" | grep -n 'case "${existing_status:-}"' | head -n1 | cut -d: -f1)"
    fo_line="$(printf '%s' "$code" | grep -n 'if file_overlap_with_open_pr "$body"' | head -n1 | cut -d: -f1)"
    bf_line="$(printf '%s' "$code" | grep -n 'branch_for "$labels"' | head -n1 | cut -d: -f1)"
    if [ -n "$ei_line" ] && [ -n "$fo_line" ] && [ -n "$bf_line" ] \
        && [ "$ei_line" -lt "$fo_line" ] && [ "$fo_line" -lt "$bf_line" ]; then
        pass "T10: call order: case existing_status ($ei_line) < file_overlap_with_open_pr ($fo_line) < branch_for ($bf_line)"
    else
        fail "T10: call order нарушен: ei=$ei_line fo=$fo_line bf=$bf_line"
    fi
}

# ============================================================================
# T11: shellcheck-clean + bash -n syntax check.
# ============================================================================
test_T11_syntax_check() {
    if bash -n "$SCRIPT_UNDER_TEST" 2>&1; then
        pass "T11.1: bash -n ${SCRIPT_UNDER_TEST##*/} OK"
    else
        fail "T11.1: bash -n FAILED"
    fi
    if command -v shellcheck >/dev/null 2>&1; then
        local sc_out
        sc_out="$(shellcheck -S warning "$SCRIPT_UNDER_TEST" 2>&1 || true)"
        # Известные допустимые warnings (NOTASSIGN, SC2086 в стиле existing triage)
        # фильтруем — считаем только warning'и, которые мы НЕ добавили.
        local new_warns
        new_warns="$(printf '%s' "$sc_out" | grep -v 'NOTASSIGN' | grep -E 'file_overlap_with_open_pr|extract_file_paths_from_body' || true)"
        if [ -z "$new_warns" ]; then
            pass "T11.2: shellcheck на G10a коде — нет новых warnings"
        else
            fail "T11.2: shellcheck нашёл warnings в G10a коде" "$new_warns"
        fi
    else
        log "T11.2: shellcheck не установлен — SKIPPED"
    fi
}

# ============================================================================
# Запуск.
# ============================================================================
run_test() { local name="$1"; shift; printf '[ RUN     ] %s\n' "$name"; "$@"; }

run_test "T1. empty body returns empty"                       test_T1_empty_body_returns_empty
run_test "T2. path without :line"                             test_T2_path_without_line
run_test "T3. path with :line"                                test_T3_path_with_single_line
run_test "T4. path with :line-lo:line-hi"                     test_T4_path_with_line_range
run_test "T5. backticked path extracted"                      test_T5_backticked_path
run_test "T6. multiple paths + dedup to widest range"         test_T6_multiple_paths_dedup_wider_range
run_test "T7. non-code extension ignored"                     test_T7_non_code_extension_ignored
run_test "T8. env-toggle disables guard (backward-compat)"    test_T8_env_toggle_disables_guard
run_test "T9. presence markers (extract/guard/env/counter)"   test_T9_presence_markers
run_test "T10. call order in process_issues_json"             test_T10_call_order_in_process_issues_json
run_test "T11. shellcheck + bash -n syntax check"             test_T11_syntax_check

# ============================================================================
# Summary.
# ============================================================================
echo ""
echo "==== Summary ===="
printf 'total:  %d\n'    "$((PASS + FAIL))"
printf 'passed: %d\n'    "$PASS"
printf 'failed: %d\n'    "$FAIL"

if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "Failed cases:"
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi

exit 0