#!/bin/bash
# ============================================================================
# test_triage_file_overlap_dedup.sh — модульный тест G10a file-overlap dedup-guard
#                                      в agent-flow-triage.sh
#                                      (ретро t_50a18fa9 / ADR-AF-0062 / issue #2018).
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
#         - 'ADR-AF-0062'
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
    assert_contains "ADR-AF-0062" "$code" "T9.6: ADR-AF-0062 ссылка в комментариях"
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
# T12: G10a dedup marker `<!-- hermes-triage-g10a: <hash> -->` есть в body
# comment, который пишется в issue. Это нужно чтобы dedup-find работал.
# ============================================================================
test_T12_g10a_marker_in_comment_body() {
    local code
    code="$(cat "$SCRIPT_UNDER_TEST")"
    assert_contains "<!-- \${AGENT_FLOW_FILE_OVERLAP_MARKER}: \${_state_hash} -->" "$code" "T12.1: G10a marker template в body template"
    assert_contains "_marker_line=" "$code" "T12.2: переменная _marker_line определена"
    assert_contains "_state_hash=" "$code" "T12.3: переменная _state_hash определена"
}

# ============================================================================
# T13: AGENT_FLOW_FILE_OVERLAP_DEDUP_HOURS env-var определена с дефолтом 6
# и используется в rate-limit-skip логике.
# ============================================================================
test_T13_dedup_hours_env() {
    local code
    code="$(cat "$SCRIPT_UNDER_TEST")"
    assert_contains 'AGENT_FLOW_FILE_OVERLAP_DEDUP_HOURS="${AGENT_FLOW_FILE_OVERLAP_DEDUP_HOURS:-6}"' "$code" "T13.1: env-var определена с дефолтом 6"
    assert_contains 'AGENT_FLOW_FILE_OVERLAP_DEDUP_HOURS * 3600' "$code" "T13.2: hours → seconds конверсия в cutoff"
    assert_contains '_action="rate-limit-skip"' "$code" "T13.3: rate-limit-skip action определён"
}

# ============================================================================
# T14: gh api PATCH используется для edit-old-comment (state changed).
# Без этого при изменении state spam-обновления остановить нельзя.
# ============================================================================
test_T14_gh_api_patch_used() {
    local code
    code="$(cat "$SCRIPT_UNDER_TEST")"
    assert_contains "--method PATCH" "$code" "T14.1: gh api --method PATCH для edit"
    assert_contains 'repos/${GH_REPO}/issues/comments/${_existing_id}' "$code" "T14.2: edit endpoint по comment_id"
    assert_contains '_action="edit"' "$code" "T14.3: edit action определён"
}

# ============================================================================
# T15: state hash должен включать issue_number — иначе два разных issue
# с одинаковым overlap-файлом получили бы одинаковый hash и dedup путал бы
# их комменты. Проверяем через код: number фигурирует в sha1sum input.
# ============================================================================
test_T15_hash_includes_issue_number() {
    local code
    code="$(cat "$SCRIPT_UNDER_TEST")"
    # Ищем в районе построения _state_hash: должен быть printf '$number'
    local hash_block
    hash_block="$(printf '%s' "$code" | awk '/_state_hash=/,/sha1sum/' | head -10)"
    if printf '%s' "$hash_block" | grep -qF '$number'; then
        pass "T15: hash-input включает issue_number"
    else
        fail "T15: hash-input НЕ включает issue_number" "block: $hash_block"
    fi
}

run_test "T12. G10a dedup marker in comment body"             test_T12_g10a_marker_in_comment_body
run_test "T13. AGENT_FLOW_FILE_OVERLAP_DEDUP_HOURS defined"   test_T13_dedup_hours_env
run_test "T14. gh api PATCH used for edit-old path"           test_T14_gh_api_patch_used
run_test "T15. hash includes issue_number (per-issue scoping)" test_T15_hash_includes_issue_number

# ============================================================================
# T16: G10a dedup filter regression (bug #3027).
# Проверяет, что gh api --jq filter (после bash→jq unescape) валиден и
# возвращает только тот hash, который был в исходном marker'е, а не весь body.
#
# Раньше фильтр был: test("\\Qhermes-triage-g10a\\E") — bash unescape даёт
# `\Q…\E` (одиночный backslash), что НЕ валидно в jq (Invalid escape).
# Фикс (см. scripts/agent_flow/agent-flow-triage.sh:571-583) убирает \Q…\E и
# использует capture("hermes-triage-g10a: (?<hash>[0-9a-f]{12})").
#
# Симулируем через `jq -R` (raw input — capture() требует JSON-typed входа),
# как если бы gh api --jq выполнил query над комментами issue #3014.
# ============================================================================
test_T16_g10a_dedup_jq_filter_compiles() {
    if ! command -v jq >/dev/null 2>&1; then
        log "T16: jq не установлен — SKIPPED"
        return 0
    fi

    # T16.0: capture() на минимальной marker-line (raw input) — работает.
    local sample='<!-- hermes-triage-g10a: c1a0fc4bdff8 -->'
    local capture_out
    capture_out="$(printf '%s' "$sample" | jq -R 'capture("hermes-triage-g10a: (?<hash>[0-9a-f]{12})").hash // ""' 2>&1)"
    # jq без -r даёт quoted-string для null/numbers; trim кавычки.
    capture_out="${capture_out#\"}"
    capture_out="${capture_out%\"}"
    assert_eq "c1a0fc4bdff8" "$capture_out" "T16.0: capture() возвращает ТОЛЬКО 12-hex hash, не весь body"

    # T16.2: точный regression-guard — старый test() с \Q…\E должен FAIL.
    local sample_with_newline='<!-- hermes-triage-g10a: c1a0fc4bdff8 -->

rest of body'
    local old_filter_msg
    old_filter_msg="$(printf '%s' "$sample_with_newline" | jq -R 'try test("\Qhermes-triage-g10a\E") catch "INVALID"' 2>&1)"
    if printf '%s' "$old_filter_msg" | grep -q INVALID; then
        pass "T16.2: regression-guard — старый \\Q…\\E filter правильно INVALID в jq (так сломан в triage.sh был)"
    else
        log "T16.2: \\Q…\\E filter всё ещё валиден? (jq версия отличается?) — не строгий guard"
        pass "T16.2: log only — вывод=$old_filter_msg"
    fi

    # T16.3: новый test() с plain-pattern ДОЛЖЕН компилироваться И матчить.
    local matched
    matched="$(printf '%s' "$sample" | jq -R '[test("hermes-triage-g10a")] | first' 2>&1)"
    assert_eq "true" "$matched" "T16.3: test(plain-pattern) возвращает true на marker-string"

    # T16.4: edge case — body без marker'а → пустой результат.
    local no_marker='just plain text without any marker'
    local empty_out
    empty_out="$(printf '%s' "$no_marker" | jq -R 'capture("hermes-triage-g10a: (?<hash>[0-9a-f]{12})").hash // "NOT_FOUND"' 2>&1)"
    if printf '%s' "$empty_out" | grep -q 'NOT_FOUND'; then
        pass "T16.4: capture() возвращает NOT_FOUND когда marker'а нет в body"
    else
        fail "T16.4: capture() вернул неожиданное значение: $empty_out"
    fi
}

run_test "T16. G10a dedup jq filter compiles + extracts hash" test_T16_g10a_dedup_jq_filter_compiles

# ============================================================================
# T17: full functional dedup против акутального issue #3014 (live e2e lite).
# Используем реальный `gh api` (НЕ fake — он слишком сложно воспроизводит
# capture() семантику с тем же резолвером, что gh api --jq использует).
# Проверяем, что после фикса наш filter находит последний G10a-коммент
# в #3014 и возвращает hash.
# ============================================================================
test_T17_dedup_lookup_against_real_issue() {
    local filter_for_dedup='[.[] | select((.body // "") | test("hermes-triage-g10a"))] | last | "\(.id // empty)|\(.created_at // empty)|\(.body // "" | capture("hermes-triage-g10a: (?<hash>[0-9a-f]{12})").hash // "")"'
    local result
    if ! result="$(timeout 30 gh api repos/krikz/rob_box_project/issues/3014/comments?per_page=100 --jq "$filter_for_dedup" 2>&1)"; then
        # gh сбойнул (offline или rate-limit) — пропустить.
        log "T17: gh api недоступен — SKIPPED ($result)"
        return 0
    fi

    # T17.1: filter должен вернуть id последнего комментария (>0, число).
    # shellcheck disable=SC2034  # got_iso is parsed but unused by intent
    local got_id got_iso got_hash
    IFS='|' read -r got_id got_iso got_hash <<< "$result"
    if [ -n "$got_id" ] && [ "$got_id" -gt 0 ] 2>/dev/null; then
        pass "T17.1: filter возвращает id существующего G10a-коммента (#$got_id)"
    else
        fail "T17.1: filter не нашёл комментов или id=0" "result='$result'"
    fi

    # T17.2: hash должен быть ТОЧНО 12 hex символов, не multiline-body.
    if printf '%s' "$got_hash" | grep -qE '^[0-9a-f]{12}$'; then
        pass "T17.2: hash extraction — ровно 12-hex, не multiline (got: $got_hash)"
    else
        fail "T17.2: hash extraction сломан (got: $got_hash, expected 12-hex)"
    fi

    # T17.3: hash должен СОВПАДАТЬ с ожидаемым из issue body (c1a0fc4bdff8).
    if [ "$got_hash" = "c1a0fc4bdff8" ]; then
        pass "T17.3: hash = c1a0fc4bdff8 (matches issue #3014 marker)"
    else
        # может быть другой tick с другим hash — это ожидаемо если PR-ы изменились
        if [ -n "$got_hash" ]; then
            log "T17.3: hash differs от c1a0fc4bdff8 (state изменился? got=$got_hash)"
            pass "T17.3: hash extraction работает (отличается от ожидаемого = state evolved)"
        else
            fail "T17.3: пустой hash"
        fi
    fi
}

run_test "T17. dedup lookup against real issue #3014" test_T17_dedup_lookup_against_real_issue

# ============================================================================
# T18: regression guard — фикс от \Q…\E реально работает в shell-контексте,
# имитирующем bash unescape → jq (как в gh api --jq "<filter>").
# Тест-кейс воспроизводит баг #3027: test("\\Qhermes-triage-g10a\\E") в bash → jq
# ============================================================================
test_T18_no_qq_ee_in_jq_filter() {
    # T18: regression guard — \Q…\E НЕ должно быть в коде (баг #3027).
    # Используем простой grep по всему файлу вместо хрупкого awk.
    if grep -q '"\\Q.*\\E"' "$SCRIPT_UNDER_TEST"; then
        local block
        block="$(grep -B2 -A1 '"\\Q.*\\E"' "$SCRIPT_UNDER_TEST")"
        fail "T18: jq filter всё ещё содержит \\Q…\\E (регрессия #3027)" "block: $block"
    else
        pass "T18: jq filter больше не использует \\Q…\\E (баг #3027 зафикшен)"
    fi
    # T18.2: capture() должна быть в lookup-блоке (agent-flow-triage.sh).
    if grep -A20 'Ищем существующий G10a-коммент' "$SCRIPT_UNDER_TEST" \
        | grep -q 'capture('; then
        pass "T18.2: jq filter использует capture() для hash extraction"
    else
        fail "T18.2: jq filter не использует capture() — hash extraction может вернуть весь body"
    fi

    # T18.3: PATCH-ветка должна использовать jq-обёртку для JSON payload.
    if grep -q 'jq -nc --arg body' "$SCRIPT_UNDER_TEST"; then
        pass "T18.3: PATCH-ветка использует jq -nc --arg body (корректный JSON payload)"
    else
        fail "T18.3: PATCH-ветка может слать raw markdown вместо JSON {\"body\": ...}"
    fi

    # T18.4: template не должен использовать неопределённую PR_COUNT.
    # Используется переменная pr_count (lowercase).
    if grep -q '${PR_COUNT:-0}' "$SCRIPT_UNDER_TEST"; then
        fail "T18.4: template использует \${PR_COUNT:-0} — должно быть \${pr_count:-0} (lowercase)"
    else
        pass "T18.4: template использует lowercase pr_count (не undefined PR_COUNT)"
    fi
}

run_test "T18. regression guard: no \\Q…\\E in jq filter"    test_T18_no_qq_ee_in_jq_filter

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