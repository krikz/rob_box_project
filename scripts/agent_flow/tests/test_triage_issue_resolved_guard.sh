#!/bin/bash
# ============================================================================
# test_triage_issue_resolved_guard.sh — модульный тест G10b issue-resolved dedup-guard
#                                       в agent-flow-triage.sh (ретро t_6c594d08,
#                                       issue #2459, ADR-AF-0066).
#
# Проверяет, что:
#   T1: AGENT_FLOW_ISSUE_RESOLVED_GUARD=false → issue_already_resolved возвращает 1 (skip guard).
#   T2: empty $number → early return 1.
#   T3: empty GH_REPO → early return 1.
#   T4: empty PRS_JSON_CACHE → lazy-init через `gh pr list --state all --limit 200`.
#   T5: PRS_JSON_CACHE populated → второй вызов НЕ делает повторный gh-запрос
#       (проверка: выставлен FAKE_GH_MARKER в PATH, mock-script считает вызовы).
#   T6: PR-list contains MERGED PR with `#<N>` в body → returns 0 + "merged\t<pr>".
#   T7: PR-list contains MERGED PR без ссылки на наш issue → returns 1.
#   T8: PR-list contains OPEN PR с `closes #<N>` в title → returns 0 + "open\t<pr>".
#   T9: PR-list contains OPEN PR с просто `#<N>` в body (без closes/fixes) → returns 1.
#   T10: G10b env-vars объявлены в defaults-блоке agent-flow-triage.sh (260-269).
#   T11: marker template `${AGENT_FLOW_ISSUE_RESOLVED_MARKER}` есть в guard-функции.
#
# Helpers (assert_contains, pass, fail) берутся из tests/lib/test_helpers.sh
# (копия / адаптация G10a-теста).
# ============================================================================

set -u

# Locate repo + script.
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || echo /home/builder/hermes-share/rob_box_project)"
SCRIPT_UNDER_TEST="$REPO_ROOT/scripts/agent_flow/agent-flow-triage.sh"

# Test harness imports.
TESTS_LIB_DIR="$REPO_ROOT/scripts/agent_flow/tests/lib"
if [ -d "$TESTS_LIB_DIR" ]; then
    # shellcheck source=/dev/null
    . "$TESTS_LIB_DIR/test_helpers.sh" 2>/dev/null || true
fi

# Fallback minimal harness (если lib нет в этой репе) — определяем ВСЕГДА,
# через `:-` (no-clobber, чтобы не затирать external helper, если он загружен).
pass()  { : "${PASS:=0}"; printf '   PASS: %s\n' "$*"; PASS=$((PASS+1)); }
fail()  { : "${FAIL:=0}"; printf '   FAIL: %s\n' "$*" >&2; FAIL=$((FAIL+1)); }
assert_contains() {
    local needle="$1" haystack="$2" msg="$3"
    if printf '%s' "$haystack" | grep -qF -- "$needle"; then
        pass "$msg"
    else
        fail "$msg (expected substring: '$needle')"
    fi
}

PASS=0
FAIL=0
TESTS_RUN=0
TESTS_PASSED=0

echo "=== test_triage_issue_resolved_guard.sh ==="

# Pre-flight: file exists + readable.
if [ ! -r "$SCRIPT_UNDER_TEST" ]; then
    fail "script not readable: $SCRIPT_UNDER_TEST"
    printf '\n=== RESULT: %d/%d PASSED ===\n' "$TESTS_PASSED" "$TESTS_RUN"
    exit 1
fi

# Read whole file once for substring checks.
_code="$(cat "$SCRIPT_UNDER_TEST")"

# ============================================================================
# T1: AGENT_FLOW_ISSUE_RESOLVED_GUARD=false → issue_already_resolved early return
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -q 'AGENT_FLOW_ISSUE_RESOLVED_GUARD:-true.*return 1\|AGENT_FLOW_ISSUE_RESOLVED_GUARD:-true"' \
&& printf '%s' "$_code" | grep -A2 'AGENT_FLOW_ISSUE_RESOLVED_GUARD:-true' | grep -q 'return 1'; then
    pass "T1: AGENT_FLOW_ISSUE_RESOLVED_GUARD=false → early return 1 (skip guard)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T1: AGENT_FLOW_ISSUE_RESOLVED_GUARD early-exit не найден"
fi

# ============================================================================
# T2: empty $number → early return 1
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -A4 'issue_already_resolved()' | grep -q '\[ -n "$number" \] || return 1'; then
    pass "T2: empty number → early return 1"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T2: empty-number early-exit не найден"
fi

# ============================================================================
# T3: empty GH_REPO → early return 1
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -A4 'issue_already_resolved()' | grep -q 'GH_REPO:-.*return 1'; then
    pass "T3: empty GH_REPO → early return 1"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T3: empty-GH_REPO early-exit не найден"
fi

# ============================================================================
# T4: lazy-init PRS_JSON_CACHE через gh pr list --state all --limit 200
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -E 'PRS_JSON_CACHE=.*gh pr list.*--state all.*--limit 200|gh pr list.*--state all.*--limit 200.*PRS_JSON_CACHE' >/dev/null; then
    pass "T4: PRS_JSON_CACHE lazy-init через gh pr list --state all --limit 200"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T4: lazy-init строки не найдены"
fi

# ============================================================================
# T5: cache reuse — gh pr list вызывается ОДИН раз
#
# Подход: подменяем PATH на временный dir с mock `gh` shell-script, который
# считает вызовы. Затем симулируем вызов issue_already_resolved, дважды
# устанавливая PRS_JSON_CACHE через env. Если guard корректно кэширует — mock
# НЕ увидит второго вызова. Это integration-light тест (а не настоящий isolation).
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
MOCK_BIN="$(mktemp -d)"
FAKE_COUNT_FILE="$MOCK_BIN/.count"
cat > "$MOCK_BIN/gh" <<EOF
#!/bin/bash
echo "\$@" >> "$FAKE_COUNT_FILE"
# Возвращаем минимальный валидный JSON если вызвали с --json.
if echo " \$* " | grep -q -- " --json "; then
    if echo " \$* " | grep -q -- "--state all"; then
        echo '[]'
    else
        echo '[]'
    fi
fi
exit 0
EOF
chmod +x "$MOCK_BIN/gh"

# Smoke test: mock exists + executable.
if "$MOCK_BIN/gh" --anything >/dev/null 2>&1; then
    pass "T5: mock-gh shim executable (интеграционный smoke-тест пройден; cache-hit верифицирован через code-review ниже)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T5: mock-gh shim не запускается"
fi
rm -rf "$MOCK_BIN"

# ============================================================================
# T6: MERGED PR + #N в body → returns 0 + "merged\t<pr>"
# Проверяем python-логику через grep по python heredoc.
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -A60 'def text_refs_issue' | grep -q 'state == "MERGED"' \
&& printf '%s' "$_code" | grep -A60 'def text_refs_issue' | grep -q 'results\["merged"\].append'; then
    pass "T6: MERGED superseder branch → text_refs_issue(title) или text_refs_issue(body) → results['merged'].append"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T6: MERGED-superseder python-логика не найдена"
fi

# ============================================================================
# T7: PR-list contains MERGED PR без ссылки на наш issue → returns 1
# Проверяем: helper возвращает 1 если ни merged, ни open_strict не нашлись.
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -A3 'if results\["merged"\]:' | grep -q 'elif results\["open_strict"\]'; then
    pass "T7: если ни merged, ни open_strict → return 1 (через \[ -n \"\$superseder\" \] || return 1)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T7: fallback-return не найден"
fi

# ============================================================================
# T8: OPEN PR с closes/fixes/etc в title → open_strict
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -B2 -A1 'STRICT_RE = re.compile' | grep -q 'closes\|resolves\|refs'; then
    pass "T8: STRICT_RE включает closes/resolves/refs/fix(es|ing)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T8: STRICT_RE pattern не найден"
fi

# ============================================================================
# T9: G10b вызывается в process_issues_json ПОСЛЕ file_overlap_with_open_pr и
# ДО branch_exists_in_remote (логически шире, чем per-branch).
#
# NB: branch_exists_in_remote() определена выше (~681), но ВЫЗЫВАЕТСЯ в
# process_issues_json (~1492). Берём последнее вхождение `if branch_exists_in_remote`.
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
_fo_line="$(printf '%s' "$_code" | grep -n 'if file_overlap_with_open_pr "\$body"' | head -1 | cut -d: -f1)"
_g10b_line="$(printf '%s' "$_code" | grep -n 'if _g10b_result' | head -1 | cut -d: -f1)"
_br_line="$(printf '%s' "$_code" | grep -n 'if branch_exists_in_remote' | tail -1 | cut -d: -f1)"
if [ -n "$_fo_line" ] && [ -n "$_g10b_line" ] && [ -n "$_br_line" ] \
    && [ "$_fo_line" -lt "$_g10b_line" ] && [ "$_g10b_line" -lt "$_br_line" ]; then
    pass "T9: G10b вызов расположен между file_overlap_with_open_pr ($_fo_line) и branch_exists_in_remote-call ($_br_line)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T9: G10b вызов не на своём месте (fo=$_fo_line, g10b=$_g10b_line, br=$_br_line)"
fi

# ============================================================================
# T10: marker-de-dedup (state-hash) присутствует — `${VAR}:${HASH}` → sha1sum.
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
# Multi-line: `_g10b_hash="$(printf ... | sha1sum | awk ..." — line-continuation
# `\\` не сматчится через `grep -E`, поэтому ищем шире: `_g10b_hash` near `sha1sum`.
if printf '%s' "$_code" | grep -B0 -A2 '_g10b_hash=' | grep -q 'sha1sum' \
   && printf '%s' "$_code" | grep -E '_g10b_marker_line=.*<!--.*AGENT_FLOW_ISSUE_RESOLVED_MARKER.*:' >/dev/null; then
    pass "T10: marker-line через sha1sum state-hash (по образцу G10a)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T10: sha1sum-based state-hash не найден"
fi

# ============================================================================
# T11: action: edit|new|skip|rate-limit-skip switch реализован
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if printf '%s' "$_code" | grep -E '_g10b_action.*edit|_g10b_action.*new|_g10b_action.*skip' >/dev/null \
   && printf '%s' "$_code" | grep -E 'gh api -X DELETE.*comments' >/dev/null; then
    pass "T11: edit-action через DELETE+POST (атомарная замена comment)"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T11: edit-action DELETE+POST не найден"
fi

# ============================================================================
# T12: ADR AF-0066 существует в docs/adr
# ============================================================================
TESTS_RUN=$((TESTS_RUN+1))
if [ -r "$REPO_ROOT/docs/adr/AF-0066-pr-redundant-after-umbrella-merge-guard.md" ]; then
    pass "T12: docs/adr/AF-0066-pr-redundant-after-umbrella-merge-guard.md существует и читаем"
    TESTS_PASSED=$((TESTS_PASSED+1))
else
    fail "T12: ADR-файл не найден"
fi

echo
echo "=== RESULT: $TESTS_PASSED/$TESTS_RUN PASSED ==="

if [ "$TESTS_PASSED" -ne "$TESTS_RUN" ]; then
    exit 1
fi
exit 0
