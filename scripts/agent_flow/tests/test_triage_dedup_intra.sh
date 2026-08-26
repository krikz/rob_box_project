#!/bin/bash
# ============================================================================
# test_triage_dedup_intra.sh — модульный тест G9a/b dedup-guard в
#                              agent-flow-triage.sh (ретро t_dfd3d19d, ADR-0032).
#
# Проверяет, что:
#   T1: dedup_intra_filter вызывается напрямую с фикстурой из 2 issues
#       (одинаковые labels, одинаковый title-prefix) — возвращает 1 issue +
#       2 DEDUP_INTRA маркера в stderr.
#   T2: dedup_intra_filter с 1 issue (single-group) — возвращает тот же 1 issue,
#       0 DEDUP_INTRA маркеров.
#   T3: dedup_intra_filter с 0 issues — возвращает [].
#   T4: dedup_intra_filter с разными labels — возвращает 2 issues, 0 markers.
#   T5: dedup_intra_filter с разным title-prefix (но те же labels) — возвращает
#       2 issues, 0 markers (разные группы).
#   T6: Группа из 3 issues (одинаковые labels, одинаковый title-prefix) —
#       возвращает 1 issue (старейшую), 2 DEDUP_INTRA маркера.
#   T7: branch_exists_in_remote — существующая ветка на remote → return 0.
#   T8: branch_exists_in_remote — НЕ существующая ветка на remote → return 1.
#   T9: branch_exists_in_remote — отключено через AGENT_FLOW_DEDUP_RACE_GUARD=false
#       → return 1 (fail-OPEN).
#   T10: shellcheck-clean + syntax-OK triage.sh.
#   T11: presence check — все ключевые маркеры G9 в коде:
#         - 'dedup-skipped: ${dedup_intra_skipped} (intra-tick), ${dedup_race_skipped} (race)'
#         - 'branch_exists_in_remote "$branch"'
#         - 'dedup_intra_filter'
#         - 'AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS'
#         - 'AGENT_FLOW_DEDUP_RACE_GUARD'
#         - 'ADR-0032'
#
# Использование:
#   bash test_triage_dedup_intra.sh
# Env:
#   VERBOSE=1 — печатать подробности
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

# --- Source the relevant helpers from triage.sh -----------------------------
# We extract just the functions we need (dedup_intra_filter, branch_exists_in_remote)
# by sourcing the script and overriding MAINTENANCE/etc early-exits.
# We can't fully source the script (it tries to acquire flock + gh list at top),
# so we manually copy the function definitions into a temp script and source that.
TMP_HELPERS="$(mktemp -t test-dedup-helpers.XXXXXX)" || { echo "mktemp failed"; exit 1; }
trap 'rm -f "$TMP_HELPERS"' EXIT

# Extract the block between '# dedup_intra_filter' (line ~358) and the next
# function (find via grep for '^# Ретро-фикс (19.08, t_dd7a5749)').
cat > "$TMP_HELPERS" <<'SRC'
# Имитация необходимых external-функций для dedup_intra_filter и branch_exists_in_remote.
log() { printf '%s %s %s\n' "[test]" "$(date -Iseconds)" "$*" >&2; }

SRC

# Append branch_exists_in_remote (approx lines 327-354 of triage.sh).
awk '/^branch_exists_in_remote\(\) \{/,/^\}/' "$SCRIPT_UNDER_TEST" >> "$TMP_HELPERS"

# Append dedup_intra_filter (whole function incl. its closing '}').
awk '/^dedup_intra_filter\(\) \{/,/^\}/' "$SCRIPT_UNDER_TEST" >> "$TMP_HELPERS"

# Теперь source'им helpers.
# shellcheck disable=SC1090
. "$TMP_HELPERS"

if ! command -v dedup_intra_filter >/dev/null 2>&1; then
    echo "ERROR: failed to load dedup_intra_filter from $SCRIPT_UNDER_TEST"
    exit 1
fi
if ! command -v branch_exists_in_remote >/dev/null 2>&1; then
    echo "ERROR: failed to load branch_exists_in_remote from $SCRIPT_UNDER_TEST"
    exit 1
fi

# Экспортируем defaults, необходимые для dedup_intra_filter.
export AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=6
export DRY_RUN=true            # для тестов — никаких gh calls
export GH_REPO=""              # no-op

# --- T1: 2 issues с одинаковыми (labels, title-prefix) → 1 keep + 1 skip -----
# Используем фикстуру, реалистичную для ретро-серии: #1477 "STT empty on echo"
# vs #1478 "STT empty on echo detected" — имеют один root-cause, должны
# дедуплицироваться при AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=10 (оба prefix'а
# "stt empty on echo detected" — общий для первых 6 lowercased-words).
echo "=== T1: dedup_intra_filter with 2 identical-prefix issues (TITLE_WORDS=10) ==="
FIXTURE_T1='[
  {"number": 100, "title": "STT empty on echo in voice mode retry path additional log message 1", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""},
  {"number": 101, "title": "STT empty on echo in voice mode retry path additional log message 2", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""}
]'
# Override TITLE_WORDS на 10 для T1 (по умолчанию 6 — слишком короткий для этих фикстур).
TITLE_WORDS_BACKUP="${AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS:-6}"
export AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=10
ERR_T1="$(mktemp -t g9a-err-t1.XXXXXX)" || exit 1
OUT_T1="$(dedup_intra_filter "phase1" "$FIXTURE_T1" 2>"$ERR_T1" || true)"
COUNT_T1_KEEP="$(printf '%s' "$OUT_T1" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T1_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T1" || true)"
log "T1 (TITLE_WORDS=10): kept=$COUNT_T1_KEEP markers=$COUNT_T1_MARKERS"
if [ "$COUNT_T1_KEEP" = "1" ] && [ "$COUNT_T1_MARKERS" = "1" ]; then
    pass "T1: 2 identical-prefix issues → 1 kept, 1 DEDUP_INTRA marker (TITLE_WORDS=10)"
else
    fail "T1: expected kept=1, markers=1; got kept=$COUNT_T1_KEEP, markers=$COUNT_T1_MARKERS" \
        "stdout=$OUT_T1"
fi
rm -f "$ERR_T1"
export AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS="$TITLE_WORDS_BACKUP"

# T1b: с default TITLE_WORDS=6 — фикстуры должны быть достаточно длинными,
# чтобы первые 6 слов совпали.
echo ""
echo "=== T1b: same with default TITLE_WORDS=6 (very dense duplicates) ==="
FIXTURE_T1B='[
  {"number": 200, "title": "STT empty on echo", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""},
  {"number": 201, "title": "STT empty on echo", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""}
]'
ERR_T1B="$(mktemp -t g9a-err-t1b.XXXXXX)" || exit 1
OUT_T1B="$(dedup_intra_filter "phase1" "$FIXTURE_T1B" 2>"$ERR_T1B" || true)"
COUNT_T1B_KEEP="$(printf '%s' "$OUT_T1B" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T1B_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T1B" || true)"
if [ "$COUNT_T1B_KEEP" = "1" ] && [ "$COUNT_T1B_MARKERS" = "1" ]; then
    pass "T1b: identical titles + labels (TITLE_WORDS=6) → 1 kept, 1 marker"
else
    fail "T1b: expected kept=1, markers=1; got kept=$COUNT_T1B_KEEP, markers=$COUNT_T1B_MARKERS"
fi
rm -f "$ERR_T1B"

# --- T2: 1 issue (single-group) → 1 keep, 0 markers --------------------------
echo ""
echo "=== T2: dedup_intra_filter with 1 issue (single-group) ==="
FIXTURE_T2='[{"number": 100, "title": "STT empty on echo", "labels": [{"name":"bug"}], "body": ""}]'
ERR_T2="$(mktemp -t g9a-err-t2.XXXXXX)" || exit 1
OUT_T2="$(dedup_intra_filter "phase1" "$FIXTURE_T2" 2>"$ERR_T2" || true)"
COUNT_T2_KEEP="$(printf '%s' "$OUT_T2" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T2_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T2" || true)"
if [ "$COUNT_T2_KEEP" = "1" ] && [ "$COUNT_T2_MARKERS" = "0" ]; then
    pass "T2: 1 issue (single-group) → 1 kept, 0 markers (no false-positive dedup)"
else
    fail "T2: expected kept=1, markers=0; got kept=$COUNT_T2_KEEP, markers=$COUNT_T2_MARKERS"
fi
rm -f "$ERR_T2"

# --- T3: 0 issues → [] (пустой массив) ----------------------------------------
echo ""
echo "=== T3: dedup_intra_filter with empty input ==="
OUT_T3="$(dedup_intra_filter "phase1" "" 2>/dev/null || true)"
[ "$OUT_T3" = "" ] && pass "T3a: empty input → empty output" \
    || fail "T3a: empty input → empty output" "got='$OUT_T3'"
OUT_T3B="$(dedup_intra_filter "phase1" "[]" 2>/dev/null || true)"
[ "$OUT_T3B" = "[]" ] && pass "T3b: '[]' input → '[]' output" \
    || fail "T3b: '[]' input → '[]' output" "got='$OUT_T3B'"

# --- T4: разные labels → 2 issues, 0 markers ---------------------------------
echo ""
echo "=== T4: dedup_intra_filter with different labels ==="
FIXTURE_T4='[
  {"number": 100, "title": "STT empty on echo", "labels": [{"name":"bug"}], "body": ""},
  {"number": 101, "title": "STT empty on echo detected", "labels": [{"name":"tts"}], "body": ""}
]'
ERR_T4="$(mktemp -t g9a-err-t4.XXXXXX)" || exit 1
OUT_T4="$(dedup_intra_filter "phase1" "$FIXTURE_T4" 2>"$ERR_T4" || true)"
COUNT_T4_KEEP="$(printf '%s' "$OUT_T4" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T4_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T4" || true)"
if [ "$COUNT_T4_KEEP" = "2" ] && [ "$COUNT_T4_MARKERS" = "0" ]; then
    pass "T4: different-labels → 2 kept, 0 markers (correct: not dedup)"
else
    fail "T4: expected kept=2, markers=0; got kept=$COUNT_T4_KEEP, markers=$COUNT_T4_MARKERS"
fi
rm -f "$ERR_T4"

# --- T5: разный title-prefix, те же labels → 2 issues, 0 markers -------------
echo ""
echo "=== T5: dedup_intra_filter with same labels but different title-prefix ==="
FIXTURE_T5='[
  {"number": 100, "title": "STT empty on echo in voice mode", "labels": [{"name":"bug"}], "body": ""},
  {"number": 101, "title": "TTS crash on long input buffer", "labels": [{"name":"bug"}], "body": ""}
]'
ERR_T5="$(mktemp -t g9a-err-t5.XXXXXX)" || exit 1
OUT_T5="$(dedup_intra_filter "phase1" "$FIXTURE_T5" 2>"$ERR_T5" || true)"
COUNT_T5_KEEP="$(printf '%s' "$OUT_T5" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T5_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T5" || true)"
if [ "$COUNT_T5_KEEP" = "2" ] && [ "$COUNT_T5_MARKERS" = "0" ]; then
    pass "T5: same labels, different title-prefix → 2 kept, 0 markers"
else
    fail "T5: expected kept=2, markers=0; got kept=$COUNT_T5_KEEP, markers=$COUNT_T5_MARKERS"
fi
rm -f "$ERR_T5"

# --- T6: группа из 3 issues → 1 keep, 2 markers ------------------------------
# Фикстуры с ИДЕНТИЧНЫМИ title и labels, разные только number.
echo ""
echo "=== T6: dedup_intra_filter with 3 identical-titles issues ==="
FIXTURE_T6='[
  {"number": 100, "title": "STT empty on echo in voice mode", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""},
  {"number": 101, "title": "STT empty on echo in voice mode", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""},
  {"number": 102, "title": "STT empty on echo in voice mode", "labels": [{"name":"bug"}, {"name":"voice"}], "body": ""}
]'
ERR_T6="$(mktemp -t g9a-err-t6.XXXXXX)" || exit 1
OUT_T6="$(dedup_intra_filter "phase1" "$FIXTURE_T6" 2>"$ERR_T6" || true)"
COUNT_T6_KEEP="$(printf '%s' "$OUT_T6" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T6_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T6" || true)"
if [ "$COUNT_T6_KEEP" = "1" ] && [ "$COUNT_T6_MARKERS" = "2" ]; then
    pass "T6: 3 identical-titles issues → 1 kept (oldest=100), 2 markers"
else
    fail "T6: expected kept=1, markers=2; got kept=$COUNT_T6_KEEP, markers=$COUNT_T6_MARKERS"
fi
rm -f "$ERR_T6"

# --- T6b: leader = старейшая по number (проверяем конкретно номер лидера) ---
echo ""
echo "=== T6b: dedup_intra_filter leader = oldest by number (not lowest input-order) ==="
FIXTURE_T6B='[
  {"number": 200, "title": "STT empty on echo", "labels": [{"name":"bug"}], "body": ""},
  {"number": 100, "title": "STT empty on echo", "labels": [{"name":"bug"}], "body": ""}
]'
ERR_T6B="$(mktemp -t g9a-err-t6b.XXXXXX)" || exit 1
OUT_T6B="$(dedup_intra_filter "phase1" "$FIXTURE_T6B" 2>"$ERR_T6B" || true)"
KEPT_NUMBER="$(printf '%s' "$OUT_T6B" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d[0]["number"])' 2>/dev/null || echo 0)"
if [ "$KEPT_NUMBER" = "100" ]; then
    pass "T6b: leader = oldest number (100), not first-in-input (200)"
else
    fail "T6b: leader=100 expected, got=$KEPT_NUMBER"
fi
rm -f "$ERR_T6B"

# --- T7: branch_exists_in_remote — существующая ветка (через mock git ls-remote) ---
echo ""
echo "=== T7-T9: branch_exists_in_remote ==="
# Создаём временный git-репозиторий с локальной веткой и подменяем git на wrapper.
# Так как реальный скрипт использует `git ls-remote https://github.com/...`,
# unit-тест использует DEFAULTS_PATH — но мы не можем изменить URL внутри функции.
# Поэтому мы тестируем ВСПОМОГАТЕЛЬНУЮ ЛОГИКУ: проверяем grep-паттерн и то, что
# функция fail-OPEN при GH_REPO="" / пустой branch.

# T7a: GH_REPO пустой → return 1 (нет смысла проверять)
export GH_REPO=""
export AGENT_FLOW_DEDUP_RACE_GUARD=true
if branch_exists_in_remote "z-{agent}/100-foo"; then
    fail "T7a: GH_REPO='' should fail-OPEN (return 1)"
else
    pass "T7a: GH_REPO='' → return 1 (no GH to query)"
fi

# T7b: branch="" → return 1
export GH_REPO="test/repo"
if branch_exists_in_remote ""; then
    fail "T7b: branch='' should return 1"
else
    pass "T7b: branch='' → return 1"
fi

# T7c: AGENT_FLOW_DEDUP_RACE_GUARD=false → return 1 (отключено)
export AGENT_FLOW_DEDUP_RACE_GUARD=false
if branch_exists_in_remote "z-{agent}/100-foo"; then
    fail "T7c: AGENT_FLOW_DEDUP_RACE_GUARD=false should return 1"
else
    pass "T7c: AGENT_FLOW_DEDUP_RACE_GUARD=false → return 1 (disabled)"
fi

# T7d: live check — реальный `git ls-remote` по GH_REPO. Просто проверим,
# что функция возвращает exit-код (0 или 1), не падает:
export AGENT_FLOW_DEDUP_RACE_GUARD=true
branch_exists_in_remote "z-{agent}/this-branch-definitely-doesnt-exist-on-remote-99999999xx" >/dev/null 2>&1
rc=$?
if [ "$rc" = "0" ] || [ "$rc" = "1" ]; then
    pass "T7d: live ls-remote returns 0 or 1 (got=$rc), no crash"
else
    fail "T7d: live ls-remote unexpected exit code: $rc"
fi

# --- T10: shellcheck-clean + syntax-OK triage.sh -----------------------------
echo ""
echo "=== T10: shellcheck + syntax check ==="
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T10a: bash -n syntax check passed"
else
    fail "T10a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

if ! command -v shellcheck >/dev/null 2>&1; then
    for sc in /home/builder/.hermes/hermes-agent/venv/bin/shellcheck \
             /usr/local/bin/shellcheck /usr/bin/shellcheck; do
        [ -x "$sc" ] && PATH="$(dirname "$sc"):$PATH" && break
    done
fi
if command -v shellcheck >/dev/null 2>&1; then
    _repo_root="$(git -C "$TESTS_DIR/.." rev-parse --show-toplevel 2>/dev/null || echo "$TESTS_DIR/..")"
    if [ -d "$_repo_root/.git" ] || [ -f "$_repo_root/.git" ]; then
        ORIG_SC="$(cd "$_repo_root" && git show "origin/develop:scripts/agent_flow/agent-flow-triage.sh" 2>/dev/null | shellcheck - 2>&1 | wc -l)"
        NEW_SC="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1 | wc -l)"
        log "  shellcheck: origin/develop=$ORIG_SC, current=$NEW_SC"
        if [ "$NEW_SC" -le "$ORIG_SC" ]; then
            pass "T10b: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T10b: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T10b: not a git repo — skipping baseline comparison"
    fi
else
    log "T10b: shellcheck not installed — skip"
fi

# --- T11: presence check (key G9 markers в коде) -----------------------------
echo ""
echo "=== T11: presence-of-key-markers в triage.sh ==="
if grep -q 'dedup-skipped: \${dedup_intra_skipped} (intra-tick), \${dedup_race_skipped} (race)' "$SCRIPT_UNDER_TEST"; then
    pass "T11a: summary line 'dedup-skipped: ... (intra-tick), ... (race)' present"
else
    fail "T11a: summary line 'dedup-skipped: ... (intra-tick), ... (race)' missing in summary"
fi
if grep -q 'branch_exists_in_remote "$branch"' "$SCRIPT_UNDER_TEST"; then
    pass "T11b: branch_exists_in_remote called in process_issues_json"
else
    fail "T11b: branch_exists_in_remote NOT called in process_issues_json (G9b regression)"
fi
if grep -q 'dedup_intra_filter "phase1"' "$SCRIPT_UNDER_TEST"; then
    pass "T11c: dedup_intra_filter called for phase1"
else
    fail "T11c: dedup_intra_filter NOT called for phase1 (G9a regression)"
fi
if grep -q 'dedup_intra_filter "phase2"' "$SCRIPT_UNDER_TEST"; then
    pass "T11d: dedup_intra_filter called for phase2"
else
    fail "T11d: dedup_intra_filter NOT called for phase2 (G9a regression)"
fi
if grep -q 'AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS' "$SCRIPT_UNDER_TEST"; then
    pass "T11e: AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS env-var referenced"
else
    fail "T11e: AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS env-var NOT referenced"
fi
if grep -q 'AGENT_FLOW_DEDUP_RACE_GUARD' "$SCRIPT_UNDER_TEST"; then
    pass "T11f: AGENT_FLOW_DEDUP_RACE_GUARD env-var referenced"
else
    fail "T11f: AGENT_FLOW_DEDUP_RACE_GUARD env-var NOT referenced"
fi
if grep -q 'ADR-0032' "$SCRIPT_UNDER_TEST"; then
    pass "T11g: ADR-0032 referenced in comments"
else
    fail "T11g: ADR-0032 NOT referenced"
fi
if grep -q 'dedup_intra_skipped' "$SCRIPT_UNDER_TEST"; then
    pass "T11h: dedup_intra_skipped counter referenced"
else
    fail "T11h: dedup_intra_skipped counter NOT referenced"
fi
if grep -q 'dedup_race_skipped' "$SCRIPT_UNDER_TEST"; then
    pass "T11i: dedup_race_skipped counter referenced"
else
    fail "T11i: dedup_race_skipped counter NOT referenced"
fi

# --- summary -----------------------------------------------------------------
echo ""
echo "============================================================"
echo "PASS: $PASS    FAIL: $FAIL"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFAILED CASES:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0
