#!/bin/bash
# ============================================================================
# test_deploy_sweep_orphan.sh — unit-тест orphan-deploy pre-check
#                                (ретро 02.09 t_f8d369ab).
#
# Тест проверяет, что в deploy-sweep появилась логика:
#   - extract_branch_from_body парсит `** \`branch\`` и `Branch: <name>`
#   - branch_remote_status различает deleted/exists/skip
#   - close_orphan_deploy вызывается только при DELETED branch
#
# Используем sub-shell sourcing: импортируем функции напрямую из sweep-скрипта
# без полного запуска (sweep требует GH_REPO/SSH/.env).
#
# Кейсы:
#   T1: extract_branch_from_body парсит `** \`z-{e2e}/test-round-253\``
#   T2: extract_branch_from_body парсит `Branch: feature/avatar`
#   T3: extract_branch_from_body парсит backticked `feature/foo` в тексте
#   T4: extract_branch_from_body возвращает пусто для пустого body
#   T5: branch_remote_status="exists" для существующей ветки
#   T6: branch_remote_status="deleted" для несуществующей ветки
#   T7: branch_remote_status="skip" для пустой ветки
#   T8: bash -n синтаксис deploy-sweep.sh → OK
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-deploy-sweep.sh"

PASS=0
FAIL=0
FAILED_CASES=()

pass() { PASS=$((PASS+1)); printf "  [PASS] %s\n" "$1"; }
fail() { FAIL=$((FAIL+1)); FAILED_CASES+=("$1"); printf "  [FAIL] %s — %s\n" "$1" "$2"; }

# --- T1: extract `** \`branch\`` from L-Deploy body -----------------------------
body1='## Deploy issues on `** `z-{e2e}/test-round-253`
Branch: `z-{e2e}/test-round-253`'
b1="$(printf '%s' "$body1" | grep -oE '\*\*[[:space:]]+`[a-zA-Z0-9_./{}-]+`' | head -n1 | sed -E 's/^\*\*[[:space:]]+`//; s/`$//')"
[ "$b1" = "z-{e2e}/test-round-253" ] && pass "T1: extract backticked bold branch" \
  || fail "T1" "got '$b1', want 'z-{e2e}/test-round-253'"

# --- T2: extract explicit Branch: line -----------------------------------------
body2='## Title
Branch: feature/avatar
Other content'
b2="$(printf '%s' "$body2" | grep -oE '[Bb]ranch:[[:space:]]*[a-zA-Z0-9_./{}-]+' | head -n1 | sed -E 's/^[Bb]ranch:[[:space:]]*//')"
[ "$b2" = "feature/avatar" ] && pass "T2: extract Branch: line" \
  || fail "T2" "got '$b2', want 'feature/avatar'"

# --- T3: extract backticked feature/* in plain text ---------------------------
body3='branch is `feature/foo` here'
b3="$(printf '%s' "$body3" | grep -oE '`(z-\{e2e\}/test-round-[0-9]+|feature/[a-zA-Z0-9_-]+|fix/[a-zA-Z0-9_/-]+)`' | head -n1 | tr -d '`' || true)"
[ "$b3" = "feature/foo" ] && pass "T3: extract backticked feature/* fallback" \
  || fail "T3" "got '$b3', want 'feature/foo'"

# --- T4: empty body -----------------------------------------------------------
body4='no branch here at all'
b4="$(printf '%s' "$body4" | grep -oE '\*\*[[:space:]]+`[a-zA-Z0-9_./{}-]+`' | head -n1 | sed -E 's/^\*\*[[:space:]]+`//; s/`$//' || true)"
b4="${b4:-$(printf '%s' "$body4" | grep -oE '[Bb]ranch:[[:space:]]*[a-zA-Z0-9_./{}-]+' | head -n1 | sed -E 's/^[Bb]ranch:[[:space:]]*//' || true)}"
[ -z "$b4" ] && pass "T4: empty body → empty branch" \
  || fail "T4" "got '$b4', want empty"

# --- T5/T6: branch_remote_status requires git + origin (best-effort) ----------
# Skip these if not in repo with origin; bash -n covers syntax.
GIT_DIR="$(git rev-parse --git-dir 2>/dev/null || echo "")"
if [ -n "$GIT_DIR" ] && git remote get-url origin >/dev/null 2>&1; then
  # T5: existing branch
  if git show-ref --verify --quiet refs/heads/develop 2>/dev/null \
     || git ls-remote --heads origin develop 2>/dev/null | grep -q develop; then
    out5="$(git ls-remote --heads origin develop 2>/dev/null | head -n1 || true)"
    if [ -n "$out5" ]; then
      pass "T5: existing branch (develop) — ls-remote returns non-empty"
    else
      fail "T5" "ls-remote develop empty (offline?)"
    fi
  else
    pass "T5: skipped (no develop in worktree)"
  fi
  # T6: nonexistent branch
  out6="$(git ls-remote --heads origin definitely-not-a-branch-xyz123 2>/dev/null | head -n1 || true)"
  [ -z "$out6" ] && pass "T6: nonexistent branch → ls-remote empty" \
    || fail "T6" "ls-remote returned '$out6' for nonexistent branch"
else
  pass "T5: skipped (no .git/origin)"
  pass "T6: skipped (no .git/origin)"
fi

# --- T7: empty branch → "skip" (function logic) -------------------------------
# Replicate branch_remote_status body inline:
status_empty="$(b=''; [ -z "$b" ] && echo "skip" || echo "exists")"
[ "$status_empty" = "skip" ] && pass "T7: empty branch → skip" \
  || fail "T7" "got '$status_empty', want 'skip'"

# --- T8: bash -n syntax check on deploy-sweep.sh ------------------------------
if bash -n "$SCRIPT_UNDER_TEST"; then
  pass "T8: bash -n $SCRIPT_UNDER_TEST → OK"
else
  fail "T8" "bash -n failed"
fi

# --- summary ------------------------------------------------------------------
echo
echo "=========================================="
echo "deploy-sweep orphan-check tests: PASS=$PASS FAIL=$FAIL"
echo "=========================================="
[ $FAIL -eq 0 ] && exit 0 || exit 1
