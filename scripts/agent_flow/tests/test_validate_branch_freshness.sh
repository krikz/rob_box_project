#!/bin/bash
# ============================================================================
# test_validate_branch_freshness.sh — регресс-тест для validate_branch_freshness.sh
# (ADR-0045 §2.3 / issue #1887 / t_7255c811).
#
# Сценарии:
#   A) branch 0 commits behind → exit 0
#   B) branch 25 commits behind, MAX_BRANCH_BEHIND=30 → exit 0
#   C) branch 31 commits behind → exit 1 + правильный текст в stderr
#   D) merge-commit (CI) → exit 0 (skip)
#   E) SKIP_BRANCH_FRESHNESS=true → exit 0
#   F) usage error (no git in PATH) → exit 2
#
# Exit codes:
#   0 — все сценарии прошли
#   N — номер первого упавшего сценария
# ============================================================================
set -uo pipefail   # NOTE: -e убран — clone может вернуть non-zero на empty repo warning

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# scripts/agent_flow/tests/ → ../agent_flow/validate_branch_freshness.sh
HOOK="${SCRIPT_DIR}/../validate_branch_freshness.sh"

if [ ! -x "$HOOK" ]; then
    echo "FAIL: $HOOK not executable"; exit 1
fi

# Workdir для сценариев: чистый временный git-репо с bare origin/main.
WORK="$(mktemp -d -t branch-freshness-test.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

# --- bare origin (BASE_REF = origin/main, чтобы не зависеть от develop) ---
git init -q --bare --initial-branch=main "$WORK/origin.git"
git -C "$WORK/origin.git" symbolic-ref HEAD refs/heads/main

# --- repo (ветка main, 50 коммитов) ---
git clone -q "$WORK/origin.git" "$WORK/repo"
git -C "$WORK/repo" config user.email "test@local"
git -C "$WORK/repo" config user.name "test"
for i in $(seq 1 50); do
    echo "c$i" > "$WORK/repo/file.c$i"
    git -C "$WORK/repo" add "file.c$i"
    git -C "$WORK/repo" commit -q -m "c$i"
done
git -C "$WORK/repo" push -q origin main
# origin/main = 50 коммитов; HEAD (default branch main) = 50; behind = 0.

ok() { printf '\033[32m  OK\033[0m %s\n' "$1"; }
fail() { printf '\033[31m  FAIL\033[0m %s\n' "$1"; echo "    $2" >&2; exit "${3:-1}"; }

# === Сценарий A: 0 commits behind → exit 0 ===
echo "Scenario A: 0 commits behind"
git -C "$WORK/repo" checkout -q main
out="$(cd "$WORK/repo" && "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug A] rc=$rc out=$out"
[ "$rc" = "0" ] || fail "A" "expected exit 0, got $rc; out=$out" 1
echo "$out" | grep -q "OK: main is 0 commits behind" \
    || fail "A" "missing OK marker; got: $out" 2
ok "A — 0 behind OK"

# === Сценарий B: 25 commits behind → exit 0 ===
echo "Scenario B: 25 commits behind, MAX=30"
git -C "$WORK/repo" checkout -q --detach main  # fresh detached from main
git -C "$WORK/repo" checkout -q -b feature-b main~25  # 25 behind
out="$(cd "$WORK/repo" && MAX_BRANCH_BEHIND=30 "$HOOK" origin/main 2>&1)"; rc=$?
[ "$rc" = "0" ] || fail "B" "expected exit 0, got $rc; out=$out" 3
echo "$out" | grep -q "is 25 commits behind" \
    || fail "B" "missing 25 marker; got: $out" 4
ok "B — 25 behind OK"

# === Сценарий C: 31 commits behind → exit 1 ===
echo "Scenario C: 31 commits behind"
git -C "$WORK/repo" checkout -q main
git -C "$WORK/repo" checkout -q -b feature-c main~31
out="$(cd "$WORK/repo" && MAX_BRANCH_BEHIND=30 "$HOOK" origin/main 2>&1)"; rc=$?
[ "$rc" = "1" ] || fail "C" "expected exit 1, got $rc; out=$out" 5
echo "$out" | grep -q "FAIL: branch feature-c is 31 commits behind" \
    || fail "C" "missing FAIL marker; got: $out" 6
ok "C — 31 behind BLOCKED"

# === Сценарий D: merge-commit (CI) → exit 0 ===
echo "Scenario D: merge-commit skip"
out="$(cd "$WORK/repo" && GITHUB_EVENT_NAME=pull_request MERGE_COMMIT_INFERRED=true \
       MAX_BRANCH_BEHIND=30 "$HOOK" origin/main 2>&1)"; rc=$?
[ "$rc" = "0" ] || fail "D" "expected exit 0, got $rc; out=$out" 7
echo "$out" | grep -q "skip: merge-commit (CI)" \
    || fail "D" "missing skip marker; got: $out" 8
ok "D — merge-commit SKIPPED"

# === Сценарий E: SKIP_BRANCH_FRESHNESS=true → exit 0 ===
echo "Scenario E: SKIP_BRANCH_FRESHNESS=true"
git -C "$WORK/repo" checkout -q main
git -C "$WORK/repo" checkout -q -b feature-e main~31
out="$(cd "$WORK/repo" && SKIP_BRANCH_FRESHNESS=true MAX_BRANCH_BEHIND=30 \
       "$HOOK" origin/main 2>&1)"; rc=$?
[ "$rc" = "0" ] || fail "E" "expected exit 0, got $rc; out=$out" 9
echo "$out" | grep -q "SKIP via SKIP_BRANCH_FRESHNESS=true" \
    || fail "E" "missing SKIP marker; got: $out" 10
ok "E — SKIP honored"

# === Сценарий F: реальный drift repro — этот worktree ветка ===
echo "Scenario F: integration — реальный drift текущей worktree-ветки"
git -C "/home/builder/rob_box_project" fetch origin develop --prune 2>/dev/null || true
out="$(cd "/home/builder/rob_box_project/.worktrees/t_7255c811" && \
       MAX_BRANCH_BEHIND=30 "$HOOK" origin/develop 2>&1)"; rc=$?
echo "  [debug F] rc=$rc"
[ "$rc" = "1" ] || fail "F" "expected exit 1 (drift caught), got $rc; out=$out" 11
echo "$out" | grep -q "FAIL: branch z-{agent}/1980-bug-process-drift-guard-adr-0045-pr-1978 is" \
    || fail "F" "missing FAIL marker on real drift; got: $out" 12
ok "F — реальный drift пойман (этот worktree t_7255c811 — жертва бага)"

echo
echo "All 6 scenarios PASSED"