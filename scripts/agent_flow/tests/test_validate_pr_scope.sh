#!/bin/bash
# ============================================================================
# test_validate_pr_scope.sh — регресс-тест для validate_pr_scope.sh
# (issue #2038, ADR-0055: scope-drift gate, post-flight guard).
#
# Сценарии:
#   A) Нет diff → exit 0 (OK)
#   B) Только allowed prefix файлы → exit 0 (OK)
#   C) Prefix + glob → exit 0
#   D) Out-of-scope файлы → exit 1 + правильный текст в stderr
#   E) INFO-режим (без PR_ALLOWED_PREFIXES) → exit 0 (печатает файлы)
#   F) SKIP_PR_SCOPE=true → exit 0
#   G) BASE_REF недоступен → exit 2 (usage error)
#   H) Реальный repro на фикстуре PR #2036 (смоделированный)
#
# Exit codes:
#   0 — все сценарии прошли
#   N — номер первого упавшего сценария (с raw-выводом)
# ============================================================================
set -uo pipefail   # без -e (clone может вернуть non-zero на warning)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOOK="${SCRIPT_DIR}/../validate_pr_scope.sh"

if [ ! -x "$HOOK" ]; then
    echo "FAIL: $HOOK not executable"
    exit 1
fi

WORK="$(mktemp -d -t pr-scope-test.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

# --- bare origin с default branch main ---
git init -q --bare --initial-branch=main "$WORK/origin.git"
git -C "$WORK/origin.git" symbolic-ref HEAD refs/heads/main

# --- repo (main, с базовыми файлами) ---
git clone -q "$WORK/origin.git" "$WORK/repo"
git -C "$WORK/repo" config user.email "test@local"
git -C "$WORK/repo" config user.name "test"
echo "# base" > "$WORK/repo/README.md"
mkdir -p "$WORK/repo/docs/adr" "$WORK/repo/src/rob_box_voice" "$WORK/repo/src/rob_box_quest/webxr_client" "$WORK/repo/scripts/agent_flow"
echo "adr" > "$WORK/repo/docs/adr/0001-base.md"
echo "voice" > "$WORK/repo/src/rob_box_voice/voice.py"
echo "webxr" > "$WORK/repo/src/rob_box_quest/webxr_client/main.ts"
echo "agent-flow" > "$WORK/repo/scripts/agent_flow/lib.sh"
git -C "$WORK/repo" add -A
git -C "$WORK/repo" commit -q -m "init"
git -C "$WORK/repo" push -q origin main

ok() { printf '\033[32m  OK\033[0m %s\n' "$1"; }
fail() { printf '\033[31m  FAIL\033[0m %s\n' "$1"; echo "    $2" >&2; exit "${3:-1}"; }

# === Сценарий A: пустой diff → exit 0 ===
echo "Scenario A: no diff → OK"
git -C "$WORK/repo" checkout -q main
out="$(cd "$WORK/repo" && "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug A] rc=$rc out=$out"
[ "$rc" = "0" ] || fail "A" "expected 0, got $rc; out=$out" 1
echo "$out" | grep -q "no file changes" \
    || fail "A" "missing 'no file changes'; got: $out" 2
ok "A — пустой diff OK"

# === Сценарий B: только allowed prefix (docs/adr/) →
# в коммите PR только docs/adr/0002-new.md
echo "Scenario B: only allowed prefix → OK"
git -C "$WORK/repo" checkout -q -b feat-adr main
echo "adr2" > "$WORK/repo/docs/adr/0002-new.md"
git -C "$WORK/repo" add -A
git -C "$WORK/repo" commit -q -m "feat(adr): add 0002"
git -C "$WORK/repo" push -q origin feat-adr
out="$(cd "$WORK/repo" && \
    PR_ALLOWED_PREFIXES="docs/adr/" "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug B] rc=$rc out=$out"
[ "$rc" = "0" ] || fail "B" "expected 0, got $rc; out=$out" 3
echo "$out" | grep -q "OK: all 1 files in allowed scope" \
    || fail "B" "missing OK marker; got: $out" 4
ok "B — prefix-match OK"

# === Сценарий C: prefix + glob (ТОЛЬКО glob *.md вне prefix) →
# в коммите PR docs/adr/0003.md + scripts/agent_flow/README.md, scope=*.md
echo "Scenario C: prefix + glob → OK"
git -C "$WORK/repo" checkout -q main
git -C "$WORK/repo" checkout -q -b feat-glob main
echo "adr3" > "$WORK/repo/docs/adr/0003.md"
mkdir -p "$WORK/repo/scripts/agent_flow"
echo "readme" > "$WORK/repo/scripts/agent_flow/README.md"
git -C "$WORK/repo" add -A
git -C "$WORK/repo" commit -q -m "feat: *.md files"
git -C "$WORK/repo" push -q origin feat-glob
out="$(cd "$WORK/repo" && \
    PR_ALLOWED_PREFIXES="docs/adr/" PR_ALLOWED_GLOBS="*.md" "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug C] rc=$rc out=$out"
[ "$rc" = "0" ] || fail "C" "expected 0, got $rc; out=$out" 5
ok "C — prefix+glob OK"

# === Сценарий D: out-of-scope → exit 1 (модель PR #2036) ===
# В коммите PR: 2 своих + 12 чужих (webxr_client/* + scripts/agent_flow/*)
echo "Scenario D: scope drift — 12 out-of-scope → FAIL exit 1"
git -C "$WORK/repo" checkout -q main
git -C "$WORK/repo" checkout -q -b feat-drift main
mkdir -p "$WORK/repo/docs/adr" "$WORK/repo/src/rob_box_quest/webxr_client/src" \
         "$WORK/repo/src/rob_box_quest/webxr_client/tests"
# 2 своих файла
echo "wake" > "$WORK/repo/docs/adr/0054-wake-stream.md"
echo "wake2" > "$WORK/repo/src/rob_box_voice/wake.py"
# 12 чужих (как в PR #2036)
for f in src/main.ts src/scene/status_hud.ts src/state/supervisor_state.ts \
         src/style.css src/ui/toast.ts src/wire/connection.ts src/wire/msgpack.ts \
         src/wire/protocol.ts tests/msgpack.test.ts tests/status_hud.test.ts \
         tests/supervisor_state.test.ts tests/supervisor_wire.test.ts; do
    mkdir -p "$WORK/repo/src/rob_box_quest/webxr_client/$(dirname "$f")"
    echo "x" > "$WORK/repo/src/rob_box_quest/webxr_client/$f"
done
# 1 скрипт — выглядит как drift от предыдущей задачи
echo "lib2" > "$WORK/repo/scripts/agent_flow/leftover.sh"
git -C "$WORK/repo" add -A
git -C "$WORK/repo" commit -q -m "feat(adr-0054): wake stream + 12 AV-17 leftovers"
git -C "$WORK/repo" push -q origin feat-drift
out="$(cd "$WORK/repo" && \
    PR_ALLOWED_PREFIXES="docs/adr/,src/rob_box_voice/" "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug D] rc=$rc out=$(printf '%s' "$out" | head -5)"
[ "$rc" = "1" ] || fail "D" "expected 1, got $rc; out=$out" 6
echo "$out" | grep -q "FAIL: 13 of 15 files in diff" \
    || fail "D" "missing FAIL marker (13 of 15); got: $out" 7
echo "$out" | grep -q "src/rob_box_quest/webxr_client/src/main.ts" \
    || fail "D" "missing one of the 12 webxr_client files in listing; got: $out" 8
echo "$out" | grep -q "scripts/agent_flow/leftover.sh" \
    || fail "D" "missing leftover.sh; got: $out" 9
ok "D — out-of-scope DETECTED и перечислен"

# === Сценарий E: INFO-режим (без PR_ALLOWED_PREFIXES) → exit 0 ===
# Используем feat-adr (только 1 файл) чтобы не сработал defensive MAX_OUT_OF_SCOPE=10.
echo "Scenario E: INFO-режим — exit 0, печатает файлы"
git -C "$WORK/repo" checkout -q feat-adr
out="$(cd "$WORK/repo" && "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug E] rc=$rc out=$(printf '%s' "$out" | head -3)"
[ "$rc" = "0" ] || fail "E" "expected 0, got $rc; out=$out" 10
echo "$out" | grep -q "INFO: PR_ALLOWED_PREFIXES not set" \
    || fail "E" "missing INFO marker; got: $out" 11
ok "E — INFO-режим работает"

# === Сценарий E2: INFO + defensive MAX_OUT_OF_SCOPE > 10 → exit 1 ===
# Проверяем, что defensive guard ловит явный drift даже без allowed-prefixes.
echo "Scenario E2: INFO-режим + defensive guard на >10 файлах → exit 1"
git -C "$WORK/repo" checkout -q feat-drift
out="$(cd "$WORK/repo" && "$HOOK" origin/main 2>&1)"; rc=$?
echo "  [debug E2] rc=$rc"
[ "$rc" = "1" ] || fail "E2" "expected 1 (defensive), got $rc; out=$out" 11
echo "$out" | grep -q "WARN: 15 files in diff > MAX_OUT_OF_SCOPE=10" \
    || fail "E2" "missing WARN marker; got: $out" 11
ok "E2 — defensive guard работает"

# === Сценарий F: SKIP_PR_SCOPE=true → exit 0 ===
echo "Scenario F: SKIP_PR_SCOPE=true → exit 0"
out="$(cd "$WORK/repo" && SKIP_PR_SCOPE=true "$HOOK" origin/main 2>&1)"; rc=$?
[ "$rc" = "0" ] || fail "F" "expected 0, got $rc; out=$out" 12
echo "$out" | grep -q "SKIP via SKIP_PR_SCOPE=true" \
    || fail "F" "missing SKIP marker; got: $out" 13
ok "F — SKIP honored"

# === Сценарий G: BASE_REF недоступен → exit 2 ===
echo "Scenario G: BASE_REF unknown → exit 2"
out="$(cd "$WORK/repo" && "$HOOK" origin/totally-fake-branch 2>&1)"; rc=$?
echo "  [debug G] rc=$rc out=$(printf '%s' "$out" | head -3)"
[ "$rc" = "2" ] || fail "G" "expected 2, got $rc; out=$out" 14
ok "G — bad base ref rejected"

# === Сценарий H: merge-commit (CI) → skip ===
echo "Scenario H: merge-commit skip → exit 0"
out="$(cd "$WORK/repo" && \
    GITHUB_EVENT_NAME=pull_request MERGE_COMMIT_INFERRED=true "$HOOK" origin/main 2>&1)"; rc=$?
[ "$rc" = "0" ] || fail "H" "expected 0, got $rc; out=$out" 15
echo "$out" | grep -q "skip: merge-commit" \
    || fail "H" "missing skip marker; got: $out" 16
ok "H — merge-commit SKIPPED"

echo
echo "All scenarios PASSED"
