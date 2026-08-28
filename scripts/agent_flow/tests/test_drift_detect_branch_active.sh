#!/bin/bash
# ============================================================================
# test_drift_detect_branch_active.sh — ретро 14.08 t_ea771b06
#
# Регресс-гард для автофикса drift-detect при BRANCH_ACTIVE (главный worktree
# репо на фича-ветке воркера, current branch != develop).
#
# Проблема: при BRANCH_ACTIVE + DRIFT (host-копии != origin/develop) drift-detect
# раньше просто создавал карточку (auto-fix deferred), потому что install.sh из
# текущего дерева разложил бы ВЕТОЧНЫЙ код. Это 2-й случай ручной разборки
# (16:24 14.08 merge-gate+e2e-process; 01:52 14.08 triage).
#
# Решение: автофикс из ВРЕМЕННОГО worktree на origin/develop:
#   git worktree add --detach <wt> origin/develop
#   REPO_DIR=<wt> bash <wt>/scripts/agent_flow/install.sh
#   git worktree remove --force <wt>
# Карточка создаётся ТОЛЬКО если и этот путь не помог (md5-сверка после).
#
# Проверяем на фикстуре из реального git-репо:
#   A. BRANCH_ACTIVE + дрейф, который лечится  -> exit 0, карточки НЕТ,
#      md5 всех 12 файлов × 4 host-пути == origin/develop, worktree убран.
#   B. BRANCH_ACTIVE + дрейф, который НЕ лечится (install.sh из worktree
#      падает: origin/develop без watchdog.sh) -> exit 0 (BRANCH_ACTIVE не
#      валит cron), карточка создаётся (как раньше).
#   C. Идемпотентность: повторный запуск после вылеченного дрейфа ->
#      exit 0, карточки НЕТ, md5 не изменился.
#
# Run:
#   bash scripts/agent_flow/tests/test_drift_detect_branch_active.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
DRIFT_SH="$TEST_DIR/../agent-flow-drift-detect.sh"
AGENT_FLOW_DIR="$(cd "$TEST_DIR/.." && pwd)"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -f "$DRIFT_SH" ] || fail "drift-detect script not found: $DRIFT_SH"

# --- фикстура: origin (bare) + рабочее репо с scripts/agent_flow -----------
# Файлы копируем ИЗ РЕАЛЬНОГО agent_flow (install.sh --list-files = 12 имён).
setup_fixture() { # $1 = (optional) файл, отсутствующий в origin/develop
    local missing="${1:-}"
    rm -rf "$WORK/origin.git" "$WORK/repo"
    git init -q --bare "$WORK/origin.git" || fail "git init --bare"
    git init -q "$WORK/repo" || fail "git init repo"
    git -C "$WORK/repo" config user.email test@test
    git -C "$WORK/repo" config user.name test
    git -C "$WORK/repo" remote add origin "$WORK/origin.git"
    mkdir -p "$WORK/repo/scripts/agent_flow"
    cp "$AGENT_FLOW_DIR"/*.sh "$WORK/repo/scripts/agent_flow/"
    if [ -n "$missing" ] && [ -f "$WORK/repo/scripts/agent_flow/$missing" ]; then
        rm -f "$WORK/repo/scripts/agent_flow/$missing"
    fi
    git -C "$WORK/repo" add -A
    git -C "$WORK/repo" commit -qm "fixture: agent_flow scripts"
    git -C "$WORK/repo" branch -M develop
    git -C "$WORK/repo" push -q -u origin develop || fail "push develop"
    # BRANCH_ACTIVE: уводим главное дерево на фича-ветку (как воркер)
    git -C "$WORK/repo" checkout -qb z-test/worker
}

# 4 fixture host-директории (замена реальных ~/.hermes/... для hermetic-теста)
setup_hosts() {
    rm -rf "$WORK/hosts"
    mkdir -p "$WORK/hosts/a" "$WORK/hosts/b" "$WORK/hosts/c" "$WORK/hosts/d"
    for f in "$AGENT_FLOW_DIR"/*.sh; do
        local base
        base="$(basename "$f")"
        for h in a b c d; do
            printf 'STALE %s\n' "$base" > "$WORK/hosts/$h/$base"
        done
    done
}

# фейковый kanban-retro-create.sh: логирует вызов, ничего не создаёт
cat > "$WORK/fake-retro-create.sh" <<'FAKE'
#!/bin/bash
echo "RETRO-CREATE-CALLED $*" >> "${RETRO_JOURNAL:-/dev/null}"
exit 0
FAKE
chmod +x "$WORK/fake-retro-create.sh"

# фейковый hermes: intercept любых вызовов install.sh ensure_cleanup_cron
mkdir -p "$WORK/bin"
cat > "$WORK/bin/hermes" <<'FAKE'
#!/bin/bash
echo "HERMES-CALLED $*" >> "${HERMES_JOURNAL:-/dev/null}"
exit 0
FAKE
chmod +x "$WORK/bin/hermes"

run_drift() {
    PATH="$WORK/bin:$PATH" \
    REPO_DIR="$WORK/repo" \
    DRIFT_TARGETS="$WORK/repo/scripts/agent_flow:$WORK/hosts/a:$WORK/hosts/b:$WORK/hosts/c:$WORK/hosts/d" \
    INSTALL_TARGET_DIRS="$WORK/hosts/a:$WORK/hosts/b:$WORK/hosts/c:$WORK/hosts/d" \
    HERMES_SCRIPTS_DIR="$WORK/hosts/d" \
    HERMES_AGENT_DIR="$WORK/fake-hermes-agent" \
    DRIFT_ALERT_LOG="$WORK/alert.log" \
    RETRO_CREATE="$WORK/fake-retro-create.sh" \
    RETRO_JOURNAL="$WORK/retro.log" \
    HERMES_JOURNAL="$WORK/hermes.log" \
    DRIFT_WT_PREFIX="$WORK/wt-" \
    bash "$DRIFT_SH"
    return $?
}

# md5 файла на origin/develop
origin_md5() { # $1 = file
    git -C "$WORK/repo" show "origin/develop:scripts/agent_flow/$1" 2>/dev/null | md5sum | cut -c1-12
}

# --- A. BRANCH_ACTIVE + дрейф, который лечится ------------------------------
echo "=== TEST A: healable drift -> exit 0, NO card, hosts == origin/develop ==="
setup_fixture
setup_hosts
rm -f "$WORK/retro.log"
OUT="$(run_drift 2>&1)"
RC=$?
echo "$OUT" | sed 's/^/  /'
[ "$RC" = "0" ] || fail "expected exit 0, got $RC"
echo "$OUT" | grep -q "BRANCH_ACTIVE" || fail "missing BRANCH_ACTIVE marker"
echo "$OUT" | grep -q "FIXED" || fail "expected FIXED marker, got: $OUT"
[ ! -f "$WORK/retro.log" ] && pass "no card created" \
    || fail "card should NOT be created: $(cat "$WORK/retro.log")"

# md5 всех 12 файлов × 4 host-пути == origin/develop
FILES_LIST="$(bash "$AGENT_FLOW_DIR/install.sh" --list-files)"
COUNT=0
for f in $FILES_LIST; do
    EXPECTED="$(origin_md5 "$f")"
    [ -n "$EXPECTED" ] || fail "origin/develop missing $f"
    for h in a b c d; do
        CUR="$(md5sum "$WORK/hosts/$h/$f" 2>/dev/null | cut -c1-12)"
        [ "$CUR" = "$EXPECTED" ] || fail "hosts/$h/$f md5=$CUR != origin=$EXPECTED"
        COUNT=$((COUNT+1))
    done
done
pass "md5 all $COUNT host copies == origin/develop"

# worktree убран
git -C "$WORK/repo" worktree list --porcelain 2>/dev/null | grep -q "$WORK/wt-" \
    && fail "temp worktree still registered" || pass "temp worktree removed"

# --- B. BRANCH_ACTIVE + дрейф, который НЕ лечится ---------------------------
echo "=== TEST B: unhealable drift (install.sh fails in worktree) -> card ==="
setup_fixture watchdog.sh   # origin/develop без watchdog.sh -> install.sh exit 2
setup_hosts
rm -f "$WORK/retro.log"
OUT="$(run_drift 2>&1)"
RC=$?
echo "$OUT" | sed 's/^/  /'
[ "$RC" = "0" ] || fail "expected exit 0 (BRANCH_ACTIVE no-op), got $RC"
echo "$OUT" | grep -q "FIX FAILED" || fail "expected FIX FAILED marker"
[ -f "$WORK/retro.log" ] && pass "card created (FIX FAILED)" \
    || fail "card SHOULD be created on unhealable drift"
grep -q "RETRO-CREATE-CALLED" "$WORK/retro.log" || fail "retro-create not called"
git -C "$WORK/repo" worktree list --porcelain 2>/dev/null | grep -q "$WORK/wt-" \
    && fail "temp worktree still registered" || pass "temp worktree removed"

# --- C. Идемпотентность ------------------------------------------------------
echo "=== TEST C: idempotent re-run after healed drift -> exit 0, NO card ==="
setup_fixture
setup_hosts
rm -f "$WORK/retro.log"
OUT1="$(run_drift 2>&1)" || fail "first run failed"
echo "$OUT1" | grep -q "FIXED" || fail "first run expected FIXED"
[ ! -f "$WORK/retro.log" ] || fail "first run should not create card"

rm -f "$WORK/retro.log"
OUT2="$(run_drift 2>&1)"
RC=$?
echo "$OUT2" | sed 's/^/  /'
[ "$RC" = "0" ] || fail "re-run expected exit 0, got $RC"
[ ! -f "$WORK/retro.log" ] && pass "re-run: no card created" \
    || fail "re-run should NOT create card: $(cat "$WORK/retro.log")"

# md5 не изменился
for f in $FILES_LIST; do
    EXPECTED="$(origin_md5 "$f")"
    for h in a b c d; do
        CUR="$(md5sum "$WORK/hosts/$h/$f" 2>/dev/null | cut -c1-12)"
        [ "$CUR" = "$EXPECTED" ] || fail "after re-run hosts/$h/$f md5=$CUR != origin=$EXPECTED"
    done
done
pass "re-run: md5 unchanged, hosts still == origin/develop"

echo
echo "ALL TESTS PASSED"
