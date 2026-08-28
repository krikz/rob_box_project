#!/bin/bash
# ============================================================================
# test_watchdog_wt_sweep.sh — ретро 14.08 t_ee70ffc2
#
# Регресс-гард для sweep_stale_local_worktrees() в watchdog.sh: локальные
# worktree от завершённых карточек (/tmp/wt-*, /home/builder/wt-*) должны
# чиститься, если ветки нет в origin и каталог старше 48ч. Guard: ветка в
# origin → keep; develop/main/master → keep; осиротевшие (gitdir нет) → rm -rf.
#
# Проверяем на фикстуре из реального git-репо:
#   1. worktree на ветке, удалённой из origin  -> REMOVED (git worktree remove)
#   2. worktree на ветке, живущей в origin     -> keep
#   3. worktree на develop                     -> keep
#   4. detached worktree (ветки нет)           -> REMOVED
#   5. осиротевший каталог (gitdir отсутствует) -> REMOVED_ORPHAN (rm -rf)
#
# Run:
#   bash scripts/agent_flow/tests/test_watchdog_wt_sweep.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../watchdog.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

log() { echo "[test-log] $*" >&2; }

# --- извлекаем sweep_stale_local_worktrees() из watchdog.sh ------------------
# Функция начинается с `sweep_stale_local_worktrees() {` на 0-й колонке,
# закрывающая `}` тоже на 0-й колонке (вложенные блоки с отступом).
awk '/^sweep_stale_local_worktrees\(\) \{/{f=1} f{print} f && /^\}/{exit}' \
    "$WATCHDOG_SH" > "$WORK/fn.sh"
[ -s "$WORK/fn.sh" ] || fail "sweep_stale_local_worktrees() not found in $WATCHDOG_SH"

# --- фикстура: origin (bare) + рабочее репо + worktree ----------------------
git init -q --bare "$WORK/origin.git" || fail "git init --bare"
git init -q "$WORK/repo" || fail "git init repo"
git -C "$WORK/repo" config user.email test@test
git -C "$WORK/repo" config user.name test
git -C "$WORK/repo" remote add origin "$WORK/origin.git"
echo base > "$WORK/repo/base.txt"
git -C "$WORK/repo" add base.txt
git -C "$WORK/repo" commit -qm base
git -C "$WORK/repo" branch -M develop
git -C "$WORK/repo" push -q -u origin develop || fail "push develop"
git -C "$WORK/repo" branch alive
git -C "$WORK/repo" branch stale
git -C "$WORK/repo" push -q -u origin alive || fail "push alive"
git -C "$WORK/repo" push -q -u origin stale || fail "push stale"

# главное репо уводим с develop, чтобы ветку develop можно было отдать worktree
git -C "$WORK/repo" checkout -qb fixture-main

git -C "$WORK/repo" worktree add -q "$WORK/wt-alive" alive || fail "worktree alive"
git -C "$WORK/repo" worktree add -q "$WORK/wt-stale" stale || fail "worktree stale"
git -C "$WORK/repo" worktree add -q "$WORK/wt-develop" develop || fail "worktree develop"
git -C "$WORK/repo" worktree add -q --detach "$WORK/wt-detached" develop || fail "worktree detached"

# ветку stale удаляем из origin (имитация MERGED PR → branch deleted)
git -C "$WORK/repo" push -q origin --delete stale || fail "delete stale from origin"

# осиротевший каталог: .git-файл указывает на несуществующий gitdir
mkdir -p "$WORK/wt-orphan"
echo "gitdir: $WORK/repo/.git/worktrees/wt-orphan" > "$WORK/wt-orphan/.git"

# каталог без .git-файла (не наш worktree) — трогать НЕ должен
mkdir -p "$WORK/wt-nogit"
echo "not a worktree" > "$WORK/wt-nogit/readme.txt"

# старим все кандидаты (> 48ч), чтобы возрастной guard пропустил
touch -d '3 days ago' "$WORK"/wt-* 2>/dev/null || true

# --- запускаем функцию с тестовым окружением ---------------------------------
cat > "$WORK/run.sh" <<RUN
#!/bin/bash
set -u
export HERMES_HOME="$WORK/hermes"
export REPO_DIR="$WORK/repo"
export WT_SWEEP_AGE_HOURS=48
export WT_SWEEP_COOLDOWN_SEC=0
export WT_SWEEP_STATE="$WORK/hermes/state/agent-flow-wt-sweep.last"
export WT_SWEEP_LOG="$WORK/sweep.log"
export WT_SWEEP_PATTERNS="$WORK/wt-*"
export WT_SWEEP_LOCKS=""
log() { echo "[test-log] \$*" >&2; }
. "$WORK/fn.sh"
sweep_stale_local_worktrees
RUN
chmod +x "$WORK/run.sh"
bash "$WORK/run.sh" >/dev/null 2>&1 || fail "sweep_stale_local_worktrees exited non-zero"

# --- ассерты ------------------------------------------------------------------
[ ! -e "$WORK/wt-stale" ] && pass "wt-stale removed (branch deleted from origin)" \
    || fail "wt-stale should be removed (branch 'stale' not in origin)"
[ -d "$WORK/wt-alive" ] && pass "wt-alive kept (branch 'alive' in origin)" \
    || fail "wt-alive should be kept (branch 'alive' in origin)"
[ -d "$WORK/wt-develop" ] && pass "wt-develop kept (develop)" \
    || fail "wt-develop should be kept (develop)"
[ ! -e "$WORK/wt-detached" ] && pass "wt-detached removed (no branch in origin)" \
    || fail "wt-detached should be removed (detached HEAD, no branch)"
[ ! -e "$WORK/wt-orphan" ] && pass "wt-orphan removed (gitdir missing → rm -rf)" \
    || fail "wt-orphan should be removed (gitdir missing)"
[ -d "$WORK/wt-nogit" ] && pass "wt-nogit kept (no .git file → not our worktree)" \
    || fail "wt-nogit should be kept (no .git file)"

# регистрация worktree в репо не должна содержать удалённых
for gone in wt-stale wt-detached; do
    if git -C "$WORK/repo" worktree list --porcelain 2>/dev/null | grep -q "worktree $WORK/$gone"; then
        fail "$gone still registered in git worktree list"
    fi
done
pass "git worktree list clean of removed paths"

# лог sweep содержит записи
grep -q "REMOVED" "$WORK/sweep.log" || fail "sweep.log missing REMOVED entries"
pass "sweep.log written ($(wc -l < "$WORK/sweep.log") lines)"

echo "ALL TESTS PASSED"
