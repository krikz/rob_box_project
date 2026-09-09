#!/bin/bash
# ============================================================================
# test_watchdog_project_wt_sweep.sh — ретро 14.08 t_d007c365
#
# Регресс-гард для sweep_stale_project_worktrees() в watchdog.sh: проектные
# worktree .worktrees/<task_id> от archived/done карточек должны чиститься,
# даже если ветка карточки ещё живёт в origin. Критерий — статус в kanban DB:
#   archived|done  -> REMOVED (git worktree remove --force + prune)
#   running/ready  -> keep
#   карточки нет в DB -> keep
#   каталог без .git-файла -> не наш worktree, keep
#   осиротевший каталог (gitdir отсутствует) -> REMOVED_ORPHAN (rm -rf)
#   каталог не t_* (например .worktrees/other) -> keep
#
# Проверяем на фикстуре из реального git-репо + kanban DB (sqlite3).
#
# Run:
#   bash scripts/agent_flow/tests/test_watchdog_project_wt_sweep.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../watchdog.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

log() { echo "[test-log] $*" >&2; }

# --- извлекаем sweep_stale_project_worktrees() из watchdog.sh ---------------
awk '/^sweep_stale_project_worktrees\(\) \{/{f=1} f{print} f && /^\}/{exit}' \
    "$WATCHDOG_SH" > "$WORK/fn.sh"
[ -s "$WORK/fn.sh" ] || fail "sweep_stale_project_worktrees() not found in $WATCHDOG_SH"

# --- фикстура: origin (bare) + рабочее репо + worktree в .worktrees/ ---------
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

# ветки карточек: archived и done ПУШИМ в origin (главный кейс ретро — ветка
# жива, но карточка archived/done; старый sweep по «ветки нет в origin» их
# НЕ чистил). running/ready-ветки тоже пушим (они живые и карточки не done).
for b in t_archived-branch t_done-branch t_running-branch t_ready-branch t_notindb-branch; do
    git -C "$WORK/repo" branch "$b"
    git -C "$WORK/repo" push -q -u origin "$b" || fail "push $b"
done

# главное репо уводим с develop, чтобы ветку develop можно было отдать worktree
git -C "$WORK/repo" checkout -qb fixture-main

mkdir -p "$WORK/repo/.worktrees"
git -C "$WORK/repo" worktree add -q "$WORK/repo/.worktrees/t_archived01" t_archived-branch || fail "wt archived"
git -C "$WORK/repo" worktree add -q "$WORK/repo/.worktrees/t_done0001" t_done-branch || fail "wt done"
git -C "$WORK/repo" worktree add -q "$WORK/repo/.worktrees/t_running01" t_running-branch || fail "wt running"
git -C "$WORK/repo" worktree add -q "$WORK/repo/.worktrees/t_ready0001" t_ready-branch || fail "wt ready"
git -C "$WORK/repo" worktree add -q "$WORK/repo/.worktrees/t_notindb01" t_notindb-branch || fail "wt notindb"

# каталог не t_* внутри .worktrees/ — трогать НЕ должен
mkdir -p "$WORK/repo/.worktrees/other"
echo "not a card" > "$WORK/repo/.worktrees/other/readme.txt"

# осиротевший каталог t_*: .git-файл указывает на несуществующий gitdir
mkdir -p "$WORK/repo/.worktrees/t_orphan001"
echo "gitdir: $WORK/repo/.git/worktrees/t_orphan001" > "$WORK/repo/.worktrees/t_orphan001/.git"

# каталог t_* без .git-файла (не наш worktree) — трогать НЕ должен
mkdir -p "$WORK/repo/.worktrees/t_nogit001"
echo "not a worktree" > "$WORK/repo/.worktrees/t_nogit001/readme.txt"

# --- kanban DB (sqlite3): статусы карточек -----------------------------------
python3 - "$WORK/kanban.db" <<'PYEOF'
import sqlite3, sys
con = sqlite3.connect(sys.argv[1])
con.execute("CREATE TABLE tasks (id TEXT PRIMARY KEY, title TEXT, status TEXT)")
con.executemany(
    "INSERT INTO tasks (id, title, status) VALUES (?, ?, ?)",
    [
        ("t_archived01", "archived card", "archived"),
        ("t_done0001", "done card", "done"),
        ("t_running01", "running card", "running"),
        ("t_ready0001", "ready card", "ready"),
        # t_notindb01 намеренно отсутствует
    ],
)
con.commit()
con.close()
PYEOF
[ -f "$WORK/kanban.db" ] || fail "kanban DB fixture not created"

# --- запускаем функцию с тестовым окружением ---------------------------------
cat > "$WORK/run.sh" <<RUN
#!/bin/bash
set -u
export HERMES_HOME="$WORK/hermes"
export KANBAN_BOARDS_DIR="$WORK/boards"
export REPO_DIR="$WORK/repo"
export WT_SWEEP_COOLDOWN_SEC=0
export WT_SWEEP_LOCKS=""
export WT_PROJECT_WT_DIR="$WORK/repo/.worktrees"
export WT_PROJECT_SWEEP_KANBAN_DB="$WORK/kanban.db"
export WT_PROJECT_SWEEP_STATE="$WORK/hermes/state/agent-flow-project-wt-sweep.last"
export WT_PROJECT_SWEEP_LOG="$WORK/sweep.log"
export WT_PROJECT_SWEEP_REMOVE_STATUSES="archived done"
log() { echo "[test-log] \$*" >&2; }
. "$WORK/fn.sh"
sweep_stale_project_worktrees
RUN
chmod +x "$WORK/run.sh"
bash "$WORK/run.sh" >/dev/null 2>&1 || fail "sweep_stale_project_worktrees exited non-zero"

# --- ассерты ------------------------------------------------------------------
[ ! -e "$WORK/repo/.worktrees/t_archived01" ] && pass "t_archived01 removed (card archived, branch in origin)" \
    || fail "t_archived01 should be removed (card archived)"
[ ! -e "$WORK/repo/.worktrees/t_done0001" ] && pass "t_done0001 removed (card done, branch in origin)" \
    || fail "t_done0001 should be removed (card done)"
[ -d "$WORK/repo/.worktrees/t_running01" ] && pass "t_running01 kept (card running)" \
    || fail "t_running01 should be kept (card running)"
[ -d "$WORK/repo/.worktrees/t_ready0001" ] && pass "t_ready0001 kept (card ready)" \
    || fail "t_ready0001 should be kept (card ready)"
[ -d "$WORK/repo/.worktrees/t_notindb01" ] && pass "t_notindb01 kept (card not in DB)" \
    || fail "t_notindb01 should be kept (card not in DB)"
[ -d "$WORK/repo/.worktrees/other" ] && pass "other kept (not t_*)" \
    || fail "other should be kept (not t_*)"
[ ! -e "$WORK/repo/.worktrees/t_orphan001" ] && pass "t_orphan001 removed (gitdir missing → rm -rf)" \
    || fail "t_orphan001 should be removed (gitdir missing)"
[ -d "$WORK/repo/.worktrees/t_nogit001" ] && pass "t_nogit001 kept (no .git file → not our worktree)" \
    || fail "t_nogit001 should be kept (no .git file)"

# регистрация worktree в репо не должна содержать удалённых
for gone in t_archived01 t_done0001; do
    if git -C "$WORK/repo" worktree list --porcelain 2>/dev/null | grep -q "worktree $WORK/repo/.worktrees/$gone"; then
        fail "$gone still registered in git worktree list"
    fi
done
pass "git worktree list clean of removed paths"

# лог sweep содержит записи
grep -q "REMOVED" "$WORK/sweep.log" || fail "sweep.log missing REMOVED entries"
pass "sweep.log written ($(wc -l < "$WORK/sweep.log") lines)"

echo "ALL TESTS PASSED"
