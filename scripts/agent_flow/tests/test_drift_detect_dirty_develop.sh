#!/bin/bash
# ============================================================================
# test_drift_detect_dirty_develop.sh — ретро 15.09 t_40611e65
#
# Регресс-гард для автофикса drift-detect при DIRTY_DEVELOP (главный worktree
# репо на develop, но дерево грязное — FF-merge невозможен).
#
# Проблема (ветка t_197de62a, 15.09 23:30): карточка t_197de62a смержила
# только в remote-ветку z-devops/t_197de62a-cancel-provider-exhausted-cron
# (коммит 4ed6e3ad), НЕ в develop. Те же изменения висели как uncommitted
# правки в основном worktree (MD5 совпадал с 4ed6e3ad 1:1). В результате:
#   - LOCAL_DESYNC: local develop=cadb8952 != origin/develop=8ccf3edb
#     (4 незамерженных коммита, включая нужные install.sh/watchdog-provider-quick)
#   - DRIFT: 14 файлов (включая install.sh и watchdog-provider-quick) на
#     хостах != origin/develop
#   - drift-detect вызывал try_ff_update, FF-merge падал из-за dirty worktree
#     ("Your local changes would be overwritten by merge"), скрипт писал
#     "WARN: continuing auto-fix with local tree as-is (may deploy stale scripts)"
#   - install.sh запускался на УСТАРЕВШЕМ локальном дереве → host-копии не
#     обновлялись → FIX FAILED → create_drift_card каждые 30 мин, 4 цикла
#     подряд (23:06 / 23:17 / 23:37 + see agent-flow-drift.alert.log).
#
# Решение: fallback на wt_origin_autofix — тот же механизм, что уже работал
# для BRANCH_ACTIVE (current branch != develop), но теперь применён и для
# DIRTY_DEVELOP (current branch == develop + dirty):
#   git worktree add --detach /tmp/wt-driftfix-$$-DIRTY_DEVELOP origin/develop
#   REPO_DIR=<wt> bash <wt>/scripts/agent_flow/install.sh
#   git worktree remove --force <wt>
# Карточка создаётся ТОЛЬКО если и этот путь не помог.
#
# Проверяем на фикстуре:
#   A. DIRTY_DEVELOP + healable drift -> exit 0, NO card, hosts == origin/develop,
#      uncommitted локальные правки СОХРАНЕНЫ (мы не убиваем дерево воркера),
#      temp worktree убран.
#   B. DIRTY_DEVELOP + unhealable drift (install.sh fails in worktree)
#      -> exit 1, card created.
#   C. Идемпотентность: повторный запуск после вылеченного дрейфа ->
#      exit 0, карточки НЕТ, md5 не изменился.
#
# Run:
#   bash scripts/agent_flow/tests/test_drift_detect_dirty_develop.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
DRIFT_SH="$TEST_DIR/../agent-flow-drift-detect.sh"
AGENT_FLOW_DIR="$(cd "$TEST_DIR/.." && pwd)"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -f "$DRIFT_SH" ] || fail "drift-detect script not found: $DRIFT_SH"

# --- фикстура: origin (bare) + рабочее репо с scripts/agent_flow ----------
setup_fixture() { # $1 = (optional) файл, отсутствующий в origin/develop
    local missing="${1:-}"
    rm -rf "$WORK/origin.git" "$WORK/repo"
    git init -q --bare "$WORK/origin.git" || fail "git init --bare"
    git init -q "$WORK/repo" || fail "git init repo"
    git -C "$WORK/repo" config user.email test@test
    git -C "$WORK/repo" config user.name test
    git -C "$WORK/repo" remote add origin "$WORK/origin.git"
    mkdir -p "$WORK/repo/scripts/agent_flow"
    cp -r "$AGENT_FLOW_DIR"/. "$WORK/repo/scripts/agent_flow/"
    if [ -n "$missing" ] && [ -f "$WORK/repo/scripts/agent_flow/$missing" ]; then
        rm -f "$WORK/repo/scripts/agent_flow/$missing"
    fi
    git -C "$WORK/repo" add -A
    git -C "$WORK/repo" commit -qm "fixture: agent_flow scripts"
    git -C "$WORK/repo" branch -M develop
    git -C "$WORK/repo" push -q -u origin develop || fail "push develop"
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

# DIRTY_DEVELOP setup: текущая ветка = develop, дерево грязное (модифицирован
# install.sh). Имитирует воркера, который правит скрипты локально, не коммитя.
# Дополнительно: пушим второй коммит в origin/develop, чтобы локальный develop
# отставал на 1 коммит (LOCAL_DESYNC).
setup_dirty_develop() {
    # 1) убедимся что мы на develop (setup_fixture мог оставить на feature-ветке)
    local cur_branch
    cur_branch="$(git -C "$WORK/repo" branch --show-current 2>/dev/null || true)"
    if [ -n "$cur_branch" ] && [ "$cur_branch" != "develop" ]; then
        git -C "$WORK/repo" checkout -q develop || fail "checkout develop"
    elif [ -z "$cur_branch" ]; then
        # detached HEAD — переключаемся на develop
        git -C "$WORK/repo" checkout -q develop || fail "checkout develop (detached)"
    fi

    # 2) push второй коммит на origin/develop (новый origin/develop)
    echo "# additional change" >> "$WORK/repo/scripts/agent_flow/install.sh"
    git -C "$WORK/repo" commit -qam "fixture: additional change on origin/develop"
    git -C "$WORK/repo" push -q origin develop || fail "push develop #2"

    # 3) reset --hard на предыдущий коммит (локальный develop отстаёт на 1)
    git -C "$WORK/repo" reset --hard -q HEAD~1 || fail "reset local develop"

    # 4) делаем "грязный" uncommitted diff поверх устаревшего HEAD:
    #    модифицируем install.sh (НЕ комитим) — имитация воркера, который
    #    что-то правит в основном worktree.
    echo "# worker WIP — НЕ commit" >> "$WORK/repo/scripts/agent_flow/install.sh"

    # Проверка фикстуры: грязный worktree + отстающий develop.
    local dirty
    dirty="$(git -C "$WORK/repo" status --porcelain)"
    [ -n "$dirty" ] || fail "fixture: expected dirty worktree, got clean"
    local local_h origin_h
    local_h="$(git -C "$WORK/repo" rev-parse HEAD)"
    origin_h="$(git -C "$WORK/repo" rev-parse origin/develop)"
    [ "$local_h" != "$origin_h" ] || fail "fixture: local develop must be behind origin/develop"
    pass "fixture: local=$local_h origin=$origin_h, dirty worktree confirmed"
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

# --- A. DIRTY_DEVELOP + healable drift --------------------------------------
echo "=== TEST A: DIRTY_DEVELOP + healable drift -> exit 0, NO card, hosts == origin/develop ==="
setup_fixture
setup_hosts
setup_dirty_develop
rm -f "$WORK/retro.log"
# Запомним "грязную" правку в worktree, чтобы после drift-detect проверить,
# что она СОХРАНЕНА (мы не убиваем дерево воркера).
WIP_LINE="# worker WIP — НЕ commit"
WIP_BEFORE="$(grep -c "$WIP_LINE" "$WORK/repo/scripts/agent_flow/install.sh" || true)"

OUT="$(run_drift 2>&1)"
RC=$?
echo "$OUT" | sed 's/^/  /'
[ "$RC" = "0" ] || fail "expected exit 0, got $RC"
echo "$OUT" | grep -q "DIRTY_DEVELOP" || fail "missing DIRTY_DEVELOP marker"
echo "$OUT" | grep -q "FIXED" || fail "expected FIXED marker, got: $OUT"
# КРИТИЧНО: должно быть "FIXED — drift resolved via origin/develop worktree",
# а НЕ "FIX FAILED — drift still present after install.sh".
echo "$OUT" | grep -q "FIXED — drift resolved via origin/develop worktree" \
    || fail "expected worktree-FIXED marker (not stale-install FIX FAILED)"
echo "$OUT" | grep -q "FIX FAILED" && fail "unexpected FIX FAILED in: $OUT"

# Карточка НЕ должна быть создана.
[ ! -f "$WORK/retro.log" ] && pass "no card created" \
    || fail "card should NOT be created: $(cat "$WORK/retro.log")"

# md5 всех EXPECTED файлов × 4 host-пути == origin/develop
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
git -C "$WORK/repo" worktree list --porcelain 2>/dev/null | grep -q "DIRTY_DEVELOP" \
    && fail "DIRTY_DEVELOP temp worktree still registered" || pass "DIRTY_DEVELOP temp worktree removed"

# Uncommitted правка в основном worktree СОХРАНЕНА (мы не делаем checkout/reset
# в основном worktree, фикс устанавливает host-копии через отдельный worktree).
WIP_AFTER="$(grep -c "$WIP_LINE" "$WORK/repo/scripts/agent_flow/install.sh" || true)"
[ "$WIP_AFTER" = "$WIP_BEFORE" ] && pass "worker WIP line preserved ($WIP_AFTER occurrences)" \
    || fail "worker WIP line lost: before=$WIP_BEFORE after=$WIP_AFTER"

# Локальный develop по-прежнему отстаёт от origin/develop (мы НЕ делали FF).
LOCAL_AFTER="$(git -C "$WORK/repo" rev-parse HEAD)"
ORIGIN_AFTER="$(git -C "$WORK/repo" rev-parse origin/develop)"
[ "$LOCAL_AFTER" != "$ORIGIN_AFTER" ] && pass "local develop NOT force-updated (worker tree safe)" \
    || fail "local develop should still be behind (LOCAL=$LOCAL_AFTER ORIGIN=$ORIGIN_AFTER)"

# --- B. DIRTY_DEVELOP + unhealable drift ------------------------------------
echo "=== TEST B: DIRTY_DEVELOP + unhealable drift -> card ==="
setup_fixture watchdog.sh   # origin/develop без watchdog.sh -> install.sh exit 2
setup_hosts
setup_dirty_develop
rm -f "$WORK/retro.log"
OUT="$(run_drift 2>&1)"
RC=$?
echo "$OUT" | sed 's/^/  /'
[ "$RC" = "1" ] || fail "expected exit 1 (FIX FAILED), got $RC"
echo "$OUT" | grep -q "DIRTY_DEVELOP" || fail "missing DIRTY_DEVELOP marker"
echo "$OUT" | grep -q "FIX FAILED" || fail "expected FIX FAILED marker"
[ -f "$WORK/retro.log" ] && pass "card created (FIX FAILED)" \
    || fail "card should be created for unhealable drift"

# --- C. идемпотентность: повторный запуск после вылеченного дрейфа ---------
# Сценарий: воркер НЕ делает uncommitted правок в основном worktree (это
# "хороший" кейс без DIRTY_DEVELOP). Первый прогон — host дрифтанут, но
# local develop свежий. После первого прогона install.sh разложит скрипты
# на хост и DRIFT=0. Второй прогон — тихий (exit 0, без карточки).
# Дополнительно проверяем: если run_drift оставляет LOCAL_DESYNC, второй
# прогон не должен создавать новую карточку.
echo "=== TEST C: idempotent re-run after healed drift -> exit 0, NO card ==="
setup_fixture
setup_hosts
# Первый прогон вылечит (local develop синхронен с origin/develop после
# setup_fixture+push, дерево чистое → try_ff_update rc=0 → DRIFT вылечен
# install.sh'ом из локального дерева).
rm -f "$WORK/retro.log"
OUT1="$(run_drift 2>&1)"
RC1=$?
[ "$RC1" = "0" ] || fail "first run expected exit 0, got $RC1: $OUT1"
echo "$OUT1" | grep -q "FIXED" || fail "first run expected FIXED marker"
[ ! -f "$WORK/retro.log" ] && pass "first run: no card" \
    || fail "first run: card should NOT be created"

# Второй прогон — должен быть тихим (нет дрейфа).
rm -f "$WORK/retro.log"
OUT2="$(run_drift 2>&1)"
RC2=$?
echo "$OUT2" | sed 's/^/  /'
[ "$RC2" = "0" ] || fail "expected exit 0 on re-run, got $RC2"
[ ! -f "$WORK/retro.log" ] && pass "re-run: no card created" \
    || fail "re-run: card should NOT be created: $(cat "$WORK/retro.log")"
# stdout должен быть ПУСТЫМ (тихий tick)
[ -z "$OUT2" ] && pass "re-run: silent (empty stdout)" \
    || fail "re-run: expected silent stdout, got: $OUT2"

echo
echo "ALL TESTS PASSED"