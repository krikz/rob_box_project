#!/bin/bash
# ============================================================================
# test_drift_detect_install_sh.sh — ретро 14.09 t_06956919
#
# Регресс-тест на chicken-egg: install.sh раньше НЕ был в EXPECTED, поэтому
# drift-detect его не проверял. После фикса (install.sh первый в EXPECTED,
# t_06956919) drift-detect должен:
#   - ловить md5+size mismatch install.sh между origin/develop и host-копиями;
#   - в dry-run — exit 1 + alert в логе с явным упоминанием install.sh;
#   - в автофикс-режиме — лечить (install.sh self-replace через EXPECTED);
#   - идемпотентно: повторный прогон — exit 0, без карточки.
#
# Проверяем на фикстуре из реального git-репо + 4 host-папки (как в
# test_drift_detect_branch_active.sh):
#   A. install.sh origin == hosts (всё чистое) -> exit 0, no card, alert
#      пустой (no_op).
#   B. install.sh origin != hosts (имитируем 5 хостов из t_06956919, у
#      которых старая версия 1b767e5) -> в dry-run: exit 1, alert содержит
#      "DRIFT" + "install.sh"; в автофикс-режиме: exit 0, "FIXED", хосты
#      == origin/develop, install.sh на хостах = SOT.
#   C. Идемпотентность: после вылеченного дрейфа повторный прогон в
#      автофикс-режиме — exit 0, no card, md5 не изменился.
#
# Run:
#   bash scripts/agent_flow/tests/test_drift_detect_install_sh.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
DRIFT_SH="$TEST_DIR/../agent-flow-drift-detect.sh"
AGENT_FLOW_DIR="$(cd "$TEST_DIR/.." && pwd)"

WORK="$(mktemp -d -t drift-install-sh-XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -f "$DRIFT_SH" ] || fail "drift-detect script not found: $DRIFT_SH"

# --- фикстура: bare origin + рабочее репо с scripts/agent_flow --------------
# Копируем install.sh и ВСЕ файлы из EXPECTED (через `bash install.sh
# --list-files`) — это эталон SOT, на котором мы имитируем "origin/develop".
setup_fixture() {
    rm -rf "$WORK/origin.git" "$WORK/repo"
    git init -q --bare "$WORK/origin.git" || fail "git init --bare"
    git init -q "$WORK/repo" || fail "git init repo"
    git -C "$WORK/repo" config user.email test@test
    git -C "$WORK/repo" config user.name test
    git -C "$WORK/repo" remote add origin "$WORK/origin.git"
    mkdir -p "$WORK/repo/scripts/agent_flow"
    # Копируем все файлы из реального agent_flow (install.sh + ~47 .sh +
    # conf + py). Drift-detect фильтрует `case "$f" in *.sh`, но install.sh
    # sanity check требует, чтобы ВСЕ EXPECTED-файлы были на месте.
    cp -r "$AGENT_FLOW_DIR"/. "$WORK/repo/scripts/agent_flow/"
    # sanity: install.sh на месте (это и есть наш SOT install.sh)
    [ -f "$WORK/repo/scripts/agent_flow/install.sh" ] || fail "fixture: install.sh missing"
    git -C "$WORK/repo" add -A
    git -C "$WORK/repo" commit -qm "fixture: agent_flow scripts (incl. install.sh)"
    git -C "$WORK/repo" branch -M develop
    git -C "$WORK/repo" push -q -u origin develop || fail "push develop"
}

# 4 fixture host-директории. По умолчанию host-папки получают РЕАЛЬНЫЕ файлы
# из SOT (= no drift). Если передан $1 = путь к stale-install.sh, то в
# каждой host-папке install.sh заменяется на этот стабак (имитация 5 хостов
# с устаревшей версией install.sh из t_06956919).
setup_hosts() {
    local stale_install="${1:-}"
    rm -rf "$WORK/hosts"
    mkdir -p "$WORK/hosts/a" "$WORK/hosts/b" "$WORK/hosts/c" "$WORK/hosts/d"
    for h in a b c d; do
        # Берём ВСЮ SOT-папку как есть (host-папки должны быть зеркалом
        # SOT по умолчанию, чтобы no-drift прогон не падал).
        cp -r "$AGENT_FLOW_DIR"/. "$WORK/hosts/$h/"
        if [ -n "$stale_install" ]; then
            cp "$stale_install" "$WORK/hosts/$h/install.sh"
        fi
    done
}

# Фейковый kanban-retro-create.sh: логирует вызов, ничего не создаёт
cat > "$WORK/fake-retro-create.sh" <<'FAKE'
#!/bin/bash
echo "RETRO-CREATE-CALLED $*" >> "${RETRO_JOURNAL:-/dev/null}"
exit 0
FAKE
chmod +x "$WORK/fake-retro-create.sh"

# Фейковый hermes: intercept ensure_*_cron вызовов install.sh
mkdir -p "$WORK/bin"
cat > "$WORK/bin/hermes" <<'FAKE'
#!/bin/bash
echo "HERMES-CALLED $*" >> "${HERMES_JOURNAL:-/dev/null}"
exit 0
FAKE
chmod +x "$WORK/bin/hermes"

# run_drift_dryrun — прогон drift-detect в dry-run (без автофикса)
run_drift_dryrun() {
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
    DRIFT_DRY_RUN=1 \
    bash "$DRIFT_SH"
    return $?
}

# run_drift_autofix — прогон drift-detect с автофиксом (по умолчанию)
run_drift_autofix() {
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

# ============================================================
# A. Чисто: install.sh на origin == hosts -> exit 0, no card.
# ============================================================
echo "=== TEST A: install.sh in sync -> exit 0, no card, no alert ==="
setup_fixture
setup_hosts                              # без stale_install — хосты == SOT
rm -f "$WORK/alert.log" "$WORK/retro.log"
OUT="$(run_drift_autofix 2>&1)"
RC=$?
echo "$OUT" | sed 's/^/  /'
[ "$RC" = "0" ] || fail "expected exit 0 (clean), got $RC"
echo "$OUT" | grep -q "BRANCH_ACTIVE" && fail "should NOT be BRANCH_ACTIVE on clean tree" || true
# На чистом прогоне drift-detect печатает только в BRANCH_ACTIVE; иначе
# тихий tick (exit 0, stdout пустой). Допустим любой не-алерт в stdout.
[ ! -s "$WORK/alert.log" ] && pass "alert log empty (clean tick)" \
    || fail "alert log should be empty on clean tick: $(cat "$WORK/alert.log")"
[ ! -f "$WORK/retro.log" ] && pass "no card created" \
    || fail "card should NOT be created: $(cat "$WORK/retro.log")"

# ============================================================
# B. install.sh устарел на хостах -> dry-run exit 1 + alert; autofix exit 0.
# ============================================================
echo
echo "=== TEST B: install.sh drifted (5 hosts with old version) -> dry-run fails, autofix heals ==="

# Имитация "старой версии 1b767e5": создаём файл-стабак ровно с тем
# содержимым, что был до фикса (без install.sh в EXPECTED). Для теста
# достаточно: другой md5/size чем у SOT.
STALE_FILE="$WORK/stale_install.sh"
{
    echo "#!/bin/bash"
    echo "# stale: pre-t_06956919 install.sh — без install.sh в EXPECTED"
    echo "EXPECTED=( agent-flow-triage.sh )"
    echo "if [ \"\${1:-}\" = \"--list-files\" ]; then printf '%s\\n' \"\${EXPECTED[@]}\"; exit 0; fi"
    echo "exit 0"
} > "$STALE_FILE"
chmod +x "$STALE_FILE"
# md5 стабака не должен совпадать с SOT (чтобы был реальный DRIFT)
STALE_MD5="$(md5sum "$STALE_FILE" | cut -c1-12)"
SOT_MD5="$(origin_md5 install.sh)"
[ -n "$STALE_MD5" ] && [ -n "$SOT_MD5" ] || fail "fixture: cannot compute md5"
[ "$STALE_MD5" != "$SOT_MD5" ] || fail "fixture: stale md5 unexpectedly equals SOT"

setup_fixture
setup_hosts "$STALE_FILE"
rm -f "$WORK/alert.log" "$WORK/retro.log"

# --- B1. dry-run: exit 1, alert содержит DRIFT и install.sh ------------
OUT_DRY="$(run_drift_dryrun 2>&1)"
RC_DRY=$?
echo "$OUT_DRY" | sed 's/^/  [dry] /'
[ "$RC_DRY" = "1" ] || fail "dry-run expected exit 1 (DRIFT), got $RC_DRY"
# alert log должен содержать install.sh + DRIFT + размер/md5
grep -q "DRIFT" "$WORK/alert.log" || fail "alert.log missing DRIFT marker"
grep -q "install\.sh" "$WORK/alert.log" || fail "alert.log missing install.sh mention: $(cat "$WORK/alert.log")"
pass "dry-run: exit 1 + alert mentions install.sh + DRIFT"
[ ! -f "$WORK/retro.log" ] && pass "dry-run: no retro card created" \
    || fail "dry-run: should NOT create retro card: $(cat "$WORK/retro.log")"

# dry-run не должен был лечить: хосты всё ещё на старой версии
STILL_STALE=0
for h in a b c d; do
    CUR="$(md5sum "$WORK/hosts/$h/install.sh" 2>/dev/null | cut -c1-12)"
    [ "$CUR" = "$SOT_MD5" ] || STILL_STALE=1
done
[ "$STILL_STALE" = "1" ] && pass "dry-run: hosts unchanged (still stale)" \
    || fail "dry-run: hosts got fixed (autofix should NOT run)"

# --- B2. autofix: exit 0, FIXED, install.sh на хостах == SOT ------------
rm -f "$WORK/alert.log" "$WORK/retro.log"
OUT_FIX="$(run_drift_autofix 2>&1)"
RC_FIX=$?
echo "$OUT_FIX" | sed 's/^/  [fix] /'
[ "$RC_FIX" = "0" ] || fail "autofix expected exit 0, got $RC_FIX"
echo "$OUT_FIX" | grep -q "FIXED" || fail "autofix: expected FIXED marker"
grep -q "install\.sh" "$WORK/alert.log" || fail "autofix alert.log missing install.sh mention"
pass "autofix: exit 0 + FIXED + alert mentions install.sh"

# md5 install.sh на всех хостах должен стать == SOT
HEALED=0
for h in a b c d; do
    CUR="$(md5sum "$WORK/hosts/$h/install.sh" 2>/dev/null | cut -c1-12)"
    [ "$CUR" = "$SOT_MD5" ] || { echo "  host $h/install.sh md5=$CUR != SOT=$SOT_MD5"; HEALED=1; }
done
[ "$HEALED" = "0" ] && pass "autofix: install.sh on all 4 hosts == origin/develop SOT" \
    || fail "autofix: install.sh not healed on all hosts"

# md5 остальных файлов на всех хостах == origin/develop SOT (install.sh
# само-раскладывание не сломало остальные EXPECTED)
FILES_LIST="$(bash "$AGENT_FLOW_DIR/install.sh" --list-files)"
ALL_OK=1
for f in $FILES_LIST; do
    EXP="$(origin_md5 "$f")"
    for h in a b c d; do
        CUR="$(md5sum "$WORK/hosts/$h/$f" 2>/dev/null | cut -c1-12)"
        [ "$CUR" = "$EXP" ] || { ALL_OK=0; echo "  $f on $h md5=$CUR != origin=$EXP"; }
    done
done
[ "$ALL_OK" = "1" ] && pass "autofix: all EXPECTED files (incl. install.sh) in sync on 4 hosts" \
    || fail "autofix: some EXPECTED files still drifted"

[ ! -f "$WORK/retro.log" ] && pass "autofix: no retro card created" \
    || fail "autofix: should NOT create card on heal: $(cat "$WORK/retro.log")"

# ============================================================
# C. Идемпотентность: повторный прогон после вылеченного дрейфа.
# ============================================================
echo
echo "=== TEST C: idempotent re-run after heal -> exit 0, no card ==="
rm -f "$WORK/alert.log" "$WORK/retro.log"
OUT2="$(run_drift_autofix 2>&1)"
RC2=$?
echo "$OUT2" | sed 's/^/  /'
[ "$RC2" = "0" ] || fail "re-run expected exit 0, got $RC2"
[ ! -f "$WORK/retro.log" ] && pass "re-run: no card created" \
    || fail "re-run: should NOT create card: $(cat "$WORK/retro.log")"
# md5 не изменился
ALL_OK=1
for f in $FILES_LIST; do
    EXP="$(origin_md5 "$f")"
    for h in a b c d; do
        CUR="$(md5sum "$WORK/hosts/$h/$f" 2>/dev/null | cut -c1-12)"
        [ "$CUR" = "$EXP" ] || { ALL_OK=0; echo "  $f on $h md5=$CUR != origin=$EXP"; }
    done
done
[ "$ALL_OK" = "1" ] && pass "re-run: md5 all EXPECTED files unchanged" \
    || fail "re-run: some files drifted after heal"

echo
echo "ALL TESTS PASSED"
