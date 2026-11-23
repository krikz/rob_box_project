#!/usr/bin/env bash
# Офлайн-тест окна паттернов с грейсом (issue #5030).
#
# Класс бага: шаг n701_sasha_back_recognized 21.11 покраснел, хотя робот
# узнал голос. check_patterns судил лог в момент проверки, а строка
# «Speaker: ...» пришла на 15.9 с позже. Повтор шага был запрещён гардом
# #2902 (регистрация уже принята) -- шаг не мог пройти ни первой попыткой,
# ни повтором.
#
# Контракт фикса:
#   1. строка паттерна появляется позже первого чтения окна, но её timestamp
#      в пределах шага + грейса -> шаг зелёный (PATTERN_OK, rc=0);
#   2. строка с timestamp позже верхней границы (момент проверки + грейс) ->
#      шаг красный (PATTERN_MISS, rc=1): гард #5002 сохранён, строки
#      следующего шага не судятся;
#   3. грейса нет -> прежнее поведение: судим один раз, не ждём;
#   4. поздняя строка несёт улику смены состояния (регистрация принята) ->
#      паттерн зелёный, но retry_block_reason() всё равно запрещает повтор
#      шага. Грейс не обходит гард #2902.
#
# Офлайн: source e2e_voice_lib.sh + check_patterns (вырезана awk'ом из
# e2e_voice_test.sh). ROBOT_SSH -- стаб. Строки лога с RFC3339-префиксом:
# fetch_step_window_logs режет их клиентским фильтром, когда «docker» не
# принял --until (rc стаба != 0). Без префикса фильтр отдаёт лог целиком,
# и тест врал бы, что граница работает.
# Запуск: bash scripts/testing/test_e2e_pattern_grace_window.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/scripts/ci/e2e/e2e_voice_lib.sh"
HARNESS="$REPO_ROOT/scripts/ci/e2e/e2e_voice_test.sh"
TOOL_MATCH="$REPO_ROOT/scripts/ci/e2e/e2e_tool_match.py"

[ -f "$LIB" ] && [ -f "$HARNESS" ] && [ -f "$TOOL_MATCH" ] \
    || { echo "FATAL: нет lib/harness/tool_match"; exit 1; }
# shellcheck disable=SC1090
source "$LIB" 2>/dev/null
type fetch_step_window_logs >/dev/null 2>&1 || { echo "FATAL: fetch_step_window_logs не определена"; exit 2; }

FN="$(awk '/^check_patterns\(\) \{/,/^\}/' "$HARNESS")"
[ -n "$FN" ] || { echo "FATAL: check_patterns не найдена в harness"; exit 3; }
eval "$FN"
type check_patterns >/dev/null 2>&1 || { echo "FATAL: check_patterns не определена"; exit 3; }

PASS=0
FAIL=0
ok()  { printf 'OK   %s\n' "$1"; PASS=$((PASS + 1)); }
bad() { printf 'FAIL %s\n' "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# Лог копится по тикам. Стаб на каждый вызов двигает тик и дописывает строку,
# назначенную на этот тик. docker logs «не принял» --until (rc=1), поэтому
# fetch_step_window_logs режет лог клиентским фильтром по timestamp.
: > "$WORKDIR/log.txt"
printf '0\n' > "$WORKDIR/tick"
log() { :; }

fake_ssh() {
    # Часы не двигают лог: check_patterns зовёт date один раз за границу.
    # «+N seconds» от базы 14:34:00Z -- то, что посчитал бы GNU date.
    case "$*" in
        *"+5 seconds"*) printf '2026-11-21T14:34:05.000000000Z\n'; return 0 ;;
        *"+1 seconds"*) printf '2026-11-21T14:34:01.000000000Z\n'; return 0 ;;
        date*|*"date "*) printf '2026-11-21T14:34:00.000000000Z\n'; return 0 ;;
    esac
    local tick
    tick="$(cat "$WORKDIR/tick")"
    if [ -f "$WORKDIR/arrive_at_${tick}" ]; then
        cat "$WORKDIR/arrive_at_${tick}" >> "$WORKDIR/log.txt"
    fi
    printf '%s\n' "$((tick + 1))" > "$WORKDIR/tick"
    # rc != 0: «docker» не принял --until, fetch режет клиентским фильтром.
    cat "$WORKDIR/log.txt"
    return 1
}
ROBOT_SSH="fake_ssh"
export ROBOT_SSH

SINCE="2026-11-21T14:34:00.000000000Z"
PAT="Speaker: 'Саша'"
# Тик 1 = 14:34:01Z. При грейсе 5 граница = 14:34:05Z: строка внутри окна.
LATE_LINE="2026-11-21T14:34:01.200000000Z [speaker_id_node-8] [INFO] Speaker: 'Саша' confidence=0.936 (533 ms)"
# Тик не важен: timestamp 14:34:09Z позже любой границы этого теста.
AFTER_LINE="2026-11-21T14:34:09.200000000Z [speaker_id_node-8] [INFO] Speaker: 'Саша' confidence=0.936 (533 ms)"

reset_log() {
    : > "$WORKDIR/log.txt"
    printf '0\n' > "$WORKDIR/tick"
    rm -f "$WORKDIR"/arrive_at_*
}

# 1. Строка приходит на втором чтении, её timestamp внутри грейса -> OK.
reset_log
printf '%s\n' "$LATE_LINE" > "$WORKDIR/arrive_at_1"
OUT="$(E2E_PATTERN_GRACE_SEC=5 E2E_PATTERN_GRACE_POLL=1 STEP_LABEL=n701 \
    E2E_PATTERN_UNTIL="$SINCE" check_patterns "$SINCE" "$PAT")"
RC=$?
{ [ "$RC" = 0 ] && printf '%s' "$OUT" | grep -q "PATTERN_OK: $PAT"; } \
    && ok "1: поздняя строка в пределах грейса -> OK" \
    || bad "1: ждали OK, rc=$RC out=$OUT"

# 2. Строка есть уже на первом чтении, но её timestamp позже границы грейса.
# Ждать её нечего: она не в окне. Гард #5002.
reset_log
printf '%s\n' "$AFTER_LINE" > "$WORKDIR/arrive_at_0"
OUT="$(E2E_PATTERN_GRACE_SEC=5 E2E_PATTERN_GRACE_POLL=1 STEP_LABEL=n701 \
    E2E_PATTERN_UNTIL="$SINCE" check_patterns "$SINCE" "$PAT")"
RC=$?
{ [ "$RC" = 1 ] && printf '%s' "$OUT" | grep -q "PATTERN_MISS: $PAT"; } \
    && ok "2: строка позже границы грейса -> MISS (гард #5002)" \
    || bad "2: ждали MISS, rc=$RC out=$OUT"

# 3. Грейса нет: строка, которая пришла бы на следующем чтении, не судится.
reset_log
printf '%s\n' "$LATE_LINE" > "$WORKDIR/arrive_at_1"
OUT="$(E2E_PATTERN_GRACE_SEC=0 STEP_LABEL=n701 \
    E2E_PATTERN_UNTIL="$SINCE" check_patterns "$SINCE" "$PAT")"
RC=$?
{ [ "$RC" = 1 ] && printf '%s' "$OUT" | grep -q "PATTERN_MISS: $PAT"; } \
    && ok "3: без грейса поздняя строка не судится" \
    || bad "3: ждали MISS, rc=$RC out=$OUT"

# 4. Гард #2902 не обходится грейсом. Поздняя строка несёт и паттерн, и улику
# принятой регистрации: паттерн зелёный, повтор шага всё равно запрещён.
reset_log
printf '%s\n%s\n' "$LATE_LINE" \
    "2026-11-21T14:34:01.400000000Z [speaker_id_node-8] [INFO] ✅ Speaker 'Саша' registered (id=abc)" \
    > "$WORKDIR/arrive_at_1"
OUT="$(E2E_PATTERN_GRACE_SEC=5 E2E_PATTERN_GRACE_POLL=1 STEP_LABEL=n701 \
    E2E_PATTERN_UNTIL="$SINCE" check_patterns "$SINCE" "$PAT")"
RC=$?
BLOCK="$(PYTHONPATH="$REPO_ROOT/scripts/ci/e2e${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -c '
import sys
from e2e_tool_match import retry_block_reason
sys.stdout.write(retry_block_reason(sys.stdin.read(), ""))
' < "$WORKDIR/log.txt")"
{ [ "$RC" = 0 ] && printf '%s' "$BLOCK" | grep -q "speaker registration accepted"; } \
    && ok "4: поздняя строка зелёная, ретрай всё равно запрещён (#2902)" \
    || bad "4: rc=$RC block=$BLOCK out=$OUT"

printf '\nPASS=%s FAIL=%s\n' "$PASS" "$FAIL"
exit "$FAIL"
