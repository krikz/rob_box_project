#!/bin/bash
# ============================================================================
# test_e2e_process_robot_busy_gate.sh — гейт G3.5 в agent-flow-e2e-process.sh.
#
# Что проверяем и зачем
# ---------------------
# Робот один: ротация (cron every 20m) и ночной голосовой марафон
# (docs/e2e/night-voice-marathon.md, ~4 часа) играют команды в один динамик.
# Развязка — sentinel $HERMES_HOME/state/robot-busy.
#
# Цена ошибки в обе стороны разная, поэтому тестируем обе:
#   - гейт не сработал → два e2e слышат команды друг друга, оба прогона
#     превращаются в мусор («✅ ПОЛНЫЙ ЦИКЛ + PATTERN_MISS»);
#   - гейт залип   → ротация стоит сутками, а мы узнаём об этом из тишины
#     в канбане. Именно поэтому у sentinel ДВА независимых срока годности.
#
# Сценарии:
#   RB1. нет sentinel                       → гейт пропускает тик
#   RB2. свежий sentinel, end в будущем     → skip (exit 0), «robot BUSY»
#   RB3. end в прошлом                      → sentinel удалён, тик идёт
#   RB4. mtime старше ROBOT_BUSY_MAX_AGE    → sentinel удалён, тик идёт
#   RB5. кривой sentinel (`touch robot-busy`) → skip, но не дольше MAX_AGE
#        (ручной «руки прочь от робота» — легальный способ применения)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_robot_busy_gate.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_SH="${SCRIPT_SH:-$TEST_DIR/../agent-flow-e2e-process.sh}"

[ -f "$SCRIPT_SH" ] || { echo "FAIL: $SCRIPT_SH not found"; exit 1; }

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Вырезаем секцию G3.5 и гоняем её в изоляции: полный скрипт по дороге сюда
# ходит в gh API и MAINTENANCE-гейт, которых в юните быть не должно.
GATE="$WORK/g35.sh"
{
    echo 'log() { printf "%s\n" "$*" >&2; }'
    awk '
        /^# --- G3\.5: robot-busy sentinel/ {flag=1}
        flag {
            if (/^# --- required env/) { exit }
            print
        }
    ' "$SCRIPT_SH"
    echo 'echo "GATE_PASSED"'
} > "$GATE"

grep -q 'ROBOT_BUSY_SENTINEL' "$GATE" || {
    echo "FAIL: не удалось вырезать секцию G3.5 из $SCRIPT_SH"
    exit 1
}

run_gate() {  # $1=sentinel_path ; печатает stdout+stderr
    bash -c "
        set -u
        export HERMES_HOME='$WORK'
        export ROBOT_BUSY_SENTINEL='$1'
        source '$GATE'
    " 2>&1
}

check() {  # $1=name $2=haystack $3=needle $4=expect(present|absent)
    local name="$1" hay="$2" needle="$3" mode="${4:-present}"
    if [ "$mode" = "present" ]; then
        if printf '%s' "$hay" | grep -qF "$needle"; then
            echo "  ✅ $name"; PASS=$((PASS + 1)); return 0
        fi
    else
        if ! printf '%s' "$hay" | grep -qF "$needle"; then
            echo "  ✅ $name"; PASS=$((PASS + 1)); return 0
        fi
    fi
    echo "  ❌ $name"
    printf '     ожидалось %s: %s\n     вывод: %s\n' "$mode" "$needle" "$hay"
    FAIL=$((FAIL + 1))
}

now="$(date +%s)"

# --- RB1 --------------------------------------------------------------------
echo "=== RB1: sentinel отсутствует — тик идёт ==="
S="$WORK/rb1"
out="$(run_gate "$S")"
check "гейт пропустил тик" "$out" "GATE_PASSED"

# --- RB2 --------------------------------------------------------------------
echo "=== RB2: свежий sentinel, expected_end в будущем — skip ==="
S="$WORK/rb2"
printf 'night-marathon %s %s marathon\n' "$now" "$((now + 3600))" > "$S"
out="$(run_gate "$S")"
check "тик пропущен"           "$out" "GATE_PASSED" absent
check "причина в логе"         "$out" "robot BUSY"
check "владелец назван"        "$out" "night-marathon"
[ -f "$S" ] && { echo "  ✅ sentinel не тронут"; PASS=$((PASS + 1)); } \
            || { echo "  ❌ sentinel удалён, хотя он живой"; FAIL=$((FAIL + 1)); }

# --- RB3 --------------------------------------------------------------------
echo "=== RB3: expected_end в прошлом — sentinel снят, тик идёт ==="
S="$WORK/rb3"
printf 'night-marathon %s %s marathon\n' "$((now - 7200))" "$((now - 60))" > "$S"
out="$(run_gate "$S")"
check "тик продолжен"          "$out" "GATE_PASSED"
check "лог про просрочку"      "$out" "просрочен"
[ ! -f "$S" ] && { echo "  ✅ протухший sentinel удалён"; PASS=$((PASS + 1)); } \
              || { echo "  ❌ sentinel остался — ротация встанет навсегда"; FAIL=$((FAIL + 1)); }

# --- RB4 --------------------------------------------------------------------
# Владельца убили -9, trap не отработал, expected_end остался в будущем.
# Спасает только жёсткий потолок по mtime.
echo "=== RB4: mtime старше ROBOT_BUSY_MAX_AGE — sentinel снят, тик идёт ==="
S="$WORK/rb4"
printf 'night-marathon %s %s marathon\n' "$((now - 90000))" "$((now + 36000))" > "$S"
touch -d '@'"$((now - 86400))" "$S" 2>/dev/null || touch -t 200001010000 "$S"
out="$(run_gate "$S")"
check "тик продолжен"          "$out" "GATE_PASSED"
check "лог про протухание"     "$out" "протух"
[ ! -f "$S" ] && { echo "  ✅ sentinel удалён по возрасту"; PASS=$((PASS + 1)); } \
              || { echo "  ❌ sentinel пережил MAX_AGE — вечная заморозка"; FAIL=$((FAIL + 1)); }

# --- RB5 --------------------------------------------------------------------
# `touch $HERMES_HOME/state/robot-busy` — задокументированный ручной способ
# сказать «робот занят, руки прочь». Полей нет, end=0 → уважаем до MAX_AGE.
echo "=== RB5: кривой/пустой sentinel — skip до MAX_AGE ==="
S="$WORK/rb5"
: > "$S"
out="$(run_gate "$S")"
check "тик пропущен"           "$out" "GATE_PASSED" absent
check "причина в логе"         "$out" "robot BUSY"

echo
echo "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" = "0" ] || exit 1
