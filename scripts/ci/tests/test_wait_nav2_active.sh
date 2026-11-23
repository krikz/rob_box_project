#!/usr/bin/env bash
# ============================================================================
# test_wait_nav2_active.sh — регрессия issue #5114
#
# Ретро run 45019034598: L: Deploy and Verify закончился success, пока
# controller_server уже умер `[ros2run]: Illegal instruction`. Оба замера
# статуса показали health=starting внутри start_period, а код healthcheck
# nav2 (lifecycle трёх узлов == active) воркфлоу не спрашивал.
#
# scripts/ci/wait_nav2_active.sh — только про nav2 на Main Pi:
#   * вне старт-окна читает последний Health.Log[].ExitCode и отдаёт ok=false
#     при rc≠0 (и при отсутствии контейнера / отсутствии проб);
#   * внутри окна (check_container_status.sh вернул pending_start с nav2)
#     ждёт конца окна и мерит снова — не пишет «жив» раньше вердикта;
#   * лечение #5039 — ручной рубильник NAV2_RESTART_ON_FAIL, по умолчанию 0
#     (решение Шифу 23.11, #5149): без него гейт не рестартует и честно
#     краснеет. При =1 — ровно один `docker restart nav2`, и только после
#     второй подтверждающей пробы при healthy zenoh-router. Таймаут (-1)
#     рестарт не лечит. После рестарта старая запись Health.Log не принимается.
#
# docker/sshpass мокаются через PATH. stdout скрипта — одна JSON-строка
# {ok, reason}; exit 0 всегда, шаг воркфлоу валит джобу по .ok.
#
# Run:
#   bash scripts/ci/tests/test_wait_nav2_active.sh
# ============================================================================
set -euo pipefail

SCRIPT_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR_HERE/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REPO_ROOT/scripts/ci/wait_nav2_active.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"

[ -f "$SCRIPT_UNDER_TEST" ] || { echo "FATAL: $SCRIPT_UNDER_TEST не найден" >&2; exit 1; }
[ -f "$WORKFLOW" ] || { echo "FATAL: $WORKFLOW не найден" >&2; exit 1; }
command -v jq >/dev/null 2>&1 || { echo "FATAL: jq не найден в PATH" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "FATAL: python3 не найден в PATH" >&2; exit 1; }

MOCK_BIN="$(mktemp -d)"
trap 'rm -rf "$MOCK_BIN"' EXIT

# Mock sshpass. По удалённой команде выбирает фикстуру:
#   * `docker restart nav2` — пишет метку в $FIXTURE_DIR/.restarts, ничего не печатает;
#   * `zenoh-router`        — отдаёт router_N.txt (состояние healthcheck роутера);
#   * всё остальное (nav2)  — отдаёт sample_N.txt (сырой docker inspect).
# Счётчики раздельные: перезапуск не съедает замер, а проба роутера — нет.
cat >"$MOCK_BIN/sshpass" <<'SSHPASS_EOF'
#!/usr/bin/env bash
case "$*" in
    *"docker restart nav2"*)
        printf 'restart\n' >> "$FIXTURE_DIR/.restarts"
        exit 0
        ;;
    *zenoh-router*)
        calls_file="$FIXTURE_DIR/.router_calls"
        n="$( [ -f "$calls_file" ] && cat "$calls_file" || echo 0 )"
        n=$((n + 1))
        echo "$n" > "$calls_file"
        f="$FIXTURE_DIR/router_${n}.txt"
        [ -f "$f" ] && cat "$f"
        exit 0
        ;;
esac
calls_file="$FIXTURE_DIR/.calls"
n="$( [ -f "$calls_file" ] && cat "$calls_file" || echo 0 )"
n=$((n + 1))
echo "$n" > "$calls_file"
sample_file="$FIXTURE_DIR/sample_${n}.txt"
if [ -f "$sample_file" ]; then
    cat "$sample_file"
fi
SSHPASS_EOF
chmod +x "$MOCK_BIN/sshpass"

TOTAL=0
FAILED=0
if [ -t 1 ]; then
    RED=$'\033[31m'; GREEN=$'\033[32m'; NC=$'\033[0m'
else
    RED=""; GREEN=""; NC=""
fi

assert_eq() {  # $1=actual $2=expected $3=label
    TOTAL=$((TOTAL + 1))
    if [ "$1" = "$2" ]; then
        printf '  %s✓%s %s = %s\n' "$GREEN" "$NC" "$3" "$1"
    else
        printf '  %s✗%s %s: expected %s, got %s\n' "$RED" "$NC" "$3" "$2" "$1" >&2
        FAILED=$((FAILED + 1))
    fi
}

# Сырой docker inspect одного контейнера nav2.
# $1=health $2=exit_code $3=started_at $4=start_period_ns $5=n_log_entries $6=probe_start
# n_log_entries=0 — проб ещё не было: ExitCode должен читаться как -1, а не 0.
# probe_start пуст — Start пробы = StartedAt контейнера (проба свежая для него).
inspect_nav2() {
    local health="$1" exit_code="$2" started_at="$3" period_ns="$4" n_log="${5:-1}" probe_start="${6:-}"
    if [ -z "$probe_start" ]; then
        probe_start="$started_at"
    fi
    jq -nc \
        --arg health "$health" \
        --argjson exit_code "$exit_code" \
        --arg started_at "$started_at" \
        --arg probe_start "$probe_start" \
        --argjson period_ns "$period_ns" \
        --argjson n_log "$n_log" \
        '{
            Name: "/nav2",
            State: {
                Status: "running",
                StartedAt: $started_at,
                Health: {
                    Status: $health,
                    Log: ([range($n_log) | {ExitCode: $exit_code, Start: $probe_start, Output: "probe"}])
                }
            },
            Config: {Healthcheck: {StartPeriod: $period_ns}}
        }'
}

assert_contains() {  # $1=haystack $2=needle $3=label
    TOTAL=$((TOTAL + 1))
    if printf '%s\n' "$1" | grep -qF -- "$2"; then
        printf '  %s✓%s %s\n' "$GREEN" "$NC" "$3"
    else
        printf '  %s✗%s %s: не найдено %s\n' "$RED" "$NC" "$3" "$2" >&2
        FAILED=$((FAILED + 1))
    fi
}

# Дополнительные фикстуры и env для следующего вызова run_case.
#   ROUTER_FIXTURES — содержимое router_1, router_2, ... (состояние роутера);
#   EXTRA_ENV       — пары VAR=VALUE поверх базового env (например, рубильник).
ROUTER_FIXTURES=()
EXTRA_ENV=()
LAST_STDERR=""
LAST_RESULT=""
LAST_RESTARTS=0

# run_case EXPECT_OK STATUS_JSON FIXTURES... -- LABEL
#   FIXTURES — содержимое sample_1, sample_2, ... (по одному аргументу).
#   "--" отделяет фикстуры от метки.
run_case() {
    local expect_ok="$1" status_json="$2"
    shift 2
    local fixtures=()
    while [ "$1" != "--" ]; do
        fixtures+=("$1")
        shift
    done
    shift
    local label="$1"

    local fixture_dir
    fixture_dir="$(mktemp -d)"
    local i=1
    for fix in "${fixtures[@]}"; do
        printf '%s\n' "$fix" > "$fixture_dir/sample_${i}.txt"
        i=$((i + 1))
    done
    i=1
    for fix in "${ROUTER_FIXTURES[@]}"; do
        printf '%s\n' "$fix" > "$fixture_dir/router_${i}.txt"
        i=$((i + 1))
    done
    ROUTER_FIXTURES=()

    local env_block=(
        "PATH=$MOCK_BIN:$PATH"
        "PI_HOST=10.1.1.10"
        "PI_USER=ros2"
        "PI_PASSWORD=open"
        "PI_COMPOSE_DIR=/tmp"
        "PI_SSH_OPTS=-o StrictHostKeyChecking=no"
        "FIXTURE_DIR=$fixture_dir"
        "STATUS_JSON=$status_json"
        # В тестах окно уже позади или замер сразу после конца: запас не нужен,
        # а реальные 25s раздули бы прогон.
        "PROBE_SLACK_S=0"
        "MAX_WAIT_S=180"
        "${EXTRA_ENV[@]}"
    )
    EXTRA_ENV=()

    local stderr_file="$fixture_dir/.stderr"
    local stdout
    stdout="$(env -i "${env_block[@]}" bash "$SCRIPT_UNDER_TEST" 2>"$stderr_file" || true)"
    LAST_STDERR="$(cat "$stderr_file" 2>/dev/null || true)"
    if [ -f "$fixture_dir/.restarts" ]; then
        LAST_RESTARTS="$(wc -l < "$fixture_dir/.restarts" | tr -d ' ')"
    else
        LAST_RESTARTS=0
    fi
    rm -rf "$fixture_dir"

    LAST_RESULT="$(printf '%s\n' "$stdout" | jq -c 'select(.ok != null)' 2>/dev/null | tail -n 1 || true)"
    printf '  [%s]\n' "$label"
    if [ -z "$LAST_RESULT" ]; then
        printf '  %s✗%s no JSON result, stdout: %s\n' "$RED" "$NC" "$stdout" >&2
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
        return
    fi
    assert_eq "$(printf '%s' "$LAST_RESULT" | jq -r '.ok')" "$expect_ok" "ok"
}

NOW_EPOCH="$(date +%s)"
OLD_STARTED="$(date -u -d "@$((NOW_EPOCH - 120))" +%Y-%m-%dT%H:%M:%SZ)"
YOUNG_STARTED="$(date -u -d "@$((NOW_EPOCH - 5))" +%Y-%m-%dT%H:%M:%SZ)"
NAV2_START_PERIOD_NS=90000000000  # 90s, как у nav2

NO_PENDING='{"healthy":true,"failed_count":0,"pending_start":false,"pending":[]}'
PENDING_NAV2="$(jq -nc --arg started_at "$YOUNG_STARTED" --argjson period_ns "$NAV2_START_PERIOD_NS" \
    '{healthy:false, failed_count:0, pending_start:true, pending:[{name:"nav2", started_at:$started_at, start_period_ns:$period_ns}]}')"

# A: вне окна, healthcheck rc=1 — провал. Тот самый ретро: SIGILL не даёт
# узлам стать active, healthcheck.sh возвращает 1, а деплой был зелёным.
FIX="$(inspect_nav2 starting 1 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
run_case false "$NO_PENDING" "$FIX" -- \
    "A: outside window, ExitCode=1 → ok=false (issue #5114)"

# B: вне окна, rc=0 — жив. Гейт не должен красить здоровый nav2.
FIX="$(inspect_nav2 healthy 0 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
run_case true "$NO_PENDING" "$FIX" -- \
    "B: outside window, ExitCode=0 → ok=true"

# C: вне окна проб ещё не было. «Не знаем» не равно «жив»: ExitCode читается
# как -1, а не как 0 (пустой Health.Log не должен давать ложный успех).
FIX="$(inspect_nav2 starting 0 "$OLD_STARTED" "$NAV2_START_PERIOD_NS" 0)"
run_case false "$NO_PENDING" "$FIX" -- \
    "C: outside window, no healthcheck probes yet → ok=false (unknown is not alive)"

# D: контейнера нет — провал, а не тишина.
run_case false "$NO_PENDING" "" -- \
    "D: nav2 container absent → ok=false"

# E: внутри окна ждём конца и мерим снова. Первый замер ещё starting rc=1
# (окно не кончилось — вердикта нет), второй уже вне окна с rc=1.
# PROBE_SLACK_S=0, возраст 5s при окне 90s: скрипт спит ~85s. Это и есть
# «дождаться конца окна», поэтому кейс не ускоряем до нуля.
FIX1="$(inspect_nav2 starting 1 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
FIX2="$(inspect_nav2 unhealthy 1 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
run_case false "$PENDING_NAV2" "$FIX1" "$FIX2" -- \
    "E: inside window, wait, re-measure ExitCode=1 → ok=false (issue #5114)"

# F: то же ожидание, но повторный замер уже rc=0 — после окна nav2 жив,
# гейт не валит деплой за сам факт ожидания.
FIX2="$(inspect_nav2 healthy 0 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
run_case true "$PENDING_NAV2" "$FIX1" "$FIX2" -- \
    "F: inside window, wait, re-measure ExitCode=0 → ok=true"

# G: pending_start есть, но не про nav2 (другой контейнер ещё стартует).
# nav2 вне окна с rc=0 — гейт его не держит и не валит.
OTHER_PENDING='{"healthy":false,"failed_count":0,"pending_start":true,"pending":[{"name":"perception","started_at":"x","start_period_ns":1}]}'
FIX="$(inspect_nav2 healthy 0 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
run_case true "$OTHER_PENDING" "$FIX" -- \
    "G: pending is another container, nav2 ExitCode=0 → ok=true (gate is nav2-only)"

# --- лечение #5039: один перезапуск nav2 при живом роутере --------------------
# Рубильник по умолчанию выключен (решение Шифу 23.11, #5149). H/J включают
# его явно. Рестарт — только после второй подтверждающей пробы: одна красная
# healthy контейнер не трогает.

# H: NAV2_RESTART_ON_FAIL=1, две пробы rc=1, роутер healthy → ровно один
# restart, новый замер rc=0 → ok=true. Первая проба рестарта не даёт.
FIX1="$(inspect_nav2 starting 1 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
FIX2="$(inspect_nav2 healthy 0 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0")
ROUTER_FIXTURES=("healthy" "healthy")
run_case true "$NO_PENDING" "$FIX1" "$FIX1" "$FIX2" -- \
    "H: NAV2_RESTART_ON_FAIL=1, ExitCode=1 confirmed + healthy router → 1 restart, re-measure 0 → ok=true (#5039)"
assert_eq "$LAST_RESTARTS" "1" "restart count (H)"
assert_contains "$LAST_STDERR" "рестарта ещё нет" \
    "first red probe waits, no restart yet (H)"
assert_contains "$LAST_STDERR" "nav2 healthcheck ExitCode=1 — однократный restart nav2" \
    "log line about the one-shot restart (H)"

# I: rc=1, но роутер ещё не healthy → перезапуск не лечит причину: fail, 0 restart.
ROUTER_FIXTURES=("starting")
run_case false "$NO_PENDING" "$FIX1" -- \
    "I: ExitCode=1 + router starting → no restart, ok=false (#5039)"
assert_eq "$LAST_RESTARTS" "0" "restart count (I)"

# J: NAV2_RESTART_ON_FAIL=1, две пробы rc=1, после перезапуска снова rc=1 →
# fail и БОЛЬШЕ НИ ОДНОГО перезапуска (второго не бывает: повтор — это уже
# цикл, а не лечение). Подтверждение после рестарта рестарт не повторяет.
FIX3="$(inspect_nav2 unhealthy 1 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0")
ROUTER_FIXTURES=("healthy" "healthy" "healthy")
run_case false "$NO_PENDING" "$FIX1" "$FIX1" "$FIX3" "$FIX3" -- \
    "J: NAV2_RESTART_ON_FAIL=1, ExitCode=1 confirmed → 1 restart, re-measure 1 → ok=false, no second restart (#5039)"
assert_eq "$LAST_RESTARTS" "1" "restart count (J)"

# K: rc=-1 (healthcheck ни разу не отработал вне окна) — это не «упал после
# старта», перезапуск такое не лечит: fail без restart даже при живом роутере.
FIX_NOPROBE="$(inspect_nav2 starting 0 "$OLD_STARTED" "$NAV2_START_PERIOD_NS" 0)"
ROUTER_FIXTURES=("healthy")
run_case false "$NO_PENDING" "$FIX_NOPROBE" -- \
    "K: ExitCode=-1 (no probes) + healthy router → no restart, ok=false (#5039)"
assert_eq "$LAST_RESTARTS" "0" "restart count (K)"

# L: дефолт (переменная не задана) — лечение выключено. Даже при healthy
# роутере и rc=1 гейт не рестартует и честно краснеет с причиной.
# Явный NAV2_RESTART_ON_FAIL=0 — тот же исход (рубильник выключен руками).
ROUTER_FIXTURES=("healthy")
run_case false "$NO_PENDING" "$FIX1" -- \
    "L: default (NAV2_RESTART_ON_FAIL unset) + ExitCode=1 + healthy router → 0 restarts, ok=false (#5149)"
assert_eq "$LAST_RESTARTS" "0" "restart count (L, default)"
assert_contains "$LAST_STDERR" "ExitCode=1" "log names ExitCode (L)"
assert_contains "$LAST_RESULT" "ExitCode=1" "JSON reason names ExitCode (L)"
assert_contains "$LAST_RESULT" "controller_server" "JSON reason names the nodes the probe wants (L)"
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=0")
ROUTER_FIXTURES=("healthy")
run_case false "$NO_PENDING" "$FIX1" -- \
    "L0: NAV2_RESTART_ON_FAIL=0 explicit → no restart, ok=false (#5039)"
assert_eq "$LAST_RESTARTS" "0" "restart count (L0)"

# M: после перезапуска контейнер снова в start_period — гейт ждёт НОВОЕ окно
# (короткое, 10s) и решает по третьему замеру, а не по старому.
SHORT_PERIOD_NS=10000000000  # 10s
# Свежая метка: к моменту этого кейса прогон уже потратил ~3 минуты на E/F,
# и YOUNG_STARTED успел бы выйти из короткого окна.
M_YOUNG="$(date -u -d "@$(( $(date +%s) - 2 ))" +%Y-%m-%dT%H:%M:%SZ)"
FIX_NEW_WINDOW="$(inspect_nav2 starting 1 "$M_YOUNG" "$SHORT_PERIOD_NS")"
FIX_REMEASURED="$(inspect_nav2 healthy 0 "$OLD_STARTED" "$SHORT_PERIOD_NS")"
# Рубильник явным =1 и две подтверждающие пробы: иначе дефолт 0 рестарта не даст,
# а одно красное окно рестарт не включает (#5149).
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0")
ROUTER_FIXTURES=("healthy" "healthy")
run_case true "$NO_PENDING" "$FIX1" "$FIX1" "$FIX_NEW_WINDOW" "$FIX_REMEASURED" -- \
    "M: NAV2_RESTART_ON_FAIL=1, after restart new start_period → wait and decide on the new measurement (#5039)"
assert_eq "$LAST_RESTARTS" "1" "restart count (M)"

# --- #5149: дефолт выключен, подтверждение, свежая проба ---------------------
# N: =1 и одна красная проба, второй замер rc=0. Рестарта нет: одиночный
# провал (таймаут пробы, разовый сбой) healthy контейнер не трогает.
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0")
ROUTER_FIXTURES=("healthy")
run_case true "$NO_PENDING" "$FIX1" "$FIX2" -- \
    "N: NAV2_RESTART_ON_FAIL=1 + one red probe then ExitCode=0 → 0 restarts, ok=true (#5149)"
assert_eq "$LAST_RESTARTS" "0" "restart count (N)"
assert_contains "$LAST_STDERR" "рестарта ещё нет" "confirm wait logged, restart not taken (N)"

# O: =1 и таймаут ExitCode=-1 (проб не было) — рестарт такое не лечит.
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0")
ROUTER_FIXTURES=("healthy")
run_case false "$NO_PENDING" "$FIX_NOPROBE" -- \
    "O: NAV2_RESTART_ON_FAIL=1 + ExitCode=-1 → 0 restarts, ok=false (#5149)"
assert_eq "$LAST_RESTARTS" "0" "restart count (O)"

# P: =1, провал подтверждён, после рестарта Health.Log ещё старый (Start
# раньше нового StartedAt, StartPeriod=0). Гейт старую запись не принимает,
# ждёт свежую и зеленеет только по ней. Рестарт ровно один.
STALE_PROBE="$(date -u -d "@$(( $(date +%s) - 300 ))" +%Y-%m-%dT%H:%M:%SZ)"
P_STARTED="$(date -u -d "@$(( $(date +%s) - 1 ))" +%Y-%m-%dT%H:%M:%SZ)"
FIX_STALE="$(inspect_nav2 unhealthy 1 "$P_STARTED" 0 1 "$STALE_PROBE")"
FIX_FRESH="$(inspect_nav2 healthy 0 "$P_STARTED" 0 1 "$P_STARTED")"
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0")
ROUTER_FIXTURES=("healthy" "healthy")
run_case true "$NO_PENDING" "$FIX1" "$FIX1" "$FIX_STALE" "$FIX_FRESH" -- \
    "P: after restart stale Health.Log ignored, ok only on a fresh probe (#5149)"
assert_eq "$LAST_RESTARTS" "1" "restart count (P)"
assert_contains "$LAST_STDERR" "older than StartedAt" "stale probe waited, not accepted (P)"

# Q: =1, провал подтверждён (MAX_WAIT_S хватает на confirm), после рестарта
# RESTART_WAIT_S=0 — свежую пробу ждать нечем. Старая запись Health.Log не
# принимается. Честный fail, рестарт ровно один.
EXTRA_ENV=("NAV2_RESTART_ON_FAIL=1" "CONFIRM_WAIT_S=0" "RESTART_WAIT_S=0")
ROUTER_FIXTURES=("healthy" "healthy")
run_case false "$NO_PENDING" "$FIX1" "$FIX1" "$FIX_STALE" "$FIX_STALE" -- \
    "Q: after restart no fresh probe and budget exhausted → ok=false, 1 restart (#5149)"
assert_eq "$LAST_RESTARTS" "1" "restart count (Q)"
assert_contains "$LAST_STDERR" "no fresh healthcheck probe after restart" "budget fail names missing fresh probe (Q)"

# --- таблица решения: nav2_action без робота ---------------------------------
# Чистая функция решения скармливается напрямую (source и вызов), без ssh.
# shellcheck disable=SC1090
source "$SCRIPT_UNDER_TEST"

assert_eq "$(nav2_action 0 false unknown false)" "ok" "decision: rc=0 → ok"
assert_eq "$(nav2_action 0 true unknown false)" "ok" "decision: rc=0 inside window → ok"
assert_eq "$(nav2_action 1 true healthy false)" "wait" "decision: rc=1 inside window → wait"
# Рубильник в source-скрипте по умолчанию 0: одна красная проба — fail, не restart.
assert_eq "$(nav2_action 1 false healthy false)" "fail" "decision: default is fail, not restart (#5149)"
assert_eq "$(nav2_action 1 false starting false)" "fail" "decision: rc=1 + router not healthy → fail"
assert_eq "$(nav2_action 1 false unknown false)" "fail" "decision: rc=1 + router unknown → fail"
assert_eq "$(nav2_action 1 false healthy true)" "fail" "decision: rc=1 after one restart → fail (never twice)"
assert_eq "$(nav2_action -1 false healthy false)" "fail" "decision: no probes → fail, no restart"
# shellcheck disable=SC2034  # переменную читает nav2_action из source-скрипта
NAV2_RESTART_ON_FAIL=1
assert_eq "$(nav2_action 1 false healthy false false)" "confirm" \
    "decision: =1 + one red probe → confirm, not restart (#5149)"
assert_eq "$(nav2_action 1 false healthy false true)" "restart" \
    "decision: =1 + confirmed failure + healthy router → restart"
assert_eq "$(nav2_action -1 false healthy false true)" "fail" \
    "decision: =1 + ExitCode=-1 → fail, timeout is not cured by restart"
assert_eq "$(nav2_action 1 false healthy true true)" "fail" \
    "decision: =1 after one restart → fail (never twice)"
# shellcheck disable=SC2034  # nav2_action читает переменную из окружения source
NAV2_RESTART_ON_FAIL=0
assert_eq "$(nav2_action 1 false healthy false true)" "fail" \
    "decision: NAV2_RESTART_ON_FAIL=0 kills the cure even when confirmed"

# --- контракт воркфлоу -------------------------------------------------------
# Шаг Main Pi обязан звать wait_nav2_active.sh и валить джобу по .ok.
# Ветка issues в Summary обязана делать exit 1: до #5114 она только писала
# deploy_status=issues, и джоба оставалась зелёной.

python3 - "$WORKFLOW" <<'PY'
import sys
from pathlib import Path

import yaml

workflow = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
steps = [s for job in workflow["jobs"].values() for s in job.get("steps", [])]

def step(name):
    found = [s for s in steps if s.get("name") == name]
    assert len(found) == 1, name
    return found[0]["run"]

main = step("[Main Pi] Check Container Status")
assert "scripts/ci/wait_nav2_active.sh" in main, "Main Pi step does not call wait_nav2_active.sh"
assert "STATUS_JSON" in main, "Main Pi step does not pass STATUS_JSON to the nav2 gate"
assert "NAV2_RESTART_ON_FAIL" in main, "Main Pi step does not pass NAV2_RESTART_ON_FAIL"
assert "inputs.nav2_restart_on_fail" in main, "Main Pi step does not read the dispatch input"
assert "exit 1" in main, "Main Pi step never exits 1: a dead nav2 would stay green"
# Приёмка #5118: гейт ok=true обязан писать main_healthy=true. Иначе false из
# статуса (pending внутри окна) висит до Summary и красит зелёный деплой.
ok_arm = main.split('".ok"', 1)[-1]
assert 'main_healthy=true' in ok_arm, "nav2 gate never writes main_healthy=true after ok"
# Гейт — только про nav2. Vision/Sound шаги его не зовут.
for other in ("[Vision Pi] Check Container Status", "[Sound Pi] Check Container Status"):
    assert "wait_nav2_active.sh" not in step(other), other

summary = step("Deployment Summary")
issues_branch = summary.split("deploy_status=issues", 1)
assert len(issues_branch) == 2, "deploy_status=issues branch missing"
assert "exit 1" in issues_branch[1].split("fi", 1)[0], (
    "issues branch writes deploy_status but does not exit 1"
)
# Успех и отмена не должны краснеть из-за нового exit 1.
for marker in ("deploy_status=success", "deploy_status=cancelled", "deploy_status=hil_locked"):
    branch = summary.split(marker, 1)[1].split("elif", 1)[0].split("else", 1)[0]
    assert "exit 1" not in branch, marker
inputs = workflow[True]["workflow_dispatch"]["inputs"]
flag = inputs.get("nav2_restart_on_fail")
assert flag is not None, "workflow_dispatch has no nav2_restart_on_fail input"
assert flag.get("type") == "boolean", flag
assert flag.get("default") is False, flag
print("  workflow contract: Main Pi gate + nav2_restart_on_fail input + issues exit 1 ok")
PY

echo
if [ "$FAILED" -eq 0 ]; then
    printf '%sPASS%s: %d/%d assertions ok\n' "$GREEN" "$NC" "$TOTAL" "$TOTAL"
    exit 0
else
    printf '%sFAIL%s: %d/%d assertions failed\n' "$RED" "$NC" "$FAILED" "$TOTAL"
    exit 1
fi
