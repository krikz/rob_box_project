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
#     ждёт конца окна и мерит снова — не пишет «жив» раньше вердикта.
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

# Mock sshpass: каждый вызов отдаёт следующую фикстуру из $FIXTURE_DIR
# (sample_1.txt, sample_2.txt, ...). Пустой файл = контейнера нет.
cat >"$MOCK_BIN/sshpass" <<'SSHPASS_EOF'
#!/usr/bin/env bash
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
# $1=health $2=exit_code $3=started_at $4=start_period_ns $5=n_log_entries
# n_log_entries=0 — проб ещё не было: ExitCode должен читаться как -1, а не 0.
inspect_nav2() {
    local health="$1" exit_code="$2" started_at="$3" period_ns="$4" n_log="${5:-1}"
    jq -nc \
        --arg health "$health" \
        --argjson exit_code "$exit_code" \
        --arg started_at "$started_at" \
        --argjson period_ns "$period_ns" \
        --argjson n_log "$n_log" \
        '{
            Name: "/nav2",
            State: {
                Status: "running",
                StartedAt: $started_at,
                Health: {
                    Status: $health,
                    Log: ([range($n_log) | {ExitCode: $exit_code, Output: "probe"}])
                }
            },
            Config: {Healthcheck: {StartPeriod: $period_ns}}
        }'
}

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
    )

    local stdout
    stdout="$(env -i "${env_block[@]}" bash "$SCRIPT_UNDER_TEST" 2>/dev/null || true)"
    rm -rf "$fixture_dir"

    local result
    result="$(printf '%s\n' "$stdout" | jq -c 'select(.ok != null)' 2>/dev/null | tail -n 1 || true)"
    printf '  [%s]\n' "$label"
    if [ -z "$result" ]; then
        printf '  %s✗%s no JSON result, stdout: %s\n' "$RED" "$NC" "$stdout" >&2
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
        return
    fi
    assert_eq "$(printf '%s' "$result" | jq -r '.ok')" "$expect_ok" "ok"
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
print("  workflow contract: Main Pi gate + issues exit 1 ok")
PY

echo
if [ "$FAILED" -eq 0 ]; then
    printf '%sPASS%s: %d/%d assertions ok\n' "$GREEN" "$NC" "$TOTAL" "$TOTAL"
    exit 0
else
    printf '%sFAIL%s: %d/%d assertions failed\n' "$RED" "$NC" "$FAILED" "$TOTAL"
    exit 1
fi
