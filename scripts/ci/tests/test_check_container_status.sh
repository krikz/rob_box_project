#!/usr/bin/env bash
# ============================================================================
# test_check_container_status.sh — регрессионный тест (issue #2122)
#
# Источник истины: <repo>/scripts/ci/tests/test_check_container_status.sh
#
# Тестирует <repo>/scripts/ci/check_container_status.sh (вынесенная из
# workflow L-Deploy and Verify.yml логика «[Vision Pi] Check Container Status»
# + «[Main Pi] Check Container Status»). Каждый кейс запускает СКРИПТ с
# мокнутыми `sshpass`/`docker` через PATH, читает HEALTHY_RESULT/FAILED_COUNT
# и сверяет с ожиданием.
#
# Зачем: ретро run #34153246013 (2026-09-07) — deploy отрапортовал success
# при контейнере rob-box-quest в бесконечном «Restarting (1)» loop,
# потому что старая проверка брала ОДИН снимок `docker compose ps` сразу
# после sleep 30 и сравнивала .State != "running". Контейнер под
# restart-policy в активном crash-loop ЛЕГИТИМНО показывает running
# на доли секунды между падениями — единичный неудачно попавший замер
# читает «здоров». Тем самым весь deploy-verify pipeline потерял
# сигнал «робот лежит».
#
# Этот тест фиксирует поведение фикса (двухзамерная проверка через
# `docker inspect`):
#   1. running в обоих замерах                                → healthy=true
#   2. restarting в обоих замерах                              → healthy=false
#      (тот самый баг #2122; единичный snapshot наоборот сказал бы ОК)
#   3. exited с ExitCode=0 (one-shot init)                    → healthy=true
#      (НЕ валить здоровое состояние — ретро 7f8939d28)
#   4. exited с ExitCode=1 (one-shot сломался)                → healthy=false
#   5. Health.Status=unhealthy в любом замере                 → healthy=false
#   6. running в обоих, но RestartCount вырос (та самая гонка) → healthy=false
#
# Run:
#   bash scripts/ci/tests/test_check_container_status.sh
#   make -C scripts/ci/tests test_check_container_status
# ============================================================================
set -euo pipefail

# --- paths -----------------------------------------------------------------
SCRIPT_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR_HERE/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REPO_ROOT/scripts/ci/check_container_status.sh"

# Проверяем что мы запущены из репо (а не из снапшота).
[ -f "$SCRIPT_UNDER_TEST" ] || {
    echo "FATAL: $SCRIPT_UNDER_TEST не найден. Запускай из корня репо." >&2
    exit 1
}

# Проверяем что в $PATH есть jq. Workflow полагается на него, и тест
# делает то же.
command -v jq >/dev/null 2>&1 || {
    echo "FATAL: jq не найден в PATH. Установи: apt-get install jq" >&2
    exit 1
}

# Подменяем PATH на изолированный bin с моками sshpass + docker.
MOCK_BIN="$(mktemp -d)"
trap 'rm -rf "$MOCK_BIN"' EXIT

# --- mock sshpass -----------------------------------------------------------
# Возвращает фикстуру JSON-инспекции (один или несколько контейнеров) на
# каждый вызов. fixture-index.txt в $MOCK_BIN содержит номер текущего
# замера (1 или 2) — после каждого замера инкрементируется.
cat >"$MOCK_BIN/sshpass" <<'SSHPASS_EOF'
#!/usr/bin/env bash
# Mock sshpass: возвращает фикстуру, имитируя «docker compose ps -aq» +
# «docker inspect ...» внутри ssh-команды на Pi. Нам важен не SSH-коннект,
# а то, что логика скрипта вызывает ровно ДВА раза (sample #1 + sample #2)
# с CONFIRM_INTERVAL секунд между ними. Mock считает вызовы в файле
# $FIXTURE_DIR/.calls (FIFO-подобный счётчик) и возвращает соответствующую
# фикстуру.
calls_file="$FIXTURE_DIR/.calls"
n="$( [ -f "$calls_file" ] && cat "$calls_file" || echo 0 )"
n=$((n + 1))
echo "$n" > "$calls_file"
# Замеры 1 и 2 — пара двухзамерного подтверждения. Замер 3 и дальше —
# перемер после ожидания конца start_period (issue #5114, приёмка #5118):
# sample_N.txt, если есть, иначе последний заданный замер.
sample_file="$FIXTURE_DIR/sample_${n}.txt"
if [ ! -f "$sample_file" ]; then
    # Последний заданный замер, не всегда sample_2: перемер (n>=3) без своей
    # фикстуры повторяет то, что уже измерили, а не замер #2.
    sample_file="$(ls -1 "$FIXTURE_DIR"/sample_*.txt 2>/dev/null | sort -V | tail -n 1)"
fi
if [ -f "$sample_file" ]; then
    cat "$sample_file"
else
    echo ""
fi
SSHPASS_EOF
chmod +x "$MOCK_BIN/sshpass"

# --- mock docker (compose) --------------------------------------------------
# Должен вернуть список cid'ов (через `docker compose ps -aq`). Наш
# скрипт гоняет «docker compose ps -aq» в ssh-команде, поэтому mock-sshpass
# должен эмулировать это. Для простоты mock-sshpass выдаёт полный JSON
# (как будто `docker compose ps -aq` вернул ID и `docker inspect` уже
# сработал). Эмулировать `ps -aq` отдельно не нужно, т.к. цикл
# `for cid in $(docker compose ps -aq); do docker inspect ...; done`
# целиком происходит внутри ssh-команды; на стороне ssh-mock мы
# возвращаем уже плоский JSON-Lines.
#
# Тем не менее mock-sshpass возвращает содержимое для ОБОИХ замеров
# подряд? Нет — тест-раннер переключает FIXTURE_INDEX между замерами.
# См. run_case() ниже.

# --- test runner ------------------------------------------------------------

TOTAL=0
FAILED=0

# Цвета только если подключены к TTY.
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

# JSON-обёртка: docker inspect возвращает «сырой» JSON одного контейнера
# (как при вызове `docker inspect --format '{{json .}}' CONTAINER`). Наш
# скрипт получает эти строки через ssh и сам проецирует их через jq
# (name/status/health/restarts/exitcode). Поэтому фикстура должна
# содержать ПОЛНЫЙ JSON, а не плоскую проекцию.
inspect() {  # $1=name $2=status $3=health $4=restarts $5=exitcode [$6=started_at] [$7=start_period_ns]
    jq -nc \
        --arg name "$1" \
        --arg status "$2" \
        --arg health "$3" \
        --argjson restarts "$4" \
        --argjson exitcode "$5" \
        --arg started_at "${6:-}" \
        --argjson start_period_ns "${7:-0}" \
        '{
            Name: ("/"+$name),
            State: {
                Status: $status,
                Restarting: ($status == "restarting"),
                ExitCode: $exitcode,
                StartedAt: $started_at,
                Health: (if $health == "none" then null else {Status: $health} end)
            },
            RestartCount: $restarts,
            Config: {Healthcheck: {StartPeriod: $start_period_ns}}
        }'
}

# run_case EXPECT_HEALTHY EXPECT_FAILED_FIXTURE1 FIXTURE1 FIXTURE2 LABEL
#   EXPECT_HEALTHY  = "true" | "false"
#   EXPECT_FAILED   = "0" | "1" | ...
#   FIXTURE1        = содержимое sample #1 (один или несколько JSONL)
#   FIXTURE2        = содержимое sample #2
#   LABEL           = человекочитаемое имя кейса
#   EXPECT_PENDING  = "true" | "false" | "" (пусто = не проверять; старые
#                     кейсы не знают про pending_start, issue #5114)
run_case() {
    local expect_healthy="$1" expect_failed="$2" fix1="$3" fix2="$4" label="$5"
    # ${7:-} в `local` нельзя: local — команда, и $? внутри подстановки
    # становится её кодом, а не кодом предыдущей. Пустой $7 тогда всегда
    # «задан», и sample_3 пишется пустым.
    local expect_pending="${6-}" remeasure="${7-}"

    local fixture_dir
    fixture_dir="$(mktemp -d)"
    printf '%s\n' "$fix1" > "$fixture_dir/sample_1.txt"
    printf '%s\n' "$fix2" > "$fixture_dir/sample_2.txt"
    # Третий замер — перемер после ожидания конца окна. Пусто = мок
    # повторит sample_2 (старые кейсы ждут ровно два вызова).
    if [ -n "$remeasure" ]; then
        printf '%s\n' "$remeasure" > "$fixture_dir/sample_3.txt"
    fi

    # Ускоряем CONFIRM_INTERVAL до 0 — тесту не нужны реальные 15s.
    local env_block=(
        "PATH=$MOCK_BIN:$PATH"
        "PI_HOST=10.1.1.11"
        "PI_USER=ros2"
        "PI_PASSWORD=open"
        "PI_COMPOSE_DIR=/tmp"
        "PI_SSH_OPTS=-o StrictHostKeyChecking=no"
        "FIXTURE_DIR=$fixture_dir"
        "CONFIRM_INTERVAL=0"
        # Ожидание конца окна в тестах выключено: фикстуры либо уже вне
        # окна, либо (кейсы P/Q) задают cap и третий замер отдельно.
        "MAX_PENDING_WAIT_S=${MAX_PENDING_WAIT_S:-0}"
        "PROBE_SLACK_S=${PROBE_SLACK_S:-0}"
        "PENDING_POLL_S=${PENDING_POLL_S:-0}"
        "ENVIRONMENT=test"
        "SCOPE=vision"
        "SUMMARY_FILE=$(mktemp)"
        "FINDINGS_FILE=/dev/null"
        "EXPECTED_CONTAINERS=${EXPECTED_CONTAINERS:-}"
    )

    # Запускаем скрипт как subshell (НЕ source) — это нативный путь, и
    # mock-sshpass видит каждый запуск как «свежий» (свой $FIXTURE_DIR/.calls).
    # Если бы мы делали source + subshell, второй запуск видел бы мусор
    # в `.calls` (n=3, n=4 → защита sample_2.txt для обоих).
    local stdout
    stdout="$(env -i "${env_block[@]}" bash "$SCRIPT_UNDER_TEST" 2>/dev/null || true)"

    rm -rf "$fixture_dir"

    # Берём JSON результат. Скрипт печатает один JSON-объект в stdout.
    local result
    result="$(echo "$stdout" | jq -c 'select(.healthy != null) | {healthy, failed_count, pending_start}' 2>/dev/null | tail -n 1 || true)"
    if [ -z "$result" ]; then
        printf '%s  [ %s ] no JSON result from script — subshell failed\n' "$RED" "$label" >&2
        printf '    stdout: %s\n' "$stdout" >&2
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
        return
    fi

    local got_healthy got_failed
    got_healthy="$(echo "$result" | jq -r '.healthy')"
    got_failed="$(echo "$result" | jq -r '.failed_count')"

    printf '  [%s]\n' "$label"
    assert_eq "$got_healthy" "$expect_healthy" "healthy"
    assert_eq "$got_failed" "$expect_failed" "failed_count"
    if [ -n "$expect_pending" ]; then
        assert_eq "$(echo "$result" | jq -r '.pending_start')" "$expect_pending" "pending_start"
    fi
}

# --- cases ------------------------------------------------------------------

# Кейс A: оба замера running → healthy (базовая линия).
FIX1="$(inspect voice-resources-init exited none 0 0
inspect oak-d running healthy 0 0)"
# (упрощаем: наш скрипт фильтрует по name; для теста достаточно 1 контейнера)
FIX1="$(inspect oak-d running healthy 0 0)"
FIX2="$(inspect oak-d running healthy 0 0)"
run_case true 0 "$FIX1" "$FIX2" "A: running+running → healthy=true (baseline)"

# Кейс B: оба замера restarting → healthy=false. ТОТ САМЫЙ БАГ #2122.
FIX1="$(inspect rob-box-quest restarting none 1 1)"
FIX2="$(inspect rob-box-quest restarting none 1 1)"
run_case false 1 "$FIX1" "$FIX2" "B: restarting+restarting → healthy=false (issue #2122 repro)"

# Кейс C: один контейнер exited 0 (one-shot init) → healthy=true.
# Это проверка ретро 7f8939d28: правило «exited → fail» БЕЗ разбора
# ExitCode валит КАЖДЫЙ деплой. Провал только при ненулевом коде.
FIX1="$(inspect voice-resources-init exited none 0 0)"
FIX2="$(inspect voice-resources-init exited none 0 0)"
run_case true 0 "$FIX1" "$FIX2" "C: one-shot init exited 0 → healthy=true (retro 7f8939d28)"

# Кейс D: exited 1 → healthy=false. Контейнер-инициализатор СЛОМАЛСЯ.
FIX1="$(inspect voice-resources-init exited none 0 1)"
FIX2="$(inspect voice-resources-init exited none 0 1)"
run_case false 1 "$FIX1" "$FIX2" "D: one-shot init exited 1 → healthy=false (init failure)"

# Кейс E: Health.Status=unhealthy в любом замере → healthy=false.
FIX1="$(inspect oak-d running healthy 0 0)"
FIX2="$(inspect oak-d running unhealthy 0 0)"
run_case false 1 "$FIX1" "$FIX2" "E: Health.Status=unhealthy in sample #2 → healthy=false"

# Кейс F: running в обоих, но RestartCount вырос между замерами →
# healthy=false. Это та самая гонка, которую НЕ ловил единичный snapshot.
FIX1="$(inspect oak-d running healthy 3 0)"
FIX2="$(inspect oak-d running healthy 4 0)"
run_case false 1 "$FIX1" "$FIX2" "F: running+running but RestartCount grew 3→4 → healthy=false (race window)"

# Кейс G: dead → healthy=false.
FIX1="$(inspect oak-d dead none 5 137)"
FIX2="$(inspect oak-d dead none 5 137)"
run_case false 1 "$FIX1" "$FIX2" "G: State.Status=dead → healthy=false"

# Кейсы H-J: EXPECTED_CONTAINERS (ADR-0159, Sound Pi) — обязательные
# контейнеры узла. Пустой `compose ps` НЕ должен давать «healthy» по пустому
# списку, если вызывающий знает, что на узле должно что-то работать.
EXPECTED_CONTAINERS="voice-assistant supercollider"
FIX1="$(inspect voice-assistant running healthy 0 0
inspect supercollider running healthy 0 0)"
run_case true 0 "$FIX1" "$FIX1" "H: expected both present and running → healthy=true"

FIX1="$(inspect voice-assistant running healthy 0 0)"
run_case false 1 "$FIX1" "$FIX1" "I: expected supercollider absent from compose ps → healthy=false"

run_case false 2 "" "" "J: empty compose ps with 2 expected → healthy=false (no green on empty list)"
EXPECTED_CONTAINERS=""

# Кейс K: без EXPECTED_CONTAINERS пустой список по-прежнему healthy (поведение
# Vision/Main не меняется).
run_case true 0 "" "" "K: empty compose ps, nothing expected → healthy=true (legacy behaviour kept)"

# Кейсы L-N (issue #5012): unhealthy внутри start_period — не провал.
# Окно «3×interval» у perception (docker/main/docker-compose.yaml) длиннее
# start_period, и гейт красил стартовый unhealthy. Поднимать start_period
# «на глаз» нельзя — льготу даёт возраст контейнера, не константа сервиса.
# StartPeriod — наносекунды, как отдаёт `docker inspect`.
NOW_EPOCH="$(date +%s)"
YOUNG_STARTED="$(date -u -d "@$((NOW_EPOCH - 5))" +%Y-%m-%dT%H:%M:%SZ)"
OLD_STARTED="$(date -u -d "@$((NOW_EPOCH - 120))" +%Y-%m-%dT%H:%M:%SZ)"
START_PERIOD_NS=20000000000  # 20s, как у perception

FIX1="$(inspect perception running unhealthy 0 0 "$YOUNG_STARTED" "$START_PERIOD_NS")"
run_case true 0 "$FIX1" "$FIX1" "L: unhealthy, container age 5s < start_period 20s → not failed (issue #5012)"

FIX1="$(inspect perception running unhealthy 0 0 "$OLD_STARTED" "$START_PERIOD_NS")"
run_case false 1 "$FIX1" "$FIX1" "M: unhealthy, container age 120s > start_period 20s → failed (issue #5012)"

FIX1="$(inspect perception running unhealthy 1 0 "$YOUNG_STARTED" "$START_PERIOD_NS")"
FIX2="$(inspect perception running unhealthy 2 0 "$YOUNG_STARTED" "$START_PERIOD_NS")"
run_case false 1 "$FIX1" "$FIX2" "N: restarts grew 1→2 inside start_period → still failed (restart rule unchanged)"

# Кейсы O-S (issue #5114): health=starting — не здоровье.
# Ретро run 45019034598: nav2 на обоих замерах health=starting, restarts=0,
# а шаг статуса написал «All Main Pi containers running and confirmed stable».
# Docker не выносит вердикт, пока идёт start_period, поэтому starting внутри
# окна — не провал (failed_count=0), но и не healthy: pending_start=true,
# вызывающий должен дождаться конца окна и смерить снова. После окна
# starting — провал с reason=starting_after_start_period.
NAV2_START_PERIOD_NS=90000000000  # 90s, как у nav2 (docker/main/docker-compose.yaml)

FIX1="$(inspect nav2 running starting 0 0 "$OLD_STARTED" "$NAV2_START_PERIOD_NS")"
run_case false 1 "$FIX1" "$FIX1" "O: starting on both samples, age 120s > start_period 90s → failed (issue #5114)" false

# Кейс P (приёмка #5118): starting внутри окна — скрипт сам ждёт конца окна и
# мерит снова. Третий замер уже healthy: к финальному JSON pending_start=false
# и healthy=true. poll=0, чтобы не спать реальные секунды: дедлайн в будущем,
# но ожидание режется до шага опроса.
FIX1="$(inspect nav2 running starting 0 0 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
FIX3="$(inspect nav2 running healthy 0 0 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
MAX_PENDING_WAIT_S=120 PROBE_SLACK_S=0 PENDING_POLL_S=0 \
    run_case true 0 "$FIX1" "$FIX1" "P: starting inside window, re-measure healthy → healthy=true, pending_start=false (issue #5114, приёмка #5118)" false "$FIX3"
MAX_PENDING_WAIT_S="" PROBE_SLACK_S="" PENDING_POLL_S=""

# Окно с запасом против границы секунды: возраст считается строго меньше
# start_period, и замер на стыке 90-й секунды уже «после окна».
Q_PERIOD_NS=300000000000
FIX1="$(inspect nav2 running healthy 0 0 "$YOUNG_STARTED" "$Q_PERIOD_NS")"
FIX2="$(inspect nav2 running starting 0 0 "$YOUNG_STARTED" "$Q_PERIOD_NS")"
FIX3="$(inspect nav2 running healthy 0 0 "$YOUNG_STARTED" "$Q_PERIOD_NS")"
MAX_PENDING_WAIT_S=120 PROBE_SLACK_S=0 PENDING_POLL_S=0 \
    run_case true 0 "$FIX1" "$FIX2" "Q: starting only in sample #2 inside window, re-measure healthy → healthy=true (issue #5114, приёмка #5118)" false "$FIX3"
MAX_PENDING_WAIT_S="" PROBE_SLACK_S="" PENDING_POLL_S=""

# Кейс T: ожидание упёрлось в cap, контейнер всё ещё starting внутри окна.
# Вердикта нет — не здоровье (pending_start=true, failed_count=0) и не ложный
# провал: окно ещё не кончилось, Docker не обязан был вынести его.
# Окно длиннее cap: перемер за cap всё ещё внутри start_period, поэтому
# starting остаётся pending, а не starting_after_start_period.
FIX1="$(inspect nav2 running starting 0 0 "$YOUNG_STARTED" 600000000000)"
MAX_PENDING_WAIT_S=2 PENDING_POLL_S=1 \
    run_case false 0 "$FIX1" "$FIX1" "T: starting inside window, wait cap exhausted → not healthy, pending_start=true (issue #5114)" true
MAX_PENDING_WAIT_S=""

FIX1="$(inspect nav2 running starting 0 0)"
run_case false 1 "$FIX1" "$FIX1" "R: starting with no start_period data → failed, no grace (issue #5114)" false

FIX1="$(inspect nav2 running starting 1 0 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
FIX2="$(inspect nav2 running starting 2 0 "$YOUNG_STARTED" "$NAV2_START_PERIOD_NS")"
run_case false 1 "$FIX1" "$FIX2" "S: RestartCount grew 1→2 while starting inside window → still failed (restart rule unchanged)" false

# --- summary ----------------------------------------------------------------

echo
if [ "$FAILED" -eq 0 ]; then
    printf '%sPASS%s: %d/%d assertions ok\n' "$GREEN" "$NC" "$TOTAL" "$TOTAL"
    exit 0
else
    printf '%sFAIL%s: %d/%d assertions failed\n' "$RED" "$NC" "$FAILED" "$TOTAL"
    exit 1
fi
