#!/usr/bin/env bash
# ============================================================================
# test_check_container_status.sh — регрессионный тест (issue #2122)
#
# Источник истины: <repo>/.github/scripts/tests/test_check_container_status.sh
#
# Тестирует <repo>/.github/scripts/check_container_status.sh (вынесенная из
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
#   bash .github/scripts/tests/test_check_container_status.sh
#   make -C .github/scripts/tests test_check_container_status
# ============================================================================
set -euo pipefail

# --- paths -----------------------------------------------------------------
SCRIPT_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR_HERE/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REPO_ROOT/.github/scripts/check_container_status.sh"

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
case "$n" in
    1) sample_file="$FIXTURE_DIR/sample_1.txt" ;;
    2) sample_file="$FIXTURE_DIR/sample_2.txt" ;;
    *) sample_file="$FIXTURE_DIR/sample_2.txt" ;;  # защита от >2 (не должно быть)
esac
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
inspect() {  # $1=name $2=status $3=health $4=restarts $5=exitcode
    jq -nc \
        --arg name "$1" \
        --arg status "$2" \
        --arg health "$3" \
        --argjson restarts "$4" \
        --argjson exitcode "$5" \
        '{
            Name: ("/"+$name),
            State: {
                Status: $status,
                Restarting: ($status == "restarting"),
                ExitCode: $exitcode,
                Health: (if $health == "none" then null else {Status: $health} end)
            },
            RestartCount: $restarts
        }'
}

# run_case EXPECT_HEALTHY EXPECT_FAILED_FIXTURE1 FIXTURE1 FIXTURE2 LABEL
#   EXPECT_HEALTHY  = "true" | "false"
#   EXPECT_FAILED   = "0" | "1" | ...
#   FIXTURE1        = содержимое sample #1 (один или несколько JSONL)
#   FIXTURE2        = содержимое sample #2
#   LABEL           = человекочитаемое имя кейса
run_case() {
    local expect_healthy="$1" expect_failed="$2" fix1="$3" fix2="$4" label="$5"

    local fixture_dir
    fixture_dir="$(mktemp -d)"
    printf '%s\n' "$fix1" > "$fixture_dir/sample_1.txt"
    printf '%s\n' "$fix2" > "$fixture_dir/sample_2.txt"

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
        "ENVIRONMENT=test"
        "SCOPE=vision"
        "SUMMARY_FILE=$(mktemp)"
        "FINDINGS_FILE=/dev/null"
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
    result="$(echo "$stdout" | jq -c 'select(.healthy != null) | {healthy, failed_count}' 2>/dev/null | tail -n 1 || true)"
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

# --- summary ----------------------------------------------------------------

echo
if [ "$FAILED" -eq 0 ]; then
    printf '%sPASS%s: %d/%d assertions ok\n' "$GREEN" "$NC" "$TOTAL" "$TOTAL"
    exit 0
else
    printf '%sFAIL%s: %d/%d assertions failed\n' "$RED" "$NC" "$FAILED" "$TOTAL"
    exit 1
fi
