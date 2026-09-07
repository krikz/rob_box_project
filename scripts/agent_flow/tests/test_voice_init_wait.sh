#!/usr/bin/env bash
# ============================================================================
# test_voice_init_wait.sh — регресс-тест voice_init_wait.sh (issue #2095)
#
# Сценарий: docker compose up -d в deploy workflow запускает voice-resources-init
# (one-shot init-контейнер). После Exit docker daemon ~5-6 секунд удерживает
# name-slot — и `docker compose up` в своей финальной reconcile-фазе пытается
# resolve exited ID → "No such container" → exit 1 (deploy #34144648195).
#
# voice_init_wait.sh фиксит это: docker wait + sleep + docker rm -f.
#
# Acceptance (issue #2095, acceptance #1):
#   1. Container существует → wait + sleep + rm, exit 0
#   2. Container уже Exited → wait возвращает мгновенно, всё равно sleep + rm
#   3. Container не существует (init не запускался) → noop, exit 0
#   4. timeout в docker wait → exit 3 (FATAL)
#   5. --wait/--timeout с не-числовым значением → exit 2 (usage)
#   6. --bogus аргумент → exit 2 (usage)
#   7. docker команды вызваны в правильном порядке: wait → sleep → rm
#   8. DOCKER_CMD env позволяет подменить docker на mock (testability)
#
# Run:
#   bash scripts/agent_flow/tests/test_voice_init_wait.sh
# Returns exit 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_LIB_DIR/.." && pwd)"  # scripts/agent_flow/
TARGET="$REPO_ROOT/voice_init_wait.sh"

# Sanity: target файл существует и исполняем
[ -x "$TARGET" ] || {
    echo "[FATAL] $TARGET not found or not executable" >&2
    exit 1
}

# Per-run scratch
TEST_TMP="${TEST_TMP:-/tmp/voice-init-wait-tests.$$}"
rm -rf "$TEST_TMP"
mkdir -p "$TEST_TMP/bin" "$TEST_TMP/state"

# Colors
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

# --- Mock docker ------------------------------------------------------------
# STATE FILE: $TEST_TMP/state/containers — список имён контейнеров (по строке)
# JOURNAL:    $TEST_TMP/journal — append-only лог вызовов: <ts>\t<cmd...>
#
# Поддерживаемые подкоманды:
#   ps -a --filter name=^/X$ --format '{{.Names}}'    → выводит X если есть в state
#   wait X                                            → возвращает exit code из state/<X>.wait (default 0)
#   rm -f X                                            → удаляет X из state
#   ps -a --filter ... --filter status=running ...    → для совместимости (noop, ничего не выводит)
#
# Спец.переменные окружения для тестов:
#   MOCK_WAIT_DELAY=<sec>     — sleep перед exit в `docker wait` (default 0)
#   MOCK_WAIT_HANG=1          — sleep forever (для теста timeout)
#   MOCK_WAIT_EXIT_CODE=<n>   — exit code `docker wait` (default 0)
#   MOCK_RM_FAIL=1            — `docker rm -f` возвращает 1

cat > "$TEST_TMP/bin/docker" <<'DOCKER_MOCK_EOF'
#!/bin/bash
state_dir="${DOCKER_STATE_DIR:-/tmp/voice-init-wait-tests.UNSET/state}"
journal="${DOCKER_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
printf '%s\t%s\n' "$ts" "$*" >>"$journal"

# Parse: docker <subcmd> [args...]
subcmd="${1:-}"
shift 2>/dev/null || true

# Helper: extract positional name из args (пропускает флаги и достаёт первый
# non-flag аргумент). Это для `docker rm -f NAME` / `docker wait NAME` —
# реальный docker CLI принимает -f flag в произвольной позиции.
extract_name() {
    local arg
    for arg in "$@"; do
        case "$arg" in
            -*) ;;  # skip flags
            *)
                printf '%s\n' "$arg"
                return 0
                ;;
        esac
    done
    return 1
}

case "$subcmd" in
    ps)
        # Filter: name=^/X$ (точное совпадение имени контейнера)
        want_name=""
        for arg in "$@"; do
            case "$arg" in
                name=^*\$)
                    want_name="${arg#name=^/}"
                    want_name="${want_name%\$}"
                    ;;
            esac
        done
        if [ -n "$want_name" ] && [ -f "$state_dir/$want_name" ]; then
            printf '%s\n' "$want_name"
        fi
        ;;
    wait)
        name="$(extract_name "$@" || true)"
        [ -n "$name" ] || { echo "Error: No such container: (no name)" >&2; exit 1; }
        # Marker-файл наличия контейнера
        [ -f "$state_dir/$name" ] || { echo "Error: No such container: $name" >&2; exit 1; }
        # Симуляция зависания wait (для теста timeout)
        if [ "${MOCK_WAIT_HANG:-0}" = "1" ]; then
            # В реальности `docker wait` при SIGTERM от `timeout` возвращает
            # exit 124 (или 137 при SIGKILL). Наш mock должен это
            # симулировать — bash по умолчанию выходит с кодом сигнала+128
            # при killed-by-signal, НО sleep корректно ловит SIGTERM и
            # завершается со 128+15=143, что bash игнорирует и завершает
            # скрипт штатно (exit 0). Поэтому явно ловим и завершаем как
            # timeout'нутый процесс: signal handler → exit 124.
            trap 'exit 124' TERM
            sleep 3600
            # Сюда не должны дойти (sleep прервётся по сигналу)
        fi
        sleep "${MOCK_WAIT_DELAY:-0}"
        # Exit code из маркера (если есть)
        if [ -f "$state_dir/$name.wait" ]; then
            cat "$state_dir/$name.wait"
        else
            printf '0\n'
        fi
        ;;
    rm)
        name="$(extract_name "$@" || true)"
        [ -n "$name" ] || { echo "Error: No such container: (no name)" >&2; exit 1; }
        if [ "${MOCK_RM_FAIL:-0}" = "1" ]; then
            echo "Error: cannot remove container (mock fail)" >&2
            exit 1
        fi
        rm -f "$state_dir/$name" "$state_dir/$name.wait"
        ;;
    *)
        echo "mock-docker: unsupported subcmd: $subcmd" >&2
        exit 99
        ;;
esac
DOCKER_MOCK_EOF
chmod +x "$TEST_TMP/bin/docker"

# Helper: положить контейнер в state
seed_container() {  # $1=name [ $2=wait_exit_code ]
    touch "$TEST_TMP/state/$1"
    if [ -n "${2:-}" ]; then
        printf '%s\n' "$2" > "$TEST_TMP/state/$1.wait"
    fi
}

# Helper: очистить state для изоляции между тестами
reset_state() {
    rm -f "$TEST_TMP/state"/* 2>/dev/null || true
}

# Helper: проверить что контейнер есть в state
assert_container_exists() {  # $1=name
    if [ ! -f "$TEST_TMP/state/$1" ]; then
        printf '  %sassert fail:%s container %s should still exist\n' "$RED" "$END" "$1" >&2
        return 1
    fi
}

# Helper: проверить что контейнер удалён
assert_container_gone() {  # $1=name
    if [ -f "$TEST_TMP/state/$1" ]; then
        printf '  %sassert fail:%s container %s should be gone\n' "$RED" "$END" "$1" >&2
        return 1
    fi
}

# Helper: журнал вызовов docker — это просто файл, грепаем по подкоманде
journal_has() {  # $1=subcmd
    grep -qE "[[:space:]]$1([[:space:]]|$)" "$TEST_TMP/journal"
}

# Helper: assert journal содержит подкоманды в ожидаемом порядке.
# Использует line numbers: wait_line < rm_line.
assert_call_order() {  # $1=first_subcmd $2=second_subcmd
    local first_line second_line
    first_line="$(grep -nE "[[:space:]]$1([[:space:]]|$)" "$TEST_TMP/journal" | head -n1 | cut -d: -f1 || true)"
    second_line="$(grep -nE "[[:space:]]$2([[:space:]]|$)" "$TEST_TMP/journal" | head -n1 | cut -d: -f1 || true)"
    if [ -z "$first_line" ] || [ -z "$second_line" ]; then
        printf '  %sassert fail:%s order check — %s line=%q %s line=%q\n' \
            "$RED" "$END" "$1" "$first_line" "$2" "$second_line" >&2
        return 1
    fi
    if [ "$first_line" -ge "$second_line" ]; then
        printf '  %sassert fail:%s expected %s before %s, got lines %s vs %s\n' \
            "$RED" "$END" "$1" "$2" "$first_line" "$second_line" >&2
        return 1
    fi
}

# Helper: запустить TARGET с заранее выставленными env.
# Использует bash -c "export ...; bash TARGET" — критично для AC4 (timeout),
# где MOCK_WAIT_HANG должен дойти до mock-docker через цепочку
# bash → timeout → docker. Без export MOCK_WAIT_HANG не прокидывается.
#
# Args: остальные args (после 6 setup) → TARGET args.
# Args: $1=init_container $2=daemon_release_wait $3=wait_timeout
#       $4=mock_wait_delay $5=mock_wait_hang $6=mock_rm_fail
run_target_env() {
    local init_c="${1:-voice-resources-init}"
    local drw="${2:-1}"
    local wto="${3:-120}"
    local mwd="${4:-0}"
    local mwh="${5:-0}"
    local mrf="${6:-0}"
    shift 6 || shift $#
    local d="$TEST_TMP/bin"
    local s="$TEST_TMP/state"
    local j="$TEST_TMP/journal"
    local t="$TARGET"
    bash -c "export PATH='$d:'\"\$PATH\"; export DOCKER_STATE_DIR='$s'; export DOCKER_JOURNAL='$j'; export MOCK_WAIT_DELAY='$mwd'; export MOCK_WAIT_HANG='$mwh'; export MOCK_RM_FAIL='$mrf'; export DAEMON_RELEASE_WAIT_SECONDS='$drw'; export WAIT_TIMEOUT_SECONDS='$wto'; exec bash '$t' \"\$@\"" bash --container "$init_c" "$@"
}

# --- Test registry ----------------------------------------------------------
run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

# ============================================================================
# Acceptance tests
# ============================================================================

# 1. Container существует, ещё running → wait блокирует (мгновенно в mock),
#    потом sleep, потом rm → exit 0, container удалён.
test_existing_container_happy_path() {
    reset_state
    rm -f "$TEST_TMP/journal"
    seed_container voice-resources-init

    run_target_env voice-resources-init 1 120 0 0 0 --wait 1
    local rc=$?

    if [ "$rc" -ne 0 ]; then
        printf '  exit was %d, expected 0\n' "$rc" >&2
        return 1
    fi
    assert_container_gone voice-resources-init || return 1
    journal_has wait || { echo "  no docker wait in journal" >&2; return 1; }
    journal_has rm   || { echo "  no docker rm in journal"   >&2; return 1; }
    assert_call_order wait rm || return 1
}

# 2. Container уже Exited (wait_exit_code=0 в state). wait возвращает
#    мгновенно, но sleep+rm всё равно отрабатывают.
test_exited_container() {
    reset_state
    rm -f "$TEST_TMP/journal"
    seed_container voice-resources-init 0

    run_target_env voice-resources-init 1 120 0 0 0 --wait 1
    local rc=$?

    if [ "$rc" -ne 0 ]; then
        printf '  exit was %d, expected 0\n' "$rc" >&2
        return 1
    fi
    assert_container_gone voice-resources-init || return 1
}

# 3. Container не существует (init не запускался в этом деплое) → noop, exit 0.
test_missing_container_noop() {
    reset_state
    rm -f "$TEST_TMP/journal"
    # State пустой — контейнера нет

    run_target_env voice-resources-init 1 120 0 0 0 --wait 1
    local rc=$?

    if [ "$rc" -ne 0 ]; then
        printf '  exit was %d, expected 0 (noop)\n' "$rc" >&2
        return 1
    fi
    # В журнале не должно быть wait/rm (только ps-проверка, которая возвращает 0 строк)
    if journal_has wait; then
        echo "  docker wait should not be called for missing container" >&2
        return 1
    fi
    if journal_has rm; then
        echo "  docker rm should not be called for missing container" >&2
        return 1
    fi
}

# 4. docker wait зависает (timeout в wait). MOCK_WAIT_HANG=1 → wait_timeout
#    должен сработать → exit 3.
test_wait_timeout() {
    reset_state
    rm -f "$TEST_TMP/journal"
    seed_container voice-resources-init

    run_target_env voice-resources-init 1 2 0 1 0 --wait 1 --timeout 2
    local rc=$?

    if [ "$rc" -ne 3 ]; then
        printf '  exit was %d, expected 3 (FATAL timeout)\n' "$rc" >&2
        return 1
    fi
}

# 5. Невалидный --wait (не число) → exit 2
test_invalid_wait_arg() {
    reset_state
    rm -f "$TEST_TMP/journal"
    run_target_env voice-resources-init 1 120 0 0 0 --wait abc
    local rc=$?

    if [ "$rc" -ne 2 ]; then
        printf '  exit was %d, expected 2 (usage)\n' "$rc" >&2
        return 1
    fi
}

# 6. Неизвестный флаг → exit 2
test_bogus_arg() {
    reset_state
    rm -f "$TEST_TMP/journal"
    run_target_env voice-resources-init 1 120 0 0 0 --bogus
    local rc=$?

    if [ "$rc" -ne 2 ]; then
        printf '  exit was %d, expected 2 (usage)\n' "$rc" >&2
        return 1
    fi
}

# 7. --help → exit 0, usage на stdout
test_help() {
    local out
    out=$(run_target_env voice-resources-init 1 120 0 0 0 --help 2>&1)
    local rc=$?

    if [ "$rc" -ne 0 ]; then
        printf '  exit was %d, expected 0\n' "$rc" >&2
        return 1
    fi
    case "$out" in
        *Usage:*--container*) return 0 ;;
        *)
            echo "  --help output missing usage block" >&2
            return 1
            ;;
    esac
}

# 8. Не-имя-контейнера (фильтр точного совпадения): если в state лежит
#    'voice-resources-init-extra', то фильтр name=^/voice-resources-init$
#    НЕ должен его ловить.
test_exact_name_match() {
    reset_state
    rm -f "$TEST_TMP/journal"
    seed_container voice-resources-init-extra

    run_target_env voice-resources-init 1 120 0 0 0 --wait 1
    local rc=$?

    # Контейнер voice-resources-init не существует → noop, exit 0.
    # voice-resources-init-extra должен ОСТАТЬСЯ в state (не наш таргет).
    if [ "$rc" -ne 0 ]; then
        printf '  exit was %d, expected 0 (noop для отсутствующего target)\n' "$rc" >&2
        return 1
    fi
    assert_container_exists voice-resources-init-extra || return 1
}

# 9. Команды вызваны в правильном порядке: ps (проверка) → wait → rm.
test_call_sequence() {
    reset_state
    rm -f "$TEST_TMP/journal"
    seed_container voice-resources-init

    run_target_env voice-resources-init 1 120 0 0 0 --wait 1
    local rc=$?

    if [ "$rc" -ne 0 ]; then
        printf '  exit was %d, expected 0\n' "$rc" >&2
        return 1
    fi
    # ps проверка должна быть ДО wait
    assert_call_order ps wait || return 1
    # wait ДО rm
    assert_call_order wait rm || return 1
}

# 10. Sleep — реальный, не noop. Проверяем через замер времени: сон >= значение.
test_sleep_actually_runs() {
    reset_state
    rm -f "$TEST_TMP/journal"
    seed_container voice-resources-init

    local start end elapsed
    start=$(date +%s)
    run_target_env voice-resources-init 1 120 0 0 0 --wait 2
    end=$(date +%s)
    elapsed=$((end - start))

    # Должно занять минимум 2 сек (sleep 2)
    if [ "$elapsed" -lt 2 ]; then
        printf '  elapsed=%ds, expected >=2s (sleep не отработал)\n' "$elapsed" >&2
        return 1
    fi
}

# ============================================================================
# Run all tests
# ============================================================================
run_test "AC1:  existing container → wait+sleep+rm, exit 0"   test_existing_container_happy_path
run_test "AC2:  already-Exited container → wait instant, rm"   test_exited_container
run_test "AC3:  missing container → noop, exit 0"             test_missing_container_noop
run_test "AC4:  docker wait timeout → exit 3"                 test_wait_timeout
run_test "AC5:  invalid --wait value → exit 2"                test_invalid_wait_arg
run_test "AC6:  unknown flag → exit 2"                        test_bogus_arg
run_test "AC7:  --help → exit 0, usage on stdout"             test_help
run_test "AC8:  exact-name match (no false positives)"        test_exact_name_match
run_test "AC9:  call order ps → wait → rm"                    test_call_sequence
run_test "AC10: sleep actually sleeps ≥ configured seconds"   test_sleep_actually_runs

printf '\n%s==== Summary ====%s\n' "$YEL" "$END"
printf 'total:  %d\n' "$TESTS_TOTAL"
printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf '%sfailed: %d%s\n' "$RED" "$TESTS_FAILED" "$END"
    printf 'failures:\n'
    for n in "${FAILED_NAMES[@]}"; do printf '  - %s\n' "$n"; done
    exit 1
fi
exit 0