#!/usr/bin/env bash
# ============================================================================
# test_check_set_voice_traffic.sh — регрессионный тест (issue #2138.C / ADR-0067)
#
# Источник истины: <repo>/.github/scripts/tests/test_check_set_voice_traffic.sh
#
# Тестирует <repo>/.github/scripts/check_set_voice_traffic.sh (вынесенная
# из workflow L-Deploy and Verify.yml логика healthcheck'а трафика на
# /avatar/set_voice). Каждый кейс запускает СКРИПТ с мокнутыми `sshpass`/
# `timeout`/`ros2` через PATH, читает HEALTHY_RESULT/HZ_RESULT и сверяет
# с ожиданием.
#
# Зачем: ретро run #34153246013 + ADR-0067 §3.3 — на robot'е picker
# «голос не меняется» без видимой причины. Нужно зафиксировать, что
# healthcheck умеет отличать «есть трафик» от «совсем нет трафика»
# (H1/H2/H3) и НЕ валит деплой в случае «редкого единичного message
# за окно» (что нормально для picker'а — он шлёт только при APPLY).
#
# Этот тест фиксирует поведение скрипта:
#   1. hz > порога с живыми samples       → healthy=true
#   2. hz == 0, нет samples               → healthy=false (CI/«нет трафика»)
#   3. hz == 0, но пустой output          → healthy=false (topic не существует)
#   4. `timeout` отвалился по signal       → healthy=false (никакого вывода)
#   5. average rate: <0.001               → healthy=false (ниже порога)
#
# Run:
#   bash .github/scripts/tests/test_check_set_voice_traffic.sh
# ============================================================================
set -euo pipefail

# --- paths -----------------------------------------------------------------
SCRIPT_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR_HERE/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REPO_ROOT/.github/scripts/check_set_voice_traffic.sh"

[ -f "$SCRIPT_UNDER_TEST" ] || {
    echo "FATAL: $SCRIPT_UNDER_TEST не найден. Запускай из корня репо." >&2
    exit 1
}

command -v jq >/dev/null 2>&1 || {
    echo "FATAL: jq не найден в PATH. Установи: apt-get install jq" >&2
    exit 1
}

# Подменяем PATH на изолированный bin с моками sshpass + timeout.
MOCK_BIN="$(mktemp -d)"
trap 'rm -rf "$MOCK_BIN"' EXIT

# --- mock sshpass -----------------------------------------------------------
# Возвращает фикстуру, имитируя вывод ``ros2 topic hz <TOPIC>``.
# Тест-раннер задаёт ``FIXTURE=`` через env, указывающую на нужную фикстуру.
cat >"$MOCK_BIN/sshpass" <<'SSHPASS_EOF'
#!/usr/bin/env bash
# Mock sshpass: возвращает содержимое $FIXTURE_FILE, обёрнутое в
# ``timeout``-команду (наш скрипт гоняет ``timeout <W>s ros2 topic hz ...``
# внутри ssh-команды — нас интересует stdout, который вернёт ssh).
if [ -n "${FIXTURE_FILE:-}" ] && [ -f "$FIXTURE_FILE" ]; then
    cat "$FIXTURE_FILE"
else
    echo ""
fi
SSHPASS_EOF
chmod +x "$MOCK_BIN/sshpass"

# Также мокаем ``timeout`` для гарантии — наш скрипт использует
# ``timeout <W>s ros2 topic hz ...`` на стороне ssh, в фикстуре это
# не нужно. Но если скрипт случайно вызывает ``timeout`` локально,
# не пускаем выполнение на реальный бинарник.
cat >"$MOCK_BIN/timeout" <<'TIMEOUT_EOF'
#!/usr/bin/env bash
# Mock: «выполнить» команду (наш скрипт вызывает его на стороне ssh,
# но для теста мы подменяем весь ssh-side execution через sshpass-mock).
# Здесь мы — заглушка для случая, когда скрипт вдруг дёрнет timeout
# локально (в нашем случае — нет, но безопасность прежде всего).
echo "[mock-timeout] WARN: локальный timeout вызван неожиданно" >&2
exit 124
TIMEOUT_EOF
chmod +x "$MOCK_BIN/timeout"

# --- test runner ------------------------------------------------------------

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

# run_case EXPECT_HEALTHY EXPECT_HZ FIXTURE_TEXT LABEL
#   EXPECT_HEALTHY = "true" | "false"
#   EXPECT_HZ      = строка hz (сравниваем как текст)
#   FIXTURE_TEXT   = содержимое mock-вывода ``ros2 topic hz``
#   LABEL          = человекочитаемое имя
run_case() {
    local expect_healthy="$1" expect_hz="$2" fixture_text="$3" label="$4"

    local fixture_file
    fixture_file="$(mktemp)"
    printf '%s\n' "$fixture_text" > "$fixture_file"

    # Ускоряем HZ_WINDOW до 0 (скрипт всё равно парсит stdout по фикстуре).
    local env_block=(
        "PATH=$MOCK_BIN:$PATH"
        "PI_HOST=10.1.1.11"
        "PI_USER=ros2"
        "PI_PASSWORD=open"
        "PI_COMPOSE_DIR=/tmp"
        "PI_SSH_OPTS=-o StrictHostKeyChecking=no"
        "FIXTURE_FILE=$fixture_file"
        "HZ_WINDOW=10"
        "HZ_ALERT_THRESHOLD=0.001"
        "ENVIRONMENT=test"
        "SCOPE=vision"
        "SUMMARY_FILE=$(mktemp)"
        "FINDINGS_FILE=/dev/null"
        "TOPIC=/avatar/set_voice"
    )

    # Запускаем скрипт как subshell — HEALTHY_RESULT приходит через export,
    # workflow читает их через ``source``. Но в тесте мы не source'им,
    # а парсим stdout-вывод скрипта (там JSON c healthy/hz/samples).
    local stdout
    stdout="$(env -i "${env_block[@]}" bash "$SCRIPT_UNDER_TEST" 2>/dev/null || true)"

    rm -f "$fixture_file"

    local result
    result="$(echo "$stdout" | jq -c 'select(.healthy != null) | {healthy, hz}' 2>/dev/null | tail -n 1 || true)"
    if [ -z "$result" ]; then
        printf '  %s✗%s no JSON result from script\n' "$RED" "$label" >&2
        printf '    stdout: %s\n' "$stdout" >&2
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
        return
    fi

    local got_healthy got_hz
    got_healthy="$(echo "$result" | jq -r '.healthy')"
    got_hz="$(echo "$result" | jq -r '.hz')"

    printf '  [%s]\n' "$label"
    assert_eq "$got_healthy" "$expect_healthy" "healthy"
    assert_eq "$got_hz" "$expect_hz" "hz"
}

# --- cases ------------------------------------------------------------------

# Кейс A: классический живой трафик — hz ~0.5 (2 msg/s). healthy=true.
FIX_A=$(cat <<'EOF'
subscribed to topic [/avatar/set_voice]
average rate: 0.5000
	min: 2.000s max: 2.000s std dev: 0.000s window: 4
average rate: 0.5000
	min: 2.000s max: 2.000s std dev: 0.000s window: 30
EOF
)
run_case true 0.5000 "$FIX_A" "A: живой трафик (hz=0.5) → healthy=true"

# Кейс B: picker жив, шлёт редко (1 msg/min) → hz ≈ 0.0167 → healthy=true
# (порог 0.001, так что 0.0167 выше порога; «редко, но шлёт» не алерт).
FIX_B=$(cat <<'EOF'
subscribed to topic [/avatar/set_voice]
average rate: 0.0167
	min: 60.000s max: 60.000s std dev: 0.000s window: 60
EOF
)
run_case true 0.0167 "$FIX_B" "B: редкий picker-traffic (hz=0.0167, 1/мин) → healthy=true (выше порога)"

# Кейс C: пустой вывод (топик не существует / ros2 topic hz завис).
# Парсер не нашёл «average rate:» → hz=0.0 → healthy=false.
FIX_C=""
run_case false 0.0000 "$FIX_C" "C: топик не существует (empty stdout) → healthy=false"

# Кейс D: subscribed, но average rate = 0 (никто не пишет за окно). hz=0.0.
FIX_D=$(cat <<'EOF'
subscribed to topic [/avatar/set_voice]
average rate: 0.0000
	min: 0.000s max: 0.000s std dev: 0.000s window: 10
EOF
)
run_case false 0.0000 "$FIX_D" "D: topic существует, но нет сообщений (hz=0.0) → healthy=false (H1: pub/sub дохлый)"

# Кейс E: hz ниже порога (0.0005 < 0.001) — пограничный кейс.
FIX_E=$(cat <<'EOF'
subscribed to topic [/avatar/set_voice]
average rate: 0.0005
	min: 2000.000s max: 2000.000s std dev: 0.000s window: 10
EOF
)
run_case false 0.0005 "$FIX_E" "E: hz ниже порога (0.0005 < 0.001) → healthy=false"

# Кейс F: много сообщений, высокое hz (10 msg/s). healthy=true (не при
# повышении порога наоборот).
FIX_F=$(cat <<'EOF'
subscribed to topic [/avatar/set_voice]
average rate: 10.0000
	min: 0.100s max: 0.100s std dev: 0.000s window: 100
EOF
)
run_case true 10.0000 "$FIX_F" "F: интенсивный трафик (hz=10) → healthy=true"

# --- summary ----------------------------------------------------------------

echo
if [ "$FAILED" -eq 0 ]; then
    printf '%sPASS%s: %d/%d assertions ok\n' "$GREEN" "$NC" "$TOTAL" "$TOTAL"
    exit 0
else
    printf '%sFAIL%s: %d/%d assertions failed\n' "$RED" "$NC" "$FAILED" "$TOTAL"
    exit 1
fi
