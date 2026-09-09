#!/usr/bin/env bash
# ============================================================================
# test_merge_gate_logging.sh — regression test для silent-tick фикса.
#
# Контекст (ADR-0079 / ретро t_e3fc9bfe, issue #1977):
#   50+ тиков merge-gate числились «silent (empty output)» при exit=0 —
#   потому что cron читает STDOUT (hermes_cli.subcommands.cron:
#   «Empty stdout = silent»), а скрипт исторически писал в stderr.
#   Фикс: helper `out()` пишет в stdout + per-day log-файл, marker'ы
#   tick_start_marker / tick_end_marker зеркалятся туда же.
#
# Этот тест проверяет контракт:
#   1. Запустить merge-gate с пустым issue list (нет needs-e2e).
#   2. Убедиться, что stdout содержит `# TICK_SUMMARY: start` и
#      `# TICK_SUMMARY: end` (вместо прежнего «silent (empty output)»).
#   3. Убедиться, что exit 0 (нормальный skip-tick).
#   4. Убедиться, что per-day log-файл создан в
#      `$MERGE_GATE_TICK_LOG_DIR/YYYY-MM-DD.log` и содержит обе строки.
#
# Использование:
#   bash tests/agent_flow/test_merge_gate_logging.sh
#
# Exit codes:
#   0 — PASS (silent fixed)
#   1 — FAIL (silent not fixed / marker missing / log file missing)
# ============================================================================
set -euo pipefail

SCRIPT_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT_HERE="$(cd "$SCRIPT_DIR_HERE/../.." && pwd)"
SCRIPT_UNDER_TEST="${REPO_ROOT_HERE}/scripts/agent_flow/agent-flow-merge-gate.sh"
TMP_DIR="$(mktemp -d -t merge-gate-log-test.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT

echo "test_merge_gate_logging: REPO_ROOT=${REPO_ROOT_HERE} TMP=${TMP_DIR}"

if [ ! -f "$SCRIPT_UNDER_TEST" ]; then
    echo "FAIL: script not found: $SCRIPT_UNDER_TEST" >&2
    exit 1
fi

# --- mock gh: возвращает [] для issue list (empty input) -------------------
mkdir -p "$TMP_DIR/bin"
cat > "$TMP_DIR/bin/gh" <<'EOSHIM'
#!/usr/bin/env bash
# Mock gh для test_merge_gate_logging.sh: empty input case (нет issues).
# Покрывает основные subcommand'ы merge-gate: issue, pr, api, label.
case "$1" in
    auth) exit 0 ;;
    issue)
        case "$2" in
            list)     printf '[]\n'; exit 0 ;;
            edit)     exit 0 ;;
            view)     printf '{"comments":[],"body":"","labels":[],"title":""}'; exit 0 ;;
            comment)  exit 0 ;;
            close)    exit 0 ;;
        esac
        ;;
    api)
        # rate_limit — спецкейс (GH возвращает resources.*).
        if echo "$*" | grep -q "rate_limit"; then
            printf '{"resources":{"core":{"remaining":999},"graphql":{"remaining":999}}}\n'
            exit 0
        fi
        # Все остальные api-вызовы merge-gate → пустой JSON (нет issues/PRs/etc).
        printf '[]\n'
        exit 0
        ;;
    pr)
        case "$2" in
            list)   printf '[]\n'; exit 0 ;;
            view)   printf '{}'; exit 0 ;;
            edit)   exit 0 ;;
            checks) printf '[]'; exit 0 ;;
            comment) exit 0 ;;
        esac
        ;;
    label) exit 0 ;;
esac
exit 0
EOSHIM
chmod +x "$TMP_DIR/bin/gh"

# --- mock git: для MAINTENANCE gate и flock ---------------------------------
mkdir -p "$TMP_DIR/git-bin"
cat > "$TMP_DIR/git-bin/git" <<'EOSHIM'
#!/usr/bin/env bash
# Mock git: ls-remote пустой (MAINTENANCE flag НЕ установлен),
# show с отсутствующим путём → exit 128 (имитация real-git behavior).
# ВАЖНО: для всего остального — exit 0 (no-op), чтобы merge-gate не падал
# на других git-вызовах (branch --contains, log, fetch, etc).
#
# Git принимает флаги ДО subcommand (`-C <path>`, `--git-dir=<p>`, ...).
# Парсим args чтобы найти subcommand:
#   - Флаги начинающиеся с '-' → пропускаем
#   - Для флагов с аргументом (-C, --git-dir) → пропускаем следующий arg тоже
_subcmd=""
_skip_next=0
for _arg in "$@"; do
    if [ "$_skip_next" = "1" ]; then _skip_next=0; continue; fi
    case "$_arg" in
        -C|--git-dir|--work-tree|-c)
            # Флаги с обязательным аргументом — пропускаем arg.
            _skip_next=1
            ;;
        -*) ;;  # прочие флаги без аргумента
        *)
            _subcmd="$_arg"
            break
            ;;
    esac
done
case "$_subcmd" in
    ls-remote)
        # Нет MAINTENANCE flag → пустой stdout
        exit 0
        ;;
    show)
        # Эмулируем failure для несуществующего пути в дереве — как
        # настоящий git (`fatal: path 'X' does not exist in 'Y'`).
        # Но только если в args есть ':' (ref:path формат) и путь — это
        # 'MAINTENANCE'. Для нормальных show — exit 0.
        for arg in "$@"; do
            case "$arg" in
                *:MAINTENANCE)
                    echo "fatal: path 'MAINTENANCE' does not exist" >&2
                    exit 128
                    ;;
            esac
        done
        exit 0
        ;;
    *) exit 0 ;;
esac
EOSHIM
chmod +x "$TMP_DIR/git-bin/git"

# --- запуск ------------------------------------------------------------------
# Используем настоящий REPO_DIR из репо, чтобы merge-gate мог пройти
# MAINTENANCE gate (git ls-remote → empty → continue).
LOCK_FILE="$TMP_DIR/lock"
LOG_DIR="$TMP_DIR/logs"
mkdir -p "$LOG_DIR"

# Merge-gate + env для теста (lock изолирован, log-dir временный).
PATH="$TMP_DIR/bin:$TMP_DIR/git-bin:$PATH" \
    REPO_DIR="$REPO_ROOT_HERE" \
    GH_REPO="krikz/test-repo" \
    KANBAN_BOARD="robbox" \
    HOME="$TMP_DIR/home" \
    GH_CONFIG_DIR="$TMP_DIR/gh-config" \
    LOCK_FILE="$LOCK_FILE" \
    MERGE_GATE_TICK_LOG_DIR="$LOG_DIR" \
    bash "$SCRIPT_UNDER_TEST" \
        >"$TMP_DIR/stdout.log" 2>"$TMP_DIR/stderr.log" \
    &
    PID=$!

# Ждём максимум 30 секунд (типичный merge-gate при пустом input = ~5-10 сек)
TIMEOUT=30
ELAPSED=0
while kill -0 "$PID" 2>/dev/null; do
    if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
        echo "FAIL: test exceeded ${TIMEOUT}s timeout — killing PID=$PID" >&2
        kill "$PID" 2>/dev/null || true
        wait "$PID" 2>/dev/null || true
        exit 1
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done
wait "$PID" 2>/dev/null || EXIT_CODE=$? || true
EXIT_CODE="${EXIT_CODE:-0}"

echo "test_merge_gate_logging: exit=${EXIT_CODE}"
echo "---stdout ($(wc -l < "$TMP_DIR/stdout.log") lines)---"
cat "$TMP_DIR/stdout.log"
echo "---stderr ($(wc -l < "$TMP_DIR/stderr.log") lines)---"
head -5 "$TMP_DIR/stderr.log" || true

# --- assertions --------------------------------------------------------------
FAIL=0

# 1. Exit code = 0 (skip-tick — норма)
if [ "$EXIT_CODE" -ne 0 ]; then
    echo "FAIL: expected exit 0, got $EXIT_CODE" >&2
    FAIL=1
fi

# 2. Stdout содержит tick-start marker
if ! grep -q "# TICK_SUMMARY: start" "$TMP_DIR/stdout.log"; then
    echo "FAIL: stdout missing '# TICK_SUMMARY: start'" >&2
    echo "  (это ИМЕННО silent-bug: cron видит 'silent (empty output)')" >&2
    FAIL=1
fi

# 3. Stdout содержит tick-end marker
if ! grep -q "# TICK_SUMMARY: end" "$TMP_DIR/stdout.log"; then
    echo "FAIL: stdout missing '# TICK_SUMMARY: end'" >&2
    FAIL=1
fi

# 4. Stdout не пустой (root cause теста — silent означало empty stdout)
if [ ! -s "$TMP_DIR/stdout.log" ]; then
    echo "FAIL: stdout empty (silent bug regression)" >&2
    FAIL=1
fi

# 5. Per-day log-файл создан
LOG_FILE="$LOG_DIR/$(date -u +%Y-%m-%d).log"
if [ ! -f "$LOG_FILE" ]; then
    echo "FAIL: log file not created: $LOG_FILE" >&2
    FAIL=1
else
    # 5a. Log-файл содержит tick-start
    if ! grep -q "# TICK_SUMMARY: start" "$LOG_FILE"; then
        echo "FAIL: log file missing tick-start: $LOG_FILE" >&2
        FAIL=1
    fi
    # 5b. Log-файл содержит tick-end
    if ! grep -q "# TICK_SUMMARY: end" "$LOG_FILE"; then
        echo "FAIL: log file missing tick-end: $LOG_FILE" >&2
        FAIL=1
    fi
fi

if [ "$FAIL" -eq 0 ]; then
    echo ""
    echo "PASS: tick-summary logging works (ADR-0079 / issue #1977 fixed)"
    exit 0
else
    echo "FAIL: ${FAIL} assertion(s) failed" >&2
    exit 1
fi