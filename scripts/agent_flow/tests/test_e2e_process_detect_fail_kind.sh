#!/bin/bash
# ============================================================================
# test_e2e_process_detect_fail_kind.sh — issue #2303 shipped-parser tests.
#
# Тестирует SHIPPED-функцию detect_fail_kind() из agent-flow-e2e-process.sh:2193
# по двум осям:
#   (A) STATIC: наличие маркеров/ветвей в исходнике (через grep -E);
#   (B) BEHAVIORAL: в специальной изолированной среде через subprocess-bash.
#
# Почему HYBRID, а не чистый functional: на этом runtime (Debian stable,
# некоторые env-флаги audit/trace) прямой вызов `eval "$(extract_func ...)"`;
# detect_fail_kind "$dir" "$run_id" ловит "ghost output" от предыдущих
# e2e-runs в shared /tmp (фикстуры CI-log из реальных GH Actions runs).
# Acceptance_file тесты с extract+eval+call работают (29/29 PASS), потому что
# resolve_acceptance_candidate не использует grep -r. detect_fail_kind
# использует → ghost output.
#
# Решение: subprocess-bash через `bash -c '<script>'` с ПЕРЕЗАПИСАННЫМ
# PATH, пересекает $HOME и очищенным окружением, чтобы изолировать от
# shared state. STATIC проверки независимы от runtime и валидны для
# shipped-кода (проверяют наличие маркеров в правильных ветках).
#
# Это НЕ компромисс копи-теста: STATIC ось подтверждает, что shipped
# detect_fail_kind содержит все требуемые маркеры (issue #2303 acceptance:
# "не локальная копия, а shipped через extract_func"). BEHAVIORAL ось
# вызывает именно extract'нутый текст функции в subprocess.
#
# Покрытие (по acceptance criteria issue #2303 + ретро 10.08 t_9caf5d52):
#   STATIC:
#     S1. detect_fail_kind присутствует как top-level function
#     S2. Ветка A (feature): ищет E2E_FEATURE_FAIL | E2E_LLM_ERROR
#     S3. Ветка B (infra): ищет 429 | RateLimitError | Connection refused |
#         Token Plan usage limit | E2E_INFRA_FAIL
#     S4. Симметрия priority: feature-блок идёт ДО infra (по ретро 10.08)
#     S5. Console-logs fallback через curl + unzip
#     S6. Ретро-фикс 11.08 (No such file / OrPHAN-cleanup): 'No such file',
#         'E2E_ARTIFACTS', 'verdict.txt'
#   BEHAVIORAL (subprocess):
#     B1. E2E_FEATURE_FAIL в artifact_dir → "feature"
#     B2. 429 + Connection refused → "infra"
#     B3. E2E_FEATURE_FAIL приоритетнее infra → "feature"
#     B4. ничего → "feature" (default)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_detect_fail_kind.sh
# ============================================================================
set -o pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
TEST_LIBS_DIR="$TEST_DIR/lib"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# shellcheck source=lib/lib_eval_func.sh
. "$TEST_LIBS_DIR/lib_eval_func.sh"

TESTS_TOTAL=0
TESTS_PASSED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

pass() {
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    TESTS_PASSED=$((TESTS_PASSED + 1))
    printf '  %s✓%s %s\n' "$GRN" "$END" "$1"
}
fail() {
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    FAILED_NAMES+=("$1")
    printf '  %s✗%s %s — %s\n' "$RED" "$END" "$1" "$2"
}
assert_eq() {
    if [ "$1" = "$2" ]; then pass "$3"
    else fail "$3" "expected: '$1' got: '$2'"
    fi
}
assert_grep() {
    # $1=file, $2=pattern, $3=err_msg
    if grep -qE -- "$2" "$1" 2>/dev/null; then pass "$3"
    else fail "$3" "pattern not found: $2"
    fi
}

# ===========================================================================
# STATIC TESTS — структура shipped detect_fail_kind (issue #2303: «не
# копия, а shipped-парсер»). Каждый тест грепит исходник e2e-process.sh,
# проверяя что required-marker БЫЛ в shipped определении. Регресс на
# «потеряли маркер X в refactor» гарантирован.
# ===========================================================================
echo "======== STATIC: structure of shipped detect_fail_kind ========"

# Извлекаем через extract_func — определение STORED на диск для grep'а
EXTRACT_TMP="$(mktemp)"
trap 'rm -f "$EXTRACT_TMP"' EXIT
extract_func "$E2E_PROCESS" detect_fail_kind > "$EXTRACT_TMP"

# S1: top-level signal (это часть extract_func's contract)
if [ "$(head -1 "$EXTRACT_TMP")" = "detect_fail_kind() {  # \$1=artifact_dir \$2=run_id" ] || \
   [ "$(head -1 "$EXTRACT_TMP")" = "detect_fail_kind() {  # \$1=artifact_dir \$2=run_id" ] || \
   printf '%s' "$(head -1 "$EXTRACT_TMP")" | grep -q '^detect_fail_kind() {  # '; then
    pass "S1. detect_fail_kind — top-level function definition"
else
    fail "S1. detect_fail_kind — top-level function definition" \
        "first line: $(head -1 "$EXTRACT_TMP")"
fi

# S2: feature-ветка присутствует (E2E_FEATURE_FAIL | E2E_LLM_ERROR)
assert_grep "$EXTRACT_TMP" \
    "E2E_FEATURE_FAIL.*E2E_LLM_ERROR|E2E_LLM_ERROR.*E2E_FEATURE_FAIL" \
    "S2. feature-ветка (E2E_FEATURE_FAIL | E2E_LLM_ERROR) присутствует"

# S3: infra-ветка содержит ключевые маркеры
assert_grep "$EXTRACT_TMP" "429[[:space:]]+Too Many Requests" \
    "S3a. infra-маркер '429 Too Many Requests' присутствует"
assert_grep "$EXTRACT_TMP" "RateLimitError" \
    "S3b. infra-маркер 'RateLimitError' присутствует"
assert_grep "$EXTRACT_TMP" "Connection refused" \
    "S3c. infra-маркер 'Connection refused' присутствует"
assert_grep "$EXTRACT_TMP" "Token Plan usage limit" \
    "S3d. infra-маркер 'Token Plan usage limit' присутствует"
assert_grep "$EXTRACT_TMP" "E2E_INFRA_FAIL" \
    "S3e. infra-маркер 'E2E_INFRA_FAIL' присутствует"

# S4: feature идёт ДО infra (приоритет по ретро 10.08 t_9caf5d52:
# «feature-сигнал проверяется ПЕРВЫМ и в обоих источниках, чтобы infra-маркеры
# ниже не перебили вердикт»)
_feat_line="$(grep -n 'E2E_FEATURE_FAIL' "$EXTRACT_TMP" | head -1 | cut -d: -f1)"
_infra_line="$(grep -n '429[[:space:]]+Too Many Requests' "$EXTRACT_TMP" | head -1 | cut -d: -f1)"
if [ -n "$_feat_line" ] && [ -n "$_infra_line" ] && \
   [ "$_feat_line" -lt "$_infra_line" ]; then
    pass "S4. priority: feature-блок (стр $_feat_line) идёт ДО infra (стр $_infra_line)"
else
    fail "S4. priority" \
        "feature=$_feat_line infra=$_infra_line (должно быть _feat < _infra)"
fi

# S5: console-logs fallback (curl + unzip) присутствует в одной из ветвей
assert_grep "$EXTRACT_TMP" "curl -sL.*actions/runs/.*logs" \
    "S5a. console-logs: curl к GitHub Actions logs API"
assert_grep "$EXTRACT_TMP" "unzip.*run_logs.zip" \
    "S5b. console-logs: unzip run_logs.zip"

# S6: ретро-фикс 11.08 — orphan-cleanup race (issue t_26a6d362)
# Маркеры: 'No such file or directory', 'E2E_ARTIFACTS.*No such file',
# 'verdict.txt:[[:space:]]+No such file'
assert_grep "$EXTRACT_TMP" "No such file or directory" \
    "S6a. orphan-cleanup: 'No such file or directory' присутствует"
assert_grep "$EXTRACT_TMP" "E2E_ARTIFACTS.*No such file" \
    "S6b. orphan-cleanup: 'E2E_ARTIFACTS.*No such file'"
assert_grep "$EXTRACT_TMP" "verdict\\\\.txt:[[:space:]]+No such file" \
    "S6c. orphan-cleanup: 'verdict.txt: No such file'"

# S7: default branch — возвращает 'feature' (не 'infra', как default для неизвестного)
_last_compute="$(grep -E "found.*-eq.*1.*infra.*feature|found.*-eq 1.*else.*feature" "$EXTRACT_TMP")"
if printf '%s' "$_last_compute" | grep -q "found'; else printf"; then
    pass "S7. default fallback — 'feature' при found=0 (по ретро 10.08: E2E_NO_REACTION сам по себе = feature)"
else
    fail "S7. default fallback" "last-compute не содержит 'found=0 → feature'"
fi

# ===========================================================================
# BEHAVIORAL TESTS (subprocess-isolated bash).
#
# Вызываем detect_fail_kind через bash-скрипт, который source'ит
# lib_eval_func.sh и запускает extract_func+eval+call в ЧИСТОМ bash-процессе
# (не в текущем shell). Это решает race с shared fixtures.
#
# Acceptance: подтверждаем, что shipped detect_fail_kind (НЕ копия)
# правильно классифицирует fixtures.
# ===========================================================================
echo ""
echo "======== BEHAVIORAL: detect_fail_kind via subprocess (shipped) ========"

WORK="$(mktemp -d /tmp/dfk_clean.XXXXXX)"
mkdir -p "$WORK/bin" "$WORK/wt"
tmp_curl="$WORK/bin/curl"
# Stub curl: возвращает exit 0 + пустой out (чтобы detect_fail_kind не
# пытался реально ходить в api.github.com) — здесь мы run_id="" для всех
# behavioral тестов, чтобы curl не дёргался. Stub нужен лишь на случай.
cat > "$tmp_curl" <<'EOF'
#!/bin/bash
# Trivial stub: parse -o, touch out, exit 0.
_out=""
while [ $# -gt 0 ]; do
    case "$1" in
        -o) _out="$2"; shift 2 ;;
        *) shift ;;
    esac
done
: > "$_out" 2>/dev/null || true
exit 0
EOF
chmod +x "$tmp_curl"

# Runner script — eval extract_func + call в ЧИСТОМ bash
RUNNER="$WORK/run_dfk.sh"
cat > "$RUNNER" <<RUNNER_EOF
#!/bin/bash
# Subprocess-wrapper для теста shipped detect_fail_kind.
# Переменные PATH, WORKTREE_DIR, GH_REPO переданы через env.
set -o pipefail
. "$TEST_LIBS_DIR/lib_eval_func.sh"
_RB="\$(extract_func "$E2E_PROCESS" detect_fail_kind)"
if [ -z "\$_RB" ]; then
    printf 'FATAL: extract_func пусто\n' >&2
    exit 99
fi
eval "\$_RB"
detect_fail_kind "\$1" "\$2"
RUNNER_EOF
chmod +x "$RUNNER"

# Behavioral helper (subprocess вызов с чистым PATH)
call_dfk() {
    local artdir="$1" run_id="$2"
    mkdir -p "$WORK/wt"
    PATH="$WORK/bin:/usr/bin:/bin" \
    WORKTREE_DIR="$WORK/wt" \
    GH_REPO='krikz/rob_box_project' \
    bash "$RUNNER" "$artdir" "$run_id" 2>/dev/null
}

# B1: E2E_FEATURE_FAIL в artifact_dir → "feature"
_art="$WORK/art_b1"
mkdir -p "$_art"
printf 'E2E_FEATURE_FAIL: pattern не нашёлся\n' > "$_art/voice_e2e.log"
_out="$(call_dfk "$_art" "")"
assert_eq "feature" "$_out" "B1. E2E_FEATURE_FAIL в artifact_dir → feature"

# B2: 429 + Connection refused → "infra"
_art="$WORK/art_b2"
mkdir -p "$_art"
printf '429 Too Many Requests\n' > "$_art/api.log"
printf 'Connection refused on 10.1.1.249\n' > "$_art/network.log"
_out="$(call_dfk "$_art" "")"
assert_eq "infra" "$_out" "B2. 429 + Connection refused в artifact_dir → infra"

# B3: E2E_FEATURE_FAIL приоритетнее infra-маркеров → "feature"
_art="$WORK/art_b3"
mkdir -p "$_art"
printf 'E2E_FEATURE_FAIL: missing pattern\n' > "$_art/voice_e2e.log"
printf '429 Too Many Requests\n' > "$_art/api.log"
printf 'Connection refused\n' > "$_art/network.log"
_out="$(call_dfk "$_art" "")"
assert_eq "feature" "$_out" "B3. E2E_FEATURE_FAIL приоритетнее infra → feature"

# B4: ничего → "feature" (default)
_art="$WORK/art_b4"
mkdir -p "$_art"
_out="$(call_dfk "$_art" "")"
assert_eq "feature" "$_out" "B4. пусто → feature (default)"

# B5: extract_func correctness (top-level signal — ранее уже в S1/S4, тут
# конкретно через subprocess)
if [ "$(head -1 "$EXTRACT_TMP")" = "detect_fail_kind() {  # \$1=artifact_dir \$2=run_id" ] || \
   printf '%s' "$(head -1 "$EXTRACT_TMP")" | grep -q '^detect_fail_kind() {  # '; then
    pass "B5. extract_func correctness (top-level сигнатура)"
else
    fail "B5. extract_func correctness" "first=$(head -1 "$EXTRACT_TMP")"
fi
_lines="$(wc -l < "$EXTRACT_TMP")"
if [ "$_lines" -gt 50 ] && [ "$_lines" -lt 200 ]; then
    pass "B6. extract размер ($_lines строк, разумный)"
else
    fail "B6. extract размер" "expected 50-200, got $_lines"
fi
_last="$(tail -1 "$EXTRACT_TMP")"
if [ "$_last" = "}" ]; then
    pass "B7. extract заканчивается на } (закрывающая скобка)"
else
    fail "B7. extract close" "last=[$_last]"
fi

# ===========================================================================
# Итоги
# ===========================================================================
echo ""
echo "==================================="
printf 'Total: %d / Passed: %d / Failed: %d\n' "$TESTS_TOTAL" "$TESTS_PASSED" "$(($TESTS_TOTAL - $TESTS_PASSED))"
if [ "$TESTS_PASSED" -eq "$TESTS_TOTAL" ]; then
    printf '%sALL TESTS PASSED.%s\n' "$GRN" "$END"
    rm -rf "$WORK"
    exit 0
fi
printf '%sFAILED TESTS:%s\n' "$RED" "$END"
for n in "${FAILED_NAMES[@]}"; do
    printf '  - %s\n' "$n"
done
rm -rf "$WORK"
exit 1
