#!/usr/bin/env bash
# ============================================================================
# test_validate_honesty.sh — регресс-тест validate_honesty.sh (ADR-0018).
#
# Покрывает acceptance:
#   - clean PR (без claim-маркеров или с evidence) → exit 0, без WARN/CRITICAL
#   - critical (closes # / [x] / fixes #) без evidence → CRITICAL + exit 1
#   - major (проверил/PASS/✅/done/fixed) без evidence → WARN, exit 0
#   - minor (вроде работает) без evidence → silent, exit 0
#   - raw-evidence (pytest / docker logs / gh run view / git log / sqlite /
#     code-block) оправдывает ВСЕ классы (включая critical)
#   - --strict: voice-claim → exit 1 (major → exit 1 в strict mode)
#   - --verbose: печатает info-маркеры (minor hits)
#   - `gh pr view <N>` не нужен, если передан --file / stdin
#   - пустой ввод (только пробелы) → clean
#
# Стратегия: пишем фикстуры во временный каталог, дёргаем скрипт,
# проверяем exit + stdout/stderr. Никаких внешних зависимостей.
#
# Run:
#   bash scripts/agent_flow/tests/test_validate_honesty.sh
# ============================================================================

set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"
TARGET="$ROOT_DIR/validate_honesty.sh"

if [ ! -x "$TARGET" ]; then
    echo "FAIL: $TARGET not executable" >&2
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Вспомогалки.
pass_count=0
fail_count=0
fail_log=""

# run_case: запускает с дефолтными флагами (без --strict, без --verbose).
# Проверяет:
#   - exit == expected_exit
#   - WARN count (>= expected_warn, если expected_warn > 0)
#   - CRITICAL count (>= expected_critical, если expected_critical > 0)
run_case() {
    local name="$1"
    local content="$2"
    local expected_warn="$3"
    local expected_exit="$4"
    local expected_critical="${5:-0}"
    local f="$WORK/case.txt"
    printf '%s\n' "$content" > "$f"

    local err rc
    bash "$TARGET" --file "$f" >/dev/null 2>/tmp/case.err
    rc=$?
    err="$(cat /tmp/case.err)"

    local got_warn=0
    local got_critical=0
    if [ -n "$err" ]; then
        got_warn="$(printf '%s' "$err" | grep -c '^WARN:')"
        got_critical="$(printf '%s' "$err" | grep -c '^CRITICAL:')"
    fi

    if [ "$rc" -ne "$expected_exit" ]; then
        fail_count=$((fail_count + 1))
        fail_log="${fail_log}FAIL [$name]: expected exit $expected_exit, got $rc
  stderr: $err
"
        return
    fi

    if [ "$expected_warn" -gt 0 ] && [ "$got_warn" -lt "$expected_warn" ]; then
        fail_count=$((fail_count + 1))
        fail_log="${fail_log}FAIL [$name]: expected >=$expected_warn WARN, got $got_warn
  stderr: $err
"
        return
    fi

    if [ "$expected_critical" -gt 0 ] && [ "$got_critical" -lt "$expected_critical" ]; then
        fail_count=$((fail_count + 1))
        fail_log="${fail_log}FAIL [$name]: expected >=$expected_critical CRITICAL, got $got_critical
  stderr: $err
"
        return
    fi

    pass_count=$((pass_count + 1))
    echo "PASS [$name] (exit=$rc warn=$got_warn critical=$got_critical)"
}

# --- Сценарии ---

# 1. Clean PR: только описание, без claim-маркеров.
run_case "clean-description" \
    "Что делает: добавляет helper для edge case.
Поведение: при X → Y, иначе Z.
Тесты: tests/test_module.py::test_edge." \
    0 0 0

# 2. Голословный «проверил, работает» без evidence → WARN (major, exit 0).
run_case "voice-claim-no-evidence" \
    "Проверил локально, всё работает. Готово к мержу." \
    1 0 0

# 3. Raw-evidence оправдывает claim-маркеры → clean.
run_case "raw-evidence-absolves" \
    "Проверил. Вот вывод:
pytest -v tests/test_x.py::test_a ... PASSED
docker logs robot_voice:
  2026-08-18 17:00:00 robot: hello
  2026-08-18 17:00:01 robot: ready
gh run view 12345678 → success" \
    0 0 0

# 4. ✅ / PASS / done без evidence → WARN (major, exit 0).
#    Это уже не critical (нет closes/[x]), даже если воркер ставит e2e-done.
run_case "pass-checkmark-no-evidence" \
    "Voice test PASS.
e2e-done поставлен.
Можно мержить." \
    1 0 0

# 5. `closes #N` без evidence → CRITICAL (soft-fail, exit 1).
run_case "closes-without-evidence-critical" \
    "closes #1397
Fixed by changing the if-else branch.
Ready." \
    1 1 1

# 6. `closes #N` + pytest output → clean (evidence оправдывает critical).
run_case "closes-with-evidence" \
    "closes #1397
\`
pytest -v tests/test_fix.py::test_regression ... PASSED
\`" \
    0 0 0

# 7. Только `готово` без evidence → WARN (major).
run_case "gotovo-only" \
    "Сделано. Готово." \
    1 0 0

# 8. Пустой ввод (только пробелы) → clean.
run_case "empty-input" \
    "   " \
    0 0 0

# 9. Mixed: один блок с evidence, второй голословный → ВСЁ РАВНО clean
#    (есть evidence в тексте → оправдывает все claim-маркеры).
run_case "mixed-evidence-and-claim" \
    "Сделал.
docker logs robot_voice:
  ready
А ещё проверил локально, всё работает." \
    0 0 0

# 10. --strict: voice-claim (major) → exit 1.
STRICT_FILE="$WORK/strict.txt"
printf 'Проверил, работает.\n' > "$STRICT_FILE"
bash "$TARGET" --strict --file "$STRICT_FILE" >/dev/null 2>/tmp/strict.err
strict_rc=$?
strict_warn="$(grep -c '^WARN:' /tmp/strict.err || true)"
if [ "$strict_rc" -ne 1 ] || [ "${strict_warn:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [strict-mode]: expected exit 1 with >=1 WARN, got exit=$strict_rc warn=${strict_warn:-0}
  stderr: $(cat /tmp/strict.err)
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [strict-mode] (exit=$strict_rc warn=$strict_warn)"
fi

# 11. Английский голословный: "it works" / "should work" → WARN (major).
run_case "english-claim-no-evidence" \
    "it works locally, should work in prod. Fixed." \
    1 0 0

# 12. Code-fence (backticks) = evidence.
run_case "code-fence-evidence" \
    "Поправил, теперь работает.
\`
git log --oneline -3
74d25ae5 wip(process): ADR-0018
a8d1f251 wip(process): fix backtick
\`" \
    0 0 0

# 13. CRITICAL: `[x]` checkbox без evidence → CRITICAL + exit 1.
#     Только critical-паттерны, без major → WARN=0.
run_case "checkbox-without-evidence-critical" \
    "[X] Feature implemented
[x] Tests added
Пожалуйста, посмотрите." \
    0 1 1

# 14. CRITICAL: `fixes #N` без evidence → CRITICAL + exit 1.
#     Только critical-паттерны, без major → WARN=0.
run_case "fixes-without-evidence-critical" \
    "fixes #1397
Жду ревью." \
    0 1 1

# 15. CRITICAL + major mixed без evidence → CRITICAL + WARN, exit 1.
run_case "critical-plus-major" \
    "closes #1397
Проверил, работает." \
    1 1 1

# 16. MINOR: «вроде работает» без evidence → silent, exit 0, без WARN.
#     NB: формулировка должна быть ТОЛЬКО minor — НЕ содержать «работает»
#     отдельно (это major-паттерн) или `[x]` (critical).
MINOR_FILE="$WORK/minor.txt"
printf 'Вроде ок, но не уверен.\n' > "$MINOR_FILE"
bash "$TARGET" --file "$MINOR_FILE" >/dev/null 2>/tmp/minor.err
minor_rc=$?
minor_warn="$(grep -c '^WARN:' /tmp/minor.err || true)"
minor_critical="$(grep -c '^CRITICAL:' /tmp/minor.err || true)"
if [ "$minor_rc" -ne 0 ] || [ "${minor_warn:-0}" -ne 0 ] || [ "${minor_critical:-0}" -ne 0 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [minor-silent]: expected silent (exit=0 warn=0 critical=0), got exit=$minor_rc warn=${minor_warn:-0} critical=${minor_critical:-0}
  stderr: $(cat /tmp/minor.err)
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [minor-silent] (exit=$minor_rc warn=$minor_warn critical=$minor_critical)"
fi

# 17. MINOR + major: minor НЕ поднимается до warn, major поднимается.
#     «Вроде ок» (minor) + «проверил» (major) → 1 WARN, exit 0.
run_case "minor-plus-major-major-only" \
    "Вроде ок. Проверил локально." \
    1 0 0

# 18. --verbose: печатает info-маркеры для minor, но exit всё равно 0.
VERBOSE_FILE="$WORK/verbose.txt"
printf 'Вроде работает.\n' > "$VERBOSE_FILE"
bash "$TARGET" --verbose --file "$VERBOSE_FILE" >/dev/null 2>/tmp/verbose.err
verbose_rc=$?
verbose_info="$(grep -c '^info:' /tmp/verbose.err || true)"
if [ "$verbose_rc" -ne 0 ] || [ "${verbose_info:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [verbose-mode]: expected exit 0 with >=1 info line, got exit=$verbose_rc info=${verbose_info:-0}
  stderr: $(cat /tmp/verbose.err)
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [verbose-mode] (exit=$verbose_rc info=$verbose_info)"
fi

# 19. stdin (без --file): тоже работает.
stdin_content="closes #1397
Fixed."
stdin_rc=0
bash "$TARGET" >/dev/null 2>/tmp/stdin.err <<<"$stdin_content" || stdin_rc=$?
stdin_warn="$(grep -c '^WARN:' /tmp/stdin.err || true)"
stdin_critical="$(grep -c '^CRITICAL:' /tmp/stdin.err || true)"
if [ "$stdin_rc" -ne 1 ] || [ "${stdin_critical:-0}" -lt 1 ]; then
    fail_count=$((fail_count + 1))
    fail_log="${fail_log}FAIL [stdin]: expected exit=1 critical>=1 via stdin, got exit=$stdin_rc warn=${stdin_warn:-0} critical=${stdin_critical:-0}
  stderr: $(cat /tmp/stdin.err)
"
else
    pass_count=$((pass_count + 1))
    echo "PASS [stdin] (exit=$stdin_rc warn=$stdin_warn critical=$stdin_critical)"
fi

# 20. CRITICAL + evidence → clean (оправдано).
run_case "critical-with-evidence" \
    "closes #1397
\`\`\`
pytest -v tests/test_x.py::test_fix ... PASSED
\`\`\`" \
    0 0 0

# 21. `resolves #N` без evidence → CRITICAL + exit 1.
run_case "resolves-without-evidence-critical" \
    "resolves #1397
Готово." \
    1 1 1

# --- Итог ---
echo
echo "=== validate_honesty test summary ==="
echo "PASS: $pass_count"
echo "FAIL: $fail_count"
if [ "$fail_count" -gt 0 ]; then
    printf '%s' "$fail_log"
    exit 1
fi
echo "ALL OK"
exit 0
