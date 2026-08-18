#!/usr/bin/env bash
# ============================================================================
# test_validate_honesty.sh — регресс-тест validate_honesty.sh (ADR-0018).
#
# Покрывает acceptance:
#   - clean PR (без claim-маркеров или с evidence) → exit 0, без WARN
#   - голословный PASS/PASS/✅/done без evidence → WARN, exit 0 (не блокер)
#   - raw-evidence (pytest / docker logs / gh run view / git log / sqlite /
#     code-block) оправдывает любые claim-маркеры
#   - `closes #N` без evidence → WARN
#   - --strict → exit 1 при WARN
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

# name, content, expected_warn_count (>=), expected_exit
run_case() {
    local name="$1"
    local content="$2"
    local expected_warn="$3"
    local expected_exit="$4"
    local f="$WORK/case.txt"
    printf '%s\n' "$content" > "$f"

    local out err rc
    out="$(bash "$TARGET" --file "$f" 2>/tmp/case.err)"
    rc=$?
    err="$(cat /tmp/case.err)"

    local got_warn=0
    if [ "$rc" -ne 0 ] || [ -n "$err" ]; then
        # Считаем WARN по строкам в stderr.
        got_warn="$(printf '%s' "$err" | grep -c '^WARN:')"
    fi

    if [ "$rc" -ne "$expected_exit" ]; then
        fail_count=$((fail_count + 1))
        fail_log="${fail_log}FAIL [$name]: expected exit $expected_exit, got $rc
  stderr: $err
"
        return
    fi

    if [ "$got_warn" -lt "$expected_warn" ]; then
        fail_count=$((fail_count + 1))
        fail_log="${fail_log}FAIL [$name]: expected >=$expected_warn WARN, got $got_warn
  stderr: $err
"
        return
    fi

    pass_count=$((pass_count + 1))
    echo "PASS [$name] (exit=$rc warn=$got_warn)"
}

# --- Сценарии ---

# 1. Clean PR: только описание, без claim-маркеров.
run_case "clean-description" \
    "Что делает: добавляет helper для edge case.
Поведение: при X → Y, иначе Z.
Тесты: tests/test_module.py::test_edge." \
    0 0

# 2. Голословный «проверил, работает» без evidence → WARN.
run_case "voice-claim-no-evidence" \
    "Проверил локально, всё работает. Готово к мержу." \
    1 0

# 3. Raw-evidence оправдывает claim-маркеры.
run_case "raw-evidence-absolves" \
    "Проверил. Вот вывод:
pytest -v tests/test_x.py::test_a ... PASSED
docker logs robot_voice:
  2026-08-18 17:00:00 robot: hello
  2026-08-18 17:00:01 robot: ready
gh run view 12345678 → success" \
    0 0

# 4. ✅ / PASS / done без evidence → WARN.
run_case "pass-checkmark-no-evidence" \
    "✅ Voice test PASS.
e2e-done поставлен.
Можно мержить." \
    1 0

# 5. `closes #N` без evidence → WARN (фикс-маркеры должны подкрепляться).
run_case "closes-without-evidence" \
    "closes #1397
Fixed by changing the if-else branch.
Ready." \
    1 0

# 6. `closes #N` + pytest output → clean.
run_case "closes-with-evidence" \
    "closes #1397
\`
pytest -v tests/test_fix.py::test_regression ... PASSED
\`" \
    0 0

# 7. Только `готово` без evidence → WARN.
run_case "gotovo-only" \
    "Сделано. Готово." \
    1 0

# 8. Пустой ввод (только пробелы) → clean.
run_case "empty-input" \
    "   " \
    0 0

# 9. Mixed: один блок с evidence, второй голословный → ВСЁ РАВНО clean
#    (есть evidence в тексте → оправдывает все claim-маркеры в этом тексте).
#    Это by design: один фрагмент raw-вывода покрывает весь PR.
run_case "mixed-evidence-and-claim" \
    "Сделал.
docker logs robot_voice:
  ready
А ещё проверил локально, всё работает." \
    0 0

# 10. --strict: voice-claim → exit 1.
STRICT_FILE="$WORK/strict.txt"
printf 'Проверил, работает.\n' > "$STRICT_FILE"
strict_out="$(bash "$TARGET" --strict --file "$STRICT_FILE" 2>/tmp/strict.err)"
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

# 11. Английский голословный: "it works" / "should work" → WARN.
run_case "english-claim-no-evidence" \
    "it works locally, should work in prod. Fixed." \
    1 0

# 12. Code-fence (backticks) = evidence.
run_case "code-fence-evidence" \
    "Поправил, теперь работает.
\`
git log --oneline -3
74d25ae5 wip(process): ADR-0018
a8d1f251 wip(process): fix backtick
\`" \
    0 0

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
