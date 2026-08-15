#!/bin/bash
# ============================================================================
# test_e2e_process_acceptance_check_quoting.sh — ретро 15.08 (round-120)
#
# Regression test for the e2e-process ## e2e parser quote-stripping bug.
#
# Проблема (issue #1331, run 31884957397): PR body содержал
#   acceptance_check: "музык"        ← пробел ПОСЛЕ закрывающей кавычки
# Парсер делал `s/^"//; s/"$//` — снимал открывающую кавычку, но НЕ закрывающую
# (строка заканчивалась пробелом, не кавычкой). В workflow уезжало
# `acceptance_check=музык" ` → сгенерированный скрипт содержал
#   PATTERNS_VAL="музык" "
# → bash: unexpected EOF while looking for matching `"' (шаг E2E не стартовал).
#
# Фикс: в sed-цепочку добавлен `s/[[:space:]]*$//` ПЕРЕД `s/"$//` — сначала
# срезаем хвостовые пробелы, потом закрывающую кавычку.
#
# Тест читает РЕАЛЬНУЮ sed-цепочку из agent-flow-e2e-process.sh (не копию),
# чтобы не разойтись с продакшн-скриптом.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_acceptance_check_quoting.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

E2E_PROCESS="$REPO_ROOT/agent-flow-e2e-process.sh"

# Извлекаем sed-цепочку для acceptance_check прямо из живого скрипта.
# В скрипте строка вида:
#   e2e_acceptance_check="$(... | sed -E 's/...//; s/^"//; s/[[:space:]]*$//; s/"$//' || true)"
# Вырезаем содержимое между sed -E ' и ' (последняя одинарная кавычка перед || true).
extract_acceptance_sed() {
    local line
    line="$(grep -E 'e2e_acceptance_check=.*sed -E' "$E2E_PROCESS" | head -1 || true)"
    [ -n "$line" ] || { echo "ERROR: acceptance_check sed not found in $E2E_PROCESS" >&2; return 1; }
    # sed -E '<chain>' || true)
    printf '%s' "$line" | sed -E "s/.*sed -E '([^']*)'.*/\1/"
}

SED_CHAIN="$(extract_acceptance_sed)"

parse_field() {  # $1 = body text; echoes parsed acceptance_check value
    printf '%s\n' "$1" | grep -iE '^[[:space:]]*acceptance_check[[:space:]]*:' | head -1 | sed -E "$SED_CHAIN" || true
}

test_A_trailing_space_after_closing_quote() {
    # Прямое воспроизведение бага round-120: `"музык" ` (пробел после кавычки).
    local val
    val="$(parse_field $'## e2e\nacceptance_check: "музык" ')"
    assert_eq "музык" "$val" "trailing space after closing quote → clean value"
}

test_B_normal_quoted_value() {
    local val
    val="$(parse_field $'## e2e\nacceptance_check: "музык"')"
    assert_eq "музык" "$val" "normal quoted value still parses"
}

test_C_unquoted_value() {
    local val
    val="$(parse_field $'## e2e\nacceptance_check: музык')"
    assert_eq "музык" "$val" "unquoted value still parses"
}

test_D_multiword_pattern_with_trailing_space() {
    local val
    val="$(parse_field $'## e2e\nacceptance_check: "issue 1077" ')"
    assert_eq "issue 1077" "$val" "multiword pattern + trailing space → clean"
}

test_E_generated_script_is_valid_bash() {
    # Итоговая проверка: значение из body не должно ломать bash-скрипт workflow.
    local val script
    val="$(parse_field $'## e2e\nacceptance_check: "музык" ')"
    script="PATTERNS_VAL=\"$val\"
[ -n \"\$PATTERNS_VAL\" ] && CMD=\"\$CMD --patterns '\$PATTERNS_VAL'\"
echo \"RUN_CMD: \$CMD\""
    if ! printf '%s\n' "$script" | bash -n 2>/dev/null; then
        echo "FAIL: script invalid for value [$val]" >&2
        return 1
    fi
    local out
    out="$(printf '%s\n' "$script" | bash 2>/dev/null || true)"
    assert_contains "--patterns 'музык'" "$out" "workflow CMD contains clean pattern"
}

test_F_sed_chain_contains_whitespace_trim() {
    # Защита от регрессии: цепочка обязана срезать хвостовые пробелы до кавычки.
    assert_contains 's/[[:space:]]*$//' "$SED_CHAIN" "sed chain trims trailing whitespace before closing quote"
}

# ===========================================================================
# Run all tests.
# ===========================================================================
echo "Using sed chain from $E2E_PROCESS:"
echo "  $SED_CHAIN"
echo ""
run_test "A. trailing space after closing quote" test_A_trailing_space_after_closing_quote
run_test "B. normal quoted value" test_B_normal_quoted_value
run_test "C. unquoted value" test_C_unquoted_value
run_test "D. multiword pattern + trailing space" test_D_multiword_pattern_with_trailing_space
run_test "E. generated script is valid bash" test_E_generated_script_is_valid_bash
run_test "F. sed chain has whitespace trim" test_F_sed_chain_contains_whitespace_trim

summary
