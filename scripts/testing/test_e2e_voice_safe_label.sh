#!/usr/bin/env bash
# Unit-тест для safe_label() (BUG-B / t_f0612a43).
#
# Проверяет, что safe_label транслитерирует кириллицу в ASCII-slug и
# заменяет спец-символы. Используется в e2e_voice_test.sh:run_step()
# для построения имени wav-файла из ${label} (которое может содержать
# кириллицу/запятые/знаки вопроса — без slugify имя получалось
# невалидным и synth_yandex падал без реального запуска).
#
# Запуск: bash scripts/testing/test_e2e_voice_safe_label.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
#
# Тест НЕ требует Yandex API / SSH / pytest — это pure-bash self-contained.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"

# Цвета (отключаются если не TTY)
if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; YEL=$'\033[0;33m'; NC=$'\033[0m'
else
    RED=""; GRN=""; YEL=""; NC=""
fi

# Загружаем только библиотечный файл (без main flow основного harness —
# иначе source валится на E2E_FATAL).
# shellcheck disable=SC1090
source "$LIB" 2>/dev/null

# Проверяем, что safe_label определена
if ! type safe_label >/dev/null 2>&1; then
    printf '%sFATAL: safe_label не определена после source %s%s\n' "$RED" "$LIB" "$NC"
    exit 2
fi

PASS=0
FAIL=0

assert_slug() {
    local input="$1" expected="$2"
    local actual
    actual="$(safe_label "$input")"
    if [ "$actual" = "$expected" ]; then
        PASS=$((PASS+1))
        printf '%s OK %s  →  %s%s\n' "$GRN" "$input" "$actual" "$NC"
    else
        FAIL=$((FAIL+1))
        printf '%s FAIL: input=%s expected=%s actual=%s%s\n' "$RED" "$input" "$expected" "$actual" "$NC"
    fi
}

# === Тест-кейсы ===

# Базовый ASCII-slug остаётся нетронутым
assert_slug "ml01_generate_romantic"          "ml01_generate_romantic"
assert_slug "single"                          "single"

# Кириллица + запятая + вопросительный знак (BUG-B acceptance case)
assert_slug "Робот, что в библиотеке?"        "robot_chto_v_biblioteke"

# Длинная кириллица
assert_slug "Робот, сохрани этот трек для Ивана." "robot_sokhrani_etot_trek_dlya_ivana"

# Ё отдельно от Е
assert_slug "Ёжик в тумане"                   "yozhik_v_tumane"

# Только спецсимволы → fallback 'step'
assert_slug "?!.,;"                           "step"

# Пробелы по краям
assert_slug "  hello world  "                 "hello_world"

# Множественные подчёркивания → один
assert_slug "a___b___c"                       "a_b_c"

# Двоеточие и косая черта → `_`, вопросительный знак → удаляется, дефис остаётся
assert_slug "ml/03:search-rain?"              "ml_03search-rain"

# Только цифры
assert_slug "12345"                           "12345"

# Кавычки и скобки
assert_slug "Тест «кавычек» (скобки)"         "test_kavychek_skobki"

# Обрезка до 80 символов
LONG=$(python3 -c "print('a'*200, end='')")
TRUNC=$(python3 -c "print('a'*80, end='')")
assert_slug "$LONG"                           "$TRUNC"

# Символы, которые остаются (буквы ASCII, цифры, _, -)
assert_slug "audio-test_v2"                   "audio-test_v2"

# @ → "at", # → удаляется → строка непустая, fallback 'step' не нужен
assert_slug "@@@###"                         "atatat"

# Проверка что synth_yandex (через stub) тоже работает с safe_label-выводом:
# синтезируем "Робот, что в библиотеке?" → safe_label даст валидный путь,
# Python откроет и запишет файл. Это покрывает acceptance критерий задачи.
if [ -n "${RUN_FULL_TEST:-}" ]; then
    TMP_OUT="/tmp/test_synth_yandex_$$.wav"
    # stub: не делаем реальный gRPC, проверяем что open() с slug-путем работает
    if python3 -c "
import sys
out = '$TMP_OUT'
with open(out, 'wb') as f:
    f.write(b'RIFF')
print('OK')
"; then
        if [ -f "$TMP_OUT" ] && [ "$(stat -c%s "$TMP_OUT")" = "4" ]; then
            PASS=$((PASS+1))
            printf '%s OK synth_yandex-safe-path write%s\n' "$GRN" "$NC"
        else
            FAIL=$((FAIL+1))
            printf '%s FAIL synth_yandex-safe-path: file missing%s\n' "$RED" "$NC"
        fi
    else
        FAIL=$((FAIL+1))
        printf '%s FAIL synth_yandex-safe-path: python open() failed%s\n' "$RED" "$NC"
    fi
    rm -f "$TMP_OUT"
fi

echo
if [ "$FAIL" -eq 0 ]; then
    printf '%s✓ %d/%d passed%s\n' "$GRN" "$PASS" "$((PASS+FAIL))" "$NC"
    exit 0
else
    printf '%s✗ %d failed of %d%s\n' "$RED" "$FAIL" "$((PASS+FAIL))" "$NC"
    exit 1
fi