#!/usr/bin/env bash
# Unit-тест для when_robot_asked_matches() — issue #2809 (E2E-харнесс
# переспроса личности, follow-up PR #2818).
#
# Зачем: новый акт night_marathon_act2b_identity_question отвечает на
# вопрос-гипотезу робота ("<Имя>, это ты?") только если он реально прозвучал
# на ПРЕДЫДУЩЕМ шаге. Без явного pure-теста контракт "grep -E, регистро-
# независимо, ищем ТОЛЬКО в переданной строке (robot_speech() предыдущего
# шага, а не во всём логе)" легко сломать незаметно — например, забыть
# grep -i и получить ложный SKIP на "Саша, ЭТО ТЫ?" в верхнем регистре TTS.
#
# Тест офлайновый — робота не требует, sourc'ит e2e_voice_lib.sh напрямую
# (тот же приём, что test_e2e_voice_map_tts_voice.sh / test_e2e_voice_safe_label.sh).
#
# Запуск: bash scripts/testing/test_e2e_when_robot_asked.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"

if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; NC=$'\033[0m'
else
    RED=""; GRN=""; NC=""
fi

[ -f "$LIB" ] || { printf '❌ отсутствует: %s\n' "$LIB"; exit 1; }

# shellcheck disable=SC1090
source "$LIB" 2>/dev/null

if ! type when_robot_asked_matches >/dev/null 2>&1; then
    printf '%sFATAL: when_robot_asked_matches не определена после source %s%s\n' "$RED" "$LIB" "$NC"
    exit 2
fi

PASS=0
FAIL=0

# assert_match $desc $prev_speech $pattern
assert_match() {
    local desc="$1" prev="$2" pattern="$3"
    if when_robot_asked_matches "$prev" "$pattern"; then
        printf '%sOK%s   %s\n' "$GRN" "$NC" "$desc"
        PASS=$((PASS + 1))
    else
        printf '%sFAIL%s %s (ожидали match, prev=%q pattern=%q)\n' "$RED" "$NC" "$desc" "$prev" "$pattern"
        FAIL=$((FAIL + 1))
    fi
}

assert_no_match() {
    local desc="$1" prev="$2" pattern="$3"
    if when_robot_asked_matches "$prev" "$pattern"; then
        printf '%sFAIL%s %s (ожидали НЕ match, prev=%q pattern=%q)\n' "$RED" "$NC" "$desc" "$prev" "$pattern"
        FAIL=$((FAIL + 1))
    else
        printf '%sOK%s   %s\n' "$GRN" "$NC" "$desc"
        PASS=$((PASS + 1))
    fi
}

# --- CASE: базовое совпадение ------------------------------------------------
assert_match "single-вопрос с именем совпал" \
    "Саша, это ты?" \
    "Саш.*это ты|это ты.*Саш"

# --- CASE: регистронезависимость (issue #2809: TTS/лог может дать любой регистр) ---
assert_match "верхний регистр не мешает совпадению" \
    "САША, ЭТО ТЫ?" \
    "Саш.*это ты|это ты.*Саш"

# --- CASE: обратный порядок слов (альтернация "|") --------------------------
assert_match "альтернация ловит оба порядка слов" \
    "Это ты, Саша?" \
    "Саш.*это ты|это ты.*Саш"

# --- CASE: нет совпадения — робот сказал что-то другое -----------------------
assert_no_match "нейтральный ответ не считается вопросом" \
    "Привет! Чем могу помочь?" \
    "Саш.*это ты|это ты.*Саш"

# --- CASE: пустая речь (робот промолчал в предыдущем шаге) -------------------
assert_no_match "пустая речь предыдущего шага не совпадает" \
    "" \
    "Саш.*это ты|это ты.*Саш"

# --- CASE: пустой паттерн — намеренно НЕ считается match (issue #2809: иначе
# шаг без when_robot_asked ошибочно попал бы в условную ветку, если её вызвать
# по ошибке с пустой строкой). ------------------------------------------------
assert_no_match "пустой паттерн никогда не матчит (даже пустую речь)" \
    "Саша, это ты?" \
    ""

# --- CASE: contested-формулировка без имени ("как тебя зовут") --------------
assert_match "contested-вопрос без имени" \
    "Извини, а как тебя зовут?" \
    "как (тебя )?зовут|представиться|назов"

assert_no_match "contested-паттерн не матчит single-вопрос с именем" \
    "Саша, это ты?" \
    "как (тебя )?зовут|представиться|назов"

# --- CASE: речь предыдущего шага НЕ включает служебные строки биометрии -----
# (сам факт этого гарантирует robot_speech() в e2e_tool_match.py — здесь
# проверяем только что when_robot_asked_matches ищет РОВНО в переданной
# строке, а не залезает куда-то ещё сама).
assert_no_match "функция не видит ничего, кроме переданной строки" \
    "identify candidates: best='Саша' conf=0.81" \
    "Саш.*это ты|это ты.*Саш"

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
