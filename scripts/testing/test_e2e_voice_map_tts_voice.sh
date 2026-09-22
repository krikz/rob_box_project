#!/usr/bin/env bash
# Unit-тест для map_tts_voice() — таблицы «голос сценария → голос провайдера».
#
# Зачем: night-marathon 22.09.2026 (run 35667281570, акт 2) показал, что
# заявленная в комментарии «гарантированная различимость» четырёх голосов
# не выполнялась под minimax: anton (Russian_ReliableMan) и ermil
# (Russian_HandsomeChildhoodFriend) звучали для resemblyzer на cos=0.846,
# робот склеил Сашу и Бориса в один профиль. Таблицу пересобрали по
# замеру (scripts/e2e/measure_tts_voice_distinctness.py,
# evidence/tts-voice-distinctness-2026-09-22/), и этот тест держит
# результат замера: если кто-то вернёт старую пару, тест покраснеет и
# скажет, куда смотреть.
#
# Тест проверяет ДВА инварианта:
#   1. Четыре голоса сценария дают четыре РАЗНЫХ голоса провайдера
#      (иначе два персонажа зазвучат одним голосом — и диаризация
#      акта 3 не имеет смысла).
#   2. Конкретные значения совпадают с измеренными (числа — в
#      комментарии к map_tts_voice в e2e_voice_lib.sh).
#
# Запуск: bash scripts/testing/test_e2e_voice_map_tts_voice.sh
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

# shellcheck disable=SC1090
source "$LIB" 2>/dev/null

if ! type map_tts_voice >/dev/null 2>&1; then
    printf '%sFATAL: map_tts_voice не определена после source %s%s\n' "$RED" "$LIB" "$NC"
    exit 2
fi

PASS=0
FAIL=0

assert_voice() {
    local provider="$1" scenario_voice="$2" expected="$3" note="${4:-}"
    local actual
    actual="$(map_tts_voice "$provider" "$scenario_voice")"
    if [ "$actual" = "$expected" ]; then
        PASS=$((PASS + 1))
        printf '%sOK%s   %-8s %-8s -> %s\n' "$GRN" "$NC" "$provider" "$scenario_voice" "$actual"
    else
        FAIL=$((FAIL + 1))
        printf '%sFAIL%s %-8s %-8s -> %s (ожидалось %s) %s\n' \
            "$RED" "$NC" "$provider" "$scenario_voice" "$actual" "$expected" "$note"
    fi
}

assert_four_distinct() {
    local provider="$1"
    local voices=""
    local v
    for v in anton ermil zahar filipp; do
        voices="$voices$(map_tts_voice "$provider" "$v")"$'\n'
    done
    local uniq_count
    uniq_count="$(printf '%s' "$voices" | sort -u | grep -c .)"
    if [ "$uniq_count" = "4" ]; then
        PASS=$((PASS + 1))
        printf '%sOK%s   %-8s четыре голоса сценария → четыре разных голоса\n' \
            "$GRN" "$NC" "$provider"
    else
        FAIL=$((FAIL + 1))
        printf '%sFAIL%s %-8s четыре голоса сценария дали %s уникальных:\n%s\n' \
            "$RED" "$NC" "$provider" "$uniq_count" "$voices"
    fi
}

echo "== yandex: исходный каталог сценариев, перевод не нужен =="
assert_voice yandex anton anton
assert_voice yandex ermil ermil
assert_voice yandex "" anton "пустой голос → дефолт провайдера"
assert_four_distinct yandex

echo
echo "== minimax: таблица по замеру 22.09.2026, худшая пара 0.684 =="
# anton/ermil = 0.527 — именно эту пару регистрирует акт 2.
assert_voice minimax anton Russian_ReliableMan
assert_voice minimax ermil Russian_PessimisticGirl "старая таблица давала HandsomeChildhoodFriend: cos(anton,ermil)=0.739 (0.846 на роботе)"
assert_voice minimax zahar Russian_CrazyQueen
assert_voice minimax filipp Russian_AttractiveGuy
assert_voice minimax "" Russian_ReliableMan "пустой голос → дефолт провайдера"
assert_voice minimax Russian_CrazyQueen Russian_CrazyQueen "native голос не переводим"
assert_four_distinct minimax

echo
echo "== silero: zahar переехал с baya на kseniya, худшая пара 0.678 =="
assert_voice silero anton aidar
assert_voice silero ermil eugene
assert_voice silero zahar kseniya "baya давала cos(zahar,filipp)=0.722 и cos(zahar,alena)=0.788"
assert_voice silero filipp xenia
assert_voice silero alena baya
assert_voice silero aidar aidar "native голос не переводим"
assert_voice silero "" aidar "пустой голос → дефолт провайдера"
assert_four_distinct silero

echo
printf 'PASS=%s FAIL=%s\n' "$PASS" "$FAIL"
exit "$FAIL"
