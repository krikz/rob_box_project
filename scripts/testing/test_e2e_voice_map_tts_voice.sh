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
echo "== minimax: каст 26.10.2026 (#3992), voice:pitch:speed, замер на ЖИВОМ канале (колонка -> ReSpeaker -> speaker_id) =="
# Подобран по эмбеддингам: четыре голоса далеко друг от друга (худшая пара Саша-Борис: макс 0.76, p95 0.70, 2.7% пар
# записей выше порога опознания 0.72; остальные 0.66-0.74). Просодия 0/1.0 у всех (питч/темп вейк ломают, не разводят).
# Гриша и Валера - ЖЕНСКИЕ голоса (вариант Б Шифу): реплики в сценариях в женском роде. Док: docs/dialog/role_voices_biometrics_2026-10-26.md
assert_voice minimax anton "English_Jovialman:0:1.0"
assert_voice minimax ermil "English_WiseScholar:0:1.0"
assert_voice minimax madirus "English_WiseScholar:0:1.0" "алиас Бориса"
assert_voice minimax zahar "Chinese (Mandarin)_Gentle_Senior:0:1.0" "Гриша: женский пожилой"
assert_voice minimax filipp "English_Whispering_girl:0:1.0" "Валера: женский шёпот"
assert_voice minimax kostya "English_Whispering_girl:0:1.0" "алиас Валеры"
assert_voice minimax arina "Portuguese_StrictBoss:0:1.0" "запасной (вейк не мерили)"
assert_voice minimax "" "English_Jovialman:0:1.0" "пустой голос → дефолт провайдера (Саша)"
assert_voice minimax rita "Russian_ReliableMan" "rita: старые запасные склеивались с Гришей 0.82-0.84, имя ушло в голос по умолчанию"
assert_voice minimax Russian_CrazyQueen Russian_CrazyQueen "native голос не переводим"
assert_voice minimax Russian_ReliableMan:-2:0.9 Russian_ReliableMan:-2:0.9 "native со своей просодией не переводим"
assert_four_distinct minimax
# просодия у ролей нулевая: питч/темп не разводят голоса и ломают вейк «Робот»
for v in anton ermil zahar filipp; do
    case "$(map_tts_voice minimax "$v")" in
        *:0:1.0) PASS=$((PASS + 1)); printf '%sOK%s   minimax %-6s просодия 0/1.0\n' "$GRN" "$NC" "$v" ;;
        *) FAIL=$((FAIL + 1)); printf '%sFAIL%s minimax %s просодия не 0/1.0: %s\n' "$RED" "$NC" "$v" "$(map_tts_voice minimax "$v")" ;;
    esac
done
# запасной и голос по умолчанию для rita не совпадают ни с одной из четырёх ролей
for spare in arina rita; do
    sv="$(map_tts_voice minimax "$spare")"
    clash=0
    for v in anton ermil zahar filipp; do [ "$(map_tts_voice minimax "$v")" = "$sv" ] && clash=1; done
    if [ "$clash" = "0" ]; then PASS=$((PASS + 1)); printf '%sOK%s   minimax %-6s не совпадает с ролью (%s)\n' "$GRN" "$NC" "$spare" "$sv"
    else FAIL=$((FAIL + 1)); printf '%sFAIL%s minimax %s совпал с ролью (%s)\n' "$RED" "$NC" "$spare" "$sv"; fi
done
# имя с пробелом и скобками (Chinese (Mandarin)_...) доезжает до синтеза одним аргументом
sv="$(map_tts_voice minimax zahar)"
if [ "$(printf '%s' "$sv" | cut -d: -f1)" = "Chinese (Mandarin)_Gentle_Senior" ]; then
    PASS=$((PASS + 1)); printf '%sOK%s   minimax zahar    имя голоса с пробелом и скобками цело (первое поле до «:»)\n' "$GRN" "$NC"
else FAIL=$((FAIL + 1)); printf '%sFAIL%s minimax zahar: имя голоса разобралось неверно (%s)\n' "$RED" "$NC" "$sv"; fi

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
