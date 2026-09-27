#!/usr/bin/env bash
# Регресс-тест для issue #2824: parse_scenario_to_tsv() + E2E_SCENARIO_ROW_READ
# (e2e_voice_lib.sh) должны разбирать РЕАЛЬНЫЕ сценарии ЧЕРЕЗ РЕАЛЬНЫЙ
# bash `read`-цикл — не через отдельную копию/реимплементацию.
#
# Что сломалось (issue #2824, prod-регресс после PR #2823, develop b2f5560e5,
# живой прогон 35851587044): scenario-цикл в e2e_voice_test.sh разбирал
# python-сгенерированный TSV через `IFS=$'\t' read -r ...`. \t относится к
# "IFS whitespace" НЕЗАВИСИМО от того, что явно записано в IFS — bash
# схлопывает подряд идущие разделители-whitespace и стрижёт пустые поля по
# краям СТРОКИ, даже когда IFS переопределён на один-единственный символ
# табуляции. У шага БЕЗ when_robot_asked это поле пустое — на выходе python
# оно просто пропадает из строки, все поля ПОСЛЕ него сдвигаются на одну
# позицию влево: required_question получает то, что должно быть в
# sleep_before_sec, а сам when_robot_asked — "0" (сдвинутое
# required_question). Непустой when_robot_asked="0" не совпадает ни с чем в
# речи предыдущего шага → КАЖДЫЙ обычный шаг уходил в SKIP robot_did_not_ask.
# Прогон 35851587044 (акт 2): steps=0/19, ни одна реплика не проиграна —
# сломаны были ВСЕ сценарии харнесса, не только новый акт "переспрос личности".
#
# Почему тесты #2823 это не поймали (см. PR #2823, task honesty-note):
# test_e2e_sleep_before_and_required_question.sh извлекал python-heredoc
# ИЗ e2e_voice_test.sh и прогонял его САМ ПО СЕБЕ, проверяя TSV-строку через
# `cut -f<N>` — то есть напрямую по позиции столбца в python-выводе,
# НИКОГДА не пропуская её через bash `read`. Баг жил РОВНО на границе
# python-вывод -> bash-read, и тест эту границу не пересекал ни разу.
# Этот тест закрывает именно её: он вызывает parse_scenario_to_tsv() (та же
# функция, что вызывает main flow) и разбирает результат ТЕМ ЖЕ
# `IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"`, что и main flow — оба места
# читают один и тот же код из e2e_voice_lib.sh, скопировать-и-разойтись
# невозможно физически.
#
# Тест офлайновый — робота не требует. Source'ит e2e_voice_lib.sh (единственный
# файл, безопасный для source вне главного скрипта).
#
# Запуск: bash scripts/testing/test_e2e_scenario_tsv_row.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"
ACT2="$REPO_ROOT/.github/e2e/scenarios/night/night_marathon_act2_acquaintance_v1.json"
ACT2B="$REPO_ROOT/.github/e2e/scenarios/night/night_marathon_act2b_identity_question_v1.json"

if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; NC=$'\033[0m'
else
    RED=""; GRN=""; NC=""
fi

for f in "$LIB" "$ACT2" "$ACT2B"; do
    [ -f "$f" ] || { printf '❌ отсутствует: %s\n' "$f"; exit 1; }
done

# shellcheck disable=SC1090
source "$LIB" 2>/dev/null

for fn in parse_scenario_to_tsv; do
    if ! type "$fn" >/dev/null 2>&1; then
        printf '%sFATAL: %s не определена после source %s%s\n' "$RED" "$fn" "$LIB" "$NC"
        exit 2
    fi
done
if [ -z "${E2E_SCENARIO_ROW_READ:-}" ]; then
    printf '%sFATAL: E2E_SCENARIO_ROW_READ не определена после source %s%s\n' "$RED" "$LIB" "$NC"
    exit 2
fi

PASS=0
FAIL=0
ok()  { printf '%sOK%s   %s\n' "$GRN" "$NC" "$1"; PASS=$((PASS + 1)); }
bad() { printf '%sFAIL%s %s\n' "$RED" "$NC" "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# =============================================================================
# CASE 1 — night_marathon_act2_acquaintance_v1.json: 20 обычных шагов, НИ ОДИН
# не задаёт when_robot_asked/required_question/sleep_before_sec. Это ровно
# та ситуация, что сломал регресс #2824 — каждый такой шаг должен пройти
# через реальный read-цикл с ПУСТЫМ when_robot_asked и required_question=0,
# а не унаследовать сдвинутые соседние значения.
# =============================================================================
PARSED2="$WORKDIR/act2.tsv"
parse_scenario_to_tsv "$ACT2" "$PARSED2"

expected_labels_2=(
    n201_sasha_intro_long n201b_sasha_backstory n201c_sasha_voice_note
    n202a_sasha_warmup n202b_sasha_warmup_hum n202c_sasha_warmup_guess
    n203_sasha_memory_tea n204_boris_intro_long n204b_boris_voice_note
    n204c_boris_voice_reason n205a_boris_warmup n205b_boris_warmup_spartak
    n205c_boris_warmup_tease n206_boris_memory n207_recall_sasha
    n209_recall_boris n210_grisha_no_name n211_who_do_you_know
    n212_session_reset_before_search n213_memory_search_tea_after_reset
)

act2_rows_seen=0
act2_bad_when=0
act2_bad_required=0
act2_bad_sleep=0
act2_label_mismatch=0
i=0
while IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"; do
    [ -z "$idx" ] && continue
    act2_rows_seen=$((act2_rows_seen + 1))
    want_label="${expected_labels_2[$i]:-<none>}"
    if [ "$label" != "$want_label" ]; then
        act2_label_mismatch=$((act2_label_mismatch + 1))
        bad "act2 строка $i: label='$label', ожидали '$want_label' (текст/label сдвинулись — сигнатура регресса #2824)"
    fi
    if [ -n "$when_robot_asked" ]; then
        act2_bad_when=$((act2_bad_when + 1))
        bad "act2 шаг '$label': when_robot_asked='$when_robot_asked', ожидали ПУСТО (сценарий его не задаёт — сигнатура регресса #2824: сюда приезжает сдвинутое required_question)"
    fi
    if [ "$required_question" != "0" ]; then
        act2_bad_required=$((act2_bad_required + 1))
        bad "act2 шаг '$label': required_question='$required_question', ожидали '0'"
    fi
    if [ "$sleep_before_sec" != "0.0" ]; then
        act2_bad_sleep=$((act2_bad_sleep + 1))
        bad "act2 шаг '$label': sleep_before_sec='$sleep_before_sec', ожидали '0.0'"
    fi
    i=$((i + 1))
done < "$PARSED2"

[ "$act2_rows_seen" -eq 20 ] \
    && ok "act2: прочитано 20 строк через реальный read-цикл" \
    || bad "act2: прочитано $act2_rows_seen строк, ожидали 20"
[ "$act2_label_mismatch" -eq 0 ] && ok "act2: все label на своих местах (текст/label не сдвинулись)"
[ "$act2_bad_when" -eq 0 ] && ok "act2: when_robot_asked пуст у ВСЕХ 20 шагов"
[ "$act2_bad_required" -eq 0 ] && ok "act2: required_question='0' у ВСЕХ 20 шагов"
[ "$act2_bad_sleep" -eq 0 ] && ok "act2: sleep_before_sec='0.0' у ВСЕХ 20 шагов"

# =============================================================================
# CASE 2 — night_marathon_act2b_identity_question_v1.json: смесь шагов с
# заполненными и пустыми новыми полями. Проверяем ТОЧНОЕ соответствие
# scenario.json (не просто "не пусто").
# =============================================================================
PARSED2B="$WORKDIR/act2b.tsv"
parse_scenario_to_tsv "$ACT2B" "$PARSED2B"

declare -A want_when=(
    [n701_sasha_intro]=""
    [n702_sasha_followup]=""
    [n703_sasha_confirm_yes]="Саш.*это ты|это ты.*Саш"
    [n704_sasha_no_repeat_question]=""
    [n705_sasha_after_gap]=""
    [n706_sasha_confirm_no]="Саш.*это ты|это ты.*Саш"
    [n707_sasha_after_no]=""
    [n708_boris_intro]=""
    [n709_gena_intro]=""
    [n710_gena_reply_if_asked]="как (тебя )?зовут|представиться|назов"
)
declare -A want_required=(
    [n701_sasha_intro]="0"
    [n702_sasha_followup]="0"
    [n703_sasha_confirm_yes]="1"
    [n704_sasha_no_repeat_question]="0"
    [n705_sasha_after_gap]="0"
    [n706_sasha_confirm_no]="1"
    [n707_sasha_after_no]="0"
    [n708_boris_intro]="0"
    [n709_gena_intro]="0"
    [n710_gena_reply_if_asked]="0"
)
declare -A want_sleep=(
    [n701_sasha_intro]="0.0"
    [n702_sasha_followup]="0.0"
    [n703_sasha_confirm_yes]="0.0"
    [n704_sasha_no_repeat_question]="0.0"
    [n705_sasha_after_gap]="150.0"
    [n706_sasha_confirm_no]="0.0"
    [n707_sasha_after_no]="0.0"
    [n708_boris_intro]="0.0"
    [n709_gena_intro]="0.0"
    [n710_gena_reply_if_asked]="0.0"
)

act2b_rows_seen=0
act2b_mismatches=0
while IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"; do
    [ -z "$idx" ] && continue
    act2b_rows_seen=$((act2b_rows_seen + 1))
    if [ -z "${want_when[$label]+x}" ]; then
        act2b_mismatches=$((act2b_mismatches + 1))
        bad "act2b: незнакомый label '$label' — сценарий поменялся, обнови ожидания теста"
        continue
    fi
    exp_when="${want_when[$label]}"
    exp_req="${want_required[$label]}"
    exp_sleep="${want_sleep[$label]}"
    if [ "$when_robot_asked" != "$exp_when" ]; then
        act2b_mismatches=$((act2b_mismatches + 1))
        bad "act2b шаг '$label': when_robot_asked='$when_robot_asked', ожидали '$exp_when'"
    fi
    if [ "$required_question" != "$exp_req" ]; then
        act2b_mismatches=$((act2b_mismatches + 1))
        bad "act2b шаг '$label': required_question='$required_question', ожидали '$exp_req'"
    fi
    if [ "$sleep_before_sec" != "$exp_sleep" ]; then
        act2b_mismatches=$((act2b_mismatches + 1))
        bad "act2b шаг '$label': sleep_before_sec='$sleep_before_sec', ожидали '$exp_sleep'"
    fi
done < "$PARSED2B"

[ "$act2b_rows_seen" -eq 10 ] \
    && ok "act2b: прочитано 10 строк через реальный read-цикл" \
    || bad "act2b: прочитано $act2b_rows_seen строк, ожидали 10"
[ "$act2b_mismatches" -eq 0 ] && ok "act2b: when_robot_asked/required_question/sleep_before_sec ТОЧНО совпадают с scenario.json для всех 10 шагов"

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
