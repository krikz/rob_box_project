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

# Сверка строк TSV с ожиданиями: $1 — tsv, $2 — имя случая, $3 — сколько строк ждём.
# Ожидания — ассоциативные массивы want_when/want_required/want_sleep (по label).
check_rows_exact() {
    local parsed="$1" name="$2" want_rows="$3" rows_seen=0 mismatches=0
    local exp_when exp_req exp_sleep
    while IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"; do
        [ -z "$idx" ] && continue
        rows_seen=$((rows_seen + 1))
        if [ -z "${want_when[$label]+x}" ]; then
            mismatches=$((mismatches + 1))
            bad "$name: незнакомый label '$label' — сценарий поменялся, обнови ожидания теста"
            continue
        fi
        exp_when="${want_when[$label]}"
        exp_req="${want_required[$label]}"
        exp_sleep="${want_sleep[$label]}"
        if [ "$when_robot_asked" != "$exp_when" ]; then
            mismatches=$((mismatches + 1))
            bad "$name шаг '$label': when_robot_asked='$when_robot_asked', ожидали '$exp_when'"
        fi
        if [ "$required_question" != "$exp_req" ]; then
            mismatches=$((mismatches + 1))
            bad "$name шаг '$label': required_question='$required_question', ожидали '$exp_req'"
        fi
        if [ "$sleep_before_sec" != "$exp_sleep" ]; then
            mismatches=$((mismatches + 1))
            bad "$name шаг '$label': sleep_before_sec='$sleep_before_sec', ожидали '$exp_sleep'"
        fi
    done < "$parsed"
    [ "$rows_seen" -eq "$want_rows" ] \
        && ok "$name: прочитано $want_rows строк через реальный read-цикл" \
        || bad "$name: прочитано $rows_seen строк, ожидали $want_rows"
    [ "$mismatches" -eq 0 ] && ok "$name: when_robot_asked/required_question/sleep_before_sec ТОЧНО совпадают с scenario.json для всех $want_rows шагов"
}

# =============================================================================
# CASE 2 — night_marathon_act2b_identity_question_v1.json (#3992: узнавание без
# переспроса): вопроса личности нет, when_robot_asked/required_question пусты,
# но посреди акта стоит шаг с паузой (sleep_before_sec=150) — непустое поле
# между пустыми. Проверяем ТОЧНОЕ соответствие scenario.json.
# =============================================================================
PARSED2B="$WORKDIR/act2b.tsv"
parse_scenario_to_tsv "$ACT2B" "$PARSED2B"

declare -A want_when=() want_required=() want_sleep=()
for label in n701_sasha_back_recognized n702_sasha_recognized_no_question n703_sasha_memory_from_act2 \
        n704_sasha_after_gap_no_question n705_boris_back_recognized n706_boris_recognized_no_question \
        n707_boris_memory_from_act2 n708_sasha_not_boris n709_grisha_drop_in n710_grisha_not_named_by_voice; do
    want_when[$label]=""
    want_required[$label]="0"
    want_sleep[$label]="0.0"
done
want_sleep[n704_sasha_after_gap_no_question]="150.0"
check_rows_exact "$PARSED2B" "act2b" 10

# =============================================================================
# CASE 3 — смесь ВСЕХ трёх полей в одном сценарии (до #3992 её давал акт 2b с
# вопросом личности; реальных сценариев с when_robot_asked больше нет, а
# граница python-вывод -> bash-read у поля осталась). Сценарий — временный
# файл, разбор — тот же parse_scenario_to_tsv + E2E_SCENARIO_ROW_READ.
# Пустое поле стоит ПЕРЕД заполненными — сигнатура регресса #2824.
# =============================================================================
MIXED="$WORKDIR/mixed_v1.json"
cat > "$MIXED" <<'JSON'
{"steps": [
  {"label": "m1_plain", "voice": "anton", "text": "Робот, привет."},
  {"label": "m2_asked", "voice": "anton", "text": "Робот, да, это я.", "when_robot_asked": "Саш.*это ты|это ты.*Саш", "required_question": true},
  {"label": "m3_gap", "voice": "anton", "text": "Робот, ты здесь?", "sleep_before_sec": 150},
  {"label": "m4_if_asked", "voice": "zahar", "text": "Робот, это пока не важно.", "when_robot_asked": "как (тебя )?зовут|назов"},
  {"label": "m5_plain", "voice": "ermil", "text": "Робот, пока."}
]}
JSON
PARSEDM="$WORKDIR/mixed.tsv"
parse_scenario_to_tsv "$MIXED" "$PARSEDM"
want_when=([m1_plain]="" [m2_asked]="Саш.*это ты|это ты.*Саш" [m3_gap]="" [m4_if_asked]="как (тебя )?зовут|назов" [m5_plain]="")
want_required=([m1_plain]="0" [m2_asked]="1" [m3_gap]="0" [m4_if_asked]="0" [m5_plain]="0")
want_sleep=([m1_plain]="0.0" [m2_asked]="0.0" [m3_gap]="150.0" [m4_if_asked]="0.0" [m5_plain]="0.0")
check_rows_exact "$PARSEDM" "mixed" 5

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
