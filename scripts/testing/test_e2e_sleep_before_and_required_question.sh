#!/usr/bin/env bash
# Гард для sleep_before_sec / when_robot_asked / required_question
# (issue #2809, E2E-харнесс "переспрос личности"; issue #2824 -- регресс
# от прод-разделителя \t, разбор см. в e2e_voice_lib.sh:parse_scenario_to_tsv).
#
# Часть 1 — ФУНКЦИОНАЛЬНАЯ: sourc'ит parse_scenario_to_tsv() из
# e2e_voice_lib.sh (ЕДИНСТВЕННОЕ место, где реально живёт python-парсер —
# issue #2824 переехал сюда из inline-heredoc в e2e_voice_test.sh) и
# прогоняет её на тестовом scenario.json с edge-case значениями (тип
# int/bool/str, битые значения, отсутствующие поля) — читая результат ЧЕРЕЗ
# РЕАЛЬНЫЙ `IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"`, а не через `cut -f`
# по позиции столбца. issue #2824 началось РОВНО с того, что предыдущая
# версия этого теста проверяла python-вывод напрямую (`cut -f9`) и никогда
# не пропускала его через bash `read` — а баг жил именно на этой границе.
# Синтетические edge-case'ы (int vs float sleep_before_sec, мусорная строка
# вместо числа) здесь ПОЛЕЗНЕЕ реальных сценариев (см.
# test_e2e_scenario_tsv_row.sh) — те дают только валидные production-значения.
#
# Часть 2 — СТАТИЧЕСКАЯ: sleep_before_sec обязан идти ДО того, как шаг
# зафиксирует STEP_BEFORE/step_window_start (иначе окно логов шага захватит
# саму паузу), а required_question=1 обязан вести к emit_step FAIL, не SKIP
# (иначе акт "переспрос личности" не детерминирован — issue #2809 п.3).
#
# Запуск: bash scripts/testing/test_e2e_sleep_before_and_required_question.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"
LIB="$REPO_ROOT/.github/workflows/scripts/e2e_voice_lib.sh"

if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; NC=$'\033[0m'
else
    RED=""; GRN=""; NC=""
fi

for f in "$HARNESS" "$LIB"; do
    [ -f "$f" ] || { printf '❌ отсутствует: %s\n' "$f"; exit 1; }
done

PASS=0
FAIL=0
ok()  { printf '%sOK%s   %s\n' "$GRN" "$NC" "$1"; PASS=$((PASS + 1)); }
bad() { printf '%sFAIL%s %s\n' "$RED" "$NC" "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# =============================================================================
# ЧАСТЬ 1 — функциональный тест реального парсера + реального read-цикла
# =============================================================================
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
ok "parse_scenario_to_tsv/E2E_SCENARIO_ROW_READ определены после source $LIB"

SCENARIO="$WORKDIR/scenario.json"
cat > "$SCENARIO" <<'JSON'
{
  "steps": [
    {
      "label": "full_fields",
      "text": "Робот, да, это я.",
      "voice": "anton",
      "when_robot_asked": "Саш.*это ты|это ты.*Саш",
      "required_question": true,
      "sleep_before_sec": 35
    },
    {
      "label": "defaults_only",
      "text": "Робот, привет."
    },
    {
      "label": "when_asked_no_required",
      "text": "Робот, это пока не важно.",
      "when_robot_asked": "как (тебя )?зовут",
      "required_question": false
    },
    {
      "label": "sleep_as_int",
      "text": "Робот, ты тут?",
      "sleep_before_sec": 10
    },
    {
      "label": "sleep_bad_value",
      "text": "Робот, привет ещё раз.",
      "sleep_before_sec": "не число"
    }
  ]
}
JSON

PARSED="$WORKDIR/scenario_parsed.txt"
if PYTHONIOENCODING=utf-8 parse_scenario_to_tsv "$SCENARIO" "$PARSED" 2>"$WORKDIR/parser_stderr.txt"; then
    ok "parse_scenario_to_tsv отработала без ошибок на тестовом сценарии"
else
    bad "parse_scenario_to_tsv упала: $(cat "$WORKDIR/parser_stderr.txt")"
fi

declare -A ROW_WHEN ROW_REQ ROW_SLEEP
rows_seen=0
# issue #2824 -- КЛЮЧЕВОЕ ОТЛИЧИЕ от версии теста, не поймавшей регресс:
# читаем через ТОТ ЖЕ read-цикл, что использует main flow, а не через
# `cut -f<N>` по python-выводу напрямую.
while IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"; do
    [ -z "$idx" ] && continue
    rows_seen=$((rows_seen + 1))
    ROW_WHEN["$label"]="$when_robot_asked"
    ROW_REQ["$label"]="$required_question"
    ROW_SLEEP["$label"]="$sleep_before_sec"
done < "$PARSED"
[ "$rows_seen" -eq 5 ] \
    && ok "прочитано 5 строк через реальный read-цикл" \
    || bad "прочитано $rows_seen строк, ожидали 5"

if [ "${ROW_WHEN[full_fields]-<нет>}" = "Саш.*это ты|это ты.*Саш" ]; then
    ok "when_robot_asked распарсился дословно"
else
    bad "when_robot_asked исказился: '${ROW_WHEN[full_fields]-<нет>}'"
fi
if [ "${ROW_REQ[full_fields]-<нет>}" = "1" ]; then
    ok "required_question: true -> '1'"
else
    bad "required_question должен был стать '1', получили '${ROW_REQ[full_fields]-<нет>}'"
fi
if [ "${ROW_SLEEP[full_fields]-<нет>}" = "35.0" ]; then
    ok "sleep_before_sec: 35 -> '35.0'"
else
    bad "sleep_before_sec должен был стать '35.0', получили '${ROW_SLEEP[full_fields]-<нет>}'"
fi

if [ -z "${ROW_WHEN[defaults_only]-НЕТ}" ] && [ -n "${ROW_WHEN[defaults_only]+x}" ]; then
    ok "шаг без when_robot_asked -> пустая колонка (не матчит никогда, и это САМА СИГНАТУРА регресса #2824)"
else
    bad "шаг без when_robot_asked дал '${ROW_WHEN[defaults_only]-<нет>}' вместо пустой строки"
fi
if [ "${ROW_REQ[defaults_only]-<нет>}" = "0" ]; then
    ok "шаг без required_question -> дефолт '0'"
else
    bad "required_question дефолт должен быть '0', получили '${ROW_REQ[defaults_only]-<нет>}'"
fi
if [ "${ROW_SLEEP[defaults_only]-<нет>}" = "0.0" ]; then
    ok "шаг без sleep_before_sec -> дефолт '0.0'"
else
    bad "sleep_before_sec дефолт должен быть '0.0', получили '${ROW_SLEEP[defaults_only]-<нет>}'"
fi

if [ "${ROW_REQ[when_asked_no_required]-<нет>}" = "0" ]; then
    ok "required_question: false -> '0' (не совпадает с true-веткой)"
else
    bad "required_question: false должен был дать '0', получили '${ROW_REQ[when_asked_no_required]-<нет>}'"
fi

if [ "${ROW_SLEEP[sleep_as_int]-<нет>}" = "10.0" ]; then
    ok "sleep_before_sec как int (10) -> '10.0' (тип не роняет парсер)"
else
    bad "sleep_before_sec=10 (int) дал '${ROW_SLEEP[sleep_as_int]-<нет>}', ожидали '10.0'"
fi

if [ "${ROW_SLEEP[sleep_bad_value]-<нет>}" = "0.0" ]; then
    ok "sleep_before_sec с мусорным значением не роняет парсер, даёт 0.0"
else
    bad "sleep_before_sec с мусором дал '${ROW_SLEEP[sleep_bad_value]-<нет>}', ожидали безопасный дефолт 0.0"
fi

# =============================================================================
# ЧАСТЬ 2 — статические проверки порядка/wiring в главном скрипте
# =============================================================================
# issue #2824: сигнатура read-цикла сменилась на eval + общую переменную
# E2E_SCENARIO_ROW_READ (см. e2e_voice_lib.sh) — ищем НОВУЮ форму.
LOOP_BODY="$(awk '/while IFS=\$.\\x1f. eval "\$E2E_SCENARIO_ROW_READ"; do/,/^    done < "\$OUT_DIR\/scenario_parsed.txt"$/' "$HARNESS")"

if [ -z "$LOOP_BODY" ]; then
    bad "не нашёл тело scenario-цикла (while IFS=\$'\\x1f' eval ... E2E_SCENARIO_ROW_READ) — сигнатура read съехала?"
else
    ok "тело scenario-цикла с новым read-циклом найдено"

    sleep_pos="$(printf '%s\n' "$LOOP_BODY" | grep -n 'sleep "\$sleep_before_sec"' | head -1 | cut -d: -f1)"
    step_before_pos="$(printf '%s\n' "$LOOP_BODY" | grep -n 'STEP_BEFORE=' | head -1 | cut -d: -f1)"
    if [ -n "$sleep_pos" ] && [ -n "$step_before_pos" ] && [ "$sleep_pos" -lt "$step_before_pos" ]; then
        ok "sleep_before_sec выполняется ДО первого STEP_BEFORE (окно логов шага не включает саму паузу)"
    else
        bad "порядок sleep_before_sec vs STEP_BEFORE неверный (sleep@${sleep_pos:-?} step_before@${step_before_pos:-?}) — окно логов шага может включать паузу"
    fi

    if printf '%s\n' "$LOOP_BODY" | grep -q 'required_question" = "1"'; then
        ok "required_question проверяется в scenario-цикле"
    else
        bad "не нашёл ветвление по required_question в scenario-цикле"
    fi

    required_block="$(printf '%s\n' "$LOOP_BODY" | awk '/required_question" = "1"/,/fi$/' | head -20)"
    if printf '%s\n' "$required_block" | grep -q 'FAIL robot_did_not_ask'; then
        ok "required_question=1 без вопроса -> FAIL robot_did_not_ask (не SKIP)"
    else
        bad "required_question=1 не ведёт к FAIL robot_did_not_ask — акт не будет детерминированным"
    fi

    if printf '%s\n' "$LOOP_BODY" | grep -q 'SKIP robot_did_not_ask'; then
        ok "мягкая ветка (required_question не задан) даёт SKIP robot_did_not_ask, отдельно от FAIL"
    else
        bad "не нашёл мягкий SKIP robot_did_not_ask — опциональный when_robot_asked должен уметь молча пропускать шаг"
    fi

    if printf '%s\n' "$required_block" | grep -q 'PASS=0' && printf '%s\n' "$required_block" | grep -q 'mark_fail_kind feature'; then
        ok "required_question FAIL помечает PASS=0 и mark_fail_kind feature (влияет на общий вердикт)"
    else
        bad "required_question FAIL не помечает PASS=0/mark_fail_kind feature — FAIL может не долететь до общего вердикта прогона"
    fi
fi

# scenario-цикл обязан вызывать ЕДИНУЮ функцию парсинга, не свою копию.
if grep -q 'parse_scenario_to_tsv "\$SCENARIO_FILE" "\$OUT_DIR/scenario_parsed.txt"' "$HARNESS"; then
    ok "scenario-цикл вызывает parse_scenario_to_tsv (единая точка истины, issue #2824)"
else
    bad "не нашёл вызов parse_scenario_to_tsv в scenario-цикле — парсер снова задублирован?"
fi

# node_params: apply ДО первого шага, restore в trap EXIT.
if grep -q 'apply_node_params "\$SCENARIO_FILE"' "$HARNESS"; then
    ok "apply_node_params вызывается для сценария"
else
    bad "не нашёл вызов apply_node_params — node_params сценария не применяется вообще"
fi
if grep -qE "trap 'restore_node_params;.*EXIT" "$HARNESS"; then
    ok "restore_node_params включён в trap EXIT (восстановление при любом завершении)"
else
    bad "restore_node_params НЕ в trap EXIT — форсированный порог переживёт аварийный обрыв акта"
fi

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
