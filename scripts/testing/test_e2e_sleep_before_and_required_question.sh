#!/usr/bin/env bash
# Гард для sleep_before_sec / when_robot_asked / required_question
# (issue #2809, E2E-харнесс "переспрос личности", follow-up PR #2818).
#
# Часть 1 — ФУНКЦИОНАЛЬНАЯ: вытаскивает РЕАЛЬНЫЙ python-парсер сценария из
# e2e_voice_test.sh (тот самый heredoc, что печатает scenario_parsed.txt) и
# прогоняет его на тестовом scenario.json — проверяет, что новые колонки
# (when_robot_asked/required_question/sleep_before_sec) парсятся правильно:
# дефолты, санитайз \t/\n, приведение типов, битые значения не роняют парсер.
# Так тест ловит регресс в САМОМ коде харнесса, а не в его копии.
#
# Часть 2 — СТАТИЧЕСКАЯ: sleep_before_sec обязан идти ДО того, как шаг
# зафиксирует STEP_BEFORE/step_window_start (иначе окно логов шага захватит
# саму паузу, и робот "получит" лишние секунды в измерении latency), а
# required_question=1 обязан вести к emit_step FAIL, не SKIP (иначе акт
# "переспрос личности" не детерминирован — п.3 issue #2809).
#
# Запуск: bash scripts/testing/test_e2e_sleep_before_and_required_question.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"

if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; NC=$'\033[0m'
else
    RED=""; GRN=""; NC=""
fi

[ -f "$HARNESS" ] || { printf '❌ отсутствует: %s\n' "$HARNESS"; exit 1; }

PASS=0
FAIL=0
ok()  { printf '%sOK%s   %s\n' "$GRN" "$NC" "$1"; PASS=$((PASS + 1)); }
bad() { printf '%sFAIL%s %s\n' "$RED" "$NC" "$1"; FAIL=$((FAIL + 1)); }

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# =============================================================================
# ЧАСТЬ 1 — функциональный тест реального парсера
# =============================================================================
PARSER_PY="$WORKDIR/extracted_parser.py"
python3 - "$HARNESS" "$PARSER_PY" <<'EXTRACT'
import sys
harness_path, out_path = sys.argv[1], sys.argv[2]
content = open(harness_path, encoding="utf-8").read()
marker = 'scenario_parsed.txt"\n'
start = content.index(marker) + len(marker)
end = content.index('\nPY\n', start)
block = content[start:end]
if "when_robot_asked" not in block or "sleep_before_sec" not in block:
    sys.stderr.write("EXTRACT_FATAL: извлечённый блок не содержит новых полей — маркер съехал?\n")
    sys.exit(1)
open(out_path, "w", encoding="utf-8").write(block)
EXTRACT
extract_rc=$?
if [ "$extract_rc" != "0" ] || [ ! -s "$PARSER_PY" ]; then
    bad "не удалось извлечь python-парсер сценария из $HARNESS (маркер scenario_parsed.txt съехал?)"
    printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
    exit 1
fi
ok "python-парсер сценария извлечён из харнесса дословно"

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
PYTHONIOENCODING=utf-8 python3 "$PARSER_PY" "$SCENARIO" > "$PARSED" 2>"$WORKDIR/parser_stderr.txt"
parser_rc=$?
if [ "$parser_rc" != "0" ]; then
    bad "извлечённый парсер упал с rc=$parser_rc: $(cat "$WORKDIR/parser_stderr.txt")"
else
    ok "извлечённый парсер отработал без ошибок на тестовом сценарии"
fi

get_row() {  # $1=label -> печатает всю TSV-строку
    awk -F'\t' -v want="$2" '$2==want' "$PARSED"
}

row_full="$(get_row 0 full_fields)"
if [ -n "$row_full" ]; then
    when_col="$(printf '%s' "$row_full" | cut -f9)"
    req_col="$(printf '%s' "$row_full" | cut -f10)"
    sleep_col="$(printf '%s' "$row_full" | cut -f11)"
    [ "$when_col" = "Саш.*это ты|это ты.*Саш" ] \
        && ok "when_robot_asked распарсился дословно" \
        || bad "when_robot_asked исказился: '$when_col'"
    [ "$req_col" = "1" ] \
        && ok "required_question: true -> '1'" \
        || bad "required_question должен был стать '1', получили '$req_col'"
    [ "$sleep_col" = "35.0" ] \
        && ok "sleep_before_sec: 35 -> '35.0'" \
        || bad "sleep_before_sec должен был стать '35.0', получили '$sleep_col'"
else
    bad "строка full_fields не найдена в scenario_parsed.txt"
fi

row_defaults="$(get_row 0 defaults_only)"
if [ -n "$row_defaults" ]; then
    when_col="$(printf '%s' "$row_defaults" | cut -f9)"
    req_col="$(printf '%s' "$row_defaults" | cut -f10)"
    sleep_col="$(printf '%s' "$row_defaults" | cut -f11)"
    [ -z "$when_col" ] \
        && ok "шаг без when_robot_asked -> пустая колонка (не матчит никогда)" \
        || bad "шаг без when_robot_asked дал непустое значение: '$when_col'"
    [ "$req_col" = "0" ] \
        && ok "шаг без required_question -> дефолт '0'" \
        || bad "required_question дефолт должен быть '0', получили '$req_col'"
    [ "$sleep_col" = "0.0" ] \
        && ok "шаг без sleep_before_sec -> дефолт '0.0'" \
        || bad "sleep_before_sec дефолт должен быть '0.0', получили '$sleep_col'"
else
    bad "строка defaults_only не найдена в scenario_parsed.txt"
fi

row_soft="$(get_row 0 when_asked_no_required)"
if [ -n "$row_soft" ]; then
    req_col="$(printf '%s' "$row_soft" | cut -f10)"
    [ "$req_col" = "0" ] \
        && ok "required_question: false -> '0' (не совпадает с true-веткой)" \
        || bad "required_question: false должен был дать '0', получили '$req_col'"
else
    bad "строка when_asked_no_required не найдена в scenario_parsed.txt"
fi

row_bad="$(get_row 0 sleep_bad_value)"
if [ -n "$row_bad" ]; then
    sleep_col="$(printf '%s' "$row_bad" | cut -f11)"
    [ "$sleep_col" = "0.0" ] \
        && ok "sleep_before_sec с мусорным значением не роняет парсер, даёт 0.0" \
        || bad "sleep_before_sec с мусором дал '$sleep_col', ожидали безопасный дефолт 0.0"
else
    bad "строка sleep_bad_value не найдена — мусорное значение уронило парсер?"
fi

# =============================================================================
# ЧАСТЬ 2 — статические проверки порядка/wiring в главном скрипте
# =============================================================================
LOOP_BODY="$(awk '/while IFS=\$.\\t. read -r idx label text voice patterns_json acceptance_json expect_raw retry_acceptance when_robot_asked required_question sleep_before_sec/,/^    done < "\$OUT_DIR\/scenario_parsed.txt"$/' "$HARNESS")"

if [ -z "$LOOP_BODY" ]; then
    bad "не нашёл тело scenario-цикла (while IFS=\$'\\t' read ... when_robot_asked ...) — сигнатура read съехала?"
else
    ok "тело scenario-цикла с новыми полями найдено"

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
