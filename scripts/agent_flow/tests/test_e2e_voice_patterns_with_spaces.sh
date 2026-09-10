#!/bin/bash
# ============================================================================
# test_e2e_voice_patterns_with_spaces.sh — паттерн с пробелами = ОДИН паттерн.
#
# Баг (run 34414065635, акт 3 ночного марафона, 09.09.2026)
# --------------------------------------------------------
# В scenario-цикле было:
#
#     pats="$(... " ".join(patterns) ...)"
#     check_patterns "$STEP_BEFORE" $pats      # ← без кавычек
#
# bash разбивал строку по пробелам, и многословный regex превращался в
# несколько независимых паттернов. Живьём:
#
#     pattern: \[backlog\] accumulated \(no_wake_word\).*speaker='Саш
#     →  PATTERN_OK:   \[backlog\]
#        PATTERN_OK:   accumulated
#        PATTERN_MISS: \(no_wake_word\).*speaker='Саш
#
# Цена бага двусторонняя, и вторая сторона хуже:
#   * ложный красный — три четверти паттерна совпали, шаг всё равно упал;
#   * ЛОЖНЫЙ ЗЕЛЁНЫЙ — в 1280_barge_in_abort_old_topic.json паттерн
#     «Cancel: new STT input» проверялся как четыре штуки, из которых
#     «new» и «input» матчат почти любой лог. Сьюта зеленела на мусоре.
#
# Проверяем ровно контракт: сколько элементов в JSON — столько паттернов
# уходит в check_patterns, и каждый доезжает целиком.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_voice_patterns_with_spaces.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$TEST_DIR/../../.." && pwd)"
HARNESS="${HARNESS:-$REPO/.github/workflows/scripts/e2e_voice_test.sh}"

[ -f "$HARNESS" ] || { echo "FAIL: $HARNESS not found"; exit 1; }

# python3 на Windows-хостах бывает WindowsApps-заглушкой: `command -v` её
# находит, а запуск уходит в Microsoft Store. Проверяем ЗАПУСКОМ.
# Харнесс на билд-машине зовёт именно `python3`, поэтому если рабочий
# интерпретатор называется иначе — подставляем его функцией с этим именем.
PY_REAL=""
for _cand in python3 python py; do
    if "$_cand" -c 'import sys' >/dev/null 2>&1; then PY_REAL="$_cand"; break; fi
done
[ -n "$PY_REAL" ] || { echo "SKIP: рабочий python3 не найден"; exit 0; }
if [ "$PY_REAL" != "python3" ]; then
    python3() { "$PY_REAL" "$@"; }
    export -f python3
    export PY_REAL
fi

# Паттерны и названия шагов кириллические; под POSIX-локалью python пишет
# в cp1252 и падает с UnicodeEncodeError раньше, чем что-то проверит.
export PYTHONIOENCODING=utf-8

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Вырезаем из харнесса ровно тот кусок, который превращает JSON в аргументы
# check_patterns. Гоняем со стабом check_patterns, который печатает, что
# получил.
EXTRACT="$WORK/pats.sh"
{
    echo 'log() { :; }'
    echo 'check_patterns() { shift; echo "COUNT=$#"; for p in "$@"; do echo "PAT[$p]"; done; }'
    echo 'STEP_BEFORE=BEFORE'
    echo 'label=test'
    awk '/mapfile -t _pats_arr/,/check_patterns "\$STEP_BEFORE" "\$\{_pats_arr\[@\]\}"/' "$HARNESS"
} > "$EXTRACT"

grep -q 'mapfile -t _pats_arr' "$EXTRACT" || {
    echo "FAIL: не удалось вырезать блок разбора patterns из $HARNESS"
    echo "      (ожидался 'mapfile -t _pats_arr' — регресс к \$pats без кавычек?)"
    exit 1
}

run_with() {  # $1=json → stdout стаба
    printf '%s' "$1" > "$WORK/pats.json"
    patterns_json="$(cat "$WORK/pats.json")" bash -c "
        set -u
        patterns_json=\"\$patterns_json\"
        source '$EXTRACT'
    " 2>/dev/null | tr -d ''
}

expect_count() {  # $1=name $2=json $3=ожидаемое число паттернов
    local name="$1" json="$2" want="$3" out got
    out="$(run_with "$json")"
    got="$(printf '%s' "$out" | sed -n 's/^COUNT=//p')"
    if [ "$got" = "$want" ]; then
        echo "  ✅ $name (COUNT=$got)"; PASS=$((PASS + 1))
    else
        echo "  ❌ $name: ожидалось COUNT=$want, получено COUNT=${got:-<нет>}"
        printf '%s\n' "$out" | sed 's/^/     /'
        FAIL=$((FAIL + 1))
    fi
}

expect_pat() {  # $1=name $2=json $3=паттерн, который должен доехать целиком
    local name="$1" json="$2" want="$3" out
    out="$(run_with "$json")"
    if printf '%s' "$out" | grep -qxF "PAT[$want]"; then
        echo "  ✅ $name"; PASS=$((PASS + 1))
    else
        echo "  ❌ $name: паттерн не доехал целиком"
        printf '     ожидалось PAT[%s]\n' "$want"
        printf '%s\n' "$out" | sed 's/^/     /'
        FAIL=$((FAIL + 1))
    fi
}

# JSON собираем питоном — экранирование бэкслешей в bash-литерале слишком
# легко испортить, а тест должен падать на харнессе, а не на своей фикстуре.
mk_json() { python3 -c 'import json,sys; print(json.dumps(sys.argv[1:], ensure_ascii=False))' "$@"; }

BACKLOG_PAT="\\[backlog\\] accumulated \\(no_wake_word\\).*speaker='Саш"
BARGE_PAT="Cancel: new STT input"

echo "=== P1: один многословный паттерн остаётся одним ==="
expect_count "regex с пробелами не разбит" "$(mk_json "$BACKLOG_PAT")" 1
expect_pat   "regex доехал целиком"        "$(mk_json "$BACKLOG_PAT")" "$BACKLOG_PAT"

echo "=== P2: 1280_barge_in — три фразы, а не десять слов ==="
expect_count "три паттерна" \
    "$(mk_json "$BARGE_PAT" "STOP command received" "Воспроизведение прервано")" 3
expect_pat   "«Cancel: new STT input» целиком" "$(mk_json "$BARGE_PAT")" "$BARGE_PAT"

echo "=== P3: односложные паттерны не сломаны ==="
expect_count "три однословных" "$(mk_json set_voice stop_music voice_used)" 3
expect_pat   "set_voice"       "$(mk_json set_voice stop_music voice_used)" "set_voice"

echo "=== P4: смесь и пустой список ==="
expect_count "смесь длинных и коротких" \
    "$(mk_json "$BACKLOG_PAT" set_voice "STOP command received")" 3
expect_count "пустой список" "$(mk_json)" 0

echo
echo "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" = "0" ] || exit 1
