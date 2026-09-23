#!/bin/bash
# ============================================================================
# test_e2e_voice_robot_ros_quoting.sh — регресс-тест хрупкой обёртки robot_ros()
#
# Баг (issue #2805, найден в окне ревью компонента 2026-09-22)
# ----------------------------------------------------------------
# В .github/workflows/scripts/e2e_voice_test.sh:353-355:
#
#     robot_ros() {
#         ${ROBOT_SSH} "docker exec voice-assistant bash -lc 'source /opt/ros/...; source /ws/install/setup.bash; $*'"
#     }
#
# Две проблемы, обе архитектурные:
#
#  1. $* (НЕ "$@") склеивает позиционные аргументы через первый символ IFS.
#     Текущие вызовы передают ОДИН строковый аргумент, поэтому случайно
#     работает — но контракт нарушен, и любая будущая правка с двумя+
#     словами через пробел (или без кавычек) потеряет разделители.
#
#  2. bash -lc '...$*...' внутри одинарных кавычек НЕ разделяет $* обратно
#     на слова, и ЛОМАЕТСЯ синтаксически на любом аргументе с одинарной
#     кавычкой:
#
#         robot_ros "echo it's broken"
#         → bash -lc '... echo it's broken ...'   ← syntax error
#
# Цена поломки (issue #2750, ADR-0022 §4.1 R1 «smoke-false-PASS»):
# activate_e2e_speaker_db() / activate_e2e_memory_db() молча выходят с
# syntax error, _read_e2e_mode() возвращает пусто, ветка FATAL exit 2
# НЕ срабатывает, и сценарий едет по боевой /data/speakers.db.
#
# Этот тест НЕ фиксит харнесс (задача backend), а фиксирует КОНТРАКТ:
# после фикса вызов `robot_ros <arg>` должен доставлять <arg> на робота
# 1-в-1, независимо от shell-метасимволов внутри.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_voice_robot_ros_quoting.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$TEST_DIR/../../.." && pwd)"
HARNESS="${HARNESS:-$REPO/.github/workflows/scripts/e2e_voice_test.sh}"

[ -f "$HARNESS" ] || { echo "FAIL: $HARNESS not found"; exit 1; }

# Харнесс требует env (ROBOT_HOST/SSHPASS/YANDEX_API_KEY); нам нужна ТОЛЬКО
# определение robot_ros. Вырезаем её дословно и подменяем ROBOT_SSH на
# «печать итоговой команды без реального ssh».
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

STUB="$WORK/robot_ros_stub.sh"
{
    # Подмена ssh: сохраняем итоговую команду в файл как ОДНУ строку —
    # именно её получит удалённый sshd (а не список слов, как при
    # подстановке $*). Иначе не увидим single-quote внутри double-quoted
    # аргумента.
    echo 'ROBOT_SSH="_print_cmd"'
    echo '_print_cmd() { printf "%s\n" "$1" > "'"$WORK"'/last_cmd.txt"; }'
    # Дословно вырезаем robot_ros() из харнесса (включая ОДИНАРНЫЕ кавычки
    # вокруг «source ...; $*» — именно в них корень бага).
    awk '/^robot_ros\(\)/,/^\}/' "$HARNESS"
} > "$STUB"

# Санити: убеждаемся, что вырезали именно определение.
grep -q '^robot_ros()' "$STUB" || {
    echo "FAIL: не удалось вырезать robot_ros() из $HARNESS"
    echo "      (регресс: переименовали функцию? ожидалось имя robot_ros)"
    exit 1
}

PASS=0
FAIL=0

# case $1=name $2=arg $3=expected-substring-in-final-cmd
#
# ВАЖНО: вызываем robot_ros так, как это делают все 6 вызовов в харнессе
# (issue #2805) — без printf %q, просто «robot_ros "arg"». Если бы
# вызывающая сторона экранировала через printf %q, баг бы не
# воспроизводился.
#
# Контекст парсинга: реальный харнесс пишет в исходнике
#     robot_ros "echo $(whoami) hello"
# bash парсит эту строку ОДИН раз, $(whoami) разворачивается.
# Чтобы воспроизвести ровно этот путь, формируем временный bash-скрипт,
# где вызов буквально записан так же, как в харнессе, и запускаем его.
# Альтернативы (eval / bash -c) НЕ сохраняют контекст парсинга
# (особенно $(...) и апострофы).
expect() {
    local name="$1" arg="$2" want="$3"
    rm -f "$WORK/last_cmd.txt"

    # Временный bash-скрипт вызывает robot_ros с аргументом, записанным
    # ровно так, как его увидел бы bash в исходнике харнесса. Используем
    # heredoc БЕЗ кавычек вокруг разделителя, чтобы $arg подставился
    # литералом, и $(...) в нём развернулся при парсинге (точно как
    # в харнессе). Имя функции и её вызов формируем printf'ом, чтобы
    # спецсимволы в $arg не сломали shell.
    local case_script="$WORK/case.sh"
    {
        printf 'source %q\n' "$STUB"
        cat <<CASE
robot_ros "${arg}"
CASE
    } > "$case_script"

    bash "$case_script" 2>/dev/null

    if [ ! -f "$WORK/last_cmd.txt" ]; then
        echo "  ❌ $name: robot_ros не отправил команду (ничего в last_cmd.txt)"
        echo "     case-script:"
        sed 's/^/       /' "$case_script"
        FAIL=$((FAIL + 1))
        return
    fi
    local sent
    sent="$(cat "$WORK/last_cmd.txt")"
    # 1) Подстрока должна быть в итоговой команде.
    if ! printf '%s' "$sent" | grep -qF -- "$want"; then
        echo "  ❌ $name: итоговая команда не содержит «$want»"
        echo "     case-script:"
        sed 's/^/       /' "$case_script"
        echo "     отправлено:"
        printf '%s\n' "$sent" | sed 's/^/       /'
        FAIL=$((FAIL + 1))
        return
    fi
    # 2) Команда, доезжающая до робота, должна быть SYNTACTICALLY VALID:
    # это ровно тот класс бага, что issue #2805 описывает. Достаём
    # аргумент bash -lc и прогоняем через bash -n.
    local inner
    inner="$(printf '%s' "$sent" | sed -nE "s/.*bash -lc '(.*)'\$/\\1/p")"
    if [ -z "$inner" ]; then
        echo "  ⚠️  $name: не удалось выделить bash -lc '...'; пропускаем syntax-check"
    elif bash -n <<<"$inner" 2>/dev/null; then
        echo "  ✅ $name (syntax OK)"; PASS=$((PASS + 1))
    else
        echo "  ❌ $name: bash -lc на роботе ПОЛУЧИТ syntax error (issue #2805)"
        echo "     inner: $inner"
        echo "     подробности:"
        bash -n <<<"$inner" 2>&1 | sed 's/^/       /'
        FAIL=$((FAIL + 1))
    fi
}

echo "=== P1: текущий контракт — один строковый аргумент, команда доезжает целиком ==="
expect "ros2 param get … без shell-метасимволов" \
    "ros2 param get /speaker_id_node e2e_mode --no-daemon" \
    "ros2 param get /speaker_id_node e2e_mode --no-daemon"

echo
echo "=== P2: аргумент с апострофом (раньше — syntax error, теперь должен доехать) ==="
expect "echo it's broken" "echo it's broken" "echo it's broken"
expect "путь с апострофом" "ls /tmp/foo's bar/" "ls /tmp/foo's bar/"

echo
echo "=== P3: shell-инъекция через \$(...) не должна ВЫПОЛНЯТЬСЯ на стороне 249 ==="
# Сейчас $* стоит в DOUBLE-quoted строке «docker exec ... bash -lc '... $*'».
# Если вызывающий передаёт литерал в double-quotes «robot_ros "echo $(whoami) hello"»,
# bash РАЗВОРАЧИВАЕТ $(...) ещё ДО того, как $* подставляется — на 249,
# не на роботе. Это и дыра в безопасности, и неправильная семантика:
# оператор писал команду для робота, а она выполнилась на 249.
#
# После фикса (printf %q + eval) строка едет как литерал, и $(whoami)
# выполняется уже на роботе (где у него своё «whoami»).
#
# Репро: см. docs/reports/repro_robot_ros_issue_2805.sh, TEST 2.
expect "echo \$(whoami) — должна ДОЕХАТЬ как литерал \$((whoami) до робота" \
    'echo $(whoami) hello' \
    '$(whoami) hello'

echo
echo "=== P4: многословный аргумент с пробелами и кавычками разного типа ==="
expect "смесь кавычек и пробелов" 'echo "hi" it'"'"'s fine' 'fine'

echo
echo "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" = "0" ] || exit 1
