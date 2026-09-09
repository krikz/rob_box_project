#!/bin/bash
# ============================================================================
# lib_eval_func.sh — вызвать ОДНУ функцию из процессного скрипта, не запуская
# сам скрипт (у скриптов top-level код: flock, MAINTENANCE-гейт, обход issues).
#
# История: test_gh_label_filter_fallback.sh (гард бага #1457, «gh issue list
# --label отдаёт пустой массив») сорсит этот файл с момента своего появления,
# но самого файла в репозитории НИКОГДА не было — тест падал первой же
# строкой и, соответственно, ни разу не проверил ни одной регрессии.
# Восстановлен 30.08 вместе с дедупом процессного слоя: gh_list_issues_by_label
# переехал из четырёх скриптов в lib_agent_flow_common.sh, и этот тест —
# единственное, что его покрывает.
#
# API:
#   eval_helper <script_path> <func_name> [args...]
#   extract_func <script_path> <func_name>      — печатает тело в stdout
#   extract_func_or_die <script_path> <func_name> — то же, но с FAIL-сообщением
#                                                   и exit 1 при отсутствии
#   load_func <script_path> <func_name>          — eval'ит тело в текущий scope
#
# Как ищем функцию (важно для смысла теста):
#   1. Определение `func_name() {` в самом скрипте — берём его.
#   2. Нет — скрипт обязан сорсить lib_agent_flow_common.sh (проверяем
#      грепом по тексту скрипта); только тогда подключаем библиотеку.
#      Это сохраняет исходный смысл гарда «скрипт X умеет helper Y»:
#      если скрипт забудет подключить библиотеку, тест упадёт, а не
#      молча возьмёт функцию из соседнего файла.
#
# Окружение внутри вызова: подставляем `log`, чтобы _af_log писал в stderr
# (тесты проверяют текст сообщения о fallback), и дефолты GH_REPO/лимитов.
# Всё выполняется в СУБШЕЛЛЕ — top-level `exit` в функции (af_*_or_exit)
# не убьёт сам тест.
# ============================================================================

# extract_func <script_path> <func_name> — печатает текст определения функции
# (от `name() {` до балансирующей `}`; корректно работает с вложенными `{` в
# телах — подсчитывает глубину через awk; работает и для функций с отступом,
# например внутри if-блоков или loop'ов — issue #2295, например
# ` resolve_acceptance_candidate() {` в agent-flow-e2e-process.sh). Если в
# скрипте функция не найдена, печатает ничего (молчаливый mode) — это
# поведение совместимо с историческими вызовами в тестах до 30.08. Для
# жёсткого контракта используйте extract_func_or_die.
#
# БЫЛО: ~10 копий awk-обёрток в tests/*.sh различались сигнатурами вызова
# (positional args, outvar через printf -v, file output через >file,
# marker-based для #2341-блоков). Консолидировано в ОДНУ реализацию после
# того как 30.08-дедуп процессного слоя (функции в lib_agent_flow_common.sh)
# сломал 4 теста (t_xxx-yyy-zzz / #2295): каждая копия awk искала функцию
# в конкретном скрипте и при реинденте/переносе возвращала пустое тело.
#
# Алгоритм:
#   1. На строке с `name() {` (возможно с ведущим whitespace) — capture=1,
#      depth=0 (старт до подсчёта скобок на ЭТОЙ строке).
#   2. Печатаем строку, считаем `{`/`}`, обновляем depth.
#   3. Когда depth возвращается в 0 — это балансирующая `}`, выходим.
#      (Трюк: на открывающей строке `name() {` сначала depth=0, потом
#      считаем `{` и depth→1. Поэтому проверка depth==0 СРАБАТЫВАЕТ
#      только после реальной закрывающей скобки.)
extract_func() {
    awk -v fn="$2" '
        function trim_ws(s) { sub(/^[[:space:]]+/, "", s); return s }
        {
            line = $0
            # detect opener: optional leading whitespace, then fn, then ()
            # then optional spaces, then {. Comment-strip via first field
            # check is unnecessary — функция может начинаться с комментария
            # ПОСЛЕ { (типа `name() {  # foo`), это наш awk и так ловит
            # (комментарий остаётся частью строки, depth считается до него).
            #
            # ВАЖНО про регекс: в mawk (default awk на Debian/Ubuntu, mawk
            # без --re-interval) синтаксис `\{` трактует `{` как литерал, а
            # `\(` НЕ трактует `(` как литерал — там работает только char-class
            # `[(]`. Поэтому везде ниже используем `[(][)][{]` вместо
            # `\\(\\)\\{"` — это переносимо между mawk / gawk / BWK awk.
            # (issue #2295 — изначальный wip использовал `\(...` который
            # работал только в gawk; на mawk opener не матчился и тесты
            # падали с "function not found".)
            stripped = line; sub(/^[[:space:]]+/, "", stripped)
            if (!capture && stripped ~ "^" fn "[[:space:]]*[(][)][[:space:]]*[{]" ) {
                capture = 1
                depth = 0
            }
            if (capture) {
                print line
                n = gsub(/[{]/, "x", line); depth += n
                n = gsub(/[}]/, "x", line); depth -= n
                if (depth == 0) { capture = 0; exit }
            }
        }
    ' "$1"
}

# extract_func_or_die <script_path> <func_name> — как extract_func, но
# при отсутствии функции печатает FAIL-сообщение в stderr и возвращает 1.
# Используется в тестах, где «функция X живёт в скрипте Y» — это часть
# контракта (например, test_detect_pr_kind.sh: detect_pr_kind в ЛИБЕ, не в
# e2e-process.sh / merge-gate.sh). Если перенесли — тест СРАЗУ это видит.
extract_func_or_die() {
    local _body
    _body="$(extract_func "$1" "$2")"
    if [ -z "$_body" ]; then
        printf 'FAIL: func %s не найдена в %s\n' "$2" "$1" >&2
        return 1
    fi
    printf '%s\n' "$_body"
}

# load_func <script_path> <func_name> — обёртка над eval "$(extract_func ...)"
# для удобства тестов, которые просто хотят «определить функцию в текущем
# scope». Сбой → return 1 + сообщение в stderr.
load_func() {
    local _body
    _body="$(extract_func_or_die "$1" "$2")" || return 1
    eval "$_body"
}

eval_helper() {  # $1=script $2=func [args...]
    local _script="$1" _func="$2"
    shift 2
    local _dir _body _lib
    _dir="$(cd "$(dirname "$_script")" && pwd)"
    _lib="$_dir/lib_agent_flow_common.sh"
    _body="$(extract_func "$_script" "$_func")"

    (
        set +e
        export GH_REPO="${GH_REPO:-krikz/rob_box_project}"
        export ISSUE_LIMIT="${ISSUE_LIMIT:-20}"
        export LIMIT="${LIMIT:-20}"
        # Свой log: _af_log из библиотеки делегирует в него, а тесты
        # грепают stderr на «fallback на REST».
        log() { printf '[eval_helper] %s\n' "$*" >&2; }

        if [ -n "$_body" ]; then
            eval "$_body"
        else
            if ! grep -q 'lib_agent_flow_common\.sh' "$_script"; then
                printf 'eval_helper: %s не определяет %s и не сорсит lib_agent_flow_common.sh\n' \
                    "$(basename "$_script")" "$_func" >&2
                exit 127
            fi
            if [ ! -f "$_lib" ]; then
                printf 'eval_helper: %s не найден рядом с %s\n' "$_lib" "$_script" >&2
                exit 127
            fi
            # shellcheck source=../../lib_agent_flow_common.sh
            . "$_lib"
        fi

        if ! declare -F "$_func" >/dev/null 2>&1; then
            printf 'eval_helper: функция %s не найдена (%s)\n' "$_func" "$_script" >&2
            exit 127
        fi
        "$_func" "$@"
    )
}
