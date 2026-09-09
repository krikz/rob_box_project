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
# (от `name() {` до закрывающей `}` в нулевой колонке) или ничего.
extract_func() {
    awk -v fn="$2" '
        $0 == fn "() {" || index($0, fn "() {") == 1 { f = 1 }
        f { print }
        f && /^\}$/ { exit }
    ' "$1"
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
