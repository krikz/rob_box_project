#!/bin/bash
# ============================================================================
# lib_agent_flow_common.sh — общие помощники процессных скриптов agent-flow.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/lib_agent_flow_common.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
#
# Зачем (дедуп процессного слоя 30.08):
#   До этого одно и то же жило копипастой по шести скриптам —
#   gh_list_issues_by_label ×4 (135 лишних строк), detect_pr_kind ×2,
#   free_stale_worktrees_for ×2, slugify ×3, has_label ×4, .env-загрузчик ×5,
#   flock-преамбула ×5, MAINTENANCE-гейт ×4. Копии успели разъехаться
#   дефолтами и текстами логов, а один и тот же баг приходилось чинить в
#   четырёх местах (или, чаще, в одном — см. updated_at ниже).
#
#   Кто что берёт: triage / merge-gate / e2e-process / deploy-sweep — почти
#   всё; unlabeled-sweep — has_label + оба гейта; handoff — .env + оба гейта.
#
# ⚠️ ЧЕГО ЗДЕСЬ НЕТ И ПОЧЕМУ (урок хендофа 30.08 §6: совпадение имени —
#    гипотеза, а не диагноз):
#   - `log()` — у каждого скрипта свой LOG_PREFIX. Функции ниже зовут
#     `_af_log`, который делегирует в `log`, если тот уже определён.
#   - `has_label` из agent-flow-deploy-sweep.sh НЕ сведён с остальными: там
#     на вход приходит labels **JSON** от `gh issue view --json labels`, а у
#     merge-gate / e2e-process / unlabeled-sweep — labels **CSV**. Одно имя,
#     два контракта. Обе версии здесь, но под разными именами:
#     `has_label` (CSV) и `has_label_json` (JSON).
#
# Использование (после `set -euo pipefail` и определения своего `log`):
#   _LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
#   # shellcheck source=lib_agent_flow_common.sh
#   . "$_LIB_DIR_HERE/lib_agent_flow_common.sh"
# ============================================================================

# ---------------------------------------------------------------------------
# _af_log <msg...> — внутренний лог библиотеки.
#
# Делегирует в `log` вызывающего скрипта, если тот определён (у каждого свой
# LOG_PREFIX), иначе печатает сам. Проверка через declare -F, а не наличие
# переменной, — на момент source'а `log` ещё может быть не определён, а на
# момент ВЫЗОВА функций ниже уже определён.
# ---------------------------------------------------------------------------
_af_log() {
    if declare -F log >/dev/null 2>&1; then
        log "$@"
    else
        printf '[agent-flow] %s %s\n' "$(date -Iseconds)" "$*" >&2
    fi
}

# ---------------------------------------------------------------------------
# af_load_profile_env [env_path] — подгрузить profile .env.
#
# Приоритет: env вызывающего > .env > дефолты скрипта. Намеренно БЕЗ `set -a`:
# он затёр бы override'ы вызывающего, а на них держатся тесты и cron-флаги.
# Пустое значение считается «не задано».
#
# Дефолты (`: "${X:=...}"`) НЕ здесь — они у каждого скрипта свои.
# ---------------------------------------------------------------------------
af_load_profile_env() {
    # Аргумент $1 — желаемый путь. Если файла нет — пробуем fallback-кандидаты
    # (per-profile gateway может передать HERMES_HOME=<profile_dir>; тогда
    # прямой путь уйдёт мимо реальной иерархии и GH_REPO не загрузится).
    # Ретро 12.08 t_061d466e + ретро 31.08 t_18941c54 (deploy-sweep).
    local _env_path="" _cand
    if [ -n "${1:-}" ] && [ -f "$1" ]; then
        _env_path="$1"
    else
        for _cand in \
            "/home/builder/.hermes/profiles/agent-flow/.env" \
            "${HERMES_HOME}/profiles/agent-flow/.env" \
            "${HOME}/hermes/profiles/agent-flow/.env" \
            "${HOME}/.hermes/profiles/agent-flow/.env"; do
            [ -f "$_cand" ] && { _env_path="$_cand"; break; }
        done
    fi
    unset _cand
    [ -z "$_env_path" ] && return 0
    local key val
    while IFS='=' read -r key val; do
        # skip comments / blanks
        case "$key" in ''|\#*) continue ;; esac
        # strip surrounding quotes from .env value
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        # only set if not already in caller env (treat empty as unset)
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$_env_path"
}

# ---------------------------------------------------------------------------
# af_flock_guard_or_exit [lock_file] — взять tick-lock или выйти (exit 0).
#
# Открывает fd 9 на lock-файл и берёт неблокирующий flock. Занят — значит
# предыдущий тик ещё идёт: выходим с 0 (это НЕ ошибка, cron позовёт снова).
#
# ⚠️ Функция ВЫХОДИТ ИЗ СКРИПТА. Зовите её только напрямую, из top-level:
#    в подстановке `$(...)` или пайпе `exit` убьёт лишь подоболочку.
# fd 9 остаётся открытым после return — на нём и держится замок до конца тика.
#
# Вариант с ожиданием (e2e-process G6: ждать до 60с, если тик запущен
# вручную через RUN_NOW) НЕ сведён сюда — там другая семантика.
# ---------------------------------------------------------------------------
af_flock_guard_or_exit() {  # $1=lock_file (default $LOCK_FILE)
    local _lock="${1:-${LOCK_FILE:-}}"
    if [ -z "$_lock" ]; then
        _af_log "af_flock_guard_or_exit: LOCK_FILE не задан"; exit 1
    fi
    exec 9>"$_lock" || { _af_log "cannot open lock $_lock"; exit 1; }
    if ! flock -n 9; then
        _af_log "another instance holds $_lock — skip"; exit 0
    fi
    return 0
}

# ---------------------------------------------------------------------------
# af_maintenance_gate_or_exit — kill-switch: exit 0, если стоит MAINTENANCE.
#
# Флаг — файл ${MAINTENANCE_FILE} (default MAINTENANCE) в ветке
# ${MAINTENANCE_BRANCH} (default develop). Две проверки:
#   1) remote (git ls-remote по $GH_REPO) — источник истины, его ставит
#      человек или agents_sleep.sh (PEAK-часы: «все спят»);
#   2) local clone $REPO_DIR — на случай, если сеть недоступна.
# Любая сработала → тик пропускается (exit 0, не ошибка).
#
# ⚠️ Функция ВЫХОДИТ ИЗ СКРИПТА — зовите только из top-level (см. выше).
# ---------------------------------------------------------------------------
af_maintenance_gate_or_exit() {
    local _branch="${MAINTENANCE_BRANCH:-develop}"
    local _file="${MAINTENANCE_FILE:-MAINTENANCE}"
    local _remote_ref
    if [ -n "${GH_REPO:-}" ]; then
        _remote_ref="${_branch}:${_file}"
        if git ls-remote "https://github.com/${GH_REPO}.git" "$_remote_ref" 2>/dev/null | grep -q .; then
            _af_log "🛑 MAINTENANCE flag set on remote ${_remote_ref} — skip"; exit 0
        fi
    fi
    if [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ]; then
        if git -C "$REPO_DIR" show "${_branch}:${_file}" >/dev/null 2>&1; then
            _af_log "🛑 MAINTENANCE flag set locally in ${REPO_DIR} — skip"; exit 0
        fi
    fi
    return 0
}

# ---------------------------------------------------------------------------
# has_label <labels_csv> <label_name> — есть ли метка в CSV-списке.
#
# Контракт: $1 — «a,b,c» (обычно уже в lowercase). Через `case`, без
# подпроцессов: на 5-минутном тике merge-gate это зовётся сотни раз.
# ---------------------------------------------------------------------------
has_label() {  # $1=labels_csv (lowercased) $2=label_name
    case ",${1}," in *",${2},"*) return 0 ;; *) return 1 ;; esac
}

# ---------------------------------------------------------------------------
# has_label_json <labels_json> <label_name> — есть ли метка в JSON-массиве.
#
# Контракт: $1 — вывод `gh issue view --json labels --jq '.labels'`, то есть
# компактный JSON вида [{"name":"hermes"},...] (gh печатает --jq компактно,
# без пробела после двоеточия — на этом держится образец поиска).
# ---------------------------------------------------------------------------
has_label_json() {  # $1=labels_json  $2=label_name
    printf '%s' "$1" | grep -q "\"name\":\"$2\""
}

# ---------------------------------------------------------------------------
# slugify <text> — kebab-case, только [a-z0-9-], максимум 40 символов.
# Используется для имён веток z-{agent}/<issue>-<slug>.
# ---------------------------------------------------------------------------
slugify() {
    printf '%s' "$1" \
        | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40
}

# ---------------------------------------------------------------------------
# gh_list_issues_by_label <label> [state] [limit] [fields]
#
# Печатает JSON-массив issue с меткой. Обёртка нужна из-за бага фильтрации
# по label в `gh issue list`: при некоторых состояниях кэша он отдаёт пустой
# массив там, где issue есть. Primary — gh-list, при пустом ответе fallback
# на REST /issues?labels=... с приведением к форме gh-list.
#
# ⚠️ Fallback возвращает ПОДМНОЖЕСТВО полей (number/title/labels/body/
#    updatedAt), а не произвольный $4 — REST отдаёт другую схему.
#
# Баг, живший во всех четырёх копиях до 30.08: маппинг делал
# `if "updatedAt" in it`, а REST отдаёт `updated_at` (проверено:
# `gh api repos/<owner>/<repo>/issues --jq '.[0]|keys'`). Условие не
# срабатывало никогда, поэтому на fallback-пути updatedAt отсутствовал —
# и agent-flow-deploy-sweep.sh:314 (`str(i["updatedAt"])`) падал с KeyError
# внутри `< <(python3 ...)`, то есть тихо: трейс в stderr, ноль обработанных
# issue, exit-код тика не меняется. Теперь читаем оба имени.
# ---------------------------------------------------------------------------
gh_list_issues_by_label() {
    local _label="$1" _state="${2:-open}"
    local _limit="${3:-${ISSUE_LIMIT:-${LIMIT:-20}}}"
    local _fields="${4:-number,title,labels,body,updatedAt}"
    local _json="" _api_json=""
    _json="$(gh issue list \
        --repo "$GH_REPO" \
        --label "$_label" \
        --state "$_state" \
        --limit "$_limit" \
        --json "$_fields" 2>/dev/null || true)"
    if [ -n "$_json" ] && [ "$_json" != "[]" ]; then
        printf '%s' "$_json"
        return 0
    fi
    _api_json="$(gh api "repos/${GH_REPO}/issues?labels=${_label}&state=${_state}&per_page=${_limit}" 2>/dev/null || true)"
    if [ -z "$_api_json" ] || [ "$_api_json" = "[]" ]; then
        printf '[]'
        return 0
    fi
    _af_log "gh_list_issues_by_label(${_label}): gh-list пустой, fallback на REST API /issues?labels=${_label}"
    printf '%s' "$_api_json" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    print("[]"); sys.exit(0)
if not isinstance(data, list):
    print("[]"); sys.exit(0)
keep = []
for it in data:
    if not isinstance(it, dict):
        continue
    if it.get("pull_request"):
        continue
    rec = {
        "number": it.get("number"),
        "title": it.get("title") or "",
        "labels": [{"name": (l.get("name") if isinstance(l, dict) else l)} for l in it.get("labels", [])],
        "body": it.get("body") or "",
    }
    # REST отдаёт updated_at (snake_case), gh-list — updatedAt. Потребители
    # (deploy-sweep, stale-conflicting-scan) ждут updatedAt, причём
    # deploy-sweep обращается по ключу жёстко, без .get().
    updated = it.get("updatedAt") or it.get("updated_at")
    if updated:
        rec["updatedAt"] = updated
    keep.append(rec)
print(json.dumps(keep, ensure_ascii=False))
'
}

# ---------------------------------------------------------------------------
# detect_pr_kind <pr_labels_csv> <pr_title> → печатает "lint" | "functional"
#
# "lint" = e2e на железе не нужен, зелёного CI достаточно.
# Источники сигнала по приоритету:
#   1) метка ${NO_E2E_LABEL} на PR — явный opt-out воркера
#   2) префикс `[lint]` / `[refactor]` в заголовке — сокращение воркера
#   3) `fix(agent-flow` / `fix(agent_flow` (ретро 13.08 t_de63be1f): фиксы
#      КОНВЕЙЕРА не меняют поведение робота. Раньше такие PR (#1189/#1190)
#      уходили в e2e-очередь как functional и застревали.
#   4) `docs(adr` / `docs(architecture` (ретро 24.08 t_388bb652): ADR-черновики
#      архитектора docs-only, runtime не трогают. Раньше #1577/#1580/#1581/
#      #1578 залипали с e2e:rejected (cold-start wake-gate, ретро t_d9e70587).
#   5) `wip(arch` / `wip(infra` (ретро 24.08 t_388bb652): PR #1559 висел
#      e2e:rejected 11ч49м без прогресса.
#   6) `wip(voice-core` (ретро 24.08 t_388bb652): verification-suite черновик.
#   7) иначе → functional (e2e обязателен)
#
# rc=0 всегда. Зависит от глобального $NO_E2E_LABEL и от has_label выше.
# ---------------------------------------------------------------------------
detect_pr_kind() {  # $1=labels_csv $2=title
    local labels_csv title_lc prefix
    labels_csv="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    title_lc="$(printf '%s' "$2" | tr '[:upper:]' '[:lower:]')"
    if has_label "$labels_csv" "$NO_E2E_LABEL"; then
        printf '%s' "lint"; return 0
    fi
    # Два независимых case'а:
    #   - по первому токену `prefix` для тегов без скобок: [lint], [refactor];
    #   - по всей строке с glob для conventional-commit префиксов.
    # Первый токен берём отдельно, потому что `(` и `)` внутри
    # docs(adr-0027) / wip(arch #1506) ломают extglob-группировку.
    prefix="${title_lc%% *}"
    case "$prefix" in
        '[lint]'|'[refactor]') printf '%s' "lint"; return 0 ;;
    esac
    case "$title_lc" in
        'fix(agent-flow'*|'fix(agent_flow'*|\
        'docs(adr'*|'docs(architecture'*|\
        'wip(arch'*|'wip(infra'*|'wip(voice-core'*)
            printf '%s' "lint"; return 0 ;;
    esac
    printf '%s' "functional"
}

# ---------------------------------------------------------------------------
# free_stale_worktrees_for <task_id> — снять чужие worktree на нашей ветке.
#
# Карточка держит свой worktree; если та же ветка занята worktree'ем другой
# (уже мёртвой) карточки, git не даст с ней работать. Сносим только чужие
# (basename != task_id) и только те, что стоят на НАШЕЙ ветке.
# Требует $HERMES_BIN и $KANBAN_BOARD в окружении.
# ---------------------------------------------------------------------------
free_stale_worktrees_for() {  # $1=task_id (t_<hex>)
    local task_id="$1" my_wt my_branch line wt_path wt_branch owner
    my_wt="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$task_id" --json 2>/dev/null \
        | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); print(d.get("task",{}).get("workspace_path") or "")
except Exception: print("")' 2>/dev/null || true)"
    if [ -z "$my_wt" ] || [ ! -d "$my_wt" ]; then
        return 0
    fi
    my_branch="$(git -C "$my_wt" branch --show-current 2>/dev/null || true)"
    [ -z "$my_branch" ] && return 0
    wt_path=""
    while IFS= read -r line; do
        case "$line" in
            worktree\ *) wt_path="${line#worktree }" ;;
            branch\ *)
                wt_branch="${line#branch refs/heads/}"
                if [ "$wt_branch" = "$my_branch" ] && [ "$wt_path" != "$my_wt" ]; then
                    owner="$(basename "$wt_path")"
                    if [ "$owner" != "$task_id" ]; then
                        git -C "$my_wt" worktree remove --force "$wt_path" 2>/dev/null \
                            && _af_log "  freed stale worktree $wt_path (branch $my_branch, card $owner)"
                    fi
                fi
                ;;
        esac
    done < <(git -C "$my_wt" worktree list --porcelain 2>/dev/null)
    git -C "$my_wt" worktree prune 2>/dev/null || true
    return 0
}
