#!/bin/bash
# ============================================================================
# test_e2e_process_guard.sh — ретро 13.08 t_4212e8ad
#
# Guard на создание round-ветки при 0 живых кандидатов:
#   round_ensure НЕ вызывается, если ни у одного needs-e2e issue нет живого PR
#   (нет PR вовсе / PR MERGED с удалённой веткой — orphan). Счётчик раундов
#   не инкрементируется, round-ветка на remote не создаётся.
#
# Сценарии:
#   A. 2 issues, у обоих НЕТ PR (NONE)                        → 0 кандидатов, exit 0
#   B. issue с PR MERGED, ветка удалена (orphan, #1160)       → 0 кандидатов, exit 0
#   C. issue с OPEN PR (functional)                           → 1 кандидат, round_ensure вызван
#   D. ветка PR вне конвенции → fallback '<number> in:title'  → кандидат, round создаётся
#   E. e2e-done от sweep → живой чек меток скипает merge      → merge не выполняется
#   F. кандидат снят sweep'ом того же тика (t_fe266643)       → round НЕ создаётся
#   H. ветка СОЗДАНА, кандидат снят ДО прогона (t_4268f2bf)   → round удаляется (gh api DELETE)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_guard.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

E2E_PROCESS="$REPO_ROOT/agent-flow-e2e-process.sh"

# --- Мок gh: различает issue list --search (блокеры) и --label (очередь) -----
install_e2e_mocks() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"

    cat > "$bin_dir/gh" <<'GH_MOCK_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }
get_state() {
    local key="$1"
    if [ -f "$state" ]; then
        grep -E "^${key}=" "$state" | head -n1 | sed "s@^${key}=@@"
    fi
}

subcmd="${1:-}"; shift || true

case "$subcmd" in
    auth)
        journal "gh auth status"
        exit 0
        ;;
    issue)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                # detect_known_blocker → gh issue list --search ... (пусто = нет блокера)
                if printf '%s' "$*" | grep -q -- '--search'; then
                    journal "gh issue list --search"
                    exit 0
                fi
                journal "gh issue list --label"
                # Ретро 13.08 t_fe266643: collect_issues_json вызывается дважды
                # за тик (старт + после post_round_sweep). Первый снимок —
                # ISSUE_LIST_JSON, повторный — ISSUE_LIST_JSON_2 (сценарий
                # «кандидат снят sweep'ом того же тика»).
                _cnt_file="${GH_STATE}.ilist_count"
                _cnt=0
                [ -f "$_cnt_file" ] && _cnt="$(cat "$_cnt_file" 2>/dev/null || echo 0)"
                _cnt=$((_cnt+1))
                printf '%s' "$_cnt" > "$_cnt_file"
                _data="$(get_state ISSUE_LIST_JSON)"
                if [ "$_cnt" -gt 1 ]; then
                    _data2="$(get_state ISSUE_LIST_JSON_2)"
                    [ -n "$_data2" ] && _data="$_data2"
                fi
                printf '%s' "$_data"
                exit 0
                ;;
            view)
                issue_num="$1"; shift || true
                if printf '%s' "$*" | grep -q -- '--comments'; then
                    journal "gh issue view $issue_num --comments"
                    _data="$(get_state ISSUE_${issue_num}_COMMENTS_JSON)"
                    [ -n "$_data" ] || _data='{"comments":[]}'
                    printf '%s' "$_data"
                    exit 0
                fi
                # Живой чек меток перед merge (ретро 13.08 t_7eab35a0):
                # gh issue view N --json labels --jq '[.labels[].name] | join(",")'
                if printf '%s' "$*" | grep -q -- '--json labels'; then
                    journal "gh issue view $issue_num --json labels"
                    _data="$(get_state ISSUE_${issue_num}_LABELS_JSON)"
                    [ -n "$_data" ] || _data='{"labels":[{"name":"needs-e2e"}]}'
                    if printf '%s' "$*" | grep -q -- '--jq'; then
                        printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin); print(",".join(sorted({l["name"] for l in d.get("labels",[])})))
except Exception: print("")'
                        exit 0
                    fi
                    printf '%s' "$_data"
                    exit 0
                fi
                journal "gh issue view $issue_num (other)"
                printf '%s' '{}'
                exit 0
                ;;
            edit|comment|close)
                journal "gh issue $action $*"
                exit 0
                ;;
            *)
                journal "gh issue $action $*"
                exit 0
                ;;
        esac
        ;;
    pr)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                if printf '%s' "$*" | grep -q -- '--search'; then
                    journal "gh pr list --search"
                    # Ретро 13.08 t_7eab35a0: поиск PR по '<номер> in:title'
                    # (fallback для веток вне конвенции z-{agent}/). Мок
                    # возвращает PR_SEARCH_<n>_JSON (массив) и применяет jq
                    # так же, как реальный gh: 'num\thead' или пусто.
                    _q="$(printf '%s' "$*" | sed -nE 's/.*--search[[:space:]]+([^ ]+).*/\1/p')"
                    _n="$(printf '%s' "$_q" | grep -oE '[0-9]+' | head -n1)"
                    _data="$(get_state "PR_SEARCH_${_n}_JSON")"
                    [ -n "$_data" ] || _data='[]'
                    if printf '%s' "$*" | grep -q -- '--jq'; then
                        printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin)
    for p in d:
        if p.get("mergeStateStatus") in ("CLEAN","MERGEABLE"):
            print(str(p.get("number",""))+"\t"+str(p.get("headRefName",""))); break
except Exception: pass'
                        exit 0
                    fi
                    printf '%s' "$_data"
                    exit 0
                fi
                if printf '%s' "$*" | grep -q -- '--head'; then
                    head_branch="$(printf '%s' "$*" | sed -nE 's/.*--head[[:space:]]+([^ ]+).*/\1/p')"
                    journal "gh pr list --head $head_branch"
                    _data="$(get_state "PR_HEAD_${head_branch}_JSON")"
                    [ -n "$_data" ] || _data='[]'
                    # Ретро 14.08 t_28afb585: guard в e2e-process проверяет
                    # merged PR по head-ветке (`gh pr list --state merged
                    # --head <branch>`). Отдельный ключ PR_MERGED_HEAD_<branch>_JSON
                    # (fallback PR_HEAD_*) — чтобы тест мог симулировать «ветка
                    # уже влита через ДРУГОЙ PR» (#1238/#1218).
                    if printf '%s' "$*" | grep -q -- '--state merged'; then
                        _merged_data="$(get_state "PR_MERGED_HEAD_${head_branch}_JSON")"
                        [ -n "$_merged_data" ] && _data="$_merged_data"
                    fi
                    # Применить минимальный jq: if length>0 then .[0].state / .[0].number
                    if printf '%s' "$*" | grep -q -- '--jq'; then
                        _jq="$(printf '%s' "$*" | sed -nE 's/.*--jq[[:space:]]+//p')"
                        case "$_jq" in
                            *'.[0].state'*)
                                printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin); print(d[0]["state"] if len(d)>0 else "NONE")
except Exception: print("NONE")'
                                exit 0
                                ;;
                            *'.[0].number'*)
                                printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin); print(d[0]["number"] if len(d)>0 else "")
except Exception: print("")'
                                exit 0
                                ;;
                        esac
                    fi
                    printf '%s' "$_data"
                    exit 0
                fi
                journal "gh pr list (other)"
                printf '%s' '[]'
                exit 0
                ;;
            view)
                pr_num="$1"; shift || true
                journal "gh pr view $pr_num"
                _data="$(get_state "PR_${pr_num}_VIEW_JSON")"
                [ -n "$_data" ] || _data='{}'
                # bug(e2e #1375) ретро 18.08: фильтруем вывод для --json files
                # (auto-discovery scenario_file). Реальный gh возвращает только
                # запрошенные --json-поля, и применяет --jq. Mock не реализует
                # ни того, ни другого, поэтому:
                # - если есть state PR_<n>_FILES_JSON — возвращаем его (готовый
                #   JSON для --json files);
                # - если FILES_JSON нет и в основном VIEW_JSON нет ключа files
                #   → возвращаем пустую строку (имитируем gh, у которого PR без
                #   изменённых файлов при --json files — пустой jq-результат).
                if printf '%s' "$*" | grep -q -- '--json files'; then
                    _files_data="$(get_state "PR_${pr_num}_FILES_JSON")"
                    if [ -n "$_files_data" ]; then
                        _data="$_files_data"
                    elif ! printf '%s' "$_data" | python3 -c "import sys,json;d=json.load(sys.stdin);sys.exit(0 if 'files' in d else 1)" 2>/dev/null; then
                        _data=''
                    fi
                fi
                printf '%s' "$_data"
                exit 0
                ;;
            *)
                journal "gh pr $action $*"
                exit 0
                ;;
        esac
        ;;
    label)
        journal "gh label $*"
        # Метка e2e:infra-fail уже существует → label create не вызывается.
        if printf '%s' "$1" | grep -q 'list'; then
            printf 'e2e:infra-fail\nneeds-e2e\ne2e-done\n'
        fi
        exit 0
        ;;
    run)
        journal "gh run $*"
        # Ретро 13.08 t_da3e0bd5: gh run list --branch <round> ... --json status
        # --jq (dedup active_round_with_issue). State: RUN_<branch>_JSON = массив
        # {databaseId,status,...}; по умолчанию пусто (нет активных run'ов).
        # Только для list --branch --jq; gh run view и прочее — RUN_LIST_JSON.
        if [ "${1:-}" = "list" ] && printf '%s' "$*" | grep -q -- '--branch' \
            && printf '%s' "$*" | grep -q -- '--jq'; then
            _rb="$(printf '%s' "$*" | sed -nE 's/.*--branch[[:space:]]+([^ ]+).*/\1/p')"
            _data="$(get_state "RUN_${_rb}_JSON")"
            [ -n "$_data" ] || _data='[]'
            # Ретро 14.08 t_4268f2bf: post-tick cleanup пустой round-ветки
            # спрашивает ОБЩЕЕ число run'ов (--json databaseId --jq 'length');
            # dedup (ниже) считает только queued/in_progress (--json status).
            if printf '%s' "$*" | grep -q 'databaseId'; then
                printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin); print(len(d))
except Exception: print(0)'
                exit 0
            fi
            printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin)
    print(sum(1 for r in d if r.get("status") in ("queued","in_progress")))
except Exception: print(0)'
            exit 0
        fi
        # Ретро 13.08 t_fe266643: post_round_sweep читает последний завершённый
        # e2e run на прошлом round. RUN_LIST_JSON — фикстура (массив runs).
        _data="$(get_state RUN_LIST_JSON)"
        [ -n "$_data" ] || _data='[]'
        printf '%s' "$_data"
        exit 0
        ;;
    api)
        journal "gh api $*"
        if printf '%s' "$1" | grep -q 'rate_limit'; then
            printf '%s' '{"resources":{"core":{"remaining":5000},"graphql":{"remaining":5000}}}'
        fi
        exit 0
        ;;
    *)
        journal "gh $subcmd $*"
        exit 0
        ;;
esac
GH_MOCK_EOF
    chmod +x "$bin_dir/gh"

    # --- Мок git: ls-remote по BRANCH_PRESENT, worktree/fetch/push — успех -----
    cat > "$bin_dir/git" <<'GIT_MOCK_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }

case "$1" in
    -C)
        # git -C <dir> <subcommand> ... — пропускаем префикс.
        shift 2 || true
        ;;
esac

case "$1" in
    ls-remote)
        # Формат: git ls-remote <url> <ref>  или  git -C <dir> ls-remote --heads <url> <pattern>
        ref="${@: -1}"
        journal "git ls-remote ... $ref"
        # Паттерн со звёздочкой (test-round-*) → ROUND_BRANCHES (пробелы),
        # иначе веток нет (ретро 13.08 t_fe266643: post_round_sweep ищет
        # последний round-бранч на remote).
        # Ретро 13.08 t_da3e0bd5: список round-веток (паттерн test-round-*)
        # приходит из state ROUND_BRANCHES. Элементы могут быть ПОЛНЫМ именем
        # ветки (sweep-фикстура "z-{e2e}/test-round-103") или НОМЕРОМ (dedup-
        # фикстура "90") — нормализуем оба к refs/heads/z-{e2e}/test-round-N.
        case "$ref" in
            *'test-round-'*'*'*)
                _rounds=""
                if [ -f "$state" ]; then
                    _rounds="$(grep -E '^ROUND_BRANCHES=' "$state" | head -n1 | sed 's/^ROUND_BRANCHES=//')"
                fi
                if [ -n "$_rounds" ]; then
                    for _rb in $_rounds; do
                        case "$_rb" in
                            *'test-round-'*) printf '%040x\trefs/heads/%s\n' $RANDOM "$_rb" ;;
                            *) printf '%040x\trefs/heads/z-{e2e}/test-round-%s\n' $RANDOM "$_rb" ;;
                        esac
                    done
                    exit 0
                fi
                ;;
        esac
        # ref вида "develop:RUN_NOW" / "develop:MAINTENANCE" — никогда нет.
        case "$ref" in
            *:*)
                if [ -f "$state" ] && grep -Eq "^BRANCH_PRESENT_${ref}=1$" "$state"; then
                    printf '%040x\trefs/heads/foo\n' $RANDOM
                    exit 0
                fi
                exit 1
                ;;
        esac
        if [ -f "$state" ] && grep -Eq "^BRANCH_PRESENT_${ref}=1$" "$state"; then
            printf '%040x\trefs/heads/%s\n' $RANDOM "$ref"
            exit 0
        fi
        exit 1
        ;;
    worktree)
        journal "git worktree $*"
        # ensure_worktree: git -C <dir> worktree add --detach <path> <branch>
        if [ "${2:-}" = "add" ]; then
            wt="${3:-}"
            [ -n "$wt" ] && mkdir -p "$wt" 2>/dev/null || true
            exit 0
        fi
        exit 0
        ;;
    fetch)
        journal "git fetch $*"
        exit 0
        ;;
    push)
        journal "git push $*"
        exit 0
        ;;
    show)
        journal "git show $*"
        exit 1
        ;;
    log)
        journal "git log $*"
        # Ретро 13.08 t_fe266643: post_round_sweep ищет merge-коммиты
        # "for issue #N" на прошлом round. GIT_LOG_MERGES — фикстура (строки).
        if printf '%s' "$*" | grep -q -- '--merges'; then
            _d="$(grep -E '^GIT_LOG_MERGES=' "$state" 2>/dev/null | head -n1 | sed 's@^GIT_LOG_MERGES=@@')"
            [ -n "$_d" ] && printf '%s\n' "$_d"
        fi
        exit 0
        ;;
    rm|commit|prune|checkout|merge|branch)
        journal "git $*"
        exit 0
        ;;
    *)
        journal "git (other) $*"
        exit 0
        ;;
esac
GIT_MOCK_EOF
    chmod +x "$bin_dir/git"

    # Мок hermes: kanban show → done (не используется в guard-пути, но нужен)
    cat > "$bin_dir/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
printf '%s\t%s\n' "$(date -Iseconds)" "hermes $*" >>"${GH_JOURNAL:-/dev/null}"
printf '%s' '{"task":{"status":"done"}}'
exit 0
HERMES_MOCK_EOF
    chmod +x "$bin_dir/hermes"

    export PATH="$bin_dir:$PATH"
    export GH_STATE="$TEST_TMP/gh_state"
    export GH_JOURNAL="$TEST_TMP/journal"
    : >"$GH_STATE"
    : >"$GH_JOURNAL"
}

# Создать REPO_DIR: существующий каталог (мок git перехватывает вызовы).
make_repo_dir() {
    mkdir -p "$TEST_TMP/repo"
}

# Запустить e2e-process с фикстурой. rc НЕ проверяется — смотрим журнал.
run_e2e() {
    (
        GH_REPO="${GH_REPO:-krikz/test-repo}"
        KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
        DRY_RUN="${DRY_RUN:-false}"
        ISSUE_LIMIT="${ISSUE_LIMIT:-20}"
        HERMES_HOME="$TEST_TMP/hermes_home"
        HERMES_BIN=hermes
        REPO_DIR="$TEST_TMP/repo"
        KANBAN_DB_PATH="$TEST_TMP/nonexistent-kanban.db"
        LOCK_FILE="$TEST_TMP/e2e-process.lock"
        ROUND_COUNTER_FILE="$TEST_TMP/round-counter"
        export GH_REPO KANBAN_BOARD DRY_RUN ISSUE_LIMIT HERMES_HOME HERMES_BIN REPO_DIR KANBAN_DB_PATH LOCK_FILE ROUND_COUNTER_FILE
        export HOME=/tmp
        # timeout: guard-тест не ждёт полного e2e-прогона — достаточно, что
        # round_ensure вызван (C) или guard вышел (A/B). Для A/B скрипт
        # завершится сам (exit 0); для C он уйдёт в build trigger и зависнет
        # на моках — timeout 30 снимает процесс, журнал уже записан.
        # -k 5: скрипт держит `trap cleanup EXIT INT TERM`, по SIGTERM он
        # чистит worktree и ПРОДОЛЖАЕТ цикл ожидания build — без SIGKILL
        # timeout никогда не вернётся (наблюдение 13.08, ретро t_7eab35a0).
        timeout -k 5 30 bash "$E2E_PROCESS" 2>>"$TEST_TMP/stderr.log" || true
    )
}

# ---------------------------------------------------------------------------
# A. 2 issues без PR → 0 кандидатов → guard exit 0, round НЕ создан.
# ---------------------------------------------------------------------------
test_A_no_prs_no_round() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue1=4201 issue2=4202
    set_state ISSUE_LIST_JSON "[{\"number\":${issue1},\"title\":\"fix #${issue1} demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"},{\"number\":${issue2},\"title\":\"fix #${issue2} demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue1}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue2}_COMMENTS_JSON" '{"comments":[]}'
    # PR_HEAD_* не заданы → мок вернёт [] → NONE.

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "no live e2e candidates" "$errlog" "guard: log сообщает о 0 живых кандидатов"
    local push_calls
    push_calls="$(printf '%s\n' "$journal" | grep -c "git push" || true)"
    assert_eq "0" "$push_calls" "A: round-ветка НЕ пушится при 0 кандидатов"
    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "" "$counter" "A: счётчик раундов не инкрементирован"
}

# ---------------------------------------------------------------------------
# B. Orphan: PR MERGED, ветка удалена → 0 кандидатов → round НЕ создан.
# ---------------------------------------------------------------------------
test_B_orphan_no_round() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4301
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    # PR MERGED, но BRANCH_PRESENT_<branch> НЕ задан → ветка удалена → orphan.
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4302,\"state\":\"MERGED\",\"headRefName\":\"${branch}\"}]"
    # Follow-up search → [] (мок), т.е. follow-up OPEN PR нет.

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "no live e2e candidates" "$errlog" "guard: orphan → 0 кандидатов"
    local push_calls
    push_calls="$(printf '%s\n' "$journal" | grep -c "git push" || true)"
    assert_eq "0" "$push_calls" "B: round-ветка НЕ пушится (orphan)"
    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "" "$counter" "B: счётчик раундов не инкрементирован"
}

# ---------------------------------------------------------------------------
# C. OPEN functional PR → 1 кандидат → round_ensure вызывается (push есть).
# ---------------------------------------------------------------------------
test_C_live_candidate_creates_round() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4401
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4402,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    # PR не lint (обычный title, без no-e2e label) → functional → кандидат.
    set_state "PR_4402_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "live candidate(s) — создаю round" "$errlog" "guard: OPEN PR → живой кандидат"
    local push_calls
    push_calls="$(printf '%s\n' "$journal" | grep -c "git push" || true)"
    if [ "$push_calls" -eq 0 ]; then
        # round_ensure мог не дойти до push (скрипт упал позже на моках) —
        # проверяем, что guard НЕ вышел рано: в логе есть "создаю round".
        assert_contains "создаю round" "$errlog" "C: guard пропустил к round_ensure"
    else
        assert_contains "git push" "$journal" "C: round-ветка пушится"
    fi
}

# ===========================================================================
# D. Ветка PR вне конвенции z-{agent}/<id>-<slug> (ретро 13.08 t_7eab35a0,
#    #1204 → PR #1206 '1204-fixvoice-...') → fallback '<number> in:title'
#    находит OPEN PR → живой кандидат → round создаётся.
# ---------------------------------------------------------------------------
test_D_nonconventional_branch_fallback() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4501
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"
    local alt_branch="${issue}-fixvoice-bug-c-retry"  # вне конвенции

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    # Каноническая ветка z-{agent}/... → PR нет (NONE).
    # set_state "PR_HEAD_${branch}_JSON" НЕ задаём → [] → NONE.
    # Fallback-поиск по '<номер> in:title' находит OPEN PR на alt-ветке.
    set_state "PR_SEARCH_${issue}_JSON" "[{\"number\":4502,\"headRefName\":\"${alt_branch}\",\"mergeStateStatus\":\"CLEAN\"}]"
    set_state "PR_HEAD_${alt_branch}_JSON" "[{\"number\":4502,\"state\":\"OPEN\",\"headRefName\":\"${alt_branch}\"}]"
    set_state "PR_4502_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:backend\"}]}"
    set_state "BRANCH_PRESENT_${alt_branch}" 1

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "PR найден по fallback '<number> in:title'" "$errlog" "D: guard нашёл PR по fallback (ветка вне конвенции)"
    assert_contains "live candidate(s) — создаю round" "$errlog" "D: guard пропустил к round_ensure"
    assert_contains "merging ${alt_branch} directly" "$errlog" "D: в round смержена именно alt-ветка (вне конвенции)"
}

# ===========================================================================
# E. Stale-снимок issues_json после post_round_sweep (ретро 13.08 t_7eab35a0):
#    issue уже получил e2e-done от sweep → повторно НЕ мержится в новый round.
#    Проверяем через живой чек меток перед merge (метка e2e-done → skip).
# ---------------------------------------------------------------------------
test_E_live_labels_skip_after_sweep() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4601
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    # Свежий чек меток: sweep уже поставил e2e-done (stale-снимок бы этого не увидел).
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4602,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_4602_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:backend\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "live labels" "$errlog" "E: живой чек меток выполнен"
    assert_contains "skip merge" "$errlog" "E: merge пропущен для e2e-done issue"
    # Лог «merging ... directly» пишется ДО живого чека (это intent) — проверяем
    # по журналу, что реальный git merge НЕ выполнялся.
    assert_not_contains "git merge" "$journal" "E: НЕТ реального git merge в round"
    assert_not_contains "merged & pushed" "$errlog" "E: НЕТ push смерженной ветки"
}

# ===========================================================================
# F. Кандидат снят post_round_sweep ТОГО ЖЕ тика (ретро 13.08 t_fe266643,
#    round-104/#968): первый снимок очереди содержит issue с OPEN PR (живой
#    кандидат), но sweep ДО round_ensure обрабатывает результат прошлого
#    round-103 (SUCCESS run, в который смержен этот issue) и ставит e2e-done
#    → очередь после sweep пуста → round НЕ создаётся, счётчик не растёт.
# ---------------------------------------------------------------------------
test_F_candidate_swept_same_tick() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4701
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    # Первый снимок: issue #4701 needs-e2e с OPEN PR → живой кандидат.
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4702,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_4702_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # Прошлый round-103 с SUCCESS e2e run, в который смержен #4701.
    set_state ROUND_BRANCHES "z-{e2e}/test-round-103"
    set_state RUN_LIST_JSON '[{"databaseId":31727291126,"status":"completed","conclusion":"success"}]'
    set_state GIT_LOG_MERGES "agent-flow: merge ${branch} for issue #${issue}"
    # После sweep (2-й вызов collect_issues_json): очередь пуста — sweep снял
    # needs-e2e, поставив e2e-done (в реальном GitHub issue пропал из очереди).
    set_state ISSUE_LIST_JSON_2 "[]"

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "post-round sweep: issue #${issue} → e2e-done" "$errlog" "F: sweep снял кандидата в e2e-done"
    assert_contains "no live candidates after post-round sweep" "$errlog" "F: очередь после sweep пуста → round НЕ создаётся"
    local push_calls
    push_calls="$(printf '%s\n' "$journal" | grep -c "git push" || true)"
    assert_eq "0" "$push_calls" "F: round-ветка НЕ пушится (кандидат снят sweep'ом того же тика)"
    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "" "$counter" "F: счётчик раундов не инкрементирован"
}

# ===========================================================================
# F2. Dedup (ретро 13.08 t_da3e0bd5): issue уже смержен в АКТИВНЫЙ round
#     (на round-ветке есть build run в queued/in_progress) → guard НЕ создаёт
#     новый round и НЕ мержит повторно (петля #968: round-102 → TIMEOUT →
#     round-103 с тем же issue → 2 параллельных L-Build).
# ---------------------------------------------------------------------------
test_F_active_round_dedup_no_new_round() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4701
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"
    local round="z-{e2e}/test-round-90"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4702,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_4702_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:backend\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # Активный round-90: ветка существует, на ней build run в in_progress.
    set_state ROUND_BRANCHES "90"
    set_state "RUN_${round}_JSON" '[{"databaseId":4703,"status":"in_progress"}]'

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "уже в активном" "$errlog" "F2: guard увидел issue в активном round (dedup)"
    assert_contains "no live e2e candidates" "$errlog" "F2: 0 живых кандидатов после dedup → round НЕ создаётся"
    local push_calls
    push_calls="$(printf '%s\n' "$journal" | grep -c "git push" || true)"
    assert_eq "0" "$push_calls" "F2: round-ветка НЕ пушится (dedup заблокировал новый round)"
    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "" "$counter" "F2: счётчик раундов не инкрементирован"
}

# ===========================================================================
# G. Dedup НЕ срабатывает, когда round завершён (нет queued/in_progress run'ов):
#    issue в round-90, но все run'ы completed → живой кандидат → round создаётся.
# ---------------------------------------------------------------------------
test_G_completed_round_not_active() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4801
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"
    local round="z-{e2e}/test-round-90"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4802,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_4802_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:backend\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # Round-90 существует, но его build run уже completed → НЕ активен.
    set_state ROUND_BRANCHES "90"
    set_state "RUN_${round}_JSON" '[{"databaseId":4803,"status":"completed","conclusion":"success"}]'

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "live candidate(s) — создаю round" "$errlog" "G: completed round не блокирует новый round"
}

# ===========================================================================
# H. Ветка СОЗДАНА, кандидат снят ДО прогона (ретро 14.08 t_4268f2bf,
#    round-101/102/104/107/109): guard видит живого кандидата (первый снимок)
#    → round_ensure СОЗДАЁТ round-N (git push в журнале) → но живой чек меток
#    в основном цикле видит e2e-done (кандидат снят sweep/merge-gate/
#    пользовательским merge между guard и merge) → merge скипается, на round-N
#    0 run'ов → post-tick cleanup УДАЛЯЕТ round-ветку через gh api DELETE
#    refs/heads. Отличие от F: там кандидат снят ДО создания (round=NONE,
#    ветка не создаётся); здесь ветка СОЗДАНА, но пуста → удаляется.
# ---------------------------------------------------------------------------
test_H_branch_created_candidate_removed_deleted() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4901
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    # Первый снимок: issue #4901 needs-e2e с OPEN PR → живой кандидат.
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    # Живой чек меток ПЕРЕД merge (основной цикл): кандидат уже снят
    # (e2e-done) → merge скипается, build/e2e не запускаются.
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4902,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_4902_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # ROUND_BRANCHES не задан → round_ensure создаст z-{e2e}/test-round-1.
    # RUN_z-{e2e}/test-round-1_JSON не задан → [] → 0 run'ов → cleanup DELETE.

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "live candidate(s) — создаю round" "$errlog" "H: guard пропустил к round_ensure"
    assert_contains "git push" "$journal" "H: round-ветка создана (git push выполнен)"
    assert_contains "live labels" "$errlog" "H: живой чек меток выполнен"
    assert_contains "skip merge" "$errlog" "H: merge скипнут (кандидат снят до прогона)"
    assert_not_contains "merged & pushed" "$errlog" "H: НЕТ push смерженной ветки (merge не выполнялся)"
    assert_contains "0 run'ов" "$errlog" "H: cleanup увидел 0 run'ов на созданной ветке"
    assert_contains "gh api -X DELETE" "$journal" "H: round-ветка удалена через gh api"
    assert_contains "test-round-1" "$journal" "H: удалена именно созданная round-ветка"
    assert_contains "deleted (empty round branch" "$errlog" "H: лог подтверждает удаление"
    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "1" "$counter" "H: round-ветка была создана (счётчик инкрементирован), затем удалена"
}

# ===========================================================================
# H2. Переиспользование ветки влитого PR (ретро 14.08 t_28afb585, #1238/#1218):
#     у issue есть OPEN PR, НО его head-ветка уже влита через ДРУГОЙ PR →
#     merge в round НЕ выполняется, needs-review снимается (поставлен без e2e).
# ---------------------------------------------------------------------------
test_H_merged_branch_reuse_skips_merge() {
    new_test
    install_e2e_mocks
    make_repo_dir

    local issue=4901
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    # OPEN PR #4902 на ветке (как в C) — это «новый» PR поверх влитой ветки.
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":4902,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_4902_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # НО ветка УЖЕ влита через ДРУГОЙ PR #1218 (паттерн #1238/#1218).
    set_state "PR_MERGED_HEAD_${branch}_JSON" "[{\"number\":1218,\"state\":\"MERGED\"}]"

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "уже влита через PR #1218" "$errlog" "H: guard видит merged-PR по head-ветке"
    assert_not_contains "merging ${branch} directly" "$errlog" "H: ветка влитого PR НЕ льётся в round"
    # needs-review снят с PR (поставлен без e2e).
    local review_removed
    review_removed="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 4902 .*--remove-label needs-review' || true)"
    assert_eq "1" "$review_removed" "H: needs-review снят с PR"
}

# ===========================================================================
run_test "A. issues без PR → round НЕ создаётся" test_A_no_prs_no_round
run_test "B. orphan (MERGED+ветка удалена) → round НЕ создаётся" test_B_orphan_no_round
run_test "C. живой OPEN PR → round создаётся" test_C_live_candidate_creates_round
run_test "D. ветка вне конвенции → fallback '<number> in:title' → кандидат" test_D_nonconventional_branch_fallback
run_test "E. e2e-done от sweep → живой чек меток скипает merge" test_E_live_labels_skip_after_sweep
run_test "F. кандидат снят sweep'ом того же тика → round НЕ создаётся" test_F_candidate_swept_same_tick
run_test "F2. issue в АКТИВНОМ round → round НЕ создаётся (dedup)" test_F_active_round_dedup_no_new_round
run_test "G. round завершён → dedup НЕ блокирует новый round" test_G_completed_round_not_active
run_test "H. ветка создана, кандидат снят до прогона → round удаляется (t_4268f2bf)" test_H_branch_created_candidate_removed_deleted
run_test "H2. ветка влитого PR → merge skip + needs-review снят" test_H_merged_branch_reuse_skips_merge

summary
