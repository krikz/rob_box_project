#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-triage.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
# ============================================================================
# agent-flow-triage.sh — Phase 1 agent-flow: GitHub Issues -> Hermes Kanban cards.
#
# Pure bash. No LLM. Idempotent. Driven entirely by env (see ~/.hermes/profiles/agent-flow/.env).
#
# Pipeline per tick:
#   1. MAINTENANCE gate (remote + local)  -> exit 0 if paused
#   2. gh auth check                       -> exit 1 if not authed
#   3. List open issues with label $ISSUE_LABEL on $GH_REPO   (Phase 1)
#   4. List open issues with label $GSD_SOURCE_LABEL on $GH_REPO (Phase 2, retro t_360dc1a4)
#      — оставляем только те, у которых нет метки $ISSUE_LABEL (GSD-orphans)
#   5. For each issue (оба этапа используют один pipeline process_issues_json):
#        G9a (intra-tick dedup — ретро t_dfd3d19d, ADR-0032): на этапе
#             загрузки issues_json для каждой фазы группы issues с одинаковыми
#             (sorted-labels, first-N-words-of-title) схлопываются — оставляем
#             старейшую по number, остальные skip+comment+label
#             «agent-flow:dedup-skip» (DRY_RUN — skip без side-effect).
#             Реализован в Python pre-pass над issues_json (ниже).
#        a. Skip if it already has $DONE_LABEL (e2e-done — work complete)
#        b. Skip if comment marker `kanban: t_<id>` already present (idempotency)
#        c. Skip if a card with `issue: #N` in body already exists (idempotency v2)
#        G9b (race-window dedup — ретро t_dfd3d19d, ADR-0032): если
#             вычисленная ветка `z-{agent}/<N>-<slug>` УЖЕ ЕСТЬ в remote refs
#             (предыдущий tick запушил, или параллельный worker) — карточка
#             создаст worker'а, который не сможет открыть ту же ветку
#             (`git worktree add` → «already checked out»). Skip+inc
#             dedup_race_skipped. Использует `git ls-remote
#             https://github.com/${GH_REPO}.git refs/heads/${branch}` (см. G1).
#             Реализован в process_issues_json — функция branch_exists_in_remote.
#        d. Skip if the would-be branch already has a MERGED PR (work in develop)
#        e. Resolve role from `agent:<role>` label, else $AGENT_FLOW_DEFAULT_ROLE
#        f. Compute branch name (z-{agent}/<issue>-<slug> or z-{<slug>} for service/infra)
#        g. `hermes kanban create` with --workspace worktree --branch $branch
#        h. Comment the new t_<id> into the issue (3x retry, exp-backoff)
#   6. flock lock prevents parallel ticks.
#
# Gates G2..G8 follow the table in ~/.hermes/profiles/agent-flow/skills/.../SKILL.md.
# G9 — ретро-фикс t_dfd3d19d (26.08, 4-я повторяющаяся дупликат-серия:
# #1477/#1478 → #1506 → #1562/#1563 → #1650/#1653/#1655/#1658); см. ADR-0032.
# G8 (fingerprint dedup, t_b0fe4398) и G9 (intra-tick+race) дополняют друг друга:
# G8 ловит «разные issues → один fix (added-lines fingerprint)»;
# G9 ловит «один root-cause → разные issues в одном тике» + «потерянная ветка в remote».

set -euo pipefail

# --- defaults (overridden by env / .env) -------------------------------------
# NOTE: hardcode /home/builder/.hermes — this script is owned by the host
# hermes install, not by the calling profile's $HOME (cron may spawn us from
# the agent-flow profile where $HOME is profile-relative).
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"

# Force HOME=/home/builder so that gh CLI and hermes binaries (which look in
# $HOME/.config/gh and $HOME/.hermes respectively) resolve to the real user
# install, not the per-profile $HOME that cron sets via build_subprocess_env.
export HOME=/home/builder
ISSUE_LABEL="${ISSUE_LABEL:-hermes}"
DONE_LABEL="${DONE_LABEL:-e2e-done}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
AGENT_FLOW_DEFAULT_ROLE="${AGENT_FLOW_DEFAULT_ROLE:-architect}"
AGENT_FLOW_MAX_RUNTIME="${AGENT_FLOW_MAX_RUNTIME:-1800}"
# Ретро-фикс (09.08 #2): крупные задачи (priority:P0 или объёмный body) получают
# увеличенный --max-runtime — иначе воркер упирается в бюджет на полпути.
AGENT_FLOW_MAX_RUNTIME_LARGE="${AGENT_FLOW_MAX_RUNTIME_LARGE:-3600}"
# Порог "объёмного" body issue (символов) — грубый прокси размера задачи.
AGENT_FLOW_LARGE_BODY_CHARS="${AGENT_FLOW_LARGE_BODY_CHARS:-2000}"
AGENT_FLOW_MAX_RETRIES="${AGENT_FLOW_MAX_RETRIES:-2}"
# Ретро-фикс (26.08 t_dfd3d19d, ADR-0032): intra-tick dedup (G9a).
# Группы issues с одинаковыми (sorted-labels, first-N-words-of-title) схлопы-
# ваются в одну — оставляем старейшую по number, остальные skip+comment
# «agent-flow:dedup-skip». Длина prefix (в словах, default 6) — баланс:
# слишком короткий → одинаковые группы у разных багов; слишком длинный →
# пропускаем реальные дубли (#1477 «STT empty on echo» vs #1478 «echo STT
# empty detected» — префиксы совпадают в первых 3-4 словах).
AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS="${AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS:-6}"
# Ретро-фикс (26.08 t_dfd3d19d, ADR-0032): race-window dedup (G9b).
# Если вычисленная ветка `z-{agent}/<N>-<slug>` уже есть в remote refs (предыдущий
# tick запушил, или параллельный worker) — skip. Disable через
# AGENT_FLOW_DEDUP_RACE_GUARD=false (для тестов / отладки).
AGENT_FLOW_DEDUP_RACE_GUARD="${AGENT_FLOW_DEDUP_RACE_GUARD:-true}"
# ADR-0013 (docs/adr/0013-incremental-delivery-over-big-bang.md): PR > 50
# коммитов ИЛИ > 3000 строк запрещён без метки `big-bang-override` на issue.
# Triage проверяет это ДО `hermes kanban create` — если к issue уже привязан
# огромный PR (например, воркер случайно запушил 100 коммитов до того как
# merge-gate успел среагировать), мы НЕ создаём новую карточку на тот же issue
# (worker всё равно упрётся в merge-gate → бессмысленная работа). Шифу
# ставит метку вручную, чтобы явно разрешить.
BIG_BANG_OVERRIDE_LABEL="${BIG_BANG_OVERRIDE_LABEL:-big-bang-override}"
BIG_BANG_MAX_COMMITS="${BIG_BANG_MAX_COMMITS:-50}"
BIG_BANG_MAX_LINES="${BIG_BANG_MAX_LINES:-3000}"
# Ретро-фикс (26.08 t_b0fe4398, issues #1650/#1653/#1655/#1658): 4 devops-воркера
# за 5ч открыли 4 PR на одну и ту же проблему (opt-in quest svc через compose
# profile) на РАЗНЫХ ветках. OPEN-PR guard (per-branch) и throttle (per-issue)
# не ловили — у каждого issue свой PR. Throttle v3 (4h-window) не видит
# `kanban: t_*` от archived-карточек и тоже не помог.
#
# Решение (G8 — fingerprint dedup gate): перед `hermes kanban create` ищем
# в OPEN PR-ах такой же fix-fingerprint (sha256 от added-lines в whitelist
# файлах). Если ≥1 OPEN PR уже делает тот же fix → issue = дубль → skip +
# comment + label `agent-flow-error: duplicate-fix`.
#
# Whitelist — файлы, где devops-воркер обычно делает одно-строчный фикс
# (compose, .env.example, package.xml, setup.py, Dockerfile, install/setup).
# Список glob'ов через `|`, проверяется case-функцией `file_in_fp_whitelist`.
FINGERPRINT_FILE_GLOBS="${FINGERPRINT_FILE_GLOBS:-docker/*/docker-compose.yaml|docker/*/.env.example|docker/*/Dockerfile|docker/*/setup.sh|docker/*/install.sh|src/*/package.xml|src/*/setup.py|install/setup*.sh}"
# Сколько существующих OPEN PR с ТЕМ ЖЕ fix-fingerprint достаточно, чтобы
# считать это дубликатом (1 = любой существующий PR с тем же фиксом → дубль).
FINGERPRINT_DUPLICATE_THRESHOLD="${FINGERPRINT_DUPLICATE_THRESHOLD:-1}"
DRY_RUN="${DRY_RUN:-false}"
ISSUE_LIMIT="${ISSUE_LIMIT:-50}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-triage.lock}"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-triage]}"

# --- source profile .env if present -----------------------------------------
# Precedence: caller env > .env > defaults. We do NOT use `set -a` because
# that would clobber caller overrides (matters for tests / cron flags).
PROFILE_ENV="${HERMES_HOME}/profiles/agent-flow/.env"
if [ -f "$PROFILE_ENV" ]; then
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
    done < "$PROFILE_ENV"
fi

# Re-apply defaults for any vars still empty (defensive — .env may be partial).
: "${KANBAN_BOARD:=robbox}"
: "${MAINTENANCE_BRANCH:=develop}"
: "${MAINTENANCE_FILE:=MAINTENANCE}"
: "${REPO_DIR:=}"
: "${AGENT_FLOW_DEFAULT_ROLE:=architect}"
: "${AGENT_FLOW_MAX_RUNTIME:=1800}"
: "${AGENT_FLOW_MAX_RUNTIME_LARGE:=3600}"
: "${AGENT_FLOW_LARGE_BODY_CHARS:=2000}"
: "${AGENT_FLOW_MAX_RETRIES:=2}"
: "${AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS:=6}"
: "${AGENT_FLOW_DEDUP_RACE_GUARD:=true}"
: "${BIG_BANG_OVERRIDE_LABEL:=big-bang-override}"
: "${BIG_BANG_MAX_COMMITS:=50}"
: "${BIG_BANG_MAX_LINES:=3000}"
: "${FINGERPRINT_FILE_GLOBS:=docker/*/docker-compose.yaml|docker/*/.env.example|docker/*/Dockerfile|docker/*/setup.sh|docker/*/install.sh|src/*/package.xml|src/*/setup.py|install/setup*.sh}"
: "${FINGERPRINT_DUPLICATE_THRESHOLD:=1}"
: "${DRY_RUN:=false}"
: "${ISSUE_LABEL:=hermes}"
: "${DONE_LABEL:=e2e-done}"
: "${ISSUE_LIMIT:=50}"

# --- helpers -----------------------------------------------------------------
log()  { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
run()  { if [ "$DRY_RUN" = "true" ]; then printf '%s DRY-RUN %s\n' "$LOG_PREFIX" "$*"; else eval "$@"; fi; }

# self-id / whoami helper (issue #1534): перед side-effect'ами на issue
# (label-changes, добавляет `agent-flow-error` при провале kanban create)
# этот скрипт пишет «🤖 [agent:<role>] script=… action=…» чтобы в истории
# GitHub было видно КТО это сделал, а не только krikz (actor = holder of
# GH token). HERMES_AGENT_ROLE дефолтится в «agent:devops», переопределяется
# env из profile .env. Идемпотентность: helper скипает дубль в окне 2ч.
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=hermes_github.sh
. "$_LIB_DIR_HERE/hermes_github.sh"

# flock: skip tick if another instance holds the lock.
exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; exit 1; }
if ! flock -n 9; then
    log "another instance holds $LOCK_FILE — skip"; exit 0
fi

# --- G1: MAINTENANCE gate (remote + local) -----------------------------------
if [ -n "${GH_REPO:-}" ]; then
    remote_ref="${MAINTENANCE_BRANCH}:${MAINTENANCE_FILE}"
    if git ls-remote "https://github.com/${GH_REPO}.git" "$remote_ref" 2>/dev/null | grep -q .; then
        log "🛑 MAINTENANCE flag set on remote ${remote_ref} — skip"; exit 0
    fi
fi
if [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ]; then
    if git -C "$REPO_DIR" show "${MAINTENANCE_BRANCH}:${MAINTENANCE_FILE}" >/dev/null 2>&1; then
        log "🛑 MAINTENANCE flag set locally in ${REPO_DIR} — skip"; exit 0
    fi
fi

# --- G2: gh auth check -------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    log "gh auth not configured — exit 1"; exit 1
fi

# --- required env ------------------------------------------------------------
: "${GH_REPO:?GH_REPO must be set (owner/repo)}"

# --- gh_list_issues_by_label (ретро 19.08 #1457) ------------------------------
# Fallback для `gh issue list --label X` (GraphQL-фильтр по label ломается на
# некоторых версиях gh CLI). При пустом ответе gh-list — пробуем REST API
# /issues?labels=X. Возвращает JSON-массив с полями: number,title,labels,body.
gh_list_issues_by_label() {
    local _label="$1" _state="${2:-open}" _limit="${3:-${ISSUE_LIMIT}}" _fields="${4:-number,title,labels,body}"
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
    log "gh_list_issues_by_label(${_label}): gh-list пустой, fallback на REST API /issues?labels=${_label}"
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
    if "updatedAt" in it:
        rec["updatedAt"] = it.get("updatedAt")
    keep.append(rec)
print(json.dumps(keep, ensure_ascii=False))
'
}

# --- Phase 1: primary filter (label=$ISSUE_LABEL, defaults to "hermes") ------
# Ретро t_360dc1a4: до этого фикса triage фильтровал ТОЛЬКО по label 'hermes'.
# Issue #1643 (Phase 1.7 e2e smoke) был создан БЕЗ метки 'hermes' (только
# source:gsd + needs-e2e + area:rob_box_quest + phase:1.7) → провалился через
# фильтр → 10ч+ orphan без kanban-карточки и PR.
#
# Основной цикл: Phase 1 обрабатывает hermes-flagged issues. Phase 2 (ниже)
# ловит GSD-orphans: issues с source:gsd БЕЗ метки hermes (специфика GSD-
# workflow: он выставляет source:gsd, но не всегда hermes — только для issues,
# прошедших первичный triage-фильтр бота).
#
# NOTE: Phase 1/2 invocations and the G3 rate-limit guard живут НИЖЕ,
# после определения process_issues_json (~L608). bash требует, чтобы
# функция была определена ДО первого вызова.
phase1_json=""

# --- branch-naming helpers --------------------------------------------------
slugify() {
    # lowercase, replace non-alnum with -, collapse, trim, kebab-case, cap 40
    printf '%s' "$1" \
        | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40
}

branch_for() {  # $1=labels_json  $2=issue_number  $3=title
    labels_norm="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    if printf '%s' "$labels_norm" | grep -Eq 'service:|infra:|ops:'; then
        printf 'z-{%s}' "$(slugify "$3")"
    else
        printf 'z-{agent}/%s-%s' "$2" "$(slugify "$3")"
    fi
}

# branch_label_override — извлечь имя ветки из явной метки `branch:NAME`
# (ретро 22.08 t_8cde8449, issue #1506). Если метка есть — это ground truth
# от товарища Шифу: «работай на этой ветке, не выдумывай свою». Используется
# pre-create guard'ом ниже: если на этой ветке уже есть OPEN PR — карточка
# не нужна (работа и так в PR).
#
# Формат метки: `branch:<name>` где <name> = [a-z0-9_/{}-]+ (как git ref-name,
# включая формат agent-flow `z-{agent}/...`). Регистронезависимо. Возвращает
# пустую строку, если метки нет — backward-compat (текущее поведение branch_for
# сохраняется).
branch_label_override() {  # $1=labels_json
    # Строгий парсинг: метка должна быть самостоятельным label (отделена
    # запятой или быть единственной). Это защищает от случайных совпадений
    # вроде "mybranch:foo" или "abranch:bar".
    #
    # Алгоритм: нормализуем в lowercase, разбиваем по запятой, ищем точное
    # совпадение префикса "branch:" в начале элемента. Если нашли — печатаем
    # остаток, обрезав "branch:".
    printf '%s' "$1" \
        | tr '[:upper:]' '[:lower:]' \
        | tr ',' '\n' \
        | awk -F: '
            /^branch:/ {
                sub(/^branch:/, "")
                print
                exit
            }
        ' \
        || true
}

role_for() {  # $1=labels_json
    printf '%s' "$1" \
        | grep -oE 'agent:[a-z0-9_-]+' \
        | head -n1 \
        | sed 's/^agent://' \
        || printf '%s' "$AGENT_FLOW_DEFAULT_ROLE"
}

# branch_exists_in_remote — ретро-фикс (26.08 t_dfd3d19d, ADR-0032): G9b
# race-window dedup. Проверяет, существует ли уже ветка $1 в remote refs.
# Использует тот же механизм, что G1 MAINTENANCE gate (line 189): `git ls-remote
# https://github.com/${GH_REPO}.git refs/heads/${branch}`. Возвращает 0 если
# ветка существует, 1 если нет, fail-OPEN если gh недоступен (чтобы не
# блокировать нормальный поток из-за сетевого глюка).
#
# Пример:
#   if branch_exists_in_remote "z-{agent}/1477-stt-empty-on-echo"; then
#       log "branch уже в remote — skip kanban-create"
#       skipped=$((skipped+1)); continue
#   fi
#
# Disabling через AGENT_FLOW_DEDUP_RACE_GUARD=false (env-переменная).
branch_exists_in_remote() {  # $1=branch
    local branch="$1"
    [ -n "${GH_REPO:-}" ] || return 1   # нет GH_REPO → нечего проверять → skip
    [ "${AGENT_FLOW_DEDUP_RACE_GUARD:-true}" = "true" ] || return 1
    [ -n "$branch" ] || return 1
    # git ls-remote печатает tab-delimited: refs/heads/<branch>\t<sha>.
    # Пустой результат (или код !=0) означает, что ветки нет.
    if git ls-remote "https://github.com/${GH_REPO}.git" "refs/heads/${branch}" 2>/dev/null \
        | grep -q "^${branch}[[:space:]]" ; then
        return 0
    fi
    return 1
}

# dedup_intra_filter — ретро-фикс (26.08 t_dfd3d19d, ADR-0032): G9a intra-tick
# dedup pre-pass. Принимает issues_json (из gh_list_issues_by_label или REST
# fallback) и возвращает новый issues_json, в котором для каждой группы issues
# с одинаковыми (sorted-labels, first-N-words-of-title) оставлена только
# старейшая по number. Остальным — comment+label «agent-flow:dedup-skip».
#
# Аргументы:
#   $1 — phase_label ("phase1" / "phase2") — только для логов
#   $2 — issues_json (входной JSON-массив, формат как у gh_list_issues_by_label)
#
# Использование:
#   g9_filtered="$(dedup_intra_filter "phase1" "$phase1_json" 2>"$g9_err_file")"
#   g9_count="$(awk -F'[= ]' '/^DEDUP_INTRA:/{c++} END{print c+0}' "$g9_err_file")"
#   dedup_intra_skipped=$((dedup_intra_skipped + g9_count))
#   phase1_json="$g9_filtered"
#
# Вывод: stdout = JSON-array kept-issues; stderr → "$g9_err_file" (read by caller).
dedup_intra_filter() {  # $1=phase_label  $2=issues_json
    local phase_label="$1" issues_in="$2"
    [ -n "$issues_in" ] || { printf '%s' ""; return 0; }
    [ "$issues_in" != "[]" ] || { printf '%s' "[]"; return 0; }

    # shellcheck disable=SC2016  # python heredoc — ${...} выглядит как vars, но это env
    __PHASE_LABEL="$phase_label" \
    __TITLE_WORDS="${AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS}" \
    __DRY_RUN="${DRY_RUN:-false}" \
    __GH_REPO="${GH_REPO:-}" \
    printf '%s' "$issues_in" | python3 -c '
import json, os, re, subprocess, sys

PHASE_LABEL = os.environ.get("__PHASE_LABEL", "?")
TITLE_WORDS = int(os.environ.get("__TITLE_WORDS", "6") or "6")
DRY_RUN = (os.environ.get("__DRY_RUN", "false").lower() == "true")
GH_REPO = os.environ.get("__GH_REPO", "")

try:
    raw = sys.stdin.read()
    issues = json.loads(raw)
except Exception as e:
    sys.stderr.write("dedup_intra_filter: parse failed: %s, returning input as-is (fail-OPEN)\n" % e)
    sys.stdout.write(raw if raw else "[]")
    sys.exit(0)

if not isinstance(issues, list) or not issues:
    sys.stdout.write("[]")
    sys.exit(0)

def title_prefix(t, n):
    s = (t or "").lower()
    tokens = re.findall(r"[\w]+", s, flags=re.UNICODE)
    return " ".join(tokens[:n])

def sorted_labels(issue):
    """Возвращает sorted-list имён меток через запятую."""
    labels = issue.get("labels", []) or []
    names = []
    for lab in labels:
        if isinstance(lab, dict):
            n = lab.get("name", "")
        else:
            n = str(lab)
        if n:
            names.append(n)
    return ",".join(sorted(set(names)))

groups = {}
for it in issues:
    if not isinstance(it, dict):
        continue
    n = it.get("number")
    if not isinstance(n, int):
        continue
    key = sorted_labels(it) + "||" + title_prefix(it.get("title", ""), TITLE_WORDS)
    groups.setdefault(key, []).append(it)

leader_nums = set()
skips = []
for _key, recs in groups.items():
    if len(recs) == 1:
        leader_nums.add(recs[0].get("number"))
    else:
        sorted_recs = sorted(recs, key=lambda x: x.get("number", 0))
        leader = sorted_recs[0]
        leader_nums.add(leader.get("number"))
        for r in sorted_recs[1:]:
            skips.append((r, leader.get("number")))

# Emit лидеров в ИСХОДНОМ input-order для стабильности при повторных тиках.
emitted = set()
kept = []
for it in issues:
    if it.get("number") in leader_nums and it.get("number") not in emitted:
        kept.append(it)
        emitted.add(it["number"])

# Output на stdout: keep-лидеров как JSON-array (тот же формат, что потребляет
# process_issues_json на входе).
sys.stdout.write(json.dumps(kept, ensure_ascii=False))

# Логирование SKIP-маркеров в stderr (tab-separated для outer-парсера).
# Side-effects (gh comment/edit) делаются прямо здесь — fail-OPEN (try/except).
for r, leader_num in skips:
    n = r.get("number")
    tp = title_prefix(r.get("title", ""), TITLE_WORDS)
    lbl = sorted_labels(r)
    sys.stderr.write("DEDUP_INTRA\tphase=%s\tskip=%s\tleader=%s\ttitle_prefix=%s\tlabels=%s\n" %
                     (PHASE_LABEL, n, leader_num, tp[:80], lbl[:80]))
    if not DRY_RUN and GH_REPO:
        comment_body = (
            "agent-flow:dedup-skip (intra-tick, ретро t_dfd3d19d, ADR-0032)\n\n"
            "Triage определил этот issue как дубликат #%d в текущем тике (%s) — оба имеют "
            "идентичный набор меток и совпадающее начало заголовка («%s»). "
            "Карточка будет создана для #%d, для этого — НЕ создаём.\n\n"
            "Если это разные баги (одинаковое начало заголовка — совпадение):\n"
            "1. **товарищ Шифу**: добавь уникальный distinguishing label (например "
            "`distinguish:<короткий-тег>`) к одному из issues — дедуп их разнесёт.\n"
            "2. Либо обнови title так, чтобы первые %d слов не совпадали.\n\n"
            "Карточка для issue #%d будет создана на следующем тике после re-triage, если label добавить."
            % (leader_num, PHASE_LABEL, tp, leader_num, TITLE_WORDS, n)
        )
        for argv in (
            ["gh", "issue", "comment", str(n), "--repo", GH_REPO, "--body", comment_body],
            ["gh", "issue", "edit", str(n), "--repo", GH_REPO, "--add-label", "agent-flow-error"],
            ["gh", "issue", "edit", str(n), "--repo", GH_REPO, "--add-label", "agent-flow-dedup-skip"],
        ):
            try:
                subprocess.run(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20, check=False)
            except Exception:
                pass
'
}

# Ретро-фикс (19.08, t_dd7a5749): assignee-existence guard.
# Профиль `triager` НЕ существует в /home/builder/.hermes/profiles/ — карточка
# t_1ca827a6 (issue #1444) висела в ready 2.4ч потому что dispatcher не нашёл
# worker-pool для assignee=triager. Раньше triage без проверки радостно создавал
# карточку с любым assignee из label `agent:<role>` или дефолтом, и она навсегда
# зависала в ready. Теперь — проверяем против реального списка профилей через
# `hermes profile list` (одна команда, кэшируется на тик в $VALID_PROFILES).
#
# Дефект был в обоих путях:
#   - role из label `agent:triager` → профиль не существует
#   - role из AGENT_FLOW_DEFAULT_ROLE (architect) → всегда валиден
# Поэтому guard принимает ТОЛЬКО role, который есть в списке профилей.
# Если профиля нет — issue пропускается с errored++ (НЕ skipped++), потому что
# это означает ошибку конфигурации, а не нормальный skip. Также комментируем в
# issue, чтобы юзер увидел "невалидный assignee" (это Q22, но triage не может
# пометить issue как needs-triage-rewrite без явного сигнала — только информирует).
VALID_PROFILES=""
load_valid_profiles() {
    if [ -n "$VALID_PROFILES" ]; then return 0; fi
    if [ -z "$HERMES_BIN" ] || [ ! -x "$HERMES_BIN" ]; then
        log "load_valid_profiles: HERMES_BIN not set/executable — guard disabled (fail-open)"
        VALID_PROFILES="__disabled__"
        return 0
    fi
    # `hermes profile list` returns human-readable table with profile names in
    # first column. Parse with awk, skipping header/footer/blank lines.
    _out="$("$HERMES_BIN" profile list 2>/dev/null || true)"
    if [ -z "$_out" ]; then
        log "load_valid_profiles: hermes profile list returned empty — guard disabled (fail-open)"
        VALID_PROFILES="__disabled__"
        return 0
    fi
    VALID_PROFILES="$(printf '%s' "$_out" | awk '
        /^[ \t]*─/ {next}             # table separator lines
        /^[ \t]*Profile[ \t]/ {next}  # header row (may be indented)
        /^[ \t]*$/ {next}             # blank lines
        /^[ \t]*default[ \t]/ {next}  # skip `default` profile (placeholder)
        {gsub(/^[ \t]+|[ \t]+$/, ""); print $1}
    ' | sort -u | paste -sd'|' -)"
    log "loaded valid profiles ($(printf '%s' "$VALID_PROFILES" | tr '|' ',' | head -c 200))..."
    return 0
}

is_valid_profile() {  # $1=role
    local role="$1"
    [ -z "$role" ] && return 1
    # Fail-open: если guard disabled (hermes CLI недоступен) — пропускаем все,
    # чтобы не сломать работающий процесс из-за временного сбоя CLI.
    [ "$VALID_PROFILES" = "__disabled__" ] && return 0
    # Проверяем через case (быстрее grep) — список из pipe-delimited.
    case "|$VALID_PROFILES|" in
        *"|$role|"*) return 0 ;;
        *) return 1 ;;
    esac
}

# Ретро-фикс (09.08 #2): крупные задачи получают увеличенный --max-runtime.
# Крупная = label `priority:P0` ИЛИ объёмный body (>= AGENT_FLOW_LARGE_BODY_CHARS).
runtime_for() {  # $1=labels_csv  $2=body
    local labels_lower body_len
    labels_lower="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    body_len="${#2}"
    if printf '%s' "$labels_lower" | grep -Eq '(^|,)priority:p0(,|$)'; then
        printf '%s' "$AGENT_FLOW_MAX_RUNTIME_LARGE"; return
    fi
    if [ "$body_len" -ge "$AGENT_FLOW_LARGE_BODY_CHARS" ]; then
        printf '%s' "$AGENT_FLOW_MAX_RUNTIME_LARGE"; return
    fi
    printf '%s' "$AGENT_FLOW_MAX_RUNTIME"
}

# Ретро-фикс (09.08 #2): контракт воркера в каждой карточке — воркер коммитит
# WIP каждые ~15-20 мин (или при половине max_runtime), чтобы работа не
# пропадала при исчерпании бюджета (см. t_9435a3c5, t_0c0a98ac).
worker_contract_block() {  # $1=max_runtime
    cat <<EOF
## Контракт воркера (agent-flow)
- **Коммить WIP каждые ~15-20 мин** (или при достижении половины max_runtime=$1) — \`wip(scope): ...\` коммиты в свою ветку, push сразу. Незакоммиченная работа пропадает при исчерпании бюджета.
- PR один на всю задачу (WIP-коммиты идут в ту же ветку; merge-gate/e2e работают по ветке/PR, не по числу коммитов).
- **Одна карточка = одна сессия.** Не продолжай «отравленную» сессию на новой итерации этой карточки — начни новую, handoff через коммент в issue.
- **Читай точечно.** Начинай с sources_of_truth из блока Context карточки; разведку («где обрабатывается X») отдавай сабагенту, в сессию — только вывод «файл:строка».
- **В issue — релевантные куски, не полные логи.** Raw-вывод обязателен (ADR-0018), но прикладывай нужный фрагмент, не весь дамп.

EOF
}

# ============================================================================
# G8: fingerprint dedup helpers (ретро t_b0fe4398, 26.08, issues
# #1650/#1653/#1655/#1658). 4 devops-воркера за 5ч открыли 4 PR на одну и ту же
# проблему (opt-in quest svc через compose profile) на РАЗНЫХ ветках.
# OPEN-PR guard (per-branch) и throttle (per-issue) не ловили — каждый issue
# имел свой PR, у каждого PR своя ветка. Fix: перед `hermes kanban create`
# проверить, нет ли в OPEN PR-ах уже ТАКОГО ЖЕ фикса (по fingerprint от
# added-lines в whitelist-файлах).
# ============================================================================

# file_in_fp_whitelist <path>: проверяет, входит ли path в whitelist из
# $FINGERPRINT_FILE_GLOBS (список glob'ов через `|`). Использует bash case с
# extglob, чтобы поддерживать `*` без подоболочки. Возвращает exit 0 если
# файл подходит, 1 — если нет. Нечувствителен к ведущему `./`.
#
# CRITICAL: disable globbing перед `local patterns=($FINGERPRINT_FILE_GLOBS)`,
# иначе bash РАЗВЁРНЕТ `docker/*/docker-compose.yaml` по реальным файлам
# CWD и паттерн потеряется. set -f блокирует glob'инг, set +f восстанавливает.
file_in_fp_whitelist() {
    local path="${1#./}"
    case "$path" in
        /*) path="${path#/}" ;;
    esac
    local IFS='|'
    set -f
    # shellcheck disable=SC2206
    local patterns=($FINGERPRINT_FILE_GLOBS)
    set +f
    local pat
    # SC2254 unquoted $pat is INTENTIONAL — нам нужна именно glob-магия
    # (`*` в паттерне должна матчиться как wildcard, а не literal).
    # shellcheck disable=SC2254
    for pat in "${patterns[@]}"; do
        case "$path" in
            $pat) return 0 ;;
        esac
    done
    return 1
}

# fp_for_pr_file <pr_number> <file_path>:
# Возвращает fingerprint (16 hex-символов sha256) от canonical-added-lines
# указанного файла в PR. Учитываются ТОЛЬКО "semantic" added lines — не
# комментарии (строки, начинающиеся с `#` ПОСЛЕ trim), не пустые. Это
# критично для ретро t_b0fe4398: 4 devops-PR (#1651/#1654/#1656/#1659)
# добавили ОДНУ И ТУ ЖЕ строку `profiles: ["quest"]`, но РАЗНЫЕ NOTE-блоки
# с обоснованиями (t_4d530162 vs #1653 vs t_6ccdfa10 vs deploy-fail
# test-round-{232,233,234,235}). Без фильтра комментариев fingerprint'ы
# разные → G8 не сматчит → багат не починен.
#
# Нормализация: trim пробелов, удаление пустых и `#`-prefixed строк, sort -u,
# обрезка до 1024 байт (защита от огромных blob'ов). Stable для одной и той
# же сути фикса, не зависит от порядка строк в файле.
#
# NOTE: `gh pr diff` (в отличие от `git diff`) НЕ принимает path-positional
# фильтр — выдаёт ПОЛНЫЙ diff. Поэтому мы берём весь diff и post-filter'уем
# по `diff --git a/<path> b/<path>` (header) → собираем только hunk'и для
# нужного файла. Делает это awk, чтобы не тащить regex-библиотеки.
fp_for_pr_file() {  # $1=pr $2=file
    local pr="$1" file="$2"
    gh pr diff "$pr" 2>/dev/null \
        | awk -v target="$file" '
            # diff --git a/<path> b/<path>
            /^diff --git / {
                p = $0
                sub(/^diff --git a\//, "", p)
                sub(/ b\/.*$/, "", p)
                current = p
                in_hunk = 0
                next
            }
            current != target { next }
            /^@@/ { in_hunk = 1; next }
            in_hunk && /^---/ { next }      # skip --- separator
            in_hunk && /^\+\+\+/ { next }   # skip +++ header
            in_hunk && /^\+/ {
                sub(/^\+/, "")
                gsub(/^[[:space:]]+|[[:space:]]+$/, "")
                # Skip blank lines and comment lines (после trim начинаются с "#").
                # YAML/docker compose комментарии = "# ..." (с пробелом или без).
                # Это даёт fingerprint ТОЛЬКО по функциональным изменениям —
                # критично для дедупа 4 PR с одним фиксом но разными NOTE-блоками.
                if (length($0) == 0) next
                if (substr($0, 1, 1) == "#") next
                print
            }
        ' \
        | sort -u \
        | head -c 1024 \
        | (if command -v sha256sum >/dev/null 2>&1; then sha256sum; else shasum -a 256; fi) \
        | awk '{print substr($1, 1, 16)}'
}

# find_duplicate_fix_prs <issue_number> <issue_body>:
# Возвращает строки "<pr_number>\t<file>\t<fingerprint>" для каждого OPEN PR,
# который модифицирует whitelist-файл с ТАКИМ ЖЕ fingerprint, что и хотя бы
# один whitelist-файл, упомянутый в issue_body. Если тело issue не
# содержит явных путей к whitelist-файлам — пустой результат (gate
# пропускается, поведение = ровно то же, что было до фикса — backward compat).
#
# Алгоритм:
#   1. Извлечь whitelist-файлы из issue_body (grep по glob'ам).
#   2. Для каждого такого файла получить список OPEN PR, которые его
#      модифицировали (через gh pr list --json files и фильтр на клиенте).
#   3. Для каждого PR вычислить fp_for_pr_file.
#   4. Emit "<pr>\t<file>\t<fp>" — кол-во unique fingerprint'ов сверяет caller.
find_duplicate_fix_prs() {  # $1=issue_number $2=body
    local body="$2"
    local IFS='|'
    set -f
    # shellcheck disable=SC2206
    local patterns=($FINGERPRINT_FILE_GLOBS)
    set +f
    local candidate_files="" pat rx extracted
    for pat in "${patterns[@]}"; do
        # Конвертируем glob-pattern в regex для grep -E:
        #   . -> [.]   (literal dot, не "любой символ")
        #   * -> [a-zA-Z0-9._/-]*   (basename/dirname chars в нашем whitelist)
        rx="$(printf '%s' "$pat" | sed 's/\./[.]/g; s/\*/[a-zA-Z0-9._\/-]*/g')"
        # ищем как самостоятельное слово (не внутри более длинного пути).
        # `(^|[^A-Za-z0-9_/-])` + `([^A-Za-z0-9_/-]|$)` — word-boundary через
        # "не файловые символы".
        extracted="$(printf '%s' "$body" | grep -oE "(^|[^A-Za-z0-9_/-])(${rx})([^A-Za-z0-9_/-]|$)" 2>/dev/null \
            | sed -E 's@^[^A-Za-z0-9_/-]+@@; s@[^A-Za-z0-9_/-]+$@@' \
            | sort -u)"
        if [ -n "$extracted" ]; then
            candidate_files="${candidate_files}${candidate_files:+$'\n'}${extracted}"
        fi
    done
    candidate_files="$(printf '%s\n' "$candidate_files" | awk 'NF && !seen[$0]++')"
    if [ -z "$candidate_files" ]; then
        return 0  # нет whitelist-файлов в body → gate пропускается
    fi

    # Собрать все OPEN PR в JSON за один запрос (до 100, обычно их 5-30).
    local prs_json
    prs_json="$(gh pr list --repo "$GH_REPO" --state open --limit 100 \
        --json number,files 2>/dev/null || echo '[]')"

    # Для каждого кандидат-файла пройтись по PR-ам и сравнить fingerprint.
    local file pr_numbers pr_num
    while IFS= read -r file; do
        [ -z "$file" ] && continue
        # Найти PR, которые модифицировали этот файл (python парсит JSON).
        # python выводит по одному PR-number на строку.
        pr_numbers="$(printf '%s' "$prs_json" | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    d = []
target = sys.argv[1]
for pr in d:
    files = pr.get('files') or []
    for f in files:
        if f.get('path') == target:
            print(pr.get('number', ''))
            break
" "$file" 2>/dev/null || true)"
        # Итерируем построчно (IFS= на время чтения), а не через `for pr_num in
        # $pr_numbers` — иначе bash склеит многострочный вывод в одну переменную.
        while IFS= read -r pr_num; do
            [ -z "$pr_num" ] && continue
            fp="$(fp_for_pr_file "$pr_num" "$file" 2>/dev/null || true)"
            [ -z "$fp" ] && continue
            printf '%s\t%s\t%s\n' "$pr_num" "$file" "$fp"
        done <<< "$pr_numbers"
    done <<< "$candidate_files"
}

# --- process each issue ------------------------------------------------------
# Ретро-фикс (09.08 #14): дубликаты карточек от triage при переключении меток.
# Маркер `kanban: t_` в комментариях мог потеряться (коммент не записался /
# удалён) → triage создавал вторую карточку на тот же issue. Дополнительная
# идемпотентность: собрать мапу issue -> task_id из СУЩЕСТВУЮЩИХ карточек
# (по "issue: #N" в body) и не создавать дубль, если карточка уже есть.
#
# Ретро-фикс (18.08 t_a0fac345): регекс r"issue:\s*#(\d+)" ловил ТОЛЬКО формат
# с двоеточием (`Source issue: #N`). Manual-карточки Шифу имеют формат
# `**Source**: issue #N` (без двоеточия перед номером, только после Source) →
# triage не видел ручную карточку → создавал дубль → 2 worker-а параллельно
# работали над одним issue (race). Расширили регекс:
#   - ловит и `issue: #N`, и `issue #N`, и `Issue #N` (case-insensitive на слово issue)
#   - в карту попадают ВСЕ статусы (включая done/archived) — но downstream
#     фильтрует по статусу (skip только если ACTIVE — running/ready/todo/blocked)
# shellcheck disable=SC2016  # python heredoc — $ внутри одинарных кавычек literal
existing_by_issue="$(printf '%s' "$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json --archived 2>/dev/null || echo '[]')" | python3 -c '
import json, sys, re
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
for t in tasks:
    body = t.get("body") or ""
    # Ловим и `Source**: issue #N`, и `Source: issue: #N`, и любой регистр.
    # Слово "issue" опционально с двоеточием после — \W* съедает 0+ не-word.
    m = re.search(r"\bissue\W*#(\d+)", body, re.IGNORECASE)
    if m:
        print("%s\t%s\t%s" % (m.group(1), t.get("id", ""), t.get("status", "")))
')"

# Ретро-фикс (09.08 #1): старые done/archived карточки держат ветку через
# worktree → при спавне новой карточки `git worktree add` падает
# («already checked out») и карточка навсегда виснет в blocked. Собираем
# кандидатов-клонов (из workspace_path всех карточек + REPO_DIR) — главный
# клон, где диспетчер создаёт .worktrees/<task-id>, может отличаться от
# REPO_DIR (cron-клон) и от самой старой карточки.
WORKTREE_CLONES="$(printf '%s' "$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null || echo '[]')" | python3 -c '
import json, sys, os
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
seen = set()
for t in tasks:
    wp = t.get("workspace_path") or ""
    if "/.worktrees/" in wp:
        repo = os.path.dirname(os.path.dirname(wp))
        if repo not in seen:
            seen.add(repo)
            print(repo)
')"
if [ -n "${REPO_DIR:-}" ]; then
    WORKTREE_CLONES="$(printf '%s\n%s\n' "$WORKTREE_CLONES" "$REPO_DIR" | awk 'NF && !seen[$0]++')"
fi

# Освободить worktree'ы на ветке $1, принадлежащие done/archived карточкам.
free_stale_worktrees_for_branch() {  # $1=branch
    local branch="$1" clone line wt_path wt_branch owner status
    [ -z "$WORKTREE_CLONES" ] && { log "  no worktree-owner card found — cannot locate main clone; skip"; return 0; }
    while IFS= read -r clone; do
        [ -n "$clone" ] || continue
        [ -d "$clone/.git" ] || continue
        wt_path=""
        while IFS= read -r line; do
            case "$line" in
                worktree\ *) wt_path="${line#worktree }" ;;
                branch\ *)
                    wt_branch="${line#branch refs/heads/}"
                    if [ "$wt_branch" = "$branch" ] && [ -n "$wt_path" ]; then
                        owner="$(basename "$wt_path")"
                        case "$owner" in
                            t_[a-f0-9]*)
                                # Ретро 12.08 t_8af6bf29: `hermes kanban show` падает
                                # после v0.20.0 (sqlite3.ProgrammingError). Читаем статус
                                # из kanban.db, fallback на JSON-режим show (не падает).
                                _db="${KANBAN_DB_PATH:-${HERMES_HOME}/kanban/boards/${KANBAN_BOARD}/kanban.db}"
                                status="$(python3 - "$_db" "$owner" <<'PY' 2>/dev/null || true
import sqlite3, sys
db, tid = sys.argv[1], sys.argv[2]
try:
    conn = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    row = conn.execute("SELECT status FROM tasks WHERE id=?", (tid,)).fetchone()
    conn.close()
    print(row[0] if row else "")
except Exception:
    pass
PY
)"
                                if [ -z "$status" ] && [ ! -f "$_db" ]; then
                                    status="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$owner" --json 2>/dev/null \
                                        | python3 -c 'import sys,json
try: print(json.load(sys.stdin).get("task",{}).get("status",""))
except Exception: print("")' 2>/dev/null || true)"
                                fi
                                if [ "$status" = "done" ] || [ "$status" = "archived" ]; then
                                    git -C "$clone" worktree remove --force "$wt_path" 2>/dev/null \
                                        && log "  freed stale worktree $wt_path (branch $branch, card $owner, status=$status)"
                                fi
                                ;;
                        esac
                    fi
                    ;;
            esac
        done < <(git -C "$clone" worktree list --porcelain 2>/dev/null)
        git -C "$clone" worktree prune 2>/dev/null || true
    done <<< "$WORKTREE_CLONES"
    return 0
}

# process_issues_json — единый pipeline обработки issues (ретро-фикс t_360dc1a4
# для Phase 2 GSD-orphans: чтобы не дублировать ~430 строк кода и чтобы оба
# этапа имели ИДЕНТИЧНЫЕ guards (idempotency, throttle, big-bang, runtime,
# role validation, MERGED-PR skip, etc.) — выносим основной цикл в функцию.
#
# Аргументы:
#   $1 — phase_label ("phase1" или "phase2"), используется только в логах
#   $2 — issues_json (newline-separated records: "number\ttitle\tlabels\tbody")
#
# Зависит от outer-scope (снапшот берётся на момент вызова):
#   existing_by_issue, WORKTREE_CLONES, REPO_DIR, KANBAN_BOARD,
#   MAINTENANCE_BRANCH, DONE_LABEL, BIG_BANG_OVERRIDE_LABEL, BIG_BANG_MAX_COMMITS,
#   BIG_BANG_MAX_LINES, VALID_PROFILES, AGENT_FLOW_DEFAULT_ROLE, AGENT_FLOW_MAX_RUNTIME,
#   AGENT_FLOW_MAX_RETRIES, AGENT_FLOW_LARGE_BODY_CHARS, AGENT_FLOW_MAX_RUNTIME_LARGE,
#   GH_REPO, HERMES_BIN, DRY_RUN, LOG_PREFIX, role_for, branch_for, branch_label_override,
#   is_valid_profile, load_valid_profiles, free_stale_worktrees_for_branch, runtime_for,
#   worker_contract_block, gh (auth).
#
# Обновляет outer-scope counters (created, skipped, errored) — bash scoping
# без `local` позволяет писать в родительские переменные.
process_issues_json() {
    local phase_label="$1" issues_stream="$2"
    while IFS=$'\t' read -r number title labels body; do
        [ -z "$number" ] && continue

    # Ретро-фикс (13.08, #968): переоткрытые issue (closed → reopened) —
    # это доработка, карточку создавать ЗАНОВО. Для них оба idempotency-гарда
    # (мёртвый kanban-маркер от archived-карточки, merged PR guard) — ложные:
    # работа была сделана, но юзер вернул задачу на доработку.
    is_reopened=false
    if _reopen_ts="$(gh api "repos/${GH_REPO}/issues/${number}/events" \
        --jq '[.[] | select(.event=="reopened")] | last | .created_at' 2>/dev/null || true)" \
        && [ -n "$_reopen_ts" ]; then
        # Ретро-фикс (22.08, #1506 reopened-loop): is_reopened должен быть
        # ОДНОРАЗОВЫМ. Раньше он оставался true на КАЖДОМ тике, пока в timeline
        # есть reopen-событие → гард по маркеру `kanban: t_` (ниже) пропускался,
        # а если свежая карточка умирала быстро (блокер воркера), следующий тик
        # создавал новую → бесконечный цикл (9 карточек за 16 мин, #1506).
        # Теперь: если после последнего reopen уже есть маркер `kanban: t_`
        # (карточка доработки создана) — reopen считается «потреблённым»,
        # is_reopened=false, и дальше работает обычная идемпотентность по маркеру.
        _last_marker_ts="$(gh api "repos/${GH_REPO}/issues/${number}/comments" --paginate \
            --jq '([.[] | select((.body // "") | test("^kanban: t_[a-f0-9]+"))] | last | .created_at) // empty' 2>/dev/null || true)"
        if [ -z "$_last_marker_ts" ] || [[ "$_last_marker_ts" < "$_reopen_ts" ]]; then
            is_reopened=true
            log "issue #${number} was REOPENED at ${_reopen_ts} (маркер: ${_last_marker_ts:-нет}) — доработка, создаю свежую карточку"
        else
            log "issue #${number}: reopen ${_reopen_ts} уже потреблён маркером ${_last_marker_ts} — обычная идемпотентность"
        fi
    fi

    # Ретро-фикс (11.08 t_ce3ca0d9): НЕ создаём карточку для issue, где работа
    # уже завершена — метка $DONE_LABEL (e2e-done) ставится merge-gate после
    # мержа PR + успешного e2e. Раньше триаж плодил дубликаты для таких issue
    # (пример #1104 → t_8ebc85d9), т.к. issue остаётся OPEN после мержа.
    if printf '%s' "$labels" | tr ',' '\n' | tr '[:upper:]' '[:lower:]' | grep -Fxq "$DONE_LABEL"; then
        log "issue #${number} has ${DONE_LABEL} label (work done) — skip"
        skipped=$((skipped+1)); continue
    fi

    # Idempotency: check if a `kanban: t_` marker already exists in comments.
    # Ретро-фикс (11.08 t_ce3ca0d9): раньше использовался `gh issue view
    # --comments` (GraphQL), который может не вернуть старые комментарии при
    # пагинации. Теперь — REST API с --paginate, гарантированно все комментарии.
    if [ "$is_reopened" = "false" ] && \
        gh api "repos/${GH_REPO}/issues/${number}/comments" --paginate \
        --jq '.[].body' 2>/dev/null \
        | grep -Eq '^kanban: t_[a-f0-9]+'; then
        log "issue #${number} already has kanban marker — skip"
        skipped=$((skipped+1)); continue
    fi

    # Idempotency v2 (ретро 09.08 #14): карточка для этого issue уже есть.
    # Ретро-фикс (13.08, #968): для REOPENED issue проверяем статус существующей
    # карточки: если она ЖИВАЯ (running/ready/todo/blocked) — воркер уже работает,
    # дубль НЕ создаём; если мертва (done/archived) — создаём свежую.
    # Ретро-фикс (18.08 t_a0fac345): `existing_by_issue` теперь содержит статус
    # в 3-м поле (id\tstatus) — используем его напрямую, без лишнего
    # `kanban show` (экономит ~1-2 сек на тик + не зависит от возможных падений show).
    existing_line="$(printf '%s\n' "$existing_by_issue" | awk -F'\t' -v n="$number" '$1==n {print; exit}')"
    existing_id="$(printf '%s' "$existing_line" | cut -f2)"
    existing_status="$(printf '%s' "$existing_line" | cut -f3)"
    if [ -n "$existing_id" ]; then
        if [ "$is_reopened" = "false" ]; then
            # НЕ reopened → любая существующая карточка (active или dead) —
            # означает что для этого issue уже была работа. Для non-reopened
            # случая мы не пересоздаём карточку даже если старая done/archived:
            # если юзер хочет доработку, он сам переоткроет issue (триггернёт
            # is_reopened=true ветку ниже). Это закрывает ретро-bug t_a0fac345
            # (race manual+auto-triage на свежем issue).
            log "issue #${number} already has card ${existing_id} (status=${existing_status:-unknown}) — skip"
            skipped=$((skipped+1)); continue
        fi
        # REOPENED → проверяем статус: живая → skip, мёртвая → создаём свежую.
        case "${existing_status:-}" in
            running|ready|todo|blocked)
                log "issue #${number}: карточка ${existing_id} ЖИВАЯ (status=${existing_status}) — воркер уже работает, дубль не создаём"
                skipped=$((skipped+1)); continue
                ;;
            done|archived|"")
                log "issue #${number}: старая карточка ${existing_id} мертва (status=${existing_status:-unknown}) — создаю свежую на доработку"
                ;;
        esac
    fi

    role="$(role_for "$labels")"
    branch="$(branch_for "$labels" "$number" "$title")"
    max_runtime="$(runtime_for "$labels" "$body")"

    # Ретро-фикс (26.08 t_dfd3d19d, ADR-0032): G9b race-window dedup.
    # Если вычисленная ветка уже есть в remote refs — карточка создаст
    # worker'а, который не сможет открыть эту ветку (`git worktree add` →
    # «already checked out»), карточка навсегда зависнет в blocked.
    # Сценарий: предыдущий tick запушил branch, или параллельный worker
    # уже ведёт ту же работу. `gh pr list` тут не поможет — нужен
    # ls-remote refs check.
    # branch_exists_in_remote fail-OPEN при сетевом сбое (подробности в ADR-0032
    # §4.2). Возвращает 0 = branch существует → skip.
    if branch_exists_in_remote "$branch"; then
        log "issue #${number}: branch ${branch} already in remote refs (G9b race-window, ретро t_dfd3d19d) — карточку НЕ создаём"
        if [ "$DRY_RUN" != "true" ]; then
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "agent-flow:dedup-raceskip (ретро t_dfd3d19d, ADR-0032)

Triage: вычисленная ветка \`${branch}\` уже существует в remote refs. Скорее всего, её создал предыдущий tick или параллельный worker, и новая карточка не сможет открыть эту ветку (\`worktree add\` → «already checked out»). Карточка для этого issue будет создана после очистки ветки.

Если ветка должна быть переписана (старая неактуальна):
1. **товарищ Шифу**: удалите ветку на remote \`git push origin :${branch}\` (или через gh CLI), либо присвойте этому issue explicit-ветку через label \`branch:<имя>\` — тогда triage использует её вместо вычисленной.
2. После удаления повторный тик triage создаст карточку.

См. ADR-0032 §4.2 для деталей race-scenarios." >/dev/null 2>&1 || true
            gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow-error" >/dev/null 2>&1 || true
        fi
        dedup_race_skipped=$((dedup_race_skipped+1))
        skipped=$((skipped+1)); continue
    fi

    # Ретро-фикс (22.08 t_8cde8449, issue #1506 reopened-loop): если на issue
    # есть явная метка `branch:NAME` (Шифу указал целевую ветку руками — например,
    # потому что slugify на кириллице неустойчив или ветка должна совпадать с
    # уже открытым PR), и на этой ветке уже есть OPEN PR — карточку НЕ создаём.
    # Это закрывает класс багов «triage плодит 4 карточки на одну ветку за 1ч»:
    # PR #1517 на ветке `z-{agent}/1506-task-...` уже в работе (ждёт merge-gate/
    # e2e), а triage продолжал спавнить kanban-карточки t_2e148de9 → t_21d3bded
    # → t_50018d92 → t_37134371 на ту же branch_name. Каждая умирала на
    # worktree-collision, новая порождалась заново.
    #
    # Backward-compat (acceptance #2): если метки `branch:` нет — guard
    # полностью пропускается, поведение = ровно то же, что было до фикса.
    # Существующий OPEN-PR guard (584-589) для вычисленного $branch остаётся
    # — он ловит случай, когда PR-head совпадает с branch_for().
    _branch_explicit="$(branch_label_override "$labels")"
    if [ -n "$_branch_explicit" ]; then
        # Явная ветка от Шифу. Используем её и для guard'а, и для финального
        # `--branch` в kanban create (Шифу сказал — так и делаем).
        branch="${_branch_explicit}"
        if open_pr_explicit="$(gh pr list --repo "$GH_REPO" --head "$branch" \
            --state open --json number --jq '.[0].number' 2>/dev/null || true)" \
            && [ -n "$open_pr_explicit" ]; then
            log "issue #${number}: label branch:${branch} → OPEN PR #${open_pr_explicit} уже ведёт эту ветку — карточку не создаём (pre-create guard, ретро t_8cde8449)"
            skipped=$((skipped+1)); continue
        fi
        log "issue #${number}: label branch:${branch} → используем как ground truth (явная метка override'ит slugify)"
    fi

    # Ретро-фикс (19.08, t_dd7a5749): assignee-existence guard — ВАЛИДАЦИЯ role
    # против реального списка профилей. Без этой проверки карточка создаётся с
    # несуществующим assignee (например, `triager` в t_1ca827a6) и навсегда
    # висит в ready — dispatcher не имеет worker-pool для такого assignee.
    #
    # Поведение:
    #   - role валиден → continue (нормальный путь, карточка создастся)
    #   - role НЕ валиден → errored++, комментарий в issue, НЕ создаём карточку
    #   - guard disabled (fail-open) → continue (CLI был недоступен, не ломаем процесс)
    #
    # Это ДО branch/merge-pr guards: если role невалиден, дальнейшие проверки
    # (merged_pr на этой ветке, recent_cards) — бессмысленны, мы всё равно не
    # создадим карточку. Load profiles lazily — один раз за тик.
    load_valid_profiles
    if ! is_valid_profile "$role"; then
        log "🚨 issue #${number}: assignee '${role}' НЕВАЛИДЕН (нет в profile list) — пропускаем (errored)"
        if [ "$DRY_RUN" != "true" ]; then
            _valid_csv="$(printf '%s' "$VALID_PROFILES" | tr '|' ',' | sed 's/^,//;s/,$//')"
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "🚨 **agent-flow-triage: invalid assignee**

Triage **НЕ создал** kanban-карточку для этого issue, потому что assignee=\`${role}\` (из label \`agent:${role}\` или \`AGENT_FLOW_DEFAULT_ROLE\`) **не существует** в списке профилей hermes:

\`\`\`
${_valid_csv}
\`\`\`

**Что делать (товарищ Шифу):**
1. Поставить правильную метку \`agent:<valid-role>\` на этот issue (например, \`agent:devops\`)
2. Либо создать новый профиль \`${role}\` через \`hermes profile create ${role}\` (если роль действительно нужна)
3. Либо удалить эту метку — тогда triage возьмёт \`AGENT_FLOW_DEFAULT_ROLE\` (по дефолту \`architect\`)

Ретро-карточка: t_dd7a5749." >/dev/null 2>&1 || true
            # issue #1534: self-id whoami BEFORE adding agent-flow-error label.
            whoami_add_label "$number" "agent-flow-error" "invalid assignee=${role} (label agent:${role} or default), valid profiles: ${_valid_csv} (retro t_dd7a5749)"
            gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow-error" >/dev/null 2>&1 || true
        fi
        errored=$((errored+1)); continue
    fi

    # Ретро-фикс (11.08 t_ce3ca0d9): если на ветке, которую мы бы создали для
    # этого issue, уже есть MERGED PR — работа ушла в develop, карточка не нужна.
    # Это второй рубеж после DONE_LABEL (страховка, если e2e-done не успели
    # проставить, а PR уже смержен).
    # Ретро-фикс (13.08, #968): для REOPENED issue этот гард ложный — юзер
    # вернул задачу на доработку, карточку создаём заново.
    if [ "$is_reopened" = "false" ] && \
        merged_pr="$(gh pr list --repo "$GH_REPO" --head "$branch" --state merged \
        --json number --jq '.[0].number' 2>/dev/null || true)" \
        && [ -n "$merged_pr" ]; then
        log "issue #${number}: branch ${branch} already has MERGED PR #${merged_pr} — skip"
        skipped=$((skipped+1)); continue
    fi

    # Ретро-фикс (13.08, #968): REOPENED issue, но на ветке уже есть OPEN PR —
    # работа в PR (ждёт merge-gate/юзера), карточку НЕ создаём. Без этого гарда
    # триаж плодил бесконечный цикл карточек на один issue: прошлая карточка
    # done → «старая мертва → создаю свежую» → воркер делает ту же работу →
    # done → ... (5+ карточек architect за 20 мин, t_6a4d501b → t_e25720e3).
    if open_pr="$(gh pr list --repo "$GH_REPO" --head "$branch" --state open \
        --json number --jq '.[0].number' 2>/dev/null || true)" \
        && [ -n "$open_pr" ]; then
        log "issue #${number}: branch ${branch} already has OPEN PR #${open_pr} — работа в PR, карточку не создаём (reopened-loop guard)"
        skipped=$((skipped+1)); continue
    fi

    # --- G8: fingerprint dedup gate (ретро 26.08 t_b0fe4398, #1650/#1653/#1655/#1658) ---
    # Per-branch OPEN-PR guard выше ловит ТОЛЬКО тот случай, когда issue
    # уже имеет свой PR на ЭТОЙ ветке. Но что если 4 разных issue (разные
    # ветки) — все про одну и ту же проблему (#1650/#1653/#1655/#1658: opt-in
    # quest svc через compose profile)? Per-branch guard не видит, throttle
    # не видит (throttle per-issue). Здесь проверяем: если в OPEN PR-ах
    # репы уже есть ≥FINGERPRINT_DUPLICATE_THRESHOLD фиксов с тем же
    # fingerprint (added-lines в whitelist-файлах docker-compose / package.xml
    # / setup.py / Dockerfile / install/setup), что мог бы сделать и этот
    # issue — это дубль.
    #
    # Backward-compat: если в body issue нет явного пути к whitelist-файлу,
    # `find_duplicate_fix_prs` возвращает пусто → gate полностью пропускается,
    # поведение = ровно то же, что было до фикса. Это защищает от ложных
    # срабатываний на issue, где речь про Python/voice/ROS код (не наш кейс).
    _fp_dups="$(find_duplicate_fix_prs "$number" "$body" 2>/dev/null || true)"
    if [ -n "$_fp_dups" ]; then
        _fp_dup_count="$(printf '%s\n' "$_fp_dups" | awk -F'\t' '{print $3}' | sort -u | wc -l | tr -d ' ')"
        if [ "${_fp_dup_count:-0}" -ge "${FINGERPRINT_DUPLICATE_THRESHOLD:-1}" ] 2>/dev/null; then
            _fp_first_pr="$(printf '%s\n' "$_fp_dups" | head -n1 | cut -f1)"
            _fp_first_file="$(printf '%s\n' "$_fp_dups" | head -n1 | cut -f2)"
            _fp_first_fp="$(printf '%s\n' "$_fp_dups" | head -n1 | cut -f3)"
            log "🚨 issue #${number}: G8 fingerprint dedup — найдено ${_fp_dup_count} OPEN PR с тем же фиксом (file=${_fp_first_file}, fp=${_fp_first_fp}, первый PR=#${_fp_first_pr}) — карточку НЕ создаём (ретро t_b0fe4398)"
            if [ "$DRY_RUN" != "true" ]; then
                # Comment + label. Comment показывает список ВСЕХ найденных
                # дубликатов, чтобы Шифу мог сразу выбрать canonical-PR.
                _fp_dup_list="$(printf '%s\n' "$_fp_dups" | awk -F'\t' '{print "- PR #" $1 " (file=" $2 ", fp=" $3 ")"}' | sort -u | head -20)"
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "🚨 **agent-flow-triage: G8 fingerprint dedup (ретро t_b0fe4398)**

Triage **НЕ создал** kanban-карточку для этого issue — обнаружен duplicate-fix: ≥\${FINGERPRINT_DUPLICATE_THRESHOLD:-1} OPEN PR уже делает ТОТ ЖЕ fix в whitelist-файле (fingerprint=\`${_fp_first_fp}\`, file=\`${_fp_first_file}\`).

Найденные дубликаты (PR + файл + fingerprint):
${_fp_dup_list}

**Почему так:** OPEN-PR guard (per-branch) и throttle (per-issue) не ловили
4 разных issue на одну и ту же проблему (разные ветки → guard не видит).
Новый G8-guard (fingerprint of added-lines в docker-compose/package.xml/setup.py)
ловит cross-branch дубликаты.

**Что делать (товарищ Шифу):**
1. Закрыть этот issue как дубликат одного из найденных PR, ИЛИ
2. Смержить один из найденных PR (предпочтительно PR #${_fp_first_pr}) — он
   сделает фикс для всех связанных issue, ИЛИ
3. Если этот issue про ДРУГОЙ fix — переформулировать body так, чтобы был
   упомянут конкретный whitelist-файл, который отличается от уже идущих в
   PR (тогда G8 не сматчит и triage создаст карточку).

**Override:** поставьте метку \`big-bang-override\` на этот issue — gate
пропустит fingerprint-проверку (явное разрешение от Шифу).

После того как Шифу закроет/смержит дубликаты, повторный тик triage создаст
карточку (если body всё ещё указывает на whitelist-файл)." >/dev/null 2>&1 || true
                gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow-error" >/dev/null 2>&1 || true
            fi
            skipped=$((skipped+1)); continue
        fi
    fi

    # Ретро-фикс (14.08 t_0a765152): REOPENED issue — карточку создаём ЗАНОВО,
    # но branch_name НЕ переиспользуем, если на нём уже влит PR. Триаж брал
    # branch_name из прошлой карточки того же issue (#1217: PR #1220 merged на
    # z-{agent}/1217-e2e-40-deepseek, триаж создал t_7cc96c7d с ТОЙ ЖЕ веткой
    # → merge-gate по exact-match ветки ложно заархивировал ЖИВУЮ карточку,
    # чья работа ещё в OPEN PR #1231). Если на кандидате уже есть MERGED PR —
    # добавляем суффикс -v2/-r2 (цикл: -v3, -v4... пока ветка не свободна).
    if [ "$is_reopened" = "true" ]; then
        _branch_base="$branch"
        _branch_v=2
        while _merged_on_branch="$(gh pr list --repo "$GH_REPO" --head "$branch" \
            --state merged --json number --jq '.[0].number' 2>/dev/null || true)" \
            && [ -n "$_merged_on_branch" ]; do
            branch="${_branch_base}-v${_branch_v}"
            _branch_v=$((_branch_v+1))
        done
        if [ "$branch" != "$_branch_base" ]; then
            log "issue #${number}: REOPENED — ветка ${_branch_base} уже влита через PR #${_merged_on_branch}, беру ${branch} (ретро t_0a765152)"
        fi
    fi

    # Ретро-фикс (13.08, #968 v3): THROTTLE — если по issue в БД уже есть
    # карточка (status=running/ready/todo/blocked — "ЖИВАЯ") за последние 4 часа
    # — не создаём новую. Без этого тик каждые 2 мин плодил карточку (13:00,
    # 13:02, 13:05...): воркер падал на spawn (worktree занят живой веткой),
    # карточка уходила в archived, следующий тик видел «нет живой» и создавал
    # снова. OPEN-PR guard не ловит, т.к. PR #1197 уже CLOSED.
    # Ретро-фикс (13.08, #968 v3.1): list БЕЗ --archived НЕ возвращает archived-
    # карточки, поэтому throttle не видел предыдущие карточки цикла (все они
    # уходили в archived) и тик создавал новую каждые ~2-5 мин (13:13, 13:18 —
    # даже после деплоя v3 в 13:07). Добавляем --archived: throttle видит ВСЕ
    # карточки за 4ч, включая archived.
    # Ретро-фикс (22.08, t_a24ffe39, #1513): throttle v3 учитывал created_at,
    # но НЕ статус — archived карточка считалась "свежей" и блокировала
    # создание. После ручной архивации (Шифу вычистил цикл) триаж не мог
    # создать новую карточку для #1506 ~4ч, пока старая карточка не
    # "состарится" (cutoff). Теперь: archived/done карточка — игнорируется
    # (throttle её пропускает), throttle блокирует только по-настоящему
    # живым (running/ready/todo/blocked).
    # shellcheck disable=SC2016  # python heredoc — $number внутри literal
    _recent_line="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json --archived 2>/dev/null | python3 -c '
import json, sys, re, time
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
now = time.time()
cutoff = now - 4 * 3600
for t in tasks:
    if (t.get("created_at") or 0) < cutoff:
        continue
    body = t.get("body") or ""
    # Тот же регекс, что в existing_by_issue выше (ретро t_a0fac345):
    # \bissue\W*#(\d+) — ловит и "issue: #N", и "issue #N", и "Issue #N".
    if re.search(r"\bissue\W*#%s\b" % "'"$number"'", body, re.IGNORECASE):
        print("%s\t%s" % (t.get("id", ""), t.get("status", "")))
        break
' 2>/dev/null || true)"
    _recent_id="$(printf '%s' "$_recent_line" | cut -f1)"
    _recent_status="$(printf '%s' "$_recent_line" | cut -f2)"
    if [ -n "$_recent_id" ]; then
        case "${_recent_status:-}" in
            done|archived)
                # Мёртвая карточка — throttle НЕ блокирует. Пускаем дальше,
                # и блок idempotency (473-503) сам решит: для REOPENED —
                # создать свежую, для non-reopened — skip (там это уже
                # обработано выше, до throttle).
                log "issue #${number}: найдена мёртвая карточка ${_recent_id} (status=${_recent_status}) за последние 4ч — throttle игнорирует, идём дальше (ретро 22.08 t_a24ffe39)"
                ;;
            *)
                # Живая карточка — throttle блокирует создание, чтобы
                # воркеры не дублировались.
                log "issue #${number}: живая карточка ${_recent_id} (status=${_recent_status}) за последние 4ч — throttle, не создаём (reopened-loop v3)"
                skipped=$((skipped+1)); continue
                ;;
        esac
    fi

    # Ретро-фикс (09.08 #1): освободить stale worktree'ы на этой ветке от
    # done/archived карточек ДО create — иначе диспетчер при спавне упадёт
    # «git worktree add failed» (ветка уже занята) и карточка навсегда
    # зависнет в blocked.
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would free stale worktrees for branch ${branch}"
    else
        free_stale_worktrees_for_branch "$branch" || true
    fi

    # --- big-bang-override check (ADR-0013, ретро t_9726053d) ------------
    # Если к issue уже привязан PR (воркер мог запушить в обход triage —
    # например, нашёл issue в GitHub UI и сделал PR руками), и этот PR
    # огромный (> ${BIG_BANG_MAX_COMMITS} коммитов ИЛИ > ${BIG_BANG_MAX_LINES}
    # строк) — не создаём новую карточку. Причина:
    #   - merge-gate всё равно заблокирует needs-e2e (нет override)
    #   - воркер потратит max_runtime впустую, потом встанет в тупик
    # Ищем PR по вычисленной ветке (`z-{agent}/N-slug` или service-ветка).
    # Если override-метка уже на issue — пропускаем gate (явное разрешение).
    # Если PR нет (нормальный путь) — пропускаем check (нечего проверять).
    if ! printf '%s' "$labels" | tr ',' '\n' | tr '[:upper:]' '[:lower:]' | grep -Fxq "$BIG_BANG_OVERRIDE_LABEL"; then
        _bb_pr_json="$(gh pr list --repo "$GH_REPO" --state all --head "$branch" \
            --json number,additions,commits 2>/dev/null || echo '[]')"
        _bb_pr_count="$(printf '%s' "$_bb_pr_json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(len(d) if isinstance(d, list) else 0)
except Exception:
    print(0)
' 2>/dev/null || echo 0)"
        if [ "${_bb_pr_count:-0}" -gt 0 ] 2>/dev/null; then
            _bb_pr_info="$(printf '%s' "$_bb_pr_json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    if not isinstance(d, list) or not d:
        print("0	0"); sys.exit(0)
    p = d[0]
    ncommits = len(p.get("commits") or [])
    nadd = int(p.get("additions") or 0)
    print(f"{ncommits}	{nadd}")
except Exception:
    print("0	0")
' 2>/dev/null || echo "0	0")"
            _bb_commits="$(printf '%s' "$_bb_pr_info" | cut -f1)"
            _bb_lines="$(printf '%s' "$_bb_pr_info" | cut -f2)"
            _bb_pr_num="$(printf '%s' "$_bb_pr_json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(d[0].get("number", "") if d else "")
except Exception:
    print("")
' 2>/dev/null || true)"
            _bb_reasons=""
            if [ "${_bb_commits:-0}" -gt "${BIG_BANG_MAX_COMMITS}" ] 2>/dev/null; then
                _bb_reasons="${_bb_reasons}${_bb_commits} коммитов > ${BIG_BANG_MAX_COMMITS}; "
            fi
            if [ "${_bb_lines:-0}" -gt "${BIG_BANG_MAX_LINES}" ] 2>/dev/null; then
                _bb_reasons="${_bb_reasons}${_bb_lines} строк > ${BIG_BANG_MAX_LINES}; "
            fi
            if [ -n "$_bb_reasons" ]; then
                log "issue #${number}: PR #${_bb_pr_num} BIG-BANG (${_bb_reasons% ;}) — блокируем triage, требуется split или ${BIG_BANG_OVERRIDE_LABEL}"
                if [ "$DRY_RUN" != "true" ]; then
                    gh issue comment "$number" --repo "$GH_REPO" --body \
                        "🚨 **PR #${_bb_pr_num} BIG-BANG** — нарушение ADR-0013: ${_bb_reasons% ;}.

Triage **НЕ создал** kanban-карточку для этого issue, чтобы воркер не сжёг max_runtime впустую (merge-gate всё равно заблокирует needs-e2e без override).

Что делать:
1. **split** на инкрементальные PR (по 1 эпику), ИЛИ
2. **товарищ Шифу** ставит \`${BIG_BANG_OVERRIDE_LABEL}\` на этот issue (явное override).

После override повторный тик triage создаст карточку.

Ссылки: ADR-0013, CONTRIBUTING.md §69-71." >/dev/null 2>&1 || true
                fi
                skipped=$((skipped+1)); continue
            fi
        fi
    fi

    body_block="${body:-}"
    labels_block="$(printf '%s' "$labels" | tr ',' '\n' | sort -u | paste -sd, -)"
    # Context block: sources of truth + where the repo lives (generic, from env)
    sources_block=""
    if [ -n "${GH_REPO:-}" ]; then
        sources_block="Context
  repo: ${GH_REPO}
  local_clone: ${REPO_DIR:-<not set>}
  base_branch: ${MAINTENANCE_BRANCH:-develop}
  sources_of_truth: README.md, CONTRIBUTING.md, AGENTS.md, SPEC_CURRENT.md (если есть), docs/adr/
  access: env GH_REPO/REPO_DIR, gh auth (уже авторизован). Токены не хардкодить.

"
    fi
    contract_block="$(worker_contract_block "$max_runtime")"
    # NOTE: $(...) strips trailing newlines, so add explicit "\n\n" separator
    # between the contract block and the issue body.
    full_body=$(printf 'Source\n  repo: %s\n  issue: #%s\n  labels: %s\n\n%s%s\n\n%s' \
        "$GH_REPO" "$number" "$labels_block" "$sources_block" "$contract_block" "$body_block")

    log "creating card: issue=#${number} role=${role} branch=${branch} max_runtime=${max_runtime}"

    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would run: ${HERMES_BIN} kanban --board ${KANBAN_BOARD} create --assignee ${role} --workspace worktree --branch ${branch} --max-runtime ${max_runtime} --max-retries ${AGENT_FLOW_MAX_RETRIES} --body <...> -- \"<title>\""
        created=$((created+1)); continue
    fi

    # G4: kanban create. Note: --board is a GLOBAL flag (must precede `create`).
    if ! create_out="$(
        "$HERMES_BIN" kanban --board "$KANBAN_BOARD" create \
            --assignee "$role" \
            --workspace worktree \
            --branch "$branch" \
            --max-runtime "$max_runtime" \
            --max-retries "$AGENT_FLOW_MAX_RETRIES" \
            --body "$full_body" \
            --created-by "agent-flow-triage" \
            -- "$title" 2>&1
    )"; then
        log "kanban create FAILED for issue #${number}: ${create_out}"
        gh issue comment "$number" --repo "$GH_REPO" --body "agent-flow-error: kanban create failed
\`\`\`
${create_out}
\`\`\`" >/dev/null 2>&1 || true
        # issue #1534: self-id whoami BEFORE adding agent-flow-error label.
        whoami_add_label "$number" "agent-flow-error" "kanban create failed: ${create_out:0:120}" "branch=${branch}"
        gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow-error" >/dev/null 2>&1 || true
        errored=$((errored+1)); continue
    fi

    # Extract t_<id> from `Created t_<id>` or JSON envelope.
    task_id="$(printf '%s' "$create_out" | grep -oE 't_[a-f0-9]+' | head -n1)"
    if [ -z "$task_id" ]; then
        log "could not parse task id from create output: ${create_out}"
        errored=$((errored+1)); continue
    fi

    # G5: comment write — 3x retry, exp-backoff (card already exists).
    comment_body="kanban: ${task_id}
branch: ${branch}
role: ${role}"
    attempt=1
    while [ "$attempt" -le 3 ]; do
        if gh issue comment "$number" --repo "$GH_REPO" --body "$comment_body" >/dev/null 2>&1; then
            break
        fi
        sleep=$((2 ** attempt))
        log "comment attempt ${attempt}/3 failed, retry in ${sleep}s"
        sleep "$sleep"
        attempt=$((attempt+1))
    done
    if [ "$attempt" -gt 3 ]; then
        log "comment write failed after 3 retries — card ${task_id} exists but not marked in issue #${number}"
        errored=$((errored+1)); continue
    fi

    log "ok: issue #${number} -> ${task_id} (branch=${branch}, role=${role}) [${phase_label}]"
    created=$((created+1))
    done < <(printf '%s' "$issues_stream" | python3 -c '
import json, sys
data = json.load(sys.stdin)
for issue in data:
    n = issue["number"]
    t = issue["title"]
    l = ",".join(sorted({lab["name"] for lab in issue.get("labels", [])}))
    b = issue.get("body") or ""
    # tab-delimited, escape tabs/newlines in fields
    def esc(s):
        return s.replace("\\", "\\\\").replace("\t", "\\t").replace("\n", "\\n")
    sys.stdout.write(f"{n}\t{esc(t)}\t{esc(l)}\t{esc(b)}\n")
')
}

created=0
skipped=0
errored=0
# Ретро-фикс (26.08 t_dfd3d19d, ADR-0032): отдельные счётчики для G9a/G9b —
# чтобы в summary видеть, сколько issues поймал каждый guard (а не растворялось
# в общем «skipped»). summary печатает «dedup-skipped: N (intra-tick), M (race)».
dedup_intra_skipped=0
dedup_race_skipped=0

# --- Phase 1: primary filter (label=$ISSUE_LABEL, defaults to "hermes") ------
# Ретро t_360dc1a4: до этого фикса triage фильтровал ТОЛЬКО по label 'hermes'.
# Issue #1643 (Phase 1.7 e2e smoke) был создан БЕЗ метки 'hermes' (только
# source:gsd + needs-e2e + area:rob_box_quest + phase:1.7) → провалился через
# фильтр → 10ч+ orphan без kanban-карточки и PR.
#
# Основной цикл: Phase 1 обрабатывает hermes-flagged issues. Phase 2 (ниже)
# ловит GSD-orphans: issues с source:gsd БЕЗ метки hermes (специфика GSD-
# workflow: он выставляет source:gsd, но не всегда hermes — только для issues,
# прошедших первичный triage-фильтр бота).
#
# G3: empty output could mean rate-limit. Detect via direct API status if possible.
phase1_json="$(gh_list_issues_by_label "$ISSUE_LABEL" open "$ISSUE_LIMIT")"
if [ -z "$phase1_json" ] || [ "$phase1_json" = "[]" ]; then
    rate="$(gh api rate_limit --jq '.resources.core.remaining' 2>/dev/null || echo 999)"
    if [ "${rate:-999}" = "0" ]; then
        log "GitHub rate-limit exhausted — skip tick"; exit 0
    fi
    log "Phase 1: no issues with label '${ISSUE_LABEL}' on ${GH_REPO} (will still try Phase 2 GSD-orphans)"
    phase1_json=""
fi

# Ретро-фикс (26.08 t_dfd3d19d, ADR-0032): G9a intra-tick dedup для Phase 1.
# Прогоняем phase1_json через dedup_intra_filter: группы issues с одинаковыми
# (sorted-labels, first-N-words-of-title) схлопываются в одну (старейшую по
# number), остальные skip+comment+label. Side-effects (gh calls) делаются
# внутри python, fail-OPEN. Stderr-маркеры инкрементят dedup_intra_skipped.
if [ -n "$phase1_json" ]; then
    _g9_err="$(mktemp -t g9a-err.XXXXXX 2>/dev/null)" || _g9_err="/tmp/g9a-err.$$"
    phase1_json="$(dedup_intra_filter "phase1" "$phase1_json" 2>"$_g9_err" || true)"
    if [ -s "$_g9_err" ]; then
        log "phase1 G9a intra-tick dedup: $(wc -l < "$_g9_err") skip-marker(s)"
        # Парсим маркеры формата «DEDUP_INTRA\tphase=...\tskip=N\tleader=M\t...»
        # и инкрементим счётчик + лог каждой строки.
        awk -F'\t' '
            /^DEDUP_INTRA/ {
                # fields: 1=DEDUP_INTRA, 2="phase=X", 3="skip=N", 4="leader=M", 5="title_prefix=X", 6="labels=X"
                for (i = 2; i <= NF; i++) {
                    split($i, kv, "=")
                    if (kv[1] == "skip") skp = kv[2]
                    if (kv[1] == "leader") ld = kv[2]
                    if (kv[1] == "title_prefix") tp = kv[2]
                }
                printf "  skip=#%s leader=#%s title-prefix=\"%s\"\n", skp, ld, tp
            }
        ' "$_g9_err" >&2
        dedup_intra_skipped=$((dedup_intra_skipped + $(grep -c '^DEDUP_INTRA' "$_g9_err" || true)))
    fi
    rm -f "$_g9_err" 2>/dev/null || true
fi

# Track Phase 1 issue numbers so Phase 2 doesn't double-process them.
phase1_issue_numbers=""
if [ -n "$phase1_json" ]; then
    phase1_issue_numbers="$(printf '%s' "$phase1_json" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for it in data:
    if isinstance(it, dict) and it.get("number"):
        print(it["number"])
' 2>/dev/null | sort -u | paste -sd'|' -)"
fi

# Run Phase 1 — primary filter
log "Phase 1: ${ISSUE_LABEL}-flagged issues (count=$(printf '%s' "$phase1_json" | python3 -c '
import json, sys
try: print(len(json.load(sys.stdin)))
except Exception: print(0)
' 2>/dev/null))"
process_issues_json "phase1" "$phase1_json"

# --- Phase 2: GSD-orphans (ретро t_360dc1a4, issue #1643) -----------------
# Issue #1643: создан с source:gsd + needs-e2e + area:rob_box_quest + phase:1.7,
# но БЕЗ метки hermes (специфика GSD workflow). Phase 1 его не видел → orphan
# 10ч+. Эта фаза: берём все open issues с source:gsd, фильтруем:
#   1) убрать те, что уже попали в Phase 1 (есть метка hermes) → дедуп по
#      phase1_issue_numbers
#   2) оставить только те, у которых НЕТ метки hermes (они и есть orphans)
#
# Всю остальную обработку (idempotency по existing_by_issue, throttle, big-bang,
# assignee-guard, MERGED-PR skip, role resolution, comment-marker) выполняет
# ТА ЖЕ process_issues_json — никакого дублирования guards. Это даёт
# идемпотентность «из коробки»: для #1643 уже есть карточка t_9b0786a7 →
# existing_by_issue ловит → skip, никаких дублей.
#
# Долгосрочное решение (вне scope этого PR): GSD workflow hook в
# github/workflows/gsd-*.yml, который добавляет label hermes автоматически.
# ADR-0028 фиксирует двухфазный фильтр как промежуточное решение.
GSD_SOURCE_LABEL="${GSD_SOURCE_LABEL:-source:gsd}"
phase2_json="$(gh issue list \
    --repo "$GH_REPO" \
    --label "$GSD_SOURCE_LABEL" \
    --state open \
    --json number,title,labels,body \
    --limit "$ISSUE_LIMIT" 2>/dev/null || true)"

if [ -z "$phase2_json" ] || [ "$phase2_json" = "[]" ]; then
    log "Phase 2: GSD-orphans (0 issues, source:gsd returned empty)"
    phase2_json=""
else
    # Filter: оставить ТОЛЬКО issues с source:gsd + НЕ hermes + НЕ в Phase 1.
    # Дедуп: вычитаем phase1_issue_numbers. Если hermes-метка выставлена —
    # этот issue попал в Phase 1 и обработан там, не трогаем.
    # shellcheck disable=SC2016  # python source in single quotes — no shell vars expected
    phase2_filtered="$(HERMES_LABEL="$ISSUE_LABEL" PHASE1_NUMS="$phase1_issue_numbers" \
        printf '%s' "$phase2_json" | HERMES_LABEL="$ISSUE_LABEL" PHASE1_NUMS="$phase1_issue_numbers" python3 -c '
import os, sys, json
hermes_label = os.environ.get("HERMES_LABEL", "hermes")
phase1_nums = set()
p1 = os.environ.get("PHASE1_NUMS", "")
if p1:
    for x in p1.split("|"):
        x = x.strip()
        if x.isdigit():
            phase1_nums.add(int(x))
try:
    data = json.load(sys.stdin)
except Exception:
    print("[]")
    sys.exit(0)
if not isinstance(data, list):
    print("[]")
    sys.exit(0)
keep = []
for it in data:
    if not isinstance(it, dict):
        continue
    n = it.get("number")
    if not isinstance(n, int):
        continue
    # Skip if already in Phase 1 (есть hermes → попал в phase1_json)
    if n in phase1_nums:
        continue
    # Skip if labels include hermes (defensive — phase1_nums мог не сматчить
    # если Phase 1 упал по rate-limit, но hermes-метка всё равно есть).
    label_names = {l.get("name") for l in it.get("labels", []) if isinstance(l, dict)}
    if hermes_label in label_names:
        continue
    # Skip PRs (defensive — gh issue list с --label source:gsd не должен
    # вернуть PRs, но на всякий случай). Используем `is not None` а не truthy-
    # check, потому что пустой dict {} в Python — falsy, и реальный PR из gh
    # приходит с непустым dict ({"url": ...}); но defensive-guard должен
    # работать и для синтетических пустых PR-объектов.
    if it.get("pull_request") is not None:
        continue
    keep.append(it)
print(json.dumps(keep, ensure_ascii=False))
')"

    orphan_count="$(printf '%s' "$phase2_filtered" | python3 -c '
import json, sys
try: print(len(json.load(sys.stdin)))
except Exception: print(0)
')"
    log "Phase 2: GSD-orphans (${orphan_count} issues, source:gsd without hermes)"

    # Ретро-фикс (26.08 t_dfd3d19d, ADR-0032): G9a intra-tick dedup для Phase 2.
    # Аналогично Phase 1 — прогоняем phase2_filtered через dedup_intra_filter,
    # схлопываем группировки по (labels, title-prefix), инкрементим counter.
    if [ -n "$phase2_filtered" ] && [ "$phase2_filtered" != "[]" ]; then
        _g9_err="$(mktemp -t g9a-err.XXXXXX 2>/dev/null)" || _g9_err="/tmp/g9a-err.$$"
        phase2_filtered="$(dedup_intra_filter "phase2" "$phase2_filtered" 2>"$_g9_err" || true)"
        if [ -s "$_g9_err" ]; then
            log "phase2 G9a intra-tick dedup: $(wc -l < "$_g9_err") skip-marker(s)"
            awk -F'\t' '
                /^DEDUP_INTRA/ {
                    for (i = 2; i <= NF; i++) {
                        split($i, kv, "=")
                        if (kv[1] == "skip") skp = kv[2]
                        if (kv[1] == "leader") ld = kv[2]
                        if (kv[1] == "title_prefix") tp = kv[2]
                    }
                    printf "  skip=#%s leader=#%s title-prefix=\"%s\"\n", skp, ld, tp
                }
            ' "$_g9_err" >&2
            dedup_intra_skipped=$((dedup_intra_skipped + $(grep -c '^DEDUP_INTRA' "$_g9_err" || true)))
        fi
        rm -f "$_g9_err" 2>/dev/null || true
    fi

    if [ -n "$phase2_filtered" ] && [ "$phase2_filtered" != "[]" ]; then
        process_issues_json "phase2" "$phase2_filtered"
    fi
fi

# --- summary -----------------------------------------------------------------
log "tick done: created=${created} skipped=${skipped} errored=${errored} dedup-skipped: ${dedup_intra_skipped} (intra-tick), ${dedup_race_skipped} (race)"

# Exit non-zero only on hard errors (G4/G5) so cron can alert.
if [ "$errored" -gt 0 ]; then exit 1; fi
exit 0
