#!/bin/bash
# ============================================================================
# agent-flow-nightly-review.sh — ночной ревью-цикл конвейера (ADR-0049).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-nightly-review.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/profiles/backend/scripts/
#   - ~/.hermes/profiles/analyst/scripts/
#   - ~/.hermes/scripts/
#
# ЗАЧЕМ (пробел в процессе, зафиксирован 03.09):
#   Весь надзор конвейера сегодня — РЕАКТИВНЫЙ и поштучный: watchdog ловит
#   зависший worker, merge-gate ловит stale PR, blocked-watchdog-scope ловит
#   mis-scope карточку, drift-detect ловит расхождение копий скриптов. Никто
#   не смотрит на ДЕНЬ ЦЕЛИКОМ («что вчера реально доехало, что осталось
#   висеть») и никто не смотрит на КОД, который воркеры за день написали
#   (дубли, LLM-галлюцинации, недоделки). Ревью PR-diff'а в merge-gate нет:
#   зелёный CI + e2e-done = merge. Итог — дубликаты функций и полу-фичи
#   всплывают только через ретро, когда уже сгорели токены и время.
#
# ЧТО ДЕЛАЕТ (per tick, идемпотентно):
#   1. Гейты: flock, MAINTENANCE, ночное окно [HOUR, HOUR+WINDOW_HOURS).
#   2. Собирает дайджест за ревью-сутки (REVIEW_DATE 00:00 local → now):
#      merged PR, коммиты в develop, issues open/closed, красные CI runs,
#      kanban (done / failed / всё ещё висящие), ретро-карточки за сутки.
#   3. Создаёт ОДНУ карточку «🌙 ночной ревью <REVIEW_DATE>» на
#      NIGHTLY_REVIEW_ASSIGNEE (default architect) через kanban-retro-create.sh
#      с key `nightly-review-<REVIEW_DATE>` (дедуп: один тик = одна карточка,
#      повторный тик той же ночью → SKIP).
#   4. Считает churn по компонентам (первые два сегмента пути), берёт top-N
#      кодовых компонентов и на каждый создаёт карточку
#      «🔍 ревью компонента: <comp> (<REVIEW_DATE>)» на
#      COMPONENT_REVIEW_ASSIGNEE (default analyst) с key
#      `component-review-<slug>-<REVIEW_DATE>`.
#
# ЧТО НЕ ДЕЛАЕТ (явно):
#   - НЕ чинит код и НЕ трогает метки/PR/issues. Только читает и создаёт
#     карточки. Все решения — воркер-ревьюер и Шифу.
#   - НЕ вызывает LLM сам (no_agent job). LLM работает ВНУТРИ созданной
#     карточки — так дайджест остаётся механическим (raw evidence), а
#     рассуждения живут там, где их видно и можно откатить.
#   - НЕ создаёт карточку на компонент, который ревьюили < COOLDOWN дней
#     назад (иначе src/rob_box_voice получал бы карточку каждую ночь).
#
# ENV:
#   REPO_DIR                        — клон репо (default hermes-share путь)
#   GH_REPO                         — owner/repo (из profile .env)
#   KANBAN_BOARD                    — default robbox
#   NIGHTLY_REVIEW_HOUR             — час старта окна, local TZ (default 2)
#   NIGHTLY_REVIEW_WINDOW_HOURS     — ширина окна в часах (default 4)
#   NIGHTLY_REVIEW_ASSIGNEE         — default architect
#   COMPONENT_REVIEW_ASSIGNEE       — default analyst
#   COMPONENT_REVIEW_MAX            — сколько компонентных карточек за ночь (default 3)
#   COMPONENT_REVIEW_MIN_FILES      — порог churn: минимум файлов (default 2)
#   COMPONENT_REVIEW_COOLDOWN_DAYS  — не ревьюить тот же компонент чаще (default 7)
#   COMPONENT_REVIEW_EXCLUDE_RE     — python-regex путей-исключений
#   NIGHTLY_REVIEW_DRY_RUN=true     — всё посчитать, карточки НЕ создавать
#   NIGHTLY_REVIEW_FORCE=true       — игнорировать ночное окно и sentinel
#   NIGHTLY_REVIEW_DATE=YYYY-MM-DD  — переопределить ревью-сутки (для тестов)
#   NIGHTLY_REVIEW_STATE_DIR        — где лежит sentinel (default /tmp)
#   NIGHTLY_REVIEW_TEST_MODE=1      — пропустить MAINTENANCE-гейт (сетевой
#                                     ls-remote); только для юнит-тестов
#   HERMES_BIN / GH_BIN             — бинарники (default hermes / gh)
#   RETRO_CREATE                    — путь к kanban-retro-create.sh (default рядом)
#   LOCK_FILE                       — flock (default /tmp/agent-flow-nightly-review.lock)
#
# Выходы:
#   stdout — markdown-дайджест (то же тело, что уходит в карточку);
#   stderr — структурный лог тика (для cron delivery);
#   exit 0 — ok (в т.ч. «окно не наступило» / «уже сделано этой ночью»);
#   exit 1 — критичный сбой (нет python3, нет kanban-retro-create.sh).
#
# Pitfalls:
#   - MAINTENANCE в ночном окне (agents_sleep PEAK 04:00-07:00 MSK) съедает
#     тик: окно 02:00-06:00 берёт первые два часа до PEAK. Если Шифу сдвинет
#     PEAK — двигать и NIGHTLY_REVIEW_HOUR.
#   - `gh` / `hermes` могут отсутствовать или падать. Каждая секция дайджеста
#     тогда печатает `НЕТ ДАННЫХ (<причина>)` — это честнее, чем пустой
#     список, который воркер прочтёт как «за сутки ничего не было».
#   - Компонент = первые два сегмента пути (src/rob_box_voice,
#     scripts/agent_flow, docker/vision). Файл в корне репо — сам файл.
#   - Дайджест обрезается (SECTION_LIMIT строк на секцию), иначе тело
#     карточки в дни больших мержей уходит за лимит kanban body.
#   - Python-вставки НЕ используют обратные слэши внутри f-string выражений
#     (SyntaxError на python < 3.12) — значения кладём в переменные заранее.
# ============================================================================
set -euo pipefail

PREFIX="[agent-flow-nightly-review]"

# HERMES_HOME/HOME нужны ДО первого вызова af_load_profile_env: библиотека
# читает ${HERMES_HOME} напрямую, а cron может звать нас без них — под
# `set -u` это падение «unbound variable» на первой же строке гейтов.
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
export HOME="${HOME:-/home/builder}"

REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
HERMES_BIN="${HERMES_BIN:-hermes}"
GH_BIN="${GH_BIN:-gh}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-nightly-review.lock}"
STATE_DIR="${NIGHTLY_REVIEW_STATE_DIR:-/tmp}"

NIGHTLY_REVIEW_HOUR="${NIGHTLY_REVIEW_HOUR:-2}"
NIGHTLY_REVIEW_WINDOW_HOURS="${NIGHTLY_REVIEW_WINDOW_HOURS:-4}"
NIGHTLY_REVIEW_ASSIGNEE="${NIGHTLY_REVIEW_ASSIGNEE:-architect}"
COMPONENT_REVIEW_ASSIGNEE="${COMPONENT_REVIEW_ASSIGNEE:-analyst}"
COMPONENT_REVIEW_MAX="${COMPONENT_REVIEW_MAX:-3}"
COMPONENT_REVIEW_MIN_FILES="${COMPONENT_REVIEW_MIN_FILES:-2}"
COMPONENT_REVIEW_COOLDOWN_DAYS="${COMPONENT_REVIEW_COOLDOWN_DAYS:-7}"
COMPONENT_REVIEW_EXCLUDE_RE="${COMPONENT_REVIEW_EXCLUDE_RE:-^(docs/|evidence/|analysis/|local_test/|tts_audio_bench/|[.]planning/|[.]hermes/|[.]kanban/|[.]github/e2e/|CHANGELOG|coverage|clog|MAINTENANCE|RUN_NOW)}"
DRY_RUN="${NIGHTLY_REVIEW_DRY_RUN:-false}"
FORCE="${NIGHTLY_REVIEW_FORCE:-false}"
SECTION_LIMIT="${NIGHTLY_REVIEW_SECTION_LIMIT:-40}"
MAX_RUNTIME_NIGHTLY="${NIGHTLY_REVIEW_MAX_RUNTIME:-3600}"
MAX_RUNTIME_COMPONENT="${COMPONENT_REVIEW_MAX_RUNTIME:-2700}"
export COMPONENT_REVIEW_EXCLUDE_RE
# Cron может звать нас с POSIX-локалью, а секции дайджеста печатают
# кириллицу. Без этого python падает с UnicodeEncodeError и тик умирает.
export PYTHONIOENCODING="${PYTHONIOENCODING:-utf-8}"

log() { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }

# --- shared library bootstrap ------------------------------------------------
# Отсюда берём af_load_profile_env / af_flock_guard_or_exit /
# af_maintenance_gate_or_exit — те же гейты, что у остальных cron-скриптов.
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_agent_flow_common.sh
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh"

RETRO_CREATE="${RETRO_CREATE:-$_LIB_DIR_HERE/kanban-retro-create.sh}"

af_load_profile_env ""

command -v python3 >/dev/null 2>&1 || { log "python3 not on PATH — exit 1"; exit 1; }
[ -f "$RETRO_CREATE" ] || { log "kanban-retro-create.sh не найден: $RETRO_CREATE — exit 1"; exit 1; }

# --- gates -------------------------------------------------------------------
af_flock_guard_or_exit "$LOCK_FILE"
# Test mode (tests/test_nightly_review.sh): пропускаем ТОЛЬКО MAINTENANCE-гейт —
# он делает `git ls-remote https://github.com/$GH_REPO`, то есть сетевой вызов,
# которого в юнит-тесте быть не должно. Всё остальное (flock, ночное окно,
# sentinel, дедуп) тесты проходят как в бою.
if [ "${NIGHTLY_REVIEW_TEST_MODE:-0}" != "1" ]; then
    af_maintenance_gate_or_exit
fi

# Ревью-сутки: вчерашняя локальная дата (тик идёт ночью, «вчера» = день,
# который только что закончился).
REVIEW_DATE="${NIGHTLY_REVIEW_DATE:-$(date -d 'yesterday' +%F)}"
SENTINEL="$STATE_DIR/agent-flow-nightly-review.${REVIEW_DATE}.done"

_hour_now="$(date +%-H)"
_hour_end=$((NIGHTLY_REVIEW_HOUR + NIGHTLY_REVIEW_WINDOW_HOURS))
if [ "$FORCE" != "true" ]; then
    if [ "$_hour_now" -lt "$NIGHTLY_REVIEW_HOUR" ] || [ "$_hour_now" -ge "$_hour_end" ]; then
        log "вне ночного окна [${NIGHTLY_REVIEW_HOUR}:00, ${_hour_end}:00) — сейчас ${_hour_now}:xx, skip"
        exit 0
    fi
    if [ -f "$SENTINEL" ]; then
        log "ревью за ${REVIEW_DATE} уже создано (sentinel ${SENTINEL}) — skip"
        exit 0
    fi
fi

WIN_START_LOCAL="${REVIEW_DATE} 00:00:00"
WIN_START_EPOCH="$(date -d "$WIN_START_LOCAL" +%s)"
WIN_START_UTC="$(date -u -d "$WIN_START_LOCAL" +%Y-%m-%dT%H:%M:%SZ)"
NOW_EPOCH="$(date +%s)"
NOW_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
export WIN_START_EPOCH WIN_START_UTC NOW_EPOCH SECTION_LIMIT

log "ревью-сутки ${REVIEW_DATE}: окно ${WIN_START_UTC} → ${NOW_UTC} (UTC), dry_run=${DRY_RUN}"

# --- data collectors ---------------------------------------------------------
# Каждый коллектор печатает готовую markdown-секцию. Ни один не валит тик:
# нет инструмента / упал вызов → «НЕТ ДАННЫХ (<причина>)».

_gh_json() {  # $1..=аргументы gh; печатает JSON или rc!=0
    command -v "$GH_BIN" >/dev/null 2>&1 || return 1
    [ -n "${GH_REPO:-}" ] || return 1
    "$GH_BIN" "$@" 2>/dev/null || return 1
}

section_prs() {
    local json
    if ! json="$(_gh_json pr list --repo "$GH_REPO" --state merged --limit 100 \
        --json number,title,author,mergedAt,baseRefName,url,additions,deletions,files)"; then
        echo "НЕТ ДАННЫХ (gh pr list недоступен: нет gh/GH_REPO или вызов упал)"
        return 0
    fi
    printf '%s' "$json" | python3 -c '
import json, os, sys
from datetime import datetime, timezone

start = datetime.strptime(os.environ["WIN_START_UTC"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
limit = int(os.environ.get("SECTION_LIMIT", "40"))
try:
    prs = json.load(sys.stdin)
except Exception:
    print("НЕТ ДАННЫХ (gh вернул не-JSON)")
    sys.exit(0)
rows = []
for p in prs:
    merged = p.get("mergedAt")
    if not merged:
        continue
    try:
        ts = datetime.strptime(merged, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except Exception:
        continue
    if ts >= start:
        rows.append(p)
if not rows:
    print("За сутки не смержено ни одного PR.")
    sys.exit(0)
rows.sort(key=lambda p: p.get("mergedAt") or "")
print("Всего смержено: **%d**" % len(rows))
print()
print("| PR | base | автор | +/- | файлов | title |")
print("|---|---|---|---|---|---|")
for p in rows[:limit]:
    num = p.get("number", "?")
    base = p.get("baseRefName", "?")
    author = (p.get("author") or {}).get("login", "?")
    add = p.get("additions", 0)
    dele = p.get("deletions", 0)
    nfiles = len(p.get("files") or [])
    title = (p.get("title") or "").replace("|", " ")[:90]
    print("| #%s | %s | %s | +%s/-%s | %s | %s |" % (num, base, author, add, dele, nfiles, title))
if len(rows) > limit:
    print()
    print("_(показаны первые %d из %d)_" % (limit, len(rows)))
'
}

section_commits() {
    if [ ! -d "$REPO_DIR/.git" ]; then
        echo "НЕТ ДАННЫХ (REPO_DIR=$REPO_DIR не git-клон)"
        return 0
    fi
    git -C "$REPO_DIR" fetch --quiet origin develop 2>/dev/null || \
        log "git fetch origin develop не удался — считаем по локальному origin/develop"
    local out count
    out="$(git -C "$REPO_DIR" log origin/develop --since="$WIN_START_LOCAL" \
        --pretty=format:'- `%h` %s — %an' 2>/dev/null || true)"
    if [ -z "$out" ]; then
        echo "Коммитов в origin/develop за сутки нет."
        return 0
    fi
    count="$(printf '%s\n' "$out" | wc -l | tr -d ' ')"
    printf 'Коммитов в origin/develop: **%s**\n\n' "$count"
    printf '%s\n' "$out" | head -n "$SECTION_LIMIT"
}

section_issues() {
    local json
    if ! json="$(_gh_json issue list --repo "$GH_REPO" --state all --limit 200 \
        --json number,title,state,createdAt,closedAt,labels,url)"; then
        echo "НЕТ ДАННЫХ (gh issue list недоступен)"
        return 0
    fi
    printf '%s' "$json" | python3 -c '
import json, os, sys
from datetime import datetime, timezone

start = datetime.strptime(os.environ["WIN_START_UTC"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
limit = int(os.environ.get("SECTION_LIMIT", "40"))

def parse(value):
    try:
        return datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except Exception:
        return None

try:
    issues = json.load(sys.stdin)
except Exception:
    print("НЕТ ДАННЫХ (gh вернул не-JSON)")
    sys.exit(0)
opened, closed = [], []
for i in issues:
    created = parse(i.get("createdAt") or "")
    shut = parse(i.get("closedAt") or "")
    if created and created >= start:
        opened.append(i)
    if shut and shut >= start:
        closed.append(i)
print("Заведено за сутки: **%d**, закрыто: **%d**" % (len(opened), len(closed)))
for caption, rows in (("Заведены", opened), ("Закрыты", closed)):
    if not rows:
        continue
    print()
    print("**%s:**" % caption)
    for i in rows[:limit]:
        labels = ",".join(l.get("name", "") for l in (i.get("labels") or []))
        num = i.get("number", "?")
        title = (i.get("title") or "")[:90]
        print("- #%s %s `[%s]`" % (num, title, labels))
    if len(rows) > limit:
        print("_(показаны первые %d из %d)_" % (limit, len(rows)))
'
}

section_ci() {
    local json
    if ! json="$(_gh_json run list --repo "$GH_REPO" --limit 120 \
        --json name,conclusion,status,createdAt,headBranch,url,displayTitle)"; then
        echo "НЕТ ДАННЫХ (gh run list недоступен)"
        return 0
    fi
    printf '%s' "$json" | python3 -c '
import json, os, sys
from collections import Counter
from datetime import datetime, timezone

start = datetime.strptime(os.environ["WIN_START_UTC"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
limit = int(os.environ.get("SECTION_LIMIT", "40"))
try:
    runs = json.load(sys.stdin)
except Exception:
    print("НЕТ ДАННЫХ (gh вернул не-JSON)")
    sys.exit(0)
window = []
for r in runs:
    created = (r.get("createdAt") or "")[:20]
    try:
        ts = datetime.strptime(created, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except Exception:
        continue
    if ts >= start:
        window.append(r)
if not window:
    print("Прогонов CI за сутки не видно (или окно старше retention `gh run list`).")
    sys.exit(0)
bad = [r for r in window if r.get("conclusion") in ("failure", "timed_out", "cancelled")]
print("Прогонов за сутки: **%d**, красных: **%d**" % (len(window), len(bad)))
if bad:
    print()
    top = Counter(r.get("name", "?") for r in bad).most_common(10)
    print("Топ красных workflow: " + ", ".join("`%s`x%d" % (n, c) for n, c in top))
    print()
    for r in bad[:limit]:
        print("- `%s` (%s) на `%s` — %s" % (
            r.get("name", "?"), r.get("conclusion", "?"),
            r.get("headBranch", "?"), r.get("url", "")))
    if len(bad) > limit:
        print("_(показаны первые %d из %d)_" % (limit, len(bad)))
'
}

_kanban_json() {
    command -v "$HERMES_BIN" >/dev/null 2>&1 || return 1
    "$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null || return 1
}

section_kanban() {
    local json
    if ! json="$(_kanban_json)"; then
        echo "НЕТ ДАННЫХ (hermes kanban list упал или hermes не на PATH)"
        return 0
    fi
    printf '%s' "$json" | python3 -c '
import json, os, sys
from collections import Counter

start = int(os.environ["WIN_START_EPOCH"])
now = int(os.environ["NOW_EPOCH"])
limit = int(os.environ.get("SECTION_LIMIT", "40"))
try:
    data = json.load(sys.stdin)
except Exception:
    print("НЕТ ДАННЫХ (kanban вернул не-JSON)")
    sys.exit(0)
tasks = data if isinstance(data, list) else data.get("tasks", [])

def stamp(task, keys):
    for k in keys:
        v = task.get(k)
        if v:
            try:
                return int(v)
            except Exception:
                continue
    return 0

done, failed, stuck, retro = [], [], [], []
for t in tasks:
    status = (t.get("status") or "").lower()
    end = stamp(t, ("completed_at", "updated_at", "created_at"))
    if status in ("done", "archived") and end >= start:
        done.append(t)
    if status in ("failed", "error") and end >= start:
        failed.append(t)
    if status in ("running", "blocked", "ready", "todo"):
        began = stamp(t, ("started_at", "created_at"))
        if began and (now - began) > 6 * 3600:
            stuck.append((t, (now - began) // 3600))
    title = (t.get("title") or "").lower()
    if end >= start and ("ретро" in title or "retro" in title or "process-fix" in title):
        retro.append(t)
print("Закрыто карточек: **%d**, упало: **%d**, висит >6ч: **%d**, "
      "ретро/process-fix заведено: **%d**" % (len(done), len(failed), len(stuck), len(retro)))
by_assignee = Counter((t.get("assignee") or "?") for t in done)
if by_assignee:
    print()
    print("По исполнителям (закрытые): " +
          ", ".join("`%s`x%d" % (a, c) for a, c in by_assignee.most_common()))
groups = (
    ("Упавшие карточки", [(t, 0) for t in failed]),
    ("Висят дольше 6ч", stuck),
    ("Ретро / process-fix за сутки", [(t, 0) for t in retro]),
)
for caption, rows in groups:
    if not rows:
        continue
    print()
    print("**%s:**" % caption)
    for t, age in rows[:limit]:
        age_s = (" (%dч)" % age) if age else ""
        print("- `%s` [%s/%s]%s %s" % (
            t.get("id", "?"), t.get("status", "?"), t.get("assignee", "?"),
            age_s, (t.get("title") or "")[:90]))
    if len(rows) > limit:
        print("_(показаны первые %d из %d)_" % (limit, len(rows)))
'
}

# --- component churn ---------------------------------------------------------
# Печатает TSV: <component>\t<files>\t<commits>\t<added>\t<deleted>, по убыванию
# «веса» (files*2 + commits). Пустой вывод — кода за сутки не трогали.
compute_churn() {
    [ -d "$REPO_DIR/.git" ] || return 0
    git -C "$REPO_DIR" log origin/develop --since="$WIN_START_LOCAL" \
        --numstat --pretty=format:'__COMMIT__%H' 2>/dev/null | python3 -c '
import os, re, sys

pattern = os.environ.get("COMPONENT_REVIEW_EXCLUDE_RE") or "(?!)"
exclude = re.compile(pattern)
components = {}
current = None
for line in sys.stdin:
    line = line.rstrip("\n")
    if line.startswith("__COMMIT__"):
        current = line[len("__COMMIT__"):]
        continue
    if not line.strip():
        continue
    parts = line.split("\t")
    if len(parts) != 3:
        continue
    added, deleted, path = parts
    if exclude.search(path):
        continue
    segments = path.split("/")
    if len(segments) >= 3:
        key = "/".join(segments[:2])
    else:
        key = segments[0]
    entry = components.setdefault(key, {"files": set(), "commits": set(), "add": 0, "del": 0})
    entry["files"].add(path)
    if current:
        entry["commits"].add(current)
    for raw, field in ((added, "add"), (deleted, "del")):
        try:
            entry[field] += int(raw)
        except ValueError:
            pass  # бинарь: git печатает "-"
rows = [(k, len(v["files"]), len(v["commits"]), v["add"], v["del"])
        for k, v in components.items()]
rows.sort(key=lambda r: (r[1] * 2 + r[2], r[3]), reverse=True)
for r in rows:
    print("\t".join(str(x) for x in r))
'
}

section_components() {
    local churn="$1"
    if [ -z "$churn" ]; then
        echo "Кодовые компоненты за сутки не менялись (или REPO_DIR не git-клон)."
        return 0
    fi
    echo "| компонент | файлов | коммитов | +строк | -строк |"
    echo "|---|---|---|---|---|"
    printf '%s\n' "$churn" | head -n "$SECTION_LIMIT" \
        | awk -F'\t' '{printf "| `%s` | %s | %s | +%s | -%s |\n", $1, $2, $3, $4, $5}'
}

# Файлы одного компонента за окно (для тела компонентной карточки).
component_files() {  # $1=component
    local comp="$1"
    git -C "$REPO_DIR" log origin/develop --since="$WIN_START_LOCAL" \
        --numstat --pretty=format: 2>/dev/null \
        | awk -F'\t' -v c="$comp" '
            NF==3 && index($3, c"/")==1 {
                a[$3] += ($1=="-" ? 0 : $1); d[$3] += ($2=="-" ? 0 : $2)
            }
            END { for (f in a) printf "- `%s` (+%d/-%d)\n", f, a[f], d[f] }' \
        | sort | head -n 30
}

component_commits() {  # $1=component
    local comp="$1"
    git -C "$REPO_DIR" log origin/develop --since="$WIN_START_LOCAL" \
        --pretty=format:'- `%h` %s — %an' -- "$comp" 2>/dev/null | head -n 20
}

# --- cooldown guard ----------------------------------------------------------
# Был ли компонент отревьюен за последние COOLDOWN дней? Ищем в kanban
# карточку с маркером `ретро-key: component-review-<slug>-` (его вписывает
# kanban-retro-create.sh) — живая карточка ИЛИ свежая архивная = кулдаун.
component_on_cooldown() {  # $1=slug → rc 0 = на кулдауне (пропустить)
    local slug="$1" json
    json="$(_kanban_json)" || return 1
    printf '%s' "$json" | SLUG="$slug" COOLDOWN_DAYS="$COMPONENT_REVIEW_COOLDOWN_DAYS" \
        NOW_EPOCH="$NOW_EPOCH" python3 -c '
import json, os, sys

slug = os.environ["SLUG"]
cooldown = int(os.environ["COOLDOWN_DAYS"]) * 86400
now = int(os.environ["NOW_EPOCH"])
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(1)
tasks = data if isinstance(data, list) else data.get("tasks", [])
marker = "ретро-key: component-review-" + slug + "-"
for t in tasks:
    if marker not in (t.get("body") or ""):
        continue
    if (t.get("status") or "").lower() != "archived":
        sys.exit(0)          # живая карточка на этот компонент — кулдаун
    ts = 0
    for k in ("updated_at", "completed_at", "created_at"):
        v = t.get(k)
        if v:
            try:
                ts = int(v)
                break
            except Exception:
                continue
    if ts and (now - ts) < cooldown:
        sys.exit(0)          # ревьюили недавно — кулдаун
sys.exit(1)
'
}

# --- build digest ------------------------------------------------------------
CHURN="$(compute_churn || true)"

DIGEST_FILE="$(mktemp)"
trap 'rm -f "$DIGEST_FILE"' EXIT

{
    echo "Ночной ревью за **${REVIEW_DATE}** (окно ${WIN_START_UTC} → ${NOW_UTC} UTC)."
    echo "Дайджест собран механически: \`scripts/agent_flow/agent-flow-nightly-review.sh\` (ADR-0049)."
    echo
    echo "## 1. Смержено"
    section_prs || echo "НЕТ ДАННЫХ (секция PR упала — см. stderr тика)"
    echo
    echo "## 2. Коммиты в develop"
    section_commits || echo "НЕТ ДАННЫХ (секция коммитов упала — см. stderr тика)"
    echo
    echo "## 3. Issues"
    section_issues || echo "НЕТ ДАННЫХ (секция issues упала — см. stderr тика)"
    echo
    echo "## 4. CI"
    section_ci || echo "НЕТ ДАННЫХ (секция CI упала — см. stderr тика)"
    echo
    echo "## 5. Kanban"
    section_kanban || echo "НЕТ ДАННЫХ (секция kanban упала — см. stderr тика)"
    echo
    echo "## 6. Компоненты, которые менялись"
    section_components "$CHURN" || echo "НЕТ ДАННЫХ (секция компонентов упала — см. stderr тика)"
    echo
    cat <<'TASK_EOF'
## 7. Что сделать (это и есть работа карточки)

Ты — ночной ревьюер. Дайджест выше **механический**: он говорит, что
произошло, но не говорит, хорошо ли это. Твоя работа — посмотреть на день
целиком и найти то, что конвейер пропустил.

1. **Что реально доехало.** Для каждого смерженного PR проверь, что issue,
   которую он закрывает, закрыта, а карточка не висит на доске. Расхождение
   (PR merged, а issue/карточка живые — или наоборот) — находка.
2. **Что не доделано.** Пройди по «висят >6ч» и «упавшие карточки»: почему
   стоят, кто ждёт, нужен ли пинок или декомпозиция. Не чини сам — заведи
   задачу.
3. **Что стоило бы поправить.** Красные CI, повторяющиеся ретро-темы, один и
   тот же симптом в третий раз за неделю — кандидаты в process-fix / ADR.
4. **Что осталось на сегодня.** Короткий список приоритетов на новый день:
   что горит, что заблокировано, чего ждём от Шифу.

### Контракт вывода (иначе карточка не закрыта)

- Комментарий в карточку со списком находок; на каждую — **raw evidence**
  (номер PR/issue, `t_<id>` карточки, ссылка на run, `file:line`). Без raw
  находка не считается (AGENTS.md, ADR-0018).
- Реальные дефекты — **отдельные GitHub issues** с меткой `hermes`, чтобы их
  подхватил триаж. НЕ чинить руками в этой карточке.
- Если находок нет — так и напиши: «находок нет», с перечислением того, что
  проверил. Честный пустой отчёт лучше выдуманного списка.
TASK_EOF
} > "$DIGEST_FILE"

cat "$DIGEST_FILE"

# --- create nightly card -----------------------------------------------------
NIGHTLY_TITLE="🌙 ночной ревью ${REVIEW_DATE}"
NIGHTLY_KEY="nightly-review-${REVIEW_DATE}"
created_nightly=""

if [ "$DRY_RUN" = "true" ]; then
    log "DRY-RUN: карточка '${NIGHTLY_TITLE}' (key=${NIGHTLY_KEY}, assignee=${NIGHTLY_REVIEW_ASSIGNEE}) НЕ создаётся"
else
    if created_nightly="$(bash "$RETRO_CREATE" \
        --board "$KANBAN_BOARD" \
        --title "$NIGHTLY_TITLE" \
        --body "$(cat "$DIGEST_FILE")" \
        --assignee "$NIGHTLY_REVIEW_ASSIGNEE" \
        --key "$NIGHTLY_KEY" \
        --max-runtime "$MAX_RUNTIME_NIGHTLY" 2>&1)"; then
        log "nightly card: ${created_nightly}"
    else
        log "nightly card: ОШИБКА создания — ${created_nightly}"
    fi
fi

# --- create component review cards -------------------------------------------
_comp_created=0
_comp_skipped_cooldown=0
_comp_skipped_small=0

if [ -n "$CHURN" ]; then
    while IFS=$'\t' read -r comp files commits added deleted; do
        [ -n "$comp" ] || continue
        [ "$_comp_created" -lt "$COMPONENT_REVIEW_MAX" ] || break
        if [ "${files:-0}" -lt "$COMPONENT_REVIEW_MIN_FILES" ]; then
            _comp_skipped_small=$((_comp_skipped_small + 1))
            continue
        fi
        comp_slug="$(printf '%s' "$comp" | tr '/' '-' | tr '[:upper:]' '[:lower:]' | tr -cd 'a-z0-9._-')"
        if component_on_cooldown "$comp_slug"; then
            log "компонент ${comp}: на кулдауне (${COMPONENT_REVIEW_COOLDOWN_DAYS}д) — карточку не создаю"
            _comp_skipped_cooldown=$((_comp_skipped_cooldown + 1))
            continue
        fi

        comp_body="$(
            printf 'Ревью компонента `%s` по изменениям за **%s** (окно %s → %s UTC).\n\n' \
                "$comp" "$REVIEW_DATE" "$WIN_START_UTC" "$NOW_UTC"
            printf 'Churn за сутки: **%s** файлов, **%s** коммитов, +%s/-%s строк.\n\n' \
                "$files" "$commits" "$added" "$deleted"
            echo "## Изменённые файлы"
            component_files "$comp"
            echo
            echo "## Коммиты"
            component_commits "$comp"
            echo
            cat <<'COMP_TASK_EOF'
## Что искать (это и есть работа карточки)

Код в этот компонент за сутки писали LLM-воркеры под давлением зелёного CI.
CI ловит падения, но не ловит четыре вещи — их ищешь ты:

1. **Дубликаты.** Новая функция / скрипт / константа, повторяющие уже
   существующие в этом же компоненте или в общих либах
   (`lib_agent_flow_common.sh`, общие utils пакета). Типовой симптом —
   воркер не нашёл существующее и написал своё.
2. **Глюки LLM.** Мёртвый код и недостижимые ветки; вызовы несуществующих
   API/полей; заглушки, которые всегда возвращают успех; комментарий или
   docstring, противоречащие коду; «исправление», продублированное дважды в
   двух местах; тесты, которые ничего не проверяют (assert True, мок
   проверяется сам на себя).
3. **Недоделки.** `TODO` / `FIXME` / `NotImplementedError`, уехавшие в
   merged-код; фича без теста; код без регистрации (нода не в launch,
   скрипт не в `install.sh` EXPECTED, параметр не в конфиге); документация,
   отставшая от кода.
4. **Расхождение с контрактом.** Поведение кода против того, что обещают
   README / ADR / `docs/` для этого компонента.

## Контракт вывода (иначе карточка не закрыта)

- Комментарий в карточку: список находок, на каждую — `file:line` и цитата
  строки. Без `file:line` находка не считается (AGENTS.md, ADR-0018).
- Каждая подтверждённая находка — **отдельный GitHub issue** с меткой
  `hermes` (или одна issue на группу однотипных), чтобы её подхватил триаж.
- **НЕ чинить в этой карточке.** Ревью не открывает PR с фиксами: работа
  ревью — найти и описать. Исключение — Шифу явно попросил в комментарии.
- Находок нет → напиши «находок нет» и перечисли, что именно проверил.
COMP_TASK_EOF
        )"

        comp_title="🔍 ревью компонента: ${comp} (${REVIEW_DATE})"
        comp_key="component-review-${comp_slug}-${REVIEW_DATE}"

        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN: компонентная карточка '${comp_title}' (key=${comp_key}) НЕ создаётся"
            _comp_created=$((_comp_created + 1))
            continue
        fi
        if comp_out="$(bash "$RETRO_CREATE" \
            --board "$KANBAN_BOARD" \
            --title "$comp_title" \
            --body "$comp_body" \
            --assignee "$COMPONENT_REVIEW_ASSIGNEE" \
            --key "$comp_key" \
            --max-runtime "$MAX_RUNTIME_COMPONENT" 2>&1)"; then
            log "component card [${comp}]: ${comp_out}"
            _comp_created=$((_comp_created + 1))
        else
            log "component card [${comp}]: ОШИБКА создания — ${comp_out}"
        fi
    done <<< "$CHURN"
fi

if [ "$DRY_RUN" != "true" ]; then
    : > "$SENTINEL" 2>/dev/null || log "не смог записать sentinel ${SENTINEL} (не фатально)"
fi

log "итог: nightly='${created_nightly:-dry-run}' component_cards=${_comp_created} skipped_cooldown=${_comp_skipped_cooldown} skipped_small=${_comp_skipped_small}"
exit 0
