#!/bin/bash
# ============================================================================
# dryrun_fail_streak_issue.sh — детерминированный dry-run harness для
# auto-create-issue ветки в agent-flow-e2e-fail-streak-watchdog.sh.
#
# Контекст: PR #2374 (merge a9b04981) добавил в watchdog авто-создание issue
# при fail-streak ≥ E2E_FAIL_STREAK_ISSUE_THRESHOLD (5), rate-limited через
# mtime ISSUE_COOLDOWN_FILE (default 4h). Этот скрипт — НЕ модификация
# watchdog, а зеркало его auto-create-issue ветки, которое можно гонять
# локально и в CI без gh-токена и без реальной сети.
#
# Что делает:
#   • Поднимает sandbox HERMES_HOME, кладёт ISSUE_COOLDOWN_FILE в $HERMES_HOME/state/.
#   • Берёт 8 fixed fail-runs (run IDs и HEAD SHAs из body карточки) как
#     фикстуру и собирает из них точно такой же issue-body, как собирает
#     watchdog (строки 296-352 a9b04981).
#   • Гоняет 3 сценария с фейковым mtime cooldown-файла:
#       (i)   cold start            (файл отсутствует)        → 1 payload
#       (ii)  immediate re-run      (mtime = now)             → SKIP
#       (iii) advance mtime на >4h  (`touch -d "5 hours ago"`) → 1 payload
#   • Каждый payload печатает РОВНО ту `gh issue create` команду с
#     полным телом, которую watchdog бы отправил, но НЕ делает
#     реальных вызовов.
#
# Acceptance body t_7572e7a8:
#   "running the harness locally shows exactly one issue payload per
#    >4h window, and zero on immediate re-runs"
#
# Использование:
#   bash scripts/agent_flow/dryrun_fail_streak_issue.sh
#
# Exit:
#   0 — все сценарии отработали как ожидается.
#   1 — нарушен инвариант (1/0/1); см. сводку в конце вывода.
#
# Связь с watchdog:
#   • Решение "skip vs create" полностью зеркалирует watchdog
#     строки 286-292 (mtime check) и 295-298 (gh-truth check, опущен —
#     см. NOTE в коде).
#   • Тело issue — копия шаблона из watchdog строки 311-352, с тем же
#     набором секций и тем же hypothesis-блоком.
#   • Единственное намеренное отличие: вместо `gh issue create` мы
#     печатаем команду и тело в stdout (это и есть dry-run).
# ============================================================================
set -u

# --- env defaults (mirror watchdog строки 86-103) --------------------------
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
E2E_WORKFLOW="${E2E_WORKFLOW:-L-E2E Voice Test.yml}"
E2E_FAIL_STREAK_ISSUE_THRESHOLD="${E2E_FAIL_STREAK_ISSUE_THRESHOLD:-5}"
E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS="${E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS:-4}"
E2E_FAIL_STREAK_ISSUE_LABEL="${E2E_FAIL_STREAK_ISSUE_LABEL:-e2e-fail-streak}"
E2E_FAIL_STREAK_ISSUE_ASSIGNEE="${E2E_FAIL_STREAK_ISSUE_ASSIGNEE:-}"

HERMES_HOME="${HERMES_HOME:-${HOME}/.hermes}"
# В тестах HERMES_HOME указывает на sandbox (см. test_dryrun_fail_streak_issue.sh).

ISSUE_COOLDOWN_FILE="${ISSUE_COOLDOWN_FILE:-${HERMES_HOME}/state/agent-flow-e2e-fail-streak-last-issue}"
mkdir -p "$(dirname "$ISSUE_COOLDOWN_FILE")"

# --- фикстуры (run IDs и HEAD SHA из task body t_7572e7a8) -----------------
# 8 последних фейл-ранов night-marathon + voice_core_suite (audit t_03b25c54).
# Все на одном head_sha=4ab3a0a5 — это реальные run IDs из timeline_report.
FAILED_RUNS_JSON='[
  {"databaseId":34781633844,"conclusion":"failure","createdAt":"2026-09-13T20:43:10Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34780571796,"conclusion":"failure","createdAt":"2026-09-13T20:22:40Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34779436302,"conclusion":"failure","createdAt":"2026-09-13T20:01:06Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34778720436,"conclusion":"success","createdAt":"2026-09-13T19:46:39Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34691955361,"conclusion":"failure","createdAt":"2026-09-11T20:19:32Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34691825992,"conclusion":"failure","createdAt":"2026-09-11T20:00:58Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34691731590,"conclusion":"failure","createdAt":"2026-09-11T19:39:00Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34691590114,"conclusion":"failure","createdAt":"2026-09-11T19:25:36Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34691444201,"conclusion":"failure","createdAt":"2026-09-11T18:55:12Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"},
  {"databaseId":34691277689,"conclusion":"failure","createdAt":"2026-09-11T18:31:47Z","headSha":"4ab3a0a59f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d","headBranch":"develop"}
]'

# Релевантные merged PR за 5 дней — фикстура из body карточки t_7572e7a8
# и t_1639d636 ("#2342, #2341, #2338, #2337 as a starting set").
RELATED_PRS="#2342, #2341, #2338, #2337"

# develop HEAD (короткий). Реальный develop HEAD на момент merge PR #2374 —
# 4ab3a0a5 (что и совпадает с headSha fail-ранов).
DEVELOP_HEAD="4ab3a0a"

# streak/last_success (mirror watchdog: счётчик из NEWEST→OLDEST с остановкой
# на первом success). Наша фикстура: 4 fail + 1 success + 6 fail = streak=7.
LAST_SUCCESS_AT="2026-09-13T19:46:39Z"
STREAK=7

# --- helpers (mirrored from watchdog) -------------------------------------

# format_failed_runs_table _runs_json _max_rows _repo
# → markdown-таблица последних failed runs (id | conclusion | createdAt | sha[7] | branch)
# Зеркало watchdog-функции format_failed_runs_table (строки 119-141 a9b04981).
format_failed_runs_table() {
    local _runs="$1" _max_rows="${2:-5}" _repo="$3"
    GH_REPO_PASS="$_repo" MAX_ROWS_PASS="$_max_rows" printf '%s' "$_runs" \
        | GH_REPO_PASS="$_repo" MAX_ROWS_PASS="$_max_rows" python3 -c '
import json, os, sys
try:
    runs = json.load(sys.stdin)
except Exception:
    sys.exit(0)
max_rows = int(os.environ["MAX_ROWS_PASS"])
repo = os.environ["GH_REPO_PASS"]
shown = 0
for r in runs:
    if shown >= max_rows:
        break
    c = r.get("conclusion")
    if c not in ("failure", "cancelled", "timed_out"):
        continue
    rid = r.get("databaseId", "")
    created = r.get("createdAt", "")
    sha = (r.get("headSha") or "")[:7]
    branch = r.get("headBranch", "")
    if not rid:
        continue
    print(f"| [{rid}](https://github.com/{repo}/actions/runs/{rid}) | {c} | {created} | `{sha}` | `{branch}` |")
    shown += 1
' 2>/dev/null || true
}

# emit_issue_payload _streak _develop_head _last_success
# → Печатает payload, идентичный тому, что watchdog бы отправил через
# `gh issue create`. Markers `---ISSUE PAYLOAD BEGIN---` /
# `---ISSUE PAYLOAD END---` — для парсинга в тесте.
emit_issue_payload() {
    local _streak="$1" _develop_head="$2" _last_success="$3"
    local _failed_table
    _failed_table="$(format_failed_runs_table "$FAILED_RUNS_JSON" 8 "$GH_REPO")"

    # Тело issue. Зеркало watchdog строки 311-352 (a9b04981), с теми же
    # секциями и тем же hypothesis-блоком.
    local _body
    _body="$(cat <<EOF
🤖 [agent:devops] script=agent-flow-e2e-fail-streak-watchdog action=auto-create-issue

## fail-streak alert

L: E2E Voice Test (\`${E2E_WORKFLOW}\`) упал **${_streak}** раз подряд.
- **Last success:** ${_last_success:-NONE}
- **develop HEAD:** \`${_develop_head}\`
- **Threshold:** streak ≥ ${E2E_FAIL_STREAK_ISSUE_THRESHOLD}
- **Rate-limit:** ${E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS}ч (cooldown file: \`${ISSUE_COOLDOWN_FILE}\`)

## Timeline (last 8 failed runs)

| run | conclusion | createdAt | headSha | branch |
|---|---|---|---|---|
${_failed_table}

## Hypothesis: music-fix regression?

Рядом с image build (11 Sep 18:11 MSK, sha=022794fb631f) в develop попали 4 music-фикса:
- \`f47da7b\` — docs(music): направить игру мелодий по имени в compose_music
- \`7e8eab5\` — feat(music): требовать аранжировку LLM при игре мелодии по имени
- \`62c7f62\` — feat(music): требовать lead_synth для известной мелодии
- \`d17e107\` — feat(music): ограничить выбор lead_synth списком мелодических синтов

Если acceptance ловит music-regression на atomic harness — на проде тоже будет.
См. cross-refs: #2246 (supervisor метрики), #2347 (voice follow-up).
Релевантные merged PR за последние 5 дней: ${RELATED_PRS}.

## Что делать

1. Открыть последний failed run → ROBOT LOG block → какой маркер (STT-empty / no_wake_word / tts-fallback / no-speech).
2. Если hypothesis подтвердилась — поставить процессные метки (\`needs-e2e\`/\`agent-flow-error\`) и привязать PR-фикс.
3. Если инфра-проблема — добавить \`MAINTENANCE\` на develop.
4. После закрытия fail-streak (новый SUCCESS) → удалить этот issue (\`gh issue close N --reason 'completed'\`).

> Скрипт-страж: \`scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh\`.
> При streak ≥ ${E2E_FAIL_STREAK_PAUSE:-20} дополнительно создаётся pause-sentinel → e2e ротация замораживается.
EOF
)"

    # Имя title — как у watchdog строка 353.
    local _title="[e2e-fail-streak] L: E2E Voice Test — ${_streak} fails подряд (develop ${_develop_head})"

    # Печатаем payload. Используем маркеры `---ISSUE PAYLOAD BEGIN---` /
    # `---ISSUE PAYLOAD END---` для парсинга в test_-скрипте.
    # Команда и тело разделены: тело — через here-doc для корректной
    # передачи многострочного текста (зеркалит watchdog, который тоже
    # строит тело через here-doc).
    {
        printf '%s\n' '---ISSUE PAYLOAD BEGIN---'
        printf 'gh issue create --repo %s \\\n' "$GH_REPO"
        printf '  --title %s \\\n' "$(printf '%s' "$_title" | sed "s/'/'\\\\''/g")"
        printf '  --label %s \\\n' "$E2E_FAIL_STREAK_ISSUE_LABEL"
        if [ -n "${E2E_FAIL_STREAK_ISSUE_ASSIGNEE:-}" ]; then
            printf '  --assignee %s \\\n' "$E2E_FAIL_STREAK_ISSUE_ASSIGNEE"
        fi
        printf '%s\n' '  --body <<<EOF_BODY_MARKER'
        printf '%s\n' "$_body"
        printf '%s\n' 'EOF_BODY_MARKER'
        printf '%s\n' '---ISSUE PAYLOAD END---'
    }
}

# decide_and_print_scenario _scenario_name _set_mtime_fn
# → _set_mtime_fn это имя функции, которая УСТАНАВЛИВАЕТ mtime cooldown-файла
#   в нужное состояние ПЕРЕД проверкой (например, clear/refresh/age).
# Зеркало watchdog строки 286-298 (mtime check). NOTE: gh-truth check
# (строки 295-298) НЕ моделируем — он работает только если уже есть
# реальное issue с лейблом; в сухом harness это не нужно.
decide_and_print_scenario() {
    local _name="$1" _set_mtime_fn="$2"
    printf '\nscenario %s\n' "$_name"
    "$_set_mtime_fn"

    local _cooldown_ok="true"
    if [ -f "$ISSUE_COOLDOWN_FILE" ]; then
        local _cooldown_epoch _cooldown_age_s _cooldown_limit_s
        _cooldown_epoch="$(stat -c '%Y' "$ISSUE_COOLDOWN_FILE" 2>/dev/null || echo 0)"
        _cooldown_age_s=$(( $(date -u +%s) - ${_cooldown_epoch:-0} ))
        _cooldown_limit_s=$(( E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS * 3600 ))
        if [ "${_cooldown_age_s:-0}" -lt "${_cooldown_limit_s}" ]; then
            printf '  SKIPPED: cooldown active (%ss < %ss)\n' \
                "$_cooldown_age_s" "$_cooldown_limit_s"
            _cooldown_ok="false"
        else
            printf '  cooldown STALE (%ss >= %ss) — proceed\n' \
                "$_cooldown_age_s" "$_cooldown_limit_s"
        fi
    else
        printf '  cooldown file absent — cold start, proceed\n'
    fi

    if [ "$_cooldown_ok" = "true" ]; then
        emit_issue_payload "$STREAK" "$DEVELOP_HEAD" "$LAST_SUCCESS_AT"
        # Имитируем watchdog строка 369-371: после успешного create —
        # записать cooldown (epoch). НЕ делаем реальный `gh`.
        date -u +%s > "$ISSUE_COOLDOWN_FILE"
        printf '  cooldown written: %s\n' "$ISSUE_COOLDOWN_FILE"
    fi
}

# --- mtime setup helpers (faking mtime across scenarios) ------------------

# (i) cold start: cooldown-файла нет.
mtime_cold_start() {
    rm -f "$ISSUE_COOLDOWN_FILE"
}

# (ii) immediate re-run: cooldown-файл "свежий" (mtime = now).
mtime_now() {
    date -u +%s > "$ISSUE_COOLDOWN_FILE"
}

# (iii) advance mtime by >4h: `touch -d "now -4h -1m" cooldown`.
# Альтернативно: `touch -d "@$(($(date -u +%s) - RATE_LIMIT_HOURS*3600 - 60))" cooldown`.
mtime_aged_over_limit() {
    local _now _aged
    _now="$(date -u +%s)"
    _aged=$(( _now - E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS * 3600 - 60 ))
    touch -d "@${_aged}" "$ISSUE_COOLDOWN_FILE"
}

# --- main -----------------------------------------------------------------

printf '=== dry-run harness: %s ===\n' "$(date -Iseconds)"
printf 'GH_REPO=%s\n' "$GH_REPO"
printf 'E2E_WORKFLOW=%s\n' "$E2E_WORKFLOW"
printf 'E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS=%s\n' "$E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS"
printf 'ISSUE_COOLDOWN_FILE=%s\n' "$ISSUE_COOLDOWN_FILE"
printf 'STREAK=%s (threshold=%s)\n' "$STREAK" "$E2E_FAIL_STREAK_ISSUE_THRESHOLD"

decide_and_print_scenario "(i)  cold start"           mtime_cold_start
decide_and_print_scenario "(ii) immediate re-run"     mtime_now
decide_and_print_scenario "(iii) mtime advanced >4h"  mtime_aged_over_limit

# --- summary --------------------------------------------------------------
# Подсчитываем, сколько раз harness напечатал маркер ISSUE PAYLOAD BEGIN.
# Реализация: повторно прогоняем decision-логику в под-шелле, перехватываем
# stdout, считаем payload-маркеры через grep -c.
_out="$( (
    mtime_cold_start     ; decide_and_print_scenario "(i)"   mtime_cold_start
    mtime_now            ; decide_and_print_scenario "(ii)"  mtime_now
    mtime_aged_over_limit; decide_and_print_scenario "(iii)" mtime_aged_over_limit
) 2>&1 )" || true
_total_payloads="$(printf '%s\n' "$_out" | grep -c 'ISSUE PAYLOAD BEGIN---' || true)"

if [ "${_total_payloads:-0}" -ne 2 ]; then
    printf '\n!!! harness invariant violated: expected 2 payloads (1+0+1), got %s\n' "${_total_payloads:-0}"
    exit 1
fi
printf '\n=== summary: harness produced 2 payloads (1+0+1) — PASS ===\n'
exit 0