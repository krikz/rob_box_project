#!/bin/bash
# ============================================================================
# agent-flow-e2e-fail-streak-watchdog.sh — auto-escalation when L: E2E Voice
# Test workflow fails repeatedly without an intervening success.
#
# Контекст / ретро t_faac94b0 (2026-08-28):
#   L: E2E Voice Test fail-streak 24 раунда подряд (~3 дня) без какого-либо
#   автоматического алерта. Issue #1668 (STT-регрессия) открыт 46h+, без
#   process-меток → drift. PR #1673 (фикс wake-gate) готов, но не смёржен
#   из-за отсутствия process-маркера.
#
#   Причина: agent-flow-e2e-process-launcher.sh (cron 84864db04347, every 20m)
#   только ротирует раунды, fail-streak не проверяет. Issue остаются
#   «осиротевшими» пока кто-то вручную не поставит stale-candidate
#   (но unlabeled-sweep упал с GH_REPO must be set — 19 failures в ряд).
#
# Контракт (per tick, no-agent bash, вызывается из launcher):
#   1. List последние N (default 30) E2E workflow runs across all branches
#      (conclusion != null, status == completed).
#   2. Считаем streak_with_success — от самого нового run считаем FAIL'ы подряд,
#      пока не встретим SUCCESS (streak = 0). Если streak > STREAK_WARN (5):
#        - Issue comment в открытый process-релевантный issue (поиск по
#          label `needs-e2e` ИЛИ недавний FAIL-round ветке → если есть
#          упоминание в issue body — fallback на #1668 как known issue).
#        - log alert для cron-delivery.
#      Если streak > STREAK_WARN (5) И нет открытого issue с лейблом
#      `e2e-fail-streak` → СОЗДАТЬ новый issue с этим лейблом + статистикой
#      streak, последними failed runs и ссылкой на последние develop-коммиты
#      (включая music-фиксы). Rate-limit через RATE_LIMIT_HOURS (default 4ч).
#      Если streak > STREAK_PAUSE (20):
#        - Auto-pause: создаём файл PAUSE_SENTINEL → e2e-process в начале
#          следующего тика увидит sentinel и пропустит round creation
#          (аналог MAINTENANCE gate).
#        - Manual override через удаление файла (решение принимает Шиф).
#   3. Idempotent: comment пишется только если последний marker старше
#      MARKER_DEDUP_HOURS (default 6h). Pause-sentinel НЕ снимается —
#      это manual override (Шиф/юзер).
#   4. AUTO-CREATE ISSUE (ADR-FS-001, ретро t_401e52de):
#      При streak ≥ E2E_FAIL_STREAK_ISSUE_THRESHOLD (default 5) И НЕТ
#      открытого issue с лейблом `e2e-fail-streak` → создать ОДИН issue
#      через `gh issue create` с body: timeline failed runs (id/conclusion/
#      createdAt/headSha[7]/headBranch), develop HEAD SHA, релевантные
#      merged PR (через `git log origin/develop --merges --since=…`), и
#      hypothesis `music-fix regression` со ссылками на #2246/#2347 и
#      на другие связанные issue из последних PR.
#      Rate-limit: mtime ISSUE_COOLDOWN_FILE < E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS
#      → skip. Файл живёт в $HERMES_HOME/state/agent-flow-e2e-fail-streak-last-issue
#      и содержит epoch последнего успешного создания issue (Unix time).
#      Если `gh issue list --label e2e-fail-streak --state open` уже
#      возвращает хотя бы 1 → skip (идемпотентность по GitHub, не по локальному
#      state — на случай если state-файл потерян, но issue уже висит).
#
# ENV:
#   GH_REPO                                  — owner/repo (default krikz/rob_box_project)
#   FAIL_STREAK_DRY_RUN=true                 — log only, no API writes
#   E2E_FAIL_STREAK_WARN=5                   — порог алерта (issue comment)
#   E2E_FAIL_STREAK_PAUSE=20                 — порог auto-pause (sentinel file)
#   E2E_FAIL_STREAK_LIMIT=30                 — сколько последних run'ов смотрим
#   E2E_FAIL_STREAK_DEDUP_HOURS=6            — дедупликация алерт-комментариев
#   E2E_FAIL_STREAK_ISSUE_THRESHOLD=5        — порог для auto-create issue
#                                              (default = WARN: один тик на каждый streak)
#   E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS=4 — дедупликация создания issue
#   E2E_FAIL_STREAK_ISSUE_LABEL=e2e-fail-streak — лейбл нового issue
#   E2E_FAIL_STREAK_ISSUE_ASSIGNEE=          — assignee (пусто = без assignee)
#   HERMES_HOME                              — для sentinel path (default ~/.hermes)
#   REPO_DIR                                 — путь к локальному clone репо (для
#                                              `git -C` develop HEAD + merges).
#                                              Если пусто — fallback на `git`
#                                              без -C (текущий cwd).
#   LOCK_FILE                                — flock guard
#
# Выходы:
#   - Exit 0 — всё ok (даже если streak=0).
#   - Exit 1 — критичный сбой (нет gh auth, sentinel write failed).
#
# Pitfalls:
#   - gh run list --workflow принимает filename на DEFAULT branch (см.
#     github-actions-orchestration skill). Workflow display name «L: E2E
#     Voice Test» на main == «L-E2E Voice Test.yml». Используем filename.
#   - Без --branch: берём все round-ветки (z-{e2e}/test-round-*) +
#     main/master, чтобы streak не сбрасывался на каждом раунде.
#   - Sentinel — НЕ блокер навечно: при streak > 20 e2e ротация
#     замораживается пока человек не разберётся (это намеренно — следующие
#     20+ FAIL'ов только усугубят ситуацию и сожгут CI minutes).
# ============================================================================
set -uo pipefail  # без -e — ошибки не должны убивать cron

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
E2E_WORKFLOW="${E2E_WORKFLOW:-L-E2E Voice Test.yml}"
E2E_FAIL_STREAK_WARN="${E2E_FAIL_STREAK_WARN:-5}"
E2E_FAIL_STREAK_PAUSE="${E2E_FAIL_STREAK_PAUSE:-20}"
E2E_FAIL_STREAK_LIMIT="${E2E_FAIL_STREAK_LIMIT:-30}"
E2E_FAIL_STREAK_DEDUP_HOURS="${E2E_FAIL_STREAK_DEDUP_HOURS:-6}"
E2E_FAIL_STREAK_ISSUE_THRESHOLD="${E2E_FAIL_STREAK_ISSUE_THRESHOLD:-5}"
E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS="${E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS:-4}"
E2E_FAIL_STREAK_ISSUE_LABEL="${E2E_FAIL_STREAK_ISSUE_LABEL:-e2e-fail-streak}"
E2E_FAIL_STREAK_ISSUE_ASSIGNEE="${E2E_FAIL_STREAK_ISSUE_ASSIGNEE:-}"
HERMES_HOME="${HERMES_HOME:-${HOME}/.hermes}"
REPO_DIR="${REPO_DIR:-}"     # for `git -C` (develop HEAD + recent merges)
DRY_RUN="${FAIL_STREAK_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-fail-streak-watchdog.lock}"
PAUSE_SENTINEL="${PAUSE_SENTINEL:-${HERMES_HOME}/state/agent-flow-e2e-fail-streak-pause}"
ISSUE_COOLDOWN_FILE="${ISSUE_COOLDOWN_FILE:-${HERMES_HOME}/state/agent-flow-e2e-fail-streak-last-issue}"
MARKER_TAG="🤖 [agent:devops] script=agent-flow-e2e-fail-streak-watchdog streak=${E2E_FAIL_STREAK_WARN}+"

PREFIX="[agent-flow-e2e-fail-streak-watchdog]"

log() { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }

# --- helpers ---------------------------------------------------------------

# format_failed_runs_table _runs_json _max_rows _repo
# → печатает markdown-таблицу последних failed runs (id | conclusion | createdAt | sha[7] | branch)
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

# --- flock guard ----------------------------------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    log "another instance running — skip"
    exit 0
fi

# --- gh auth probe --------------------------------------------------------
if ! command -v gh >/dev/null 2>&1; then
    log "ERROR: gh CLI not found"
    exit 1
fi
if ! gh auth status >/dev/null 2>&1; then
    log "ERROR: gh auth failed"
    exit 1
fi

# --- compute fail-streak (newest → oldest, stop at first success) ---------
log "querying last ${E2E_FAIL_STREAK_LIMIT} runs of ${E2E_WORKFLOW}"
_runs_json="$(gh run list --repo "$GH_REPO" --workflow "$E2E_WORKFLOW" \
    --limit "$E2E_FAIL_STREAK_LIMIT" --json databaseId,conclusion,createdAt,headBranch,name 2>/dev/null || true)"

if [ -z "$_runs_json" ] || [ "$_runs_json" = "[]" ]; then
    log "no runs found (workflow may not exist yet) — skip"
    exit 0
fi

# Compute streak + last_success_at via python3 (json reliable here).
_streak_info="$(printf '%s' "$_runs_json" | python3 -c '
import json, sys
try:
    runs = json.load(sys.stdin)
except Exception:
    print("ERR"); raise SystemExit(0)
streak = 0
last_success_at = ""
for r in runs:  # already newest-first
    c = r.get("conclusion")
    if c == "success":
        last_success_at = r.get("createdAt", "")
        break
    if c in ("failure", "cancelled", "timed_out"):
        streak += 1
    # in_progress / queued / null conclusion → пропускаем (не failure)
print(f"{streak}|{last_success_at}")
' 2>/dev/null || echo "ERR|")"

if [ "${_streak_info%%|*}" = "ERR" ]; then
    log "ERROR: cannot parse runs json"
    exit 1
fi
_streak="${_streak_info%%|*}"
_last_success_at="${_streak_info#*|}"
log "streak=${_streak} last_success=${_last_success_at:-NEVER} warn=${E2E_FAIL_STREAK_WARN} pause=${E2E_FAIL_STREAK_PAUSE}"

# --- decide action --------------------------------------------------------
if [ "${_streak:-0}" -lt "$E2E_FAIL_STREAK_WARN" ] 2>/dev/null; then
    log "streak < WARN — no action"
    exit 0
fi

# Найти issue для alert: открытые issues с label needs-e2e (в ротации) ИЛИ
# fallback на конкретный known issue из задачи (issue #1668).
_target_issue=""
_target_issues="$(gh issue list --repo "$GH_REPO" --state open --label needs-e2e \
    --limit 5 --json number,title 2>/dev/null || echo '[]')"
_n_count="$(printf '%s' "$_target_issues" | python3 -c 'import json,sys;print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
log "open needs-e2e issues: ${_n_count}"

# Если needs-e2e пусто — ищем «голые» открытые issues, привязанные к fail-streak
# (например, issue #1668 был открыт 26.08 без process-меток). Берём 3 самых
# старых open issue без process-меток (это то, что должен был поймать
# unlabeled-sweep, но он упал).
if [ "${_n_count:-0}" -eq 0 ] 2>/dev/null; then
    _target_issues="$(gh issue list --repo "$GH_REPO" --state open \
        --json number,title,labels,createdAt \
        --jq '[.[] | select((.labels | map(.name) | inside(["hermes","needs-e2e","e2e-done","e2e:rejected","no-e2e-required","agent-flow-error"])) | not)] | sort_by(.createdAt) | .[0:3]' \
        2>/dev/null || echo '[]')"
    _n_count="$(printf '%s' "$_target_issues" | python3 -c 'import json,sys;print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
    log "open unlabeled-process issues (fallback pool): ${_n_count}"
fi

# --- WARN: issue comment (idempotent по marker dedup window) --------------
if [ "${_streak:-0}" -ge "$E2E_FAIL_STREAK_WARN" ] 2>/dev/null; then
    _now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    _now_epoch="$(date -u +%s)"
    _dedup_epoch=$((_now_epoch - E2E_FAIL_STREAK_DEDUP_HOURS * 3600))

    while IFS=$'\t' read -r _issue_num _issue_title; do
        [ -z "$_issue_num" ] && continue
        # Проверяем был ли marker за последние dedup hours
        _recent_marker="$(gh api "repos/${GH_REPO}/issues/${_issue_num}/comments?since=$(date -u -d "@${_dedup_epoch}" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo 1970-01-01T00:00:00Z)&per_page=100" \
            --jq '[.[] | select(.body | startswith("'"${MARKER_TAG}"'"))] | length' 2>/dev/null || echo 0)"
        if [ "${_recent_marker:-0}" -gt 0 ] 2>/dev/null; then
            log "issue #${_issue_num}: marker already posted within ${E2E_FAIL_STREAK_DEDUP_HOURS}h — skip"
            continue
        fi

        _body="${MARKER_TAG} fail-streak=${_streak} (>${E2E_FAIL_STREAK_WARN}) last_success=${_last_success_at:-NONE}.

Workflow \`${E2E_WORKFLOW}\` упал ${_streak} раз подряд без SUCCESS. Это не похоже на обычный рандомный fail — возможна регрессия (голос/STT/wake-gate/network/инфра).

Рекомендуемые действия:
1. Открыть последний failed run и посмотреть ROBOT LOG block — какой маркер (STT-empty / no_wake_word / tts-fallback / no-speech).
2. Если регрессия — поставить process-метку (\`needs-e2e\` или \`agent-flow-error\`) и привязать PR-фикс.
3. Если инфра-проблема — добавить в \`MAINTENANCE\` (kill-switch) на develop.

⚠️ При fail-streak > ${E2E_FAIL_STREAK_PAUSE} watchdog создаст pause-sentinel и ротация раундов будет заморожена до ручного override.
"

        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: gh issue comment ${_issue_num} (${_issue_title})"
        else
            if gh issue comment "$_issue_num" --repo "$GH_REPO" --body "$_body" >/dev/null 2>&1; then
                log "issue #${_issue_num}: alert posted (streak=${_streak})"
            else
                log "issue #${_issue_num}: WARNING comment failed (will retry next tick)"
            fi
        fi
    done < <(printf '%s' "$_target_issues" | python3 -c '
import json, sys
try:
    arr = json.load(sys.stdin)
    for it in arr:
        n = it.get("number", "")
        t = (it.get("title", "") or "")[:60]
        if n:
            print(f"{n}\t{t}")
except Exception:
    pass
' 2>/dev/null)
fi

# --- AUTO-CREATE ISSUE: idempotent + rate-limited (ADR-FS-001, t_401e52de) ---
# При fail-streak ≥ E2E_FAIL_STREAK_ISSUE_THRESHOLD создаём ОДИН issue с
# лейблом `e2e-fail-streak`. Два независимых guard'а:
#   (a) Rate-limit: ISSUE_COOLDOWN_FILE (mtime) старше RATE_LIMIT_HOURS
#       (если файл существует и свежий — skip).
#   (b) GitHub-truth: если уже есть ОТКРЫТЫЙ issue с лейблом
#       `e2e-fail-streak` — skip (защита от дублей при потере state-файла).
# Issue body: timeline failed runs + develop HEAD + релевантные merged PR
# (issue-ссылки из commit messages) + hypothesis `music-fix regression`
# со ссылками на #2246/#2347.
if [ "${_streak:-0}" -ge "$E2E_FAIL_STREAK_ISSUE_THRESHOLD" ] 2>/dev/null; then
    _cooldown_ok="true"
    if [ -f "$ISSUE_COOLDOWN_FILE" ]; then
        _cooldown_epoch="$(stat -c '%Y' "$ISSUE_COOLDOWN_FILE" 2>/dev/null || echo 0)"
        _cooldown_age_s=$(( $(date -u +%s) - ${_cooldown_epoch:-0} ))
        _cooldown_limit_s=$(( E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS * 3600 ))
        if [ "${_cooldown_age_s:-0}" -lt "${_cooldown_limit_s}" ]; then
            log "ISSUE_COOLDOWN active: ${_cooldown_age_s}s < ${_cooldown_limit_s}s — skip create"
            _cooldown_ok="false"
        fi
    fi

    _existing_e2e_issues="$(gh issue list --repo "$GH_REPO" --state open \
        --label "$E2E_FAIL_STREAK_ISSUE_LABEL" --limit 1 --json number 2>/dev/null \
        | python3 -c 'import json,sys; a=json.load(sys.stdin); print(len(a))' 2>/dev/null || echo 0)"
    if [ "${_existing_e2e_issues:-0}" -gt 0 ] 2>/dev/null; then
        log "open ${E2E_FAIL_STREAK_ISSUE_LABEL} issues: ${_existing_e2e_issues} — skip create"
        _cooldown_ok="false"
    fi

    if [ "$_cooldown_ok" = "true" ]; then
        # Собрать develop HEAD + релевантные merged PR за последние 5 дней
        _develop_head="$(git -C "${REPO_DIR:-$HERMES_HOME}" rev-parse --short=7 origin/develop 2>/dev/null \
            || git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)"
        # Issue-refs из последних 10 merge-коммитов develop (заголовок PR содержит "(#NNNN)")
        _related_prs="$(git -C "${REPO_DIR:-$HERMES_HOME}" log origin/develop \
            --merges --since='5 days ago' --pretty=format:'%s' 2>/dev/null \
            | grep -oE '#[0-9]+' | sort -u | tr '\n' ' ' | head -c 400 || echo "")"
        _failed_table="$(format_failed_runs_table "$_runs_json" 8 "$GH_REPO")"

        _create_body="🤖 [agent:devops] script=agent-flow-e2e-fail-streak-watchdog action=auto-create-issue

## fail-streak alert

L: E2E Voice Test (\`${E2E_WORKFLOW}\`) упал **${_streak}** раз подряд.
- **Last success:** ${_last_success_at:-NONE}
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
Релевантные merged PR за последние 5 дней: ${_related_prs:-_(none parsed)_}.

## Что делать

1. Открыть последний failed run → ROBOT LOG block → какой маркер (STT-empty / no_wake_word / tts-fallback / no-speech).
2. Если hypothesis подтвердилась — поставить процессные метки (\`needs-e2e\`/\`agent-flow-error\`) и привязать PR-фикс.
3. Если инфра-проблема — добавить \`MAINTENANCE\` на develop.
4. После закрытия fail-streak (новый SUCCESS) → удалить этот issue (\`gh issue close N --reason 'completed'\`).

> Скрипт-страж: \`scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh\`.
> При streak ≥ ${E2E_FAIL_STREAK_PAUSE} дополнительно создаётся pause-sentinel → e2e ротация замораживается.
"

        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: gh issue create --label ${E2E_FAIL_STREAK_ISSUE_LABEL} (streak=${_streak}, develop=${_develop_head})"
        else
            _create_args=(--repo "$GH_REPO" --title "[e2e-fail-streak] L: E2E Voice Test — ${_streak} fails подряд (develop ${_develop_head})" --label "$E2E_FAIL_STREAK_ISSUE_LABEL" --body "$_create_body")
            if [ -n "${E2E_FAIL_STREAK_ISSUE_ASSIGNEE:-}" ]; then
                _create_args+=(--assignee "$E2E_FAIL_STREAK_ISSUE_ASSIGNEE")
            fi
            _create_out=""
            _create_rc=0
            _create_out="$(gh issue create "${_create_args[@]}" 2>&1)" || _create_rc=$?
            if [ "${_create_rc}" = "0" ]; then
                _issue_url="$(printf '%s' "$_create_out" | grep -oE 'https://github.com/[^ ]+/issues/[0-9]+' | head -n 1 || true)"
                log "🚨 AUTO-CREATED fail-streak issue: ${_issue_url:-${_create_out}}"
                # Записать cooldown (epoch) — следующие 4ч не создавать ещё
                mkdir -p "$(dirname "$ISSUE_COOLDOWN_FILE")" 2>/dev/null || true
                date -u +%s > "$ISSUE_COOLDOWN_FILE" 2>/dev/null \
                    && log "cooldown written: $ISSUE_COOLDOWN_FILE" \
                    || log "WARN: cannot write cooldown file $ISSUE_COOLDOWN_FILE"
            else
                log "ERROR: gh issue create failed (rc=${_create_rc}): ${_create_out}"
            fi
            unset _create_rc
        fi
    fi
fi

# --- PAUSE: sentinel file (manual override only) ---------------------------
if [ "${_streak:-0}" -ge "$E2E_FAIL_STREAK_PAUSE" ] 2>/dev/null; then
    mkdir -p "$(dirname "$PAUSE_SENTINEL")" 2>/dev/null || true
    if [ -f "$PAUSE_SENTINEL" ]; then
        log "pause-sentinel already exists: ${PAUSE_SENTINEL} — no-op (manual override required to resume)"
    else
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: touch ${PAUSE_SENTINEL}"
        else
            cat > "$PAUSE_SENTINEL" <<EOF
# Auto-pause: L: E2E Voice Test fail-streak=${_streak} (>${E2E_FAIL_STREAK_PAUSE})
# Triggered: $(date -u +%Y-%m-%dT%H:%M:%SZ)
# Last success: ${_last_success_at:-NONE}
#
# Этот файл замораживает auto-rotation раундов в agent-flow-e2e-process.sh
# (см. _e2e_fail_streak_pause_check). Удалить ВРУЧНУЮ когда:
#   - регрессия пофикшена И свежий run прошёл SUCCESS, ИЛИ
#   - Шиф дал override (reason: «это нормальный fail-streak, продолжаем»).
#
# Не удалять «чтобы посмотреть что будет» — следующие 20+ FAIL'ов сожгут CI.
EOF
            if [ -f "$PAUSE_SENTINEL" ]; then
                log "🚨 PAUSE-SENTINEL CREATED: ${PAUSE_SENTINEL} (streak=${_streak})"
                log "   e2e-process auto-rotation ЗАМОРОЖЕНА до ручного override."
            else
                log "ERROR: cannot create pause-sentinel ${PAUSE_SENTINEL}"
                exit 1
            fi
        fi
    fi
fi

log "tick done: streak=${_streak} action=${_streak_action:-none}"
exit 0
