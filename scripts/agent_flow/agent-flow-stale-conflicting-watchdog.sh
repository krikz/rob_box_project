#!/bin/bash
# ============================================================================
# agent-flow-stale-conflicting-watchdog.sh — auto-detect stale CONFLICTING PRs
# (open, mergeable=false, mergeable_state=dirty) без активной карточки на
# rebase. Долго висят CONFLICTING без владельца → блокируют merge-gate,
# видны в daily-report как CONFLICTING без видимого владельца.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-stale-conflicting-watchdog.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/  (legacy)
#
# Контекст / ретро t_a7d642cd (2026-09-16, wip-conflict-wave-after-cc-budget):
#   5 PR в hang'ело CONFLICTING 12-18 часов после волны merge PR
#   #2633/#2638/#2641/#2643 в develop (особенно `refactor(voice):
#   DialogueNode._on_stt → SttAdmission` 6eee3c5a):
#   - #2647 (z-architect/2627-adr-post-turn-music-finalizer, docs/adr)
#   - #2644 (z-{agent}/2609-fix-voice-torch-2-14-0-cu130-..., src+docker)
#   - #2642 (z-{agent}/2631-refactor-voice-dialoguenode-run-turn-..., + needs-e2e)
#   - #2640 (z-{agent}/2627-refactor-voice-dialoguenode-run-turn-cc-59-4-...)
#   - #2639 (z-{agent}/2630-refactor-harness-agentcore-run-with-tools-...)
#
#   Root cause: wip-стратегия воркеров не справляется с быстрым develop.
#   Воркер делает wip-ветку утром от develop, к вечеру в develop вливаются
#   5+ PR включая структурный рефакторинг → wip-коммиты остаются в ветке,
#   PR создаётся, НО rebase перед push не делается. PR висит CONFLICTING
#   без активной running/todo карточки на rebase.
#
#   Паттерн системный: уже был t_c6850330 (e2e-process GraphQL race), но
#   для CC-budget/large refactor волн не формализован.
#
# Решение (этот скрипт): reactive sweep КАК FALLBACK на случай зависшего
# worker. Каждый час сканирует OPEN PR с mergeable=false & mergeable_state=dirty,
# для каждого проверяет:
#   1. updatedAt > STALE_THRESHOLD (default 4h) — отсекает свежесозданные
#   2. нет ли АКТИВНОЙ running/todo карточки-владельца по этому PR
#      (kanban cards с PR#N в body + status='running' or 'todo')
#   3. если нет — emit rebase-рекомендацию через kanban-retro-create.sh
#      с ключом `rebase-pr-<N>` (idempotency через marker).
#
# Контракт (per tick):
#   1. flock lock (не два тика одновременно)
#   2. gh pr list --state open --json number,mergeable,mergeableState,headRefName,baseRefName,updatedAt,title
#   3. Для каждого PR с mergeableState='dirty' & updated_at < now-STALE_THRESHOLD:
#        a) ищем активную карточку в kanban (sqlite scan по body LIKE '%PR #N%'
#           И status in (running, todo)) — если есть, SKIP (worker уже
#           взялся).
#        b) иначе — вызываем kanban-retro-create.sh с title
#           "rebase PR #N (CONFLICTING <hours>h)" и body, описывающим
#           контекст (head/base/updated_at/why-conflicting).
#   4. emit НЕ merge'ит, НЕ rebase'ит, НЕ comment'ит в issue — только
#      rebase-рекомендация, assignee=devops (worktree wt-origin-autofix
#      может выполнить git rebase origin/develop).
#   5. Log stats: scanned, stale, has_card (skipped), recommended (created),
#      errors.
#
# ENV:
#   GH_REPO                — owner/repo (default krikz/rob_box_project)
#   DRY_RUN                — true → log only, no card create
#   STALE_THRESHOLD_HOURS  — default 4 (только PR старше N часов)
#   WATCHDOG_BOARD         — kanban board для поиска existing cards
#                            (default robbox)
#   KANBAN_DB_PATH         — direct path to kanban.db (default
#                            /home/builder/.hermes/kanban/boards/robbox/kanban.db)
#   LOCK_FILE              — flock guard (default
#                            /tmp/agent-flow-stale-conflicting-watchdog.lock)
#   KANBAN_RETRO_CREATE_SH — path to kanban-retro-create.sh wrapper (default
#                            /home/builder/.hermes/scripts/kanban-retro-create.sh,
#                            fallback to <repo>/scripts/agent_flow/kanban-retro-create.sh)
#   LOG_FILE               — stats log (default
#                            /tmp/agent-flow-stale-conflicting-watchdog.log)
#
# Выходы:
#   - Stderr: structured summary (для cron delivery).
#   - Exit 0 — всё ok (даже если ничего не emit'или).
#   - Exit 1 — критичный сбой (нет gh auth, нет python3, нет sqlite3, нет
#              kanban-retro-create.sh).
#   - Exit 2 — emit'или ≥1 rebase-рекомендацию. Alert для cron.
#
# Что НЕ делаем (явно):
#   - НЕ rebase'им PR сами (это решение assignee карточки, у него есть
#     контекст «что в wip-коммитах»).
#   - НЕ merge'им, НЕ close'им PR (там могут быть wip-коммиты, которые
#     надо сохранить как draft / salvage).
#   - НЕ auto-add label на PR (это политика merge-gate).
#   - НЕ удаляем существующие карточки если они потеряли актуальность.
#
# Pitfalls (gotchas):
#   - gh возвращает `mergeable=null` пока GitHub не посчитал mergeability
#     (cold cache); через 30-60s после open он становится true/false. Мы
#     пропускаем null (не считаем ни stale, ни свежим — пусть следующий
#     тик проверит). Это снижает false-positive шум.
#   - mergeable_state может быть 'behind' (просто устарел, не dirty).
#     behind ≠ CONFLICTING. Реально CONFLICTING = 'dirty'. behind тоже
#     требует rebase, НО без активной карточки — оставляем пока в covered
#     by other watchdogs (agent-flow-e2e-wt-sweep round-rotation handles
#     branches via wt-origin-autofix). Поэтому фильтруем ТОЛЬКО dirty.
#   - sqlite3 может быть залочен транзакцией (kanban-агент пишет). Берём
#     через PRAGMA busy_timeout=2s + read-committed isolation. Если unlock
#     не получился — fail-open (этот тик SKIP, не emit).
#   - kanban-retro-create.sh идёт через `hermes kanban create`, что требует
#     интерактивного контекста. На хосте это нормально (CLI), но в CI
#     cron-job нужно PATH-hijack или hermes на $PATH.
#   - idempotency-key kanban-retro-create: `retro:rebase-pr-<N>` →
#     повторный тик (та же PR, тот же ключ) → SKIP. Отдельно хранится
#     маркер `ретро-key: rebase-pr-<N>` в body карточки для pre-check.
#   - OWNER-CONFLICT-PULL-POLICY: «уже есть активная карточка на rebase»
#     определяется как task со status ∈ {running, todo} (НЕ blocked/ready/
#     review/done). Blocked карточка — другая семантика (prereq-merged),
#     она не «активный rebase worker».
# ============================================================================
set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="${DRY_RUN:-false}"
STALE_THRESHOLD_HOURS="${STALE_THRESHOLD_HOURS:-4}"
KANBAN_DB_PATH="${KANBAN_DB_PATH:-/home/builder/.hermes/kanban/boards/robbox/kanban.db}"
KANBAN_RETRO_CREATE_SH="${KANBAN_RETRO_CREATE_SH:-}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-stale-conflicting-watchdog.lock}"
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-stale-conflicting-watchdog.log}"
WATCHDOG_BOARD="${WATCHDOG_BOARD:-robbox}"

_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

# --- flock guard (avoid race with merge-gate / other watchdogs) -------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(_now_iso)] stale-conflicting-watchdog: another instance running — skip" >&2
    exit 0
fi

# --- preflight: gh auth, python3, sqlite3, kanban-retro-create.sh ------------
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(_now_iso)] stale-conflicting-watchdog: gh auth failed — exit 1" >&2
    exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
    echo "[$(_now_iso)] stale-conflicting-watchdog: python3 missing — exit 1" >&2
    exit 1
fi

# Resolve kanban-retro-create.sh (host copy preferred, repo fallback)
if [ -z "$KANBAN_RETRO_CREATE_SH" ]; then
    if [ -x "/home/builder/.hermes/scripts/kanban-retro-create.sh" ]; then
        KANBAN_RETRO_CREATE_SH="/home/builder/.hermes/scripts/kanban-retro-create.sh"
    elif [ -x "$(dirname "$0")/kanban-retro-create.sh" ]; then
        KANBAN_RETRO_CREATE_SH="$(dirname "$0")/kanban-retro-create.sh"
    else
        echo "[$(_now_iso)] stale-conflicting-watchdog: kanban-retro-create.sh not found — exit 1" >&2
        exit 1
    fi
fi
[ -x "$KANBAN_RETRO_CREATE_SH" ] || {
    echo "[$(_now_iso)] stale-conflicting-watchdog: $KANBAN_RETRO_CREATE_SH not executable — exit 1" >&2
    exit 1
}

# --- get open PRs -----------------------------------------------------------
_prs_json="$(gh pr list --repo "$GH_REPO" --state open \
    --json number,mergeable,mergeableState,headRefName,baseRefName,updatedAt,title \
    --limit 50 2>/dev/null || echo '[]')"

_threshold_epoch=$(( $(date -u +%s) - STALE_THRESHOLD_HOURS * 3600 ))

# --- main loop: filter via python3 (to a tmp file), then iterate -----------
# Пишем JSONL в tmp-файл (на каждую строку — pipe-delimited поля).
# Используем `python3 -c` со встроенным скриптом: stdin heredoc ОТДЕЛЬНО
# от pipe (`printf | python3`) — bash разрешает только один stdin source.
# Передаём argv[1]=threshold_epoch, argv[2]=tmp-file для вывода,
# argv[3]=json-файл с _prs_json (без pipe).
_SCAN_TMP="$(mktemp -t cfwatch.XXXXXX)"
_INPUT_JSON_TMP="$(mktemp -t cfwatch.XXXXXX)"
trap 'rm -f "$_SCAN_TMP" "$_INPUT_JSON_TMP"' EXIT
printf '%s' "$_prs_json" > "$_INPUT_JSON_TMP"

python3 - "$_threshold_epoch" "$_SCAN_TMP" "$_INPUT_JSON_TMP" 2>/dev/null <<'PYEOF'
import json, sys
threshold = int(sys.argv[1])
out_path = sys.argv[2]
in_path = sys.argv[3]
out = open(out_path, "w")
try:
    with open(in_path, "r") as fh:
        data = json.loads(fh.read())
except Exception:
    sys.exit(0)
for pr in data:
    if not isinstance(pr, dict):
        continue
    n = pr.get("number")
    upd = (pr.get("updatedAt") or "").strip()
    state = (pr.get("mergeableState") or "").strip()
    head = pr.get("headRefName") or ""
    base = pr.get("baseRefName") or ""
    title = (pr.get("title") or "").strip()
    # mergeableState='dirty' — единственный наш целевой сигнал;
    # null (cold cache) и 'behind' пропускаем.
    if n is None or not upd or state != "dirty":
        continue
    # Title sanitization: pipe ('|') — наш field delimiter; заменяем на '/'.
    safe_title = title.replace("|", "/").replace("\n", " ")
    # Head/Base могут содержать '/', но не '|'.
    safe_head = head.replace("|", "/")
    safe_base = base.replace("|", "/")
    print(f"{n}|{upd}|{safe_head}|{safe_base}|{safe_title}", file=out)
out.close()
PYEOF

_scan_total=0
_stale_total=0
_has_card_total=0
_recommended_total=0
_errors_total=0
_records=()

# Сначала посчитаем сколько PR вернул `gh pr list` (raw, без фильтров).
# `scanned` = total-from-gh, `stale` = subset с mergeableState=dirty AND old.
_scan_total="$(python3 - "$_INPUT_JSON_TMP" <<'PYEOF' 2>/dev/null
import json, sys
try:
    with open(sys.argv[1]) as fh:
        data = json.load(fh)
except Exception:
    print(0)
    sys.exit(0)
if not isinstance(data, list):
    print(0)
else:
    print(len([p for p in data if isinstance(p, dict)]))
PYEOF
)"
_scan_total="${_scan_total:-0}"

while IFS= read -r _line; do
    [ -n "$_line" ] || continue

    # pipe-delimited fields: number|updated|head|base|title
    pr_number="$(printf '%s' "$_line" | awk -F'|' '{print $1}')"
    pr_updated="$(printf '%s' "$_line" | awk -F'|' '{print $2}')"
    pr_head="$(printf '%s' "$_line" | awk -F'|' '{print $3}')"
    pr_base="$(printf '%s' "$_line" | awk -F'|' '{print $4}')"
    pr_title="$(printf '%s' "$_line" | awk -F'|' '{for(i=5;i<=NF;i++){printf "%s",$i;(i<NF?OFS:ORS)}}')"

    # stale-check (повторяем здесь после парсинга).
    pr_updated_epoch=$(date -u -d "$pr_updated" +%s 2>/dev/null || echo 0)
    if [ "$pr_updated_epoch" -le 0 ] || [ "$pr_updated_epoch" -gt "$_threshold_epoch" ]; then
        continue
    fi
    _stale_total=$(( _stale_total + 1 ))

    # Проверяем, есть ли активная running/todo карточка на rebase для этого PR.
    # Берём по KANBAN_DB_PATH через python3 sqlite3 (нет зависимости от
    # `sqlite3` CLI — в host-image python3 всегда есть). Сканируем body
    # LIKE '%PR #N%' И status in (running, todo). НЕ считаем blocked
    # (другая семантика).
    if [ ! -f "$KANBAN_DB_PATH" ]; then
        # kanban DB недоступна — fail-open: emit'им карточку (под dedup-key
        # kanban-retro-create идемпотентность).
        echo "[$(_now_iso)] stale-conflicting-watchdog: WARN kanban DB not found at $KANBAN_DB_PATH — emitting anyway" >&2
    else
        # NB: sqlite_open может залипнуть на locked DB; PRAGMA busy_timeout=2s
        # даёт шанс на unlock без навсегда-зависшего запроса. Timeout после —
        # fail-open (ничего не emit'им, но и не падаем).
        _has_card="$(KANBAN_DB_PATH="$KANBAN_DB_PATH" pr_number="$pr_number" python3 <<'PYEOF' 2>/dev/null || true
import sqlite3, os
try:
    con = sqlite3.connect(os.environ["KANBAN_DB_PATH"], timeout=2.0)
    cur = con.execute(
        "SELECT id FROM tasks WHERE status IN ('running','todo') "
        "AND (body LIKE '%PR #" + str(os.environ["pr_number"]) + "%' "
        "     OR body LIKE '%#" + str(os.environ["pr_number"]) + "%') "
        "AND id NOT LIKE 'archived-%' LIMIT 1"
    )
    row = cur.fetchone()
    print(row[0] if row else "")
    con.close()
except Exception:
    pass
PYEOF
)"
        if [ -n "$_has_card" ]; then
            _has_card_total=$(( _has_card_total + 1 ))
            echo "[$(_now_iso)] stale-conflicting-watchdog: SKIP #${pr_number} (active card ${_has_card} on rebase)" >&2
            continue
        fi
    fi

    # Build body — описание PR + rebase-команды + marker ретро-key.
    _age_hours=$(( ( $(date -u +%s) - pr_updated_epoch ) / 3600 ))
    _body="**PR #${pr_number} stale-CONFLICTING ${_age_hours}h** (ретро t_a7d642cd, wip-conflict-wave-after-cc-budget).

PR уже mergeable=false & mergeable_state=dirty. Разработчик его не rebase'ил после волны merge в develop (особенно после \`refactor(voice): DialogueNode._on_stt → SttAdmission\`, ADR-0021 R1, commit 6eee3c5a, PR #2638).

| Поле | Значение |
|---|---|
| PR | #${pr_number} |
| Title | ${pr_title} |
| Head | \`${pr_head}\` |
| Base | \`${pr_base}\` |
| Updated | ${pr_updated} (${_age_hours}h ago, threshold ${STALE_THRESHOLD_HOURS}h) |
| Conflicting since | ${pr_updated} |

## Что делать (assignee=devops)

\`\`\`bash
# Worktree от origin/develop
git fetch --no-tags origin refs/heads/develop:refs/remotes/origin/develop
git worktree add /tmp/rebase-pr-${pr_number} -b rebase-pr-${pr_number} origin/develop
cd /tmp/rebase-pr-${pr_number}
# Подтянуть ветку PR (auto-detect через gh pr view)
PR_BRANCH=\$(gh pr view ${pr_number} --repo ${GH_REPO} --json headRefName -q .headRefName)
git fetch --no-tags origin "\${PR_BRANCH}:\${PR_BRANCH}" 2>/dev/null || true
git checkout "${pr_head}"
git rebase origin/develop
# Resolve conflicts, run tests, force-push
git push --force-with-lease origin HEAD:"${pr_head}"
\`\`\`

ВАЖНО:
- Не merge'ить в develop.
- Не удалять wip-коммиты (там могут быть valuable refactor steps).
- После successful rebase PR станет mergeable → merge-gate сам подхватит.

---

Touchpoints: ADR-0018 (process honesty), ADR-0045 (worker worktree base ref), #2626, #2627, #2630, #2631, develop tip 8c8b60135.

Авто-создано agent-flow-stale-conflicting-watchdog (wip-conflict-wave-after-cc-budget)."

    if [ "$DRY_RUN" = "true" ]; then
        echo "[$(_now_iso)] stale-conflicting-watchdog: [DRY-RUN] would-recommend PR #${pr_number} age=${_age_hours}h" >&2
        _recommended_total=$(( _recommended_total + 1 ))
        continue
    fi

    # emit через kanban-retro-create.sh — дедуп через
    # --idempotency-key=retro:rebase-pr-<N>.
    _emit_out="$(bash "$KANBAN_RETRO_CREATE_SH" \
        --title "rebase PR #${pr_number} (CONFLICTING ${_age_hours}h, t_a7d642cd)" \
        --body "$_body" \
        --assignee devops \
        --key "rebase-pr-${pr_number}" \
        --board "$WATCHDOG_BOARD" \
        --max-runtime 1800 \
        2>&1)" || {
            _errors_total=$(( _errors_total + 1 ))
            echo "[$(_now_iso)] stale-conflicting-watchdog: ERROR kanban-retro-create for PR #${pr_number}: $_emit_out" >&2
            continue
        }

    _recommended_total=$(( _recommended_total + 1 ))
    _records+=("$(printf '%s\tPR #%s\thead=%s\tbase=%s\tage=%sh\t%s' \
        "$(_now_iso)" "$pr_number" "$pr_head" "$pr_base" "$_age_hours" "$_emit_out")")
    echo "[$(_now_iso)] stale-conflicting-watchdog: RECOMMEND PR #${pr_number} age=${_age_hours}h → $_emit_out" >&2
done < "$_SCAN_TMP"

# --- summary ---------------------------------------------------------------
echo "[$(_now_iso)] stale-conflicting-watchdog: ✓ done scanned=${_scan_total} stale=${_stale_total} has_card=${_has_card_total} recommended=${_recommended_total} errors=${_errors_total} repo=${GH_REPO}" >&2

# --- write stats log -------------------------------------------------------
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# stale-conflicting-watchdog snapshot %s\n' "$(_now_iso)"
    printf 'timestamp\tPR\thead\tbase\tage\tdetail\n'
    for r in "${_records[@]:-}"; do
        [ -n "$r" ] && printf '%s\n' "$r"
    done
    printf '# scanned=%s stale=%s has_card=%s recommended=%s errors=%s repo=%s dry_run=%s threshold=%sh\n' \
        "$_scan_total" "$_stale_total" "$_has_card_total" \
        "$_recommended_total" "$_errors_total" "$GH_REPO" "$DRY_RUN" "$STALE_THRESHOLD_HOURS"
} >> "$LOG_FILE" 2>/dev/null || true

# --- exit code -------------------------------------------------------------
# exit 2 если emit'или хоть одну (alert для cron), exit 0 если ничего
# не emit'или (норма), exit 1 если критичный сбой (выше).
if [ "$_recommended_total" -gt 0 ] && [ "$DRY_RUN" != "true" ]; then
    exit 2
fi
exit 0
