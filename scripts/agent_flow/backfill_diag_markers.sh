#!/bin/bash
# ============================================================================
# backfill_diag_markers.sh
#
# Ретро 31.08 t_9d375e3e / ADR-0035: backfill `<!-- diag-* -->` маркеров в
# существующие diagnostic-карточки (PR #1743, retro t_e00f448d).
#
# Бэкграунд: stale-after-upstream-fix detector (см.
# agent-flow-merge-gate.sh → stale_after_upstream_fix_scan_all) парсит
# маркеры из body карточки. До того, как UNSTABLE-блок основного цикла
# merge-gate начал писать маркеры (это будет после merge PR #1743 →
# ADR-0035 §5.4 Этап 1), старые diagnostic-карточки маркеров не имеют
# и detector их skip'ает с логом "no diag-pr marker, skip (legacy)".
#
# Backfill добавляет маркеры в существующие карточки ОДНОКРАТНО
# (idempotent — повторный запуск не дублирует comment).
#
# Что делает:
#   1. `hermes kanban --board X list --json` — все non-terminal карточки.
#   2. Фильтр: title начинается с "🐛 CI UNSTABLE DIAGNOSTIC #..." или
#      "🔀 rebase PR #..." (signature PR #1743 / retro t_e00f448d).
#   3. Для каждой карточки: hermes kanban show --json → проверить наличие
#      <!-- diag-pr: ... -->. Если есть — skip (idempotent).
#   4. Если нет — извлечь PR-номер из title (regex 'PR #([0-9]+)'),
#      получить head SHA через gh pr view, и запостить comment с маркерами
#      через `hermes kanban comment`.
#
# Маркеры, которые пишутся:
#   <!-- diag-pr: <N> -->
#   <!-- diag-pr-sha: <sha> -->
#   <!-- diag-pr-base: develop -->
#   <!-- diag-sig:  -->            (backfill не парсит failing-tests;
#                                   detector skip с warning log)
#   <!-- diag-tests:  -->
#   <!-- diag-classification: unit_lint -->
#   <!-- diag-created-ts: <now epoch> -->
#
# Использование:
#   bash scripts/agent_flow/backfill_diag_markers.sh         # реальный run
#   DRY_RUN=true bash scripts/agent_flow/backfill_diag_markers.sh
#   KANBAN_BOARD=robbox bash scripts/agent_flow/backfill_diag_markers.sh
#
# Source-of-truth: <repo>/scripts/agent_flow/backfill_diag_markers.sh
# Копии раскладываются install.sh (если потребуется) в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/backend/scripts/
# ============================================================================
set -uo pipefail

KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="${DRY_RUN:-false}"

log() { printf '[backfill-diag] %s %s\n' "$(date -Iseconds 2>/dev/null || date)" "$*" >&2; }

# 1. Список кандидатов (diagnostic / rebase).
diag_cards="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get("title", "") or ""
    status = t.get("status", "") or ""
    # Сигнатуры diagnostic-карточек (синхронизировано с
    # stale_after_upstream_fix_scan_all в agent-flow-merge-gate.sh):
    #   - LEGACY: "🐛 CI UNSTABLE: ..." (PR #1743 Этап 0, до маркеров).
    #     ВАЖНО для ретро t_beefef7a (02.09.2026): без этой строки
    #     backfill не подбирает карточки t_8f764875 / t_5c524b12
    #     (PR #1740/#1741 CLOSED Шифу, upstream-фикс PR #1748 уже в
    #     develop). С 02.09 они висят в todo 33ч+.
    #   - NEW: "🐛 CI UNSTABLE DIAGNOSTIC ..." (после PR #1743).
    #   - REBASE: "🔀 rebase PR #..." (reminder).
    is_diag = (title.startswith("🐛 CI UNSTABLE DIAGNOSTIC") or
               title.startswith("🐛 CI UNSTABLE:") or
               title.startswith("🔀 rebase PR #"))
    if is_diag and status not in ("done", "archived"):
        print(t.get("id", "") + chr(9) + title)
' 2>/dev/null || true)"

if [ -z "$diag_cards" ]; then
    log "no live diagnostic / rebase cards — nothing to backfill"
    exit 0
fi

# Счётчики для итогового лога.
total=0
already=0
posted=0
skipped=0
errors=0

while IFS=$'\t' read -r card_id card_title; do
    [ -z "$card_id" ] && continue
    total=$((total + 1))

    # 2. Достать body (для проверки наличия маркера diag-pr).
    body="$(hermes kanban --board "$KANBAN_BOARD" show "$card_id" --json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.loads(sys.stdin.read())
    print(data.get("body") or (data.get("task", {}) or {}).get("body", ""))
except Exception:
    pass
' 2>/dev/null || true)"

    if printf '%s' "$body" | grep -qE '<!-- diag-pr: [0-9]+ -->'; then
        already=$((already + 1))
        log "${card_id}: already has diag-pr marker, skip (idempotent)"
        continue
    fi

    # 3. Извлечь PR-номер из title ("... DIAGNOSTIC #1743 — branch").
    pr_num="$(printf '%s' "$card_title" | grep -oE '#([0-9]+)' | head -1 | tr -d '#')"
    if [ -z "$pr_num" ]; then
        skipped=$((skipped + 1))
        log "${card_id}: cannot extract PR number from title='${card_title}', skip"
        continue
    fi

    # 4. Достать head SHA и base branch через gh pr view.
    pr_view="$(gh pr view "$pr_num" --repo "$GH_REPO" --json headRefOid,baseRefName 2>/dev/null || echo '{}')"
    pr_sha="$(printf '%s' "$pr_view" | python3 -c '
import json, sys
try:
    d = json.loads(sys.stdin.read())
    print(d.get("headRefOid", "") or "")
except Exception:
    pass
' 2>/dev/null || true)"
    pr_base="$(printf '%s' "$pr_view" | python3 -c '
import json, sys
try:
    d = json.loads(sys.stdin.read())
    print((d.get("baseRefName", "") or "develop"))
except Exception:
    print("develop")
' 2>/dev/null || echo develop)"

    if [ -z "$pr_sha" ]; then
        # Если PR не существует / gh упал — пишем pr_num, но без sha.
        # Detector сможет работать с пустым sha (стратегия A/B/C skip,
        # REST compare fallback возьмёт sha из gh api).
        log "${card_id}: WARNING PR #${pr_num} headRefOid empty (gh pr view failed or PR closed)"
        pr_sha=""
    fi

    # 5. Сформировать маркер-блок (одна строка для совместимости с
    # mock_env.sh, который читает key=value построчно).
    now_ts="$(date +%s)"
    markers="<!-- diag-pr: ${pr_num} --> <!-- diag-pr-sha: ${pr_sha} --> <!-- diag-pr-base: ${pr_base} --> <!-- diag-sig:  --> <!-- diag-tests:  --> <!-- diag-classification: unit_lint --> <!-- diag-created-ts: ${now_ts} -->"

    # 6. DRY_RUN: только лог, без comment.
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: comment ${card_id} with markers: ${markers}"
        continue
    fi

    # 7. Постить comment с маркерами. Формат: Markdown-блок,
    # легко парсится и человеком, и detector'ом.
    comment_body="$(printf '## \xF0\x9F\x93\x9D backfill: stale-after-upstream-fix markers (ADR-0035)\n\n%s\n\n_This comment was added by `backfill_diag_markers.sh` (retro 31.08 t_9d375e3e / ADR-0035). The detector in `agent-flow-merge-gate.sh` uses them to auto-block this card as `stale-after-upstream-fix` once the upstream PR is merged / closed / fixed-in-PR. Marker format is hidden `<!-- ... -->` so it does not affect human readability of the card body._' "$markers")"

    if hermes kanban --board "$KANBAN_BOARD" comment "$card_id" "$comment_body" >/dev/null 2>&1; then
        posted=$((posted + 1))
        log "${card_id}: posted diag-pr marker comment (PR #${pr_num})"
    else
        errors=$((errors + 1))
        log "${card_id}: ERROR posting comment failed"
    fi
done < <(printf '%s\n' "$diag_cards")

log "summary: total=${total} already=${already} posted=${posted} skipped=${skipped} errors=${errors}"
exit 0