# AF-0066 — G10d guard: PR-orphan-after-issue-merged (anti-spam в CLOSED issues)

**Дата:** 2026-09-15
**Автор:** devops worker (ретро-карточка t_df2ae7ca)
**Контекст:** `scripts/agent_flow/agent-flow-merge-gate.sh`, секция guards (~L5466-L5471)
**Статус:** proposed
**Связанные тикеты:** t_df2ae7ca (эта карточка), t_40a610d0 (предыдущая блокировка по тому же инциденту)
**Связанные ретро:** t_50a18fa9 (G10a file-overlap), t_a8e82f2d (AF-0065 scan-all-prs race), t_1a4f3275 (pr_without_marker_scan)
**Связанные ADR:** AF-0062 (§2.3 G10c — merge-time file-overlap, sibling-guard), AF-0065 (spawn-спам защита)

## Контекст

`agent-flow-merge-gate.sh` (cron 5-min) содержит guard `pr_without_marker_scan_all`
(ретро 25.08 t_1a4f3275 / issue #1624), который сканирует **все** open PR с
процесс-меткой (agent-flow*/needs-e2e/needs-review) и пишет в issue «⚠️ process marker
missing on PR», если в issue нет kanban-marker.

**Симптом (инцидент 2026-09-14 22:14Z→23:34Z, ~80 мин, 12 спам-комментов):**

PR #2457 (`z-backend/t_ec75ba3e-tool-call-enforce-discovery`) был создан в 21:29Z
с процесс-меткой `needs-e2e`. Issue #2406 (к которому относился PR) был закрыт
через merge **другого** PR (#2458) в 21:48Z — всего 19 минут спустя. Когда issue
закрылся, kanban-карточка `t_40a610d0` оказалась stale (worker не мог открыть
новый PR — неёсёт чужой ADR-0095, 826 строк).

Guard `pr_without_marker_scan_all` начал писать в CLOSED issue #2406 каждые ~10 мин:
«⚠️ process marker missing on PR #2457» (итого 8+ одинаковых комментов подряд).
Закрытый issue не получит kanban-marker от живого процесса → guard в принципе
не мог решить проблему, только шуметь.

**Impact:**
- Шум в issue-таймлайне (8+ комментов подряд, периодичность ~10 мин)
- Лишняя нагрузка на agent-flow-merge-gate cron (REST+GraphQL round-trip каждые 10 мин)
- Signal/noise размывание: при реальных проблемах репорт теряется среди спама

## Решение

Добавляем новый guard **`g10d_pr_orphan_after_issue_merged_scan_all`** в
`agent-flow-merge-gate.sh` (вызывается ПОСЛЕ `pr_without_marker_scan_all`,
~L5470). Guard:

1. Берёт **все** open PR с процесс-меткой.
2. Извлекает issue_number (из `title #NNNN` ИЛИ из `branch z-{agent}/NNNN-...`.
3. Запрашивает state issue через REST API.
4. Если `state == "CLOSED"`:
   - Снимает процесс-метки с PR (`needs-e2e`, `needs-review`, `agent-flow`,
     `agent-flow-error`) → merge-gate перестаёт его сканировать.
   - Пишет **ОДИН раз** idempotency-комментарий в issue (marker
     `<!-- merge-gate-g10d-skip: <PR> -->`, dedup через `comment_recently_posted`,
     TTL=30 дней).
   - Пишет **ОДИН раз** комментарий в PR (cross-link на issue + контекст для Шифу).
5. Иначе — silent skip.

**Почему после `pr_without_marker_scan_all`, а не до:** guard-batching —
на этом тике `pr_without_marker_scan_all` отработает ОДИН последний раз
(issue ещё CLOSED, kanban-marker нет), спам остановится, и в следующий тик
PR уже без процесс-метки → ни `pr_without_marker_scan_all`, ни `g10d`
его не увидят. Альтернатива (вызвать g10d ДО) сэкономит один спам-коммент,
но усложняет порядок — следующий инженер не поймёт почему g10d перехватывает.

## Отличие от sibling-guards

| Guard | Когда ловит | Что делает |
|---|---|---|
| **G10a** (`file_overlap_with_open_pr` в triage) | Pre-create: 2 worker'а стартанули фикс одного defect | Skip создания kanban-карточки |
| **G10b** (`existing_active_card_for_issue` в triage) | Pre-create: для issue уже есть активная карточка | Skip создания дубль-карточки |
| **G10c** (`competing_prs_block_scan_all` в merge-gate, ADR-AF-0062 §2.3) | Merge-time: 2 PR правят один файл в пересекающихся строках | Block оба PR через label, Шифу выбирает canonical |
| **G10d** (этот ADR) | **Post-merge**: issue закрыт, PR с процесс-меткой остался открытым | Remove process-labels, ОДИН idempotency-коммент |

## Acceptance

- [x] После влития G10d `merge-gate` НЕ триггерит комментов в CLOSED issue от
      `pr_without_marker_scan_all` (метки сняты → guard не видит PR).
- [x] Каждый такой комментарий пишется **ОДИН раз** (idempotency через marker
      `<!-- merge-gate-g10d-skip: $PR -->`).
- [x] Тривиальная ручная чистка `gh pr close <PR> --comment 'stale after issue merged'`
      остаётся опцией для Шифу — guard автоматический, не блокирующий.

## Backward-compat

- `PR_ORPHAN_AFTER_ISSUE_MERGED_GUARD=false` — отключить guard (например, на
  maintenance-окне или при ручной чистке спама).
- Guard fail-OPEN: gh-ошибки → warning-лог, PR не трогаем.
- Идемпотентность по маркеру — повторные тики skip через dedup.

## Файлы

- `scripts/agent_flow/agent-flow-merge-gate.sh` — определение
  `g10d_pr_orphan_after_issue_merged_scan_all` (L1311-L1411) + вызов в guards-секции
  (L5466-L5471).
- `scripts/agent_flow/test_g10d.sh` — 5 unit-тестов с mock-gh (closed-issue-triggers,
  open-issue-skip, no-process-label-skip, dry-run-no-side-effects, branch-only-issue-num).
- ADR-AF-0062 §2.3 G10c — **sibling**, не часть этой карточки. G10d имя занято
  потому что G10c уже зарезервирован в AF-0062 для merge-time file-overlap.
