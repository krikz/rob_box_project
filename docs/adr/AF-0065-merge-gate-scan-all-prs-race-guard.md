# AF-0065 — merge-gate scan-all-prs race-guard против spawn-спама

**Дата:** 2026-09-14
**Автор:** devops worker (ретро-карточка t_a8e82f2d)
**Контекст:** `agent-flow-merge-gate.sh`, scan-all-prs блок (~L5480-L5630)
**Статус:** proposed
**Связанные тикеты:** t_a8e82f2d, t_de83f868, t_61389e9e, t_2059fd6d, t_ff8f2c39, t_88b621d2, t_879a3a06, t_637e9484, t_7779658f
**Связанные ретро:** t_42741511 (13.08, done-match), t_8af6bf29 (12.08, rate-limit), t_618208c0 (12.08, серая зона CONFLICTING)

## Контекст

`agent-flow-merge-gate.sh` (cron 5-min) — основной watchdog, сканирующий
**все** open PR и автоматически создающий rebase/recovery карточки для
PR в состоянии `mergeable=CONFLICTING` или `mergeStateStatus=DIRTY`.
Также работает через основной цикл main-cycle (L4300-L4430).

Симптом (тик 2026-09-14 12:10Z): на каждый CONFLICTING PR крон
создавал **3-5 rebase/recovery карточек в разных статусах** (todo / done /
blocked / triage), все с разными assignee. Результат — спам в очереди,
воркеры не видят новые тикеты за шумом, диспатчер тратит лимиты на
спам, и ни одна карточка реально не выполняет rebase.

**Пример спама для PR #2372:**

| task        | status   | assignee | создан | комментарий               |
| ----------- | -------- | -------- | ------ | ------------------------- |
| t_de83f868  | blocked  | devops   | 11:s   | активная (PR MERGEABLE)   |
| t_61389e9e  | todo     | default  | 11:5x  | спам, дубликат            |
| t_2059fd6d  | done     | devops   | 12:50  | спам, дубль после done    |

Аналогичная картина для PR #2366, #2367.

## Root cause

В `agent-flow-merge-gate.sh` scan-all-prs блок **4 места создания карточек**:

1. **L5521** — fresh recovery после `_done_match` (PR был MERGEABLE → стал
   CONFLICTING снова → воркер завершил старую карточку → делаем fresh).
2. **L5542** — fresh recovery когда карточки нет вообще (else ветка).
3. **L5605** — fresh conflict-card после `_existing_conflict` в done/archived.
4. **L5622** — fresh conflict-card когда конфликт-карточки нет вообще.

Каждое из этих мест **НЕ делает повторную проверку** между
`hermes kanban list --json` и `hermes kanban create`. Race-window:
cron-тик A делает list → пусто → переходит к create → cron-тик B (5-min
re-entry или параллельный merge-gate) делает list → пусто → тоже
переходит к create → оба создают дубль.

В коде есть обширный комментарий (L5529-5541) почему **нельзя просто
добавить `--idempotency-key`**:

```text
idempotency-key возвращает существующую карточку ДАЖЕ в done
(SELECT ... status != 'archived'), из-за чего recovery-карточка
после done НЕ пере-создавалась и PR висел CONFLICTING навсегда.
Старые карточки выше уже обработаны (active/blocked/done match) —
до else доходим только когда карточки НЕТ вообще. Поэтому create
БЕЗ idempotency-key: каждая свежая конфликтная ситуация получает
СВЕЖУЮ ready-карточку. Гонка (два merge-gate тика подряд) →
дубликат, но дубликат безопаснее deadlock.
```

Аргумент про deadlock валиден (после done нужна СВЕЖАЯ карточка).
Аргумент про дубль — **устаревший**: добавление race-recheck сразу
перед create устраняет дубль, не возвращаясь к deadlock.

## Решение

**Pre-create race-recheck** в 4 местах scan-all-prs:

```bash
# Перед fresh-create вызвать:
_recheck_matches="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get('title','')
    if '${head}' in title or 'rebase PR #${pr_num}' in title:
        print(t['id'], t.get('status',''))
" 2>/dev/null || true)"
_recheck_active="$(printf '%s\n' "$_recheck_matches" | awk '$2 ~ /^(running|ready|todo)$/ {print $1" "$2; exit}')"
if [ -n "$_recheck_active" ]; then
    log "scan-all-prs: race-recheck — active card already exists (${_recheck_active}) for PR #${pr_num}, skip fresh create"
else
    # original create block
fi
```

**Что изменилось:**
- Между `_branch_matches` (собранным в начале scan-all-prs) и `hermes
  kanban create` теперь ещё один fresh-list + python parse + awk check.
- Если за время обработки другой тик уже создал active карточку —
  пропускаем create (race resolved).
- Доп. стоимость: 1× `hermes kanban list --json` (~50-100ms) + python
  parse. Cron 5-min tolerates this.

**Что НЕ изменилось:**
- main-cycle (L4300-L4430) уже имеет dedup-coordinator
  (`_branch_matches` + active/blocked/done match) — НЕ трогаем.
- `_existing_conflict` ветка (L5587-L5620) уже имеет dedup-coordinator
  для первого случая (нет карточки) — добавили только pre-create
  recheck.
- `--idempotency-key` НЕ используем — идёмпотентность по-другому
  работает с done (см. root cause).

## Acceptance criteria

- [x] 4 места в scan-all-prs имеют `_recheck_matches` + `_recheck_active`
      блок непосредственно перед create.
- [x] `bash -n scripts/agent_flow/agent-flow-merge-gate.sh` exit 0.
- [x] Cleanup: stale дубликаты `t_61389e9e`, `t_2059fd6d` заархивированы
      (комментарий с причиной).
- [x] `t_de83f868` оставлена (активная работа, PR #2372 = MERGEABLE+CI pass).
- [ ] После merge в develop: следующий cron-тик на CONFLICTING PR НЕ
      создаёт >1 rebase/recovery карточки на одну (PR+branch).

## Что осталось за рамками

1. **assignee-skill validation в agent-flow-triage.sh** (ADR-0036 §4.1) —
   отдельный issue из body ретро-карточки. Не блокер для этого фикса.
2. **Race-window между cron-тиками < 5min** остаётся (между list и
   recheck окно ~50-100ms). Достаточно для большинства случаев, но не
   для двух **одновременных** тиков (что нетипично — merge-gate не
   запускается параллельно, только последовательно через cron).

## Pitfalls

- **hermes kanban list --json дорогой** (~50-100ms per call). 4 места
  × 1 recheck = 4× ~100ms = 400ms дополнительной нагрузки на cron.
  Tolerable.
- **python парсинг молча exit 1 на malformed JSON** — обёрнут в
  `try/except Exception: sys.exit(0)` чтобы recheck возвращал пусто,
  и fallback срабатывал на create.
- **Не удалять `_branch_matches` сборку выше** — она нужна для
  active/blocked/done match логики. Recheck только дополняет race-guard.

## Verification

Запустить в dry-run режиме:

```bash
DRY_RUN=true CONFLICTING_PR=2372 bash -x scripts/agent_flow/agent-flow-merge-gate.sh \
    2>&1 | grep -E 'race-recheck|fresh recovery card|fresh conflict card'
```

Ожидаемый вывод:
- 1× "fresh recovery card created" (первый тик)
- 1× "race-recheck — active card already exists" (второй тик с тем же PR)
