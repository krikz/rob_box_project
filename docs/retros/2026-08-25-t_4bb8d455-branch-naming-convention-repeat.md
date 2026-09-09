# Ретро 2026-08-25: PR/ветки НЕ по конвенции z-{agent}/ — повторный паттерн

**Автор:** architect (товарищ Шифу, надзор)
**Карточка:** t_4bb8d455
**Предыдущий ретро:** t_4515d767 (24.08 16:00 UTC, status=done) — первый кейс PR #1206, фикс не докатился
**Класс бага:** `branch-naming-convention-violation-repeat` (повторный, retro-key)

## TL;DR

Конвенция `z-{agent}/<issue>-<slug>` для воркерских PR **не зафиксирована ни в одном
скрипте-валидаторе**, и это второй раз за 30ч, когда PR на ветке `feature/av-...`
зависает без `needs-review`/`needs-e2e` потому, что `merge-gate` ищет канонический
`z-{agent}/<id>-<slug>`-pattern. Шифу создаёт issue через GSD workflow → воркер
толкает код на свою default feature-ветку (что согласуется с CONTRIBUTING.md §🌿
"Стратегия веток") → конвенция `z-{agent}/...` нарушается → пайплайн не видит PR.

**Корень:** `agent-flow-merge-gate.sh` имеет три отдельные логики, которые
должны были сойтись в одной точке, но **ни одна не валидирует имя ветки как
первоклассную сущность**. Это:
1. OPEN-PR guard в триаже (строки 643-647, ищет только каноническую `branch` через
   `branch_for()`).
2. `PR↔issue e2e-done drift reconcile` (merge-gate:1016-1056, ищет
   `_drift_branch_pattern="z-{agent}/${number}-"`).
3. `_followup_json` follow-up PR по `${number} in:title` (merge-gate:962) — совпадает
   только если в title PR есть номер issue (PR #1610 body содержит `[AV-4]`,
   `issue #1598`, но **title** — без номера issue).

PR #1610 title: `"[AV-4] LockManager teleop_floor/voice_floor + 500ms dead-man (TDD)"`.
`${number} in:title` → 1598 ✓. Но дальше `grep -F "z-{agent}/${number}-"` отсекает
потому что PR-head=`feature/av-4-supervisor-lock-manager` ≠ `z-{agent}/1598-av-4-...`.

## Симптомы (новые наблюдения 25.08 07:35 UTC)

### Случай 1 — PR #1610 (AV-4 / issue #1598)

| Поле | Значение |
|------|----------|
| `headRefName` | `feature/av-4-supervisor-lock-manager` |
| `baseRefName` | `feature/avatar` ← AV-* эпик-ветка |
| `state` | open, mergeable=true, mergeable_state=clean |
| `created_at` | 2026-08-25 03:32 UTC (age 2.0ч на момент ретро) |
| `additions/deletions/files` | +415/-0/4 |
| Labels | `[]` ← пусто |
| Issue #1598 labels | `source:gsd`,`type:functional`,`priority:high`,`hermes`,`agent:backend`,`no-e2e-required` |
| Issue #1598 state | open, milestone=M4:Avatar, **assignee=empty** (т.к. это issue, не PR) |

**Главный феномен:** PR висит 5+ часов без `needs-e2e` И без `needs-review`. PR
mergeable+clean, но `merge-gate` **не выбрал его как кандидата** для reconcile.

**Кто должен был подхватить:** `agent-flow-merge-gate.sh` цикл Phase 3. Но он
проверяет PR по issue-номеру в title → нашёл #1610 (есть `AV-4` в title, но **нет
`#1598`** в title). Тогда:

- `gh pr list --search "1598 in:title"` → **возвращает пусто** для PR #1610
- `grep -F "z-{agent}/1598-"` → не находит
- Цикл reconcile skip-ит PR #1610

**Дополнительная улика — issue body issue #1598** содержит явный шаблон:

```
## Branch
`feature/av-4-supervisor-lock-manager` (от `feature/avatar`).
```

→ Шифу/воркер сознательно выбрал `feature/av-4-...` вместо `z-{agent}/1598-av-4-...`.
Это **не случайность** — это сознательное решение, основанное на CONTRIBUTING.md §🌿,
где `feature/{slug}` — канонический шаблон.

### Случай 2 — карточка t_783a8a4e (AV-10 / issue #1604)

| Поле | Значение |
|------|----------|
| `branch` (worktree) | `feature/av-10-telegram-supervisor-client` (HEAD d94eed4a) |
| Worktree path | `/home/builder/rob_box_project/.worktrees/t_783a8a4e` |
| Created | 2026-08-24 22:56 by agent-flow-triage |
| Issue #1604 state | open, labels: `source:gsd`,`priority:medium`,`hermes`,`agent:backend`,`needs-e2e`,`refactor` |
| Card t_783a8a4e status | **blocked** (consecutive_failures=2, worker exited cleanly) |
| Last commit | d94eed4a — `wip(telegram #t_783a8a4e AV-10): UI gate через /avatar/state + subscribe` (architect, 25.08 05:27 CEST) |

**Тут другой класс:** worktree + branch были созданы через триаж (с
`branch_for()`), НО архитектор переименовал ветку вручную после worktree-add,
и push пошёл на `feature/av-10-...` вместо `z-{agent}/1604-av-10-...`. Worktree
остался на старом имени, дальнейшие коммиты — на feature-ветке.

Подтверждение: коммит `d94eed4a` имеет в commit-message `#t_783a8a4e` — это
ссылка на КАРТОЧКУ, а не на issue. Значит архитектор, который взял карточку,
поменял ветку не сообщая процессу.

## Корень — почему так вышло дважды

1. **Документация двойственная:**
   - `CONTRIBUTING.md` §🌿 "Стратегия веток (Git Flow)" — **только**:
     `feature/{slug}`, `fix/{slug}`, `release/v*.*.*`, `hotfix/{slug}`. Нет
     `z-{agent}/...` нигде в CONTRIBUTING.md.
   - `agent-flow-triage.sh:219` — `printf 'z-{agent}/%s-%s'` жёстко вшит в
     скрипт, но это **runtime-конвенция воркеров**, а не общешкольный канон.
   - `AGENT_FLOW_PROPOSAL.md` (если есть) — отдельный документ процесса, не
     пересекается с CONTRIBUTING.md.

   **Расхождение:** человек/воркер читает CONTRIBUTING.md, видит
   `feature/{slug}`, делает PR. Процесс-скрипт ждёт `z-{agent}/...`,
   не находит, **молча skip-ает**. Шифу ничего не подозревает, потому что PR
   в GitHub UI показывается как обычный OPEN PR.

2. **Триаж не валидирует имя ветки в существующем PR** — OPEN-PR guard
   (строки 643-647) ищет PR **по вычисленной `branch_for()` ветке**. Если
   PR есть на ДРУГОЙ ветке (feature/...), триаж этого не видит → пытается
   создать новую карточку. Worktree-add тогда либо фейлится (если ветка
   занята), либо создаёт параллельный `z-{agent}/...`, и воркер в нём
   переоткрывает ту же работу.

3. **Merge-gate не имеет универсального «найти issue по PR»** — три отдельные
   логики ищут PR по разным правилам:
   - `_drift_branch_pattern="z-{agent}/${number}-"` (merge-gate:1016)
   - `_followup_json ... --search "${number} in:title"` (merge-gate:962)
   - `is_proposal_branch` только для `z-architect/*` без номера и без t_*.

   Ни одна не учитывает `feature/<id>-...` или другие префиксы. PR #1610
   title содержит `[AV-4]` и `issue #1598` в body, но **не в title**.

4. **Распределённая ответственность** — `no-e2e-required` (issue #1598
   имеет эту метку) означает «PR сразу ставится в `needs-review`» без e2e.
   Merge-gate должен был:
   - найти PR (через `gh pr list --search "1598"`)
   - увидеть `no-e2e-required` на issue
   - поставить `needs-review` на PR

   Но так как **поиск PR-а по issue-номеру идёт через grep по title или
   по каноническому `z-{agent}/<id>-<slug>`**, PR #1610 не находится.

5. **Техдолг** — прошлый ретро t_4515d767 (24.08) писал то же самое: «нужен
   rebase/rename ветки», но конкретных фиксов в merge-gate/triage не было
   сделано. PR #1610 + t_783a8a4e — **тот же баг, та же тихая поломка**.

## Что должно было быть (нормальная схема)

PR создаётся → воркер **до** `git push` проверяет, что имя ветки соответствует
`z-{agent}/<issue>-<slug>` (можно override через label `branch:NAME`). Если
не соответствует — либо автоматический rename, либо жёсткий fail.

Merge-gate при reconcile ищет PR по issue-номеру **во всех** префиксах, не
только `z-{agent}/`. Это `gh pr list --search "<issue> in:body,title"` плюс
фильтр по milestone/labels (avatar эпик → `milestone=M4`).

## Решение (assignee=devops)

Зафиксировать в retro-карточке и подготовить 4 acceptance-критерия для воркера
devops (этот ретро-архитектор пишет план, фикс делает воркер):

### A. `agent-flow-triage.sh` — pre-create OPEN-PR guard (ALL branches)

Сейчас (строки 643-647):
```bash
if open_pr="$(gh pr list --repo "$GH_REPO" --head "$branch" --state open \
    --json number --jq '.[0].number' 2>/dev/null || true)" \
    && [ -n "$open_pr" ]; then
    log "issue #${number}: branch ${branch} already has OPEN PR #${open_pr} — ..."
    skipped=$((skipped+1)); continue
fi
```

Изменить: искать PR **по issue-номеру во ВСЕХ префиксах**, а не только по
`$branch`:

```bash
# Ретро 25.08 t_4bb8d455: PR может быть открыт на НЕканонической ветке
# (feature/<slug> от GSD workflow). Если PR содержит issue-номер в title или
# body — это "работа в PR", новую карточку не создаём.
_existing_pr=$(gh pr list --repo "$GH_REPO" --state open --search "${number} in:body,title" \
    --json number,headRefName --jq '.[0].number // empty' 2>/dev/null || true)
if [ -n "$_existing_pr" ]; then
    log "issue #${number}: OPEN PR #${_existing_pr} уже ссылается на этот issue — skip (reopened-loop guard, ретро t_4bb8d455)"
    skipped=$((skipped+1)); continue
fi
```

Backward-compat: явная `branch:` метка остаётся приоритетом (override).

### B. `agent-flow-merge-gate.sh` — branch-naming валидатор

Добавить функцию `validate_branch_name()`:

```bash
validate_branch_name() {  # $1=head → 0 (OK) / 1 (warn)
    local head="$1"
    case "$head" in
        # Канонические воркерские ветки
        z-{agent}/[0-9]*-*)  return 0 ;;
        z-{agent}/t_*-*)      return 0 ;;
        z-architect/[0-9]*-*) return 0 ;;
        z-architect/t_*-*)    return 0 ;;
        z-devops/*)           return 0 ;;
        # Proposal-ветки (по is_proposal_branch)
        z-architect/*)        return 0 ;;
        # Разрешённые CONTRIBUTING.md префиксы (фичи/фиксы)
        feature/[a-z0-9-]*)   return 0 ;;
        fix/[a-z0-9-]*)       return 0 ;;
        release/v*)           return 0 ;;
        hotfix/[a-z0-9-]*)    return 0 ;;
        # PR-ы от Шифу/людей через GSD workflow — warn
        *)                    return 1 ;;
    esac
}
```

В Phase 3 цикле: если PR прошёл guard (CI green, mergeable), и ветка не
проходит `validate_branch_name` → ставить label `needs-branch-rename` + comment:

```
⚠️ **branch-naming convention** (merge-gate, ретро 25.08 t_4bb8d455)

PR-head `feature/av-4-supervisor-lock-manager` НЕ соответствует канону
`z-{agent}/<issue>-<slug>` или `feature/<slug>` (CONTRIBUTING.md §🌿).

Что делать:
1. Если это feature-ветка — допустимо, переименуйте в
   `feature/1598-av-4-supervisor-lock-manager` чтобы триаж смог найти
   PR по issue-номеру.
2. Если воркерская — `git branch -m z-{agent}/1598-av-4-supervisor-lock-manager`
   + `git push --force-with-lease origin z-{agent}/1598-av-4-supervisor-lock-manager`
   + close старый PR + создать новый.

Без переименования PR не попадёт в ротацию и не получит `needs-e2e`/`needs-review`.
```

### C. `auto-rename-pr-branch.sh` (helper для воркера)

`scripts/agent_flow/rename-pr-branch.sh <pr> <new-branch>` — безопасный
helper: проверяет что `$new-branch` не существует, делает
`git push origin <old>:<new>` + переоткрытие PR с тем же body. Для случаев
«PR готов, но воркер забыл переименовать».

### D. `CONTRIBUTING.md` — секция "Branch naming: agent-flow convention"

Добавить явную секцию с двумя режимами:

```markdown
### Режим A: human-driven (Шифу/GSD workflow)

Если PR создаётся руками или через GSD workflow:
- Используйте `feature/<slug>` или `fix/<slug>` (канонично по Git Flow)
- В **title** PR добавьте `[<AV-N>]` И `#<issue-number>` — чтобы
  merge-gate смог найти PR по issue-номеру через `--search`.
- В **body** PR ссылка на issue обязательна (`Closes #<id>`).

### Режим B: worker-driven (agent-flow triage)

Если воркер подхватил issue через kanban-карточку:
- Ветка ОБЯЗАНА быть `z-{agent}/<issue>-<slug>` (вычисляется
  автоматически из `branch_for()`).
- Push на другую ветку → воркер будет заблокирован merge-gate'ом
  через `validate_branch_name`.
```

## Acceptance для воркера devops (будущая карточка t_XXX)

- [ ] agent-flow-triage.sh: pre-create guard ищет PR по `--search "${number} in:body,title"`
      (а не только по `--head "$branch"`). Skip с правильным сообщением.
- [ ] agent-flow-merge-gate.sh: добавлена `validate_branch_name()`, вызывается в Phase 3.
      Не-валидные PR получают label `needs-branch-rename` + комментарий с инструкцией.
- [ ] scripts/agent_flow/rename-pr-branch.sh существует, executable, +test (`bats` или
      inline python3 dry-run на mock-репо).
- [ ] CONTRIBUTING.md: добавлена секция "Branch naming: agent-flow convention" с
      двумя режимами + reference на PR #1610 как warning-case.
- [ ] CI зелёный (docs-only + bash lint).
- [ ] **Backfill существующих "потерянных" PR:**
      - PR #1610 (`feature/av-4-supervisor-lock-manager`) — поставить `needs-review`
        после rename ветки или добавить label `needs-branch-rename` + comment.
      - t_783a8a4e / issue #1604 (`feature/av-10-telegram-supervisor-client`) — то же.
      - Воркер devops **НЕ** rename'ит ветки руками — только ставит метки и
        предлагает Шифу через `gh pr comment`.

## Связанные

- t_4515d767 (24.08, done) — первый кейс PR #1206, фикс не докатился.
- ADR-0014 (process) — convention (но НЕ упоминает `z-{agent}/`).
- AV-* эпик 24.08 22:53..23:00 — все карточки созданы пакетом из #1595 (decompose),
  проверка ветки была пропущена.
- t_8cde8449 (24.08) — pre-create PR guard (для `branch:` override метки). Это
  полезно, но **не покрывает** случай «PR есть на feature/* ветке без `branch:` метки».

## Метрики

- 25.08 07:35 UTC: обнаружено 2 потерянных PR за последние 8ч (24.08 23:00..25.08 07:00).
- 24.08 16:00 UTC: 1 потерянный PR (PR #1206, ретро t_4515d767).
- Итого за 16ч: 3 кейса. Если воркеры будут активнее — будет расти линейно.
