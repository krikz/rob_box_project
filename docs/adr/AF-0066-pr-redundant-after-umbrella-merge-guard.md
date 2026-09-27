# AF-0066 — guard от PR-redundant-after-umbrella-merge (G10b в triage)

- **Статус:** proposed
- **Дата:** 2026-09-15
- **Автор:** devops-профиль (ретро-карточка t_6c594d08)
- **Связанная issue:** [#2459](https://github.com/krikz/rob_box_project/issues/2459)
- **Кейс-пример:** PR #2453 (ветка `z-architect/2440-znakomyi-identity-seam`, ADR-only) висел UNSTABLE >24ч после того, как ADR-0095 уже попал в develop через PR #2458 (commit `8cb84bc`), который закрывал issue #2406 как umbrella и попутно увлёк реализацию identity-seam по тому же issue #2440.

## Проблема

Race-condition при параллельной работе нескольких воркеров над одной issue:

1. Воркер A (architect, t_5986a91c) берёт issue #2440 → создаёт ветку `z-architect/2440-znakomyi-identity-seam` с ADR-only коммитом (`8299fa2`, 826 строк `docs/adr/0095-...md`).
2. Параллельно воркер B (видимо другой поток того же архитектора или dev, t_некий) берёт umbrella-issue #2406 → ведёт PR #2458, который merge-ит в develop коммит `8cb84bc` — реализация identity-seam + ADR 0095 (через коммит `42cd89b fix(process #2444)`).
3. Когда воркер A пытается смёрджить свой PR #2453, develop уже содержит и ADR-файл и реализацию → UNSTABLE, merge-base = origin/develop tip, merge-tree чист, но манифест изменений пересекается.
4. PR #2453 висит неделями, мешает drift-detect / merge-gate, никто его не закрывает.

**Корневая причина:** на момент создания ветки `z-{agent}/<issue>-<slug>` НЕТ проверки, что для этого issue **уже есть merged superseder через umbrella или другой PR**, чьи commit messages / body ссылаются на тот же issue-ref.

## Решение

Добавить в `scripts/agent_flow/agent-flow-triage.sh` новый guard **G10b `issue_already_resolved_by_other_pr`**, срабатывающий **до** создания ветки (после `existing_by_issue` idempotency-check, до `branch_exists_in_remote` G9b).

### Логика

1. Получить JSON всех PR в репо (одним запросом, кэш на тик через `$PRS_JSON_CACHE` — открыть сразу для open и merged, последние N=200):
   - `gh pr list --repo "$GH_REPO" --state all --limit 200 --json number,title,body,state`

2. Для каждого PR определить, ссылается ли он на наш issue (`$1` — issue_number). Сигналы ссылки:
   - В title: `\(#${number}\)` или `#${number}` standalone,
   - В body: `closes #${number}`, `fix #${number}`, `fixes #${number}`, `resolves #${number}`, `ref #${number}`, `refs #${number}`, либо `#${number}` как standalone (с word-boundaries).
   - **Merge commit message НЕ проверяем** (потребует `gh api repos/.../commits` для каждого кандидата — лишний вес).

3. **Ступень A (superseded-by-MERGED, hard skip):** если найден PR с state=MERGED, у которого в title или body есть явная ссылка `#${number}` → **skip + comment «superseded by umbrella-merge #<that_pr>» + label `agent-flow-error:superseded`**.

4. **Ступень B (other-OPEN same-issue, soft skip):** если найден PR с state=OPEN, у которого **в title** есть `#${number}` или текст title совпадает с issue title ≥60% (fuzzy word-overlap) — это ПОЧТИ наверняка параллельный воркер. Skip с другим маркером «agent-flow-error:race-window», но только если ступень A не сработала.

5. **Backward-compat:** если `AGENT_FLOW_ISSUE_RESOLVED_GUARD=false` → early-return (полный skip поведения, ровно как до фикса). Дефолт — `true`.

6. **Fail-OPEN:** любые сетевые/gh-ошибки → log + return 1 (продолжаем обычный flow). Принцип идентичен G10a (`file_overlap_with_open_pr`) и G9b (`branch_exists_in_remote`).

### Marker и комментарий

Помечаем issue-комменты маркером `<!-- hermes-triage-g10b: <hash> -->` (по образцу G10a marker), где `hash = sha1("<issue_number>|<superseder_pr>|<state>")`. DEDUP_HOURS по дефолту 6 часов (state change → edit вместо нового коммента, как в G10a).

### Idempotency vs существующий G10a

G10a ловит **file-overlap** (другая ветка правит тот же файл). G10b ловит **issue-supersed** (другая ветка ссылается на тот же issue-ref). Это ортогональные guard'ы; G10a плюс G10b вместе покрывают оба кейса.

| Guard | Что ловит | Сигнал |
| --- | --- | --- |
| G5/G6b | OPEN PR на той же branch | `gh pr list --head "$branch"` |
| G9b | Уже-существующая ветка | `git ls-remote refs/heads/<branch>` |
| G10a | file-overlap | `gh pr list --json files` + basename match |
| **G10b** | **issue-supersed** | **`gh pr list --json title,body` + issue-ref** |

## Acceptance

Восходит к issue #2459 § Acceptance:

- [x] G10b срабатывает наличии MERGED superseder с явной ссылкой на тот же issue → skip + comment + label.
- [x] G10b-idempotency: повторный тик в течение 6ч с тем же state-hash → no-op, без нового коммента.
- [x] G10b-disabler: `AGENT_FLOW_ISSUE_RESOLVED_GUARD=false` → backward-compat, guard полностью пропускается.
- [x] Тест `tests/test_triage_issue_resolved_guard.sh`: unit-style, проверяет helper-функцию + что guard вызывается в `process_issues_json` до branch_exists_in_remote.
- [x] `bash -n scripts/agent_flow/agent-flow-triage.sh` — без синтаксических ошибок.
- [x] `shellcheck scripts/agent_flow/agent-flow-triage.sh` — без HIGH-severity.
- [ ] Один прогон на develop (после merge) — нет новых phantom PR; верификация — после merge этого ADR.

## Файлы

- `scripts/agent_flow/agent-flow-triage.sh` — добавляем helper `issue_already_resolved()` и его вызов в `process_issues_json`.
- `scripts/agent_flow/tests/test_triage_issue_resolved_guard.sh` — unit-тест по образцу G10a.
- `docs/design/agent-flow.md` — короткая запись в changelog (опционально, в этом PR можно отложить если changelog-процесс отдельный).

## Учтённые нюансы

- **#2453 уже получил supersede-комментарий от krikz вручную** (см. timeline-event `cross-referenced`). Этот фикс формализует поведение guard'а — следующие подобные кейсы будут ловиться автоматически.
- **Issue #2459 — спека работы**, а не отдельная задача. Решается в рамках ретро-карточки t_6c594d08.
- **Fail-OPEN обязателен**: cron не должен ломаться из-за временного gh-сетевого сбоя. Принцип fail-OPEN уже внедрён в G10a.
- **Кэш PR-list на тик**: один `gh pr list` запрос используется и G10a, и G10b (через `$PRS_JSON_CACHE` вместо двух отдельных запросов) — следующая попытка оптимизации (сейчас каждый guard запрашивает своё — оставляем существующее поведение, в этом ADR не рефакторим).
