# stale-PR detection (SOT)

> Source of truth: что считаем stale, что НЕ считаем, threshold, edge cases.
> Ретро: t_a2cd5753 (22.08 — оригинальная BEHIND-логика),
>        t_bfb3c01a (28.08 — false-positive, переход на AHEAD-логику).

## Что считаем STALE

`scripts/agent_flow/agent-flow-e2e-process.sh::stale_branch_check()` блокирует
PR если число **AHEAD-коммитов** в PR tip превышает `E2E_STALE_BRANCH_THRESHOLD`
(по умолчанию 10) после вычета SHA-tag noise.

**AHEAD** = коммиты в PR tip, которых НЕТ в `origin/${FOUNDATION_BRANCH}` (develop).
Команда: `git rev-list --count origin/${foundation}..origin/${agent_branch}`.

Это эквивалентно «сколько нового кода несёт PR» — что и нужно для e2e-staleness.

### Почему AHEAD, а не BEHIND (бывшая метрика)

**BEHIND** = коммиты в develop, которых нет в PR tip.
Команда: `git rev-list --count origin/${agent_branch}..origin/${foundation}`.

Проблема (ретро 28.08 t_bfb3c01a, issue #1707 / PR #1726): на долгоживущем
репо число BEHIND растёт само по себе при каждом merge в develop, даже если
PR MERGEABLE/CLEAN по GitHub (3-way merge подхватывает develop HEAD).

Live evidence (28.08 16:59Z):
- issue #1707: PR tip 1 коммит ahead, 4205 BEHIND, raw=4012, real=3424,
  GitHub `mergeable=MERGEABLE / mergeStateStatus=CLEAN`. Detector BLOCKED →
  rebase-комментарий впустую.
- PR #1726: 1 ahead, 192 BEHIND, MERGEABLE/CLEAN — та же проблема.

**BEHIND НЕ ловит нужный баг** — устаревшие скрипты уже решены через
`E2E_RUN_BEFORE` (test-branch берётся от develop HEAD → актуальные сценарии),
см. PR #1547. Метрика AHEAD ловит реальный риск — большая пачка
непротестированных изменений маскирует регрессии.

## Что НЕ считаем stale

### 1. SHA-tag noise (whitelist)

CI auto-tagger коммитит на develop:
- `ci: vision SHA tags → dev-XXX`
- `ci: main SHA tags → dev-XXX`

Они идут парой после каждого Merge-PR и не меняют код. Guard вычитает их из
AHEAD перед сравнением с threshold через:

```bash
git log --oneline ${_dev_sha}..${_tip_sha} \
  | grep -vE '^[0-9a-f]+ ci: (vision|main) SHA tags '
```

Если после whitelist `real_ahead ≤ threshold` → OK.

### 2. PR tip содержит develop HEAD

AHEAD = 0 → PR tip форвард/равен develop HEAD → OK.

### 3. `E2E_STALE_BRANCH_SKIP=1`

Разовый override (escape hatch) — пропустить guard для конкретного PR,
например когда дробление физически невозможно (один атомарный рефакторинг).
Шифу подтверждает override явно в issue (см. шаблон в stale-PR comment).

## Threshold

`E2E_STALE_BRANCH_THRESHOLD=10` по умолчанию. Переопределяется через env
(например `E2E_STALE_BRANCH_THRESHOLD=30` для batch-merge нескольких fix'ов).

| AHEAD после whitelist | Решение | Комментарий |
|---|---|---|
| 0 | OK | PR tip содержит develop HEAD |
| 1-10 (default) | OK | Нормальный dev cycle |
| >10 (default) | BLOCKED | Дробить на sub-PR'ы по ≤10 коммитов |

## Fail-safe поведение

- Если `git fetch`/`rev-parse` упали → return 0 (пропуск check, не блокируем).
- Если `rev-list --count` вернул пусто → return 0 (пропуск check).
- Если `git log --oneline` упал (whitelist) → `_real_ahead = _ahead` (fail-safe
  на raw — лучше заблокировать, чем пропустить).

## Live integration

Pre-round guard (`pre-round guard` в e2e-process main loop) вызывает
`stale_branch_check` перед созданием round-ветки. Если guard BLOCKED:
- не создавать `z-{e2e}/test-round-N`
- написать идемпотентный issue-коммент (24h окно)
- issue остаётся в очереди — следующий тик (или rebase) переподхватит

Pre-merge re-check (перед `git merge origin/${agent_branch}`) вызывает
`stale_branch_check` ещё раз. Если за время между guard и merge PR успел
«разрастись» — skip merge, требовать rebase.

## Тестирование

`scripts/agent_flow/tests/test_e2e_process_stale_branch.sh` покрывает:

| # | Сценарий | Фикстура |
|---|---|---|
| A | ahead=20, threshold=10 → BLOCKED | `STALE_AHEAD=20` (20 real) |
| B | ahead=0 → OK | (default) |
| C | ahead=5, threshold=10 → OK | `STALE_AHEAD=5` |
| D | ahead=20, threshold=30 → OK | `STALE_AHEAD=20` + threshold override |
| E | ahead=25 → BLOCKED + dedup | `STALE_AHEAD=25` + `STALE_COMMENT_PRESENT=1` |
| F | stale_branch_check в обоих местах (grep) | (structural check) |
| G | live evidence #1707: ahead=1 → OK | `STALE_AHEAD=1` |
| H | live evidence PR #1726: ahead=1 → OK | `STALE_AHEAD=1` |

Для тестов whitelist (SHA-tag noise) — фикстура `STALE_LOG_<branch>=<noise>:<real>`:
- `STALE_LOG_branch=5:15` → 5 noise + 15 real = 20 коммитов, после whitelist real=15.
- Дефолт: `STALE_AHEAD_<branch>=N` → N real, 0 noise.

## Связанные

- `scripts/agent_flow/agent-flow-e2e-process.sh::stale_branch_check` (lines ~2250-2380)
- `scripts/agent_flow/tests/test_e2e_process_stale_branch.sh`
- ADR-0022 (e2e-staleness general policy)
- Ретро t_a2cd5753, t_bfb3c01a (issue #1707, PR #1726)
- ADR-0023 (skill validation history)
