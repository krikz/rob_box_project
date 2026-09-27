# ADR-0030: e2e stale-branch guard — false-positive rate и ремедиация

- **Статус:** PROPOSED (2026-08-25)
- **Ретро-карточка:** t_336de9df
- **Автор:** architect
- **Assignee на имплементацию:** devops

## 1. Контекст

e2e-ротация на rob_box **застряла на 36+ часов** (с 23.08, последний успешный E2E — round-212).
Тики идут, round-ветки создаются и удаляются (round-224), но **0 раундов с E2E-прогоном**,
все 12 needs-e2e issues блокированы pre-round guard.

### Raw-evidence подтверждение (2026-08-25 07:32 UTC)

`/home/builder/.hermes/logs/agent-flow-e2e-process-run-now.log`:
```
[07:06:59] stale-branch: PR tip .../1561-bug-voice-992-llm-music-tools-execute-mu raw=29, real=12 — STALE
[07:07:21] stale-branch: PR tip .../1564-bug-voice-n-ndone-done-marker-tts raw=29, real=12 — STALE
[07:07:06] stale-branch: PR tip .../1562-bug-voice-new-session-tts raw=204, real=79 — STALE
[07:07:14] stale-branch: PR tip .../1563-bug-voice-new-session-tts raw=204, real=79 — STALE
[07:07:29] stale-branch: PR tip .../1593-deploy-issues-on-develop-staging-2026-08 raw=204, real=79 — STALE
[07:08:42] 🛑 no live e2e candidates (12 needs-e2e issues, все без живых PR) — round-ветку НЕ создаю
```

### Состояние 6 проблемных PR (REST-проверка 2026-08-25 05:40 UTC)

| PR  | branch                                           | mergeable | commits | additions | deletions | real-behind | категория     |
|-----|--------------------------------------------------|-----------|---------|-----------|-----------|-------------|---------------|
| 1565 | z-{agent}/1561-bug-voice-992-llm-music-tools...  | true      | 1       | 620       | 10        | **12**      | пограничный   |
| 1567 | z-{agent}/1564-bug-voice-n-ndone-done-marker-tts | true      | 1       | 413       | 3         | **12**      | пограничный   |
| 1568 | z-{agent}/1563-bug-voice-new-session-tts         | true      | 2       | 490       | 4         | **79**      | реально stale |
| 1572 | z-{agent}/1562-bug-voice-new-session-tts         | true      | 1       | 237       | 2         | **79**      | реально stale |
| 1594 | z-{agent}/1593-deploy-issues-on-develop-...      | true      | 2       | 379       | 1         | **79**      | реально stale |
| 1566 | z-{agent}/1560-bug-ci-develop-build-15-20-...    | null      | 2       | 302       | 0         | (closed)    | исключён      |

Все open PR — `mergeable=true` (нет конфликтов с develop). Отстают, но не конфликтуют.
PR #1544 (в ретро) — удалён (404). PR #1566 — closed.

## 2. Что блокирует — два независимых фактора

### Фактор A: stale-branch guard (вторике — главный)

- `E2E_STALE_BRANCH_THRESHOLD=10` (default в `scripts/agent_flow/agent-flow-e2e-process.sh:148`).
- **801 из 4923 коммитов develop (16%)** — это `ci: (vision|main) SHA tags` noise.
  Whitelist (5967b073) уже отфильтровывает noise → "real" остаётся ~80 для "старых" PR.
- При активной разработке 10 коммитов отставания — это **нормальное** состояние для PR,
  который открыт до сегодняшних merge'ей (zenoh fixes, motion stack revert, agent-flow fixes,
  ADR-0027/0028).
- Guard был введён (ретро 22.08 t_a2cd5753, commit e1432848) для fail-streak round-166/167/168/170/178,
  где test-branch наследовал устаревший `voice_core_suite_v1.json` без `E2E_RUN_BEFORE`. **Проблема
  была в test-branch, не в PR**. Текущая реализация путает "stale PR" и "test-branch without
  current scenario" — два разных риска.

### Фактор B: GraphQL rate-limit (secondary, маскирующийся)

- `gh auth status` — REST, работает.
- `gh issue list --json` / `gh pr list --json` — **GraphQL**, падает на rc=1 когда quota=0.
- Скрипт под `set -euo pipefail` (line 49) умирает.
- Логи пишут "gh auth not configured" (line 458) — **misleading**: auth есть, проблема в GraphQL.
- Подтверждение: GraphQL reset в 05:47 UTC, в 05:39 quota=0/5000, в 05:50 quota должна восстановиться.

## 3. Корневые причины и trade-off

### 3.1 Stale-branch guard слишком жёсткий (ГРИП в ретро)

**Почему так вышло:** guard проектировался исходя из "должно быть ≤1 PR отстаёт от develop".
Это было верно, когда 1 раунд = 1 PR. Сейчас:
- 5-10 влитых PR в день (zenoh, voice, agent-flow, adr).
- SHA-tag noise от CI auto-tagger добавляет 2 коммита на каждый merged PR.
- 16% develop — noise; whitelist помогает, но real=10 уже на грани.

**Trade-off:**
- Слишком жёсткий (10): false-positive блокирует живые PR (текущий баг).
- Слишком мягкий (100): вернётся fail-streak 22.08, устаревший `voice_core_suite_v1.json`.

**Реальная защита от fail-streak 22.08 — не в PR freshness, а в test-branch check:**
  test-branch должен иметь **актуальный** `voice_core_suite_v1.json` и `e2e_voice_test.sh`,
  даже если PR tip отстал. Это отдельная проверка в момент `round_ensure()`.

### 3.2 GraphQL exhausted — ложный auth-fail (ГРИП 2)

**Почему так вышло:** `gh auth status` (REST) проверяется, но `gh issue list --json` (GraphQL)
никем не проверяется **до использования**. Упасть может посередине скрипта.

**Trade-off:**
- Catch GraphQL на старте → +1 REST вызов на тик, но честный fail.
- НЕ catch → скрипт падает в неожиданных местах с misleading "auth not configured".

### 3.3 "PR отстал на N" ≠ "PR нельзя мержить"

**Архитектурный insight:** guard сейчас сравнивает **коммиты**, а должен сравнивать **файлы, влияющие на e2e**:
- `.github/e2e/scenarios/voice_core_suite_v1.json` — обновляется ~раз в неделю.
- `.github/workflows/scripts/e2e_voice_test.sh` — обновляется в рамках voice PR.
- `e2e_voice_test.sh` — аналогично.

Если файлы в PR tip **идентичны** develop — PR не устарел с точки зрения e2e, даже если
коммитов отстаёт 80.

## 4. Решение (proposed)

### 4.1 Tiered stale-branch guard

Заменить single threshold на **двухуровневую** проверку:

| real-behind | поведение |
|-------------|-----------|
| ≤ 20        | OK, лог INFO |
| 21-50       | WARNING, проверка file-freshness, если файлы e2e в PR tip идентичны develop → OK |
| 51-80       | WARNING + auto-stale-rebase-try (best-effort, fail-safe) |
| > 80        | BLOCKED (как сейчас) |

**Реализация:** новый env `E2E_STALE_BRANCH_TIER_HARD=80`, `E2E_STALE_BRANCH_TIER_SOFT=20`.
Переменная `E2E_STALE_BRANCH_THRESHOLD` оставить как алиас для tier_hard (back-compat).

### 4.2 File-freshness check (новое)

Сравнивать не "N коммитов", а "PR tip vs develop по 3 файлам":
- `.github/e2e/scenarios/*.json`
- `.github/workflows/scripts/e2e_voice_test.sh`
- `scripts/e2e_voice_test.sh` (если есть)

Если PR tip == develop по этим файлам — guard не блокирует независимо от commit count.

### 4.3 Auto-stale-rebase-try (для tier 51-80)

best-effort, в worktree:
```bash
git fetch origin develop "$branch"
git checkout "$branch"
git rebase origin/develop   # НЕ merge-commit
if clean → push --force-with-lease, log INFO "auto-rebased"
if conflict → skip, log WARN "auto-rebase conflict — нужна ручная rebase-карточка"
```

**Не** трогает merge-commit PR (которые делают Шифу — `Merge pull request #N from ...`).
**Не** трогает PR, которые Шифу явно отметил `do-not-rebase` (новая label `rebase:manual`).

### 4.4 GraphQL rate-limit fallback

В `agent-flow-e2e-process.sh`:
- В G2 (line 451) добавить **параллельно** REST-проверку `gh api rate_limit`.
- Если `resources.graphql.remaining=0` → `log "GraphQL exhausted, exit 0 (skip tick)"`.
- Не "exit 1" с misleading "auth not configured".

В `stale_branch_check()` (line 1604):
- Если GraphQL exhausted **в момент** `gh pr list` — skip stale-check (fail-safe).

### 4.5 recovery-карточки для текущего затора

Для PR #1568/#1572/#1594 (real=79) — **не** auto-rebase (tier > 80), но **создать** devops-карточки:
- `git fetch origin develop <branch>`
- `git rebase origin/develop` (или merge если rebase конфликтует)
- `git push --force-with-lease origin <branch>` через `push-via-gh-api.sh` (см. memory)
- e2e-process переподхватит на следующем тике (метки `needs-e2e` не трогать)

Для PR #1565/#1567 (real=12) — **ничего не делать**, после фикса 4.1 они пройдут guard
автоматически на следующем тике.

## 5. Acceptance criteria

- [ ] Tiered guard: real≤20 OK, 21-50 WARN+file-check, 51-80 auto-rebase-try, >80 BLOCKED.
- [ ] File-freshness check: PR tip == develop по 3 файлам → OK.
- [ ] GraphQL quota=0 → exit 0, лог "skip tick", не exit 1 с misleading auth-fail.
- [ ] Auto-rebase НЕ ломает merge-commit PR; НЕ ломает PR с label `rebase:manual`.
- [ ] regression-test: `tests/` — `test_stale_branch_tiers.sh` (real=0/12/30/60/100 →
      expected outcome); `test_graphql_exhausted_skip.sh` (mock quota=0 → skip).
- [ ] Recovery-карточки для #1568/#1572/#1594: devops перебазирует → e2e-ротация оживает
      в течение 2 тиков (≤2ч).

## 6. Не делаем

- **Не** поднимаем threshold до 100 "чтобы жило" — это вернёт fail-streak 22.08.
- **Не** удаляем stale-branch guard совсем — fail-streak round-166..178 стоил 5 пустых раундов.
- **Не** путаем "PR отстал" и "test-branch унаследовал устаревший scenario.json" —
  это разные проблемы; file-freshness check адресует вторую.

## 7. Связанные

- **Ретро 22.08 t_a2cd5753** — добавил guard (e1432848).
- **Ретро t_fdb19f7b** — SHA-tag whitelist (5967b073), решил часть проблемы.
- **PR #1592** (MERGED) — `blocker_filter exclude no-e2e-required`; частично снял #1586 false-positive.
- **Ретро t_57b9b28c** — диагноз "e2e-rotation PAUSED" был ложным; этот ADR уточняет корень.
- **Память agent-flow-pipeline-ops** — `RUN_NOW round explosion` и `e2e-process source-order`
  могут быть косвенно затронуты при изменении guard (следить за watchdog dedup).

## 8. Следующие шаги

1. **devops** имплементирует 4.1-4.4 в `agent-flow-e2e-process.sh` (один PR в develop,
   assignee=devops, skill=agent-flow-pipeline-ops).
2. **devops** создаёт 3 recovery-карточки для #1568/#1572/#1594 (assignee=devops,
   body = "rebase + push-via-gh-api.sh + проверить needs-e2e не снят").
3. **architect** (этот ретро) — закрыть t_336de9df после merge devops-фикса + green e2e round.
4. **Юзер** (Шифу) проверяет, что e2e-ротация ожила ≤2ч после merge devops-фикса.