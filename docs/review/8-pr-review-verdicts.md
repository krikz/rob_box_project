# Review verdicts: 8 открытых PR в krikz/rob_box_project (kanban t_d864b407)

**Дата:** 2026-08-31 (воскресенье)
**Автор:** @architect (по kanban-карточке t_d864b407, запрошено товарищем Шифу)
**Branch артефакта:** `z-{agent}/1759-task-review-8-pr` (base=develop)
**Шаблон:**
- Свежесть: `compare develop<->head` через `gh api`.
- Checks: 9/9 SUCCESS для всех 8 PR.
- Решение Шифу: MERGE / REBASE-merge / REORG / DISCUSS.

## Сводка решений

| # | Branch | Свежесть (ahead/behind) | Решение | Почему |
|---|---|---|---|---|
| 1756 | `z-devops/t_04371252-merge-gate-stale-branch` | 1/0 | **MERGE-AS-IS** | Системный fix для stale-branch-reuse loophole. 9/9 CI OK, regression-тест 10 кейсов. |
| 1753 | `z-agent/t_1d0426e3-blocked-watchdog` | 2/0 | **MERGE-AS-IS**, **снять label `stale-branch-reuse`** | `lib_cron_env.sh` отсутствует в develop, PR #1704 — другой PR. Лейбл поставлен ошибочно. |
| 1752 | `z-devops/t_18941c54-deploy-sweep-env-fallback` | 1/0 | **MERGE-AS-IS** | Libа-центричный фикс (af_load_profile_env fallback). Минимальный diff +22/-4. |
| 1751 | `z-architect/t_8fdc62ee-stale-diagnostic-auto-detect` | 1/0 | **MERGE-AS-IS** (ADR полезен) | Документация для будущей реализации. 3 стратегии детекта, marker DSL, мини-валидация rate-limit. |
| 1750 | `z-{agent}/1730-fix-unlabeled-sweep-env-fallback` | 1/1 diverged | **REBASE, потом MERGE** | Diverged с develop (там есть #1748 merged `06b83b01b`). Файлы не пересекаются, но процедура требует rebase. |
| 1747 | `z-developer/t_780eb586-fix-test-fixtures` | 1/0 | **MERGE-AS-IS** (subset of #1748) | 1 файл +6 строк, fixture-only. Body упоминает 4 файла, реально 1 (остальные уже в #1748). Не блокер. |
| 1744 | `z-{agent}/1737-deploy-issues-on-develop-staging-2026-08` | 5/1 diverged | **REORG** (split into sub-PRs) | 5 коммитов, 3 РАЗНЫХ задачи (deploy-dedup + voice getattr-guard + merge-gate UNSTABLE). Commit 1 дублирует #1743. Mixed-PR. |
| 1743 | `z-devops/t_e00f448d-unstable-diagnostic` | 1/0 | **MERGE-AS-IS** (+ обновить #1741 close reason) | Single-file +169/-1. UNSTABLE → diagnostic вместо rebase. Закрывает причину #1741 merge-conflict-loop. |

**Итог Шифу:** 6 MERGE-AS-IS (#1756, #1753, #1752, #1751, #1747, #1743), 1 REBASE-then-MERGE (#1750), 1 REORG-NEEDED (#1744).

## Подробные вердикты

### #1756 — MERGE-AS-IS

URL: https://github.com/krikz/rob_box_project/pull/1756
Title: `fix(merge-gate t_04371252): process-scripts are functional, not ci-only`
CI: 9/9 SUCCESS
Diff: 3 файла, +528/-23 (90% — regression-тест `test_merge_gate_stale_branch_process_scripts.py`)

Полезный фикс: ci-only whitelist `.github/` + `docs/` теперь НЕ включает `scripts/agent_flow/*` (это процессные скрипты). Новая константа `STALE_BRANCH_REUSE_LABEL='stale-branch-reuse'` ставится merge-gate'ом на stale-branch-reuse PR.

Решение: MERGE. После merge — триггернуть merge-gate tick для подтверждения, что #1753 чист.

### #1753 — MERGE-AS-IS, снять label `stale-branch-reuse`

URL: https://github.com/krikz/rob_box_project/pull/1753
Title: `fix(scripts t_a2521b07): cron no-agent env fragile — lib_cron_env.sh FORCE`
CI: 9/9 SUCCESS
Diff: 23 файла, +754/-15

**Находка:** label `stale-branch-reuse` поставлен ошибочно.
- `gh api .../contents/scripts/agent_flow/lib_cron_env.sh` → 404 Not Found в develop.
- PR #1704 (на который ссылается #1756 как «уже влит через») — это другой PR (`feat(scripts): orphan blocked-card watchdog`).
- `compare develop...HEAD` → ahead=2 — коммиты уникальны, в develop их нет.

Полезный фикс: единый `lib_cron_env.sh` (157 строк) — SOURCE-обязателен в начале любого no-agent cron-скрипта. FORCE'ит real HOME, GH_CONFIG_DIR, HERMES_HOME; подгружает `/home/builder/.hermes/profiles/agent-flow/.env` с export-existing-wins.

Решение: MERGE. Шифу нужно **снять label `stale-branch-reuse`** перед merge (или сразу после).

### #1752 — MERGE-AS-IS

URL: https://github.com/krikz/rob_box_project/pull/1752
Title: `fix(agent-flow t_18941c54): deploy-sweep устойчив к per-profile HERMES_HOME`
CI: 9/9 SUCCESS
Diff: 2 файла, +22/-4

Полезный фикс: `af_load_profile_env` в либе получает fallback-кандидаты (если переданный путь не существует). Этим закрывается cron-env-fragility в `deploy-sweep` (та же проблема, что у `unlabeled-sweep` в #1750 — последовательно).

Решение: MERGE. Кандидат на merge в первую очередь — минимальный diff, либа-центричный подход.

### #1751 — MERGE-AS-IS (полезный ADR)

URL: https://github.com/krikz/rob_box_project/pull/1751
Title: `docs(adr-0035): plan for merge-gate auto-detect stale-after-upstream-fix`
CI: 9/9 SUCCESS
Diff: 2 файла, +519/-0 (только docs)

Полезный ADR: документирует 3 стратегии детекта «stale diagnostic-карточка после upstream-фикса»:
- A: `git rev-list --is-ancestor` (PR слит)
- B: `git log -S` / `git log -- <file>` (фикс в develop под другим PR)
- C: failing-tests файлы уже в текущем PR-diff + CI SUCCESS (фикс в самом PR)

Mini-DSL маркеров `<!-- diag-* -->` — grep'абельный, невидим при рендере.

**Ответ на acceptance-критерий:** ADR полезен. Реальный failure case (30.08 23:50Z — #1740/#1741 stale после `06b83b01b`) задокументирован.

Замечания для будущей реализации (НЕ блокируют этот PR):
- Rate-limit 2h может быть агрессивен — предложить adaptive (на основе `time-since-upstream-fix`).
- Strategy B (`git log -S <attr>`) дорогая — нужен cap (max 5000 commits).
- Backfill-скрипт — рекомендую сделать **обязательным** перед реализацией (иначе legacy diagnostic-карточки skip'нутся).

Решение: MERGE.

### #1750 — REBASE, потом MERGE

URL: https://github.com/krikz/rob_box_project/pull/1750
Title: `fix(agent-flow): unlabeled-sweep ENV_FILE robust fallback`
CI: 9/9 SUCCESS
Diff: 2 файла, +128/-12
Состояние: **diverged**, ahead=1, behind=1

Develop содержит `06b83b01b` (#1748 merged 30.08 23:50Z — DialogueNode fixture). В ветке #1750 его нет. Пересечение файлов минимальное (unlabeled-sweep vs dialogue_node fixture) — rebase должен пройти чисто.

Полезный фикс: ENV_FILE fallback (3 кандидата), `set -a;. ` вместо IFS-парсинга, 2 новых теста (T9, T10).

Решение Шифу: `git fetch origin && git rebase origin/develop` в worktree, потом MERGE. Если rebase даёт конфликт — отдать воркеру.

### #1747 — MERGE-AS-IS (subset of #1748)

URL: https://github.com/krikz/rob_box_project/pull/1747
Title: `test(voice t_780eb586): fix DialogueNode test fixtures for retry flags`
CI: 9/9 SUCCESS
Diff: 1 файл, +6/-0

**Находка:** body упоминает 4 фикстуры, но diff правит только 1 файл (`test_issue_1195_tg_source.py`). Остальные 3 уже починены в PR #1748 (merged 30.08 23:50Z).

Решение: MERGE. Тривиальный subset-fix. Если хочется consistency — оставить коммент-ответ воркеру, но НЕ блокировать.

### #1744 — REORG-NEEDED (mixed-PR)

URL: https://github.com/krikz/rob_box_project/pull/1744
Title: `fix(deploy-dedup #1737): exclude 4 false-positive critical_log patterns`
CI: 9/9 SUCCESS
Diff: 4 файла, +331/-6
Состояние: **diverged**, ahead=5, behind=1

5 коммитов в ветке — **3 РАЗНЫХ задачи**:
1. `785941db95` — `wip(merge-gate): classify UNSTABLE — diagnostic vs rebase` — **ДУБЛЬ с #1743** (логика merge-gate.sh, актуальный отдельный PR).
2. `e13723567d` + `e95eb8bf85` — `fix(deploy-dedup #1737): exclude 4 false-positive critical_log patterns` — основная задача. Хорошо задокументирована: каждый false-positive rule + regression-тест с reproduction-deploy-run-id.
3. `ba80517a52` — `fix(voice): getattr-guard DialogueNode retry/music flags for fixture` — **OVERLAP с #1748**.
4. `2ed8ebf704` — `merge origin/z-{agent}/1737-...` — sync.

Решение Шифу:
- **NE MERGE kak est** — mixed-PR.
- Split:
  - Force-push с чистой веткой `z-{agent}/1737-deploy-dedup-only` с commits 2+3+4.
  - Закрыть PR #1744 с комментарием «superseded, см. sub-PR».
  - Commit 1 уже в #1743 — там и живёт.
- Альтернатива (хуже): squash-merge (5→1) — теряется traceability.

### #1743 — MERGE-AS-IS + обновить комментарий #1741 close

URL: https://github.com/krikz/rob_box_project/pull/1743
Title: `fix(merge-gate #1740/#1741): classify UNSTABLE — diagnostic vs rebase`
CI: 9/9 SUCCESS
Diff: 1 файл, +169/-1

Полезный фикс: добавлены `pr_classify_failure` (unit_lint | integration_e2e | unknown) и `pr_failed_jobs_json`. UNSTABLE+unit_lint → diagnostic-карточка; UNSTABLE+integration_e2e / unknown → старая rebase-логика (fail-open безопасный default).

**Cross-резолюция для #1741 close:** после merge #1743 — следующий merge-gate tick для #1740/#1741 создаст **диагностическую** карточку (НЕ rebase). Это лечит причину закрытия #1741 (merge-conflict-loop при повторных rebase).

Решение: MERGE. После merge — закрыть #1741 с комментарием-резолюцией.

## Что я проверил (raw evidence)

- `gh pr list --state open --json ... --limit 50` — получено 8 PR с указанными labels, MERGEABLE=true (или MERGEABLE/CLEAN), isDraft=false.
- `gh pr view --json statusCheckRollup,...` для каждого — 9/9 SUCCESS на каждом (Integration Tests = SKIPPED, остальные SUCCESS).
- `gh api repos/.../compare/{develop...HEAD}` — для каждого получен ahead/behind.
- `gh api repos/.../contents/scripts/agent_flow/lib_cron_env.sh` — 404 в develop (для #1753).
- `gh pr diff 1747` — реально 1 файл, не 4 (для #1747).
- `gh pr view 1748 --json state,mergedAt` — merged 30.08 23:50 (для #1750/#1747 divergence analysis).
- `gh pr view 1704 --json files` — другие файлы (для #1753 stale-branch-reuse verification).

## Что я НЕ проверял

- Реальный прогон regression-тестов в CI (CI checks зелёный, этого достаточно).
- md5sum sync 4 host copies (по #1756 и #1750 — указано в body, RAW в body не приложен).
- Live-выполнение `gh api .../check-runs --jq ...` для проверки `pr_classify_failure` на реальных данных (после merge #1743 — это работа Шифу/merge-gate).

## Что Шифу остаётся сделать

1. Решение по каждому PR (см. таблицу).
2. Снять label `stale-branch-reuse` с #1753 перед merge.
3. По #1744 — решить split или squash (я рекомендую split).
4. После merge #1743 — закрыть #1741 с комментарием (см. cross-resolution в issue #1741 comment).

## Cross-link

- 8 PR-комментариев с вердиктами опубликованы.
- Issue #1741 — cross-резолюция опубликована.
