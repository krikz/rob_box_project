# ADR-0095 — PR pollution detection gate

- **Status**: ACCEPTED
- **Date**: 2026-09-14
- **Supersedes**: частично ADR-0055 §3 (только про drift; добавляем pollution-detection)
- **Linked issues**: #2444
- **Related**: #2038 (closed — drift-guard predecessors), #2391 (worktree-drift ongoing), ADR-0045, PR #2443 (worker_post_flight)
- **ADR-number**: 0095 (collision-fix, 2026-09-15: см. issue #2582 — сосед `0095-znakomyi-identity-seam.md` получил тот же номер и переехал в ADR-0106; этот ADR первый по merge-time, коммит `42cd89be` (PR #2447))

## Context

Issue #2444 (товарищ Шифу, 2026-09-14): в develop открыто 6 PR, 5 из которых содержат мусорные файлы, не относящиеся к issue. Юзер не может их закрыть, потому что в diff мусор не от issue.

### Raw-evidence из диагностики (см. `analysis/diagnose-2444-pr-pollution.md`)

- Pollution-файлы (`hailo_smoke.py`, `start_vision_hailo.sh`, `voice_capture_*.test.ts`, `evidence/issue-2003-pregenerate-metrics.md`) **отсутствуют** в `origin/develop` (проверено `git cat-file -e`).
- Они сидят прямо в HEAD PR-веток как leftover от wip-коммитов прошлых эпиков (#2349 hailo, #2003 pregenerate), оставшихся после squash-merge PR #2352.
- `git merge-base origin/develop pr-{2420,2429,2431,2443}` = `a77ba09e` — коммит, идущий **до** PR #2352. То есть pollution попала в эти ветки после rebase на свежий origin/develop с merge #2352; ребейз принёс мусорные файлы в HEAD.
- `validate_pr_scope.sh` (`scripts/agent_flow/`) **уже ловит pollution** через трёхточечный diff (`BASE...HEAD`) — файлы попадают в diff, потому что есть в HEAD, а в merge-base их нет. **Реальная причина, почему PR не блокируются**: воркеры создают PR без `PR_ALLOWED_PREFIXES`, скрипт уходит в INFO-режим (exit 0 всегда).

### Гэпы

1. **Pre-merge (working tree)**: validate_pr_scope текущая версия сравнивает только закоммиченный HEAD. Воркер, который сделал `rebase origin/develop` и видит в `git status` мусорные файлы **до** `git add` — пройдёт мимо текущей проверки.
2. **Coverage в CI gate**: pre-PR vоркеры не запускают validate_pr_scope в pre-merge режиме (нет автоматического вызова из `kanban-report-write.sh` / `worker_pre_flight.sh`).
3. **Документация**: правило «после rebase проверь diff» не записано нигде явно — только в issue-комментариях.

## Decision

### 1. `validate_pr_scope.sh` расширяется режимом `PR_SCOPE_MODE=pre-merge`

- Трёхточечный diff (по умолчанию, post-PR gate) — без изменений.
- Pre-merge (новый): двухточечный `git diff BASE_REF` (без HEAD — это индекс + working tree) + untracked через `git ls-files --others --exclude-standard`. Воркер включает явно перед push.
- Режим по умолчанию OFF → совместимо с существующим CI usage.
- Тесты: `scripts/agent_flow/tests/test_validate_pr_scope.sh` (добавлены сценарии I и J — raw-evidence PASS 2026-09-14).

### 2. Skill `bundled/worker-rebase-pollution-check.md`

Шаги для воркера:
1. `git fetch origin develop`
2. (опц.) `git rebase origin/develop` — зафиксировать конфликт
3. `PR_SCOPE_MODE=pre-merge bash scripts/agent_flow/validate_pr_scope.sh origin/develop` — должен exit 0
4. Если exit 1 → `git checkout origin/develop -- <файлы>` (удалить мусор из HEAD) → `git commit --amend --no-edit`
5. `git push --force-with-lease`

### 3. AGENTS.md обновляется

Добавляется правило: **«rebase может принести чужие файлы — проверяй diff (validate_pr_scope в pre-merge)»**.

## Rationale

- **Не чиним воркеров за них**: не делаем force-push в чужие PR-ветки — это ответственность владельца ветки (товарища worker'а). Issue #2444 «5 из 6 PR содержат мусор» лечится per-PR отдельными фиксами, не этим ADR.
- **Минимальный rule**: только pre-merge режим в существующем скрипте + 2 теста + skill + AGENTS.md правило. Не трогаем worker_pre/post_flight.sh — это PR #2443.
- **Расширяемость**: pre-merge режим можно вызывать из любой части agent-flow (pre_flight, post_flight, ручной pre-PR-check), не блокируя существующие usage.

## Trade-offs

- **PRO**: ловит pollution до push (`force-push avoidance`); существующий post-PR gate не сломан; skill + AGENTS.md делает ритуал явным.
- **CON**: добавляет ещё одну точку запуска скрипта в worker'е — нужно покрыть AGENTS.md чтобы не забыли.
- **ALTERNATIVE-A**: лимитировать HEAD через pre-receive hook в remote — отвергнуто, это организационная мера, не для воркера.
- **ALTERNATIVE-B**: закрыть мусорные PR автоматически через cron — отвергнуто, противоречит принципу «не чинить баги руками» (CONTRIBUTING.md).

## Verification

- **Тесты**: `bash scripts/agent_flow/tests/test_validate_pr_scope.sh` — 11/11 PASS, raw-evidence см. §Execution #1 этой задачи (kanban t_e43619b6).
- **Real-world repro**: для каждого из 5 мусорных PR (`#2420`, `#2429`, `#2431`, `#2414`, `#2373`) — `PR_SCOPE_MODE=pre-merge PR_ALLOWED_PREFIXES="<scope>" bash scripts/agent_flow/validate_pr_scope.sh origin/develop` корректно перечисляет pollution. (Запуск оформлен в комментарии к issue #2444.)
- **PR pollution detection plan**: see Phase 4 + `analysis/diagnose-2444-pr-pollution.md`.

## Execution

Kanban t_e43619b6, commit `<HEAD>` ветки `z-{agent}/2444-bug-process-5-6-open-pr-race-condition-rebase-2349`. WIP-коммиты:
- `2723dacb` — `wip(diagnose #2444)`: raw-evidence phase 1.
- `<next>` — `feat(validate_pr_scope): pre-merge mode + tests I/J`.
- `<final>` — `docs(adr-0095): PR pollution detection gate`.
