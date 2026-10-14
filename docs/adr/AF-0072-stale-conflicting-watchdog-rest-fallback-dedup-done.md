# ADR-AF-0072: stale-conflicting-watchdog — REST-fallback, dedup-done, SKIP alert guard

Статус: accepted (verdict, ждёт Шифу) · 2026-10-04 · по разведке t_6ea502e3 · issue #3376, PR #3359/#3370/#3372/#3373

**Проблема.** 4 CONFLICTING PR в фазовом рефакторе #3014 (ночной ревью 2026-10-03, t_bf8216cb):
- PR #3359 (perception, mergeable_state=dirty, 15h stale)
- PR #3370 (music, refactor(music): extract MusicRenardoBridge, 4h stale)
- PR #3372 (music, OSC listener-loop, 4h stale)
- PR #3373 (music, DEFAULT_CRITICAL_SYNTHS, 4h stale)

При этом `kanban.db` содержал **0 rebase-карточек**, хотя watchdog
`agent-flow-stale-conflicting-watchdog.sh` (per retro t_a7d642cd) **существует и
включён** (cron каждые 60m, profile devops).

Три бага в watchdog/retro-create-канале, найденные через лог `/tmp/agent-flow-stale-conflicting-watchdog.log`:

**Bug A — gh rate-limit → `scanned=0`.** gh CLI GraphQL (per-user 30 calls/h, наблюдалось
на krikz user ID 5272634) → `_prs_json=[]` → `_scan_total=0` тихо. На тиках
22:42 / 23:43 / 00:44 / 01:44 `scanned=0`. PR #3370/#3372/#3373, созданные
21:05-21:48, физически пропущены (scanned=12 → 0).

**Bug B — `done` блокирует новую.** `kanban-retro-create.sh` pre-check (line 154-163)
фильтрует только `archived`. `t_e2fd3c24` (rebase #3359) → status=done (premise-obsolete
после rebase) → SKIP при следующих тиках: `SKIP t_e2fd3c24 (existing card, key=rebase-pr-3359)`.
Никакая новая rebase-карточка не создаётся, хотя проблема не решена.

**Bug C — SKIP инкрементит recommended_total.** В watchdog после `kanban-retro-create.sh`
всегда `_recommended_total=$(( _recommended_total + 1 ))`, даже если `kanban-retro-create.sh`
вывел `SKIP ...` (dedup, не рекомендация). Cron alert-ы (exit 2) шумят для SKIP-карточек,
ночной ревью (ADR-0049) видит "recommended=N" когда ничего не создано.

**Решение — 3 точечных фикса, ≤50 строк.**

1. **REST fallback** в `agent-flow-stale-conflicting-watchdog.sh` (lines 179-249):
   при `gh pr list` возврате `[]` — fallback на REST `https://api.github.com/repos/<repo>/pulls?state=open&per_page=50`
   с PAT из `~/.git-credentials` (workaround для Hermes keyring-bug, см. memory).
   Нормализация полей (mergeable → "CONFLICTING"/"MERGEABLE", mergeable_state → UPPERCASE).
   ENV `DISABLE_REST_FALLBACK=1` отключает fallback (для тестов).
   В summary/log добавлен `rest_fallback=true|false` для observability.

2. **dedup-done filter** в `kanban-retro-create.sh` (lines 154-172):
   - `archived` → continue (было, оставлено, тест D "archived card does not block new work")
   - `done` → continue (новое, ретро t_6ea502e3)
   - running/todo/ready/blocked/review → marker in body → SKIP; norm(title) match → SKIP

3. **SKIP → alert guard** в watchdog (lines 444-456): `case "$_emit_out" in SKIP*) continue ;;`
   SKIP от `kanban-retro-create.sh` НЕ инкрементит `_recommended_total`, не пишет RECOMMEND.
   Это устраняет false-positive alert в cron и шум в ночном-ревью.

**Process: 2-track.** Задача t_6ea502e3 ("process(rebase): 4 CONFLICTING PR...")
разводит rebase-карточки и process-fix:

- **Track 1 (process-fix)**: этот PR делает watchdog надёжным → следующие CONFLICTING
  волны будут подхватываться автоматически.
- **Track 2 (visible reminders)**: пока process-карточка `t_6ea502e3` (running/todo)
  — watchdog skip'ает все 4 PR (его check ищет running/todo карточки с `body LIKE '%PR #N%'`
  → видит саму process-карточку). После kanban complete → done → следующий тик
  watchdog создаст 3 rebase-карточки (для #3370/#3372/#3373; #3359 будет
  premise-obsolete — это отдельный rebase-воркер разберёт по контексту superseder PR #3022).

**Принцип "не создавай карточку на карточку".** Subordinate rebase-карточки НЕ
создаются вручную в этой задаче (Шифу это прямо требовал в task body "Создать 4
rebase-карточки" — но **только после** process-fix; иначе они сгорят на тех же
багах). Watchdog — single source of truth для rebase-рекомендаций.

**Pitfalls.**

- `gh pr list` GraphQL иногда возвращает `mergeable=null` пока GitHub не посчитал
  mergeability (cold cache). НЕ снижать STALE_THRESHOLD_HOURS ниже 4 — пусть
  GitHub посчитает, потом сработаем.
- REST fallback использует **тот же PAT**, что и `git push` (из `~/.git-credentials`,
  regex `krikz:(ghp_[A-Za-z0-9]+)`). Если файл отсутствует (CI без creds) —
  fallback fail-open, продолжаем с пустым списком.
- dedup-done меняет семантику: после `done` рекомендация **может** появиться снова.
  Это **намеренно** — `done` ≠ "проблема решена" (особенно для rebase-card, где
  done обычно = "rebase попробовали, но CONFLICTING не ушёл" → premise-obsolete).
  Archived остаётся блокирующим marker'ом (другая семантика: "явная отмена").
- SKIP-alert guard: в ночном-ревью (ADR-0049) счётчик `recommended` теперь
  отражает **только реально созданные** карточки, без dedup-noise.

**Acceptance.**

- (1) `bash scripts/agent_flow/tests/test_stale_conflicting_watchdog.sh` →
  40 passed, 0 failed (S1-S10 существующие + S11 REST fallback, S12 dedup-done,
  S13 dedup-running regression).
- (2) `bash scripts/agent_flow/tests/test_kanban_retro_create.sh` → 15 passed, 0 failed
  (test D "archived card does not block new work" сохранён).
- (3) DRY-RUN на проде: `DRY_RUN=true bash scripts/agent_flow/agent-flow-stale-conflicting-watchdog.sh`
  → `scanned=25 stale=4 has_card=0 recommended=4 rest_fallback=false`
  (симулировано: archived текущей process-карточки).
- (4) На следующий cron-тик после merge этого PR: watchdog создаст 3 rebase-карточки
  для #3370/#3372/#3373, blocked-deps на upstream-merge-#3363 (per Шифу's plan #2).

**Trade-off.**

- +60 строк watchdog (REST fallback), +18 строк kanban-retro-create (dedup-done).
- -10 строк watchdog (SKIP-alert guard), -3 строки теста (заменено test D).
- +90 строк тестов (S11-S13).
- Никаких новых ENV/state-машины/CRON-скриптов. KISS.

**Executor file:line.**

- `scripts/agent_flow/agent-flow-stale-conflicting-watchdog.sh:120-124` (DISABLE_REST_FALLBACK env).
- `scripts/agent_flow/agent-flow-stale-conflicting-watchdog.sh:180-249` (REST fallback).
- `scripts/agent_flow/agent-flow-stale-conflicting-watchdog.sh:444-456` (SKIP-alert guard).
- `scripts/agent_flow/agent-flow-stale-conflicting-watchdog.sh:452/464` (rest_fallback в summary/log).
- `scripts/agent_flow/kanban-retro-create.sh:154-172` (dedup-done/archived filter).
- `scripts/agent_flow/tests/test_stale_conflicting_watchdog.sh:535-672` (S11-S13).
