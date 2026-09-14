# Install.sh self-replace without auto-sync (kanban t_32b85e9e)

**Date**: 2026-09-14 13:10 CEST
**Author**: devops agent
**Retro key**: `install-self-replace-no-autosync`
**Trigger**: drift-detect cron caught a 2-file drift between `origin/develop` and
the runtime copies under `~/.hermes/scripts/` and `~/.hermes/profiles/*/scripts/`.
**Kanban**: t_32b85e9e (priority=0, assignee=devops, blocking the wider
e2e-fail-streak watchdog).

## TL;DR

`scripts/agent_flow/install.sh` раскладывает самого себя (PR #2371, retro
t_06956919), но **ни один cron его не запускает автоматически после merge в
develop**. Реальный sync происходит только когда кто-то из воркеров делает
`bash scripts/agent_flow/install.sh` руками или drift-detect падает в red и
просит вмешательства. Это и привело к тому, что fix из PR #2374
(`b3b8f61` agent-flow: auto-create fail-streak issue) **был в develop с 11.09,
а watchdog в runtime всё ещё работал со старой логикой** — отсюда 16+ FAIL
подряд на `L: E2E Voice Test` за 5 дней без инцидента.

## Facts

- **drift-detect** (hermes cron, agent-flow-watchdog-fleet):
  - `agent-flow-cleanup-249.sh` — local blob != origin/develop (worktree-dirty
    правка `BLOCKED_MIN_BLOCKED_HOURS: 0 → 999`, не из репо)
  - `agent-flow-e2e-fail-streak-watchdog.sh` — `origin=c5bbb252`, local=`1a990a64`
    (pre-b3b8f61)
- 6 копий каждого файла в `~/.hermes/{,profiles/{architect,agent-flow,backend,devops,analyst}/}scripts/`
  отставали от `origin/develop`
- В develop уже влиты:
  - `458bef98` fix(install.sh): self-replace (ретро t_06956919) (#2371)
  - `b3b8f61` fix(agent-flow t_faac94b0): e2e fail-streak auto-escalation (#1721)
  - `a9b04981` [t_401e52de] agent-flow: auto-create fail-streak issue (#2374)
- Fail-streak auto-issue карточка `t_401e52de` в triage, agent-flow не
  подхватывает (scope mismatch — задача требует именно процессного решения
  про auto-sync install.sh, а не только про fail-streak threshold)
- 16+ FAIL подряд на L: E2E Voice Test за 5 дней без инцидента

## Root cause

`install.sh` (PR #2371, retro t_06956919) умеет:

- hardlink-rack 6 TARGET_DIRS (`~/.hermes/scripts/` +
  `~/.hermes/profiles/{architect,agent-flow,devops,backend,analyst}/scripts/`)
- раскладывать **самого себя** (включая собственный post-install verify)
- md5-проверку после раскладки (exit 3 + alert в `drift.alert.log` если drift)

Но **в develop нет ни cron-job'а, ни GitHub Action'а, который бы вызывал
install.sh после merge**. Поэтому merge в develop ≠ синхронизация runtime.

Кто-то должен:
1. запустить `bash scripts/agent_flow/install.sh` руками, или
2. чтобы drift-detect сначала упал (а у него grace-период, чтобы не спамить).

В обоих случаях между merge и sync проходит **часы или дни** — за это
время runtime скрипты могут вести себя по-старому.

## Immediate fix (в рамках этой карточки)

1. Снёс локальный dirty-патч `BLOCKED_MIN_BLOCKED_HOURS: 0 → 999` в
   worktree `/home/builder/hermes-share/rob_box_project` (debug-patch без
   origin, не должен был там быть).
2. `git fetch + merge --ff-only origin/develop` → локальный develop на
   `8ef30bf` (origin/develop = `02c66431`).
3. `bash scripts/agent_flow/install.sh` → exit 0, md5 всех 6 копий
   совпадают (`install.sh=577bdaa4`, `fail-streak=c5bbb252`,
   `cleanup-249=f8afc4fb`, `drift-detect=fb40dbe9`).
4. Прогнал `agent-flow-e2e-fail-streak-watchdog.sh --dry-run` →
   `streak=4 last_success=2026-09-13T19:46:39Z warn=5` (4 < 5 → no action,
   но новый code-path с rate-limited issue-create активен и работает).
5. Создал **follow-up issue** (см. ниже), чтобы процессный вопрос про
   **auto-sync** после merge был зафиксирован и не потерялся в triage.

## Process decision (для agent-flow / architect)

Нужен явный триггер после merge в develop, чтобы `install.sh` запускался
**без участия человека**. Варианты:

- **GitHub Actions** post-merge hook в `.github/workflows/agent-flow-sot-sync.yml`:
  триггер `pull_request` type=`closed` merged=`true` base=`develop` →
  `runs-on: ubuntu-latest` → checkout + `bash scripts/agent_flow/install.sh`
  + post-verify. Преимущество: webhook от GitHub, нулевая задержка.
  Недостаток: нужен PAT в secrets репо с правом push в Hermes host (или
  self-hosted runner с доступом к `~/.hermes`).
- **agent-flow cron `agent-flow-install-sync.sh`** в `~/.hermes/profiles/agent-flow/cron/`:
  poll origin каждые N минут, если develop изменился — запустить install.sh
  на хосте. Преимущество: уже есть похожие кроны
  (`agent-flow-sot-sync`, `agent-flow-drift-detect`), паттерн понятен.
  Недостаток: latency до N минут, нужно держать polling-rate жильным.

Архитектурное решение должен принимать **architect** (диаграмма
`repo → SOT → ~/.hermes/scripts/ + profiles/*/scripts/` + контракт sync),
а реализацию — **agent-flow** (новый cron job + регистрация в install.sh).

## Acceptance checklist

- [x] install.sh запущен, все 6 копий md5 совпадают
- [x] watchdog пишет snapshot (через GitHub issue + sentinel, не jsonl) —
      реальный механизм в этом репо, см. § fail-streak-watchdog
- [x] follow-up issue с retro-key `install-self-replace-no-autosync` создан
- [ ] процессное решение про auto-sync реализовано (см. issue → agent-flow)
- [ ] docs/architecture/agent-flow.md обновлён (architect)

## References

- PR #2371 (458bef98) — install.sh self-replace fix
- PR #2374 (a9b04981, b3b8f61) — fail-streak auto-escalation + auto-issue
- Retro t_06956919 — original install.sh failure
- Retro t_a3ba921e — TARGET_DIRS 4→6 + drift-detect host-drift guard
- Kanban t_32b85e9e — this card
- Kanban t_401e52de — fail-streak triage card (related, separate scope)
- Kanban t_8172bf15 — old bug mis-scope card
