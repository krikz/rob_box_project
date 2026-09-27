# 2026-09-14 — PR cleanup sweep (issue #2451)

## TL;DR

Аудит всех open PR в `krikz/rob_box_project` показал **0 open PR**. Дублей
закрывать нечего, pollution revert'ить нечего — большая часть работы уже
закрыта через PR #2447 (validate_pr_scope pre-merge mode + ADR-0095 +
rebase-pollution-check skill), merged в develop как `42cd89be`.

Этот отчёт фиксирует **raw-evidence** состояния и оставляет
`pr-cleanup-checklist` skill, чтобы будущий sweep не начинался с нуля.

## Acceptance (из issue #2451)

| Фаза | Acceptance | Статус | Evidence |
|---|---|---|---|
| Phase 1 | Список всех open PR через REST `repos/.../pulls?state=open` | DONE | `[]` (count 0); `search/issues?q=is:pr+is:open` → total_count 0 |
| Phase 1 | Per-PR: `gh pr view <N>` + `gh pr diff --name-only` + `git log --merges --grep="#N"` | N/A | 0 PR для аудита |
| Phase 2 | Закрыть дубли (если issue merged) | N/A | 0 PR |
| Phase 3 | Revert pollution коммитов | N/A | 0 PR; pollution в develop отсутствует (raw-evidence ниже) |
| Phase 4 | Документация: sweep-отчёт + skill | DONE | этот файл + `.agents/skills/pr-cleanup-checklist/SKILL.md` |
| Phase 4 | post-merge sanity в `validate_pr_scope.sh` | DEFER | требует отдельной задачи (см. §Open questions ниже) |
| Phase 5 | `gh pr list --state open` = 0 дублей | DONE | REST → `[]` |
| Phase 5 | Каждый open PR `gh pr diff --name-only` ⊂ issue-scope | N/A | 0 PR |
| Phase 5 | Issue #2444 и #2391 закрыты | PARTIAL | #2444 — Шифу поставил `needs-e2e`, ждёт e2e-process (не моя зона); #2391 — отдельно |

## Raw-evidence (2026-09-14 21:17 UTC)

### 1. Open PR = 0

```
$ curl -sS -H "Authorization: token <gh>" \
    "https://api.github.com/repos/krikz/rob_box_project/pulls?state=open&per_page=100"
[]

$ curl -sS -H "Authorization: token <gh>" \
    "https://api.github.com/search/issues?q=is:pr+is:open+repo:krikz/rob_box_project&per_page=30"
{"total_count": 0, ...}
```

> GraphQL `gh pr list --state open` падает с
> `API rate limit already exceeded for user ID 5272634` —
> используем REST (`gh api` или `curl`).

### 2. Все PR, упомянутые в issue #2451 как «open с pollution», уже closed

Проверено через `state=all&sort=updated&direction=desc` (100 PR):

| PR | state | merged_at | Title (сокращённо) |
|---|---|---|---|
| #2429 | closed | 2026-09-14T21:03:20Z | fix(docker #2425): drop phantom MICRO_ROS_AGENT_TAG |
| #2363 | closed | 2026-09-14T21:03:18Z | feat(perception): VisionEvent field parity |
| #2373 | closed | 2026-09-14T21:03:06Z | wip(architect t_c44a7cf2): verdict issue #2003 |
| #2450 | closed | 2026-09-14T21:02:50Z | perf(e2e): gh_pr_state_by_head bulk-cache |
| #2449 | closed | 2026-09-14T21:02:44Z | docs(adr): AF-0096 worktree cleanup |
| #2420 | closed | 2026-09-14T21:02:30Z | test(webxr_client): getUserMedia sustained session |
| #2431 | closed | 2026-09-14T21:02:24Z | analysis(kanban-t_59b76eee): RID 34864016777 |
| #2414 | closed | 2026-09-14T21:02:20Z | test(t_3a6be943): pytest markers pregenerate_latency |
| #2445 | closed | 2026-09-14T20:51:05Z | fix(e2e-process): G2 auth-check |
| #2448 | closed | 2026-09-14T20:30:52Z | fix(dj): single source of arrangement craft (#2441) |
| **#2447** | **closed (merged)** | **2026-09-14T19:45:45Z** | **fix(process #2444): validate_pr_scope pre-merge mode + ADR-0095 + rebase-pollution-check skill** |
| #2443 | closed (merged) | 2026-09-14T19:44:53Z | bug(process): pre/post-flight rebase protocol (issue #2438) |
| #2428 | closed (merged) | 2026-09-14T16:31:45Z | ADR-0094 + phantom env linter — recover lost ADR-0031 |

Все «дубли» (#2429 ↔ #2428, #2431 ↔ #2437, #2420 ↔ #2373) уже разрешены:
- **#2428 ADR-0094 (merged 3fefcc0d)** закрывает работу #2425 и #2426 →
  #2429 больше не нужен (superseded).
- **#2437 compose_music без нот (merged 6e21a6e3)** закрывает работу #2431 →
  #2431 superseded.
- **#2003 (verdict, закрыт в #2373)** — verdict issue, не feature; #2420
  (test webxr getUserMedia) — отдельная фича, не дубль.

### 3. Pollution в develop отсутствует

Все 4 «pollution файла» из issue #2444 проверены через
`git ls-tree -r origin/develop --name-only`:

| Файл из issue #2444 | Фактически в develop | Вердикт |
|---|---|---|
| `docker/vision/vision-hailo/hailo_smoke.py` | `docker/vision/scripts/vision-hailo/hailo_smoke.py` | **другая директория**, но файл legitimate: пришёл из PR #2352 (ADR-0089 Phase 1 PoC, commit `82dcd0a0`) — feature, не pollution |
| `docker/vision/vision-hailo/start_vision_hailo.sh` | `docker/vision/scripts/vision-hailo/start_vision_hailo.sh` | то же — legitimate feature PR #2352 |
| `src/rob_box_quest/webxr_client/tests/voice_capture_break_detection.test.ts` | **отсутствует** | test переименован/удалён; актуальные тесты — `voice_capture.test.ts`, `voice_capture_wiring.test.ts` (legitimate) |
| `src/rob_box_quest/webxr_client/tests/voice_capture_sustained_session.test.ts` | **отсутствует** | то же |

### 4. PR #2447 (42cd89be) — фикс #2444 уже в develop

```
commit 42cd89be1f35f4c32a4ce888ed8d57f40a84c3c0
Author: Denis <krikzster@gmail.com>
Date:   Mon Sep 14 22:45:41 2026 +0300

    fix(process #2444): validate_pr_scope pre-merge mode + ADR-0095 +
    rebase-pollution-check skill (#2447)

    - validate_pr_scope.sh: новый PR_SCOPE_MODE=pre-merge
    - test_validate_pr_scope.sh: сценарии I и J (raw-evidence pre-merge
      pollution DETECTED). 11/11 PASS.
    - docs/adr/0095-pr-pollution-detection.md: ADR для PR pollution gate
    - .agents/skills/rebase-pollution-check/SKILL.md: 5-шаговый ритуал
    - analysis/diagnose-2444-pr-pollution.md: raw-evidence диагностика

 .agents/skills/rebase-pollution-check/SKILL.md     | 106 +++
 analysis/diagnose-2444-pr-pollution.md             |  93 +++
 docs/adr/0095-pr-pollution-detection.md            |  72 +++
 scripts/agent_flow/tests/test_validate_pr_scope.sh |  52 +++
 scripts/agent_flow/validate_pr_scope.sh            |  29 ++-
 5 files changed, 351 insertions(+), 1 deletion(-)
```

Issue #2444 — Шифу поставил `needs-e2e` (см. issue comments):
- `agent-flow: 🔄 ретро-путь: PR #2447 смержен, но PASS-доказательства
   не найдены. Поставлен needs-e2e — e2e-process возьмёт issue в ротацию.`
- `agent-flow: ⏸️ e2e приостановлен: блокер #2344 — новый round не создаётся,
   пока блокер открыт. Когда блокер закроют, ротация возобновится.`

> Закрытие #2444 — НЕ моя зона (товарищ Шифу / e2e-process). Согласно
> CONTRIBUTING.md «не фиксить баги руками».

### 5. validate_pr_scope на текущей ветке

```
$ PR_SCOPE_MODE=pre-merge bash scripts/agent_flow/validate_pr_scope.sh origin/develop
[validate_pr_scope] mode=pre-merge: working tree + index + untracked vs origin/develop
[validate_pr_scope] OK: no file changes between origin/develop and HEAD
```

Working tree clean. Ветка `z-{agent}/2451-chore-process-audit-cleanup-open-pr-issue-merged-r`
на коммите `5158bad2` (develop HEAD), никаких новых файлов не добавляет.

## Что сделано в этом PR

1. **`docs/process/2026-09-14-pr-cleanup-sweep.md`** — этот отчёт.
2. **`.agents/skills/pr-cleanup-checklist/SKILL.md`** — новый skill
   с 5-шаговым ритуалом для следующего sweep'а. Дополняет
   `.agents/skills/rebase-pollution-check/SKILL.md` (из #2447), который
   ловит pollution при rebase.

## Open questions / Deferred

### Post-merge sanity в validate_pr_scope.sh

ADR-0095 реализовал **pre-merge** режим (working tree vs BASE_REF).
Phase 4 issue #2451 просил ещё **post-merge sanity** — но:

- Это **другая задача**: post-merge — это когда PR уже в develop и
  нужен cron, который сканирует `origin/develop` и ищет регрессии
  (файлы вне scope последнего PR, raw-evidence).
- Post-merge gate требует отдельного ADR и отдельного теста (нетривиально
  — у post-merge нет explicit BASE_REF, нужно сравнивать с предыдущим
  merge commit'ом и парсить commits).
- Это **не блокер** для issue #2451 — текущий pre-merge gate закрывает
  основной сценарий (воркер пушит мусор).

Решение: **отложено** до отдельной карточки. Если товарищ Шифу захочет
post-merge gate — это будет новая issue `process: post-merge regression
detector` и отдельный worker (architect для ADR, devops для cron).

### Issue #2391 (wt/* ветки)

ADR-0045 (worktree-drift-guard). Эта задача упоминает #2391 в Phase 5
acceptance, но wt/* ветки — это **worktree**, не remote branches, и
`gh pr list --state open` их не показывает. Закрытие #2391 — отдельная
работа (force-delete wt/ веток после merge через pre-flight cleanup).
Сейчас **не** в scope этой задачи.

## Канбан-связанное

- Kanban `t_efe62f8f`: эта задача, devops-профиль.
- Источник: issue #2451 (открыт), agent-flow-triage создал.
- Связанные воркеры: `t_e20579f3` (rebase #2429), `t_1d9d4682` (CI UNSTABLE
  #2443), `t_c6850330` (gh rate-limit). Все — closed до того, как эта
  задача была создана; пересечений нет.

## Не делал (по правилам)

- **Не закрывал** issue #2444 (needs-e2e → e2e-process, не моя зона).
- **Не merge'ил** PR (только Шифу по AGENTS.md).
- **Не правил** AGENTS.md / install.sh / secrets.
- **Не force-push'ил** чужие ветки (их владельцы — другие воркеры).