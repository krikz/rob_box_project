# AV-11 Blocker Analysis — t_307bae4a, attempt 6 (run #2392)

**Дата:** 2026-08-24 ~23:11 (CEST)
**Воркер:** backend
**Ветка:** z-{agent}/1605-av-11-e2e-avatar-mixed-mode-quest-teleop
**HEAD:** de121244 (поверх 0f71ca1d, который поверх feature/avatar@955cbf58)
**Предыдущие попытки на этой карточке:** 1 (blocked 23:00), 2 (blocked 23:06), 3 (crashed 23:07), 4 (blocked 23:08), 5 (blocked 23:09) — все констатировали одно и то же.
**Изменение мира с попытки 1:** `feature/avatar` HEAD остался `955cbf58`. Единственное движение — PR #1606 (AV-1 plan script) MERGED в 21:11:57Z. Никакого кода AV-2..AV-10 в src/ не появилось, #1586 OPEN.

## TL;DR

AV-11 не выполнима в текущем состоянии репо. Все prerequisites отсутствуют как merged code; e2e-process глобально приостановлен блокером #1586.

## Prerequisites (issue body: «Блокируется: AV-9, AV-10» + mermaid graph в AV-1)

| ID | title | state | merged to feature/avatar? | evidence |
|----|-------|-------|---------------------------|----------|
| AV-1 (#1595) | epic decomposition plan + script | PR #1606 MERGED 2026-08-24T21:11:57Z | YES (docs only) | `gh pr view 1606 --json state,mergedAt` |
| AV-2 (#1596) | rob_box_supervisor package skeleton | OPEN | NO | `gh issue view 1596` |
| AV-3 (#1597) | FSM ModeManager | OPEN | NO | `gh issue view 1597` |
| AV-4 (#1598) | LockManager teleop_floor/voice_floor + 500ms dead-man | OPEN | NO | `gh issue view 1598` |
| AV-5 (#1599) | /avatar/state msgpack schema | OPEN | NO | `gh issue view 1599` |
| AV-6 (#1600) | supervisor_node.py monitor-mode | OPEN | NO | `gh issue view 1600` |
| AV-7 (#1601) | dialogue_node voice_input_mode param | OPEN | NO | `gh issue view 1601` |
| AV-8 (#1602) | meta-quest-api.md frame-types 0x30-0x33 | OPEN | NO | `gh issue view 1602` |
| AV-9 (#1603) | docker supervisor service Vision Pi | OPEN | NO | `gh issue view 1603` |
| AV-10 (#1604) | rob_box_telegram refactor on supervisor client API | OPEN | NO | `gh issue view 1604` |

## Hard evidence (raw)

1. `src/rob_box_supervisor/` does NOT exist:

```
$ ls src/rob_box_supervisor
ls: cannot access 'src/rob_box_supervisor': No such file or directory
```

2. Поиск supervisor-логики в репо:

```
$ find . -name "supervisor_node*" -o -name "mode_manager*" -o -name "lock_manager*"
(empty)
```

3. Поиск Avatar API в src/tests/docker:

```
$ grep -rln "avatar_supervisor\|AcquireFloor\|teleop_floor" src/ tests/ docker/
(empty)
```

4. Моя ветка НЕ diverged от feature/avatar — она ancestor:

```
$ git merge-base HEAD feature/avatar
0f71ca1d26e12db71efaaf53a925cf12fc7704f0  (== HEAD)

$ git rev-list --count HEAD ^feature/avatar
0   # HEAD — предок feature/avatar (в обратную сторону)
```

feature/avatar УЖЕ СОДЕРЖИТ мою ветку как предка, но не наоборот. Мой коммит
(e2e harness fix из t_c7761956) попал в feature/avatar ДО того, как разошлись AV-ветки.
Все AV-ветки (av-2..av-10) — ответвления от feature/avatar поверх моего HEAD,
но ни одна не merged.

5. e2e-process приостановлен (issue comment, 24.08):

```
agent-flow: ⏸️ e2e приостановлен: блокер #1586 — новый round не создаётся,
пока блокер открыт (ретро 11.08). Когда блокер закроют, ротация возобновится
автоматически.
```

6. #1586 (e2e-process crash) открыт, PR #1592 fix OPEN:

```
$ gh issue view 1586
state: OPEN
```

```
$ gh pr view 1592 --json state,title
state: OPEN
title: fix(agent-flow #1586 #t_d935096b): blocker_filter exclude no-e2e-required + agent-flow
```

## Почему нельзя выполнить «честный PASS»

Acceptance criteria #1605 (issue body) требуют:

1. Поднять `rob_box_quest` + `rob_box_supervisor` (mode:=active) + `rob_box_telegram` —
   rob_box_supervisor НЕ СУЩЕСТВУЕТ как пакет, ни monitor-версия, ни active.
2. WebXR-клиент, Quest grip + twist — `rob_box_quest` пакета тоже нет
   (ADR-0027 design-only, 0 строк кода в src).
3. `/avatar/state` → `mode=mixed, teleop_floor=quest, voice_floor=telegram` — нет
   /avatar/state топика, нет LockManager, нет ModeManager.
4. ≤ 500 мс safe-stop при Wi-Fi fail — LockManager не существует.
5. `/forward` → робот едет — Telegram ещё не отрефакторен на supervisor client API.

Без supervisor-кода я не могу:
- запустить pytest (тестов нет, кода нет)
- снять docker logs (контейнера supervisor нет)
- сделать скриншот mixed-state (mixed-state не существует)

«Без нового кода — результаты в issue-комментарий» означает «не пиши supervisor, а
собери evidence». Evidence нечего собирать — сценарий ещё не существует как
исполняемая система.

## Возможные действия

A. Заблокировать task (kanban_block kind=dependency) — то, что делали попытки 1/2/4/5. Риск: диспетчер может auto-escalate до triage, потому что 4 одинаковых dependency-блока подряд.
B. Начать AV-2..AV-6 самому — НЕ моя задача (assignee у этих issue свой, scope-creep помимо собственной карточки запрещён процессом).
C. Запустить dry-run на существующем rob_box_telegram — частичное покрытие pytest из raw-evidence, не выполняет 7 acceptance steps. Плюс нельзя поднять контейнер `rob_box_supervisor` — пакета нет.
D. Сменить kind на `capability` — «у backend-воркера нет SSH к Vision Pi + Main Pi, поэтому даже после merge AV-2..AV-10 живой e2e-прогон невозможен из этой среды». Это перенаправит карточку в human-triage, а не в бесконечный dependency-цикл.

## Рекомендация (для попытки 6)

D. `kanban_block(kind='capability')` с CRP. Аргументы:
- 4 dependency-блока подряд (попытки 1, 2, 4, 5) дают одинаковый вывод, который
  диспетчер по правилам unblock-loop-detection может сам перевести в triage.
  Сменить kind — это и есть тот сигнал, что воркер осознал, что dependency-loop
  бесперспективен.
- Карточка требует live-прогона на Vision Pi (10.1.1.21) + Main Pi. У воркера
  есть только локальный worktree; docker exec / ssh в этой сессии не работают
  (проверено в ранних попытках — `kanban_comment` t_3a2fcb55 / t_325dd611 из
  предыдущих сессий).
- Acceptance «500ms safe-stop» + «mixed-state /avatar/state» физически нельзя
  проверить без работающего supervisor-пакета, который не в src/.

Артефакт в WIP `de121244`: эта таблица + raw-evidence по 6 пунктам + история 5
попыток + рекомендация D. Push выполнен.
