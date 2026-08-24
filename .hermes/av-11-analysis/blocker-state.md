# AV-11 Blocker Analysis — t_307bae4a run #2373

**Дата:** 2026-08-24 (CEST)
**Воркер:** backend
**Ветка:** z-{agent}/1605-av-11-e2e-avatar-mixed-mode-quest-teleop
**HEAD:** 0f71ca1d (НЕ feature/avatar — коммит e2e harness, см. сессию t_c7761956)

## TL;DR

AV-11 не выполнима в текущем состоянии репо. Все prerequisites отсутствуют как merged code; e2e-process глобально приостановлен блокером #1586.

## Prerequisites (issue body: «Блокируется: AV-9, AV-10» + mermaid graph в AV-1)

| ID | title | state | merged to feature/avatar? | evidence |
|----|-------|-------|---------------------------|----------|
| AV-1 (#1595) | epic decomposition plan + script | PR #1606 OPEN | NO | `gh pr view 1606` |
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

A. Заблокировать task (kanban_block kind=dependency) — ждать AV-2..AV-10 + #1586 fix.
B. Начать AV-2..AV-6 самому — это НЕ моя задача (assignee у этих issue свой,
   scope-creep помимо собственной карточки запрещён процессом).
C. Запустить dry-run на существующем rob_box_telegram — частичное покрытие
   pytest из raw-evidence, не выполняет 7 acceptance steps.

## Рекомендация

A. kanban_block kind=dependency — task не выполнима. Я фиксирую raw-evidence в
worktree (`blocker-state.md` + этот коммит), комментирую issue #1605 с явным
указанием блокера, выхожу через `kanban_block`. Когда AV-2..AV-10 смерджат в
feature/avatar и #1586 закроют, диспетчер выдернет карточку обратно в ready.
