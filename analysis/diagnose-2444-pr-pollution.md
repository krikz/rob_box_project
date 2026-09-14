# diagnose-2444-pr-pollution.md

WIP-коммит Phase 1: raw-evidence диагностика issue #2444.

**Task**: kanban t_e43619b6, devops-worker.
**Issue**: #2444 — "5 из 6 OPEN PR содержат мусорные файлы, не относящиеся к задаче — race-condition при rebase".

## Что выяснили (raw)

Команды и реальный вывод зафиксированы ниже.

### 1. Загружены 6 PR из origin и посчитан merge-base:

```
pr-2431 merge-base=a77ba09e ahead-of-develop=3
pr-2429 merge-base=a77ba09e ahead-of-develop=3
pr-2420 merge-base=a77ba09e ahead-of-develop=2
pr-2414 merge-base=7dcf3342 ahead-of-develop=2
pr-2373 merge-base=5ccabc26 ahead-of-develop=2
pr-2443 merge-base=a77ba09e ahead-of-develop=6
```

### 2. Pollution-файлы *отсутствуют* в origin/develop:

```
docker/vision/vision-hailo/hailo_smoke.py                        -> in origin/develop: NO
docker/vision/vision-hailo/start_vision_hailo.sh                 -> in origin/develop: NO
src/rob_box_quest/webxr_client/tests/voice_capture_break_detection.test.ts -> in origin/develop: NO
src/rob_box_quest/webxr_client/tests/voice_capture_sustained_session.test.ts -> in origin/develop: NO
```

=> Pollution **НЕ попала** из develop через rebase — она сидит прямо в HEAD PR-веток
(т.е. была закоммичена этими воркерами в их собственных коммитах, мимо issue-scope).

### 3. Source of pollution — НЕ race-condition, а miscommit:

`git log --diff-filter=A --follow` для hailo_smoke.py показывает только PR #2352 (#2349),
но **в origin/develop нет** этих файлов (см. выше). Это означает:

- `82dcd0a0` (PR #2352 ADR-0089 Phase 1 PoC, 2026-09-14) добавил файлы в свою ветку.
- Ветка была смёржена squash'ем → в origin/develop пошли *только* hailo_node.py и Phase-1
  файлы. Файлы `hailo_smoke.py` + `start_vision_hailo.sh` лежат в **исходных wip-коммитах
  PR #2352**, но видимо смёржены были с конфликтом/выкинуты → они остались висеть в
  tip'ах `wt/t_d4ce35e3`, `wt/t_59b76eee`, `wt/t_3a6be943` (== PR #2420/#2431/#2414)
  как stale content.

### 4. validate_pr_scope.sh УЖЕ ловит pollution:

Запуск:
```
git checkout pr-2429 -- .   # bare tree pollution check
PR_ALLOWED_PREFIXES="src/rob_box_perception/,docker/main/" \
  bash scripts/agent_flow/validate_pr_scope.sh origin/develop
```

Возвращает "FAIL: 4 of 4 files in diff vs origin/develop are out-of-scope" с
списком `hailo_smoke.py`, `start_vision_hailo.sh`, оба `voice_capture_*.test.ts`.

=> Утверждение из issue #2444 про то, что "validate_pr_scope.sh не видит pollution
из-за race-condition" — **НЕ подтверждается**. validate_pr_scope.sh использует
трёхточечный diff `BASE...HEAD`, который показывает только дельту HEAD относительно
merge-base. Pollution попадает в этот diff ровно потому, что она в HEAD, не в base.

**Реальная причина, почему 5 PR с мусором не закрыты**: никто из воркеров не
передавал `PR_ALLOWED_PREFIXES` при `gh pr create` (validate_pr_scope.sh в
INFO-режиме выдаёт 0).

## Что делаем (минимальный fix в этом PR)

1. **validate_pr_scope.sh**: расширяем режим `pre-merge` (сравнение working tree vs
   origin/develop — ловит даже если воркер ещё не сделал commit). По умолчанию
   pre-merge ВЫКЛЮЧЕН (чтобы не сломать dev-режим). Воркеры включают через env
   `PR_SCOPE_MODE=pre-merge`.
2. **tests/test_pollution_detection.sh**: регрессионный тест — создаём branch от
   develop с легитимным файлом + 4 мусорными; validate_pr_scope.sh должен
   вернуть FAIL.
3. **ADR-0095-pr-pollution-detection.md**: фиксируем контракт, link на issue #2444.
4. **Skill bundled/worker-rebase-pollution-check.md**: ритуал «после rebase
   запусти validate_pr_scope.sh в pre-merge режиме».
5. **AGENTS.md**: правило «rebase может принести чужие файлы — проверяй diff».

## Чего НЕ делаем в этом PR

- Не чистим pollution из чужих PR — это отдельные фиксы для каждого автора
  (только force-push в их ветки, и моя ветка 41b5805a сама = wt/t_d4ce35e3,
  pollution в HEAD, чистка в другой карточке).
- Не правим worker_pre_flight.sh / worker_post_flight.sh — это PR #2443 в работе.
