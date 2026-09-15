# ADR-0022 Amendment — best-effort stale-candidate alert as valid completion path

> **Статус:** Proposed (требует owner-approval товарища Шифу).
> **Дата:** 2026-09-15.
> **Автор:** architect (Hermes Agent, kanban t_fc6da8b6).
> **Родительский ADR:** [0022 § 2 «Инвариант завершения»](0022-process-e2e-done-gates.md).
> **Мотивация:** issue [#2394](https://github.com/krikz/rob_box_project/issues/2394) (devops, t_6927a4c5, blocked) — добавление алерта в `#devops`-канал при stale-candidate + priority:high + voice/operator/bug. Существующий § 2 invariant не имеет строки «alert fired», что делает эту функциональность вне ADR-контракта. Без amendment нельзя ни принять #2394 в работу, ни отказать ему в принятии — оба варианта нарушают либо архитектурный канон, либо процесс-инвариант issue.

## 1. Бизнес-проблема

11.09–14.09.2026 в репо было 11 orphan-stale issues (`docs/retros/orphan-stale-no-agent-assign-2026-09-14.md`): 4 из них — подтверждённый P0 voice-bugs (#2131/#2132/#2136/#2137). Медиана created → agent-assigned = 79.8 ч при SLA для `priority:high` = 2 ч (40× перерасход). Владелец репо **узнавал** о P0-bug не из push-уведомления, а из ретро-таблицы через 80 часов.

Источник gap: ADR-0022 § 2 закрытие «успешно» проходит при `e2e-done + PR MERGED + gh pr checks OK + robot_pre-flight + PR base == develop`. Никакого канала, по которому stale-candidate > N часов должен **дойти до человека**, в инварианте нет. Issue #2394 закрывает этот gap **через отдельный on-call алерт** (не через жёсткое условие в § 2), но без ADR-amendment этот алерт **выходит за рамки канона** — что недопустимо по правилу ADR-0018 (любая процесс-функция, влияющая на close/invariant, должна быть в ADR).

## 2. Что предлагается внести в § 2

### 2.1 Текущий § 2 (ADR-0022:29–39, до amendment)

```diff
 issue may close  <=>  issue has e2e-done produced by e2e PASS
                        AND (issue_body acceptance_check = scenario_file.expected_tool_calls + must_not_call)
                             OR (PR adds .json to .github/e2e/scenarios/ with explicit acceptance)
                        AND living PR gh pr checks <N> == []    # no FAILURE
                        AND robot_pre-flight healthy           # 10-minute silence window
                        AND selected agent PR is MERGED
                        AND PR base is develop
```

### 2.2 Предлагаемая новая строка (вариант A — **рекомендую**)

```diff
                        AND (alert fired OR alert_fired=best_effort_failed with logged error)
```

**Семантика строки:**

- `alert fired` — `stale-candidate-alerter.sh` (issue #2393 → новая фича) успешно доставил push-уведомление в `#devops` канал. Это означает, что **до** момента `close` кто-то из owners/deps-on-call получил сигнал «этот issue закрывается, последний шанс вмешаться».
- `alert_fired=best_effort_failed with logged error` — инфра доставки (GitHub webhook / Telegram API / Slack API / Discord webhook) вернула ошибку. Это **не блокер закрытия**, но требует записи в `agent-flow-error` log + manual runbook (см. `docs/runbooks/stale-candidate-alerter-failure.md`, который должен завести devops).
- **Никакого `#devops-канал реально нет` → alert cannot fire** → алерт **не блокирует** (best-effort). Деталь ниже в § 4.

### 2.3 Альтернативы (вынесены на голосование owner'а)

#### Вариант B — hard-blocker

```diff
-                       AND (alert fired OR alert_fired=best_effort_failed with logged error)
+                       AND alert fired
```

**Trade-off:**

- ✅ Симметрично остальным строкам § 2 (все — hard conditions).
- ❌ Если инфра-канал `#devops` упал, **никакие issue не закрываются**. Это превращает алерт в **общий gate на close**, что не входит в acceptance criteria #2394 («алерт сработал до GATE-2 close», не «без алерта не закрывать»).
- ❌ Требует SLA на канал доставки и fallback (alert escalation timeout) — отдельный ADR.
- ❌ Если канал `#devops` так и не будет заведён в инфре (что сейчас и наблюдается — см. parent-context t_6927a4c5: «реального infra-приёмника «#devops» в репо не существует»), то § 2 становится **невыполнимым**, и ADR-0022 ломается.

#### Вариант C — warning

```diff
# ничего не менять в § 2; вынести в отдельный ADR «notification hygiene»
```

**Trade-off:**

- ✅ Не трогает канон.
- ❌ ADR-0018 контра: «голословная метка без raw-evidence» недопустима. Alert, который не зафиксирован в invariant, невозможно аудировать: где доказательство, что алерт реально сработал перед close?
- ❌ Issue #2394 не сможет дать acceptance без изменений в § 2 (приёмник требует enforcement, а не «рекомендацию»).
- ❌ Это фактически **отказ** от #2394 под видом «вынести в отдельный ADR». Затягивание без operational impact.

## 3. Почему **A**

| Критерий | A (best-effort + audit) | B (hard-blocker) | C (warning) |
|---|---|---|---|
| Решает business-проблему (P0 voice-bug → владелец узнаёт за ≤ 5 мин) | ✅ через rate-limit + audit | ✅ но только если канал жив | ⚠️ частично (нет enforcement) |
| Минимальный diff | ✅ +1 строка в § 2 | ✅ +1 строка | ❌ отдельный ADR |
| Совместим с ADR-0018 (честный FAIL лучше красивого PASS) | ✅ audit-trail через `agent-flow-error` | ⚠️ требует fallback chain (over-engineering) | ❌ нарушает (no enforcement) |
| Совместим с текущим отсутствием `#devops` инфра-канала | ✅ best-effort + log → не блокирует | ❌ § 2 становится невыполнимым | ✅ |
| Backwards-compat с уже merged PR | ✅ только NEW e2e-done events (staged rollout, как GATE-2 в § 7.2) | ⚠️ требует full deploy перед применением | ✅ ничего не ломает |
| Симметрично с GATE-3 (CI-blocking, § 4.3) | ⚠️ не симметрично, но GATE-3 = CI check (детерминирован), alert = сеть (не детерминирован) | ✅ симметрично, но за счёт SPOF | ⚠️ |
| Force-triage (Phase 4, `z-{devops}/25a2b395-...`) не ломается | ✅ alert живёт параллельно, не конкурирует | ⚠️ если алерт жёстко блокирует → force-triage невозможен | ✅ |

**Аргументы за A (выбрано):**

1. **Сетевая функция ≠ детерминированная.** GATE-3 проверяет `gh pr checks <N>` — это **локальный** CI state, не сетевой. Alert доставка зависит от GitHub webhook / Telegram / Slack / Discord uptime, которые архитектурно **внешние** и **нашими руками не контролируются**. Жёсткое условие в § 2 для недетерминированной сети — это SPOF для всего close-path.
2. **«Honest FAIL» лучше «forced close».** Если канал упал и мы блокируем close — issue копится. Если канал упал и мы логируем + продолжаем — issue закрывается, и runbook даёт owner'у механизм ручного вмешательства. Это согласуется с ADR-0018: «факт зафиксирован, ничего не замолчано».
3. **Реальность инфры.** Из parent-context t_6927a4c5: «реального infra-приёмника «#devops» в репо не существует». Вариант A **развязывает** § 2 invariant от факта существования канала: если канал живёт — лучше (alert доходит), если нет — хуже не стало (issue закрывается + есть log). Вариант B делает § 2 **зависимым** от инфры, которой нет.
4. **Rate-limit + audit.** «best-effort» не значит «без контроля»: `stale-candidate-alerter.sh` (issue #2393) уже задаёт `rate-limit ≤ 1 алерт/issue/сутки` и `авто-отключение при появлении agent:*`. То есть у нас уже есть anti-spam + auto-deactivation. Best-effort остаётся **наблюдаемым**.
5. **Staged rollout** совместим с GATE-2 (там тоже staged: «только NEW events», § 7.2). Применяется тот же паттерн: amendment вступает в силу для **новых** e2e-done событий после merge этого ADR, не retroactively.

**Аргументы против B и C:**

- **B**: best-effort без hard-blocker — это ложная дихотомия. ADR-0022 § 2 уже имеет **3 hard-blocker'а** (acceptance.json, gh pr checks, robot_pre-flight) и **2 informational** (stale-candidate timer — тоже по сути best-effort). Добавить ещё один informational — это **консистентно**, не «слабее».
- **C**: «вынести в отдельный ADR» — это перенос решения, а не решение. Issue #2394 уже 13 ч без движения, owner-sync на PM (t_cf60f006) уже завершён, recommendations (b)(c) в retro §5 ждут реализации. Откладывание = потеря momentum.

## 4. Acceptance для devops-реализации (после owner-approval)

После merge этого amendment в develop, devops-карточка (issue #2393, t_6927a4c5 parent, или новый child) берёт в работу `stale-candidate-alerter.sh`:

1. **Триггер**: `stale-candidate` AND `priority:high|critical` AND `bug` AND (`voice` OR `operator` OR `quest` OR `llm` OR `tts` OR `stt`) AND (`created → now` > 60 мин) — воркер `agent:devops` подтверждает порог в #2393 acceptance.
2. **Действие**: попытка доставки в `#devops`-канал (GitHub Discussion webhook / Telegram / Slack / Discord — на выбор devops-профиля через `.env`).
3. **Rate-limit**: ≤ 1 алерт / issue / сутки (дедуп через issue-comment с уникальным marker'ом `agent-flow-stale-alert:<issue>:<sha>`).
4. **Auto-deactivation**: если в течение 5 мин после алерта на issue появляется `agent:*` метка — алерт-cron пропускает эту issue до следующего цикла (anti-spam).
5. **Failure handling**: при ошибке доставки — запись в `agent-flow-error` log с полным контекстом (issue URL, target channel, error code, retry count). Manual runbook в `docs/runbooks/stale-candidate-alerter-failure.md` (отдельная devops-карточка).
6. **Audit**: `agent-flow-merge-gate.sh` перед `gh issue close <N>` должен проверить (a) был ли alert attempt на эту issue, (b) результат (success | best-effort-failed-with-log | not-applicable-no-trigger). Результат пишется в issue-comment перед close.
7. **Backwards-compat**: только для NEW e2e-done events после даты merge этого ADR (staged rollout, как GATE-2 § 7.2).
8. **Stale-candidate → close таймер**: § 4.2 GATE-2 таймер остаётся 24 ч. Alert срабатывает на 60 мин stale, до таймера. **Alert НЕ удлиняет** stale-timer — это не goal. Goal — owner успевает среагировать в течение 24 ч до GATE-2 close, а не «отодвинуть GATE-2».

## 5. Acceptance для amendment'а (этот документ)

- [ ] **Owner-approval товарищ Шифу** — выбор A/B/C, явный комментарий «принято» / «правки».
- [ ] **PM-sync** (t_870273be, channel question) подтверждает: какой именно канал используется как `#devops` инфра-приёмник.
- [ ] **Diff § 2 принят** в develop этого документа (или правки по комментарию owner'а).
- [ ] **Issue #2393** разблокирован devops для реализации `stale-candidate-alerter.sh` (новая devops-карточка).
- [ ] **Тесты** (по аналогии с § 7.1 ADR-0022):
  - [ ] Test-1: stale-candidate + priority:high + voice → alert attempt → `agent-flow-error` пустой (success) → § 2 строка PASS.
  - [ ] Test-2: stale-candidate + priority:high + voice → webhook 500 → `agent-flow-error` non-empty (best_effort_failed with log) → § 2 строка PASS (не блокер).
  - [ ] Test-3: stale-candidate + priority:medium → alert НЕ срабатывает (триггер не выполнен) → § 2 строка = N/A, close идёт по основным условиям.
  - [ ] Test-4: alert сработал → в течение 5 мин на issue повесили `agent:backend` → следующий alert на ту же issue = skip (auto-deactivation).
  - [ ] Test-5: 5 одинаковых алертов в час на одну issue → только первый проходит (rate-limit).

## 6. Rollback

Если amendment создаёт regression (например, best-effort-failed случаи учащаются и owner просит ужесточить):

1. Revert commit hash этого amendment в develop.
2. Issue #2393 → `blocked` с reason «rollback ADR-0022 amendment — см. <issue>».
3. `stale-candidate-alerter.sh` остаётся в репо (уже не gate, но как observability tool).

Если owner выбирает B (hard-blocker) после A-rollout:

- Reapply amendment с вариантом B.
- Перед reapply: подтвердить, что `#devops` канал **реально существует** (PM-sync t_870273be ответ) и SLA на канал документирован (отдельная ADR на «notification SLA», или pinned в § 4 amendment).

## 7. Связанные

- **ADR-0022** (`docs/adr/0022-process-e2e-done-gates.md`) — родительский ADR, § 2 invariant (строки 29–39), § 4.2 GATE-2 stale-candidate, § 7.2 staged rollout.
- **ADR-0018** — честный FAIL лучше красивого PASS (audit-trail обязателен).
- **ADR-0014** — закрытие issue на merge (GATE-3 dependency).
- **Issue #2394** — `Alert stale-candidate + priority:high → #devops channel (t_cf60f006)` — родительская карточка, devops assignee, blocked 13+ ч.
- **Issue #2393** — `devops-канал алерт на stale-candidate priority:high voice/operator-bug` — фактическая реализация alerter'а, 3 комментария, agent-flow-error уже есть в labels.
- **Issue #2394 parent-context** (kanban t_6927a4c5) — зафиксировал 3 блокера: канал не определён Шифу/PM, ADR-0022 § 2 invariant не расширен, реального `#devops` приёмника нет.
- **PM-документ** `docs/p0-voice-bugs-decisions.md` § 4 — Шифу sync очередь, вопрос 3 «алерт в devops-канал» outstanding.
- **Ретро** `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md` § 5 (c) — рекомендация «отдельный алерт» одобрена как целевое поведение.
- **Runbook** `docs/runbooks/stale-candidate-triage.md` § «Escalation» — ручной пинг owner'а уже описан, **on-call alert в `#devops`** — это gap, закрываемый #2393.
- **Force-triage PR** `z-{devops}/25a2b395-force-triage-priority-high` (Phase 4) — параллельный механизм, не конкурирует с alerter'ом (alerter только notification, force-triage меняет labels).

## 8. Вердикт architect

**Голосую за вариант A** (best-effort + audit).

**Обоснование в одном предложении:** § 2 invariant должен отражать **наблюдаемое** поведение системы; сетевой алерт лучше фиксировать как `fired OR best-effort-failed-with-log` (аудит + ADR-0018 совместие), чем как hard-blocker (SPOF на внешний webhook) или как warning (no enforcement).

**Что нужно от товарища Шифу:**

1. Подтвердить выбор A (или выбрать B/C с обоснованием).
2. Подтвердить, что staged rollout (только NEW e2e-done events после merge) приемлем.
3. Подтвердить, что `stale-candidate-alerter.sh` после merge этого amendment **может быть реализован devops-воркером** без дополнительных ADR (т.е. это всё, что нужно для #2393 → ready → done).

После owner-approval: devops берёт #2393 → реализует `stale-candidate-alerter.sh` + 5 тестов из § 5 → PR → e2e-done через GATE-1 (сценарий «stale-candidate priority:high voice alert fired»).