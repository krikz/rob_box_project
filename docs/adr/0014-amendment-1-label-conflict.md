# ADR-0014 Amendment 1: `needs-e2e + e2e-done` label conflict — data race, не user override

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-09-09 |
| Автор | architect (Hermes Agent) |
| Контекст | Ретроспектива `t_8fba04b9` (issue #1977 stuck-open после merge + label conflict) |
| Затрагивает | ADR-0014 §6 (новая §6.1, §6.2); `agent-flow-merge-gate.sh` (pre-check перед user-reopen guard); `tests/test_merge_gate_conflict_sweep.sh` (новый) |
| Связанные | #1977, #1979 (NOOP-reopen), ADR-AF-0063 (fallback auto-close), retro `t_8fba04b9` §5.3 |

## 1. Мотивация

ADR-0014 §5 (race conditions) рассматривает только **последовательный** race:
e2e-process ставит `e2e-done`, потом merge-gate закрывает. Ретроспектива
`t_8fba04b9` показала **параллельный** race, который ADR-0014 не покрывает.

### 1.1 Воспроизводимая хронология (#1977)

| Время (UTC) | Событие | Источник |
|---|---|---|
| 2026-09-07T03:42:10Z | e2e-process добавляет `e2e-done` (post-round sweep) | GitHub events |
| 2026-09-07T03:42:12Z | sweep снимает `needs-e2e` (штатная очистка) | GitHub events |
| 2026-09-07T11:14:33Z | PR #1979 MERGED в `develop` | `pulls/1979` |
| 2026-09-07T11:28:37Z | **krikz вручную добавляет `needs-e2e` обратно** | GitHub events |
| 2026-09-09T23:00Z | Issue всё ещё OPEN с `needs-e2e + e2e-done` | live API |

После merge-gate увидел эту issue, ADR-0014 §4 user-reopen guard
(проблема #1391, ретро `t_c4f1d5c8`) интерпретировал «reopen метки
после `e2e-done`» как user-reopen intent — и **подавил close**.
Результат: инвариант ADR-0014 §2 формально выполнен
(`MERGED + base=develop + e2e-done + OPEN`), но issue висит OPEN
неопределённо долго, потому что **manual label add ≠ user reopen** —
это другой класс события.

### 1.2 Корневая причина

ADR-0014 §6 (follow-up policy) трактует `needs-e2e` на уже
`e2e-done` issue как **user-override signal**: «если стоит — НЕ
закрывать автоматически». Это работает для user-reopen
(intent = «фикс не принят»). Но **manual add `needs-e2e`** —
это не user-reopen event, это **артефакт рассогласования state**:
два процесса (e2e-process и оператор/скрипт) независимо правят
labels, не синхронизируясь через timeline.

Метка `needs-e2e` имеет два разных источника:

1. **e2e-process** ставит её, чтобы issue попала в следующий
   e2e-раунд. Это нормальный pipeline state.
2. **Оператор вручную** добавляет её (часто в ходе triage
   или sweep cleanup), не подозревая что в этот же момент
   e2e-process уже поставил `e2e-done`.

В обоих случаях метка попадает на issue, но семантика разная.
ADR-0014 склеивает эти два случая в один «user-override», что
неверно для случая 2.

## 2. Решение

### 2.1 §6.1 (новый): Семантика label conflict `needs-e2e + e2e-done`

> **Инвариант:** одновременное присутствие `needs-e2e` И `e2e-done`
> на одной issue — это **invariant violation** (data race detector
> signal), а НЕ user override.

**Почему это инвариант:**

- `needs-e2e` означает «e2e ещё не пройден» (negative claim).
- `e2e-done` означает «e2e пройден успешно» (positive claim).
- Два противоположных утверждения не могут быть одновременно
  истинными для одного и того же артефакта. Если они есть —
  один из них stale (либо `needs-e2e` stale после успешного e2e,
  либо `e2e-done` stale после ручного возврата в очередь).

**Семантика для merge-gate и conflict-sweep:**

- `needs-e2e + e2e-done` ⇒ это сигнал рассогласования, а не user intent.
- User intent читается через **timeline events** (reopened, closed,
  comment с явным manual signal), а не через состав текущих labels.
- User-reopen guard (ADR-0014 §5, issue #1391) **не должен
  триггериться** на этом сочетании меток — там явно проверяется
  timeline-reopen, а не наличие `needs-e2e`.

### 2.2 §6.2 (новый): data race handling — strip + close без user-override guard

> **Правило:** если merge-gate видит `needs-e2e + e2e-done + OPEN`
> в одной issue, это data race. Действия зависят от состояния PR.

**Случай A: PR `MERGED` (нормальный happy path для #1977).**

1. Pre-check **до** user-reopen guard:
   `if has_label(needs-e2e) AND has_label(e2e-done) AND state=OPEN AND pr_state=MERGED:`
2. Логировать: `data race detected, stripping needs-e2e (stale after e2e-done + merge)`.
3. `gh issue edit --remove-label needs-e2e` (идемпотентно).
4. Опубликовать audit-комментарий через `comment_recently_posted`
   (24h dedup, marker `data-race-label-conflict`).
5. **Продолжить нормальный close-path** (ADR-0014 §4 req 4):
   `gh issue close --reason completed`.
6. User-reopen guard **пропускается** для этого case: data-race
   handling выше — это не user intent, а cleanup.

**Случай B: PR `OPEN` (фикс ещё не влит, но e2e прошёл).**

1. Pre-check **до** user-reopen guard: `has_label(needs-e2e) AND
   has_label(e2e-done) AND state=OPEN AND pr_state=OPEN`.
2. **Документированное поведение:** для `OPEN PR` уже работает
   штатный reconcile-путь (ADR-0014 §retro t_92ec94f3,
   `test_merge_gate_e2e_done_review.sh`): `e2e-done + OPEN PR`
   добавляет `needs-review` на PR и **снимает `needs-e2e` с PR, но
   не с issue**. Issue остаётся с conflict до merge-тика.
3. Наш pre-check в `case $_issue_state` **не достигается** для
   `OPEN PR` — reconcile вышел раньше. Это by-design: на merge-тике
   (случай A) тот же самый conflict разрешается полностью (strip +
   close), так что ручное разрешение для `OPEN PR` не требуется.
4. Если нужен немедленный strip на `OPEN PR`-тике — отдельный
   conflict-sweep cron (см. §6.2 devops-карточка) подхватит.

**Случай C: PR `CLOSED` unmerged (закрыт без merge).**

1. Тот же pre-check, `pr_state=CLOSED AND merged=false`.
2. Действие: **strip `needs-e2e`**, оставить OPEN (issue живёт,
   новый PR понадобится). Audit-коммент.

**Случай D: PR `MERGED` + `user-reopened-this` whitelist label.**

Whitelist (ADR-0014 §4 req 6, issue #1391 supplement) **выше** по
приоритету, чем data-race handling. Если Шифу явно поставил
whitelist — это user intent побеждает. Data-race pre-check
срабатывает, но audit-коммент другой: «data race detected, but
whitelist overrides — skipping close». `needs-e2e` НЕ снимаем
(Шифу хочет видеть её в ротации).

**Почему отдельный cron (conflict-sweep) а не только merge-gate:**

- merge-gate живёт на 5-минутном цикле, сканирует только
  `hermes`-labeled open issues через `gh_list_issues_by_label`.
- Conflicting issues могут не иметь `hermes` метки (issue #1977
  имеет `hermes`, но retro показал что `gh_list_issues_by_label`
  иногда возвращает пустой JSON из-за pagination / rate-limit —
  drift-detect тик 22:59:38).
- Reactive sweep: `gh issue list --label needs-e2e --label e2e-done --state open`
  — это прямой запрос, **не зависит от `hermes`-filter**.
- Sweep запускается отдельным cron (например, devops every 1h) и
  применяет ту же логику §6.2: strip + close (если PR MERGED) /
  strip + leave-open (если PR OPEN).
- Sweep и merge-gate **разделяют один helper**
  (`_conflict_sweep_resolve`) — никакого дублирования логики.

## 3. Acceptance criteria

### 3.1 Автоматические тесты

Должны быть покрыты сценарии:

1. **C1 (happy path):** `MERGED + base=develop + e2e-done + needs-e2e + OPEN`
   ⇒ strip `needs-e2e`, `gh issue close --reason completed` вызван
   ровно один раз, audit-коммент с маркером `data-race-label-conflict`
   опубликован, user-reopen guard НЕ задействован.
2. **C2 (defer):** `OPEN PR + e2e-done + needs-e2e + OPEN issue` —
   штатный reconcile (`test_merge_gate_e2e_done_review.sh`) добавляет
   `needs-review` на PR. Наш data-race pre-check не достигается
   (reconcile вышел раньше) — это by-design, документировано в §2.2
   случай B. Issue остаётся OPEN с conflict; merge-тик разрешит
   полностью (см. C1).
3. **C3 (whitelist override):** `MERGED + e2e-done + needs-e2e +
   user-reopened-this + OPEN` ⇒ audit-коммент упоминает whitelist,
   `needs-e2e` НЕ снимается, `gh issue close` НЕ вызван.
4. **C4 (idempotency):** повторный тик после C1 ⇒ `needs-e2e` уже
   снят ⇒ strip-операция no-op (через `comment_recently_posted`),
   `gh issue close` повторно НЕ вызван (issue уже CLOSED).
5. **C5 (no PR found):** `e2e-done + needs-e2e + OPEN` без
   ассоциированного PR ⇒ skip + лог «no PR to evaluate, defer»,
   `needs-e2e` НЕ снимается (нет доказательства data-race vs
   manual user action без timeline-события).
6. **C6 (existing regression):** все существующие тесты
   `test_merge_gate_e2e_done_data_only_pr.sh`,
   `test_merge_gate_e2e_done_review.sh`,
   `test_merge_gate_post_merge.sh`,
   `test_merge_gate_fallback_keyword.sh` остаются зелёными —
   pre-check **строго до** user-reopen guard не меняет happy path
   для issue без conflict.

### 3.2 Rollout

1. SOT-изменения только в `scripts/agent_flow/` (`agent-flow-merge-gate.sh`,
   новый `tests/test_merge_gate_conflict_sweep.sh`).
2. **Helper `_conflict_sweep_resolve`** выделен в merge-gate.sh —
   общая логика для merge-gate и conflict-sweep cron.
3. `bash -n` + `shellcheck` для всех затронутых shell-скриптов.
4. Все существующие merge-gate тесты (см. §3.1 #6) зелёные.
5. PR в `develop`, штатное ревью.
6. После merge: `scripts/agent_flow/install.sh` раскладывает
   host-копии (drift-detect подхватит).
7. **Отдельная devops-карточка** для conflict-sweep cron
   (не входит в этот PR — это отдельный файл/cron, администрируется
   devops-профилем).

## 4. Последствия

### Положительные

- Stale-open после merge больше не зависит от того, успеет ли
  merge-gate увидеть issue до manual label add.
- Whitelist (`user-reopened-this`) остаётся единственным
  настоящим user-override signal — ADR-0014 §4 req 6 не нарушен.
- Reactive sweep закрывает gap merge-gate для issues без
  `hermes`-метки или при pagination/rate-limit edge-cases.
- Audit-комменты идемпотентны через `comment_recently_posted`
  (ADR-AF-0063) — нет спама каждые 5 минут.

### Отрицательные / риски

- Strip `needs-e2e` без timeline-доказательства может быть
  неверным, если оператор действительно хотел re-rotate issue
  в e2e (случай «фикс не сработал, нужна новая итерация»).
  Решение: **Q22-supplement** (отдельная метка `manual-rotate`),
  которая защищает от strip. Это вне scope этого amendment.
- Дополнительный cron (conflict-sweep) добавляет ещё одну точку
  отказа — нужен отдельный мониторинг liveness (alarm на
  silent output).

### Если не внедрять сейчас

Pattern «stale-open после merge + label conflict» будет
повторяться. Уже зафиксированы: `t_9b0d60f7`, `t_9d375e3e`,
`t_a2521b07`, `t_bba981eb`, `t_a09e893a`, `t_fd604461`,
`t_8fba04b9` (#1977). Каждый инцидент = silent process fail +
ручная диагностика + ручной close. Amendment устраняет
**корневой класс ошибки**, а не симптом.

## 5. История

- 2026-09-09: Accepted (architect, на основе retro `t_8fba04b9`).
- Implementation: PR #TBD (z-architect/t_e4e617a6-amendment-conflict → develop).