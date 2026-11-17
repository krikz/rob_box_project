# ADR-0031: agent-flow-triage двухфазный фильтр (Phase 1 hermes + Phase 2 GSD-orphans)

| Поле | Значение |
|---|---|
| Статус | Accepted (после merge PR в develop) |
| Дата | 2026-08-26 |
| Автор | devops (Hermes Agent); ретро-карточка t_360dc1a4 |
| Контекст | Orphan issue #1643 ([quest] Phase 1.7 e2e smoke, source:gsd + needs-e2e) лежал 10ч+ без kanban-карточки и PR, потому что `agent-flow-triage.sh` фильтровал ТОЛЬКО по label `hermes`. Специфика GSD-workflow: он выставляет `source:gsd`, но не всегда `hermes` (видимо, только для issues, прошедших первичный triage-фильтр бота). |
| Затрагивает | `scripts/agent_flow/agent-flow-triage.sh` (новые Phase 2 + рефакторинг → `process_issues_json()`), `scripts/agent_flow/tests/test_triage_phase2_gsd_orphans.sh` (новый), потенциально `github/workflows/gsd-*.yml` (долгосрочное — вне scope этого ADR) |
| Родители | ADR-0018 (честность — orphan был невидим 10ч), ADR-0013 (incremental delivery — фикс маленький и точечный), `t_360dc1a4` (этот фикс), `t_1a6c74f6` (родительская ретро-карточка) |
| Связанные | issue #1643, t_9b0786a7 (orphan поднят как kanban-карточка Шифу вручную), t_a0fac345 (предыдущий dedup-фикс — расширяет общую картину idempotency) |

> **TL;DR.** Triage-cron был однозадачным: брал issues с label `hermes` →
> создавал kanban-карточки. Issue #1643 не попал под этот фильтр (только
> `source:gsd`, без `hermes`), провалялся 10 часов как orphan. Добавляем
> **Phase 2** в основной цикл: после обработки `hermes`-flagged issues
> берём все open issues с `source:gsd`, вычитаем Phase 1 (дедуп по номерам),
> оставшиеся (orphans, без метки `hermes`) обрабатываем тем же pipeline.
> Один refactor: основной while-loop вынесен в функцию `process_issues_json`
> — оба этапа используют ИДЕНТИЧНЫЕ guards (idempotency, throttle, big-bang,
> MERGED-PR skip, role validation, comment-marker).

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

Ретро-карточка `t_1a6c74f6` (родительская) обнаружила 25.08: issue #1643
`[quest] Phase 1.7 — e2e smoke по локальной сети` лежит открытым с метками:

```
source:gsd, type:functional, priority:high, needs-e2e, feature,
area:rob_box_quest, phase:1.7
```

— **но без метки `hermes`**. `agent-flow-triage.sh` (строки ~204, до этого
ADR) выполнял только:

```bash
issues_json="$(gh_list_issues_by_label "$ISSUE_LABEL" open "$ISSUE_LIMIT")"
# ISSUE_LABEL="${ISSUE_LABEL:-hermes}"
```

— и тихо выходил с `tick done: created=0 skipped=0 errored=0`. Issue
**10 часов** не имел ни kanban-карточки, ни PR, ни комментариев от triage.

### 1.2 Почему это блокер (а не косметика)

1. **Скрытая потеря работы.** Гермес-cron — единственный механизм,
   поднимающий issues в работу. Orphan = задача забыта.
2. **Высокий приоритет.** `#1643` имеет `priority:high`, `needs-e2e`,
   phase 1.7 (т.е. блокирует downstream Phase 2). 10ч простоя × high-priority
   = потенциально сорванный milestone.
3. **Невидимость для оператора.** Логи cron не показывают аномалию —
   `tick done: created=0 skipped=0 errored=0` выглядит как штатное
   завершение. Шифу узнал о проблеме только через ручной просмотр issues.
4. **Системность.** Если `#1643` создан по специфике GSD-workflow (он
   выставляет `source:gsd`, но не `hermes`), **другие orphan'ы уже
   существуют или появятся** в будущем.

### 1.3 Где именно баг

`scripts/agent_flow/agent-flow-triage.sh:204` (до фикса):

```bash
issues_json="$(gh_list_issues_by_label "$ISSUE_LABEL" open "$ISSUE_LIMIT")"
```

— один-единственный фильтр, и он строго по `label=hermes`. Любой issue
без этой метки проваливается мимо. В нашем случае:

| Issue | labels | Виден Phase 1? |
|-------|--------|----------------|
| #1639 | source:gsd + hermes + agent:backend | ✓ да |
| #1643 | source:gsd + needs-e2e + ... БЕЗ hermes | ✗ **нет — orphan** |
| #1605 | source:gsd + hermes + agent:backend | ✓ да |
| #1600 | source:gsd + agent:backend БЕЗ hermes | ✗ **orphan** |
| #1597 | source:gsd + agent:backend БЕЗ hermes | ✗ **orphan** |
| #1590 | source:gsd + ai-generated БЕЗ hermes | ✗ **orphan** |

(Данные на момент ретро 26.08 06:22 UTC.)

---

## 2. Решение

### 2.1 Двухфазный фильтр

```
┌─────────────────────────────────────────────────────────────┐
│  agent-flow-triage.sh                                       │
│                                                             │
│  G1: MAINTENANCE gate                                       │
│  G2: gh auth check                                          │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  Phase 1: primary filter                              │  │
│  │  issues = gh_list_issues_by_label(hermes)             │  │
│  │  for issue: process_issues_json("phase1", issues)     │  │
│  └───────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  Phase 2: GSD-orphans                                 │  │
│  │  gsd = gh issue list --label source:gsd               │  │
│  │  orphans = filter(gsd, NOT hermes AND NOT in phase1)  │  │
│  │  for issue: process_issues_json("phase2", orphans)    │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  G3: rate-limit guard (общий)                               │
│  G4: kanban create + comment-marker                         │
│  G5: idempotency по existing_by_issue, marker, MERGED-PR    │
│  G6: throttle 4ч                                            │
│  G7: big-bang check                                         │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 `process_issues_json` — общий pipeline

Раньше основной `while` loop был inline (~430 строк). Чтобы **оба этапа
использовали идентичные guards** (не дублировать idempotency / throttle /
big-bang / role-guard / MERGED-PR skip / коммент-marker), вынесли его в
функцию:

```bash
process_issues_json() {
    local phase_label="$1" issues_stream="$2"
    while IFS=$'\t' read -r number title labels body; do
        # ... все guards (e2e-done, kanban marker, existing_by_issue,
        # MERGED-PR, role validation, big-bang, throttle, kanban create,
        # comment marker, etc.) ...
    done
}
```

Bash scoping: без `local` на счётчиках (`created`, `skipped`, `errored`),
функция пишет в outer-scope переменные. Это OK и идиоматично для нашего
style (см. `free_stale_worktrees_for_branch` в том же файле).

### 2.3 Дедуп Phase 1 ↔ Phase 2

Чтобы Phase 2 не создавал дубль для issue, уже обработанного в Phase 1:

1. **Перед Phase 2** собираем `phase1_issue_numbers` — отсортированный
   `|`-separated список номеров из `phase1_json`.
2. **В Phase 2 filter** (python-блок) вычитаем эти номера + defensive
   check по `hermes` метке в labels (на случай, если phase1_nums пустой
   из-за rate-limit).

```python
# Python-блок фильтра Phase 2 (полная версия в коде)
phase1_nums = set(...)  # from PHASE1_NUMS env
for it in data:
    n = it.get("number")
    if n in phase1_nums: continue            # уже в Phase 1
    if "hermes" in {l["name"] for l in labels}: continue  # defensive
    if it.get("pull_request") is not None: continue      # не issue
    keep.append(it)
```

### 2.4 Idempotency (важно!)

Phase 2 идёт через **тот же** `process_issues_json` → все guards работают
«из коробки». Для issue #1643 (на момент фикса уже есть карточка
`t_9b0786a7` со status=running):

- `existing_by_issue` ловит regex `issue: #1643` в body карточки →
  skip (line ~538-551 скрипта).

То есть Phase 2 **не плодит дублей** для уже поднятых orphan'ов. Это
именно то поведение, которое ожидается: Шифу поднял #1643 вручную как
`t_9b0786a7` — после фикса triage увидит orphan, проверит existing cards,
скажет «уже есть» и пойдёт дальше.

### 2.5 Логирование

Каждый тик пишет cron-readable логи:

```
[agent-flow-triage] 2026-08-26T06:30:25+02:00 Phase 1: hermes-flagged issues (count=12)
[agent-flow-triage] 2026-08-26T06:30:29+02:00 issue #1658 already has kanban marker — skip
...
[agent-flow-triage] 2026-08-26T06:30:43+02:00 Phase 2: GSD-orphans (4 issues, source:gsd without hermes)
[agent-flow-triage] 2026-08-26T06:30:44+02:00 issue #1643 already has card t_9b0786a7 (status=running) — skip
...
[agent-flow-triage] 2026-08-26T06:30:52+02:00 tick done: created=1 skipped=15 errored=0
```

Логи `Phase 1` и `Phase 2` с явным счётчиком делают cron-рассылку
дебагабельной: «orphan N штук» сразу видно оператору.

---

## 3. Альтернативы (рассмотренные и отвергнутые)

### 3.1 ❌ Сменить `ISSUE_LABEL` на `source:gsd`

Простое, но **слишком широкое**: тогда triage будет подбирать ВСЕ issues
от GSD, включая мелкие tech-debt и обсуждения. Текущий дизайн с явной
opt-in меткой `hermes` — правильный, не размываем его.

### 3.2 ❌ GSD-workflow hook (долгосрочное)

Идея: `github/workflows/gsd-*.yml` добавляет `gh issue edit <num>
--add-label hermes` при создании issue через GSD. Это **правильное**
долгосрочное решение, но:

- Требует менять GSD-workflow (отдельный PR, нужен анализ всех мест где
  GSD создаёт issues).
- Не решает уже существующие orphan'ы (не backfill).
- Этот PR явно вне scope (см. body карточки t_360dc1a4 «Scope НЕ делаем»).

**Принято как future work.** Когда GSD-hook будет готов, Phase 2 станет
пустой и её можно будет удалить без вреда (Phase 1 сделает всю работу).

### 3.3 ❌ Расширить существующий цикл (без выноса в функцию)

Можно было бы продублировать ~430 строк guards для Phase 2. Это
категорически **плохо**: любое изменение guard'а (throttle, big-bang,
idempotency) пришлось бы применять в двух местах → расхождение → баги.
**Отвергнуто.**

### 3.4 ❌ Один общий `--label hermes,source:gsd` (AND)

GitHub label-search не поддерживает `AND` через `--label` (это `OR`). А
даже если бы поддерживал — нам нужен не «intersection», а «union minus
intersection» (orphans = source:gsd − hermes). Реализуемо только
двумя запросами + filter, что и делает наш двухфазный подход.

---

## 4. Acceptance Criteria (проверено в PR)

- [x] PR в `krikz/rob_box_project` base=develop, ветка
  `z-devops/fix-triage-orphan-gsd-quest-1643`
- [x] `bash scripts/agent_flow/agent-flow-triage.sh --dry-run` НЕ падает
- [x] Backfill: при apply на проде триаж подхватывает #1643 (но
  t_9b0786a7 уже есть → dedup работает, карточка не дублируется)
- [x] Юнит-тест: фиктивный `source:gsd` + `needs-e2e` issue без `hermes`
  → попадает в Phase 2 (T1, T3 в `test_triage_phase2_gsd_orphans.sh`)
- [x] Юнит-тест: фиктивный `source:gsd` + `needs-e2e` + `hermes`
  → попадает ТОЛЬКО в Phase 1, Phase 2 skip (T1, T2)
- [x] Логи cron-readable: `log "Phase 2: GSD-orphans (${count} issues)"`
- [x] ADR-0031 (этот документ) фиксирует двухфазный фильтр

---

## 5. Известные ограничения и future work

### 5.1 Backfill уже существующих orphan'ов

Phase 2 ловит **новые** orphans на следующем cron-тике (раз в 5 мин).
Для уже-существующих orphan'ов (`#1600`, `#1597`, `#1590` на момент
этого ADR) — тик подхватит. Если они нужны срочно — запустить
`triage --dry-run` НЕ создаст карточек; нужно реальное выполнение.

### 5.2 GSD-workflow hook (долгосрочное)

Когда сделаем `gh issue edit --add-label hermes` в
`github/workflows/gsd-*.yml`, Phase 2 станет опциональной защитой.
Можно оставить как defence-in-depth (если hook сломается — orphan'ы всё
равно подхватятся) или удалить (после стабилизации hook'а).

### 5.3 Мониторинг

После merge → 1 неделю смотрим логи cron на:

- `Phase 2: GSD-orphans (N issues)` где N > 0: нормально, если это
  свежие orphan'ы (свежий triage).
- `Phase 2: GSD-orphans (N issues)` где N константно > 5: что-то не так,
  GSD-workflow перестал выставлять hermes (или метку вообще убрали).

Сейчас мониторинг — ручной через `journalctl -u agent-flow-cron` или
`~/.local/state/agent-flow-triage.log`. В планах — alert в Telegram
при N > threshold.

---

## 6. Changelog

- **2026-08-26 t_360dc1a4**: Initial ADR. Phase 2 GSD-orphans +
  `process_issues_json()` refactor + тесты.
- **Future**: GSD-workflow hook (отдельный PR). Удаление Phase 2 после
  стабилизации hook'а.
