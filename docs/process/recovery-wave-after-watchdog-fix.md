# Recovery-Wave после watchdog-provider-quick fix

**Ретро-карточка:** t_916ef347 (25.08 05:18 UTC)
**Фикс:** PR #1609 (`z-devops/t_4c73490f-llm-quick-crash-guard`, +690/-1, 4 files, 9 unit-tests passing)
**Корневая проблема:** AV-* карточки упали в `blocked` за <120с из-за quick-crash провайдеров (MiniMax 429 / DeepSeek 401 invalid api key). Старый `watchdog.sh` детектил только долгие зависы (>1 тик), а quick-crash успевал натикать `consecutive_failures=2 → gave_up` раньше, чем guard вмешивался.
**Watchdog-фикс:** `scripts/agent_flow/watchdog-provider-quick.sh` — 1-min fast-tick с расширенными `PROVIDER_MARKERS` (HTTP 401, Authentication Fails, invalid_request_error, Token Plan rate limit, Insufficient Balance).

Этот runbook фиксирует **пошаговую recovery-процедуру** для 9 AV-* карточек, чтобы при повторении подобной ситуации в будущем оператор или cron recovery-волны действовал по единому контракту, а не догадывался.

---

## 0. Когда применять этот runbook

Применимо **только** если в Kanban-карточках одновременно:

1. `status='blocked'` (8 из 9) или `status='blocked' + block_kind='capability'` (1 из 9 — t_307bae4a).
2. `last_failure_error` содержит один из паттернов:
   - `pid N not alive` (для `status='blocked'` группы)
   - `worker exited cleanly (rc=0) without calling kanban_complete` (новая protocol-violation семантика dispatcher'а)
3. В `~/.hermes/logs/<task_id>.log` или в теле карточки встречается **любой** из watchdog-маркеров:
   - `429`, `2062` (MiniMax Token Plan rate limit)
   - `401`, `Authentication Fails`, `invalid_request_error` (DeepSeek)
   - `402`, `Insufficient Balance`
4. Provider-exhaustion fix (PR #1609 или эквивалент) уже смёржен в `develop`.

**НЕ применять**, если хотя бы один из пунктов не выполнен — recovery на healthy pipeline может зациклить quick-crash обратно.

---

## 1. Фактическое состояние AV-* на 25.08 05:30 UTC (точка отсчёта)

Из 9 карточек списка в retro t_4c73490f:

| task_id | status | block_kind | recovery подходит? | примечание |
|---|---|---|---|---|
| t_a3944375 | running | — | не сейчас (worker уже работает) | AV-3 |
| t_dff9c60a | running | — | не сейчас (worker уже работает) | AV-4 |
| t_4005db7d | blocked | — (NULL) | ДА (gave_up) | AV-5 |
| t_6e991d73 | blocked | — (NULL) | ДА (gave_up) | AV-6 |
| t_c796970a | blocked | — (NULL) | ДА (gave_up) | AV-7 |
| t_17a01b57 | done | — | уже закрыта, исключить | AV-8 (#1607 merge) |
| t_547e17a7 | blocked | — (NULL) | ДА (gave_up) | AV-9 |
| t_783a8a4e | blocked | — (NULL) | ДА (gave_up) | AV-10 |
| t_307bae4a | blocked | capability | **НЕТ** (ручной block — не трогать) | AV-11 |

**Итого под recovery: 5 gave_up карточек** (AV-5, 6, 7, 9, 10). t_a3944375 и t_dff9c60a — уже запущены воркерами (видимо devops применил `hermes kanban unblock` вручную либо cron recovery-волны отработал до того, как мы открыли карточку). t_17a01b57 — done, игнор.

> Ожидаемое **после полного прохода runbook**: 0 blocked (кроме ручного `block_kind=capability` AV-11) + все 8 work-задач в `done` или `running`.

---

## 2. Пошаговый runbook (deploy → install → cron → recovery)

### Шаг 1. Verify PR #1609 merge в develop

```bash
gh pr view 1609 --json state,merged,mergedAt,mergeCommit --jq '{state,merged,mergedAt,mergeCommit:.mergeCommit.oid}'
# ОЖИДАЕМЫЙ ОТВЕТ: {"merged":true,"mergedAt":"<не null>","mergeCommit":"<sha>"}
```

Если `merged=false` — **СТОП**. Не пытаться recovery: без guard'а unblock снова зациклит quick-crash → те же 8 блокировок через <120с.

Если `merged=true` — переходим к шагу 2.

### Шаг 2. Verify watchdog-provider-quick.sh в репо

```bash
git ls-files scripts/agent_flow/ | grep watchdog-provider-quick
# ОЖИДАЕТСЯ: scripts/agent_flow/watchdog-provider-quick.sh

grep -A3 "EXPECTED=(" scripts/agent_flow/install.sh | grep -c "watchdog-provider-quick"
# ОЖИДАЕТСЯ: 1 (PR #1609 добавляет его в EXPECTED для hardlink-sync)
```

Если файла нет в репо или нет в `EXPECTED` — **СТОП**: install.sh не разложит скрипт на хост, и cron в шаге 4 не сможет его вызвать.

### Шаг 3. Раскладка скрипта на хост (install.sh)

```bash
cd <repo>
bash scripts/agent_flow/install.sh
# ОЖИДАЕМЫЙ ВЫВОД: "OK watchdog-provider-quick.sh (hardlink to src)" или "(already hardlink to src)"
```

После выполнения проверить:
```bash
ls -la ~/.hermes/scripts/watchdog-provider-quick.sh \
       ~/.hermes/profiles/agent-flow/scripts/watchdog-provider-quick.sh \
       ~/.hermes/profiles/architect/scripts/watchdog-provider-quick.sh \
       ~/.hermes/profiles/devops/scripts/watchdog-provider-quick.sh
# ОЖИДАЕТСЯ: все 4 пути — hardlink на одну inode
```

**Почему 4 пути важны:** cron может стартовать под любым профилем (architect, devops, agent-flow), и SOT-контракт (`scripts/agent_flow/README.md` — раздел про hardlink-sync) требует, чтобы файл лежал во всех 4 локациях.

### Шаг 4. Регистрация cron `Watchdog Provider Quick-Tick`

В PR #1609 есть `tests/agent_flow/test_watchdog_provider_quick_guard.py` (9 pytest-тестов), но **сам cron registration НЕ автоматизирован** — это делается отдельно. Вариант A (предпочтительный, единый источник истины):

```bash
# ~/.hermes/profiles/devops/cron/jobs.json — добавить запись
{
  "name": "Watchdog Provider Quick-Tick",
  "schedule": "every 1m",
  "script": "watchdog-provider-quick.sh",
  "no_agent": true,
  "deliver": "local"
}
```

Либо вариант B (CLI-эквивалент, если профиль не умеет JSON-edits):
```bash
hermes cron add watchdog-provider-quick --schedule 'every 1m' --no-agent
```

Проверка:
```bash
crontab -l | grep watchdog-provider-quick
# ОЖИДАЕТСЯ: строка с watchdog-provider-quick.sh
```

Подождать 60-90 секунд и убедиться, что guard активен:
```bash
ls ~/.hermes/state/watchdog-provider-quick-actions.txt
# ОЖИДАЕТСЯ: файл существует (создаётся при первом tick)
tail -20 ~/.hermes/logs/watchdog-provider-quick.log
# ОЖИДАЕТСЯ: [watchdog-provider-quick] tick: providers_alive=...
```

### Шаг 5. Sanity check — providers alive?

Прежде чем делать recovery, подтвердить что guard **видит** живые провайдеры (иначе unblock снова зациклит):

```bash
bash ~/.hermes/scripts/watchdog-provider-quick.sh
# ОЖИДАЕМЫЙ ВЫВОД: providers_alive=True (или хотя бы action=0 при False, но только если
# действительно идёт восстановление ключей — НЕ в steady state)
```

Если `providers_alive=False` после merge #1609 и нормальных ключей — **СТОП**. Либо ключ MiniMax/DeepSeek всё ещё невалиден (DeepSeek key `****bd8b` помечен как invalid по факту 24.08), либо guard неправильно сконфигурирован. Эскалация: проверить `.env` (MiniMax API key, DeepSeek API key), потом issue #1193 (запрет на создание отдельных issue про баланс).

### Шаг 6. Recovery-wave для 5 gave_up AV-* карточек

**Только для карточек из таблицы §1 со `status='blocked'` и `block_kind IS NULL`.** Карточки с `block_kind='capability'` (AV-11, t_307bae4a) **НЕ ТРОГАТЬ** — это ручной block Шифу, watchdog-provider-quick имеет отдельный test `test_blocked_capability_kind_does_NOT_unblock` именно для этой защиты.

**Вариант A. Ручной unblock через `hermes kanban unblock`** (если оператор работает сейчас):

```bash
for tid in t_4005db7d t_6e991d73 t_c796970a t_547e17a7 t_783a8a4e; do
  hermes kanban unblock "$tid" \
    --reason "retro t_916ef347: PR #1609 watchdog-provider-quick merged 25.08; providers alive; guard will catch quick-crash if it repeats"
done
```

**Вариант B. Автоматический unblock через cron watchdog-provider-quick** (предпочтительно, если оператор спит):

Cron в шаге 4 сам выполнит recovery в течение 1-2 минут после providers_alive=True. Никаких ручных действий не нужно — guard знает про gave_up ветку через test `test_blocked_gaveup_with_providers_alive_unblocks`.

Чтобы убедиться, что recovery прошёл, дождаться следующего tick и проверить:
```bash
tail -30 ~/.hermes/logs/watchdog-provider-quick.log
# ОЖИДАЕМЫЙ ВЫВОД: provider-quick-unblock <task_id> для каждой из 5 карточек

tail -30 ~/.hermes/state/watchdog-provider-quick-actions.txt
# ОЖИДАЕТСЯ: 5 строк с task_id из списка
```

### Шаг 7. Verify recovery завершён

```bash
# Через sqlite (если доступен) или через kanban CLI:
hermes kanban show t_4005db7d t_6e991d73 t_c796970a t_547e17a7 t_783a8a4e --json status
# ОЖИДАЕТСЯ: status ∈ {ready, running, done} для всех 5
```

Альтернатива (sqlite):
```bash
sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT id,status,block_kind FROM tasks WHERE id IN ('t_4005db7d','t_6e991d73','t_c796970a','t_547e17a7','t_783a8a4e')"
```

### Шаг 8. Archive родительской retro-карточки

После шага 7 — закрыть t_916ef347 как `done` (эта карточка как раз является документом runbook). Если остались незакрытые смежные карточки (t_57b9b28c про e2e-rotation PAUSED) — их можно закрыть **только** если e2e rotation уже разблокирован (после merge #1592, см. retro t_d935096b).

---

## 3. Сценарии отказа runbook

| Симптом | Вероятная причина | Что делать |
|---|---|---|
| `providers_alive=False` на шаге 5, хотя ключи "нормальные" | DeepSeek key всё ещё invalid (****bd8b помечен с 24.08) | Эскалация в issue #1193; **НЕ** делать unblock |
| После шага 6 карточки снова дают quick-crash за <120с | watchdog-provider-quick.sh не в EXPECTED install.sh → не разложен на хост → cron в шаге 4 не запустился | Проверить `ls ~/.hermes/scripts/watchdog-provider-quick.sh`; повторить шаг 3 |
| `crontab -l \| grep watchdog-provider-quick` пуст | Шаг 4 не выполнен / jobs.json не подхвачен | Повторить шаг 4 |
| AV-11 (t_307bae4a) случайно разблокирована | Bug в guard (тест не покрывает edge case) | Срочный block Шифу + issue с repro; **не recovery** |
| 8 карточек снова в blocked через 2-3 минуты | Провайдеры не восстановились, либо новые ключи имеют другую сигнатуру в ошибке (не попадает в PROVIDER_MARKERS) | Эскалация в issue: добавить новый маркер в PROVIDER_MARKERS, новый PR, merge, повторить runbook |

---

## 4. Связь с существующими ADR и процедурами

| Документ | Связь |
|---|---|
| `docs/adr/ADR-0013-incremental-delivery.md` | Quick-fix (PR #1609) → recovery (этот runbook) → close retro — инкрементальная поставка |
| `docs/adr/ADR-0026-recovery-card-contract.md` | Этот runbook — конкретное наполнение «recovery-card-contract» для сценария provider-exhaustion |
| `scripts/agent_flow/README.md` (раздел hardlink-sync) | Шаг 3 ссылается на SOT-контракт hardlink в 4 профилях |
| `scripts/agent_flow/tests/test_watchdog_provider_exhaustion.sh` | Регресс-тест для provider-exhaustion секции watchdog.sh (предыдущий watchdog) |
| `tests/agent_flow/test_watchdog_provider_quick_guard.py` (из PR #1609) | Новые 9 pytest-тестов — покрывают оба ветки (gave_up unblock + capability no-touch) |

---

## 5. Что делать **НЕ** нужно

1. **НЕ создавать issue про MiniMax/DeepSeek баланс** — правило #1193 (это tech-debt про охрану watchdog-guard'а, не про оплату). Факты про баланс — в retro-карточке, не в issue.
2. **НЕ пытаться разблокировать AV-* вручную до merge PR #1609** — quick-crash зациклится (см. §0).
3. **НЕ менять PROVIDER_MARKERS ad-hoc через hot-patch в карточке** — фикс должен идти через PR + tests, иначе регресс на следующей аномалии провайдера.
4. **НЕ удалять старый watchdog.sh или test_watchdog_provider_exhaustion.sh** — это разные слои (2-min долгие зависы vs 1-min quick-crash). Они дополняют друг друга.

---

## 6. Open follow-ups (не блокирует архив retro)

- [ ] **Добавить в jobs.json регистрацию cron watchdog-provider-quick из PR** — сейчас регистрация ручная (вариант A/B в шаге 4). В идеале — отдельный PR с post-merge hook, чтобы install.sh сам вызывал `hermes cron add watchdog-provider-quick --schedule 'every 1m'` (но это уже не scope t_916ef347).
- [ ] **Расширить регресс-тест `test_watchdog_provider_quick_guard.py`** на edge-case `block_kind='capability' + body содержит 401` — сейчас покрыт no-touch, но без комбинации с marker'ом в body. Задача на следующий watchdog-тред, не блокер.
- [ ] **PR #1609 содержит test на capability-kind no-touch**, но реальный AV-11 (t_307bae4a) — это единственный заблокированный этой семантикой в проде. После прохода recovery-wave стоит отдельной задачей убедиться, что guard **никогда** не unblock'нет ручной блок (regression check через месяц).

---

## 7. Контрольный чек-лист для архива retro t_916ef347

- [x] Документ создан в `docs/process/recovery-wave-after-watchdog-fix.md` (этот файл).
- [ ] PR #1609 merged в develop (см. §1 шаг 1).
- [ ] `watchdog-provider-quick.sh` разложен на хост в 4 локациях (см. §2 шаг 3).
- [ ] Cron `Watchdog Provider Quick-Tick` зарегистрирован на `every 1m` (см. §2 шаг 4).
- [ ] `providers_alive=True` на шаге 5 (DeepSeek key `****bd8b` либо восстановлен, либо явно помечен как not-used в конфиге).
- [ ] 5 gave_up AV-* карточек вышли из blocked (см. §2 шаг 7).
- [ ] t_307bae4a (AV-11, capability-kind) **остался** в blocked (см. §2 шаг 6 — НЕ ТРОГАТЬ).
- [ ] t_916ef347 архивирована через `kanban complete`.

После прохождения всех пунктов — retro-карточка t_916ef347 закрывается, и при следующей аномалии провайдера оператор/worker идёт по этому runbook, а не повторяет ретроспективу с нуля.
