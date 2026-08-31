# ADR-0040: e2e-process — жёсткий контракт «run не стартанул → round не считается»

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-01 |
| Автор | architect (Hermes Agent); карточка `t_a0890749`, issue #1831 |
| Контекст | `agent-flow-e2e-process.sh` циклически создаёт round-ветки (`round-279…round-308` за день, `round-counter = 311`) потому что `gh workflow run` для `L: E2E Voice Test` либо не возвращает run-id, либо возвращает ошибку — а следующий тик всё равно видит `needs-e2e` issue и стартует round заново, инкрементируя счётчик. CI spam → лишние workflow runs → зашумлённый аудит trail. |
| Затрагивает | `scripts/agent_flow/agent-flow-e2e-process.sh` (`_trigger_workflow_with_retry`, `wait_workflow`, основной цикл `process_issue`, post-tick cleanup, `round_ensure`), `~/.hermes/state/agent-flow-e2e-round-counter`, `~/.hermes/state/agent-flow-e2e-run-state` (новый), label `e2e:infra-fail` (расширение контракта). |
| Родители | ADR-0013 (incremental delivery — узкая правка, не переписывать процесс), ADR-0018 (honesty culture — голословное «e2e прошёл» карается), ADR-0022 (e2e done gates — этот ADR уточняет один gap), ADR-0025 (stale-PR detection — у нас другая root cause, но архитектурный стиль похож), ADR-0026 (recovery card contract — `infra-fail` теперь идёт через тот же канал). |
| Связанные | issue #1831 (эта задача), issue #1826 (loop on rounds — частично fix в PR #1828, skip SHA-tags), issue #1825 (round cleanup), issue #1707 (worktree cleanup), PR #1828 (skip SHA-tags), agent-flow-install-daily cron (workaround до merge этого фикса). |

---

## TL;DR

`agent-flow-e2e-process.sh` сейчас делает **неявное предположение**: «если `gh workflow run` вернул exit 0 — run стартанул». Это **неверно**: GitHub API возвращает 202 Accepted, но run может не появиться в `gh run list` минуту или больше (eventual consistency), а в исключительных ситуациях (429, repo permission change, workflow file error) — не появиться вообще. Текущий код отрабатывает «успешно» (wait_workflow timeout → errored++), но `round-counter` уже инкрементирован (round_ensure отработал ДО issue-loop), ветка может остаться, и следующий тик снова видит `needs-e2e` issue → снова round_ensure → counter++. **Нижняя граница для исправления**: round-ветка без run = round-ветка-фантом, она должна быть либо удалена, либо на ней должен быть run. Этот ADR формализует жёсткий контракт:

1. **`_trigger_workflow_with_retry` ВСЕГДА возвращает run_id** (числовой) или non-zero exit + явно логирует «run not started» (а не «retry failed»). Никаких «возможно стартанул — попробуем ждать».
2. **`process_issue` после non-zero от `_trigger_workflow_with_retry`** НЕ инкрементирует `round-counter` (counter уже мог быть инкрементирован в `round_ensure`, но `round_ensure` будет откатывать через `post_round_sweep`, если на ветке не появилось ни одного run).
3. **`run_state.json` (новый state-файл)** хранит `last_e2e_run_id`, `last_e2e_verdict`, `consecutive_fails` per-issue, чтобы `process_issue` мог принять решение «3 фейла подряд → label `e2e:infra-fail`» без обращения к GitHub API каждый раз.
4. **`consecutive_fails ≥ 3`** для одного issue → label `e2e:infra-fail` + comment + **НЕ создавать новый round** на следующий тик (issue помечается `infra-fail` и `needs-e2e` снимается, чтобы не было infinite loop).
5. **stale lock-file recovery**: если lock пустой или owner PID мёртв — process НЕ считает, что другой инстанс работает. Уже сейчас flock защищает от race, но lock-empty state ввёл в заблуждение юзера при расследовании.

Цель: `round-counter` должен расти **только когда на round-ветке появился хотя бы один workflow run**. Иначе — counter rollback, round-ветка удаляется (или просто не пишется в counter).

---

## 1. Контекст и бизнес-проблема

### 1.1 Симптом (issue #1831, наблюдение 31.08)

Раунд ветки бесконечно создавались 30.08 (round-279 … round-308, по несколько штук в час) потому что `agent-flow-e2e-process.sh`:

1. Видит `needs-e2e` issue (10 открытых на 31.08).
2. Создаёт round-ветку через `round_ensure` (line 922) — **`round-counter` инкрементируется здесь**, до issue-loop.
3. Мерджит PR в round.
4. Запускает `L: E2E Voice Test` через `_trigger_workflow_with_retry` (line 2007).
5. **E2E падает** (либо `gh workflow run` не возвращает run_id, либо run failed/timeout) — verdict не получен.
6. Issue не получает `e2e-done` label.
7. Следующий тик: `needs-e2e` issue всё ещё открыт → round_ensure → counter++ → ещё один round.

Цикл продолжается, пока не вмешается юзер (MAINTENANCE flag, ручной drop round-веток).

### 1.2 Raw-evidence (31.08 ~01:03 UTC)

```
$ ps -ef | grep agent-flow-e2e-process | grep -v grep
builder  3403902     823  0 01:03 ?  bash /home/builder/.hermes/scripts/agent-flow-e2e-process.sh

$ cat /home/builder/.hermes/state/agent-flow-e2e-round-counter
311

$ cat /home/builder/.hermes/state/agent-flow-e2e-ghost-rounds-total
250

$ ls -la /tmp/agent-flow-e2e-process.lock
-rw-rw-r-- 1 builder builder 0 sep  1 01:03 /tmp/agent-flow-e2e-process.lock
```

- **Process жив** (PID 3403902, started 01:03).
- **Counter = 311** (counter file не сбрасывался с момента дропа rounds в PR #1827 — issue #1825).
- **ghost-rounds = 250** (issue: написано «0», но в реальности 250 — это кумулятивный счётчик ghost-rounds за всё время, ретро-метрика).
- **Lock пустой (0 bytes)** — corrupted/empty; невозможно понять, владеет ли кто-то flock. Process сам по себе flock проверяет (line 580–597), так что race нет, но состояние **наблюдаемое через файловую систему** вводит в заблуждение при расследовании.

### 1.3 Гипотезы (root cause) — что было исключено

| # | Гипотеза | Проверка | Вердикт |
|---|---|---|---|
| 1 | E2E workflow fail → `verify_recent_run` ломается → не двигает labels | last run #33321193003 — failure, verdict получен → значит verify_recent_run работает | **исключено** |
| 2 | Каждый тик e2e видит «нужна итерация» → новый round (без проверки «run был на этой round-ветке») | counter=311, ghost=250 — counter растёт быстрее, чем реальные runs | **подтверждено частично** |
| 3 | Множественные инстансы → race | flock работает (только 1 PID parent) | **исключено** |
| 4 | `gh workflow run` падает по authentication/permission | test на 31.08 — `gh auth status` ok, `gh workflow run` возвращает 0; но run_id не появляется в `gh run list` после 30+ сек (eventual consistency) | **частично подтверждено** |
| 5 | E2E workflow_dispatch trigger input некорректный | trigger inputs валидируются до dispatch (e2e_args[] проверка) | **исключено** |
| 6 | Stale PR > threshold10 (ветка жила давно) | ADR-0025 stale_branch_check, threshold=10, exclude SHA-tags (PR #1828). Live `git ls-remote` показывает round-ветки живые | **не основная причина** |

**Реальная картина**: комбинация гипотез 2 + 4. `gh workflow run` иногда не создаёт run (eventual consistency), `_trigger_workflow_with_retry` либо сразу возвращает 0 (race-dedup решил, что run «уже есть» — но это другой run на другой ветке), либо возвращает 1 после 3 ретраев. В обоих случаях `wait_workflow` либо ждёт чужой run (и timeout), либо timeout сразу. Counter уже инкрементирован, round-ветка уже создана (или reuse'нута).

### 1.4 Бизнес-последствие

Без фикса:
- Counter растёт неограниченно (311 → 312 → 313 → …), расходясь с реальным количеством round-веток в `origin` (counter-vs-remote drift увеличивается, нарушая ADR-0025 §3 «real-behind»).
- Round-ветки без runs копятся в origin → нужны отдельные cleanup-карточки (issue #1707, #1825).
- CI runs spam → у юзера невозможно отличить «реальный e2e прогон» от «retry-цикл». Расследование regress'ов замедляется.
- При MAINTENANCE (см. PR #1828 cleanup) round-counter **не** откатывается, и при возобновлении процесс продолжает с «фантомного» номера.

---

## 2. Решение

### 2.1 Архитектурный принцип

> **Round-ветка без run = round-ветка-фантом. Не считается.**

Counter инкрементируется **только после успешного старта хотя бы одного workflow run** на round-ветке. Если `_trigger_workflow_with_retry` не смог гарантировать, что run стартанул (run_id пуст или non-zero exit + dedup не подтвердил) → counter **не** инкрементируется. Round-ветка, если она была reuse'нута, остаётся, но **не** считается «активной» для этого тика. Post-tick cleanup (line 3946) удаляет её по правилу «0 runs за тик» (уже работает, ретро 14.08 t_4268f2bf).

### 2.2 Изменения в `agent-flow-e2e-process.sh`

#### 2.2.1 `_trigger_workflow_with_retry`: явный возврат run_id

Текущий код (line 2007–2066) возвращает 0 при «возможно успешно» (либо `gh workflow run` exit 0, либо race-dedup подтвердил). Новый контракт:

```bash
# Returns: 0 + run_id printed to stdout if started/confirmed
#          1 + nothing on stdout if NOT started (caller MUST treat as fail)
_trigger_workflow_with_retry() {
    local _wf_name="$1"; shift
    local _attempts=0 _max=3 _sleep
    # ... pre-dispatch dedup (existing, line 2016-2034) ...
    # ... race-dedup (existing, line 2040-2063) ...

    # NEW: после КАЖДОГО успешного пути — НЕ return 0 сразу, а
    # РАСКОПАТЬ run_id из gh run list (createdAt >= e_epoch) и вернуть его
    # в stdout. Если run_id не найден за ≤10 сек → return 1.
    local _run_id _ep="${E2E_TRIGGER_EPOCH:-$(date -u +%Y-%m-%dT%H:%M:%SZ)}"
    for _attempts in 1 2 3; do
        # [existing race-dedup logic]
        if gh workflow run "$_wf_name" --repo "$GH_REPO" ...; then
            # NEW: подтвердить run_id через poll
            _run_id="$(poll_run_for_epoch "$_wf_name" "$_dedup_branch" "$GH_REPO" "$_ep" 10)"
            if [[ "$_run_id" =~ ^[0-9]+$ ]]; then
                printf '%s\n' "$_run_id"
                return 0
            fi
        fi
        # ... race-dedup / retry logic ...
    done
    return 1  # run_id так и не получен
}

# Новый хелпер: poll run list с фильтром createdAt >= epoch, max 10 сек
poll_run_for_epoch() {
    local _wf="$1" _br="$2" _repo="$3" _ep="$4" _max=${5:-10}
    local _deadline=$((SECONDS + _max)) _run_id
    while [ "$SECONDS" -lt "$_deadline" ]; do
        _run_id="$(gh run list --repo "$_repo" --workflow "$_wf" --branch "$_br" \
            --limit 3 --json databaseId,createdAt --jq \
            "[.[] | select(.createdAt >= \"$_ep\")][0].databaseId" 2>/dev/null || echo "")"
        _run_id="$(printf '%s' "$_run_id" | grep -oE '[0-9]+' | head -n1 || true)"
        if [[ "$_run_id" =~ ^[0-9]+$ ]]; then
            printf '%s\n' "$_run_id"
            return 0
        fi
        sleep 2
    done
    return 1
}
```

#### 2.2.2 `process_issue`: жёсткая реакция на non-zero trigger

Сейчас (line 3396–3398):

```bash
if ! _trigger_workflow_with_retry "$E2E_WORKFLOW" --ref "$ROUND_BRANCH" "${e2e_args[@]}"; then
    log "issue #${number}: failed to trigger ${E2E_WORKFLOW} after retries"; errored=$((errored+1)); continue
fi
```

Новое поведение: если trigger вернул non-zero (run не стартанул) →

1. Прочитать `run_state.json` для этого issue. Если `consecutive_fails` (для e2e_workflow specifically) уже ≥ 3 — label `e2e:infra-fail` (idempotent), **снять** `needs-e2e` (issue помечается как «infra-broken, не наш баг»), comment с run-link (`run_id = empty`, объяснение «run не стартанул N раз подряд»).
2. Если `consecutive_fails < 3` — increment, save state, **продолжаем тик** (не `continue`!), чтобы остальные issues в этом тике получили свой шанс. Round-ветка **не** используется для следующих issues (помечается `E2E_TRIGGER_FAILED_THIS_TICK=1`).
3. `round-counter` НЕ инкрементируется (откатывается через post-tick cleanup, который уже работает для «0 runs» — line 3946).

#### 2.2.3 `run_state.json` (новый файл)

Путь: `${HERMES_HOME}/state/agent-flow-e2e-run-state.json`. Формат:

```json
{
  "schema_version": 1,
  "issues": {
    "1831": {
      "last_e2e_run_id": "33321193003",
      "last_e2e_verdict": "failure",
      "consecutive_fails": 1,
      "first_fail_at": "2026-08-30T16:00:00Z",
      "last_attempt_at": "2026-08-31T01:00:00Z",
      "infra_fail": false
    }
  }
}
```

- `consecutive_fails` сбрасывается в 0 при `verdict == "success"` или при ручном `e2e:infra-fail` → reopen.
- Файл обновляется **после** каждого verdict (line 3520+ в существующем коде, после `fail_kind` decision tree).
- Concurrent-safe: flock (line 580) защищает от параллельной записи.

#### 2.2.4 Lock-file recovery

Текущее поведение: flock (line 580–597) — корректно. Но **lock-файл `/tmp/agent-flow-e2e-process.lock`** пустой (0 bytes) — неинформативен. Изменение:

- Записывать в lock-файл **PID + epoch старта** (формат `PID:EPOCH\n`). На старте process проверяет: если в lock-файле PID != свой — значит, flock пропустил (теоретически не должно быть, но paranoia); если lock-файл пустой или owner PID мёртв (`kill -0` возвращает non-zero) — process логирует «lock stale, recovered», продолжает работу. Не считает, что другой инстанс жив.

### 2.3 Acceptance (test plan для agent-flow developer)

- [ ] Создать issue с `needs-e2e` label, дать агенту сделать ОДИН run. Запустить `agent-flow-e2e-process.sh`. Trigger возвращает run_id. fail → label `e2e:infra-fail` НЕ ставится (consecutive_fails=1 < 3), но `consecutive_fails` в state = 1.
- [ ] Тот же issue, второй тик: trigger опять fail. consecutive_fails=2. Issue остаётся `needs-e2e`.
- [ ] Третий тик: trigger опять fail. consecutive_fails=3 → label `e2e:infra-fail`, `needs-e2e` снимается, comment с run_id (даже если empty — пишем «run_id=N/A, repo perm check failed»).
- [ ] Четвёртый тик: issue больше не имеет `needs-e2e` → skip в `get_issue_needs_e2e_open`. Counter НЕ инкрементируется.
- [ ] Happy path: issue с `needs-e2e`, run успешен → label `e2e-done`, consecutive_fails сбрасывается в 0 (для следующих re-test).
- [ ] `round-counter`: создать ветку, run стартанул (counter++), завершился fail → counter остаётся (round-ветка жива до post-tick cleanup). Создать ветку, run НЕ стартанул → counter НЕ инкрементируется (post-tick cleanup удалит ветку по «0 runs»).
- [ ] Lock recovery: убить процесс руками (kill -9), дождаться нового тика. Новый process видит stale lock, пишет «lock stale, recovered», продолжает работу (не выходит с «already running»).
- [ ] `run_state.json`: проверить schema и что consecutive_fails сбрасывается на success.

### 2.4 Альтернативы, которые отвергли

#### 2.4.1 Снимать `needs-e2e` на первом fail (без consecutive_fails)

**Почему нет**: e2e может упасть по реальной причине (баг в коде) и через 1 ретрай на следующем тике (на новом round с rebase) — пройти. Сразу списывать issue = потерять self-healing.

#### 2.4.2 Делать `_trigger_workflow_with_retry` синхронным с poll_until_run_started (timeout 30 сек вместо 10)

**Почему нет**: GitHub API eventual consistency обычно 5-10 сек. 30 сек — лишний tax на каждый issue в тике (у нас сейчас 10+ needs-e2e). Лучше явное «не стартанул» за 10 сек, чем silent timeout за 30.

#### 2.4.3 Вернуть `round_ensure` post-tick (после trigger), чтобы counter инкрементировался ТОЛЬКО при успешном trigger

**Почему нет**: round_ensure сейчас создаёт round-ветку ДО issue-loop, потому что несколько issues в одном тике используют одну round-ветку (мерж PR идёт в неё). Если перенести round_ensure после trigger — каждый issue получит свой round-ветку, что противоречит существующему контракту (один round = много issues). Лучше хранить round_ensure как есть, но **post-tick cleanup удаляет round-ветку с 0 runs** (он уже это делает, ретро 14.08 t_4268f2bf), и counter откатывается (он уже откатывается, ретро 23.08 t_fdb19f7b). Достаточно убедиться, что **trigger failure → не считается как run**.

#### 2.4.4 Использовать `gh workflow run --json` (если поддерживается) для прямого run_id

**Почему нет**: `gh workflow run` в текущей версии не возвращает run_id в JSON. `gh workflow run --json` есть, но не даёт databaseId в JSON — только «queued» message. Не решит проблему.

---

## 3. План реализации (для agent-flow developer)

Фича реализуется в **3 коммитах** (incremental delivery, ADR-0013):

### Commit 1: state-файл + lock recovery (small, low-risk)

- `run_state.json` создаётся при первом запуске (пустой schema_version=1, issues={}).
- Lock-файл пишется с PID:EPOCH.
- **Acceptance:** run_state.json существует после первого тика, lock-файл содержит PID.

### Commit 2: `_trigger_workflow_with_retry` → return run_id

- Poll-based run_id resolution (max 10 сек).
- Существующие callers (line 3130, 3154, 3396) обновляются под новый контракт.
- **Acceptance:** trigger возвращает run_id в stdout (для логирования + передачи в wait_workflow).

### Commit 3: `process_issue` reactions + `consecutive_fails` + `e2e:infra-fail` label

- State file читается в начале `process_issue`.
- На non-zero trigger — increment, проверка ≥3.
- На verdict — update state file (success → reset, fail → no-op кроме timestamp).
- **Acceptance:** test plan из §2.3.

### Commit 4: unit-тесты

- `tests/test_e2e_process_trigger_no_run_no_round.sh` — mock `gh workflow run` failure, проверить counter.
- `tests/test_e2e_process_consecutive_fails.sh` — 3 fails подряд → label `e2e:infra-fail`.
- `tests/test_e2e_process_lock_recovery.sh` — stale lock → recovery.

---

## 4. Workaround (до merge)

`agent-flow-install-daily` (упомянут в issue #1831) уже содержит fallback:

- **Ночь 02:00–08:00 MSK** — issues с `needs-e2e` получают pause-коммент, новый round не создаётся.
- **5 consecutive fails** (по старой метрике без run_state.json — через `_existing_build` + `_existing_deploy` lookup) — label `e2e:blocked`.

Это **быстрый workaround**, не замена фиксу. После merge этого ADR workaround можно удалить.

---

## 5. Чеклист «почему этот ADR, а не просто фикс»

| Вопрос | Ответ |
|---|---|
| Какую бизнес-проблему это решает? | CI spam (counter 311 за день), phantom round-ветки, невозможность отличить real-run от retry-loop. |
| Какая самая простая альтернатива? | Удалять round-ветки cron'ом каждые 5 мин (workaround). Не лечит root cause. |
| Какой trade-off (complexity vs benefit)? | +1 state-файл, +~30 строк bash, +3 коммита. Benefit: counter отражает реальность, automatic infra-fail, можно отличить real CI от retry-loop. |
| Что будет, если НЕ делать это сейчас? | Counter drift продолжит расти (311 → 312 → …). Расследование regress'ов будет требовать ручной проверки «этот round — фантом или реальный прогон?» в каждом случае. |
| Почему ADR, а не сразу фикс? | Юзер явно сказал: «Просто фиксирую в issue. agent-flow developer починит.» Архитектурное решение (контракт «run → counter», state-файл, lock recovery) — мой артефакт. Имплементация (3 коммита bash) — делегируется developer-карточкой. ADR-0030 §4 требует pre-merge guard на уникальность номера; этот ADR — формальная фиксация решения до написания кода. |
| Совместимо с ADR-0022 (e2e done gates)? | Да. ADR-0022 говорит, какие labels в каких случаях ставятся. Этот ADR уточняет **новый** label `e2e:infra-fail` (уже существовал, но без формального триггера) и state-машину для `consecutive_fails`. |
| Совместимо с ADR-0025 (stale-PR detection)? | Да. ADR-0025 защищает от stale-PR-tip (PR отстал от develop). Этот ADR защищает от «trigger не сработал» (run не появился на round-ветке). Независимые failure modes, оба нужны. |
| Совместимо с ADR-0026 (recovery card contract)? | Да. Если `consecutive_fails ≥ 3`, скрипт создаёт recovery-карточку devops'у (как в line 3111 для build-failed). Этот ADR уточняет, что `e2e:infra-fail` — это label + comment, recovery-card — отдельный канал (если run так и не стартанул после devops intervention). |

---

## 6. Open Questions (для developer-карточки)

| # | Вопрос | Решение по умолчанию (если не уточнит Шифу) |
|---|---|---|
| Q1 | Что делать, если round-ветка уже существует, но `gh workflow run` упал 3 раза подряд — удалять ветку или оставить до post-tick cleanup? | **Не удалять** автоматически. round-ветка может понадобиться для re-run после ручного фикса (developer правит код и пушит в ту же ветку). Post-tick cleanup (line 3946) удалит её в конце тика, если 0 runs. |
| Q2 | Порог `consecutive_fails ≥ 3` — это «3 раза подряд для одного issue» или «3 разных issue в одном тике»? | **Один issue, 3 тика подряд**. Если 3 разных issue в одном тике trigger'нули и упали — это НЕ infra-fail, это проблема batch (например, общий ref был deleted). В таком случае process должен логировать «batch trigger fail N issues» и **не** выставлять `e2e:infra-fail`. |
| Q3 | `run_state.json` — переживает ли он MAINTENANCE flag? | **Да, переживает** (как round-counter). При MAINTENANCE counter не откатывается, и при возобновлении `consecutive_fails` тоже сохраняется — это правильно, иначе issue будет бесконечно триггериться после каждой паузы. |
| Q4 | Что делать с уже существующим `e2e:infra-fail` label, если run успешно стартанул через 5 минут? | Ничего. `e2e:infra-fail` — terminal label (как `e2e-done`), снимается ТОЛЬКО руками Шифу. Авто-rotation на `needs-e2e` НЕ делается. |
| Q5 | `_trigger_workflow_with_retry` сейчас возвращает exit 0 на race-dedup success (строка 2055-2057). Этот путь тоже должен возвращать run_id в stdout, или только при прямом `gh workflow run`? | **Только при прямом**. Race-dedup success означает «уже был run, не надо новый» → run_id берётся из `verify_recent_run` (есть в lib_workflow_dedup.sh). Помечаем stdout как `existing:<run_id>` чтобы caller понимал. |
| Q6 | Backward-compat: что делать со state-файлом, если schema_version=1 нет (первый запуск после merge)? | Инициализировать с `{"schema_version": 1, "issues": {}}`. Не валить процесс. |
| Q7 | Lock-file PID check (`kill -0`) — что если PID жив, но это **другой** процесс (PID reuse)? | Принять риск. flock уже защищает от race. PID-collision в течение 60s крайне маловероятно на 249-сервере. Если хочется параноидальной защиты — писать в lock `PID:EPOCH:GH_TOKEN_HASH`, где `GH_TOKEN_HASH = $(gh auth token | sha256sum | head -c 8)`. **Дефолт: не делать**, оставить TODO. |

---

## 7. Concrete Test Plan (для developer-карточки)

### 7.1 Unit-тесты: `scripts/agent_flow/tests/test_e2e_process_trigger.sh`

Использует bash + функции из `lib_workflow_dedup.sh` (моки). 4 кейса:

1. **happy path** — `gh workflow run` exit 0 + run_id найден за 5 сек → return 0, stdout=run_id
2. **eventual-consistency** — `gh workflow run` exit 0 + run_id появляется за 7 сек → return 0, stdout=run_id (poll работает)
3. **trigger fail** — `gh workflow run` exit 1 + race-dedup не подтвердил → return 1, stdout=пустой (3 retry тоже не помогли)
4. **race-dedup success** — `gh workflow run` exit 1, но `verify_recent_run` находит свежий run → return 0, stdout=`existing:<run_id>`

### 7.2 Integration-тест: `tests/test_e2e_process_infra_fail.sh`

Использует реальный `gh workflow run` (на репо krikz/rob_box_project под тестовым workflow), mock label API.

1. Создать test issue с label `needs-e2e`
2. Запустить process. Trigger fail (через подмену `gh` в PATH → фейковый `gh` всегда exit 1, race-dedup не срабатывает)
3. После 3 тиков (через `--max-ticks=3`) проверить:
   - issue имеет label `e2e:infra-fail`
   - issue НЕ имеет label `needs-e2e`
   - `run_state.json` имеет запись с `consecutive_fails=3`, `infra_fail=true`
   - `round-counter` НЕ инкрементировался (равен pre-test value)
4. Cleanup: руками удалить label, удалить issue

### 7.3 Lock-recovery тест: `tests/test_e2e_process_lock_recovery.sh`

1. Запустить process в background
2. `kill -9 $PID` (имитация crash)
3. Lock-файл остаётся (0 bytes). Через 30 сек запустить process снова
4. Проверить: новый process **НЕ** выходит с «already running», пишет «lock stale, recovered» в лог

---

## 8. Handoff (для следующей карточки)

**Assignee:** agent-flow developer (или devops, если developer заблокирован).

**Branch:** новая `z-{agent}/1831-impl-e2e-no-run-no-round` (от develop, не от моей ветки).

**Делегация содержимого:**

- §2.2.1 → реализация `_trigger_workflow_with_retry` + `poll_run_for_epoch` (Commit 2)
- §2.2.2 → реакции в `process_issue` (Commit 3)
- §2.2.3 → `run_state.json` schema + helpers (Commit 1 + Commit 3)
- §2.2.4 → lock-file recovery (Commit 1)
- §7 → 3 test-файла (Commit 4)

**Критерий готовности developer-карточки:** все 4 коммита зелёные на CI (lint + shell tests + pytest), все acceptance из §2.3 покрыты тестами.

**Критерий merge:** Шифу ревьюит PR, мерджит, и ADR-0040 переводится в статус **Accepted** отдельным коммитом (`chore(adr): ADR-0040 status Accepted`).

**Что делать прямо сейчас (до merge developer-карточки):**

- `agent-flow-install-daily` workaround (issue #1831) продолжает работать (night pause + 5 fails → `e2e:blocked`). Этого достаточно, чтобы CI не спамил.
- Мониторить `~/.hermes/state/agent-flow-e2e-round-counter` — если снова начал расти (311 → 320 за час) → эскалация через комментарий в issue #1831.

**Не делать (out of scope):**

- Не рефакторить весь `agent-flow-e2e-process.sh` (4033 строк). Менять ТОЛЬКО 4 точки: `_trigger_workflow_with_retry` (line 2007), `process_issue` non-zero branch (line 3396-3398), lock init (line 580), verdict handler (line ~3520).
- Не менять label `e2e:infra-fail` definition (уже правильный, ADR-0026 совместим).
- Не менять `wait_workflow` (line 2992) — он продолжает работать с тем же контрактом.
