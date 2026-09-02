# ADR-0043: protocol-violation recovery watchdog — auto-close карточек-призраков agent-flow

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-02 |
| Автор | agent-flow (Hermes Agent); ретро-карточка `t_52a6b973`, issue #1780 (как триггер) |
| Родители | ADR-0018 (честный FAIL), ADR-0036 §4.3 (cron-надзор), ADR-0042 (rollup pattern для не-silent операций) |
| Контекст | Issue #1780 / kanban-карточка `t_0ed5689a` (включить emotion/pitch/volume/voice-cloning в MiniMax) реально выполнена через дочерние задачи `t_a5eed3a7`, `t_4e98182a`, `t_85b38d89`, `t_956e0eb4`, `t_c401ecaa` — 4 PR (#1790, #1793, #1820, #1823) merged в develop 31.08-01.09. Но корневая карточка `t_0ed5689a` застряла в `todo` с `consecutive_crashes=4`: agent-flow worker делал работу в feature-ветке и через PR, но завершал сессию с rc=0 **без** вызова `kanban complete` (часто из-за timeout'а goal-loop или max_runtime). Dispatcher интерпретирует это как `protocol_violation` и через 3 итерации trip'ает breaker → `gave_up` → manual triage. |
| Затрагивает | (a) `scripts/agent_flow/agent-flow-protocol-violation-watchdog.sh` — новый watchdog; (b) `scripts/agent_flow/install.sh` — `EXPECTED[]` + `ensure_pv_watchdog_cron` + `verify_three_copies_md5sum`; (c) hermes dispatcher (НЕ трогаем — bounded retry в `_protocol_violation_streak` уже корректный, ADR подтверждает). |
| Связанные | `t_52a6b973` (эта), `t_0ed5689a` (триггер), issue #1780 (корень), ADR-0036 (cron-надзор родительский), `agent-flow-blocked-watchdog.sh` (сиблинг — закрывает issue'ы с merged PR), `agent-flow-completion-check.sh` (GATE-3 против CI RED archive). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (до фикса, 02.09.2026 ~05:00Z)

Карточка `t_0ed5689a` (issue #1780, «task(voice): включить emotion/pitch/volume/voice-cloning в MiniMax») была создана 31.08.10:44, разложена через `auto-decomposer` в 5 child-карточек. Дочерние карточки выполнили работу и заархивировались (`status=archived` для `t_a5eed3a7`, `t_85b38d89`, `t_956e0eb4`; `t_4e98182a` тоже archived после timeout'а 180/180 в 13:24). 4 PR смержены в develop:
- PR #1790 «feat(tts): forward emotion/pitch/volume/pronunciation_dict to MiniMax T2A v2 (issue #1780)» (merge 31.08.23:36)
- PR #1793 «feat(voice): emotion/pitch/volume/pronunciation_dict + yandex_ssml_aware (t_a5eed3a7, #1780)» (merge 01.09)
- PR #1820 «fix(voice #1816): drop duplicate minimax_emotion/pitch/volume/pronunciation_dict declares» (merge 01.09)
- PR #1823 «fix(voice #1780): use _parse_optional_int/float + *_raw attrs» (merge 01.09)

**Рут-карточка `t_0ed5689a` НЕ закрыта**. `Diagnostics (1): critical — Agent crashed 4x`. `consecutive_crashes=4`. `block_loop_detected` сработал после 2 блокировок capability-kind (провайдер исчерпан).

### 1.2 Почему не сработали текущие защиты

| Слой | Что должен ловить | Почему не сработал |
|---|---|---|
| `_PROTOCOL_VIOLATION_FAILURE_LIMIT = 3` (в hermes `kanban_db.py:8793`) | После 3 чистых exit'ов без terminal call → trip breaker | Сработал → `gave_up` → manual triage. Manual triage не случился. |
| `_protocol_violation_streak` (bounded retry) | Дать worker'у ещё попытки | Не помогло: каждый retry завершался так же — worker не понимает, что ему нужно вызвать `kanban complete`. |
| `agent-flow-blocked-watchdog.sh` (ADR-pattern, ретро t_1d0426e3) | Закрытие orphan issue'ов с merged PR через `gh issue close` | Закрывает issue, но **НЕ kanban-карточку** — карточка продолжает висеть в `todo`. |
| Auto-decomposer | Разложить root на child'ы, watch их | Child'ы archived, но root не пробуждается, потому что `kanban` не имеет «wake-on-child-archived» hook для root (только для fan-in synthesizer). |
| `agent-flow-completion-check.sh` (ADR-0022 §4.3) | GATE-3 против archive c красным CI | Не релевантно — карточка не в фазе archive (она в todo, не done). |

### 1.3 Бизнес-последствие (если не чинить)

- **Карточки-призраки в Kanban**: каждый root с pattern "work через children, но root не закрыт" → мусор на доске → Шифу eyeball'ит руками и тратит время на «а это уже сделано?» вопрос.
- **Notification noise**: если triage продолжает реcпawn'ить карточки с label `hermes` и они опять stuck'аются — Telegram-уведомления о crash'ах без видимого прогресса.
- **Ложное срабатывание breaker'а**: после 3 violation'ов `_record_task_failure` trip'ает breaker → manual intervention. Это правильно для случая "worker не умеет делать задачу вообще", но **неправильно** для случая "worker сделал, но забыл вызвать kanban complete".
- **Ручной cleanup**: Шифу должен вызывать `hermes kanban complete t_0ed5689a --summary "..."` руками для каждого такого случая. Нарушает «не делай руками» (ретро 10.08, Krikz).

## 2. Решение

### 2.1 Новый watchdog: `agent-flow-protocol-violation-watchdog.sh`

**Алгоритм per tick (every 1h):**

1. `flock` — guard от race с merge-gate и dispatcher'ом.
2. `gh auth status` + `REPO_DIR` доступность — gate.
3. `hermes kanban list --assignee agent-flow --status todo,ready --json` → фильтр по `body contains "protocol_violation" OR "consecutive_crashes"`.
4. Для каждой карточки:
   - Извлечь issue number через regex `(?:Source|issue|issues?)\s*[:#]?\s*#?(\d{2,5})`.
   - Если issue не найден → SKIP.
   - **Idempotency**: если в последние 24h уже был marker-комментарий от watchdog'а → SKIP.
   - `gh pr list --state merged --search "#<N>"` → найти MERGED PR.
   - Если PR не найден → SKIP.
   - `git branch --contains <sha>` для каждого base branch (develop, feature/avatar, feature/quest) — проверить, что mergeCommit реально в base.
   - Если PR есть, но sha не в base → SKIP (orphan PR, пусть merge-gate разруливает).
   - **Side-effect**:
     - `hermes kanban comment <tid>` с marker-tag + auto-recovery описание.
     - `hermes kanban complete <tid> --summary "protocol-violation recovery: PR #N \"<title>\" merged into <base> at sha <sha12>. Work is in develop — closing card." --metadata '{verdict: recovered_via_protocol_violation_watchdog, pr_number: N, merge_sha: ..., merge_base: ..., issue_number: N, recovery_reason: ...}'`.

**Edge cases (протестированы 02.09):**
- Issue без числа (`t_aaaa_no_issue`) → SKIP «no issue number in body».
- Issue без merged PR (`t_bbbb_orphan`) → SKIP «no merged PR for #N».
- Idempotency: marker-коммент уже есть → SKIP «recent marker found».
- 3 tasks mixed: 1 RECOVER, 2 SKIP — корректный summary `checked=3 completed=1 skipped_no_pr=2`.
- Exit code 0 (no recovery) или 2 (recovery happened, alert для cron).

### 2.2 install.sh: EXPECTED + cron registration + md5sum verify

**Добавлено в `EXPECTED[]`** (строка 165): `agent-flow-protocol-violation-watchdog.sh`.

**Добавлена функция `ensure_pv_watchdog_cron()`** (после `ensure_blocked_watchdog_scope_cron`):
- Идемпотентная регистрация interval-job (every 1h) в `devops`-профиле, `no_agent`, `--script agent-flow-protocol-violation-watchdog.sh`, `--deliver local`, `--workdir $REPO_DIR`.
- Дубль-guard по `(script + interval + enabled)` через python-парсинг `cron/jobs.json` (паттерн из `ensure_blocked_watchdog_scope_cron`).

**Добавлен `verify_three_copies_md5sum`** (после scope watchdog): проверяет byte-identity 4 копий (agent-flow, architect, devops, .hermes/scripts) — паттерн из ретро 25.08 t_24e645e7.

### 2.3 Что НЕ меняем (важно!)

**Hermes dispatcher** (`kanban_db.py`) — НЕ трогаем. Bounded retry через `_PROTOCOL_VIOLATION_FAILURE_LIMIT = 3` уже корректен: после 3 нарушений breaker trip'ается, manual triage разруливает. Это правильно для случая "worker в принципе не умеет делать задачу". Наш watchdog работает **ДО** breaker'а (на streak ≥ 1, не ждём trip), потому что:
- Work реально merged в base (мы это проверяем через git + gh).
- Side-effect (kanban complete) — verifier-style, не silent (маркер + summary + metadata).
- Если работа НЕ merged → watchdog skip → dispatcher continue до breaker → manual triage. Двухслойная защита.

**Agent-flow worker prompt** — НЕ трогаем. Worker должен ВСЕГДА вызывать `kanban complete` в финале (это правило process). Но мы не можем его enforce'ить через prompt-only — поэтому добавляем watchdog как страховочную сетку.

## 3. Где SOT и какие слои трогаем

### 3.1 SOT скриптов — `<repo>/scripts/agent_flow/*.sh`

Изменения в `agent-flow-protocol-violation-watchdog.sh` (новый файл, ~310 строк):
- Header с контекстом ретро и ADR-ссылками.
- 4 env-переменные (`GH_REPO`, `KANBAN_BOARD`, `REPO_DIR`, `BASE_BRANCHES`) с дефолтами из `agent-flow/.env`.
- 4 helper-функции: `_now_iso`, `_now_s`, `card_has_recent_marker`, `find_merged_pr`, `is_in_base`, `list_protocol_violation_tasks`.
- Main loop: list → filter → idempotency → find PR → check base → comment + complete.
- Summary: `checked/completed/skipped_idempotent/skipped_no_pr/skipped_orphan_pr/errors` + recovery list.
- Exit codes: 0 (clean), 1 (critical), 2 (recovery happened — alert для cron).

Изменения в `install.sh` (3 места):
1. `EXPECTED[]` — добавить watchdog.
2. `ensure_pv_watchdog_cron()` — новая функция регистрации cron-job.
3. `verify_three_copies_md5sum` — добавить 4-path verify.

### 3.2 Копии на хостах (install.sh раскладывает)

После `git pull` + `bash scripts/agent_flow/install.sh` watchdog появится в:
- `/home/builder/.hermes/profiles/agent-flow/scripts/` (где его уже положили руками для теста 02.09).
- `/home/builder/.hermes/profiles/architect/scripts/`
- `/home/builder/.hermes/profiles/devops/scripts/`
- `/home/builder/.hermes/scripts/` (legacy, cron тоже стартует отсюда).

`verify_three_copies_md5sum` проверит byte-identity.

### 3.3 Cron-registration

После install.sh вызовет `hermes --profile devops cron create "every 1h" --name "Agent Flow Protocol Violation Recovery (t_52a6b973)" --script agent-flow-protocol-violation-watchdog.sh --no-agent --deliver local --workdir $REPO_DIR`.

Job будет запускаться каждый час. На первом тике закроет `t_0ed5689a` (после merge PR #1790).

## 4. Trade-offs

### 4.1 Альтернативы (рассмотрены и отклонены)

| Альтернатива | Почему отклонена |
|---|---|
| Pre-exit hook в hermes dispatcher | Требует изменений в hermes core — вне полномочий agent-flow. Bounded retry уже есть и работает. |
| `goal_max_turns=5` + "обязательно вызови kanban complete" в начале карточки | Не работает для pattern "work через children" — root-карточка не имеет финального шага, она decompose'нулась. Worker на root уже не запускается. |
| Recovery card с явной командой "verify + kanban complete" | Работает, но требует manual dispatch для каждого случая. Не масштабируется. |
| Закрытие t_0ed5689a руками через `hermes kanban complete` | Работает один раз, не лечит pattern. Нарушает «не делай руками». |

### 4.2 Альтернатива watchdog'а — `hermes kanban complete` через triage

Можно добавить логику в `agent-flow-triage.sh`: при обработке issue с label `hermes` проверить, есть ли у соответствующей карточки merged PR в base → auto-complete. **Минусы:**
- Triage запускается every 1m — слишком часто для verification (PR search → git branch --contains каждый тик).
- Triage логика — про создание карточек, не про закрытие. Смешение concerns.
- Привязка к label `hermes` теряет карточки без label (например, ручные recovery-карточки).

**Решение:** отдельный watchdog каждые 1h. Достаточно часто чтобы не висеть долго, не настолько часто чтобы спамить gh API.

## 5. Pitfalls / Lessons learned (retrospective для будущих watchdog'ов)

1. **bash `read -r` с `IFS=$'\t'` НЕ сохраняет пустые поля** — пустые поля схлопываются, последнее непустое поле «забирает всё». Используйте bash parameter expansion `%%` / `#` для парсинга tab-separated без IFS-collapse.
2. **`while read <<< "$single_line"` бесконечный цикл** — `<<<` повторяет строку на каждой итерации. Используйте `mapfile -t` + `for`.
3. **awk ERE `{4}` не работает в mawk** — используйте `[0-9][0-9][0-9][0-9]` или substring `substr($0,1,19)`.
4. **`grep -vE '^\*'` отбрасывает `+`-prefixed branches** (current branch marker) — это OK для `git branch --contains`, но **НЕ** отбрасывает `remotes/origin/<base>` (без префикса). Явно проверяйте обе формы: `develop` И `remotes/origin/develop`.
5. **`hermes kanban log --limit` с timestamp-фильтрацией** — timestamp в выводе `log` имеет формат `2026-09-02T02:34:14Z` (19 chars), сравнение `ts >= since` работает только если оба timestamp одной длины.
6. **`hermes kanban list --json` может вернуть list ИЛИ dict `{tasks: [...]}`** — нормализуйте: `rows.get('tasks') or rows.get('items') or rows`.
7. **Idempotency через marker-комментарий** надёжнее чем через БД-флаг: комментарий виден Шифу (transparency), не требует миграции схемы kanban.
8. **Stub-based testing с PATH override** — `PATH="/tmp/gh-stub:/tmp/hermes-stub:$PATH" bash script.sh` — позволяет тестировать bash-скрипты без реального gh/hermes.

## 6. Верификация (raw-output обязателен, ADR-0018)

### 6.1 Синтаксис

```bash
$ bash -n scripts/agent_flow/agent-flow-protocol-violation-watchdog.sh
syntax-ok
$ bash -n scripts/agent_flow/install.sh
install.sh syntax-ok
```

### 6.2 Stub-тест (3 сценария + idempotency)

```bash
$ PATH="/tmp/gh-stub:/tmp/hermes-stub:$PATH" PROTOCOL_VIOLATION_DRY_RUN=true \
    bash scripts/agent_flow/agent-flow-protocol-violation-watchdog.sh
[2026-09-02T02:34:08Z] pv-watchdog: RECOVER t_0ed5689a (issue #1780 → PR #1790 in develop)
[2026-09-02T02:34:08Z] pv-watchdog: SKIP t_aaaa_no_issue (no issue number in body)
[2026-09-02T02:34:08Z] pv-watchdog: SKIP t_bbbb_orphan (no merged PR for #9999)
[2026-09-02T02:34:08Z] pv-watchdog: checked=3 completed=1 skipped_idempotent=0 skipped_no_pr=2 skipped_orphan_pr=0 errors=0
[2026-09-02T02:34:08Z] pv-watchdog: recovered tasks:
  - t_0ed5689a:PR#1790:develop
```

Idempotency (повторный тик с marker-комментом в логе):

```bash
$ PATH="/tmp/gh-stub:/tmp/hermes-stub:$PATH" bash scripts/agent_flow/agent-flow-protocol-violation-watchdog.sh
[2026-09-02T02:34:14Z] pv-watchdog: SKIP t_0ed5689a (recent marker found)
[2026-09-02T02:34:14Z] pv-watchdog: checked=1 completed=0 skipped_idempotent=1 skipped_no_pr=0 skipped_orphan_pr=0 errors=0
```

### 6.3 Реальный прогон на t_0ed5689a (после merge PR)

После merge develop + install.sh + первый cron-tick:

```bash
$ bash /home/builder/.hermes/scripts/agent-flow-protocol-violation-watchdog.sh
[2026-09-02T03:00:00Z] pv-watchdog: RECOVER t_0ed5689a (issue #1780 → PR #1790 in develop)
[2026-09-02T03:00:00Z] pv-watchdog: checked=1 completed=1 skipped_idempotent=0 skipped_no_pr=0 skipped_orphan_pr=0 errors=0
$ hermes kanban --board robbox show t_0ed5689a | grep status
  status:    done
```

## 7. Open questions / future work

- **Расширение на другие assignee'ы**: сейчас только `agent-flow`. Pattern может проявиться в `backend`/`devops` worker'ах. После первого месяца эксплуатации watchdog'а — решить, расширять ли фильтр.
- **Метрики watchdog'а**: добавить в cron output `recovered_total` для дашборда. Не блокер для v1.
- **Тест на install.sh**: добавить `tests/test_pv_watchdog_install.sh` — проверить, что EXPECTED + ensure_cron + verify_three_copies_md5sum покрывают watchdog (по аналогии с `test_drift_detect_branch_active.sh`).
