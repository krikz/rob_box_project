# ADR-0019: Авто-создание incident-issue при рецидиве provider-exhaustion

**Дата:** 2026-10-04
**Автор:** devops worker (kanban t_4aaeeef6)
**Статус:** draft (предложен на основе ретро t_bf8216cb и ночного ревью 2026-10-03)
**Severity:** MEDIUM — без формализации следующий рецидив опять пройдёт молча
**Контекст:** `scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh`

## Контекст

3 октября 2026 года произошёл **5-й рецидив исчерпания MiniMax/DeepSeek LLM-провайдера** с момента закрытия issue #1193 (13.08.2026, completed). Рецидивы хронологически:

| Дата | Эффект | Issue |
|------|--------|-------|
| 13.08.2026 | MiniMax TTS error 2056 | #1193 (closed completed) |
| 14.08.2026 | TTS error 2056 на роботе | (ретро, нет issue) |
| 15.08.2026 | TTS error 2056 | (ретро, нет issue) |
| 16.08.2026 | TTS error 2056 продолжается | (ретро, нет issue) |
| **03.10.2026** | LLM провайдер 402/429 (5 попыток упали) | **(нет issue)** |

Воркеры писали в комментариях заблокированных kanban-карточек «провайдер исчерпан, ждать», но **НИКТО** не открыл GitHub incident-issue, чтобы у Шифу был сигнал и триггер на пополнение бюджета. Шифу узнал только из ночного ревью (t_bf8216cb). 6 фазовых карточек #3014 (MusicManager split) **блокировались 14+ часов**, потому что Шифу никто не подсказал, что нужно пополнить MiniMax/DeepSeek.

Существующий скрипт `agent-flow-cancel-on-provider-exhausted.sh` (внедрён в ретро t_197de62a) умеет блокировать kanban-карточки по сигнатуре provider-exhaust + писать sentinel-комментарий в **уже существующий** issue. Но он **не умеет создавать НОВЫЙ** issue, когда такового ещё нет.

## Решение

Расширяем `agent-flow-cancel-on-provider-exhausted.sh` опциональным `--auto-issue` режимом (env-gated через `PROVIDER_EXHAUST_AUTO_ISSUE=1`, **default OFF — safe-by-default**). При включении:

1. Скрипт делает свою обычную работу: блокирует карточки, пишет sentinel-комментарии в уже существующие issue'ы (как раньше).
2. **После** успешного cancel-flow проверяет два guard'а:
   - **(a) Local rate-limit** — `ISSUE_COOLDOWN_FILE` (`$HERMES_HOME/state/agent-flow-cancel-provider-exhausted-last-issue`) — если файл моложе `PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS` (default 24ч) → SKIP.
   - **(b) GitHub-truth** — `gh issue list --label recurrent-incident --state open --limit 1` — если уже есть ОТКРЫТЫЙ issue с этим лейблом → SKIP.
3. Если оба guard'а пропускают — собирает body (timeline cancel-actions за этот tick, root cause ссылка на #1193, develop HEAD, что делать блок для Шифу) и вызывает `gh issue create --repo krikz/rob_box_project --label recurrent-incident,hermes,agent:devops --assignee <ваше>`.
4. Записывает cooldown (epoch) — следующие 24ч не создавать ещё.

**Тело issue содержит:**

- 🤖 marker tag (распознаётся как auto-generated).
- Recurrent incident header с указанием даты + количество заблокированных карточек.
- Root cause: MiniMax provider budget → #1193.
- develop HEAD SHA.
- Cooldown file path.
- Markdown-таблица заблокированных карточек (task_id, board, linked issue, signal, title).
- **Что нужно от Шифу** — пополнить MiniMax/Token Plan → подождать 5-15 мин → запустить `--recover` → закрыть issue.

## Безопасность (default-OFF — почему это важно)

Поведение скрипта **точно такое же, как до этого PR**, пока `PROVIDER_EXHAUST_AUTO_ISSUE=1` не выставлен в env cron-job'а. Шифу/Юзер **включает осознанно** после merge. Это:

- Защищает от случайного фандинга issue'ов во время первоначального rollout.
- Даёт возможность протестировать auto-issue через `--dry-run` без сетевых эффектов.
- Не ломает существующих инсталляций: ни один хост не изменит поведение до ручного включения.

## Гарантии идемпотентности (по аналогии с ADR-FS-001)

| Guard | Защищает от | Эшелон |
|-------|-------------|--------|
| Sentinel-комментарий на карточке | дублей блокировок одной карточки | per-task |
| `ISSUE_COOLDOWN_FILE` (mtime 24ч) | шторма incident-issue'ов в одном HA-окружении | per-host |
| `gh issue list --label recurrent-incident` | дублей при потере state-файла (рестарт контейнера, сбой диска) | per-repo |

Три независимых уровня защиты покрывают разные сценарии отказа — аналогично ADR-FS-001 для fail-streak-watchdog.

## ENV-контракт (SOT для порогов и путей)

```
PROVIDER_EXHAUST_AUTO_ISSUE=0          # default OFF. Включить через env cron-job.
PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS=24  # cooldown между auto-issue (default 24ч)
RECURRENT_INCIDENT_LABEL=recurrent-incident  # лейбл нового issue
PROVIDER_EXHAUST_ISSUE_ASSIGNEES=      # comma-separated assignees (default: пусто)
ISSUE_COOLDOWN_FILE=$HERMES_HOME/state/agent-flow-cancel-provider-exhausted-last-issue
GH_REPO=krikz/rob_box_project          # для auto-issue
```

Любая правка дефолта = PR в `agent-flow-cancel-on-provider-exhausted.sh` + комментарий «env-контракт изменился» в этом ADR.

## Альтернативы (рассмотренные и отвергнутые)

| Альтернатива | Почему нет |
|--------------|-----------|
| Воркеры открывают incident-issue вручную | Ретро t_4aaeeef6: воркеры НЕ открывают, потому что «это не кодовая задача» |
| Открывать issue на КАЖДОЙ cancel-карточке отдельно | Шторм из 100+ issues за ночь; Шифу не нужен список из 50 incident-ссылок |
| Cooldown 4ч (как у fail-streak) | Ретро t_4aaeeef6: 14ч-блокировка фаз #3014 (несколько рецидивов в окне). 4ч → 1 issue за ночь (спам); 24ч → 1 issue за инцидент |
| Только local mtime без gh-truth | Потеря state-файла = шторм при первом же restart (как ADR-FS-001) |
| Только gh-truth без local | HA-окружения с двумя scheduler'ами могут создать 2 issue одновременно |
| Email/Slack алерт | Вне scope cron-инфраструктуры, требует секретов в env крона |

## Pitfalls

- **Не включать `PROVIDER_EXHAUST_AUTO_ISSUE=1` сразу после merge.** Сначала проверить через `--dry-run`, что cancel-flow работает корректно (тесты T1-T8 в `test_cancel_provider_exhausted.sh`), и что cooldown-файл создаётся в правильном месте. Dry-run покажет логи «AUTO_ISSUE [DRY-RUN] would: gh issue create ...» — это нормально.
- **`GH_CONFIG_DIR` обязателен.** Без него `gh auth status` упадёт, auto-issue скипнет gracefully (логика `AUTO_ISSUE: gh auth failed — skip`). В cron-job install.sh env-блок уже настроен (см. `ensure_cancel_provider_exhausted_cron`).
- **Recovery-фаза остаётся отдельной.** После пополнения MiniMax Шифу запускает `bash scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh --recover` (НЕ --auto-issue). Это разблокирует карточки в `ready` через `kanban unblock`. Auto-issue при этом **НЕ** создаёт новых issues (это уже закрытый инцидент).
- **Лейбл `recurrent-incident` отделён от `e2e-fail-streak`.** Это разные типы сбоев: provider-exhaust — внешняя (нужно действие Шифу); fail-streak — внутренняя (нужен код-фикс). Шифу может фильтровать через label по типу инцидента.

## Тест-контракт

Регресс-тесты (оба обязаны быть зелёные после любой правки):

- `scripts/agent_flow/tests/test_cancel_provider_exhausted.sh` — T1-T9 (cancel-логика + auto-issue gate): 9 кейсов.
- `scripts/agent_flow/tests/test_cancel_provider_exhausted_auto_issue.sh` — S1-S10 (auto-issue payload/gh-truth/cooldown/multi-assignee/dry-run/gh-fail): 10 кейсов.

Оба должны запускаться в pre-PR pipeline (см. `validate_honesty.sh`, `validate_test_ws_dirs.py`).

## Где это уже записано (после merge)

- `scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh` — auto-issue-функция + env-контракт + help.
- `scripts/agent_flow/tests/test_cancel_provider_exhausted_auto_issue.sh` — 10 регресс-сценариев.
- `docs/adr/0019-recurrent-provider-exhaustion-auto-issue.md` — этот документ.

## Связанные

- ADR-0018 (честный FAIL лучше красивого PASS) — культурный контекст: воркеры обязаны сигналить о проблеме, а не молча «ждать».
- ADR-FS-001 (fail-streak auto-issue) — прямой прецедент: ровно тот же паттерн для другого типа сбоя.
- ADR-0014 (agent-flow issue closure) — формализует, что auto-issue живёт только до ручного merge Шифу (--recover → unblock).
- Issue #1193 — root cause: исчерпание MiniMax Token Plan (закрыт completed, fallback на deepseek сработал).
- Issue #3378 — триггер этой работы.

## Что НЕ делаем

- ❌ НЕ открываем issue без явного `PROVIDER_EXHAUST_AUTO_ISSUE=1` (safe-by-default).
- ❌ НЕ используем cooldown < 4ч (шторм из issues заполнит Шифу inbox).
- ❌ НЕ блокируем воркеры на этом fix (это operational enhancement, не code-task).
- ❌ НЕ авто-мержим PR с auto-issue фиксом (Шифу решает, merge — ручной).

## Заключение

Incident 2026-10-03 показал **process-gap**: cancel-flow работает, но incident-tracking issue не создаётся → Шифу не видит сигнал. Auto-issue закрывает gap через тот же паттерн, что ADR-FS-001 для fail-streak: cooldown file + gh-truth guard + env-gated default-OFF. Включается Шифу руками через env cron-job после merge.