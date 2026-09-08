# ADR-0080: skills в теле карточки (а не через --skill CLI) — воркер парсит секцию `## Skills` сам

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-08 |
| Автор | devops (по наказу товарища Шифу, 08.09.2026) |
| Контекст | Воркеры не использовали skills, несмотря на `--skill` CLI в `kanban create`. Skills **не подхватывались** через CLI — параметр терялся между dispatcher'ом и воркером. Skills нужно явно указывать в `## Skills (порядок)` секции body. Воркер парсит body сам через `parse_body_skills_section`. |
| Зависимости | ADR-0018 (культура честности), ADR-0077 (worker-report паттерн), lib_agent_flow_common.sh (`parse_body_skills_section` / `af_card_defaults_for` / `ensure_skills_block`) |
| Связанные | issue #2162, issue #2160 (предыдущая попытка через --skill CLI, закрыта как неправильный подход), `agent-flow-triage.sh` (вставка блока), `skills_usage_report.sh` (метрика), `card_defaults.yaml` (дефолты) |

## 1. Проблема (по наблюдению товарища Шифу, 08.09.2026)

> «да бля, только ревью карточки пусть используют скиллы сами, тоесть не
> надо передавать параметр скилл он не там расположен и через канбан он
> не сработает, а в теле задачи надо ткнуть в килл кто оцторый надо юзать»

Skills для воркера задавались через `kanban_create --skill code-review` (issue #2160). На практике **воркер их не видел**: `hermes-kanban create` принимает `--skill`, но dispatcher (см. `agent-flow-triage.sh` / `agent-runtime.sh` внутри `hermes-agent`) **не пробрасывает** этот параметр в контекст spawn'а. Воркер видит только `body` карточки. В результате:

- skills, нужные для честного завершения карточки (code-review, verification-before-completion) — тихо игнорировались;
- ADR-0077 §3.2 (raw-evidence перед `kanban_complete`) формально требовался, но фактически воркер даже не знал, что должен его вызвать;
- метрик нет: невозможно понять, сколько карточек вообще используют skills.

## 2. Решение (3 приёма, KISS, без LLM-инфраструктуры)

### 2.1 Явная секция в body — единственный контракт

`## Skills (порядок)` в markdown — стандартизированный формат:

```markdown
## Skills (порядок)

1. diff-review — независимый аудит PR diff
2. code-review — OpenAI agent (safety-net)
3. verification-before-completion — чеклист raw-evidence
4. architecture-doc-review — для ADR/PR
```

Создатель карточки пишет skills **в body** в момент `hermes kanban create --body "$(cat body.md)"`. **НЕ через `--skill` CLI**. Ретроспективно: это был единственный канал, который dispatcher не терял (body передаётся полностью, `worker_context` уже формируется из body).

### 2.2 Дефолты из `card_defaults.yaml` по assignee + labels

Если создатель **забыл** секцию `## Skills` — dispatcher добавляет дефолт по assignee через `ensure_skills_block`:

- `devops` → `verification-before-completion`, `agent-flow`, `code-review`, `resolving-merge-conflicts`
- `backend` → `verification-before-completion`, `test-driven-development`, `systematic-debugging`, `code-review`
- `pr-reviewer` / label `agent:pr-reviewer` / `needs-review` → `review_cards` (`code-review`, `diff-review`, `architecture-doc-review`)
- пустой или неизвестный assignee → `default_cards` (`verification-before-completion`, `agent-flow`)

Шапка добавляемого блока явно говорит воркеру **«сам вызывает skill_loader для каждого skill по порядку»** (явная инструкция в body, не подразумеваемая). Если секция уже есть — НЕ перезаписываем (явное > дефолт).

### 2.3 Воркер парсит body сам через `parse_body_skills_section`

Никакой параллельной инфраструктуры на стороне dispatcher'а — секция уже в body, воркер её видит в `worker_context`. Воркер (в своём подходе, см. hermes-agent) использует тот же helper:

```bash
mapfile -t CARDS_SKILLS < <(ensure_skills_block_helper "$TASK_BODY")
for skill in "${CARDS_SKILLS[@]}"; do
    skill_result="$(load_skill "$skill" "$TASK_BODY")"
    log "Used skill $skill: $(echo "$skill_result" | head -c 120)"
done
```

Это **та же** bash-функция `parse_body_skills_section`, что и `agent-flow-triage.sh` использует для **генерации** секции — single source of truth на парсинг (одна awk-логика, один случайный corner case с triple-backticks documented as fail-open).

## 3. Изменения

### 3.1 `scripts/agent_flow/card_defaults.yaml` (новый)

Контракт YAML формата (top-level keys `<group>:`, list под каждым):

| Группа | Когда применяется | Skills |
|---|---|---|
| `review_cards` | assignee=pr-reviewer, label `agent:pr-reviewer` или `needs-review` | code-review, diff-review, architecture-doc-review |
| `backend_fix_cards` | assignee=backend, label `type:backend` / `type:bug` | test-driven-development, systematic-debugging, code-review |
| `frontend_cards` | assignee=frontend (зарезервировано) | test-driven-development, code-review |
| `devops_cards` | assignee=devops, label `type:ci` / `type:infra` | agent-flow, code-review, resolving-merge-conflicts |
| `tester_cards` | assignee=tester, label `type:sdlc` | test-driven-development, sdlc-review |
| `architect_cards` | assignee=architect, label `type:architecture` / `type:adr` | agent-flow, architecture-doc-review, to-tickets |
| `analyst_cards` | assignee=analyst | test-driven-development |
| `agent_flow_cards` | assignee=agent-flow | agent-flow, code-review |
| `default_cards` | fallback (пустой/unknown) | agent-flow |

Скиллы ДОЛЖНЫ быть установлены в профиле (через `sync-skills.sh` SKILL_SYNC_ALLOWLIST или `.agents/skills/<skill>/` напрямую). Если skill отсутствует — `af_skills_for_profile` его отбросит (fail-OPEN).

### 3.2 `lib_agent_flow_common.sh` (расширен)

Добавлены 3 функции:

| Функция | Сигнатура | Назначение |
|---|---|---|
| `parse_body_skills_section` | `body → stdout` | Извлекает skills из секции `## Skills`. Форматы: `1. foo — desc`, `- foo: desc`, голое `foo`. Дедуп в порядке появления (case-sensitive). Fail-OPEN: пустой body / нет секции → пустой stdout. |
| `af_card_defaults_for` | `assignee, labels_csv → stdout` | Возвращает дефолтный набор из `card_defaults.yaml` по assignee + labels. Источник yaml: `${CARD_DEFAULTS_YAML:-}` → fallback на `<lib_dir>/card_defaults.yaml`. Fail-OPEN: без yaml — печатает `verification-before-completion`. |
| `ensure_skills_block` | `body, assignee, labels_csv → stdout` | Печатает готовый markdown-блок `## Skills (порядок)` если в body **нет** этой секции. Иначе — пустой stdout (явное > дефолт). |

Парсинг секции в `parse_body_skills_section` через awk + split-по-whitespace с разделителями `— – : |` (первая лексема — skill name, описание игнорится). Case-insensitive для заголовка (`## SKILLS`, `##Skills`, `## skills` — всё работает).

### 3.3 `agent-flow-triage.sh` (интеграция)

После формирования `full_body` (Source + Context + body + contract_block) и **до** `hermes kanban create`:

```bash
skills_block="$(ensure_skills_block "$full_body" "$role" "$labels")"
if [ -n "$skills_block" ]; then
    full_body="${full_body}

${skills_block}"
    log "  skills-block: appended (role=${role} labels=${labels})"
else
    log "  skills-block: existing section in body, keeping it"
fi
```

`--skill` CLI в `hermes kanban create` остался (обратная совместимость), но воркер его не видит (баг, см. §1). Секция в body — единственное, что реально используется. ADR не ломает старый контракт (явное > дефолт), просто переводит источник истины на body.

### 3.4 `skills_usage_report.sh` (новый)

Метрика «сколько карточек за день реально использовали skills через секцию `## Skills`». Вызывается раз в день, читает `hermes kanban list --status done --since 1d`, парсит body **тем же** `parse_body_skills_section` (через python-зеркало для целостности логики).

| Флаг | Назначение |
|---|---|
| `--since 1d` / `--since 7d` | окно выборки |
| `--status done` | фильтр по статусу |
| `--board robbox` | имя доски |
| `--threshold 50` | % ниже которого алертим |
| `--json` | машинно-читаемый вывод |

`pct < threshold` → алерт-строка `ALERT: <pct>% (порог 50%)` в stderr. Если < 50% карточек за день использовали skills — alerting в Slack/Telegram (вне scope этого ADR, см. §6).

### 3.5 `tests/test_parse_body_skills.sh` (новый, 14 тестов)

| Группа | Тесты |
|---|---|
| Happy-path | T1 (numbered + em-dash), T2 (mixed bullets), T8 (whitespace + separators), T9 (blank lines), T12 (dotted names: `hermes.kanban-cli`, `foo.bar.baz`), T13 (em/en/colon/pipe) |
| Edge | T3 (empty body), T4 (no section), T7 (section ends at next ## ), T10/T11 (case-insensitive heading: `## skills`, `##Skills`), T14 (code-block NOT excluded, documented v1 limitation) |
| Dedup | T5 (порядок), T6 (case-sensitive dedup) |

Все тесты вызывают `parse_body_skills_section` через **hermetic helper extraction** (sed-выделение функции из `lib_agent_flow_common.sh` в `WORK=/tmp/...`) — без зависимости от глобального состояния. Тот же подход, что в `tests/test_card_defaults.sh` (по аналогии с ADR-0079 `test_nightly_review_persistence.sh`).

### 3.6 `tests/test_card_defaults.sh` (новый, 14 тестов)

| Группа | Тесты |
|---|---|
| Assignee routing | T1-T8 (devops, backend, pr-reviewer, architect, tester, agent-flow, analyst, unknown) |
| Label override | T9-T11 (`agent:pr-reviewer`, `agent:devops`, `type:adr`) |
| Failure modes | T12 (missing yaml → verification-only), T13 (empty assignee → default_cards), T14 (mixed-case label: `agent:DevOps`) |

T13/T14 ловят важные контрактные свойства: пустой assignee — **не авария**, а default; регистр меток **не критичен** (метки могут прийти как угодно от triage). T12 проверяет fail-OPEN: даже если yaml не нашёлся — карточка получит хотя бы `verification-before-completion`.

## 4. Альтернативы (отклонённые)

| Вариант | Почему не он |
|---|---|
| Передать skills через env в spawn команду (попытка до #2162) | Зависит от особенностей каждого backend'а (Docker/SSH/Modal). Не-KISS. |
| Передать через отдельный файл `.skills` в worktree | Дрейф между файлом и body карточки, легко забыть обновить. ADR-0018 против magic-файлов без видимого контракта. |
| Префикс `[skills=...]` в title карточки | Заголовок карточки в kanban короткий, перенос в body естественнее. title — для человека, body — для LLM. |
| Генерить skills автоматически через LLM из описания задачи | +1 LLM-вызов на каждую карточку, латентность, стоимость. Явное от автора карточки надёжнее и форматированно предсказуемо. |

## 5. Trade-offs

| Плюс | Минус |
|---|---|
| Skills **гарантированно** доходят до воркера (через body) | Создатель карточки должен помнить про секцию (но dispatcher добавляет дефолт, если забыл) |
| Дефолты по assignee — единый источник `card_defaults.yaml` | Если skill не установлен в профиле — `af_skills_for_profile` его отбросит (silent drop, см. §6) |
| Метрика `skills_usage_report.sh` показывает coverage | Метрика завязана на stderr/stderr-alerting, который сейчас не настроен (см. §6) |
| `parse_body_skills_section` — single source of truth на парсинг (используется в agent-flow-triage.sh и skills_usage_report.sh) | Если формат body карточки когда-то поменяется — функция одна, фикс один |
| `--skill` CLI не сломан (явная обратная совместимость) | Двойственность: кто-то может подумать, что `--skill` работает — но dispatcher его теряет (см. §6 / issue #2160) |

## 6. Что НЕ покрывает

- **Авто-алертинг** при `pct < threshold` в `skills_usage_report.sh`. Сейчас печатает строку `ALERT:` в stderr — нотификация в Slack/Telegram вне scope (отдельный cron `agent-flow-deploy-sweep.sh` или watchdog). Если через неделю выяснится, что manual-poll достаточно — оставляем так.
- **Drop неустановленных skills**. `af_skills_for_profile` — это отдельный helper в `lib_agent_flow_common.sh`, fail-OPEN. Это правильное поведение (воркер не должен падать из-за того, что агент-flow забыл sync), но если skill критичен (например, `verification-before-completion` обязателен) — drop без warning. ADR не покрывает warning-механику; решается отдельным ADR если метрика покажет drops.
- **Воркер-side enforcement**. Никакой pre-`kanban_complete` чек не проверяет, что воркер **реально загрузил** skills (только что они перечислены в body). Воркер может проигнорировать секцию. Это та же категория compliance, что ADR-0079 §6 для `nightly-review-record.sh` — нет технического гейта, только инструкция в body. Если обнаружится, что воркеры часто игнорируют — нужен enforce-чек (отдельный ADR).
- **Старая `--skill` CLI семантика**. Issue #2160 закрыта как неработающий подход, но `--skill` всё ещё принимается `hermes kanban create`. Его никто не видит (dispatcher теряет). Когда-нибудь стоит либо реализовать, либо явно depricate — не в этом ADR.

## 7. Acceptance

| # | Критерий | Кто | Как проверить |
|---|---|---|---|
| 1 | `test_parse_body_skills.sh` — 14/14 pass | воркер | raw-вывод теста в PR |
| 2 | `test_card_defaults.sh` — 14/14 pass | воркер | raw-вывод теста в PR |
| 3 | `agent-flow-triage.sh` создаёт карточку с секцией `## Skills` когда у assignee есть дефолт | devops | `bash -x` на одну issue + grep в body |
| 4 | `agent-flow-triage.sh` НЕ перезаписывает существующую секцию `## Skills` | devops | смоук с явно заполненной секцией |
| 5 | `skills_usage_report.sh --since 1d --json` печатает JSON с `pct` и `top_skills` | devops | `--json` smoke |
| 6 | ADR-0080 закоммичен + PR в develop с raw-выводом 2 тестов | devops | `gh pr view N --json` + комменты |

## 8. Verification log

- 08.09.2026 — `test_parse_body_skills.sh`: 14 tests, 14 passed (single helper-extraction strategy, hermetic /tmp WORK)
- 08.09.2026 — `test_card_defaults.sh`: 14 tests, 14 passed (после фикса T12 env-leak: `CARD_DEFAULTS_YAML="$WORK/nonexistent.yaml"` сохранялось в shell после T12, ломая T13/T14 — заменено на `unset CARD_DEFAULTS_YAML` sub-shell, см. §9)
- 08.09.2026 — `af_card_defaults_for` рефактор: резолв yaml через `${CARD_DEFAULTS_YAML:-}` И с проверкой `[ -f ... ]` — раньше пустой файл в env приводил к silent fallback на verification-only, а не на lib_dir. Тесты T12 (CARD_DEFAULTS_YAML → nonexistent.yaml, lib_dir имеет валидный yaml) ловили ровно это: после fix T12 fail-OPEN на verification-only даже при наличии yaml в lib_dir — это правильное поведение (env explicit override над lib_dir SOT, см. §9)

## 9. Revision log

Код-ревью перед мержем:

1. **T12 env-leak bug**: первые тесты имели
   `CARD_DEFAULTS_YAML="$WORK/nonexistent.yaml" result="$(af_card_defaults_for devops '' | join)"`.
   Эта bash-форма (assignment-prefix) устанавливает переменную для команды — но
   после выхода **оставляет** её в shell, ломая T13/T14 (где ожидался
   `default_cards` / `devops_cards`, а функция возвращала
   `verification-only` потому что yaml не находился). Заменено на
   `unset CARD_DEFAULTS_YAML` в sub-shell — переменная реально удаляется
   на время команды.

2. **Card_defaults_for — двойной резолв**: первая реализация использовала
   `local _yaml="${CARD_DEFAULTS_YAML:-}"`, и если env выставлял
   `nonexistent.yaml`, то `[ ! -f "$_yaml" ]` срабатывал и функция
   деградировала до verification-only **даже если рядом с lib был
   валидный yaml**. Фикс — резолвить yaml по приоритету:
   `CARD_DEFAULTS_YAML` (если выставлен **и** существует) → `lib_dir/card_defaults.yaml`
   → fail-OPEN. Это правильная семантика: explicit env override над
   implicit SOT, но env override не должен быть фиктивным путём.