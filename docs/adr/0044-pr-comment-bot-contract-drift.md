# ADR-0044: PR-comment bot — contract-drift vs rebase-candidate классификация красного CI

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-01 |
| Автор | devops (Hermes Agent); ретро-карточка `t_527e1231`, process-fix `t_58c69473` |
| Родители | ADR-0035 (stale-after-upstream-fix auto-detect), ADR-0036 (mis-scope task guard), ADR-0030 (ADR-нумерация SOT) |
| Затрагивает | (a) `.github/workflows/G-PR-Contract-Drift-Bot.yml` (новый); (b) `scripts/agent_flow/agent-flow-merge-gate.sh` (`pr_classify_rollup` / `pr_failed_rollup_md`, блок B этой же ретро); (c) карточки-«rebase PR #N» — их body и assignee |
| Связанные | `t_002aae48` (mis-scope: rebase-карточка вместо fix-карточки), `t_0fb7ac48` (правильное описание фикса, но assignee=default), `t_6d19aace` (ретро «contract drift после 9 rebase-циклов»), PR #1857, PR #1849, issue #1810/#1811 |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (01.09.2026)

PR #1857 (`z-architect/t_5d93c7b1-adr-0040-provider-sync`) стоял в состоянии
`mergeable=MERGEABLE` + `mergeStateStatus=UNSTABLE`. Единственный красный чек —
**Unit Tests (ROS2 Humble)**, job `99842198694`: тест
`test_resolve_provider_chain_parses_csv` ожидал старый порядок провайдеров, а
реализация в этом же PR уже отдавала новый. То есть **contract drift ВНУТРИ
PR** — реализация и тест разошлись в одном и том же диффе.

Процесс отреагировал так, будто PR просто отстал от develop:

1. merge-gate `scan-all-prs` создал карточку `t_002aae48` — «Rebase PR #1857 на
   develop и вернуть CI в green», `assignee=default`.
2. `default`-профиль не имеет скиллов под разбор упавшего юнит-теста → карточка
   провисела 1.6ч в `ready` и не двинулась.
3. Ретро-карточка `t_0fb7ac48` с ПРАВИЛЬНЫМ описанием фикса (assert-строка,
   файл, номер строки) получила `assignee=default` — её тоже никто не взял.
4. Параллельно ретро `t_6d19aace` уже фиксировала этот класс: «contract drift
   после 9 rebase-циклов» — то есть 9 rebase-ов подряд не могли починить то,
   что rebase-ом не чинится.

### 1.2 Почему rebase — неверный дефолт

`rebase на origin/develop` лечит ровно один класс: **PR отстал, фикс уже в
develop**. Для contract drift внутри PR rebase:

- не меняет упавший assert (он в этом же PR);
- сжигает CI-прогон (~2-4 мин × N попыток);
- порождает force-push → новую итерацию CI → новый красный тик merge-gate;
- создаёт ложный сигнал «работа идёт», из-за которого карточка не эскалируется.

### 1.3 Почему это не поймали существующие слои

| Слой | Что должен ловить | Почему не сработал на PR #1857 |
|---|---|---|
| main-cycle `pr_classify_failure` (ADR §PR #1743) | UNSTABLE → diagnostic vs rebase по check-runs | Работает по `head_oid` внутри цикла по **issues с меткой `hermes`**. У PR #1857 hermes-issue нет → цикл до него не доходит. |
| `scan-all-prs` | Любой OPEN PR с UNSTABLE/CONFLICTING | Классификации не делал вообще: любой UNSTABLE → rebase-карточка, `assignee` по метке issue, а issue нет → `default`. |
| ADR-0035 stale-after-upstream-fix | Авто-блок карточек, чья работа уже в develop | Здесь противоположный случай: фикса в develop НЕТ, он нужен внутри PR. |
| ADR-0036 mis-scope guard | Архитектурная карточка ушла не тому профилю | Ловит по признаку «body архитектурный», а тут body — рабочий rebase-чеклист. |

## 2. Решение

Два независимых слоя: **процессный** (merge-gate, уже реализован блоком B этой
ретро) и **человеческий** (PR-comment bot, этот ADR).

### 2.1 Классификация (общий контракт для обоих слоёв)

По именам красных чеков (`statusCheckRollup[].conclusion ∈ {FAILURE,
TIMED_OUT, CANCELLED}`), регистронезависимо:

```
contract_drift    unit | lint | build | pytest | mypy | ruff | flake8 | black |
                  coverage | test summary | code quality | dockerfile | yaml
rebase_candidate  integration | e2e | deploy | docker build | release | smoke
unknown           rollup недоступен / красных чеков нет
```

Правила:

- смесь категорий → **contract_drift** (худший случай: лечим код, не rebase-им);
- `unknown` → **fail-open**: старое поведение (rebase-карточка, `assignee=default`),
  никаких новых блокировок.

### 2.2 Слой 1 — merge-gate (реализовано, блок B `t_58c69473`)

Для OPEN PR с `mergeStateStatus=UNSTABLE` (НЕ `CONFLICTING`/`DIRTY`):

- `contract_drift` + нет метки `agent:*` на issue → `assignee=backend`
  (метка `agent:*` всегда сильнее эвристики);
- в body карточки обязательный блок `## Contract-drift pre-check` со списком
  failing jobs (имя + `detailsUrl`) и порядком работы «сначала разбери drift,
  rebase — только если фикс реально в develop»;
- `rebase_candidate` / `unknown` → поведение без изменений.

Функции: `pr_classify_rollup()`, `pr_failed_rollup_md()` в
`scripts/agent_flow/agent-flow-merge-gate.sh`.
Тесты: `scripts/agent_flow/tests/test_merge_gate_rebase_card_contract_drift.sh`
(B1-B5).

### 2.3 Слой 2 — PR-comment bot (этот ADR)

Workflow `.github/workflows/G-PR-Contract-Drift-Bot.yml`:

- **Триггер**: `workflow_run` (`completed`) по CI-workflow репозитория
  (`G: Run Tests`, `G: Lint Code`, `G: TTS Provider Tests`) + `workflow_dispatch`
  с `pr_number` для ручного прогона/отладки.
- **Условие работы**: PR OPEN, `mergeStateStatus == UNSTABLE`, хотя бы один
  красный чек, и классификация ≠ `rebase_candidate`-only.
- **Действие**: ОДИН комментарий на PR вида
  «CI failing: rebase candidate (X jobs) vs contract-drift inside PR (Y jobs)»
  + список jobs по категориям + инструкция.
- **Дедуп**: маркер `<!-- contract-drift-bot -->` в теле; перед постингом
  ищем свой комментарий за последние `DEDUP_HOURS=6`. Если он есть — обновляем
  (`PATCH`), а не плодим новый. Так же, как post-merge cleanup в merge-gate
  (24h-дедуп) и pr-backfill (30-мин порог).
- **Права**: `pull-requests: write`, `contents: read`, `actions: read`.
  Никаких `gh issue close`, никаких меток — бот **только информирует**.

### 2.4 Почему бот, а не только merge-gate

merge-gate пишет в kanban-карточку — это видит воркер. Комментарий на PR видит
**человек** (Шифу) в GitHub-ленте, без захода на доску. Ретро `t_6d19aace`
(9 rebase-циклов) показала: пока сигнал живёт только в карточке, человек в
ревью PR не понимает, почему PR «мигает красным» и продолжает просить rebase.

## 3. Инварианты (что бот НЕ делает)

- НЕ ставит и НЕ снимает метки (`needs-e2e`, `e2e-done`, …) — это territory
  merge-gate/e2e-process (ADR-0014, ADR-0022).
- НЕ закрывает issues и PR.
- НЕ создаёт kanban-карточки (иначе дубли с merge-gate блоком B).
- НЕ комментирует `CONFLICTING`/`DIRTY` PR: там rebase — корректный ответ, и
  merge-gate уже пишет rebase-инструкцию.
- НЕ комментирует, если классификация `unknown` (fail-open).
- Один комментарий на PR в окне `DEDUP_HOURS` (обновление вместо дубля).

## 4. CI-only PR: метка `no-e2e-required`

Реализация этого ADR — CI/процесс (workflow + скрипт + тесты), функциональный
код робота не затрагивается, e2e-прогон на железе не нужен. PR c реализацией
помечается `no-e2e-required` (ADR-0022 §4.2) — merge-gate закроет issue после
merge по no-e2e-пути.

## 5. Acceptance

- [x] `pr_classify_rollup()` / `pr_failed_rollup_md()` в merge-gate + тесты B1-B5.
- [x] contract_drift → `assignee=backend`, `## Contract-drift pre-check` в body.
- [x] `agent:*` метка сильнее эвристики (B4), `unknown` → fail-open (B5).
- [x] CONFLICTING → блок не добавляется (B3).
- [x] `.github/workflows/G-PR-Contract-Drift-Bot.yml` с дедупом по маркеру.
- [x] Этот документ (`docs/adr/0044-pr-comment-bot-contract-drift.md`).
- [x] Номер 0044 не коллизирует: в `origin/develop` максимум 0042, 0043 занят
      открытым PR #1857 (ADR-0030 §ADR-нумерация SOT).

## 6. Что осталось за рамками

- Автоматическое исправление drift (агент, который сам правит assert) — нет,
  это работа воркера по карточке.
- Классификация по содержимому лога (assertion diff) вместо имени job —
  возможное развитие; пока имя job даёт достаточную точность и не требует
  скачивания логов.
