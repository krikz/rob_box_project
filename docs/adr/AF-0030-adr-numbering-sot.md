# ADR-AF-0030: ADR-нумерация — два домена AF/RT, pre-merge guard, ручной коммит запрещён

| Поле | Значение |
|---|---|
| Статус | **Accepted** (Phase 1 — PR #2073 в develop 07.09; Phase 2 — issue #2076 в этом PR) |
| Дата | 2026-08-25 (Phase 1), 2026-09-07 (Phase 2: AF/RT split) |
| Автор | architect (Hermes Agent); ретро-карточка `t_45db74ad` (Phase 1), `t_5055c13c` (Phase 2) |
| Контекст | Коллизия ADR-номеров в `origin/develop` (на 25.08): 5 файлов под 3 номерами — `0027×3`, `0028×2`. Никакой процесс нумерации ADR сегодня не enforced: ни merge-gate, ни CONTRIBUTING, ни ADR-0001. Phase 2 (07.09): 13 коллизий, плюс обнаружилось что одноимённый номер может указывать на совершенно разные домены (агент-flow процесс vs рантайм робота). Принятая схема (issue #2076) — **два независимых домена** AF/RT с префиксом. |
| Затрагивает | `CONTRIBUTING.md` (секция "ADR-нумерация: два домена"), `scripts/agent_flow/agent-flow-merge-gate.sh` (pre-merge guard), `scripts/agent_flow/validate_adr_namespace.sh` (pre-PR guard с поддержкой двух доменов, Phase 2), `docs/adr/*` (16 файлов переехали в AF-домен, 6 получили новые номера для разрешения внутридоменных коллизий) |
| Родители | ADR-AF-0018 (честность), ADR-AF-0026 (recovery contract — worker отвечает за следствие), `t_45db74ad` (Phase 1 retro), `t_5055c13c` (Phase 2 retro) |
| Связанные | Phase 1: PR #1577, #1578, #1580, #1581, #1584; ручной коммит `955cbf58` (Denis 24.08 23:40); `t_45db74ad`. Phase 2: PR #2073 (Phase 1 в develop), issue #2076 (Phase 2 — этот PR) |

## 1. Контекст и бизнес-проблема

### 1.1 Phase 1 (25.08): глобальная коллизия

`git ls-tree -r origin/develop --name-only | grep -oE 'docs/adr/[0-9]{4}' | sort -u` (25.08 2026):

```
0027-harness-improvements-from-external-talk.md        (PR #1580, d70a316d, Denis 24.08 10:39)
0027-meta-quest-ar-control.md                          (PR #1578, 50d0b4f9, Denis 24.08 15:51)
0027-systemic-wake-gate-no-wake-word-blocker.md        (PR #1577, 5bdccfbb, Denis 24.08)
0028-avatar-supervisor.md                              (955cbf58, GOODWORKRINKZ/Denis 24.08 23:40, ручной коммит)
0028-harness-5-improvements.md                         (PR #1581, 8b12e8ec, Denis 24.08)
0029-wake-gate-cold-start-known-state.md               (PR #1584, 7fc8f5a9, Denis 24.08)
```

5 файлов под 3 номерами. Это **не** одна аномалия — это симптом: процесс нумерации ADR в проекте сегодня **полностью отсутствует** как enforced-инвариант.

### 1.2 Phase 2 (07.09): неполнота Phase 1

PR #2073 (07.09) закрыл Phase 1 — добавил `validate_adr_namespace.sh` как hard gate в CI. Это решило **будущие** коллизии (любой новый ADR-файл с занятым номером теперь блокируется). Но **существующие** коллизии в `origin/develop` остались:

- 13 коллизий в 9 номерах: 0009×2, 0013×2, 0016×3, 0021×2, 0024×2, 0026×2, 0027×3, 0028×2, 0030×2, 0032×2, 0052×5 (sic!), 0054×2, 0055×3.
- Слово **«harness»** в репозитории имеет ДВА разных значения: (а) рантайм-пакет `rob_box_harness` (ADR-0001, ADR-0009 — TTSProvider contract); (б) AI-харнес агентов / Hermes (ADR-0027 «анализ внешнего опыта построения AI-harness», ADR-0028 «subagent-review», ADR-0024 `OUT_DIR/verdict.txt`). **Классифицировать по слову в имени нельзя — только по содержанию**.
- Phase 1 не различал домены. Guard ловил `0027` × любой файл — но оба `0027-systemic-wake-gate` (рантайм) и `0027-harness-improvements` (процесс) попадали под один namespace.

### 1.3 Почему два домена, а не глобальный счётчик

Phase 1 предполагал, что **один ADR-номер = один файл** в develop. Phase 2 показал, что это false: под номером `0027` живут два РАЗНЫХ решения — про наблюдение wake-gate на роботе (рантайм) и про улучира AI-харнеса (процесс). Они не конфликтуют по смыслу, только по имени файла.

**Принятое решение (Phase 2, issue #2076, владелец 07.09):** разделить пространство на два независимых домена — `ADR-AF-NNNN` (agent-flow / процесс) и `ADR-NNNN` (рантайм робота). Документ при переезде сохраняет свой номер и получает префикс. Внутри домена — монотонный счётчик, как в Phase 1.

## 2. Принятое решение

### 2.1 Два домена: AF и RT

**Правило (Phase 2).** В `docs/adr/` живут два независимых пространства:

```
docs/adr/AF-NNNN-<slug>.md   → домен agent-flow (процесс, воркеры, AI-харнес)
docs/adr/NNNN-<slug>.md       → домен рантайма (голос, Quest, perception, supervisor, harness runtime)
```

- `NNNN` ∈ `[0001, 9999]`, **уникален внутри своего домена** на момент merge.
- `<slug>` — kebab-case, описывает решение.
- Имя файла совпадает с заголовком H1: `# ADR-AF-NNNN: <slug>` или `# ADR-NNNN: <slug>`.

**Какой домен выбрать (decision rule):**

| Если ADR описывает... | Домен |
|---|---|
| triage / merge-gate / e2e / воркеры / kanban / карточки / AI-харнес агентов | AF |
| голос / Quest / perception / supervisor / `rob_box_harness` (рантайм) / navigation | RT |

Классификация — **по содержанию**, не по ключевому слову в имени файла. Слово `harness` в этом репозитории неоднозначно (см. §1.2).

### 2.2 Как выбрать NNNN перед созданием

```bash
git fetch origin develop

# Какие номера заняты в AF-домене
git ls-tree -r origin/develop --name-only \
  | grep -oE 'docs/adr/AF-[0-9]{4}' | sort -u

# Какие номера заняты в RT-домене
git ls-tree -r origin/develop --name-only \
  | grep -oE 'docs/adr/[0-9]{4}' | grep -v '/AF-' | sort -u

# Следующий свободный (для своего домена)
NEXT=$(($(git ls-tree -r origin/develop --name-only \
          | grep -E "docs/adr/${MY_DOMAIN}[0-9]+" \
          | sed -E "s|docs/adr/${MY_DOMAIN}([0-9]+).*|\1|" \
          | sort -n | tail -1) + 1))
printf '%04d\n' "$NEXT"
```

`MY_DOMAIN` — `AF-` для агент-flow, пустая строка для рантайма.

**Запрещено:**
- Использовать номер, не проверив `origin/develop`.
- Использовать номер, который уже занят **в твоём домене** (даже если «логичный»).
- Сокращать (`27` вместо `0027`) — ломает grep-инвариант.
- Путать домена: AF-0052 (decomposed-watchdog) ≠ 0052 (mcp-slice-guard).

### 2.3 Cross-reference внутри ADR

При ссылке на другой ADR:

```markdown
См. [ADR-AF-0022 §4.6](../AF-0022-process-e2e-done-gates.md) (e2e done gates).
См. [ADR-0028 §4.5](../0028-avatar-supervisor.md) (avatar supervisor).
```

- Префикс AF обязателен для ссылок на agent-flow-документы.
- RT-ссылки (на `docs/adr/NNNN-*.md`) — без префикса.
- Cross-ref **должен** разрешаться в существующий файл на момент merge.
- Если cross-ref указывает на «будущий» ADR — это **TODO**, не cross-ref; пишите явно `TODO(ADR-AF-NNNN)` или `TODO(ADR-NNNN)`.

### 2.4 Pre-merge guard (hard gate в CI)

В `scripts/agent_flow/validate_adr_namespace.sh` (Phase 2 — поддержка AF) + `.github/workflows/G-Lint Code.yml` (hard gate в job `python-lint`):

- Для PR, которые создают **новый** файл `docs/adr/(AF-)?NNNN-*.md`:
  - Получить `<domain>:<NNNN>` из имени файла (AF → AF, иначе RT).
  - Сверить с `git ls-tree -r origin/develop --name-only | extract_keys`.
  - Если `<domain>:<NNNN>` уже есть **в том же домене** → **reject** с сообщением (collision-линии + next free slot для затронутого домена).
  - AF-NNNN vs NNNN — **разные** домены, коллизия **не считается**.

Ратилиция: `scripts/agent_flow/tests/test_validate_adr_namespace.sh` (13/13 регресс-тестов: 10 Phase 1 + 3 Phase 2 для AF-домена и cross-domain).

### 2.5 Ручной коммит в develop — запрещён

Любой коммит в `develop` (включая ручной от Шифу) **должен** идти через feature-branch + PR, **даже если коммит единственный**. Исключений нет. Включено в `CONTRIBUTING.md` как §2d-bis.

### 2.6 Phase 2: cleanup существующих коллизий (issue #2076)

Phase 1 cleanup **не входил** в ADR. Phase 2 (issue #2076, этот PR) делает:

- **AF-домен создан:** 16 файлов переехали в `docs/adr/AF-NNNN-<slug>.md` (см. таблицу ниже).
- **Внутридоменные коллизии разрешены** перенумерацией (правило владельца: документ с бо́льшим числом контекстных ссылок сохраняет номер; тай-брейк — более ранний коммит):
  - `AF-0016`: 3 файла → 1 primary + 2 перенумерованы (AF-0058, AF-0059).
  - `AF-0024`: 2 файла → 1 primary + 1 перенумерован (AF-0060).
  - `AF-0030`: 2 файла → 1 primary + 1 перенумерован (AF-0061).
  - `AF-0052`: 3 файла → 1 primary + 2 перенумерованы (AF-0062, AF-0063).
  - RT-домен: внутри AF-схемы владельца не требует переименования, т.к. контентные ссылки на каждый RT-номер однозначны (по контексту читателя).
- `0055-operator-tts-headset-channel-impl-plan.md` → перенесён в `docs/plans/2026-08-30-operator-tts-headset-channel-impl-plan.md` (это не ADR, был ADR-номер неправомерно).

**AF-список (по решению владельца в issue #2076):**

| Старое имя | Новое имя | Домен |
|---|---|---|
| `0009-integration-test-report` | `AF-0009-integration-test-report` | AF |
| `0013-incremental-delivery-over-big-bang` | `AF-0013-incremental-delivery-over-big-bang` | AF |
| `0016-stt-empty-rejection-known-noise` | `AF-0016-stt-empty-rejection-known-noise` (primary) | AF |
| `0016-nav2-odom-startup-race-fp-rule` | `AF-0058-nav2-odom-startup-race-fp-rule` | AF |
| `0016-health-monitor-scope-leak` | `AF-0059-health-monitor-scope-leak` | AF |
| `0024-harness-verdict-sot` | `AF-0024-harness-verdict-sot` (primary) | AF |
| `0024-worker-scope-cross-task-archive` | `AF-0060-worker-scope-cross-task-archive` | AF |
| `0026-recovery-card-contract` | `AF-0026-recovery-card-contract` | AF |
| `0027-harness-improvements-from-external-talk` | `AF-0027-harness-improvements-from-external-talk` | AF |
| `0028-harness-5-improvements` | `AF-0028-harness-5-improvements` | AF |
| `0030-adr-numbering-sot` | `AF-0030-adr-numbering-sot` (этот документ, primary) | AF |
| `0030-e2e-stale-branch-guard` | `AF-0061-e2e-stale-branch-guard` | AF |
| `0032-triage-dedup-guard` | `AF-0032-triage-dedup-guard` | AF |
| `0052-decomposed-children-wake-up-watchdog` | `AF-0052-decomposed-children-wake-up-watchdog` (primary) | AF |
| `0052-fan-out-dedup-by-file-overlap` | `AF-0062-fan-out-dedup-by-file-overlap` | AF |
| `0052-issue-auto-close-after-merge` | `AF-0063-issue-auto-close-after-merge` | AF |

## 3. Альтернативы, которые мы отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **Глобальный монотонный счётчик (Phase 1)** | Не различает домены: 0027-systemic-wake-gate (RT) и 0027-harness-improvements (AF) — это два РАЗНЫХ решения, которые не конфликтуют по смыслу. Phase 1 был неполон. |
| **Двух-уровневая нумерация (`0027-proc-...`, `0027-quest-...`)** | Хрупка (что считать «поддоменом»?), ломает существующие ADR-номера, ADR-0001 принят без такого префикса. |
| **Использовать существующие 0027/0028 как есть, пометить «DEPRECATED»** | Не решает cross-ref: ADR-0027 по-прежнему неоднозначен. Deprecation — лечение симптома, не причины. |
| **Удалить дубликаты автоматически (most recent wins)** | Уничтожает решение. ADR — задокументированное решение, а не временный файл. Авто-удаление без ревью — нарушение ADR-AF-0018. |
| **Не вводить guard — ограничиться документированием правила** | Не работает: 6 попыток ретро этой карточки (`t_45db74ad`, runs 2397–2472) показали, что без enforced-инварианта процесс не держится. |
| **External ADR-counter (CI action / bot)** | Overkill. Guard в `merge-gate.sh` + `validate_adr_namespace.sh` — уже существующие точки принятия решения. |

## 4. Trade-offs

| Что получаем | Чем платим |
|---|---|
| Однозначные cross-ref между ADR навсегда | Pre-merge guard добавляет ~50 строк (Phase 2) + 3 регресс-теста |
| Различение доменов AF/RT устраняет ложные коллизии | Worker/человек обязан **классифицировать** свой ADR перед созданием (1 мысленная операция) |
| Процесс ADR масштабируется на параллельных воркеров | Worker обязан `git fetch origin develop` (медленнее на ~1-2 c) |
| Существующие 13 коллизий cleanup отделён от правила | 6 перенумерованных файлов требуют синхронного апдейта ссылок в `src/scripts/.github` |
| Ручной коммит в develop запрещён | Горячие правки — `hotfix/*` → PR |
| ADR — single-source-of-truth восстановлен | Шифу согласился с rule-of-future, историю не правим задним числом за исключением коллизий |

## 5. План внедрения

| # | Действие | Кто | Acceptance |
|---|---|---|---|
| 1 | Phase 1: ADR-AF-0030 смержен в develop (PR #1577) | architect | CI зелёный, base=develop |
| 2 | Phase 1: CONTRIBUTING.md дополнен § ADR-нумерация | architect | в этом же PR |
| 3 | Phase 1: Pre-merge guard в `agent-flow-merge-gate.sh` | devops | PR #2073 в develop, 10/10 тестов |
| 4 | Phase 2: AF-домен создан, 16 файлов переехали, 6 перенумерованы | devops | этот PR (issue #2076) |
| 5 | Phase 2: `validate_adr_namespace.sh` поддерживает AF-домен | devops | 13/13 тестов |
| 6 | Phase 2: Ссылки в `src/scripts/.github` обновлены по контексту | devops | `git grep` показывает только валидные ссылки |
| 7 | Phase 2: CONTRIBUTING.md дополнен § ADR-нумерация: два домена | devops | в этом же PR |
| 8 | Phase 2: ADR-AF-0030 → Accepted (этот PR) | (авто) | merge commit переключает статус |

## 6. Что НЕ делаем

- Не делаем авто-merge этого ADR — Шифу мержит сам (правило 2d).
- Не переименовываем **RT-файлы** за пределами списка §2.6 (0027-wake-gate, 0027-meta-quest остаются RT).
- Не вводим ADR-bot / external counter — overkill.
- Не наказываем существующих воркеров за коллизии — фикс идёт вперёд, не назад.

## 7. Ссылки

- `t_45db74ad` — Phase 1 ретро-карточка.
- `t_5055c13c` — Phase 2 ретро-карточка (этот PR).
- PR #1577, #1578, #1580, #1581, #1584 — источники коллизий (Phase 1).
- PR #2073 — Phase 1 guard в CI.
- issue #2076 — Phase 2 cleanup + AF/RT split.
- `docs/adr/AF-0018-agent-honesty-culture.md` — принцип «честный FAIL лучше красивого PASS».
- `docs/adr/AF-0026-recovery-card-contract.md` — worker отвечает за последствия.
- `CONTRIBUTING.md` §2d — запрет ручного merge; §2.5 расширяет до запрета ручного **коммита** в develop.
- `scripts/agent_flow/agent-flow-merge-gate.sh` — целевое место для guard (Phase 1).
- `scripts/agent_flow/validate_adr_namespace.sh` — pre-PR guard с поддержкой двух доменов (Phase 2).
- `scripts/agent_flow/tests/test_validate_adr_namespace.sh` — 13 регресс-тестов.