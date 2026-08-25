# ADR-0030: ADR-нумерация — single source of truth, pre-merge guard, ручной коммит запрещён

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge → Accepted) |
| Дата | 2026-08-25 |
| Автор | architect (Hermes Agent); ретро-карточка `t_45db74ad` |
| Контекст | Коллизия ADR-номеров в `origin/develop` (на 25.08 00:59 UTC): 5 файлов под 3 номерами — `0027×3`, `0028×2`. Никакой процесс нумерации ADR сегодня не enforced: ни merge-gate, ни CONTRIBUTING, ни ADR-0001. |
| Затрагивает | `CONTRIBUTING.md` (новая секция "ADR-нумерация"), `scripts/agent_flow/agent-flow-merge-gate.sh` (pre-merge guard, child-карточка devops), `docs/adr/0027-*` / `0028-*` (cleanup, child-карточка devops — требует решения Шифу по домен-неймспейсу) |
| Родители | ADR-0018 (честность), ADR-0026 (recovery contract — worker отвечает за следствие), `t_45db74ad` (эта ретро) |
| Связанные | PR #1577, #1578, #1580, #1581, #1584; ручной коммит `955cbf58` (Denis 24.08 23:40); `t_45db74ad` |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

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

### 1.2 Почему это блокер (а не косметика)

ADR — single-source-of-truth архитектурных решений проекта. Cross-reference в тексте — обычное дело: в коммите `955cbf58` (Avatar supervisor) стоит фраза

> Links to ADR-0027 (Quest, voice modes §3.4)

— но под номером `0027` теперь три файла. Любой такой cross-ref **теряет однозначность**, а значит:

- ADR-First поиск перестаёт работать (какой из трёх 0027?).
- Комментарии вроде «см. ADR-0027 §X» становятся неоднозначными → ревью ADR'ов замедляется.
- Новые worker'ы, открывающие PR с ADR, рискуют попасть в чужой занятый номер — **процесс ADR-документации не масштабируется**.

### 1.3 Гипотеза (root cause)

Три независимые дыры сложились в один эффект:

1. **Нет pre-merge guard на уникальность `NNNN-*.md` в `docs/adr/`.**
   `scripts/agent_flow/agent-flow-merge-gate.sh` проверяет размер PR (ADR-0013), honesty-claim (ADR-0018), e2e gates (ADR-0022), lint-ветки (PR kind detection) — но **не** проверяет, что новый ADR-файл с номером `NNNN` ещё не существует в `develop`.

2. **Ручной коммит Шифу в обход PR (24.08 23:40, `955cbf58`).**
   Avatar supervisor ADR был положен напрямую в `develop`, **без PR**, **без проверки номера**. Это единичный случай, но он ровно того же сорта: никакого gate'а между человеком и `develop` нет, когда commit идёт мимо PR.

3. **Параллельные worker'ы (#1577 / #1578 / #1580 / #1581) выбрали `0027` / `0028`, не сверившись с `origin/develop`.**
   Worker-протокол требует `git fetch origin develop` и проверки существующих файлов перед созданием нового — но эта проверка сейчас **неформализованная** (нет правила в `CONTRIBUTING.md`, нет gate'а, который отказывал бы).

### 1.4 Бизнес-последствие

Без фикса:

- Каждый новый ADR **несёт риск** попасть в занятый номер → процесс документации не масштабируется, ретро-карточки типа этой будут повторяться.
- Cross-reference между ADR разрушается → снижается ценность существующих 28 ADR'ов (по факту 33 файла, из них 5 — коллизии).
- ADR-0029 (wake-gate cold-start known state) уже стоит в `develop` под уникальным номером, но **никакой гарантии**, что следующий worker не выберет `0029` повторно.

## 2. Принятое решение

### 2.1 ADR-номер = глобальный монотонный счётчик

**Правило.** Номер ADR — это 4-значный zero-padded монотонный счётчик, привязанный к файлам в `docs/adr/NNNN-*.md` на `origin/develop`. Имя файла:

```
docs/adr/NNNN-<kebab-case-slug>.md
```

Где:

- `NNNN` ∈ `[0001, 9999]`, **уникален** в пределах `origin/develop` на момент merge.
- `<slug>` — kebab-case, описывает решение (не задачу).
- Полное имя файла совпадает с заголовком первого H1 внутри: `# ADR-NNNN: <slug>`.

### 2.2 Как выбирать NNNN перед созданием

Перед созданием нового ADR любой worker / человек обязан **прогнать**:

```bash
# Свежий снимок origin/develop
git fetch origin develop

# Какие номера заняты
git ls-tree -r origin/develop --name-only \
  | grep -oE 'docs/adr/[0-9]{4}' \
  | sort -u

# Следующий свободный
NEXT=$(( $(git ls-tree -r origin/develop --name-only \
            | grep -oE 'docs/adr/[0-9]{4}' \
            | sort -u | tail -1 | grep -oE '[0-9]{4}') + 1 ))
printf '%04d\n' "$NEXT"
```

Число `NNNN` в имени файла = число `NNNN` в первом H1 = число `NNNN` в frontmatter таблице.

**Запрещено:**

- Использовать номер, не проверив `origin/develop`.
- Использовать номер, который уже занят (даже если кажется «логичным» — это создаёт коллизию).
- Сокращать (`27` вместо `0027`) — ломает grep-инвариант.

### 2.3 Cross-reference внутри ADR

При ссылке на другой ADR в тексте:

```markdown
См. [ADR-0022 §4.6](../0022-process-e2e-done-gates.md) (process e2e done gates).
```

Требования:

- `ADR-NNNN` + опционально `§X.Y` + опционально `(slug-anchor)`.
- Cross-ref **должен** разрешаться в существующий файл на момент merge.
- Если cross-ref указывает на «будущий» ADR — это **TODO**, не cross-ref; пишите явно `TODO(ADR-NNNN)` (см. ADR-0022 §7).

### 2.4 Pre-merge guard (child-задача devops)

В `scripts/agent_flow/agent-flow-merge-gate.sh` добавляется новая проверка:

- Для PR, которые создают **новый** файл `docs/adr/NNNN-*.md`:
  - Получить NNNN из имени файла.
  - Сверить с `git ls-tree -r origin/develop --name-only | grep "^docs/adr/NNNN-"`.
  - Если такой префикс уже есть в `develop` → **reject** с сообщением:

    ```
    ADR-number collision: docs/adr/NNNN-<slug>.md conflicts with existing
    docs/adr/NNNN-<existing-slug>.md in origin/develop.
    Выберите следующий свободный номер через:
      git ls-tree -r origin/develop --name-only | grep -oE 'docs/adr/[0-9]{4}' | sort -u | tail -1
    и переименуйте файл + обновите H1 / frontmatter.
    ```

- Расположение в скрипте: рядом с `detect_pr_kind` (lint-ветка для ADR) или новый блок GATE в `process_pr` — конкретное место решает devops-воркер в рамках child-карточки.
- **Тест:** `scripts/agent_flow/tests/test_merge_gate_adr_uniqueness.sh` — кейсы (a) новый номер → ок, (b) коллизия → reject, (c) изменение существующего ADR-файла (без нового NNNN) → не трогаем.

### 2.5 Ручной коммит в develop — запрещён

Любой коммит в `develop` (включая ручной от Шифу) **должен** идти через feature-branch + PR, **даже если коммит единственный**. Исключений нет.

Это:

- Даёт pre-merge guard шанс сработать.
- Оставляет audit-trail (PR → ревью → CI).
- Сохраняет правило ADR-0026 (recovery contract): если что-то ломается — есть ветка, откат — один клик.

Если нужно срочно (горячая правка) — `hotfix/<name>` → PR в `develop`. Отдельной ветки «ручной коммит Шифу» не существует. Это правило включается в `CONTRIBUTING.md` как §2d-bis.

### 2.6 Cleanup существующих коллизий

Cleanup **не входит в этот PR** — он требует решения Шифу по домен-неймспейсу (см. §4). См. child-карточку `t_45db74ad-c` (devops).

Кратко (для контекста devops-воркеру):

| Текущий файл                                              | Вариант A (выделить primary)     | Вариант B (разделить на новые номера)        |
|-----------------------------------------------------------|----------------------------------|----------------------------------------------|
| `0027-harness-improvements-from-external-talk.md`         | оставить как primary             | → `0031-harness-improvements-from-external-talk.md` |
| `0027-meta-quest-ar-control.md`                           | rename → `0031-meta-quest-ar-control.md` | → `0032-meta-quest-ar-control.md`           |
| `0027-systemic-wake-gate-no-wake-word-blocker.md`         | rename → `0032-systemic-wake-gate-no-wake-word-blocker.md` | → `0033-systemic-wake-gate-no-wake-word-blocker.md` |
| `0028-avatar-supervisor.md`                               | **primary** (первый по времени, ручной коммит) | → `0034-avatar-supervisor.md`               |
| `0028-harness-5-improvements.md`                          | rename → `0031-harness-5-improvements.md` (если primary = harness-improvements, тогда clash) / либо `0033-harness-5-improvements.md` | → `0035-harness-5-improvements.md`           |

Решение — за Шифу. **До решения** этот ADR остаётся Proposed, и guard в §2.4 не активен для существующих коллизий (он проверяет только новые файлы).

## 3. Альтернативы, которые мы отвергли

| Альтернатива                                      | Почему отвергли |
|---------------------------------------------------|-----------------|
| **Домен-неймспейс в номере** (`0027-proc-...`, `0027-quest-...`) | Двух-уровневая нумерация хрупка (что считать «доменом»?), ломает существующие 30 ADR-номеров, ADR-0001 принят без такого префикса. |
| **Использовать существующие 0027/0028 как есть, просто пометить «DEPRECATED»** | Не решает cross-ref: ADR-0027 по-прежнему неоднозначен. Deprecation — это лечение симптома, не причины. |
| **Удалить дубликаты автоматически (most recent wins)** | Уничтожает решение. ADR — это задокументированное решение, а не временный файл. Авто-удаление без ревью — нарушение ADR-0018. |
| **Не вводить guard — ограничиться документированием правила** | Не работает: 6 попыток ретро этой карточки (`t_45db74ad`, runs 2397–2472) показали, что без enforced-инварианта процесс не держится. Документированные правила без gate'а = пожелания. |
| **External ADR-counter (CI action / bot)** | Overkill для проекта. Guard в `merge-gate.sh` — уже существующая точка принятия решения, добавить туда блок дешевле, чем отдельный бот. |

## 4. Trade-offs

| Что получаем                                              | Чем платим                                                |
|-----------------------------------------------------------|-----------------------------------------------------------|
| Однозначные cross-ref между ADR'ами навсегда              | Pre-merge guard добавляет ~20 строк + 1 shell-test       |
| Процесс ADR-документации масштабируется на параллельных worker'ов | Worker обязан **всегда** `git fetch origin develop` перед созданием ADR (медленнее на ~1–2 c) |
| Cleanup существующих 5 коллизий отделён от правила (можно делать потом) | До cleanup'а cross-ref на «0027» / «0028» всё ещё неоднозначен — но это **известное** состояние, документировано в §2.6 |
| Ручной коммит в develop запрещён                          | Если Шифу нужно «прямо сейчас» — путь через PR + CI. Для горячих правок уже есть `hotfix/*`. |
| ADR — single-source-of-truth восстановлен                 | Решение Шифу по домен-неймспейсу (§2.6) требует его времени — это блокирует полный cleanup, но не блокирует rule-of-future |

## 5. План внедрения

| # | Действие                                                                                                | Кто         | Acceptance |
|---|----------------------------------------------------------------------------------------------------------|-------------|------------|
| 1 | Этот ADR смержен в `develop`                                                                            | architect   | PR открыт, base=develop, CI зелёный |
| 2 | `CONTRIBUTING.md` дополнен § ADR-нумерация (ссылка на ADR-0030 + §2.4 + §2.5)                            | architect   | в этом же PR |
| 3 | Pre-merge guard реализован в `agent-flow-merge-gate.sh` + unit-тесты                                      | devops      | см. child `t_45db74ad-d` |
| 4 | Cleanup 5 существующих коллизий по решению Шифу                                                          | devops (после Шифу) | см. child `t_45db74ad-c` |
| 5 | `docs/adr/0030` → Accepted после merge                                                                   | (авто)      | merge commit переключает статус |

## 6. Что **не** делаем

- Не делаем авто-merge этого ADR — Шифу мержит сам (правило 2d).
- Не правим существующие ADR-файлы (0001–0029) в этом PR — только §2.1–2.5 правила.
- Не вводим ADR-bot / external counter — overkill (см. §3).
- Не наказываем worker'ов #1577/#1578/#1580/#1581 за уже-сделанный выбор — фикс идёт вперёд, не назад.

## 7. Ссылки

- `t_45db74ad` — эта ретро-карточка.
- PR #1577, #1578, #1580, #1581, #1584 — источники коллизий.
- `docs/adr/0018-agent-honesty-culture.md` — принцип «честный FAIL лучше красивого PASS» — применим и сюда: лучше reject PR с коллизией, чем вмёржить её и починить потом.
- `docs/adr/0026-recovery-card-contract.md` — worker отвечает за последствия (включая документирование).
- `CONTRIBUTING.md` §2d — запрет ручного merge; §2.5 расширяет это до запрета ручного **коммита** в develop.
- `scripts/agent_flow/agent-flow-merge-gate.sh` — целевое место для guard (child devops).
