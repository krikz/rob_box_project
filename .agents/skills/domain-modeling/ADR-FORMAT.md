# ADR Format (Rob Box)

> ⚠️ Rob Box использует СВОЙ формат ADR, отличный от generic-формата этого
> скилла. Канонические правила — `CONTRIBUTING.md` §«ADR-процесс» и
> `docs/adr/0030-adr-numbering-sot.md`. Ниже — выжимка; где расходятся —
> побеждает CONTRIBUTING.md.

ADR живут в `docs/adr/NNNN-<kebab-case-slug>.md` и нумеруются **глобальным
монотонным счётчиком** (4 цифры). Номер — **следующий свободный после
максимума в `origin/develop`**, а не «максимум в локальном `docs/adr/`».

## Template

```md
# ADR-NNNN: <short title>

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | YYYY-MM-DD |
| Автор | … |
| Контекст | 1-2 предложения: что за проблема и что решили |
| Затрагивает | файлы/компоненты, которых касается решение |
| Родители | ADR-00xx, … |
| Связанные | скрипты, ADR, issues |

## 1. Контекст и бизнес-проблема

## 2. Решение

## 3. Инварианты / Проверка

## 4. Trade-offs / Открытые вопросы
```

Таблица-шапка с полями выше — обязательный контракт (её читают
`validate_adr_namespace.sh` и merge-gate `check_adr_number_collision()`).
Секции после таблицы — свободные, по сути решения.

## Numbering (КРИТИЧНО)

```bash
used="$(git ls-tree origin/develop --name-only | grep -E '^docs/adr/0[0-9]{3}-.*\.md$' | sed 's@.*/@@' | sed 's@-.*@@' | sort -u)"
next_free="$(printf '%s\n0000\n' "$used" | sort -u | awk 'BEGIN{n=0} {if ($0+0 == n+1) n=$0+0} END{printf "%04d\n", n+1}')"
echo "$next_free"
```

Перед `gh pr create` прогони `bash scripts/agent_flow/validate_adr_namespace.sh`
(exit 0 = clean, exit 1 = collision + подсказка с next-free). Переименование
старого номера — только с согласия товарища Шифу (label `adr-collision-override`
на issue).

## When to offer an ADR

All three of these must be true:

1. **Hard to reverse**: the cost of changing your mind later is meaningful
2. **Surprising without context**: a future reader will look at the code and wonder "why on earth did they do it this way?"
3. **The result of a real trade-off**: there were genuine alternatives and you picked one for specific reasons

If a decision is easy to reverse, skip it: you'll just reverse it. If it's not surprising, nobody will wonder why. If there was no real alternative, there's nothing to record beyond "we did the obvious thing."

### What qualifies

- **Architectural shape.** "We're using a monorepo." "The write model is event-sourced, the read model is projected into Postgres."
- **Integration patterns between contexts.** "Ordering and Billing communicate via domain events, not synchronous HTTP."
- **Technology choices that carry lock-in.** Database, message bus, auth provider, deployment target. Not every library: just the ones that would take a quarter to swap out.
- **Boundary and scope decisions.** "Customer data is owned by the Customer context; other contexts reference it by ID only." The explicit no-s are as valuable as the yes-s.
- **Deliberate deviations from the obvious path.** "We're using manual SQL instead of an ORM because X." Anything where a reasonable reader would assume the opposite. These stop the next engineer from "fixing" something that was deliberate.
- **Constraints not visible in the code.** "We can't use AWS because of compliance requirements." "Response times must be under 200ms because of the partner API contract."
- **Rejected alternatives when the rejection is non-obvious.** If you considered GraphQL and picked REST for subtle reasons, record it; otherwise someone will suggest GraphQL again in six months.
