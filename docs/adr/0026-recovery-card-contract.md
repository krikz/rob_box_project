# ADR-0026: recovery-card contract — обязательство закрыть parent

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-08-23 |
| Автор | architect (Hermes Agent); ретро-карточка t_1dd950ff |
| Контекст | Карточки t_507b0474 / t_976a5c0c — процесс-bag: recovery-worker полностью отрабатывает ретро и завершается, **но** исходный parent остаётся в `blocked`. ADR-0024 уже формализовал workaround для archive (cross-task-archive-sweeper); этот ADR формализует **обязательство** самого recovery-worker'а |
| Затрагивает | `CONTRIBUTING.md` (новая секция "Recovery cards"), skill `sdlc-review` (acceptance criterion), процессная ответственность worker'а |
| Родители | ADR-0024 (cross-task-archive-sweeper — fallback для cron-sweep), ADR-0018 (честность) |
| Связанные | t_507b0474 (это), t_976a5c0c (root cause — recovery t_43d5e94e, t_cf3d17a0), t_8c8c7c69 (earlier spike-skill issue), t_1dd950ff (этот архитектурный fix) |

## 1. Контекст и бизнес-проблема

В agent-flow процесс recovery-card'ы создаются автоматически
(`agent-flow-merge-gate.sh`, ~50 мест вхождений строки "recovery card"),
когда исходная карточка stuck'нулась (crash-loop, blocked-on-outside,
deadlock, force-push race). Recovery-card наследует `parents=[<id>]` —
то есть dispatcher **знает**, какая карточка её породила.

Что произошло 22–23.08 (ретро t_507b0474, t_976a5c0c):

1. Карточки t_1075bd72 / t_9d229634 / t_8abada71 зависли в `blocked`.
2. Dispatcher создал 2 recovery-карточки: t_43d5e94e, t_cf3d17a0.
3. Recovery-карточки **полностью отработали** — закоммитили фиксы, открыли
   PR (#1548 MERGED, #1547 MERGEABLE), прошли e2e, оставили raw-evidence
   в комментариях.
4. **Но исходные parent'ы остались в `blocked`.** Recovery-worker не
   сделал ничего с ними — ни `kanban archive`, ни `kanban complete`,
   ни `kanban block` с новой формулировкой.
5. Process-bag: 2 stale blocked-карточки + 2 done recovery-карточки.
   Человек-оператор (товарищ Шифу) вынужден вручную делать
   `hermes kanban archive <parent>` через CLI.

Это нарушает контракт "recovery-card" — она обязана **разрешить parent'а**,
а не просто отработать свой собственный scope.

## 2. Root cause

Recovery-worker (worker-tools framework Hermes Agent) думает, что его
scope — только **его собственная карточка**. Это правда для **мутации**
(`kanban_complete`, `kanban_block`, `kanban_request_review` —
worker-scope-limit, "worker is scoped to task X; refusing to mutate Y").

Но recovery-card — это **особый случай**: её идентичность **определена**
parent'ом. Dispatcher буквально создаёт её с `parents=[<parent_id>]`,
то есть связь «child → parent» декларативна. Без перевода parent'а из
`blocked` recovery-card не завершает свой смысл — она просто разгружает
себя, оставляя parent как zombie.

## 3. Решение — контракт recovery-card

Recovery-worker **обязан** до завершения собственной карточки
перевести parent в одно из трёх терминальных/полутерминальных
состояний:

| Сценарий | Действие worker'а | Через что |
|---|---|---|
| Работа parent'а фактически завершена моим фиксом (recovery выполнил суть задачи) | `hermes kanban complete <parent>` с ссылкой на recovery-evidence | CLI |
| Parent obsolete — фикс не нужен / решено иначе | `hermes kanban archive <parent>` с обоснованием | CLI |
| Parent остаётся в `blocked` но причина сменилась (новая зависимость) | `hermes kanban block <parent> --reason "новый blocker X"` | CLI |

**Механизм — CLI, не worker-tool.** Причина:

- Worker-tools `kanban_complete` / `kanban_block` / `kanban_archive`
  (`kanban_archive` вообще не существует как Hermes tool — см. ADR-0024)
  ограничены scope'ом текущего worker'а.
- CLI-команды `hermes kanban {complete,block,archive}` — операторские,
  worker-lock не наследуется, scope-guard не применяется (raw-SQL bypass).
- ADR-0024 cross-task-archive-sweeper — fallback для cron-cleanup,
  **не замена** worker-обязательству.

## 4. Альтернативы, которые НЕ были выбраны

- **Пропатчить hermes-agent, добавить `kanban_complete --allow-cross-task`**.
  Архитектурный change, cross-profile запрет, нужен accept от товарища
  Шифу и отдельный ADR. **out-of-scope** для этой карточки —
  архитектурная реформа владения, не процесс-fix.
- **Оставить только sweeper (ADR-0024) как единственный механизм**.
  Уже работает, но перекладывает cleanup на cron. Worker при этом
  формально не несёт ответственности за parent'а — этот пробел ADR-0024
  признаёт, но не закрывает.
- **Worker-tool добавить без CLI bypass**. Невозможно без правки kernel.
- **Делать recovery-card без `parents=`**. Теряется декларативная связь,
  dispatcher не сможет гейтить завершение.

## 5. Изменения в этом ADR

1. **`CONTRIBUTING.md`** — новая секция `## Recovery cards` с явным
   текстом контракта (см. ниже).
2. **Skill `sdlc-review`** (`~/.hermes/profiles/devops/skills/devops/sdlc-review/SKILL.md`) —
   новый acceptance criterion: parent state MUST change до recovery →
   done; evidence обязан присутствовать в recovery-handoff.
3. **Сам процесс не меняется** — sweeper (ADR-0024) остаётся
   safety-net для случаев, когда worker этого не сделал.

## 6. Контрактный текст для CONTRIBUTING.md

```markdown
## Recovery cards

A recovery card (created by orchestrator or nadzor when an original card
gets stuck — e.g., crash-loop, blocked-on-outside, deadlock) inherits
resolution responsibility for the parent. A worker cannot consider a
recovery card done until the parent has moved out of `blocked`:
- `kanban archive <parent>` if the parent's findings are obsolete
- `kanban complete <parent>` if the work was finished by the recovery
- `kanban block <parent>` with concrete reason if the parent remains
  blocked on a new external dependency

Archive is allowed via `terminal: hermes kanban archive <id>` (CLI path
has no worker-scope guard, unlike the missing `kanban_archive` Hermes
tool). Workers cannot rely on `kanban_complete` for parent cards —
that is also worker-scoped.

sdlc-review (when reviewing a recovery card): verify the parent state
change appears in the recovery handoff. Missing evidence → request
changes. See `docs/adr/0026-recovery-card-contract.md` for the full
architectural decision.
```

## 7. Acceptance criterion для skill `sdlc-review`

```markdown
When this skill is used by a recovery-card worker:
- Parent card state must change (archive | complete | block-with-reason)
  BEFORE the recovery card transitions to `done`.
- Evidence of the parent-state change must appear in the recovery handoff
  (raw `hermes kanban list --id <parent>` или `kanban_show <parent>`
  output, ссылка на archival или `kanban_comment <parent>` с новым reason).
- If parent remains in `blocked` without any new reason → request changes.
```

## 8. Verification (как проверять)

- [ ] `CONTRIBUTING.md` содержит секцию `## Recovery cards` с полным
  текстом из §6 (grep: `^## Recovery cards`).
- [ ] `~/.hermes/profiles/devops/skills/devops/sdlc-review/SKILL.md`
  содержит acceptance criterion из §7 (grep: `recovery-card worker`).
- [ ] PR в `develop` открыт с этими изменениями, base = `develop`.
- [ ] CI зелёный (markdown lint, link-check если есть).
- [ ] После merge: следующая recovery-карточка обязана прислать
  evidence parent-state change в своём handoff.

## 9. Что НЕ делаем в этом ADR / следующие шаги

- ❌ Не правим hermes-agent sources (`kanban_complete --allow-cross-task`
  или `kanban_archive` tool) — out-of-scope, нужен отдельный
  архитектурный review.
- ❌ Не удаляем `cross-task-archive-sweeper.sh` (ADR-0024) — это
  safety-net для случаев, когда worker-обязательство не выполнено.
- 🔄 Если в kernel добавят `kanban_archive` tool — ADR-0026 §3 можно
  упростить, разрешив worker-tool archive вместо CLI. Зависит от
  hermes-agent maintainer'а.
