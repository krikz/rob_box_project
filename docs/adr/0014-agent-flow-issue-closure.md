# ADR-0014: Закрытие GitHub issue на пересечении `e2e-done` и merge в `develop`

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-08-11 |
| Автор | architect (Hermes Agent) |
| Контекст | Ретроспектива незакрытых `e2e-done` issues после merge |
| Затрагивает | `scripts/agent_flow/agent-flow-merge-gate.sh`, тесты agent-flow |

## 1. Контекст и бизнес-проблема

Agent-flow считает функциональную задачу завершённой только после двух независимых
событий:

1. e2e-прогон завершился успешно и issue получила label `e2e-done`;
2. связанный agent PR смержен в `develop`.

Фактическая автоматизация обновляет промежуточные состояния, но не выполняет
последний переход GitHub issue в `CLOSED`:

- `agent-flow-e2e-process.sh` на `success` ставит `e2e-done` и снимает
  `needs-e2e`;
- `agent-flow-merge-gate.sh` для PR в состоянии `MERGED` удаляет ветку,
  освобождает worktree, архивирует Kanban-карточку, пишет cleanup-комментарий и
  нормализует labels;
- команды закрытия issue нет ни в одном из этих переходов.

Это расходится с `docs/design/AGENT_FLOW_PROPOSAL.md`, где финальный cleanup после
ручного merge включает `issue close`. В результате завершённые issues остаются
OPEN, засоряют пользовательскую очередь и до появления triage-gate могли
повторно создавать Kanban-карточки. Triage-gate устраняет дубликаты, но не
исправляет ложное состояние issue.

Наблюдаемые примеры на 2026-08-11:

- issue #1082: `e2e-done`, OPEN; связанные PR #1084 и follow-up #1099 смержены
  в `develop`; PASS подтверждён e2e-докладом run 31337429107;
- issue #1104: `e2e-done`, OPEN; PR #1105 смержен в `develop`. Для #1104
  отдельный PASS-доклад в comments не найден: label был добавлен post-merge
  cleanup-логикой, которая сейчас ставит `e2e-done` без доказательства PASS.
  Поэтому #1104 — доказательство lifecycle-багa, но не безопасный кандидат для
  авто-закрытия до provenance-migration.

## 2. Инвариант завершения

```text
issue may close  <=>  issue has e2e-done produced by e2e PASS
                       AND selected agent PR is MERGED
                       AND PR base is develop
```

`e2e-done` — не просто строковый label, а свидетельство успешного e2e. Merge-gate
не имеет права сам создавать это свидетельство: post-merge cleanup только
сохраняет уже существующий `e2e-done` и снимает stale labels. До rollout нового
инварианта legacy labels без подтверждаемого PASS provenance не закрывают issue
автоматически. Оба terminal-факта проверяются по GitHub как по источнику истины.
Cleanup-комментарий,
статус Kanban-карточки и наличие/удаление ветки не являются достаточными
доказательствами завершения.

Для lint/refactor-пути с `no-e2e-required` нужен отдельный lifecycle-инвариант;
этот ADR его намеренно не меняет.

## 3. Рассмотренные варианты

| Вариант | Какую проблему решает | Самая простая реализация | Выгода | Цена / риск | Что будет, если не делать |
|---|---|---|---|---|---|
| 1. Закрывать в `e2e-process` при PASS, если PR уже MERGED | Закрывает редкий порядок `merge → e2e PASS` | После PASS запросить PR state и вызвать `gh issue close` | Решение рядом с e2e-verdict | Не покрывает основной порядок `e2e PASS → ручной merge`: в момент PASS PR ещё OPEN; появляется вторая обязанность у тяжёлого hourly-процесса | Большинство issues останутся OPEN |
| 2. Ручной чек-лист для Шифу | Не требует кода | Документировать `merge → close issue` | Нулевая техническая сложность | Повторяет уже доказанно ненадёжный ручной шаг; нет retry/idempotency; очередь снова дрейфует | Текущий баг сохраняется как операционная норма |
| 3. Post-merge transition в `merge-gate` | Закрывает основной порядок после ручного merge | В существующей ветке `MERGED`, после проверки `e2e-done`, закрыть issue | Использует cron каждые 5 минут как reconciliation loop; минимальный diff; естественный владелец merge-side перехода | Нужен явный guard от merge до PASS и отдельное покрытие обратного порядка событий | OPEN issues продолжают ложно выглядеть незавершёнными |

## 4. Решение

Выбран **вариант 3: post-merge close в `agent-flow-merge-gate.sh`**.

> **Порядок side effects:** сначала доказать `e2e-done` и успешно закрыть issue,
> затем удалять remote branch и архивировать карточку. Если close не удался,
> destructive cleanup откладывается до следующего reconciliation tick; это
> сохраняет mapping для retry и не оставляет завершённую работу без
> машиночитаемого close.

### Почему

- Бизнес-финиш наступает на втором из двух событий. В нормальном flow первым
  приходит e2e PASS, вторым — ручной merge; значит merge-gate наблюдает момент,
  когда конъюнкция впервые становится истинной.
- Merge-gate уже владеет post-merge cleanup и тикает каждые 5 минут. Ошибка
  GitHub API не требует нового retry-механизма: issue остаётся OPEN и следующий
  тик повторяет reconcile.
- Решение не переносит ответственность на человека и не добавляет новый сервис,
  очередь или state store.

### Минимальный алгоритм

В существующей ветке:

```text
if PR.state == MERGED and PR.base == develop:
    выполнить существующий cleanup

    перечитать актуальные labels issue
    if issue has e2e-done:
        закрыть issue с reason=completed
    else:
        оставить issue OPEN и залогировать "merged, awaiting e2e-done"
```

Требования к реализации:

1. Использовать `gh issue close <number> --reason completed` либо эквивалентный
   REST PATCH.
2. Удалить из post-merge cleanup команду, которая безусловно добавляет
   `e2e-done`. Merge-gate может снять `e2e:rejected`/`needs-e2e`, но PASS-label
   создаёт только e2e-process после успешного verdict.
3. Не использовать labels, прочитанные в начале тика, после side effects без
   повторной проверки: e2e-process может поставить `e2e-done` параллельно.
4. Ошибка закрытия не маскируется `|| true` без лога. Скрипт логирует warning и
   оставляет issue OPEN; следующий тик повторяет операцию. До успешного close
   нельзя удалять последний доступный способ сопоставить issue с PR: либо close
   выполняется до удаления remote branch, либо mapping сохраняется независимо
   от branch ref.
5. Комментарий об успешном cleanup публикуется только после успешного close и
   говорит правду: PR смержен, cleanup завершён, issue закрыта. Существующий
   dedup сохраняется.
6. Повторный тик безопасен: закрытые issues не входят в исходный запрос
   `--state open`; GitHub close сам по себе также идемпотентен.
7. Связанный PR выбирается существующей детерминированной логикой ветки/карточки,
   а не поиском номера issue в произвольном title.

## 5. Порядок событий и race conditions

### Основной путь: PASS → merge

1. e2e-process ставит `e2e-done`; issue остаётся OPEN для ручного review.
2. Пользователь мержит PR в `develop`.
3. Следующий merge-gate tick видит `MERGED + e2e-done`, сначала закрывает issue,
   затем завершает destructive cleanup ветки/worktree.

### Обратный путь: merge → PASS

Merge-gate видит MERGED, но без `e2e-done`, поэтому не закрывает issue. Когда
PASS придёт позже, e2e-process поставит label. Следующий merge-gate tick должен
снова сопоставить open issue с уже MERGED PR и закрыть его. Поэтому нельзя
добавлять постоянный marker «merged cleanup complete», который исключит issue
из последующих reconciliation ticks.

### Параллельный tick

Merge-gate повторно читает labels непосредственно перед close. Если label ещё не
виден, issue остаётся OPEN; следующий tick сойдётся к правильному состоянию.
Предпочтение отдаётся безопасной задержке до 5 минут, а не риску преждевременного
закрытия.

### Orphan: user-merge без e2e (Q22), ветка удалена

Ретро 13.08 t_423453b1 (#1160): пользователь смержил PR вручную (Q22) **без**
e2e-прогона, а ветка `z-{agent}/<id>-<slug>` удалена после merge (GitHub
"delete branch on merge" или ручная уборка). В этом случае:

- `gh pr list --head <branch>` всё ещё возвращает MERGED-запись PR (headRefName
  сохраняется в GitHub), поэтому merge-gate видит `MERGED`, но `e2e-done` не
  появится никогда: e2e-process физически не может собрать round (ветки нет,
  `git fetch origin/<branch>` пуст).
- Итог без фикса: issue вечно висит в ротации `needs-e2e` (шум каждый тик
  e2e-process + ложные conflict-карточки).

Решение (в merge-gate): при `MERGED + OPEN + needs-e2e без e2e-done` проверить
`git ls-remote` ветки:

- ветка **существует** → прежнее поведение: defer, ждём `e2e-done`
  (обратный путь merge → PASS, §5 выше);
- ветка **удалена** → e2e невозможен: снять `needs-e2e` (и `e2e:rejected`),
  прокомментировать юзеру «мерж без e2e (Q22): закрыть вручную или завести
  follow-up на e2e», issue **НЕ закрывать** (решение за юзером), `needs-review`
  **НЕ ставить** (PR уже нет), destructive cleanup **не запускать**.

e2e-process дополнительно скипает такие issue (тихо, без round), пока merge-gate
не снял метку.

## 6. Follow-up policy

Закрытая issue является неизменяемым свидетельством завершённого scope. Новый
follow-up PR после закрытия должен иметь новую issue и новый e2e lifecycle.
Автоматически переоткрывать закрытую issue или снимать с неё `e2e-done`
запрещено.

Существующая поддержка follow-up PR поверх **ещё открытой** `e2e-done` issue —
переходный механизм. После внедрения этого ADR он не должен использоваться как
нормальный способ расширять завершённый scope.

## 7. Acceptance criteria для devops-реализации

### Автоматические тесты

Минимум должны быть детерминированно проверены сценарии:

1. `MERGED + base=develop + e2e-done + OPEN` → ровно одна попытка close с
   `reason=completed`.
2. `MERGED + base=develop`, но без `e2e-done` → issue остаётся OPEN; close не
   вызывается.
3. `e2e-done`, но PR OPEN/CLOSED-unmerged или base не `develop` → close не
   вызывается.
4. Ошибка `gh issue close` → warning, tick не объявляет cleanup успешным,
   remote branch не удаляется до сохранения retry-capable mapping; повторный tick
   снова пытается закрыть.
5. Уже закрытая issue не создаёт побочных эффектов и не публикует дублирующий
   cleanup-комментарий.
6. Race `merge → label позже`: первый tick не закрывает, второй после появления
   `e2e-done` закрывает.
7. Merge-gate не создаёт `e2e-done`: MERGED без PASS provenance остаётся OPEN.
8. Existing regression: follow-up detection и triage `e2e-done` dedup остаются
   зелёными.
9. Orphan (ретро 13.08 t_423453b1): `MERGED + needs-e2e без e2e-done`, ветка
   удалена (`git ls-remote` пуст) → `needs-e2e` снят, комментарий юзеру (Q22)
   опубликован, close НЕ вызывается, `needs-review` НЕ ставится, destructive
   cleanup НЕ запускается. Если ветка существует — прежнее defer (criterion 2).

Тесты должны мокать `gh`/GitHub fixtures и не менять реальные issues.

### Rollout

1. Изменить SOT только в `scripts/agent_flow/`; runtime-копии вручную не править.
2. `bash -n` и `shellcheck` для затронутых shell-скриптов.
3. Смёржить PR в `develop`, затем применить `scripts/agent_flow/install.sh` по
   штатной процедуре раскладки.
4. Проверить на dry-run/fixture до включения side effect.
5. После rollout один раз reconcile legacy cases:
   - #1082 можно закрыть автоматически/миграцией: PASS provenance найден;
   - #1104 не закрывать только по legacy `e2e-done`: сначала подтвердить PASS
     provenance либо выполнить отдельный ручной review/close.
   Ручное закрытие остаётся аварийным fallback, если сопоставление старой ветки
   невозможно.

## 8. Последствия

### Положительные

- GitHub issue снова отражает фактическое завершение работы.
- Убирается повторяемый ручной шаг и причина повторного triage.
- Retry получается бесплатно через существующий cron reconciliation loop.
- Изменение локально: один владелец post-merge lifecycle, без новой
  инфраструктуры.

### Отрицательные / риски

- Закрытие может задержаться до следующего 5-минутного тика.
- Legacy issue с нестандартной branch mapping может потребовать одноразового
  ручного reconcile.
- Изменение делает `e2e-done` terminal для данного scope; follow-up теперь
  требует новой issue.

### Если не внедрять сейчас

PR #1119 предотвращает новые дубликаты Kanban-карточек, но завершённые GitHub
issues продолжат оставаться OPEN. Это скрывает реальный throughput, требует
ручной уборки и сохраняет расхождение между документированным и исполняемым
lifecycle.
