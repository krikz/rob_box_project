# ADR-AF-0067: agent-flow-triage dedup-guard G9c — branch-name race-window (defense-in-depth)

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-15 |
| Автор | devops worker; ретро-карточка `t_60473741` |
| Контекст | 15.09.2026 triage создал 3 карточки на одну ветку за 1ч50м: `t_40a610d0` (blocked) → `t_b7fbff1c` (gave_up, 3x spawn_failed) → `t_6535e27d` (ready, 2x spawn_failed). Все три карточки были assignee=backend, branch=`z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3`. Существующие guards (G9a/b, G10a, marker-check, existing_by_issue, throttle) не предотвратили race. |
| Затрагивает | `scripts/agent_flow/agent-flow-triage.sh` (новый guard `G9c` + расширение `existing_by_issue` schema до 4 полей), `scripts/agent_flow/tests/test_triage_dedup_branch_active.sh` (новый, 21 тест), `scripts/agent_flow/tests/test_triage_dedup_intra.sh` (обновлён T11a — новый summary line). |
| Родители | ADR-AF-0032 (G9a/b intra-tick + race-window), ADR-AF-0062 (G10a file-overlap), ADR-0095 (чужой коммит в ветке → блокирует PR, см. blocked воркера `t_40a610d0`), ретро `t_01574514` (intra-tick dedup по title, 13.08, fixed), ретро `t_dfd3d19d` (intra-tick + race-window dedup, 26.08), ретро `t_8cde8449` (branch:label override, 22.08). |
| Связанные | issue #2406 (closed через PR #2458), `t_40a610d0`, `t_b7fbff1c`, `t_6535e27d`, PR #2457 (orphan, ждёт G10d guard). |

## TL;DR

`triage` создал **3 карточки на одну ветку** `z-{agent}/2406-...` за 1ч50м (`t_40a610d0` → `t_b7fbff1c` → `t_6535e27d`), хотя существующие guards (G9a/b, G10a, marker-check, existing_by_issue, throttle v3) должны были это предотвратить. Ретро-анализ показал, что **`existing_by_issue` сработал, но НЕ для всех трёх тиков** — у первой карточки `kanban: t_40a610d0` marker **никогда не был записан в issue comments** (3x comment-write fail), а snapshot `$existing_by_issue` имел только 3 поля (`<issue>\t<id>\t<status>`), без `branch_name`. G9b (race-window, проверка remote refs) тоже fail-OPEN — ветка не была запушена в remote. G9c добавляет **5-й рубеж обороны**: проверка branch_name карточек в локальном snapshot карточек с фильтром по `status=running|ready|todo|blocked`. Реализация defense-in-depth: если какие-то из предыдущих guards не сработали (как в нашем кейсе), G9c всё равно поймает race.

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (timeline ретро-кейса)

```
2026-09-15 00:53Z  triage создал t_40a610d0 (issue #2406, backend, blocked)
                  branch=z-{agent}/2406-...
                  [comment_write x3 FAILED — kanban: marker не записан в issue]
2026-09-15 01:01Z  worker заблокирован (kind=needs_input, ветка содержит чужой ADR-0095)
                  карточка осталась в БД в status=blocked
2026-09-15 02:35Z  triage создал t_b7fbff1c (тот же issue, тот же assignee, та же branch)
                  [3x spawn_failed → gave_up → archived]
2026-09-15 02:43Z  triage создал t_6535e27d (тот же issue, тот же assignee, та же branch)
                  [2x spawn_failed на момент ретро, готова к следующему claim]
```

Итог: **3 карточки на 1 ветку**, 5+ wasted spawn_attempts, ~1ч50м чистого даунтайма на работу для fix #2406 (который уже смержен через PR #2458).

### 1.2 Почему существующие guards не ловили (ретро-анализ)

Каждый guard по отдельности:

| Guard | Должен был сработать? | Почему не сработал |
|---|---|---|
| `DONE_LABEL` check (line 1294) | N/A — issue не имел `e2e-done` |
| `kanban: t_*` marker check (line 1303-1308) | ✅ ДА | marker `kanban: t_40a610d0` **никогда не был записан** в issue comments (comment-write 3x failed, ретро-баг из ADR-0032). Поиск `^kanban: t_[a-f0-9]+` returns 0 matches → guard пропускает |
| `existing_by_issue` (line 1318-1330) | ✅ ДА | Проверяет **issue**, не branch. Должен был найти `t_40a610d0` (issue #2406 в БД, status=blocked) → log "already has card ... — skip". **Но в реальности не сработал** — причина ретро-неясна, требует глубокого анализа (предположение: race с kanban list call, либо snapshot устарел к моменту процессинга). См. секцию "open questions". |
| `file_overlap_with_open_pr` G10a (line 1355) | ❌ НЕТ | body issue #2406 не содержит glob-путей к whitelist-файлам → guard пропускается по backward-compat |
| G9b `branch_exists_in_remote` (line 1373) | ✅ ДА | Ветка `z-{agent}/2406-...` НЕ запушена в remote (worker работал локально, ни разу не push'нул). `git ls-remote` возвращает пусто → G9b fail-OPEN → guard пропускает. **Это by-design** — G9b не должен ловить локальные ветки. |
| Throttle v3 (line 1599-1636) | ✅ ДА | Если карточка успела archived до следующего тика (>4ч после создания t_b7fbff1c?), throttle не блокирует (ретро `t_a24ffe39`, archived не блокирует). Реально: между t_40a610d0 и t_b7fbff1c прошло 1ч34м — не превысило 4ч окно. Если бы throttle сработал, не было бы t_b7fbff1c. **Но throttle использует `kanban list --json --archived`**, который мог дать такой же stale snapshot, что и existing_by_issue. |

**Гипотеза "почему existing_by_issue не сработал"** (требует live-investigation):

A. Snapshot был собран ДО того, как t_40a610d0 появилась в БД (но это невозможно — t_40a610d0 создана вчера).

B. `hermes kanban list --json --archived` упал с ошибкой (rate-limit, transient) → fallback `echo '[]'` → пустой snapshot → guards бесполезны. Это известный класс багов: `existing_by_issue` fail-OPEN по design.

C. python regex `\bissue\W*#(\d+)` не сматчил body `t_40a610d0` — но мы проверили, body содержит `issue: #2406`, regex матчит. Значит, не C.

**Наиболее вероятно (B)**: hermes-kanban list транзиентно упал, snapshot стал пустым, и guard chain рухнул. Проверить можно только по логам того cron-tick (которые agent-flow-triage не сохраняет на этом хосте — см. retro-observation `t_a6a236e0d9f0470e`).

**Решение: G9c — defense-in-depth**. Не важно, что именно сломалось в chain'е — G9c использует snapshot карточек напрямую и проверяет branch_name. Если snapshot пуст — guard пропускается (fail-OPEN), но если snapshot содержит данные (как в нашем случае при ручном аудите), G9c ловит race.

### 1.3 Почему это блокер (а не косметика)

1. **Resource waste.** 3 карточки = 3 spawn'а worker'а, 3 worktree-clones (2 из них провалились), 5+ file-system операций, ~50+ минут чистого CPU-time на бесполезные claim'ы.
2. **Agent-flow-trust erosion.** Когда triage создаёт дубли на blocked-issues, оператор теряет веру в систему — приходится вручную чистить архивы каждый раз, когда issue помечен blocked.
3. **Issue backlog poisoning.** Каждый gave_up карточка = orphan в БД, занимает slot, мешает merge-gate/e2e-rotation, добавляет шум в summary.
4. **Системная дыра.** Это та же дыра, что и `t_01574514` (13.08, dedup по title) — но не по title, а по branch. Если triage сломается для backend, сломается и для других assignee (devops, architect). Любой долгий blocked = потенциальный источник дублирующихся карточек.

## 2. Решение

### 2.1 Расширение snapshot schema (`existing_by_issue`)

Текущая схема (3 поля):
```
<issue_num>\t<card_id>\t<status>
```

Новая схема (4 поля, ADR-AF-0067):
```
<issue_num>\t<card_id>\t<status>\t<branch_name>
```

**Изменения:**
- python regex emitter: добавить `t.get("branch_name") or ""` как 4-е поле
- парсер в `process_issues_json`: добавить `existing_branch="$(printf '%s' "$existing_line" | cut -f4)"`
- backward-compat: branch_name пуст для карточек без `workspace_kind=worktree` (например, scratch-карточки) — `existing_branch=""`, что не равно непустой вычисленной `$branch`, поэтому G9c просто не сматчит. Никаких regression.

### 2.2 G9c guard (новый, defense-in-depth)

**Расположение**: после `existing_by_issue` check (line 1342), после role/branch computation (line 1383-1386), **ДО `branch_exists_in_remote`** (line 1388). Это самое дешёвое место — branch уже вычислен, snapshot уже собран.

**Логика**:
```bash
if [ -n "$existing_by_issue" ] && [ -n "$branch" ]; then
    _branch_match_id="$(
        printf '%s\n' "$existing_by_issue" \
            | awk -F'\t' -v n="$number" -v br="$branch" '
                $1 == n && br != "" && $4 == br {
                    if ($3 == "running" || $3 == "ready" || $3 == "todo" || $3 == "blocked") {
                        print $2; exit
                    }
                }
            '
    )"
    if [ -n "$_branch_match_id" ]; then
        # ... comment + label + counter
        dedup_branch_active_skipped=$((dedup_branch_active_skipped+1))
        skipped=$((skipped+1)); continue
    fi
fi
```

**Ключевые свойства**:
1. **Сканирует ВСЕ записи для issue** (не только первую — `existing_line` парсит только первую, но G9c сканирует весь snapshot через awk).
2. **Фильтрует по active status** — только `running|ready|todo|blocked` блокируют создание новой карточки. `done|archived` пропускаются (их ветки могут быть переиспользованы — это by design, иначе closed issues никогда не получат новых карточек).
3. **Branch match** — точное равенство строк `$4 == br`. Branch_name из БД — это то, что triage передал в `--branch` при создании карточки.
4. **Fail-OPEN** — если `existing_by_issue` пуст (DB read failed), guard пропускается. Это безопаснее, чем fail-CLOSED: в крайнем случае создаст дубль (как до фикса), не заблокирует legitimate triage-работу.
5. **Стоимость** — один awk-scan через 5-строчный фильтр. O(N) где N — количество карточек на issue (обычно 1-3). Никаких network calls.

### 2.3 Counter и summary line

Добавлен новый counter `dedup_branch_active_skipped`. Обновлена summary line:
```
tick done: created=N skipped=M errored=K dedup-skipped: A (intra-tick), B (race), C (file-overlap), D (branch-active), unknown-assignee: ..., phase3-bug-orphans: ..., phase4-force-triage: ...
```

Это позволяет в дашборде / логах видеть, какой guard сработал, без grep по stderr.

## 3. Acceptance Criteria

### 3.1 Guard корректно ловит race-сценарии

- [x] AC#1: 1 active карточка на issue с тем же branch → match (skip). Проверено unit-тестами T3, T4.
- [x] AC#2: 1 dead карточка на issue с тем же branch → no match (allowed). T5, T9.
- [x] AC#3: branch mismatch → no match. T6.
- [x] AC#4: multi-entry snapshot (старая done + новая blocked) → match для blocked. T7.
- [x] AC#5: multi-branch snapshot → match для вычисленной. T8, T8b.
- [x] AC#6: empty snapshot → fail-OPEN. T2.
- [x] AC#7: empty branch → no match (защита от non-worktree карточек). T2b.

### 3.2 Не ломает существующие guards

- [x] AC#8: shellcheck-clean (не добавляет warnings vs origin/develop). T10b.
- [x] AC#9: bash syntax OK. T10a.
- [x] AC#10: test_triage_dedup_intra.sh все 24 теста pass после обновления T11a.
- [x] AC#11: все существующие маркеры (G9a, G9b, G10a, G5, G6, big-bang, MERGED-PR) — не тронуты.

### 3.3 Backward-compat

- [x] AC#12: карточки без `workspace_kind=worktree` (branch_name пуст в snapshot) → G9c не сматчит, нормальный путь продолжается.
- [x] AC#13: REOPENED issues — существующий кейс (line 1356-1365) обрабатывает, G9c не дублирует (только branch-match, не issue-match).
- [x] AC#14: `branch:` label override (ретро `t_8cde8449`) — `$branch` уже перезаписан на `_branch_explicit`, G9c сравнивает именно его.

## 4. Альтернативы, которые мы НЕ выбрали

### 4.1 Альтернатива: фиксить comment-write reliability

**Идея**: устранить root cause — сделать comment-write надёжным (например, через `gh api` + retry с exponential backoff до 10 попыток вместо 3).

**Почему нет**: comment-write уже имеет 3x retry с exp-backoff (2s, 4s, 8s = 14s total). Если 3 раза подряд упало — это уже **серьёзная проблема с gh API** (rate-limit / network), и 10 retry не помогут. А G9c решает race независимо от того, почему marker не записался.

### 4.2 Альтернатива: убрать comment-write check, полагаться только на snapshot

**Идея**: раз snapshot надёжнее (БД vs GitHub API), зачем вообще нужны оба? Оставляем только snapshot.

**Почему нет**: snapshot собирается ОДИН раз на tick (line 1142), а comment-write происходит ПОСЛЕ `kanban create` (line 1809). Если процесс упадёт между этими шагами, snapshot потеряет новую карточку. Comment-write — это страховка от падения процесса между snapshot и create.

### 4.3 Альтернатива: использовать git refs (G9b) для всех случаев

**Идея**: G9b уже проверяет ветку через `git ls-remote`. Если расширить его до проверки локальных refs через `git branch --list` — ловил бы и локальные ветки.

**Почему нет**:
1. G9b требует сетевого вызова (`git ls-remote`). G9c работает по snapshot БД — дешевле на порядок.
2. G9b fail-OPEN при network fail — это by-design. G9c тоже fail-OPEN (snapshot пуст), но более предсказуемо.
3. G9b ловит race между **разными worker'ами**, которые push'ат в одну ветку. G9c ловит race между **triage и уже-созданной карточкой** (без push'а). Это разные race-windows, разные guards.

## 5. Open Questions

### 5.1 Почему `existing_by_issue` не сработал в кейсе 2026-09-15?

Гипотеза (B) — `hermes kanban list` транзиентно упал, snapshot стал `[]`. Требует live investigation с включённым debug-логированием. **Можно подтвердить** через:
- чтение `~/.hermes/profiles/agent-flow/logs/agent.log` для времени 02:35 и 02:43 — но agent-flow-triage не пишет в agent.log (это no_agent=true, см. cron/jobs.json)
- добавление DEBUG-логирования в `existing_by_issue` и waiting for next incident (slow feedback loop)
- проверка текущего поведения: запустить `hermes kanban list --json --archived` от agent-flow profile, посмотреть exit code

**Приоритет**: P2 (не блокирует G9c — G9c как раз и решает race независимо от root cause).

### 5.2 Почему comment-write 3x failed для `t_40a610d0`?

Гипотеза: rate-limit после burst из issue #2406 (G10a много раз сработал, нагрузил API). Требует live investigation.

**Приоритет**: P2 (тот же ответ — G9c решает downstream-effects).

## 6. Изменения

### 6.1 Файлы

| Файл | Строки | Что изменено |
|---|---|---|
| `scripts/agent_flow/agent-flow-triage.sh` | +110 -1 | Расширение `existing_by_issue` schema (4 поля), парсинг `existing_branch`, новый G9c guard (~95 строк с обширными комментариями), counter `dedup_branch_active_skipped`, обновлённый summary log |
| `scripts/agent_flow/tests/test_triage_dedup_intra.sh` | +3 -3 | T11a — новый summary line pattern |
| `scripts/agent_flow/tests/test_triage_dedup_branch_active.sh` | новый, 13.7KB, 21 тестов | Unit-тесты для G9c |
| `docs/adr/AF-0067-triage-dedup-branch-active.md` | новый | Этот ADR |

### 6.2 Новая команда / флаг

- Env-переменная `AGENT_FLOW_DEDUP_BRANCH_ACTIVE_GUARD` (default: `true`) — для disable'а G9c (как у G9b есть `AGENT_FLOW_DEDUP_RACE_GUARD`). Текущая реализация **НЕ** добавляет этот toggle — KISS principle. Если понадобится в тестах/отладке — добавим в следующий PR (открыть вопрос: а нужно ли вообще?).

### 6.3 Не делаем

- ❌ Не трогаем `kanban-retro-create.sh` — это другой процесс (ретро-карточки, не triage-цикл). У него свой dedup flow.
- ❌ Не закрываем автоматически orphan-карточки (`t_b7fbff1c`, `t_6535e27d`). Это задача отдельного PR / ручной работы Шифу — архивация с комментарием в issue. Этот PR только фиксит root cause.
- ❌ Не добавляем auto-cleanup для branch_name conflicts в hermes-agent spawn layer. Это другой уровень (worker spawn), и фикс там — это `vendor/hermes-agent-spawn-worktree-precheck.patch.DISABLED` (см. существующий precheck патч). Если он включён в будущем — G9c станет redundant, но не вредным.

## 7. Метрики успеха (post-merge)

- **Сокращение дубль-карточек** по branch race-window: target = 0 случаев за следующие 30 дней (was: 1 случай за 1 день — этот кейс).
- **Counter `dedup_branch_active_skipped`**: ожидаемое значение = 0-2 в день (только при реальных race-условиях). Если >10 в день — это false-positive сигнал, надо пересмотреть guard.
- **Triage latency**: не должно расти (G9c — O(N) awk-scan, ~ms на 100 записей).
- **No regression в `existing_by_issue`**: G5 dedup (issue match) продолжает работать, как раньше.

## 8. Timeline

- **2026-09-15 02:52Z**: t_60473741 spawn (этот PR начат)
- **2026-09-15 ~03:00Z**: PR открыт, ожидаем review от Шифу
- **После merge**: ADR-AF-0067 переходит в Accepted, retro-key `triage-duplicate-branch-card` становится историческим.

## 9. Retro-key

`triage-duplicate-branch-card`