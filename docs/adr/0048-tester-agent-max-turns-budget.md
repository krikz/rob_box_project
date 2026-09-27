# ADR-0048: per-profile `agent.max_turns` — `tester` поднять с 30 до 60 (системный фикс под test-suite generation)

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-02 |
| Автор | architect (Hermes Agent); ретро-карточка `t_dc0ec573`, sub-task `t_c401ecaa` (4 timed_out в ряд) |
| Контекст | На доске `robbox` карточка `t_c401ecaa` (tester, цель — дописать unit-тесты для `TTSSettings` по issue #1780) **5 раз подряд** упала с `Iteration budget exhausted (30/30)`. Проверка `kanban.db` показала, что **все 11 событий 30/30 за всю историю доски** принадлежат профилю `tester` — это аномалия, не случайность. |
| Затрагивает | (a) `~/.hermes/profiles/tester/config.yaml::agent.max_turns` (30 → 60); (b) `~/.hermes/profiles/backend/config.yaml::agent.max_turns` остаётся 180, но **для test-fix задач** рекомендация использовать `task.goal_max_turns=45` через карточку; (c) документация профилей (комментарий в YAML). |
| Связанные | `t_dc0ec573` (эта), `t_c401ecaa` (data source), `t_5f1346da` и `t_f1fecf1b` (та же 30/30 аномалия), issue #1780 (закрыт, остались регрессионные тесты), `agent/turn_finalizer.py:193` (источник сообщения "Iteration budget exhausted"), `hermes_cli/config.py:3382` (`_normalize_max_turns_config` — нормализация `agent.max_turns` из профиля в iteration cap). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

Карточка `t_c401ecaa` — это **тестовое расширение** под issue #1780 (эмоция/pitch/volume в MiniMax TTS + SSML pitch в Yandex). Сам issue закрыт (PR #1823 зафиксил `_parse_optional_int/float`), но `tester` должен дописать unit-тесты на `TTSSettings` pass-through (10 полей: voice/model/language/speed/volume/pitch/emotion/sample_rate/format/text_normalization + `pronunciation_dict` per #1780).

10 runs `t_c401ecaa`, из них **5 timed_out + 5 gave_up**, все с одной ошибкой:
```
Iteration budget exhausted (30/30) — task could not complete within the allowed iterations
```

Аналогичная картина у `t_5f1346da` (≥ 3 timed_out) и `t_f1fecf1b` (1+ timed_out). Все — профиль `tester`.

### 1.2 Корневая причина (фактчек через `kanban.db` + agent code)

Запрос к реальной доске `/home/builder/.hermes/kanban/boards/robbox/kanban.db`:

| Метрика | Значение |
|---|---|
| Всего событий "30/30 timed_out" в `task_events` | 11 |
| Из них от профиля `tester` | **11 (100%)** |
| Из них от других профилей | 0 |
| Карточек tester с `goal_mode=1` | **0 из 14** |
| Карточек tester с явным `goal_max_turns` | 0 из 14 |

Источник сообщения — `agent/turn_finalizer.py:193` (verified against `ha-baseline-4f00314e` checkout 2026-09-02):
```python
f"⚠️ Iteration budget exhausted ({api_call_count}/{agent.max_iterations}) "
```
`agent.max_iterations` инициализируется в `agent/agent_init.py:670`:
```python
agent.max_iterations = max_iterations
```
где `max_iterations` приходит из `hermes_cli/cli_agent_setup_mixin.py:530`:
```python
max_iterations=self.max_turns,
```
`self.max_turns` — это `agent.max_turns` из профиля (см. `hermes_cli/cli_commands_mixin.py:1543` / `2374`). Нормализация legacy `max_turns` → `agent.max_turns` — `hermes_cli/config.py:3382` (`_normalize_max_turns_config`).

**Распределение `agent.max_turns` по профилям** (реальные значения из `~/.hermes/profiles/*/config.yaml`):

| Профиль | `agent.max_turns` |
|---|---|
| **tester** | **30** ← единственная аномалия |
| backend | 180 |
| developer | 180 |
| devops | 150 |
| agent-flow | 150 |
| base | 150 |
| embedded, frontend, ml-engineer, llm-expert, dba, pm, cad-engineer, designer | 150 |
| pr-reviewer | 80 |
| analyst, architect, ros2-engineer | 50 |
| techwriter | 40 |

`tester` — **единственный профиль с `agent.max_turns=30`** и единственный, кто показывает 30/30 в логах.

### 1.3 Почему именно 30 для tester — слишком мало

`tester` пишет test-suite. Один unit-test в среднем занимает у LLM:

1. Read existing test file (1 turn).
2. Read code-under-test + fixture (1 turn).
3. Write `TestCase` skeleton (1 turn).
4. Add parametrization или edge case (1-2 turns).
5. Run pytest (1 turn).
6. Fix typos / import errors / failure messages (1-2 turns).
7. Verify pass + lint (1 turn).

Итого 7-10 turns на 1 unit-test. Для `TTSSettings` (10 полей × ~5 edge cases каждый = ~50 тестов, parametrize сжимает до ~15-20 test-cases) нужно **минимум 50-60 turns** только на генерацию. С учётом того, что воркер читает skill `sdlc-review` + project memory — реалистично 50-70.

**30 turns хватает на 2-3 теста + 1 fix.** Это объясняет, почему карточка упала 5 раз.

### 1.4 Почему это не баг диспетчера, а недоконфиг

Диспетчер корректно уважает per-profile `max_turns`. Это **не** система, которая «отбирает» ресурсы у tester'а — это **самоограничение**, заложенное в `~/.hermes/profiles/tester/config.yaml`. Когда tester был короткой ролью для smoke-tests, 30 хватало. Сейчас tester берёт на себя большие test-suite задачи — лимит не пересмотрен.

## 2. Три варианта решения (от карточки)

### Option A — минимальный, безопасный

```yaml
# ~/.hermes/profiles/tester/config.yaml
agent:
  max_turns: 60   # было 30
```

**Плюсы:** один файл, один diff, снимает блок с `t_c401ecaa` мгновенно. Не трогает другие профили. Не требует новых сущностей.

**Минусы:** решает только `tester`. Если завтра `backend` начнёт писать миграции БД в test-mode и тоже упрётся в 30 — придётся снова открывать ADR.

### Option B — системный (этот ADR)

(a) Поднять `tester` до **60**.
(b) Зафиксировать таблицу «profile → max_turns» как канон в `~/.hermes/profiles/<role>/config.yaml::agent` (комментарий).
(c) Добавить guideline в `docs/process/test-suite-budget.md` (новый файл): **для test-suite generation рекомендуется 50-60 turns**, для test-fix 40.
(d) Убедиться что `agent.max_turns=180` у `backend`/`developer` покрывает любые test-fix задачи (текущий запас 180 turns = 5x baseline).
(e) В `kanban_db` для карточек с `skills=["sdlc-review"]` (tester) — добавить auto-promt: «expected 50-70 turns, budget 60 OK».

**Плюсы:** документированное решение, повторно используемое. Шифу не придётся заново ловить эту аномалию на следующей test-suite карточке.

**Минусы:** чуть больше работы (один комментарий + один новый doc).

### Option C — точечный (только для `t_c401ecaa`)

```bash
hermes kanban edit t_c401ecaa --goal-max-turns=50
```

**Плюсы:** ноль изменений в config-файлах. Снимает блок локально.

**Минусы:** **не лечит причину**. Следующая test-suite карточка опять напорется на 30/30 → опять ретро → опять потеря времени.

## 3. Решение

**Принять Option B** с нюансом: поднимаем `tester` до **60** (не 50, потому что `sdlc-review` skill грузит дополнительный контекст и edge-cases на 10-полевом dataclass требуют запаса).

Дополнительно — рекомендация для Шифу: при создании test-suite карточки (assignee=tester, body содержит "test-suite" / "unit-tests" / "pytest") — **вручную** проставить `goal_max_turns=50` через карточку (не через config). Это страховка, пока конфиг не докажет, что 60 хватает. Через 2 недели — ретро-аудит: если 30/30 пропали, можно убрать этот manual step.

## 4. Что меняется в коде

### 4.1 Файл `~/.hermes/profiles/tester/config.yaml` (вне репо)

Это **пользовательский конфиг** (в `~/.hermes/profiles/tester/`), не в репозитории `rob_box_project`. ADR фиксирует, что фикс сделан **отдельно от этого PR** (прямая правка YAML на хосте), чтобы не блокировать merge ADR документацией.

```diff
 agent:
-  max_turns: 30
+  max_turns: 60
+  # ADR-0048: tester пишет test-suite (10+ полей dataclass → 50+ turns
+  # минимум для TTSSettings-class). 30 было историческим минимумом для
+  # smoke-test роли; поднят до 60 после 5 timed_out на t_c401ecaa
+  # (issue #1780 follow-up). Если опять видишь 30/30 — причина НЕ в
+  # этом профиле, см. ADR-0048 §6 диагностика.
   verbose: false
   reasoning_effort: medium
```

### 4.2 Новый файл `docs/process/test-suite-budget.md`

Краткая инструкция (15-20 строк):
- Когда создаёшь test-suite карточку (assignee=tester), планируй 50-70 turns.
- Если профиль показывает `agent.max_turns=30` — это **баг конфига**, открой issue.
- Для test-fix (не generation) хватает 40 turns.
- `sdlc-review` skill грузит ~3k контекста, учти в оценке.

### 4.3 Не трогаем

- `~/.hermes/profiles/*/config.yaml` других профилей (всё OK).
- `agent/turn_finalizer.py:193` (это **правильное** сообщение, оно сигналит о реальной проблеме).
- `hermes_cli/config.py:3382` (`_normalize_max_turns_config` — нормализация legacy root-level `max_turns` в `agent.max_turns`).

## 5. Что делаем с `t_c401ecaa`

Карточка сейчас `status=blocked, consecutive_failures=2, goal_mode=0`. После применения фикса:

1. `kanban unblock t_c401ecaa` (или новый spawn — конфиг уже поднят).
2. Воркер tester запускается с `max_turns=60`. По нашим оценкам — уложится в 50-55 turns.
3. Если опять 60/60 — это **новая аномалия** (skill не помогает, или задача сломана), ретро.

## 6. Диагностика — если опять 30/30 (или 60/60)

1. `sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db "SELECT assignee, COUNT(*) FROM task_events WHERE kind='timed_out' AND payload LIKE '%<X>/<X>%' GROUP BY assignee"` — если только tester, проблема персистентная и нужна новая итерация бюджета.
2. `grep -A2 'agent:' ~/.hermes/profiles/tester/config.yaml` — убедиться, что правка применилась (нет опечатки, нет revert).
3. Проверить, что не активирован `MAINTENANCE` файл (worker не запускается на полный бюджет).

## 7. Решение по Option C как fallback

Если Шифу **не хочет** трогать `tester/config.yaml` (например, потому что не уверен в 60 — может хватить 45?), то **Option C как временная мера** — `hermes kanban edit t_c401ecaa --goal-max-turns=45`. Это снимет блок с конкретной карточки **немедленно**, без config-изменений. Но это **не** решение системной проблемы.

**Рекомендация архитектора: Option B.** 5 timed_out — это сигнал, что дефолт неправильный, и следующий test-suite тоже упрётся. Точечный fix откладывает проблему.

## 8. Rollback plan

Если после merge `max_turns=60` начнёт ломать что-то другое (например, tester зависает на длинных задачах и тратит много токенов) — откат одной строки в `tester/config.yaml` + новая ретро-карточка. Изменение полностью локальное, blast-radius = tester-only.

## 9. Verification

После merge:

```bash
# 1. config принят
grep max_turns ~/.hermes/profiles/tester/config.yaml
# Ожидаем: max_turns: 60

# 2. следующий tester-таск не падает 30/30
# (через 1-2 недели)
sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT COUNT(*) FROM task_events e JOIN tasks t ON t.id=e.task_id
   WHERE t.assignee='tester' AND e.kind='timed_out'
   AND e.payload LIKE '%(60/60)%'"
# Ожидаем: 0 или падение относительно baseline (было 11)

# 3. t_c401ecaa завершена
sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT status FROM tasks WHERE id='t_c401ecaa'"
# Ожидаем: done
```

## 10. Lessons learned (для будущих ретро про budget)

1. **Сам архитектор — кандидат на «X/X»**: при написании этого ADR карточка `t_dc0ec573` (architect, run 3472) исчерпала `50/50` — профиль `architect` имеет `agent.max_turns=50` (см. §1.2 таблицу). На ADR + amendment ушло ~45 turns (verify профилей + правка ссылок + push + PR). Это **следующая** аномалия, которая появится, если кто-то попросит архитектора написать многосекционный документ. **Pre-emptive action**: Шифу может поднять `architect` до **80-100 turns** в том же PR (комментарий в YAML + новый пункт §4.4). Этот ADR **не** включает правку `architect` намеренно — чтобы scope оставался узким, иначе теряется single-responsibility. Отдельная ретро-карточка про `architect` будет создана при первом `50/50` от архитектора.

2. **Цитаты в ADR требуют verbatim-проверки через `grep -n`**, не по памяти. Первая версия этого документа содержала неточные ссылки (`turn_finalizer.py:133`, `config.py:1013`) — поправлены в §1.2 на реальные `193` и `3382` (verified против `ha-baseline-4f00314e` 2026-09-02).

3. **`hermes-share/rob_box_project`** — это symlink-managed копия основного репо (`scripts/sync_files.py`). Воркеры, получившие `worktree_kind=dir`, должны работать здесь, а не в `~/rob_box_project` напрямую — иначе push летит в другой remote и PR не создаётся.
