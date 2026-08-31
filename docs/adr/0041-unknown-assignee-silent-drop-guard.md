# ADR-0041: unknown assignee → silent drop (защита dispatcher'а)

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-09-01 |
| Автор | architect (Hermes Agent); ретро-карточка `t_e8b3df13` |
| Контекст | Архитектор создал карточку `t_9f0195ab` с `assignee=agent-flow-developer` — профиль, которого НЕ существует в системе. Диспетчер молча положил карточку в `ready` и оставил там на 22 часа. Никакого warning, никакого отказа. Карточка не имела шансов быть выполненной — и никто об этом не узнал, пока архитектор не провёл ретроспективу. |
| Затрагивает | (a) `hermes_cli/kanban_db.py::create_task` (расширение vendor-патча из ADR-0036 — новая функция `_validate_assignee_known`); (b) `hermes_cli/kanban_dispatch.py` (еже-tick-логирование `unresolved_assignees`); (c) `scripts/agent_flow/watchdog.sh` (новый блок `unresolved-assignee-watch`); (d) `CONTRIBUTING.md §2f` (правило «assignee должен быть в списке known профилей»). |
| Родители | ADR-0036 §4.1 (scope-hint vendor-патч — расширяем тот же механизм), ADR-0023 (`_validate_skills_for_assignee`), ADR-0018 (honesty culture — silent drop нарушает её) |
| Связанные | `t_e8b3df13` (эта), `t_9f0195ab` (stuck-карточка 22ч, mis-scope #4), `t_e2ae0c29` / `t_7a7463f4` (mis-scope #1-3 — другой класс, scope-hint), `t_6c6c98fb` (skill чужого профиля — прецедент), PR #1833 (ADR-0040), Issue #1831 |

## TL;DR

Карточки kanban с `assignee`, указывающим на несуществующий профиль, **невозможно обнаружить без ручного аудита**. Диспетчер их не пикает (профиль не зарегистрирован → некому spawn'ить worker), не выкидывает (формально валидно — это просто строка), не предупреждает создателя (нет валидации на стадии `kanban create`). Карточка сидит в `ready` вечно, **потребляя board real-estate и доверяя наблюдателя**, что кто-то «увидит и поправит».

Этот ADR вводит:

1. **Fail-fast валидация на стадии `kanban create`**: если `assignee` не входит в known profiles (`hermes profile list`), `kanban create` возвращает non-zero exit + понятное сообщение.
2. **Watchdog-блок**: каждые 2 минуты watchdog проверяет `status=ready AND assignee NOT IN known_profiles` для всех карточек; если такие есть — авто-комментарий в каждую (от имени watchdog) с указанием, какой профиль был запрошен и список known alternatives.
3. **Cron-надзор**: если карточка остаётся `ready+unknown_assignee` > 4 часов, watchdog **создаёт triaging-карточку** на pr-reviewer с ссылкой на зависшую, чтобы человек увидел на следующем утреннем cron-tick.
4. **`CONTRIBUTING.md §2f` дополняется** разделом «Known profiles (SOT = `hermes profile list`)».

Цель: zero silent drops. Если создатель опечатался — fail-fast на стадии create. Если профиль удалили между create и pick-up — watchdog поймает в течение 2 минут.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (01.09.2026 ~22:00Z)

Ретро-карточка `t_e8b3df13`: архитектор создал `t_9f0195ab` «реализация ADR-0040» с `assignee=agent-flow-developer`. Через 22 часа карточка всё ещё `ready`, не выполняется. Проверка:

```
$ hermes profile show agent-flow-developer
Error: Profile 'agent-flow-developer' does not exist.

$ hermes profile list | head -5
  default         MiniMax-M3     running
  agent-flow      MiniMax-M3     running
  analyst         MiniMax-M3     stopped
  ◆architect      MiniMax-M3     running
  backend         MiniMax-M3     stopped
  ...
```

Профиля `agent-flow-developer` нет. Есть `agent-flow` (для супервизора pipeline'а) и `devops` (для реализации скриптов, что и нужно было).

### 1.2 Почему это mis-scope класса #4

Паттерн mis-scope (4 случая за месяц):

| # | Карточка | Symptom | Корневая причина |
|---|---|---|---|
| 1 | t_e2ae0c29 | backend + TDD на architectural task | assignee формально в known profiles, но scope-hint отсутствовал |
| 2 | t_7a7463f4 | retry t_e2ae0c29 | то же самое после reassign |
| 3 | (recurring) | skill чужого профиля (t_6c6c98fb) | `_validate_skills_for_assignee` есть, но не на content |
| 4 | **t_9f0195ab** | **assignee=agent-flow-developer** | **профиля НЕ СУЩЕСТВУЕТ, диспетчер silent-drop** |

Случаи 1-3 ловились бы scope-hint'ом (ADR-0036 §4.1, merged). Случай 4 — **ортогональная дыра**: assignee невалиден на уровне существования профиля, а не на уровне scope-соответствия.

### 1.3 Почему не сработали текущие защиты

| Слой | Что должен ловить | Почему не сработал |
|---|---|---|
| `kanban create` schema | assignee — non-empty string | строка валидна, формат не проверяется |
| Dispatcher claim loop | unknown profile → drop or warn | drop без логирования (silent) |
| Watchdog stale-heartbeat | no heartbeat > 600 сек | карточка в `ready`, не в `running` — watchdog считает её «не застрявшей» (нет процесса → нет аномалии) |
| User / cron-надзор | board scan для stuck cards | ручной скан, никто не запускает регулярно |
| `_validate_skills_for_assignee` (ADR-0023) | skill ⊆ profile.skills | assignee проходит первичную проверку (строка непуста), skill vs assignee проверяется |

**Корневая причина**: dispatcher (и весь пайплайн) трактует `assignee` как opaque string и полагается на «кто-то его создаст или удалит». Если профиль не существует — карточка **не имеет шансов быть выполненной**, и никто не сигналит.

### 1.4 Бизнес-последствие (если не чинить)

- 22-часовая задержка реализации ADR-0040 = **задержка починки e2e-process round-spam** (issue #1831 продолжает генерировать ~30 ghost round-веток/день).
- Каждая silent-drop карточка = **потраченное LLM-время на создание** + **невозможность автоматического обнаружения** + **ручной аудит для исправления**.
- Нарушение ADR-0018 (honesty culture): silent drop = молчаливое «всё ок» при фактическом «карточка мертва».
- Расход board real-estate: карточки-захоронения занимают место в `ready` колонке, мешают обзору.

---

## 2. Где SOT и какие слои трогаем

### 2.1 Источник истины для known profiles

`hermes profile list` (вызов `hermes_cli/profile_registry.py::list_profiles`) — единственный SOT. Диспетчер **уже** использует эту функцию при claim-loop:

```python
# hermes_cli/kanban_dispatch.py (псевдокод существующего claim)
known = profile_registry.list_profiles()  # set[str]
if task.assignee not in known:
    log.debug(f"unknown assignee {task.assignee} on task {task.id}, skipping claim")
    continue  # ← SILENT — единственное место, где профиль отбрасывается
```

**Дыра**: `log.debug` (не видно в normal logs), `continue` (нет side-effect → не остаётся следа в task history).

### 2.2 Vendor-патч — расширяем существующий

`scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` (ADR-0023 §2.5, ADR-0036 §4.1) — единое место для pre-create валидаций. **Расширяем** тот же patch новой функцией `_validate_assignee_known`:

```python
def _validate_assignee_known(assignee: str) -> None:
    """Fail-fast: assignee must be in known profiles (SOT = hermes profile list)."""
    from hermes_cli import profile_registry
    known = profile_registry.list_profiles()
    if assignee not in known:
        # Suggest closest match (Levenshtein distance ≤ 3) to help typos
        suggestion = _suggest_closest(assignee, known, max_distance=3)
        msg = f"assignee={assignee!r} is not a known profile."
        if suggestion:
            msg += f" Did you mean {suggestion!r}?"
        msg += f"\nKnown profiles: {sorted(known)}"
        msg += "\nRun: hermes profile list  (SOT)"
        raise ValueError(msg)
```

Вызывается в `kanban_db.py::create_task` **перед** записью в БД — fail-fast за <100 ms.

### 2.3 Watchdog — новый блок `unresolved-assignee-watch`

`scripts/agent_flow/watchdog.sh` (ADR-0036 §4.2) — single SOT для runtime-anomaly detection. Расширяем новым блоком (каждые 2 мин, между stale-heartbeat и runtime-overshoot):

```bash
# Block: unresolved-assignee-watch
# Детектирует: status=ready AND assignee not in $(hermes profile list --names-only)
# Действие: (a) авто-комментарий "watchdog: assignee unknown since creation"; (b) если >4ч — создать triaging-карточку pr-reviewer.
```

**Не отдельный крон**, а блок внутри существующего — DRY, один SOT для всех runtime-anomaly.

### 2.4 CONTRIBUTING.md §2f

Дополняется разделом:

```markdown
### §2f.4 Known profiles (SOT)

Список профилей фиксирован и доступен через `hermes profile list`.
Перед `kanban create` — проверьте assignee в выводе. **Создание карточки
с неизвестным assignee = silent drop** (см. ADR-0041) и 22-часовая задержка
выполнения.

Если профиля, который вам нужен, нет — это **отдельная задача на pr-reviewer**
(создать профиль), не workaround «прописать assignee наугад».
```

---

## 3. Инвариант

**Карточка с `assignee`, не входящим в known profiles, не может оставаться в `ready` больше 4 часов без обнаружения.**

- На стадии `kanban create`: fail-fast за <100 ms (если профиль не существует на момент создания).
- На стадии runtime: если профиль удалили **между create и pick-up** (race-condition при admin-действиях) — watchdog ловит в течение 2 минут + создаёт triaging-карточку через 4 часа.
- На стадии discovery: `hermes kanban --board X show <id>` всегда показывает assignee (уже работает) + `kanban ready --unresolved-assignee` (новый CLI-флаг) — для ad-hoc аудита.

---

## 4. Решение

### 4.1 Vendor-патч: `_validate_assignee_known` (fail-fast на create)

В `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` добавляется (рядом с `_validate_skills_for_assignee`):

```python
def _validate_assignee_known(assignee: str) -> None:
    """Fail-fast: assignee must be in known profiles (SOT = hermes profile list)."""
    known = profile_registry.list_profiles()
    if assignee not in known:
        suggestion = _suggest_closest(assignee, known)
        msg = (
            f"BLOCKED: assignee={assignee!r} is not a known profile.\n"
            f"Known profiles: {sorted(known)}\n"
            f"Run: hermes profile list  (single source of truth)"
        )
        if suggestion:
            msg += f"\nDid you mean {suggestion!r}?"
        raise ValueError(msg)


def _suggest_closest(target: str, candidates: set[str], max_distance: int = 3) -> str | None:
    """Return closest match by Levenshtein distance, or None if no good match."""
    # stdlib only; ≤30 candidates → O(N×|target|) fine
    best = None
    best_dist = max_distance + 1
    for c in candidates:
        d = _levenshtein(target.lower(), c.lower())
        if d < best_dist:
            best = c
            best_dist = d
    return best if best_dist <= max_distance else None
```

Вызывается в `kanban_db.py::create_task` **перед** `INSERT INTO tasks`:

```python
def create_task(*, title, assignee, body, ..., skills=None, ...):
    # ... existing param validation ...
    _validate_assignee_known(assignee)  # ← NEW: fail-fast
    _validate_skills_for_assignee(assignee, skills or [])  # existing from ADR-0023
    # ... INSERT ...
```

### 4.2 Watchdog-блок: `unresolved-assignee-watch`

В `scripts/agent_flow/watchdog.sh` добавляется новый блок (после `stale-heartbeat` и до `runtime-overshoot`):

```bash
# === unresolved-assignee-watch ===
# Детектирует: status=ready AND assignee NOT IN known_profiles.
# Действие 1 (немедленно): авто-комментарий "watchdog: assignee unknown".
# Действие 2 (после 4ч): создать triaging-карточку на pr-reviewer.

KNOWN_PROFILES=$(hermes profile list --names-only --format json | jq -r '.[]' 2>/dev/null || true)
[ -z "$KNOWN_PROFILES" ] && { log_warn "watchdog: cannot read known profiles, skipping block"; return 0; }

KNOWN_RE=$(printf '%s\n' "$KNOWN_PROFILES" | paste -sd'|' -)

sqlite3 "$KANBAN_DB" "
  SELECT id, assignee, created_at
  FROM tasks
  WHERE status = 'ready' AND assignee NOT IN ($(printf "'%s'," $KNOWN_PROFILES | sed 's/,$//'))
" | while IFS='|' read -r tid assignee created_at; do
    [ -z "$tid" ] && continue
    age_sec=$(( $(date +%s) - created_at ))

    # Действие 1: всегда комментарий (idempotent — только если последний watchdog-коммент старше 1ч)
    if ! has_recent_watchdog_comment "$tid" 3600; then
        post_comment "$tid" "$(cat <<EOF
🐺 watchdog (ADR-0041): assignee \`${assignee}\` НЕ в known profiles.
Карточка не будет подобрана диспетчером. Решение:
1. Если assignee опечатка — \`hermes kanban --board $(basename $(dirname $KANBAN_DB)) update $tid --assignee <known>\`
2. Если профиля нет — задача на pr-reviewer: создать профиль
Known profiles: \`$(echo $KNOWN_PROFILES | tr '\n' ' ')\`
EOF
)"
    fi

    # Действие 2: после 4ч → triaging-карточка на pr-reviewer
    if [ "$age_sec" -gt 14400 ]; then
        if ! has_open_triage_for "$tid"; then
            create_triage_card "$tid" "assignee=${assignee} не существует"
        fi
    fi
done
```

### 4.3 CLI-флаг для ad-hoc аудита

В `hermes_cli/kanban_cli.py` добавляется команда:

```bash
hermes kanban --board X ready --unresolved-assignee
# Output:
# t_9f0195ab  ready  agent-flow-developer  22h stuck  "impl ADR-0040"
```

Используется для разовой проверки + ежедневного cron-аудита (отдельная задача, не в этом ADR).

### 4.4 CONTRIBUTING.md §2f — дополнение

Дописывается раздел «§2f.4 Known profiles (SOT)» с явным указанием: «Создание карточки с неизвестным assignee = silent drop (см. ADR-0041)».

---

## 5. Альтернативы и почему не выбраны

### 5.1 Альтернатива: «let it silently drop, board operators будут сканировать»

- **Плюс**: zero code change.
- **Минус**: точно текущее поведение, 22-часовая задержка уже случилась. Не соответствует ADR-0018 (honesty).

### 5.2 Альтернатива: «kill dispatcher unknown-assignee claim loop entirely»

- **Плюс**: dispatcher падает на unknown → карточка `ready` вечно, но диспетчер хотя бы **пишет error log**.
- **Минус**: error log всё равно `log.debug`, не видно; не решает ad-hoc audit; race-condition с удалением профилей между create и pick-up всё равно опасен.

### 5.3 Альтернатива: «авто-suggest assignee при create (ML)»

- **Плюс**: user-friendly.
- **Минус**: over-engineering; Levenshtein на 30 строк делает то же за 0.1 ms.

---

## 6. Что НЕ делаем

- **Не делаем** авто-reassign карточки на «ближайший» профиль — слишком агрессивно, может затереть намерение создателя. Только fail-fast + suggest.
- **Не валидируем** title/body на этапе create (это scope-hint из ADR-0036, отдельный класс задач).
- **Не удаляем** карточки автоматически (даже после triaging-карточки от pr-reviewer) — финальное решение за человеком.
- **Не трогаем** существующие карточки в `ready` (t_9f0195ab уже архивирована архитектором в рамках ретро).

---

## 7. Acceptance (test plan)

- [ ] **Test 1 (fail-fast create)**: `hermes kanban --board robbox create --assignee=nonexistent ...` → exit 1, stderr содержит «BLOCKED: assignee='nonexistent' is not a known profile» + sorted list known profiles.
- [ ] **Test 2 (suggestion)**: `hermes kanban create --assignee=architet ...` (опечатка) → exit 1 + «Did you mean 'architect'?»
- [ ] **Test 3 (watchdog immediate comment)**: создать карточку напрямую в БД (обходя `_validate_assignee_known`) с `assignee=foo`, status=ready → следующий тик watchdog (≤2 мин) добавляет комментарий «🐺 watchdog (ADR-0041): assignee foo НЕ в known profiles».
- [ ] **Test 4 (triage after 4h)**: карточка с unknown assignee, created_at=5ч назад → watchdog создаёт triaging-карточку на pr-reviewer.
- [ ] **Test 5 (CLI ad-hoc audit)**: `hermes kanban --board X ready --unresolved-assignee` показывает список (без значений assignee в known).
- [ ] **Test 6 (CONTRIBUTING.md)**: §2f.4 присутствует, ссылается на ADR-0041.

---

## 8. Связи с другими ADR

| ADR | Связь |
|---|---|
| ADR-0036 | scope-hint для known profiles. ADR-0041 — расширение на уровень «профиль существует» |
| ADR-0023 | `_validate_skills_for_assignee` — pre-existing pattern, DRY через тот же vendor-патч |
| ADR-0018 | honesty culture: silent drop нарушает — ADR-0041 устраняет |
| ADR-0026 | recovery contract: triaging-карточка от watchdog использует тот же формат |
| ADR-0040 | триггер ретро: 22-часовая задержка реализации ADR-0040 = прямое бизнес-последствие |

---

## 9. Открытые вопросы

- **Q1**: `hermes profile list` возвращает profiles по `~/.hermes/profiles/<name>/`. Если профиля нет в ФС, но есть в `profiles.json` (например, vendor default) — какой SOT? Предлагаю: SOT = filesystem (`~/.hermes/profiles/`), а не `profiles.json`. Решается на этапе реализации.
- **Q2**: если `kanban create` идёт через CLI Шифу, fail-fast error от python → понятное сообщение в stderr? Или нужно отдельное форматирование? (Можно использовать `rich` console из существующего CLI.)
- **Q3**: triaging-карточка от watchdog — assignee=pr-reviewer или assignee=user? user лучше (Шифу принимает решения), но требует больше внимания; pr-reviewer — обычный flow. По умолчанию = user.
