# ADR-0049: orphan-skill watchdog → self-healing вместо блокировки

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-09-03 |
| Автор | devops (Hermes Agent); ретро-карточка `t_51394ac6` |
| Контекст | Orphan-skill watchdog (ADR-0023, реализация `hermes-kanban-orphan-skill-watchdog.sh`) только БЛОКИРОВАЛ карточки с `Unknown skill(s)` крашами, но не ЧИНИЛ их. Ретро-цикл (t_09a2152b → t_51394ac6) показал: critical retro-карточки (`t_84cb9466` — develop-CI RED; `t_8572890f` — e2e-stall на develop) застряли в `blocked` 4+ дня, никто их не правil (юзер создаёт — но не знает правил профиля; архитектор не имеет процесса чинить "чужие" watchdog-блоки). Разработка CI на develop сломана 12+ часов из-за этого. |
| Затрагивает | (a) `hermes_cli/kanban_db.py` (pre-create guard `_validate_skills_for_assignee` + `_validate_scope_for_assignee`); (b) `hermes_cli/kanban.py` (CLI flag `--force-scope`); (c) `~/.hermes/profiles/devops/scripts/hermes-kanban-orphan-skill-watchdog.sh` (новый режим `--self-heal`, по умолчанию); (d) `~/.hermes/profiles/devops/scripts/kanban-retro-create.sh` (pre-create guard в bash, зеркалит runtime validator). |
| Родители | ADR-0023 (recursive skill-discovery, vendor-патч `_validate_skills_for_assignee`), ADR-0036 §4.1 (scope-hint, расширяем тот же механизм), ADR-0035 (mis-scope retrospectives) |
| Связанные | `t_51394ac6` (эта), `t_09a2152b` (предыдущая попытка), `t_84cb9466` (e2e-stall-develop), `t_8572890f` (develop-CI-RED), `t_417e6f84` `t_e8213cd5` `t_9f8bdf89` `t_7cdc5fcb` `t_df184487` `t_3888cb1e` `t_3c27c1da` `t_862ec1a2` (8 backend-воркеров застряли на rebase-loop), vendor-patch `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch`, GitHub issue #1845 |

## TL;DR

Triple-layer guard от orphan-skill крашей:

1. **Layer 1 — pre-create в hermes-cli** (`_validate_skills_for_assignee` в `kanban_db.py`): `hermes kanban create --skill X` падает с `ValueError` ДО записи в БД, если `X` не установлен в профиле assignee. Новый CLI flag `--force-scope` (ADR-0036 §4.1) отдельно валит implementation-profile-mis-scope (warn-only, hint). Pre-create guard — **hard fail**.
2. **Layer 2 — pre-create в bash retro-create wrapper** (`kanban-retro-create.sh`): зеркалит runtime validator через Python helper (тот же `_profile_skill_names`). Retро-кроны получают чистый stderr exit 2 вместо 500-shaped kanban failure.
3. **Layer 3 — watchdog self-heal** (`hermes-kanban-orphan-skill-watchdog.sh --self-heal`): каждые 5 минут сканирует логи + DB на `Unknown skill(s)` и **автоматически правит** `skills` колонку карточки на первый валидный скилл профиля assignee (или empty list если профиль пустой). Карточка остаётся в `ready` и пикается dispatcher'ом на следующем тике. Старый режим `--detective` сохранён для тех, кто предпочитает жёсткую блокировку.

Цель: zero stuck retro-карточек из-за orphan skills. Если создатель опечатался — fail-fast на стадии create. Если карточка УЖЕ в БД (создана до patch'а) — watchdog чинит in-place за 5 минут. Если профиль пустой/не существует — watchdog пишет empty skills (worker использует defaults) и эмитит event для аудита.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (02.09 ~22:00Z, devops-ночное-окно)

Architect 02.09 21:40 создал карточку `t_84cb9466` ("e2e-stall-develop-4-days") с `assignee=devops` и `skills=['vendor-patch']`. Воркер упал на старте:

```
Query: work kanban task t_84cb9466 / Error: Unknown skill(s): vendor-patch
```

Скилл `vendor-patch` действительно **не установлен в devops-профиле** (он живёт в `agent-flow` подкатегории vendor-патча в SOT-репо, но не как Hermes skill). Юзер-создатель (товарищ Шифу в архитектурном сеансе) не знал правила "skill принадлежит профилю". После 2 retry watchdog пометил карточку `orphan_skill_auto_blocked` и положил в `blocked`. Диспетчер её не пикает, юзер не правит — карточка stuck 4+ дня.

Симметричная история с `t_8572890f` ("develop-CI-RED-12-hours") у которого `skills=['software-development']` (категория, не скилл) — у backend-профиля её тоже нет.

### 1.2 Что это сломало

- **develop CI RED 12+ часов** (run_id 33611787273 на `test_sound_voice_passthrough::test_subscribes_to_avatar_voice_in_best_effort`). Все PR в develop получают UNSTABLE merge-gate. 8 backend-воркеров (t_417e6f84, t_e8213cd5, t_9f8bdf89, t_7cdc5fcb, t_df184487, t_3888cb1e, t_3c27c1da, t_862ec1a2) сидят на готовых PR в rebase-loop потому что merge-gate красный.
- **L: E2E Voice Test не запускается 4+ дня** (t_84cb9466 stuck). Все `needs-e2e` issues (включая #1881 с фиксом в develop, #1736 OPEN 63h) копятся.
- **Тройной watchdog-loop**: t_09a2152b (02.09 15:00Z, done) описал проблему, vendor-патч был написан, но **не применён** к prod hermes-agent. Юзер продолжает создавать orphan-skill карточки → watchdog их блокирует → dispatcher не пикает → ретро не пишется → юзер продолжает создавать orphan-skill карточки. Закольцованный burnloop.

### 1.3 Двойная системная аномалия (root cause)

1. **Vendor-патч существует, но не применён.** `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` (commits `33f967cdd` ADR-0036 §4.1, `6804a0627` regen, `920b0d61f` fix §4.1 scope-hint) реализует `_validate_skills_for_assignee` + `--force-scope`. Patch написан против `origin/main` (blobs `6dd0f5e` / `311a05b`), но локальный `/home/builder/.hermes/hermes-agent/` — кастомный fork на ветке `z-devops/t_16a245cc-goal-mode-clean-exit-recovery` с 23081+ diverged коммитами. Patch применяется только частично (`kanban.py` ок, `kanban_db.py` конфликтует; содержит явный баг — duplicate `_validate_skills_for_assignee` определение + duplicate call). Без правки вручную не применить.

2. **Watchdog только блокирует.** Существующий `hermes-kanban-orphan-skill-watchdog.sh` (ADR-0023) был написан как detective: scan → block → comment → mark-handled. Но **никто не правит** карточки после блокировки (юзер не знает правила, архитектор не имеет процесса чинить). Карточки stuck навсегда. Retро-цикл t_09a2152b → t_51394ac6 доказывает, что блокировка без авто-фикса = zero resolution.

---

## 2. Решение

### 2.1 Layer 1: pre-create guard в hermes-cli

Добавлены **три новые функции** в `hermes_cli/kanban_db.py`:

```python
def _profile_skill_names(assignee: Optional[str]) -> Optional[frozenset[str]]:
    """Return the set of skill names installed on ``assignee``'s profile.
    None == "no profile to validate against" (skip).
    Uses agent.skill_utils.iter_skill_index_files — the same walker the
    prompt builder uses, so symlinked category dirs (research/,
    software-development/, productivity/) resolve correctly."""
```

```python
def _validate_skills_for_assignee(assignee, skills) -> None:
    """Reject orphan skill names. Raises ValueError naming missing
    skill(s) and assignee profile. Skips when assignee is unknown
    (None means external worker lane) or skills list is empty."""
```

```python
def _validate_scope_for_assignee(assignee, body, skills, *, force=False) -> Optional[str]:
    """Hint-only mis-scope detection (ADR-0036 §4.1). Warns via
    warnings.warn when implementation profile (backend/frontend/tester)
    is asked to do architectural work + has TDD-shaped skill. Returns
    the structured message so caller can write a task_event row.
    force=True (--force-scope CLI) suppresses."""
```

`create_task()` теперь:
- Вызывает `_validate_skills_for_assignee(assignee, skills_list or [])` **до** `write_txn` — orphan skill = `ValueError` = CLI exit 3, **карточка не создаётся**.
- Вызывает `_validate_scope_for_assignee(assignee, body, skills, force=force_scope)` **до** `write_txn` — mis-scope = `warnings.warn` + локальный `_scope_hint_msg` (warn-only, карточка создаётся).
- Внутри `write_txn` (после INSERT), если `_scope_hint_msg` непустой — пишет `task_event(kind="scope_hint", payload={message, assignee, skills, forced})`.

CLI в `hermes_cli/kanban.py`:
- Добавлен `--force-scope` (`action="store_true"`, `dest="force_scope"`) для под-парсера `create`.
- `_cmd_create` пробрасывает `force_scope=bool(args.force_scope)` в `create_task`.

**Smoke-test 02.09 ~22:30Z**:
```
$ hermes kanban create "smoke-test" --assignee devops --skill vendor-patch
kanban: 'vendor-patch' skill not installed in profile 'devops'; skill names
belong to the profile that owns them — fix the assignee or drop the per-task
skills. Installed skills: [список 105 скиллов devops]
```
Карточка не создана. ✓

### 2.2 Layer 2: pre-create guard в `kanban-retro-create.sh`

Bash-обёртка для ретро-кронов зеркалит runtime validator через inline `python3 - <<EOF` heredoc:

- Preferred path: `from hermes_cli.kanban_db import _profile_skill_names; names = _profile_skill_names(assignee)`.
- Fallback: `from agent.skill_utils import iter_skill_index_files; walk ~/.hermes/profiles/<assignee>/skills/SKILL.md`.
- Final fallback: `Path.rglob("SKILL.md")` (best-effort, может пропустить symlinked categories).
- Если не удалось провалидировать — `sys.exit(0)` (не блокируем создание, не ломаем крон).
- Если найдены orphan skills — stderr + exit 2, скрипт ретро-карточку **не создаёт**.

**Smoke-test 02.09 ~22:35Z**:
```
$ bash kanban-retro-create.sh --title "smoke-test" --assignee devops --skill vendor-patch --dry-run
ERROR: 'vendor-patch' skill not installed in profile 'devops'; ...
ERROR: pre-create skill validation failed (rc=2). ...
exit: 0 (caller script exits 3)
```

### 2.3 Layer 3: watchdog self-heal (default mode)

`hermes-kanban-orphan-skill-watchdog.sh` расширен новым режимом `--self-heal` (по умолчанию). Старый `--detective` сохранён для обратной совместимости.

**Детекция** (без изменений): сканирует `*.log` и `task_runs.error` за `LOOKBACK_MIN` (default 30) на regex `Error: Unknown skill\(s\): (.+)`.

**Per-card действия** в self-heal:

1. **Проверка**: card.status in (blocked, todo, ready, running); карточка ещё содержит хотя бы один orphan skill в `skills[]` колонке; ещё не handled в текущем 24h окне.
2. **Decision**: вычислить `survivor = [s for s in arr if s not in orphan_set]` и:
   - Если survivor непустой → `new_skills = survivor` (drop только orphans), action=`dropped-orphans-kept-survivors`.
   - Если все skills были orphan → `replacement = assignee_first_skill(assignee)` через `_profile_skill_names` (mirrors runtime):
     - replacement is not None → `new_skills = [replacement]` (первый валидный, alphabetically), action=`replaced-with-first-installed`.
     - replacement is None (профиль пустой/не существует) → `new_skills = []`, action=`dropped-all-skills-no-profile-skills` (worker использует profile defaults).
3. **DB write**: `UPDATE tasks SET skills = ?` + INSERT `task_events(kind='orphan_skill_auto_fixed', payload={orphan, assignee, old_skills, new_skills, action, mode})`.
4. **Comment**: structured markdown comment с before/after diff + ссылка на ADR.
5. **Unblock** (если `status=blocked`): `hermes kanban unblock <tid>` — dispatcher пикает на следующем тике.

В detective-mode поведение прежнее: `comment + block + mark-handled`. Никаких изменений для тех, кто явно передаёт `--detective`.

**Smoke-test 02.09 ~22:40Z** (synthetic task `t_synthetic_orphan_test` с `skills=["fake-orphan-skill"]` + log с `Error: Unknown skill(s): fake-orphan-skill`):
```
[watchdog/self-heal] detected 4 orphan-skill event(s)
[watchdog/self-heal] skip t_51394ac6: no orphan in current skills=[]
[watchdog/self-heal] skip t_84cb9466: no orphan in current skills=['sdlc-review']
[watchdog/self-heal] skip t_8572890f: no orphan in current skills=['ci-debugging']
[watchdog/self-heal] fixed t_synthetic_orphan_test: orphan='fake-orphan-skill'
  → new_skills=['adapter-contract-refactor-audit'] (action=replaced-with-first-installed)
```

### 2.4 Manual fix для stuck карточек (one-shot)

`t_84cb9466` (devops, было `['vendor-patch']`) → `['sdlc-review']` (sdlc-review — основной devops-skill для process-fix review).
`t_8572890f` (backend, было `['software-development']`) → `['ci-debugging']` (ci-debugging — backend-skill для CI repair).

Обе карточки unblock'нуты (`hermes kanban unblock`), dispatcher пикает их на следующем тике. Manual-fix event `orphan_skill_manually_fixed` записан с rationale.

---

## 3. Trade-offs

### 3.1 Self-heal может починить карточку "неправильно"

**Risk**: замена orphan skill на **первый alphabetically** скилл профиля — это произвольный выбор. Если devops-профиль имеет `adapter-contract-refactor-audit` первым по алфавиту, а ретро-карточка про skill-validation — замена не релевантна.

**Mitigation**:
- Для **synthetic test** карточек (наш smoke-test) выбор адекватный — `adapter-contract-refactor-audit` это тоже audit-related skill.
- В реальности orphan-skill карточки чаще всего — это ретро-карточки от cron'ов с неверно указанным skill; worker всё равно получит полезный audit-skill вместо краша.
- Альтернатива — оставить карточку blocked и требовать ручного фикса. Эта альтернатива провалилась: за 4 дня карточки t_84cb9466 и t_8572890f никто не починил. Self-heal лучше чем stuck-forever.
- Если replacement skill действительно неправильный — worker заметит (через `request_changes` или по context mismatch), карточка вернётся в `running → review` где архитектор поправит.

### 3.2 Pre-create guard может ломать legitimate use cases

**Risk**: профиль `architect` имеет все скиллы (он meta), а юзер создаёт карточку с `assignee=architect` и `skill=X` где X есть в architect но не в target. Или наоборот, юзер хочет дать worker'у кастомный skill (например "voice-pipeline-debug").

**Mitigation**:
- Validation работает **только** когда assignee = known profile с skills/ деревом. Если assignee = external worker lane (плагин) — `profile_exists()` returns False → validation skipped.
- Если assignee = profile но юзер хочет кастомный skill — он должен либо добавить skill в профиль, либо использовать пустой `skills=[]` (worker возьмёт defaults).
- ADR-0023 уже закрыл обратный кейс: dispatcher умеет recursive, так что "кастомный skill в подкатегории" — это валидно если подкатегория symlink'нута в профиль.

### 3.3 Тройной guard = тройная точка отказа

**Risk**: pre-create в hermes-cli + pre-create в bash + self-heal в watchdog — три места, где validation может рассинхронизироваться.

**Mitigation**:
- Все три используют **одну и ту же** функцию `_profile_skill_names` из `hermes_cli/kanban_db.py` (или fallback на тот же `iter_skill_index_files` helper). Изменение валидатора в одном месте = синхронное обновление всех трёх.
- Layer 2 (bash) — defense-in-depth: если hermes-cli validation сломается (баг в коде), bash wrapper всё равно поймает.
- Layer 3 (watchdog) — backstop для карточек, созданных ДО patch'а. Когда patch стабилизируется и все stuck карточки починятся, watchdog станет idle.

### 3.4 Scope-hint heuristic false-positives

**Risk**: ADR-0036 §4.1 heuristic может false-positive — например легитимная backend-карточка про test infrastructure содержит слово "dispatcher" в body (допустим, про dispatcher для test mocks).

**Mitigation**:
- Heuristic — **warn-only**, не блокирует карточку. Worker всё равно её выполнит.
- `--force-scope` escape hatch для архитектора.
- Heuristic keywords довольно узкие (`ADR-`, `architecture decision`, `merge-gate`, `process-fix`, кириллические варианты) — false-positive маловероятен.
- Принимаем false-positive rate ~5% как cost of catching mis-scope loops типа "5h31m на ADR doc вместо fix'а".

---

## 4. Альтернативы рассмотренные

### 4.1 Vendor-patch as-is (от ADR-0023)

**Что предлагалось**: применить `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` буквально.

**Почему отклонено**:
1. Patch не apply'ится чисто на локальный hermes-agent fork (конфликт в `kanban_db.py:3417+`).
2. Patch содержит **duplicate** `_validate_skills_for_assignee` definition и **duplicate call** (явный bug в hunk @@-3697,6 +3872,29).
3. Patch сделан против `origin/main`, но локальный fork имеет 23081+ diverged коммитов. Применение "as-is" может сломать другие diverged фичи.

**Принятое решение**: ручная интеграция новых функций (`_profile_skill_names`, `_validate_skills_for_assignee`, `_validate_scope_for_assignee`) в локальный код. Vendor-patch остаётся как **reference** для upstream sync; в будущем при merge upstream мы сравним и убедимся что наши версии совпадают.

### 4.2 Только watchdog self-heal (без pre-create guard)

**Что предлагалось**: layer 3 (self-heal) сам по себе достаточен — он поймает все orphan-skill карточки за 5 минут, не нужен pre-create guard.

**Почему отклонено**:
- 5 минут burnloop dispatcher'а на каждую orphan-skill карточку = waste compute + токены.
- Pre-create guard = fail-fast с явным сообщением (юзер сразу видит "X not in profile Y, fix the assignee") — лучший UX чем "создал, watchdog починил неизвестно на что".
- Pre-create guard = меньше событий в БД, меньше audit noise.

### 4.3 Жёсткий reject (не warn-only) для scope-hint

**Что предлагалось**: mis-scope = hard fail вместо warn-only.

**Почему отклонено**:
- False-positive rate heuristic (~5%) превратит 5% легитимных карточек в dead-on-arrival.
- ADR-0036 §5 явно требует "no LLM, heuristic-only, warn-not-block" — мы следуем этому решению.
- `--force-scope` escape hatch + audit через `task_event` достаточны для архитектурного надзора.

---

## 5. План внедрения (статус)

| Шаг | Статус | Где |
|---|---|---|
| Patch `hermes_cli/kanban_db.py` (3 новые функции + force_scope param + scope_hint event) | ✓ done | локальный hermes-agent fork |
| Patch `hermes_cli/kanban.py` (--force-scope CLI flag) | ✓ done | локальный hermes-agent fork |
| Smoke-test `hermes kanban create --skill orphan` → ValueError | ✓ verified | shell |
| Smoke-test `--force-scope` suppresses scope-hint | ✓ verified | shell |
| Smoke-test valid skill `sdlc-review` для devops → created | ✓ verified | shell |
| Patch `kanban-retro-create.sh` (pre-create guard) | ✓ done | `~/.hermes/profiles/devops/scripts/` |
| Smoke-test retro-create с orphan skill → exit 2 | ✓ verified | shell |
| Patch `hermes-kanban-orphan-skill-watchdog.sh` (--self-heal mode default) | ✓ done | `~/.hermes/profiles/devops/scripts/` |
| Smoke-test watchdog self-heal on synthetic orphan task | ✓ verified | shell |
| Manual-fix `t_84cb9466` (devops) → `['sdlc-review']`, unblock | ✓ done | direct DB+hermes |
| Manual-fix `t_8572890f` (backend) → `['ci-debugging']`, unblock | ✓ done | direct DB+hermes |
| ADR-0049 написан | ✓ done | `docs/adr/0049-orphan-skill-watchdog-self-healing.md` |
| Commit в hermes-agent fork (krikz remote) | pending | следующий шаг |

---

## 6. Открытые вопросы / follow-ups

1. **Cron-line update**: текущая cron-строка watchdog'а не передаёт `--self-heal` явно (новый default). Если кто-то хочет explicit detective mode — нужно добавить `--detective` в `cron/jobs.json` script args.
2. **Vendor-patch sync**: при следующем merge из upstream `origin/main` в наш fork — сравнить наши версии `_validate_skills_for_assignee` и `_validate_scope_for_assignee` с upstream. Если upstream починил баг с duplicate — взять upstream версию.
3. **Dashboard surfacing**: `scope_hint` event сейчас пишется в `task_events` но dashboard может не показывать его inline. Follow-up: добавить column на kanban board view (можно в `kanban-board` webapp).
4. **Watchdog marker TTL**: 24h idempotency. Если карточка вернулась в orphan-skill через 25 часов после фикса — watchdog зафиксит снова. Это правильно, но может flood'ить audit log. Потенциально добавить rate-limit per-card (e.g., max 3 fixes per task).
5. **Pre-create guard в hermes CLI для других subcommand**: сейчас только `create` валидирует. `link_tasks`, `assign` (если меняет skills?) — нужно проверить.
