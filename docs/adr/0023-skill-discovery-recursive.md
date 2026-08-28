# ADR-0023: skill-discovery recursive — guard в `kanban-retro-create.sh` ошибочен, dispatcher уже recursive

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-08-22 |
| Автор | architect (Hermes Agent); доработки — товарищ Шифу (Copilot, 22.08) |
| Контекст | Ретро-карточка [товарищ Шифу#t_83cb72d4](https://github.com/krikz/rob_box_project) — «skill-flattening: воркеры падают 'Unknown skill(s)' хотя skills/ содержат скилл в подкатегории» |
| Затрагивает | `scripts/agent_flow/kanban-retro-create.sh` (guard удалён), `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` (recursive валидация), `hermes-agent/tools/skill_usage.py` (sot для `_find_skill_dir`) |
| Родители | ADR-0014 (issue-closure on merge), ADR-0022 (e2e-done gates), архитектурный SOT для hermes-agent skill discovery |
| Связанные | issue t_83cb72d4 (это), t_8c8c7c69 (оригинал "spike" — теперь под сомнением), t_28dcdaf0 (cobra crash), t_6c6c98fb (скилл чужого профиля), t_1ab37fa8 (vendor-патч) |

## 1. Контекст и бизнес-проблема

После двух инцидентов подряд (t_8c8c7c69 «spike» и t_28dcdaf0 «cobra», 19.08) `kanban-retro-create.sh` был пропатчен коммитом `6ceb43eb` чтобы pre-check'ить наличие `--skill` в профиле исполнителя перед созданием ретро-карточки. Guard считал, что dispatcher берёт скиллы **ТОЛЬКО из плоского** `profiles/<assignee>/skills/<SKILL>/SKILL.md`, и режет создание карточки exit 2 если находит скилл только в подкатегории.

**Доказано эмпирически, что dispatcher на самом деле recursive.** При попытке 22.08~14:05 создать ретро-карточку с `skill=sdlc-review` для devops — guard сразу отбил:

```
ERROR: skill 'sdlc-review' не найден в /home/builder/.hermes/profiles/devops/skills/sdlc-review/SKILL.md
       воркер упадёт 'Unknown skill(s): sdlc-review' если создать карточку
       доступные скиллы профиля 'devops':
         - hermes-themes
         - kanban-parallel-cherry-pick
       (профиль берёт скиллы ТОЛЬКО из плоского каталога,
        под-категории skills/<category>/<SKILL>/ НЕ видны)
```

При этом реально `sdlc-review` живёт в `profiles/devops/skills/devops/sdlc-review/SKILL.md` (с подкатегорией `devops`), и `hermes skills list | grep sdlc-review` показывает его как enabled.

**Следствие для бизнеса:** архитектор/девопс не могут создавать ретро-карточки с подкатегорийными скиллами через обёртку. Ретро-процесс надзора/падавана затыкается на первом же скилле из категории (а таких — большинство: `devops/`, `software-development/`, `rob-box/`, `bundled/`, ...).

## 2. Где находится SOT и почему guard ошибочен

### 2.1. Dispatcher в hermes-agent — recursive

Файл `/home/builder/.hermes/hermes-agent/tools/skill_usage.py`, функция `_find_skill_dir` (lines 1243-1261):

```python
def _find_skill_dir(skill_name: str) -> Optional[Path]:
    """Locate the directory for a skill by its frontmatter `name:` field.

    Handles both flat (~/.hermes/skills/<skill>/SKILL.md) and category-nested
    (~/.hermes/skills/<category>/<skill>/SKILL.md) layouts.
    """
    base = _skills_dir()
    if not base.exists():
        return None
    from agent.skill_utils import iter_skill_index_files
    for skill_md in iter_skill_index_files(base, "SKILL.md"):
        ...
        if _read_skill_name(skill_md, fallback=skill_md.parent.name) == skill_name:
            return skill_md.parent
```

Источник правды — `iter_skill_index_files` — обходит всё дерево `skills/` рекурсивно и матчит по `name:` из frontmatter. Это подтверждено в 30+ местах codebase: `tools/skills_tool.py`, `tools/skills_sync.py`, `hermes_cli/profile_distribution.py`, `hermes_cli/web_server.py`, `gateway/run.py`, и т.д. Все они используют `rglob("SKILL.md")`.

Документация в `tools/skills_tool.py:14-26` явно описывает категории как стандартный формат:

```
skills/
├── my-skill/
│   ├── SKILL.md           # Main instructions (required)
│   ├── references/
│   └── assets/
└── category/              # Category folder for organization
    └── another-skill/
        └── SKILL.md
```

### 2.2. CLI валидация — на стадии загрузки профиля

`cli.py:8172-8212` (`finalize_preloaded_skills`):

- Если ВСЕ запрошенные скиллы unknown → `ValueError("Unknown skill(s): ...")`
- Если хотя бы один загружен → graceful degradation с warning

Это **runtime-проверка после создания карточки и старта воркера**. То есть guard в `kanban-retro-create.sh` — это pre-emptive защита, которая **дублирует** логику dispatcher, но с неверной моделью рекурсии.

> Слоёв валидации на самом деле три: (1) плоский bash-guard (неверный, удаляется), (2) `_validate_skills_for_assignee` на `kanban create` (recursive, vendor-патч — см. §2.5), (3) runtime `finalize_preloaded_skills` (graceful-degrade). Первая версия ADR считала «реальным валидатором» только (3).

### 2.3. Реальная ошибка в t_8c8c7c69 / t_28dcdaf0 — другая

Скорее всего, воркеры в t_8c8c7c69 и t_28dcdaf0 упали НЕ из-за того, что dispatcher не видит подкатегории, а из-за:

1. **Краша tool-loop** воркера до того, как skill-preload завершился — `cli.py:8187` ждёт thread с timeout 120 сек, и если воркер по любой причине упал раньше (например, syntax error в skill content) → "Unknown skill(s)" не из-за отсутствия, а из-за недогрузки.
2. **Опечатки в имени** скилла (например, "spike" vs "Spike" — `_find_skill_dir` сравнивает case-sensitively).
3. **Skills dir исключения** через `is_excluded_skill_path()` — некоторые skill dirs могут быть явно исключены.
4. **Скилл чужого профиля** — воркер просит скилл, который принадлежит другому профилю (`t_6c6c98fb`: `assignee=architect` + `skills=[devops]`). Именно этот класс закрывает vendor-патч через `_validate_skills_for_assignee` (§2.5), а не «dispatcher не видит подкатегории».

Нужно ретроспективно достать stderr из тех сессий, чтобы понять root cause. Но **точно не "dispatcher не видит подкатегории"** — этот класс ошибок не воспроизводится.

### 2.4. SOUL.md архитектора — не упоминает скиллы

Проверено: `~/.hermes/profiles/architect/SOUL.md` (112 строк) НЕ содержит перечень skills. Утверждение ретро "SOUL.md говорит архитектору использовать 7 skill" — не подтверждается. SOUL.md описывает только роль + процесс. H2 в ретро — отвергнут.

### 2.5. Recursive-валидация уже реализована vendor-патчем — guard устарел

В репо уже есть `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch`, который добавляет в `hermes kanban create` (`hermes_cli/kanban_db.py::create_task`) функцию `_validate_skills_for_assignee`, идущую через `iter_skill_index_files` — тот же recursive (и symlink-following) вокер, что и runtime. Патч:

- применяется идемпотентно `scripts/agent_flow/install.sh` (`git apply --reverse --check` → уже применён; `git apply --check` → применяет);
- покрыт тестами `scripts/agent_flow/tests/e2e_skill_validation.py` (accept/reject на реальном профиле) и `scripts/agent_flow/tests/test_vendor_patch_apply.sh` (apply/reverse/idempotency).

**Следствие:** fail-fast при unknown skill остаётся на уровне `kanban create` даже после удаления bash-guard — и с лучшим сообщением (перечисляет installed skills и подсказывает чинить assignee, а не скилл). Это делает «Вариант A» безопасным: pre-check не теряется, неверный плоский guard заменяется на корректный recursive, который уже существует.

## 3. Инвариант

**Dispatcher — single source of truth для skill discovery.** Любой guard на стороне процесса должен либо:
- (A) быть удалён как дублирующая логика с неверной моделью, либо
- (B) использовать ту же recursive логику с тем же матчингом по `name:` field.

Плоский guard — карго-культ, закреплённый в фиксном коммите 19.08 без эмпирической проверки поведения dispatcher.

## 4. Решение

**Вариант A (рекомендуемый) — удалить guard.**

Заменить lines 90-128 в `scripts/agent_flow/kanban-retro-create.sh` пустым блоком (или удалить целиком). Dispatcher сам умеет валидировать и graceful-degrade (см. cli.py:8203-8210), guard — лишний слой.

**Trade-offs варианта A:**
- ✅ Убирает ложные отказы (которые воспроизводятся на каждой ретро-карточке с категорийным скиллом)
- ✅ Меньше кода в процессе, нет дублирования логики dispatcher
- ✅ Pre-check НЕ теряется: fail-fast остаётся на `kanban create` через vendor-патч `_validate_skills_for_assignee` (§2.5) — с лучшим сообщением об ошибке
- ⚠️ Зависимость: безопасность варианта A держится на том, что vendor-патч применён. Контролируется `install.sh` + `test_vendor_patch_apply.sh` + `e2e_skill_validation.py`.

**Вариант B — починить guard на recursive.**

Заменить `-maxdepth 2` на recursive `find` + матчинг по `name:` (или dirname как fallback), плюс список доступных скиллов через `_find_skill_dir`.

**Trade-offs варианта B:**
- ✅ Сохраняет pre-check
- ❌ Дублирует dispatcher логику → риск рассинхрона при апстрим-изменениях
- ❌ Больше кода в bash, который исторически ломается на edge cases

**Вариант C — переименовать все skills в плоские (отвергнут).**

Теряется организация по категориям, ломаются skill-hub привязки. Худший вариант.

## 5. Реализация в этом PR

- [x] Удалён guard (вариант A): блок «skill validation» в `scripts/agent_flow/kanban-retro-create.sh`, оставлен только dedup-блок.
- [x] Добавлен регресс-тест J: `--skill` с категорийным (не плоским) именем больше не режется exit 2 — карточка создаётся, скилл pass-through.
- [x] Подтверждено, что fail-fast при unknown skill сохраняется на `kanban create` (vendor-патч `_validate_skills_for_assignee`), поэтому pre-check не потерян.

**Верификация (raw-вывод в PR):**
- `bash scripts/agent_flow/tests/test_kanban_retro_create.sh` → все A–J PASS.
- На билдер-машине: `python3 scripts/agent_flow/tests/e2e_skill_validation.py devops` → accept/reject корректны (подтверждает, что патч применён).

## 6. Открытые вопросы

- Реальная причина t_8c8c7c69 (spike-fail) — нужно достать stderr/log той сессии. Если подтвердится что НЕ dispatcher-не-видит, можно отозвать guard-фикс как ошибочный. Доп. карточка для devops.
- Аналогичный guard в других обёртках? `find /home/builder/.hermes/scripts` — проверить, есть ли другие места с тем же cargo-cult pattern (дублирование логики между bash и hermes-agent).
- Проверяемость citations: ссылки на `/home/builder/.hermes/hermes-agent/*:NNN` в §2.1–2.3 не проверяемы из git-репо (исходник hermes-agent лежит на билдер-машине, не в репо). Тезис «recursive» подтверждён независимо через vendor-патч (§2.5) и его e2e-тесты. Номера строк проверить на билдер-машине при следующей возможности.

## 7. Связанные

- t_83cb72d4 (это)
- t_8c8c7c69 (done, 19.08 — spike fail, теперь под сомнением)
- t_28dcdaf0 (cobra crash по тому же скиллу)
- `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` — recursive валидация на `kanban create` (`_validate_skills_for_assignee`)
- `scripts/agent_flow/tests/e2e_skill_validation.py` — e2e accept/reject на реальном профиле
- `scripts/agent_flow/tests/test_vendor_patch_apply.sh` — apply/reverse/idempotency патча
- `scripts/agent_flow/tests/test_kanban_retro_create.sh` — регресс A–J обёртки
- `~/.hermes/scripts/kanban-retro-create.sh` — guard удалён (вариант A)
- `~/.hermes/hermes-agent/tools/skill_usage.py:1243-1261` — `_find_skill_dir` (recursive)
- `~/.hermes/hermes-agent/cli.py:8172-8212` — runtime-валидатор dispatcher
- `~/.hermes/hermes-agent/tools/skills_tool.py:14-26` — документация формата с категориями
- commit `6ceb43eb` — cargo-cult фикс в develop
