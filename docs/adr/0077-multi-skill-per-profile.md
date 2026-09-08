# ADR-0077: multi-skill per profile — воркеры получают ОБЯЗАТЕЛЬНЫЙ набор skills (verification-before-completion + доменные), не один

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-08 |
| Автор | devops worker по issue #2160 (наблюдение товарища Шифу) |
| Контекст | Hermes-воркеры в Kanban получают от triage ровно один skill (`af_skill_for_profile` → строка). Из 199 done карточек только ~30% имеют skills, и у большинства — один. Карточки делаются без self-review: воркеры не вызывают `verification-before-completion`, `code-review`, `test-driven-development`. ADR-0050 закрыл доставку скиллов в профили и маппинг тип→primary skill, но не закрыл «1 skill на карточку». |
| Затрагивает | (a) `scripts/agent_flow/lib_agent_flow_common.sh` — новая функция `af_skills_for_profile` (multi); (b) `scripts/agent_flow/sync-skills.sh` — расширение `SKILL_SYNC_ALLOWLIST` (`verification-before-completion`, `code-review`, `requesting-code-review`); (c) `scripts/agent_flow/agent-flow-triage.sh` — проброс нескольких `--skill`; (d) `scripts/agent_flow/agent-flow-handoff.sh` — то же; (e) `scripts/agent_flow/tests/test_triage_skill_inference.sh` — новая секция T7_multi; (f) новый `scripts/agent_flow/tests/test_skills_multi_skill.sh` — regression. **НЕ затрагивает** dispatcher, merge-gate, e2e-process, hermes-agent: `--skill` уже repeatable в `hermes kanban create`, schema `tasks.skills` — JSON list. |
| Родители | ADR-0018 (культура честности — `verification-before-completion` ОБЯЗАН быть вызван), ADR-0023 (skill-discovery recursive), ADR-0050 (доставка + маппинг), ADR-0036 (mis-scope guard — primary skill по-прежнему проходит валидацию) |
| Связанные | issue #2160, `scripts/agent_flow/install.sh` (sync-skills best-effort), `.agents/skills/verification-before-completion/SKILL.md`, `.agents/skills/code-review/SKILL.md`, `.agents/skills/requesting-code-review/SKILL.md` |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдает Шифу

> «проверь что эти карточки используют скилы которые мы добавили»

В `.agents/skills/` лежит 58 skills, в т.ч. критичные для культуры:

- `verification-before-completion` — чеклист «pytest -v output, gh pr view + run_id, git log --stat» перед `kanban_complete`
- `code-review` — ревью PR diff через OpenAI agent
- `test-driven-development` — RED-GREEN-REFACTOR
- `systematic-debugging` — root cause до фикса
- `requesting-code-review` — comment в PR для Шифу
- `receiving-code-review` — как читать фидбек Шифу

Из 199 done карточек в Kanban **НИ ОДНА** не имеет явных skills в body. Малый процент (~30% в БД) вообще получает `--skill` (один). Следствие: воркеры **выполняют задачу**, но **не делают self-review** перед `kanban_complete`. Нарушается ADR-0018 (честный FAIL).

### 1.2 Почему ADR-0050 не закрыл проблему

ADR-0050 ввёл:

- `sync-skills.sh` — доставка allowlist в профили
- `af_skill_for_profile(assignee, labels)` — primary skill по типу задачи

Но:

- `af_skill_for_profile` возвращает **строку** (один skill)
- `agent-flow-triage.sh` пробрасывает **один** `--skill "$skill_for_card"`
- `verification-before-completion` есть в `SKILL_SYNC_ALLOWLIST` (доставлен в профили), но **не вызывается** воркером — он его просто не знает, потому что triage его не передал

`hermes kanban create --skill` уже принимает repeatable флаг. `tasks.skills` в БД — JSON list (длины 1, 2, 3 встречаются в существующих карточках). Контракт позволяет multi-skill, конвейер — нет.

### 1.3 Что должно быть

Каждая новая карточка должна приходить к воркеру с **набором** skills:

1. **`verification-before-completion`** — ВСЕГДА. Воркер ОБЯЗАН вызвать перед `kanban_complete` (чеклист ADR-0018).
2. **`code-review`** — для карточек, которые создают PR (assignee ∈ {backend, developer, tester, devops}). Не для agent-flow / analyst / architect-only карточек.
3. **Доменный primary skill** по типу задачи (bug → `systematic-debugging`, feature → `test-driven-development`, refactor → `codebase-design`, process → `agent-flow`) — как раньше.
4. **Ролевой fallback** — `git-workflow` / `sdlc-review` / `agent-flow-merge-gate` / `agent-flow-pipeline-ops` / `code-review` (для pr-reviewer) — как раньше.

Дубликаты убираются; проверяется, что skill реально установлен в профиле (fail-OPEN если нет — карточка создаётся с тем, что есть).

## 2. Решение

### 2.1 `af_skills_for_profile <assignee> [labels_csv] [creates_pr=auto]`

Новая функция рядом со старой `af_skill_for_profile`. Контракт:

- `$1` = assignee (profile id)
- `$2` = CSV меток issue (опционально)
- `$3` = `"pr"` / `""` — explicit PR-флаг (auto-detect: если есть `agent:*` assignee из {backend, developer, tester, devops} → "pr")
- `stdout` = skills, **разделённые newline** (для удобства bash-array), дедуплицированные, **включая обязательный `verification-before-completion`** первым
- `exit` = 0 всегда (fail-OPEN)

Пример вывода для `af_skills_for_profile backend bug,agent:backend`:
```
verification-before-completion
systematic-debugging
code-review
```
(3 skills, порядок: обязательный → task → role/PR-required)

Пример для `af_skills_for_profile architect type:process`:
```
verification-before-completion
agent-flow
```
(2 skills, role-кандидат `agent-flow-pipeline-ops` отброшен — task-кандидат `agent-flow` приоритетнее)

Пример для `af_skills_for_profile pr-reviewer`:
```
verification-before-completion
code-review
```
(роль pr-reviewer → `code-review`, не дублируется)

### 2.2 Что меняется в `agent-flow-triage.sh`

Старый код:
```bash
skill_for_card="$(af_skill_for_profile "$role" "$labels")"
skill_args=()
if [ -n "$skill_for_card" ]; then
    skill_args=(--skill "$skill_for_card")
fi
```

Новый код:
```bash
mapfile -t skills_for_card < <(af_skills_for_profile "$role" "$labels" "")
skill_args=()
for s in "${skills_for_card[@]}"; do
    [ -n "$s" ] && skill_args+=(--skill "$s")
done
log "  skill-inference: role=${role} labels=${labels} -> skills=[${skills_for_card[*]:-}]"
```

Тот же приём в `agent-flow-handoff.sh` для children.

### 2.3 Что меняется в `sync-skills.sh`

`SKILL_SYNC_ALLOWLIST` расширяется обязательными:

```bash
SKILL_SYNC_ALLOWLIST=(
    systematic-debugging
    test-driven-development
    codebase-design
    verification-before-completion   # NEW: mandatory for all workers
    agent-flow
    code-review                      # NEW: pr-reviewer + backend/devops PR-карточки
    requesting-code-review           # NEW: для PR workflow воркеров
    to-tickets
    resolving-merge-conflicts
    ponytail
)
```

Все три новых skill'а уже лежат в `.agents/skills/` (проверено `ls .agents/skills/`).

### 2.4 Что НЕ меняется

- `af_skill_for_profile` (single-skill helper) — сохранён для backward-compat и теста `test_triage_skill_inference.sh` (T1..T6). Single helper остаётся первичным skill-кандидатом (внутренний use в `af_skills_for_profile`).
- dispatcher / merge-gate / e2e-process — они читают `tasks.skills` (JSON list), не их дело сколько skills внутри.
- ADR-0036 mis-scope guard: первичный skill по-прежнему проверяется в `_validate_scope_for_assignee` (hermes-agent). Дополнительные skills (`verification-before-completion`, `code-review`) mis-scope не нарушают.

## 3. Инварианты

1. **Первая позиция в списке — `verification-before-completion`** (если он есть в профиле). Это гарантирует, что воркер видит чеклист ADR-0018 в контексте.
2. **`code-review` добавляется ТОЛЬКО если профиль может породить PR** (assignee ∈ {backend, developer, tester, devops, pr-reviewer}). Архитектор / analyst / agent-flow — не получают (у них нет кода для ревью).
3. **Дедупликация**: если primary skill уже совпадает с обязательным (например, pr-reviewer → `code-review`), дубликат убирается.
4. **fail-OPEN**: если ни один skill не найден в профиле (например, профиль без `skills/repo/verification-before-completion/`), функция возвращает пустой массив, как раньше. Карточка создаётся без `--skill`, но **WARNING пишется в task_events** через `kanban_comment` post-create.
5. **Обратная совместимость**: `af_skill_for_profile` остаётся как был. Новый код использует `af_skills_for_profile` явно.

## 4. Альтернативы (отвергнутые)

- **A) Передавать все скиллы профиля (`_profile_skill_names`) без фильтра.** Плохо: 50+ skills раздувают контекст воркера, замедляют LLM, не таргетированы на задачу.
- **B) Передавать только `verification-before-completion`.** Хуже: теряем доменный signal (TDD vs debugging vs codebase-design).
- **C) Hard-code список в `install.sh`.** Хуже: настройка skills ≠ contract между triage и профилем; ADR-0050 уже сделал sync-skills.sh SOT.
- **D) LLM-решает, какие skills брать.** Плохо: non-deterministic, дороже по токенам, нарушает ADR-0050.

## 5. План раскатки

1. **lib_agent_flow_common.sh** — добавить `af_skills_for_profile` рядом со старой.
2. **sync-skills.sh** — расширить `SKILL_SYNC_ALLOWLIST` 3-мя скиллами.
3. **agent-flow-triage.sh** — переключить на multi-skill.
4. **agent-flow-handoff.sh** — то же.
5. **Тесты**:
   - `test_triage_skill_inference.sh` — сохранить T1..T6 для single helper, добавить T7_multi_skill (multi возвращает ≥2 skills, обязательный первый, дедупликация работает).
   - Новый `test_skills_multi_skill.sh` — unit на `af_skills_for_profile` для 6 профилей × 3 набора меток.
6. **Прогон `bash scripts/agent_flow/tests/test_triage_skill_inference.sh`** + `test_skills_multi_skill.sh`.
7. **smoke**: создать 1 test card через `hermes kanban create` с multi-skill, проверить, что в `tasks.skills` — JSON list длины ≥2.

После merge: следующая партия карточек в Kanban автоматически получает multi-skill (через cron-tick triage).

## 6. Метрики (как поймём, что помогло)

- Доля новых карточек с `skills` JSON длины ≥2: **0% → ≥90%** в течение 24ч после merge.
- Доля карточек с `verification-before-completion` в skills: **0% → 100%** для всех профилей с этим skill.
- Re-rate `e2e-done` / `kanban-done` не ухудшается (новый skill замедляет воркера на 30-60с на карточку, что приемлемо).