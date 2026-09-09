# ADR-0050: доставка repo-скиллов в профили воркеров + маппинг скилла по типу задачи

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-05 |
| Автор | Copilot по наказу товарища Шифу (05.09.2026) |
| Контекст | Два несвязанных контура скиллов: (1) интерактивные сессии — `.agents/skills/` (46 скиллов), (2) Hermes-воркеры по cron — скиллы живут только на хосте в `~/.hermes/profiles/*/skills/` и в репо их нет. `af_skill_for_profile()` (ретро t_b3476561) привязывает скилл **к роли** (backend/devops → `git-workflow`, tester/pr-reviewer → `sdlc-review`), а не к типу задачи: bug и feature у backend получают один и тот же скилл. |
| Затрагивает | (a) новый `scripts/agent_flow/sync-skills.sh`; (b) `scripts/agent_flow/lib_agent_flow_common.sh` — `af_skill_for_profile()`; (c) `scripts/agent_flow/agent-flow-triage.sh` — передача `$labels`; (d) `scripts/agent_flow/install.sh` — EXPECTED + best-effort вызов; (e) `scripts/agent_flow/tests/test_sync_skills.sh`, `test_af_skill_task_type.sh`; (f) `scripts/agent_flow/README.md`. **НЕ затрагивает** drift-detect, merge-gate, e2e-process — доставка скиллов ничего не блокирует и ничего не мержит. |
| Родители | ADR-0018 (культура честности — воркер обязан верифицировать, а не «сделано»), ADR-0023 (skill-discovery recursive — валидатор `_validate_skills_for_assignee` ходит рекурсивно и видит категорийные скиллы), ADR-0036 (mis-scope guard — скилл по типу задачи не должен подсовывать TDD на архитектурную задачу) |
| Связанные | `scripts/agent_flow/install.sh` (дрифт-контроль через EXPECTED), `vendor/hermes-agent-skill-validation.patch` (`_profile_skill_names` / `_validate_skills_for_assignee`), `docs/adr/0023-skill-discovery-recursive.md` |

## 1. Контекст и бизнес-проблема

### 1.1 Две несвязанные стороны скиллов

- **Интерактивные сессии** (Claude Code / Copilot / Cursor) читают `.agents/skills/`
  из репо: весь superpowers-пакет, `karpathy-guidelines`, `skill-creator`, проектные
  скиллы. Правка скилла + commit + merge → видна в сессиях.
- **Hermes-воркеры** (triage → kanban → worker → PR → e2e) получают скилл
  **детерминированно по профилю** из `~/.hermes/profiles/*/skills/`, а этот
  каталог наполняется только на хосте (`profile-create.sh`, ручная раскладка).
  Ничего из `.agents/skills/` в профили воркеров не синхронизируется.

Следствие: **любой скилл, добавленный в репо, улучшает только сессии Шифу**,
но не качество кода воркеров. Это ровно то, что болит в ADR-0049 §1.2
(дубли функций, `assert True`, TODO в merged-коде): воркеры не имеют
`verification-before-completion` и `systematic-debugging` в момент работы.

### 1.2 Скилл привязан к роли, а не к задаче

`af_skill_for_profile(assignee)` (ретро t_b3476561) закрыл краши карточек без
`--skill`, но оставил грубую привязку: `backend` и `devops` всегда получают
`git-workflow`, `tester`/`pr-reviewer` — `sdlc-review`. Багфикс и фича у одного
профиля неразличимы для воркера.

### 1.3 Что уже есть и почему этого мало

- `hermes-agent/tools/skills_sync.py` — синхронизирует скиллы, но это
  host-механизм, не привязанный к git-репо (SOT живёт на хосте, дрейфует).
- `install.sh` уже раскладывает **скрипты** hardlink-ами и контролирует их
  дрейф (EXPECTED + md5-verify). Для скиллов аналога нет.

## 2. Решение

Два взаимодополняющих изменения в контуре воркеров:

### 2.1 `sync-skills.sh` — доставка allowlist-скиллов в профили

Новый скрипт, по образцу `install.sh`:

- **SOT** — `<repo>/.agents/skills/<skill>/SKILL.md`.
- **Target** — `<profile>/skills/repo/<skill>/` (категория `repo/`, hardlink
  `cp -al`, fallback `cp -aL` при cross-device).
- **Allowlist** — `SKILL_SYNC_ALLOWLIST` (владелец списка — сам файл):
  `systematic-debugging`, `test-driven-development`, `codebase-design`,
  `verification-before-completion`, `agent-flow`.
- **Профили** — `SKILL_TARGET_PROFILES` (backend/devops/tester/pr-reviewer/
  architect/agent-flow/analyst), override `SKILL_SYNC_PROFILES`.
- **Верификация** — post-sync md5 по всем профилям × allowlist; расхождение → exit 3.
- **Идемпотентность** — `cmp -s` по SKILL.md → skip.
- **Запуск** — `install.sh` вызывает его best-effort (сбой доставки не валит
  install.sh), плюс ручной `bash sync-skills.sh [--dry-run]`.

Почему категория `repo/`, а не плоский `skills/<skill>/` или чужая `bundled/`:
(a) не коллизит с уже установленными скиллами профиля; (b) валидатор
`_validate_skills_for_assignee` (ADR-0023 §2.5) ходит рекурсивно через
`iter_skill_index_files` и видит `repo/<skill>/SKILL.md` так же, как runtime
skill-loader; (c) `_profile_skill_names` считает именем скилла имя родительской
директории SKILL.md — ровно `<skill>`.

### 2.2 `af_skill_for_profile(assignee, labels_csv)` — маппинг по типу задачи

Функция получает опциональный второй аргумент — CSV-метки issue. Приоритет:

1. **Тип задачи** (если label есть И скилл доставлен в профиль):
   - `bug` / `type:bug` → `systematic-debugging`
   - `type:functional` / `type:feature` / `feature` → `test-driven-development`
   - `type:refactor` / `type:tech-debt` / `type:stub` → `codebase-design`
   - `type:process` → `agent-flow`
2. **Роль** (fallback, backward-compat): прежний маппинг `git-workflow` /
   `sdlc-review` / `agent-flow-merge-gate` / `agent-flow-pipeline-ops` /
   `simplify-code`.
3. **fail-OPEN** — пусто, если ничего не установлено (поведение t_b3476561
   сохранено: карточка без `--skill`, а не fail-fast над process-скриптом).

`agent-flow-triage.sh` передаёт `"$labels"` вторым аргументом. Проверка
установленности дополнена категорией `repo/` (куда кладёт sync-skills.sh).

## 3. Инварианты

1. **SOT скиллов воркеров — git-репо** (`.agents/skills/`), как и SOT скриптов
   (`scripts/agent_flow/`). Хост-копии — производные, пересоздаются `sync-skills.sh`.
2. **Маппинг тип→скилл ссылается только на allowlist** `SKILL_SYNC_ALLOWLIST`.
   Иначе fallback на роль (fail-OPEN).
3. **Доставка не блокирует конвейер.** Сбой `sync-skills.sh` не валит `install.sh`;
   дрейф файла контролируется через EXPECTED + drift-detect.
4. **Mis-scope guard (ADR-0036) не обходится**: TDD-скилл для архитектурной
   задачи по-прежнему ловится `_validate_scope_for_assignee` на `kanban create`.

## 4. Проверка (raw-вывод в PR)

- `bash scripts/agent_flow/tests/test_sync_skills.sh` → `ALL TESTS PASSED`
  (доставка, dry-run без side-effect, идемпотентность, exit 2 при отсутствии source).
- `bash scripts/agent_flow/tests/test_af_skill_task_type.sh` → `PASS 13/13`
  (тип→скилл, роль-fallback, fail-OPEN, case-insensitive labels).
- `bash -n` на `sync-skills.sh`, `lib_agent_flow_common.sh`,
  `agent-flow-triage.sh`, `install.sh` — без ошибок.
- `bash scripts/agent_flow/tests/test_install_ensure_cleanup_cron.sh` →
  `ALL 3 TESTS PASSED` (install.sh edit не сломал извлечение cron-функций).
- `bash scripts/agent_flow/tests/test_triage_skill_inference.sh` — T4/T5/T6 PASS
  (извлечение `af_skill_for_profile` не сломалось; T1/T3 host-dependent).

## 5. Trade-offs

| Плюс | Минус / риск |
|---|---|
| Репо становится SOT и для воркеров, а не только для сессий Шифу | Ещё один слой доставки поверх `skills_sync.py` на хосте |
| Скилл по типу задачи → меньше «не туда» направленных воркеров | Категория `repo/` — новый layout, требует проверки на билдер-машине (`e2e_skill_validation.py`) |
| fail-OPEN сохранён — доставка не может уронить triage | `verification-before-completion` доставлен, но не прикрепляется вторым `--skill` автоматически (ждём подтверждения поддержки повторяемого `--skill` в hermes CLI) |

## 6. Открытые вопросы

- Подтвердить на билдер-машине, что `iter_skill_index_files` резолвит
  `skills/repo/<skill>/SKILL.md` (прогнать `python3 scripts/agent_flow/tests/e2e_skill_validation.py devops`).
- Решить, прикреплять ли `verification-before-completion` как второй `--skill`
  на каждую карточку (совпадает с ADR-0018) — зависит от поддержки
  повторяемого `--skill` в `hermes kanban create`.
- Отдельная доставка `code-review` / `to-tickets` / `resolving-merge-conflicts` /
  `ponytail` (скиллы mattpocock, не в этом PR).
