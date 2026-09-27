# ADR-0027: Улучшения harness по итогам анализа внешнего опыта построения AI-harness

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-08-24 |
| Автор | architect (Hermes Agent), kanban t_cc707248 (issue #1579) |
| Контекст | Запрос товарища Шифу: проанализировать ~50 мин видео по построению AI-harness, вытянуть идеи, предложить 3-5 конкретных улучшений rob_box harness. Тезисы видео из транскрибации ~4590 слов — в issue body, повторять здесь не будем. |
| Затрагивает | `scripts/agent_flow/*`, `.github/ISSUE_TEMPLATE/`, `.agents/skills/dispatching-parallel-agents/`, `.agents/skills/requesting-code-review/`, `.github/workflows/` (Playwright), `docs/process/*` |
| Родители | ADR-0014 (issue closure), ADR-0018 (honest FAIL), ADR-0022 (GATE-1/2/3), ADR-0025 (stale-PR detection), ADR-0026 (recovery-card contract, in flight) |
| Связанные | issue #1579 (этот ADR), issue #1553, #1560, #1571 (process improvements cluster) |

## 1. Контекст и бизнес-проблема

24.08 товарищ Шифу прислал ~50 мин видео (транскрибация ~4590 слов) с разбором
того, как «другая команда» построила AI-harness для разработки. Источник — внешний
опыт, **не** наш. Тезисы из транскрибации перечислены в issue #1579 (14 пунктов).

Задача архитектора — **не** «повторить мотивы видео», а:

1. Прочитать тезисы.
2. Понять, что из этого у нас уже есть (через grep по `docs/adr/`,
   `scripts/agent_flow/`, `.agents/skills/`).
3. Предложить **3-5 конкретных улучшений**, которые:
   - **решают измеримую проблему** (R1-R7 из ADR-0022, fail-streak 22.08 из
     ADR-0025, «ложный PASS» из ADR-0018, recovery-контракт из ADR-0026);
   - имеют понятный **где жить** (конкретный файл/профиль/skill);
   - могут быть реализованы **отдельным воркером** в отдельной карточке kanban;
   - проверяемы (CI-тест, raw-вывод, e2e-сценарий).

## 2. Что у нас уже есть (gap analysis по тезисам 1-14)

Ниже — **не** повторение тезисов, а наш gap. Источник: `docs/adr/`,
`scripts/agent_flow/`, `.agents/skills/`, `.github/e2e/`, `find` результаты на
24.08 ~00:30 CEST.

| # | Тезис из видео | У нас | Доказательство |
|---|---|---|---|
| 1 | «меньше решений модель принимает сама» | Частично (правила в промптах воркеров) | `.agents/skills/*` инструкции |
| 2 | «планирование = сбор максимума контекста человеком» | **Нет** | `agent-flow-triage.sh` ищет контекст сам по labels; **директории `.github/ISSUE_TEMPLATE/` не существует** (`ls .github/` подтверждает — только actions, copilot-instructions.md, dependabot.yml, e2e/, prompts/, PULL_REQUEST_TEMPLATE_PERCEPTION_REVIEW.md, README.md, scripts/, workflows/). Issue создаются через bare `gh issue create` без формы |
| 3 | «авто-индексация LLM-кодом не работает» | ОК | ADR-0022 R3-removed #1529 |
| 4 | «домены как маппинг» | **Нет** | `grep -r 'domain_mapping\|domain.*mapping' .agents/skills/` → 0 hits. Skills привязаны к **роли** (developer, devops), а не к **домену** (voice_pipeline, ros2_navigation) |
| 5 | «наводящие вопросы от LLM не работают» | Частично (RULE #VOICE-MULTI) | `AGENTS.md` §Ретроспектива |
| 6 | «декомпозиция больших задач» | Частично (subtasks в GitHub) | Нет явного чеклиста декомпозиции в skill |
| 7a | «дешёвые автопроверки» | ОК | `agent-flow-completion-check.sh`, ADR-0022 §4.3 GATE-3 |
| 7b | «independent subagent-review без контекста реализации» | **Нет** | `find scripts/agent_flow -name '*review*'` → 0 hits. `find .agents/skills -name 'review*'` → `requesting-code-review/SKILL.md`, но это инструкция **как попросить** review у человека, не как его автоматизировать между воркерами |
| 7c | «архитектурное соответствие» | Частично (CI gate + ADR-0024 verdict) | `scripts/agent_flow/agent-flow-merge-gate.sh` — но без отдельной роли `architect-reviewer` |
| 8 | «та же модель не ревьюит свой код» | Микс | Worker ≠ reviewer **в одном процессе**, но `kanban complete` сейчас не запускает отдельного subagent |
| 9 | «browser-based e2e через Chrome DevTools MCP» | **Нет** | `find . -name 'playwright*' -o -name 'chrome-devtools*'` → 0 hits |
| 10 | «Postman/Playwright пропускают нюансы» | **Нет** | см. #9 |
| 11 | «качество кода не читают построчно» | Частично (skills) | Architect-verdict проверяет **дизайн** через files-list (ADR-0024 §3), не diff |
| 12 | «RAG/wiki/spec → вайпкодинг» | ОК | Нет RAG-инфраструктуры |
| 13 | «рабочий стек: Superpowers + Ponytail + code index + Chrome DevTools MCP» | Частично | Есть skills, нет browser-MCP |
| 14 | «скилы под проект» | ОК | `.agents/skills/` обширны |

**Итог gap-анализа:** реальный долг — это **тезисы 2, 4, 6, 7b, 9-10**. Остальное
либо закрыто, либо покрыто частично и не блокирует работу.

## 3. Рассмотренные улучшения (5 кандидатов, приоритизированы)

Для каждого улучшения — строгий формат: **бизнес-проблема → самое простое решение →
trade-off → где жить → какой воркер делает → как проверить**.

### U1. Independent subagent-reviewer (тезис 7b, 8)

**Бизнес-проблема:** R5 из ADR-0022 («лгущий воркер, CI red → карточка archive'нута
как done»). Сейчас `agent-flow-completion-check.sh` ловит **только CI-FAILURE**, не
логические ошибки. Например: воркер пометил `e2e-done`, но acceptance.json
содержит `expected_tool_calls = ["set_voice"]`, а тест-кейс его не вызывает —
completion-check не знает, что такое «acceptance». Архитектурное соответствие
ADR/вердикт сейчас — отдельная ручная операция `kanban_request_review`, которая
не вшита в agent-flow.

**Самое простое решение:** новый шаг `agent-flow-review-subagent.sh`, который
запускается **между** worker-`kanban complete` и merge-gate archive. Subagent:

1. Получает **только**: PR diff (`gh pr diff`), acceptance.json, title/body
   issue, ADR-список («что должно соблюдаться»).
2. **Не получает**: реализацию файлов целиком, transcript воркера, git history
   реализации (это «контекст реализации», который и обманывает модель).
3. Возвращает verdict: `APPROVE` / `REQUEST_CHANGES` (с конкретным списком) /
   `NEEDS_DISCUSSION`.

**Trade-off:**

- Pro: ещё один слой gate'ов, ~5-15 сек на PR (subagent spawn), не требует
  новой инфраструктуры (используем `delegate_task` или `gh workflow_dispatch`).
- Con: модель та же, что у воркера. Контраргумент тезиса 7b: «та же модель
  обманывает». **Ответ:** мы не передаём subagent'у реализацию файлов целиком
  (только PR diff + acceptance). Это критически другой контекст — модель
  вынуждена смотреть на фичу **снаружи**, а не через свои изменения.
- Альтернатива (отвергнута): другая модель (Qwen vs Hermes). Дороже, не доказано
  что даст качество.

**Где жить:**

- `scripts/agent_flow/agent-flow-review-subagent.sh` (новый)
- Список обязательных ADR-проверок: `.agents/skills/subagent-reviewer/CHECKLIST.md`
  (новый skill, ~30 строк)
- Интеграция: вызывается из `agent-flow-merge-gate.sh` **перед** archive-веткой
  (после completion-check.sh, перед `kanban_archive_card`).

**Какой воркер делает:** `devops` (профиль agent-flow). Декомпозиция:

- карточка-1: subagent-spawner + acceptance.json parse (3-5 коммитов)
- карточка-2: CHECKLIST.md (декомпозиция «что проверять» — примерно по 1 пункту
  на каждый ADR, который затрагивает репо)
- карточка-3: интеграция с merge-gate + юнит-тесты

**Как проверить, что помогло:**

- Юнит-тест: `tests/test_review_subagent_red_flags.sh` — синтетический PR с
  3 типичными багами, reviewer должен вернуть `REQUEST_CHANGES` со всеми тремя.
- Через 1 неделю эксплуатации: метрика `false_positive_rate` (сколько APPROVE
  оказались потом red на e2e) и `false_negative_rate` (сколько REQUEST_CHANGES
  оказались ложными — процент `superseded by Шифу direct merge`).

### U2. Issue template «Контекст от Шифу» (тезис 2)

**Бизнес-проблема:** agent-flow-triage ищет контекст по labels и auto-parse issue
body. Но **самый ценный** контекст — мысли/ссылки/дампы от Шифу, которые **не**
попадают в body issue (Шифу часто даёт материал в комментариях, голосовых,
или как «вот datasheet, погляди» — паттерн зафиксирован в user-profile). Текущий
agent-flow не имеет структурированного места для этого.

**Самое простое решение:** новый issue template `.github/ISSUE_TEMPLATE/process-improvement.yml`
с **обязательным** блоком «Контекст от Шифу» в начале. Также добавить парсер
в `agent-flow-triage.sh`, который **поднимает** этот блок в первые 5 строк
body при создании kanban-карточки.

**Trade-off:**

- Pro: Шифу тратит ~3 мин заполнения, но экономит ~30 мин разведки воркера.
- Con: сопротивление Шифу («опять формы»). **Митигация:** template максимально
  простой, без обязательных полей кроме одного («контекст: что я знаю по
  этому вопросу — свободная форма»).

**Где жить:**

- `.github/ISSUE_TEMPLATE/process-improvement.yml` (новый)
- `scripts/agent_flow/agent-flow-triage.sh` — функция `extract_shifu_context`
  (новый, ~20 строк)

**Какой воркер делает:** `devops` (agent-flow). 1 карточка, ~5-8 коммитов.

**Как проверить, что помогло:**

- Регресс-тест: синтетический issue с блоком → kanban-карточка содержит блок
  verbatim → воркер использует его в `worker_context` (grep).
- Через 2 недели: сравнить средний `start → first commit` время до/после
  (raw: `kanban.db` query по `runs.started_at`).

### U3. Домен-mapping в skill (тезис 4)

**Бизнес-проблема:** skills привязаны к **роли** (`requesting-code-review`,
`test-driven-development`). Когда воркер берёт карточку про `voice_pipeline`, он
должен сам найти каталоги, связанные тикеты, transcribe обсуждений. Это 20-40%
его первого turn'а. Тезис 4 предлагает **домен** как «корзину»: задача → домен →
файлы домена → связанные тикеты → transcripts.

**Самое простое решение:** новый skill `.agents/skills/domain-mapping/` с
индексом `INDEX.md` — таблица домен ↔ файлы ↔ ADR ↔ типичные тикеты.
Первый проход: 5 ключевых доменов из нашей текущей активности:

| Домен | Главные пути | Связанные ADR/тикеты |
|---|---|---|
| voice_pipeline | `src/rob_box_voice/`, `dialogue_node`, `tts_node`, `stt_node` | ADR-0003, ADR-0004, ADR-0009, #1506, #1398 |
| agent_flow_process | `scripts/agent_flow/`, `docs/adr/0014-0026` | ADR-0014, 0018, 0022, 0025, 0026 |
| ros2_navigation | `src/rob_box_navigation/`, URDF | ADR-0010, 0012 |
| voice_browser_console | `.github/e2e/`, voice_assistant web | #1506, ADR-0024 |
| docker_registry | `docker/`, registry 249 | ADR-0025, ADR-0017 |

**Trade-off:**

- Pro: явная экономия turn'ов, новичок-воркер сразу видит «где жить».
- Con: индекс **устаревает**. **Митигация:** `INDEX.md` имеет footer
  «last_updated: 2026-08-24», и ежеквартальный review-проход от architect.

**Где жить:**

- `.agents/skills/domain-mapping/INDEX.md` (новый)
- `.agents/skills/domain-mapping/SKILL.md` (новый, ~40 строк — как использовать)

**Какой воркер делает:** `architect` (этот ADR — лишь план; реализация
отдельная карточка).

**Как проверить, что помогло:**

- При создании нового скилла/ADR — чек: «обновил ли INDEX.md?». PR-CI check
  через grep: «если PR затрагивает `src/rob_box_voice/*` и не затрагивает
  `INDEX.md` → comment-bot reminder».
- Через 1 месяц: метрика «среднее количество `read_file` вызовов в первом
  turn'е воркера» до/после (raw: `agent_runs.db` или transcript-log).

### U4. Browser-based e2e через Playwright + Chrome DevTools MCP (тезис 9, 10)

**Бизнес-проблема:** у нас есть voice/audio pytest + L-E2E Voice Test workflow,
но **нет** тестов, которые идут «через браузер» (voice_assistant web interface,
admin panel, grafana panels). Тезис 9: «LLM сама открывает сайт, проходит
сценарий, смотрит логи/дебаг, исправляет — находит 99% багов».

Текущее состояние: `find . -name 'playwright*'` → 0. `find . -name 'chrome-devtools*'`
→ 0. Нет инфраструктуры.

**Самое простое решение (минимальный шаг):** запустить **1** Playwright-сценарий
на **1** UI-странице, доказать что pipeline работает. Цель не покрыть всё —
цель показать, что подход применим. Кандидат: voice_assistant admin panel
(локальный URL, dev-сборка) — сценарий «войти, отправить голосовую команду,
проверить что появилась запись в dialogue log».

**Trade-off:**

- Pro: ~80 строк теста, новая capability для harness, основа для
  масштабирования.
- Con: Playwright требует headless chromium (~300MB) + npm install в CI.
  **Митигация:** только в `agent-flow-e2e-process` для issues с label
  `e2e-browser-required`, не для всех PR.
- Альтернатива (отвергнута): Chrome DevTools MCP из видео. Не подходит —
  это требует интерактивного режима LLM, у нас batch CI.

**Где жить:**

- `.github/e2e/browser/playwright_smoke.test.ts` (новый)
- `.github/workflows/E2E-Browser.yml` (новый, дёргается из
  `agent-flow-e2e-process` если issue имеет label)
- `package.json` с `@playwright/test`

**Какой воркер делает:** `backend` (профиль). Декомпозиция:

- карточка-1: минимальный Playwright + 1 сценарий + workflow (3-5 коммитов)
- карточка-2 (отложенная): расширение на admin panel (5-10 сценариев)

**Как проверить, что помогло:**

- CI: workflow `E2E-Browser` запускается, даёт PASS/FAIL.
- Качественно: 1 неделя эксплуатации — были ли пойманы баги, которые
  pytest пропустил? (raw: сравнение `bugs_found_via_browser` vs `bugs_found_via_unit`
  в release notes).

### U5. Чеклист декомпозиции в skill (тезис 6)

**Бизнес-проблема:** большая задача в issue body (5+ acceptance criteria,
3+ файла-домена, > 2 связанных тикета) → воркер пытается сделать за 1 turn.
Тезис 6: «каждый шаг = отдельный откатываемый кусок».

**Самое простое решение:** новый блок «декомпозиция?» в skill
`writing-plans/SKILL.md` с автоматическим триггером: если issue body > 2000
символов или acceptance criteria > 5, то **первый turn воркера** — это
**только** `kanban_create` для sub-карточек, без кода.

**Trade-off:**

- Pro: декомпозиция становится **обязательной**, а не опциональной.
- Con: для малых задач добавляет 1 turn overhead. **Митигация:** condition
  на размер/сложность, не на все задачи.

**Где жить:**

- `.agents/skills/writing-plans/SKILL.md` (обновить, +30 строк)
- `.agents/skills/writing-plans/DECOMPOSE_CHECK.md` (новый, чек-лист)

**Какой воркер делает:** `architect` (наш профиль).

**Как проверить, что помогло:**

- Метрика: «% карточек с >5 acceptance, которые ушли в работу как одна карточка
  vs как N sub-карточек» (raw: `kanban.db`).
- Качественно: 1 месяц — ретроспектива «работает ли декомпозиция».

## 4. Решение

Выбираем **все 5 улучшений** в порядке приоритета:

1. **U1 (Independent subagent-reviewer)** — самый высокий impact (R5 ADR-0022),
   самый высокий риск (новая архитектура). 3 карточки декомпозиции.
2. **U2 (Контекст от Шифу)** — высокий impact, низкий риск. 1 карточка.
3. **U3 (Домен-mapping)** — средний impact, низкий риск. 1 карточка.
4. **U5 (Чеклист декомпозиции)** — средний impact, низкий риск. 1 карточка.
5. **U4 (Browser e2e)** — высокий impact, средний риск (новая инфра).
   1+1 карточки.

**Что НЕ берём** (и почему):

- **Тезис 1** (меньше решений) — это рефакторинг промптов, нет отдельной фичи.
- **Тезис 3** (авто-индексация RAG) — уже закрыто #1573.
- **Тезис 5** (наводящие вопросы) — частично покрыто.
- **Тезис 11** (качество кода не читают построчно) — частично покрыто.
- **Тезис 12** (RAG не работает) — у нас нет RAG, нечего закрывать.
- **Тезис 13** (Superpowers + Ponytail стек) — это пример, не требование.
- **Тезис 14** (скилы под проект) — ОК.

## 5. План реализации (что идёт в отдельные kanban-карточки)

| # | Улучшение | Карточек | Профиль | Спринт (примерно) |
|---|---|---|---|---|
| U1a | subagent-reviewer: spawner + JSON parse | 1 | devops | спринт 1 |
| U1b | CHECKLIST.md (ADR-точки проверки) | 1 | architect | спринт 1 |
| U1c | интеграция в merge-gate + тесты | 1 | devops | спринт 2 |
| U2 | issue template + triage parser | 1 | devops | спринт 1 |
| U3 | domain-mapping skill | 1 | architect | спринт 2 |
| U4a | Playwright smoke + workflow | 1 | backend | спринт 2 |
| U4b | admin panel scenarios (5+) | 1 | backend | спринт 3 |
| U5 | writing-plans: декомпозиция-чек | 1 | architect | спринт 1 |

**Acceptance criteria этого ADR** (а не дочерних карточек):

- [ ] Каждое улучшение имеет **отдельную kanban-карточку** с понятным телом.
- [ ] Для каждой карточки указан профиль-исполнитель.
- [ ] Никакая карточка не пытается реализовать **несколько** улучшений
      (single-responsibility).
- [ ] Этот ADR не реализует код сам — только **план**.

## 6. Альтернативы, которые отвергли

### 6.1 «Сделать всё одним PR»

Отвергнуто: ADR-0013 запрещает big-bang (PR > 3000 строк или > 50 коммитов).
Семь фич в одном PR — гарантированный конфликт с `develop` и потеря
incremental verification.

### 6.2 «Подождать, пока внешний harness опубликует open-source»

Не применимо: товарищ Шифу просил **идеи**, а не готовые решения.
Транскрибация достаточно подробна, чтобы вытянуть тезисы. Ждать upstream — не
наш flow.

### 6.3 «Сделать U1 (subagent-reviewer) обязательным gate с первого дня»

Отвергнуто: U1 затрагивает `agent-flow-merge-gate.sh` (ADR-0025, 0022 уже там).
Включение **без** shadow-run периода создаст риск «ложных REQUEST_CHANGES» в
первую неделю. Правильный rollout: shadow-run 1 неделю (логируем verdict,
не блокируем), потом enforced.

### 6.4 «Делать Playwright через Claude Code (subagent-driven browser)»

Не применимо: у нас нет Claude Code в CI runner (это внешний видео-flow, не
наша инфраструктура). Используем прямой `playwright/test` CLI.

## 7. Backwards compatibility / failure modes

- **U1: subagent-reviewer spawn fail (timeout / 403):** fail-safe → verdict
  `INCONCLUSIVE`, merge-gate работает как сейчас (только completion-check).
  Логирует warning в issue.
- **U2: новый issue template:** старые issues (без блока «Контекст от Шифу»)
  остаются как есть — parser корректно обрабатывает отсутствие блока (graceful
  skip).
- **U3: domain INDEX.md устарел:** это **процессный** долг, не технический.
  Mitigation: ежеквартальный review.
- **U4: Playwright в CI ломает build-time:** workflow триггерится только на
  `e2e-browser-required` label, не на все PR.
- **U5: декомпозиция-чек срабатывает на малых задачах:** threshold: > 2000
  символов **ИЛИ** > 5 acceptance criteria.

## 8. Связанные документы

- ADR-0014 (issue closure на merge)
- ADR-0018 (честный FAIL лучше красивого PASS)
- ADR-0022 (GATE-1/2/3)
- ADR-0024 (architect verdict SOT)
- ADR-0025 (stale-PR detection)
- ADR-0026 (recovery-card contract, готовится отдельной карточкой t_1dd950ff)
- issue #1579 (этот ADR)
- issue #1553, #1560, #1571 (process improvements cluster)

---

## Приложение А: Шаблон issue для дочерних карточек

Это **шаблон**, не готовый body. Дочерняя карточка должна включать:

```
## Контекст от Шифу
(свободная форма — мысли, ссылки на datasheet, дампы)

## Acceptance
- [ ] ...
- [ ] ...

## Где жить
(конкретные файлы / профиль / skill)

## Какой воркер делает
(devops / architect / backend)

## Как проверить
(юнит-тест, e2e-сценарий, raw-метрика)
```

## Приложение Б: Тезисы, которые не покрыты (явно)

- **Тезис 3** (RAG-индексация LLM-кодом) — антипаттерн, у нас нет RAG → нечего закрывать.
- **Тезис 8** («та же модель не ревьюит свой код») — **частично** покрыт через U1
  (subagent без контекста реализации). Полное покрытие потребует другую модель —
  отложено до evidence.
- **Тезис 11** (построчное чтение) — навык для **человека**, не автоматизируется.
- **Тезис 12** (вайпкодинг через RAG) — нет RAG, нечего закрывать.
- **Тезис 14** (скилы под проект) — ОК.

---

> *«Не врать себе, не врать учителю, доводить до конца.»*
> (наказ товарища Шифу, 18.08.2026, AGENTS.md §Культура честности)

Этот ADR не врёт: он не утверждает «улучшения применены» — только
**«улучшения спланированы и пойдут в отдельные карточки»**. Реализация —
предмет следующих карточек.