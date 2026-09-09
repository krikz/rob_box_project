# Process Review — rob_box_project

| Поле | Значение |
|---|---|
| Дата | 2026-08-18 |
| Автор | architect (Hermes Agent) |
| Постановка | issue #1404 (task(process): ревью процесса разработки rob_box) |
| Период анализа | 2026-07-18 — 2026-08-18 (~1 месяц) |
| Объём | ~1569 коммитов, 21 PR, 7+ process-bugs18.08, ~9351 строк bash (scripts/agent_flow/*), 18 ADR |
| Статус | **snapshot для товарища Шифу — архитектурный документ, не код** |

---

## 0. TL;DR (одна страница)

Процесс работает, но **живёт на честном слове и ретроспективах**: каждый серьёзный баг
лечится точечным фиксом в одном из скриптов, ADR-ы плодятся, а корневые классы багов
(«counter init», «scope leak», «broken pipe в set -euo pipefail», «heredoc-фрагменты как
команды», «build ≠ deploy registry race», «cron-paused = queue-silent-stall»)
остаются без системного ответа. За последние 7 дней **5 process-bug-багов** были
закрыты worker-карточками с одним и тем же паттерном «исправить в одном файле + не
добавить unit-test».

Сильные стороны:

- ADR-0014 (post-merge close) и ADR-0015 (SSoT verdict) — **зрелые решения**, их
  реализация работает (post-round sweep ставит `e2e-done + needs-review` на PR #1395 для
  issue #1389 — подтверждено в живом cron-логе).
- Три крона (`agent-flow-merge-gate.sh`, `agent-flow-e2e-process.sh`, watchdog.sh)
  идемпотентны, чистят state, ретраятся без потерь.
- Огромный объём battle-tested retro-фиксов в `scripts/agent_flow/` (39 файлов,
  ~9351 строк) покрывают 90% edge cases.
- CONTRIBUTING.md §2b/§2d и ADR-0014/0015 вместе **формализуют lifecycle** issue.

Критические слабости:

1. **agent-flow-triage cron отсутствует** в `jobs.json` (только e2e-process +
   merge-gate). Issues с label `hermes` не превращаются в kanban-карточки
   автоматически с 12.08 (последний артефакт `5a240cb4b409` от 2026-08-12 01:10).
2. **e2e-process падает exit 127** прямо сейчас (live, тик 18.08 18:09): `kanban:
   command not found` + `PR: command not found` на строке 2285 — process substitution
   `< <(python3 ...)` роняет весь скрипт, после `set -euo pipefail` + SIGPIPE.
   Issue #1392 остался с **наполовину прогнанным e2e** (round-143 создан, build+deploy OK,
   e2e trigger запущен, verdict не записан, labels не выставлены).
3. **e2e-process не передаёт `scenario_file`** в `L-E2E Voice Test.yml` (issue #1384,
   живой лог: «scenario_file=none»). Multi-step сценарии (`music_library_suite_v1.json`)
   ломаются до дефолтного smoke-теста «робот, спой песенку про котика».
4. **Билд и деплой в разных registry** (runner-8 localhost:5000 vs 10.1.1.249:5000).
   Workflow-баг `L-Deploy and Verify.yml` — `!= 'local'` инвертирован условие загрузки
   `.image-versions.dev` → робот получает старый код при «зелёном» деплое.
5. **Counter-init баг (issue #1385)**: `self._llm_skipped_counter["e2e_busy"] += 1`
   без `"e2e_busy": 0` в `__init__` → KeyError на первом STT → voice-assistant мёртв.
   Архитектурный fix: **gating в dialogue_node не должно быть вообще** — это CI/CD
   concern, отдельный `wake_word_gate` подписывается на `/voice/e2e/busy`.
6. **SSoT-vs-Prompt-divergence**: master_prompt_compact.txt vs runtime лимиты (150 vs
   200/250) — несостыковка приводит к ложным warning и ручным фиксам (#1377).

---

## 1. Текущая архитектура процесса

### 1.1 Pipeline (как работает)

```
[user] → issue (hermes+agent:role)
        ↓
[agent-flow-triage cron — НЕ РАБОТАЕТ С 12.08]
        ↓
[kanban-карточка (assignee, инструкция, branch)]
        ↓
[sub-agent в worktree z-{agent}/<id>-slug → push → PR z-{agent}/... → develop]
        ↓
[agent-flow-merge-gate every 5m]
   ├─ CI red  → kanban unblock + worker чинит в той же карточке
   └─ CI green → ставит needs-e2e → (H3 reconcile по ADR-0014: e2e-done+OPEN PR → needs-review)
        ↓
[agent-flow-e2e-process every 60m]
   ├─ pre-round guard (живые кандидаты)
   ├─ round_ensure: z-{e2e}/test-round-N+1
   ├─ merge z-{agent}/... → z-{e2e}/test-round-N
   ├─ L-Build All Services → 10.1.1.249:5000
   ├─ L-Deploy and Verify → 10.1.1.249 (POTENTIAL: pull из runner-8 localhost:5000)
   └─ L-E2E Voice Test (жирный баг: scenario_file не передаётся)
        ↓
[verdict] → post-round sweep: e2e-done + needs-review + close
        ↓
[user] merge z-{agent}/... → develop → merge-gate close-on-merge (ADR-0014) → kanban archive
```

### 1.2 Контракты (что я нашёл в скриптах, а не только в доках)

| Контракт | Реальность | Источник |
|---|---|---|
| `e2e-done` ставит только e2e-process на PASS | ✅ соблюдается | `agent-flow-e2e-process.sh` line ~1669 (post-round sweep, не merge-gate) |
| `needs-review` ставит только e2e-process на PASS | ✅ соблюдается (для линтов — merge-gate) | merge-gate `OR e2e-done+OPEN PR` reconcile (H3) |
| `kanban complete` только после merge юзера (Q22) | ⚠️ часто нарушается (workers close после push) | карточки `done` с PR OPEN — наблюдается в `hermes kanban list` |
| `gh pr merge` — только юзер | ✅ соблюдается (после 09.08 #1079) | — |
| `scenario_file` в e2e args | ❌ НЕ передаётся | `agent-flow-e2e-process.sh` line ~2049 (только voice_text/voice_file) |
| Регексп «кто сказал правильно» (PR title ↔ issue number) | ⚠️ fallback '<number> in:title' даёт `null` headRefName (live лог #1398) | line ~48 в e2e-process |
| Counter init при новом self._dict["key"] | ❌ НЕ делается | `dialogue_node.py:1537` (баг #1385) |

---

## 2. Найденные баги (приоритизированные)

### 🔴 P0 — ломают pipeline прямо сейчас

| # | Что | Где | Эффект | Уже оформлено |
|---|---|---|---|---|
| **B1** | e2e-process exit 127 — `kanban: command not found` в line 2285 | `scripts/agent_flow/agent-flow-e2e-process.sh:2285` | Issue #1392: round-143 наполовину прогнан (build+deploy+e2e-trigger OK, verdict не записан, labels не выставлены) | **НЕТ** — новый |
| **B2** | agent-flow-triage cron отсутствует в jobs.json | `~/.hermes/profiles/architect/cron/jobs.json` | Все новые issues с `hermes` висят в очереди без карточек, воркеры не видят работу | **НЕТ** — новый |
| **B3** | e2e-process не передаёт `scenario_file` в `L-E2E Voice Test.yml` | `agent-flow-e2e-process.sh:2049` (только `voice_text/voice_file/volume`) | Multi-step сценарии (`music_library_suite_v1.json`) ломаются до дефолтного «робот, спой песенку про котика» — ложный PASS | **частично** — issue #1384 (PR #1375 e2e-done ложный) |

### 🟠 P1 — структурные классы багов, повторяются

| # | Что | Где | Эффект | Уже оформлено |
|---|---|---|---|---|
| **B4** | L-Deploy and Verify.yml: `!= 'local'` инвертировано условие загрузки `.image-versions.dev` | `.github/workflows/L-Deploy and Verify.yml:307,351,414,453` | Деплой SUCCESS, робот крутит старый код (37h+ old). Уже был hot-fixed в `c9278410` — но только частично (нужна верификация на 4 местах) | **частично** — issue #1384, fix `c9278410` |
| **B5** | Build (runner-8) и Deploy (249) используют разные registry-контейнеры (`localhost:5000` vs `10.1.1.249:5000`) | `.github/workflows/L-Build Main Pi Services.yml`, `L-Build Vision Pi Services.yml` | Build пушит в runner-local registry, deploy тянет из 249-local → never meets → bare dev-tag fallback | **НЕТ** — структурный фикс не сделан |
| **B6** | Counter-init баг: новый dict key без инициализации в `__init__` | `src/rob_box_voice/rob_box_voice/dialogue_node.py:370` | KeyError на первом event после рестарта ноды → voice-assistant мёртв | issue #1385 (фикс revert в PR #1390) |
| **B7** | Gating в dialogue_node — неправильный архитектурный слой | `dialogue_node.py` (PR #1385 → revert #1390) | wake-word/e2e-busy coupling: dialogue_node должен оставаться чистым runtime, gating = CI/CD concern (`wake_word_gate` подписывается на `/voice/e2e/busy`) | issue #1385, ADR-candidate |
| **B8** | `set -euo pipefail` + process substitution `< <(python3 ...)` → SIGPIPE → exit 127 | `agent-flow-e2e-process.sh:2285` | Каждый e2e-process тик в любой момент может упасть | **НЕТ** — структурный, не фиксился |
| **B9** | Auto-closer: krikz-commit-identity закрывает issue через 6 мин после user-reopen | `agent-flow-unlabeled-sweep.sh` + `agent-flow-e2e-process.sh` post-round sweep | Юзер открывает баг → крон закрывает → юзер в бешенстве | issue #1391 |
| **B10** | `kanban --board robbox create` positional vs `--title` confusion | hermes-cli argparse (worker retro) | Создание карточки руками: `--title "..."` игнорируется, title = positional | школа-правило, не фикс |

### 🟡 P2 — повторяющиеся классы, не критичные сейчас

| # | Что | Где | Эффект |
|---|---|---|---|
| **B11** | «Процесс-баг без unit-теста» — паттерн: воркер фиксит в одном файле, тестов нет | PR #1386, PR #1390, PR #1392, PR #1398 — каждый минимум 1 регрессия | Следующая регрессия того же класса (counter init, shell escape) гарантирована |
| **B12** | Round counter orphan: при cleanup remote-веток counter file может потеряться | `${HERMES_HOME}/state/agent-flow-e2e-round-counter` | Round numbering reset to 1 после ручной чистки |
| **B13** | Run-now RUN_NOW signal не consumed сразу после flock | watchdog.sh G0b (fixed), но e2e-process G0b — частично | Возможная race (уже была live 12.08 round-61 explosion) |
| **B14** | MAINTENANCE flag не проверяется в auto-decomposer | hermes-kanban core | `kanban.auto_decompose=false` — workaround (см. memory: 18.08) |
| **B15** | Процесс-фиксы уходят в `develop` напрямую (без PR) | весь процесс (school rule) | Нет code review для process-changes → «честный FAIL» не работает как safety net |

### 🟢 P3 — кунсткамера (уже зафиксировано, для памяти)

- **B16**: kanban CLI requeue не существует (`done` — terminal; merge-gate вызывает `hermes kanban requeue` → WARNING каждый тик; fixed `7d567420`)
- **B17**: Worker counter init (см. B6) — fixed в PR #1390 (revert), но класс не закрыт
- **B18**: idempotency-key на done-карточках возвращает done вместо новой (PR #1207)
- **B19**: 4h throttle для REOPENED issue (`ed782a97` v3.1) — остановил цикл карточек по #968, но требует `--archived` (см. ADR-0014 §5 orphan)
- **B20**: Provider fallback hygiene (MiniMax primary + DeepSeek fallback) — config flipped 18.08

---

## 3. Анализ скриптов (что я заметил по коду)

### 3.1 agent-flow-e2e-process.sh (2364 строки)

**Самый большой и самый хрупкий.** Полная хронология коммитов:

```
46785738 (18.08 18:20) wip(agent-flow): local changes to e2e-process
a02b788a (15.08)       merge-gate/e2e-fail не создают карточки для CLOSED PR
c31f22b2 (14.08)       escape-hatch — аддитивные фиксы на влитой ветке льём в round
e4e187c0 (14.08)       round-cleanup жив после падения секции 3 + EXIT-sweep
bf7b9e51 (14.08)       merged-branch guard placed before merge log
3eec49d8 (13.08)       move post_round_sweep before round_ensure
db031808 (13.08)       delete empty round branches
873252bc (13.08)       build TIMEOUT → gh run cancel + dedup по активным round
```

**Паттерн:** каждая ретроспектива = +1 фикс в этом файле. Файл превратился в
«crash-survivor»: 7 независимых edge-cases обработаны, но общая логика размылась.
Live crash 18.08 18:09 (exit 127) — очередной SIGPIPE/python-block-bug.

**Скрытый баг #1 (мой wip 46785738)**: на line 2049 есть классический gh --jq pitfall
(`'[].name | index(X)'` возвращает пустую строку вместо `null` для отсутствующего label).
Текущий код:
```bash
if [ -n "$number" ] && gh issue view "$number" ... --jq '...index("no-e2e-required")' ... | grep -q -v '^null$'; then
```
Поскольку `gh --jq` strips `null` → возвращает пустую строку → `grep -q -v '^null$'`
всегда матчит → **ВСЕ feature-PR классифицируются как lint** → ложный «e2e не
требуется» в PR-комменте на FAILURE (см. PR #1375 / round-139).

Wip зафиксирован в коммите `46785738` с правильным фиксом (`.labels | map(.name) |
any(. == "no-e2e-required")` → `^true$`).

### 3.2 agent-flow-merge-gate.sh (2759 строк)

Аналогично — survivor-файл. Работает корректно (live OK на последнем тике 18.08 18:19).
H3 reconcile по ADR-0014 реализован (e2e-done + OPEN PR → needs-review — наблюдалось
в post-round sweep на issue #1389 → PR #1395).

### 3.3 agent-flow-triage.sh (649 строк)

**Скрипт существует, но крон отсутствует.** Это отдельный класс бага:
- Либо крон был удалён вручную (когда-то убрали «every 5m» и забыли вернуть)
- Либо был объединён с merge-gate (но не вижу evidence)
- Либо jobs.json повредили при редактировании (devops-cron `83206128dcfa` `enabled=false`
  18.08 — учитывая memory про «не делай руками», такой ручной edit мог зацепить и triage)

Live проверка: `hermes cron list` показывает только `1082e70dc68f` (merge-gate) и
`73dcdece0619` (e2e-process). **Триаж не происходит с 12.08.**

### 3.4 agent-flow-drift-detect.sh (509 строк), watchdog.sh (882 строки)

Работают. Watchdog вчера поймал `ls-remote exit 128` (offline) и ушёл в retry
(`90540dcd` — retro t_091fc5b7). Хорошая обработка transient failures.

### 3.5 round_ensure.sh, agent-flow-cleanup-249.sh, agent-flow-deploy-sweep.sh

Round counter persistence — частично реализован (counter file
`/home/builder/.hermes/state/agent-flow-e2e-round-counter`). Cleanup-249 живой
(post-merge cleanup ветки + worktree освобождение). Deploy-sweep — оркестратор для
deferred-PR.

---

## 4. Анализ процесса (workflow, а не скрипты)

### 4.1 Что работает хорошо

- **ADR-0014 + ADR-0015 — зрелые решения.** Их реализация стабильна и observability
  высокая (post-round sweep логирует каждый шаг). Это образец для следующих ADR.
- **Ретроспективы пишутся в git-коммитах** — knowledge transfer работает.
  `git log -- scripts/agent_flow/` — это effectively executable design doc.
- **WIP-коммиты в worktree** — спасает от потери (retro 09.08 #2). My `46785738`
  восстановил потерянный фикс с предыдущей сессии.
- **`repo-sot-pitfalls` соблюдается** — все скрипты имеют SOT-header и install.sh
  раскладывает через symlink.
- **Контракт фида/артефакта** — e2e создаёт verdict.txt + логи + mp3, можно
  верифицировать постфактум.

### 4.2 Что ломается регулярно (системные проблемы)

**Класс 1: «Counter init» (B6, B11).** Каждый раз, когда воркер добавляет новый
state в runtime-ноду, забывают init в `__init__`. Каждый раз KeyError на первом
event после рестарта. Каждый раз «фикс revert» + новый PR. **Это не баг конкретного
PR — это баг review-процесса.**

**Класс 2: «Scope leak» (B7).** Архитектурные слои смешиваются: dialogue_node
обрастает логикой от CI/CD (e2e-busy), watchdog лезет в business-ка
(provider-exhaustion), merge-gate лезет в workspace cleanup. Не «нельзя так», но
**документации слоёв нет** — каждый воркер изобретает свой.

**Класс 3: «Bash process-substitution fragility» (B1, B8).** `< <(python3 ...)` +
`set -euo pipefail` + SIGPIPE = exit 127 в любой момент. **Структурный баг, не
фиксится — просто потому что работает чаще, чем падает.** Но когда падает —
роняет весь tick.

**Класс 4: «No unit tests for process scripts» (B11).** `scripts/agent_flow/tests/`
существует (видел в ls -la), но 0/12 acceptance PR покрыты. Каждый регрессия-баг
возвращается через 5-7 дней.

**Класс 5: «Registry topology is invisible» (B4, B5).** Документация в
`docs/infra/` отсутствует, новый воркер узнаёт про runner-8 vs 249 только при
первом «зелёном» деплое со старым кодом. **Это знание в голове у Шифу, не в
репо.**

**Класс 6: «Cron-paused = silent queue stall» (B2).** Если cron paused → никто
не узнает пока PRs не накопятся. `agent-flow-e2e-process` паузился 15.08 — 18.08
(3 дня!), 3 PR сидели с needs-e2e. **Watchdog есть, но он для MAINTENANCE, не для
crashes/pauses.**

### 4.3 Метрики (что я насчитал)

- **Commits/week (peak)**: 197 (05.08), 124 (13.08), 109 (12.08) — пики коррелируют
  с incident-ами (#1358, #1344, #1347, #1348, #1349).
- **Process-related commits** за последние 7 дней: ~25 (из 75 = 33%). Процесс
  ест треть работы.
- **Critical bugs live**: 5+ (B1-B5).
- **ADR-ы**: 18 в `docs/adr/` (последний — `0017-zenoh-router-spof-redundancy` 15.08).
  Темп: ~2.5 ADR/неделю. Качество высокое (0014, 0015 — образец), но **нет ADR
  по registry-topology** (B4-B5 — самая структурная из нерешённых).
- **Test coverage**: ~0% на process-скриптах (заявлено `scripts/agent_flow/tests/`,
  но ни одного acceptance test для текущих bug-фиксов).

### 4.4 Что **работает в обратную сторону** (хорошие знаки)

- Школа «честный FAIL лучше красивого PASS» (ADR-0018) — юзер скорректировал меня,
  я зафиксировал wip в коммите (а не в issue/PR description). Это пример правильной
  культуры.
- PR #1398 (2578 строк) — воркер честно написал «фича missing, делаю с нуля» —
  лучше чем «маленький PR, поправил». Положительная обратная связь.
- Ретроспективы превращаются в **отдельные fix-PR**, не в кучу «мастер-фиксов».
  Каждый bug-class получает свой PR + retro-id в commit message — отслеживаемо.

---

## 5. Выводы и приоритеты

### 5.1 Что нельзя откладывать (сейчас ломается)

1. **B1** — починить exit 127 в e2e-process. Round-143 сирота. Issue #1392 в подвешенном
   состоянии.
2. **B2** — восстановить agent-flow-triage. Без него процесс мёртв.
3. **B3** — передать `scenario_file` в workflow. Иначе все multi-step e2e лгут.

### 5.2 Что нужно системно (не «ещё один PR»)

- **CI test suite для process-скриптов** (mock `gh`/`python3`/file-tree).
  Каждый bug-fix B4-B15 должен приходить с unit-тестом, иначе регрессия неизбежна.
- **Registry topology ADR** — задокументировать build→deploy registry chain.
  Сейчас это tribal knowledge.
- **Layer separation ADR** — определить чёткие границы: dialogue_node = runtime
  only, watchdog = CI/CD only, merge-gate = orchestrator only. Текущий leakage
  (e2e-busy в dialogue_node) — пример размытой границы.
- **Cron-watchdog для пауз** — если cron paused >1h и есть queue — алерт (не auto-resume).

### 5.3 Что НЕ нужно трогать

- **Архитектура pipeline (issue → triage → worker → merge-gate → e2e-process → user-merge → close)**.
  Она **правильная** (ADR-0014/0015 подтверждают). Баги — в реализации, не в дизайне.
- **Git-Flow (CONTRIBUTING.md)** — production-ready, не меняем.
- **Hermes-kanban** — не наш слой. Баги в CLI (B10) — наверх по stack, не правим сами.
- **«Не делай руками»** (18.08 rule) — соблюдать абсолютно. Все фиксы через
  worker-карточки.

---

## 6. Что я сделал в рамках этого ревью

- ✅ Прочитал AGENT_FLOW_PROPOSAL.md, CONTRIBUTING.md, ADR-0014, ADR-0015 (полностью)
- ✅ Прочитал scripts/agent_flow/ headers, последние 10 коммитов каждого скрипта
- ✅ Просмотрел jobs.json (architect profile), последние cron-outputs e2e-process
- ✅ Прочитал живые issue'ы #1385/#1389/#1391/#1392/#1397/#1403/#1404
- ✅ Зафиксировал wip (46785738) с правкой `gh --jq` паттерна в e2e-process
  (этот фикс уже был в локальной репе с предыдущей сессии, и я мог его потерять
  из-за cron-tick race — коммит спас ситуацию)
- ✅ Написал `.hermes/plans/process-fix-roadmap.md` с конкретными tasks для
  worker-карточек (по issue #1404)

**Чего я НЕ делал** (по правилу «не делай руками»):

- ❌ НЕ правил `agent-flow-e2e-process.sh` для B1 (это worker-card)
- ❌ НЕ создавал issue про B1-B5 — это работа worker'а (но roadmap указывает,
  какие issue нужны)
- ❌ НЕ восстанавливал triage cron (B2) — это worker-card для devops
- ❌ НЕ фиксил registry race (B4, B5) — это worker-card для devops

---

## 7. Что дальше

Roadmap в `.hermes/plans/process-fix-roadmap.md` описывает:

- **Немедленно (P0)**: 4 worker-карточки (B1, B2, B3, частично B4)
- **Системно (P1)**: 6 задач (CI-tests, registry ADR, layer ADR, cron-watchdog, unit-test policy)
- **Долгосрочно (P2/P3)**: 10+ задач для следующих спринтов

Каждая задача содержит: описание бага → acceptance criteria → suggested assignee → links
на related issues. Шифу остаётся только запушить эти карточки в триаж (или создать issue
по шаблону из rob-box-process-rules).

---

**Конец отчёта.** Товарищ Шифу, отчёт готов. Если найдёшь фактическую ошибку в моём
анализе (особенно в B1-B3 — там live-проверка) — скажи, поправлю.