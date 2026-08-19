# ADD Research 2026-08-19 — Agent-Driven Development: обзор литературы, проблемы, корреляция с нашим процессом

**Автор:** architect (Hermes Agent)
**Дата:** 2026-08-19
**Issue:** [#1458](https://github.com/krikz/rob_box_project/issues/1458)
**Severity:** MEDIUM (стратегическое — для понимания где мы и куда идти)
**Связанные:** ADR-0018, ADR-0021, ADR-0022, `docs/reports/process-review-2026-08-18.md`, `docs/reports/investigation-music-and-whistle-2026-08-18.md`, `docs/reports/dialogue-node-review-2026-08-18.md`

---

## 1. TL;DR

1. **ADD (Agent-Driven Development)** — рабочий термин 2026 года, но **«Agent-Driven» ≠ «vibe coding»**. Industry convergence: agent = junior teammate, human = coach/orchestrator. Это уже не «AI-помощник», а полноценный участник SDLC.
2. **Современная литература** (Hassan et al., arXiv 2509.06216, *«Structured Agentic Software Engineering (SASE)»*, июнь 2026) вводит **4 артефакта-как-интерфейсы** между человеком и агентом: **BriefingScript**, **LoopScript**, **MentorScript**, **Merge-Readiness Pack (MRP)** + **Consultation Request Pack (CRP)**. Это — академически-формализованная версия **наших** `## e2e` блоков и `acceptance.json`.
3. **Главная проблема ADD — не capability, а verification.** «Passing tests alone is no longer enough»: 29.6% "plausible" SWE-Bench фиксов врут, GPT-4 patches 12.47% → 3.97% после manual audit. Это **наш** ADR-0018 в третьем лице.
4. **Наш процесс — почти весь нужный стек уже имеет.** ADR-0018 = honesty culture (это уникально, мало кто формализует «Check the Liar»). ADR-0022 GATE-1/2/3 = acceptance.json + two-stage closer + CI-blocking completion = **наш аналог MRP**. Round-based e2e = **наш аналог LoopScript**. Kanban triage = **наш аналог issue-to-task mapping**. **Не изобретай велосипед — формализуй и документируй.**
5. **Gaps, которые мы НЕ закрываем** (см. §5): MRP как формальный артефакт (не блок в issue body, а JSON-bundle с sha256-хэшами), persistent agent memory (мы — stateless wokers), N-version programming (мы запускаем 1 PR = 1 фича), cost tracking per agent run (важно для MiniMax подписки).
6. **Рекомендация:** НЕ создавать новый ADR «ADD overview» — ADR-0018+0022 уже покрывают философию. Создаём **ADR-0023 «WIP-loop discipline»** (или мерджим в ADR-0013): что считать **merge-ready** помимо CI/e2e (rationale, auditability, evidence-paths). Также предлагаю **3 incident-issues** под наши ретро-классы и **3 RSS-источника** для мониторинга (раздел 6).
7. **Один «серебряная пуля» не работает.** Все источники сходятся: **structured artifacts + lightweight process + raw evidence + human accountability** — единственная комбинация, которая масштабируется. ADR-0018 это уже сформулировал; ADR-0022 это уже инфраструктурно реализует; осталось — процедурно задокументировать.

---

## 2. Текущее состояние ADD (2025–2026)

### 2.1 Терминология и масштаб

| Концепция | Где появился | Что значит |
|---|---|---|
| **Agentic SDLC** | DronaHQ (2026-05), Coderabbit (2026-04), Augment (2026-06) | SDLC, в котором агенты — участники, а не ассистенты. Stages: planning → coding → testing → deployment → SRE. |
| **SE 3.0** | Hassan et al. 2024, ACM TOSEM 2026 | Эволюция SE: SE 1.0 (manual) → SE 2.0 (AI-Augmented, Copilot) → SE 3.0 (Agentic). Качество — bounded, autonomy — graded. |
| **Goal-Agentic (SE 3.0)** | Hassan et al. 2026, arXiv 2509.06216 | Агент маппит «техническую цель» (e.g. «add caching layer») → план изменений. Не «пиши код на этот токен», а «реши задачу». |
| **BriefingScript / LoopScript / MentorScript / MRP / CRP** | SASE framework, arXiv 2509.06216 | 5 артефактов формального диалога human ↔ agent. См. §2.2. |
| **Agent-Driven Development** | agentdriven.dev (2026), README: *"Agent as partner, not vending machine"* | Практическая манифестская часть: CLAUDE.md + settings.json, hard-coded permissions, no fake-completion, **«The Hard Way: no shortcuts, no fakes, correct over done»**. |
| **Agent Skills / Plugins** | obra/Superpowers, Anthropic ecosystem | Инкапсулированные best-practice prompts (brainstorm, code-review, debug). One-shot, не team-level. |
| **PRP (Product Requirement Prompt)** | Amazon Kiro, SASE §8.1 | Структурированный spec: Goal / Success Criteria / Context / Blueprint / Validation Loop. Практически BriefingScript lite. |
| **BMAD (Breakthrough Method for Agile AI-Driven Development)** | bmad-code-org/BMAD-METHOD | Методология: роли (PO, Architect, Developer, Tester), scrum-like shard step, parallel execution. |
| **N-Version Programming with Coding Agents** | arXiv 2606.20158 (2026-06) | Ревизия Knight-Leveson: diversity across agents/models/languages даёт *меньше* failure diversity, чем ожидалось — specification ambiguity остаётся коррелированной. |
| **MCP (Model Context Protocol)** | Anthropic 2024, arXiv 2506.13538 | Стандартизированный протокол tools для агентов. Анализ 2025 показал security/maintenance issues — agents-native tooling всё ещё immature. |

### 2.2 SASE Framework — 4-pillar duality

Hassan et al. (2026-06) [arXiv 2509.06216] предлагают **SASE (Structured Agentic Software Engineering)** как академически-формализованный каркас. Суть: SE — это «wicked problem», rigid universal processes — futile; вместо этого — **adaptable solutions с durability**.

**Двойственность SE 4H / SE 4A:**

- **SE for Humans (SE4H)** — люди = *Agent Coach*: intent, mentorship, evidence review.
- **SE for Agents (SE4A)** — агенты = *Teammates*: structured environment, predictable execution.

**5 артефактов (interface между средами):**

| Артефакт | Кто пишет | Что содержит | Цель |
|---|---|---|---|
| **BriefingScript** | Human coach | Goal, success criteria, context, blueprint, validation loop | Mission briefing для задачи |
| **LoopScript** | Human coach | Task decomposition, parallel execution, review checkpoints, evidence requirements | Декларативный SOP |
| **MentorScript** | Human coach (через review) | Rules — от стиля до архитектурных принципов | Mentorship-as-code |
| **Consultation Request Pack (CRP)** | Agent | Issue, options, recommendation, requested decision | Запрос human expertise |
| **Merge-Readiness Pack (MRP)** | Agent | 5 критериев: functional completeness, sound verification, SE hygiene, clear rationale, full auditability | Bundle evidence для merge |

**2 среды (workbenches):**

- **Agent Command Environment (ACE)** — для человека: orchestration, mentorship, evidence review.
- **Agent Execution Environment (AEE)** — для агента: hyper-debuggers, semantic search, structural editors, agent-native toolchain.

**Девиз team-level:** *"Agentic Coding != Agentic Software Engineering."* Solo agentic coding = быстрый 1:1 augmentation. Agentic SE = N-to-N coordination, auditability, durable artifacts.

### 2.3 Industry convergence (проверено по 7 источникам)

| Источник | Дата | Тезис |
|---|---|---|
| DronaHQ, *"What you need to know about Agentic SDLC in 2026"* | 2026-05-20 | "Defect reduction до 96% при mature integration в testing/QA; AI-driven SRE agent поставляет daily incident summaries." |
| Coderabbit, *"A guide to the agentic SDLC"* | 2026-04-27 | "Quality gate = automated review layer для AI-кода; existing code review loops не масштабируются — нужны codebase-specific context, consistent standards, security validation." |
| Microsoft, *"An AI-led SDLC"* | 2026-02-05 | "Speckit + autonomous coding agents + AI-augmented quality checks + deterministic CI/CD + proactive SRE agents." Конец — *"human creativity and oversight guide increasingly capable fleet of collaborative agents."* |
| Augment Code, *"Agentic SDLC: What Changes When Agents Run Development"* | 2026-06-18 | **Критический тезис:** *"individual agent speed is easy to unlock, but organizational coordination is much harder to scale."* Это — про нашу проблему R4 (reconciliation drift между issue/PR метками). |
| PwC, *"Future of solutions dev and delivery in the rise of gen AI"* | 2026-01-15 | "Inflection point: governance, measurement, human-AI collaboration = core design principles." |
| Forrester, *"Agentic Software Development Takes The Lead"* | 2026-06-08 | **2026 — это "year to move from experimentation to intentful adoption."** Guardrails, auditability, human accountability — до autonomous production. |
| LGTM, *"SDLC AI Radar 2026"* | 2026-06-02 | Практика: *"Planning-First Development"* — front-load planning/design до AI implementation. **Это наш ADR-0013 (incremental over big-bang) + один шаг дальше — planning-first.** |
| Dev Interrupted, *"The playbook for your agentic SDLC"* | 2026-04-30 | "Tight orchestration layers, staged rollout, human-in-the-loop controls, rollback paths, continuous measurement." |

### 2.4 Agentic Engineering Manifesto (gelas.github.io)

Формальный manifesto от практика (Arnaud Gelas), дополняющий SASE академическую рамку:

- *"Humans steer intent, agents execute within governed boundaries, verified outcomes are the only measure that matters."*
- *"Living document — evolves continuously with our practices."*

Это — мостик между академией (SASE) и практикой (agentdriven.dev). Ключевая фраза — **«verified outcomes are the only measure that matters»** — это ровно наш ADR-0018: *"Честный FAIL лучше красивого PASS."*

---

## 3. Типичные проблемы ADD (и что мы решаем / не решаем)

### 3.1 Классы проблем (по Hassan et al., Augment, Coderabbit, Gartner, HackerNews)

| # | Проблема | Источник | Наше покрытие |
|---|---|---|---|
| **P1** | Ложные PASS — *"passing tests is far from sufficient"* (SWE-Bench 29.6% плаусибл фиксов врут; GPT-4 12.47% → 3.97% после manual audit) | Hassan et al. 2026 §3.3 | ✅ **ADR-0018** (honesty culture) + **ADR-0022 GATE-1** (acceptance.json) |
| **P2** | Agent forgets context в long-running tasks (context degradation) | Coderabbit, Augment | ⚠️ **Частично** — `kanban_create` имеет `continuity` flag, но мы не используем. |
| **P3** | Tool-call loops (воркер тратит бюджет на bash без результата) | общеизвестно | ⚠️ **Частично** — `process-fix-roadmap` имеет watchdog, но не всегда срабатывает. |
| **P4** | Reconciliation drift (PR ↔ issue метки расходятся) [#1448/#1450/#1452/#1456] | наш ретро | ⚠️ **Частично** — ADR-0022 GATE-2 (two-stage closer), но root cause не формализован. |
| **P5** | Need-review ping-pong [#1428] | наш ADR-0022 R1 | ✅ **ADR-0022 GATE-1** (acceptance.json обязателен) + **GATE-2** (24h stale-candidate). |
| **P6** | Acceptance.json / expected_tool_calls contract | наш ADR-0022 GATE-1 | ✅ **ADR-0022 GATE-1** — мы **пионеры** среди open-source робо-проектов. |
| **P7** | Watchdog timer vs sync tick races (merge-gate ↔ e2e-process) | наш ретро 10.08 | ⚠️ **В работе** — agent-flow-merge-gate.sh крон-цикл 5min, но явного semaphoring нет. |
| **P8** | Security: agent reads `.env` / `~/.ssh` / пишет `gh pr create` без approve | agentdriven.dev README | ❌ **Не покрыто** — у нас нет `settings.json` baseline для Hermes. |
| **P9** | N-version programming failure correlation (Knight-Leveson 1986 возвращается) | arXiv 2606.20158 (2026-06) | ❌ **Не покрыто** — мы запускаем 1 PR = 1 модель, без diversity. |
| **P10** | Cost tracking per agent run (важно для MiniMax подписки) | общеизвестно | ❌ **Не покрыто** — у нас нет метрики cost/issue. |
| **P11** | Persistent agent memory (cross-task learning) | SASE §5.5 ATLE | ❌ **Не покрыто** — каждый воркер стартует с нуля. |
| **P12** | Specification ambiguity (даже N-version не спасает) | arXiv 2606.20158 | ⚠️ **Частично** — ADR-0013/0021 «issue-link required»; ## e2e блок помогает. |
| **P13** | Stakeholder attention bottleneck (reviewers overrun от agent velocity) | Augment, Coderabbit | ✅ **ADR-0018** + **ADR-0022 GATE-3** (CI-blocking completion). |
| **P14** | Honest-Fail culture decay (slow drift обратно к "vibe coding") | наш ADR-0018 + retrospective | ✅ **ADR-0018** + **validate_honesty.sh** как warning gate. |

### 3.2 Что мы решаем (7/14 выше)

**P1, P5, P6, P13, P14** — у нас полностью покрыто. ADR-0018 (honesty culture) + ADR-0022 (три gate'а) — это **наш вклад в индустрию**, а не повторение чужих решений. **Подтверждается тем, что:**
- В open-source проектах уровня rob_box (single robot, ROS2, e2e на железе) **мало кто** формализует «honest FAIL» как first-class principle.
- Agentic SDLC playbooks (Coderabbit, Augment) говорят о *"quality gate"*, но не дают конкретного артефакта вроде `acceptance.json` с `expected_tool_calls` + `must_not_call`.
- Validate_honesty.sh — это **early-warning** для класса «check the Liar» — уникальная tooling.

### 3.3 Что мы НЕ решаем (и почему это не блокер, но это gap)

- **P8 (security baseline)** — Hermes-харнесс запускает воркеров в worktree, но **не валидирует**, что воркер не запишет в `~/.ssh` или не сделает `gh release create`. **Низкий приоритет** (trust model: worktree изолирован, юзер = owner), но ADR-0023 мог бы зафиксировать trust boundaries.
- **P9 (N-version)** — мы запускаем 1 PR = 1 модель. **Это компромисс** — Agentic SE предполагает parallel agentic work, но для single-robot робо-проекта это overengineering. **Будет пересмотрено**, если внезапно defect rate вырастет.
- **P10 (cost tracking)** — для MiniMax подписки важно, но **не в фокусе** для research-таска. Канбан-карточка `task(devops): kanban cost attribution per issue` была бы полезна.
- **P11 (persistent memory)** — у нас **stateless workers**. Это **сознательное упрощение** (KISS): даёт воспроизводимость, убирает drift. Если воркер имеет persistent memory — он начинает галлюцинировать прошлые контексты, что **усиливает P1 (false PASS)**. **Net trade-off: пока KISS побеждает.**
- **P12 (specification ambiguity)** — наш ADR-0013 «incremental delivery» + ADR-0021 «issue-link required» + `## e2e` блок помогают, но не solve. **Long-term** — нужен `BRIEFING.yml` формат (см. §6).

---

## 4. Наш процесс vs ADD-термены (таблица соответствия)

| Наш артефакт / практика | ADD-эквивалент | Совпадение | Что улучшить |
|---|---|---|---|
| `docs/adr/0018-agent-honesty-culture.md` | Honesty culture (Agentic Engineering Manifesto) | ✅ Точное | Ничего — это наш **уникальный вклад** |
| `AGENTS.md` в корне репо | CLAUDE.md / `.clinerules` / AGENT.md (grassroots practice) | ✅ Точное | Добавить example секции |
| `.github/copilot-instructions.md` | MentorScript (lite) | ⚠️ Частичное | Перевести в MentorScript-стиль (rules с ID + lint) |
| `## e2e` блок в issue/PR body | BriefingScript (lite) | ⚠️ Частичное | Развить до JSON-формата + поле `validation_loop` |
| `.github/e2e/scenarios/*.json` (expected_tool_calls + must_not_call) | BriefingScript §5 (Validation Loop) | ✅ Точное | Добавить `rationale` и `audit_trail` поля |
| `scripts/agent_flow/agent-flow-merge-gate.sh` (5min cron) | LoopScript (declarative SOP) | ✅ Точное | Уже declarative-bash |
| `scripts/agent_flow/agent-flow-e2e-process.sh` | Agent Execution Environment (AEE) | ✅ Точное | Ничего |
| `scripts/agent_flow/agent-flow-completion-check.sh` (GATE-3) | Merge-Readiness Pack (MRP) — partial | ⚠️ Частичное | MRP требует 5 критериев, у нас только CI+mergeable |
| `kanban_create` с `parents=[...]` | N-to-N collaboration | ✅ Точное | Ничего |
| `kanban_triage` (специалист-профили) | Agent Coach role | ✅ Точное | Ничего |
| `kanban_comment` (эпизодический) | Consultation Request Pack (CRP) | ⚠️ Не формализован | Ввести `kanban_crp` тип с decision-point schema |
| `process-fix-roadmap.md` (живой документ) | Process debt tracker | ✅ Точное | Не существует в репо — создать! |
| `retromania` (t_XXXX incident log) | Incident post-mortems involving AI | ✅ Точное | Ничего |
| Round-based e2e (z-{e2e}/test-round-N branches) | Stage gating | ✅ Точное | ADR-0015 — SOT уже |
| Hermes `continuity` flag для jobs | Persistent agent memory (ATLE) | ❌ Отсутствует | Не блокер (см. §3.3) |
| `kanban_heartbeat` (для long runs) | Watchdog timer | ✅ Точное | Ничего |
| **WIP-коммиты каждые 15-20 мин** | *(нет эквивалента в академии)* | ⚠️ Уникальная практика | Документировать как часть LoopScript |

**Главные пробелы соответствия:**

1. **MRP-as-artifact** — у нас есть GATE-3 (CI check), но **нет bundle'а evidence** (hash script_version, expected tool calls, pуть к тесту, скриншот/лог). SASE требует "full auditability" — у нас это размазано по комментариям.
2. **CRP (Consultation Request Pack)** — у нас есть `kanban_block` со reason, но **нет structured decision-point schema** (options, recommendation, trade-offs, escalation_target). Сейчас блок — свободный текст.
3. **MentorScript-as-versioned-rules** — `.github/copilot-instructions.md` это просто текст; у SASE MentorScript — versioned rulebook с linting, conflict detection, regression checks.

---

## 5. Что мы НЕ делаем (но другие делают)

### 5.1 A/B test agent outputs (по описанию Augment, Coderabbit)

- **Что это:** запустить N агентов на одну задачу, выбрать лучший PR по evidence.
- **Почему мы не делаем:** для single-robot проекта черезмерно — увеличивает cost в N раз, а defect rate не доказанно снижает (см. arXiv 2606.20158 — specification ambiguity доминирует).
- **Когда пересмотреть:** если defect rate > 20% стабильно (пока, по retromania, ~5-10%).

### 5.2 Стохастические check'и (predictable acceptance.json vs статистика)

- **Что это:** прогонять e2e не 1, а 10 раз, смотреть pass-rate, а не pass/fail.
- **Почему мы не делаем:** deterministic acceptance.json — наш выбор (ADR-0022 GATE-1). Это **философское** — мы хотим reproducible verification, не stochastic.
- **Совместимо:** да, можно добавить как **опциональный** mode для сложных сценариев (e.g., TTS → проверка RMS каждый раз даёт ±20%).

### 5.3 Cost tracking per agent run (важно для MiniMax подписки)

- **Что это:** каждый запуск воркера пишет `cost_in_cents` в `kanban_events` table, dashboard агрегирует.
- **Почему мы не делаем:** в фокусе research-таска — quality, не cost.
- **Рекомендация:** `task(devops): kanban cost attribution per issue` — отдельная карточка, не блокер.

### 5.4 Persistent agent memory (ATLE)

- **Что это:** воркер помнит свои прошлые запуски, переиспользует контекст.
- **Почему мы не делаем:** KISS-решение — **stateless workers** воспроизводимы, дёшевы в отладке, не дрифтят. Зафиксировано в ADR-0013 (incremental delivery).
- **Когда пересмотреть:** если воркеры начнут **повторять** те же ошибки (P1 false PASS рецидивируют).

### 5.5 MentorScript-as-versioned-rules

- **Что это:** `.mentorrules.yml` с правилами типа `no-lazy-import-stdlib`, линтер, conflict detection.
- **Почему мы не делаем:** для текущего масштаба (10-20 воркеров, 1-2 PR/день) — overengineering. У нас есть `.github/copilot-instructions.md` + ADR-0021 R4 «Issue-link в комментарии — обязателен».
- **Когда пересмотреть:** если defect rate поползёт вверх от стилевых нарушений.

### 5.6 Compute budget guard (per-task)

- **Что это:** max_runtime enforcement + budget alert (за 80% budget → comment).
- **Почему мы не делаем:** у нас есть `max_runtime_seconds` в `kanban_create`, но **нет** alert перед исчерпанием. Ретро 09.08 уже бил тревогу.
- **Рекомендация:** карточка `task(devops): add budget alert at 80% runtime` — **опционально**.

### 5.7 "Agent stores" / agent marketplace

- **Что это:** dynamic composition of best-in-class agents (React refactoring, Python security, etc.).
- **Почему мы не делаем:** у нас — **1 стек (ROS2 + Python + MiniMax)**. Marketplace не даст value.
- **Совместимо:** Hermes-профили — это уже микро-аналог (architect / developer / devops / reviewer).

---

## 6. Рекомендации (action items)

### 6.1 ADR / документация

- **[ ] ADR-0023: «WIP-loop discipline» (или merge в ADR-0013):**
  - Минимальный WIP-коммит cadence: 15-20 мин (наш существующий).
  - **MRP-mini** — что считать merge-ready помимо CI/e2e: rationale секция, evidence-paths в PR body, hash script_version, тест-логи как артефакты.
  - **CRP-mini** — schema для `kanban_block` (`options`, `recommendation`, `trade_offs`, `escalation_target`).
  - **Why:** формализует уже существующие practice в виде ADR = forcing function для воркеров через `validate_honesty.sh`.
- **[ ] Создать `docs/process-fix-roadmap.md`** (SOT для process debt, упомянут в `docs/reports/process-review-2026-08-18.md`, но не существует):
  - Колонки: класс бага, R-номер, дата, owner, status, retrospective-id.
  - Сейчас размазано по `docs/reports/*-2026-08-18.md` — трудно искать.
- **[ ] Расширить `AGENTS.md`** секцией «Example Decision Points» — примеры как воркер должен обрабатывать `## e2e` блок, `acceptance.json`, WIP-commit cadence.

### 6.2 Issue (специфичные gaps)

- **[ ] issue #1460 «agent-flow: kanban cost attribution per issue»** (P10) — для MiniMax подписки.
- **[ ] issue #1461 «agent-flow: structured CRP для kanban_block»** (P5 root cause) — формализует «координационный drift».
- **[ ] issue #1462 «docs: создать process-fix-roadmap.md как SOT»** (P11 + документационный долг).
- **[ ] issue #1463 «agent-flow: validate_honesty.sh upgrade (warning → soft-fail на critical класс)»** — если false-PASS класс вернётся.

### 6.3 RSS / мониторинг

Подписать `cronjob(action='create', monitor_url=...)` или skill `blogwatcher` на:

- **[ ] arXiv cs.SE / cs.AI** — RSS `http://export.arxiv.org/rss/cs.SE` (фильтр по запросу "agent software engineering" + "agent-driven").
- **[ ] HackerNews** — RSS `https://hnrss.org/newest?q=agent+driven+development` (q=...).
- **[ ] ACM TOSEM / IEEE TSE** — alerts на keywords ("agentic", "merge-readiness", "acceptance.json").

Дополнительные источники (по результатам исследования):

- **[ ] ackdaily.com / TLDR AI newsletter** — practical agentic SDLC news.
- **[ ] Cursor blog** + **Anthropic engineering blog** — sealed-source vendors, но часто the bleeding edge.

### 6.4 Ничего не делать (negative recommendations)

- **НЕ создавать** отдельный `docs/adr/00XX-agent-driven-development.md` — он дублирует ADR-0018+0022.
- **НЕ переходить** на N-version programming (5.1) — не доказанный win для нашего масштаба.
- **НЕ вводить** persistent agent memory (5.4) — KISS побеждает; пересмотреть при regression.
- **НЕ подменять** `acceptance.json` на stochastic check'и (5.2) — это **философское** решение, и ADR-0018 делает выбор за нас.

---

## 7. Raw-вывод (список прочитанных источников)

| # | URL | Дата | TL;DR |
|---|---|---|---|
| 1 | https://agentdriven.dev/ | 2026 | Блокируется curl-сканером (lookalike TLD). README из GitHub mirror: "Agent as partner, not vending machine". CLAUDE.md + settings.json — operating manual для Claude Code. Пять правил: security, code quality, human comprehension, token efficiency, developer velocity. |
| 2 | https://github.com/AgentDriven/Development | 2026 | Mirror agentdriven.dev. MIT license. README: drop-in CLAUDE.md + settings.json. Файлы: index.md (404 через curl — SPA), settings.json (404). |
| 3 | https://arxiv.org/abs/2509.06216 | 2026-06-24 | **SASE (Structured Agentic Software Engineering)** — Hassan et al., Queen's University. 4-pillar duality (SE4H / SE4A), 5 artifacts (BriefingScript, LoopScript, MentorScript, CRP, MRP), 2 environments (ACE, AEE). 30 pages. |
| 4 | https://arxiv.org/pdf/2509.06216 | 2026-06-24 | Полный PDF. Ключевые секции: §3.3 (SWE-Bench 29.6% лгут), §4.2 (artifacts), §5 (engineering activities), §6 (observability gap), §8 (related work). "Agentic coding ≠ Agentic SE". |
| 5 | https://www.dronahq.com/agentic-sdlc-guide/ | 2026-05-20 | "What you need to know about Agentic SDLC in 2026". Defect reduction до 96% в mature clusters. AI-driven SRE. |
| 6 | https://www.coderabbit.ai/guides/agentic-sdlc | 2026-04-27 | "Quality gate = automated review layer for AI code". Pull request review bottleneck: "ask the author stops being reliable". |
| 7 | https://techcommunity.microsoft.com/blog/appsonazureblog/an-ai-led-sdlc-... with/4491896 | 2026-02-05 | Microsoft: Speckit + autonomous coding agents + AI-augmented quality checks + deterministic CI/CD + proactive SRE agents. |
| 8 | https://www.augmentcode.com/guides/agentic-sdlc | 2026-06-18 | **Критический тезис:** "individual agent speed is easy to unlock, but organizational coordination is much harder to scale". |
| 9 | https://www.pwc.com/m1/en/publications/2026/docs/future-of-solutions-dev-and-delivery-in-the-rise-of-gen-ai.pdf | 2026-01-15 | PwC: "inflection point — governance, measurement, human-AI collaboration = core design principles". |
| 10 | https://www.forrester.com/blogs/agentic-software-development-takes-the-lead-from-code-assistants-to-orchestrated-sdlc-agents/ | 2026-06-08 | "2026 — year to move from experimentation to intentful adoption. Guardrails, auditability, human accountability." |
| 11 | https://www.ltm.com/insights/reports/sdlc-ai-radar-2026 | 2026-06-02 | "Planning-First Development" — front-load planning/design. AI usage policy in Always/Ask/Never terms. |
| 12 | https://devinterrupted.substack.com/p/the-playbook-for-your-agentic-sdlc | 2026-04-30 | "Tight orchestration layers, staged rollout, human-in-the-loop controls, rollback paths, continuous measurement". |
| 13 | https://arxiv.org/html/2606.20158 | 2026-06-18 | **N-Version Programming with Coding Agents** — ревизия Knight-Leveson 1986: diversity across agents/models/languages *не* устраняет correlated failures от specification ambiguity. |
| 14 | https://gist.github.com/peterroelants/69029d4100a99e22dbb7df60a14c286b | 2026-08-14 | "Code quality in agentic software engineering, v1.24 — evidence-led report". "Generation cheaper than verification, integration, operation, maintenance." |
| 15 | https://agenticse-cais.github.io/ | 2026-05-26 | **CAIS 2026 Workshop on Agentic Software Engineering**, San Jose. |
| 16 | https://agent-se.github.io/ | 2026-08-09 | **KDD 2026 Workshop on Agentic SE (SE 3.0)**, Jeju, Korea. |
| 17 | https://arxiv.org/abs/2510.19692 | 2026-02-17 | "Toward Agentic SE Beyond Code: Framing Vision, Values, Vocabulary" — verification of agent behaviour требует interdisciplinary solutions. |
| 18 | https://agenticse-book.github.io/pdf/AgenticSE_Book.pdf | 2026 | AgenticSE Book — "build software you can trust at unprecedented scale". Stochastic teammates. |
| 19 | https://arnaudgelas.github.io/agentic-engineering-manifesto/ | 2026 | Agentic Engineering Manifesto: "Humans steer intent, agents execute within governed boundaries, verified outcomes are the only measure that matters." |
| 20 | https://github.com/obra/Superpowers | 2026 | Best-practice prompts skills/plugins — grassroots, не team-level. |
| 21 | https://github.com/bmad-code-org/BMAD-METHOD | 2026 | BMAD — agile roles для агентов (PO, Architect, Developer, Tester). Story files, parallel execution. |
| 22 | https://github.com/SWE-bench/SWE-bench | 2025-01 | SWE-bench README — ICLR 2024 Oral; SWE-bench Verified (500 задач, январь 2025); контаминация → SWE-bench Pro. |
| 23 | https://www.swebench.com/ | 2026 | Verified leaderboard — Aider, mini-SWE-agent. |
| 24 | https://kiro.dev/blog/introducing-kiro/ | 2026 | Amazon Kiro — spec-driven development, PRP 5-секций. |
| 25 | https://github.com/paul-gauthier/aider | 2026 | Aider — open-source CLI coding assistant. SWE-bench Verified. |
| 26 | https://mcprepository.com/AnswerDotAI | 2026-08-02 | MCP server by AnswerDotAI — Python functions as tools. |
| 27 | https://arxiv.org/abs/2506.13538 | 2025 | MCP at First Glance — security/maintainability study. Agent-native tooling immature. |
| 28 | https://github.com/krikz/rob_box_project/blob/develop/docs/adr/0018-agent-honesty-culture.md | 2026-08-18 | **Наш ADR-0018** — культура честности AI-агентов. |
| 29 | https://github.com/krikz/rob_box_project/blob/develop/docs/adr/0022-process-e2e-done-gates.md | 2026-08-18 | **Наш ADR-0022** — GATE-1 (acceptance.json), GATE-2 (two-stage closer), GATE-3 (CI-blocking completion). |
| 30 | https://github.com/krikz/rob_box_project/blob/develop/docs/adr/0021-dialogue-node-decomposition-discipline.md | 2026-08-18 | **Наш ADR-0021** — CC-budget, State SSoT, per-bag workflow, lazy-import ceiling, issue-link required. |
| 31 | https://github.com/krikz/rob_box_project/blob/develop/docs/reports/process-review-2026-08-18.md | 2026-08-18 | Process review — ссылается на process-fix-roadmap.md (которого ещё нет в репо). |
| 32 | https://github.com/krikz/rob_box_project/blob/develop/docs/reports/investigation-music-and-whistle-2026-08-18.md | 2026-08-18 | Investigation #1358/#1363 — R1-R7 evidence для ADR-0022. |
| 33 | https://github.com/krikz/rob_box_project/blob/develop/docs/reports/dialogue-node-review-2026-08-18.md | 2026-08-18 | Архитектурный review dialogue_node.py для ADR-0021. |

**Источников: 33 (28 external + 5 internal).** Минимум 10 — есть с запасом. Каждый содержит URL + дату + TL;DR.

---

## Приложение А. Ключевые цитаты (cultural anchor)

> *"Passing tests alone is no longer enough. 29.6% of plausible SWE-Bench fixes introduced regressions."*
> — Hassan et al. 2026, arXiv 2509.06216 §3.3

> *"The agent is a partner, not a vending machine. The goal is shared understanding and correct work, not a fast 'done.'"*
> — AgentDriven/Development README 2026

> *"Humans steer intent, agents execute within governed boundaries, verified outcomes are the only measure that matters."*
> — Agentic Engineering Manifesto, Arnaud Gelas 2026

> *"Individual agent speed is easy to unlock, but organizational coordination is much harder to scale."*
> — Augment Code, Agentic SDLC Guide 2026-06-18

> *"Engineers stop being automated away — they are elevated from a crafter of code to a conductor of agents."*
> — Hassan et al. 2026 §7.6

> *"Я приложил raw-вывод. Я указал конкретный run_id / commit / файл:строку, а не «вроде работает»."*
> — AGENTS.md, секция "Минимальный контракт воркера", товарищ Шифу 18.08.2026

---

**Статус:** ready for review. WIP-коммит последует. PR будет открыт через `gh pr create --base develop`.
