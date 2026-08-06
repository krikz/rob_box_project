# Agent Flow Proposal — Issue → e2e Queue → Closed (draft)

| Поле | Значение |
|------|----------|
| Документ | `docs/design/AGENT_FLOW_PROPOSAL.md` |
| Статус | **draft (черновик юзера, оформлен архитектором)**, требует согласования |
| Дата | 2026-08-06 (зачаток по итогам сессии) |
| Автор | PM / юзер (krikz) → оформил: Hermes (subagent по запросу юзера) |
| Связанное | `docs/design/E2E_TESTING_DESIGN_v2.md`, `.github/workflows/L-E2E Voice Test.yml`, существующий cron `e2e-sound-delivery`, канбан `t_*` |
| Назначение | Описать сквозной поток: **GitHub Issue → cron-Hermes увидел → задача в канбан → branch+PR → зелёные CI → e2e-сценарий → артефакты в задачу → re-assign исполнителю → close** |

---

## 0. TL;DR — что юзер накидал черновиком

Сценарий «end-to-end для агент-воркера, который ведёт задачу через GitHub + канбан + e2e-проверку»:

1. **Триггер** — человек создаёт Issue в GitHub (или просит «создай issue для ...»).
2. **Cron** на стороне Hermes (например `*/2 * * * *`) забирает все новые Issue с лейблом `hermes` (или по assign — `krikz`). Это **`agent-flow-triage`** cron — НЕ самостоятельно пишет код, а создаёт **карточку канбана** с инструкцией.
3. **Карточка канбана** содержит (а) issue-метаданные (title/body/labels), (б) **короткую инструкцию для агента-исполнителя** (например «fix voice-assistant silence gate», «docs(api): add error schema»), (в) поле `assignee: <исполнитель>` — для issue-бота обычно «backend»/«architect»/«reviewer» — определяется по labels или эвристике.
4. **Исполнитель** (sub-agent или воркер) должен:
   - создать **ветку** `agent/<short-id>` от актуального main / feature-ветки
   - внести изменения по **инструкции из карточки** (НЕ по issue — карточка может быть сокращена)
   - открыть **PR от своей ветки**
   - **связать PR с issue** (ключевые слова `closes #N`)
5. **Гейт** — PR открыт → ждать зелёных **всех CI**: build OK, все линтеры (G: Lint / Run Tests / Validate Docker Compose / TTS Provider Tests), все кросс-чек.
6. **E2E-маркировка** — после зелёных CI: автоматически (через GitHub labeler) ставится лейбл `needs-e2e` (или руками юзером / reviewer'ом). **Дальше — отдельная очередь e2e**.
7. **E2E-очередь** (`e2e-process` — отдельная сущность / роль / канбан-карточка):
   - читает issue/PR + diff (через `gh pr diff`)
   - **генерирует сценарий** (1–3 голосовые команды), кладёт OGG в `.github/e2e/voice_commands/e2e_<issue_id>/`
   - **запускает** серию через `gh workflow run L-E2E Voice Test.yml --ref <PR-branch>`, --ref — обязательно (иначе берётся main → mimo, дважды наступали)
   - **собирает артефакты** (`.wav` 16kHz, mp3 через `ffmpeg`, RMS-метрики, transcript-dump)
   - **комментит в issue/PR** сводку + прикладывает артефакты (`gh pr comment`)
   - **re-assign** задачу в исходную роль исполнителя (он уже сделал код) — там он видит доказательства работы
8. **Close** — исполнитель (или ревьюер, или юзер) видит артефакты, подтверждает → Issue → `done`, карточка канбана → `done`, в changelog записывается что вышло.

Цель: **полный цикл «идея → доказательства работы → close» без человеческих «передергиваний» посередине**. Юзер делает только две вещи: «создай issue» и «закрой issue когда уверен» (или «вот доказательства — сам закрою»).

---

## 1. Что сейчас УЖЕ работает (06.08) — точки опоры для proposal

| Что | Где | Покрытие |
|-----|-----|----------|
| Cron в Hermes качает deliver-mp3 | `~/.hermes/.../cronjobs/04b0846a4066 e2e-sound-delivery` (paused юзером) | ✅ в одну сторону — watcher→Telegram mp3 |
| Worker | `claude-code / codex / opencode / hermes` subagents | ✅ — умеют писать коммит→билд→деплой→rebase |
| Сами скрипты e2e | `e2e_series7.sh`, `e2e_watcher7.sh`, watcher poll | ✅ — 10/10 verdicts v7 |
| Отдельный e2e-раннер | `rob-box-e2e-1` (лейбл `e2e`), workflow `L-E2E Voice Test.yml` → `runs-on: e2e` | ✅ сделано сегодня по требованию юзера |
| Канбан | Hermes kanban + `~/.hermes/kanban/boards/robbox` | ✅ — карточки t_* существуют |
| `gh` доступ | GOODWORKRINKZ (perms) | ✅ |

**На proposal нужны новые сущности:**

- `agent-flow-triage` — cron для «видит issue с лейблом → создаёт карточку канбана» (раз в 2 мин, как watchdog).
- `agent-flow-merge-gate` — бот, который после PR-open ждёт `checks=ok` и **триггерит `e2e-process`** (или ставит лейбл `needs-e2e`).
- `e2e-process` (может быть cron, может быть отдельный worktree) — генерит OGG → dispatch'ит workflow → собирает артефакты → re-assign.

---

## 2. Маркеры для агента (что именно делать)

### 2.1 Issue-side

| Маркер | Пример | Что делает cron-triage |
|--------|--------|-------------------------|
| Лейбл `hermes` | `labels: hermes, voice` | подхватывается |
| Лейбл `agent:backend` | (assign=backend/architect/reviewer) | определяет **дефолтного исполнителя** карточки |
| Лейбл `priority:P0..P2` | | мапится на канбан-приоритет |
| Milestone / Project | | опционально |

**Минимум для оформления proposal:** «нужны 1+ маркеров на issue» — например **только `hermes`** (=кто-то другой может поставить лейбл, тогда триаж включится).

### 2.2 PR-side

| Маркер | Что значит |
|--------|-----------|
| Связь с issue (`Closes #N` / `Fixes #N` в description) | стандартно, увязывает канбан |
| Лейбл `needs-e2e` | внешний cron e2e-process может её подобрать |
| Лейбл `e2e-ready` (альтернатива) | то же |

**Триггер триажа на КАРТОЧКЕ при наличии issue == `assignee`/лейбла** — это про дизайн.

### 2.3 e2e-процесс ожидает

- **название PR-ветки** вида `agent/<id>` или `feature/<id>` — parser вытаскивает
- **bashкоманда запуска**: `gh workflow run "L-E2E Voice Test.yml" --ref <branch> -f environment=test -f voice_file=.github/e2e/voice_commands/<file>.ogg -f volume=150 -f record_seconds=120`
- **артефакты**: `.wav` 16k mono + .mp3 + transcript-dump (отдельная тема)

---

## 3. Контракты между частями

### 3.1 `agent-flow-triage` (cron)

**Inputs:** GitHub issues с лейблом `hermes` (за период с прошлого тика).

**Outputs:** канбан-карточки с:
```
title: <issue title truncated>
body:
  -- issue-meta --
  ## Source
  • repo: krikz/rob_box_project
  • issue: #<N>
  • url: <link>
  • labels: hermes, ...
  
  ## Agent instruction (короткое — ПОЛНОЕ тело issue НЕ копируем, оно большое)
  <тут 1-3 предложения максимум — что конкретно делать>
  
  ## Branch + PR convention
  • branch: agent/issue-<N>-<short-slug>
  • commit style: feat/fix/docs(scope): ...
  • link PR with "Closes #<N>"
  
  ## Done criteria
  • CI all green (build + lint + tests + e2e)
  • PR opened + reviewed
  
  • assignee: <определён по labels>
labels: [<assigned role>]   # backend / architect / reviewer
priority: P<P0-P2>
```

**TBD:** как детектить «issue уже обработан» (state `in_progress`, parent issue number в комментарии → state `closed`, или by created comment с маркером `kanban: t_<task_id>`).

### 3.2 agent-исполнитель (sub-agent)

**Inputs:** kanban-карточка (через Hermes API для selection).

**Действия:**
1. Получает карточку, читает инструкцию + родительскую issue.
2. Создаёт ветку `agent/issue-<N>-<short-slug>` от main (или от feature/harness-p0-foundation, если флажок).
3. Делает **локальные изменения** (включая `docs/`/скрипты если нужно). Если архитектор/док — только документы, коммит, PR без кода.
4. **Перед push:** `git fetch origin` + `rebase origin/<base>` (защита от race с воркерами-архитекторами, мы уже наступали).
5. Push, `gh pr create --base <base> --fill` (title из issue / card, body с `Closes #<N>`).
6. Ждёт CI (или cron/agent-flow-merge-gate за этим смотрит).
7. **По завершении ставит** на PR + issue комментарий `kanban: t_<task_id>`.

### 3.3 `agent-flow-merge-gate` (cron / on-PR-check)

**Inputs:** PR с label `needs-e2e` (или хотя бы у PR-ветки `agent/issue-*` + merge_state=clean).

**Checks:**
- `gh pr checks --watch` — все passed (build / lint / unit tests / docker-compose / TTS provider tests)
- `gh pr view --json mergeable` — `MERGEABLE=true`, конфликтов нет
- `gh pr diff <NUM> --patch` — обязательно глянуть (можно и не смотреть, но желательно)

**Action:** ставит лейбл `e2e-triggered` или прямо диспатчит `gh workflow run` для e2e (через отдельный worktree или sub-agent).

### 3.4 `e2e-process` (отдельный субъект)

**Inputs:** `gh issue view <N>`, `gh pr view <PR_NUM> --json title,body,files` (чтобы понять, что меняется), текущий список OGG.

**Действия:**
1. Сгенерирует **1–3 OGG-сценария** в папку `.github/e2e/voice_commands/e2e_<issue_id>/`. Генерация вне CI:
   - docker `gen_prov.py` (есть в проекте — MiniMax TTS, voice=`male-qn-qingse` или юзер) → `docker cp` → `ffmpeg -i raw.wav -c:a libopus -b:a 32k -ar 16000 -ac 1 <file>.ogg` → коммит в ветку `agent/e2e-<id>`.
   - **Триаж:** если issue = «voice / e2e / и т.п.», можно сгенерить вручную (юзер/архитектор).
2. `gh workflow run L-E2E Voice Test.yml --ref <branch> -f environment=test -f voice_file=... -f volume=150 -f record_seconds=...` — **БЕЗ `--ref` берётся main → mimo-конфиг** (юзер дважды наступал).
3. `gh run watch $RUNID --exit-status` — пауза между прогонами (наш `e2e_series7.sh` уже так делает).
4. Сбор артефактов: `gh run download --name run-<id>-dialog_e2e.wav` + ffmpeg→mp3 + mediainfo / RMS-аудит.
5. **Comment** в PR (и issue) с markdown-блоком:
   ```
   ### e2e автоматика для #<issue_id> (PR #<pr_num>)
   - сценарий: <file>.ogg (содержание: <asr>)
   - результат: SUCCESS, mean RMS -16.6 dB
   - mp3: attach1 (сегодня — через Telegram-доставку MEDIA:/path/to.mp3)
   - verdict: A39 + A40 satisfied
   ```
6. **Re-assign** исполнителя (по labels) — ставит assignee обратно на роль, что бы агент-разработчик **получил обратно карточку с доказательствами**.
7. Issue/PR — `status: green` + готов к merge. Если FAIL — повторно assign + комментарий с ошибками.

### 3.5 Юзер (финальный close)

- Видит mp3 в Telegram (по cron-delivery или MEDIA-файл).
- Или **сам** лезет в run logs, проверяет.
- `gh issue close #N` (или approve PR для вливания).

---

## 4. Триггеры (как стыкуются с реальным API Hermes)

| Триггер | Где | Период |
|---------|-----|--------|
| `agent-flow-triage` cron | Hermes → `~/.hermes/...cronjobs/<id>.sh` (script = `gh issue list --label hermes --json ... | build card`) | every 2 min |
| `e2e-process` cron | отдельный job poll'ит PR с `needs-e2e` | every 5 min |
| `agent-flow-merge-gate` | webhook on `pull_request` (нота — пока бот через cron check by sha) | every 2 min |

Хермес cronjob — это `cronjob(no_agent=True, script=...)` + `deliver=telegram` или `local`. (см. существующий `Agent Cockpeat Watch Tock` — pattern есть).

---

## 5. Открытые вопросы (нужен ответ юзера / PM)

| Q# | Вопрос |
|----|--------|
| **Q1** | Маркер на issue — **только лейбл `hermes`** или +обязательно `agent:<role>`? Если второе — как роль определяется (`assignee` от юзера / по другим лейблам)? |
| **Q2** | E2E: триггерим **только при лейбле `needs-e2e`** (ручной контроль reviewer'а) или автоматом на любой PR в `agent/<id>/`? Юзер сказал «всегда», но в v2 e2e было отдельной фазой. |
| **Q3** | OGG-генерация в CI или **offline**? Если в CI — нужно держать `MINIMAX_API_KEY` в GH Secrets, есть риск лимита (как сейчас с prod). Offline безопаснее, но дольше. |
| **Q4** | Re-assign на исполнителя — **программно через bot** (комментарий «@user, проверь артефакты») или **на того же роль (по лейблу карточки)**? |
| **Q5** | Авто-merge `gh pr merge --auto` после e2e-SUCCESS или **только re-assign** и merge ручной (юзер/ревьюер)? Юзер раньше говорил «не спешить с merge». |
| **Q6** | TL;DR гипотеза: 8 фаз = 8 PR-ов? Нужен **мастер-план** или каждая issue — сама по себе карточка? |
| **Q7** | Куда складывать артефакты (wav+mp3) — github-action artifacts (коротко, 90 дней), release assets или **отдельный репо/папка**? |
| **Q8** | Observability: хочется ли dashboard (Live EOS spreadsheet) или достаточно «1 issue + 1 PR + 1 e2e run + 1 mp3»? |

---

## 6. Порядок реализации (proposal по фазам для согласования)

### Phase 1 — Triage MVP (1-2 дня)

**Goal:** cron видит issue с лейблом `hermes` → создаёт карточку канбана с правильной метаданной.

- [ ] Cron `agent-flow-triage` в Hermes
- [ ] Парсинг issue via `gh issue list --label hermes --json number,title,body,labels,assignees`
- [ ] Создание карточки канбана через kanban-tool (нужен cli-обёртка?)
- [ ] Тест: создаю issue → жду 2 мин → карточка появляется в robbox-board
- [ ] Не открывать issue сразу sub-agent'ом — только `ready` статус

**Accept:** 1 issue → 1 card, idempotency при ретри (уже обработанные issue не дёргаются повторно).

### Phase 2 — Agent в ветке+PR (2-3 дня)

**Goal:** выбранный агент действительно решает задачу, делает ветку, открывает PR.

- [ ] Hand-off от карточки → sub-agent: `select_agent(role)` (можно первый slice — только architect/backend/devops)
- [ ] agent привязан к подготовке: git clone / checkout / создать `agent/issue-<N>-<slug>`
- [ ] PR creation с правильным шаблоном: title, body, `Closes #<N>`
- [ ] Acceptance: создать issue «fix voice-action-server aiohttp» (та самая A43) → через 30-60 мин PR с фиксом появился + CI запущен

**Accept:** manual test of 1 воркера — issue доходит до зелёного PR.

### Phase 3 — Merge gate + e2e (2-3 дня)

**Goal:** зелёный CI автоматом запускает e2e-серию, артефакты прикладываются.

- [ ] `agent-flow-merge-gate` cron: PR all-checks-pass → label `e2e-ready`
- [ ] `e2e-process` cron: dispatch L-E2E Voice Test.yml (ТОЛЬКО через наш e2e-раннер)
- [ ] Comment summary в PR + issue
- [ ] OGG-генерация (offline или через secrets)
- [ ] `e2e_<issue_id>/` папка, прикладываемая через upload-artifact

**Accept:** завести тестовую issue, проверить, что e2e-прогон отрабатывает через наш runner с правильными артефактами.

### Phase 4 — Polishing / observability (по запросу)

- [ ] Manager dashboard (tab в вебе / telegram-bot)
- [ ] SLA / метрики (средний time-to-merge per role)
- [ ] Self-approve-e2e (rollback на fallback'и)

---

## 7. Risks / открытые риски

| R | Risk | Mitigation |
|---|------|-----------|
| R1 | Sub-agent плохо интерпретирует «короткое тело карточки» | Ссылка на родительское issue, никогда не обрезать |
| R2 | Race воркеров на общем workspace (как раньше sparse-checkout) | Отдельный раннер per-role, очистка workspace между прогонами (как сделано 06.08) |
| R3 | MiniMax 429 — e2e не проходит из-за лимита | Fallback на deepseek для ogg-generation if min лимит, см. `LLM fallback` фикс 06.08 |
| R4 | Юзер не получает mp3 в Telegram — снова спам cron | mp3-доставка on-demand: bot pin «pull your mp3 here» + ручное прикладывание к issue |
| R5 | Re-assign on-the-fly нарушает assignees в GitHub | Использовать **только внутренний канбан-assignee**, не GitHub-assignee. |

---

## 8. Что НЕ входит в proposal (открыто для следующих итераций)

- **авто-merge** (`gh pr merge --auto` после e2e-success) — отдельно, юзер явно не хочет спешки
- **multi-model e2e matrix** — A42 OPEN, требует OGG для каждого TTS/LLM/STT комбо
- **Hypothesis-test mode** — issue → 8 фаз → 8 PR-ов автоматом
- **Re-rank / planning decisions** — выставление приоритетов P0..P2 по содержимому issue
- **Self-healing про e2e** — автоматический re-test при flake

---

## 9. TODO / on-merge

После согласования (юзер sign-off на Phase 1):

1. Создать issue `feat(agent-flow): phase 1 triage MVP`
2. Сделать карточку канбана (как тут описано в §1)
3. Завести cron `agent-flow-triage` в Hermes
4. Подождать первого реального issue (для теста) — посмотреть, пришла ли карточка
5. Review в слёте (отчёт по достижениям setup'а)

---

**Готовый артефакт:** данный файл — после согласования юзером → берется за Phase 1.
