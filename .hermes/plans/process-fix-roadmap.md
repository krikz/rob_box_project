# Process Fix Roadmap — rob_box_project

| Поле | Значение |
|---|---|
| Дата | 2026-08-18 |
| Автор | architect (Hermes Agent) |
| Парный документ | `docs/reports/process-review-2026-08-18.md` |
| Назначение | Конвертация находок ревью в worker-карточки (P0..P3) |
| Статус | ready — Шифу превращает пункты в issue/карточки или правит |
| Re-check (reopen #1404, 18:42 CEST) | Live-подтверждение P0-1, P0-2 (см. §0) |

---

## Re-check при reopen карточки #1404 (18.08 18:42 CEST)

Карточка была переоткрыта (предыдущий run 1932 → completed как "дубль"; см. комментарий
дашборда "Але!! ты же делал ревью процесса а не архитектурное ревью!!!"). На этой
попытке я не переделываю работу: отчёт (commit `1e1fea5b`) и roadmap (commit `1840d09b`)
уже в develop и **соответствуют body карточки** — это ревью **процесса** (issue #1404),
а не архитектурное ревью dialogue_node (это была отдельная карточка t_b127f9b7 → PR #1405).

Что я сверил live прямо сейчас (`hermes cron list` от 2026-08-18T18:42):

```
1082e70dc68f [active] agent-flow-merge-gate    every 5m   Last run 18:34:37  ok
73dcdece0619 [active] agent-flow-e2e-process   every 60m  Last run 18:09:44  error: Script exited with code 127
```

Это **подтверждает в реальном времени два критических бага** из этого roadmap:

- **P0-1 (exit 127)**: last run `error: Script exited with code 127` — тот же
  `kanban: command not found` / `PR: command not found`, что вчера. Issue #1392
  по-прежнему в подвешенном состоянии (round-143 сирота).
- **P0-2 (отсутствующий triage cron)**: в списке только 2 job'а, ни одного
  `agent-flow-triage`. Issues с label `hermes` не превращаются в kanban-карточки
  с 12.08 — сам **этот тикет #1404** по-прежнему висит как `ready` без активной
  dispatch-активности по triage.

Новых архитектурных находок при re-check не появилось (это и не задача карточки).
Если у товарища Шифу есть **конкретный** вопрос к отчёту — напишите в комментарии,
я отвечу или перевыпущу нужную секцию. Если же нужно **архитектурное ревью** — это
отдельная карточка (`agent:architect` для dialogue_node уже отработано в t_b127f9b7;
для скриптов process — будет ADR-0019 / P1-3 из этого roadmap).

---

## Как пользоваться этим roadmap

1. Каждый раздел — **одна или несколько worker-карточек** (assignee указан).
2. Для каждой карточки есть:
   - **Что** — что сломано
   - **Где** — конкретный файл/строка
   - **Acceptance** — что считается «сделано»
   - **Related** — связанные issue/PR/retro
   - **Suggested assignee** — какой профиль
3. Создание issue: Шифу сам решает (правило из rob-box-process-rules: «Issue creation
   (юзер) и merge approval (юзер). Остальное — автомат»). Агент НЕ создаёт issue
   руками — только готовит body.
4. После создания issue + label `hermes` + `agent:<role>` → triage → worker card.

---

## 🔴 P0 — немедленно (сегодня)

### P0-1. Починить exit 127 в agent-flow-e2e-process.sh

- **Что**: bash error `kanban: command not found` / `PR: command not found` на
  line 2285 → SIGPIPE → exit 127 → tick роняется.
- **Где**: `scripts/agent_flow/agent-flow-e2e-process.sh:2285` (process substitution
  `< <(python3 ...)` под `set -euo pipefail`).
- **Live evidence (18:42 CEST)**: `hermes cron list` → `73dcdece0619 Last run
  2026-08-18T18:09:44 error: Script exited with code 127` (тот же тик, что был
  зафиксирован в исходном отчёте — баг **не починен**). Issue #1392 застрял с
  наполовину прогнанным e2e.
- **Acceptance**:
  - [ ] bash `set -euo pipefail` не роняет тик при SIGPIPE в python3 process substitution
  - [ ] Issue #1392 перепрогнан в следующем round без ручного вмешательства
  - [ ] Unit-тест: мок python3 (выдаёт BrokenPipeError) → script exit 0
  - [ ] Лог последнего тика содержит `verdict.txt` для issue #1392
- **Suggested fix** (НЕ делать агенту, но как стартовая точка для воркера):
  - Убрать `pipefail` для process substitution ИЛИ обернуть `< <(python3 ...)` в
    `|| true` с явным логом «SIGPIPE swallowed, content already buffered».
  - Альтернатива: заменить process substitution на именованный pipe с явным
    cleanup + retry.
- **Suggested assignee**: `agent:devops`
- **Related**: live issue #1392; retro t_854c7c67 (post-round sweep mechanics);
  skill `agent-flow-pipeline-ops` → "Common failure modes" → "Host script overwrite".

### P0-2. Восстановить agent-flow-triage cron

- **Что**: jobs.json содержит только 2 job'а (merge-gate, e2e-process). triage cron
  отсутствует. Issues с `hermes` label не превращаются в kanban-карточки с 12.08.
- **Где**: `~/.hermes/profiles/architect/cron/jobs.json` (отсутствует запись);
  репо `scripts/agent_flow/agent-flow-triage.sh` — жив.
- **Live evidence (18:42 CEST)**: `hermes cron list` показывает только `1082e70dc68f`
  (merge-gate) и `73dcdece0619` (e2e-process). **Нет ни одной записи с именем
  `agent-flow-triage`**. Issue #1404 (эта карточка) — живой пример: висит `ready`
  уже >12 минут после создания, карточка завелась через `kanban_create` из
  triage-процесса 09.08 (тот работал), но с 12.08 новые issue'ы могут копиться
  в очереди без диспатча.
- **Acceptance**:
  - [ ] jobs.json содержит запись с `name: "agent-flow-triage"`, schedule `every 1m` или `every 5m`
  - [ ] `script: agent-flow-triage.sh`, `workdir: /home/builder/hermes-share/rob_box_project`
  - [ ] `enabled: true`, `state: scheduled`
  - [ ] Gateway restart после edit (см. memory «jobs.json ≠ gateway»)
  - [ ] Тест: создать issue с `hermes` label → через 5 мин kanban-карточка появилась
- **Suggested assignee**: `agent:devops`
- **Related**: skill `agent-flow-pipeline-ops` → "Creating a task for an agent";
  retro t_b084ae44 (ADR-0018, недавний cron-edit hand-fix).

### P0-3. e2e-process: передавать `scenario_file` в L-E2E Voice Test

- **Что**: `agent-flow-e2e-process.sh` не включает `scenario_file` в `gh workflow run`
  args. Multi-step сценарии (`music_library_suite_v1.json`) ломаются до дефолтного
  smoke-теста «робот, спой песенку про котика» → ложный PASS.
- **Где**: `scripts/agent_flow/agent-flow-e2e-process.sh:2049` (и далее ~10 строк).
- **Live evidence**: cron log 18.08 18:04: `scenario_file=none` в вызове e2e workflow.
- **Acceptance**:
  - [ ] Скрипт сканирует `git diff origin/develop...HEAD --name-only` на наличие
    `.github/e2e/scenarios/*.json` в PR-diff
  - [ ] Если найдено — добавляет `-f scenario_file=<path>` в `gh workflow run` args
  - [ ] Если не найдено — fallback на текущий smoke-test (без изменений)
  - [ ] Unit-тест: мок diff с `music_library_suite_v1.json` → e2e args содержат
    `scenario_file=.github/e2e/scenarios/music_library_suite_v1.json`
  - [ ] Live: PR с scenario → следующий round прогоняет N шагов (не один voice_text)
- **Suggested assignee**: `agent:devops`
- **Related**: issue #1384 (live bug report), PR #1375 (e2e-done ложный).

### P0-4. Верифицировать фикс L-Deploy and Verify.yml

- **Что**: `c9278410` (18.08 10:51) исправил одну из 4 строк (line ~307). Остались
  ещё 3 (`~351, ~414, ~453`). Без полного фикса — registry race B4 жив.
- **Где**: `.github/workflows/L-Deploy and Verify.yml`.
- **Acceptance**:
  - [ ] Все 4 места в workflow используют `== 'local'` (не `!= 'local'`)
  - [ ] Лог deploy с `registry_source=local` содержит `Loading .image-versions.dev`
    ДО `docker compose pull`
  - [ ] `docker inspect <container> --format '{{.Image}}'` на роботе соответствует
    `dev-<sha>` из `.image-versions.dev`
  - [ ] На роботе `grep -c <new_feature>` внутри контейнера > 0 (не пустой)
- **Suggested assignee**: `agent:devops`
- **Related**: issue #1384 (частично), commit `c9278410` (только line ~307).

---

## 🟠 P1 — системные фиксы (эта неделя)

### P1-1. CI test suite для process-скриптов

- **Что**: 0% coverage на bash-скриптах. Каждый bug-fix возвращается регрессией
  через 5-7 дней (counter init #1385, scenario_file #1384, registry race #1384).
- **Где**: создать `scripts/agent_flow/tests/` (директория уже есть, но не используется).
- **Acceptance**:
  - [ ] `tests/test_e2e_process.sh` — проверяет: пустой queue → exit 0,
    SIGPIPE в python3 → не exit 127, scenario_file в args при наличии в diff
  - [ ] `tests/test_merge_gate.sh` — проверяет: orphan-reconcile (MERGED+e2e-done+OPEN PR → needs-review), CI red → block, CI green → needs-e2e
  - [ ] `tests/test_triage.sh` — проверяет: новая issue → карточка, REOPENED →
    новая карточка (после 4h throttle), idempotency guard
  - [ ] `tests/test_round_ensure.sh` — counter file persistence, race с remote clean
  - [ ] Mock-инфраструктура для `gh` (export PATH с shim-скриптами)
  - [ ] CI workflow `G-Run Tests.yml` запускает `bash scripts/agent_flow/tests/*.sh`
    на каждый PR
- **Suggested assignee**: `agent:devops` (создание), `agent:backend` (моки Python)
- **Related**: PR #1386 (regression не покрыта тестом), PR #1390 (revert без теста).

### P1-2. ADR-0018+: Registry topology (build → deploy → robot)

- **Что**: документировать что runner-8 build пушит в свой localhost:5000, deploy
  тянет из 249:5000. Сейчас это tribal knowledge.
- **Где**: создать `docs/adr/0018-registry-topology.md`.
- **Acceptance**:
  - [ ] ADR описывает: где билд, где registry, как SHA-тег попадает в `.image-versions.dev`,
    почему deploy может тянуть старый код, как проверить
  - [ ] Diagrams (mermaid) для каждой Pi (main + vision)
  - [ ] Раздел «симптомы диагностики» — какие команды запускать при «зелёный деплой
    но старый код»
  - [ ] Раздел «фиксы» — что нужно менять чтобы build пушил в 249:5000 напрямую
- **Suggested assignee**: `agent:architect`
- **Related**: commit `c9278410` (частичный fix); issues #1384 (registry race).

### P1-3. ADR-0019: Layer separation (runtime vs CI/CD vs orchestrator)

- **Что**: определить чёткие границы. Сейчас dialogue_node знает про e2e-busy
  (CI/CD concern), watchdog знает про provider-exhaustion (business concern),
  merge-gate знает про workspace cleanup (orchestrator concern).
- **Где**: создать `docs/adr/0019-layer-separation.md`.
- **Acceptance**:
  - [ ] Определены 3-4 слоя (runtime / orchestration / CI-CD / observability)
  - [ ] Для каждого слоя — какие данные он может читать, какие side effects допустимы
  - [ ] Anti-patterns: «не делай X в слое Y» с примерами (e2e-busy в dialogue_node
    как negative example)
  - [ ] Правило: «новый state в runtime-ноде → обязательно init в `__init__`»
  - [ ] Шаблон для review: «этот PR трогает слои X, Y — проверить что не слои Z, W»
- **Suggested assignee**: `agent:architect`
- **Related**: issue #1385 (counter init), issue #1392 (MCP tools registration
  в неправильном слое).

### P1-4. Cron-watchdog для пауз (не только MAINTENANCE)

- **Что**: текущий watchdog проверяет MAINTENANCE flag. Если cron paused вручную
  (через `hermes cron pause` или jobs.json edit) и забыт — очередь накапливается
  без алерта (было live 15.08-18.08: e2e-process paused 3 дня).
- **Где**: новый cron `hermes-cron-watchdog` или расширение `watchdog.sh`.
- **Acceptance**:
  - [ ] Каждые 30 мин: для каждого enabled cron — если `last_run_at` > 2h назад
    И есть очередь (issues с нужными labels, OPEN PR с needs-e2e/e2e-done/merge) →
    алерт
  - [ ] Алерт в Telegram Home (chat_id Krikz Ster), priority:high
  - [ ] НЕ auto-resume (может маскировать legit pause)
  - [ ] Лог в `~/.hermes/profiles/<profile>/cron/output/<watchdog-job>/`
- **Suggested assignee**: `agent:devops`
- **Related**: 18.08 incident (e2e-process paused 3 дня, 3 PR застряли),
  skill `agent-flow-pipeline-ops` → "e2e-process paused = pipeline stalls silently".

### P1-5. WIP-коммит helper: `agent-flow wip <task-id>`

- **Что**: сейчас каждый воркер коммитит wip вручную (правильно, но легко забыть).
  Скрипт-помощник упрощает.
- **Где**: `scripts/agent_flow/wip_commit.sh` в репо + install.sh.
- **Acceptance**:
  - [ ] `bash wip_commit.sh <task-id> <scope> "msg"` → автоматически:
    - проверка branch (develop? warn если нет)
    - `git add -A`, `git commit -m "wip(<scope>): <msg>"`
    - `git push origin HEAD:develop` если на develop worktree
    - лог в kanban коммент «wip committed»
  - [ ] `--no-push` флаг для develop worktrees без авторизации
  - [ ] Unit-тест: dry-run не пушит, но показывает что бы запушил
- **Suggested assignee**: `agent:devops`
- **Related**: retro 09.08 #2 (WIP-коммит rule); live инцидент (мой wip 46785738).

### P1-6. End-to-end smoke-test для всего pipeline

- **Что**: интеграционный тест, который прогоняет fake-issue через всю цепочку
  (issue → triage → worker → PR → merge-gate → e2e → close) и проверяет что
  каждый этап делает то что должен.
- **Где**: `tests/integration/test_full_pipeline.sh`.
- **Acceptance**:
  - [ ] Создаёт sandbox issue, проверяет что через 5 мин есть карточка
  - [ ] Мок-воркер создаёт branch + push, проверяет что PR подхвачен merge-gate
  - [ ] Мок-e2e возвращает PASS, проверяет что issue получила e2e-done + PR needs-review
  - [ ] Cleanup: удаляет issue, branch, карточку
  - [ ] Запускается в `G-Run Tests.yml` ночью (не блокирует PR)
- **Suggested assignee**: `agent:backend` + `agent:devops`
- **Related**: ADR-0014 + ADR-0015 (это их acceptance на уровне pipeline).

---

## 🟡 P2 — следующие 2-4 недели

### P2-1. Round counter persistence hardening

- **Что**: counter file теряется при ручной чистке remote-веток. Live инцидент
  был — после `git push --delete z-{e2e}/test-round-X` numbering может reset.
- **Acceptance**:
  - [ ] counter file хранится в GitHub gist (SOT) + локальный кэш
  - [ ] round_ensure.sh валидирует counter против remote при старте
  - [ ] Если counter < max(remote) — использовать max(remote) + 1
- **Related**: skill `agent-flow-pipeline-ops` → "Round numbering".

### P2-2. Kanban auto-cleanup (stale blocked/todo cards)

- **Что**: `hermes kanban list` показывает >15 мусорных карточек в todo/blocked
  (TASK-***, parent+orphans от auto-decompose, stale PR).
- **Acceptance**:
  - [ ] Cron `kanban-cleanup.sh` (daily): архивирует карточки в `todo`/`blocked`
    старше 7 дней без апдейтов
  - [ ] Skip: карточки с live worker (running), с недавними комментариями, blocked
    by external issue
  - [ ] Dry-run mode по умолчанию; opt-in `--apply`
- **Related**: school rule «Kanban hygiene» (memory note).

### P2-3. Doc-as-code: ADR-0020 «Что мы НЕ делаем в agent-flow»

- **Что**: формализовать anti-patterns. Шифу уже 5+ раз говорил «не делай руками»
  — пора в документ.
- **Acceptance**:
  - [ ] ADR-0020 в `docs/adr/` с явным списком «NEVER»:
    - никогда не править process-скрипты руками (только worker-карточка)
    - никогда не править jobs.json руками (только через `hermes cron update` или worker)
    - никогда не мержить PR самому (Q22)
    - никогда не создавать issue руками (юзер сам, или worker-карточка)
    - никогда не удалять remote-ветку без коммита в develop (R6)
  - [ ] Cross-reference из CONTRIBUTING.md §1
- **Related**: skill `rob-box-process-rules` (уже содержит эти правила, но
  в narrative, не в формальном ADR).

### P2-4. Provider fallback health-check в cron

- **Что**: provider-exhaustion обрабатывается в watchdog'е, но нет proactive
  health-check. Workers узнают про 429/402 только при попытке.
- **Acceptance**:
  - [ ] Cron `provider-health.sh` (every 15m): для каждого profile (architect,
    backend, devops) проверяет MiniMax balance + DeepSeek credits
  - [ ] Если balance < threshold → set MAINTENANCE flag (auto-pause)
  - [ ] Если credits OK → unpause MAINTENANCE
  - [ ] Не касается MiniMax balance для issue-creating (per #1193 narrow rule)
- **Related**: skill `agent-flow-pipeline-ops` → "Provider fallback hygiene".

### P2-5. Worker success rate metric

- **Что**: нет метрик «сколько карточек застряли, сколько прошли с первого раза».
- **Acceptance**:
  - [ ] Prometheus endpoint на каждом profile gateway
  - [ ] Метрики: kanban_cards_total{status}, worker_iterations_total,
    e2e_pass_rate, merge_to_close_time_seconds
  - [ ] Grafana board (опционально, но хотя бы JSON dump)
- **Related**: ADR-0018 («честный FAIL лучше красивого PASS» — нужны данные).

### P2-6. Multi-model e2e matrix (A42)

- **Что**: Phase 4+ в AGENT_FLOW_PROPOSAL.md §10.
- **Acceptance**: 3+ LLM провайдера в одном round, verdict сравнивается.
- **Related**: A42 OPEN в roadmap.

---

## 🟢 P3 — кунсткамера (для следующих спринтов)

### P3-1. Kanban CLI: исправить `--title` parsing (вверх по stack)

- Шифу → hermes-cli maintainer: positional title, не `--title`.

### P3-2. ADR-0014 provenance migration

- Issue #1104 имеет `e2e-done` без PASS provenance → не auto-close. Нужен
  отдельный ручной reconcile (ADR-0014 §7.5).

### P3-3. Worker contract: «обязан коммитить wip каждые 15 мин»

- Сейчас правило есть в школе, но не enforced. Можно добавить watchdog:
  если воркер на branch > 15 мин без коммитов → алерт.

### P3-4. Hermes-cli: `kanban requeue` (вверх по stack)

- Несуществующая команда роняет merge-gate каждый тик (warning в логах).
  Fixed в `7d567420` workaround'ом (create fresh card), но upstream fix нужен.

### P3-5. Test environment isolation

- Сейчас e2e прогоняется на боевом роботе (10.1.1.10/21). Нужна изолированная
  test-среда (docker-compose с stub-rs2) для CI smoke-tests.

---

## Шаблон issue body (для Шифу)

Каждая задача P0/P1 может стать issue. Шаблон:

```markdown
## <type>(<scope>): <что>

**Связанные:** issue #N, PR #M, retro t_X
**Severity:** bug|process|feature
**Тикет на:** <role>-роль
**Assignee:** <profile>

### Что сломано
<1-2 предложения из P0-X.Что>

### Где
<конкретный файл/строка из P0-X.Где>

### Live evidence
<output из cron или live log>

### Acceptance
<скопировать чеклист из P0-X.Acceptance>

### Связанные
- <related issues/PRs>
- skill `agent-flow-pipeline-ops` (если применимо)
- retro t_X (если есть)
```

---

**Конец roadmap.** Товарищ Шифу, готов создавать issue по любому из P0/P1 пунктов
по шаблону выше — скажи какой приоритет, я подготовлю body-файлы (`/tmp/issue_<id>.md`)
и дам команды `gh issue create`. Или запушить roadmap+отчёт как есть, а issue создашь
сам (по rob-box-process-rules — это юзерское право).