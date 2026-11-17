# ADR-0162: Процесс один — координатор и воркеры; автоматика hermes / agent-flow выводится из эксплуатации

| Поле | Значение |
|---|---|
| Статус | **Proposed**: черновик для товарища Шифу, решения — §7 |
| Дата | 2026-11-05 |
| Автор | Claude Code (воркер), по задаче Шифу 05.11 |
| Код сверен | `origin/develop` @ `7dc4a962b` (05.11) |
| Слова Шифу (05.11) | «автоматизация hermes, скорее всего, уже заброшенная тема. Надо провести анализ этих карточек и того, что есть в репозитории, перевести это всё в deprecated и вывести процесс, которым мы пользуемся сейчас, в абсолют. Проанализировать и, может, взять оттуда что-то полезное» |
| Что меняет | Единственный процесс — `docs/process/claude-coordinator-robot-flow.md`. `docs/design/AGENT_FLOW_PROPOSAL.md`, `scripts/agent_flow/`, `.hermes/`, процессные ADR домена `AF-` и hermes-ADR в общей серии — **Deprecated** (история). Код в этом ADR не удаляется и workflow не отключаются: это отдельные маленькие PR по §6 (ADR-AF-0013). |
| Родители | ADR-0018 (культура честности — остаётся), ADR-AF-0013 (маленькие PR), ADR-0148 (код решает, LLM говорит) |
| Заменяет (как процесс) | `docs/design/AGENT_FLOW_PROPOSAL.md` (#1038), ADR-0014, 0019, 0022, 0025, 0031, 0035, 0036, 0040, 0041, 0042, 0045, 0046, 0047, 0048, 0049, 0050, 0115, 0116 и все `AF-*` — в части cron-триажа, kanban, merge-gate, e2e-раундов |

Пометки достоверности: **измерено** — команда и вывод в §2; **по коду** — прочитано в репо; **догадка** — помечено явно.

## 1. Контекст

Hermes — агентная платформа на build-хосте (`/home/builder/.hermes/`, профили `agent-flow`, `architect`, `devops`, `backend`, `analyst`). Поток (`AGENT_FLOW_PROPOSAL.md` §0): issue с меткой `hermes` → cron `agent-flow-triage` (каждую минуту) → kanban-карточка `t_*` → воркер на ветке `z-{agent}/…` → cron `agent-flow-merge-gate` (5 мин) → cron `e2e-process` (раунды `z-{e2e}/test-round-N`) → метки `needs-e2e` / `e2e-done` → закрытие. Вокруг — 14 cron-джобов из `install.sh` (watchdog-и, sweep-ы, ночной ревью, ночной марафон) плюс триаж, merge-gate и drift-detect (по коду: `scripts/agent_flow/install.sh`, строки 983–1459).

Объём (измерено, `find`/`wc`): `scripts/agent_flow/` — 248 файлов, ~91 тыс. строк `.sh`/`.py`; `.hermes/` — 5 файлов; 28 ADR `AF-*`; 82 ADR общей серии упоминают hermes (`grep -li hermes docs/adr/[0-9]*.md`).

С 05.10 автоматика стоит, а работа идёт иначе:

- **05.10** коммит `6b7ed35cf` (автор `hermes`): «maintenance: pause agent-flow (Шифу работает руками)» — пустой файл `MAINTENANCE` в корне репо. Файл лежит в `develop` до сих пор. Последний коммит автора `hermes` — этот (измерено).
- Последний PR воркера hermes (ветка `z-*`) — #3390 от 05.10, закрыт; открытых PR с `z-*` — 0 (измерено).
- С 22.10 в `develop` влито **351 PR** через сессию Claude Code (автор `GOODWORKRINKZ`), ни одного через merge-gate hermes (измерено).
- Эпики #3683 и #4236 ведутся по `docs/process/claude-coordinator-robot-flow.md` — это записано в их телах (по коду / по issue).
- Единственный живой след hermes в GitHub — метки `hermes` + `agent:devops`, которые ставит `L: Deploy and Verify` на карточки «🚨 Deploy issues on …» (§3). Триажа, который их «превратит в kanban-карточку» (комментарий в workflow), нет; карточки закрывают координатор и Шифу руками (измерено: #4353, #4277, #4210).

Что на build-хосте сейчас (крутятся ли crontab-и `~/.hermes`, гейт `MAINTENANCE` во всех ли скриптах) — **не проверено**: на хост не ходил. По #3389 гейта нет в 19 watchdog-скриптах; в репо файлов с `MAINTENANCE`/`af_maintenance_gate` — 25 из 65 `.sh` (измерено `grep -l`). Догадка: если crontab жив, часть watchdog-ов тикает вхолостую или пишет в kanban.db, которую никто не читает.

## 2. Факты: workflow и карточки (raw, 05.11 ~07:00 UTC)

### 2.1 Workflow GitHub Actions

`gh workflow list --all` — все 36 записей `active` (в т.ч. 4 без файла в репо: «Copilot cloud agent», «L: Integration Tests», «G: Agent Mode Tests», «L: Build On Branch Push»). Прогоны — `gh run list --workflow <файл> --limit 3 --json conclusion,createdAt,event,databaseId`:

| Workflow | cron | Последние 3 прогона (run_id дата событие итог) | Связь с hermes | Оценка |
|---|---|---|---|---|
| G: Run Tests | — | 41970559871 11-05 push success; 41969773195 11-05 PR success; 41966475918 11-05 push success | нет | текущий процесс |
| G: Lint Code (+ Repo Guards) | — | 41970559880 11-05 push success; 41969773231 PR success; 41966475845 push success | вызывает `scripts/agent_flow/validate_adr_namespace.sh` (стр. 633) и `tests/test_retro_intent_extractor.sh` (стр. 766) | текущий процесс; держит 2 скрипта agent_flow |
| G: Dialog Core Package | — | 41970559869 11-05 push success; 41969773170 PR success; 41966475644 push success | нет | текущий процесс |
| G: TTS Provider Tests + Coverage | — | 41969773232 11-05 PR success; 41965198515 PR success; 41963102238 PR success | нет | текущий процесс |
| G: Architecture Audit | `17 2 * * *` | 41970559856 11-05 push success; 41969773222 PR success; 41966475733 push success | нет | текущий процесс |
| G: Auto-close Issues on Develop Merge | — | 41970561493 11-05 PR success; 41966476461 PR success; 41963831705 PR success | наследие AF-0063 (#3013) | полезно: `Closes #N` при мёрже в develop |
| G: PR Contract-Drift Bot | — | 41971206228 11-05 workflow_run success; 41971149199 success; 41970452125 success | ADR-0044, задуман для merge-gate | живой; полезность не мерил |
| L: Build All Services | — | 41979886670 11-05 dispatch success; 41967200873 success; 41942942912 success | нет | текущий процесс (ручной) |
| L: Deploy and Verify | — | 41980499366 11-05 dispatch success; 41967820283 success; 41943592644 success | ставит метки `hermes`, `agent:devops` на deploy-карточки | текущий процесс; метки — наследие |
| L: E2E Voice Test | — | 41978744025 11-05 dispatch success; 41977708603 failure; 41976832403 failure | был целью `e2e-process` | живой, запускается вручную |
| L: Architecture Audit | — | 41944640581 11-05 dispatch success; 40499068353 10-27 success; 37749775763 10-08 success | нет | ручной |
| G: Agent Flow Process Checks | — | 39422373895 10-20 push success; 39421661677 10-20 PR success; 39401491949 10-20 push success | **да**: требует регистрации cron watchdog-а в `install.sh` | deprecated — защищает то, чем не пользуемся |
| G: CC-Ref Remote Verify (nightly) | `7 4 * * *` | 41825401262 11-04 schedule **failure**; 41630634354 11-03 failure; 41443030083 11-02 failure | «human (or agent-flow triage) picks it up» | не hermes, но **красный каждую ночь**: `TTSNode.dialogue_callback (since) — указывает на #3291 в state='closed'` (лог 41825401262). Подбирать некому — отдельная карточка |
| B: Build CI Image | `0 4 * * 2` | 41645124763 11-03 dispatch success; 41645092117 success; 41629444473 11-03 schedule success | нет | текущий процесс |
| G: Auto-merge to Main | — | 41970559840 11-05 push success; 41966475771 success; 41963831217 success | нет | держит вечный PR develop→main (#3713) |
| G: Auto-merge Feature to Develop | — | 41910067378 11-04 push success; 41906609715 success; 41867615878 success | нет (feature/feat/copilot ветки) | живой; авто-PR без CI (см. память «PR от бота без CI») |
| G: Validate Docker Compose | — | 41902002318 11-04 push success; … | нет | текущий процесс |
| G: Quest Caddyfile Contract | — | 41803561167 11-04 push success; … | нет | текущий процесс |
| G: Bridge Protocol TS Drift | — | 40901150922 10-29 push success; … | нет | текущий процесс |
| G: WebXR glTF Pipeline Verify | — | 40703386581 10-28 PR success; … | нет | текущий процесс |
| G: Check Container Status Tests | — | 39421662143 10-20 PR success; … | нет | текущий процесс |
| G: Vision Pi Cold-Start Tests | — | 41970566538 11-05 PR success; … | нет | текущий процесс |
| CodeQL Advanced | `26 7 * * 2` | 41970566237 11-05 PR success; … | нет | безопасность |
| Microsoft Defender For Devops | `18 10 * * 5` | 41970566265 11-05 PR success; … | нет | безопасность |
| L: Build Base Images | — | 32447301839 **08-21** dispatch success | нет | редкий ручной |
| L: Build Single Service | — | 35657644591 09-21 dispatch failure; 35657614868 failure; 35625222951 success | нет | редкий ручной |
| L: Build Msgs Image, L: Build Pi Services | — | прогонов нет (вызываются из Build All как reusable) | нет | текущий процесс |
| L-Diagnose Runner Network | — | 36551967874 09-29 push success | нет | диагностика |
| [DEPRECATED] build-all-local | — | 18817359175 **2025-10-26** failure | нет | уже deprecated, файл лежит |

Итог по workflow: **cron-джобов hermes в GitHub Actions нет** (по коду: `cron:` есть только у пяти строк выше, ни одна не agent-flow). Шум от hermes в GitHub — не cron, а метки на deploy-карточках. Cron-шум hermes, если он есть, живёт на build-хосте (не проверено).

### 2.2 Открытые карточки системы

`gh issue list --state open --label <метка>` (измерено):

| Метка | Открытые |
|---|---|
| `hermes` | #3374, #3389 |
| `agent:devops` | #3374, #3389 |
| `agent-flow-error` | #3389 |
| `ai-generated` | #3389, #4009, #4014, #4015 |
| `source:gsd` | #4009, #4014, #4015 |
| `process` | #3374, #3389 |

| # | Что | Актуальна? | Рекомендация |
|---|---|---|---|
| #3374 | «agent-flow-triage-cron не подхватил #3354» (04.10). #3354 закрыт 14.10 | нет: триаж на паузе с 05.10, deploy-карточки разбирает координатор | закрыть как устаревшую, ссылкой на ADR-0162 (решение Шифу) |
| #3389 | «MAINTENANCE gate в 19 watchdog скриптах» (05.10); PR #3390 закрыт без мёржа | нет, если hermes выводится; станет «гасить crontab на хосте» | закрыть как устаревшую, заменить шагом §6.1 |
| #4009, #4014, #4015 | баги/фичи музыки (24.10), метки `ai-generated`/`source:gsd` по шаблону `.github/copilot-instructions.md` | да, это живая работа | оставить; метки — наследие шаблона, не сигнал hermes |

Ещё 30+ меток hermes-процесса (`needs-e2e`, `e2e-done`, `e2e:rejected`, `kanban`, `agent-flow`, `agent:*`, `stale-candidate`, `agent-flow-watchdog-orphan`, …) — `gh label list` (измерено). Удалять — §6.5.

### 2.3 Кто создаёт «🚨 Deploy issues on develop»

`L: Deploy and Verify`, шаг «🚨 Create Deployment Issue (any issues)» (строки ~1572–1695): при `deploy_status == 'issues'` — один issue на (ветка, окружение, день), дедуп по маркеру `deploy-fail:<branch>:<env>:<date>` в теле, повторный прогон — комментарием; метки `deployment`, `hermes`, `agent:devops`; в теле хвосты логов и статусы контейнеров. Создано с 06.10 — 20 карточек (измерено), все закрыты руками; последняя #4353 закрыта 05.11 06:25 с raw-списком зелёных run_id.

Оценка: **полезно** — это единственный сигнал красного деплоя, который не теряется, дедуп работает. Бесполезны только метки `hermes`/`agent:devops` и комментарий «триаж превратит в kanban-карточку».

## 3. Решение

1. **Процесс один** — `docs/process/claude-coordinator-robot-flow.md`. Его суть (по 351 влитому PR и эпикам #3683, #4236):
   - **Шифу** ставит цель и даёт права; канал — Telegram через бота робота (`[tg-in]` в логе `telegram-bot`) и чат приложения; «да» на мёрж/конфиги хостов — в чате приложения.
   - **Координатор** (сессия Claude Code) держит план эпика с критерием приёмки, пишет issue с raw-фактами и брифы, принимает отчёты, мёржит в `develop` (squash через REST, проверка `merged=true`), запускает `L: Build All Services` и `L: Deploy and Verify`, ведёт марафон и хендофф, закрывает issue с raw.
   - **Воркер** (субагент, `isolation: worktree`, ветка от `origin/develop`) — одна задача: код + тесты + гарды локально + проверка на роботе (`hotpatch.sh`, `inject.sh`, HIL `scripts/hil/hil.sh`) + PR через `gh api … pulls --method POST` + отчёт с raw и «что не проверено».
   - **CI на PR:** G: Run Tests, G: Lint Code (Repo Guards), G: Dialog Core Package, G: TTS Provider Tests, G: Architecture Audit и др. Образы PR-CI не собирает — только `L: Build All Services`.
   - **Приёмка эпика** — марафон `scripts/e2e/run_night_marathon.sh` со сверкой по логу робота (П1 ≥ 85 % честных, П2 = 0 ложных) и прослушивание Шифу.
2. **Hermes / agent-flow — Deprecated.** Документы остаются как история; новых правил на их основе не писать; карточки kanban `t_*`, метки `needs-e2e`/`e2e-done`, раунды `z-{e2e}/test-round-*` и cron-триаж в текущем процессе не участвуют.
3. **Культура честности (ADR-0018, AGENTS.md) остаётся без изменений** — это часть текущего процесса, а не hermes.
4. **Домен номеров `AF-` замораживается** вместе с hermes: новые процессные ADR идут в общую серию (этот — 0162). Гард `validate_adr_namespace.sh` по-прежнему проверяет оба домена, его не трогаем.
5. Отключение и удаление — **отдельными маленькими PR** по §6, порядок и «да» — за Шифу (§7).

## 4. Сверка AGENTS.md «Процесс-команды» с практикой

| Было в AGENTS.md | Как на деле | Правка в этом PR |
|---|---|---|
| «Issue → triage → kanban → worker → PR → wait for Шифу merge» | triage и kanban с 05.10 нет; цикл — issue → бриф → воркер → PR → CI → мёрж координатором → сборка → деплой → проверка → закрытие с raw | заменено на реальный цикл |
| «НЕ мёржить PR (только Шифу)» | в `develop` мёржит координатор по праву, данному Шифу в чате (эпики #3683, #4236); `main` — только Шифу; воркер не мёржит | уточнено: кто и куда |
| «НЕ фиксить баги руками (только через воркеров)» | так и есть: код пишут воркеры | переформулировано позитивно |
| «В карточках kanban…», «в kanban-комментариях» | карточки — GitHub issue | «kanban» → «issue» |
| ссылка `docs/adr/0013-incremental-delivery-over-big-bang.md` | такого файла нет: под 0013 лежит ReSpeaker DSP, правило — `AF-0013-…` | ссылка исправлена |
| `AGENT_FLOW_PROPOSAL.md` — «общий процесс» | процесс — `docs/process/claude-coordinator-robot-flow.md` | добавлены ссылки, PROPOSAL помечен «история» |

## 5. Компоненты: оставить / deprecated / удалить позже

| Компонент | Решение | Почему |
|---|---|---|
| ADR-0018, раздел честности в AGENTS.md / CONTRIBUTING | **оставить** | основа текущего процесса |
| `scripts/agent_flow/validate_adr_namespace.sh` + `tests/test_validate_adr_namespace.sh` | **оставить → перенести** в `scripts/lint/` или `scripts/ci/` | hard gate в G: Lint Code (стр. 633); защищает от коллизий номеров ADR, нужен и сейчас |
| `scripts/agent_flow/tests/test_retro_intent_extractor.sh` + его цель | **перенести или удалить вместе** | шаг в G: Lint Code (стр. 766); извлекатель интента ретро нужен только hermes — проверить потребителя перед удалением |
| `scripts/agent_flow/validate_test_ws_dirs.py` | **проверить** | упоминается в G: Run Tests (стр. 143, в комментарии) |
| `scripts/agent_flow/validate_honesty.sh` | **перенести** в `scripts/lint/`, по желанию подключить soft-шагом к PR-телу | ловит «работает/PASS» без raw — ровно то, чего требует AGENTS.md. Но тест в Git Bash: 18 PASS / 5 FAIL (exit 1, кириллица в байтовой локали; в Linux не гонял) — сначала починить тест |
| `scripts/agent_flow/gh-pr-create-via-gh-api.sh`, `push-via-gh-api.sh` | **перенести** в `scripts/dev/` (догадка: пригодятся воркерам при лимите GraphQL) | текущий процесс создаёт PR через REST — та же идея |
| `worker_pre_flight.sh` / `worker_post_flight.sh`, `validate_branch_freshness.sh`, `validate_pr_scope.sh` | **взять идеи, код — удалить позже** | «база = origin/develop», «ветка не отстала», «PR не тащит чужое» — уже в брифах воркера; скрипты завязаны на kanban и `~/.hermes` |
| Дедуп deploy-карточек в `L: Deploy and Verify` | **оставить**, снять метки `hermes`/`agent:devops` и комментарий про триаж | живой и полезный сигнал |
| G: Auto-close Issues on Develop Merge | **оставить** | закрывает `Closes #N` при мёрже в develop |
| G: PR Contract-Drift Bot | **оставить пока**, оценить отдельно | работает; задуман под merge-gate |
| G: Agent Flow Process Checks | **deprecated → удалить** первым из workflow (PR-2) | охраняет регистрацию cron в `install.sh` |
| G: CC-Ref Remote Verify (nightly) | **оставить**, но починить красный (#3291 закрыт) | не hermes; красный 3 ночи подряд, «подберёт триаж» — некому |
| `install.sh`, `cron-loop.sh`, `watchdog*.sh`, все `agent-flow-*-watchdog.sh`, `*-sweep.sh`, `agent-flow-triage.sh`, `agent-flow-merge-gate.sh`, `agent-flow-e2e-process*.sh`, `agent-flow-handoff.sh`, `agents_sleep*`, `round_*.sh`, `kanban-*.sh`, `hermes_github.sh`, `vendor/`, `lib_*` | **deprecated → удалить позже** | рантайм hermes; потребителей вне `scripts/agent_flow/` нет, кроме перечисленного выше (по коду: `grep -rln agent_flow/` — `scripts/testing/test_blocker_filter.sh`, `src/rob_box_voice/test/unit/test_issue_1532_voice_multi_skazka.py`; проверить их до удаления) |
| `agent-flow-night-marathon.sh` | **удалить позже** | марафон теперь — `scripts/e2e/run_night_marathon.sh` |
| `.hermes/` (5 файлов планов/инвентаря) | **удалить позже** (или в `docs/archive/`) | артефакты карточек `t_*` |
| `.agents/skills/` (61 скилл) — `agent-flow`, `worker-rebase-protocol`, `pr-cleanup-checklist`, `rebase-pollution-check`, `triage`, `retro`, … | **проверить по одному** | часть — общие (`diagnosing-bugs`, `codebase-design`, `grilling`), часть привязана к kanban |
| `docs/design/AGENT_FLOW_PROPOSAL.md`, `docs/process/agent-flow-openspec-integration.md`, `scripts/agent_flow/README*.md` | **deprecated** (шапка в этом PR) | история |
| процессные ADR (`AF-*` и перечисленные в «Заменяет») | **deprecated** (как класс, этим ADR; файлы не трогаем) | ~47 файлов (28 `AF-*` + 19 из «Заменяет») править ради шапки — шум; достаточно этого ADR. Рантайм-ADR, написанные агентами hermes (автор «architect (Hermes Agent)»), не deprecated |
| `CONTRIBUTING.md` — разделы merge-gate, triage, kanban, e2e-метки | **переписать позже** (PR-6) | честность оставить, механику hermes убрать |
| `.github/copilot-instructions.md` — метки `ai-generated`, `source:gsd` в шаблоне `gh issue create` | **поправить позже** | порождает метки, которые уже ничего не значат |
| файл `MAINTENANCE` в корне | **удалить последним**, после того как crontab на хосте снят | пока hermes может проснуться, он — единственный стоп-кран |
| Метки hermes-процесса (30+) | **удалить позже** | после закрытия карточек, которые на них ссылаются |

## 6. План вывода из эксплуатации (маленькие PR, ADR-AF-0013)

Порядок — предложение; что первым — решает Шифу (§7, В1).

1. **Хост (не PR):** Шифу или координатор с доступом смотрит на build-хосте `hermes cron list` / `crontab -l` / `ps -eo pid,cmd | grep agent-flow` и снимает cron-джобы профилей. Это первое, что даёт эффект: если что-то и шумит, то там. Raw в карточку.
2. **PR-1:** `L: Deploy and Verify` — снять метки `hermes`, `agent:devops` и строку «догоняем метки» с deploy-карточек; комментарий «триаж превратит в карточку» → «разбирает координатор». Дедуп не трогать.
3. **PR-2:** удалить `G: Agent Flow Process Checks.yml` (с grep-доказательством, что других потребителей нет).
4. **PR-3:** перенести `validate_adr_namespace.sh` (+ тест) из `scripts/agent_flow/` в `scripts/lint/`, поправить пути в G: Lint Code. Перенести/удалить `test_retro_intent_extractor.sh` с его целью.
5. **PR-4:** перенести `validate_honesty.sh` (+ починенный тест) и `gh-pr-create-via-gh-api.sh` / `push-via-gh-api.sh`.
6. **PR-5:** `git rm` остального `scripts/agent_flow/` и `.hermes/` по точным путям, с `grep` по репо на ноль ссылок.
7. **PR-6:** `CONTRIBUTING.md` и `.github/copilot-instructions.md` — убрать механику hermes; удалить файл `MAINTENANCE`.
8. **Метки:** удалить 30+ меток hermes-процесса (не PR, `gh label delete` — с «да» Шифу).

Каждый PR — свой CI-прогон; «удалено» — только с grep-доказательством (правило «не держать две реализации»).

## 7. Ждёт решения Шифу

- **В1. Что гасим первым?** (а) cron на build-хосте (§6.1) — рекомендую, это единственный возможный источник фонового шума; (б) метки на deploy-карточках (PR-1); (в) сразу снос `scripts/agent_flow/` одним пакетом PR-1…PR-6.
- **В2. Карточки #3374 и #3389** — закрыть как устаревшие со ссылкой на этот ADR? (рекомендую да).
- **В3. `validate_honesty.sh`** — (а) перенести и подключить soft-шагом к телу PR в CI; (б) перенести без CI, как ручной инструмент; (в) удалить вместе с hermes.
- **В4. Домен `AF-`** — заморозить (новые процессные ADR — в общую серию, как этот) или продолжать вести процессные ADR как `AF-0072…`?
- **В5. Красный `G: CC-Ref Remote Verify (nightly)`** — завести карточку на починку (#3291 закрыт, а baseline на него ссылается)?

## 8. Последствия

- Плюс: один документ процесса, AGENTS.md не обещает того, чего нет (triage, kanban, «мёржит только Шифу»).
- Плюс: 91 тыс. строк процессного кода перестают выглядеть живыми; агенты не тянут правила kanban в текущую работу.
- Минус: пока не пройдены PR-3…PR-5, два гарда G: Lint Code живут в папке с пометкой deprecated — удалять папку целиком нельзя.
- Риск (не проверено): если crontab на хосте жив и `MAINTENANCE` удалят раньше §6.1, автоматика может проснуться и начать создавать карточки/PR.
