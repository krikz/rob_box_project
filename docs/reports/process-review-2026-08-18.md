# Process Review — 2026-08-18

| Поле | Значение |
|------|----------|
| Документ | `docs/reports/process-review-2026-08-18.md` |
| Дата | 2026-08-18 |
| Автор | architect (Hermes Agent) |
| Источник запроса | issue #1404 («ревью процесса разработки rob_box») |
| Scope | документальный аудит + рекомендации. **Без правок кода/скриптов** |
| Период анализа | ~30 дней (2026-07-18 → 2026-08-18), фокус на последнюю неделю |
| Источники | CONTRIBUTING.md, AGENT_FLOW_PROPOSAL.md, CI_CD_PIPELINE.md, E2E_TESTING_DESIGN_v2.md, ADR-0010..0017, scripts/agent_flow/*.sh, scripts/agent_flow/tests/*.sh, `git log` 30d, open/closed issues #1380-#1403, retrospective notes (`docs/retros/`) |

---

## 1. Резюме

Процесс разработки rob_box за месяц **из хаотичного «коммитим и молимся» превратился в дисциплинированный конвейер** с тремя cron-скриптами (triage/merge-gate/e2e-process), каноническими ADR-ами (#0013-#0017) и 25 unit-тестами для самих скриптов. Главная инфраструктурная находка месяца — **e2e_verdict Single Source of Truth (ADR-0015)**: второй валидатор в workflow или агенте неизбежно вступает в конфликт с харнессом; правило «вердикт ровно в одном месте» формализовано и стало несущей конструкцией.

Главные проблемы — **тонкие race conditions между cron-ами** (один тик merge-gate, другой — e2e-process, оба пишут labels), **усталость документации** (SPEC_CURRENT за 27.07 — это месячной давности; README про процесс молчит), **размер процессных скриптов** (e2e-process 2359 строк, merge-gate 2759 — порог моно-скрипта пройден) и **узкие места в acceptance** (#1384 «scenario_file не передаётся → fallback на smoke-тест» — процессная верификация PASS без реального прогона была невозможна до #1387; #1391 «автозакрывалка закрывает user-reopen» — баг Q22/quarantine-логики).

**Чего не хватает:** AGENTS.md (правила для агентов, в работе #1397), явных правил «когда retrigger карточки допустим/недопустим» (Hermes-kanban status-transitions), формализованного process-мониторинга (метрики прохода по раундам, MTTR, false-positive rate автозакрывалки), и observability для самих cron-ов (логи лежат на 249, чтение с host-машины недоступно по текущему SOT — это риск).

---

## 2. Что работает (закрепить)

### 2.1. RТ-конвейер «triage → merge-gate → e2e-process» устойчив

- **triage** (645 строк, every 5m, pure bash) — работает детерминированно, идемпотентность через comment marker `kanban: t_<id>` (verified live #1048/#1049 в 08.08).
- **merge-gate** (2759 строк, every 5m, pure bash) — несмотря на размер, несёт главную ценность: единая точка транзиций label'ов (`needs-e2e`, `needs-review`, `e2e-done`) и post-merge cleanup. ADR-0014 (принят 11.08) перевёл жизненный цикл issue на инвариант «issue close iff `e2e-done` AND PR MERGED AND base=develop» — это сняло ручной шаг «закрыть issue после merge» и заодно убрало повторный триаж.
- **e2e-process** (2359 строк, every 1h, LLM-driven) — успешно переваривает по 30+ PR/неделю, артефакты (recording, ASR, RMS, diff, baseline) прикладываются даже на FAIL (PR #1382, fixes #1353). Round-ветки `z-{e2e}/test-round-N` создаются и ротируются.

### 2.2. ADR-практика стала реальным механизмом, а не «бумагой»

ADR-0013 (incremental over big-bang, 11.08) → правило «≤50 коммитов ИЛИ ≤3000 строк, иначе `big-bang-override`» в CONTRIBUTING.md. **Сработало:** PR #876 (100 коммитов, +7156 строк, 43 дня вне develop) закрыт 11.08, перенос портирован инкрементально через issue #1000.

ADR-0014 (e2e-done & merge-gate, 11.08) — реализован в `merge-gate.sh`, покрыт 8 acceptance criteriaми + 25 unit-тестами. Срабатывает на проде: 95+ issues автоматически закрыты с момента внедрения.

ADR-0015 (e2e verdict SOT, 12.08, **Proposed → wait for implementation**) — главная процессная починка. После фикса #1387 (scenario_file передаётся) и #1390 (revert gating-эксперимента #1386) процесс стал self-consistent: «e2e PASS» = «прогон по РЕАЛЬНОМУ сценарию PR» = «узел, имеющий право поставить `e2e-done` — ровно один».

ADR-0017 (zenoh SPOF, 15.08) — пример принятия «не делать» (variants A/B требовали пересборки, выбран C = observability). Это правильное решение: в lab среды SPOF приемлем, пока есть алерт.

### 2.3. Тестовое покрытие самих cron-скриптов — есть и растёт

25 unit-тестов в `scripts/agent_flow/tests/`:
- `test_merge_gate_*.sh` — 16 файлов, покрывают: big-bang, clean-pr-sweep, conflicting, deploy-reconcile, **duplicate_file** (ретро 15.08 t_20383d32), e2e-done-review, post-merge, pr-backfill, pr-orphan-needs-e2e, retro-card-archive, **retro_path** (12/12), stale-branch.
- `test_e2e_process_*.sh` — guard (42 KB, годно), deploy-recovery.
- `test_agents_sleep.sh`, `test_kanban_retro_create.sh`, `test_watchdog_*.sh`, `test_install_*.sh`, `test_drift_detect_branch_active.sh`, `test_vendor_patch_apply.sh`.

Mock-freemwork `tests/lib/mock_env.sh` — позволяет прогонять тесты без реального gh/GitHub. Это **очень правильная инвестиция** — без неё скрипты в 2.5K строк давно бы сломались.

### 2.4. Воркеры коммитят WIP и пишут в issue

С момента ретро 09.08 #2 (t_9435a3c5, t_0c0a98ac) — паттерн «WIP-коммиты каждые ~15-20 мин, push сразу» усвоен. В логе месяца: 25+ `wip(*):` коммитов, ни одной потери незакоммиченной работы. Контракт воркера явно прописан в шаблоне карточки и в `AGENT_FLOW_PROPOSAL.md` §3.1.

### 2.5. Процедура post-merge cleanup

ADR-0014 + merge-gate: после merge в `develop` → close issue → archive card → delete remote branch → cleanup worktree. Race-safe (повторный tick безопасен: closed issue не входит в `--state open`). Покрыто 8 acceptance criteriaми.

### 2.6. Out-of-band детект дублей (PR #1262/#1267)

`duplicate_file_scan_all()` в merge-gate постит **инфо-коммент на ОБА PR** если `(filename, sha)` совпадает в ≥2 открытых PR. Решение — за Шифу (какой влить, какой закрыть), guard не блокирует CI. Покрыто 5 unit-тестами. Хороший пример «радар для человека, а не автоматическое решение».

---

## 3. Что сломано (5–10 пунктов, fix)

### 3.1. Race: merge-gate и e2e-process одновременно пишут labels

**Симптом:** #1391 (закрыто 18.08): автозакрывалка (merge-gate) закрывает issue через 6 мин после user-reopen. #1395 (OPEN 18.08): PR помечен `e2e-done`+`needs-review`, но **реальный прогон не запускался** — e2e-process подхватил PR по label `needs-e2e`, merge-gate поставил `e2e-done` по другому tripwire'у.

**Корень:** оба cron-а независимо читают/пишут `needs-e2e`/`needs-review`/`e2e-done`. Между ними нет mutex'а на уровне GitHub API. ADR-0014 (race `merge → label позже`) формализовал повторное чтение labels перед close, но **общего lockfile между двумя cron-ами нет**.

**Fix (medium):**
- Ввести `state/process.lock` (flock 200) внутри merge-gate И e2e-process на секцию «read labels → decide → write labels». Это не защитит от race на GH API, но **сузит окно** и сделает поведение воспроизводимым в dry-run.
- Альтернатива (ADR-уровень): ADR-0019 «single-writer-per-issue» — каждая issue имеет уникальный writer (e2e-process или merge-gate), определяемый первым state-transition. Это сложнее, но чище.

**Кто:** devops (flock) или architect (ADR-0019).

### 3.2. e2e verdict легко подделать, если не проверить весь pipeline

#1384 (закрыт) — `agent-flow-e2e-process не передаёт scenario_file` → прогоняет «спой про енотика» (smoke) вместо `music_library_suite_v1.json`. PASS формально, но это **не тот тест, который нужен был PR**. ADR-0015 это формализует, но практика показала: один-единственный параметр в `gh workflow run -f` мог свести всю верификацию к фарсу.

**Текущее состояние:** #1387 (merged 18.08) — auto-discover scenario_file из PR files. PASS теперь требует, чтобы файл сценария существовал в PR. **Но нет unit-теста на guard: «если auto-discover не нашёл — e2e FAIL, а не fallback smoke»**. Это структурная дыра.

**Fix (medium):**
- Добавить unit-тест `test_e2e_process_scenario_required.sh` — сценарий: PR без `.github/e2e/scenarios/*.json` файла → e2e-process **должен** ставить `e2e:rejected` + комментарий «scenario_file required, none found in PR», **не** fallback на smoke.
- Сделать `gh workflow run -f scenario_file=...` обязательным параметром (exit 1, если пустой) — это второй уровень защиты.

**Кто:** devops (тест).

### 3.3. Автозакрывалка и quarantine логика не Brother-Locked

**#1391:** автозакрывалка закрывает issue через 6 мин после user-reopen. Решение (#1399, merged 18.08) — user-reopen guard. **Но:**
- Guard появился только 18.08, а проблема была с 16.08. **Не было alert'а**: «autocloser закрыл X issue, которые были re-opened за последние 24h».
- Тригер перезакрытия основан на тексте «user-reopen» в комментариях — если юзер откроет issue без явного комментария, guard не сработает.

**Fix (low):**
- Cron-аудит на 249: раз в час проверять список `closed issue` за последние 24h, грепать на `reopened` events, и при наличии → алерт в Telegram Шифу с перечнем.
- Guard: если issue имеет label `user-touched` (ставится при любом user-action), merge-gate НЕ закрывает такой issue в этом tick'е.

**Кто:** devops (аудит) + reviewer (label `user-touched`).

### 3.4. Процессные скрипты переросли порог моно-скрипта

`agent-flow-e2e-process.sh` 2359 строк, `agent-flow-merge-gate.sh` 2759 строк. **Признаки переростка:**
- grep по именам функций даёт десятки hits, half of them — short helpers (`log`, `run`, `has_label`, `slugify`).
- Mixed responsibilities: merge-gate отвечает и за label-transitions, и за cleanup артефактов, и за archive kanban-карточек, и за drift-detect side-effects.
- Одна ошибка в логике дублирования файлов — ретро 15.08 t_20383d32 — вылилась в 5 тестов и тёзку ADR; следующая ошибка потребует сравнимых усилий, потому что код не читается.

**Trade-off:**
- Split (e.g. `merge-gate-labels.sh`, `merge-gate-cleanup.sh`, `merge-gate-archive.sh`) — больше файлов, но тестируемость ↑↑. Требует рефактора + перезаписи тестов.
- Оставить — быстрее сейчас, но техдолг растёт линейно, и каждое ретро тратит первые N минут на «найти, где сидит».

**Fix (medium, можно M2):** разбить merge-gate на 3 модуля по transition: `merge-gate-classify.sh` (только решает что делать, dry-run), `merge-gate-execute.sh` (только side effects), `merge-gate-cleanup.sh` (архив/cleanup). Это M2 — после стабилизации #1401/#1391/#1398.

**Кто:** architect (дизайн) + devops (имплементация).

### 3.5. Документация отстаёт от процессов

- **AGENTS.md** отсутствует (issue #1397 в работе, PR #1402 OPEN). Все правила для воркеров — рассыпаны по CONTRIBUTING.md, ADR-0014, ADR-0015, шаблону карточки. Это **anti-pattern** — единого source-of-truth нет.
- **SPEC_CURRENT.md** датирован 27.07, описывает `feature/harness-p0-foundation` (PR #907). Эта ветка давно вмёржена; документ мёртв. Путает воркеров: им приходится гуглить, что актуально.
- **README.md** не упоминает agent-flow process — новичок читает README и не знает, что есть triage/merge-gate/e2e-process.
- **CI_CD_PIPELINE.md** (36K) и **E2E_TESTING_DESIGN_v2.md** (51K) — большие, не проверены на актуальность (e2e-process на сейчас уже отличается от дизайна: rotation/round N+1, scrollback, idempotency layers ADR-0014/0015 — это всё новые слои поверх оригинального дизайна).

**Fix (low, инкрементально):**
- Landить #1397 (AGENTS.md + .cursorrules) — **highest priority**.
- SPEC_CURRENT.md: заменить на короткий pointer «прошло 2 месяца, планка P0 закрыта; актуальное в ROADMAP.md/PRD.md/AGENTS.md». Ретро #1104 в docs/adr/0014 уже показывает, что такие «snapshot-документы» устаревают за 2-3 недели.
- README.md: добавить секцию «## Agent-driven development» с одной ссылкой на AGENT_FLOW_PROPOSAL.md.
- CI_CD_PIPELINE.md / E2E_TESTING_DESIGN_v2.md: добавить в начало «Last verified: <date>» + «Deltas from design: <ADR-XXXX>». Сейчас невозможно понять, что изменилось.

**Кто:** architect (SPEC_CURRENT, README, CI_CD_PIPELINE/E2E DESIGN diff-секции) + reviewer (AGENTS.md через #1397).

### 3.6. «Honesty culture» формализована только в одном месте

ADR-0018 (Proposed, в работе #1397) — «честный FAIL лучше красивого PASS». PR #1402 содержит implementation. **Но:**
- `validate_honesty.sh` (скрипт-валидатор) — проверяет, что коммит/PR содержит raw evidence. **Где он крутится?** Сейчас — только ручной запуск перед merge. Это слабо.
- Cron-аналога нет: «каждое утро Шифу видит список PR за 24h, у которых нет `Evidence:` блока в комментариях».

**Fix (low):**
- Pre-merge gate: добавить `validate_honesty.sh` как обязательную проверку в `G-Run Tests.yml` (или `big-bang-override` PR). Не пускать PR с `needs-review` без evidence.
- Daily summary cron: 1 раз в 7:00 MSK — список PR за 24h, у которых `needs-review` без `# Evidence:` блока → Telegram Шифу.

**Кто:** architect (ADR-0018 close) + devops (CI gating).

### 3.7. Hermes-kanban: re-triggers правила не формализованы

В task context есть старые ретро: `#1342` (resume cron → backfill-скан), `t_238ff3f7` (deploy-reconcile), `t_5cf0162b` (PR-side orphan reconcile). Все три — случаи, когда **карточка лежала в wrong state**, и пришлось писать ОТДЕЛЬНЫЙ cron-scan для лечения. Это **симптом**: state machine карточки не описана формально, и каждое новое состояние добавляет новый scan.

**Fix (medium):**
- Сделать одну таблицу: `state × trigger → action` для всех 9+ возможных комбинаций (`ready/running/blocked/done` × `ci/red/green/e2e/red/green/merge/reopen/...`). Это будет ADR-0020.
- Реализовать: единый `state-reconciler.sh` cron, который читает эту таблицу и применяет. Все три backfill-скрипта (`pr-backfill`, `deploy-reconcile`, `pr-orphan-needs-e2e`) — переписать как выходы reconciler'а.

**Кто:** architect (таблица + ADR-0020) + devops (reconciler).

### 3.8. observability самих cron-ов — дыра

Cron-логи лежат на 249 в `~/.hermes/profiles/*/cron/output/`. С host-машины (та, где я работаю) — **не доступны** (я проверял: `ls ~/.hermes/profiles/architect/cron/output/` → пусто). Это значит:
- Если cron падает с exit 127 (`kanban: command not found`), Шифу узнаёт только когда карточка «зависла».
- Нет общего dashboard: «какие cron-ы зелёные за последние 24h, какие красные».

**Fix (medium):**
- Скопировать `~/.hermes/profiles/*/cron/output/` на host (или настроить sshfs). Минимальный SOT — `last_status` каждого cron-джеба, опрашиваемый с 249 по SSH.
- Имплементировать `cron_health.sh` (опционально — в watch dog): раз в 30 минут проверять `last_status` всех agent-flow джебов, алерт в Telegram при 2+ подряд красных.

**Кто:** devops.

### 3.9. e2e-сессии не изолированы между собой

**#1385/#1386 (revert в #1390):** воркер-попытка сделать gating через `/voice/e2e/busy` topic в dialogue_node. **Структурная ошибка** — домен voice runtime не должен знать о CI/CD. Revert правильный. **Но:** реальная проблема (e2e на работающем роботе слышит соседний трафик) — не решена. Решение = физически отдельный robot instance (на labs Docker pool) или **mute-период** через `ros2 topic pub /voice/e2e/mute` (но это тоже runtime, не dialogue_node).

**Fix (architect, ADR-0021):**
- Сформулировать: «e2e = изолированный прогон на Labs Docker pool, не на prod-роботе». Тогда «gating» не нужен — тестовое окружение само по себе изолировано.
- Если labs pool нет — короткий вариант: `e2e:start` announce через ntfy / Telegram, чтобы оператор знал «сейчас на роботе идёт e2e, не трогай 5 мин».

**Кто:** architect (ADR-0021).

### 3.10. `--no-ff` merge в test-round-N — потенциальная мина

В `e2e-process.sh` step 5d: «Merge agent branch DIRECTLY into `z-{e2e}/test-round-N` (--no-ff, no wip layer). Resolve trivial conflicts with `-X theirs` is intentionally NOT done — we let the merge fail and mark `e2e:rejected` (manual).»

**Что может пойти не так:**
- Если 2 PR с `needs-e2e` мерджатся параллельно (не по факту, но в одном тике), merge конфликтует → весь раунд откатывается → e2e пропущен.
- ADR-0014 защищает только финальный merge в develop. Промежуточный merge в test-round-N никак не защищён.

**Fix (low):**
- В тике e2e-process мерджить PR в test-round-N **последовательно** (sort by PR number, after each merge `git push origin test-round-N`). Это уже так (документация says sequential), но **unit-теста на два PR одновременно нет**.
- Добавить `test_e2e_process_conflicting_merges.sh` — мок: 2 PR, попытка merge второго → merge conflict → второй PR помечается `e2e:rejected`, первый сохраняется.

**Кто:** devops.

---

## 4. Что устарело (3–5 пунктов, обновить)

### 4.1. `scripts/agent_flow/agent-flow-e2e-process.sh` — header устарел

Строки 1-13 говорят «SOT (source-of-truth)», но реальный SOT — уже целая система (ADR-0014, ADR-0015, drift-detect, post-merge reconcile). Header не упоминает **где** сейчас реальная спецификация — отсылает к самому себе.

**Fix:** переписать header как стандартное объявление: «Sync с develop: `bash install.sh`; design: `AGENT_FLOW_PROPOSAL.md` §3.4 + ADR-0014 + ADR-0015».

### 4.2. Phase 3 mini / Phase 4 статус в AGENT_FLOW_PROPOSAL.md §7

Phase 3 mini всё ещё «[ ]» (не реализовано), хотя фактически работает с 12.08. Phase 4 (multi-model matrix, dashboard, auto-merge) — auto-merge запрещён Q5, остальное — не в фокусе. **Стоит обновить чекбоксы** и удалить Phase 4 как «out of scope for MVP».

### 4.3. SPEC_CURRENT.md — мёртв

Датирован 27.07. Ветка `feature/harness-p0-foundation` вмёржена. Все «этапы» в §1.1 — done; нового — нет. **Лучшее:** переименовать в `ARCHIVE/p0_snapshot_2026-07-27.md` (или удалить). Актуальный source-of-truth: `ROADMAP.md` + `AGENTS.md` (после #1397).

### 4.4. README.md не упоминает agent-flow

`README.md` описывает проект, быстрый старт, документацию — нигде нет «## Разработка» или «## Agent-driven process». Новичок/воркер, открывающий README, не знает, что есть AGENT_FLOW_PROPOSAL.md и как работает конвейер.

### 4.5. CI_CD_PIPELINE.md, E2E_TESTING_DESIGN_v2.md — нет «Last verified»

36K + 51K документов без даты «last verified». Воркеры тратят время, чтобы понять «это ещё актуально или нет». Добавить в начало 2 строки: `Last verified: <date> by <who>`. Если больше месяца — `[STALE]` watermark.

---

## 5. Предложения (5–10 пунктов, roadmap)

### 5.1. ADR-0019 (race single-writer-per-issue)

Зафиксировать правило: «у одной issue в каждый момент ОДИН writer (e2e-process или merge-gate)». Определяется первым state-transition. mutex через `state/issue-writer.json` (per-issue lock). Это сделает поведение детерминированным.

**T-shirt:** M. **Приоритет:** medium. **Когда:** после #1391/#1395 close (через 1 релизный цикл).

### 5.2. ADR-0020 (state machine карточки — единый reconciler)

Таблица `state × trigger → action`. Реализация: `state-reconciler.sh` cron. Заменяет 3 ad-hoc backfill-скрипта (`pr-backfill`, `deploy-reconcile`, `pr-orphan-needs-e2e`).

**T-shirt:** L. **Приоритет:** high. **Когда:** M2. **Сейчас:** подготовить таблицу как draft, обсудить с Шифу.

### 5.3. ADR-0018 formal close

ADR-0018 (honesty FAIL) — **Proposed**, хочется Accepted. После #1397 merge — добавить § «Acceptance» (pre-merge gate + daily summary cron).

**T-shirt:** S. **Приоритет:** high. **Когда:** ASAP после #1397 merge.

### 5.4. SPEC_CURRENT.md → ARCHIVE

Превратить snapshot-документ в архивный. Актуальное — `ROADMAP.md` + `AGENTS.md` после #1397.

**T-shirt:** XS. **Приоритет:** low. **Когда:** в этом PR.

### 5.5. cron-observability — sshfs / health probe

Смонтировать `~/.hermes/profiles/*/cron/output/` на host (или настроить `cron_health.sh`). Цель: «за 30 секунд увидеть, что cron-красный».

**T-shirt:** S. **Приоритет:** medium. **Когда:** M2.

### 5.6. e2e: scenario_file required — guard тест

Добавить `test_e2e_process_scenario_required.sh` — «PR без scenario_file → e2e:rejected, НЕ fallback smoke». Это закроет класс багов #1384.

**T-shirt:** S. **Приоритет:** high. **Когда:** ASAP.

### 5.7. Split `merge-gate.sh` на 3 модуля

`merge-gate-classify.sh` (dry-run, decide), `merge-gate-execute.sh` (side effects), `merge-gate-cleanup.sh` (archive). Требует рефактор + переписать тесты.

**T-shirt:** L. **Приоритет:** low. **Когда:** M2 (когда стабилизируется основной поток).

### 5.8. README — секция «Agent-driven development»

5-7 строк: что есть AGENT_FLOW_PROPOSAL.md, как работает конвейер, ссылка на AGENT_FLOW_PROPOSAL.md.

**T-shirt:** XS. **Приоритет:** low. **Когда:** в этом PR.

### 5.9. PR-side labels reconciliation — единый cron

Сейчас 3+ скрипта независимо пишут `needs-review`/`needs-e2e`/`e2e-done`. Свести в ОДИН transitor, с таблицей (как ADR-0020). Это **часть ADR-0020**, выношу отдельным пунктом для акцента.

### 5.10. big-bang override → ADR-0022

Сейчас метка `big-bang-override` — это exception, который ставит **Шифу лично**. Но **нет автоматической проверки**: «PR > 50 коммитов ИЛИ > 3000 строк без override → блокируем merge». Реализовать как pre-merge check в `G-Run Tests.yml`.

**T-shirt:** S. **Приоритет:** medium. **Когда:** M2.

---

## 6. Ссылки

- Issue #1404 — текущая задача
- Issue/PR #1397 — AGENTS.md (честный FAIL), в работе
- Issue/PR #1391/#1399 — user-reopen guard
- Issue/PR #1384/#1385/#1386/#1387 — scenario_file saga
- Issue/PR #1388/#1394 — race-safe push-image-versions
- Issue/PR #1389/#1393 — defensive SSoT for `_llm_skipped_counter`
- ADR-0013 — incremental over big-bang
- ADR-0014 — e2e-done & merge-gate
- ADR-0015 — e2e verdict SOT (Proposed)
- ADR-0017 — zenoh SPOF (M2)
- AGENT_FLOW_PROPOSAL.md §3.1-3.5 — контракты cron-ов
- CONTRIBUTING.md — правила (Q22, big-bang, needs-review)
- `docs/retros/2026-08-12-t_bedc87e6-pr1080-audit.md`
- `docs/retros/2026-08-15-t_20383d32-dupe-teleop-setup-cfg.md`

---

## 7. Что НЕ входит в это ревью

- **Воркеры/агенты (код в `src/`, `tools/`, `docker/`)** — это SCOPE бизнес-фич, не процесс. Баги ловятся в своих ретро.
- **Метрики тестов** (`pytest`, coverage) — out of scope (CI процесс, не процесс разработки).
- **Hardware / Telegram bot / TTS chain** — это доменные компоненты, не процесс.
- **Тюнинг конкретных cron-интервалов** (5m/1h) — нет данных для принятия решения; нужны замеры.

