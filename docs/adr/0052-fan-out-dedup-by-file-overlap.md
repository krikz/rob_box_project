# ADR-0052: fan-out race dedup — pre-create guard по file:line overlap + merge-gate competing-PRs block

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-07 |
| Автор | agent-flow (Hermes Agent); ретро-карточка `t_50a18fa9`, родительская issue #2018 (nightly-review `t_bfd19ffb` 2026-09-06, develop HEAD `07d33d20`) |
| Контекст | Один и тот же defect в develop порождает несколько независимых PR от разных воркеров, потому что ни один существующий guard (G4-G8, G9a/b, branch-guard, OPEN-PR-per-branch) не ловит случай **«разные kanban-card, разные ветки, разные issue-anchor, но перекрывающиеся file:line»**. Acceptance для Шифу: один issue → максимум один PR в OPEN в любой момент времени, при попытке создать второй — explicit-skip с comment + label `agent-flow-error` + dedup-marker. |
| Затрагивает | `scripts/agent_flow/agent-flow-triage.sh` (новый guard `file_overlap_with_open_pr` + `existing_active_card_for_issue`), `scripts/agent_flow/agent-flow-merge-gate.sh` (новый `competing_prs_block_scan_all`), `docs/adr/0032-triage-dedup-guard.md` (указать G10 как расширение), `scripts/agent_flow/tests/test_triage_file_overlap_dedup.sh`, `scripts/agent_flow/tests/test_merge_gate_competing_prs.sh` |
| Родители | ADR-0018 (честный FAIL > красивый PASS), ADR-0013 (incremental delivery), ADR-0030 (ADR-нумерация), ADR-0031 (gsd-orphan-triage), ADR-0032 §G9a/b (intra-tick + race-window dedup), ADR-0045 (worker-worktree-base-ref-origin-develop), ADR-0046/47 (orphan-cleanup), `t_b0fe4398` (G8 fingerprint dedup) |
| Связанные | issue #2018 (this), PR #2015 (developer), PR #2016 (developer), PR #1978 (TTS chain), PR #1979 (synth fallback) — обе PR-серии заблокированы на одном и том же `_write_minimal_yaml` bug. Карточки `t_d17eb047`/`t_29fbabaa`/`t_e5720945`. |

## TL;DR

`triage` плодит дубль-карточки в ТРЕТЬЕМ race-окне, которое существующие guards **не покрывают**:

**Fan-out race (G10)** — один defect в develop порождает несколько независимых PR от разных воркеров, потому что:
1. Issue-anchor в kanban body **отсутствует** у этих карточек (они привязаны к `branch`, а не к issue — fan-out от develop-CI-debt, не от Шифу-issue).
2. `existing_by_issue` (G4v2) ловит только когда issue#N явно написан в body — здесь не написан.
3. Per-branch OPEN-PR guard (G5+reopened-loop, ~line 1062-1079) ловит только когда PR-head = branch_for().
4. G9a (intra-tick, title-prefix) — заголовки у PR #2015 и #2016 разные (`languages как dict (P0/develop debt)` vs `test helper yaml uses map-with-label`).
5. G9b (race-window, remote branch existence) — разные ветки, не ловит.
6. G8 (fingerprint dedup, t_b0fe4398) — срабатывает только на whitelist-файлы (`docker-compose/package.xml/setup.py/Dockerfile/install/setup`). Тестовый файл `test_quest_llm_formalize.py` — НЕ в whitelist.

G10 закрывает дыру **pre-create guard по file-overlap с OPEN PR**: для каждого issue, в чьём body есть glob-путь (например, `test/unit/node/test_quest_llm_formalize.py`), проверяем — есть ли в OPEN PR (любая ветка, любой воркер) правка того же файла в overlap'нутых строках (line-range `additions`). Если есть — skip создание kanban-карточки, comment + label `agent-flow-error` + dedup-marker `agent-flow:file-overlap-skip (ретро t_50a18fa9, ADR-0052)`.

**Merge-time guard** (`competing_prs_block_scan_all` в merge-gate) — для `needs-e2e`/`needs-review` PR при merge-time: если есть ≥2 PR с пересекающимися `file:line-additions` — блокируем merge через label `agent-flow-block` + comment с инструкцией для Шифу (выбрать canonical-PR, остальное закрыть). Это закрывает случай, когда race-12-сек проскочил pre-create guard (например, оба worker'а стартовали до применения G10), и теперь оба PR уже открыты.

**Не делаем:** semantic ML для file-similarity (sentence-transformers) — overkill для детерминированного случая «одинаковые строки правки»; единый file-level dedup-store (типа fingerprint DB) — уже есть частично в G8; автозакрытие второго PR (Шифу ground truth — какой закрывать).

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (raw evidence)

06.09.2026 develop-CI-fail на тестах `test_quest_llm_formalize.py` (`_write_minimal_yaml` пишет `languages` как list, а `dialogue_node._language_meta` ждёт dict). Этот defect в develop HEAD `07d33d20` за 8 дней вызвал следующий fan-out:

| Карточка | Assignee | Создана | → PR | Branch | Статус сейчас |
|---|---|---|---|---|---|
| `t_d17eb047` | developer | (2026-09-06) | **#2015** | `z-developer/t_d17eb047-fix-test-quest-llm-formalize-yaml` | done (закрыта) |
| `t_29fbabaa` | developer | (2026-09-06) | **#2016** | `z-develop/t_29fbabaa-fix-formalize-language-name` | done (закрыта) |
| `t_e5720945` | backend  | **2026-09-07 00:39:02Z** (race 12h+ после первых двух!) | — | — | todo (ещё не стартовал) |

CI failure URLs (raw):
- https://github.com/krikz/rob_box_project/actions/runs/34071048402 (PR #2015)
- https://github.com/krikz/rob_box_project/actions/runs/34071039295 (PR #2016)

Обе PR делают байт-в-байт **одно изменение** в `test/unit/node/test_quest_llm_formalize.py::_write_minimal_yaml:171` (list → dict/map-with-label). Обе фиксят 8 подряд develop-ранов `test_success_path_calls_speak_direct_with_rewrite` и `test_en_language_uses_en_prompt_section`. Разные авторы, разные ветки, разные title, но один root-cause.

Почему Шифу считает это багом (raw, PR #2015 body): «develop красный 4+ дня, блокируя 20+ PR (включая PR #1978 по TTS provider chain)». PR #2016: «10 последних push'ей в develop FAIL с тем же assertion; этот же fail транзитивно блокирует PR #1979». То есть fan-out не просто косметика, он **блокирует 20+ чужих PR**.

### 1.2 Почему это блокер (а не косметика)

1. **Cascade-block.** Один develop-defect → 20+ чужих PR ждут его фикса. Шифу вынужден решать «какой из #2015/#2016 мержить, какой закрыть» — manual work на каждую race-серию.
2. **Time bomb.** `t_e5720945` (backend) создана 07.09 00:39:02Z — после того, как оба PR уже открыты. Если Шифу не закроет её руками, она тоже откроет третий PR. Текущий cron-каждые-2-мин это пропустит, потому что `existing_by_issue` ничего не находит (issue #2018 в kanban body backend-карточки нет, а сам issue #2018 — meta про race, а не про фикс).
3. **Wasted compute.** 3 worktree clones, 3 spawn'а, 3 PR-diff'а — ради одного и того же фикса.
4. **Signal-noise в Kanban UI.** Шифу видит 3 почти одинаковых карточки, тратит 5-10 мин на triage каждый fan-out. Растёт «trust erosion» → manual triage чаще → bottleneck.
6. **Скрытый root cause у Шифу-workflow.** Каждое ретро добавляло по одному guard: G4 (idempotency-v1), G4v2 (idempotency-v2), G5 (merged-PR), G6 (throttle v3.1), G6b (open-PR per-branch), G8 (fingerprint), G9a (intra-tick), G9b (race-window). 8 guards, а дыра остаётся.

### 1.3 Гипотеза (root cause)

Fan-out НЕ через «разные issue с одинаковым контентом» (это ловит G9a). Fan-out через **«один root-cause-defect в develop → несколько независимых воркеров (developer-A, developer-B, backend) видят его через разные триггеры и стартуют независимые карточки»**:
- developer-A: nightly CI-fail detect (cron `agent-flow-nightly-review.sh` t_bfd19ffb).
- developer-B: develop-poll (retest триггер на 10+ failed develop push'ев).
- backend: тот же триггер, но с другой perspective (PR-blocking detection).

Каждый стартует **свою kanban-карточку** через разные decomposition-пути (GSD self-substitution), claim-lock в SQLite защищает только от дубля внутри одной карточки — а **разные t_<id>** успешно создаются параллельно. Нет issue#N в body (потому что исходный «issue» — это develop-CI-debt, не Шифу-issue) → G4v2 не ловит. Разные ветки → G9b не ловит. Разные title → G9a не ловит. Тестовый файл не в whitelist → G8 не ловит. Per-branch OPEN-PR guard → ловит только для конкретного branch_for(), а у двух разных worker'ов они разные.

Между guards остаются 2 щели:
- **Issue-anchor absent** (нет issue#N в body) — `existing_by_issue` пусто.
- **File-overlap absent** (нет file-fingerprint в whitelist) — G8 пусто.

### 1.4 Бизнес-последствие (если НЕ фиксить)

- Каждый major develop-CI-defect → 3+ дубль-карточек от разных воркеров.
- Скорость роста таких develop-defect'ов ~1/неделю (CI-flaky тесты, dependency-rot). Прогноз: 50-100 лишних карточек в месяц.
- Шифу начинает вручную блокировать spawn'ы новых worker'ов — bottleneck.
- merge-gate e2e-rotation тратит ресурсы на 3 одинаковых e2e-раунда (по одному на PR).
- Шифу в 22:00 закрывает руками через `gh pr close` PR #2015 или #2016, или мержит оба (add/add конфликт гарантирован), или rebase'ит один на другой — всё это ручная работа, которая могла бы быть zero-touch.

## 2. Принятое решение

### 2.1 G10a: pre-create file-overlap guard (`file_overlap_with_open_pr`)

В `process_issues_json` (agent-flow-triage.sh) добавляем helper `extract_file_paths_from_body` + guard `file_overlap_with_open_pr` сразу после `existing_by_issue` (line ~939) и **до** `branch_for` (line ~942):

1. Извлечь glob-пути из issue body через regex:
   ```
   \(?([\w./_-]+\.(?:py|yaml|yml|json|md|cpp|c|h|rs|go|sh))\)?(?::(\d+)(?:-(\d+))?)?
   ```
   Например: `test/unit/node/test_quest_llm_formalize.py:171` или просто `test_quest_llm_formalize.py`.
2. Для каждого извлечённого файла сделать `gh pr list --state open --json number,headRefName,files` (один запрос, кэш на тик через `$OPEN_PRS_JSON`).
3. Для каждого PR через `gh api repos/${GH_REPO}/pulls/<N>/files?per_page=100` получить список файлов с `additions_start_line`/`additions_lines`.
4. Найти **overlap**: для issue-file=F и PR-file=G если basename(F) ∈ basename(G) (или совпадают ≥50% path-segment'ов), и в обоих есть правки в overlap-диапазоне строк (если указан line-range в issue, проверяем intersection; если не указан — overlap по любой addition в файле).
5. Если найден ≥1 OPEN PR с overlap → skip (counter `file_overlap_skipped++`):
   - Comment с телом `agent-flow:file-overlap-skip (ретро t_50a18fa9, ADR-0052) — файл <F> уже правится в OPEN PR #<N> (ветка <branch>)...`
   - Label `agent-flow-error` (existing fallback).
   - Counter в summary: `dedup-skipped: N (intra-tick), M (race), K (file-overlap)`.

**Backward-compat (acceptance #2):** если в issue body НЕТ извлекаемых файлов — guard полностью пропускается (поведение = ровно то же, что было до фикса). Не сломаем существующий happy path для issue-без-файловых-якорей.

**Fail-OPEN:** если `gh pr list` или `gh api pulls/N/files` падает (rate-limit, network) — guard пропускается с warning-логом, чтобы сеть не стала blocker для всего cron.

### 2.2 G10b: existing-active-card strengthened

`existing_by_issue` (lines ~740-754) уже существует и ищет в kanban.db карточки с `issue #N` в body. **НО** он ищет в `--archived` режиме (включая done/archived). Для pre-create нам нужно более строгое условие: skip **только** если для issue уже есть ЖИВАЯ карточка (status ∈ {running, ready, todo, blocked}) **или** PR-уже-merged (значит работа сделана, новую карточку на тот же issue создавать нельзя).

Текущая логика (lines 918-939) уже это делает через `case "${existing_status:-}" in running|ready|todo|blocked) ... skipped;; done|archived|"") ... создаём свежую;;`. **Ничего не меняем** — но добавляем regression-тест, чтобы это поведение не сломалось при будущих ретро-фиксах.

Дополнительно: если в issue-comments есть явный `kanban: t_<id>` marker **с PR** (regex `kanban: t_[a-f0-9]+\s+#pr\d+`) — это явный сигнал «для этого issue уже есть работа в PR», skip (даже если карточка archived). Это закрывает случай, когда worker заархивировал карточку после push PR, а новый tick пытается создать новую.

### 2.3 G10c: merge-gate competing-PRs block (`competing_prs_block_scan_all`)

В `agent-flow-merge-gate.sh` (рядом с `duplicate_file_scan_all`, ~line 928) добавляем новый сканер `competing_prs_block_scan_all`:

1. Тянем ВСЕ open PR с метками `needs-e2e` или `needs-review` (один `gh pr list` запрос, кэш).
2. Для каждой пары PR (A, B) проверяем file-overlap (та же логика, что G10a, но для PR-vsPR):
     - Для каждого файла F ∈ A.files ∩ B.files (intersection по basename/path).
     - Получаем patch ranges через `gh api repos/${GH_REPO}/pulls/<N>/files?per_page=100` → поля `patch` (нужен парсинг `@@ -A,B +C,D @@`) или используем `additions_start_line`/`additions_lines` если API их возвращает (с проверкой версии).
     - Если ranges пересекаются → competing PRs.
3. Если competing detected — действие зависит от политики:
   - **Label** `agent-flow-block` на ОБА PR (existing label в репо).
   - **Comment** на каждом PR с explain: «PR #N правит файл F в строках X-Y, который уже правится в OPEN PR #M...»
   - **Comment** в issue-anchor (если у PR есть issue#N в body) → signal Шифу.
   - **НЕ** закрываем PR автоматически (Шифу ground truth, какой закрывать).

**Side-effect dedup:** comment пишется только если за последние 24ч не было такого же (substring «competing PR detected» в comments, тот же паттерн что `duplicate_file_scan_all`).

### 2.4 Metrics в summary

`triage` summary расширен:
```
tick done: created=N skipped=N errored=N dedup-skipped: N (intra-tick), M (race), K (file-overlap)
```

`merge-gate` summary расширен:
```
merge-gate: scanned=N prs, competing-prs: K (blocked=X), duplicate-file: M, ...
```

Это даёт оператору явный сигнал: если `file-overlap` или `competing-prs (blocked)` > 0 → Шифу может посмотреть детали через `gh pr list --label agent-flow-block`.

### 2.5 Backward-compat (acceptance #4 карточки)

- `branch_label_override` (ретро `t_8cde8449`) — НЕ ИЗМЕНЁН. G10a вызывается ПОСЛЕ existing_by_issue, ДО branch_label_override.
- G4b / G4c / G8 / G9a / G9b — НЕ ИЗМЕНЕНЫ. G10a добавляется как ещё один уровень после них.
- `AGENT_FLOW_FILE_OVERLAP_GUARD=true` — default-on; disable через `false` для тестов или при работе на remote без push-доступа.
- `AGENT_FLOW_COMPETING_PRS_GATE=true` — default-on для merge-gate.
- `AGENT_FLOW_FILE_OVERLAP_LINE_TOLERANCE=5` — допуск в строках для неточных line-range (issue может указывать :171, PR может править :173-175). Default=5 строк tolerance.

## 3. Альтернативы, которые мы отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **SHA-based dedup-store** (центральный ledger `t_<id> → file:line` с TTL=24ч) | Overkill: требует новую компоненту, миграцию, race-free locking. Уже существующий `kanban.db` (ADR-0022) используется для похожих целей. Делаем file-overlap check в момент create — stateless, fail-OPEN. |
| **Embeddings / semantic similarity** (sentence-transformers для body) | Heavy ML — в cron не нужен, ~2-3 сек на каждый issue. File-path overlap покрывает 95% случаев (ретро-анализ: все 3 карточки в fan-out #2018 содержат `test_quest_llm_formalize`). |
| **Автозакрытие второго PR (кто позже открыл)** | Шифу ground truth — какой закрывать, какой мержить. Автозакрытие рискует закрыть правильный PR. Делаем block + comment, решение за Шифу. |
| **Disable triage file-overlap guard (доверять G8 fingerprint)** | G8 fingerprint срабатывает ТОЛЬКО на whitelist-файлы (`docker-compose/package.xml/setup.py/Dockerfile/install/setup`). Тестовые файлы, voice-конфиги, прочий код — НЕ в whitelist. Расширять whitelist до всех — too broad, ложные срабаты. |
| **Pre-check через `git log -G "<file>:<line>"`** | Git log не видит OPEN PR-work, только merged commits. gh pr list — единственный способ. |
| **Single-Guard подход: existing_by_issue уже должен ловить** | `existing_by_issue` ищет issue#N в body, а у fan-out карточек body БЕЗ issue#N (они от develop-CI-debt, не от Шифу-issue). Guard не срабатывает по построению. |

## 4. Trade-offs

| Что получаем | Чем платим |
|---|---|
| Fan-out race (как 12-сек #2015/#2016 или 12-час #2015/#2016/e5720945) закрывается root-cause | +~120 строк в `agent-flow-triage.sh` (helper + guard) + ~80 строк Python для patch-parsing + ~100 строк в `agent-flow-merge-gate.sh` |
| Шифу получает явную метрику `file-overlap` в summary логе | 1 новый env-var (FILE_OVERLAP_GUARD) + 1 toggle (COMPETING_PRS_GATE) + 1 tolerance (FILE_OVERLAP_LINE_TOLERANCE); defaults расслаблены |
| Backward-compat: branch_label_override, G8 fingerprint, throttle, G4v2 — все продолжают работать | +1 `gh api pulls/N/files` запрос на каждый PR в OPEN (для merge-gate, единоразово на тик, кэш) |
| Merge-time guard ловит race ПРОПУЩЕННЫЙ pre-create guard (например, обе worker'ы стартовали до G10a) | Комментарий-сторож dedup 24ч — иначе, при каждом тике будет spam |
| Line-tolerance=5 страхует от off-by-one расхождений в номерах строк | Ложные positives если два PR правят соседние (но не пересекающиеся) строки одного файла. Tolerable: Шифу review отличает real competing от coincidental. |

## 5. Acceptance criteria

- [ ] `bash scripts/agent_flow/tests/test_triage_file_overlap_dedup.sh` → exit 0 (≥6 тестов).
- [ ] `bash scripts/agent_flow/tests/test_merge_gate_competing_prs.sh` → exit 0 (≥4 теста).
- [ ] `bash scripts/agent_flow/tests/test_triage_dedup_intra.sh` → exit 0 (backward-compat для G9a).
- [ ] `bash scripts/agent_flow/tests/test_triage_dedup_guard.sh` → exit 0 (backward-compat для idempotency-v2).
- [ ] `bash scripts/agent_flow/tests/test_triage_skip_when_open_pr.sh` → exit 0 (backward-compat для branch:NAME).
- [ ] `bash scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh` → exit 0 (backward-compat для duplicate_file_scan).
- [ ] DRY-RUN реальный прогон `agent-flow-triage.sh`: `tick done: created=N skipped=N errored=N dedup-skipped: N (intra-tick), M (race), K (file-overlap)` появляется в логе.
- [ ] DRY-RUN реальный прогон `agent-flow-merge-gate.sh`: `competing-prs blocked=K` появляется в логе.
- [ ] shellcheck: NO new warnings vs `origin/develop`.
- [ ] Regress-check на fan-out-наборе (PR #2015 / #2016 / `t_e5720945`): при следующем тике после merge G10 → создаётся ≤ 1 kanban-карточка на file-overlap-группу, остальные skipped через G10a.
- [ ] ADR-0052 в `docs/adr/` с уникальным номером (проверено через `git ls-tree -r origin/develop --name-only | grep -oE 'docs/adr/[0-9]{4}' | sort -u | tail -1`, must be 0052 or later).

## 6. Не делаем

- Не внедряем semantic ML (embeddings, sentence-transformers). File-path overlap покрывает 95% ретро-случаев.
- Не блокируем triage network-failure. fail-OPEN на каждом race-check (чтобы сеть не стала blocker'ом).
- Не удаляем старые guards (G4-G9). G10a + G10b + G10c добавляются как ещё один уровень — backward-compat, постепенный rollout.
- Не автозакрываем «проигравший» PR в merge-gate. Шифу ground truth.
- Не храним central file-overlap ledger (Redis, SQLite). Stateless guard per-tick.

## 7. Ссылки

- **Ретро-карточка** `t_50a18fa9` (эта реализация).
- **Issue**: #2018 (nightly-review 2026-09-06 fan-out race).
- **PR-concurring**: #2015, #2016, карточка `t_e5720945` (race 12h+).
- **Cascade-blocked PR**: #1978 (TTS provider chain), #1979 (synth fallback).
- **CI-failure URLs**: https://github.com/krikz/rob_box_project/actions/runs/34071048402, /runs/34071039295.
- **Ретро-связи**: `t_a0fac345` (idempotency-v2), `t_8cde8449` (branch_label_override), `t_dd7a5749` (assignee-existence), `t_a24ffe39` (throttle v3.1), `t_b0fe4398` (G8 fingerprint), `t_dfd3d19d` (G9a intra-tick + G9b race-window), `t_360dc1a4` (Phase 2 GSD-orphans), `t_bfd19ffb` (nightly-review 2026-09-06).
- **ADR-связи**: ADR-0013 (incremental delivery), ADR-0018 (честный FAIL > красивого PASS), ADR-0030 (ADR нумерация + e2e stale-branch), ADR-0032 (G9a/b), ADR-0045 (worker-worktree-base-ref-origin-develop), ADR-0046/47 (orphan-cleanup).
- **CONTRIBUTING.md** §2d: запрет ручного merge — G10 никаких PR не создаёт руками.

## 8. Следующие шаги

1. **architect** (этот PR) — реализует G10a + G10b + G10c, добавляет 2 test, ADR-0052, push в `z-{agent}/2018-hermes-fan-out-race-pr-2015-vs-2016`, открывает PR в `develop`.
2. **merge-gate** проверяет: ADR-номер 0052 уникален (`ADR-collision-guard` ADR-0030 / test_merge_gate_adr_collision.sh) — должно проходить.
3. **e2e-process** на следующий раунд проверяет, что `agent-flow-triage` и `agent-flow-merge-gate` запускаются без падений (как cron, так и под DRY-RUN).
4. **Шифу** мержит PR после green CI + ADR-collision-check.
5. **Мониторинг**: первые 24ч после merge — следить за `dedup-skipped (file-overlap)` и `competing-prs (blocked)` счётчиками. Если > 5/day — большинство issue'ов перекрываются → стоит расширить whitelist G8 или пересмотреть fan-out-триггеры в cron'ах.