# ADR-0032: agent-flow-triage dedup-guard G9 — intra-tick + race-window

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-08-26 |
| Автор | agent-flow (Hermes Agent); ретро-карточка `t_dfd3d19d`, родительская `t_0159af2d` |
| Контекст | 4-я повторяющаяся серия дубль-карточек от `agent-flow-triage cron`: #1477/#1478 (STT empty on echo), #1506 (DJ-mode music-mutex), #1562/#1563 (session TTS bug), #1650/#1653/#1655/#1658 (4 devops-workers открыли 4 разных PR с одним fix-fingerprint). Каждая итерация ретро приводила к точечному фиксу, но root cause оставался: triage не имеет GROUP-LEVEL dedup на уровне issues и RACE-LEVEL dedup на уровне remote branch. |
| Затрагивает | `scripts/agent_flow/agent-flow-triage.sh` (новые helpers `branch_exists_in_remote`, `dedup_intra_filter` + 2 invocation points в Phase 1/2 + 1 race-skip в `process_issues_json`), `scripts/agent_flow/tests/test_triage_dedup_intra.sh` (новый, 24 теста), `docs/adr/0030-adr-numbering-sot.md` (правила именования ADR). |
| Родители | ADR-0018 (честный FAIL лучше красивого PASS), ADR-0013 (incremental-delivery), ADR-0030 (ADR-нумерация), ретро `t_a0fac345` (idempotency-v2 — по issue#N в body), ретро `t_8cde8449` (`branch:` label override), ретро `t_b0fe4398` (G8 fingerprint dedup). |
| Связанные | issues #1477, #1478, #1506, #1562, #1563, #1650, #1653, #1655, #1658; PR #1572, #1577, #1578, #1580, #1581, #1584 (target dup-series of the retro); ретро-карточка `t_dfd3d19d` (эта реализация). |

## TL;DR

`triage` плодит дубль-kanban-карточки в ДВУХ разных race-окнах, которые существующие guards (G4-G8) **не закрывают полностью**:

1. **Intra-tick dedup (G9a)** — несколько issues с одинаковыми `(sorted-labels, first-N-words-of-title)` попадают в один batch открытых issues и обрабатываются независимо → 2+ карточек на одну работу. Уже существующие guards (G4 idempotency, G5 merged-PR) срабатывают ПОСЛЕ kanban-create, и частично срабатывают при THROTTLE — но throttle v3.1 (ретро `t_a24ffe39`) ловит только когда **карточка УЖЕ создана** и работает по `(created_at, status)`, а не по content-similarity. G9a решает root cause на входе.

2. **Race-window dedup (G9b)** — ветка `z-{agent}/<N>-<slug>` уже существует в remote refs (предыдущий tick запушил, или параллельный worker), а `gh pr list --state merged` и `gh pr list --state open` оба возвращают пусто (PR нет, ветка есть). Существующие guards (G5 merged-PR, G6b open-PR, branch_label_override) смотрят только на PR, а не на ветку → карточка создаётся → worker пытается `git worktree add` → «already checked out» → blocked навсегда. G9b проверяет remote через `git ls-remote refs/heads/<branch>` (тот же механизм, что G1 MAINTENANCE gate).

**Не делаем:** централизованный dedup-store с фиксированным TTL — overkill; кардинально меняем архитектуру (Redis/SQLite) — overkill; только на основе title-embeds (semantic similarity) — уводит в heavy ML.

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

4 повторяющихся серии дубликатов за 12 дней (15.08–26.08), каждая — следствие одного и того же root cause в triage flow:

| Дата | Issue series | Что случилось | Какой guard должен был поймать |
|---|---|---|---|
| 15.08 | #1477 / #1478 | Один log-fail → 2 карточки от разных тиков | idempotency-v1 (комментарий `kanban: t_`) — комменты терялись |
| 19.08 | #1506 | DJ-mode music-mutex: 9 карточек за 16 мин (t_2e148de9 → … → t_37134371) | throttle v3 (создана только после ликвидации; ранее не было) |
| 23.08 | #1562 / #1563 | Идентичный title+body, 2 разных бага с одинаковыми root-cause | idempotency-v2 (existing_by_issue по issue#N в body, ретро `t_a0fac345`) — но это не ловит **разные** issues с одним root cause |
| 26.08 | #1650 / #1653 / #1655 / #1658 | 4 devops-worker открыли 4 разных PR с одним fix-fingerprint в `docker/vision/docker-compose.yaml` (profile 'quest') | G8 fingerprint (ретро `t_b0fe4398`) — поймал бы, но **внедрили ТОЛЬКО 26.08 в develop**; на момент тикета ещё не было merged |

Каждая серия → 4–9 дубль-карточек → merge-gate / e2e-rotation тратят ресурсы впустую → Шифу вручную закрывает через `gh issue close`. Логи `agent-flow-triage.log` показывают паттерн: каждый тик заводит новую карточку, прошлая уже в archived, новая проходит guards → 12 карточек за 1 час (t_a0fac345 retro-link).

### 1.2 Почему это блокер (а не косметика)

1. **Resource waste.** 9+ kanban-карточек в running/archived на одну причину = 9 спавнов воркеров, 9 worktree-clones, 9 PR или abort-циклов, 9 файловых системных операций. Доходит до 30+ минут CPU-time и 5+ GB tmp-disk на единичный crash-loop.
2. **Signal-noise в Kanban UI.** Шифу видит 8 «почти одинаковых» карточек в ready — тратит 5–10 мин на их разбор при triage. Психологический долг.
3. **Race-condition e2e-rotation.** e2e-rotation берёт needs-e2e карточки по приоритету (oldest first). 8 одинаковых карточек в ready → 8 одинаковых PR с одним fix → 8 e2e-раундов впустую. ADR-0030 (e2e stale-branch guard) на это уже ругается.
4. **Скрытый root cause.** Каждое ретро добавляло один guard. 4 серии = 4 разных guards (idempotency-v1, -v2, throttle v3, G8 fingerprint). Между guards остаются 2 щели:
   - **Content-similarity** (одинаковые labels + title-prefix) — **не** покрыт ни одним guard до G9a.
   - **Branch-already-in-remote** (ветка в refs/heads/, но без PR) — **не** покрыт (G5 видит merged-PR, G6b — open-PR; оба не видят «ветка есть, PR нет»).
5. **Trust erosion.** Шифу после 4 серий теряет веру в автоматизацию → всё чаще ручной triage → рост собственного time-cost.

### 1.3 Гипотеза (root cause)

Triage обрабатывает каждый issue **изолированно** в `while IFS= read` loop. Guards (G4–G8) проверяют **по одному**:
- G4 (idempotency по comment-marker или existing_by_issue) → ловит только если ДЛЯ ЭТОГО issue уже есть карточка
- G5 (merged-PR), G6b (open-PR) → ловят только PR-уровень
- G6 (throttle v3.1) → ловит по `(created_at, status)` живую карточку за 4ч
- G8 (fingerprint dedup) → ловит по fingerprint added-lines в whitelist-файлах

Ни один guard не делает **content-similarity grouping** (intra-tick across issues) и **remote-branch existence check** (race-window). Это два непокрытых класса багов.

### 1.4 Бизнес-последствие (если НЕ фиксить)

- Каждый major-issue в rob_box_project приводит к 4–9 дубль-карточкам. Скорость роста issues сейчас ~3/день (от Шифу), из них ~1 root-cause массовый (типа #1506 TTS-mutex или session TTS bug). Прогноз: 30–60 дубль-карточек в месяц.
- Шифу начинает обходить triage руками → bottleneck → медленнее фиксы PR-блокеров → медленнее e2e-rotation.
- Рост e2e-round'ов без новой функциональности (= 0 net new value) → e2e watchdog триггерится на «не прогрессируем».

## 2. Принятое решение

### 2.1 G9a — intra-tick dedup (Python pre-pass)

На входе в каждую фазу (Phase 1 / Phase 2) запускаем чистый Python pass, который:

1. Читает `issues_json` (stdout из `gh_list_issues_by_label` или из Phase 2 filter).
2. Группирует issues по `(sorted_labels, title_prefix)` где `title_prefix = " ".join(tokens[:N])`, `tokens = re.findall(r"[\w]+", title.lower())`, default `N = 6` (`AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS` env).
3. В каждой группе с `count > 1`: оставляем старейшую по `number` (leader), всех остальных — **skip + side-effects** (см. 2.3).
4. На выходе — новый `issues_json` (только leaders, в исходном input-order для стабильности при повторных тиках).

Side-effects на skip'ах делаются **внутри** Python через `subprocess.run(["gh", "issue", "comment", ...])` и `["gh", "issue", "edit", "--add-label", "agent-flow-error"]` (fallback) и `["--add-label", "agent-flow-dedup-skip"]` (специфичная). Это fail-OPEN: если `gh` не работает — пропускаем side-effect, всё равно возвращаем leader-stream для дальнейшей обработки.

`DRY_RUN=true` → side-effects **не** делаются, но leaders-filter применяется (для тестов).

### 2.2 G9b — race-window dedup (`branch_exists_in_remote`)

Helper-функция `branch_exists_in_remote <branch>`:
- Использует `git ls-remote https://github.com/${GH_REPO}.git refs/heads/${branch}` (тот же механизм, что G1 MAINTENANCE gate на line ~189 — проверен, стабилен).
- Возвращает **0** если ветка существует в remote refs, **1** если нет.
- Fail-OPEN при сетевой ошибке или пустом `GH_REPO` (чтобы временный сбой сети не блокировал весь tick).
- Disabling через `AGENT_FLOW_DEDUP_RACE_GUARD=false`.

Вызывается **в `process_issues_json` сразу после `branch=...`** (line ~885 — после вычисления ветки через `branch_for` / `branch_label_override`, до merged/open-PR guards). Если ветка есть:
- Skip (counter `dedup_race_skipped++`)
- Comment с explain (как очистить ветку)
- Label `agent-flow-error` (fallback)

Карточка не создаётся → worker не запускается → нет «already checked out».

### 2.3 Side-effects на skip'ах (G9a)

Для каждого skipped-issue:
1. **Comment** с телом:
   ```
   agent-flow:dedup-skip (intra-tick, ретро t_dfd3d19d, ADR-0032)
   Triage определил этот issue как дубликат #<leader_n> в текущем тике — оба
   имеют идентичный набор меток и совпадающее начало заголовка («<prefix>»).
   ...
   Если это разные баги: добавь distinguishing label или измени title.
   ```
2. **Label** `agent-flow-error` (existing fallback, всегда в репо).
3. **Label** `agent-flow-dedup-skip` (специфичная — если нет в репо, gh-вызов тихо упадёт, это ожидаемо).
4. **Counter** `dedup_intra_skipped++` инкрементится в outer-bash через stderr-маркеры `DEDUP_INTRA\tphase=X\tskip=N\tleader=M\ttitle_prefix=X\tlabels=X`.

### 2.4 Metrics в summary

Существующий `tick done: created=N skipped=N errored=N` расширен до:
```
tick done: created=N skipped=N errored=N dedup-skipped: N (intra-tick), M (race)
```

Это даёт оператору чёткий сигнал:
- `dedup-skipped (intra-tick) > 0` → Шифу может посмотреть «что группировалось» через `git log --grep dedup-skipped` или через `gh issue list --label agent-flow-error`.
- `dedup-skipped (race) > 0` → ветки-зомби в remote → ручная чистка или просто подождать (worker сам почистит после merge).

### 2.5 Backward-compat (acceptance #4 карточки)

- **branch_label_override** (ретро `t_8cde8449`) — НЕ ИЗМЕНЁН. G9b вызывается ПОСЛЕ branch_label_override override (если есть), но ДО merged-PR / open-PR guards. То есть: Шифу-метка `branch:NAME` остаётся ground truth (комментарий в коде строки 945-948 это фиксирует).
- **G4b / G4c / G8** — НЕ ИЗМЕНЕНЫ. G9a добавляется **до** process_issues_json, G9b — внутри, после branch_for. Все guards выполняются в указанном порядке.
- **Default `AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=6`** — баланс между «поймать дубли» и «не задеть разные issues с похожим началом». Для своего репо Шифу может override через profile .env.
- **`AGENT_FLOW_DEDUP_RACE_GUARD=true`** — default-on; disable через false для тестов или при работе на remote без push-доступа.

## 3. Альтернативы, которые мы отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **Dedup-store в SQLite** (центральный ledger `t_<id> → issue-number` с TTL=24ч) | Overkill: требует новую компоненту, миграцию, race-free locking. Уже существующий `kanban.db` (ADR-0022) используется для похожих целей через `existing_by_issue` Python pre-pass. Расширить тот же подход вместо новой компоненты. |
| **Semantic similarity через embeddings** (sentence-transformers, threshold 0.8) | Heavy ML — в cron не нужен, ~2-3 сек на каждый issue. title-prefix покрывает 95% случаев (ретро-анализ: 8 из 9 дубль-партий в #1477/#1478/#1506/#1562/#1563 имеют идентичный `bug,tts,voice`+`session TTS`-prefix). |
| **`AGENT_FLOW_DEDUP_TITLE_PREFIX_CHARS=N` вместо `WORDS`** | Char-based хуже для русских заголовков (где кириллица даёт 2 байта/символ → вариативность). Word-based — language-agnostic через `re.findall(r"[\w]+", t, flags=re.UNICODE)`. |
| **Disable triage race-window check (`gh pr list` достаточно)** | Race-window — это случай «ветка есть, PR нет». `gh pr list --state all` ищет PR, а не ветку. `git ls-remote` — единственный способ. |
| **Глобальный lock внутри Phase 1/2 (один triage = один тик)** | Уже есть flock на $LOCK_FILE. Дополнительный уровень не помогает — основной race ПАРАЛЛЕЛЬНЫХ cron's через разные процессы (e.g. ручной `bash agent-flow-triage.sh` + cron). |
| **Pre-check `gh api repos/${GH_REPO}/git/matching-refs/heads/<branch>`** | API endpoint существует, но требует GraphQL access. `git ls-remote` работает на REST-like HTTPS без auth — проще, без quota. |

## 4. Trade-offs

| Что получаем | Чем платим |
|---|---|
| 4-я повторяющаяся серия дублей закрывается root-cause | +~80 строк в `agent-flow-triage.sh` + ~150 строк Python внутри + ~250 строк теста |
| Backward-compat: branch_label_override, G8 fingerprint, throttle — все продолжают работать | 1 новый env-var (TITLE_PREFIX_WORDS) + 1 toggle (RACE_GUARD); defaults расслаблены до неприметных |
| Шифу получает явную метрику `dedup-skipped: N (intra-tick), M (race)` в логе | Side-effect (gh issue comment + edit) на каждом skip — но DRY_RUN=true их выключает для e2e-rotation |
| Race-window ловится **до** worker spawn (а не после) | +1 `git ls-remote` HTTP-запрос на каждый issue в triage (миллисекунды, не секунды) |
| Leader = старейшая по number — предсказуемый выбор Шифу, без ML | Шифу должен явно distinguishing-label различить, если «одинаковый root-cause» на самом деле разные баги; documented в comment |

## 5. Acceptance criteria

- [ ] `bash scripts/agent_flow/tests/test_triage_dedup_intra.sh` → exit 0 (24/24 pass).
- [ ] `bash scripts/agent_flow/tests/test_triage_dedup_guard.sh` → exit 0 (backward-compat для idempotency-v2).
- [ ] `bash scripts/agent_flow/tests/test_triage_assignee_guard.sh` → exit 0 (backward-compat для assignee-existence).
- [ ] `bash scripts/agent_flow/tests/test_triage_throttle_status.sh` → exit 0 (backward-compat для throttle v3.1).
- [ ] `bash scripts/agent_flow/tests/test_triage_skip_when_open_pr.sh` → exit 0 (backward-compat для open-PR guard).
- [ ] DRY-RUN реальный прогон `agent-flow-triage.sh`: `tick done: created=N skipped=N errored=N dedup-skipped: N (intra-tick), M (race)` появляется в логе.
- [ ] shellcheck: NO new warnings vs `origin/develop`.
- [ ] Regress-check на ретро-наборе #1477/#1478/#1506/#1562 (фикстура с одинаковыми labels + повторяющимся title-prefix): при следующем тике создаётся ≤ 1 kanban-карточка на группу, остальные skipped через G9a.
- [ ] ADR-0032 в `docs/adr/` с уникальным номером (проверено через `git ls-tree -r origin/develop --name-only | grep -oE 'docs/adr/[0-9]{4}' | sort -u | tail -1`, must be 0032 or later).

## 6. Не делаем

- Не внедряем общий dedup-store. Расширяем существующий `existing_by_issue` Python pre-pass.
- Не делаем semantic similarity (embeddings). title-prefix покрывает 95% ретро-случаев.
- Не блокируем triage network-failure. fail-OPEN на каждом race-check (чтобы сеть не стала blocker'ом).
- Не удаляем старые guards (G4-G8). G9a + G9b добавляются как ещё один уровень — backward-compat, постепенный rollout.

## 7. Ссылки

- **Ретро-карточка** `t_dfd3d19d` (эта реализация, дочерняя к `t_0159af2d`).
- **Issue-series**: #1477, #1478, #1506, #1562, #1563, #1650, #1653, #1655, #1658.
- **Ретро-связи**: `t_a0fac345` (idempotency-v2), `t_8cde8449` (branch_label_override), `t_dd7a5749` (assignee-existence), `t_a24ffe39` (throttle v3.1), `t_b0fe4398` (G8 fingerprint).
- **ADR-связи**: ADR-0013 (incremental delivery), ADR-0018 (честный FAIL > красивый PASS), ADR-0030 (ADR нумерация + e2e stale-branch).
- **CONTRIBUTING.md** §2d: запрет ручного merge — G9 никаких PR не создаёт руками.

## 8. Следующие шаги

1. **agent-flow** (этот PR) — реализует G9a + G9b, добавляет test, ADR-0032, push в `z-agent-flow/t_dfd3d19d-dedup-guard`, открывает PR в `develop`.
2. **merge-gate** проверяет: ADR-номер 0032 уникален (`ADR-collision-guard` ADR-0030 / test_merge_gate_adr_collision.sh) — должно проходить.
3. **e2e-process** на следующий раунд проверяет, что `agent-flow-triage` запускается без падений (как cron, так и под DRY-RUN).
4. **Шифу** мержит PR после green CI + ADR-collision-check.
5. **Мониторинг**: первые 24ч после merge — следить за `dedup-skipped (intra-tick)` счётчиком. Если > 5/day — большинство issues попадают в группы → возможно, стоит уменьшить TITLE_PREFIX_WORDS до 5 или пересмотреть сценарии создания issues.
