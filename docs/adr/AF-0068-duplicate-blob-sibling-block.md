# AF-0068 — duplicate-blob sibling PR → hard-block + Шифу-escalation (G10e)

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-15 |
| Автор | devops worker (ретро-карточка `t_763713e6`, parent `t_c361e6bb`) |
| Контекст | `scripts/agent_flow/agent-flow-merge-gate.sh` `duplicate_file_scan_all()` (было: info-only коммент, ретро 15.08 t_20383d32); должно стать hard-block (label `agent-flow-block` + Шифу-escalation) — расширение G10. |
| Затрагивает | `scripts/agent_flow/agent-flow-merge-gate.sh` (`duplicate_file_scan_all`), `scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh` (кейсы A/B/D/E переименованы, добавлены G/H), `scripts/agent_flow/tests/lib/mock_env.sh` (новый `assert_ge` helper) |
| Родители | ADR-AF-0030 (ADR-нумерация), ADR-AF-0062 (competing-PRs block — G10b), ADR-AF-0065 (scan-all-prs race-guard), ADR-0018 (честный FAIL > красивый PASS) |
| Связанные | issue #2499 (binding cleanup), PR #2517 (binding cleanup), PR #2526 (deploy-fix поверх #2517), карточки `t_c361e6bb` (ретро race-condition), `t_763713e6` (этот fix), `t_20383d32` (исходная info-only версия) |

## TL;DR

Расширяем `duplicate_file_scan_all()` с **info-only** (ретро 15.08 t_20383d32) до **HARD-BLOCK**: при identical-blob overlap между двумя needs-review/needs-e2e PR оба получают label `agent-flow-block` (G10e), merge-gate пропускает только после явного Шифу-одобрения (через label remove или отдельный ACK). Коммент содержит **explicit canonical-PR recommendation** (по критериям: больше коммитов → позже создан → уже в needs-e2e-ротации) и команды для Шифу: `gh pr merge` canonical, `gh pr close #other --delete-branch`, `gh pr edit #canonical --remove-label`.

**Не делаем:** автозакрытие второго PR (Шифу ground truth — какой закрывать); ML-based similarity (overkill для детерминированного identical-blob); единый dedup-store (уже есть G8 fingerprint для whitelist-файлов).

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (raw evidence)

15.09.2026 issue #2499 «binding cleanup» породил **два независимых fix-PR**: PR #2517 (developer) и PR #2526 (developer, поверх cherry-pick #2517 + deploy-fix). Оба содержат **3 файла с identical blob-sha**:

```
Dockerfile     d35dc1c4...
ADR-0099.md    1d0405007...
loader.py      93acbf1c...
```

`duplicate_file_scan_all()` (существующий) детектит identical-blob overlap, **НО только info-коммент** (см. PR #2517 comments 03:04:48Z–55Z, ретро t_20383d32 текст: «merge-gate НЕ блокирует CI/e2e»). Никакого hard-block, никакой Шифу-эскалации.

`git merge-tree` показывает `changed_in_both=0` — формально оба PR MERGEABLE, merge первого пройдёт без конфликта (orphan-merge). Второй PR остаётся висеть как orphan с пустым diff. Шифу обнаруживает это только при ревью (если канбан-карточка ещё не истекла), либо когда orphan-PR ломает e2e-rotation (двойной деплой того же контейнера).

### 1.2 Почему существующий guard не помогает

| Guard | Что ловит | Почему не ловит identical-blob orphan |
|---|---|---|
| **G10a** (triage file_overlap, ADR-AF-0062) | file_overlap на этапе создания kanban-карточки | Issue-anchor не написан в body → guard skip |
| **G10b** (merge-gate competing-prs, ADR-AF-0062) | path-overlap (basename-match, subpath-containment) при merge-time | **Срабатывает** на тот же случай, но не даёт Шифу явной canonical-recommendation + не использует identical-blob как STRONGER signal |
| **G8** (fingerprint dedup) | whitelist-файлы (docker-compose/package.xml/setup.py/Dockerfile/install/setup) | Частично ловит Dockerfile, но остальные 2 (ADR-0099.md, loader.py) — нет |
| **`duplicate_file_scan_all` (t_20383d32)** | identical blob-sha между needs-* PR | **Только info**, не блокирует |

Gap: identical-blob (G10e) — STRONGER signal чем path-overlap (G10b), потому что identical-blob = orphan-merge по определению. Шифу ground truth «какой PR закрыть» определяется trivial-rules: больше коммитов → позже создан → уже в e2e-ротации. Этих правил нет ни в одном существующем guard.

### 1.3 Бизнес-эффект

- **Orphan-merge = пустой diff, потраченный CI-цикл, потенциальный двойной deploy.** В round-405 (2026-09-15) orphan-PR #2517 заблокировал merge #2526 — Шифу вручную выбирал canonical.
- **Без hard-block Шифу должен отслеживать вручную info-комменты на ~30 needs-* PR** — реалистично он их не читает до e2e-rotation phase.
- **Без canonical-pick** даже если Шифу заметил — нет подсказки «какой PR закрыть», он тратит 5-10 мин на `gh pr view N --json commits,createdAt,labels` для каждого orphan-pair.

## 2. Решение

### 2.1 Что делаем (G10e, hard-block)

Расширяем `duplicate_file_scan_all()` в `agent-flow-merge-gate.sh`:

1. **Filter (unchanged):** оба PR должны иметь `needs-review` или `needs-e2e` метку.
2. **Detection (unchanged):** identical blob-sha на одном filename в ≥2 PR.
3. **Canonical-PR pick (NEW):** из пары (A, B) выбираем canonical по критериям:
   1. Больше коммитов (через `gh api repos/X/pulls/N/commits`).
   2. Если равно — позже `createdAt`.
   3. Если равно — тот, у которого `needs-e2e` (приоритетнее `needs-review`).
   4. Если всё равно — первый по номеру (стабильный tie-break).
4. **Action (HARD-BLOCK, NEW):**
   - `gh pr edit N --add-label agent-flow-block` на **оба** PR.
   - `gh pr comment N` с explain + canonical-pick + команды для Шифу.
   - **Dedup 24h** через `comment_recently_posted` (contains-mode, окно 86400s).
5. **Backward-compat:** отключается через `DUPLICATE_BLOB_GUARD=false`.

### 2.2 Чего НЕ делаем

- **Не auto-close второго PR.** Шифу ground truth важнее machine-решения. Шифу может захотеть влить оба (разные deploy-environments), сохранить оба для ревью параллельно, или закрыть «первый» если второй содержит cherry-pick поверх (как в PR #2526 vs #2517).
- **Не единый dedup-store (fingerprint DB).** G8 уже частично решает через whitelist. Идентичный-blob — это **stronger** signal, не требует нового store.
- **Не ML-based similarity.** Overkill для детерминированного case «одинаковый blob-sha». GitHub API даёт точный ответ через `sha`.
- **Не блокируем PR без needs-* меток.** Черновик-фиксы (нет process-метки) — нормальный случай, их merge-gate не должен трогать (это уже было в info-only версии).

### 2.3 Acceptance criteria (закреплено в kanban-задаче)

- [x] `duplicate_file_scan_all()` при identical-blob overlap с sibling-PR блокирует оба PR (label `agent-flow-block`), а не только info-коммент.
- [x] Только если оба PR имеют метку `needs-review` или `needs-e2e` (как `competing_prs_block_scan_all`).
- [x] Блок срабатывает **до** merge — merge-gate пропускает только после явного Шифу-одобрения (через label remove).
- [x] Коммент содержит canonical-PR + команды `gh pr merge`, `gh pr close`, `gh pr edit --remove-label`.
- [x] Dedup 24ч через `comment_recently_posted` (contains-mode, 86400s).
- [x] Тесты в `test_merge_gate_duplicate_file.sh` расширены: кейс G (identical-blob → block) и кейс H (один PR без needs-* метки → не блокируется).
- [x] `bash scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh` — все тесты зелёные.

## 3. Реализация

### 3.1 Шаги

1. **Расширить `duplicate_file_scan_all()`** в `agent-flow-merge-gate.sh:970-1026`:
   - Pull `createdAt` через `gh pr list --json number,headRefName,createdAt`.
   - В Python: для каждой пары (A, B) с identical-blob — вычислить canonical по `(commits, needs-e2e-priority, createdAt)`.
   - Emit `\t`-separated lines: `fname\tsha\tcanonical\tother\ta\tb`.
   - Bash loop: для каждой пары — comment + label на оба PR (dedup 24h).
2. **Добавить env-vars:** `DUPLICATE_BLOB_BLOCKED_LABEL` (default `agent-flow-block`), `DUPLICATE_BLOB_GUARD` (default `true`).
3. **Обновить тесты:** кейсы A/B/D переименованы под новый маркер `duplicate blob sibling detected`, кейс E → SKIPPED (mock_env не персистит state), кейсы G и H — новые.
4. **ADR:** этот документ (`AF-0068`).
5. **Process-tests:** `bash scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh` + `validate_adr_namespace.sh` + `validate_honesty.sh`.

### 3.2 Грабли, которые были учтены

- **Двойной label-add (G10b + G10e на той же паре):** оба guard'а идемпотентно ставят `agent-flow-block`. Тест-ассерт `>= 1`, не `== 1`. В проде `gh pr edit --add-label` — no-op для существующего label.
- **mock_env не персистит `ISSUE_*_COMMENTS_SINCE_JSON`** между `run_merge_gate` (та же проблема, что test_merge_gate_competing_prs case E). Тест E → SKIPPED с явным объяснением.
- **Пустой `createdAt`** → python sort по tuple `(commits, needs-e2e-priority, createdAt)`, пустая строка < non-empty → может выбрать «первый» как canonical. Fail-open в этом случае (canonical-PR не критичен, главное — оба PR заблокированы).
- **`gh api repos/X/pulls/N?per_page=1`** в mock_env не возвращает `commits` field (mock возвращает `{"number":N}` для PR-existence guard). Поэтому fallback на `gh api repos/X/pulls/N/commits?per_page=100` (mock читает `PR_N_COMMITS_JSON`). В проде оба API работают — fallback не нужен, но он сохраняется для backwards-compat.

## 4. Последствия

### 4.1 Что меняется для Шифу

- **Раньше:** info-коммент «merge-gate НЕ блокирует CI/e2e» — Шифу мог пропустить, и orphan-PR зависал до e2e-rotation phase.
- **Теперь:** hard-block с явной canonical-pick + команды `gh pr merge`, `gh pr close`, `gh pr edit --remove-label`. Шифу за 30 сек выбирает canonical и закрывает orphan.
- **Workflow:** «пометил оба PR `agent-flow-block`» → e2e-rotation **пропускает** round → Шифу ревьюит → `gh pr merge canonical` → `gh pr close #other --delete-branch` → `gh pr edit #canonical --remove-label agent-flow-block` → merge-gate следующего тика видит clean state → e2e-rotation resume.

### 4.2 Что меняется для worker'ов

- **Ничего.** Worker создаёт PR как обычно. Если его PR — orphan (Шифу закроет) — worker получает close-event, может переоткрыть через rebase поверх canonical-PR.
- **Процесс для Шифу остаётся:** выбрать canonical, merge, close.

### 4.3 Operational impact

- **CI minutes:** без изменений (блок не запускает дополнительных проверок).
- **e2e-rotation:** оба PR пропускаются пока `agent-flow-block` висит → minor slow-down для fan-out-PR (Шифу должен быстро решить canonical).
- **Backlog noise:** -1 orphan-PR за случай (раньше висел неограниченно, теперь закрывается Шифу за 1 ревью).

## 5. Альтернативы, которые рассмотрели и отвергли

### 5.1 Auto-close второго PR (по canonical-pick criteria)

- **Плюс:** zero Шифу-effort, мгновенный orphan-cleanup.
- **Минус:** Шифу может хотеть merge оба (разные deploy-env), или оба нужны для параллельного ревью. Machine-решение → loss-of-control.
- **Решение:** оставить за Шифу. Hard-block даёт тот же эффект (round-skipped), но с human-in-the-loop.

### 5.2 ML-based canonical-pick (sentence-transformers file-similarity)

- **Плюс:** может учесть семантику («PR #2015 vs #2016 — оба фиксят yaml, но один из них шире»).
- **Минус:** non-deterministic, slow (model-load), overkill для identical-blob (детерминированный сигнал уже есть).
- **Решение:** heuristic canonical-pick (commits → createdAt → needs-e2e) — детерминированный, <10ms per pair.

### 5.3 Pull-file-once strategy (не тянуть `/pulls/N/files` дважды)

- Сейчас G10e и G10b оба тянут `/pulls/N/files` для needs-* PR. Можно cache'ить между guard'ами.
- **Решение:** out-of-scope для этого PR. Можно сделать отдельный guard-cache (ADR-AF-0069?) если profile hits > 50% requests.

## 6. Out of scope (явно)

- Не трогаем `competing_prs_block_scan_all` (G10b) — он работает для path-overlap, отличается от G10e (identical-blob).
- Не трогаем G10c (issue-closed-by-PR-body) — отдельная PR-body-гигиена.
- Не правим e2e-process / triage — только merge-gate + тесты.
- Не делаем fingerprint-DB для non-whitelist файлов (G8 частично решает для whitelist).

## 7. Ссылки

- Issue #2499 — root: binding cleanup.
- PR #2517 — orphan-fix (3 identical blob в Dockerfile/ADR-0099.md/loader.py).
- PR #2526 — canonical-fix (cherry-pick PR #2517 + deploy-fix сверху).
- Retro `t_c361e6bb` (2026-09-15) — race-анализ, created child `t_763713e6`.
- Retro `t_20383d32` (15.08) — исходная info-only `duplicate_file_scan_all`.
- ADR-AF-0062 (G10b competing-prs) — шаблон для G10e.
- `scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh` — регрессионные тесты (A,B,C,D,E,G,H).