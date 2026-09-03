# ADR-0047: orphan-cleanup — verification criterion через `merge_commit_sha` в целевой ветке, а не только PR state

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge → Accepted) |
| Дата | 2026-09-02 |
| Автор | architect (Hermes Agent); ретро-карточка `t_dc7ce789` (false-anomaly поверх `t_a9b28a7f`) |
| Контекст | ADR-0046 формализует orphan-cleanup-after-merge как паттерн и предлагает два варианта реализации (A — расширение merge-gate; B — отдельный cron-надзор). Оба варианта **используют `gh pr list --state merged` как единственный сигнал «PR доехал»**. Это создаёт false-positive cleanup в ситуациях, когда PR смержен в **feature-ветку** (например `feature/avatar`), а не в `develop`. Worker-надзор от 02.09 18:30 CEST зафиксировал такой случай: 6 issues AV-3..AV-9 (#1597, #1598, #1599, #1601, #1602, #1603) ссылаются на PR (#1632, #1610, #1622, #1618, #1607, #1633), которые MERGED 25.08 в `feature/avatar`. `merge_commit_sha` (138affcbc1, b0021b81a1, 1ee4a06f4b, 8629137291, 54fbb21456) **отсутствуют** в `origin/develop`. merge feature/avatar → develop = `610bf10eb` (28.08.2026 21:14 +0300). До этого merge — cleanup триггерить нельзя. ADR-0046 §6 «Что НЕ покрывает» фиксирует ограничение «карточки без `PR #N` в body», но **не** формулирует критерий verification через git-ancestry. Этот ADR закрывает пробел. |
| Затрагивает | (a) ADR-0046 §3 «Инвариант», §4 «Решение», §5 «Рекомендация» — добавить уточнение; (b) ADR-0046 §6 «Что НЕ покрывает» — добавить явный критерий verification; (c) `scripts/agent_flow/agent-flow-merge-gate.sh` — функция `archive_merged_card` (ADR-0046 §4.1) и функция `process_pr` (~line 3136-3346, где уже есть `pr_base = $DEVELOP_BRANCH` guard) — добавить обязательную ancestry-проверку; (d) новый скрипт `agent-flow-orphan-cleanup-cron.sh` (ADR-0046 §4.2) — добавить ту же проверку в оба места использования `gh pr list --state merged`. **Не затрагивает** `archive_merged_card` для issue→card связи (там `merge-base --is-ancestor` уже используется на line 1229 — это reference implementation). |
| Родители | ADR-0046 (orphan-cleanup-after-merge), ADR-0030 (ADR-нумерация), ADR-0014 (post-merge cleanup), ADR-0026 (recovery contract). |
| Связанные | `scripts/agent_flow/agent-flow-merge-gate.sh` (line 1229 — reference `merge-base --is-ancestor`, файл SOT); ADR-0046 §1.2 «Почему не сработали текущие защиты» (таблица защит); ретро `t_a9b28a7f` (false-anomaly, корректировка здесь); ретро `t_d778653a` (orphan-kanban-cleanup-after-merge — другая разновидность паттерна, process-fix в PR #1860/#1889); feature-ветка `feature/avatar` (отдельный жизненный цикл — merge в develop `610bf10eb` 28.08); `~/.hermes/kanban/boards/robbox/kanban.db` (SOT карточек). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (на 02.09 18:30 CEST)

Worker `t_a9b28a7f` (02.09 18:06) объявил «orphan-issues-после-merge» паттерном и предложил process-fix в merge-gate: «close issue при merged→develop». Надзор-тик (`agent-flow` в комментарии 18:10 к карточке) проверил факты через `gh api /repos/{owner}/{repo}/commits/{sha}` и GitHub timeline cross-ref — **гипотеза ретро-карточки оказалась неверной**:

| issue | assignee | PR | state PR | merge-base | in develop? |
|-------|----------|----|----------|-----------|-------------|
| #1597 AV-3 FSM ModeManager | backend | #1632 | MERGED 25.08 | feature/avatar | NO |
| #1598 AV-4 LockManager | backend | #1610 | MERGED 25.08 | feature/avatar | NO |
| #1599 AV-5 msgpack /avatar/state | backend | #1622 | MERGED 25.08 | feature/avatar | NO |
| #1601 AV-7 voice_input_mode | backend | #1618 | MERGED 25.08 | feature/avatar | NO |
| #1602 AV-8 frame types 0x30-0x33 | architect | #1607 | MERGED 25.08 | feature/avatar | NO |
| #1603 AV-9 docker supervisor | (см. PR #1633) | #1633 | MERGED 25.08 | feature/avatar | NO |

`feature/avatar` — отдельная feature-ветка с собственным жизненным циклом. До merge `feature/avatar` → `develop` (`610bf10eb`, 28.08.2026 21:14 +0300) **ни один из этих merge не должен был триггерить close-issue/cleanup**. Карточка `t_a9b28a7f` — **false-anomaly**: паттерн «merged PR + open issue» существует и в здоровом процессе (когда PR уходит в feature-ветку и ждёт feature→develop merge), поэтому универсальное правило `merged → close` некорректно.

### 1.2 Почему не сработали текущие защиты

| Слой | Что должен ловить | Почему не сработал |
|---|---|---|
| `gh pr list --state merged` (merge-gate §4 ADR-0046) | список «доехавших» PR | Возвращает **все** MERGED PR, включая PR в feature-ветки (AV-* эпик). Не различает target-branch. |
| `pr_base = $DEVELOP_BRANCH` guard (merge-gate:3144, `if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ]`) | фильтр по целевой ветке для issue-close | **Уже есть** для issue-close path (line 3144). **Отсутствует** для orphan-cleanup path (вариант A ADR-0046 §4.1 — расширение `archive_merged_card`; вариант B §4.2 — новый cron). Это и есть дыра. |
| `merge-base --is-ancestor` (merge-gate:1229, stale-after-upstream-fix) | reference implementation ancestry-проверки | Существует как **отдельный use-case** для stale-card detection. Не переиспользуется в cleanup path. |
| ADR-0046 §3 «Инвариант» | формулировка цели cleanup | «карточка завершается lifecycle ≤ 10 мин после merge этого PR» — **не уточняет**, в какую ветку merge. Это и фиксирует данный ADR. |

### 1.3 Гипотеза (root cause)

Три независимые неточности сложились в false-anomaly `t_a9b28a7f` и в потенциальную ловушку для будущих cleanup-скриптов:

1. **ADR-0046 §3 инвариант сформулирован в терминах PR-state, а не git-ancestry.** «Карточка завершается lifecycle после merge этого PR» — формально верно, но не уточняет merge **в какую ветку**. Это допускает cleanup на любой MERGED PR, что для feature-веток = преждевременное закрытие issue.
2. **Cleanup-путь (вариант A §4.1 + вариант B §4.2 ADR-0046) не имеет обязательной ancestry-проверки.** Оба варианта используют `gh pr list --state merged` как единственный источник истины. `pr_base != $DEVELOP_BRANCH` guard существует в **issue-close path** (merge-gate:3138), но не в **orphan-cleanup path** — потому что orphan-cleanup разрабатывался для случая «PR в develop, карточка забыта», а не «PR в feature-ветку, карточка ждёт feature→develop merge».
3. **Worker, создавший ретро `t_a9b28a7f`, не проверил `merge_commit_sha` в `git log origin/develop`.** Это типичный класс ошибок: «PR state=MERGED → доехал» без cross-ref с target-branch. ADR-0047 формализует verification criterion как pre-merge guard для cleanup-скриптов и как документированную процедуру для worker-надзора.

### 1.4 Бизнес-последствие (если не чинить)

- **Преждевременный cleanup**: если devops-воркер реализует ADR-0046 §4.1 (вариант A) или §4.2 (вариант B) без ancestry-проверки, то 6 AV-* issues закроются автоматически до merge `feature/avatar` → `develop`. Когда merge дойдёт, issues уже CLOSED, и надзор не увидит, что работа была сделана; kanban-карточки заархивируются как «auto-cleaned», хотя фактически merge только что произошёл.
- **Дискредитация ретро-процесса**: ровно тот же класс ошибки, что ADR-0046 §1.4 описывает для «stale cards копятся», но в обратную сторону — **false-cleanup** ломает доверие к merge-gate (issues закрываются, а work не дошёл).
- **Эрозия `pr_base` guard**: если сейчас не зафиксировать критерий verification, каждый следующий cleanup-скрипт будет переизобретать фильтр «что считать доехавшим» — и каждый раз будет существовать риск feature-branch leak.
- **Сложность отладки post-mortem**: если cleanup сработал для feature-branch PR, восстановить «а когда это случилось?» сложно — нужна реконструкция `gh pr list --state merged --base feature/avatar` истории, которой никто не ведёт.

## 2. Где SOT и какие слои трогаем

### 2.1 SOT скриптов — `<repo>/scripts/agent_flow/*.sh`

Изменения вносятся **только** в `scripts/agent_flow/agent-flow-merge-gate.sh` (вариант A ADR-0046) и/или `scripts/agent_flow/agent-flow-orphan-cleanup-cron.sh` (вариант B ADR-0046). Никаких ручных правок на роботе; sync через `bash <repo>/scripts/agent_flow/install.sh` + `md5sum` (процедура Шифу от 14.08).

### 2.2 SOT данных — `~/.hermes/kanban/boards/robbox/kanban.db`

Изменения не нужны. Schema kanban DB стабильна; поля `body`, `assignee`, `status`, `created_at` используются как сейчас.

### 2.3 SOT решений — `docs/adr/0046-orphan-cleanup-after-merge.md`, `docs/adr/0047-*.md` (этот ADR)

ADR-0046 — primary source для orphan-cleanup pattern. Этот ADR (0047) — **дополнение**, не замена. ADR-0046 §3 «Инвариант» нуждается в уточнении; ADR-0046 §4 «Решение» (варианты A и B) нуждается в добавлении ancestry-проверки; ADR-0046 §6 «Что НЕ покрывает» нуждается в явном критерии verification (этот ADR).

## 3. Инвариант (уточнение ADR-0046 §3)

**Карточка со статусом ∈ {ready, todo, blocked}, у которой `body` содержит `PR #N`, завершает lifecycle (unblock → complete → archive) только если оба условия выполнены одновременно:**

1. `gh pr view <N> --json state` → `"MERGED"` (текущий сигнал, остаётся).
2. **`merge_commit_sha` PR-а является ancestor-ом `origin/<target-branch>`, где `<target-branch>` — целевая ветка PR (определяется через `gh pr view <N> --json baseRefName`).**

Эквивалентный shell-критерий:

```bash
pr_sha="$(gh pr view "$pr_num" --repo "$GH_REPO" --json mergeCommit --jq '.mergeCommit.oid' 2>/dev/null)"
pr_base="$(gh pr view "$pr_num" --repo "$GH_REPO" --json baseRefName --jq '.baseRefName' 2>/dev/null)"
git -C "$repo_dir" merge-base --is-ancestor "$pr_sha" "origin/${pr_base}"
```

**Reference implementation** уже существует в `agent-flow-merge-gate.sh:1229` (для stale-after-upstream-fix guard, ADR-0035). Этот ADR предлагает **переиспользовать тот же паттерн** в cleanup-пути ADR-0046 (варианты A и B).

**Допустимые упрощения** (для случая, когда merge-base невозможен):

- **fallback на `gh api /repos/{owner}/{repo}/commits/{sha}` → 200 OK** для целевой ветки через `?ref=<target-branch>`. Это менее надёжно (sha может существовать в reflog, но не в истории), но лучше, чем «только PR state». Применять только если `git merge-base` физически недоступен (например, в окружении без `repo_dir`).
- **Hard target-branch whitelist**: если в данной инсталляции cleanup разрешён **только** для определённых веток (например, `develop` + `main`), то ancestry-проверку можно заменить на `case "$pr_base" in develop|main) ... ;; *) skip ;; esac`. Это **строже**, чем ancestry, и рекомендуется для production cleanup (не архивировать карточки для PR в feature-ветки в принципе).

**Допускается fail-OPEN с WARN-логом** (как и ADR-0046 §3): если `gh pr view` падает с rate-limit или `git merge-base` возвращает ошибку, cleanup откладывается до следующего тика, **не** крашит cron. Но **не допускается** fail-CLOSED без verification — то есть cleanup «наугад» по `state=MERGED` без target-branch ancestry проверки **запрещён**.

### 3.1 Изменение формулировки ADR-0046 §3

ADR-0046 §3 (текущая формулировка):

> Каждая kanban-карточка со статусом ∈ {ready, todo, blocked}, у которой `body` содержит `PR #N` с merged-PR `N`, завершается lifecycle (unblock → complete → archive) в течение ≤ 10 мин после merge этого PR.

ADR-0047 §3 (уточнённая формулировка, действует вместе с §3.1 ADR-0046):

> Каждая kanban-карточка со статусом ∈ {ready, todo, blocked}, у которой `body` содержит `PR #N`, завершает lifecycle (unblock → complete → archive) в течение ≤ 10 мин после merge этого PR **в его целевую ветку** (`baseRefName`). Verification: `git merge-base --is-ancestor <merge_commit_sha> origin/<baseRefName>`. Если merge произошёл в feature-ветку (не в `develop`/`main`), cleanup подавляется до merge feature→develop.

## 4. Решение

### 4.1 Единственный вариант — **обязательная ancestry-проверка в обоих вариантах ADR-0046**

Этот ADR **не вводит новый вариант реализации** — он обязывает **оба** существующих варианта (A и B из ADR-0046) включать ancestry-проверку как hard gate. Причина: природа дыры — «cleanup триггерится на feature-branch merge». Неважно, какой вариант cleanup выбран (merge-gate extension или cron-надзор) — оба подвержены одной и той же ошибке. Фикс — единый.

**Конкретные изменения**:

1. **`scripts/agent_flow/agent-flow-merge-gate.sh::archive_merged_card` (ADR-0046 §4.1)**:
   - В цикле по `scan_kanban_for_stale_pr_refs "$pr"`, **перед** `unblock → complete → archive`:
     - Получить `merge_commit_sha` через `gh pr view "$pr" --json mergeCommit --jq '.mergeCommit.oid'`.
     - Получить `baseRefName` через `gh pr view "$pr" --json baseRefName --jq '.baseRefName'`.
     - Выполнить `git -C "$repo_dir" merge-base --is-ancestor "$sha" "origin/${base}"`.
     - Если false → `log "orphan-scan: PR#${pr} MERGED в ${base} (не в ${DEVELOP_BRANCH}) — skip cleanup, retry после merge feature→${DEVELOP_BRANCH}"`, continue.
     - Если true → existing flow (unblock → complete → archive).

2. **`scripts/agent_flow/agent-flow-orphan-cleanup-cron.sh` (ADR-0046 §4.2)**:
   - В python-блоке `for m in stale_pat.finditer(body)` добавить **до** вызова unblock/complete/archive:
     - `pr_sha = subprocess.run(['gh', 'pr', 'view', str(pr_num), '--repo', repo, '--json', 'mergeCommit,baseRefName', '--jq', '.mergeCommit.oid + " " + .baseRefName'], capture_output=True, text=True).stdout.split()`.
     - `subprocess.run(['git', '-C', repo_dir, 'merge-base', '--is-ancestor', pr_sha[0], f'origin/{pr_sha[1]}'], check=False)` → если exit != 0, skip + WARN log.
     - Альтернатива (проще): whitelist `pr_sha[1] in {'develop', 'main'}` — fail-CLOSED для любых feature-веток. **Рекомендуется** для cron-надзора, поскольку cron может работать в окружении без `repo_dir` (например, отдельный контейнер).

3. **`scripts/agent_flow/agent-flow-merge-gate.sh::process_pr`** (issue-close path, lines 3136-3346):
   - Существующий guard `if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ]; then` **уже** содержит target-branch проверку. **Дополнить** ancestry-проверкой:
     - В начале блока (после `log "issue #${number}: PR #${pr_number} MERGED into ${pr_base} — post-merge reconcile"`) добавить:
       ```bash
       _pr_sha="$(gh pr view "$pr_number" --repo "$GH_REPO" --json mergeCommit --jq '.mergeCommit.oid' 2>/dev/null || echo "")"
       if [ -n "$_pr_sha" ] && [ -d "$repo_dir" ]; then
           if ! git -C "$repo_dir" merge-base --is-ancestor "$_pr_sha" "origin/${pr_base}" 2>/dev/null; then
               log "issue #${number}: PR #${pr_number} MERGED-state но sha ${_pr_sha:0:8} НЕ в origin/${pr_base} (ancestry check failed, retro t_a9b28a7f / ADR-0047) — skip close, retry after target-branch catches up"
               continue
           fi
       fi
       ```
   - Это **defense in depth**: текущий `pr_base = $DEVELOP_BRANCH` — строковое равенство, но если когда-нибудь в `DEVELOP_BRANCH` появится edge-case (например, `develop` rename), ancestry-check остаётся надёжным.

### 4.2 Параметр `CLEANUP_TARGET_BRANCHES` (whitelist, рекомендуется для cron)

Для явного fail-CLOSED поведения в cron-надзоре (вариант B ADR-0046) — добавить env-переменную:

```bash
CLEANUP_TARGET_BRANCHES="${CLEANUP_TARGET_BRANCHES:-develop main}"
```

И в python-блоке:

```python
allowed = set(os.environ.get('CLEANUP_TARGET_BRANCHES', 'develop main').split())
if pr_sha[1] not in allowed:
    print(f"orphan-cleanup-cron: PR#{pr_num} base={pr_sha[1]} not in whitelist {allowed} — skip cleanup", file=sys.stderr)
    continue
```

**Плюсы**:
- Hard guard, не зависит от git state (можно запускать в окружении без `repo_dir`).
- Явный список разрешённых веток виден в конфиге cron-задачи.

**Минусы**:
- Требует обновления при добавлении новых долгоживущих веток (например, `release/*`).

**Рекомендация**: использовать whitelist (`CLEANUP_TARGET_BRANCHES`) для варианта B (cron), использовать ancestry-check для варианта A (merge-gate extension, где `repo_dir` всегда доступен).

## 5. Рекомендация архитектора

**Применить §4.1 (оба варианта ADR-0046 получают ancestry-проверку) + §4.2 whitelist для cron-надзора.**

**Обоснование**:
1. **KISS**: не вводим новый вариант, фикс — тривиальная проверка через уже существующий reference pattern (line 1229).
2. **Минимальный blast radius**: ~15 строк в merge-gate.sh + ~5 строк в orphan-cleanup-cron.sh (если будет создан). Никакой новой cron-инфраструктуры.
3. **Defense in depth**: `pr_base = $DEVELOP_BRANCH` строковый guard **уже есть** в issue-close path; ancestry-check его дополняет, не заменяет.
4. **Whitelist для cron-надзора**: явный список разрешённых веток виден в jobs.json — проще аудировать, чем полагаться на git state окружения.

## 6. Что НЕ покрывает ADR-0047

- **Карточки, у которых в body указан `merge_commit_sha` напрямую (не `PR #N`).** Таких пока не наблюдалось, но если появятся — нужен второй regex (`merge_commit_sha: ([0-9a-f]{40})`) в cleanup-скриптах. ADR-0047 не покрывает этот случай явно.
- **Cross-board cleanup** (ADR-0046 §6 follow-up). ADR-0047 покрывает только target-branch ancestry для **robbox** board.
- **PR, у которых `mergeCommit.oid` пустой** (например, PR был squash-merged через GitHub UI без squash-commit, или PR был rebased и merge-state обновился). В этом случае fallback `gh api /repos/{owner}/{repo}/commits/{sha}?ref=<base>` (см. §3) — но это engineering workaround, не vendor guarantee.
- **Race condition**: `merge-base --is-ancestor` смотрит на текущий `origin/<branch>`. Если между проверкой и `git fetch` в другом тике target-branch получит force-push (rebase), результат может измениться. Это известное ограничение git, не специфика ADR-0047; merge-gate уже делает `git fetch origin ${pr_base}` перед каждым тиком (строка ~1980), что минимизирует окно гонки.

## 7. Verification — как проверить, что фикс работает

### 7.1 Pre-fix state (snapshot сейчас, 02.09 18:30Z)

```bash
# 1) Подтвердить, что 6 AV-* issues остаются OPEN (это правильное состояние — merge ещё не в develop):
gh issue view 1597 --repo krikz/rob_box_project --json state --jq '.state'  # → OPEN
gh issue view 1598 --repo krikz/rob_box_project --json state --jq '.state'  # → OPEN
gh issue view 1599 --repo krikz/rob_box_project --json state --jq '.state'  # → OPEN
gh issue view 1601 --repo krikz/rob_box_project --json state --jq '.state'  # → OPEN
gh issue view 1602 --repo krikz/rob_box_project --json state --jq '.state'  # → OPEN
gh issue view 1603 --repo krikz/rob_box_project --json state --jq '.state'  # → OPEN

# 2) Подтвердить, что merge_commit_sha этих PR отсутствуют в origin/develop:
for pr in 1632 1610 1622 1618 1607 1633; do
  sha="$(gh pr view "$pr" --repo krikz/rob_box_project --json mergeCommit --jq '.mergeCommit.oid' 2>/dev/null)"
  git -C /home/builder/hermes-share/rob_box_project merge-base --is-ancestor "$sha" origin/develop \
    && echo "PR#${pr}: ${sha:0:8} IS in develop (FALSE EXPECTATION)" \
    || echo "PR#${pr}: ${sha:0:8} NOT in develop (CORRECT — feature/avatar only)"
done

# 3) Подтвердить, что merge feature/avatar → develop = 610bf10eb уже произошёл:
git -C /home/builder/hermes-share/rob_box_project log --oneline origin/develop | grep -F 610bf10eb | head -1
# Ожидаемо: 591b8b4e1 Merge remote-tracking branch 'origin/develop' into develop (содержит 610bf10eb)

# 4) Подтвердить ancestry после merge feature→develop:
for pr in 1632 1610 1622 1618 1607 1633; do
  sha="$(gh pr view "$pr" --repo krikz/rob_box_project --json mergeCommit --jq '.mergeCommit.oid' 2>/dev/null)"
  git -C /home/builder/hermes-share/rob_box_project merge-base --is-ancestor "$sha" origin/develop \
    && echo "PR#${pr}: ${sha:0:8} NOW in develop (post-610bf10eb merge)" \
    || echo "PR#${pr}: ${sha:0:8} STILL not in develop (unexpected)"
done
```

### 7.2 После реализации §4.1 + §4.2

1. **Unit-тест** для ancestry-check в merge-gate (`scripts/agent_flow/tests/test_archive_merged_card_ancestry.sh`):
   - **Кейс A**: PR в `develop` (merged-commit ancestor of origin/develop) → cleanup выполняется.
   - **Кейс B**: PR в `feature/avatar` (merged-commit НЕ ancestor of origin/develop) → cleanup skip с WARN.
   - **Кейс C**: PR в `feature/avatar`, но `feature/avatar` уже merged в develop (ancestor-of-chain через merge-commit) → cleanup выполняется.
   - **Кейс D**: `gh pr view` rate-limit / network error → cleanup fail-OPEN с WARN (отложен до следующего тика).
   - **Кейс E**: `mergeCommit.oid` пустой → fallback `gh api /repos/{owner}/{repo}/commits/{sha}?ref=<base>` → если 200 OK, cleanup; иначе skip.

2. **Integration-тест** на ретро-данных: после merge ADR-0047 в develop — дождаться следующего orphan-cleanup тика (либо ручной `--once` запуск merge-gate), проверить, что 6 AV-* issues **не закрылись преждевременно** (cleanup skip, поскольку merge `feature/avatar` → `develop` уже произошёл, но конкретно эти merge_commit_sha и так есть в origin/develop → cleanup отрабатывает легитимно).

3. **Re-trigger pattern**: создать через `kanban_create` тестовую карточку со ссылкой на `PR #1632` (merge в feature/avatar) до того, как `feature/avatar` сольётся в develop — проверить, что **cleanup skip** в течение следующих 5 мин тиков. После ручного merge `feature/avatar-another` в develop → следующий тик cleanup выполняется.

### 7.3 Метрика для надзора

Подсчёт false-cleanup попыток (PR state=MERGED, но `merge_commit_sha` НЕ в целевой ветке, cleanup skip) логировать как `orphan-scan: skip-feature-branch` в `agent-flow-merge-gate.log`. Если за 24ч > 5 таких skip — открыть ретро-карточку по ключу `target-branch-cleanup-skip-recurring` (см. ADR-0046 §10 — re-verification cadence 90 дней, дополнить скип-метрикой).

## 8. Решение принимает Шифу (Q22)

Этот ADR — **надстройка** над ADR-0046. Acceptance:

- [ ] **Шифу решает** (Q22): принять ADR-0047 целиком; или принять только §4.1 (ancestry-check) без §4.2 (whitelist); или отклонить и оставить cleanup-путь ADR-0046 как есть (с риском false-anomaly повторения, см. §1.4).
- [ ] Если принят — `merge-base --is-ancestor` ancestry-проверка обязательна **в обоих** вариантах ADR-0046 (A и B), если они будут реализованы.
- [ ] Sync 3 копий скрипта через `bash <repo>/scripts/agent_flow/install.sh` + `md5sum` (процедура Шифу от 14.08).
- [ ] После merge — задним числом **никаких cleanup-операций** для 6 AV-* issues не требуется (их merge в develop уже произошёл 28.08, и текущий `pr_base = $DEVELOP_BRANCH` guard в issue-close path корректно их обслуживает). ADR-0047 закрывает **будущие** false-anomaly, не текущий бэклог.

## 9. Источники истины

- `scripts/agent_flow/agent-flow-merge-gate.sh`:
  - **line 1229** — reference implementation `merge-base --is-ancestor` (для stale-after-upstream-fix, ADR-0035). Переиспользовать как шаблон.
  - **lines 3136-3346** — issue-close path с существующим `pr_base = $DEVELOP_BRANCH` guard (line 3138). Дополнить ancestry-check.
  - **line 1568** — `archive_merged_card` (вариант A ADR-0046). Дополнить ancestry-check.
- `docs/adr/0046-orphan-cleanup-after-merge.md` — primary source для orphan-cleanup pattern. Этот ADR — дополнение.
- `docs/adr/0030-adr-numbering-sot.md` — правила выбора номера ADR (0047 — следующий за 0046).
- `~/.hermes/kanban/boards/robbox/kanban.db` — kanban SOT.
- Ретро-карточки: `t_a9b28a7f` (false-anomaly), `t_dc7ce789` (этот ADR), `t_d778653a` (orphan-kanban-cleanup, другая разновидность).
- GitHub feature-ветка `feature/avatar` — контекст для false-anomaly (merge `610bf10eb` 28.08 21:14 +0300).

## 10. Verification log (для будущего надзора)

Дата верификации: 2026-09-02 18:30 CEST.
Метод: cross-ref `gh pr view <N> --json mergeCommit,baseRefName` с `git -C <repo> merge-base --is-ancestor <sha> origin/develop` через Python sqlite3 (sqlite3 CLI не установлен) + `gh api /repos/krikz/rob_box_project/commits/{sha}`.
Результат: 6 AV-* issues ссылаются на PR с `merge_commit_sha` (138affcbc1, b0021b81a1, 1ee4a06f4b, 8629137291, 54fbb21456), которые отсутствуют в `origin/develop` напрямую, но присутствуют через merge-commit `610bf10eb` (feature/avatar → develop, 28.08.2026 21:14 +0300). После `610bf10eb` ancestry-check `true`, до — `false`. Гипотеза ретро `t_a9b28a7f` подтверждена как false-anomaly.

Рекомендация по re-verification: через 90 дней после merge ADR-0047 (если будет реализован) — повторить §7.1 на 6 AV-* issues, убедиться, что они закрыты штатным issue-close path (после merge `feature/avatar` → develop, ancestry-check `true`). Если >2 AV-* issues остаются OPEN после merge — открыть ретро-карточку по ключу `target-branch-cleanup-skip-recurring` (см. ADR-0046 §10 — re-verification cadence, дополнить ancestry-метрикой).
