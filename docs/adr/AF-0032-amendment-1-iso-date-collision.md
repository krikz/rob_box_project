# ADR-AF-0032 Amendment-1: ISO-date collision in G9a title_prefix + Backfill procedure

> **Статус:** Proposed (требует owner-approval товарища Шифу).
> **Дата:** 2026-10-04.
> **Автор:** techwriter (Hermes Agent, kanban `t_8616bfbb`).
> **Родительский ADR:** [AF-0032 §2.1 «G9a — intra-tick dedup»](AF-0032-triage-dedup-guard.md).
> **Мотивация:** kanban `t_cdd9524a` (decomposed → `t_38515f1b`/`t_845c1b49`/`t_c172f5c9`/`t_8616bfbb`)
> обнаружил, что дефолт `AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=6` **схлопывает разные
> deploy-инциденты** (issue #3354 от 2026-10-03 с issue #3346 от 2026-10-02) как
> «одинаковые по title-prefix». Issue #3354 пролежал 19+ часов без kanban-карточки;
> поймал `t_bf8216cb` (nightly review 2026-W40). Этот amendment фиксирует
> failure mode, mitigations, backfill procedure и alert query, чтобы класс
> багов был закрыт структурно.

## 1. Бизнес-проблема

### 1.1 Что произошло

03.10.2026 05:57Z deploy-bot создал issue **#3354** «🚨 Deploy issues on develop
(staging) — 2026-10-03» (labels: deployment, hermes, agent:devops) — ежедневный
тикет, по шаблону, идентичный #3346 (от 02.10) во всём, кроме даты.

`agent-flow-triage.sh` запустился, дошёл до **G9a intra-tick dedup**
(AF-0032 §2.1) и skip-лит #3354 как «дубль #3346»:

```
[agent-flow-triage] 2026-10-04T02:46:46+02:00 phase1 G9a intra-tick dedup: 1 skip-marker(s)
  skip=#3354 leader=#3346 title-prefix="deploy issues on develop staging 2026"
```

**Group key** (AF-0032 §2.1, реализация `agent-flow-triage.sh:870`):

```python
def title_prefix(t, n):
    s = (t or "").lower()
    tokens = re.findall(r"[\w]+", s, flags=re.UNICODE)
    return " ".join(tokens[:n])
```

`AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=6`. Регекс `[\w]+` отбрасывает эмодзи 🚨,
скобки `()`, длинное тире `—`, дефис в дате. Title-tokenization:

- `#3346` "🚨 Deploy issues on develop (staging) — 2026-10-02" → tokens: `deploy issues on develop staging 2026 10 02` → first 6 = `deploy issues on develop staging 2026`
- `#3354` "🚨 Deploy issues on develop (staging) — 2026-10-03" → tokens: `deploy issues on develop staging 2026 10 03` → first 6 = `deploy issues on develop staging 2026`

**ИДЕНТИЧНЫ** → G9a оставляет #3346 (старейшую), skip-ает #3354.

Skip должен был оставить **комментарий с маркером** `agent-flow:dedup-skip` и
**label** `agent-flow-dedup-skip` (AF-0032 §2.3), но `gh api issues/.../comments`
упал на записи (3 ретрая) → `process_issues_json` exit 1 → **маркер не записан**,
**label не выставлен** → silent fail-OPEN. Issue #3354 «потерялся» в тишине на 19+ часов.

Поймал `t_bf8216cb` (nightly review) — это **первый** watchdog, способный
засечь такой класс. До nightly review (retro W40) класс **не ловился** вообще.

### 1.2 Почему это системный баг, а не единичный сбой

1. **Каждый день** deploy-bot создаёт issue с шаблоном
   `🚨 Deploy issues on develop (staging) — YYYY-MM-DD`. Это **8 токенов**
   (deploy/issues/on/develop/staging/2026/MM/DD), из которых 6 первых —
   **константа**. TITLE_PREFIX_WORDS=6 → **все** ежедневные deploy-issue'ы
   коллидят в одну группу.
2. **AF-0032 §2.1 дизайн-обоснование** (`TITLE_PREFIX_WORDS=6` default) —
   «баланс между "поймать дубли" и "не задеть разные issues"». На момент
   дизайна (15.08–26.08, серии #1477/#1478/#1506/#1562/#1563/#1650/#1653/#1655/#1658)
   deploy-bot ещё не делал ежедневные авто-issue'ы → default не
   тестировался на этом классе.
3. **Side-effect fail-OPEN** без warning — тихий путь. AF-0032 §2.3
   документирует, что `gh`-вызовы skip-marker'а **fail-OPEN**: «если `gh` не
   работает — пропускаем side-effect, всё равно возвращаем leader-stream
   для дальнейшей обработки». Это by design — но **отсутствует**
   compensating control: если fail-OPEN произошёл, **некому** сообщить
   Шифу, что «skip без маркера».
4. **Прогноз без фикса:** 1 пропуск / день, ежедневно. Nightly review
   смягчает, но не устраняет (между review-окнами — 19+ ч видимого «висения»).

### 1.3 Что должен делать amendment

- **Зафиксировать** failure mode как известный класс, а не как разовый сбой.
- **Описать два независимых mitigation'а** (group-key extension, atomic-token)
  с trade-off'ами, чтобы выбрать и merge мог devops-воркер.
- **Зафиксировать backfill procedure** для уже случившихся пропусков
  (когда nightly review нашёл issue без kanban-карточки).
- **Добавить alert query** как secondary watchdog (после nightly review).

## 2. Mitigations (уже в работе в PR)

### 2.1 Mitigation A: расширение group key через `deploy-signature` из body

**Источник:** PR #3385 (devops, kanban `t_235579f1`).
**Идея:** в `dedup_intra_filter` group key = `sorted_labels + "||" + title_prefix + "||" + deploy_signature`,
где `deploy_signature` извлекается из body по regex
`deploy-fail:([\w\-:]+)` (например, `deploy-fail:develop:staging:2026-10-03`).

**Поведение:** issue с разными deploy-сигнатурами (даже при одинаковом
title-prefix) попадают в **разные группы** → не схлопываются.

**CI:** 10 SUCCESS + 1 SKIPPED (Integration Tests not in CI sandbox),
MERGEABLE, готов к merge. Подробности: `gh pr view 3385 --repo krikz/rob_box_project`.

**Trade-off:**

| Pro | Con |
|---|---|
| Решает **точечно**: только deploy-issue'ы с deploy-сигнатурой в body | Требует, чтобы deploy-bot **всегда** клал deploy-signature в body — контракт |
| Не ломает существующую логику для багов (нет deploy-signature → fallback на TITLE_PREFIX_WORDS) | +1 regex pass по body на каждый issue (миллисекунды) |
| Backward-compat: для issues без deploy-signature — старое поведение | Если deploy-bot забудет signature — регресс (но это контракт-вопрос, не код-баг) |

### 2.2 Mitigation B: `YYYY-MM-DD` как атомарный токен

**Источник:** PR #3381 (devops, kanban `t_cdd9524a` parent, ADR-AF-0080 в коммите).
**Идея:** в `title_prefix` парсить ISO-date regex
`(\d{4}-\d{2}-\d{2})` **до** word-tokenization и сохранять как **один атомарный
токен** (например, `date20261003`), не как 3 отдельных токена (2026/10/03).

**Поведение:** при `TITLE_PREFIX_WORDS=6` первые 6 токенов теперь
`deploy issues on develop staging date20261002` (для #3346) и
`deploy issues on develop staging date20261003` (для #3354) → **разные**,
issues в разных группах.

**CI на момент 04.10.2026 03:04Z:** in_progress для части чеков
(Unit Tests ROS2 Humble, Python Code Quality). 8/8 regression в
`test_triage_dedup_iso_date_collision.sh` PASS локально.
Полный прогон нужен перед merge.

**Trade-off:**

| Pro | Con |
|---|---|
| **Универсально**: решает все классы ежедневных/еженедельных авто-issue'ов (не только deploy) | Требует аккуратной regex-обработки (escape-символов, локали) |
| Не требует контракта с bot'ами — работает на любом issue с датой в title | Если в title есть дата в не-ISO формате (например, `02.10.2026` или `Oct 02`) — не поможет |
| Минимальный diff в существующем `title_prefix` | +1 regex pass на каждый issue |

### 2.3 Выбор между A и B

Оба mitigation'а **не конфликтуют** и могут быть **применены вместе**:
- A точечно лечит deploy-issue'ы через контракт с deploy-bot.
- B универсально лечит любые авто-issue'ы с ISO-датой в title.

**Рекомендация (этого amendment'а):** **применить оба, в этом порядке**:
1. Сначала merge A (PR #3385) — **немедленная помощь** для deploy-issue'ов.
   Backfill-карточка t_29682500 уже подтверждает проблему, deploy-сигнатура
   в body — это правильный контракт.
2. Затем merge B (PR #3381) — **универсальная защита** для всех будущих
   авто-issue'ов, даже если deploy-bot забудет signature.

Это staged rollout, аналогичный GATE-2 §7.2 AF-0032 (только NEW events).

**Важно:** этот amendment **не блокирует** ни один из PR — оба mitigation'а
готовятся независимо. Amendment фиксирует **acceptance** (что после merge
оба + backfill procedure становятся обязательными), а не implementation.

## 3. Failure mode: skip без маркера (side-effect fail-OPEN)

### 3.1 Что пошло не так в #3354

AF-0032 §2.3 предписывает при G9a skip'е:

1. Оставить `gh issue comment` с маркером `agent-flow:dedup-skip`.
2. Проставить label `agent-flow-error` (fallback).
3. Проставить label `agent-flow-dedup-skip` (специфичная).

В #3354 `gh api issues/3354/comments` (POST) **упал 3 раза с экспонентой**
→ `process_issues_json` exit 1 → **все 3 side-effect'а пропущены** →
маркер в issue не записан, labels не выставлены.

**Проблема:** AF-0032 §2.3 явно говорит «fail-OPEN: если `gh` не работает —
пропускаем side-effect» (это **by design**, чтобы сеть не блокировала
triage). Но при этом **отсутствует compensating control**: «если
side-effect fail-OPEN произошёл, нужно где-то зафиксировать, что
произошёл skip без маркера».

### 3.2 Что должен делать amendment (compensating control)

Добавить в `agent-flow-triage.sh` после цикла skip'ов **internal log**
с маркером пропуска, **отделённый от gh-side-effect'ов**:

```bash
# В process_issues_json, после цикла G9a:
for skip_record in "${dedup_intra_skipped_records[@]}"; do
    echo "DEDUP_INTRA_FALLBACK skip=${skip_record} gh_side_effect=FAILED ts=$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        >> "$LOG_DIR/agent-flow-triage/dedup-fallback-$(date -u +%Y-%m-%d).log"
done
```

Этот лог — **secondary watchdog**: nightly review может grep'нуть
`dedup-fallback-*.log` за последние 24ч и алертить, если записей > 0
без соответствующих `agent-flow:dedup-skip` маркеров в GitHub.

**Acceptance:** после merge, devops-воркер (новая карточка, см. §7) добавляет:
1. Fallback log writer в `agent-flow-triage.sh` (после G9a loop).
2. Daily cron `dedup-fallback-watchdog.sh`: `grep -c DEDUP_INTRA_FALLBACK
   $LOG_DIR/agent-flow-triage/dedup-fallback-$(date -u +%Y-%m-%d).log` →
   если > 0, comment в `agent-flow-error` issue.
3. Backward-compat: log пишется **только** если gh-side-effect fail (т.е.
   когда compensating control **нужен**), не при успехе.

## 4. Backfill procedure (для пропущенных issues)

### 4.1 Триггер

Nightly review (retro `t_bf8216cb`) нашёл `issue_ref IS NULL` для issue
с label `hermes` (т.е. triage **должен был** его подхватить, но не сделал
это из-за G9a skip'а или другого бага).

### 4.2 Кого звать

- **Primary:** fanout к devops-воркеру (assignee=`agent:devops`, profile=devops).
  Дефолт по stale-candidate-triage.md §"Escalation" §"force-triage" — `agent:backend`,
  но для deploy-issues — `agent:devops` (точнее по домену issue).
- **Secondary (если devops-воркер не доступен):** `agent:backend` (fallback,
  см. ADR-0022 §4.2.1).

### 4.3 Что делает devops-воркер

1. **Проверить root cause** через `agent-flow-triage.log`:
   `grep "skip=${issue_number}" /home/builder/.hermes/logs/agent_flow/triage-*.log`.
2. **Создать kanban-карточку** вручную по шаблону архивных deploy-issue
   (например, `t_76bdb45a` для #3346, `t_0d6f165b` для #3315):
   - title: «🚨 Deploy issues on develop (staging) — YYYY-MM-DD»
   - body: deploy-signature `deploy-fail:develop:staging:YYYY-MM-DD` +
     Vision Pi/Main Pi/rtabmap evidence + cross-link на PR (если есть) +
     meta-issue.
3. **Cross-link** в meta-issue (например, #3374) — comment с marker
   `agent-flow:backfill-applied issue=N t_XXXX`.
4. **Post-mortem** в issue #3354 (или эквивалент): comment с diagnostic
   ссылкой на `docs/diagnostics/YYYY-MM-DD-triage-skip-NNNN.md` +
   краткое summary root cause (1 абзац).

### 4.4 Что делает Шифу (опционально)

- Approve backfill-карточку или назначить assignee.
- Если backfill карточка obsolete (fix уже в merged PR) — закрыть с
  verdict `premise-obsolete` (AF-0066) и cross-link на fix-PR.

### 4.5 Audit trail

Каждая backfill-карточка **обязана** в комментарии `agent-flow:backfill-applied`
содержать:

- `issue=<N>` — какой issue был пропущен
- `kanban_card=<t_XXXX>` — какая карточка создана вручную
- `meta_issue=<#M>` — в каком meta-issue cross-link сделан
- `root_cause_class=<triage-dedup|cron-silent|gh-error|other>` — класс бага
- `fix_status=<in-progress|merged|not-yet>` — статус фикса (если есть)

Этот audit-trail потом используется в nightly review для подсчёта класса
багов и приоритизации fix'ов.

## 5. Alert query (secondary watchdog)

### 5.1 SQL для SQLite (kanban.db)

```sql
SELECT id, title, body, created_at
FROM tasks
WHERE issue_ref IS NULL
  AND labels LIKE '%hermes%'
  AND created_at > datetime('now', '-24 hours')
ORDER BY created_at DESC;
```

**Семантика:** задачи, которые были созданы за последние 24ч, **связаны с
issue'ами** (label `hermes` означает «привязано к issue»), но почему-то
**не имеют ссылки** на issue. Это сигнал, что triage создал карточку
без issue_ref — **очень подозрительно**.

### 5.2 Когда срабатывает

- **Ожидаемое (false-positive):** 0 строк (каждая карточка с label
  `hermes` должна иметь `issue_ref`).
- **Тревожное:** 1+ строка → backfill (см. §4) или расследование (если
  карточка создана **вручную** без `issue_ref` — это violation, отдельный
  аудит).

### 5.3 Где опрашивать

- Cron `dedup-fallback-watchdog.sh` (см. §3.2 acceptance #2) — daily tick.
- Nightly review (`t_bf8216cb`) — primary, ручной grep через
  `sqlite3 ~/.hermes/kanban.db "..."`.

### 5.4 Почему это secondary, а не primary

- **Primary** watchdog — nightly review, потому что он смотрит **в GitHub
  issues**, а не в локальную kanban.db (alert query может пропустить
  issue, который вообще не попал в kanban — что и было с #3354).
- **Secondary** — alert query, потому что kanban.db — это **наше**
  хранилище, и если карточка не создана, alert query не поможет.
- Оба watchdog'а **дополняют** друг друга: nightly ловит «issue без
  kanban-карточки», alert query ловит «kanban-карточка без issue_ref».

## 6. Backfill example (уже применено)

**Первый задокументированный случай применения backfill-procedure** —
issue #3354, kanban-карточка `t_29682500`.

### 6.1 Хронология

| Время (UTC) | Событие | Источник |
|---|---|---|
| 2026-10-03T05:57:01Z | Issue #3354 создан deploy-bot | `gh api issues/3354` |
| 2026-10-03T05:57–04.10T02:46Z | #3354 в OPEN, G9a skip-ит его | `agent-flow-triage.log` |
| 2026-10-04T02:44Z | Nightly review `t_bf8216cb` ловит пропуск | kanban events |
| 2026-10-04T02:46:46Z | DRY-RUN воспроизводит G9a skip (devops) | `t_38515f1b` comment #5975601228 |
| 2026-10-04T03:00Z (примерно) | Devops-воркер создаёт `t_29682500` вручную | kanban events |
| 2026-10-04T03:00+51s | Другой devops-воркер завершает `t_29682500` verdict `premise-obsolete` | `t_845c1b49` parent-handoff |
| 2026-10-04T03:01Z (примерно) | PR #3355 APPROVE (`t_c172f5c9`) | kanban events |
| 2026-10-04T03:18Z | Документ ADR-0022-amendment создан (этот) | `t_8616bfbb` (running) |

### 6.2 Что подтвердилось

- **Backfill-procedure** отработала за **~16 мин** от обнаружения до
  verdict'а (включая cross-link и comments).
- **Premise-obsolete** — корректный verdict, потому что fix уже в
  PR #3355 MERGEABLE+CLEAN (9 SUCCESS + 1 SKIPPED + 2 summary).
- **Nightly review** — primary watchdog, поймал пропуск через 19+ ч.
- **Связанные** карточки (`t_38515f1b`, `t_235579f1`, `t_3381`) — созданы
  по результату backfill'а как follow-up для mitigations A и B.

### 6.3 Lessons learned (для следующих backfill'ов)

1. **Карточка-результат может быть premise-obsolete** — не страшно,
   backfill отрабатывает в обоих направлениях.
2. **Cross-link в meta-issue обязателен** — иначе parent process не
   знает, что backfill применён.
3. **PR-link в карточке обязателен** — чтобы вердикт-воркер видел fix
   сразу и не тратил время на повторное расследование.

## 7. Acceptance для devops-реализации

После owner-approval этого amendment'а devops-воркер берёт в работу:

1. **Merge A (PR #3385)** — расширение group key через deploy-signature.
   Acceptance: `gh pr view 3385` → MERGEABLE + CLEAN.
2. **Merge B (PR #3381)** — YYYY-MM-DD atomic token. Acceptance:
   `gh pr view 3381` → MERGEABLE + CLEAN + все CI чек-ы зелёные
   (включая Python Code Quality, Unit Tests ROS2 Humble).
3. **Добавить fallback log writer** в `agent-flow-triage.sh` (§3.2 acceptance #1).
4. **Создать `dedup-fallback-watchdog.sh`** (новая cron-tick) — daily
   проверка `dedup-fallback-$(date).log` + alert при > 0 (§3.2 acceptance #2).
5. **Backfill procedure §4** — зафиксировать в `docs/runbooks/stale-candidate-triage.md`
   (отдельная секция, уже сделано в этом PR — kanban `t_8616bfbb`).
6. **Alert query §5** — добавить в nightly review script `t_bf8216cb`.

## 8. Acceptance для amendment'а (этот документ)

- [ ] **Owner-approval товарищ Шифу** — выбор между A-first/A+B/A-only/B-only,
  явный комментарий «принято» / «правки».
- [ ] **PR #3385** ready to merge (уже MERGEABLE+CLEAN, ожидает Шифу click).
- [ ] **PR #3381** ready to merge (после полного CI).
- [ ] **Backfill-procedure** в `docs/runbooks/stale-candidate-triage.md` (этот PR).
- [ ] **Alert query** в nightly review script.
- [ ] **Tests** (по аналогии с AF-0032 §5):
  - [ ] Test-1: #3354-style deploy-issue (deploy-signature в body) — НЕ skip-ается
    G9a после merge A.
  - [ ] Test-2: #3354-style deploy-issue (только title-prefix) — НЕ skip-ается
    G9a после merge B (YYYY-MM-DD atomic token).
  - [ ] Test-3: gh-side-effect fail на skip → fallback log записан, в GitHub
    маркер НЕ записан (compensating control работает).
  - [ ] Test-4: nightly review находит issue без kanban → backfill применён за
    ≤ 30 мин → cross-link в meta-issue есть → audit-trail в issue comment есть.

## 9. Rollback

Если amendment создаёт regression (например, deploy-signature в body ломает
какой-то существующий deploy-issue или YYYY-MM-DD atomic token ломает
не-deploy-issue'ы):

1. **Revert merge A:** `gh pr close 3385` или revert commit hash. Deploy-issue'ы
   снова подвержены G9a skip'у — **revert быстро** нужен, потому что риск
   ежедневных пропусков.
2. **Revert merge B:** `gh pr close 3381` или revert. Универсальная защита
   теряется — но если B виноват, это видно сразу (CI на develop красный).
3. **Backfill procedure** — оставить в runbook **независимо** от merge A/B,
   это compensating control, не implementation.
4. **Alert query** — оставить в nightly review **независимо**.

## 10. Связанные

- **ADR-AF-0032** (`docs/adr/AF-0032-triage-dedup-guard.md`) — родительский ADR,
  §2.1 G9a dedup, §2.3 side-effects, §5 acceptance.
- **ADR-0018** — честный FAIL лучше красивого PASS (audit-trail обязателен,
  compensating controls для silent fail-OPEN).
- **ADR-0019** (`docs/adr/0019-agent-flow-triage-already-live.md`) — общая
  модель triage как live-процесса.
- **Issue #3354** — `🚨 Deploy issues on develop (staging) — 2026-10-03`,
  primary пример этого amendment'а.
- **Issue #3346** — deploy 02.10, лидер в G9a skip'е.
- **Issue #3315** — deploy 01.10, fixed by PR #3355.
- **Issue #3374** — meta-issue «agent-flow-triage-cron не подхватил #3354».
- **Issue #3378** — auto-create incident-issue при рецидиве provider-exhaust
  (parallel fix для другого класса).
- **PR #3355** — fix deploy-gate FP (cadvisor + promtail), APPROVE, готов к merge.
- **PR #3385** — mitigation A (deploy-signature в group key), MERGEABLE, 10/11 CI.
- **PR #3381** — mitigation B (YYYY-MM-DD atomic token), MERGEABLE, in-progress CI.
- **Kanban `t_29682500`** — backfill-карточка, **первый задокументированный
  пример применения backfill-procedure** (§6).
- **Kanban `t_bf8216cb`** — nightly review 2026-W40, primary watchdog.
- **Kanban `t_38515f1b`** — root cause, devops (DRY-RUN evidence).
- **Kanban `t_235579f1`** — child devops для implementation mitigation A.
- **Kanban `t_845c1b49`** — manual backfill-карточка (parent-handoff).
- **Kanban `t_c172f5c9`** — PR #3355 review (APPROVE).
- **Diagnostic** `docs/diagnostics/2026-10-03-triage-skip-3354.md` —
  полный raw-evidence report (kanban `t_8616bfbb`).
- **Runbook** `docs/runbooks/stale-candidate-triage.md` — backfill procedure
  секция (kanban `t_8616bfbb`).
- **Retro** `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md` —
  предыдущий аналогичный класс (orphan-stale), другой scope.

## 11. Вердикт techwriter

Рекомендую **применить оба mitigation'а (A + B)** в порядке staged rollout:
1. Merge A (PR #3385) — **первый** (готов).
2. Merge B (PR #3381) — **второй** (после полного CI).
3. Backfill procedure — **в runbook** (уже сделано в этом PR).
4. Alert query — **в nightly review** (отдельный follow-up).
5. Fallback log writer + `dedup-fallback-watchdog.sh` — **отдельный
   devops follow-up** (compensation control).

**Обоснование в одном предложении:** G9a skip-маркер fail-OPEN — это
известный AF-0032 trade-off (by design, чтобы сеть не блокировала
triage), но без compensating control'а этот trade-off **привёл к
19-часовому пропуску** deploy-issue'а. Amendment фиксирует failure
mode, mitigations и backfill procedure, чтобы класс багов был
**закрыт структурно**, а не ловился только nightly review.

**Что нужно от товарища Шифу:**

1. Approve amendment (выбор A-first / A+B / B-only).
2. Merge PR #3385 (готов, 1 click) — это **немедленная помощь** для
   deploy-issue'ов.
3. Approve PR #3381 после полного CI (in_progress).
4. Confirm, что backfill procedure в runbook приемлема как compensating control.
