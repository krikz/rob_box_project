# ADR-AF-0080: agent-flow-triage G9a — YYYY-MM-DD атомарный токен в title_prefix

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR → Accepted) |
| Дата | 2026-10-04 |
| Автор | devops (Hermes Agent); ретро-карточка `t_cdd9524a`, родительская issue #3374 |
| Контекст | `agent-flow-triage` cron **живой и работает** (agent-flow profile: last_run=ok, repeat=37046), но на тике 2026-10-04T02:46:46 ложно объединил issue **#3354** (🚨 Deploy issues 2026-10-03) с **#3346** (🚨 Deploy issues 2026-10-02) как «одинаковый по `(sorted-labels, first-6-words-of-title)`». #3354 стал skip-нут в пользу #3346 → 19+ часов orphan без kanban-карточки. `merge-gate` на каждом тике ругался: `issue #3354 has no kanban marker — triage not finished yet — skip`. PR #3355 уже готов и фиксит root cause (cadvisor+promtail FP), но triage-cron продолжит ломаться на следующих deploy-issue, потому что баг в G9a — он не зависит от того, есть ли уже merged-fix PR. |
| Затрагивает | `scripts/agent_flow/agent-flow-triage.sh` (`title_prefix()` в `dedup_intra_filter`), `scripts/agent_flow/tests/test_triage_dedup_iso_date_collision.sh` (новый, 8 тестов). |
| Родители | ADR-0032 (G9 intra-tick+race dedup, `t_dfd3d19d`), ADR-0018 (честный FAIL), ADR-0013 (incremental-delivery), ADR-AF-0066 (deploy-issue premise-obsolete procedure). |
| Связанные | issue **#3374** (эта ретро), issue **#3354** (orphan 19ч), issue **#3346** (true-positive лидер), PR #3355 (готовый фикс cadvisor+promtail FP), ретро-карточка `t_cdd9524a`, ретро-карточка `t_bf8216cb` (ночной ревью 2026-W40, который и обнаружил orphan). |

## TL;DR

`dedup_intra_filter()` использует `title_prefix(t, n)` для группировки issues по `(sorted-labels, first-N-words-of-title)`. Текущий токенайзер — `re.findall(r"[\w]+", s)` — **отбрасывает** дефис, эмодзи, длинное тире, скобки. ISO-даты `2026-10-02` режутся на 3 токена `["2026", "10", "02"]`, и при `n=6` (default) в prefix попадает только `2026` — одинаковый для разных deploy-issue одной серии.

**Фикс**: выделить `YYYY-MM-DD` как **атомарный токен** ПЕРЕД обычной токенизацией word-chars. Тогда `title_prefix("🚨 Deploy issues on develop (staging) — 2026-10-02", 6)` → `deploy issues on develop staging 2026-10-02` (атомарно!), а для 2026-10-03 — `... 2026-10-03`. Разные deploy-инциденты → разные группы → каждый получает свою kanban-карточку.

**Не делаем**: убирать G9a целиком (overkill — другие серии дублей (#1477/#1478 STT empty on echo) реальны); менять `AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS` default (даже `n=6` теперь достаточно после фикса — атомарная дата занимает 1 слот вместо 3); semantic similarity (ML — overkill).

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

На тике `2026-10-04T02:46:46+02:00` (architect profile, см. `jobs.json[architect/cron/jobs.json]` last_error):

```
[agent-flow-triage] phase1 G9a intra-tick dedup: 1 skip-marker(s)
  skip=#3354 leader=#3346 title-prefix="deploy issues on develop staging 2026"
```

- **#3346** «🚨 Deploy issues on develop (staging) — 2026-10-02» — создан 2026-10-02T06:59:00Z (deploy 2026-10-02 06:50:55 UTC, run 36975533586).
- **#3354** «🚨 Deploy issues on develop (staging) — 2026-10-03» — создан 2026-10-03T05:57:01Z (deploy 2026-10-03 05:52:01 UTC, run 37101202577).

**Оба — разные deploy-инциденты, оба со своим stacktrace**, но `title_prefix(first-6)` для обоих вернул `deploy issues on develop staging 2026`. G9a оставил старейшую (#3346), skip-нул #3354 как «дубль» → 19+ часов без kanban-карточки и PR.

PR #3355 (`z-{agent}/3315-deploy-issues-on-develop-staging-2026-10-01`) готов и закрывает root cause для обеих issues (cadvisor product_name ARM-probe + promtail 1.42/1.44 mismatch). После его мержа deploy-gate перестанет создавать **новые** FP-issue. Но **G9a-баг остаётся**: любой следующий deploy-issue (10-04+) тоже будет skip-нут как «дубль» сегодняшнего, потому что баг в токенайзере, а не в deploy-gate.

### 1.2 Токенизация: что сломалось

`agent-flow-triage.sh:863-866` (до фикса):

```python
def title_prefix(t, n):
    s = (t or "").lower()
    tokens = re.findall(r"[\w]+", s, flags=re.UNICODE)
    return " ".join(tokens[:n])
```

| Token | \w+ | Какой токен получаем |
|---|---|---|
| `🚨` (U+1F6A8) | нет | пропущен |
| `Deploy` | да | `deploy` |
| `issues` | да | `issues` |
| `on` | да | `on` |
| `develop` | да | `develop` |
| `(` `)` | нет | пропущены |
| `staging` | да | `staging` |
| `—` (U+2014) | нет | пропущен |
| `2026-10-02` | да (3 шт.) | `2026`, `10`, `02` |
| `2026-10-03` | да (3 шт.) | `2026`, `10`, `03` |

При `n=6` оба дают: `deploy issues on develop staging 2026` (идентично).

### 1.3 Почему это блокер (а не косметика)

1. **Silent orphan.** В отличие от явных сбоев (HTTP 5xx, exit 1), G9a-баг — **silent**: triage-cron отрабатывает, last_status=ok, repeat инкрементируется, но issue **без карточки**. Ни один watchdog не смотрит «issue OPEN с label hermes, но без kanban-карточки более N часов».
2. **Cascading delay.** `merge-gate` (другой крон) видит #3354, но не может ничего сделать: «triage not finished yet — skip». Каждые 5 минут — один и тот же лог. 19ч = 228 пропусков.
3. **Прячется за «уже пофикшено».** PR #3355 закрывает root cause самих deploy-issue, но не закрывает баг в triage. Следующий deploy (после 03.10) создаст #3360+ с тем же FP — и снова уйдёт в orphan.
4. **Trust erosion.** Шифу после t_bf8216cb (ночной ревью 2026-W40) увидел orphan 19ч — снова под сомнением автоматизация. ADR-0018 запрещает такие сюрпризы.

### 1.4 Гипотеза (root cause)

G9a проектировался в #1477/#1478 (STT empty on echo — у них `first-4 = "stt empty on echo"` идентично) и #1650/1653/1655/1658 (4 devops-worker'а на одном docker-compose.yaml). В обоих случаях title-prefix реально совпадал, и dedup был оправдан. Но для deploy-issue баг никогда не воспроизводился — потому что deploy-issue создаются bot'ом по стабильному шаблону `🚨 Deploy issues on <branch> (<env>) — YYYY-MM-DD`, и до #3346/#3354 таких пар с **разными датами в один день** не возникало.

## 2. Решение

### 2.1 Атомарный ISO-date token

`agent-flow-triage.sh:852-887` (после фикса):

```python
def title_prefix(t, n):
    """Build first-N-token signature of a title for G9a intra-tick dedup.

    Critical (ретро t_cdd9524a, issue #3374, ADR-AF-0080): YYYY-MM-DD даты
    должны быть АТОМАРНЫМ токеном, иначе разные deploy-issue одной серии
    (#3346 «2026-10-02» vs #3354 «2026-10-03») схлопываются как дубли.
    """
    s = (t or "").lower()
    if not s:
        return ""
    iso_re = re.compile(r"\b\d{4}-\d{2}-\d{2}\b")
    word_re = re.compile(r"[\w]+", flags=re.UNICODE)
    tokens = []
    cursor = 0
    for m in iso_re.finditer(s):
        tokens.extend(word_re.findall(s[cursor:m.start()]))
        tokens.append(m.group(0))   # вся дата — ОДИН токен
        cursor = m.end()
    tokens.extend(word_re.findall(s[cursor:]))
    return " ".join(tokens[:n])
```

Результат:
- `#3346` prefix (n=6): `deploy issues on develop staging 2026-10-02`
- `#3354` prefix (n=6): `deploy issues on develop staging 2026-10-03`
- → РАЗНЫЕ → 2 issues в разных группах → каждая получает свою kanban-карточку.

### 2.2 Что НЕ меняем (deliberately)

- **`AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=6`** — оставляем. Даже при n=6 атомарная дата «2026-10-02» занимает 1 слот вместо 3 — и помещается в prefix без увеличения N. ADR-AF-0032 (retros t_dfd3d19d) говорил: «слишком короткий → одинаковые группы у разных багов; слишком длинный → пропускаем реальные дубли (#1477/#1478 STT empty on echo — префиксы совпадают в первых 3-4 словах)». n=6 — это баланс, и после фикса он достаточен.
- **Другие token-классы** — не трогаем. Если в будущем понадобится сохранить issue-id (`#1643`), tag-prefix (`feature:`) и т.п. как атомарные токены — это отдельный ADR (по образцу этого).
- **DRY_RUN/прочее** — не трогаем.

### 2.3 Backward-compat

Старые dedup-серии (без дат) **продолжают работать** как раньше: token-список для них не меняется (в title нет ISO-дат → iso_re не срабатывает → обычная `[\w]+` токенизация). Проверено в regression test T2.

## 3. Альтернативы (рассмотрено и отклонено)

### 3.1 Увеличить `AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS` до 9+

Если бы default был 9, то для `🚨 Deploy issues on develop (staging) — 2026-10-02` tokens=`[deploy, issues, on, develop, staging, 2026, 10, 02]` → first 9 = `deploy issues on develop staging 2026 10 02`. Для `2026-10-03` → `... 10 03`. Различаются. Но:
- Это не лечит **принципиально**: при n=8 баг вернётся.
- Это не поможет для deploy-issue с **двумя** датами в title (например «Re-deploy 2026-10-01 after fix 2026-10-05»): tokens=`[re, deploy, 2026, 10, 01, after, fix, 2026]` — при n=9 не различаются.
- Семантически неправильно: «первые N слов» — слабый дискриминатор. Лучше иметь **семантически осмысленные** атомарные токены.

### 3.2 Отключить G9a целиком (`AGENT_FLOW_DEDUP_INTRA_TICK=false`)

Тогда #1477/#1478 (#1562/#1563 STT empty on echo) вернутся как дубли. ADR-AF-0032 их явно описывает. Не вариант.

### 3.3 Делать date-aware matcher на уровне всего triage

Вместо «атомарный токен» — отдельная проверка «если в title есть `YYYY-MM-DD`, добавить её в группирующий key целиком, минуя token-prefix». Это требует менять сигнатуру `sorted_labels` + `title_prefix` + место их склейки (`agent-flow-triage.sh:888`). Больше кода, больше шансов на регресс. Текущее решение — **локально в `title_prefix`** и **DRY-RUN совместимо**.

### 3.4 Hash от всего title (semantic similarity через embedding)

ML — overkill, требует новой инфры (sentence-transformers + sidecar), плюс в deploy-issue с разными датами косинус-расстояние между «2026-10-02» и «2026-10-03» всё равно высокое — embeddings эту проблему частично решают, но за 100x больше CPU. Не для текущего масштаба (~3 issue/день).

## 4. Verification

### 4.1 Regression test

`scripts/agent_flow/tests/test_triage_dedup_iso_date_collision.sh` — **новый**, 8 кейсов:

- T1: deploy-issue с разными датами (10-02 vs 10-03) → 2 kept, 0 markers (root-bug fix).
- T2: deploy-issue с одинаковой датой (10-02 vs 10-02) → 1 kept, 1 marker (настоящий дубль — backward-compat).
- T3: разные даты в обратном input-order → 2 kept, 0 markers.
- T4: 3 deploy-issue (10-01, 10-02, 10-03) → 3 kept, 0 markers.
- T5: смешанный (10-02 + 10-02-dup + 10-03) → 2 kept, 1 marker.
- T6: title_prefix() сохраняет YYYY-MM-DD атомарно (inline-Python проверка через awk + eval).
- T7: presence check — bug-context в комментариях triage.sh (`t_cdd9524a` / `#3374` / `ADR-AF-0080`).
- bash syntax check.

### 4.2 Pre-existing тесты

- `test_triage_dedup_intra.sh` — 24/24 PASS (все pre-existing кейсы сохраняются).
- `test_triage_dedup_branch_active.sh` — 21/21 PASS.
- `test_triage_dedup_guard.sh` — 27/27 PASS.
- `test_triage_assignee_guard.sh` — 33/33 PASS.
- `test_triage_phase2_gsd_orphans.sh` — 14/14 PASS.
- `test_triage_file_overlap_dedup.sh` — 42/42 PASS.
- `test_triage_issue_resolved_dedup.sh` — 18/18 PASS.
- `test_triage_issue_resolved_guard.sh` — 12/12 PASS.
- shellcheck: origin/develop=48 warnings, current=48 — **no new warnings**.

### 4.3 Pre-existing failures (НЕ наш регресс)

- `test_triage_fingerprint.sh` G. find_duplicate_fix_prs end-to-end — требует OPEN PR #1651/1654/1656/1659, network-dependent. **Не блокирует**.
- `test_triage_force_triage.sh` T17 — hardcoded path `/home/builder/rob_box_project_main/docs/adr/...` (не наша машина). **Не блокирует**.
- `test_triage_phase3_bug_orphans.sh` — timeout 180s, network-dependent. **Не блокирует**.

## 5. Cleanup #3354

После merge этого фикса triage-cron увидит #3354 с правильным prefix'ом (отличным от #3346) и создаст для него kanban-карточку. Это автоматический cleanup.

Однако есть нюанс: PR #3355 уже готов к merge (Шифу ещё не слил). Шифу может пойти двумя путями:

**Путь A (рекомендую):** сначала влить PR #3355 → #3354 закроется автоматически (deploy-gate генерирует новый issue только при новом deploy). Затем влить этот фикс (t_cdd9524a) → следующий deploy-issue корректно триажится.

**Путь B:** влить этот фикс → triage создаст карточку на #3354 → воркер увидит, что PR #3355 уже слит → сделает `premise-obsolete_check` → закроет #3354 с ссылкой на #3355. Это лишняя работа, но технически работает.

## 6. Процессные последствия

- **Bug-class lesson.** G9a был спроектирован под кейсы из августа (без дат). После этого ретро любой дедуп guard, который использует title-prefix, должен явно проверять «а не режет ли токенайзер семантически важные конструкции (даты, ID, версии)?» — и **покрываться тестом на регресс**.
- **Watchdog.** В карточке `t_bf8216cb` (ночной ревью) orphan #3354 обнаружил именно человек. Предлагаю **отдельное задание** (не в этом PR): cron-watchdog «hermes-labeled OPEN issues > 24h без kanban-маркера» → создаёт issue для devops. Это выходит за скоуп этой карточки, но это кандидат для следующего ночного ревью.
- **Multi-profile race.** Дополнительно обнаружено: процесс architect профиля (`c322e1e1`) **зависает** на каждом тике (3+ минуты), держит `/tmp/agent-flow-triage.lock`, и другие профили (agent-flow, devops) скипаются. Это отдельный баг — фикс тоже за скоупом этой карточки, но добавлю в качестве follow-up task.

## 7. Чеклист приёмки (ADR-0018)

- [x] Raw-evidence приложен (лог `architect/cron/jobs.json` last_error, deploy-fail signature, timestamps).
- [x] `pytest`-style test (`bash test_triage_dedup_iso_date_collision.sh`) — 8/8 PASS, raw output сохранён.
- [x] Root cause проверен: токенайзер `[\w]+` режет дефис → 3 токена вместо 1 → first-6 prefix одинаковый.
- [x] Anti-patterns проверены: НЕ убираем G9a (overkill), НЕ увеличиваем default N (deliberately out of scope), НЕ добавляем ML (overkill).
- [x] Воркер не «приукрашивает» — пишет «triage-cron живой и работает», а баг найден в логике, не в инфраструктуре.
- [x] ADR-AF-0080 написан (этот документ).
- [x] Regression test покрывает основной сценарий + 4 edge-кейса.
- [x] PR с минимальным diff (1 функция + 1 новый тест-файл + этот ADR).

## 8. Реализация

- Commit: см. PR (после push).
- Файлы изменены:
  - `scripts/agent_flow/agent-flow-triage.sh` — `title_prefix()` переписан (function-only change, ~30 строк добавлено, ~2 удалено).
  - `scripts/agent_flow/tests/test_triage_dedup_iso_date_collision.sh` — новый, 280 строк.
  - `docs/adr/AF-0080-title-prefix-iso-date-atomic-token.md` — этот ADR.
- Сторона SOT (host): после merge в develop `agent-flow-install-daily.sh` разложит обновлённый скрипт в `~/.hermes/scripts/agent-flow-triage.sh`. Cron следующего тика увидит фикс.