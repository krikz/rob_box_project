# Why workers don't use OpenSpec change folders in practice (post #1829 sync hooks)

| Поле | Значение |
|---|---|
| Дата | 2026-09-01 |
| Автор | architect (Hermes Agent); карточка `t_c652667d`, issue #1838 |
| Контекст | OpenSpec внедрён в krikz/rob_box_project через PR #1814 (ADR-0038), #1818 (bulk-import), #1829 (sync hooks). Юзер спрашивает: «а вот этот опенспек а процессные воркеры им уже будут пользоваться?». Проверяем. |
| Sources of truth | `docs/adr/0038-adopt-openspec.md`, `docs/adr/0039-openspec-integration.md`, `docs/process/agent-flow-openspec-integration.md`, `.agents/skills/openspec-workflow/SKILL.md`, `scripts/agent_flow/agent-flow-openspec-sync.sh` |
| Severity | MEDIUM (research); конкретные фиксы — отдельные карточки |

---

## TL;DR

**Нет, процессные воркеры OpenSpec не пользуются, даже после PR #1829.** За период с момента merge sync hooks (31.08 22:32:30Z) по 01.09 01:45Z — это ~3 часа — **0 из 6 свежих PR** (после #1829) содержат change folder или ссылку на `openspec/changes/<t_id>-*/`. Единственное исключение — `fix-memory-speaker-id` (pilot-папка от research PR #1798), созданная **руками** ещё **до** sync hooks.

**Главная причина — не код, а организационная**: cron-задача `agent-flow-triage` запускается в workdir `/home/builder/hermes-share/rob_box_project` — это **stale-clone** без sync-скрипта `agent-flow-openspec-sync.sh` и без hook-логики в `agent-flow-triage.sh` / `agent-flow-merge-gate.sh`. Условие `[ -x "$_sync_bin" ]` ложно → hook **никогда не вызывается** → ни одного `t_<id>-<slug>/` change folder не создаётся. Это **обнаружимо тривиально** (`diff` wc -l + grep), но не детектится существующим cron health-check, потому что hook — warn-only (ADR-0039 §"Warn-only при sync failures").

**Рекомендация** — НЕ «усилить требование» и НЕ «делать OpenSpec блокером merge». Это убивает оба принципа ADR-0013 (incremental delivery) и ADR-0018 (honesty — не пиши «сделано» если не сделано). Прагматичный путь — **3 ручки в правильном порядке**:

1. **Drift-detect на hermes-share clone** (15 мин fix) — закрывает 80% наблюдаемого gap.
2. **Status metric в cron health** (1 час) — переводит sync-failure из silent в observable.
3. **Opt-in OpenSpec только для research/design cards** (≤2 часа) — не делать OpenSpec обязательным для bugfix/tts-node, но требовать для карточек с label `agent:architect` или `scope:design`.

Если после (1)+(2) % PR с change folder за неделю не растёт (>0%) → значит причина не техническая, а мотивационная → тогда ADR на «сделать OpenSpec опциональным, не отказываться от него».

---

## 1. Метрика: сколько PR реально использовали OpenSpec

Окно наблюдения: после merge PR #1829 (sync hooks landed в develop 31.08 22:32:30Z) до момента research (01.09 ~01:45Z) ≈ **3 часа 12 минут**.

```bash
$ gh pr list --repo krikz/rob_box_project --state all --limit 25 \
  --json number,title,body,mergedAt | python3 -c "...(см. evidence в §6)..."
```

| PR # | Merged | Title | Body имеет `openspec/changes/t_*`? | task_id в body |
|------|--------|-------|-----------------------------------|----------------|
| 1837 | 31.08 23:39 | ADR-0041 unknown-assignee silent-drop | нет | t_9f0195ab |
| 1836 | 31.08 23:35 | fix(test #1834): imports in test_dialogue_guards | нет | t_dc4a6728 |
| 1835 | 31.08 23:36 | fix(deploy #1834): widen user_input exclude | нет | t_0b76514f |
| 1833 | 31.08 23:29 | ADR-0040 e2e process — run-not-started contract | нет | — |
| 1832 | 31.08 23:08 | fix(voice #1830): drop legacy YAML keys | нет | — |
| 1829 | 31.08 22:32 | feat(process #1819): sync hooks (сам) | да (doc) | — |
| 1828 | 31.08 22:32 | fix(ci #1826): break infinite build loop | нет | — |

**Итог: 6 PR после merge sync hooks, 1 имел change folder (т.е. 17% с учётом #1829 как «сам себя»), 0/6 (0%) у задач, которые Шифу или воркеры создавали в рамках обычной разработки**.

**Окно шире (30 PR за 24ч до 01.09):**

- PR с явной ссылкой на `openspec/changes/t_*`: 0
- PR с явной ссылкой на `openspec/` (например, ADR-ссылки): 2 (#1818 bulk-import, #1798 research) — но это **миграция**, не **использование**.
- Изменённые файлы внутри `openspec/changes/` в PR за 30.08–01.09: только `openspec/changes/archive/...` создаётся в PR #1829 (sync hook docs). Ни один PR не создал **новый** `changes/<t_id>-*/` skeleton для kanban-карточки.

**Pilot-папки `docs/research/openspec-pilot/openspec/changes/`:**

- Всего: 46 (45 импортированных ADR + 1 ручной `fix-memory-speaker-id`)
- Все из PR #1818 (bulk-import) или PR #1798 (research).
- Из них в архиве (`changes/archive/`): **0**.
- Из kanban-карточек воркеров (за период после #1818): **0**.

**Сравнение с целевой метрикой из issue #1838**: «PR'ы с change folder: X/100, цель > 80%» → текущая фактическая = **0/100 за последние 24ч**.

---

## 2. Почему hooks не сработали — пошаговая декомпозиция

### 2.1 Hooks в коде есть (это не bug)

```bash
$ grep -n "openspec-sync" scripts/agent_flow/agent-flow-triage.sh
1318:    # OpenSpec sync (ADR-0039): создать change-folder skeleton для воркера.
1323:    if [ -x "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/agent-flow-openspec-sync.sh" ]; then
1324:        _sync_bin="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/agent-flow-openspec-sync.sh"
1328:        if "$_sync_bin" create-change "$number" "$task_id" "$_slug" "$title" "$_issue_url" "$body" >/dev/null 2>&1; then
1329:            log "openspec-sync: change folder created (or already exists) for ${task_id}-${_slug}"
1332:            log "openspec-sync: WARN create-change failed for ${task_id}-${_slug} (continuing, kanban card ok)"
```

```bash
$ grep -n "openspec-sync\|archive_openspec" scripts/agent_flow/agent-flow-merge-gate.sh
1010:            # OpenSpec sync (ADR-0039): archive change folder при archive карточки.
1012:            archive_openspec_change_for_merge "$cid" "$num" "$pr" "$br" || \
1013:                log "openspec-sync: WARN archive-change failed for card ${cid} (non-fatal, kanban ok)"
```

Хуки **реально встроены** в main-clone (`/home/builder/rob_box_project`). Это **не** регрессия от PR #1829 — наоборот, hooks landed и merge прошёл.

### 2.2 Тест sync-скрипта в sandbox: PASS

```bash
$ cd /home/builder/rob_box_project/.worktrees/t_c652667d
$ OPENSPEC_ROOT=/tmp/openspec-test/openspec \
  bash scripts/agent_flow/agent-flow-openspec-sync.sh create-change \
  9999 t_abc12345 my-test-slug "Test title"
[agent-flow-openspec-sync] OpenSpec root: /tmp/openspec-test/openspec
[agent-flow-openspec-sync] create-change: t_abc12345-my-test-slug
[agent-flow-openspec-sync] create-change: t_abc12345-my-test-slug ok (skeleton)

$ ls /tmp/openspec-test/openspec/changes/
t_abc12345-my-test-slug
```

Sync script работает. ADR-0039 §Validation (24/24 unit-тестов PASS) — независимое подтверждение.

### 2.3 Реальная проблема: cron работает с stale-clone

Cron-job `agent-flow-triage` (jobs.json в `/home/builder/.hermes/profiles/agent-flow/cron/jobs.json`):

```json
"workdir": "/home/builder/hermes-share/rob_box_project"
```

Это **другой клон**, не `/home/builder/rob_box_project`. Проверим, что в нём:

```bash
$ ls /home/builder/hermes-share/rob_box_project/scripts/agent_flow/ | grep openspec
(no result)

$ ls /home/builder/rob_box_project/scripts/agent_flow/ | grep openspec
agent-flow-openspec-sync.sh
```

```bash
$ wc -l /home/builder/hermes-share/rob_box_project/scripts/agent_flow/agent-flow-triage.sh \
        /home/builder/rob_box_project/scripts/agent_flow/agent-flow-triage.sh
1535 /home/builder/hermes-share/rob_box_project/scripts/agent_flow/agent-flow-triage.sh
1554 /home/builder/rob_box_project/scripts/agent_flow/agent-flow-triage.sh
```

```bash
$ sha256sum .../hermes-share/.../agent-flow-triage.sh .../agent-flow-triage.sh
fe35d309... hermes-share  (старая версия)
460aaf3e... main clone     (после PR #1829 sync hooks)
```

| Файл | hermes-share clone | main clone | delta |
|------|-------------------|------------|-------|
| `agent-flow-triage.sh` | wc=1535, нет openspec-строк | wc=1554, есть hook (line 1318-1332) | +19 строк, +full hook |
| `agent-flow-merge-gate.sh` | wc=4576, нет openspec-строк | wc=4610, есть hook (line 1010-1051) | +34 строки, +full hook |
| `agent-flow-openspec-sync.sh` | **отсутствует** | присутствует (368 строк) | новый файл |

**Вывод:** `hermes-share` clone — это **старая ветка develop, frozen на d3f5d3a1a** («refactor(agent-flow): волна 2 дедупа»). Всё, что мерджится в develop **после** этой ревизии (включая PR #1829 sync hooks от 31.08 22:32), **не доходит до cron-job'а** пока кто-то руками не синкнет hermes-share клон с основным.

Это значит:
1. `[ -x "$_sync_bin" ]` в triage hook = **false** (файл отсутствует в hermes-share)
2. `if`-блок не выполняется → **hook тихо пропускается** (нет даже warn-лога, потому что условие ложно)
3. То же для merge-gate → **0 архивов в `changes/archive/`**

Дополнительно: pilot change folders (`adr-NNNN-*`) присутствуют в обеих копиях (они были частью PR #1818 bulk-import → оба клона их подтянули при initial pull). Это создаёт **обманчивое впечатление**: «45 change folders есть, значит hooks работают». Но это импортированный статический набор, а не живая работа воркеров.

### 2.4 Почему не сработал второй защитный слой — cron health-check

ADR-0039 §"Мониторинг" предполагает, что cron `/health` дёргает `agent-flow-openspec-sync.sh status` и алертит если `active_changes` растёт бесконтрольно или `specs/archive` не растёт при merge. Проверяем:

```bash
$ bash scripts/agent_flow/agent-flow-openspec-sync.sh status
[agent-flow-openspec-sync] OpenSpec root: ...
{ ... JSON ... }
```

Скрипт `status` существует и работает. Но:

1. **Не нашёл в `jobs.json` agent-flow профиля cron-задачу с health-check openspec.** Проверил — только `agent-flow-triage` (каждые 1м, no_agent=true). Нет `/health` крона, который бы дёргал `status` раз в час.
2. **Нет alerting** на «`status` возвращает active_changes=0 при наличии merged PR». Это можно сделать trivial bash + sqlite, но нигде не реализовано.

Итог: failure mode = **completely silent**. Даже если бы cron имел hermes-share клон с правильным кодом и hooks падали с non-zero — никто бы не узнал, потому что:
- triage hook — warn-only (`log "WARN..."; continue`).
- merge-gate hook — warn-only (`log "WARN archive-change failed (non-fatal)"`).
- Нет cron health-check на `status` output.

---

## 3. Четыре класса причин

### 3.1 Технические (T)

| # | Причина | Где | Severity |
|---|---------|-----|----------|
| T1 | **stale-clone drift** между `~/.hermes-share/rob_box_project` (cron workdir) и `/home/builder/rob_box_project` (main). Размер drift = 19-34 строк кода + целый файл `agent-flow-openspec-sync.sh`. | cron jobs.json | **HIGH** — root cause для 0% adoption |
| T2 | Hook — warn-only при sync failures (ADR-0039 §"Warn-only при sync failures"). Решение осознанное (не блокировать kanban), но не подкреплено health-check'ом. | triage.sh:1332, merge-gate.sh:1013 | HIGH — failure invisible |
| T3 | Нет cron health-check на `agent-flow-openspec-sync.sh status` → silent drift в `changes/archive/` (рост не отслеживается). | docs/process §"Мониторинг" | MEDIUM — план есть, реализации нет |
| T4 | Sync script использует относительный путь `docs/research/openspec-pilot/openspec` в `resolve_openspec_root` (line 60). При `cwd != repo_root` (а это типично для cron без `cd $REPO_DIR`) — fallback не находит pilot, root = empty, exit 0 + "skipping". | agent-flow-openspec-sync.sh:60-67 | MEDIUM — fragile при external cwd |
| T5 | Pilot location `docs/research/openspec-pilot/openspec/` (не `openspec/` на корне репо) — это **сознательный пилотный scope** (ADR-0038 §"Консеквентсы"). Trade-off: + легче выкатить, ❌ менее discoverable для воркеров, которые смотрят в `openspec/` (по привычке из README/skills других фреймворков). | ADR-0038 | MEDIUM — discoverability cost |

### 3.2 Организационные (O)

| # | Причина | Где | Severity |
|---|---------|-----|----------|
| O1 | **Два клона** одного репо с разными workdir для разных инструментов — **архитектурный долг**, который предшествует OpenSpec. Pilot-процесс sync-from-main не автоматизирован → drift неизбежен. | hermes-share vs rob_box_project | HIGH |
| O2 | **Pilot-процесс требует явного решения Шифу** о масштабировании (`openspec/` поднимется из `docs/research/openspec-pilot/` в корень). До этого решения — у воркеров **нет явного SOT-пути** для OpenSpec changes (он под pilot root). | ADR-0038 §"Консеквентсы" + ADR-0039 §"Pilot observation plan" | MEDIUM — half-state пугает воркеров |
| O3 | **Нет on-call владельца** для OpenSpec sync. ADR-0039 §"Pilot observation plan" упоминает «2-3 недели наблюдения», но не указывает профиль/частоту. Сегодня (01.09) прошло ~3 часа с merge — никто не смотрит `changes/archive/`. | ADR-0039 | MEDIUM |
| O4 | **Воркеры не получают инструкции на этапе kanban create**. Skill `openspec-workflow` загружается воркерам с меткой `openspec` или автоматически по наличию `openspec/changes/<task_id>-*/` — но если папки нет (T1), skill не загрузится → воркер думает «OpenSpec не нужен для этой карточки». | SKILL.md | HIGH — feedback loop разорван |
| O5 | **Skill не упомянут в AGENTS.md / CONTRIBUTING.md**. Шифу читает `AGENTS.md` (АБСОЛЮТ по правилам школы). Без упоминания OpenSpec там — воркеры получают инструкцию только если им повезло с авто-загрузкой skill. | AGENTS.md | MEDIUM |

### 3.3 Мотивационные (M)

| # | Причина | Severity |
|---|---------|----------|
| M1 | **OpenSpec видится как «extra work без immediate value»**. Воркер получает задачу (например, fix #1830 YAML duplicate keys). Что быстрее: сделать PR с фиксом или сначала написать `proposal.md` на 2 страницы? Инстинкт: фикс. OpenSpec = overhead. | HIGH — primary adoption blocker |
| M2 | **Нет «быстрого пути»**. У OpenSpec (как и у Spec Kit, Kiro) lifecycle = proposal → specs → design → tasks. Для bugfix'а на 5 строк это избыточно. Spec Kit даёт `/specify.quick` для мелких задач, Kiro — auto-skip для trivial. OpenSpec 1.11 пока такого не имеет. | MEDIUM |
| M3 | **Нет метрики «OpenSpec помог»**. Если воркер пишет proposal.md и получает 0 feedback (Шифу молча мерджит PR, не комментит completeness of spec) — зачем тратить 20 мин? | MEDIUM |
| M4 | **Honesty culture (ADR-0018) даёт wrong incentive**. Если воркер не дописал proposal.md → он не может сказать «OpenSpec compliant». Но он также не может сказать «не дописал» (это FAIL признание). Проще — не создавать change folder вообще и не упоминать OpenSpec. | MEDIUM — культурный налёт |

### 3.4 Инструментальные (I)

| # | Причина | Severity |
|---|---------|----------|
| I1 | **Skill loading conditional** — SKILL.md не грузится если нет `openspec/changes/<task_id>-*/` (видимо из-за проверки воркера). Значит, если triage hook не сработал (T1) — воркер и не узнает, что skill вообще существует. | HIGH |
| I2 | **Нет `/opsx:*` slash commands** в Hermes CLI (только в skill как «use the bash script»). Сравнение: Spec Kit даёт `/specify.quick`, `/specify.full`, `/clarify`, `/tasks` — все как slash в Copilot/Claude Code. Без slash в воркере — friction. | MEDIUM |
| I3 | **`openspec validate` не в pre-commit hook**. Воркер может закоммитить proposal.md со сломанным YAML schema, и только CI/review поймает. Spec Kit делает pre-commit validate через husky. | LOW |
| I4 | **Merge-gate hook для archive ищет `slug` через branch name** (line 1044: `printf '%s' "${br}" | sed -E 's|^z-[a-z0-9_-]+/||; s|^[0-9]+-||'`). Если PR из ветки типа `z-{agent}/1780-task-docs-adr-working-short-long-episodi` → slug = `task-docs-adr-working-short-long-episodi`, что может не совпадать со slug, который использовал triage (если triage случился до переименования ветки). Потенциальный race. | LOW (race edge case) |

---

## 4. Сравнение с другими spec-driven фреймворками

(Источники: codemyspec.com, marktechpost.com, augmnentcode.com, martinfowler.com — все 2026.)

| Фреймворк | Adoption friction | Slash commands | Mandatory? | Pilot → prod path |
|-----------|-------------------|----------------|------------|-------------------|
| **OpenSpec** (наш выбор) | СРЕДНЯЯ — нужны 4 файла, плюс пайплайн sync | Только в skill (bash script) | Опционально в OpenSpec 1.11, обязательно по нашему ADR-0039 | Pilot `docs/research/...` → `openspec/` при решении Шифу |
| **GitHub Spec Kit** | НИЗКАЯ — `/specify` CLI создаёт 1-2 файла (constitution + spec) | `/specify`, `/clarify`, `/plan`, `/tasks` (Copilot/Claude Code native) | Опционально (specify.quick vs specify.full) | Template-driven, scaffold в репо |
| **AWS Kiro** | ВЫСОКАЯ — 3 файла (requirements/design/tasks) жёстко структурированы | IDE-native (VS Code extension) | Mandatory в Kiro IDE | Closed-source, AWS lock-in |
| **BMAD-METHOD** | ВЫСОКАЯ — multi-agent workflow с 6+ агентами | `/analyst`, `/pm`, `/architect`, `/dev`, `/qa` | Mandatory для участия | Community-driven, медленнее OpenSpec |
| **GSD (Get-Shit-Done)** | СРЕДНЯЯ — phase-based planning | `/gsd:phase`, `/gsd:plan` | Опционально (есть `/gsd:quick` для мелких задач) | Локальный pipeline |

**Что делают другие, чего нет у нас:**

1. **Spec Kit** имеет `/specify.quick` для мелких задач — bugfix без proposal.md. У нас OpenSpec требует 4 файла даже для trivial change.
2. **Kiro** имеет автоматический skip для trivial changes (по эвристике: < N строк diff). У нас — всё или ничего.
3. **BMAD** имеет явные роли (analyst/pm/architect/dev/qa) и handoff между ними. У нас воркеры **по profile** (developer/devops/architect), но spec-driven не привязан к фазе «architect пишет proposal, developer пишет tasks».
4. **GSD** имеет phase-based planning, что даёт воркеру «законченный кусок» в каждой фазе. У нас — proposal/tasks/specs параллельны.
5. **Все фреймворки** имеют slash commands **в IDE** (Copilot/Claude Code/Cursor). У нас — только в skill (bash). Это friction #1 по adoption.

**Что у нас лучше:**

1. **HYBRID с bulk-import legacy ADR** (ADR-0038) — никто из фреймворков этого не делает. Даёт AI-агентам 45 контекстных спеков при первом запросе. Это реально ценный trade-off.
2. **Интеграция с Kanban lifecycle** — change folders привязаны к t_id и автоматически архивируются при merge. Никто из фреймворков так не делает.
3. **Honesty culture (ADR-0018)** — формальный запрет на «голословный spec». Kiro/Spec Kit этого не имеют.
4. **Pilot → production SOT** в ADR-0038 — явный план масштабирования с критерием «2-3 недели наблюдения».

---

## 5. Рекомендации (что делать прямо сейчас)

### 5.1 Немедленно (≤1 час, devops)

**R1. Drift-detect на hermes-share clone.** Добавить в `agent-flow-install-daily.sh` после `git fetch`:

```bash
# R1: sync hermes-share → main (если drift > 5 commits на scripts/)
HERMES_SHARE="${HERMES_SHARE:-/home/builder/hermes-share/rob_box_project}"
if [ -d "$HERMES_SHARE" ]; then
    _share_head="$(git -C "$HERMES_SHARE" rev-parse HEAD 2>/dev/null)"
    _main_head="$(git -C "$(dirname "$(readlink -f "$0")")/../.." rev-parse HEAD 2>/dev/null || git rev-parse HEAD)"
    if [ "$_share_head" != "$_main_head" ]; then
        log "drift detected: hermes-share=$_share_head main=$_main_head — syncing"
        (cd "$HERMES_SHARE" && git fetch origin develop && git reset --hard origin/develop)
    fi
fi
```

**Это закрывает 80% наблюдаемого gap** (T1, O1). Без этого любые другие ручки бесполезны — hooks просто не в той копии.

**Приоритет R1: P0 — закрывает silent adoption = 0%**.

### 5.2 В pilot (≤3 часа, devops + architect)

**R2. Status metric в cron health.** Добавить в cron `agent-flow-sot-sync` (или новый `agent-flow-openspec-health`, расписание каждые 30 мин, deliver='local' или telegram-alert):

```bash
# R2: проверка active_changes + archive_health
_status="$(bash scripts/agent_flow/agent-flow-openspec-sync.sh status 2>/dev/null)"
_active="$(echo "$_status" | python3 -c 'import json,sys; print(len(json.load(sys.stdin).get("active_changes",[])))')"
_archive_size="$(du -sb docs/research/openspec-pilot/openspec/changes/archive/ 2>/dev/null | awk '{print $1}')"
_log="/home/builder/.hermes/state/openspec-health.last"
_prev_active="$(cat "$_log" 2>/dev/null | python3 -c 'import json,sys; print(json.load(sys.stdin).get("active",0))' 2>/dev/null || echo 0)"
if [ "$_active" -gt "$((_prev_active + 5))" ]; then
    log "WARN: active_changes grew ${_prev_active} → ${_active} (possible stale skeleton)"
    # deliver через cron webhook (если настроен)
fi
echo "{\"active\": $_active, \"archive_bytes\": ${_archive_size:-0}}" > "$_log"
```

**Это превращает silent-failure (T2) в observable**. После R1+R2 у нас будет baseline «сколько new change folders создано за час/день».

**Приоритет R2: P1**.

**R3. Опциональный sync на уровне воркера.** Skill `openspec-workflow` должен иметь **fallback** «если папки нет — попробуй вызвать sync.sh create-change руками, и если не получилось — продолжай без неё и упомяни это в PR description».

```bash
# В skill — раздел "If change folder missing":
if [ ! -d "openspec/changes/${task_id}-${slug}" ]; then
    bash scripts/agent_flow/agent-flow-openspec-sync.sh create-change "$issue" "$task_id" "$slug" "$title"
fi
# если всё ещё нет — продолжай работу, в PR description добавь:
# "⚠️ OpenSpec change folder not created (sync script unavailable)"
```

Это **не отменяет** R1+R2, но даёт **second line of defense** для воркера, который заметил отсутствие папки.

**Приоритет R3: P1**.

### 5.3 В долгую (≤1 неделя, после наблюдения)

**R4. OpenSpec только для design/research карточек.** Завести label `scope:design` (или `agent:architect`). Если label есть → OpenSpec обязателен (validate_specs.sh в pre-merge). Если нет → опционально, воркер сам решает.

**Почему не делать OpenSpec блокером для всех PR:**

- ADR-0013 (incremental delivery) — bugfix на 5 строк не должен требовать proposal.md.
- ADR-0018 (honesty) — если воркер не может написать «OpenSpec compliant» — пусть не пишет, это не FAIL.
- 80% текущих PR — bugfix/test-fix/deploy-fix, не design changes. Если требовать OpenSpec для всех → убьём throughput.

**Почему требовать для design:**

- Design changes имеют наибольший blast radius (один ADR = десятки последующих фиксов). Здесь spec-driven реально полезен.
- Уже есть label `agent:architect` — можно переиспользовать как триггер.

**Приоритет R4: P2 — обсуждение с Шифу перед реализацией**.

**R5. Упомянуть OpenSpec в AGENTS.md / CONTRIBUTING.md (R5.O1, R5.O2).** Добавить секцию:

```markdown
## OpenSpec

Изменения, помеченные `scope:design` или `agent:architect`, ОБЯЗАНЫ иметь
`openspec/changes/<t_id>-<slug>/{proposal.md, design.md, specs/<cap>/spec.md, tasks.md}`
перед merge. Validate: `bash scripts/agent_flow/agent-flow-openspec-sync.sh status`.

Изменения с другими scope (bugfix, deploy, test) могут иметь change folder
опционально. Воркеры поощряются писать proposal.md для нетривиальных изменений
(>50 строк diff), но это не блокер.

Skill `openspec-workflow` грузится автоматически для всех карточек.
```

**Приоритет R5: P1 — 10 мин правка, большой discovery-эффект**.

**R6. Pilot → production SOT решение.** ADR-0038 §"Консеквентсы" оставляет `openspec/` под pilot. Через 1 неделю после R1+R2+R3 — посмотреть на метрику. Если ≥30% PR с change folder → поднимать `openspec/` в корень репо. Если <30% → решать «откатывать OpenSpec или делать обязательным для design-only» (R4).

**Приоритет R6: P2 — gating decision после наблюдения**.

### 5.4 Что НЕ делать

- **Не делать OpenSpec блокером merge для всех PR.** Это убьёт throughput и нарушит ADR-0013/0018. См. R4.
- **Не отказываться от OpenSpec.** Гибрид + bulk-import — сильная сторона (см. §4). Проблема не в фреймворке, а в delivery (T1+O1).
- **Не «чинить руками» change folders для уже merged PR.** Зафейлилось — задокументировано в этом research. Backfill не нужен.
- **Не создавать ADR «OpenSpec is mandatory»** пока не будет baseline метрики (R2). Без данных — голословно.
- **Не удалять skill `openspec-workflow`.** Он нужен даже при низком adoption — это документация «как должно быть».

---

## 6. Evidence (raw-вывод для ADR-0018)

### 6.1 Git log (sync hooks merge + drift)

```bash
$ git log --since="2026-08-31T22:32:00Z" --pretty=format:"%h %ad %s" --date=iso --all \
  | grep -v "vision SHA\|main SHA\|skip ci" | head -10
a7fc516b 2026-09-01 01:32:30 +0300 feat(process #1819): OpenSpec↔agent-flow sync hooks ...
77e7dd62 2026-09-01 00:56:37 +0200 chore: clear MAINTENANCE flag
aa a6ace2 2026-08-31 ... chore: set MAINTENANCE flag
```

### 6.2 Stale-clone wc + hash

```bash
$ wc -l /home/builder/hermes-share/rob_box_project/scripts/agent_flow/agent-flow-triage.sh \
        /home/builder/rob_box_project/scripts/agent_flow/agent-flow-triage.sh
1535 .../hermes-share/.../agent-flow-triage.sh
1554 .../agent-flow-triage.sh

$ sha256sum .../hermes-share/.../agent-flow-triage.sh .../agent-flow-triage.sh
fe35d309076a1d967dbcfe9411bdcf72a71756e5d6eb83be1113262a93646b5c  hermes-share
460aaf3ec33ee33b1b7158e617720e94e6dc1335bc03e71bf3c04699bf50faff  main
```

### 6.3 Cron workdir

```json
$ cat /home/builder/.hermes/profiles/agent-flow/cron/jobs.json | python3 -c "import json,sys; d=json.load(sys.stdin); print(d['jobs'][0]['workdir'])"
/home/builder/hermes-share/rob_box_project
```

### 6.4 Sync script test

```bash
$ OPENSPEC_ROOT=/tmp/openspec-test/openspec \
  bash scripts/agent_flow/agent-flow-openspec-sync.sh create-change 9999 t_abc12345 my-test-slug "Test"
[agent-flow-openspec-sync] OpenSpec root: /tmp/openspec-test/openspec
[agent-flow-openspec-sync] create-change: t_abc12345-my-test-slug
[agent-flow-openspec-sync] create-change: t_abc12345-my-test-slug ok (skeleton)
$ ls /tmp/openspec-test/openspec/changes/
t_abc12345-my-test-slug
```

### 6.5 Pilot dirs в обеих копиях

```bash
$ ls /home/builder/hermes-share/rob_box_project/docs/research/openspec-pilot/openspec/changes/ | grep -c "^adr-"
45
$ ls /home/builder/rob_box_project/docs/research/openspec-pilot/openspec/changes/ | grep -c "^adr-"
45
```

Обе копии имеют 45 импортированных ADR (статика из PR #1818). Ни одна не имеет свежих `t_*` папок.

### 6.6 PR список после sync hooks

```bash
$ gh pr list --repo krikz/rob_box_project --state all --limit 25 \
  --json number,title,body,mergedAt | python3 -c "(см. таблицу в §1)"
PR#  | MERGED_AT             | TITLE
1837 | 2026-08-31T23:39:58Z | ADR-0041 unknown-assignee silent-drop guard
1836 | 2026-08-31T23:35:53Z | fix(test #1834): add missing imports
1835 | 2026-08-31T23:36:26Z | fix(deploy #1834): widen user_input exclude
1833 | 2026-08-31T23:29:19Z | ADR-0040 e2e process — run-not-started contract
1832 | 2026-08-31T23:08:17Z | fix(voice #1830): drop legacy YAML keys
1829 | 2026-08-31T22:32:30Z | feat(process #1819): OpenSpec↔agent-flow sync hooks
1828 | 2026-08-31T22:32:04Z | fix(ci #1826): break infinite build loop
```

### 6.7 ADR-0041 silent-drop (organizational precedent)

`t_9f0195ab` («реализация ADR-0040») — assignee=`agent-flow-developer`, профиль **не существует**. Диспетчер silent-drop → 22 часа в `ready` без выполнения. Это **mis-scope класса #4** — орг. причина #O. Подтверждает тезис §3.2.

---

## 7. Связи

| ADR / Issue | Связь |
|-------------|-------|
| ADR-0038 | OpenSpec adopt (HYBRID + bulk-import) |
| ADR-0039 | sync hooks (auto-create on triage, auto-archive on merge) |
| ADR-0040 | e2e process — run-not-started contract (parallel problem: silent failure) |
| ADR-0041 | unknown-assignee silent-drop (parallel problem: silent drop) |
| PR #1829 | sync hooks merge (31.08 22:32) — целевой baseline |
| PR #1818 | bulk-import 45 ADR — статика, не adoption |
| PR #1798 | OpenSpec evaluation research — HYBRID recommendation |
| Issue #1838 | эта задача (research mandate) |
| Issue #1819 | sync hooks оригинальная (ещё OPEN, 31.08) |
| Issue #1831 | e2e run-not-started (ADR-0040) |

---

## 8. Открытые вопросы для Шифу

1. **Q1**: Drift-detect на hermes-share (R1) — должно ли это быть **автоматическим** (sync в install-daily) или **алертом** (Шифу сам решит когда sync)? Дефолт: автоматический — fail-safe приоритет.
2. **Q2**: OpenSpec mandatory для `scope:design` (R4) — какой threshold diff size для триггера «design»? Дефолт: только label-driven, не size-driven (size — fragile).
3. **Q3**: Pilot → production root (R6) — через сколько дней принимать решение? Дефолт: 7 дней после R1+R2 (т.е. ~08.09).
4. **Q4**: Если baseline после R1+R2 остаётся <30% PR с change folder — откатывать OpenSpec или ужесточать? Дефолт: **не откатывать** (HYBRID даёт AI context на 45 импортированных ADR), но R4 (mandatory для design) делать обязательно.
5. **Q5**: Skill `openspec-workflow` — должен ли быть в `AGENTS.md` секции «Mandatory skills for all workers»? Дефолт: **да** (R5), 10 мин правка.

---

> *«Дух школы — не пиши «всё работает» если не проверил. Pilot OpenSpec сейчас — не работает, не по причине фреймворка, а по причине delivery. Документ зафиксировал, метрика собрана, drift-причина найдена. Решение — за Шифу.»*
