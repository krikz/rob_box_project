# Agent-flow scripts

## Source of Truth (SOT) — единственная точка правки

**`<repo>/scripts/agent_flow/`** (эта папка в репо). Если ты правишь
**здесь** + commit + merge в develop → изменения автоматически
расходятся по хосту через `install.sh`.

**НЕ править руками в:**
- `/home/builder/.hermes/scripts/`
- `/home/builder/.hermes/profiles/agent-flow/scripts/`
- `/home/builder/.hermes/profiles/architect/scripts/`

Эти копии — **hardlink'и** (`cp -al`), которые кладёт `install.sh`. Именно
hardlink, а не симлинк: симлинк в `~/.hermes/scripts/` ресолвится наружу
каталога и отклоняется guard'ом `scheduler.py::_validate_script_path`
(ретро 11.08 `t_a6a236e0d9f0470e` — 50 упавших тиков подряд, 1ч42м
даунтайма). Любая правка в копии уйдёт при следующем `install.sh`.

---

Процессные bash-скрипты, вызываемые по cron в Hermes-профилях
(`agent-flow-merge-gate`, `agent-flow-e2e-process`, `agent-flow-triage`,
`agent-flow-handoff`). Это **наша автоматизация** для issues → kanban →
agent-PR → e2e → close цикла (см. `docs/design/AGENT_FLOW_PROPOSAL.md`).

## ⚠️ КРИТИЧНО: drift между копиями (историческое)

Сейчас кроны стартуют эти скрипты по разным путям (через gateway
разных профилей, см. output `ps -eo pid,cmd | grep agent-flow`):

1. `/home/builder/.hermes/scripts/agent-flow-*.sh` — legacy, что-то стартует ещё
2. `/home/builder/.hermes/profiles/agent-flow/scripts/` — gateway agent-flow
3. `/home/builder/.hermes/profiles/architect/scripts/` — gateway architect
4. `/home/builder/.hermes/profiles/devops/scripts/` — gateway devops (с 21.08)
5. `/home/builder/.hermes/profiles/backend/scripts/` — gateway backend (с 01.09, см. ретро t_a3ba921e)
6. `/home/builder/.hermes/profiles/analyst/scripts/` — gateway analyst (с 01.09, см. ретро t_a3ba921e)

Чтобы избежать drift, **используй `install.sh` для раскладки hardlink-копий**:

```bash
bash <repo>/scripts/agent_flow/install.sh --dry-run   # только посмотреть
bash <repo>/scripts/agent_flow/install.sh             # реальная раскладка
```

После этого все 6 путей (agent-flow / architect / devops / backend / analyst
profiles + `~/.hermes/scripts`) — hardlink-копии (inode) или одинаковое
содержимое. Правка в репо видна везде после следующего `install.sh`.

После раскладки `install.sh` делает жёсткий dual md5+size verify по всем
6 TARGET_DIRS × N EXPECTED-файлам (post_install_verify). Если host-копия
отличается от SOT — `install.sh` завершается с exit code 3 + alert-log
(см. `agent-flow-drift.alert.log`). Раньше эта проверка отсутствовала —
ретро 01.09 t_a3ba921e как раз и вышло из того, что 480 строк
(§4 stale-after-upstream-fix detector) отстали на backend/analyst после
PR #1849 ADR-0035, потому что install.sh их не покрывал. Теперь покрывает,
и post_install_verify это поймает немедленно.

**Контроль дрейфа: `agent-flow-drift-detect.sh`** (cron, every 30m).
Эталон — `origin/develop` (после `git fetch origin develop`), НЕ локальное
дерево: при local!=origin локальное дерево больше НЕ используется как
эталон (ретро 13.08 t_9a3f2e0c — слепота дрейфа host↔origin при
устаревшем local). С 01.09 t_a3ba921e dual md5+size check (а не только
md5 — `compute_drift`). Автофикс через `install.sh`; если не помог —
сразу создаётся kanban-карточка (create_drift_card), не ждём следующего тика.

**BRANCH_ACTIVE (главный worktree на фича-ветке воркера):** при дрейфе
host↔origin автофикс выполняется из ВРЕМЕННОГО worktree на `origin/develop`
(ретро 14.08 t_ea771b06), а НЕ из текущего дерева (оно на z-ветке и
содержит незамерженный код — install.sh из него разнёс бы веточные скрипты):
`git worktree add --detach <wt> origin/develop` → `REPO_DIR=<wt> bash
<wt>/scripts/agent_flow/install.sh` → `git worktree remove --force <wt>`.
Карточка создаётся только если и этот путь не помог (md5+size-сверка после).

## Добавление нового профиля в install.sh

При появлении нового Hermes-профиля, который должен запускать cron-job'ы
(например, `developer`, `pr-reviewer`, `frontend` — все уже существуют,
но без скриптов):

1. Откройте `<repo>/scripts/agent_flow/install.sh`.
2. Добавьте путь профиля в массив `TARGET_DIRS` (после строки 222):
   ```bash
   "/home/builder/.hermes/profiles/<profile>/scripts"
   ```
3. Добавьте комментарий-строку в шапке install.sh (раздел «Копии»).
4. Запустите `bash <repo>/scripts/agent_flow/install.sh --dry-run` —
   post_install_verify покажет «FAIL missing» для ВСЕХ EXPECTED-файлов,
   которые ещё не разложены. Не пугайтесь: это значит, что install.sh
   их ТОЛЬКО ЧТО впервые разложит в новый профиль.
5. Без `--dry-run` — реальная раскладка. Все EXPECTED-файлы окажутся
   во всех 7 TARGET_DIRS.

Профиль создаётся через `profile-create.sh` или вручную (`mkdir -p`).

## Почему install.sh всё-таки drift'нулся на backend/analyst в t_a3ba921e

> Был проведён ретро: install.sh раскладывал 4 TARGET_DIRS, а backend и
> analyst получили скрипты через profile-create.sh (snapshot с 31.08 —
> до MERGE PR #1849 ADR-0035). 480 строк §4 stale-after-upstream-fix
> detector у этих профилей — старая версия; ADR-0035 для них не работало.

После merge PR починки install.sh в t_a3ba921e фикс автоматически
дотянется: каждый cron-job (cleanup-249 / e2e-process / blocked-watchdog /
blocked-watchdog-scope), зарегистрированный на backend и analyst, теперь
получает свежие EXPECTED-скрипты.

## Скиллы воркеров: `sync-skills.sh` (ретро 05.09)

`af_skill_for_profile()` в `lib_agent_flow_common.sh` маппит тип задачи на
repo-скилл (`.agents/skills/<skill>`): `bug`/`type:bug` →
`systematic-debugging`, `type:functional`/`type:feature` →
`test-driven-development`, `type:refactor`/`type:tech-debt` →
`codebase-design`, `type:process` → `agent-flow`. Если type-label нет или
task-скилл не установлен в профиле — fallback на роль (как раньше).

Роль-маппинг (база): `pr-reviewer` → `code-review` (двухосевое ревью diff),
`tester` → `sdlc-review`, `backend`/`devops` → `git-workflow`,
`architect` → `agent-flow-pipeline-ops`. Дополнительно доставляются, но НЕ
являются первичным `--skill`: `to-tickets` (его явно передаёт big-bang guard
architect-карточке), `resolving-merge-conflicts`, `ponytail`.

Но repo-скиллы живут в репо, а профили воркеров их не видят — раньше
доставки не было, и любой скилл из репо улучшал только сессии Шифу.
`sync-skills.sh` закрывает дыру: раскладывает allowlist-скиллы hardlink-ами
в `skills/repo/<skill>/` каждого профиля (категория `repo/` — валидатор
`_validate_skills_for_assignee` ходит рекурсивно и видит её так же, как
runtime skill-loader).

```bash
bash <repo>/scripts/agent_flow/sync-skills.sh --dry-run   # только посмотреть
bash <repo>/scripts/agent_flow/sync-skills.sh             # реальная раскладка
```

- **Allowlist** — `SKILL_SYNC_ALLOWLIST` в `sync-skills.sh` (владелец списка —
  этот файл; `af_skill_for_profile` маппит только на скиллы отсюда):
  `systematic-debugging`, `test-driven-development`, `codebase-design`,
  `verification-before-completion`, `agent-flow`, `code-review`, `to-tickets`,
  `resolving-merge-conflicts`, `ponytail`.
- **Профили** — `SKILL_TARGET_PROFILES` (backend/devops/tester/pr-reviewer/
  architect/agent-flow/analyst); override через `SKILL_SYNC_PROFILES`.
- **Вызывается** install.sh best-effort (после раскладки скриптов) — сбой
  доставки НЕ валит install.sh. Дрифт контролируется через EXPECTED:
  `sync-skills.sh` в списке → `install.sh --list-files` + drift-detect
  следят, что файл не пропал.
- **Идемпотентен** + post-sync md5-verify (exit 3 при расхождении с SOT).

Тесты: `tests/test_sync_skills.sh` (доставка/идемпотентность/exit-коды) и
`tests/test_af_skill_task_type.sh` (маппинг тип→скилл + роль-fallback).

## Скрипты

### `agent-flow-triage.sh` — no_agent=true, every 1m

Тикает каждую минуту (профиль `agent-flow`, см. ADR-0019). Берёт issues с лейблом `hermes`, заводит для
них kanban-карточки на доске `robbox`. Далее диспатчер `hermes gateway`
подхватывает карточки на `ready` и спавнит воркеров под нужный профиль.

### `agent-flow-merge-gate.sh` — no_agent=true, every 5m

Каждые 5 минут сканирует открытые PR с зелёным CI и label `needs-e2e`,
проверяет mergeable-состояние и управляет block/unblock карточек (красный
CI → unblock воркеру; зелёный → ждёт e2e). **НЕ мерджит PR без human review**
(Q22 — только Шифу). Закрывает issue после merge в `develop` (ADR-0014).

**Deploy-issue label-less orphan backstop (ретро 15.08 t_238ff3f7):**
L-Deploy and Verify создаёт deploy-issues с версией workflow-файла С ВЕТКИ
e2e-раунда (`z-{e2e}/test-round-N`). Если round-ветка ответвилась ДО фикса
#1263 (hermes+agent:devops при создании), issue получает только метку
`deployment` → агентский триаж (фильтр по `hermes`) карточку не создаёт →
issue висит open навсегда без обработчика (#1276, round-116). Merge-gate
реконсилит: open deployment-issue без process-меток старше
`DEPLOY_RECONCILE_MINUTES` (default 30м) → добавляет `hermes` + `agent:devops`
→ триаж на следующем тике создаст kanban-карточку. Idempotent: после
добавления `hermes` issue больше не подпадает под правило.

#### ENV-тюнинг

Все дефолты безопасные (24ч dedup, evidence-missing метка уже есть в
репо). Переопределять имеет смысл только в test-сборках / расследованиях.

| Var | Default | Назначение |
|---|---|---|
| `EVIDENCE_REQUEST_DEDUP_HOURS` | `24` | дедуп «worker-evidence request» комментариев (merge-gate шлёт шаблон сразу после `needs-review`, чтобы воркер / pr-reviewer получил чёткие требования к рапорту; см. ADR-0018 «зелёный ≠ ок»). |
| `EVIDENCE_ALERT_AGE_HOURS` | `24` | watchdog-pass `needs_review_evidence_alert_pass_all` сканирует OPEN PR с `needs-review` старше этого порога без worker-evidence комментария → alert + метка. |
| `EVIDENCE_ALERT_DEDUP_HOURS` | `24` | дедуп alert-комментариев watchdog'а (1 раз в окно на 1 PR). |
| `EVIDENCE_MISSING_LABEL` | `evidence-missing` | метка, которую watchdog ставит на PR без worker-evidence рапорта (НЕ gate — PR не блокируется, это сигнал Шифу). |
| `EVIDENCE_REPORT_MARKER` | `worker-evidence report` | substring в тексте комментария, по которому watchdog определяет «уже рапортовал» (contains-режим). Воркер отвечает на request-коммент (или пишет отдельный) с этой строкой, чтобы watchdog перестал флапать. |
| `DEPLOY_RECONCILE_MINUTES` | `30` | возраст deployment-issue без process-меток, после которого merge-gate ставит `hermes`+`agent:devops` (backstop для label-less orphan, ретро 15.08 t_238ff3f7, #1276). |
| `BIG_BANG_MAX_COMMITS` | `50` | ADR-0013: PR > N коммитов ИЛИ > `BIG_BANG_MAX_LINES` строк ЗАПРЕЩЕНЫ без explicit `big-bang-override` label. Enforce на двух уровнях: triage + merge-gate. |
| `BIG_BANG_MAX_LINES` | `3000` | см. выше. |
| `STALE_REBASE_AHEAD_THRESHOLD` | `30` | ретро 22.08 t_562a8682: ahead-of-develop > N через REST compare API → alert в карточку (2ч rate-limit) + comment на issue (24h dedup). Watchdog, не gate. |
| `STALE_REBASE_COMMENT_DEDUP_HOURS` | `24` | дедуп comment-alert. |
| `STALE_REBASE_REMINDER_COOLDOWN_SECONDS` | `7200` | rate-limit alert в карточку воркеру. |
| `NEEDS_FOLLOWUP_LABEL` | `needs-followup` | ретро t_6127fb86: pr-reviewer оставил содержательный review (не approve, не request-changes) → merge-gate явно переводит PR в follow-up режим + kanban-карточка. |
| `GH_REPO` | `krikz/rob_box_project` | owner/repo для всех `gh` вызовов (загружается из `lib_agent_flow_common.sh`). |
| `GH_CONFIG_DIR` | `/home/builder/.config/gh` | путь к gh auth (ретро 03.09 t_a2ce09f8 — force canonical, иначе 401/404 на gh api). |

### `agent-flow-e2e-process.sh` — no_agent=true, every 60m

Главный e2e-процессор. Каждый час берёт issues с label `needs-e2e`,
мержит agent-PR в `z-{e2e}/test-round-N` (создаёт ветку если нет),
триггерит билд→деплой→e2e через `gh workflow run`, ждёт verdict
(`E2E_VERDICT PASS|FAIL` из атомарного харнесса), выставляет лейблы
`e2e-done` / `e2e:rejected` / `e2e:infra-fail`, комментит карточку.

**Deploy-fail → recovery-карточка (ретро 14.08 t_d01fe536):** при
падении деплоя (compose-конфликт, робот недоступен и т.п.) процесс
НЕ просто комментит `errored++` — он создаёт kanban-карточку
`🔧 re-deploy <round> — deploy failed` (assignee=devops, priority 90).
Идемпотентно по round-ветке в title: активная карточка → skip, done/
archived → свежая ready-карточка. Урок: round-109 упал на
`voice-resources-init` compose-конфликте, issue #1229 закрылся БЕЗ e2e
(40 кейсов не гонялись), recovery не создавался.

**Содержит контракт `## e2e` блока в issue** — что воркеры должны
написать в body issue, чтобы процесс нашёл параметры теста (voice_text,
voice, scenario_file, patterns, volume и т.д.). Подробности —
`docs/design/E2E_TESTING_DESIGN_v2.md` §A.10.

**Пауза ротации при известном блокере (ретро 11.08 t_c26b73e7):**
перед созданием нового `test-round-N` процесс проверяет известные
блокеры — открытые issues, в title/body которых есть сигнатура из
`KNOWN_BLOCKER_SIGNATURES` (по умолчанию `no_wake_word` → #1117), и
робот-логи voice-assistant (best-effort, если задан `E2E_ROBOT_PASS`).
Если блокер найден — новый round НЕ создаётся, в каждый needs-e2e issue
публикуется коммент `e2e приостановлен: блокер #N` (идемпотентно), тик
завершается. Дополнительно: если у issue уже `BLOCKER_CONSECUTIVE_FAILS`
(по умолчанию 2) подряд однотипных FAIL с одной сигнатурой (маркер
`e2e-signature: <sig>` в докладах) — ставится `e2e:rejected` с указанием
блокера вместо нового round. Управляется env: `KNOWN_BLOCKER_SIGNATURES`,
`BLOCKER_ROBOT_LOG_SINCE` (default `6h`), `BLOCKER_CONSECUTIVE_FAILS`.

### `agent-flow-handoff.sh` — invoked manually / from kanban

Хелпер для хэндоффа между worker-профилями (например, devops →
architect, или backend → pr-reviewer). Используется редко, в основном
вручную.

### `kanban-retro-create.sh` — dedup-guard для ретро-карточек LLM-кронов (ретро 13.08 t_35ff29f1)

Единственная разрешённая точка создания «ретро: ...» карточек для
LLM-кронов (архитектор-надзор 5c96a6eedf93 и т.п.). Защищает от дублей:

1. **PRE-CHECK**: перед `create` читает `kanban list --json` и ищет
   НЕ-archived карточку с маркером `ретро-key: <key>` в body или точным
   нормализованным title → `SKIP <id>`, create не вызывается.
2. **IDEMPOTENCY-KEY**: create всегда идёт с
   `--idempotency-key "retro:<key>"` — повторный вызов в одном тике
   вернёт существующий id (атомарный гард от гонки).
3. **МАРКЕР**: скрипт дописывает `ретро-key: <key>` в конец body —
   следующий тик с тем же `--key` находит карточку на шаге 1.

`--key` — стабильный slug аномалии БЕЗ дат/времён (например
`e2e-stop-build-runners`); для одной аномалии — один ключ во всех тиках.
Вывод: `CREATED <id>` / `SKIP <id>` / `WOULD_CREATE` (--dry-run).

```bash
~/.hermes/scripts/kanban-retro-create.sh \
  --title "ретро: <аномалия>" --body "<факты+гипотеза+решение>" \
  --assignee <профиль> --skill <скил-из-профиля> --max-runtime 1800 \
  --key <стабильный-slug>
```

### `agent-flow-nightly-review.sh` — ночной ревью-цикл (ADR-0049, 03.09.2026)

`no_agent`, cron `every 1h` (профиль devops), но работает только внутри
ночного окна `[NIGHTLY_REVIEW_HOUR, +NIGHTLY_REVIEW_WINDOW_HOURS)` —
default `[02:00, 06:00)` по локальному времени хоста. Остальные тики стоят
один `date` и `exit 0`.

Закрывает два пробела, которых не покрывает ни один другой контур: (a) нет
среза «что за сутки реально доехало и что осталось висеть»; (b) никто не
перечитывает код, который воркеры за день влили (дубли, глюки LLM,
недоделки, расхождение с ADR/README).

Что делает за тик:

1. Механически собирает дайджест за ревью-сутки (`REVIEW_DATE 00:00`
   локально → now): merged PR, коммиты `origin/develop`, issues
   open/closed, красные CI-прогоны, kanban (закрытые / упавшие / висящие
   >6ч / ретро), churn по компонентам, **last-green метрика E2E Voice Test
   на develop** (см. § «E2E last-green метрика» ниже). Секция без данных
   печатает `НЕТ ДАННЫХ (<причина>)`, а не пустой список.
2. Создаёт ОДНУ карточку **«🌙 ночной ревью \<дата\>»** на `architect`
   (key `nightly-review-<ISO-неделя>`, см. ADR-0049 §6.1 — issue #2159).
3. Создаёт до `COMPONENT_REVIEW_MAX` (default 3) карточек
   **«🔍 ревью компонента: \<comp\>** на `analyst`
   (key `component-review-<slug>-<ISO-неделя>`) — по компонентам с
   наибольшим churn, мимо `COMPONENT_REVIEW_EXCLUDE_RE` (`docs/`,
   `evidence/`, …) и мимо компонентов на кулдауне
   (`COMPONENT_REVIEW_COOLDOWN_DAYS`, default 7 дней).

Обе карточки идут через `kanban-retro-create.sh` (4 слоя дедупа: pre-check
по маркеру, idempotency-key, маркер в body, **issue-label guard для
`nightly-review-*` / `component-review-*`** — issue #2159, ADR-0116). Слой
4: перед create скрипт ищет открытый GitHub issue с label `nightly-review`,
созданный в текущей ISO-неделе (`date -u +%G-W%V` → понедельник 00:00 UTC);
если есть — SKIP, дайджест уже ушёл через issue (читать Шифу удобнее там).
Fail-open (если `gh` недоступен / нет issue с label — пропускаем слой 4,
полагаемся на 1-3). Плюс sentinel
`/tmp/agent-flow-nightly-review.<дата>.done` — «одна ночь = один комплект
карточек». Скрипт НЕ чинит код, НЕ трогает метки/PR/issues и НЕ зовёт
LLM: рассуждения живут внутри созданных карточек.

**Персистентность находок (ADR-0116) — пишет ревьюер, не этот скрипт.**
`agent-flow-nightly-review.sh` создаёт карточку ДО того, как кто-либо
посмотрел на код — он физически не знает, найдёт ли ревьюер дефект.
Поэтому запись находок сделана отдельным шагом ревьюера: тело каждой
карточки требует перед `kanban_complete` вызвать

```bash
scripts/agent_flow/nightly-review-record.sh \
    --task-id t_<id> --component <slug> --outcome <outcome> \
    [--finding '{"type":...,"file":...,"line":...,"symbol":...,"raw":...}']... \
    [--files-changed a.py,b.py]
git add docs/reports/nightly-review/*.jsonl && git commit ... && git push
```

`--outcome` ∈ `open-issue-<N>` | `no-real-defect` | `duplicate-suppressed:<fp>`
(последние два не требуют `--finding`). Скрипт сам считает fingerprint
находки (`sha1(type:file:line:symbol)[:12]`, калька SARIF
`partialFingerprints`) и предупреждает (WARNING, fail-open), если такая
же находка уже трекается открытым issue за последние 30 дней. Пишет ОДНУ
строку в `docs/reports/nightly-review/<review-date>.jsonl`:
`{ts, review_date, iso_week, task_id, component, files_changed,
findings[{type, severity, file, line, symbol, fingerprint, raw}], outcome}`
— переживает merge в git-истории и архивирование kanban-карточки. Скрипт
НЕ коммитит и НЕ пушит — это делает воркер, как и в ADR-0115.

```bash
# сухой прогон в любое время суток (карточки не создаются):
NIGHTLY_REVIEW_FORCE=true NIGHTLY_REVIEW_DRY_RUN=true bash scripts/agent_flow/agent-flow-nightly-review.sh

# ревью за конкретные сутки:
NIGHTLY_REVIEW_FORCE=true NIGHTLY_REVIEW_DATE=2026-09-02 bash scripts/agent_flow/agent-flow-nightly-review.sh

# тест: bash scripts/agent_flow/tests/test_nightly_review.sh
```

### E2E last-green метрика (issue t_b6961c87)

Добавляет в дайджест секцию «## 7. E2E Voice Test (develop)» с однострочным
блоком вида:

```
- L: E2E Voice Test (develop) — последний success: 2026-09-25T19:11:57Z
  ([run](…)), текущий статус: RED (conclusion=failure), consecutive fails: 1
  (HEAD `962262a9` НЕ покрыт последними 20 прогонами — старее retention или
   develop откатился)
```

Архитектор-надзор видит зелёность E2E develop без ручного захода в GH Actions.

- Источник: `GET /repos/{owner}/{repo}/actions/workflows/<file>/runs?branch=develop&per_page=N`
  через `gh api`. Имя workflow-файла содержит пробел — URL-кодируется в `%20`.
- `consecutive_fails` — сколько `failure/timed_out/cancelled` подряд идёт
  от самого последнего `success` (in_progress не считается).
- `current_status`:
  - `GREEN` — последний run `success`;
  - `RED` — последний run `failure/timed_out/cancelled`;
  - `IN_PROGRESS` — последний run ещё выполняется (conclusion пуст).
- Если `head_sha` develop-а покрыт одним из последних N прогонов — отдельная
  ремарка «HEAD \`<sha>\` уже покрыт: conclusion=…»; если нет — «HEAD \`<sha>\`
  НЕ покрыт …» (старее retention или develop откатился).
- Нет данных (`gh` не на PATH, auth упал, workflow runs пуст) → стандартное
  `НЕТ ДАННЫХ (<причина>)` по контракту остальных `section_*`.

ENV-переменные (override):

| var | default | смысл |
|---|---|---|
| `E2E_DEVELOP_WORKFLOW_FILE` | `L-E2E Voice Test.yml` | workflow-файл для last-green (URL-кодируется автоматически) |
| `E2E_DEVELOP_PER_PAGE` | `20` | размер окна прогонов (max retention ~50) |

Pitfalls:

- Не путать `createdAt` (PR/issue list, camelCase) и `created_at` (actions/runs,
  snake_case). У GH Actions API — только snake_case.
- Если в репо несколько develop-веток (`origin/develop`, `origin/develop-old`)
  — секция всё равно берёт ровно `branch=develop`, без альтернатив.
- Секция считается за всё время, не за окно `WIN_START_LOCAL` — «last-green»
  это инвариант HEAD develop, а не срез за сутки. (Ретро-логика: архитектор
  должен видеть «был ли вообще когда-нибудь зелёный develop», а не «что
  изменилось за 24 часа».)

Ночное окно НЕ должно попадать в PEAK-окна `agents_sleep_schedule.conf`
(`04:00-07:00` и `09:00-13:00` MSK) — там висит MAINTENANCE и тик
пропускается. Двигаете PEAK — двигайте `NIGHTLY_REVIEW_HOUR`.

### `validate_pr_scope.sh` — post-PR / pre-merge scope gate (ADR-0095, 14.09.2026)

Блокирует push/PR, если в diff vs `BASE_REF` (default `origin/develop`)
есть файлы вне allowed prefixes. Закрывает **два** класса pollution:

1. **Drift** (ADR-0055, issue #2038) — воркер притащил в PR застрявшие
   коммиты прошлых эпиков (msgpack encoder AV-17, supervisor_state,
   status_hud, tests) — 12 «чужих» файлов в PR #2036.
2. **Post-rebase pollution** (ADR-0095, issue #2444) — после
   `git rebase origin/develop` в HEAD/working tree появились мусорные
   файлы от предыдущих эпиков (#2349 hailo, #2003 pregenerate).
   Классический симптом: 5 из 6 PR в develop содержат мусорные файлы
   не от своего issue.

**Два режима** (переключаются переменной `PR_SCOPE_MODE`):

|| Режим | Diff | Когда использовать | Default |
||-------|------|---------------------|---------|
|| `post-PR` (по умолчанию) | `git diff BASE_REF...HEAD` (трёхточечный) | merge-gate после `gh pr create` | да |
|| `pre-merge` (`PR_SCOPE_MODE=pre-merge`) | `git diff BASE_REF` + `git ls-files --others --exclude-standard` | воркер после `rebase origin/develop`, **до** `git push`/`gh pr create` | нет, явный opt-in |

**Pre-merge режим ловит pollution ДО коммита** — типичный сценарий:
воркер сделал `rebase`, в `git status` появились untracked файлы от
прошлых эпиков. С `PR_SCOPE_MODE=pre-merge` скрипт возьмёт и закоммиченный
HEAD, и staged/unstaged, и untracked — суммарно. Без него — только
HEAD (трёхточечный diff), pollution между rebase и `git add` пройдёт
мимо.

**Регресси-тест** покрывает оба режима: `tests/test_validate_pr_scope.sh`
(11 сценариев: A–J — OK, prefix, prefix+glob, drift, INFO, defensive
MAX_OUT_OF_SCOPE, SKIP, bad base, merge-commit skip, **pre-merge clean,
pre-merge dirty working tree, pre-merge committed branch**).

**Env:**

|| Var | Default | Что делает |
||-----|---------|------------|
|| `PR_ALLOWED_PREFIXES` | (пусто) | comma-separated: `"docs/adr/,src/rob_box_voice/"`. Пусто → INFO-режим (exit 0, печатает файлы). |
|| `PR_ALLOWED_GLOBS` | (пусто) | fnmatch-style: `"*.md,docs/**/*.png"`. Дополняет prefix'ы. |
|| `BASE_REF` | `origin/develop` | эталон; первый позиционный аргумент перекрывает. |
|| `MAX_OUT_OF_SCOPE` | `10` | defensive guard в INFO-режиме (`> MAX` → exit 1 даже без prefixes). |
|| `SKIP_PR_SCOPE` | `false` | opt-out (legitimate fix для смежного файла). |
|| `PR_SCOPE_MODE` | (пусто → post-PR) | `pre-merge` — расширенный режим (working tree + index + untracked vs `BASE_REF`). |

**Exit codes:**

- `0` — OK (нет out-of-scope, или SKIP, или INFO-режим без drift);
- `1` — есть out-of-scope файлы, blocking fail; в stderr — список файлов
  и actionable «fix path» (cherry-pick / пересоздать ветку / opt-out);
- `2` — usage error (нет git, base ref недоступен).

**Использование (воркер вызывает перед `gh pr create` / push):**

```bash
# 1) post-PR gate (по умолчанию) — после коммита, перед push:
PR_ALLOWED_PREFIXES="docs/adr/,src/rob_box_voice/" \
    bash scripts/agent_flow/validate_pr_scope.sh origin/develop
# → OK или FAIL со списком файлов

# 2) pre-merge gate (ADR-0095) — после rebase, ДО push, чтобы поймать
# pollution до коммита:
PR_SCOPE_MODE=pre-merge \
PR_ALLOWED_PREFIXES="docs/adr/" \
    bash scripts/agent_flow/validate_pr_scope.sh origin/develop
# → если FAIL:  git checkout origin/develop -- <junk-files>
#                git commit --amend --no-edit
#                git push --force-with-lease

# 3) INFO-режим — без PR_ALLOWED_PREFIXES (только посмотреть, что в diff):
bash scripts/agent_flow/validate_pr_scope.sh origin/develop

# 4) Opt-out для legitimate fix'а (файл формально вне scope карточки):
SKIP_PR_SCOPE=true bash scripts/agent_flow/validate_pr_scope.sh

# 5) Тест:
bash scripts/agent_flow/tests/test_validate_pr_scope.sh
# ожидаемый итог: All scenarios PASSED exit 0
```

**Регистрация:** в `EXPECTED` `install.sh` → drift-detect контролирует,
что скрипт не пропал из репо. Также см. `worker_scope_check.sh`
(обёртка для воркеров) и `worker_post_flight.sh` (вызывает
post-PR gate перед `gh pr create`).

**См. также:**

- ADR-0055 §3 — оригинальный drift-guard;
- ADR-0095 — pollution-detection rationale + raw-evidence;
- `analysis/diagnose-2444-pr-pollution.md` — диагностика 5/6 PR с мусором;
- `.agents/skills/rebase-pollution-check/SKILL.md` — пошаговый ритуал
  воркера после rebase (`git fetch origin develop` → pre-merge gate →
  `git checkout origin/develop -- <junk-files>` → `git commit --amend`
  → `git push --force-with-lease`).

### `validate_honesty.sh` — pre-PR check на «голословный PASS» (ADR-0018, 18.08.2026)

Сканирует PR body (или файл / stdin) на claim-маркеры (`проверил`, `работает`,
`PASS`, `✅`, `done`, `fixed`, `closes #N`) и проверяет, что рядом есть
raw-evidence (`pytest -v`, `gh run view`, `docker logs`, `sqlite3 .dump`,
`git log --stat`, code-fence). Если claim без evidence — печатает `WARN:`
в stderr. **Всегда exit 0** (не блокер; ревьюер сам решит). `--strict`
превращает в exit 1 для CI.

```bash
# Pre-PR (воркеры прогоняют локально до kanban complete):
bash scripts/agent_flow/validate_honesty.sh --file pr-body.md
bash scripts/agent_flow/validate_honesty.sh --pr 1397      # через gh pr view
cat pr-body.md | bash scripts/agent_flow/validate_honesty.sh

# Тест:
bash scripts/agent_flow/tests/test_validate_honesty.sh
```

**Интеграция:** в `agent-flow-merge-gate.sh` есть helper `honesty_hint_for_pr`,
который дёргает валидатор в e2e-done review и логирует WARN (без блокировки).
В `EXPECTED` `install.sh` → drift-detect контролирует, что скрипт не пропал
из репо. Регистрация: `validate_honesty.sh` в `EXPECTED` (см. PR #1397).

### `validate_adr_namespace.sh` — pre-PR check на ADR namespace collision (ретро 01.09 t_debcb647)

Дополняет `validate_honesty.sh` функцией проверки ADR-нумерации (ADR-AF-0030).
Сравнивает номера **новых** ADR-файлов в diff `origin/develop...HEAD` с
**существующими** номерами в `origin/develop docs/adr/`. Если номер занят
→ exit 1 + actionable сообщение со списком коллизий, slug'ом файла в
baseline и **next free slot** (= max(existing ADR number) + 1).

Это pre-PR версия `check_adr_number_collision()` из `agent-flow-merge-gate.sh`:
merge-gate ловит коллизию ПОСЛЕ открытия PR (и reject'ит + label), этот
скрипт — ДО (`gh pr create` ещё не было), чтобы воркер мог переименовать
`0040-collide.md → 0043-fresh.md` и сразу открыть чистый PR.

```bash
# Pre-PR (воркеры прогоняют локально перед `gh pr create`):
cd /home/builder/rob_box_project
bash scripts/agent_flow/validate_adr_namespace.sh
# → exit 0 (clean) или exit 1 (collision, см. stderr)

# Альтернативный baseline (например для экспериментальных веток):
bash scripts/agent_flow/validate_adr_namespace.sh --ref main

# Тест:
bash scripts/agent_flow/tests/test_validate_adr_namespace.sh
```

**Регистрация:** в `EXPECTED` `install.sh` → drift-detect контролирует.
**НЕ вызывается из merge-gate** (там своя полная реализация с override-метками
`adr-collision-override` и 24h dedup; см. `agent-flow-merge-gate.sh` →
`check_adr_number_collision`). Запускается воркером вручную как часть
локального pre-PR чек-листа.

**Exit codes:**
- `0` — нет коллизии (clean) или нет новых ADR-файлов в diff
- `1` — ADR namespace collision (см. stderr для списка конфликтов и next-free)
- `2` — usage error (неизвестный флаг, baseline не достижим)

### `validate_test_ws_dirs.py` — pre-PR check на молчаливый контракт test_ws (ретро 03.09 t_cfa21388)

`G-Run Tests.yml` собирает CI-workspace `test_ws/` из **подмножества** корня репо:

```yaml
rsync src/ -> test_ws/src/
for d in docker migrations docs .github scripts; do rsync "$d" test_ws/; done
```

Список `for d in ...` — **молчаливый контракт**. Тест, который ходит walk-up'ом
до корня репо и читает корневой каталог ВНЕ этого списка, локально зелёный, а
на CI падает collect-error'ом — и роняет весь батч пакета, а не один тест.

Баг случался дважды в одном файле:

| Карточка | PR | Чего не было | Последствие |
|----------|-----|--------------|-------------|
| `t_29b9ce36` (02.09) | #1874 | `docker/` | `metrics_server.py not found` |
| `t_cfa21388` (03.09) | #1958 | `scripts/` | `score.py not found`, develop RED ~9ч, 20+ PR заблокированы |

Guard закрывает **класс**, а не третий экземпляр: сверяет rsync-список
**каждого** job'а со всеми корневыми каталогами, на которые тесты ссылаются
**от корня репо** (`Path(__file__).resolve().parents[N]` с N до корня, обход
`parents`, или якорь `ROB_BOX_REPO_ROOT`). Package-local каталоги
(`src/rob_box_animations/scripts/`) не считаются — иначе ложные срабатывания.

**Severity:**
- `FAIL` (exit 1) — каталог не скопирован и обращение не защищено `skipif` →
  CI упадёт collect-error'ом. Блокирует.
- `WARN` (exit 0) — обращение под `pytest.mark.skipif(...exists())` → тест
  молча **скипается** на CI. Не блокирует, но это дыра в покрытии.
  На 03.09 таких 4 (`tools/gen_tool_catalog.py`, `test_skill_catalog.py`) —
  тех-долг, отдельной карточкой.

```bash
# Pre-PR (воркеры прогоняют локально перед `gh pr create`):
cd /home/builder/rob_box_project
python3 scripts/agent_flow/validate_test_ws_dirs.py
# → exit 0 (clean / только WARN) или exit 1 (непокрытый каталог)

# Строгий режим — WARN тоже валит (для аудита тех-долга):
python3 scripts/agent_flow/validate_test_ws_dirs.py --strict

# Тест:
bash scripts/agent_flow/tests/test_validate_test_ws_dirs.sh
```

**Регистрация:** в `EXPECTED` `install.sh` → drift-detect контролирует.
**НЕ вызывается из merge-gate** — запускается воркером вручную как часть
локального pre-PR чек-листа (по аналогии с `validate_adr_namespace.sh`).

**Exit codes:**
- `0` — все читаемые тестами корневые каталоги покрыты во всех job'ах
- `1` — есть непокрытый каталог (FAIL), либо WARN при `--strict`
- `2` — usage error (workflow не найден / изменилась структура `for d in`)

### `round_ensure.sh` — ручной валидационный e2e-раунд (ретро 11.08 t_26a6d362)

**Процессное правило:** ручные валидационные раунды devops (проверить
харнесс-фикс на живом роботе, прогнать конкретную команду) — **ТОЛЬКО
через этот скрипт** или `ROUND_ONLY=1` режим `agent-flow-e2e-process.sh`.

Скрипт берёт **тот же flock**, что и автоматическая ротация
(`/tmp/agent-flow-e2e-process.lock`): если e2e-process активен — выход с
ошибкой (`--wait N` ждёт до N секунд). Никогда не создавай round вручную
мимо него — параллельный ручной round + автоматическая ротация на одном
роботе жгут артефакты друг друга (11.08: round-49 FAIL из-за cleanup
артефактов, ложный вердикт #1077).

```bash
bash <repo>/scripts/agent_flow/round_ensure.sh            # печатает z-{e2e}/test-round-N
bash <repo>/scripts/agent_flow/round_ensure.sh --wait 300 # ждать до 5 мин
```

### `round_formation.sh` — единый модуль формирования e2e test-round (issue #2299, 09.09.2026)

**Single source of truth** для ls-remote → max-N → freshness-check → create/
reuse/recreate. До этого та же логика жила копипастой в `agent-flow-e2e-process.sh:round_ensure()`
и `round_ensure.sh` с **противоположной** семантикой записи счётчика (DEFERRED
у автоматики vs IMMEDIATE у ручного — последняя создавала ghost-дрейф на
ручных прогонах, ретро t_d3aeaa9b).

**Канон (09.09.2026):** оба пути (автоматический и ручной) теперь используют
этот модуль с DEFERRED-семантикой — счётчик персистится ТОЛЬКО после
подтверждённого `≥1` запуска раунда (для автоматики — в post-tick cleanup;
для ручного — оператор может вызвать `rf_persist_counter_if_real_round` явно
после успешного e2e run).

**Контракт:**
- `. round_formation.sh` — после `set -euo pipefail` и определения своего `log()`
- Env: `REPO_DIR`, `GH_REPO`, `FOUNDATION_BRANCH` (default: develop),
  `TEST_ROUND_PREFIX` (default: `z-{e2e}/test-round-`),
  `ROUND_COUNTER_FILE` (default: `~/.hermes/state/agent-flow-e2e-round-counter`),
  `DRY_RUN`, `GIT_PUSH_FN` (default: `git_push_with_cred_fallback`).
- API:
  - `round_formation [gh_push_fn_override]` — главная функция. Выставляет
    `ROUND_BRANCH`, `ROUND_FORMATION_CREATED=1` (если создано/пересоздано) или
    `ROUND_FORMATION_REUSED=1` (если reuse), `n` / `max_n` / `counter_n`.
    Counter НЕ пишет.
  - `rf_persist_counter_if_real_round` — записать counter (≥1 run);
    вызывать из post-tick cleanup.
  - `rf_ghost_round_log_and_metric` — маркер `GHOST_ROUND counter_rollback`
    + cumulative metric `agent-flow-e2e-ghost-rounds-total`; вызывать на
    0 run'ов вместо записи counter.

### `agent-flow-cleanup-249.sh` — безопасный cleanup /tmp на build-хосте (ретро 11.08 t_26a6d362)

Удаляет мусор прошлых e2e-ранов на `10.1.1.249` (`yandex_key_*`,
`build_*.log`, `dialog_e2e_*.wav`, `e2e_v2_*`, `voice_e2e_*.log`), но:

1. **не трогает файлы моложе `CLEANUP_MIN_AGE_MIN`** (default 30 мин) —
   активный e2e-прогон пишет свежие `/tmp/e2e_v2_*`;
2. **skip целиком, если e2e-process активен** (локальный flock
   `/tmp/agent-flow-e2e-process.lock` занят);
3. никогда не удаляет `e2e_voice_test.sh` (актуальный харнесс).

Дополнительно (ретро 12.08 t_d3aeaa9b): удаляет **stale round-ветки** на
remote (`z-{e2e}/test-round-N` без e2e-активности > `ROUND_STALE_HOURS`,
default **24ч**; e2e-активность = свежий коммит в ветке, e2e-process пушит
перед каждым прогоном). Guard: тот же flock e2e-process — активный round
не тронем.

Дополнительно (ретро 14.08 t_3cfb3b5b): удаляет **stale PR-ветки** на remote —
ветки, чей PR **MERGED** > `MERGED_STALE_HOURS` (default **2ч**) или **CLOSED**
без merge > `CLOSED_STALE_HOURS` (default **24ч**). Без этого per-card/прочие
ветки копятся вечно (в репо `auto-delete-head-branches` выключен) и мусорят
реконсилейшн PR-сканы. Guard'ы: (a) ветки **OPEN PR** не трогаются никогда;
(b) защищённые ветки (default + `develop`); (c) round-ветки `z-{e2e}/test-round-*`
(их чистит round-sweep); (d) fork-PR (ветка живёт в fork'е); (e) **переиспользование**
— если HEAD-коммит ветки новее момента merge/close её PR (в ветку пушили после
закрытия PR), ветка не удаляется. Проверяется `gh pr list --state merged/closed`
+ `gh api branches`; удаление — `DELETE /git/refs/heads/{branch}`.

**Дополнительно (ретро 22.08 t_deba66ef): auto-retry **blocked-карточек** с
worktree-collision. Раз в cron-tick проходит `hermes kanban list --status blocked`,
для каждой карточки с `branch_name` + `workspace_kind=worktree` делает dry-run
`git worktree add <branch>` (сразу сносит probe). Если ветка свободна
(e2e-process снёс `.worktrees/<id>` после round-cleanup, но dispatcher уже
поставил карточку в `blocked` и больше не пытается) → `hermes kanban unblock <id>`.
Dispatcher подхватит retry на следующем тике.

Закрывает паттерн «autonomous-blocked-loop» (t_50018d92 + t_37134371
застряли 22.08 13:38–15:12, пока надзор не сделал ручной `hermes kanban unblock`).
PR #1518/#1519 закрывают только upstream причину новых дублей — уже-зависшие
карточки остаются stuck.

Guard'ы:
- `branch_name` не пуст + `workspace_kind=worktree` (retro/PR-orphan с
  `null`-branch — другая аномалия, см. t_5e50675b);
- `BLOCKED_MIN_BLOCKED_HOURS` (default **0**) — не unblock'ать совсем свежие;
- `.worktrees/<workspace_basename>` ещё не существует (другая карточка владеет);
- dry-run `git worktree add` (если ветка свободна → unblock).

**Cron (ретро 13.08 t_04d73108):** зарегистрирован в devops-профиле,
`every 6h`, no_agent=true. Регистрация идемпотентно пересоздаётся
`install.sh` (секция "Ensure cron job registration") — не потеряется при
переустановке.

```bash
bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh --dry-run  # показать, что удалит
bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh            # удалить (с guard'ами)
ROUND_STALE_HOURS=48 bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh  # консервативный порог round
MERGED_STALE_HOURS=6 CLOSED_STALE_HOURS=72 bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh --dry-run  # консервативный порог PR-веток
BLOCKED_MIN_BLOCKED_HOURS=1 bash <repo>/scripts/agent_flow/agent-flow-cleanup-249.sh --dry-run  # не unblock'ать карточки младше 1ч
```

### `agent-flow-deploy-sweep.sh` — авто-sweep stale deployment issues (ретро 12.08 t_d3e44336)

**Правило:** deployment issue без апдейтов > `STALE_HOURS` (default 72ч) →
авто-проверка актуальности на живых Pi (SSH) → resolved: close с
комментарием; актуально: авто-метка `hermes` (triage создаст карточку);
проверить нельзя: НЕ трогаем.

**Почему:** деплой-монитор (L-Deploy and Verify.yml) создавал issues
автоматически, но никто не верифицировал/закрывал — висели неделями
(9 штук Jul31–Aug7, ретро-триаж 12.08). Triage фильтрует только по метке
`hermes`, поэтому deployment issues не попадали в конвейер by design.

Идемпотентен: пропускает issues с меткой `hermes`/`e2e-done`/`e2e:rejected`
и свежие (< STALE_HOURS). Понимает `deploy-signature` из body issue
(`deploy-problem:env:scope:container:kind:digest`) и проверяет:
- `container_status` → `docker ps -a` (Up = resolved, restarting = actual)
- `critical_log` → `deployment_issue_dedup.py extract-log` (те же exclude-правила)
- `topic_check` → `ros2 topic list` в контейнере

```bash
bash <repo>/scripts/agent_flow/agent-flow-deploy-sweep.sh --dry-run          # показать, что сделает
STALE_HOURS=72 bash <repo>/scripts/agent_flow/agent-flow-deploy-sweep.sh     # реальный sweep
```

Рекомендуемый cron: `every 6h`, no_agent=true.

### `agents_sleep.sh` — авто-сон агентов по расписанию DeepSeek peak/off-peak (issue #1281)

**Правило:** DeepSeek ввёл peak/off-peak биллинг (с 16.08.2026): PEAK = 01:00–04:00
и 06:00–10:00 UTC = **04:00–07:00 и 09:00–13:00 MSK** (100% цены), остальное —
OFF-PEAK (50%). В пиковые часы все LLM-агенты должны спать (экономия бюджета).

Механизм — существующий MAINTENANCE-флаг в `origin/develop`: если файл есть,
все agent-flow скрипты и промпты спят. Скрипт автоматически ставит/снимает его:

- **PEAK** и MAINTENANCE нет → `touch MAINTENANCE` + commit + push (все спят)
- **OFF-PEAK** и MAINTENANCE есть (с маркером `auto-sleep:`) → `git rm MAINTENANCE`
  + commit + push (проснулись)
- состояние уже правильное → ничего (идемпотентно, пустых коммитов нет)

**Защита ручного maintenance:** скрипт снимает ТОЛЬКО MAINTENANCE, созданный им
самим (маркер `auto-sleep:` в содержимом). MAINTENANCE, поставленный человеком
для live-отладки, не трогается (правило «ничего руками не делай, всё по процессу»).

Расписание — SOT в `agents_sleep_schedule.conf` (правится через PR).

```bash
bash <repo>/scripts/agent_flow/agents_sleep.sh              # тик (cron no_agent, 5–15 мин)
NOW_MSK=06:30 bash <repo>/scripts/agent_flow/agents_sleep.sh  # принудительное время (тест/демо)
DRY_RUN=true bash <repo>/scripts/agent_flow/agents_sleep.sh   # показать решение без git-ops
```

Рекомендуемый cron: `every 5m`, no_agent=true. Тесты:
`bash scripts/agent_flow/tests/test_agents_sleep.sh`.

### `agent-flow-unlabeled-sweep.sh` — авто-sweep stale unlabeled issues (ретро 12.08 t_061d466e)

**Правило:** open issue БЕЗ process-меток (hermes/agent:*/needs-e2e/e2e-done/
e2e:rejected/no-e2e-required/needs-discussion) без апдейтов > `SWEEP_DAYS`
(default 2д) → эвристика по меткам/title/body определяет роль:
- voice/tts/music/audio/stt/vad → `agent:backend`
- ci/deploy/docker/build/workflow → `agent:devops`
- architecture/design/adr/refactor → `agent:architect`

Роль определена и возраст ≤ `MAX_AGE_DAYS` (default 21д) → авто-метки
`agent:<role>` + `hermes` (triage создаст kanban-карточку) + коммент.
Роль НЕ определена или issue слишком старая → только коммент-напоминание
(без `hermes` — не запускаем воркеров на потенциально неактуальные задачи).

**Build-failed issues** (метка build-failure / title «Build Failed») старше
`BUILD_FAILED_CLOSE_DAYS` (default 30д) → проверка, что L-Build Vision/Main Pi
на develop зелёные → resolved: close с комментарием; CI не зелёный → НЕ трогаем.

**Почему:** триаж фильтрует только по метке `hermes`; старые issues без неё
(#918 busy-loop 29.07, #929 OOM, #931/#933 TTS, #1016 музыка и др.) висели
неделями неразмеченными (ретро 12.08 t_061d466e).

Идемпотентен: пропускает process-issues, свежие (< SWEEP_DAYS), уже
размеченные; комментарии дедуплицируются (24h).

```bash
bash <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh --dry-run   # показать, что сделает
SWEEP_DAYS=2 bash <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh # реальный sweep
```

Рекомендуемый cron: `every 12h`, no_agent=true.

#### GraphQL → REST fallback при rate-limit (ретро 10.09, t_291506bf)

`gh issue list --json` ходит в **GraphQL** API. У GraphQL отдельный бюджет
(5000 points/час), выгорающий независимо от REST — при активном
agent-flow (merge-gate каждые 5 мин, triage каждую минуту) он регулярно
уходит в ноль на 1–3 часа в сутки:

```
$ gh api graphql -f query='{rateLimit{limit,remaining,resetAt}}'
{"data":{"rateLimit":{"limit":5000,"remaining":0,"resetAt":"2026-09-09T22:25:38Z"}}}
```

Старый код листинга был `gh issue list ... 2>/dev/null || echo '[]'` —
в rate-limit это давало **пустой массив, `considered=0` и exit 0**:
скрипт рапортовал успех, ничего не сделав (silent-fail). Наблюдаемое
последствие 10.09: PR #2340 и #2338 висели без меток `needs-e2e` /
`no-e2e-required`, merge-gate не мог их провести.

Текущее поведение:

1. **GraphQL** (`gh issue list --json`) — основной путь; ответ обязан быть
   валидным JSON-массивом (проверка `is_json_array`), иначе считается сбоем.
2. **REST fallback** — `gh api repos/{owner}/{repo}/issues?state=open&per_page=100`
   (свой лимит 5000 req/час, GraphQL не трогает). Ответ нормализуется в схему
   `gh issue list --json` (`updated_at`→`updatedAt`, `created_at`→`createdAt`);
   записи с ключом `pull_request` отбрасываются — REST `/issues` отдаёт и PR.
   В лог пишется `WARNING: ... falling back to REST` + `issues listing source=rest`.
3. **Оба сбоя** → `ERROR: обе ветки листинга issues отказали (...)` +
   `tick done: ... errored=1 source=none` + **exit 1** (fail-closed).
   Cron видит ненулевой код и может алертить, вместо тихого `considered=0`.

В каждом тике теперь печатается `issues listing source=graphql|rest` — по
логам видно, как часто мы упираемся в GraphQL-лимит.

Регресс-покрытие: `tests/test_unlabeled_sweep.sh` T12 (GraphQL rate-limit →
REST, `considered>0`, PR-записи отфильтрованы) и T12b (двойной сбой → exit 1).

### `cron-loop.sh` — низкоуровневый цикл

Тонкая обёртка над cron-вызовами (используется как fallback когда
Hermes-cron недоступен). Маленький, 854 байт.

### `watchdog.sh` — heartbeat агентов, every 2m

Не «сторожевой таймер для e2e-build/deploy» (так было написано здесь до
30.08 — описание не совпадало с кодом). Реально: целостность kanban-БД,
залипшие карточки (heartbeat старше 10 мин), перезапуск диспетчера, когда
running нет, а ready есть, recovery-карточка на умершего воркера, prune
мёртвых PID, `RUN_NOW`-триггер немедленного e2e-прогона и block/unblock
карточек при исчерпании провайдера (402/429 в логе воркера).
Пустой stdout = тихий тик, токены не тратятся.

### `watchdog-provider-quick.sh` — тот же provider-guard, но every 1m

Горячий путь для исчерпания провайдера: 2-минутного скана `watchdog.sh`
не хватает, чтобы среагировать до `consecutive_failures=2` → `gave_up`
(ретро 24.08 `t_4c73490f`). Делит с `watchdog.sh` `PROVIDER_MARKERS` и
логику recovery-волны, плюс маркеры HTTP 401 / Authentication Fails.

### `agent-flow-e2e-drift-watchdog.sh` — метрика PR↔issue drift

Только читает. Считает PR с меткой `e2e-done`, у которых issue уже вернулась
в ротацию (`needs-e2e`) — то есть reconcile в merge-gate не сработал.
`exit 1`, если максимальный drift старше `DRIFT_THRESHOLD` (30 мин).
Reconcile делает merge-gate; этот скрипт — единственный способ увидеть,
что тот НЕ сделал.

### `agent-flow-e2e-fail-streak-watchdog.sh` — auto-escalation + auto-create issue

> Каноническое имя секции по карточке t_e72760e9: **«Fail-streak auto-issue»**
> (ADR-FS-001, kanban t_401e52de). Ниже — auto-escalation (comment + pause)
> и сам auto-create-issue, единый watchdog.

При fail-streak ≥ `E2E_FAIL_STREAK_WARN` (default 5):

1. **Comment-alert** в открытый `needs-e2e` / unlabeled-process issue (idem­po­tent по
   `E2E_FAIL_STREAK_DEDUP_HOURS`, default 6ч).
2. **Auto-create issue** с лейблом `e2e-fail-streak` (ADR-FS-001, kanban t_401e52de).
   Body: timeline последних 8 failed runs (id/conclusion/createdAt/headSha[7]/headBranch),
   develop HEAD, релевантные merged PR за 5 дней (парсятся из
   `git log origin/develop --merges`), hypothesis `music-fix regression` со ссылками на
   `#2246`/`#2347`. Два guard'а:
   - **Rate-limit** (mtime `ISSUE_COOLDOWN_FILE = $HERMES_HOME/state/agent-flow-e2e-fail-streak-last-issue`):
     если файл младше `E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS` (default 4ч) — skip.
   - **GitHub-truth**: `gh issue list --label e2e-fail-streak --state open --limit 1`
     уже возвращает 1+ → skip (защита от дублей при потере state-файла).
3. **Auto-pause** (при streak ≥ `E2E_FAIL_STREAK_PAUSE`, default 20) — sentinel-файл
   `$HERMES_HOME/state/agent-flow-e2e-fail-streak-pause`, который
   `agent-flow-e2e-process.sh` читает в начале каждого tick и пропускает round
   creation. Manual override — удаление файла.

#### State-файлы (rate-limit + pause-sentinel)

Оба файла живут под **`$HERMES_HOME`** (default `~/.hermes`, переопределяется
через `HERMES_HOME` в env крона). Фиксированные пути:

| Файл | Назначение | Очистка |
|---|---|---|
| `$HERMES_HOME/state/agent-flow-e2e-fail-streak-last-issue` | epoch последнего успешного `gh issue create` (Unix sec). Используется для rate-limit guard. | Удалить, чтобы следующий тик снова мог создать issue (или подождать `E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS` ч). |
| `$HERMES_HOME/state/agent-flow-e2e-fail-streak-pause` | sentinel «заморозить e2e-ротацию», созданный при streak ≥ PAUSE. Содержит `streak=N`, `triggered=<iso>`, инструкцию по ручному override. | **Только вручную** (см. комментарии в самом файле). Удаление до фикса регрессии → следующие 20+ FAIL'ов сожгут CI minutes. |

> ⚠️ Путь из task body `/var/lib/agent-flow/e2e-fail-streak-issue.last` — это
> абстрактный «systemd-style» reference; реальный скрипт использует
> `$HERMES_HOME/state/...` (см. ENV-таблицу скрипта: `ISSUE_COOLDOWN_FILE`,
> `PAUSE_SENTINEL`). Это намеренно: на хосте Hermes конфиг/стейт
> концентрируется под `~/.hermes/`, чтобы `install.sh` мог раскладывать
> скрипт без `root` и без отдельной `/var/lib` договорённости.

#### Требования к окружению

* **`gh` CLI установлен** (`command -v gh`). Watchdog в начале тика делает
  `gh auth status` — если не `logged in`, exit 1 и тик молча завершается
  (см. comment-alert: cron-delivery сработает, но issue НЕ создаётся).
* **GitHub-токен с scope `repo` (full control of repositories)** — нужен и
  для чтения `gh run list`, и для создания issue. На нашем devops-профиле
  это Personal Access Token пользователя `krikz` (см. `gh auth status` →
  `Token scopes: '...', 'repo', ...`).
* **`read:org`** — нужен ТОЛЬКО если `E2E_FAIL_STREAK_ISSUE_ASSIGNEE` задан
  как `@org-member`. По умолчанию assignee пустой, scope не требуется.
* **`python3`** — используется для парсинга JSON-ответов `gh run list` /
  `gh issue list` (streak counter + timeline table). Без python3 скрипт
  падает на первом же parse с `ERROR: cannot parse runs json` → exit 1.
* **`flock`** (util-linux) — guard от параллельного запуска (`LOCK_FILE`).
* **Repo path через `REPO_DIR`** — для `git -C origin/develop ...` (develop
  HEAD + релевантные merges в issue-body). Если пусто, fallback на `git`
  без `-C` (текущий cwd), что работает только если watchdog запущен
  ИЗ корня репо.

#### ENV-тюнинг (все с разумными дефолтами)

| Var | Default | Назначение |
|---|---|---|
| `E2E_FAIL_STREAK_WARN` | 5 | порог для comment + auto-create issue |
| `E2E_FAIL_STREAK_PAUSE` | 20 | порог для pause-sentinel |
| `E2E_FAIL_STREAK_ISSUE_THRESHOLD` | 5 | порог для auto-create issue (= WARN, можно поднять) |
| `E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS` | 4 | rate-limit создания issue |
| `E2E_FAIL_STREAK_ISSUE_LABEL` | `e2e-fail-streak` | лейбл нового issue |
| `E2E_FAIL_STREAK_ISSUE_ASSIGNEE` | `` (пусто) | assignee issue (опц.) |
| `E2E_FAIL_STREAK_DEDUP_HOURS` | 6 | дедуп alert-комментариев |
| `REPO_DIR` | `` (cwd) | путь к локальному clone репо для `git -C` (develop HEAD + merges) |
| `HERMES_HOME` | `~/.hermes` | корень для state-файлов (cooldown + pause) |
| `GH_REPO` | `krikz/rob_box_project` | owner/repo для всех `gh` вызовов |
| `LOCK_FILE` | `/tmp/agent-flow-e2e-fail-streak-watchdog.lock` | flock guard |
| `FAIL_STREAK_DRY_RUN` | `false` | **DRY-RUN** — log only, никаких `gh issue comment` / `gh issue create` / touch sentinel. См. «DRY-RUN для оператора» ниже. |

#### Как отключить или сильно ослабить watchdog

* **Полностью отключить auto-create issue** (но оставить comment-alert и
  pause):
  ```bash
  E2E_FAIL_STREAK_ISSUE_THRESHOLD=999999 bash scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh
  ```
  Threshold выше максимально наблюдаемого streak → условие
  `streak ≥ THRESHOLD` никогда не сработает, comment + pause продолжат
  работать как раньше.
* **Полностью отключить весь watchdog** (ни comment, ни issue, ни pause):
  убери `agent-flow-e2e-fail-streak-watchdog.sh` из chain вызовов
  `agent-flow-e2e-process-launcher.sh` (см. комментарий-строку в `install.sh`,
  раздел «Fail-streak escalation watchdog»). Альтернатива — обнулить все три
  порога: `E2E_FAIL_STREAK_WARN=999999 E2E_FAIL_STREAK_ISSUE_THRESHOLD=999999 E2E_FAIL_STREAK_PAUSE=999999`.
* **Сменить rate-limit window** (как требует task body): через
  `E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS`. Пример: «не чаще раза в сутки»:
  ```bash
  E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS=24 bash scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh
  ```
* **Сменить cooldown срочно** (например, нужно СЕЙЧАС создать issue при
  следующем тике, не дожидаясь окна): удалить state-файл
  `rm -f "$HERMES_HOME/state/agent-flow-e2e-fail-streak-last-issue"` —
  следующий тик увидит «cooldown absent» и пройдёт второй guard
  (GitHub-truth: нет открытого issue с лейблом `e2e-fail-streak`).
* **Снять pause-sentinel** (после fix регрессии): удалить файл
  `rm -f "$HERMES_HOME/state/agent-flow-e2e-fail-streak-pause"`. Это
  ручное действие намеренно — следующие 20+ FAIL'ов без подтверждения
  фикса сожгут CI minutes (см. ADR-0018 + sentinel header).

#### DRY-RUN для оператора

Перед любым изменением watchdog (новый ENV-тюнинг, новая версия скрипта,
эксперимент с threshold'ами) — **сначала прогнать DRY-RUN**, чтобы убедиться,
что тик видит streak и action-ветки логируются без side-effect:

```bash
# На хосте, где живёт cron-tick (default /home/builder):
FAIL_STREAK_DRY_RUN=true bash /home/builder/.hermes/scripts/agent-flow-e2e-fail-streak-watchdog.sh

# Или прямо из репо (после install.sh раскладки):
FAIL_STREAK_DRY_RUN=true bash scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh
```

В DRY-RUN:
* `gh issue comment …` → логируется `DRY-RUN would: gh issue comment …` (no API call);
* `gh issue create …` → логируется `DRY-RUN would: gh issue create --label …`;
* pause-sentinel `touch …` → логируется `DRY-RUN would: touch …` (файл НЕ создаётся);
* cooldown `date +%s > "$ISSUE_COOLDOWN_FILE"` НЕ выполняется → следующий
  реальный тик НЕ будет «заблокирован» от DRY-RUN'а.

Все остальные side-effects (`gh run list`, `gh auth status`, `git -C …`)
выполняются как обычно — это READ-операции, безопасные.

#### Тесты / регресс-гард

Регрессионные unit-тесты через PATH-hijack mock-gh / mock-git (без сети,
без реальных issues) — `scripts/agent_flow/tests/test_e2e_fail_streak_auto_issue.sh`
(10 кейсов: streak<threshold, DRY-RUN, fresh/stale cooldown, existing issue,
create-call correctness, 8-fails→1-issue acceptance, gh-failure handling,
assignee, marker).

Запуск:

```bash
bash scripts/agent_flow/tests/test_e2e_fail_streak_auto_issue.sh
# ожидаемый итог: PASS=10 FAIL=0 exit 0
```

Тесты можно гонять **в любом окружении** (включая CI без `gh` auth) — mock-gh
лежит в `mktemp -d/bin/gh` и не уходит в сеть. Это самый дешёвый способ
проверить, что очередной refactor watchdog'а не сломал идемпотентность / DRY-RUN
/ cooldown guards.

### `worker_pre_flight.sh` — rebase pre-check (issue #2438, 2026-09-14)

Воркер-helper: вызывается в самом начале сессии (после `cd` в worktree)
ПЕРЕД началом кода. Делает `git fetch origin develop --prune`, считает
`BEHIND=$(git rev-list --count HEAD..origin/develop)`, и если BEHIND превышает
`MAX_BRANCH_BEHIND` (default 30) — пишет warn в `task_comments` (`gh issue
comment` если задан `ISSUE_NUM`) и ДЕЛАЕТ auto-rebase. Конфликт → инструкция
в task_comments + non-zero exit (воркер должен `kanban_block`).

Контракт (ADR-0115 §3.3 расширение, issue #2438):

- Аргументы: `<task_id> <branch> [ISSUE_NUM]`. `task_id` должен матчить
  `^t_[a-f0-9]{6,}$`, иначе usage error.
- Если BEHIND ≤ `MAX_BRANCH_BEHIND` — no-op (exit 0).
- Если BEHIND > `MAX_BRANCH_BEHIND` → auto-rebase origin/develop.
- `SKIP_PRE_FLIGHT=true` → exit 0 без действий (opt-out для emergency).
- Не вызывать вне git worktree → exit 2.

#### ENV-тюнинг

| Var | Default | Назначение |
|---|---|---|
| `MAX_BRANCH_BEHIND` | `30` | порог drift, выше которого warn + auto-rebase. PR #2351 — 66 коммитов behind, PR #2363 — add/add конфликт → были выше этого. |
| `GITHUB_REPO` | `krikz/rob_box_project` | owner/repo для `gh issue comment` (task_comments warn). |
| `GH_CONFIG_DIR` | `/home/builder/.config/gh` | путь к gh auth (для `gh` CLI из cron-окружения). |
| `KANBAN_BOARD` | `robbox` | board-name для kanban-tools. |
| `SKIP_PRE_FLIGHT` | `false` | `true` → exit 0 без fetch/rebase. Emergency opt-out. |

Exit codes: `0` (success / fresh / auto-rebase OK), `1` (rebase conflict — worktree в rebase-merge, нужен manual resolve), `2` (usage error).

SOT: `<repo>/scripts/agent_flow/worker_pre_flight.sh`. Раскладывается
`install.sh` в `~/.hermes/profiles/devops/scripts/` и `~/.hermes/scripts/`.
Дрейф на хосте ловит `agent-flow-drift-detect.sh` (файл в `EXPECTED`).

Тест: `bash scripts/agent_flow/tests/test_worker_pre_flight.sh`.

### `worker_post_flight.sh` — rebase post-check (issue #2438, 2026-09-14)

Воркер-helper: вызывается в КОНЦЕ сессии (после `git push`, ПЕРЕД
`kanban_complete`). Делает `git fetch origin develop --prune`, считает BEHIND,
и если BEHIND > 0 — auto-rebase. Конфликт → инструкция в `task_comments`
+ exit 1 (воркер НЕ должен вызывать `kanban_complete` — должен `kanban_block`).

Парный к `worker_pre_flight.sh` (issue #2438: «rebase-protocol неполный»,
нет post-work rebase → PR diverged, merge-gate ловит add/add конфликты, ретро
PR #2363).

Контракт (ADR-0115 §3.3 расширение, issue #2438):

- Аргументы: `<task_id> <branch> [ISSUE_NUM]`. Те же валидации, что в pre.
- BEHIND == 0 → exit 0 (no-op).
- BEHIND > 0 → auto-rebase. Успех → exit 0. Конфликт → инструкция + exit 1.
- `SKIP_POST_FLIGHT=true` → exit 0 без действий.
- Не вызывать вне git worktree → exit 2.

Из скрипта **автоматически вызывается `worker_scope_check.sh`** (см. ниже) —
pre-push scope self-check. Opt-out: `SKIP_SCOPE_CHECK=true` (см. комментарий в
`install.sh:153`).

#### ENV-тюнинг

| Var | Default | Назначение |
|---|---|---|
| `MAX_BRANCH_BEHIND` | `30` | порог warn (для post-flight это «info»: rebase делается при любом BEHIND > 0, но warn логируется только если > этого порога). Совместимо с pre-flight. |
| `GITHUB_REPO` | `krikz/rob_box_project` | owner/repo для `gh issue comment`. |
| `GH_CONFIG_DIR` | `/home/builder/.config/gh` | gh auth path. |
| `KANBAN_BOARD` | `robbox` | board-name для kanban-tools. |
| `SKIP_POST_FLIGHT` | `false` | `true` → exit 0 без fetch/rebase. |
| `SKIP_SCOPE_CHECK` | `false` | `true` → пропустить auto-вызов `worker_scope_check.sh`. Legitimate для смежных fix'ов, которые формально вне scope карточки. Упомянут в комментарии `install.sh:153`, но не задокументирован в ENV-таблицах — фиксируем здесь. |

Exit codes: `0` (up-to-date / auto-rebase OK), `1` (rebase conflict), `2` (usage error).

SOT: `<repo>/scripts/agent_flow/worker_post_flight.sh`. Раскладывается
`install.sh` (EXPECTED). Тест: `bash scripts/agent_flow/tests/test_worker_post_flight.sh`.

### `worker_scope_check.sh` — pre-push scope self-check (issue #2438, PR #2443, 2026-09-14)

Воркер-helper: самопроверка файлов ПЕРЕД push и `kanban_complete`. Ловит
«левые» файлы, которые воркер подхватил с чужого worktree или закоммитил
по ошибке, и блокирует карточку до их разбора.

Третий рубеж после freshness (`validate_branch_freshness.sh`) и post-PR
scope (`validate_pr_scope.sh`): он проверяет ДО push/`kanban_complete`, пока
карточку ещё можно починить дёшево.

Контекст (PR #2443): PR «rebase-protocol» ушёл с 4 чужими файлами —
`docker/vision/vision-hailo/*` (hailo, ADR-0089) и
`src/rob_box_quest/webxr_client/tests/voice_capture_*.test.ts` (Quest,
ADR-0027). Воркер писал поверх ветки с чужими коммитами и не смотрел
`git status` перед push.

Что проверяет:

- **working tree**: staged + unstaged + untracked (`git status --porcelain`).
- **committed diff**: `git diff --name-only BASE_REF...HEAD` (то, что пойдёт в PR).
- Файлы из обоих наборов сверяются с `PR_ALLOWED_PREFIXES` / `PR_ALLOWED_GLOBS`
  (контракт совместим с `validate_pr_scope.sh`).

Режимы:

- Без `PR_ALLOWED_PREFIXES`/`GLOBS` → **INFO-режим**: печатает список файлов
  (воркер ВИДИТ что у него в working tree), exit 0 — но exit 1 если файлов
  больше `MAX_OUT_OF_SCOPE` (default 10, defensive — почти наверняка drift).
- С `prefixes`/`globs` → **блокирующий**: exit 1 если есть out-of-scope файл,
  список нарушителей в stderr.

`SKIP_SCOPE_CHECK=true` → exit 0 без проверки (opt-out для legitimate
смежного fix'а).

#### ENV-тюнинг

| Var | Default | Назначение |
|---|---|---|
| `PR_ALLOWED_PREFIXES` | `` (пусто) | comma-separated allowed path prefixes. Те же правила, что в `validate_pr_scope.sh`: trim, empty entries skip, case-sensitive match `path.startswith(prefix)`. |
| `PR_ALLOWED_GLOBS` | `` (пусто) | comma-separated fnmatch-style globs. Дополняют prefix'ы, обрабатываются последними. |
| `PR_SCOPE_MODE` | `` (off) | `pre-merge` → двухточечный `git diff origin/develop` (working tree vs origin/develop). Ловит pollution ДО commit. Default OFF (как в `validate_pr_scope.sh`). |
| `SKIP_PR_SCOPE` | `false` | `true` → exit 0 без проверки (legacy alias, оставлен для совместимости с `validate_pr_scope.sh`). |
| `SKIP_SCOPE_CHECK` | `false` | `true` → exit 0 без проверки (canonical opt-out, используется воркерами через `worker_post_flight.sh`). |
| `BASE_REF` | `origin/develop` | эталон для committed diff. |
| `MAX_OUT_OF_SCOPE` | `10` | defensive INFO-mode cap: даже без prefixes при > N файлов exit 1. |
| `GITHUB_REPO` | `krikz/rob_box_project` | owner/repo для `gh issue comment` (warn). |
| `GH_CONFIG_DIR` | `/home/builder/.config/gh` | gh auth path. |
| `KANBAN_BOARD` | `robbox` | board-name для kanban-tools. |

Exit codes: `0` (OK / skip / INFO в пределах cap), `1` (out-of-scope файлы — блокирующий fail), `2` (usage error).

SOT: `<repo>/scripts/agent_flow/worker_scope_check.sh`. Раскладывается
`install.sh` (EXPECTED). Тест: `bash scripts/agent_flow/tests/test_worker_scope_check.sh`.

### `agent-flow-stale-blocked-watchdog.sh` — cron watch для stale-blocked карточек (ретро t_55c6c882, 2026-09-14)

Auto-detect blocked kanban-карточки, все prerequisites которых уже merged
в develop, и alert'ить Шифу (без авто-unblock).

Паттерн (`stale-blocked-after-prerequisites-merged`): карточка в `blocked`
с body, ссылающимся на `PR #NNNN`, но все эти PR уже merged. Карточка висит,
потому что:

1. `block_fn` не имеет trigger на merged-PR (ручной unblock через Шифу).
2. `blocked-watchdog-scope` не покрывает prereq-merge (он для mis-scope архитектурных).
3. e2e-/merge-процессы НЕ имеют callback на блокирующие карточки.

Контракт (per tick):

1. `flock` lock.
2. Iterate по всем kanban-доскам (если `KANBAN_DB_PATH` задан — только эта БД, test mode).
3. SELECT blocked-tasks WHERE body LIKE `%PR #%` OR `%#NNNN%`.
4. Для каждого кандидата: extract PR numbers из body + из последнего blocked-event payload → `gh pr view` на каждый → SKIP если хоть один open или closed-not-merged.
5. SKIP если есть parent со status ≠ done.
6. Иначе — match; idempotency: 1 row в task_comments за сегодня с `MARKER_TAG` → SKIP.
7. Emit **ОДИН** alert-comment через `hermes kanban --board <board> comment <tid> <body>`.

**НЕ auto-unblock'ит.** Шифу eyeball'ит, делает unblock вручную или запускает
ручной `workflow_dispatch` / cron. Это намеренный trade-off: auto-unblock
может пропустить карточку, которую Шифу хочет подержать blocked дольше
(например, ожидает ручной QA). Manual step гарантирует eyeball.

Stats: scanned, matched, skipped_idempotent, skipped_unmerged_pr, skipped_unfinished_parent, skipped_no_pr_ref, emitted, errors → stderr (для cron delivery).

#### ENV-тюнинг

| Var | Default | Назначение |
|---|---|---|
| `GH_REPO` | `krikz/rob_box_project` | owner/repo для `gh pr view`. |
| `GH_CONFIG_DIR` | `~/.config/gh` | gh CLI auth path. |
| `KANBAN_DB_PATH` | `` (пусто) | single DB override (test mode — сканирует только эту БД). |
| `KANBAN_BOARD` | `` (пусто) | board-name для `hermes kanban comment`. |
| `KANBAN_BOARDS_DIR` | `/home/builder/.hermes/kanban/boards` | production scan root. |
| `DRY_RUN` | `false` | `true` → log only, no comment emit. Полезно для проверки tick'а перед прод. |
| `AGE_THRESHOLD_SECONDS` | `3600` (1h) | карточка должна провисеть в blocked хотя бы час, чтобы не alert'ить свежезаблокированные. |
| `MARKER_TAG` | `⚠️ stale-blocked: prerequisites merged` | marker для idempotency check (substring в тексте comment'а за сегодня → SKIP). |
| `LOCK_FILE` | `/tmp/agent-flow-stale-blocked-watchdog.lock` | flock guard. |
| `LOG_FILE` | `/tmp/agent-flow-stale-blocked-watchdog.log` | stats log (append). |
| `HERMES_CLI` | `hermes` | путь к hermes binary для `hermes kanban comment`. |

Exit code: `0` (тик чистый / DRY-RUN), `2` (был emit хотя бы одного alert — намеренный non-zero, чтобы cron delivery заметил).

SOT: `<repo>/scripts/agent_flow/agent-flow-stale-blocked-watchdog.sh`. Раскладывается
`install.sh` (EXPECTED). Регистрация cron-job — отдельный шаг
(`ensure-stale-blocked-watchdog-cron` в `install.sh`, см. ~стр. 1002-1030) —
не auto-register, Шифу явно вызывает после approve.

### `dryrun_fail_streak_issue.sh` — local harness для fail-streak auto-issue (PR #2418, 2026-09-14)

Детерминированный dry-run harness для auto-create-issue ветки в
`agent-flow-e2e-fail-streak-watchdog.sh`. **НЕ модификация watchdog**,
а зеркало его auto-create-issue ветки, которое можно гонять локально
и в CI без gh-токена и без реальной сети.

Контекст (PR #2418, merge commit a9b04981): watchdog добавил авто-создание
issue при fail-streak ≥ `E2E_FAIL_STREAK_ISSUE_THRESHOLD` (5), rate-limited
через mtime `ISSUE_COOLDOWN_FILE` (default 4ч). Acceptance критерий из
карточки t_7572e7a8: «running the harness locally shows exactly one issue
payload per >4h window, and zero on immediate re-runs».

Что делает:

- Поднимает sandbox `HERMES_HOME`, кладёт `ISSUE_COOLDOWN_FILE` в `$HERMES_HOME/state/`.
- Берёт 8 fixed fail-runs (run IDs и HEAD SHAs из body карточки) как фикстуру
  и собирает точно такой же issue-body, как watchdog (строки 296-352 a9b04981).
- Гоняет 3 сценария с фейковым mtime cooldown-файла:
  1. **cold start** (файл отсутствует) → 1 payload
  2. **immediate re-run** (mtime = now) → SKIP
  3. **advance mtime на >4h** (`touch -d "5 hours ago"`) → 1 payload
- Каждый payload печатает РОВНО ту `gh issue create` команду с полным телом,
  которую watchdog бы отправил, но НЕ делает реальных вызовов.

Решение «skip vs create» зеркалирует watchdog строки 286-292 (mtime check)
и 295-298 (gh-truth check, опущен — см. NOTE в коде). Тело issue — копия
шаблона watchdog строки 311-352, с тем же набором секций и тем же
hypothesis-блоком. Единственное намеренное отличие: вместо `gh issue
create` печатает команду и тело в stdout.

#### ENV-тюнинг (mirror watchdog)

| Var | Default | Назначение |
|---|---|---|
| `GH_REPO` | `krikz/rob_box_project` | owner/repo для `gh issue create` payload. |
| `E2E_WORKFLOW` | `L-E2E Voice Test.yml` | имя workflow для fail-runs. |
| `E2E_FAIL_STREAK_ISSUE_THRESHOLD` | `5` | порог (mirror watchdog). |
| `E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS` | `4` | rate-limit (mirror watchdog). |
| `E2E_FAIL_STREAK_ISSUE_LABEL` | `e2e-fail-streak` | лейбл нового issue. |
| `E2E_FAIL_STREAK_ISSUE_ASSIGNEE` | `` (пусто) | assignee (опц.). |
| `HERMES_HOME` | `~/.hermes` | sandbox root для `state/` (в тестах указывают на временную БД). |
| `ISSUE_COOLDOWN_FILE` | `${HERMES_HOME}/state/agent-flow-e2e-fail-streak-last-issue` | mtime-target для cooldown guard (mirror watchdog). |

Exit codes: `0` (все 3 сценария — ожидаемые 1/0/1 payload), `1` (нарушен инвариант; см. сводку в конце вывода).

SOT: `<repo>/scripts/agent_flow/dryrun_fail_streak_issue.sh`. Раскладывается
`install.sh` (EXPECTED). Тест: `bash scripts/agent_flow/tests/test_dryrun_fail_streak_issue.sh`.

### `agent-flow-rotation-watchdog.sh` — жива ли e2e-ротация

Только читает, в GitHub не пишет вообще. Нет ни одного cron-тика и ни
одного коммита в `origin/develop` за `ROTATION_DEAD_MIN` (2 ч) → `exit 1`.
«N упавших e2e подряд» здесь НЕ считается — этим занимается
`agent-flow-e2e-fail-streak-watchdog.sh` (дубль снят 30.08).

> Оба вотчдога до 30.08 лежали в репо, но отсутствовали в `EXPECTED`
> внутри `install.sh` — то есть на хост не раскладывались и запускаться
> физически не могли. Сейчас раскладываются и покрыты drift-детектором;
> **cron-job для них не зарегистрирован** — это отдельное решение Шифу.

## Общие библиотеки

Четыре файла рядом со скриптами; раскладываются install.sh наравне с ними
(они есть в `EXPECTED`, значит их сверяет и `agent-flow-drift-detect.sh`):

| файл | что внутри | кто сорсит |
|---|---|---|
| `lib_agent_flow_common.sh` | `af_load_profile_env`, `af_flock_guard_or_exit`, `af_maintenance_gate_or_exit`, `gh_list_issues_by_label`, `has_label` / `has_label_json`, `slugify`, `detect_pr_kind`, `free_stale_worktrees_for` | triage, merge-gate, e2e-process, deploy-sweep, unlabeled-sweep, handoff |
| `lib_user_unlabel_check.sh` | «user-unlabel respect» guard (ретро 18.08 t_de6bea69) | merge-gate, e2e-process |
| `lib_workflow_dedup.sh` | дедуп запусков workflow между двумя кронами | e2e-process, post-merge-build |
| `hermes_github.sh` | `whoami_*`-обёртки над мутациями issue/PR (кто именно правил метку) | triage, merge-gate, e2e-process, completion-check |

`lib_agent_flow_common.sh` появился 30.08 при дедупе процессного слоя: до
него `gh_list_issues_by_label` лежала в четырёх копиях, `.env`-преамбула и
flock-преамбула — в пяти, MAINTENANCE-гейт — в четырёх. Копии успели
разъехаться — дефолтами полей и текстами логов, — и баг чинился в одной из
них (REST отдаёт `updated_at`, а маппинг читал `updatedAt`: на fallback-пути
поле терялось, deploy-sweep падал по KeyError внутри подстановки, то есть
молча). Гард на это — `tests/test_gh_label_filter_fallback.sh`, кейс G.

Два намеренных исключения, не сводить:
- `log()` у каждого скрипта свой (свой `LOG_PREFIX`, у handoff ещё и вывод в
  stdout). Функции библиотеки зовут `_af_log`, который делегирует в `log`
  вызывающего, если тот определён.
- `has_label` из deploy-sweep принимает labels **JSON** (`gh issue view --json
  labels`), а у остальных на входе **CSV**. Одно имя, два контракта: в
  библиотеке они лежат как `has_label` (CSV) и `has_label_json` (JSON).

Свой flock у `agent-flow-e2e-process.sh` (G6) тоже не сведён: он умеет ждать
замок до 60с, если тик поднят вручную через RUN_NOW.

## Vendor-патчи hermes-agent (ретро t_f00676f8)

Локальные фиксы `hermes-agent` (валидация скиллов по профилю — t_1ab37fa8:
`_profile_skill_names`/`_validate_skills_for_assignee` в `hermes_cli/kanban_db.py`,
symlink-following подсчёт скиллов в `hermes_cli/profiles.py`) накладывались на
хост руками **без сохранения в репо** → при `git pull`/`pip install -U
hermes-agent` патчи терялись, регресс t_1ab37fa8 возвращался (карточки со
скилами не из профиля падали — главная ошибка ретро t_6c6c98fb).

Как устроено теперь:
- Дифф хранится в репо: `vendor/hermes-agent-skill-validation.patch`.
- `install.sh` применяет его идемпотентно (`git apply --reverse --check` →
  уже применён; `git apply --check` → применяет). Вызывать **после** каждого
  обновления hermes-agent.
- Если upstream сдвинулся и патч не ложится — `install.sh` честно падает с
  ошибкой «regenerate vendor patch»; перегенерировать: `git -C
  ~/.hermes/hermes-agent diff hermes_cli/kanban_db.py hermes_cli/profiles.py
  tests/hermes_cli/test_kanban_db.py > vendor/hermes-agent-skill-validation.patch`
  и обновить тесты.

Проверка после обновления:
```bash
bash scripts/agent_flow/tests/test_vendor_patch_apply.sh   # патч ложится на origin/main
python3 scripts/agent_flow/tests/e2e_skill_validation.py devops  # валидация работает
```

### Cost attribution (issue #1462, P10 gap)

`hermes-agent` умеет писать `cost_attribution` event в `task_events`
(`cost_in_cents REAL`), агрегировать через `_compute_run_cost_cents`
по `session_model_usage` сессии воркера, и отдавать breakdown через
`hermes kanban cost --task=<id> --group=task|day|assignee|total`.
CLI/диск сервера: `hermes_agent/hermes_cli/kanban.py::_cmd_cost`,
агрегация: `hermes_cli/kanban_db.py::cost_summary`,
dashboard endpoint: `plugins/kanban/dashboard/plugin_api.py::get_cost`.

Дифф в репо: `vendor/hermes-agent-kanban-cost-attribution.patch`
(4 modified + 1 new test file, ~1360 строк).

Покрытие символа для регрессии: `_compute_run_cost_cents`
(см. `tests/test_vendor_patch_apply.sh::EXPECTED_SYMBOL`).

### Spawn worktree base — origin/develop (issue #1571)

`hermes-agent::_ensure_git_worktree` создавал новые worktree-ветки от
**локального `HEAD`** главного worktree. Если главный worktree давно
не делал `git fetch`/`pull`, его `HEAD` отставал от `origin/develop`
на десятки коммитов — каждый новый воркер стартовал с устаревшего
base (наблюдалось 48-50 коммитов дрифта на нескольких карточках).
Симптом: каждый PR открывался с «фантомным» нет-диффом, CI часто
падал с устаревшими шагами, а ретро приходилось чистить stale worktree.

Два фикса:

1. **`vendor/hermes-agent-spawn-worktree-precheck.patch`** — pre-spawn
   worktree-collision guard. Добавляет `_find_worktree_for_branch`
   (парсер `git worktree list --porcelain`, ищет
   `branch refs/heads/<name>`) и `WorktreeBranchBusyError` — чтобы
   диспетчер не уходил в retry-storm при коллизии. Plain-патч
   (применяется в фазе 1).

2. **`vendor/hermes-agent-z-spawn-base-origin-develop.patch`** —
   перед созданием НОВОЙ ветки делает `git fetch origin develop`
   (best-effort, timeout 30s, fallback на `HEAD` +
   `WORKTREE_BASE_FETCH_FAILED` warning в stderr). Использует
   `origin/develop` как base вместо `HEAD`. Дополнительно эмитит
   `WORKTREE_BASE_DRIFT: HEAD..origin/develop = N commits` warning
   при `N > 10`. Z-prefixed (применяется в фазе 2 поверх precheck).
   Символы-фиксы: `_resolve_worktree_base_ref`,
   `_warn_worktree_base_drift`.

Покрытие для регрессии:
```bash
bash scripts/agent_flow/tests/test_vendor_patch_apply.sh           # патч ложится + идемпотентен
bash scripts/agent_flow/tests/test_spawn_worktree_origin_develop_base.sh  # 5 функциональных сценариев
```

Сценарии в `test_spawn_worktree_origin_develop_base.sh`:
- **uses_origin_develop_when_fresh**: новая ветка указывает на
  `origin/develop`, а не на stale HEAD.
- **warns_on_drift_above_threshold**: при HEAD >10 коммитов за
  `origin/develop` в stderr уходит `WORKTREE_BASE_DRIFT: ... 15 commits`.
- **fetch_fail_falls_back_to_head**: без upstream `fetch` падает →
  фикс деградирует на `HEAD` с `WORKTREE_BASE_FETCH_FAILED` warning.
- **existing_branch_no_fetch**: если ветка уже есть, fetch не делается.
- **warning_format_is_grep_friendly**: формат warning — стабильный
  prefix `WORKTREE_BASE_DRIFT:` / `WORKTREE_BASE_FETCH_FAILED:` для
  парсинга в dispatcher log.

Тест написан на чистом origin/main без сетевых зависимостей (создаёт
bare `origin.git` локально).

## Связь с cron-jobs

Скрипты регистрируются как `cronjob` через `hermes cron run --script
... --schedule "every Nm" --no-agent`. Управлять ими:

```bash
hermes cron list                  # какие кроны запущены
hermes cron pause <job_id>       # временно отключить
hermes cron resume <job_id>      # обратно включить
```

Для патча скрипта — см. **MAINTENANCE-процедуру**: pause → patch →
resume (порядок важен, см. skill `synthesis-tts-chain-debugging` §
«CRITICAL: pause → patch → resume для crons»).

## MAINTENANCE-flag

Общий kill-switch: файл `MAINTENANCE` в `origin/develop`. Если он есть —
тик пропускается (exit 0, это не ошибка). Проверяют его `agent-flow-triage.sh`,
`agent-flow-merge-gate.sh`, `agent-flow-e2e-process.sh`, `agent-flow-handoff.sh`
и (с 30.08) `agent-flow-deploy-sweep.sh` + `agent-flow-unlabeled-sweep.sh` —
общей функцией `af_maintenance_gate_or_exit` из `lib_agent_flow_common.sh`.

> deploy-sweep и unlabeled-sweep до 30.08 гейта НЕ имели, хотя секция в обоих
> так и называлась — «MAINTENANCE gate + env». Первый продолжал ходить по SSH
> на Pi, вешать `hermes` и закрывать issues, второй — вешать `stale-candidate`
> и закрывать issues, пока весь остальной конвейер стоял. Особенно заметно с
> `agents_sleep.sh`, который ставит MAINTENANCE в PEAK-часы со смыслом «все
> спят».

Включается через:

```bash
cd /home/builder/hermes-share/rob_box_project
git checkout develop
touch MAINTENANCE
git commit -m "maintenance: pause agent-flow crons"
git push origin develop
```

Снимается аналогично (`git rm MAINTENANCE`). Подробно — skill
`synthesis-tts-chain-debugging` § «MAINTENANCE-флаг».
