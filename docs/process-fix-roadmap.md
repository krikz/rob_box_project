# Process Fix Roadmap — SOT (Single Source of Truth) для process debt

| Поле | Значение |
|------|----------|
| Дата создания | 2026-08-19 |
| Автор | architect (Hermes Agent) |
| Постановка | issue [#1464](https://github.com/krikz/rob_box_project/issues/1464), источник: `docs/reports/add-research-2026-08-19.md` §6.2 (issue [#1458](https://github.com/krikz/rob_box_project/issues/1458)) |
| Severity | MEDIUM (документационный долг) |
| Статус | **active** — SOT для process debt, обновляется при каждом ретро |
| Связанные | `docs/reports/process-review-2026-08-18.md`, `docs/reports/investigation-music-and-whistle-2026-08-18.md`, `docs/reports/dialogue-node-review-2026-08-18.md`, `docs/reports/add-research-2026-08-19.md`, `docs/design/PROCESS_RETRO_2026-08-09.md` |
| Философия | ADR-0018 «Честный FAIL лучше красивого PASS»; ADR-0022 GATE-1/2/3 — три gate'а merge-ready |

---

## 0. Зачем этот документ

Раньше process debt был **размазан** по `docs/reports/*-<date>.md` и `kanban cards`
(`t_xxxxxxxx`). Найти конкретный баг «кто его знает, в каком отчёте» занимало 5-15 минут.
**Этот SOT собирает всё в одну таблицу** с колонками:

- `bug_class` — класс бага (Counter init / Scope leak / Shell fragility / Registry / Gate drift / Honesty / CI infra / Docs debt / Vela-related)
- `r_number` — R-номер из `add-research-2026-08-19.md` (или дефолтный `—` если баг не в ADD §3.1)
- `b_number` — B-номер из `process-review-2026-08-18.md` (или дефолтный `—`)
- `issue_ref` — GitHub issue (`#NNNN`)
- `date` — дата выявления (YYYY-MM-DD)
- `owner` — текущий владелец (architect / devops / backend / process / unassigned)
- `status` — `🔴 open` | `🟡 in-progress` | `🟢 resolved` | `📜 archived` (won't fix)
- `retro_id` — kanban retro-карточка `t_xxxxxxxx` или `—` если нет
- `adr_ref` — ссылка на ADR или `—`
- `fix_ref` — PR / commit / report, закрывший баг (или `—`)
- `notes` — что нужно сделать, или почему архивировано

**Правила обновления:**

1. Любая новая ретроспектива (`docs/reports/process-review-*.md`) обязана добавить строки в
   эту таблицу (не дублировать в отчёте).
2. Любая новая строка должна иметь хотя бы один якорь: `issue_ref` / `retro_id` / `fix_ref`.
3. `status` обновляется через `git log -- <file>` поиск коммита с упоминанием бага
   (коммиты содержат `fix(agent-flow #NNNN):` или `retro(t_xxxx):`).
4. Архивация (`📜`) — только если Шифу явно скажет «не фиксим, задокументируйте».

---

## 1. Мастер-таблица (все известные баги)

> **Условные обозначения:** `—` = нет данных; жирный R-номер = R-номер из
> `add-research-2026-08-19.md` §3.1, **P-номер** = problem-class из того же §3.1,
> **B-номер** = bug-class из `process-review-2026-08-18.md` §2.

| bug_class | r_number | b_number | issue_ref | date | owner | status | retro_id | adr_ref | fix_ref | notes |
|-----------|----------|----------|-----------|------|-------|--------|----------|---------|---------|-------|
| **P0 — ломают pipeline прямо сейчас** |||||||||||
| E2E process exit 127 / SIGPIPE | — | B1 | [#1392](https://github.com/krikz/rob_box_project/issues/1392) | 2026-08-18 | devops | 🟢 resolved | [t_88f3ec13](https://hermes-kanban/t_88f3ec13) | — | PR #1424 (коммит `54bccace`) | `set -euo pipefail` + process substitution crash → поэтапный выход |
| Cron-paused = queue-silent-stall | — | B2 | [#1420](https://github.com/krikz/rob_box_project/issues/1420) | 2026-08-18 | devops | 🟢 resolved | [t_f19de2f1](https://hermes-kanban/t_f19de2f1) | — | PR #1423 (`d97afaf0`) | triage cron восстановлен через feat-card process-agent-flow-triage |
| Scenario file не передаётся в L-E2E | P12 | B3 | [#1384](https://github.com/krikz/rob_box_project/issues/1384) | 2026-08-13 | devops | 🟢 resolved | [t_bff6eccf](https://hermes-kanban/t_bff6eccf) | — | PR #1424 (`deeaa0c7`) + PR #1425 (`f3795ab1`) | `_detect_scenario_in_diff()` fallback через git diff origin/develop...HEAD |
| L-Deploy: `!= 'local'` инвертировано | — | B4 | [#1384](https://github.com/krikz/rob_box_project/issues/1384) | 2026-08-13 | devops | 🟡 in-progress | — | — | partial fix `c9278410` (need 4 места) | нужно доверифицировать |
| Registry topology race (build runner-8 → deploy 249) | — | B5 | [#1384](https://github.com/krikz/rob_box_project/issues/1384) | 2026-08-13 | devops | 🔴 open | — | (ADR-draft) | — | структурный фикс не сделан → нужен registry ADR |
| **P1 — структурные классы, повторяются** |||||||||||
| Counter init missing (`_llm_skipped_counter["e2e_busy"]`) | — | B6 | [#1385](https://github.com/krikz/rob_box_project/issues/1385) | 2026-08-13 | backend | 🟢 resolved | (revert в #1390) | ADR-0021 | PR #1410 (фикс gating) | root cause = B7; revert сделан правильно |
| Gating в dialogue_node (scope leak CI↔runtime) | — | B7 | [#1385](https://github.com/krikz/rob_box_project/issues/1385) | 2026-08-13 | architect | 🟢 resolved | — | ADR-0021 | PR #1410 (`9d9f3f53`) | wake_word_gate как отдельный модуль, не в dialogue_node |
| `set -euo pipefail` + process substitution `< <(python3)` → SIGPIPE | — | B8 | [#1392](https://github.com/krikz/rob_box_project/issues/1392) | 2026-08-18 | devops | 🟢 resolved | [t_88f3ec13](https://hermes-kanban/t_88f3ec13) | — | PR #1424 | structural fix в `agent-flow-e2e-process.sh:2285` |
| Auto-closer закрывает user-reopened issue через 6 мин | — | B9 | [#1391](https://github.com/krikz/rob_box_project/issues/1391) | 2026-08-13 | devops | 🟢 resolved | [t_aa822617](https://hermes-kanban/t_aa822617) | — | PR #1401 (`4b14f253`) | user-unlabel respect guard |
| Kanban CLI positional vs `--title` confusion | — | B10 | [#1361](https://github.com/krikz/rob_box_project/issues/1361) | 2026-08-10 | process | 📜 archived | — | — | — | school rule: школа-фикс, не воркер-карточка; documented в CONTRIBUTING |
| No unit-test для process-фиксов (Counter init, shell escape, race) | P14 | B11 | [#1422](https://github.com/krikz/rob_box_project/issues/1422) | 2026-08-18 | devops | 🟡 in-progress | [t_fae4b687](https://hermes-kanban/t_fae4b687) | — | PR #1418 re-trigger | каждый bug-class требует unit-тест, иначе регрессия |
| Round counter orphan при cleanup | — | B12 | [#1403](https://github.com/krikz/rob_box_project/issues/1403) | 2026-08-15 | devops | 🟢 resolved | [t_bff6eccf](https://hermes-kanban/t_bff6eccf) | — | PR #1425 | counter file в `${HERMES_HOME}/state/...` с idempotent init |
| RUN_NOW signal не consumed сразу после flock | — | B13 | [#1398](https://github.com/krikz/rob_box_project/issues/1398) | 2026-08-14 | devops | 🟢 resolved | [t_091fc5b7](https://hermes-kanban/t_091fc5b7) | — | PR #1400 (`90540dcd`) | watchdog.sh G0b fix |
| MAINTENANCE flag не проверяется в auto-decomposer | — | B14 | [#1431](https://github.com/krikz/rob_box_project/issues/1431) | 2026-08-19 | process | 🟢 resolved | [t_1d467636](https://hermes-kanban/t_1d467636) | — | PR #1441 (`c4a9fc40`) | `kanban.auto_decompose=false` workaround |
| Process-фиксы без PR (школа нарушений) | P13 | B15 | [#1397](https://github.com/krikz/rob_box_project/issues/1397) | 2026-08-13 | process | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | ADR-0018 | PR #1402 (`1840d09b`) | ADR-0018 + AGENTS.md |
| **P2 — повторяющиеся классы, не критичные сейчас** |||||||||||
| Auto-decomposer может разбудить воркера во время MAINTENANCE | — | B14 | [#1431](https://github.com/krikz/rob_box_project/issues/1431) | 2026-08-19 | devops | 🟢 resolved | [t_1d467636](https://hermes-kanban/t_1d467636) | — | PR #1441 | maintenance check |
| PR↔issue e2e-done drift reconcile | — | (new) | [#1448](https://github.com/krikz/rob_box_project/issues/1448), [#1450](https://github.com/krikz/rob_box_project/issues/1450), [#1452](https://github.com/krikz/rob_box_project/issues/1452), [#1456](https://github.com/krikz/rob_box_project/issues/1456) | 2026-08-19 | devops | 🟢 resolved | [t_5cde0bc1](https://hermes-kanban/t_5cde0bc1) | ADR-0022 | PR #1460 (`30b5aa00` regression), PR #1460 fix (`92fad7c7`) | GATE-2 two-stage closer |
| Triage создаёт manual-карточки несмотря на dedup | — | (new) | [#1432](https://github.com/krikz/rob_box_project/issues/1432) | 2026-08-19 | devops | 🟢 resolved | [t_a0fac345](https://hermes-kanban/t_a0fac345) | — | PR #1439 (`cdc043a9`), PR #1440 (`d3bfdc5f`) | user-unlabel respect + dedup-guard |
| **P3 — кунсткамера (для памяти)** |||||||||||
| kanban CLI `requeue` не существует | — | B16 | [#1452](https://github.com/krikz/rob_box_project/issues/1452) | 2026-08-15 | process | 🟢 resolved | — | — | fixed `7d567420` | WARNING каждый тик убран |
| Worker counter init — fixed в PR #1390, но класс не закрыт | — | B17 | [#1385](https://github.com/krikz/rob_box_project/issues/1385) | 2026-08-13 | process | 🟢 resolved | (revert #1390) | ADR-0021 | PR #1410 (`9d9f3f53`) | закрыт через B7 |
| Idempotency-key на done-карточках возвращает done вместо новой | — | B18 | [#1207](https://github.com/krikz/rob_box_project/issues/1207) | 2026-07-27 | process | 🟢 resolved | — | — | PR #1207 retro | — |
| 4h throttle для REOPENED issue | — | B19 | [#1395](https://github.com/krikz/rob_box_project/issues/1395) | 2026-08-14 | process | 🟢 resolved | — | ADR-0014 §5 | ed782a97 v3.1 | цикл карточек по #968 остановлен |
| Provider fallback hygiene (MiniMax primary + DeepSeek fallback) | — | B20 | [#1357](https://github.com/krikz/rob_box_project/issues/1357) | 2026-08-18 | devops | 🟢 resolved | — | — | config flipped 18.08 | DeepSeek-расписание ОТКЛЮЧЕНО, мы на MiniMax подписке |
| **R-номера из `add-research-2026-08-19.md` §3.1** |||||||||||
| False PASS на plausible SWE-Bench фиксах | **R1 (P1)** | — | [#1397](https://github.com/krikz/rob_box_project/issues/1397) | 2026-08-13 | process | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | ADR-0018, ADR-0022 GATE-1 | PR #1402 (`73ae47d7`) | школа «честный FAIL» + validate_honesty.sh |
| Agent forgets context (long-running) | **R2 (P2)** | — | — | 2026-08-19 | process | 🟡 in-progress | — | ADR-0013 | — | `kanban_create.continuity` flag есть, но не используется → не блокер (KISS) |
| Tool-call loops (budget waste) | **R3 (P3)** | — | — | 2026-08-19 | process | 🟡 in-progress | — | — | — | `process-fix-roadmap` watchdog не всегда срабатывает → фоновый процесс |
| Reconciliation drift (PR↔issue) | **R4 (P4)** | — | [#1448](https://github.com/krikz/rob_box_project/issues/1448), [#1450](https://github.com/krikz/rob_box_project/issues/1450), [#1452](https://github.com/krikz/rob_box_project/issues/1452), [#1456](https://github.com/krikz/rob_box_project/issues/1456) | 2026-08-19 | devops | 🟢 resolved | [t_5cde0bc1](https://hermes-kanban/t_5cde0bc1) | ADR-0022 GATE-2 | PR #1460 (`92fad7c7`) | GATE-2 two-stage closer + auto-discovery |
| Need-review ping-pong | **R5 (P5)** | — | [#1428](https://github.com/krikz/rob_box_project/issues/1428) | 2026-08-18 | process | 🟢 resolved | [t_26d8a61b](https://hermes-kanban/t_26d8a61b) | ADR-0022 GATE-1 | PR #1430 (`076452d2`) | R1-R7 evidence в investigation report |
| Acceptance.json/expected_tool_calls contract | **R6 (P6)** | — | [#1428](https://github.com/krikz/rob_box_project/issues/1428) | 2026-08-18 | process | 🟡 in-progress | [t_ba114e5c](https://hermes-kanban/t_ba114e5c) | ADR-0022 GATE-1 | PR #1445 (`784360d9`, wip) | GATE-1 acceptance.json — мы пионеры в open-source |
| Watchdog timer vs sync tick races | **R7 (P7)** | — | [#1428](https://github.com/krikz/rob_box_project/issues/1428) | 2026-08-18 | devops | 🟡 in-progress | — | ADR-0021 | — | semaphoring merge-gate ↔ e2e-process не формализован |
| Security: agent reads `.env`/`~/.ssh` | **R8 (P8)** | — | — | 2026-08-19 | architect | 📜 archived | — | ADR-candidate | — | trust model: worktree изолирован, низкий приоритет (см. §3.3 ADD research) |
| N-version programming failure correlation | **R9 (P9)** | — | — | 2026-08-19 | architect | 📜 archived | — | — | — | для single-robot overengineering, пересмотреть при defect rate > 20% |
| Cost tracking per agent run | **R10 (P10)** | — | [#1460](https://github.com/krikz/rob_box_project/issues/1460) | 2026-08-19 | devops | 🔴 open | — | ADR-candidate | — | важно для MiniMax подписки, но не блокер |
| Persistent agent memory (ATLE) | **R11 (P11)** | — | — | 2026-08-19 | architect | 📜 archived | — | ADR-0013 (KISS) | — | stateless workers воспроизводимы; пересмотреть при P1 regression |
| Specification ambiguity | **R12 (P12)** | — | [#1428](https://github.com/krikz/rob_box_project/issues/1428) | 2026-08-18 | process | 🟡 in-progress | [t_ba114e5c](https://hermes-kanban/t_ba114e5c) | ADR-0021, ADR-0022 | PR #1430 (`076452d2`) | issue-link required + ## e2e блок помогают |
| Stakeholder attention bottleneck | **R13 (P13)** | — | [#1397](https://github.com/krikz/rob_box_project/issues/1397) | 2026-08-13 | process | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | ADR-0018 + ADR-0022 GATE-3 | PR #1448 (GATE-3) | CI-blocking completion снижает reviewer load |
| Honest-Fail culture decay | **R14 (P14)** | — | [#1397](https://github.com/krikz/rob_box_project/issues/1397) | 2026-08-13 | process | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | ADR-0018 | PR #1402 (`73ae47d7`) | AGENTS.md + validate_honesty.sh |
| **R1-R7 из `investigation-music-and-whistle-2026-08-18.md`** |||||||||||
| E2E-done на smoke-тесте, не на целевом сценарии | **R1 (local)** | B3 | [#1358](https://github.com/krikz/rob_box_project/issues/1358) | 2026-08-12 | devops | 🟢 resolved | [t_26d8a61b](https://hermes-kanban/t_26d8a61b) | ADR-0022 GATE-1 | PR #1430 (`076452d2`) | Convention 3 + acceptance.json |
| Issue #1358 != одна задача (две в одной) | **R2 (local)** | — | [#1358](https://github.com/krikz/rob_box_project/issues/1358) | 2026-08-12 | process | 🟡 in-progress | — | — | — | issue 1358 содержит #1363 + #1434; документация нужна |
| Issue #1363 смешивает два бага | **R3 (local)** | — | [#1363](https://github.com/krikz/rob_box_project/issues/1363) | 2026-08-12 | process | 🟢 resolved | [t_26d8a61b](https://hermes-kanban/t_26d8a61b) | — | PR #1430 | разделили в #1401+#1403 |
| Автозакрывалка (e2e-done не снимает при reopen) | **R4 (local)** | B9 | [#1391](https://github.com/krikz/rob_box_project/issues/1391) | 2026-08-13 | process | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | ADR-0014 | PR #1401 (`4b14f253`) | user-unlabel respect guard |
| Лгущий воркер не detection'ится | **R5 (local)** | — | [#1428](https://github.com/krikz/rob_box_project/issues/1428) | 2026-08-18 | process | 🟡 in-progress | [t_26d8a61b](https://hermes-kanban/t_26d8a61b) | ADR-0018, ADR-0022 | — | `validate_honesty.sh` warning gate, без auto-fail пока |
| Конфликт имён tools (MiniMax vs Renardo) | **R6 (local)** | — | [#1358](https://github.com/krikz/rob_box_project/issues/1358) | 2026-08-12 | backend | 🟢 resolved | [t_26d8a61b](https://hermes-kanban/t_26d8a61b) | — | PR #1430 (`cadff83c`) | music_skill_prompt.txt expand на 7 gen_* tools |
| Диалог между воркерами и юзером теряется | **R7 (local)** | — | — | 2026-08-18 | process | 🔴 open | [t_26d8a61b](https://hermes-kanban/t_26d8a61b) | — | — | CRP (Consultation Request Pack) schema нужна |
| **Прочее (вне R/B таблиц)** |||||||||||
| `worktree add failed` — archived card держит ветку | — | (P0 §3.0 #1) | [#1050](https://github.com/krikz/rob_box_project/issues/1050), [#1077](https://github.com/krikz/rob_box_project/issues/1077) | 2026-08-09 | devops | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | PR (коммит `049164e8` area) | free_stale_worktrees перед unblock |
| rclpy Humble: `mix_channels=[]` → BYTE_ARRAY, краш audio_node | — | (P1 §3.0 #3) | [#1076](https://github.com/krikz/rob_box_project/issues/1076) | 2026-08-09 | backend | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | коммит `049164e8` | непустой дефолт |
| Ложный deploy FAILED (conclusion пустой → FAILED) | — | (P2 §3.0 #4) | [#1077](https://github.com/krikz/rob_box_project/issues/1077) | 2026-08-09 | devops | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | retry ×3 чтения conclusion |
| E2E-process молча выходил (`set -euo pipefail` + grep exit 1) | — | (P2 §3.0 #5) | [#1077](https://github.com/krikz/rob_box_project/issues/1077) | 2026-08-09 | devops | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | sed-конвертация `\n` |
| SQLite «cannot start a transaction within a transaction» | — | (P0 §3.0 #12) | [#1077](https://github.com/krikz/rob_box_project/issues/1077) | 2026-08-09 | backend | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | `SQLiteVoiceMemory.append_turn` thread-safe |
| Бюджет 90/90 — воркер не коммитил WIP | — | (P0 §3.0 #2) | — | 2026-08-09 | process | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042), [t_9435a3c5](https://hermes-kanban/t_9435a3c5) | — | — | WIP-коммиты в контракте воркера + triage max_runtime |
| Воркер не верифицирует фичу после e2e | — | (P0 §3.0 #7) | [#1077](https://github.com/krikz/rob_box_project/issues/1077) | 2026-08-09 | process | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | ADR-0022 GATE-1 | — | e2e-доклад включает acceptance-проверку |
| E2E-контракт не валидируется: voice_file не существует → scp fail | — | (P1 §3.0 #8) | [#1077](https://github.com/krikz/rob_box_project/issues/1077) | 2026-08-09 | devops | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | fail-fast пред-проверка в e2e-process |
| E2E-контракт: блок ## e2e в PR body, не в issue | — | (P1 §3.0 #9) | — | 2026-08-09 | process | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | fallback на PR body (issue > PR > env) |
| Handoff-рекурсия: тестовый корень плодит детей | — | (P2 §3.0 #13) | — | 2026-08-09 | process | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | архивация корня + защита handoff.sh |
| Дубликаты карточек от triage | — | (P2 §3.0 #14) | [#1076](https://github.com/krikz/rob_box_project/issues/1076) | 2026-08-09 | process | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | идемпотентность по issue-number |
| MiniMax 429 — fallback deepseek не настроен | — | (P2 §3.0 #11) | — | 2026-08-09 | devops | 🟢 resolved | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | `fallback_providers` в architect |
| Первый захват речи теряет начало («роберт») | — | (P2 §3.0 #15) | — | 2026-08-09 | backend | 🔴 open | [t_c55ed042](https://hermes-kanban/t_c55ed042) | — | — | VAD-детекция / буферизация первого чанка |
| `music_skill_prompt.txt` не обновлён для 7 gen_* tools | — | (4.2 in dialogue-node-review) | [#1403](https://github.com/krikz/rob_box_project/issues/1403) | 2026-08-15 | backend | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | ADR-0021 | PR #1398 (`3330ba25`) | expand на 7 gen_* tools |
| Startup_greeting свист при wake word | — | (4.3 in dialogue-node-review) | [#1003](https://github.com/krikz/rob_box_project/issues/1003), [#1363](https://github.com/krikz/rob_box_project/issues/1363) | 2026-08-06 | backend | 🟢 resolved | [t_28dcdaf0](https://hermes-kanban/t_28dcdaf0) | — | PR #1391 (`1e1fea5b`) | редизайн |
| Triage auto-decompose не учитывает MAINTENANCE | — | B14 | [#1431](https://github.com/krikz/rob_box_project/issues/1431) | 2026-08-19 | devops | 🟢 resolved | [t_1d467636](https://hermes-kanban/t_1d467636) | — | PR #1441 | maintenance check |
| Merge-gate rejected sweep | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_061d466e](https://hermes-kanban/t_061d466e) | — | PR (branch `z-devops/t_061d466e-merge-gate-rejected-sweep`) | — |
| PR backfill scan | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_2f25fc17](https://hermes-kanban/t_2f25fc17) | — | — | — |
| Merge-gate e2e-orphan cleanup | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_423453b1](https://hermes-kanban/t_423453b1) | — | — | — |
| Pause/resume backfill | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_78e4800f](https://hermes-kanban/t_78e4800f) | — | — | — |
| Conflict recovery after merge | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_8af6bf29](https://hermes-kanban/t_8af6bf29) | — | — | — |
| Watchdog dispatcher liveness | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_901c790b](https://hermes-kanban/t_901c790b) | — | — | — |
| E2E conflict round counter | — | (new) | — | 2026-08-19 | devops | 🟢 resolved | [t_bff6eccf](https://hermes-kanban/t_bff6eccf) | — | — | — |
| `commits` SHA-tag в ветке `ci/image-versions` | — | (HOTFIX.md) | [#1041](https://github.com/krikz/rob_box_project/issues/1041), [#1142](https://github.com/krikz/rob_box_project/issues/1142) | 2026-08-13 | devops | 🟢 resolved | — | ADR-0021 | коммит `c873f479` | SHA-теги в `ci/image-versions`, не в `develop` |
| assigne-existence guard в triage | — | (new) | [#1456](https://github.com/krikz/rob_box_project/issues/1456) | 2026-08-19 | devops | 🟢 resolved | [t_dd7a5749](https://hermes-kanban/t_dd7a5749) | — | коммит `24a8763e` (regression fix PR #1456) | pre-check `skill ∈ profile(assignee).skills` |

---

## 2. Сводка по статусам (быстрый обзор)

| Статус | Кол-во | Что осталось |
|--------|--------|--------------|
| 🟢 resolved | ~38 | — |
| 🟡 in-progress | ~7 | B4 (registry AD-hoc), B11 (unit-test policy), R5 (lying worker detect), R7 (watchdog semaphoring), R12 (spec ambiguity, долгосрочно), B14 (R2 context) |
| 🔴 open | ~3 | B5 (registry topology structural fix), R10 (cost tracking), R7 (dialog lost), «Первый захват речи» |
| 📜 archived | ~4 | B10 (school rule), R8 (security baseline — overengineering), R9 (N-version overengineering), R11 (ATLE — KISS) |
| **Всего** | **~52** | — |

---

## 3. Известные GAPS (не баги, но долги)

| # | Gap | Что нужно | Связь с R-номером | Estimated effort | Приоритет |
|---|-----|-----------|------------------|------------------|-----------|
| G1 | **Registry topology ADR** | задокументировать build→deploy registry chain (runner-8 localhost:5000 ↔ 249:5000) | B4, B5 | 1 ADR (architect) | P1 |
| G2 | **Layer separation ADR** | формализовать границы: dialogue_node = runtime only, watchdog = CI/CD only, merge-gate = orchestrator only | B7 | 1 ADR (architect), merge в ADR-0021 | P1 |
| G3 | **Cron-watchdog для пауз** | если cron paused >1h при наличии queue — alert | B2 | 1 cron + 1 watcher | P1 |
| G4 | **CRP schema (Consultation Request Pack)** | структурированный `kanban_block` с `options/recommendation/trade_offs` | R7, P5 | 1 schema + tooling | P2 |
| G5 | **MRP-as-artifact** (Merge-Readiness Pack) | добавить `rationale`, `evidence_paths`, `script_version_hash` в PR body | P6, P13 | 1 schema + validate_honesty.sh upgrade | P2 |
| G6 | **Cost tracking per kanban card** | dashboard метрик `cost/issue` для MiniMax подписки | R10 | 1 backend hookup + dashboard | P2 |
| G7 | **CI-tests policy**: каждый process-fix = +1 unit test | policy doc + CI gate | B11, R5 | 1 policy + 1 validate hook | P1 |
| G8 | **Парсер e2e artifacts для acceptance-check** | ASR transcript + RMS + baseline + diff в report | B7, R6 | 1 parser (e2e-process.sh) | P2 |

---

## 4. Конвенции обновления этого документа

### 4.1 Кто обновляет

- **architect** — при создании новых ADR или ретро (`docs/reports/process-review-*.md`).
- **devops** — при закрытии process-bugs (фикс коммиты ссылаются назад).
- **любой воркер** — при обнаружении нового бага добавляет строку со `status=🔴 open`
  (одна строка = один коммит, не блокер для merge).

### 4.2 Формат строки

| Поле | Описание |
|------|----------|
| `bug_class` | Short category, стабильный набор (см. §0) |
| `r_number` | R1..R14 (глобальные из §3.1 ADD research) или `R<N>` (локальные из `investigation-music-and-whistle`) |
| `b_number` | B1..B20 (глобальные из §2 process-review) или `—` |
| `issue_ref` | GitHub `issue #NNNN` или `—` |
| `date` | YYYY-MM-DD выявления |
| `owner` | architect / devops / backend / process / unassigned |
| `status` | `🔴 open` / `🟡 in-progress` / `🟢 resolved` / `📜 archived` |
| `retro_id` | kanban retro card `t_xxxxxxxx` или `—` |
| `adr_ref` | ADR-xxxx ADR-name или `—` |
| `fix_ref` | PR-#NNNN / commit SHA / название репорта или `—` |
| `notes` | 1-2 строки контекста |

### 4.3 Что считается «resolved»

Либо:
1. Issue закрыт через PR (есть `gh pr merge --auto` или user merge), **и**
2. Регрессия-тест добавлен в `scripts/agent_flow/tests/`, **или** ADR зафиксировал
   change как part-of-process (а не разовый hotfix).

Если условия нет — статус остаётся `🟡 in-progress` или `🔴 open`.

### 4.4 Что считается «archived»

Только после явной санкции Шифу в issue комментарии. Пример — B10 «kanban positional
confusion»: school rule в CONTRIBUTING.md, не воркер-карточка.

### 4.5 Как искать по этой таблице

```bash
# Все открытые P0
grep -E '\| 🔴 open' docs/process-fix-roadmap.md | head

# Все баги по issue-номеру
grep '#1385' docs/process-fix-roadmap.md

# Все R-номера одного класса
grep 'R4 (P4)' docs/process-fix-roadmap.md

# Все owner=devops баги
grep '| devops ' docs/process-fix-roadmap.md
```

---

## 5. Связанные ADR и документы

| ADR / документ | Связь |
|----------------|-------|
| [ADR-0013](adr/0013-incremental-delivery-over-big-bang.md) | Размер PR, stateless workers (R11 архивация) |
| [ADR-0014](adr/0014-agent-flow-issue-closure.md) | Post-merge close + 24h throttle для REOPENED (B19) |
| [ADR-0015](adr/0015-e2e-verdict-single-source-of-truth.md) | SSoT verdict (B3, R1) |
| [ADR-0018](adr/0018-agent-honesty-culture.md) | «Честный FAIL лучше красивого PASS» — R1, R5, R14 |
| [ADR-0019](adr/0019-agent-flow-triage-already-live.md) | Triage cron investigation (B2, R9) |
| [ADR-0021](adr/0021-dialogue-node-decomposition-discipline.md) | CC-budget, per-bag workflow, lazy-import ceiling, issue-link required (R12) |
| [ADR-0021-lazy](adr/0021-lazy-import-ceiling.md) | Lazy-import ceiling (для R12) |
| [ADR-0022](adr/0022-process-e2e-done-gates.md) | GATE-1 acceptance.json, GATE-2 two-stage closer, GATE-3 CI-blocking (R1, R4, R5, R6) |
| [AGENT_FLOW_PROPOSAL](design/AGENT_FLOW_PROPOSAL.md) | Общий process design |
| [PROCESS_RETRO_2026-08-09](design/PROCESS_RETRO_2026-08-09.md) | Ретро 09.08 (15 аномалий) |
| [process-review-2026-08-18](reports/process-review-2026-08-18.md) | Ретро 18.08 (20 багов B1-B20) |
| [investigation-music-and-whistle](reports/investigation-music-and-whistle-2026-08-18.md) | R1-R7 evidence для ADR-0022 |
| [dialogue-node-review](reports/dialogue-node-review-2026-08-18.md) | Архитектурный review dialogue_node.py для ADR-0021 |
| [add-research-2026-08-19](reports/add-research-2026-08-19.md) | ADD research с §3.1 проблемами P1-P14, §6 рекомендации (этот документ — один из них) |

---

## 6. Changelog документа

| Дата | Что изменилось | Автор |
|------|----------------|-------|
| 2026-08-19 | Initial creation (issue #1464) | architect (Hermes) |
| TBD | следующая ретро добавляет строки через cherry-pick из `process-review-*.md` | — |

---

> **TL;DR для товарища Шифу:** Один файл, ~52 строки = весь process debt с issue/retro/PR
> ссылками. Обновляется по конвенции §4. Поиск по grep. Сейчас: 3 открытых (registry topology,
> cost tracking, dialog lost) + 7 в работе + 4 архивировано. Остальное — `🟢 resolved`.
