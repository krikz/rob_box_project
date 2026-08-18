# Process-fix Roadmap — 2026-08-18

| Поле | Значение |
|------|----------|
| Документ | `.hermes/plans/process-fix-roadmap.md` |
| Дата | 2026-08-18 |
| Автор | architect (Hermes Agent) |
| Откуда | `docs/reports/process-review-2026-08-18.md` §3 (сломано) + §5 (предложения) |
| Формат | приоритезированный список fix'ов; на каждый — воркер (роль), сложность, ретро-ссылка |

**Приоритеты:**
- **P0** — сделать в текущем релизе (до стабилизации); обычно это то, что блокирует daily flow.
- **P1** — M2 (после стабилизации #1391/#1395/#1398).
- **P2** — рассмотреть в roadmap M2+, если есть band-width.

**T-shirt:** XS (≤2h) / S (≤0.5d) / M (≤2d) / L (≤1w) / XL (>1w).

**Воркеры:**
- **backend** — owner процессных скриптов, ROS2-specific
- **devops** — cron-ы, CI/CD, observability, infra
- **architect** — ADR, дизайн, таблицы состояний, рефакторинг интерфейсов
- **reviewer** — review-vetka, метки, evidence-check

---

## P0 — немедленно (текущий релиз)

### FIX-01: scenario_file required — guard тест (e2e regression)

| Поле | Значение |
|------|----------|
| Источник | review §3.2 |
| Что | Добавить `test_e2e_process_scenario_required.sh` — мок: PR без `.github/e2e/scenarios/*.json` → e2e-process **должен** ставить `e2e:rejected` + комментарий «scenario_file required, none found in PR», **не** fallback на smoke. |
| Почему | #1384 был именно этой дырой — PASS формально, но прогон smoke. |
| Воркер | devops |
| T-shirt | S |
| Сложность | низкая (там уже есть похожие тесты в `test_e2e_process_guard.sh`) |
| Acceptance | 1) новый тест PASS; 2) существующий guard-тест не сломан; 3) e2e-process.sh правит ровно одну проверку (`if [ -z "$SCENARIO_FILE" ]; then …; fi`) |
| Блокирует | none |
| Retro-link | #1384, #1387, ADR-0015 |

### FIX-02: ADR-0018 → Accepted + pre-merge gate

| Поле | Значение |
|------|----------|
| Источник | review §3.6, §5.3 |
| Что | После #1397 merge (AGENTS.md) — формально закрыть ADR-0018 (Currently Proposed → Accepted). Добавить § Acceptance: a) CI gate `validate_honesty.sh` на all PR с `needs-review`; b) daily summary cron (07:00 MSK) — список PR за 24h без `# Evidence:` блока → Telegram Шифу. |
| Почему | Honesty culture — единственный «не сломано» принцип, который мы в этом ревью акцентируем. Без gating'а это «бумага». |
| Воркер | architect (ADR close) + devops (CI gate) + reviewer (label mechanics) |
| T-shirt | S (ADR close) + M (CI gate) |
| Сложность | низкая для ADR, средняя для CI gate |
| Acceptance | 1) ADR-0018 § Acceptance добавлен; 2) `validate_honesty.sh` запускается в `G-Run Tests.yml` на PR с `needs-review`; 3) daily cron создаётся, отрабатывает 1 раз, Telegram приходит |
| Блокирует | FIX-01 (оба про e2e/evidence) |
| Retro-link | #1397, #1402 |

### FIX-03: Dead-code — обновить Phase 3 mini / Phase 4 в AGENT_FLOW_PROPOSAL.md §7

| Поле | Значение |
|------|----------|
| Источник | review §4.2 |
| Что | Заменить `[ ]` на `[x]` для Phase 3 mini (фактически работает). Удалить Phase 4 как «out of scope for MVP». |
| Почему | Документ — SOT; старые чек-боксы вводят воркеров в заблуждение. |
| Воркер | architect |
| T-shirt | XS |
| Сложность | тривиальная |
| Acceptance | diff в §7 только в галочках; commit отдельный |
| Блокирует | none |

### FIX-04: README.md — секция «Agent-driven development»

| Поле | Значение |
|------|----------|
| Источник | review §4.4, §5.8 |
| Что | Добавить 5-7 строк: что есть конвейер, ссылка на AGENT_FLOW_PROPOSAL.md. |
| Почему | Новичок/воркер, открывающий README, не знает о process. |
| Воркер | architect |
| T-shirt | XS |
| Сложность | тривиальная |
| Acceptance | diff только в README.md, секция «Кто пишет код» или «Agent-driven» |
| Блокирует | none |

### FIX-05: SPEC_CURRENT.md → ARCHIVE (snapshot file)

| Поле | Значение |
|------|----------|
| Источник | review §4.3, §5.4 |
| Что | Переместить в `docs/archive/SPEC_CURRENT_2026-07-27.md` (или удалить). Актуальный SOT — `ROADMAP.md` + `AGENTS.md`. |
| Почему | Snapshot-документ 1.5-месячной давности не полезен и вреден. |
| Воркер | architect |
| T-shirt | XS |
| Acceptance | 1) SPEC_CURRENT.md перемещён; 2) README.md и docs/README.md обновлены (link fix); 3) нигде в репо не осталось ссылок на старый путь |
| Блокирует | none |

---

## P1 — M2 (после стабилизации текущего релиза)

### FIX-06: ADR-0019 — single-writer-per-issue (race fix)

| Поле | Значение |
|------|----------|
| Источник | review §3.1, §5.1 |
| Что | Каждая issue имеет уникальный writer (e2e-process или merge-gate) на уровне GitHub API; mutex через `state/issue-writer.json` (per-issue lock). Снимает класс race #1391, #1395. |
| Почему | race между cron-ами — структурная проблема, ADR-0014 закрыл только одну её половину. |
| Воркер | architect (ADR) + devops (impl) |
| T-shirt | M |
| Сложность | средняя (нужно продумать lock TTL и stale-lock recovery) |
| Acceptance | 1) ADR-0019 написан, status Accepted; 2) unit-тест `test_e2e_process_writer_lock.sh`; 3) live-test на 1 PR с `needs-e2e` |
| Блокирует | ADR-0020 |
| Retro-link | #1391, #1395, ADR-0014 |

### FIX-07: scenario_file — добавить unit-тест на параметр workflow

| Поле | Значение |
|------|----------|
| Источник | §3.2 (дополнение к FIX-01) |
| Что | `gh workflow run -f scenario_file=...` — сделать обязательным. Если пусто — exit 1. |
| Почему | второй уровень защиты к FIX-01. |
| Воркер | devops |
| T-shirt | S |
| Acceptance | 1) workflow yml: `scenario_file` required; 2) e2e-process без значения → exit 1; 3) dry-run |
| Блокирует | none |

### FIX-08: cron-observability — health probe

| Поле | Значение |
|------|----------|
| Источник | review §3.8, §5.5 |
| Что | Смонтировать `~/.hermes/profiles/*/cron/output/` на host (sshfs / копия). Реализовать `cron_health.sh` — раз в 30 минут проверять `last_status` всех agent-flow джебов, алерт в Telegram при 2+ подряд красных. |
| Почему | Сейчас «cron упал» = «карточка зависла → Шифу заметил через N часов». |
| Воркер | devops |
| T-shirt | S |
| Сложность | средняя (нужен systemd-mount или эквивалент) |
| Acceptance | 1) `cron_health.sh` запускается, exit 0; 2) тест на 1 упавший cron → Telegram; 3) на 249 настроен cron |
| Блокирует | none |

### FIX-09: ADR-0020 — state machine + reconciler

| Поле | Значение |
|------|----------|
| Источник | review §3.7, §5.2 |
| Что | Таблица `state × trigger → action` для всех 9+ комбинаций. Реализация: `state-reconciler.sh` cron. Заменяет 3 ad-hoc backfill-скрипта (`pr-backfill`, `deploy-reconcile`, `pr-orphan-needs-e2e`). |
| Почему | Каждое новое состояние → новый scan. Это не масштабируется. |
| Воркер | architect (таблица + ADR) + devops (reconciler) |
| T-shirt | L |
| Сложность | высокая (большой refactor, нужно переписать тесты) |
| Acceptance | 1) таблица в ADR-0020; 2) reconciler.sh покрыт 5+ unit-тестами; 3) live-test 24h на staging |
| Блокирует | FIX-06 |
| Retro-link | #1342, #238ff3f7, #5cf0162b |

### FIX-10: ADR-0018 — daily summary cron

| Поле | Значение |
|------|----------|
| Источник | §3.6 (дополнение к FIX-02) |
| Что | Cron в 07:00 MSK: список PR за 24h, у которых `needs-review` без `# Evidence:` блока → Telegram Шифу. Связан с FIX-02. |
| Воркер | devops |
| T-shirt | S |
| Acceptance | 1) cron создаётся; 2) Telegram приходит; 3) интеграция с FIX-02 |
| Блокирует | FIX-02 |

### FIX-11: CI_CD_PIPELINE.md / E2E_TESTING_DESIGN_v2.md — «Last verified» header

| Поле | Значение |
|------|----------|
| Источник | review §4.5 |
| Что | Добавить в начало 2 строки: `Last verified: <date> by <who>`. Если > 30 дней — `[STALE]` watermark. |
| Почему | Документы большие (36K + 51K), без метки актуальности воркеры не знают, что верить. |
| Воркер | architect |
| T-shirt | XS |
| Acceptance | оба файла с обновлённым header |

### FIX-12: user-reopen guard — расширить триггеры

| Поле | Значение |
|------|----------|
| Источник | review §3.3 |
| Что | Расширить user-reopen guard: ловить не только текст «user-reopen» в комментариях, но и любой user-action на issue (label change by non-bot, comment by non-bot, re-open event). Реализация: `gh api` events list → filter `actor.login != '[bot]'` + recent. |
| Почему | guard #1399 слишком узкий, ловит только явный «user-reopen» текст. |
| Воркер | devops (impl) + reviewer (label mechanics) |
| T-shirt | S |
| Acceptance | unit-тест `test_merge_gate_user_touched.sh` (3 сценария: label, comment, re-open) |

---

## P2 — M2+ (по возможности)

### FIX-13: ADR-0021 — e2e isolation (labs docker pool)

| Поле | Значение |
|------|----------|
| Источник | review §3.9, §5.x |
| Что | Сформулировать ADR: «e2e = изолированный Labs Docker pool, не prod-робот». Альтернатива: announce через Telegram «e2e:start / e2e:done» (короткий вариант). |
| Почему | Все попытки gating в dialogue_node провалились (#1386 revert). Решение — не в runtime, а в process. |
| Воркер | architect |
| T-shirt | M |
| Acceptance | ADR-0021 написан; если выбран announce — блок-скрипт готов |

### FIX-14: ADR-0022 — big-bang override CI gate

| Поле | Значение |
|------|----------|
| Источник | review §5.10 |
| Что | Реализовать `big-bang-guard.sh` в `G-Run Tests.yml`: «PR > 50 коммитов ИЛИ > 3000 строк AND `big-bang-override` label отсутствует → exit 1». |
| Почему | Сейчас метка — exception, но автоматической проверки нет. |
| Воркер | devops |
| T-shirt | S |
| Acceptance | 1) guard в workflow; 2) unit-тест на 3 PR (маленький, средний, большой); 3) override работает |

### FIX-15: Split `merge-gate.sh` на 3 модуля

| Поле | Значение |
|------|----------|
| Источник | review §3.4, §5.7 |
| Что | `merge-gate-classify.sh` (dry-run), `merge-gate-execute.sh` (side effects), `merge-gate-cleanup.sh` (archive). |
| Почему | Скрипт 2759 строк перерос порог моно-скрипта. |
| Воркер | architect (дизайн) + devops (impl) |
| T-shirt | L |
| Блокирует | FIX-06 (разделяемая логика) |

### FIX-16: e2e-process — unit-тест на conflicting merges

| Поле | Значение |
|------|----------|
| Источник | review §3.10 |
| Что | `test_e2e_process_conflicting_merges.sh` — мок: 2 PR, попытка merge второго → conflict → второй `e2e:rejected`, первый OK. |
| Воркер | devops |
| T-shirt | S |
| Acceptance | 1) тест PASS; 2) в основном пути e2e есть этот guard |

### FIX-17: PR-side labels reconciliation — единый transitor

| Поле | Значение |
|------|----------|
| Источник | review §5.9 |
| Что | Сейчас 3+ скрипта независимо пишут `needs-review`/`needs-e2e`/`e2e-done`. Свести в ОДИН transitor с явной таблицей. **Часть FIX-09 (ADR-0020).** |
| Воркер | devops |
| T-shirt | M |
| Блокирует | FIX-09 |

---

## Что НЕ берём в этот roadmap (вне scope)

- **Метрики тестов** (`pytest`, coverage) — CI процесс, не dev-process.
- **Auto-merge** — запрет Q5.
- **Multi-model e2e matrix (A42)** — отдельный roadmap, не блокер.
- **Dashboard / Grafana для cron-ов** — пока FIX-08 (sshfs + health probe) достаточно.
- **Тюнинг cron-интервалов** (5m/1h) — нужны замеры, делать после FIX-08.

---

## Cross-cutting fixes

Все fixes из этого roadmap **должны** сопровождаться:
1. WIP-коммит каждые ~15-20 мин (контракт воркера).
2. `kanban_show` comment в соответствующей issue/PR.
3. `kanban_complete` только после зелёного CI.
4. **Honesty check (`validate_honesty.sh`)** — после внедрения FIX-02.

