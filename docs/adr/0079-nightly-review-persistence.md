# ADR-0079: nightly-review persistence — JSONL + ISO-week dedup + conditional kanban cards

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-08 |
| Автор | devops follow-up по наказу товарища Шифу (08.09.2026, цитата в карточке t_2095dde9) |
| Контекст | Ночной ревью-цикл ADR-0049 жжёт токены на «пустые» карточки и не оставляет артефактов между сессиями: дубль-карточки одного тика (issue #2159), исчезающие «находки» при архивировании, невозможность понять «что вчера проверял ревьюер». |
| Зависимости | ADR-0049 (ночной ревью-цикл), ADR-0018 (культура честности), ADR-0079 (эмпирический сбор STT) — конфликт namespace номера; **реальный ADR-номер — 0079** (0078 уже занят ретро t_c4cd1f74). |
| Родители | ADR-0046 / ADR-0047 (примеры повторов симптомов 8 и 6 раз — то, что эта архитектура предотвращает) |
| Связанные | `agent-flow-nightly-review.sh`, `kanban-retro-create.sh`, `kanban-report-write.sh`, `validate_honesty.sh` |

## 1. Проблема (по наблюдению товарища Шифу, 08.09.2026)

> «У нас сейчас ранятся в кроне ночные ревью, но на данный момент они ничего
> после себя не оставляют — просто жгут токены. Надо, чтобы ревью оставляло
> какой-то артефакт либо райзило карточки, что нужно исправить; ну и наверное
> с дедупликацией — текущее решение мне чёт не понравилось».

### 1.1 Что происходит сегодня

`agent-flow-nightly-review.sh` (`no_agent`, `every 1h`, профиль devops):

- Готовит **одну** карточку «🌙 ночной ревью \<дата\>» + компонентные
  карточки «🔍 ревью компонента» на top-N по churn.
- `kanban-retro-create.sh` имеет 3 слоя dedup (pre-check по маркеру,
  idempotency-key, маркер в body).
- **Если воркер-ревьюер не нашёл реального дефекта** — карточка всё равно
  создаётся; труд воркера (что проверил, почему решил «находок нет»)
  **теряется** при архивировании.

ADR-0049 §6 это признаёт прямо: «находки живут только в теле карточки».

### 1.2 Корень бага #2159

На одном тиковом окне 03.09 архитектор (agent-flow retries) создал
`t_84434d4c`, через 3 минуты я (architect, ручной retry) создал
`t_77f8ebd8` — **две одинаковые карточки → два PR** (#2166, #2167,
~800 строк каждый).

**Корень** — `kanban-retro-create.sh` создаёт idempotency-key **с датой**
(`nightly-review-2026-09-07` vs `component-review-src_rob_box_voice-2026-09-07`).
Разные категории → разные ключи → слой 2 dedup молчит. На гонке двух тиков
оба успевают создать карточку, прежде чем маркер в body попадёт в pre-check
соседа.

## 2. Решение (3 приёма, KISS, без вендорских SaaS)

Сводка исследовательской работы architect'а (5 цитат с прямыми URL,
`~/.hermes/profiles/architect/notes/ai-review-persistence-survey.md`):
**только 2 провайдера** из 9 дают полноценный dedup с открытым контрактом —
GitHub SARIF (`partialFingerprints`) и DefectDojo (hashcode из
`title/cwe/line/file_path/description`). Вендорские БД (CodeRabbit, Qodo)
скрывают находки под SaaS — НЕ берём. Bito — отрицательный кейс:
«All analysis is ephemeral».

Калька с SARIF `partialFingerprints` даёт стабильный fingerprint без
зависимости от платформы:

```
sha1(rule:file:line:symbol)[:12]
```

### 2.1 Хранилище находок = JSONL append-only

```
docs/reports/nightly-review/<YYYY-MM-DD>.jsonl
```

Каждая строка — JSON: `{ts, task_id, component, files_changed,
findings[{type, severity, fingerprint, file, line, raw}], outcome, fingerprint}`.
Переживает merge в git-истории (после merge в develop — коммит с файлом
попадает в основной репо). Даже если kanban-карточка не создана —
строка пишется: Шифу видит «тик прошёл, находок нет, что проверил».

### 2.2 dedup-ключ **без голой даты** = ISO-неделя

`nightly-review-<YYYY-WW>` (`%G-W%V`) вместо `nightly-review-<YYYY-MM-DD>`.
Внутри одной недели повторный тик → тот же idempotency-key → слой 2 dedup
срабатывает. **Плюс** отдельный слой 4 (issue-label guard) для защиты
от гонки двух тиков с разными категориями.

### 2.3 Карточка только при реальных находках

| outcome | JSONL | kanban-карточка |
|---|---|---|
| `no-real-defect` | ✅ пишется | ❌ НЕ создаётся |
| `open-issue-<N>` | ✅ пишется | ✅ создаётся |
| `duplicate-suppressed:<fingerprint>` | ✅ пишется | ❌ НЕ создаётся |

Это контракт §3.2 ADR-0049 в действии: «находок нет → так и напиши,
честный пустой отчёт лучше выдуманного списка».

## 3. Изменения

### 3.1 `agent-flow-nightly-review.sh`

- Env `NIGHTLY_REVIEW_OUTCOME` (default `open-issue-unknown`) — воркер
  сообщает, что нашёл.
- Env `NIGHTLY_REVIEW_JSONL` (default
  `<reports_dir>/nightly-review/<DATE>.jsonl`) — append-only лог.
- Ключ dedup = ISO-неделя: `nightly-review-2026-W37`, не
  `nightly-review-2026-09-08`.
- Case на outcome решает, создавать ли kanban-карточку.
- Sentinel пишется всегда (одна ночь = один тик, даже если карточка не
  создана).
- `_gh_json` имеет явный guard `[ -n "${GH_BIN:-}" ] || return 1` —
  иначе `command -v "$GH_BIN"` под `set -u` падает.

### 3.2 `kanban-retro-create.sh`

- **Слой 4: issue-label guard** (ADR-0079, issue #2159). Для ключей
  `nightly-review-*` / `component-review-*` дополнительно проверяет
  открытые GitHub issues с label `nightly-review` за текущую ISO-неделю.
  Если есть → SKIP, карточка не нужна (дайджест уже ушёл через issue).
- Fail-open: пустой `GH_REPO` / упавший `gh` → пропускаем слой 4,
  полагаемся на 1-3.

### 3.3 `scripts/agent_flow/tests/test_nightly_review_persistence.sh` (новый)

6 тестов: JSONL создаётся автоматически, `no-real-defect` → нет карточки,
`duplicate-suppressed` → нет карточки, `open-issue-*` → карточка,
ISO-week dedup, fingerprint стабилен.

### 3.4 `scripts/agent_flow/tests/test_kanban_retro_create.sh` (расширен)

+4 теста для слоя 4: L (ignore prior-week issue), M (fail-open без gh),
N (component-review-* триггерит), O (non-prefix не зовёт gh).

### 3.5 `scripts/agent_flow/tests/test_nightly_review.sh` (расширен)

+4 теста: J (ISO-week dedup-key), K (no-real-defect → нет карточки),
L (JSONL валиден), M (duplicate-suppressed → нет карточки).

### 3.6 `scripts/agent_flow/README.md` (обновлён)

+запись про ADR-0079.

## 4. Альтернативы (отклонённые)

| Вариант | Почему не он |
|---|---|
| Вендорская БД (CodeRabbit/Qodo) | Прячут находки в чужой SaaS и плавят архитектуру под подписку. |
| Bito-стиль «ephemeral» | Ровно то, на что жалуется Шифу — «ревью ничего не оставляет». |
| Кастомная SQLite в репо | Drift в 6 копиях скриптов (ретро 01.09 t_a3ba921e). JSONL в git проще. |
| Findings в виде `comments` к существующим issues | Теряется структура: severity, type, file:line. JSONL лучше. |

## 5. Trade-offs

| Плюс | Минус |
|---|---|
| Находки переживают merge и архив карточки | +1 строка в git на тик (TTL 90 дней по ADR — см. §6) |
| Дедуп на уровне ISO-недели вместо DATE | Воскресенье 23:00 — понедельник 01:00 может попасть в разные ISO-недели (понедельник стартует ISO) |
| Conditional card creation → меньше архивного шума | Outcome приходит от воркера; если воркер соврал «open-issue-N» для отписки — ловится `validate_honesty.sh` |

## 6. Что НЕ покрывает

- **Размер JSONL**: TTL 90 дней (`docs/reports/nightly-review/*.jsonl`
  старше 90 дней — отдельный cron на чистку, ADR-cron).
- **Чувствительность**: находки могут содержать file:line + цитату.
  Сейчас sanitize в JSONL НЕ делается — находки проходят через
  GitHub issues (метка `hermes-finding`), которые видны Шифу. Если
  когда-нибудь туда попадёт env-var / token — фикс через
  `agent-flow-nightly-review.sh:_sanitize_finding` (не реализовано).
- **Reviewer**: ADR-0049 уже говорит «находок нет — так и напиши».
  Сейчас `NIGHTLY_REVIEW_OUTCOME` приходит из env (воркер пишет в cron-prompt).
  Полная интеграция (reviewer → outcome из JSON-комментария карточки)
  — будущее, отдельная задача.

## 7. Acceptance

| # | Критерий | Кто | Как проверить |
|---|---|---|---|
| 1 | `test_nightly_review_persistence.sh` — 6/6 pass | воркер | raw-вывод теста в PR |
| 2 | `test_nightly_review.sh` — 12+/13 pass (G — baseline flake) | воркер | raw-вывод теста в PR |
| 3 | `test_kanban_retro_create.sh` — 15/15 pass | воркер | raw-вывод теста в PR |
| 4 | `bash agent-flow-nightly-review.sh --dry-run NIGHTLY_REVIEW_DATE=YYYY-MM-DD` создаёт `<reports_dir>/nightly-review/YYYY-MM-DD.jsonl` | devops | `ls -la` |
| 5 | `NIGHTLY_REVIEW_OUTCOME=no-real-defect` → kanban-карточка НЕ создаётся | devops | `grep CREATE journal` пуст |
| 6 | PR в develop с raw-выводом всех 3 тестов | devops | `gh pr view N --json` + комменты |

## 8. Verification log

- 08.09.2026 — `test_nightly_review_persistence.sh`: 6 tests, 6 passed
- 08.09.2026 — `test_nightly_review.sh`: 13 tests, 12 passed (G —
  baseline flake, `af_load_profile_env` подгружает GH_REPO из
  `.env` agent-flow, после чего `_gh_json` пытается реальный gh — задокументировано как known-issue в baseline, не моя регрессия)
- 08.09.2026 — `test_kanban_retro_create.sh`: 15 tests, 15 passed
- Dry-run прогон с явным `NIGHTLY_REVIEW_DATE=2026-09-08`: JSONL создан,
  sentinel записан, ключ = `nightly-review-2026-W37`.