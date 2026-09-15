# ADR-0079: nightly-review persistence — JSONL + ISO-week dedup + worker-recorded findings

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-08 (ревизия 08.09: §2.3/§3.1 переписаны после ревью, см. §9) |
| Автор | devops follow-up по наказу товарища Шифу (08.09.2026); ревизия — code review перед мержем #2177 |
| Контекст | Ночной ревью-цикл ADR-0049 жжёт токены на «пустые» карточки и не оставляет артефактов между сессиями: дубль-карточки одного тика (issue #2159), исчезающие «находки» при архивировании, невозможность понять «что вчера проверял ревьюер». |
| Зависимости | ADR-0049 (ночной ревью-цикл), ADR-0018 (культура честности), ADR-0077 (worker-report паттерн, которому здесь следуем) — **реальный ADR-номер этого документа — 0079** (0078 уже занят ретро t_c4cd1f74). |
| Родители | ADR-0046 / ADR-0047 (примеры повторов симптомов 8 и 6 раз — то, что эта архитектура предотвращает) |
| Связанные | `agent-flow-nightly-review.sh`, `kanban-retro-create.sh`, `nightly-review-record.sh`, `kanban-report-write.sh` (ADR-0077), `validate_honesty.sh` |

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

### 2.3 Кто пишет JSONL — и почему не механический скрипт

Первая версия этого ADR (до ревизии 08.09) пыталась решить и «карточка
только при реальных находках», и «JSONL со структурой находок» ОДНИМ
приёмом: `agent-flow-nightly-review.sh` читал бы переменную
`NIGHTLY_REVIEW_OUTCOME` и по ней решал, создавать карточку и что писать
в JSONL.

**Это не работает** — и вот почему: `agent-flow-nightly-review.sh` —
`no_agent` cron-скрипт (ADR-0049 §2.2). Он запускается ДО того, как
кто-либо посмотрел на код, и именно ОН создаёт карточку, которая
диспетчеризирует LLM-ревьюера. Переменная «что нашёл ревьюер» физически
не может быть известна на этом шаге — ревьюер ещё не начал работать. В
проде cron вызывает скрипт без `NIGHTLY_REVIEW_OUTCOME` → он всегда берёт
дефолт → карточка создаётся всегда (как и раньше ADR-0079), а JSONL (в
первой версии писавшийся тем же скриптом) получал бы стаб с пустыми
`findings: []` и захардкоженным `component`. Тесты первой версии были
зелёными только потому, что сами передавали `NIGHTLY_REVIEW_OUTCOME` через
env — то есть проверяли ветку кода, а не то, что переменную кто-то
реально выставляет в бою. Найдено на ревью 08.09, до мержа #2177.

**Исправление — тот же паттерн, что ADR-0077 уже использует для обычных
worker-отчётов**: карточку создаёт механический скрипт (как и раньше,
безусловно — иначе некому будет посмотреть на код вообще), а находки
персистит САМ ревьюер, своим последним шагом, вызовом
`nightly-review-record.sh` перед `kanban_complete` (инструкция — прямо в
теле карточки, см. §3.1).

| Кто | Когда | Что |
|---|---|---|
| `agent-flow-nightly-review.sh` (no_agent) | до ревью | создаёт карточку (всегда), собирает механический дайджест |
| ревьюер (LLM внутри карточки) | после ревью | решает outcome, вызывает `nightly-review-record.sh`, коммитит JSONL |

Это тот же контракт §3.2 ADR-0049 в действии («находок нет → так и
напиши, честный пустой отчёт лучше выдуманного списка») — просто запись
теперь делает тот, кто действительно знает ответ.

## 3. Изменения

### 3.1 `agent-flow-nightly-review.sh`

- Ключ dedup = ISO-неделя: `nightly-review-2026-W37`, не
  `nightly-review-2026-09-08` (issue #2159 — корень бага, см. §1.2).
- Карточка создаётся **безусловно** (как и до этого ADR) — нечего решать
  условно на этом шаге, см. §2.3.
- Тело обеих карточек (nightly + component) получило новую секцию
  «Персистентность» с явной инструкцией ревьюеру: перед `kanban_complete`
  вызвать `nightly-review-record.sh` с реальным outcome/находками,
  закоммитить и запушить `docs/reports/nightly-review/*.jsonl`.
- `_gh_json` имеет явный guard `[ -n "${GH_BIN:-}" ] || return 1` —
  иначе `command -v "$GH_BIN"` под `set -u` падает.
- Убрана мёртвая ветка `NIGHTLY_REVIEW_OUTCOME`/`NIGHTLY_REVIEW_JSONL`
  внутри самого скрипта (см. §2.3) — она никогда не исполнялась иначе,
  чем с дефолтом, в реальном cron-вызове.

### 3.2 `nightly-review-record.sh` (новый)

Вызывается САМИМ ревьюером (не cron'ом) перед `kanban_complete` — тот же
паттерн, что `kanban-report-write.sh` для ADR-0077:

- `--task-id`, `--component`, `--outcome` (`open-issue-<N>` |
  `no-real-defect` | `duplicate-suppressed:<fp>`), повторяемый
  `--finding <JSON>`, `--files-changed`, `--review-date`, `--reports-dir`.
- Считает `fingerprint = sha1(type:file:line:symbol)[:12]` для каждой
  находки, если он не передан явно (калька SARIF `partialFingerprints`).
- Сканирует `<reports-dir>/nightly-review/*.jsonl` за
  `NIGHTLY_REVIEW_LOOKBACK_DAYS` (default 30) на совпадение fingerprint с
  находкой из записи `outcome=open-issue-*` — печатает WARNING в stderr
  (fail-open, не блокирует), чтобы ревьюер не заводил дубль issue.
- Дописывает ОДНУ строку в
  `<reports-dir>/nightly-review/<review-date>.jsonl`.
- НЕ коммитит, НЕ пушит, НЕ трогает kanban/GitHub issues — это работа
  воркера (как и в ADR-0077).

### 3.3 `kanban-retro-create.sh`

- **Слой 4: issue-label guard** (ADR-0079, issue #2159). Для ключей
  `nightly-review-*` / `component-review-*` дополнительно проверяет
  открытые GitHub issues с label `nightly-review` за текущую ISO-неделю.
  Если есть → SKIP, карточка не нужна (дайджест уже ушёл через issue).
- Fail-open: пустой `GH_REPO` / упавший `gh` → пропускаем слой 4,
  полагаемся на 1-3.

### 3.4 `scripts/agent_flow/tests/test_nightly_review_persistence.sh` (переписан)

Раньше тестировал условное создание карточки внутри
`agent-flow-nightly-review.sh` (мёртвая ветка, см. §2.3). Теперь — 9
тестов на `nightly-review-record.sh` напрямую: базовая запись и её поля,
`no-real-defect` не требует `--finding`, `open-issue-*` требует хотя бы
одну находку, fingerprint детерминирован (12 символов), `files-changed`
парсится в массив, append-only на несколько вызовов, dedup-warning на
повторный fingerprint, невалидные `--outcome`/`--finding` → exit ≠ 0.

### 3.5 `scripts/agent_flow/tests/test_kanban_retro_create.sh` (расширен)

+4 теста для слоя 4: L (ignore prior-week issue), M (fail-open без gh),
N (component-review-* триггерит), O (non-prefix не зовёт gh).

### 3.6 `scripts/agent_flow/tests/test_nightly_review.sh` (расширен)

+2 теста: J (ISO-week dedup-key), K (карточка создаётся всегда — замена
прежнего теста на условное создание, который проверял мёртвую ветку).

### 3.7 `scripts/agent_flow/README.md` (обновлён)

+запись про ADR-0079 и `nightly-review-record.sh`.

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
| Находки переживают merge и архив карточки | +1 строка в git на тик, когда ревьюер реально отчитался (TTL 90 дней по ADR — см. §6) |
| Дедуп на уровне ISO-недели вместо DATE | Воскресенье 23:00 — понедельник 01:00 может попасть в разные ISO-недели (понедельник стартует ISO) |
| Layer-4 issue-label guard ловит гонку двух тиков с разными категориями | Ещё один сетевой вызов `gh` на create (fail-open, не фатально) |
| Persistence пишет ревьюер, а не cron — он единственный, кто реально знает outcome | Ничего не гарантирует, что ревьюер запустит `nightly-review-record.sh` — это инструкция в теле карточки, не технический гейт (см. §6) |
| Карточка создаётся каждую ночь безусловно, как в исходном ADR-0049 | Токен-стоимость не снижена этим ADR — снижение через outcome-условное создание оказалось нереализуемым (см. §2.3), это принятая цена ADR-0049 §5, не новый минус |

## 6. Что НЕ покрывает

- **Размер JSONL**: TTL 90 дней (`docs/reports/nightly-review/*.jsonl`
  старше 90 дней — отдельный cron на чистку, ADR-cron).
- **Чувствительность**: находки могут содержать file:line + цитату.
  Сейчас sanitize в JSONL НЕ делается — находки проходят через
  GitHub issues (метка `hermes-finding`), которые видны Шифу. Если
  когда-нибудь туда попадёт env-var / token — фикс через отдельный
  sanitize-шаг в `nightly-review-record.sh` (не реализовано).
- **Compliance ревьюера**: ничто технически не заставляет LLM-ревьюера
  вызвать `nightly-review-record.sh` перед `kanban_complete` — это
  инструкция в теле карточки (как и весь остальной контракт ADR-0049
  §3.2), а не гейт. Если воркер её проигнорирует — карточка закроется, а
  находка так и останется только в комментарии (то есть ровно исходная
  проблема issue #2159, просто пропущенная одним конкретным воркером, а
  не архитектурой). Технический гейт (merge-gate проверяет, что у
  завершённой `nightly-review-*`/`component-review-*` карточки есть
  соответствующая запись в JSONL за тот же `review_date`, аналогично
  `warn_no_worker_report` из ADR-0077) — не реализован в этом ADR,
  кандидат в follow-up, если compliance окажется низкой (см. метрику в
  ADR-0077 — тот же вопрос для worker-отчётов вообще).
- **Общая проблема дублирования работы** (не только nightly-review).
  Layer 4 фиксит дедуп конкретно для `nightly-review-*` /
  `component-review-*` карточек. Она НЕ фиксит более общий случай — два
  воркера, независимо взявшиеся за один и тот же GitHub issue вне этого
  механизма (ровно так родились #2166/#2167, и — уже во время работы над
  этим самым ADR — #2170/#2177). Нужен lock/claim на уровне issue при
  триаже, это отдельная задача.

## 7. Acceptance

| # | Критерий | Кто | Как проверить |
|---|---|---|---|
| 1 | `test_nightly_review_persistence.sh` — 9/9 pass | воркер | raw-вывод теста в PR |
| 2 | `test_nightly_review.sh` — 11/11 pass | воркер | raw-вывод теста в PR |
| 3 | `test_kanban_retro_create.sh` — 15/15 pass | воркер | raw-вывод теста в PR |
| 4 | `nightly-review-record.sh --task-id t_x --component nightly --outcome no-real-defect --review-date YYYY-MM-DD` создаёт `<reports-dir>/nightly-review/YYYY-MM-DD.jsonl` | devops | `ls -la` |
| 5 | `agent-flow-nightly-review.sh` создаёт kanban-карточку на каждом боевом тике в окне — независимо ни от чего (условной ветки больше нет) | devops | `hermes kanban list` |
| 6 | PR в develop с raw-выводом всех 3 тестов | devops | `gh pr view N --json` + комменты |

## 8. Verification log

- 08.09.2026 — `test_nightly_review_persistence.sh`: 9 tests, 9 passed
  (переписан под `nightly-review-record.sh`, см. §9)
- 08.09.2026 — `test_nightly_review.sh`: 11 tests, 11 passed (тест G —
  «деградация без gh» — зелёный при `PYTHONIOENCODING=utf-8`; на прежнем
  прогоне #2177 репортился как flake, воспроизвести на чистом коде layer 4
  не удалось — layer 4 корректно гейтит себя пустым `GH_REPO`, см. §9)
- 08.09.2026 — `test_kanban_retro_create.sh`: 15 tests, 15 passed
- Смоук `nightly-review-record.sh`: happy-path (no-real-defect,
  open-issue-N с находкой), dedup-warning на повторный fingerprint,
  validation errors (bad outcome, missing --finding) — все ожидаемые
  результаты подтверждены вручную.

## 9. Revision log (ревью 08.09, до мержа #2177)

Код-ревью веток `z-{devops}/nightly-review-persistence` (#2170) и
`z-{devops}/nightly-review-persistence-clean` (#2177) перед мержем нашёл,
что §2.3/§3.1 первой версии этого ADR описывали нереализуемый механизм:
`agent-flow-nightly-review.sh` не может знать outcome ревью в момент,
когда он же создаёт карточку для этого ревью (skрипт `no_agent`,
запускается ДО ревьюера). `NIGHTLY_REVIEW_OUTCOME`/`NIGHTLY_REVIEW_JSONL`
внутри этого скрипта были мёртвым кодом — работали только когда тест сам
их выставлял.

Исправление: персистентность вынесена в отдельный скрипт
`nightly-review-record.sh`, вызываемый ревьюером (паттерн ADR-0077).
Карточка снова создаётся безусловно, как в исходном ADR-0049. ISO-week
dedup-ключ и layer-4 issue-label guard в `kanban-retro-create.sh` —
единственные части первой версии, которые были технически корректны с
самого начала — оставлены без изменений.

Заодно, во время работы над этим самым фиксом, конвейер породил ЕЩЁ один
дубль (#2170 и #2177 — идентичный код на один и тот же issue #2159,
разница только в номере ADR и наличии самого файла ADR) — живая
иллюстрация того, что §6 «Общая проблема дублирования работы» описывает
реальный, не гипотетический, риск.