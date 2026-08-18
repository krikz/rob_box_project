# ADR-0021 — dialogue_node.py: правила декомпозиции (CC-budget, SSoT, per-bag workflow)

**Дата:** 2026-08-18
**Статус:** proposed
**Автор:** architect
**Связанные:** ARCH-review `docs/reports/dialogue-node-review-2026-08-18.md` (issue #1405), PR #1386/#1389/#1390/#1395/#1403, ADR-0013 (incremental delivery), ADR-0018 (честный FAIL)

---

## Контекст

`src/rob_box_voice/rob_box_voice/dialogue_node.py` вырос до **4062 LOC / 74 метода** в одном классе `DialogueNode`. За последнюю сессию (18.08) на нём произошло **три боевых регрессии**:

1. **#1389 — crash на первом STT** из-за `_llm_skipped_counter["e2e_busy"] += 1`
   без инициализации в `__init__` (PR #1386, reverted в #1390).
2. **#1403 — LLM не знает про `generate_music`** после merge #1398 —
   `music_skill_prompt.txt` не обновлён, defensive check отсутствует.
3. **#1363 — свист при старте**, hypothesis на race-conditions в
   startup_greeting state machine (флаг + timer, два независимых механизма).

Все три — следствие одного паттерна: **каждое «маленькое расширение» добавляет 5-15 строк в случайное место файла, и через неделю никто не помнит, что они там есть.** Архитектура файла не сдерживает рост — он просто копится.

Метрики baseline @ develop:
- `_run_turn` CC=47, 368 LOC
- `_handle_result` CC=44, 349 LOC
- `_on_stt` CC=28, 161 LOC
- 18 lazy-imports внутри тел методов
- 60+ ссылок на issue-номера в комментариях (файл живёт как changelog)

---

## Решение

Кодифицируем **три правила** для `dialogue_node.py` (и любых новых голосовых
узлов в `rob_box_voice`). Это **не серебряная пуля** — это инварианты,
которые делают невозможным повторение паттерна #1389 на следующем PR.

### R1. CC-budget

Ни один метод в `dialogue_node.py` не должен превышать **CC=15** (исключение —
`__init__`, допустимо CC≤20 из-за условных блоков по фичам).

**При превышении** воркер **обязан** открыть worker-карточку
`feat(refactor voice): выделить X в core/...` **до** (или в том же PR, что
расширяет метод). Проверяется скриптом `scripts/lint/cc_budget.py`.

### R2. State SSoT

Каждый **счётчик / словарь / enum** в `DialogueNode` должен иметь **одно
источник правды**:

- `Enum` или `frozenset` литерал — для множеств значений (пример:
  `_LLM_SKIP_REASONS` из PR #1395).
- `dataclass` — для сложных структур с дефолтами.
- `collections.Counter` (или собственный dict-subclass) — для счётчиков с
  **авто-инициализацией** отсутствующих ключей (вторая линия обороны после SSoT).

**Запрещено**: ручной `dict literal` в `__init__` с N ключами + N
increment-сайтов в разных методах. Это **bug-class** (см. #1389).

### R3. Per-bag workflow

Каждый серьёзный bag (Severity ≥ medium) = **отдельная worker-карточка** с
явным `Closes #N` в PR description. «Большой рефакторинг» допустим только
как серия **маленьких, локальных** PR (ADR-0013).

Для каждого bag:
1. **Issue**: root cause + line-номера + ссылки на issue/PR.
2. **Acceptance**: чек-лист `[ ]` с конкретными шагами.
3. **Tests**: unit + e2e (если затрагивает поведение).
4. **Revert**: revert-ветка `z-{agent}/revert-NNNN-<slug>` готова до merge.

### R4. Lazy-import ceiling

Максимум **2 lazy-imports на файл**, и только для **optional** зависимостей
(`prometheus_client`, `opentelemetry`). Всё остальное — на top of file.

Stdlib (`yaml`, `uuid`, `concurrent.futures`, `traceback`) — **никогда** lazy.
Импорт — это side-effect; если он падает в рантайме первого вызова,
виноват **не воркер, который это написал**, а архитектурный запрет.

### R5. Issue-link в комментарии — обязателен

Каждый `if`/`except`/fix в `dialogue_node.py`, который **фиксит регрессию
или добавляет новое поведение**, должен иметь ссылку `Issue #NNNN` в
docstring или `#` комментарии. **Никакого** `# FIX (live 06.08)` без issue.

Исключение: чисто-косметические правки (переименование, typing).

---

## Альтернативы, которые мы НЕ выбрали

### ❌ Big-bang рефакторинг «DialogueNode → 6 классов с DI»

- **Почему нет**: ADR-0013 запрещает. Каждый такой рефакторинг = серия
  регрессий. Голосовой runtime не прощает нестабильности.
- **Когда бы подошло**: если бы мы переписывали voice-assistant с нуля
  на новой версии ROS (Humble → Jazzy или форк). Тогда — да.

### ❌ Полный переход на `core/dialog_core` (вынос всего в harness)

- **Почему нет**: уже частично сделано (DialogCore, ToolProvider, LLMProvider
  из `rob_box_harness`). Остаток — это ROS2 glue + кросс-доменный retry
  orchestration. Это и есть «DialogueNode», вынос не нужен.
- **Когда бы подошло**: если бы у нас было >1 голосового runtime (например,
  text-only и voice-only варианты). Сейчас — один.

### ❌ Принудительный pytest coverage ≥ 80%

- **Почему нет**: coverage не ловит bug-class (то, что мы забыли
  инициализировать ключ в dict — не вопрос coverage). CC-budget +
  статические проверки (R2) ловят.
- **Когда бы подошло**: для новых фич (musik_guard, stt_gate), у них
  coverage будет естественно высокий.

---

## Последствия

### Положительные

- **Bug-class #1389 закрыт навсегда**: R2 запрещает dict-literal +
  increment-сайты. Любая попытка повторить паттерн → CI fail.
- **CC-budget R1**: `_run_turn` 47 → ≤15, `_handle_result` 44 → ≤15,
  `_on_stt` 28 → ≤10. Это **3-5 маленьких воркер-карточек** (issues 1405-1411).
- **Каждый рефакторинг локальный**: per-bag workflow (R3) означает, что
  один PR не сломает всё.
- **Скорость разработки**: новый воркер читает issues, делает маленький
  PR, юзер мержит. Не надо лезть в god-class.

### Отрицательные / риски

- **Initial slowdown**: 3-5 воркер-карточек за следующие 2-3 цикла
  триажа. Каждая — маленькая, но количество есть.
- **Сопротивление воркеров**: «опять рефакторинг, я просто хотел
  добавить X». ADR-0018 (честный FAIL) — мы **не идём** на компромисс,
  воркер пишет PR с извлечением + extension.
- **CI-скрипт `cc_budget.py`** нужно написать. ~50 LOC Python, +
  1 PR (отдельный). Оценка: 30 мин работы + тест.

### Нейтральные

- ADR-0021 не отменяет существующие ADR-0013/0014/0015/0018. Дополняет.

---

## План внедрения

1. **Этап 1 (немедленно)**: создать issues 1405-1411 (см. ниже), отдать
   воркерам через триаж.
2. **Этап 2 (1-2 цикла)**: воркеры делают PR-ы:
   - #1405 (SSoT counter) — quick fix, 1 PR
   - #1406 (stt_gate extraction) — refactor, 1 PR
   - #1407 (startup_greeting extraction) — refactor, 1 PR
   - #1408 (music_guard extraction) — refactor, 1 PR
   - #1409 (tools-vs-prompt check) — quick fix, 1 PR
   - #1410 (startup_greeting hypotheses) — диагностика, 0 PR (только
     issues если подтвердятся)
   - #1411 (lazy-import audit) — quick fix, 1 PR
3. **Этап 3 (после PR)**: добавить CI-проверку `scripts/lint/cc_budget.py`
   для enforce R1.
4. **Этап 4 (после CC-budget PR)**: ADR-0021 становится **active**
   (а не proposed). Старые нарушения (CC>15 в `_run_turn` и т.д.)
   формально ОК до их рефакторинга (issues 1406-1408), но **новый код**
   должен соответствовать.

---

## Acceptance для ADR (он сам)

- [ ] 7 worker-карточек созданы (issues 1405-1411) через `gh issue create`
      с метками `hermes`, `agent:backend`, `voice`.
- [ ] Товарищ Шифу одобрил ADR (не делаем «silent self-instruction write»,
      см. ADR-0018 + правило «архитектор не правит свои правила молча»).
- [ ] После approve: ADR status → **active**.

---

## Связанные

- `docs/reports/dialogue-node-review-2026-08-18.md` — полное ревью
- `docs/adr/0013-incremental-delivery-over-big-bang.md` — большие рефакторы запрещены
- `docs/adr/0014-agent-flow-issue-closure.md` — процесс issue→card→PR
- `docs/adr/0018-honest-fail-over-fake-pass.md` — culture rule
- Issue #1389 (crash), #1395 (PR draft), #1403 (LLM не знает tool), #1363 (свист)