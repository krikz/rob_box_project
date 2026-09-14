# t_c44a7cf2 — Архитектурный вердикт: карточка устарела

> Артефакт architect (kanban t_c44a7cf2, issue #2003, base=develop).
> Цель: зафиксировать raw-доказательства того, что реализация `pregenerate`
> в `tts_node` уже существует в коде `develop`, и что в плане оператор-агента
> (`docs/architecture/target-operator-agent-and-dialogue.md`) карточка уже
> помечена как выполненная.

## 1. Статус в источнике истины

`docs/architecture/target-operator-agent-and-dialogue.md`, §13.0 «Статус шагов
на 2026-09-08 (проверено по коду `develop` @ `7ef2f98d`)», строка таблицы:

> **13. Спекулятивная генерация | ✅ сделано | `scheduler/pregen/*` подключён
> в `tts_node`, `pregenerate_enabled=True` (ADR-0056). Issue #2003
> «ЗАБЛОКИРОВАН» устарел.**

Сам план (§13 «Шаг 13. Спекулятивная генерация») формулировал зависимость от
шага 07а как «последним, после того как базовый путь оператора заработает».
Карточка t_c44a7cf2 (эта) была открыта, когда 07а ещё не был сделан.
Сейчас весь базовый путь оператора в плане помечен ✅, и §13.0 явно пишет
«Issue #2003 „ЗАБЛОКИРОВАН“ устарел».

## 2. Проверка реализации в `develop` (raw, файл:строка)

`src/rob_box_voice/rob_box_voice/tts_node.py`:

- 113-120 — реальные импорты пакета:
  ```python
  from .scheduler.pregen import (
      CONFIDENCE_FLOOR as _PREGEN_CONFIDENCE_FLOOR,
      Decision as _PreGenDecision,
      PreGenResult as _PreGenResult,
      PreGenTask as _PreGenTask,
      SpeculativeExecutor as _PreGenExecutor,
      build_pregen_task as _build_pregen_task,
  )
  ```
- 1106-1110 — ROS-параметры (с дефолтом `True`):
  ```python
  self.declare_parameter("pregenerate_enabled", True)
  self.declare_parameter("pregenerate_confidence_floor", _PREGEN_CONFIDENCE_FLOOR)
  self.declare_parameter("pregenerate_history_window", 10)
  ```
- 1678-1686 — чтение параметров в `_init_prefetch_state` (lazy-init engine).
- 2557-2585 — публикатор передаёт `pregenerate` из payload в
  `self.pregenerate(chunk_data, ctx=None)` (подавляется как debug при сбое).
- 3627-3652 — публичные методы `pregenerate` / `claim_pregen` / `cancel_pregen`,
  с защитой `if not self._pregenerate_enabled`.
- 3754 — `def pregenerate(self, current_chunk, ctx=None) -> None` —
  основной вход спекулятивной генерации (синтез следующего чанка до
  `batch_complete` текущего).
- 6750-6757 — `parameter_callback` реагирует на изменение
  `pregenerate_enabled` в runtime.

`src/rob_box_voice/rob_box_voice/scheduler/pregen/` — все 5 модулей
ADR-0056 §2.1 на месте (raw `wc -l`):

```
 82 decision.py
217 estimator.py
101 __init__.py
238 pre_gen.py
192 quality.py
565 speculative_executor.py
```

Карточка ссылается на «tts_node.py 4198 LOC» — текущий `tts_node.py` имеет
**6808 строк** (`wc -l` подтверждает). Цифра 4198 устарела.

## 3. Проверка блокера «зависит от шага 07а»

Шаг 07а — «[operator-agent 07a] Приоритет в `tts_node`», issue #1996.
Карточка t_c44a7cf2 ссылается на него как «dep 7a». По §13.0 этот шаг уже
выполнен (без него §13.0 не мог бы сказать «Шаг 13 ✅» — базовый путь
оператора не работал бы). В коде `tts_node.py` 597-660 (`operator/personality/
normal` whitelist, синхронизирован с `pre_gen.py`) — синтаксис приоритетов
ADR-0056 уже валидируется в `tts_node` без схлопывания `personality → normal`.

## 4. Проверка пункта DoD «`pregenerate` реализован в `tts_node` (grep)»

См. §2 — `grep pregenerate` находит и сам метод, и параметры, и публикатор.
Реализация не «только комментарий в YAML».

## 5. Проверка пункта DoD «качество не деградировало» (raw pytest)

`pytest test/unit/pregen/ -v` в пакете `rob_box_voice`:

```
===================== 66 passed, 8 skipped in 3.15s =====================
```

8 SKIPPED — это `test_pregen_tts_integration.py`, который `pytest.ini` явно
выводит из CI (`requires rclpy/sounddevice/torch; those aren't available in
this test environment`). Все 66 чисто-Python unit-тестов проходят, в т.ч.:

- `test_pregen_decision.py` — таблица истинности `quality × confidence → accept/reject`,
  включая «pass ниже floor → reject» (защита от деградации).
- `test_pregen_estimator.py` — `SegmentEstimate` отвергает out-of-range и NaN,
  важно для корректного confidence-floor.
- `test_pregen_quality.py` — RMS / duration-ratio / silence-ratio гейты
  для `silent`/`clipped`/`trimmed` артефактов (если качество падает — чанк reject).
- `test_speculative_path_latency.py` — параметризованный замер, что
  `speculative_path < baseline` на разных латентностях (это и есть
  измерение уменьшения `latency_chunk_to_chunk` из DoD).

## 6. Проверка пункта DoD «спекулятивный чанк сгенерирован ДО `batch_complete`»

Реализация `pregenerate()` запускается синхронно в публикаторе
(`tts_node.py:2583`), **не дожидаясь** `_on_synthesis_done` предыдущего чанка
(см. §2 — `2575: «``pregenerate`` field if the publisher set one»`).
Архитектурное намерение ADR-0056 зафиксировано и в коде, и в §13.0.
Live-замер `latency_chunk_to_chunk` на реальном железе — это работа
e2e-процесса (после merge), не одной карточки t_c44a7cf2.

## 7. Вердикт

Карточка t_c44a7cf2 устарела: реализация `pregenerate` в `tts_node` уже
присутствует в `develop` (импорты, параметры, методы, модули `scheduler/pregen/*`,
66 юнит-тестов проходят). Сам план оператор-агента (§13.0) и ADR-0056
фиксируют, что Issue #2003 «устарел». Дальнейшие действия:

- **Шифу** (merge-gate) — принять решение о закрытии issue #2003
  или оставить открытой, если он планирует e2e-замер как отдельную
  карточку (это и есть корректный путь: live-latency измеряется ПОСЛЕ merge).
- **При закрытии** — issue #2003 → done с комментарием «§13.0 ✅; см.
  commit `7ef2f98d` в develop, raw pytest в `docs/kanban/t_c44a7cf2-…md`».
- **При продолжении** — переформулировать t_c44a7cf2 в «live e2e замер
  `latency_chunk_to_chunk` с pregenerate ON vs OFF» с явной зависимостью
  от merge в develop.

## 8. Что architect (я) НЕ делаю

- Не пишу фичу (она уже есть). Реализация — это работа `developer`,
  assignee для имплементации, и эта работа сделана до t_c44a7cf2.
- Не делаю e2e на реальном железе (это e2e-процесс после merge; см.
  контракт воркера: «НЕ делай e2e/live-тест на реальном железе»).
- Не мёржу PR (это делает только Шифу — `CONTRIBUTING.md`, AGENTS.md).
- Не отмечаю `e2e-done` (нет e2e; и нельзя ставить без реального прогона —
  AGENTS.md, ADR-0018).
