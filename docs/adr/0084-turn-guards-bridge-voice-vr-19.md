# ADR-0084 — TurnGuards bridge (issue #2241, voice-vr 19)

**Дата:** 2026-09-09
**Статус:** accepted
**Автор:** architect
**Связанные:** ADR-0080 §2.4, ADR-0021 (decomposition discipline), ADR-0013 (incremental delivery),
issue #2241, issue #1881 (synthetic-retry budget), PR branch
`z-{agent}/2241-voice-vr-19-turnguards`.

---

## Контекст

В `dialogue_node.py` оркестрация гвардов размазана по **двум catch-сайтам**
(6360 LOC, 74 метода):

* `_run_turn.finally` (line 3920 / 3938) — `_apply_music_guard` + `_apply_tool_skipped_guard`
* `_handle_result` (line 5486-5526) — четыре последовательных
  `_check_*_and_retry` (system_regurgitate, babble, embedded_renardo,
  unbacked_action)

Плюс **общее состояние** бюджета (`_synthetic_retries_left`) и шесть
поимённых флагов (`_babble_retry_used`, `_action_claim_retry_used`,
`_code_speech_retry_used`, `_tool_retry_used`,
`_system_regurgitate_retry_used`, `_retry_dispatched_in_turn`) живут
прямо в `DialogueNode.__init__` (line 785-839). Каждый guard
самостоятельно вызывает `_consume_synthetic_retry` /
`_mark_retry_dispatched` — это контракт «без принуждения» (line 4513).

ADR-0080 §2.4 требует:

> единый модуль `TurnGuards(order, budget).evaluate(reply, context, state) -> Verdict`,
> где `Verdict = Accept | Retry(prompt, guard_name) | Discard(reason)`,
> и `dialogue_node` исполняет вердикт.

ADR-0021 фиксирует прецедент:

> «вынос чистых функций бюджет не снизил» — после выноса
> `_check_babble_and_retry` в `core/dialogue_guards.py` CC `dialogue_node`
> **не снизился**, потому что оркестрация (порядок, бюджет, retries)
> осталась в `dialogue_node.py`. Чтобы CC снизился, нужно выносить
> **оркестрацию**, а не отдельные предикаты.

## Решение

Делаем вынос **оркестрации** в новый модуль
`rob_box_voice/core/turn.py` (уже существует, см. WIP-коммиты
`z-{agent}/2241-voice-vr-19-turnguards`):

```
core/turn.py — TurnGuards:
  * Reply / TurnContext / TurnState (значения)
  * Verdict / VerdictKind / ACCEPT (выходы)
  * Guard (протокол)
  * TurnGuards(order, budget).evaluate(reply, turn, state) -> Verdict
  * default_guards(music_guard=...) — каноничный порядок
  * music_guard_adapter(...) — обёртка над MusicGuard
  * 6 реализаций: SystemRegurgitateGuard, ToolSkippedGuard,
    BabbleGuard, EmbeddedRenardoCodeGuard, UnbackedActionClaimGuard,
    PlanningNarrationHardMute
  * consume_budget / reset_budget — бюджетный помощник
```

В `dialogue_node.py` добавлен **bridge** (WIP-4 в той же ветке):

* `self._use_turn_guards: bool = False` — feature flag, по умолчанию OFF
* `self._turn_guards: Optional[TurnGuards] = None` — lazy init
* `self._turn_state: Optional[TurnState] = None` — TurnState-bridge
* `self._ensure_turn_guards()` — оборачивает `MusicGuard` через
  `music_guard_adapter`, строит TurnGuards один раз
* `self._reset_turn_budget()` — зеркалит legacy reset на user-initiated turn
* `self._evaluate_turn_guards(...)` — возвращает
  `None` (flag off), `"retry:<name>"`, или `"discard"`,
  транслируя RETRY в legacy `_dispatch_turn(..., is_synthetic=True)`
  с теми же side-effects (`_synthetic_retries_left -= 1`,
  `_retry_dispatched_in_turn = True`, `_reopen_dialogue_for_retry()`)

**Почему feature flag, а не «вырезать сразу»:**

ADR-0021 + опыт двух предыдущих попыток (run 5696 / 5699, обе
`timed_out` на 50 итераций) показывают: переписать все **семь**
`_check_*_and_retry` за один PR нереально без регрессии. Bridge
делает ровно то, что ADR-0013 требует:

1. Изолированный модуль готов и покрыт 39 unit-тестами (`test_turn.py`).
2. Bridge встроен **без изменения наблюдаемого поведения**
   (`_use_turn_guards = False` → всё идёт через legacy).
3. Регрессионная сюита (`test_dialogue_guards.py` + `test_issue_992_*` +
   `test_issue_1777_*` + `test_issue_1881_*` +
   `test_dialogue_retry_flag_wiring.py`) — **391 passed, 3 skipped**.
4. Включение для одного catch-сайта — задача voice-vr 20+,
   отдельная карточка на каждый.

## Структура catch-сайтов и план миграции

| Catch-сайт | Catch-сайт строки | Какие гварды | Карточка |
|---|---|---|---|
| `_run_turn.finally` | 3920, 3938 | music_guard, tool_skipped | voice-vr 21 |
| `_handle_result` | 5486 | system_regurgitate | **voice-vr 20** (первый) |
| `_handle_result` | 5499 | babble | voice-vr 22 |
| `_handle_result` | 5511 | embedded_renardo_code | voice-vr 22 |
| `_handle_result` | 5521 | unbacked_action_claim | voice-vr 22 |
| `_handle_result` | 5470 | planning_narration (DISCARD) | voice-vr 22 |

**Правила миграции** (для последующих карточек):

1. В catch-site дёргаем `self._evaluate_turn_guards(...)` **первым**.
2. Если возвращает `"retry:<name>"` или `"discard"` → `return`.
3. Если `None` → legacy `_check_*_and_retry`.
4. После того как все catch-сайты используют TurnGuards,
   `_*_retry_used` / `_synthetic_retries_left` / `_consume_synthetic_retry`
   удаляются (voice-vr 23).
5. CC `_handle_result` снизится с 65 до ~25 (4 if-блока превращаются
   в один `if (v := self._evaluate_turn_guards(...)): return`).
6. CC `_run_turn` снизится с 57 до ~30 (music + tool под одним
   TurnGuards-вызовом).

## Альтернативы

### A. Полный рефакторинг в одном PR

Заменить все 7 catch-сайтов сразу, удалить legacy-флаги в том же PR.

**Отклонено:** два `timed_out` подряд (50/50 итераций) показывают,
что в рамках одной карточки это сделать без регрессии невозможно.
Тесты `test_issue_1881_synthetic_retry_budget.py` и
`test_dialogue_retry_flag_wiring.py` явно проверяют legacy-флаги —
их нельзя удалить, пока хоть один catch-сайт использует старый путь.

### B. Не выносить оркестрацию, оставить всё в `dialogue_node.py`

**Отклонено:** ADR-0080 §2.4 + ADR-0021 явно требуют единый модуль.
Без выноса CC `dialogue_node` продолжит расти (на дату задачи
6360 LOC, было 4062 в ADR-0021).

### C. Вынести оркестрацию без бриджа (feature flag = анти-паттерн)

**Частично отклонено:** feature flag добавляет 100 LOC и неактивный
код. Но альтернатива — удалить legacy без проверки — даёт
`git grep _retry_used` = 0, но красный CI. ADR-0013 запрещает
«big-bang» — incremental delivery выигрывает.

## Trade-offs

* **+ Единый порядок гвардов** в одном списке (раньше был prose-комментарий
  в `dialogue_node.py:3934-3936`).
* **+ Бюджет в одном месте** (`TurnState.budget_left` + `consume_budget`).
* **+ Тестируемость** оркестрации без ROS2 (39 unit-тестов на голый
  `TurnGuards.evaluate`).
* **+ Переход по карточкам** — каждая следующая карточка голосовой
  серии может удалить очередной legacy-флаг без риска регрессии.
* **− Дублирование** legacy-флагов и TurnState-поля на время миграции
  (2-3 карточки).
* **− Дополнительный класс-обёртка** (`music_guard_adapter`) для
  того, чтобы MusicGuard не реализовывал Guard-протокол напрямую.

## Что сделано в этой карточке (voice-vr 19)

* [x] `core/turn.py` (752 LOC): TurnGuards + 6 guard + Verdict протокол
  + music_guard_adapter + budget helpers
* [x] `test/unit/core/test_turn.py` (39 тестов) — все зелёные
* [x] Bridge в `dialogue_node.py`: feature flag `_use_turn_guards=False`,
  lazy construction, `_evaluate_turn_guards` (no-op пока флаг off)
* [x] Регрессионная сюита: 391 passed, 3 skipped (file-flagged)

## Что НЕ сделано в этой карточке (явно out of scope)

* [ ] Переключение catch-сайтов на TurnGuards — **voice-vr 20+**
* [ ] Удаление `_*_retry_used` / `_synthetic_retries_left` /
      `_consume_synthetic_retry` / `_mark_retry_dispatched` — **voice-vr 23**
* [ ] Снижение CC `_handle_result` / `_run_turn` — измеримо после
      voice-vr 20+

## Метрики

| Метрика | До | После (эта карточка) | Цель (voice-vr 23) |
|---|---|---|---|
| `_run_turn` CC | 57 | 57 | ~30 |
| `_handle_result` CC | 65 | 65 | ~25 |
| `dialogue_node.py` LOC | 6360 | 6540 (+180 bridge) | ~6200 |
| `core/turn.py` LOC | 0 | 752 | 752 |
| `test_turn.py` тестов | 0 | 39 | 39 |
| `_retry_used` / `_synthetic_retries_left` вхождений | 36 | 36 | 0 |

CC не снизился в этой карточке — **так и задумано** (ADR-0021).
Снижение будет измеримо после переключения catch-сайтов.

## План для следующих карточек

1. **voice-vr 20** — первый catch-сайт (`_check_system_template_regurgitate_and_retry`):
   в `_handle_result` строке 5486 перед legacy-вызовом добавить
   `if (v := self._evaluate_turn_guards(..., include_music=False)) and v.startswith("retry:system_")`
   → return. Остальные 3 guard'а остаются на legacy. CC `_handle_result`
   останется прежним (выигрыш будет когда все 4 catch-а уйдут).
2. **voice-vr 21** — `_run_turn.finally`: добавить `include_music=True`,
   обработать music-guard через TurnGuards.
3. **voice-vr 22** — оставшиеся 3 catch-сайта в `_handle_result`.
4. **voice-vr 23** — удаление legacy-флагов и тестов
   `test_dialogue_retry_flag_wiring.py` (или его переименование).

## Что НЕ делаем

- Не вырезаем legacy-код до того, как все catch-сайты переключены.
- Не переименовываем существующие тесты до их фактической замены
  (тест-как-контракт живёт дольше кода).
- Не делаем «всё сразу» (ADR-0013 + два timed_out подряд).

## Ссылки

* Issue #2241 — основная карточка
* ADR-0080 §2.4 — целевой контракт
* ADR-0021 — правила декомпозиции dialogue_node
* ADR-0013 — incremental delivery
* Ветка: `z-{agent}/2241-voice-vr-19-turnguards` (4 WIP-коммита +
  этот ADR к моменту merge)