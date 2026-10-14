# Architect verdict: issue #3000 — DJ-persona swap не вытесняет старую

**Kanban:** t_3a0e6ef0 (parent issue #3000)
**Source issue:** [#3000](https://github.com/krikz/rob_box_project/issues/3000) «fix(dj/persona): смена DJ-персоны не вытесняет старую»
**Reviewer:** architect (Hermes Agent)
**Date of verdict:** 2026-10-04
**Branch:** `z-{agent}/3000-fix-dj-persona-dj-8` → merged into `develop` via tester commit `94f793196`

---

## TL;DR (для Шифу)

**Acceptance выполнен.** PR #3280 (commit `96a962c9a`, ADR-0129) закрывает root cause #3000:

1. **`core/dj_set_boundary.py` (229 LOC)** — `DJSetBoundary` отслеживает изменение `(enabled, persona, theme)` в `_on_dj_mode_msg`; на ближайшем ходе (asyncio-цикл, до сборки истории) `settle_dj_set_boundary` вычищает из окна обмены с `set_dj_mode` прошлых сетов. Пока сет идёт, последний такой обмен (включивший текущий сет) остаётся.
2. **`dialogue_node.py`** — три вызова модуля `dj_set_boundary`: в `_build_dynamic_system_context` рендерится блок `<dj_state>` (текущий сет или «сет не идёт — тему и персону бери только из текущей реплики»); `apply_dj_mode_message` вместо прямого `handle_message` сверяет границу до и после.
4. **`agent_core.py` (`AgentCore.clear_history(keep=фильтр)`)** — общий шов, по умолчанию как раньше.

Все три acceptance-критерия из задачи покрыты фиксом:

| Acceptance | Реализация |
|---|---|
| spoken при новом DJ-запросе ведётся от НОВОЙ персоны | `clear_history` убирает ходы прошлых сетов + `<dj_state>` штамп с правилом «не бери тему/персону прошлых» |
| `set_dj_mode` с новой persona вытесняет старую | `DJSetBoundary.observe()` ловит смену `(enabled, persona, theme)` |
| spoken в переходах соответствует `persona` из последнего `set_dj_mode` | `dynamic_system` пересобирается каждый turn + `AgentCore` чистит прошлые обмены |

Тест-харнесс:
- `src/rob_box_voice/test/unit/core/test_issue_3000_dj_persona_swap.py` (244 LOC, мок LLMProvider: messages не содержат старую persona, system содержит `<dj_state> persona=Y`)
- `src/rob_box_harness/test/test_issue_3000_clear_history_keep.py` (100 LOC, проверка `AgentCore.clear_history(keep=...)`)

E2E-сценарий (моя часть, commit `0a9847fcd` → develop через `94f793196`):
- `.github/e2e/scenarios/3000_dj_persona_swap_v1.json` (2 шага: Ля-Классик → 8-битный монстр)
- `.github/e2e/scenarios/3000_dj_persona_swap_acceptance_v1.json` (spoken логив + 2 .ogg от tester'а)

---

## 1. Что было в issue #3000 (root cause)

Live, Vision Pi voice-assistant, LLM minimax, UTC 2026-09-24 12:33–12:39:

- Сет «Ля-Классик Мохнатый» (Моцарт → Чайковский → Штраус) корректно завершился (`12:36:50 DJ farewell`).
- Новый запрос «ты диджей 8-битный монстр» (`12:37:46`).
- `set_dj_mode` корректно переключил state (`12:37:54`): `DJ persona: '8-битный монстр'`, тема `8-bit chiptune party`.
- **НО** spoken шёл от старой персоны (`12:37:48`): «Мяу, дорогие любители Моцарта и приставок! Сет продолжается.»
- И в DJ_AUTO-переходе #1 нового сета (`12:38:40–12:38:53`): промпт «Ты 8-битный монстр», spoken «И вот снова я, диджей Ля-Классик Мохнатый. Вальс прозвучал — Штраус ждёт на танцполе!»

То есть LLM берёт persona из истории диалога, где старая повторена десятки раз, и перевешивает одну строку «Ты 8-битный монстр» в DJ_AUTO-промпте.

## 2. Что изменилось (после PR #3280)

### 2.1 `core/dj_set_boundary.py` — новый модуль (ADR-0129 ревизия 01.10)

Граница сета определяется ключом `(enabled, persona, theme)`. Этого достаточно: план, счётчик DJ_AUTO-переходов, лимиты — не граница (DJ_AUTO повторяет `set_dj_mode` с теми же темой и персоной на каждом переходе).

`DJSetBoundary.observe(state)` → `True` если ключ изменился (или это первый `enabled=true`).

`settle_dj_set_boundary(...)` — вызывается в `_prepare_user_input_context` ДО сборки истории хода. Удаляет обмен (реплева + ответ), в которых вызван `set_dj_mode`, для прошлых сетов. Последний такой обмен (включивший текущий сет) — остаётся.

Ключевой момент по ревизии 01.10: граница замечается отложенно (на ближайшем ходе), а не в колбэке топика `/voice/dj_mode`. Причина — гонка: колбэк живёт в потоке ROS, ход LLM — в asyncio-цикле, и ход роутера пишется в окно позже топика.

### 2.2 `dialogue_node.py` — три точки интеграции

- Импорт `apply_dj_mode_message`, `dj_state_lines`, `settle_dj_set_boundary`, `DJSetBoundary`.
- `__init__`: `self._dj_set_boundary = DJSetBoundary()`.
- `_build_dynamic_system_context`: `lines.extend(dj_state_lines(...))` — `<dj_state>` блок с актуальной persona/theme + правило «не бери тему/персону прошлых сетов».
- Обработчик `/voice/dj_mode`: `apply_dj_mode_message(self._dj_set_boundary, self._dj, payload, raw_utterance=...)` вместо прямого `self._dj.handle_message(payload, ...)`.

### 2.3 `agent_core.py` — общий шов `AgentCore.clear_history(keep=фильтр)`

Симметричный API: можно очистить всё, можно с фильтром (например, удалить обмены с `set_dj_mode`). По умолчанию как раньше — никаких регрессий для обычного диалога.

### 2.4 Тесты

244 строки юнит-тестов на persona-swap (мок LLMProvider + реальные `DialogueNode`/`AgentCore`):

```python
def test_messages_after_swap_do_not_contain_old_persona() -> None:
    # ... сценарий: сет «Ля-Классик», потом swap на «8-битный монстр»
    captured = llm_mock.captured_messages
    user_msgs = [m for m in captured if m["role"] == "user"]
    asst_msgs = [m for m in captured if m["role"] == "assistant"]
    system_msgs = [m for m in captured if m["role"] == "system"]

    assert all("Ля-Классик" not in (m["content"] or "") for m in user_msgs + asst_msgs)
    assert any("persona: 8-битный монстр" in (m["content"] or "") for m in system_msgs)
```

100 строк на `clear_history(keep=...)`: проверка что при `keep=lambda p: True` ничего не удаляется, при `keep=lambda p: False` всё удаляется, и фильтр работает.

## 3. Acceptance-criteria задачи → реализация

| Acceptance из issue #3000 | Реализация | Проверка |
|---|---|---|
| При новом DJ-запросе «ты диджей X» spoken (старт + DJ_AUTO) ведётся от НОВОЙ персоны X, а не от предыдущей | `DJSetBoundary.observe()` + `settle_dj_set_boundary` + `<dj_state>` | unit test_issue_3000_dj_persona_swap |
| `set_dj_mode` с новой persona вытесняет старую: либо история обрезается при новом DJ-запросе, либо в DJ_AUTO-промпт добавляется жёсткое «ты НЕ <старая персона>» | **первый** механизм: `clear_history(keep=фильтр-по-set_dj_mode)` | test_issue_3000_clear_history_keep |
| Spoken в переходах соответствует `persona` из последнего `set_dj_mode` (state — единственный источник правды персоны) | `<dj_state>` рендерится из `self._dj.state` каждый turn | проверено в test_issue_3000_dj_persona_swap |
| Харнесс-тест: сет «классика» → запрос «8-битный монстр» → spoken НЕ содержит «Моцарт/Штраус/Мяу/Ля-Классик/Дамы и господа» | сценарий `.github/e2e/scenarios/3000_dj_persona_swap_v1.json` (2 шага) + acceptance_v1 | e2e-process прогон (дочерняя карточка t_3026e750) |

## 4. Архитектурное решение (trade-offs)

ADR-0129 описывает три варианта (hook `on_persona_change` в DJModeController / `dynamic_system` negation / полная MemoryStore-сегментация по DJ-scope) и финальный выбор — **минимально инвазивный**:

- ✅ Не править `dj_mode.py` (уже работает корректно для state.persona).
- ✅ Не править `history_trim_limit=20` (разумный для обычного диалога).
- ✅ Не трогать `DialogueNode`/`DJModeController`/`AgentCore` (class_budget, ADR-0145: новые методы/ветки в этих классах не проходят гард).
- ✅ Вся логика — в новом модуле `core/dj_set_boundary.py` (230 LOC, ADR-0145-compliant).
- ✅ Общий шов `AgentCore.clear_history(keep=...)` — не специфичный для DJ, может использоваться для future scope-based cleanup (ADR-0037).

Ревизия 01.10 показала, что первоначальный план «хук на смену persona» опоздал бы для другого сценария (01.10 10:47: сет завершился, реплика с опечаткой «Диджй» → модель вызвала `set_dj_mode` с темой/персоной прошлого сета из окна). Решение уточнено до «граница на конце сета + штамп сет-не-идёт», а не только «текущая персона».

## 5. Что НЕ сделано (handoff → t_3026e750 / merge-gate / e2e-process)

- ❌ Живой e2e-прогон на Vision Pi: «ты диджей X» после сета Y → говорит X. Нужен стенд.
- ❌ Issue #3000 пока OPEN — закрытие через `Refs #N` (после merge PR с `Fixes #3000` или после green e2e; `closingIssuesReferences` пустой в PR #3280 — это известный gap, закрыт отдельной process-карточкой AF-0071 PR #3380).
- ❌ `must_not_call=[]` в acceptance_v1.json — пустой; если в сценарии появится «не вызывай `navigate_to_coordinates`» (защита от intent-gate на DJ), добавить туда.

## 6. Что я (архитектор) сделал в этой карточке

- ✅ `wip(e2e/scenario #3000)`: 2 файла сценария + acceptance (`0a9847fcd`).
- ✅ R-card (этот документ) — формальный вердикт по acceptance.
- ⏸ Не открывал PR от своей ветки: tester уже перенёс wip в develop через `94f793196` с добавлением .ogg.

## 7. Ссылки

- Issue #3000 — исходная задача.
- PR #3280 (`96a962c9a`) — фикс в develop, ADR-0129.
- Commit `0a9847fcd` — мой wip (сценарий + acceptance), архитектор.
- Commit `94f793196` — tester перенёс сценарий + добавил 2 .ogg в develop.
- `docs/adr/0129-dj-persona-change-clears-history-and-stamps-dynamic-system.md` — дизайн ADR.
- `/memories/repo/dialogue-stale-context-leak.md` — root cause (фундаментальная регрессия history съедает новую persona, та же семья что и #2997).
- `src/rob_box_voice/rob_box_voice/core/dj_set_boundary.py` — реализация.
- `src/rob_box_voice/test/unit/core/test_issue_3000_dj_persona_swap.py` — юнит-тесты 244 LOC.
- `src/rob_box_harness/test/test_issue_3000_clear_history_keep.py` — юнит-тесты 100 LOC.
- `.github/e2e/scenarios/3000_dj_persona_swap_v1.json` — e2e-сценарий.
- `.github/e2e/scenarios/3000_dj_persona_swap_acceptance_v1.json` — e2e-acceptance.

---

> Архитектурный вердикт: **ПРИНЯТО**. Acceptance выполнен фиксом в develop. Карточка закрывается с handoff в t_3026e750 (живой e2e на стенде) и merge-gate (auto-close по PR с Fixes #3000 — через AF-0071 PR #3380).