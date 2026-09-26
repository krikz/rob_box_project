# ADR-0129: Смена DJ-персоны — очищать in-memory историю и штамповать persona в dynamic_system

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-24 |
| Автор | architect (Hermes Agent); карточка `t_cb10cab2`, issue #3000 |
| Контекст | Баг: `set_dj_mode(persona=X)` корректно обновляет `DJModeController.state.persona`, `build_auto_prompt(n)` использует свежую persona, но LLM продолжает говорить от лица ПРЕДЫДУЩЕЙ персоны. Наблюдалось на живом Vision Pi 2026-09-24 12:36–12:39 UTC: сет «Ля-Классик Мохнатый» → новый запрос «ты диджей 8-битный монстр» → spoken «Мяу, дорогие любители Моцарта» (старая персона). То же в DJ_AUTO-переходе #1 нового сета. |
| Затрагивает | `src/rob_box_voice/rob_box_voice/core/dj_mode.py` (новый хук `on_persona_change`), `src/rob_box_voice/rob_box_voice/dialogue_node.py` (хук `core.clear_history()` + `_build_dynamic_system_context` пишет `<dj_state>`), `src/rob_box_harness/rob_box_harness/core/agent_core.py` (публичный метод `clear_history` уже есть), новый файл `src/rob_box_voice/test/unit/core/test_issue_3000_dj_persona_swap.py`. |
| Родители | ADR-0037 (memory layers / DJ-scope), ADR-0001 §2.4.3 (MemoryStore port — частично пересекается), ADR-0013 (incremental delivery — это маленький, точечный фикс, не «перепишем диалоговый движок»). |
| Связанные | issue #3000 (эта задача), issue #2997 (stale-context-leak в голосовых swap'ах — та же семья), `/memories/repo/dialogue-stale-context-leak.md` (root cause), ADR-0037 (5 слоёв памяти; этот ADR закрывает конкретный acceptance criteria #3000 в RAM-слое). |

---

## TL;DR

Смена DJ-персоны — это **не просто перезапись state.persona**. Это семантически новый сценарий («новая вечеринка»), и in-memory история прошлой вечеринки не должна попадать в LLM-контекст новой. Решение в 2 слоя:

1. **Хук `on_persona_change` в `DJModeController`**: при свежем `enabled: true` с непустой persona (или при смене persona внутри идущего сета) дёргать колбэк shell'а → `AgentCore.clear_history()` (вытеснить 20 ходов старой персоны из контекста) + сохранить snapshot новой persona для `dynamic_system`.
2. **`dynamic_system` (`<system_context>` в `_build_dynamic_system_context`) рендерит `<dj_state>`** с актуальной persona/theme/plan + явный **negation-instruction**: «LLM должна говорить ТОЛЬКО от лица ЭТОЙ персоны. Старые ходы диалога могут упоминать другого диджея — игнорируй.» `dynamic_system` стоит последним system-сообщением перед user-input — самая свежая инструкция, перевешивает 20 ходов истории.

Третий слой — **тесты харнесса**: новый `test_issue_3000_dj_persona_swap.py` с моком `LLMProvider`: проверяет, что `messages`, доехавшие до LLM, после `set_dj_mode(persona=Y)` (а) **не содержат** старую persona и (б) **содержат** `<dj_state> persona=Y` в system.

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (баг #3000, юзерский сценарий, live 2026-09-24 12:33–12:39 UTC на Vision Pi, LLM minimax)

```
12:36:50 DJ farewell: Вечеринка подошла к концу. диджей Ля-Классик Мохнатый выключается…
12:37:46 STT: [TG] ты диджей 8 битный монстр и у нас сегодня слет ретро любителей приставок денди
12:37:54 set_dj_mode {'enabled': True, 'next_transition_sec': 45, 'theme': '… 8-bit chiptune party'}
12:37:54 🎧 DJ persona: '8-битный монстр'
12:37:48 [turn] spoken='Мяу, дорогие любители Моцарта и приставок! Сет продолжается.' tools=[]
12:38:40 [DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] Ты 8-битный монстр — первый в мире робот-диджей…
12:38:53 [handle_result] spoken='И вот снова я, диджей Ля-Классик Мохнатый. Вальс прозвучал — Штраус ждёт на танцполе!'
```

То есть: `DJState.persona` корректно переключился, `build_auto_prompt` для DJ_AUTO-перехода #1 корректно подставил «Ты 8-битный монстр» — **но spoken всё равно от Ля-Классик Мохнатый**. Это та же семья, что и #2997 (после смены голоса TTS робот говорит «от лица прошлого голоса»), и `/memories/repo/dialogue-stale-context-leak.md` (фундаментальная регрессия — history съедает новую persona).

### 1.2 Что уже есть

| Слой | Сейчас | Что не хватает |
|---|---|---|
| `DJState.persona` | обновляется в `DJModeController._apply_enable_payload` | — (работает корректно) |
| `build_auto_prompt(n)` | подставляет `state.persona or persona_default` | — (работает корректно для DJ_AUTO) |
| `preamble()` (user-turn) | подмешивается в `clean = preamble() + user_text` | недостаточно: только подсказка, нет negation |
| `_build_dynamic_system_context` (`<system_context>`) | speaker, TTS, hardware, session_lock, scheduler | **нет `<dj_state>`** |
| `AgentCore.clear_history()` | существует, дёргается из `_clear_session_turns` | **не вызывается на persona-change** |
| `DJModeController._apply_enable_payload` | нет shell-хука | нужна инжекция колбэка |

### 1.3 Что не нужно (out-of-scope)

- **Не нужно** расширять MemoryStore до per-DJ-scope persistence (это задача ADR-0037 — закрыть `scope=episode:<uuid>` для turns и TTL). ADR-0129 закрывает **только** in-memory часть (working + short-term RAM), которая сегодня и так обнуляется при рестарте ноды. ADR-0037 уже в Proposed; ADR-0129 — **отдельный, инкрементальный** шаг по acceptance #3000.
- **Не нужно** менять `history_trim_limit=20` — он разумный для обычного диалога. Проблема не в размере окна, а в том, что DJ-сессии переиспользуют один и тот же scope и старые DJ-ходы «отравляют» новый сет.
- **Не нужно** править `build_auto_prompt` — он уже использует актуальный state.persona. Проблема в **истории**, а не в DJ_AUTO-промпте.
- **Не нужно** вводить «hard prompt negation» в system-промпт: `dynamic_system` — это место, где runtime-факты выкладываются каждый turn заново, и LLM видит его свежим взглядом перед самым user-input. Это самый сильный рычаг для «перебить» 20 ходов старой персоны.

## 2. Решение (декомпозиция)

### 2.1 Хук persona-change в `DJModeController` (10 LOC + тесты)

```python
# dj_mode.py — новый Optional-колбэк в DJHook
@dataclass
class DJHook:
    dispatch: Callable[..., Any]
    is_active: Callable[[], bool]
    is_dialogue_active: Callable[[], bool]
    persona_default: str = "ДиДжей РОббокс"
    on_stop: Optional[Callable[[str], None]] = None
    # ADR-0129: shell-side колбэк на СМЕНУ persona (fresh start с
    # непустой persona, или persona изменилась внутри сета).
    on_persona_change: Optional[Callable[[str, str], None]] = None  # (old, new)
```

В `_apply_enable_payload` (строки 120–213):
- Запомнить `old_persona = self.state.persona` **до** записи новой.
- После записи новой persona вызвать `self._hook.on_persona_change(old_persona, self.state.persona)`, **если**:
  - `is_fresh_start` (был `was_enabled=False`, теперь `True`) и persona непустая; **или**
  - persona изменилась внутри идущего сета (`old != new` и обе непустые).
- Хук **не вызывается** на одних и тех же persona (no-op) и на persona→пустая при reset.

### 2.2 Shell-side обработчик в `dialogue_node.py`

```python
# dialogue_node.py — новый метод
def _on_dj_persona_change(self, old: str, new: str) -> None:
    """ADR-0129: persona-change → обнулить in-memory историю + штамп."""
    self._current_dj_persona = new
    # 1) clear in-memory turn window (вытесняем 20 ходов старой персоны)
    try:
        core = getattr(self, "_core", None)
        if core is not None:
            core.clear_history()
            self.get_logger().info(
                f"🎧 [ADR-0129] persona-change {old!r}→{new!r}: "
                "in-memory turn window cleared"
            )
    except Exception as exc:  # noqa: BLE001
        self.get_logger().warning(
            f"⚠️ [ADR-0129] clear_history failed: {type(exc).__name__}: {exc}"
        )
```

И в `DJHook(...)`:
```python
DJHook(
    dispatch=...,
    is_active=...,
    is_dialogue_active=...,
    persona_default=self._persona_default,
    on_stop=self._on_dj_stop_farewell,
    on_persona_change=self._on_dj_persona_change,  # ← новое
)
```

### 2.3 `<dj_state>` в `dynamic_system` (5 LOC)

В `_build_dynamic_system_context` после `<hardware>` (или перед `</system_context>`) добавить блок:

```xml
<dj_state>
  <enabled>{True/False}</enabled>
  <persona>{DJState.persona}</persona>
  <theme>{DJState.theme}</theme>
  <transition_count>{DJState.transition_count}</transition_count>
</dj_state>
<dj_persona_rule>
  Сейчас DJ-режим ВКЛЮЧЕН с персоной «{DJState.persona}». Ты — ТОЛЬКО эта
  персона: говори от её лица, используй её стиль и тему.
  Если в ИСТОРИИ ДИАЛОГА (выше) есть ходы, где ты представлялся другим
  диджеем (другая persona), это прошлый сет — ИГНОРИРУЙ те ходы и не
  повторяй старую персону. Текущая persona — единственный источник правды.
</dj_persona_rule>
```

**Почему именно сюда:**
- `<system_context>` рендерится каждый turn заново (live 10.08 two-system-prompt).
- Это **последнее** system-сообщение перед user-input — LLM смотрит на него свежим взглядом.
- LLM обычно сильно доверяет system-сообщениям больше, чем user-turn 5 ходов назад — это documented OpenAI/MiniMax behavior.

### 2.4 Тесты харнесса (новый файл)

**`src/rob_box_voice/test/unit/core/test_issue_3000_dj_persona_swap.py`** — минимум 3 теста:

1. **`test_clear_history_on_fresh_start_with_persona`**: AgentCore с `history_trim_limit=20`, добавляем 5 ходов → `handle_message({"enabled":true,"persona":"8-битный монстр"})` от диспетчера-стенда → `_turn_window` пуст + `_current_dj_persona == "8-битный монстр"`.

2. **`test_clear_history_on_persona_change_mid_set`**: AgentCore + DJ активирован с persona="Ля-Классик Мохнатый", добавляем 3 хода → `handle_message({"enabled":true,"persona":"8-битный монстр"})` (enabled уже True — fresh start=False, но persona меняется) → хук дёрнут, `_turn_window` пуст.

3. **`test_dynamic_system_contains_dj_state`**: dialogue_node mock без ROS2 + AgentCore + DJ; после `set_dj_mode` `_build_dynamic_system_context()` содержит `<dj_state>` с правильной persona **и** `<dj_persona_rule>` с инструкцией игнорировать старую persona из истории.

**`src/rob_box_harness/test/test_agent_core.py`** — расширить существующий тест `clear_history()` (он уже есть) проверкой, что `DJHook.on_persona_change` дёргается с правильными аргументами.

## 3. Trade-off анализ

| Вариант | Плюсы | Минусы | Решение |
|---|---|---|---|
| A. Только `<dj_state>` в dynamic_system, без clear_history | Минимальный diff | Не спасёт, если в истории 20 ходов «диджей Ля-Классик Мохнатый»: модель их продолжит, dynamic_system перевешивает только когда **очень свежий** | **отвергаем** |
| B. Только clear_history на persona-change | Чисто, детерминированно | LLM получает пустой контекст + ничего о новой persona в system → не знает, что говорить → hallucinate «привет, я DJ» | **отвергаем** |
| C. (B) + (A): clear_history + `<dj_state>` | Полная картина: пустой контекст + явный stamp с persona/theme | 2 точки изменения (dj_mode + dialogue_node) | **выбираем** |
| D. Per-DJ-scope persistence (ADR-0037 §2-Short-term) | Persistent — переживает рестарт | Требует расширения MemoryStore port + DDL + TTL + per-scope clear; **большой** PR | **deferred** (ADR-0037 — другая карточка) |
| E. Hard-prompt negation в master_prompt_compact.txt | Глобально | Раздувает system, влияет на ВСЕ режимы, не только DJ | **отвергаем** |
| F. Ничего не делать | Ноль работы | Баг сохраняется; live лог уже показывает регрессию | **отвергаем** |

**Решение: вариант C** (combo). ADR-0129 закрывает acceptance criteria issue #3000 **в RAM**; ADR-0037 закроет persistence часть позже, **отдельным** инкрементом.

## 4. Что меняется (минимальный diff)

| Файл | +/-, LOC | Что |
|---|---|---|
| `src/rob_box_voice/rob_box_voice/core/dj_mode.py` | +18 | добавить поле `on_persona_change` в `DJHook`; вызвать его в `_apply_enable_payload` (2 строки + запомнить `old_persona`) |
| `src/rob_box_voice/rob_box_voice/dialogue_node.py` | +30 | новый метод `_on_dj_persona_change`; пробросить его в `DJHook(...)`; в `_build_dynamic_system_context` добавить блок `<dj_state>` + `<dj_persona_rule>` (~12 строк XML) |
| `src/rob_box_voice/test/unit/core/test_issue_3000_dj_persona_swap.py` | новый, ~120 LOC | 3 теста (см. §2.4) |
| `src/rob_box_harness/test/test_agent_core.py` | +15 | расширить существующий `test_clear_history` |

Суммарно: **~180 LOC, 1 новый файл тестов, 4 файла изменено**. ADR-0013 «incremental delivery» соблюдён: 1 PR ≤ 200 LOC, можно безопасно сделать e2e один прогон.

## 5. Acceptance criteria для воркера (backend)

1. **`set_dj_mode({enabled:true, persona:"8-битный монстр"})` после сета «Ля-Классик Мохнатый»**:
   - in-memory `_turn_window` пуст сразу после `handle_message` (проверяется через mock LLMProvider + проверка `messages` отправленных в LLM).
   - Первый user-turn после смены persona: spoken **не содержит** «Моцарт/Штраус/Мяу/Дамы и господа».
   - Первый user-turn после смены persona: spoken **от лица** 8-битного монстра (произвольная тематическая фраза в его стиле).

3. **`build_auto_prompt` для DJ_AUTO-перехода #1 в новом сете**:
   - уже корректен (state.persona актуальная) — это **regression guard**, не основная фича.

4. **Dynamic system context** содержит `<dj_state>` + `<dj_persona_rule>` после любого `set_dj_mode`.

5. **Никаких** изменений в master_prompt_compact.txt, в `history_trim_limit=20`, в поведении обычного (не-DJ) диалога.

## 6. Что НЕ делается (out-of-scope, чтобы не разрастаться)

- Per-DJ-scope persistence (ADR-0037) — отдельная карточка.
- LLM-side hard negation через system prompt — `dynamic_system` уже перекрывает.
- Изменения в `preamble()` — он работает; пусть остаётся без persona-строки.

## 7. Решение по процессу

- Реализация → **backend-воркер** (`agent:backend`), карточка после `kanban complete` architect-фазы.
- e2e-тест: `t_cb10cab2` подготовит voice-команду и `.ogg` для голосового теста «ты диджей X → говорит X-стилем», положит в `.github/e2e/voice_commands/dj_persona_swap.ogg` (см. `## e2e` блок в теле issue #3000).
- **Acceptance в PR**: 4 unit-теста + 1 integration-test + 1 e2e-прогон.