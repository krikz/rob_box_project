# Архитектурное ревью `dialogue_node.py` — 2026-08-18

**Репортёр:** architect
**Severity:** HIGH (god-class, регрессии на каждой правке)
**Скоуп:** `src/rob_box_voice/rob_box_voice/dialogue_node.py` @ `develop`
**Контекст:** после #1386/#1389/#1390/#1395/#1403 (батчинг gating + SSoT для counter)
**ADR кандидат:** ADR-0021 (см. конец документа)

---

## 0. TL;DR

`dialogue_node.py` — **god-class на 4062 строк** с 79 методами и десятками скрытых
состояний. За последние 18.08 на нём произошли **три боевых регрессии**
(#1385/1386/1389/1390/1395/1403) — все из-за одной и той же причины: **класс
растёт быстрее, чем архитектура его сдерживает**. Каждое «маленькое»
расширение (gating по `/voice/e2e/busy`, `e2e_busy` counter, music_guard
edge case) — это 5–15 строк в случайном месте файла, и через неделю мы не
помним, что они там есть.

**Метрики (см. §1)** показывают, что боттлнек — **не в LLM и не в e2e**, а в
том, что **один файл делает 7 разных работ** и забывает одну из них. Решение —
**не рефакторить всё сразу** (KISS), а **оформить per-bag worker-карточки** на
каждый серьёзный баг + **нарезать извлечения** в `core/dialogue_guards.py` /
`core/startup_greeting.py` / `core/llm_skip_reasons.py` (TD-2 decomposition).
Воркеры реализуют, юзер мержит (по процессу ADR-0014).

---

## 1. Метрики (baseline @ `develop` после #1390 merge)

| Метрика                      | Значение | Где                                           |
| ---------------------------- | ------- | --------------------------------------------- |
| **LOC всего**                | **4062** | `dialogue_node.py`                            |
| LOC класса `DialogueNode`    | **3790** | строки 255–4044                                |
| Методов в `DialogueNode`     | **74**   | все internal/public/приватные                  |
| Публичных `_on_*` хэндлеров  | **18**   | ROS2 subscriptions                             |
| `__init__` LOC               | **329**  | инициализация всех зависимостей                |
| `__init__` CC                | **12**   | много условных блоков по фичам                |
| Cyclomatic complexity max    | **47**   | `_run_turn` (368 LOC, гигантская try/except)  |
| CC вторая                    | **44**   | `_handle_result` (349 LOC)                    |
| CC третья                    | **28**   | `_on_stt` (161 LOC)                           |
| **Lazy imports внутри тела** | **18**   | по 1 на фичу (FAQ/music/web_search/...)       |
| TODO/FIXME                   | **1**    | `# TODO #1160, шаг 2B` (record_fallback)      |
| Ссылок на `Issue #NNNN`      | **60+**  | changelog внутри файла                        |
| Упоминаний ADR               | **0**    | ни одного прямого `See ADR-NNNN`              |

### Топ-15 методов по CC

| Метод                              |    CC |  LOC | Line |
| ---------------------------------- | ----: | ---: | ---: |
| `_run_turn`                        |  **47** |  368 | 2138 |
| `_handle_result`                   |  **44** |  349 | 3292 |
| `_on_stt`                          |  **28** |  161 | 1419 |
| `_build_single_provider`           |  **27** |  106 |  816 |
| `_continue_after_tool_calls`       |  **22** |  168 | 2942 |
| `_do_recursive_streaming`          |  **22** |   93 | 3111 |
| `_execute_tool_calls`              |  **18** |   86 | 3205 |
| `_build_dynamic_system_context`    |  **15** |   91 | 1976 |
| `_build_llm`                       |  **14** |   88 |  923 |
| `_apply_music_guard`               |  **13** |  126 | 2771 |
| `__init__`                         |  **12** |  329 |  257 |
| `_load_event_profile`              |  **12** |   41 | 1198 |
| `_on_tts_batch_complete`           |  **12** |   62 | 1722 |
| `_build_event_faq_prefetch_context`|  **11** |   25 | 1282 |
| `_check_babble_and_retry`          |  **11** |   93 | 2645 |

**Рекомендация (не рефакторинг, а SSoT-disciplin):**
- `_run_turn` 47 → ≤15 (выделить speaker-prelude / metrics-span / retry-classification)
- `_handle_result` 44 → ≤15 (выделить music_guard, babble_guard, strip_response)
- `_on_stt` 28 → ≤10 (выделить `parse_tg_marker`, `check_wake_gate`,
  `check_silence_state`, `check_command_intent_gate`)

Это **3-5 воркер-карточек**, не один «рефактор».

---

## 2. SOLID-разбор (точечно)

### 2.1 SRP — единоличная ответственность

`DialogueNode` сегодня отвечает за:

1. **ROS2 shell** — pub/sub на 18 топиков (`/voice/stt/result`,
   `/voice/tts/finished`, `/voice/e2e/busy` и т.д.)
2. **LLM-цикл** — `_run_turn`, `_handle_result`, `_continue_after_tool_calls`,
   `_do_recursive_streaming`, `_execute_tool_calls`
3. **Provider chain** — `_build_single_provider`, `_resolve_provider_chain`,
   `_build_llm`, `_deepseek_balance_probe`, `_is_llm_unavailable_error`
4. **Tool/MCP routing** — `_build_tool_provider`, `_build_skills`,
   `_on_mcp_tools_update`
5. **Memory/speaker** — `_build_memory`, `_handle_speaker_turn`,
   `_apply_speaker_identity`, `_speaker_by_text` буфер
6. **Диалог/guards** — `_on_stt` (7 inline guards), `_apply_music_guard`,
   `_check_babble_and_retry`, `_build_dj_retry_prompt`, `_user_wants_music*`
7. **TTS lifecycle** — `_publish_response`, `_publish_response_batch`,
   `_register_active_batch`, `_on_tts_finished`, `_on_tts_batch_complete`
8. **Startup greeting** — `_on_startup_greeting*` (3 фазы + timer)
9. **Metrics/tracing** — `_maybe_record_session_end`, `_maybe_log_skip_summary`
10. **DJ-mode hook** — `_dispatch_dj_turn`, `_on_dj_stop_farewell`,
    `_dispatch_turn`
11. **Events** — `_load_event_profile`, `_render_event_instructions`

Это **11 разных ответственностей** в одном классе. Нормально для «glue»-узла,
но **он перестал быть тонкой обёрткой** — он стал средоточием бизнес-логики.

### 2.2 OCP — расширяемость без модификации

**Не выполняется.** Каждое новое правило (e2e_busy, command_intent_gate,
silence vs music_stop) — это **inline if** в `_on_stt` или `_handle_result`.
Декораторов/policy-цепочек нет. PR #1386 (gating) добавлял `if is_e2e_busy: ...`
внутрь метода — и забыл инициализировать counter → crash #1389.

**Связанный код-запах**: ручной `dict literal` для `_llm_skipped_counter`
строка 370 — это бомба, повторяющая паттерн #1389 на любом будущем
increment-сайте.

### 2.3 LSP / ISP / DIP

- **LSP**: N/A (нет иерархии классов, всё в одном).
- **ISP**: ноды смешивают «чтение из ROS» и «принятие решения» в одном методе.
  Тестирование требует `rclpy` + spin — это и есть причина, почему edge-case
  тесты на gating не словили #1389.
- **DIP**: частично соблюдён через `ToolProvider`, `LLMProvider`, `MemoryStore`
  из `rob_box_harness` (9 модулей импортируется). Но внутри `DialogueNode` ещё
  живут собственные реализации (`_FallbackLLM`, ручной publish на
  `/voice/dialogue/response` и т.д.), которые **минуют harness-абстракции**.

---

## 3. KISS / DRY

### 3.1 KISS нарушения

- `_apply_music_guard` (126 LOC) — Bug B и Bug C сплетены в одном методе
  с переплетением retry-budget + DJ-state + vocal-request-override. **Каждый
  bag в `#992` добавлял if-ветку**, никто не выделял подфункции.
- `_run_turn` — 47 CC, 368 LOC; тело одного `try/except` для всей LLM-сессии.
- 18 lazy-imports в случайных местах (`yaml`, `uuid`, `concurrent.futures`,
  `traceback`, `ament_index_python`). Импорт — это уже side-effect; если он
  вдруг упадёт (например, удалили `tts_voice_registry`), ошибка всплывёт в
  рантайме первого вызова, а не при импорте модуля.

### 3.2 DRY нарушения

- `_publish_response` vs `_publish_response_batch` — две функции для одного
  контракта (`build_ssml_payload` + publisher), отличаются только batch_id.
  Возможно объединить через kwargs.
- `try/except (TypeError, ValueError): speaker_duration_s = 0.0` повторяется
  дважды (line 1451/1452) — мелочь, но индикатор «мы не доверяем типу».
- Lazy-import `from rob_box_voice.core.dialogue_state_machine import (...)` в
  строках 2704 и в начале файла — **тот же модуль** импортирован дважды, в
  разных scope.
- `_startup_greeting_fired` (флаг) + `_greeting_timer` (rclpy-timer) — два
  независимых механизма «не повторять». Достаточно одного.

---

## 4. Конкретные баги, которые уже выстрелили

### 4.1 `#1389` — `_llm_skipped_counter` KeyError на первом STT (CRASH)

**Что произошло (PR #1386, commit `91bf3a8a`):**
- Добавили `self._llm_skipped_counter["e2e_busy"] += 1` в `_on_stt`.
- Забыли добавить `"e2e_busy": 0` в литерал на строке 370.
- Voice-assistant крашнулся на первом wake-word после deploy.

**Зафиксированный fix:** PR #1390 (revert). Открыт **PR #1395** с defensive
SSoT — кортеж `_LLM_SKIP_REASONS = (...)` → dict строится из него.

**Что я вижу в коде @ develop:** кортежа **нет**, всё ещё литерал, **7
increment-сайтов** руками (lines 1466, 1471, 1493, 1502, 1513, 1539).

**Аromорфный риск:** пока PR #1395 не влит, **любая правка в `_on_stt`**
снова может уронить prod. Это **не баг в одном месте — это bug class**.

### 4.2 `#1403` — `music_skill_prompt.txt` НЕ обновлён

Это не баг `dialogue_node.py`, но **сосед**: PR #1398 (music, merge) добавил
`generate_music` MCP tool, но **LLM не знает о нём** — `music_skill_prompt.txt`
(внешний артефакт) остался старым. LLM в ответ на «спой песенку» говорит «нет
такой функции». **Симптом**: код в роботе есть, **знания в промпте нет**.

Это урок для `dialogue_node._load_system_prompt` (line 708): нет проверки,
что для всех tools из MCP-registry есть описание в `music_skill_prompt.txt`.
**Defensive SSoT-вариант**: список tools из `_on_mcp_tools_update` → diff с
упоминаниями в системном промпте → warn если tool не описан.

### 4.3 `#1363` — startup_greeting свист (#1003 → 06.08 редизайн)

**Что в коде:**
- `_startup_greeting_fired` (строка 691) — `bool` флаг на уровне self.
- `_greeting_timer` — rclpy-timer, создаётся в `_on_startup_greeting`,
  отменяется в `_on_startup_greeting_finish`/`_speak` через
  `_cancel_greeting_timer`.

**Edge cases, которые я вижу (без live-прогона, чтение кода):**
1. **Параметр `startup_greeting_sec=0`** — таймер не создаётся, флаг
   `_startup_greeting_fired` остаётся `False` навсегда; если параметр
   пере-выставляется в runtime (через `set_parameters_callback`), приветствие
   никогда не сработает, потому что флаг **не per-node-restart, а per-instance**.
2. **Race**: если юзер говорит в окне 0…12с (`_startup_greeting_sec=12`) →
   `_on_stt` поднимает state из IDLE → `_on_startup_greeting` всё равно
   запускается через 12с, **если DSM не успел вернуться в IDLE к этому
   моменту** — guard есть (`if self._dsm.current_state != IDLE: return`), но
   `_startup_greeting_fired = True` уже выставлен → повторно не сработает.
3. **`destroy_node`**: `_cancel_greeting_timer` есть, но **не вызывается** в
   `destroy_node` (lines 4040+). На shutdown активный rclpy-timer может
   триггернуть колбэк после `destroy_node` → log-шум.
4. **Timer race + flag**: если `_on_startup_greeting` ставит флаг ДО проверки
   DSM-состояния, и DSM не IDLE — приветствие пропущено, но флаг уже True.
   На следующем startup (restart node) **новый инстанс** DialogueNode
   имеет `False`, всё ок. **А если параметр поменяли в runtime и снова
   меняют обратно** — флаг останется True навсегда.

**Что не проверил вживую:** нужен live-deploy + runtime-параметр эксперимент.
Это **нельзя утверждать как bug** без прогона. См. §7 — это **hypothesis bag**.

### 4.4 music_guard edge cases (Bug B/C/D паттерн)

`_apply_music_guard` (126 LOC) — три сценария:
- **Bug B (DJ auto)**: нет `execute_music_code` → sync retry до
  `MAX_DJ_AUTO_RETRIES=2` → ребилд через `_reopen_dialogue_for_retry`.
- **Bug C (user music)**: нет `execute_music_code`, но юзер явно просил →
  vocal override (если есть `speak_text`) / sync retry до
  `MAX_MUSIC_GUARD_RETRIES` / spoken nudge fallback.
- **Bug D (babble)**: тут не здесь, но связан через `_check_babble_and_retry`.

**Edge cases, которые я вижу:**
1. **`MAX_DJ_AUTO_RETRIES` сбрасывается в одном месте** (`_apply_music_guard`
   line 2807), но **`MAX_MUSIC_GUARD_RETRIES` сбрасывается в `_run_turn`**
   line 2171. Если guard запускается через другую точку входа (пока не
   нашёл), состояние может разойтись.
2. **`is_vocal_request(user_input)` + `tools_now` non-empty** → return False.
   Но `is_vocal_request` не различает «speaking-only» vs «speak_text +
   execute_music_code (без DJ)» — для второго случая `tools_now` уже
   содержит music, guard skip **правильно**, всё ок.
3. **Retry budget exhaustion + DJ state**: если DJ был в середине
   `transition_count=N`, retry ребилдит с тем же `N` → следующий auto-tick
   может быть **рассинхрон с реальным прогрессом**. Не критично (5-сек tick),
   но индикатор «retry-логика думает в своей вселенной».
4. **Stop-override race**: `is_music_stop_command` пропускает guard
   полностью, но если юзер сказал «хватит, поиграй другую» — guard
   увидит «другую» → music-request → retry → DJ зациклится.

### 4.5 `_handle_result` — гигантский if-elif

`_handle_result` (CC=44, 349 LOC) — обрабатывает 8+ веток:
- success without tools
- success with tools (дёргает `_apply_music_guard`)
- success with empty tools + DJ auto
- success with vocal content
- success with stop_music
- error (`ProviderError` vs generic)
- babble-detected → schedule retry
- fallback (`_generate_fallback_response`)

Каждое добавление нового сценария = новый if-elif в этом методе. Это **тот
же antipattern, что и `_apply_music_guard`**: все guard-логики в одном теле.

---

## 5. Что НЕ трогать (KISS — не делаем серебряную пулю)

Я **не предлагаю**:
- ❌ Полный рефакторинг в стиле «DialogueNode → 6 классов с DI» — это
  big-bang риск, а архитектура **должна обслуживать бизнес**, не наоборот.
- ❌ Заменять rclpy-NODE на чистую функцию в harness — это другой
  слой (см. ADR-0013 «incremental delivery over big-bang»).
- ❌ Удалять inline-комментарии `Issue #NNNN` — это **полезный changelog**,
  при рефакторинге они остаются ссылками.
- ❌ Менять публичные API без явного OK.

Я **предлагаю** (ADR-0021 draft в §8):
- ✅ Per-bag worker-карточки (issues 1405–1411 ниже) — каждая **локальна** и
  **не мешает** другим.
- ✅ SSoT `_LLM_SKIP_REASONS` кортеж — фикс бага-класса #1389.
- ✅ Выделить `core/dialogue_stt_gate.py` — 7 inline guards из `_on_stt`
  в один класс-стратегию.
- ✅ Выделить `core/startup_greeting.py` — startup logic + timer.
- ✅ Выделить `core/music_guard.py` — `_apply_music_guard` + retry budget
  logic.
- ✅ Defensive проверка tools-vs-prompt в `_load_system_prompt`.

---

## 6. Per-bag issue-list (готовые тела в `/tmp/kc_logs/issue_1405_*.md`)

| # | Issue title | Severity | Bag class | Где в коде |
|---|---|---|---|---|
| 1405 | feat(refactor voice): SSoT `_LLM_SKIP_REASONS` кортеж — предотвратить повтор #1389 | high | bug-class | `__init__` line 370 + 7 inc-сайтов |
| 1406 | feat(refactor voice): выделить `core/dialogue_stt_gate.py` — STT guards | medium | CC>10 | `_on_stt` line 1419 (CC=28) |
| 1407 | feat(refactor voice): выделить `core/startup_greeting.py` — таймеры + flag | medium | edge-cases | `_on_startup_greeting*` line 3645 |
| 1408 | feat(refactor voice): выделить `core/music_guard.py` — Bug B/C + retry budget | medium | CC>10 | `_apply_music_guard` line 2771 |
| 1409 | feat(refactor voice): SSoT tools-vs-prompt в `_load_system_prompt` — связано с #1403 | high | regression-class | `_load_system_prompt` line 708 |
| 1410 | feat(voice #1363): старт-up greeting — hypotheses для live-проверки | low | hypothesis | `_startup_greeting_fired` + `_greeting_timer` |
| 1411 | feat(refactor voice): `Lazy import audit` — 18 import'ов внутри методов | low | tech-debt | lines 711, 829, 961, 1049, 1066, 1123, 1221, 2017, 2341, 2704, 2956, 3881 |

**Каждая карточка имеет**:
- root cause (с line-номерами и ссылками на issue/PR)
- acceptance checklist `[ ]`
- тесты, которые должны быть зелёными ДО merge
- e2e-сценарий (если затрагивает поведение)
- без «victory lap» — задача техническая, формат как у #1395/PR #1386.

---

## 7. Что я НЕ утверждаю как баг (hypothesis-bag)

Эти наблюдения могут оказаться ложными — нужны live-прогоны. Не оформляю
как `bug`, оформляю как `hypothesis` в карточке 1410:

- **startup_greeting_fired навсегда True** при runtime-переключении
  `startup_greeting_sec` — нужно проверить через runtime параметр.
- **destroy_node не отменяет greeting_timer** — нужно увидеть в логах при
  kill voice-assistant container.
- **Lazy-import `yaml` (line 1221) внутри метода** — что если его используют
  до того, как settings готовы? Проверить cold start.
- **`_run_turn` finally-блок + sync retry** — при исключении в sync retry
  (line 2878) parent finally уже выполнил `DIALOGUE_END` → retry в IDLE
  не дойдёт до LLM. **Это уже известный bug** (#1204), просто комментарий.

---

## 8. ADR-0021 (draft, отдельный файл)

`docs/adr/0021-dialogue-node-decomposition-discipline.md` — правило
**«один метод = одна ответственность»** для `DialogueNode`:

1. **CC budget**: ни один метод не должен превышать CC=15.
   При превышении — открыть worker-карточку «выделить X в core/».
2. **State SSoT**: новые счётчики/словари — через `Enum` или `frozenset`,
   не литералы. (`_LLM_SKIP_REASONS` уже готовится в PR #1395.)
3. **Lazy-import ceiling**: максимум 2 lazy-imports на файл (для optional
   зависимостей). Остальное — на верх модуля.
4. **Issue-link в комментарии** = обязателен для каждого `if`/`except`,
   который **фиксит регрессию**. Иначе — никакого `# FIX (live 06.08)` без
   issue-ссылки.
5. **Per-bag workflow**: каждый серьёзный баг = отдельная worker-карточка
   с явным `Closes #N` в PR description.

Это правило **не серебряная пуля** — оно кодифицирует то, что мы и так
делаем руками (см. §5). Цель — чтобы следующий воркер не сломал тот же
   паттерн, что сломал #1386/1389/1403.

---

## 9. Где смотреть ещё

- `src/rob_box_harness/core/dialog_core.py` — partner-класс для `_run_turn`;
  если его CC тоже > 15, это **системный bag-class**, а не локальный.
- `src/rob_box_voice/core/dialogue_guards.py` — уже вынесены BABBLE/PERF/MUSIC
  keywords. **Удачный прецедент**: значит, можно вынести music_guard и
  stt_gate по той же схеме.
- `src/rob_box_voice/core/dialogue_state_machine.py` — DSM уже отдельный
  класс (событие/состояние/enum). **Ещё одно подтверждение**: «тонкая» часть
  DialogCore уже в `harness`, а в `dialogue_node` осталось glue.

---

**TL;DR для товарища Шифу**: 7 issue-карточек готовы (тела в
`/tmp/kc_logs/issue_1405_*.md`), каждая — маленькая, локальная, с
acceptance + тесты + e2e. Никакого big-bang рефакторинга. Архитектура
обслуживает процесс, не наоборот. Команды `gh issue create` — ниже в
summary.