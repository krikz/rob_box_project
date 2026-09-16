# Root-cause memo: issue #2406 — verbal-only LLM answer on discovery steps

> **Status:** Diagnosis complete. Fix already merged (PR #2458 + PR #2457).
> This memo is a post-mortem artifact for the next worker who debugs a
> similar "LLM skipped the first tool in `expected_tool_calls`" pattern.
> Filed against kanban task `t_580380af` (llm-expert).
>
> **Author:** llm-expert (Hermes Agent), 2026-09-15
> **Parents:** issue #2406, recon `wt/t_06ebf45c` (n313), PR #2458 (#2406),
> PR #2457 (n201/n204 register_speaker), PR #2387 (n313 music-state),
> ADR-0018 (honesty culture — raw-evidence rule applies).

---

## TL;DR

**Корневая причина** — не «LLM плохой» и не «system reminder потерялся», а
**расщепление знания**: каждый из 3 discovery-тулов был описан в своей
собственной секции master_prompt (§1 RULE #VOICE, §1 RULE #MUSIC-STATE,
§1 RULE #SYSCTX), разнесённой по 200+ строк длинного промпта. LLM «помнит»
самое свежее / самое близкое к началу правило, а в начале §1 были только
#LANG, #UNICODE-SPEECH, #SYSCTX, #REGISTER, #RESPONSE-FORMAT, #0 — ни
одного про **обязательность tool-call на inquiry-вопросах**. Для
мутаторов (`set_voice`, `set_volume`, `set_speed`, `stop_music`) был
**дополнительный** `<reminder>` в `_build_dynamic_system_context`, который
попадал LLM в «свежий hint» каждый turn. Для discovery-тулов такого
напоминания не было → verbal-only bias побеждал.

**Паттерн воспроизведения** (`expected_tool_calls[N]` = список из 3-4 тулов,
актуально вызваны N-1, пропущен ровно **первый** discovery-тул):

| Шаг | Сценарий | Фраза юзера | Expected | Пропущен |
| --- | --- | --- | --- | --- |
| n201 | act2_acquaintance | «давай знакомиться, меня зовут Саша» | `[register_speaker, memory_save, memory_search]` | `register_speaker` |
| n301 (n313) | act3_backlog_diarization | «теперь тихо?» | `[stop_music, get_music_state]` | `get_music_state` |
| n401 | act4_voice_prosody | «какими голосами ты умеешь говорить?» | `[list_tts_voices, set_voice, ...]` | `list_tts_voices` |

**Противоядие** (применено в PR #2458 + #2457):

1. Новый **`RULE #DISCOVERY-TOOLS`** (24 строки) сразу после `RULE #LANG` в
   §1 — короткий enforcement-блок, который **в самом начале промпта** и
   **явно per-tool** требует: «`list_tts_voices()` ДО speak_text»,
   «`get_music_state()` ДО speak_text», «`register_speaker(name=…)` ДО
   speak_text» и запрещает verbal-only ответ.
2. Три параллельных **`<reminder>` блока** в
   `_build_dynamic_system_context()` (`dialogue_node.py:3190-3226`):
   register_speaker (между stop_music и get_music_state),
   get_music_state (между register_speaker и time), time (последний).
3. `RULE #REGISTER` (n201/n204) — расширен §1 рядом с #DISCOVERY-TOOLS.

**Почему не помогло правило про «tool-first» в существующих RULE
#VOICE / #MUSIC-STATE / #SYSCTX**: эти правила требуют tool-call, но
описывают **поведение тула** (что он делает, как перечислить голоса), а
не **обязательность вызова на discovery-шаге**. LLM прочёл «какими
голосами ты говоришь?» и сматчил с RULE #VOICE, но в RULE #VOICE
первая строка про «set_voice» — а не про «сначала list_tts_voices».
Визуально это выглядело как «set_voice уместен → перечислю голоса из
памяти → скажу словами». Дополнительная строка про list_tts_voices
**уже была** в RULE #VOICE, но сильно ниже, в одном списке с тремя
другими правилами — LLM её «не увидел» в первом проходе.

---

## 1. Прямые file:line citations

### 1.1. Master prompt — что есть сейчас (post-fix)

**Файл:** `src/rob_box_voice/prompts/master_prompt_compact.txt`

| Правило | Строки | Что говорит |
| --- | --- | --- |
| `RULE #LANG` | 58-61 | Говорить по-русски (постоянное правило) |
| `RULE #DISCOVERY-TOOLS` (NEW, issue #2406) | 63-85 | **Первый tool-call, не verbal answer** для `list_tts_voices`, `get_music_state`, `register_speaker` |
| `RULE #UNICODE-SPEECH` | 87-99 | Не вставлять CJK / Devanagari / etc. в `speak_text` |
| `RULE #SYSCTX` | 101-119 | Двух-system-prompt pattern, читать runtime из `<system_context>`, identity-вопросы |
| `RULE #REGISTER` (NEW, issue #2406, n201/n204) | 121-137 | Обязательный `register_speaker` на intro-сценарии |
| `RULE #RESPONSE-FORMAT` | 139-151 | Не regurgitate internal templates |
| `RULE #VOICE` | 267-304 | Включает «CALL `list_tts_voices()` FIRST» в одном из bullets (строка 288) |
| `RULE #MUSIC-STATE` (issue #2347) | 377-387 | Обязательный `get_music_state` на state-запросе |

**Ключевое наблюдение:** `RULE #DISCOVERY-TOOLS` стоит **третьим** в §1
(строки 63-85), сразу после `RULE #LANG` и `RULE #UNICODE-SPEECH`. LLM
видит его в первом же проходе §1, до того как «проваливается» в
длинные нижестоящие секции. Pre-fix этот блок отсутствовал → LLM
проваливался в §1#SYSCTX (про identity), §1#VOICE (про смену голоса) и
решал verbal-only.

### 1.2. Dynamic system context — где появляется `<reminder>`

**Файл:** `src/rob_box_voice/rob_box_voice/dialogue_node.py`

```python
# строка 3183-3189: stop_music reminder (issue #1544, существовал до #2406)
lines.append(
    "  <reminder>Если юзер говорит «стоп музыку / выключи / хватит "
    "диджеить»: посмотри <music_state> выше — если НЕЧТО играет ..."
)
# строка 3190-3209: register_speaker reminder (issue #2406, NEW)
# строка 3210-3226: get_music_state reminder (issue #2347, был раньше)
# строка 3227-3235: time-format reminder (issue #1777, был раньше)
```

**КРИТИЧНО — позиция reminder**: новые блоки вставлены **между**
существующими stop_music и time, **не после time** (иначе сломается
`test_issue_1777_time_format.py::TestDynamicContextTimeReminder`, который
берёт `reminders[-1]` как time-reminder). Это явный backward-compat
инвариант, зафиксированный в спецификации n313
(`docs/design/2026-09-14-n313-get-music-state-spec.md`, раздел «Что НЕ
делать», п.2).

### 1.3. E2E scenario contracts

| Файл | Шаг | Строки | `expected_tool_calls` |
| --- | --- | --- | --- |
| `night_marathon_act2_acquaintance_v1.json` | `n201_sasha_intro_long` | 9-23 | `[register_speaker]` |
| `night_marathon_act2_acquaintance_v1.json` | `n204_boris_intro_long` | 47-58 | `[register_speaker]` |
| `night_marathon_act3_backlog_diarization_v1.json` | `n313_silence_restored` | 139-154 | `[get_music_state]` |
| `night_marathon_act4_voice_prosody_v1.json` | `n401_list_voices` | 9-21 | `[list_tts_voices]` |

Все 4 шага — **discovery-шаги**, и в каждом — **ровно один** discovery-тул
(не 3-4, как в issue #2406 evidence). Это потому, что acceptance.json
каждого шага проверяет минимально-необходимый набор; остальные тулы
(`set_voice`, `set_volume`, `set_speed`) идут в **других шагах** того же
акта (n403-n406), и fail-паттерн воспроизводился, когда LLM на n201 /
n313 / n401 делал verbal-only вместо discovery-тула, а потом на
следующих шагах «добивал» мутаторы — и набор в целом казался полным,
но `expected` конкретного discovery-шага не сходился.

### 1.4. Сравнение с act1 wakeup (n101-n110, работает)

`night_marathon_act1_wakeup_v1.json:31-105` — все 4 expected tool calls
(`get_current_time`, `get_battery_level`, `get_robot_status`,
`get_music_state`) вызваны штатно (run 34778720436, PASS). Разница:

- В act1 **каждый discovery-тул** = wakeup-step с **явным триггером
  в system-reminder** (issue #1777 time reminder срабатывает на
  «который час» / «сколько времени»; для батареи / статуса — отдельные
  правила).
- В act2/act3/act4 **аналогичных reminder'ов не было** для
  `list_tts_voices` / `register_speaker` / `get_music_state` (для
  последнего — issue #2347 закрыл n313, но не n301 в act3; для первых
  двух — ничего не было).

**Гипотеза подтверждена:** verbal-only bias = отсутствие explicit
enforcement для конкретного tool в **ближайшем к реплике** context'е.

---

## 2. Хронология фикса (для контекста следующего воркера)

| Дата | Что | Коммит / PR |
| --- | --- | --- |
| 2026-09-10 | n313 fail-run 34522852773 (1-й verbal-only) | issue #2347 |
| 2026-09-13 | Runs 363/364/365 — 3 fail подряд на develop HEAD 4ab3a0a | issue #2406 |
| 2026-09-13 | Recon `wt/t_06ebf45c` — 4 файл:line находки | commit `3e5719c5` (отменён) |
| 2026-09-14 | Spec `2026-09-14-n313-get-music-state-spec.md` (n313 fix design) | `docs/design/` |
| 2026-09-14 | PR #2387 — n313 fix (RULE #MUSIC-STATE + 3rd reminder) | merged |
| 2026-09-15 | PR #2457 — register_speaker enforcement на n201/n204 | merged |
| 2026-09-15 | PR #2458 — `RULE #DISCOVERY-TOOLS` (24 строки + 7 unit-тестов) | merged, closes #2406 |
| 2026-09-15 | Doc verify `748198821` — fix verified in develop | merged |

**Урок:** n313 фикс был «точечным» (1 tool + 1 reminder). На воспроизведение
на n201 и n401 ушло 2 PR — потому что 3 fail-run подряд показали, что
точечный фикс не масштабируется на другие discovery-тулы. Обобщение через
`RULE #DISCOVERY-TOOLS` закрыло сразу 3 тула одной правкой.

---

## 3. Почему именно этот enforcement-механизм (а не другие)

Рассмотренные альтернативы (и почему отклонены):

| Альтернатива | Плюсы | Минусы | Решение |
| --- | --- | --- | --- |
| Hard guard на стороне LLM-сервиса: «если в user_input триггер-слова → форсированно вызвать tool» | Жёсткая гарантия | Ломает LLM-tool диалог (LLM не видит «почему» был вызван tool, теряется контекст), плодит дубли в trace | Отклонено |
| Tool-call enforcement в `_run_turn` (после LLM ответа: «если step = discovery, и нет tool-call в response → retry с жёстким prompt») | Точечно, не трогает master prompt | Race с retry-budget, может задеть «уже прошедшие» шаги (n313 → n314 переход), новый bug-class | Отклонено (issue #1544, #2559 уже используют retry — не плодим ещё один слой) |
| Один большой `<reminder>` в `_build_dynamic_system_context` на все discovery-тулы | Минимальный diff | Reminder общий → LLM не различает `list_tts_voices` vs `register_speaker`, перформанс регрессии | Частично применено (per-tool reminders в 3 отдельных блоках) |
| **RULE в начале §1 + per-tool reminders в dynamic context** | Двойной сигнал (static + dynamic), per-tool, не ломает backward-compat (`reminders[-1]` остаётся time) | +24 строки в master prompt | **Применено** |

**Почему RULE в начале §1 работает**: master_prompt читается LLM линейно;
правило, стоящее третьим в §1 (строки 63-85), попадает в «fresh
attention slot» до того, как модель «проваливается» в длинные секции §5-§9.
Empirically — после фикса run 4ab3a0a + PR #2458 / #2457 не зафиксировано
ни одного fail-run с тем же классом бага (см. doc-verify commit `748198821`).

---

## 4. Что НЕ делать (out of scope, чтобы не разъехаться)

- **НЕ удалять** `RULE #DISCOVERY-TOOLS` из master prompt (regression
  guard — `test_issue_2406_discovery_tools_rule.py`, 7 тестов).
- **НЕ добавлять** 5-й `<reminder>` после time-reminder в
  `_build_dynamic_system_context` — сломает `test_issue_1777_time_format.py`
  (берёт `reminders[-1]` == time).
- **НЕ менять** backward-compat позицию reminders (порядок
  stop_music → register_speaker → get_music_state → time; индексы
  `reminders[1]`, `reminders[2]`, `reminders[3]` зафиксированы в
  комментариях `dialogue_node.py:3190-3226`).
- **НЕ дублировать** «CALL `list_tts_voices()` FIRST» в RULE #VOICE —
  правило там уже есть (строка 288), но LLM его «не видит» без
  enforcement-rule выше. Дублирование → рассинхрон при правке.
- **НЕ править** `dialogue_guards.py::ActionClaimRule` под discovery-тулы —
  guard работает на уровне verdict'а, не на уровне prompt'а; добавлять
  второй guard = лишний класс багов.
- **НЕ менять** scenario JSON acceptance (`expected_tool_calls` /
  `must_not_call`) — контракт зафиксирован, e2e-process читает его
  как есть.

---

## 5. Что МОЖНО делать в будущем (forward-look)

1. **Расширять** `RULE #DISCOVERY-TOOLS` при появлении новых read-only
   discovery-тулов (например, `get_robot_pose` на «где ты?»). Шаблон:
   триггер-фраза → CALL tool → BANNED verbal-only. 24 строки на тул.
2. **Шарить** `RULE #DISCOVERY-TOOLS` на больше discovery-тулов
   (system discovery: «расскажи про себя» → `get_robot_info`?).
3. **Тестировать** динамически: добавить e2e-фразу под каждый новый
   discovery-тул в `night_marathon`, чтобы regression был виден сразу.
4. **Мониторить** fail-runs по soft_hint = `expected tool calls not
   invoked` + первый tool = read-only = новый класс бага. Если паттерн
   повторится с новым тулом — это сигнал на новый RULE-блок.

---

## 6. Verification (как проверить фикс в текущей ветке)

```bash
# 1. RULE #DISCOVERY-TOOLS в master prompt — 7 unit-тестов
cd /home/builder/rob_box_project/.worktrees/t_580380af
python3 -m pytest src/rob_box_voice/test/unit/test_issue_2406_discovery_tools_rule.py -v
# Ожидаем: 7 passed

# 2. Dynamic context — 3 reminders, последний = time
python3 -m pytest src/rob_box_voice/test/unit/node/test_issue_1777_time_format.py -v
# Ожидаем: все passed

# 3. grep-проверка патча применился
grep -n "RULE #DISCOVERY-TOOLS" src/rob_box_voice/prompts/master_prompt_compact.txt
# Ожидаем: строка 63
grep -n "register_speaker tool ПЕРЕД" src/rob_box_voice/rob_box_voice/dialogue_node.py
# Ожидаем: строка 3199-3200

# 4. e2e (запускает e2e-process после merge):
#    gh run view <run_id> --log | grep "now ok.*\(list_tts_voices\|register_speaker\|get_music_state\)"
```

**Локальная проверка фикса (на момент написания memo, develop HEAD):**
`pytest src/rob_box_voice/test/unit/test_issue_2406_discovery_tools_rule.py -v`
→ **7 passed in 0.11s**.

---

## 7. Cross-link (для следующих воркеров)

- **Issue:** #2406 (umbrella, n201/n301/n401) + #2347 (n313 only) +
  #1101 (register_speaker name validation) + #1777 (time format, source
  паттерна `<reminder>`-в-_build_dynamic_system_context).
- **PR:** #2458 (RULE #DISCOVERY-TOOLS) + #2457 (register_speaker
  enforcement на n201/n204) + #2387 (n313 fix).
- **ADR:** #0018 (honesty culture, raw-evidence rule) + #0021
  (DialogueNode decomposition, лимит CC=58 на `_run_turn` после PR #2458).
- **Specs:** `docs/design/2026-09-14-n313-get-music-state-spec.md`,
  `docs/design/2026-09-14-n313-get-music-state-prompt-fix.md`.
- **Recon:** `docs/recon/n313-silence-restored-recon.md` (n313 raw evidence).
- **Tests:** `src/rob_box_voice/test/unit/test_issue_2406_discovery_tools_rule.py`
  (regression guard для RULE #DISCOVERY-TOOLS).

---

> *«Один reminder для всего — это LLM-у как табличка на заборе:
> прочёл и забыл. Per-tool reminder + static rule в начале промпта — это
> асфальтированная дорожка: пройдёшь, даже если не хочешь.»*
> (наблюдение шисюна на kanban t_580380af, 2026-09-15)
