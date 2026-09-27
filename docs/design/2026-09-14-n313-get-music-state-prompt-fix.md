# Спецификация: принудительный вызов `get_music_state` на state-запросах (n313)

> Документ — дизайн-вход для implementer'а (см. дочернюю карточку
> `t_df05da52`, profile: `developer` или `ml-engineer`). Зафиксированные в
> нём формулировки — **контракт**: менять текст без согласования с
> шисюном / карточкой нельзя.
>
> **Контекст.** В CI-ране `34522852773` (e2e_atomic шаг n313) юзер говорит
> «Робот, теперь тихо?» сразу после успешного `n312_stop_music` (тот же
> ран, шаг ранее). LLM делает **verbal-only** ответ «Сейчас тишина»
> (GATE-1 подтверждено), пропуская `get_music_state`. Контракт шага
> требует `expected tool call(s) skipped: get_music_state`. Регрессия.
>
> Корень: LLM-у визуально кажется, что `<music_state>` в
> `<system_context>` — уже достаточный источник истины, и `get_music_state`
> tool «лишний». Это тот же verbal-only bias, что issue #1544 для
> `stop_music`, но в зеркальном направлении: там LLM игнорировал state и
> не звал `stop_music`, тут LLM доверяет state и не зовёт
> `get_music_state` для подтверждения.

| Поле         | Значение                                                          |
|--------------|-------------------------------------------------------------------|
| Статус       | Accepted (дизайн-фаза, реализация отложена)                       |
| Дата         | 2026-09-14                                                        |
| Автор        | llm-expert (Hermes Agent), kanban t_f853f52c                      |
| Родители     | issue #2347 (n313 silence_restored), recon `wt/t_06ebf45c` commit `3e5719c5` |
| Затрагивает  | `src/rob_box_voice/prompts/master_prompt_compact.txt` (RULE); `src/rob_box_voice/rob_box_voice/dialogue_node.py` (`_build_dynamic_system_context`); `src/rob_box_voice/prompts/skills/composer.txt` (опц.) |

---

## 1. Acceptance criteria (для implementer'а)

1. **RULE**: в `master_prompt_compact.txt` есть явная RULE про обязательный
   вызов `get_music_state` на state-запросах, с триггер-словами из n313
   («тихо?», «играет ли музыка?», «что играет?»).
2. **Reminder**: в `_build_dynamic_system_context()` появляется третий
   `<reminder>` блок, который перекрывает verbal-only bias.
3. **Composer (опц.)**: `composer.txt` имеет строку, требующую
   `get_music_state` перед ответом «тишина/играет».
4. **Backward compat**: существующие тесты
   `test_issue_1777_time_format.py::TestDynamicContextTimeReminder::*` и
   `test_issue_1544_*` остаются зелёными (напоминаю: тест берёт
   `reminders[-1]` как time-reminder — новый reminder должен быть
   вставлен **между** stop_music и time, не после time).
5. **e2e**: один голосовой шаг n313_silence_restored проходит в
   `e2e_atomic_out.log` без `expected tool calls not invoked:
   ['get_music_state']`.

---

## 2. Patch (3 hunks, copy-paste)

### Hunk 1 — `master_prompt_compact.txt`: расширение RULE #MUSIC

**Место:** после строки 286 (после строки «НЕ выдумывай «музыка уже
закончилась» — если тег `playing`, трек играет.»), перед `🚨 **RULE
#SEARCH`.

```diff
 🚨 **RULE #MUSIC — ОСТАНОВКА СГЕНЕРИРОВАННОГО ТРЕКА**:
 `<system_context>` содержит `<generated_music>`: `playing` или `idle`. Читай
 это состояние ТОЛЬКО из тега. Если `playing` и юзер просит «стоп/хватит/
 выключи/другую» — вызови `stop_music()`, затем подтверди через `speak_text`.
 НЕ выдумывай «музыка уже закончилась» — если тег `playing`, трек играет.
+
+🚨 **RULE #MUSIC-STATE — ОБЯЗАТЕЛЬНЫЙ `get_music_state` НА STATE-ЗАПРОСЕ**:
+Если юзер спрашивает про текущее состояние музыки («тихо?», «тишина?»,
+«играет ли музыка?», «что сейчас играет?», «что играет?», «музыка
+включена?», «есть звук?», «слышно что-нибудь?», «any music?») — ты ДОЛЖЕН
+СНАЧАЛА вызвать `get_music_state` tool, прочитать его ответ и ТОЛЬКО
+ПОТОМ отвечать через `speak_text`. ❌ НЕ отвечай «Сейчас тишина» /
+«Ничего не играет» / «Music is off» на основе только тега `<music_state>`
+— этот тег может быть stale (cleanup pending, batch ещё активен, DJ
+только что переключился), а e2e-гейт проверяет наличие tool call в
+трейсе. ✅ Если tool вернул `idle/silent` — подтверди словами; если
+`playing: <title>` — назови трек/тему и спроси, что сделать.

 🚨 **RULE #SEARCH — NO FAKE SEARCH / NO HALLUCINATED RESULTS**:
```

### Hunk 2 — `dialogue_node.py`: новый `<reminder>` в `_build_dynamic_system_context`

**Место:** после строки 3184 (после `</reminder>` для stop_music), перед
комментарием `# Issue #1777 — SYSTEM REMINDER: русский формат времени.`.
КРИТИЧНО: позиция — **между** stop_music и time, иначе сломается
`test_issue_1777_time_format.py:234` (берёт `reminders[-1]` как
time-reminder).

```diff
         lines.append(
             "  <reminder>Если юзер говорит «стоп музыку / выключи / хватит "
             "диджеить»: посмотри <music_state> выше — если НЕЧТО играет "
             "(dj_active или ai_active или beat_active), ОБЯЗАТЕЛЬНО вызови "
             "stop_music tool, а потом коротко подтверди; если ВСЁ stopped — "
             "verbal «уже выключено» без tool call.</reminder>"
         )
+        # Issue #2347 (n313 silence_restored) — SYSTEM REMINDER: на
+        # state-запрос LLM по умолчанию делает verbal-only ответ из
+        # <music_state> тега и пропускает get_music_state tool. e2e-гейт
+        # требует tool call в трейсе. Дублируем правило в dynamic
+        # context, чтобы LLM не «угадывал» ответ на основе stale snapshot.
+        # Ставим МЕЖДУ stop_music и time — test_issue_1777_time_format
+        # берёт reminders[-1] как time-reminder, не сдвигаем его.
+        lines.append(
+            "  <reminder>Если юзер спрашивает про состояние музыки "
+            "(«тихо?», «тишина?», «играет ли музыка?», «что играет?», "
+            "«что сейчас играет?», «музыка включена?», «слышно что-нибудь?»): "
+            "ОБЯЗАТЕЛЬНО вызови get_music_state tool ПЕРЕД ответом, "
+            "прочитай результат и только потом отвечай через speak_text. "
+            "НЕ угадывай ответ по <music_state> тегу — он может быть stale "
+            "(DJ переключился, beat ещё держится под TTS-батчем, cleanup "
+            "pending). Tool call обязателен даже если кажется, что и так "
+            "ясно.</reminder>"
+        )
         # Issue #1777 — SYSTEM REMINDER: русский формат времени. Tool
```

### Hunk 3 — `prompts/skills/composer.txt`: уточнение строки про `get_music_state`

**Место:** строки 12-13 — там уже есть упоминание `get_music_state`.
Добавим одно предложение про state-ответ (опционально, но согласуется с
RULE #MUSIC-STATE).

```diff
 - `get_music_state` — что играет прямо сейчас. Проверяй перед тем, как
-  запускать новое поверх.
+  запускать новое поверх. На state-вопрос юзера («тихо?», «играет ли
+  музыка?», «что играет?») вызывай `get_music_state` ОБЯЗАТЕЛЬНО и
+  отвечай по его результату — НЕ угадывай «тишина» по тегу.
 - `stop_music` — остановить. Если юзер говорит «стоп/хватит/выключи» и в
   `<music_state>` что-то активно — вызывай ОБЯЗАТЕЛЬНО, а не отвечай
   словами «уже выключено».
```

---

## 3. Rationale (почему именно так)

1. **Dual enforcement (RULE + reminder)**: статическая RULE в master prompt
   — это «долгосрочная память» LLM. Reminder в `<system_context>` —
   «свежий hint» каждый turn. Одного reminder'а недостаточно: n313 recon
   показал, что даже при наличии `<music_state>` тега LLM решает «и так
   ясно». Двойной сигнал закрывает оба пути (reasoning + attention).

2. **Явный список триггер-слов** («тихо?», «тишина?», «играет ли музыка?»
   и т.д.) — LLM склонен матчить «семантически близкие» формулировки;
   перечисление снижает false-negative на n313 и родственных
   формулировках («слышно что-нибудь?», «any music?»).

3. **Объяснение «почему tool, а не тег»** (stale snapshot, DJ switching,
   cleanup pending) — LLM имеет тенденцию «оптимизировать» очевидные
   действия; явный rationale «e2e-гейт требует tool call» +
   «тег может быть stale» снимает мотивацию срезать углы.

4. **Позиция reminder между stop_music и time** — единственный безопасный
   слот, не ломающий `test_issue_1777_time_format.py:234` (`reminders[-1]`
   == time-reminder). Альтернативы (перед stop_music или после time)
   требуют править существующие тесты — лишний diff, лишний риск.

5. **composer.txt** — скилл уже упоминает `get_music_state`, но только
   в контексте «перед запуском нового». Дополнение про state-вопрос
   делает поведение консистентным между RULE и активным skill'ом.

## 4. Что НЕ делать (out of scope)

- НЕ менять `<music_state>` snapshot-формат (он уже корректен для stop,
  не надо ломать issue #1544 fix).
- НЕ добавлять 4-й `<reminder>` после time — сломает
  `test_issue_1777_time_format.py`.
- НЕ редактировать `dialogue_guards.py` `ActionClaimRule` для
  `get_music_state` (issue #2347 уже закрывает эту проверку на уровне
  verdict'а; добавлять второй guard — лишнее).
- НЕ трогать composer.txt:18-19 «ГЛАВНОЕ ПРАВИЛО» — это про
  performance-запрос, не про state-ответ; confusion-минимизация.

## 5. Verification (как проверить)

```bash
# 1. Unit: dynamic context содержит 3 reminder блока и нужный текст
pytest src/rob_box_voice/test/unit/node/test_issue_1777_time_format.py -v
pytest src/rob_box_voice/test/unit/node/test_pure_methods.py -v -k music_state

# 2. Unit: RULE #MUSIC-STATE парсится в master_prompt_compact.txt
pytest src/rob_box_voice/test/unit/test_prompt_skill_sections.py -v

# 3. Manual grep — патч применился
grep -n "RULE #MUSIC-STATE" src/rob_box_voice/prompts/master_prompt_compact.txt
grep -n "get_music_state tool ПЕРЕД" src/rob_box_voice/rob_box_voice/dialogue_node.py
grep -n "На state-вопрос юзера" src/rob_box_voice/prompts/skills/composer.txt

# 4. e2e (запускает implementer уже в CI, но проверка до merge):
#    n313_silence_restored → "now ok: tool called get_music_state"
grep "n313" /tmp/rob_box_e2e_atomic.log  # или путь актуального harness
```