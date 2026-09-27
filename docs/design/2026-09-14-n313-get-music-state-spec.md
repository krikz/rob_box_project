# Спецификация: `get_music_state` на state-запросах (карточка t_f853f52c)

> Документ — финальная спецификация для implementer'а (см. sibling-карточку
> `t_df05da52`, profile: `developer` или `ml-engineer`). Зафиксированные в
> нём формулировки и acceptance — **контракт**, менять текст без согласования
> с шисюном нельзя.
>
> **Контекст.** В CI-ране `34522852773` (e2e_atomic шаг n313) юзер говорит
> «Робот, теперь тихо?» сразу после успешного `n312_stop_music` (тот же ран,
> шаг ранее). LLM делает **verbal-only** ответ «Сейчас тишина» (GATE-1
> подтверждено), пропуская `get_music_state`. Контракт сценария
> `.github/e2e/scenarios/night/night_marathon_act3_backlog_diarization_v1.json:144-153`
> требует `expected_tool_calls: ["get_music_state"]` и `must_not_call:
> ["execute_music_code", "stop_music"]`. Регрессия.
>
> **Эта карточка (t_f853f52c)** — e2e-check фаза: довести промпт-фикс до
> зелёного e2e (см. parent recon `wt/t_06ebf45c`, commit `3e5719c5`, raw-evidence
> в `gh run view 34522852773 --log` lines 449-470).

| Поле         | Значение                                                          |
|--------------|-------------------------------------------------------------------|
| Статус       | Accepted (финальная спецификация)                                 |
| Дата         | 2026-09-14                                                        |
| Автор        | llm-expert (Hermes Agent), kanban t_f853f52c                      |
| Родители     | issue #2347 (n313 silence_restored), recon `wt/t_06ebf45c` commit `3e5719c5` |
| Затрагивает  | `src/rob_box_voice/prompts/master_prompt_compact.txt` (RULE); `src/rob_box_voice/rob_box_voice/dialogue_node.py` (`_build_dynamic_system_context`); `src/rob_box_voice/prompts/skills/composer.txt` (опц.) |
| E2E          | `.github/e2e/scenarios/night/night_marathon_act3_backlog_diarization_v1.json` (контракт уже зафиксирован) |

---

## 1. Acceptance criteria

1. **RULE**: в `master_prompt_compact.txt` есть RULE `#MUSIC-STATE`
   с триггер-словами из n313 («тихо?», «играет ли музыка?», «что играет?»).
2. **Reminder**: в `_build_dynamic_system_context()` появляется третий
   `<reminder>` блок, между stop_music и time.
3. **Composer (опц.)**: `composer.txt:12-13` дополнен — `get_music_state`
   обязателен и на state-вопросе.
4. **Backward compat** (existing tests stay green):
   - `test_issue_1777_time_format.py::TestDynamicContextTimeReminder::*`
     — берёт `reminders[-1]` как time-reminder; новый reminder идёт **ПЕРЕД**
     time, не после.
   - `test_issue_1544_*` — stop_music reminder не сдвинут.
5. **e2e n313**: один голосовой шаг `n313_silence_restored` проходит
   без `expected tool calls not invoked: ['get_music_state']` и без
   `must_not_call` нарушений.

---

## 2. Patch (3 hunks, copy-paste)

### Hunk 1 — `master_prompt_compact.txt`: расширение RULE #MUSIC

**Место:** после строки 286 (после строки «НЕ выдумывай «музыка уже
закончилась» — если тег `playing`, трек играет.»), перед `🚨 **RULE #SEARCH**`
на строке 288.

```diff
 🚨 **RULE #MUSIC — ОСТАНОВКА СГЕНЕРИРОВАННОГО ТРЕКА**:
 `<system_context>` содержит `<generated_music>`: `playing` или `idle`. Читай
 это состояние ТОЛЬКО из тега. Если `playing` и юзер просит «стоп/хватит/
 выключи/другую» — вызови `stop_music()`, затем подтверди через `speak_text`.
 НЕ выдумывай «музыка уже закончилась» — если тег `playing`, трек играет.
+
+🚨 **RULE #MUSIC-STATE — ОБЯЗАТЕЛЬНЫЙ `get_music_state` НА STATE-ЗАПРОСЕ**:
+Если юзер спрашивает про текущее состояние музыки («тихо?», «тишина?», «тише?»,
+«играет ли музыка?», «что сейчас играет?», «что играет?», «музыка включена?»,
+«есть звук?», «слышно что-нибудь?», «any music?») — ты ДОЛЖЕН сначала вызвать
+`get_music_state` tool, прочитать его ответ и ТОЛЬКО ПОТОМ отвечать через
+`speak_text`. ❌ НЕ отвечай «Сейчас тишина» / «Ничего не играет» / «Music is off»
+на основе только тега `<music_state>` — этот тег может быть stale (cleanup
+pending, batch ещё активен, DJ только что переключился), а e2e-гейт
+`n313_silence_restored` проверяет наличие tool call в трейсе. ✅ Если tool
+вернул `idle/silent` — подтверди словами; если `playing: <title>` — назови
+трек/тему и спроси, что сделать.

 🚨 **RULE #SEARCH — NO FAKE SEARCH / NO HALLUCINATED RESULTS**:
```

### Hunk 2 — `dialogue_node.py`: новый `<reminder>` в `_build_dynamic_system_context`

**Место:** после строки 3184 (`</reminder>` для stop_music), перед комментарием
`# Issue #1777 — SYSTEM REMINDER: русский формат времени.` на строке 3185.
**КРИТИЧНО:** позиция — **между** stop_music и time, иначе сломается
`test_issue_1777_time_format.py` (берёт `reminders[-1]` как time-reminder).

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
+        # n313_silence_restored требует tool call в трейсе. Дублируем правило
+        # в dynamic context, чтобы LLM не «угадывал» ответ на основе stale
+        # snapshot. Ставим МЕЖДУ stop_music и time — test_issue_1777_time_format
+        # берёт reminders[-1] как time-reminder, не сдвигаем его.
+        lines.append(
+            "  <reminder>Если юзер спрашивает про состояние музыки "
+            "(«тихо?», «тишина?», «тише?», «играет ли музыка?», «что играет?», "
+            "«что сейчас играет?», «музыка включена?», «слышно что-нибудь?»): "
+            "ОБЯЗАТЕЛЬНО вызови get_music_state tool ПЕРЕД ответом, прочитай "
+            "результат и только потом отвечай через speak_text. НЕ угадывай "
+            "ответ по <music_state> тегу — он может быть stale (DJ переключился, "
+            "beat ещё держится под TTS-батчем, cleanup pending). Tool call "
+            "обязателен даже если кажется, что и так ясно.</reminder>"
+        )
         # Issue #1777 — SYSTEM REMINDER: русский формат времени. Tool
```

### Hunk 3 — `prompts/skills/composer.txt`: уточнение строки про `get_music_state`

**Место:** строки 12-13 — там уже есть упоминание `get_music_state`. Дополним
одним предложением про state-вопрос (опционально, но согласуется с RULE).

```diff
 - `get_music_state` — что играет прямо сейчас. Проверяй перед тем, как
-  запускать новое поверх.
+  запускать новое поверх. На state-вопрос юзера («тихо?», «играет ли
+  музыка?», «что играет?») вызывай `get_music_state` ОБЯЗАТЕЛЬНО и отвечай
+  по его результату — НЕ угадывай «тишина» по тегу.
 - `stop_music` — остановить. Если юзер говорит «стоп/хватит/выключи» и в
   `<music_state>` что-то активно — вызывай ОБЯЗАТЕЛЬНО, а не отвечай
   словами «уже выключено».
```

---

## 3. E2E contract (что проверит e2e-process после merge)

**Сценарий:** `.github/e2e/scenarios/night/night_marathon_act3_backlog_diarization_v1.json`
(контракт зафиксирован в строках 140-154). Конкретно шаг `n313_silence_restored`:

```json
{
  "label": "n313_silence_restored",
  "voice": "anton",
  "text": "Робот, теперь тихо?",
  "patterns": [],
  "acceptance": {
    "expected_tool_calls": ["get_music_state"],
    "must_not_call": ["execute_music_code", "stop_music"],
    "_comment": "Возврат к якорю тишины из акта 1 — акт не оставляет хвостов."
  }
}
```

**Голосовой файл** для воспроизведения: на момент написания спека
`.github/e2e/voice_commands/rabot_teper_tiho.ogg` ещё не закоммичен.
Рекомендация implementer'у:
- Либо сгенерировать через Yandex TTS (anton) текст «Робот, теперь тихо?»
  и положить рядом с другими `rabot_*.ogg` (16kHz, opus, volume ≥150).
- Либо оставить на `ensure_voice_file` шаге e2e-process (он сам синтезирует).

При commit'е файла — проверить через `VOICE_COMMANDS_RESEARCH.md`:
длина синтеза ≤ 4с, wake word «Робот» в начале чётко.

**Pass-criteria (как e2e-процесс понимает, что фикс работает):**

```bash
# В логе e2e_atomic должны появиться оба маркера на шаге n313:
grep "now ok.*get_music_state" /tmp/e2e_atomic.log
grep -v "expected tool calls not invoked: \['get_music_state'\]" /tmp/e2e_atomic.log
# must_not_call — нет вызовов execute_music_code/stop_music
grep "n313_silence_restored" /tmp/e2e_atomic.log
```

---

## 4. Rationale (почему именно так)

1. **Dual enforcement (RULE + reminder)** — статическая RULE в master prompt
   это «долгосрочная память» LLM; reminder в `<system_context>` это «свежий
   hint» каждый turn. Одного reminder'а недостаточно: n313 recon показал,
   что даже при наличии `<music_state>` тега LLM решает «и так ясно».
   Двойной сигнал закрывает оба пути (reasoning + attention).
2. **Явный список триггер-слов** («тихо?», «тишина?», «играет ли музыка?» и
   т.д.) — LLM склонен матчить «семантически близкие» формулировки;
   перечисление снижает false-negative на n313 и родственных формулировках.
3. **Объяснение «почему tool, а не тег»** (stale snapshot, DJ switching,
   cleanup pending, e2e-гейт требует tool call) — LLM имеет тенденцию
   «оптимизировать» очевидные действия; явный rationale снимает мотивацию
   срезать углы.
4. **Позиция reminder между stop_music и time** — единственный безопасный
   слот, не ломающий `test_issue_1777_time_format.py` (`reminders[-1]` ==
   time-reminder). Альтернативы (перед stop_music или после time) требуют
   править существующие тесты — лишний diff, лишний риск.
5. **composer.txt** — скилл уже упоминает `get_music_state`, но только в
   контексте «перед запуском нового». Дополнение про state-вопрос делает
   поведение консистентным между RULE и активным skill'ом.

---

## 5. Что НЕ делать (out of scope)

- **НЕ менять** `<music_state>` snapshot-формат (`_build_music_state_snapshot`,
  dialogue_node.py:3015) — он корректен для stop, не надо ломать fix #1544.
- **НЕ добавлять** 4-й `<reminder>` после time — сломает
  `test_issue_1777_time_format.py`.
- **НЕ редактировать** `dialogue_guards.py::ActionClaimRule` для
  `get_music_state` — issue #2347 уже закрывает проверку на уровне verdict'а;
  добавлять второй guard — лишнее.
- **НЕ трогать** composer.txt:18-19 «ГЛАВНОЕ ПРАВИЛО» — это про
  performance-запрос, не про state-ответ; confusion-минимизация.
- **НЕ менять** scenario JSON (`.github/e2e/scenarios/night/night_marathon_act3_backlog_diarization_v1.json`)
  — контракт acceptance уже зафиксирован в строках 144-153.

---

## 6. Verification (как проверить implementer'у ДО отправки PR)

```bash
# 1. Unit: dynamic context содержит 3 reminder блока, последний = time
pytest src/rob_box_voice/test/unit/node/test_issue_1777_time_format.py -v
pytest src/rob_box_voice/test/unit/node/test_pure_methods.py -v -k music_state
# 2. Unit: RULE #MUSIC-STATE парсится в master_prompt_compact.txt
pytest src/rob_box_voice/test/unit/test_prompt_skill_sections.py -v
# 3. Manual grep — патч применился
grep -n "RULE #MUSIC-STATE" src/rob_box_voice/prompts/master_prompt_compact.txt
grep -n "get_music_state tool ПЕРЕД" src/rob_box_voice/rob_box_voice/dialogue_node.py
grep -n "На state-вопрос юзера" src/rob_box_voice/prompts/skills/composer.txt
# 4. Напрямую из Python: dynamic context должен содержать 3 reminder, последний = time
python3 -c "..." # см. test_issue_1777_time_format.py — там эталон
```

После PR — CI прогоняет сценарий night_marathon_act3. Зелёный acceptance
`n313_silence_restored` (= pass этой карточки).
