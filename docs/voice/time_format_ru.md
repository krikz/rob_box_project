# Формат русского ответа о времени (issue #1777)

> Контракт между `GetCurrentTimeTool` (MCP), LLM и TTS: что именно
> говорит робот, когда его спрашивают «который час», и какие правила
> зашиты в системный промпт.

| Поле | Значение |
|---|---|
| Статус | Accepted (issue #1777, формат-гайд внедрён) |
| Контекст | Kanban task `t_5a8379b7`, upstream `t_cd414361` (разведка) |
| Реализация | `src/rob_box_mcp_tools/.../tools/system.py` (`format_time_ru`, `GetCurrentTimeTool`) |
| Prompt | `src/rob_box_voice/prompts/master_prompt_compact.txt` §1 «RULE #TIME-FORMAT» |
| Dynamic context | `dialogue_node._build_dynamic_system_context()` — `<reminder>` про `get_current_time` |
| Тесты | `test/test_tools/test_get_current_time.py` (22 unit), `test/unit/node/test_issue_1777_time_format.py` (8 unit, 3 skipped на 3.11+) |

---

## 0. TL;DR

LLM раньше халлюцинировал «тридцать семь минут одиннадцатого» / «22:37
вечера» / «37 минут 11» — tool возвращал корректные данные, но модель
склеивала форматы как попало. Решение в два слоя:

1. **Tool contract**: `GetCurrentTimeTool.execute()` возвращает уже
   озвученную строку `formatted_time` («двадцать два тридцать семь»).
   LLM читает её дословно через `speak_text` — минимум свободы для
   галлюцинаций.
2. **Prompt guard**: в `master_prompt_compact.txt` появилась
   `RULE #TIME-FORMAT` (issue #1777) с явными ✅/❌ примерами. В
   `_build_dynamic_system_context()` на каждом ходу в `<reminder>`
   перечислены 5 фраз-триггеров — если LLM всё-таки захочет проигнорить
   tool, reminder его дожмёт.

---

## 1. Формат `formatted_time` (pure-функция `format_time_ru`)

`format_time_ru(dt: datetime, tz: tzinfo | None = None) -> str`

| HH:MM | Результат | Комментарий |
|---|---|---|
| 00:00 | «ноль часов ровно» | род «часов» (множественное) |
| 00:30 | «ноль тридцать» | без «ровно» — нецелый час |
| 01:00 | «один час ровно» | род «час» (именительный) |
| 02:00 | «два часа ровно» | род «часа» (родительный ед.) |
| 05:00 | «пять часов ровно» | род «часов» (род. мн.) |
| 12:00 | «двенадцать часов ровно» | НЕ «полдень» — period живёт отдельно |
| 13:00 | «тринадцать часов ровно» | 13..20 — род «часов» |
| 14:00 | «четырнадцать часов ровно» | принято в задаче как «два часа дня» — НЕ, формат-гайд требует цифровое |
| 21:00 | «двадцать один час ровно» | 21 = «час» (последняя цифра 1) |
| 22:00 | «двадцать два часа ровно» | 22..23 = «часа» |
| 22:37 | «двадцать два тридцать семь» | **тот самый bug-кейс** |
| 23:59 | «двадцать три пятьдесят девять» | граница дня |
| 07:05 | «семь пять» | без ведущего «ноль пять» |

### 1.1. Почему «ровно»

«Ровно» маркирует целый час, чтобы TTS не склеивал «два часа ровно» в
«два часа равно́» (TTS иногда нормализует «ровно» как предикат). Мы
сознательно оставляем — это устоявшийся шаблон для робота-ассистента.

### 1.2. Почему НЕ «полдень / полночь»

Карточка требует явного цифрового формата — period суток («утро»,
«день», «вечер», «ночь») живёт в отдельном поле `period` ответа tool и
в озвучке прибавляется по усмотрению LLM (`speak_text(f"Сейчас
{data['formatted_time']}, {data['period']}")`). Смешение «полдень» +
цифровое время запутывает LLM и TTS.

### 1.3. Почему НЕ «два часа дня»

Карточка явно требует **цифровое чтение**: «14:00 → «два часа дня»»
допустимо, но ведёт к двойной интерпретации («два часа дня» = 14:00 или
14:00-16:00?). Цифровое — однозначно, и LLM не выбирает между «вечера»
/ «дня» / «ночи» сам.

---

## 2. Формат `formatted_time` ответа tool

`GetCurrentTimeTool.execute()` возвращает `MCPToolResult.data`:

```python
{
    "time": "22:37",            # HH:MM (legacy, для отладки)
    "date": "31 августа 2026",  # legacy
    "weekday": "понедельник",   # legacy
    "period": "вечер",          # legacy (сутки)
    "iso": "2026-08-31T22:37:00+03:00",
    "formatted_time": "двадцать два тридцать семь",   # NEW (issue #1777)
}
```

`message` теперь дополнительно включает:

> «Сейчас 22:37, понедельник, 31 августа 2026, вечер. По-русски:
> двадцать два тридцать семь.»

LLM читает `data.formatted_time` и подставляет в `speak_text`.

---

## 3. Тайм-зона

* Дефолт: `Europe/Moscow` (робот-ассистент сейчас в Москве).
* Override: переменная окружения `ROBOT_TIMEZONE=<IANA name>`.
* Если `zoneinfo` недоступна / имя кривое — fallback на naive local
  `datetime.now()`. Голосовой пайплайн не валится.
* Если юзер в другой зоне — LLM должен сказать честно, что робот
  знает только московское время (RULE #TIME-FORMAT в промпте).

---

## 4. Правила для LLM (RULE #TIME-FORMAT в master_prompt_compact.txt)

Минимум:

* ✅ `speak_text(f"Сейчас {data.formatted_time}, {data.period}")`
* ✅ `speak_text("Сейчас двадцать два тридцать семь, вечер")`
* ❌ `speak_text("Тридцать семь минут одиннадцатого")` — выдумка
* ❌ `speak_text("22:37 вечера")` — смешение цифр и слова
* ❌ `speak_text("Двадцать два часа тридцать семь минут")` — раздуто

Полный блок — `RULE #TIME-FORMAT` в
`src/rob_box_voice/prompts/master_prompt_compact.txt` §1 (выше
`RULE #VOICE`).

---

## 5. Dynamic reminder

`_build_dynamic_system_context()` добавляет вторым `<reminder>` блок:

```
<reminder>Если юзер спрашивает «который час», «сколько времени»,
«время в Москве», «time?», «date today» — ОБЯЗАТЕЛЬНО вызови
get_current_time tool, прочитай поле formatted_time дословно и
озвучь его через speak_text. НЕ выдумывай время сам.</reminder>
```

Это fallback на случай, если LLM «забыл» RULE #TIME-FORMAT (длинный
промпт → первые правила могут выпасть из effective attention).

---

## 6. Тестовое покрытие

### Юнит (`src/rob_box_mcp_tools/test/test_tools/test_get_current_time.py`)
* `TestFormatTimeRu` — 18 граничных кейсов формата.
* `TestGetCurrentTimeTool` — 4 теста на tool contract:
  `formatted_time` присутствует, legacy поля сохранены, в message
  упомянуто, без арабских цифр.

### Регрессия / prompt (`src/rob_box_voice/test/unit/node/test_issue_1777_time_format.py`)
* `TestPromptTimeFormatRule` (4) — RULE #TIME-FORMAT в промпте,
  упоминания tool/examples/✅❌/строка в §4.
* `TestDynamicContextTimeReminder` (2, skip на 3.11+) —
  `<reminder>` блок присутствует, перечисляет 5 фраз.
* `TestPhraseTriggerContract` (2) — 5 триггеров покрыты RULE.

### E2E (отдельный процесс)
Голосовой e2e (`scripts/e2e/e2e_voice_test.sh`) — TODO по плану,
выполняется merge-gate → e2e-process после PR.

---

## 7. Вне scope (см. t_cd414361, не моего PR)

* `_resolve_timezone()` helper для системных тулзов — там в develop
  нет zoneinfo. Эта карточка использует свой локальный helper.
* `_apply_tool_skipped_guard()` для non-music tool-skip (issue #1762) —
  есть только в fix-ветке, не смёржено в develop. Если LLM иногда
  игнорирует tool полностью (не вызывает, а придумывает ответ) — это
  регрессия #1762, не #1777.
