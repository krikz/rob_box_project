# Voice Backlog Accumulator Design

Goal: не терять речь, которую робот слышит без обращённого wake-слова. Вместо дропа — копить её в аккумулятор со скользящим окном и при следующем wake-word отдавать накопленное в LLM с разметкой «кто / когда / что сказал», чтобы LLM могла восстановить запрос, который плохо распознался с первого раза.

## Текущее поведение

В `dialogue_node._on_stt` универсальный wake-word gate: фраза без wake-слова отбрасывается (счётчик `no_wake_word`), робот продолжает ждать. Wake-word должен быть в той же фразе, что и команда. Проблема: долгий запрос + плохое распознавание wake-слова → всё теряется.

## Новое поведение

- Фраза без wake-слова (во всех состояниях, кроме SILENCED) кладётся в аккумулятор: `ts`, `text`, `speaker_tag` (Yandex `/voice/stt/speaker`), `speaker_name` (биометрия `_current_speaker`: имя если `is_known`, иначе `"незнакомец"`).
- При следующем wake-word накопленное (в пределах окна) сливается в LLM **отдельным блоком в `<system_context>` XML** (вариант Б) и аккумулятор очищается.
- Голое wake-слово («робот» без команды) при непустом аккумуляторе тоже сливает бэклог.

## Конфиг (`dialogue_node.yaml` + `_declare_params`)

| Параметр | Дефолт | Смысл |
|---|---|---|
| `accumulate_no_wake_enabled` | `true` | вкл/выкл фичи |
| `accumulate_window_sec` | `180.0` | скользящее окно, сек |

Внутренний лимит — до 30 записей в аккумуляторе (защита памяти).

## Структура данных

Новый чистый класс `SpeechAccumulator` в `src/rob_box_voice/rob_box_voice/core/speech_accumulator.py`:

- `add(text, speaker_tag, speaker_name)` — добавляет запись с `time.time()`.
- `prune(now, window_sec)` — выкидывает записи старше окна.
- `format_block(now)` — возвращает XML-блок `<speech_backlog>` с инструкцией и строками вида `[tag=0, Антон, ~12с назад] «текст»`, либо `None`, если пусто.
- `clear()` — очистка.

Чистый модуль без I/O/ROS2 → дёшево тестируется unit-тестами.

## Поток в `_on_stt`

1. Ветка `no_wake_word`: если фича включена и состояние ≠ SILENCED — `self._speech_accumulator.add(...)` + `prune` + лог, вместо чистого дропа. Если фича выключена — прежний дроп.
2. Wake-word найден, `clean` после `strip_wake_word`:
   - если `clean` пуст и аккумулятор непуст → пометить `self._pending_backlog_flush = True`, продолжить (бэклог уйдёт без «текущей фразы»);
   - если `clean` пуст и аккумулятор пуст → прежний `empty_after_strip` дроп;
   - если `clean` непуст → пометить `self._pending_backlog_flush = True`, текущая фраза идёт как обычно.
3. `_build_dynamic_system_context()` читает `self._pending_backlog_flush`, при True вставляет `self._speech_accumulator.format_block(now)` в `<system_context>` и сбрасывает флаг + очищает аккумулятор.

`raw_user_command` остаётся только текущей фразой — музыка/babble/command-гарды не видят бэклог.

## Формат блока

```xml
<speech_backlog>
  <instruction>Ниже — фразы, услышанные без обращённого wake-слова до текущего «робот». Пользователь мог иметь в виду одну из них как запрос. Определи, о чём просили, и ответь по существу, не пересказывая эти фразы дословно.</instruction>
  <entry speaker_tag="0" speaker="Антон" ago_s="12">«...»</entry>
  <entry speaker_tag="1" speaker="незнакомец" ago_s="45">«...»</entry>
</speech_backlog>
```

## Граничные случаи

- SILENCED — не копим (ветка возвращается до wake-gate).
- Telegram `[TG:...]` и DJ-auto — идут другими путями, не трогаем.
- Пустой аккумулятор при wake → блок не добавляется, поведение не меняется.
- Фразы-«помолчи» без wake — попадают в бэклог как обычная речь, не исполняются.
- Бэклог не проходит wake-классификацию повторно: `preclassified_event=STT_RESULT` уже передан в `process_input`.

## Тестирование

- Unit: `SpeechAccumulator` — add/prune/format/clear, окно, лимит, «незнакомец», формат времени.
- Node (`_on_stt`): no-wake копится; wake+бэклог → в `<system_context>` появляется `<speech_backlog>`; голое «робот» сливает бэклог; после слива аккумулятор пуст; SILENCED не копит; фича off = старый дроп.
