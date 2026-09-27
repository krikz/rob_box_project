# Issue #2346 — n308 Борис (ermil) не опознан в backlog (run 34522852773)

> Диагностический отчёт architect по issue #2346 (kanban t_b3f7100e).
> Дата разбора: 2026-09-14 (UTC+02:00).
> Источник: https://github.com/krikz/rob_box_project/actions/runs/34522852773
> Скачанные артефакты: `e2e-voice-logs-34522852773`, `e2e-voice-artifacts-34522852773`,
> `e2e-voice-recording-34522852773` (см. `/tmp/run34522852773/dl/`).

## TL;DR (версия для Шифу)

- **Issue тезис не подтвердился в той формулировке, в которой написан.** Это
  не «n308 один упал, n303 прошёл» — это **все три** зарегистрированных голоса
  в акте 3 не были атрибутированы в `🗒️ [backlog] accumulated (no_wake_word)`-логе:
  `n302` (anton=Саша) → MISS, `n303` (ermil=Борис) → MISS, `n308` (ermil=Борис) →
  MISS. Для незарегистрированных `n304` (zahar), `n305`/`n309` (filipp) →
  `speaker='незнакомец'` OK.
- **Гипотезы (1)-(3) из issue проверить без полных логов робота невозможно.**
  Шаг `Collect robot logs` в прогоне был `skipped` → диагностический лог
  `🔍 identify candidates: best=… score=… | second=… score=… | gap=…`
  (speaker_id_node.py L367-370) не выгружен. Это **отдельный блокер** —
  без него ни один голосовой сценарий с `_log_identify_candidates`
  постфактум не разбирается.
- **Вердикт по фиксу: на сегодня НЕ делать фикс в `speaker_id_node` или
  `dialogue_node` вслепую.** Сначала починить шаг `Collect robot logs`,
  повторить прогон один раз, получить score/gap, и только тогда выбирать
  между тремя гипотезами. Иначе мы будем ремонтировать код, не зная, что
  именно сломалось — то есть нарушим ADR-0018 («честный FAIL лучше красивого PASS»).
- **Что сделать сейчас (минимально, дешёво, обратимо):** добавить
  `acceptance`-лог-чек в сценарий act3, чтобы pattern failure давал
  понятный срез (был ли `speaker='Борис'`, или робот записал
  `speaker='незнакомец'`, или вообще не записал). Это позволит локализовать
  причину без ремонта рантайма.

---

## 1. Что я проверил (raw)

### 1.1 Шаги с pattern-чеком по backlog-атрибуции

Из `e2e_atomic_out.log` (полный лог прогона, 14564 байт, см.
`/tmp/run34522852773/dl/e2e-voice-artifacts-34522852773/e2e_atomic_out.log`):

| step                | voice | text (фрагмент)                   | pattern результат                                                |
|---------------------|-------|------------------------------------|------------------------------------------------------------------|
| n301_wake_open      | anton | «Робот, мы тут сейчас немного…»   | (нет backlog-pattern, wake-цикл)                                 |
| n302_bg_sasha       | anton | «Борис, я тебе точно говорю…»     | `PATTERN_MISS: \[backlog\] accumulated \(no_wake_word\).*speaker='Саш` |
| n303_bg_boris       | ermil | «Да брось ты, Саша, у тебя разводка…» | `PATTERN_MISS: …speaker='Борис`                                |
| n304_bg_grisha      | zahar | «Молодые люди, вы вообще спать…»  | `PATTERN_OK: …speaker='незнакомец'`                              |
| n305_bg_valera      | filipp| «Я вообще-то курьер…»             | `PATTERN_OK: …speaker='незнакомец'`                              |
| n306_who_was_talking| anton | «Робот, ты всё это слышал…»       | `PATTERN_OK: \[backlog\] flushed to LLM backlog_handled=true`     |
| n307_count_voices   | anton | «Робот, а сколько всего…»         | (нет pattern)                                                    |
| n308_bg_command_boris| ermil| «Слушай, включи ты уже…»          | `PATTERN_MISS: …speaker='Борис`                                  |
| n309_bg_command_valera| filipp| «Только не электронщину…»        | `PATTERN_OK: …speaker='незнакомец'`                              |
| n310_execute_backlog_lru| anton| «Робот, ну ты слышал…»         | `PATTERN_OK: \[backlog\] flushed …` + tool-call                  |
| n311_who_asked      | anton | «Робот, а кто именно…»            | `NO_ACCEPT` после 3 попыток (отдельная карточка, см. issue)      |
| n312_stop_music     | anton | «Робот, всё, спасибо, выключай…»  | acceptance PASS (tool-call `stop_music`)                         |
| n313_silence_restored| anton| «Робот, теперь тихо?»            | acceptance FAIL: `expected tool calls not invoked: ['get_music_state']` (отдельная карточка) |

**Ключевое наблюдение:** harness **не печатает** в `e2e_atomic_out.log` строку
`🗒️ [backlog] accumulated (no_wake_word) … speaker='…'` целиком — он печатает
**только свой pattern-запрос** (то, что ищет) и итоговый `PATTERN_OK`/`PATTERN_MISS`.
То есть сам факт, попала ли фраза в backlog, в harness-логе НЕ различим: и для
n303 (где pattern MISS), и для n305 (где pattern OK) написан `✅ backlog
accumulated (no_wake_word)`. Что различается — это только суффикс `speaker='Борис'`.

Из этого однозначно следует: **фраза в backlog попадает во всех 4 голосовых
шагах**, но в n302/n303/n308 в одной строке лога робота не оказалось
`speaker='Борис'`/`'Саша'`. То есть `is_known` для ermil/anton в этих шагах
было False, либо `sp_name` после `sanitize_speaker_name` стал пустым.

### 1.2 GATE-1 (агрегатный acceptance)

`/tmp/run34522852773/dl/e2e-voice-artifacts-34522852773/e2e_v2_34522852773/acceptance.json`:

```
gate: GATE-1
expected_tool_calls: [stop_music, get_music_state]
actual_tool_calls:    [stop_music]
missing:              [get_music_state]
soft_hints: ["…LLM сделал verbal-only answer…"]
pass: false
```

Это **отдельный от issue #2346** FAIL — n313 (`get_music_state` пропущен).
Помечен в issue явно как «отдельная карточка». Не часть нашей диагностики.

### 1.3 Сценарий

`/home/builder/rob_box_project/.worktrees/t_b3f7100e/.github/e2e/scenarios/night/night_marathon_act3_backlog_diarization_v1.json`:

- Для n302: `pattern = \\[backlog\\] accumulated \\(no_wake_word\\).*speaker='Саш`
- Для n303: `pattern = \\[backlog\\] accumulated \\(no_wake_word\\).*speaker='Борис`
- Для n308: `pattern = \\[backlog\\] accumulated \\(no_wake_word\\).*speaker='Борис`
- Для n304/n305/n309: `pattern = …speaker='незнакомец'`

Pattern — single-line regex с `.*`. `re.search` (как в harness'е) при логе,
где `speaker='незнакомец'` на той же строке — ок; если `speaker='Борис'`
на отдельной строке — fail (нет, по факту дизайн — одна f-string, см. код ниже).

### 1.4 Код лог-строки

`src/rob_box_voice/rob_box_voice/dialogue_node.py` L2185-2189:

```python
self.get_logger().info(
    f"🗒️ [backlog] accumulated (no_wake_word) "
    f"tag={speaker_tag!r} speaker={sp_name or 'незнакомец'!r} "
    f"text={text[:60]!r}"
)
```

где (L2178-2183):

```python
with self._speaker_lock:
    sp = dict(getattr(self, "_current_speaker", {}) or {})
sp_name = sanitize_speaker_name(sp.get("name")) if sp.get("is_known") else ""
accumulator.add(
    text,
    speaker_tag=speaker_tag,
    speaker_name=sp_name or None,
)
```

→ `sp_name` всегда либо **валидное имя**, либо `""` (и тогда печатается
`'незнакомец'`). Промежуточных вариантов формата нет. То есть pattern
`'speaker='Борис'` не сматчится, если в `_current_speaker` не было
`is_known=True` с именем "Борис".

### 1.5 Диагностический лог speaker_id_node

`src/rob_box_voice/rob_box_voice/speaker_id_node.py` L350-376:

```python
def _log_identify_candidates(self, embedding):
    candidates = self._db.identify_candidates(embedding, top_n=2)
    if not candidates:
        return
    best = candidates[0]
    if len(candidates) > 1:
        second = candidates[1]
        gap = best.confidence - second.confidence
        self.get_logger().info(
            f"🔍 identify candidates: best='{best.name}'({best.speaker_id[:8]}) "
            f"score={best.confidence:.3f} | second='{second.name}'"
            f"({second.speaker_id[:8]}) score={second.confidence:.3f} | gap={gap:.3f}"
        )
    …
```

Идентификатор `L367-370` — ровно то, что просит issue как raw-evidence.
**В артефактах этого лога нет.** Из 26 step-логов в
`/tmp/run34522852773/dl/e2e-voice-artifacts-34522852773/e2e_v2_34522852773/`
только `synth_*.log` (меты синтеза Yandex TTS, ~134 байта каждый) и
`e2e_atomic_out.log`. Cycle_log.txt содержит 4 строки — только финальный
шаг n313 (`📥 LLM INPUT: 'теперь тихо'`).

Из 32 шагов workflow в
`gh run view 34522852773 … --json jobs …` шаги 10-15 (включая `Collect robot
logs`) имели `conclusion: skipped` — это объясняет отсутствие логов робота.

### 1.6 Что не проверено (честно)

| Что нужно для верификации                           | Где взять                                      | Статус |
|------------------------------------------------------|------------------------------------------------|--------|
| `🔍 identify candidates` для n302/n303/n308         | robot-лог                                       | ❌ нет в артефактах (шаг skipped)             |
| `👤 Speaker: …` или `👤 Speaker: unknown`            | robot-лог                                       | ❌ нет                                                  |
| Содержимое `speakers.db` до/после прогона           | резервная копия робота                         | ❌ недоступно из этой сессии                            |
| Persistence speakers.db между act2 и act3           | оба прогона в одной сессии? или restart?        | ❓ нет прямого доказательства в выгрузке               |
| `accumulator.add(... speaker_name=…)` — что попало   | robot-лог (accumulator — приватный атрибут)    | ❌ не пишется никуда                                    |
| `sanitize_speaker_name` (вдруг портит «Борис»?)      | прочитать код                                  | ⏳ см. ниже                                              |

## 2. Закрытие пункта «sanitize_speaker_name» (без логов робота)

`src/rob_box_voice/rob_box_voice/core/dialogue_helpers.py:112-130`:

```python
def sanitize_speaker_name(name: Optional[str]) -> str:
    if name is None:
        return ""
    cleaned = str(name).strip()
    if cleaned.lower() in INVALID_SPEAKER_NAMES:
        return ""
    return cleaned
```

**`"Борис"` сюда попадает как есть** (не в `INVALID_SPEAKER_NAMES`, strip не меняет).
→ Гипотеза «sanitize портит имя Борис» — **отвергнута** по коду. Юнит-тест
`tests/unit/core/test_dialogue_helpers.py:158-171` это подтверждает (включая
явный кейс с кириллицей).

## 3. Уточнённый вердикт и follow-up

### 3.1 Что точно (по коду)

- `dialogue_node.py:2178-2183` — `sp_name` берётся **из `_current_speaker`** под
  `_speaker_lock`, через `sanitize_speaker_name`. Если `is_known=False` →
  `sp_name=""` → в backlog-логе печатается `speaker='незнакомец'` (формула
  `sp_name or 'незнакомец'`). Этот fallback ровно один и тот же для anton и ermil.
- `speaker_id_node.py:350-376` — диагностический лог `_log_identify_candidates`
  вызывается **до** `identify()` (L334). Это лучший сигнал о том, был ли
  score у ermil/anton на грани порога.

### 3.2 Что НЕ точно и почему

Из выгрузки **нельзя** различить два варианта MISS:
1. ermil вообще не был top-1 (`best.name == 'Борис'`, score < 0.75).
2. ermil был top-1 и даже прошёл порог, но `_current_speaker` был взят
   из **предыдущего** хода (`hyp 2` из issue).
3. ermil прошёл порог, но `sp.get('name') == 'Борис'`, тогда возможен баг
   в `speaker_id_node` (например, не выставляет `is_known=True`).

Все три варианта требуют лог робота за окно act3.

### 3.3 Минимальное предложение (создать follow-up issue)

Зафиксировать в сценарии act3 **две диагностические строки** в
`e2e_atomic_out.log`:
- `🔍 identify candidates` — должно совпадать с тем, что пишет
  `_log_identify_candidates` на роботе. Сейчас **не выгружается** из-за
  `skipped: Collect robot logs`.
- В шаге-сценарии `pattern` уточнить: если `is_known=False` на момент
  backlog-add, дополнительно требовать **отсутствие** в той же строке
  `speaker='незнакомец'` И **присутствие** `speaker='Борис'`. Сейчас
  шаблон ровно один — и когда робот в backlog кладёт `'незнакомец'`,
  harness считает это валидным fallback'ом (но в act3 `n304/n305/n309`
  тоже настроены на `'незнакомец'` — это сбивает с толку).

**Не правлю `speaker_id_node` / `dialogue_node` / сценарий вслепую** — это
нарушило бы ADR-0018.

### 3.4 Follow-up issue

Создан: https://github.com/krikz/rob_box_project/issues/2351
(см. `## handoff` ниже).