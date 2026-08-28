# ADR-0024 — music-aware intent priority gate: «стоп музыку» не должен идти в Nav2 даже при потерянном wake-слове

| Поле         | Значение                                                                                  |
|--------------|-------------------------------------------------------------------------------------------|
| Статус       | **proposed**                                                                              |
| Дата         | 2026-08-22                                                                                |
| Автор        | architect                                                                                 |
| Issue        | [#1525](https://github.com/krikz/rob_box_project/issues/1525) — dj02 wake-word под музыкой |
| Closes       | #1525                                                                                     |
| Связанные    | ADR-0018 (честный FAIL), ADR-0021 (dialogue_node декомпозиция, CC-budget ≤15), ADR-0013 (incremental delivery), issues #1279 (intent-gate), #992 (music_guard), #1392 (AI music) |
| Прогон       | e2e run 32573773556 / `dj02_stop_music` head `7b67ff0b`                                   |

---

## 1. Контекст и бизнес-проблема

**Симптом** (e2e-прогон `dj02_stop_music`, 22.08.2026 ~13:50 UTC): пользователь говорит
«Робот, стоп музыку» во время играющего Renardo-бита. Робот **не останавливает музыку**,
а вместо этого произносит «Останавливаюсь» и отменяет активный Nav2 goal. Музыка
продолжает играть.

### 1.1 Acceptance из issue #1525 (дословно)

- [ ] «Робот, стоп музыку» при играющей музыке → `stop_music` вызван, музыка
      остановлена, Nav2-цели НЕ трогаются.
- [ ] «стоп» (без «музык», контекст движения) → STOP-движения как раньше (регресс-проверка).
- [ ] unit/e2e-тест на приоритет `MUSIC_STOP_OVERRIDES` над STOP-интентом `command_parser`.

### 1.2 Корневая причина — два независимых дефекта

Из логов `docker logs voice-assistant` (run 32573773556, окно 13:50–14:06 UTC):

```
[stt_node-6]    [1787407505.837] ✅ ПРИНЯТО: стоп музыку          ← wake «робот» ПОТЕРЯН
[command_node-8][1787407505.841] 🎤 STT: стоп музыку
[command_node-8][1787407505.842] 🎯 Intent: stop (0.87)            ← command_parser матчит
[dialogue_node-4][1787407505.843] 🗒️ [backlog] accumulated (no_wake_word)  ← без «робот» → в backlog
[command_node-8][1787407505.844] 💬 Feedback: Останавливаюсь       ← Nav2 cancel вместо music
[command_node-8][1787407505.849] 🛑 Отменяю все Nav2 goals...
```

**Дефект A — command_parser context-blind.** Регекс `IntentType.STOP` в
`src/rob_box_voice/rob_box_voice/core/command_parser.py:119-122`:

```python
IntentType.STOP: [
    (r'(стой|стоп|остановись|останови|halt|стоять|хватит|замри)', None),
    ...
]
```

матчит слово «стоп» в любом предложении. Когда STT возвращает «стоп музыку»,
command_parser ставит `intent=STOP, confidence=0.87`. Затем `command_node.handle_stop`
буквально отменяет Nav2-цели (см. `command_node.py:258-300`). Семантика «это про
музыку» полностью потеряна.

**Дефект B — backlog не флашится без wake.** `dialogue_node._on_stt` (строки
1605-1630): «Universal wake-word gate — only direct address to robot can start or
interrupt a dialogue». Фраза без wake-слова копится в `SpeechAccumulator` и
**никогда не дойдёт до LLM**, пока не появится новое wake-слово
(см. `test_speech_backlog_accumulator.py::test_bare_wake_word_flushes_backlog` —
флаш ТОЛЬКО при wake). Когда STT не распознал «робот» из-за громкой музыки
(причина регрессии в репорте — «wake-слово потеряно на фоне громкой музыки»),
бэклог зависает навсегда.

### 1.3 Существующие части фикса (частично работают)

В коде **уже** есть два куска правды, которые изолированы и не сшиты:

1. **`MUSIC_STOP_OVERRIDES` в `dialogue_guards.py:161-171`** — корректный набор
   подстрок: «диджеить», «выключи музыку», «стоп музык», «останови музык», «убери
   музык». Используется в **двух** местах (dialogue_node:1655 + 1674), чтобы НЕ
   гейтить команду про музыку в command_node. **Не используется** в самом
   command_parser.

2. **`is_music_stop_command()` в `dialogue_guards.py:275-286`** — чистая функция,
   один источник истины для детекта «это про музыку, а не про движение».
   Используется только в dialogue_node. **Не используется** в command_parser и
   command_node.

3. **Существующий тест `test_music_stop_phrase_still_goes_to_llm`** (issue #1279)
   проверяет, что фраза «робот стоп музыку» (С wake-словом) идёт в LLM, а не
   гейтится как STOP. **Не покрывает** случай без wake-слова при активной музыке —
   а это и есть сценарий dj02.

### 1.4 Почему это важно архитектурно

Это не разовый «if-else» фикс. Это **класс багов**:
«голосовой интент интерпретируется буквально, без учёта контекстных маркеров
из parallel-stream (музыка, движение, диалог)». Если не зафиксировать архитектурно,
следующий регресс придёт через неделю: «робот, едь вперёд» под играющей музыкой
отменит `execute_music_code` (тот же путь в command_parser → command_node).

---

## 2. Решение

**Двухслойный music-aware intent priority gate.** Слой 1 — в `command_parser`
(context detection до классификации STOP). Слой 2 — в `dialogue_node` (music-aware
backlog flush, когда wake-слово потеряно из-за громкого аудио). Оба слоя
опираются на **один источник истины** — `dialogue_guards.MUSIC_STOP_OVERRIDES`
и `is_music_stop_command()` (уже существуют, жирно подчёркиваем R2 ADR-0021).

### 2.1 Слой 1 — context-aware STOP detection в `command_parser`

Добавить **приоритезированный проход** перед обычной классификацией:

```python
# В CommandParser.classify_intent() — порядок имеет значение
def classify_intent(self, text: str) -> Command:
    # (1) Music-stop override — ПЕРВЫЙ. Если фраза про музыку, она ВСЕГДА
    # должна дойти до LLM/dialogue_node, не зависимо от wake-гейта и
    # command_intent_gate. command_node её НЕ выполняет.
    if any(kw in text for kw in MUSIC_STOP_OVERRIDES):
        return Command(
            intent=IntentType.STOP,         # ← тот же STOP, но...
            text=text,
            entities={"music_stop": True},  # ← ...с контекстным флагом
            confidence=0.95,
        )
    # (2) Существующая логика — без изменений
    ...
```

Затем в `command_node.execute_command()`:

```python
def execute_command(self, command: Command) -> None:
    if command.intent == IntentType.STOP:
        if command.entities.get("music_stop"):
            # Пробрасываем как music-stop, command_node НЕ трогает Nav2.
            # intent уже опубликован в /voice/command/intent, dialogue_node
            # увидит его и через command_intent_gate пойдёт в LLM.
            self.publish_feedback("Останавливаю музыку")
            self.get_logger().info(
                "🎵 [issue 1525] music_stop detected — deferring to LLM, "
                "Nav2 goals NOT cancelled"
            )
            return
        self.handle_stop(command)  # Существующая логика — Nav2 cancel
```

**Почему так, а не через `IntentType.MUSIC_STOP`:** добавление нового IntentType
= новая ветка в каждом if-else (execute_command, _command_intent_gate, тесты).
Стоимость — много мест, риск регрессии — высокий. Альтернатива с entities
(= контекстный флаг) — это **минимально-инвазивное изменение**: одна новая
проверка в `execute_command`, остальной код не знает, что STOP бывает
«движение» и «музыка».

### 2.2 Слой 2 — music-aware backlog flush в `dialogue_node`

**Проблема**: если wake-слово потеряно в STT, фраза копится в backlog и
никогда не дойдёт до LLM (`test_speech_backlog_accumulator.py::test_bare_wake_word_flushes_backlog`
требует wake для флаша).

**Решение**: добавить в `dialogue_node._on_stt` **music-aware branch ДО**
текущего wake-gate (строки 1605-1630):

```python
# Issue #1525 — music-aware wake-bypass: если музыка/DJ активны И фраза
# содержит music-stop маркер — wake-word НЕ требуется. Без этого громкая
# музыка глушит «робот» в STT, фраза навсегда остаётся в backlog,
# музыка не останавливается.
if (
    getattr(self, "_accumulate_no_wake_enabled", False)
    and (self._dj.state.enabled or _is_generated_music_playing(self))
    and is_music_stop_command(text_lower)
):
    # Флашнем backlog немедленно с синтетическим wake-word, чтобы
    # downstream-логика (state machine, _dispatch_turn, hint injection)
    # работала штатно.
    text_with_wake = f"{self._wake_words[0]}, {text}"
    self.get_logger().info(
        f"🎵 [issue 1525] music-active wake-bypass: text={text[:60]!r}"
    )
    # Подменяем text и продолжаем обработку с wake-веткой
    text = text_with_wake
```

Условие `_is_generated_music_playing` — простая проверка
`getattr(self, '_generated_music_state', {}) or {}` → status=='playing'.
State уже публикуется mcp_server-ом (см. `_on_generated_music_state` в
dialogue_node.py:1826). DJ-состояние уже хранится в `self._dj.state.enabled`.

**Почему не убираем wake-gate целиком:** wake-gate защищает от **barge-in** —
фона, ТВ, эха TTS. Его смягчение только для music-stop + только при активной
музыке = минимальная регрессия. Цена ложного срабатывания низкая: если музыка
вдруг остановится по дороге, фраза пойдёт в LLM как «стоп» — а это уже текущее
поведение `command_intent_gate` (STOP без wake = гейтится как STOP = Nav2 cancel).
То есть worst-case = текущее поведение, не хуже.

### 2.3 Контракт тестов (ADR-0021 R3 — каждый баг = регрессия-тест)

1. `test_command_parser.py` — добавить `TestMusicStopPriority`:
   - `test_stop_music_classified_as_stop_with_music_flag` — «стоп музыку» →
     `intent=STOP, entities={'music_stop': True}`, **НЕ** `entities={}`.
   - `test_stop_alone_no_music_flag` — «стоп» → `entities={}` (регресс-проверка).
   - `test_dj_mode_classified_as_stop_with_music_flag` — «хватит диджеить» →
     `music_stop=True`.
   - `test_nav_command_unaffected` — «стоп» + нет маркера музыки →
     `music_stop is False`.

2. `test_command_node.py` (новый файл или расширение) — добавить `TestMusicStopBypass`:
   - мокнуть Nav2 cancel_client, выполнить «стоп музыку» через parser → убедиться,
     что `nav_cancel_callback` НЕ вызван, `intent_pub` опубликован с
     `stop:0.95`, `feedback_pub` содержит «Останавливаю музыку».

3. `test_speech_backlog_accumulator.py` — добавить `TestMusicActiveBypass`:
   - `_dj.state.enabled=True`, фраза «стоп музыку» (нет wake) → `_dispatch_turn`
     вызван, `_pending_backlog_flush=True`, в `dispatched` есть синтетический
     wake-word («робот, стоп музыку»).
   - `_dj.state.enabled=False`, та же фраза → `_dispatch_turn` НЕ вызван
     (регресс — wake-gate остаётся для обычных случаев).

4. `test_command_intent_gate.py` — расширить существующий тест
   `test_music_stop_phrase_still_goes_to_llm`, добавив параметризацию **без
   wake-word** (`"стоп музыку"` через `tg_chat_id=None`, `wake-words=[...]`,
   `_dj.state.enabled=True`) — должен идти в LLM.

### 2.4 Почему НЕ делаем

- ❌ **Перенос всего wake-gate в core/intent_router.** ADR-0021 R1 (CC-budget ≤15)
  и так под нагрузкой; новый слой = новый god-class. Локальная правка в
  dialogue_node + command_parser — 2 маленьких PR, не один большой (ADR-0013).
- ❌ **Music-detection в STT (акустический gate).** Это правильное **долгосрочное**
  решение (issue #992 retro), но: (а) требует retraining модели; (б) уже есть
  SpeechAccumulator + is_music_stop_command — текстовый gate дёшев и достаточен.
  Когда появится acoustic-gate — заменим **этот** слой, не весь подход.
- � **Удалить IntentType.STOP и заменить на STOP_MOVEMENT/STOP_MUSIC.** Даёт
  максимальную семантическую ясность, но = новая ветка в 8+ местах (тесты,
  command_node, dialogue_node, prompt, tool_registry). Неоправданный риск
  регрессии vs минимальный выигрыш (entities флаг = та же семантика для
  вызывающего).
- ❌ **Wake-word barge-in через VAD (music ducking).** Это инфра-уровень
  (audio_node + ReSpeaker DSP), относится к ADR-0013 (respeaker) и требует
  изменений в чужом контейнере. Выходит за scope issue #1525. Может быть
  отдельной карточкой позже.

---

## 3. Trade-off

| Аспект                  | Плюс                                                            | Минус / Риск                                                       |
|-------------------------|-----------------------------------------------------------------|--------------------------------------------------------------------|
| Минимальная инвазивность | +2 места в 2 файлах, +1 keyword import, +3 теста. CC-budget не нарушает. | — |
| Семантика               | entities={'music_stop': True} явно показывает intent, не скрытый | Если забудем проверить флаг в новом execute_command — регрессия   |
| Wake-bypass             | Решает реальный сценарий «робот + громкая музыка»               | Ложное срабатывание, если музыка остановилась в момент STT — текст пойдёт в LLM как «стоп» (то же поведение, что сейчас с command_intent_gate) |
| Тесты                   | Полное покрытие: parser → node → dialogue                       | +3 новых файла, требуют ревью (per ADR-0021 R3)                    |
| Откат                   | Простой: revertить 2 PR                                         | — |

**Главный риск:** если кто-то добавит новый intent-handler в command_node
(например, для нового `IntentType.MAP` или follow), он может забыть проверить
`entities['music_stop']`. Защита: добавить CC-budget проверку **в execute_command
через unit-test на handler matrix** (тест «каждый handler знает про music_stop
флаг»). Не делаем в этом PR — ADR-0021 R1 уже требует unit-tests на новый код.

---

## 4. План реализации (для backend-воркера)

**Два маленьких PR-а, не один большой** (ADR-0013).

### PR #1 — `fix(voice): command_parser + command_node — music-stop priority gate`
- Файл: `src/rob_box_voice/rob_box_voice/core/command_parser.py`
  - Добавить import `from rob_box_voice.core.dialogue_guards import MUSIC_STOP_OVERRIDES`
  - В `classify_intent()` — приоритезированный проход (entities={'music_stop': True})
- Файл: `src/rob_box_voice/rob_box_voice/command_node.py`
  - В `execute_command()` — if music_stop → return после `publish_feedback`
- Тесты: `test/unit/core/test_command_parser.py` (TestMusicStopPriority),
  `test/test_command_node.py` (TestMusicStopBypass) или новый файл
- WIP-коммит каждые 15-20 мин (per worker contract)
- base: develop

### PR #2 — `fix(dialogue): music-active wake-bypass для backlog flush (issue #1525)`
- Файл: `src/rob_box_voice/rob_box_voice/dialogue_node.py`
  - В `_on_stt` — music-aware ветка ДО wake-gate (строки 1605-1630)
  - Helper `_is_generated_music_playing()` рядом
- Тесты: расширить `test_speech_backlog_accumulator.py` (TestMusicActiveBypass),
  расширить `test_command_intent_gate.py::test_music_stop_phrase_still_goes_to_llm`
- base: develop

### PR #3 (опционально, ADR-0013 позволяет) — единый e2e регресс-тест
- Файл: `src/rob_box_voice/test/unit/test_issue_1525_music_stop_priority.py`
- Можно объединить с PR #1 или #2, если не превышает лимит 3000 строк.
- Если выделяем — отдельная карточка, не блокер для #1525.

### Acceptance для PR (закрывает issue #1525)

- [ ] Все три acceptance-критерия из issue выполнены (raw-вывод pytest + ручной
      e2e на VisionPi после merge).
- [ ] Существующий `test_music_stop_phrase_still_goes_to_llm` и
      `test_bare_wake_word_flushes_backlog` НЕ сломаны (регресс-проверка).
- [ ] `make test` (или `pytest src/rob_box_voice/test/unit/`) зелёный, raw-вывод
      приложен в PR description (ADR-0018).
- [ ] Локальный e2e на VisionPi (`dj02_stop_music` step): `stop_music` вызван,
      Nav2 НЕ отменён. docker-logs приложены.
- [ ] Голосовая команда для e2e-процесса: «Робот, стоп музыку» (файл
      `.github/e2e/voice_commands/rabot_stop_muzyku.ogg` уже есть в репо по
      данным e2e-процесса). Если нет — приложить в PR.

### Что НЕ делаем в этих PR

- Не трогаем `intent_gate.py` (issue #1279) — наш флаг уже исключён через
  `MUSIC_STOP_OVERRIDES` в dialogue_node.py:1674.
- Не трогаем `dialogue_guards.py` — он уже SSoT.
- Не трогаем `music_skill.py` / `mcp_server` — `stop_music` уже есть как tool.
- Не добавляем новый `IntentType.MUSIC_STOP` (см. секцию 2.4 «почему нет»).

---

## 5. Связь с существующими ADR

- **ADR-0018 (честный FAIL)** — реализация **обязана** приложить raw-вывод
  pytest и docker-logs VisionPi. «Проверил, работает» без raw = наказание.
- **ADR-0021 (dialogue_node декомпозиция)** — PR #2 в dialogue_node.py должен
  проверить CC-budget `_on_stt` (сейчас CC=28, лимит ≤15). Если CC вырастет
  после правки — вынести music-bypass в `core/speech_backlog.py` (уже ADR-0021
  R1 требует).
- **ADR-0013 (incremental delivery)** — два PR-а, не один большой. Каждый ≤50
  коммитов, ≤3000 строк.
- **ADR-0014 (issue→card→PR)** — обе карточки должны быть созданы **через
  триаж**, не руками (per memory: «НЕ создавать ручные kanban-карточки для
  воркеров»). Архитектор пишет только ADR + план, реализует backend-воркер.

---

## 6. Статус и критерий перевода в active

- [proposed → active] после того как:
  1. Backend-воркер реализовал оба PR-а
  2. e2e на VisionPi зелёный (`dj02_stop_music` PASS, raw-логи приложены)
  3. Товарищ Шифу одобрил merge обоих PR-ов
  4. issue #1525 закрыт через `Closes #1525` с ссылкой на PR

---

## 7. Appendix — raw-логи, использованные при анализе

Полный дамп логов за окно 13:50–14:06 UTC 22.08.2026:
`/tmp/e2e_artifacts_32573773556/` (workflow run, скачать по ссылке из issue).

Ключевые строки (8 штук) приведены в секции 1.2 — это минимальный набор,
по которому воспроизводится root cause в любом другом логе того же прогона.

Acceptance-сценарий:
`.github/e2e/scenarios/voice_core_suite_v1.json::dj02_stop_music` —
текст «Робот, стоп музыку», expected_tool_calls: ["stop_music"].

Acceptance-файл (GATE-1):
`.github/e2e/scenarios/voice_core_acceptance_v1.json` —
AND-семантика по `set_voice, execute_music_code, stop_music` за весь прогон.
