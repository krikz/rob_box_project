# Диагноз: persona/speaker-identification кластер #2809 — почему `must_not_say` ловит «Борис» в n210

**Дата:** 2026-09-23
**Автор:** architect (kanban t_f2e38cbc)
**Статус:** Диагноз, готов для worker (agent:backend, голос)
**Связано:** issue #2809, PR #2789, PR #2791, PR #2797, PR #2798, ADR-0127, ADR-0119, issue #2747

---

## TL;DR (verdict)

Гипотеза #1 в issue #2809 — **must_not_say не покрывает путь** — **неверна**. `must_not_say` работает корректно: поймал «Борис» в `robot_speech()`, цитата из лога run 35788126541:

```
>>> ACCEPTANCE[n210_grisha_no_name]: ❌ forbidden phrases spoken by robot: ['Борис']
```

Проблема не в checker. Проблема — **в LLM**: он **произносит** «Борис» в шаге n210_grisha_no_name. Это происходит из-за **двухступенчатой бреши**, не закрытой PR #2789 и смежными фиксами:

1. **speaker_id_node** false-positive: голос «Гриши» (e2e-синтез MiniMax-ом, не путать с голосом из живого теста) даёт **score=0.763** против Бориса в БД — выше `identify_threshold=0.72`. Узел публикует `is_known=True name='Борис'`.
2. **`_apply_speaker_identity`** (dialogue_node.py:2964) не имеет градации по confidence — `if sp.get("is_known"):` подставляет `[Spkr:Борис]` в user_input **и в `<system_context><user_profile>`** независимо от того, насколько слабая была биометрия.
3. **LLM** получает противоречивый сигнал: `<user_profile><name>Борис</name><voice_confidence>0.76</voice_confidence>` говорит «это Борис», но пользователь говорит «имя не назову». В шаге n210 модель выбирает system-context и отвечает «Как скажешь, Борис».

Существующие фиксы (`PR #2789`, `PR #2791`) закрывают соседние случаи:
- **#2789** — утечка имени через бэклог непройденной wake-гейт реплики, когда **биометрия говорит `Speaker: unknown`**.
- **#2791** — добавил `must_not_say` checker-уровневое поле.
- **#2797** — починил гейт длительности (`voiced_sec` вместо длины окна).
- **#2798** — переспрашивание при `voice_conflict` (другой человек с похожим голосом).

Ни один из них не закрывает кейс «**биометрия говорит known, но с низкой confidence**». Это и есть та флапающая брешь, которую мы видим в 3/3 последних ночных прогонах.

---

## 1. Доказательства (raw-evidence)

### 1.1 Лог робота (run 35788126541, шаг n210_grisha_no_name)

Полный лог вытащен через read-only `ssh ros2@10.1.1.21 "docker logs voice-assistant --since 2026-09-22T21:00:00Z --until 2026-09-22T23:00:00Z"`. Хронология:

| t (s) | Узел | Событие |
|---|---|---|
| 1790114500.659 | speaker_id_node | `👤 Speaker: 'Борис' confidence=0.763 (1174 ms)` |
| 1790114500.660 | speaker_id_node | `📢 Publishing: is_known=true name='Борис' epithet='Собеседник' conf=0.763` |
| 1790114500.664 | mcp_server | `👤 [issue 1770] current_speaker_id: ∅ → 0ddc1ab9-9af… (name='Борис')` |
| 1790114502.840 | stt_node | `✅ ПРИНЯТО (respeaker): Робот, я мимо шел. Имя свое я тебе называть не буду. Я старый закалки и недоверяком.` |
| 1790114503.166 | dialogue_node | `👤 [issue 1077] Speaker: 'Борис' conf=0.76` |
| 1790114503.175 | dialogue_node | `🚀 [turn] calling process_input: user_input='[Spkr:Борис] я мимо шел. Имя свое я тебе называть не буду. Я старый закалки и недоверяком.'` |
| 1790114503.* | dialogue_node | (LLM REQUEST START — в messages виден `<system_context><user_profile><name>Борис</name><voice_confidence>0.76</voice_confidence>...`) |
| 1790114508.082 | dialogue_node | `✅ [turn] process_input returned: spoken='Как скажешь, Борис. Я вас узнаю по голосу.'[:60] tools=[]` |
| 1790114508.086 | tts_node | `🔊 TTS: text='Как скажешь, Борис. Я вас узнаю по голосу.'` |

### 1.2 Промежуточные выводы из лога

- `identify()` сработал за 1174 мс, результат — `Борис conf=0.763`. Это **выше** `identify_threshold=0.72`, поэтому `is_known=True`.
- Между `identify` (4500.659) и STT-приёмом (4502.840) — зазор 2.2 секунды. `_apply_speaker_identity` (`dialogue_node.py:2990`) ждёт только 0.30 с (`await asyncio.sleep(0.30)`), после чего читает `self._current_speaker` — успевает увидеть `Борис conf=0.763`.
- В LLM-request виден **`<system_context><user_profile><name>Борис</name><voice_confidence>0.76</voice_confidence>`** (строка 1706 в логе). LLM получил имя **из двух мест** одновременно: `[Spkr:Борис]` в user_input и `<user_profile>` в system_context. Это удвоенное подтверждение «это Борис».
- В n210 user явно говорит «Имя свое я тебе называть не буду» — модель должна была отказаться. Не отказалась: «Как скажешь, **Борис**».

### 1.3 Флапающая природа

3 последовательных прогона develop:

| run | failed | steps failed |
|---|---|---|
| 35788126541 | 1/19 | n210_grisha_no_name |
| 35781881888 | 2/19 | n208_memory_search_tea, n210_grisha_no_name |
| 35775984954 | 3/19 | n204_boris_intro_long, n209_recall_boris, n211_who_do_you_know |

**n210_grisha_no_name** провалился во всех трёх прогонах. Остальные шаги флапают — т.е. проблема **не** в одном стабильном шаге, а в **самом механизме биометрии**, который даёт разные результаты от прогона к прогону из-за того, что голоса в e2e-синтезе (MiniMax TTS, разные голоса) иногда пересекаются с порогом 0.72.

---

## 2. Почему `must_not_say` не спасает

PR #2791 (от 22.09) добавил `must_not_say` поле в acceptance.json. Семантика:

```python
# .github/workflows/scripts/e2e_voice_test.sh:2096-2104
must_not_say = acc.get("must_not_say", []) or []  # НЕ должно ЗВУЧАТЬ
# ... scoped to robot_speech() — только spoken=/speak_text/TTS text
forbidden_said = [k for k in must_not_say if _keyword_hit(k)]
```

Это **правильный** дизайн: проверка scoped к тому же каналу, что `expected_keywords` (issue #2764). Тест на изоляцию — `tests/unit/e2e_scripts/test_issue_2779_must_not_say.py` — зелёный.

Но `must_not_say` — это **detection**, а не **prevention**. Checker умеет только сказать «ты сказал запрещённое». Он не умеет сказать «не говори». Поэтому каждый раз, когда LLM решает произнести «Борис», `must_not_say` срабатывает и шаг падает. Это и видим.

Для **стабильного прохода** нужно починить именно **путь, по которому LLM решает назвать имя** — а это архитектурная проблема в `speaker_id_node` + `_apply_speaker_identity`.

---

## 3. Архитектурный разбор — где именно брешь

### 3.1 Текущая модель (бинарная)

```
identify() → {is_known: bool, name?: str, confidence: float}
             │
             ▼
        _current_speaker
             │
             ▼
   _apply_speaker_identity()
             │
       ┌─────┴─────┐
   is_known=T   is_known=F
       │             │
  [Spkr:Имя]   [Speaker:unknown]
  + <user_profile> + (no profile)
    name=Имя
    conf=0.76
```

**Два состояния, без градации.** Confidence **записана**, но **не используется** для принятия решения «подставлять имя или нет».

### 3.2 Желаемая модель (трёхступенчатая)

```
identify() → {is_known, confidence}
             │
             ▼
   ┌─────────────────────────────────┐
   │  confidence ≥ CONFIDENT (0.85?) │  → is_known=T, [Spkr:Имя], <profile>name
   ├─────────────────────────────────┤
   │  identify_threshold ≤ conf      │  → is_known="tentative",
   │             < CONFIDENT         │    НЕ подставлять имя в user_input,
   │                                 │    НЕ подставлять <profile>name
   │                                 │    [Speaker:tentative] или [Speaker:unknown]
   │                                 │    + <user_profile> голос НЕ назван
   ├─────────────────────────────────┤
   │  conf < identify_threshold      │  → is_known=F, [Speaker:unknown]
   └─────────────────────────────────┘
```

«Tentative» — это **новый уровень** между «unknown» и «known». LLM получает signal «это может быть Борис (0.76), но я не уверен — обращайся нейтрально». В n210 модель бы ответила «Как скажешь» без имени.

### 3.3 Что нужно менять (минимально-инвазивно)

**Главный выбор — где хранить порог `CONFIDENT`**. Варианты:

#### Вариант A: жёсткий код-уровень (простота)

`dialogue_node.py:_apply_speaker_identity`:
```python
if sp.get("is_known"):
    conf = float(sp.get("confidence") or 0.0)
    if conf < CONFIDENT_THRESHOLD:    # новый параметр, default 0.85
        # Treat as tentative — don't leak the name into prompt
        user_input = f"[Speaker:tentative name≈{name} conf={conf:.2f}] {user_input}"
        # system_context: тоже — <user_profile><voice_confidence>...</voice_confidence>
        # НЕ name, а voice_match=Борис? (неопределённо)
        return user_input
    # existing confident path
```

**Плюсы:** один файл, 10 строк, быстрый merge.
**Минусы:** магическое число 0.85. Нужно его обосновать.

#### Вариант B: новый флаг в speaker_id_node (семантическая чистота)

`speaker_id_node.py` добавляет:
```python
if match and match.confidence >= CONFIDENT_THRESHOLD:
    payload = {"is_known": True, "name": ..., "confidence": ..., "recognition": "confident"}
elif match:    # weak match
    payload = {"is_known": False, "tentative_match": match.name, "tentative_confidence": ..., "recognition": "tentative"}
else:
    payload = {"is_known": False, "recognition": "unknown"}
```

`dialogue_node._apply_speaker_identity`:
```python
recognition = sp.get("recognition", "unknown")
if recognition == "confident":
    # existing path
elif recognition == "tentative":
    # log "weak match — would have been Борис at 0.76" but don't propagate
    user_input = f"[Speaker:tentative] {user_input}"
else:  # unknown
    # existing path
```

**Плюсы:** ясная семантика, `_apply_speaker_identity` не знает про магические числа.
**Минусы:** новый поле `recognition` в JSON-контракте топика — это breaking change для всех подписчиков (`mcp_server`, любые debug-инструменты). Нужен план миграции.

#### Вариант C: hybrid — порог в speaker_id_node, но без нового поля

`speaker_id_node` оставляет `is_known` boolean, но начинает **подавлять `name`** при low-confidence. Т.е. при conf=0.76 публикует:
```python
{"is_known": True, "speaker_id": ..., "name": None, "confidence": 0.763, "suppressed": "low_confidence"}
```

Тогда `_apply_speaker_identity`:
```python
if sp.get("is_known") and sp.get("name"):
    # existing confident path
elif sp.get("is_known") and not sp.get("name"):
    # low-confidence known: log, don't address by name
    user_input = f"[Speaker:tentative conf={sp.get('confidence'):.2f}] {user_input}"
else:
    # unknown
```

**Плюсы:** нет новых полей, нет breaking change. Семантика «нет имени = нет имени» уже понятна подписчикам.
**Минусы:** теряем информацию о том, **кто** был tentative match (для дебага). Можно вернуть её через `tentative_name` поле опционально.

### 3.4 Рекомендация

**Вариант C** — минимально-инвазивный и не плодит новых полей в JSON-контракте. Семантика «нет имени в payload = нет имени в речи» уже понятна всем подписчикам. Дебаг-информация (кто был tentative) доступна в логе `speaker_id_node` (там и так печатается `👤 Speaker: 'Борис' confidence=0.763 (1174 ms)`).

**Значение `CONFIDENT_THRESHOLD`** — отдельный вопрос. Эмпирика из PR #2797:
- Внутри профиля 1ae4b0ac: 0.653 … 0.872
- Внутри профиля c9e981cb: 0.888 … 0.908
- Между профилями: 0.417 … 0.585

**0.85 — разумный стартовый порог**: отсекает «качели» (0.908 / 0.550 / 0.820 / 0.556) внутри профиля от стабильных match-ей (0.888+). Но нужно мерить на реальных прогонах. ADR-0127 явно говорит: «по одному только голосу отличить того же от другого невозможно — обе картины выглядят одинаково». Поэтому **0.85 — это не решение, а компромисс**: меньше false-positive, но больше false-negative (Бориса перестанут узнавать в шумной обстановке).

Лучший долгосрочный путь — **voice_conflict** в стиле PR #2798: «уверен, что похож на Бориса, но Борис только что был здесь — может, тот же?». Это потребует session-aware логики, что шире текущей задачи.

---

## 4. Что нужно от воркера (backend-голос)

### 4.1 Минимально (для стабильного e2e)

1. Добавить параметр `confident_identify_threshold` (default 0.85) в `speaker_id_node` рядом с `identify_threshold` (0.72).
2. В `speaker_id_node._publish_speaker_result()` (около строки 1416) или ранее, где формируется payload:
   ```python
   if match.confidence < confident_identify_threshold:
       payload["name"] = None   # suppress name in payload
       payload["suppressed_reason"] = "low_confidence"
       # Но is_known и speaker_id оставляем для трейсинга и логов
   ```
3. В `dialogue_node._apply_speaker_identity()` (строка ~2990):
   ```python
   if sp.get("is_known") and sp.get("name"):
       # existing path (confident known speaker)
   elif sp.get("is_known") and not sp.get("name"):
       # low-confidence known — log but don't address by name
       self.get_logger().info(f"👤 [low_conf] tentative speaker_id={sp.get('speaker_id')[:8]} conf={sp.get('confidence'):.2f}")
       if speaker_context is None:
           user_input = f"[Speaker:tentative] {user_input}"
   else:
       # unknown
   ```
4. Регресс-тесты:
   - `tests/unit/voice/test_low_confidence_speaker_does_not_leak_name.py` — мок `_current_speaker` с `name=None, is_known=True, confidence=0.76`, проверить что в user_input нет «Борис» и в `<system_context><user_profile>` нет `<name>`.
5. Прогон e2e: должен пройти n210_grisha_no_name на develop.

### 4.2 По желанию (расширение)

- Параметр конфигурируется через `ros2 param set /voice/speaker_id confident_identify_threshold 0.85` — оперативная калибровка на живых данных.
- В `scripts/maintenance/voice_threshold_sweep.py` (уже существует после PR #2797) добавить сценарий «confident» vs «tentative» для настройки порога.
- Логировать в `mcp_server` строки вида `[issue 2809] tentative match suppressed: Борис@0.763 (would have leaked name)` для observability.

### 4.3 НЕ нужно делать

- НЕ менять `must_not_say` checker — он работает правильно.
- НЕ менять e2e_voice_test.sh — там `n210` acceptance уже настроен (`must_not_say: ["Борис", ...]`).
- НЕ трогать `SpeechAccumulator`/`format_user_hint` — PR #2789 уже закрыл эту сторону.
- НЕ переписывать `identify_threshold=0.72` — это другой порог (для решения «кто это»), трогать его опасно.

---

## 5. Связанные места в коде (file:line)

- `src/rob_box_voice/rob_box_voice/speaker_id_node.py:110` — `declare_parameter("identify_threshold", 0.72)`.
- `src/rob_box_voice/rob_box_voice/speaker_id_node.py:1416` — формирование payload с `is_known=True name=match.name` (точку модификации для варианта C).
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:2989-3033` — `_apply_speaker_identity` (точка модификации).
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:579-583` — `_current_speaker = {"is_known": False}` default.
- `tests/unit/e2e_scripts/test_issue_2779_must_not_say.py` — регресс-тест на checker (зелёный, не трогать).

---

## 6. Файлы, на которые НЕ нужно трогать (для worker)

- `docs/adr/0127-speaker-register-name-conflict.md` — прецедент false-positive, не менять.
- `docs/adr/0119-encounter-seam-current-speaker-id-sso.md` — SSO спикера, не менять.
- `.github/workflows/scripts/e2e_voice_test.sh` — `must_not_say` уже работает (issue #2791).
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:_prepare_user_input_context` — PR #2789 уже починил бэклог-утечку.

---

## 7. Acceptance для worker-карточки (когда будет создана)

- [ ] Параметр `confident_identify_threshold` в `speaker_id_node` (default 0.85).
- [ ] В `speaker_id_node` при conf < порога `name` подавляется в payload (вариант C).
- [ ] В `_apply_speaker_identity` ветка «is_known=True, name=None» → `[Speaker:tentative]`, **НЕ** `[Spkr:Борис]`.
- [ ] Unit-тест: мок `name=None, is_known=True, confidence=0.76` → в `user_input` и `<system_context>` нет имени.
- [ ] E2E прогон `night_marathon_act2_acquaintance_acceptance_v1` на develop → n210_grisha_no_name PASS, остальные шаги не сломались.
- [ ] CI зелёный на момент close.
- [ ] PR открыт, `Closes #2809` в description.
- [ ] Решение ждёт мержа от товарища Шифу.

---

> «Честный FAIL лучше красивого PASS» (ADR-0018). Этот документ — диагноз, а не «я починил». Реализация — за worker-карточкой.
