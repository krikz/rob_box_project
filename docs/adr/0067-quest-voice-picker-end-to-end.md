# ADR-0067 — quest voice picker: end-to-end gap analysis (issue #2138)

- Status: proposed (разбор инцидента; фикс — отдельные карточки)
- Date: 2026-09-08
- Deciders: architect (расследование), backend + frontend воркеры (фикс)
- Source: issue krikz/rob_box_project#2138
- Related: ADR-0028 (avatar supervisor), ADR-0066 (dialogue control pause/resume),
  ADR-0018 (agent honesty culture — capability-honest UI),
  PR #2058 (ADR-0055 deliver_audio — обратный канал ТАРС в шлем),
  commits 3c706ade9 (раньше «голос применялся»), 7022e89a3 (#2099 — NameError защищённый импорт).

---

## 1. Контекст и проблема

Оператор из шлема открывает TTS picker, выбирает голос, нажимает APPLY — UI возвращается в
прежнее состояние без видимой реакции. За 20 минут живой сессии:

- `docker logs avatar-supervisor | grep -i SetVoice` → **0 строк** (ни ACK, ни
  warning "missing voice_id", ни deprecated-reason).
- При этом `SetVoiceMode` и `SetVoicePreset` (AV-28) логируются: 4 строки.
- `ros2 topic info /avatar/set_voice` → pub=1 / sub=1 (провод жив).

Карточка issue #2138 утверждает, что клиент «осознанно шлёт пустой voice_id» и
винит `webxr_client/src/main.ts:279-293` (`sendStyleChange` шлёт `voice_id: ""`).
При поверхностном чтении это похоже на правду — но проверка показывает, что
picker использует **другую** точку отправки (`apply` → `cmd:"set_voice", voice_id`)
и НЕ ту функцию, на которую показывает issue. Расхождение — первый архитектурный
артефакт, требующий фиксации.

---

## 2. Контракт: две фичи, один cmd (`set_voice`)

Один и тот же `JSON_CMD{cmd:"set_voice"}` обслуживает **две независимые** фичи:

| Фича | Семантика | Поля | Поток |
| --- | --- | --- | --- |
| AV-27 (TTS picker, issue #1919) | сменить конкретный голос TTS у активного провайдера | `voice_id` (обяз.), `preset` (опц.) | quest-сервер валидирует → `/avatar/set_voice` → супервизор `SetParameters` на tts_node |
| AV-28 §P7 (issue #1920) | сменить **стиль речи** или **язык вывода** dialogue_node | `preset` ∈ `VOICE_PRESET_IDS`, `language` ∈ `VOICE_LANGUAGES` | супервизор `SetParameters` на dialogue_node |

Развилка в `ws_server._on_json_cmd` (`src/rob_box_quest/rob_box_quest/server/ws_server.py:2035-2039`):

```python
is_av28_request = (
    av28_preset is not None
    or av28_language is not None
    or bool(style_without_voice)
)
```

Picker (`webxr_client/src/main.ts:832-851`) шлёт `{cmd:"set_voice", voice_id}` БЕЗ
`preset` и БЕЗ `language` → `is_av28_request == False` → попадает в AV-27-ветку
(`ws_server.py:2100+`). Клиентский код для picker'а **уже корректен** — это
не та же функция, на которую показывает issue.

`sendStyleChange` (`main.ts:286-297`) — отдельный путь для AV-28-чипов
«технический/ленин/…» и языка. Шлёт `voice_id: ""` **намеренно** (комментарий
в коде это явно объясняет). Это НЕ тот код, который используется для picker'а.

---

## 3. Что код говорит vs что говорит runtime

### 3.1 Код (статический анализ)

| Шаг | Файл:строка | Что делает |
| --- | --- | --- |
| 1. Picker apply → cmd | `main.ts:841-845` | шлёт `{cmd:"set_voice", voice_id}` без preset |
| 2. WS-server dispatch | `ws_server.py:1989-2155` | AV-27-ветка → `bridge.set_voice(voice_id, preset=None)` |
| 3. Bridge validate | `quest_node.py:694-739` | проверяет `_active_provider` и `voice_id ∈ voices_for(provider)` |
| 4. Publish ROS | `quest_node.py:733` | `String` JSON в `/avatar/set_voice` |
| 5. Supervisor | `supervisor_node.py:1509-1541` | `_on_set_voice` → лог `SetVoice: voice_id=… applied=… reason=…` |
| 6. Apply params | `supervisor_node.py:1543+` | `SetParameters` на tts_node (через param_client) |

Все 6 шагов выглядят корректно. Контракт соблюдён.

### 3.2 Runtime (доказательство из issue)

- `SetVoice:*` — **0 строк** в супервизоре за 20 минут активной сессии оператора.
- `SetVoiceMode` (legacy) — 4 строки. Топик работает.
- Топик `/avatar/set_voice` живой (`pub=1/sub=1`).
- `docker logs rob-box-quest | grep -i voice` — пусто.

### 3.3 Гипотезы разрыва (по убыванию вероятности)

**H1. `_active_provider` пуст на момент выбора голоса.**
`bridge.set_voice` (`quest_node.py:716-721`): если `provider == ""` или
`voices_for(provider)` пуст — return `(False, "tts_unreachable", None)`.
ws_server отвечает `voice_set_nack{reason:"tts_unreachable"}` → клиент
показывает ошибку. **НО**: оператор говорит «ничего не происходит», а
`voice_set_nack` в логе `rob-box-quest` пуст. Если nack реально уходит в
сторону ws-клиента, клиент его игнорирует (UI-стейт остаётся в APPLYING,
applyTimeout через 6с → toast «Голос не применён: нет ответа от робота»
→ `voice_set_nack{reason:"нет ответа от робота"}`).

Но в issue написано «в логе супервизора ни одной строки SetVoice» — это
**исключает** H1 как единственную причину: даже nack «tts_unreachable»
должен логироваться на стороне супервизора (см. шаг 5 — лог пишется
**до** `SetParameters`). Если 0 строк → сообщение **не дошло** до супервизора.

**H2. Сообщение не публикуется в `/avatar/set_voice` (rclpy publish дроп).**
`bridge.set_voice` возвращает `(True, voice_id, None, None)` даже если
`publish()` бросил исключение — НЕТ: в коде (`quest_node.py:732-738`)
publish обёрнут в try/except, при исключении возвращается `publish_failed`.
Но этот nack тоже должен попасть в ws_server. Несоответствие не воспроизводится
по тексту issue.

**H3. Пакет не доходит до quest-сервера по WS.**
Picker уходит в `APPLYING` (стор) → `armApplyTimeout(voiceId)` → через 6с
`voice_set_nack{reason:"нет ответа от робота"}` + toast. Если бы пакет не
дошёл до ws-сервера вообще, ws-сервер не отвечал бы nack'ом — клиент сам
отрисовывает «нет ответа от робота». **Совпадает с наблюдением оператора**.
Но тогда в ws-сервере логов про cmd set_voice тоже не должно быть, а это
нельзя проверить из issue (там только `docker logs avatar-supervisor`).

**H4. ws-server валит cmd до публикации (rate-limit + сторонний предикат).**
`VOICE_SET_MIN_INTERVAL_S` → `voice_set_nack{reason:"rate_limited"}`. Это
**NACK** → клиент отрисует ошибку. Оператор видит «ничего не происходит»
→ несовместимо, если только клиент не игнорирует nack.

**H5. Клиент не доходит до picker apply (залипает на чипе стиля/языка).**
Оператор говорит «выбирает голос из шлема» — в UI это TTS picker (V-клавиша).
Если оператор на самом деле нажимает чип «ГОЛОС» (mode_manager), а не
открывает picker, клиент шлёт не `set_voice`, а что-то ещё (см. `modeManager`
ниже).

**H6. Архитектурная проблема: один cmd `set_voice` для двух фич.**
Проблема регрессионная. Если разработчик в будущем добавит поле `preset`
или `language` к picker-запросу — оно молча уйдёт в AV-28-ветку, и голос
не сменится. Контракт хрупкий.

---

## 4. Обязательно нужно проверить (action items → child cards)

### 4.1 Backend/runtime (assignee: backend)

- [ ] На **живом** роботе (10.1.1.21) выполнить один picker-apply и собрать:
  - `docker logs avatar-supervisor --since=10m | grep -E "SetVoice|voice"` —
    **полный** фрагмент, не выжимку.
  - `docker logs rob-box-quest --since=10m | grep -E "voice|nack|publish"` —
    **полный** фрагмент.
  - `ros2 topic echo /avatar/set_voice --once` во время apply.
  - `ros2 topic hz /avatar/set_voice --window 30` — есть ли трафик вообще.
  - WS-логи на стороне сервера (если пишутся — там будет видно, дошёл ли
    cmd и какой был payload).
- [ ] Проверить, что `_active_provider` в `QuestBridge` непустой на момент
  apply (debug-лог или прямой `ros2 topic echo /voice/tts/provider_state`).
- [ ] Если пакет доходит до супервизора, но `applied=False reason=…` — фикс
  сводится к устранению причины (race с provider, устаревший кэш).
- [ ] Если пакет НЕ доходит до супервизора — найти место дропа (ws-server?
  quest_node.py? мост?).

### 4.2 Frontend honesty (assignee: frontend)

- [ ] Сейчас picker **должен** показывать ошибку в случае nack (`armApplyTimeout`
  + `dispatchTts({kind:"voice_set_nack", voiceId, reason})` + toast). Проверить,
  что UI шлема реально рендерит nack и не «забывает» его (state-машина picker'а
  в `state/tts_picker_state.ts`).
- [ ] Пока расследование идёт — UI picker'а должен отображать **текущий голос**
  из `modeManager.currentVoice` (он же приходит в `voice_list`) рядом с
  применяемым, чтобы оператор видел «было X → пытаюсь Y», а не только
  optimistic-галочку.
- [ ] Клиентский комментарий в `main.ts:277-285` про «voice_id здесь НЕ
  шлём» относится ТОЛЬКО к `sendStyleChange`. Сейчас он рядом с другим
  кодом (picker's `apply`) и **вводит читателя в заблуждение** (issue #2138
  — жертва этого комментария). Переформулировать или вынести рядом с
  picker-блоком для ясности.

### 4.3 Devops/observability (assignee: devops)

- [ ] Добавить `bridge.set_voice` debug-лог: «publishing voice_id=X to
  /avatar/set_voice, provider=Y» (по аналогии с `tts_node` —
  `voice_used=<voice_id>` в `SetVoice:*`). Без него при следующем инциденте
  снова будем гадать.
- [ ] Добавить health-metric «set_voice pub/sub traffic» в дашборд, чтобы
  разрыв был виден за секунды, а не за 20 минут.

### 4.4 Архитектурный долг (assignee: architect — следующая карточка)

- [ ] Развести AV-27 и AV-28 по разным cmd: `JSON_CMD{tts_set_voice}` и
  `JSON_CMD{style_set}` (или эквивалентно). Один cmd, два предиката —
  это мина регрессии. См. §5 ниже.

---

## 5. Trade-off (раздел «что делаем» по результатам)

### Вариант A — точечный runtime-фикс

- **Что**: починить первопричину (provider-кэш / publish-дроп / клиентский nack-handler).
- **Плюс**: маленький PR, быстро, не ломает контракт.
- **Минус**: следующий разработчик снова наступит на мину AV-27/AV-28.
- **Когда**: если в runtime-расследовании (4.1) найдена локальная причина.

### Вариант B — развести cmd + точечный фикс

- **Что**: добавить отдельный cmd `tts_set_voice`, ввести deprecation period
  для `set_voice` без preset/language, потом удалить развилку.
- **Плюс**: устраняет класс багов; picker и style-чипы больше не могут
  пересечься.
- **Минус**: больше работы; требует миграции web-admin (если он тоже шлёт
  `set_voice`).
- **Когда**: даже если A починит — B устраняет архитектурный риск. Рекомендую.

### Вариант C — capability-honest UI + «голос выбирается через picker»

- **Что**: вместо кнопки «выбрать голос» в `voicePipeline`-панели сделать
  единственный путь — TTS picker (V-клавиша). Убрать возможный «третий путь»
  выбора голоса, если такой есть (см. H5).
- **Плюс**: устраняет путаницу.
- **Минус**: регрессия UX (если кнопка была сделана специально).
- **Когда**: если в runtime-расследовании найдётся, что оператор нажимает
  не picker.

**Рекомендация**: сначала A (диагностика → фикс первопричины), затем B
(архитектурный фикс cmd-развилки) — это и ADR-0018 (честный FAIL) и KISS
(минимальное изменение для исправления).

---

## 6. Что НЕ входит в этот разбор (out of scope)

- Реестр голосов у провайдеров (preset/style — работают, проверено).
- WS-сессия (падение #2099 починено #2110, воспроизводимости нет).
- ADR-0066 (legacy voice_mode → deprecated) — отдельная карточка, не здесь.

---

## 7. Определение готовности (DoD для child cards)

DoD считается выполненным, когда **все три** дочерние карточки (backend /
frontend / devops) дают raw-evidence:

- [ ] backend: `docker logs avatar-supervisor | grep SetVoice` ≥ 1 строка за
      сессию picker-apply; в строке `applied=True reason=applied`.
- [ ] backend: следующая за picker-apply реплика озвучена выбранным голосом;
      `voice_used=<voice_id>` в `tts_node` логе.
- [ ] backend: invalid voice_id → оператор видит **отказ** (тост или
      inline-ошибка), а не тишину.
- [ ] frontend: `docker logs avatar-supervisor | grep voice_mode_deprecated`
      за сессию — 0 строк после фикса legacy-чистки в клиенте.
- [ ] devops: метрика «set_voice traffic» в дашборде, или эквивалентный
      healthcheck, который виден на живом роботе.
- [ ] архитектурный долг (4.4): отдельная карточка создана, не в этом PR.

---

## 8. Связанные ссылки

- Issue: krikz/rob_box_project#2138
- Live evidence: робот 10.1.1.21, 2026-09-08, 20 минут сессии оператора.
- Дизайн: `docs/architecture/tts-picker-ros-path.md` §128-150
- API: `meta-quest-api.md` §5 (WIRE_TO_VOICE_INPUT_MODE), §P7 (AV-28)
- ADR: 0018 (honesty), 0028 (avatar supervisor), 0066 (dialogue control).
