# ADR-0077 — ТАРС в шлем: автоплей чанков + sample_rate + акцепт-тон + tars_state

| | |
|---|---|
| Статус | accepted (P0 follow-up #2162) |
| Контекст | ТАРС в шлем оператора не отвечает голосом: текст видно, в наушниках тишина |
| Следствие | Шифу теряет канал громкой связи из шлема |
| Решение | см. §3 |

## 1. Проблема

Байты синтезированной речи доходят до WS-клиента, но в наушниках шлема
оператора — тишина. Воспроизведение голоса личности (динамики робота)
работает; сценарий ТАРС-в-шлем — нет.

Дефекты уже диагностированы (см. тело задачи `t_30a3025b`):

1. **ws_server**: `_send_audio_done` намеренно НЕ шлёт `operator_tts_done`,
   клиентский `play()` зовётся **только** в обработчике `operator_tts_done`,
   которого никогда не приходит. Чанки копятся в `pending` и молча
   выбрасываются лимитом.
2. **decodeAudioData**: сырой int16-LE PCM не декодируется (sink).
3. **Нет `sample_rate`**: клиент не знает, на какой частоте играть.
4. **Нет сигнала "принято"** — оператор не слышит подтверждения, что wake
   услышан. В `/avatar/command_result` ответ приходит уже после LLM —
   слишком поздно для UI-подтверждения.
5. **Нет видимой стадии цикла** — оператор не понимает, что сейчас
   делает ТАРС.

## 2. Не делаем

* **Микросервисы.** Всё в одном `ws_server` + `quest_node`.
* **Event sourcing.** Нам нужен существующий поток чанков, не новая
  абстракция поверх.
* **CQRS.** Тут один потребитель — шлем.
* **preview-канал** (AV-27): `done` от сервера публикуется, не трогаем.

## 3. Решение

### 3.1 Каждый `operator_tts_audio` — самодостаточный чанк

Меняем контракт `ws_server.deliver_audio` для `stream="operator_tts"`:

* В meta-`operator_tts_audio` добавляем поле `sample_rate: int` (Гц).
* Никакого ожидания `operator_tts_done`. Клиент играет чанк **сразу
  по приходу байт** (по meta+chunk), ставя в **последовательную
  очередь** (следующий стартует, когда предыдущий source закончился).
* `operator_tts_done` / `operator_tts_error` **оставляем** как
  flush/сброс (на случай, если позже понадобятся).

**Обратная совместимость:** для `stream="preview"` контракт НЕ меняется
(`done` уже публикуется сервером, тесты AV-19/AV-27 зелёные).

### 3.2 Ручная сборка AudioBuffer из Int16 PCM

`AudioContext.createBuffer(1, samples, sampleRate)` + `copyToChannel` из
`Int16Array` (нормализация делением на 32768). Для `mp3/opus/wav` —
fallback на `decodeAudioData` (как раньше).

### 3.3 AudioContext.resume()

Quest — immersive VR, автоплей-политика режет аудио до user gesture.
`audioCtx.state === 'suspended'` → `await audioCtx.resume()` ПЕРЕД
`start()`. Лог на неуспех.

### 3.4 Barge-in (`stop()`)

Гасит **текущий** source **и** всю очередь (а не только активный
request_id). Иначе в очереди останется «накопилось» и сыграет после
отпускания PTT.

### 3.5 Акцепт-тон

**Источник:** `/avatar/stt/result` (String JSON, шлёт `stt_node` сразу
после wake-матча). Это **самый честный** сигнал «принято» — он появляется
в момент, когда STT подтвердил wake-word.

**Маршрут:** `quest_node` подписывается на `/avatar/stt/result` →
`broadcast_json_event({type: "tars_state", stage: "accepted", request_id, text, ts_ms})`.

**Синтез тона:** **локально в клиенте** (WebAudio OscillatorNode),
огибающая ADSR, ~100 мс, частота ~880 Гц. **Не через сеть** — иначе
будет лаг и сетевой мусор.

**Не конфликтует с речью:** тон звучит в момент wake (TTA ≈ 50-150 мс),
голос ТАРС приходит через 1-2 с (LLM + TTS). Разрыв достаточный.

### 3.6 tars_state

Новый JSON_EVENT `{type: "tars_state", stage, request_id?, text?, ts_ms, sample_rate?}`.
Стадии:

* `accepted` — `/avatar/stt/result` (wake принят, **stt_node**) **или**
  `/avatar/tts/audio_meta` (TTS готов начать чанк, **tts_node**). Оба
  источника симметричны — клиент может показывать «ACCEPTED» в обоих
  случаях; обычно stt/result приходит первым, audio_meta ближе к голосу.
  Контракт (ADR-0078 §3.6): на стороне клиента принимается
  `tars_state_indicator.parseTarsStateEvent`.
* `thinking` — между accepted и speaking. Не публикуем явно — клиент
  вычисляет сам (timing).
* `speaking` — **первый** `operator_tts_audio` чанк в шлем (генерится
  на стороне клиента в `main.ts`).
* `idle` — через 100 мс после `operator_tts_done`/`operator_tts_error`
  или после `command_result.ok=True` без TTS (например, `agent_disabled`).

**Клиент:** подписка на тип `tars_state`, обновление
`src/ui/voice_state_indicator.ts`, `src/scene/tars1_text_panel.ts`,
`src/scene/tars2_metrics_panel.ts`, `src/state/supervisor_state.ts`.

**Реализация (quest_node, ROS→WS relay):**
* `_on_avatar_stt_result` (sub `/avatar/stt/result` от stt_node) →
  WS `tars_state accepted` с `text` и `request_id={client_id}:{ts_ms}`.
* `_on_avatar_tts_audio_meta` (sub `/avatar/tts/audio_meta` от tts_node)
  → кеш `request_id → sample_rate` + WS `tars_state accepted` с
  `sample_rate`.

## 4. sample_rate (side-channel audio_meta)

* Реальный rate — `tts_node.audio_output_sample_rate` (declare,
  default 16000 Гц; **сверено на железе 10.1.1.21**:
  `ros2 param get /tts_node audio_output_sample_rate` →
  `Integer value is: 16000`).

### 4.1 Проблема исходного wip

Изначально планировалось пробрасывать `sample_rate` через **приватный
атрибут** AudioData-сообщения (`setattr(msg, "_tars_sample_rate", ...)`).
rclpy через DDS сериализует **только поля IDL**; произвольные Python-атрибуты
на стороне подписчика **отсутствуют**. Наблюдалось в stderr vitest:
`operator_tts PCM without sample_rate in meta — fallback 16000`. Это
критичный GAP, который нужно закрыть.

### 4.2 Решение: side-channel топик `/avatar/tts/audio_meta`

`tts_node` создаёт **отдельный** publisher:

* `headset_audio_meta_topic` (declare, default `/avatar/tts/audio_meta`).
* Тип: `std_msgs/String`, payload — JSON:
  ```json
  {"request_id": "<uuid8 от supervisor>", "sample_rate": 16000, "ts_ms": 1700000000000}
  ```

`tts_node._publish_headset_audio(audio, sample_rate, *, request_id=None)`
публикует audio_meta **ДО** AudioData (порядок гарантирован в одном
DDS-потоке). `request_id` берётся из `self._avatar_tts_request_id`,
которое tts_node выставляет в `_on_avatar_tts_request` ДО старта
синтеза (supervisor генерит uuid8 в `_publish_avatar_tts`).

### 4.3 Потребитель: `quest_node._on_avatar_tts_audio_meta`

* Кеш `self._avatar_request_sample_rate: dict[str, int]`.
* Рассылает в WS `tars_state accepted` с `request_id` и `sample_rate`.
* LRU-обрезка до 8 записей (защита от утечки).

`quest_node._on_avatar_tts_audio` берёт `sample_rate` из кеша и
передаёт в `ws_server.deliver_audio(sample_rate=...)`. Fallback —
16000 (для обратной совместимости со старыми клиентами).

### 4.4 Тестовая матрица

* `test_publish_headset_audio_emits_meta_before_audio_with_request_id` —
  audio_meta+AudioData в правильном порядке, payload корректен.
* `test_publish_headset_audio_meta_falls_back_to_declared_sample_rate` —
  fallback на `audio_output_sample_rate`.
* `test_on_avatar_tts_audio_meta_caches_sample_rate_and_emits_tars_state` —
  кеш + WS-event.
* `test_on_avatar_tts_audio_uses_cached_sample_rate` /
  `test_on_avatar_tts_audio_falls_back_to_16khz_when_no_cache` —
  deliver_audio подставляет кешированный sample_rate.

### 4.5 Что НЕ проверяем

* **Живой шлем** с реальным MiniMax-стримом 24 кГц: требует живого
  прогона на 10.1.1.21, делается e2e-процессом после PR.
* **Race-condition audio_meta после AudioData** (в теории возможен, но
  в одном DDS-потоке порядок гарантирован; fallback покрывает
  практический случай).

## 5. Границы

**Трогаем:**
- `src/rob_box_quest/rob_box_quest/quest_node.py` — avatar-tts путь +
  `tars_state` (B, C)
- `src/rob_box_quest/rob_box_quest/server/ws_server.py` — `sample_rate`
  в meta (A)
- `src/rob_box_quest/webxr_client/src/ui/operator_audio_sink.ts` —
  очередь + PCM (A)
- `src/rob_box_quest/webxr_client/src/main.ts` — только operator_tts /
  tars_state (A, B, C)
- индикаторы в `src/ui`
- `src/rob_box_voice/rob_box_voice/tts_node.py` — `_publish_headset_audio`
  принимает `sample_rate` (A)
- тесты

**НЕ трогаем:**
- `quest_node.set_voice` и валидация голосов (issue #2138)
- `/avatar/voice_pipeline` и грип-пайплайн
- `streams/registry.py`
- `webxr_client/src/scene/captain_bridge.ts`
- preview-синтез в supervisor/tts_node

## 6. Тесты

**Python** (`cd src/rob_box_quest && python -m pytest -v`):
- `deliver_audio` шлёт `sample_rate` в meta для `operator_tts`.
- `tars_state: accepted/thinking/speaking/idle` релеится через WS.
- Регрессия preview-канала.

**TS** (`cd src/rob_box_quest/webxr_client && npm test && npm run typecheck`):
- `operator_audio_sink`: ручная сборка AudioBuffer из Int16 PCM.
- Очередь чанков, barge-in сбрасывает всё.
- `sample_rate` в meta.
- `tars_state` события.

## 7. Что НЕ проверяем (честно)

* **Живой шлем.** Тип возврата `ArrayBuffer`, точность
  `createBuffer`/`copyToChannel` и `AudioContext.resume()` в Quest
  браузере без шлема не воспроизвести. Запуск шлема и робота — после PR
  отдельным процессом e2e.
* **Реальная синхронизация акцепт-тона и речи.** Тон синтезируется
  локально, задержка < 50 мс; речь идёт через ROS→WS, не менее
  800 мс. На шлеме не проверял — записано честно.
