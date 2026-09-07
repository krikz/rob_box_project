# ADR-0055 implementation plan: обратный канал ТАРС в шлем (issue #1993, карточка t_dce67f3f)

> Это **implementation plan** для backend-воркера. Дизайн/контракты —
> в `0055-operator-tts-headset-channel.md`. Этот файл — последовательность
> коммитов, тестов и регрессий, которая превращает ADR-0055 в код
> без пересечения с шagaми 04а, 5а (PR #2013 / #2037) и 07а (PR #2041).

## Что в итоге появится в коде

- `ws_server.deliver_audio(stream, ws, request_id, ...)` — обобщённая
  функция доставки аудио в WS, два известных стрима: `"preview"` (уже есть)
  и `"operator_tts"` (новый).
- `ws_server._audio_pending: dict[str, dict[request_id, (ws, ts)]]` —
  per-stream реестр активных запросов с лимитом `VOICE_PREVIEW_MAX_CONCURRENT`
  на каждый stream (а не суммарно — два пользователя не должны мешать
  друг другу).
- `quest_node`: подписка на `/avatar/tts/audio` (`AudioData`) +
  side-channel подписка на `/avatar/tts/request` (`String` JSON)
  для регистрации `request_id → ws` **до** прихода первого аудио-чанка
  (устраняет race «request ушёл, ws ещё не зарегистрирован»).
- `tts_node`: подписка `/avatar/tts/request` через тот же slot pool,
  что и `/voice/tts/request`, но **без** ALSA и **без** `/voice/audio/speech`;
  публикует результат в `/avatar/tts/audio` (`AudioData` int16 LE PCM).
- `avatar_supervisor`: новый `_avatar_tts_request_pub` (топик
  `/avatar/tts/request`); все реплики «принял», «не умею», «камера повёрнута»
  теперь идут через него, а не через `/voice/tts/request`.
- `webxr_client`: новый `operator_audio_sink.ts`, симметричный
  `preview_audio_sink.ts`; типы `operator_tts_audio/_done/_error`
  в `wire/messages.ts:263+`; barge-in: `voice_ptt_start` →
  `operatorAudioSink.stop()` + серверный `control_callback("STOP")`.

## Структура коммитов (5 штук)

Каждый коммит — отдельный зелёный шаг. WIP-коммиты
(`wip(operator-agent 05b #1993): ...`) идут в ту же ветку, base = `develop`.

### Коммит 1: `feat(quest #1993): ws_server.deliver_audio(stream, …) + реестр по стримам`

**Файлы:**

- `src/rob_box_quest/rob_box_quest/server/ws_server.py`
- `src/rob_box_quest/test/unit/server/test_ws_server_av19.py`
  (или новый `test_ws_server_deliver_audio.py`)

**Что:**

1. Добавить `_AUDIO_STREAMS: frozenset[str] = frozenset({"preview",
   "operator_tts"})` — whitelist стримов; неизвестный stream →
   `log.warning(...)` и `return False` (защита от «широкого шва»).
2. Заменить `self._preview_pending: dict[str, tuple[Any, float]]` на
   `self._audio_pending: dict[str, dict[str, tuple[Any, float]]]`
   (per-stream).
3. Реализовать:
   ```python
   def deliver_audio(
       self, *, stream: str, ws: Any, request_id: str,
       audio_bytes: bytes, audio_format: str, content_type: str,
       seq: int, total: int,
   ) -> bool: ...
   ```
   Логика — копия `deliver_preview_audio`, но `meta["type"]` —
   `"preview_voice_audio"` для `stream="preview"`,
   `"operator_tts_audio"` для `stream="operator_tts"`.
4. Старые `deliver_preview_audio / deliver_preview_done /
   deliver_preview_error` — становятся **тонкими обёртками** над
   `deliver_audio(stream="preview", ...)` и `start_preview_session`.
   Поведение и сигнатура **не меняются** (тесты AV-19, AV-27 должны
   остаться зелёными без правок).
5. Утилитарный API для side-channel:
   ```python
   def register_audio_session(
       self, stream: str, request_id: str, ws: Any,
   ) -> bool: ...
   ```
   Возвращает `False`, если для этого stream уже
   `VOICE_PREVIEW_MAX_CONCURRENT` активных request_id'ов — клиент
   получит `*_error{reason: "too_many"}`. (Допускается тот же лимит,
   что и для preview: на Quest один оператор.)

**Тесты (юнит, без железа):**

- `test_deliver_audio_operator_tts_sends_binary_frame` — мок ws,
  `deliver_audio(stream="operator_tts", ws=mock_ws, ...)`, ожидаем
  `_schedule_ws_send` с `meta["type"] == "operator_tts_audio"` и
  `_schedule_ws_send_binary(mock_ws, audio_bytes)`.
- `test_deliver_audio_unknown_stream_is_dropped` —
  `stream="bogus"` → `False`, без побочных эффектов.
- `test_register_audio_session_too_many` — заполнить реестр до лимита,
  `register_audio_session(...)` → `False`.
- `test_deliver_preview_audio_still_works_after_refactor` —
  регрессия: вызвать `deliver_preview_audio` → проверить meta +
  binary frame через ту же обёртку.
- `test_deliver_preview_concurrent_limit_isolated_per_stream` —
  заполнить preview до лимита, `register_audio_session("operator_tts",
  ...)` → `True` (стримы не делят слот).

**Проверки:**

- `pytest -q src/rob_box_quest/test/unit/server/test_ws_server_av19.py
  src/rob_box_quest/test/unit/server/test_ws_server.py` — зелёное.
- `rg -n "deliver_preview_audio" src/` — не должно быть **новых**
  прямых использований вне самой обёртки (старые вызовы quest_node
  остаются, но идут через обёртку).
- `rg -n "type ==" src/rob_box_quest/webxr_client/src/wire/messages.ts` —
  добавить `OperatorTtsAudioMessage`/`OperatorTtsDoneMessage`/
  `OperatorTtsErrorMessage` discriminated union (без runtime-логики —
  чисто типы, чтобы фронт не падал при компиляции).

**Acceptance для коммита:** все новые юнит-тесты зелёные, AV-19/AV-27 —
  зелёные, типы на клиенте компилируются.

### Коммит 2: `feat(voice #1993): tts_node подписка /avatar/tts/request → /avatar/tts/audio`

**Файлы:**

- `src/rob_box_voice/rob_box_voice/tts_node.py`
- `src/rob_box_voice/test/unit/test_tts_node_avatar.py` (новый)

**Что:**

1. Параметр ноды `headset_audio_topic` (default `/avatar/tts/audio`),
   параметр `avatar_request_topic` (default `/avatar/tts/request`),
   параметр `avatar_error_topic` (default `/avatar/tts/error`).
2. Подписка `String, "/avatar/tts/request", self._on_avatar_tts_request,
   10`. Контракт сообщения — копия `/voice/tts/request`
   (`request_id`, `ssml`, `voice?`, `language?`, `priority?`), плюс
   обязательное `sink: "headset"`. Любой другой `sink` →
   `self._avatar_tts_error_pub.publish(JSON{request_id, error:
   "invalid_sink"})` и DROP, **без** side-effects.
3. Хранить `self._avatar_tts_request_id: Optional[str]` (один активный
   avatar-запрос; cancel через тот же `control_callback` —
   см. коммит 4).
4. Синтез через существующую `_run_synthesis_worker` (BLK-9 slot
   pool) **без** `_publish_audio(...)`. После успешного
   `_prepare_audio_for_topic(...)` — публиковать `AudioData` в
   `self.headset_audio_topic`:
   ```python
   from audio_common_msgs.msg import AudioData
   msg = AudioData()
   msg.data = audio_bytes  # int16 LE PCM, тот же SR что и /voice/audio/speech
   self._avatar_audio_pub.publish(msg)
   ```
5. В конце — `self._voice_tts_finished_pub.publish(JSON{request_id,
   success, speech_id, dialogue_id, batch_id, batch_size})` (тот же
   топик, что и для `/voice/tts/request`, чтобы существующие
   подписчики не ломались).
6. Ошибки → `self._avatar_tts_error_pub.publish(JSON{request_id,
   error})` и тот же `_voice_tts_finished_pub` с `success=False`.

**Тесты (юнит, с моком `rclpy.node.Node`):**

- `test_avatar_tts_request_invalid_sink_published_error` —
  `sink="speaker"` → `error_pub` получил сообщение, синтез **не**
  стартовал.
- `test_avatar_tts_request_publishes_audio_data` —
  мок `_run_synthesis_worker` (monkeypatch), ожидаем
  `audio_pub.publish(AudioData(...))` с тем же PCM, что вернул
  воркер.
- `test_avatar_tts_request_emits_finished_with_request_id` —
  в конце успешного синтеза `_voice_tts_finished_pub` получил
  `request_id` из входа.

**Проверки:**

- `pytest -q src/rob_box_voice/test/unit/test_tts_node.py
  src/rob_box_voice/test/unit/test_tts_node_avatar.py` — зелёное.
- `rg -n "_publish_audio" src/rob_box_voice/rob_box_voice/tts_node.py` —
  в `_on_avatar_tts_request` НЕ должно быть вызова `_publish_audio`
  (только `_avatar_audio_pub.publish`).
- `rg -n "headset_audio_topic\|avatar_request_topic" src/` —
  параметры объявлены в `__init__` и используются в `_on_*`.

**Acceptance:** юнит-тесты зелёные, регрессии в `/voice/tts/request`
  нет (там ALSA-путь не тронут).

### Коммит 3: `feat(quest #1993): quest_node маршрут /avatar/tts/audio → ws_server.deliver_audio + side-channel /avatar/tts/request`

**Файлы:**

- `src/rob_box_quest/rob_box_quest/quest_node.py`
- `src/rob_box_quest/test/unit/test_quest_node_avatar.py` (новый)

**Что:**

1. Параметры `avatar_tts_audio_topic` (default `/avatar/tts/audio`),
   `avatar_tts_request_topic` (default `/avatar/tts/request`),
   `audio_qos` (default 10 — тот же, что у других `AudioData` подписок).
2. Подписка `AudioData, self.avatar_tts_audio_topic,
   self._on_avatar_tts_audio, audio_qos`. Хранить
   `self._current_avatar_request_id: Optional[str]` и
   `self._current_avatar_ws: Optional[Any]` (обновляются в коммит-4
   через side-channel).
3. `self._on_avatar_tts_audio(msg)` →
   `self.ws_server.deliver_audio(stream="operator_tts",
   ws=self._current_avatar_ws, request_id=self._current_avatar_request_id,
   audio_bytes=msg.data, audio_format="pcm_s16le",
   content_type="audio/pcm", seq=0, total=0)`.
   `seq/total` для операторского канала — `0/0` (клиент собирает
   чанки в `operatorAudioSink` по `request_id` без seq-нумерации;
   seq заведён в типе для forward-compat с приоритетной очередью 07а).
4. Подписка `String, self.avatar_tts_request_topic,
   self._on_avatar_tts_request_meta, 10` — **side-channel** для
   детерминированной сессионной привязки (ADR-0055 §quest_node).
   Внутри: парсим JSON, проверяем `sink == "headset"` (иначе игнор —
   это tts_node канал, не quest), берём **активную** ws-сессию:
   - `ws_server.get_active_sessions() == 1` → этот единственный ws;
   - иначе — самый свежий session_id из `ws_server._sessions`.
   - `register_audio_session("operator_tts", request_id, ws)` →
     `True`/`False` (последний случай — лог `WARNING too_many`, ответ
     не доставляем, реплика ТАРС теряется; supervisor должен видеть
     `/voice/tts/finished{success=False}` и не переспрашивать).
5. Сохранить `self._current_avatar_request_id = request_id`,
   `self._current_avatar_ws = ws` для колбэка из п.3.
6. На `ws.close` (через существующий хук `ws_server`) — сбросить
   `self._current_avatar_ws = None`.

**Тесты (юнит, мок ws_server):**

- `test_avatar_tts_request_registers_session_for_unique_ws` —
  ws_server с одной активной сессией → `_on_avatar_tts_request_meta`
  вызвал `register_audio_session(stream="operator_tts", request_id="r1",
  ws=the_only_ws)`.
- `test_avatar_tts_audio_routes_to_registered_ws` —
  предрегистрировать request_id → ws, вызвать `_on_avatar_tts_audio`
  с PCM-байтами → `deliver_audio` вызван с тем же request_id и
  `stream="operator_tts"`.
- `test_avatar_tts_request_ignored_when_sink_not_headset` —
  `sink="speaker"` → `register_audio_session` НЕ вызван.

**Проверки:**

- `pytest -q src/rob_box_quest/test/unit/test_quest_node.py
  src/rob_box_quest/test/unit/test_quest_node_avatar.py` — зелёное.
- `rg -n "avatar_tts_audio_topic" src/rob_box_quest/` — параметр
  объявлен, default совпадает с ADR-0055.

**Acceptance:** юнит-тесты зелёные, ws_server-регрессии (коммит 1)
  не сломаны.

### Коммит 4: `feat(supervisor #1993): avatar_supervisor публикует /avatar/tts/request для собственных реплик`

**Файлы:**

- `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py`
- `src/rob_box_supervisor/test/unit/test_supervisor_avatar_tts.py` (новый)

**Что:**

1. Publisher `self._avatar_tts_request_pub = self.create_publisher(String,
   "/avatar/tts/request", 10)`.
2. Метод `_publish_avatar_tts(text: str, language: str | None = None,
   voice: str | None = None) -> str` (возвращает `request_id`):
   - генерирует `uuid.uuid4().hex[:8]` как `request_id`;
   - публикует `String(data=json.dumps({request_id, ssml, sink:
     "headset", voice?, language?}))`.
3. Все точки, где сегодня supervisor отвечает «принял», «не умею»,
   «камера повёрнута», «не понял» (через `/voice/tts/request`,
   `_publish_say_request` или эквивалент) — заменить на
   `_publish_avatar_tts(text, ...)`.
4. `/voice/tts/request` остаётся только для **инструмента `say`** —
   см. `tools/say.py` (или эквивалентный); supervisor **не** ходит
   туда для собственных реплик.

**Тесты (юнит, мок `rclpy.node.Node`):**

- `test_publish_avatar_tts_writes_headset_request` — вызвать
  `_publish_avatar_tts("готово")`, перехватить `String` через
  `mock_publisher`, проверить JSON `{request_id: not_empty, ssml:
  "готово", sink: "headset"}`.
- `test_supervisor_does_not_publish_to_voice_tts_request_for_own_lines`
  — мок-фильтр на все publisher'ы supervisor'а, вызвать реплику
  supervisor'а (например, через ветку «команда не распознана»),
  убедиться что НЕ было публикации в `/voice/tts/request`.

**Проверки:**

- `pytest -q src/rob_box_supervisor/test/unit/` — зелёное.
- `rg -n "_publish_say_request\|/voice/tts/request" src/rob_box_supervisor/` —
  в `supervisor_node.py` остаётся **только** в обработчике инструмента
  `say`, не в путях собственных реплик.

**Acceptance:** регресс `say` (отдельный путь) не сломан;
  реплики supervisor'а идут в `/avatar/tts/request`.

### Коммит 5: `feat(quest-webxr #1993): operator_audio_sink + barge-in + wire типы`

**Файлы:**

- `src/rob_box_quest/webxr_client/src/wire/messages.ts` (только типы)
- `src/rob_box_quest/webxr_client/src/ui/operator_audio_sink.ts` (новый)
- `src/rob_box_quest/webxr_client/src/main.ts`
- `src/rob_box_quest/webxr_client/test/unit/operator_audio_sink.test.ts`
  (новый — vitest)

**Что:**

1. Добавить discriminated union:
   ```ts
   | { type: "operator_tts_audio"; request_id: string;
       format: "pcm_s16le"|"mp3"|"opus"|"wav"; content_type: string;
       seq: number; total: number; ts_ms: number }
   | { type: "operator_tts_done"; request_id: string; ts_ms: number }
   | { type: "operator_tts_error"; request_id: string; reason: string;
       ts_ms: number }
   ```
   (`_done`/`_error` пока не шлются сервером — ADR-0055 — но типы
   заведены для forward-compat.)
2. `operator_audio_sink.ts` — симметричный `preview_audio_sink.ts`,
   но без `dispatchTts` (это речь в шлем, не превью-UI) и без
   `preview-ui` чанков. На `voice_ptt_start` → `stop()` (тот же
   поведенческий контракт, что у preview на `voice_ptt_start`).
3. В `main.ts` добавить `case "operator_tts_audio"` →
   `operatorAudioSink.push(meta, binaryFrame)`; `case
   "operator_tts_done"` → `operatorAudioSink.finish()`; `case
   "operator_tts_error"` → `operatorAudioSink.error(reason)`.
4. `voice_ptt_start` уже шлётся на сервер (ModeManager) — это
   поведение не меняем. Локальный barge-in: `operatorAudioSink.stop()`
   **до** отправки `voice_ptt_start`, чтобы оператор услышал тишину
   раньше, чем сервер обработает STOP.
5. На сервере `control_callback("STOP")` для `/avatar/tts/control` —
   используем **тот же** `_avatar_tts_control_pub`, что и для
   `/voice/tts/control` (формат команды совпадает, см. ADR-0055
   §tts_node). Это **без нового кода** в tts_node, кроме
   подписки `String, "/avatar/tts/control", self.control_callback, 10`
   (т.е. **тот же** `control_callback` без изменений).

**Тесты (vitest):**

- `test_operator_audio_sink_buffers_chunks_by_request_id` —
  закинуть 2 чанка с одним `request_id`, потом `done` → sink
  проигрывает конкатенацию.
- `test_operator_audio_sink_ptt_start_stops_immediately` —
  закинуть чанк, `voice_ptt_start` → `stop()`, второй чанк
  игнорируется.
- `test_operator_audio_sink_error_drops_pending` — `error{reason:
  "synth_failed"}` после 1 чанка → sink не публикует звук.

**Проверки:**

- `pnpm --filter webxr-client test` — зелёное.
- `pnpm --filter webxr-client typecheck` — без ошибок (важно: типы
  `wire/messages.ts` discriminated union).

**Acceptance:** локальный barge-in работает в unit-тестах; серверный
  STOP идёт через существующий `control_callback` без новых топиков.

## Чего этот план НЕ делает

- Приоритет в `tts_node` — шаг 07а, PR #2041 (открыт, не пересекаемся).
- AudioWorklet — шаг 5а-0, PR #2013 (merged, уже в develop).
- Wake-маршрутизация в `stt_node` — шаг 5.
- Реальный e2e на железе (Quest, TARS на шлеме) — отдельная
  карточка после merge всех 5 коммитов (см. ADR-0055 DoD).

## Definition of Done (5 коммитов вместе)

- [ ] `deliver_audio(stream="operator_tts", ...)` существует и проходит
      юнит-тест: ros-msg AudioData → registered ws → JSON_EVENT + binary
      frame (без base64 в JSON).
- [ ] Preview-канал не сломан: регрессия на `deliver_preview_audio`
      через обёртку.
- [ ] Barge-in: юнит-тест на `voice_ptt_start` →
      `operatorAudioSink.stop()` (клиент) + `control_callback("STOP")`
      → `_avatar_tts_request_id` сбрасывается (сервер).
- [ ] `avatar_supervisor` публикует `/avatar/tts/request` для каждой
      собственной реплики (не для `say`).
- [ ] `say` и реплики ТАРС **не** пересекаются на топиках
      (`/voice/tts/*` vs `/avatar/tts/*`).
- [ ] Юнит-тесты пакетов `ws_server`, `quest_node`, `tts_node`,
      `supervisor_node`, `webxr-client` — все зелёные.
- [ ] `gh pr checks` — все required checks SUCCESS.
- [ ] `scripts/agent_flow/validate_honesty.sh` — без warning
      (никаких «проверил», «работает» без raw-цифр).

## Revert-ветка (ADR-0013)

После merge — создать `z-{revert}/1993-revert-operator-tts-headset-channel`
(Шифу делает руками, воркеры не создают `z-{revert}`).
Команда отката: `git revert -n <merge-sha>..HEAD~5 && git push`.

## Связанные ADR

- ADR-0055 (этот шаг) — основной дизайн-документ.
- ADR-0051 (supervisor/operator-agent/arbiter split, §2.9) — родитель.
- ADR-0054 (wake stream, шаг 5а) — параллельный шаг, не пересекается.
- ADR-0052 (wake_words.yaml SSoT) — личность wake живёт в коде.
- ADR-0013 (incremental delivery) — поэтому 5 коммитов, не один big-bang.
- ADR-0018 (honest FAIL) — DoD требует raw-цифры и grep-evidence, без
  голословных «проверил».