# ADR-0131 — utterance_id: один источник правды «кто сказал ЭТУ фразу»

**Дата:** 2026-09-23
**Статус:** Accepted (PR-1) — юнит-тесты зелёные (raw ниже); e2e на роботе НЕ прогонялся, координатор запустит отдельно
**Автор:** шиди (Claude Code) по задаче товарища Шифу
**Issue:** [#2829](https://github.com/krikz/rob_box_project/issues/2829)
**Связанные:** #2818 (зона сомнения/переспрос, `classify_name_confidence`), #2798 (переспрос при регистрации), #2757 (рост галереи по сессии), #2822 (system_context), #2779/#2789 (privacy_note, backlog-хинт), #2769 (честный отказ регистрации), #2747/#2748/ADR-0127 (регистрация и рост галереи — PR-2 этого issue)

## 1. Проблема

Робот после смены собеседника какое-то время обращается к новому человеку именем предыдущего (наблюдение товарища Шифу в мастерской 23.09, develop `1c6bb6bcd`). Причина — четыре разных сигнала о том, «кто говорит», ни один из которых не привязан к конкретной фразе:

| Источник | Топик | Упоминаний в `dialogue_node.py` |
|---|---|---|
| `_current_speaker` | `/voice/speaker/result` (resemblyzer, `speaker_id_node`) | 18 |
| `speaker_context`/`speaker_tag` | `/voice/stt/speaker` (метка Yandex speaker_analysis) | 15/30 |
| `_speaker_by_text` | сопоставление по тексту фразы | 6 |
| `_speaker_tracker` | трекер подтверждения тега (2+ фразы подряд) | 5 |

`/audio/speech_audio` уходит как `AudioData` (сырые PCM-байты, без заголовка/id) одним сообщением на фразу — `audio_node.py` публикует его целиком, без разбиения (см. `speech_audio_pub.publish(msg)` после накопления `speech_audio_buffer`). И `stt_node`, и `speaker_id_node` подписаны на этот же топик и получают байт-в-байт одинаковое сообщение, но их результаты между собой никак не связаны.

Ядро бага — `dialogue_node._apply_speaker_identity` (было: строка ~3080):

```python
await asyncio.sleep(0.30)
with self._speaker_lock:
    sp = dict(self._current_speaker)
```

STT (Yandex/Vosk) занимает 1-4с, биометрия (resemblyzer) — 0.6-1.9с в норме, но иногда до ~50с сразу после старта ноды (первая фраза, холодная модель). Когда STT приходит раньше биометрии текущей фразы, `_current_speaker` всё ещё хранит результат ПРЕДЫДУЩЕЙ — робот адресуется не тому человеку. `_current_speaker` сам по себе никогда не протухает.

## 2. Решение

Единый `utterance_id` для фразы, известный и `stt_node`, и `speaker_id_node`, и join по нему в одном модуле-владельце.

### 2.1 Механизм id — вариант (а): хеш PCM-байт

Рассмотрены варианты:

* **(а) детерминированный хеш байт `/audio/speech_audio`** — `sha1(bytes)[:12]`, считают обе ноды независимо от одних и тех же байт. Не меняет тип сообщения, не требует новой договорённости между нодами по формату/протоколу — только общая чистая функция `core/utterance_id.py::compute_utterance_id`.
* (б) явный id от `audio_node` (новое поле/топик) — требует правки протокола `AudioData`-обёртки или отдельного топика с порядковым номером, которого сейчас нет; больше поверхность изменений при том же результате.

Выбран (а). Проверено, что ни `stt_node`, ни `speaker_id_node` не режут и не склеивают буфер до вычисления хеша:
* `stt_node._process_audio`: `audio_bytes = bytes(msg.data)` — используется как есть для распознавания (chunked только ВНУТРИ Vosk recognizer, сами байты не мутируются).
* `speaker_id_node._on_speech_audio`: `pcm_bytes = bytes(msg.data)` — тоже как есть.

Оба места хешируют один и тот же `bytes(msg.data)` — значит `utterance_id` гарантированно совпадёт.

12 hex символов (48 бит) — компромисс между читаемостью в логах/JSON и вероятностью коллизии (пренебрежимо мала для количества фраз за время жизни сессии робота).

### 2.2 Quest-аудио (грип/wake шлема)

`stt_node.quest_audio_callback` / `quest_wake_audio_callback` идут в `_process_audio(source=_SRC_PTT|_SRC_WAKE)`, результат — в `/avatar/ptt/result` / `/avatar/stt/result`, совсем другой путь, `speaker_id_node` эти топики не слушает вообще (оператор, не голосовая биометрия). Для них `utterance_id` не публикуется (диктора не отличаем — это оператор по конструкции), в `_apply_speaker_identity` эти реплики никогда не попадают. Явно НЕ в скоупе.

### 2.3 Топики: без смены типа `/voice/stt/result`

`/voice/stt/result` — **plain text**, не JSON (`stt_node.py` — «контракт /voice/stt/result (plain text) НЕ меняем», `publish_result`: `msg.data = text`). Потребители: `telegram_node`, `command_node`, `context_aggregator_node`, `tools/gui/*`, `scripts/tts_bench`, `scripts/voice_bench`, e2e-харнесс (`.github/e2e/telegram_userbot_e2e.py`). Превращать его в JSON — это НЕ инкрементальный PR (ADR-0013), это правка десятка файлов ради одного поля. Вместо этого:

* Новый топик **`/voice/stt/utterance`** (`String`, JSON `{"utterance_id": "..."}`), публикуется `stt_node` для КАЖДОЙ принятой ReSpeaker-фразы, ПЕРЕД `/voice/stt/result` (тот же порядок гарантий, что уже используется для `/voice/stt/speaker` — публикатор один и тот же поток, публикации строго последовательны). В отличие от `/voice/stt/speaker` (публикуется только когда есть Yandex speaker_tag — на Vosk-фоллбэке пропускается), `/voice/stt/utterance` публикуется всегда: `dialogue_node` должен иметь id для каждой фразы, а не только для тех, где Yandex дал тег.
* Поле **`utterance_id`** добавлено в существующий JSON `/voice/speaker/result` (уже JSON: `is_known`/`speaker_id`/`name`/`confidence`/…) — аддитивно, все потребители (`mcp_server`, `vision_face_node`, `dialogue_node`, `rob_box_harness.encounter.voice_adapter`) читают через `.get()`, лишнее поле не ломает ни одного.
* То же поле — в `register`/`register_error` ack на том же топике (для PR-2).

### 2.4 Модуль-владелец: `core/utterance_speaker.py::UtteranceSpeakerRegistry`

`submit(utterance_id, result)` — вызывается из колбэка `/voice/speaker/result` (любой поток). `await resolve(utterance_id, timeout_sec)` — ждёт результат ИМЕННО этой фразы, опрашивая внутренний thread-safe словарь с интервалом 20мс; таймаут → `None` (unknown), не прошлое значение. Ring-buffer с TTL 30с и потолком 64 записей — без memory leak на долгой сессии.

`dialogue_node._apply_speaker_identity` теперь:

```python
if utterance_id:
    resolved = await self._utterance_speaker.resolve(
        utterance_id, self._speaker_resolve_timeout_sec
    )
    with self._speaker_lock:
        self._current_speaker = resolved if resolved is not None else {"is_known": False}
```

`_current_speaker` остаётся как «последний РАЗРЕШЁННЫЙ снимок» — но теперь его пишет только `_apply_speaker_identity`, после `resolve()`, никогда напрямую из `_on_speaker_result` в обход join'а. `_on_speaker_result` продолжает получать сырые сообщения и кладёт их и в `_current_speaker` (для диагностики/обратной совместимости чтения из других мест — `_current_acquaintance`, `_ask_identity_if_ambiguous`), и в `_utterance_speaker.submit()` — но решение «кто сейчас говорит для LLM» принимает только `resolve()` по конкретному id.

Ходы без новой фразы (`utterance_id=None`: babble-retry, DJ auto-переход, action-claim retry, synthetic-ретраи из guard'ов) не вызывают `resolve()` вообще — ждать нечего, гонка неприменима, читаем последний снимок как раньше. Это осознанно ограничивает скоуп фикса именно к сценарию issue (новая фраза от нового человека), не трогая ~15 остальных мест вызова `_dispatch_turn` без живой аудио-фразы.

### 2.5 Таймаут — 2.5с

Из логов issue #2829: биометрия — 0.6-1.9с в норме, редкий выброс ~50с сразу после старта (холодная модель). `2.5с` — запас ×1.3 над верхней границей нормы. Для выброса в 50с сознательно НЕ увеличиваем таймаут: лучше честный `unknown` через 2.5с, чем 50с тишины в диалоге (или, что было раньше, имя случайного предыдущего собеседника). Параметр `speaker_resolve_timeout_sec` (declare_parameter, дефолт 2.5) — можно перекрутить в конфиге без пересборки.

## 3. Источники: было → стало

| Источник | Было | Стало | Обоснование |
|---|---|---|---|
| `_current_speaker` | Читается после блокирующего `sleep(0.30)`, источник гонки | Пишется только из `_apply_speaker_identity` после `resolve(utterance_id)`; таймаут → явный `{"is_known": False}` | Единственный писатель = единственная точка правды для решения «кто сейчас говорит» |
| `speaker_context`/`speaker_tag` (`/voice/stt/speaker`) | Yandex per-session tag → создание/подтверждение профиля (`_handle_speaker_turn`, `SpeakerTracker`) | Без изменений — это другая задача (scoping профиля по session-tag, эпитеты), не «имя в промпте LLM». Явно понижено до диагностики/scoping-роли в этом ADR | Название в user-prefix `[Spkr:...]`/`[Speaker:unknown]` и раньше бралось только из `_current_speaker`/`sp`, а не из `speaker_context` (см. `_apply_speaker_identity`: `speaker_context` используется только как «уже был какой-то контекст, не подставлять `[Speaker:unknown]` дважды») |
| `_speaker_by_text` | Сопоставление тега с фразой по точному тексту, pop-on-read в `_on_stt` | Без изменений — механизм доставки `speaker_tag` от `/voice/stt/speaker` до `_on_stt` (см. выше), не механизм принятия решения о фразе | Тот же аргумент — не участвует в «кто сказал», участвует в scoping-профиле |
| `_speaker_tracker` | Подтверждение стабильности Yandex tag (2+ фразы подряд) для профиля | Без изменений | Защита профиля от нестабильных session-тегов Yandex, ортогональна biometric-identity |

Единственный источник правды «кто сказал ЭТУ фразу» для LLM-промпта — `UtteranceSpeakerRegistry`, доступный через `_apply_speaker_identity`/`_current_speaker`-после-resolve. Остальные три источника — про другую задачу (session-scoped Yandex-профиль с эпитетами) и намеренно не тронуты, чтобы не расширять blast radius PR-1 за пределы гонки из issue (ADR-0013).

## 4. Что не входит в PR-1 (PR-2, тот же issue)

* `speaker_id_node._on_register_request` — регистрация «следующей фразой кого угодно» без срока (`_pending_register_name`).
* `speaker_id_node._apply_growth_session` — рост галереи не проверяет, что реплика опознана как владелец сессии (пропускает молчаливого незнакомца).

Оба привязываются к тому же `utterance_id` (поле уже проброшено в `_do_register`/`_publish_result` в PR-1 как задел) — см. PR-2.

## 5. Риски / что не проверено

* Порядок `/voice/stt/utterance` → `/voice/stt/result` гарантирован только тем, что оба публикуются из одного потока `stt_node` последовательно (`_publish_utterance_id` вызывается прямо перед `_publish_speaker`/`publish_result`). ROS2 DDS с `reliable`+`transient_local` для одного паблишера/одного подписчика обычно сохраняет порядок публикации, но это не формальная гарантия протокола на 100% всех DDS-реализаций/QoS. Не проверено на роботе под нагрузкой (несколько фраз подряд, барж-ин).
* Telegram-путь (`from_tg=True`) вызывает `_pop_pending_utterance_id()` наравне с голосовым — теоретическая гонка «телеграм-сообщение съело id голосовой фразы, которая пришла в этот же момент», но `from_tg` ветка не использует `utterance_id` (сразу `[TG]`-префикс, `_apply_speaker_identity` не вызывается), так что съеденный id просто пропадает впустую — фраза, для которой он был выставлен, получит `utterance_id=None` и упадёт в fallback-ветку (читает последний снимок без ожидания). Не e2e-проверено, вероятность события низкая (нужно совпадение по времени голоса и Telegram).
* E2E на роботе (акты со сменой собеседника, #2818/#2798/#2757 регрессия) НЕ прогонялся координатором — только юнит-тесты (raw в PR).
