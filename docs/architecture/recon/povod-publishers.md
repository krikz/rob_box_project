# Разведка: «кто уже публикует события, на которые мог бы подписаться повод»

> Артефакт architect по kanban t_a98a25e1 (issue #2536, ADR-0103 «Повод»).
> Факты — только из текущего кода, без интерпретации и без предложений.

## 0. Постановка (из задачи)

- Полный список **подписок** `dialogue_node.py` в районе `:638-782` — кому нода уже слушает.
- Полный список **publishers** в `dialogue_node.py` и в смежных нодах восприятия
  (`perception_*`, `vision_*`, `telegram_node`) — кто что публикует, на какой топик, с каким триггером/частотой.
- Цель: дать карту «событие X на топике Y — повод мог бы слушать Y».
- Только разведка. Решений не предлагать.

## 1. Подписки `dialogue_node.py` (район `:638-782`)

Подтверждено ripgrep-парсером по `src/rob_box_voice/rob_box_voice/dialogue_node.py`.
Всего в файле 19 `create_subscription(...)`; ниже перечислены все, с координатами
и msg-типом. **Ни одной подписки на `/perception/*`, `/vision/*`, `/robot/*`,
`/sensors/*` — ни в этом блоке, ни во всём файле.**

| file:line | топик | msg-тип | назначение (из комментария в файле) |
|---|---|---|---|
| dialogue_node.py:645 | `/voice/stt/result` | `String` | единственный внешний «вход в ход» (пользовательская речь, вейк-слово). |
| dialogue_node.py:657 | `/dialogue/control` | `String` | ADR-0066 §6.1 — оператор ставит личность в pause/resume. |
| dialogue_node.py:668 | `/voice/command/feedback` | `String` | issue #1279 — feedback после движения/статуса. |
| dialogue_node.py:676 | `/voice/stt/speaker` | `String` | issue #1077 — speaker_tag от Yandex (отдельный топик). |
| dialogue_node.py:682 | `/voice/speaker/result` | `String` | голосовая биометрия (resemblyzer d-vector). |
| dialogue_node.py:696 | `/voice/speaker/epithet_request` | `String` | issue #1787 — speaker_id_node просит придумать кличку. |
| dialogue_node.py:701 | `/audio/vad` | `Bool` | VAD-сигнал (barge-in/echo-suppression). |
| dialogue_node.py:703 | `/voice/tts/finished` | `String` | конец TTS-чанка. |
| dialogue_node.py:709 | `/voice/tts/current_voice` | `String` | issue #1219 — LLM voice selection. |
| dialogue_node.py:717 | `/voice/tts/provider_state` | `String` | issue #1229 — фактический TTS-провайдер после фолбека. |
| dialogue_node.py:722 | `/voice/generated_music/state` | `String` | issue #1392 follow-up — состояние сген. музыки. |
| dialogue_node.py:737 | `/voice/music/state` | `String` | факт play/stop музыки (точное равенство `playing/idle` в audio_node). |
| dialogue_node.py:743 | `/voice/tts/batch_complete` | `String` | issue #980 — конец батча (rap/poetry). |
| dialogue_node.py:754 | `/voice/tts/batch_registered` | `String` | issue #992 — pre-register `batch_id`. |
| dialogue_node.py:758 | `/voice/sound/state` | `String` | состояние проигрываемого sfx. |
| dialogue_node.py:766 | `/odom` | `Odometry` | лёгкий снимок {x,y,θ} для LLM-контекста `<position>` (только `_pose_snapshot`). |
| dialogue_node.py:769 | `/voice/dj_mode` | `String` | DJ-режим, лямбда `lambda m: self._dj.handle_message(m.data)`. |
| dialogue_node.py:777 | `/voice/music/form` | `String` | issue #2461 — структурный конец прохода формы (НАРОЧНО отдельно от `/voice/music/state`). |
| dialogue_node.py:789 | `/mcp/tools` | `String` | latched (TRANSIENT_LOCAL) — каталог инструментов от mcp_server. |

Дополнительные подписки вне блока `:638-782` (для полноты, в этом же файле):
- нет ни одной на `/perception/*`, `/vision/*`, `/robot/*`, `/sensors/*` —
  проверено по `grep -c` всех вхождений (0 штук).
- единственное, что пересекается с восприятием — `/odom` (одометрия,
  используется только как snapshot, не как событие).

## 2. Publishers `dialogue_node.py` (все, для полноты)

| file:line | топик | msg-тип | QoS / назначение |
|---|---|---|---|
| dialogue_node.py:521 | `/harness/task_events` | `String` | depth=10. task lifecycle monitoring (W7b scheduler `on_event`). |
| dialogue_node.py:570 | `/voice/dialogue/response` | `String` | depth=10. Ответ бота → tts_node + (legacy) telegram_node. |
| dialogue_node.py:577 | `AVATAR_COMMAND_TOPIC` = `/avatar/command` | `String` | depth=10 RELIABLE. ADR-0066 §6.3 legacy fallback. |
| dialogue_node.py:579 | `/voice/dialogue/state` | `String` | depth=10. Состояние DSM (`IDLE/LISTENING/...`). |
| dialogue_node.py:580 | `/voice/sound/trigger` | `String` | depth=10. Триггер sfx. |
| dialogue_node.py:593 | `/voice/tts/control` | `String` | depth=10. Контроль TTS (pause/resume/barge). |
| dialogue_node.py:605 | `/voice/dialogue/barge_in_policy` | `String` | TRANSIENT_LOCAL, issue #1734 — latched SSoT для barge-in. |
| dialogue_node.py:620 | `/mcp/music_cleanup` | `String` | depth=10. issue #935 — fire stop_music_on_session_end. |
| dialogue_node.py:635 | `/mcp/music_fallback` | `String` | depth=10. issue #1016 — пустой ответ LLM на музыку → top-rated трек. |
| dialogue_node.py:655 | `/dialogue/control_ack` | `String` | RELIABLE. ACK для `/dialogue/control`. |
| dialogue_node.py:685 | `/voice/speaker/register` | `String` | depth=10. Регистрация нового спикера. |
| dialogue_node.py:691 | `/voice/speaker/observe` | `String` | depth=10. issue #1787 — реплики опознанного спикера. |
| dialogue_node.py:699 | `/voice/speaker/epithet` | `String` | depth=10. issue #1787 — кличка от LLM. |
| dialogue_node.py:6040 | `/voice/dj_mode` | `String` | depth=10. Публикация переключения режима DJ. |

Примечание: `create_publisher` встречается в файле 86 раз по ripgrep, но это
вспомогательные внутренние классы (DJ/EventBus/etc.); реальных узлов-publishers
(создаются через `rclpy.create_publisher` или `self.create_publisher`) — 14
штук, все в таблице.

## 3. Publishers смежных нод восприятия и телеграма

### 3.1. `perception_bridge.py` — `src/rob_box_perception/rob_box_perception/perception_bridge.py`

| file:line | топик | msg-тип | триггер / частота |
|---|---|---|---|
| perception_bridge.py:70 | `/sensors/data` | `String` (JSON) | timer `SENSOR_READ_PERIOD=0.1` → 10 Hz (если UART доступен; иначе — stub-режим, timer не создаётся, см. `:86-89`). |
| perception_bridge.py:73 | `/perception/health` | `String` (JSON snapshot) | timer `HEALTH_PERIOD=1.0` → 1 Hz. |

### 3.2. `vision_hailo_node.py` — `src/rob_box_perception/rob_box_perception/vision_hailo_node.py`

| file:line | топик | msg-тип | триггер / частота |
|---|---|---|---|
| vision_hailo_node.py:155 | `/vision/hailo/events` (default; параметр `output_topic`) | `VisionEventMsg` (`rob_box_perception_msgs/VisionEvent`) | timer period = `max(0.1, stub_period_sec/4.0)`. В stub-режиме — `stub_period_sec=2.0` (дефолт); в real-режиме — на каждый кадр с камеры `input_topic='/oak/rgb/image_raw/compressed'`. Если `publish_when_no_input=True` (дефолт) — публикует даже без входных кадров. |

### 3.3. `context_aggregator_node.py` — `src/rob_box_perception/rob_box_perception/context_aggregator_node.py`

(исходник агрегатора; в тестах упоминается как «VisionEventsAggregator»)

| file:line | топик | msg-тип | триггер / частота |
|---|---|---|---|
| context_aggregator_node.py:244 | `/perception/context_update` | `PerceptionEvent` (см. `rob_box_perception_msgs/msg/PerceptionEvent.msg`) | агрегатор: подписан на `/vision/hailo/events` (`:225-228`), `/perception/vision_context` (`:134-137`), `/rtabmap/localization_pose` (`:142-145`), `/odom` (`:150-153`), `/dynamic_joint_states` (`:166-169`), `/rosout` (`:182-185`), `/voice/stt/result` (`:190-193`), `/voice/dialogue/response` (`:198-201`), `/voice/command/intent` (`:206-209`), `/voice/command/feedback` (`:214-217`); публикует при изменении (`:599`). |

Состав `PerceptionEvent` (по `.msg`):
- `stamp`, `pose`, `velocity`, `is_moving`,
- `battery_voltage`, `temperature`,
- `apriltag_ids`,
- `system_health_status`, `health_issues`, `current_time_human`,
  `time_period`, `time_context_json`, `internet_available`,
- `active_nodes`, `failed_nodes`, `missing_nodes`,
- `equipment_summary_json`, `mapping_mode`,
- `memory_summary`, `speech_summaries`, `robot_response_summaries`,
  `robot_thought_summaries`, `vision_summaries`, `system_summaries`,
- `vision_event_count`, `vision_events_json` (агрегат VisionEvent из `/vision/hailo/events`).

### 3.4. `telegram_node.py` — `src/rob_box_telegram/rob_box_telegram/telegram_node.py`

| file:line | топик | msg-тип | триггер |
|---|---|---|---|
| telegram_node.py:154 | `/voice/stt/result` | `String` (RELIABLE) | handler текстового сообщения из Telegram: `:309 self._stt_pub.publish(m)`. Не STT как таковой — человек печатает в чат, его текст вбрасывается в тот же канал, что и распознанная речь. |
| telegram_node.py:155 | `/voice/dialogue/response` | `String` (RELIABLE) | публикуется из reply-пути — `:364 self._response_pub.publish(m)`. |
| telegram_node.py:162 | `AVATAR_COMMAND_TOPIC` = `/avatar/command` | `String` (RELIABLE, depth=10) | операторская команда из Telegram: `:334 self._avatar_command_pub.publish(m)`. |
| telegram_node.py:191 | `/cmd_vel_web` | `Twist` (RELIABLE) | движение из Telegram: `:410 self.cmd_vel_pub.publish(twist)`. Гейтится `with_floor(Floor.TELEOP, ...)`. |
| telegram_node.py:192 | `/voice/tts/request` | `String` (RELIABLE) | отправка реплики из Telegram в TTS: `:397 target.publish(m)` (target = tts_pub или response_pub в зависимости от `supervisor.mode`). Гейтится `with_floor(Floor.VOICE, ...)`. |
| telegram_node.py:206 | `/avatar/voice_in` | `AudioData` | AV-23 (issue #1915) — рация из Telegram, PCM-чанки: `:425 self._voice_in_pub.publish(msg)`. |
| telegram_node.py:209 | `/voice/sound/stop` | `String` | AV-23 — явный STOP после рации: `:437 self._voice_stop_pub.publish(msg)`. |

## 4. Кто публикует «про-человека»-события в восприятии (сводка)

(прочерк = топика нет в этом ноде)

| событие / «факт про человека» | `perception_bridge` | `vision_hailo_node` | `context_aggregator_node` | `telegram_node` |
|---|---|---|---|---|
| датчики (battery, temp, …) | `/sensors/data` 10 Hz | — | агрегирует как `battery_voltage/temperature` | — |
| health-статус | `/perception/health` 1 Hz | — | агрегирует как `system_health_status` | — |
| «вижу человека/объект» (VisionEvent) | — | `/vision/hailo/events` (default 2 с stub / per-frame real) | агрегирует как `vision_events_json` | — |
| объединённый контекст (агрегат всего) | — | — | `/perception/context_update` | — |
| текст пользователя (не голос) | — | — | подписан | `/voice/stt/result` (text → тот же канал) |
| «Денис написал/сказал» — повод `meeting` | — | — | слышит через `/voice/stt/result` | публикует в `/voice/stt/result` |
| команда оператора | — | — | подписан на `/voice/command/intent` | публикует в `/avatar/command` |
| команда из Telegram (без голоса) | — | — | — | `/voice/tts/request` (→ TTS напрямую) |
| рация (PCM-поток из TG) | — | — | — | `/avatar/voice_in` |

## 5. Сводка формата «файл:строка — топик/поле — частота/триггер»

(свернуто для issue-комментария)

```
dialogue_node.py:645   /voice/stt/result                    (sub) — единственный «вход в ход»
dialogue_node.py:657   /dialogue/control                    (sub) — ADR-0066 §6.1 pause/resume
dialogue_node.py:668   /voice/command/feedback              (sub) — #1279
dialogue_node.py:676   /voice/stt/speaker                   (sub) — #1077 speaker_tag
dialogue_node.py:682   /voice/speaker/result                (sub) — голосовая биометрия
dialogue_node.py:696   /voice/speaker/epithet_request       (sub) — #1787
dialogue_node.py:701   /audio/vad                           (sub) — VAD
dialogue_node.py:703   /voice/tts/finished                  (sub) — конец TTS-чанка
dialogue_node.py:709   /voice/tts/current_voice             (sub) — #1219
dialogue_node.py:717   /voice/tts/provider_state            (sub) — #1229
dialogue_node.py:722   /voice/generated_music/state         (sub) — #1392
dialogue_node.py:737   /voice/music/state                   (sub) — play/idle (точное равенство)
dialogue_node.py:743   /voice/tts/batch_complete            (sub) — #980
dialogue_node.py:754   /voice/tts/batch_registered          (sub) — #992
dialogue_node.py:758   /voice/sound/state                   (sub) — состояние sfx
dialogue_node.py:766   /odom                                (sub) — snapshot {x,y,θ} для LLM
dialogue_node.py:769   /voice/dj_mode                       (sub) — DJ handle_message
dialogue_node.py:777   /voice/music/form                    (sub) — #2461 (НАРОЧНО отдельно)
dialogue_node.py:789   /mcp/tools                           (sub) — latched TRANSIENT_LOCAL
```

```
dialogue_node.py:521   /harness/task_events                 (pub, depth=10) — task lifecycle
dialogue_node.py:570   /voice/dialogue/response             (pub, depth=10)
dialogue_node.py:577   /avatar/command                      (pub, depth=10 RELIABLE) — ADR-0066 §6.3 legacy
dialogue_node.py:579   /voice/dialogue/state                (pub, depth=10)
dialogue_node.py:580   /voice/sound/trigger                 (pub, depth=10)
dialogue_node.py:593   /voice/tts/control                   (pub, depth=10)
dialogue_node.py:605   /voice/dialogue/barge_in_policy      (pub, TRANSIENT_LOCAL) — #1734 latched
dialogue_node.py:620   /mcp/music_cleanup                   (pub, depth=10) — #935
dialogue_node.py:635   /mcp/music_fallback                  (pub, depth=10) — #1016
dialogue_node.py:655   /dialogue/control_ack                (pub, RELIABLE)
dialogue_node.py:685   /voice/speaker/register              (pub, depth=10)
dialogue_node.py:691   /voice/speaker/observe               (pub, depth=10) — #1787
dialogue_node.py:699   /voice/speaker/epithet               (pub, depth=10) — #1787
dialogue_node.py:6040  /voice/dj_mode                       (pub, depth=10)
```

```
perception_bridge.py:70   /sensors/data              (pub, depth=10) — timer 10 Hz (SENSOR_READ_PERIOD=0.1)
perception_bridge.py:73   /perception/health         (pub, depth=10) — timer 1 Hz (HEALTH_PERIOD=1.0)
vision_hailo_node.py:155  /vision/hailo/events       (pub, depth=10, VisionEventMsg)
                                                  — параметр output_topic, default '/vision/hailo/events'
                                                  — stub_period_sec=2.0 (timer period = max(0.1, stub/4))
                                                  — в real-режиме — на каждый кадр input_topic
context_aggregator_node.py:244 /perception/context_update (pub, depth=10, PerceptionEvent)
                                                  — агрегат: vision/pose/odom/sensors/health/time/memory
                                                  — публикуется при изменении агрегата
telegram_node.py:154  /voice/stt/result             (pub, RELIABLE) — handler текста из TG (не STT)
telegram_node.py:155  /voice/dialogue/response      (pub, RELIABLE) — TG reply-путь
telegram_node.py:162  /avatar/command               (pub, RELIABLE, depth=10) — TG operator → avatar
telegram_node.py:191  /cmd_vel_web                  (pub, RELIABLE, Twist) — TG движение (with_floor TELEOP)
telegram_node.py:192  /voice/tts/request            (pub, RELIABLE) — TG реплика в TTS (with_floor VOICE)
telegram_node.py:206  /avatar/voice_in              (pub, AudioData) — AV-23 рация из TG
telegram_node.py:209  /voice/sound/stop             (pub, RELIABLE) — AV-23 STOP после рации
```

## 6. Сводка для комментария в issue #2536 (≤5 строк по формату)

«Восприятие публикует X на топике Y — повод мог бы слушать Y»:

1. **`/perception/context_update`** (PerceptionEvent, агрегатор): `context_aggregator_node.py:244` — факт «всё о мире прямо сейчас» (pose, vision_events_json, system_health, time_period, speech_summaries, robot_response_summaries, vision_summaries и т.д.); триггер — при изменении агрегата.
2. **`/vision/hailo/events`** (VisionEvent): `vision_hailo_node.py:155` — сырые структурные события зрения; периодич. `stub_period_sec=2.0` в stub-режиме, на каждый кадр в real; параметр `output_topic` (default `/vision/hailo/events`).
3. **`/perception/health`** (String JSON snapshot): `perception_bridge.py:73` — health-status/age/degraded, timer 1 Hz.
4. **`/sensors/data`** (String JSON): `perception_bridge.py:70` — battery/temperature и пр., timer 10 Hz (SENSOR_READ_PERIOD=0.1).
5. **`/voice/stt/result`** (String) — НЕ из восприятия, а из **telegram_node.py:154** (текст от оператора, вбрасывается в тот же топик, что и распознанная речь), плюс stt_node (распознанная речь) и command_node для тестов.

(Все координаты — `git log` origin/develop @ ff4719798.)

## 7. Что эта разведка НЕ покрывает

- Решения «можно ли заговорить» — не рассматривается (это ADR-0103 §3+).
- Куда публиковать повод (новый топик или в существующий) — не рассматривается.
- Контракт payload — не рассматривается.
- Этические/UX-аспекты (как часто можно «заговаривать первым») — не входит в задачу.
- Топики `/avatar/ptt/result`, `/avatar/stt/result` (упомянуты в комментариях
  dialogue_node.py:651-654) — здесь не исследовались, на текущий момент в
  `dialogue_node.py` подписок нет.
