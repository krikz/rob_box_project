# 06. Дайджест архитектурных аудитов для диалоговой системы

Источники (все в `docs/research/dialog-v2/raw/`):

- статика: `architecture-static/` — CI `G: Architecture Audit`, run 36918223549, develop, 01.10.2026;
- рантайм: `architecture-runtime/architecture-local-runtime/` — `L: Architecture Audit`, run 36689419795, 30.09.2026, живой робот (Main Pi 10.1.1.20 контейнер nav2, Vision Pi 10.1.1.21 контейнер oak-d). Снимок рантайма склеен из двух (`runtime.json`, поле `captures`). Каталог исходников в этом артефакте старше статики на сутки (DialogueNode там 9872 LOC, в статике 9401).

Пометка: `ownership.yml` в репо пуст, ADR-0145 в самих аудитах не считается (он живёт в `scripts/lint/`), поэтому раздел про бюджет сравнивает по `class_budget_baseline.json` отдельно и явно помечен.

## 1. Как устроен аудит

| Этап | Скрипт | Что меряет |
|---|---|---|
| inventory | `tools/architecture_audit.py` | контейнеры (33), пакеты (17), py-файлы (442), классы (717), ROS-интерфейсы (249, уникальных 131), launch-узлы (51), статические топики (130) |
| structural findings | `tools/architecture_findings.py` | правила ниже, все со `severity: review` (автоматического вердикта нет) |
| каталоги | `tools/architecture_catalog.py` | node-catalog, topic-catalog |
| class metrics | `tools/architecture_class_metrics.py` | LOC, методы, WMC (сумма CC), max CC, TCC, LCOM4 (`responsibilities`), ROS-эндпоинты, test_refs (прокси, не покрытие); god-class: `WMC>=47 and TCC<1/3 and LOC>=500` (константы `GOD_WMC/GOD_TCC/GOD_LOC`, ~стр. 46-49); большой файл `>=2000` строк |
| runtime snapshot/merge | `tools/architecture_runtime_snapshot.py`, `..._merge.py` | `ros2 topic info -v` по каждому топику с каждого Pi (workflow `L-Architecture Audit.yml`, стр. ~121-153) |
| runtime diff | `tools/architecture_runtime_diff.py` | сравнение имён статических и рантайм топиков; сравнение с `architecture/runtime-baseline.json` |
| runtime findings | `tools/architecture_runtime_findings.py` (`analyze`, стр. 112-157) | см. ниже |
| graph | `tools/architecture_runtime_graph.py` | mermaid-карта контейнеров и топиков |

Правила статических findings (`architecture_findings.py`, все `review`):
- `multiple_publishers`: у топика больше одного статического писателя;
- `publish_without_static_subscriber` и `subscribe_without_static_publisher`: непарные интерфейсы;
- `semantic_topic_name_review`: топик с именем `person|user|speaker|identity|face|current_user|current_person` (регэксп `GENERIC_SEMANTIC_NAMES`) требует явного контракта и владельца;
- `architectural_duplicate_class_name`, `multiple_node_classes_in_file`, `node_package_without_static_launch`.

Правила рантайма (`architecture_runtime_findings.py`): `dead_input` (читатель без писателя), `dead_output` (писатель без читателя), `multiple_writers` (больше одного узла-писателя), `duplicate_publishers` (один узел держит несколько publisher'ов на топик), `type_mismatch`, `qos_incompatible` (BEST_EFFORT -> RELIABLE; VOLATILE -> TRANSIENT_LOCAL). Инфраструктурные топики и узлы-хелперы (tf2 listener, launch) отбрасываются. `scope=repo`, если наш узел участвует или топик объявлен в нашем коде, иначе `external`. Снимок показывает только то, что было подключено в момент съёма. Исход сравнивается с `architecture/runtime-baseline.json` (known-ключи вида `dead_input|/voice/state|`), новые помечены 🆕.

Итоги сводок: статика, findings 87 (multiple_publishers 13, unpaired 62, semantic_topic_names 8, duplicate_class_names 0, launch_review 4) (`architecture-static/findings.md:7-11`). Рантайм, findings 121, repo-scope 40, new 2, resolved 9, dead_input 25, dead_output 77, multiple_writers 18, duplicate_publishers 1, type_mismatch 0, qos_incompatible 0 (`runtime-findings.md:9-17`).

## 2. Узлы диалогового контура на живом роботе

Все ноды диалога стоят на Vision Pi. Контейнер voice-assistant держит 10 нод: audio_node, command_node, dialogue_node, led_node, mcp_server, sound_node, speaker_id_node, stt_node, tts_node, voice_animation_player (`runtime-graph.mmd:81-91`). Отдельные контейнеры Vision Pi: avatar-supervisor, avatar-arbiter, rob-box-quest (quest_node), telegram-bot (telegram_node), vision-face, vision-hailo, led-matrix. На Main Pi: perception (context_aggregator, health_monitor, perception_bridge) и teleop (joystick_control_node). Пересечение хостов: context_aggregator (Main Pi) шлёт `/perception/context_update` и `/voice/sound/trigger`-соседство в voice-assistant (Vision Pi); joystick_control_node (Main Pi) пишет `/voice/tts/request` (`runtime-graph.md:36,46`).

`startup_greeting_node` объявлен в статике (`node-catalog.md`), но в рантайме не запущен: его нет в списке `nodes` в `runtime.json`.

Число publisher/subscriber топиков на ноду (из `runtime.json`, без `/parameter_events` и `/rosout` учитывать нельзя: они входят в счёт publishers только для справки; ниже без них):

| Нода | Публикует | Подписана | Заметное |
|---|---|---|---|
| dialogue_node | `/avatar/command`, `/dialogue/control_ack`, `/harness/task_events`, `/mcp/execute`, `/mcp/music_cleanup`, `/mcp/music_fallback`, `/voice/dialogue/{barge_in_policy,response,state}`, `/voice/dj_mode`, `/voice/sound/trigger`, `/voice/speaker/{epithet,merge,observe,register}`, `/voice/tts/control` (16) | 22 топика: vad, `/dialogue/control`, `/mcp/{result,tools}`, odom, hailo events, command feedback, music/dj/sound state, speaker, `/voice/stt/{result,speaker,utterance}`, tts finished/batch/provider_state | вершина графа: 16 + 22 |
| stt_node | `/avatar/ptt/result`, `/avatar/stt/result`, `/voice/sound/trigger`, `/voice/stt/{result,speaker,state,utterance}`, `/voice/tts/control`, `/voice/tts/request` | `/audio/{quest_in,quest_wake,speech_audio}`, barge_in_policy, `/voice/tts/state` | пишет напрямую в TTS и sound, минуя диалог |
| tts_node | avatar preview/tts audio, `/tars1/text`, `/voice/audio/speech`, `batch_complete`, `finished`, `provider_state`, `state`, `voices` | `/avatar/tts/{control,request}`, `/voice/current_dialogue_id`, `/voice/dialogue/response`, `/voice/tts/{control,request,set_provider,set_voice}` | 5 писателей в `/voice/tts/request` |
| audio_node | `/audio/{audio,direction,speech_audio,state,vad}`, `/voice/tts/control` | `/voice/music/state`, `/voice/tts/state` | |
| sound_node | `/animations/trigger`, `/voice/generated_music/state`, `/voice/sound/state` | `/avatar/voice_in`, `/voice/sound/{play_file,stop,trigger}` | |
| speaker_id_node | `/voice/speaker/{epithet_request,result}` | `/audio/speech_audio`, `/voice/speaker/{epithet,merge,observe,register,rename}`, `/voice/tts/finished` | |
| command_node | `/cmd_vel_voice`, `/voice/command/{feedback,intent}` | `/voice/dialogue/state`, `/voice/stt/result` | |
| mcp_server | `/mcp/{result,tools}`, `/voice/{animation/request,dj_mode,generated_music/state,music/form,music/state,sound/play_file,sound/stop,sound/trigger}`, `/voice/speaker/{register,rename}`, `/voice/tts/{batch_complete,batch_registered,current_voice,request,set_provider}`, `/avatar/tars/panel_request` (19) | `/mcp/{execute,music_cleanup,music_fallback}`, `/avatar/tars/panel_data`, `/battery_state`, `/odom`, `/perception/context_update`, `/voice/current_dialogue_id`, `/voice/dj_mode`, `/voice/speaker/result`, `/voice/tts/{finished,provider_state}` | `/voice/music/state` единственный TRANSIENT_LOCAL |
| avatar_supervisor | `/avatar/{command_result,preview_voice/*,tars/panel_data,tars/panel_request,tars/panel_url,tts/request}`, `/dialogue/control`, `/voice/tts/{request,set_voice}` | `/avatar/{command,preview_voice,ptt/result,set_voice,set_voice_mode,stt/result,tars/panel_request,voice_pipeline}` | подписан на собственный `/avatar/tars/panel_request` |
| avatar_arbiter | `/avatar/state`, `/teleop_lock` | `/device/snapshot`, `/odom`, `/teleop_heartbeat`, `/voice/dialogue/state` | сервисы `acquire_floor`, `release_floor`, `set_avatar_mode` |
| telegram_node | `/avatar/command`, `/cmd_vel_web`, `/voice/dialogue/response`, `/voice/sound/stop`, `/voice/stt/result`, `/voice/tts/request` | `/avatar/command_result`, `/rtabmap/grid_prob_map`, `/voice/dialogue/response` | подписан на собственный `/voice/dialogue/response` |
| quest_node | 13 топиков (`/audio/quest_{in,wake}`, `/avatar/*`, `/voice/sound/stop`, `/voice/tts/control`, cmd_vel) | 20 топиков | |
| context_aggregator | `/perception/context_update` | stt/result, dialogue/response, command intent/feedback, hailo events и т.д. | |
| vision_face, vision_hailo | `/perception/observations`, `/vision/hailo/events` оба | camera, vision_face слушает `/voice/speaker/result` | |
| voice_animation_player | `/panel_image`, `/voice_animation_player/status` | `/voice/animation/request`, `/voice/tts/state`, `.../load_animation` | |

Сервисы диалоговых нод (без parameter-сервисов): `/avatar_arbiter/{acquire_floor,release_floor,set_avatar_mode}`, `/voice_animation_player/{list_animations,pause,play,stop}`, `/supervisor/execute`, `/voice/set_auto_led`. У dialogue_node, mcp_server, tts_node, stt_node, speaker_id_node собственных сервисов нет: весь контроль идёт через `std_msgs/String`-топики (всего сервисов 401, из них основная масса параметрические). Экшены (11) все навигационные, диалоговых нет.

Типы: из 81 топика префиксов `/voice|/avatar|/dialogue|/mcp|/audio|/perception|/vision` 69 на `std_msgs/msg/String`, 7 на `audio_common_msgs/AudioData`, по одному на Int32, Bool, `PerceptionEvent`, `Observation`, `VisionEvent` (подсчёт по `runtime.json`, поле `topic_info`). То есть контракт диалога это JSON внутри String. `type_mismatch` и `qos_incompatible` равны 0.

### Топики с несколькими писателями (`runtime-findings.md:157-177`)

| Топик | Писатели | Читатели |
|---|---|---|
| `/voice/tts/request` | avatar_supervisor, joystick_control_node, mcp_server, stt_node, telegram_node (5) | tts_node |
| `/voice/tts/control` | audio_node, dialogue_node, quest_node, stt_node (4) | tts_node |
| `/voice/sound/trigger` | dialogue_node, health_monitor, mcp_server, stt_node (4) | sound_node |
| `/voice/sound/stop` | mcp_server, quest_node, telegram_node | sound_node |
| `/voice/stt/result` | stt_node, telegram_node | context_aggregator, command_node, dialogue_node |
| `/voice/dialogue/response` | dialogue_node, telegram_node | context_aggregator, telegram_node, tts_node |
| `/avatar/command` | dialogue_node, telegram_node | avatar_supervisor |
| `/voice/dj_mode` | dialogue_node, mcp_server | оба же (замкнутая петля) |
| `/voice/speaker/register` | dialogue_node, mcp_server | speaker_id_node |
| `/voice/generated_music/state` | mcp_server, sound_node | dialogue_node |
| `/voice/tts/batch_complete` | mcp_server, tts_node | dialogue_node |
| `/avatar/tars/panel_request` | avatar_supervisor, mcp_server | avatar_supervisor |
| `/avatar/preview_voice/{audio,error,result}` | avatar_supervisor, tts_node | quest_node |
| `/perception/observations` 🆕, `/vision/hailo/events` | vision_face, vision_hailo | dialogue_node, context_aggregator |

Все они RELIABLE/VOLATILE, кроме `/avatar/voice_in` (AudioData, BEST_EFFORT, писатель quest_node, читатель sound_node).

### Мёртвые входы и выходы (диалог)

Dead input (читатель без писателя), repo-scope (`runtime-findings.md:31-40`): `/avatar/tts/control` (tts_node), `/voice/current_dialogue_id` (mcp_server, tts_node), `/perception/vision_context` (context_aggregator), `/device/snapshot` (avatar_arbiter, quest_node), `/battery_state` (mcp_server), `/voice_animation_player/load_animation`. `/voice/state` из baseline ушёл (resolved).

Dead output (писатель без читателя), диалоговые (`runtime-findings.md:68-84`): `/animations/trigger` (sound_node), `/audio/audio`, `/audio/state` (audio_node), `/avatar/tts/error` (tts_node), `/avatar/wake_stream` (quest_node), `/dialogue/control_ack` и `/harness/task_events` (dialogue_node), `/voice/audio/speech` (tts_node), `/voice/stt/state` (stt_node), `/voice_animation_player/status`, `/perception/health`, `/sensors/data`, `/perception/observations` 🆕. Итого dead_output в scope repo 17 (из 77).

### Расхождения рантайма со статикой и baseline

- Resolved с baseline (`runtime-findings.md:19-28`): dead_input `/voice/state`; dead_output `/tts/speak`, `/voice/stt/request`, `/camera/camera/depth/image_rect_raw`; duplicate_publishers на `/panel_image`, `/voice/animation/request`, `/voice/generated_music/state`, `/voice/sound/stop`, `/voice/tts/set_provider`.
- Новые: `dead_output|/perception/observations` и `multiple_writers|/perception/observations` (оба 🆕).
- Статика/рантайм: статических топиков 130, рантайм 220, общих 102 (`runtime-diff.md:5-7`). Объявлены статически, но в рантайме отсутствуют: `/voice/tts/metrics`, `/audio/level`, `/audio/enable_reactive`, `/animation/audio_trigger`, `/voice/set_auto_led`, `/avatar_arbiter/*`, `/supervisor/execute`, `/tts_node/{get,set}_parameters`. Часть из них на деле существует как сервисы (`/avatar_arbiter/*`, `/supervisor/execute`, `/voice/set_auto_led`) — сравнение только по именам топиков, это ложные расхождения. `/voice/tts/metrics` осталась непроявленной.
- Только в рантайме (диалог): `/avatar/command`, `/avatar/command_result`, `/voice/music/state`, `/led_matrix/data`, `/voice_animation_player/{load_animation,status}`. Статика этого контура их не видит: `/voice/music/state` и `/avatar/command` не попали в `inventory` (динамические имена или нестандартная регистрация).
- Статика ошибочно считает пары непарными из-за границ контейнеров: 62 `unpaired_interfaces`, из них для диалога большинство `/avatar/*` и `/voice/*` (топики с quest_node, supervisor).

## 3. Findings, относящиеся к диалогу

### Статика (`architecture-static/findings.json`, 87 шт, все `review`)

| Тип | Диалоговые топики |
|---|---|
| `multiple_publishers` (13) | `/avatar/voice_in` (quest + telegram), `/voice/animation/request`, `/voice/dialogue/response`, `/voice/dj_mode`, `/voice/generated_music/state`, `/voice/sound/stop`, `/voice/sound/trigger`, `/voice/speaker/register`, `/voice/stt/result`, `/voice/tts/batch_complete`, `/voice/tts/control`, `/voice/tts/request`, `/voice/tts/set_provider` |
| `semantic_topic_name_review` (8) | `/voice/speaker/{epithet,epithet_request,merge,observe,register,rename,result}`, `/voice/stt/speaker`. Файлы: dialogue_node.py, speaker_id_node.py, stt_node.py, mcp_server.py, quest_node.py, tools/dialogue.py. Суть: имя `speaker` несёт разные смыслы (обнаружение, личность, текущий диктор), нужен явный контракт и владелец |
| `publish_without_static_subscriber` | `/audio/audio`, `/audio/state`, `/avatar/tts/error`, `/avatar/wake_stream`, `/avatar/ptt/result` (stt_node.py:584), `/dialogue/control_ack` (dialogue_node.py:909), `/harness/task_events` (dialogue_node.py:775), `/voice/audio/speech`, `/voice/stt/state` (stt_node.py:586), `/voice/tts/metrics` (tts_node.py:3824), `/perception/health`, `/sensors/data`, `/avatar/set_voice*`, `/avatar/voice_pipeline`, `/avatar/preview_voice` |
| `subscribe_without_static_publisher` | `/avatar/tts/control` (tts_node.py:1297), `/avatar/tts/request`, `/avatar/preview_voice/*`, `/voice/current_dialogue_id` (tools/dialogue.py:108, tts_node), `/voice/tts/set_voice` (tts_node.py:1332), `/dialogue/control` (dialogue_node.py:911), `/perception/vision_context` (context_aggregator_node.py:141), `/device/snapshot`, `/battery_state`, `/tars1/text` |
| `node_package_without_static_launch` (4) | rob_box_quest, rob_box_supervisor, rob_box_telegram, rob_box_teleop: ноды существуют, статических launch нет (их запускают compose-контейнеры) |

`architectural_duplicate_class_name` равен 0.

### Рантайм (`runtime-findings.json`)

Тип `dead_input` 25 (repo 6), `dead_output` 77 (repo 17), `multiple_writers` 18 (repo 17), `duplicate_publishers` 1 (внешний `/cmd_vel` у behavior_server), `type_mismatch` и `qos_incompatible` 0. Нового диалогового, кроме `/perception/observations`, нет.

### Метрики класса (findings из `class-metrics`)

148 классов без test_refs (из 717), 21 god-класс, 13 файлов >= 2000 строк (`class-metrics.md:8-13`).

## 4. Метрики классов диалоговых пакетов

Статика 01.10 (`architecture-static/class-metrics.md`). Агрегат по диалоговым пакетам (классов / суммарный LOC / суммарный WMC / god-классов): rob_box_voice 194 / 36498 / 4399 / 8, rob_box_mcp_tools 138 / 19172 / 2585 / 4, rob_box_harness 129 / 8899 / 1048 / 3, rob_box_quest 48 / 6670 / 952 / 2, rob_box_supervisor 27 / 5479 / 693 / 0 (AvatarSupervisor не помечен god из-за TCC 0.41), rob_box_llm 44 / 2371 / 326 / 0, rob_box_telegram 16 / 2126 / 280 / 2, rob_box_core 37 / 987 / 133 / 0 (подсчёт по `class-metrics.json`).

| Класс | Файл:строка | LOC | Методов | WMC | Max CC | TCC | Resp. (LCOM4) | ROS | Test refs |
|---|---|---:|---:|---:|---|---:|---:|---:|---:|
| DialogueNode | voice/dialogue_node.py:516 | 9401 | 210 | 1105 | 82 `_handle_result` | 0.04 | 8 | 41 | 88 |
| TTSNode | voice/tts_node.py:637 | 6652 | 112 | 707 | 28 `__init__` | 0.05 | 3 | 22 | 32 |
| AvatarSupervisor | supervisor/supervisor_node.py:452 | 2511 | 63 | 320 | 19 `_run_agent_sync` | 0.41 | 1 | 20 | 9 |
| STTNode | voice/stt_node.py:284 | 1773 | 42 | 223 | 22 `_recognize_yandex_phase` | 0.10 | 1 | 14 | 3 |
| SpeakerIdNode | voice/speaker_id_node.py:238 | 2091 | 38 | 204 | 15 `_publish_result` | 0.33 | 1 | 9 | 11 |
| AgentCore | harness/core/agent_core.py:732 | 1749 | 29 | 175 | 19 `process_input` | 0.26 | 1 | 0 | 39 |
| MCPServer | mcp_tools/mcp_server.py:355 | 1597 | 29 | 168 | 13 `__init__` | 0.08 | 6 | 13 | 10 |
| QuestNode | quest/quest_node.py:1649 | 1530 | 36 | 171 | 15 | 0.35 | 1 | 54 | 13 |
| QuestBridge | quest/quest_node.py:292 | 1278 | 59 | 170 | 13 | 0.15 | 2 | 0 | 11 |
| DJModeController | voice/core/dj_mode.py:289 | 1096 | 36 | 163 | 14 | 0.82 | 1 | 0 | 34 |
| MiniMaxTTSProvider | llm/providers/minimax_tts.py:511 | 890 | 21 | 133 | 39 `stream` | 0.56 | 2 | 0 | 20 |
| AudioNode | voice/audio_node.py:31 | 1138 | 24 | 129 | 16 `check_vad_and_doa` | 0.18 | 1 | 11 | 7 |
| SupervisorClient | telegram/supervisor_client.py:338 | 953 | 31 | 126 | 22 | 0.24 | 2 | 4 | 3 |
| TaskScheduler | voice/scheduler/task_scheduler.py:563 | 751 | 26 | 126 | 25 `update` | 0.27 | 1 | 0 | 12 |
| SpeakerDatabase | voice/utils/speaker_embeddings.py:439 | 830 | 27 | 99 | 10 | 0.54 | 2 | 0 | 18 |
| AvatarArbiter | supervisor/arbiter_node.py:341 | 1159 | 34 | 96 | 10 | 0.45 | 2 | 15 | 1 |
| SoundNode | voice/sound_node.py:33 | 630 | 20 | 87 | 13 | 0.22 | 1 | 8 | 4 |
| TelegramNode | telegram/telegram_node.py:87 | 552 | 20 | 81 | 13 | 0.08 | 8 | 11 | 5 |
| ContextAggregatorNode | perception/context_aggregator_node.py:75 | 621 | 18 | 80 | 18 `add_to_memory` | 0.12 | 1 | 12 | 2 |
| SpeakTextTool | mcp_tools/tools/dialogue.py:87 | 612 | 11 | 65 | 21 `execute` | 0.33 | 1 | 5 | 11 |
| MusicGuard | voice/core/music_guard.py:116 | 647 | 17 | 50 | 15 `evaluate` | 0.20 | 1 | 0 | 26 |
| CommandNode | voice/command_node.py | 377 | 20 | 46 | 7 | 0.06 | 2 | 6 | 1 |

Самые сложные методы по max CC в диалоговых пакетах: `DialogueNode._handle_result` 82, `MiniMaxTTSProvider.stream` 39, `TTSNode.__init__` 28, `TaskScheduler.update` 25, `MinimaxMusicClient.generate` 24, `_OpenAICompatibleProvider.stream` 22, `AsyncToolExecutor.execute_tools_parallel` 22, `SupervisorClient._parse_execute_response` 22, `STTNode._recognize_yandex_phase` 22, `ToolSliceAuthority.from_mapping` 21, `SpeakTextTool.execute` 21.

Самые «многоответственные» (LCOM4 >= 3): DialogueNode 8, TelegramNode 8, MCPServer 6, TTSNode 3 (`class-metrics.md`, раздел Most responsibilities).

Файлы >= 2000 строк (диалоговые): dialogue_node.py 9954, tts_node.py 7316, tools/music.py 7296, `rob_box_core/_tool_catalog_data.py` 6948, quest_node.py 3197, `core/dialogue_guards.py` 3063 (не класс, модуль-мусорка), ws_server.py 3044, supervisor_node.py 3026, agent_core.py 2690, speaker_id_node.py 2344, stt_node.py 2073, mcp_server.py 2058.

Сложные классы без test_refs в диалоге: `TurnSpeechGate` (voice/core/turn_speech_gate.py:62, WMC 20), `AsyncToolExecutor` (mcp_tools/async_executor.py:52, WMC 56, max CC 22), скриптовые классы из `rob_box_voice/scripts/` (SileroTTSGUI 61, TextNormalizer 23 и др.).

Динамика между снимками 30.09 -> 01.10: DialogueNode LOC 9872 -> 9401, WMC 1184 -> 1105, методов 218 -> 210 (файл 10468 -> 9954). TTSNode 6647 -> 6652, WMC 707 стабильно. Размер падает (идёт разборка DialogueNode по карточкам ADR-0145).

### Бюджет (ADR-0145) — вне аудита, по репо

Аудит ADR-0145 не считает. Правило гарда `scripts/lint/class_budget.py`: большой класс = WMC > 80 или методов > 40, shrink-only ratchet (`docs/adr/0145-class-budget-and-cc-scope.md:60-69`). В `scripts/lint/class_budget_baseline.json` (чекаут этого worktree) записаны: DialogueNode WMC 1125 / 210 методов, TTSNode 721 / 112, STTNode 228 / 42, SpeakerIdNode 209 / 38, AvatarSupervisor 321 / 63, AgentCore 174 / 29, MCPServer 171 / 29, QuestBridge 175 / 59, QuestNode 171 / 36, AudioNode 131 / 24, SupervisorClient 133 / 31, SoundNode 88 / 20, TelegramNode 81 / 20, AvatarArbiter 96 / 34. Аудит от 01.10 показывает значения ниже baseline в этом чекауте (DialogueNode 1105 против 1125, TTSNode 707 против 721, STTNode 223 против 228, AvatarSupervisor 320 против 321): гард требует опускать baseline при уменьшении, значит baseline на develop уже обновлён или будет. Все классы выше порога по факту нарушают бюджет и держатся только на «залогированном legacy».

## 5. ownership.yml: декларация против реальности

`architecture/ownership.yml` (7 строк) пуст: `nodes: {}`, `topics: {}`, `capabilities: {}`, `features: {}`, комментарий «Human-reviewed ownership map. Do not infer canonical ownership from names alone.» Ни один владелец состояния или топика диалога не объявлен. Следствия:

- 18 `multiple_writers` и 13 статических `multiple_publishers` нельзя назвать нарушением: проверить не с чем. Сообщение findings само говорит «verify whether ownership is intentionally multi-writer» (`architecture_findings.py`).
- 8 `semantic_topic_name_review` требуют «explicit semantic contract and owner» — их владелец тоже нигде не записан.
- Единственная фиксация известных проблем рантайма — `architecture/runtime-baseline.json` (source: run 36422274238 от 28.09, ключи `dead_*`, `multiple_writers`, `duplicate_publishers`, ссылка на карточки #3104-#3108). Это список принятых долгов, а не владение.
- Реальные «де-факто владельцы» по рантайму: `/voice/dialogue/state` пишет только dialogue_node, `/voice/tts/state` только tts_node, `/voice/music/state` только mcp_server, `/avatar/state` только avatar_arbiter. Состояние TTS при этом пишут через `/voice/tts/control` четыре ноды, а запросы озвучки пять (см. §2).

## 6. Десять главных выводов для нового витка диалога

1. Нет декларации владельцев: `architecture/ownership.yml` пуст (`nodes/topics/capabilities/features: {}`), все 18 multiple_writers и 8 semantic-имён висят без вердикта. Новая архитектура должна начать с заполнения этого файла.
2. Озвучку просят пять писателей в `/voice/tts/request`: avatar_supervisor, joystick_control_node (с Main Pi), mcp_server, stt_node, telegram_node (`runtime-findings.md:177`). Нужен единый владелец очереди TTS (узел-арбитр), остальные идут через него.
3. Управление TTS идёт через `/voice/tts/control` с четырьмя писателями: audio_node, dialogue_node, quest_node, stt_node (`runtime-findings.md:176`), то есть barge-in и стоп разбросаны по четырём нодам без общего протокола.
4. `/voice/sound/trigger` пишут dialogue_node, health_monitor, mcp_server, stt_node (`runtime-findings.md:172`); `/voice/sound/stop` ещё quest_node и telegram_node. Звуковые эффекты и музыка не имеют одного владельца.
5. Диалог и Telegram делят топики ответа и ввода: `/voice/stt/result`, `/voice/dialogue/response`, `/avatar/command` пишутся и dialogue_node, и telegram_node (`runtime-findings.md:161,170,174`); telegram_node подписан на собственный `/voice/dialogue/response`. Источник реплики (голос, Telegram, Quest) не отражён в схеме топика, а только в payload.
6. Контракт диалога это JSON в `std_msgs/String`: 69 из 81 диалоговых топиков (подсчёт по `runtime.json`), у dialogue_node, mcp_server, tts_node, stt_node нет ни одного собственного сервиса; типизированных сообщений нет (`type_mismatch` = 0 только потому, что тип везде один). Проверок схемы на уровне ROS нет.
7. Мёртвые концы в диалоге: `/avatar/tts/control`, `/voice/current_dialogue_id` (читают tts_node и mcp_server, писателя нет — `runtime-findings.md:35,39`), и выходы без читателя `/dialogue/control_ack`, `/harness/task_events` (dialogue_node), `/voice/stt/state`, `/voice/audio/speech`, `/avatar/tts/error`, `/avatar/wake_stream` (`runtime-findings.md:75-85`). Механизм подтверждения команд управления диалогом (`control_ack`) и события harness никому не нужны: либо подключить потребителя, либо удалить.
8. DialogueNode остаётся центром: LOC 9401, WMC 1105, max CC 82 `_handle_result`, TCC 0.04, 8 групп ответственности, 41 ROS-эндпоинт, 16 publisher'ов и 22 subscriber'а на живом роботе (`class-metrics.md:24`; `runtime.json`). Новый диалог не должен наращивать этот класс: бюджет ADR-0145 запрещает, но он держится на baseline.
9. Рядом ещё один god-узел TTSNode (6652 LOC, WMC 707, 112 методов, TCC 0.05) и god-модуль `core/dialogue_guards.py` 3063 строки; `rob_box_core/_tool_catalog_data.py` 6948 строк (`class-metrics.md:105-113`). Знание о тулах и guard'ах надо делить по ответственностям, а не складывать в один файл.
10. Контур диалога живёт в двух процессных зонах и на двух хостах без явной границы: voice-assistant (10 нод в одном контейнере), отдельно avatar-supervisor, quest, telegram, а context_aggregator и joystick на Main Pi (`runtime-graph.mmd:81-91`, `runtime-graph.md:36,46`). Ошибки сети между Pi ломают `/voice/tts/request` и `/voice/sound/trigger` из Main Pi. Нода `startup_greeting_node` объявлена статически, но в рантайме не запущена (нет в `nodes` в `runtime.json`) — мёртвая декларация.

Ограничения и честность: аудит показывает только то, что было подключено в момент съёма; рантайм-снимок на сутки старше статики и снят с реального робота, в нём нет диалоговых экшенов и нет проверки нод без публикаций в момент съёма. Метрики это review-кандидаты, не вердикты (`class-metrics.md:3`). Сопоставление с `class_budget_baseline.json` сделано по чекауту этого worktree, не по develop.
