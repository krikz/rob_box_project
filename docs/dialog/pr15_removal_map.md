# Карта удаления старого пути диалога (v1) для PR-15 (#3711) — состояние на 23.10.2026

Карточка PR-15 эпика #3683: [#3711](https://github.com/krikz/rob_box_project/issues/3711) (текст и комментарий про
джойстик teleop), хвосты из [#3704](https://github.com/krikz/rob_box_project/issues/3704) (PR-10b, удаление
`DialogueStateMachine`), [#3705](https://github.com/krikz/rob_box_project/issues/3705) (PR-11),
[#3706](https://github.com/krikz/rob_box_project/issues/3706) (PR-12a). План ADR:
[`docs/adr/0150-dialog-v2-session-command-event.md`](../adr/0150-dialog-v2-session-command-event.md) §11 PR-15, §12, §14 A4/A6/A8.
Парный документ — [`acceptance_2026-10-23.md`](acceptance_2026-10-23.md) (скелет приёмки PR-14).

Всё в документе — факты из кода `origin/develop @ 8178e2809` (23.10.2026). Команды и их вывод — в приложении. Код не
менялся, робот не использовался. Размеры «строк» — `wc -l` (физические строки, включая пустые и комментарии); «без
тестов» — как в карточке (ADR-AF-0013: формальный потолок ADR — 3000 строк или 50 коммитов, ориентир эпика — 600).
Оценки без измерения помечены словом «оценка».

## 1. Что в карточке устарело и что это меняет

1. **`core/turn.py` и `turn_speech_gate.py` не существуют** (`git ls-files` → пусто для обоих). `src/rob_box_dialog/rob_box_dialog/turn.py`
   (хост хода v2) — живой код v2, **удалять нельзя**. Из семейства `turn_*` в `rob_box_voice/core` есть `turn_speech.py`,
   `turn_stages.py`, `turn_tools.py`, `turn_reflexes.py`, `turn_context.py`, `turn_identity.py`.
2. **`prompt_sections.py` не существует** (`git ls-files | grep prompt_sections` → пусто); промпт v2 уже собирает
   `rob_box_dialog.knowledge.prompt_fragment()`.
3. **`dialogue_node.py` — 4700 строк** (в карточке 7 956; частично разобран ранее), `DialogueNode` — WMC 525, 112 методов.
4. **`/voice/stt/result` в коде нет** (grep по `src docker scripts architecture` вне тестов и документов → 0; остались
   упоминания в `scripts/lint/test_ownership_check.py`, `architecture/runtime-baseline.json` и в устаревшем
   `.planning/`). Пункт карточки про этот топик уже выполнен PR-2.
5. **`/voice/dialogue/state` при `dialog_engine: v2` никто не пишет**: единственный писатель — `dialogue_node.py:615`
   (`create_publisher(String, "/voice/dialogue/state")`); в `dialog_node.py`, `core/dialog_v2_ports.py`, `rob_box_dialog/*`
   такой публикации нет (grep → 0). Читатели (`led_node`, `command_node`, `quest_node`, `arbiter_node`, `aggregator`,
   `streams/voice_state.py`, `bridge_protocol.py`, `mock_robot.ts`) при v2 не получают ничего. Аналогично
   `/voice/dialogue/response` пишет только `dialogue_node` v1: `context_aggregator_node` (перцепция) и `tts_node`-вход при v2
   тоже без писателя. Это нужно учесть **до** включения v2 по умолчанию (PR-14), а не после удаления.
6. **В v2-хосте нет аналогов (grep по `dialog_node.py`, `core/dialog_v2_ports.py`, `rob_box_dialog/*` → 0)** для:
   встречи лицом `core/meeting.py` («главная фича» по слову Шифу 21.10 — приветствие знакомого после долгой разлуки),
   стартового приветствия `startup_greeting.py`, фида статуса ТАРС `tars_status_feed.py`, `speaker_profiles.py`
   (`SpeakerTracker`, `extract_speaker_name`, `format_speaker_context`). Удаляя `dialogue_node.py`, мы потеряем эти функции,
   если не перенести их или не принять решение их отбросить (решение Шифу, п. 7 D1).
7. **Промпты `master_prompt_compact.txt` и `telegram_chat_prompt.txt` читает и v2** (`dialog_node.py:92,100`,
   `config/dialog_node.yaml`), поэтому «`RULE #` = 0» меняет поведение живого v2, а не только v1 (15 вхождений: 8 + 7).
8. **Четыре модуля из карточки только частично v1**: `dialogue_guards.py` (2 функции использует `tts_node`, константу —
   `track_start_guard`), `speak_helpers.py` (3 функции использует `tts_node`), `dialogue_helpers.py`
   (`sanitize_speaker_name` использует `speaker_turn`, а через него — v2), `dialogue_text.py` (использует `stt_node`).
   Они сокращаются, а не удаляются.
9. **Флаг `dialog_engine` — отдельный объём**: 44 строки с `dialog_engine` вне тестов в 17 файлах кроме самого
   `rob_box_core/dialog_engine.py` (ноды, launch×2, yaml×4, скрипты запуска). После его снятия откат на v1 невозможен (ADR §16).

## 2. Как собрана карта (и её пределы)

- Граф импортов: AST всех `*.py` в `src/` (1 333 файла, 3 962 ребра). «Читатели (AST)» — файлы, которые импортируют модуль
  напрямую; реэкспорт через `__init__.py` не считается читателем. «grep» — `grep -rlw <имя>` по `src docker scripts
  architecture tests` (`*.py *.yaml *.yml *.sh *.cfg *.txt *.json *.ts`, `Dockerfile`), без файла-модуля; в grep попадают
  и комментарии, поэтому он даёт верхнюю границу.
- «Замкнутое v1-множество»: файлы, у которых **все** не-тестовые читатели лежат внутри множества, начиная с
  `dialogue_node.py`. Таких файлов 22 (8 089 строк с `dialogue_node.py`), плюс `conversation_history.py` (255 строк, читатель —
  только реэкспорт в `core/__init__.py`, который сам нигде не используется).
- Динамические загрузки (строковые `importlib`, `ament` entry points кроме `setup.py`, `launch`) графом не видны; их я
  искал grep'ом отдельно: `dialogue_node` в `setup.py:49` и двух launch-файлах (`voice_assistant.launch.py:116`,
  `voice_assistant_headless.launch.py:90`).
- Тесты считаются по AST-импорту. Тесты, которые только подменяют модуль строкой (`sys.modules`, `patch("...")`), в счёт
  не вошли; поэтому «60 файлов» по grep против «47» по AST для `dialogue_node`.

## 3. Файлы и модули v1

«Часть» — номер из раздела 6. «Тестов» — файлов, импортирующих модуль (AST), и их строк.

| Файл | Строк | Читатели (AST, не-тест; grep не-тест / тест) | Чем заменён в v2 | grep-критерий удаления | Часть | Риск |
|---|---|---|---|---|---|---|
| `src/rob_box_voice/rob_box_voice/dialogue_node.py` | 4700 | 1 (`stt_admission_host.py`, под `TYPE_CHECKING`); `setup.py:49`, 2 launch; grep 131 / 166. Тестов 47 (10 650 строк) | `dialog_node.py` (444) + `core/dialog_v2_ports.py` (225) + `rob_box_dialog/turn.py` | `ls .../dialogue_node.py` → нет; `grep -rn "dialogue_node import\|rob_box_voice.dialogue_node" src docker scripts --include=*.py --include=*.yaml --include=*.sh` вне комментариев = 0; `grep -n dialogue_node src/rob_box_voice/setup.py` = 0 | 15a–15p (по методам) | высокий: корень v1; режется 16 частями |
| `config/dialogue_node.yaml` ×2 (`src/rob_box_voice/config/`, `docker/sound/config/voice_assistant/`) | 141 + 141 | `launch×2`, `startup_greeting` (поле в yaml) | `config/dialog_node.yaml` | `ls` обеих копий → нет | 15p | низкий |
| `core/dialogue_stage_mixin.py` | 81 | 1 (`dialogue_node.py`) / 1 / 2 | `rob_box_dialog/turn_log.py`, `core/dialog_v2_ports.py` | `grep -rlw dialogue_stage_mixin src --include=*.py` вне тестов = 0 | 15e | низкий |
| `core/turn_stages.py` | 275 | 2 (`dialogue_node`, `dialogue_stage_mixin`) / 3 / 3. Тестов 2 (907) | стадии хода и рефлексы: `core/dialog_v2_ports.py` + `rob_box_dialog/expression.py` | `grep -rlw turn_stages …` = 0 | 15e | средний: рефлексы `addressed/thinking/…` (A17) должны остаться в v2 — сверить |
| `core/turn_tools.py` | 111 | 2 (`dialogue_stage_mixin`, `turn_stages`) / 2 / 1 | `rob_box_dialog/execute.py` (исполнение) | `grep -rlw turn_tools …` = 0 | 15e | низкий |
| `core/turn_reflexes.py` | 130 | 2 (`turn_stages`, `turn_tools`) / 2 / 1 | `rob_box_dialog/expression.py` (`reflex()`) | `grep -rlw turn_reflexes …` = 0 | 15e | низкий |
| `core/turn_context.py` | 68 | 2 (`dialogue_node`, `tier1_host`) / 4 / 4 | `rob_box_dialog/context.py` | `grep -rlw turn_context src --include=*.py` вне тестов — только `llm_adapter.py` (другая сущность, проверить) | 15c | низкий |
| `core/turn_identity.py` | 146 | 1 (`dialogue_node`) / 2 / 2. Тестов 2 (359) | раздел `speaker` снимка: `rob_box_dialog/speaker_adapter.py`, `context.py` | `grep -rlw turn_identity …` = 0 (останется упоминание в `knowledge.py` — комментарий) | 15h | средний: `user_profile_lines` (имя, профиль) — убедиться, что v2 кладёт то же в контекст |
| `core/turn_speech.py` | 163 | 1 (`dialogue_node`) / 2 / 1. Тестов 1 (94) | `rob_box_dialog/respond.py` (фраза по событию) | `grep -rlw turn_speech …` = 0 | 15h | низкий. `final_text_for_speech` уходит, нормализация TTS остаётся в `tts_node` (ср. `speak_helpers`) |
| `core/stt_admission.py` | 840 | 4 (`dialogue_node`, `dialogue_stage_mixin`, `stt_admission_host`, `tier1_host`) / 6 / 8. Тестов 6 (2 156) | `rob_box_dialog/address.py`, `backlog.py`, `grammar.py`; `stt_node` → `InterruptRequest` | `grep -rlw stt_admission …` вне тестов = 0; `grep -rn "parse_tg_prefix" src` вне тестов = 0 (сейчас в нём 19 из 51 вхождений `[TG:`/`tg_chat_id`/`parse_tg_prefix`) | 15a (шаги 545–840) + 15m (ядро 1–544) | средний: самый большой файл; в 600 строк не влезает целиком, поэтому разрезан на два PR |
| `core/stt_admission_host.py` | 261 | 1 (`dialogue_node`) / 1 / 0 | `rob_box_voice/dialog_node.py` (тонкий хост) | `grep -rlw stt_admission_host …` = 0 | 15d | низкий (тестов нет) |
| `core/tier1_host.py` | 230 | 3 (`dialogue_node`, `stt_admission_host`, `tools_view_host`) / 3 / 3. Тестов 2 (432) | `rob_box_dialog/grammar.py` + `interrupt.py` | `grep -rlw tier1_host …` = 0 | 15c | средний: «спорный стоп» (В19) — проверить, что у v2 есть аналог `stop_verdict` |
| `core/tools_view_host.py` | 34 | 1 (`dialogue_node`) / 1 / 2 | `rob_box_dialog/tools_view.py` | `grep -rlw tools_view_host …` = 0 | 15c | низкий |
| `core/stop_chooser.py` | 85 | 2 (`dialogue_node`, `tier1_host`) / 4 / 2. Тестов 1 (112) | `rob_box_dialog/interrupt.py` (`stop_verdict`) | `grep -rlw stop_chooser src --include=*.py` вне тестов = 0 (ссылки в `identity_dedup.py`, `interrupt_request.py` — комментарии/имя класса, проверить) | 15c | средний |
| `core/greeting_window.py` | 79 | 3 (`dialogue_stage_mixin`, `meeting`, `stt_admission_host`) / 4 / 2. Тестов 1 (154) | `rob_box_dialog/greeting_reply.py` (окно ответа на приветствие) | `grep -rlw greeting_window …` = 0 | 15d | средний: окно ответа 15 с (решение Шифу 21.10) — сверить поведение |
| `core/identity_dedup.py` | 83 | 1 (`dialogue_node`) / 2 / 1 | `rob_box_dialog/speaker_adapter.py` | `grep -rlw identity_dedup …` = 0 | 15f | низкий |
| `core/session_epoch.py` | 62 | 1 (`dialogue_node`) / 1 / 5. Тестов 2 (197) | эпоха — у `tts_node` (`SpeechCancel`, `interrupt.py`) | `grep -rlw session_epoch …` вне тестов = 0 | 15j | средний: «0 действий старой эпохи» (A12) должна держаться на стороне владельца речи |
| `core/media_plan_run.py` | 128 | 1 (`dialogue_node`) / 3 / 8. Тестов 8 (1 309) | `rob_box_dialog/plan/*` + `core/plan_host.py` | `grep -rlw media_plan_run src --include=*.py` вне тестов = 0 (остаётся имя в `media_router.py` — комментарий) | 15i | средний: 8 тестовых файлов — 3 из них могут проверять общую `media_router` и требуют переноса, а не удаления |
| `core/library_command.py` | 63 | 1 (`media_plan_run`) / 3 / 1 | `core/media_router.py` (остаётся, используется `music_turn`) | `grep -rlw library_command …` = 0 | 15i | низкий |
| `core/meeting.py` | 213 | 1 (`dialogue_node`) / 6 (слово «meeting» шире) / 4. Тестов 2 (432) | **нет аналога в v2** (grep `meeting\|greet` в `dialog_node.py`, `dialog_v2_ports.py` = 0; есть только `greeting_reply.py` — ответ на приветствие) | `ls` → нет | 15b (после решения D1) | **высокий**: удаление = потеря приветствия по встрече |
| `startup_greeting.py` | 41 | 1 (`dialogue_node`) / 2 / 3. Тестов 3 (456) | **нет аналога в v2** (grep 0) | `ls` → нет | 15b (D1) | средний: потеря стартовой фразы |
| `core/tars_status_feed.py` | 115 | 1 (`dialogue_node`) / 1 / 1. Тестов 1 (70) | **нет аналога в v2** (grep 0) | `ls` → нет | 15b (D1) | средний: фид статуса LLM/вейков для ТАРС |
| `speaker_profiles.py` | 181 | 1 (`dialogue_node`) / 2 (`epithets.py` — в комментарии) / 2. Тестов 2 (439) | `rob_box_dialog/speaker_adapter.py` (частично; `SpeakerTracker` — нет аналога по имени) | `ls` → нет | 15g | средний |
| `core/conversation_history.py` | 255 | 0 прямых (только реэкспорт в `core/__init__.py`; `voice_memory.py:28,30` обращается к атрибуту `self.conversation_history`, а не к классу) / 4 / 2. Тестов 1 (250) | `rob_box_dialog/session.py` (кадры) + `context.py` | `ls` → нет; `grep -n conversation_history src/rob_box_voice/rob_box_voice/core/__init__.py` = 0 | 15a | низкий (читателей нет) |
| `core/speech_compat.py` | 74 | 2 (`tts_node`, `speech_owner`) / 6 / 2. Тестов 2 (562) | `rob_box_core/speech_client.say` (SpeechRequest) | `ls` → нет; `grep -rn "speech_compat\|attach_legacy" src` вне тестов и комментариев = 0 | 15q (после teleop 15-0d) | высокий: последний писатель `/voice/tts/request` — джойстик teleop (Main Pi) |
| `core/speak_helpers.py` | 594 → остаётся ≈ 97 | 2 (`dialogue_node`, `tts_node`) / 5 / 13. Тестов 11 (1 627) | нормализация TTS остаётся в `tts_node`; фраза о недоступном языке — `rob_box_dialog/phrases` | `grep -c "re.compile" …/speak_helpers.py` → 18 → цель 0 (остаются `strip_bracket_asides`, `strip_markdown`, `unsupported_language_notice`, `provider_speaks` ≈ 97 строк функций) | 15q | средний: регексы 18 → 0 не достигаются простым переносом; см. D2 |
| `core/dialogue_guards.py` | 169 → остаётся ≈ 27 функций + 2 регекса и константа | 5 (`dialogue_node`, `stt_admission_host`, `track_start_guard`, `tts_node`, `turn_speech`) / 10 / 4. Тестов 3 (325) | защита SSML (`is_system_template_regurgitated_in_ssml`, `is_tool_call_markup`) нужна `tts_node`; `MUSIC_STARTING_TOOLS` — `track_start_guard` | `grep -c "re.compile" …` 2 → 2 (см. D2) | 15s | средний: A8 «`re.compile` → 0» без решения Шифу не достижим |
| `core/dialogue_helpers.py` | 142 → остаётся 19 (`sanitize_speaker_name`) | 2 (`dialogue_node`, `speaker_turn`) / 4 / 3. Тестов 1 (176) | `speaker_turn` (v2) остаётся читателем | `grep -rn "EMOTION_TO_ANIMATION\|detect_pitch_intent\|generate_fallback_response" src` вне тестов = 0 | 15r | низкий |
| `core/dialogue_text.py` | 48 | 2 (`dialogue_node`, `stt_node`) / 5 / 12 | остаётся (читает `stt_node`) | — | не удаляется | — |
| `src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py` | 469 | 8 (`__init__`, `agent_core`, `assembly`, `meeting`, `stt_admission_host`, `tier1_host`, `supervisor_node`, `dialogue_node`) / 10 / 37. Тестов 35 (12 315, с пересечением с тестами ноды) | кадры стека: `rob_box_dialog/session.py`, `scenes.py`; `DialogControl.srv` (PR-10a #3851) | `ls` → нет; `grep -rn "DialogueStateMachine\|dialogue_state_machine\|_pause_reason\|check_silence_timeout" src --include=*.py` вне тестов: **34 → 0** (22 → 0 со всеми тестами для критерия #3704 «`"/dialogue/control`…») | 15r | высокий: потребитель вне v1 — ТАРС (`supervisor_node`, `AgentCore(dsm=…)`) |

Модули вне замкнутого множества, которые карточка не называет, но у которых **нет не-тестовых читателей** (AST): `core/llm_config.py`
(257 строк, читатель — только тест), `core/voice_memory_init.py` (113), `core/skip_reason_checker.py` (35, тест), `utils/github_summary.py`
(297, тест). Это не v1-диалог в явном виде и не входит в PR-15; по правилу «мёртвое — удалять с grep-доказательством»
их можно добавить в часть 15a с отдельного разрешения (D6).

## 4. Топики и контракты

| Топик / контракт | Писатель сейчас | Читатели (файлов, не-тест) | Чем заменён | grep-критерий | Часть | Риск |
|---|---|---|---|---|---|---|
| `/voice/dialogue/state` (String JSON) | `dialogue_node.py:615` (1); **при v2 — никто** | 12 файлов: `led_node` (3), `command_node` (1), `quest_node` (3), `quest/streams/voice_state.py` (1), `arbiter_node` (2), `supervisor/core/aggregator.py` (3), `supervisor/core/__init__.py` (1, комментарий), `bridge_protocol.py` (1), `webxr_client/src/dev/mock_robot.ts` (1), `agent_core.py` (1, комментарий), `scripts/agent_flow/create_avatar_issues.sh` (2, текст) | `/dialog/session` (`SessionState`, уже публикуется v2) и `/dialog/turn_event` (`TurnEvent`) | `grep -rFn "/voice/dialogue/state" src docker scripts --include=*.py --include=*.ts --include=*.yaml` вне тестов = 0 | 15-0a (читатели) → 15p (писатель) | высокий: меняет проводной формат `voice_state (0x1202)` для шлема Quest; гард «G-Bridge-Protocol-Drift» |
| `/voice/tts/state` (String) | `tts_node.py:1344` | `face_node` (3), `face_director` (1), `audio_node` (2), `stt_node` (1), `speech_compat` (1, комментарий), `scripts/maintenance/scene_set/replay.py` (1), `face_node.yaml` (1, комментарий) | `SpeechEvent` (`/voice/speech/event`) | то же, `grep -rFn "/voice/tts/state"` вне тестов = 0 | 15-0b → 15t | средний: `face_node` помечает его как «transitional source of speaking» (`face_director.py:208`) |
| `/voice/tts/finished` (String) | `tts_node.py:1346` (12 упоминаний) | `quest_node` (2), `quest/core/tars_stages.py` (2), `tars_stage_relay.py` (2), `quest/server/ws_server.py` (1), `supervisor_node` (4), `supervisor/tars_stage.py` (1), `speaker_id_node` (2), `mcp_server` (2), `speak_helpers` (3), `dialogue_node` (2), `speech_owner` (1), `speech_compat` (1), `scripts/maintenance/scene_set/conductor.py` (2), `replay.py` (1) | `SpeechEvent finished` по `speech_id` | `grep -rFn "/voice/tts/finished"` вне тестов = 0 | 15-0c → 15t | средний: `mcp_server` и `quest` коррелируют по `speech_id` (#3253) |
| `/voice/dialogue/response` (String JSON) | `dialogue_node.py` (3); при v2 — никто | `telegram_node` (4), `context_aggregator_node` (2), `tts_node` (5), `agent_core` (2, комментарий), `speech_compat` (3), `speak_helpers` (1), `internal_dialogue.launch.py` (1), `scripts/example_voice_usage.py` (1), `dialog_engine.py` (1), `dialog_engine.yaml` ×2 (комментарий) | `/dialog/output` (`DialogOutput`, PR-12a) для Telegram; для `context_aggregator` — решение (см. D4) | `grep -rFn "/voice/dialogue/response" src --include=*.py` вне тестов = 0 | 15-0e → 15p/15q | средний: в `runtime-baseline.json` запись `multiple_writers|/voice/dialogue/response` держит долг PR-15 |
| `/voice/tts/request` (String JSON) | `rob_box_teleop` (джойстик, `joystick_control_node.py:419`), `scripts/maintenance/scene_set/conductor.py:231`; вход `tts_node.dialogue_callback` | `tts_node` (подписка через `speech_compat.attach_legacy`), `scripts/diagnostics/watchdog-tts.sh` | `rob_box_core.speech_client.say` → `SpeechRequest` | `grep -rFn "/voice/tts/request" src --include=*.py` вне тестов и комментариев = 0; запись `multiple_writers|/voice/tts/request` в `runtime-baseline.json` удалена | 15-0d → 15q | высокий: Main Pi (teleop), пересборка образа |
| `/voice/tts/control` (String `STOP`) | `speech_compat`/внешние | `tts_node` | `SpeechCancel` | запись `multiple_writers|/voice/tts/control` в `runtime-baseline.json` удалена | 15q | средний: проверить писателей вне `src/` |
| `/dialogue/control`, `/dialogue/control_ack` (String JSON) | `supervisor_node` (28 упоминаний; при v2 не публикует) | `dialogue_node` (11) | `DialogControl.srv` (PR-10a) | `grep -rn '"/dialogue/control' src --include=*.py --include=*.ts` вне тестов = 0 (сейчас: `supervisor_node` 28, `dialogue_node` 11, `webxr_client` 2, `dialog_engine.yaml`/`dialogue_node.yaml` — комментарии); запись `dead_output|/dialogue/control_ack` в `runtime-baseline.json` и `seam_baseline.json` удалена | 15r | средний |
| `/voice/stt/result` | — | — | уже заменён `/voice/stt/utterance` | `grep -rn "voice/stt/result" src docker scripts` вне тестов = 0 (**уже выполнено**) | — | — |
| `architecture/ownership.yml` | комментарий у `/dialog/output` («его удалит PR-15»), блок `dialogue_node` в `nodes:`, комментарий про `speech_compat` (строка 94) | — | — | `grep -n "dialogue_node\|speech_compat" architecture/ownership.yml` = 0 | 15t | низкий |
| Бюджеты | `scripts/lint/cc_budget_baseline.json`, `class_budget_baseline.json` (`DialogueNode` WMC 525, 112 методов), `seam_baseline.json`, `seam_allowlist.json`, `runtime-baseline.json` | — | пересчитать вниз после каждой части | `python scripts/lint/class_budget.py` без `dialogue_node.py` | каждая часть + 15u | низкий |

## 5. Читатели, которые надо перевести до удаления

Образы: **voice-assistant**, **supervisor** (общий для `supervisor` и `avatar-arbiter`), **quest**, **telegram-bot** — Sound Pi;
**led-matrix** — Vision Pi; **teleop** — Main Pi; **perception** — Main Pi (и `sensor-bridge` на Sound Pi).

| Часть | Что переводим | Читатели (строк в файле) | Образы, которые пересобираем | Размер правки |
|---|---|---|---|---|
| 15-0a | `/voice/dialogue/state` → `SessionState`/`TurnEvent` | `led_node.py` (308), `command_node.py` (396), `quest_node.py` (3203), `streams/voice_state.py` (176), `arbiter_node.py` (1518), `core/aggregator.py` (185), `bridge_protocol.py` (1510), `webxr_client/src/dev/mock_robot.ts` | voice-assistant, quest, supervisor | оценка 150–300 строк (протокол шлема — отдельное согласование; не измерял) |
| 15-0b | `/voice/tts/state` → `SpeechEvent` | `face_node.py` (265), `face_director.py` (350), `audio_node.py` (1187), `stt_node.py` (1894) | led-matrix, voice-assistant | оценка 80–150 |
| 15-0c | `/voice/tts/finished` → `SpeechEvent finished` | `quest_node.py`, `core/tars_stages.py` (184), `tars_stage_relay.py` (145), `supervisor_node.py` (3057), `speaker_id_node.py` (2386), `mcp_server.py`; скрипты `conductor.py`, `replay.py` | voice-assistant, quest, supervisor | оценка 100–200 |
| 15-0d | teleop: `speech_client.say` вместо JSON `/voice/tts/request` (по комментарию к #3711: добавить сборку `rob_box_dialog_msgs` + `rob_box_core` в `docker/main/teleop/Dockerfile` по образцу `docker/sound/quest/Dockerfile`, строки 197–198) **или** убрать голос джойстика | `joystick_control_node.py` (443), `joystick_logic.py` (102); тесты `test_issue_3107_tts_request.py`, `test_joystick_*.py` | **teleop (Main Pi)** | оценка 60 строк + Dockerfile 10 строк |
| 15-0e | `/voice/dialogue/response` → `/dialog/output` или `SpeechEvent` | `telegram_node.py` (688; ветка v1 `_subscribe_replies`), `context_aggregator_node.py` (714; «ответы робота» для контекста) | telegram-bot, perception | оценка 40–80 |
| 15-0f | Паритет v2: встреча лицом, стартовое приветствие, фид статуса ТАРС, `SpeakerTracker` (если Шифу решит сохранить) | перенос `meeting.py` (213) + `startup_greeting.py` (41) + `tars_status_feed.py` (115) + части `speaker_profiles.py` в `dialog_node`/`rob_box_dialog` | voice-assistant | зависит от D1; оценка 300–400 строк переноса |

Что есть и что ещё нужно по ТАРС (supervisor): путь v1 `AgentCore` в `supervisor_node.py` (`_dialogue_control_publisher` 13,
`_publish_dialogue_control` 33, `_dialogue_control_swap` 47, `_ensure_agent_core` 14, `_build_agent_core_sync` 81 — всего 188 строк
по AST) и ветка `agent_path` в `operator_turns.py:228`; удаляется в части 15r вместе с `DialogueStateMachine` (ТАРС работает на общем
движке при v2 с PR-13a, но `AgentCore(dsm=…)` ещё на месте).

## 6. План частей

Правило размера: «строк» = удаляемые не-тестовые строки (`wc -l` файлов + диапазоны методов по AST), порог 600; тесты
удаляются в тех же PR и не входят в размер. Порядок — от тех, у кого читателей нет, к корню (`__init__` и шапка
`dialogue_node.py` — в последней). Каждая часть, кроме оговоренных, меняет образ **voice-assistant** (Sound Pi).
Цифры по `dialogue_node.py` — `slices2.py` по AST (приложение); `__init__` режется постепенно, строки его убираемых блоков
в частях не учтены (оценка +30…80 на часть; поэтому последняя часть меньше расчётной).

«Фразы» — инъекция `Робот, …` в `/voice/stt/utterance` на Sound Pi (без пунктуации, один `ssh`, стенд свободен — по правилам
карточки); смоук для каждой части: `ros2 node list` (есть `/dialog_node`, нет `/dialogue_node`) и последние 50 строк
`docker logs voice-assistant` без `Traceback`.

| Часть | Что удаляется | Строк (без тестов) | Тестов (файлов, оценка) | Образы | Зависит от | Что проверить на образе (3–10 фраз) |
|---|---|---|---|---|---|---|
| **15-0a…15-0f** | перевод читателей и паритет (раздел 5) | не удаление; оценка 700–1100 строк правок суммарно | — | см. раздел 5 | PR-14 (v2 дефолт) частично нужны **до** него | см. разделы 5 и 7: на каждый образ — 3–5 фраз и просмотр читателя (LED-кольцо, шлем Quest, лицо) |
| **15a** | `core/conversation_history.py` (255) + реэкспорт в `core/__init__.py`; хвост `core/stt_admission.py` строки 545–840 (шаги допуска и `default_steps`, 296) | 551 | 1 + часть 6 тестов допуска | voice-assistant | — | смоук: «Робот, который час», «Робот, сколько у тебя заряда», «Робот, расскажи короткую шутку» |
| **15b** | `core/meeting.py` (213), `startup_greeting.py` (41), `core/tars_status_feed.py` (115); методы `_on_startup_greeting*`, `_subscribe_vision_events`, `_on_vision_event`, `_cancel_greeting_timer` (154) | 523 | 6 | voice-assistant | **15-0f / решение D1** | если перенесено: знакомый, не виденный > 20 ч, входит в кадр → одна фраза с именем без вейка; рестарт ноды → стартовая фраза; ТАРС показывает статус LLM (панель). Если отброшено — только смоук |
| **15c** | `core/tier1_host.py` (230), `stop_chooser.py` (85), `tools_view_host.py` (34), `turn_context.py` (68); методы `_on_stt`, `_finish_stt_admission`, `_resume_stt_after_media`, `_dispatch_cleaned` (148) | 565 | 7 | voice-assistant | 15a | «Робот, стой»; «Робот, хватит»; «Робот, замолчи» → потом «Робот, говори»; «Робот, тише»; «Робот, покажи анимацию стоп» (спорный стоп — не стоп); «Робот, хватит, расскажи анекдот» (стоп + команда) |
| **15d** | `core/stt_admission_host.py` (261), `core/greeting_window.py` (79); мелкие обработчики `_on_vad`, `vad_callback`, `_handle_silence`, `_on_command_feedback`, `_on_tts_finished`, `_on_tts_provider_state`, `_on_odom_snapshot`, `_publish_state`, `_map_emotion_to_animation`, `_is_llm_unavailable_error`, `_generate_fallback_response`, `_detect_pitch_intent`, `_on_mcp_tools_update`, `_on_perception_update` (207) | 547 | 3 | voice-assistant | 15c | фраза без вейка («который час») — молчит и уходит в бэклог; голое «Робот» → по правилу В12′; «рома, сколько времени» не будит (В18); приветствие и ответ без вейка в окне 15 с |
| **15e** | `core/dialogue_stage_mixin.py` (81), `turn_stages.py` (275), `turn_tools.py` (111), `turn_reflexes.py` (130) | 597 | 3 | voice-assistant | 15c | «Робот, который час»: в логе стадии `addressed → thinking → speaking → done`, рефлекс ≤ 300 мс, LED-кольцо и earcon; ход с ошибкой тула (отключить тул на стенде) → стадия `failed` |
| **15f** | методы идентичности `_ask_identity_if_ambiguous` … `_current_acquaintance` и `_resolve_speaker_for_utterance` … `_confirm_tentative_speaker` (482), `core/identity_dedup.py` (83) | 565 | 3 | voice-assistant | 15d | **только e2e-режим БД дикторов** (`e2e_mode`, не боевая `speakers.db`): «Робот, меня зовут Саша», «Робот, как меня зовут», «Робот, это ты?»-ветка (≤ 1 вопрос на собеседника) |
| **15g** | методы `_publish_confirmed_identity_growth` … `_publish_speaker_observation` (318), `speaker_profiles.py` (181) | 499 | 3 | voice-assistant | 15f | e2e-режим: регистрация, `Робот, кто это сказал` (диктор в бэклоге), эпитеты; без записи в боевую БД |
| **15h** | `core/turn_identity.py` (146), `core/turn_speech.py` (163); методы `_voice_turn_text`, `_release_turn_speech`, `_music_reply_context`, `_publish_turn_text`, `_publish_response`, `_publish_response_batch`, `_speak_direct`, функции `_clean_name`, `identity_question`, `classify_identity_confirmation` (206) | 515 | 4 | voice-assistant | 15g | «Робот, расскажи скороговорку», «Робот, повтори последнюю фразу», «Робот, который час» → в `SpeechEvent started` нет служебного текста; Telegram `/say` (test chat) |
| **15i** | `core/media_plan_run.py` (128), `core/library_command.py` (63); методы `_music_playing_now` … `_execute_media_tool_result`, `_on_music_state`, `_on_sound_state` (208) | 399 | 9 (проверить 3 теста `media_router`) | voice-assistant | 15e | «Робот, включи музыку», «Робот, сыграй что-нибудь для души», «Робот, стоп» посреди трека, «Робот, добавь бит», «Робот, выключи музыку» (стенд свободен, громко — по согласованию) |
| **15j** | `core/session_epoch.py` (62); методы управления сессией: `_on_dialogue_control` … `_publish_control_ack`, `_session_epoch_gate` … `_finalize_turn_dsm`, `_reset_dialogue_session` … `_maybe_record_session_end` (412) | 474 | 4 | voice-assistant | 15i | «Робот, новая сессия» (подтверждение вслух), `DialogControl hold` → «Робот, который час» не слушается → `resume`; «Робот, стой» посреди длинной реплики (≤ 300 мс) |
| **15k** | ход v1: `_handle_llm_error`, `_invoke_llm_with_telemetry`, `_prepare_user_input_context`, `_handle_result` (465) | 465 | оценка 6 | voice-assistant | 15h | набор из 10: «который час», «сколько заряда», «что видишь», «статус», «расскажи анекдот», «включи звук», «покажи анимацию радуги», «Робот, и про енота» в рэпе; `honesty_audit.py` по логу |
| **15l** | ход v1: `_dispatch_turn`, `_build_dynamic_system_context`, `_handle_speaker_turn`, `_run_turn` (327) | 327 | оценка 4 | voice-assistant | 15k | те же 10 фраз; `turn_metrics.py` по логу (A1/A2) |
| **15m** | ядро `core/stt_admission.py`, строки 1–544 (типы, протоколы, `SttAdmission`) | 544 | 5 | voice-assistant | 15d, 15a | как 15d; плюс `Робот` без продолжения через 20 с |
| **15n** | сборка и разбор настроек: `_parse_provider_chain` … `_music_state_mem`, `parameters_callback` (509) | 509 | оценка 3 | voice-assistant | 15l | старт ноды с каждым провайдером цепочки (`llm_providers`): 3 фразы, по одной на провайдера |
| **15o** | `_declare_params`, `shutdown_asyncio_loop`, `destroy_node`, `main` (230) + импорты/шапка 1–297 | 527 | — | voice-assistant | 15n | смоук + корректная остановка контейнера (`docker stop` ≤ 10 с, без `Traceback`) |
| **15p** | `__init__` (484) и остаток шапки (≈ 253 вне методов); сам `dialogue_node.py`; entry point `setup.py:49`; `config/dialogue_node.yaml` ×2 (141 + 141); ветка `dialogue_node` в launch×2; запись `dialogue_node` в `ownership.yml` | ≤ 737 + 282 (конфиги); фактически меньше (части a–o вынули часть `__init__`) | оставшиеся тесты ноды | voice-assistant | 15a–15o | смоук + 10 фраз набора; `ls src/rob_box_voice/rob_box_voice/dialogue_node.py` → нет; `grep -c "Bug [A-F]\b" dialogue_node.py` — файла нет |
| **15q** | `core/speech_compat.py` (74), `core/speak_helpers.py` (594 → ≈ 97: −497); `attach_legacy` в `tts_node.py:1250`, `meta_from_legacy` в `speech_owner.py:100` | 571 + хвосты `tts_node` (оценка 25) | 13 | voice-assistant | 15-0d, 15p | `/say` из Telegram (русский и CJK → `unsupported_script`), «Робот, расскажи скороговорку» с markdown/скобками в тексте ответа (звёздочки не произносятся), джойстик: голосовая реплика (если оставлена) |
| **15r** | `harness/core/dialogue_state_machine.py` (469) и его читатели (`agent_core`, `assembly`, `__init__`, `supervisor_node`); путь v1 `AgentCore` ТАРС в `supervisor_node.py` (188) и `agent_path` v1-ветка; `core/dialogue_helpers.py` (142 → 19: −123) | 469 + 123 + 188 ≈ 780 → делится на 15r1 (DSM + dialogue_helpers = 592) и 15r2 (путь v1 ТАРС ≈ 190–250, оценка) | 37 (DSM) | voice-assistant, **supervisor** | 15p | ТАРС: 8 команд как в #3708 (батарея, время, статус, что видит, «скажи вслух», громкость, анимация, стоп); `hold/resume` от ТАРС; «замолчи» → «говори» |
| **15s** | флаг `dialog_engine`: `rob_box_core/dialog_engine.py` (77), 44 строки в 17 файлах (`tts_node`, `dialog_node`, `telegram_node`, `supervisor_node`, `operator_turns`, `operator_admin`, `node_monitor`, launch×2, `setup.py`, `start_supervisor.sh`, `start_telegram_bot.sh`), `dialog_engine.yaml` ×2; ветка v1 `_subscribe_replies` в `telegram_node`; `dialogue_guards.py` 169 → ≈ 70 | оценка 280–350 | оценка 4 | voice-assistant, supervisor, telegram-bot, **led-matrix/teleop — не затрагиваются** | 15p, 15r | рестарт всех трёх контейнеров: 3 фразы голосом, 3 сообщения в Telegram (`Клод …`, `/say`), 3 команды ТАРС; `ros2 param get /dialog_node dialog_engine` → параметра нет |
| **15t** | String-топики: публикации `/voice/tts/state` и `/voice/tts/finished` в `tts_node` (≈ 12 упоминаний), запись об адаптере в `ownership.yml`, записи `multiple_writers` / `dead_output` в `runtime-baseline.json`, `seam_baseline.json`, `seam_allowlist.json` | оценка 150–250 | оценка 5 | voice-assistant | 15-0b, 15-0c, 15q | `ros2 topic info` по каждому: писателей 0; «L: Architecture Audit» без `multiple_writers` по диалогу (A6); 5 фраз с LED-кольцом, лицом, шлемом |
| **15u** | промпты: `RULE #` 15 → 0 в `master_prompt_compact.txt`, `telegram_chat_prompt.txt`; генерация из `knowledge.prompt_fragment()` + персона | оценка 80–150 (правка текста) | 5 тестов промпта | voice-assistant | D3 | **A/B на живом v2**: набор честности ≥ 60 фраз до и после (A3 не хуже), A1/A14 по `turn_metrics.py` |
| **15v** | `#NNNN` в `rob_box_dialog/rob_box_dialog`: 158 → ≤ 20 (правка комментариев, `knowledge.py` 47, `speaker_adapter.py` 20, `command.py` 12, `address.py` 12, …); `Bug [A-F]` в оставшихся (`dialogue_guards.py` 1, `media_command_grammar.py` 1) → 0 | оценка 140 строк правок | — | образ не меняется по поведению | 15p | `pytest -v src/rob_box_dialog`, смоук |
| **15w** | тесты по номерам issue: 77 `test_issue_*` в `rob_box_voice/test`, 28 из них удаляются с v1-модулями, 49 переписываются на поведение модулей (+30 в других пакетах src, +17 в `tests/`); `CONTEXT.md` по §13; пересчёт baseline `cc_budget`/`class_budget`/`seam` | оценка 200–400 (CONTEXT.md, baselines) | 49 + 12 смешанных переписать | — | 15v | `pytest -v` всех пакетов, 5 гардов (`cc_budget`, `class_budget`, `seam_without_consumer`, `cc_budget_refs`, `dialogue_skip_reasons`), `L: Architecture Audit` |

12 «смешанных» тестов (импортируют и v1-модули, и `rob_box_dialog`/`dialog_node`) нельзя просто удалить, их надо переписать
(список в приложении).

## 7. Решения, нужные от товарища Шифу до старта PR-15

- **D1.** Паритет v2: приветствие по встрече лицом (`meeting.py`), стартовая фраза, фид статуса ТАРС, `SpeakerTracker` — переносим в
  v2 или отбрасываем? Без решения часть 15b блокирована.
- **D2.** A8 «`re.compile` → 0»: две защитные регулярки SSML в `tts_node` (`dialogue_guards.py`) и 18 в `speak_helpers.py`
  (из них функции `strip_bracket_asides`, `strip_markdown`, `unsupported_language_notice` нужны `tts_node`) — простой перенос не даёт 0;
  принять «≤ N, но не в слое диалога» или переписать структурно?
- **D3.** `RULE #` = 0 меняет промпт живого v2 (15 вхождений в двух файлах): допустим A/B-прогон набора честности до/после.
- **D4.** `context_aggregator_node` читает `/voice/dialogue/response` («ответы робота» в контекст перцепции): заменить на `/dialog/output` /
  `SpeechEvent` или отказаться от этого входа?
- **D5.** После 15s откат `dialog_engine: v1` невозможен — подтвердить, что откатываться будем гитом, а не флагом.
- **D6.** Мёртвые модули без читателей вне карточки (`llm_config.py` 257, `voice_memory_init.py` 113, `skip_reason_checker.py` 35,
  `utils/github_summary.py` 297) — включать в 15a?
- Критерий #3705 `voice_memory.db` слишком широк: grep даёт 45 вхождений в 12 файлах, включая `rob_box_music`, `waypoint_*`,
  `speaker_id_node` (5), `mcp_server` (19). Закрывается ли он в PR-15 или это отдельная карточка памяти (ADR-0055 `harness_voice.db`)?

## 8. Хвосты PR-10b, PR-11, PR-12a (по комментариям эпика) — состояние grep на develop

| Карточка | Критерий из карточки | Сейчас (вне тестов) | Что остаётся | Часть |
|---|---|---|---|---|
| #3704 PR-10b | `ls …/dialogue_state_machine.py` → нет; `grep -rn '"/dialogue/control\|_pause_reason\|check_silence_timeout' src --include=*.py` = 0 | файл есть (в `rob_box_harness/core/`, а не `rob_box_voice/core/`, как в карточке); grep: 10 вхождений в 3 файлах (`dialogue_state_machine.py` 2, `supervisor_node.py` 2, `dialogue_node.py` 6); шире `DialogueStateMachine\|dialogue_state_machine\|…` — 34 | DSM, `/dialogue/control*`, `_pause_reason` | 15r (+15j) |
| #3705 PR-11 | `grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py` = 0 | 59 вхождений в 16 файлах: `_HISTORY_MARKER_RE` и `_META_PREFIX_RE` по 2 в `speak_helpers.py`; `_SPEAKER_TAG_RE` 0; `discard_last_reply` 10 (`agent_core` 6, `dialogue_node` 3, `turn_speech` 1); `voice_memory.db` 45 (см. раздел 7, последний пункт) | маркеры и `discard_last_reply` уходят с v1 (15q, 15h, 15k); `voice_memory.db` — вопрос | 15h, 15k, 15q |
| #3706 PR-12a | `grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py` = 0; `grep '"/voice/dialogue/response"\|"/voice/sound/stop"' src/rob_box_telegram` = 0 | 51 вхождение в 9 файлах: `stt_admission.py` 19, `dialogue_node.py` 18, `speak_helpers.py` 4, `turn_identity.py` 3, `telegram_node.py` 2, `address.py` 2, `utterance.py` 1, `speech_compat.py` 1, `tier1_host.py` 1; второй grep по `rob_box_telegram` — 2 | v1-путь Telegram: `telegram_node._subscribe_replies` и `_on_response` | 15s (+15a/15m, 15h) |
| #3708 PR-13b (не PR-15, для контекста) | `_utterance_fallback.py` → нет; `GRIP_TTS_REQUEST_TOPIC` = 0 | файла нет; `GRIP_TTS_REQUEST_TOPIC` 0 — **уже выполнено** | — | — |

## 9. Сводка

- **Удаляется целиком или сокращается (измерено):** файлы v1 — 8 344 строки (`dialogue_node.py` 4 700 + 21 модуль 3 389 +
  `conversation_history.py` 255) в частях 15a–15p; `speech_compat.py` 74 + сокращение `speak_helpers.py` на 497 (15q);
  `dialogue_state_machine.py` 469 + сокращение `dialogue_helpers.py` на 123 (15r) = **9 507 строк** с точностью до
  `__init__`-блоков. Не измерено (оценка): путь v1 ТАРС 190–250, флаг `dialog_engine` и `dialogue_guards` 280–350,
  String-топики в `tts_node` 150–250, комментарии `#NNNN` 140, промпты 80–150, CONTEXT.md и baseline 200–400,
  конфиги `dialogue_node.yaml` 282.
- **Тесты, уходящие вместе с v1:** 72 файла, 16 136 строк (AST-импорт v1-модулей, без DSM-тестов, которые пересекаются);
  35 файлов (12 315 строк) используют `DialogueStateMachine`; 12 «смешанных» тестов переписываются, 49 из 77 `test_issue_*` в
  `rob_box_voice/test` остаются и переписываются на поведение.
- **Частей:** удаления **16** (15a–15p, по `dialogue_node.py` и модулям) + **7** (15q, 15r1, 15r2, 15s, 15t, 15u, 15v) + 15w
  (тесты/CONTEXT/baseline) = **24**; перед ними **6** подготовительных (15-0a…15-0f). В порог 600 не укладываются
  без оговорок: 15p (до 737 + конфиги, оценка уменьшится при вынимании `__init__` по частям) и 15r (780 → разделена на 15r1/15r2).
- **Какие образы пересобирать:** voice-assistant — все части 15a–15u; **supervisor** — 15-0a, 15-0c, 15r, 15s; **quest** —
  15-0a, 15-0c; **telegram-bot** — 15-0e, 15s; **led-matrix (Vision Pi)** — 15-0b; **teleop (Main Pi)** — 15-0d;
  **perception** — 15-0e (если решено по D4). Пересборка Main Pi — единственная в 15-0d.
- **Что нужно решить до старта:** D1–D6 (раздел 7).

## Приложение. Команды и вывод

Среда: worktree на `origin/develop @ 8178e2809`, Windows. Скрипты разбора (AST-граф, срезы по методам) — одноразовые, в репозиторий
не добавлены; ниже — их результат и эквивалентные команды.

1. Существование файлов из карточки: `git ls-files | grep -E "(^|/)(turn[a-z_]*|dialogue_[a-z_]*|speak_helpers|speech_compat|conversation_history|stt_admission[a-z_]*|dialogue_state_machine)\.py$"` →
   `src/rob_box_dialog/rob_box_dialog/turn.py`, `turn_log.py`, `src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py`,
   `src/rob_box_voice/rob_box_voice/core/{conversation_history,dialogue_guards,dialogue_helpers,dialogue_stage_mixin,dialogue_text,speak_helpers,speech_compat,stt_admission,stt_admission_host,turn_context,turn_identity,turn_reflexes,turn_speech,turn_stages,turn_tools}.py`,
   `src/rob_box_voice/rob_box_voice/dialogue_node.py`, `scripts/dialog/turn_metrics.py`, `scripts/lint/dialogue_skip_reasons.py`.
   `core/turn.py` и `turn_speech_gate.py` в списке отсутствуют.

2. Размеры: `wc -l` — `dialogue_node.py` 4700, `dialog_node.py` 444, `core/dialog_v2_ports.py` 225, `stt_admission.py` 840,
   `speak_helpers.py` 594, `dialogue_state_machine.py` 469, `supervisor_node.py` 3057, `telegram_node.py` 688.
   Остальные — в таблице раздела 3 (значения получены тем же `wc -l`).

3. Методы `DialogueNode` по AST (начало, конец, строк): `__init__` 407–890 (484), `_declare_params` 891–1067 (177), `_run_turn` 3005–3176 (172),
   `_handle_result` 3660–3968 (309) и т. д.; всего 112 методов, 4267 строк класса; вне методов — 550 строк.
   Разрезка по частям: `15a 551 / 15b 523 / 15c 565 / 15d 547 / 15e 597 / 15f 565 / 15g 499 / 15h 515 / 15i 399 / 15j 474 / 15k 465 / 15l 327 / 15m 544 / 15n 509 / 15o 527`,
   остаток `__init__` + шапка 737; сумма 15a–15o = 7 607, плюс 737 = 8 344.

4. Замкнутое v1-множество (AST): 22 файла, 8 089 строк (с `dialogue_node.py`); файлы: `dialogue_stage_mixin`, `greeting_window`, `identity_dedup`,
   `library_command`, `media_plan_run`, `meeting`, `session_epoch`, `stop_chooser`, `stt_admission`, `stt_admission_host`, `tars_status_feed`,
   `tier1_host`, `tools_view_host`, `turn_context`, `turn_identity`, `turn_reflexes`, `turn_speech`, `turn_stages`, `turn_tools`, `speaker_profiles`,
   `startup_greeting`, `dialogue_node` (+ `harness/identity/base.py`, который AST ошибочно включил через `from rob_box_harness.identity.base import FaceSignal`;
   он общий — его читают `identity/__init__` и через него `encounter`, `mcp_server`, `identity_seam` — в карту не входит).

5. Читатели и критерии карточек (вне тестов):
   - `grep -rn '"/dialogue/control\|_pause_reason\|check_silence_timeout' src --include=*.py` → `dialogue_state_machine.py` 2, `supervisor_node.py` 2, `dialogue_node.py` 6;
   - `grep -rn "DialogueStateMachine\|dialogue_state_machine\|_pause_reason\|check_silence_timeout" src --include=*.py` → 34 строки (`harness/core/__init__` 3, `agent_core` 4, `assembly` 2, `dialogue_state_machine` 8, `quest/streams/voice_state` 1, `supervisor_node` 5, `meeting` 1, `stt_admission_host` 1, `tier1_host` 1, `dialogue_node` 8);
   - `grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py` → 51;
   - `grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py` → 59;
   - `grep -c "RULE #" src/rob_box_voice/prompts/*.txt` → `master_prompt_compact.txt` 8, `telegram_chat_prompt.txt` 7;
   - `grep -rc "#[0-9]\{4\}" src/rob_box_dialog/rob_box_dialog` (сумма) → 158;
   - `grep -rn "Bug [A-F]\b" src/rob_box_voice src/rob_box_dialog --include=*.py` → 26 (8 вне тестов).

6. Топики, читатели вне тестов (`grep -rFn "<топик>" src docker scripts …`): `/voice/dialogue/state` — 12 файлов (3+3+3+2+2+1+1+1+1+1+1+1 по `led_node`, `aggregator`, `quest_node`, `arbiter_node`, `create_avatar_issues.sh`, `dialogue_node`, `command_node`, `core/__init__`, `mock_robot.ts`, `voice_state.py`, `agent_core`, `bridge_protocol`);
   `/voice/tts/state` — 7 файлов-читателей и писатель `tts_node`; `/voice/tts/finished` — 15 файлов вместе с писателем; `/voice/dialogue/response` — 12 файлов; `/voice/tts/request` — писатели `joystick_control_node.py`, `scene_set/conductor.py`.

7. Команда запуска гардов на этом дереве: `python scripts/lint/class_budget.py` → `DialogueNode WMC=525 methods=112`, `class_budget: OK`;
   `python scripts/lint/ownership_check.py` → `OK - 25 топиков диалога в коде имеют владельца, 27 записей в ownership.yml`.

8. 12 смешанных тестов (импортируют и v1-модули, и `rob_box_dialog`/`dialog_node`): `test/identity_v17_legacy.py`,
   `unit/core/test_adr0149_pr6_media_router_v2.py`, `test_command_bridge.py`, `test_greeting_window.py`, `test_identity_dedup.py`,
   `test_stop_chooser.py`, `test_stt_admission.py`, `test_tools_view_host.py`, `unit/node/test_address_backlog.py`,
   `test_issue_2828_identity_question_collision.py`, `test_issue_3801_one_voice_one_person.py`, `test_speaker_turn_v17.py` (все в `src/rob_box_voice/test/`).

## Что не проверено

- Ничего не запускалось на роботе и в CI; ни одна часть не реализована, размеры частей — расчёт по строкам файлов и диапазонам методов AST, а не по
  готовым дифам. Фактический дифф частей будет отличаться (удаление импортов, правка `__init__`, тесты).
- Не доказано, что методы внутри одной части не вызываются из соседней (перекрёстные вызовы `self._…`) — это проверяется
  `pyflakes`/тестами при реализации; v1 при v2 не стартует, но тесты ноды до удаления гоняют методы.
- Не проверялось, что `tts_node.dialogue_callback` (252 строки; «ЕДИНЫЙ вход синтезатора») не используется путём `SpeechRequest`. Поэтому он **не включён** в 15q;
  его удаление — отдельная проверка.
- Не сверялось семантическое соответствие замен v2 (рефлексы, `stop_verdict`, окно ответа 15 с, `user_profile_lines`) — столбец «чем заменён»
  — по имени и назначению модуля, не по сравнению поведения.
- Оценки правок читателей (раздел 5), путей ТАРС/флага/топиков и промптов — не измерены, помечены «оценка».
- Не запускались `markdownlint` (в репозитории не настроен) и полный `pytest -v`.
