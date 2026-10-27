# Карта удаления старого пути диалога (v1) для PR-15 (#3711) — состояние на 27.10.2026

Карточка PR-15 эпика #3683: [#3711](https://github.com/krikz/rob_box_project/issues/3711) (текст и комментарий про
джойстик teleop), хвосты из [#3704](https://github.com/krikz/rob_box_project/issues/3704) (PR-10b, удаление
`DialogueStateMachine`), [#3705](https://github.com/krikz/rob_box_project/issues/3705) (PR-11),
[#3706](https://github.com/krikz/rob_box_project/issues/3706) (PR-12a). План ADR:
[`docs/adr/0150-dialog-v2-session-command-event.md`](../adr/0150-dialog-v2-session-command-event.md) §11 PR-15, §12, §14 A4/A6/A8.
Парный документ — [`acceptance_2026-10-23.md`](acceptance_2026-10-23.md) (скелет приёмки PR-14).

**Пересверка 27.10.2026** на `origin/develop @ d076b8b55` (предыдущая карта — `8178e2809` от 23.10).
Между картами влито ~40 коммитов в диалоговые файлы, включая #3912 (перенос логики meeting/startup/status_feed в `rob_box_dialog`),
#3708 PR-13b (ТАРС на v2), марафонные фиксы v2 (#3953–#4068). Ключевые изменения размеров и метрик — в секциях ниже с пометкой «Δ 27.10».
Команды пересверки — PowerShell (Windows), эквиваленты `wc -l` / `grep` — в приложении.

## 1. Что изменилось между картами (23.10 → 27.10)

1. **`meeting.py` 213→42 строки**, **`startup_greeting.py` 41→9**, **`tars_status_feed.py` 115→27** — #3912 перенёс логику
   в `rob_box_dialog.meeting`, `rob_box_dialog.startup`, `rob_box_dialog.status_feed`. Модули v1 стали тонкими обёртками
   (реэкспорт из `rob_box_dialog` + обвязка `DialogueNode`). **Решение D1 (перенос) выполнено до нас** — удаление этих
   обёрток тривиально (15b сокращается с 523 до ~78 строк).
2. **`dialogue_node.py` 4700→4403** (−297 строк), **`stt_admission.py` 840→683** (−157), **`speak_helpers.py` 594→509**
   (−85), **`dialogue_guards.py` 169→146** (−23), **`dialogue_helpers.py` 142→118** (−24), **`dialogue_state_machine.py`
   469→378** (−91), **`speech_compat.py` 74→58** (−16). Суммарно v1-множество похудело на ~700 строк.
3. **`Bug [A-F]` 26→5** (в `src/rob_box_voice` + `src/rob_box_dialog`): почти готово — осталось 5 вхождений.
4. **`#NNNN` в `rob_box_dialog` 158→384**: стало больше (новые ссылки на issue в комментариях v2). Цель ≤20 после 15v
   остаётся.
5. **`RULE #` без изменений**: 8 в `master_prompt_compact.txt` + 7 в `telegram_chat_prompt.txt` = 15.
6. **Хвост #3704 (DSM)**: `DialogueStateMachine` на месте (378 строк), grep по `DialogueStateMachine|_pause_reason` —
   10 вхождений в `agent_core.py`, `assembly.py`, `dialogue_state_machine.py`. Критерий «grep = 0» не достигнут.
7. **Хвост #3705 (voice_memory.db)**: 45→10 вхождений (в основном комментарии). `_HISTORY_MARKER_RE` и `_META_PREFIX_RE`
   — 0 (уже удалены). `discard_last_reply` — 0. Маркеры ушли с сокращением `speak_helpers.py`.
8. **Хвост #3706 (TG)**: `tg_chat_id` — 4 вхождения (`utterance.py` комментарий, `telegram_node.py`, `speak_helpers.py`,
   `speech_compat.py`). `[TG:` и `parse_tg_prefix` — 0.
9. **#3708 PR-13b влит** (`58187f868`): `read_logs`/`ros2_node_status`/`container_status` для ТАРС, срез `operator.admin`.
   Путь v1 `AgentCore` в `supervisor_node.py` ещё на месте.
10. **`dialog_engine` флаг**: 44 строки в 17 файлах (как и было). `dialog_engine.yaml` ×2 на месте. Значение по умолчанию
    `v2` (`src/rob_box_voice/config/dialog_engine.yaml`).
11. **Мёртвые модули**: `llm_config.py` 218 строк, `voice_memory_init.py` 92, `skip_reason_checker.py` 26,
    `utils/github_summary.py` 242 — на месте, читателей нет (D6).
12. **`conversation_history.py`** 255→216 строк — на месте, читателей нет.

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
| `src/rob_box_voice/rob_box_voice/dialogue_node.py` | 4403 | 1 (`stt_admission_host.py`, под `TYPE_CHECKING`); `setup.py:49`, 2 launch; grep 131 / 166. Тестов 47 (10 650 строк) | `dialog_node.py` (452) + `core/dialog_v2_ports.py` (242) + `rob_box_dialog/turn.py` | `ls .../dialogue_node.py` → нет; `grep -rn "dialogue_node import\|rob_box_voice.dialogue_node" src docker scripts --include=*.py --include=*.yaml --include=*.sh` вне комментариев = 0; `grep -n dialogue_node src/rob_box_voice/setup.py` = 0 | 15a–15p (по методам) | высокий: корень v1; режется 16 частями. Δ 27.10: −297 строк |
| `config/dialogue_node.yaml` ×2 (`src/rob_box_voice/config/`, `docker/sound/config/voice_assistant/`) | 141 + 141 | `launch×2`, `startup_greeting` (поле в yaml) | `config/dialog_node.yaml` | `ls` обеих копий → нет | 15p | низкий |
| `core/dialogue_stage_mixin.py` | 81 | 1 (`dialogue_node.py`) / 1 / 2 | `rob_box_dialog/turn_log.py`, `core/dialog_v2_ports.py` | `grep -rlw dialogue_stage_mixin src --include=*.py` вне тестов = 0 | 15e | низкий |
| `core/turn_stages.py` | 275 | 2 (`dialogue_node`, `dialogue_stage_mixin`) / 3 / 3. Тестов 2 (907) | стадии хода и рефлексы: `core/dialog_v2_ports.py` + `rob_box_dialog/expression.py` | `grep -rlw turn_stages …` = 0 | 15e | средний: рефлексы `addressed/thinking/…` (A17) должны остаться в v2 — сверить |
| `core/turn_tools.py` | 111 | 2 (`dialogue_stage_mixin`, `turn_stages`) / 2 / 1 | `rob_box_dialog/execute.py` (исполнение) | `grep -rlw turn_tools …` = 0 | 15e | низкий |
| `core/turn_reflexes.py` | 130 | 2 (`turn_stages`, `turn_tools`) / 2 / 1 | `rob_box_dialog/expression.py` (`reflex()`) | `grep -rlw turn_reflexes …` = 0 | 15e | низкий |
| `core/turn_context.py` | 68 | 2 (`dialogue_node`, `tier1_host`) / 4 / 4 | `rob_box_dialog/context.py` | `grep -rlw turn_context src --include=*.py` вне тестов — только `llm_adapter.py` (другая сущность, проверить) | 15c | низкий |
| `core/turn_identity.py` | 146 | 1 (`dialogue_node`) / 2 / 2. Тестов 2 (359) | раздел `speaker` снимка: `rob_box_dialog/speaker_adapter.py`, `context.py` | `grep -rlw turn_identity …` = 0 (останется упоминание в `knowledge.py` — комментарий) | 15h | средний: `user_profile_lines` (имя, профиль) — убедиться, что v2 кладёт то же в контекст |
| `core/turn_speech.py` | 163 | 1 (`dialogue_node`) / 2 / 1. Тестов 1 (94) | `rob_box_dialog/respond.py` (фраза по событию) | `grep -rlw turn_speech …` = 0 | 15h | низкий. `final_text_for_speech` уходит, нормализация TTS остаётся в `tts_node` (ср. `speak_helpers`) |
| `core/stt_admission.py` | 683 | 4 (`dialogue_node`, `dialogue_stage_mixin`, `stt_admission_host`, `tier1_host`) / 6 / 8. Тестов 6 (2 156) | `rob_box_dialog/address.py`, `backlog.py`, `grammar.py`; `stt_node` → `InterruptRequest` | `grep -rlw stt_admission …` вне тестов = 0; `grep -rn "parse_tg_prefix" src` вне тестов = 0 (сейчас 0) | 15a (шаги 545–683) + 15m (ядро 1–544) | средний. Δ 27.10: −157 строк |
| `core/stt_admission_host.py` | 261 | 1 (`dialogue_node`) / 1 / 0 | `rob_box_voice/dialog_node.py` (тонкий хост) | `grep -rlw stt_admission_host …` = 0 | 15d | низкий (тестов нет) |
| `core/tier1_host.py` | 230 | 3 (`dialogue_node`, `stt_admission_host`, `tools_view_host`) / 3 / 3. Тестов 2 (432) | `rob_box_dialog/grammar.py` + `interrupt.py` | `grep -rlw tier1_host …` = 0 | 15c | средний: «спорный стоп» (В19) — проверить, что у v2 есть аналог `stop_verdict` |
| `core/tools_view_host.py` | 34 | 1 (`dialogue_node`) / 1 / 2 | `rob_box_dialog/tools_view.py` | `grep -rlw tools_view_host …` = 0 | 15c | низкий |
| `core/stop_chooser.py` | 85 | 2 (`dialogue_node`, `tier1_host`) / 4 / 2. Тестов 1 (112) | `rob_box_dialog/interrupt.py` (`stop_verdict`) | `grep -rlw stop_chooser src --include=*.py` вне тестов = 0 (ссылки в `identity_dedup.py`, `interrupt_request.py` — комментарии/имя класса, проверить) | 15c | средний |
| `core/greeting_window.py` | 79 | 3 (`dialogue_stage_mixin`, `meeting`, `stt_admission_host`) / 4 / 2. Тестов 1 (154) | `rob_box_dialog/greeting_reply.py` (окно ответа на приветствие) | `grep -rlw greeting_window …` = 0 | 15d | средний: окно ответа 15 с (решение Шифу 21.10) — сверить поведение |
| `core/identity_dedup.py` | 83 | 1 (`dialogue_node`) / 2 / 1 | `rob_box_dialog/speaker_adapter.py` | `grep -rlw identity_dedup …` = 0 | 15f | низкий |
| `core/session_epoch.py` | 62 | 1 (`dialogue_node`) / 1 / 5. Тестов 2 (197) | эпоха — у `tts_node` (`SpeechCancel`, `interrupt.py`) | `grep -rlw session_epoch …` вне тестов = 0 | 15j | средний: «0 действий старой эпохи» (A12) должна держаться на стороне владельца речи |
| `core/media_plan_run.py` | 128 | 1 (`dialogue_node`) / 3 / 8. Тестов 8 (1 309) | `rob_box_dialog/plan/*` + `core/plan_host.py` | `grep -rlw media_plan_run src --include=*.py` вне тестов = 0 (остаётся имя в `media_router.py` — комментарий) | 15i | средний: 8 тестовых файлов — 3 из них могут проверять общую `media_router` и требуют переноса, а не удаления |
| `core/library_command.py` | 63 | 1 (`media_plan_run`) / 3 / 1 | `core/media_router.py` (остаётся, используется `music_turn`) | `grep -rlw library_command …` = 0 | 15i | низкий |
| `core/meeting.py` | 42 | 1 (`dialogue_node`) / 6 / 4. Тестов 2 (432) | **#3912**: логика в `rob_box_dialog.meeting`; здесь только обвязка `greet_node_on_meeting` | `ls` → нет | 15b | **низкий** (Δ 27.10: 213→42, логика уже в v2) |
| `startup_greeting.py` | 9 | 1 (`dialogue_node`) / 2 / 3. Тестов 3 (456) | **#3912**: `rob_box_dialog.startup`; здесь реэкспорт | `ls` → нет | 15b | низкий (Δ 27.10: 41→9) |
| `core/tars_status_feed.py` | 27 | 1 (`dialogue_node`) / 1 / 1. Тестов 1 (70) | **#3912**: `rob_box_dialog.status_feed` + `core/dialog_v2_parity`; здесь обвязка | `ls` → нет | 15b | низкий (Δ 27.10: 115→27) |
| `speaker_profiles.py` | 181 | 1 (`dialogue_node`) / 2 (`epithets.py` — в комментарии) / 2. Тестов 2 (439) | `rob_box_dialog/speaker_adapter.py` (частично; `SpeakerTracker` — нет аналога по имени) | `ls` → нет | 15g | средний |
| `core/conversation_history.py` | 255 | 0 прямых (только реэкспорт в `core/__init__.py`; `voice_memory.py:28,30` обращается к атрибуту `self.conversation_history`, а не к классу) / 4 / 2. Тестов 1 (250) | `rob_box_dialog/session.py` (кадры) + `context.py` | `ls` → нет; `grep -n conversation_history src/rob_box_voice/rob_box_voice/core/__init__.py` = 0 | 15a | низкий (читателей нет) |
| `core/speech_compat.py` | 74 | 2 (`tts_node`, `speech_owner`) / 6 / 2. Тестов 2 (562) | `rob_box_core/speech_client.say` (SpeechRequest) | `ls` → нет; `grep -rn "speech_compat\|attach_legacy" src` вне тестов и комментариев = 0 | 15q (после teleop 15-0d) | высокий: последний писатель `/voice/tts/request` — джойстик teleop (Main Pi) |
| `core/speak_helpers.py` | 509 → остаётся ≈ 97 | 2 (`dialogue_node`, `tts_node`) / 5 / 13. Тестов 11 (1 627) | нормализация TTS остаётся в `tts_node`; фраза о недоступном языке — `rob_box_dialog/phrases` | `grep -c "re.compile" …/speak_helpers.py` → 18 → цель 0 (остаются `strip_bracket_asides`, `strip_markdown`, `unsupported_language_notice`, `provider_speaks` ≈ 97 строк функций) | 15q | средний. Δ 27.10: 594→509 (−85) |
| `core/dialogue_guards.py` | 146 → остаётся ≈ 27 функций + 2 регекса и константа | 5 (`dialogue_node`, `stt_admission_host`, `track_start_guard`, `tts_node`, `turn_speech`) / 10 / 4. Тестов 3 (325) | защита SSML нужна `tts_node`; `MUSIC_STARTING_TOOLS` — `track_start_guard` | `grep -c "re.compile" …` 2 → 2 (см. D2) | 15s | средний. Δ 27.10: 169→146 (−23) |
| `core/dialogue_helpers.py` | 118 → остаётся 19 (`sanitize_speaker_name`) | 2 (`dialogue_node`, `speaker_turn`) / 4 / 3. Тестов 1 (176) | `speaker_turn` (v2) остаётся читателем | `grep -rn "EMOTION_TO_ANIMATION\|detect_pitch_intent\|generate_fallback_response" src` вне тестов = 0 | 15r | низкий. Δ 27.10: 142→118 (−24) |
| `core/dialogue_text.py` | 48 | 2 (`dialogue_node`, `stt_node`) / 5 / 12 | остаётся (читает `stt_node`) | — | не удаляется | — |
| `src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py` | 378 | 8 (`__init__`, `agent_core`, `assembly`, `meeting`, `stt_admission_host`, `tier1_host`, `supervisor_node`, `dialogue_node`) / 10 / 37. Тестов 35 (12 315, с пересечением с тестами ноды) | кадры стека: `rob_box_dialog/session.py`, `scenes.py`; `DialogControl.srv` (PR-10a #3851) | `ls` → нет; `grep -rn "DialogueStateMachine\|dialogue_state_machine\|_pause_reason\|check_silence_timeout" src --include=*.py` вне тестов: **10 → 0** | 15r | высокий. Δ 27.10: 469→378 (−91) |

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
| **15a** | `core/conversation_history.py` (216) + реэкспорт в `core/__init__.py`; хвост `core/stt_admission.py` строки 545–683 (шаги допуска и `default_steps`, 138) | 354 | 1 + часть 6 тестов допуска | voice-assistant | — | Δ 27.10: 551→354. Смоук: «Робот, который час», «Робот, сколько у тебя заряда», «Робот, расскажи короткую шутку» |
| **15b** | `core/meeting.py` (42), `startup_greeting.py` (9), `core/tars_status_feed.py` (27); методы `_on_startup_greeting*`, `_subscribe_vision_events`, `_on_vision_event`, `_cancel_greeting_timer` (154) | 232 | 6 | voice-assistant | **D1 выполнено #3912** | Δ 27.10: 523→232. Логика уже в `rob_box_dialog`; здесь только обвязка `DialogueNode`. Смоук: знакомый в кадре → приветствие; рестарт → стартовая фраза; ТАРС показывает статус |
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
| **15m** | ядро `core/stt_admission.py`, строки 1–544 (типы, протоколы, `SttAdmission`) | 544 | 5 | voice-assistant | 15d, 15a | Δ 27.10: файл 840→683, ядро ~544. Как 15d; плюс `Робот` без продолжения через 20 с |
| **15n** | сборка и разбор настроек: `_parse_provider_chain` … `_music_state_mem`, `parameters_callback` (509) | 509 | оценка 3 | voice-assistant | 15l | старт ноды с каждым провайдером цепочки (`llm_providers`): 3 фразы, по одной на провайдера |
| **15o** | `_declare_params`, `shutdown_asyncio_loop`, `destroy_node`, `main` (230) + импорты/шапка 1–297 | 527 | — | voice-assistant | 15n | смоук + корректная остановка контейнера (`docker stop` ≤ 10 с, без `Traceback`) |
| **15p** | `__init__` (484) и остаток шапки (≈ 253 вне методов); сам `dialogue_node.py`; entry point `setup.py:49`; `config/dialogue_node.yaml` ×2 (141 + 141); ветка `dialogue_node` в launch×2; запись `dialogue_node` в `ownership.yml` | ≤ 737 + 282 (конфиги); фактически меньше (части a–o вынули часть `__init__`) | оставшиеся тесты ноды | voice-assistant | 15a–15o | смоук + 10 фраз набора; `ls src/rob_box_voice/rob_box_voice/dialogue_node.py` → нет; `grep -c "Bug [A-F]\b" dialogue_node.py` — файла нет |
| **15q** | `core/speech_compat.py` (58), `core/speak_helpers.py` (509 → ≈ 97: −412); `attach_legacy` в `tts_node.py:1250`, `meta_from_legacy` в `speech_owner.py:100` | 470 + хвосты `tts_node` (оценка 25) | 13 | voice-assistant | 15-0d, 15p | Δ 27.10: 571→470. `/say` из Telegram, скороговорка с markdown, джойстик |
| **15r** | `harness/core/dialogue_state_machine.py` (378) и его читатели (`agent_core`, `assembly`, `__init__`, `supervisor_node`); путь v1 `AgentCore` ТАРС в `supervisor_node.py` (188) и `agent_path` v1-ветка; `core/dialogue_helpers.py` (118 → 19: −99) | 378 + 99 + 188 ≈ 665 → делится на 15r1 (DSM + dialogue_helpers = 477) и 15r2 (путь v1 ТАРС ≈ 190–250, оценка) | 37 (DSM) | voice-assistant, **supervisor** | 15p | Δ 27.10: 780→665. ТАРС: 8 команд как в #3708; `hold/resume`; «замолчи» → «говори» |
| **15s** | флаг `dialog_engine`: `rob_box_core/dialog_engine.py` (77), 44 строки в 17 файлах (`tts_node`, `dialog_node`, `telegram_node`, `supervisor_node`, `operator_turns`, `operator_admin`, `node_monitor`, launch×2, `setup.py`, `start_supervisor.sh`, `start_telegram_bot.sh`), `dialog_engine.yaml` ×2; ветка v1 `_subscribe_replies` в `telegram_node`; `dialogue_guards.py` 169 → ≈ 70 | оценка 280–350 | оценка 4 | voice-assistant, supervisor, telegram-bot, **led-matrix/teleop — не затрагиваются** | 15p, 15r | рестарт всех трёх контейнеров: 3 фразы голосом, 3 сообщения в Telegram (`Клод …`, `/say`), 3 команды ТАРС; `ros2 param get /dialog_node dialog_engine` → параметра нет |
| **15t** | String-топики: публикации `/voice/tts/state` и `/voice/tts/finished` в `tts_node` (≈ 12 упоминаний), запись об адаптере в `ownership.yml`, записи `multiple_writers` / `dead_output` в `runtime-baseline.json`, `seam_baseline.json`, `seam_allowlist.json` | оценка 150–250 | оценка 5 | voice-assistant | 15-0b, 15-0c, 15q | `ros2 topic info` по каждому: писателей 0; «L: Architecture Audit» без `multiple_writers` по диалогу (A6); 5 фраз с LED-кольцом, лицом, шлемом |
| **15u** | промпты: `RULE #` 15 → 0 в `master_prompt_compact.txt`, `telegram_chat_prompt.txt`; генерация из `knowledge.prompt_fragment()` + персона | оценка 80–150 (правка текста) | 5 тестов промпта | voice-assistant | D3 | **A/B на живом v2**: набор честности ≥ 60 фраз до и после (A3 не хуже), A1/A14 по `turn_metrics.py` |
| **15v** | `#NNNN` в `rob_box_dialog/rob_box_dialog`: 384 → ≤ 20 (правка комментариев); `Bug [A-F]` в оставшихся → 0 (сейчас 5) | оценка 140 строк правок | — | образ не меняется по поведению | 15p | Δ 27.10: #NNNN 158→384 (новые ссылки в v2). `pytest -v src/rob_box_dialog`, смоук |
| **15w** | тесты по номерам issue: 77 `test_issue_*` в `rob_box_voice/test`, 28 из них удаляются с v1-модулями, 49 переписываются на поведение модулей (+30 в других пакетах src, +17 в `tests/`); `CONTEXT.md` по §13; пересчёт baseline `cc_budget`/`class_budget`/`seam` | оценка 200–400 (CONTEXT.md, baselines) | 49 + 12 смешанных переписать | — | 15v | `pytest -v` всех пакетов, 5 гардов (`cc_budget`, `class_budget`, `seam_without_consumer`, `cc_budget_refs`, `dialogue_skip_reasons`), `L: Architecture Audit` |

12 «смешанных» тестов (импортируют и v1-модули, и `rob_box_dialog`/`dialog_node`) нельзя просто удалить, их надо переписать
(список в приложении).

## 7. Решения, нужные от товарища Шифу до старта PR-15

- **D1.** Паритет v2: приветствие по встрече лицом (`meeting.py`), стартовая фраза, фид статуса ТАРС, `SpeakerTracker`.
  **Δ 27.10: выполнено #3912** — логика в `rob_box_dialog.meeting`/`startup`/`status_feed`, модули v1 стали обёртками
  (42+9+27=78 строк). `SpeakerTracker` — решение Шифу 23.10 «удалить» (D1a). **D1 больше не блокирует 15b.**
- **D2.** A8 «`re.compile` → 0»: две защитные регулярки SSML в `tts_node` (`dialogue_guards.py`) и 18 в `speak_helpers.py`
  (из них функции `strip_bracket_asides`, `strip_markdown`, `unsupported_language_notice` нужны `tts_node`) — простой перенос не даёт 0;
  принять «≤ N, но не в слое диалога» или переписать структурно? **Без изменений.**
- **D3.** `RULE #` = 0 меняет промпт живого v2 (15 вхождений в двух файлах): допустим A/B-прогон набора честности до/после.
  **Без изменений.**
- **D4.** `context_aggregator_node` читает `/voice/dialogue/response` («ответы робота» в контекст перцепции): заменить на `/dialog/output` /
  `SpeechEvent` или отказаться от этого входа? **Без изменений.**
- **D5.** После 15s откат `dialog_engine: v1` невозможен — подтвердить, что откатываться будем гитом, а не флагом.
  **Без изменений.**
- **D6.** Мёртвые модули без читателей вне карточки (`llm_config.py` 218, `voice_memory_init.py` 92, `skip_reason_checker.py` 26,
  `utils/github_summary.py` 242) — включать в 15a? **Без изменений (размеры обновлены).**
- **D7.** Критерий #3705 `voice_memory.db`: grep 45→10 вхождений (в основном комментарии). `_HISTORY_MARKER_RE` и `_META_PREFIX_RE` — 0.
  `discard_last_reply` — 0. Закрывается в PR-15 (маркеры уже ушли), `voice_memory.db` — отдельная карточка.
- **D8.** Джойстик teleop пишет `/voice/tts/request` через `speech_compat.py`. **Без изменений.**

## 8. Хвосты PR-10b, PR-11, PR-12a (по комментариям эпика) — состояние grep на develop @ d076b8b55

| Карточка | Критерий из карточки | Сейчас (вне тестов) | Что остаётся | Часть |
|---|---|---|---|---|
| #3704 PR-10b | `ls …/dialogue_state_machine.py` → нет; `grep -rn '"/dialogue/control\|_pause_reason\|check_silence_timeout' src --include=*.py` = 0 | файл есть (378 строк); grep: 10 вхождений (`dialogue_state_machine.py`, `agent_core.py`, `assembly.py`). Δ 27.10: было 34, стало 10 | DSM, `/dialogue/control*`, `_pause_reason` | 15r (+15j) |
| #3705 PR-11 | `grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py` = 0 | `_HISTORY_MARKER_RE` 0, `_META_PREFIX_RE` 0, `_SPEAKER_TAG_RE` 0, `discard_last_reply` 0; `voice_memory.db` 10 (комментарии). Δ 27.10: было 59, стало 10 | `voice_memory.db` — отдельная карточка (D7) | 15h, 15k, 15q (маркеры уже ушли) |
| #3706 PR-12a | `grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py` = 0; `grep '"/voice/dialogue/response"\|"/voice/sound/stop"' src/rob_box_telegram` = 0 | `tg_chat_id` 4 (`utterance.py`, `telegram_node.py`, `speak_helpers.py`, `speech_compat.py`); `[TG:` 0; `parse_tg_prefix` 0. Δ 27.10: было 51, стало 4 | v1-путь Telegram: `telegram_node._subscribe_replies` | 15s (+15a/15m, 15h) |
| #3708 PR-13b | влит `58187f868` — не хвост PR-15 | — | — | — |

## 9. Сводка (Δ 27.10)

- **Удаляется целиком или сокращается (измерено 27.10):** файлы v1 — ~7 644 строки (`dialogue_node.py` 4 403 + 21 модуль ~3 241 +
  `conversation_history.py` 216) в частях 15a–15p; `speech_compat.py` 58 + сокращение `speak_helpers.py` на ~412 (15q);
  `dialogue_state_machine.py` 378 + сокращение `dialogue_helpers.py` на ~99 (15r) = **~8 591 строка** (было 9 507, −916).
  Не измерено (оценка): путь v1 ТАРС 190–250, флаг `dialog_engine` и `dialogue_guards` 280–350,
  String-топики в `tts_node` 150–250, комментарии `#NNNN` 140, промпты 80–150, CONTEXT.md и baseline 200–400,
  конфиги `dialogue_node.yaml` 282.
- **Тесты, уходящие вместе с v1:** 72 файла, ~16 136 строк (AST-импорт v1-модулей, без DSM-тестов, которые пересекаются);
  35 файлов (12 315 строк) используют `DialogueStateMachine`; 12 «смешанных» тестов переписываются.
- **Частей:** удаления **16** (15a–15p) + **7** (15q, 15r1, 15r2, 15s, 15t, 15u, 15v) + 15w = **24**; перед ними **6**
  подготовительных (15-0a…15-0f). 15b сократилась с 523 до ~78 строк (D1 выполнено #3912).
- **Какие образы пересобирать:** voice-assistant — все части 15a–15u; **supervisor** — 15-0a, 15-0c, 15r, 15s; **quest** —
  15-0a, 15-0c; **telegram-bot** — 15-0e, 15s; **led-matrix (Vision Pi)** — 15-0b; **teleop (Main Pi)** — 15-0d;
  **perception** — 15-0e (если решено по D4).
- **Что нужно решить до старта:** D2–D6, D8 (раздел 7). D1 выполнено #3912.

## Приложение. Команды и вывод (27.10.2026, PowerShell)

Среда: worktree на `origin/develop @ d076b8b55`, Windows. Предыдущая карта — `8178e2809` (23.10).

### А. Размеры файлов (PowerShell: `Get-Content <f> | Measure-Object -Line`)

| Файл | 23.10 | 27.10 | Δ |
|---|---|---|---|
| `dialogue_node.py` | 4700 | 4403 | −297 |
| `dialog_node.py` | 444 | 452 | +8 |
| `core/dialog_v2_ports.py` | 225 | 242 | +17 |
| `core/stt_admission.py` | 840 | 683 | −157 |
| `core/speak_helpers.py` | 594 | 509 | −85 |
| `core/dialogue_guards.py` | 169 | 146 | −23 |
| `core/dialogue_helpers.py` | 142 | 118 | −24 |
| `core/speech_compat.py` | 74 | 58 | −16 |
| `core/conversation_history.py` | 255 | 216 | −39 |
| `core/meeting.py` | 213 | 42 | −171 |
| `startup_greeting.py` | 41 | 9 | −32 |
| `core/tars_status_feed.py` | 115 | 27 | −88 |
| `speaker_profiles.py` | 181 | 155 | −26 |
| `core/turn_stages.py` | 275 | 231 | −44 |
| `core/turn_tools.py` | 111 | 88 | −23 |
| `core/turn_reflexes.py` | 130 | 110 | −20 |
| `core/turn_context.py` | 68 | 62 | −6 |
| `core/turn_identity.py` | 146 | 122 | −24 |
| `core/turn_speech.py` | 163 | 127 | −36 |
| `core/stt_admission_host.py` | 261 | 216 | −45 |
| `core/tier1_host.py` | 230 | 197 | −33 |
| `core/tools_view_host.py` | 34 | 26 | −8 |
| `core/stop_chooser.py` | 85 | 65 | −20 |
| `core/greeting_window.py` | 79 | 64 | −15 |
| `core/identity_dedup.py` | 83 | 65 | −18 |
| `core/session_epoch.py` | 62 | 48 | −14 |
| `core/media_plan_run.py` | 128 | 106 | −22 |
| `core/library_command.py` | 63 | 47 | −16 |
| `core/dialogue_stage_mixin.py` | 81 | 65 | −16 |
| `core/dialogue_text.py` | 48 | 37 | −11 |
| `core/llm_config.py` | 257 | 218 | −39 |
| `core/voice_memory_init.py` | 113 | 92 | −21 |
| `core/skip_reason_checker.py` | 35 | 26 | −9 |
| `utils/github_summary.py` | 297 | 242 | −55 |
| `harness/…/dialogue_state_machine.py` | 469 | 378 | −91 |

### Б. Ключевые grep-метрики

```powershell
# RULE # (без изменений)
Select-String -Path "src/rob_box_voice/prompts/*.txt" -Pattern "RULE #" -AllMatches
# → master_prompt_compact.txt: 8, telegram_chat_prompt.txt: 7

# #NNNN в rob_box_dialog (158→384)
Get-ChildItem src/rob_box_dialog/rob_box_dialog -Recurse -Include *.py |
  Select-String -Pattern "#[0-9]{4}" -AllMatches | Measure-Object | % Count
# → 384

# Bug [A-F] (26→5)
Get-ChildItem -Path src/rob_box_voice,src/rob_box_dialog -Recurse -Include *.py |
  Select-String -Pattern "Bug [A-F]" -AllMatches | Measure-Object | % Count
# → 5

# DSM grep (34→10)
Get-ChildItem -Path src -Recurse -Include *.py |
  Select-String -Pattern "DialogueStateMachine|dialogue_state_machine|_pause_reason|check_silence_timeout"
# → 10 вхождений (agent_core.py 3, assembly.py 2, dialogue_state_machine.py 5)

# voice_memory.db (45→10)
Get-ChildItem -Path src -Recurse -Include *.py |
  Select-String -Pattern "voice_memory\.db"
# → 10 (в основном комментарии)

# tg_chat_id (51→4)
Get-ChildItem -Path src -Recurse -Include *.py |
  Select-String -Pattern "tg_chat_id"
# → 4 (utterance.py, telegram_node.py, speak_helpers.py, speech_compat.py)

# dialog_engine (без изменений)
Get-ChildItem -Path src -Recurse -Include *.py,*.yaml,*.sh |
  Select-String -Pattern "dialog_engine" | Measure-Object | % Count
# → ~30 строк в 17 файлах
```
