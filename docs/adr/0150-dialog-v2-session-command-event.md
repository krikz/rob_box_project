# ADR-0150: Новый виток диалоговой системы (dialog v2) — сессия с одним владельцем, команда вместо свободного ответа, фраза об исполнении из события

| Поле | Значение |
|---|---|
| Статус | Proposed (на утверждение товарищу Шифу). Только дизайн и план поставки, код не меняется. Ревизия 2 (02.10.2026) — по решениям Шифу В1–В11, см. §18. Ревизия 3 (02.10.2026) — планировщик ресурсов и одновременное исполнение (§5.6) по материалу `07-action-scheduler.md` |
| Дата | 2026-10-01; ревизии 2026-10-02 |
| Автор | Claude Code (шисюн), по брифу товарища Шифу `docs/research/dialog-v2/00-brief.md` |
| Issue | эпик — завести при принятии (аналог #3312 для музыки); входные карточки: #3266, #3271, #3297, #3145, #3144, #3108, #3109, #3000, #2754, #3269, #3296 (все open на 01.10) |
| Основание | `docs/research/dialog-v2/01…07` (в git; `07-action-scheduler.md` — планировщик, хотелки Х1–Х15); аудиты CI run 36918223549 (статика, 01.10) и 36689419795 (рантайм, 30.09); код сверен на этом worktree, HEAD `0e7371fed` (ревизия 1 — `61fd84ff7`); история git по В1/В2/В10 — §18.1 |
| Родители | ADR-0148 (код решает, LLM говорит — обязателен), ADR-0149 (музыка v2: владелец плеера, события `started/rejected`, флаг `music_engine`), ADR-0141, ADR-0131 (`utterance_id`), ADR-0102/0103 («Повод»), ADR-0083 (`build_agent`), ADR-0051 (ТАРС), ADR-0145 (бюджет классов), ADR-AF-0013 (мелкие PR), ADR-0018 (честный FAIL) |
| Заменяет после приёмки | ADR-0084 целиком; ADR-0021 целиком (R1–R5); ADR-0066 §2–§3 (pause/resume без владельца → типизированный `DialogControl` с полем `set_by` и TTL); ADR-0143 (ретраи Bug B/C как норма → режим команд по возможностям провайдера); ADR-0129-dj §2 (штамп персоны в промпт → кадр сцены); ADR-0140 остаток; ADR-0065 §2 (списки вейк-слов в `dialogue_text.py` → `rob_box_dialog.knowledge`); ADR-0001 §Telegram-харнес (телеграм-агент на общем движке, §9.2); ADR-0011 (action protocol: HTTP-сайдкар и PASTE — заменяются планировщиком ресурсов §5.6, берётся контракт accepted/feedback/result/cancel и `commit` пред-генерации); ADR-0033 (MERGE не трогает музыку — заменяется таблицей `COMPOSITION`); ADR-0056/0092 §pregenerate (поле в payload чанка никто не публикует — заменяется пред-генерацией выступления §5.6.3). Подробно — §13 |

---

## Оглавление

0. Что проверено, что нет; 0.1 расхождения материалов с кодом
1. Проблема: классы отказов и семь корней
2. Решение: границы, модули, владельцы
3. Поток одной реплики (как будет)
4. Что решает код, что — LLM; 4.4 канал речи: бейк-офф; 4.5 провайдер-агностик
5. Исполнение, событие, фраза; 5.5 экспрессия и живость без LLM; **5.6 исполнение во времени: планировщик ресурсов** (ресурсы и владельцы, матрица одновременности, класс «выступление», политики APPEND/MERGE/REPLACE/REJECT, синхронизация с тактом, эталонные сценарии X3 и #993)
6. Речь, перебивание, тишина (перебивание — по классу текущего действия)
7. Сессия, сцены, персоны, история, память
8. Провайдеры: возможности и честная деградация
9. Интерфейс с музыкой (ADR-0149), Telegram-агент, ТАРС
10. Типы сообщений: уход от JSON в `std_msgs/String`
11. Миграция: strangler за флагом `dialog_engine`, план PR (в т.ч. PR-7b/7c планировщик), зависимости от #3312
12. Таблица костылей на удаление (в т.ч. планировщик: мёртвые хуки, второй планировщик в tts_node, пять гардов исполнителя)
13. Какие ADR замещаются (в т.ч. ADR-0011); что поправить в CONTEXT.md
14. Приёмка числами (A1–A23) и трассировка хотелок Х1–Х15
15. Что сохранить из старого
16. Риски и откат
17. Альтернативы (в том числе отвергнутое окно адресации без вейка — с доказательством)
18. Решения владельца (02.10.2026) и 18.1 находки в истории; 18.2 что осталось открытым
19. Что не проверено

---

## 0. Что проверено, что нет

- Прочитаны целиком: `00-brief.md`, `01-bug-archaeology.md` (488 строк), `02-current-architecture.md` (398), `03-kludge-inventory.md` (286), `04-adr-landscape.md` (223), `05-oss-landscape.md` (402), `06-architecture-audit-digest.md` (185), ADR-0148, ADR-0149, `AGENTS.md`.
- **Сверено мной по коду на HEAD `61fd84ff7`/`0e7371fed`** (grep/Read; ниже каждое помечено «(проверено)»):

| # | Утверждение | Результат на HEAD |
|---|---|---|
| V1 | `dialogue_node.py` 9 954 строки; `dialogue_guards.py` 3 063; `music_guard.py` 876; `tts_node.py` 7 316; `agent_core.py` 2 690 | `wc -l` — совпало |
| V2 | `re.compile` в `dialogue_guards.py` = 34, `skill_router.py` = 27, `speak_helpers.py` = 15 | совпало; в `music_guard.py` — **0** (см. §0.1) |
| V3 | Меток `Bug A–F`: 113 в `dialogue_node.py`, 57 в `dialogue_guards.py`; ссылок `#NNNN` в `dialogue_node.py` 705 (144 уникальных) | совпало |
| V4 | `DEFAULT_SYNTHETIC_RETRIES = 2` (`dialogue_node.py:7861`) против `DEFAULT_MAX_SYNTHETIC_RETRIES = 3` (`core/turn.py:36`) | совпало |
| V5 | `_MAX_TOOL_ITERATIONS = 8` (`agent_core.py:87`), цикл `:1598` | совпало |
| V6 | `skill_tool_narrowing: false` в обоих yaml (`docker/vision/config/voice_assistant/dialogue_node.yaml:112`, `src/rob_box_voice/config/dialogue_node.yaml:43`) | совпало |
| V7 | `music_engine: "v1"` (`mcp_server.yaml:12`), выбор движка `mcp_server.py:299-327` | совпало |
| V8 | «Принял.» на пустой ответ — `dialogue_node.py:8508` | совпало |
| V9 | `/voice/current_dialogue_id`: подписчики `tts_node.py:1308`, `tools/dialogue.py:109`; писателя нет, `dialogue_node.py:9456` только присваивает атрибут | совпало |
| V10 | `DEFAULT_WAKE_WORDS` — 26 вариантов, включая «бот», «роб», «рома», «робота» (`dialogue_text.py:31-75`); `is_silence_command` — подстрока (`:215-218`); `stt_node.py:226-240` — запасная копия с подстрочным матчем и «пустой список = пропускать всё» | совпало |
| V11 | Писатели `/voice/tts/request`: `tools/dialogue.py:99`, `tools/say.py:61`, `telegram_node.py:206`, `stt_node.py:605`, супервизор через `GRIP_TTS_REQUEST_TOPIC` (`supervisor_node.py:397`); рантайм добавляет `joystick_control_node` | 5–6 писателей, совпало |
| V12 | Писатели `/voice/tts/control`: `audio_node.py:152`, `stt_node.py:587`, `dialogue_node.py:848`, `quest_node.py:1752` | 4, совпало |
| V13 | `/voice/dj_mode` пишут `dialogue_node.py:1051,5274` и `tools/music.py:7055`; два владельца DJ-флага: `DJState` (`dj_mode.py:157`) и `MusicManager._dj_mode_enabled` (`music.py:714`) | совпало |
| V14 | `/voice/stt/result` пишут `stt_node.py:578` и `telegram_node.py:168`; telegram также пишет `/voice/dialogue/response`, `/avatar/command`, `/voice/tts/request`, `/voice/sound/stop` (`telegram_node.py:168-223`); dialogue_node разбирает префикс `[TG:chat_id]` (`:2977`) и кладёт `[TG]` в текст пользователя (`:6896`) | совпало |
| V15 | `check_silence_timeout` (`dialogue_state_machine.py:456`) в prod не вызывается; докстринг `:6` обещает «SILENCED → IDLE (after timeout)» | совпало — SILENCED бессрочен |
| V16 | `IGNORE_STOP_MS:700` (`dialogue_node.py:9639`, `tts_node.py:1843`) | совпало |
| V17 | Таймаут тула 10 с, `_LONG_TOOL_TIMEOUTS = {}` (`executors/ros_mcp.py:25,48`) | совпало |
| V18 | 10 флагов `_*_retry_used` в `dialogue_node.py`; 11 `build_*_retry_prompt` в `dialogue_guards.py`; `PHANTOM_ACTION_VERB_STEMS :2074`, `detect_phantom_action_claim :2216`, `detect_universal_action_claim :1830`, `is_metalanguage_babble :719` | совпало |
| V19 | `_SILENT_DONE_MARKERS` (`agent_core.py:440`, проверки `:1241,1265`) | совпало |
| V20 | `master_prompt_compact.txt` 60 490 байт, 45 упоминаний `RULE #`; промпт ссылается на гард (`:228`) | совпало |
| V21 | Каталог тулов: 66 записей, 60 видимых LLM (`python -c` через `rob_box_core.tool_catalog`) | совпало |
| V22 | `architecture/ownership.yml` — `nodes: {}`, `topics: {}`, `capabilities: {}`, `features: {}` | пуст, совпало |
| V23 | Пакетов сообщений в `src/`: `rob_box_perception_msgs`, `rob_box_supervisor_msgs`, `robot_sensor_hub_msg`; голосовых `*_msgs` нет; `rob_box_core/utterance.py` даёт dataclass `Utterance{text, sink, priority, voice, language, emotion, speech_id}` | совпало |
| V24 | ADR-0084 «Статус: accepted»; ADR-0021 «Статус: proposed» (18.08); `CONTEXT.md:109-116` «Ход» ссылается на `TurnGuards`; файлов `docs/adr/015*` кроме этого нет | совпало |
| V25 | Интерфейс LLM: `rob_box_llm/provider.py:226 class LLMProvider` с `capabilities -> ProviderCapabilities{text, streaming_text, tools, streaming_tools, image_input}` (`:78-94`); флагов `tool_choice/json_mode/structured_output` нет; адаптеры `providers/{deepseek,mimo,minimax}.py` | совпало — основа для §4.5 есть, расширяется |
| V26 | Экспрессия: `KNOWN_ANIMATIONS` — 18 имён (`mcp_tools/animations.py:36`), `speak_text(text, animation, voice)` (`tools/dialogue.py:409`); `/voice/animation/request` читают `animation_player_node.py:66` (матрица) и `led_node.py:180` (кольцо ReSpeaker), `led_node` ещё читает `/voice/dialogue/state` и `/audio/direction` (`:165,172`); `animation_player` сам переключается по `/voice/tts/state` (`:74`); `/voice/sound/trigger` пишут `stt_node.py:608` («boop»), `dialogue_node.py:834`, `tools/sound.py:30` (+ `health_monitor` в рантайме), читает `sound_node.py:59`; алиас `boop → button_click` `sound_node.py:108` | совпало — 4 писателя звука, 2 исполнителя анимаций без владельца решения |
| V27 | Telegram: `telegram_bot.yaml:llm_provider/llm_model/llm_max_history` присутствуют, но в `src/rob_box_telegram` ни один `.py` их не читает (grep пуст) — мёртвые с W7 (`07dfc28aa`) | совпало |
| V28 | **Голосовая задача завершена в момент публикации**: `SpeakTextTool.execute` публикует чанки в `/voice/tts/request` и возвращает `success=True, data.async=True` (`tools/dialogue.py:625-645`); ожидание `tts/finished` отсутствует — `EffectAwaiterRegistry.register_tts` определён (`speak_helpers.py:663`), вызывающих вне определения нет (grep `register_tts` даёт только `def` и несвязанный `_register_tts_loop_atexit` в tts_node) | совпало — ключевое утверждение 07 §0 п.2 |
| V29 | `barge_in_policy: "replace"` в боевом конфиге (`docker/vision/config/voice_assistant/dialogue_node.yaml:23`) — ветка `classify` (QUEUE/MERGE через `quick_decide`) выключена | совпало |
| V30 | Планировщик: `scheduler/task_scheduler.py` 1 312 строк, `tool_executor.py` 691, `quick_decide.py` 137, `delta.py` 109, `event_bus.py` 45; каналы — `_VOICE_TOOLS={speak_text}`, `_MUSIC_TOOLS={stop_music}`, `_ANIM_TOOLS={play_animation}` (`tool_executor.py:82-97`), музыкальные стартеры в bypass; подключение `dialogue_node.py:2204-2215` | совпало (2 395 строк) |
| V31 | `harness/decision/scheduler_shadow.py` — вне самого файла и тестов ссылок нет (grep `scheduler_shadow\|SchedulerShadow` = 0) | совпало — мёртвый PoC |
| V32 | Поле `pregenerate` payload чанка: в `src` вне `scheduler/pregen` и тестов встречается только в комментариях/параметрах `tts_node.py:870-878,1478-1488`; публикатора поля нет | совпало — подключено, данных не получает |
| V33 | `play_sound` отвечает `success=True, message="Звук запущен…"` сразу после `publish` (`tools/sound.py:145-151`); `sound_node.trigger_callback` молча `return` при активном голосовом стриме или если звук уже играет (`sound_node.py:221-228`) | совпало — класс честности К2 |
| V34 | Гарды исполнителя: `track_start_guard.py` несёт #2859 (`:1`), #2878 (`:79,263`), #3221 (`:30,238,276`), #3246 (`:106,289`), #3247 (`:116,331`); гейт #2913 — `tool_executor.py:128`; `tts_node` держит второй планировщик: `priority` вклинивание (`:1435`), `_pending_speech_queue` чужих батчей (`:1468`) | совпало |
| V35 | ADR-0011 — статус Accepted, транспорт HTTP-сайдкар `voice-action-server` вместо ROS2 Action (ADR-0002); пакет `action_server/{http,http_server,paste,server}.py` на месте; `PlayerOwner.track_started` — колбэк из потока клока «трек реально встал на долю `start_beat`» (`engine/player_owner.py:78-89`) — источник точки синхронизации для §5.6 | совпало |

- **Ничего не запускалось** ни локально, ни на роботе. Все величины задержек, долей и частот — из тел issue, памяти проекта и материалов 01–06; где я их цитирую, стоит «(из материалов)».

### 0.1 Расхождения материалов с кодом (верю коду)

| Утверждение в материалах | Что на HEAD | Следствие |
|---|---|---|
| 03 §0, 01 §0: меток `TEMP(ADR-0148)` в `src` — 0 | `grep "TEMP(ADR"` даёт **2**: `club_transition.py:233`, `dj_mode.py:85` (страховка #3136 на музыкальном пути); точная строка не матчилась из-за запятой `TEMP(ADR-0148, #3136…)` | на диалоговом пути меток 0 — вывод 03 остаётся; гард моратория (PR-0) ищет `TEMP(ADR-0148` без закрывающей скобки |
| ADR-0149 §5.3: «число `re.compile` в `dialogue_guards.py` (34) и `music_guard.py` должно падать» | в `music_guard.py` `re.compile` = 0 | метрика для `music_guard.py` — размер файла и число вердиктов (§14 A8) |
| 00-brief: «ADR-0013 — малые PR» | `0013-respeaker-dsp-tuning-mix-ch0.md` — про ReSpeaker; правило PR — `AF-0013-incremental-delivery-over-big-bang.md` | ссылаюсь на ADR-AF-0013 |
| 02 §2.2: `/voice/tts/request` — 5 писателей в коде | по `create_publisher` с литералом — 3; ещё 2 через константы; рантайм — 5 с `joystick_control_node` | статический аудит пропускает константы — гард владельцев (§10.3) считает по рантайм-снимку |
| 02 §4: «живого вызова `save_turn` нет» | вызовы `voice_memory.py:33,36` — докстринг; `VoiceMemoryAdapter.save_turn` — deprecated-заглушка по **директиве Шифу 02.09** («forbids persisting dialogue turns in production», `voice_memory_adapter.py:47-49`) | история не персистится намеренно; v2 — §7.4, решение В8 |
| 02 §2.5: «Telegram-канал: отдельных ADR нет», 01 §17: «[TG] отдельно не разбирал» | история git: Telegram **имел собственную LLM** с 28.02 (`3c91ba155`) до W7 28.07 (`07dfc28aa`), потом стал мостом в `/voice/stt/result`; эхо удалено `88cecc91f`, возвращено #1195 (`def24baaa`) с маркером `[TG:chat_id]` | §9.2 возвращает телеграм-агента на новом движке (решение В10) |
| 05 §1–§2: `Pipecat #5305`, `LiveKit FallbackAdapter` — «(поиск)» | не проверял | беру форму паттерна, не API |
| Ревизия 2 этого ADR: §3 «`_order_tool_calls` переносится», §5.2 «одна `SpeechRequest` на ход», §4.1 «при действиях `speech` не озвучивается», §6.2 «вейк во время речи → отмена всегда» | 07 §9 показал: эти четыре формулировки делают «рэп под бит» и «дочитай и вплети» невозможными (Х1–Х5), а `_order_tool_calls` тащит холостой `deferred_call_ids` (`agent_core.py:426-431`) | исправлено в ревизии 3: §4.1 (класс «выступление»), §5.2 (последовательность сегментов), §5.6 (таблица `COMPOSITION` вместо перестановки), §6.2 (перебивание по классу) |
| 07 §1.6 «`led_node` слушает `/voice/animation/request` — взято из ADR-0150, код не открывал» | я открывал: `led_node.py:180` (V26) | подтверждено |

---

## 1. Проблема: классы отказов и семь корней

Товарищ Шифу слышит: «робот сказал, что записал, а не записал», «растерялся — бит не запустился» при играющей музыке, «читает звёздочки и system», молчит минуту, после «хватит» глух до утра, называет незнакомца Борисом, говорит голосом вчерашнего диджея. Материалы сводят это к десяти классам (01 §1) и семи корням (03 §3). Коротко, с цифрами (из материалов, если не сказано «проверено»):

| Класс отказа (01 §1) | Цифры | Open на 01.10 |
|---|---|---|
| 1. LLM не вызвала тул / выдумала успех | ≥ 10 итераций заплаток с 02.2026 по 01.10; детекторы под каждую предметную область | #3266, #3271, #3297 |
| 2. Гарды и синтетические ретраи ломают диалог сами | 14 типов ретраев, 10 флагов `_retry_used` (проверено); до 8 вызовов LLM за ~70 с (#1881); 27 возможных вызовов на фразу (проверено V4–V5) | #3144 |
| 3. Служебный текст и разметка уходят в TTS | strip-фильтр расширяли ≥ 6 раз; регрессия ×193/час (#2558); 5 копий done-маркеров (проверено V19) | #3269, #3296 |
| 4. Тишина, задержки, пустой ответ | 33 с – 3 мин; «Принял.» вместо честного отказа (проверено V8) | #3265 |
| 5. Молчаливая деградация облаков | дефолтный LLM-провайдер менялся ≥ 7 раз; `tool_choice` трижды туда-обратно; MiniMax молча игнорирует `tool_choice` (ADR-0143) | — |
| 6. Wake-word и допуск | 26 вариантов вейка (проверено V10); три дрейфующие копии списка | — |
| 7. Барж-ин, отмена, гонки | STOP шлют 4 ноды (проверено V12); отмена не доходит до HTTP-стрима (#1280); `/voice/current_dialogue_id` никто не пишет (V9); `IGNORE_STOP_MS:700` (V16) | — |
| 8. Утечки состояния | SILENCED без TTL (V15); марафон 29→30.09: 1/12 актов зелёный; историю чистили ≥ 25 коммитов | #3145, #3000 |
| 9. Идентичность | ~25 issue за три дня 22–24.09; `utterance_id` чинили 5 раз за 2 дня | #2754 |
| 10. Дубли реализаций | 5 детекторов «стоп», 4 — «громче», ~18 списков музыкальных тулов, 17 топиков с несколькими писателями (06 §2); `ownership.yml` пуст (V22) | #3108, #3109 |

Темп: `dialogue_node.py` — 353 fix-коммита из 526; август 274 и сентябрь 272 fix-коммита по диалоговым каталогам (01 §15). `DialogueNode` WMC 1 103 при лимите 80, 210 методов, TCC 0.04 (02 §1.1, 06 §4).

### 1.1 Семь корней (03 §3, принимаю как есть)

- **К1.** LLM одним свободным ответом выбирает намерение, параметры и фразу об успехе.
- **К2.** Fire-and-forget тулы (`speak_text`, `set_voice`, `set_tts_provider`, `set_dj_mode`, `play_sound`, `play_animation`, `register_speaker`) возвращают успех в момент публикации в топик (02 §6.1).
- **К3.** Состояние без единственного владельца и событийного контракта: два владельца DJ-флага и голоса TTS, один SILENCED на двух писателей без «кто поставил», 17 топиков с несколькими писателями, JSON в `String` без схем.
- **К4.** Два канала речи (свободный текст и `speak_text`) плюс протокол done-маркера.
- **К5.** Контекст — смесь текста и разметки (`[Spkr:…]`, `[TG]`, `[URGENT_BACKLOG]`, `[CRITICAL]`), ~70k токенов на ход.
- **К6.** Решение «принять ответ» — после генерации, эвристиками по тексту, с синтетическими ретраями (`_handle_result`, CC 82).
- **К7.** Божественный узел и размазанное знание без механизма удаления.
- **К8 (07 §0, §2).** Исполнение во времени без владельца: решение «все тулы через шедулер» (#968, 01.08) обросло заплатками и потерялось — планировщик (`scheduler/`, 2 395 строк, V30) в живом пути только раскладывает вызовы по трём FIFO и отвечает `{"status":"queued"}`; голосовая задача «завершена» в момент **публикации**, а не звучания (V28), поэтому MERGE/ETA/QUEUE недостижимы; реальная очередь речи живёт в `tts_node` как второй планировщик (V34); музыкальные стартеры выведены в bypass (19.08, `1579d56bf`); «дочитай, потом сделай» в бою невозможно — вейк во время речи = STOP + REPLACE (V29). Хотелки Х1–Х15 (07 §3): работает полностью 1, частично 4, сломано/не сделано 10.

ADR-0148 §4 постановил: музыка — первая волна (ADR-0149), `DialogueNode` — следующая, на тех же принципах. Этот ADR — её дизайн.

---

## 2. Решение: границы, модули, владельцы

### 2.1 Принципы (ADR-0148, применённые к диалогу)

1. **Понимание — команда, не ответ.** LLM на каждом ходе выдаёт **одну команду** со структурными полями: речь (опционально), вопрос, до трёх действий, экспрессия. Код её валидирует и исполняет. Паттерн — Rasa CALM (05 §6), не ReAct-агент.
2. **Одна точка входа речи** (решение В1). Всё, что звучит, идёт одним путём: `Command.speech` → `SpeechRequest` → владелец речи. Транспорт слота речи внутри ответа LLM (поле команды против тула `speak_text`) **не выбирается заранее** — его выбирает бейк-офф по критерию 100 % доставки (§4.4); проигравший удаляется. Служебное в речь попасть не может, потому что служебное в контексте идёт структурой (§7.3).
3. **Фразу об исполнении строит код из события владельца ресурса.** Действие → исполнитель → событие (`started/finished/rejected/timeout`) → шаблон из `rob_box_dialog/phrases/ru.yaml` (HA `responses`, OVOS `.dialog`, 05 §3–§4). Речь LLM в команде с действиями не озвучивается (§4.1), LLM-комментарий — за флагом, выключен (решение В6).
4. **Сессия — один объект с одним владельцем.** `Session` живёт в `dialog_node`, пишет только код, LLM видит read-only снимок (Letta `read_only`, OVOS Session, 05 §4, §7). Сцены — стек кадров с явным `pop` (Rasa dialogue stack). Рестарт контейнера = новая сессия (решение В11).
5. **Тулы видны по возможности** (SayCan «can», 05 §8): `available_tools(session, capabilities)` — функция, не правило в промпте.
6. **Провайдер LLM — деталь конфигурации** (решение В4). Один интерфейс адаптера, возможности объявляются, режим команд выбирает код, новый провайдер — один адаптер + одна строка конфига, контрактный тест на эталонном наборе (§4.5).
7. **Деградация громкая и типизированная** (Wyoming `describe/info`, коды HA, 05 §3, §5).
8. **В историю пишется только озвученное** (OpenAI `truncate`, Pipecat `TTSTextFrame`, 05 §1, §9) плюс структурные результаты.
9. **Живость — у кода, не у LLM** (решение В5): реакции на стадии хода (слушаю/думаю/выполняю/готово/не вышло) делает код без вызова LLM; экспрессия LLM едет в той же команде enum-ами (§5.5).
10. **Знание — один модуль** `rob_box_dialog.knowledge`; копии удаляются с grep-доказательством.
11. **Никаких регексов по тексту ответа LLM и синтетических ретраев** в v2. Невалидная команда → `Ask`-шаблон из кода, один раз, без второго вызова.
12. **Адресация — только по вейк-слову** (решение В2). Окна адресации без вейка нет и флага под него нет; надёжность вейка (одна таблица, границы слов, искажения по логам) — в плане. Доказательство — §17 Ж.
13. **Исполнение во времени — у планировщика ресурсов** (§5.6). Каждое действие занимает ресурс (голос, лицо, кольцо, SFX, музыка, движение); статус задачи — **событие владельца ресурса** (`SpeechEvent finished`, а не «опубликовал»); одновременность и порядок — таблица `COMPOSITION` (данные, одна на проект) с политиками APPEND/MERGE/REPLACE/REJECT по словарю BML; выступление (песня, рэп, сказка) — отдельный класс действия со связанным временем жизни подложки, речи и экспрессии и стартом на такте от владельца плеера v2 (ADR-0149). Политику выбирает код, LLM — максимум `when` из enum.

### 2.2 Пакеты и модули

```
src/rob_box_dialog/                      # НОВЫЙ пакет. Чистый Python: без rclpy, без сети, без LLM-клиентов.
  rob_box_dialog/
    knowledge.py        # одна таблица: WAKE_WORDS (из wake_words.yaml), SILENCE/UNSILENCE, VOLUME_WORDS, VOLUME_CHANNELS,
                        # ACTION_CLASSES{query, action.*} с таймаутами/прерываемостью, EMOTIONS, GESTURES, EARCONS (enum),
                        # REFLEXES (стадия хода → экспрессия), SCENE_KINDS, PHRASE_KEYS. Генерирует фрагменты промпта и схем.
    session.py          # Session (frozen snapshot + owner-методы), SceneFrame, стек сцен, TTL; событие SessionChanged
    address.py          # AddressPolicy: адресовано ли роботу — вейк по границам слов (одна таблица); для source=telegram/operator — всегда
    grammar.py          # Tier-1: закрытая грамматика команд (расширение media_command_grammar): стоп, тишина/выход,
                        # громкость по каналу, голос, новая сессия, диджей, да/нет на вопрос о личности
    command.py          # Command{speech?, ask?, actions≤3, expression} (frozen) + parse/validate из tool_calls или JSON
    speech_channel.py   # транспорт слота речи: FieldChannel | ToolChannel — один включён по итогам бейк-оффа (§4.4)
    tools_view.py       # available_tools(session, capabilities) -> узкий срез каталога (имена + схемы из rob_box_core)
    context.py          # build_context(session, turn_log, facts) -> структурные блоки для LLM, окно озвученной истории
    execute.py          # Executor: actions -> ActionRequest; ожидание ActionEvent владельца; ActionResult{status, …}
    plan/               # ПЛАНИРОВЩИК РЕСУРСОВ (§5.6) — замена scheduler/ (2 395 строк) и второго планировщика в tts_node
      resources.py      #   ResourceState по снимкам владельцев: что занято, чем, с какой эпохой, граница (sentence|bar|now)
      composition.py    #   compose(new_cls, busy_cls, tier1_hint, when) -> APPEND|MERGE|REPLACE|REJECT по knowledge.COMPOSITION
      perform.py        #   Performance: сегменты (куплеты) PENDING/ACTIVE/DONE, подложка, экспрессия; MERGE только в PENDING;
                        #   Parallel(success_count=1): речь кончилась → outro подложки → стоп; required: подложка не стартовала → отказ
      sync.py           #   точки синхронизации: next_bar(started{bpm, beat_at}, output_latency) -> t; play(segment, at=t)
      plan_events.py    #   /dialog/plan_event (BML blockProgress): block start/end, segment started/finished, prediction
    expression.py       # Reflexes: стадия хода → ExpressionRequest без LLM; слияние с Command.expression
    phrases/ru.yaml     # каталог фраз: {action_class}.{status}[.{reason}] с вариантами и слотами
    respond.py          # phrase_from_result(result, session) -> Utterance; speech_for(Command)
    turn_log.py         # TurnLog: события хода с turn_id/epoch; проекция в окно LLM; хранение в памяти (§7.4)
    interrupt.py        # InterruptPolicy: по классам/эпохе — отменить речь, стрим, не трогать действие
    degrade.py          # ProviderState -> режим команд (tool_calls | json_command) и порядок провайдеров; фраза деградации
    agents.py           # AgentSpec(personality | operator | telegram): грамматика, срез, промпт, sink по умолчанию

src/rob_box_llm/rob_box_llm/provider.py  # ОСТАЁТСЯ как единый интерфейс адаптера: ProviderCapabilities расширяется
                                         # полями tool_choice, json_mode, structured_output, thinking (§4.5)
src/rob_box_dialog_msgs/                 # НОВЫЙ пакет сообщений (§10)

src/rob_box_voice/rob_box_voice/
    dialog_node.py      # НОВЫЙ тонкий хост v2 (≤ 600 строк, WMC ≤ 80): ROS-подписки/публикации, таймеры, вызовы rob_box_dialog
    core/stt_admission.py                              # ОСТАЁТСЯ: 12 шагов; WakeWordStep → address.py; MediaCommandStep → grammar.py
    core/media_command_grammar.py, media_router.py     # ОСТАЮТСЯ как Tier-1 для музыки; грамматика расширяется, не копируется
    tts_node.py         # ВЛАДЕЛЕЦ РЕЧИ: один вход SpeechRequest (с полем at — «играть в момент t» и boundary — граница REPLACE),
                        # события SpeechEvent, отмена по speech_id/epoch; пред-синтез без воспроизведения (commit=false);
                        # собственная очередь/приоритеты/pending-батчи (второй планировщик) удаляются — порядок задаёт plan/
    stt_node.py         # STT + ProviderState(stt); STOP не шлёт — шлёт InterruptRequest; «boop» не шлёт — рефлекс у диалога
    command_node.py     # движение по /dialog/intent, а не по /voice/stt/result напрямую
    sound_node.py       # исполнитель earcon'ов по ExpressionRequest; /voice/sound/trigger остаётся только для mcp-тула play_sound
src/rob_box_animations/scripts/animation_player_node.py   # исполнитель анимаций матрицы по ExpressionRequest
src/rob_box_voice/rob_box_voice/led_node.py               # исполнитель кольца по ExpressionRequest и стадии хода
src/rob_box_harness/rob_box_harness/core/agent_core.py    # LLM-клиент с циклом ≤ 2 query-итераций; история/скиллы уходят в rob_box_dialog
src/rob_box_supervisor/                  # ТАРС — AgentSpec(operator) на rob_box_dialog (§9.3)
src/rob_box_telegram/                    # телеграм-агент — AgentSpec(telegram) на rob_box_dialog, своя LLM-сессия на чат (§9.2)
```

Бюджет ADR-0145 (WMC ≤ 80, методов ≤ 40) — без исключений для новых классов. Каждый модуль `rob_box_dialog` — ≤ 5 публичных функций.

### 2.3 Владельцы состояния (заполняет `architecture/ownership.yml`, §10.3)

| Состояние | Владелец (единственный писатель) | Как публикуется | Кто читает |
|---|---|---|---|
| Сессия голосового диалога: эпоха, стек сцен, тишина (TTL, `set_by`), персона, голос `requested/applied`, собеседник | `dialog_node` (`rob_box_dialog.session`) | latched `/dialog/session` (`SessionState`) | tts_node, command_node, led_node, arbiter, quest, telegram-агент, супервизор |
| Сессия телеграм-агента (на `chat_id`) и ТАРС | соответствующий агент; **не** `dialog_node` | свой `/telegram/session`, `/avatar/session` (`SessionState`, поле `agent`) | dialog_node читает для правил совместного доступа к речи (§9.2.3) |
| Ход: `turn_id`, эпоха, стадия (`addressed → understood → executing → speaking → done/failed/cancelled`) | владелец соответствующей сессии | `/dialog/turn_event` (`TurnEvent`, поле `agent`) | экспрессия (рефлексы), ТАРС-журнал, e2e, метрики |
| Речь робота: что звучит, сколько символов озвучено, пред-синтезированные сегменты | **tts_node** (исполнитель ресурса «голос») | `/voice/speech/event` (`SpeechEvent`) | все агенты (история по `spoken_chars`), audio_node, stt_node, animation_player, `plan/` |
| **Расписание ресурсов**: кто занял голос/лицо/кольцо/SFX/музыку/движение, чем, политика при конфликте, границы; **выступление** (сегменты PENDING/ACTIVE, подложка, время жизни) | `rob_box_dialog.plan` в процессе агента-владельца хода (рекомендация, вопрос О6 — альтернатива: часть tts_node) | `/dialog/plan_event` (`PlanEvent`, BML `blockProgress`) | LLM-контекст (блок «что звучит / что в очереди / сколько осталось»), e2e, метрики A19–A23 |
| Экспрессия: какая эмоция/жест/earcon сейчас | **решение** — агент-владелец хода (`expression.py`); **исполнение** — animation_player (матрица), led_node (кольцо), sound_node (earcon) | `/voice/expression/request` (`ExpressionRequest`, писатели — агенты через один клиент) → события `/voice/expression/event` | — |
| Вход речи: сегмент, текст, провайдер STT, `utterance_id` | stt_node | `/voice/stt/utterance` (типизируется в PR-2) | dialog_node — **единственный** читатель |
| Собеседник (биометрия) | speaker_id_node | `/voice/speaker/result` (ADR-0131) | dialog_node |
| Музыка | `PlayerOwner` в mcp_server (ADR-0149 §2.3) | latched `/voice/music/state`, `/voice/music/event` | dialog_node (кадр `DJ`, фраза по `started/rejected`), телеграм-агент |
| Провайдеры: здоровье и возможности | каждая нода за свой вид: tts_node (TTS), stt_node (STT), каждый агент (свой LLM) | latched `/voice/providers/state` (`ProviderState[]`) | `degrade.py` каждого агента, панель ТАРС |
| Floor / режим аватара | арбитр (образец) | `/avatar/state` | без изменений |
| Громкость | три канала у трёх владельцев; **решение «какой канал»** — `grammar.py` по снимку | действия `set_volume(channel=…)` | — |

Удаляются как топики без владельца: `/voice/current_dialogue_id`, `/voice/dialogue/response`, `/voice/tts/request`, `/voice/tts/control`, `/dialogue/control` + `/dialogue/control_ack`, `/voice/dj_mode` как писатель диалога, `/harness/task_events` (заменён `/dialog/plan_event`), `/voice/animation/request` (заменён `ExpressionRequest`), «boop» в `/voice/sound/trigger`, `/voice/tts/batch_registered` и `/voice/tts/batch_complete` (их публикует mcp_server, а не владелец речи, V28 — заменены `SpeechEvent finished` по сегменту и `PlanEvent block end`), `/mcp/music_cleanup`/`/mcp/music_fallback` (время жизни подложки — у выступления, §5.6.5).

---

## 3. Поток одной реплики (как будет)

Пример: «Робби, запомни, что Саша любит зелёный чай» (ровно #2755: «Записала…» при `tools=[]`).

```mermaid
flowchart TD
  MIC[ReSpeaker] --> AN[audio_node: VAD, сегмент]
  AN -- "/audio/speech_audio" --> STT[stt_node: minimax→yandex→vosk<br/>+ ProviderState(stt)]
  AN -- "/audio/speech_audio" --> SID[speaker_id_node]
  STT -- "/voice/stt/utterance (utterance_id, text)" --> ADM
  SID -- "/voice/speaker/result (utterance_id)" --> ADM
  STT -- "InterruptRequest, если речь робота звучит" --> INT

  subgraph DN["dialog_node (тонкий хост) + rob_box_dialog"]
    ADM["SttAdmission (12 шагов)<br/>WakeWordStep → address.py: ТОЛЬКО вейк"] -->|"не адресовано"| BL[бэклог/drop]
    ADM -->|"адресовано"| RX1["expression: рефлекс addressed<br/>earcon «услышал» + кольцо, без LLM"]
    RX1 --> T1{"Tier-1 grammar.py<br/>стоп · тишина · громкость · голос ·<br/>новая сессия · диджей · да/нет"}
    T1 -- "распознано" --> EXE
    T1 -- "нет" --> RX2["рефлекс thinking: анимация «думаю»"]
    RX2 --> CTX["context.py: снимок Session,<br/>окно озвученного, факты, available_tools ≈ 6–12"]
    CTX --> LLM{{"LLM через единый адаптер:<br/>ОДНА команда {speech?, ask?, actions≤3, expression}<br/>tool_calls или JSON — по capabilities"}}
    LLM --> VAL["command.parse+validate<br/>невалидно → Ask-шаблон, 0 ретраев"]
    VAL -- "speech/ask без actions" --> SP
    VAL -- "actions=[remember(person, fact)], expression=happy" --> EXE["execute.py: ActionRequest →<br/>владелец ресурса; рефлекс executing"]
    EXE -- "ActionEvent done/rejected/timeout" --> RES["respond.py: phrase_from_result<br/>«Запомнил: Саша любит зелёный чай»<br/>/ «Не смог записать — память недоступна»<br/>+ рефлекс done/failed"]
    EXE -- "query-тул (search_web, get_time)" --> LLM
    RES --> SP["SpeechRequest(speech_id, epoch, text, interruptible)<br/>+ ExpressionRequest(emotion из команды)"]
    INT["interrupt.py: по эпохе —<br/>отменить речь и стрим, действие по классу"] --> SP
    SP --> LOG["turn_log: spoken только по SpeechEvent.spoken_chars"]
  end

  SP -- "/voice/speech/request" --> TTS["tts_node — владелец речи"]
  SP -- "/voice/expression/request" --> EXP["animation_player · led_node · sound_node"]
  TTS -- "/voice/speech/event" --> DN
  TTS --> SPK[динамики]
  EXE -- "/mcp/execute (ActionRequest)" --> MCP[mcp_server: тулы, PlayerOwner, память]
  MCP -- "/mcp/result + /voice/music/event" --> EXE
```

Точки решения — в целевой форме (против таблицы 02 §3.1):

| Решение | v1 (02 §3.1) | v2 |
|---|---|---|
| 1–6 речь/эхо/провайдер STT/ТАРС-или-личность | код в audio/stt | **без изменений**, плюс `ProviderState(stt)` |
| 7 адресовано роботу | вейк на каждой фразе, список из 26 искажений, три копии | **вейк на каждой фразе** (В2); одна таблица, границы слов, искажения пополняются по логам e2e (ADR-0114), не придумываются; `source=telegram/operator` — адресовано всегда (как #1195) |
| 8–9 тишина | подстрока «хватит», без TTL | `grammar.py` по границам слов; кадр `Silence(ttl=600 с, set_by)` (В3) |
| 10 движение/стоп | command_node мимо допуска | только после допуска: `/dialog/intent` |
| 12 медиа-команды | `MediaRouter`, при лишнем слове `to_llm` | остаётся Tier-1; при промахе — узкий `request_music/dj_set` (ADR-0149 §5.1) |
| 14 кто говорит, спросить ли имя | код (после #2888) | код; вопрос — `Ask`-шаблон, ответ — Tier-1 |
| 15 скилл | 27 регексов `skill_router` + LLM `load_skill` | сцена из грамматики/команды; `tools_view` даёт срез; `skill_router.py` удаляется |
| **16 что сказать и какие тулы** | **LLM, свободно** | **LLM — одна команда**: речь + ≤ 3 действий + экспрессия; параметры — по узкой схеме с enum |
| 17–18 порядок/очередь тулов | код: `_order_tool_calls` (музыка вперёд, `stop_*` в конец) + три FIFO с `queued` | код: таблица `knowledge.COMPOSITION` и `plan/` (§5.6); `_order_tool_calls` и его холостой `deferred_call_ids` **не переносятся** |
| **19 принять/переспросить/замьютить** | **эвристики по тексту, 14 ретраев** | **валидатор до исполнения; 0 ретраев** |
| 20 сказать целиком/первое предложение | `decide_turn_speech` | длина `speech` ограничена схемой (280, В5), не промптом |
| 21 провайдер TTS | tts_node | tts_node + `ProviderState(tts)` + фраза деградации |
| 22 DJ-переход | таймер + LLM | **музыка v2** (ADR-0149), диалог только слушает события |
| новое: экспрессия | LLM через `speak_text(animation=…)`, `play_animation`, `play_sound`; `animation_player` сам по `/voice/tts/state` | рефлексы кода по стадиям хода + enum в команде (§5.5) |
| новое: новая команда во время исполнения | stt_node STOP на любой вейк → REPLACE всегда (V29) | `compose()` по классу занятого и нового действия + Tier-1; «стоп/хватит» — REPLACE, дополнение к выступлению — APPEND/MERGE на границе (§5.6.4) |
| новое: когда завершено действие | публикация в топик (V28) | событие владельца ресурса (§5.6.2) |

---

## 4. Что решает код, что — LLM

### 4.1 Узкий слот LLM: команда

```python
# rob_box_dialog/command.py (frozen dataclasses; схема генерируется из них для tool_calls и для JSON-режима)
class Expression:  emotion: Emotion = "neutral"; gesture: Gesture | None; earcon: Earcon | None   # enum из knowledge
class ToolCall:    tool: str; args: Mapping[str, Any]                                            # тул из available_tools(session)

class Performance:  # выступление (§5.6.3): речь ЗДЕСЬ — содержимое, а не отчёт об исполнении
    kind: Literal["rap", "song", "tale", "poem"]
    segments: tuple[str, ...]     # куплеты/абзацы, 2..12, каждый ≤ 400 символов; сегменты режет код, LLM отдаёт список
    backing: Literal["beat", "calm", "none"] = "none"   # подложка → request_music(intent=backing, …), ADR-0149 §5.1
    expression_per_segment: tuple[Expression, ...] | None

@dataclass(frozen=True)
class Command:
    speech: str | None            # ≤ 280 символов; реплика человеку. Транспорт — speech_channel (§4.4)
    ask: Ask | None               # вопрос с ожиданием: expects ∈ {"yes_no", "name", "free"}; открывает ожидание ответа С вейком
    actions: tuple[ToolCall, ...] # ≤ 3; порядок и одновременность назначает код (§5.6); ≤ 1 действие класса action.media|motion
    perform: Performance | None   # класс action.perform — длинная речь с подложкой; взаимоисключает speech
    when: Literal["after_current", "now"] = "after_current"   # единственное, что LLM говорит о времени; валидатор режет now для непрерываемых
    edit_pending: bool = False    # только для perform: править ещё не начатые сегменты текущего выступления (MERGE), не начинать новое
    expression: Expression        # всегда есть; дефолт neutral
    passed: Literal["not_addressed", "nothing_to_say", "unclear"] | None   # «молчать» — честный исход
```

- **Правило речи при действиях (честность):** если `actions` непустой, `speech` **не озвучивается** — в этом ходе звучат только шаблоны по событиям (§5.4); `speech` уходит в лог и в текстовый канал (Telegram), где он не может выдать себя за отчёт об исполнении. Так LLM структурно не может сказать «Записала» до события. LLM-комментарий после действия — флаг `post_action_comment: false` (В6).
- **Исключение — класс «выступление»** (исправление по 07 §9 п.1): в `perform` речь **и есть действие** — куплеты рэпа, строфы сказки. Они озвучиваются сегментами по расписанию планировщика (§5.6.3), лимит 280 на них не действует (лимит на сегмент — 400, на число сегментов — 12, оба в схеме). Правило честности сохраняется в другой форме: `perform.segments` не могут содержать отчёт об исполнении, потому что фраза «Включаю бит» / «Бит не завёлся — читаю без него» всё равно строится кодом по `started/rejected` подложки, а `perform.finished` — событие планировщика, не слова LLM. В6 (шаблон после действия) остаётся для обычных `actions`.
- **Время — только `when`:** LLM выбирает `after_current` (дочитать текущее, потом исполнить — **дефолт**) или `now`; остальное (APPEND/MERGE/REPLACE/REJECT, граница, такт) решает `compose()` по таблице классов и Tier-1 (§5.6.4). `now` для непрерываемых классов занятого ресурса отклоняется валидатором с понятной ошибкой.
- **Форма:** `degrade.py` по `ProviderCapabilities` выбирает: (а) `tool_calls` + `tool_choice="required"` с одним тулом `command` (или с набором `command`+узкие тулы); (б) `structured_output`/JSON-mode по схеме; (в) свободный текст с одним JSON-объектом — парсер `command.parse(text)` берёт первый валидный объект, остальное отбрасывается и не озвучивается. Режим — не правило в промпте, а ветка кода по возможностям.
- **Валидатор** (`command.validate`): тулы ∈ `available_tools`; аргументы по JSON-схеме каталога (`rob_box_core.tool_catalog`), enum/границы в схеме — poka-yoke (05 §10); `expression.*` ∈ enum `knowledge`; `speech` без управляющих последовательностей (структурно). Невалидно → `Ask("Не понял, повтори")` из шаблона. **Одна** попытка LLM на ход.
- **Query-цикл:** для тулов класса `query` результат возвращается LLM, она снова выдаёт одну команду (обычно `speech`). Максимум 2 итерации (Letta `MaxCountPerStepToolRule`, 05 §7). Итого ≤ 3 вызова LLM на фразу.
- **Терминальность:** команда с действиями класса `action.*` терминальна (Letta `TerminalToolRule`): после них LLM слова не получает.

### 4.2 Что остаётся коду

| Решение | Владелец v2 | Форма |
|---|---|---|
| Адресовано ли роботу | `address.py` | вейк по границам слов (одна таблица); `source ∈ {telegram, operator}` — всегда |
| Стоп, тишина/выход, громкость, голос, новая сессия, диджей вкл/выкл, да/нет | `grammar.py` | закрытая грамматика; **работает в любом состоянии**, включая SILENCED (урок HA 883012, 05 §3) |
| Какой канал громкости | `grammar.py` по снимку | играет музыка → музыка; звучит речь → TTS; иначе последний активный |
| Какие тулы видны | `tools_view.py` | функция от `Session` × `ProviderCapabilities` × `music_engine`; ≈ 6–12 схем |
| Принять команду | `command.validate` | до исполнения |
| Таймаут действия, прерываемость | `knowledge.ACTION_CLASSES` | навигация 120 с/непрерываемо; музыка ≤ 6 с; память 3 с; поиск 15 с/прерываемо |
| Фраза об исполнении | `respond.py` | шаблон по `(class, status, reason)` |
| Живость по стадиям хода | `expression.py` | `knowledge.REFLEXES` (§5.5), 0 вызовов LLM |
| Что в истории | `turn_log.py` | только озвученное + структурные результаты |
| Кто говорит / чей ход | владелец сессии (эпоха) | эпоха в каждом запросе; устаревшая → отбрасывается владельцем ресурса |
| Спросить ли имя | код (ADR-0131/0139) | `Ask(expects="name")` из шаблона |
| Деградация и режим команд | `degrade.py` | по `ProviderState`, фраза один раз на смену |

### 4.3 Что остаётся LLM

Смысл и речь: понять фразу в контексте, выбрать действия и значения из перечислений (`mood`, `genre`, `voice`), сформулировать `speech`/`ask`, выбрать `expression` (эмоция, жест, earcon), ответить по результату `query`. Персона и стиль — в системном промпте, который **генерируется** из `knowledge` + `Session.snapshot` (имена тулов, единицы, списки голосов, enum эмоций — не вручную; урок HA #182006/#182568, 05 §3). Целевой объём: системный блок ≤ 4k токенов + снимок ≤ 1k + схемы ≤ 12 тулов ≤ 4k + окно ≤ 3k → **≤ 12k токенов** против ~70k (02 §5).

### 4.4 Канал речи — бейк-офф (решение В1)

**Почему не решено заранее.** История (§18.1): `speak_text` введён 22.02 (`569f50194`), чтобы завершать агентный цикл SDK и нести голос/анимацию на фразу; свободный текст тогда давал молчание (`eaeb8e3a6` 28.02: «speak_text was never called, robot was silent» → auto-speak фолбэк) или двойную озвучку (#988); 01.03 `speak_text` объявлен MANDATORY (`3104a75e7`); 10.09 для DJ свободный текст заглушен (`2ad1bdd29`); 01.10 #3269 (`c2c31c26d`): MiniMax восемь итераций отдавала реплику в `content` рядом с `set_voice`, `speak_text` не звал никто — реплики сказки терялись. То есть **оба канала в разное время теряли речь**, и выбор зависит от провайдера. Шифу: «нужно разобраться в начале, что даст 100 % результат — это критерий выбора».

**Варианты** (одинаковый `Command`, разный транспорт слота `speech`):
- **A. Поле `speech` в команде** (`FieldChannel`): речь — строка в структурном ответе (tool_call `command` или JSON). Плюс: один объект, нет второго канала; минус: провайдер может положить речь в `content` вместо поля.
- **B. Тул `speak_text` как единственный канал** (`ToolChannel`): `speech` = аргумент тула, `content` **игнорируется всегда** (не фолбэк, а правило кода); действия — отдельные вызовы той же пачки. Плюс: доказанный путь, голос/эмоция на фразу; минус: провайдер без надёжного `tool_calls` (ADR-0143) может не позвать тул → потеря речи.

**Прогон** `scripts/dialog/speech_channel_bakeoff.py` (PR-1c): эталонный набор ≥ 60 фраз (классы 1–3 по 01 §2–§4 + сказка/рэп из #3269/#3221 + смена голоса из mv03) × {A, B} × каждый адаптер из конфига (DeepSeek, MiniMax, MiMo) × 3 прогона × тулы-заглушки с событиями. Метрики на фразу:
- `delivered` — для каждого хода, где ожидается речь, порождён ровно один `SpeechRequest` с непустым текстом;
- `leak` — служебный текст/разметка/`done`/псевдовызов в тексте речи;
- `double` — два `SpeechRequest` на одну реплику;
- `phantom` — текст речи содержит отчёт об исполнении до события (на эталонном наборе ответ известен — размечен вручную);
- `llm_calls`, `latency_p50/p95`.

**Критерий:** побеждает транспорт с `delivered = 100 %` и `leak = double = phantom = 0` на **всех** поддерживаемых адаптерах; при равенстве — меньше `llm_calls`, затем латентность. Если ни один не даёт 100 % — побеждает тот, у кого недостача закрывается **структурно** (например, B + правило «content игнорируется» или A + `structured_output`), и причина записывается в ADR-amendment. Проигравший транспорт удаляется в PR-7 (`speech_channel.py` остаётся с одним классом, `grep -c "class .*Channel" = 1`). До бейк-оффа PR-1…PR-2 канала не касаются.

### 4.5 Провайдер-агностик (решение В4)

- **Один интерфейс** — существующий `rob_box_llm.provider.LLMProvider` (V25). `ProviderCapabilities` расширяется: `tool_choice: bool`, `json_mode: bool`, `structured_output: bool`, `thinking: bool`, `max_context_tokens: int`. Адаптер объявляет честно (MiniMax: `tool_choice=False` по ADR-0143).
- **Режим команд выбирает код** (`degrade.py`): `structured_output` → схема; иначе `tool_choice` → один обязательный тул; иначе `json_mode`; иначе JSON в тексте. Промпт одинаков по смыслу, различается только форма ответа — генерируется из `knowledge`.
- **Выбор в конфиге:** `llm_providers: [deepseek, minimax]` (порядок = приоритет, здоровье — `HealthAwareFallbackLLM`). Новый провайдер = один файл `rob_box_llm/providers/<name>.py` + строка в каталоге `harness/providers/catalog.py` + строка в yaml. Кода в `rob_box_dialog` не меняется (тест: `rob_box_dialog` не импортирует ни одного адаптера напрямую — `grep -rn "providers\." src/rob_box_dialog` = 0).
- **Контрактный тест** `test_llm_adapter_contract.py`: офлайн — `FakeProvider` в каждом из 4 режимов обязан пройти эталонный набор на 100 % (это тест парсера/валидатора); с ключами — `scripts/dialog/adapter_benchmark.py` гоняет набор против каждого живого адаптера, пишет `delivered/invalid/latency` по адаптеру (A14 измеряется **по каждому провайдеру**).
- **Дефолт не прибивается в ADR**: выбирается по A14 в PR-8; стартовый кандидат по словам Шифу — **DeepSeek** (есть `tool_choice`, проба баланса, `health.py:647-687`).

---

## 5. Исполнение, событие, фраза

### 5.1 Классы действий (`knowledge.ACTION_CLASSES`)

| Класс | Примеры | Результат | Фраза | Прерываемо |
|---|---|---|---|---|
| `query` | `search_web`, `get_current_time`, `memory_search`, `get_music_state` | `data` → обратно LLM (≤ 2 итераций) | LLM `speech` | да |
| `action.local` | `remember(person, fact)`, `set_volume(channel)`, `set_voice`, `forget_session` | `ActionEvent` от владельца ≤ 3 с | шаблон | нет (мгновенно) |
| `action.media` | `request_music`, `dj_set` (ADR-0149 §5.1), `play_sound`(файл) | `/voice/music/event started|rejected` (≤ 6 с) / `sound started` | шаблон по событию | `dj_set stop` — да; старт — нет |
| `action.motion` | `navigate_to_waypoint`, `stop_motion` | Nav2 result (до 120 с) | «Еду к…» (обещание, ключ `motion.started`) → «Приехал»/«Не доехал: …» | **нет** (LiveKit `disallow_interruptions`); «стой» — Tier-1 |
| `action.identity` | `register_speaker`, `merge_speaker` | событие speaker_id_node (`registered`) | шаблон | нет |
| `action.speak_aloud` (только агенты operator/telegram) | `say_aloud(text, voice?)` | `SpeechEvent finished{spoken_chars, seconds}` | в канал агента: «Озвучил (12 с)» / «Не озвучил: тишина/занято» | да |
| `action.perform` (§5.6.3) | `Performance{kind, segments, backing}` | `PlanEvent block_end{segments_done, spoken_chars}`; подложка `started/rejected` | `perform.backing_started` («Поехали!»), `perform.no_backing`, `perform.finished` — только шаблон; куплеты — речь LLM | на границе сегмента (REPLACE по Tier-1); MERGE в PENDING |

### 5.2 Контракт исполнителя

```python
@dataclass(frozen=True)
class ActionResult:
    turn_id: str; epoch: int; agent: str; tool: str; cls: ActionClass
    status: Literal["done", "rejected", "timeout", "cancelled", "pending"]
    reason: str | None          # ключ из phrases ("quota", "not_found", "no_motors", "busy", "silenced")
    data: Mapping[str, Any]     # слоты для шаблона
    event_ref: str | None       # track_id, speech_id, nav goal_id
```

- Для fire-and-forget тулов v1 (К2) владелец **обязан** ответить событием; нет события к дедлайну класса → `timeout` → шаблон «Не дождался …». Тул класса `action.*` без источника события в v2 не регистрируется (гард `seam_without_consumer.py` расширяется проверкой, PR-7).
- **Идемпотентность по `turn_id`**: повторный `ActionRequest` отбрасывается владельцем (ADR-0149 I6).
- Несколько действий в одной команде раскладывает **планировщик ресурсов** (§5.6) по `knowledge.COMPOSITION`: совместимые по ресурсам — параллельно (анимация + речь + подложка), конфликтующие — по политике класса. `_order_tool_calls` (`agent_core.py:395`, музыка вперёд, `stop_*` в конец, холостой `deferred_call_ids :426-431`) не переносится. Фразы по событиям обычных действий собираются в одну реплику (`respond.join`); выступление — последовательность сегментов с моментами старта (исправление по 07 §9 п.3).

### 5.3 Как событие возвращается в диалог

Все события владельцев — типизированные топики (§10). `execute.py` сопоставляет событие с ожиданием по `(turn_id | event_ref)`; нет совпадения → игнор + `WARNING stale_event`; нет события → таймер класса → `timeout`. Успех = событие (закрывает #2949).

### 5.4 Каталог фраз

`rob_box_dialog/phrases/ru.yaml`: ключ `{class}.{status}[.{reason}]`, 1–3 варианта, слоты. Тест покрытия всех `(class,status)`. `MediaRouter.*_TEXT` (`media_router.py:165-176`) переезжают сюда; «Принял.», «Что-то я задумался», «Я тут растерялся…» удаляются. Пустой/невалидный ответ LLM → `understand.invalid` («Не понял, повтори, пожалуйста»). Длинный `speech` озвучивается до границы предложения в лимите, остаток — `WARNING say_truncated` (A13).

### 5.5 Экспрессия и живость без LLM (решение В5)

**Сегодня** (V26): анимацию выбирает LLM аргументом `speak_text(animation=…)` или тулом `play_animation`; `animation_player` сам переключает «talking/idle» по `/voice/tts/state`; кольцо ReSpeaker (`led_node`) слушает `/voice/dialogue/state`, `/audio/direction` и `/voice/animation/request`; earcon «услышал» шлёт stt_node (`boop`), ещё звуки — dialogue_node, mcp-тул, health_monitor — 4 писателя `/voice/sound/trigger` без владельца решения. Живость стоит вызовов LLM и зависит от того, вспомнит ли модель про `animation`.

**Два слоя.**

1. **Рефлексы кода** — `expression.py` по `TurnEvent.stage`, таблица `knowledge.REFLEXES` (данные, не промпт), 0 вызовов LLM:

| Стадия хода | Матрица (`KNOWN_ANIMATIONS`) | Кольцо | Earcon | Когда |
|---|---|---|---|---|
| `addressed` | `wakeup` | подсветка по DOA | `heard` (сегодняшний `boop → button_click`) | ≤ 300 мс после `utterance` с вейком |
| `thinking` | `thinking` | медленное вращение | — | от вызова LLM до команды |
| `executing` | по классу: `motion → turn_left/right`, `media → talking`-пауза, прочее — `thinking` | — | — | от `ActionRequest` до события |
| `speaking` | `talking` (как сегодня у `animation_player` по TTS) | дыхание | — | `SpeechEvent started…finished` |
| `done` | `happy` (кратко) или эмоция команды | вспышка | `done` (короткий) | по последнему событию |
| `failed`/`timeout` | `sad`/`error` | красная вспышка | `fail` | по `ActionResult.status` |
| `silenced` | `sleep` | погашено | — | кадр `Silence` на стеке |
| `not_addressed` | ничего | ничего | ничего | фраза без вейка |

2. **Экспрессия в команде** — `Command.expression{emotion, gesture, earcon}` из enum (`EMOTIONS` ⊂ `KNOWN_ANIMATIONS` плюс алиасы `ANIMATION_ALIASES` остаются только как нормализация ввода, не в схеме); едет в той же команде, что речь и действия, — **дополнительных вызовов LLM нет**. Эмоция команды накладывается на рефлекс `speaking`/`done`; при `failed` — рефлекс побеждает (нельзя «радостно» о провале).

**Владение.** Решение об экспрессии — у агента-владельца хода (`expression.py`); исполнение — у трёх исполнителей по одному типизированному `ExpressionRequest{turn_id, epoch, agent, emotion, gesture, earcon, ttl_ms}` (§10.1): `animation_player_node` (матрица), `led_node` (кольцо), `sound_node` (earcon'ы). Каждый исполнитель отвечает `ExpressionEvent{started|skipped{reason}}` — например, `sound_node` честно отвечает `skipped{voice_stream_active}` вместо тихого «пропускаю эффект» (`sound_node.py:223`). Писатели: агенты через один клиент `rob_box_core.expression_client`; `/voice/animation/request` и «boop» в `/voice/sound/trigger` удаляются (PR-3b). `/voice/sound/trigger` остаётся только для mcp-тула `play_sound` (файл по имени — это `action.media`, не экспрессия) и health_monitor (системный, в `writers_allowed`). Пакет `rob_box_animations` и `led_node` — исполнители, их внутренняя логика не переписывается.

**Приёмка:** A1 считает **только вызовы LLM**; невербальные действия — отдельно A17 (каждый адресованный ход имеет ≥ 1 рефлекс `addressed` ≤ 300 мс и ≥ 1 терминальный рефлекс `done|failed`). Порог A1 = 3 — **принят предварительно (В5), уточняется после замера в PR-8** на марафоне с включёнными рефлексами.

### 5.6 Исполнение во времени: планировщик ресурсов

Товарищ Шифу (02.10): «робот читает рэп про одно, а если ему в процессе нагрузить — чтоб он дочитывал… решение, что все тулы идут в шедулер, поросло заплатками и потерялось. Должно работать: робот проигрывает анимацию и говорит одновременно, либо запустил бит и читает рэпчик». Разбор — `07-action-scheduler.md`; здесь — дизайн.

#### 5.6.1 Ресурсы и владельцы

| Ресурс | Владелец-исполнитель | Событие владельца (статус задачи) | Что принимает |
|---|---|---|---|
| **Голос** (речь в динамик/наушник) | `tts_node` | `SpeechEvent queued/started/progress{spoken_chars}/finished/cancelled` (§6.1) | `SpeechRequest{…, at: время старта | null, boundary: sentence|none, commit: bool}` — новое: «играть в момент t» и пред-синтез без воспроизведения |
| **Лицо** (матрица) | `animation_player_node` | `ExpressionEvent started/skipped{reason}` (§5.5) | `ExpressionRequest` |
| **Кольцо LED** | `led_node` | `ExpressionEvent` | `ExpressionRequest` |
| **SFX** | `sound_node` | `SoundEvent started/finished/skipped{busy|voice_stream}` — **честно вместо молчаливого `return`** (V33) | `ActionRequest play_sound` / `ExpressionRequest earcon` |
| **Музыка** | `PlayerOwner` (mcp_server, ADR-0149) | `/voice/music/event started{bpm, beat_at, bar}/nearly_finished/finished/rejected/idle` | `request_music(intent=backing|track)`, `dj_set` |
| **Движение** | Nav2 через command_node/mcp | результат action, `feedback` | `Intent`, `navigate_*` |
| **Внешнее** (память, поиск, Telegram-текст) | свои сервисы | ответ тула | `ActionRequest` |
| **Расписание** (кто что занял, политика, границы; выступление как целое) | `rob_box_dialog.plan` в процессе агента (рекомендация; О6) | `PlanEvent block start/end, segment started/finished, prediction` | команды агента |

**Статус задачи приходит только от владельца ресурса.** `{"status":"queued"}` как ответ тула исчезает: в LLM возвращается `ActionResult` по событию, а для ещё не начатого — структурный блок «в очереди после X, старт ≈ через N с» (BML `predictionFeedback`). Канал `VOICE`, который считал задачу выполненной по публикации (V28), не существует.

#### 5.6.2 Матрица одновременности (`knowledge.COMPOSITION`, данные)

Обозначения: ✅ параллельно (MERGE); ⏭ после текущего (APPEND); ⟳ заменить на границе (REPLACE); ✖ отказ с фразой (REJECT).

| новое ↓ \ занято → | голос: реплика | голос: **выступление** | лицо/кольцо | SFX | музыка: подложка выступления | музыка: трек/сет | движение |
|---|---|---|---|---|---|---|---|
| **реплика** | ⏭ (дефолт, О7) | ⏭ дочитать сегмент → реплика → продолжить; ⟳ на границе предложения только по Tier-1 «хватит/стоп» | ✅ | ✅ | ✅ (ducking — О8) | ✅ (ducking) | ✅ |
| **выступление** | ⏭ | ⏭ по умолчанию; **MERGE в PENDING-сегменты**, если `edit_pending` («и про енота»); ⟳ только Tier-1 | ✅ | ✅ | ⟳ смена подложки на такте, речь продолжается | ⟳ подложка вытесняет трек на границе фразы (ADR-0149) **или** ✖ «сначала выключить трек?» — О9 | ✅ |
| **экспрессия** | ✅ MERGE с TTL поверх рефлекса `speaking` | ✅ | MERGE | ✅ | ✅ | ✅ | ✅ |
| **SFX** | ✅ | ✅ | ✅ | ⏭ или вытеснение по приоритету; всегда событие, не молчание | ✅ | ✅ | ✅ |
| **музыка: старт** | ✅ | ⟳ подложка на такте, речь не рвётся (**#993**) | ✅ | ✅ | ⟳ на такте | ⟳ на фразе (ADR-0149) | ✅ |
| **движение** («направо») | ✅ | ✅ параллельно (решение SCHEDULER_DESIGN v5 Q5, сегодня нарушено — 07 Х11) | ✅ | ✅ | ✅ | ✅ | ⟳ preempt Nav2 |
| **«стой»** (Tier-1) | ⟳ сразу: речь, выступление, движение; музыка — `dj_set(stop)`/стоп подложки по снимку | | | | | | |
| **query** | ✅ параллельно, отменяется при перебивании | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |

Непрерываемые классы (`motion`, `identity`) не вытесняются ничем, кроме «стой» (для `motion`). Таблица — единственное место знания об одновременности; тест проверяет поведение `compose()` на всех парах, а не текст промпта (ADR-0148 §3).

#### 5.6.3 Класс действия «выступление» (`action.perform`)

Песня, рэп, сказка, стих — **не N независимых `speak_text`**, а одно действие с тремя связанными частями и общим временем жизни:
- **речь** — сегменты (куплеты/абзацы) из `Performance.segments`, режет код (сегмент ≤ 400 символов, по строфам/предложениям); состояние сегмента `PENDING → ACTIVE → DONE`;
- **подложка** — `request_music(intent=backing, mood=…)` к `PlayerOwner`; форма ≥ Σ длительностей сегментов (из пред-синтеза) — с запасом на outro;
- **экспрессия** — рефлекс `speaking` + `expression_per_segment`.

Жизненный цикл — `Parallel(success_count=1)` (BehaviorTree.CPP): речь дочитана → подложке `outro` на границе фразы → `stop`; подложка кончилась раньше — продлевается проходом формы (ADR-0149 I1). **`required`** (BML): подложка `rejected` или не `started` за 6 с → выступление **не начинается «в тишину» молча**: код говорит шаблон `perform.no_backing` («Бит не завёлся — читаю так») и продолжает без подложки, либо, если `backing` обязателен для `kind=rap` (О10), отказывает честно. Это заменяет `_pending_music_cleanup`, `_active_batches`, отсрочку при живом ходе, `_track_mode_music_active`, `/mcp/music_cleanup` (07 П9).

**Пред-генерация.** Все сегменты синтезируются заранее (`SpeechRequest commit=false` → `SpeechEvent synthesized{speech_id, duration_ms}`), воспроизводятся по `commit` в момент `at`. Длительности дают раскладку по тактам и ETA для блока LLM «осталось N с». Это единственная реализация пред-генерации: поле `pregenerate` payload чанка (ADR-0056/0092), которое никто не публикует (V32), **удаляется** вместе с `scheduler/pregen/decision|estimator|quality|speculative_executor` (решение по п.5 задания: оставлять неподключённый механизм рядом с подключённым — две реализации); `estimator.py` переносится как библиотечная функция оценки длительности до синтеза.

**Старт на такте (X2).** `sync.next_bar(started{bpm, beat_at, bar}, output_latency_ms) → t`: первый сегмент стартует на ближайшей границе такта подложки после `now + output_latency`; каждый следующий — на ближайшем такте после `finished` предыдущего (Ableton Link quantized launch, BML `synchronize speech:start ↔ music:bar`). Источник доли — колбэк `PlayerOwner.track_started` из потока клока (V35) и далее `beat_at` в снимке; `output_latency_ms` **меряется** `jack_rec` (память «Аудит записей»), не угадывается. Старт на такте есть только у `PlayerOwner` v2 → при `music_engine: v1` выступление идёт **без квантования** (подложка v1 стартует сразу, сегменты — по `finished`), и это честно отражается в `PlanEvent.synced=false`.

#### 5.6.4 Политики при новой команде во время исполнения (BML)

| Политика | Смысл | Граница |
|---|---|---|
| **APPEND** | начать после конца текущего блока («а потом спой колыбельную», X4; реплика во время выступления) | конец блока / сегмента |
| **MERGE** | исполнять вместе, не трогая начатое; для выступления — правка только **PENDING**-сегментов (`edit_pending`, X3 «и про енота») | сразу; правка — ближайший PENDING |
| **REPLACE** | закончить текущее и начать новое | голос — конец предложения, не позже 3 с; музыка — такт; «стой» — сразу |
| **REJECT** | не исполнять, честно сказать почему | — |

**Кто решает (ADR-0148):** (1) код — `compose(new_cls, busy_cls)` по `COMPOSITION`; (2) Tier-1 переопределяет по закрытому списку: «стоп/хватит/замолчи» → REPLACE на границе, «стой» → REPLACE сразу, «потом/после/когда закончишь» → APPEND, «сейчас/прямо сейчас» → REPLACE на границе; (3) LLM — только `when ∈ {after_current, now}` и `edit_pending`; (4) фразу «Дочитаю и включу» / «Поставлю после куплета» строит код по `PlanEvent queued{after, eta}`. `stt_node` STOP на любой вейк (V29, `stt_node.py:2035-2045`), `barge_in_policy` с обеими ветками и `quick_decide` уходят: вейк во время речи = `interrupt_request` → `compose()`, а не STOP (§6.2).

**Эталонные сценарии (тесты поведения `plan/` + e2e):**
- **X3 «дочитай и вплети»** (#968 «комар + енот»): рэп 6 сегментов, на ACTIVE=2 приходит «Робби, и про енота!» → LLM `perform{edit_pending=true, segments=[новые 3..6]}` → MERGE: сегмент 2 дочитывается до конца, 3–6 заменяются, подложка не трогается; `PlanEvent segment replaced{3..6}`; A20.
- **#993 «добавь музыку во время рэпа»**: рэп без подложки, на ACTIVE=1 «Робби, добавь бит» → `request_music(intent=backing)` класс `action.media` против занятого «выступление» → ⟳ подложка на такте, речь **не обрывается**, сегмент 2 стартует уже на такте; A22.
- **X4 «а потом спой колыбельную»**: `when=after_current` → APPEND, фраза «Спою после рэпа», `PlanEvent queued{after=perform_1}`; A19.
- **X5 «хватит, анекдот»**: Tier-1 «хватит» → REPLACE на границе предложения (≤ 3 с), outro подложки, затем новый ход; A21.
- **X11 «направо» во время песни**: `motion` ✅ параллельно, песня не рвётся.

#### 5.6.5 Что это удаляет (07 §1.6–1.7, П15)

Канал `VOICE` с «завершено = опубликовано», `wait_until_idle`, `_pending_music_cleanup`/`_active_batches`/прелюдия/`_track_mode_music_active` (`dialogue_node.py:1085-1120, 3396-3460`), `batch_registered/batch_complete` из mcp_server, мёртвые хуки `TaskScheduler` (`cancel`, `set_llm_continue_hook`, `notify_event`, `set_eta_provider`, `set_group_boundary`, `set_frozen_touch_hook`), `task_delta` (тул, схема, скилл `scheduler.txt`), `[ACTIVE TASKS]/[SEGMENT PLAN]` с `eta=?`, `scheduler_shadow.py` (V31), `action_server/` + сайдкар `voice-action-server` (ADR-0011), `register_tts`, `deferred_call_ids`, пять гардов исполнителя (#2859 «один трек за ход», #2878/#3221 лимит DJ-реплик, #2913 гейт речи до регистрации, #3246/#3247 отказы DJ_AUTO — становятся полями `ACTION_CLASSES`/проверками валидатора или исчезают с ADR-0149), второй планировщик в `tts_node` (`priority`-вклинивание `:1435`, `_pending_speech_queue` `:1468`, буфер «STOP после синтеза» `:1868-1900`). PR и grep-критерии — §11 (PR-7b/7c).

---

## 6. Речь, перебивание, тишина

### 6.1 Владелец речи — tts_node

- Один вход `/voice/speech/request` (`SpeechRequest`: `speech_id`, `turn_id`, `epoch`, `agent ∈ {personality, operator, telegram, system}`, `sink`, `priority`, `interruptible`, `text`, `voice?`, `emotion?`). Все писатели v1 (V11) идут через клиент `rob_box_core.speech_client.say(Utterance, …)`; `Utterance` (`rob_box_core/utterance.py:119`) — полезная нагрузка.
- Один канал отмены `/voice/speech/cancel` (`SpeechCancel{speech_id | epoch, agent}`), писатели — агенты-владельцы сессий; отмена по эпохе не трогает реплики другой эпохи/другого агента. `IGNORE_STOP_MS:700` (T1) исчезает.
- События `/voice/speech/event`: `queued, started{text_len}, progress{spoken_chars}, finished, cancelled{spoken_chars}, voice_changed{requested, applied, provider}, rejected{reason}`. Агент пишет в историю `text[:spoken_chars]`.
- Нормализация текста для TTS — только в tts_node (R14). `SYSTEM_TEMPLATE_REGURGITATE` в tts_node остаётся **только** как страховка с меткой `TEMP(ADR-0148` и счётчиком → удалить при 0 за марафон (R9).

### 6.2 Политика перебивания (`interrupt.py`)

| Сигнал | Кто решает | Действие |
|---|---|---|
| Фраза **с вейком** во время речи робота — **Tier-1 «стоп/хватит/замолчи/стой»** | `grammar.py` → `interrupt.py` | REPLACE: «стой» — `SpeechCancel(epoch)` сразу (≤ 300 мс); «хватит/стоп» — на границе предложения (≤ 3 с), outro подложки; стрим LLM отменяется по эпохе до HTTP-клиента (`AgentCore.cancel(epoch)`, #1280); `motion`/`identity` не отменяются (кроме «стой» для motion), `query` — отменяется (Pipecat `cancel_on_interruption`) |
| Фраза **с вейком** во время речи робота — **не стоп** (дополнение, новая просьба) | `address.py` → ход LLM → `compose()` (§5.6.4) | речь **не отменяется** (исправление по 07 §9 п.2): во время **реплики** новая реплика — APPEND; во время **выступления** — APPEND после сегмента или MERGE в PENDING (`edit_pending`), подложка меняется на такте; действия, совместимые по ресурсу (анимация, движение, SFX) — параллельно. Рефлекс `addressed` (earcon) подтверждает, что услышал, пока дочитывает |
| Фраза **без вейка** во время речи робота | `address.py` | игнор всегда (В2; `0326d7e9f`: без гейта робот прерывал сам себя эхом) |
| «Стой/стоп» (Tier-1) | `grammar.py` | `stop_motion` + `SpeechCancel` + `dj_set(stop)` по снимку |
| Эхо/короткий шум | audio/stt как сегодня (грейс 2.5 с, T2) | остаётся с `TEMP(ADR-0148`; ложное прерывание с возобновлением — не в этом витке (§17 Г) |
| Новая сессия | `grammar.py` | `Session.reset()` → эпоха+1 |

Писателей STOP становится: агенты через один клиент (было 4 ноды, V12). stt_node публикует `/dialog/interrupt_request{utterance_id}`, диалог решает за ≤ 20 мс: Tier-1 стоп-слово в первом сегменте STT → REPLACE немедленно; иначе ход идёт как обычно, речь продолжается. `barge_in_policy` (`replace`/`classify`, V29) и `quick_decide` удаляются — политика одна, табличная. Если замер в PR-9 покажет > 300 мс до тишины — stt_node получает право прямой отмены **только по эпохе** из latched `/dialog/session` (§16).

### 6.3 Тишина (решение В3)

- `Silence` — кадр сцены `{set_by: user|operator, ttl_s, reason}`. Голосовое «хватит/помолчи» → `set_by=user, ttl=600 с` (**10 мин, Шифу 02.10**); операторская пауза (`DialogControl.hold`) → `set_by=operator`, до `resume`. `resume` оператора снимает только операторский кадр; пользовательская тишина кончается по TTL, по «говори/отвечай» (Tier-1, с вейком) или по новой сессии.
- В SILENCED Tier-1 работает; `available_tools` не содержит ничего, что звучит; рефлекс `silenced` (`sleep`, кольцо погашено); «Останавливаюсь» от command_node через диалог молчит (вопрос 20, 03 §2).
- Озвучка от **других агентов** (ТАРС `say`, телеграм `say_aloud`) в пользовательской тишине — правило §9.2.3, открытый вопрос О2.
- `check_silence_timeout` в DSM удаляется вместе с DSM.

---

## 7. Сессия, сцены, персоны, история, память

### 7.1 Session

```python
@dataclass(frozen=True)
class SessionSnapshot:
    agent: str; session_id: str; epoch: int; started_at: float
    scenes: tuple[SceneFrame, ...]          # стек: Silence | Persona | DJ | OperatorHold | Identity(ask)
    voice: VoiceState                       # requested, applied, provider  (одно место вместо D6)
    speaker: SpeakerRef | None              # person_id, name, confidence, since
    pending_ask: Ask | None                 # робот задал вопрос; ответ ожидается С вейком (В2)
    music: MusicStateRef                    # копия latched-снимка плеера
    providers: Mapping[str, ProviderHealth]
```

Пишет только владелец сессии через owner-методы. LLM получает снимок как **структурный read-only блок**. Рестарт процесса → новая сессия, эпоха с нуля, latched-снимок перезаписывается (В11); музыка переживает рестарт своим снимком (ADR-0149).

### 7.2 Сцены как кадры стека

| Кадр | Push | Pop | Что возвращается при pop |
|---|---|---|---|
| `Silence(set_by, ttl)` | Tier-1 «хватит» / `DialogControl.hold` | TTL (10 мин), Tier-1 «говори», `resume` того же `set_by`, reset | речь разрешена |
| `Persona(name, voice)` | действие `set_persona` / `dj_set start` | явная команда, конец DJ (`/voice/music/event idle`), reset | голос и промпт прошлого кадра (#3000, акт 4 марафона) |
| `DJ(set_id)` | `/voice/music/event started{dj.enabled}` | `idle`/`finished`/`dj_set stop` | срез тулов без `dj_set(next…)` |
| `OperatorHold` | `DialogControl.hold(set_by=operator)` | `resume`, таймаут супервизора | — |
| `Identity(ask, person)` | код задал «X, это ты?» | ответ да/нет (Tier-1, с вейком), таймаут 15 с | — |

`Session.reset()` выталкивает все кадры, увеличивает эпоху, очищает окно LLM, **не трогает** долгую память о людях и плеер («забудь всё» ≠ «выключи музыку» — #2835/#3217 решаются двумя разными Tier-1 командами).

### 7.3 История для LLM

Источник — `turn_log` текущей сессии; проекция: `user: <текст фразы>` (без `[Spkr:]`, `[TG]`, `[URGENT_BACKLOG]` — эти факты в снимке и в поле `source`), `assistant: speech[:spoken_chars]`, `tool: ActionResult` структурой. Отвергнутые команды и невалидный JSON в окно не попадают (#3145). Окно ≤ 12 ходов / ≤ 3k токенов; сброс по `reset()`; смена `Persona` — `context_strategy` кадра (`RESET` для DJ, `APPEND` иначе; Pipecat Flows). Бэклог неадресованных фраз — поле снимка `recent_unaddressed` (≤ 3, ≤ 120 с), как и сегодня аккумулятор (`7e8d8c208`): без вейка робот не отвечает, но помнит.

### 7.4 Память о людях и турнах (решение В8)

- Директива Шифу 02.09: турны в prod не персистятся. v2: `turn_log` **в памяти процесса**, ретенция — сессия. На диск в проде — только агрегаты без текста (`turn_id, agent, stage, status, latency_ms, llm_calls, llm_provider`) для §14.
- **Приёмка (В8 — разрешено):** при `e2e_mode` текст турнов и речи пишется в **отдельную e2e-БД** по пути `e2e_db_path` **вне `/data`** (боевая `/data` общая с живыми людьми — память `robot-data-is-shared-with-live-people`; ADR-0128), таблица `dialog_events`; её читают `honesty_audit.py`/`turn_metrics.py`. Включается только харнессом, в проде ключ отсутствует (тест: дефолт пуст → запись текста невозможна).
- Факты о людях: только через действие `remember(person_id, fact)` → владелец памяти (mcp_server, одна БД `harness_voice.db`, одна таблица для записи и чтения — #2793) → событие `saved{fact_id}` → фраза. LLM не пишет в память сама (05 §7). Чтение — `memory_search(person_id)` (#1770).
- ТАРС-журнал (#3297): записи с `ts` и `ttl`; в контекст — только моложе 30 мин плюс сводка «за сутки» с датой.

---

## 8. Провайдеры: возможности и честная деградация

- `ProviderState` (latched `/voice/providers/state`): `{name, kind: llm|stt|tts, health: ok|degraded|quota|auth|down, since, capabilities{…}, last_error_code}`. Классы ошибок — enum; подстроки `2056/1008/token plan` (`health.py:163-168`) остаются только внутри адаптера MiniMax.
- `degrade.py`: режим команд (§4.5), порядок провайдеров (`HealthAwareFallbackLLM` как транспорт), **одна** фраза на смену состояния по шаблону, не чаще 1 раза в 10 мин.
- `VoiceState.requested ≠ applied` видно в снимке; «каким голосом ты говоришь» — Tier-1 из снимка.
- Бюджет времени хода `TURN_DEADLINE_S = 12` (В5): истёк → отмена ожиданий, `understand.timeout`, лог `provider_latency`. Таймауты провайдеров = `min(provider_timeout, deadline_left)`.
- Мёртвые параметры (`llm_timeout_sec`, `agent_max_turns`, `<provider>.*`, 03 §1.6; `llm_*` в `telegram_bot.yaml`, V27) удаляются (PR-4, PR-12).

---

## 9. Интерфейс с музыкой (ADR-0149), Telegram-агент, ТАРС

### 9.1 Музыка — только интерфейс (не перепроектируется; решение В9)

- Диалог **читает** `/voice/music/state`, `/voice/music/event` (владелец `PlayerOwner`) и **вызывает** узкие `request_music`/`dj_set` как `action.media`; Tier-1 медиа-команды — `MediaRouter → PlayerOwner` без LLM (ADR-0149 I19).
- Фраза об успехе — из `started{track_id, title, bpm}` / `rejected{reason}` (`media.started`, `media.rejected.{reason}`); при `music_engine: v1` событий нет → `request_music` в `available_tools` отсутствует, работает только `MediaRouter` v1.
- Диалог v2 **не вызывает** `MusicGuard`, `DJModeController.tick`, `dj_set_boundary` ни при каком флаге и **ничего из них не удаляет**: удаление — у эпика #3312 (PR-13…15 ADR-0149), «там обещали всё удалить и почистить» (Шифу 02.10). Зависимости — §11.1.
- Кадр `DJ` создаётся/снимается по событиям плеера; `/voice/dj_mode` диалог не пишет.
- **Подложка выступления** (§5.6.3) — `request_music(intent=backing)` к `PlayerOwner`; владелец плеера даёт `started{bpm, beat_at, bar}` для старта речи на такте (ADR-0149 I9, `player_owner.py:78-89`), `nearly_finished` для продления формы под длину речи и принимает `outro/stop` от планировщика выступления. Политика «подложка против играющего трека/сета» — О9. Ничего в движке музыки ради этого не меняется, кроме контракта `request_music(intent=backing, min_form_beats)` — если его нет в #3312 PR-6, это запрос к эпику, не правка здесь.

### 9.2 Telegram — отдельный агент на общем движке (решение В10)

**История** (§18.1): с 28.02 (`3c91ba155`) у телеграм-бота была **своя LLM** (DeepSeek/Qwen, независимые сессии на пользователя, роль оператора, MCP-мост на 28 тулов, `/say`) — это тот режим, когда «сочини историю и расскажи» работало: LLM телеграма сочиняла и звала `say`. 28.07 W7 (`07dfc28aa`) LLM удалили, бот стал мостом в `/voice/stt/result` → ход Личности; эхо-ответов удалили (`88cecc91f`) и вернули 13.08 (#1195, `def24baaa`) с маркером `[TG:chat_id]`, пропуском вейк-гейта и правилом «оператор шепчет». С тех пор телеграм — вторая голова голосового диалога: пишет 5 голосовых топиков (V14), делит историю и персону с голосом, его речь проходит те же гарды. Параметры `llm_*` в `telegram_bot.yaml` мертвы с W7 (V27).

**Дизайн.** `rob_box_telegram` поднимает `rob_box_dialog` с `AgentSpec(telegram)`:
- **своя LLM-сессия на `chat_id`** (`Session.agent="telegram"`, своя история, своя персона «ассистент в чате», свой `ProviderState(llm)`; провайдер — из yaml, те же адаптеры §4.5);
- **адресация всегда** (текст в чате адресован), Tier-1 — слэш-команды и те же грамматические команды;
- **тулы** — тот же каталог через `tools_view(AgentSpec.telegram)`: команды роботу (навигация, фото, музыка, память, громкость) идут к **тем же владельцам** с событиями; фраза о результате — из того же `phrases/ru.yaml`, но в **текстовый канал** (`DialogOutput(chat_id, text)`), а не в динамики;
- **действие `say_aloud(text, voice?)`** класса `action.speak_aloud`: `SpeechRequest(agent=telegram, sink=speakers)` → `SpeechEvent finished{seconds}` → в чат «Озвучил (12 с)» — по событию, а не «отправил»; `cancelled/rejected{reason}` → честно «Не озвучил: робот в тишине / занят / TTS недоступен»;
- `speech` команды — текст ответа в чат; эталонный сценарий «сочини историю и расскажи»: LLM телеграма выдаёт команду `{speech: "<история>", actions: [say_aloud(text=<история>)]}`; по правилу §4.1 `speech` при действиях не озвучивается динамиками (его озвучивает действие), а в чат уходит текст истории + подтверждение по событию.

**9.2.3 Совместный доступ к речи и тишине.** Голосовая сессия и телеграм-сессия — разные объекты; общий ресурс — tts_node. Правила:
- очередь tts_node: `priority` по агенту, реплики разных агентов не перебивают друг друга, если `interruptible=false`;
- `say_aloud` при кадре `Silence(set_by=user)` у голосового диалога: **предложение** — исполнить (телеграм — операторский класс, как «оператор шепчет» в #1195), но ответить в чат «робот в тишине ещё N мин, озвучил по вашей просьбе»; альтернатива — отказ с кнопкой «всё равно озвучить». **Решает Шифу (О2)**;
- `say_aloud` при `OperatorHold` — отказ `rejected{operator_hold}`;
- floor: как сегодня через `supervisor_client` (`voice_floor`), не меняется.

Телеграм перестаёт писать `/voice/stt/result`, `/voice/dialogue/response`, `/voice/tts/request`, `/voice/sound/stop`; `/avatar/command` → супервизор остаётся (операторские команды режима). Маркер `[TG:chat_id]` и `tg_chat_id` в `dialogue_node` удаляются (PR-12).

### 9.3 avatar-supervisor (ТАРС) — тот же движок, другая спецификация (решение В7)

- `AvatarSupervisor` строит `rob_box_dialog` с `AgentSpec(operator)`: своя грамматика Tier-1, срез `operator.*` (после удаления 5 призраков D17), те же `Command`, `Executor`, `phrase_from_result`, `SpeechRequest(agent=operator, sink=headset|speakers)`.
- **`say` остаётся** у ТАРС как `action.speak_aloud` с событием `SpeechEvent` (Шифу 02.10 «да, а»); регистрируется в каталоге с `operator_visible` (как #3305), результат — по событию, не «success=True» при публикации (память `speak-through-robot-needs-ssml`).
- `/dialogue/control` + `control_ack` → сервис `/dialog/control` (`DialogControl.srv`) со структурным ack.
- ТАРС-журнал по §7.4; `operator_system_prompt.txt` генерируется из `knowledge` + реестра (#3298).

---

## 10. Типы сообщений: уход от JSON в `std_msgs/String`

### 10.1 Пакет `rob_box_dialog_msgs`

| Тип | Поля (кратко) | Заменяет |
|---|---|---|
| `msg/SpeechRequest` | `speech_id, turn_id, epoch, agent, sink, priority, interruptible, text, voice, language, emotion, at (время старта, 0 = сразу), boundary {none, sentence}, commit (false = только синтез), stamp` | `/voice/tts/request`, `/voice/dialogue/response`, поле `pregenerate` чанка |
| `msg/PlanEvent` | `block_id, turn_id, epoch, agent, kind {block_start, block_end, segment_started, segment_finished, segment_replaced, queued, prediction, rejected}, segment_no, after_block, eta_ms, synced, reason, stamp` | `/harness/task_events`, `[ACTIVE TASKS]/[SEGMENT PLAN]` в контексте |
| `msg/SoundEvent` | `request_id, kind {started, finished, skipped}, reason {busy, voice_stream}, sound` | молчаливый `return` в `sound_node.trigger_callback` (V33) |
| `msg/SpeechCancel` | `speech_id, epoch, agent, reason` | `/voice/tts/control` STOP/IGNORE_STOP_MS |
| `msg/SpeechEvent` | `kind {queued,synthesized,started,progress,finished,cancelled,voice_changed,rejected}, duration_ms, speech_id, turn_id, epoch, agent, spoken_chars, text_len, seconds, voice_requested, voice_applied, provider, reason, stamp` | `/voice/tts/state`, `/finished`, `/batch_*`, `/current_voice`, часть `/provider_state` |
| `msg/ExpressionRequest` / `msg/ExpressionEvent` | `turn_id, epoch, agent, emotion, gesture, earcon, ttl_ms` / `executor, kind {started,skipped}, reason` | `/voice/animation/request`, «boop» в `/voice/sound/trigger`, auto-switch по `/voice/tts/state` |
| `msg/SessionState` | `agent, session_id, epoch, scenes[] (SceneFrame), voice_requested, voice_applied, speaker_id, speaker_name, pending_ask, stamp` | `/voice/dialogue/state`, `/voice/dialogue/barge_in_policy` |
| `msg/SceneFrame` | `kind, set_by, ttl_until, payload_json` | — |
| `msg/TurnEvent` | `turn_id, epoch, agent, stage, status, reason, llm_calls, llm_provider, latency_ms, source, stamp` | `/harness/task_events`, `/dialogue/control_ack` |
| `msg/ActionRequest` / `msg/ActionResult` | `turn_id, epoch, agent, tool, args_json, cls, timeout_s` / `turn_id, tool, status, reason, data_json, event_ref` | `/mcp/execute`, `/mcp/result` |
| `msg/ProviderState` (+Array) | `name, kind, health, since, tool_calls, tool_choice, json_mode, structured_output, thinking, ssml, streaming, voices[], last_error_code` | `/voice/tts/provider_state`, `~/.rob_box/llm_health.json` |
| `msg/DialogOutput` | `agent, channel_ref, text, speech_id` | telegram ← `/voice/dialogue/response` |
| `msg/Intent` | `turn_id, epoch, kind {stop_motion, move, …}, args_json` | command_node ← `/voice/stt/result` |
| `msg/Utterance`, `msg/SpeakerResult` | по ADR-0131 | `/voice/stt/utterance`, `/voice/speaker/result` (String) |
| `srv/DialogControl` | req `action {hold,resume,reset}, set_by, ttl_s` → resp `ok, epoch, scenes[]` | `/dialogue/control` + `control_ack` |

Правило: поле `*_json` — только для открытых словарей аргументов тулов (схему проверяет каталог). QoS: события — RELIABLE KEEP_LAST 10; снимки — latched. `DialogInput` из ревизии 1 не нужен: телеграм — агент, не канал ввода в голосовой диалог.

### 10.2 Переход

Типизированный топик вводится вместе с потребителем в одном PR (`seam_without_consumer.py`); старый `String`-топик удаляется в том же PR, когда все писатели переведены; внешний пир (quest) — `seam_allowlist.json` с датой. Два контракта одного состояния параллельно — только внутри одного PR-окна.

### 10.3 `architecture/ownership.yml`

PR-2 заполняет `topics:` для `/dialog/*`, `/telegram/*`, `/avatar/session`, `/voice/speech/*`, `/voice/expression/*`, `/voice/providers/*`, `/voice/stt/*`, `/voice/speaker/*`, `/voice/music/*` и `nodes:` с `owner` и `writers_allowed`. Гард `scripts/lint/ownership_check.py`: (а) каждый топик этих префиксов имеет владельца; (б) рантайм-снимок не содержит `multiple_writers` вне `writers_allowed`. Сегодняшние 17 — в `runtime-baseline.json` как долг с номером PR.

---

## 11. Миграция: strangler за флагом `dialog_engine`, план PR

**Флаг** `dialog_engine: "v1" | "v2"` — ROS-параметр launch-файла (общий yaml `docker/vision/config/voice_assistant/*.yaml`, дубль в `src/.../config` синхронизируется тестом). `v1`: `dialogue_node` + адаптер совместимости в tts_node (`SpeechRequest` ↔ старый JSON; один модуль, удаляется PR-15). `v2`: `dialog_node`, `dialogue_node` не стартует. Дефолт `v1` до приёмки. Телеграм-агент и ТАРС читают тот же флаг.

Правила каждого PR: ≤ ~600 строк (ADR-AF-0013); гарды `cc_budget.py`, `class_budget.py`, `seam_without_consumer.py`, `cc_budget_refs.py`, `dialogue_skip_reasons.py`, `validate_adr_namespace.sh`, `ownership_check.py`, `temp_adr0148_check.py`; **ни один PR не правит старый путь, кроме удаления**; каждый PR удаляет то, что заменил, с grep-критерием; raw-вывод обязателен (AGENTS.md).

| PR | Что делает | Что удаляет (grep-критерий) | Как проверить | Метрика |
|---|---|---|---|---|
| **PR-0** | ADR; `scripts/dialog/turn_metrics.py`, `honesty_audit.py`; эталонный набор `scripts/dialog/reference_set.yaml` (≥ 60 фраз из issue классов 1–3, #3269/#3221 сказка/рэп, mv03 голоса; разметка ожидаемых действий/речи — утверждает Шифу); baseline по логам марафона 29→30.09 → `docs/dialog/baseline_2026-10-01.md`; гард `temp_adr0148_check.py`; эпик и карточки; развести дубли номеров 0021/0129 | мёртвое: `master_prompt.txt`, `master_prompt_simple.txt` (D10), `voice_command_handler.py` (D11), `startup_greeting_node.py`, `action_server/http_server.py` (D15), D12 — `ls`/grep = 0 | гарды docs; скрипты на логах (вывод в PR) | baseline A1–A17 |
| **PR-1** | `rob_box_dialog`: `knowledge.py` (вейк из `wake_words.yaml`, тишина, громкость, `ACTION_CLASSES`, `EMOTIONS/GESTURES/EARCONS`, `REFLEXES`), `command.py` + валидатор + генерация схем, `session.py`, `phrases/ru.yaml` + `respond`, `agents.py` (3 спецификации) | копии знания: `command_parser.py:97,136-141,319`, `command_node.py:81`, `dialogue_node.py:587`, `dialogue_state_machine.py:431-435`, `dialogue_helpers.py:79-93`, `stt_node.py:226-240`; `ANIMATION_ALIASES` как источник enum — `grep -rn '"хватит"\|"робокс"\|"громче"' src --include=*.py` вне `knowledge.py`/`media_command_grammar.py`/тестов = 0 | unit: 1000 случайных команд → валидатор; `phrases` покрывают `(class,status)`; `class_budget` | A8 (копии 5→1, 4→1, 3→1) |
| **PR-1b** | провайдер-агностик: `ProviderCapabilities` += `tool_choice/json_mode/structured_output/thinking`; честные флаги у `deepseek/minimax/mimo`; `degrade.select_mode`; `speech_channel.py` с **двумя** классами (на время бейк-оффа); `test_llm_adapter_contract.py` (FakeProvider × 4 режима × эталонный набор = 100 %); `scripts/dialog/adapter_benchmark.py` | — (гард: `rob_box_dialog` не импортирует адаптеры — grep = 0) | pytest -v; benchmark по живым адаптерам (raw в PR) | A14 по провайдерам (первый замер) |
| **PR-1c** | **бейк-офф канала речи** (§4.4): `speech_channel_bakeoff.py`, прогон {A, B} × все адаптеры × 3; отчёт `docs/dialog/speech_channel_bakeoff_<дата>.md`; amendment к этому ADR с победителем | — (решение; код канала-проигравшего удаляется в PR-7) | raw-таблица `delivered/leak/double/phantom/llm_calls/latency` | критерий §4.4 |
| **PR-2** | `rob_box_dialog_msgs` (§10.1) + `ownership.yml` + `ownership_check.py`; `/voice/stt/utterance`, `/voice/speaker/result` типизированы с потребителем dialogue_node (v1) | String-версии этих топиков — grep = 0 | colcon build; `ownership_check`; `ros2 topic info -v` на роботе | A7 |
| **PR-3** | tts_node — владелец речи: `SpeechRequest/Cancel/Event` (включая `synthesized{duration_ms}`, `at`, `boundary`, `commit`), отмена по эпохе, `rob_box_core.speech_client`; все писатели → клиент; адаптер совместимости v1 | `IGNORE_STOP_MS` (T1), `/voice/current_dialogue_id` (D16), `batch_registered/batch_complete` из mcp_server, `register_tts` (`speak_helpers.py:663`) — `grep -rn "IGNORE_STOP_MS\|current_dialogue_id\|register_tts\|batch_registered" src --include=*.py` = 0; `create_publisher(... "/voice/tts/request"` вне клиента = 0 | робот: 20 реплик из 3 источников; `spoken_chars` при отмене; `at` — старт в заданный момент с погрешностью (замер `jack_rec`, `output_latency_ms` в PR); рантайм-аудит `/voice/tts/*` | A6 (−5), A12; первый замер латентности вывода для A19 |
| **PR-3b** | экспрессия: `ExpressionRequest/Event`, `expression_client`, рефлексы `expression.py` в dialogue_node v1 (чтобы живость появилась до v2), исполнители animation_player/led_node/sound_node | «boop» из stt_node (`stt_node.py:608`), `/voice/animation/request` (писатели `tools/dialogue.py:102`, `tools/animation.py:34`), auto-switch `animation_player` по `/voice/tts/state` (заменён `speaking`-рефлексом) — `grep -rn '"/voice/animation/request"' src` = 0; писателей `/voice/sound/trigger` 4 → 2 (play_sound, health_monitor) | робот: 20 фраз, лог `ExpressionEvent started` на `addressed ≤ 300 мс`, `done/failed` | A17 |
| **PR-4** | `ProviderState` от tts/stt/агентов; `degrade.py`; `set_voice/set_tts_provider` → запрос + `voice_changed`; фраза деградации | `VoiceStateStore` (D6), «Голос установлен» (`tools/dialogue.py:1464`), мёртвые параметры `llm_timeout_sec`/`agent_max_turns`/`<provider>.*` — grep = 0; подстроки квоты вне адаптера MiniMax = 0 | робот: мёртвый ключ → одна фраза, `health=quota` | A11 |
| **PR-5** | адресация: `address.py` (вейк по границам слов, одна таблица), stt_node → `interrupt_request`, command_node ← `/dialog/intent` | писатели STOP в stt/audio (`"/voice/tts/control"` = 1, адаптер v1), подстрочный матч в stt_node = 0, `DEFAULT_WAKE_WORDS` определён один раз | корпус «работает/робота-диджея» (#1292, #2971); «робот, хватит» → один ответ | A15, A16 |
| **PR-6** | понимание: `grammar.py`, `tools_view`, `context.build_context`, режим команд по capabilities; подключается в dialogue_node v1 как замена `skill_router`/`skill_tool_narrowing` (ADR-0148 §2.4) | `skill_router.py` (27 `re.compile`), ключ `skill_tool_narrowing`, `_MUSIC_STOP_OVERRIDES`, U8, U9, T10 — grep = 0 | unit на грамматике; робот 30 фраз, `tools_visible ≤ 12` | A5 |
| **PR-7** | исполнение и ответ: `execute.py`, `respond.py`, события владельцев (speaker_id `registered`, sound_node `started/finished`); **канал речи-победитель**, проигравший удалён; `seam` проверяет «action-тул имеет событие». **Ждёт #3312 PR-6** (узкие `request_music/dj_set`) для медиа-фраз — до него медиа через `MediaRouter` v1 | в v1-ноде: phantom/universal/`ACTION_CLAIM_RULES`, S4–S7, S12, done-маркеры ×5, R11, фолбэки — `re.compile` в `dialogue_guards.py` 34 → ≤ 15; `_retry_used` 10 → ≤ 3; `_SILENT_DONE_MARKERS` = 0; `speech_channel.py`: 1 класс | эталонный набор на роботе, `honesty_audit.py` | A3 = 0/60 |
| **PR-7b** | **планировщик ресурсов** `rob_box_dialog/plan/{resources,composition,plan_events}.py` + `knowledge.COMPOSITION`, `RESOURCE_OF`; `compose()`; `PlanEvent`; статус задачи — от владельца; подключается в v1-ноде **вместо** `SchedulerToolExecutor` (роутинг тулов по ресурсам, без `queued`); `SoundEvent` в sound_node и `play_sound` по событию (V33) | `scheduler/task_scheduler.py` (1 312), `tool_executor.py` (691), `delta.py`, `event_bus.py`, `quick_decide.py`, `harness/decision/scheduler_shadow.py` (V31), `TaskDeltaTool` + `skills/scheduler.txt`, `_order_tool_calls`/`deferred_call_ids` (`agent_core.py:395-431`), `_VOICE_TOOLS/_MUSIC_TOOLS/_ANIM_TOOLS`, `_MUSIC_PRELUDE_TOOLS/_DEFER_TO_END_TOOLS` (`agent_core.py:189-203`), `[ACTIVE TASKS]/[SEGMENT PLAN]`, `/harness/task_events`, `barge_in_policy` + обе ветки, STOP из stt_node; «Звук запущен» (`tools/sound.py:151`) — `ls src/rob_box_voice/rob_box_voice/scheduler/{task_scheduler,tool_executor,delta,event_bus,quick_decide}.py` = нет; `grep -rn "status.*queued\|task_delta\|SEGMENT PLAN\|barge_in_policy\|scheduler_shadow\|_order_tool_calls" src --include=*.py` = 0; `grep -rn "Звук запущен" src` = 0 | unit: `compose()` на всех парах `COMPOSITION`; робот: речь + анимация + SFX в одной команде — все три `started` в логе (A23); `play_sound` при занятом SFX → `skipped{busy}` и честная фраза | A23, A3 (SFX) |
| **PR-7c** | **выступление**: `plan/perform.py`, `plan/sync.py`, класс `action.perform`, пред-синтез всех сегментов (`commit=false`), `Parallel(success_count=1)` с подложкой, `required`, старт сегмента на такте по `started{beat_at}` при `music_engine: v2` (**ждёт #3312 PR-4/PR-6**: `started` со снимком фазы, `request_music(intent=backing)`), без квантования при v1; MERGE в PENDING (`edit_pending`); `estimator.py` → библиотека длительности | `_pending_music_cleanup`, `_active_batches`, прелюдия, `_track_mode_music_active` (`dialogue_node.py:1085-1120, 3396-3460`), `/mcp/music_cleanup`, `/mcp/music_fallback`; `scheduler/pregen/{decision,quality,speculative_executor,pre_gen}.py` и поле `pregenerate`; второй планировщик tts_node: `priority`-вклинивание (`:1435`), `_pending_speech_queue` (`:1468`), буфер «STOP после синтеза» (`:1868-1900`); пять гардов исполнителя: `track_start_guard.py` (#2859, #2878, #3221, #3246, #3247), гейт #2913 (`tool_executor.py:128`, `turn_speech_gate.py`) — переносятся как поля `ACTION_CLASSES`/валидатор или уходят с ADR-0149; `action_server/` + сайдкар `voice-action-server` (`docker-compose.yaml:448-460`, ADR-0011); правило промпта «rap ≥ 6 speak_text» (`master_prompt_compact.txt:445-456`) — `grep -rn "pregenerate\|_pending_music_cleanup\|_pending_speech_queue\|music_cleanup\|track_start_guard\|turn_speech_gate" src --include=*.py` = 0; `ls src/rob_box_voice/rob_box_voice/action_server` = нет; `grep -n voice-action-server docker/vision/docker-compose.yaml` = 0 | робот (`music_engine: v2` на стенде): 10 рэпов × 6 сегментов, запись `jack_rec`, `audit_wav.py` — смещение старта сегмента от такта (A19); сценарии X3 (A20), X5 (A21), #993 (A22) по 10 прогонов; `perform.no_backing` при подложенном отказе плеера | A19–A22, Х1–Х6 |
| **PR-8** | `dialog_node` v2 за флагом: хост, `turn_log`, окно из озвученного, `TURN_DEADLINE_S`; e2e-БД текста по `e2e_db_path` вне `/data` (В8); launch по флагу | при `v2`: `dialogue_node` не стартует; старые топики не создаются (`ros2 topic list`, raw) | робот `v2`: акты 1–3 марафона; `turn_metrics.py`; **замер A1 с рефлексами → уточнение порога** | A1, A2, A4, A13, A14 (по провайдерам → выбор дефолта) |
| **PR-9** | `interrupt.py`: отмена по эпохе до HTTP-стрима, классы прерываемости, «стой» по снимку | D14, ветка `barge_in_policy=classify`, `/voice/dialogue/barge_in_policy` — grep = 0 | 20 перебиваний; 0 тулов старой эпохи | A12 |
| **PR-10** | сцены: `Silence(TTL 600, set_by)`, `Persona`, `DJ` по событиям музыки (**ждёт #3312 PR-5**: `nearly_finished/idle`), `OperatorHold`; `DialogControl.srv`; `reset()` | `DialogueStateMachine`, `_pause_reason/_paused_at_ms`, `dj_set_boundary.py`, `/dialogue/control*` — grep = 0 | марафон 12 актов: акт 8 — тишина кончается через 10 мин; голос акта 4 не доживает | A9, A10 |
| **PR-11** | память: `remember/memory_search` на одной таблице с `person_id`; снимок вместо маркеров; ТАРС-журнал с `ts/ttl` | R13 (`_HISTORY_MARKER_RE`, `_SPEAKER_TAG_RE`, `_META_PREFIX_RE`), `discard_last_reply`, `URGENT_BACKLOG`/`[Spkr:` — grep = 0; `voice_memory.db` = 0 | «запомни/что помнишь» ×10, два собеседника | A3 (память), A13 |
| **PR-12a** | **телеграм-агент**: `AgentSpec(telegram)`, сессия на `chat_id`, своя LLM из yaml (живые `llm_*` вместо мёртвых), тулы-команды с текстовыми фразами результата, `DialogOutput` | писатели telegram в `/voice/stt/result`, `/voice/dialogue/response`, `/voice/sound/stop`; `[TG:` и `tg_chat_id` в `dialogue_node.py` — grep = 0 | Telegram: 10 команд, raw логи; «Клод …» канал (память) | A6 (−3), A3 (телеграм) |
| **PR-12b** | `say_aloud` как `action.speak_aloud` + правило совместного доступа (О2 — по решению Шифу); сценарий «сочини историю и расскажи» | писатель telegram в `/voice/tts/request` — grep = 0 | сценарий: история в чате + озвучка + подтверждение по `SpeechEvent finished` (raw) | A3, A6 (−1) |
| **PR-13** | ТАРС на `rob_box_dialog`/`AgentSpec(operator)`; `say` как `action.speak_aloud` с событием; `slice_policy` без призраков; `_utterance_fallback.py` удалён | `sup/_utterance_fallback.py`, призраки `slice_policy.yaml:137-143`, собственная история/гарды супервизора — grep = 0 | 20 операторских команд; `honesty_audit` на журнале (#3297) | A3 (ТАРС), A6 |
| **PR-14** | приёмка §14 целиком с raw → решение Шифу → `dialog_engine: v2` по умолчанию, дефолтный LLM по A14 | — | все A1–A17, run_id | — |
| **PR-15…17** | удаление старого пути (**после #3312 PR-13…15** для файлов, которые делят с музыкой): (а) `dialogue_node.py`, `dialogue_guards.py` (диалоговая часть), `turn.py`, `turn_speech*.py`, `speak_helpers.py` регексы, адаптер совместимости, старые String-топики; (б) промпты → генерация из `knowledge` (`RULE #` = 0), скиллы → схемы; (в) тесты на номера issue/текст промпта, baseline-ы | `wc -l dialogue_node.py` → файла нет; `Bug [A-F]` в диалоговых файлах = 0; `#NNNN` в `src/rob_box_dialog` ≤ 20; `RULE #` = 0 | pytest -v; CI run_id; рантайм-аудит без `multiple_writers` | A4, A6 = 0, A8 |

**Первые PR можно начать без решений**: PR-0, PR-1, PR-1b, PR-2 не меняют поведение на роботе. PR-1c даёт решение по каналу речи до того, как он понадобится (PR-7). Открытый вопрос О2 нужен только к PR-12b.

### 11.1 Зависимости от эпика #3312 (ADR-0149)

| PR этого эпика | Ждёт из #3312 | Почему |
|---|---|---|
| PR-7 | PR-6 (`tools_v2.request_music/dj_set`, фраза по `started`) | медиа-фразы из событий; до него — `MediaRouter` v1 с текстами из `phrases` |
| PR-7c (старт на такте, подложка) | PR-4 (`PlayerOwner.started` со снимком фазы — уже влит `96ff9e38a`), PR-6 (`request_music(intent=backing, min_form_beats)` — если поля нет, запрос к #3312), PR-5 (`nearly_finished` для продления под длину речи) | квантование речи по такту даёт только `PlayerOwner` v2 за `music_engine`; при v1 выступление идёт без квантования, A19 меряется только на стенде с v2 |
| PR-10 | PR-5 (`nearly_finished`, `idle`, `SetSession`) | кадр `DJ` живёт по событиям плеера |
| PR-14 | PR-12 (приёмка музыки, `music_engine: v2` дефолт) | марафон с DJ-актами на одном флаге |
| PR-15…17 | PR-13…15 (удаление `music_guard.py`, DJ-веток `dialogue_guards.py`, `dj_mode.py`) | общие файлы удаляет один владелец — #3312 |

Обратных зависимостей нет: #3312 не ждёт этот эпик. PR-3…PR-13 не трогают `tools/music.py`, `core/dj_mode.py`, `core/music_guard.py`.

---

## 12. Таблица костылей на удаление (группы из 03 §1, класс по ADR-0148, PR)

| Группа (03) | Что | Класс | Чем заменено | PR |
|---|---|---|---|---|
| R1, S1, P1(часть) | babble-гард и ретрай | (а) | команда: речь/действия структурно | PR-7, PR-15 |
| R2 | `is_planning_narration` → мьют | (б) | одна точка входа речи (победитель бейк-оффа); `content` вне канала не озвучивается | PR-7, PR-15 |
| R3, R4, R5, S4, S5, S7, P1 | `ACTION_CLAIM_RULES`, `_ACTION_VERBS_*`, `PHANTOM_*`, ретраи | (а) | фраза из события; речь при действиях не озвучивается (§4.1) | PR-7 |
| R6, S6, P3 | «не знаю мелодии», Bug F | (а) | `search.find` (ADR-0149) как `query` | PR-7, #3312 |
| R7, S2 | код Renardo в речи | (г) | ADR-0149 | #3312 PR-13 |
| R8, S3, P2 | `HALLUCINATED_MIDI_RE` | (б)→(г) | нет поля для нот | #3312 |
| R9, S9, P5 | эхо `<system>` + ретрай; отказ tts_node | (в)/(г) | ретрай удалить; отказ — `TEMP(ADR-0148` + счётчик | PR-7, PR-15 |
| R10, S8, P7 | вызов тула текстом ×3 слоя | (в) → одно место | `command.parse` — единственный парсер | PR-6, PR-15 |
| R11, P6 | done-маркеры ×5 | (б)+(г) | команда терминальна | PR-7 |
| R12 | `startswith(("[SYSTEM",…))` | (г) | ретраев с `[CRITICAL]` нет | PR-7 |
| R13, вопрос 28 | `_HISTORY_MARKER_RE` и др. | (а) | снимок структурой | PR-11 |
| R14 | снятие markdown ×2 | (в) → одно место | tts_node | PR-3 |
| R15, U7, P10 | выдуманная лирика после музыки | (б) | у `request_music` нет текста песни; команда терминальна | PR-7 |
| R16 | аргументы в тексте `speak_text` | (б)/(в) | парсер канала-победителя | PR-7 |
| U1 | `media_command_grammar` | образец | расширяется | PR-6 |
| U2 | `skill_router` 27 регексов | (а)/(г) | сцена + `tools_view` | PR-6 |
| U3, U4 | стоп ×5, громче ×4 | (г) | `knowledge` + `grammar` | PR-1, PR-6 |
| U5 | `MUSIC_GUARD_KEYWORDS`, `TOOL_REQUEST_PATTERNS` | (а) | Tier-1 исполняет сам | PR-7 |
| U6 | топ-трек на пустой ответ | (в)→(г) | честный `Ask` | PR-7 |
| U8, U9 | «новая сессия», да/нет | (а) | Tier-1 | PR-6 |
| S10, S11 | Bug C/B | (а)+(г) | роутер + ADR-0149 | PR-7 (не вызывается), #3312 |
| S12 | `tool_skipped` | (а) | `query`-тулы по команде | PR-7 |
| S13 | `truncated_args` | (в)→(б) | ≤ 12 схем | PR-6 |
| S14 | silent_response | (в) | `command.parse`; пусто → `Ask` | PR-7 |
| P4, P11–P14 | правила промпта ↔ код | (а)/(б)/(г) | промпт из `knowledge`; клампы в схеме | PR-15(б) |
| P8, P9 | DJ-правила | (б)/(а) | ADR-0149 | #3312 |
| 1.5 входные | `_MUSIC_STOP_OVERRIDES`, `_should_force_dj_off…`, `_PREFIXES` | (г)/(а)/(б) | грамматика; статус STT полем | PR-6, PR-2 |
| 1.5 подставные | «Принял.», «растерялся», «попробую», «Готово, играю.» | (в)/(а)/(г) | `phrases` по событию | PR-7, PR-8 |
| 1.6 флаги | `skill_tool_narrowing`, `barge_in_policy`, `e2e_*` в prod-ноде, `faq_mode_enabled` ×2, мёртвые параметры, два бюджета; `llm_*` telegram | (б)/(г) | срез по возможности; `DialogControl.reset`; удалить | PR-6, PR-9, PR-8, PR-4, PR-12a |
| T1 | `IGNORE_STOP_MS` | (а) | отмена по эпохе | PR-3 |
| T2–T4 | грейс эхо, sleep USB, Silero wait | (в) | `TEMP(ADR-0148` + замер | PR-0 |
| T5 | таймеры приветствия | (а) | `ProviderState(tts).ok` как событие | PR-8 |
| T6 | `TurnSpeechGate`, фальшивый батч | (г) | решение до генерации | PR-8, PR-15 |
| T7–T9, T11 | DJ-таймеры | (а)+(г) | ADR-0149 | #3312 |
| T10 | `MEDIA_ACTION_BACKING_S` | (г) | — | PR-6 |
| D1 | music v1/v2 | (в) | #3312 | — |
| D2 | `_utterance_fallback.py` | (г) | `rob_box_core.utterance` | PR-13 |
| D3, D4 | зеркала `TurnState`, `NUDGE` | (г) | — | PR-15 |
| D5 | два владельца DJ-флага | (а) | владелец — плеер | PR-10, #3312 |
| D6 | два владельца голоса | (а) | `VoiceState` + `voice_changed` | PR-4 |
| D7, D8 | списки ×18, знание ×N | (б)/(г) | `knowledge` | PR-1, PR-7, #3312 |
| D9 | две launch-конфигурации | (г) | одна | PR-8 |
| D10–D15 | мёртвое | (г) | — | PR-0, PR-9 |
| D16 | мёртвые топики | (а)/(г) | §2.3 | PR-3, PR-8 |
| D17 | призраки `slice_policy` | (г) | — | PR-13 |
| новое (V26) | экспрессия: LLM-аргумент `animation`, auto-switch по TTS, «boop» из stt | (а) | рефлексы + enum в команде | PR-3b |
| новое (V14, V27) | `[TG:chat_id]`, `tg_chat_id`, мёртвые `llm_*` | (а)/(г) | телеграм-агент | PR-12a |
| 07 §1.6 | `TaskScheduler` FIFO с `{"status":"queued"}`, `wait_until_idle` (опрос 20 мс), `task_delta`/`TaskDeltaTool`/`skills/scheduler.txt`, `[ACTIVE TASKS]/[SEGMENT PLAN]` с `eta=?`, мёртвые хуки `cancel/set_llm_continue_hook/notify_event/set_eta_provider/set_group_boundary/set_frozen_touch_hook` | (а)/(г) | `plan/` + события владельцев | PR-7b |
| 07 §1.4 | `barge_in_policy` (`replace`/`classify`), `quick_decide`, `_pending_user_messages`, STOP из stt_node на вейк | (а)/(г) | `compose()` + Tier-1 | PR-7b, PR-5 |
| 07 §1.6 | `scheduler_shadow.py` (PoC без вызывающих), `register_tts`, `deferred_call_ids`, `action_server/` + сайдкар `voice-action-server` (ADR-0011) | (г) | — | PR-3, PR-7b, PR-7c |
| 07 §1.5 | поле `pregenerate` чанка и `scheduler/pregen/{decision,quality,speculative_executor,pre_gen}` (подключено, данных нет) | (г) → одна реализация | пред-синтез сегментов `commit=false` (§5.6.3) | PR-7c |
| 07 §1.7 | пять гардов исполнителя: #2859 «один трек за ход», #2878/#3221 лимит DJ-реплик, #2913 гейт речи до регистрации, #3246/#3247 отказы DJ_AUTO (`track_start_guard.py`, `tool_executor.py:128`) | (б) → поля `ACTION_CLASSES`/валидатор; (г) с ADR-0149 | правила класса, не исполнителя | PR-7c, #3312 |
| 07 §1.7 | cleanup музыки по `batch_complete`, `_active_batches`, прелюдия, `_track_mode_music_active`, `/mcp/music_cleanup` | (а) | время жизни подложки = выступление (`Parallel`) | PR-7c |
| 07 §1.2 | второй планировщик в `tts_node`: `priority`-вклинивание, `_pending_speech_queue` чужих батчей (#2553), буфер «STOP после синтеза» | (г) | порядок задаёт `plan/`, tts_node исполняет `at/boundary` | PR-7c |
| 07 §1.2 | `play_sound` «Звук запущен» без события; `sound_node` молчаливый `return` | (а) честность | `SoundEvent started/skipped{reason}` | PR-7b |
| 07 §1.5 | два списка «голосовых» слов (`_VOCAL_REQUEST_KEYWORDS`, `MUSIC_GUARD_VOCAL_KEYWORDS`) | (г) | `Performance.backing` — поле команды, слов не нужно | PR-7c |

Метрика прогресса в каждом PR: `re.compile` в `dialogue_guards.py` (34 → 0), `_retry_used` (10 → 0), `Bug [A-F]` в диалоговых файлах (170 → 0), `#NNNN` в `src/rob_box_dialog` (≤ 20), `TEMP(ADR-0148` в диалоге (0 → ≤ 5 → 0).

---

## 13. Какие ADR замещаются; что поправить в CONTEXT.md

| ADR | Статус сейчас | Решение |
|---|---|---|
| **0084** TurnGuards | accepted; удалён #3327 | **Superseded** |
| **0021** декомпозиция (R1–R5) | proposed с 18.08; узел ×2.4 | **Superseded**: замена, не декомпозиция; R5 отменяется (705 ссылок); 0021-r1 остаётся инструментом |
| **0066** pause/resume | — | §2–§3 superseded: `DialogControl.srv` с `set_by`, TTL |
| **0143** ретраи как норма | принято | Superseded: режим команд по возможностям (§4.5) |
| **0129-dj** штамп персоны | proposed | Superseded §2: кадр `Persona` |
| **0140** остаток | — | superseded ADR-0149 + §9.1 |
| **0065** вейк SSoT в коде | accepted | amended: SSoT — `rob_box_dialog.knowledge`; **принцип «вейк на каждой фразе» подтверждён** (В2) |
| **0001** §Telegram-харнес, W7-решение «телеграм — мост без LLM» (`07dfc28aa`) | — | superseded §9.2: телеграм — агент со своей LLM на общем движке |
| **0102/0103** «Повод» | proposed | остаются внутри `address.py`/`respond.py` |
| **0131** `utterance_id` | accepted | остаётся; типизируется |
| **0083** `build_agent` | proposed | остаётся: `AgentSpec(personality|operator|telegram)` |
| **0037** память | proposed | частично superseded §7.4 |
| **0055/0128** одна БД / e2e-границы | — | 0055 подтверждается; e2e-БД текста вне `/data` (В8) |
| **0114** искажения «ТАРС» по логам | proposed | подтверждается и расширяется на вейк Личности (не придумывать варианты) |
| **0011** action protocol (ActionServer HTTP-сайдкар + PASTE) | Accepted post-factum; сайдкар `voice-action-server` — заглушка без клиентов (V35, 07 §1.6) | **Superseded** §2 (транспорт HTTP-сайдкар, отдельный процесс, plugin registry) и §PASTE (shadow queue). **Берётся**: семантика goal `accepted/rejected` синхронно → `feedback` → `result`, `cancel` по id (ADR-0002 §3.4, повторено ROS2 actions) — как форма `PlanEvent`/`ActionResult`; идея `prefetch без побочных эффектов + commit ровно одного` — как пред-синтез `commit=false` (§5.6.3). Реализация в `rob_box_dialog.plan`, не отдельный процесс |
| **0002** ROS2 Action как транспорт планировщика | Proposed, перекрыт 0011 | Superseded вместе с 0011: транспорт — типизированные топики владельцев (§10), экшены ROS не вводятся |
| **0033** MERGE не распространяется на музыкальный канал | Accepted | Superseded: границы одновременности — таблица `COMPOSITION` (§5.6.2); «подложка против трека» — О9 |
| **0056/0092** `pregenerate` контракт | Proposed/Accepted; поле никто не публикует (V32) | Superseded §pregenerate: пред-генерация — `SpeechRequest commit=false` + `SpeechEvent synthesized` (§5.6.3); #2003 закрывается замером A19/A2 в PR-7c |
| **0086** EventBus/ReflexLayer удалить | Accepted | подтверждается; внешние события (батарея) — П12 07, отдельный виток после PR-7c |
| `SCHEDULER_DESIGN.md`, `W7_INTEGRATION_PLAN.md`, `docs/plans/2026-08-30-scheduler-integration-review.md` | design-документы #968 | помечаются «заменены ADR-0150 §5.6»; их решения (каналы, MERGE PENDING, события-источники, естественные границы) перенесены или отвергнуты явно в §5.6 |
| **0148**, **0149** | proposed | родители |

**CONTEXT.md:** «Планировщик» (`:99-104`) и «Канал» (`:106-107`) переписать: планировщик — `rob_box_dialog.plan`, владеет расписанием **ресурсов** (голос, лицо, кольцо, SFX, музыка, движение), статус задачи — событие владельца ресурса, политики APPEND/MERGE/REPLACE/REJECT по таблице; «канал» → «ресурс»; добавить **Выступление** (песня/рэп/сказка: сегменты + подложка + экспрессия, одно время жизни, старт на такте) и **Граница** (предложение / такт / сразу). «Ход» (`:109-116`) — без `TurnGuards`: «одна попытка понять и исполнить адресованную фразу: команда LLM {речь, вопрос, ≤ 3 действий, экспрессия}, валидатор, фраза из результата; владеет эпохой и дедлайном; ретраев не имеет». Добавить **Сессия**, **Сцена/кадр**, **Команда**, **Результат действия**, **Владелец речи**, **Экспрессия/рефлекс**, **Агент** (личность, ТАРС, телеграм — три спецификации одного движка), **Возможность**. «Срез» — функция; «AgentCore» — LLM-клиент; «Пауза» — кадр `OperatorHold`; «Вейк-слово» — источник `knowledge`, **единственный способ адресации голосом**.

---

## 14. Приёмка числами

Прогоны: (П1) ночной марафон 12 актов (`scripts/e2e/run_night_marathon.sh`), `tts_provider=minimax`, e2e-БД по ADR-0128 вне `/data`; (П2) эталонный набор честности ≥ 60 фраз, `honesty_audit.py`; (П3) Telegram: 10 команд + «сочини историю и расскажи»; (П4) ТАРС 20 команд; (П5) аудиты `G/L: Architecture Audit`; (П6) `turn_metrics.py`; (П7) `adapter_benchmark.py` по каждому провайдеру; (П8) бейк-офф §4.4. Пороги приняты Шифу 02.10 (В5), кроме отмеченного.

| # | Критерий | Порог | Сейчас (01.10) | Как мерить |
|---|---|---|---|---|
| A1 | **вызовов LLM** на адресованную фразу (невербальные действия не считаются) | p50 = 1, p100 ≤ 3; Tier-1 — 0. **Принято предварительно, уточнить после замера в PR-8** | до 27 возможных; 8 за ход (#1881) | `TurnEvent.llm_calls` (П1, П6) |
| A2 | STT → первый звук | Tier-1: p50 ≤ 1.0 с; LLM-ход: p50 ≤ 4 с, p95 ≤ 8 с | 15–50 с в инцидентах | `🎤 STT` → `SpeechEvent.started` |
| A3 | выдуманных успехов | 0/60 на наборе; 0 за марафон; 0 в Telegram/ТАРС | #2755, #2780, #2949, #3266, #3297 | `honesty_audit.py` |
| A4 | размер хоста | `dialog_node` WMC ≤ 80, ≤ 40 методов, ≤ 600 строк; `dialogue_node.py` удалён | WMC 1 103, 210, 9 954 | radon, `class_budget`, `wc -l` |
| A5 | токены на вызов | p50 ≤ 12k, p100 ≤ 20k | ≈ 70k | `estimate_tokens` в `TurnEvent` |
| A6 | multi-writer топики диалога | 0 вне `writers_allowed` | 17 | рантайм-аудит (П5) |
| A7 | `ownership.yml` | все топики префиксов имеют `owner`; CI зелёный | пуст | `ownership_check.py` |
| A8 | заплатки | `re.compile` 34 → 0; `_retry_used` 10 → 0; `Bug [A-F]` 170 → 0; копии 5,4,3 → 1; `TEMP(ADR-0148` → 0 | проверено | grep в PR |
| A9 | тишина по TTL | «хватит» → речь через 10 мин; `resume` оператора не снимает | бессрочно | П1 акт 8 |
| A10 | утечки между актами | ≥ 11/12 зелёных; голос/персона после `reset` = дефолт | 1/12 | П1 |
| A11 | честная деградация | 1 фраза на смену, `health` ≤ 10 с, 0 молчаливых подмен | молча | сценарий PR-4 |
| A12 | перебивание | ≤ 300 мс; 0 действий старой эпохи; история = `text[:spoken_chars]` | хвост доигрывает | П1 + 20 перебиваний |
| A13 | служебный текст в речи | 0 вхождений маркеров в `SpeechEvent.started.text`; `say_truncated` ≤ 2 % | ×193/час (#2558) | grep по логу TTS |
| A14 | пустой/невалидный ответ **по каждому провайдеру** | 0 «Принял.»; `invalid` ≤ 10 %; 0 ретраев; **дефолтный провайдер = лучший по A14** | 19 пустых/50 мин (#1253) | `TurnEvent.status=invalid`, П7 |
| A15 | адресация | 0 пробуждений на корпусе «работает/робота-диджея»; 0 ответов на фразы без вейка | ложные wake (#1292) | unit-корпус + П1 |
| A16 | двойная обработка | «робот, хватит» → ровно 1 `SpeechRequest` | 2 (вывод из кода) | П1 |
| A17 | **живость без LLM** | каждый адресованный ход: рефлекс `addressed` ≤ 300 мс и терминальный `done|failed`; `ExpressionEvent skipped` ≤ 5 %; 0 вызовов LLM ради экспрессии | анимация — аргумент `speak_text` | лог `ExpressionEvent` (П1) |
| A18 | канал речи (бейк-офф) | `delivered = 100 %`, `leak = double = phantom = 0` на всех адаптерах | оба канала теряли речь (§18.1) | П8 |
| A19 | **старт сегмента выступления на такте** (при `music_engine: v2`) | смещение начала каждого сегмента от ближайшей доли такта подложки: p50 ≤ 40 мс, p95 ≤ 80 мс (≈ 1/16 такта при 130 BPM = 115 мс); **порог предварительный — утверждает Шифу (О11)**; `PlanEvent.synced=true` у 100 % сегментов | привязки к такту нет (07 §1.5 п.3) | запись `jack_rec` цифрового выхода, `audit_wav.py` (сетка 16-х) + лог `SpeechEvent started` ↔ `started{beat_at}`; 10 рэпов × 6 сегментов (П9 — прогон PR-7c) |
| A20 | **«дочитай и вплети» (X3)** | текущий ACTIVE-сегмент дочитан до конца в 10/10 прогонов (`spoken_chars == text_len`); PENDING заменены (`segment_replaced`); подложка не прерывалась (0 `music/event idle` до `block_end`); 0 новых выступлений с нуля | REPLACE всегда, «новая история с нуля» (29.08) | П9 сценарий X3, лог `PlanEvent` + `SpeechEvent` |
| A21 | **REPLACE на границе (X5)** | после Tier-1 «хватит» речь кончается на границе предложения ≤ 3 с; outro подложки ≤ 1 фраза; «стой» — тишина ≤ 300 мс | мгновенный STOP посреди слова | П9 сценарий X5 + 20 «стой» |
| A22 | **«добавь музыку» во время выступления (#993)** | речь не обрывается (0 `cancelled`); подложка `started` на границе такта; следующий сегмент `synced=true`; 10/10 | «не реагирует» (#993) / REPLACE | П9 сценарий #993 |
| A23 | **одновременность речи и экспрессии** | в команде с `speech` + `expression` + `play_sound`: `ExpressionEvent started` и `SpeechEvent started` в пределах 150 мс друг от друга; `SoundEvent started` или честный `skipped{reason}` — 0 молчаливых пропусков; `motion` во время выступления не рвёт речь | ручная анимация сбрасывается на первом `ready`; звук молча пропускается (V33) | П1 + П9, лог событий |

Необходимое условие сверх чисел — прослушивание товарищем Шифу живого диалога (10 фраз набора) и вердикт «не врёт, не молчит, не читает мусор, живой», плюс один рэп под бит с «и про енота» посреди.

### 14.1 Трассировка хотелок Х1–Х15 (07 §3)

| # | Хотелка | Статус сейчас (07) | Чем закрывается | PR | Порог |
|---|---|---|---|---|---|
| Х1 | рэп под бит, бит гаснет после речи | частично, на заплатках | `action.perform` + `Parallel(success_count=1)`: outro после последнего сегмента (§5.6.3) | PR-7c | A22 (0 обрывов), `block_end` → `music idle` ≤ 1 фраза |
| Х2 | начинать с начала такта, аранжировка под длину | не сделано | пред-синтез → длительности → форма подложки ≥ Σ; `sync.next_bar` | PR-7c (нужен #3312 PR-4/6) | A19 |
| Х3 | догрузить во время рэпа, дочитать и вплести | сломано (REPLACE) | MERGE в PENDING (`edit_pending`), вейк ≠ STOP (§5.6.4, §6.2) | PR-7b (политика), PR-7c (сегменты) | A20 |
| Х4 | «а потом спой колыбельную» | не сделано | `when=after_current` → APPEND, фраза `queued{after}` | PR-7b | `PlanEvent queued` + старт после `block_end`, 10/10 |
| Х5 | «хватит, анекдот» — допеть фразу | не сделано | Tier-1 → REPLACE на границе предложения (`boundary=sentence`) | PR-3 (`boundary`), PR-7b | A21 |
| Х6 | `stop_music` не режет речь | на заплатке cleanup | время жизни подложки у выступления; `stop_music` как явная команда — `compose()` против занятого голоса → APPEND/REPLACE на границе | PR-7c | A22, `grep music_cleanup` = 0 |
| Х7 | анимация одновременно с речью | частично | `ExpressionRequest` параллельно речи, эмоция поверх рефлекса `speaking`, не сбрасывается `ready` | PR-3b | A23 (≤ 150 мс) |
| Х8 | кольцо живёт со стадиями | не сделано | рефлексы `REFLEXES` (§5.5) | PR-3b | A17 |
| Х9 | SFX поверх речи, честно | работает с враньём | `SoundEvent started/skipped`, `play_sound` по событию | PR-7b | A23 (0 молчаливых пропусков), A3 |
| Х10 | батарея/события вплетаются в песню | не сделано, шина удалена | П12 07 — внешние события как APPEND-сегмент на границе; **отдельный виток после PR-7c** | — (следующий ADR/эпик) | — |
| Х11 | «стой» гасит всё; «направо» во время песни — параллельно | «стой» работает; «направо» рвёт песню | «стой» — Tier-1 REPLACE сразу; `motion` ✅ в `COMPOSITION` | PR-7b | A21 («стой» ≤ 300 мс), A23 (motion не рвёт речь) |
| Х12 | LLM видит, что звучит, очередь, ETA | номинально, `eta=?` | блок контекста из `PlanEvent prediction` и `ResourceState` (§5.6.1) | PR-7c | блок непустой во время выступления 100 %; `eta_ms` заполнен |
| Х13 | не молчать, пока LLM думает над длинным | не сделано | пред-синтез сегмента 1 стартует, пока LLM отдаёт остальные? — **нет**: команда приходит целиком; живость — рефлекс `thinking` (§5.5); первый звук ≤ A2 | PR-3b, PR-8 | A2, A17 |
| Х14 | операторская врезка после чанка без обрыва | в коде, не проверено | `SpeechRequest(agent=operator, priority)` → `compose()` реплика против реплики = APPEND на границе предложения (не внутри tts_node) | PR-7b, PR-13 | врезка стартует на границе предложения ≤ 3 с, 10/10 |
| Х15 | DJ-фразы не обрывают друг друга | сделано в tts_node (#2553) | то же правило APPEND в `plan/`; `_pending_speech_queue` удаляется | PR-7c | 0 `cancelled` между DJ-фразами за сет |

---

## 15. Что сохранить из старого

| Что | Где | Куда |
|---|---|---|
| Допуск реплики, 12 шагов | `core/stt_admission.py:449-840` | как есть; шаги → `address.py`/`grammar.py` |
| Грамматика медиа-команд и роутер | `core/media_command_grammar.py`, `media_router.py` | расширяется |
| `Utterance`, `Sink` | `rob_box_core/utterance.py:55-220` | нагрузка `SpeechRequest` |
| `utterance_id`, join | `core/utterance_id.py`, `utterance_speaker.py`, `utterance_binding.py` | как есть |
| «Повод» | `core/occasion.py:40-228` | внутри `address.py`/`respond.py` |
| Каталог тулов и генератор | `rob_box_core/tool_catalog.py`, `tools/gen_tool_catalog.py` | источник схем |
| **Интерфейс провайдера** | `rob_box_llm/provider.py:226`, `ProviderCapabilities :78` | расширяется (§4.5) |
| LLM-клиент, здоровье, ретраи транспорта, `_order_tool_calls` | `agent_core.py` (часть), `providers/*`, `health.py` | клиент под `degrade.py` |
| Швы идентичности/встречи | `harness/identity`, `harness/encounter` | читатель — `Session.speaker` |
| Эпоха сессии | `core/session_epoch.py:56-114` | `Session.epoch` |
| Арбитр floor | `sup/core/locks.py`, `fsm.py` | без изменений |
| STT-каскад | `stt_node.py:1446`, `stt_fallback.py` | + `ProviderState(stt)` |
| Оценка длительности речи до синтеза | `scheduler/pregen/estimator.py` | библиотечная функция для ETA (`plan/sync.py`); остальной `pregen/*` удаляется (§5.6.3) |
| Снимок фазы, `started` из потока клока | `engine/player_owner.py:78-89`, `core/clock_phase.py` (ADR-0149) | источник `beat_at` для `sync.next_bar` — без изменений |
| Семантика goal/feedback/result/cancel и `commit` пред-генерации | ADR-0011/0002 (контракт), `action_server/paste.py` (идея) | форма `PlanEvent`/`SpeechRequest.commit`; код сайдкара не переносится |
| Квантизация и поиск сетки 16-х в записи | `audit_wav.py` (память «Аудит записей») | замер A19 |
| Анимации и исполнители | `rob_box_animations`, `led_node.py`, `sound_node.py`, `KNOWN_ANIMATIONS` (`animations.py:36`) | исполнители `ExpressionRequest`; enum — в `knowledge` |
| Телеграм: auth, камеры, карточки, floor-клиент, STT голосовых | `rob_box_telegram/{auth,camera_*,avatar_card,face_card,supervisor_client,voice_processor}.py` | как есть; меняется только мозг (агент) и выход |
| Марафон и инъекция | `scripts/e2e/*` | П1 |

---

## 16. Риски и откат

| Риск | Смягчение | Откат |
|---|---|---|
| Ни один канал речи не даёт 100 % на всех адаптерах | критерий §4.4 второго порядка: структурное закрытие недостачи; amendment с причиной | — |
| JSON-режим у провайдера без `tool_choice` даёт `invalid` > 10 % | A14 по провайдерам → дефолт — лучший; провайдер-агностик позволяет сменить строкой конфига | `dialog_engine: v1` |
| Задержка отмены речи через `interrupt_request` > 300 мс | замер в PR-9; план Б — прямая отмена по эпохе из latched сессии | то же |
| Рефлексы «мельтешат» (анимация на каждую стадию) | `REFLEXES` — данные, `ttl_ms` и минимальная длительность; слепая оценка Шифу живости | таблица рефлексов правится без кода |
| Терминальность и неозвученный `speech` при действиях делают ответ сухим | шаблоны с вариантами + экспрессия из команды; флаг `post_action_comment` (В6, выкл) | — |
| Телеграм-агент озвучивает в пользовательскую тишину | правило §9.2.3 по О2; событие `rejected{silenced}` честно | — |
| Два контракта речи в переходный период | один адаптер, удаляется PR-15(а) | — |
| Параллельный эпик #3312 | зависимости §11.1; общие файлы удаляет #3312 | — |
| `develop` force-push; гарды красные после мержа соседа (память) | свежий `origin/develop`, локальные гарды перед пушем | — |
| Директива о турнах | текст — только в e2e-БД вне `/data` при `e2e_mode` | — |
| `at`-старт в tts_node не держит ±40 мс (планировщик Python, пул синтеза, USB-аудио `sleep(0.1)` T3) | пред-синтез убирает синтез из критического пути; замер латентности вывода в PR-3; порог A19 предварительный (О11); план Б — `at` исполняет sound-сервер (jack) по таймстампу, tts_node только отдаёт PCM | выступление без квантования (`synced=false`, честно в `PlanEvent`) |
| `request_music(intent=backing, min_form_beats)` не появится в #3312 PR-6 в нужной форме | запрос к эпику #3312 заранее (при принятии этого ADR); до него подложка — любой трек v2, продление по `nearly_finished` | рэп без подложки с шаблоном `perform.no_backing` |
| Удаление `scheduler/` ломает `pregen` и тесты tts_node | PR-7b/7c удаляют вместе с тестами и baseline `cc_budget`/`class_budget`; `estimator.py` переносится до удаления | `dialog_engine: v1` не помогает — планировщик общий; откат — revert PR |
| MERGE в PENDING: LLM отдаёт «новые куплеты» не с того места (повтор уже спетого) | блок контекста содержит DONE/ACTIVE-сегменты дословно и номер первого PENDING; схема `edit_pending` требует `from_segment` ≥ ACTIVE+1 | валидатор отклоняет, выступление продолжается без правки, фраза `perform.edit_rejected` |

---

## 17. Альтернативы

| Вариант | Суть | + | − | Вердикт |
|---|---|---|---|---|
| **А. Декомпозиция `DialogueNode` по ADR-0021** | выносить методы, держать гарды | без флага | узел ×2.4 с 18.08; К1, К2, К4, К6 не закрываются | **отклонено** |
| **Б. ReAct-агент с лучшей моделью и `tool_choice`** | заменить провайдера, оставить свободный ответ | мало кода | один провайдер = прибитый гвоздь (против В4); фраза об успехе у LLM | **отклонено** |
| **В. Команда из закрытого перечня + код исполняет + фраза из события** (этот ADR) | Rasa CALM + HA responses + LiveKit `say/StopResponse` + SayCan «can» | закрывает К1–К6; провайдер-агностик | речь после действий шаблонная; новый пакет и сообщения | **выбран** |
| **Г. Realtime речь-в-речь** | убрать STT/TTS | барж-ин «из коробки» | русский голос, персоны, музыка, Pi; нет контроля действий | **отложено**; паттерны `truncate`/`create_response=false` взяты |
| **Д. Pipecat/LiveKit как рантайм** | заменить ROS-контур | готовые turn-стратегии | второй рантайм рядом с ROS2/Zenoh | **отклонено**; берём паттерны |
| **Е. Отдельный контейнер `dialog_node`** | изоляция падений | — | ещё одна граница сети | **отложено** |
| **Ж. Окно адресации без вейка** (ревизия 1 В2(б)/(в): N с после ответа робота или для подтверждённого собеседника; OVOS converse, HA `continue_conversation`) | отвечать на «громче» без «Робби» | меньше пропусков из-за неуслышанного вейка (память `act2-fails-on-robbi-wake-miss`) | **Шифу 02.10: «пробовали без Робби — он постоянно лез во все разговоры и мешал, когда несколько человек в помещении; выключали, пробовали забороть, вернули».** Доказательство в истории: `0326d7e9f` (31.07) — W5 сузил вейк-гейт до IDLE, в диалоге робот прерывал сам себя эхом → гейт возвращён «безусловно: только прямое обращение может начать или прервать диалог»; `9ca7fb295` (21.02) — `wake_words=[]` как bypass блокировал всё; 20.08 (`e0b20fa14`, `7e8d8c208`) — фразы без вейка **накапливаются**, а не отвечаются (аккумулятор); #1668 (26.08) — фоновый голос в комнате непрерывно заполняет бэклог, 16 e2e-фейлов подряд: любое окно без вейка в такой комнате ловило бы чужую речь; #1195 (13.08) — вейк-гейт снят **только** для текста из чата, для микрофона оставлен намеренно («защита от фоновой речи»). Карточки с формулировкой «лез во все разговоры» в трекере я не нашёл (§19) — свидетельство Шифу + коммиты | **отвергнуто**; флаг не закладывается. Пропуски вейка лечатся одной таблицей, границами слов и искажениями по логам (ADR-0114), A15 |
| **З. Телеграм как канал ввода в голосовой диалог** (ревизия 1 §9.2; текущее состояние после W7) | один мозг на всё | нет второй LLM | «телеграм сейчас фигово работает» (Шифу); чат делит историю/персону/тишину с голосом, `[TG]`-маркеры в тексте (К5), 5 писателей в голосовые топики; в Feb–Jul своя LLM с `/say` работала (`3c91ba155`) | **отвергнуто** в пользу агента §9.2 |
| **И. Выбрать канал речи в ADR** (ревизия 1 рекомендовала поле `Say`) | решить сейчас | нет бейк-оффа | история показывает потери речи у обоих каналов в зависимости от провайдера (§4.4); Шифу: «критерий — что даст 100 %» | **отвергнуто**; бейк-офф PR-1c |
| **К. Починить существующий `TaskScheduler`** (подключить `register_tts`/`tts/finished` в канал VOICE, завести хуки, включить `classify`) | довести #968 как задумано | код уже написан и покрыт тестами (07 §1.6) | 13 модулей, подключены 4; каналы — по именам тулов, а не по ресурсам; MERGE через `task_delta` — решение у LLM (против ADR-0148); второй планировщик в tts_node остаётся; `queued` в LLM; история: 01.08→19.08→29.08→05.09 — каждое подключение добавляло заплатку, не убирало (07 §2) | **отвергнуто**; замена `plan/` с табличной политикой и статусом от владельца; переносится только `estimator.py` и идеи (каналы → ресурсы, MERGE PENDING) |
| **Л. Выступление как отдельная ROS-нода/action-сервер** (ADR-0011 сайдкар) | изоляция процесса | goal/feedback/cancel «из коробки» | ещё один процесс и граница с клоком Renardo и очередью tts_node; сайдкар два месяца был заглушкой без клиентов (V35) | **отвергнуто**; `plan/` в процессе агента, форма контракта — из ADR-0011 |
| **М. Выступление внутри tts_node** (владелец речи сам ведёт сегменты и подложку) | один процесс с аудио | минимальная латентность `at` | tts_node получает знание о музыке, экспрессии и командах LLM (сегодняшний второй планировщик — ровно этот путь, V34); WMC 694 уже над бюджетом | **не рекомендуется**, оставлено как вариант О6 для Шифу |

---

## 18. Решения владельца (товарищ Шифу, 02.10.2026)

| № | Вопрос (ревизия 1) | Ответ Шифу (цитата) | Как принято в ADR |
|---|---|---|---|
| В1 | Один канал речи: поле `Say` или тул `speak_text`? | «там было выковано потом и кровью чтобы можно было говорить из ллм если будет одна точка входа это круто просто нужно разобраться в начале что даст 100% результат это критерий выбора!!» | Одна точка входа речи — да (§2.1 п.2). Канал не выбирается заранее: бейк-офф PR-1c по критерию 100 % доставки без потерь/служебного текста/дублей на всех адаптерах (§4.4, A18); проигравший удаляется в PR-7. История «потом и кровью» — §18.1 |
| В2 | Окно адресации без вейка | «пробовали без робби но тогда бал проблема что он потоянно лез во все разговоры и мешл когда несколько человек в помещении по этому если ты гянешь историю мы выключали пробовали както забороть но потом вернули» | Окна нет, флага нет; вейк на каждой фразе (§2.1 п.12, §3 п.7, §6.2). Перенесено в §17 Ж как отвергнутое с доказательством из истории |
| В3 | TTL пользовательской тишины | «да 10 мин норм» | `Silence(set_by=user, ttl=600 с)` (§6.3, §7.2, A9) |
| В4 | LLM для ходов | «хотелось бы построить систему в которой нет разницы какая там ллм я бы выбирал какая будет лучше справляться дипсик сейчс вроде как лучше минимакса но думаю нало оставить настройку ЛЛМ чтоб можно было выбрать вариант потом чтоб можно было добавить еще провайдеров других ЛЛМ» | Провайдер-агностик — требование (§2.1 п.6, §4.5): один интерфейс адаптера, возможности в `ProviderCapabilities`, режим команд выбирает код, выбор в конфиге, контрактный тест, PR-1b. Дефолт — по A14 по всем провайдерам (PR-8/14); стартовый кандидат — DeepSeek |
| В5 | Пороги приёмки | «ок но только A1 интересно чтоб робот был живым тоесть он мог и глазами помикать и кольцом и звук какойнибудь восрпоизвести тоесть надо тут подумать число но мне нравится!» | Пороги приняты. A1 считает только вызовы LLM, «принято предварительно, уточнить после замера в PR-8»; живость — два слоя без LLM (§5.5), владельцы экспрессии и `ExpressionRequest` (§2.3, §10.1), PR-3b, A17 |
| В6 | Реплика после действия | «как рекомендуешь да» | (а) только шаблон; LLM-комментарий — флаг `post_action_comment: false` (§4.1) |
| В7 | ТАРС `say` | «да а» | (а) `say` остаётся как `action.speak_aloud` с `SpeechEvent` (§5.1, §9.3) |
| В8 | Текст турнов на диске | «можно» | (б) текст — только в отдельной e2e-БД вне `/data` на время приёмки; в проде на диск не пишется (§7.4, PR-8) |
| В9 | Удаление музыкальных гардов здесь или в #3312 | «там сейчас ведется работа и в этом эпике обещали все удалить почистить» | Удаление — у #3312; здесь только зависимости §11.1 и «v2 не вызывает» (§9.1) |
| В10 | Telegram | «телеграм сейчас фигово работает там должна быть своя ллм которая может слать команды роботу тоесть там должны быть обычные функции но и если я порпошу в телеги допустип сочини историю и расскажи то ллм для телеграма это сделает и озвучит роботом - это так работало уже както» | Телеграм — отдельный агент со своей LLM-сессией на общем движке (§9.2), тулы-команды через владельцев с событиями, `say_aloud` с подтверждением по `SpeechEvent`, сценарий-эталон «сочини историю и расскажи», PR-12a/12b; история — §18.1 |
| В11 | Персист сессии при рестарте | «по идее всеравно но мне кажется что лучше не надо чтоб переживало» | (а) рестарт = новая сессия (§2.1 п.4, §7.1) |

### 18.1 Что нашёл в истории (git log на этом worktree; тела issue — через `gh issue view`)

**В1 — канал речи `speak_text`:**
- `569f50194` (22.02) «speak_text returns TASK_COMPLETE to terminate agent loop»: цикл SDK кончается только текстом без tool_calls; промпт запрещал текст → `MaxTurnsExceeded`. Исток протокола done-маркера.
- `b41e4bb87` (22.02) «идиоматичный OpenAI Agents SDK»: история только из user-текста и **spoken**-текста — иначе модель копировала tool_calls из истории.
- `eaeb8e3a6` (28.02): LLM скопировала `[tools: …]` как текст и **не позвала `speak_text` → робот молчал**; добавлен auto-speak фолбэк для текста без тула.
- `3104a75e7` (01.03): `speak_text` объявлен MANDATORY; «LLM was sometimes outputting text directly as final_output instead of calling tool».
- `814d5d060` (24.02), `b0e28abd8` (31.07): остановка параллельного `speak_text` отменённого хода; анимация как аргумент `speak_text`.
- #988 (04.08) двойная озвучка; #1564/#2557 (08–09.2026) `done`/«Готово, играю» (`d9b57e919`); `2ad1bdd29` (10.09) DJ: только `speak_text`, свободный текст глушится.
- `c2c31c26d` #3269 (01.10): MiniMax 8 итераций клала реплику в `content` рядом с `set_voice`, `speak_text` не звал никто — сказка потеряна; заплатка `content_speech.py` превращает `content` в `speak_text`.
Вывод: оба канала теряли речь; какой надёжнее — зависит от провайдера → бейк-офф.

**В2 — адресация без вейка:**
- `9ca7fb295` (21.02) `wake_words=[]` как bypass; `9d5b515d9` — блокировал всё.
- `0326d7e9f` (31.07) «restore universal wake-word gate (barge-in regression)»: W5 (`18ff45ce3`, 28.07) сузил гейт до IDLE «presumably to allow natural interruption»; робот слышал своё эхо и прерывал себя; восстановлено «только прямое обращение может начать или прервать диалог».
- `e0b20fa14`, `979516475`, `7e8d8c208`, `b5b006666` (20.08): дизайн и реализация **аккумулятора** фраз без вейка — накапливать, не отвечать; `dialogue-mode-spec-2026-08-28.md` §1 закрепляет.
- #1668 (26.08, тело прочитано): фоновый голос в комнате непрерывно заполняет бэклог, 16 e2e-фейлов — комната робота шумная.
- #1195 (13.08, тело прочитано): вейк-гейт снят только для `[TG:…]`, «голосовой gate нужен для микрофона (защита от фоновой речи)».
- #1292 (`fc98e886e`, 15.08): подстрока «бот» ∈ «работает» → ложный вейк.
- **Не нашёл**: отдельной карточки с формулировкой «лез во все разговоры / мешал при нескольких людях» (поиск `gh api search` вернул ошибку, `gh issue list --search` — пусто); коммита, который прямо включал «окно без вейка» как фичу (W5-сужение — ближайшее).

**В10 — Telegram с собственной LLM:**
- `3c91ba155` (28.02) «add Telegram bot operator interface»: своя LLM (DeepSeek/Qwen) с MCP-тулами (28), **независимые LLM-сессии на пользователя, роль оператора**, `/say`, голосовые сообщения → STT → LLM или TTS. Это и есть «работало уже как-то».
- `6daa87f8e` (09.03) музыкальные команды; `d0442138f` (04.08) LLM телеграма → `deepseek-v4-flash` (yaml остался, код уже не читает — V27).
- `c6b1dd285`, `5e9470528` (27–28.07) `TelegramHarness` по ADR-0001; затем `07dfc28aa` W7 (28.07) **«remove all LLM dependencies from telegram_node»** → всё в `/voice/stt/result`; `b2ed94808` W8 «pure ROS2 bridge»; `88cecc91f` (28.07) эхо ответов удалено из-за asyncio-бага.
- #1195 (13.08, `def24baaa`): эхо возвращено через `asyncio.Queue`, `[TG:chat_id]`, пропуск вейк-гейта для чата, «оператор шепчет» в промпте Личности. С этого момента телеграм — вторая голова голосового диалога.
- Позже: `0143a5ac1` AV-23 рация, `0207abc9e` AV-10 клиент супервизора, `2f784ebe0` #3025 `/faces`.
- **Не нашёл**: карточки или коммита, фиксирующего поломку сценария «сочини историю и расскажи» после W7 — сценарий, судя по коду, перестал быть возможным по построению (нет LLM у телеграма, `/say` стал прямым TTS без LLM).

### 18.2 Что осталось открытым

- **О1. Состав эталонного набора и разметка** (≥ 60 фраз, ожидаемые действия/речь) — собирается в PR-0, утверждает Шифу; без него нет A3/A14/A18.
- **О2. Озвучка из Telegram в пользовательской тишине** (§9.2.3): (а) исполнить и сообщить в чат «робот в тишине ещё N мин, озвучил по вашей просьбе» — **рекомендую**, Telegram — операторский класс; (б) отказ с кнопкой «всё равно озвучить». Нужно к PR-12b.
- **О3. Пересмотр порога A1** после замера с рефлексами в PR-8 (В5: «надо тут подумать число») — Шифу решает по данным PR-8.
- **О4. Победитель бейк-оффа** (PR-1c) — фиксируется amendment'ом к этому ADR; критерий задан, выбор — по данным.
- **О5. Дефолтный провайдер** — по A14 в PR-8/14; стартовый кандидат DeepSeek.
- **О6. Кто владеет ресурсом «выступление»** (07 §9 п.5): (а) модуль `rob_box_dialog.plan.perform` в процессе агента поверх `SpeechEvent` и `music/event` — **рекомендую**: планировщик видит все ресурсы и команды LLM, tts_node остаётся исполнителем с бюджетом ADR-0145, латентность `at` закрывается пред-синтезом и измеренной задержкой вывода; (б) часть tts_node (вариант М §17) — ближе к аудио, но tts_node получает знание о музыке и командах, WMC 694 уже над лимитом. Нужно к PR-7c.
- **О7. Политика по умолчанию «речь во время речи»**: (а) APPEND — дочитать текущую реплику/сегмент, потом новую (07 П4 рекомендует) — **рекомендую**; (б) REPLACE на границе предложения. Tier-1 «стоп/хватит/стой» в любом случае REPLACE. Нужно к PR-7b.
- **О8. Ducking музыки под речь** (сегодня `grep duck` в голосовом стеке = 0, 07 §4.1): (а) `PlayerOwner` принимает `duck(depth_db, ms)` от планировщика на `SpeechEvent started/finished` — запрос к #3312; (б) не делать, полагаться на уровни ролей ADR-0149 §3.10. **Рекомендую (а)**, но только после приёмки музыки v2 (PR-12 #3312).
- **О9. Подложка выступления против играющего трека/сета**: (а) подложка вытесняет на границе фразы (ADR-0149), после выступления трек/сет не возвращается; (б) отказ с вопросом «сначала выключить трек?» (`Ask`); (в) выступление без подложки поверх играющего. **Рекомендую (а) для трека, (б) для DJ-сета** (сет — явно заказанная сцена). Нужно к PR-7c.
- **О10. Рэп без подложки**: если `backing` не стартовал — читать без бита с фразой `perform.no_backing` (**рекомендую**) или отказывать целиком (`required`)? Нужно к PR-7c.
- **О11. Порог A19** (смещение старта сегмента от такта): предлагаю p50 ≤ 40 мс / p95 ≤ 80 мс; альтернатива — «на слух» по записи (слепой A/B с квантованием и без, как ADR-0149 §7.3). Утверждает Шифу после первого замера латентности вывода в PR-3.
- **О12. Поле `pregenerate` (ADR-0056/0092)**: удалить вместе с неподключённым `pregen/*` (**рекомендую**, §5.6.3: одна реализация — пред-синтез сегментов) или оставить как есть до PR-7c.

---

## 19. Что не проверено

- Ничего не запускалось: все задержки, доли и частоты — из тел issue, памяти проекта и материалов 01–06; baseline — PR-0.
- Не проверял на роботе задержку маршрута `stt_node → interrupt_request → dialog_node → SpeechCancel → tts_node` (§6.2 допускает план Б).
- Не проверял долю валидного JSON/`tool_calls` у провайдеров без `tool_choice` — порог A14 стартовый; A18 решает бейк-офф.
- Не проверял тела ~280 issue; классификацию 01 §1 принимаю.
- Не нашёл карточек с формулировками Шифу по В2 («лез во все разговоры») и по поломке телеграм-сценария после W7 — опираюсь на свидетельство Шифу и коммиты §18.1; `gh api search` в этой сессии возвращал ошибку (не отличил лимит от пустой выдачи).
- Не проверял, что `animation_player_node`/`led_node` выдерживают частоту `ExpressionRequest` на каждую стадию — `ttl_ms` и минимальная длительность в `REFLEXES` стартовые.
- Паттерны опенсорса с пометкой «(поиск)» в 05 — не проверял.
- Не пересчитывал музыкальный путь (`next_transition_at`, `rob_box_music`) — интерфейс §9.1 из ADR-0149.
- Полевой состав `rob_box_dialog_msgs` (§10.1) — уточняется в PR-2 по фактическим JSON-полям v1.
- Параметры LLM телеграм-агента (провайдер, история на чат) — из мёртвого yaml (V27) как стартовые; живая настройка — PR-12a.
- По планировщику (07): не проверял на роботе, что канал `VOICE` пуст к приходу второй фразы и что деферрал `stop_music` холостой — вывод из кода (V28, `tool_executor.py:575-584`); не мерил латентность вывода звука и джиттер планировщика Python — порог A19 предварительный (О11); не проверял, что `tts_node` способен держать `at` без переноса воспроизведения в jack-сервер (риск §16); не проверял поведение `priority=operator` (#1996) и `_pending_speech_queue` (#2553) живьём — удаление в PR-7c опирается на то, что APPEND в `plan/` даёт то же поведение, это надо доказать сценарием Х14/Х15 до удаления; не читал первоисточники NAOqi `^wait`/`^run`, Nav2 `is_preempt_requested`, LiveKit `SpeechHandle` целиком — взято из 07 §6 с его пометками; не проверял, есть ли в #3312 PR-6 поле `min_form_beats` у `request_music` — зависимость §11.1 помечена как запрос.
