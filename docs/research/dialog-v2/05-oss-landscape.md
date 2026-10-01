# 05. Ландшафт опенсорс-диалоговых систем для голосовых ассистентов и роботов

> Дата: 01.10.2026. Исследование для dialog-v2 (rob_box).
> Метод: первоисточники (официальные доки, репозитории, исходники, issue-трекеры), собранные через WebFetch/WebSearch в этой сессии.
> Пометка **«не подтверждено»** означает, что утверждение не удалось сверить с первоисточником в этой сессии (по памяти или по выдаче поиска без открытия страницы).
> Пометка **«(поиск)»** означает, что факт взят из сниппета поисковой выдачи, а сама страница не открывалась.

## 0. Как читать

Для каждого проекта: архитектура в 5–10 строк, затем оси:

- **(а) решение**: кто выбирает действие, код или LLM;
- **(б) turn-taking и барж-ин**;
- **(в) владение состоянием и сессии**;
- **(г) подтверждение исполнения и формирование ответа**;
- **(д) фолбеки и деградация**;
- **(е) память и история**.

После осей идут блоки «перенять» и «избегать». В конце файла есть сравнительная таблица и 10–15 паттернов-кандидатов, у каждого указано, какую нашу боль он закрывает.

Наши боли (контекст): LLM выдумывает успех действий; решения в промпте вместо кода; облака деградируют молча; состояние утекает между сценами (тишина, голос, персона); у одного состояния несколько владельцев; регексы по тексту ответа и синтетические ретраи; пустые ответы LLM; провайдер без `tool_choice`; плохой wake-word ведёт к пропуску реплик; озвучка слишком длинная; путаница с тем, кто говорит.

---

## 1. Pipecat (pipecat-ai)

**Архитектура.**

- Линейный pipeline из `FrameProcessor`. Каждый процессор реализует `process_frame()` и `push_frame()`, кадр передаётся дальше, а не поглощается. Источник: https://docs.pipecat.ai/guides/learn/pipeline
- Две полосы обработки. **SystemFrame** (`InputAudioRawFrame`, `UserStartedSpeakingFrame`, `InterruptionFrame`, `ErrorFrame`) идут в приоритетную очередь, и прерывание их не выбрасывает. **Data/Control** кадры (`TranscriptionFrame`, `LLMTextFrame`, `TTSTextFrame`, `OutputAudioRawFrame`, `EndFrame`) идут в обычную очередь, порядок внутри полосы гарантирован.
- Контекст LLM собирают `context_aggregator.user()` и `context_aggregator.assistant()`.
- Turn-taking задаётся стратегиями в `LLMUserAggregatorParams`. Начало хода: `VADUserTurnStartStrategy`, `TranscriptionUserTurnStartStrategy`, `MinWordsUserTurnStartStrategy`, `ExternalUserTurnStartStrategy`. Конец хода: `TranscriptionUserTurnStopStrategy`, `TurnAnalyzerUserTurnStopStrategy` (smart-turn модель, `LocalSmartTurnAnalyzerV3`), `ExternalUserTurnStopStrategy`. Старый `MinWordsInterruptionStrategy` объявлен deprecated. Источник (поиск): https://docs.pipecat.ai/api-reference/server/utilities/turn-management/user-turn-strategies
- Function calling. Хэндлеру приходит `FunctionCallParams` с полями `function_name`, `tool_call_id`, `arguments`, `result_callback`, `context`, `llm`, `app_resources`. В `result_callback` можно передать `FunctionCallResultProperties` (`run_llm`, `on_context_updated`, `is_final`). Декоратор `@tool_options` даёт `cancel_on_interruption`, `timeout_secs`, `cancellable_by_llm`. Источник: https://docs.pipecat.ai/guides/learn/function-calling
- **Pipecat Flows** (с версии 1.5.0 влиты в ядро как `pipecat.flows`, отдельный пакет `pipecat-ai-flows` заморожен: https://github.com/pipecat-ai/pipecat-flows). Диалог описывается графом узлов. У `NodeConfig` есть `task_messages` (обязательное поле), `role_messages`, `functions`, `pre_actions`, `post_actions`, `context_strategy`, `respond_immediately`. Значения `ContextStrategy`: `APPEND`, `RESET` и deprecated `RESET_WITH_SUMMARY`. Хэндлер функции возвращает `tuple[result, NodeConfig | None]`: первый элемент уходит в LLM, второй задаёт следующий узел. У `FlowsFunctionSchema` есть `cancel_on_interruption` и `timeout_secs`. Источник: `src/pipecat_flows/types.py` в архивном репо.

**(а) Решение.** В базовом pipeline решает LLM через tools. Во Flows переходы между узлами решает **код хэндлера**: функция возвращает следующий узел, а LLM в каждом узле видит только тулы этого узла.

**(б) Turn-taking.** VAD плюс smart-turn модель. `InterruptionFrame` идёт в системной полосе и сбрасывает очереди генерации и TTS. По словам документации, assistant-агрегатор пишет в контекст `TTSTextFrame`, то есть текст, который реально озвучен; при word-timestamps прерванная реплика обрезается по словам (поиск: https://docs.pipecat.ai/pipecat/learn/context-management). В трекере есть открытая жалоба, что неозвученный текст всё же попадает в контекст, а тихий reconnect websocket не порождает `ErrorFrame` (поиск, issue #5305, не открывал: https://github.com/pipecat-ai/pipecat/issues/5305). Это наш класс бага «молчаливая деградация».

**(в) Состояние.** Состояние диалога живёт в `FlowManager` (текущий узел) и в контексте. Shared state передаётся через `app_resources` (подробности не подтверждены).

**(г) Подтверждение и ответ.** Результат тула возвращается в LLM. При `run_llm=False` LLM не вызывается, и ответ можно сказать кодом (через `pre_actions`/`post_actions` типа `tts_say`, точный набор action-типов не подтверждён).

**(д) Фолбеки.** Есть `ErrorFrame`. Для LLM/TTS существуют ServiceSwitcher/failover, но в этой сессии не подтверждено.

**(е) Память.** Контекст как список сообщений. Во Flows можно сбросить контекст при переходе (`RESET`).

**Перенять.**

- Системная полоса для событий прерывания и тишины, которую не блокирует генерация.
- `cancel_on_interruption` на уровне тула: движение нельзя отменять, поиск музыки можно.
- Узел графа с узким набором тулов и явным `context_strategy`: при смене сцены контекст сбрасывается **явно**.
- В контекст пишется только то, что озвучено.

**Избегать.** Не полагаться на то, что фреймворк сам сообщит о падении облака: issue #5305 показывает, что даже у Pipecat бывает тихий reconnect.

---

## 2. LiveKit Agents

**Архитектура.**

- `AgentSession` оркеструет STT→LLM→TTS (или realtime-модель) и может содержать несколько `Agent`. У каждого агента свои instructions и tools.
- **Handoff**: тул возвращает другой `Agent`, и управление переходит к нему.
- **AgentTask**: короткоживущая единица работы, которая выполняется до конца и возвращает **типизированный результат**. **TaskGroup** объединяет последовательность задач с возможностью вернуться к предыдущему шагу.
- При handoff контекст передаётся явно через `chat_ctx`: целиком, суммаризацией или с чистого листа. Источник: https://docs.livekit.io/agents/build/workflows/
- Тулы объявляются через `@function_tool`. `RunContext` даёт доступ к `session`, `function_call`, `speech_handle`, `userdata`. Есть `ctx.disallow_interruptions()` для необратимых записей и `ctx.wait_for_playout()`. `raise ToolError(...)` возвращает в LLM читаемую ошибку. Если тул вернул `None`, LLM-ответа не будет. `StopResponse` полностью подавляет вербальный ответ после тула (поиск). Источник: https://docs.livekit.io/agents/logic/tools/definition/
- Речь: `session.say(text)` озвучивает **фиксированный текст кодом**, `generate_reply()` просит LLM. Оба вызова возвращают `SpeechHandle` (`await handle`, `handle.interrupted`). Источник: https://docs.livekit.io/agents/build/audio/

**(а) Решение.** Действие выбирает LLM, но код может взять речь на себя (`say`, `StopResponse`, `None`), а структуру задаёт через Task с типизированным результатом.

**(б) Turn-taking.** Turn-detector модель (аудио плюс текст) поверх VAD, `min_delay`/`max_delay`, динамический endpointing. **Ложное прерывание** обрабатывается через `false_interruption_timeout` (после него эмитится `agent_false_interruption`) и `resume_false_interruption`, то есть агент продолжает речь, если «прерывание» оказалось кашлем. Флаг `allow_interruptions` выставляется на сессию или на реплику. `preemptive_generation` (включена по умолчанию) запускает LLM до подтверждения конца хода. Источники: https://docs.livekit.io/agents/build/turns/ и https://docs.livekit.io/agents/build/audio/. При прерывании транскрипт обрезается до сказанного (https://docs.livekit.io/agents/multimodality/text/), хотя в трекере есть баги рассинхрона (#5038, #4183, поиск).

**(в) Состояние.** `userdata` — один типизированный объект состояния сессии, общий для агентов. Сессия равна комнате или звонку.

**(г) Подтверждение и ответ.** Тул возвращает значение, которое строкой уходит в LLM. Альтернатива: тул сам вызывает `session.say(...)` и подавляет LLM-ответ.

**(д) Фолбеки.** Есть FallbackAdapter для LLM/STT/TTS (не подтверждено в этой сессии). События ошибок описаны в https://docs.livekit.io/reference/agents/events/.

**(е) Память.** `chat_ctx` на каждого агента, явная передача при handoff.

**Перенять.**

- `StopResponse` / `say()` из тула: фраза об успехе строится кодом, LLM молчит. Это прямо наш ADR-0148.
- `disallow_interruptions()` на время необратимого действия.
- Ложные прерывания: после паузы без продолжения возобновлять речь.
- Явная передача контекста при смене агента или персоны, а не «вся история по умолчанию».
- Типизированный результат Task вместо свободного текста.

**Избегать.** Preemptive generation на нашем железе и бюджете: двойная оплата облака плюс риск начать действие до конца фразы. Если и включать, то только для генерации текста, без тулов.

---

## 3. Home Assistant Assist

**Архитектура.**

- Assist pipeline: стадии `wake_word → stt → intent → tts`. События `run-start`, `wake_word-start/end`, `stt-start`, `stt-vad-start/end`, `intent-start/progress/end`, `tts-start/end`, `run-end`, `error`. Коды ошибок вроде `wake-word-timeout`, `stt-provider-missing`, `intent-failed`, `tts-not-supported`. Сессию несут `conversation_id` и `device_id`. Источник: https://developers.home-assistant.io/docs/voice/pipelines/
- Локальное распознавание интентов делает **hassil**, шаблоны предложений: `[optional]`, `(a|b)`, `{list:slot}`, `<rule>`, `{0..100:slot}`, wildcard-списки (https://github.com/OHF-Voice/hassil).
- Предложения, **ответы** и тесты лежат в репо `intents`: `sentences/<lang>/*.yaml`, `responses/<lang>/*.yaml`, `tests/<lang>/*.yaml`. Ответ — это шаблон на интент и ключ ответа (например `default` для `HassTurnOn`), подставляются слоты. Источник: https://github.com/home-assistant/intents
- Ответ API типизирован: `action_done` (со списками success/failed targets), `query_answer`, `error` (например `no_intent_match`). Также есть `speech` (plain/SSML) и `continue_conversation`. Источник: https://developers.home-assistant.io/docs/intent_conversation_api/
- LLM API: `llm.API` → `APIInstance(api_prompt, llm_context, tools)`, `Tool.async_call` → `ToolResult`. `IntentTool` оборачивает тот же intent handler и возвращает `IntentResponseDict(intent_response)`. Источники: https://developers.home-assistant.io/docs/core/llm/ и `homeassistant/helpers/llm.py`.
- Обоснование выбора интентов: «smallest API», потому что большая поверхность путает маленькие модели (https://www.home-assistant.io/blog/2024/06/07/ai-agents-for-the-smart-home/).
- **Prefer handling commands locally** (`prefer_local_intents`): сначала локальные интенты и sentence triggers, LLM только как фолбек (поиск: https://www.home-assistant.io/integrations/conversation/).

**(а) Решение.** Гибрид. Шаблонная фраза ведёт к детерминированному интенту, а LLM получает те же интенты как тулы.

**(б) Turn-taking.** Wake word и VAD на сателлите, `continue_conversation` для follow-up. Барж-ин примитивный.

**(в) Состояние.** Владелец состояния — сама HA (entity states), диалог хранит только `conversation_id`. Состояние приходит к LLM снимком exposed entities в промпте и через тул GetLiveContext.

**(г) Подтверждение и ответ.** На локальном пути ответ **строит код** из шаблона responses и результата интента (`action_done` со списками success/failed). На LLM-пути LLM пересказывает `IntentResponseDict`.

**(д) Фолбеки.** Локальный путь работает без облака. Ошибки стадий типизированы.

**(е) Память.** История на `conversation_id`, без долгой памяти.

**Свежие уроки из трекера HA (2026), прямо про наши боли.**

- **Знание в промпте разъехалось с кодом.** После неймспейсинга тулов в 2026.9 встроенный промпт называл тул `homeassistant__GetLiveContext`, а реальное имя было `assist__homeassistant__GetLiveContext`, и чтение состояния сломалось. Фикс: убрать хардкод имени тула из промпта (issue #182006, PR #182111, поиск).
- **Модель отвечает по протухшему снимку в промпте**, игнорируя инструкцию вызвать GetLiveContext (поиск, там же).
- **Единицы чтения и записи не совпадают.** Live Context отдаёт яркость 0–255, а тул принимает 0–100, модель считает и шлёт вне диапазона. Issue #182568 открыт 18.09.2026, закрыт PR #182571 (открывал). Классический случай «знание в двух местах».
- `prefer_local_intents` не применяется в continued conversation: «Nevermind» уходит в LLM, и та переспрашивает (поиск, community-тред 883012). Локальные стоп-команды должны иметь приоритет в любом состоянии диалога.

**Перенять.**

- Двухуровневый роутер: детерминированные шаблоны (стоп, тише, громче, хватит, «Робби, замолчи») идут мимо LLM, и только остальное попадает в LLM.
- Типизированный результат действия (`action_done` / `query_answer` / `error` плюс success/failed targets), из которого шаблон строит фразу.
- Шаблоны ответов в одном каталоге на язык, с тестами.
- Ошибки стадий pipeline как типизированные события.

**Избегать.**

- Имена тулов и единицы измерения в тексте промпта.
- Снимок состояния в промпте без отметки времени.
- Локальный роутер, который отключается в «режиме продолжения».

---

## 4. OpenVoiceOS (OVOS) / Mycroft

**Архитектура.**

- Всё общается через **messagebus** (websocket, сообщения `type`/`data`/`context`).
- Интент-сервис — упорядоченный **pipeline** плагинов. Пример из Session: `['converse', 'padatious_high', 'adapt', 'common_qa', 'fallback_high', 'padatious_medium', 'fallback_medium', 'padatious_low', 'fallback_low']` (поиск).
- Стадии описаны в https://openvoiceos.github.io/ovos-technical-manual/ (Converse, Padatious, Adapt, Fallback, Common Query, Persona).
- **Converse**: активные скиллы (вызванные за последние 5 минут, `skill_timeouts`) получают реплику первыми через `converse()` → `True/False`, в обратном хронологическом порядке. Есть защита от угона диалога: `max_skill_runtime` и blacklist. Источник: https://openvoiceos.github.io/ovos-technical-manual/converse_pipeline/
- **Persona** (ovos-persona) — LLM-«персона» как цепочка solver-плагинов: если solver вернул None, пробуется следующий. Плагин `persona-high` ставится после высокоуверенных матчеров, `persona-low` — перед последним fallback. Место LLM в пайплайне **конфигурируется**. Источник: https://github.com/OpenVoiceOS/ovos-persona
- **Session** путешествует в `message.context` и содержит `session_id`, `lang`, `active_skills` (с timestamp), `pipeline`, `context` (timeout, frame_stack), `site_id`, настройки stt/tts (поиск).
- Речь: `self.speak_dialog('name', data)` выбирает случайный вариант из `.dialog`-файла локали и подставляет `{var}`. `wait=True` делает вызов блокирующим. Источник: https://openvoiceos.github.io/ovos-technical-manual/402-statements/

**(а) Решение.** Код: детерминированные матчеры по уверенности, LLM только в persona-стадии.

**(б) Turn-taking.** Wake word, затем utterance. Converse даёт «следующая реплика без wake-word идёт активному скиллу». Есть `stop`.

**(в) Состояние.** **Сессия — сериализуемый объект в каждом сообщении**: скиллы stateless относительно сессии, несколько сессий (устройств, хабов) не смешиваются.

**(г) Подтверждение и ответ.** Скилл сам говорит шаблоном после выполнения (`speak_dialog`). LLM текст не генерирует, если интент сматчился.

**(д) Фолбеки.** Fallback-скиллы по приоритетам, а в самом конце `complete_intent_failure`.

**(е) Память.** Минимальная: `active_skills` и context frame_stack с таймаутом.

**Перенять.**

- Pipeline с явными порогами уверенности: LLM ставится в конкретный слот, а не «всё через LLM».
- Session как явный объект с TTL, который едет с сообщением. Сброс сцены означает новую сессию, а не чистку десяти полей в разных нодах.
- Converse-окно на 5 минут для активного навыка: «громче» после «включи музыку» без wake-word.
- Диалоговые шаблоны с вариантами против роботизированности.

**Избегать.** Шину без схем. Сообщения OVOS слабо типизированы, интеграции ломаются молча (см. PR про «spec handlers act on payload skill_id, not sender» в поиске).

---

## 5. Rhasspy 3 / Wyoming

**Архитектура.**

- **Wyoming** — TCP-протокол «JSONL-заголовок плюс бинарный payload» (`type`, `data`, `data_length`, `payload_length`). События: `audio-start/chunk/stop`, `transcribe`/`transcript` (плюс streaming `transcript-start/chunk/stop`), `synthesize` (плюс `synthesize-start/chunk`), `detect`/`detection`/`not-detected`, **`describe`→`info`** для обнаружения возможностей сервиса. Источник: https://github.com/OHF-Voice/wyoming
- Rhasspy 3 делит работу на домены `mic, wake, vad, asr, intent, handle, tts, snd`, программы общаются по Wyoming через stdin/stdout. Проект **архивирован 06.10.2025** (https://github.com/rhasspy/rhasspy3). Wyoming живёт дальше в HA.

**Оси.** (а) Решение в домене `handle` (HA conversation или своё). (б) Барж-ин не описан. (в) Состояние не определено протоколом. (г) Подтверждение и ответ не определены. (д) Деградация через `describe/info`: можно узнать, что сервис умеет. (е) Памяти нет.

**Перенять.** **Capability negotiation** (`describe`→`info`): каждый TTS/STT-провайдер отвечает, что умеет (языки, голоса, SSML, streaming), и оркестратор выбирает по ответу, а не по конфигу.

**Избегать.** Протокол без событий здоровья. `info` статичен и не говорит, что у провайдера кончились деньги.

---

## 6. Rasa CALM

**Архитектура.**

- **Dialogue Understanding**: LLM (command generator) читает реплику в контексте и выдаёт **список команд**, а не ответ. Команды: `StartFlow(flow)`, `SetSlot(slot, value)`, `CorrectSlot`, `CancelFlow()`, `Clarify(flow_a, flow_b, …)`, `ChitChat`, `KnowledgeAnswer`, `HumanHandoff`, `Error` (поиск: https://rasa.com/docs/reference/config/components/llm-command-generators/).
- **Flows** (бизнес-логика по шагам) исполняет детерминированный `FlowPolicy` — state machine с **dialogue stack** (LIFO): прерывающий flow кладётся сверху, после него возобновляется прежний (https://rasa.com/docs/reference/config/policies/flow-policy/).
- **Conversation repair** — системные pattern-flows: `pattern_correction`, `pattern_cancel_flow` (`action_cancel_flow` чистит слоты отменённого flow), `pattern_clarification`, `pattern_internal_error`, `pattern_cannot_handle`, `pattern_chitchat` (поиск: https://rasa.com/docs/learn/concepts/conversation-patterns).
- Ответы — шаблоны (responses). Необязательный **contextual response rephraser** только перефразирует шаблон для беглости.
- Принцип из доков, коротко: LLM ведёт разговор, но не угадывает бизнес-логику (https://rasa.com/docs/learn/concepts/calm/).

**(а) Решение.** Решает код (flows). LLM лишь переводит реплику в команды из закрытого перечня.

**(б) Turn-taking.** Не про голос, есть только текстовые ходы.

**(в) Состояние.** Слоты и dialogue stack, единственный владелец — трекер.

**(г) Подтверждение и ответ.** Шаг flow выполняет action, ответ — шаблон, rephraser опционален.

**(д) Фолбеки.** `pattern_internal_error` и `pattern_cannot_handle` — явные flows на сбой.

**(е) Память.** Трекер событий (event-sourced история), слоты со сбросом при отмене flow.

**Перенять.**

- **Команды вместо ответов**: LLM выдаёт `StartFlow/SetSlot/Cancel/Clarify/ChitChat`. Это ровно «узкая схема» для нашего случая без `tool_choice`: даже без нативного function calling одну команду из перечня можно распарсить и валидировать.
- `Clarify(a, b)` как штатный исход вместо выдумки, когда не понял.
- Стек сцен: «тишина» или «DJ-сет» — flow на стеке, у которого есть явный pop и cleanup слотов. Это лечит утечки между сценами.
- Repair-паттерны как код, а не как правила в промпте.

**Избегать.** Перекладывать на YAML-flows открытый small talk. Rasa сама уводит его в `ChitChat`/`KnowledgeAnswer`.

---

## 7. Letta (MemGPT) и mem0

**Letta.**

- **Memory blocks** (`label`, `value`, `limit`, `description`, `read_only`) всегда в контексте, их можно **шарить между агентами**, агент сам правит их тулами (`memory_replace`, `memory_insert`, `core_memory_append`), внешний код может писать через API (https://docs.letta.com/guides/agents/memory-blocks).
- Вся история, тул-вызовы и reasoning персистятся в БД и после выселения из окна доступны через recall/archival (https://docs.letta.com/guides/agents/context-engineering).
- **Tool rules** ограничивают порядок тулов структурно: `InitToolRule`, `TerminalToolRule` (по умолчанию на `send_message`), `ChildToolRule`, `ParentToolRule`, `ConditionalToolRule` (следующий тул по выходу предыдущего), `ContinueToolRule`, `MaxCountPerStepToolRule` (поиск: https://docs.letta.com/guides/agents/tool-rules).

**mem0.**

- `add` прогоняет сообщения через LLM-экстракцию фактов. Текущая документация описывает модель «ADD-only», без перезаписи. Скоуп задаётся через `user_id`, `agent_id`, `app_id`, `run_id`. `infer=False` пишет как есть (https://docs.mem0.ai/core-concepts/memory-operations/add).
- Ранние версии делали ADD/UPDATE/DELETE/NONE. Для текущей версии это не подтверждено, доки говорят об ADD-only.

**Оси.** (а) Решает LLM, но с tool rules. (в) Состояние в блоках с владельцем; `read_only` защищает от записи LLM. (е) Многоуровневая память, явные скоупы.

**Перенять.**

- Блок `read_only` для фактов, которые пишет **код** (текущий голос, персона, режим тишины, кто говорит): LLM видит, но не правит.
- Скоупы памяти `user_id` (диктор), `run_id` (сцена или сет). Сцена получает свой `run_id`, при её закрытии ничего не утекает.
- `TerminalToolRule` / `ConditionalToolRule` как структурные ограничения последовательности. Например, после `play_music` разрешён только `say_result`.

**Избегать.** Позволять LLM самой писать долговременную память про людей без валидации: у нас уже была «отравленная галерея» лиц. LLM-экстракция фактов на русском без проверки множит дубли.

---

## 8. Робототехника: ROSA, RAI, ROS-LLM, SayCan

**ROSA (NASA JPL).**

- LangChain-агент с набором ROS1/ROS2-тулов (интроспекция топиков, нод, сервисов, параметров) плюс custom tools, `RobotSystemPrompts` и blacklist.
- Агент по сути ReAct, действия — тулы. Механика подтверждения исполнения в README не описана (https://github.com/nasa-jpl/rosa).
- (а) Решает LLM. (д) blacklist. Интересно как диагностический агент, а не как диалог.

**RAI (Robotec.ai).**

- Вендор-агностичный мультиагентный фреймворк для ROS 2: `rai_asr`, `rai_tts` (голосовые агенты), `rai_perception`, `rai_whoami` (воплощение робота из доков и URDF), `rai_nomad`.
- Доки: https://robotecai.github.io/rai, статья arXiv 2505.07532 (https://github.com/RobotecAI/rai).
- Детали про валидацию действий в этой сессии не подтверждены.

**ROS-LLM (Auromix).** Пакеты `llm_input` / `llm_model` / `llm_robot` / `llm_output`, роботу дают «функциональный интерфейс». Безопасность и подтверждение не описаны (https://github.com/Auromix/ROS-LLM).

**SayCan (Google).** LLM оценивает, насколько навык из **фиксированной библиотеки** продвигает к цели («say»). Это значение умножается на affordance value function («can»: выполнимо ли сейчас). Выполняется лучший навык, и цикл повторяется (https://say-can.github.io/).

**Перенять.**

- SayCan-идея **«can» от кода**: перед тем как LLM выберет тул, код помечает, какие тулы сейчас выполнимы. Например, `play_music` недоступен в SILENCED, движение недоступно без моторов. Недоступные тулы вообще не показываются.
- Фиксированная библиотека навыков с детерминированным исполнением.

**Избегать.** ROSA-стиль «LLM сама ходит по ROS-графу» в боевом диалоге. У ROS-LLM и ROSA нет описанной петли подтверждения исполнения, это ровно наша боль «выдумывает успех».

---

## 9. OpenAI Realtime и Agents SDK

**Realtime.**

- `turn_detection`: `server_vad` / `semantic_vad` (с eagerness, не открывал), `null` для push-to-talk. Флаги `interrupt_response` и `create_response` отделяют детекцию речи от автозапуска ответа.
- При прерывании клиент шлёт `conversation.item.truncate` с `audio_end_ms`, и неозвученная часть транскрипта удаляется.
- **Out-of-band responses** (`"conversation": "none"`) позволяют классифицировать интент параллельно, не засоряя историю.
- Источник: https://developers.openai.com/api/docs/guides/realtime-conversations

**Agents SDK guardrails.**

- Input-guardrail работает параллельно (по умолчанию) или блокирующе, output-guardrail проверяет финальный выход. Tripwire поднимает `InputGuardrailTripwireTriggered` / `OutputGuardrailTripwireTriggered` и останавливает выполнение.
- **Tool guardrails** оборачивают `FunctionTool` до и после вызова (могут пропустить вызов или заменить вывод), исключения `ToolInputGuardrailTripwireTriggered` / `ToolOutputGuardrailTripwireTriggered`.
- Источник: https://openai.github.io/openai-agents-python/guardrails/

**Перенять.**

- Truncate истории до озвученного (`audio_end_ms`).
- `create_response=false`: детекция хода и запуск ответа разделены, решение о запуске принимает код (например, не отвечать, если реплика не адресована роботу).
- Out-of-band классификация.
- Tool guardrail «до вызова»: валидатор аргументов с понятной ошибкой.

**Избегать.** Output-guardrail как регекс по тексту ответа. Это наша же временная заплатка из ADR-0148.

---

## 10. Anthropic «Building effective agents»

- **Workflows** — предопределённые пути в коде. **Agents** — LLM сама управляет процессом.
- Паттерны: prompt chaining, **routing**, parallelization, orchestrator-workers, evaluator-optimizer.
- Agent-computer interface: инструментам уделяется больше внимания, чем промпту, и действует принцип **poka-yoke** — аргументы устроены так, чтобы ошибиться было трудно.
- Агенты подходят открытым задачам, где число шагов непредсказуемо, и требуют гардрейлов и тестов в песочнице.
- Источник: https://www.anthropic.com/engineering/building-effective-agents

**Перенять.**

- Голосовой ассистент робота в основном **workflow с routing**: классификация, затем узкий обработчик. «Агент» нужен только для открытых задач (композитор, поиск).
- Poka-yoke-схемы: enum вместо строки, громкость в процентах 0–100 с одной единицей.

---

## 11. Leon, OpenInterpreter 01, Willow (кратко)

**Leon 2.0 (developer preview).**

- Ушёл от чистой intent-классификации к трём режимам: **Smart** (автовыбор), **Controlled** (детерминированные native skills) и **Agent** (пошаговое планирование).
- Иерархия «Skills → Actions → Tools → Functions», слоистая память (предпочтения, дневной контекст, недавний разговор), контекст «grounded in actual machine state».
- Архитектура описана в `core/context/ARCHITECTURE.md` (https://github.com/leon-ai/leon).
- **Перенять**: явный режим Controlled против Agent, выбираемый роутером.

**OpenInterpreter 01.** Клиенты (ESP32, мобильные), сервер LiveKit или Light, LLM исполняет код как тул. Сами авторы пишут, что проекту не хватает базовых safeguards (https://github.com/openinterpreter/01). **Избегать**: «код как универсальный тул» на роботе.

**Willow.** ESP32-S3-BOX: wake и VAD на устройстве, команды на устройстве или через Willow Inference Server, затем в HA, openHAB или REST. Обратная связь звуком и экраном (https://heywillow.io/). **Перенять**: подтверждение действия неречевым сигналом (earcon или свет) вместо длинной фразы.

Juniper в этой сессии не исследован, данных нет.

---

## 12. Сквозные паттерны (по литературе и проектам выше)

- **Statecharts / state machine диалога.** Rasa FlowPolicy (стек), Pipecat Flows (граф узлов), LiveKit Task/TaskGroup. Общее: состояние сцены — явный объект с входом и выходом, сброс контекста при переходе задаётся явно.
- **Dialogue state tracking.** Слоты Rasa, `userdata` LiveKit, Session OVOS. Одно хранилище, один владелец.
- **Event sourcing исполнения.** Трекер событий Rasa, события Assist pipeline (`intent-start/end`, `error`), персист всех тул-вызовов в Letta.
- **Structured outputs / команды.** Команды Rasa, типизированный результат AgentTask, `ToolResult` в HA.
- **Tool result → templated response.** HA responses YAML, OVOS `.dialog`, Rasa responses (плюс опциональный rephraser), LiveKit `session.say` + `StopResponse`.
- **Capability negotiation.** Wyoming `describe/info`, OVOS Session с stt/tts-настройками.
- **Health / degradation.** Типизированные коды ошибок в HA pipeline, `ErrorFrame` в Pipecat (и его дыра #5305).

---

## 13. Сравнительная таблица по осям (а)–(е)

| Проект | (а) Кто решает | (б) Turn-taking / барж-ин | (в) Владение состоянием, сессии | (г) Подтверждение и ответ | (д) Фолбеки и деградация | (е) Память и история |
|---|---|---|---|---|---|---|
| Pipecat | LLM-тулы; во Flows переход решает код хэндлера | VAD + smart-turn, `InterruptionFrame` в системной полосе, `cancel_on_interruption` | FlowManager (узел) + контекст | результат → LLM; `run_llm=False` + actions | `ErrorFrame`; тихий reconnect (#5305, поиск) | контекст-список; `RESET` при переходе |
| LiveKit Agents | LLM; код через Task, handoff, `say` | turn-detector, false-interruption resume, `allow_interruptions`, preemptive | `userdata` на сессию | `say()`/`StopResponse`/`None` — код говорит; `ToolError` | FallbackAdapter (не подтверждено) | `chat_ctx` на агента, явная передача |
| HA Assist | шаблоны hassil, затем интент; LLM как фолбек | wake + VAD, `continue_conversation` | HA entities (единый владелец), `conversation_id` | `action_done` + success/failed, шаблон responses | локальный путь без облака; коды ошибок стадий | история на conversation_id |
| OVOS | pipeline матчеров по уверенности; LLM в persona-слоте | wake, converse-окно 5 мин, stop | Session в `message.context` | скилл: `speak_dialog` по шаблону | fallback-скиллы, `complete_intent_failure` | active_skills + frame_stack с TTL |
| Rhasspy3 / Wyoming | домен handle | не описано | не определено | не определено | `describe/info` (capabilities) | нет |
| Rasa CALM | код (flows); LLM → команды из перечня | текстовые ходы | слоты + dialogue stack | шаблоны + опциональный rephraser | `pattern_internal_error`, `pattern_cannot_handle` | трекер событий; сброс слотов при cancel |
| Letta / mem0 | LLM + tool rules | — | memory blocks (shared, `read_only`) | `send_message` terminal | — | блоки + recall/archival; скоупы user/agent/run |
| ROSA / ROS-LLM / RAI | LLM (ReAct) | — (RAI: ASR/TTS-агенты) | не описано | не описано | blacklist (ROSA) | не описано |
| SayCan | LLM × affordance (код) | — | — | пошаговое исполнение навыка | невыполнимые навыки отсекаются | — |
| OpenAI Realtime / Agents SDK | LLM + guardrails | semantic_vad, `interrupt_response`, `create_response`, truncate | conversation | tool guardrails до и после | tripwire-исключения | truncate до озвученного; out-of-band |
| Anthropic BEA | workflow (код) против agent | — | — | poka-yoke тулы | гардрейлы | — |
| Leon 2.0 | Controlled (код) / Agent (LLM) / Smart | не изучено | контекст «по реальному состоянию машины» | tools вместо свободного текста | не изучено | слоистая память |

---

## 14. Паттерны-кандидаты для rob_box (13)

1. **Двухуровневый роутер «локальные шаблоны → LLM»** (HA `prefer_local_intents`, OVOS pipeline). Стоп, тише/громче, «хватит», «замолчи», «включи голос X» матчатся детерминированно (hassil-подобная грамматика на русском) и **работают в любом состоянии**, включая continued conversation (урок HA-треда 883012). *Закрывает:* решения в промпте; пустые ответы LLM; молчащую деградацию облака LLM (базовые команды живут без него).

2. **Команды вместо свободного ответа** (Rasa CALM). LLM выдаёт одну команду из закрытого перечня (`StartScene`, `SetSlot`, `Cancel`, `Clarify`, `ChitChat`, `CallTool`), парсер и валидатор проверяют её кодом. *Закрывает:* провайдер без `tool_choice` (JSON-команда валидируется так же); выдуманные действия; регексы по тексту ответа.

3. **Типизированный результат действия → шаблон ответа** (HA `action_done` + success/failed targets, OVOS `.dialog`, LiveKit `say` + `StopResponse`). Тул возвращает `ActionResult{status, targets_ok, targets_failed, reason}`, фразу строит шаблон из одного каталога, а LLM-ответ по этому ходу подавлен или ограничен перефразированием (Rasa rephraser). *Закрывает:* LLM выдумывает успех; длинная озвучка (шаблоны короткие).

4. **Сцена как flow на стеке с явным выходом и cleanup** (Rasa dialogue stack + `action_cancel_flow`, Pipecat `context_strategy=RESET`, LiveKit TaskGroup). Тишина, DJ-сет, персона — это frame на стеке, а pop возвращает слоты к значениям до входа. *Закрывает:* утечки состояния между сценами (SILENCED по «хватит», set_voice из акта 4).

5. **Session-объект с TTL, единый владелец** (OVOS Session, LiveKit `userdata`, Letta `read_only` блоки). Голос, персона, режим тишины, активный навык и текущий диктор лежат в одном объекте, пишет только владелец (код), LLM видит read-only снимок. *Закрывает:* нескольких владельцев одного состояния; LLM, «переключающую» голос словами.

6. **«Can» от кода: фильтр доступных тулов по состоянию** (SayCan affordance, Pipecat Flows «тулы узла»). Перед вызовом LLM код отдаёт только выполнимые сейчас тулы: в SILENCED нет `play_music`, без моторов нет движения. *Закрывает:* выдумки про невыполнимое; решения в промпте («если тишина, не играй»).

7. **Capability negotiation + health-события провайдеров** (Wyoming `describe/info`, HA коды ошибок стадий). Каждый TTS/LLM-адаптер публикует `capabilities` (SSML, голоса, язык, `tool_choice`) и `health` (ok / degraded / out_of_credit / down) топиком, оркестратор выбирает по health, деградация озвучивается или логируется явно. *Закрывает:* молчаливую деградацию облаков; провайдер без `tool_choice` (оркестратор знает это заранее и переключает режим на команды из п.2).

8. **В историю пишется только озвученное** (OpenAI `conversation.item.truncate` с `audio_end_ms`, Pipecat `TTSTextFrame`, LiveKit синхронизированный транскрипт). При барж-ине реплика обрезается до озвученного. *Закрывает:* путаницу «что робот сказал» после прерываний; LLM «помнит» обещание, которое человек не слышал.

9. **Политика прерываний на уровне действия** (Pipecat `cancel_on_interruption`, LiveKit `disallow_interruptions()`, `allow_interruptions` на реплику). Движение и запись не отменяются прерыванием, поиск и болтовня отменяются. Плюс **ложные прерывания** (LiveKit `false_interruption_timeout` + `resume_false_interruption`): шум или кашель не убивает реплику. *Закрывает:* барж-ин, рвущий действия; лишние обрывы речи.

10. **Разделить «детекцию хода» и «решение отвечать»** (OpenAI `create_response=false`, Pipecat External turn strategies, OVOS converse-окно). Код решает, адресована ли реплика роботу: wake-word **или** активное converse-окно у навыка на N секунд **или** идентифицированный диктор в активной сессии. *Закрывает:* пропуск реплик из-за неуслышанного wake-word (n204 → акт 2); ответы на чужую речь.

11. **Event log исполнения** (трекер Rasa, события Assist pipeline, персист тул-вызовов Letta). Каждый ход порождает события `turn_started / intent / tool_called / tool_result / spoken / interrupted / error` с correlation id, по ним строятся ответ, аудит и тесты. *Закрывает:* «пустой tool_calls у "не вышло" = выдумка» (видно по логу); честный raw-вывод для E2E.

12. **Знание в одном модуле, промпт генерируется из него** (уроки HA #182006 и #182568). Имена тулов, единицы (громкость 0–100), списки голосов и персон берутся из одного реестра. Промпт не хардкодит имена, тул чтения и тул записи используют одни единицы. *Закрывает:* знание в двух местах; ошибки параметров LLM.

13. **Структурные правила последовательности тулов вместо правил в промпте** (Letta `TerminalToolRule` / `ConditionalToolRule` / `MaxCountPerStepToolRule`, OpenAI tool guardrails «до вызова»). Например: после действия разрешён только финал; не больше одного `set_voice` за ход; валидатор аргументов возвращает понятную ошибку (`ToolError`). *Закрывает:* синтетические ретраи и регекс-заплатки; зацикливание тулов.

**Дополнительно (низкий приоритет).** Неречевое подтверждение (earcon или свет, как у Willow) для простых действий вместо фразы, против длинной озвучки. Память о людях скоупить `user_id`, сцены скоупить `run_id` (mem0), LLM в память людей сама не пишет (урок отравленной галереи).

## 15. Что не подтверждено и требует отдельной проверки

- Точный набор action-типов в Pipecat Flows (`tts_say`, `end_conversation`, `function`) и наличие failover-свитчеров в Pipecat.
- LiveKit FallbackAdapter (имя и поведение).
- `semantic_vad.eagerness` в OpenAI Realtime (страница не раскрыла параметр).
- Текущая семантика mem0 (ADD-only против ADD/UPDATE/DELETE).
- Детали валидации действий в RAI.
- Juniper не исследован.
- Issues HA #182006 и PR #182111, Pipecat #5305, LiveKit #5038 и #4183 взяты из поисковой выдачи, сами страницы не открывались (кроме HA #182568).
