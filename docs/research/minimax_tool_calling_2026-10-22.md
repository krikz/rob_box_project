# MiniMax: вызов тулов и режим команд — исследование 22.10.2026

Заказ: товарищ Шифу, 22.10 09:17Z («поищем их харнес или CLI, глянем, как они работают с моделью, почитаем документацию»).
Эпик #3683, ADR-0150 §4.5. Ничего в репо не менялось, к API MiniMax запросов не было (ключи не использовались).
Все выводы о поведении модели ниже — из документации и чужого кода; **на нашем роботе это не проверено**, план замера — в конце.

## 1. Источники

Документация (читано 22.10, WebFetch; страницы местами противоречат друг другу — отмечено):
- https://platform.minimax.io/docs/llms.txt — оглавление
- https://platform.minimax.io/docs/api-reference/text-chat-openai.md — Create Chat Completion (OpenAI-совместимый)
- https://platform.minimax.io/docs/api-reference/text-post.md — Text Generation (`/v1/text/chatcompletion_v2`)
- https://platform.minimax.io/docs/api-reference/text-openai-api.md — OpenAI SDK
- https://platform.minimax.io/docs/api-reference/text-anthropic-api.md и `text-chat-anthropic.md` — Anthropic-совместимый
- https://platform.minimax.io/docs/guides/text-m3-function-call.md и `.../text-m2-function-call` — Tool Use & Interleaved Thinking
- https://platform.minimax.io/docs/guides/text-generation.md — обзор, thinking
- https://platform.minimax.io/docs/api-reference/text-prompt-caching.md — кэш промпта

Репозитории (клонированы read-only в scratchpad, ничего не запускалось; SHA и дата HEAD):
- https://github.com/MiniMax-AI/Mini-Agent — `d76a4f6` (14.02.2026), эталонный агент на M2.5, два клиента: Anthropic (по умолчанию) и OpenAI.
- https://github.com/MiniMax-AI/minimax-code — `da22f11` (21.10.2026), их кодовый агент для терминала (TypeScript), поверх форка pi-mono (`third_party/pi-mono`).
- https://github.com/MiniMax-AI/MiniMax-Provider-Verifier — `2e387fb` (21.10.2026), официальный набор проверок «ведёт ли себя провайдер M3 как эталон» (каталог `m3_format_check`).
- https://github.com/MiniMax-AI/MiniMax-M2.7 — `docs/tool_calling_guide.md` (формат тулов у самой модели).
- https://github.com/MiniMax-AI/cli (mmx-cli) — только упомянут: это медиа-CLI (картинки, голос, музыка), к чату с тулами отношения не имеет; не разбирался.

## 2. Что делает их харнес

### 2.1 Mini-Agent (`mini_agent/`)
- Запрос с тулами: `tools` и всё. **`tool_choice`, `temperature`, `top_p` не задаются нигде** (grep по `mini_agent/` пуст). Агент зовёт `self.llm.generate(messages=self.messages, tools=tool_list)` (`agent.py:372`).
- OpenAI-клиент: единственная добавка к телу — `extra_body={"reasoning_split": True}` (`llm/openai_client.py:69`). Мысли приходят отдельно и **возвращаются в историю** полем `reasoning_details=[{"text": ...}]` в assistant-сообщении (`openai_client.py:165-166`); над этим комментарий «CRITICAL for Interleaved Thinking» (строки 160-164).
- Anthropic-клиент (дефолт, `config-example.yaml`: `provider: "anthropic"`): блоки `thinking`, `text`, `tool_use` собираются обратно в assistant-сообщение (`anthropic_client.py:134-158`), `max_tokens=16384` (стр. 69). Результат тула — `user` с блоком `tool_result` (стр. 163-176).
- Параллельные вызовы: цикл идёт по всем `response.tool_calls` по порядку, каждому — свой `tool`-ответ (`agent.py` ~425-500).
- Невалидные аргументы: **схемы никто не проверяет**. Незнакомый тул → `Unknown tool: <имя>` (`agent.py:458`); исключение тула → `Tool execution failed: ...` + traceback (стр. 473); в историю уходит `"Error: ..."` как обычный `tool`-ответ (стр. 497). Повтор запроса к модели — только транспортный (`retry.py`, 3 раза, экспонента) и только на сбой HTTP. Ошибка `json.loads` аргументов в OpenAI-клиенте **не ловится** (`openai_client.py:231`) — упадёт весь вызов.
- Системный промпт — обычный файл (`config/system_prompt.md`), в начале истории. Температуры нет.

### 2.2 minimax-code (их «официальный» CLI-агент)
- Ходит на **Anthropic-совместимый** эндпойнт: `MINIMAX_API_FORMAT = 'anthropic-messages'`, база `https://api.minimax.io/anthropic` (`packages/local-runtime/src/model-provider/minimax-api.ts:7-12`).
- `toolChoice` в слое агента не задаётся (grep по `packages/agent-core/src` пуст); общий код pi-mono пробрасывает `tool_choice`, только если его передали (`third_party/pi-mono/packages/ai/src/providers/openai-completions.ts:535`).
- Thinking по моделям (`packages/config/src/minimax-model-catalog.ts`): M3 — переключаемый (`thinking_config: switchable`, варианты `disabled` / `adaptive`, стр. 25-28); M3.1-Flash — `forced_on` с градациями усилия `low…max` (стр. 36-47). Совпадает с нашим `apply_model_thinking`.
- Аргументы тулов **валидируются по JSON-схеме с мягким приведением типов** (`pi-mono/packages/ai/src/utils/validation.ts:292-326`: `Value.Convert`, затем `coerceWithJsonSchema`, затем ошибка со списком полей и «Received arguments»). Ошибка превращается в tool-результат `isError: true`, цикл продолжается, модель правит сама (`pi-mono/packages/agent/src/agent-loop.ts:733, 773-777`). Отдельного повтора «всего запроса» нет.
- Температура: передаётся, только если модель её поддерживает и thinking выключен (`anthropic.ts:1027`) — для M3 всё равно игнорируется сервером.

### 2.3 MiniMax-Provider-Verifier (что MiniMax считает правильным поведением M3)
- Тест `13_08` (`m3_text_tests.py:1444`) требует: `tool_choice="none"` — тул не вызван; **`"required"` и `"auto"` — вызван** (запрос «погода в Пекине» с тулом `get_weather`). То есть эталонная платформа заявляет поддержку `required` — см. расхождение с ADR-0143 в §5.
- Стрим-тесты `02_09/02_10` шлют `tool_choice="auto"`, `parallel_tool_calls=true`, `reasoning_split=true`, `max_tokens=64000`.
- Граничные тесты: несовпавший `tool_call_id` → ожидается HTTP 400 (`16_08`), ответ только на один из двух вызовов → 400 (`16_09`), невалидный JSON в `arguments` истории → 400 (`16_12`). Для нас это правило сборки истории: ответ на КАЖДЫЙ `tool_call_id`, аргументы в истории — валидный JSON.
- Метрики README: «только рассуждение, без текста и тула» (`Error-Only-Reasoning-Rate`) — известный вид брака; порядок `parameters.properties` должен сохраняться (провайдер, сортирующий свойства по алфавиту, проваливает проверку порядка параметров).

### 2.4 Формат самой модели (`MiniMax-M2.7/docs/tool_calling_guide.md`)
- Тулы модель пишет XML-блоком `<minimax:tool_call>` с `<invoke name=…>`; в одном блоке может быть несколько `invoke` (параллельные вызовы возможны). Пример в гайде использует `tool_choice="auto"`, рекомендованных `temperature`/`top_p` в гайде нет (в README модели: `temperature=1.0, top_p=0.95, top_k=40`).

## 3. Что говорит документация

| Вопрос | Что написано | Оговорка |
|---|---|---|
| `tool_choice` | Спецификация (OpenAI-стр. `text-post.md` и Anthropic-стр. `text-chat-anthropic.md`): поддерживается **только `auto`**, он же дефолт. Сводная страница Anthropic-совместимого API называет `tool_choice` «fully supported». | Страницы противоречат друг другу; `text-chat-openai.md` про `tool_choice` молчит. Официальный верификатор (§2.3) ждёт работу `required`. |
| `parallel_tool_calls` | В справочниках не описан; верификатор его шлёт; примеры гайда разбирают только первый вызов. | |
| `strict` в тулах / `response_format` | `strict` не описан. `response_format` — только `json_schema` и, по тексту, «только `MiniMax-Text-01`»; для M3 верификатор прямо убрал `response_format` («M3 пока не поддерживает»). Режима `json_object` в доке нет. | Наши флаги `json_mode=False`, `structured_output=False` честны. |
| `temperature` / `top_p` | «Не поддерживается, любое значение не влияет»: M3 и M3.1-Flash всегда 1 и 0.95. | Наш `temperature: 0.7` в конфиге ничего не делает. |
| Thinking | `thinking.type`: `disabled` / `adaptive`. M3: `disabled` отключает; M3.1-Flash: `disabled` = HTTP 400, глубина через `reasoning_effort` (`low…max`, дефолт `max`, `none` = 400). Для M2.x параметр игнорируется. | Страница OpenAI: дефолт `adaptive`. Страница Anthropic: у M3 при пропуске thinking **выключен**. Обзорная `text-generation.md` утверждает, что thinking отключить нельзя вовсе. Правду скажет только замер. |
| Где мысли | `reasoning_split=true` → отдельное поле `reasoning_content`; `false` → `<think>…</think>` внутри `content` (Flash `false` не поддерживает). В Anthropic — блок `thinking` с `signature`. | Поле `reasoning_details` (его использует Mini-Agent) в текущей доке **не описано** — описано `reasoning_content`. |
| История с тулами | Возвращать ответ модели целиком (мысли + `tool_calls`) «каждый раз»; в Anthropic — весь `response.content`, блоки `thinking` без правок. Для простых вызовов мысли может не быть — всё равно целиком. | Что будет, если мысли не вернуть, док не говорит (ошибки нет; в коде примера — «потеряется контекст»). |
| Лимит вывода | `max_tokens` устарел, нужен `max_completion_tokens`; токены мыслей входят в лимит. | |
| Кэш | Автоматический, от 512 входных токенов, без `cache_control`; попадания — `usage.prompt_tokens_details.cached_tokens`. `cache_control` для M3 не поддерживается. | |
| Эндпойнты | Anthropic-совместимый в доке помечен «рекомендуемым»; OpenAI-совместимый — полноценный. | Мы на OpenAI-стороне. |

## 4. Как это стыкуется с режимом команд (данные бейк-оффа 21.10)

Из `docs/dialog/speech_channel_bakeoff_2026-10-21.md` (канал `field`, 174 прогона на ячейку, минимальный промпт, тулы-заглушки):

| ячейка | delivered | invalid | p50 / p95, мс |
|---|---|---|---|
| MiniMax-M3, json_in_text | 80/126 (63 %) | 52 (49 × `passed:false`) | 1458 / 5025 |
| MiniMax-M3, **тул `command` + `tool_choice="required"`** | 105/126 (83 %) | 23 (17 × `args:""`, 6 × текст без тула) | 2079 / 5130 |
| MiniMax-M3.1-Flash, json_in_text | 82/126 (65 %) | 38 | 1913 / 7642 |
| MiniMax-M3.1-Flash, **тул + `required`** | 115/126 (91 %) | 6 | 2072 / 6956 |
| DeepSeek (для сравнения), тул + `required` | 121/126 (96 %) | 0 | 1118 / 1535 |

Вывод, который я из этого делаю (гипотеза, не доказано): почти весь выигрыш даёт не сам `required`, а то, что **схема приходит в `tools`, а не словами в промпте**. Сервер `required`, возможно, не исполняет (ADR-0143), но модель с одним тулом и жёсткой инструкцией зовёт его в ~97 % ходов. Мы пока не знаем, что даст честное `auto` — ячейки «тул без `required`» в бейк-оффе нет.

## 5. Расхождения с нашим адаптером

| # | Наш код | Что в доке / у них | Следствие |
|---|---|---|---|
| 1 | `MiniMaxProvider._CAPABILITIES`: `tool_choice=False` (`src/rob_box_llm/rob_box_llm/providers/minimax.py:336`), `json_mode=False`, `structured_output=False` (стр. 337-338) | Док: только `auto`. Верификатор: `required` работает (на уместном запросе). | Флаг честен для doc, спорен по верификатору. Решает замер (§7, шаг 0). |
| 2 | ADR-0143 (`docs/adr/0143-minimax-tool-choice-not-supported.md`, §2-§4): пробник слал `required` на «привет, как дела?» с тулом `get_time` — 9/9 без `tool_calls` | Верификатор проверяет `required` на запросе, который тул **требует** («погода в Пекине»). | Пробник проверил более трудный случай (принуждение вопреки смыслу), а команда — именно он: тул `command` нужен даже на «привет». Но модель тогда была иная (28.09); перемерить на 22.10 — дёшево. Вердикт «НЕТ» нельзя считать окончательным для «модель не вызывает тул, когда он уместен». |
| 3 | `caps.thinking=True` с комментарием «M3 is an always-thinking model» (`minimax.py:339`); `select_mode` пускает в `tool_choice` только при `not caps.thinking` (`src/rob_box_dialog/rob_box_dialog/degrade.py:61`) | M3 thinking отключается (две страницы из трёх); мы шлём `{"type":"disabled"}` по умолчанию (`minimax.py:114`, `_apply_thinking_policy` 442-473). | Даже если `tool_choice` станет `True`, MiniMax **не попадёт** в режим `tool_choice`: заявленное `thinking=True` его блокирует. Для M3 при выключенном thinking флаг должен быть `False` (через `capabilities_for`), для Flash — `True`. |
| 4 | Рассуждение в историю не возвращается: `LLMMessage` не имеет поля для него (`src/rob_box_llm/rob_box_llm/provider.py:148-165`), `_to_openai_messages` (`providers/deepseek.py:102-135`) шлёт только `content`/`tool_calls`; `complete()` кладёт ответ в `raw`, `reasoning_content` теряется (стр. 464-490); стрим его игнорирует (стр. 516-522, 625-650); `without_reasoning` вырезает `<think>` из `content` (`command_bridge.py:258, 264`). | Док: возвращать мысли целиком. Mini-Agent: возвращает всегда (`reasoning_details`). | Для ходов с `thinking=disabled` (M3, голосовой путь) безразлично — мыслей нет. Для Flash (мышление не выключить) и для ходов `TURN_REASONING` повтор после невалидной команды идёт без мыслей. В режиме команд это одно-два сообщения в рамках запроса; ущерб ограничен, но док прямо требует вернуть. |
| 5 | Тело не содержит `reasoning_split`. Мысли у M3 приходят в `content` как `<think>` (`harness/providers/reasoning.py:14-22`, живое 06.08), у Flash — в `reasoning_content`. | Mini-Agent и верификатор шлют `reasoning_split=true`; тогда `content` чист. | Нет явной договорённости о том, где мысли. Парсер команды спасает `without_reasoning`, но это заплатка. Явный `reasoning_split=true` убрал бы класс «`<think>` перед JSON». (Верификатор отмечает у `reasoning_split` перемежающийся HTTP 400, «BUG-13» — проверить замером.) |
| 6 | Шлём `temperature` из конфига (`docker/sound/config/voice_assistant/dialogue_node.yaml:71`, `0.7`; в тело — `providers/deepseek.py:376-377`) и `max_tokens` (стр. 378-379, `3000` в yaml:72) | Температура и top_p игнорируются (всегда 1 / 0.95). `max_tokens` помечен устаревшим, нужен `max_completion_tokens`. | Температура не рычаг: точность формы обеспечивают схема тула и валидатор, а не «понизить температуру». `max_tokens` пока работает; следить за предупреждениями устаревания. |
| 7 | Режим `json_in_text` для MiniMax: схема текстом в промпте (`command_bridge.py:76-77`), тулов нет | Модель обучена на схемах тулов; бейк-офф: 63 % против 83 % | Главная причина 52 invalid. |
| 8 | `shape()` в режиме `tool_choice` шлёт `tool_choice="required"` (`speech_channel.py:58-60`, `command_bridge.py:251`); адаптер MiniMax это бы пропустил в `kwargs["tool_choice"]` (`providers/deepseek.py:382-383`) | Док: только `auto`. | При включении MiniMax в этот режим «как есть» мы шлём параметр, который док не обещает. |
| 9 | Повтор после невалидной команды: assistant-сообщение со всеми вызовами + `tool`-результат на каждый `call.id` (`command_bridge.py:121-133`) | Верификатор: ответ нужен на каждый `tool_call_id`, иначе 400. minimax-code делает то же: ошибка валидации — tool-результат `isError`. | Совпадает. Аргументы в истории при оборванном JSON уйдут как `{}` (`_safe_json`) — это валидный JSON, 400 не будет. |
| 10 | Таймаут чтения 20 с (`minimax.py:134-136`), в yaml `timeout_s: 90` | У Flash p95 до 7 с в бейк-оффе. | Запас есть. |

Что **совпадает** с их практикой: OpenAI-совместимый клиент, `tools` с `type: function`, результат тула как `role: tool` с `tool_call_id`, `finish_reason: tool_calls`, thinking: для Flash глубина вместо `disabled` (`apply_model_thinking`, `minimax.py:250-271`), `base_resp` ловится в `_post_process_response` (`minimax.py:421-424`).

## 6. Рекомендации для перевода MiniMax в режим команд

1. **Режим запроса.** Не `json_in_text`. Добавить в таблицу режимов (`degrade.MODE_ORDER`, единственная таблица знания о режимах) честный режим «тул предложен, принуждения нет» (рабочее имя `tool_offered`): тул `command` в `tools`, `tool_choice` не отправляется (эквивалент `auto`), схема несётся тулом, а не промптом. Ответ без вызова тула идёт в тот же `reply_command` (он уже разбирает JSON в тексте) и в тот же единственный повтор. Это решение кода (ADR-0148): модель выбор не делает. Ожидание по бейк-оффу: невалидных 6–23 вместо 38–52; точные цифры без `required` — только замер. В `select_mode` порядок: structured_output, tool_choice, **tool_offered**, json_mode, json_in_text; MiniMax объявляет «умею тулы, принуждение не заявляю» — `tool_choice=False` остаётся, пока замер §7.0 не покажет обратное.
2. **`tool_choice`.** В запросе MiniMax — не отправлять `required`. Если замер §7.0 покажет, что `required` на 22.10 держится (≥ 98 % на «неуместных» запросах вроде приветствия), ADR-0143 переиздать (новая ревизия, не правка), поднять `tool_choice=True` только для модели, где это измерено, и убрать блокировку `thinking=True` (п.5.3).
3. **Capabilities.** `thinking` вычислять в `capabilities_for(model)`: M3 при политике `disabled` — `False`, M3.1-Flash — `True`. Комментарий «always-thinking» поправить (док: M3 переключаем).
4. **Thinking.** M3: оставить `disabled` (голосовой путь, +10–20 с по живому 06.08). Flash: `reasoning_effort=low` (уже есть). Явно слать `reasoning_split=true` и для Flash, и для ходов с мышлением — чтобы `content` был чист и не требовал вырезания `<think>`; проверить, что `reasoning_split` не даёт перемежающиеся 400 (§5.5).
5. **Что возвращать в историю.** Рамки: режим команд — один ответ плюс один повтор. Для повтора возвращать ответ модели полностью: `tool_calls` с валидным JSON в `arguments`, `tool`-результат на каждый `call.id` (уже так), для ходов с мышлением — ещё и `reasoning_content` (нужен дополнительный сквозной атрибут в `LLMMessage` и в `_to_openai_messages`). Межходовую историю мыслями не нагружать: док требует их только внутри цикла вызовов тулов.
6. **Ошибки.** Оставить текущую схему: нарушения словами как tool-результат с признаком ошибки + `COMMAND_RETRIES_MAX` (это то же самое, что делает minimax-code: «Validation failed … Received arguments»). Добавить в сообщение об ошибке то, что реально присылала модель (как у них), если `explain` ещё не делает. Нормализацию очевидного (`args: ""` → `{}`, `passed:false` → поле отсутствует) держать в `command.parse` — Mini-Agent с такой нормализацией не заморачивается, minimax-code использует мягкое приведение типов по схеме, это то же направление.
7. **Ответ из одних мыслей / пустой.** Известный вид брака (метрика верификатора). Убедиться, что пустой `content` без тулов после `without_reasoning` даёт `Invalid("no_command")` и уходит в повтор, а не в молчание. Не проверено.
8. **Схема `command`.** Порядок свойств не пересортировывать (в `knowledge.command_schema` не проверялся): для M3 порядок параметров — часть контракта.
9. **Параллельные вызовы.** Параметр `parallel_tool_calls` не слать. Но модель может вернуть несколько вызовов в одном ответе; как `parse` ведёт себя при двух вызовах `command`, я не проверял — внести в замер.
10. **Температура.** Убрать иллюзию: в `dialogue_node.yaml` ключ `minimax.temperature` ничего не делает (док). Можно оставить, но пометить комментарием; рычагом точности он не является.
11. **Эндпойнт.** Остаться на OpenAI-совместимом. Anthropic-совместимый «рекомендуется» докой и используется обоими их агентами, но преимущество для нашей задачи ничем не подтверждено, а переход — новый адаптер. Держать как запасной вариант, если OpenAI-путь по аргументам окажется заметно хуже.
12. **Кэш.** Автоматический с 512 токенов; добавка режима (`_with_note`) должна быть постоянной строкой в конце первого system-сообщения, чтобы префикс не менялся (сейчас так). Замерять `cached_tokens`.
13. **Фолбек.** В цепочке `deepseek,minimax` MiniMax — запасной (решение Шифу О5, #3746): в режиме команд он должен отвечать так же, как основной, иначе при отказе DeepSeek поведение робота меняется.

## 7. План замера на роботе (~20 ходов)

Метод — как в бейк-оффе 21.10: временный контейнер `docker run --rm` из образа `voice-assistant` на Sound Pi с env живого контейнера, ключ не печатать, `voice-assistant` не трогать; замок стенда (flock по pid, см. память); concurrency 1 и `--retry-429 4` (общий ключ с живым роботом, на параллели были 429). Инструмент: `scripts/dialog/adapter_benchmark.py` / `speech_channel_bakeoff.py`, набор `scripts/dialog/reference_set.yaml` (20 фраз, `--limit 20`), `--runs 3` (60 вызовов на вариант). Для режима `tool_offered` нужна правка бенчмарка (параметр режима, без `tool_choice`) — отдельная карточка, до замера не делается.

Шаг 0 (дёшево, ~60 вызовов): пересчёт ADR-0143. Один тул `command`/`get_time`, `tool_choice` ∈ {`auto`, `required`, именованный} × запрос ∈ {уместный («сколько времени»), неуместный («привет, как дела?»)}, по 10 раз, M3 и Flash. Мерить долю ответов с `tool_calls`, `finish_reason`, ошибки HTTP и `base_resp`.

Варианты (M3 — `thinking: disabled`; Flash — `reasoning_effort=low`):

| вариант | модель | режим | зачем |
|---|---|---|---|
| A | M3 | `json_in_text` (после 7a, нормализация включена) | контроль: сколько вернула нормализация (заявлено 49/52, не замерено) |
| B | M3 | тул + `required` | повтор бейк-оффа на живом промпте |
| C | M3 | тул без `tool_choice` | кандидат (`tool_offered`) |
| D | Flash | тул без `tool_choice` | кандидат на запас |
| E (по желанию) | M3 | C + `thinking: adaptive` + `reasoning_split=true` | потолок качества и цена по задержке |

Что мерить на каждый ход (из сырых строк, не «на глаз»):
- `delivered` / `invalid` с разбивкой по `reason` (`bad_type: args|passed`, `no_command`, `truncated`, `bad_json`);
- доля ответов **без вызова тула** (текстом) при тулах в запросе;
- повтор: сколько ходов ушло в повтор, сколько из них исправилось, итог после повтора (цель — `failed` ≤ 2 %; порог предложение, утверждает Шифу);
- `phantom` (в речи «готово» до события), `leak`, `double`;
- число `tool_calls` в одном ответе (параллельные), в том числе двойные `command`;
- `finish_reason` (`length` с мыслями), пустые ответы «только рассуждение»;
- задержка p50 / p95, `usage.prompt_tokens` и `prompt_tokens_details.cached_tokens`;
- `<think>` в `content` и наличие `reasoning_content` (по модели и по варианту), 400 от `reasoning_split`;
- `reasoning_content` в повторе: вариант E с возвратом мыслей и без (A/B внутри E, 20 повторов) — чтобы проверить пункт 5.5 про деградацию без мыслей, а не верить доке на слово.

Честно о границах: промпт бейк-оффа минимальный, не боевой; набор — черновик без утверждения Шифу (О1); 20 ходов × 3 прогона дают порядок величин, а не доверительный интервал; решения принимать по сырому JSONL, который приложить к карточке.

## 8. Что я не проверил

- Ни одного запроса к API MiniMax. Все утверждения о поведении модели — из документации, чужих тестов и бейк-оффа 21.10 (его я перечитал, не перезапускал).
- Документация противоречит себе по `tool_choice` и по поведению thinking у M3 (§3); какое утверждение верно на 22.10, неизвестно.
- Не смотрел, как `command.parse` / `validate` ведут себя при двух вызовах `command` в одном ответе, и как `knowledge.command_schema` упорядочивает свойства.
- Вес Anthropic-эндпойнта для качества тулов не измерялся.
- Страницы WebFetch отдавались через промежуточную модель-пересказчик; ключевые цитаты и номера строк в репозиториях я сверял по клонам, документные формулировки — нет (повторить чтение первоисточника перед тем, как ссылаться на них в ADR).
