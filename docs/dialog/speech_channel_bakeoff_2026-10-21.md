# Бейк-офф канала речи — 21.10.2026

ADR-0150 ревизия 5 (§4.4, В1, О4, A18), карточка #3697 (PR-1c эпика #3683). Меряется **транспорт слота речи**:
поле `speech` команды (`FieldChannel`, вариант A) против тула `speak_text` (`ToolChannel`, вариант B), на каждом
адаптере, 58 фраз × 3 прогона. Ничего в каналах не менялось — выбор и удаление проигравшего делает PR-7.

## Как прогнано (воспроизводимо)

Код: `scripts/dialog/speech_channel_bakeoff.py` + `scripts/dialog/adapter_benchmark.py` (реестр адаптеров), тесты —
`tests/dialog_adapters/`. Ключей локально нет: прогон шёл на Sound Pi (`ros2@10.1.1.22`) во временном контейнере
`docker run --rm` из образа `voice-assistant` с env живого контейнера (ключи не покидали робот и нигде не печатались;
`/tmp/bench` и `/tmp/va.env` после прогона удалены; `voice-assistant` не трогался).

```
python3 scripts/dialog/speech_channel_bakeoff.py --runs 3 --concurrency 4 \
    --raw full_raw.jsonl --json full.json              # все адаптеры
python3 scripts/dialog/speech_channel_bakeoff.py --providers minimax,minimax-flash --runs 3 \
    --concurrency 1 --retry-429 4 --raw mm_raw.jsonl --json mm.json   # повтор MiniMax (см. ниже)
python3 scripts/dialog/speech_channel_bakeoff.py --rescore merged_raw.jsonl --summary merged.json   # таблица из сырых
```

Сырые строки на прогон (2784 хода, 3584 вызова LLM) — `speech_channel_bakeoff_2026-10-21_raw.jsonl.gz`, сводка по
ячейкам — `..._summary.json` (рядом). Таблица ниже получена `--rescore` из них.

Что внутри прогона:
- набор — `scripts/dialog/reference_set.yaml` (**черновик, не утверждён Шифу, О1**), 64 фразы минус 6 tier1 (там LLM не
  зовётся) = **58** (карточка просила ≥ 60; 58 — все фразы, на которых LLM вообще вызывается);
- тулы-заглушки (имена из `expected.actions`, результат — `ok`), побочных эффектов нет; промпт **минимальный, не боевой**;
- `field` — один вызов; режим — по `select_mode(capabilities)` и принудительно `tool_choice` (проверить, не занижены ли
  флаги возможностей); `tool` — `speak_text` + `stay_silent` (молчание) + тулы действий, `content` не читается,
  после тулов второй вызов с заглушкой результата (цикл v1) — отсюда `llm_calls` ≈ 1,8;
- DeepSeek `v4-pro` гонялся с `thinking: disabled` (по умолчанию думает и отвечает HTTP 400 «Thinking mode does not
  support this tool_choice»).

## Метрики

ADR §4.4: `delivered` (ровно одна непустая речь; для фраз с действиями — хоть одно действие), `leak` (служебное в
тексте речи), `double` (два запроса речи), `phantom` (в речи «готово/включил» до события). Сверх ADR, чтобы видеть
причину недостачи: `lost` (речь ждали, слот пуст, текст ушёл мимо слота), `stray` (любой текст вне слота — канал его не
озвучивает), `spurious` (молчать, а есть речь/действие, либо речь/вопрос, а есть действие), `invalid` (канал не собрал
команду), `errors` (отказ провайдера — не качество канала). Делитель `delivered` — прогоны, где что-то ожидается (126).

## Таблица

Модель — из поля `model` ответа API, а не запрошенное имя.

| адаптер | модель (из ответа API) | канал | режим | прогонов | delivered | leak | double | phantom | lost | stray | spurious | invalid | errors | llm_calls | p50, мс | p95, мс |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| deepseek | deepseek-flash | field | tool_choice | 174 | 121/126 (96 %) | 0 | 0 | 1 | 0 | 0 | 44 | 0 | 0 | 1.00 | 1118 | 1535 |
| deepseek | deepseek-flash | tool | tool_choice | 174 | 110/126 (87 %) | 0 | 38 | 20 | 0 | 43 | 35 | 0 | 0 | 1.81 | 1914 | 2517 |
| deepseek-pro | deepseek-v4-pro | field | tool_choice | 174 | 115/126 (91 %) | 1 | 0 | 0 | 0 | 0 | 54 | 0 | 0 | 1.00 | 1492 | 2403 |
| deepseek-pro | deepseek-v4-pro | tool | tool_choice | 174 | 103/126 (82 %) | 0 | 12 | 50 | 0 | 1 | 33 | 0 | 0 | 1.80 | 2448 | 3628 |
| minimax | MiniMax-M3 | field | json_in_text | 174 | 80/126 (63 %) | 0 | 0 | 2 | 0 | 5 | 28 | 52 | 0 | 1.00 | 1458 | 5025 |
| minimax | MiniMax-M3 | field | tool_choice | 174 | 105/126 (83 %) | 0 | 0 | 0 | 0 | 8 | 54 | 23 | 0 | 1.00 | 2079 | 5130 |
| minimax | MiniMax-M3 | tool | json_in_text | 174 | 106/126 (84 %) | 0 | 17 | 14 | 1 | 22 | 33 | 0 | 0 | 1.78 | 3400 | 10231 |
| minimax-flash | MiniMax-M3.1-Flash | field | json_in_text | 174 | 82/126 (65 %) | 1 | 0 | 3 | 0 | 0 | 22 | 38 | 0 | 1.00 | 1913 | 7642 |
| minimax-flash | MiniMax-M3.1-Flash | field | tool_choice | 174 | 115/126 (91 %) | 0 | 0 | 10 | 0 | 1 | 35 | 6 | 0 | 1.00 | 2072 | 6956 |
| minimax-flash | MiniMax-M3.1-Flash | tool | json_in_text | 174 | 108/126 (86 %) | 0 | 4 | 19 | 0 | 57 | 36 | 0 | 0 | 1.82 | 3172 | 17911 |
| qwen-flash | qwen3.8-flash | field | json_in_text | 174 | 100/126 (79 %) | 0 | 0 | 2 | 0 | 0 | 16 | 14 | 0 | 1.00 | 5941 | 21986 |
| qwen-flash | qwen3.8-flash | field | tool_choice | 174 | 91/126 (72 %) | 0 | 0 | 3 | 0 | 10 | 14 | 5 | 0 | 1.00 | 4576 | 13928 |
| qwen-flash | qwen3.8-flash | tool | json_in_text | 174 | 110/126 (87 %) | 0 | 4 | 23 | 1 | 27 | 20 | 0 | 0 | 1.71 | 3953 | 11989 |
| qwen-max | qwen3.8-max | field | json_in_text | 174 | 105/126 (83 %) | 0 | 0 | 0 | 0 | 0 | 13 | 2 | 0 | 1.00 | 8096 | 29202 |
| qwen-max | qwen3.8-max | field | tool_choice | 174 | 92/126 (73 %) | 0 | 0 | 9 | 1 | 17 | 19 | 4 | 0 | 1.00 | 6374 | 19070 |
| qwen-max | qwen3.8-max | tool | json_in_text | 174 | 109/126 (87 %) | 0 | 5 | 39 | 0 | 20 | 12 | 0 | 0 | 1.69 | 6997 | 16339 |
| mimo | — | field, tool | — | — | **не прогнан:** `402 Insufficient account balance` ×174 в каждой из трёх ячеек | — | — | — | — | — | — | — | — | — | — | — |
| jev | — | — | — | — | **не прогнан:** System One API (choice/score), `POST /v1/chat/completions` → 404; речь/тулы им не выражаются | — | — | — | — | — | — | — | — | — | — | — |

«deepseek» запрашивался как `deepseek-chat` (дефолт кода до этого PR), «minimax» — `MiniMax-M3`; qwen — token-plan
`https://token-plan.maas.qwencloudapi.com/compatible-mode/v1`, имена моделей — из `GET /models` того же ключа.
`GET /models`: DeepSeek `["deepseek-flash","deepseek-v4-pro"]`; `deepseek-chat`, `deepseek-v4-flash` и `deepseek-flash`
в запросе обслуживаются **одной** моделью `deepseek-flash` (проверено тремя вызовами, см. «Raw»).

## Что видно

1. **Критерий ADR (100 % и ноль утечек/дублей/фантомов на всех адаптерах) не выполняет ни один канал.** Лучшее
   `delivered` — `field` на DeepSeek, 96 %.
2. **`double` — слабость тула, а не поля.** `field`: `double = 0` во всех ячейках (слот один по построению). `tool`:
   4–38 дублей — модель говорит `speak_text` ещё раз после результата тула (цикл v1); у DeepSeek flash 38 из 174.
   `stray` (текст в `content`, который `ToolChannel` игнорирует): 1–57 (у четырёх из шести ячеек ≥ 20) против 0–17 у поля.
3. **`llm_calls` и латентность:** `field` — 1,00 вызова, `tool` — 1,7–1,8; p50 DeepSeek 1118 против 1914 мс.
4. **Недостача `delivered` у `field` — структурная и закрываемая кодом, не выбором транспорта** (разбор invalid ниже).
5. **Недостача `delivered` у обоих каналов на фразах без контекста** (`r03` «Робот», `r27`, `r20`, `r18`, `r11`:
   действие ждут из бэклога/контекста, которого в замере нет) — одни и те же фразы падают в обоих каналах
   (DeepSeek: у `field` падают r27/r18/r20; у `tool` — те же плюс r03, r64, r13, r51). Это ограничение минимального промпта, не транспорта.
6. **Оговорка про `phantom` у `tool`:** вторая итерация цикла идёт ПОСЛЕ результата тула, и «готово» там честно; в
   ячейках `tool` `phantom` завышен именно этим (20–50). Для `field` речь и действие — один ход без обратной связи,
   поэтому `phantom` = 0–10 прямо показывает «сказал готово, не дождавшись события».
7. **`spurious` (молчать/без действия) высок везде (12–54):** набор содержит фразы без вейка и контекстные, а вейк-гейт и
   бэклог живут вне LLM. Сравнивать каналы между собой можно, абсолютное число — нет.

## Разбор invalid (вход A14: MiniMax json_in_text 18 invalid из 20)

Повтор `adapter_benchmark` (20 фраз, MiniMax-M3, json_in_text) на роботе: 7 delivered, 13 invalid, **все 13 =
`bad_type`**. Пример ответа модели: `{"speech":"…","actions":[…],"passed":false}`. Полный прогон (`detail` в сырых):

| ячейка | invalid | причины |
|---|---|---|
| minimax / field / json_in_text | 52 | **49 × `bad_type: passed`** (булево вместо enum-строки), по 1 × `actions[]` (строка вместо объекта), `args` (массив), `no_command` (`"passed":not_addressed` без кавычек) |
| minimax-flash / field / json_in_text | 38 | 28 × `bad_type: passed`, 9 × `unknown_field` (`field`, `tool`/`args` на верхнем уровне), 1 × `actions[]` |
| minimax / field / tool_choice | 23 | 17 × `bad_type: args` (`"args": ""` вместо объекта), 6 × `no_command` (отказ текстом без тула — `tool_choice` не удержан) |
| minimax-flash / field / tool_choice | 6 | 5 × `bad_type: args`, 1 × `no_command` |
| qwen-flash / field / json_in_text | 14 | 14 × `bad_type` |
| qwen-flash, qwen-max / field / tool_choice | 5 / 4 | `no_command` |

Причины двух видов: (а) режим `json_in_text` не передаёт модели схему — только слова в системном промпте, поэтому
`passed` пишут булевым, а `parse` строгий; (б) `args` пустой строкой. Оба — недостача формы, которую закрывает код по ADR-0148:
узкая схема в инструкции режима + нормализация очевидных пустых значений (`passed: false` ≡ поле отсутствует,
`args: ""` ≡ `{}`) в `command.parse` (PR-1, не здесь). Сколько это вернёт — **не замерено** (парсер в этом PR не менялся).

## Что НЕ проверено (честно)

- Не боевой промпт и не боевой контекст (нет бэклога, вейк-гейта, истории); `reference_set.yaml` — черновик без
  утверждённой разметки Шифу (О1). Абсолютные числа `delivered`/`spurious` — оценка снизу, сравнение каналов — относительное.
- Тулы-заглушки; `SpeechRequest` и владелец речи (PR-3) в замере не участвуют — `delivered` считается по слоту канала.
- Метрики считает код по структуре и по маркерам (`SERVICE_WORDS`, `CLAIM_WORDS`) — это эвристика, не оценка смысла.
- MiMo не прогнан (402, баланс). Jev — не чат-модель, прогнать канал речи нельзя. Qwen: режим `tool_choice` принудительный
  (флаги возможностей адаптера пока не заявлены), в `auto` он идёт как `json_in_text`.
- Первый полный прогон MiniMax получил `429 Token Plan rate limit` на ~60 % вызовов (шесть ячеек параллельно с общим
  ключом) — **эти числа отброшены**, MiniMax прогнан заново отдельно (concurrency 1, `--retry-429 4`), `errors = 0`.
  Остальные адаптеры — из первого прогона (`errors = 0`). Прогон один раз по 3 прогона на ячейку, без повторов «на всякий случай».
- Сравнение при одинаковой латентности некорректно для Qwen/MiniMax (общий ключ с живым роботом, сеть 21.10).

## Raw

- сырой JSONL (строка на прогон: ячейка, id фразы, `speeches`, `actions`, `leaks`, `invalid`/`detail`/`head`, `llm_calls`,
  `ms`, `served`) и сводка: `docs/dialog/speech_channel_bakeoff_2026-10-21_raw.jsonl.gz`, `..._summary.json`;
- проверка имён DeepSeek на роботе (`extra_checks.py`, вывод без правок):
  `deepseek requested deepseek-chat -> served deepseek-flash`, `deepseek-v4-flash -> deepseek-flash`, `deepseek-flash -> deepseek-flash`;
- `GET /models` (одним запросом с робота): MiniMax — `MiniMax-M3.1-Flash, MiniMax-M3, MiniMax-M2.7, …`; MiMo —
  `mimo-v2.5-pro, mimo-v2.6-flash, mimo-v2.6-pro, …`; Qwen — `qwen3.8-max, qwen3.8-flash, qwen3.7-max, …, glm-5.3, deepseek-v4.1-flash`; Jev — `[]`.
