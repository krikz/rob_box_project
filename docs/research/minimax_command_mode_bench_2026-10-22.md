# MiniMax в режиме команд — замер 22.10.2026

Карточка #3698 (PR-7, ч.7d, вариант А Шифу), эпик #3683. Продолжение исследования
`docs/research/minimax_tool_calling_2026-10-22.md` (§7 — план замера). Решение Шифу 22.10 по итогам: «результат уже
есть; фаворит DeepSeek (быстрый), MiniMax как запасной в режиме команд оправдан по скорости и качеству»; замеры
остановлены, варианты thinking/`reasoning_split` и M2.7/M2.5 в мосте **не прогнаны** (см. «Что не проверено»).

## Как прогнано

Временный контейнер `docker run --rm` из образа `voice-assistant-humble-dev` на Sound Pi (`ros2@10.1.1.22`), ключи —
из env живого `voice-assistant` через `docker inspect` (в argv и файлы не попадали, не печатались), живые контейнеры
не трогались; concurrency 1 на ключ, `--retry-429 4` (ни одного 429 не случилось). После прогона контейнеры и
`/tmp/wmm` на Pi удалены. Код замера — ветка этого PR.

**Шаг 0 — держит ли сервер `tool_choice`** (`scripts/research/minimax_tool_choice_probe.py`, сырой HTTP без адаптера):
один тул `get_time`, `tool_choice` ∈ {`auto`, `required`, именованный, не отправлять} × запрос {уместный «Сколько сейчас
времени?», неуместный «привет, как дела?»} × модель × 10. Thinking — как у робота (M3 `disabled`, Flash
`reasoning_effort=low`, M2.x без поля).

```
python3 scripts/research/minimax_tool_choice_probe.py --list-models
python3 scripts/research/minimax_tool_choice_probe.py --models MiniMax-M3,MiniMax-M3.1-Flash,MiniMax-M2.7,\
MiniMax-M2.7-highspeed,MiniMax-M2.5,MiniMax-M2.5-highspeed,MiniMax-M2.1,MiniMax-M2.1-highspeed,MiniMax-M2 \
    --n 10 --retry-429 4 --raw step0.jsonl
```

**Мост режима команд** (`scripts/dialog/adapter_benchmark.py --cells`): ход ровно как у робота — `CommandModeLLM.shape`
(боевой `master_prompt_compact.txt` + `mode_note`, схема `knowledge.command_schema`, тулы каталога из разметки набора),
`CommandModeLLM.answer` (`judge` с валидатором по срезу тулов, один повтор с нарушениями), `temperature 0.7`,
`max_tokens 3000` — как в `dialogue_node.yaml`. Набор — `scripts/dialog/reference_set.yaml` (черновик, О1): 75 фраз без
tier1 × 3 прогона = 225 ходов на ячейку, из них 177 с ожиданием (48 — «молчать»).

```
python3 scripts/dialog/adapter_benchmark.py --cells deepseek/tool_choice --limit 0 --runs 3 --retry-429 4 \
    --system-file src/rob_box_voice/prompts/master_prompt_compact.txt --raw ds.jsonl
python3 scripts/dialog/adapter_benchmark.py --cells minimax:MiniMax-M3/tool_offered,minimax:MiniMax-M3/tool_choice,\
minimax:MiniMax-M3.1-Flash/tool_offered,minimax:MiniMax-M3/json_in_text,minimax:MiniMax-M3.1-Flash/tool_choice,\
minimax:MiniMax-M3.1-Flash/json_in_text --limit 0 --runs 3 --retry-429 4 --system-file … --raw mmA.jsonl
python3 scripts/dialog/adapter_benchmark.py --rescore bridge.jsonl --runs 3        # таблица ниже — из сырых
```

Сырые строки: `minimax_command_mode_bench_2026-10-22_step0.jsonl.gz` (720 запросов),
`minimax_command_mode_bench_2026-10-22_bridge.jsonl.gz` (1575 ходов, строка на ход: вердикт первого ответа и итога,
причина, речь, действия, по каждому ответу — finish, вызовы, `<think>`/`reasoning_content`, токены, мс).

**Запросов всего:** MiniMax — 2329 (шаг 0: 720, из них 240 — HTTP 404 у снятых моделей; мост: 1600 + 9 в прерванном
запуске); DeepSeek — 325 (мост 250 + 72 в первом запуске на 20 фразах + 3 пробных). 429 — 0.

## Шаг 0: `tool_choice`

Доля ответов с `tool_calls` из 10 (ошибок HTTP — 0, кроме снятых моделей).

| модель | запрос | auto | required | именованный | не отправлять |
|---|---|---|---|---|---|
| MiniMax-M3 | неуместный | 0 | **0** | 0 | 0 |
| MiniMax-M3 | уместный | 8 | 10 | 8 | 9 |
| MiniMax-M3.1-Flash | неуместный | 4 | 3 | 3 | 0 |
| MiniMax-M3.1-Flash | уместный | 10 | 10 | 10 | 10 |
| MiniMax-M2.7 | неуместный | 1 | 2 | 2 | 4 |
| MiniMax-M2.7-highspeed | неуместный | 5 | 2 | 4 | 1 |
| MiniMax-M2.5 | неуместный | 4 | 4 | 6 | 5 |
| MiniMax-M2.5-highspeed | неуместный | 3 | 5 | 3 | 2 |
| все M2.7 / M2.5 | уместный | 10 | 10 | 10 | 10 |
| MiniMax-M2.1, M2.1-highspeed, M2 | любой | HTTP 404 «The model … is no longer available (2069)» ×80 у каждой (в `GET /models` они есть) |||||

Вывод: **`required` и именованный тул сервер не исполняет ни на одной модели** (на неуместном запросе 0–6 из 10, как
у `auto`) — ADR-0143 на 22.10 подтверждён, `tool_choice=False` у MiniMax честен. У M3 `required` лишь слегка
подталкивает на уместном запросе (10 против 8–9).

Мысли (80 ответов на модель): M3 с `thinking: disabled` — 0 `<think>`, 0 `reasoning_content` (не думает); M3.1-Flash —
`reasoning_content` в 19, `<think>` 0; M2.7 / M2.5 (и `-highspeed`) — `<think>` в `content` в 25–46 (поле thinking
игнорируют). Отсюда `capabilities_for`: `thinking=False` только у M3 с политикой `disabled`.

## Мост режима команд

`delivered` — по разметке набора (действие для `actions`, речь/вопрос для `speech_only`/`ask`, `passed` для «молчать»);
делитель — 177 ходов с ожиданием. «1-й» — первый ответ модели, «итог» — после единственного повтора с нарушениями.
`phantom` — регекс «готово/включил/сделал…» в речи без действий (эвристика бейк-оффа PR-1c).

| модель (из ответа API) | режим | delivered 1-й | delivered итог | invalid 1-й (главное) | invalid итог | повторов / исправил | phantom (регекс) | без тула, ответов | p50 / p90, мс | вызовов LLM |
|---|---|---|---|---|---|---|---|---|---|---|
| deepseek-flash | tool_choice | 114 (64 %) | 132 (75 %) | 25 (19 speech_with_actions) | 2 | 25 / 23 | 3 | 0 / 250 | **1599 / 3351** | 250 |
| MiniMax-M3 | **tool_offered** | 106 (60 %) | 135 (76 %) | 44 (16 bad_args, 11 speech_with_actions, 7 tool_not_available) | 5 | 44 / 39 | 1 | 93 / 269 | 2572 / 6983 | 269 |
| MiniMax-M3 | tool_choice (required) | 107 (60 %) | 133 (75 %) | 53 (20 bad_args, 17 speech_with_actions) | 9 | 53 / 44 | 2 | 98 / 278 | 2193 / 7570 | 278 |
| MiniMax-M3 | json_in_text | 114 (64 %) | 141 (80 %) | 49 (26 speech_with_actions, 7 unknown_field) | 5 | 49 / 44 | 1 | 228 / 274 | 2553 / 9641 | 274 |
| MiniMax-M3.1-Flash | tool_offered | 127 (72 %) | 141 (80 %) | 34 (11 speech_with_actions, 10 bad_args) | 13 | 34 / 21 | 0 | 7 / 259 | 4390 / 15823 | 259 |
| MiniMax-M3.1-Flash | tool_choice (required) | 127 (72 %) | 142 (80 %) | 34 (13 bad_args, 11 speech_with_actions) | 12 | 34 / 22 | 1 | 7 / 259 | 4017 / 12570 | 259 |
| MiniMax-M3.1-Flash | json_in_text | 119 (67 %) | 137 (77 %) | 36 (12 speech_with_actions, 7 bad_args) | 11 | 36 / 25 | 1 | 116 / 261 | 3906 / 14597 | 261 |

Прочее из сырых: ошибок провайдера — 0 во всех ячейках; два вызова `command` в одном ответе — по 1 разу у Flash
(`tool_offered`, `tool_choice`) — `command.parse` даёт `several_commands`, мост уходит в повтор; токены на вызов —
вход ≈ 9,4–9,6 тыс. (из них из кэша в среднем 7,0–9,1 тыс.), выход DeepSeek 151, M3 92–124, Flash 210–222. Все 10 срабатываний `phantom`
просмотрены руками — **все ложные** (сказка, стих, вопрос «что ты хочешь, чтобы я сделал»); настоящих «готово» без
действия — 0.

## Что из этого следует

1. **`required` для MiniMax бесполезен** и в мосте: M3 `tool_choice` ≈ `tool_offered` (75 % против 76 %), повторов
   больше (53 против 44). Принуждение не слать — режим `tool_offered`.
2. **M3 в режиме команд держит уровень DeepSeek по качеству** (итог 76 % против 75 %, 1-й ответ 60 % против 64 %),
   но медленнее: p50 +1,0 с, p90 ×2 (6,98 против 3,35 с). Flash качественнее (80 %), но p50 4,4 с и p90 15,8 с — для
   голоса не годится; запасной — M3 (решение Шифу).
3. **Честно: `json_in_text` у M3 не хуже `tool_offered`** (80 % против 76 %; разница 7 ходов из 177, ≈ 0,8 σ — в пределах
   шума трёх прогонов). Бейк-офф PR-1c (63 % против 83 %) мерил до нормализации `passed:false`/`args:""` в
   `command.parse` (PR-7a): нормализация закрыла большую часть разрыва. За `tool_offered` — меньший p90 (6,98 против
   9,64 с) и меньше невалидных первых ответов (44 против 49); против — модель в 93 из 269 ответов всё равно пишет JSON
   текстом (парсер его берёт). Выбор `tool_offered` — решение Шифу 22.10, данные его не опровергают и не доказывают.
4. Главная причина недобора у всех — `speech_with_actions` (правило §4.1: речь рядом с действием — нарушение) и
   `bad_args`; повтор исправляет 62–92 % (DeepSeek 23/25, M3 39/44–44/53, Flash 21/34–25/36). У «молчать» delivered 0–4 из 48 у всех: боевой промпт не учит `passed`,
   вейк-гейт живёт в коде до LLM — абсолют не показатель.

## Что не проверено

- Варианты `thinking: adaptive`, `reasoning_split=true`, M2.7 / M2.5 в мосте — **не прогнаны** (замеры остановлены
  решением Шифу). Влияние `reasoning_split` на перемежающиеся 400 (BUG-13 верификатора) — не проверено.
- Возврат `reasoning_content` в историю повтора — не реализован и не мерился (у M3 с `disabled` мыслей нет — для
  запасного не нужно; для Flash — задача «потом»).
- `temperature`: что M3/Flash её игнорируют — из документации, замером не проверено (в yaml помечено).
- Промпт боевой, но без снимка `<system_context>`, истории и бэклога; набор — черновик без утверждения Шифу (О1);
  3 прогона дают порядок величин, разница 3–4 п.п. между ячейками — в пределах шума.
- Живой ход на роботе (DeepSeek лёг → MiniMax отвечает командой) — не проверен, ждёт «стенд свободен».
