# Локальный диалог — чат с мозгом робота без ROS2

Диалоговая система РОББОКСа живёт в `rob_box_harness` и намеренно не
знает про ROS2: `DialogCore` — это LLM + каталог инструментов + память +
машина состояний, а `dialogue_node.py` только подключает её к топикам.
Здесь лежит **вторая оболочка** вокруг того же ядра — терминальный чат.

```bash
# 1. ключи
cp scripts/dialogue/.env.example scripts/dialogue/.env   # и вписать ключ

# 2. запуск
scripts/dialogue/run.ps1          # Windows (PowerShell)
./scripts/dialogue/run.sh         # Linux / macOS / git-bash
```

```
› роббокс, привет
🤖 Привет! Чем помочь?  [anim:happy]
[ход 1 · 2.3s · state=IDLE · tools: speak_text]
```

## Что здесь ровно то же, что на роботе

| | локально | на роботе |
|---|---|---|
| системный промпт | `src/rob_box_voice/prompts/master_prompt_compact.txt` | он же |
| цепочка LLM | `minimax,deepseek` + health-aware fallback | она же (`dialogue_node.yaml`) |
| дефолты провайдеров | `rob_box_harness.providers.catalog` | оттуда же (`_LLM_PROVIDER_REGISTRY`) |
| каталог инструментов | 51 tool из `rob_box_core.tool_catalog` | он же, через `/mcp/execute` |
| переходы состояний | `WAKE_WORD → STT_RESULT → DIALOGUE_END` | те же |
| память | `SQLiteVoiceMemory` | она же (`/data/harness_voice.db`) |

## Что отличается

**Инструменты симулируются.** Мотора, колонки и LED-матрицы на ноутбуке
нет, поэтому `execute()` подделан ([`local_tools.py`](local_tools.py)):

* `speak_text` — печатается в терминал: на роботе робот говорит **только**
  через этот тул, значит в текстовом чате это и есть его реплика;
* `memory_save` / `memory_context` / `memory_search` — выполняются
  по-настоящему, поверх той же `MemoryStore`, что и у `DialogCore`
  («запомни, что…» переживает перезапуск);
* `get_current_time` — настоящие часы;
* остальные — `{"status": "ok", "simulated": true}`. Ответ намеренно
  успешный: цель — гонять промпт и выбор инструментов, а ошибка увела бы
  модель в её ретрай-ветки.

Модель при этом **не знает**, что она не на железе: `<system_context>`
собирается той же формы, что и на роботе. Так и задумано — иначе мы бы
тестировали другой промпт.

**Нет STT/TTS/VAD, barge-in, DJ-режима, спикер-биометрии и планировщика
задач** — всё это живёт в `dialogue_node`, не в ядре.

## Флаги

```bash
scripts/dialogue/run.ps1 --providers deepseek        # одна модель, без fallback
scripts/dialogue/run.ps1 --providers deepseek --model deepseek-reasoner   # другая модель
scripts/dialogue/run.ps1 --wake-word                 # требовать «роббокс, ...»
scripts/dialogue/run.ps1 --no-tools                  # чистый чат без tool-calls
scripts/dialogue/run.ps1 --db :memory:               # не сохранять диалог
scripts/dialogue/run.ps1 --speaker-name Кирилл       # робот «узнал» вас по голосу
scripts/dialogue/run.ps1 --once "привет" --once "как дела"   # сценарий и выход
scripts/dialogue/run.ps1 --debug                     # трассировка запросов к LLM
```

`--model` и `--base-url` относятся к **primary** — первому провайдеру в
`--providers`. Фолбеку они не передаются (у него свой API и свои модели),
и если primary не собрался, скрипт об этом предупредит.

`--debug` включает `ROBOT_LLM_VERBOSE=1` — тот самый дамп полного
контекста в stderr, который на роботе виден в `docker logs`. Без него он
выключен: 26 КБ промпта на каждый ход в интерактивном чате нечитаемы.

Полный список — `--help`.

## Команды в чате

| | |
|---|---|
| `/help` | справка |
| `/exit`, `/quit`, `/q`, Ctrl+C | выход |
| `/reset` | новая сессия: очистить историю диалога |
| `/state` | состояние машины состояний, провайдер, файл памяти |
| `/tools [фильтр]` | каталог инструментов, который видит LLM |
| `/history [N]` | последние N ходов из памяти |
| `/facts` | что робот о вас запомнил |

## Ключи

`chat.py` читает `scripts/dialogue/.env`, затем корневой `.env`, затем
берёт то, что уже есть в окружении — **экспортированная переменная
всегда важнее файла**, чтобы CI не подхватил локальные ключи. Файл
`scripts/dialogue/.env` игнорируется гитом (правило `.env` в корневом
`.gitignore` срабатывает на любой глубине) — проверить можно так:

```bash
git check-ignore -v scripts/dialogue/.env
```

Достаточно одного ключа: цепочка `minimax,deepseek` просто пропустит
провайдера, для которого ключа нет, и скажет об этом при старте.

## Требования

* **Python 3.12+** — `rob_box_core` объявляет dataclass-поле со значением
  `mappingproxy`, а это разрешено только начиная с 3.12. Лаунчеры сами
  берут `.venv` из корня репозитория, если он есть.
* Пакеты: `openai`, `httpx` (они уже в зависимостях `rob_box_llm`).
  ROS2 **не нужен** — `rclpy` здесь не импортируется вообще.

```bash
# если .venv в корне ещё нет
python3.12 -m venv .venv
.venv/Scripts/pip install -e src/rob_box_core -e src/rob_box_llm -e src/rob_box_harness
```
