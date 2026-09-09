# Хендофф — волна 3 дедупликации, 2026-08-30

> **Кто писал:** Claude Opus 5. Сессия с доступом на робота (katana →
> vision 10.1.1.21), продолжение
> [`2026-08-29-tool-catalog-dedup-handoff.md`](2026-08-29-tool-catalog-dedup-handoff.md).
> **База:** `develop` @ `0a9769da`.
> **Честность (ADR-0018):** каждое утверждение — либо вывод команды,
> приведённой рядом, либо ссылка на файл со строкой. Что не прогонялось —
> помечено явно.

---

## 1. Что закрыто в предыдущей сессии

Подробности — в хендофе от 29.08, §§6–8. Коротко:

| | было | стало |
|---|---|---|
| `src/rob_box_voice/test/unit` | 1418 passed / 18 failed | **1442 / 0** |
| `src/rob_box_harness/test/test_dialog_core.py` | 64 / 3 failed | **71 / 0** |
| `G: Run Tests` в CI | 20 failed / 1414 | **зелёный** |
| `mcp_server` на роботе | падал на импорте, 336 трейсбеков за 40 мин | 51 тул, 3/3 вызова успешно |

Ключевое, что стоит знать новому воркеру:

- **`ToolExecutionType` импортировался не из того модуля** (`731623c4`) —
  из-за этого `mcp_server` не стартовал вообще, и все «таймауты
  инструментов» в логе были следствием одной строки. Предохранитель:
  `test_relative_imports_resolve`.
- **`[CRITICAL]`-ретраи писались в историю как реплики пользователя**
  (`eab8518e`) — модель читала транскрипт, где её отчитывают, и
  переставала звать тулы. Появился `process_input(is_synthetic=True)`.
- **`<system_context>` стоял перед историей** — переехал вплотную к
  текущей реплике.
- **`wake_words` жил в девяти местах тремя разными списками**
  (`cd551181`) — теперь один, в
  `rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS`.
- **Монолит `voice_assistant.yaml` удалён** (`2b0bba11`) — ADR-0004
  объявил его несуществующим ещё в #1004, а пакет продолжал его ставить.

---

## 2. 🔴 Память: спека и код расходятся

Это самое важное для следующей волны, потому что объясняет живой сбой.

**Канон — [`docs/design/dialogue-mode-spec-2026-08-28.md`](../design/dialogue-mode-spec-2026-08-28.md) §2.4:**

> - Тёрны в БД **не храним**. Окно диалога — временное (в памяти).
> - В долгую память робот пишет **только значимые события/факты**, а не
>   каждый тёрн.
> - Факты хранятся **по идентифицированному пользователю**.
> - Если с роботом ~3 минуты никто не говорит — короткая сводка в долгую
>   память, что делали и с кем.
> - Память **честная**: старые тёрны не протекают в новый вопрос.

**Что в коде:** тёрны пишутся каждым ходом и читаются обратно каждым
ходом — `dialog_core.py` (`append_turn` в ветке персиста, `load_recent`
в `_resolve_history`), таблица `turns` в `memory/sqlite_voice.py`.
Ни `persist_turns`, ни журнального scope в коде нет:

```bash
rg -n "persist_turns|scope=\"journal\"" src/
```

**Чем это обошлось.** Живое окно с vision 29.08 (20 ходов):

```
user      | [Speaker:unknown] [CRITICAL] В прошлом цикле ты НЕ вызвал…
assistant | Менеджер не отвечает. Попробуй ещё раз чуть позже.
user      | загрузи и включи трек тисбит
assistant | Менеджер не отвечает. Попробуй ещё раз через секунду.
```

Четыре хода из двадцати — извинения за тулы, которые тогда были мертвы,
два — `[CRITICAL]`-ругань. Модель отвечала «Менеджер не отвечает» на
«какие звуки ты умеешь проигрывать» — фразы, которой в репозитории нет:
она выучила её из собственной истории. `eab8518e` убрал синтетические
ходы, но **сам механизм «тёрны переживают ход» остался**, и пункт
«память честная» держится только на этой заплатке.

**Карточки уже написаны** — [`2026-08-29-wave2-worker-prompts.md`](2026-08-29-wave2-worker-prompts.md):
W5-1 (не хранить тёрны, `persist_turns=false`), W5-2 (журнал при тишине,
`scope="journal"`), W5-3 (факты в профиль каждому участнику), W5-4 (баг
«один голос — два профиля»). Ни одна не сделана.

⚠️ **Спека в гите есть** (`c95b31af`), а вот планы волны 2 — нет:
`docs/plans/2026-08-29-wave2-worker-prompts.md` и
`docs/plans/2026-08-29-dialogue-mode-wave2-plan.md` untracked. Именно в
них лежат решения Шифу Q1–Q14 и промты W4/W5. Закоммитить раньше, чем
брать по ним карточку — иначе воркер их просто не увидит.

**Про `/data/harness_voice.db`.** В прошлой сессии база переехала на
смонтированный том (`718e7c80`), потому что `~/.rob_box/voice.db` — это
`/root` внутри контейнера, тома там нет, и всё умирало с контейнером.
Это остаётся верным и после W5-1: в том же файле лежат `facts`,
`waypoints`, `faq_items`, `event_profile`, которые по спеке обязаны
переживать перезапуск. Уходит только таблица `turns`. Решение Шифу (Q3):
накопленную БД можно просто удалить, миграция не нужна.

---

## 3. Найденные дубли

Скан: обход `src/**/*.py` через `ast`, сбор top-level `def`/`class`/
`UPPER_CASE`-констант, группировка по имени, сравнение нормализованного
исходника (без комментариев и докстрингов) через `difflib`.

### 3.1. 🔴 `rob_box_core` — три абстракции без единого потребителя

Пакет объявлен как «ROS-free contracts shared by the harness packages».
Наружу используются **два модуля из пяти**:

| Модуль | Кто импортирует снаружи пакета |
|---|---|
| `ports.py` | `rob_box_harness/executors/{ros_mcp,local,core_adapter}.py` ✅ |
| `tool_catalog.py` | `rob_box_harness/core/tool_registry.py`, `mcp_tools` ✅ |
| `clock.py` | никто |
| `memory.py` | никто |
| `dialogue_state.py` | никто |

`from rob_box_core import ...` не встречается нигде за пределами самого
пакета — только импорты подмодулей `ports` и `tool_catalog`.

И они **уже разъехались** с живыми аналогами в harness:

| | `rob_box_core` | `rob_box_harness` |
|---|---|---|
| `MemoryStore.append_turn` | `(role, content, *, scope, timestamp)` | `(scope, turn)` |
| `MemoryStore.load_recent` | `(limit, *, scope)` | `(scope, *, limit)` |
| `DialogueStateMachine` | `transition_to` / `can_transition_to` / `on_enter` / `on_exit` | `on_event` / `on_user_input` / `check_silence_timeout` |
| `Clock` | `now` / `monotonic` / `sleep` | те же три |

У `MemoryStore` **позиционные аргументы поменяны местами** — код,
написанный против одного протокола, против второго не просто упадёт, а
может молча передать limit туда, где ждут scope. У
`DialogueStateMachine` общего — только `__init__`, `reset`, `state`: это
два разных проекта под одним именем.

Это тот же незавершённый переезд, что P3 в прошлом хендофе
(`ToolProvider`), только там хотя бы есть мост
(`executors/core_adapter.py`), а здесь второй экземпляр просто лежит и
расходится.

**Нужно решение Шифу** — см. §4.

### 3.2. Дословные копипасты

| Что | Где | Сходство |
|---|---|---|
| `ignore_stderr` | `audio_node.py:31`, `sound_node.py:36`, `tts_node.py:147` | **3 идентичные копии** |
| `_wait_future` | `tools/navigation.py:32`, `tools/system.py:25`, `tools/mapping.py:30` | **3 копии одного тела**, у mapping короче докстринг |
| `RetryPolicy` | `providers/deepseek.py:99`, `providers/minimax.py:128` | **идентичны**, обе живые |
| `ToolCallAccumulator` | `async_executor.py:52`, `core/tool_call_accumulator.py:12` | 0.91, **обе живые** |

`ToolCallAccumulator` — особый случай: обе копии в одном пакете и обе
используются. `core/__init__.py:3` экспортирует одну, `llm_adapter.py:29`
импортирует другую из `async_executor`.

### 3.3. Параллельные подсистемы

| Что | Где | Комментарий |
|---|---|---|
| `TTSProviderRegistry`, `TTSProviderFactory`, `register_builtin_tts_providers` | `rob_box_harness/tts/registry.py`, `rob_box_llm/tts_provider_registry.py` | два реестра, каждый экспортится своим `__init__` |
| `start_metrics_server`, `record_telegram_message` | `rob_box_telegram/observability.py`, `rob_box_voice/observability/metrics.py` | 0.90 / 0.69 |
| `MinimaxMusicClient` | `mcp_tools/core/` (187 строк), `voice/core/` (301) | сильно разошлись |
| `GeneratedMusicLibrary` | `mcp_tools/core/` (327), `voice/core/` (492) | сильно разошлись |

Дублей строковых констант, кроме уже схлопнутых `wake_words`, скан не
нашёл.

### 3.4. Из прошлого хендофа, всё ещё открыто

P2 (`DialogHarness` без точки входа), P3 (два `ToolProvider`), P4 (два
валидатора параметров), P5 (e2e-мок `MOCK_MCP_TOOLS`), P6 (декомпозиция
`llm_adapter` / `async_executor`), P7 (мёртвые модули), **P8
(`dialogue_node` — 4911 строк вместо «thin shell» из плана 06-02)**.

---

## 4. Вопросы к Шифу (блокируют карточки)

1. **`rob_box_core`: доводить миграцию или снести?** Три модуля
   (`clock`, `memory`, `dialogue_state`) не используются никем и уже
   разошлись с живыми. Либо переносим harness на них (большой заход,
   ломает сигнатуры), либо удаляем и оставляем в пакете только `ports`
   и `tool_catalog`. Карточку без этого решения не написать.
2. **`faq_mode_enabled: true`** в
   `docker/vision/config/voice_assistant/dialogue_node.yaml` при пустом
   `faq_event_config_file`. Сейчас это no-op: `_load_event_profile`
   возвращает `{}`. Выключить флаг или положить профиль? Плюс у
   `mcp_server` отдельный переключатель — переменная окружения
   `FAQ_MODE_ENABLED` (сейчас off), и они могут разойтись.
3. **9 параметров в `voice_assistant_test.yaml`, которых нода не
   объявляет** и потому молча игнорирует: `provider` (нода читает
   `llm_providers`), `streaming` (`llm_streaming`), `system_prompt`
   (`system_prompt_file` — и это путь, а не текст), `enable_fallback`,
   `max_iterations`, `silence_words`, `internet_available`,
   `max_history_messages`, `history_window_tokens`. Редирект на локальную
   Ollama жив — `base_url`/`api_key`/`model`/`temperature`/`max_tokens`
   объявлены и работают. Переименование остальных изменит поведение
   e2e — это решение, а не правка.

---

## 5. Промт воркеру: W6-1 — четыре дословные копипасты

Шапка §0 из [`2026-08-29-wave2-worker-prompts.md`](2026-08-29-wave2-worker-prompts.md)
обязательна — вставить перед задачей.

```
Проект: rob_box_project, ветка develop. ROS 2 Humble, Python.

ПРАВИЛА (обязательны):
1. ADR-0018 «честность»: не пиши «работает» / «зелёно» без raw-вывода прогона.
   Если прогнать не можешь (нет стенда/железа) — так и напиши в PR и в шапке
   изменённого файла. Выдуманный PASS хуже честного FAIL.
2. ADR-0013 «инкрементально»: одна карточка = один PR. Не расширяй скоуп.
   Нашёл смежную проблему — заведи отдельную задачу, не чини по дороге.
3. TDD: сначала тест (падающий), потом реализация. Тесты рядом с существующими
   в том же пакете (src/<pkg>/test/...), стиль копируй с соседних файлов.
4. Комментарии и docstring — на русском, как в окружающем коде. Ссылайся на
   issue/ADR в комментарии там, где чинишь неочевидное.
5. Ничего не коммить и не пушить, пока не спросишь. Покажи diff.
6. Не трогай прод-конфиги docker/vision/config/**, если это не сказано в задаче.

Проверка перед сдачей:
   pytest src/rob_box_voice/test -q
   pytest src/rob_box_mcp_tools/test -q
   pytest src/rob_box_harness/test -q

Задача W6-1: убрать четыре дословные копипасты. Одно объявление на каждую.

Контекст: скан по `src/**/*.py` (ast + difflib по нормализованному исходнику)
нашёл четыре функции/класса, продублированных дословно. Это НЕ однофамильцы —
нормализованный текст совпадает символ в символ либо на 0.91.

--- 1. ignore_stderr: три идентичные копии
    src/rob_box_voice/rob_box_voice/audio_node.py:31
    src/rob_box_voice/rob_box_voice/sound_node.py:36
    src/rob_box_voice/rob_box_voice/tts_node.py:147
Куда свести: src/rob_box_voice/rob_box_voice/utils/ (рядом с audio_utils.py).
Все три ноды импортируют оттуда. Проверь, нет ли уже подходящего модуля —
не заводи новый, если есть куда положить.

--- 2. _wait_future: три копии одного тела
    src/rob_box_mcp_tools/rob_box_mcp_tools/tools/navigation.py:32  (докстринг на 12 строк)
    src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py:25      (тот же докстринг)
    src/rob_box_mcp_tools/rob_box_mcp_tools/tools/mapping.py:30     (докстринг в одну строку)
Тело у всех трёх одинаковое: threading.Event + future.add_done_callback +
event.wait. Развёрнутый докстринг в navigation объясняет ПОЧЕМУ так, а не через
rclpy.spin_until_future_complete (тот ломает MultiThreadedExecutor) — этот текст
надо сохранить при слиянии, он единственное место, где причина записана.
Куда свести: рядом с базой тулов (src/rob_box_mcp_tools/rob_box_mcp_tools/base.py
или соседний хелпер-модуль). ВАЖНО: не тащи ROS-импорты на уровень модуля —
tools/navigation.py уже тянет rclpy.action на импорте, и именно поэтому каталог
тулов собирается через AST, а не импортом (см. tools/gen_tool_catalog.py).

--- 3. RetryPolicy: две идентичные, обе живые
    src/rob_box_harness/rob_box_harness/providers/deepseek.py:99
    src/rob_box_harness/rob_box_harness/providers/minimax.py:128
Обе импортируются: providers/mimo.py:29 берёт версию из deepseek,
test_integration_offline.py — из minimax. Свести в один модуль внутри
providers/ и оставить ре-экспорт из обоих мест, чтобы существующие импорты
не сломались (или почини импорты — но тогда прогоняй оба теста).

--- 4. ToolCallAccumulator: две живые копии в ОДНОМ пакете
    src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py:52          (81 строка)
    src/rob_box_mcp_tools/rob_box_mcp_tools/core/tool_call_accumulator.py:12 (105 строк)
Сходство 0.91. Обе используются:
    core/__init__.py:3      экспортирует версию из core/
    llm_adapter.py:29       импортирует версию из async_executor
Это самая опасная из четырёх — разошлись на 0.91, значит уже расходятся.
Сначала выясни, ЧЕМ они отличаются (какая версия полнее и почему), и только
потом своди. Если поведение разное — это баг, опиши его в PR отдельно.

Порядок работы:
1. На каждую копипасту — сначала тест, который ловит расхождение
   (или пользуется общей версией из обоих мест), потом слияние.
2. Отдельный коммит на каждую из четырёх. PR можно один.
3. В сообщении коммита: где лежали копии, чем отличались (если отличались),
   куда свели. Пример формата — коммит cd551181 («one wake-word list
   instead of nine»).
4. Предохранитель: подумай, нужен ли тест, который упадёт, если копия
   заведётся снова. Для wake_words такой сделали
   (src/rob_box_voice/test/unit/core/test_dialogue_state_paths.py,
   test_wake_words_are_declared_exactly_once) — можно взять за образец,
   но не копируй бездумно: для функций подход другой.

Чего НЕ делать:
- Не трогай rob_box_core (clock/memory/dialogue_state) — там нужно решение
  Шифу, см. §4 хендофа 2026-08-30.
- Не трогай MinimaxMusicClient / GeneratedMusicLibrary — они разошлись
  сильно, это отдельная большая карточка.
- Не трогай два TTS-реестра (harness/tts/registry.py vs
  rob_box_llm/tts_provider_registry.py) — отдельная карточка W6-2.
```

---

## 6. Следующие карточки (промты не написаны)

- **W6-2** — два TTS-реестра: `rob_box_harness/tts/registry.py` и
  `rob_box_llm/tts_provider_registry.py`. Оба живые, оба экспортятся
  своим `__init__`. Нужно понять, какой канонический.
- **W6-3** — observability: `start_metrics_server` (0.90) и
  `record_telegram_message` (0.69) в telegram и voice.
- **W6-4** — `MinimaxMusicClient` + `GeneratedMusicLibrary` в
  `mcp_tools/core` и `voice/core`. Разошлись сильно (187/301 и 327/492
  строк) — сначала анализ, потом решение.
- **W5-1…W5-4** — память по спеке §2.4, промты уже написаны в
  `2026-08-29-wave2-worker-prompts.md`. Начинать с W5-4 (стабильность
  speaker_id), иначе W5-3 будет писать факты не тому человеку.

---

## 7. Как проверить состояние

```bash
python tools/gen_tool_catalog.py --check
```

```bash
.venv/Scripts/python.exe -m pytest src/rob_box_voice/test/unit -q -p no:cacheprovider --continue-on-collection-errors
```

Ожидается **1442 passed / 0 failed / 14 skipped**. Порядок директорий на
результат не влияет — если хочется убедиться:

```bash
.venv/Scripts/python.exe -m pytest src/rob_box_voice/test/unit/sound src/rob_box_voice/test/unit/node -q -p no:cacheprovider
```

```bash
.venv/Scripts/python.exe -m pytest src/rob_box_harness/test/test_dialog_core.py -q -p no:cacheprovider
```

Ожидается **71 passed**. По всему `src/rob_box_harness/test` — 21 failed
/ 802 passed; все падения преддуществуют этим сессиям (сверено с
`43b64b2a`: было 24 / 795).

`src/rob_box_mcp_tools/test` — 33 failed / 219 passed / 4 errors, тоже
всё преддуществует (на `43b64b2a` было 33 / 218 / 4). Отдельная задача.
