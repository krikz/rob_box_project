# Скан дублей: `dialogue_node` + `rob_box_harness`, 2026-08-30

> **Продолжение** [`2026-08-30-dedup-wave3-handoff.md`](2026-08-30-dedup-wave3-handoff.md) §6.
> **База:** `develop` @ `cbe16bb9`.
> **Статус:** §5 закрыт коммитом `6eaf5b72`; ревью и остальные
> правки — в [`2026-08-30-harness-dialogue-code-review.md`](2026-08-30-harness-dialogue-code-review.md).
> **Честность (ADR-0018):** каждая строка ниже — вывод команды, приведённой
> рядом, либо ссылка `файл:строка`. **Тесты в этой сессии не прогонялись** —
> это скан, а не правка. Ни один файл проекта не изменён.

**Метод.** AST-обход (`ast.unparse` по нормализованному дереву: докстринги
вырезаны, комментарии в unparse не попадают), сравнение через
`difflib.SequenceMatcher`. Три прохода: кросс-файловый по одинаковым именам,
внутрифайловый по `dialogue_node.py`, и отдельный проход по строковым
литералам от 25 символов.

---

## 1. Главный вывод по `dialogue_node`: дублей там нет

Это отрицательный результат, и он важен, потому что закрывает вопрос.

85 определений `dialogue_node.py` (от 8 строк) сравнены с 985 определениями
из `rob_box_harness/**` и `rob_box_voice/**`. При пороге сходства 0.60 —
**ноль совпадений**. При ослабленном пороге 0.55 всплывают шесть пар, и все
шесть — структурный шум:

```
0.59  DN._user_wants_music         ->  harness/core/dialog_core.py::is_wake_word
0.59  DN._cancel_greeting_timer    ->  voice/utils/respeaker_interface.py::disconnect
0.57  DN._build_music_retry_prompt ->  voice/scheduler/pre_gen.py::claim
```

`_user_wants_music` и `is_wake_word` — обе «проверить вхождение из списка в
строку», общего кода нет. То же с остальными.

**Перепроверено вторым, более широким прогоном.** Тот скан шёл без
префильтра (полный перебор пар, чтобы отсечь подозрение, что префильтр
что-то спрятал) и по более низкому порогу длины: 90 определений
`dialogue_node.py` от 5 строк против пула в 1134. Единственное совпадение
от 0.60:

```
0.70  DN.destroy_node:4867 (5L)  ->  voice/speaker_id_node.py::destroy_node:487 (6L)
```

Это ROS-lifecycle teardown: у ноды `try / finally` вокруг
`shutdown_asyncio_loop`, у `speaker_id_node` — две проверки `hasattr` с
закрытием executor'а и БД. Схожи они только тем, что обе кончаются
`super().destroy_node()`. Дублем не являются.

Итого: два независимых прохода, широкий и узкий, дают один ответ.

**Что это значит для плана.** P8 из хендофа 29.08 («`dialogue_node` — 4911
строк вместо thin shell») — задача **декомпозиции, а не дедупликации**.
Нода уже делегирует рассуждение в harness:

```bash
grep -n "^from " src/rob_box_voice/rob_box_voice/dialogue_node.py | grep harness
```

```
44:from rob_box_harness.config import LLMConfig
45:from rob_box_harness.core.dialog_core import DialogCore, DialogResult
46:from rob_box_harness.core.dialogue_state_machine import (
51:from rob_box_harness.core.tool_registry import ToolRegistry
52:from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider
```

Второго движка диалога нет. Раздут не мозг, а ROS-обвязка вокруг него.

**Где именно раздут** — 14 методов из 86 держат 2637 строк (57 % класса):

| метод | строка | длина |
|---|---|---|
| `__init__` | 284 | **418** |
| `_handle_result` | 3889 | **402** |
| `_run_turn` | 2712 | **384** |
| `_on_stt` | 1694 | **294** |
| `_continue_after_tool_calls` | 3541 | 166 |
| `_build_tool_provider` | 1269 | 150 |
| `_declare_params` | 702 | 143 |
| `_build_dynamic_system_context` | 2499 | 142 |
| `_build_single_provider` | 1083 | 104 |
| `_do_recursive_streaming` | 3708 | 93 |
| `_check_babble_and_retry` | 3289 | 91 |
| `_execute_tool_calls` | 3802 | 86 |
| `_apply_music_guard` | 3413 | 83 |
| `_build_llm` | 1188 | 81 |

Остальные 62 метода — 1599 строк, из них 10 (108 строк) вообще пустышки:
однострочные делегации в `core/dialogue_helpers.py`, оставшиеся от TD-1
(`_detect_volume_intent:4519`, `_detect_pitch_intent:4528`,
`_generate_fallback_response:4510`, `_map_emotion_to_animation:4473` и др.).
Их вызовы можно заменить прямыми импортами, а обёртки удалить — безопасная
механическая правка, но она даст около 100 строк из 4900. Настоящий объём
лежит в четырёх методах сверху.

---

## 2. 🔴 Три поколения одного реестра

Хендофф (§6, W6-2) описывал это как «два TTS-реестра». На деле реестра
**три**, и TTS тут ни при чём — скопирован generic-каркас `register /
resolve / unregister / names` плюс `Factory.create / reset_cache`.

| | файл |
|---|---|
| A | `rob_box_llm/rob_box_llm/tts_provider_registry.py` (163 строки) |
| B | `rob_box_harness/rob_box_harness/tts/registry.py` |
| C | `rob_box_harness/rob_box_harness/registry.py` (харнессы, не TTS) |

Пометодное сходство нормализованного исходника:

| метод | C vs B | B vs A |
|---|---|---|
| `names` | **1.00** | **1.00** |
| `unregister` | **1.00** | **1.00** |
| `reset_cache` | **1.00** | **1.00** |
| `create` | 0.95 | 0.79 |
| `register` | 0.95 | 0.33 |
| `__init__` | 0.93 | 0.92 |
| `resolve` | 0.88 | 0.56 |

Копипаста здесь **задокументированная**. Докстринг C (`registry.py:12`)
прямо ссылается на реестр из `rob_box_llm` как на образец, которому он
следует «for consistency»; докстринг B (`tts/registry.py:3`) говорит, что
повторяет структуру `rob_box_harness.registry` и апстримного реестра. То
есть каждое следующее поколение копировали осознанно.

Расходятся они ровно в трёх точках — и это те самые точки, которые должны
быть параметрами обобщённого реестра:

1. **класс исключения** — `HarnessNotFoundError(..., harness=name)` против
   `ProviderNotFoundError(..., port="tts")`;
2. **тип конфига** — `HarnessConfig` / `TTSConfig` / `Mapping[str, Any]`;
3. **`_stable_config_hash`** — сходство C vs B всего **0.17**: у C это
   восемь под-конфигов харнесса, у B — девять скалярных полей TTS.

Всё остальное — буквально один код. `reset_cache`, `unregister` и `names`
совпадают символ в символ во всех трёх.

**Риск, а не только эстетика.** `_cache` объявлен атрибутом класса
(`registry.py:88`, `tts/registry.py:92`) — то есть это процесс-глобальный
мутабельный словарь. Тесты чистят его через `reset_cache()`, но каждый
реестр надо чистить отдельно: `test/conftest.py:17` импортирует только
`HarnessFactory`. Забыть третий — получить протекание инстанса между
тестами.

**Карточка W6-2 (переписать).** Не «выбрать канонический из двух», а:
вынести `Registry` + `Factory` в один модуль, параметризовать классом
ошибки и функцией хеша конфига, а три существующих имени оставить тонкими
подклассами ради обратной совместимости импортов. Предохранитель — тест,
который упадёт при появлении четвёртой копии (по образцу
`test_wake_words_are_declared_exactly_once`).

---

## 3. 🔴 Имена ROS-топиков захардкожены в 2–4 пакетах

Тот же класс, что `wake_words` (девять мест, три списка, `cd551181`), и он
ещё не тронут. Одинаковые строковые литералы в разных файлах:

| топик | файлы |
|---|---|
| `/voice/generated_music/state` | `mcp_tools/tools/minimax_music.py:702`, `mcp_tools/tools/music.py:1737`, `voice/dialogue_node.py:550`, `voice/sound_node.py:112` |
| `/voice/tts/provider_state` | `mcp_tools/mcp_server.py:236`, `voice/dialogue_node.py:545`, `voice/tts_node.py:897` |
| `/voice/tts/batch_complete` | `mcp_tools/tools/dialogue.py:113`, `voice/dialogue_node.py:556`, `voice/tts_node.py:903` |
| `/voice/tts/batch_registered` | `mcp_tools/tools/dialogue.py:127`, `voice/dialogue_node.py:567` |
| `/voice/dialogue/barge_in_policy` | `voice/dialogue_node.py:457`, `voice/stt_node.py:280` |
| `/navigate_to_pose/_action/cancel_goal` | `mcp_tools/tools/navigation.py:296`, `voice/command_node.py:273` |

Опечатка или переименование в одном месте не ломает сборку и не роняет
тест — узел просто молча перестаёт получать сообщения. Ровно тот режим
отказа, который в прошлой волне маскировал мёртвый `mcp_server`.

Готового общего модуля топиков для voice нет, но **прецедент в репозитории
есть**: `rob_box_quest/rob_box_quest/protocol/topics.py` (с тестом
`test_topics.py`) и `rob_box_telegram/supervisor_client.py:114`
(`TOPIC_STATE`, `TOPIC_HEARTBEAT`). Стиль можно копировать оттуда.

Осложнение, которое надо снять до карточки: топики делят **три пакета**
(`voice`, `mcp_tools`, `harness`), общего низкоуровневого пакета у них нет.
На эту роль подходит `rob_box_core`, но по нему висит вопрос §4.1 прошлого
хендофа. **Эта карточка блокируется тем же решением Шифу про
`rob_box_core`.**

---

## 4. 🟡 `VoiceCommandHandler` — 340 строк мёртвого кода с копией фраз

`src/rob_box_voice/rob_box_voice/core/voice_command_handler.py` не
импортируется **нигде**, кроме собственного теста:

```bash
grep -rn "voice_command_handler" --include=*.py src/
```

```
src/rob_box_voice/test/unit/core/test_voice_command_handler.py:6:from rob_box_voice.core.voice_command_handler import (
```

Коммит `3c5b193d` «извлечён VoiceCommandHandler в core слой» — извлекли, но
к ноде так и не подключили. Тест держит его зелёным, поэтому пропажа не
заметна.

При этом внутри лежит **вторая копия пользовательских фраз** управления
громкостью/тоном/скоростью, дословно совпадающих с живым путём в
`mcp_tools/tools/system.py`:

| фраза | мёртвая копия | живая |
|---|---|---|
| «Громкость уже максимальная» | `voice_command_handler.py:211` | `tools/system.py:122` |
| «Громкость уже минимальная» | `voice_command_handler.py:213` | `tools/system.py:124` |
| «Голос уже максимально высокий» | `voice_command_handler.py:248` | `tools/system.py:224` |
| «Голос уже минимально низкий» | `voice_command_handler.py:250` | `tools/system.py:226` |
| «Скорость уже максимальная» | `voice_command_handler.py:285` | `tools/system.py:324` |

Это P7 («мёртвые модули») с конкретным адресом. Решение — не сведение, а
удаление вместе с тестом. Но сперва стоит убедиться, что замысел
«обрабатывать громкость локально, без похода в LLM» не был осознанно
отложенной фичей. Вопрос Шифу, ответ дешёвый.

---

## 5. 🟡 `memory.py` / `transport.py` — модуль и пакет под одним именем

В `rob_box_harness/` одновременно лежат и модуль, и пакет:

```
memory.py     (626 строк)   memory/__init__.py    + memory/sqlite_voice.py
transport.py  (185 строк)   transport/__init__.py + transport/ros2_transport.py
```

Python в такой ситуации всегда выбирает **пакет**:

```bash
python -c "import rob_box_harness.memory as m; print(m.__file__)"
```

```
...\rob_box_harness\memory\__init__.py
```

Чтобы `memory.py` не стал мёртвым, оба `__init__.py` грузят его вручную
через `importlib.util.spec_from_file_location` по относительному пути
`../memory.py` и переливают `__all__` в неймспейс пакета
(`memory/__init__.py:17-33`, `transport/__init__.py:19-39`). Работает — оба
файла проверены импортом выше.

Две оговорки:

1. **Сам хак — дубль.** Это два экземпляра одного примерно 50-строчного
   шаблона, отличающиеся тремя идентификаторами. Появится третья пара имён —
   появится третья копия.
2. **Модуль исполняется под чужим именем** — `rob_box_harness._memory_module`
   (`memory/__init__.py:20`). Пока грузит только `__init__`, идентичность
   классов одна. Но любой прямой импорт того же файла вторым путём даст
   второй объект класса `Turn` / `MemoryStore`, и `isinstance` тихо вернёт
   `False`. Сейчас такого импорта нет — **проверено**, но предохранителя
   тоже нет.

Дублирования API между `memory.py` и `memory/sqlite_voice.py` **нет**:
первый — ABC `MemoryStore` плюс фейковый `InMemoryStore`, второй — живой
`SQLiteVoiceMemory`. Тройное появление `clear_waypoints` в скане
(`memory.py:213`, `memory.py:399`, `sqlite_voice.py:535`) — это абстрактный
метод, фейк и реализация, законный полиморфизм.

Правка дешёвая и без изменения поведения: переименовать `memory.py` в
`memory/base.py`, `transport.py` в `transport/base.py`, а в `__init__`
поставить обычный `from .base import *`. Хак с `importlib` исчезает.

---

## 6. Что проверено и оказалось НЕ дублем

Чтобы следующий воркер не тратил на это время повторно.

- **`validate_args` в `executors/local.py:85` и `executors/ros_mcp.py:98`**
  (P4 из прошлого хендофа) — две реализации одного интерфейса с разной
  логикой: `local` отвергает любые аргументы (`local.py:90-96`), `ros_mcp`
  валидирует по JSON-схеме. Сводить нечего.
- **Но по дороге нашлось другое**, и это стоит отдельной задачи. Валидатор
  харнесса `_validate_json_schema` (`ros_mcp.py:208`) проверяет `required`,
  `type` и `additionalProperties`, **но не `enum`** — слова `enum` в файле
  нет вообще. Валидатор mcp_tools `validate_parameters`
  (`mcp_tools/base.py:301`) проверяет `required` и `enum`, **но не `type`**.
  Они не конкуренты, а два слоя по разные стороны ROS-границы (второй слой —
  `mcp_tools/registry.py:162`). Итог: нарушение `enum` проходит харнесс
  беспрепятственно и ловится только после перехода границы, а нарушение типа
  вторым слоем не ловится вовсе. Это дыра в покрытии, не дубль — **завести
  отдельно, в W6-1 не тащить**.
- **`aclose` — 13 вхождений** в харнессе: реализации протокола закрытия по
  3–6 строк, тела разные. Шум скана.
- **`teardown`, `capabilities`, `list_tools`, `register`** в харнессах —
  реализации интерфейса, не копии.
- **`dialogue_node` против harness/voice** — см. §1, ноль совпадений.

---

## 7. Очередь после этого скана

Порядок — по отношению «риск / цена», а не по номерам.

| | задача | блокирует |
|---|---|---|
| 1 | **W6-1** — четыре дословные копипасты. Промт готов, §5 хендофа 30.08. | — |
| 2 | **§5 здесь** — `memory.py` / `transport.py` → `base.py`. Механическая, без смены поведения, снимает два `importlib`-хака. | — |
| 3 | **§2 здесь** — обобщить три реестра (переписанный W6-2). | — |
| 4 | **§4 здесь** — удалить `VoiceCommandHandler` вместе с тестом. | вопрос Шифу (дешёвый) |
| 5 | **§6 здесь** — дыра `enum` / `type` в двух слоях валидации. | — |
| 6 | **§3 здесь** — топики в общий модуль. | **решение по `rob_box_core`** (§4.1 хендофа 30.08) |
| 7 | **P8** — декомпозиция четырёх методов `dialogue_node` (`__init__` 418, `_handle_result` 402, `_run_turn` 384, `_on_stt` 294). Не дедупликация. | — |

Открытыми остаются все четыре вопроса Шифу из §4 хендофа 30.08; вопрос про
`rob_box_core` теперь блокирует на одну карточку больше — топики.
