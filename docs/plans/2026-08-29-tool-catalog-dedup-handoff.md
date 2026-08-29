# Хендофф — вычистка дуальностей вокруг MCP-каталога тулов, 2026-08-29

> **Кто писал:** Claude Opus 5. Сессия 1 — без ROS 2 и без железа
> (Windows, `rclpy` не установлен, тесты через `.venv`). Сессия 2 (вечер
> того же дня) — с доступом на робота через katana → vision 10.1.1.21:
> закрыт P1 и проведена проверка на железе, см. §6.
> **Честность (ADR-0018):** каждое утверждение ниже — либо вывод команды,
> приведённой рядом, либо ссылка на файл со строкой. Ничего не помечено
> «работает», если это не подтверждено прогоном.

---

## 1. С чего началось

Запрос: «проведи кодревью MCP ноды и регистрации тулов, там такое ощущение
что код почему-то два раза написан».

Ощущение подтвердилось, но масштаб оказался больше: **каталог инструментов
был объявлен четырежды**, из них два объявления — живые и разъехавшиеся.

Дальше задача расширилась до «вычищять дуальности, строго; такого кода не
должно быть в репе».

---

## 2. Что сделано

**Сессия 1 — 3 коммита в `develop`:**

| Коммит | Суть |
|--------|------|
| `e96b912d` | Единое объявление каталога тулов + предохранитель от дрейфа |
| `20cd3358` | Починка зависания процесса после тестов (TTS event loop) |
| `f1c97a9a` | Единый словарь анимаций + единая заглушка `rclpy.qos` |

Суммарно: 44 файла, **+3.4k / −5.6k строк** (нетто −2.2k).

**Сессия 2 — 8 коммитов в `develop`** (детали в §3 P1 и §6):

| Коммит | Суть |
|--------|------|
| `731623c4` | 🔴 `ToolExecutionType` из `.base`, не `.animations` — `mcp_server` падал на старте + тест на разрешимость относительных импортов |
| `6291f91e` | Якорь `RULE #VOICE` на само правило, а не на таблицу маршрутизации |
| `f6889f8c` | Убран ложный warn «Голос 'None' недоступен» на каждой реплике |
| `13238a9a` | Сняты два теста, сторожившие текст, удалённый в #1665 |
| `b8674685` | `nav_msgs` — объявленная зависимость, импорт наверх файла (ADR-0021) |
| `971d399b` | Пять фикстур, проверявших фикстуру, а не код |
| `786122e0` | Один `FakeNode` вместо четырёх; `std_msgs`-сообщения — классы, а не MagicMock'и |
| `ed2ca049` | `hasattr` на MagicMock и гонка на гранулярности `monotonic()` |

### 2.1. Каталог тулов — одно объявление

Было: схемы писались в классах `MCPTool` (их исполняет `mcp_server`) **и**
руками в `rob_box_harness/core/tool_registry.py` (их видит LLM).

Стало: источник истины — класс тула. `tools/gen_tool_catalog.py` парсит
классы через AST (импортировать их нельзя — `navigation.py` тянет
`rclpy.action` на уровне модуля) и генерирует
`src/rob_box_core/rob_box_core/_tool_catalog_data.py`. Harness читает его
через `rob_box_core.tool_catalog`.

`rob_box_core` выбран потому, что у него **ноль зависимостей**: harness не
должен тянуть ROS 2, а `rob_box_mcp_tools` не должен тянуть harness.

Регенерация:

```bash
python tools/gen_tool_catalog.py
```

### 2.2. Что нашлось из-за расхождения двух копий

Всё ниже — реальные баги, не гипотезы:

- **`navigate_to_waypoint`**: LLM'у объявлялся параметр `name`, а
  `execute()` принимает `waypoint`, и он обязательный. Любая навигация по
  вейпоинтам через LLM падала на валидации.
- **`move_direction`**: объявлялся `duration` при параметре `distance` →
  `TypeError` на каждом вызове с ним.
- **`gen_*` (4 тула)**: `track_id` объявлен опциональным, хотя `execute()`
  его требует.
- **13 тулов** были зарегистрированы на сервере, но отсутствовали в
  LLM-каталоге: `task_delta` (из-за чего перехват S6 в
  `scheduler/tool_executor.py:201` был физически недостижим),
  `stop_navigation`, весь mapping-FSM (включая тулы выхода из него),
  `set_volume`/`set_pitch`/`set_speed`, `get_perception_context`,
  `get_sound_info`, `listen_for_response`.
- **29 из 38** общих описаний деградировали до заглушек на стороне harness
  (`play_sound`: 782 символа руководства → 40).
- Описание `search_web` отправляло LLM к `search_artist_style` — тула,
  который существовал только в мёртвом `skills/music_skill.py`.

### 2.3. Предохранитель

`src/rob_box_mcp_tools/test/test_tool_catalog_sync.py` — 6 тестов:

1. каталог актуален (перегенерация + diff);
2. схема не обещает параметров, которых `execute()` не принимает;
3. обязательные аргументы `execute()` объявлены в схеме;
4. каждый видимый LLM тул зарегистрирован в `mcp_server`;
5. каждый зарегистрированный тул есть в каталоге и не скрыт молча;
6. гигиена схем (типы, описания, `required ⊆ properties`).

Тесты 2 и 3 нашли расхождения, которых я не искал (`gen_*`), и один баг в
моём же генераторе (молча терял параметры из фабрик `_track_id_param()`).

### 2.4. Попутные починки

- `/mcp/tools` **не имел подписчика**: `_on_mcp_tools_update` существовал с
  issue #1409, но `create_subscription` для него не было ни одного. Из-за
  этого `mcp_tools_available` навсегда оставался `False`, и в `_on_vad`
  ветка «не рвать agent-loop, пока идут тул-вызовы» была недостижима.
- Таймер `publish_tools` каждые 10 с сериализовал ~50 схем в топик без
  подписчиков → одна latched-публикация (`TRANSIENT_LOCAL`) при старте.
- Промпты event-режима звали `handle_music` / `handle_faq` — фасады,
  удалённые после party-регрессии. Переименованы в существующие тулы.
- `history_excluded_tools` по умолчанию фильтровал `handle_navigation`,
  то есть не фильтровал ничего.
- **Зависание после тестов**: `_ensure_tts_loop()` парковал `run_forever()`
  на воркере `ThreadPoolExecutor` и не хранил ссылок — остановить было
  нечем. Воркеры пула не-daemon, CPython join'ит их в
  `threading._shutdown()`. Проверено экспериментом: `atexit` не помогает
  (выполняется после join'а), работает только
  `threading._register_atexit`.

### 2.5. Словарь анимаций

Канон — 21 манифест в `src/rob_box_animations/animations/manifests/`.
Теперь `rob_box_mcp_tools/animations.py` владеет списком, псевдонимами и
`normalize_animation()`; `speak_text` и `play_animation` импортируют его,
а `test_animation_catalog.py` падает при расхождении с манифестами.

`excited` — псевдоним, а не анимация: манифеста нет. Раньше он был в enum
`play_animation` и переписывался руками через `if animation == "excited"`.

### 2.6. Заглушки `rclpy`

В тестах `rob_box_voice` было **14** рукописных заглушек, спеллящих одни и
те же енумы четырьмя способами, и все ставились через
`sys.modules.setdefault` — то есть побеждала первая по алфавиту директория.
`unit/greeting` публиковала `rclpy.qos` **без** `DurabilityPolicy`, из-за
чего в полном прогоне ломался *импорт* 19 несвязанных тест-модулей, каждый
из которых по отдельности проходил. Выглядело как флак.

`src/rob_box_voice/test/ros_stubs.py` теперь владеет общим `rclpy.qos`.

**Эффект на `src/rob_box_voice/test/unit`:**

| | было | стало |
|---|---|---|
| passed | 1112 | **1418** |
| failed | 9 | 18 |
| errors | 19 | **0** |

Рост падений — это 306 тестов, которые раньше просто не запускались.

---

## 3. Что осталось — по приоритету

### ~~P1. Досмотреть 9 новых видимых падений~~ — ЗАКРЫТО (сессия 2)

`src/rob_box_voice/test/unit`: **1435 passed, 0 failed, 14 skipped**
(было 1418 / 18 / 0). Ни одно падение не было регрессией продакшна —
все девять чинились в тестах, и три из них были багами самих тестов,
маскировавшимися под баги кода:

- **`RULE #VOICE` — НЕ регрессия промпта.** Регекс
  `RULE #VOICE.*?(?=🚨 \*\*RULE #)` цеплялся за первое вхождение строки,
  а это таблица маршрутизации в шапке промпта (стр. 22), не сам блок
  правила (стр. 123). Четыре теста проверяли не те ~800 символов, а
  `test_master_prompt_default_matches_registry` по той же причине
  проходил вхолостую: в таблице нет ни одного voice id, так что
  сверять с реестром было нечего. Блок на месте и полон.
- **Stranger Things / Imperial March в master-промпте** — текст удалён
  намеренно в `f7a374e6` («удалить оверфит», #1590/#1665), тесты не
  обновили. Детализация живёт в `music_skill_prompt.txt` и покрыта
  живым тестом-близнецом; master-промпт теперь сторожится на общее
  правило «Style-specific tracks».
- **`робокс стоп`** — в фикстуре был зашит урезанный список из трёх
  wake-word без «робокс», а `has_wake_word` матчит слово целиком
  (#1292), поэтому фраза отсекалась wake-гейтом задолго до
  command-intent гейта, который тест и проверяет. В деплое все 21
  написание на месте (сверено с `ros2 param get /dialogue_node
  wake_words` на vision). Фикстура теперь читает
  `config/dialogue_node.yaml` — тот же файл, что грузит нода.
- `_active_tg_chat_id` (2 фикстуры), cp1252-чтение UTF-8 (2 места),
  Windows-путь внутри YAML-скаляра в двойных кавычках.

**Плюс два ordering-бага того же класса, что `rclpy.qos`:**
`rclpy.node` ставится через `sys.modules.setdefault`, а `FakeNode` было
**четыре** — побеждала первая загруженная директория, и она решала,
какие ассерты вообще способны пройти. `unit/sound` читает
`node._subscriptions`, которых копия `unit/node` не писала; копия
`unit/node` отвечала `None` на любой `get_parameter`, и `SoundNode`
падал на `sound_pack_dir.startswith`. Теперь `FakeNode` один
(`test/ros_stubs.py`), и `unit/sound`+`unit/node` дают одинаковый
результат в любом порядке. Там же: приватный `std_msgs.msg` в
`unit/sound` нёс `String`, но не `Bool`, из-за чего ломался *импорт* 18
модулей dialogue_node; и сообщения в общем `std_msgs.msg` были
`MagicMock()`-**экземплярами**, так что `String()` возвращал один и тот
же объект всем подряд.

И два теста, которые падали не всегда:
`test_synthesize_yandex_retries_too_long_chunk` (guard через `hasattr`
на MagicMock — а он отвечает «да» на всё) и
`test_dead_cache_expires_after_ttl` (`ttl_s=0.01` против `sleep(0.02)`
при гранулярности `time.monotonic()` ~15.6 мс на Windows).

### P2. `DialogHarness` — ROS-free шелл диалога

Пользователь помнит, что «шелл можно было выдернуть из ROS2 и запустить
отдельно на компе». Он существует:

- `rob_box_harness/harnesses/dialog.py` — `DialogHarness`;
- `rob_box_harness/runner.py` — `run_harness(name, input, config)`.

**Но запустить его сегодня нечем:** у `rob_box_harness/setup.py` нет ни
одного `console_scripts`, в пакете нет ни `main()`, ни `__main__.py`
(проверено grep'ом). То есть миграция доведена до библиотеки, но не до
точки входа.

Что сделать: добавить CLI (`python -m rob_box_harness.cli dialog` или
console_script), и подключить его `SkillRegistry([DJPlaylistSkill(),
MappingSkill()])` к общему каталогу — иначе там заведётся ещё одна копия
списка тулов.

⚠️ **Не удалять.** Я чуть не снёс его как мёртвый код; это целевая
архитектура, просто недоведённая.

### P3. Две абстракции `ToolProvider`

- `rob_box_core/ports.py:114` — канонический (новый);
- `rob_box_harness/tools.py:50` — legacy P0;
- мост: `executors/core_adapter.py`, чей докстринг прямо говорит «New code
  should inject `rob_box_core.ports.ToolProvider` directly. The adapter
  below keeps P0/P1.1 harness code working while migration proceeds».

Миграция явно не завершена. Здесь же живут алиасы-пустышки
`MCPBridgeProviderAdapter = LegacyToolProviderAdapter` и
`LocalSkillProviderAdapter = LegacyToolProviderAdapter`.

### P4. Два валидатора параметров

`MCPToolRegistry.execute` → `validate_parameters` (по MCP-схеме) и
`ROSMCPToolProvider.validate_args` → `_validate_json_schema` (по
JSON-схеме). **Контракт теперь единый**, разъехаться они не могут — это
дубликат кода, а не декларации. Поведение на ошибках разное (одна
возвращает `MCPToolResult`, другая бросает типизированное исключение), так
что слияние требует решения, какое поведение оставить.

### P5. e2e-мок `MOCK_MCP_TOOLS`

`docker/vision/test/scenario_runner/runner.py` — 8 тулов ручной копией
(сейчас с предупреждающим комментарием). Строить из каталога **можно**:
образ scenario_runner наследуется от voice-assistant, чей `ENTRYPOINT`
делает `source /ws/install/setup.bash`, так что `rob_box_core`
импортируем. Локально не проверить — подтвердит только прогон
`L-Integration Tests`.

### P6. Декомпозиция `llm_adapter.py` (454) / `async_executor.py` (601)

По архивному `docs/development/archive/REFACTORING_PLAN_MCP_TOOLS.md`. К
дублированию каталога отношения не имеет — отдельная большая задача.

### P7. Прочие залежи старого кода

Найдено сканом «модули без единого импорта из продакшна»
(70 кандидатов, отфильтровано от ROS-entrypoint'ов и launch-файлов):

- `src/rob_box_voice/scripts/robbox_chat.py` (317) и
  `robbox_chat_streaming.py` (376) — самостоятельный чат **до** харнеса
  (прямой DeepSeek + Silero + `text_normalizer_v2`). Это не тот шелл из P2.
- `src/rob_box_mcp_tools/rob_box_mcp_tools/deepseek_adapter.py` (230) —
  ноль импортов и в проде, и в тестах.
- `src/rob_box_voice/scripts/record_yandex_voice_v1_old.py` (351) — имя
  говорит само за себя.
- `src/rob_box_harness/examples/health_check_demo.py` (347).

Скрипт скана лежать не остался; воспроизводится за 20 строк на `ast`
(обход `src/**/*.py`, сбор имён из `Import`/`ImportFrom`, вычитание).

### P8. `dialogue_node` снова монолит

План 06-02 закрыт с формулировкой «dialogue_node → thin shell (357
строк)». Фактически сейчас **4911 строк**. `DialogCore` при этом живой
(`dialogue_node.py:411`), то есть сосуществуют оба слоя: тонкая обёртка,
как задумано, и вернувшаяся в неё ROS-логика. Это самая крупная
оставшаяся дуальность и, судя по всему, корень исходного вопроса
«почему код написан два раза».

---

## 4. Инварианты, которые теперь держат тесты

| Инвариант | Тест |
|---|---|
| Каталог = классам тулов | `test_tool_catalog_is_current` |
| Схема ⊆ сигнатуре `execute()` | `test_advertised_parameters_are_accepted_by_execute` |
| Обязательные аргументы объявлены | `test_required_execute_arguments_are_advertised` |
| Видимый LLM тул исполним | `test_every_llm_visible_tool_is_registered_on_the_server` |
| Скрытие тула — осознанное | `test_registered_tools_are_not_hidden_from_the_llm` |
| Анимации = манифестам | `test_animation_catalog_matches_manifests` |
| Псевдоним ≠ анимация | `test_aliases_never_shadow_a_real_animation` |
| `from .x import y` разрешим | `test_relative_imports_resolve` |

Добавляя тул: пишете один класс `MCPTool`, регистрируете в
`mcp_server._register_tools`, гоняете `python tools/gen_tool_catalog.py`.
Второго списка больше нет.

---

## 5. Как проверить состояние

```bash
python tools/gen_tool_catalog.py --check
```

```bash
.venv/Scripts/python.exe -m pytest src/rob_box_mcp_tools/test/test_tool_catalog_sync.py src/rob_box_mcp_tools/test/test_animation_catalog.py -q
```

```bash
.venv/Scripts/python.exe -m pytest src/rob_box_voice/test/unit -q -p no:cacheprovider --continue-on-collection-errors
```

Ожидается **1435 passed / 0 failed / 14 skipped**. Порядок директорий
больше не влияет на результат — если хочется убедиться:

```bash
.venv/Scripts/python.exe -m pytest src/rob_box_voice/test/unit/sound src/rob_box_voice/test/unit/node -q -p no:cacheprovider
```

---

## 6. Проверка на железе — выполнена (сессия 2, vision 10.1.1.21)

### 6.0. 🔴 Регрессия, которая ложила весь тул-слой

`f1c97a9a` вписал в `tools/animation.py`

```python
from ..animations import KNOWN_ANIMATIONS, normalize_animation, ToolExecutionType
```

`ToolExecutionType` там никогда не жил — он в `base.py:17`. `mcp_server`
падал на импорте при каждом старте, launch его перезапускал, он падал
снова: **336 трейсбеков за 40 минут**. И поскольку ни один тул-вызов
не мог быть отвечен, `dialogue_node` сыпал `⏱️ Timeout ожидания
результата` на `memory_save`, `speak_text`, `stop_music`,
`execute_music_code` и остальных — все 15 «разных эксепшнов» в логе
были одним багом.

Теста перед этим не стояло: `tools` никто не импортирует в сюите —
`navigation.py` тянет `rclpy.action` на уровне модуля, ровно поэтому
каталог и собирается через AST. Теперь стоит
`test_relative_imports_resolve` (`731623c4`) — обходит все
`from .x import y` в пакете и проверяет, что имя где-то определено.
Чистый AST, без ROS 2. Скан всего `src/` больше таких не находит.

Исправлено в `731623c4`; на роботе установленная копия пропатчена
вживую (см. §6.5).

### 6.1. QoS `/mcp/tools` — ✅ сматчился

```
ros2 topic info -v /mcp/tools
  PUBLISHER  mcp_server     RELIABLE / KEEP_LAST(1) / TRANSIENT_LOCAL
  SUBSCRIPTION dialogue_node RELIABLE / KEEP_LAST(1) / TRANSIENT_LOCAL
```

`ros2 topic echo --once --qos-durability transient_local` отдаёт
залатченный каталог. `mcp_server` рапортует «📤 Опубликован каталог из
51 инструментов (latched)». Порядок старта нод больше не важен.

### 6.2. 13 вернувшихся тулов — ✅ зарегистрированы

`🛠️ MCP Server запущен с 51 инструментами`, в списке есть
`task_delta`, `stop_navigation`, весь mapping-FSM,
`set_volume`/`set_pitch`/`set_speed`, `get_perception_context`,
`get_sound_info`, `listen_for_response`. Что LLM реально начнёт их
звать — покажет живой диалог; регистрация подтверждена.

### 6.3. Ложные warn'ы `[issue 1219]` — ✅ убраны

В логе 21 раз `⚠️ Голос 'None' недоступен у MiniMax`.
`resolve_voice(provider, None)` возвращает `fell_back=True` и когда
голос не запрашивали вовсе (это её контракт, он пришпилен тестом), а
все три ветки в `tts_node` читали флаг как «просили голос, которого
нет». Ничего не ломалось — но лог выглядел как сломанный `set_voice`.
Логи теперь требуют ещё и `voice` (`f6889f8c`).

### 6.4. Остальные контейнеры — чисто

`voice-action-server`, `avatar-supervisor`, `led-matrix`,
`telegram-bot`, `oak-d`, `ceiling-camera`, `supercollider`,
`rob-box-quest` — ноль error/traceback за 40 минут.

Остаётся шум, не связанный с этой задачей: `audio_node` —
`Ошибка записи параметра HPFONOFF: argument 4: TypeError: wrong type`
(ReSpeaker, фолбек на дефолт прошивки) и periodic
`PyAudio paInputOverflow` (issue #1050, потеряно ~0 байт).

### 6.5. ⚠️ Патч на роботе — временный

Правка внесена в установленную копию внутри работающего контейнера
(`/ws/install/.../tools/animation.py`), потому что образ собирается CI.
`mcp_server` живёт с ней 1 ч без единой ошибки — но **правка исчезнет
при пересоздании контейнера**. Чтобы закрепить, нужно запушить
`develop` и дождаться сборки образа.

### 6.6. Что осталось непроверенным

- **e2e-мок публикует `/mcp/tools` с VOLATILE** — с
  `TRANSIENT_LOCAL`-подпиской он не сматчится. На сценарии не влияет,
  но если нужно, чтобы мок доезжал, ему нужен тот же durability.
- **Изменённые описания 29 тулов** — это изменение промпта; смотреть
  надо на поведении LLM в живом диалоге, лог этого не покажет.
- `src/rob_box_mcp_tools/test`: **33 failed / 219 passed / 4 errors**.
  Сверено с `43b64b2a` (до сессии) — там 33 / 218 / 4, то есть все
  падения преддуществуют этой работе (+1 прошедший — новый тест
  импортов). Отдельная задача, в P1 не входила.

---

## 7. Второй e2e-прогон: тулы живы, но модель их не звала

Пользователь пустил e2e сразу после фикса §6.0. Результат раздвоился:

**Транспорт починен.** `mcp_server` за прогон: 3 запроса
(`execute_music_code`, `save_track`, `speak_text`) → 3 успеха, ноль
таймаутов, ноль ошибок. До фикса таймаутил каждый.

**Но модель не звала тулы там, где обязана была.** «какие звуки ты
умеешь проигрывать» → `tools=[]`, ответ «Менеджер не отвечает». «найди
в своей библиотеке синглы барабанов» → `tools=[]`, то же самое. Фразы
«Менеджер не отвечает» в репозитории нет — модель её сочинила и
повторяла.

### 7.1. Откуда она её взяла

Окно из 20 ходов, которое LLM получает каждый turn
(`/root/.rob_box/voice.db`, живой дамп с vision):

```
user      | [Speaker:unknown] [CRITICAL] В прошлом цикле ты НЕ вызвал…
assistant | Менеджер не отвечает. Попробуй ещё раз чуть позже.
user      | загрузи и включи трек тисбит
assistant | Менеджер не отвечает. Попробуй ещё раз через секунду.
```

Два хода из двадцати — `[CRITICAL]`-ретраи babble/music guard'ов,
записанные **в роль `user`**. Четыре — извинения за тулы, которые тогда
и правда были мертвы (краш из §6.0). Модель каждый ход перечитывала
транскрипт, где тулы не работают, а её за это отчитывают, — и вела себя
соответственно.

Ровно этот механизм уже описан в
[`dialog_core.py`](../../src/rob_box_harness/rob_box_harness/core/dialog_core.py)
у DJ-исключения: «LLM тонет в 20+ одинаковых → пустые ответы». Лекарство
применили к DJ-переходам и не применили к `[CRITICAL]`-ретраям.

### 7.2. Что сделано (`eab8518e`, `718e7c80`)

| Проблема | Решение |
|---|---|
| `[CRITICAL]`-ретраи пишутся в историю как реплика юзера | `process_input(is_synthetic=True)` — user-ход не пишется, ответ пишется (его человек слышал) |
| `<system_context>` вставлялся в `messages[1]`, перед 20 ходами истории | Кладётся последним system-сообщением, вплотную к текущей реплике |
| `result.error = Exception(f"{exc}\n{tb}")` — тип терялся, шелл ловил провайдерный сбой подстрокой | Исключение остаётся собой, трейсбек в `DialogResult.error_traceback` |
| История в `~/.rob_box/voice.db` — вне тома, умирала с контейнером | `/data/harness_voice.db` |
| `history_excluded_tools: [handle_navigation, handle_music]` — фасады удалены, фильтр не фильтровал | `[move_direction]` + тест сверяет с каталогом тулов |

Про порядок сообщений: несколько system-сообщений в OpenAI-совместимом
формате (а MiniMax отдаёт именно его) — легально, само по себе не баг.
Ломала **позиция**: снапшот «вот что сейчас» стоял перед двумя десятками
ходов прошлого, и отличить, что он свежее, модели было не по чему.
Порядок сообщений — единственный сигнал времени, который у неё есть.

Почему память НЕ слили в один файл: `/data/voice_memory.db` (VoiceMemory
из mcp_server) и стор harness'а объявляют `waypoints` и `faq_items` с
несовместимыми схемами — у harness `waypoints.name` PRIMARY KEY, у
VoiceMemory `map_id NOT NULL` + FK на `maps`. `CREATE TABLE IF NOT
EXISTS` промолчал бы, а вставки падали бы в рантайме. Слияние двух
сторов — миграция, а не правка конфига; осталось как задача.

### 7.3. Тесты

`test_dialog_core.py`: 64 passed / 3 failed → **71 passed** (три
падения — как раз потерянный тип исключения). Новые:
`test_dynamic_system_sits_last_before_the_user_turn`,
`test_dynamic_system_stays_after_history`,
`test_synthetic_input_is_not_persisted_as_a_user_turn`,
`test_synthetic_turn_still_persists_what_the_user_heard`,
`test_ordinary_input_is_still_persisted`, плюс
`test_dialogue_state_paths.py` (10 тестов на конфиги).

`src/rob_box_voice/test/unit`: **1445 passed, 0 failed**.
`G: Run Tests` и `G: Lint Code` — зелёные.

### 7.4. Чтобы это доехало до робота

- **Код** (`is_synthetic`, порядок сообщений, тип ошибки) — нужен
  пересбор образа: `L: Build All Services` запускается вручную
  (`workflow_dispatch`).
- **Конфиг** (`/data/harness_voice.db`, `history_excluded_tools`) — едет
  bind-mount'ом `docker/vision/config` → `/config`, то есть нужен
  `git pull` на vision и рестарт ноды.
- После пересоздания контейнера горячий патч из §6.5 больше не нужен, а
  отравленная история уйдёт сама: `/data/harness_voice.db` будет пустой.
- **Проверять после деплоя:** те же фразы, что в прогоне 7 — «какие
  звуки ты умеешь проигрывать» должно давать `get_sound_info`, а не
  `tools=[]`.
