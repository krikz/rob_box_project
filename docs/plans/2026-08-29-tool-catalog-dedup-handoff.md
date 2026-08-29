# Хендофф — вычистка дуальностей вокруг MCP-каталога тулов, 2026-08-29

> **Кто писал:** Claude Opus 5, сессия без ROS 2 и без железа (Windows,
> `rclpy` не установлен, тесты гоняются через `.venv`).
> **Честность (ADR-0018):** каждое утверждение ниже — либо вывод команды,
> приведённой рядом, либо ссылка на файл со строкой. Ничего не помечено
> «работает», если это не подтверждено прогоном. Что проверить нельзя —
> помечено явно в §6.

---

## 1. С чего началось

Запрос: «проведи кодревью MCP ноды и регистрации тулов, там такое ощущение
что код почему-то два раза написан».

Ощущение подтвердилось, но масштаб оказался больше: **каталог инструментов
был объявлен четырежды**, из них два объявления — живые и разъехавшиеся.

Дальше задача расширилась до «вычищять дуальности, строго; такого кода не
должно быть в репе».

---

## 2. Что сделано (3 коммита в `develop`)

| Коммит | Суть |
|--------|------|
| `e96b912d` | Единое объявление каталога тулов + предохранитель от дрейфа |
| `20cd3358` | Починка зависания процесса после тестов (TTS event loop) |
| `f1c97a9a` | Единый словарь анимаций + единая заглушка `rclpy.qos` |

Суммарно: 44 файла, **+3.4k / −5.6k строк** (нетто −2.2k).

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

### P1. Досмотреть 9 новых видимых падений

Раньше эти модули не исполнялись вообще. Причины (проверены, не мои
правки):

- `UnicodeDecodeError: 'charmap'` — чтение UTF-8 файла без `encoding=`
  (`test_music_library.py`). Чинится добавлением `encoding="utf-8"`.
- `AttributeError: 'DialogueNode' object has no attribute
  '_active_tg_chat_id'` — тесты `test_barge_in_policy.py` конструируют ноду
  в обход инициализации атрибута.
- `assert node._llm_skipped_counter["command_intent"] == 1` → `0` —
  `test_command_intent_gate.py`, логика gate'а.
- `test_issue_1219_set_voice_rule.py` (4 шт.) — промпт `master_prompt` не
  содержит блок `RULE #VOICE` с `set_voice`. **Это может быть реальной
  регрессией промпта**, стоит проверить в первую очередь.

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

---

## 6. Чего я НЕ проверял — обязательно к прогону на железе

Всё покрыто юнит-тестами, но **ни одна правка не проверена на живом ROS**:
в этом окружении `rclpy` — заглушка.

1. **QoS `/mcp/tools`**. Топик переведён на `TRANSIENT_LOCAL` + одна
   публикация; подписка `dialogue_node` создана с той же durability.
   Совместимость publisher/subscriber заглушкой не проверяется. При первом
   запуске: в логе `dialogue_node` должен появиться каталог, а
   `mcp_tools_available` стать `True`. Если QoS не сматчится — на LLM это
   не повлияет (схемы берутся из каталога), но флаг barge-in снова
   останется `False`.
2. **13 вернувшихся тулов**. LLM теперь их видит впервые. Особенно
   `task_delta` (S6) — его перехват до сих пор ни разу не исполнялся.
3. **e2e-мок публикует `/mcp/tools` с VOLATILE** — с новой
   `TRANSIENT_LOCAL`-подпиской он матчиться не будет. На сценарии не
   влияет, но если нужно, чтобы мок доезжал, ему нужен тот же durability.
4. **Изменённые описания тулов.** Для 29 тулов LLM теперь получает
   развёрнутые описания вместо заглушек — это меняет его поведение к
   лучшему по замыслу, но это изменение промпта, и его стоит посмотреть
   вживую.
