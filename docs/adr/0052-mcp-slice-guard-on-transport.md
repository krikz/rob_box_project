# ADR-0052: Срез каталога проверяется на транспорте `/mcp/execute`

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-09-07 |
| Автор | architect (карточка `t_332bdbb1`, issue #1998 §6.2) |
| Контекст | Без серверной проверки среза все инварианты про `operator.*` в целевой архитектуре — пожелания: `dialogue_node` (личность) мог бы галлюцинировать имя `ros2_node_status` и получить исполнение через общий `/mcp/execute` топик. |
| Решение | Сопоставлять `sender` из подписи запроса со срезом и отказывать на стороне `mcp_server` до любых других гардов. Карта `sender → slices` и `slice → tools` живёт в `data/slice_policy.yaml` (data-driven, как `confirmation_policy.yaml`). |
| Затрагивает | `src/rob_box_mcp_tools/rob_box_mcp_tools/base.py` (`MCPTool.slice`), `rob_box_mcp_tools/slice_authority.py` (новый модуль), `rob_box_mcp_tools/data/slice_policy.yaml` (новый), `mcp_server.on_execute_request` (новый slice.guard), `tools/*.py` (аннотация `slice` на подклассах), `test/test_slice_authority.py` (новый), `test/test_mcp_server_slice.py` (новый) |
| Родители | ADR-0051 §2.6 (срез как инвариант архитектуры), целевая §6.2 |
| Связанные | ADR-0053 (планируется — `read_logs` в `operator.admin`), `confirmation_policy.py` (тот же data-driven паттерн) |

---

## 1. Бизнес-проблема

`mcp_server` уже аутентифицирует отправителя `/mcp/execute` общим HMAC-секретом
(модуль `mcp_auth.py`, тесты `test_mcp_auth.py`). Эта проверка отвечает на вопрос
«свой/чужой», но не «оператор/личность». Срез каталога (целевая §6) влияет
сейчас только на то, какие схемы показали модели: если модель галлюцинирует имя
инструмента, чужого её срезу, `mcp_server` всё равно исполнит запрос. Без
серверной проверки инвариант `dialogue_node` ⇏ `operator.admin` остаётся
пожеланием, а не гарантией.

## 2. Решение

Карта «кто какой срез имеет» и «какие инструменты в каждом срезе» —
**YAML-данные** (`data/slice_policy.yaml`), не хардкод в Python. Это
соответствует паттерну, уже принятому для `confirmation_policy.yaml` (issue
#968 §8): политика читается из данных, валидируется на старте, fail-fast.

### 2.1 Слой данных

```yaml
# slice_policy.yaml — кто какой срез каталога имеет и что в каждом срезе
senders:
  dialogue_node:        [core, personality]            # личность: всё, что умеет робот, кроме admin
  avatar_supervisor:    [core, personality, operator.speech, operator.control, operator.admin]   # ТАРС — привилегированный пользователь
  harness:              [core, personality]            # LLM-каркас (тестовая/дебаг-нода) — то же, что личность

slices:
  core:            [get_battery_level, get_current_time, get_current_pose, get_robot_status,
                   get_perception_context, list_waypoints, get_sound_info, get_music_state]
  personality:     [speak_text, listen_for_response, estimate_tts_duration, register_speaker,
                   set_voice, set_tts_provider, list_tts_voices, play_animation, play_sound,
                   set_volume, set_pitch, set_speed, set_dj_mode, search_web, faq_search,
                   memory_save, memory_search, memory_context,
                   execute_music_code, compose_music, stop_music, set_vibe_preset,
                   save_track, list_tracks, load_track, delete_track, search_samples,
                   navigate_to_waypoint, navigate_to_coordinates, move_direction,
                   stop_navigation, save_waypoint, delete_waypoint, clear_waypoints,
                   start_mapping, continue_mapping, finish_mapping, optimize_map, load_map,
                   task_delta]
  operator.speech:    [say, set_voice, set_voice_preset, set_voice_language, preview_voice]
  operator.control:   [dialogue_pause, dialogue_resume, set_avatar_mode, acquire_floor, release_floor]
  operator.admin:     [ros2_node_status, read_logs, container_status]      # шаг 11 (issue #1998)
```

### 2.2 Слой Python

* `MCPTool.slice` — property, по умолчанию `"core"`. Каждый подкласс
  переопределяет на `"personality"` (для LLM-доступных) и т.п. Это даёт
  fail-closed дефолт: тулы, у которых `slice` не задан явно, попадают в
  самый узкий срез.
* `ToolSliceAuthority` — pure-data classifier (как `ToolConfirmationPolicy`):
  читает `slice_policy.yaml`, валидирует на старте, отдаёт
  `is_allowed(sender, tool_name) -> tuple[bool, str]`. `sender` — из блока
  `auth` (тот же, что уже проверил `RequestAuthenticator`).
* `mcp_server.on_execute_request` — после `authenticator.verify()`, **до**
  `mapping_state.is_tool_allowed()` — slice.guard. Отказ пишет в лог
  «sender X не имеет права вызывать tool Y (требуется срез Z)» и
  публикует `result` с `success=False`.

### 2.3 Поток запроса после ADR

```
┌──────────┐     ┌──────────────┐     ┌────────────┐     ┌──────────────┐     ┌────────┐
│ /mcp/    │ ──▶ │ AuthGuard    │ ──▶ │ SliceGuard │ ──▶ │ MappingFSM   │ ──▶ │ Tool   │
│ execute  │     │ (HMAC)       │     │ (sender→   │     │ (block nav   │     │ exec   │
│          │     │              │     │  slice→    │     │  during map) │     │        │
│          │     │              │     │  tool)     │     │              │     │        │
└──────────┘     └──────────────┘     └────────────┘     └──────────────┘     └────────┘
```

Каждый гард отвечает за свой инвариант и не знает про остальные — это
даёт детальные логи при отказе и не позволяет «обходному пути» на одном
уровне маскировать проблему на другом.

## 3. Альтернативы, которые отвергли

* **A. Хардкод в Python.** Слишком хрупко: каждое новое имя инструмента или
  новый sender требует пересборки `voice-assistant`. Данные позволяют
  редактировать политику без redeploy.
* **B. Проверять срез на стороне LLM (в prompt).** Это именно то, что
  сегодня: модель знает, что у неё нет `operator.admin`, но **может
  ошибиться**, и у нас нет защиты. Целевая §6.2 явно требует
  серверной проверки.
* **C. Подписать каждое сообщение, какой срез ему доступен.** Можно, но
  это удваивает работу sender'а (`llm_adapter.py`) и не решает проблему,
  если sender скомпрометирован — он подпишет что угодно. Проверка
  на стороне `mcp_server` дешевле и надёжнее.
* **D. Сделать один `/mcp/execute` на каждый срез.** Это требовало бы
  переделки всего транспорта (топики, QoS, sender'ы) и **не снимает**
  задачу: `dialogue_node` всё ещё может попасть на топик `operator.*`,
  если discovery даст ему доступ. Серверная проверка решает задачу
  в одной точке.

## 4. Trade-offs

* **+** Срез становится **инвариантом транспорта**, а не LLM-промпта.
  Любая попытка обхода (прямая публикация, чужой sender, баг в
  adapter'е) получает отказ на сервере, а не в логах LLM.
* **+** Карта — YAML. Добавление нового sender'а (например,
  `web_console` для ручной отладки) — одна строчка в YAML, без
  пересборки образа.
* **+** `MCPTool.slice` — property, проверяется unit-тестом на
  каждом зарегистрированном tool'е (отказ регистрации, если у тула
  не задан slice).
* **−** Нужно аннотировать все 55 существующих тулов. Делается
  в этом же PR через grep + property, механически.
* **−** Карта `slice → tools` дублирует факт из `MCPTool.slice`.
  Дублирование **намеренное**: YAML — это политика (что *разрешено*
  данному sender'у), `MCPTool.slice` — это декларация (к какому
  срезу *принадлежит* сам инструмент). Карта может быть уже
  (sender не имеет всего среза), property — никогда.

## 5. Что не делаем

* Сами инструменты `operator.admin` (`read_logs`, `ros2_node_status`,
  `container_status`) — карточка issue #1998 шаг 11. Здесь только
  каркас: YAML, slice property, slice.guard, тест на отказ
  `dialogue_node` от попытки вызвать ещё-не-существующий
  `ros2_node_status`.
* `ConfirmationPolicy` для operator.admin — отдельная карточка
  (issue #968 §8 уже покрывает существующие тулы, но не operator.admin).
* `restart_container` (§6.1) — заблокирован на уровне контейнера,
  не делаем.
* Проверка `sender` через подпись vs allowlist — `RequestAuthenticator`
  уже делает и то, и другое. Slice.guard читает `sender` из
  прошедшего `verify()`-запроса, не дублирует HMAC.

## 6. Проверка (DoD)

* [ ] Unit-тест `test_slice_authority.py` — happy path + sender не в
  списке + tool не в срезе + невалидный YAML + sender с пустым
  списком срезов.
* [ ] Тест `test_mcp_server_slice.py` — `dialogue_node` подписывает
  `ros2_node_status` → `mcp_server` отвечает `success=False` с
  сообщением про срез. Сделано без ROS (мок + прямой вызов
  `on_execute_request` через mock-publisher).
* [ ] Все 55 тулов в `mcp_server` имеют непустой `slice` —
  проверяется `test_registry_complete.py::test_all_tools_have_slice`.
* [ ] `redact_log_text` покрывает YAML colon form
  (`DEEPSEEK_API_KEY:` / `deepseek_api_key:`) — unit-тесты в
  `test_redact.py::TestRedactLogText` уже зелёные.
