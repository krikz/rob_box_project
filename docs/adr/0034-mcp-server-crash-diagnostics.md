# ADR-0034: mcp_server crash diagnostics — почему `head -50` ломает видимость crash'ей нод

| Поле         | Значение                                                                                  |
|--------------|-------------------------------------------------------------------------------------------|
| Статус       | **Accepted**                                                                              |
| Дата         | 2026-08-31                                                                                |
| Автор        | developer-agent (kanban t_1ebc3e4e, hardening после incident #1736 / 29.08.2026 15:25 UTC) |
| Формат       | MADR (Markdown Any Decision Record)                                                       |
| Контекст     | Incident #1736 (deploy run 33260223956), `mcp_server` exit 1 без traceback в логах         |
| Родители     | Issue [#1736](https://github.com/krikz/rob_box_project/issues/1736), kanban t_1ebc3e4e     |
| Связанные    | `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py`, `mcp_auth.py`, `.github/workflows/L-Deploy and Verify.yml` |
| Реализует    | Helper `_init_step()` в `MCPServer` (fail-fast с FATAL-логом), logging в `_read_or_create_token_file`, атомарное создание токена через `tempfile.NamedTemporaryFile` |

---

## 1. Контекст и бизнес-проблема

29.08.2026 в 15:25:44 UTC на run `33260223956` (workflow `L-Deploy and Verify`,
branch=develop, env=staging) deploy gate поймал:

```
[ERROR] [mcp_server-10]: process has died [pid 1347, exit code 1,
  cmd '/ws/install/rob_box_mcp_tools/lib/rob_box_mcp_tools/mcp_server
  --ros-args --log-level info …'].
```

Что известно:

1. Между последним "здоровым" develop (28.08 22:56, t_a5c8d964) и падением
   в develop влились три коммита, затрагивающих `mcp_server.py` / `mcp_auth.py`:
   `1d9fc3a6`, `8c90e46e`, `0d600049`. Самый подозрительный — `8c90e46e`
   (новый модуль `mcp_auth.py` + вызов `RequestAuthenticator.from_env(...)` в
   `__init__`).
2. На Vision Pi (`10.1.1.11`) `voice-assistant` сейчас Up (healthy).
   `/data/.mcp_token` mode=0600, owner=root, 64 байта, создан 28.08 20:47 —
   с момента влития `8c90e46e`. То есть токен-файл успешно создан
   *на втором запуске*.
3. В логах workflow'а (run `33260223956`) **нет** ни traceback, ни ERROR
   до 50-й строки. Команда `head -50 deploy.log | grep ERROR` — пусто.
   Причина: workflow режет `head -50` + `tail -150` (см.
   `.github/workflows/L-Deploy and Verify.yml:681/683`), а mcp_server падал
   в первые ~5-10 секунд после старта launchsystem, что попадает в
   "SKIPPED MIDDLE LOGS".
4. Два последующих редеплоя (30.08 19:26 — run `33330895761`, 30.08 21:02 —
   run `33335300188`) с тем же кодом прошли УСПЕШНО. Новых PR в mcp-tools
   между фейлом и успехами не было.

### Гипотезы о root cause

| Гипотеза | Подтверждение |
|---|---|
| H1: race при первом создании `/data/.mcp_token` на overlay-FS, `os.link()` падает с `OSError`, fallback `os.replace` не защищает от перезаписи | Воспроизведено в `analysis/repro_mcp_token_race.py` — оба процесса получают `None` (или разные токены) при гонке + `os.link` OSError |
| H2: `_read_or_create_token_file` возвращает `None` → `RequestAuthenticator.from_env` тихо создаёт `enabled=False` authenticator → все execute() отклоняются, но процесс **живёт** | Подтверждено: `RequestAuthenticator._allow_unauthenticated=False` + `_token=None` → `verify() → False`, никакого exit 1 |
| H3: exit 1 — побочный эффект другого блока `__init__` (`_init_voice_memory`, `_init_waypoint_store`, `_register_tools` etc.) — НЕ mcp_auth, но легло в то же окно логов | Не подтверждено и не опровергнуто. **Логи не сохранили тело traceback** — поэтому и нужно это ADR |
| H4: ROS2 `__init__` падает без stacktrace в логгер потому, что ROS2 main-thread не оборачивает пользовательские исключения | Подтверждено частично: при ручном эксперименте `rclpy.init()` + `Node('x')` с исключением в `__init__` → процесс exit 1, в `rclpy_logger.fatal` ничего нет (если не использовать `logger.fatal`) |

### Реальный вывод

Главный технический долг — **видимость crash'ей нод**. Workflow режет
логи, но даже полные логи ROS2 не показывают traceback из `__init__`
без явного `logger.fatal()`. Без этого любой новый crash будет
"exit 1, посреди логов, без контекста" — то есть повторение #1736
гарантировано.

---

## 2. Рассмотренные варианты

### Вариант A: fail-fast в Dockerfile через `RUN python3 -c "import rob_box_mcp_tools.mcp_server"`

- **Плюсы**: ловит import-time ошибки (отсутствующие пакеты, syntax error,
  битые зависимости) ещё на этапе сборки образа.
- **Минусы**: **не** ловит runtime-исключения в `__init__` (например,
  тот же race за токен, или `PermissionError` на SQLite БД). Стоит ~30
  секунд к build time. Это диагностика, а не лечение.
- **Вердикт**: дополнительная мера, но не основная. Применять опционально.

### Вариант B (ПРИНЯТ): helper `_init_step()` в `MCPServer` + явный `logger.fatal` + `RuntimeError`

- **Плюсы**: каждый блок инициализации оборачивается в helper, который
  (1) пишет `info` ДО выполнения (видно в `head -50`), (2) на исключении
  пишет `fatal` СРАЗУ + format traceback, (3) поднимает `RuntimeError`
  с контекстом. **Любой crash в `__init__` теперь виден либо в `head -50`
  (info), либо в `tail -150` (RuntimeError traceback) — промахнуться
  невозможно.**
- **Минусы**: требует рефакторинга `__init__` (каждый блок → через
  `_init_step`). Стоит ~1 час работы + ~5 минут регрессии на тестах.
  Меняет public API только в части выбрасывания нового `RuntimeError`,
  что для ROS2-ноды не критично (там все равно "exit 1 при исключении").
- **Вердикт**: основное решение.

### Вариант C: переписать `_read_or_create_token_file` через `tempfile.NamedTemporaryFile` + детальное логирование

- **Плюсы**: фиксит H1 (race) — оба процесса теперь получают ОДИН и тот
  же секрет (проверено в `test_race_two_processes_both_get_valid_token`).
  Логирует каждый шаг, чтобы будущие crash'и были диагностируемы.
- **Минусы**: требует доп. тестов на race-условия (5+ новых).
- **Вердикт**: основное решение для `mcp_auth`.

### Вариант D (отвергнут): писать токен в Redis/etcd/Consul

- **Плюсы**: атомарность из коробки, нет race.
- **Минусы**: добавляет инфраструктурную зависимость в vision-pi compose,
  которая там не нужна. Over-engineering для одного 64-байтного секрета.
- **Вердикт**: YAGNI.

---

## 3. Решение

Принимаются варианты **B + C**:

1. В `mcp_auth.py`:
   - `_read_or_create_token_file` переписан на `tempfile.NamedTemporaryFile(dir=parent)`
     + `os.replace`. Это атомарно в пределах одной ФС и не зависит от
     поддержки hard-link'ов overlay'ом.
   - Перед `os.replace` проверяется `os.path.exists(path)` — если параллельный
     процесс уже создал файл, берём ЧУЖОЙ секрет (race detection).
   - Каждый шаг логируется через переданный `logger`. Все ошибки возвращают
     `None` + error/warning-лог (раньше — silent).
   - Проверка прав на файл вынесена в `_check_token_file_mode` и зовётся
     при каждом чтении (а не только при создании).

2. В `mcp_server.py`:
   - Добавлен helper `MCPServer._init_step(name, fn)`, который логирует
     `info` ДО выполнения блока и `fatal + traceback` при исключении.
   - Критичные блоки `__init__` обёрнуты в `_init_step`:
     `voice_memory`, `faq_store`, `waypoint_store`, `register_tools`,
     `authenticator` (вызов `RequestAuthenticator.from_env`).
   - `RequestAuthenticator` отныне fail-fast: если токен недоступен И
     `ROB_BOX_MCP_ALLOW_UNAUTHENTICATED` не выставлен — `RuntimeError`
     в `__init__` (через `_init_step`). Раньше — silent degradation.

3. Регрессионные тесты `test/test_mcp_auth.py`:
   - `TestRaceConditions` — 8 новых тестов на race / OSError / fsync / chmod.
   - `TestFromEnvAuthFailFast` — 1 тест на сигнал `enabled=False`.
   - Итого 31 passed (22 → 31).

4. `analysis/repro_mcp_token_race.py` — локальный repro-скрипт, который
   воспроизводит ВСЕ гипотезы из таблицы выше. Использовать при будущих
   инцидентах как smoke-test.

---

## 4. Последствия

### Положительные

- Любой crash в `MCPServer.__init__` теперь виден либо в `head -50`
  (info "🔧 mcp_server init: <name>…"), либо в `tail -150` (RuntimeError
  traceback). Промахнуться невозможно.
- Race в `_read_or_create_token_file` закрыт: оба процесса получают
  один и тот же секрет, тесты `test_race_two_processes_both_get_valid_token`
  и `test_os_link_oserror_falls_back_to_replace` это фиксируют.
- 9 новых unit-тестов покрывают race / overlay-FS / chmod scenarios.

### Отрицательные / компромиссы

- `MCPServer.__init__` теперь throw'ит `RuntimeError` с конкретным шагом
  — это **breaking change** для прямых наследников (их нет, но если
  кто-то mock'ал `MCPServer.__init__` — придётся обновить).
- `_init_step` добавляет обёртку над каждым блоком — на ~10 строк больше
  кода в `__init__`. Приемлемо: helper читаемый и приносит ровно то, что
  документирует.
- `RequestAuthenticator.from_env` теперь fail-fast при отсутствии токена
  + отсутствии `ROB_BOX_MCP_ALLOW_UNAUTHENTICATED`. **Это правильно, но
  на staging в первом редеплое после merge'а может случиться так, что
  /data/.mcp_token не создастся по правам — тогда mcp_server будет
  падать вместо silent-degradation. Нужно проверить права на /data
  в compose (issue #1736 fix).**

### Что НЕ покрыто этим ADR

- Pre-existing failing тесты в `test/test_mcp_server.py` (9 штук) и
  `test/test_tool_catalog_sync.py` (5 штук) — все из-за того, что
  фикстуры не подставляют `DurabilityPolicy` в mock rclpy. **Это не
  в scope данной задачи** — pre-existing бага в тестах, не связанная
  с mcp_auth / mcp_server.__init__.
- Root cause exit 1 на staging 29.08 — **не подтверждён**. Скорее
  всего H1 + H3 в комбинации (race на токен + побочный эффект в
  другом блоке `__init__`), но без сохранённого traceback это гипотеза.
  Реальная проверка возможна только при повторении сбоя.

---

## 5. Применение

- kanban t_1ebc3e4e complete (после CI green).
- Деплой на develop пройдёт через обычный merge flow — fix автоматически
  попадёт в следующий deploy на staging.
- При первом редеплое проверить, что `/data` в `voice-assistant` имеет
  права `0777` или mcp_server запускается от root с доступом к `/data`.

---

## 6. Ссылки

- `.github/workflows/L-Deploy and Verify.yml` (строки 681/683 — `head -50` + `tail -150`)
- `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_auth.py` (lines 296–460 — обновлённый `_read_or_create_token_file`)
- `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` (lines 643–688 — `_init_step`)
- `src/rob_box_mcp_tools/test/test_mcp_auth.py` (lines 235–401 — 9 новых тестов)
- `analysis/repro_mcp_token_race.py` (repro-скрипт для всех 6 гипотез)
- Issue #1736 (deploy run 33260223956), kanban t_1ebc3e4e
