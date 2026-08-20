# Zenoh в ROS 2 Lyrical — ревью текущей конфигурации

**Дата:** 2026-08-21
**Ветка:** feature/lyrical
**Статус:** ревью + рекомендации (изменения на роботе НЕ проверены)
**Источники:** `ros2/rmw_zenoh@lyrical` — README.md, `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`, `DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5`, `zenoh_cpp_vendor/CMakeLists.txt`, `zenoh_cpp_vendor/package.xml`; Docker Hub `eclipse/zenoh` tags.

---

## 1. Что изменилось в Lyrical по сравнению с Humble

| Аспект | Humble | Lyrical |
|--------|--------|---------|
| Репозиторий RMW | `eclipse-zenoh/rmw_zenoh` | **`ros2/rmw_zenoh`** (официальный ROS org, 495★) |
| Пакет | `ros-humble-rmw-zenoh-cpp` | `ros-lyrical-rmw-zenoh-cpp` |
| Zenoh-биндинги | zenoh-cpp (C++) | **zenoh-c 1.8.0** через `zenoh_cpp_vendor` v0.10.5 (vendor коммит `2687c5135`, zenoh-c коммит `05bd3703`) |
| Router | `ros2 run rmw_zenoh_cpp rmw_zenohd` | то же |
| Config override | — | **`ZENOH_CONFIG_OVERRIDE`** (новое) |
| Проверка router'а | — | **`ZENOH_ROUTER_CHECK_ATTEMPTS`** |
| Default-конфиги | — | `DEFAULT_RMW_ZENOH_{SESSION,ROUTER}_CONFIG.json5` лежат в пакете |
| Zenoh SHM | — | есть, по умолчанию выключен в ROS-конфиге |

---

## 2. Ключевые факты (проверено по исходникам lyrical)

1. **`namespace` — официальное поле session-конфига.** В `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5` есть строка
   `// namespace: "my/namespace"`. Текущий подход репо (sed по этой строке) корректен, но избыточен — см. п.4.

2. **`ZENOH_CONFIG_OVERRIDE`** — переопределение любых полей конфига через env без правки файлов.
   Пример: `ZENOH_CONFIG_OVERRIDE='namespace="robots/X"'` или `ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/1.2.3.4:7447"]'`.

3. **`ZENOH_ROUTER_CHECK_ATTEMPTS`** семантика: `0` = ждать бесконечно; `<0` = пропустить проверку;
   `>0` = N попыток с паузой 1с; `unset` = 1 попытка. Репо использует `10` — корректно.

4. **Default router слушает только IPv6** (`listen/endpoints: ["tcp/[::]:7447"]`) — это known issue:
   на IPv4-only системах роутер падает. Кастомные конфиги репо (`listen` на `10.1.1.10`/`10.1.1.11`) этот
   баг обходят — правильно, это значение оставить.

5. **Версия router'а не совпадает с RMW.** Роутер репо = `eclipse/zenoh:1.6.2`; RMW в Lyrical вендорит
   **zenoh-c 1.8.0**. Актуальные теги `eclipse/zenoh`: `1.9.0`, `1.10.0` (авг 2026). Zenoh 1.x в целом
   wire-совместим, но расхождение в 3+ минорных версии — источник риска (особенно на фоне истории
   transport-ошибок в репо: `ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md`).

6. **`ROS_AUTOMATIC_DISCOVERY_RANGE` и `ROS_DOMAIN_ID`** — DDS-понятия. Нативный `rmw_zenoh` их **не читает**
   (discovery делает сам Zenoh). Это наследие Humble/DDS, безвредно, но шумит.

7. **`LD_LIBRARY_PATH=/opt/ros/lyrical/opt/zenoh_cpp_vendor/lib:...`** — нужен, т.к. `zenoh_cpp_vendor`
   ставит `libzenohc.so` в нестандартный путь. Это задокументированное решение миграции (D-03), НЕ удалять.
   Но он продублирован в базовых образах И в compose — см. п.4.

---

## 3. Сравнение as-is vs should-be

| Что | Как сейчас | Как должно быть в Lyrical | Вердикт |
|-----|-----------|--------------------------|---------|
| Router image | `eclipse/zenoh:1.6.2` | `rmw_zenohd` из ROS-образа (гарантированный match версии) **или** `eclipse/zenoh:1.8.x+` | ⚠️ см. п.4 P1 |
| Namespace | sed-замена в `ros_with_namespace.sh` + генерация `/tmp/zenoh_session_config.json5` | `ZENOH_CONFIG_OVERRIDE='namespace=robots/$ROBOT_ID'` | Упростить (P2) |
| Session/router конфиги | 674 строки кастомные | дефолт из пакета + override нужных полей | Упростить (P2) |
| `LD_LIBRARY_PATH` | в базовых образах + в каждом сервисе compose | только в базовых образах (ENV) | Почистить дубли (P3) |
| `ROS_DOMAIN_ID=0` | в каждом сервисе | убрать (rmw_zenoh не читает) | Почистить (P3) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | в каждом сервисе | убрать (rmw_zenoh не читает) | Почистить (P3) |
| Комментарий в vision compose | «Namespace НЕ используется» | фактически namespace применяется (ROBOT_ID + ros_with_namespace.sh) | Поправить коммент |
| `docker/main/zenoh-router/Dockerfile` | `FROM introlab3it/rtabmap_ros:lyrical-latest` (не используется compose) | мёртвый код либо переписать на `ros:lyrical-ros-base` | Почистить |

---

## 4. Рекомендации (по приоритету)

### P1 — Выровнять версию router'а с RMW (риск, требует проверки на роботе)
RMW вендорит zenoh-c **1.8.0**, роутер на **1.6.2**. Два варианта:
- **A (минимальный):** поднять `eclipse/zenoh` до `1.8.x` (ближайший к RMW). Проверить совместимость
  674-строчного конфига со схемой 1.8.
- **B (правильный по-ROS):** перевести роутер на `rmw_zenohd` из того же образа (`ros:lyrical-ros-base`
  + `ros-lyrical-rmw-zenoh-cpp`) — гарантированный match версии и конфиг-схемы. Команда ранее откатила
  `rmw_zenohd` в пользу `eclipse/zenoh` (см. `docs/development/AGENT_GUIDE.md`), но тогда это был Humble;
  в Lyrical `rmw_zenohd` ставится одной apt-командой без обёрток.

### P2 — Заменить sed/генерацию namespace на `ZENOH_CONFIG_OVERRIDE`
`ros_with_namespace.sh` и `generate_zenoh_session_config.sh` можно свести к одной env-переменной в compose:
`ZENOH_CONFIG_OVERRIDE="namespace=robots/$${ROBOT_ID}"`. Это меньше подвижных частей и меньше места для ошибок.

### P3 — Почистить DDS-наследие и дубли
- Убрать `ROS_DOMAIN_ID=0` и `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` из сервисов (rmw_zenoh их игнорирует).
- Оставить `LD_LIBRARY_PATH` только в базовых Dockerfile (уже есть), убрать дубли в compose.
- Поправить/удалить мёртвые `docker/{main,vision}/zenoh-router/Dockerfile` (не используются compose).

---

## 5. Что НЕ трогать (доказанно нужно)
- `listen` роутеров на явных IPv4 (`10.1.1.10`/`10.1.1.11`) — обходит IPv6-only баг дефолта.
- `so_sndbuf/so_rcvbuf=16MB` в session-конфиге Main Pi — защита от transport-closure на больших сообщениях rtabmap.
- `connect` Main Pi → облако (`zenoh.robbox.online:7447`), Vision Pi → Main Pi (`10.1.1.10:7447#iface=eth0`).
- `ZENOH_ROUTER_CHECK_ATTEMPTS=10`.

## 6. Что реализовано (2026-08-21)

Выполнена чистка под Lyrical (решение: router → `rmw_zenohd`, namespace → `ZENOH_CONFIG_OVERRIDE`):

1. **Router** (`docker/{main,vision}/zenoh-router/Dockerfile`): `FROM ros2-zenoh` база + `CMD ros2 run rmw_zenoh_cpp rmw_zenohd`.
   CI `L-Build Single Service.yml`: `BASE_IMAGE` → `rob_box_base:ros2-zenoh-${ROS_DISTRO}` (было `eclipse/zenoh:1.6.2`).
2. **Compose** (`docker/{main,vision}/docker-compose.yaml`):
   - `zenoh-router`: `rmw_zenohd` + `ZENOH_CONFIG_OVERRIDE` (connect в облако/Main Pi, listen на eth0 IP + loopback);
     healthcheck `pgrep -f rmw_zenohd` (старый REST `:8000` не работал).
   - ROS-ноды: `ZENOH_SESSION_CONFIG_URI=/tmp/...` → `ZENOH_CONFIG_OVERRIDE=namespace="robots/${ROBOT_ID}"`;
     убраны `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE`, wrapper `ros_with_namespace.sh`, volume `/ros_scripts`
     (кроме rtabmap, которому нужен `patch_rtabmap_launch.py`).
3. **Dockerfile ENV**: убраны `ROS_DOMAIN_ID`/`ROS_AUTOMATIC_DISCOVERY_RANGE`/`ZENOH_SESSION_CONFIG_URI`/`ZENOH_CONFIG`
   из `ros2_control`, `vesc_nexus`, `led_matrix`, `voice_assistant` и трёх базовых образов.
4. **Скрипты**: удалены `ros_with_namespace.sh`, `generate_zenoh_session_config.sh`, `start_zenoh_router.sh`
   (main+vision); вычищены hardcoded env из `start_lslidar.sh`, `start_robot_state_publisher.sh`, `start_perception.sh`.

### Изменение поведения (проверить на роботе)
- **QoS drop-on-congestion и тюнинг буферов rtabmap/nav2 сохранены** через `ZENOH_CONFIG_OVERRIDE` (JSON5):
  `qos/publication=[{key_exprs:["rt/rtabmap/**"],...},{key_exprs:["rt/global_costmap/**"],...}]`,
  `transport/link/rx/buffer_size=16777216`, `transport/link/tx/queue/size/*` (data_high/data/data_low=16,
  background=8, interactive_high/low=4). Это та же защита от transport-closure, что была в старом session-конфиге
  (на неё ссылается `rtabmap.ini`): mapData/cloud_map дропаются при конгестии вместо kill транспорта.
- **Убраны только 16MB `so_sndbuf/so_rcvbuf`** (TCP socket-буферы на connect-endpoint): их нельзя выразить через
  `ZENOH_CONFIG_OVERRIDE`, т.к. разделитель пар `;` совпадает с разделителем опций endpoint'а. Если loop closure
  снова даст transport-closure — вернуть через отдельный session-конфиг.
- **Session подключается к router через loopback** (`tcp/localhost:7447` дефолт), а не через `10.1.1.10`.
  Router слушает и eth0 IP, и `127.0.0.1`.

### Follow-up (сделано 21.08, дочистка)
- Удалены неиспользуемые `docker/{main,vision}/config/zenoh_{router,session}_config.json5`.
- Переписаны `scripts/testing/validate_zenoh_config.sh` и `validate_zenoh_namespace.sh` под новую схему
  (`ZENOH_CONFIG_OVERRIDE` + `rmw_zenohd` вместо файлов конфига и sed).
- `docker/vision/test/docker-compose.test.yml`: убраны `ROS_DOMAIN_ID`/`ROS_AUTOMATIC_DISCOVERY_RANGE` из
  `x-ros-env`. Тестовый роутер намеренно оставлен на `eclipse/zenoh:1.6.2` — это изолированное тестовое
  окружение на нестандартном порту 17447 (REST-плагин для healthcheck), не связано с продом.

## 7. Статус проверки
Изменения **НЕ проверены на роботе**: нужен `docker compose up` на Main/Vision Pi и сверка
`ros2 topic list` / `docker logs zenoh-router`. Валидность YAML обоих compose проверена локально.
Выводы о `rmw_zenoh` — из исходников `ros2/rmw_zenoh@lyrical`, а не со стенда.
