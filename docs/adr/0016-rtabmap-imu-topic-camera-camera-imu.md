# ADR-0016: IMU-топик rtabmap на Main Pi — /camera/camera/imu (issue #681)

| Поле         | Значение                                                                  |
|--------------|---------------------------------------------------------------------------|
| Статус       | **Proposed**                                                              |
| Дата         | 2026-08-12                                                                |
| Автор        | architect (Hermes Agent)                                                  |
| Формат       | MADR (Markdown Any Decision Record)                                       |
| Контекст     | deployment-critical #681: test / Main / perception / critical_log          |
| Родители     | [ADR-0010](0010-perception-bridge-and-aggregator.md) (perception bridge), docs/architecture/ICP_ODOMETRY.md |
| Связанные    | issue #681 (сигнатура `deploy-problem:test:main:perception:critical_log:01d106f85660`), issue #698 (дубликат) |
| Закрывает    | ложный critical_log: `rtabmap.icp_odometry: We didn't receive IMU newer than previous image/scan` |

---

## 1. Контекст и проблема

### 1.1 Симптом

Деплой-пайплайн (L-Deploy and Verify) поймал в контейнере `perception` на test-окружении:

```text
[health_monitor-1]   [ERROR] rtabmap.icp_odometry (3s ago): We didn't receive IMU newer than previous image/scan (0.0000
```

и классифицировал его как `critical_log` → авто-issue #681. Дубликат #698 закрыт автоматически. Проблема повторяется: авто-sweep 12.08 подтвердил «АКТУАЛЬНА».

### 1.2 Симптом с точки зрения системы

`rtabmap.icp_odometry` — узел внутри контейнера `rtabmap` на Main Pi. Он **не падал** и SLAM продолжал работать (health_monitor показывал `✅ HEALTHY`, `Total Errors: 0 за минуту`). Это был не инцидент доступности, а ложный critical в мониторинге.

---

## 2. Root Cause

### 2.1 Факты, подтверждённые на живых роботах (2026-08-12)

| Проверка | Результат |
|----------|-----------|
| Vision Pi (10.1.1.21), `oak-d` container: `ros2 topic list \| grep imu` | `/camera/camera/imu`, `/camera/imu/data` |
| Vision Pi: `ros2 topic info /camera/camera/imu -v` | **Publisher count: 1** |
| Vision Pi: `ros2 topic hz /camera/camera/imu` | **~250 Hz** |
| Vision Pi: `ros2 topic info /camera/imu/data -v` | **Publisher count: 0** (никто не публикует) |
| Main Pi (10.1.1.20), `rtabmap` container: `ros2 topic info /camera/imu/data -v` | **Publisher count: 0**, Subscription count: 1 (сам rtabmap) |
| Main Pi: `ros2 topic hz /camera/camera/imu` | **~250 Hz** (топик доступен через zenoh) |

### 2.2 Почему так вышло

OAK-D (Vision Pi) публикует IMU в `/camera/camera/imu`, потому что в `oakd_with_apriltag.launch.py`:

```python
Node(
    package='depthai_ros_driver',
    executable='camera_node',
    name='camera',
    namespace='camera',   # → node path /camera/camera
    ...
)
```

Namespace + node name дают двойной префикс `/camera/camera/...`. Это закреплено коммитами `e761edf1` (namespace='camera') и `4fd62ce0`.

А `docker/main/docker-compose.yaml` (сервис `rtabmap`) с декабря 2025 (коммит `466ba631`, «add IMU topic for ICP odometry stability») передаёт:

```
- imu_topic:=/camera/imu/data
```

Топик `/camera/imu/data` **не существует** (Publisher count: 0 на обоих Pi). rtabmap подписывается на мёртвый топик, `icp_odometry` не получает свежий IMU и на каждый скан/кадр пишет ERROR «We didn't receive IMU newer than previous image/scan». health_monitor (в `perception`) собирает ERROR из `/rosout` → деплой-пайплайн считает это critical.

### 2.3 Почему это не было critical

- `wait_imu_to_init:=false` — rtabmap не ждёт IMU для инициализации.
- SLAM работает в LiDAR-only режиме: ICP-одометрия по `/scan` + wheel odometry guess (`odom_guess_frame_id:=odom`). IMU — опциональное улучшение для graph optimization, а не обязательный вход.
- health_monitor: `Status: ✅ HEALTHY`, ошибки старые (>2000s), ни одного свежего.

---

## 3. Рассмотренные альтернативы

### 3.1 Убрать `imu_topic` совсем

| | |
|---|---|
| Плюсы | Проще всего; icp_odometry перестанет ждать IMU вообще |
| Минусы | Теряем реальную IMU-поддержку (OAK-D публикует ~250 Hz, доступен через zenoh); при резких поворотах ICP без IMU менее стабилен (см. коммит 466ba631 — IMU добавляли именно для стабильности) |

**Отклонено**: IMU полезен и доступен, проблема была только в неверном имени топика.

### 3.2 Оставить `/camera/imu/data` и добавить ремоунт/бридж на Vision Pi

| | |
|---|---|
| Плюсы | Не трогаем Main Pi |
| Минусы | Лишний движущийся элемент (relay/bridge), который надо поддерживать; проще поправить один аргумент launch |

**Отклонено**: лишняя сложность ради несуществующего топика.

### 3.3 ✅ Исправить `imu_topic` на `/camera/camera/imu`

| | |
|---|---|
| Плюсы | Минимальное изменение (1 строка compose); rtabmap получает реальный IMU (~250 Hz); root cause устранён; поведение соответствует задуманному в 466ba631 |
| Минусы | Если OAK-D/zenoh недоступен, снова появится ERROR — поэтому дополнительно добавляем exclude-паттерн в dedup (см. §4.2) |

**Выбрано.**

---

## 4. Решение

### 4.1 Конфигурация

`docker/main/docker-compose.yaml` (сервис `rtabmap`):

```diff
-      - imu_topic:=/camera/imu/data
+      - imu_topic:=/camera/camera/imu
```

Комментарий над command обновлён: IMU приходит с Vision Pi OAK-D через zenoh, `/camera/imu/data` не существует — не менять обратно.

### 4.2 Мониторинг (страховка)

`.github/scripts/deployment_issue_dedup.py`, `CRITICAL_EXCLUDE_BY_SCOPE["main"]`:

```python
r"didn't receive imu newer",
r"dropping image/scan data.*delay",
```

Если IMU снова пропадёт (OAK-D недоступен, zenoh отвалился, камера не инициализировалась), `icp_odometry` продолжит писать этот ERROR — но это **не** аутэйдж: SLAM работает по лидар-ICP + wheel odom. Такой лог не должен создавать critical-issue в деплой-пайплайне.

### 4.3 Тесты

- `scripts/testing/test_rtabmap_main_pi_has_no_camera_inputs.py`: assert на `/camera/camera/imu` + регрессионный assert «`/camera/imu/data` не должен встречаться».
- `.github/scripts/tests/test_deployment_issue_dedup.py`: 2 новых теста — `didn't receive imu newer` и `dropping image/scan data.*delay` не извлекаются как critical.

### 4.4 Документация

`docs/architecture/ICP_ODOMETRY.md`: подписка rtabmap обновлена на `/camera/camera/imu`.

---

## 5. Последствия

### Положительные

- Устраняется источник ложного critical_log (#681 и будущие дубликаты).
- rtabmap начнёт реально использовать IMU (~250 Hz) — как задумано в 466ba631.
- Мониторинг перестанет шуметь по известному безобидному паттерну.

### Отрицательные / риски

- Пока фикс не задеплоен на test-робота, ERROR может появляться снова — но exclude-паттерн уже в скрипте dedup, так что новый issue не создастся.
- Если в будущем OAK-D переедет на другой namespace, топик IMU снова разъедется — момент зафиксирован в комментарии compose и в этом ADR.

### Нейтральные

- Никаких изменений в health_monitor, perception-bridge, TF-дереве и nav2.

---

## 6. Проверка

1. Юнит-тесты: `python3 -m pytest .github/scripts/tests/test_deployment_issue_dedup.py scripts/testing/test_rtabmap_main_pi_has_no_camera_inputs.py -q` — PASS.
2. После деплоя на test: `docker logs rtabmap --tail 50 | grep -i imu` — не должно быть «We didn't receive IMU newer».
3. `docker exec rtabmap bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /camera/camera/imu'` — ~250 Hz.

---

*Решение принято архитектором по итогам live-диагностики 2026-08-12 (Main Pi 10.1.1.20, Vision Pi 10.1.1.21).*
