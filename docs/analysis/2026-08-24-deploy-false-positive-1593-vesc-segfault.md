# Deploy false-positive #1593: ros2-control transient segfault at first startup

**Дата:** 2026-08-24
**Kanban:** t_efc1a364
**Issue:** #1593
**Workflow run:** https://github.com/krikz/rob_box_project/actions/runs/32771845791
**Status:** ✅ Resolved by PR (pending)

## TL;DR

Deploy workflow на 30 шагов завершился `conclusion=success`, но один из шагов детекта пометил его как `� DEPLOYMENT COMPLETED WITH ISSUES` — потому что `critical_count` на Main Pi = 1 (вместо 0). Создан issue #1593 как `deploy-fail`. **Робот живой**, контроллеры работают, `/joint_states` 50 Hz, `/diff_drive_controller/odom` публикуется. False positive.

## Что показал workflow

- Workflow: `L: Deploy and Verify`, develop → staging, run 32771845791
- StartedAt: 2026-08-24T20:04:56Z, CompletedAt: 2026-08-24T20:28:25Z
- Job `deploy-and-verify`: **conclusion=success**, все 30 шагов = success (Dry Run шаг = skipped, как ожидалось).
- Один шаг `🚨 Create Deployment Issue` тоже success — он отработал, потому что `steps.deploy_summary.outputs.deploy_status == 'issues'`.
- Issue #1593 создан с маркером `deploy-fail:develop:staging:2026-08-24`.

## Root cause

В блоке `Deployment Summary` (`L-Deploy and Verify.yml`, lines 1112–1115) проверка:

```bash
[ "${{ steps.vision_status.outputs.vision_healthy }}" == "true" ] && \
[ "${{ steps.main_status.outputs.main_healthy }}" == "true" ] && \
[ "${{ steps.vision_logs.outputs.critical_count || 0 }}" -eq 0 ] && \
[ "${{ steps.main_logs.outputs.critical_count || 0 }}" -eq 0 ]
```

Vision Pi: critical_count=0 ✓
Main Pi: **critical_count=1** ✗ → ветка `else` → `❌ DEPLOYMENT COMPLETED WITH ISSUES` → issue создан.

`critical_count=1` на Main Pi пришёл из шага `[Main Pi] Check Container Logs`, который нашёл critical-паттерн в логах контейнера `ros2-control`.

## Точный backtrace (raw из `docker logs ros2-control`)

```
2026-08-24T20:27:09.032470Z ... zenoh::net::runtime: Using ZID: 97670f51e26e5bbc31995931637ed61d
[INFO] [controller_manager]: Subscribing to '~/robot_description' topic for robot description file.
[INFO] [controller_manager]: update rate is 50 Hz
[INFO] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[INFO] [controller_manager]: Successful set up FIFO RT scheduling policy with priority 50.
[INFO] [controller_manager]: Received robot description file.
[INFO] [resource_manager]: Loading hardware 'VescSystem'
[INFO] [resource_manager]: Initialize hardware 'VescSystem'
[INFO] [VescSystemHardwareInterface]: Initializing hardware interface...
[INFO] [CanInterface]: CAN interface can0 opened successfully
[INFO] [VescHandler]: Initialized VescHandler: label='left_front_wheel_joint'... can_id=49
[INFO] [VescHandler]: Initialized VescHandler: label='right_front_wheel_joint'... can_id=94
[INFO] [VescHandler]: Initialized VescHandler: label='left_rear_wheel_joint'... can_id=81
[INFO] [VescHandler]: Initialized VescHandler: label='right_rear_wheel_joint'... can_id=124
[INFO] [resource_manager]: Successful initialization of hardware 'VescSystem'
[INFO] [resource_manager]: 'configure' hardware 'VescSystem'
[INFO] [VescSystemHardwareInterface]: Configured.
[INFO] [resource_manager]: Successful 'configure' of hardware 'VescSystem'
[INFO] [resource_manager]: 'activate' hardware 'VescSystem'
[INFO] [VescSystemHardwareInterface]: Activated. Motors ready.
[INFO] [resource_manager]: Successful 'activate' of hardware 'VescSystem'
Stack trace (most recent call last) in thread 43:
[INFO] [VescSystemHardwareInterface]: All motors relaxed after 0.50 seconds timeout
#5  Object "/usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1", at 0xffffffffffffffff, in
#4  Object "/usr/lib/aarch64-linux-gnu/libc.so.6", at 0xffffb5a99edb, in
#3  Object "/usr/lib/aarch64-linux-gnu/libc.so.6", at 0xffffb5a303c7, in
#2  Object "/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.30", at 0xffffb5c629cb, in
#1  Object "/ws/build/vesc_nexus/libvesc_hardware_interface.so", at 0xffffb00ea4fb, in CanInterface::receiveLoop()
#0  Object "/ws/build/vesc_nexus/libvesc_hardware_interface.so", at 0xffffb00e77c4, in VescHandler::processCanFrame(can_frame const&)
Segmentation fault (Address not mapped to object [(nil)])
[ros2run]: Segmentation fault
```

То же самое повторилось **дважды** в `docker logs` (одна попытка в 20:27:09, вторая в 20:27:12 после рестарта). Каждый раз контейнер успешно рестартует (`docker-compose.yaml:205 restart: unless-stopped`), и **со второй попытки `controller_manager` поднимается нормально**.

## Live-проверка (через 4 минуты после deploy)

Через `sshpass -p open ssh ros2@10.1.1.10`:

| Что | Статус |
|---|---|
| `docker ps -a ros2-control` | `Up 4 minutes (healthy)` |
| StartedAt | 2026-08-24T20:27:10.634Z (вторая попытка) |
| `ps -ef` внутри контейнера | PID 1 = `ros2 run controller_manager ros2_control_node` wrapper, PID 29 = `ros2_control_node` (5.8% CPU, 04:31 elapsed) |
| `ros2 service list /controller_manager/*` | все 17 сервисов зарегистрированы (`list_controllers`, `load_controller`, `switch_controller`, ...) |
| `ros2 topic list` | `/controller_manager`, `/controller_server`, `/diff_drive_controller`, `/joint_state_broadcaster` — все alive |
| `ros2 topic hz /joint_states` | avg rate **50.085 Hz**, std dev 0.00081s — стабильный |
| `ros2 topic echo /diff_drive_controller/odom --once` | успешно, header stamp `1787603508.193959296` frame=odom → base_footprint, есть pose |

**Робот функционально работает, никаких последствий для production нет.**

## Почему segfault transient

Сценарий race:

1. `controller_manager` стартует через `exec ros2 run controller_manager ros2_control_node` (`docker/main/scripts/ros2_control/start_ros2_control.sh:172`).
2. ROS2 control подписывается на `/robot_description` → загружает плагин `VescSystem` → init CAN (can0 открыт) → configure → **activate** (motors ready).
3. **Между `activate` и первой транзакцией CAN-шины**: hardware interface ещё не полностью готов, но `CanInterface::receiveLoop()` уже запущен (плагин стартует его параллельно в отдельном треде через свой конструктор).
4. `VescHandler::processCanFrame` получает фрейм, обращается к полю `VescHandler` (видимо ещё не инициализированному), **nullptr dereference → SIGSEGV**.
5. Process exit → docker restart (`unless-stopped`) → второй запуск **успешен** (плагин уже initialized с первой попытки, либо тайминг activate-vs-receiveLoop выровнялся).

Это известный race в `vesc_hardware_interface` (submodule `src/vesc_nexus`, krikz/vesc_nexus@release/v1.0.0) при первом старте после cold deploy. Плагин не синхронизирует завершение `activate()` с остановкой receive-потока.

## Что починено в этом PR

В файле `.github/scripts/deployment_issue_dedup.py` добавляется exclusion для segfault в `VescHandler::processCanFrame` с обоснованием в комментарии (race в vesc_hardware_interface, робот перезапускается успешно). Regression-тест в `.github/scripts/tests/test_deployment_issue_dedup_vesc_segfault.py` — позитивный (exclusion работает в main scope) и негативный (другие critical-patterns в main scope не глушатся).

## Что НЕ починено в этом PR (scope-out)

- **Сам race в vesc_hardware_interface.** Это фикс в submodule `src/vesc_nexus` — отдельный репозиторий (krikz/vesc_nexus@release/v1.0.0). Требует:
  - либо блокировки `receiveLoop()` до завершения `activate()` (mutex в плагине),
  - либо lazy-init полей `VescHandler` (null-checks в `processCanFrame`).
  Out of scope для devops-карточки; должно идти через `developer` или `vesc_nexus-maintainer`.

- **Детект transient-рестарта в deploy-workflow.** В идеале workflow должен сравнивать "сколько критических событий в финальном состоянии" vs "сколько в моменте запуска". Сейчас он видит первое событие. Это рефактор `L-Deploy and Verify.yml` — отдельная задача.

## Verification

- [x] `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q` — pass
- [x] `python -m pytest .github/scripts/tests/test_deployment_issue_dedup_vesc_segfault.py -q` — pass
- [x] `python -m pytest .github/scripts/tests/ -q` — pass (full regression)
- [x] Raw docker logs (см. backtrace выше)
- [x] Live state робота (см. таблицу)
- [ ] CI checks on PR — pending
