# BG-6: VESC wheel jitter при старте и резком торможении (issue #817)

**Дата:** 2026-08-15
**Тикет:** [TASK-050] issue #817 (source: CONCERNS.md BG-6)
**Статус:** частично исправлено ранее (command-timeout/relax), закрыты edge cases
на уровне конфигурации.

## Симптомы

- Колёса вибрируют/дёргаются при старте с места.
- Рывок при резком торможении (в т.ч. при реверсе через ноль).
- Риск `overcurrent` в VESC при старте (inrush из-за ступеньки duty).

## Корневая причина (edge case, оставшийся после command-timeout фикса)

В режиме `control_mode=duty` hardware interface (submodule `vesc_nexus`,
`VescHandler::sendSpeed()`) применяет deadzone-компенсацию:

```cpp
scaled_duty = min_duty + |target_duty| * (1 - min_duty);
```

Любая ненулевая скорость (даже 0.001 м/с из акселерационной рампы
diff_drive_controller) мапится в duty ≥ `min_duty`. При `min_duty=0.05`:

- **Старт:** первый же тик рампы даёт скачок duty 0 → 5% — рывок + inrush-ток.
- **Торможение через ноль:** за один тик (20 мс) duty переключается
  +5% → −5% (размах 2×min_duty = 10%) — вибрация.
- **Нелинейность в диапазоне 0.1–0.5 м/с:** 0.1 м/с → duty≈7.3%,
  0.5 м/с → ≈16.8%; мелкие ошибки скорости дают крупные изменения duty.

Плюс конфигурационные дрейфы (проверено по задаче «проверить gear_ratio
в конфигурации»):

| Параметр | vesc_config.yaml (standalone) | URDF (runtime, истина) |
|---|---|---|
| gear_ratio | 5.0 ❌ | 2.17 (калибровка 2026-02-28) |
| control_mode | rpm ❌ | duty |
| min_duty | 0.08 ❌ | 0.05 |
| wheel_max_rps | 6.5 ❌ | 12.2 |

Runtime-цепочка: `/cmd_vel` → twist_mux → `diff_drive_controller` →
`VescSystemHardwareInterface` (конфиг из `rob_box_ros2_control.xacro`) → CAN.

## Изменения (PR #1289)

1. **`src/rob_box_description/urdf/rob_box_ros2_control.xacro`** — `min_duty`
   `0.05 → 0.04`: шаг duty при старте и размах реверса 10% → 8% (на 20%
   меньше). Значение 0.04 уже использовалось в калибровке (git log
   `dc1cc00c`), безопасно для преодоления мёртвой зоны (~3-4%).
2. **`docker/main/config/controllers/controller_manager.yaml`** (runtime):
   `linear.x.max_acceleration 1.0 → 0.5 м/с²` (и `min_acceleration -1.0 →
   -0.5`), `angular.z.max_acceleration 3.0 → 2.0 рад/с²` — плавный старт/стоп,
   меньше inrush (программный аналог «снизить acceleration limit в VESC
   Tool»).
3. **`docker/main/config/vesc_nexus/vesc_config.yaml`** — синхронизирован с
   URDF: `gear_ratio 2.17`, `control_mode duty`, `min_duty 0.04`,
   `wheel_max_rps [12.2×4]`.
4. **`docker/main/config/vesc_nexus/robot_controller.yaml`** — зеркало
   runtime-файла: `wheel_separation 1.11`, `base_frame_id base_footprint`,
   `open_loop true`, `max_acceleration 0.5`, `max_velocity 1.5`.
5. **`tests/unit/test_control_config_consistency.py`** — консистентность
   URDF ↔ vesc_config ↔ controller-конфиги + границы duty-маппинга.

## Как проверить на роботе

```bash
# 1. Применить конфиги и перезапустить стек
cd ~/rob_box_project/docker/main
docker compose up -d --force-recreate ros2-control robot-state-publisher

# 2. Плавный старт/стоп на 0.1–0.5 м/с
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
sleep 2
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# 3. Проверить, что нет overcurrent в логах VESC/ros2-control
docker logs ros2-control --since 2m | grep -iE 'overcurrent|faul|err' || echo "OK: нет ошибок"
```

Acceptance (из issue): робот плавно стартует и останавливается без видимого
jitter при 0.1–0.5 м/с; логи VESC не содержат overcurrent при старте.

## e2e

Блок `## e2e` в issue #817: голосовая команда движения
(`voice_text: "Робот, поезжай вперёд"`), харнесс проверит полный цикл
STT→LLM→TTS. Acceptance-pattern: `CMD:move_forward` (диалог публикует
команду движения в `/voice/command` → cmd_vel).
