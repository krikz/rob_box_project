# rob_box_bringup

Launch-пакет для запуска системы Rob Box: отображение, контроль движения и полная система.

## Описание

Пакет содержит launch-файлы для запуска различных конфигураций системы Rob Box вне Docker (например, для разработки, RViz-визуализации или тестирования компонентов). В production среде компоненты запускаются через Docker Compose (`docker/main/`, `docker/vision/`).

## Launch-файлы

| Файл | Назначение | Ключевые параметры |
|------|-----------|-------------------|
| `display.launch.py` | RViz + robot_state_publisher для визуализации URDF | `gui` (bool), `rviz` (bool) |
| `display_simple.launch.py` | Упрощённая визуализация без joint_state_publisher GUI | — |
| `rob_box_control.launch.py` | ros2_control + VESC CAN драйвер | `port`, `can_id` |
| `complete_system.launch.py` | Полная система: navigation + control + perception | `use_sim_time` (default: false) |

## Запуск

```bash
# Визуализация URDF в RViz (bare metal / dev machine)
ros2 launch rob_box_bringup display.launch.py

# Упрощённая визуализация
ros2 launch rob_box_bringup display_simple.launch.py

# Полная система (для разработки без Docker)
ros2 launch rob_box_bringup complete_system.launch.py

# Production: используйте Docker Compose вместо launch-файлов
cd docker/main && docker compose up -d
```

## Примечание

В production-среде эти launch-файлы **не используются напрямую**. Каждый Docker-контейнер запускает соответствующие ноды через собственные entrypoint-скрипты. Launch-файлы предназначены для:

- Разработки и тестирования на dev-машине
- RViz-визуализации модели робота
- Bare metal запуска для отладки компонентов

## Зависимости

- `rob_box_description` — URDF-модель робота
- `rob_box_perception` — perception ноды
- `vesc_nexus` — VESC motor controller driver
