#!/usr/bin/env python3
"""Источник колёсной одометрии diff_drive_controller — переключатель DIFF_DRIVE_OPEN_LOOP.

``open_loop: true`` (боевое значение в controller_manager.yaml): одометрия
``/diff_drive_controller/odom`` и TF ``odom→base_footprint`` интегрируют КОМАНДЫ.
``open_loop: false``: интегрируют state interface ``velocity`` колёс, который
vesc_nexus считает из eRPM холлов (CAN_PACKET_STATUS): ERPM / pole_pairs /
gear_ratio → рад/с колеса.

Издатель TF один и тот же (diff_drive_controller); переключатель меняет только
источник данных для интегратора, второго ``odom→base_footprint`` не появляется.

Использование (start_ros2_control.sh)::

    render_controller_config.py <src.yaml> <dst.yaml>

печатает путь к конфигу, который надо отдать ros2_control_node:
* переменная не задана или пустая → исходный путь, файл не копируется
  (деплой без переменной ведёт себя ровно как раньше);
* ``true`` / ``false`` → копия с переопределённым ``open_loop`` в <dst.yaml>;
* что-то другое → ошибка с понятным текстом, контейнер не стартует молча
  в непонятном режиме.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml

ENV_VAR = "DIFF_DRIVE_OPEN_LOOP"
CONTROLLER = "diff_drive_controller"
_VALUES = {"true": True, "false": False}


class OpenLoopValueError(ValueError):
    """Значение DIFF_DRIVE_OPEN_LOOP не true/false."""


def parse_open_loop(raw: str | None) -> bool | None:
    """``None`` — переопределения нет; иначе bool. Регистр не важен."""
    if raw is None or raw.strip() == "":
        return None
    value = _VALUES.get(raw.strip().lower())
    if value is None:
        raise OpenLoopValueError(f"{ENV_VAR}={raw!r}: допустимо только true или false (или пусто — значение из yaml)")
    return value


def apply_open_loop(config: dict, open_loop: bool) -> dict:
    """Возвращает конфиг с ``diff_drive_controller.ros__parameters.open_loop``."""
    try:
        params = config[CONTROLLER]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise KeyError(f"в конфиге нет {CONTROLLER}.ros__parameters") from exc
    params["open_loop"] = open_loop
    return config


def render(src: Path, dst: Path, raw: str | None) -> Path:
    """Путь к конфигу для ros2_control_node (src или свежий dst)."""
    open_loop = parse_open_loop(raw)
    if open_loop is None:
        return src
    config = yaml.safe_load(src.read_text(encoding="utf-8"))
    apply_open_loop(config, open_loop)
    dst.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return dst


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(f"usage: {argv[0]} <src.yaml> <dst.yaml>", file=sys.stderr)
        return 2
    try:
        path = render(Path(argv[1]), Path(argv[2]), os.environ.get(ENV_VAR))
    except (OpenLoopValueError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
