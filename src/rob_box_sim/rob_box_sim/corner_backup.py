"""Выезд из угла задним ходом (#4607). Чистая функция, без ROS (ADR-0148).

В тупике уже двух ``turn_clearance`` развернуться нельзя: recovery Nav2 по умолчанию
крутит Spin в стену (``navigate_*`` RoundRobin: Spin раньше BackUp). Решение здесь,
не в тексте промпта и не в новой константе «под один угол».

Порог — тот же, что у защиты ``rob_box_teleop.proximity.Thresholds.stop_m`` (0.20 м
в коридоре хода) и у ``coverage_plan.GUARD_STOP_M``. Впереди ближе порога, сзади
не ближе — ``backup``. Иначе ``spin``: открытое место и задний тупик не меняются.
Длина и скорость BackUp — уже стоящие в дереве ``backup_dist`` / ``backup_speed``.
"""

from __future__ import annotations

BACKUP = "backup"
SPIN = "spin"

#: Зеркало ``proximity.Thresholds.stop_m`` и ``coverage_plan.GUARD_STOP_M``.
#: Сим не импортирует teleop (песочница); равенство держит тест.
STOP_M = 0.20


def bt_script() -> str:
    """Условие Script узла BackUp: то же сравнение, что ``corner_escape``.

    Поля — порты BT (зазор впереди/сзади, порог ``backup_stop_m``). Имя результата
    читает ``if`` у BackUp. Порог в формулу не зашит.
    """
    return (
        "corner_escape := (obstacle_ahead < backup_stop_m "
        "&& obstacle_behind >= backup_stop_m) ? 'backup' : 'spin'"
    )


def corner_escape(ahead_m: float, behind_m: float, stop_m: float = STOP_M) -> str:
    """``backup``, если впереди ближе ``stop_m``, а сзади нет. Иначе ``spin``.

    ``ahead_m`` / ``behind_m`` — зазор от корпуса по ходу и против хода, м.
    Отрицательный зазор — препятствие уже в корпусе, это тоже ближе порога.
    ``None`` и нечисло — нет измерения, Spin не подменяется.
    """
    if stop_m <= 0.0:
        raise ValueError(f"stop_m > 0, получено {stop_m}")
    try:
        ahead = float(ahead_m)
        behind = float(behind_m)
    except (TypeError, ValueError):
        return SPIN
    if ahead != ahead or behind != behind:  # NaN
        return SPIN
    if ahead < stop_m and not behind < stop_m:
        return BACKUP
    return SPIN
