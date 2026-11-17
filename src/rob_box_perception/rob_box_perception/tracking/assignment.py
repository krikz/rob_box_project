"""Оптимальное сопоставление треков и детекций (венгерский алгоритм).

Жадный выбор «ближайший первым» на пересечении двух людей может отдать
трек чужой детекции и оставить вторую без пары; венгерский алгоритм
минимизирует сумму стоимостей. Детерминирован: при равных стоимостях
результат зависит только от порядка строк и столбцов.

Реализация — метод потенциалов O(n²·m) без numpy/scipy.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

#: Стоимость «нельзя»: пары с ней в результат не попадают.
_INFEASIBLE = 1.0e9

CostMatrix = Sequence[Sequence[Optional[float]]]


def _augment(cost: List[List[float]], row: int, u: List[float],
             v: List[float], match: List[int]) -> None:
    """Добавить строку ``row`` (1-based) в паросочетание: потенциалы u, v."""
    m = len(cost[0])
    way = [0] * (m + 1)
    minv = [math.inf] * (m + 1)
    used = [False] * (m + 1)
    match[0] = row
    col = 0
    while match[col] != 0:
        used[col] = True
        i0 = match[col]
        delta, nxt = math.inf, 0
        for j in range(1, m + 1):
            if used[j]:
                continue
            reduced = cost[i0 - 1][j - 1] - u[i0] - v[j]
            if reduced < minv[j]:
                minv[j], way[j] = reduced, col
            if minv[j] < delta:
                delta, nxt = minv[j], j
        for j in range(m + 1):
            if used[j]:
                u[match[j]] += delta
                v[j] -= delta
            else:
                minv[j] -= delta
        col = nxt
    while col:
        prev = way[col]
        match[col] = match[prev]
        col = prev


def _solve(cost: List[List[float]]) -> List[Tuple[int, int]]:
    """Решить задачу о назначениях для ``n <= m``; пары (строка, столбец)."""
    n, m = len(cost), len(cost[0])
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    match = [0] * (m + 1)  # match[j] = строка (1-based), занявшая столбец j
    for row in range(1, n + 1):
        _augment(cost, row, u, v, match)
    return sorted((match[j] - 1, j - 1) for j in range(1, m + 1) if match[j])


def assign(costs: CostMatrix) -> List[Tuple[int, int]]:
    """Минимальное по сумме сопоставление строк и столбцов.

    ``costs[i][j]`` — стоимость пары, ``None`` — пара запрещена (вне ворот,
    другой класс). Возвращает отсортированные пары ``(i, j)``; запрещённые
    пары в результат не попадают, такие строки и столбцы остаются без пары.
    """
    if not costs or not costs[0]:
        return []
    dense = [[_INFEASIBLE if c is None else float(c) for c in row]
             for row in costs]
    transposed = len(dense) > len(dense[0])
    if transposed:
        dense = [list(col) for col in zip(*dense)]
    pairs = _solve(dense)
    if transposed:
        pairs = sorted((j, i) for i, j in pairs)
    return [(i, j) for i, j in pairs if costs[i][j] is not None]
