"""Side test frame (ADR-0150 §5.5.8, A27): which panel is where, and which way it reads.

Mouth: an arrow pointing to the viewer's right, i.e. at the robot's LEFT
headlight. Boards: ``L``/``R`` = the robot's side, digit = panel number
(1 front-left, 2 front-right, 3 rear-left, 4 rear-right). Facing the robot,
the arrow must point at ``L1``; from behind, ``L3`` is on the viewer's left.
Every glyph is asymmetric, so a mirrored or upside-down panel is visible.
"""
from __future__ import annotations

import numpy as np

from .face import GROUP_OF, Frame, col

GLYPHS = {
    'L': ["#..", "#..", "#..", "#..", "###"],
    'R': ["##.", "#.#", "##.", "#.#", "#.#"],
    '1': [".#.", "##.", ".#.", ".#.", "###"],
    '2': ["##.", "..#", ".#.", "#..", "###"],
    '3': ["##.", "..#", ".#.", "..#", "##."],
    '4': ["#.#", "#.#", "###", "..#", "..#"],
}
ARROW = [
    "...................#.....",
    "...................###...",
    "#######################..",
    "...................###...",
    "...................#.....",
]
#: logical group -> (side letter, panel number)
MARKS = {
    'wheel_front_left': ('L', '1'),
    'wheel_front_right': ('R', '2'),
    'wheel_rear_left': ('L', '3'),
    'wheel_rear_right': ('R', '4'),
}
LETTER = col(20, 20, 20)
DIGIT = col(25, 12, 0)
ARROW_C = col(0, 20, 4)


def _blit(arr, glyph, x0, y0, c):
    for y, row in enumerate(glyph):
        for x, ch in enumerate(row):
            if ch == '#':
                arr[y0 + y, x0 + x] = c


def sides_frame() -> Frame:
    f = Frame()
    panel_of = {group: panel for panel, group in GROUP_OF.items()}
    _blit(f.mouth, ARROW, 0, 0, ARROW_C)
    for group, (side, num) in MARKS.items():
        arr = np.zeros((8, 8, 3))
        _blit(arr, GLYPHS[side], 0, 1, LETTER)
        _blit(arr, GLYPHS[num], 4, 1, DIGIT)
        setattr(f, panel_of[group], arr)
    return f
