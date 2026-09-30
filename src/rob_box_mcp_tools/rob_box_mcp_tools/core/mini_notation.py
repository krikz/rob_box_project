"""mini_notation.py — честное подмножество mini-notation Strudel/Tidal (issue #3227).

Разбирает строку из ``note("...")`` / ``n("...")`` в события ``(старт, длина,
токен)`` в ЦИКЛАХ (``Fraction``; цикл = такт из 4 долей — соглашение Strudel
``setcpm(bpm/4)``).

Поддержано: последовательность через пробел; ``[a b]`` — подгруппа (делит
отведённое время); ``a, b`` внутри ``[]`` — стек (аккорд/слои); ``*N`` — N
повторов в отведённом времени; ``@w`` — вес шага; ``!N`` / ``!`` — повтор
шага; ``_`` — продлить предыдущий шаг; ``~`` и ``-`` — пауза; ``<a b c>`` —
чередование по циклам (разворачивается ПОСЛЕДОВАТЕЛЬНО: шаг с весом ``w``
занимает ``w`` циклов, содержимое шага сжато в это время; ``<e2,g2,b2>`` —
стек из одношаговых чередований, то есть аккорд, как в Strudel).

Чередование ``<...>`` ВНУТРИ более быстрой группы (``[a <b c>]``) не
разворачивается: берётся первый шаг (цикл 0) — это единственная
неоднозначность, остальное соответствует Strudel.

Всё, чего здесь нет (``/N``, ``?``, ``|``, ``(3,8)``, ``{}``, ``%``, ``..``,
``:``), — :class:`Unsupported`: разбор отвергается целиком, ноты не выдумываются.
"""

from __future__ import annotations

import re
from fractions import Fraction
from typing import Callable, List, Tuple

__all__ = ["Event", "Unsupported", "parse_events"]

Event = Tuple[Fraction, Fraction, str]

#: Верхняя граница числа циклов чередования и событий (защита от патологий).
MAX_CYCLES = 64
MAX_EVENTS = 4096

_ATOM = re.compile(r"[A-Za-z0-9#.\-]+")
_NUMBER = re.compile(r"\d+(?:\.\d+)?")


class Unsupported(ValueError):
    """Конструкция вне поддерживаемого подмножества."""


class _Parser:
    """Рекурсивный спуск по строке; AST — вложенные кортежи."""

    def __init__(self, text: str) -> None:
        self.s = text
        self.i = 0

    def _peek(self) -> str:
        return self.s[self.i] if self.i < len(self.s) else ""

    def _skip_ws(self) -> None:
        while self._peek() and self._peek().isspace():
            self.i += 1

    def _number(self) -> Fraction:
        m = _NUMBER.match(self.s, self.i)
        if not m:
            raise Unsupported(f"ожидалось число в позиции {self.i}: {self.s[self.i:self.i + 8]!r}")
        self.i = m.end()
        return Fraction(m.group())

    def layers(self, closer: str) -> list:
        """Слои через запятую до ``closer`` (``""`` — до конца строки)."""
        out = [self.sequence(closer)]
        while self._peek() == ",":
            self.i += 1
            out.append(self.sequence(closer))
        return out

    def sequence(self, closer: str) -> tuple:
        steps: List[Tuple[tuple, Fraction]] = []
        while True:
            self._skip_ws()
            ch = self._peek()
            if ch == "" or ch == "," or ch == closer:
                break
            self._step(steps)
        if not steps:
            raise Unsupported("пустая группа")
        return ("seq", steps)

    def _step(self, steps: list) -> None:
        ch = self._peek()
        if ch == "_":
            self.i += 1
            if not steps:
                raise Unsupported("'_' в начале последовательности")
            node, w = steps[-1]
            steps[-1] = (node, w + 1)
            return
        node = self._atom_or_group()
        weight = Fraction(1)
        repeat = 1
        while True:
            ch = self._peek()
            if ch == "*":
                self.i += 1
                factor = self._number()
                if factor.denominator != 1 or factor < 1:
                    raise Unsupported(f"*{factor}: только целое число повторов")
                node = ("fast", node, int(factor))
            elif ch == "@":
                self.i += 1
                weight = self._number()
            elif ch == "!":
                self.i += 1
                m = _NUMBER.match(self.s, self.i)
                repeat = 2 if not m else int(self._number())
            else:
                break
        if self._peek() in ("/", "?", "|", "(", "{", "%", ":"):
            raise Unsupported(f"оператор {self._peek()!r} вне подмножества")
        steps.extend([(node, weight)] * repeat)

    def _atom_or_group(self) -> tuple:
        ch = self._peek()
        if ch == "[":
            self.i += 1
            inner = self.layers("]")
            self._expect("]")
            return _stack(inner)
        if ch == "<":
            self.i += 1
            inner = self.layers(">")
            self._expect(">")
            alts = [("alt", seq[1]) for seq in inner]
            return alts[0] if len(alts) == 1 else ("stack", alts)
        if ch == "~":
            self.i += 1
            return ("rest",)
        m = _ATOM.match(self.s, self.i)
        if not m:
            raise Unsupported(f"неожиданный символ {ch!r} в позиции {self.i}")
        self.i = m.end()
        tok = m.group()
        return ("rest",) if tok in ("~", "-") else ("atom", tok)

    def _expect(self, ch: str) -> None:
        self._skip_ws()
        if self._peek() != ch:
            raise Unsupported(f"не закрыта скобка: ожидалось {ch!r}")
        self.i += 1


def _stack(layers: list) -> tuple:
    return layers[0] if len(layers) == 1 else ("stack", layers)


def _parse(text: str) -> tuple:
    p = _Parser(text)
    tree = _stack(p.layers(""))
    p._skip_ws()
    if p.i < len(p.s):
        raise Unsupported(f"лишнее в позиции {p.i}: {p.s[p.i:p.i + 8]!r}")
    while tree[0] == "seq" and len(tree[1]) == 1 and tree[1][0][1] == 1 and tree[1][0][0][0] != "atom":
        tree = tree[1][0][0]
    return tree


def _total_weight(steps: list) -> Fraction:
    return sum((w for _n, w in steps), Fraction(0))


def _cycles(node: tuple) -> Fraction:
    """Сколько циклов занимает верхнеуровневый узел (чередование — сумма весов)."""
    if node[0] == "alt":
        return _total_weight(node[1])
    if node[0] == "stack":
        return max(_cycles(c) for c in node[1])
    return Fraction(1)


def _render(node: tuple, start: Fraction, span: Fraction, top: bool, emit: Callable) -> None:
    kind = node[0]
    if kind == "atom":
        emit(start, span, node[1])
    elif kind == "seq":
        _render_seq(node[1], start, span, top, emit)
    elif kind == "stack":
        for child in node[1]:
            _render(child, start, span, top, emit)
    elif kind == "fast":
        n = node[2]
        for k in range(n):
            _render(node[1], start + span * k / n, span / n, False, emit)
    elif kind == "alt":
        _render_alt(node[1], start, span, top, emit)


def _render_seq(steps: list, start: Fraction, span: Fraction, top: bool, emit: Callable) -> None:
    total = _total_weight(steps)
    if top and len(steps) == 1 and steps[0][0][0] in ("alt", "stack"):
        _render(steps[0][0], start, span, True, emit)
        return
    pos = start
    for child, w in steps:
        _render(child, pos, span * w / total, False, emit)
        pos += span * w / total


def _render_alt(steps: list, start: Fraction, span: Fraction, top: bool, emit: Callable) -> None:
    if not top:
        _render(steps[0][0], start, span, False, emit)
        return
    total = _total_weight(steps)
    if total > MAX_CYCLES:
        raise Unsupported(f"чередование длиннее {MAX_CYCLES} циклов")
    pos = start
    for child, w in steps:
        _render(child, pos, span * w, False, emit)
        pos += span * w


def parse_events(text: str) -> Tuple[List[Event], Fraction]:
    """Строка mini-notation → ``(события, длина_в_циклах)``.

    Raises:
        Unsupported: синтаксис вне подмножества (см. модульный докстринг).
    """
    tree = _parse(text)
    cycles = _cycles(tree)
    events: List[Event] = []

    def emit(start: Fraction, dur: Fraction, tok: str) -> None:
        if len(events) >= MAX_EVENTS:
            raise Unsupported(f"больше {MAX_EVENTS} событий")
        events.append((start, dur, tok))

    _render(tree, Fraction(0), Fraction(1), True, emit)
    if tree[0] == "stack" and cycles > 1:
        _loop_short_alts(tree, events, cycles)
    return events, cycles


def _loop_short_alts(tree: tuple, events: List[Event], cycles: Fraction) -> None:
    """Слой-чередование короче самого длинного повторяется (как в Strudel)."""
    for child in tree[1]:
        length = _cycles(child)
        if child[0] != "alt" or length >= cycles or length <= 0:
            continue
        layer: List[Event] = []
        _render(child, Fraction(0), Fraction(1), True, lambda s, d, t: layer.append((s, d, t)))
        shift = length
        while shift < cycles:
            events.extend((s + shift, d, t) for s, d, t in layer)
            shift += length
