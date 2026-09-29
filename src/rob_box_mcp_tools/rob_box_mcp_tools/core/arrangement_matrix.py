"""arrangement_matrix.py — аранжировка трека матрицей «слой × 2-тактовый блок».

План: docs/design/2026-09-28-dj-live-coding-quality-plan.md, §7.10 и §7.6 п.10.

Идиома ``mask4`` из референса товарища Шифу (Strudel, froos «theres no soul
here»): вся форма трека — одна матрица небольших целых чисел. Строка — слой
(kick, bass, lead …), столбец — блок из ``block_bars`` тактов (по умолчанию 2),
ячейка — 4-битная маска четвертей блока, старший бит = ПЕРВАЯ четверть:

    x / 15 = 1111 — играет весь блок
    0      = 0000 — блок молчит
    1      = 0001 — только последняя четверть (затакт перед входом)
    7      = 0111 — выпадает первая четверть (брейк)
    9      = 1001 — дыра посередине (филл)
    12     = 1100 — вторая половина молчит (пред-дроп «вдох»)
    14     = 1110 — молчит последняя четверть

Матрица детерминированно рендерится в FoxDot ``var([уровни], [доли])`` —
тот же приём, что уже звучит вживую в :mod:`core.arranger`
(``amp=var([0, 0.41, 0, ...], [56, 84, ...])``). ``var`` зацикливается после
суммарной длительности, поэтому матрица длиной N блоков повторяется каждые
``total_beats`` долей.

Модуль чистый: без Renardo/ROS — только строки и числа.

Санитайзер: ``_cap_amp`` в :mod:`core.renardo_sanitizer` капает ``amp=N``,
``amp=P[...]``, ``amplify=var(...)``, но НЕ трогает ``amp=var(...)`` (регэксп
``amp\\s*=\\s*(\\d+...)`` не матчит ``var``). Значит уровень, переданный в
:meth:`ArrangementMatrix.gate_var`, уходит в Renardo как есть — вызывающий
сам отвечает за потолок громкости (или кладёт гейт в ``amplify=``).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import gcd
from types import MappingProxyType
from typing import Dict, List, Mapping, Sequence, Tuple, Union

QUARTERS = 4
FULL = 15

Cell = Union[int, str]


def parse_cell(cell: Cell) -> int:
    """Ячейка матрицы → маска 0..15. ``'x'`` = 15 (весь блок)."""
    if isinstance(cell, bool):
        raise ValueError(f"Ячейка матрицы не может быть bool: {cell!r} (нужно 'x' или 0..15)")
    if isinstance(cell, int):
        value = cell
    elif isinstance(cell, str):
        token = cell.strip().lower()
        if token == "x":
            return FULL
        if not token.isdigit():
            raise ValueError(f"Неизвестная ячейка матрицы: {cell!r} (нужно 'x' или число 0..15)")
        value = int(token)
    else:
        raise ValueError(f"Ячейка матрицы должна быть int или str, получено {type(cell).__name__}: {cell!r}")
    if not 0 <= value <= FULL:
        raise ValueError(f"Маска ячейки вне диапазона 0..15: {cell!r}")
    return value


def _expand_token(token: str) -> List[int]:
    """``'x!4'`` → [15, 15, 15, 15]; ``'7'`` → [7]."""
    if "!" not in token:
        return [parse_cell(token)]
    value, _, count = token.partition("!")
    if not count.isdigit() or int(count) < 1:
        raise ValueError(f"Повтор '!n' требует целое n ≥ 1: {token!r}")
    return [parse_cell(value)] * int(count)


def parse_lane(spec: str) -> Tuple[int, ...]:
    """Строка слоя ``"0 0 1 x x!4 7 x 9"`` → кортеж масок.

    Разделитель — пробелы; ``v!n`` — повтор ``v`` n раз (как в Strudel).
    Для удобства вставки референса допускаются обрамляющие ``< >``.
    """
    if not isinstance(spec, str):
        raise ValueError(f"Слой матрицы задаётся строкой, получено {type(spec).__name__}")
    body = spec.strip()
    if body.startswith("<") and body.endswith(">"):
        body = body[1:-1]
    cells: List[int] = []
    for token in body.split():
        cells.extend(_expand_token(token))
    if not cells:
        raise ValueError(f"Пустой слой матрицы: {spec!r}")
    return tuple(cells)


def cell_quarters(mask: int) -> Tuple[bool, bool, bool, bool]:
    """Маска → (q1, q2, q3, q4); старший бит — первая четверть блока."""
    mask = parse_cell(mask)
    return tuple(bool(mask >> (QUARTERS - 1 - i) & 1) for i in range(QUARTERS))  # type: ignore[return-value]


def cycle_lane(masks: Tuple[int, ...], n_blocks: int) -> Tuple[int, ...]:
    """Зациклить слой до длины ``n_blocks`` (обрезая лишнее) — как Strudel ``<...>``."""
    if not masks:
        raise ValueError("Нельзя зациклить пустой слой")
    if n_blocks < 1:
        raise ValueError(f"Длина слоя должна быть ≥ 1, получено {n_blocks}")
    return tuple(masks[i % len(masks)] for i in range(n_blocks))


def _fmt_num(value: float, digits: int = 6) -> str:
    """Число для FoxDot-кода: целое без точки, дробь без хвостовых нулей."""
    rounded = round(float(value), digits)
    if rounded.is_integer():
        return str(int(rounded))
    return f"{rounded:.{digits}f}".rstrip("0").rstrip(".")


def _normalize_lane(name: str, masks: Tuple[Cell, ...]) -> Tuple[int, ...]:
    if not isinstance(name, str) or not name:
        raise ValueError(f"Имя слоя должно быть непустой строкой: {name!r}")
    cells = tuple(parse_cell(m) for m in masks)
    if not cells:
        raise ValueError(f"Слой '{name}' пуст")
    return cells


def _render_cell(mask: int) -> str:
    return "".join("#" if on else "." for on in cell_quarters(mask))


@dataclass(frozen=True)
class ArrangementMatrix:
    """Матрица аранжировки: слой → маски по блокам (все слои одной длины)."""

    lanes: Mapping[str, Tuple[int, ...]]
    block_bars: int = 2
    beats_per_bar: int = 4

    def __post_init__(self) -> None:
        if not isinstance(self.block_bars, int) or self.block_bars < 1:
            raise ValueError(f"block_bars должен быть целым ≥ 1, получено {self.block_bars!r}")
        if not isinstance(self.beats_per_bar, (int, float)) or self.beats_per_bar <= 0:
            raise ValueError(f"beats_per_bar должен быть > 0, получено {self.beats_per_bar!r}")
        if not self.lanes:
            raise ValueError("Матрица аранжировки без слоёв")
        normalized = {name: _normalize_lane(name, masks) for name, masks in self.lanes.items()}
        lengths = {name: len(cells) for name, cells in normalized.items()}
        if len(set(lengths.values())) != 1:
            raise ValueError(f"Слои матрицы разной длины (в блоках): {lengths}")
        object.__setattr__(self, "lanes", MappingProxyType(normalized))

    @classmethod
    def from_specs(
        cls, specs: Mapping[str, str], block_bars: int = 2, beats_per_bar: int = 4
    ) -> "ArrangementMatrix":
        """Собрать матрицу из строк слоёв (см. :func:`parse_lane`)."""
        lanes = {name: parse_lane(spec) for name, spec in specs.items()}
        return cls(lanes=lanes, block_bars=block_bars, beats_per_bar=beats_per_bar)

    @property
    def n_blocks(self) -> int:
        return len(next(iter(self.lanes.values())))

    @property
    def block_beats(self) -> float:
        return self.block_bars * self.beats_per_bar

    @property
    def quarter_beats(self) -> float:
        return self.block_beats / QUARTERS

    @property
    def total_beats(self) -> float:
        return self.n_blocks * self.block_beats

    def _lane(self, lane: str) -> Tuple[int, ...]:
        if lane not in self.lanes:
            raise ValueError(f"Нет слоя '{lane}' в матрице (есть: {', '.join(self.lanes)})")
        return self.lanes[lane]

    def gate_segments(self, lane: str) -> List[Tuple[int, float]]:
        """Слитые отрезки (0|1, доли); соседние равные склеены, сумма = total_beats."""
        segments: List[List[float]] = []
        for mask in self._lane(lane):
            for on in cell_quarters(mask):
                state = 1 if on else 0
                if segments and segments[-1][0] == state:
                    segments[-1][1] += self.quarter_beats
                else:
                    segments.append([state, self.quarter_beats])
        return [(int(state), beats) for state, beats in segments]

    def gate_var(self, lane: str, level: float = 1.0, digits: int = 3) -> str:
        """FoxDot-выражение гейта слоя: ``"var([0, 0.7, 0], [8, 24, 32])"``.

        Весь слой звучит → просто число ``"0.7"``; весь молчит → ``"0"``.
        Внимание: ``amp=var(...)`` санитайзер НЕ капает (см. docstring модуля).
        """
        if level < 0:
            raise ValueError(f"Уровень гейта не может быть отрицательным: {level}")
        on_text = _fmt_num(level, digits)
        segments = self.gate_segments(lane)
        if len(segments) == 1:
            return on_text if segments[0][0] == 1 else "0"
        values = ", ".join(on_text if state else "0" for state, _ in segments)
        durations = ", ".join(_fmt_num(beats) for _, beats in segments)
        return f"var([{values}], [{durations}])"

    def gate_var_blocks(self, lane: str, levels: Sequence[float], digits: int = 3) -> str:
        """Гейт слоя с уровнем ПО БЛОКАМ (issue #3154): ``levels[i]`` — уровень блока i.

        Как :meth:`gate_var`, но звучащая четверть блока берёт уровень своего
        блока; соседние отрезки с одинаковым значением склеены. Все блоки на
        одном уровне → результат побайтно равен ``gate_var(lane, level)``.
        """
        masks = self._lane(lane)
        if len(levels) != len(masks):
            raise ValueError(f"Уровней {len(levels)}, а блоков в слое '{lane}' {len(masks)}")
        if any(level < 0 for level in levels):
            raise ValueError(f"Уровень гейта не может быть отрицательным: {list(levels)}")
        segments: List[List] = []
        for mask, level in zip(masks, levels):
            text = _fmt_num(level, digits)
            for on in cell_quarters(mask):
                value = text if on else "0"
                if segments and segments[-1][0] == value:
                    segments[-1][1] += self.quarter_beats
                else:
                    segments.append([value, self.quarter_beats])
        if len(segments) == 1:
            return segments[0][0]
        values = ", ".join(value for value, _ in segments)
        durations = ", ".join(_fmt_num(beats) for _, beats in segments)
        return f"var([{values}], [{durations}])"

    def active_blocks(self, lane: str) -> List[int]:
        """Индексы блоков, где слой звучит хотя бы одну четверть."""
        return [i for i, mask in enumerate(self._lane(lane)) if mask]

    def to_text(self) -> str:
        """Сетка «как в DAW»: строка на слой, ``#`` — четверть звучит, ``.`` — молчит.

        Блоки сгруппированы по фразам в 8 тактов (``|`` между группами).
        """
        per_phrase = max(1, 8 // self.block_bars)
        width = max(len(name) for name in self.lanes)
        header = (
            f"{self.n_blocks} блоков × {self.block_bars} такта, "
            f"{_fmt_num(self.total_beats)} долей; '#' = четверть звучит"
        )
        lines = [header]
        for name, masks in self.lanes.items():
            groups = []
            for start in range(0, len(masks), per_phrase):
                groups.append(" ".join(_render_cell(m) for m in masks[start:start + per_phrase]))
            lines.append(f"{name.ljust(width)} | " + " | ".join(groups))
        return "\n".join(lines)


def _lcm(a: int, b: int) -> int:
    return a * b // gcd(a, b)


def _cycled_spec(spec: str, n_blocks: int) -> str:
    return " ".join("x" if m == FULL else str(m) for m in cycle_lane(parse_lane(spec), n_blocks))


# ---------------------------------------------------------------------------
# Готовые шаблоны (референсы товарища Шифу)
# ---------------------------------------------------------------------------

# DJ Dave-style, 16 блоков × 2 такта = 32 такта; 4 фразы по 8 тактов:
#   блоки 0-3   build   — раскрытие: хэты с нуля, бас с блока 1, бочка
#                         с затактом (1) в блоке 1 и целиком с блока 2;
#   блоки 4-7   predrop — бочка выпадает: 14 (последняя четверть пустая),
#                         затем 0 — «вдох» перед дропом, бас 12, клэп-затакт 1;
#   блоки 8-11  drop    — все слои;
#   блоки 12-15 verse   — тоньше: без перкуссии, клэп через блок, лид позже.
# Последний блок каждой 8-тактовой фразы — брейк 7/9 на одном слое:
#   блок 3 хэты 7, блок 7 перкуссия 9, блок 11 клэп 7, блок 15 бочка 9.
_DJ_DAVE_32: Dict[str, str] = {
    "kick": "0 1 x x   x x 14 0   x x x x   x x x 9",
    "clap": "0 0 0 0   0 x x 1    x x x 7   0 x 0 x",
    "hats": "x x x 7   x x x x    x x x x   x x x x",
    "perc": "0 0 0 0   0 0 x 9    x x x x   0 0 0 0",
    "bass": "0 x x x   x x 12 0   x x x x   x x x x",
    "lead": "0 0 0 0   0 0 x x    x x x x   0 0 x x",
    "pad": "x!16",
}

# froos «theres no soul here» — слои референса ДОСЛОВНО (Strudel, `/2` =
# ячейка на 2 цикла = 2 такта). В оригинале длины РАЗНЫЕ: drums 22,
# guitar 24, piano 24, bass 28 — Strudel крутит каждый ``<...>`` независимо.
LOFI_FROOS_ORIGINAL: Dict[str, str] = {
    "drums": "0 0 0 1 x!4 7 x x x 9 x x x x x 0 0 0 0",
    "guitar": "0 0 0 0 0!4 x x x x x x x x x x x 9 x x x x",
    "piano": "x!24",
    "bass": "0 0 x x x!4 x x x x x x x x 0 0 0 1 x x x x x x x x",
}

# Решение по длинам: ArrangementMatrix требует равной длины (один var на
# слой = один общий цикл). Честное общее кратное lcm(22, 24, 28) = 1848
# блоков (3696 тактов) — бессмысленно для живого сета. Поэтому шаблон —
# первые max(len) = 28 блоков (56 тактов) РОВНО так, как их сыграл бы
# Strudel: короткие слои зациклены (drums на блоке 22 снова начинает
# «0 0 0 1 x x», guitar/piano на 24 — снова с начала). Бас не обрезан.
# Маппинг: drums → kick/clap/hats (одна маска на все барабаны, как в
# референсе), guitar → lead, piano → pad, perc в референсе нет → 0.
_LOFI_BLOCKS = max(len(parse_lane(s)) for s in LOFI_FROOS_ORIGINAL.values())
_LOFI_FROOS: Dict[str, str] = {
    "kick": _cycled_spec(LOFI_FROOS_ORIGINAL["drums"], _LOFI_BLOCKS),
    "clap": _cycled_spec(LOFI_FROOS_ORIGINAL["drums"], _LOFI_BLOCKS),
    "hats": _cycled_spec(LOFI_FROOS_ORIGINAL["drums"], _LOFI_BLOCKS),
    "perc": f"0!{_LOFI_BLOCKS}",
    "bass": _cycled_spec(LOFI_FROOS_ORIGINAL["bass"], _LOFI_BLOCKS),
    "lead": _cycled_spec(LOFI_FROOS_ORIGINAL["guitar"], _LOFI_BLOCKS),
    "pad": _cycled_spec(LOFI_FROOS_ORIGINAL["piano"], _LOFI_BLOCKS),
}

# Issue #3113 (живой прогон 28.09): три DJ-трека подряд на одном шаблоне
# dj_dave_32 звучали однотипно. Ещё два клубных шаблона той же длины
# (16 блоков × 2 такта = 32 такта, те же 7 слоёв) — сид club выбирает
# между ними (core/club_arranger.club_kit).
#
# drop_first_32 — трек входит сразу дропом (после фейда уходящего трека
# энергия не проваливается), потом брейкдаун без бочки и повторный разгон:
#   блоки 0-3   drop      — все слои; блок 3: бочка 9, клэп 7, перкуссия 9;
#   блоки 4-7   breakdown — без бочки/клэпа: пэд + лид, хэты возвращаются
#                           с затакта (1) в блоке 5, бас-затакт (1) в блоке 7;
#   блоки 8-11  build     — бочка/бас снова, пред-дроп 14 → 0 (бас 12 → 0);
#   блоки 12-15 drop2     — все слои, брейк 7 на клэпе в последнем блоке.
_DROP_FIRST_32: Dict[str, str] = {
    "kick": "x x x 9   0 0 0 1   x x 14 0   x x x x",
    "clap": "x x x 7   0 0 0 0   0 x x 1    x x x 7",
    "hats": "x x x x   0 1 x 7   x x x x    x x x x",
    "perc": "x x x 9   0 0 0 0   0 0 x 9    x x x x",
    "bass": "x x x x   0 0 0 1   x x 12 0   x x x x",
    "lead": "x x x x   x x 0 0   0 0 x x    x x x x",
    "pad": "x!16",
}

# long_build_32 — длинный разгон, лид приберегается до пред-дропа:
#   блоки 0-3   intro — пэд, хэты с затакта (1) в блоке 2, бочка-затакт в 3;
#   блоки 4-7   build — бочка, перкуссия, бас с затакта (1) в блоке 6;
#   блоки 8-11  lift  — клэп с блока 9, лид с блока 10 (под растущий hpf),
#                       пред-дроп: бочка 12 → 0, бас 12 → 0;
#   блоки 12-15 drop  — все слои, брейк 9 у бочки в последнем блоке.
_LONG_BUILD_32: Dict[str, str] = {
    "kick": "0 0 0 1   x x x x   x x 12 0   x x x 9",
    "clap": "0 0 0 0   0 0 0 0   0 x x 1    x x x 7",
    "hats": "0 0 1 x   x x x 7   x x x x    x x x x",
    "perc": "0 0 0 0   x x x 9   x x 0 0    x x x x",
    "bass": "0 0 0 0   0 0 1 x   x x 12 0   x x x x",
    "lead": "0 0 0 0   0 0 0 0   0 0 x x    x x x x",
    "pad": "x!16",
}

SECTION_TEMPLATES: Dict[str, Dict[str, str]] = {
    "dj_dave_32": _DJ_DAVE_32,
    "lofi_froos": _LOFI_FROOS,
    "drop_first_32": _DROP_FIRST_32,
    "long_build_32": _LONG_BUILD_32,
}


def original_cycle_blocks(specs: Mapping[str, str]) -> int:
    """Длина полного цикла слоёв разной длины (lcm), в блоках — для справки."""
    total = 1
    for spec in specs.values():
        total = _lcm(total, len(parse_lane(spec)))
    return total


__all__ = [
    "ArrangementMatrix",
    "Cell",
    "FULL",
    "LOFI_FROOS_ORIGINAL",
    "QUARTERS",
    "SECTION_TEMPLATES",
    "cell_quarters",
    "cycle_lane",
    "original_cycle_blocks",
    "parse_cell",
    "parse_lane",
]
