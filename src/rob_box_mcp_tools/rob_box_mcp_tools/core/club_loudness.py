"""club_loudness.py — статическая модель громкости club и калибровка уровней (issue #3154).

Живые замеры (issue #3154, jack_rec на выходе scsynth, ``music_master_gain``
один и тот же): треки одного DJ-сета расходились до ~30 dB (превью
``long_build_32`` −52…−55 dB RMS, следующий ``drop_first_32`` −22…−25),
club против classic «К Элизе» ~17 dB.

Откуда разница (офлайн-рендер, см. ниже): уровни слоёв в
:data:`core.club_arranger.LAYER_LEVELS` — амплитуды ``amp``, а громкость
слоя на выходе зависит от синта/сэмпла на десятки dB. Бочка и бас звучат
около −24 dB, а пэд ``sinepad`` на той же шкале −74 dB, лид ``marimba`` −69,
хэты −68…−74. Весь микс держат бочка и бас; секции без бочки (интро
``long_build_32``, брейкдаун ``drop_first_32``, «вдох» пред-дропа) падают
на 30–50 dB.

Модель
======

Уровень слоя, когда он звучит весь блок, — :data:`LANE_DB_AT_UNIT` (dB RMS
при ``amp``-гейте 1.0) плюс ``20·p·log10(уровень)``, где ``p`` —
:data:`AMP_EXPONENT` (у ``dub``/``karp``/``sinepad`` ``amp`` входит в синт
дважды, громкость растёт быстрее амплитуды). Слой, звучащий ``q`` четвертей
из 4, даёт ``q/4`` мощности. Уровень блока — сумма мощностей слоёв
(некоррелированные источники). «Основной блок» — блоки, где бочка звучит
весь блок (маска ``x``); его уровень — средняя мощность таких блоков.

Откуда числа — МОДЕЛЬ, НЕ ЗАМЕР НА РОБОТЕ
=========================================

:data:`LANE_DB_AT_UNIT` и :data:`AMP_EXPONENT` сняты офлайн-рендером
``scripts/music/club_loudness_nrt.py --sweep`` (29.09.2026): каждый слой
``render_club_kit`` в одиночку через НАСТОЯЩИЕ SynthDef'ы renardo_lib
0.9.13, сэмплы ``0_foxdot_default`` и ``masterfilter.scd`` (gain 0.5) в
scsynth 3.14.1 NRT на 16 кГц, шаблон ``dj_dave_32`` блоки 8–10, 124 BPM,
среднее по тоникам A#, D, A, E, G (разброс по тоникам ≤ 1.2 dB, у пэдов
до 3 dB). Показатель ``p`` — по второму прогону с уровнями ×0.5.

Сверка рендера с живыми замерами: эталон seed=0 @128 BPM рендер −20…−22 dB
RMS по блокам, пик −9.0 dBFS; живой jack_rec 28.09 (``n1_club.wav``)
−24…−26 dB, пик −9.7 dBFS. Шкала модели ≈ на 2–4 dB громче живой записи
по RMS (окна разные: блок 8 долей против 0,1 с), по пику совпадает.

Калибровка (:func:`calibrate_levels`)
=====================================

1. **Стиль** — основной блок club приводится к :data:`TARGET_MAIN_DB`, уровню
   громкой секции classic «К Элизе» в той же шкале (:data:`STYLE_MAIN_DB`).
   Это общий множитель стиля club против classic: classic не трогается,
   club опускается к нему (до калибровки club был на ~6 dB громче громкой
   секции classic и на ~15–20 dB громче её тихих секций).
2. **Каркас** — один множитель на все слои каркаса (шаблон + бочка + синты),
   чтобы основной блок любого сида был на :data:`TARGET_MAIN_DB`.
3. **Секции** — в блоках без полной бочки, которые тише основного больше чем
   на :data:`SECTION_FLOOR_DB`, поднимаются слои фактуры
   :data:`LIFT_LANES` (пэд, лид, хэты) — «пэд раскрывается, когда бочка
   уходит». Бочка, бас и клэп не поднимаются выше своего основного уровня.

Всё — в пределах потолка слоя ``MAX_LAYER_AMP`` (0.85, как ``max_amp``
санитайзера): где синт не дотягивает даже на потолке (``sinepad`` — самый
тихий), модель это видит (:func:`kit_report`), и тест фиксирует остаток,
а не прячет его.

Шкала модели — мастер БЕЗ радио-динамики (``masterfilter`` ``dyn = 0``).
На роботе поверх неё работает выравниватель/компрессор/лимитер мастер-шины
(issue #3154, ``masterfilter.scd``): он сжимает оставшийся разброс секций
примерно втрое и поднимает общий уровень; модель и калибровка его НЕ
учитывают — их задача, чтобы до мастера доходил уже ровный микс.
"""

from __future__ import annotations

import math
from typing import Dict, List, Mapping, Sequence, Tuple

from .arrangement_matrix import FULL, QUARTERS, ArrangementMatrix, cell_quarters

#: Условия снятия :data:`LANE_DB_AT_UNIT` (для docstring'ов и отчёта).
MODEL_SOURCE = (
    "офлайн-рендер scsynth 3.14.1 NRT 16 кГц, renardo_lib 0.9.13, masterfilter gain=0.5 dyn=0, "
    "dj_dave_32 блоки 8-10, 124 BPM, среднее по 5 тоникам (29.09.2026); не замер на роботе"
)

#: dB RMS слоя, звучащего весь блок, при ``amp``-гейте 1.0 (пампинг
#: ``amplify`` у лида/баса — как в ``render_club_kit``). Получено из замера
#: при уровне :data:`core.club_arranger.LAYER_LEVELS`: ``dB − 20·p·log10(L)``.
#: Ключ слоя ударных — рисунок (``KICK_PATTERNS``/``HATS_PATTERNS``), клэп
#: один (``clap``), мелодических — синт.
_MEASURED_DB: Dict[str, Tuple[float, Dict[str, float]]] = {
    # слой: (уровень замера, {вариант: dB RMS})
    "kick": (0.6, {"by_design": -24.1, "four_on_floor": -24.6, "half_time": -27.6,
                   "breakbeat": -24.6, "outrun": -23.7}),
    "hats": (0.14, {"by_design": -68.4, "offbeat": -74.0, "eighths": -70.9, "shuffle": -70.0}),
    "clap": (0.24, {"clap": -55.7}),
    "lead": (0.28, {"pluck": -53.4, "blip": -55.0, "arpy": -56.3, "karp": -66.3, "marimba": -68.9}),
    "bass": (0.4, {"bass": -24.4, "retrobass": -41.9, "dub": -24.8}),
    "pad": (0.11, {"sinepad": -74.4, "warmpad": -43.1, "space": -54.5}),
}

#: Как громкость синта растёт с ``amp``: ``dB = 20·p·log10(amp)``. Замер —
#: кривая «слой в одиночку на постоянном amp» 0.05/0.11/0.3/0.6/0.85
#: (офлайн-рендер, 29.09): у ``dub`` (``amp*2`` и в осцилляторе, и в
#: огибающей), ``karp`` и ``sinepad`` (``amp`` в ``mul`` и в огибающей)
#: наклон 2 на участке 0.11…0.85 (у ``dub`` выше 0.3 — 1.6: tanh мастера);
#: у остальных 1.0 (pluck, marimba, space, warmpad, хэты — сняты кривой,
#: прочие — ×1 против ×0.5).
AMP_EXPONENT: Dict[str, float] = {"dub": 2.0, "karp": 2.0, "sinepad": 2.0}


def _exponent(option: str) -> float:
    return AMP_EXPONENT.get(option, 1.0)


LANE_DB_AT_UNIT: Dict[str, Dict[str, float]] = {
    lane: {
        option: round(db - 20.0 * _exponent(option) * math.log10(level), 2)
        for option, db in table.items()
    }
    for lane, (level, table) in _MEASURED_DB.items()
}

#: Уровень громкой секции classic «К Элизе» (pluck/bass/warmpad, блоки
#: 21–27 формы «arc», бас + бочка) в той же шкале офлайн-рендера. Тихие
#: секции того же трека −37…−43 dB — это форма classic, её эта калибровка
#: не трогает.
STYLE_MAIN_DB: Dict[str, float] = {"classic": -28.4}

#: Цель основного блока club (dB RMS в шкале модели) — уровень classic.
TARGET_MAIN_DB = STYLE_MAIN_DB["classic"]
#: Секция без полной бочки не тише основного блока больше чем на столько.
#: Приёмка issue #3154 — «~10 dB»; 2 dB запаса на ошибку модели.
SECTION_FLOOR_DB = 8.0
#: Слои, которые поднимаются в тихих секциях (фактура, не ритм).
LIFT_LANES: Tuple[str, ...] = ("pad", "lead", "hats")
#: Ударные и роли, чей вариант задаёт каркас (``club_kit``).
KIT_KEY: Dict[str, str] = {"kick": "kick", "hats": "hats", "lead": "lead", "bass": "bass", "pad": "pad"}

_SILENCE_DB = -200.0
_BISECT_STEPS = 40


def lane_option(kit: Mapping[str, str], lane: str) -> str:
    """Вариант слоя в каркасе: рисунок ударных, ``clap`` или синт роли."""
    return kit[KIT_KEY[lane]] if lane in KIT_KEY else lane


def unit_db(lane: str, option: str) -> Tuple[float, float]:
    """``(dB при гейте 1.0, показатель amp)`` варианта слоя.

    Замер — :data:`LANE_DB_AT_UNIT`; тембр, выбранный моделью поверх пула
    (issue #3268, ``core.club_timbre.TIMBRE_EXTRAS``), — оценка по таблице
    классик-громкости (:func:`core.club_timbre.estimated_unit_db`).
    """
    measured = LANE_DB_AT_UNIT[lane]
    if option in measured:
        return measured[option], _exponent(option)
    from .club_timbre import estimated_unit_db  # classic_loudness импортирует этот модуль

    return estimated_unit_db(lane, option)


def lane_db(kit: Mapping[str, str], lane: str, level: float) -> float:
    """dB RMS слоя, звучащего весь блок на уровне ``level`` (модель)."""
    if level <= 0:
        return _SILENCE_DB
    db, exponent = unit_db(lane, lane_option(kit, lane))
    return db + 20.0 * exponent * math.log10(level)


def _db_to_power(db: float) -> float:
    return 0.0 if db <= _SILENCE_DB else 10.0 ** (db / 10.0)


def _power_to_db(power: float) -> float:
    return _SILENCE_DB if power <= 0 else 10.0 * math.log10(power)


def _on_fraction(mask: int) -> float:
    return sum(cell_quarters(mask)) / QUARTERS


def block_power(matrix: ArrangementMatrix, kit: Mapping[str, str], levels: Mapping[str, Sequence[float]],
                block: int) -> float:
    """Мощность блока: сумма слоёв (доля звучащих четвертей × мощность слоя)."""
    total = 0.0
    for lane, masks in matrix.lanes.items():
        if lane in LANE_DB_AT_UNIT and masks[block]:
            total += _on_fraction(masks[block]) * _db_to_power(lane_db(kit, lane, levels[lane][block]))
    return total


def block_db(matrix: ArrangementMatrix, kit: Mapping[str, str],
             levels: Mapping[str, Sequence[float]]) -> List[float]:
    """dB RMS каждого блока формы (модель)."""
    return [round(_power_to_db(block_power(matrix, kit, levels, b)), 1) for b in range(matrix.n_blocks)]


def main_blocks(matrix: ArrangementMatrix) -> List[int]:
    """Блоки основного уровня: бочка звучит весь блок."""
    return [i for i, mask in enumerate(matrix.lanes["kick"]) if mask == FULL]


def main_db(matrix: ArrangementMatrix, kit: Mapping[str, str], levels: Mapping[str, Sequence[float]]) -> float:
    """dB основного блока: средняя мощность блоков с полной бочкой."""
    blocks = main_blocks(matrix)
    power = sum(block_power(matrix, kit, levels, b) for b in blocks) / len(blocks)
    return round(_power_to_db(power), 2)


def flat_levels(matrix: ArrangementMatrix, base: Mapping[str, float]) -> Dict[str, List[float]]:
    """Уровни «как до калибровки»: один уровень слоя на все блоки."""
    return {lane: [float(base[lane])] * matrix.n_blocks for lane in matrix.lanes}


def _bisect(fn, lo: float, hi: float) -> float:
    """Наибольший x в [lo, hi], где монотонно растущая ``fn(x) <= 0``."""
    if fn(hi) <= 0:
        return hi
    for _ in range(_BISECT_STEPS):
        mid = math.sqrt(lo * hi)
        if fn(mid) <= 0:
            lo = mid
        else:
            hi = mid
    return lo


def _scaled(base: Mapping[str, float], gain: float, cap: float) -> Dict[str, float]:
    return {lane: min(cap, level * gain) for lane, level in base.items()}


def kit_gain(matrix: ArrangementMatrix, kit: Mapping[str, str], base: Mapping[str, float], cap: float) -> float:
    """Множитель каркаса: основной блок = :data:`TARGET_MAIN_DB` (с потолком ``cap``)."""
    def over(gain: float) -> float:
        return main_db(matrix, kit, flat_levels(matrix, _scaled(base, gain, cap))) - TARGET_MAIN_DB

    return _bisect(over, 1e-3, cap / min(base.values()))


def _lift_block(matrix: ArrangementMatrix, kit: Mapping[str, str], levels: Dict[str, List[float]],
                block: int, floor_db: float, cap: float) -> None:
    """Поднять :data:`LIFT_LANES` блока до ``floor_db`` (не выше ``cap``), на месте."""
    lanes = [lane for lane in LIFT_LANES if lane in matrix.lanes and matrix.lanes[lane][block]]
    if not lanes:
        return
    start = {lane: levels[lane][block] for lane in lanes}

    def under(lift: float) -> float:
        for lane in lanes:
            levels[lane][block] = min(cap, start[lane] * lift)
        return _power_to_db(block_power(matrix, kit, levels, block)) - floor_db

    top = cap / min(start.values())
    lift = _bisect(under, 1.0, max(1.0, top))
    under(lift)


def calibrate_levels(matrix: ArrangementMatrix, kit: Mapping[str, str], base: Mapping[str, float],
                     cap: float) -> Dict[str, List[float]]:
    """Уровни гейта по блокам: множитель каркаса + подъём тихих секций.

    Args:
        matrix: матрица шаблона (только звучащие слои).
        kit: каркас ``club_kit`` (шаблон, бочка, хэты, синты ролей).
        base: исходные уровни слоёв (``LAYER_LEVELS``).
        cap: потолок уровня одного слоя (``MAX_LAYER_AMP``).
    """
    gain = kit_gain(matrix, kit, base, cap)
    levels = flat_levels(matrix, _scaled(base, gain, cap))
    floor_db = main_db(matrix, kit, levels) - SECTION_FLOOR_DB
    main = set(main_blocks(matrix))
    for block in range(matrix.n_blocks):
        if block in main:
            continue
        if _power_to_db(block_power(matrix, kit, levels, block)) < floor_db:
            _lift_block(matrix, kit, levels, block, floor_db, cap)
    return levels


def kit_report(matrix: ArrangementMatrix, kit: Mapping[str, str], base: Mapping[str, float],
               cap: float) -> Dict[str, object]:
    """До/после по модели: основной блок, блоки, худшая секция относительно основного."""
    before = flat_levels(matrix, base)
    after = calibrate_levels(matrix, kit, base, cap)
    report: Dict[str, object] = {}
    for name, levels in (("before", before), ("after", after)):
        blocks = block_db(matrix, kit, levels)
        main = main_db(matrix, kit, levels)
        audible = [db for db in blocks if db > _SILENCE_DB]
        report[name] = {"main_db": round(main, 1), "blocks_db": blocks,
                        "worst_drop_db": round(main - min(audible), 1)}
    return report


__all__ = [
    "AMP_EXPONENT",
    "LANE_DB_AT_UNIT",
    "LIFT_LANES",
    "MODEL_SOURCE",
    "SECTION_FLOOR_DB",
    "STYLE_MAIN_DB",
    "TARGET_MAIN_DB",
    "block_db",
    "calibrate_levels",
    "kit_gain",
    "kit_report",
    "lane_db",
    "main_blocks",
    "main_db",
    "unit_db",
]
