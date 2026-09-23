"""Мелодия → все партии аранжировки (бас, пэд, ударные, контрмелодия).

Зачем модуль существует
-----------------------
До него RTTTL-путь давал аранжировщику ТОЛЬКО тему (``lead_midi`` +
``lead_dur``), а бас, пэд, прогрессию и рисунок ударных придумывала LLM —
не видя ни одной ноты мелодии. Получалась не аранжировка, а «тема и рядом
несколько слоёв»: в живом прогоне 14.09 имперский марш в ре-миноре шёл
под басом на ми, потому что модель написала ``progression='1,5,6,4'``
наугад. Никакой обратной связи от мелодии к аккомпанементу не было
вообще.

Здесь она появляется: гармония ВЫВОДИТСЯ из нот темы по тактам, а бас,
пэд и контрмелодия строятся из полученных аккордов. Промахнуться мимо
тональности такой аккомпанемент не может — он буквально сделан из тех же
нот.

Что остаётся за LLM
-------------------
Тембр и характер: какими синтами играть, каким сэмплом бить, какая форма
и темп. Ноты аккомпанемента — нет: это арифметика от темы, а не вкус.

Абсолютный MIDI везде
---------------------
Все выведенные партии — абсолютные MIDI-ноты, а не ступени лада. Ступени
зависят от ``Root.default``/``Scale.default``, которые в аранжировке
меняются по форме; тема при этом фиксирована, и любой такой сдвиг
развалил бы согласие баса с мелодией. Абсолютные ноты развалить нельзя.
Как абсолютный MIDI попадает в Renardo — см. ``core.rtttl``.

Ручки вместо скрытых эвристик (ADR-0132, PR-3)
----------------------------------------------
Каждое музыкальное авто-решение раскладки — отдельное поле
:class:`HarmonizeOptions` (тональность, аккорды, гармонический ритм,
плотность, бас, подходы, пэд, регистр пэда, ударные, октава и выбросы
темы). Значение по умолчанию каждой ручки — ``auto``, то есть ровно
сегодняшнее поведение: golden-снимок ``test_arranger_golden`` обязан
совпадать байт-в-байт. Не-``auto`` значение — осознанный выбор вызывающего
(модели, с PR-4), а не новая эвристика: код исполняет его буквально.
Итог каждой ручки пишется в ``Harmonization.decisions`` и оттуда — в
строку «Решения по умолчанию» партитуры (:mod:`core.score_sheet`). Ручки,
которые решает сборка слоёв (второй голос, октавы темы, громкости), —
в ``arranger.ArrangeOptions``. Проверка значений — в ``__post_init__``:
неизвестное значение → ``ValueError`` со списком, а не тихая замена.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from statistics import median
from typing import Dict, List, Optional, Sequence, Tuple, Union

from .arranger import (
    BEATS_PER_BAR,
    PAD_STAB_SUS,
    SCALE_INTERVALS,
    VALID_ROOTS,
    check_root,
)

__all__ = [
    "ChordWindow",
    "Harmonization",
    "HarmonizeOptions",
    "harmonize",
    "parse_chord",
]

#: Порог плотности темы: атак на один бит.
#:
#: 🔴 FIX (live 14.09, «Pink Panther не узнать»): плотность считалась по
#: МЕДИАННОЙ ДЛИНЕ ноты, а это не то же самое. Тема Пантеры — пары
#: восьмых, разделённые паузами по два бита: ноты короткие, но времени
#: она занимает мало, и по медиане считалась «плотной». В ответ
#: аранжировка закатывала её паузы остинато на каждую долю, басом
#: четвертями, удвоением в октаву и вторым голосом — а именно в этих
#: паузах весь её характер. Тему было не узнать не потому, что ноты
#: неверные, а потому что её засыпало.
#:
#: Атаки на бит меряют ровно то, что нужно: насколько густо тема
#: заполняет время. Порог выбран по контрольной выборке — марши и
#: чиптюн дают 1.4-1.8, крадущаяся тема Пантеры 0.95.
DENSE_ONSETS_PER_BEAT = 1.2

#: Сколько 16-х в такте — сетка, на которую квантуются атаки мелодии при
#: выводе рисунка ударных. 16-я — самая мелкая длительность, которая
#: реально встречается в RTTTL-рингтонах как ритмическая (32-е там почти
#: всегда «дыхательные» паузы, а не ноты).
STEPS_PER_BAR = 16

#: Нижняя граница баса в MIDI-нотах. Ниже до-большой октавы бас на
#: динамике робота превращается в гул без высоты.
BASS_MIDI_FLOOR = 36   # C2

#: Зазор между верхней нотой подклада и самой низкой нотой темы, полутоны.
#: Целый тон — минимум, при котором подклад перестаёт сливаться с темой в
#: унисон и ловится слухом как отдельный слой.
PAD_CLEARANCE = 2

#: Куда опускать потолок подклада, если тема сама лежит низко. Без этого
#: предела низкая тема (басовый рифф, мужской вокал) вдавила бы подклад в
#: бас, и оба слоя слиплись бы в кашу.
PAD_MIDI_FLOOR = 48    # C3

#: Зазор между самым низким тоном подклада и самой ВЫСОКОЙ нотой баса,
#: полутоны — в звучащей высоте (см. модуль-докстринг: гармонизация вся
#: считает в звучащих нотах, поправку на транспонирование синта вносит
#: аранжировщик последним шагом).
#:
#: 🔴 FIX (issue #2876, живой прогон 23.09.2026, «диджей Снупдог» —
#: Still Dre): подклад держался ТОЛЬКО потолком (:func:`_pad_ceiling`,
#: от самой низкой ноты темы) и понятия не имел, где лежит бас. У Still
#: Dre бас (``moogbass``, ``SYNTH_SEMITONE_SHIFT['moogbass']=24`` —
#: звучит на октаву ниже написанного) уходил в F2-C3, а подклад
#: (``strings``, без сдвига) складывался вниз от заниженного потолка и
#: садился ровно туда же — F2-A#3. Бас и подклад слились в одну кашу.
#: Теперь итоговый потолок подклада (:func:`_pick_chords`) поднимается
#: минимум до фактического потолка баса ПЛЮС этот зазор, а
#: :func:`_stack_chord` не даёт ни одному тону аккорда провалиться ниже
#: этой границы, даже если стек «через октаву» на неудачных классах
#: высоты того бы захотел.
PAD_BASS_CLEARANCE = 3

#: Насколько корень аккорда весомее остальных его тонов при выборе
#: гармонии. Без перевеса трезвучия с общими нотами (например i и VI в
#: миноре — две ноты из трёх общие) выбираются монеткой, и гармония
#: дёргается между ними каждый такт.
_ROOT_WEIGHT = 1.6

#: Бонус аккорду, чья тоника совпадает с ПЕРВОЙ прозвучавшей нотой такта.
#: Сильная доля слышится как опора гармонии сильнее, чем любая нота
#: в середине такта, и вес по длительности этого не передаёт.
_DOWNBEAT_BONUS = 0.9

#: Штраф за смену аккорда относительно предыдущего такта. Гармония,
#: меняющаяся каждый такт, звучит суетливо даже когда каждый отдельный
#: выбор формально верен; инерция склеивает соседние такты в фразу.
_CHANGE_PENALTY = 0.9

#: Штраф аккорду, которого нет в определённом ладу темы.
#:
#: Заимствованные аккорды нужны (см. :func:`_chord_candidates`), но по
#: умолчанию гармония обязана оставаться в тональности: без штрафа любой
#: хроматический проход тянул бы за собой смену аккорда, и тональный
#: центр рассыпался бы. Недиатонический аккорд выигрывает только когда
#: объясняет ноты окна заметно лучше любого диатонического.
_CHROMATIC_PENALTY = 1.6

#: Бонус тонике в ПЕРВОМ и ПОСЛЕДНЕМ такте темы. Луп смыкается сам с
#: собой, и если стык приходится не на тонику, каждый повтор слышится
#: как оборванная фраза.
_CADENCE_BONUS = 1.2


# ---------------------------------------------------------------------------
# Ручки раскладки (ADR-0132 PR-3)
# ---------------------------------------------------------------------------

#: Значение любой ручки «как сегодня» — решает автоматика.
AUTO = "auto"

#: Ручка -> допустимые строковые значения. Первое — значение по умолчанию.
KNOB_VALUES: Dict[str, Tuple[str, ...]] = {
    # auto — корреляция Крумхансл + опора на тоническое трезвучие (#2873);
    # profile — чистый Крумхансл (гистограмма высот без позиций).
    "key_detection": (AUTO, "profile"),
    # fix — подтянуть к корпусу ноты дальше октавы от него (#2876); keep —
    # играть тему как записана.
    "lead_outliers": ("fix", "keep"),
    # auto — окно полтакта со склейкой одинаковых (частоту смены задаёт
    # мелодия); bar — не чаще аккорда на такт; half — окно полтакта без
    # инерции (аккорд может меняться каждые полтакта).
    "harmonic_rhythm": (AUTO, "bar", "half"),
    # auto — по атакам на бит (DENSE_ONSETS_PER_BEAT); sparse/dense —
    # принудительно (шаг баса и пэда, второй голос, октавы темы).
    "density": (AUTO, "sparse", "dense"),
    # auto — тоны аккорда по форме (корень-корень-квинта-терция у плотной,
    # корень-квинта у редкой); root — только корни; root_fifth — корень и
    # квинта; pedal — тоника лада на всё окно; off — без баса.
    "bass_style": (AUTO, "root", "root_fifth", "pedal", "off"),
    # auto — подход в последнюю восьмую окна из двух и более нот; on — и в
    # окне из одной ноты; off — без подходов.
    "bass_approach": (AUTO, "on", "off"),
    # auto/stab — удары аккорда по долям с коротким sus; sustain — аккорд
    # держится всё окно; off — без пэда.
    "pad_style": (AUTO, "stab", "sustain", "off"),
}

#: Именованные регистры пэда ``(низ, верх)`` в звучащем MIDI. ``auto`` —
#: потолок от низа корпуса темы и пол над басом (#2876).
PAD_REGISTERS: Dict[str, Tuple[int, int]] = {
    "low": (48, 60),    # C3–C4
    "mid": (55, 67),    # G3–G4
    "high": (60, 72),   # C4–C5
}

#: Явный регистр пэда ``(низ, верх)``: не уже октавы (иначе трезвучие не
#: укладывается без выхода за край) и в пределах слышимого на динамике.
PAD_REGISTER_SPAN_MIN = 12
PAD_REGISTER_LIMITS = (36, 96)

#: Сдвиг темы целыми октавами от записанного регистра.
LEAD_OCTAVE_RANGE = (-2, 2)
LEAD_OCTAVE_WORDS = (AUTO, "keep")


def parse_chord(name: str) -> Tuple[int, int, int]:
    """Имя аккорда → классы высоты трезвучия ``(корень, терция, квинта)``.

    ``Am`` — минор, ``F`` — мажор, бемоль можно (``Bbm`` = ``A#m``).

    Raises:
        ValueError: не аккорд — с примером формата.
    """
    text = (name or "").strip()
    minor = len(text) > 1 and text.endswith("m")
    root_text = text[:-1] if minor else text
    try:
        root = check_root(root_text)
    except ValueError:
        root = None
    if root is None:
        raise ValueError(
            f"Неизвестный аккорд {name!r}. Формат: тоника + «m» для минора "
            "(Am, F, C#m, Bb)."
        )
    pc = VALID_ROOTS.index(root)
    third = 3 if minor else 4
    return (pc, (pc + third) % 12, (pc + 7) % 12)


def _check_word(knob: str, value: object) -> None:
    allowed = KNOB_VALUES[knob]
    if value not in allowed:
        raise ValueError(
            f"Неизвестное значение {knob}={value!r}. Доступны: {', '.join(allowed)}."
        )


def _check_lead_octave(value: object) -> None:
    if value in LEAD_OCTAVE_WORDS:
        return
    lo, hi = LEAD_OCTAVE_RANGE
    if isinstance(value, bool) or not isinstance(value, int) or not lo <= value <= hi:
        raise ValueError(
            f"lead_octave={value!r}: допустимо auto, keep или целое {lo:+d}..{hi:+d} "
            "(октавы от записанного регистра темы)."
        )


def _check_pad_register(value: object) -> None:
    if value == AUTO or value in PAD_REGISTERS:
        return
    ok = (
        isinstance(value, tuple) and len(value) == 2
        and all(isinstance(v, int) and not isinstance(v, bool) for v in value)
    )
    lo_limit, hi_limit = PAD_REGISTER_LIMITS
    if ok and lo_limit <= value[0] and value[1] <= hi_limit \
            and value[1] - value[0] >= PAD_REGISTER_SPAN_MIN:
        return
    raise ValueError(
        f"pad_register={value!r}: допустимо auto, {', '.join(PAD_REGISTERS)} или "
        f"(низ, верх) MIDI в {lo_limit}..{hi_limit}, не уже октавы."
    )


@dataclass(frozen=True)
class HarmonizeOptions:
    """Ручки подготовки темы и раскладки на партии (ADR-0132 PR-3).

    Все по умолчанию — ``auto`` (``lead_outliers`` — ``fix``, ``chords``/
    ``drums``/``hats`` — ``None``): сегодняшнее поведение байт-в-байт.
    Допустимые значения — :data:`KNOB_VALUES`, :data:`PAD_REGISTERS`,
    :data:`LEAD_OCTAVE_RANGE`.

    Attributes:
        key_detection: ``auto`` | ``profile`` — чем определять тональность
            (``rtttl_compose.detect_key_ranked``).
        lead_octave: ``auto`` (к рабочему регистру) | ``keep`` | целое
            -2..+2 — октавы от записанного регистра темы.
        lead_outliers: ``fix`` | ``keep``.
        chords: имена аккордов по тактам (``("Am", "F", "C", "G")``), по
            кругу; ``None`` — выводить из мелодии.
        harmonic_rhythm, density, bass_style, bass_approach, pad_style: см.
            :data:`KNOB_VALUES`.
        pad_register: ``auto`` | ``low`` | ``mid`` | ``high`` | ``(низ, верх)``.
        drums, hats: рисунок вместо выведенного (16 шагов на такт, как у
            ``_build_drums``); ``None`` — выводить.
    """

    key_detection: str = AUTO
    lead_octave: Union[str, int] = AUTO
    lead_outliers: str = "fix"
    chords: Optional[Tuple[str, ...]] = None
    harmonic_rhythm: str = AUTO
    density: str = AUTO
    bass_style: str = AUTO
    bass_approach: str = AUTO
    pad_style: str = AUTO
    pad_register: Union[str, Tuple[int, int]] = AUTO
    drums: Optional[str] = None
    hats: Optional[str] = None

    def __post_init__(self) -> None:
        for knob in KNOB_VALUES:
            _check_word(knob, getattr(self, knob))
        _check_lead_octave(self.lead_octave)
        if isinstance(self.pad_register, list):
            object.__setattr__(self, "pad_register", tuple(self.pad_register))
        _check_pad_register(self.pad_register)
        if self.chords is not None:
            object.__setattr__(self, "chords", tuple(self.chords))
            if not self.chords:
                raise ValueError("chords: пустой список — задай хотя бы один аккорд или None.")
            for name in self.chords:
                parse_chord(name)
        for knob in ("drums", "hats"):
            if getattr(self, knob) is not None and not isinstance(getattr(self, knob), str):
                raise ValueError(f"{knob}: рисунок должен быть строкой, получено {getattr(self, knob)!r}.")


@dataclass(frozen=True)
class ChordWindow:
    """Аккорд одного окна гармонизации (такта темы).

    Attributes:
        start: начало окна в битах от начала темы.
        beats: длина окна в битах.
        degree: ступень лада, на которой построено трезвучие (0-based).
        root_midi: абсолютный MIDI корня в басовом регистре.
        tones: абсолютные MIDI тонов трезвучия в регистре пэда.
        pitch_classes: классы высоты трезвучия (0..11) — для контрмелодии.
    """

    start: float
    beats: float
    degree: int
    root_midi: int
    tones: Tuple[int, ...]
    pitch_classes: Tuple[int, ...]


@dataclass(frozen=True)
class Harmonization:
    """Тема, разложенная на партии. Всё — абсолютные MIDI и биты.

    Партии отдаются парами ``(ноты, длительности)`` одинаковой длины, как
    их принимает аранжировщик: ``None`` в нотах — пауза, в аккорде пэда
    кортеж — одновременно звучащие ноты.
    """

    bpm: int
    root: str
    scale: str
    bars: int
    #: Атак темы на один бит. Мера того, насколько густо тема заполняет
    #: время; ею определяется, сколько аранжировки тема выдержит.
    density: float
    #: ``density >= DENSE_ONSETS_PER_BEAT``. Плотной теме положен полный
    #: наряд (остинато по долям, удвоение в октаву, второй голос), редкой
    #: — только скелет.
    dense: bool
    chords: Tuple[ChordWindow, ...]
    lead: Tuple[Tuple[Optional[int], float], ...]
    bass: Tuple[Tuple[Optional[int], float], ...]
    pad: Tuple[Tuple[Optional[Tuple[int, ...]], float], ...]
    counter: Tuple[Tuple[Optional[int], float], ...]
    drums: str
    hats: str
    #: Длина звучания аккорда пэда в битах (ADR-0132 PR-3, ``pad_style``):
    #: короткий удар остинато (:data:`arranger.PAD_STAB_SUS`) или ``None`` —
    #: аккорд держится всю свою длительность (``sustain``). Аранжировщик
    #: ставит его слою пэда как ``sus``.
    pad_sus: Optional[float] = PAD_STAB_SUS
    #: ADR-0132: что автоматика решила при раскладке (шаг баса и пэда,
    #: число подходов баса, потолок пэда от темы, стиль ударных). Только
    #: запись для партитуры: в сравнении объектов не участвует и на ноты
    #: не влияет.
    decisions: Dict[str, object] = field(
        default_factory=dict, compare=False, hash=False
    )


# ---------------------------------------------------------------------------
# Разбор темы во времени
# ---------------------------------------------------------------------------


def _timed(
    notes: Sequence[Tuple[Optional[int], float]],
) -> Tuple[List[Tuple[float, Optional[int], float]], float]:
    """``[(midi, дл.)]`` → ``[(начало, midi, дл.)]`` + общая длина в битах."""
    out: List[Tuple[float, Optional[int], float]] = []
    cursor = 0.0
    for midi, dur in notes:
        out.append((cursor, midi, float(dur)))
        cursor += float(dur)
    return out, cursor


def _pitch_weights(
    timed: Sequence[Tuple[float, Optional[int], float]],
    start: float,
    end: float,
) -> Dict[int, float]:
    """Вес каждого класса высоты в окне ``[start, end)``.

    Вес — сколько битов нота реально звучит ВНУТРИ окна. Нота, начавшаяся
    раньше и тянущаяся в окно, считается по пересечению: именно она
    держит гармонию такта, хотя её атака осталась в прошлом такте.
    """
    weights: Dict[int, float] = {}
    for onset, midi, dur in timed:
        if midi is None:
            continue
        overlap = min(onset + dur, end) - max(onset, start)
        if overlap > 0:
            pc = midi % 12
            weights[pc] = weights.get(pc, 0.0) + overlap
    return weights


def _first_note_in(
    timed: Sequence[Tuple[float, Optional[int], float]],
    start: float,
    end: float,
) -> Optional[int]:
    """Первая нота, АТАКА которой попадает в окно (``None`` — таких нет)."""
    for onset, midi, _dur in timed:
        if midi is not None and start <= onset < end:
            return midi
    return None


def _weighted_percentile(pairs: Sequence[Tuple[int, float]], pct: float) -> float:
    """Перцентиль ``pct`` (0..1) значений, взвешенных длительностью.

    🔴 FIX (issue #2876, «Still Dre»): и потолок подклада
    (:func:`_pad_ceiling`), и нормализация регистра лида
    (``rtttl_compose._normalize_lead_register``) раньше смотрели на
    САМУЮ НИЗКУЮ/высокую ноту темы. Одной короткой ноты — затакта,
    предикта — хватало, чтобы утащить границу за собой: у Still Dre
    четыре затакта по 0.25 доли на MIDI 72 (после нормализации — 60)
    стоят рядом с телом темы на 75-77 длинными нотами по целой доле, но
    именно они, а не корпус темы, определяли потолок подклада.

    Взвешенный перцентиль игнорирует такие выбросы САМ ПО СЕБЕ, без
    отдельной проверки «короткая ли нота»: вес затакта тонет в весе
    корпуса темы, и он на результат почти не влияет. Реализация —
    ближайший ранг (nearest-rank) по накопленному весу, сортировка по
    значению.
    """
    items = sorted(pairs, key=lambda pair: pair[0])
    total = sum(weight for _value, weight in items)
    if total <= 0:
        return float(items[0][0])
    target = pct * total
    cumulative = 0.0
    for value, weight in items:
        cumulative += weight
        if cumulative >= target:
            return float(value)
    return float(items[-1][0])


# ---------------------------------------------------------------------------
# Гармония
# ---------------------------------------------------------------------------


def _triad_pitch_classes(
    degree: int, root_semitone: int, intervals: Sequence[int]
) -> Tuple[int, ...]:
    """Трезвучие на ступени лада → классы высоты (0..11).

    Терции берутся ПО ЛАДУ (ступени d, d+2, d+4), а не по полутонам —
    поэтому на каждой ступени получается свой натуральный вид аккорда
    (в миноре: i минорный, III мажорный, V минорный и так далее), и
    ни одна нота трезвучия не выпадает из лада мелодии.
    """
    size = len(intervals)
    out: List[int] = []
    for step in (0, 2, 4):
        index = degree + step
        octave, wrapped = divmod(index, size)
        out.append((root_semitone + intervals[wrapped] + 12 * octave) % 12)
    return tuple(out)


def _chord_candidates(
    root_semitone: int, intervals: Sequence[int]
) -> List[Tuple[Tuple[int, ...], bool]]:
    """Все аккорды-кандидаты: ``(классы высоты, диатонический ли)``.

    Кандидаты строятся на ВСЕХ ДВЕНАДЦАТИ ступенях хроматики, мажорные и
    минорные, а не только на семи ступенях лада.

    🔴 FIX (live 14.09, «Марио — восемь одинаковых аккордов»): диатоникой
    одного лада часто нечем описать такт. Заимствованные аккорды (♭VI,
    ♭VII), побочные доминанты и короткие отклонения — обычное дело в
    мелодиях этой библиотеки, и без них такт сваливается на тонику по
    умолчанию. Плюс это страховка от неточной тональности: даже когда
    ``detect_key`` ошибся ладом, нужный аккорд остаётся достижим.

    Строятся только мажорные и минорные трезвучия: увеличенные и
    уменьшённые, которые раньше возникали сами собой из терций по ладу,
    как выдержанный аккорд слышатся фальшью, а не краской.

    Недиатонические помечаются, а не запрещаются: вызывающий добавляет им
    штраф, чтобы они выигрывали только там, где действительно объясняют
    ноты окна лучше диатонических.

    🔴 FIX (live 14.09, «марш странно играет»): «диатонический» проверялся
    как ПРИНАДЛЕЖНОСТЬ НОТ ЛАДУ — а это не то же самое, что «аккорд этого
    лада». В ре гармоническом миноре трезвучие на си-бемоле мажорное
    (A#-D-F), но си-бемоль МИНОР (A#-C#-F) тоже проходил проверку: его
    до-диез — это повышенная седьмая ступень, она в ладу есть. И он шёл
    без штрафа наравне с правильным, выигрывая в каждом такте, где звучит
    до-диез, — хотя его ре-бемоль бьётся с ре, тоникой всей пьесы.
    Теперь диатоническими считаются ровно те трезвучия, что строятся
    терциями ПО СТУПЕНЯМ лада.
    """
    diatonic_sets = {
        frozenset(_triad_pitch_classes(degree, root_semitone, intervals))
        for degree in range(len(intervals))
    }
    out: List[Tuple[Tuple[int, ...], bool]] = []
    for pc_root in range(12):
        for third in (4, 3):  # мажорное и минорное трезвучие
            pcs = (pc_root, (pc_root + third) % 12, (pc_root + 7) % 12)
            out.append((pcs, frozenset(pcs) in diatonic_sets))
    return out


def _best_chord(
    candidates: Sequence[Tuple[Tuple[int, ...], bool]],
    weights: Dict[int, float],
    scale_factor: float,
    downbeat: Optional[int],
    is_edge: bool,
    tonic_pcs: Tuple[int, ...],
    previous: Optional[Tuple[int, ...]],
    change_penalty: float = _CHANGE_PENALTY,
) -> Tuple[int, ...]:
    """Выбрать аккорд окна с лучшим взвешенным скором.

    Скор: сумма весов нот, попавших в аккорд; бонус за корень — ТОЛЬКО
    диатоническим (разводит аккорды лада с общими нотами; хроматическому
    он давал выиграть по совпадению баса), штраф недиатоническому, бонус
    за совпадение с первой нотой окна, бонус тонике на стыке лупа, штраф
    за смену аккорда (``change_penalty``; 0 у ``harmonic_rhythm=half``).
    """
    best: Tuple[int, ...] = tonic_pcs
    best_score = float("-inf")
    for pcs, diatonic in candidates:
        score = sum(weights.get(pc, 0.0) for pc in pcs)
        if diatonic:
            score += (_ROOT_WEIGHT - 1.0) * weights.get(pcs[0], 0.0)
        if not diatonic:
            score -= _CHROMATIC_PENALTY * scale_factor
        if downbeat is not None and downbeat % 12 == pcs[0]:
            score += _DOWNBEAT_BONUS * scale_factor
        if is_edge and pcs == tonic_pcs:
            score += _CADENCE_BONUS * scale_factor
        if previous is not None and pcs != previous:
            score -= change_penalty * scale_factor
        if score > best_score:
            best_score = score
            best = pcs
    return best


def _pick_chords(
    timed: Sequence[Tuple[float, Optional[int], float]],
    total_beats: float,
    root: str,
    scale: str,
    options: Optional["HarmonizeOptions"] = None,
) -> Tuple[ChordWindow, ...]:
    """Выбрать гармонию: аккорд на каждые полтакта, соседние одинаковые слить.

    Окно — ПОЛТАКТА, а не такт. Гармония живых мелодий меняется и внутри
    такта, а окном в целый такт такая смена невидима: обе половины
    усредняются, побеждает тоника, и тема получает один аккорд на всю
    длину. Слияние соседних одинаковых окон возвращает целый такт там, где
    половины согласны, — то есть частота смены гармонии определяется самой
    мелодией, а не сеткой.

    Скор окна: длительности нот, попавших в аккорд (корень весомее), бонус
    за совпадение с первой нотой окна, бонус тонике на стыке лупа, штрафы
    недиатоническому аккорду и смене аккорда.

    ADR-0132 PR-3: ``options.chords`` — аккорды по тактам вместо выведенных
    (:func:`_explicit_windows`); ``options.harmonic_rhythm`` — окно и
    инерция (:func:`_auto_windows`); ``options.pad_register`` — регистр
    тонов пэда (:func:`_pad_bounds`). По умолчанию — всё как раньше.
    """
    options = options or HarmonizeOptions()
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    root_semitone = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    if options.chords:
        window = float(BEATS_PER_BAR)
        picked = _explicit_windows(options.chords, total_beats)
    else:
        window = BEATS_PER_BAR if options.harmonic_rhythm == "bar" else BEATS_PER_BAR / 2.0
        picked = _auto_windows(
            timed, total_beats, root_semitone, intervals, window,
            0.0 if options.harmonic_rhythm == "half" else _CHANGE_PENALTY,
        )
    chords = _merge_windows(picked, window, root_semitone, intervals)
    ceiling, floor = _pad_bounds(timed, chords, options.pad_register)
    return tuple(
        replace(chord, tones=_stack_chord(chord.pitch_classes, ceiling, floor))
        for chord in chords
    )


def _auto_windows(
    timed: Sequence[Tuple[float, Optional[int], float]],
    total_beats: float,
    root_semitone: int,
    intervals: Sequence[int],
    window: float,
    change_penalty: float,
) -> List[Tuple[float, Tuple[int, ...]]]:
    """Аккорд каждого окна длиной ``window``, выведенный из нот темы."""
    candidates = _chord_candidates(root_semitone, intervals)
    tonic_pcs = _triad_pitch_classes(0, root_semitone, intervals)
    count = max(1, int(round(total_beats / window)))

    picked: List[Tuple[float, Tuple[int, ...]]] = []
    previous: Optional[Tuple[int, ...]] = None
    for index in range(count):
        begin = index * window
        weights = _pitch_weights(timed, begin, begin + window)
        downbeat = _first_note_in(timed, begin, begin + window)
        is_edge = index == 0 or index == count - 1
        # Инерция и бонусы соразмерны весу окна: в почти пустом окне
        # (хвост темы, выдержанная нота) абсолютная добавка перевешивала
        # сами ноты и намертво тянула прошлый аккорд.
        scale_factor = min(1.0, sum(weights.values()) / window)

        best = _best_chord(
            candidates, weights, scale_factor, downbeat, is_edge, tonic_pcs,
            previous, change_penalty,
        )
        picked.append((begin, best))
        previous = best
    return picked


def _explicit_windows(
    names: Sequence[str], total_beats: float
) -> List[Tuple[float, Tuple[int, ...]]]:
    """Аккорды по тактам от вызывающего (ADR-0132 PR-3), по кругу.

    Raises:
        ValueError: аккордов больше, чем тактов в теме, — лишние не
            прозвучали бы ни разу, а молча выбрасывать их нельзя.
    """
    bars = max(1, int(round(total_beats / BEATS_PER_BAR)))
    if len(names) > bars:
        raise ValueError(
            f"chords: {len(names)} аккордов на тему из {bars} тактов — "
            "по одному аккорду на такт, лишние не прозвучат."
        )
    triads = [parse_chord(name) for name in names]
    return [
        (float(bar * BEATS_PER_BAR), triads[bar % len(triads)])
        for bar in range(bars)
    ]


def _merge_windows(
    picked: Sequence[Tuple[float, Tuple[int, ...]]],
    window: float,
    root_semitone: int,
    intervals: Sequence[int],
) -> List[ChordWindow]:
    """Соседние окна с одним аккордом становятся одним окном.

    Тона подклада (``tones``) достраиваются ВТОРЫМ проходом
    (:func:`_pad_bounds`) — им нужен фактический потолок баса, а он
    известен только когда вся гармония (root_midi каждого окна) выбрана.
    """
    chords: List[ChordWindow] = []
    for begin, pcs in picked:
        if chords and chords[-1].pitch_classes == pcs:
            last = chords[-1]
            chords[-1] = replace(last, beats=last.beats + window)
            continue
        chords.append(
            ChordWindow(
                start=float(begin),
                beats=float(window),
                degree=_scale_degree(pcs[0], root_semitone, intervals),
                root_midi=_lift(pcs[0], BASS_MIDI_FLOOR),
                tones=(),
                pitch_classes=pcs,
            )
        )
    return chords


def _pad_bounds(
    timed: Sequence[Tuple[float, Optional[int], float]],
    chords: Sequence[ChordWindow],
    register: Union[str, Tuple[int, int]] = AUTO,
) -> Tuple[int, int]:
    """``(потолок, пол)`` укладки пэда (:func:`_stack_chord`).

    Явный регистр (ADR-0132 PR-3, ``pad_register``) берётся как есть:
    ``(низ, верх)`` из :data:`PAD_REGISTERS` или заданный числами.

    ``auto`` — 🔴 FIX (issue #2876, «Still Dre»): потолок подклада раньше
    знал только о теме (:func:`_pad_ceiling`) — не о басе, который тоже
    строится из этой же гармонии (:func:`_build_bass`). У Still Dre
    ceiling от заниженной темы (58) сел прямо на потолок баса (48): оба
    слоя звучали в одной полосе. Считаем фактический потолок баса ПО ТЕМ
    ЖЕ аккордам (``root_midi + смещение тона от корня`` — та же
    арифметика, что в :func:`_build_bass`) и поднимаем потолок подклада
    над ним минимум на :data:`PAD_BASS_CLEARANCE`.
    """
    if register != AUTO:
        low, high = PAD_REGISTERS.get(register, register)  # type: ignore[arg-type]
        return int(high), int(low)
    bass_ceiling = max(
        (
            chord.root_midi + max((pc - chord.pitch_classes[0]) % 12 for pc in chord.pitch_classes)
            for chord in chords
        ),
        default=BASS_MIDI_FLOOR,
    )
    stack_floor = max(PAD_MIDI_FLOOR, bass_ceiling + PAD_BASS_CLEARANCE)
    return max(_pad_ceiling(timed), stack_floor), stack_floor


#: Нижний перцентиль темы (по весу длительности), от которого отсчитывается
#: потолок подклада. 10-й, а не 0-й (минимум): минимум ловит любой
#: одиночный затакт, 10-й перцентиль требует, чтобы «низа» набралось
#: заметно (issue #2876, см. :func:`_weighted_percentile`).
_PAD_CEILING_PERCENTILE = 0.10


def _pad_ceiling(
    timed: Sequence[Tuple[float, Optional[int], float]],
) -> int:
    """Потолок подклада: на :data:`PAD_CLEARANCE` ниже НИЗА корпуса темы.

    «Низ корпуса» — 10-й перцентиль высоты нот, взвешенный длительностью
    (:func:`_weighted_percentile`), а не голый минимум.

    🔴 FIX (issue #2876, «Still Dre»): по голому минимуму хватало ОДНОЙ
    короткой ноты темы (затакт, предикт), чтобы утащить потолок подклада
    вниз вместе с собой — даже когда всё тело темы стоит заметно выше.
    Перцентиль, взвешенный длительностью, такую ноту в расчёт почти не
    берёт: её вес тонет в весе куда более длинных нот корпуса.
    """
    pairs = [(midi, dur) for _onset, midi, dur in timed if midi is not None]
    if not pairs:
        return PAD_MIDI_FLOOR
    low = _weighted_percentile(pairs, _PAD_CEILING_PERCENTILE)
    return max(PAD_MIDI_FLOOR, int(round(low)) - PAD_CLEARANCE)


def _scale_degree(
    pitch_class: int, root_semitone: int, intervals: Sequence[int]
) -> int:
    """Ступень лада для корня аккорда; ``-1`` для недиатонического."""
    offset = (pitch_class - root_semitone) % 12
    for degree, semitones in enumerate(intervals):
        if semitones % 12 == offset:
            return degree
    return -1


def _lift(pitch_class: int, floor_midi: int) -> int:
    """Класс высоты → ближайшая MIDI-нота не ниже ``floor_midi``."""
    note = floor_midi + ((pitch_class - floor_midi) % 12)
    return note


def _stack_chord(
    pitch_classes: Sequence[int], ceiling: int, floor: Optional[int] = None
) -> Tuple[int, ...]:
    """Классы высоты → аккорд, уложенный ВНИЗ от ``ceiling``.

    🔴 FIX (live 14.09, «будто ноты пропускает»): аккорд складывался ВВЕРХ
    от постоянного пола (E3), не глядя, где лежит тема. У имперского марша
    это ставило подклад в D4-D5 — прямо в тему: 23 из 66 её нот подклад
    играл В УНИСОН и долбил их аккордом на каждую долю. Атаки мелодии
    тонули в этом пульсе, и на слух казалось, что тема пропускает ноты.

    В оркестровке аккомпанемент стоит ПОД мелодией — это не стиль, а
    условие того, чтобы мелодию было слышно. Поэтому укладываем вниз от
    потолка, а потолок задаёт вызывающий по самой низкой ноте темы.

    Кладём именно последовательно вниз (каждая следующая нота ниже
    предыдущей), а не берём три ближайшие по отдельности: иначе аккорд
    вывернется в случайное обращение и подклад будет прыгать регистром от
    такта к такту.

    Ровно три ноты, без удвоений. Удвоение корня октавой ниже тут было —
    его добавляли ради веса, когда подклад стоял в регистре темы и звучал
    тонко. Теперь вес даёт разделение регистров, а удвоение только роняло
    подклад в бас (у марша — до D2, прямо в басовую партию) и стоило
    лишнего голоса scsynth на каждой доле остинато.

    ``floor`` (issue #2876) — страховка снизу: тон, попавший ниже него,
    поднимается октавами, пока не выйдет из-под пола. Обычная работа
    ``ceiling``, поднятого до :data:`PAD_BASS_CLEARANCE` над потолком
    баса (см. :func:`_pick_chords`), делает это событие редким — но
    складывая ТРИ ноты подряд вниз с шагом до октавы, нижний тон
    трезвучия теоретически может провалиться дальше баса, даже когда сам
    потолок стоит над ним с запасом. Без страховки это был бы тот же
    баг #2876, просто на другой ступени аккорда.
    """
    out: List[int] = []
    current = ceiling
    for pc in reversed(pitch_classes):
        note = current - ((current - pc) % 12)
        if floor is not None:
            while note < floor:
                note += 12
        out.append(note)
        current = note - 1
    return tuple(sorted(out))


# ---------------------------------------------------------------------------
# Партии
# ---------------------------------------------------------------------------


#: Форма басовой линии внутри одного окна гармонии: индексы тонов аккорда
#: (0 — корень, 1 — терция, 2 — квинта) по шагам.
#:
#: Корень на сильных долях, квинта в середине, терция как краска. Это
#: скелет; интереснее его делают подходы к следующему аккорду, см.
#: :func:`_approach_note`.
_BASS_SHAPE_DENSE = (0, 0, 2, 1)
_BASS_SHAPE_SPARSE = (0, 2)

#: Длина ноты-подхода к следующему аккорду, биты.
#:
#: 🔴 FIX (live 23.09, #2839, «гимн — лютый мусор»): подход длился целый
#: шаг баса (бит у плотной темы, два у редкой) и был хроматическим. При
#: смене аккорда каждые полтакта это половина всей басовой партии — и
#: вся она вне лада: у гимна в до мажоре бас играл C F# G G# A G# G C#.
#: Подход — затакт в следующую опору, а не опора сама по себе: восьмая
#: в конце окна, всё остальное окно звучит тон аккорда.
_APPROACH_BEATS = 0.5


def _approach_note(
    held: int, target: int, scale_pcs: frozenset
) -> Optional[int]:
    """Нота-подход к ``target``: соседняя ступень лада снизу или сверху.

    Подход — то, чем осмысленная басовая линия отличается от механической:
    последняя восьмая перед сменой гармонии ведёт в новый корень шагом, и
    линия слышится как ЛИНИЯ, а не как набор опор.

    🔴 FIX (live 23.09, #2839): подход был хроматическим (полутон к корню)
    и потому почти всегда вне лада. Теперь это соседняя СТУПЕНЬ лада
    (полутон или тон от корня). Хроматический полутон остаётся только там,
    где лад не даёт ступени ближе тона — в пентатонике с её терцовыми
    дырами; и тогда это короткая проходящая нота, разрешающаяся в корень
    ровно на полтона.

    Из двух сторон берётся ближайшая к ``held`` (звучащему перед подходом
    тону аккорда), чтобы бас шёл плавно; сторона, совпадающая с ``held``,
    отбрасывается — повтор той же ноты подходом не является. Ниже
    :data:`BASS_MIDI_FLOOR` подход не опускается. Если после этого
    подходить нечем (корень на полу, а ступень сверху уже звучит) —
    ``None``: окно дозвучит тоном аккорда.
    """
    candidates: List[int] = []
    for direction in (-1, 1):
        diatonic = [
            target + direction * distance
            for distance in (1, 2)
            if (target + direction * distance) % 12 in scale_pcs
        ]
        candidates.append(diatonic[0] if diatonic else target + direction)
    usable = [
        note for note in candidates
        if note != held and note >= BASS_MIDI_FLOOR
    ]
    if not usable:
        return None
    return min(usable, key=lambda note: (abs(note - held), -note))


def _build_bass(
    chords: Sequence[ChordWindow], dense: bool, scale_pcs: frozenset
) -> Tuple[Tuple[Optional[int], float], ...]:
    """Бас (см. :func:`_bass_line`) — без счёта подходов."""
    return _bass_line(chords, dense, scale_pcs)[0]


#: ``bass_style`` → форма линии в окне (индексы тонов аккорда). ``auto`` —
#: :data:`_BASS_SHAPE_DENSE`/:data:`_BASS_SHAPE_SPARSE` по плотности.
_BASS_STYLE_SHAPES: Dict[str, Tuple[int, ...]] = {
    "root": (0,),
    "root_fifth": (0, 2),
}


def _bass_shape(style: str, dense: bool) -> Tuple[int, ...]:
    """Форма басовой линии для ``bass_style`` (ADR-0132 PR-3)."""
    if style in _BASS_STYLE_SHAPES:
        return _BASS_STYLE_SHAPES[style]
    return _BASS_SHAPE_DENSE if dense else _BASS_SHAPE_SPARSE


def _wants_approach(
    mode: str, is_last: bool, changes: bool, count: int, dur: float
) -> bool:
    """Ставить ли подход в последнюю восьмую ноты (``bass_approach``).

    ``auto`` — прежнее правило: только в окне из двух и более нот (в окне
    из одной ноты опора важнее движения). ``on`` — и в окне из одной ноты.
    ``off`` — никогда. Подход возможен только в последней ноте окна перед
    сменой корня и только если она длиннее самого подхода.
    """
    if mode == "off" or not (is_last and changes and dur > _APPROACH_BEATS):
        return False
    return mode == "on" or count > 1


def _pedal_bass(
    chords: Sequence[ChordWindow], tonic_pc: int
) -> Tuple[Tuple[Optional[int], float], ...]:
    """Педаль (``bass_style=pedal``): тоника лада на всё окно каждого аккорда."""
    note = _lift(tonic_pc, BASS_MIDI_FLOOR)
    return tuple((note, float(chord.beats)) for chord in chords)


def _styled_bass(
    chords: Sequence[ChordWindow],
    dense: bool,
    scale_pcs: frozenset,
    options: "HarmonizeOptions",
    tonic_pc: int,
) -> Tuple[Tuple[Tuple[Optional[int], float], ...], int]:
    """Бас по ручкам ``bass_style``/``bass_approach`` (ADR-0132 PR-3).

    ``off`` — партии нет (пустой кортеж: аранжировщик слой не добавит),
    ``pedal`` — :func:`_pedal_bass` без подходов, остальное —
    :func:`_bass_line`.
    """
    if options.bass_style == "off":
        return (), 0
    if options.bass_style == "pedal":
        return _pedal_bass(chords, tonic_pc), 0
    return _bass_line(
        chords, dense, scale_pcs, style=options.bass_style, approach=options.bass_approach
    )


def _bass_line(
    chords: Sequence[ChordWindow],
    dense: bool,
    scale_pcs: frozenset,
    style: str = "auto",
    approach: str = "auto",
) -> Tuple[Tuple[Tuple[Optional[int], float], ...], int]:
    """Бас и число поставленных нот-подходов (ADR-0132: видно в партитуре).

    Бас: тоны аккорда по долям, с подходом к следующему аккорду.

    ``dense`` (плотная тема, атака почти на каждой доле — марш, чиптюн)
    даёт бас четвертями: ровный шаг держит такую тему лучше, чем половины,
    под которыми она рассыпается. Разреженная тема получает половины,
    чтобы бас не забивал её собственное движение.

    Перед сменой гармонии последняя нота окна ДЕЛИТСЯ: тон аккорда звучит
    до последней восьмой, а в неё ложится подход к корню следующего
    аккорда (:func:`_approach_note`, :data:`_APPROACH_BEATS`). Подход не
    ставится, когда на всё окно приходится одна нота: там опора важнее
    движения. ``scale_pcs`` — классы высоты лада темы, из них берётся
    ступень подхода.

    Окна гармонии переменной длины (см. :func:`_pick_chords`), поэтому шаг
    раскладывается по фактической длине окна, а остаток достаётся
    последней ноте: сумма длительностей баса обязана совпадать с темой
    нота в ноту, иначе партии разъедутся на первом же повторе лупа.

    Аккорды берутся ПО КРУГУ: тема зациклена, и последнее окно ведёт не в
    тишину, а обратно в первое.

    ``style`` (:func:`_bass_shape`) и ``approach`` (:func:`_wants_approach`)
    — ручки ADR-0132 PR-3; ``auto`` у обеих — поведение выше.
    """
    step = 1.0 if dense else 2.0
    shape = _bass_shape(style, dense)
    out: List[Tuple[Optional[int], float]] = []
    approaches = 0
    for index, chord in enumerate(chords):
        following = chords[(index + 1) % len(chords)]
        changes = following.pitch_classes[0] != chord.pitch_classes[0]
        tones = tuple(
            chord.root_midi + (pc - chord.pitch_classes[0]) % 12
            for pc in chord.pitch_classes
        )
        count = max(1, int(chord.beats // step))
        remainder = chord.beats - count * step
        for position in range(count):
            is_last = position == count - 1
            note = tones[shape[position % len(shape)]]
            dur = step + (remainder if is_last else 0.0)
            if _wants_approach(approach, is_last, changes, count, dur):
                passing = _approach_note(note, following.root_midi, scale_pcs)
            else:
                passing = None
            if passing is None:
                out.append((note, dur))
                continue
            out.append((note, dur - _APPROACH_BEATS))
            out.append((passing, _APPROACH_BEATS))
            approaches += 1
    return tuple(out), approaches


def _scale_pitch_classes(root: str, scale: str) -> frozenset:
    """Классы высоты (0..11) лада темы."""
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    root_semitone = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    return frozenset((root_semitone + i) % 12 for i in intervals)


def _styled_pad(
    chords: Sequence[ChordWindow], dense: bool, style: str
) -> Tuple[Tuple[Tuple[Optional[Tuple[int, ...]], float], ...], Optional[float]]:
    """Пэд по ручке ``pad_style`` (ADR-0132 PR-3) и его ``sus``.

    ``auto``/``stab`` — остинато :func:`_build_pad` с коротким ударом
    :data:`arranger.PAD_STAB_SUS`; ``sustain`` — один аккорд на окно
    гармонии, звучит всю длину (``sus=None``); ``off`` — партии нет.
    """
    if style == "off":
        return (), PAD_STAB_SUS
    if style == "sustain":
        return tuple((chord.tones, float(chord.beats)) for chord in chords), None
    return _build_pad(chords, dense), PAD_STAB_SUS


def _build_pad(
    chords: Sequence[ChordWindow], dense: bool
) -> Tuple[Tuple[Optional[Tuple[int, ...]], float], ...]:
    """Подклад: аккорд, ПОВТОРЯЕМЫЙ по долям, а не выдержанный весь такт.

    🔴 FIX (live 14.09, «звучит плосковато, соло и фон»): здесь был один
    аккорд на такт длиной в такт. Выдержанное созвучие под подвижной темой
    на слух перестаёт быть партией и превращается в подложку — ровно то,
    что слышал человек. В настоящей оркестровке марша струнные ведут
    РИТМИЧЕСКОЕ остинато сквозь всю пьесу: повторяющаяся фигура и есть
    маршевый шаг, а гармонию она держит заодно.

    Плотная тема получает аккорд на каждой доле (маршевый шаг), разреженная
    — на сильных долях, чтобы не забивать собственное движение темы.
    Короткими эти удары делает ``sus`` на стороне аранжировщика: длинные
    ноты слились бы обратно в тот же выдержанный аккорд.
    """
    step = 1.0 if dense else 2.0
    out: List[Tuple[Optional[Tuple[int, ...]], float]] = []
    for chord in chords:
        hits = max(1, int(round(chord.beats / step)))
        out.extend([(chord.tones, step)] * hits)
    return tuple(out)


def _build_counter(
    timed: Sequence[Tuple[float, Optional[int], float]],
    chords: Sequence[ChordWindow],
) -> Tuple[Tuple[Optional[int], float], ...]:
    """Контрмелодия: тема, положенная на ближайший аккордовый тон снизу.

    Ритм — ритм темы нота в ноту, высота — ближайший тон текущего аккорда
    строго ниже ноты темы (терция или секста снизу, смотря что подвернётся
    в аккорде). Это классический второй голос: он движется вместе с темой,
    но никогда не спорит с гармонией, потому что состоит только из её нот.
    Паузы темы остаются паузами.
    """
    out: List[Tuple[Optional[int], float]] = []
    for onset, midi, dur in timed:
        if midi is None:
            out.append((None, dur))
            continue
        chord = _chord_at(chords, onset)
        if chord is None:
            out.append((None, dur))
            continue
        below = [
            note
            for pc in chord.pitch_classes
            for note in (midi - ((midi - pc) % 12 or 12),)
        ]
        # Ближайший аккордовый тон снизу, но не ближе малой терции:
        # секунда под темой звучит как грязь, а не как второй голос.
        candidates = [n for n in below if midi - n >= 3]
        out.append((max(candidates) if candidates else None, dur))
    return tuple(out)


def _chord_at(
    chords: Sequence[ChordWindow], beat: float
) -> Optional[ChordWindow]:
    for chord in chords:
        if chord.start <= beat < chord.start + chord.beats:
            return chord
    return chords[-1] if chords else None


# ---------------------------------------------------------------------------
# Ударные
# ---------------------------------------------------------------------------


def _onset_histogram(
    timed: Sequence[Tuple[float, Optional[int], float]],
) -> List[float]:
    """Плотность атак темы по 16-м долям такта, сложенная по всем тактам.

    Свёртка по тактам (а не по всей теме) — потому что рисунок ударных
    длиной в такт: нам нужно, КУДА внутри такта тема чаще всего бьёт, а
    не где она бьёт в конкретном такте.
    """
    hist = [0.0] * STEPS_PER_BAR
    step_beats = BEATS_PER_BAR / STEPS_PER_BAR
    for onset, midi, dur in timed:
        if midi is None:
            continue
        step = int(round((onset % BEATS_PER_BAR) / step_beats)) % STEPS_PER_BAR
        # Длинная нота на доле — более весомая опора, чем проходящая
        # шестнадцатая, поэтому вес атаки растёт с её длительностью.
        hist[step] += 1.0 + min(float(dur), 2.0)
    return hist


#: Issue #2841 — жанровые каркасы ударных. Живой сет 23.09.2026: у всех
#: RTTTL-тем один и тот же бит (``X...o...X...o...`` / ``-.-.-.-.``), потому
#: что каркас ниже прибит намертво. ``auto`` — прежнее поведение (бочка на
#: 1, малый на 2 и 4, бочка на 3 у плотной темы, хэты по плотности); прочие
#: стили — готовые рисунки жанра, которые модель выбирает параметром
#: ``drum_style`` в ``compose_music``.
DEFAULT_DRUM_STYLE = "auto"
DRUM_STYLES: Tuple[str, ...] = (
    "auto", "four_on_floor", "backbeat", "halftime", "breakbeat", "march", "none",
)

#: Стиль -> (бочка/малый для редкой темы, для плотной). 16 шагов на такт.
#: Каждый каркас держит опору на первой доле — иначе тема и бит не
#: сходятся в такт (см. FIX live 14.09 в :func:`_build_drums`).
_STYLE_DRUMS: Dict[str, Tuple[str, str]] = {
    # Хаус/диско: бочка на каждую долю. Малого нет — один play() не
    # кладёт два символа на один шаг, а бочка на 2 и 4 важнее хлопка.
    "four_on_floor": ("X...X...X...X...", "X...X...X...X..."),
    # Поп/рок: малый на 2 и 4; плотной теме — бочка ещё и на «и» третьей.
    "backbeat": ("X...o...X...o...", "X...o...X.X.o..."),
    # Халфтайм (трэп, даб, медленный рок): малый только на третьей доле.
    "halftime": ("X.......o.......", "X.....X.o......."),
    # Брейкбит/бум-бэп: синкопированная бочка вокруг малого на 2 и 4.
    "breakbeat": ("X...o..X..X.o...", "X.X.o..X..X.o..o"),
    # Марш: квадратный шаг, дробь малого перед сильной долей.
    "march": ("X...o...X...o.o.", "X...o.o.X...o.oo"),
    "none": ("", ""),
}

#: Стиль -> фиксированный рисунок хэтов. Стиля нет в словаре — хэты по
#: плотности темы (:func:`_build_hats`, прежнее поведение).
_STYLE_HATS: Dict[str, str] = {
    "four_on_floor": "..-...-...-...-.",   # офбит — «хаусный» открытый хэт
    "halftime": "-.-.-.-.-.-.-.-.",
    "breakbeat": "-.-.-.---.-.-.-.",
    "march": "-...-...-...-...",
    "none": "",
}


def check_drum_style(drum_style: Optional[str]) -> str:
    """Нормализовать ``drum_style`` (``None``/пусто -> ``auto``).

    Raises:
        ValueError: неизвестный стиль — сообщение перечисляет допустимые.
    """
    style = (drum_style or DEFAULT_DRUM_STYLE).strip().lower()
    if style not in DRUM_STYLES:
        raise ValueError(
            f"Неизвестный drum_style {drum_style!r}. Доступны: "
            f"{', '.join(DRUM_STYLES)}."
        )
    return style


def style_patterns(drum_style: str, dense: bool) -> Tuple[str, str]:
    """Готовые ``(бочка/малый, хэты)`` стиля — для сочинённого трека без темы.

    ``auto`` здесь даёт бэкбит: без темы выводить рисунок не из чего, а
    бэкбит — то, что ``auto`` и так строит на теме.
    """
    style = check_drum_style(drum_style)
    if style == DEFAULT_DRUM_STYLE:
        style = "backbeat"
    sparse, full = _STYLE_DRUMS[style]
    hats = _STYLE_HATS.get(style, "-.-.-.-.-.-.-.-.")
    return (full if dense else sparse), hats


def _build_drums(
    hist: Sequence[float], dense: bool, style: str = DEFAULT_DRUM_STYLE
) -> str:
    """Рисунок бочки и малого: жёсткий каркас + синкопа от мелодии.

    🔴 FIX (live 14.09, «ломаные ритмы»): здесь бочка ставилась на первую
    долю и на ту из оставшихся, куда тема бьёт сильнее всего, а малый —
    на все прочие. Правило выглядело «выводим грув из мелодии», а на деле
    давало хромые рисунки: у Pink Panther вышло ``X...o...o...X...``
    (бочка на 1 и 4, то есть ДВЕ БОЧКИ ПОДРЯД на стыке тактов), у Марио —
    ``X...X...o...o...`` (бочка на 1 и 2). Ни то, ни другое не читается
    как доля.

    Обратный бит — малый на 2 и 4 — это не вкус и не свойство конкретной
    мелодии, а то, из чего вообще складывается ощущение доли в
    четырёхдольном размере. Поэтому каркас фиксирован: бочка на 1, малый
    на 2 и 4. Мелодия решает ровно одно: даётся ли бочка на 3.

    🔴 FIX #2 (live 14.09): вместе с починкой каркаса сюда добавлялась
    ещё и «синкопа от мелодии» — добавочная бочка на той восьмушке между
    долями, куда тема бьёт сильнее. Задумывалась как то, что отличает
    грув одного трека от другого, а на деле сломала марш: он получил
    бочку на «и» второй доли сразу после малого (``X...o.X.X...o...``),
    и квадратный маршевый шаг превратился в хромой. Признак был негодный
    — у марша вес распределён 40% на долях и 40% между ними, то есть он
    вовсе не синкопирован, просто много шестнадцатых. Разнообразие грува
    даёт выбор сэмпла и плотность хэтов; выдумывать его в рисунке бочки
    не нужно.

    Issue #2841: ``style`` не ``auto`` — готовый каркас жанра
    (:data:`_STYLE_DRUMS`); плотность темы выбирает только его вариант.
    """
    skeleton = _STYLE_DRUMS.get(style)
    if skeleton is not None:
        return skeleton[1] if dense else skeleton[0]
    pattern = ["."] * STEPS_PER_BAR
    pattern[0] = "X"
    pattern[4] = "o"
    pattern[12] = "o"
    if dense:
        pattern[8] = "X"
    return "".join(pattern)


def _build_hats(
    timed: Sequence[Tuple[float, Optional[int], float]],
    style: str = DEFAULT_DRUM_STYLE,
) -> str:
    """Хэты: сетка по плотности темы — 16-е, 8-е или четверти.

    Медиана длительности нот — устойчивая мера «мелкости» темы (среднее
    сбивает одна длинная финальная нота). Хэты мельче самой темы звучат
    как суета, крупнее — как будто их забыли включить.

    Issue #2841: у стилей из :data:`_STYLE_HATS` рисунок фиксирован
    жанром (офбит хауса, четверти марша); ``auto`` и ``backbeat`` —
    прежняя сетка по плотности.
    """
    fixed = _STYLE_HATS.get(style)
    if fixed is not None:
        return fixed
    durs = [dur for _onset, midi, dur in timed if midi is not None]
    typical = median(durs) if durs else 1.0
    if typical <= 0.3:
        every = 1
    elif typical <= 0.6:
        every = 2
    else:
        every = 4
    return "".join(
        "-" if step % every == 0 else "." for step in range(STEPS_PER_BAR)
    )


# ---------------------------------------------------------------------------
# Единый вход
# ---------------------------------------------------------------------------


def harmonize(
    notes: Sequence[Tuple[Optional[int], float]],
    bpm: int,
    root: str,
    scale: str,
    drum_style: str = DEFAULT_DRUM_STYLE,
    options: Optional[HarmonizeOptions] = None,
) -> Harmonization:
    """Разложить тему на партии: аккорды, бас, пэд, контрмелодию, ударные.

    Args:
        notes: тема как ``[(midi|None, биты)]`` — выход RTTTL-парсера,
            уже выровненный по такту.
        bpm: темп темы.
        root: тоника, определённая по теме (``detect_key``).
        scale: лад, определённый по теме.
        drum_style: жанровый каркас ударных (:data:`DRUM_STYLES`, issue
            #2841); ``auto`` — прежний рисунок, выведенный из темы.
        options: ручки раскладки (:class:`HarmonizeOptions`, ADR-0132 PR-3);
            ``None`` — все ``auto``, прежнее поведение байт-в-байт.
            Ручки подготовки темы (``key_detection``, ``lead_octave``,
            ``lead_outliers``) здесь не читаются — их исполняет
            ``rtttl_compose.melody_to_compose_params`` до вызова.

    Returns:
        :class:`Harmonization` — все партии в абсолютных MIDI и битах.

    Raises:
        ValueError: тема пустая или состоит из одних пауз — выводить
            гармонию не из чего; неизвестный ``drum_style``; аккордов
            ``options.chords`` больше, чем тактов в теме.
    """
    style = check_drum_style(drum_style)
    options = options or HarmonizeOptions()
    if not notes:
        raise ValueError("Пустая тема: гармонизировать нечего.")
    if all(midi is None for midi, _dur in notes):
        raise ValueError("Тема состоит из одних пауз: гармонизировать нечего.")

    timed, total = _timed(notes)
    chords = _pick_chords(timed, total, root, scale, options)
    hist = _onset_histogram(timed)

    onsets = sum(1 for _onset, midi, _dur in timed if midi is not None)
    density = onsets / total if total > 0 else 0.0
    dense = _resolve_dense(density, options.density)
    tonic_pc = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    bass, approaches = _styled_bass(
        chords, dense, _scale_pitch_classes(root, scale), options, tonic_pc
    )
    pad, pad_sus = _styled_pad(chords, dense, options.pad_style)

    return Harmonization(
        bpm=int(bpm),
        root=root,
        scale=scale,
        bars=max(1, int(round(total / BEATS_PER_BAR))),
        density=density,
        dense=dense,
        chords=chords,
        lead=tuple((midi, float(dur)) for midi, dur in notes),
        bass=bass,
        pad=pad,
        counter=_build_counter(timed, chords),
        drums=_build_drums(hist, dense, style) if options.drums is None else options.drums,
        hats=_build_hats(timed, style) if options.hats is None else options.hats,
        pad_sus=pad_sus,
        decisions={
            **_harmony_decisions(timed, dense, approaches, style),
            **_option_decisions(options),
        },
    )


def _resolve_dense(density: float, mode: str) -> bool:
    """Плотная ли тема: по атакам на бит или по ручке ``density`` (PR-3)."""
    if mode == "dense":
        return True
    if mode == "sparse":
        return False
    return density >= DENSE_ONSETS_PER_BEAT


def _harmony_decisions(
    timed: Sequence[Tuple[float, Optional[int], float]],
    dense: bool,
    approaches: int,
    style: str,
) -> Dict[str, object]:
    """Запись авто-решений раскладки (ADR-0132) — только для партитуры.

    Значения пересчитываются теми же правилами, что строили партии, и на
    сами партии не влияют.
    """
    step = 1.0 if dense else 2.0
    return {
        "dense_threshold": DENSE_ONSETS_PER_BEAT,
        "bass_step": step,
        "bass_approaches": approaches,
        "pad_step": step,
        "pad_theme_ceiling": _pad_ceiling(timed),
        "drum_style": style,
    }


def _option_decisions(options: HarmonizeOptions) -> Dict[str, object]:
    """Значения ручек раскладки, с которыми построены партии (ADR-0132 PR-3).

    Партитура по ним пишет ``ручка=auto→…`` или заданное значение.
    ``chords``/``drums``/``hats`` — ``auto`` или ``explicit``.
    """
    return {
        "knob_density": options.density,
        "knob_harmonic_rhythm": options.harmonic_rhythm,
        "knob_chords": AUTO if options.chords is None else "explicit",
        "knob_bass_style": options.bass_style,
        "knob_bass_approach": options.bass_approach,
        "knob_pad_style": options.pad_style,
        "knob_pad_register": options.pad_register,
        "knob_drums": AUTO if options.drums is None else "explicit",
        "knob_hats": AUTO if options.hats is None else "explicit",
    }
