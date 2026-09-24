"""RTTTL-мелодия → плоские параметры ``compose_music``.

Связка между RTTTL-библиотекой (SQLite, :mod:`core.rtttl_library`) и
композитором (``ComposeMusicTool``). Раньше модель сама разбирала RTTTL и
генерировала Renardo-код вручную — это был ручной шаг, на котором она
ошибалась (корень #1810 «сыграл гамму и назвал её кузнечиком»). Теперь
``compose_music(name=..., variants=...)`` сам ищет мелодию, конвертирует
ноты в абсолютные MIDI и передаёт их аранжировщику.

Абсолютные MIDI (``lead_midi``) + точный ритм (``lead_dur``) — путь ТОЧНОГО
воспроизведения: аранжировщик играет тему дословно, а форму, бас и ударные
строит вокруг неё. Ступени лада (``lead_notes``) сюда не подходят: в них
нельзя выразить хроматические ноты (диез/бемоль вне лада), поэтому точность
мелодии была бы потеряна.

Ручки подготовки темы (ADR-0132 PR-3)
-------------------------------------
Три авто-решения подготовки — ``HarmonizeOptions.key_detection``
(``auto`` — корреляция + тональный центр #2873, ``profile`` — чистый
Крумхансл), ``lead_octave`` (``auto`` — к рабочему регистру, ``keep``,
либо -2..+2 октавы от записанного) и ``lead_outliers`` (``fix``/``keep``)
— исполняются здесь, до :func:`~core.harmonize.harmonize`; остальные
ручки передаются в неё. Выбранное значение каждой ручки пишется в
``decisions`` (``key_detection``, ``lead_octave_mode``,
``lead_outliers_mode``) для партитуры. Все по умолчанию — байт-в-байт
прежнее поведение (``test_arranger_golden``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from .arranger import BEATS_PER_BAR, SCALE_INTERVALS, VALID_ROOTS
from .harmonize import (
    AUTO,
    DEFAULT_DRUM_STYLE,
    HarmonizeOptions,
    _weighted_percentile,
    harmonize,
)
from .rtttl import parse_rtttl

__all__ = [
    "RtttlMelody",
    "rtttl_to_melody",
    "KeyCandidate",
    "detect_key",
    "detect_key_ranked",
    "melody_to_compose_params",
    "key_fit",
]

#: Профили Крумхансл-Шмуклера: насколько «своей» слышится каждая ступень
#: хроматики в мажоре и в миноре. Числа — усреднённые оценки слушателей
#: из психоакустических экспериментов Кэрол Крумхансл; тоника весит
#: больше всех, за ней доминанта и медианта.
#:
#: 🔴 FIX (live 14.09): здесь считалось, сколько веса нот ПОПАДАЕТ в лад.
#: Такой счёт не различает параллельные тональности и лады-повороты в
#: принципе: у ля-минора и до-мажора набор нот совпадает полностью, у
#: ля-фригийского и фа-мажора тоже — счёт у них одинаков до последнего
#: знака, и выбор решал порядок перебора, то есть монетка. «В пещере
#: горного короля» (ля-минор) определялась как ре-мажор, «Ода к радости»
#: (фа-мажор) — как ля-фригийский.
#:
#: Профиль различает их, потому что смотрит НЕ на вхождение ноты в лад, а
#: на то, какие ступени несут вес: тема в миноре задерживается на минорной
#: терции, тема в мажоре — на большой. Сравнение идёт корреляцией, а не
#: суммой, чтобы результат не зависел от общей длины темы.
_KRUMHANSL_MAJOR = (
    6.35, 2.23, 3.48, 2.33, 4.38, 4.09, 2.52, 5.19, 2.39, 3.66, 2.29, 2.88,
)
_KRUMHANSL_MINOR = (
    6.33, 2.68, 3.52, 5.38, 2.60, 3.53, 2.54, 4.75, 3.98, 2.69, 3.34, 3.17,
)


#: Штраф за долю веса, лежащую ВНЕ лада. Соразмерен корреляции (та живёт
#: в [-1, 1]), поэтому тональность, не содержащую заметной части нот темы,
#: он снимает, а на выбор между двумя одинаково подходящими не влияет.
_OUT_OF_SCALE_PENALTY = 2.0

#: 🔴 FIX (live 23.09, issue #2873): гистограмма высот одна не знает, ГДЕ
#: звучит нота. «В пещере горного короля» (си-минор: B C# D E F# D F#,
#: дальше хроматическая секвенция F C# F · E C E) бо́льшую часть времени
#: стоит на F# и кончается долгим A — корреляция уверенно выбирала
#: ля-мажор, а версия в ля-миноре уезжала в ми-минор. Слушатель же
#: слышит тонику по ПОЗИЦИИ: тема начинается с тоники и в первых нотах
#: проходит её трезвучие; фрагмент часто кончается на тонике или
#: доминанте. Поэтому к корреляции добавлена «опора на тоническое
#: трезвучие» — три позиционных свидетельства, каждое объяснимо на слух.
#:
#: Веса подобраны перебором по эталонной таблице (``test_rtttl_compose``,
#: ``_KEY_REFERENCE``, 36 тем). Порядок величин — как у корреляции, чтобы
#: позиционная опора решала спор близких тональностей и перевешивала
#: гистограмму только там, где тема ЯВНО утверждает тонику. Честно: на
#: части тем запас мал (40-я Моцарта, «Jingle Bells», «Rudolph» — 0.02-0.03),
#: поэтому любой сдвиг весов проверять прогоном всей таблицы.
#:
#: Длина «начала фразы» в звучащих нотах (без пауз): столько нот обычно
#: занимает первый мотив, утверждающий тональность (B C# D E F# D F# у
#: Грига, E D# E D# E B D C у «К Элизе»).
_OPENING_NOTES = 8
#: Доля начала фразы, лежащая на тоническом трезвучии кандидата.
_OPENING_TRIAD_WEIGHT = 0.8
#: Первая сильная нота (затакт пропущен) — звук тонического трезвучия.
_FIRST_NOTE_WEIGHT = 0.2
#: Последняя нота — звук тонического трезвучия. Весит меньше первой:
#: рингтон — часто обрывок, конец которого приходится на середину
#: периода (у Грига вторая фраза кончается на VII ступени — A в си-миноре).
_LAST_NOTE_WEIGHT = 0.1
#: Сколько опоры даёт каждый звук трезвучия. Тоника — полная; квинта —
#: почти полная (темы часто начинаются с доминанты: «К Элизе», «Тетрис»,
#: Пятая Бетховена); терция — половина: мажорная тема, начатая с терции
#: («Jingle Bells» с A в фа-мажоре, «Белое Рождество»), иначе уехала бы
#: в минор от этой терции.
_TRIAD_CREDIT = {0: 1.0, 7: 0.75, 3: 0.5, 4: 0.5}


def _correlation(xs: Sequence[float], ys: Sequence[float]) -> float:
    """Корреляция Пирсона двух векторов одной длины (0.0 при вырождении)."""
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    dx = [x - mean_x for x in xs]
    dy = [y - mean_y for y in ys]
    denom = (sum(v * v for v in dx) * sum(v * v for v in dy)) ** 0.5
    if denom == 0:
        return 0.0
    return sum(a * b for a, b in zip(dx, dy)) / denom


@dataclass(frozen=True)
class RtttlMelody:
    """Разобранная RTTTL-мелодия: темп + список ``(midi|None, доли)``.

    ``midi`` — абсолютный номер MIDI-ноты (``None`` — пауза).
    ``dur`` — длительность ноты в битах (четверть = 1.0).
    """

    bpm: int
    notes: Tuple[Tuple[Optional[int], float], ...]


def rtttl_to_melody(rtttl: str) -> RtttlMelody:
    """Разобрать RTTTL-строку в :class:`RtttlMelody` (через ``core.rtttl``)."""
    _name, bpm, notes = parse_rtttl(rtttl)
    return RtttlMelody(bpm=bpm, notes=tuple(notes))


def _pitch_weights(sounding: Sequence[Tuple[int, float]]) -> Dict[int, float]:
    """Вес каждого класса высот: сумма КОРНЕЙ длительностей его нот.

    Длительность учитывается (долгая/частая тоника перевешивает проходящие
    ноты), но сублинейно: рингтон часто кончается выдержанной нотой в 2-4
    доли, и при линейном весе одна она занимает треть гистограммы короткой
    темы (у Грига финальное A/2 тянуло тональность в ля-мажор, #2873).
    """
    weights: Dict[int, float] = {}
    for pc, dur in sounding:
        weights[pc] = weights.get(pc, 0.0) + max(dur, 0.0) ** 0.5
    return weights


def _profile_score(weights: Dict[int, float], root: int, scale: str) -> float:
    """Корреляция с профилем Крумхансл минус штраф за ноты вне лада."""
    profile = [weights.get(pc, 0.0) for pc in range(12)]
    total = sum(profile) or 1.0
    rotated = profile[root:] + profile[:root]
    reference = _KRUMHANSL_MAJOR if scale == "major" else _KRUMHANSL_MINOR
    # Корреляция объясняет ИЕРАРХИЮ ступеней, но ничего не знает о
    # принадлежности: она не против ноты, которой в ладу нет вовсе.
    # На коротком фрагменте этого мало — «Jingle Bells» (фа-мажор)
    # почти не касается своей тоники и всем весом лежит на терции,
    # из-за чего выигрывал ля-минор, где си-бемоля темы просто нет.
    # Второй член требует, чтобы лад ещё и СОДЕРЖАЛ ноты темы.
    in_scale = {i % 12 for i in SCALE_INTERVALS[scale]}
    outside = sum(
        w for pc, w in weights.items() if (pc - root) % 12 not in in_scale
    )
    return _correlation(rotated, reference) - _OUT_OF_SCALE_PENALTY * (
        outside / total
    )


def _first_strong_note(
    sounding: Sequence[Tuple[int, float]], root: Optional[int] = None
) -> int:
    """Первая «опорная» нота темы: настоящий затакт пропускается.

    Короткая первая нота перед более долгой — затакт (гимн России: G/8,
    пауза, C/4 на сильной доле). Тональность утверждает нота сильной
    доли, а не затакт — обычно доминанта, к тонике кандидата отношения
    не имеющая.

    🔴 FIX (issue #2961): затакт — это подход К тонике, а не звук самой
    тоники. Если короткая первая нота — это САМА тоника кандидата (тема
    Терминатора: 16d перед 8e — D и есть тоника ре-минора), отбрасывать
    её нельзя: это не затакт, а укороченное вступление в тонику.
    Проверка — именно на тонику (``root``), не на трезвучие целиком:
    более широкая проверка (любой звук трезвучия) пробовалась и снята —
    она меняет то, какая доля темы служит «первой сильной нотой» почти
    для любого кандидата (у C minor и F minor общая пятая ступень C, и
    из-за неё ``stilldre_2`` уезжал в C minor вместо F minor), поэтому
    небезопасна. Тоника — однозначный, не делимый с соседними
    кандидатами признак: одна и та же нота — тоника ровно одного
    кандидата на каждой из 12 высот.
    """
    if root is not None and sounding[0][0] == root:
        return sounding[0][0]
    if len(sounding) > 1 and sounding[0][1] < sounding[1][1]:
        return sounding[1][0]
    return sounding[0][0]


def _anchor(pc: int, root: int, third: int) -> float:
    """Опора ноты на тоническое трезвучие (см. :data:`_TRIAD_CREDIT`)."""
    interval = (pc - root) % 12
    if interval in (0, 7, third):
        return _TRIAD_CREDIT[interval]
    return 0.0


def _tonal_center_score(
    sounding: Sequence[Tuple[int, float]], root: int, scale: str
) -> float:
    """Позиционная опора кандидата ``(root, scale)`` — см. #2873.

    Три свидетельства, которые слух использует для тоники:

    * начало фразы (первые :data:`_OPENING_NOTES` нот) лежит на тоническом
      трезвучии — минорном или мажорном в зависимости от лада, поэтому
      параллельные тональности (ля-минор / до-мажор) здесь различаются;
    * первая сильная нота (настоящий затакт пропущен, см. #2961 у
      :func:`_first_strong_note`) — звук тонического трезвучия;
    * последняя нота — звук тонического трезвучия.
    """
    third = 4 if scale == "major" else 3
    triad = frozenset({root, (root + third) % 12, (root + 7) % 12})
    opening = sounding[:_OPENING_NOTES]
    opening_total = sum(d ** 0.5 for _pc, d in opening) or 1.0
    on_triad = sum(d ** 0.5 for pc, d in opening if pc in triad)
    first = _first_strong_note(sounding, root)
    last = sounding[-1][0]
    return (
        _OPENING_TRIAD_WEIGHT * on_triad / opening_total
        + _FIRST_NOTE_WEIGHT * _anchor(first, root, third)
        + _LAST_NOTE_WEIGHT * _anchor(last, root, third)
    )


class KeyCandidate(NamedTuple):
    """Кандидат тональности с его скором (ADR-0132: уверенность видна модели)."""

    root: str
    scale: str
    score: float


def _minor_variant(weights: Dict[int, float], root: int) -> str:
    """Натуральный минор или гармонический — решает седьмая ступень.

    Повышенная (вводный тон) против натуральной — единственное, чем они
    отличаются, и профиль минора их не различает (он один на оба).
    """
    raised_seventh = weights.get((root + 11) % 12, 0.0)
    natural_seventh = weights.get((root + 10) % 12, 0.0)
    return "harmonicMinor" if raised_seventh > natural_seventh else "minor"


def detect_key_ranked(
    midi_notes: Sequence[Optional[int]],
    durations: Optional[Sequence[float]] = None,
    method: str = AUTO,
) -> List[KeyCandidate]:
    """Все 24 кандидата ``(тоника, лад)`` по убыванию скора.

    Скор каждой пары (тоника, лад) складывается из двух частей:

    1. **Гистограмма** (:func:`_profile_score`): корреляция взвешенной по
       длительности гистограммы высот с профилем Крумхансл минус штраф за
       ноты вне лада. Без ``durations`` все ноты весят одинаково.
    2. **Тонический центр** (:func:`_tonal_center_score`): начинается ли
       тема с трезвучия кандидата, стоят ли звуки этого трезвучия (тоника
       весомее всех) первой сильной и последней нотой. Гистограмма не
       знает, ГДЕ звучит нота, и на хроматических темах промахивается на
       тон-кварту (#2873).

    Сортировка устойчивая: при равном скоре первым остаётся кандидат,
    раньше стоящий в переборе (C major, C minor, C# major, ...) — ровно
    тот, кого выбирал ``max()`` в прежнем :func:`detect_key`. Минорный
    кандидат уточняется до ``harmonicMinor`` по седьмой ступени.

    ADR-0132: разрыв между первым и вторым кандидатом — мера уверенности,
    её показывает партитура. Без звучащих нот — ``[("C", "major", 0.0)]``.

    ``method`` (ADR-0132 PR-3, ручка ``key_detection``): ``auto`` — обе
    части скора; ``profile`` — только гистограмма (чистый Крумхансл со
    штрафом за ноты вне лада), без позиционной опоры.
    """
    if durations is None:
        durations = [1.0] * len(midi_notes)
    sounding = [
        (int(midi) % 12, float(dur))
        for midi, dur in zip(midi_notes, durations)
        if midi is not None
    ]
    if not sounding:
        return [KeyCandidate("C", "major", 0.0)]

    weights = _pitch_weights(sounding)
    positional = method != "profile"
    scored = [
        (root, scale, _profile_score(weights, root, scale)
         + (_tonal_center_score(sounding, root, scale) if positional else 0.0))
        for root in range(12)
        for scale in ("major", "minor")
    ]
    scored.sort(key=lambda item: item[2], reverse=True)
    return [
        KeyCandidate(
            VALID_ROOTS[root],
            scale if scale == "major" else _minor_variant(weights, root),
            score,
        )
        for root, scale, score in scored
    ]


def detect_key(
    midi_notes: Sequence[Optional[int]],
    durations: Optional[Sequence[float]] = None,
) -> Tuple[str, str]:
    """Определить ``(тоника, лад)`` по абсолютным MIDI-нотам темы.

    Лучший кандидат :func:`detect_key_ranked` (там — как считается скор).
    Паузы (``None``) игнорируются. Без нот — ``("C", "major")``.

    Тональность нужна не для самой темы (она играется абсолютным MIDI),
    а для баса и подклада, которые аранжировщик достраивает вокруг неё.
    """
    best = detect_key_ranked(midi_notes, durations)[0]
    return best.root, best.scale


def melody_to_compose_params(
    melody: RtttlMelody,
    drum_style: str = DEFAULT_DRUM_STYLE,
    root: Optional[str] = None,
    scale: Optional[str] = None,
    options: Optional[HarmonizeOptions] = None,
) -> Dict[str, object]:
    """RTTTL-мелодия → плоские параметры ``compose_music``.

    Возвращает dict с ключами:
      * ``bpm`` — темп из RTTTL (``compose_music.bpm``);
      * ``root`` / ``scale`` — определённая тональность (для баса/подклада);
      * ``lead_midi`` — строка абсолютных MIDI через запятую (``None`` = пауза);
      * ``lead_dur`` — ритм в битах, той же длины;
      * ``harmony`` — :class:`core.harmonize.Harmonization`: та же тема,
        разложенная на бас, пэд, контрмелодию и рисунки ударных.

    Мелодия выравнивается по такту с обоих концов: затакт в начале сдвигает
    сетку паузой-лид-ином, чтобы первая сильная нота темы попала на долю 0
    такта (:func:`_anacrusis_lead_in`, issue #2960), а хвостовая пауза
    доводит луп до целого числа тактов (:func:`_snap_to_bar`) — иначе луп
    плывёт относительно ударной сетки и тема звучит «не в тайминг».

    НОТЫ аккомпанемента модель больше не выбирает: они выведены из самой
    темы (``harmony``). За моделью остаются тембры, форма и темп — см.
    :mod:`core.harmonize`. Плоские ``lead_midi``/``lead_dur`` остаются в
    ответе для обратной совместимости и для логов.

    ``drum_style`` — жанровый каркас ударных темы (issue #2841, см.
    :data:`core.harmonize.DRUM_STYLES`); ``auto`` — прежний рисунок.

    ``decisions`` (ADR-0132) — что автоматика решила за модель по дороге:
    свёртка темпа, хвостовая пауза, перенос регистра темы, подтянутые
    выбросы, ранжированные кандидаты тональности. Только запись: на ноты
    и на аккомпанемент она не влияет (golden-тест ``test_arranger_golden``).

    ``root`` / ``scale`` (ADR-0132 PR-2) — явная тональность от модели:
    аккомпанемент ПЕРЕГАРМОНИЗИРУЕТСЯ в ней вместо определённой по теме
    (тема играется как есть — абсолютным MIDI). Заданная только тоника
    берёт лад определённой тональности, заданный только лад — её тонику.
    Значения должны быть уже проверены (``arranger.check_root`` /
    ``check_scale``). Оба ``None`` — прежнее поведение байт-в-байт.
    Спорная с мелодией тональность не отклоняется — решает модель, а в
    ``decisions`` пишутся ``key_detected`` и ``key_fit`` (доля
    длительности темы в заданном ладу) для предупреждения партитуры.

    ``options`` (ADR-0132 PR-3) — ручки :class:`core.harmonize.HarmonizeOptions`:
    ``key_detection``/``lead_octave``/``lead_outliers`` исполняются здесь,
    остальные — в :func:`~core.harmonize.harmonize`. ``None`` — все ``auto``.
    """
    options = options or HarmonizeOptions()
    source_bpm = melody.bpm
    folded = _normalize_tempo(melody)
    leadin = _anacrusis_lead_in(folded)
    snapped = _snap_to_bar(leadin)
    registered = _apply_lead_octave(snapped, options.lead_octave)
    melody = _apply_lead_outliers(registered, options.lead_outliers)
    ranked = detect_key_ranked(
        [m for m, _ in melody.notes],
        [d for _, d in melody.notes],
        method=options.key_detection,
    )
    explicit = root is not None or scale is not None
    root = root or ranked[0].root
    scale = scale or ranked[0].scale
    midi: List[str] = ["None" if m is None else str(int(m)) for m, _ in melody.notes]
    dur: List[str] = [f"{d:g}" for _, d in melody.notes]
    anacrusis_pad = sum(d for _, d in leadin.notes) - sum(d for _, d in folded.notes)
    decisions = _prep_decisions(
        source_bpm, (folded, leadin, snapped, registered, melody), ranked
    )
    decisions.update(_prep_option_decisions(options))
    if explicit:
        decisions.update(_explicit_key_decisions(melody, ranked[0], root, scale))
    return {
        "bpm": melody.bpm,
        "root": root,
        "scale": scale,
        "lead_midi": ", ".join(midi),
        "lead_dur": ", ".join(dur),
        "harmony": harmonize(
            melody.notes, melody.bpm, root, scale, drum_style=drum_style,
            options=options, anacrusis_pad=anacrusis_pad,
        ),
        "decisions": decisions,
    }


def _apply_lead_octave(melody: "RtttlMelody", mode: object) -> "RtttlMelody":
    """Регистр темы по ручке ``lead_octave`` (ADR-0132 PR-3).

    ``auto`` — :func:`_normalize_lead_register` (к рабочему регистру);
    ``keep`` — как записано, БЕЗ нормализации; целое N — N октав от уже
    НОРМАЛИЗОВАННОГО регистра (см. ниже), с клампом в рабочий диапазон.

    🔴 FIX (live 24.09, issue #2962): ручной сдвиг применялся поверх
    СЫРОЙ, ненормализованной темы — ``_LEAD_MAX_CEILING`` в этом случае не
    проверялся вовсе. Live-прогон: ``terminat`` (auto переносит в рабочий
    регистр, медиана ~80) + ``lead_octave='+1'`` → тема уехала в MIDI
    104-111 (свист, «темы не слышно»), контрмелодия следом за ней — до 107
    (:func:`~core.harmonize._build_counter` кладёт её от нот темы, поэтому
    отдельного клампа не требует — чинится клампом самой темы).

    Теперь ручной сдвиг считается от того же нормализованного регистра,
    что и ``auto`` (:func:`_normalize_lead_register`) — ``+1``/``-1``
    значит «на октаву выше/ниже РАБОЧЕГО регистра», а не записанного as
    is. Если результат не помещается в :data:`_LEAD_MAX_CEILING` /
    :data:`_LEAD_MIN_FLOOR` — честная ``ValueError`` вместо тихого выхода
    за рабочий диапазон: смысла в частичном (не целую октаву) сдвиге нет
    — это была бы уже не та тема.
    """
    if mode == AUTO:
        return _normalize_lead_register(melody)
    if mode == "keep":
        return melody
    shift_octaves = int(mode)  # type: ignore[call-overload]
    if shift_octaves == 0:
        return melody
    base = _normalize_lead_register(melody)
    pitches = [m for m, _dur in base.notes if m is not None]
    if not pitches:
        return base
    shift = 12 * shift_octaves
    lo, hi = min(pitches), max(pitches)
    if hi + shift > _LEAD_MAX_CEILING:
        raise ValueError(
            f"lead_octave={mode!r}: тема уже у потолка рабочего регистра "
            f"после нормализации (макс. нота {hi}, потолок "
            f"{_LEAD_MAX_CEILING}) — выше сдвигать нельзя, иначе тема "
            "уйдёт в свист. Оставь auto/keep или меньший сдвиг."
        )
    if lo + shift < _LEAD_MIN_FLOOR:
        raise ValueError(
            f"lead_octave={mode!r}: тема уже у пола рабочего регистра "
            f"после нормализации (мин. нота {lo}, пол {_LEAD_MIN_FLOOR}) "
            "— ниже сдвигать нельзя, тема утонет в басу. Оставь auto/keep "
            "или меньший сдвиг."
        )
    return RtttlMelody(
        bpm=base.bpm,
        notes=tuple((None if m is None else m + shift, d) for m, d in base.notes),
    )


def _apply_lead_outliers(melody: "RtttlMelody", mode: str) -> "RtttlMelody":
    """Выбросы темы по ручке ``lead_outliers``: ``fix`` — подтянуть, ``keep`` — нет."""
    if mode == "keep":
        return melody
    return _fix_isolated_lead_outliers(melody)


def _prep_option_decisions(options: HarmonizeOptions) -> Dict[str, object]:
    """Значения ручек подготовки темы (ADR-0132 PR-3) — для партитуры."""
    return {
        "key_detection": options.key_detection,
        "lead_octave_mode": options.lead_octave,
        "lead_outliers_mode": options.lead_outliers,
    }


def key_fit(
    notes: Sequence[Tuple[Optional[int], float]], root: str, scale: str
) -> float:
    """Доля звучащей длительности темы, лежащая в ладу ``root scale``.

    Мера спора явной тональности с мелодией (ADR-0132 PR-2): у верной
    тональности обычно ≥ 0.9, у параллельной — столько же, у чужой — меньше
    половины. Без звучащих нот — 1.0 (спорить не с чем).
    """
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    tonic = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    pcs = {(tonic + i) % 12 for i in intervals}
    total = sum(float(d) for m, d in notes if m is not None)
    inside = sum(float(d) for m, d in notes if m is not None and int(m) % 12 in pcs)
    return round(inside / total, 3) if total > 0 else 1.0


def _explicit_key_decisions(
    melody: RtttlMelody, detected: KeyCandidate, root: str, scale: str
) -> Dict[str, object]:
    """Запись явной тональности вызова рядом с определённой (ADR-0132 PR-2)."""
    return {
        "key_source": "explicit",
        "key_explicit": (root, scale),
        "key_detected": (detected.root, detected.scale),
        "key_fit": key_fit(melody.notes, root, scale),
    }


#: Сколько кандидатов тональности кроме лучшего показывать в решениях.
_KEY_ALTERNATIVES = 3


def _prep_decisions(
    source_bpm: int,
    steps: Tuple[RtttlMelody, RtttlMelody, RtttlMelody, RtttlMelody, RtttlMelody],
    ranked: Sequence[KeyCandidate],
) -> Dict[str, object]:
    """Запись авто-решений подготовки темы (ADR-0132) — только для партитуры.

    ``steps`` — тема после каждого шага конвейера: свёртка темпа,
    затактовый лид-ин (issue #2960), выравнивание по такту, перенос
    регистра, подтяжка выбросов. Решения считаются сравнением соседних
    шагов, сами шаги не трогаются.
    """
    folded, leadin, snapped, registered, final = steps
    before = [m for m, _ in snapped.notes if m is not None]
    after = [m for m, _ in registered.notes if m is not None]
    moved = sum(
        1 for (a, _da), (b, _db) in zip(registered.notes, final.notes) if a != b
    )
    gap = ranked[0].score - ranked[1].score if len(ranked) > 1 else 0.0
    return {
        "source_bpm": int(source_bpm),
        "bpm": int(final.bpm),
        "anacrusis_pad_beats": round(
            sum(d for _, d in leadin.notes) - sum(d for _, d in folded.notes), 4
        ),
        "tail_pad_beats": round(
            sum(d for _, d in snapped.notes) - sum(d for _, d in leadin.notes), 4
        ),
        "lead_shift": (after[0] - before[0]) if before else 0,
        "outliers_moved": moved,
        "key_ranked": [
            (c.root, c.scale, round(c.score, 3))
            for c in ranked[: 1 + _KEY_ALTERNATIVES]
        ],
        "key_gap": round(gap, 3),
    }


#: Рабочий диапазон темпа аранжировщика (совпадает с ``arranger.BPM_RANGE``).
#: Держим копию, а не импорт, по той же причине, что и SCALE_INTERVALS:
#: модуль остаётся независимым от деталей рендера.
_TEMPO_RANGE = (60.0, 180.0)


def _normalize_tempo(melody: RtttlMelody) -> RtttlMelody:
    """Свернуть темп в рабочий диапазон, ВДВОЕ меняя и bpm, и длительности.

    🔴 FIX (live 14.09): аранжировщик клампит bpm в [60, 180], а в архиве
    1321 мелодия записана быстрее и 547 медленнее — 18% библиотеки. Кламп
    не трогает длительности, поэтому такая мелодия играла в чужом темпе:
    «В пещере горного короля» с ``b=260`` превращалась в 180 и шла на
    треть медленнее, чем задумано.

    Сворачивание вдвое звучит РОВНО так же: половинный темп с половинными
    длительностями даёт то же абсолютное время (``beats/2`` при ``bpm/2``
    — та же секунда), просто «четверть при 260» записывается как «восьмая
    при 130». Это стандартная смена единицы записи, а не изменение музыки.

    Выход из диапазона больше чем вдвое-втрое встречается (до ``b=900``),
    поэтому свёртка идёт циклом; ограничитель шагов защищает от
    вырожденных значений вроде ``b=0``.
    """
    bpm = float(melody.bpm)
    if bpm <= 0:
        return melody
    factor = 1.0
    for _ in range(8):
        if bpm > _TEMPO_RANGE[1]:
            bpm /= 2.0
            factor /= 2.0
        elif bpm < _TEMPO_RANGE[0]:
            bpm *= 2.0
            factor *= 2.0
        else:
            break
    if factor == 1.0:
        return melody
    return RtttlMelody(
        bpm=int(round(bpm)),
        notes=tuple((midi, dur * factor) for midi, dur in melody.notes),
    )


#: Центр рабочего регистра лида — MIDI 78 (между C5=72 и C6=84, см.
#: ``rtttl._to_midi``: ``12*(octave+1)+semitone`` — стандартная MIDI-шкала,
#: где C4=60). Аранжировщик строит вокруг темы бас (``BASS_MIDI_FLOOR=36``,
#: C2) и подклад (``PAD_MIDI_FLOOR=48``, C3, потолок — на 2 полутона ниже
#: САМОЙ НИЗКОЙ ноты темы, см. ``harmonize._pad_ceiling``): если тема стоит
#: в o=7 (медиана ~MIDI 98, как у мусорной ``russiann``, issue #2840), пэд и
#: контрмелодия громоздятся следом за ней туда же, в тот же визг, а не под
#: неё. Транспонирование — единственный рычаг: инструменты аранжировщика
#: (``imperialbrass`` и т.п.) сами по себе диапазон не ограничивают.
_LEAD_TARGET_CENTER = 78.0

#: Жёсткий потолок лида после нормализации (issue #2840, живой прогон:
#: сдвиг по одной медиане пропускал ``terminat`` — median=80 (в рабочем
#: регистре, сдвиг 0), но max=99: несколько высоких проходящих нот тянут
#: потолок за собой, медиана их не видит). Если после сдвига по медиане
#: max всё ещё выше потолка — досдвигаем ещё на октаву вниз, пока не
#: упрёмся в :data:`_LEAD_MIN_FLOOR` (чтобы не утопить и без того низкие
#: темы в подвал баса).
_LEAD_MAX_CEILING = 88
_LEAD_MIN_FLOOR = 55


def _normalize_lead_register(melody: RtttlMelody) -> RtttlMelody:
    """Транспонировать тему ЦЕЛЫМИ октавами в рабочий регистр лида.

    Двухшаговый сдвиг, оба — целыми октавами (не меняет мелодию: интервалы
    между нотами и лад сохраняются один в один, просто переносит её в
    другой регистр):

    1. По медиане высоты нот (без пауз) — к :data:`_LEAD_TARGET_CENTER`
       (~C5–C6). Тема, уже стоящая в рабочем регистре (медиана в пределах
       половины октавы от центра — округление даёт сдвиг 0), им не
       трогается.
    2. По максимуму — если после шага 1 верхняя нота всё ещё выше
       :data:`_LEAD_MAX_CEILING` (медиана не видит одиночных высоких
       проходящих нот, см. ``terminat`` MIDI 71-99 из живого прогона),
       досдвигаем вниз ещё октавами, пока максимум не впишется или
       минимум не упрётся в :data:`_LEAD_MIN_FLOOR`.

    Транспонировать нужно ДО :func:`~core.harmonize.harmonize` — гармонизация
    строит бас/пэд/контрмелодию от фактической высоты нот темы (пэд —
    "на 2 полутона ниже самой низкой ноты темы"), так что применённый после
    неё сдвиг рассинхронизировал бы тему с уже построенным аккомпанементом.
    """
    pitches = sorted(m for m, _dur in melody.notes if m is not None)
    if not pitches:
        return melody
    n = len(pitches)
    mid = n // 2
    if n % 2:
        median = float(pitches[mid])
    else:
        median = (pitches[mid - 1] + pitches[mid]) / 2.0
    shift = int(round((_LEAD_TARGET_CENTER - median) / 12.0)) * 12

    lo, hi = pitches[0], pitches[-1]
    while hi + shift > _LEAD_MAX_CEILING:
        if lo + shift - 12 < _LEAD_MIN_FLOOR:
            break
        shift -= 12

    if shift == 0:
        return melody
    return RtttlMelody(
        bpm=melody.bpm,
        notes=tuple(
            (None if m is None else m + shift, dur) for m, dur in melody.notes
        ),
    )


#: Перцентили (взвешенные длительностью), задающие «корпус темы» для
#: :func:`_fix_isolated_lead_outliers` — 10-й и 90-й, тот же выбор, что и
#: у потолка подклада (``harmonize._PAD_CEILING_PERCENTILE``): достаточно
#: широкий, чтобы не задеть саму тему, и достаточно узкий, чтобы короткий
#: затакт/проходящая нота в него не попали.
_LEAD_OUTLIER_LO_PERCENTILE = 0.10
_LEAD_OUTLIER_HI_PERCENTILE = 0.90

#: Дальше скольких полутонов от края корпуса нота считается «выбросом» и
#: переносится октавой ближе. 12 — сама октава: issue #2876 явно требует
#: убирать именно скачки БОЛЬШЕ октавы, оставляя обычные широкие ходы
#: мелодии (терцдецима, октава с хвостиком в рабочем диапазоне) нетронутыми.
_LEAD_OUTLIER_OCTAVE_SPAN = 12


def _fix_isolated_lead_outliers(melody: RtttlMelody) -> RtttlMelody:
    """Затакт/одиночную ноту дальше октавы от корпуса темы — подтянуть к нему.

    🔴 FIX (issue #2876, живой прогон 23.09.2026, «диджей Снупдог» — Still
    Dre): :func:`_normalize_lead_register` переносит ВСЮ тему октавами как
    один блок — она не может починить одну ноту внутри уже нормальной
    темы. У Still Dre затакт перед каждой фразой (4 раза, четверть длины
    соседних нот) стоял на MIDI 72, тело фразы — на 87-89; после общего
    сдвига в рабочий регистр это 60 против 75-77 — скачок в 15-17
    полутонов на КАЖДОМ повторе затакта, и потолок подклада
    (``harmonize._pad_ceiling``) до FIX #2876 в этом модуле садился на
    затакт же, утаскивая подклад в бас.

    «Корпус темы» — диапазон между 10-м и 90-м перцентилем высоты нот,
    взвешенным длительностью (:func:`~core.harmonize._weighted_percentile`,
    тот же приём, что у потолка подклада): короткий затакт не может
    сдвинуть перцентиль, его вес тонет в весе длинных нот тела фразы.

    Нота дальше :data:`_LEAD_OUTLIER_OCTAVE_SPAN` полутонов от ближайшего
    края корпуса переносится ЦЕЛЫМИ октавами навстречу корпусу — ровно
    «затакт переносится октавой к теме» из акцептанса, но универсально
    (не только затакты, любая одиночная нота вне корпуса), и ровно
    настолько, чтобы выйти из-под четвертьоктавного разрыва, не залезая
    внутрь корпуса дальше необходимого.

    Применяется ПОСЛЕ :func:`_normalize_lead_register` (весь блок уже в
    рабочем регистре) и ДО :func:`~core.harmonize.harmonize` — по той же
    причине: гармонизация строит бас/пэд от фактической высоты нот темы.
    """
    pairs = [(m, dur) for m, dur in melody.notes if m is not None]
    if len(pairs) < 2:
        return melody
    core_lo = int(round(_weighted_percentile(pairs, _LEAD_OUTLIER_LO_PERCENTILE)))
    core_hi = int(round(_weighted_percentile(pairs, _LEAD_OUTLIER_HI_PERCENTILE)))

    changed = False
    out: List[Tuple[Optional[int], float]] = []
    for m, dur in melody.notes:
        if m is None:
            out.append((m, dur))
            continue
        note = m
        while note < core_lo - _LEAD_OUTLIER_OCTAVE_SPAN:
            note += 12
        while note > core_hi + _LEAD_OUTLIER_OCTAVE_SPAN:
            note -= 12
        if note != m:
            changed = True
        out.append((note, dur))

    if not changed:
        return melody
    return RtttlMelody(bpm=melody.bpm, notes=tuple(out))


def _anacrusis_lead_in(melody: RtttlMelody) -> RtttlMelody:
    """Затакт (пикап) — добавить паузу в начало, чтобы сильная доля темы
    совпала с долей 0 такта аккомпанемента (issue #2960).

    🔴 FIX (live 24.09, гимн России ``national_2``: ``8g,8p,c6,8g.,...``):
    тема начинается с короткой затактовой ноты (``G``, восьмая), за ней —
    первая ОПОРНАЯ, сильная нота (``C6``, четверть). Аранжировщик ставит
    ПЕРВУЮ ноту темы на долю 0 (см. :func:`~core.harmonize.harmonize`,
    :func:`_timed`) — поэтому опорная нота гимна («си-») оказывалась на
    доле 1, а не на сильной доле такта, и била мимо каркаса ударных
    (бочка ``X`` на 0/4, малый ``o`` на 2/4) и смен аккорда пэда (тоже по
    долям такта): на слух — «ноты промахиваются».

    Детектор — тот же критерий, что уже используется для тональности
    (:func:`_first_strong_note`): если первая звучащая нота темы КОРОЧЕ
    следующей звучащей ноты, она — затакт, а следующая — первая сильная
    нота фразы. Общий признак затакта (пикап слабее и короче опорной
    ноты), без привязки к конкретной песне (ADR-0132).

    Величина паузы — ровно столько долей, чтобы онсет первой сильной ноты
    (считая паузы между затактом и ней) стал кратен такту
    (:data:`~core.arranger.BEATS_PER_BAR`): затакт оказывается в хвосте
    нового вступительного такта (доигрывает его последними долями — «или
    в intro», см. issue), а первая сильная нота начинает СЛЕДУЮЩИЙ такт
    ровно с доли 0, синхронно с ударными и первой сменой аккорда пэда.

    Тема без затакта (первая звучащая нота не короче второй, как в
    большинстве RTTTL-рингтонов) не трогается — байт-в-байт прежнее
    поведение (``test_arranger_golden``).
    """
    notes = melody.notes
    sounding_idx = [i for i, (m, _d) in enumerate(notes) if m is not None]
    if len(sounding_idx) < 2:
        return melody
    first_i, second_i = sounding_idx[0], sounding_idx[1]
    first_dur = notes[first_i][1]
    second_dur = notes[second_i][1]
    if first_dur >= second_dur:
        return melody
    onset = sum(d for _, d in notes[:second_i])
    remainder = onset % BEATS_PER_BAR
    if remainder == 0:
        return melody
    pad = BEATS_PER_BAR - remainder
    return RtttlMelody(bpm=melody.bpm, notes=((None, pad),) + tuple(notes))


def _snap_to_bar(melody: RtttlMelody) -> RtttlMelody:
    """Довести длину мелодии до целого числа тактов хвостовой паузой.

    RTTTL-мелодии — рингтоны с «дыхательными» паузами (``32p``), из-за
    которых суммарная длина не кратна такту. Без выравнивания луп каждый
    повтор смещается на дробный остаток и уезжает от ударной сетки.
    """
    total = sum(d for _, d in melody.notes)
    remainder = total % BEATS_PER_BAR
    if remainder == 0:
        return melody
    pad = BEATS_PER_BAR - remainder
    return RtttlMelody(
        bpm=melody.bpm,
        notes=tuple(melody.notes) + ((None, pad),),
    )
