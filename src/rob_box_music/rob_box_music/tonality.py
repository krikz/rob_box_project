"""Тональность мелодии: профиль Крумхансл + тонический центр, ``key_fit`` (ADR-0149 §3.3, §8.1).

Перенесено из ``rob_box_mcp_tools/core/rtttl_compose.py`` без изменений логики (ADR-0149 PR-3):
одна реализация, старый модуль импортирует отсюда. Эталон — ``test_rtttl_compose`` (36 тем, ``_KEY_REFERENCE``).
"""

from __future__ import annotations

from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from . import knowledge as kn

#: Значение ручки ``key_detection`` по умолчанию (``core.harmonize.AUTO``).
AUTO = "auto"

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
    in_scale = {i % 12 for i in kn.SCALES[scale]}
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
            kn.ROOTS[root],
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


def key_fit(
    notes: Sequence[Tuple[Optional[int], float]], root: str, scale: str
) -> float:
    """Доля звучащей длительности темы, лежащая в ладу ``root scale``.

    Мера спора явной тональности с мелодией (ADR-0132 PR-2): у верной
    тональности обычно ≥ 0.9, у параллельной — столько же, у чужой — меньше
    половины. Без звучащих нот — 1.0 (спорить не с чем).
    """
    intervals = kn.SCALES.get(scale, kn.SCALES["minor"])
    tonic = kn.ROOTS.index(root) if root in kn.ROOTS else 0
    pcs = {(tonic + i) % 12 for i in intervals}
    total = sum(float(d) for m, d in notes if m is not None)
    inside = sum(float(d) for m, d in notes if m is not None and int(m) % 12 in pcs)
    return round(inside / total, 3) if total > 0 else 1.0


__all__ = ["AUTO", "KeyCandidate", "detect_key", "detect_key_ranked", "key_fit"]
