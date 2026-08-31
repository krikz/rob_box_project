"""Аранжировщик: превращает компактную спецификацию трека в Renardo-код с формой.

Зачем (docs/analysis/2026-08-30-music-quality-audit.md, RC4)
===========================================================

Renardo-плеер — бесконечный луп. «Композиция» в нём = расписание изменений
во времени. До этого модуля расписания не было ни в промпте, ни в коде:
LLM писала ``p1 >> blip([0,2,4,7], dur=0.5)`` и это играло одинаково до
самой остановки. Отсюда жалоба «простая мелодия, которая повторяется всё
время» — она буквально повторяется, потому что форму никто не задаёт.

Разделение ответственности здесь такое:

* **LLM отдаёт материал** — тональность, темп, палитра синтов, 2-4 мотива.
  Это то, что языковая модель делает хорошо.
* **Этот модуль отдаёт форму** — когда слой входит, когда уходит, где брейк,
  где кульминация. Это то, что детерминированный код делает надёжно, а LLM
  на длинной дистанции держать не может.

Как форма кодируется
====================

Не через ``Clock.future`` и не через ``def intro()/verse()`` — а декларативно,
через ``var()``-огибающие на ``amp`` каждого слоя. Это идиоматичный для
FoxDot/Renardo приём (в пресете `rypop_120bpm` из миграции 006 живой
человек пишет ровно так: ``vv.amp = var([.5,0],[48,16])``), и у него три
важных свойства:

1. Точность по тактам — таймингом занимается Clock, а не Python-поток.
2. Автоматическое зацикливание — ``var`` проходит форму и начинает заново,
   поэтому DJ-режиму не нужен отдельный планировщик.
3. Проходит все существующие фильтры: нет ``def``, нет ``lambda``,
   нет ``Clock.future`` — то есть нечего вырезать и не на чем падать.

Единицы: длительности ``var`` считаются в БИТАХ (``TimeVar.__init__``:
``dur = metro.bar_length()`` = 4). Поэтому такты умножаются на
:data:`BEATS_PER_BAR`.

Регистры (RC2): октава назначается по РОЛИ слоя, а не по вкусу модели.
Бас, пэд, лид и колокольчики физически разведены по регистрам — без этого
микс слышится как одна дорожка независимо от того, сколько в нём слоёв.

Материал по секциям, не только громкость (RC5, issues #1805/#1806)
====================================================================

RC4 дал форме огибающую amp, но мелодия внутри неё была одной и той же от
начала до конца — «человек в припеве играет другую фразу, а не ту же самую
громче» (#1805). RC5 добавляет ещё две декларативные огибающие поверх той
же идиомы ``var()``/``Pvar()``, ничего не выдумывая — только трансформируя
материал, который уже дала LLM:

* :func:`_motif_variants` — ступени лада по секциям через ``Pvar``.
  Транспозиция/инверсия/ретроград мотива выбираются по тому, как меняется
  собственная интенсивность роли в этой секции (см. :data:`FORMS`), а не
  по имени секции и не случайно — форма уже это знает, спрашивать LLM не
  нужно.

  RC5.1 (правка после ревью): транспозиция и инверсия — приёмы для ТЕМЫ,
  а не для баса и не для аккордовой подкладки. Открытая раскладка пэда
  (``0, 4, 7, 11`` — уже больше октавы) от инверсии/транспозиции уезжала
  в подвал и в верха одновременно и рвала регистровое разведение RC2.
  Поэтому: транспозиция/инверсия — только у ``lead``; ``bass`` может
  только переставлять свои же ноты (ретроград), никогда не выходя из
  своей октавы; ``pad`` держит гармонию и material не меняет вовсе
  (развитие пэда — это :func:`_dur_var` и огибающая amp, неизменный
  аккорд под движущейся мелодией — это голосоведение, а не тот луп, на
  который жаловался #1805). Дополнительная страховка —
  :func:`_fold_into_range`: любой вариант складывается обратно в
  собственный диапазон ступеней мотива, так что роль физически не может
  вылезти за пределы регистра, который сама же и получила от LLM.
* :func:`_dur_var` — плотность нот по секциям через ``var()`` на ``dur``:
  громче роль в секции — гуще ноты, тише — ноты растянуты. Это отдельный
  диагноз (#1806): «постоянная длительность ноты = механическая сетка»
  верно даже там, где мелодия уже варьируется.

``Clock.swing()`` (issue #1806) — отдельная строка в начале кода, не
огибающая: смещает нечётные восьмые через ``nudge``
(``renardo_lib/TempoClock.py:290``) для жанров, где ровная сетка физически
не звучит как жанр (джаз, блюз, шафл). Материал по определению
(``CompositionSpec.swing``), не форма — арранжировщик его только выводит.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

#: 1 такт = 4 бита в дефолтном метре Renardo (``TempoClock.bar_length()``).
BEATS_PER_BAR = 4

#: Роль -> (имя плеера, октава, базовая амплитуда).
#:
#: Плееры удержаны в d1-d3 / p1-p3 — это ограничение деплоя (d4+/p4+ на
#: роботе не звучат, см. music_skill_prompt.txt).
#:
#: Октавы разведены намеренно: бас 3, пэд 4, лид 5, колокольчики 6. Раньше
#: ``_cap_amp`` зажимал всё в oct<=4 и бас с лидом оказывались в соседних
#: октавах.
#:
#: Амплитуды — ОТНОСИТЕЛЬНЫЙ баланс ролей, а не абсолютная громкость.
#: Сумму держит synthdef ``masterlimiter`` в scsynth, абсолютный уровень —
#: ROS-параметр ``music_master_gain``.
ROLE_PROFILE: Dict[str, Tuple[str, int, float]] = {
    "drums": ("d1", 0, 0.55),
    "hats":  ("d2", 0, 0.24),
    "perc":  ("d3", 0, 0.30),
    "bass":  ("p1", 3, 0.50),
    "lead":  ("p2", 5, 0.52),
    "pad":   ("p3", 4, 0.30),
}

#: Роли, которые играют сэмплами через ``play(...)``, а не синтом.
DRUM_ROLES = frozenset({"drums", "hats", "perc"})

#: Формы: имя -> список секций ``(имя, тактов, {роль: интенсивность 0..1})``.
#:
#: Интенсивность умножается на базовую амплитуду роли. 0.0 = слой молчит
#: (нота всё равно шлётся, но с amp=0 — это дешевле, чем стоп/старт плеера,
#: и не даёт щелчка на входе).
#:
#: Формы намеренно НЕ симметричны: у секций разная длина, а кульминация
#: приходит после брейка. Симметричная сетка 16/16/16/16 на слух — то же
#: самое, что луп.
FORMS: Dict[str, List[Tuple[str, int, Dict[str, float]]]] = {
    # Универсальная дуга. Дефолт: работает и для DJ-петли, и для трека.
    "arc": [
        ("intro",  8,  {"pad": 0.65, "hats": 0.30}),
        ("build",  8,  {"pad": 0.70, "hats": 0.55, "drums": 0.50, "bass": 0.70}),
        ("main",  16,  {"pad": 0.45, "hats": 0.80, "drums": 1.00, "bass": 1.00,
                        "lead": 0.95, "perc": 0.60}),
        ("break",  8,  {"pad": 0.90, "hats": 0.25, "bass": 0.40, "lead": 0.55}),
        ("peak",  16,  {"pad": 0.55, "hats": 1.00, "drums": 1.00, "bass": 1.00,
                        "lead": 1.00, "perc": 0.85}),
        ("outro",  8,  {"pad": 0.50, "hats": 0.20, "drums": 0.35, "bass": 0.30}),
    ],
    # Песенная форма: куплет тише припева, бридж снимает барабаны.
    "verse_chorus": [
        ("intro",   8,  {"pad": 0.60, "hats": 0.35}),
        ("verse",  16,  {"pad": 0.50, "hats": 0.60, "drums": 0.70, "bass": 0.80,
                         "lead": 0.55}),
        ("chorus", 16,  {"pad": 0.60, "hats": 0.90, "drums": 1.00, "bass": 1.00,
                         "lead": 1.00, "perc": 0.70}),
        ("bridge",  8,  {"pad": 0.85, "hats": 0.20, "bass": 0.45, "lead": 0.60}),
        ("chorus2", 16, {"pad": 0.65, "hats": 1.00, "drums": 1.00, "bass": 1.00,
                         "lead": 1.00, "perc": 0.90}),
        ("outro",   8,  {"pad": 0.55, "hats": 0.25, "bass": 0.35}),
    ],
    # Клубная: длинный разгон, тишина перед дропом, дроп на полную.
    "buildup": [
        ("intro",   8,  {"pad": 0.55, "hats": 0.40, "drums": 0.35}),
        ("build",  16,  {"pad": 0.70, "hats": 0.75, "drums": 0.70, "bass": 0.75,
                         "lead": 0.40}),
        ("gap",     4,  {"pad": 0.80}),
        ("drop",   16,  {"hats": 1.00, "drums": 1.00, "bass": 1.00, "lead": 1.00,
                         "perc": 0.90, "pad": 0.35}),
        ("break",   8,  {"pad": 0.85, "bass": 0.45, "lead": 0.50}),
        ("drop2",  16,  {"hats": 1.00, "drums": 1.00, "bass": 1.00, "lead": 0.95,
                         "perc": 1.00, "pad": 0.40}),
        ("outro",   8,  {"pad": 0.60, "hats": 0.20}),
    ],
    # Без ударной сетки: медленные наплывы, для «сделай что-то для души».
    #
    # Live 30.08: первая версия открывалась 16 тактами ОДНОГО пэда. На 80 BPM
    # это 48 секунд почти неподвижного звука — слушатель успевает решить, что
    # робот сломался. Плюс сетка 16/16/16/16 симметрична, а симметричная
    # сетка на слух — тот же луп, только длиннее. Теперь мелодия входит
    # сразу, вполголоса, а секции разной длины.
    "ambient": [
        ("emerge",  8,  {"pad": 0.60, "lead": 0.30}),
        ("drift",  16,  {"pad": 0.85, "lead": 0.55}),
        ("swell",  20,  {"pad": 1.00, "lead": 0.75, "bass": 0.50}),
        ("recede", 12,  {"pad": 0.70, "lead": 0.35}),
    ],
}

#: Дефолтная форма, когда LLM не указала или указала неизвестную.
DEFAULT_FORM = "arc"

#: Диапазон BPM, за который выходить бессмысленно.
BPM_RANGE = (60.0, 180.0)

VALID_ROOTS = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")


class ArrangementError(ValueError):
    """Спецификация не может быть развёрнута в корректный Renardo-код."""


@dataclass
class Layer:
    """Один слой аранжировки — материал без формы.

    Attributes:
        role: ключ из :data:`ROLE_PROFILE`.
        synth: имя синта для мелодических ролей (``dub``, ``blip``, ...).
        pattern: строка паттерна для ударных ролей (``"X..o.X.o"``).
        degrees: ступени лада для мелодических ролей.
        dur: длительность ноты в битах.
        sample: индекс сэмпла для ударных.
        oct_shift: сдвиг относительно октавы роли — на случай, когда бас
            должен уйти ещё ниже или лид ещё выше.
    """

    role: str
    synth: Optional[str] = None
    pattern: Optional[str] = None
    degrees: Sequence[float] = field(default_factory=tuple)
    dur: float = 1.0
    sample: int = 0
    oct_shift: int = 0


@dataclass
class CompositionSpec:
    """Полная спецификация трека — то, что отдаёт LLM.

    Всё, что связано со временем и балансом, сюда НЕ входит: это забота
    аранжировщика.
    """

    bpm: float = 120.0
    root: str = "C"
    scale: str = "minor"
    form: str = DEFAULT_FORM
    layers: Sequence[Layer] = field(default_factory=tuple)
    #: Движение тоники по ступеням лада, например ``[0, 0, 5, 3]``. Один
    #: цикл прогрессии растягивается ровно на одну форму.
    progression: Sequence[int] = field(default_factory=tuple)
    #: Медленный фильтр-свип на всю форму. Даёт «дыхание» даже там, где
    #: слои статичны.
    filter_sweep: bool = True
    #: False -> в конце формы музыка останавливается сама. True (DJ-режим)
    #: -> форма зацикливается бесконечно.
    repeat: bool = True
    #: Свинг восьмых, 0..0.3 (issue #1806). 0 = ровная сетка — дефолт для
    #: большинства жанров. Ненулевой свинг нужен там, где ровные восьмые
    #: физически не звучат как жанр (джаз, блюз, шафл, фанк) — ``Clock.
    #: swing()`` смещает нечётные восьмые через ``nudge`` и не требует
    #: правки нот или длительностей. Это материал (ощущение времени), а не
    #: форма, поэтому поле, а не встроенная логика формы.
    swing: float = 0.0


def _fmt(value: float) -> str:
    """Компактно отформатировать число для Renardo-кода.

    ``1.0 -> "1"``, ``0.25 -> "0.25"``, ``0.5000000001 -> "0.5"``.
    Renardo-код читают люди (он попадает в логи и в save_track), поэтому
    ``0.30000000000000004`` там неуместен.
    """
    rounded = round(float(value), 4)
    if rounded == int(rounded):
        return str(int(rounded))
    return f"{rounded:g}"


def _fmt_list(values: Sequence[float]) -> str:
    return "[" + ", ".join(_fmt(v) for v in values) + "]"


def _fmt_nested_list(values: Sequence[Sequence[float]]) -> str:
    """Format a list of degree-lists for ``Pvar([[...], [...]], ...)``."""
    return "[" + ", ".join(_fmt_list(v) for v in values) + "]"


def _merge_adjacent(values: Sequence, durations: Sequence[int]) -> Tuple[List, List[int]]:
    """Collapse consecutive equal values, summing their durations.

    Shared by every per-section ``var()``/``Pvar()`` timeline this module
    builds (amplitude, motif, dur): a run of identical values renders as one
    entry, not one per section — ``var([0.5, 0.5], [32, 32])`` is a spurious
    switch and unreadable code (see :func:`_amp_envelope`, the original of
    this pattern).
    """
    merged_values: List = []
    merged_durations: List[int] = []
    for value, beats in zip(values, durations):
        if merged_values and merged_values[-1] == value:
            merged_durations[-1] += beats
        else:
            merged_values.append(value)
            merged_durations.append(beats)
    return merged_values, merged_durations


def resolve_form(name: Optional[str]) -> List[Tuple[str, int, Dict[str, float]]]:
    """Вернуть план формы, молча падая на дефолт для неизвестного имени.

    Неизвестная форма — не повод отказать в музыке: LLM регулярно
    выдумывает названия, и лучше сыграть дугу, чем вернуть ошибку.
    """
    return FORMS.get((name or "").strip().lower(), FORMS[DEFAULT_FORM])


def _amp_envelope(
    role: str,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    base_amp: float,
) -> Tuple[List[float], List[int]]:
    """Собрать (значения amp, длительности в битах) для одной роли.

    Соседние секции с одинаковой интенсивностью склеиваются — иначе
    ``var([0.5, 0.5, 0.5], [32, 32, 64])`` порождает лишние переключения
    и делает код нечитаемым.
    """
    amps: List[float] = []
    durs: List[int] = []
    for _name, bars, intensities in plan:
        amp = round(base_amp * float(intensities.get(role, 0.0)), 4)
        amps.append(amp)
        durs.append(int(bars) * BEATS_PER_BAR)
    return _merge_adjacent(amps, durs)


#: Scale degrees per octave, used only to fold a transformed motif back
#: into its own register (see :func:`_fold_into_range`). Renardo's degree
#: indexing wraps at the scale length — degree N and N+len(scale) are the
#: same pitch class an octave apart — and every scale this module exposes
#: (minor, major, dorian, mixolydian, lydian, phrygian, harmonicMinor) has
#: 7 notes. Pentatonic scales fold slightly loosely (their true octave is
#: 5 degrees), but 7 still keeps a folded note from crossing into a
#: neighbouring role's register, which is the only property this constant
#: needs to guarantee.
OCTAVE_STEP = 7


def _transpose(degrees: Sequence[float], steps: float) -> Tuple[float, ...]:
    return tuple(d + steps for d in degrees)


def _invert(degrees: Sequence[float]) -> Tuple[float, ...]:
    """Mirror the motif around its own first note.

    Standard melodic inversion: every interval that went up now goes down
    by the same amount. Stays diatonic automatically — these are scale
    degrees, not semitones, so there is nothing to clash.
    """
    axis = degrees[0]
    return tuple(2 * axis - d for d in degrees)


def _retrograde(degrees: Sequence[float]) -> Tuple[float, ...]:
    return tuple(reversed(degrees))


def _fold_into_range(degrees: Sequence[float], lo: float, hi: float) -> Tuple[float, ...]:
    """Bring every note back inside ``[lo, hi]`` by whole octaves.

    RC2 physically separates bass/pad/lead into adjacent octaves — a
    transform that pushes a note outside the motif's OWN span can climb
    into the role above or fall into the role below, which is exactly the
    register bleed the RC2 separation exists to prevent. Folding by
    :data:`OCTAVE_STEP` keeps the note's identity in the scale (unlike
    clipping, which would flatten the contour into repeated boundary
    notes) while guaranteeing it never leaves the register the LLM's own
    motif already established.
    """
    folded: List[float] = []
    for d in degrees:
        while d > hi:
            d -= OCTAVE_STEP
        while d < lo:
            d += OCTAVE_STEP
        folded.append(d)
    return tuple(folded)


def _motif_variants(
    role: str,
    degrees: Sequence[float],
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
) -> Tuple[List[Tuple[float, ...]], List[int]]:
    """Build (motif variant, beats) per section — the material half of the
    form that #1805 found missing: sections used to change only ``amp``, so
    a melody played the same seven notes for the whole piece, only louder
    or quieter.

    Nothing here is invented — every variant is a transform of the motif
    the LLM already supplied (see :func:`_autofill_bass` for the same
    principle applied to harmony).

    Register review (#1805 follow-up, RC5.1): transposition and inversion
    are devices for a THEME, not for a chord pad or a bassline — a wide
    open-voiced pad chord like ``(0, 4, 7, 11)`` already spans more than an
    octave, so inverting or transposing it reliably breaks RC2's register
    separation (the pad climbs above the lead, or the bass drops below the
    masterbus HPF). Only ``lead`` gets pitch-shifting development:

    * First entrance -> the motif verbatim (state the idea before varying
      it).
    * Intensity keeps climbing -> transpose up a third (a real sequence —
      the standard "raise it a step" build device).
    * Intensity drops -> invert around the first note (a mirrored phrase —
      "a different idea", the exact contrast #1805 asked for at a break or
      chorus, not the same one quieter).
    * Intensity holds -> retrograde (played backwards — recognisably the
      same idea without being a literal repeat).

    ``bass`` may still develop, but only by reordering its own tones
    (retrograde) — never transposed or inverted, so it can never leave its
    own octave. ``pad`` holds the harmony: its material stays constant
    (development there comes from :func:`_dur_var` and the amp envelope
    alone) — an unchanging chord under a moving lead is voice-leading, not
    the static loop #1805 complained about.

    As a second line of defence (in case a future transform is added
    here), every non-original variant is folded back into the original
    motif's own ``[min(degrees), max(degrees)]`` span via
    :func:`_fold_into_range` before it is used — a role can never end up
    outside the register footprint the LLM itself chose.
    """
    original = tuple(degrees)
    if role == "pad":
        total_beats = sum(int(bars) for _n, bars, _i in plan) * BEATS_PER_BAR
        return [original], [total_beats]

    lo, hi = min(original), max(original)
    values: List[Tuple[float, ...]] = []
    durations: List[int] = []
    prev_intensity = 0.0
    prev_audible = False
    for _name, bars, intensities in plan:
        intensity = float(intensities.get(role, 0.0))
        if intensity <= 0.0 or not prev_audible:
            variant = original
        elif role != "lead":
            # bass: reorder tones only — never transpose/invert out of
            # register (see docstring above).
            variant = _fold_into_range(_retrograde(original), lo, hi)
        else:
            delta = intensity - prev_intensity
            if abs(delta) < 1e-6:
                variant = _fold_into_range(_retrograde(original), lo, hi)
            elif delta > 0:
                variant = _fold_into_range(_transpose(original, 2), lo, hi)
            else:
                variant = _fold_into_range(_invert(original), lo, hi)
        values.append(variant)
        durations.append(int(bars) * BEATS_PER_BAR)
        prev_intensity = intensity
        prev_audible = intensity > 0.0
    return _merge_adjacent(values, durations)


def _dur_var(
    role: str,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    base_dur: float,
) -> Tuple[List[float], List[int]]:
    """Tie note density to the section's own intensity (issue #1806).

    A single ``dur`` for the whole piece is the "механическая сетка"
    complaint by itself, independent of swing: every layer plays at exactly
    the same rate for three minutes regardless of genre or instrument.
    Density already has a driver sitting in the form data the arranger
    already owns — the busier a section is (main/peak have drums+bass+lead
    all near 1.0), the busier the notes should feel; a quiet break can
    afford to stretch out. Halves/doubles ``base_dur`` relative to the
    role's own loudest section, so the effect is proportionate whether the
    base is a fast lead (``dur=0.5``) or a slow pad (``dur=4``).
    """
    role_intensities = [float(i.get(role, 0.0)) for _n, _b, i in plan]
    peak = max(role_intensities) if role_intensities else 0.0
    values: List[float] = []
    durations: List[int] = []
    for (_name, bars, _intensities), intensity in zip(plan, role_intensities):
        if intensity <= 0.0 or peak <= 0.0:
            dur = base_dur
        else:
            ratio = intensity / peak
            if ratio >= 0.85:
                dur = base_dur / 2
            elif ratio <= 0.5:
                dur = base_dur * 2
            else:
                dur = base_dur
        values.append(round(dur, 4))
        durations.append(int(bars) * BEATS_PER_BAR)
    return _merge_adjacent(values, durations)


def _render_layer(
    layer: Layer,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    use_filter: bool,
) -> Optional[str]:
    """Отрендерить одну строку Renardo-кода, либо None если слой молчит."""
    profile = ROLE_PROFILE.get(layer.role)
    if profile is None:
        raise ArrangementError(
            f"Неизвестная роль слоя: {layer.role!r}. "
            f"Доступны: {', '.join(sorted(ROLE_PROFILE))}."
        )
    player, role_oct, base_amp = profile

    amps, durs = _amp_envelope(layer.role, plan, base_amp)
    if not any(amps):
        # Роль не участвует ни в одной секции этой формы (например drums в
        # ambient) — плеер не создаём вовсе, чтобы не гонять тихие ноты.
        return None

    if len(amps) == 1:
        amp_expr = _fmt(amps[0])
    else:
        amp_expr = f"var({_fmt_list(amps)}, {_fmt_list(durs)})"

    pre_lines: List[str] = []
    args: List[str] = []
    if layer.role in DRUM_ROLES:
        if not layer.pattern:
            raise ArrangementError(
                f"Роль {layer.role!r} играет сэмплами — нужен pattern, "
                'например "X..o.X.o".'
            )
        head = f'play({layer.pattern!r}'
        if layer.sample:
            args.append(f"sample={int(layer.sample)}")
    else:
        if not layer.synth:
            raise ArrangementError(
                f"Роль {layer.role!r} играет синтом — нужно поле synth."
            )
        if not layer.degrees:
            raise ArrangementError(
                f"Роль {layer.role!r} без degrees — играть нечего."
            )

        # #1805 — материал по секциям, не только громкость. Пишем как
        # отдельную переменную (тот же идиом, что и ``gflt = linvar(...)``
        # ниже), а не инлайном: инлайновый ``Pvar(...)`` внутри аргументов
        # плеера ломает regex-парсер валидатора качества (ищет ``dur=`` до
        # первой закрывающей скобки — см. tools/music.py::_PLAYER_LINE_RE).
        variants, variant_durs = _motif_variants(layer.role, layer.degrees, plan)
        if len(variants) > 1:
            motif_name = f"{player}_motif"
            pre_lines.append(
                f"{motif_name} = Pvar({_fmt_nested_list(variants)}, "
                f"{_fmt_list(variant_durs)})"
            )
            head = f"{layer.synth}({motif_name}"
        else:
            head = f"{layer.synth}({_fmt_list(layer.degrees)}"

        # #1806 — плотность нот по секциям, не constant dur всю форму.
        dur_values, dur_durs = _dur_var(layer.role, plan, layer.dur)
        if len(dur_values) > 1:
            args.append(f"dur=var({_fmt_list(dur_values)}, {_fmt_list(dur_durs)})")
        else:
            args.append(f"dur={_fmt(layer.dur)}")
        args.append(f"oct={max(2, min(7, role_oct + int(layer.oct_shift)))}")

    args.append(f"amp={amp_expr}")
    # Фильтр-свип вешаем на держащие слои. На ударные не вешаем: срезанная
    # атака бочки слышна как проваленный грув.
    if use_filter and layer.role in ("bass", "pad", "lead"):
        args.append("lpf=gflt")

    line = f"{player} >> {head}, " + ", ".join(args) + ")"
    return "\n".join(pre_lines + [line]) if pre_lines else line


def render(spec: CompositionSpec) -> str:
    """Развернуть спецификацию в Renardo-код с формой.

    Returns:
        Многострочный Renardo-код, готовый для ``execute_music_code``.

    Raises:
        ArrangementError: спецификация внутренне противоречива (неизвестная
            роль, ударный слой без паттерна, мелодический без ступеней).
    """
    if not spec.layers:
        raise ArrangementError("Спецификация без слоёв — играть нечего.")

    bpm = max(BPM_RANGE[0], min(BPM_RANGE[1], float(spec.bpm)))
    root = spec.root if spec.root in VALID_ROOTS else "C"
    plan = resolve_form(spec.form)
    total_beats = sum(int(bars) for _n, bars, _i in plan) * BEATS_PER_BAR

    lines: List[str] = ["Clock.clear()", f"Clock.bpm = {_fmt(bpm)}"]

    if spec.swing > 0:
        # #1806 — ровные восьмые не читаются как джаз/блюз/шафл ни при
        # каком выборе синтов. ``Clock.swing`` смещает нечётные восьмые
        # через ``nudge`` (renardo_lib/TempoClock.py:290) — одна строка на
        # всю форму, ноты и dur трогать не нужно.
        swing = max(0.0, min(0.3, float(spec.swing)))
        lines.append(f"Clock.swing({_fmt(swing)})")

    if spec.progression:
        # Один проход прогрессии растягивается ровно на одну форму, чтобы
        # гармония и аранжировка не разъезжались.
        step = max(1, total_beats // len(spec.progression))
        lines.append(
            f"Root.default = var({_fmt_list(spec.progression)}, {_fmt(step)})"
        )
    else:
        lines.append(f'Root.default = "{root}"')

    lines.append(f'Scale.default = "{spec.scale}"')

    if spec.filter_sweep:
        # Свип длиной в половину формы: за один проход формы фильтр
        # успевает открыться и закрыться.
        lines.append(f"gflt = linvar([700, 4500], {_fmt(total_beats // 2)})")

    for layer in spec.layers:
        rendered = _render_layer(layer, plan, use_filter=spec.filter_sweep)
        if rendered is not None:
            lines.append(rendered)

    if len(lines) <= 4:
        raise ArrangementError(
            "Ни один слой не звучит в выбранной форме — проверь роли "
            f"(форма {spec.form!r} задействует: "
            f"{', '.join(sorted({r for _n, _b, i in plan for r in i}))})."
        )

    if not spec.repeat:
        # Функция, а не lambda: lambda режется AST-фильтром, а Clock.clear
        # передаётся как объект и вызывается планировщиком.
        lines.append(f"Clock.future({total_beats}, Clock.clear)")

    return "\n".join(lines)


#: Длительность ноты по умолчанию для мелодических ролей (в битах).
#:
#: Задаётся здесь, а не запрашивается у LLM: выбор dur — это вопрос
#: плотности фактуры, где модель систематически ошибается в сторону
#: слишком коротких значений (лог: dur 2 -> 1 -> 0.5 -> 0.25 -> 0.125).
#: Роль знает свою плотность лучше.
ROLE_DEFAULT_DUR: Dict[str, float] = {"bass": 1.0, "lead": 0.5, "pad": 4.0}


def parse_notes(raw: Optional[str]) -> Tuple[float, ...]:
    """Разобрать ступени лада из строки вида ``"0, 2, 4, 7"``.

    Принимает и ``"[0,2,4]"``, и ``"0 2 4"`` — маленькие модели пишут
    и так, и так, а отклонять запрос из-за скобок бессмысленно.

    Returns:
        Кортеж ступеней; пустой, если строка пустая.

    Raises:
        ArrangementError: в строке есть что-то, что не число.
    """
    if not raw or not raw.strip():
        return ()
    cleaned = raw.strip().strip("[]()").replace(";", ",").replace(" ", ",")
    out: List[float] = []
    for chunk in cleaned.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            value = float(chunk)
        except ValueError as exc:
            raise ArrangementError(
                f"Ступени лада должны быть числами, получено {chunk!r} "
                f"в {raw!r}. Пример: \"0, 2, 4, 7\"."
            ) from exc
        out.append(int(value) if value == int(value) else value)
    return tuple(out)


# 🔴 FIX (issue #1803): рисунок play(...), чья длина не делит такт, плывёт
# относительно соседних слоёв на каждом повторе. Живые прогоны 30-31.08,
# четыре трека подряд — модель писала "X..o.X.o." (9 шагов) рядом с
# "....o..." (8 шагов): 9 не кратно 8, и уже со второго такта рисунки
# расходятся по фазе. Модель символы не считает и считать не научится —
# длина приводится к ближайшей СВЕРХУ степени двойки (4, 8, 16, 32, ...).
# Округление именно вверх, а не вниз: степень двойки всегда кратна всем
# меньшим степеням двойки, поэтому паттерн остаётся в фазе с любым другим
# слоем той же природы, а округление вниз обрезало бы последний удар.
#
# 🔴 FIX (ревью после первого прохода): добивка ставилась символом "-".
# Это НЕ пауза в FoxDot/Renardo — "-" маппится на реальный сэмпл ("hyphen",
# renardo_gatherer/collections.py) и физически лежит в каждом сэмпл-паке
# (samples/0_foxdot_default/_/hyphen), т.е. это звучащий хэт. Семь "-" на
# конце девятишагового рисунка добавляли модели семь ударов, которых она
# не писала — эффект хуже исходного уползания по фазе. Настоящая пауза —
# "." (для неё сэмпл-каталога нет ни в одном паке); ею и добиваем.
def _next_pow2(n: int) -> int:
    """Наименьшая степень двойки, которая ``>= n`` (``n >= 1``)."""
    p = 1
    while p < n:
        p *= 2
    return p


def _normalize_bar_pattern(pattern: str) -> str:
    """Привести длину рисунка ``play(...)`` к степени двойки (issue #1803).

    Паттерны длиной 0 или 1 уже тривиально делят такт — не трогаем.

    🔴 FIX (live 01.09): сначала снимаем ХВОСТОВЫЕ ПАУЗЫ, и только потом
    округляем вверх. Без этого типовой промах модели удваивал такт:

        'X..o.X.o.'  9 → 'X..o.X.o........'  16

    Девятый символ здесь — пауза. Отбросив её, получаем ровно 8: готовый
    грув нужной плотности. Добивка же растягивала такт вдвое, и бочка
    начинала бить в половину задуманного темпа, а вторую половину такта
    занимала тишина. То есть лекарство от уползания по фазе портило грув
    сильнее самой болезни.

    Паузы снимаем ПООДИНОЧКЕ и останавливаемся на первой же степени
    двойки. Снимать их все подряд нельзя: 'X.....' — это «бочка раз в
    шесть шагов», и обрезка до 'X' заставила бы её бить на каждом шаге.
    Округление вверх остаётся страховкой для рисунков без хвостовых пауз
    ('-.---' → '-.---...'): терять звучащие символы мы по-прежнему не
    имеем права.
    """
    n = len(pattern)
    if n <= 1:
        return pattern

    trimmed = pattern
    while len(trimmed) > 1 and _next_pow2(len(trimmed)) != len(trimmed) \
            and trimmed[-1] == ".":
        trimmed = trimmed[:-1]
    if _next_pow2(len(trimmed)) == len(trimmed):
        return trimmed

    target = _next_pow2(len(trimmed))
    return trimmed + "." * (target - len(trimmed))


def spec_from_flat(
    *,
    bpm: float = 120.0,
    root: str = "C",
    scale: str = "minor",
    form: str = DEFAULT_FORM,
    drums: Optional[str] = None,
    drums_sample: int = 0,
    hats: Optional[str] = None,
    hats_sample: int = 3,
    perc: Optional[str] = None,
    perc_sample: int = 0,
    bass_synth: Optional[str] = None,
    bass_notes: Optional[str] = None,
    lead_synth: Optional[str] = None,
    lead_notes: Optional[str] = None,
    pad_synth: Optional[str] = None,
    pad_notes: Optional[str] = None,
    progression: Optional[str] = None,
    repeat: bool = True,
    swing: float = 0.0,
) -> CompositionSpec:
    """Собрать :class:`CompositionSpec` из плоских скалярных аргументов.

    Плоский интерфейс, а не вложенный JSON: маленькие модели заметно
    надёжнее заполняют десяток простых полей, чем одну структуру с
    массивом объектов внутри.

    Слой добавляется только если для него есть И синт, И ступени —
    полупустой слой молча пропускается, а не роняет запрос.
    """
    layers: List[Layer] = []

    if drums and drums.strip():
        layers.append(
            Layer(
                role="drums",
                pattern=_normalize_bar_pattern(drums.strip()),
                sample=int(drums_sample or 0),
            )
        )
    if hats and hats.strip():
        # 🔴 FIX (live 31.08): здесь стояло sample=3 намертво. В библиотеке
        # 4585 сэмплов в трёх паках, а compose_music дотягивался только до
        # вариантов бочки через drums_sample — хэты всегда звучали одним и
        # тем же, перкуссия наружу не выводилась вовсе. Один и тот же
        # тембр во всех треках слышится как «однотипно» ровно так же, как
        # одна и та же мелодия.
        layers.append(
            Layer(
                role="hats",
                pattern=_normalize_bar_pattern(hats.strip()),
                sample=int(hats_sample or 0),
            )
        )
    if perc and perc.strip():
        layers.append(
            Layer(
                role="perc",
                pattern=_normalize_bar_pattern(perc.strip()),
                sample=int(perc_sample or 0),
            )
        )

    for role, synth, notes in (
        ("bass", bass_synth, bass_notes),
        ("lead", lead_synth, lead_notes),
        ("pad", pad_synth, pad_notes),
    ):
        degrees = parse_notes(notes)
        if synth and synth.strip() and degrees:
            layers.append(
                Layer(
                    role=role,
                    synth=synth.strip(),
                    degrees=degrees,
                    dur=ROLE_DEFAULT_DUR[role],
                )
            )

    resolved_form = (form or DEFAULT_FORM).strip()
    _autofill_bass(layers, resolved_form)

    return CompositionSpec(
        bpm=float(bpm),
        root=(root or "C").strip(),
        scale=(scale or "minor").strip(),
        form=resolved_form,
        layers=tuple(layers),
        progression=tuple(int(v) for v in parse_notes(progression)),
        repeat=bool(repeat),
        swing=float(swing or 0.0),
    )


def _autofill_bass(layers: List[Layer], form: str) -> None:
    """Добавить бас, если форма его ждёт, а модель его не дала.

    Промпт просит «минимум 3 слоя», но модель регулярно отдаёт два (live
    30.08: дважды подряд lead + pad без баса). Форма при этом планирует
    басу заметную роль — в `ambient` секция `swell` рассчитана на него, —
    и без баса середина композиции проваливается.

    Бас выводится, а не выдумывается: берётся основной тон гармонии (первая
    ступень пэда, иначе первая ступень мелодии) и его квинта. Получается
    опора, которая гарантированно консонирует с тем, что уже играет —
    в отличие от случайных ступеней, дающих ту самую диссонирующую кашу,
    от которой промпт отговаривает отдельным правилом.

    Мутирует ``layers`` на месте. Ничего не делает, если бас уже есть или
    форма его не задействует.
    """
    if any(layer.role == "bass" for layer in layers):
        return
    plan = resolve_form(form)
    if not any(section.get("bass", 0.0) > 0 for _n, _b, section in plan):
        return

    source = next(
        (layer for layer in layers if layer.role == "pad" and layer.degrees),
        None,
    ) or next(
        (layer for layer in layers if layer.role == "lead" and layer.degrees),
        None,
    )
    if source is None:
        return

    tonic = source.degrees[0]
    layers.append(
        Layer(
            role="bass",
            synth="dub",
            degrees=(tonic, tonic, tonic + 4, tonic),
            dur=ROLE_DEFAULT_DUR["bass"],
        )
    )


def form_summary(name: Optional[str]) -> str:
    """Однострочное описание формы — для сообщения LLM и для логов."""
    plan = resolve_form(name)
    total_bars = sum(int(bars) for _n, bars, _i in plan)
    sections = " → ".join(f"{n}({b})" for n, b, _i in plan)
    return f"{sections} = {total_bars} тактов"
