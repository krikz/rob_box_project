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

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

#: 1 такт = 4 бита в дефолтном метре Renardo (``TempoClock.bar_length()``).
BEATS_PER_BAR = 4

#: Роль -> (имя плеера, октава, базовая амплитуда).
#:
#: Плееры удержаны в d1-d3 / p1-p3 — это ограничение деплоя (d4+/p4+ на
#: роботе не звучат, см. composer.txt и мастер-промпт).
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
    # Второй голос выведенной аранжировки. Слот d3 — потому что Renardo
    # даёт ровно 6 слотов (d1-d3/p1-p3, renardo_sanitizer::
    # _ALLOWED_PLAYER_SLOTS), и остальные пять уже заняты. Имя слота на
    # звук не влияет; конфликта с перкуссией нет, потому что выведенная
    # аранжировка перкуссию не добавляет (грув несут drums + hats).
    "counter": ("d3", 4, 0.26),
}

#: Роли, которые играют сэмплами через ``play(...)``, а не синтом.
DRUM_ROLES = frozenset({"drums", "hats", "perc"})

#: Issue #2841 — жанровый луп (``loop(...)``) поверх ударных. В
#: :data:`ROLE_PROFILE` его нет намеренно: фиксированного слота у лупа
#: быть не может, все шесть уже розданы. Он занимает первый СВОБОДНЫЙ
#: из d1-d3 (см. :func:`_render_loop_layer`).
LOOP_ROLE = "loop"
LOOP_SLOTS: Tuple[str, ...] = ("d1", "d2", "d3")
#: Базовая громкость лупа: тише бочки (0.55) — луп несёт фактуру, а не
#: долю, и не должен перекрывать выведенный из темы бит.
LOOP_BASE_AMP = 0.35
#: В форме без ударных (ambient) луп идёт за подкладом, но тише.
LOOP_PAD_FOLLOW = 0.6

#: Полутоновые интервалы ладов, которые предъявляет схема ``compose_music``.
#:
#: Нужны ровно для одного: перевести ``progression`` (ступени лада) в сдвиг
#: ``Root.default`` (полутоны). Держать здесь копию таблицы дешевле, чем
#: тянуть в этот модуль зависимость от Renardo: он остаётся чистым и
#: тестируемым без звукового стека.
SCALE_INTERVALS: Dict[str, Tuple[int, ...]] = {
    "minor":           (0, 2, 3, 5, 7, 8, 10),
    "major":           (0, 2, 4, 5, 7, 9, 11),
    "dorian":          (0, 2, 3, 5, 7, 9, 10),
    "phrygian":        (0, 1, 3, 5, 7, 8, 10),
    "lydian":          (0, 2, 4, 6, 7, 9, 11),
    "mixolydian":      (0, 2, 4, 5, 7, 9, 10),
    "harmonicMinor":   (0, 2, 3, 5, 7, 8, 11),
    "majorPentatonic": (0, 2, 4, 7, 9),
    "minorPentatonic": (0, 3, 5, 7, 10),
}

#: Сколько тактов держится одна ступень прогрессии.
#:
#: Четыре такта — квадрат, на котором построена почти вся танцевальная и
#: песенная музыка. Гармония, меняющаяся раз в 19 тактов (столько давал
#: прежний расчёт «одна прогрессия на всю форму»), на слух не читается как
#: гармония вообще: слушатель слышит не смену аккорда, а сбой.
BARS_PER_CHORD = 4


def _degree_to_semitones(degree: float, intervals: Sequence[int]) -> int:
    """Ступень лада -> полутоны, с переносом по октавам.

    Ступень может быть отрицательной («на секунду вниз») или больше длины
    лада («октавой выше») — обе формы встречаются в живых вызовах, и обе
    должны давать ноту того же лада, а не хроматический сдвиг.
    """
    size = len(intervals)
    index = int(degree)
    octave, step = divmod(index, size)
    return octave * 12 + intervals[step]

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
        durs: точный ритм нот для фиксированной темы (``None`` — плотность
            владеет аранжировщик через :func:`_dur_var`).
        midi: абсолютные MIDI-ноты темы (``None`` = пауза; ``None`` у слоя —
            тема задана ступенями лада в ``degrees``).
        sample: индекс сэмпла для ударных.
        oct_shift: сдвиг относительно октавы роли — на случай, когда бас
            должен уйти ещё ниже или лид ещё выше.
    """

    role: str
    synth: Optional[str] = None
    pattern: Optional[str] = None
    degrees: Sequence[float] = field(default_factory=tuple)
    dur: float = 1.0
    durs: Optional[Sequence[float]] = None
    midi: Optional[Sequence[Optional[int]]] = None
    sample: int = 0
    oct_shift: int = 0
    #: Длина звучания ноты в битах, независимо от ``dur`` (шага сетки).
    #: В Renardo ``sus`` по умолчанию равен ``dur``, то есть нота тянется
    #: до следующей. Остинато подклада без короткого ``sus`` слипается
    #: обратно в выдержанный аккорд и перестаёт быть ритмом.
    sus: Optional[float] = None


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
    #: Длина фиксированной темы в тактах (0 — темы нет). Секции формы
    #: подгоняются под целое число её повторов, чтобы границы формы
    #: совпадали с границами фразы (:func:`_snap_plan_to_theme`).
    theme_bars: int = 0
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


#: Аргументы плеера, при которых ступень Renardo численно равна MIDI-ноте.
#:
#: Дублирует :data:`core.rtttl.ABSOLUTE_MIDI_ARGS` намеренно: там лежит
#: полное объяснение, почему ``midinote=`` не работает, а здесь модуль
#: остаётся без зависимости от RTTTL-парсера (аранжировщик умеет играть
#: дословную тему из любого источника, не только из рингтона).
ABSOLUTE_MIDI_ARGS = "oct=0, root=0, scale=Scale.chromatic"


def _fmt_midi_note(value) -> str:
    """Одна позиция списка нот: пауза, нота или аккорд.

    Кортеж рендерится круглыми скобками — в Renardo это PGroup, то есть
    ОДНОВРЕМЕННОЕ звучание (renardo_lib/Patterns/Main.py::PGroup). Тот же
    урок, что и в :func:`_fmt_chord`: квадратные скобки дали бы арпеджио
    по одной ноте за такт вместо аккорда.
    """
    if value is None:
        return "None"
    if isinstance(value, (tuple, list)):
        return "(" + ", ".join(str(int(v)) for v in value) + ")"
    return str(int(value))


def _fmt_midi_list(values: Sequence[object]) -> str:
    """Список абсолютных MIDI как ступени Renardo: ``None`` = пауза."""
    return "[" + ", ".join(_fmt_midi_note(v) for v in values) + "]"


def _fmt_chord(values: Sequence[float]) -> str:
    """Отформатировать ступени как ОДНОВРЕМЕННО звучащий аккорд.

    🔴 FIX (live 02.09, «какофония»): в Renardo квадратные скобки — это
    Pattern, то есть ПОСЛЕДОВАТЕЛЬНОСТЬ, а одновременное звучание даёт
    только PGroup — круглые скобки (renardo_lib/Patterns/Main.py::PGroup,
    ``bracket_style = "()"``). Пэд рендерился как ``p3 >> warmpad([0, 4,
    7], dur=4, oct=4)`` и играл арпеджио по одной ноте за такт. Гармони-
    ческой подкладки в треке не было НИ РАЗУ: бас, лид и «пэд» звучали
    как три независимых одноголосных линии, и на каждом их расхождении в
    секунду слышался диссонанс, который нечем было связать. Отсюда и
    «инструменты как будто сбиты» при формально верных ступенях.

    Соседний код это уже предполагал: docstring :func:`_motif_variants`
    описывает пэд как «широкий аккорд (0, 4, 7, 11)» и специально не даёт
    ему транспозиций, «чтобы он держал гармонию». Держать её он не мог.
    """
    if len(values) == 1:
        return _fmt(values[0])
    return "(" + ", ".join(_fmt(v) for v in values) + ")"


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


def resolve_form(
    name: Optional[str], theme_bars: int = 0
) -> List[Tuple[str, int, Dict[str, float]]]:
    """Вернуть план формы, молча падая на дефолт для неизвестного имени.

    Неизвестная форма — не повод отказать в музыке: LLM регулярно
    выдумывает названия, и лучше сыграть дугу, чем вернуть ошибку.

    Args:
        theme_bars: длина фиксированной темы в тактах. Ненулевое значение
            подгоняет секции под ЦЕЛОЕ число её повторов — см.
            :func:`_snap_plan_to_theme`. 0 — форма как записана в
            :data:`FORMS` (музыка, сочинённая с нуля: там темы фиксированной
            длины нет и подгонять не подо что).
    """
    plan = FORMS.get((name or "").strip().lower(), FORMS[DEFAULT_FORM])
    return _snap_plan_to_theme(plan, theme_bars)


def _snap_plan_to_theme(
    plan: Sequence[Tuple[str, int, Dict[str, float]]], theme_bars: int
) -> List[Tuple[str, int, Dict[str, float]]]:
    """Подогнать секции формы под целое число повторов темы.

    🔴 FIX (live 14.09): секции формы записаны круглыми числами тактов
    (8/8/16/8/16/8), а живая тема круглой не бывает — имперский марш
    занимает 9 тактов. На форме ``arc`` это давало 7.1 повтора: каждая
    смена секции приходилась на середину фразы, а на стыке лупа тема
    обрывалась. Слышно это именно как «мелодия не попадает»: громкость и
    слои переключаются там, где у темы ничего не происходит.

    Считаем не по секциям, а от БЮДЖЕТА повторов: сколько раз тема
    укладывается в форму целиком. Бюджет раздаётся секциям пропорцио-
    нально их исходной длине (метод наибольших остатков), и секции, которым
    не досталось ни одного повтора, выпадают.

    Бюджет, а не «каждой секции минимум один повтор» — потому что в
    библиотеке 10460 мелодий и попадаются темы по 40-79 тактов. При
    минимуме в один повтор шесть секций растянули бы такую тему на
    полчаса; с бюджетом длинная тема сама становится формой и играет
    один-два раза.
    """
    if theme_bars <= 0:
        return list(plan)

    total_bars = sum(int(bars) for _n, bars, _i in plan)
    budget = max(1, int(round(total_bars / float(theme_bars))))

    # Наибольшие остатки: сначала целые части, потом по одному повтору
    # тем секциям, у которых дробный хвост больше. Без этого округление
    # каждой секции по отдельности теряет или добавляет повторы, и сумма
    # перестаёт совпадать с бюджетом.
    shares = [budget * int(bars) / total_bars for _n, bars, _i in plan]
    repeats = [int(share) for share in shares]
    order = sorted(
        range(len(plan)), key=lambda i: shares[i] - repeats[i], reverse=True
    )
    for i in order[: budget - sum(repeats)]:
        repeats[i] += 1

    snapped = [
        (name, repeats[i] * theme_bars, intensities)
        for i, (name, _bars, intensities) in enumerate(plan)
        if repeats[i] > 0
    ]
    if snapped:
        return snapped
    # Бюджет целиком осел на секции, которых не осталось (возможно только
    # при вырожденном плане) — играем тему один раз самой длинной секцией.
    longest = max(range(len(plan)), key=lambda i: plan[i][1])
    name, _bars, intensities = plan[longest]
    return [(name, theme_bars, intensities)]


def form_duration_seconds(
    name: Optional[str], bpm: float, theme_bars: int = 0
) -> float:
    """Длительность одного прохода формы в секундах реального времени.

    Issue #1812: у трека, сыгранного с ``repeat=False``, форма конечна и её
    длительность вычислима заранее — та же арифметика, что и в
    :func:`render` при построении ``Clock.future(total_beats, Clock.clear)``.
    Watchdog использует это, чтобы не гасить трек по TTL простоя диалога,
    пока форма физически не доиграла: слушать музыку молча — штатный
    сценарий, а не заброшенная сессия.

    Args:
        name: Имя формы (см. :data:`FORMS`); неизвестное/пустое — дефолт.
        bpm: Темп. Клампится в :data:`BPM_RANGE`, как и в ``render()``.

    Returns:
        Длительность в секундах: ``total_bars * BEATS_PER_BAR * 60 / bpm``.
    """
    clamped_bpm = max(BPM_RANGE[0], min(BPM_RANGE[1], float(bpm)))
    plan = resolve_form(name, theme_bars)
    total_beats = sum(int(bars) for _n, bars, _i in plan) * BEATS_PER_BAR
    return total_beats * 60.0 / clamped_bpm


def _section_intensity(role: str, intensities: Dict[str, float]) -> float:
    """Интенсивность роли в одной секции формы.

    Контрмелодии в таблицах FORMS нет: она появилась вместе с выведенной
    аранжировкой и по смыслу привязана к теме, а не к секции. Её уровень
    берётся от лида (:data:`COUNTER_OF_LEAD`), если форма не назвала его
    явно — тогда второй голос входит и уходит вместе с темой в любой
    форме, включая те, что добавят позже.
    """
    if role == "counter" and "counter" not in intensities:
        return float(intensities.get("lead", 0.0)) * COUNTER_OF_LEAD
    return float(intensities.get(role, 0.0))


def _amp_envelope(
    role: str,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    base_amp: float,
    floor: float = 0.0,
) -> Tuple[List[float], List[int]]:
    """Собрать (значения amp, длительности в битах) для одной роли.

    Соседние секции с одинаковой интенсивностью склеиваются — иначе
    ``var([0.5, 0.5, 0.5], [32, 32, 64])`` порождает лишние переключения
    и делает код нечитаемым.

    Args:
        floor: нижняя граница интенсивности, доля 0..1. Нужна
            фиксированной теме (:data:`FIXED_THEME_AMP_FLOOR`): форма
            вправе делать её тише, но не вправе выключить.
    """
    amps: List[float] = []
    durs: List[int] = []
    for _name, bars, intensities in plan:
        intensity = max(_section_intensity(role, intensities), float(floor))
        amps.append(round(base_amp * intensity, 4))
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


def _is_blank_drum_pattern(pattern: str) -> bool:
    """True, если в паттерне нет ни одного удара — только точки/пробелы.

    ``play('. . . . . . . .')`` — валидный Renardo-код, который ничего не
    издаёт: точка — пауза в ``play``-нотации, пробел — визуальный
    разделитель групп, который renardo просто игнорирует. Такой слой
    нужно считать молчащим (issue #2837), а не реальным ударным паттерном.
    """
    return not any(ch not in ". " for ch in pattern)


def _is_silent_drum_layer(layer: Layer) -> bool:
    """True, если это ударный слой с паттерном из одних точек/пробелов.

    Вынесено отдельно, чтобы не раздувать цикломатику ``_render_layer``
    (cc_budget, issue #2837) лишним составным условием.
    """
    return (
        layer.role in DRUM_ROLES
        and layer.pattern is not None
        and _is_blank_drum_pattern(layer.pattern)
    )


def _render_drum_layer(
    layer: Layer,
) -> Tuple[str, List[str]]:
    """Для ударной роли: вернуть (head, args). Паттерн обязателен."""
    if not layer.pattern:
        raise ArrangementError(
            f"Роль {layer.role!r} играет сэмплами — нужен pattern, "
            'например "X..o.X.o".'
        )
    head = f'play({layer.pattern!r}'
    args: List[str] = []
    if layer.sample:
        args.append(f"sample={int(layer.sample)}")
    return head, args


def _render_melodic_player_head(
    layer: Layer,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    player: str,
) -> Tuple[str, List[str]]:
    """Для мелодической роли: вернуть (head, pre_lines) — игрок и его прелюдия.

    Развитие материала по секциям (#1805) пишется ОТДЕЛЬНОЙ переменной
    ``<player>_motif = Pvar(...)`` (pre_lines), а не инлайном: инлайновый
    ``Pvar(...)`` внутри аргументов плеера ломает regex-парсер валидатора
    качества (ищет ``dur=`` до первой закрывающей скобки — см.
    tools/music.py::_PLAYER_LINE_RE).
    """
    if not layer.synth:
        raise ArrangementError(
            f"Роль {layer.role!r} играет синтом — нужно поле synth."
        )
    if not layer.degrees and layer.midi is None:
        raise ArrangementError(
            f"Роль {layer.role!r} без degrees — играть нечего."
        )

    pre_lines: List[str] = []
    # Тема фиксированная (задан точный ритм) — играем дословно весь
    # трек: никаких транспозиций/инверсий/ретроградов (#1805) и
    # никакой смены плотности (#1806). Развитие идёт формой и слоями
    # ВОКРУГ темы, а не внутри неё.
    if layer.midi is not None:
        if layer.durs is None:
            raise ArrangementError(
                f"Роль {layer.role!r} с midi — нужен точный ритм durs."
            )
        # 🔴 FIX (live 14.09): было ``midinote=[...]`` — Renardo этот
        # ключ как источник высоты ИГНОРИРУЕТ (пересчитывает freq из
        # degree и затирает, Players.py::new_message_header). degree не
        # передавался → вся тема звучала на одной ноте MIDI 60. Ноты
        # идут позиционно, а ABSOLUTE_MIDI_ARGS делает ступень равной
        # MIDI-ноте; oct здесь не применяется (высота уже абсолютная).
        head = f"{layer.synth}({_fmt_midi_list(layer.midi)}"
        return head, pre_lines

    if layer.durs is not None:
        head = f"{layer.synth}({_fmt_list(layer.degrees)}"
        return head, pre_lines

    variants, variant_durs = _motif_variants(layer.role, layer.degrees, plan)
    if len(variants) > 1:
        motif_name = f"{player}_motif"
        pre_lines.append(
            f"{motif_name} = Pvar({_fmt_nested_list(variants)}, "
            f"{_fmt_list(variant_durs)})"
        )
        head = f"{layer.synth}({motif_name}"
        return head, pre_lines

    if layer.role == "pad":
        # Пэд держит гармонию — все его ступени звучат одновременно.
        head = f"{layer.synth}({_fmt_chord(layer.degrees)}"
    else:
        head = f"{layer.synth}({_fmt_list(layer.degrees)}"
    return head, pre_lines


def _render_melodic_args(
    layer: Layer,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    role_oct: int,
) -> List[str]:
    """Собрать аргументы (dur / oct) для мелодической роли."""
    args: List[str] = []
    if layer.durs is not None:
        args.append(f"dur={_fmt_list(layer.durs)}")
    else:
        # #1806 — плотность нот по секциям, не constant dur всю форму.
        dur_values, dur_durs = _dur_var(layer.role, plan, layer.dur)
        if len(dur_values) > 1:
            args.append(f"dur=var({_fmt_list(dur_values)}, {_fmt_list(dur_durs)})")
        else:
            args.append(f"dur={_fmt(layer.dur)}")
    if layer.sus is not None:
        args.append(f"sus={_fmt(layer.sus)}")
    if layer.midi is None:
        args.append(f"oct={max(2, min(7, role_oct + int(layer.oct_shift)))}")
    else:
        # root=0 обязателен: иначе Root.default (var с прогрессией)
        # транспонирует дословную тему вслед за гармонией.
        args.append(ABSOLUTE_MIDI_ARGS)
    return args


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

    if _is_silent_drum_layer(layer):
        # Паттерн из одних точек/пробелов — ни одного удара. Это тот же
        # «слой молчит», что и роль вне текущей секции формы (issue #2837):
        # play('. . . . . . . .') синтаксически валиден, но реально не
        # звучит, и guard на количество плееров должен это видеть.
        return None

    # Фиксированная тема (и её второй голос) не имеет права замолчать
    # совсем — её попросили сыграть. См. FIXED_THEME_AMP_FLOOR.
    floor = 0.0
    if layer.midi is not None and layer.role in ("lead", "counter"):
        floor = FIXED_THEME_AMP_FLOOR
        if layer.role == "counter":
            floor *= COUNTER_OF_LEAD
    amps, durs = _amp_envelope(layer.role, plan, base_amp, floor=floor)
    if not any(amps):
        # Роль не участвует ни в одной секции этой формы (например drums в
        # ambient) — плеер не создаём вовсе, чтобы не гонять тихие ноты.
        return None

    if len(amps) == 1:
        amp_expr = _fmt(amps[0])
    else:
        amp_expr = f"var({_fmt_list(amps)}, {_fmt_list(durs)})"

    if layer.role in DRUM_ROLES:
        head, args = _render_drum_layer(layer)
    else:
        head, pre_lines = _render_melodic_player_head(layer, plan, player)
        args = _render_melodic_args(layer, plan, role_oct)

    args.append(f"amp={amp_expr}")
    # Фильтр-свип вешаем на держащие слои. На ударные не вешаем: срезанная
    # атака бочки слышна как проваленный грув.
    # 🔴 FIX (live 14.09, «солло не все ноты играет»): свип ездит от 700 Гц,
    # и на нижнем краю он срезает ВСЁ, что выше. У имперского марша выше
    # 700 Гц лежат 37% нот темы (она идёт до D6 = 1175 Гц), а бас и
    # подклад — целиком ниже: 0 нот из 36 и 0 из 108. Поэтому аккомпанемент
    # звучал целым, а тема циклически теряла ноты по ходу свипа.
    #
    # Фиксированную тему форма вправе делать тише и ярче, но не вправе
    # стирать — тот же принцип, что у FIXED_THEME_AMP_FLOOR и что у
    # ударных, которые исключены из свипа с самого начала (срезанная атака
    # бочки слышна как проваленный грув). Сочинённой с нуля мелодии свип
    # по-прежнему достаётся: там он краска, а не потеря материала.
    fixed_theme = layer.midi is not None and layer.role in ("lead", "counter")
    if use_filter and layer.role in ("bass", "pad", "lead") and not fixed_theme:
        args.append("lpf=gflt")

    line = f"{player} >> {head}, " + ", ".join(args) + ")"
    if layer.role in DRUM_ROLES:
        return line
    return "\n".join(pre_lines + [line]) if pre_lines else line


def _no_players_error(
    spec: CompositionSpec,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
) -> ArrangementError:
    """Собрать понятную ошибку «0 плееров» для :func:`render` (issue #2837).

    Вынесено отдельно, чтобы не раздувать цикломатику ``render`` — сама
    функция только считает ``rendered_players`` и решает, звать ли эту
    фабрику.
    """
    form_roles = sorted({r for _n, _b, i in plan for r in i})
    spec_roles = sorted({layer.role for layer in spec.layers})
    spec_roles_text = ", ".join(spec_roles) or "(нет слоёв)"
    form_roles_text = ", ".join(form_roles) or "(ничего)"
    return ArrangementError(
        "Ни один слой не звучит: спецификация пуста для формы "
        f"{spec.form!r}. В спеке заданы роли {spec_roles_text}, "
        f"а форма {spec.form!r} играет только {form_roles_text}. "
        "Либо смени форму на ту, что задействует нужные роли, либо "
        "перепроверь, что паттерны/ступени не пустые (паттерн из одних "
        "точек и пробелов считается тишиной)."
    )


def _sample_loops():
    """Каталог лупов — ленивым импортом, а не на уровне модуля.

    ``tools/gen_tool_catalog.py`` грузит этот файл напрямую, без пакета
    (ради ``FORMS``/``VALID_ROOTS``), и относительный импорт на уровне
    модуля там падает. Лупы нужны только при рендере/сборке спецификации.
    """
    from . import sample_loops

    return sample_loops


_PLAYER_SLOT_RE = re.compile(r"^\s*([dp]\d)\s*>>", re.MULTILINE)


def _free_loop_slot(lines: Sequence[str]) -> str:
    """Первый из d1-d3, который не занят уже отрендеренными слоями.

    Raises:
        ArrangementError: все три заняты (бочка + хэты + перкуссия или
            контрмелодия) — лупу честно некуда встать.
    """
    used = set(_PLAYER_SLOT_RE.findall("\n".join(lines)))
    free = next((slot for slot in LOOP_SLOTS if slot not in used), None)
    if free is None:
        raise ArrangementError(
            "groove_loop: слоты d1-d3 уже заняты (бочка, хэты и перкуссия "
            "или второй голос темы). Убери perc, либо поставь "
            "drum_style='none' — луп сам несёт грув."
        )
    return free


def _render_loop_layer(
    layer: Layer,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    bpm: float,
    player: str,
) -> str:
    """Отрендерить жанровый луп: ``loop(имя, dur=N, beat_stretch=1, amp=...)``.

    ``beat_stretch=1`` растягивает файл ровно на ``dur`` битов, поэтому
    луп идёт в темп формы, а не в свой родной. ``dur`` — ближайшая к
    родной длине степень двойки (:func:`sample_loops.loop_beats`): так
    скорость и высота меняются меньше всего.

    Громкость идёт за бочкой: где форма снимает ударные (intro, break),
    молчит и луп. В форме без ударных вовсе (ambient) — за подкладом.
    Имя остаётся коротким: путь до файла подставит санитайзер, он же
    проверит флаг pack 1.
    """
    loops = _sample_loops()
    info = loops.find_loop(layer.pattern or "")
    if info is None:
        raise ArrangementError(f"groove_loop: неизвестный луп {layer.pattern!r}.")
    has_drums = any("drums" in intensities for _n, _b, intensities in plan)
    source, scale = ("drums", 1.0) if has_drums else ("pad", LOOP_PAD_FOLLOW)
    amps, durs = _amp_envelope(source, plan, LOOP_BASE_AMP * scale)
    amp_expr = _fmt(amps[0]) if len(amps) == 1 else f"var({_fmt_list(amps)}, {_fmt_list(durs)})"
    beats = loops.loop_beats(info, bpm)
    return (
        f"{player} >> loop({info.name!r}, dur={_fmt(beats)}, "
        f"beat_stretch=1, amp={amp_expr})"
    )


def _append_layer_lines(
    lines: List[str],
    spec: CompositionSpec,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    bpm: float,
) -> int:
    """Дописать в ``lines`` строки всех слоёв; лупы — последними.

    Возвращает число реально отрисованных плееров (луп тоже плеер) —
    по нему :func:`render` решает, не пустой ли трек (#2837).

    Луп (#2841) рендерится после остальных, потому что его слот — первый
    свободный из d1-d3, а занятость известна только по уже готовым строкам.
    """
    count = 0
    for layer in spec.layers:
        if layer.role == LOOP_ROLE:
            continue
        rendered = _render_layer(layer, plan, use_filter=spec.filter_sweep)
        if rendered is not None:
            lines.append(rendered)
            count += 1
    for layer in spec.layers:
        if layer.role == LOOP_ROLE:
            lines.append(_render_loop_layer(layer, plan, bpm, _free_loop_slot(lines)))
            count += 1
    return count


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
    plan = resolve_form(spec.form, getattr(spec, "theme_bars", 0))
    total_bars = sum(int(bars) for _n, bars, _i in plan)
    total_beats = total_bars * BEATS_PER_BAR

    lines: List[str] = ["Clock.clear()", f"Clock.bpm = {_fmt(bpm)}"]

    if spec.swing > 0:
        # #1806 — ровные восьмые не читаются как джаз/блюз/шафл ни при
        # каком выборе синтов. ``Clock.swing`` смещает нечётные восьмые
        # через ``nudge`` (renardo_lib/TempoClock.py:290) — одна строка на
        # всю форму, ноты и dur трогать не нужно.
        swing = max(0.0, min(0.3, float(spec.swing)))
        lines.append(f"Clock.swing({_fmt(swing)})")

    if spec.progression:
        # 🔴 FIX (live 02.09, «какофония») — здесь было три бага сразу:
        #
        # 1. ТОНИКА ТЕРЯЛАСЬ. ``Root.default = var([0, 0, 5, 3], ...)`` —
        #    Renardo трактует Root как ХРОМАТИЧЕСКИЙ номер ноты (Root.py:
        #    ``CHROMATIC_NOTES``, 0 = C), поэтому запрошенный root просто
        #    не доезжал: 91 из 99 живых вызовов шли с progression, и все
        #    они звучали в до, какую бы тонику модель ни выбрала. Заодно
        #    это съедало разнообразие — тональный центр у всех треков сета
        #    был один и тот же.
        # 2. СТУПЕНИ ЧИТАЛИСЬ КАК ПОЛУТОНА. И схема тула, и промпт, и
        #    docstring ``CompositionSpec.progression`` говорят «ступени
        #    лада», а Root складывается в полутонах: «5» вместо V ступени
        #    (7 полутонов в миноре) давала сдвиг на кварту, «3» вместо IV
        #    (5 полутонов) — на малую терцию. Гармония уезжала не туда,
        #    куда её вела мелодия, написанная в ступенях того же лада.
        # 3. СМЕНА ГАРМОНИИ НЕ ПОПАДАЛА В СЕТКУ. ``total_beats //
        #    len(progression)`` на форме buildup (76 тактов) давало шаг в
        #    76 БИТОВ — 19 тактов. Тоника менялась посреди фразы и посреди
        #    секции, под держащейся нотой пэда. Теперь шаг — целое число
        #    тактов (кратно 4 тактам, как в обычной квадратной форме), а
        #    прогрессия просто прокручивается по кругу до конца формы.
        root_semitone = VALID_ROOTS.index(root)
        intervals = SCALE_INTERVALS.get(spec.scale, SCALE_INTERVALS["minor"])
        roots = [
            root_semitone + _degree_to_semitones(degree, intervals)
            for degree in spec.progression
        ]
        # Прогрессия прокручивается по кругу столько раз, сколько уложится
        # в форму: ``var`` в Renardo зацикливается сам.
        step = BARS_PER_CHORD * BEATS_PER_BAR
        lines.append(f"Root.default = var({_fmt_list(roots)}, {_fmt(step)})")
    else:
        lines.append(f'Root.default = "{root}"')

    lines.append(f'Scale.default = "{spec.scale}"')

    if spec.filter_sweep:
        # Свип длиной в половину формы: за один проход формы фильтр
        # успевает открыться и закрыться.
        lines.append(f"gflt = linvar([700, 4500], {_fmt(total_beats // 2)})")

    # Считаем именно отрисованные строки-плееры, а не длину ``lines``: шапка
    # не фиксирована — ``progression``/``swing``/``filter_sweep`` добавляют в
    # неё свои строки (issue #2837, живой прогон 23.09: с filter_sweep=True
    # шапка всегда 5 строк, и guard на ``len(lines) <= 4`` не срабатывал ни
    # при каком количестве слоёв).
    rendered_players = _append_layer_lines(lines, spec, plan, bpm)

    if rendered_players == 0:
        raise _no_players_error(spec, plan)

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
ROLE_DEFAULT_DUR: Dict[str, float] = {
    "bass": 1.0, "lead": 0.5, "pad": 4.0, "counter": 0.5,
}

#: Доля громкости лида, которую получает контрмелодия там, где форма не
#: называет её явно. Второй голос — тень темы: он обязан быть тише, но
#: обязан появляться и исчезать ВМЕСТЕ с ней. Отдельные числа в каждой
#: секции каждой формы (FORMS) дали бы то же самое, но рассинхронились бы
#: при первой же правке формы.
COUNTER_OF_LEAD = 0.55

#: Нижняя граница громкости ФИКСИРОВАННОЙ темы (RTTTL), доля от базовой.
#:
#: 🔴 FIX (live 14.09): у роли ``lead`` нет интенсивности в секциях intro
#: и build формы ``arc`` (и в intro/gap у остальных), поэтому тема
#: получала ``amp=var([0, ...], [64, ...])`` — на 80 BPM это 48 секунд
#: барабанов без мелодии в ответ на «сыграй имперский марш». Для темы,
#: которую ПОПРОСИЛИ сыграть, молчание — не динамика, а невыполненная
#: просьба: форма продолжает менять её громкость, но больше не гасит.
FIXED_THEME_AMP_FLOOR = 0.5


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


def parse_midi(raw: Optional[str]) -> Tuple[Optional[int], ...]:
    """Разобрать абсолютные MIDI-ноты из ``"74, None, 70"``.

    ``None`` (или пустое место) — пауза. Остальное — целое число MIDI.
    Принимает и ``"[74, None, 70]"``, и ``"74 70"`` — те же послабления,
    что и :func:`parse_notes` для ступеней.

    Raises:
        ArrangementError: в строке есть не число и не ``None``.
    """
    if not raw or not raw.strip():
        return ()
    cleaned = raw.strip().strip("[]()").replace(";", ",").replace(" ", ",")
    out: List[Optional[int]] = []
    for chunk in cleaned.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if chunk.lower() == "none":
            out.append(None)
            continue
        try:
            out.append(int(float(chunk)))
        except ValueError as exc:
            raise ArrangementError(
                f"MIDI-ноты должны быть целыми числами (или None для паузы), "
                f"получено {chunk!r} в {raw!r}."
            ) from exc
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


def _add_drum_layer_if_present(
    layers: List[Layer], role: str, pattern: Optional[str], sample: int
) -> None:
    """Добавить ударный слой, если есть паттерн."""
    if not pattern or not pattern.strip():
        return
    if _is_blank_drum_pattern(pattern.strip()):
        # Только точки/пробелы — LLM явно прислала «тишину» под этой ролью
        # (issue #2837). Не создаём слой вовсе, как и при pattern=None.
        return
    layers.append(
        Layer(
            role=role,
            pattern=_normalize_bar_pattern(pattern.strip()),
            sample=int(sample or 0),
        )
    )


def _add_lead_midi_layer(
    layers: List[Layer], synth: str, lead_midi: str, lead_dur: Optional[str]
) -> bool:
    """Добавить lead-слоя по абсолютному MIDI (RTTTL-путь). True если добавлен."""
    if not (lead_midi and lead_midi.strip()):
        return False
    midi = parse_midi(lead_midi)
    durs = parse_notes(lead_dur) if lead_dur else None
    if durs is None:
        raise ArrangementError(
            "lead_midi требует lead_dur той же длины — точный ритм темы."
        )
    if len(durs) != len(midi):
        raise ArrangementError(
            f"lead_dur должно быть той же длины, что lead_midi: "
            f"{len(durs)} длительностей на {len(midi)} нот. "
            "Каждой ноте темы — своя длительность."
        )
    layers.append(
        Layer(
            role="lead",
            synth=synth.strip(),
            midi=midi,
            dur=ROLE_DEFAULT_DUR["lead"],
            durs=durs,
        )
    )
    return True


def _add_melodic_layer(
    layers: List[Layer], role: str, synth: str, notes: Optional[str],
    lead_dur: Optional[str],
) -> bool:
    """Добавить мелодический слой (bass/lead/pad). True если добавлен."""
    degrees = parse_notes(notes)
    if not degrees:
        return False
    # Тема фиксированная: точный ритм даёт LLM. Без него плотность
    # владеет аранжировщик (ROLE_DEFAULT_DUR + _dur_var) — путь
    # для сочинённой с нуля музыки не меняется.
    durs = None
    if role == "lead" and lead_dur:
        durs = parse_notes(lead_dur)
        if len(durs) != len(degrees):
            raise ArrangementError(
                f"lead_dur должно быть той же длины, что lead_notes: "
                f"{len(durs)} длительностей на {len(degrees)} нот. "
                "Каждой ноте темы — своя длительность."
            )
    layers.append(
        Layer(
            role=role,
            synth=synth.strip(),
            degrees=degrees,
            dur=ROLE_DEFAULT_DUR[role],
            durs=durs,
        )
    )
    return True


#: Полутоны, которые синт добавляет к запрошенной высоте ВНУТРИ СЕБЯ.
#:
#: 🔴 FIX (live 14.09, «режет ноты», «бас гудит»): 36 из 63 SynthDef'ов
#: на роботе меняют частоту прямо в определении — ``freq = freq / 4`` и
#: подобное. Басовые делают это осмысленно (их пишут так, чтобы играть
#: басом от обычных ступеней), но аранжировщик об этом не знал и считал
#: все регистры по НОМИНАЛЬНЫМ нотам. Последствия были разные и все
#: неприятные:
#:
#: * ``dirt`` (÷4) как тема — мелодия уезжала на две октавы вниз, ПОД
#:   подклад, и он её накрывал: на слух «ноты пропадают»;
#: * ``ecello`` / ``jbass`` / ``dub`` / ``moogbass`` (÷4) как бас — нота
#:   D2 звучала как D0, ниже слышимого: бас превращался в гул;
#: * ``blip`` (×2) как тема — октавой выше задуманного;
#: * ``subbass`` (÷3) как бас — сдвиг на 19 полутонов, то есть октава
#:   ПЛЮС КВИНТА: просишь ре, звучит соль. Бас играл не ту ступень
#:   аккорда, и никакая гармонизация этого не спасала.
#:
#: Значения — на сколько полутонов НАДО ПОДНЯТЬ ноту при отправке, чтобы
#: прозвучала задуманная. Внутри модуль всюду рассуждает о звучащей
#: высоте, поправка вносится последним шагом, при выводе кода.
#:
#: Таблица снята с самих .scd (``freq = freq * N`` / ``freq / N``,
#: полутоны = 12·log2(коэффициент)). ``subbass`` округлён с ошибкой в
#: 2 цента — она неслышима.
SYNTH_SEMITONE_SHIFT: Dict[str, int] = {
    "risseto": 80, "twang": 36,
    "bass": 24, "bassguitar": 24, "dafbass": 24, "dbass": 24, "dblbass": 24,
    "dirt": 24, "donk1": 24, "donk2": 24, "dub": 24, "ebass": 24,
    "ecello": 24, "fbass": 24, "jbass": 24, "moogbass": 24, "squish": 24,
    "subbass": 19,
    "arpy": 12, "drone": 12, "faim2": 12, "fuzz": 12, "glitchbass": 12,
    "noquarter": 12, "pbass": 12, "soft": 12, "ssaw": 12, "star": 12,
    "subbass2": 12, "vibass": 12, "wobblebass": 12, "wsawbass": 12,
    "blip": -12, "gong": -12, "noise": -12, "rissetobell": -12,
}


def _compensate(note, shift: int):
    """Поднять ноту (или аккорд) на ``shift`` полутонов; ``None`` — пауза."""
    if shift == 0 or note is None:
        return note
    if isinstance(note, (tuple, list)):
        return tuple(int(v) + shift for v in note)
    return int(note) + shift


#: Длина удара остинато в битах. Короче шага сетки (доли) — иначе
#: соседние аккорды смыкаются и остинато снова слышится как подложка.
PAD_STAB_SUS = 0.4


#: Слова, которыми модель (или safety-net в tools/music.py) просит «слоя
#: нет» для любого ``*_synth``. Сравнение регистронезависимо и после
#: ``strip()`` — ``' None '``/``'NONE'`` тоже схлопываются.
NO_SYNTH_WORDS = frozenset({"none", "off", "null"})


def normalize_synth(value: Optional[str]) -> Optional[str]:
    """Схлопнуть «синта нет» в ``None`` — единая точка на входе.

    До issue #2836 «синта нет» понимали по-разному: код, добавляющий слой
    (:func:`_add_derived_layers`, цикл в :func:`spec_from_flat`), считал
    пустым только ``None``/``''``. А ``_heavy_brass_safety_net`` в
    ``tools/music.py`` подставлял literal-строку ``'none'`` — она truthy,
    поэтому доходила до :class:`Layer` как настоящий синт и рендерилась
    ``d3 >> none([...])``. SynthDef с именем ``none`` не существует в
    scsynth, и ``renardo_sanitizer`` отклонял код («Синта 'none' не
    существует»).

    ``None``/``''``/``'none'``/``'off'``/``'null'`` (любой регистр, с
    пробелами по краям) теперь везде означают одно и то же: слоя нет.
    """
    if value is None:
        return None
    stripped = value.strip()
    if not stripped or stripped.lower() in NO_SYNTH_WORDS:
        return None
    return stripped


def _resolve_counter_synth_default(
    counter_synth: Optional[str], lead_synth: str
) -> Optional[str]:
    """Разрешить counter_synth к финальному значению перед добавлением слоя.

    Три исхода из сырого (ненормализованного) значения:

    * реальный синт → возвращается как есть (нормализованный, без
      пробелов);
    * не задан вовсе (``None``/``''``) → фолбэк на ``lead_synth`` — второй
      голос звучит тембром темы (унисон в терцию, всегда безопасный
      вариант по умолчанию);
    * явное слово-отключение (``'none'``/``'off'``/``'null'``, issue
      #2836) → ``None`` без фолбэка — второй голос выключен, а не
      восстановлен обратно на lead_synth.

    Решение «фолбэк или отключение» смотрит на СЫРОЕ значение, до
    :func:`normalize_synth`: нормализация схлопывает «не задано» и
    «явно отключено» в одно и то же ``None``, и только здесь, различая
    их заранее, можно выбрать между двумя разными исходами.
    """
    is_disable_word = (
        counter_synth is not None
        and counter_synth.strip() != ""
        and counter_synth.strip().lower() in NO_SYNTH_WORDS
    )
    normalized = normalize_synth(counter_synth)
    if normalized is not None:
        return normalized
    if is_disable_word:
        return None
    return lead_synth


def _add_derived_layers(
    layers: List[Layer],
    harmony,
    *,
    theme_octaves: bool,
    lead_synth: str,
    bass_synth: Optional[str],
    pad_synth: Optional[str],
    counter_synth: Optional[str],
    drums_sample: int,
    hats_sample: int,
) -> None:
    """Разложить готовую гармонизацию темы в слои аранжировки.

    От LLM здесь берутся ТОЛЬКО тембры (синты и сэмплы) — ноты и рисунки
    целиком выведены из мелодии (см. :mod:`core.harmonize`). Поэтому
    ``bass_notes`` / ``pad_notes`` / ``progression`` / рисунки ударных,
    если модель их прислала, сюда не попадают: они были главным
    источником фальши (модель писала их, не видя ни одной ноты темы).

    Перкуссия не добавляется намеренно: её слот занимает контрмелодия,
    а грув уже несут выведенные бочка и хэты.
    """
    _add_drum_layer_if_present(layers, "drums", harmony.drums, drums_sample)
    _add_drum_layer_if_present(layers, "hats", harmony.hats, hats_sample)

    # Наряд аранжировки соразмерен плотности темы. Второй голос и удвоение
    # в октаву делают тему ТОЛЩЕ — плотной, громкой теме (марш, гимн,
    # чиптюн) это вес, а редкой (крадущаяся тема с паузами между фразами)
    # это гибель: её узнают по тонкой одинокой линии и по тишине вокруг.
    # См. harmonize::DENSE_ONSETS_PER_BEAT.
    dense = getattr(harmony, "dense", True)
    for role, synth, part in (
        ("bass", bass_synth, harmony.bass),
        ("lead", lead_synth, harmony.lead),
        ("pad", pad_synth, harmony.pad),
        ("counter", counter_synth, harmony.counter if dense else ()),
    ):
        if not (synth and synth.strip()) or not part:
            continue
        notes = [note for note, _dur in part]
        if _should_octave_double(role, theme_octaves, dense, notes):
            notes = [_octave_double(note) for note in notes]
        # Последний шаг: поправка на собственное транспонирование синта.
        # До этой строки всё выше рассуждало о ЗВУЧАЩЕЙ высоте — регистры,
        # удвоение, потолок подклада. См. SYNTH_SEMITONE_SHIFT.
        shift = SYNTH_SEMITONE_SHIFT.get(synth.strip(), 0)
        if shift:
            notes = [_compensate(note, shift) for note in notes]
        layers.append(
            Layer(
                role=role,
                synth=synth.strip(),
                midi=tuple(notes),
                durs=tuple(float(dur) for _note, dur in part),
                dur=ROLE_DEFAULT_DUR[role],
                # Подклад ведёт остинато: удар должен быть короче шага
                # сетки, иначе соседние аккорды сливаются в выдержанный
                # звук и ритм пропадает (см. harmonize::_build_pad).
                sus=PAD_STAB_SUS if role == "pad" else None,
            )
        )


#: Ниже этой ноты удваивать тему октавой вниз нельзя: удвоение залезет
#: в регистр баса. Библиотека — 10460 разных мелодий, и низких среди них
#: полно (басовые риффы, мужские вокальные темы); для них удвоение даёт
#: не вес, а кашу на границе с басовой партией.
MIN_MIDI_FOR_OCTAVE_DOUBLE = 60  # C4


def _fits_octave_double(notes: Sequence[object]) -> bool:
    """Хватает ли теме высоты, чтобы удвоить её октавой вниз?

    Смотрим на САМУЮ НИЗКУЮ ноту темы, а не на среднюю: достаточно одной
    низкой ноты, чтобы её удвоение село на бас. Тема из одних пауз
    удвоения не получает — удваивать нечего.
    """
    pitches = [
        int(n) for n in notes if isinstance(n, (int, float)) and n is not None
    ]
    return bool(pitches) and min(pitches) >= MIN_MIDI_FOR_OCTAVE_DOUBLE


def _should_octave_double(role: str, theme_octaves: bool, dense: bool, notes: Sequence[object]) -> bool:
    """Удваивать ли тему октавой вниз для этого слоя.

    Удвоение нужно только лид-голосу, при включённой опции и в плотной
    теме: плотная, громкая тема от удвоения выигрывает в весе, редкая —
    теряет характер. Дополнительно тема должна иметь запас высоты, чтобы
    удвоение не село на регистр баса (:func:`_fits_octave_double`).
    """
    return role == "lead" and theme_octaves and dense and _fits_octave_double(notes)


def _octave_double(note):
    """Нота → она же вместе с октавой ниже (``None`` остаётся паузой).

    Тема в октавах — базовый приём оркестровки: в марше её ведут низкая
    медь и низкие струнные одновременно, и именно удвоение даёт вес.
    Одна линия одним тембром на слух и есть «монофонично и плоско».
    """
    if note is None:
        return None
    if isinstance(note, (tuple, list)):
        return tuple(note)
    return (int(note) - 12, int(note))


def spec_from_flat(
    *,
    harmony=None,
    counter_synth: Optional[str] = None,
    theme_octaves: bool = True,
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
    lead_dur: Optional[str] = None,
    lead_midi: Optional[str] = None,
    pad_synth: Optional[str] = None,
    pad_notes: Optional[str] = None,
    progression: Optional[str] = None,
    repeat: bool = True,
    swing: float = 0.0,
    groove_loop: Optional[str] = None,
) -> CompositionSpec:
    """Собрать :class:`CompositionSpec` из плоских скалярных аргументов.

    Плоский интерфейс, а не вложенный JSON: маленькие модели заметно
    надёжнее заполняют десяток простых полей, чем одну структуру с
    массивом объектов внутри.

    Слой добавляется только если для него есть И синт, И ступени —
    полупустой слой молча пропускается, а не роняет запрос.

    ``lead_midi`` — путь ТОЧНОГО воспроизведения известной темы абсолютным
    MIDI (из RTTTL-библиотеки): играется дословно с ``lead_dur``, в ступени
    лада не переводится.

    ``harmony`` (:class:`core.harmonize.Harmonization`) — тема, уже
    разложенная на партии. Если он передан, ВСЕ ноты аккомпанемента и
    рисунки ударных берутся из него, а не от модели: они выведены из
    самой мелодии и промахнуться мимо её тональности не могут. За
    моделью остаются тембры, форма и темп.

    ``groove_loop`` (issue #2841) — имя лупа из каталога
    :mod:`core.sample_loops`; добавляет слой ``loop(...)`` в свободный
    d-слот и в выведенной, и в сочинённой аранжировке.
    """
    layers: List[Layer] = []
    _add_loop_layer(layers, groove_loop)

    root = (root or "C").strip()
    scale = (scale or "minor").strip()
    form = (form or DEFAULT_FORM).strip()
    swing = float(swing or 0.0)

    # Единая точка нормализации «синта нет» (issue #2836): None/''/'none'/
    # 'off'/'null' (любой регистр) → None, до того как значение доберётся
    # до кода, решающего добавлять слой или нет.
    lead_synth = normalize_synth(lead_synth)
    bass_synth = normalize_synth(bass_synth)
    pad_synth = normalize_synth(pad_synth)

    if harmony is not None:
        if not (lead_synth and lead_synth.strip()):
            raise ArrangementError(
                "Для выведенной аранжировки нужен lead_synth — тема "
                "должна чем-то играть."
            )
        _add_derived_layers(
            layers,
            harmony,
            theme_octaves=bool(theme_octaves),
            lead_synth=lead_synth,
            bass_synth=bass_synth,
            pad_synth=pad_synth,
            # См. _resolve_counter_synth_default: по умолчанию — тембр
            # темы (унисон в терцию), явное 'none'/'off'/'null' —
            # отключение без фолбэка на lead_synth.
            counter_synth=_resolve_counter_synth_default(
                counter_synth, lead_synth
            ),
            drums_sample=drums_sample,
            hats_sample=hats_sample,
        )
        return CompositionSpec(
            bpm=float(bpm),
            root=root,
            scale=scale,
            form=form,
            layers=tuple(layers),
            # Прогрессии нет намеренно: вся гармония уже записана
            # абсолютными нотами баса и пэда. Root.default, который
            # двигал бы ступени, для них не существует — и потому не
            # может увести аккомпанемент от фиксированной темы.
            progression=(),
            theme_bars=int(harmony.bars),
            repeat=bool(repeat),
            swing=swing,
        )

    # 🔴 FIX (live 31.08): здесь стояло sample=3 намертво. В библиотеке
    # 4585 сэмплов в трёх паках, а compose_music дотягивался только до
    # вариантов бочки через drums_sample — хэты всегда звучали одним и
    # тем же, перкуссия наружу не выводилась вовсе. Один и тот же
    # тембр во всех треках слышится как «однотипно» ровно так же, как
    # одна и та же мелодия.
    _add_drum_layer_if_present(layers, "drums", drums, drums_sample)
    _add_drum_layer_if_present(layers, "hats", hats, hats_sample)
    _add_drum_layer_if_present(layers, "perc", perc, perc_sample)

    for role, synth, notes in (
        ("bass", bass_synth, bass_notes),
        ("lead", lead_synth, lead_notes),
        ("pad", pad_synth, pad_notes),
    ):
        if not (synth and synth.strip()):
            continue

        # Тема абсолютным MIDI (известная мелодия из RTTTL): играем дословно,
        # в ступени лада не переводим — иначе хроматические ноты теряются.
        if role == "lead" and _add_lead_midi_layer(
            layers, synth, lead_midi or "", lead_dur
        ):
            continue

        _add_melodic_layer(layers, role, synth, notes, lead_dur)

    _autofill_bass(layers, form)

    return CompositionSpec(
        bpm=float(bpm),
        root=root,
        scale=scale,
        form=form,
        layers=tuple(layers),
        progression=tuple(int(v) for v in parse_notes(progression)),
        repeat=bool(repeat),
        swing=swing,
    )


def _add_loop_layer(layers: List[Layer], groove_loop: Optional[str]) -> None:
    """Добавить слой жанрового лупа, если он задан (issue #2841).

    Raises:
        ArrangementError: имени нет в каталоге — сообщение перечисляет
            доступные, чтобы модель исправилась следующим вызовом.
    """
    name = (groove_loop or "").strip()
    if not name or name.lower() == "none":
        return
    loops = _sample_loops()
    info = loops.find_loop(name)
    if info is None:
        known = ", ".join(sorted(loops.loop_catalog()))
        raise ArrangementError(
            f"groove_loop {name!r} нет в каталоге лупов. Доступны: {known}."
        )
    layers.append(Layer(role=LOOP_ROLE, pattern=info.name))


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
