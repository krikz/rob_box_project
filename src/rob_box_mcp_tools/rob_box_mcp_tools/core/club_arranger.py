"""club_arranger.py — режим ``style="club"``: клубный трек по эталону DJ Dave.

План: docs/design/2026-09-28-dj-live-coding-quality-plan.md, §5 п.2, §7.1,
§7.2, §7.6, §7.10, §7.11. Эталон звучания —
``core/reference_tracks/by_design_dj_dave.foxdot`` (перекладка «By Design»).

Модуль чистый (без Renardo/ROS): :func:`render_club` детерминированно
собирает ПОЛНУЮ Renardo-программу из шести плееров (санитайзер разрешает
только d1-d3/p1-p3):

========  =========  =====================================================
слот      слой        что играет
========  =========  =====================================================
d1        kick        синкопа By Design ``X..X..X..(.X)X.....`` или 4/4
d2        hats        16-е с дырами ``(-.)``
d3        clap        клэп на 2 и 4 + открытый хэт на слабые 8-е (одна строка)
p1        lead        арпеджио 16-ми: сидированный риф в пентатонике ТЕКУЩЕГО
                      аккорда (chord-scale, §7.11), вариация во 2-м такте
p2        bass        16-е, корень аккорда слоем ``(n, n+12)``, MIDI 34..57
p3        pad         трезвучие на аккорд (2 такта), MIDI 50..70, тихо
========  =========  =====================================================

Слой ``perc`` из шаблона матрицы НЕ рендерится — седьмого слота нет.

Две громкостные оси, каждая на своём ключе (оба уже звучат вживую):

* **секции** — ``amp=<ArrangementMatrix.gate_var(слой, уровень)>``
  (``amp=var([...], [...])``; санитайзер ``amp=var`` не капает, поэтому
  уровни здесь сами ≤ :data:`MAX_LAYER_AMP`);
* **пампинг** — ``amplify=[16 значений]`` на плеерах с ``dur=1/4``: просадка
  1:3 ровно на шагах бочки. В renardo_lib ``Players.py`` (метод
  ``send_osc_message`` → ``packet["amp"] = packet["amp"] *
  packet["amplify"]``) ключи ПЕРЕМНОЖАЮТСЯ на каждое событие — гейт секции
  и пампинг не спорят. Список ``amplify`` санитайзер не трогает (капает
  только ``amplify=var(...)``/``amplify=N``), поэтому значения ≤ 0.85 здесь.

ВАЖНО про ``dur`` у ``play()``: в renardo_lib дефолт шага сэмплера —
0.5 доли (``Players.py``: ``self.dur = 0.5 if self.synthdef ==
SamplePlayer``; ``sclang_file_synthdefs.py``: ``play.defaults["dur"]=.5``).
16-шаговая строка без ``dur`` длится ДВА такта восьмыми, а бас/арп с
``dur=1/4`` — один такт 16-ми, и пампинг промахивается мимо бочки. Поэтому
здесь у каждого ``play()`` явно ``dur=1/4`` (прочитано в исходниках, не
прослушано).

Пред-дроп: в блоках, где шаблон выключает бочку (серия неполных блоков
бочки с хотя бы одним пустым, после которой бочка возвращается целиком;
вступление до первого полного блока не считается), на лид ставится
растущий ``hpf=linvar([0, 0, top, 0], [до, длина, 0, после])``. Сегмент
длиной 0 — идиома из ``renardo_lib/demo/10_using_vars.py``
(``hpf=linvar([0,4000],[32,0])``): мгновенный сброс на дропе.
"""

from __future__ import annotations

import random
from typing import Dict, List, Mapping, Sequence, Tuple

from .arrangement_matrix import FULL, SECTION_TEMPLATES, ArrangementMatrix
from .arranger import ALIGN_LEAD_BEATS, BPM_RANGE, VALID_ROOTS, clock_align_prelude

#: Паттерны бочки: 16 шагов = такт 16-ми (при ``dur=1/4``).
KICK_PATTERNS: Dict[str, str] = {
    # «By Design» (Strudel ``x ~ ~ x ~ ~ x ~ ~ [~|x] x ~ ~ ~ ~ ~``)
    "by_design": "X..X..X..(.X)X.....",
    "four_on_floor": "X...X...X...X...",
}

#: Хэты «By Design» (``x*16 | [x!3 ~!2 x!10 ~]`` → чередование дыр).
HATS_PATTERN = "---(-.)(-.)----------(-.)"
#: Клэп ``*`` на 2 и 4 + открытый хэт ``=`` на слабые 8-е — один плеер.
CLAP_PATTERN = "..=.*.=...=.*.=."

SUPPORTED_SCALES: Tuple[str, ...] = ("minor",)

#: Слой матрицы → (слот, синт). ``perc`` не рендерится (нет слота).
LANE_SLOTS: Dict[str, Tuple[str, str]] = {
    "kick": ("d1", "play"),
    "hats": ("d2", "play"),
    "clap": ("d3", "play"),
    "lead": ("p1", "pluck"),
    "bass": ("p2", "bass"),
    "pad": ("p3", "sinepad"),
}

#: Синты, которые использует club (все есть в ``CRITICAL_SYNTHS``).
CLUB_SYNTHS = frozenset({"pluck", "bass", "sinepad"})

#: Потолок уровня одного слоя (тот же, что ``max_amp`` санитайзера).
MAX_LAYER_AMP = 0.85

#: Пампинг: значения ``amplify`` вне/на шаге бочки (1:3, как в эталоне).
PUMP_HIGH = 0.75
PUMP_LOW = 0.25

#: Уровни гейта ``amp=var`` по слоям. Для бас/лида пик = уровень * PUMP_HIGH.
LAYER_LEVELS: Dict[str, float] = {
    "kick": 0.6,
    "hats": 0.14,
    "clap": 0.24,
    "bass": 0.4,
    "lead": 0.28,
    "pad": 0.11,
}

#: Верх растущего hpf на лиде в пред-дропе (Гц), §5 п.2.
PREDROP_HPF_TOP = 1500

STEPS_PER_BAR = 16
BEATS_PER_BAR = 4
CHORD_BARS = 2  # аккорд = 2-тактовый блок матрицы
CHORD_STEPS = STEPS_PER_BAR * CHORD_BARS

#: Регистры (MIDI). Лид: пул пентатоники из 11 нот (2 октавы) от LEAD_LOW —
#: первая нота 58..60, значит верх ≤ 84 (C6, запас до Найквиста 8 кГц).
LEAD_LOW = 58
LEAD_POOL_SIZE = 11
LEAD_TOP_LIMIT = 84
BASS_LOW = 34
PAD_LOW = 50

#: Пентатоника аккорда: минорный — минорная от корня; мажорный — мажорная
#: от корня (= минорная пентатоника его параллельного минора).
PENTATONIC: Dict[str, Tuple[int, ...]] = {
    "m": (0, 3, 5, 7, 10),
    "M": (0, 2, 4, 7, 9),
}
TRIAD: Dict[str, Tuple[int, ...]] = {"m": (0, 3, 7), "M": (0, 4, 7)}

Chord = Tuple[int, str]  # (сдвиг корня от тоники в полутонах, "m"|"M")

#: Банк минорных прогрессий (4 аккорда по 2 такта = 8 тактов).
PROGRESSIONS: Tuple[Tuple[str, Tuple[Chord, ...]], ...] = (
    ("VI-III-VII-i", ((8, "M"), (3, "M"), (10, "M"), (0, "m"))),  # By Design
    ("i-VI-III-VII", ((0, "m"), (8, "M"), (3, "M"), (10, "M"))),
    ("i-VII-VI-VII", ((0, "m"), (10, "M"), (8, "M"), (10, "M"))),
    ("i-iv-VI-v", ((0, "m"), (5, "m"), (8, "M"), (7, "m"))),
    ("VI-VII-i-i", ((8, "M"), (10, "M"), (0, "m"), (0, "m"))),
)


# ---------------------------------------------------------------------------
# Чистые помощники (публичны для тестов)
# ---------------------------------------------------------------------------


def _fmt(value: float) -> str:
    rounded = round(float(value), 3)
    if rounded.is_integer():
        return str(int(rounded))
    return f"{rounded:.3f}".rstrip("0").rstrip(".")


def _fmt_list(values: Sequence[float]) -> str:
    return "[" + ", ".join(_fmt(v) for v in values) + "]"


def kick_steps(pattern: str) -> List[int]:
    """Номера 16-х, где в рисунке бочки есть удар (группа ``(.X)`` — один шаг)."""
    from .renardo_sanitizer import _play_steps

    steps = _play_steps(pattern)
    if steps is None or len(steps) != STEPS_PER_BAR:
        raise ValueError(f"Рисунок бочки должен быть ровно 16 шагов: {pattern!r}")
    return [i for i, step in enumerate(steps) if "X" in step]


def pump_weights(pattern: str) -> List[float]:
    """16 значений ``amplify``: PUMP_LOW на шагах бочки, PUMP_HIGH вне их."""
    hits = set(kick_steps(pattern))
    return [PUMP_LOW if i in hits else PUMP_HIGH for i in range(STEPS_PER_BAR)]


def peak_levels() -> Dict[str, float]:
    """Пиковый уровень каждого слоя (для бас/лида — с учётом пампинга)."""
    peaks = dict(LAYER_LEVELS)
    for lane in ("bass", "lead"):
        peaks[lane] = round(LAYER_LEVELS[lane] * PUMP_HIGH, 4)
    return peaks


def chord_pentatonic(tonic_pc: int, chord: Chord) -> List[int]:
    """Пул нот лида для аккорда: 11 нот пентатоники аккорда от LEAD_LOW вверх."""
    offset, quality = chord
    classes = {(tonic_pc + offset + i) % 12 for i in PENTATONIC[quality]}
    pool: List[int] = []
    note = LEAD_LOW
    while len(pool) < LEAD_POOL_SIZE:
        if note % 12 in classes:
            pool.append(note)
        note += 1
    return pool


def bass_root(tonic_pc: int, chord: Chord) -> int:
    """Корень аккорда в басовом регистре: MIDI 34..45 (слой n+12 ≤ 57)."""
    return BASS_LOW + (tonic_pc + chord[0] - BASS_LOW) % 12


def pad_voicing(tonic_pc: int, chord: Chord) -> Tuple[int, ...]:
    """Трезвучие в основном виде, корень MIDI 50..61 (верх ≤ 68)."""
    root = PAD_LOW + (tonic_pc + chord[0] - PAD_LOW) % 12
    return tuple(root + i for i in TRIAD[chord[1]])


def _walk(rng: random.Random, start: int, length: int) -> List[int]:
    """Случайное блуждание по индексам пула (0..LEAD_POOL_SIZE-1) с отражением."""
    top = LEAD_POOL_SIZE - 1
    out: List[int] = []
    idx = start
    for _ in range(length):
        idx += rng.choice((-2, -1, -1, 0, 1, 1, 2))
        if idx < 0:
            idx = -idx
        if idx > top:
            idx = 2 * top - idx
        out.append(idx)
    return out


def riff_indices(rng: random.Random) -> Tuple[List[int], List[int]]:
    """Риф на 2 такта в индексах пула: (такт темы, такт вариации).

    Тема — 8-шаговая ячейка дважды (второй раз с новым хвостом из 2 шагов);
    вариация — тема с новыми последними 4 шагами (проходящие ноты, §7.11).
    """
    cell = _walk(rng, rng.randint(3, 7), 8)
    theme = cell + cell[:6] + _walk(rng, cell[5], 2)
    variation = theme[:12] + _walk(rng, theme[11], 4)
    return theme, variation


def lead_notes(tonic_pc: int, chords: Sequence[Chord], riff: Tuple[List[int], List[int]]) -> List[int]:
    """Лид на всю прогрессию: риф в пентатонике КАЖДОГО аккорда (chord-scale)."""
    notes: List[int] = []
    for chord in chords:
        pool = chord_pentatonic(tonic_pc, chord)
        for bar in riff:
            notes.extend(pool[i] for i in bar)
    return notes


def predrop_runs(matrix: ArrangementMatrix) -> List[Tuple[int, int]]:
    """Блоки пред-дропа ``[start, end)``: бочка неполная, есть пустой блок,
    до этого бочка уже играла целиком, после — снова целиком."""
    kick = matrix.lanes["kick"]
    runs: List[Tuple[int, int]] = []
    seen_full = False
    start = None
    for i, mask in enumerate(kick):
        if mask == FULL:
            if start is not None and seen_full and 0 in kick[start:i]:
                runs.append((start, i))
            start = None
            seen_full = True
        elif start is None:
            start = i
    return runs


def predrop_hpf(matrix: ArrangementMatrix) -> str:
    """``linvar`` растущего hpf на пред-дропах; ``""`` — пред-дропа нет."""
    runs = predrop_runs(matrix)
    if not runs:
        return ""
    block = matrix.block_beats
    values: List[float] = []
    durs: List[float] = []
    cursor = 0.0
    for start, end in runs:
        begin, finish = start * block, end * block
        if begin > cursor:
            values.append(0)
            durs.append(begin - cursor)
        values.extend([0, PREDROP_HPF_TOP])
        durs.extend([finish - begin, 0])
        cursor = finish
    values.append(0)
    durs.append(matrix.total_beats - cursor)
    return f"linvar({_fmt_list(values)}, {_fmt_list(durs)})"


# ---------------------------------------------------------------------------
# Сборка кода
# ---------------------------------------------------------------------------


def _validate(bpm: float, root: str, scale: str, template: str, kick: str) -> None:
    if isinstance(bpm, bool) or not isinstance(bpm, (int, float)):
        raise ValueError(f"bpm должен быть числом, получено {bpm!r}")
    if not BPM_RANGE[0] <= float(bpm) <= BPM_RANGE[1]:
        raise ValueError(f"bpm вне диапазона {_fmt(BPM_RANGE[0])}..{_fmt(BPM_RANGE[1])}: {bpm}")
    if root not in VALID_ROOTS:
        raise ValueError(f"Неизвестная тоника {root!r} (допустимо: {', '.join(VALID_ROOTS)})")
    if scale not in SUPPORTED_SCALES:
        raise ValueError(
            f"Лад {scale!r} не поддержан клубным режимом (допустимо: {', '.join(SUPPORTED_SCALES)})"
        )
    if template not in SECTION_TEMPLATES:
        raise ValueError(
            f"Неизвестный шаблон секций {template!r} (допустимо: {', '.join(SECTION_TEMPLATES)})"
        )
    if kick not in KICK_PATTERNS:
        raise ValueError(f"Неизвестный рисунок бочки {kick!r} (допустимо: {', '.join(KICK_PATTERNS)})")


def build_matrix(template: str) -> ArrangementMatrix:
    """Матрица шаблона только из тех слоёв, что реально звучат (без ``perc``)."""
    specs: Mapping[str, str] = SECTION_TEMPLATES[template]
    return ArrangementMatrix.from_specs({lane: specs[lane] for lane in LANE_SLOTS})


def _lead_block(notes: List[int]) -> str:
    rows = [", ".join(str(n) for n in notes[i:i + STEPS_PER_BAR]) for i in range(0, len(notes), STEPS_PER_BAR)]
    return "[" + ",\n             ".join(rows) + "]"


def render_club(
    bpm: float = 124,
    root: str = "A#",
    scale: str = "minor",
    template: str = "dj_dave_32",
    seed: int = 0,
    kick: str = "by_design",
    repeat: bool = False,
    align_clock: bool = False,
) -> str:
    """Собрать клубный трек. Одинаковые аргументы → побайтно одинаковый код.

    ``align_clock`` (issue #3112, по умолчанию выкл.): сразу после
    ``Clock.clear()`` — :func:`core.arranger.clock_align_prelude`, чтобы
    форма стартовала с позиции 0 при давно идущем клоке.

    Raises:
        ValueError: неизвестные root/scale/template/kick или bpm вне диапазона.
    """
    _validate(bpm, root, scale, template, kick)
    matrix = build_matrix(template)
    rng = random.Random(seed)
    prog_name, chords = PROGRESSIONS[rng.randrange(len(PROGRESSIONS))]
    riff = riff_indices(rng)
    tonic = VALID_ROOTS.index(root)
    kick_pattern = KICK_PATTERNS[kick]
    pump = _fmt_list(pump_weights(kick_pattern))
    gate = {lane: matrix.gate_var(lane, LAYER_LEVELS[lane]) for lane in LANE_SLOTS}

    bass = " + ".join(
        f"[({n}, {n + 12})] * {CHORD_STEPS}" for n in (bass_root(tonic, c) for c in chords)
    )
    pads = ", ".join("(" + ", ".join(str(n) for n in pad_voicing(tonic, c)) + ")" for c in chords)
    hpf = predrop_hpf(matrix)
    hpf_arg = f" hpf={hpf}," if hpf else ""

    lines = [
        f"# club: {template}, {root} {scale}, {prog_name}, бочка {kick}, seed={seed}",
        *("# " + row for row in matrix.to_text().splitlines()),
        "Clock.clear()",
        *([clock_align_prelude(matrix.total_beats)] if align_clock else []),
        f"Clock.bpm = {_fmt(bpm)}",
        "",
        f'd1 >> play("{kick_pattern}", dur=1/4, amp={gate["kick"]})',
        f'd2 >> play("{HATS_PATTERN}", dur=1/4, hpf=2500, amp={gate["hats"]})',
        f'd3 >> play("{CLAP_PATTERN}", dur=1/4, lpf=3000, room=0.25, amp={gate["clap"]})',
        "",
        f"p1 >> pluck({_lead_block(lead_notes(tonic, chords, riff))},",
        "            dur=1/4, scale=Scale.chromatic, root=0, oct=0, sus=0.15,",
        f"            lpf=linvar([900, 4000], 31),{hpf_arg} room=0.6, mix=0.3,",
        f"            amp={gate['lead']},",
        f"            amplify={pump})",
        "",
        f"p2 >> bass({bass},",
        "           dur=1/4, scale=Scale.chromatic, root=0, oct=0, sus=0.2,",
        "           lpf=linvar([500, 2500], 61), room=0.25,",
        f"           amp={gate['bass']},",
        f"           amplify={pump})",
        "",
        f"p3 >> sinepad([{pads}], dur={CHORD_BARS * BEATS_PER_BAR},",
        "              scale=Scale.chromatic, root=0, oct=0, room=0.7, mix=0.4,",
        f"              amp={gate['pad']})",
    ]
    if not repeat:
        end_beats = matrix.total_beats + (ALIGN_LEAD_BEATS if align_clock else 0)
        lines += ["", f"Clock.future({_fmt(end_beats)}, Clock.clear)"]
    return "\n".join(lines) + "\n"


def club_form_beats(template: str = "dj_dave_32") -> int:
    """Длина одного прохода клубной формы в долях (issue #3112, диагностика фазы)."""
    return int(build_matrix(template).total_beats)


def club_duration_seconds(bpm: float = 124, template: str = "dj_dave_32") -> float:
    """Длительность одного прохода формы (как ``Clock.future`` в коде)."""
    return build_matrix(template).total_beats * 60.0 / float(bpm)


__all__ = [
    "CLUB_SYNTHS",
    "KICK_PATTERNS",
    "LAYER_LEVELS",
    "PROGRESSIONS",
    "build_matrix",
    "chord_pentatonic",
    "club_duration_seconds",
    "club_form_beats",
    "kick_steps",
    "peak_levels",
    "predrop_hpf",
    "predrop_runs",
    "pump_weights",
    "render_club",
]
