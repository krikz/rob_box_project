"""Модель трека аранжировщика v2 и валидатор (ADR-0149 §3.1).

Музыка — данные: ``Track`` неизменяем, длительности в долях такта (никаких
секунд, I22), уровни в дБ — другой тип поля, поэтому «регекс по списку чисел»
невозможен. Валидатор :func:`validate` бросает :class:`TrackError` с путём до
ошибочного поля (``parts.bass.pitches[3].midi``) и причиной.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import FrozenSet, List, Mapping, Optional, Tuple

from . import knowledge as kn
from .tonality import key_fit

BEATS_PER_BAR = 4
STEPS_PER_BAR = 16
BARS_TOTAL = (32, 48, 64)
OUTRO_MIN_BARS = 8
PHRASE_BARS = (8, 16, 32)
APPROACH_MAX_BEATS = 0.5  # хроматический подход баса (ADR-0149 §3.5)
SAMPLE_ROLES = ("sample", "loop", "fx")  # роли, чья партия — файл из ``knowledge.SAMPLE_CATALOG`` (§3.11)


class TrackError(ValueError):
    """Модель трека нарушает инвариант: ``path`` — поле, ``reason`` — почему."""

    def __init__(self, path: str, reason: str) -> None:
        super().__init__(f"{path}: {reason}")
        self.path = path
        self.reason = reason


@dataclass(frozen=True)
class Key:
    root: int  # 0..11, индекс в ``knowledge.ROOTS``
    mode: str  # ключ ``knowledge.SCALES`` (не chromatic)


@dataclass(frozen=True)
class Section:
    name: str
    bars: int
    energy: int  # 0..10
    roles: frozenset  # frozenset[str]: роли, звучащие в секции
    fill_last_bar: bool = False


@dataclass(frozen=True)
class Form:
    sections: Tuple[Section, ...]
    #: ``club`` — клубная форма (32/48/64 такта, outro); ``song`` — classic: мелодия целиком по куплетам (PR-11).
    kind: str = kn.FORM_CLUB

    @property
    def song(self) -> bool:
        return self.kind == kn.FORM_SONG

    @property
    def bars_total(self) -> int:
        return sum(s.bars for s in self.sections)


@dataclass(frozen=True)
class Step:
    on: bool
    accent: int = 0  # 0..3
    offset_ms: int = 0  # свинг


@dataclass(frozen=True)
class Grid:
    steps: Tuple[Step, ...]  # 16 × такты цикла


@dataclass(frozen=True)
class PitchEvent:
    midi: int
    beat: float  # доля от начала трека
    dur_beats: float
    accent: int = 0
    #: Срез фильтра ноты, Гц (``lpf=[...]`` по нотам, ADR-0152 PR-9); 0 — не задан. Вне ``repr``: событие без среза
    #: выглядит как до PR-9 (отпечаток трека и эталон ``test_style_same_tracks`` у басов без ``acid16`` те же).
    lpf: float = field(default=0.0, repr=False)


@dataclass(frozen=True)
class Part:
    role: str
    synth_or_sample: str
    grid: Grid
    pitches: Optional[Tuple[PitchEvent, ...]]  # только у тональных ролей
    level_db: float  # dB RMS в шкале модели громкости (``knowledge.LANE_DB_AT_UNIT``), роль звучит всю секцию
    register: Tuple[int, int]
    sample: int = 0  # номер файла ``play()``-символа ударной роли (бочка — ``knowledge.KICK_SOUNDS``)
    #: Файл каталога на каждое событие роли по кругу (psr-пул DJ_Dave, PR-3d); пусто — ``synth_or_sample``.
    pool: Tuple[str, ...] = ()
    #: Символ ``play()`` ударной роли (бочка рейва — ``A``/``W``, ``knowledge.KICK_SOUNDS``, ADR-0153 S1); пусто —
    #: ``knowledge.DRUM_SYMBOLS`` роли. Вне ``repr``, как ``PitchEvent.lpf``: отпечаток трека клуба тот же, что до S1.
    symbol: str = field(default="", repr=False)

    @property
    def play_symbol(self) -> str:
        return self.symbol or kn.DRUM_SYMBOLS[self.role]


@dataclass(frozen=True)
class Chord:
    degree: int  # ступень лада 0..6
    voicing: Tuple[int, ...]  # MIDI, уже «проведённое» обращение


@dataclass(frozen=True)
class Harmony:
    progression: Mapping[str, Tuple[Chord, ...]]  # имя секции → аккорды по тактам


@dataclass(frozen=True)
class Hook:
    notes: Tuple[PitchEvent, ...]  # доли от начала мотива
    bars: int  # 4..8
    source: Optional[str]  # id мелодии RTTTL или None
    key_fit: Optional[float] = None  # доля длительности хука темы в ладу трека (I12); у мотива лида — None
    #: Ответ хуку из материала партитуры (ADR-0154 Н10, развитие ``rhythm``): ритм хука, высоты следующей фразы; пусто
    #: — ответа нет. Вне ``repr``: хук без ответа выглядит как до PR-4 (эталон ``test_style_same_tracks`` тот же).
    answer: Tuple[PitchEvent, ...] = field(default=(), repr=False)


@dataclass(frozen=True)
class Stereo:
    """Ширина роли (ADR-0149 §3.9): от декоррелированного материала, а не от постоянной панорамы.

    Ударная роль: ``pan`` — вынос, стороны чередуются на каждом ударе, ``first`` — сторона первого (+1 вправо).
    Тональная роль с ``pan`` > 0: два голоса на ``-pan``/``+pan``, второй расстроен на ``detune`` полутона и сдвинут
    на ``haas_ms``. Отсутствие роли в ``Mix.stereo`` — центр.
    """

    pan: float = 0.0  # 0..1
    first: int = 1  # ±1
    detune: float = 0.0  # полутоны
    haas_ms: float = 0.0

    @property
    def voices(self) -> int:
        return 2 if self.pan > 0 and (self.detune or self.haas_ms) else 1


@dataclass(frozen=True)
class Duck:
    """Сайдчейн секции: глубина 0..1 и шаги такта 0..15, от которых считается огибающая (рисунок бочки вида)."""

    depth: float
    trigger: Tuple[int, ...]


#: LPF роли в секции: (срез в начале, срез в конце) Гц; ``0`` — фильтр снят (``knowledge.LPF_OPEN``).
Sweep = Tuple[float, float]


@dataclass(frozen=True)
class Mix:
    level_db: Mapping[str, float]
    stereo: Mapping[str, Stereo]  # роль → ширина; бочки нет (центр), бас — только ширина синта без Хааса (#3458)
    duck: Tuple[Duck, ...] = ()  # по секциям формы (вид секции, ``Style.looks``)
    fx: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)  # имя секции → эффекты
    duck_roles: frozenset = frozenset()  # роли под сайдчейном (тональные)
    lpf: Mapping[str, Tuple[Sweep, ...]] = field(default_factory=dict)  # роль → свип по секциям формы
    #: A9-модель трека (ADR-0152 §4 п.2): поправка уровня роли, дБ (уже в ``level_db``), и доля низа худшего дропа
    #: после неё на роботе (``arrange.mix.a9_model``); ``None`` — песня, без модели.
    a9_trim: Mapping[str, float] = field(default_factory=dict)
    a9_model: Optional[float] = None


@dataclass(frozen=True)
class Transition:
    phrase_bars: int
    bass_swap_bar: int
    filter_in: bool


@dataclass(frozen=True)
class HistoryKey:
    """Оси ``music_history`` трека (ADR-0149 I17); тембры — синты партий (``diversity.track_history``)."""

    kit: str
    progression: str
    hook: Optional[str]
    sample: Optional[str]
    root: int
    hook_fingerprint: Optional[str] = None  # отпечаток фрагмента хука/мотива без транспозиции (#3245)
    fx: Optional[str] = None
    perc: Optional[str] = None  # пул psr-слоя через запятую
    pad_figure: Optional[str] = None  # рисунок пэда (``knowledge.PAD_FIGURES``, ADR-0152 PR-5)
    bass_figure: Optional[str] = None  # рисунок баса (``knowledge.BASS_FIGURES``, ADR-0152 PR-6)
    template: Optional[str] = None  # шаблон формы (``Style.forms``, ADR-0152 PR-7)
    genre: Optional[str] = None  # жанровое окно сета (``Style.genre_windows``, ADR-0152 PR-8)
    #: Ключ ``knowledge.STYLES`` трека (ADR-0153 S1): по нему валидатор берёт регистры, сайдчейн и пул бочек.
    #: Вне ``repr``, как ``PitchEvent.lpf``: модель трека клуба выглядит как до S1 (эталон ``test_style_same_tracks``).
    style: str = field(default=kn.DEFAULT_STYLE, repr=False)
    #: Семья тембров сета (``SetPlan.family``, #3460): ось «между сетами» для тем вне таблицы. Вне ``repr``, как ``style``.
    timbre: Optional[str] = field(default=None, repr=False)


@dataclass(frozen=True)
class Track:
    track_id: str
    seed: int
    bpm: int
    key: Key
    form: Form
    parts: Mapping[str, Part]
    harmony: Harmony
    hook: Optional[Hook]
    mix: Mix
    energy: int  # 1..5
    transition_in: Transition
    transition_out: Transition
    history_key: HistoryKey

    @property
    def style(self) -> str:
        """Ключ ``knowledge.STYLES`` трека (один стиль на сет, ADR-0153 §4.3)."""
        return self.history_key.style


def _require(ok: bool, path: str, reason: str) -> None:
    if not ok:
        raise TrackError(path, reason)


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _check_key(track: Track) -> None:
    key = track.key
    _require(isinstance(key.root, int) and 0 <= key.root <= 11, "key.root", f"тоника {key.root!r} вне 0..11")
    _require(key.mode in kn.SCALES and key.mode != kn.CHROMATIC, "key.mode", f"лад {key.mode!r} не из knowledge.SCALES")
    lo, hi = kn.BPM_RANGE
    _require(isinstance(track.bpm, int) and lo <= track.bpm <= hi, "bpm", f"bpm {track.bpm!r} вне {lo}..{hi}")
    _require(bool(track.track_id), "track_id", "пустой id")
    _require(track.energy in kn.ENERGY_LEVELS, "energy", f"энергия {track.energy!r} вне 1..5")


def _check_song_form(form: Form) -> None:
    """Песня: длина — от мелодии (куплет = вся мелодия), outro не нужен — песня не стыкуется в сете."""
    _require(0 < form.bars_total <= kn.SONG_MAX_BARS, "form.bars_total",
             f"{form.bars_total} тактов вне 1..{kn.SONG_MAX_BARS}")
    for i, sec in enumerate(form.sections):
        _require(sec.bars > 0 and 0 <= sec.energy <= 10, f"form.sections[{i}]", "такты ≤ 0 или энергия вне 0..10")
        unknown = sorted(set(sec.roles) - set(kn.ROLES))
        _require(not unknown, f"form.sections[{i}].roles", f"неизвестные роли {unknown}")


def _check_form(form: Form) -> None:
    _require(bool(form.sections), "form.sections", "нет секций")
    _require(form.kind in (kn.FORM_CLUB, kn.FORM_SONG), "form.kind", f"форма {form.kind!r} не club/song")
    if form.song:
        _check_song_form(form)
        return
    _require(form.bars_total in BARS_TOTAL, "form.bars_total", f"{form.bars_total} тактов не из {BARS_TOTAL}")
    for i, sec in enumerate(form.sections):
        path = f"form.sections[{i}]"
        _require(sec.bars > 0 and sec.bars % 4 == 0, f"{path}.bars", f"{sec.bars} тактов: нужно кратное 4")
        _require(0 <= sec.energy <= 10, f"{path}.energy", f"энергия {sec.energy} вне 0..10")
        unknown = sorted(set(sec.roles) - set(kn.ROLES))
        _require(not unknown, f"{path}.roles", f"неизвестные роли {unknown}")
    outro = _outro(form)
    _require(sum(s.bars for s in outro) >= OUTRO_MIN_BARS, "form.sections[-1]",
             f"конец формы — outro ≥ {OUTRO_MIN_BARS} тактов (DJ-friendly)")
    _require(all("lead" not in s.roles for s in outro), "form.sections[-1].roles", "в outro нет лида")


def _outro(form: Form) -> Tuple[Section, ...]:
    """Хвост формы из секций ``outro*`` (outro может делиться под своп баса блэнда)."""
    tail: List[Section] = []
    for sec in reversed(form.sections):
        if not sec.name.startswith("outro"):
            break
        tail.insert(0, sec)
    return tuple(tail)


def _check_grid(role: str, grid: Grid, bars_total: int) -> None:
    n = len(grid.steps)
    path = f"parts.{role}.grid.steps"
    _require(n > 0 and n % STEPS_PER_BAR == 0, path, f"длина {n} не кратна такту ({STEPS_PER_BAR})")
    _require((STEPS_PER_BAR * bars_total) % n == 0, path, f"длина {n} не делит форму ({bars_total} тактов)")
    if all(0 <= st.accent <= 3 and isinstance(st.offset_ms, int) for st in grid.steps):
        return  # без строк причин на каждом шаге (PR #3501: время валидатора)
    for i, st in enumerate(grid.steps):
        _require(0 <= st.accent <= 3, f"{path}[{i}].accent", f"акцент {st.accent} вне 0..3")
        _require(isinstance(st.offset_ms, int), f"{path}[{i}].offset_ms", "сдвиг не в целых мс")


def _pitch_ok(role: str, ev: PitchEvent, key: Key, limit_beats: float, part: Part, song: bool) -> bool:
    """Все условия :func:`_check_pitch` разом, без сборки сообщений: валидатор проходит ~10⁴ нот на трек, и строки
    причин на каждой ноте занимали заметную долю времени ``render`` (CI-таймаут пакета, PR #3501)."""
    lo, hi = part.register
    lpf_lo, lpf_hi = kn.LPF_RANGE_HZ
    if not (_finite(ev.beat) and _finite(ev.dur_beats) and ev.dur_beats > 0 and 0 <= ev.beat
            and ev.beat + ev.dur_beats <= limit_beats + 1e-9 and (ev.lpf == kn.LPF_OPEN or lpf_lo <= ev.lpf <= lpf_hi)
            and lo <= ev.midi <= hi):
        return False
    return (song or role == "lead" or ev.midi % 12 in kn.scale_pitch_classes(key.root, key.mode)
            or (role == "bass" and ev.dur_beats <= APPROACH_MAX_BEATS))


def _check_pitch(role: str, i: int, ev: PitchEvent, key: Key, limit_beats: float, part: Part, song: bool) -> None:
    if _pitch_ok(role, ev, key, limit_beats, part, song):
        return
    path = f"parts.{role}.pitches[{i}]"
    _require(_finite(ev.beat) and _finite(ev.dur_beats), path, "доли не конечные числа")
    _require(ev.dur_beats > 0 and 0 <= ev.beat and ev.beat + ev.dur_beats <= limit_beats + 1e-9,
             f"{path}.beat", f"нота {ev.beat}+{ev.dur_beats} вне формы ({limit_beats} долей)")
    _require(ev.lpf == kn.LPF_OPEN or kn.LPF_RANGE_HZ[0] <= ev.lpf <= kn.LPF_RANGE_HZ[1], f"{path}.lpf",
             f"срез ноты {ev.lpf} не 0 и не в {kn.LPF_RANGE_HZ[0]:g}..{kn.LPF_RANGE_HZ[1]:g} Гц")
    lo, hi = part.register
    _require(lo <= ev.midi <= hi, f"{path}.midi", f"MIDI {ev.midi} вне регистра партии {lo}..{hi}")
    if song or role == "lead":  # мелодия/хук темы — с хроматикой; лад — долей длительности (``_check_key_fit``)
        return
    in_scale = ev.midi % 12 in kn.scale_pitch_classes(key.root, key.mode)
    approach = role == "bass" and ev.dur_beats <= APPROACH_MAX_BEATS
    _require(in_scale or approach, f"{path}.midi", f"MIDI {ev.midi} не в ладе {kn.ROOTS[key.root]} {key.mode}")


def _check_key_fit(role: str, part: Part, key: Key, minimum: float) -> None:
    """Партия в тональности трека долей длительности в ладу ≥ ``minimum`` (``tonality.key_fit``): аккомпанемент
    песни — ``knowledge.SONG_KEY_FIT_MIN``, лид клубного трека (хук темы с проходящими) — ``HOOK_KEY_FIT_MIN``."""
    fit = key_fit([(ev.midi, ev.dur_beats) for ev in part.pitches or ()], kn.ROOTS[key.root], key.mode)
    _require(fit >= minimum, f"parts.{role}.pitches",
             f"в ладу {kn.ROOTS[key.root]} {key.mode} {fit:.2f} длительности < {minimum}")


def _check_tonal(role: str, part: Part, track: Track) -> None:
    song = track.form.song
    corridor = (kn.SONG_REGISTERS if song else kn.STYLES[track.style].registers)[role]
    lo, hi = part.register
    _require(corridor[0] <= lo <= hi <= corridor[1], f"parts.{role}.register",
             f"регистр {part.register} вне коридора {corridor}")
    _require(song or hi <= kn.LEAD_MAX_MIDI, f"parts.{role}.register", f"верх {hi} выше {kn.LEAD_MAX_MIDI}")
    _require(bool(part.pitches), f"parts.{role}.pitches", "тональная партия без нот")
    limit = float(track.form.bars_total * BEATS_PER_BAR)
    for i, ev in enumerate(part.pitches or ()):
        _check_pitch(role, i, ev, track.key, limit, part, song)
    if role == "lead" and not song:
        _check_key_fit(role, part, track.key, kn.HOOK_KEY_FIT_MIN)
    elif song and role != "lead":
        _check_key_fit(role, part, track.key, kn.SONG_KEY_FIT_MIN)
    palette = kn.SYNTH_PALETTE.get(role, ())
    _require(part.synth_or_sample in palette or part.synth_or_sample in kn.SYNTH_TRAITS,
             f"parts.{role}.synth_or_sample", f"синт {part.synth_or_sample!r} не из палитры")


def _check_parts(track: Track) -> None:
    for role, part in track.parts.items():
        _require(role in kn.ROLES and part.role == role, f"parts.{role}.role", "роль не совпадает с ключом")
        _check_grid(role, part.grid, track.form.bars_total)
        if role in kn.TONAL_ROLES:
            _check_tonal(role, part, track)
        else:
            _require(part.pitches is None, f"parts.{role}.pitches", "у ударной роли нет высот")
            _require(isinstance(part.sample, int) and part.sample >= 0, f"parts.{role}.sample", "номер сэмпла < 0")
            if role == "kick":
                _check_kick(track, part)
        if role in SAMPLE_ROLES:
            unknown = sorted({part.synth_or_sample, *part.pool} - set(kn.SAMPLE_CATALOG))
            _require(not unknown, f"parts.{role}.synth_or_sample", f"сэмплов {unknown} нет в knowledge.SAMPLE_CATALOG")
        _require(_finite(part.level_db) and part.level_db <= kn.role_ceiling(role), f"parts.{role}.level_db",
                 f"{part.level_db} дБ выше потолка роли {kn.role_ceiling(role)}")
        _require(track.mix.level_db.get(role) == part.level_db, f"mix.level_db.{role}",
                 "уровень в Mix расходится с партией (одна ось громкости)")
    for i, sec in enumerate(track.form.sections):
        missing = sorted(set(sec.roles) - set(track.parts))
        _require(not missing, f"form.sections[{i}].roles", f"роли без партии: {missing}")


def _check_kick(track: Track, part: Part) -> None:
    """Бочка — из ``knowledge.KICK_SOUNDS`` (символ и файл) и из пулов окон стиля трека."""
    name = kn.kick_of(part.play_symbol, part.sample)
    _require(name is not None, "parts.kick.sample", "бочка не из knowledge.KICK_SOUNDS")
    pools = {k for w in kn.STYLES[track.style].genre_windows.values() for k in w.kick_pool}
    _require(name in pools, "parts.kick.sample", f"бочка {name!r} не из пула стиля {track.style!r}")


def _check_duck(mix: Mix, parts: Mapping[str, Part], n_sections: int, style: kn.Style) -> None:
    roles = set(mix.duck_roles)
    _require(roles <= set(parts) & set(style.duck_roles), "mix.duck_roles",
             f"сайдчейн только на ролях Style.duck_roles, что есть в треке: {sorted(roles)}")
    _require(not roles or len(mix.duck) == n_sections, "mix.duck", f"{len(mix.duck)} секций, а в форме {n_sections}")
    for i, duck in enumerate(mix.duck):
        trigger = duck.trigger
        _require(0.0 <= duck.depth <= 1.0, f"mix.duck[{i}].depth", "глубина сайдчейна вне 0..1")
        _require(all(isinstance(s, int) and 0 <= s < STEPS_PER_BAR for s in trigger)
                 and list(trigger) == sorted(set(trigger)), f"mix.duck[{i}].trigger",
                 f"шаги {trigger} не 0..15 по возрастанию")
        _require(not duck.depth or bool(trigger), f"mix.duck[{i}].trigger", "сайдчейн без триггера")


def _check_lpf(mix: Mix, parts: Mapping[str, Part], n_sections: int) -> None:
    lo, hi = kn.LPF_RANGE_HZ
    for role, sweeps in mix.lpf.items():
        path = f"mix.lpf.{role}"
        _require(role in parts and role != "kick", path, "свип только на партиях трека, бочка без фильтра")
        _require(len(sweeps) == n_sections, path, f"{len(sweeps)} секций, а в форме {n_sections}")
        for i, sweep in enumerate(sweeps):
            _require(all(hz == kn.LPF_OPEN or lo <= hz <= hi for hz in sweep), f"{path}[{i}]",
                     f"срез {sweep} не 0 и не в {lo:g}..{hi:g} Гц")


def _check_levels(track: Track) -> None:
    limit = 10.0 ** (kn.LEVEL_CEILINGS["master_peak_db"] / 10.0)
    for i, sec in enumerate(track.form.sections):
        power = sum(10.0 ** (track.parts[r].level_db / 10.0) for r in sec.roles)
        _require(power <= limit, f"form.sections[{i}].roles",
                 f"сумма пиков {10 * math.log10(power):.1f} дБ выше потолка {kn.LEVEL_CEILINGS['master_peak_db']}")
    _check_duck(track.mix, track.parts, len(track.form.sections), kn.STYLES[track.style])
    _check_lpf(track.mix, track.parts, len(track.form.sections))
    for role, st in track.mix.stereo.items():
        path = f"mix.stereo.{role}"
        _require(role != "kick" and (role != "bass" or _bass_width_ok(track.parts[role], st)), path,
                 "низ — строго в центре (ADR-0149 §3.9; бас — только ширина синта без Хааса, knowledge.SYNTH_STEREO)")
        _require(0.0 <= st.pan <= 1.0 and st.first in (-1, 1), path, f"вынос {st.pan} вне 0..1 или сторона {st.first}")
        _require(0.0 <= st.haas_ms <= kn.HAAS_MAX_MS and 0.0 <= st.detune <= 0.5, path,
                 f"Хаас {st.haas_ms} мс или расстройка {st.detune} вне пределов")


def _bass_width_ok(part: Part, st: Stereo) -> bool:
    """Бас не в центре — только ширина своего синта из ``knowledge.SYNTH_STEREO`` и без Хааса (низ двух голосов в
    фазе: гребёнка задержки на басу вырезала бы низ, #3458)."""
    table = kn.SYNTH_STEREO.get(part.synth_or_sample)
    return table is not None and not st.haas_ms and st == Stereo(**table)


def _check_hook_and_harmony(track: Track) -> None:
    names = {s.name for s in track.form.sections}
    for name, chords in track.harmony.progression.items():
        _require(name in names, f"harmony.progression.{name}", "секции нет в форме")
        for i, chord in enumerate(chords):
            _require(0 <= chord.degree <= 6 and bool(chord.voicing), f"harmony.progression.{name}[{i}]",
                     "ступень вне 0..6 или пустое обращение")
    hook = track.hook
    if hook is None:
        return
    lo, hi = (1, kn.SONG_MAX_BARS) if track.form.song else (4, 8)  # хук песни — вся мелодия
    _require(lo <= hook.bars <= hi, "hook.bars", f"хук {hook.bars} тактов вне {lo}..{hi}")
    for i, ev in enumerate(hook.notes):
        _require(ev.dur_beats > 0 and 0 <= ev.beat and ev.beat + ev.dur_beats <= hook.bars * BEATS_PER_BAR + 1e-9,
                 f"hook.notes[{i}].beat", "нота вне длины мотива")


def _check_transitions(track: Track) -> None:
    for name in ("transition_in", "transition_out"):
        tr = getattr(track, name)
        _require(tr.phrase_bars in PHRASE_BARS, f"{name}.phrase_bars", f"фраза {tr.phrase_bars} не из {PHRASE_BARS}")
        _require(0 <= tr.bass_swap_bar < tr.phrase_bars, f"{name}.bass_swap_bar", "своп баса вне фразы")


def validate(track: Track) -> None:
    """Проверить инварианты трека; нарушение — :class:`TrackError` (путь + причина)."""
    _require(track.style in kn.STYLES, "history_key.style", f"стиль {track.style!r} не из knowledge.STYLES")
    _check_key(track)
    _check_form(track.form)
    _require(track.history_key.template in (None, *kn.FORM_NAMES), "history_key.template",
             f"форма {track.history_key.template!r} не из Style.forms")
    _require(track.history_key.genre in (None, *kn.GENRE_NAMES), "history_key.genre",
             f"окно {track.history_key.genre!r} не из Style.genre_windows")
    _check_parts(track)
    _check_levels(track)
    _check_hook_and_harmony(track)
    _check_transitions(track)


# ── Блэнд двух треков (ADR-0149 §3.12, §7.2 A4; PR-8) ─────────────────────────────────────────────────────────

#: Длина блэнда в тактах (4–8): входящий трек звучит поверх хвоста уходящего.
BLEND_BARS = (4, 8)
#: Роли, которых в каждый такт блэнда ровно одна на две деки: никогда двух и никогда ни одной.
BLEND_SINGLE_ROLES = ("kick", "bass")


def roles_at_bar(track: Track, bar: int) -> FrozenSet[str]:
    """Роли, заказанные формой на такте ``bar`` (0 — первый такт формы)."""
    start = 0
    for sec in track.form.sections:
        if start <= bar < start + sec.bars:
            return frozenset(sec.roles)
        start += sec.bars
    raise IndexError(f"такт {bar} вне формы ({track.form.bars_total} тактов)")


def blend_bars(leaving: Track, incoming: Track) -> int:
    """На сколько тактов ``incoming`` входит до конца формы ``leaving``; 0 — блэнд невозможен (стык встык).

    Блэнд — свойство пары форм, а не движка: длина — ``leaving.transition_out.phrase_bars`` (должна совпасть с
    ``incoming.transition_in``), темп один, в каждом такте блэнда оба трека звучат, ровно одна бочка и один бас
    на две деки (своп на ``bass_swap_bar``: бас и бочка уходящего гаснут ровно там, где входят у входящего),
    в хвосте уходящего нет лида.
    """
    out, inn = leaving.transition_out, incoming.transition_in
    bars = out.phrase_bars
    if (bars, out.bass_swap_bar) != (inn.phrase_bars, inn.bass_swap_bar) or leaving.bpm != incoming.bpm:
        return 0
    if not BLEND_BARS[0] <= bars <= BLEND_BARS[1] or bars > min(leaving.form.bars_total, incoming.form.bars_total):
        return 0
    tail = leaving.form.bars_total - bars
    for bar in range(bars):
        old, new = roles_at_bar(leaving, tail + bar), roles_at_bar(incoming, bar)
        if not old or not new or "lead" in old:
            return 0
        if any((role in old) + (role in new) != 1 for role in BLEND_SINGLE_ROLES):
            return 0
    return bars


__all__ = [
    "BLEND_BARS", "BLEND_SINGLE_ROLES", "Chord", "Duck", "Form", "Grid", "Harmony", "HistoryKey", "Hook", "Key", "Mix",
    "Part", "PitchEvent", "Section", "Step", "Stereo", "Sweep", "Track", "TrackError", "Transition", "blend_bars",
    "roles_at_bar", "validate",
]
