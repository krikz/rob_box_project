"""Одна таблица знания аранжировщика v2 (ADR-0149 §2.1, §2.2; ADR-0148).

Лады, тоники, роли, палитра синтов и их свойства, коридоры регистров, потолки
уровней, жанровые окна. Остальные модули пакета только импортируют отсюда.

Значения скопированы из старого кода (он не правится, ADR-0149 §9: старый путь
только удаляют), потому что старые модули лежат в ``rob_box_mcp_tools`` и
``rob_box_voice`` и тянут за собой ROS-окружение. Расхождение с источником
ловит ``test/test_knowledge_legacy.py`` — пока старый код жив.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Tuple

#: Тоники по высоте звука, индекс = pitch class 0..11 (``Root.default`` Renardo).
ROOTS: Tuple[str, ...] = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
CHROMATIC = "chromatic"

#: Интервалы ладов Renardo (``Scale.py``; источник — ``core/renardo_events.SCALES``).
SCALES: Mapping[str, Tuple[int, ...]] = {
    CHROMATIC: tuple(range(12)),
    "minor": (0, 2, 3, 5, 7, 8, 10),
    "major": (0, 2, 4, 5, 7, 9, 11),
    "dorian": (0, 2, 3, 5, 7, 9, 10),
    "phrygian": (0, 1, 3, 5, 7, 8, 10),
    "lydian": (0, 2, 4, 6, 7, 9, 11),
    "mixolydian": (0, 2, 4, 5, 7, 9, 10),
    "harmonicMinor": (0, 2, 3, 5, 7, 8, 11),
    "majorPentatonic": (0, 2, 4, 7, 9),
    "minorPentatonic": (0, 3, 5, 7, 10),
}

#: Роли партий трека (ADR-0149 §3.1); ритмические — сетка, тональные — высоты.
ROLES: Tuple[str, ...] = ("kick", "hats", "clap", "perc", "bass", "pad", "lead", "sample", "fx")
TONAL_ROLES: Tuple[str, ...] = ("bass", "pad", "lead")

#: Коридоры регистров MIDI (I13): бас < пэд < лид ≤ 88 (``harmonize.BASS_MIDI_FLOOR``,
#: ``rtttl_compose._LEAD_MAX_CEILING``, ADR-0149 §3.5–§3.7).
REGISTERS: Mapping[str, Tuple[int, int]] = {"bass": (36, 52), "pad": (50, 70), "lead": (58, 84)}
LEAD_MAX_MIDI = 88

#: Ступени энергии трека в сете (ADR-0147): 1 — интро/спад, 5 — пик.
ENERGY_LEVELS: Tuple[int, ...] = (1, 2, 3, 4, 5)
#: Смещение громкости трека по энергии, дБ, ПОСЛЕ динамики мастер-шины (ADR-0147 §3.2; проводка — PR-7).
#: Источник для старого ``core/club_energy`` (там импорт отсюда, ADR-0149 §4.6 в).
ENERGY_TRIM_DB: Mapping[int, float] = {1: -9.0, 2: -6.0, 3: -4.0, 4: -2.0, 5: 0.0}
#: Волна энергии открытого сета по номеру трека (ADR-0147 §3.4, ADR-0149 §4.6): трек 1 — превью/интро (2),
#: дальше период 5 «разгон → пик → спад»: 2 3 4 5 4 | 2 3 4 5 4 | …
ENERGY_WAVE: Tuple[int, ...] = (2, 3, 4, 5, 4)
#: Настроение одиночного трека (``request_music.mood``, ADR-0149 §5.1) → энергия; перечисление тула — ключи.
MOOD_ENERGY: Mapping[str, int] = {"calm": 2, "groove": 3, "bright": 3, "playful": 3, "dark": 4, "epic": 5}
#: Плотность по энергии — составом ролей секций, а не множителями уровней (ADR-0149 §4.6 б): роли, которых
#: нет в треке такой энергии. Бочку и бас энергия не снимает (ADR-0147 §3.3).
ENERGY_THIN_ROLES: Mapping[int, Tuple[str, ...]] = {1: ("clap",), 2: ("clap",)}

#: Потолки уровней, дБ пика (К7). ``master_peak_db`` — потолок суммы одновременно
#: звучащих ролей секции; остальное — потолок одной роли. Стартовые значения
#: ADR-0149 §3.10 (эталон ``ref_dj_16k``: пик −9 дБFS), не замер: уточняет PR-7.
LEVEL_CEILINGS: Mapping[str, float] = {
    "master_peak_db": -3.0,
    "kick": -6.0, "bass": -8.0, "pad": -12.0, "lead": -10.0,
    "hats": -14.0, "clap": -10.0, "perc": -14.0, "sample": -10.0, "fx": -14.0,
}

#: Рисунки бочки, 16 шагов (``core/club_arranger.KICK_PATTERNS``).
KICK_PATTERNS: Mapping[str, str] = {
    "by_design": "X..X..X..(.X)X.....",
    "four_on_floor": "X...X...X...X...",
    "half_time": "X.........X.....",
    "breakbeat": "X.X.......X..X..",
    "outrun": "X...X...X...X..X",
}


@dataclass(frozen=True)
class GenreWindow:
    """Жанровое окно: темп, бочка по умолчанию, допустимые лады."""

    bpm: Tuple[int, int]
    kick: str
    scales: Tuple[str, ...]
    #: Свинг нечётных 16-х хэтов — доля восьмой (ADR-0149 §3.4: 5–10 %); величину на сет выбирает план.
    swing: Tuple[float, float] = (0.05, 0.10)


#: club 128–138 — решение Шифу 01.10 (ADR-0149 §12 В6, эталон живого диджея ~138).
GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "club": GenreWindow((128, 138), "four_on_floor", ("minor", "dorian", "phrygian", "major")),
}
BPM_RANGE = (60, 180)  # ``arranger.BPM_RANGE``: за пределами Renardo-тракт не принимает темп.


@dataclass(frozen=True)
class ThemeRow:
    """Строка закрытой таблицы тем (ADR-0149 §4.7): основы слов, окно темпа, лад плана, хуки темы.

    ``hooks`` — ``name`` мелодий ЛОКАЛЬНОЙ RTTTL-библиотеки (архив ``rtttl_melodies.jsonl.gz``):
    поиск по русским словам темы в латинском архиве не находит ничего (тема → хук 0/14, эпик #3312),
    поэтому тема указывает мелодии явно. Каждая годится в хук ``arrange.hook.from_rtttl``
    (``test_theme.py`` сверяет с архивом).
    """

    stems: Tuple[str, ...]
    bpm: Tuple[int, int]
    mode: str
    hooks: Tuple[str, ...]


#: Темы сета → материал. Порядок строк решает ничью по числу совпавших основ.
THEMES: Mapping[str, ThemeRow] = {
    "space": ThemeRow(
        ("косм", "звёзд", "звезд", "галакт", "планет", "ракет", "луна", "марс", "space", "star"), (128, 132), "minor",
        ("starwars_6", "starwars_2", "the_x_files", "xfiles1", "startrek_3", "startrek_5",
         "deepspac", "spacecha", "doctorwh")),
    "cyber": ThemeRow(
        ("кибер", "робот", "матриц", "неон", "техно", "будущ", "cyber", "robot"), (134, 138), "phrygian",
        ("axelf_3", "axel_f", "robotroc", "robot", "popcorn", "popcorn_6", "aroundth_3")),
    "kids": ThemeRow(
        ("детск", "дети", "детей", "праздн", "рожден", "мульт", "игрушк", "kids"), (128, 132), "major",
        ("happybir", "chickend", "macarena", "yellowsu", "teletubb", "bobthebu", "spongebo", "flintsto_2",
         "barbiegi")),
    "slavic": ThemeRow(
        ("славян", "русск", "народн", "калинк", "балалайк", "деревен", "казач"), (130, 136), "minor",
        ("kalinkav", "kalinkav_2", "tetris", "tetris_2")),
    "winter": ThemeRow(
        ("новогод", "новый", "зим", "рождеств", "ёлк", "елк", "снег", "мороз"), (128, 132), "major",
        ("jinglebe_6", "jinglebe_3", "lastchri_5", "haveyour")),
}
#: Хуки без темы (тема не из таблицы): узнаваемые мелодии, тоже из локальной библиотеки.
DEFAULT_HOOKS: Tuple[str, ...] = ("tetris", "axelf_3", "popcorn", "macarena", "nokiatun_2", "aroundth_3", "robot")


@dataclass(frozen=True)
class SynthTraits:
    """Свойства SynthDef: ``tail`` — held|fixed|short, ``register`` — роль, ``tail_note`` — текст."""

    tail: str
    register: str
    tail_note: str

    @property
    def long_release(self) -> bool:
        return self.tail != "short"


#: Свойства синтов (``core/synth_traits.SYNTH_TRAITS``; без поля ``source`` — оно в старом файле).
SYNTH_TRAITS: Mapping[str, SynthTraits] = {
    "imperialbrass": SynthTraits("held", "lead", "≈1.5 с"),
    "supersawlead": SynthTraits("held", "lead", ""),
    "strangerbrass": SynthTraits("held", "lead", ""),
    "strangerarp": SynthTraits("held", "lead", ""),
    "marchstrings": SynthTraits("held", "pad", ""),
    "warmpad": SynthTraits("fixed", "pad", "1.2 с"),
    "strangerpulsepad": SynthTraits("fixed", "pad", "1.6 с"),
    "retrobass": SynthTraits("short", "bass", ""),
    "brass": SynthTraits("short", "lead", ""),
    "organ": SynthTraits("short", "lead", ""),
    "tb303": SynthTraits("short", "bass", ""),
}

#: Палитра синтов роли: пул сида (``club_arranger.ROLE_SYNTHS``) + явные тембры темы
#: (``club_timbre.TIMBRE_EXTRAS``). Первый — эталонный.
SYNTH_PALETTE: Mapping[str, Tuple[str, ...]] = {
    "lead": ("pluck", "blip", "arpy", "karp", "marimba", "sitar", "epiano", "brass", "orient", "viola"),
    "bass": ("bass", "retrobass", "dub"),
    "pad": ("sinepad", "warmpad", "space", "ambi", "strangerpulsepad"),
}

#: Рисунок хэтов/клэпа — это сэмпл-плееры ``play``; синт ударных один.
PLAY_SYNTH = "play"

#: Символ сэмпла ``play()`` ударной роли (Renardo: X — бочка, - — хэт, * — клэп, o — малый).
DRUM_SYMBOLS: Mapping[str, str] = {"kick": "X", "hats": "-", "clap": "*", "perc": "o"}

#: Слоты плееров деки (ADR-0149 §3.2): три ударных и три тональных. На роботе 01.10 звучат и
#: ``d4..p6``, и ``a1..b3`` (PR-2); дека B — ``a1..b3``, потому что ``d4..p6`` санитайзер v1 ещё
#: переставляет в ``d1..p3`` (#1804), а v2 и v1 пока делят один Renardo.
DECK_SLOTS: Mapping[str, Tuple[str, ...]] = {
    "A": ("d1", "d2", "d3", "p1", "p2", "p3"),
    "B": ("a1", "a2", "a3", "b1", "b2", "b3"),
}

#: Акцент шага сетки 0..3 → множитель ``amplify`` (ADR-0149 §3.4: сильные доли громче).
ACCENT_AMPLIFY: Tuple[float, ...] = (0.4, 0.6, 0.8, 1.0)


def scale_pitch_classes(root: int, mode: str) -> frozenset:
    """Множество pitch class лада от тоники ``root`` (0..11)."""
    return frozenset((root + step) % 12 for step in SCALES[mode])


def traits_of(synth: str) -> "SynthTraits | None":
    return SYNTH_TRAITS.get(synth.strip().lower()) if synth else None


def role_ceiling(role: str) -> float:
    return LEVEL_CEILINGS[role]


__all__ = [
    "ACCENT_AMPLIFY", "BPM_RANGE", "CHROMATIC", "DECK_SLOTS", "DRUM_SYMBOLS", "ENERGY_LEVELS", "ENERGY_TRIM_DB",
    "DEFAULT_HOOKS", "ENERGY_THIN_ROLES", "ENERGY_WAVE", "GENRE_WINDOWS", "GenreWindow", "THEMES", "ThemeRow",
    "KICK_PATTERNS", "LEAD_MAX_MIDI", "LEVEL_CEILINGS", "MOOD_ENERGY", "PLAY_SYNTH", "REGISTERS", "ROLES", "ROOTS",
    "SCALES", "SYNTH_PALETTE", "SYNTH_TRAITS", "SynthTraits", "TONAL_ROLES", "role_ceiling",
    "scale_pitch_classes", "traits_of",
]


# ── Микс (PR-3c, ADR-0149 §3.8, §3.10, §8.1): громкость слоёв, уровни ролей, сайдчейн, тембры, бочка ────────

#: Модель громкости слоя — перенос из ``core/club_loudness`` (старый модуль теперь импортирует отсюда).
#: dB RMS слоя, звучащего весь блок, при уровне замера: ``{слой: (уровень, {вариант: dB})}``. Ключ ударных —
#: рисунок, клэп один, тональных — синт. Это МОДЕЛЬ (офлайн-рендер), не замер на роботе: :data:`LOUDNESS_SOURCE`.
LOUDNESS_SOURCE = (
    "офлайн-рендер scsynth 3.14.1 NRT 16 кГц, renardo_lib 0.9.13, masterfilter gain=0.5 dyn=0, "
    "dj_dave_32 блоки 8-10, 124 BPM, среднее по 5 тоникам (29.09.2026); не замер на роботе"
)
LAYER_MEASURED_DB: Mapping[str, Tuple[float, Mapping[str, float]]] = {
    "kick": (0.6, {"by_design": -24.1, "four_on_floor": -24.6, "half_time": -27.6,
                   "breakbeat": -24.6, "outrun": -23.7}),
    "hats": (0.14, {"by_design": -68.4, "offbeat": -74.0, "eighths": -70.9, "shuffle": -70.0}),
    "clap": (0.24, {"clap": -55.7}),
    "lead": (0.28, {"pluck": -53.4, "blip": -55.0, "arpy": -56.3, "karp": -66.3, "marimba": -68.9}),
    "bass": (0.4, {"bass": -24.4, "retrobass": -41.9, "dub": -24.8}),
    "pad": (0.11, {"sinepad": -74.4, "warmpad": -43.1, "space": -54.5}),
}
#: Наклон громкости по ``amp``: ``dB = 20·p·log10(amp)``; у ``dub``/``karp``/``sinepad`` ``amp`` входит дважды.
AMP_EXPONENT: Mapping[str, float] = {"dub": 2.0, "karp": 2.0, "sinepad": 2.0}
#: dB RMS слоя при ``amp`` 1.0 — пересчёт замера: ``dB − 20·p·log10(уровень)``.
LANE_DB_AT_UNIT: Mapping[str, Mapping[str, float]] = {
    lane: {opt: round(db - 20.0 * AMP_EXPONENT.get(opt, 1.0) * math.log10(level), 2) for opt, db in table.items()}
    for lane, (level, table) in LAYER_MEASURED_DB.items()
}
#: Потолок ``amp`` одного слоя (``max_amp`` санитайзера v1; ``club_arranger.MAX_LAYER_AMP`` импортирует отсюда).
MAX_LAYER_AMP = 0.85

#: Вариант модели громкости ударной роли v2: рисунок, ближайший к сетке ``arrange.rhythm`` (хэты — оффбит).
DRUM_LOUDNESS_KEY: Mapping[str, str] = {"kick": "four_on_floor", "hats": "offbeat", "clap": "clap"}
#: Уровень роли v2, dB RMS в шкале модели (роль звучит всю секцию). Бочка и бас держат низ, пэд и лид ниже.
#: Подобрано по записям робота 02.10 (``compare.py``, цель A9 — низ 0.5–0.8, эталон 0.65/0.32/0.01): при пэде
#: −40 и лиде −44 середина перевешивала низ (0.43–0.68 против 0.32–0.56), при хэтах на потолке L−R уходил за
#: 1 дБ (статическая панорама хэтов). Недостижимый уровень (лид ``arpy`` на потолке −46.65) не прячется:
#: ``arrange.mix`` пишет в модель то, что синт может дать.
ROLE_LEVEL_DB: Mapping[str, float] = {
    "kick": -36.0, "bass": -37.0, "pad": -44.0, "lead": -46.0, "clap": -46.0, "hats": -61.0,
}

#: Сайдчейн «S» (ADR-0149 §3.8): усиление 16-х после удара триггера при глубине 1 — атака мгновенная,
#: подъём за 3 шага, дальше 1.0. Глубина ``d`` даёт ``1 − d·(1 − форма)``. Триггер — рисунок бочки жанра
#: («призрачная бочка»: тот же рисунок и в брейке, и в такте fill-а — огибающая не дёргается).
SIDECHAIN_SHAPE: Tuple[float, ...] = (0.3, 0.55, 0.8, 1.0)
DUCK_ROLES: Tuple[str, ...] = ("bass", "pad")
DUCK_DEPTH = 1.0

#: Тембры по теме (ADR-0149 §4.7 ``timbre_family``): роль → синты семьи; выбор внутри — по сиду трека. Только
#: синты с замером громкости (:data:`LANE_DB_AT_UNIT`), из палитры роли и не ``held`` (:data:`SYNTH_TRAITS`).
#: Пэд звучит 16-ми под сайдчейн, поэтому только пэды, чей хвост равен ``sus``: ``warmpad`` (хвост 1.2 с)
#: размазал бы огибающую и сложил бы 10 голосов в один. ``space`` убран по записям робота 02.10 (PR-3c):
#: на нём середина перевешивала низ (0.54–0.60 при низе 0.39–0.43, A9 — низ ≥ 0.5) — громче модели на 4–6 дБ.
TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("blip", "pluck"), "bass": ("dub", "bass"), "pad": ("sinepad",)},
    "hard": {"lead": ("arpy", "blip"), "bass": ("retrobass", "dub"), "pad": ("sinepad",)},
    "bright": {"lead": ("pluck", "blip"), "bass": ("bass",), "pad": ("sinepad",)},
    "warm": {"lead": ("pluck", "arpy"), "bass": ("bass", "dub"), "pad": ("sinepad",)},
}
#: Строка ``THEMES`` → семья тембров; тема не из таблицы — :data:`DEFAULT_TIMBRE`.
THEME_TIMBRE: Mapping[str, str] = {"space": "dark", "cyber": "hard", "kids": "bright", "slavic": "warm",
                                   "winter": "bright"}
DEFAULT_TIMBRE = "warm"


@dataclass(frozen=True)
class KickSound:
    """Сэмпл бочки ``play(symbol, sample=N)`` и его замер на роботе (запись ``jack_rec``, бочка одна, 4/4 @130)."""

    symbol: str
    sample: int
    file: str  # имя в ``0_foxdot_default/x/upper`` (сортировка Renardo ``_getFileInDir``)
    loudness_offset_db: float  # энергия удара против ``X:0`` по файлу — поправка к модели ``four_on_floor``
    low: float  # доля энергии записи < 250 Гц
    sub: float  # доля энергии записи < 120 Гц


#: Бочки, замеренные 02.10.2026 (Vision Pi, ``jack_rec``). ``X`` без ``sample=`` на роботе звучит щелчком:
#: < 250 Гц 0.3 %, > 2 кГц 98 % записи, хотя файл ``000_Kick_HSS2_NoN.wav`` на 99 % ниже 250 Гц (причина не
#: найдена, находка #3312). ``X:12`` — 94.5 % записи ниже 250 Гц, 5.5 % середины (атака), без гула 808.
KICK_SOUNDS: Mapping[str, KickSound] = {
    "house": KickSound("X", 12, "012_Kick_House_GhostFader.wav", 0.98, 0.945, 0.862),
}
#: Жанр → бочка из :data:`KICK_SOUNDS`.
GENRE_KICK: Mapping[str, str] = {"club": "house"}

#: Панорама (перенос таблиц ``core/club_stereo``): вынос хэтов/клэпа от центра (0.4: заметно, но не «в одну
#: колонку»), полуширина и период (доли) треугольного качания пэда v1 (``club_stereo.pad_pan``; в v2 пэд — два голоса).
PAN_HATS = 0.4
PAN_PAD_WIDTH = 0.6
PAD_PAN_BEATS = 32

# ── Стерео v2 (PR-9, ADR-0149 §3.9): ширина от декоррелированного материала, низ в центре ──────────────────────────
#: Почему не панорама: панорама моносигнала даёт корреляцию L/R 1.00 (замер PR-8: LR 1.00, Side/Mid −31 дБ против
#: 0.04 и −0.4 дБ у эталона). Пэд — два голоса как ``Player.spread()`` Renardo (``pan=(-w, w)``, ``pshift=(0, d)``),
#: второй ещё и позже на Хаас — пересборка образа не нужна. Стерео-реверба НЕТ: эффект ``room2`` Renardo (FreeVerb2)
#: на роботе глушит ВЕСЬ выход, пока звучит хоть одна нота с ним (замер 02.10: −180 dBFS, ``late``/``/fail`` 0;
#: с ``revatk=0.01`` то же) — собственный ``rbx_verb2.scd`` требует пересборки (ADR-0149 §3.9, остаток PR-9).
#: Выход робота — ReSpeaker 16 кГц: выше 8 кГц не слышно, поэтому ширина делается в середине (пэд), а не «воздухом».
#: Бочка и бас в таблицу не входят (центр, I11).
PAD_SPREAD = 1.0  # полуширина двух голосов пэда (как spread() Renardo; 0.9 дал Side/Mid −8.6 дБ под бочкой)
PAD_DETUNE = 0.125  # 1/8 полутона (ADR-0149 §3.6; spread() Renardo — то же)
PAD_HAAS_MS = 15.0  # Хаас 12–20 мс (§3.9)
HAAS_MAX_MS = 30.0  # больше — уже слышимое эхо, а не ширина
#: Роль → поля ``model.Stereo``. Клэп/перкуссия начинают с другой стороны, чем хэты (сумма двух слоёв не перекошена).
ROLE_STEREO: Mapping[str, Mapping[str, float]] = {
    "hats": {"pan": PAN_HATS, "first": 1},
    "clap": {"pan": PAN_HATS, "first": -1},
    "perc": {"pan": PAN_HATS, "first": -1},
    "pad": {"pan": PAD_SPREAD, "detune": PAD_DETUNE, "haas_ms": PAD_HAAS_MS},
}  # лида нет: на оси (§3.9)

__all__ += [
    "AMP_EXPONENT", "DEFAULT_TIMBRE", "DRUM_LOUDNESS_KEY", "DUCK_DEPTH", "DUCK_ROLES", "GENRE_KICK", "KICK_SOUNDS",
    "HAAS_MAX_MS", "KickSound", "LANE_DB_AT_UNIT", "LAYER_MEASURED_DB", "LOUDNESS_SOURCE", "MAX_LAYER_AMP",
    "PAD_DETUNE", "PAD_HAAS_MS", "PAD_PAN_BEATS", "PAD_SPREAD", "PAN_HATS", "PAN_PAD_WIDTH", "ROLE_LEVEL_DB",
    "ROLE_STEREO", "SIDECHAIN_SHAPE", "THEME_TIMBRE", "TIMBRES",
]
