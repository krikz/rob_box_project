"""Одна таблица знания аранжировщика v2 (ADR-0149 §2.1, §2.2; ADR-0148).

Лады, тоники, роли, палитра синтов и их свойства, коридоры регистров, потолки
уровней, жанровые окна. Остальные модули пакета только импортируют отсюда.

Значения перенесены из старого кода (ADR-0149 §9); старые таблицы удалены вместе
со старым путём (PR-13b), источник истины — этот модуль.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Tuple

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
ROLES: Tuple[str, ...] = ("kick", "hats", "clap", "perc", "bass", "pad", "lead", "sample", "loop", "fx")
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
    "hats": -14.0, "clap": -10.0, "perc": -14.0, "sample": -10.0, "loop": -10.0, "fx": -14.0,
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

#: Слова просьбы и темы, которые не называют мелодию (#3399: «вечеринка по темам терминатора» искала
#: «по»/«темам» и находила мусор). Сверка — по основе слова (``engine.search.stem``): падежи не перечисляются.
#: Английские — только служебные: «Theme», «Music», «Dj» — названия записей архива.
SEARCH_STOPWORDS: Tuple[str, ...] = (
    "вечеринка", "тусовка", "дискотека", "сет", "тема", "по", "для", "любители", "любитель", "играем", "играть",
    "сыграй", "поиграй", "давай", "включи", "включай", "поставь", "запусти", "хочу", "хотим", "мне", "нам", "нас",
    "диджей", "диджейский", "музыка", "музыкальный", "трек", "мелодия", "песня", "песенка", "про", "на", "в", "во",
    "и", "с", "со", "у", "о", "об", "от", "из", "к", "ко", "за", "как", "стиль", "стиле", "жанр", "ну", "а", "это",
    "этот", "эта", "сегодня", "будет", "будем", "пожалуйста", "робби", "чтобы", "чтоб", "типа", "вайб",
    "the", "of", "a", "an", "and", "in", "on", "for", "to", "with", "from", "at", "by", "de", "la", "le", "du", "des",
)
#: Понятия темы без мелодии с таким словом в названии → английский запрос к архиву (несколько слов — любое).
#: Ключ — начало основы слова темы.
THEME_CONCEPTS: Mapping[str, str] = {
    "косм": "space", "денди": "mario contra zelda", "dendy": "mario contra zelda", "приставк": "mario contra zelda",
    "нинтенд": "mario zelda", "nintendo": "mario zelda",
}


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

#: Слоты плееров деки (ADR-0149 §3.2): три ударных, три тональных и два сэмпловых — ``sample`` и ``fx`` (PR-3d;
#: ``c1/c2``, ``e1/e2`` санитайзер v1 не переставляет: он ловит только ``[dpsl]N``). На роботе 01.10 звучат и
#: ``d4..p6``, и ``a1..b3`` (PR-2); дека B — ``a1..b3``, потому что ``d4..p6`` санитайзер v1 ещё
#: переставляет в ``d1..p3`` (#1804), а v2 и v1 пока делят один Renardo.
DECK_SLOTS: Mapping[str, Tuple[str, ...]] = {
    "A": ("d1", "d2", "d3", "p1", "p2", "p3", "c1", "c2", "c3"),
    "B": ("a1", "a2", "a3", "b1", "b2", "b3", "e1", "e2", "e3"),
}

#: Акцент шага сетки 0..3 → множитель ``amplify`` (ADR-0149 §3.4: сильные доли громче).
ACCENT_AMPLIFY: Tuple[float, ...] = (0.4, 0.6, 0.8, 1.0)


# --- Каркасы ударных и каталог сэмплов DJ_Dave (ADR-0149 §3.4, §3.11, §8.1; PR-3d) -------------------

#: Каркас ударных поверх бочки и клэпа (бочку решает жанр, клэп — бэкбит 2/4): рисунки такта по 16 шагов.
#: ``X``/``x`` — удар с акцентом 3/2, ``g`` — гоуст (акцент 0; на нечётной 16-й его качает свинг сета),
#: ``.`` — пауза. ``perc`` — где бьёт слой одиночных сэмплов (``SampleInfo.role == "perc"``).
#: Ось разнообразия «каркас» (I17, A13): два трека подряд с одним каркасом не играют.
#: Стерео хэтов (PR-9) меняет сторону на каждом ударе: акценты делятся поровну между сторонами, либо ударов
#: нечётно (тогда сторона удара меняется через проход) — баланс L/R по энергии 0 (``test_diversity``).
DRUM_KITS: Mapping[str, Mapping[str, str]] = {
    "offbeat": {"hats": "..X...xg..X...xg", "perc": "...x.......x..x."},
    "sixteenths": {"hats": "ggXgggxgggXgggx.", "perc": ".......x.......x"},
    "open": {"hats": "..X..gX...X..gX.", "perc": ".x.....x.x....x."},
    "shuffle": {"hats": "..X.g.xg..X.g.x.", "perc": "......x.......x."},
    "ride": {"hats": "x.Xgx.X.x.Xgx.X.", "perc": "...x..x....x...."},
}
#: psr DJ_Dave: список из её кода «Array» — ``s("psr:[2|5|6|7|8|9|10|11|12|16|24|25|28|29]")``.
DAVE_PSR: Tuple[str, ...] = tuple(f"dirt_psr_{n:02d}" for n in (2, 5, 6, 7, 8, 9, 10, 11, 12, 16, 24, 25, 28, 29))

#: Каталог DJ_Dave — данные (перенесены из ``rob_box_mcp_tools/data``, старый ``core/sample_dave`` — вид отсюда).
#: Файлы кладёт на хост Ресурсный пак (ADR-0125/0126, запись ``dj-dave-samples``) в ``/opt/rob_box/samples/<пак>``,
#: в контейнерах это корень сэмплов Renardo ``/root/.config/renardo/samples``.
_SAMPLE_DATA = json.loads((Path(__file__).resolve().parent / "data" / "sample_dave.json").read_text(encoding="utf-8"))
SAMPLE_PACK_DIR = str(_SAMPLE_DATA["pack_dir"])
#: Роли сэмпла: ``perc`` — удар по сетке каркаса, ``loop`` — луп, растянутый на ``beats`` долей, ``fx`` —
#: одиночный акцент на границе секции (пик в начале файла), ``riser`` — нарастание (пик в конце: tn1hit2, пик на
#: 81 % длины, замер 02.10), ``vox``/``bass``/``synth`` — тональные стемы, ``kick`` — бочки (бочку решает жанр).
SAMPLE_ROLES: Tuple[str, ...] = ("perc", "loop", "fx", "riser", "vox", "bass", "synth", "kick")


@dataclass(frozen=True)
class SampleInfo:
    """Сэмпл пака: ``seconds``/``channels`` — ffprobe скачанного файла (30.09), ``peak_db``/``mean_db`` — ffmpeg
    volumedetect файла на роботе (02.10), ``path`` — от корня сэмплов, ``bpm`` — темп оригинала, ``key`` — (тоника,
    лад), если известна. Тональный сэмпл без ``key`` в трек не идёт."""

    name: str
    group: str
    role: str
    seconds: float
    channels: int
    path: str
    peak_db: float
    mean_db: float
    bpm: Optional[int] = None
    key: Optional[Tuple[int, str]] = None
    tonal: bool = False

    @property
    def loop_arg(self) -> str:
        """Аргумент ``loop()``: путь от папки лупов пака 0 (``spack=`` в Renardo — no-op, #2841)."""
        return f"../../{self.path}"

    @property
    def beats(self) -> Optional[int]:
        """Длина в долях при темпе оригинала (для ``beat_stretch``)."""
        return round(self.seconds * self.bpm / 60.0) if self.bpm else None


_SAMPLE_GROUP_ROLE = {
    "algorave_vocal": "vox", "algorave_beat": "loop", "algorave_fx": "fx", "array_vox": "vox",
    "array_bass": "bass", "array_synth": "synth",
}
_SAMPLE_ROLE_BY_NAME = {
    **dict.fromkeys(("array_perc_shaker", "array_perc_break"), "loop"),
    **dict.fromkeys(("array_perc_kick", "dirt_hh_hh3kick1", "dirt_hh_hh3kick2", "dirt_tech_tn1kick1",
                     "dirt_tech_tn1kick2"), "kick"),
    **dict.fromkeys(("dirt_hh_hh3crash", "dirt_hh_hh3hit1", "dirt_hh_hh3hit2", "dirt_hh_hh3hit3", "dirt_hh_hh3rerc1",
                     "dirt_hh_hh3rerc2", "dirt_tech_tn1crash", "dirt_tech_tn1hit1", "dirt_tech_tn1hit3",
                     "algorave_fx"), "fx"),
    "dirt_tech_tn1hit2": "riser",
}
_SAMPLE_TONAL = frozenset({"array_perc_guit1", "array_perc_guit2"})
#: Темп оригинала: Array (Lil Data) — 140, whatuneed — 128, spilltab — 100 (эталоны ``reference_tracks``).
_SAMPLE_BPM = {"algorave_wun_beat": 128, "algorave_wun_noise": 128, "algorave_wun_vox": 128,
               "algorave_spilltab": 100}
#: spilltab в оригинале звучит в A# minor (эталон «By Design», ``root=10``).
_SAMPLE_KEY = {"algorave_spilltab": (10, "minor")}


def _sample_info(name: str, meta: Mapping[str, object]) -> SampleInfo:
    group = str(meta["group"])
    role = _SAMPLE_ROLE_BY_NAME.get(name) or _SAMPLE_GROUP_ROLE.get(group, "perc")
    return SampleInfo(
        name, group, role, float(meta["seconds"]), int(meta["channels"]), f"{SAMPLE_PACK_DIR}/{meta['path']}",
        float(meta["peak_db"]), float(meta["mean_db"]),
        _SAMPLE_BPM.get(name, 140 if name.startswith("array_") else None), _SAMPLE_KEY.get(name),
        role in ("vox", "bass", "synth") or name in _SAMPLE_TONAL)


#: Каталог сэмплов DJ_Dave: имя → :class:`SampleInfo`. Один на v1 (``core/sample_dave``) и v2.
SAMPLE_CATALOG: Mapping[str, SampleInfo] = {n: _sample_info(n, m) for n, m in _SAMPLE_DATA["samples"].items()}
#: Группа каталога → описание (подсказки модели в v1).
SAMPLE_GROUPS: Mapping[str, str] = dict(_SAMPLE_DATA["groups"])


def scale_pitch_classes(root: int, mode: str) -> frozenset:
    """Множество pitch class лада от тоники ``root`` (0..11)."""
    return frozenset((root + step) % 12 for step in SCALES[mode])


def traits_of(synth: str) -> "SynthTraits | None":
    return SYNTH_TRAITS.get(synth.strip().lower()) if synth else None


def role_ceiling(role: str) -> float:
    return LEVEL_CEILINGS[role]


__all__ = [
    "ACCENT_AMPLIFY", "BPM_RANGE", "CHROMATIC", "DAVE_PSR", "DECK_SLOTS", "DRUM_KITS", "DRUM_SYMBOLS", "ENERGY_LEVELS",
    "ENERGY_TRIM_DB", "DEFAULT_HOOKS", "ENERGY_THIN_ROLES", "ENERGY_WAVE", "GENRE_WINDOWS", "GenreWindow", "THEMES",
    "ThemeRow", "KICK_PATTERNS", "LEAD_MAX_MIDI", "LEVEL_CEILINGS", "MOOD_ENERGY", "PLAY_SYNTH", "REGISTERS", "ROLES",
    "ROOTS", "SAMPLE_CATALOG", "SAMPLE_GROUPS", "SAMPLE_PACK_DIR", "SAMPLE_ROLES", "SCALES", "SEARCH_STOPWORDS",
    "SYNTH_PALETTE", "SYNTH_TRAITS", "SampleInfo", "SynthTraits", "THEME_CONCEPTS", "TONAL_ROLES", "role_ceiling",
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
    # Приёмка 02.10 (A9) и отзыв эксперта 05.10 («с ударными слабовато»): лид −46 → −50 (он забивал середину в
    # дропах: без лида доля низа 0.18 → 0.33), клэп −46 → −43 и хэты −61 → −57 (ударные были на 10–25 дБ ниже бочки).
    "kick": -36.0, "bass": -37.0, "pad": -44.0, "lead": -50.0, "clap": -43.0, "hats": -57.0,
    # PR-3d: слой DJ_Dave — amp как в v1 club (``club_samples``: шейкер 0.15–0.21, psr 0.16–0.23; сеты 30.09–01.10,
    # которые Шифу принял на слух). Замер A/B 02.10 (тот же трек без слоёв, 1–8 кГц): при −47/−45 слои дают
    # +0.1…+0.8 дБ — не слышны; −37/−35 — те же amp, что у v1. FX-удар на границе секции на 2 дБ громче слоя.
    # Раунд 5: psr-пул на 16-х и брейк-луп нарезкой — тот же уровень, что слой раунда 4 (A/B +1.7…+2.6 дБ).
    "sample": -37.0, "loop": -37.0, "fx": -35.0,
}

#: Сайдчейн «S» (ADR-0149 §3.8): усиление 16-х после удара триггера при глубине 1 — атака мгновенная,
#: подъём за 3 шага, дальше 1.0. Глубина ``d`` даёт ``1 − d·(1 − форма)``. Триггер — рисунок бочки жанра
#: («призрачная бочка»: тот же рисунок и в брейке, и в такте fill-а — огибающая не дёргается).
SIDECHAIN_SHAPE: Tuple[float, ...] = (0.3, 0.55, 0.8, 1.0)
#: psr-слой DJ_Dave — под той же огибающей, что пэд и бас (``postgain(sidechain)``, PR-3d).
DUCK_ROLES: Tuple[str, ...] = ("bass", "pad", "sample")


@dataclass(frozen=True)
class Look:
    """«Вид» секции (приём DJ_Dave, интервью 02.10): одна ось энергии переключает и рисунок бочки, и сайдчейн."""

    kick: str  # рисунок бочки 16 шагов — он же триггер огибающей сайдчейна
    duck_depth: float  # 0..1


#: Энергия секции (0..10) → вид: (порог, вид), первый подходящий сверху. Дроп — прямая бочка и полный «насос»;
#: build — бочка на 1 и 3 и мягкий насос (подъём держат фильтр и ролл клэпа); интро/аутро (блэнд двух дек) —
#: ровная прямая бочка, под которую сводятся треки.
LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["four_on_floor"], 1.0)),
    (5, Look("X.......X.......", 0.5)),
    (0, Look(KICK_PATTERNS["four_on_floor"], 0.6)),
)

#: LPF-свип на басе и нотах (DJ_Dave: «один слайдер на оба»; ADR-0149 §3.12): секция → (от, до) Гц, 0 — фильтр
#: снят. Build открывается к дропу, дроп открыт, брейк прикрыт, хвост уходящей деки закрывается 4000→300 за блэнд.
#: Потолок свипа ниже среза мастер-шины (``masterfilter`` LPF 0.55·Найквиста = 4400 Гц на 16 кГц): выше — не слышно,
#: а срез выше Найквиста у ``RLPF`` неустойчив. Шаг свипа — доля (``var``, не ``linvar``: скачок на границе секции
#: у ``linvar`` требует сегмента, и нота на нём получила бы промежуточный срез).
LPF_OPEN = 0.0
LPF_TOP_HZ = 4000.0
LPF_RANGE_HZ = (200.0, LPF_TOP_HZ)
SECTION_LPF: Mapping[str, Tuple[float, float]] = {
    "build": (400.0, LPF_TOP_HZ), "break": (1200.0, 1200.0), "outro_tail": (LPF_TOP_HZ, 300.0),
}
#: Роли под свипом секции; в хвосте блэнда (``outro_tail``) — всё, что звучит, кроме бочки (§3.12).
LPF_ROLES: Tuple[str, ...] = ("bass", "pad", "lead")
LPF_TAIL_SECTIONS: Tuple[str, ...] = ("outro_tail",)

# ── Мастер-шина (``custom_synthdefs/masterfilter.scd``, node 999): ADR-0149 §3.10, ADR-0147 §3.2, §3.5; PR-7 ───────
#: Ручки мастер-шины, которые выставляет движок v2, и их значения по умолчанию — ровно дефолты SynthDef
#: (сверяет ``test_master.py``): каждый старт трека и стоп выставляют их заново, поэтому трек вне сета не наследует
#: ``trim`` и профиль выравнивателя от DJ-трека (правило сброса ADR-0147 §3.2).
MASTER_DEFAULTS: Mapping[str, float] = {"trim": 0.0, "lvlRatio": 3.0, "lvlUp": 3.0}
#: DJ-профиль выравнивателя (ADR-0147 §3.5): из 10 дБ брейк/дроп остаётся ~6.7 вместо ~3.5, тихое не тянется
#: за секунду. Включается только по замеру ``dyn 0/1`` (решение Шифу В3) — :data:`SET_LEVELER`.
DJ_LEVELER: Mapping[str, float] = {"lvlRatio": 1.5, "lvlUp": 8.0}
#: Профиль выравнивателя сета: ВЫКЛЮЧЕН по замеру PR-7 (робот 02.10, один сет «киберпанк» seed 71, 6 мин, с trim):
#: DJ-профиль A6 не поднял (5.9 → 5.9 дБ), дроп−брейк +0.5…1.7 дБ, а crest по минутам 17.4–19.1 — выше A10 (12–18).
#: Включить — ``SET_LEVELER = DJ_LEVELER`` по слуху Шифу (В3, #3154).
SET_LEVELER: Mapping[str, float] = {}
#: ``trimLag`` при старте трека встык, с; в блэнде громкость переезжает за длину блэнда.
TRIM_LAG_S = 0.5
#: Дуга громкости трека: секция → (смещение ``trim`` относительно ``trim`` трека, дБ; подъём на всю секцию?).
#: Приёмка 02.10: брейк без бочки и баса был лишь на 1.8 дБ тише дропа — выравниватель мастер-шины (``lvlRatio 3``)
#: поднимал его, и трек звучал ровной полкой (отзыв эксперта 05.10: «нет восхождения, кульминации»). ``trim`` стоит
#: ПОСЛЕ динамики, выравниватель его не съедает. Смещения ≤ 0: громче ``trim`` трека не бывает (лимитер позади).
#: Build поднимается всю секцию к дропу, второй дроп — пик трека; интро и хвост равны: в блэнде двух дек (мастер
#: общий) громкость не прыгает. Брейк −5, не глубже: в шумной мастерской тихое пропадает (#3154, решение В3).
SECTION_TRIM_DB: Mapping[str, Tuple[float, bool]] = {
    "intro": (-4.0, False), "intro_low": (-4.0, False), "build": (-1.5, True), "drop": (-1.0, False),
    "break": (-5.0, False), "drop2": (0.0, False), "outro": (-3.0, False), "outro_tail": (-4.0, False),
}

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
    # psr-пул DJ_Dave (PR-3d): два голоса L/R с Хаасом (аналог ``jux``). Смена стороны по ударам под сайдчейном
    # перекосила бы баланс: огибающая громче на нечётных 16-х, а они всегда на одной стороне.
    "sample": {"pan": PAN_HATS, "haas_ms": PAD_HAAS_MS},
}  # лида нет: на оси (§3.9)

__all__ += [
    "AMP_EXPONENT", "DEFAULT_TIMBRE", "DJ_LEVELER", "DRUM_LOUDNESS_KEY", "DUCK_ROLES", "GENRE_KICK", "KICK_SOUNDS",
    "HAAS_MAX_MS", "KickSound", "LANE_DB_AT_UNIT", "LAYER_MEASURED_DB", "LOOKS", "LOUDNESS_SOURCE", "LPF_OPEN",
    "LPF_RANGE_HZ", "LPF_ROLES", "LPF_TAIL_SECTIONS", "LPF_TOP_HZ", "Look", "MASTER_DEFAULTS", "MAX_LAYER_AMP",
    "SECTION_LPF", "SET_LEVELER", "TRIM_LAG_S",
    "PAD_DETUNE", "PAD_HAAS_MS", "PAD_PAN_BEATS", "PAD_SPREAD", "PAN_HATS", "PAN_PAD_WIDTH", "ROLE_LEVEL_DB",
    "ROLE_STEREO", "SECTION_TRIM_DB", "SIDECHAIN_SHAPE", "THEME_TIMBRE", "TIMBRES",
]


# ── Classic-форма «песня» (PR-11, ADR-0149 §3.3, §9): мелодия целиком по куплетам, аккомпанемент — harmonize ──

#: ``Form.kind``: клубная форма (32/48/64 такта, outro ≥ 8) и песня по куплетам (длина — от мелодии).
FORM_CLUB = "club"
FORM_SONG = "song"
#: Куплетов в песне: столько проходов мелодии, чтобы форма была около :data:`SONG_TARGET_BARS` тактов, но не
#: больше, чем строк в :data:`SONG_VERSES` (длинная тема — один куплет; ср. бюджет повторов v1
#: ``arranger._snap_plan_to_theme``).
SONG_TARGET_BARS = 32
#: Самая длинная песня, которую принимает модель (архив 02.10: 10448 тем, p99 — 22 такта, максимум — 158).
SONG_MAX_BARS = 256
_SONG_FULL = frozenset({"kick", "hats", "clap", "bass", "pad", "lead"})
#: Куплеты по их числу: (имя, энергия секции 0..10, роли). Первый куплет — тема поверх пэда и баса, потом
#: входят ударные, последний — полный состав (план состава v1 ``arranger.FORMS``, #2978).
SONG_VERSES: Mapping[int, Tuple[Tuple[str, int, frozenset], ...]] = {
    1: (("verse", 7, _SONG_FULL),),
    2: (("verse", 5, frozenset({"hats", "bass", "pad", "lead"})), ("chorus", 8, _SONG_FULL)),
    3: (("verse", 4, frozenset({"bass", "pad", "lead"})),
        ("verse2", 6, frozenset({"kick", "hats", "bass", "pad", "lead"})), ("chorus", 8, _SONG_FULL)),
}
#: Коридоры регистров песни. Мелодия звучит, как записана (регистр уже нормализовал ``rtttl_compose``, но архив
#: 02.10 даёт лид 36..106, p1–p99 56..92) — коридор лида = клавиатура фортепиано A0..C8. Бас — от
#: ``harmonize.BASS_MIDI_FLOOR`` (архив: 36..54), пэд — ``harmonize.PAD_REGISTER_LIMITS`` (архив: 48..82).
SONG_REGISTERS: Mapping[str, Tuple[int, int]] = {"bass": (36, 60), "pad": (36, 96), "lead": (21, 108)}
#: Аккомпанемент песни в тональности мелодии: доля длительности баса и пэда в ладу трека не ниже порога. Порог
#: ловит чужую тональность, а не заимствованные аккорды ``harmonize``: архив 02.10 — бас min 0.42 / p1 0.81,
#: пэд min 0.50 / p1 0.83.
SONG_KEY_FIT_MIN = 0.4
#: Тембры песни (только синты с замером громкости :data:`LANE_DB_AT_UNIT`); лид выбирается по сиду.
SONG_TIMBRES: Mapping[str, Tuple[str, ...]] = {"lead": ("pluck", "marimba", "karp"), "bass": ("bass",),
                                               "pad": ("warmpad",)}
#: Символ рисунка ударных ``harmonize`` (16 шагов на такт) → роль v2: ``X`` — бочка, ``o`` (малый) — клэп
#: (у малого нет замера громкости), ``-`` — хэты.
SONG_DRUM_SYMBOLS: Mapping[str, str] = {"X": "kick", "o": "clap", "-": "hats"}

__all__ += [
    "FORM_CLUB", "FORM_SONG", "SONG_DRUM_SYMBOLS", "SONG_KEY_FIT_MIN", "SONG_MAX_BARS", "SONG_REGISTERS",
    "SONG_TARGET_BARS", "SONG_TIMBRES", "SONG_VERSES",
]
