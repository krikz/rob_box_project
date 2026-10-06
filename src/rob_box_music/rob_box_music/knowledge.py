"""Одна таблица знания аранжировщика v2 (ADR-0149 §2.1, §2.2; ADR-0148).

Лады, тоники, роли, палитра синтов и их свойства, коридоры регистров, потолки
уровней, стили аранжировщика (``STYLES``, ADR-0153). Остальные модули пакета только импортируют отсюда.

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
_CLUB_REGISTERS: Mapping[str, Tuple[int, int]] = {"bass": (36, 52), "pad": (50, 70), "lead": (58, 84)}
LEAD_MAX_MIDI = 88
#: Хук темы в тональности трека (I12): доля длительности нот лида в ладу не ниже порога; хроматика — проходящие.
HOOK_KEY_FIT_MIN = 0.6

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
    "pad": ("sinepad", "warmpad", "space", "ambi", "strangerpulsepad", "strings"),
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

#: Каркас ударных поверх бочки и клэпа (бочку решает стиль, клэп — бэкбит 2/4): рисунки такта по 16 шагов.
#: ``X``/``x`` — удар с акцентом 3/2, ``g`` — гоуст (акцент 0; на нечётной 16-й его качает свинг сета),
#: ``.`` — пауза. ``perc`` — где бьёт слой одиночных сэмплов (``SampleInfo.role == "perc"``).
#: Ось разнообразия «каркас» (I17, A13): два трека подряд с одним каркасом не играют.
#: Стерео хэтов (PR-9) меняет сторону на каждом ударе: акценты делятся поровну между сторонами, либо ударов
#: нечётно (тогда сторона удара меняется через проход) — баланс L/R по энергии 0 (``test_diversity``).
_CLUB_KITS: Mapping[str, Mapping[str, str]] = {
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
#: 81 % длины, замер 02.10), ``vox``/``bass``/``synth`` — тональные стемы, ``kick`` — бочки (бочку решает стиль).
SAMPLE_ROLES: Tuple[str, ...] = ("perc", "loop", "fx", "riser", "vox", "bass", "synth", "kick")
#: Края огибающей запуска файла синтом ``loop``, с: (атака, спад) — ВНУТРИ ``sus`` (патч ``loop.scd``, #3432).
#: Как у сэмплов Strudel DJ_Dave: удар с первого отсчёта (атака 2 мс не съедает щелчок psr), кусок ``chop``
#: кончается к следующему (``legato(1)``: спад 5 мс до начала следующего куска, без наложения). Штатный синт
#: Renardo 0.9.13 — 50 мс атаки и 50 мс спада ПОСЛЕ ``sus``: удар psr на 16-й (109 мс при 138 BPM) нарастал до
#: середины шага и наползал на следующий, а файл короче ``sus`` + 50 мс начинался заново (``PlayBuf(loop: 1)``).
SAMPLE_EDGE_S: Tuple[float, float] = (0.002, 0.005)


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
    "ACCENT_AMPLIFY", "BPM_RANGE", "CHROMATIC", "DAVE_PSR", "DECK_SLOTS", "DRUM_SYMBOLS", "ENERGY_LEVELS",
    "ENERGY_TRIM_DB", "DEFAULT_HOOKS", "ENERGY_THIN_ROLES", "ENERGY_WAVE", "THEMES",
    "ThemeRow", "HOOK_KEY_FIT_MIN",
    "KICK_PATTERNS", "LEAD_MAX_MIDI", "LEVEL_CEILINGS", "MOOD_ENERGY", "PLAY_SYNTH", "ROLES",
    "ROOTS", "SAMPLE_CATALOG", "SAMPLE_EDGE_S", "SAMPLE_GROUPS", "SAMPLE_PACK_DIR", "SAMPLE_ROLES", "SCALES",
    "SEARCH_STOPWORDS",
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
    # 05.10 (#3422, ``loudness_nrt_v2.py --sweep``): новые строки — та же рамка 29.09 с подменой синта, scsynth
    # 3.11.2 образа voice-assistant (arm64/qemu на katana), renardo_lib с патчами робота. Регресс рамки 29.09:
    # sinepad −0.16, pluck −1.24, bass +0.06, four_on_floor +0.01 дБ; прочие строки 29.09: blip −0.43, arpy −0.11,
    # karp −1.66 (вне ±1.5, причина не найдена), marimba −0.38, retrobass +0.02, dub +0.08, warmpad +0.06,
    # space −0.10. Строки 29.09 не менялись.
    "lead": (0.28, {"pluck": -53.4, "blip": -55.0, "arpy": -56.3, "karp": -66.3, "marimba": -68.9,
                    "strangerarp": -32.7, "strangerbrass": -35.5, "imperialbrass": -36.3, "supersawlead": -37.4,
                    "tubularbell": -39.9, "ecello": -41.6, "organ": -46.3, "rhpiano": -49.2, "brass": -50.8,
                    "epiano": -51.6, "kalimba": -52.0, "cs80lead": -52.2, "orient": -52.8, "hoover": -53.3,
                    "eoboe": -53.6, "pulse": -53.9, "fuzz": -55.4, "organ2": -56.5, "steeldrum": -57.7,
                    "soprano": -57.8, "keys": -58.8, "saw": -58.8, "creep": -62.9, "rave": -64.9, "varsaw": -66.1,
                    "bell": -67.3, "sitar": -69.2, "viola": -71.5, "flute": -72.5, "square": -72.5}),
    "bass": (0.4, {"bass": -24.4, "retrobass": -41.9, "dub": -24.8,
                   "jbass": -32.9, "wobblebass": -35.7, "tb303": -36.2, "subbass": -42.7, "moogbass": -56.3}),
    "pad": (0.11, {"sinepad": -74.4, "warmpad": -43.1, "space": -54.5,
                   "marchstrings": -38.1, "mhpad": -38.9, "strangerpulsepad": -43.0, "ambi": -56.5, "strings": -59.5,
                   "pads": -64.0}),
}
#: Границы полос замера слоя, Гц: низ < 150, середина 150–2000, верх ≥ 2000 (как ``live_dj/compare.profile``).
LAYER_BANDS_HZ: Tuple[float, float] = (150.0, 2000.0)
#: Доли энергии слоя по полосам :data:`LAYER_BANDS_HZ` — тот же рендер, что :data:`LAYER_MEASURED_DB`
#: (``scripts/music/loudness_nrt_v2.py --sweep``, ADR-0152 §3.1): ``{роль: {синт: (низ, середина, верх)}}``.
#: Данные для модели микса (A9), выбор синтов от них пока не зависит.
#: ``kick`` — рисунок ``four_on_floor`` рамки 29.09 (бочка ``X`` без ``sample=``; на роботе она звучит иначе,
#: ``KICK_SOUNDS``). ``moogbass`` — разброс по тоникам 8.1 дБ, ``sinepad``/``mhpad``/``marchstrings`` — 3–4 дБ.
LAYER_BANDS: Mapping[str, Mapping[str, Tuple[float, float, float]]] = {
    "kick": {"four_on_floor": (0.952, 0.045, 0.003)},
    "lead": {
        "pluck": (0.0, 0.984, 0.016), "blip": (0.0, 0.984, 0.016), "arpy": (0.007, 0.829, 0.164),
        "karp": (0.01, 0.983, 0.007), "marimba": (0.004, 0.976, 0.02), "strangerarp": (0.0, 0.997, 0.003),
        "strangerbrass": (0.0, 0.992, 0.008), "imperialbrass": (0.0, 0.99, 0.01), "supersawlead": (0.0, 0.972, 0.028),
        "tubularbell": (0.0, 0.998, 0.002), "ecello": (0.905, 0.095, 0.0), "organ": (0.0, 0.998, 0.002),
        "rhpiano": (0.0, 1.0, 0.0), "brass": (0.0, 0.999, 0.001), "epiano": (0.0, 1.0, 0.0),
        "kalimba": (0.001, 0.992, 0.007), "cs80lead": (0.0, 0.984, 0.016), "orient": (0.013, 0.924, 0.063),
        "hoover": (0.011, 0.966, 0.023), "eoboe": (0.0, 0.968, 0.032), "pulse": (0.0, 0.979, 0.021),
        "fuzz": (0.121, 0.853, 0.026), "organ2": (0.004, 0.972, 0.024), "steeldrum": (0.0, 0.998, 0.002),
        "soprano": (0.0, 0.971, 0.029), "keys": (0.0, 0.999, 0.001), "saw": (0.0, 0.967, 0.033),
        "creep": (0.002, 0.578, 0.42), "rave": (0.037, 0.813, 0.15), "varsaw": (0.0, 0.99, 0.01),
        "bell": (0.009, 0.51, 0.481), "sitar": (0.002, 0.868, 0.13), "viola": (0.0, 0.844, 0.156),
        "flute": (0.253, 0.673, 0.074), "square": (0.0, 0.983, 0.017),
    },
    "bass": {
        "bass": (0.957, 0.043, 0.0), "retrobass": (0.589, 0.411, 0.0), "dub": (0.999, 0.001, 0.0),
        "jbass": (0.994, 0.006, 0.0), "wobblebass": (0.9, 0.099, 0.001), "tb303": (0.521, 0.472, 0.007),
        "subbass": (0.996, 0.004, 0.0), "moogbass": (0.99, 0.01, 0.0),
    },
    "pad": {
        "sinepad": (0.001, 0.999, 0.0), "warmpad": (0.085, 0.915, 0.0), "space": (0.0, 1.0, 0.0),
        "marchstrings": (0.107, 0.893, 0.0), "mhpad": (0.009, 0.991, 0.0), "strangerpulsepad": (0.058, 0.942, 0.0),
        "ambi": (0.052, 0.948, 0.0), "strings": (0.009, 0.986, 0.005), "pads": (0.0, 0.394, 0.606),
    },
}
#: Наклон громкости по ``amp``: ``dB = 20·p·log10(amp)``; у ``dub``/``karp``/``sinepad`` ``amp`` входит дважды.
#: 05.10 (#3422): наклон по уровню и его половине, ``p`` ≥ 1.5 → 2.0 (замер 2.00–2.04).
AMP_EXPONENT: Mapping[str, float] = {"dub": 2.0, "karp": 2.0, "sinepad": 2.0, "sitar": 2.0, "viola": 2.0,
                                     "soprano": 2.0, "square": 2.0, "varsaw": 2.0, "rave": 2.0, "subbass": 2.0}
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
_CLUB_ROLE_LEVEL_DB: Mapping[str, float] = {
    # Приёмка 02.10 (A9) и отзыв эксперта 05.10 («с ударными слабовато»): лид −46 → −50 (он забивал середину в
    # дропах: без лида доля низа 0.18 → 0.33), клэп −46 → −43 и хэты −61 → −57 (ударные были на 10–25 дБ ниже бочки).
    # Перемер 05.10 (#3401, после #3394/#3396): низ в дропах 0.42, во втором дропе 0.37 (A9 ≥ 0.5). Лид уже не
    # виноват (2 % мощности дропа по этой модели) — середину держат psr-слой и луп на уровне баса. Доля бочки+баса
    # в мощности дропа по модели (с сайдчейном) 0.68 → 0.86, второго дропа 0.61 → 0.81; на роботе низ дропа был
    # 0.62 этой доли → ожидание 0.53 / 0.50. Ударные громче — и отзыв эксперта 05.10.
    "kick": -33.0, "bass": -32.0, "pad": -44.0, "lead": -50.0, "clap": -43.0, "hats": -57.0,
    # PR-3d: слой DJ_Dave — amp как в v1 club (``club_samples``: шейкер 0.15–0.21, psr 0.16–0.23; сеты 30.09–01.10,
    # которые Шифу принял на слух). Замер A/B 02.10 (тот же трек без слоёв, 1–8 кГц): при −47/−45 слои дают
    # +0.1…+0.8 дБ — не слышны; −37/−35 — те же amp, что у v1. FX-удар на границе секции на 2 дБ громче слоя.
    # Раунд 5: psr-пул на 16-х и брейк-луп нарезкой — тот же уровень, что слой раунда 4 (A/B +1.7…+2.6 дБ).
    # #3401: psr −37 → −39 и луп −37 → −40 — они делили середину дропов с басом на равных; −39/−40 всё ещё на 6+ дБ
    # выше неслышных −47/−45.
    "sample": -39.0, "loop": -40.0, "fx": -35.0,
}

#: Сайдчейн «S» (ADR-0149 §3.8): усиление 16-х после удара триггера при глубине 1 — атака мгновенная,
#: подъём за 3 шага, дальше 1.0. Глубина ``d`` даёт ``1 − d·(1 − форма)``. Триггер — рисунок бочки вида секции
#: («призрачная бочка»: тот же рисунок и в брейке, и в такте fill-а — огибающая не дёргается).
SIDECHAIN_SHAPE: Tuple[float, ...] = (0.3, 0.55, 0.8, 1.0)
#: psr-слой DJ_Dave — под той же огибающей, что пэд и бас (``postgain(sidechain)``, PR-3d). Луп нарезкой — тоже
#: (#3432): у Дэйва брейк-луп короткой вставкой, у нас он звучит весь второй дроп (и шумовой ``wun_noise``) —
#: без провала на ударе он заливает место бочки ровной текстурой.
_CLUB_DUCK_ROLES: Tuple[str, ...] = ("bass", "pad", "sample", "loop")


@dataclass(frozen=True)
class Look:
    """«Вид» секции (приём DJ_Dave, интервью 02.10): одна ось энергии переключает и рисунок бочки, и сайдчейн."""

    kick: str  # рисунок бочки 16 шагов — он же триггер огибающей сайдчейна
    duck_depth: float  # 0..1


#: Энергия секции (0..10) → вид: (порог, вид), первый подходящий сверху. Дроп — прямая бочка и полный «насос»;
#: build — бочка на 1 и 3 и мягкий насос (подъём держат фильтр и ролл клэпа); интро/аутро (блэнд двух дек) —
#: ровная прямая бочка, под которую сводятся треки.
_CLUB_LOOKS: Tuple[Tuple[int, Look], ...] = (
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
_CLUB_SECTION_LPF: Mapping[str, Tuple[float, float]] = {
    "build": (400.0, LPF_TOP_HZ), "break": (1200.0, 1200.0), "outro_tail": (LPF_TOP_HZ, 300.0),
}
#: Роли под свипом секции; в хвосте блэнда (``outro_tail``) — всё, что звучит, кроме бочки (§3.12).
_CLUB_LPF_ROLES: Tuple[str, ...] = ("bass", "pad", "lead")
_CLUB_LPF_TAIL_SECTIONS: Tuple[str, ...] = ("outro_tail",)

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

#: Тембры по теме (ADR-0149 §4.7 ``timbre_family``): роль → синты семьи; выбор внутри — по сиду трека со штрафом за
#: недавнее (пэд, ADR-0152 §3.2). Только синты с замером громкости и полос (:data:`LANE_DB_AT_UNIT`,
#: :data:`LAYER_BANDS`) из палитры роли. Пэд с собственным хвостом (``warmpad`` 1.2 с, ``strangerpulsepad`` 1.6 с,
#: :data:`SYNTH_TRAITS`) звучит только рисунком, который это допускает (``PadFigure.long_tails`` — ``held``): под
#: сайдчейном 16-х хвост размазал бы огибающую. ``space`` вернулся (В9 ADR-0152): громче модели на 4–6 дБ он был в
#: рисунке 16-х — тот же сдвиг, что у ``sinepad`` (+6.3, #3430); долю низа дропа держит A9-модель трека
#: (``arrange.mix.a9_trim``), а не исключение из палитры. ≥ 2 пэда на семью, у каждого рисунка ≥ 1.
#: ``mhpad``/``marchstrings`` замерены (#3430), но не в досылке ``CRITICAL_SYNTHS`` — в пул не идут.
_CLUB_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("blip", "pluck"), "bass": ("dub", "bass"), "pad": ("sinepad", "space", "strangerpulsepad")},
    "hard": {"lead": ("arpy", "blip"), "bass": ("retrobass", "dub"), "pad": ("sinepad", "strings", "strangerpulsepad")},
    "bright": {"lead": ("pluck", "blip"), "bass": ("bass",), "pad": ("strings", "ambi", "sinepad")},
    "warm": {"lead": ("pluck", "arpy"), "bass": ("bass", "dub"), "pad": ("sinepad", "ambi", "warmpad")},
}
#: Строка ``THEMES`` → семья тембров стиля; тема не из таблицы — ``Style.default_timbre``.
THEME_TIMBRE: Mapping[str, str] = {"space": "dark", "cyber": "hard", "kids": "bright", "slavic": "warm",
                                   "winter": "bright"}


@dataclass(frozen=True)
class PadFigure:
    """Рисунок пэда (ADR-0152 §3.2): генератор — ``arrange.compose.PAD_GENERATORS[ключ]``.

    ``ducked`` — пэд под сайдчейном (``Style.duck_roles``); ``long_tails`` — допускает синты с собственным хвостом;
    ``level_offset_db`` — прибавка к цели роли, при которой пэд звучит так же громко, как ``pumped16`` на цели:
    модель громкости 29.09 снята на пэде, держащем аккорд 8 долей, а ``pumped16`` на роботе громче модели на 6.3 дБ
    (харнесс ``loudness_nrt_v2.py --track 7``, #3430). Той же прибавкой A9-модель (``arrange.mix``) возвращает
    рисунок в шкалу калибровки 0.62 (снята на ``pumped16``, #3401).
    """

    ducked: bool
    long_tails: bool
    level_offset_db: float


#: Рисунки пэда: ``pumped16`` — аккорд на каждой 16-й под сайдчейном (как до PR-5); ``held`` — аккорд держится до
#: смены (2 такта), без насоса — качают бас и psr; ``stabs`` — аккорд на «и» каждой доли до следующего «и» под
#: сайдчейном. ``stabs`` +3.0 — ГИПОТЕЗА (4 атаки на такт против 16 у ``pumped16`` и 1 на 2 такта у ``held``; взята
#: середина — пэд скорее тише, чем громче, доля низа не страдает), не замер: перемерить ``loudness_nrt_v2.py --track``
#: на треке со ``stabs``.
PAD_FIGURES: Mapping[str, PadFigure] = {
    "pumped16": PadFigure(ducked=True, long_tails=False, level_offset_db=0.0),
    "held": PadFigure(ducked=False, long_tails=True, level_offset_db=6.3),
    "stabs": PadFigure(ducked=True, long_tails=False, level_offset_db=3.0),
}
#: A9-модель трека (ADR-0152 §4 п.2): доля низа каждого дропа в шкале калибровки #3401 (робот = 0.62 модели) —
#: ниже порога ``Style.a9_model_low`` пэд тише ступенями ``A9_STEP_DB`` до ``A9_PAD_FLOOR_DB`` от своей цели, затем
#: бас громче до ``A9_BASS_BOOST_DB`` (бас под сайдчейном на роботе тише модели на 6.7 дБ, #3430). Не дотянули —
#: трек играет, ``Mix.a9_model`` честно ниже порога. Ступень, поднимающая долю меньше ``A9_MIN_GAIN``, не
#: применяется: у ``retrobass`` (низ 0.59, на потолке ``amp`` −35.3 при цели −32) доля 0.73 не растёт ни от пэда −12,
#: ни от баса — глушить пэд зря нельзя, это задача палитры басов (ADR-0152 PR-6).
A9_MIN_GAIN = 0.005
A9_STEP_DB = 2.0
A9_PAD_FLOOR_DB = -12.0
A9_BASS_BOOST_DB = 4.0


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

#: Панорама (перенос таблиц ``core/club_stereo``): вынос хэтов/клэпа от центра (0.4: заметно, но не «в одну
#: колонку»), полуширина и период (доли) треугольного качания пэда v1 (``club_stereo.pad_pan``; в v2 пэд — два голоса).
PAN_HATS = 0.4
#: psr-пул DJ_Dave — два голоса на полных L/R, как ``jux`` Strudel (#3424): при выносе 0.4 корреляция голосов с
#: Хаасом ≈ cos(0.4·π/2) = 0.81, при 1.0 — ≈ 0 (Хаас 15 мс декоррелирует удары); слой на 6 дБ ниже бочки — ширина
#: возвращается без басов и бочки в стороне.
PAN_PSR = 1.0
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
_CLUB_STEREO: Mapping[str, Mapping[str, float]] = {
    "hats": {"pan": PAN_HATS, "first": 1},
    "clap": {"pan": PAN_HATS, "first": -1},
    "perc": {"pan": PAN_HATS, "first": -1},
    "pad": {"pan": PAD_SPREAD, "detune": PAD_DETUNE, "haas_ms": PAD_HAAS_MS},
    # psr-пул DJ_Dave (PR-3d): два голоса L/R с Хаасом (аналог ``jux``). Смена стороны по ударам под сайдчейном
    # перекосила бы баланс: огибающая громче на нечётных 16-х, а они всегда на одной стороне.
    "sample": {"pan": PAN_PSR, "haas_ms": PAD_HAAS_MS},
}  # лида нет: на оси (§3.9)

__all__ += [
    "AMP_EXPONENT", "DJ_LEVELER", "DRUM_LOUDNESS_KEY", "KICK_SOUNDS",
    "HAAS_MAX_MS", "KickSound", "LANE_DB_AT_UNIT", "LAYER_MEASURED_DB", "LOUDNESS_SOURCE", "LPF_OPEN",
    "LPF_RANGE_HZ", "LPF_TOP_HZ", "Look", "MASTER_DEFAULTS", "MAX_LAYER_AMP", "SET_LEVELER", "TRIM_LAG_S",
    "PAD_DETUNE", "PAD_HAAS_MS", "PAD_PAN_BEATS", "PAD_SPREAD", "PAN_HATS", "PAN_PAD_WIDTH",
    "SECTION_TRIM_DB", "SIDECHAIN_SHAPE", "THEME_TIMBRE", "PAD_FIGURES", "PadFigure", "A9_MIN_GAIN", "A9_STEP_DB",
    "A9_PAD_FLOOR_DB", "A9_BASS_BOOST_DB", "PAN_PSR",
]


# ── Стили аранжировщика (ADR-0153 §2.1, S0): стиль — запись таблицы, генераторы ``arrange`` — функции от неё ──────

#: Форма клубного трека (ADR-0149 §3.3, PR-3a): (имя, такты, энергия 0..10, роли). Имена секций — ключи развития
#: хука ``arrange.hook.DEVELOPMENT``. Интро и аутро поделены под блэнд (``model.blend_bars``, ``Style.blend``):
#: входящий трек начинает хэтами и пэдом под хвостом уходящего, бочка и бас входят ровно на такте свопа — там, где
#: их снимает уходящий. Энергия секции — ось вида (``Style.looks``, PR-7): интро/аутро (блэнд) — ровная прямая
#: бочка, build 4..7, дроп ≥ 7 — полный «насос».
_CLUB_BLEND = (8, 4)  # (phrase_bars, bass_swap_bar) блэнда двух дек (ADR-0149 §3.12, PR-8)
_CLUB_DRUMS = frozenset({"kick", "hats"})
_CLUB_FULL = _CLUB_DRUMS | {"clap", "bass", "pad", "lead"}
_CLUB_FORM: Tuple[Tuple[str, int, int, frozenset], ...] = (
    ("intro", _CLUB_BLEND[1], 2, frozenset({"hats", "pad"})),
    ("intro_low", _CLUB_BLEND[0] - _CLUB_BLEND[1], 2, _CLUB_DRUMS | {"bass", "pad"}),
    # Состав растёт по форме: build без баса — низ возвращается ударом в дроп; брейк-луп (``Style.layer_sections``)
    # вступает только во втором дропе — он плотнее первого. До этой правки build и оба дропа играли одним составом
    # и на записи не различались (робот 05.10: build −23.9 дБ, дроп −24.5, drop2 −26.0; слух Шифу и отзыв
    # эксперта: «нет восхождения, кульминации»).
    ("build", 8, 5, _CLUB_FULL - {"bass"}),
    ("drop", 8, 8, _CLUB_FULL),
    ("break", 8, 4, frozenset({"hats", "pad", "lead"})),
    ("drop2", 8, 9, _CLUB_FULL),
    ("outro", 8 - (_CLUB_BLEND[0] - _CLUB_BLEND[1]), 2, _CLUB_DRUMS | {"bass", "pad"}),
    ("outro_tail", _CLUB_BLEND[0] - _CLUB_BLEND[1], 1, frozenset({"hats", "pad"})),
)
#: Форма первого трека сета — ``dropfirst48`` (ADR-0152 §3.5; #3427): блэнда на входе у него нет, а тему человек ждёт
#: сразу — дроп с хуком целиком идёт после интро (такт 8, ~14 с при 138 BPM, а не такт 16 — ~28 с), build — перед
#: вторым дропом. Интро и аутро те же, что у :data:`_CLUB_FORM`: блэнд с любым следующим треком не меняется.
_CLUB_OPENING_FORM: Tuple[Tuple[str, int, int, frozenset], ...] = (
    *_CLUB_FORM[:2], _CLUB_FORM[3], _CLUB_FORM[4], _CLUB_FORM[2], *_CLUB_FORM[5:])
#: Секции слоёв DJ_Dave (``arrange.samples``, PR-3d): psr-слой — build, дропы и break; брейк-луп — только второй
#: дроп (им он плотнее первого); FX — первая доля дропов (в блэнде PR-8 intro/outro звучат на двух деках).
_CLUB_LAYER_SECTIONS: Mapping[str, Tuple[str, ...]] = {"sample": ("build", "drop", "break", "drop2"),
                                                       "loop": ("drop2",), "fx": ("drop", "drop2")}
#: Прогрессии club по ступеням лада, аккорд на 2 такта (8-тактовая петля).
#: Приёмка 02.10 (A13): при четырёх прогрессиях одна занимала 5 треков из 10 — пул расширен до восьми.
_CLUB_PROGRESSIONS: Tuple[Tuple[int, ...], ...] = (
    (0, 5, 2, 6), (0, 3, 5, 4), (0, 5, 3, 4), (0, 6, 5, 6),
    (0, 2, 6, 5), (0, 3, 6, 2), (0, 4, 5, 3), (0, 6, 3, 5),
)


@dataclass(frozen=True)
class Style:
    """Стиль аранжировщика v2 (ADR-0153 §2.1): все стилевые константы, сгруппированные по читателю.

    Фигуры — ключи реестров генераторов ``arrange.compose`` (``*_GENERATORS``): генератор выбирается по ключу из
    таблицы, а не ветвлением по стилю. Пулы стиля (тембры, каркасы, прогрессии) — то, из чего выбирает разнообразие.
    """

    # Время: окно темпа сета, свинг гоуст-16-х (доля восьмой; величину на сет выбирает план), лады плана.
    bpm: Tuple[int, int]
    swing: Tuple[float, float]
    modes: Tuple[str, ...]
    # Ритм-секция: бочка (ключ :data:`KICK_SOUNDS`), вид секции по энергии, каркасы хэтов/перкуссии.
    kick_sound: str
    looks: Tuple[Tuple[int, Look], ...]
    kits: Mapping[str, Mapping[str, str]]
    # Роли и тембры: коридоры регистров, семьи тембров {семья → {роль → синты}} и семья темы не из таблицы.
    registers: Mapping[str, Tuple[int, int]]
    timbres: Mapping[str, Mapping[str, Tuple[str, ...]]]
    default_timbre: str
    # Фигуры: ключи генераторов ролей. Пэд — пул :data:`PAD_FIGURES` (выбор по сиду трека с историей, ADR-0152
    # PR-5); бас и лид — первый ключ (PR-6).
    bass_figures: Tuple[str, ...]
    pad_figures: Tuple[str, ...]
    lead_figures: Tuple[str, ...]
    # Гармония: звуков в аккорде (терциями лада), прогрессии по ступеням.
    chord_size: int
    progressions: Tuple[Tuple[int, ...], ...]
    # Форма: секции, форма первого трека сета (тема раньше, #3427), блэнд (phrase_bars, bass_swap_bar), секции
    # слоёв сэмплов.
    form: Tuple[Tuple[str, int, int, frozenset], ...]
    opening_form: Tuple[Tuple[str, int, int, frozenset], ...]
    blend: Tuple[int, int]
    layer_sections: Mapping[str, Tuple[str, ...]]
    # Микс: уровни ролей, роли под сайдчейном, LPF-свип секций, стерео ролей.
    role_level_db: Mapping[str, float]
    duck_roles: Tuple[str, ...]
    section_lpf: Mapping[str, Tuple[float, float]]
    lpf_roles: Tuple[str, ...]
    lpf_tail_sections: Tuple[str, ...]
    stereo: Mapping[str, Mapping[str, float]]
    # A9-модель трека (ADR-0152 §4): нижняя граница доли низа каждого дропа в шкале калибровки (``arrange.mix``).
    a9_model_low: float


#: Стили по ключу (ключ — ``ThemeProfile.style``/``SetPlan.style``). ``club`` — сегодняшние клубные таблицы побайтно
#: (``test_style_same_tracks``): 128–138 — решение Шифу 01.10 (ADR-0149 §12 В6, эталон живого диджея ~138); свинг
#: 5–10 % (ADR-0149 §3.4).
STYLES: Mapping[str, Style] = {
    "club": Style(
        bpm=(128, 138), swing=(0.05, 0.10), modes=("minor", "dorian", "phrygian", "major"),
        kick_sound="house", looks=_CLUB_LOOKS, kits=_CLUB_KITS,
        registers=_CLUB_REGISTERS, timbres=_CLUB_TIMBRES, default_timbre="warm",
        bass_figures=("offbeat",), pad_figures=("pumped16", "held", "stabs"), lead_figures=("motif",),
        chord_size=3, progressions=_CLUB_PROGRESSIONS,
        form=_CLUB_FORM, opening_form=_CLUB_OPENING_FORM, blend=_CLUB_BLEND, layer_sections=_CLUB_LAYER_SECTIONS,
        role_level_db=_CLUB_ROLE_LEVEL_DB, duck_roles=_CLUB_DUCK_ROLES, section_lpf=_CLUB_SECTION_LPF,
        lpf_roles=_CLUB_LPF_ROLES, lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS, stereo=_CLUB_STEREO,
        # 0.5 (A9 на роботе) / 0.62 (робот на модель, #3401) — порог ``test_low_share``
        a9_model_low=0.8,
    ),
}
DEFAULT_STYLE = "club"
#: Коридоры регистров и роли под сайдчейном для валидатора модели (I13, ``model.validate``): ``Track`` пока не несёт
#: ключ стиля, поэтому — регистры стиля по умолчанию и объединение ролей сайдчейна всех стилей (ADR-0153 S1+).
REGISTERS: Mapping[str, Tuple[int, int]] = STYLES[DEFAULT_STYLE].registers
DUCK_ROLES: Tuple[str, ...] = tuple(dict.fromkeys(r for st in STYLES.values() for r in st.duck_roles))

__all__ += ["DEFAULT_STYLE", "DUCK_ROLES", "REGISTERS", "STYLES", "Style"]


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
