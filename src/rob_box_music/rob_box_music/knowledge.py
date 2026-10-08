"""Одна таблица знания аранжировщика v2 (ADR-0149 §2.1, §2.2; ADR-0148).

Лады, тоники, роли, палитра синтов и их свойства, коридоры регистров, потолки
уровней, стили аранжировщика (``STYLES``, ADR-0153). Остальные модули пакета только импортируют отсюда.

Значения перенесены из старого кода (ADR-0149 §9); старые таблицы удалены вместе
со старым путём (PR-13b), источник истины — этот модуль.
"""

from __future__ import annotations

import functools
import json
import math
from dataclasses import dataclass, field, replace
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
#: ``toms`` (ADR-0153 S5, рок) — филл томами на стыке секций, файлом пака (``Style.drum_files``), слот лупа деки.
ROLES: Tuple[str, ...] = ("kick", "hats", "clap", "perc", "bass", "pad", "lead", "sample", "loop", "fx", "toms")
TONAL_ROLES: Tuple[str, ...] = ("bass", "pad", "lead")

#: Коридоры регистров MIDI (I13): бас < пэд < лид ≤ 88 (``harmonize.BASS_MIDI_FLOOR``,
#: ``rtttl_compose._LEAD_MAX_CEILING``, ADR-0149 §3.5–§3.7).
#: Пэд: обычный низ — ``registers["pad"][0] + PAD_WIDEN``; коридор пэда на ``PAD_WIDEN`` полутонов ниже. Под низким лидом
#: (верх пэда = низ лида − 3) окно 50..59 не содержит нот C и C#, и трезвучие с ними негде поставить; окно в 12 нот
#: (верх − 11) содержит каждый звук лада, поэтому пэд опускается ниже обычного низа только когда иначе не помещается
#: (``arrange.compose._pad_chords``), остальные треки звучат как раньше. 48 = C3 (бас клуба — до 52).
PAD_WIDEN = 2
_CLUB_REGISTERS: Mapping[str, Tuple[int, int]] = {"bass": (36, 52), "pad": (48, 70), "lead": (58, 84)}
LEAD_MAX_MIDI = 88
#: Хук темы в тональности трека (I12): доля длительности нот лида в ладу не ниже порога; хроматика — проходящие.
HOOK_KEY_FIT_MIN = 0.6
#: Хроматический подход баса — нота вне лада не длиннее этого, доли (ADR-0149 §3.5); стиль с walking-басом держит
#: свой потолок (``Style.approach_max_beats``: подход — целая доля, ADR-0153 §2.2).
APPROACH_MAX_BEATS = 0.5
#: Перевод размера материала в 4/4 клуба (ADR-0154 §3.6, #3517) — данные, не ветки кода: размер →
#: ``(долей клуба на такт материала, узлы)``. Узлы ``(доля в такте материала, доля в тактах клуба)`` от ``(0, 0)``
#: до ``(длина такта, конец музыки)`` задают кусочно-линейное отображение (``material.MeterMap``); хвост такта клуба
#: после конца музыки — пауза. Размеры одного пульса (4/4, 2/2; 2/4 — два такта в один 4/4) — во всех режимах.
_SAME_PULSE: Mapping[Tuple[int, int], tuple] = {(4, 4): (4.0, ((0, 0), (4, 4))), (2, 2): (4.0, ((0, 0), (4, 4))),
                                                (2, 4): (2.0, ((0, 0), (2, 2)))}
#: Группа из трёх восьмих (пунктирная четверть) → доля клуба, 16-е «2+1+1»: первая восьмая целиком, вторая и
#: третья — шестнадцатые (шаффл на прямой сетке).
_GROUP_211 = ((0, 0), (0.5, 0.5), (1.0, 0.75), (1.5, 1.0))
#: Две группы (такт 6/8) → полтакта клуба «3+3+2» шестнадцатых (тресильо): восьмые — шестнадцатыми, последняя
#: восьмая тянется на две шестнадцатых; вторая пунктирная доля встаёт на «и» первой доли (синкопа хабанеры).
_TRESILLO = ((0, 0), (2.5, 1.25), (3.0, 2.0))


def _tile(knots: tuple, times: int, src: float, dst: float) -> tuple:
    """Узлы группы, повторённые ``times`` раз подряд (группа ``src`` долей материала → ``dst`` долей клуба)."""
    return ((0, 0),) + tuple((k * src + a, k * dst + b) for k in range(times) for a, b in knots[1:])


METER_MODES: Mapping[str, Mapping[Tuple[int, int], tuple]] = {
    # Не переводить: материал в 3/4, 6/8, 3/8, 12/8 негоден в хук (``hook.material_unfit``).
    "unfit": dict(_SAME_PULSE),
    # В5 (б): такт 3/4 с начала такта 4/4, 4-я доля — пауза (6 долей музыки + 2 тишины под ×2 — хромает, #3517).
    "pause": {**_SAME_PULSE, (3, 4): (4.0, ((0, 0), (3, 3)))},
    # Растяжение ×4/3: такт в такт, доли 2 и 3 на 2⅓ и 3⅔ — квантование к 16-м даёт триольное ощущение.
    "stretch": {**_SAME_PULSE, (3, 4): (4.0, ((0, 0), (3, 4))), (6, 8): (4.0, ((0, 0), (3, 4))),
                (3, 8): (2.0, ((0, 0), (1.5, 2))), (12, 8): (8.0, ((0, 0), (6, 8)))},
    # Долгая первая доля «2+1+1» (вальс «в четыре»): 1-я доля 3/4 — половинная, 2-я и 3-я — на 3 и 4; сложные
    # размеры — пунктирная четверть = доля клуба, внутри «2+1+1» шестнадцатых. Всё остаётся на сетке 16-х.
    "long": {**_SAME_PULSE, (3, 4): (4.0, ((0, 0), (1, 2), (3, 4))), (6, 8): (2.0, _tile(_GROUP_211, 2, 1.5, 1)),
             (3, 8): (1.0, _GROUP_211), (12, 8): (4.0, _tile(_GROUP_211, 4, 1.5, 1))},
    # Долгая третья доля «1+1+2» (затакт тянется к следующей сильной): 3/4 — 3-я доля половинная; 6/8 и 12/8 —
    # тресильо «3+3+2»; 3/8 — третья восьмая на две шестнадцатых.
    "lift": {**_SAME_PULSE, (3, 4): (4.0, ((0, 0), (2, 2), (3, 4))), (6, 8): (2.0, _TRESILLO),
             (3, 8): (1.0, ((0, 0), (1.0, 0.5), (1.5, 1.0))), (12, 8): (4.0, _tile(_TRESILLO, 2, 3, 2))},
}
#: Режим перевода из :data:`METER_MODES`. ``"unfit"`` — пока Шифу не принял перевод на слух (#3517).
TRIPLE_METER_MODE = "unfit"

#: Ступени энергии трека в сете (ADR-0147): 1 — интро/спад, 5 — пик.
ENERGY_LEVELS: Tuple[int, ...] = (1, 2, 3, 4, 5)
#: Смещение громкости трека по энергии, дБ, ПОСЛЕ динамики мастер-шины (ADR-0147 §3.2; проводка — PR-7).
#: Источник для старого ``core/club_energy`` (там импорт отсюда, ADR-0149 §4.6 в).
ENERGY_TRIM_DB: Mapping[int, float] = {1: -9.0, 2: -6.0, 3: -4.0, 4: -2.0, 5: 0.0}
#: Волна энергии сета по номеру трека (ADR-0147 §3.4, ADR-0149 §4.6): трек 1 — превью/интро (2),
#: дальше период 5 «разгон → пик → спад»: 2 3 4 5 4 | 2 3 4 5 4 | …; последний трек сета — спад к ``ENERGY_WAVE[0]``.
ENERGY_WAVE: Tuple[int, ...] = (2, 3, 4, 5, 4)
#: Длина DJ-сета, треков, если человек не назвал число (решение Шифу 06.10: сет без конца дошёл до 53-го трека).
SET_TRACKS = 10
#: Самый длинный сет по просьбе человека («на два часа» зажимается сюда): ≈ 75 мин при ``SET_TRACK_SECONDS``.
SET_MAX_TRACKS = 60
#: Средняя длина трека сета, с: «сет на полчаса» → треки (живой прогон 06.10: 5–6 треков ≈ 7 мин).
SET_TRACK_SECONDS = 75
#: Межсетовая свежесть хуков (#3399, живой факт 07.10: одна фраза темы — popcorn/axelf первым в каждом сете): мелодия,
#: звучавшая в последних ``HOOK_FRESH_SETS`` сетах (любая её версия — общий контур начала), уходит в очереди хуков
#: за несыгранные — и на ПЕРВОМ треке; хук, открывший прошлый сет, открывает новый только когда других нет.
#: Три сета: тема из 8 находок (``rtttl.THEME_HOOKS``) расходуется за два сета по 3–4 трека, третий — уже по давности,
#: а пул (21) и жанр (30) держат три сета без повтора мелодии; сыгранное раньше — забыто, снова «свежее».
HOOK_FRESH_SETS = 3
#: Версии одной мелодии внутри сета (#3497, живой сет 07.10: tetris, tetris_2 подряд): после трека с мелодией её версии
#: (общий ``rtttl.contour`` — ключ ``consensus_order`` и межсетовой свежести) не ставятся ещё ``HOOK_TUNE_GAP``
#: треков, пока есть другие мелодии; одна мелодия на всю тему — версии идут по очереди, иначе нечего ставить.
HOOK_TUNE_GAP = 2
#: Сколько треков прошлых сетов помнит память сетов (``SetMemory``): три полных сета с заранее собранным N+1.
HOOK_MEMORY_TRACKS = HOOK_FRESH_SETS * (SET_TRACKS + 1)
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
    "toms": -10.0,  # ADR-0153 S5: филл томами — как малый
}

#: Рисунки бочки, 16 шагов (``core/club_arranger.KICK_PATTERNS``).
KICK_PATTERNS: Mapping[str, str] = {
    "by_design": "X..X..X..(.X)X.....",
    "four_on_floor": "X...X...X...X...",
    "half_time": "X.........X.....",
    "breakbeat": "X.X.......X..X..",
    "outrun": "X...X...X...X..X",
    # ADR-0153 S4 (lo-fi): бочка на 1, «и» 2, «и» 3 — сетка восьмых (оффбит-восьмые качает свинг стиля); в build — 1 и «и» 3
    "boombap": "X.....X...X.....",
    "lazy": "X.........X.....",
    # ADR-0153 S5 (рок): 1, «и» 2, 3 — рок-бит (§3); 1, 3, «и» 3 — прямой; гранж — плюс «и» 4 (тяжелее); half — 1 и 3
    "rock": "X.....X.X.......",
    "rock_straight": "X.......X.X.....",
    "grunge": "X.....X.X.....X.",
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
    #: Хуки строки идут в общий пул тем без находок (``theme.HOOK_POOL``). ``False`` — праздничные и детские
    #: мелодии: только по своей теме, «интерстеллар» не получает «Happy Birthday» (#3455).
    pooled: bool = True
    #: Стиль сета темы, когда человек стиль не назвал (``dj_set(style=auto)``, ADR-0153 §4.2): ключ :data:`STYLES`;
    #: ``None`` — :data:`DEFAULT_STYLE`. Слова стиля в реплике важнее строки (``theme.style_for``).
    style: Optional[str] = None


#: Темы сета → материал. Порядок строк решает ничью по числу совпавших основ.
THEMES: Mapping[str, ThemeRow] = {
    "space": ThemeRow(
        ("косм", "звёзд", "звезд", "галакт", "планет", "ракет", "луна", "марс", "space", "star"), (128, 132), "minor",
        ("starwars_6", "starwars_2", "the_x_files", "xfiles1", "startrek_3", "startrek_5",
         "deepspac", "spacecha", "doctorwh")),
    "cyber": ThemeRow(
        ("кибер", "робот", "матриц", "неон", "техно", "будущ", "cyber", "robot"), (134, 138), "phrygian",
        ("axelf_3", "axel_f", "robotroc", "robot", "popcorn", "popcorn_6", "aroundth_3"), style="synthwave"),
    "kids": ThemeRow(
        ("детск", "дети", "детей", "праздн", "рожден", "мульт", "игрушк", "kids"), (128, 132), "major",
        ("happybir", "chickend", "macarena", "yellowsu", "teletubb", "bobthebu", "spongebo", "flintsto_2",
         "barbiegi"), pooled=False, style="chiptune"),
    "slavic": ThemeRow(
        ("славян", "русск", "народн", "калинк", "балалайк", "деревен", "казач"), (130, 136), "minor",
        ("kalinkav", "kalinkav_2", "tetris", "tetris_2")),
    "winter": ThemeRow(
        ("новогод", "новый", "зим", "рождеств", "ёлк", "елк", "снег", "мороз"), (128, 132), "major",
        ("jinglebe_6", "jinglebe_3", "lastchri_5", "haveyour"), pooled=False),
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
    # обрамление темы-перечисления 06.10 («мегасет для игроков из RTTTL-мелодий разных игр — …, Dendy и другие,
    # чтобы не повторяться»): части из этих слов не ищутся и не попадают в «не найдено»
    "мегасет", "мега", "игра", "игры", "игр", "игрок", "игроки", "игроков", "разных", "разные", "другие", "другое",
    "прочие", "повторяться", "rtttl", "пати", "чиптюн",
    # стиль и формат сета, а не мелодия (живой сет 06.10 «ретро 8-бит: Mario, Tetris, …» искал «8» и играл
    # «1812 Overture»): жанр без композитора, «ретро», публика. Цифра+«бит» — :data:`SEARCH_STYLE_PATTERNS`
    "ретро", "retro", "chiptune", "классика", "классический", "классическая", "classic", "classical", "геймер",
    "геймеры", "gamer", "gamers",
    # оценка, а не название (07.10 «играй мелодии из топовых фильмов» искал «топовых»)
    "топ", "топовый", "топовые", "топовых", "лучшие", "лучших", "популярные", "популярных", "известные", "известных",
    "мелодии", "мелодий",
    "the", "of", "a", "an", "and", "in", "on", "for", "to", "with", "from", "at", "by", "de", "la", "le", "du", "des",
)
#: Жанр словами темы → метка каталога RTTTL (``tags`` записи): «классическая музыка» — отдельная часть темы, а не
#: стоп-слово (живой сет 06.10 «денди и классическая музыка» играл только игры). Ключ — начало слова темы.
#: «кино», «фильмов», «movie» → метка ``movie`` (190 записей архива: Batman, Titanic, Rocky, Terminator; 07.10
#: «вечеринка любителей кино, играй мелодии из топовых фильмов» искала фразы целиком и играла случайный пул).
GENRE_TAGS: Mapping[str, str] = {
    "классик": "classical", "классическ": "classical", "classic": "classical",
    "кино": "movie", "фильм": "movie", "кинофильм": "movie", "movie": "movie", "film": "movie",
}
#: Метка жанра каталога → жанры PDMX в колонке ``score_index.genres`` (через «-»: ``classical-soundtrack``): по ним
#: часть темы из одного жанра ищет партитуры (#3512; «музыка из фильмов» → ``soundtrack``, 1132+263 партитуры 07.10).
SCORE_GENRES: Mapping[str, Tuple[str, ...]] = {"classical": ("classical",), "movie": ("soundtrack",)}
#: Слова при жанре, не называющие мелодию («classical music», «классической музыки»; «Music» — ещё и название записи
#: архива, поэтому не стоп-слово поиска вообще). В теме из слов реплики остаются при слове жанра
#: (``engine.theme_grounding``: «денди и классической музыки»).
GENRE_FILLER: Tuple[str, ...] = ("music", "музыка", "музыки", "музыку", "музыкой", "музыке")
#: Записи, которые метка каталога относит к жанру зря (имя исполнителя содержит «Bach»/«Strauss» и т. п.).
GENRE_NOT: Mapping[str, Tuple[str, ...]] = {
    "classical": ("raindrop", "bachelor", "mudianto", "mundaint", "mundiant", "mundiant_2", "gravelpi", "gravelpi_2",
                  "travelti", "starspla"),
}
#: Записи жанра, у которых в каталоге нет метки («1812 Overture» — самая узнаваемая классика архива).
GENRE_EXTRA: Mapping[str, Tuple[str, ...]] = {"classical": ("1812over", "1812over_2")}
#: Стиль и формат сета словами с цифрой — вырезаются из запроса до разбора на слова (``engine.search.terms``):
#: «8-бит», «8 бит», «8-битный», «16-bit», «8bit». Цифра из них искала «8» в «1812 Overture» и «Sk8er Boi» (06.10).
#: Те же слова ВЫБИРАЮТ стиль (ADR-0153 S2): регекс → ключ :data:`STYLES` (``theme.match_style_text``); темой они
#: не становятся по-прежнему. Слова стиля без цифры — :data:`STYLE_WORDS`.
STYLE_PATTERNS: Mapping[str, str] = {
    r"\b\d+\s*-?\s*(?:бит|bit)\w*": "chiptune",
    # ADR-0153 S3: «драм-н-бейс», «драм энд бейс», «drum and bass» (STT пишет и словами врозь), «брейк бит».
    r"(?<!\w)(?:драм|drum)\s*-?\s*(?:(?:н|эн|энд|and|n|и)\s*-?\s*)?(?:бейс|бэйс|bass)\w*": "dnb",
    r"(?<!\w)(?:брейк|break)\s*-?\s*(?:бит|beat)\w*": "breaks",
    # ADR-0153 S4: «лоу-фай», «лоу фай», «lo-fi», «lo fi» (STT пишет и через дефис, и врозь).
    r"(?<!\w)(?:лоу|ло|lo)\s*-?\s*(?:фай|fi)(?!\w)": "lofi",
    # ADR-0153 S5: «рок», «рока/року/роком/роке», «рок-н-ролл», «рок энд ролл», «хард-рок», «rock», «rock'n'roll»,
    # «hard rock» — слово целиком: «роковой», «рокки», «рокот», «rocket», «rocky» — не стиль.
    r"(?<!\w)(?:(?:хард|hard)\s*-?\s*)?(?:рок(?:а|у|ом|е)?|rock)"
    r"(?:\s*-?\s*'?(?:н|эн|энд|and|n)'?\s*-?\s*(?:ролл|roll)\w*)?(?!\w)": "rock",
}
SEARCH_STYLE_PATTERNS: Tuple[str, ...] = tuple(STYLE_PATTERNS)


@dataclass(frozen=True)
class SynthTraits:
    """Свойства SynthDef: ``tail`` — held|fixed|short, ``register`` — роль, ``tail_note`` — текст."""

    tail: str
    register: str
    tail_note: str

    @property
    def long_release(self) -> bool:
        return self.tail != "short"


#: Синт с фильтром, чей срез растёт с высотой ноты и уходит за Найквист 8 кГц (выход 16 кГц): выше этой ноты фильтр
#: неустойчив. Значение — старшая нота (MIDI), при которой срез ещё ≤ 7.9 кГц. ``cs80lead`` (renardo_lib):
#: ``LPF.ar(osc, fenv·freq·12 + 100)`` — сам SynthDef пишет «keep ffreq below nyquist»; 7900/12 = 658 Гц ≈ MIDI 76, коридор
#: лида — до 84. Робот 07.10 (ADR-0153 S2, synthwave): оба трека с ``cs80lead`` — серии цифрового нуля до 6.6 с и доля
#: энергии выше 4.5 кГц до 0.9 (у клуба 0.007); 4 трека без него — чисто. Остальные синты палитры с фильтром от ``freq``
#: (замер по .scd на роботе, #3502): ``brass`` ``Resonz(freq·5)`` → 1580 Гц = MIDI 91; ``supersawlead`` ``baseFreq·4`` → 95;
#: ``strangerarp`` ``·3.5`` → 100; у остальных (``imperialbrass``/``strangerbrass``/``marchstrings``/``warmpad``/
#: ``strangerpulsepad``/``retrobass``, ``·2…3.2``) предел ≥ MIDI 106, у ``tb303`` срез ограничен ``clip(…, 7200)`` — вне
#: коридоров ролей. Одно место применения — ``arrange.mix.role_palette``: синт не берётся в роль, чей коридор
#: (``Style.registers``) выше предела; никакого ``if style``.
NYQUIST_MAX_MIDI: Mapping[str, int] = {"cs80lead": 76, "brass": 91, "supersawlead": 95, "strangerarp": 100}


@dataclass(frozen=True)
class LeadClarity:
    """Читаемость синта темой — замер ``scripts/music/loudness_nrt_v2.py --clarity`` (NRT 16 кГц, моно-сумма L+R,
    132 BPM, ``amp`` цели роли лида): ``attack_ms`` — от начала ноты до пик − 3 дБ; ``tail_ms`` — от конца ``sus``
    восьмой до пик − 30 дБ (1588 — не затух за окно 1.8 с); ``purity`` — доля энергии долгой ноты в ±15 ц от сетки
    ``k·f0/2`` (хорус и шумовой синт её размазывают); ``bright_share`` — доля энергии 1–4 кГц на теме «В пещере горного
    короля» (присутствие над пэдом и басом)."""

    attack_ms: float
    tail_ms: float
    purity: float
    bright_share: float


#: Замер 07.10 (katana, образ ``voice-assistant`` с SynthDef-ами робота; сырые числа — тело PR). Почему тема у первых
#: трёх стилей «какофония, сильное эхо, размазана» (отзыв Шифу 07.10 о «Горном короле»):
#: ``supersawlead``/``strangerarp``/``strangerbrass``/``imperialbrass`` — ``Env.adsr`` без ``gate``: огибающая не
#: отпускается, нота держится до ``sus·8`` ``startSound`` — восьмые ложатся друг на друга облаком (хвост ≥ 1.6 с);
#: у ``supersawlead`` ещё 5 голосов ±1.5 % (±26 ц, чистота 0.51). ``hoover`` — 18 пил ±3.5 % (±60 ц) с задержками
#: 0–10 мс (чистота 0.09: флэнжер-хорус). ``epiano`` — ``CombL`` с затуханием ``50·amp`` с, атака 0.1 с: атака
#: 50 мс, хвост 148 мс, доля 1–4 кГц 0.09 — как у пэдов (тонет). ``rave`` — ``Gendy1`` между ``freq`` и ``2·freq``
#: (высоты нет, 0.14). ``kalimba`` — релиз 2.5–3.5 с; ``rhpiano`` — тусклый (0.026). Пэды на том же уровне —
#: доля 1–4 кГц ≤ 0.123 (``saw`` арпеджио), ``sinepad`` 0.0, ``warmpad`` 0.023, ``strings`` 0.043.
LEAD_CLARITY: Mapping[str, LeadClarity] = {
    "pluck": LeadClarity(0.0, 62.7, 0.997, 0.272), "blip": LeadClarity(0.0, 0.0, 0.988, 0.486),
    "arpy": LeadClarity(5.0, 0.0, 0.827, 0.636), "brass": LeadClarity(10.0, 0.0, 1.0, 0.704),
    "orient": LeadClarity(5.0, 2.7, 0.976, 0.674), "keys": LeadClarity(5.0, 107.7, 0.977, 0.224),
    "saw": LeadClarity(5.0, 12.7, 1.0, 0.298), "pulse": LeadClarity(5.0, 12.7, 1.0, 0.136),
    "square": LeadClarity(10.0, 7.7, 1.0, 0.123), "varsaw": LeadClarity(65.0, 0.0, 0.972, 0.354),
    "viola": LeadClarity(40.0, 0.0, 0.844, 0.557), "karp": LeadClarity(0.0, 202.7, 0.664, 0.058),
    "marimba": LeadClarity(35.0, 412.7, 0.631, 0.142), "sitar": LeadClarity(5.0, 182.7, 0.174, 0.513),
    "epiano": LeadClarity(50.0, 147.7, 0.995, 0.091), "rhpiano": LeadClarity(5.0, 27.7, 0.994, 0.026),
    "kalimba": LeadClarity(10.0, 1422.7, 0.998, 0.018), "hoover": LeadClarity(5.0, 87.7, 0.090, 0.213),
    "cs80lead": LeadClarity(205.0, 1182.7, 0.398, 0.247), "rave": LeadClarity(10.0, 2.7, 0.136, 0.638),
    "supersawlead": LeadClarity(0.0, 1587.7, 0.514, 0.263), "strangerarp": LeadClarity(0.0, 1587.7, 0.723, 0.252),
    "strangerbrass": LeadClarity(5.0, 1587.7, 0.987, 0.353),
    "imperialbrass": LeadClarity(5.0, 1587.7, 0.985, 0.335),
}
#: Пороги читаемости темы (ADR-0153 §2.4). Атака — щелчок ноты, а не нарастание; хвост — не длиннее 16-й при 132 BPM
#: (114 мс): восьмые темы не перекрываются; чистота — без хоруса-размазни; яркость — доля 1–4 кГц выше самого
#: яркого пэда (0.123) на том же уровне: тема слышна над подкладкой («инструмент яркий», Шифу 07.10).
THEME_ATTACK_MAX_MS = 30.0
THEME_TAIL_MAX_MS = 120.0
THEME_PURITY_MIN = 0.8
THEME_BRIGHT_MIN = 0.13
#: Роли, которые играют тему/хук (``arrange.compose._lead``: лид — всегда хук или мотив).
THEME_ROLES: Tuple[str, ...] = ("lead",)


def theme_lead_ok(synth: str) -> bool:
    """Синт читаемо играет тему: замерен (:data:`LEAD_CLARITY`) и проходит все пороги. Без замера — нет."""
    c = LEAD_CLARITY.get(synth)
    return c is not None and (c.attack_ms <= THEME_ATTACK_MAX_MS and c.tail_ms <= THEME_TAIL_MAX_MS
                              and c.purity >= THEME_PURITY_MIN and c.bright_share >= THEME_BRIGHT_MIN)


#: Читаемые лиды темы — одно место правила (применяет ``arrange.mix.role_palette``).
THEME_LEAD_OK: frozenset = frozenset(s for s in LEAD_CLARITY if theme_lead_ok(s))

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
#: (``club_timbre.TIMBRE_EXTRAS``). Первый — эталонный. ADR-0152 PR-6: лиды ``rhpiano``, ``kalimba``, ``hoover``,
#: ``keys``, ``cs80lead`` и басы ``jbass``, ``wobblebass``, ``subbass`` — замер #3430 (громкость и полосы), прелоад
#: робота (``sclang_robot_2026-09-23.log``: «SynthDef in scsynth»), досылка ``CRITICAL_SYNTHS``.
SYNTH_PALETTE: Mapping[str, Tuple[str, ...]] = {
    "lead": ("pluck", "blip", "arpy", "karp", "marimba", "sitar", "epiano", "brass", "orient", "viola",
             "rhpiano", "kalimba", "hoover", "keys", "cs80lead",
             # ADR-0153 S1: лиды стиля ``rave`` (замер #3430, досылка ``CRITICAL_SYNTHS``).
             "rave", "supersawlead",
             # ADR-0153 S2: лиды synthwave/chiptune (замер #3430/#3422, досылка ``CRITICAL_SYNTHS``).
             "strangerarp", "saw", "pulse", "varsaw"),
    "bass": ("bass", "retrobass", "dub", "jbass", "wobblebass", "subbass", "tb303",
             # ADR-0153 S2: 8-битный бас chiptune (``octave8``)
             "pulse"),
    "pad": ("sinepad", "warmpad", "space", "ambi", "strangerpulsepad", "strings",
            # ADR-0153 S2: арпеджио и стэбы synthwave/chiptune (замер 07.10 в рамке пэда)
            "pulse", "square", "blip", "saw"),
}

#: Рисунок хэтов/клэпа — это сэмпл-плееры ``play``; синт ударных один.
PLAY_SYNTH = "play"

#: Символ сэмпла ``play()`` ударной роли (Renardo: X — бочка, - — хэт, * — клэп, o — малый).
#: ``toms`` звучит только файлом пака (ADR-0153 S5); символ ``m`` (томы дефолтного пака) — для полноты таблицы.
DRUM_SYMBOLS: Mapping[str, str] = {"kick": "X", "hats": "-", "clap": "*", "perc": "o", "toms": "m"}

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

#: Интервалы аккордов материала партитуры от примы (``material.CHORD_QUALITIES``, ADR-0154 §3.1).
CHORD_INTERVALS: Mapping[str, Tuple[int, ...]] = {
    "maj": (0, 4, 7), "min": (0, 3, 7), "dim": (0, 3, 6), "aug": (0, 4, 8), "sus": (0, 5, 7),
    "dom7": (0, 4, 7, 10), "maj7": (0, 4, 7, 11), "min7": (0, 3, 7, 10), "other": (0,)}
#: Качество аккорда — данные (аудит 07.10 Ф2, #3530): аккорд трека — ступень лада + ключ :data:`CHORD_INTERVALS`
#: (``model.Chord.quality``), пэд и бас играют его тоны (:func:`chord_pitch_classes`). Диатоническое качество ступени —
#: трезвучие этой таблицы, совпавшее с терцовой стопкой лада (:func:`diatonic_quality`), а не своя копия по ладам.
TRIAD_QUALITIES: Tuple[str, ...] = ("maj", "min", "dim", "aug")
#: Качество аккорда автора, которое трек сохраняет (перенос в тональность трека — та же ступень и то же качество: V в
#: миноре остаётся мажорной, II# — мажорной, аудит П4); ``other`` (один звук) — диатоническое трезвучие ступени.
AUTHOR_QUALITIES: frozenset = frozenset(CHORD_INTERVALS) - {"other"}
#: Качества ступени на путях без аккорда автора (Витерби, педаль, каденция) СВЕРХ диатонического: лад → ступень →
#: качества; слот берёт то, что лучше покрывает звучащую мелодию (``arrange.harmony.qualify``), ничья —
#: диатоническое. Минор и дорийский — гармонический V (мажорная доминанта с вводным тоном): вводный тон мелодии не
#: звучит над натуральной VII (аудит П4).
DEGREE_QUALITIES: Mapping[str, Mapping[int, Tuple[str, ...]]] = {"minor": {4: ("maj",)}, "dorian": {4: ("maj",)}}


def diatonic_quality(mode: str, degree: int) -> str:
    """Качество трезвучия ступени ``degree`` семиступенного лада ``mode`` (ключ :data:`TRIAD_QUALITIES`)."""
    scale = SCALES[mode]
    shape = tuple((scale[(degree + k) % 7] - scale[degree]) % 12 for k in (2, 4))
    return next(q for q in TRIAD_QUALITIES if CHORD_INTERVALS[q][1:] == shape)


#: Расширения аккорда над септимой (``Style.chord_size`` 5 — нона, ADR-0153 S6) — терции лада, кроме этих интервалов
#: от примы: малая нона (b9) в тесном расположении пэда — полутон с примой; у минорных и мажорных септаккордов её не
#: берут (V7b9 минора — тоже: доминанта звучит септаккордом).
TENSION_AVOID: Tuple[int, ...] = (1,)


def chord_pitch_classes(root: int, mode: str, degree: int, quality: Optional[str] = None,
                        size: int = 3) -> Tuple[int, ...]:
    """Тоны аккорда ступени ``degree`` лада ``mode`` от тоники ``root``: (прима, терция, квинта[, септима]).

    ``quality`` — ключ :data:`CHORD_INTERVALS` (``None`` или пусто — терции лада, как диатонический аккорд). Тонов ``size``
    (``Style.chord_size``): септаккорд качества — свои 4 тона, трезвучие при ``size`` 4 — с септимой лада; ``size`` 5 —
    плюс нона лада, если она не из :data:`TENSION_AVOID` (тогда тонов 4)."""
    scale = SCALES[mode]
    base = root + scale[degree % len(scale)]
    if not quality:
        tones = [(root + scale[(degree + 2 * k) % len(scale)]) % 12 for k in range(min(size, 4))]
    else:
        tones = [(base + i) % 12 for i in CHORD_INTERVALS[quality][:size]]
        if len(tones) < size:
            tones.append((root + scale[(degree + 6) % len(scale)]) % 12)
    for k in range(4, size):  # расширения над септимой (нона, ``size`` 5 — джаз): терции лада без :data:`TENSION_AVOID`
        tone = (root + scale[(degree + 2 * k) % len(scale)]) % 12
        if (tone - base) % 12 not in TENSION_AVOID:
            tones.append(tone)
    return tuple(tones)


#: Переходы ступеней корпуса партитур (ADR-0154 §3.4, Н5/Н6) — данные, выученные офлайн
#: (``scripts/music/research/score_markov_harmony.py --write-table``; провенанс — в файле): лад ("major"/"minor") →
#: ``start`` — P(первая ступень), ``next`` — P(ступень b | ступень a), 7×7, строки в сумме 1.
_TRANSITIONS_DATA = json.loads(
    (Path(__file__).resolve().parent / "data" / "progression_transitions.json").read_text(encoding="utf-8"))
PROGRESSION_TRANSITIONS: Mapping[str, Mapping[str, tuple]] = {
    mode: {"start": tuple(t["start"]), "next": tuple(tuple(row) for row in t["next"])}
    for mode, t in _TRANSITIONS_DATA["tables"].items()}
PROGRESSION_TRANSITIONS_PROVENANCE: Mapping[str, object] = _TRANSITIONS_DATA["provenance"]


@dataclass(frozen=True)
class HookHarmony:
    """Гармония под звучащую мелодию секции (аудит 07.10 Ф1, #3529; ADR-0154 §3.3–3.4): одна гармонизация —
    Витерби (``arrange.harmony.viterbi``) по ``PROGRESSION_TRANSITIONS`` с эмиссией «доля мелодии в тонах аккорда».

    ``slot_bars`` — гармонический ритм (аккорд на такт: потолок лучшей триады на такт 0.76 против 0.67 на 2 такта,
    аудит). ``melody_weight`` — вес эмиссии против log P перехода (аудит ``weights`` на 108 RTTTL-треках: 1 → 0.67,
    2 → 0.70, 4 → 0.74, 8 → 0.77; 1 из Н6 подобран под восстановление аккордов автора, а не под покрытие мелодии).
    ``strong_weight`` — во столько раз весомее длительность ноты, взятой на сильной доле (1 и 3 доли такта).
    ``b9_weight`` — штраф за долю (по тем же весам) нот сильных долей на полутон выше тона аккорда (малая нона —
    самое резкое неаккордовое созвучие; аудит: 82 % треков). ``hold_p`` — P(аккорд держится следующий слот):
    таблица корпуса — переходы между РАЗНЫМИ аккордами (I→I 0.007), гармонический ритм у неё не выучен, поэтому
    удержание — отдельное число (подобрано на матрице аудита 1200 треков, не на корпусе). ``style_bonus`` — к
    log P перехода, который есть в петлях ``Style.progressions`` (лицо стиля как априорный бонус, а не шаблон поверх
    мелодии). Числа меняются здесь и только здесь."""

    slot_bars: int = 1
    melody_weight: float = 4.0
    strong_weight: float = 2.0
    b9_weight: float = 3.0
    hold_p: float = 0.25
    style_bonus: float = 0.5


HOOK_HARMONY = HookHarmony()
#: Гармония секций развития хука (аудит Ф1, П2): секция → множитель слота ``HOOK_HARMONY.slot_bars`` — ``break``
#: (хук вдвое медленнее) гармонизуется слотами вдвое длиннее; :data:`PEDAL` — педаль: один аккорд на всю секцию из
#: :data:`PEDAL_DEGREES` (``build`` крутит начало хука — педаль на тонике или доминанте идиоматична для подъёма).
#: Секции, которых нет в таблице, — гармония под их мелодию слотами ``slot_bars``.
PEDAL = 0
#: Смены темы (ADR-0153 S6, соло джаза): секция идёт по аккордам темы целиком (нет темы — петли хука) с начала, её
#: последние такты — каденция стиля (``Style.cadence``); мелодию секции строит солист по этим аккордам, а не хук.
CHANGES = -1
SECTION_HARMONY: Mapping[str, int] = {"build": PEDAL, "build2": PEDAL, "break": 2, "break2": 2, "solo": CHANGES}
PEDAL_DEGREES: Tuple[int, ...] = (0, 4)
#: Уменьшённое трезвучие Витерби берёт только вводное (прима на ``DIM_ROOT`` полутонов от тоники — vii° мажора) и
#: только перед разрешением в ``DIM_RESOLUTION`` (vii° → I); ii° минора, vi° дорийского, v° фригийского — нет;
#: последним в незамкнутой цепочке — нет (аудит П6: 12 % треков держали ум. 2 такта). Аккорд автора — как у автора.
DIM_ROOT = 11
DIM_RESOLUTION = 0
#: Тоны баса корпуса партитур (ADR-0154 §3.4, Н7) — данные, выученные офлайн
#: (``scripts/music/research/score_bass_tones.py --write-table``; провенанс — в файле): лад ("major"/"minor") →
#: доли ноты баса против аккорда ``root``/``third``/``fifth``/``other`` (в сумме 1) и ``approach_per_bar`` — подходов
#: полутоном к первой доле на такт. Политика баса из материала — ``arrange.bass.material_tones``.
_BASS_TONES_DATA = json.loads(
    (Path(__file__).resolve().parent / "data" / "bass_tones.json").read_text(encoding="utf-8"))
BASS_TONE_RELATIONS: Tuple[str, ...] = ("root", "third", "fifth", "other")
BASS_TONES: Mapping[str, Mapping[str, float]] = {
    mode: {k: float(t[k]) for k in (*BASS_TONE_RELATIONS, "approach_per_bar")}
    for mode, t in _BASS_TONES_DATA["tables"].items()}
BASS_TONES_PROVENANCE: Mapping[str, object] = _BASS_TONES_DATA["provenance"]
#: Роли сэмпла: ``perc`` — удар по сетке каркаса, ``loop`` — луп, растянутый на ``beats`` долей, ``fx`` —
#: одиночный акцент на границе секции (пик в начале файла), ``riser`` — нарастание (пик в конце: tn1hit2, пик на
#: 81 % длины, замер 02.10), ``vox``/``bass``/``synth`` — тональные стемы, ``kick`` — бочки (бочку решает стиль),
#: ``break`` — ударный брейк пака (амен), нарезаемый по сетке 16-х (``arrange.samples.breakbeat_chop``, ADR-0153 S3),
#: ``snare``/``hat`` — удары кита пака (ADR-0153 S4: малый и хэт lo-fi, :data:`PACK_DRUMS`), ``tom``/``ride``/``crash``
#: — томы, райд и крэш рок-кита Muldjord (ADR-0153 S5).
SAMPLE_ROLES: Tuple[str, ...] = ("perc", "loop", "fx", "riser", "vox", "bass", "synth", "kick", "break", "snare",
                                 "hat", "tom", "ride", "crash")
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


#: Брейки ресурсного пака Sonic Pi (CC0; ADR-0153 §8 В3, решение Шифу 07.10; каталог ``data/sample_sonicpi.json``,
#: #3496): длина брейка в долях ОРИГИНАЛА — темп файла = доли·60/секунды (амен — такт 4/4 при 136.9 BPM, полный амен —
#: 4 такта при 140.0, ``loop_breakbeat`` — такт при 126.0; длины — каталог, доли — описание пака Sonic Pi). Уровень —
#: ``rms_db``/``peak_db`` каталога (soundfile по скачанным файлам 07.10), на 16 кГц робота не слушано. Брейк без строки
#: здесь в каталог v2 не идёт (темп неизвестен — ``rate`` под сет не посчитать).
PACK_BREAK_BEATS: Mapping[str, int] = {
    "sonicpi_loop_amen": 4, "sonicpi_loop_amen_full": 16, "sonicpi_loop_breakbeat": 4}
_SONICPI_DATA = json.loads(
    (Path(__file__).resolve().parent / "data" / "sample_sonicpi.json").read_text(encoding="utf-8"))


def _pack_break(name: str, meta: Mapping[str, object], pack_dir: str) -> SampleInfo:
    seconds = float(meta["seconds"])
    return SampleInfo(name, str(meta["group"]), "break", seconds, int(meta["channels"]), f"{pack_dir}/{meta['path']}",
                      float(meta["peak_db"]), float(meta["rms_db"]), round(PACK_BREAK_BEATS[name] * 60.0 / seconds))


_MULDJORD_DATA = json.loads(
    (Path(__file__).resolve().parent / "data" / "sample_muldjord.json").read_text(encoding="utf-8"))
#: Удары китов паков, которые играет v2 (ADR-0153 S4, lo-fi: мягкие бочка/малый/хэт; щёток в свободных паках нет).
#: Отбор — по файлам на роботе 08.10 (``soundfile``, полный диапазон 44.1 кГц): бочки — доля < 200 Гц ≥ 0.97
#: (``bd_jazz`` 0.997, ``drum_bass_soft`` 0.986, ``KdrumL_19`` 0.978); малые — тихие, середина ≥ 0.8
#: (``drum_snare_soft`` 0.81, ``Snare_40/43`` 0.97); хэты — с долей ниже 8 кГц: у ``drum_cymbal_closed``/``hat_tap``
#: выше 8 кГц 0.75–0.78 (на 16 кГц робота почти не слышны), у ``HihatClosed_16/18`` 0.42–0.44, у ``cymbal_pedal`` 0.49.
#: Уровень — ``rms_db`` каталога (по всему файлу с хвостом); на 16 кГц робота не слушано.
PACK_DRUMS: Tuple[str, ...] = (
    "sonicpi_bd_jazz", "sonicpi_drum_bass_soft", "muldjord_kdruml_19",
    "sonicpi_drum_snare_soft", "muldjord_snare_40", "muldjord_snare_43",
    "muldjord_hihatclosed_16", "muldjord_hihatclosed_18", "sonicpi_drum_cymbal_pedal",
    # ADR-0153 S5 (рок-кит Muldjord). Файлы на роботе 08.10 (``soundfile``, 44.1 кГц, моно-сумма, доли энергии):
    # бочки ``KdrumL_20``/``KdrumR_20``/``KdrumR_21`` < 200 Гц 0.976/0.989/0.987; томы ``Tom1_08``…``Tom4_09`` < 200 Гц
    # 0.87/0.69/0.74/0.64 (середина 0.29–0.51); райд ``RideR_07`` 2–8 кГц 0.51, > 8 кГц 0.13 (на 16 кГц робота слышен,
    # в отличие от хэтов с > 8 кГц 0.42–0.44); крэши ``CrashL_08``/``CrashR_07`` 2–8 кГц 0.72, > 8 кГц 0.13–0.14.
    "muldjord_kdruml_20", "muldjord_kdrumr_20", "muldjord_kdrumr_21",
    "muldjord_tom1_08", "muldjord_tom2_08", "muldjord_tom3_08", "muldjord_tom4_09",
    "muldjord_rider_07", "muldjord_crashl_08", "muldjord_crashr_07",
    # ADR-0153 S6 (джаз): второй райд того же кита — райд звучит весь трек, один файл приедается.
    "muldjord_rider_09",
)


def _pack_drum(name: str) -> SampleInfo:
    data = _SONICPI_DATA if name.startswith("sonicpi_") else _MULDJORD_DATA
    meta = data["samples"][name]
    return SampleInfo(name, str(meta["group"]), str(meta["role"]), float(meta["seconds"]), int(meta["channels"]),
                      f"{data['pack_dir']}/{meta['path']}", float(meta["peak_db"]), float(meta["rms_db"]))


#: Каталог сэмплов v2: имя → :class:`SampleInfo` — DJ_Dave, брейки паков (роль ``break``) и удары китов паков
#: (:data:`PACK_DRUMS`). v1 (``core/sample_dave``) видит из него только пак DJ_Dave (``SAMPLE_PACK_DIR``).
SAMPLE_CATALOG: Mapping[str, SampleInfo] = {
    **{n: _sample_info(n, m) for n, m in _SAMPLE_DATA["samples"].items()},
    **{n: _pack_break(n, m, str(_SONICPI_DATA["pack_dir"])) for n, m in _SONICPI_DATA["samples"].items()
       if m["role"] == "break" and n in PACK_BREAK_BEATS},
    **{n: _pack_drum(n) for n in PACK_DRUMS},
}
#: Группа каталога → описание (подсказки модели в v1).
SAMPLE_GROUPS: Mapping[str, str] = dict(_SAMPLE_DATA["groups"])


@functools.lru_cache(maxsize=None)
def scale_pitch_classes(root: int, mode: str) -> frozenset:
    """Множество pitch class лада от тоники ``root`` (0..11). Кешируется: валидатор зовёт его на каждую ноту трека."""
    return frozenset((root + step) % 12 for step in SCALES[mode])


def traits_of(synth: str) -> "SynthTraits | None":
    return SYNTH_TRAITS.get(synth.strip().lower()) if synth else None


def role_ceiling(role: str) -> float:
    return LEVEL_CEILINGS[role]


__all__ = [
    "ACCENT_AMPLIFY", "BPM_RANGE", "CHROMATIC", "DAVE_PSR", "DECK_SLOTS", "DRUM_SYMBOLS", "ENERGY_LEVELS",
    "ENERGY_TRIM_DB", "DEFAULT_HOOKS", "ENERGY_THIN_ROLES", "ENERGY_WAVE", "SET_MAX_TRACKS", "SET_TRACKS",
    "SET_TRACK_SECONDS", "THEMES",
    "ThemeRow", "HOOK_KEY_FIT_MIN",
    "KICK_PATTERNS", "LEAD_MAX_MIDI", "LEVEL_CEILINGS", "MOOD_ENERGY", "PLAY_SYNTH", "ROLES",
    "PACK_BREAK_BEATS", "PACK_DRUMS", "APPROACH_MAX_BEATS", "ROOTS", "SAMPLE_CATALOG", "SAMPLE_EDGE_S", "SAMPLE_GROUPS", "SAMPLE_PACK_DIR", "SAMPLE_ROLES", "SCALES",
    "GENRE_EXTRA", "GENRE_FILLER", "GENRE_NOT", "GENRE_TAGS", "SCORE_GENRES", "SEARCH_STOPWORDS",
    "SEARCH_STYLE_PATTERNS",
    "STYLE_PATTERNS",
    "SYNTH_PALETTE", "SYNTH_TRAITS", "SampleInfo", "SynthTraits", "TONAL_ROLES", "role_ceiling",
    "scale_pitch_classes", "traits_of", "NYQUIST_MAX_MIDI",
    "LEAD_CLARITY", "LeadClarity", "THEME_ATTACK_MAX_MS", "THEME_BRIGHT_MIN", "THEME_LEAD_OK", "THEME_PURITY_MIN",
    "THEME_ROLES", "THEME_TAIL_MAX_MS", "theme_lead_ok",
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
                   "jbass": -32.9, "wobblebass": -35.7, "tb303": -36.2, "subbass": -42.7, "moogbass": -56.3,
                   # 07.10 (ADR-0153 S2, ``--sweep bass``, katana, образ 41596e9): 8-битный бас chiptune;
                   # ``square`` на потолке ``amp`` −46.2 дБ (цель роли −32) — в семьи не идёт, ``pulse`` −37.5
                   "square": -59.33, "pulse": -44.05}),
    "pad": (0.11, {"sinepad": -74.4, "warmpad": -43.1, "space": -54.5,
                   "marchstrings": -38.1, "mhpad": -38.9, "strangerpulsepad": -43.0, "ambi": -56.5, "strings": -59.5,
                   "pads": -64.0,
                   # 07.10 (ADR-0153 S2, ``loudness_nrt_v2.sh --sweep pad``, katana, образ voice-assistant 41596e9):
                   # синты арпеджио synthwave/chiptune в рамке пэда 29.09; наклон ``amp`` 1.0, у ``square`` 1.99
                   "pulse": -51.39, "square": -75.2, "blip": -53.8, "saw": -56.17}),
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
        "square": (0.573, 0.425, 0.002), "pulse": (0.58, 0.418, 0.002),
    },
    "pad": {
        "sinepad": (0.001, 0.999, 0.0), "warmpad": (0.085, 0.915, 0.0), "space": (0.0, 1.0, 0.0),
        "marchstrings": (0.107, 0.893, 0.0), "mhpad": (0.009, 0.991, 0.0), "strangerpulsepad": (0.058, 0.942, 0.0),
        "ambi": (0.052, 0.948, 0.0), "strings": (0.009, 0.986, 0.005), "pads": (0.0, 0.394, 0.606),
        "pulse": (0.032, 0.94, 0.028), "square": (0.033, 0.945, 0.022), "blip": (0.029, 0.97, 0.001),
        "saw": (0.024, 0.937, 0.039),
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
#: Удар файла пака в ударной роли (ADR-0153 S4) — в шкале той же модели: рисунок рамки роли (:data:`DRUM_LOUDNESS_KEY`:
#: ударов на такт, шаг между ними в долях) при темпе рамки :data:`LOUDNESS_FRAME_BPM`; dB при ``amp`` 1 =
#: ``rms_db`` файла + 10·lg(доля такта, которую звучат удары; удар — файл, но не дольше шага). МОДЕЛЬ, не замер.
#: ``toms`` (ADR-0153 S5) — рамка филла: 4 удара 16-ми (томы звучат только в такте филла на стыке секций; в рамке
#: модели роль звучит всю секцию — оценка сверху).
PACK_DRUM_FRAME: Mapping[str, Tuple[int, float]] = {"kick": (4, 1.0), "hats": (4, 1.0), "clap": (2, 2.0),
                                                    "toms": (4, 0.25)}
LOUDNESS_FRAME_BPM = 124.0
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
def _looks(drop_kick: str) -> Tuple[Tuple[int, Look], ...]:
    """Виды секций по энергии с рисунком бочки дропа ``drop_kick`` (ключ :data:`KICK_PATTERNS`): build и интро/аутро
    (блэнд двух дек — одна бочка на такт) от жанра окна не зависят."""
    return (
        (7, Look(KICK_PATTERNS[drop_kick], 1.0)),
        (5, Look("X.......X.......", 0.5)),
        (0, Look(KICK_PATTERNS["four_on_floor"], 0.6)),
    )


_CLUB_LOOKS: Tuple[Tuple[int, Look], ...] = _looks("four_on_floor")

#: LPF-свип на басе и нотах (DJ_Dave: «один слайдер на оба»; ADR-0149 §3.12): секция → (от, до) Гц, 0 — фильтр
#: снят. Build открывается к дропу, дроп открыт, брейк прикрыт, хвост уходящей деки закрывается 4000→300 за блэнд.
#: Потолок свипа ниже среза мастер-шины (``masterfilter`` LPF 0.55·Найквиста = 4400 Гц на 16 кГц): выше — не слышно,
#: а срез выше Найквиста у ``RLPF`` неустойчив. Шаг свипа — доля (``var``, не ``linvar``: скачок на границе секции
#: у ``linvar`` требует сегмента, и нота на нём получила бы промежуточный срез).
LPF_OPEN = 0.0
LPF_TOP_HZ = 4000.0
LPF_RANGE_HZ = (200.0, LPF_TOP_HZ)
_CLUB_SECTION_LPF: Mapping[str, Tuple[float, float]] = {
    "build": (400.0, LPF_TOP_HZ), "build2": (400.0, LPF_TOP_HZ), "break": (1200.0, 1200.0),
    "break2": (1200.0, 1200.0), "outro_tail": (LPF_TOP_HZ, 300.0),
}
#: Роли под свипом секции; в хвосте блэнда (``outro_tail``) — всё, что звучит, кроме бочки (§3.12).
_CLUB_LPF_ROLES: Tuple[str, ...] = ("bass", "pad", "lead")
_CLUB_LPF_TAIL_SECTIONS: Tuple[str, ...] = ("outro_tail",)

# ── Мастер-шина (``custom_synthdefs/masterfilter.scd``, node 999): ADR-0149 §3.10, ADR-0147 §3.2, §3.5; PR-7 ───────
#: Ручки мастер-шины, которые выставляет движок v2, и их значения по умолчанию — ровно дефолты SynthDef
#: (сверяет ``test_master.py``): каждый старт трека и стоп выставляют их заново, поэтому трек вне сета не наследует
#: ``trim`` и профиль выравнивателя от DJ-трека (правило сброса ADR-0147 §3.2).
#: ``lvlTarget``/``cmpRatio``/``makeup`` — профиль динамики стиля (``Style.master``, #3549: джазу компрессор 4:1 и
#: makeup 9 дБ срезали crest до 13 дБ против 17.8 у эталона).
MASTER_DEFAULTS: Mapping[str, float] = {"trim": 0.0, "lvlRatio": 3.0, "lvlUp": 3.0, "lvlTarget": -12.0,
                                        "cmpRatio": 4.0, "makeup": 9.0}
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
    "intro": (-4.0, False), "intro_low": (-4.0, False), "build": (-1.5, True), "build2": (-1.5, True), "drop": (-1.0, False),
    "break": (-5.0, False), "break2": (-5.0, False), "drop2": (0.0, False), "outro": (-3.0, False), "outro_tail": (-4.0, False),
}
#: Секция песенной формы стиля → секция клуба с тем же местом в треке (ADR-0153 S5, рок: куплет/припев/бридж). Одна
#: таблица: развитие хука (``arrange.hook.DEVELOPMENT``), тема целиком (:data:`THEME_SECTION`), дуга громкости
#: (:data:`SECTION_TRIM_DB`), A9 по дропам и fill перед дропом читают ВИД секции (:func:`section_kind`), а имя секции —
#: только то, что задаёт сам стиль (``layer_sections``, ``section_lpf``, ``Style.backbeat_kinds`` — по виду).
#: Куплет — подъём к припеву (``build``: начало хука), припев — дроп (хук, тема целиком в первом), бридж — брейк.
SECTION_KINDS: Mapping[str, str] = {"verse": "build", "verse2": "build2", "chorus": "drop", "chorus2": "drop2",
                                    "chorus3": "drop2", "bridge": "break",
                                    # ADR-0153 S6, джаз: голова — дроп (тема целиком), вторая голова — хук в терциях
                                    # (две «дудки» на out-head), соло — свой вид (:data:`SECTION_HARMONY` ``CHANGES``)
                                    "head": "drop", "head2": "drop2", "solo2": "solo"}


def section_kind(name: str) -> str:
    """Вид секции ``name`` (:data:`SECTION_KINDS`); секция клуба — сама себе вид."""
    return SECTION_KINDS.get(name, name)


#: Тембры по теме (ADR-0149 §4.7 ``timbre_family``): роль → синты семьи; выбор внутри — по сиду трека со штрафом за
#: недавнее (пэд, ADR-0152 §3.2). Только синты с замером громкости и полос (:data:`LANE_DB_AT_UNIT`,
#: :data:`LAYER_BANDS`) из палитры роли. Пэд с собственным хвостом (``warmpad`` 1.2 с, ``strangerpulsepad`` 1.6 с,
#: :data:`SYNTH_TRAITS`) звучит только рисунком, который это допускает (``PadFigure.long_tails`` — ``held``): под
#: сайдчейном 16-х хвост размазал бы огибающую. ``space`` вернулся (В9 ADR-0152): громче модели на 4–6 дБ он был в
#: рисунке 16-х — тот же сдвиг, что у ``sinepad`` (+6.3, #3430); долю низа дропа держит A9-модель трека
#: (``arrange.mix.a9_trim``), а не исключение из палитры. ≥ 2 пэда на семью, у каждого рисунка ≥ 1.
#: ``mhpad``/``marchstrings`` замерены (#3430), но не в досылке ``CRITICAL_SYNTHS`` — в пул не идут.
#: ADR-0152 PR-6 (§3.3): 4 лида и 2–3 баса на семью. Лид — читаемый темой (:data:`THEME_LEAD_OK`, 07.10: ``hoover``,
#: ``epiano``, ``kalimba``, ``rhpiano`` сняты — хорус/эхо/тусклый), с долей низа < 0.05, достающий цель роли на потолке
#: ``amp`` (самый тихий — ``keys``: −49.2 при цели −50). Бас — доля низа ≥ 0.9 (#3430): ``retrobass`` (0.59) и ``tb303``
#: (0.52) держат A9-модель трека на 0.72/0.67 при любом пэде (``a9_trim`` упирается в потолок ``amp``) — в семьи не
#: идут; ``tb303`` (PR-9) — только в ``hard`` и только рисунком ``acid16`` (:data:`BASS_FIGURE_SYNTHS`); ``moogbass`` — разброс замера по тоникам 8 дБ.
#: Доля низа баса — на роботе (:func:`bass_low_on_robot`, порог :data:`BASS_MIN_LOW`): ``dub`` на образе 06.10 звучит
#: серединой (#3457) — из семей снят, в ``warm`` его место занял ``jbass`` (три баса на семью темы вне ``THEMES``).
_CLUB_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("blip", "pluck", "keys", "saw"), "bass": ("subbass", "jbass"),
             "pad": ("sinepad", "space", "strangerpulsepad")},
    "hard": {"lead": ("arpy", "blip", "brass", "pluck"), "bass": ("jbass", "wobblebass", "tb303"),
             "pad": ("sinepad", "strings", "strangerpulsepad")},
    "bright": {"lead": ("pluck", "blip", "orient", "keys"), "bass": ("bass", "jbass"),
               "pad": ("strings", "ambi", "sinepad")},
    "warm": {"lead": ("pluck", "arpy", "keys", "brass"), "bass": ("bass", "jbass", "subbass"),
             "pad": ("sinepad", "ambi", "warmpad")},
}
#: Строка ``THEMES`` → семья тембров стиля; тема не из таблицы — ``Style.default_timbre``.
THEME_TIMBRE: Mapping[str, str] = {"space": "dark", "cyber": "hard", "kids": "bright", "slavic": "warm",
                                   "winter": "bright"}


def family_of(style: "Style", row: Optional[str]) -> str:
    """Семья тембров темы из таблицы (:data:`THEME_TIMBRE`); строки нет — ``Style.default_timbre``. Темы вне таблицы
    получают семью по сиду со штрафом за прошлые сеты (``set_plan.pick_timbre``, #3460) — это лишь запасное значение."""
    return THEME_TIMBRE.get(row or "", style.default_timbre)


@dataclass(frozen=True)
class PadFigure:
    """Рисунок пэда (ADR-0152 §3.2): генератор — ``arrange.compose.PAD_GENERATORS[ключ]``.

    ``ducked`` — пэд под сайдчейном (``Style.duck_roles``); ``long_tails`` — допускает синты с собственным хвостом;
    ``level_offset_db`` — прибавка к цели роли, при которой пэд звучит так же громко, как ``pumped16`` на цели:
    модель громкости 29.09 снята на пэде, держащем аккорд 8 долей, а ``pumped16`` на роботе громче модели на 6.3 дБ
    (харнесс ``loudness_nrt_v2.py --track 7``, #3430). Той же прибавкой A9-модель (``arrange.mix``) возвращает
    рисунок в шкалу ``pumped16``, где пэд на роботе громче модели на :data:`PAD_ROBOT_DB`.
    """

    ducked: bool
    long_tails: bool
    level_offset_db: float


#: Рисунки пэда: ``pumped16`` — аккорд на каждой 16-й под сайдчейном (как до PR-5); ``held`` — аккорд держится до
#: смены (2 такта), без насоса — качают бас и psr; ``stabs`` — аккорд на «и» каждой доли до следующего «и» под
#: сайдчейном. Прибавки — по записи робота 06.10 (серии ser6/ser7/ser9, #3449: 133 дропа, совместная подгонка
#: низа дропа моделью :func:`arrange.mix.low_share` с :data:`PAD_ROBOT_DB` при сыгранных уровнях). ``held`` теперь
#: отделён от синта на 11 дропах ``ambi``/``sinepad`` (а не на 3): +3.1 (бутстрап по трекам 1.3…5.0), ``stabs`` +1.2
#: (0.1…1.8; прежние +2.1 вне интервала).
PAD_FIGURES: Mapping[str, PadFigure] = {
    "pumped16": PadFigure(ducked=True, long_tails=False, level_offset_db=0.0),
    "held": PadFigure(ducked=False, long_tails=True, level_offset_db=3.1),
    "stabs": PadFigure(ducked=True, long_tails=False, level_offset_db=1.2),
    # ADR-0153 S2: арпеджио (``arrange.pad.arp``) — один тон аккорда на каждой 16-й, без насоса (synthwave/chiptune).
    # Прибавка — ОЦЕНКА, не замер: звучит один голос из трёх аккорда модели громкости (10·lg 3 = +4.8 дБ). На роботе
    # не мерено; A9-модель трека считает пэд с худшим сдвигом (:data:`PAD_ROBOT_DB_UNMEASURED`), пока нет записи.
    "arp": PadFigure(ducked=False, long_tails=False, level_offset_db=4.8),
    # ADR-0153 S4: comping (``arrange.pad.comping``) — 2–3 коротких аккорда на такт мимо сильных долей, без насоса.
    # Прибавка — ОЦЕНКА, не замер: аккорд звучит ≈ 0.3 такта (ритмы ``Style.comp_rhythms``: 4–7 шестнадцатых из 16),
    # 10·lg(16/5) ≈ +5 дБ до громкости ``pumped16``, звучащего весь такт. На роботе не мерено.
    "comping": PadFigure(ducked=False, long_tails=False, level_offset_db=5.0),
    # ADR-0153 S5: пауэр-аккорды рока (``arrange.pad.power_chords``) — прима, квинта, октава восьмыми/по ритмам
    # ``Style.power_rhythms``, без насоса. Прибавка — ОЦЕНКА, не замер: звучат 2 голоса из 3 (−1.8 дБ к трезвучию
    # рамки) примерно половину такта (палм-мьют 16-ми на восьмых): 10·lg(16/8) − 1.8 ≈ +1.2 дБ. На роботе не мерено.
    "power_chords": PadFigure(ducked=False, long_tails=False, level_offset_db=1.2),
}
#: На сколько дБ пэд рисунка ``pumped16`` на роботе громче своего ``level_db`` (модель громкости — NRT аккорда на
#: 8 долей, :data:`LAYER_MEASURED_DB`): подгонка низа 133 дропов серий ser6/ser7/ser9 06.10 (#3449; было — 46
#: дропов ser6, #3441). Модель − запись: среднее +0.001, σ 0.095 доли (по сериям +0.015/−0.019/+0.007, по окнам
#: club −0.004, breaks −0.014, deep +0.051). Бутстрап по трекам: ``ambi`` 13.1…14.7 (прежние 15.6 вне интервала),
#: ``sinepad`` 7.1…9.3, ``warmpad`` 9.0…12.8. Остаток σ ≈ 0.09 — не пэд: бас-синт (``subbass`` на роботе богаче
#: низом модели, ``dub`` беднее) и окно deep (``sinepad``/``pumped16`` при 121 bpm — модель выше записи на 0.15,
#: 3 трека); в таблицу не внесены — мало треков, решение не меняют при ``a9_model_low`` 0.6.
#: Сдвиг у синтов разный (``ambi`` на 6 дБ громче ``sinepad``), причина не найдена — замер, не модель. Калибровка
#: «робот = 0.62 модели» (#3401) снята на одном ``sinepad`` и на ``ambi``/``warmpad`` не переносится: при ней
#: A9-модель давала 0.81–0.87 всем трекам ser6, а запись — 0.75 ``sinepad`` и 0.33–0.49 ``ambi``/``warmpad``.
#: Пэд палитры без замера на роботе (``space``, ``strangerpulsepad``, ``strings``) берёт худший замеренный сдвиг
#: (:data:`PAD_ROBOT_DB_UNMEASURED`): A9-модель скорее приглушит пэд, чем пропустит дроп без низа.
PAD_ROBOT_DB: Mapping[str, float] = {"sinepad": 8.3, "warmpad": 11.1, "ambi": 14.2}
PAD_ROBOT_DB_UNMEASURED = max(PAD_ROBOT_DB.values())
#: Бас на роботе против NRT (#3457, приёмка 06.10, образ 2e0c03537): сдвиг дБ и доля низа < 150 Гц там, где запись
#: расходится с :data:`LAYER_MEASURED_DB`/:data:`LAYER_BANDS`; остальные басы — как NRT. ``dub`` (SynthDef: ``freq/4``,
#: энергия в 12–32 Гц) в 10 из 12 дропов приёмки дал суб 12–32 Гц −45…−53 дБ против −26…−40 у тех же нот в ser6/7/9
#: (у ``bass`` с тем же ``freq/4`` суб не изменился), текст программы ``dub`` в образах PR-8 и PR-10 одинаков — причина
#: на стороне робота, не найдена. По 22 дропам приёмки ``dub`` ведёт себя как слой середины: лучшая подгонка — доля
#: низа 0.0 при +3 дБ (остаток модели по ``dub`` +0.010±0.121; «бас молчит» даёт +0.265 — отвергнуто). ``jbass`` −6.5
#: (10 дропов приёмки, бутстрап по трекам −15…−1.8; было +0.084 модели над записью). ``subbass`` (ser +2.8, приёмка −3.1),
#: ``tb303`` (4 дропа), ``wobblebass`` (2) — без поправки: знак не держится или данных мало.
BASS_ROBOT_DB: Mapping[str, float] = {"dub": 3.0, "jbass": -6.5}
BASS_ROBOT_LOW: Mapping[str, float] = {"dub": 0.0}
#: Бас семьи тембров держит низ дропа: доля низа на роботе (:func:`bass_low_on_robot`) не ниже этой (критерий палитры
#: ADR-0152 PR-6, #3430); ``tb303`` (0.52) — исключение рисунка ``acid16`` (:data:`BASS_FIGURE_SYNTHS`).
BASS_MIN_LOW = 0.9


def bass_low_on_robot(synth: str) -> float:
    """Доля низа < 150 Гц баса ``synth`` на роботе: замер записи (:data:`BASS_ROBOT_LOW`), иначе NRT (``LAYER_BANDS``)."""
    return BASS_ROBOT_LOW.get(synth, LAYER_BANDS["bass"][synth][0])


#: A9-модель трека (ADR-0152 §4 п.2): доля низа каждого дропа в шкале робота (пэд — с :data:`PAD_ROBOT_DB`) —
#: ниже порога ``Style.a9_model_low`` пэд тише ступенями ``A9_STEP_DB`` до ``A9_PAD_FLOOR_DB`` от своей цели, затем
#: бас громче до ``A9_BASS_BOOST_DB`` (бас под сайдчейном на роботе тише модели на 6.7 дБ, #3430). Не дотянули —
#: трек играет, ``Mix.a9_model`` честно ниже порога. Ступень, поднимающая долю меньше ``A9_MIN_GAIN``, не
#: применяется: у ``retrobass`` (низ 0.59, на потолке ``amp`` −35.3 при цели −32) доля 0.73 не растёт ни от пэда −12,
#: ни от баса — глушить пэд зря нельзя; палитра басов (ADR-0152 PR-6) его в семьи не берёт.
A9_MIN_GAIN = 0.005
A9_STEP_DB = 2.0
A9_PAD_FLOOR_DB = -12.0
A9_BASS_BOOST_DB = 4.0


@dataclass(frozen=True)
class BassFigure:
    """Рисунок баса (ADR-0152 §3.3): генератор — ``arrange.compose.BASS_GENERATORS[ключ]``.

    ``steps`` — шаги 16-х такта; ни один не на доле (там прямая бочка, ADR-0149 §3.4), нота кончается к следующему
    шагу рисунка или к доле. ``fifth_last`` — последняя нота такта — квинта аккорда, остальные — тоника.
    """

    steps: Tuple[int, ...]
    fifth_last: bool
    #: Акцент каждой ноты такта (``PitchEvent.accent``); пусто — первая нота 3, остальные 2.
    accents: Tuple[int, ...] = ()
    #: Номера нот такта, которые звучат октавой выше (если вышли бы в регистр баса; иначе — тоникой).
    lift: Tuple[int, ...] = ()
    #: Срез фильтра каждой ноты, Гц, на цикл ``2 × len(steps)`` нот (чётный такт, нечётный); пусто — без среза на ноту
    #: (общий свип секции ``Style.section_lpf``). Рисунок со своим срезом свип секции на басе заменяет.
    lpf: Tuple[float, ...] = ()


def _acid_lpf(accents: Tuple[int, ...], low: float, high: float, accent_gain: float) -> Tuple[float, ...]:
    """Срез ``acid16``: треугольная волна по нотам двух тактов от ``low`` до ``high`` Гц, нота с акцентом 3 —
    на ``accent_gain`` шире; срез открывается к середине цикла и закрывается к его концу."""
    n = 2 * len(accents)
    out = []
    for i in range(n):
        wave = 1.0 - abs(2.0 * i / (n - 1) - 1.0)
        hz = (low + (high - low) * wave) * (accent_gain if accents[i % len(accents)] == 3 else 1.0)
        out.append(float(round(min(hz, LPF_TOP_HZ) / 10.0) * 10))
    return tuple(out)


_ACID_STEPS = (1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15)  # 16-е мимо долей: на долях бочка (ADR-0149 §3.4)
_ACID_ACCENTS = (3, 2, 2) * 4
#: Рисунки баса: ``offbeat`` — «и» каждой доли, полдоли, тоника ×3 + квинта (как до PR-6); ``rolling8`` — восемь нот
#: такта тоникой без октавы, «и» и «а» каждой доли по 16-й: сама восьмая доли занята бочкой, поэтому ролл сдвинут
#: на «и». Звучащее время у обоих — полдоли на долю: модель громкости роли (бас звучит всю секцию) у них одна.
BASS_FIGURES: Mapping[str, BassFigure] = {
    "offbeat": BassFigure(steps=(2, 6, 10, 14), fifth_last=True),
    "rolling8": BassFigure(steps=(2, 3, 6, 7, 10, 11, 14, 15), fifth_last=False),
    # PR-8: бас ломаной бочки (``breakbeat``: шаги 0, 2, 10, 13) — ни одна нота не на шаге этой бочки и не на доле.
    "broken": BassFigure(steps=(1, 6, 9, 14), fifth_last=True),
    # PR-9: кислотная 16-я линия ``tb303`` — тоника на каждой 16-й мимо долей, акцент на первой 16-й каждой доли,
    # последняя 16-я доли — октавой выше, срез на каждую ноту волной в два такта (``lpf=[...]``).
    "acid16": BassFigure(steps=_ACID_STEPS, fifth_last=False, accents=_ACID_ACCENTS, lift=(2, 5, 8, 11),
                         lpf=_acid_lpf(_ACID_ACCENTS, 500.0, 2400.0, 1.5)),
    # ADR-0153 S2: 8-битный бас ``pulse`` — шаги ``rolling8``, каждая вторая нота октавой выше (тоника–октава).
    "octave8": BassFigure(steps=(2, 3, 6, 7, 10, 11, 14, 15), fifth_last=False, lift=(1, 3, 5, 7)),
}
#: Рисунок баса ↔ синт (ADR-0152 PR-9, одна таблица): рисунок из таблицы звучит только перечисленными синтами, а
#: синт из таблицы — только рисунками, которые его называют (``arrange.mix.bass_pair_ok``). ``tb303`` держит низ на
#: 0.52 (:data:`LAYER_BANDS`) — соло-линия ``acid16`` с огибающей на ноту, а не ровный бас ``offbeat``/``rolling8``.
#: ``pulse`` (ADR-0153 S2, chiptune) — только октавами ``octave8``: низ 0.58, на потолке ``amp`` на 5.5 дБ тише цели
#: роли — звук жанра поверх бочки, а не опора дропа (``square`` ещё на 8.7 дБ тише — не взят).
BASS_FIGURE_SYNTHS: Mapping[str, Tuple[str, ...]] = {"acid16": ("tb303",), "octave8": ("pulse",)}
#: Басовые линии (ADR-0153 S4) — генераторы без рисунка шагов :data:`BASS_FIGURES`: ``walking`` — четверти на долях
#: (тон аккорда, тоны аккорда, подход к следующему такту на 4-й доле), бочка стиля их не исключает.
#: ``riff`` (ADR-0153 S5, рок) — восьмые по риффу стиля (``Style.bass_riffs``) на тонах аккорда такта; с бочкой рока
#: совпадает по замыслу (бас и бочка рока играют вместе).
BASS_LINES: Tuple[str, ...] = ("walking", "riff")


@dataclass(frozen=True)
class KickSound:
    """Сэмпл бочки ``play(symbol, sample=N)`` и его замер на роботе (запись ``jack_rec``, бочка одна, 4/4 @130)."""

    symbol: str
    sample: int
    file: str  # имя в ``0_foxdot_default/x/upper`` (сортировка Renardo ``_getFileInDir``)
    loudness_offset_db: float  # энергия удара против ``X:0`` по файлу — поправка к модели ``four_on_floor``
    low: float  # доля энергии записи < 250 Гц
    sub: float  # доля энергии записи < 120 Гц
    #: ``False`` — на роботе не мерено: ``loudness_offset_db`` — оценка, ``low`` — доля < 200 Гц ФАЙЛА (опись
    #: сэмплов 05.10, ``scan.json``), ``sub`` неизвестна (``nan``). Замер — ``scripts/music/kicks_probe.sh``.
    measured: bool = True
    #: Бочка — файл пака (ключ :data:`SAMPLE_CATALOG`, ADR-0153 S4): играет ``loop(файл)``, а не ``play(символ)``;
    #: ``symbol``/``sample`` не используются, громкость — по каталогу (``arrange.mix``), ``loudness_offset_db`` 0.
    pack: str = ""


#: Бочки, замеренные 02.10.2026 (Vision Pi, ``jack_rec``). ``X`` без ``sample=`` на роботе звучит щелчком:
#: < 250 Гц 0.3 %, > 2 кГц 98 % записи, хотя файл ``000_Kick_HSS2_NoN.wav`` на 99 % ниже 250 Гц (причина не
#: найдена, находка #3312). ``X:12`` — 94.5 % записи ниже 250 Гц, 5.5 % середины (атака), без гула 808.
#: PR-4 (ADR-0152 §3.4, #3435): ещё три из ``x/upper`` — замер 06.10 ``scripts/music/kicks_probe.sh`` (Vision Pi,
#: ``jack_rec`` 8 с, одна бочка 4/4 @130; ``X:12`` воспроизвёл 02.10: 0.941/0.853 против 0.945/0.862). Отбор по
#: описи сэмплов (длина 0.15–0.6 с, низ файла ≥ 0.85, центроид < 100 Гц) и по замеру: доли < 250/< 120 Гц ≥ 0.8/0.6,
#: > 2 кГц ≈ 0 (щелчка нет), пик в лимитере −7.0 дБ у всех (``deep`` −7.8). ``loudness_offset_db`` = 0.98 + RMS записи
#: кандидата − RMS ``X:12`` (−23.5 дБ): ``deep`` +0.4, ``techno`` −0.5, ``garage`` −1.9. Лицензии пака
#: ``0_foxdot_default`` Renardo в основном CC0 — файл за файлом не проверялись.
KICK_SOUNDS: Mapping[str, KickSound] = {
    "house": KickSound("X", 12, "012_Kick_House_GhostFader.wav", 0.98, 0.945, 0.862),
    "deep": KickSound("X", 3, "003_Kick_AK_GhostFader.wav", 1.38, 0.999, 0.938),
    "techno": KickSound("X", 30, "030_Kick_KHS_TechnoV1.wav", 0.48, 0.985, 0.803),
    "garage": KickSound("X", 18, "018_Kick_Garage_GhostFader.wav", -0.92, 0.943, 0.806),
    # ADR-0153 S1, стиль ``rave``: жёсткие бочки из ``a/upper`` (символ ``A``) и ``w/upper`` (``W``) пака
    # ``0_foxdot_default``. Отбор по описи 05.10 (``scan.json``, файл, не запись): длина 0.4–0.5 с, доля < 200 Гц
    # ≥ 0.8, > 4 кГц ≈ 0, центроид < 120 Гц; номер файла — индекс в отсортированной папке (как у ``X``), разный у
    # всех бочек таблицы. НА РОБОТЕ НЕ МЕРЕНЫ (``measured=False``): поправка громкости — оценка «как ``house``»
    # (+0.98), до ``kicks_probe.sh`` (``SYM=A``/``SYM=W``). Перегруз в файле — пик/RMS могут быть выше оценки.
    "hardtechno": KickSound("A", 17, "017_Kick_DistHardcord_Jochnhardtechno.wav", 0.98, 0.989, math.nan, False),
    "hardstyle": KickSound("A", 14, "014_Kick_DistHardstyle_Harddancewarrior.wav", 0.98, 0.962, math.nan, False),
    "hardcore": KickSound("A", 15, "015_Kick_DistHardcore_NoN.wav", 0.98, 0.819, math.nan, False),
    "acid": KickSound("W", 1, "001_Kick_KickAcid_Blackie666.wav", 0.98, 0.996, math.nan, False),
    # ADR-0153 S4, стиль ``lofi``: мягкие бочки паков (:data:`PACK_DRUMS`) — ``low`` = доля < 200 Гц файла (08.10).
    # НА РОБОТЕ НЕ МЕРЕНЫ: уровень — каталог (``rms_db``) в рамке модели громкости (``arrange.mix``).
    "jazz": KickSound("", 0, "bd_jazz.flac", 0.0, 0.997, math.nan, False, pack="sonicpi_bd_jazz"),
    "soft": KickSound("", 0, "drum_bass_soft.flac", 0.0, 0.986, math.nan, False, pack="sonicpi_drum_bass_soft"),
    "acoustic": KickSound("", 0, "19-KdrumL-KdrumL.flac", 0.0, 0.978, math.nan, False, pack="muldjord_kdruml_19"),
    # ADR-0153 S5, стиль ``rock``: бочки рок-кита Muldjord, ``low`` = доля < 200 Гц файла (робот 08.10). НЕ МЕРЕНЫ.
    "rock_l": KickSound("", 0, "20-KdrumL-KdrumL.flac", 0.0, 0.976, math.nan, False, pack="muldjord_kdruml_20"),
    "rock_r": KickSound("", 0, "20-KdrumR-KdrumR.flac", 0.0, 0.989, math.nan, False, pack="muldjord_kdrumr_20"),
    "rock_r2": KickSound("", 0, "21-KdrumR-KdrumR.flac", 0.0, 0.987, math.nan, False, pack="muldjord_kdrumr_21"),
}


def kick_of(symbol: str, sample: int, pack: str = "") -> Optional[str]:
    """Имя бочки :data:`KICK_SOUNDS` по символу ``play()`` и номеру файла, а бочки-файла пака — по ``pack`` (ключ
    каталога); нет такой — ``None``."""
    if pack:
        return next((name for name, k in KICK_SOUNDS.items() if k.pack == pack), None)
    return next((name for name, k in KICK_SOUNDS.items()
                 if not k.pack and (k.symbol, k.sample) == (symbol, sample)), None)

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
#: Ширина синта поверх ширины роли стиля (#3458): моно-середина ``hard`` держала корреляцию L/R дропов приёмки 06.10
#: на 0.74–0.93 (8 дропов ``киберпанк``): корреляция середины 150–2000 Гц 0.62–0.80 против 0.15 у прочих (там середину
#: держат два голоса пэда и psr), а LR дропа ≈ доля низа × 1 + доля середины × LR середины. Лиды ``blip``/``cs80lead``/
#: ``hoover`` — два голоса как пэд. ``tb303`` (половина энергии в середине) — два голоса на полных L/R с расстройкой БЕЗ
#: Хааса: за 16-ю (≈ 0.11 с) расстройка 1/8 тона сдвигает фазу на 80 Гц на ≈ 0.04 оборота (низ в обоих каналах один —
#: центр, как I11, без гребёнки задержки), а на 1 кГц — на ≈ 0.8 оборота (середина расходится). Остальные басы и бочка
#: — в центре (``model.validate``).
SYNTH_STEREO: Mapping[str, Mapping[str, float]] = {
    **{lead: {"pan": PAD_SPREAD, "detune": PAD_DETUNE, "haas_ms": PAD_HAAS_MS} for lead in ("blip", "cs80lead", "hoover")},
    "tb303": {"pan": PAD_SPREAD, "detune": PAD_DETUNE},
}

__all__ += [
    "AMP_EXPONENT", "DJ_LEVELER", "DRUM_LOUDNESS_KEY", "KICK_SOUNDS", "kick_of",
    "HAAS_MAX_MS", "KickSound", "LANE_DB_AT_UNIT", "LAYER_MEASURED_DB", "LOUDNESS_SOURCE", "LPF_OPEN",
    "LPF_RANGE_HZ", "LPF_TOP_HZ", "Look", "MASTER_DEFAULTS", "MAX_LAYER_AMP", "SET_LEVELER", "TRIM_LAG_S",
    "PAD_DETUNE", "PAD_HAAS_MS", "PAD_PAN_BEATS", "PAD_SPREAD", "PAN_HATS", "PAN_PAD_WIDTH", "SYNTH_STEREO",
    "SECTION_TRIM_DB", "SIDECHAIN_SHAPE", "THEME_TIMBRE", "family_of", "PAD_FIGURES", "PadFigure", "PAD_ROBOT_DB",
    "PAD_ROBOT_DB_UNMEASURED", "BASS_ROBOT_DB", "BASS_ROBOT_LOW", "BASS_MIN_LOW", "bass_low_on_robot",
    "A9_MIN_GAIN", "A9_STEP_DB",
    "A9_PAD_FLOOR_DB", "A9_BASS_BOOST_DB", "PAN_PSR", "BASS_FIGURES", "BASS_FIGURE_SYNTHS", "BassFigure", "FormSpec",
    "BASS_LINES", "PACK_DRUM_FRAME", "LOUDNESS_FRAME_BPM",
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
FormSpec = Tuple[Tuple[str, int, int, frozenset], ...]
#: Хвост всех форм клуба (блэнд ``Style.blend``): outro с ударными и басом, затем хэты и пэд — входящий трек
#: начинает хэтами и пэдом под ним. В хвосте нет лида; ``outro`` длиннее 4 тактов (``long64``) блэнда не касается.
_CLUB_TAIL: FormSpec = (
    ("outro", 8 - (_CLUB_BLEND[0] - _CLUB_BLEND[1]), 2, _CLUB_DRUMS | {"bass", "pad"}),
    ("outro_tail", _CLUB_BLEND[0] - _CLUB_BLEND[1], 1, frozenset({"hats", "pad"})),
)
_CLUB_INTRO: FormSpec = (
    ("intro", _CLUB_BLEND[1], 2, frozenset({"hats", "pad"})),
    ("intro_low", _CLUB_BLEND[0] - _CLUB_BLEND[1], 2, _CLUB_DRUMS | {"bass", "pad"}),
)
# Состав растёт по форме: build без баса — низ возвращается ударом в дроп; брейк-луп (``Style.layer_sections``)
# вступает только во втором дропе — он плотнее первого. До этой правки build и оба дропа играли одним составом
# и на записи не различались (робот 05.10: build −23.9 дБ, дроп −24.5, drop2 −26.0; слух Шифу и отзыв
# эксперта: «нет восхождения, кульминации»).
_CLUB_BUILD: FormSpec = (("build", 8, 5, _CLUB_FULL - {"bass"}),)
_CLUB_DROP: FormSpec = (("drop", 8, 8, _CLUB_FULL),)
_CLUB_BREAK: FormSpec = (("break", 8, 4, frozenset({"hats", "pad", "lead"})),)
_CLUB_DROP2: FormSpec = (("drop2", 8, 9, _CLUB_FULL),)
#: Формы клубного трека (ADR-0152 §3.5, PR-7): имя → секции ``(имя, такты, энергия 0..10, роли)``. Имена секций —
#: ключи развития хука ``arrange.hook.DEVELOPMENT``, дуги ``SECTION_TRIM_DB``, LPF ``section_lpf`` и слоёв
#: ``layer_sections``. Блэнд — свойство пары форм: у каждой интро 4+4 и хвост outro/outro_tail 4+4 (``model.blend_bars``,
#: проверено на всех парах ``test_forms``). Энергия секции — ось вида (``Style.looks``): интро/аутро (блэнд) — ровная
#: прямая бочка, build 4..7, дроп ≥ 7 — полный «насос».
#:  * ``club48`` — база: build, drop, break, drop2 (48 тактов);
#:  * ``short32`` — «проходной» трек без брейка и второго дропа (32);
#:  * ``long64`` — пик сета: два подъёма и длинный брейк под ``held`` (64, outro 8);
#:  * ``dropfirst48`` — дроп сразу после интро (такт 8, ~14 с при 138 BPM, а не такт 16), build — перед вторым
#:    дропом (#3427): тему человек ждёт сразу. Это форма первого трека сета (``Style.opening_form``).
_CLUB_FORMS: Mapping[str, FormSpec] = {
    "club48": (*_CLUB_INTRO, *_CLUB_BUILD, *_CLUB_DROP, *_CLUB_BREAK, *_CLUB_DROP2, *_CLUB_TAIL),
    "short32": (*_CLUB_INTRO, *_CLUB_BUILD, *_CLUB_DROP, *_CLUB_TAIL),
    "long64": (*_CLUB_INTRO, *_CLUB_BUILD, *_CLUB_DROP, *_CLUB_BREAK,
               ("build2", 8, 6, _CLUB_FULL - {"bass"}), *_CLUB_DROP2,
               ("break2", 4, 4, frozenset({"hats", "pad", "lead"})),
               ("outro", 8, 2, _CLUB_DRUMS | {"bass", "pad"}), _CLUB_TAIL[1]),
    "dropfirst48": (*_CLUB_INTRO, *_CLUB_DROP, *_CLUB_BREAK, *_CLUB_BUILD, *_CLUB_DROP2, *_CLUB_TAIL),
}
_CLUB_OPENING_FORM = "dropfirst48"
#: Тема целиком (ADR-0154 PR-7; запрос Шифу 07.10 «хотел бы услышать всю тему горного короля»): в этой секции формы
#: хук уступает теме — мелодии фраза за фразой (тематическая секция материала партитуры или вся RTTTL-мелодия), а не
#: 4–8 тактам хука; секция растягивается на длину темы, остаток секции — хук. Гармония и бас — на всю длину темы.
THEME_SECTION = "drop"
#: Потолок темы в тактах клуба: тема длиннее режется по концу фразы.
THEME_MAX_BARS = 32
#: Главный мотив материала по RTTTL-эталону того же произведения (отзыв Шифу 07.10 «это не совсем горный король»):
#: контур первых ``THEME_REF_NOTES`` нот эталона ищется в голосах материала (``hook.for_theme``); совпало не меньше
#: ``THEME_REF_MATCH_MIN`` интервалов — хук и тема оттуда, меньше — материал не узнаётся, трек на RTTTL-хуке. Григ
#: (pdmx QmT5K8df…, QmRTWPHs…, Qme7zppV…) — 1.00, QmYe6duX… — 0.91; QmXXdTmX… — 0.64 (тема во внутреннем голосе,
#: его в материале нет).
THEME_REF_NOTES = 12
THEME_REF_MATCH_MIN = 0.8
#: Длина формы клуба — кратна периоду рисунков ударных (16 тактов) и не длиннее потолка: тема растит трек
#: 48 → 64 → 80 тактов, а не бесконечно (``model.BARS_TOTAL``).
FORM_BARS_STEP = 16
TRACK_MAX_BARS = 80
#: Энергия трека 1..5 (``ENERGY_WAVE``) → формы, из которых выбирает план (со штрафом за недавние): проходные —
#: на спаде и в начале, ``long64`` — только у пика; ``dropfirst48`` — со середины волны.
_CLUB_ENERGY_FORMS: Mapping[int, Tuple[str, ...]] = {
    1: ("short32", "club48"), 2: ("short32", "club48"), 3: ("short32", "club48", "dropfirst48"),
    4: ("short32", "club48", "dropfirst48", "long64"), 5: ("long64", "club48", "dropfirst48"),
}
#: Секции слоёв DJ_Dave (``arrange.samples``, PR-3d): psr-слой — build, дропы и break; брейк-луп — только второй
#: дроп (им он плотнее первого); FX — первая доля дропов (в блэнде PR-8 intro/outro звучат на двух деках).
_CLUB_LAYER_SECTIONS: Mapping[str, Tuple[str, ...]] = {"sample": ("build", "build2", "drop", "break", "break2", "drop2"),
                                                       "loop": ("drop2",), "fx": ("drop", "drop2")}
#: Прогрессии club по ступеням лада, аккорд на 2 такта (8-тактовая петля).
#: Приёмка 02.10 (A13): при четырёх прогрессиях одна занимала 5 треков из 10 — пул расширен до восьми.
_CLUB_PROGRESSIONS: Tuple[Tuple[int, ...], ...] = (
    (0, 5, 2, 6), (0, 3, 5, 4), (0, 5, 3, 4), (0, 6, 5, 6),
    (0, 2, 6, 5), (0, 3, 6, 2), (0, 4, 5, 3), (0, 6, 3, 5),
)


@dataclass(frozen=True)
class GenreWindow:
    """Жанровое окно клуба (ADR-0152 §3.5, PR-8): данные, которыми сет подменяет поля :class:`Style`
    (:func:`genre_style`).

    Окно выбирает план на СЕТ (``SetPlan.genre``, один темп и один грув на сет, ADR-0149 I7), внутри сета оно не
    меняется. Это ось «жанр клуба» и не стиль (ADR-0153: ``Style`` — набор таблиц; окна — подвыбор внутри стиля).
    ``pad_figures`` — пул с повторами: повтор = вес выбора (``weighted_pick`` берёт каждую запись пула отдельно).
    """

    bpm: Tuple[int, int]
    kick_pool: Tuple[str, ...]
    looks: Tuple[Tuple[int, Look], ...]
    bass_figures: Tuple[str, ...]  # ни один шаг рисунка баса не совпадает с шагом рисунка бочки дропа окна
    pad_figures: Tuple[str, ...]
    # Подстиль со своей нормой низа (ADR-0153 S5: classic rock 0.37, grunge 0.66 по эталонам 07.10): уровни ролей и
    # порог A9-модели окна; ``None`` — поля стиля.
    role_level_db: Optional[Mapping[str, float]] = None
    a9_model_low: Optional[float] = None


#: Окна клуба. ``club`` — сегодняшние таблицы (``_CLUB_*``, побайтно); ``deep`` — мягкие бочки, пэд
#: ``held``/``pumped16``
#: чаще; ``breaks`` — ломаная бочка (``KICK_PATTERNS["breakbeat"]``) в дропе, пэд ``stabs`` чаще. Пулы бочек —
#: подмножества :data:`KICK_SOUNDS`; темпы окон — решение координатора (deep 120–126: 128 уже окно ``club``;
#: breaks 130–138).
_CLUB_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "club": GenreWindow((128, 138), ("house", "deep", "techno", "garage"), _CLUB_LOOKS,
                        ("offbeat", "rolling8", "acid16"),
                        ("pumped16", "held", "stabs")),
    "deep": GenreWindow((120, 126), ("deep", "house"), _CLUB_LOOKS, ("offbeat", "rolling8"),
                       ("held", "held", "pumped16")),
    "breaks": GenreWindow((130, 138), ("techno", "garage"), _looks("breakbeat"), ("broken",),
                          ("stabs", "stabs", "pumped16", "held")),
}


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
    # Ритм-секция: пул бочек стиля (ключи :data:`KICK_SOUNDS`, первая — «лицо» стиля), вид секции по энергии,
    # каркасы хэтов/перкуссии.
    kick_pool: Tuple[str, ...]
    looks: Tuple[Tuple[int, Look], ...]
    kits: Mapping[str, Mapping[str, str]]
    # Роли и тембры: коридоры регистров, семьи тембров {семья → {роль → синты}} и семья темы не из таблицы.
    registers: Mapping[str, Tuple[int, int]]
    timbres: Mapping[str, Mapping[str, Tuple[str, ...]]]
    default_timbre: str
    # Фигуры: ключи генераторов ролей. Пэд и бас — пулы :data:`PAD_FIGURES`/:data:`BASS_FIGURES` (выбор по сиду
    # трека с историей, ADR-0152 PR-5/PR-6); лид — первый ключ.
    bass_figures: Tuple[str, ...]
    pad_figures: Tuple[str, ...]
    lead_figures: Tuple[str, ...]
    # Гармония: звуков в аккорде (терциями лада), прогрессии по ступеням.
    chord_size: int
    progressions: Tuple[Tuple[int, ...], ...]
    # Форма: шаблоны форм (ADR-0152 §3.5), ключ формы первого трека сета (тема раньше, #3427), формы по энергии
    # трека, блэнд (phrase_bars, bass_swap_bar), секции слоёв сэмплов.
    forms: Mapping[str, FormSpec]
    opening_form: str
    energy_forms: Mapping[int, Tuple[str, ...]]
    blend: Tuple[int, int]
    layer_sections: Mapping[str, Tuple[str, ...]]
    # Жанровые окна внутри стиля (первое — окно по умолчанию, поля выше совпадают с ним): ``genre_style``.
    genre_windows: Mapping[str, GenreWindow]
    # Микс: уровни ролей, роли под сайдчейном, LPF-свип секций, стерео ролей.
    role_level_db: Mapping[str, float]
    duck_roles: Tuple[str, ...]
    section_lpf: Mapping[str, Tuple[float, float]]
    lpf_roles: Tuple[str, ...]
    lpf_tail_sections: Tuple[str, ...]
    stereo: Mapping[str, Mapping[str, float]]
    # A9-модель трека (ADR-0152 §4): нижняя граница доли низа каждого дропа на роботе (``arrange.mix``).
    a9_model_low: float
    # Дуга энергии (ADR-0147 §3.4, #3459): кульминация (энергия 5) обязана прозвучать не позже этого трека сета
    # (с 1); при ~70 с на трек это 4-й трек ≈ 5–6 мин. Валидатор ризонера отвергает дугу LLM без пика в этом окне.
    peak_by_track: int = 4
    # Порядок тонов арпеджио (``arrange.pad.arp``, ADR-0153 S2): индексы голосов аккорда снизу вверх по 16-м, по кругу.
    arp_order: Tuple[int, ...] = (0, 1, 2, 1)
    # Луп-слой (ADR-0153 S3): роли :data:`SAMPLE_CATALOG`, из которых берётся файл лупа, и ключ генератора нарезки
    # (``arrange.compose.LOOP_GENERATORS``): ``chop`` — файл по порядку восьмыми (DJ_Dave), ``breakbeat_chop`` — брейк,
    # переставленный по сиду кусками по сетке 16-х.
    loop_roles: Tuple[str, ...] = ("loop",)
    loop_figure: str = "chop"
    # Свинг (ADR-0153 S4): шаги такта, которые опаздывают на ``SetPlan.swing`` доли восьмой (``arrange.rhythm.swing``):
    # у клуба — нечётные 16-е (гоуст-хэты каркасов), у lo-fi — оффбит-восьмые 2/6/10/14 (ratio долгой и короткой
    # восьмой = (1 + swing)/(1 − swing)). Хэты качаются всегда, роли ``swing_roles`` — тоже (ноты — ``offset_ms``).
    swing_steps: Tuple[int, ...] = tuple(range(1, 16, 2))
    # Роли, которых нет в треке энергии (``ENERGY_THIN_ROLES`` клуба: клэп снят на 1–2); у lo-fi малый 2/4 — сам бит.
    thin_roles: Mapping[int, Tuple[str, ...]] = field(default_factory=lambda: dict(ENERGY_THIN_ROLES))
    swing_roles: Tuple[str, ...] = ()
    # Бас: потолок хроматического подхода, доли (валидатор ``model``): walking подходит целой четвертью.
    approach_max_beats: float = APPROACH_MAX_BEATS
    # Удары файлами паков (ADR-0153 S4): роль → пул ключей :data:`SAMPLE_CATALOG` (бочка — :data:`KICK_SOUNDS`
    # с ``pack`` через ``kick_pool``); роли нет — ``play()`` дефолтного пака Renardo.
    drum_files: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)
    # Ритмы comping (``arrange.pad.comping``): на такт по кругу — удары ``(шаг 16-х, длина в 16-х)``.
    comp_rhythms: Tuple[Tuple[Tuple[int, int], ...], ...] = ()
    # Ударные (ADR-0153 S5). Бэкбит малого — в секциях этих видов (:func:`section_kind`; клуб — только дропы). Ролл
    # малого перед дропом — тактов (клуб 2: восьмые, затем 16-е; 0 — ролла нет). Fill на КАЖДОМ стыке секций (рок —
    # филл томами), а не только перед дропом и в конце. Такт fill-а у малого: ``roll`` — ролл 16-ми второй половины
    # (клуб), ``cut`` — малый молчит с последней доли (там томы). Филлы томами — рисунки последнего такта секции
    # (``X``/``x`` — акцент 3/2), выбор по сиду трека; пусто — роли ``toms`` нет.
    backbeat_kinds: Tuple[str, ...] = ("drop", "drop2")
    roll_bars: int = 2
    fill_joints: bool = False
    clap_fill: str = "roll"
    tom_fills: Tuple[str, ...] = ()
    # Роли каталога сэмплов, из которых берётся FX-удар секций (``layer_sections["fx"]``): у рока — крэш пака.
    fx_roles: Tuple[str, ...] = ("fx",)
    # Пауэр-аккорды (``arrange.pad.power_chords``): ритмы на такт по кругу — ``(шаг 16-х, длина в 16-х)``.
    power_rhythms: Tuple[Tuple[Tuple[int, int], ...], ...] = ()
    # Рифф баса (``arrange.bass.riff``): такты по кругу, восьмые такта — ``R`` тон такта, ``F`` квинта аккорда, ``O``
    # октава тона такта, ``.`` — нота тянется (длина до следующей).
    bass_riffs: Tuple[str, ...] = ()
    # Джаз (ADR-0153 S6). Каденция — аккорды последних тактов секции со сменами темы (:data:`SECTION_HARMONY`
    # ``CHANGES``): ``(ступень, качество)`` — ключ :data:`CHORD_INTERVALS`, пусто — диатоническое трезвучие ступени.
    cadence: Tuple[Tuple[int, str], ...] = ()
    # Лид секции по её виду (:func:`section_kind`) → ключ ``arrange.compose.SOLO_GENERATORS``; вида нет — развитие хука.
    section_leads: Mapping[str, str] = field(default_factory=dict)
    # Фразы соло (``arrange.lead.solo``) по 2 такта: ноты ``(шаг 16-х от начала фразы 0..31, длина в 16-х)``, хвост —
    # пауза (солист дышит). Доля слабых нот перед нотой на доле, которые подходят к ней полутоном (хроматика), а не
    # ступенью лада.
    solo_phrases: Tuple[Tuple[Tuple[int, int], ...], ...] = ()
    solo_chromatic: float = 0.0
    # Голосоведение баса против лида (``arrange.bass.against_lead``, #3549): интервалы «лид − бас» по модулю октавы,
    # которых бас избегает на нотах, звучащих вместе с нотой лида (1 — малая нона/секунда, 11 — та же пара полутоном
    # сверху), и интервалы, на которые бас и лид не идут параллельно в одну сторону (0 — октавы, 7 — квинты). Пусто —
    # бас как его сыграл генератор (клуб побайтно тот же).
    bass_lead_clash: Tuple[int, ...] = ()
    bass_lead_parallels: Tuple[int, ...] = ()
    # Ручки мастер-шины стиля поверх :data:`SET_LEVELER` (``arrange.mix.set_master``; ключи — :data:`MASTER_DEFAULTS`,
    # каждый старт трека выставляет все ручки заново, поэтому следующий трек другого стиля их не наследует).
    master: Mapping[str, float] = field(default_factory=dict)


# ── Стиль ``rave`` (ADR-0153 S1: rave/acid/hardcore): клубная механика, свои окна темпа, бочки и тембры ─────────
#: Окна рейва 140–160 (ADR-0153 §3): ``rave`` — жёсткая прямая бочка и стэбы, ``acid`` — линия ``tb303`` чаще,
#: ``hardcore`` — 150–160, перегруженная бочка. Виды секций (рисунок бочки, насос) — клубные: механика та же.
#: Бас в окне ``acid`` — ``acid16`` весом 2 (повтор в пуле = вес выбора, как у ``pad_figures``).
_RAVE_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "rave": GenreWindow((140, 150), ("hardtechno", "hardstyle", "acid"), _CLUB_LOOKS,
                        ("offbeat", "rolling8", "acid16"), ("stabs", "stabs", "pumped16", "held")),
    "acid": GenreWindow((140, 148), ("acid", "hardtechno"), _CLUB_LOOKS,
                        ("acid16", "acid16", "offbeat"), ("stabs", "pumped16", "held")),
    "hardcore": GenreWindow((150, 160), ("hardcore", "hardstyle"), _CLUB_LOOKS,
                            ("offbeat", "rolling8"), ("stabs", "stabs", "pumped16")),
}
#: Тембры рейва: те же семьи тем (``THEME_TIMBRE``), что у клуба. Лиды — читаемые темой (:data:`THEME_LEAD_OK`, замер
#: 07.10): ``saw``/``brass``/``arpy``/``blip``/``orient``/``pluck``; ``hoover``/``rave``/``supersawlead`` (хорус,
#: шумовой ``Gendy1``, неотпускаемая огибающая — «какофония, сильное эхо», Шифу 07.10) теме не годятся, а других
#: партий лида в треке нет (лид — всегда хук или мотив). ``tb303`` с ``acid16`` — в каждой семье
#: (лицо стиля), низ держат басы с долей низа ≥ 0.9 (``subbass``/``dub``/``jbass``/``wobblebass``) рисунками
#: ``offbeat``/``rolling8``. Пэды — клубные (стэб-пэд на синтах лида ``rave``/``saw`` требует замера роли ``pad``,
#: ADR-0152 §3.1 — не сделан).
_RAVE_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("saw", "brass", "arpy"), "bass": ("subbass", "jbass", "tb303"),
             "pad": _CLUB_TIMBRES["dark"]["pad"]},
    "hard": {"lead": ("saw", "brass", "arpy", "blip"), "bass": ("wobblebass", "jbass", "tb303"),
             "pad": _CLUB_TIMBRES["hard"]["pad"]},
    "bright": {"lead": ("saw", "arpy", "blip", "orient"), "bass": ("jbass", "subbass", "tb303"),
               "pad": _CLUB_TIMBRES["bright"]["pad"]},
    "warm": {"lead": ("saw", "brass", "pluck"), "bass": ("jbass", "subbass", "tb303"),
             "pad": _CLUB_TIMBRES["warm"]["pad"]},
}
#: Каркасы рейва — клубные без качающихся (``shuffle``/``ride``): рейв ровный (S2 ADR-0153: свинг-ratio ≤ 1.2).
_RAVE_KITS: Mapping[str, Mapping[str, str]] = {k: _CLUB_KITS[k] for k in ("offbeat", "sixteenths", "open")}

# ── Стили ``synthwave`` и ``chiptune`` (ADR-0153 S2): клубная форма и микс, свои окна, бочки, пэды и тембры ───────
#: Synthwave/outrun 100–120 (§3): мягкий насос (0.3 в дропе вместо 1.0 у клуба — «без насоса или мягкий»), окно
#: ``synthwave`` — бочка ``outrun`` (``X...X...X...X..X``) в дропе, бас только оффбит (шаг 15 бочки занят — ``rolling8``
#: на нём звучал бы с бочкой); окно ``retrowave`` — прямая бочка, бас оффбит/ролл. Пэд держит аккорд (``held``,
#: ``warmpad``/``strangerpulsepad`` созданы под этот звук) или арпеджио (``arp``). Бочки — мягкие клубные, замеренные.
_SYNTHWAVE_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["outrun"], 0.3)),
    (5, Look("X.......X.......", 0.2)),
    (0, Look(KICK_PATTERNS["four_on_floor"], 0.2)),
)
_RETROWAVE_LOOKS: Tuple[Tuple[int, Look], ...] = ((7, Look(KICK_PATTERNS["four_on_floor"], 0.3)), *_SYNTHWAVE_LOOKS[1:])
_SYNTHWAVE_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "synthwave": GenreWindow((100, 120), ("deep", "house"), _SYNTHWAVE_LOOKS, ("offbeat",), ("held", "held", "arp")),
    "retrowave": GenreWindow((108, 120), ("house", "deep"), _RETROWAVE_LOOKS, ("offbeat", "rolling8"),
                             ("held", "arp", "arp")),
}
#: Тембры synthwave — семьи тем как у клуба. Лиды — читаемые темой (:data:`THEME_LEAD_OK`): ``saw``, ``brass``,
#: ``keys``, ``arpy``; ``strangerarp``/``supersawlead`` сняты 07.10 — огибающая без ``gate`` держит ноту до ``sus·8``
#: (хвост ≥ 1.6 с, тема «размазана», :data:`LEAD_CLARITY`). ``cs80lead`` (звук жанра) НЕ
#: взят: на роботе 07.10 оба трека с ним дали секунды цифрового нуля и шум выше среза мастера (:data:`NYQUIST_MAX_MIDI`).
#: Басы — с долей низа ≥ 0.9 (``BASS_MIN_LOW``): ``retrobass`` (0.59) и ``moogbass`` (разброс замера 8 дБ) в семьи не
#: идут. Пэды: держащие (``warmpad``/``strangerpulsepad`` — с хвостом, только ``held``) и короткие для ``arp``.
_SYNTHWAVE_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("saw", "keys", "brass"), "bass": ("subbass", "jbass"),
             "pad": ("strangerpulsepad", "space", "saw")},
    "hard": {"lead": ("saw", "brass", "arpy"), "bass": ("jbass", "subbass"),
             "pad": ("strangerpulsepad", "sinepad", "pulse")},
    "bright": {"lead": ("brass", "arpy", "keys"), "bass": ("bass", "jbass"),
               "pad": ("warmpad", "strings", "pulse")},
    "warm": {"lead": ("keys", "saw", "brass"), "bass": ("bass", "jbass", "subbass"),
             "pad": ("warmpad", "sinepad", "saw")},
}
_SYNTHWAVE_KITS: Mapping[str, Mapping[str, str]] = {k: _CLUB_KITS[k] for k in ("offbeat", "open")}
_SYNTHWAVE_FIELDS = dict(
    swing=(0.0, 0.03), modes=("minor", "dorian"), looks=_SYNTHWAVE_LOOKS, kits=_SYNTHWAVE_KITS,
    registers=_CLUB_REGISTERS, timbres=_SYNTHWAVE_TIMBRES, default_timbre="warm", lead_figures=("motif",),
    chord_size=3, progressions=_CLUB_PROGRESSIONS, forms=_CLUB_FORMS, opening_form=_CLUB_OPENING_FORM,
    energy_forms=_CLUB_ENERGY_FORMS, blend=_CLUB_BLEND, layer_sections=_CLUB_LAYER_SECTIONS,
    genre_windows=_SYNTHWAVE_GENRE_WINDOWS, role_level_db=_CLUB_ROLE_LEVEL_DB, duck_roles=_CLUB_DUCK_ROLES,
    section_lpf=_CLUB_SECTION_LPF, lpf_roles=_CLUB_LPF_ROLES, lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS,
    stereo=_CLUB_STEREO,
    # A9: норма низа synthwave 0.4–0.7 (§3, гипотеза до эталона В1) + запас σ модели 0.1, как клуб (0.5 + 0.1)
    a9_model_low=0.5, arp_order=(0, 1, 2, 1),
)
#: Chiptune/8-bit 120–160 (§3): без насоса (сайдчейна нет), хэты 16-ми, арпеджио вместо пэда (``arp`` — вдвое чаще
#: стэбов), лиды ``pulse``/``blip``/``saw``/``orient`` (``varsaw`` — атака 65 мс, теме не годится, :data:`LEAD_CLARITY`;
#: ``square`` как лид на потолке ``amp`` не достаёт цели роли −50
#: на 3 дБ — он в арпеджио). Бас: оффбит/ролл на басах с низом и ``octave8`` — ``pulse`` октавами (8-битный бас,
#: :data:`BASS_FIGURE_SYNTHS`). Бочки — клубные замеренные (свои «шумовые» ударные — отдельный SynthDef/пак, не здесь).
_CHIP_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["four_on_floor"], 0.0)),
    (5, Look("X.......X.......", 0.0)),
    (0, Look(KICK_PATTERNS["four_on_floor"], 0.0)),
)
_CHIPTUNE_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "chiptune": GenreWindow((120, 160), ("techno", "garage"), _CHIP_LOOKS, ("offbeat", "rolling8", "octave8"),
                            ("arp", "arp", "stabs")),
}
_CHIPTUNE_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("pulse", "blip", "saw"), "bass": ("subbass", "jbass", "pulse"),
             "pad": ("square", "pulse")},
    "hard": {"lead": ("pulse", "saw", "blip"), "bass": ("jbass", "subbass", "pulse"),
             "pad": ("square", "blip")},
    "bright": {"lead": ("blip", "pulse", "orient"), "bass": ("bass", "jbass", "pulse"),
               "pad": ("pulse", "square")},
    "warm": {"lead": ("pulse", "blip", "saw"), "bass": ("bass", "jbass", "pulse"),
             "pad": ("blip", "square", "pulse")},
}
_CHIPTUNE_KITS: Mapping[str, Mapping[str, str]] = {k: _CLUB_KITS[k] for k in ("sixteenths", "offbeat")}
_CHIPTUNE_FIELDS = dict(
    swing=(0.0, 0.0), modes=("major", "minor"), looks=_CHIP_LOOKS, kits=_CHIPTUNE_KITS,
    registers=_CLUB_REGISTERS, timbres=_CHIPTUNE_TIMBRES, default_timbre="bright", lead_figures=("motif",),
    chord_size=3, progressions=_CLUB_PROGRESSIONS, forms=_CLUB_FORMS, opening_form=_CLUB_OPENING_FORM,
    energy_forms=_CLUB_ENERGY_FORMS, blend=_CLUB_BLEND, layer_sections=_CLUB_LAYER_SECTIONS,
    genre_windows=_CHIPTUNE_GENRE_WINDOWS, role_level_db=_CLUB_ROLE_LEVEL_DB, duck_roles=(),
    section_lpf=_CLUB_SECTION_LPF, lpf_roles=_CLUB_LPF_ROLES, lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS,
    stereo=_CLUB_STEREO,
    # A9: норма низа chiptune 0.3–0.5 (§3: суб-баса нет по природе) + запас σ модели 0.1
    a9_model_low=0.4, arp_order=(0, 1, 2),
)

# ── Стили ``breaks`` и ``dnb`` (ADR-0153 S3): брейк пака нарезкой по сиду, ломаная бочка, бас мимо неё ───────────
#: Breaks 128–140, dnb 160–176 (§3). Ударные держит брейк пака (роль ``break``, :data:`PACK_BREAK_BEATS`), нарезанный
#: по сетке 16-х (``breakbeat_chop``), бочка подпирает его рисунком окна: breaks — ``breakbeat``, dnb — «two-step»
#: ``half_time`` (бочка 1 и «и» третьей доли, клэп 2/4 — при 170 BPM это малый в half-time). Насос мягкий (0.5 в
#: дропе): брейк под сайдчейн не идёт (он сам ударные), качаются бас и пэд. Интро/аутро — рисунок дропа тише насосом,
#: build — бочка на 1 и 3.
_BREAKS_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["breakbeat"], 0.5)),
    (5, Look("X.......X.......", 0.3)),
    (0, Look(KICK_PATTERNS["breakbeat"], 0.3)),
)
_DNB_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["half_time"], 0.5)),
    (5, Look("X.......X.......", 0.3)),
    (0, Look(KICK_PATTERNS["half_time"], 0.3)),
)
#: Окна. Бас — ``broken`` (шаги 1, 6, 9, 14: мимо ``breakbeat`` 0/2/10/13, ``half_time`` 0/10 и долей); пэд держит
#: аккорд (``held``) или стэбы — без ``pumped16`` (16-е аккорда × 2 голоса — самый дорогой по событиям рисунок, а dnb
#: и так на 30 % быстрее клуба, ADR-0153 §3.5 / В8). Бочки — клубные замеренные (``KICK_SOUNDS``), низ брейка — свой.
_BREAKS_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "breakbeat": GenreWindow((128, 136), ("techno", "garage"), _BREAKS_LOOKS, ("broken",), ("held", "stabs", "stabs")),
    "bigbeat": GenreWindow((132, 140), ("garage", "house"), _BREAKS_LOOKS, ("broken",), ("stabs", "held")),
}
_DNB_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "dnb": GenreWindow((170, 176), ("techno", "garage"), _DNB_LOOKS, ("broken",), ("held", "held", "stabs")),
    "jungle": GenreWindow((160, 170), ("garage", "deep"), _DNB_LOOKS, ("broken",), ("held", "stabs")),
}
#: Тембры breaks/dnb — семьи тем как у клуба. Бас — замеренные с низом ≥ 0.9 (``BASS_MIN_LOW``): ``wobblebass``
#: (ближайший к reese синт палитры; синта ``reese`` в Renardo нет — только файлы ``j``, замер #3430: низ 0.9),
#: ``subbass``, ``jbass``. Лид редкий и простой (``motif``), пэды — клубные семьи.
_BREAKBEAT_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("saw", "pluck", "blip"), "bass": ("wobblebass", "subbass", "jbass"),
             "pad": _CLUB_TIMBRES["dark"]["pad"]},
    "hard": {"lead": ("brass", "arpy", "blip"), "bass": ("wobblebass", "jbass"),
             "pad": _CLUB_TIMBRES["hard"]["pad"]},
    "bright": {"lead": ("pluck", "blip", "orient"), "bass": ("jbass", "subbass", "wobblebass"),
               "pad": _CLUB_TIMBRES["bright"]["pad"]},
    "warm": {"lead": ("pluck", "keys", "arpy"), "bass": ("subbass", "jbass", "wobblebass"),
             "pad": _CLUB_TIMBRES["warm"]["pad"]},
}
#: Минор, два аккорда на 8 тактов (§3: аккорд на 2 такта × 4 — ступень держится 4 такта).
_DNB_PROGRESSIONS: Tuple[Tuple[int, ...], ...] = ((0, 0, 5, 5), (0, 0, 3, 3), (0, 0, 6, 6), (0, 0, 4, 4), (0, 0, 2, 2))
#: Брейк звучит в build и дропах (без него — интро/аутро блэнда и брейкдаун: две деки брейков в блэнде дали бы
#: удвоенные удары); psr-слоя DJ_Dave нет — его место занимает брейк.
_BREAKBEAT_LAYER_SECTIONS: Mapping[str, Tuple[str, ...]] = {"loop": ("build", "build2", "drop", "drop2"),
                                                            "fx": ("drop", "drop2")}
#: Уровни — клубные, брейк — ударная опора: −36 (у клуба брейк-луп второго дропа −40 «текстурой»).
_BREAKBEAT_ROLE_LEVEL_DB: Mapping[str, float] = {**_CLUB_ROLE_LEVEL_DB, "loop": -36.0}
_BREAKBEAT_FIELDS = dict(
    modes=("minor", "dorian"), kits={k: _CLUB_KITS[k] for k in ("offbeat", "open")},
    registers=_CLUB_REGISTERS, timbres=_BREAKBEAT_TIMBRES, default_timbre="dark", lead_figures=("motif",),
    chord_size=3, forms=_CLUB_FORMS, opening_form=_CLUB_OPENING_FORM,
    energy_forms=_CLUB_ENERGY_FORMS, blend=_CLUB_BLEND, layer_sections=_BREAKBEAT_LAYER_SECTIONS,
    role_level_db=_BREAKBEAT_ROLE_LEVEL_DB, duck_roles=("bass", "pad"),
    section_lpf=_CLUB_SECTION_LPF, lpf_roles=_CLUB_LPF_ROLES, lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS,
    stereo=_CLUB_STEREO, loop_roles=("break",), loop_figure="breakbeat_chop",
    # A9: норма низа dnb/breaks 0.6–0.85 (§3, гипотеза до эталона В1) — нижняя граница, как у клуба и рейва
    a9_model_low=0.6,
)

# ── Стиль ``lofi`` (ADR-0153 S4, ступень к джазу): свинг оффбит-восьмых, comping, walking-бас, кит паков ───────────
#: Эталон (``style_reference_profiles.md`` 07.10; lo-fi hip-hop записи нет, ближайший — dark jazz/trip-hop): темп
#: 87–89 во всех 40 окнах, свинг 1.26 [1.10..1.43], низ < 150 Гц 0.75 [0.63..0.84], LR 0.90, σ по минутам 1.1 дБ;
#: джаз-кафе: низ 0.27 [0.08..0.48], свинг 1.54. Темп lo-fi 72–92 (окно ``lofi`` 82–92 — как эталон 88).
#: Свинг (доля восьмой) 0.14–0.22: ratio восьмых (1 + s)/(1 − s) = 1.33–1.56 — между trip-hop 1.26 и джазом 1.54,
#: не триоль 2.0. Бочка boom-bap мягкая из паков, малый 2/4 и хэты восьмыми — файлы паков (``drum_files``), без
#: сайдчейна. Низ трека — цель 0.50–0.75 (выше p90 джаза 0.48, не выше медианы trip-hop 0.75): lo-fi — бит-музыка
#: на бочке и басе, как trip-hop, но с comping-клавишами впереди; ``a9_model_low`` — нижняя граница (см. уровни ниже).
_LOFI_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["boombap"], 0.0)),
    (5, Look(KICK_PATTERNS["lazy"], 0.0)),
    (0, Look(KICK_PATTERNS["lazy"], 0.0)),
)
#: Пэд — только comping, бас — только walking (ADR-0153 §5 S3: фигуры стиля в ≥ 90 % треков); разнообразие — ритмы
#: comping, синты, каркасы, бочки, окна.
_LOFI_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "lofi": GenreWindow((82, 92), ("jazz", "soft", "acoustic"), _LOFI_LOOKS, ("walking",), ("comping",)),
    "chillhop": GenreWindow((72, 82), ("soft", "acoustic", "jazz"), _LOFI_LOOKS, ("walking",), ("comping",)),
}
#: Хэты восьмыми (оффбит-восьмые качает свинг): акцент на доле, на «и» или реже.
_LOFI_KITS: Mapping[str, Mapping[str, str]] = {
    "lazy": {"hats": "X.x.X.x.X.x.X.x.", "perc": "................"},
    "pushed": {"hats": "x.X.x.X.x.X.x.X.", "perc": "................"},
    "sparse": {"hats": "X.x.X...X.x.X.x.", "perc": "................"},
}
#: Тембры: лид — читаемый темой (:data:`THEME_LEAD_OK`) и без Хааса (``SYNTH_STEREO``: задержка голоса заняла бы
#: ``delay`` свинга нот) — ``keys`` (клавиши), ``pluck``, ``arpy``, ``orient``, ``brass``; пэд comping — замеренные в
#: роли пэда без хвоста (``rhpiano``/``epiano`` в роли пэда не замерены — не взяты); бас — с низом ≥ 0.9.
_LOFI_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("keys", "pluck"), "bass": ("subbass", "jbass"), "pad": ("sinepad", "space")},
    "hard": {"lead": ("keys", "arpy", "pluck"), "bass": ("jbass", "bass"), "pad": ("sinepad", "strings")},
    "bright": {"lead": ("pluck", "orient", "keys"), "bass": ("bass", "jbass"), "pad": ("ambi", "sinepad")},
    "warm": {"lead": ("keys", "pluck", "brass"), "bass": ("bass", "jbass", "subbass"),
             "pad": ("sinepad", "ambi", "strings")},
}
#: Септаккорды ii–V–I и соседи по ступеням (аккорд на 2 такта).
_LOFI_PROGRESSIONS: Tuple[Tuple[int, ...], ...] = (
    (1, 4, 0, 5), (0, 5, 1, 4), (1, 4, 0, 0), (3, 6, 2, 5), (0, 3, 1, 4), (5, 1, 4, 0),
)
#: Comping: ни одного удара на сильных долях 1 и 3 (шаги 0, 8) — там хук; короткие (8-я — пунктирная 8-я).
_LOFI_COMP_RHYTHMS: Tuple[Tuple[Tuple[int, int], ...], ...] = (
    ((6, 2), (14, 2)), ((4, 3), (10, 2)), ((2, 2), (12, 3)), ((6, 2), (10, 2), (14, 2)),
)
#: «Винил» дёшево: пэд под постоянным LPF 2.4 кГц (тусклые клавиши); хвост блэнда закрывается, как у клуба.
#: Треск пластинки (шумовой слой) не сделан — нужна своя роль/слот деки.
_LOFI_SECTION_LPF: Mapping[str, Tuple[float, float]] = {
    **{name: (2400.0, 2400.0) for name in ("intro", "intro_low", "build", "build2", "drop", "break", "break2",
                                           "drop2", "outro")},
    "outro_tail": (2400.0, 300.0),
}
_LOFI_WINDOW = _LOFI_GENRE_WINDOWS["lofi"]
_LOFI_STYLE = Style(
    bpm=_LOFI_WINDOW.bpm, swing=(0.14, 0.22), modes=("dorian", "minor", "major"),
    kick_pool=_LOFI_WINDOW.kick_pool, looks=_LOFI_LOOKS, kits=_LOFI_KITS, registers=_CLUB_REGISTERS,
    timbres=_LOFI_TIMBRES, default_timbre="warm", bass_figures=_LOFI_WINDOW.bass_figures,
    pad_figures=_LOFI_WINDOW.pad_figures, lead_figures=("motif",), chord_size=4, progressions=_LOFI_PROGRESSIONS,
    forms=_CLUB_FORMS, opening_form=_CLUB_OPENING_FORM, energy_forms=_CLUB_ENERGY_FORMS, blend=_CLUB_BLEND,
    layer_sections={}, genre_windows=_LOFI_GENRE_WINDOWS,
    # Уровни: робот 08.10 (2 сета, клубные уровни −33/−32/−44/−50) — низ 0.77/0.76 по медиане окон 20 с, в дропах
    # 0.79–0.91 при A9-модели 0.61–0.68: бочка и бас тише на 2 дБ, пэд и лид громче на 3/2 дБ.
    role_level_db={"kick": -35.0, "bass": -34.0, "pad": -41.0, "lead": -48.0, "clap": -43.0, "hats": -57.0},
    duck_roles=(), section_lpf=_LOFI_SECTION_LPF, lpf_roles=("pad",), lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS,
    stereo={"pad": {"pan": 0.5, "detune": PAD_DETUNE}},
    # A9-модель занижает низ lo-fi против записи робота 08.10 на 0.09–0.32 (0.61/0.68/0.55 модели при 0.77/0.76/0.87
    # записи; удары файлами и walking в модели — оценки): при пороге 0.5 она глушила пэд на 6 дБ, а запись и так
    # выше цели. Порог 0.4 — только страховка от трека совсем без низа; калибровка модели lo-fi не сделана.
    a9_model_low=0.4,
    thin_roles={}, swing_steps=(2, 6, 10, 14), swing_roles=("kick", "clap", "bass", "pad", "lead"),
    approach_max_beats=1.0,
    drum_files={"clap": ("sonicpi_drum_snare_soft", "muldjord_snare_40", "muldjord_snare_43"),
                "hats": ("muldjord_hihatclosed_16", "muldjord_hihatclosed_18", "sonicpi_drum_cymbal_pedal")},
    comp_rhythms=_LOFI_COMP_RHYTHMS,
)

# ── Стиль ``rock`` (ADR-0153 S5): бэкбит 2/4, филлы томами на стыках, пауэр-аккорды, бас-рифф, куплет/припев/бридж ──
#: Эталоны 07.10 (``style_reference_profiles.md``, 16 кГц, медиана [p10..p90] окон 20 с): classic rock — низ < 150 Гц
#: 0.37 [0.11..0.63], середина 0.54, верх 0.10, crest 15.1, LR 0.76, темп ≈ 110 [81..140], свинг ≈ 1.2 (прямые восьмые);
#: grunge (Nirvana) — низ 0.66 [0.26..0.84], середина 0.31, crest 13.2, LR 0.87, темп ≈ 121. Один норматив низа на
#: «рок» неверен (вывод 1 эталонов) — два окна-подстиля со своими уровнями ролей и порогом A9-модели
#: (``GenreWindow.role_level_db``/``a9_model_low``). Окно выбирает слово фразы («гранж» — :data:`WINDOW_WORDS`), иначе
#: план по сиду со штрафом за окна прошлых сетов.
_ROCK_HALF = Look("X.......X.......", 0.0)
_ROCK_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["rock"], 0.0)), (5, Look(KICK_PATTERNS["rock_straight"], 0.0)), (0, _ROCK_HALF))
_GRUNGE_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["grunge"], 0.0)), (5, Look(KICK_PATTERNS["rock"], 0.0)), (0, _ROCK_HALF))
#: Уровни ролей (шкала модели, роль звучит всю секцию): гитара (пэд, пауэр-аккорды) — середина 150–2000 Гц, малый
#: громче клубного клэпа; у гранжа бочка и бас громче. Подобраны по записям робота 08.10 (``jack_rec`` 150 с, моно-сумма,
#: окна 20 с, медиана): classic — низ 0.39 [0.33..0.42], середина 0.59, crest 13.3, LR 0.97 (эталон 0.37/0.54/15.1/0.76);
#: grunge — низ 0.74 [0.52..0.80], середина 0.25, crest 13.4, LR 0.98 (эталон 0.66/0.31/13.2/0.87); при бочке/басе
#: гранжа −35/−34 низ был 0.47. A9-модель для рока не откалибрована (classic 0.09 при записи 0.39: удары файлами паков
#: и хвосты в модели — оценки), порог 0.05 — только страховка от трека без низа.
_ROCK_CLASSIC_LEVEL_DB: Mapping[str, float] = {
    "kick": -39.0, "bass": -39.0, "pad": -33.0, "lead": -47.0, "clap": -42.0, "hats": -55.0, "toms": -46.0,
    "fx": -44.0}
_ROCK_GRUNGE_LEVEL_DB: Mapping[str, float] = {
    "kick": -32.0, "bass": -31.0, "pad": -33.0, "lead": -49.0, "clap": -41.0, "hats": -56.0, "toms": -45.0,
    "fx": -44.0}
_ROCK_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "classic": GenreWindow((100, 125), ("rock_r", "acoustic", "rock_r2"), _ROCK_LOOKS, ("riff",), ("power_chords",),
                           _ROCK_CLASSIC_LEVEL_DB, 0.05),
    "grunge": GenreWindow((110, 130), ("rock_r2", "rock_l", "rock_r"), _GRUNGE_LOOKS, ("riff",), ("power_chords",),
                          _ROCK_GRUNGE_LEVEL_DB, 0.05),
}
#: Хэты восьмыми (акцент на доле), «толкающие» (акцент на «и»), четверти (райд): удары только на восьмых.
_ROCK_KITS: Mapping[str, Mapping[str, str]] = {
    "eighths": {"hats": "X.x.X.x.X.x.X.x.", "perc": "................"},
    "pushed": {"hats": "x.X.x.X.x.X.x.X.", "perc": "................"},
    "quarters": {"hats": "X...x...X...x...", "perc": "................"},
}
#: Лид — читаемый темой (:data:`THEME_LEAD_OK`, #3522): ``saw``/``pulse`` — «гитарное» соло, ``brass``/``pluck``/``arpy``/
#: ``keys``; бас — с низом на роботе ≥ 0.9. Гитара (В4, проба прелоада 08.10): ``fuzz`` проиграл — его патч (#3008)
#: делит ``freq`` на 2 и модулирует ``Saw`` между 0.5 и 1.5 частоты ноты: пауэр-аккорд в регистре гитары звучит
#: суб-октавой, низ записи 0.79 и 0.75 при бочке/басе −36 и −42 дБ (середина 0.18–0.25, LR 1.00 — гитары не слышно);
#: ``dirt`` — без замера громкости и полос (в таблицах модели его нет). ``saw`` (замер в рамке пэда, середина 0.94) дал
#: низ 0.39 — «чистая» гитара без перегруза; перегруз — свой ``rbx_guitar`` (фильтр до ``tanh``, пересборка образа).
_ROCK_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("saw", "pluck"), "bass": ("jbass", "bass"), "pad": ("saw",)},
    "hard": {"lead": ("saw", "pulse", "arpy"), "bass": ("jbass", "bass"), "pad": ("saw",)},
    "bright": {"lead": ("arpy", "pluck", "saw"), "bass": ("bass", "jbass"), "pad": ("saw",)},
    "warm": {"lead": ("brass", "saw", "keys"), "bass": ("bass", "jbass", "subbass"), "pad": ("saw",)},
}
#: Прогрессии по ступеням (аккорд на 2 такта): i–bVII–bVI–bVII, i–bVI–bVII–i, I–IV–bVII–IV, I–bVII–IV–I (каденция
#: bVII→I), i–iv–v–iv, i–bVI–iv–v, I–V–vi–IV. Ступени 0, 3, 4, 5, 6 — ни одного уменьшённого аккорда в миноре и
#: миксолидийском: квинта ступени — чистая, пауэр-аккорд в ладу.
_ROCK_PROGRESSIONS: Tuple[Tuple[int, ...], ...] = (
    (0, 6, 5, 6), (0, 5, 6, 0), (0, 3, 6, 3), (0, 6, 3, 0), (0, 3, 4, 3), (0, 5, 3, 4), (0, 4, 5, 3),
)
_ROCK_DRUMS = frozenset({"kick", "hats", "clap"})
_ROCK_FULL = _ROCK_DRUMS | {"bass", "pad", "lead", "toms"}
#: Форма рока (ADR-0153 §3: verse/chorus/bridge): интро 4+4 и хвост outro/outro_tail 4+4 — как у клуба (блэнд двух
#: дек, ``model.blend_bars``); куплет — начало хука, припев — хук (первый — тема целиком), бридж — хук вдвое медленнее
#: (:data:`SECTION_KINDS`). Филл томами — последний такт каждой секции (``Style.fill_joints``), крэш — первая доля
#: секций с ``fx``.
_ROCK_INTRO: FormSpec = (
    ("intro", _CLUB_BLEND[1], 2, frozenset({"hats", "pad"})),
    ("intro_low", _CLUB_BLEND[0] - _CLUB_BLEND[1], 3, _ROCK_DRUMS | {"bass", "pad", "toms"}),
)
_ROCK_TAIL: FormSpec = (
    ("outro", 8 - (_CLUB_BLEND[0] - _CLUB_BLEND[1]), 3, _ROCK_DRUMS | {"bass", "pad"}),
    ("outro_tail", _CLUB_BLEND[0] - _CLUB_BLEND[1], 1, frozenset({"hats", "pad"})),
)
_ROCK_FORMS: Mapping[str, FormSpec] = {
    "rock48": (*_ROCK_INTRO, ("verse", 8, 5, _ROCK_FULL), ("chorus", 8, 8, _ROCK_FULL), ("verse2", 8, 6, _ROCK_FULL),
               ("chorus2", 8, 9, _ROCK_FULL), *_ROCK_TAIL),
    "rock32": (*_ROCK_INTRO, ("verse", 8, 5, _ROCK_FULL), ("chorus", 8, 8, _ROCK_FULL), *_ROCK_TAIL),
    "rock64": (*_ROCK_INTRO, ("verse", 8, 5, _ROCK_FULL), ("chorus", 8, 8, _ROCK_FULL), ("verse2", 8, 6, _ROCK_FULL),
               ("chorus2", 8, 9, _ROCK_FULL), ("bridge", 8, 4, _ROCK_FULL), ("chorus3", 8, 9, _ROCK_FULL),
               *_ROCK_TAIL),
    # Первый трек сета: припев (тема) сразу после интро (#3427), куплет и бридж — потом.
    "chorusfirst48": (*_ROCK_INTRO, ("chorus", 8, 8, _ROCK_FULL), ("verse", 8, 5, _ROCK_FULL),
                      ("bridge", 8, 4, _ROCK_FULL), ("chorus2", 8, 9, _ROCK_FULL), *_ROCK_TAIL),
}
_ROCK_ENERGY_FORMS: Mapping[int, Tuple[str, ...]] = {
    1: ("rock32", "rock48"), 2: ("rock32", "rock48"), 3: ("rock32", "rock48", "chorusfirst48"),
    4: ("rock32", "rock48", "chorusfirst48", "rock64"), 5: ("rock64", "rock48", "chorusfirst48"),
}
#: Филлы томами — последний такт секции (``X``/``x`` — акцент 3/2): последняя доля 16-ми, третья доля восьмыми +
#: четвёртая 16-ми, «и» третьей + 16-е, две доли 16-ми. Малый и бочка молчат с последней доли (``clap_fill`` ``cut``,
#: ``rhythm.KICK_CUT_STEP``).
_ROCK_TOM_FILLS: Tuple[str, ...] = (
    "............xxXX", "........x.x.xxXX", "..........x.xxXX", "........xxxxxxXX")
#: Ритмы пауэр-аккордов по тактам по кругу: палм-мьют восьмыми (16-я звучит, 16-я глушится), открытые восьмые,
#: снова палм-мьют, «толчок» 3+3+2.
_ROCK_POWER_CHUG = tuple((step, 1) for step in range(0, 16, 2))
_ROCK_POWER_DRIVE = tuple((step, 2) for step in range(0, 16, 2))
_ROCK_POWER_RHYTHMS: Tuple[Tuple[Tuple[int, int], ...], ...] = (
    _ROCK_POWER_CHUG, _ROCK_POWER_DRIVE, _ROCK_POWER_CHUG, ((0, 3), (3, 3), (6, 2), (8, 3), (11, 3), (14, 2)))
#: Рифф баса: такт восьмыми на тоне такта, такт с квинтой и октавой в конце (двутактовый рифф по кругу).
_ROCK_BASS_RIFFS: Tuple[str, ...] = ("RRRRRRRR", "RRRRRRFO")
_ROCK_SECTION_LPF: Mapping[str, Tuple[float, float]] = {"outro_tail": (LPF_TOP_HZ, 300.0)}
_ROCK_WINDOW = _ROCK_GENRE_WINDOWS["classic"]
_ROCK_STYLE = Style(
    bpm=_ROCK_WINDOW.bpm, swing=(0.0, 0.02), modes=("minor", "mixolydian"),
    kick_pool=_ROCK_WINDOW.kick_pool, looks=_ROCK_WINDOW.looks, kits=_ROCK_KITS,
    registers={"bass": (36, 52), "pad": (43, 67), "lead": (55, 84)},
    timbres=_ROCK_TIMBRES, default_timbre="hard", bass_figures=_ROCK_WINDOW.bass_figures,
    pad_figures=_ROCK_WINDOW.pad_figures, lead_figures=("motif",), chord_size=3, progressions=_ROCK_PROGRESSIONS,
    forms=_ROCK_FORMS, opening_form="chorusfirst48", energy_forms=_ROCK_ENERGY_FORMS, blend=_CLUB_BLEND,
    layer_sections={"fx": ("verse", "chorus", "verse2", "chorus2", "bridge", "chorus3")},
    genre_windows=_ROCK_GENRE_WINDOWS,
    role_level_db=_ROCK_CLASSIC_LEVEL_DB, duck_roles=(), section_lpf=_ROCK_SECTION_LPF,
    lpf_roles=("bass", "pad", "lead"), lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS,
    # Ширина: эталоны LR 0.76 (classic) / 0.87 (grunge) — гитара двумя голосами с расстройкой (дабл-трек) не шире
    # ±0.3, остальное в центре (удары файлами паков ширины не выражают).
    stereo={"pad": {"pan": 0.3, "detune": PAD_DETUNE}},
    a9_model_low=0.05,
    thin_roles={},
    drum_files={"clap": ("muldjord_snare_40", "muldjord_snare_43"),
                "hats": ("muldjord_hihatclosed_16", "muldjord_hihatclosed_18", "muldjord_rider_07"),
                "toms": ("muldjord_tom1_08", "muldjord_tom2_08", "muldjord_tom3_08", "muldjord_tom4_09")},
    backbeat_kinds=("intro_low", "build", "build2", "drop", "drop2", "break", "outro"),
    roll_bars=0, fill_joints=True, clap_fill="cut", tom_fills=_ROCK_TOM_FILLS, fx_roles=("crash",),
    power_rhythms=_ROCK_POWER_RHYTHMS, bass_riffs=_ROCK_BASS_RIFFS,
)

# ── Стиль ``jazz`` (ADR-0153 S6): head–solo–head, свинг ≈ 1.5–1.7, walking, comping, соло по сменам темы ──────────
#: Эталон джаз-кафе 07.10 (``style_reference_profiles.md``, 16 кГц, медиана [p10..p90] окон 20 с): низ < 150 Гц 0.27
#: [0.08..0.48], середина 0.73, crest 17.8, LR 0.85, LUFS −18, σ по минутам 1.6 дБ, свинг 1.54 [1.13..2.39], темп ≈ 96
#: [90..121], пики в 12-TET 0.90. Окна: ``swing`` 104–125 (свинг-квартет: comping, бочка «пёрышком» четвертями),
#: ``ballad`` 90–104 (медиана эталона 96: comping реже, пэд иногда держит аккорд). Свинг 0.20–0.25 доли восьмой: ratio
#: (1 + s)/(1 − s) = 1.50–1.67 — эталон 1.54, не триоль 2.0. Без сайдчейна; низ держит walking-бас контрабасового
#: регистра (28–50), бочка тихая.
_JAZZ_LOOKS: Tuple[Tuple[int, Look], ...] = (
    (7, Look(KICK_PATTERNS["four_on_floor"], 0.0)),
    (0, Look("X.......X.......", 0.0)),
)
_JAZZ_GENRE_WINDOWS: Mapping[str, GenreWindow] = {
    "swing": GenreWindow((104, 125), ("jazz", "soft", "acoustic"), _JAZZ_LOOKS, ("walking",), ("comping",)),
    "ballad": GenreWindow((90, 104), ("soft", "jazz"), _JAZZ_LOOKS, ("walking",), ("comping", "comping", "held")),
}
#: Райд «динь, динь-да-динь»: четверти + «и» второй и четвёртой доли (оффбит-восьмую качает свинг), акцент 2 и 4.
_JAZZ_KITS: Mapping[str, Mapping[str, str]] = {
    "ride": {"hats": "x...X.x.x...X.x.", "perc": "................"},
    "ride_skip": {"hats": "x...X.x.x...X...", "perc": "................"},
    "ride_push": {"hats": "x...X.x.x.x.X.x.", "perc": "................"},
}
#: Лид — читаемый темой (:data:`THEME_LEAD_OK`): ``brass`` (духовой вместо сакса: ``soprano``/``eoboe``/``flute`` не
#: замерены ``--clarity``), ``keys`` (фортепиано), ``pluck`` (гитара), ``arpy``. Бас — с низом на роботе ≥ 0.9
#: (``bass``/``jbass``, четвертями — контрабаса в прелоаде нет). Пэд comping — замеренные в роли пэда
#: (``rhpiano``/``epiano`` в рамке пэда не замерены — не взяты, хотя хвост 28/148 мс comping допускает:
#: :data:`LEAD_CLARITY`; ``epiano`` нечитаем как лид — тусклый, атака 50 мс). #3549: ``strings`` снят — на потолке
#: ``amp`` он −41.7 дБ, на 4.7 дБ ниже цели пэда: comping тонул; замер ``rhpiano``/``epiano``/``keys`` в рамке пэда
#: (``loudness_nrt_v2.sh --sweep pad …``, katana) — до него их в пэд не взять (нет строки модели громкости).
_JAZZ_TIMBRES: Mapping[str, Mapping[str, Tuple[str, ...]]] = {
    "dark": {"lead": ("brass", "keys"), "bass": ("jbass", "bass"), "pad": ("sinepad", "space")},
    "hard": {"lead": ("brass", "arpy", "keys"), "bass": ("jbass", "bass"), "pad": ("sinepad", "ambi")},
    "bright": {"lead": ("keys", "pluck", "brass"), "bass": ("bass", "jbass"), "pad": ("ambi", "sinepad")},
    "warm": {"lead": ("brass", "keys", "pluck"), "bass": ("bass", "jbass"), "pad": ("sinepad", "ambi")},
}
#: Петли ii–V–I и оборотов (I–vi–ii–V, iii–vi–ii–V): их переходы — априорный бонус Витерби под мелодию
#: (``HookHarmony.style_bonus``), а не шаблон поверх темы.
_JAZZ_PROGRESSIONS: Tuple[Tuple[int, ...], ...] = (
    (1, 4, 0, 0), (0, 5, 1, 4), (2, 5, 1, 4), (1, 4, 0, 5), (0, 3, 1, 4),
)
#: Comping мимо сильных долей 1 и 3 (там тема): «Чарльстон со второй доли» (2 и «и» 3), упреждения «и» 2 / «и» 4,
#: обращённый Чарльстон («и» 1 и 4), три удара.
_JAZZ_COMP_RHYTHMS: Tuple[Tuple[Tuple[int, int], ...], ...] = (
    ((4, 2), (10, 3)), ((6, 2), (14, 2)), ((2, 2), (12, 3)), ((6, 3), (12, 2), (14, 2)),
)
#: Фразы соло по 2 такта (шаги 16-х, длины): восьмые — бибоп-линия, синкопа с «и» первой, подхват со второй доли,
#: редкая «вопросом»; хвост каждой — пауза ≥ четверти.
_JAZZ_SOLO_PHRASES: Tuple[Tuple[Tuple[int, int], ...], ...] = (
    ((0, 2), (2, 2), (4, 2), (6, 2), (8, 2), (10, 2), (12, 2), (14, 2), (16, 4), (20, 6)),
    ((2, 2), (4, 2), (6, 4), (10, 2), (12, 2), (14, 2), (16, 2), (18, 2), (20, 8)),
    ((4, 2), (6, 2), (8, 2), (10, 2), (12, 2), (14, 2), (16, 2), (18, 2), (20, 2), (22, 2), (24, 4)),
    ((0, 4), (6, 2), (8, 4), (14, 6), (24, 2), (26, 4)),
)
_JAZZ_DRUMS = frozenset({"kick", "hats", "clap"})
_JAZZ_FULL = _JAZZ_DRUMS | {"bass", "pad", "lead"}
#: Форма head–solo–head (§3): интро 4+4 и хвост 4+4 — блэнд двух дек, как у клуба; голова — тема целиком
#: (:data:`THEME_SECTION`: секция растёт на длину темы), соло — 16 тактов по сменам темы с каденцией ii–V–I в конце,
#: вторая голова — хук в терциях. ``jazz64`` — два квадрата соло.
_JAZZ_INTRO: FormSpec = (
    ("intro", _CLUB_BLEND[1], 2, frozenset({"hats", "pad"})),
    ("intro_low", _CLUB_BLEND[0] - _CLUB_BLEND[1], 3, _JAZZ_DRUMS | {"bass", "pad"}),
)
_JAZZ_TAIL: FormSpec = (
    ("outro", 8 - (_CLUB_BLEND[0] - _CLUB_BLEND[1]), 3, _JAZZ_DRUMS | {"bass", "pad"}),
    ("outro_tail", _CLUB_BLEND[0] - _CLUB_BLEND[1], 1, frozenset({"hats", "pad"})),
)
_JAZZ_FORMS: Mapping[str, FormSpec] = {
    "jazz48": (*_JAZZ_INTRO, ("head", 8, 7, _JAZZ_FULL), ("solo", 16, 8, _JAZZ_FULL), ("head2", 8, 8, _JAZZ_FULL),
               *_JAZZ_TAIL),
    "jazz64": (*_JAZZ_INTRO, ("head", 8, 7, _JAZZ_FULL), ("solo", 16, 8, _JAZZ_FULL), ("solo2", 16, 9, _JAZZ_FULL),
               ("head2", 8, 8, _JAZZ_FULL), *_JAZZ_TAIL),
}
_JAZZ_ENERGY_FORMS: Mapping[int, Tuple[str, ...]] = {
    1: ("jazz48",), 2: ("jazz48",), 3: ("jazz48", "jazz64"), 4: ("jazz48", "jazz64"), 5: ("jazz64", "jazz48"),
}
#: Уровни ролей (шкала модели): лёгкий контрабас, бочка «пёрышком», comping и лид — середина, райд слышен. Приёмка S7
#: (робот 08.10, образ 3e4df37e5, трек 1 ``sinepad``/``keys``/``bass``, уровни −46/−37/−40/−46/−50/−50): низ 0.58,
#: середина 0.41 против эталона 0.27/0.73 — отношение низ/середина надо срезать на ≈ 5.8 дБ. Низ держит бас (на 9 дБ
#: громче бочки): бас −6, бочка −2; пэд +3 (``sinepad``/``ambi`` упираются в потолок ``amp`` ≈ −38.8 — фактически +1),
#: лид +2, райд +3 (атаки — crest). #3549; проверка — запись робота, не модель (A9-модель джаза не откалибрована).
_JAZZ_ROLE_LEVEL_DB: Mapping[str, float] = {
    "kick": -48.0, "bass": -43.0, "pad": -37.0, "lead": -44.0, "clap": -50.0, "hats": -47.0}
_JAZZ_WINDOW = _JAZZ_GENRE_WINDOWS["swing"]
_JAZZ_STYLE = Style(
    bpm=_JAZZ_WINDOW.bpm, swing=(0.20, 0.25), modes=("major", "dorian", "minor"),
    kick_pool=_JAZZ_WINDOW.kick_pool, looks=_JAZZ_LOOKS, kits=_JAZZ_KITS,
    registers={"bass": (28, 50), "pad": (46, 72), "lead": (60, 86)},
    timbres=_JAZZ_TIMBRES, default_timbre="warm", bass_figures=_JAZZ_WINDOW.bass_figures,
    pad_figures=_JAZZ_WINDOW.pad_figures, lead_figures=("motif",), chord_size=5, progressions=_JAZZ_PROGRESSIONS,
    forms=_JAZZ_FORMS, opening_form="jazz48", energy_forms=_JAZZ_ENERGY_FORMS, blend=_CLUB_BLEND,
    layer_sections={}, genre_windows=_JAZZ_GENRE_WINDOWS,
    role_level_db=_JAZZ_ROLE_LEVEL_DB, duck_roles=(), section_lpf={"outro_tail": (LPF_TOP_HZ, 300.0)},
    lpf_roles=("bass", "pad", "lead"), lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS,
    # Ширина: эталон LR 0.85 (робот S7 0.99). Comping — два голоса с расстройкой на ±0.6: корреляция голосов
    # sin((1 − 0.6)·π/2) ≈ 0.59 (при ±0.4 — 0.81); при доле пэда ≈ 0.4 мощности LR ≈ 0.6 + 0.4·0.59 ≈ 0.84. Бас, бочка
    # и лид — в центре; удары файлами паков (райд, педаль) ширины не выражают (``render.renardo``), а панорама моно
    # корреляцию не снижает.
    stereo={"pad": {"pan": 0.6, "detune": PAD_DETUNE}},
    # A9-модель для джаза не откалибрована (удары файлами и walking в модели — оценки): порог — страховка от трека
    # без низа, норма — эталон 0.27 [0.08..0.48] по записи. Модель занижает низ джаза: трек 1 приёмки S7 — модель 0.391,
    # запись 0.58 (отношение низ/середина в 2.2 раза ниже записи). С уровнями #3549 модель даёт 0.015–0.17, и порог
    # 0.05 глушил пэд на 2–8 дБ в 14 из 90 треков аудита — ровно против цели; 0.01 — только трек совсем без баса.
    a9_model_low=0.01,
    thin_roles={}, swing_steps=(2, 6, 10, 14), swing_roles=("kick", "clap", "bass", "pad", "lead"),
    approach_max_beats=1.0,
    # Райд — хэты (``muldjord_rider_*``, 2–8 кГц 0.51: на 16 кГц робота слышен), педаль хэта на 2 и 4 — роль клэпа.
    drum_files={"hats": ("muldjord_rider_07", "muldjord_rider_09"), "clap": ("sonicpi_drum_cymbal_pedal",)},
    comp_rhythms=_JAZZ_COMP_RHYTHMS,
    backbeat_kinds=("intro_low", "drop", "drop2", "solo", "outro"), roll_bars=0, clap_fill="cut",
    cadence=((1, ""), (4, "maj"), (0, "")), section_leads={"solo": "solo"},
    solo_phrases=_JAZZ_SOLO_PHRASES, solo_chromatic=0.4,
    # #3549: аудит S7 — м2/м9 лид–бас у 100 % треков, параллельные квинты/октавы у 62 %; walking обходит их сам.
    bass_lead_clash=(1, 11), bass_lead_parallels=(0, 7),
    # Динамика (#3549): crest робота 13.1 против 17.8 эталона — компрессор 4:1 с makeup 9 дБ давил атаки. Джазу
    # компрессор выключен (ratio 1, makeup 0), выравниватель тянет к −16 дБ: при crest ≈ 17 пик ≈ −1 dBFS — у потолка
    # лимитера, а не под ним. Громкость джаза ниже прочих стилей на ≈ 4–5 дБ — плата за динамику (эталон джаза тоже
    # тише рока на 5 LU).
    master={"lvlTarget": -16.0, "cmpRatio": 1.0, "makeup": 0.0},
)

#: Стили по ключу (ключ — ``ThemeProfile.style``/``SetPlan.style``). ``club`` — сегодняшние клубные таблицы побайтно
#: (``test_style_same_tracks``): 128–138 — решение Шифу 01.10 (ADR-0149 §12 В6, эталон живого диджея ~138); свинг
#: 5–10 % (ADR-0149 §3.4).
STYLES: Mapping[str, Style] = {
    "club": Style(
        bpm=(128, 138), swing=(0.05, 0.10), modes=("minor", "dorian", "phrygian", "major"),
        kick_pool=("house", "deep", "techno", "garage"), looks=_CLUB_LOOKS, kits=_CLUB_KITS,
        registers=_CLUB_REGISTERS, timbres=_CLUB_TIMBRES, default_timbre="warm",
        bass_figures=("offbeat", "rolling8", "acid16"), pad_figures=("pumped16", "held", "stabs"),
        lead_figures=("motif",),
        chord_size=3, progressions=_CLUB_PROGRESSIONS,
        forms=_CLUB_FORMS, opening_form=_CLUB_OPENING_FORM, energy_forms=_CLUB_ENERGY_FORMS,
        blend=_CLUB_BLEND, layer_sections=_CLUB_LAYER_SECTIONS, genre_windows=_CLUB_GENRE_WINDOWS,
        role_level_db=_CLUB_ROLE_LEVEL_DB, duck_roles=_CLUB_DUCK_ROLES, section_lpf=_CLUB_SECTION_LPF,
        lpf_roles=_CLUB_LPF_ROLES, lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS, stereo=_CLUB_STEREO,
        # A9′ (низ дропа на роботе ≥ 0.5) + запас в σ остатка модели (0.095 доли на дроп по 133 дропам, #3449):
        # при 0.55 ser7 терял ожидаемый LR дропов (0.64 → 0.56), при 0.6 LR ни в одном сете трёх серий не хуже
        a9_model_low=0.6,
    ),
    # ADR-0153 S1: поля стиля — первое окно (``rave``), как у клуба; всё окно стиля — 140–160 (три окна).
    # Свинг почти нулевой, лады — минор/фригийский (§3). Микс клубный: уровни ролей, сайдчейн, свип, стерео;
    # A9-модель — нижняя граница нормы рейва 0.6–0.85 (§2.1, гипотеза до эталона В1) = клубный порог.
    "rave": Style(
        bpm=(140, 150), swing=(0.0, 0.03), modes=("minor", "phrygian"),
        kick_pool=_RAVE_GENRE_WINDOWS["rave"].kick_pool, looks=_CLUB_LOOKS, kits=_RAVE_KITS,
        registers=_CLUB_REGISTERS, timbres=_RAVE_TIMBRES, default_timbre="hard",
        bass_figures=_RAVE_GENRE_WINDOWS["rave"].bass_figures, pad_figures=_RAVE_GENRE_WINDOWS["rave"].pad_figures,
        lead_figures=("motif",),
        chord_size=3, progressions=_CLUB_PROGRESSIONS,
        forms=_CLUB_FORMS, opening_form=_CLUB_OPENING_FORM, energy_forms=_CLUB_ENERGY_FORMS,
        blend=_CLUB_BLEND, layer_sections=_CLUB_LAYER_SECTIONS, genre_windows=_RAVE_GENRE_WINDOWS,
        role_level_db=_CLUB_ROLE_LEVEL_DB, duck_roles=_CLUB_DUCK_ROLES, section_lpf=_CLUB_SECTION_LPF,
        lpf_roles=_CLUB_LPF_ROLES, lpf_tail_sections=_CLUB_LPF_TAIL_SECTIONS, stereo=_CLUB_STEREO,
        a9_model_low=0.6,
    ),
    # ADR-0153 S2: поля стиля — первое окно, как у клуба и рейва.
    "synthwave": Style(
        bpm=_SYNTHWAVE_GENRE_WINDOWS["synthwave"].bpm, kick_pool=_SYNTHWAVE_GENRE_WINDOWS["synthwave"].kick_pool,
        bass_figures=_SYNTHWAVE_GENRE_WINDOWS["synthwave"].bass_figures,
        pad_figures=_SYNTHWAVE_GENRE_WINDOWS["synthwave"].pad_figures, **_SYNTHWAVE_FIELDS),
    "chiptune": Style(
        bpm=_CHIPTUNE_GENRE_WINDOWS["chiptune"].bpm, kick_pool=_CHIPTUNE_GENRE_WINDOWS["chiptune"].kick_pool,
        bass_figures=_CHIPTUNE_GENRE_WINDOWS["chiptune"].bass_figures,
        pad_figures=_CHIPTUNE_GENRE_WINDOWS["chiptune"].pad_figures, **_CHIPTUNE_FIELDS),
    # ADR-0153 S3: поля стиля — первое окно.
    "breaks": Style(
        bpm=_BREAKS_GENRE_WINDOWS["breakbeat"].bpm, swing=(0.0, 0.05),
        kick_pool=_BREAKS_GENRE_WINDOWS["breakbeat"].kick_pool,
        looks=_BREAKS_LOOKS, bass_figures=_BREAKS_GENRE_WINDOWS["breakbeat"].bass_figures,
        pad_figures=_BREAKS_GENRE_WINDOWS["breakbeat"].pad_figures, progressions=_CLUB_PROGRESSIONS,
        genre_windows=_BREAKS_GENRE_WINDOWS, **_BREAKBEAT_FIELDS),
    "dnb": Style(
        bpm=_DNB_GENRE_WINDOWS["dnb"].bpm, swing=(0.0, 0.03), kick_pool=_DNB_GENRE_WINDOWS["dnb"].kick_pool,
        looks=_DNB_LOOKS, bass_figures=_DNB_GENRE_WINDOWS["dnb"].bass_figures,
        pad_figures=_DNB_GENRE_WINDOWS["dnb"].pad_figures, progressions=_DNB_PROGRESSIONS,
        genre_windows=_DNB_GENRE_WINDOWS, **_BREAKBEAT_FIELDS),
    "lofi": _LOFI_STYLE,
    "rock": _ROCK_STYLE,
    "jazz": _JAZZ_STYLE,
}
DEFAULT_STYLE = "club"
#: Слова фразы человека → стиль (ADR-0153 §4.1): основа слова (начало) → ключ :data:`STYLES`. Одна таблица:
#: грамматика роутера и ``dj_set`` берут стиль отсюда (``theme.match_style``); слов нет — :data:`DEFAULT_STYLE`.
STYLE_WORDS: Mapping[str, str] = {
    "рейв": "rave", "рэйв": "rave", "rave": "rave", "эйсид": "rave", "эсид": "rave", "acid": "rave",
    "хардкор": "rave", "hardcore": "rave",
    # ADR-0153 S2. «8-бит», «8битный» (цифра) — :data:`STYLE_PATTERNS`; «ретро» без «вейв» — не стиль (стоп-слово темы)
    "синтвейв": "synthwave", "синтвэйв": "synthwave", "synthwave": "synthwave", "ретровейв": "synthwave",
    "ретровэйв": "synthwave", "retrowave": "synthwave", "аутран": "synthwave", "outrun": "synthwave",
    "чиптюн": "chiptune", "chiptune": "chiptune", "восьмибит": "chiptune",
    # ADR-0153 S3. «брейк» без «бит/с» — не стиль («брейкданс», «брейк» трека); «драм» один — не стиль («драма»):
    # «драм-н-бейс», «брейк бит» словами врозь — :data:`STYLE_PATTERNS`. «джунгли» (тема) — не «джангл».
    "брейкбит": "breaks", "брейкс": "breaks", "breakbeat": "breaks", "breaks": "breaks",
    "днб": "dnb", "dnb": "dnb", "драмнбейс": "dnb", "драмэнбейс": "dnb", "драмнбэйс": "dnb", "джангл": "dnb",
    "jungle": "dnb", "drumnbass": "dnb",
    # #3508: клуб — стиль по умолчанию, но тема со своей строкой (киберпанк → synthwave) без слова его не выберет.
    # Основы «клубны/клубна/клубно/клубну», не «клуб»: «клубника» не стиль. «хаус»/«house»/«техно» не берём: «техно» — основа
    # темы cyber, «хаус» — «Доктор Хаус»; окна club/deep/breaks (ADR-0152) стиль не выбирают — окно внутри club.
    "клубны": "club", "клубна": "club", "клубно": "club", "клубну": "club", "клубняк": "club", "club": "club",
    # ADR-0153 S4. «лоу-фай»/«lo fi» словами врозь — :data:`STYLE_PATTERNS`. «чилл» — начало «чиллаут»/«чилловый»;
    # «чили» (страна, перец) — одно «л», не стиль.
    "лоуфай": "lofi", "лофай": "lofi", "лофи": "lofi", "lofi": "lofi", "чилл": "lofi", "chill": "lofi",
    # ADR-0153 S5. «рок»/«rock» — НЕ основы: «роковой», «рокки», «рокот», «rocket» — не стиль; «рок», «рок-н-ролл»,
    # «хард-рок» — :data:`STYLE_PATTERNS` (слово целиком). «барокко» начинается не с «рок».
    "гранж": "rock", "grunge": "rock", "хардрок": "rock", "рокнролл": "rock", "рокенролл": "rock",
    # ADR-0153 S6. «джаз», «джазовый», «джазмен», «jazz»; «свинг» — стиль и окно ``swing`` (приёма «свинг» словом
    # человека в грамматике нет: свинг-ratio выбирает план); «бибоп».
    "джаз": "jazz", "jazz": "jazz", "свинг": "jazz", "swing": "jazz", "бибоп": "jazz", "bebop": "jazz",
}
#: Слова фразы человека → окно стиля (``Style.genre_windows``, ADR-0153 S5): основа → окно. Окно применяется, только
#: если оно есть у стиля сета (``theme.match_window_text``); слов нет — окно выбирает план (``set_plan.pick_genre``).
WINDOW_WORDS: Mapping[str, str] = {"гранж": "grunge", "grunge": "grunge",
                                   # ADR-0153 S6: окна джаза; у стиля без этих окон слово окна не выбирает
                                   "свинг": "swing", "swing": "swing", "баллад": "ballad", "ballad": "ballad"}
#: Окно по умолчанию (первое окно стиля) — то, чем собраны поля ``Style``.
DEFAULT_GENRE = next(iter(STYLES[DEFAULT_STYLE].genre_windows))


def genre_style(style: Style, genre: str = DEFAULT_GENRE) -> Style:
    """Стиль сета в окне ``genre``: темп, пул бочек, виды секций (рисунок бочки), пул рисунков пэда — из окна.
    Единственная точка подстановки: ``arrange/*`` читают поля ``Style`` и о жанре не знают."""
    window = style.genre_windows[genre]
    return replace(style, bpm=window.bpm, kick_pool=window.kick_pool, looks=window.looks,
                   bass_figures=window.bass_figures, pad_figures=window.pad_figures,
                   role_level_db=style.role_level_db if window.role_level_db is None else window.role_level_db,
                   a9_model_low=style.a9_model_low if window.a9_model_low is None else window.a9_model_low)


#: Коридоры регистров стиля по умолчанию (умолчание хука ``arrange.hook``) и объединение ролей сайдчейна всех
#: стилей. Валидатор модели берёт регистры и сайдчейн из стиля трека (``Track.style``, ADR-0153 S1).
REGISTERS: Mapping[str, Tuple[int, int]] = STYLES[DEFAULT_STYLE].registers
DUCK_ROLES: Tuple[str, ...] = tuple(dict.fromkeys(r for st in STYLES.values() for r in st.duck_roles))

#: Все шаблоны форм всех стилей (валидатор ``model.validate``: ``HistoryKey.template`` — известная форма).
FORM_NAMES: frozenset = frozenset(name for st in STYLES.values() for name in st.forms)
#: Все жанровые окна всех стилей (валидатор: ``HistoryKey.genre`` — известное окно).
GENRE_NAMES: frozenset = frozenset(name for st in STYLES.values() for name in st.genre_windows)

__all__ += ["DEFAULT_GENRE", "DEFAULT_STYLE", "DUCK_ROLES", "FORM_NAMES", "GENRE_NAMES", "GenreWindow",
            "PAD_WIDEN", "REGISTERS", "STYLES", "STYLE_WORDS", "Style", "genre_style", "THEME_SECTION",
            "THEME_MAX_BARS", "THEME_REF_NOTES", "THEME_REF_MATCH_MIN", "FORM_BARS_STEP", "TRACK_MAX_BARS",
            "SECTION_KINDS", "section_kind", "WINDOW_WORDS", "CHANGES", "TENSION_AVOID"]


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
#: Песня из материала партитуры (ADR-0154 PR-7B, «сыграй <произведение>»): куплеты — секции материала, без меток —
#: фразы подряд, пока куплет не наберёт столько тактов; аккомпанемент — аккорды автора арпеджио (фактура ``arp``,
#: порядок голосов ``Style.arp_order``) и басовый голос автора; рисунок ударных (символы :data:`SONG_DRUM_SYMBOLS`)
#: и темп материала без метки — отсюда.
SONG_SCORE_VERSE_BARS = 8
SONG_SCORE_DRUMS = "X...o...X...o..."
SONG_SCORE_HATS = "-.-.-.-.-.-.-.-."
SONG_SCORE_BPM = 100
#: Коридор арпеджио пэда песни из материала: внутри ``SONG_REGISTERS["pad"]``, под мелодией (архив: лид p1 56).
SONG_SCORE_PAD: Tuple[int, int] = (55, 76)

__all__ += [
    "FORM_CLUB", "FORM_SONG", "SONG_DRUM_SYMBOLS", "SONG_KEY_FIT_MIN", "SONG_MAX_BARS", "SONG_REGISTERS",
    "SONG_SCORE_BPM", "SONG_SCORE_DRUMS", "SONG_SCORE_PAD", "SONG_SCORE_HATS", "SONG_SCORE_VERSE_BARS", "SONG_TARGET_BARS",
    "SONG_TIMBRES", "SONG_VERSES",
]


# ── Реестр произведений (ADR-0155 K-2): ручные таблицы знания; логика — ``rob_box_music.works`` ───────────────
#: «Исполнитель» архива, который на деле категория (``artist`` = «Films And Tv», «Computer Games»): значение —
#: тип произведения (``Work.work_type``) или ``""`` — категория без типа («Unknown», «Various»); артиста у такой
#: записи нет. Для «Films And Tv»/«Theme» тип берётся из тегов записи (:data:`TAG_WORK_TYPE`).
CATEGORY_ARTISTS: Mapping[str, str] = {
    "": "", "unknown": "", "various": "", "various artists": "", "in development": "", "wavi": "",
    "films and tv": "", "film theme": "film_theme", "theme": "", "tv": "tv_theme", "movie": "film_theme",
    "movies": "film_theme", "soundtrack": "film_theme", "bollywood": "film_theme", "computer games": "game_theme",
    "game": "game_theme", "games": "game_theme", "anthem": "anthem", "national anthem": "anthem",
    "christmas": "christmas", "classical": "classical", "traditional": "folk",
}
#: Тег каталога RTTTL → тип произведения (когда категория-«исполнитель» типа не даёт).
TAG_WORK_TYPE: Mapping[str, str] = {
    "tv": "tv_theme", "movie": "film_theme", "game": "game_theme", "anthem": "anthem", "christmas": "christmas",
    "classical": "classical", "folk": "folk",
}
#: Название записи без названия: слово «Theme»/«Unknown» — настоящее название стоит в ``artist``.
EMPTY_TITLES: Tuple[str, ...] = ("", "theme", "unknown", "untitled")
#: Стоп-список (ADR-0155 В1, «лёгкий (б)»): композиторы и студии с живыми правами. Всё, что едет из PDMX в git и
#: на публичный показ, с таким именем в ``composer``/``artist`` не берётся; PDMX остаётся только на роботе/katana.
#: Строки — подстроки нормализованного имени (``works.norm``: буквы и цифры без пробелов). Растёт по факту.
LICENSE_STOP_LIST: Tuple[str, ...] = ("zimmer", "kondo", "uematsu", "nintendo", "disney", "johnwilliams")

__all__ += ["CATEGORY_ARTISTS", "EMPTY_TITLES", "LICENSE_STOP_LIST", "TAG_WORK_TYPE"]
