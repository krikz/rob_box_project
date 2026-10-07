"""media_command_grammar.py — закрытая грамматика медиакоманд (issue #3134, Ш2).

Одна грамматика для команд, которые исполняет КОД, а не LLM
(``docs/design/2026-09-28-music-dj-systemic-analysis.md`` §6 Ш2):

* громкость музыки — «громче / тише / потише / погромче / на максимум»,
  в том числе с названием играющего трека («горный король погромче»);
* стоп — «выключи музыку», «стоп диджей», «хватит диджеить»;
* DJ — «ты диджей X», «запусти диджей-сет»; длина сета — «сет на 3 трека», «на полчаса» (``tracks``, разбор —
  :mod:`.set_length_words`), тема-перечисление после двоеточия — «сет на 3 трека: Марио, Тетрис, Зельда»;
* заказ по имени — «поставь / сыграй / включи <название>» (issue #3176);
* заказ «какой-нибудь» музыки — «поставь клубный трек» (``REQUEST_MUSIC``, ADR-0149
  PR-6: его исполняет ``request_music`` движка v2).

Раньше эти же правила жили внутри гуардов ПОСЛЕ ответа LLM
(``music_volume_request.py`` — #3125, ``dj_request.is_dj_request`` — #2999,
``dialogue_guards.is_music_stop_command`` — #2834/#2971) и решали, ретраить
ли модель. MiniMax игнорирует ``tool_choice`` (ADR-0143), заставить модель
вызвать тул нельзя — поэтому грамматика перенесена на вход, а те гуарды,
которые она закрывает, удалены. Регексы ПЕРЕНЕСЕНЫ, не добавлены
(мораторий #3132, ``test_issue_3132_music_regex_moratorium.py``).

Модуль чистый: без ROS, без I/O. Решение «что делать при таком состоянии
плеера» — в :mod:`rob_box_voice.core.media_router`.
"""

from __future__ import annotations

import itertools
import re
from dataclasses import dataclass, replace
from enum import Enum
from typing import FrozenSet, List, Mapping, Optional, Sequence, Tuple

from rob_box_music.dj_line import persona_title
from rob_box_music.theme import match_style

from .set_length_words import LENGTH_WORDS, split_set_length


class MediaIntent(str, Enum):
    """Закрытый набор интентов роутера (варианты ``ChoiceQuestion``)."""

    NONE = "none"
    VOLUME_UP = "volume_up"
    VOLUME_DOWN = "volume_down"
    VOLUME_MAX = "volume_max"
    STOP = "stop"
    DJ = "dj"
    PLAY_NAMED = "play_named"
    REQUEST_MUSIC = "request_music"
    NOW_PLAYING = "now_playing"


#: Порядок вариантов для ``ChoiceQuestion`` (DecisionProvider).
MEDIA_INTENT_OPTIONS: Tuple[str, ...] = tuple(i.value for i in MediaIntent)


@dataclass(frozen=True)
class MediaCommand:
    """Разобранная реплика.

    Attributes:
        intent: что просит юзер.
        closed: реплика ЦЕЛИКОМ покрыта грамматикой — LLM не нужна. Для
            ``DJ`` бывает ``False``: персона назначена, но в реплике есть
            ещё что-то (план, конкретные треки) — это решает LLM.
        persona: для ``DJ`` — «диджей X» или ``""``. Тему из свободных слов реплики («у нас сегодня …»,
            «вечеринка любителей X, замути сэт») грамматика НЕ выделяет: её выделяет ``dj_set`` из скрытого
            ``heard_text`` одной функцией для пути команды и пути LLM
            (``rob_box_mcp_tools.engine.theme_grounding.heard_theme``, 07.10).
        name: для ``PLAY_NAMED`` — название мелодии словами юзера
            («к элизе»), без глагола и служебных слов; для ``REQUEST_MUSIC`` —
            родовые слова заказа («клубный трек»).
        set_theme: для ``DJ`` — тема из хвоста «… на тему X» / «… про X»
            (ADR-0149 PR-6), непусто только если до неё реплика — закрытая
            DJ-команда. ``closed``/``persona`` от неё не меняются, роутер
            считает такую реплику закрытой.
        set_persona: персона из головы реплики до темы.
        mood: для ``REQUEST_MUSIC`` из «музыку для танцев» — ключ
            ``knowledge.MOOD_ENERGY`` или ``""`` (дефолт решает движок).
        themed: для ``PLAY_NAMED`` — человек сказал слово «тема» («давай тему
            терминатора»): посреди DJ-сета это смена темы сета, не мелодия (#3410).
        style: для ``DJ`` — ключ ``knowledge.STYLES`` из слов стиля («рейв»,
            «эйсид», «в стиле хардкор», ADR-0153 S1) или ``""`` (решает тул).
            Слова стиля в тему не попадают.
        tracks: для ``DJ`` — длина сета, названная человеком («на 3 трека», «три
            десятка», «на полчаса»), ``0`` — не названа (длину решает тул).
    """

    intent: MediaIntent
    closed: bool = True
    persona: str = ""
    name: str = ""
    set_theme: str = ""
    set_persona: str = ""
    mood: str = ""
    themed: bool = False
    style: str = ""
    tracks: int = 0


NO_COMMAND = MediaCommand(intent=MediaIntent.NONE, closed=False)


# ---------------------------------------------------------------------------
# Токенизация (перенесено из music_volume_request.py, #3125)
# ---------------------------------------------------------------------------

#: Ведущие теги источника/контекста: ``[TG]``, ``[Speaker:unknown]``,
#: ``[🎧 …]``. Слова внутри тегов — не слова юзера.
_LEADING_TAGS_RE = re.compile(r"^\s*(?:\[[^\]\n]*\]\s*)+")

_WORD_RE = re.compile(r"[a-zа-яё]+", re.IGNORECASE)


def extract_user_utterance(user_input: str) -> str:
    """Слова юзера без ведущих тегов и без хвостовых служебных блоков."""
    if not user_input:
        return ""
    text = _LEADING_TAGS_RE.sub("", user_input)
    return text.split("\n", 1)[0].strip()


def _words(text: str) -> List[str]:
    return [w.lower() for w in _WORD_RE.findall(text or "")]


def _theme_words(text: str) -> List[str]:
    """Слова темы сета: цифры — отдельные слова, на границе букв и цифр слово делится: «Mozart40» -> «mozart 40»,
    «Mambo Nr 5» -> «mambo nr 5» (#3460: ``_words`` вырезал цифры, тема приходила как «mozart»/«mambo nr»).
    Без регексов (мораторий #3132): буквы — как в ``_WORD_RE`` (латиница, кириллица), цифры — ``0-9``."""
    def kind(ch: str) -> str:
        low = ch.lower()
        return "d" if ch in "0123456789" else "a" if "a" <= low <= "z" or "а" <= low <= "я" or low == "ё" else ""

    return ["".join(g).lower() for k, g in itertools.groupby(text or "", kind) if k]


# ---------------------------------------------------------------------------
# Стоп (перенесено из dialogue_guards.py, #2834 / #2971)
# ---------------------------------------------------------------------------

#: Устоявшиеся фиксированные стоп-фразы (подстроки). Голые существительные
#: без стоп-глагола («диджея») сюда НЕ входят — #2971.
MUSIC_STOP_OVERRIDES: tuple = (
    "выключи музыку",
    "выключ музыку",
    "музыку выключ",
    "стоп музык",
    "останови музык",
    "убери музык",
)

#: Issue #3169 — только ПОВЕЛИТЕЛЬНАЯ форма (приказ), не прошедшее время.
#: Живой прогон 29.09: «робот ты остановил музыку» — вопрос в прошедшем
#: времени («останов\w*» стемминг ловил и «остановил») — роутер исполнял
#: его как приказ стоп. «\w*»-стемминг у глаголов убран, у каждого —
#: явный список повелительных окончаний (ед./мн. число, «-сь» у
#: возвратных). Мораторий #3132 — паттерн ПРАВЛЮ существующий, не
#: добавляю новый ``re.compile``.
_MUSIC_STOP_VERBS: str = (
    r"стоп|хватит|"
    r"выключ(?:и|ите|ай|айте)|"
    r"останов(?:и(?:сь)?|ите(?:сь)?)|"
    r"убер(?:и|ите)|"
    r"заглуш(?:и|ите)|"
    r"заверш(?:и|ите)"
)
_MUSIC_STOP_NOUNS: str = r"музык\w*|дидж\w*|трек\w*|сет\b"

#: «стоп-глагол + музыкальное существительное» в любом порядке (#2834).
#: Issue #3169 — правая ветка (сущ. → глагол) раньше добавляла ``\w*``
#: ПОСЛЕ группы глагола, что возвращало стемминг именно там, где первая
#: ветка его уже убрала («музыку остановил» матчился, «на всякий случай
#: всё остановил» — нет, т.к. без «музык…» рядом). Обе ветки теперь
#: заканчиваются строгой границей слова сразу после глагольной группы.
MUSIC_STOP_COMMAND_RE = re.compile(
    rf"\b(?:{_MUSIC_STOP_VERBS})\b.{{0,20}}?\b(?:{_MUSIC_STOP_NOUNS})"
    rf"|\b(?:{_MUSIC_STOP_NOUNS})\b.{{0,20}}?\b(?:{_MUSIC_STOP_VERBS})\b",
    re.IGNORECASE,
)


def is_music_stop_command(user_input: str) -> bool:
    """В реплике есть стоп музыки/DJ («хватит диджеить», «выключи музыку»).

    Широкий детектор (подстрока/поиск): им пользуются силенс-гейт,
    command-гейт и пост-гуард Bug F. Роутер исполняет стоп сам только для
    ЗАКРЫТОЙ реплики (:func:`parse_media_command`).
    """
    if not user_input:
        return False
    low = user_input.lower()
    if any(kw in low for kw in MUSIC_STOP_OVERRIDES):
        return True
    return bool(MUSIC_STOP_COMMAND_RE.search(low))


# ---------------------------------------------------------------------------
# Классы слов. Один регекс вместо ``_VOLUME_CORE_RE`` из #3125: он же
# раскладывает стоп-глаголы и музыкальные существительные.
# ---------------------------------------------------------------------------

_WORD_CLASS_RE = re.compile(
    r"^(?:(?P<up>(?:по)?[гк]ромче|прибав\w*|подкрут\w*|выкрут\w*)"
    r"|(?P<down>(?:по)?тише|убав\w*|приглуш\w*)"
    r"|(?P<max>максимум\w*|максимальн\w*|полную)"
    r"|(?P<topic>громк\w*)"
    rf"|(?P<stop>{_MUSIC_STOP_VERBS})"
    r"|(?P<noun>музык\w*|музон\w*|дидж\w*|трек\w*|трей|с[еэ]т|бит|бита|звук\w*|dj))$"
)

_DIRECTIONS: FrozenSet[str] = frozenset({"up", "down", "max"})


def _word_class(word: str) -> Optional[str]:
    m = _WORD_CLASS_RE.match(word)
    return m.lastgroup if m else None


#: Служебные слова, не меняющие смысла команды.
_COMMON_FILLER: FrozenSet[str] = frozenset({
    "ну", "а", "и", "но", "же", "ка", "эй", "йо", "пожалуйста", "плиз",
    "можно", "давай", "сделай", "сделайте", "это", "так", "вот", "уже",
    "ещё", "еще", "сейчас", "там", "тут", "мне", "нам", "её", "ее", "его",
    "эту", "этот", "всё", "все", "всю", "ты", "вы", "чуть", "чуточку",
    "немного", "очень", "на", "в", "по", "раз", "два", "раза", "чтобы",
    "чтоб", "режим", "режима", "совсем",
})

#: Сверх общих — слова просьбы о громкости («играй громче», #3125).
_VOLUME_FILLER: FrozenSet[str] = _COMMON_FILLER | frozenset({
    "играй", "сыграй", "играйте", "включи", "поставь", "сильнее", "больше",
    "меньше", "побольше", "поменьше", "тихо", "слишком",
})

#: Глаголы заказа трека: «сыграй <название> погромче» — это заказ, не громкость.
_PLAY_VERBS: FrozenSet[str] = frozenset({
    "играй", "сыграй", "играйте", "включи", "поставь", "запусти", "вруби",
    "врубай",
})

#: Просьба о ГОЛОСЕ робота («говори громче») — это ``set_volume``, решает LLM.
_VOICE_WORDS: FrozenSet[str] = frozenset({
    "говори", "говорить", "голос", "голоса", "голосом", "разговаривай",
})

#: Сколько слов может занимать название трека в «<трек> погромче».
_MAX_TRACK_NAME_WORDS = 5


def _drop_voice_negation(words: Sequence[str]) -> List[str]:
    """«играй кромче а НЕ ГОВОРИ громче» (живой лог #3125): «не говори» —
    отказ от голоса, а не просьба о нём."""
    out: List[str] = []
    skip = False
    for i, word in enumerate(words):
        if skip:
            skip = False
            continue
        if word == "не" and i + 1 < len(words) and words[i + 1] in _VOICE_WORDS:
            skip = True
            continue
        out.append(word)
    return out


def _volume_direction(classes: Sequence[Optional[str]]) -> Optional[MediaIntent]:
    dirs = {c for c in classes if c in _DIRECTIONS}
    if "max" in dirs and "down" not in dirs:
        return MediaIntent.VOLUME_MAX
    if dirs == {"up"}:
        return MediaIntent.VOLUME_UP
    if dirs == {"down"}:
        return MediaIntent.VOLUME_DOWN
    return None


def _names_current_track(extras: Sequence[str], track_name: Optional[str]) -> bool:
    """Лишние слова — это название играющего трека («горный король» ↔
    «В пещере горного короля»): каждое слово совпадает по основе (первые
    4 буквы) с каким-то словом названия."""
    if not track_name or len(extras) > _MAX_TRACK_NAME_WORDS:
        return False
    title = _words(track_name)
    return all(any(t.startswith(w[:4]) for t in title) for w in extras)


def _volume_command(
    words: Sequence[str], track_name: Optional[str]
) -> MediaCommand:
    words = _drop_voice_negation(words)
    if "не" in words or any(w in _VOICE_WORDS for w in words):
        return NO_COMMAND
    classes = [_word_class(w) for w in words]
    if "stop" in classes:
        return NO_COMMAND
    intent = _volume_direction(classes)
    if intent is None:
        return NO_COMMAND
    extras = [
        w for w, c in zip(words, classes) if c is None and w not in _VOLUME_FILLER
    ]
    if extras and (
        any(w in _PLAY_VERBS for w in words)
        or not _names_current_track(extras, track_name)
    ):
        # «сыграй в пещере горного короля погромче» — заказ трека;
        # «расскажи анекдот погромче» — не про музыку. Решает LLM.
        return NO_COMMAND
    return MediaCommand(intent=intent)


def _stop_command(words: Sequence[str]) -> MediaCommand:
    """Закрытый стоп: только стоп-глагол + музыкальное существительное +
    служебные слова. «выключи музыку и включи X» — не закрытый, это LLM."""
    classes = [_word_class(w) for w in words]
    if "stop" not in classes or "noun" not in classes:
        return NO_COMMAND
    for word, cls in zip(words, classes):
        if cls in ("stop", "noun") or word in _COMMON_FILLER:
            continue
        return NO_COMMAND
    return MediaCommand(intent=MediaIntent.STOP)


# ---------------------------------------------------------------------------
# DJ (перенесено из dj_request.py, #2999 / ADR-0140)
# ---------------------------------------------------------------------------

#: «диджей», «диджеем», «ди-джей», «диджэй», латиница «dj». Глагол
#: «диджеить» сюда НЕ входит — «хватит диджеить» это стоп.
_DJ_WORD = r"(?:ди-?дж[еэ](?:й|ем)|dj)"

#: Назначение персоны: «ты / будь / стань … диджей(ем) …». После слова —
#: НЕ запятая/вопрос/восклицание: «ты диджей?», «ты диджей, сделай громче».
_PERSONA_RE = re.compile(
    r"(?<![\w-])(?:ты|будь|будешь|стань|станешь|побудь)\s+"
    r"(?:(?:теперь|сегодня|снова|опять|у\s+нас)\s+)?"
    + _DJ_WORD
    + r"(?![\w-])(?!\s*[,?!])",
    re.IGNORECASE,
)

#: Запуск сета без персоны: «запусти диджей-сет», «включи режим диджея».
_SET_START_RE = re.compile(
    r"(?<![\w-])(?:запусти|запускай|включи|включай|вруби|врубай|начни|"
    r"начинай|давай|устрой|замути)(?![\w-])[^.!?]{0,24}?"
    r"(?:ди-?дж[еэ]й[\s-]*с[еэ]т|dj[\s-]*с[еэ]т|dj[\s-]*set|диджейск\w*\s+с[еэ]т|"
    r"режим\w*\s+(?:ди-?дж[еэ]я|dj)|ди-?дж[еэ]й[\s-]*режим|dj[\s-]*режим|"
    r"dj[\s-]*mode)",
    re.IGNORECASE,
)

#: Имя персоны сразу после назначения: до « и », знака препинания или конца.
_PERSONA_NAME_RE = re.compile(
    r"\s+((?:[^\s,.!?]+\s*){1,4}?)(?=\s+и\s|\s*[,.!?]|\s*$)",
    re.IGNORECASE,
)

#: «у нас (сегодня|тут) …» — слова темы: их отрезок не делает реплику «не закрытой», саму тему выделяет ``dj_set``.
_THEME_RE = re.compile(
    r"у\s+нас\s+(?:сегодня\s+|тут\s+|здесь\s+)?([^.!?\n]{3,80})",
    re.IGNORECASE,
)

#: Слова, которые не выводят DJ-реплику за пределы закрытой грамматики.
_DJ_FILLER: FrozenSet[str] = _COMMON_FILLER | frozenset({
    "сет", "диджей", "dj", "set", "mode", "поехали", "погнали", "запускай",
    "запусти", "включай", "включи", "начинай", "начни", "врубай", "вруби",
    "играй", "зажигай", "качай", "устрой", "замути", "у", "нас", "сегодня",
    "теперь", "снова", "опять", "будь", "стань", "побудь", "диджеем",
    "диджейский", "диджея", "сэт", "сета", "сэта", "сетик", "сэтик",
})


def is_dj_request(user_input: Optional[str]) -> bool:
    """Юзер назначает DJ-персону или просит запустить DJ-сет.

    Узкий по построению: «диджей, сделай громче», «стоп диджей», «хватит
    диджеить», «выключи режим диджея», «кто такой диджей?» — НЕ DJ-запрос.
    """
    if not user_input:
        return False
    low = user_input.lower()
    return bool(_PERSONA_RE.search(low) or _SET_START_RE.search(low))


def _persona_span(text: str) -> Tuple[str, Optional[Tuple[int, int]]]:
    """Имя персоны (без хвостовых служебных слов) и занятый им отрезок."""
    assign = _PERSONA_RE.search(text)
    m = _PERSONA_NAME_RE.match(text, assign.end()) if assign else None
    if not m:
        return "", None
    name_words = m.group(1).split()
    # Без запятых (голос через STT): «ты диджей Снупдог давай сет» —
    # «давай сет» не часть имени.
    while name_words and name_words[-1].lower() in _DJ_FILLER:
        name_words.pop()
    if not name_words or name_words[0].lower() in ("и", "у"):
        return "", (assign.start(), assign.end())
    return " ".join(name_words), (assign.start(), m.end())


def extract_dj_persona(user_input: Optional[str]) -> str:
    """Персона из DJ-запроса с префиксом «диджей» («диджей Снупдог»); ``""`` — не нашлась."""
    name, _span = _persona_span((user_input or "").strip())
    return persona_title(name)


def dj_persona_span(text: str) -> Optional[Tuple[int, int]]:
    """Отрезок «ты диджей X» в реплике (назначение персоны с именем) или ``None``. Имя не заходит на ввод темы
    («ты диджей Робокс на тему космос» — имя «Робокс»). Его вырезает из темы ``theme_grounding.heard_theme``."""
    head, _theme = _theme_split(text)
    spoken = _THEME_RE.search(head)
    head = head[:spoken.start()] if spoken else head  # «ты диджей Снупдог у нас сегодня …» — имя до «у нас»
    name, span = _persona_span(head)
    assign = _PERSONA_RE.search(head)
    if name or not assign:
        return span
    rest = head[assign.end():]  # имя без границы («ты диджей вася вечеринка у меня») — одно слово после «диджей»
    word = next(iter(rest.split()), "")
    if not word or word.lower() in ("и", "у"):
        return span
    return assign.start(), assign.end() + rest.find(word) + len(word)


def _cut(text: str, spans: Sequence[Optional[Tuple[int, int]]]) -> str:
    out = text
    for span in sorted((s for s in spans if s), reverse=True):
        out = out[: span[0]] + " " + out[span[1]:]
    return out


#: Слова, после которых в DJ-реплике идёт тема сета: «… на тему космос»,
#: «… про космос» (ADR-0149 PR-6). Слова, не регексы (мораторий #3132).
_THEME_LEADS: FrozenSet[str] = frozenset({
    "тему", "тема", "темой", "тематику", "тематика", "тематикой", "про",
})


def _theme_split(text: str) -> Tuple[str, str]:
    """``(голова, тема)`` по первому вводу темы: «ты диджей X на тему космос» → («ты диджей X», «космос»)."""
    low = text.lower()
    found = [
        (i, i + len(pat))
        for lead in _THEME_LEADS
        for pat in (f" на {lead} ", f" {lead} ")
        for i in (low.find(pat),)
        if i >= 0
    ]
    if not found:
        return text, ""
    start, end = min(found)
    return text[:start], " ".join(_theme_words(text[end:]))


#: Слова перед названием стиля: «в стиле рейв» (ADR-0153 S1). Вместе со
#: словом стиля из темы и из «закрытости» реплики выпадают.
_STYLE_LEAD: FrozenSet[str] = frozenset({"в", "стиле", "стиль", "стилем"})


def _is_style_word(word: str) -> bool:
    return match_style([word]) is not None


def _style_words(words: Sequence[str]) -> Tuple[str, List[str]]:
    """``(стиль, слова без «в стиле рейв»)``; стиля нет — ``("", слова)``."""
    style = match_style(words) or ""
    out: List[str] = []
    for word in words:
        if style and _is_style_word(word):
            while out and out[-1] in _STYLE_LEAD:
                out.pop()
            continue
        out.append(word)
    return style, out


def _with_style(command: MediaCommand, text: str) -> MediaCommand:
    """DJ-команда со стилем из слов реплики; слова стиля убраны из темы."""
    style, _rest = _style_words(_words(text))
    if not style:
        return command
    theme = command.set_theme
    return replace(command, style=style, set_theme=" ".join(_style_words(_theme_words(theme))[1]) if theme else theme)


def _dj_command(text: str) -> MediaCommand:
    """DJ-команда со стилем (:func:`_with_style`); заказ сета с темой словами человека — закрыт
    (:func:`is_spoken_theme_set`)."""
    command = _dj_theme_command(text)
    if not (command.closed or command.set_theme) and is_spoken_theme_set(text):
        command = replace(command, closed=True)
    return _with_style(command, text)


def _dj_theme_command(text: str) -> MediaCommand:
    """DJ-команда: персона/тема/закрытость + ``set_theme``/``set_persona`` из «… на тему X»."""
    command = _dj_head_command(text)
    if _THEME_RE.search(text):  # «у нас сегодня …» — тема словами реплики, её выделяет dj_set
        return command
    head, theme = _theme_split(text)
    if not theme or not is_dj_request(head):
        return command
    head_command = _dj_head_command(head)
    if not head_command.closed:
        return command
    return replace(command, set_theme=theme, set_persona=head_command.persona)


def _dj_head_command(text: str) -> MediaCommand:
    """Персона и закрытость: реплика без «ты диджей X», «у нас …» и запуска сета — одни служебные слова и стиль."""
    name, persona_span = _persona_span(text)
    theme_m = _THEME_RE.search(text)
    start_m = _SET_START_RE.search(text)
    rest = _cut(text, [
        persona_span,
        theme_m.span() if theme_m else None,
        start_m.span() if start_m else None,
    ])
    closed = all(w in _DJ_FILLER or w in _STYLE_LEAD or _is_style_word(w) for w in _words(rest))
    return MediaCommand(intent=MediaIntent.DJ, closed=closed, persona=persona_title(name))


# ---------------------------------------------------------------------------
# Заказ по имени (issue #3176)
# ---------------------------------------------------------------------------
# Живой прогон 29.09: «поставь к Элизе» → модель ответила «Ставлю «К Элизе»,
# погнали» с ``tools=[]`` — заказ потерян, обещание соврало. Заставить
# MiniMax вызвать тул нельзя (ADR-0143), поэтому заказ по имени разбирает
# код. Разбор — по словам и наборам слов, без регексов (мораторий #3132):
# слова даёт ``_words`` выше, дальше только ``frozenset``.
#
# Грамматика лишь ПРЕДЛАГАЕТ название. Есть ли такая мелодия, решает база
# мелодий (``lookup_melody`` в ноде): не нашлась или совпала не целиком —
# реплика уходит в LLM, как раньше.

#: Повелительные глаголы заказа — первое значимое слово реплики.
_PLAY_NAMED_VERBS: FrozenSet[str] = frozenset({
    "поставь", "поставьте", "поставишь", "сыграй", "сыграйте", "сыграешь",
    "играй", "включи", "включите", "включишь", "давай", "давайте",
    "врубай", "врубайте", "вруби", "врубите", "заведи", "заведите",
    "заводи",
    # вежливые формы: «можешь поставить …», «не могли бы вы сыграть …»
    "поставить", "сыграть", "включить", "врубить", "завести",
})

#: Слова перед глаголом, не меняющие смысла заказа.
_PLAY_NAMED_LEAD_IN: FrozenSet[str] = frozenset({
    "ну", "а", "эй", "йо", "слушай", "робот", "пожалуйста", "плиз", "можешь",
    "можете", "могли", "мог", "могла", "бы", "ты", "вы", "будь", "добр",
    "добра", "добры", "будьте", "ка", "давай", "давайте",
})

#: Служебные слова между глаголом и названием и после названия.
_PLAY_NAMED_FILLER: FrozenSet[str] = frozenset({
    "ка", "мне", "нам", "пожалуйста", "плиз", "сейчас", "уже", "ну",
    "давай", "давайте", "поставь", "сыграй", "включи", "вруби",
})

#: Родовые слова «что играть»: песня/трек/жанр/настроение. Название из
#: одних таких слов — не название: «поставь музыку», «сыграй что-нибудь
#: весёлое», «включи клубный трек» идут в LLM (или в DJ-превью), как раньше.
#: Ведущие родовые слова перед названием срезаются: «поставь песню к элизе».
_GENERIC_WORDS: FrozenSet[str] = frozenset({
    "музыку", "музыка", "музычку", "музон", "музончик", "трек", "трэк",
    "треки", "бит", "биток", "биты", "песню", "песенку", "песни",
    "песня", "мелодию", "мелодийку", "мелодия", "композицию", "мотив",
    "тему", "тема", "темы", "микс", "сет", "плейлист", "радио", "звук", "что", "то",
    "нибудь", "либо", "чего", "какую", "какой", "какое", "какие", "любую",
    "любое", "любой", "свою", "своё", "свое", "мою", "нашу", "новую",
    "новое", "новый", "старую", "старое", "весёлое", "веселое", "весёлую",
    "веселую", "весёлый", "веселый", "грустное", "грустную", "спокойное",
    "спокойную", "бодрое", "бодрую", "энергичное", "энергичную",
    "танцевальное", "танцевальную", "клубное", "клубную", "клубный",
    "клубняк", "быстрое", "медленное", "красивое", "красивую", "эпичное",
    "эпичную", "лирическое", "романтичное", "рок", "джаз", "блюз", "рэп",
    "реп", "хип", "хоп", "техно", "хаус", "транс", "дабстеп", "диско",
    "поп", "попсу", "классику", "классическую", "электронику", "шансон",
    "металл", "панк", "фанк", "соул", "регги", "эмбиент", "чилл", "лоуфай",
    "драм", "бейс", "фонк", "диджей", "dj", "music",
})

#: Слова, с которыми хвост — не название, а просьба с условием или
#: ссылкой на контекст: «сыграй песню ПРО зайчиков», «поставь В СТИЛЕ
#: Dre», «включи ЕГО снова», «давай Я спою», «поставь НА паузу». Такое
#: решает LLM.
_NOT_A_TITLE: FrozenSet[str] = frozenset({
    "про", "стиле", "типа", "похожее", "похожую", "похожий", "как", "вроде",
    "я", "мы", "он", "она", "они", "его", "её", "ее", "их", "это", "этого",
    "эту", "этот", "эта", "то", "так", "лучше", "потом", "дальше", "снова",
    "опять", "ещё", "еще", "заново", "сначала", "обратно", "назад",
    "прошлый", "прошлую", "предыдущий", "предыдущую", "следующий",
    "следующую", "другой", "другую", "другое", "паузу", "пауза",
    "таймер", "будильник", "напоминание", "поговорим", "поиграем",
    "посмотрим", "познакомимся", "не",
})

#: Сколько слов может занимать название («в пещере горного короля» — 4).
_MAX_TITLE_WORDS = 6

#: Слова, без которых родовой заказ — не про музыку («включи звук», «поставь
#: что-то»): ``REQUEST_MUSIC`` только с одним из них (ADR-0149 PR-6).
_MUSIC_NOUNS: FrozenSet[str] = frozenset({
    "музыку", "музыка", "музычку", "музон", "музончик", "трек", "трэк",
    "треки", "бит", "биток", "биты", "микс", "клубняк", "техно", "хаус",
    "транс", "диско", "электронику",
})

#: «музыку ДЛЯ <занятия>» → настроение (ключи ``knowledge.MOOD_ENERGY``).
#: Занятия вне таблицы («для Маши») — не заказ по настроению, решает LLM.
_OCCASION_MOOD: Mapping[str, str] = {
    "танцев": "groove", "танцы": "groove", "вечеринки": "playful", "праздника": "playful",
    "работы": "calm", "учёбы": "calm", "учебы": "calm", "сна": "calm", "отдыха": "calm",
    "расслабления": "calm", "медитации": "calm", "чтения": "calm",
    "тренировки": "epic", "спорта": "epic", "зарядки": "epic",
}


def _strip_lead_in(words: Sequence[str]) -> List[str]:
    """Срезать вежливое вступление до глагола заказа.

    «давай» — и вступление («давай сыграй X»), и глагол («давай к элизе»):
    срезается, только если за ним идёт ещё один глагол заказа.
    """
    out = list(words)
    while out and out[0] in _PLAY_NAMED_LEAD_IN:
        if out[0] in _PLAY_NAMED_VERBS and not (
            len(out) > 1 and out[1] in _PLAY_NAMED_VERBS
        ):
            break
        out.pop(0)
    return out


#: Слова «тема» в заказе: «давай тему X», «включи сет на тему X» (#3410).
_THEME_WORDS: FrozenSet[str] = frozenset({"тему", "тема", "темы"})


def _split_theme_marker(tail: Sequence[str]) -> Tuple[Sequence[str], bool]:
    """``(хвост после слова «тема», было ли оно)``; без слова — хвост как есть."""
    for i, word in enumerate(tail):
        if word in _THEME_WORDS:
            return tail[i + 1:], True
    return tail, False


def _title_words(tail: Sequence[str]) -> List[str]:
    """Хвост после глагола без служебных слов по краям и ведущих родовых."""
    out = list(tail)
    while out and (out[0] in _PLAY_NAMED_FILLER or out[0] in _GENERIC_WORDS):
        out.pop(0)
    while out and out[-1] in _PLAY_NAMED_FILLER:
        out.pop()
    return out


def _is_untitled(title: Sequence[str]) -> bool:
    """Название не годится: длинное, условие/ссылка на контекст, громкость/стоп."""
    return (
        len(title) > _MAX_TITLE_WORDS
        or any(w in _NOT_A_TITLE or _word_class(w) is not None for w in title)
        or all(w in _GENERIC_WORDS for w in title)
    )


def _themed_command(tail: Sequence[str]) -> Optional[MediaCommand]:
    """«давай тему X» → ``PLAY_NAMED(name=X, themed=True)``; без слова «тема» — ``None``."""
    rest, themed = _split_theme_marker(tail)
    title = _title_words(rest)
    if not themed or not title or _is_untitled(title):
        return None
    return MediaCommand(
        intent=MediaIntent.PLAY_NAMED, closed=True, name=" ".join(title), themed=True
    )


def _play_named_command(words: Sequence[str]) -> MediaCommand:
    """«поставь / сыграй / включи <название>» → ``PLAY_NAMED`` с названием.

    Не заказ по имени (``NO_COMMAND``): нет глагола заказа первым словом;
    название пустое или из одних родовых слов («музыку», «что-нибудь
    весёлое», «клубный трек»); в хвосте громкость/стоп («сыграй X
    погромче» — решает LLM, #3125); в хвосте условие или ссылка на
    контекст (:data:`_NOT_A_TITLE`); слишком длинно для названия.
    """
    body = _strip_lead_in(words)
    if not body or body[0] not in _PLAY_NAMED_VERBS:
        return NO_COMMAND
    themed = _themed_command(body[1:])
    if themed is not None:
        return themed
    title = _title_words(body[1:])
    if title and title[0] == "для" and any(w in _MUSIC_NOUNS for w in body[1:]):
        return _occasion_command(title[1:])
    if all(w in _GENERIC_WORDS for w in title):  # и пустое название
        return _request_music_command(body[1:])
    if len(title) > _MAX_TITLE_WORDS:
        return NO_COMMAND
    if any(w in _NOT_A_TITLE or _word_class(w) is not None for w in title):
        return NO_COMMAND
    return MediaCommand(
        intent=MediaIntent.PLAY_NAMED, closed=True, name=" ".join(title)
    )


def _occasion_command(occasion: Sequence[str]) -> MediaCommand:
    """«включи музыку для танцев» → ``REQUEST_MUSIC`` с настроением занятия."""
    mood = next((_OCCASION_MOOD[w] for w in occasion if w in _OCCASION_MOOD), "")
    if not mood or len(occasion) > 3:
        return NO_COMMAND
    return MediaCommand(
        intent=MediaIntent.REQUEST_MUSIC, closed=True, name="музыку для " + " ".join(occasion), mood=mood
    )


def _request_music_command(tail: Sequence[str]) -> MediaCommand:
    """«поставь клубный трек» → ``REQUEST_MUSIC``: только родовые слова и хотя бы одно про музыку."""
    words = [w for w in tail if w not in _PLAY_NAMED_FILLER]
    if not words or len(words) > _MAX_TITLE_WORDS:
        return NO_COMMAND
    if not all(w in _GENERIC_WORDS for w in words) or not any(w in _MUSIC_NOUNS for w in words):
        return NO_COMMAND
    return MediaCommand(
        intent=MediaIntent.REQUEST_MUSIC, closed=True, name=" ".join(words)
    )


#: Слова при названии стиля вместо «диджей-сета»: «включи рейв-сет», «поставь
#: эйсид музыку» (ADR-0153 S1).
_STYLE_SET_FILLER: FrozenSet[str] = _PLAY_NAMED_FILLER | frozenset({
    "сет", "сета", "диджей", "dj", "set", "диджейский", "музыку", "музыка",
    "музычку", "трек", "треки", "микс", "вечеринку", "вечеринка",
})


def _style_set_command(words: Sequence[str]) -> Optional[MediaCommand]:
    """«включи рейв», «давай эйсид на тему космос», «рейв-сет про котов» → DJ-сет стиля (ADR-0153 S1).

    Голова до ввода темы — слово стиля, глагол заказа и служебные слова, и
    ничего больше; иначе ``None`` (решают другие правила или LLM). Стиль —
    один на сет: такая реплика посреди сета начинает новый сет этого стиля.
    """
    body = _strip_lead_in(words)
    if body and body[0] in _PLAY_NAMED_VERBS:
        body = body[1:]
    lead = next((i for i, w in enumerate(body) if w in _THEME_LEADS), len(body))
    head = list(body[:lead])
    if head and head[-1] == "на":
        head.pop()
    style, rest = _style_words(head)
    if not style or any(w not in _STYLE_SET_FILLER and w not in _STYLE_LEAD for w in rest):
        return None
    theme = _style_words(body[lead + 1:])[1]
    return MediaCommand(intent=MediaIntent.DJ, closed=True, style=style, set_theme=" ".join(theme))


#: Слово «сет» без «диджей»: «сыграй интерстеллар сэт», «сэтик про космос» (#3455). STT пишет и «сэт».
_SET_WORDS: FrozenSet[str] = frozenset({
    "сет", "сэт", "сета", "сэта", "сетик", "сэтик", "сетика", "сэтика", "set",
})

#: Глаголы в начале заказа сета: глаголы заказа по имени и запуска сета.
_SET_VERBS: FrozenSet[str] = _PLAY_NAMED_VERBS | frozenset({
    "запусти", "запускай", "включай", "начни", "начинай", "устрой", "замути",
})


def _set_theme_words(head: Sequence[str], tail: Sequence[str]) -> Optional[List[str]]:
    """Тема заказа сета: слова до «сет» или после ввода темы («на тему X», «про X»); ``None`` — не заказ."""
    head = [w for w in head if w not in _GENERIC_WORDS and w not in _PLAY_NAMED_FILLER]
    lead = next((i for i, w in enumerate(tail) if w in _THEME_LEADS), None)
    if lead is None:
        theme, rest = head, list(tail)
    else:
        theme, rest = _title_words(tail[lead + 1:]), list(tail[:lead])
        if head:  # «сыграй X сет на тему Y» — две темы, решает LLM
            return None
    if any(w not in _PLAY_NAMED_FILLER and w != "на" for w in rest):
        return None
    if any(w in _NOT_A_TITLE or _word_class(w) is not None for w in head):
        return None
    return theme if len(head) <= _MAX_TITLE_WORDS else None


def _named_set_command(words: Sequence[str]) -> Optional[MediaCommand]:
    """«сыграй интерстеллар сэт», «включи сэт на тему космос», «сэтик про котов» → DJ-сет с темой (#3455).

    Реплика начинается с глагола заказа (или прямо со слова «сет»), дальше — тема и слово
    :data:`_SET_WORDS`, после него — только ввод темы или служебные слова. Тема — без слова «сет»
    и без слов стиля (стиль — в ``style``, как у «рейв на тему X»). Иначе ``None``.
    """
    body = _strip_lead_in(words)
    verb = bool(body) and body[0] in _SET_VERBS
    if verb:
        body = body[1:]
    at = next((i for i, w in enumerate(body) if w in _SET_WORDS), None)
    if at is None or (at > 0 and not verb):
        return None
    theme = _set_theme_words(body[:at], body[at + 1:])
    if theme is None:
        return None
    style, theme = _style_words(theme)
    return MediaCommand(intent=MediaIntent.DJ, closed=True, style=style, set_theme=" ".join(theme))


# ---------------------------------------------------------------------------
# Заказ сета, тема которого — слова человека (живой замер 06.10 19:08 UTC)
# ---------------------------------------------------------------------------
# «Ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки замути сэт на 30 минут»:
# STT записал «и нас» вместо «у нас», тема не выделилась, реплика ушла в LLM — load_skill + dj_set, 14 с тишины
# до первого трека. Просьба одна — «замути сэт», остальное — тема. Тему из слов реплики выделяет dj_set
# (``engine.theme_grounding.heard_theme`` по скрытому ``heard_text``) — той же функцией, что на пути LLM.

#: Глаголы просьбы: второй такой глагол — вторая просьба («… замути сэт и поставь X»), решает LLM.
#: «давай» — вступление («давай замути сэт»), не просьба.
_ORDER_VERBS: FrozenSet[str] = (_SET_VERBS | _PLAY_NAMED_VERBS) - {"давай", "давайте"}
#: С ними слова реплики — не тема: отрицание, ссылка на контекст, условие (как у названия; «про» — ввод темы),
#: голос робота.
_NOT_THEME: FrozenSet[str] = (_NOT_A_TITLE - _THEME_LEADS) | _VOICE_WORDS
#: Классы слов громкости и стопа: «замути сэт погромче» — не только заказ сета.
_NOT_THEME_CLASSES: FrozenSet[str] = frozenset({"up", "down", "max", "topic", "stop"})


#: Слова просьбы о сете — не тема: глаголы заказа и запуска, «сет», обращение и назначение персоны, длина сета,
#: служебные. Одна таблица для грамматики и для темы из слов реплики (``theme_grounding.heard_theme``, 07.10).
SET_REQUEST_WORDS: FrozenSet[str] = _ORDER_VERBS | _SET_WORDS | _DJ_FILLER | LENGTH_WORDS | frozenset({
    "давай", "давайте", "робот", "робби", "будешь", "станешь", "побудь", "диджеем", "диджея", "сэту", "сету",
    "сеты", "замутим", "замутить", "замутишь", "сделаешь", "организуй", "забабахай", "дай", "дальше", "пусть",
    "можешь", "мы", "я", "наш", "наша", "наше", "нашу", "нашего",
})


def is_spoken_theme_set(text: str) -> bool:
    """Реплика — заказ сета и ничего больше: ровно одна просьба, и это «<глагол> сет» («замути сэт», «включи мне
    сет»); остальные слова — тема (и персона «ты диджей X»). Вопрос, отрицание, ссылка на контекст, громкость/стоп,
    вторая просьба — ``False``, решает LLM."""
    if "?" in text:
        return False
    words = _words(text)
    verbs = [i for i, w in enumerate(words) if w in _ORDER_VERBS]
    if len(verbs) != 1 or any(w in _NOT_THEME or _word_class(w) in _NOT_THEME_CLASSES for w in words):
        return False
    after = [w for w in words[verbs[0] + 1:] if w not in _PLAY_NAMED_FILLER]
    return bool(after) and after[0] in _SET_WORDS


def _spoken_set_command(text: str) -> Optional[MediaCommand]:
    """«ретро Mario Tetris замути сэт» → DJ-сет без темы в команде: тему из слов реплики решает dj_set."""
    if not is_spoken_theme_set(text):
        return None
    return _with_style(MediaCommand(intent=MediaIntent.DJ, closed=True), text)


# ---------------------------------------------------------------------------
# Точка входа
# ---------------------------------------------------------------------------


def parse_media_command(
    user_input: str, *, track_name: Optional[str] = None
) -> MediaCommand:
    """Разобрать реплику юзера (после wake-word) в медиакоманду.

    ``track_name`` — название играющего трека или ``None`` (ничего не
    играет / название неизвестно): с ним «горный король погромче» —
    громкость, без него — вопрос к LLM.
    """
    text = extract_user_utterance(user_input)
    if not text:
        return NO_COMMAND
    tracks, rest = split_set_length(text)  # «сет на 3 трека»: длину решает код, грамматика видит реплику без неё
    command = (_listed_set(rest) or _parse_text(rest, track_name)) if tracks else NO_COMMAND
    if command.intent is MediaIntent.DJ:
        return replace(command, tracks=tracks)
    return _listed_set(text) or _parse_text(text, track_name)


def _listed_set(text: str) -> Optional[MediaCommand]:
    """«включи сет: Марио, Тетрис, Зельда» → DJ-сет, тема — перечисление как сказано (запятые нужны поиску по
    частям, ``engine.search.theme_parts``). Голова до двоеточия — закрытая DJ-команда без темы, иначе ``None``."""
    head, sep, listed = text.partition(":")
    listed = listed.strip(" .!?…")
    if not sep or not listed or all(w in _DJ_FILLER or w in _PLAY_NAMED_FILLER for w in _words(listed)):
        return None
    command = _parse_text(head, None)
    if command.intent is not MediaIntent.DJ or not command.closed or command.set_theme or _THEME_RE.search(head):
        return None
    return replace(command, set_theme=listed, set_persona=command.persona)


#: «Что играет?»: вопросительное слово + слово про звучащее (классы слов, без регексов — мораторий #3132).
_NOW_QUESTION: FrozenSet[str] = frozenset({"что", "какой", "какая", "какое", "чего"})
_NOW_SUBJECT: FrozenSet[str] = frozenset({
    "играет", "звучит", "играешь", "играем", "крутишь", "крутится", "трек", "мелодия", "песня", "песенка",
    "музыка", "поставил", "включил"})
#: «Когда будет / где X?»: X — слова после вопроса без служебных; сверку X с фактами сета делает роутер.
_WHEN_QUESTION: FrozenSet[str] = frozenset({"когда", "где"})
_WHEN_FILLER: FrozenSet[str] = _COMMON_FILLER | frozenset({
    "робот", "робби", "будет", "будут", "заиграет", "заиграют", "сыграешь", "поставишь", "включишь", "их",
    "трек", "треки", "треков", "мелодия", "мелодии", "песня", "песни", "наконец", "мой", "моя", "мои", "то",
    "же", "уже", "опять", "снова"})
_NOW_MAX_WORDS = 7


def _now_playing_command(words: Sequence[str]) -> Optional[MediaCommand]:
    """Вопрос о сете: «что (сейчас) играет?» — ``NOW_PLAYING`` без имени; «когда будет Марио?», «где марио все его
    треки?» — ``NOW_PLAYING`` с ``name``. Ответ строит код из фактов снимка (``media_router``), не LLM (ADR-0148)."""
    words = [w for w in words if w not in ("робот", "робби")]
    while words and words[0] in _COMMON_FILLER and words[0] not in _NOW_QUESTION:  # «а что играет?»
        words = words[1:]
    if not words or len(words) > _NOW_MAX_WORDS:
        return None
    if words[0] in _NOW_QUESTION and any(w in _NOW_SUBJECT for w in words[1:]):
        return MediaCommand(intent=MediaIntent.NOW_PLAYING)
    if words[0] in _WHEN_QUESTION:
        name = [w for w in words[1:] if w not in _WHEN_FILLER]
        if 0 < len(name) <= 3:
            return MediaCommand(intent=MediaIntent.NOW_PLAYING, name=" ".join(name))
    return None


def _parse_text(text: str, track_name: Optional[str]) -> MediaCommand:
    """Разбор реплики без длины сета и перечисления (:func:`parse_media_command`)."""
    if is_dj_request(text):
        return _dj_command(text)
    words = _words(text)
    stop = _stop_command(words)
    if stop.intent is not MediaIntent.NONE:
        return stop
    volume = _volume_command(words, track_name)
    if volume.intent is not MediaIntent.NONE:
        return volume
    now = _now_playing_command(words)
    if now is not None:
        return now
    theme_words = _theme_words(text)  # заказ сета: цифры темы остаются словами (mambo nr 5, #3460)
    # заказ по имени — тоже с цифрами: «сыграй 1812 Overture» искал «overture» (06.10)
    return (_style_set_command(theme_words) or _named_set_command(theme_words) or _spoken_set_command(text)
            or _play_named_command(theme_words))


__all__ = [
    "MEDIA_INTENT_OPTIONS",
    "MUSIC_STOP_COMMAND_RE",
    "MUSIC_STOP_OVERRIDES",
    "MediaCommand",
    "MediaIntent",
    "NO_COMMAND",
    "SET_REQUEST_WORDS",
    "dj_persona_span",
    "extract_dj_persona",
    "extract_user_utterance",
    "is_dj_request",
    "is_music_stop_command",
    "is_spoken_theme_set",
    "parse_media_command",
]
