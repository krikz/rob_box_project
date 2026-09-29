"""media_command_grammar.py — закрытая грамматика медиакоманд (issue #3134, Ш2).

Одна грамматика для команд, которые исполняет КОД, а не LLM
(``docs/design/2026-09-28-music-dj-systemic-analysis.md`` §6 Ш2):

* громкость музыки — «громче / тише / потише / погромче / на максимум»,
  в том числе с названием играющего трека («горный король погромче»);
* стоп — «выключи музыку», «стоп диджей», «хватит диджеить»;
* DJ — «ты диджей X», «запусти диджей-сет»;
* заказ по имени — «поставь / сыграй / включи <название>» (issue #3176).

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

import re
from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, List, Optional, Sequence, Tuple


class MediaIntent(str, Enum):
    """Закрытый набор интентов роутера (варианты ``ChoiceQuestion``)."""

    NONE = "none"
    VOLUME_UP = "volume_up"
    VOLUME_DOWN = "volume_down"
    VOLUME_MAX = "volume_max"
    STOP = "stop"
    DJ = "dj"
    PLAY_NAMED = "play_named"


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
        persona: для ``DJ`` — «диджей X» или ``""``.
        theme: для ``DJ`` — тема из «у нас сегодня …» или ``""``.
        name: для ``PLAY_NAMED`` — название мелодии словами юзера
            («к элизе»), без глагола и служебных слов.
    """

    intent: MediaIntent
    closed: bool = True
    persona: str = ""
    theme: str = ""
    name: str = ""


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
    r"|(?P<noun>музык\w*|музон\w*|дидж\w*|трек\w*|трей|сет|бит|бита|звук\w*|dj))$"
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

#: Маркер авто-перехода DJ: его промпт начинается с «Ты диджей <персона>»,
#: но это НЕ запрос юзера.
_DJ_AUTO_MARKER = "[dj_auto"

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
    r"(?:ди-?дж[еэ]й[\s-]*сет|dj[\s-]*сет|dj[\s-]*set|диджейск\w*\s+сет|"
    r"режим\w*\s+(?:ди-?дж[еэ]я|dj)|ди-?дж[еэ]й[\s-]*режим|dj[\s-]*режим|"
    r"dj[\s-]*mode)",
    re.IGNORECASE,
)

#: Имя персоны сразу после назначения: до « и », знака препинания или конца.
_PERSONA_NAME_RE = re.compile(
    r"\s+((?:[^\s,.!?]+\s*){1,4}?)(?=\s+и\s|\s*[,.!?]|\s*$)",
    re.IGNORECASE,
)

#: Тема после «у нас (сегодня|тут)».
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
    "диджейский", "диджея",
})


def is_dj_request(user_input: Optional[str]) -> bool:
    """Юзер назначает DJ-персону или просит запустить DJ-сет.

    Узкий по построению: «диджей, сделай громче», «стоп диджей», «хватит
    диджеить», «выключи режим диджея», «кто такой диджей?» — НЕ DJ-запрос.
    """
    if not user_input:
        return False
    low = user_input.lower()
    if _DJ_AUTO_MARKER in low:
        return False
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


def extract_dj_request_hint(user_input: Optional[str]) -> Tuple[str, str]:
    """``(persona, theme)`` из DJ-запроса; пустые строки — не нашлось.

    Персона — с префиксом «диджей», как её пишет ``set_dj_mode``.
    """
    text = (user_input or "").strip()
    name, _span = _persona_span(text)
    persona = f"диджей {name}" if name else ""
    t = _THEME_RE.search(text)
    theme = t.group(1).strip() if t else ""
    return persona, theme


def _cut(text: str, spans: Sequence[Optional[Tuple[int, int]]]) -> str:
    out = text
    for span in sorted((s for s in spans if s), reverse=True):
        out = out[: span[0]] + " " + out[span[1]:]
    return out


def _dj_command(text: str) -> MediaCommand:
    persona, theme = extract_dj_request_hint(text)
    _name, persona_span = _persona_span(text)
    theme_m = _THEME_RE.search(text)
    start_m = _SET_START_RE.search(text)
    rest = _cut(text, [
        persona_span,
        theme_m.span() if theme_m else None,
        start_m.span() if start_m else None,
    ])
    closed = all(w in _DJ_FILLER for w in _words(rest))
    return MediaCommand(
        intent=MediaIntent.DJ, closed=closed, persona=persona, theme=theme
    )


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
    "тему", "микс", "сет", "плейлист", "радио", "звук", "что", "то",
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


def _title_words(tail: Sequence[str]) -> List[str]:
    """Хвост после глагола без служебных слов по краям и ведущих родовых."""
    out = list(tail)
    while out and (out[0] in _PLAY_NAMED_FILLER or out[0] in _GENERIC_WORDS):
        out.pop(0)
    while out and out[-1] in _PLAY_NAMED_FILLER:
        out.pop()
    return out


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
    title = _title_words(body[1:])
    if not title or len(title) > _MAX_TITLE_WORDS:
        return NO_COMMAND
    if any(w in _NOT_A_TITLE or _word_class(w) is not None for w in title):
        return NO_COMMAND
    if all(w in _GENERIC_WORDS for w in title):
        return NO_COMMAND
    return MediaCommand(
        intent=MediaIntent.PLAY_NAMED, closed=True, name=" ".join(title)
    )


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
    if is_dj_request(text):
        return _dj_command(text)
    words = _words(text)
    stop = _stop_command(words)
    if stop.intent is not MediaIntent.NONE:
        return stop
    volume = _volume_command(words, track_name)
    if volume.intent is not MediaIntent.NONE:
        return volume
    return _play_named_command(words)


__all__ = [
    "MEDIA_INTENT_OPTIONS",
    "MUSIC_STOP_COMMAND_RE",
    "MUSIC_STOP_OVERRIDES",
    "MediaCommand",
    "MediaIntent",
    "NO_COMMAND",
    "extract_dj_request_hint",
    "extract_user_utterance",
    "is_dj_request",
    "is_music_stop_command",
    "parse_media_command",
]
