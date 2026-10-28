"""Поиск мелодии по словам человека: ``find(library, text)`` (ADR-0149 §3.3, I16; #3399).

Одна реализация для заказа по имени (``engine.classic.find_record``, ``lookup_melody``) и для хуков темы сета
(:func:`theme_hooks`).
Сначала запрос как есть — ``RtttlLibrary.get`` (его RU-алиасы и ранжирование не меняются). Если найденная
запись покрывает не все значимые слова, слова разбирает код:

* служебные слова просьбы и темы (``knowledge.SEARCH_STOPWORDS``) отбрасываются по основе слова;
* понятие без мелодии в названии («космос», «денди») → строки поиска семени-начала реестра (``works.word_links``);
* русское слово → транслит слова, затем его основы (падежное окончание снято) → слова архива с тем же
  звуковым ключом (``gadzhet`` ~ ``gadget``, ``inspektor`` ~ ``inspector``); основа сверяется и по началу.
  Слово, которого в архиве нет, ничего не находит (подстрока транслита давала «дела» → «Abdelazer»).

``confidence`` — доля значимых слов запроса, нашедшихся в опознавательных полях записи; ``found`` — не
меньше :data:`FOUND_MIN`. Промах — ``found=False`` без записи (I16), а не «лучшее по тексту».
"""

from __future__ import annotations

import re
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.instruments import edit_distance, instrument_word
from rob_box_music.works import word_links
from rob_box_music.rtttl import CONTOUR_NOTES, THEME_HOOKS, consensus_order, contour  # noqa: F401

from ..core.rtttl_library import _alias_normalize, melody_quality, tagged
from ..core.translit_ru import strip_version_tail, transliterate_ru

#: Доля значимых слов запроса, которую должна покрыть запись, чтобы считаться найденной.
FOUND_MIN = 0.5
SEARCH_LIMIT = 50
#: Хуков темы-перечисления («Марио, Аладдин, Тетрис, Контра»): «мегасет» — десятки три трека. Трек сета ≈ 75 с
#: (живой прогон 06.10: 5–6 треков ≈ 7 мин), 30 треков ≈ 37 мин без повтора хука; дольше — по кругу (давние
#: первыми, ``compose.hook_candidates``). Больше не нужно: каждый хук — RTTTL из библиотеки на старте сета.
THEME_LIST_HOOKS = 30
#: Разделители частей темы-перечисления: знаки списка, скобки, тире (дефис — только с пробелами: «8-бит» — одно
#: слово), союзы «и»/«and». Тема 06.10 «… разных игр — Марио, Аладдин, Тетрис, Чёрный Плащ, Контра, Dendy и другие».
_PART_RE = re.compile(r"[,;:/+()\[\]\n—–]|\s-\s|\b(?:и|and)\b", re.IGNORECASE)
#: Самая длинная часть темы без разделителей, слов («Darkwing Duck», «Super Mario Bros»).
_SEGMENT_WORDS = 3
_WORD_MIN = 4  # русское слово короче («год», «дом», «чип») — не название: совпадений по звуку слишком много
_STEM_MIN = 5  # основа короче («мисс» от «миссия») сверяется только целым словом, не основой
_PREFIX_SLACK = 4  # слово архива длиннее основы не больше чем на столько букв
_MAX_ALTS = 4
_WORD_RE = re.compile(r"[a-z0-9а-я]+")
#: Стиль и формат словами с цифрой («8-бит») — вырезаются до разбора на слова (``knowledge.SEARCH_STYLE_PATTERNS``).
_STYLE_RE = re.compile("|".join(kn.SEARCH_STYLE_PATTERNS), re.IGNORECASE)
#: Слово латиницы не длиннее — совпадает в записи только целым словом: «8» ≠ «1812», «bach» ≠ «Bachelor»,
#: «duck» ≠ «Ducktoy» (06.10); длинное — и внутри слова («mario» в «Supermario Brothers»).
_WHOLE_WORD_MAX = 4
_CYR_RE = re.compile(r"[а-я]")
#: Падежные и родовые окончания, длинные первыми; основа короче трёх букв не остаётся.
_ENDINGS = tuple(sorted((
    "иями", "ями", "ами", "ого", "его", "ому", "ему", "ыми", "ими", "ией", "ах", "ях", "ов", "ев", "ей", "ой", "ий",
    "ый", "ая", "яя", "ое", "ее", "ую", "юю", "ом", "ем", "ам", "ям", "ия", "ие", "ию", "ии", "ы", "и", "а", "я",
    "у", "ю", "е", "о", "ь", "й"), key=len, reverse=True))
_ADJ_ENDINGS = ("ый", "ий", "ой", "ая", "яя", "ое", "ее", "ые", "ие", "ых", "их", "ым", "им")
#: Звуковой ключ латиницы: русский транслит и английское написание одного слова совпадают.
_KEY_RULES = (("dzh", "j"), ("zh", "j"), ("dg", "j"), ("dj", "j"), ("ch", "4"), ("kh", "h"), ("ph", "f"),
              ("th", "t"), ("ck", "k"), ("c", "k"), ("q", "k"), ("x", "ks"), ("w", "v"), ("y", "i"), ("ee", "i"),
              ("oo", "u"))
_IDENTITY = ("name", "title", "artist", "rtttl_name")


def stem(word: str) -> str:
    """Основа русского слова: снято одно окончание из :data:`_ENDINGS`; латиница — как есть."""
    if not _CYR_RE.search(word):
        return word
    for end in _ENDINGS:
        if word.endswith(end) and len(word) - len(end) >= 3:
            return word[:-len(end)]
    return word


def sound_key(latin: str) -> str:
    key = latin
    for src, dst in _KEY_RULES:
        key = key.replace(src, dst)
    return re.sub(r"(.)\1+", r"\1", key)


#: Английское слово, записанное STT кириллицей (#4016: «раннинг ап дэд хилл» — «Running Up That Hill»): правки
#: латиницы до звукового ключа — «wh»/«gh» английского письма; «w» и «v» читаются гласной (кириллицей их пишут то «у»,
#: то «в»: «вонт» — want, «свит» — sweet, «хайвэй» — highway).
_HEARD_LATIN = (("wh", "w"), ("gh", ""), ("w", "u"), ("v", "u"))
#: Звонкие — как глухие: STT и транскрипция путают пары («дэд» — that, «стоунз» — stones).
_DEVOICE = str.maketrans({"b": "p", "d": "t", "g": "k", "z": "s", "j": "4"})
_VOWELS = frozenset("aeiouy")


def _heard_sound(word: str) -> str:
    """Звуковой ключ слова по-английски: латиница (:func:`transliterate_ru` — одна функция транслита), правки
    :data:`_HEARD_LATIN`, :func:`sound_key`."""
    latin = transliterate_ru(word.lower().replace("ё", "е"))
    for src, dst in _HEARD_LATIN:
        latin = latin.replace(src, dst)
    return sound_key(latin)


def phonetic_key(word: str) -> str:
    """Звучание слова для сверки с английским названием (#4016): звуковой ключ без гласных (гласная в начале — метка
    «a»), звонкие оглушены, повторы схлопнуты. «раннинг» и «running» — ``rnk``, «ап» и «up» — ``ap``, «дэд» и «that» —
    ``t``, «хилл» и «hill» — ``hl``. Ключ грубый: одно слово по нему не опознаётся, только название подряд
    (:func:`heard_title`)."""
    key = _heard_sound(word)
    body = re.sub(r"(.)\1+", r"\1", "".join(ch for ch in key if ch not in _VOWELS).translate(_DEVOICE))
    return ("a" if key[:1] in _VOWELS else "") + body


def _strength(key: str) -> int:
    """Сколько согласных в :func:`phonetic_key` — чем больше, тем реже совпадение случайно."""
    return len(key) - key.startswith("a")


_STOP = frozenset(stem(w.replace("ё", "е")) for w in kn.SEARCH_STOPWORDS) | frozenset(kn.SEARCH_STOPWORDS)


@dataclass(frozen=True)
class Term:
    """Значимое слово запроса и его написания в архиве (любое совпало — слово найдено).

    ``latin`` — само слово по звуку, для названий вне словаря архива RTTTL (партитуры, ``score_library``):
    ``(транслит слова, транслит основы, основа сверяется по началу)``; у латиницы — слово как есть (семя
    «interstel*» → ``space`` не отнимает у слова «Interstellar» его собственное написание)."""

    word: str
    alts: Tuple[str, ...]
    latin: Tuple[str, str, bool] = ("", "", False)


@dataclass(frozen=True)
class Found:
    """Структурный исход поиска (ADR-0149 §3.3); ``query`` — строка, по которой нашлась запись."""

    found: bool
    confidence: float
    record: Optional[Dict[str, Any]]
    alternatives: Tuple[Dict[str, Any], ...] = ()
    query: str = ""


class _Vocab:
    def __init__(self, words: Iterable[str]) -> None:
        self.by_key: Dict[str, List[str]] = {}
        for word in sorted(words):
            self.by_key.setdefault(sound_key(word), []).append(word)
        self._sounds: Optional[Dict[str, List[str]]] = None  # по :func:`phonetic_key` — только для разбора на слух

    def sounds(self) -> Dict[str, List[str]]:
        """Указатель звучаний (:func:`phonetic_key` → слова архива) — строится один раз: прогревом при старте
        (:func:`warm_heard`), а без него — первым разбором на слух (#4016), не со словарём."""
        if self._sounds is None:
            sounds: Dict[str, List[str]] = {}
            for words in self.by_key.values():
                for w in words:
                    if w.isalpha():
                        sounds.setdefault(phonetic_key(w), []).append(w)
            self._sounds = sounds
        return self._sounds

    def sounding(self, word: str, limit: int) -> Tuple[str, ...]:
        """Слова архива, звучащие как ``word`` (:func:`phonetic_key`) и близкие по звуковому ключу (правок
        ``edit_distance`` не больше трети его длины), ближе — первыми."""
        heard = _heard_sound(word)
        near = sorted((edit_distance(heard, _heard_sound(w)), abs(len(w) - len(word)), w)
                      for w in self.sounds().get(phonetic_key(word), ()))
        return tuple(w for edits, _n, w in near if edits <= max(1, len(heard) // 3))[:limit]

    def exact(self, latin: str) -> Tuple[str, ...]:
        return tuple(self.by_key.get(sound_key(latin), ()))

    def prefix(self, latin: str) -> Tuple[str, ...]:
        key = sound_key(latin)
        hits = sorted((len(k), k) for k in self.by_key if k.startswith(key) and len(k) - len(key) <= _PREFIX_SLACK)
        return tuple(w for _n, k in hits for w in self.by_key[k])


_VOCAB_CACHE: Dict[int, Tuple[frozenset, _Vocab]] = {}


def _vocab(library: Any) -> _Vocab:
    if library is None:  # без каталога RTTTL (подставной поиск мелодий в тестах сета): только семена и звучание
        return _Vocab(())
    words = library.vocabulary()
    cached = _VOCAB_CACHE.get(id(words))
    if cached is None or cached[0] is not words:
        _VOCAB_CACHE.clear()
        cached = _VOCAB_CACHE[id(words)] = (words, _Vocab(words))
    return cached[1]


def _resolve(vocab: _Vocab, word: str, word_stem: str) -> Tuple[str, ...]:
    """Русское слово → слова архива: ключ слова (не прилагательного: «новый» ≠ «Novy»), основы, затем начало
    основы. Не нашлось — пусто: слово остаётся в знаменателе ``confidence`` и ни с чем не совпадает."""
    if not word.endswith(_ADJ_ENDINGS) and vocab.exact(transliterate_ru(word)):
        return vocab.exact(transliterate_ru(word))[:_MAX_ALTS]
    latin_stem = transliterate_ru(word_stem)
    if len(word_stem) >= _STEM_MIN:
        hits = vocab.exact(latin_stem) or vocab.prefix(latin_stem)
        if hits:
            return hits[:_MAX_ALTS]
    return ()


#: Гласные, которые STT путает без ударения (аканье/иканье): «волкирий» ↔ «валькирий» (13.10, #3599).
#: Слово с основой короче не правим: у коротких одна замена гласной чаще даёт другое настоящее слово («полет» →
#: «палет», «король» → «карол»).
_HEARD_MIN = 6
_HEARD_SWAP = {"о": "а", "а": "о", "е": "и", "и": "е"}


def _heard(vocab: _Vocab, word: str) -> str:
    """Слово, которого нет ни в семенах, ни в архиве, — исправленное одной заменой безударной гласной (``_HEARD_SWAP``),
    если исправленное слово архив знает; иначе слово как есть. Вторая правка (мягкий знак и т. п.) не нужна: звуковой
    ключ транслита её уже снимает. Слово из архива не трогаем: «молоко» ≠ «малако»."""
    if word_links(word) or word_links(stem(word)) or _resolve(vocab, word, stem(word)):
        return word
    for i, ch in enumerate(word):
        swap = _HEARD_SWAP.get(ch)
        if swap is None:
            continue
        fixed = word[:i] + swap + word[i + 1:]
        if _resolve(vocab, fixed, stem(fixed)):
            return fixed
    return word


def terms(library: Any, text: str) -> List[Term]:
    """Значимые слова запроса после алиасов библиотеки, без хвоста версии («V2.0») и служебных слов."""
    vocab = _vocab(library)
    out: List[Term] = []
    text = _STYLE_RE.sub(" ", strip_version_tail(_alias_normalize(text)).replace("ё", "е"))
    for word in _WORD_RE.findall(text):
        if _CYR_RE.search(word) and len(stem(word)) >= _HEARD_MIN and word not in _STOP:
            word = _heard(vocab, word)
        word_stem = stem(word)
        if word in _STOP or word_stem in _STOP:
            continue
        alts = word_links(word) or word_links(word_stem)
        cyrillic = bool(_CYR_RE.search(word))
        if not alts and cyrillic:
            if len(word) < _WORD_MIN:
                continue
            alts = _resolve(vocab, word, word_stem)
        if not cyrillic:
            latin = (word, word, False)
        elif len(word) >= _WORD_MIN:
            latin = (transliterate_ru(word), transliterate_ru(word_stem), len(word_stem) >= _STEM_MIN)
        else:
            latin = ("", "", False)
        out.append(Term(word, alts if cyrillic else alts or (word,), latin))
    return out


def _in(alt: str, hay: str) -> bool:
    """Написание слова в тексте записи: короткое (:data:`_WHOLE_WORD_MAX`) — целым словом, длинное — и подстрокой."""
    if len(alt) > _WHOLE_WORD_MAX:
        return alt in hay
    return re.search(rf"(?<![a-z0-9]){re.escape(alt)}(?![a-z0-9])", hay) is not None


def coverage(record: Dict[str, Any], query_terms: List[Term]) -> float:
    """Доля слов запроса, нашедшихся в опознавательных полях записи (:func:`_in`)."""
    if not query_terms:
        return 0.0
    hay = " ".join(str(record.get(f) or "") for f in _IDENTITY).lower()
    return sum(1 for t in query_terms if any(_in(a, hay) for a in t.alts)) / len(query_terms)


def ranked(library: Any, query_terms: List[Term], direct: Optional[Dict[str, Any]] = None,
           text: str = "") -> List[Tuple[float, Dict[str, Any], str]]:
    """Кандидаты ``(confidence, запись, запрос)``: запись ``get`` как есть (первой при равной доле), затем
    поиск по словам архива — он же даёт альтернативы."""
    pool = [(coverage(direct, query_terms), direct, text)] if direct else []
    query = " ".join(dict.fromkeys(a for t in query_terms for a in t.alts))
    if query:
        pool += [(coverage(r, query_terms), r, query)
                 for r in library.search(query, limit=SEARCH_LIMIT, include_rtttl=True)]
    seen, out = set(), []
    for item in sorted(pool, key=lambda it: -it[0]):  # sorted устойчив: при равной доле — порядок библиотеки
        name = item[1].get("name")
        if name not in seen:
            seen.add(name)
            out.append(item)
    return out


def find(library: Any, text: str, limit: int = 5) -> Found:
    """Мелодия по словам человека: ``found``, ``confidence``, ``record`` и до ``limit - 1`` альтернатив."""
    query_terms = terms(library, text)
    if not query_terms:
        return Found(False, 0.0, None)
    hits = ranked(library, query_terms, library.get(text) if text.strip() else None, text)
    if not hits or hits[0][0] < FOUND_MIN:
        return Found(False, hits[0][0] if hits else 0.0, None)
    confidence, record, query = hits[0]
    alternatives = tuple(r for c, r, _q in hits[1:limit] if c >= FOUND_MIN)
    return Found(True, round(confidence, 3), record, alternatives, query)


#: Редкое слово названия: записей архива с ним не больше (иначе слово общее — «гимн», «марш», «тема» — и одно оно не
#: называет мелодию).
RARE_MAX = 3


def rarely_named(library: Any, known: List[Term]) -> List[Tuple[float, Dict[str, Any], str]]:
    """Записи (:func:`ranked`), где есть все слова ``known``, — если их не больше :data:`RARE_MAX`: тогда слова редкие
    и сами называют мелодию («валькирий» — две записи); иначе ``[]``. Одно правило редкости для :func:`find_rare` и
    разбора на слух (:func:`heard_title`)."""
    full = [item for item in ranked(library, known, None, "") if item[0] >= 1.0]
    return full if len(full) <= RARE_MAX else []


def find_rare(library: Any, text: str) -> Found:
    """Мелодия по редкому слову названия, когда остальные слова архиву неизвестны (#3599: «полёт валькирий» — слова
    «полёт» в названии «Ride Of The Valkyries» нет, а «валькирий» есть у двух записей). Условия: известные слова
    разобраны по словарю архива из русского слова (алиасы и семена реестра дают латиницу: «гимн» → «soviet anthem» —
    понятие, а не слово названия), неизвестных не больше,
    чем известных, и запись, покрывающая все известные слова, не больше :data:`RARE_MAX`. Слово, которое архив знает,
    а запись нет («гимн германии», «марио и тетрис»), сюда не попадает — оно в ``find`` уже ``unmatched``."""
    query_terms = terms(library, text)
    known = _archive_words(query_terms)
    unknown = [t for t in query_terms if not t.alts]
    if not known or not unknown or len(unknown) > len(known):
        return Found(False, 0.0, None)
    full = rarely_named(library, known)
    if not full:
        return Found(False, 0.0, None)
    _confidence, record, query = full[0]
    return Found(True, round(len(known) / len(query_terms), 3), record, tuple(r for _c, r, _q in full[1:]), query)


#: Слова просьбы, а не названия («спой», «исполни», «сочини», «сыграй») — таблицы знания пения.
_ASK_WORDS = frozenset((*kn.SING_VERBS, *kn.SING_PERFORM_VERBS, *kn.SING_CREATE_VERBS, *kn.SING_PLAY_VERBS))
#: Разбор на слух (:func:`heard_title`): слов архива на одно услышанное слово (ближайшие по звуку), строк поиска
#: всего, согласных в ключе слова-опоры поиска («ты», «я» опорой не бывают: совпадений по звуку сотни), чужих слов
#: между словами длинного названия («раннинг ап … хилл») и с какого числа значимых слов название длинное (у
#: короткого чужое слово слышно только на месте его служебного: «кинг оф зе хилл» — «зе» вместо the; «си … хэлл» — не
#: «Say Hello»), доля слов названия, которая должна прозвучать (больше половины: «ин блэк» — не «Yenny From The
#: Block»), с какого числа значимых слов одно может прозвучать иначе («раннинг ап зэт хилл»: «зэт» — that) и каким
#: коротким должно быть услышанное на его месте слово (служебное английское: that, the, it; «брейк» вместо «be» — уже
#: другое название). Сила ключа (согласных в :func:`phonetic_key`), с которой значимые слова названия могут прозвучать
#: через чужие слова (:data:`HEARD_FAR_MIN`, ревью #4124): ключ из одной согласной («си» — ``s``, «эту» — ``at``, «ап» —
#: ``ap``) совпадает со словами фразы случайно — такое слово слышно только рядом с соседним словом названия.
HEARD_ALTS = 2
HEARD_QUERIES = 4
HEARD_ANCHOR_MIN = 2
HEARD_GAP = 2
HEARD_GAP_FROM = 3
HEARD_COVER_MIN = 0.6
HEARD_SLACK_FROM = 4
HEARD_SLACK_WORD = 3
HEARD_FAR_MIN = 2
#: Жанровое слово в конце текста: грамматика снимает его перед названием как род музыки («сыграй рок эраунд зе
#: клок» → название «эраунд зе клок»), а в звучании оно — первое слово названия (:func:`_sound_words`).
_LEAD_STYLE_RE = re.compile(r"(?:" + "|".join(kn.SEARCH_STYLE_PATTERNS) + r")\s*$", re.IGNORECASE)


@dataclass(frozen=True)
class Heard:
    """Название заказа на слух (#4016). ``words`` — слова названия так, как их записал STT (без служебных, слов просьбы
    и инструмента: «раннинг ап дэд хилл»; жанровое слово — тоже слово названия: «рок эраунд зе клок»); ``known`` и
    ``unknown`` — сколько слов названия архив знает и не знает (:func:`_title_span`: жанр — слот заказа, пояснение
    после названия — не название); ``record`` — запись архива, чьё название (или исполнитель — ``field``) звучит
    словами подряд; ``None`` — не нашлось или не искали (понятых не меньше непонятых); ``rare`` — понятые слова
    редкие и сами называют мелодию (:func:`rarely_named`: «черная жемчужина пиратов» — Pirates Of The Caribbean)."""

    words: Tuple[str, ...] = ()
    known: int = 0
    unknown: int = 0
    record: Optional[Dict[str, Any]] = None
    field: str = ""
    rare: bool = False

    @property
    def unclear(self) -> bool:
        """Непонятых слов названия больше, чем понятых."""
        return self.unknown > self.known

    @property
    def truncated(self) -> bool:
        """Поиск по одним понятым словам молча потерял бы название: «раннинг ап дэд хилл» → «хилл» (слово полусотни
        записей). Понятных нет — ищется фраза целиком; понятые редкие (``rare``) — они и есть название; урезания нет."""
        return self.unclear and self.known > 0 and not self.rare

    @property
    def name(self) -> str:
        """Название (или исполнитель) записи, как в каталоге; не нашлось — «»."""
        return str(self.record.get(self.field) or "") if self.record else ""

    @property
    def spoken(self) -> str:
        """Слова названия, как услышаны, — для честной фразы «не уверен, что это за песня»."""
        return " ".join(self.words)


def _title_word(word: str) -> bool:
    """Слово может быть словом названия: не служебное, не слово просьбы, не инструмент заказа («балалайке»,
    «гуслях»)."""
    return not (word in _STOP or stem(word) in _STOP or word in _ASK_WORDS or instrument_word(word))


def _said_words(text: str) -> Tuple[str, ...]:
    """Слова названия, как услышаны (:func:`_title_word`); жанровое слово — тоже слово названия («рок»)."""
    return tuple(w for w in _WORD_RE.findall(text.lower().replace("ё", "е")) if _title_word(w))


def _title_span(library: Any, text: str) -> List[List[Term]]:
    """Слова названия с написаниями в архиве (``[]`` — слово архиву непонятно) до последнего понятого слова. Понятое —
    и алиас библиотеки целиком («имперский марш» → imperial march: поодиночке «имперский» непонятно). Жанр («рок») —
    слот заказа, не слово названия. Пояснение идёт после названия: «имперский марш дарта вейдера», «марио, только
    быстрее» — непонятые слова после последнего понятого не делают название неясным (заказ на develop играл их верно);
    понятых нет — в счёт все слова."""
    normal = _STYLE_RE.sub(" ", _alias_normalize(text).replace("ё", "е"))
    spelled = [[t for t in terms(library, w) if t.alts] for w in _WORD_RE.findall(normal) if _title_word(w)]
    return spelled[:max((i + 1 for i, found in enumerate(spelled) if found), default=len(spelled))]


def _sound_words(text: str, said: str) -> Tuple[str, ...]:
    """Слова, по звучанию которых ищется название: слова названия ``text`` и жанровое слово прямо перед ним в словах
    человека ``said`` — грамматика снимает его как род музыки («сыграй рок эраунд зе клок» → «эраунд зе клок»), а в
    звучании это первое слово названия («Rock Around The Clock»)."""
    title = text.lower().replace("ё", "е")
    before, found, _after = said.lower().replace("ё", "е").partition(title)
    lead = _LEAD_STYLE_RE.search(before) if found and title.strip() else None
    return _said_words(f"{lead.group(0) if lead else ''} {title}")


def heard_title(library: Any, text: str, said: str = "") -> Heard:
    """Название заказа на слух (#4016, живой прогон 24.10: STT записал «Running Up That Hill» как «раннинг ап дэд
    хилл», код оставил одно понятое «хилл» и сыграл «King Of The Hill»). ``text`` — название от грамматики, ``said`` —
    слова человека целиком (по умолчанию — ``text``). Слова названия и сколько из них архив знает
    (:func:`_title_span`); непонятых больше — запись архива, чьё название звучит этими словами подряд
    (:func:`phonetic_key`, :func:`_sound_words`, без ожидания LLM): слова названия — все значимые, по порядку, между
    ними у длинного названия не больше :data:`HEARD_GAP` чужих (одно может прозвучать иначе,
    :data:`HEARD_SLACK_FROM`), у короткого — только на месте служебного; прозвучать должна хотя бы
    :data:`HEARD_COVER_MIN` всех слов названия; название из одного значимого слова не опознаётся."""
    span = _title_span(library, text)
    known = [t for found in span for t in found]
    count = sum(1 for found in span if found)
    heard = Heard(_said_words(text), count, len(span) - count)
    if library is None or not heard.unclear:
        return heard
    record, field = _sounding(library, _sound_words(text, said or text))
    rare = record is None and bool(known) and bool(rarely_named(library, known))
    return replace(heard, record=record, field=field, rare=rare)


def warm_heard(library: Any) -> int:
    """Прогрев разбора на слух (#4016): указатель звучаний архива (14 тыс. слов) строится при старте в фоне, а не в
    первом заказе с непонятым названием; сколько разных звучаний."""
    return len(_vocab(library).sounds())


def _sounding(library: Any, words: Sequence[str]) -> Tuple[Optional[Dict[str, Any]], str]:
    """Запись, чьё название (или исполнитель) звучит словами ``words``: по строкам :func:`_sound_queries` — лучшая
    находка (доля названия, затем число значимых слов; при равных — первая), название целиком — без следующих
    строк."""
    keys = [phonetic_key(w) for w in words]
    heard = [(key, len(word) <= HEARD_SLACK_WORD) for key, word in zip(keys, words)]
    best: Tuple[Tuple[float, int], Optional[Dict[str, Any]], str] = ((0.0, 0), None, "")
    for query in _sound_queries(_vocab(library), words, keys):
        best = max([best, *_scored(library.search(query, limit=SEARCH_LIMIT), heard)], key=lambda hit: hit[0])
        if best[0][0] >= 1.0:  # прозвучало всё название — лучше не будет
            break
    return best[1], best[2]


def _sound_queries(vocab: _Vocab, words: Sequence[str], keys: Sequence[str]) -> List[str]:
    """Строки поиска на слух: слова архива, звучащие как самое «согласное» услышанное слово (опора не слабее
    :data:`HEARD_ANCHOR_MIN`), по одному, затем ближайшее из них в паре со словами второй опоры."""
    order = sorted((i for i in range(len(words)) if _strength(keys[i]) >= HEARD_ANCHOR_MIN),
                   key=lambda i: (-_strength(keys[i]), i))
    anchors = [alts for alts in (vocab.sounding(words[i], HEARD_ALTS) for i in order) if alts][:2]
    if not anchors:
        return []
    pairs = anchors[1] if len(anchors) > 1 else ()
    return [*anchors[0], *(f"{anchors[0][0]} {alt}" for alt in pairs)][:HEARD_QUERIES]


def _scored(records: Iterable[Dict[str, Any]],
            heard: Sequence[Tuple[str, bool]]) -> List[Tuple[Tuple[float, int], Dict[str, Any], str]]:
    """``(оценка, запись, поле)`` записей, чьё название или исполнитель звучит услышанными словами
    (:func:`_aligned`)."""
    return [(score, record, field) for record in records for field in ("title", "artist")
            if (score := _aligned(str(record.get(field) or ""), heard)) is not None]


def _aligned(name: str, heard: Sequence[Tuple[str, bool]]) -> Optional[Tuple[float, int]]:
    """``(доля прозвучавших слов названия, значимых слов)``, если название ``name`` звучит подряд идущими услышанными
    словами ``heard`` (``(ключ, короткое ли)``; условия — :func:`heard_title`), иначе ``None``."""
    title = title_key(strip_version_tail(name)).split()
    significant = [i for i, w in enumerate(title) if w not in _STOP]
    if len(significant) < 2:
        return None
    tkeys = [phonetic_key(w) for w in title]
    room = (int(len(significant) >= HEARD_SLACK_FROM), HEARD_GAP if len(significant) >= HEARD_GAP_FROM else 0)
    walks = (_walk(title, tkeys, heard, start, significant[0], room) for start, (key, _short) in enumerate(heard)
             if key == tkeys[significant[0]])
    matched = max((m for m in walks if m is not None), default=0)
    cover = matched / len(title)
    return (cover, len(significant)) if matched and cover >= HEARD_COVER_MIN else None


def _walk(title: Sequence[str], tkeys: Sequence[str], heard: Sequence[Tuple[str, bool]], start: int, first: int,
          room: Tuple[int, int]) -> Optional[int]:
    """Сколько слов названия прозвучало, если его первое значимое слово — услышанное ``start``. ``room`` — ``(slack,
    gap)``: значимое не прозвучало — ``None``, кроме ``slack`` слов, на месте которых услышано другое короткое слово
    («зэт» вместо that); между прозвучавшими — не больше ``gap`` чужих слов, а на месте непрозвучавших служебных
    («of», «the») — по одному («кинг оф зе хилл»: «зе» вместо the). Через чужие слова — только между значимыми словами
    с сильными ключами (не слабее :data:`HEARD_FAR_MIN`): слово со слабым ключом слышно лишь рядом с соседним словом
    названия — «эту как её там» не «Out Of Time» («эту» — ``at``), «си … хэлл» не «Say Hello» (``s``)."""
    (slack, gap), pos, matched, free = room, start + 1, 1, 0
    strong = _strength(tkeys[first]) >= HEARD_FAR_MIN  # у последнего прозвучавшего значимого слова сильный ключ
    for i in range(first + 1, len(title)):
        stop = title[i] in _STOP
        bridge = max(gap, free) if strong and (stop or _strength(tkeys[i]) >= HEARD_FAR_MIN) else 0
        hit = next((p for p in range(pos, min(len(heard), pos + bridge + 1)) if heard[p][0] == tkeys[i]), None)
        if hit is not None:
            pos, matched, free = hit + 1, matched + 1, 0
            strong = strong if stop else _strength(tkeys[i]) >= HEARD_FAR_MIN
        elif stop:
            free += 1
        elif slack and pos < len(heard) and heard[pos][1]:
            pos, slack = pos + 1, slack - 1
        else:
            return None
    return matched


def _archive_words(query_terms: List[Term]) -> List[Term]:
    """Слова, разобранные по словарю архива из русского слова; ``[]``, если среди известных есть латиница (алиасы) или
    семя реестра — это понятие, а не слово названия."""
    known = [t for t in query_terms if t.alts]
    if any(not _CYR_RE.search(t.word) or word_links(t.word) or word_links(stem(t.word)) for t in known):
        return []
    return known


@dataclass(frozen=True)
class ThemeHits:
    """Мелодии темы сета (``theme.seeded_profile(found=names, exact=exact)``); ``exact`` — тема и есть название записи
    архива: хуки — эта запись и её версии, строка таблицы тем их не дополняет (#3427)."""

    names: Tuple[str, ...] = ()
    exact: bool = False
    missing: Tuple[str, ...] = ()  # части темы-перечисления, по которым не нашлось ни одной мелодии
    #: хуки ``names`` по частям темы-перечисления в порядке названного (франшизы) — сет чередует части
    #: (``theme.ThemeProfile.theme_parts``); пусто — тема одна
    parts: Tuple[Tuple[str, ...], ...] = ()
    #: тема называет конкретные вещи (перечисление или произведение, ``engine.theme_links``): ничего не нашлось —
    #: сет не играет случайный пул (#3493, 07.10 «нахуя тетрис»)
    named: bool = False
    #: строки поиска, по которым искались эти мелодии (:func:`part_query`); ими же ищутся партитуры
    #: (``score_library.ScoreIndex``) — одно разрешение темы на оба каталога (#3512)
    query: Optional["ThemeQuery"] = None


@dataclass(frozen=True)
class PartQuery:
    """Часть темы и её строки поиска (:func:`part_query`): слова с написаниями в архиве (семена реестра
    ``works.word_links``, алиасы библиотеки, слова архива по звуку), жанр каталога и связи реестра
    (``engine.theme_links``: семена-фразы, Шифу, проверенные каталогом ответы LLM)."""

    text: str
    terms: Tuple[Term, ...] = ()
    genre: Optional[str] = None  # метка жанра каталога, названная словами части (:func:`genre_of`)
    genre_only: bool = False  # часть называет только жанр («классическая музыка», «музыка из фильмов»)
    links: Tuple[str, ...] = ()  # проверенные строки связей реестра для части, которую прямой поиск не нашёл

    def strings(self) -> Tuple[str, ...]:
        """Все строки, которыми ищется часть, — для честной строки лога «что искали»."""
        out = [a for t in self.terms for a in (*t.alts, t.latin[0]) if a]
        out += list(self.links) + ([f"жанр:{self.genre}"] if self.genre else [])
        return tuple(dict.fromkeys(out))


@dataclass(frozen=True)
class ThemeQuery:
    """Тема сета, разрешённая в строки поиска по частям (:func:`theme_search`); потребители — поиск мелодий RTTTL
    и поиск партитур, оба по одним и тем же :class:`PartQuery`."""

    theme: str
    parts: Tuple[PartQuery, ...] = ()

    def with_links(self, links: Mapping[str, Sequence[str]]) -> "ThemeQuery":
        """Тема со строками связей реестра частей (``{часть: строки}``); части, которой нет, — добавляется."""
        texts = {p.text for p in self.parts}
        parts = [replace(p, links=tuple(dict.fromkeys((*p.links, *links.get(p.text, ()))))) for p in self.parts]
        parts += [PartQuery(t, links=tuple(q)) for t, q in links.items() if t not in texts and q]
        return replace(self, parts=tuple(parts))

    def describe(self) -> str:
        """«бах» ['bach']; «моцарт» ['mozart', 'motsart'] — строка лога, чем искали."""
        return "; ".join(f"«{p.text}» {list(p.strings())}" for p in self.parts) or "нет значимых слов"


def part_query(library: Any, text: str) -> PartQuery:
    """Часть темы → строки поиска. Одна функция для обоих каталогов (#3512): мелодии RTTTL (:func:`theme_search`)
    и партитуры (``score_library.ScoreIndex``) ищут по тому, что она вернула. ``library=None`` — без словаря
    архива RTTTL (только семена и звучание слова)."""
    tag = _genre_only(library, text)
    return PartQuery(text, tuple(terms(library, text)), tag or genre_of(text), tag is not None)


def title_key(text: str) -> str:
    """Название для сравнения целиком: нижний регистр, ё → е, только слова (знаки и регистр не различают)."""
    return " ".join(_WORD_RE.findall(str(text or "").lower().replace("ё", "е")))


def _is_exact(record: Dict[str, Any], key: str) -> bool:
    return key in (title_key(record.get("title")), title_key(record.get("name")))


def _identity_words(record: Dict[str, Any]) -> set:
    """Слова опознавательных полей записи (название, исполнитель, имена) — «Theme» от «Terminator Soundtrack»."""
    return {w for f in _IDENTITY for w in title_key(record.get(f)).split()}


def _pool(library: Any, theme: str, query_terms: List[Term],
          found_min: float = FOUND_MIN) -> List[Tuple[float, Dict[str, Any]]]:
    """Найденные по словам темы (доля ≥ ``found_min``) и — точные по названию записи из поиска по строке темы
    как есть: точная запись в набор входит, даже если слова темы её не выделили («Give In To Me» без «in», «to»)."""
    pool = [(c, r) for c, r, _q in ranked(library, query_terms) if c >= found_min]
    seen = {r.get("name") for _c, r in pool}
    key = title_key(theme)
    extra = [r for r in library.search(theme, limit=SEARCH_LIMIT, include_rtttl=True)
             if _is_exact(r, key) and r.get("name") not in seen]
    return [(1.0, r) for r in extra] + pool


def _whole_search(library: Any, theme: str, limit: int, found_min: float = FOUND_MIN,
                  query_terms: Optional[Sequence[Term]] = None) -> ThemeHits:
    """Мелодии по словам темы целиком, лучшие первыми (#3427):

    1. точное совпадение названия записи (``title`` или ``name``, :func:`title_key`) с темой — всегда первым;
       тогда в набор идут ещё только записи, в опознавательных полях которых есть все слова темы, — совпадения по
       одному общему слову («remix», «give») отсекаются;
    2. затем — :func:`consensus_order` (версии одной мелодии выше одиночной записи).

    ``found_min`` — доля слов темы, которую покрывает запись (часть темы-перечисления — все слова, 1.0);
    ``query_terms`` — уже разрешённые слова темы (:func:`part_query`), иначе — :func:`terms`."""
    query_terms = list(terms(library, theme) if query_terms is None else query_terms)
    if not query_terms:
        return ThemeHits()
    return ordered(_pool(library, theme, query_terms, found_min), theme, limit)


def ordered(pool: List[Tuple[float, Dict[str, Any]]], theme: str, limit: int) -> ThemeHits:
    """Находки :func:`_pool` по порядку :func:`_whole_search`: точное название — первым, затем версии выше одиночек."""
    key = title_key(theme)
    exact = [hit for hit in pool if _is_exact(hit[1], key)]
    if not exact:
        return ThemeHits(tuple(consensus_order(pool, limit)))
    words = set(key.split())
    rest = [hit for hit in pool if not _is_exact(hit[1], key) and words <= _identity_words(hit[1])]
    return ThemeHits(tuple((consensus_order(exact, limit) + consensus_order(rest, limit))[:limit]), True)


def genre_of(text: str) -> Optional[str]:
    """Метка каталога жанра, названного словами текста (``knowledge.GENRE_TAGS``): «классика» → ``classical``."""
    for word in _WORD_RE.findall(text.lower().replace("ё", "е")):
        for key, tag in kn.GENRE_TAGS.items():
            if word.startswith(key):
                return tag
    return None


def blank_style(text: str) -> str:
    """Текст без слов стиля с цифрой («8битный», «8-бит», «16 bit», :data:`knowledge.SEARCH_STYLE_PATTERNS`):
    они заменены пробелами той же длины — позиции остальных слов не сдвигаются. Стиль — не тема сета (#3476)."""
    return _STYLE_RE.sub(lambda m: " " * len(m.group()), text)


def content_stems(text: str) -> frozenset:
    """Значимые основы слов текста: без служебных слов темы (``knowledge.SEARCH_STOPWORDS``) и стиля («8-бит»),
    жанр — меткой ``genre:<тег>`` (:func:`genre_of`: «классическая» и «классика» — одна основа). Сверка темы сета
    с репликой человека (``engine.theme_grounding``): общая основа — тема опирается на реплику."""
    out = set()
    for word in _WORD_RE.findall(_STYLE_RE.sub(" ", text.lower().replace("ё", "е"))):
        tag = genre_of(word)
        if tag:
            out.add(f"genre:{tag}")
        elif len(word) >= 3 and word not in _STOP and stem(word) not in _STOP:
            out.add(stem(word))
    return frozenset(out)


def _genre_only(library: Any, part: str) -> Optional[str]:
    """Метка жанра, если часть темы называет только жанр («классическая музыка»), иначе ``None``."""
    words = {t.word for t in terms(library, part) if not genre_of(t.word)} - set(kn.GENRE_FILLER)  # «кино»
    return None if words else genre_of(part)


def genre_hooks(library: Any, tag: str, limit: int = THEME_LIST_HOOKS) -> Tuple[str, ...]:
    """Мелодии жанра каталога: записи с меткой ``tag`` (без ``knowledge.GENRE_NOT``, с ``GENRE_EXTRA``). Лучшие по
    :func:`melody_quality` первыми, разные исполнители по кругу — первые хуки не восемь версий одного композитора."""
    bad = set(kn.GENRE_NOT.get(tag, ()))
    records = {r["name"]: r for r in tagged(library, tag) if r["name"] not in bad}
    for name in kn.GENRE_EXTRA.get(tag, ()):
        record = library.get(name)
        if record and record.get("name") == name:
            records.setdefault(name, record)
    by_artist: Dict[str, List[Tuple[float, str]]] = {}
    for name, record in records.items():
        score = melody_quality(str(record.get("rtttl") or ""))
        by_artist.setdefault(title_key(record.get("artist")) or name, []).append((-score, name))
    groups = sorted((sorted(g) for g in by_artist.values()), key=lambda g: g[0])
    return tuple(round_robin([[n for _s, n in g] for g in groups], limit))


def _split(library: Any, theme: str) -> List[str]:
    parts = (" ".join(p.split()) for p in _PART_RE.split(theme) if p)
    return [p for p in parts if p and (terms(library, p) or genre_of(p))]


def _known(library: Any, part: str) -> bool:
    """Часть темы называет жанр или мелодию, все слова которой стоят в опознавательных полях одной записи архива
    (:func:`_identity_words`): «Stranger Things» — да; «денди-стиль», «саундтрек» — нет, это описание сета. Алиасы
    слов (``Term.alts``: «денди» → Марио, Контра) не в счёт — иначе любое слово-франшиза в голове было бы названием."""
    if genre_of(part):
        return True
    words = set(title_key(part).split())
    hits = _whole_search(library, part, THEME_HOOKS, found_min=1.0).names
    return bool(words) and any(words <= _identity_words(library.get(name) or {}) for name in hits)


def _span(library: Any, words: List[str], i: int) -> int:
    """Сколько слов с ``i`` покрывает одна запись архива (0 — ни одной), не заходя на слово жанра."""
    room = next((n for n, w in enumerate(words[i:]) if _genre_only(library, w)), len(words) - i)
    return next((n for n in range(min(_SEGMENT_WORDS, room), 0, -1)
                 if _whole_search(library, " ".join(words[i:i + n]), 1, found_min=1.0).names), 0)


def _segment(library: Any, theme: str) -> List[str]:
    """Части темы без разделителей (STT отдаёт «ретро 8-бит Mario Tetris Aladdin Contra» без запятых, 06.10): по
    каталогу. С каждого значимого слова — самая длинная фраза до :data:`_SEGMENT_WORDS` слов, которую покрывает
    одна запись архива целиком (``found_min=1.0``): «Darkwing Duck» — одна часть, «Mario Tetris» — две. Слово без
    записи — своя часть (уйдёт в ``missing``). Меньше двух найденных частей — пусто: тема одна, ищется целиком."""
    filler = set(kn.GENRE_FILLER) if genre_of(theme) else set()  # «classical music» — жанр, а не запись «Music»
    words = [w for w in _WORD_RE.findall(_STYLE_RE.sub(" ", theme.lower().replace("ё", "е")))
             if w not in filler and (terms(library, w) or genre_of(w))]
    if len(words) < 2:
        return []
    parts: List[str] = []
    found = 0
    i = 0
    while i < len(words):
        if _genre_only(library, words[i]):  # жанр — своя часть («классическая») и находка
            parts.append(words[i])
            found += 1
            i += 1
            continue
        span = _span(library, words, i)
        parts.append(" ".join(words[i:i + (span or 1)]))
        found += bool(span)
        i += span or 1
    return parts if found >= 2 else []


def theme_parts(library: Any, theme: str) -> List[str]:
    """Части темы-перечисления (:data:`_PART_RE`) со значимыми словами; части из одних служебных слов
    («мегасет для игроков из RTTTL-мелодий разных игр», «и другие», «ретро 8-бит») выпадают. Перед двоеточием —
    описание сета («денди-стиль: Тетрис, Контра»): после двоеточия перечисление (две части и больше) — части
    из него, первая названная франшиза — первая часть (06.10). Часть головы, которую архив знает целиком
    (запись покрывает все её слова, :func:`_known`) или жанр, — не описание, а названная мелодия: идёт перед
    перечислением («Stranger Things: тёмный синтвейв, 80-е» — #4005)."""
    head, colon, tail = theme.partition(":")
    listed = _split(library, tail) if colon else []
    if len(listed) >= 2:
        named = [p for p in _split(library, head) if _known(library, p)]
        return named + [p for p in listed if p not in named]
    split = _split(library, theme)
    return split if len(split) >= 2 else _segment(library, theme) or split


def round_robin(lists: Sequence[Sequence[str]], limit: int) -> List[str]:
    """Слияние по кругу: первая не взятая мелодия каждой части, затем вторая… — первые треки сета из разных
    франшиз, а не восемь версий «Марио» подряд; повтор (одна мелодия в двух частях) берётся один раз."""
    queues = [deque(names) for names in lists]
    out: List[str] = []
    while len(out) < limit and any(queues):
        for queue in queues:
            while queue and queue[0] in out:
                queue.popleft()
            if queue and len(out) < limit:
                out.append(queue.popleft())
    return out


def by_part(lists: Sequence[Sequence[str]], names: Sequence[str]) -> Tuple[Tuple[str, ...], ...]:
    """Хуки ``names`` по частям ``lists`` в их порядке; мелодия двух частей — в первой; пустые части выпадают."""
    left = set(names)
    out = []
    for part in lists:
        group = tuple(n for n in part if n in left)
        left -= set(group)
        if group:
            out.append(group)
    return tuple(out)


def theme_search(library: Any, theme: str, limit: int = THEME_HOOKS) -> ThemeHits:
    """Мелодии по словам темы сета для хука, лучшие первыми.

    Точное совпадение темы с названием записи — приоритет (#3427, :func:`_whole_search`). Тема-перечисление
    (две и больше частей со значимыми словами, :func:`theme_parts`) ищется по частям — каждая до ``limit`` мелодий,
    слияние :func:`round_robin` до :data:`THEME_LIST_HOOKS`: одна мелодия не покрывает половину слов темы «Марио,
    Тетрис, Контра», и целиком тема не находила ничего (06.10). Запись части покрывает все её слова: «Darkwing
    Duck» — не «Ducktoy» по слову «duck»; запись темы целиком («Tom and Jerry») — тоже все слова, иначе это
    находка одной части не в её очереди («Марио, Тетрис» ставил «Tetris» первым). Часть без находок — в
    ``missing`` (в лог), не подменяется. Ни одной мелодии — пусто, сет возьмёт пул по хешу темы.

    Строки поиска каждой части — :func:`part_query`; они же уходят в ``ThemeHits.query`` — по ним ищутся партитуры
    (#3512), второго разбора темы нет."""
    whole_query = part_query(library, theme)
    whole = _whole_search(library, theme, limit, query_terms=whole_query.terms)
    parts = [] if whole.exact else theme_parts(library, theme)
    if len(parts) < 2 and not (parts and _genre_only(library, parts[0])):
        return replace(whole, query=ThemeQuery(theme, (whole_query,)))
    queries = tuple(part_query(library, p) for p in parts)
    found = []
    missing = []
    for q in queries:
        names = genre_hooks(library, q.genre) if q.genre_only else _whole_search(
            library, q.text, limit, found_min=1.0, query_terms=q.terms).names
        if names:
            found.append(names)
        else:
            missing.append(q.text)
    spanning = _whole_search(library, theme, limit, found_min=1.0, query_terms=whole_query.terms).names \
        if len(parts) >= 2 else ()
    if spanning and spanning not in found:  # запись темы целиком — первой; совпавшая с частью не дублируется
        found.insert(0, spanning)
    names = tuple(round_robin(found, THEME_LIST_HOOKS))
    return ThemeHits(names, False, tuple(missing), by_part(found, names), named=True,
                     query=ThemeQuery(theme, queries))


def theme_hooks(library: Any, theme: str, limit: int = THEME_HOOKS) -> Tuple[str, ...]:
    """Имена мелодий темы (:func:`theme_search`)."""
    return theme_search(library, theme, limit).names


__all__ = ["CONTOUR_NOTES", "FOUND_MIN", "Found", "HEARD_ALTS", "HEARD_ANCHOR_MIN", "HEARD_COVER_MIN", "HEARD_GAP",
           "HEARD_FAR_MIN", "HEARD_GAP_FROM", "HEARD_QUERIES", "HEARD_SLACK_FROM", "HEARD_SLACK_WORD", "Heard",
           "RARE_MAX", "heard_title", "phonetic_key", "rarely_named", "warm_heard", "PartQuery", "SEARCH_LIMIT",
           "THEME_HOOKS", "THEME_LIST_HOOKS",
           "Term", "ThemeHits", "ThemeQuery", "blank_style", "by_part", "consensus_order", "coverage", "find",
           "genre_hooks", "genre_of", "part_query", "ranked", "round_robin", "sound_key", "stem", "terms",
           "theme_hooks",
           "theme_parts", "theme_search", "title_key"]
