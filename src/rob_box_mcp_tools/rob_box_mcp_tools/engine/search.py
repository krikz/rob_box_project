"""Поиск мелодии по словам человека: ``find(library, text)`` (ADR-0149 §3.3, I16; #3399).

Одна реализация для заказа по имени (``engine.classic.find_record``, ``lookup_melody``) и для хуков темы сета
(:func:`theme_hooks`).
Сначала запрос как есть — ``RtttlLibrary.get`` (его RU-алиасы и ранжирование не меняются). Если найденная
запись покрывает не все значимые слова, слова разбирает код:

* служебные слова просьбы и темы (``knowledge.SEARCH_STOPWORDS``) отбрасываются по основе слова;
* понятие без мелодии в названии («космос», «денди») → английский запрос (``knowledge.THEME_CONCEPTS``);
* русское слово → транслит слова, затем его основы (падежное окончание снято) → слова архива с тем же
  звуковым ключом (``gadzhet`` ~ ``gadget``, ``inspektor`` ~ ``inspector``); основа сверяется и по началу.
  Слово, которого в архиве нет, ничего не находит (подстрока транслита давала «дела» → «Abdelazer»).

``confidence`` — доля значимых слов запроса, нашедшихся в опознавательных полях записи; ``found`` — не
меньше :data:`FOUND_MIN`. Промах — ``found=False`` без записи (I16), а не «лучшее по тексту».
"""

from __future__ import annotations

import re
from collections import Counter, deque
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.rtttl import contour

from ..core.rtttl_library import _alias_normalize, melody_quality, tagged
from ..core.translit_ru import strip_version_tail, transliterate_ru

#: Доля значимых слов запроса, которую должна покрыть запись, чтобы считаться найденной.
FOUND_MIN = 0.5
SEARCH_LIMIT = 50
#: Сколько найденных по теме мелодий получает профиль сета (кандидаты хука и LLM); у темы-перечисления — на часть.
THEME_HOOKS = 8
#: Хуков темы-перечисления («Марио, Аладдин, Тетрис, Контра»): «мегасет» — десятки три трека. Трек сета ≈ 75 с
#: (живой прогон 06.10: 5–6 треков ≈ 7 мин), 30 треков ≈ 37 мин без повтора хука; дольше — по кругу (давние
#: первыми, ``compose.hook_candidates``). Больше не нужно: каждый хук — RTTTL из библиотеки на старте сета.
THEME_LIST_HOOKS = 30
#: Разделители частей темы-перечисления: знаки списка, скобки, тире (дефис — только с пробелами: «8-бит» — одно
#: слово), союзы «и»/«and». Тема 06.10 «… разных игр — Марио, Аладдин, Тетрис, Чёрный Плащ, Контра, Dendy и другие».
_PART_RE = re.compile(r"[,;:/+()\[\]\n—–]|\s-\s|\b(?:и|and)\b", re.IGNORECASE)
#: Нот в контуре начала для «консенсуса версий» (#3427): при 7 версии «Terminator» theme_177/theme_178 совпадают,
#: а повторные ноты «Space Quest» и «Exploration Of Space» уже различаются (при 6 — нет; при 8 расходятся 177/178).
CONTOUR_NOTES = 7
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


_STOP = frozenset(stem(w.replace("ё", "е")) for w in kn.SEARCH_STOPWORDS) | frozenset(kn.SEARCH_STOPWORDS)


@dataclass(frozen=True)
class Term:
    """Значимое слово запроса и его написания в архиве (любое совпало — слово найдено)."""

    word: str
    alts: Tuple[str, ...]


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

    def exact(self, latin: str) -> Tuple[str, ...]:
        return tuple(self.by_key.get(sound_key(latin), ()))

    def prefix(self, latin: str) -> Tuple[str, ...]:
        key = sound_key(latin)
        hits = sorted((len(k), k) for k in self.by_key if k.startswith(key) and len(k) - len(key) <= _PREFIX_SLACK)
        return tuple(w for _n, k in hits for w in self.by_key[k])


_VOCAB_CACHE: Dict[int, Tuple[frozenset, _Vocab]] = {}


def _vocab(library: Any) -> _Vocab:
    words = library.vocabulary()
    cached = _VOCAB_CACHE.get(id(words))
    if cached is None or cached[0] is not words:
        _VOCAB_CACHE.clear()
        cached = _VOCAB_CACHE[id(words)] = (words, _Vocab(words))
    return cached[1]


def _concept(word_stem: str) -> Tuple[str, ...]:
    for key, query in kn.THEME_CONCEPTS.items():
        if word_stem.startswith(key):
            return tuple(query.split())
    return ()


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


def terms(library: Any, text: str) -> List[Term]:
    """Значимые слова запроса после алиасов библиотеки, без хвоста версии («V2.0») и служебных слов."""
    vocab = _vocab(library)
    out: List[Term] = []
    text = _STYLE_RE.sub(" ", strip_version_tail(_alias_normalize(text)).replace("ё", "е"))
    for word in _WORD_RE.findall(text):
        word_stem = stem(word)
        if word in _STOP or word_stem in _STOP:
            continue
        alts = _concept(word) or _concept(word_stem)
        if not alts and _CYR_RE.search(word):
            if len(word) < _WORD_MIN:
                continue
            alts = _resolve(vocab, word, word_stem)
        out.append(Term(word, alts if _CYR_RE.search(word) else alts or (word,)))
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


def consensus_order(hits: Sequence[Tuple[float, Dict[str, Any]]], limit: int = THEME_HOOKS) -> List[str]:
    """Первые ``limit`` найденных записей ``(confidence, запись с rtttl)`` — в порядке хуков темы (#3427, «консенсус
    версий»); набор тот же, меняется только порядок.

    Внутри одной доли слов: сначала мелодии, чей контур начала (:func:`rob_box_music.rtttl.contour`,
    :data:`CONTOUR_NOTES` нот) есть ещё хотя бы у одной найденной записи (считаются все ``hits``), — по одной версии
    на контур, затем их повторные версии, затем одиночные; при равенстве — порядок поиска (ближе к названию).
    Узнаваемая тема лежит в архиве в нескольких версиях («Terminator» theme_177/theme_178: d e f e c f), случайный
    рингтон с тем же словом в названии — в одной (terminat)."""
    contours = [contour(str(r.get("rtttl") or ""), CONTOUR_NOTES) for _c, r in hits]
    copies = Counter(c for c in contours if c is not None)
    versions: Counter = Counter()
    keys = []
    for i, ((confidence, _r), shape) in enumerate(zip(hits[:limit], contours)):
        single = shape is None or copies[shape] < 2
        keys.append((-confidence, single, 0 if single else versions[shape], i))
        versions[shape] += 1
    return [hits[i][1]["name"] for *_rank, i in sorted(keys)]


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


def _whole_search(library: Any, theme: str, limit: int, found_min: float = FOUND_MIN) -> ThemeHits:
    """Мелодии по словам темы целиком, лучшие первыми (#3427):

    1. точное совпадение названия записи (``title`` или ``name``, :func:`title_key`) с темой — всегда первым;
       тогда в набор идут ещё только записи, в опознавательных полях которых есть все слова темы, — совпадения по
       одному общему слову («remix», «give») отсекаются;
    2. затем — :func:`consensus_order` (версии одной мелодии выше одиночной записи).

    ``found_min`` — доля слов темы, которую покрывает запись (часть темы-перечисления — все слова, 1.0)."""
    query_terms = terms(library, theme)
    if not query_terms:
        return ThemeHits()
    pool = _pool(library, theme, query_terms, found_min)
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


def _genre_only(library: Any, part: str) -> Optional[str]:
    """Метка жанра, если часть темы называет только жанр («классическая музыка»), иначе ``None``."""
    words = {t.word for t in terms(library, part)} - set(kn.GENRE_FILLER)
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
    только из него, первая названная франшиза — первая часть (06.10)."""
    _head, colon, tail = theme.partition(":")
    listed = _split(library, tail) if colon else []
    if len(listed) >= 2:
        return listed
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
    ``missing`` (в лог), не подменяется. Ни одной мелодии — пусто, сет возьмёт пул по хешу темы."""
    whole = _whole_search(library, theme, limit)
    parts = [] if whole.exact else theme_parts(library, theme)
    if len(parts) < 2 and not (parts and _genre_only(library, parts[0])):
        return whole
    found = []
    missing = []
    for part in parts:
        tag = _genre_only(library, part)
        names = genre_hooks(library, tag) if tag else _whole_search(library, part, limit, found_min=1.0).names
        if names:
            found.append(names)
        else:
            missing.append(part)
    spanning = _whole_search(library, theme, limit, found_min=1.0).names if len(parts) >= 2 else ()
    if spanning and spanning not in found:  # запись темы целиком — первой; совпавшая с частью не дублируется
        found.insert(0, spanning)
    names = tuple(round_robin(found, THEME_LIST_HOOKS))
    return ThemeHits(names, False, tuple(missing), by_part(found, names))


def theme_hooks(library: Any, theme: str, limit: int = THEME_HOOKS) -> Tuple[str, ...]:
    """Имена мелодий темы (:func:`theme_search`)."""
    return theme_search(library, theme, limit).names


__all__ = ["CONTOUR_NOTES", "FOUND_MIN", "Found", "SEARCH_LIMIT", "THEME_HOOKS", "THEME_LIST_HOOKS", "Term",
           "ThemeHits", "by_part", "consensus_order", "coverage", "find", "genre_hooks", "genre_of", "ranked",
           "round_robin", "sound_key", "stem", "terms", "theme_hooks", "theme_parts", "theme_search", "title_key"]
