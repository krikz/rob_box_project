"""Путь музыки: синтез / кэш / генерация — кандидаты кода, выбор, фразы (ADR-0158 §2.2, §2.6, §3.2; ADR-0148).

Живой прогон 13.10: «Робот, сгенерируй пиздатый музон своим голосом» → LLM ``request_music(intent=track)`` → синтез и
«Включаю музыку.» — тихая подмена: генерировать робот ещё не умеет и петь синтезом не может. Решает код:

1. :func:`source_ask` — слоты просьбы из слов человека (закрытые основы ``knowledge.SOURCE_ASK_STEMS``): «сгенерируй»,
   «спой»/«своим голосом», «новую», «сочини», жанр (``GEN_GENRE_WORDS``, окно «гранж», слова стилей синтеза).
2. :func:`source_candidates` — кандидаты с признаками ``SOURCE_WEIGHTS`` и **жёсткими фильтрами до выбора**:
   синтеза нет, если просили петь или ``Style.synth_fit`` < ``SYNTH_FIT_MIN``; ``generate:*`` нет без исполнителя
   генерации, ключа, живого провайдера, денег (баланс, лимиты дня и сета) и в режиме ``synth`` (:func:`gen_block`);
   кэш — по жанру и голосу. Звучащих нет — один ``decline:<причина>``.
3. Выбор — тот же ``choice.Chooser`` (``DeterministicChooser`` → цепочка Jev → LLM → кубик): модель видит только список
   кода. Провал генерации после выбора — :func:`choose_with_fallback` спрашивает ещё раз без ``generate:*`` (В3).
4. Фразу о выбранном строит код (:func:`route_note`, :func:`cache_text`, :func:`decline_text`): если играет не то, что
   просили, фраза называет и просьбу, и причину.

Модуль чистый (без сети и БД): кэш и деньги собирает ``rob_box_mcp_tools.engine.source_route``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, replace
from typing import Callable, List, Optional, Sequence, Tuple

from . import knowledge as kn
from .choice import Choice, ChoiceQuestion, Chooser, Option
from .theme import match_style_text

SYNTH, CACHE, GENERATE, DECLINE = "synth", "cache", "generate", "decline"
#: Синтез-кандидат одиночной пьесы по названию (наш материал: партитура/RTTTL, годность — ADR-0157).
PIECE = "piece"
#: Почему кандидатов ``generate:*`` нет / генерация не вышла → слова для фразы. Причины провайдера —
#: ``engine.gen.provider.REASONS``; ``not_wired``, ``mode_synth``, ``pending`` и причины песни — наши.
REASON_WORDS = {
    "not_wired": "генерация у меня ещё не подключена",
    "mode_synth": "генерация выключена",
    "pending": "песня ещё пишется",
    "silero_voice": "запасным голосом петь пока не умею",
    # песня голосом робота (``engine.gen.song``, ADR-0158 PR-7): наши причины провала
    "lyrics": "текст не сложился",
    "no_voice": "не знаю, каким голосом петь",
    "no_preset": "для этого голоса нет пресета пения",
    "vocal": "голос не спел текст",
    # вокал: ``vocal_`` + ``SingSynthError.reason`` (``rob_box_voice.sing_synth``); нет слов — общее «vocal»
    "vocal_no_api_key": "нет ключа голоса",
    "vocal_auth": "голос не принял ключ",
    "vocal_rate_limit": "голос просит подождать: лимит",
    "vocal_timeout": "голос не ответил вовремя",
    "vocal_network": "связь с голосом оборвалась",
    "vocal_rejected": "голос не принял текст",
    "no_track": "сервис не вернул песню",
    "play_failed": "песня не заиграла",
    "track_play_failed": "трек не заиграл",
    "internal": "что-то пошло не так",
    "no_key": "нет ключа сервиса генерации",
    "no_credits": "на счёте генерации нет кредитов",
    "budget_exhausted": "лимит генерации на сегодня вышел",
    "rate_limited": "сервис генерации просит подождать",
    "provider_down": "сервис генерации не отвечает",
    "rejected": "сервис генерации отклонил заказ",
    "timeout": "генерация не уложилась по времени",
    "no_material": "мелодии для живой версии у меня нет",
}
INSTRUCTIONS = ("Выбери, откуда взять музыку роботу: своё (синтез, сразу и бесплатно), сгенерированное раньше (кэш) "
                "или новая генерация (кредиты и ожидание). У кандидатов - очки кода в одной шкале; обычно лучший - с "
                "наибольшим итогом. Выбери один id.")


def _words(text: str) -> List[str]:
    return re.findall(r"[^\W\d_]+", (text or "").lower().replace("ё", "е"))


def _has(words: Sequence[str], stems: Sequence[str]) -> bool:
    return any(w.startswith(s.replace("ё", "е")) for w in words for s in stems)


@dataclass(frozen=True)
class SourceAsk:
    """Слоты просьбы о пути (ADR-0158 §2.2): ``generated`` «сгенерируй», ``vocal`` петь, ``robot_voice`` «своим
    голосом», ``fresh`` «новую», ``own`` «сочини», ``genre`` — ключ ``GEN_STYLES`` или «»."""

    generated: bool = False
    vocal: bool = False
    robot_voice: bool = False
    fresh: bool = False
    own: bool = False
    genre: str = ""

    def describe(self) -> str:
        flags = [name for name in ("generated", "vocal", "robot_voice", "fresh", "own") if getattr(self, name)]
        return f"asked=[{','.join(flags)}] genre={self.genre or '—'}"


def ask_genre(text: str) -> str:
    """Жанр просьбы: жанры без синтеза (``GEN_GENRE_WORDS``), окно стиля («гранж»), слово стиля синтеза; нет — «»."""
    words = _words(text)
    for stems in (kn.GEN_GENRE_WORDS, kn.WINDOW_WORDS):
        found = next((key for stem, key in stems.items() if key in kn.GEN_STYLES and _has(words, (stem,))), "")
        if found:
            return found
    style = match_style_text(text or "")
    return style if style in kn.GEN_STYLES else ""


def source_ask(text: str) -> SourceAsk:
    """Слоты просьбы о пути из слов человека (код, без LLM)."""
    words = _words(text)
    generated, own = _has(words, kn.SOURCE_ASK_STEMS["generated"]), _has(words, kn.SOURCE_ASK_STEMS["own"])
    robot_voice = any(w.startswith("голос") and words[i - 1].startswith(kn.ROBOT_VOICE_OWNERS)
                      for i, w in enumerate(words) if i)
    vocal = (_has(words, kn.SOURCE_ASK_STEMS["vocal"]) or robot_voice
             or (_has(words, kn.SONG_STEMS) and (generated or own)))
    return SourceAsk(generated, vocal, robot_voice, _has(words, kn.SOURCE_ASK_STEMS["fresh"]), own, ask_genre(text))


def synth_style(genre: str) -> Optional[str]:
    """Стиль синтеза, который играет жанр (без жанра — ``DEFAULT_STYLE``); синтеза нет — ``None``."""
    return kn.GEN_STYLES[genre].synth if genre else kn.DEFAULT_STYLE


def synth_fit(genre: str) -> float:
    style = synth_style(genre)
    return kn.STYLES[style].synth_fit if style else 0.0


@dataclass(frozen=True)
class CacheTrack:
    """Трек кэша генераций: ``group`` — варианты одной задачи (одна группа — один кандидат)."""

    id: str
    title: str
    kind: str = "instrumental"
    style_key: str = ""
    instrumental: bool = True
    rating: int = 0
    played_ago_h: Optional[float] = None
    group: str = ""
    work_id: str = ""      # кавер: каталожное название пьесы (ADR-0158 §2.5)
    instrument: str = ""   # кавер: ключ ``INSTRUMENTS`` или «»
    voice: str = ""  # голос, которым спета песня (``tts``-голос из кэша); «» — неизвестен
    prompt: str = ""  # теги заказа (``generated_tracks.prompt``): тема сета — по тегам темы (``GEN_THEME_TAGS``)
    seconds: float = 0.0  # фактическая длина файла (``duration_ms``); 0 — неизвестна


@dataclass(frozen=True)
class SourceContext:
    """Контекст пути (собирает код): кэш, готовность и деньги генерации, режим (ADR-0158 §2.2, §2.6, §2.8).
    ``gen_ready`` — исполнитель генерации подключён (нет — ``not_wired``); ``balance`` ``None`` — неизвестен (402
    провайдера приведёт к выбору заново); ``set_left`` ``None`` — не сет; ``gen_failed`` — генерация уже упала (В3)."""

    cache: Tuple[CacheTrack, ...] = ()
    gen_ready: bool = False
    has_key: bool = True
    provider_ok: bool = True
    balance: Optional[float] = None
    day_left: float = 0.0
    set_left: Optional[float] = None
    mode: str = "auto"
    gen_failed: str = ""
    cover_ready: bool = False  # исполнитель кавера пьесы подключён (ADR-0158 PR-4, ``engine.gen.cover``)
    voice: str = ""  # текущий голос робота: «спой своим голосом» берёт из кэша только песню, спетую им
    song_block: str = ""  # песню заказывать нельзя (``SongStudio.blocked``: Silero) — кавер это не касается

    def without_generate(self, reason: str) -> "SourceContext":
        return replace(self, gen_failed=reason)


def gen_kind(option_id: str) -> str:
    """Вид генерации кандидата ``generate:<вид>:<модель>`` (instrumental, song, robot_song, cover); иначе «»."""
    parts = option_id.split(":")
    return parts[1] if len(parts) == 3 and parts[0] == GENERATE else ""


def route_seconds(route: "Route", last: bool = False) -> float:
    """Сколько секунд сета занимает трек по выбранному пути (одно место, ADR-0148): длина файла — кавер
    ``COVER_DURATION_S``, прочая генерация ``GEN_SET_DURATION_S``, трек кэша — своя (неизвестна — как у генерации) —
    как её играет дека (``render.audio.set_track_seconds``: файл целиком, не последний трек за вычетом кроссфейда
    ``GEN_XFADE_S``); синтез — ``SET_TRACK_SECONDS``; отказ — 0."""
    from .render.audio import set_track_seconds

    if route.kind == GENERATE:
        return set_track_seconds(float(kn.COVER_DURATION_S if gen_kind(route.id) == "cover"
                                       else kn.GEN_SET_DURATION_S), last)
    if route.kind == CACHE:
        return set_track_seconds((route.track.seconds if route.track else 0.0) or float(kn.GEN_SET_DURATION_S), last)
    return float(kn.SET_TRACK_SECONDS) if route.kind == SYNTH else 0.0


def gen_block(ctx: SourceContext, cost: float) -> str:
    """Почему кандидата генерации нет (ключ :data:`REASON_WORDS`) или «» — генерировать можно."""
    checks = ((ctx.gen_failed, ctx.gen_failed), (ctx.mode == "synth", "mode_synth"), (not ctx.gen_ready, "not_wired"),
              (not ctx.has_key, "no_key"), (not ctx.provider_ok, "provider_down"),
              (ctx.balance is not None and ctx.balance < cost, "no_credits"),
              (ctx.day_left < cost or (ctx.set_left is not None and ctx.set_left < cost), "budget_exhausted"))
    return next((reason for blocked, reason in checks if blocked), "")


def _synth_option(ask: SourceAsk, ctx: SourceContext, piece: bool) -> Optional[Option]:
    fit = 1.0 if piece else synth_fit(ask.genre)
    if ask.vocal or ctx.mode == "generate" or fit < kn.SYNTH_FIT_MIN:
        return None
    key = PIECE if piece else synth_style(ask.genre)
    return Option(f"{SYNTH}:{key}", {"fallback": 1.0, "theme_match": 1.0 if ask.genre else 0.0,
                                     "asked": -1.0 if ask.generated else 0.0, "wait_s": kn.INSTANT_WAIT_S},
                  "своё: синтез" + ("" if piece else f" «{kn.GEN_STYLES.get(str(key), kn.GEN_STYLES['club']).name}»"))


def _gen_option(ask: SourceAsk, ctx: SourceContext) -> Tuple[Optional[Option], str]:
    kind = "robot_song" if ask.robot_voice else "song" if ask.vocal else "instrumental"
    model = kn.GEN_MODELS[kn.DEFAULT_GEN_MODEL]
    reason = ctx.song_block or gen_block(ctx, model.credits_per_task)
    if reason:
        return None, reason
    asked = ask.generated or ask.vocal or ask.fresh or ask.own
    return Option(f"{GENERATE}:{kind}:{kn.DEFAULT_GEN_MODEL}", {
        "theme_match": 1.0 if ask.genre else 0.0, "asked": 1.0 if asked else 0.0,
        "cost": model.credits_per_task, "wait_s": model.typical_s}, f"новая генерация ({kind})"), ""


def _theme(ask: SourceAsk, track: CacheTrack) -> Optional[float]:
    """Совпадение трека кэша с жанром просьбы: тот же — 1, та же семья синтеза (гранж — рок) — 0.5, чужой — ``None``."""
    if not ask.genre:
        return 0.0
    if track.style_key == ask.genre:
        return 1.0
    mine, asked = kn.GEN_STYLES.get(track.style_key), kn.GEN_STYLES[ask.genre]
    return 0.5 if mine is not None and mine.synth is not None and mine.synth == asked.synth else None


def _not_my_voice(ask: SourceAsk, track: CacheTrack, voice: str) -> bool:
    """«Своим голосом» — не песня голосом робота, или спета другим голосом, чем сейчас (ADR-0158 В8)."""
    return ask.robot_voice and (track.kind != "robot_song" or bool(voice and track.voice and track.voice != voice))


def _cache_option(ask: SourceAsk, track: CacheTrack, voice: str = "") -> Optional[Option]:
    """Кандидат трека кэша или ``None``: чужой жанр, «не нравится», петь просили — инструментал, «своим голосом» —
    не песня голосом робота (или не текущим)."""
    theme = _theme(ask, track)
    if theme is None or track.rating < 0 or (ask.vocal and track.instrumental) or _not_my_voice(ask, track, voice):
        return None
    asked = next((value for slot, value in kn.SOURCE_CACHE_ASKED if getattr(ask, slot)), 0.0)
    if not ask.vocal and not track.instrumental:  # петь не просили — песня хуже инструментала
        asked += kn.SOURCE_CACHE_SONG_PENALTY
    fresh = track.played_ago_h is None or track.played_ago_h >= kn.GEN_FRESH_HOURS
    return Option(f"{CACHE}:{track.id}", {"cache_theme": theme, "asked": asked, "fresh": float(fresh),
                                          "rating": float(track.rating), "wait_s": kn.INSTANT_WAIT_S},
                  f"сгенерированное раньше «{track.title}» ({track.kind}, {track.style_key or '—'})")


def _cache_options(ask: SourceAsk, ctx: SourceContext) -> List[Option]:
    """Кандидаты кэша: варианты одной задачи — один кандидат (первый в порядке кэша)."""
    seen, out = set(), []
    for track in ctx.cache:
        group = track.group or f"{track.kind}|{track.title}"
        option = None if group in seen else _cache_option(ask, track, ctx.voice)
        if option is not None:
            seen.add(group)
            out.append(option)
    return out


def set_cache(ask: SourceAsk, ctx: SourceContext, theme_tags: str = "") -> List[CacheTrack]:
    """Треки кэша для сета (ADR-0158 §2.7): инструментал жанра просьбы (тот же жанр или семья синтеза), не
    «не нравится», с тегами темы сета (``theme_tags``, «» — любая тема); давно не игранные первыми, затем свой жанр
    раньше семьи. Варианты одной задачи — все (соседями их не ставит ``engine.gen.gen_session``)."""
    def order(track: CacheTrack) -> Tuple[int, float, float]:
        fresh = track.played_ago_h is None or track.played_ago_h >= kn.GEN_FRESH_HOURS
        return (0 if fresh else 1, -(_theme(ask, track) or 0.0),
                -(track.played_ago_h if track.played_ago_h is not None else 1e9))

    usable = [t for t in ctx.cache if t.instrumental and t.kind == "instrumental" and t.rating >= 0
              and _theme(ask, t) is not None and theme_tags in t.prompt]
    return sorted(usable, key=order)


def theme_tags(theme: str) -> str:
    """EN-теги генерации темы сета (``GEN_THEME_TAGS``: «патриотический» → patriotic, anthemic…); нет строки — «»."""
    words = _words(theme)
    return next((tags for stems, tags in kn.GEN_THEME_TAGS.items() if _has(words, stems)), "")


def _slot_genre(token: str, mark: Optional[str]) -> str:
    """Ключ ``GEN_STYLES`` слова перечня: жанр без синтеза, окно («гранж»), слово целиком («клуб»), слово стиля."""
    for stems in (kn.GEN_GENRE_WORDS, kn.WINDOW_WORDS):
        found = next((key for stem, key in stems.items() if key in kn.GEN_STYLES and token.startswith(stem)), "")
        if found:
            return found
    found = kn.SET_STYLE_EXACT.get(token) or mark or ""
    return found if found in kn.GEN_STYLES else ""


def set_genres(text: str) -> Tuple[Tuple[str, ...], Tuple[str, ...]]:
    """Стили сета по порядку слов человека и непонятые слова перечня стилей (ADR-0148: слоты — код, не LLM).
    «в стилях клуб джангл рэйв синтвейв и джаз» → (club, dnb, rave, synthwave, jazz); «в стиле клуб и полька» —
    «полька» непонята. Перечень — после «стил…/жанр…» до знака препинания или слова-конца (``SET_STYLE_LIST_STOP``)."""
    from .theme import style_marks

    tokens = re.findall(r"[^\W_]+|[.,!?;:]", (text or "").lower().replace("ё", "е"))
    marks = style_marks(tokens)
    genres = [_slot_genre(tok, mark) for tok, mark in zip(tokens, marks)]
    unknown: List[str] = []
    head = next((i for i, tok in enumerate(tokens) if tok.startswith(kn.SET_STYLE_LIST_STEMS)), None)
    for i in range(len(tokens) if head is None else head + 1, len(tokens)):
        tok = tokens[i]
        if not tok.isalnum() or tok.isdigit() or tok.startswith(kn.SET_STYLE_LIST_STOP):
            break
        if not genres[i] and tok not in kn.SET_STYLE_LIST_JOIN:
            unknown.append(tok)
    return tuple(dict.fromkeys(g for g in genres if g)), tuple(unknown)


@dataclass(frozen=True)
class CoverWork:
    """Пьеса заказа для кавера (ADR-0158 §2.9, В14): каталожное название (``work_id`` кэша), наш материал мелодии
    (id мелодии RTTTL/партитуры, из которой собран лид; «» — мелодии нет, кавера нет), инструмент заказа (ключ
    ``INSTRUMENTS``), название для фразы."""

    title: str
    material: str
    instrument: str = ""
    spoken: str = ""


def cover_style_key(work: CoverWork, ask: SourceAsk) -> str:
    """Стиль кавера: жанр заказа; без жанра и инструмента — ``COVER_DEFAULT_STYLE``; с инструментом — «»."""
    return ask.genre or ("" if work.instrument else kn.COVER_DEFAULT_STYLE)


def _same_cover(track: CacheTrack, work: CoverWork, style: str) -> bool:
    from .works import norm

    return (track.kind == "cover" and norm(track.work_id) == norm(work.title) and track.instrument == work.instrument
            and track.style_key == style and track.rating >= 0)


def _cover_options(ask: SourceAsk, ctx: SourceContext, work: CoverWork) -> Tuple[List[Option], str]:
    """Кавер пьесы: трек кэша того же ``work_id``/``instrument``/``style`` (из вариантов — давнее игранный) и
    ``generate:cover`` — есть наш материал мелодии (В14, решение Шифу 14.10: «если у нас есть материал — идём через
    ветку кавер») и деньги (жёсткие фильтры :func:`gen_block`)."""
    style = cover_style_key(work, ask)
    same = sorted((t for t in ctx.cache if _same_cover(t, work, style)),
                  key=lambda t: -1e9 if t.played_ago_h is None else -t.played_ago_h)
    out = [Option(f"{CACHE}:{t.id}", {"live": 1.0, "theme_match": 1.0, "asked": -1.0 if ask.fresh else 0.0,
                                      "fresh": float(t.played_ago_h is None or t.played_ago_h >= kn.GEN_FRESH_HOURS),
                                      "rating": float(t.rating), "wait_s": kn.INSTANT_WAIT_S},
                  f"кавер из сочинённого раньше «{t.title}»") for t in same[:1]]
    model = kn.GEN_MODELS[kn.COVER_MODEL]
    reason = ("" if work.material else "no_material") or gen_block(
        replace(ctx, gen_ready=ctx.cover_ready), model.credits_per_task)
    if not reason:
        out.append(Option(f"{GENERATE}:cover:{kn.COVER_MODEL}", {
            "live": 1.0, "asked": 1.0 if (ask.generated or ask.fresh) else 0.0, "cost": model.credits_per_task,
            "wait_s": kn.COVER_WAIT_S}, f"живая версия: кавер нашей мелодии ({style or work.instrument})"))
    return out, reason


def source_candidates(ask: SourceAsk, ctx: SourceContext, piece: bool = False,
                      work: Optional[CoverWork] = None) -> Tuple[Tuple[Option, ...], str]:
    """``(кандидаты, почему генерации нет)``. Порядок — синтез, генерация, кэш; звучащих нет — ``decline:<причина>``.
    Пьеса с ``work`` (петь не просили) — вместо генерации кандидаты кавера (:func:`_cover_options`, ADR-0158 PR-4)."""
    synth = _synth_option(ask, ctx, piece)
    if piece and work is not None and not ask.vocal:
        covers, reason = _cover_options(ask, ctx, work)
        options = [o for o in [synth, *covers] if o is not None]
    else:
        gen, reason = _gen_option(ask, ctx)
        options = [o for o in (synth, gen) if o is not None] + ([] if piece else _cache_options(ask, ctx))
    return tuple(options) or (Option(f"{DECLINE}:{reason or 'no_source'}", {}, "честный отказ"),), reason


def set_candidates(ask: SourceAsk, ctx: SourceContext,
                   work: Optional[CoverWork] = None) -> Tuple[Tuple[Option, ...], str]:
    """Кандидаты трека сета (ADR-0158 §2.7, В14): у темы есть наш материал ``work`` и кавер возможен (кэш того же
    кавера или ``generate:cover`` при деньгах) — кавер и синтез-запас; инструментал по тегам (новый и из кэша) не
    кандидат: узнаваемость в приоритете (Шифу 14.10: «кавер в приоритете»). Иначе — как у трека без материала."""
    options, reason = source_candidates(ask, ctx)
    if work is None or ask.vocal:
        return options, reason
    covers, why = _cover_options(ask, ctx, work)
    if not covers:
        return options, why if why != "no_material" else reason
    return tuple(covers + [o for o in options if o.id.startswith(SYNTH)]), why


@dataclass(frozen=True)
class Route:
    """Выбранный путь: ``kind`` (synth/cache/generate/decline), ``id`` кандидата, просьба, почему генерации нет,
    стиль синтеза, трек кэша, ``failed`` — генерация была выбрана и не вышла (В3)."""

    kind: str
    id: str
    ask: SourceAsk
    gen_reason: str = ""
    style: str = ""
    track: Optional[CacheTrack] = None
    failed: bool = False
    provider: str = ""
    work: Optional[CoverWork] = None


def _route(choice: Choice, ask: SourceAsk, ctx: SourceContext, reason: str,
           work: Optional[CoverWork] = None) -> Route:
    kind, _sep, rest = choice.id.partition(":")
    track = next((t for t in ctx.cache if t.id == rest), None) if kind == CACHE else None
    return Route(kind, choice.id, ask, reason, rest if kind == SYNTH else "", track, provider=choice.provider,
                 work=work)


def choose_route(ask: SourceAsk, ctx: SourceContext, chooser: Chooser, piece: bool = False,
                 work: Optional[CoverWork] = None, for_set: bool = False) -> Route:
    """Кандидаты кода (трек сета — :func:`set_candidates`) → ``chooser`` (один кандидат — без вопроса) →
    :class:`Route`."""
    options, reason = set_candidates(ask, ctx, work) if for_set else source_candidates(ask, ctx, piece, work)
    if len(options) == 1:
        return _route(Choice(options[0].id, 1.0, "code"), ask, ctx, reason, work)
    question = ChoiceQuestion(f"путь {ask.describe()}", options, weights=kn.SOURCE_WEIGHTS, instructions=INSTRUCTIONS)
    return _route(chooser.choose(question), ask, ctx, reason, work)


def choose_with_fallback(ask: SourceAsk, ctx: SourceContext, chooser: Chooser,
                         attempt: Optional[Callable[[Route], str]] = None, piece: bool = False,
                         work: Optional[CoverWork] = None) -> Route:
    """Выбор пути; выбрана генерация и ``attempt`` вернул причину провала (``""`` — вышло; исполнителя нет —
    ``not_wired``) — выбор заново с тем же контекстом без ``generate:*``, второго круга генерации нет (В3)."""
    route = choose_route(ask, ctx, chooser, piece, work)
    if route.kind != GENERATE:
        return route
    failed = attempt(route) if attempt is not None else "not_wired"
    if not failed:
        return route
    return replace(choose_route(ask, ctx.without_generate(failed), chooser, piece, work), failed=True)


def _asked_text(ask: SourceAsk) -> str:
    return ("спеть своим голосом" if ask.robot_voice else "спеть" if ask.vocal
            else "сгенерировать" if ask.generated else "")


def _poor_genre(ask: SourceAsk) -> str:
    """Имя жанра просьбы, который синтез не играет («гранж»), иначе «»."""
    return kn.GEN_STYLES[ask.genre].name if ask.genre and synth_fit(ask.genre) < kn.SYNTH_FIT_MIN else ""


def _excuse(route: Route, tail: str = ".") -> str:
    """Почему играет не то, что просили: «Сгенерировать сейчас не могу: <причина>.»; просьба исполнима — «»."""
    words = REASON_WORDS.get(route.gen_reason, route.gen_reason)
    if route.failed:
        return f"Сгенерировать не вышло: {words}{tail}"
    asked, genre = _asked_text(route.ask), _poor_genre(route.ask)
    if not words:
        return ""
    if asked:
        return f"{asked[:1].upper()}{asked[1:]} сейчас не могу: {words}{tail}"
    return f"{genre[:1].upper()}{genre[1:]} синтезом у меня не выходит, а {words}{tail}" if genre else ""


def _cover_what(work: CoverWork, ask: SourceAsk) -> str:
    """«на ситаре» / «в стиле «оркестр»» — чем будет живая версия (из таблиц, не от LLM)."""
    ins = kn.INSTRUMENTS.get(work.instrument)
    style = kn.GEN_STYLES.get(cover_style_key(work, ask))
    return f"на {ins.prepositional}" if ins else f"в стиле «{style.name}»" if style else ""


def cover_note(route: Route) -> str:
    """Фраза пути пьесы с кавером (В4, ADR-0148: от кода): мост — «живую готовлю, через ~N с, а пока своя»; кавера
    нет из-за денег/сервиса — «живую сейчас не сделаю: <причина>»; нашей мелодии нет — «»."""
    work = route.work
    if work is None or route.ask.vocal:
        return ""
    what = " ".join(p for p in ("Живую версию", _cover_what(work, route.ask)) if p)
    if route.kind == GENERATE:
        return f"{what} готовлю — секунд через {int(kn.COVER_WAIT_S)}, а пока своя."
    if route.kind == SYNTH and work.material and route.gen_reason:
        words = REASON_WORDS.get(route.gen_reason, route.gen_reason)
        return f"{what} {'сделать не вышло' if route.failed else 'сейчас не сделаю'}: {words}."
    return ""


def cover_failed_text(work: CoverWork, reason: str) -> str:
    """Генерация кавера упала, мост доигрывает (В3): фраза кода с причиной, без тихой подмены."""
    return f"Живой версии «{work.spoken or work.title}» не будет: {REASON_WORDS.get(reason, reason)}."


def cover_ready_text(work: CoverWork, ask: SourceAsk) -> str:
    """Кавер сгенерирован и встал на деку вместо моста."""
    tail = _cover_what(work, ask)
    return f"А вот живая версия «{work.spoken or work.title}»{' ' + tail if tail else ''}."


#: Причины, о которых синтез-запас молчит: генерации нет совсем или её выключили (просили своё).
QUIET_REASONS: Tuple[str, ...] = ("", "not_wired", "mode_synth")


def fallback_note(reason: str) -> str:
    """Синтез — запас (В13): генерация была бы, но нельзя — «Сгенерировать сейчас не могу: <причина>, играю своё.»;
    генерации нет совсем — «»."""
    if reason in QUIET_REASONS:
        return ""
    return f"Сгенерировать сейчас не могу: {REASON_WORDS.get(reason, reason)}, играю своё."


def route_note(route: Route) -> str:
    """Фраза перед фразой запуска синтеза/пьесы: почему своё, а не то, что просили или что шло бы по умолчанию
    (генерация, В13); просьба исполнена — «»."""
    if route.work is not None:
        return cover_note(route)
    excuse = _excuse(route)
    if route.kind != SYNTH or route.style == PIECE:
        return excuse
    return f"{excuse[:-1]}, играю своё." if excuse else fallback_note(route.gen_reason)


def cache_text(route: Route) -> str:
    """Фраза запуска трека кэша (вся реплика): причина, если просили новое, и что ставим."""
    track = route.track
    title = f"«{track.title}»" if track else "трек"
    what = {"robot_song": "спетое раньше своим голосом", "song": "песню из сочинённого раньше",
            "cover": "кавер из сочинённого раньше"}.get(track.kind if track else "", "сочинённое раньше")
    return " ".join(p for p in (_excuse(route), f"Ставлю {what}: {title}.") if p)


def decline_text(route: Route) -> str:
    """Честный отказ (звука нет): что просили, почему нельзя, что могу вместо (стили, которые синтез играет)."""
    offer = ", ".join(st.name for key, st in kn.GEN_STYLES.items()
                      if st.synth == key and kn.STYLES[key].synth_fit >= kn.SYNTH_FIT_MIN)
    excuse = _excuse(route, ", а синтез не поёт." if route.ask.vocal else ".")
    head = excuse or f"Сыграть это сейчас не могу: {REASON_WORDS.get(route.gen_reason, 'нечем')}."
    return f"{head} Могу сыграть своё: {offer}."


__all__ = ["CACHE", "CacheTrack", "CoverWork", "DECLINE", "cover_failed_text", "cover_note", "cover_ready_text",
           "cover_style_key", "GENERATE", "INSTRUCTIONS", "PIECE", "REASON_WORDS", "Route", "SYNTH",
           "SourceAsk", "SourceContext", "ask_genre", "cache_text", "choose_route", "choose_with_fallback",
           "decline_text", "fallback_note", "gen_block", "gen_kind", "QUIET_REASONS", "route_note", "route_seconds",
           "set_cache",
           "set_candidates", "set_genres", "source_ask", "source_candidates", "synth_fit", "synth_style", "theme_tags"]
