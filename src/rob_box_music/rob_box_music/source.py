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
from typing import Callable, List, Optional, Sequence, Set, Tuple

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
    "theme_cache": "под тему хватает сочинённого раньше",
    "engine_cannot": "названный движок так не играет",
}
INSTRUCTIONS = ("Выбери, откуда взять музыку роботу: своё (синтез, сразу и бесплатно), сгенерированное раньше (кэш) "
                "или новая генерация (кредиты и ожидание). У кандидатов - очки кода в одной шкале; обычно лучший - с "
                "наибольшим итогом. Выбери один id.")


_WORD_RE = re.compile(r"[^\W\d_]+")


def _spans(text: str) -> List[Tuple[str, int, int]]:
    """Слова речи в нижнем регистре без «ё» и их позиции ``(слово, начало, конец)`` в исходной строке."""
    return [(m.group().lower().replace("ё", "е"), m.start(), m.end()) for m in _WORD_RE.finditer(text or "")]


def _words(text: str) -> List[str]:
    return [word for word, _start, _end in _spans(text)]


def _has(words: Sequence[str], stems: Sequence[str]) -> bool:
    return any(w.startswith(s.replace("ё", "е")) for w in words for s in stems)


@dataclass(frozen=True)
class SourceAsk:
    """Слоты просьбы о пути (ADR-0158 §2.2): ``generated`` «сгенерируй», ``vocal`` петь, ``robot_voice`` «своим
    голосом», ``fresh`` «новую», ``own`` «сочини», ``genre`` — ключ ``GEN_STYLES`` или «», ``engine`` — ключ
    ``MUSIC_ENGINES``, названный человеком («на Ренардо», «на синтезаторе», «вживую»), или «»; ``tweak`` — просьба
    правит играющее («ускорь», «добавь баса», «развивай»: ``ENGINE_TWEAK_WORDS``), а не заказывает другое."""

    generated: bool = False
    vocal: bool = False
    robot_voice: bool = False
    fresh: bool = False
    own: bool = False
    genre: str = ""
    engine: str = ""
    tweak: bool = False

    def describe(self) -> str:
        flags = [name for name in ("generated", "vocal", "robot_voice", "fresh", "own") if getattr(self, name)]
        return f"asked=[{','.join(flags)}] genre={self.genre or '—'}" + (f" engine={self.engine}" if self.engine else "")

    @property
    def asks_path(self) -> bool:
        """Просьба называет иной путь, чем «играй сам»: «сгенерируй», «спой», «своим голосом», «новую», «сочини»."""
        return self.generated or self.vocal or self.robot_voice or self.fresh or self.own

    @property
    def plain(self) -> bool:
        """Просьба не называет путь: ни «сгенерируй»/«спой»/«новую»/«сочини», ни жанра («ускорь», «добавь баса»)."""
        return not (self.asks_path or self.genre)


def ask_genre(text: str) -> str:
    """Жанр просьбы: жанры без синтеза (``GEN_GENRE_WORDS``), окно стиля («гранж»), слово стиля синтеза; нет — «»."""
    words = _words(text)
    for stems in (kn.GEN_GENRE_WORDS, kn.WINDOW_WORDS):
        found = next((key for stem, key in stems.items() if key in kn.GEN_STYLES and _has(words, (stem,))), "")
        if found:
            return found
    style = match_style_text(text or "")
    return style if style in kn.GEN_STYLES else ""


def _phrase_at(words: Sequence[str], i: int, parts: Sequence[str]) -> bool:
    """Слова ``parts`` стоят в речи подряд, точно, начиная с позиции ``i`` (запись из нескольких слов таблицы)."""
    return list(words[i:i + len(parts)]) == list(parts)


def _has_phrase(words: Sequence[str], entry: str) -> bool:
    """Запись таблицы в речи: одно слово — основа (начало слова), несколько — эти слова подряд, точно."""
    parts = entry.split()
    if len(parts) == 1:
        return _has(words, parts)
    return any(_phrase_at(words, i, parts) for i in range(len(words)))


def ask_tweak(text: str) -> bool:
    """Просьба правит играющее: в словах есть запись ``ENGINE_TWEAK_WORDS`` («ускорь», «добавь баса», «в том же духе»)."""
    words = _words(text)
    return any(_has_phrase(words, entry) for group in kn.ENGINE_TWEAK_WORDS.values() for entry in group)


def _names_engine(engine: kn.MusicEngine, word: str) -> bool:
    """Слово речи называет движок по имени: начинается с основы или содержит ядро (слово-двойник ядром не считается)."""
    if word.startswith(engine.stems):
        return True
    return not word.startswith(engine.lookalikes) and any(core in word for core in engine.cores)


def _straddles(glued: str, split: int, piece: str) -> bool:
    """``piece`` стоит в склейке ``glued`` через стык ``split`` (часть в первом слове, часть во втором)."""
    start = glued.find(piece)
    while start != -1:
        if start < split < start + len(piece):
            return True
        start = glued.find(piece, start + 1)
    return False


def _joined_names_engine(engine: kn.MusicEngine, first: str, second: str) -> bool:
    """Соседние слова, склеенные, называют движок через стык («ре нардо», «нарен дарда»): STT разрезал имя. Основа или
    ядро обязаны заходить во второе слово — имя, целиком лежащее в первом, пару не берёт (лишний сосед не съедается);
    слово-двойник склейку не даёт."""
    if first.startswith(engine.lookalikes) or second.startswith(engine.lookalikes):
        return False
    glued = first + second
    return (any(glued.startswith(stem) and len(stem) > len(first) for stem in engine.stems)
            or any(_straddles(glued, len(first), core) for core in engine.cores))


def _meaning_hits(engine: kn.MusicEngine, words: Sequence[str]) -> List[Tuple[int, int]]:
    """Слова по смыслу в речи (``MusicEngine.words``): ``(первое слово, слово после записи)``. Запись не берётся, если
    следом слово из её списка: «на синт» перед «вейв» — составной стиль."""
    hits: List[Tuple[int, int]] = []
    for i in range(len(words)):
        for entry, unless in engine.words.items():
            parts = entry.split()
            end = i + len(parts)
            if _phrase_at(words, i, parts) and (words[end] if end < len(words) else "") not in unless:
                hits.append((i, end))
                break
    return hits


def _name_hits(engine: kn.MusicEngine, words: Sequence[str]) -> List[Tuple[int, int]]:
    """Имя движка в речи (основы, ядра, склейка пары слов): ``(первое слово, слово после имени)``. Пары первыми: в «ре
    нардо» второе слово само ядро, но «ре» — часть имени."""
    pairs = [(i, i + 2) for i in range(len(words) - 1) if _joined_names_engine(engine, words[i], words[i + 1])]
    return pairs + [(i, i + 1) for i, word in enumerate(words) if _names_engine(engine, word)]


def engine_words(text: str, by_meaning: Optional[bool] = None) -> List[Tuple[str, int, int]]:
    """Слова речи, называющие движок: ``(ключ, начало, конец)`` в исходной строке. Единственное место, где код узнаёт
    движок в услышанной фразе (ADR-0148): по таблице ``MUSIC_ENGINES`` — имя (основы, ядра, склейка пары слов) и слова
    по смыслу. Слова по смыслу берутся, только пока просьба не называет иной путь (``SourceAsk.asks_path``: «спой
    вживую» — спеть); ``by_meaning`` решает за просьбу, когда её путь уже известен вызывающему (тул «сгенерируй»)."""
    tokens = _spans(text)
    words = [word for word, _start, _end in tokens]
    if by_meaning is None:
        by_meaning = not _path_ask(text).asks_path
    hits: List[Tuple[str, int, int]] = []
    taken: Set[int] = set()
    for key, engine in kn.MUSIC_ENGINES.items():
        for first, end in [*(_meaning_hits(engine, words) if by_meaning else []), *_name_hits(engine, words)]:
            if not taken & set(range(first, end)):
                hits.append((key, tokens[first][1], tokens[end - 1][2]))
                taken.update(range(first, end))
    return sorted(hits, key=lambda hit: hit[1])


def ask_engine(text: str, by_meaning: Optional[bool] = None) -> str:
    """Движок, названный в речи (``MUSIC_ENGINES``): имя «Ренардо» и искажения STT, в том числе слитые из двух слов, или
    слова по смыслу «на синтезаторе», «вживую» (:func:`engine_words`); не назван — «»."""
    found = engine_words(text, by_meaning)
    return found[0][0] if found else ""


def blank_engine(text: str) -> str:
    """Текст без слов, называющих движок (на их месте пробелы той же длины): имя движка — не тема трека."""
    out = text or ""
    for _key, start, end in engine_words(out):
        out = out[:start] + " " * (end - start) + out[end:]
    return out


def engine_route(engine: str) -> str:
    """Путь, который играет названный движок (``SYNTH`` у синтеза); движок не назван — «»."""
    known = kn.MUSIC_ENGINES.get(engine)
    return known.route if known is not None else ""


def settle_engine(ask: SourceAsk, slot: Optional[str], playing: str = "",
                  own_subject: bool = False) -> Tuple[SourceAsk, str]:
    """Движок заказа: ``(просьба, строка для лога)``. Решает код по словам человека и снимку плеера, а не LLM (#4013).

    * Слова называют движок (:func:`ask_engine`) — он и есть; слот ``engine`` схемы (его заполнила LLM) ничего не
      прибавляет: слот, которого слова не подтверждают («сыграй на балалайке» + ``engine=synth``), отбрасывается
      с причиной в строке, иначе молчаливое сужение пути до синтеза (кэш и генерация не рассматриваются).
    * Не названного движка заказ наследует играющий (``playing``, снимок плеера) только когда просьба **правит
      играющее** (``SourceAsk.tweak``, ``ENGINE_TWEAK_WORDS``: «ускорь», «добавь баса», «развивай» под синтезом — движок
      тот же), не называет путь (``SourceAsk.plain``) и не называет своего предмета (``own_subject``: пьеса по
      названию или инструмент реплики). Всякая прочая просьба — «сыграй ещё», «включи музыку», «поставь Нирвану»,
      «про космос», «что-нибудь другое», громкость — идёт обычным путём: кавер и кэш остаются кандидатами, синтез
      до стопа не липнет.

    Строка пуста, когда добавить нечего."""
    unsupported = slot in kn.MUSIC_ENGINES and slot != ask.engine  # слова человека не называют этот движок
    note = f"слот engine={slot} без опоры в словах человека — отброшен" if unsupported else ""
    if ask.engine or playing not in kn.MUSIC_ENGINES or own_subject or not ask.plain or not ask.tweak:
        return ask, note
    return replace(ask, engine=playing), "; ".join(filter(None, (note, f"движок наследуется от играющего: {playing}")))


def _path_ask(text: str) -> SourceAsk:
    """Слоты пути, который просят словами: «сгенерируй», «спой», «своим голосом», «новую», «сочини» (жанр, движок и
    правка играющего — в :func:`source_ask`)."""
    words = _words(text)
    generated, own = _has(words, kn.SOURCE_ASK_STEMS["generated"]), _has(words, kn.SOURCE_ASK_STEMS["own"])
    robot_voice = any(w.startswith("голос") and words[i - 1].startswith(kn.ROBOT_VOICE_OWNERS)
                      for i, w in enumerate(words) if i)
    vocal = (_has(words, kn.SOURCE_ASK_STEMS["vocal"]) or robot_voice
             or (_has(words, kn.SONG_STEMS) and (generated or own)))
    return SourceAsk(generated, vocal, robot_voice, _has(words, kn.SOURCE_ASK_STEMS["fresh"]), own)


def source_ask(text: str) -> SourceAsk:
    """Слоты просьбы о пути из слов человека (код, без LLM)."""
    return replace(_path_ask(text), genre=ask_genre(text), engine=ask_engine(text), tweak=ask_tweak(text))


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
    named = engine_route(ask.engine) == SYNTH  # «на синтезаторе»/«на Ренардо»: годность стиля не отговорка
    fit = 1.0 if piece or named else synth_fit(ask.genre)
    if ask.vocal or ctx.mode == "generate" or fit < kn.SYNTH_FIT_MIN:
        return None
    key = PIECE if piece else synth_style(ask.genre) if synth_fit(ask.genre) >= kn.SYNTH_FIT_MIN else kn.DEFAULT_STYLE
    return Option(f"{SYNTH}:{key}", {"fallback": 1.0, "theme_match": 1.0 if ask.genre else 0.0,
                                     "asked": 1.0 if named else -1.0 if ask.generated else 0.0,
                                     "wait_s": kn.INSTANT_WAIT_S},
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


def _theme_stems(text: str) -> Set[str]:
    return {w[:kn.THEME_STEM_LEN] for w in _words(text)
            if len(w) >= kn.THEME_WORD_MIN and not w.startswith(kn.THEME_FIT_STOP)}


def theme_fit(theme_tags: str, track: CacheTrack) -> float:
    """Насколько трек кэша по теме сета (#3775, решение Шифу 21.10 «подогнать под тему что есть»): метка темы целиком
    в ``prompt`` — 1; иначе доля основ слов темы (``theme_tags``), которые есть в ``prompt``, названии и имени стиля
    трека. Темы нет — 0. По теме — от ``THEME_FIT_MIN``."""
    if not theme_tags:
        return 0.0
    if theme_tags in track.prompt:
        return 1.0
    want = _theme_stems(theme_tags)
    style = kn.GEN_STYLES.get(track.style_key)
    have = _theme_stems(" ".join((track.prompt, track.title, style.name if style else "")))
    return len(want & have) / len(want) if want else 0.0


def theme_cache(ask: SourceAsk, ctx: SourceContext, theme_tags: str) -> List[CacheTrack]:
    """Треки кэша сета (:func:`set_cache` без фильтра темы) по теме (:func:`theme_fit` ≥ ``THEME_FIT_MIN``): ближе к
    теме — раньше, при равных — порядок :func:`set_cache`."""
    fits = [(theme_fit(theme_tags, t), t) for t in set_cache(ask, ctx)]
    ranked = sorted(((f, i, t) for i, (f, t) in enumerate(fits) if f >= kn.THEME_FIT_MIN), key=lambda x: (-x[0], x[1]))
    return [t for _f, _i, t in ranked]


def _clip(text: str) -> str:
    """Слова ``text`` через пробел, не длиннее ``GEN_THEME_MAX_CHARS`` (режется по слову)."""
    out = ""
    for word in text.split():
        if len(out) + len(word) + (1 if out else 0) > kn.GEN_THEME_MAX_CHARS:
            break
        out = f"{out} {word}" if out else word
    return out


def theme_tags(theme: str) -> str:
    """Тема в тегах заказа Suno и её метка в кэше (``CacheTrack.prompt``, фильтр :func:`set_cache`): EN-теги строки
    ``GEN_THEME_TAGS`` («патриотический» → patriotic, anthemic…); строки нет — слова темы как есть («пленников
    прибывших эскобар», #3775: без них сет на свою тему брал любой кэш стиля, а заказ уходил без темы).
    Темы нет — «»."""
    words = _words(theme)
    found = next((tags for stems, tags in kn.GEN_THEME_TAGS.items() if _has(words, stems)), "")
    return found or _clip(" ".join(words))


def theme_title(theme: str) -> str:
    """Название заказа по теме человека («Пленников прибывших эскобар»); темы нет — «» (название решает заказ)."""
    text = _clip(" ".join(re.findall(r"[^\W_]+", theme or "")))
    return text[:1].upper() + text[1:]


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
    if engine_route(ask.engine) == SYNTH:  # движок назван: кэш и генерация не кандидаты, молчаливой подмены нет
        return ((synth,), "") if synth else ((Option(f"{DECLINE}:engine_cannot", {}, "честный отказ"),), "engine_cannot")
    if piece and work is not None and not ask.vocal:
        covers, reason = _cover_options(ask, ctx, work)
        options = [o for o in [synth, *covers] if o is not None]
    else:
        gen, reason = _gen_option(ask, ctx)
        options = [o for o in (synth, gen) if o is not None] + ([] if piece else _cache_options(ask, ctx))
    return tuple(options) or (Option(f"{DECLINE}:{reason or 'no_source'}", {}, "честный отказ"),), reason


def _not_replayed(options: Sequence[Option], ctx: SourceContext) -> Tuple[Option, ...]:
    """Трек сета (В16): есть новый заказ — трек кэша, игранный ближе ``GEN_FRESH_HOURS``, не кандидат. Признак
    ``fresh`` (вес 0.5) его не опускал: у кэша кавера ``theme_match`` 1.5 — кэш выигрывал при любом прослушивании,
    и сет 14.10 повторил 4 трека сета часом раньше. Нового заказа нет — сыгранное лучше синтеза, остаётся."""
    if not any(o.id.startswith(GENERATE) for o in options):
        return tuple(options)
    recent = {f"{CACHE}:{t.id}" for t in ctx.cache
              if t.played_ago_h is not None and t.played_ago_h < kn.GEN_FRESH_HOURS}
    return tuple(o for o in options if o.id not in recent)


def set_candidates(ask: SourceAsk, ctx: SourceContext,
                   work: Optional[CoverWork] = None) -> Tuple[Tuple[Option, ...], str]:
    """Кандидаты трека сета (ADR-0158 §2.7, В14): у темы есть наш материал ``work`` и кавер возможен (кэш того же
    кавера или ``generate:cover`` при деньгах) — кавер и синтез-запас; инструментал по тегам (новый и из кэша) не
    кандидат: узнаваемость в приоритете (Шифу 14.10: «кавер в приоритете»). Иначе — как у трека без материала.
    Недавно игранный кэш при новом заказе — не кандидат (:func:`_not_replayed`)."""
    options, reason = source_candidates(ask, ctx)
    if work is None or ask.vocal or ask.engine:
        return _not_replayed(options, ctx), reason
    covers, why = _cover_options(ask, ctx, work)
    if not covers:
        return _not_replayed(options, ctx), why if why != "no_material" else reason
    return _not_replayed(covers + [o for o in options if o.id.startswith(SYNTH)], ctx), why


#: Ключ слота сета «кавер произведения темы в стиле трека» (В14, В16): ``cover|<стиль>|<произведение>`` — у каждого
#: произведения свой слот, второй вариант его задачи закрывает только его же трек; инструментал — ключ = стиль.
COVER_SLOT = "cover|"


def cover_slot(genre: str, title: str) -> str:
    """Ключ слота кавера произведения ``title`` (``CoverWork.title`` / ``work_id`` кэша) в стиле ``genre``."""
    from .works import norm

    return f"{COVER_SLOT}{genre}|{norm(title)}"


def slot_style(key: str) -> str:
    """Стиль слота сета: ``cover|<стиль>|…`` → стиль; ключ инструментала — сам стиль."""
    return key[len(COVER_SLOT):].split("|")[0] if key.startswith(COVER_SLOT) else key


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


def cover_rejected_text(work: CoverWork, others: bool) -> str:
    """Suno не взял ни одну мелодию произведения трека сета (В17): пьесу называет код; дальше — другие произведения
    темы (``others``) или сет без неё."""
    return f"«{work.spoken or work.title}» сервис не взял — {'играю другие' if others else 'доиграю без него'}."


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


__all__ = ["CACHE", "COVER_SLOT", "CacheTrack", "CoverWork", "DECLINE", "cover_failed_text", "cover_note",
           "cover_rejected_text",
           "cover_ready_text", "cover_slot", "cover_style_key", "slot_style", "GENERATE", "INSTRUCTIONS", "PIECE",
           "REASON_WORDS", "Route", "SYNTH",
           "SourceAsk", "SourceContext", "ask_genre", "ask_engine", "ask_tweak", "blank_engine", "cache_text", "choose_route", "choose_with_fallback",
           "decline_text", "engine_route", "engine_words", "fallback_note", "gen_block", "gen_kind", "QUIET_REASONS", "route_note", "route_seconds",
           "set_cache",
           "set_candidates", "set_genres", "settle_engine", "source_ask", "source_candidates", "synth_fit", "synth_style", "theme_tags",
           "theme_cache", "theme_fit", "theme_title"]
