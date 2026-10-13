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
    "no_track": "сервис не вернул песню",
    "play_failed": "песня не заиграла",
    "internal": "что-то пошло не так",
    "no_key": "нет ключа сервиса генерации",
    "no_credits": "на счёте генерации нет кредитов",
    "budget_exhausted": "лимит генерации на сегодня вышел",
    "rate_limited": "сервис генерации просит подождать",
    "provider_down": "сервис генерации не отвечает",
    "rejected": "сервис генерации отклонил заказ",
    "timeout": "генерация не уложилась по времени",
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
    voice: str = ""  # голос, которым спета песня (``tts``-голос из кэша); «» — неизвестен


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
    voice: str = ""  # текущий голос робота: «спой своим голосом» берёт из кэша только песню, спетую им

    def without_generate(self, reason: str) -> "SourceContext":
        return replace(self, gen_failed=reason)


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
    return Option(f"{SYNTH}:{key}", {"synth_fit": fit, "theme_match": 1.0 if ask.genre else 0.0,
                                     "asked": -1.0 if ask.generated else 0.0, "wait_s": kn.INSTANT_WAIT_S},
                  "своё: синтез" + ("" if piece else f" «{kn.GEN_STYLES.get(str(key), kn.GEN_STYLES['club']).name}»"))


def _gen_option(ask: SourceAsk, ctx: SourceContext) -> Tuple[Optional[Option], str]:
    kind = "robot_song" if ask.robot_voice else "song" if ask.vocal else "instrumental"
    model = kn.GEN_MODELS[kn.DEFAULT_GEN_MODEL]
    reason = gen_block(ctx, model.credits_per_task)
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
    asked = -1.0 if ask.fresh else 0.5 if (ask.generated or ask.vocal or ask.own) else 0.0
    if not ask.vocal and not track.instrumental:  # петь не просили — песня хуже инструментала
        asked -= 0.25
    fresh = track.played_ago_h is None or track.played_ago_h >= kn.GEN_FRESH_HOURS
    return Option(f"{CACHE}:{track.id}", {"theme_match": theme, "asked": asked, "fresh": float(fresh),
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


def source_candidates(ask: SourceAsk, ctx: SourceContext, piece: bool = False) -> Tuple[Tuple[Option, ...], str]:
    """``(кандидаты, почему генерации нет)``. Порядок — синтез, генерация, кэш; звучащих нет — ``decline:<причина>``."""
    synth = _synth_option(ask, ctx, piece)
    gen, reason = _gen_option(ask, ctx)
    options = [o for o in (synth, gen) if o is not None] + ([] if piece else _cache_options(ask, ctx))
    return tuple(options) or (Option(f"{DECLINE}:{reason or 'no_source'}", {}, "честный отказ"),), reason


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


def _route(choice: Choice, ask: SourceAsk, ctx: SourceContext, reason: str) -> Route:
    kind, _sep, rest = choice.id.partition(":")
    track = next((t for t in ctx.cache if t.id == rest), None) if kind == CACHE else None
    return Route(kind, choice.id, ask, reason, rest if kind == SYNTH else "", track, provider=choice.provider)


def choose_route(ask: SourceAsk, ctx: SourceContext, chooser: Chooser, piece: bool = False) -> Route:
    """Кандидаты кода → ``chooser`` (один кандидат — без вопроса) → :class:`Route`."""
    options, reason = source_candidates(ask, ctx, piece)
    if len(options) == 1:
        return _route(Choice(options[0].id, 1.0, "code"), ask, ctx, reason)
    question = ChoiceQuestion(f"путь {ask.describe()}", options, weights=kn.SOURCE_WEIGHTS, instructions=INSTRUCTIONS)
    return _route(chooser.choose(question), ask, ctx, reason)


def choose_with_fallback(ask: SourceAsk, ctx: SourceContext, chooser: Chooser,
                         attempt: Optional[Callable[[Route], str]] = None, piece: bool = False) -> Route:
    """Выбор пути; выбрана генерация и ``attempt`` вернул причину провала (``""`` — вышло; исполнителя нет —
    ``not_wired``) — выбор заново с тем же контекстом без ``generate:*``, второго круга генерации нет (В3)."""
    route = choose_route(ask, ctx, chooser, piece)
    if route.kind != GENERATE:
        return route
    failed = attempt(route) if attempt is not None else "not_wired"
    if not failed:
        return route
    return replace(choose_route(ask, ctx.without_generate(failed), chooser, piece), failed=True)


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


def route_note(route: Route) -> str:
    """Фраза перед фразой запуска синтеза/пьесы: почему своё, а не то, что просили; просьба исполнена — «»."""
    excuse = _excuse(route)
    return f"{excuse[:-1]}, играю своё." if excuse and route.kind == SYNTH and route.style != PIECE else excuse


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


__all__ = ["CACHE", "CacheTrack", "DECLINE", "GENERATE", "INSTRUCTIONS", "PIECE", "REASON_WORDS", "Route", "SYNTH",
           "SourceAsk", "SourceContext", "ask_genre", "cache_text", "choose_route", "choose_with_fallback",
           "decline_text", "gen_block", "route_note", "source_ask", "source_candidates", "synth_fit", "synth_style"]
