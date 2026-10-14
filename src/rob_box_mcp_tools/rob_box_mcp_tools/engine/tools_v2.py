"""Узкие MCP-тулы движка v2 (ADR-0149 §5.1): ``dj_set(start|stop)`` (PR-5), ``request_music`` (PR-6).

Регистрируются ``mcp_server._attach_player_owner_v2``; старый путь (``compose_music`` & Co.)
удалён вместе с флагом выбора движка (PR-13/PR-15).
Параметры трека (темп, тоника, синты, сид) тулы не принимают: тема → мелодии по её словам
(``engine.search.theme_search``, #3399, #3427) → ``theme.seeded_profile``
→ ``set_plan.seeded_plan`` → ``arrange.compose`` → ``render``. Вызывают их роутер медиакоманд
(без LLM) и LLM.

Результат ``ok: true`` — только когда плеер прислал ``started`` этого трека (``confirm``, A14:
ни одного ``ok:true`` без звука); ``rejected`` или тишина за время ожидания — ``ok: false``.
Classic («поставь Калинку», PR-11): ``request_music`` с ``intent=melody``/``genre=classical|folk`` играет
песню движка v2 (``engine.classic``: поиск и ``harmonize`` старой библиотеки → ``arrange.song`` → ``render``),
один проход формы, потом ``finished``. Не нашлась — ``ok: false, found: false`` (I16).

Длина сета (06.10, сет без конца дошёл до 53-го трека): ``dj_set(tracks)`` — целое 1..``MAX_TRACKS`` из фразы
человека (роутер разбирает число кодом, LLM передаёт его узким параметром); без числа — ``DEFAULT_TRACKS``, а посреди
идущего сета (смена темы) — сколько треков ему оставалось: длина считается от начала сета (:meth:`DjSetTool.set_length`).

Партитуры (ADR-0154 PR-5): тема ищется ещё и в индексе партитур (``engine.score_library.ScoreIndex``) теми же
строками поиска, что мелодии RTTTL (``ThemeHits.query``, ``search.part_query``, #3512) — найденные материалы идут в план первыми треками (``set_plan.plan_materials``), приоритет:
название в индексе партитур > RTTTL-библиотека > строка ``THEMES``. Библиотеки на устройстве нет — сет как до PR-5
(хуки RTTTL), причина — строкой лога.

PR-10: после ``started`` сета ``dj_set`` в фоне спрашивает ``SetReasoner`` (LLM) профиль сета; ответ ок —
план подменяется со следующего несыгранного трека (``SetSession.replan``), иначе сет целиком seeded.

Ход LLM (06.10 14:50, set98207): в одном ходе ``dj_set`` → ``request_music('1812 Overture')`` — заказ гасил
только что запущенный сет, сам не заигрывал, и наступала тишина. Теперь решает владелец деки (ADR-0148):
``turn_id`` — скрытый аргумент хода (``llm_adapter.TURN_CONTEXT_ARGS``); сет, заигравший в этом же ходе,
``request_music`` не снимает — отказ :data:`SET_PLAYING`. Заказ сначала собирается (поиск, компоновка, рендер),
и только собранный снимает идущий сет: «не нашлось» и ошибка компоновки сет не гасят.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import replace
from typing import Any, Callable, Dict, Iterable, List, Mapping, NamedTuple, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.instruments import LeadChoice, lead_choice
from rob_box_music.dj_line import now_playing_text, persona_title, set_not_found_text
from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import DEFAULT_TRACKS, MAX_TRACKS, seeded_plan, set_tracks
from rob_box_music.theme import ThemeProfile, match_style_text, match_window_text, seeded_profile, style_for

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from rob_box_music.request_slots import RequestSlots
from rob_box_music.source import (CACHE, GENERATE, SYNTH, Route, SourceAsk, cache_text, decline_text, route_note,
                                  source_ask)
from rob_box_music.works import RTTTL, SCORE, key_slot

from .chooser_model import DETERMINISTIC_MODE, chooser_from_env
from .classic import ClassicPick, classic_picker
from .gen.song import SongStudio
from .song_order import SongOrder
from .dj_lines import TransitionLines, Titles, latin_fold, library_titles
from .piece import COMPOSER_MISSING, PieceChain, RecordLookup
from .piece_cover import PieceCover, cover_work
from .reasoner import SetPlanBox, SetReasoner
from .request_slots import Catalog, RequestSlotParser
from .score_library import ScoreLibrary, seed_plan, track_materials
from .search import ThemeHits, part_query, ThemeQuery, terms, theme_search
from .session import SetMemory, SetSession, plan_source, write_track_file
from .instrumental_order import InstrumentalOrder
from .source_route import SourceRouter
from .theme_grounding import set_slots
from .theme_links import Ask, ThemeLinks

_LOG = logging.getLogger(__name__)

#: Мелодии по ``id`` для хука темы: ``ids -> {id: rtttl}``.
MelodyLookup = Callable[[Iterable[str]], Dict[str, str]]
#: Мелодии по одному слову заказа прямым поиском (без LLM): ``слово -> ThemeHits`` — ближайшее по словам (композитор).
WordHooks = Callable[[str], "ThemeHits"]
#: Мелодии по словам темы: ``тема -> ThemeHits`` (id лучшие первыми, точное совпадение названия).
ThemeFinder = Callable[[str], ThemeHits]
#: ``track_id -> MusicEvent(started|rejected) | None`` — ждёт событие плеера (``MusicEventLog.wait``).
Confirm = Callable[[Optional[str]], Any]

#: ``request_music`` в classic-песню (PR-11).
CLASSIC_GENRES = ("classical", "folk")
#: ``dj_set(style)``: ключи ``knowledge.STYLES`` (ADR-0153 §4.1) и ``auto`` — стиль по словам темы, иначе клуб.
AUTO_STYLE = "auto"
STYLE_CHOICES = (AUTO_STYLE, *kn.STYLES)
#: Код отказа ``request_music``: сет запущен в этом же ходе и играет — заказ его не снимает. Сторона голоса
#: строит по нему фразу (``rob_box_voice.core.media_phrases.SET_PLAYING_REASON``, равенство держит тест).
SET_PLAYING = "set_playing"
#: Код отказа ``dj_set``: тема называет конкретные вещи, и ни одна не нашлась (#3493).
NOT_FOUND = "not_found"
#: Код отказа ``request_music``: просили «ещё <автор>», а всё подходящее уже играло (#3599); фразу строит код
#: (``rob_box_voice.core.media_phrases.NO_MORE_REASON``, равенство держит тест).
NO_MORE = "no_more"
#: Код отказа ``dj_set(start)``: в реплике хода нет слова сета/диджея/вечеринки и стиля — человек просил одну пьесу
#: (ADR-0157 §2.1, прогон 09.10: «сыграй баха на трамбоне» → LLM ``dj_set``). Решает код по ``heard_text``.
NOT_A_SET_REQUEST = "not_a_set_request"
#: Код отказа ``request_music``/``generate_music``: путь музыки не нашёл, чем играть просьбу (ADR-0158 PR-3); фразу
#: строит ``rob_box_music.source.decline_text`` (``media_phrases.SOURCE_DECLINED_REASON``, равенство держит тест).
SOURCE_DECLINED = "music_source"
#: ``generate_music``: что просили (``kind``) и в каком жанре (``style``, ключи ``knowledge.GEN_STYLES``).
GEN_KINDS = ("instrumental", "song", "robot_song")
GEN_STYLE_CHOICES = ("", *kn.GEN_STYLES)


def set_refusal(heard_text: Optional[str], running: bool) -> Optional[Dict[str, Any]]:
    """Отказ ``dj_set(start)`` по реплике хода (ADR-0157 §2.1) или ``None``. Реплики нет (вызов не из хода) или сет
    уже идёт (смена темы посреди сета, #3400) — не отказываем."""
    from rob_box_voice.core.media_command_grammar import asks_for_set

    if heard_text is None or running or asks_for_set(heard_text):
        return None
    return {"ok": False, "reason": NOT_A_SET_REQUEST,
            "detail": "в реплике нет слова сет/диджей/вечеринка и стиля — это заказ одной пьесы: request_music"}


def lead_for(*texts: Optional[str]) -> Optional[LeadChoice]:
    """Лид инструмента из реплики хода, иначе из ``text`` тула (ADR-0157 §2.4): слот «на <инструменте>» выделяет
    грамматика, чем играть — таблица ``INSTRUMENTS``; не просили — ``None``."""
    from rob_box_voice.core.media_command_grammar import instrument_in

    return next((lead_choice(key) for key in map(instrument_in, filter(None, texts)) if key), None)


def club_theme(text: str) -> str:
    """Тема одиночного трека: «сыграй (что-нибудь) на арфе» — заказ без названия, слова заказа и инструмента не тема
    (грамматика разобрала их в ``REQUEST_MUSIC`` с инструментом, ADR-0157 §3.1); иначе — слова как есть."""
    from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command

    command = parse_media_command(text)
    return command.name if command.intent is MediaIntent.REQUEST_MUSIC and command.instrument else text


def club_name(text: str, theme: str) -> str:
    """Как назвать одиночный трек на «что играет» — так же, как фраза запуска роутера (``request_ok_text``): название
    заказа по грамматике («поставь клубный трек» → «клубный трек»), иначе тема, иначе «клубный трек» (#3613: живой
    прогон 13.10 — «Сейчас играет «поставь клубный трек»»)."""
    from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command

    command = parse_media_command(text)
    if command.intent is MediaIntent.REQUEST_MUSIC and command.name:
        return command.name
    return theme or "клубный трек"


def asks_more(heard_text: Optional[str]) -> bool:
    """Реплика хода — «продолжай играть Вагнера» / «ещё Баха» (#3599): следующая пьеса автора, не повтор. Решает
    грамматика заказа по имени (``MediaCommand.more``); «ещё раз Калинку» — не она."""
    from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command

    command = parse_media_command(heard_text or "")
    return command.intent is MediaIntent.PLAY_NAMED and command.more


def lead_data(lead: Optional[LeadChoice]) -> Dict[str, Any]:
    """Поля результата об инструменте: по ним голосовая сторона строит фразу (``media_phrases.instrument_text``)."""
    return {"instrument": lead.asked, "instrument_played": lead.played} if lead else {}


def set_style(style: Optional[str], theme: str, heard_text: Optional[str] = None) -> str:
    """Стиль сета решает код (ADR-0148): слова стиля в реплике человека этого хода (``heard_text``: «синтвейв»,
    «8-битный» — ADR-0153 S2) важнее ключа от LLM; иначе названный ключ ``STYLES``; ``auto``/пусто — слова стиля темы,
    затем стиль строки таблицы тем (``theme.style_for``: киберпанк → synthwave), иначе ``DEFAULT_STYLE``. Ключ не из
    перечня — ``ValueError``."""
    heard = match_style_text(heard_text or "")
    if heard:
        return heard
    if style and style != AUTO_STYLE:
        if style not in kn.STYLES:
            raise ValueError(f"style={style!r}: есть только {list(STYLE_CHOICES)}")
        return style
    return style_for(theme)


def _exact_records(library: Any, ids: Iterable[str]) -> Dict[str, Dict[str, Any]]:
    """Записи RTTTL по ``id``: только точное совпадение имени — ``RtttlLibrary.get`` иначе вернёт лучшую по словам
    запись, для хука темы это была бы чужая мелодия (честный пропуск лучше подмены)."""
    found = {}
    for melody_id in ids:
        record = library.get(melody_id) or {}
        if str(record.get("name", "")).lower() == melody_id.lower() and record.get("rtttl"):
            found[melody_id] = record
    return found


def library_melodies(library_factory: Callable[[], Any]) -> MelodyLookup:
    """``{id: rtttl}`` из RTTTL-библиотеки (:func:`_exact_records`); библиотека открывается при первом сете."""
    library = _shared(library_factory)
    return lambda ids: {melody_id: r["rtttl"] for melody_id, r in _exact_records(library(), ids).items()}


def library_records(library_factory: Callable[[], Any]) -> RecordLookup:
    """``{id: (запись, название для голоса)}`` из RTTTL-библиотеки — кандидаты одиночной пьесы."""
    from ..core.rtttl_library import human_track_title

    shared = _shared(library_factory)

    def lookup(ids: Iterable[str]) -> Dict[str, Tuple[Dict[str, Any], str]]:
        library = shared()
        return {melody_id: (r, human_track_title(library, r)) for melody_id, r in _exact_records(library, ids).items()}

    return lookup


def word_hooks(library_factory: Callable[[], Any]) -> WordHooks:
    """``слово -> ThemeHits`` прямым поиском темы (``theme_search``, без связей и LLM)."""
    library = _shared(library_factory)
    return lambda word: theme_search(library(), word)


def theme_finder(library_factory: Callable[[], Any], links: Optional[ThemeLinks] = None,
                 ask: bool = True) -> ThemeFinder:
    """``тема -> ThemeHits`` по её словам (``engine.search.theme_search``); библиотека — при первой теме. Часть без
    находок — через связи реестра и проверенные каталогом строки поиска LLM (``engine.theme_links``, #3493); ``ask=False`` — без вопроса LLM (одиночная пьеса: ответ
    не ждётся на пути звука, #3608), только хранилище связей."""
    box: Dict[str, Any] = {}

    def find(theme: str) -> ThemeHits:
        theme = key_slot(theme)[0]  # «сет Токката ре минор»: тональность — не слова поиска (Minority, 09.10 #19)
        if not theme.strip():
            return ThemeHits()
        if "lib" not in box:
            box["lib"] = library_factory()
        hits = theme_search(box["lib"], theme)
        if links is None:
            return hits
        try:
            return links.expand(box["lib"], theme, hits, ask=ask)
        except Exception as exc:  # noqa: BLE001 — прямые находки не теряются из-за хранилища связей
            _LOG.warning(f"⚠️ [dj_set] связи темы «{theme}» упали: {type(exc).__name__}: {exc}")
            return hits

    return find


def _shared(factory: Callable[[], Any]) -> Callable[[], Any]:
    """Одна библиотека на хуки и поиск темы: открывается при первом обращении (из прогрева в фоне или из заказа —
    одна, под замком)."""
    box: Dict[str, Any] = {}
    lock = threading.Lock()

    def get() -> Any:
        with lock:
            if "lib" not in box:
                box["lib"] = factory()
            return box["lib"]

    return get


def _with_scores(titles: Titles, scores: ScoreLibrary) -> Titles:
    """Названия для голоса диджея: материал партитуры — по индексу партитур, мелодия RTTTL — по библиотеке."""
    def lookup(ids: Iterable[str]) -> Dict[str, str]:
        ids = list(ids)
        out = scores.titles([i for i in ids if ":" in i])
        rest = [i for i in ids if i not in out]
        return {**(titles(rest) if rest else {}), **out}
    return lookup


def _rtttl_library() -> Any:
    from ..core.rtttl_library import RtttlLibrary

    return RtttlLibrary()


def confirmed(result: Dict[str, Any], confirm: Optional[Confirm]) -> Dict[str, Any]:
    """``ok`` запуска — только по ``started`` этого трека (ADR-0149 §5.1, A14)."""
    if not result.get("ok") or confirm is None:
        return result
    event = confirm(result.get("track_id"))
    if event is None:
        return {**result, "ok": False, "reason": "not_started", "detail": "started не пришло"}
    if event.event == "rejected":
        return {**result, "ok": False, "reason": event.fields.get("reason"), "detail": event.fields.get("detail")}
    return {**result, "started": True}


def theme_links(node: Any, reasoner: SetReasoner) -> ThemeLinks:
    """Связи темы (#3493): LLM — та же, что у ризонера сета (её breaker); ризонер выключен — только хранилище."""
    return ThemeLinks(ask=reasoner.ask if reasoner.enabled else None,
                      logger=node.get_logger() if node is not None else None)


def not_found(hits: ThemeHits, materials: Iterable[str], theme: str = "", log: Any = _LOG) -> Tuple[str, ...]:
    """Названное в теме, из чего не нашлось ничего (ни мелодии, ни партитуры): сет не стартует пулом (#3493).
    Часть темы-перечисления без мелодий — честно в лог, не подмена (I16)."""
    if hits.missing:
        log.info(f"🎛️ [dj_set] тема «{theme}»: не найдено: {', '.join(f'«{p}»' for p in hits.missing)}")
    return hits.missing if hits.named and not hits.names and not tuple(materials) else ()


def score_materials(scores: Any, hits: ThemeHits, theme: str) -> Tuple[Tuple[str, ...], str]:
    """``(материалы партитур темы, хвост строки лога)``: поиск строками, по которым искались мелодии темы
    (``hits.query``, #3512), с замером M7; материалов нет — в лог, какие строки искались. Подставной поиск мелодий
    (тесты) строк не отдаёт — та же ``part_query`` без каталога RTTTL."""
    query = hits.query or ThemeQuery(theme, (part_query(None, theme),))
    started = time.perf_counter()
    materials = scores.search(query)
    how = f"партитуры: {scores.state}; поиск {(time.perf_counter() - started) * 1000:.1f} мс"
    return materials, how if materials else f"{how}; искали: {query.describe()}"


def music_busy(owner: Any, dj: Any) -> bool:
    """Музыка движка идёт: дека играет (``PlayerOwner.is_playing``) или идёт сет (``DjSetTool.running``) — между
    ``started`` треков сета дека может выглядеть пустой (06.10 15:30 UTC: мягкий cleanup погасил идущий сет)."""
    deck = owner is not None and owner.is_playing() is True
    return deck or getattr(dj, "running", False) is True


#: Источник звучащего трека (#3613) — что сказать на «что играет»: трек синтов, собранный кодом (``request_music``
#: intent=track, треки сета), пьеса по нотам партитуры, мелодия из архива RTTTL. Слова — ``media_phrases.SOURCE_WORDS``.
SOURCE_SYNTH, SOURCE_PIECE, SOURCE_MELODY = "synth", "piece", "melody"
_PIECE_SOURCES = {SCORE: SOURCE_PIECE, RTTTL: SOURCE_MELODY}
#: Факты заказа, которые владелец плеера кладёт в снимок (поле ``dj``) — по ним код отвечает «что играет».
TRACK_FACTS = ("title", "name", "source", "style", "bpm", "key", "instrument_played")


def track_facts(staged: Mapping[str, Any]) -> Dict[str, Any]:
    """Факты собранного заказа для снимка плеера (#3613): название как во фразе запуска (пьеса — название и автор из
    каталога, как ``media_phrases.piece_text``), источник, стиль, bpm, тональность, инструмент, на котором играем."""
    facts: Dict[str, Any] = {"enabled": False}
    facts.update({k: staged[k] for k in TRACK_FACTS if staged.get(k) not in (None, "")})
    played = staged.get("played")
    if isinstance(played, Mapping):
        title, composer = str(played.get("title") or staged.get("title") or ""), str(played.get("composer") or "")
        facts["name"] = title
        if composer and composer.lower() not in title.lower():
            facts["composer"] = composer
        facts["source"] = _PIECE_SOURCES.get(str(played.get("source") or ""), SOURCE_MELODY)
    return facts


def tool_result(data: Dict[str, Any], what: str) -> MCPToolResult:
    if data.get("ok"):
        return MCPToolResult(success=True, data=data)
    if data.get("composer_missing") and data.get("message"):  # назван композитор, пьес нет (#3604): фраза кода
        from rob_box_voice.core.media_phrases import composer_missing_error
        return MCPToolResult(success=False, data=data, error=composer_missing_error(data["message"]))
    if data.get("reason") == NOT_FOUND and data.get("message"):  # отказ сета: фраза кода, её скажет голос
        return MCPToolResult(success=False, data=data, error=data["message"])
    if data.get("reason") == SOURCE_DECLINED and data.get("message"):  # отказ пути (ADR-0158): фраза кода
        from rob_box_voice.core.media_phrases import source_error
        return MCPToolResult(success=False, data=data, error=source_error(data["message"]))
    if data.get("reason") == NO_MORE and data.get("message"):  # «больше нет» (#3599): фраза кода, её скажет голос
        from rob_box_voice.core.media_phrases import no_more_error
        return MCPToolResult(success=False, data=data, error=no_more_error(data["message"]))
    return MCPToolResult(success=False, data=data, error=f"{what}: {data.get('reason')}")


class DjSetTool(MCPTool):
    """«Ты диджей» v2: сет без LLM на пути звука; один сет за раз."""

    def __init__(self, node: Any, owner: Any, melodies: Optional[MelodyLookup] = None, *,
                 seed: Callable[[], int] = lambda: int(time.time()), confirm: Optional[Confirm] = None,
                 reasoner: Optional[SetReasoner] = None, speak: Optional[Callable[[str], None]] = None,
                 finder: Optional[ThemeFinder] = None, history: Any = None,
                 tracks_dir: Optional[str] = None, titles: Optional[Titles] = None, lines: bool = True,
                 scores: Optional[ScoreLibrary] = None) -> None:
        super().__init__(node)
        self._owner = owner
        self._reasoner = reasoner or SetReasoner(enabled=False)
        self._speak = speak
        library = self.library = _shared(_rtttl_library)  # одна библиотека RTTTL на сет и одиночную пьесу
        self._melodies = melodies or library_melodies(library)
        links = theme_links(node, self._reasoner)
        self._find = finder or theme_finder(library, links)
        self._find_work = finder or theme_finder(library, links, ask=False)  # одиночная пьеса: LLM не на пути звука
        #: Поиск темы без вопроса LLM (связи — только хранилище): мост ожидания генерации (песня, кавер, сет) берётся
        #: кодом мгновенно — 13.10 мост песни 8 с ждал theme-links, ``request_music`` 10.6 с > таймаута хода 10 с.
        self.quick_find = self._find_work
        self._not_found: Tuple[str, ...] = ()  # названное в теме, из чего не нашлось ничего: сет не стартует (#3493)
        if scores is None:
            scores = ScoreLibrary()
            scores.warm()  # индекс поиска партитур строится в фоне: первая тема его не ждёт (M7)
        self._scores = scores
        self._titles = _with_scores(titles or library_titles(library), self._scores)
        self._missing: Tuple[str, ...] = ()  # части темы-перечисления без мелодий (последний сет): их не называть
        self._lines = lines  # реплика диджея на каждом переходе (§12 В2, решение Шифу 06.10)
        self._facts: Optional[TransitionLines] = None  # факты играющего трека последнего сета (для ``status``)
        self._seed = seed
        self._tracks_dir = tracks_dir or None
        self._confirm = confirm
        self._lock = threading.Lock()
        self._session: Optional[SetSession] = None
        self._session_turn: Optional[str] = None  # ход LLM, в котором сет ``self._session`` заиграл (``started``)
        self._last: Optional[SetSession] = None  # последний начатый сет (и доигравший сам): для ``status``
        self._last_dj: Optional[Tuple[str, Optional[str]]] = None  # (тема, персона) последнего сета: «продолжай сэт»
        # треки прошлых сетов (``history`` — music_history в БД): разнообразие между сетами (A13, I17)
        self._memory = SetMemory(store=history)
        #: Сет из кэша и генерации (ADR-0158 PR-8, ``engine.gen.gen_session.GenSetMaker``): любой сет (тема, стиль,
        #: auto) — путь каждого трека чойсером: кэш → генерация → синтез-запас (В13, 14.10); ``None`` — только синтез.
        self.gen_sets: Any = None

    @property
    def name(self) -> str:
        return "dj_set"

    @property
    def description(self) -> str:
        return ("Диджей-сет: action=start — начать сет на тему theme (треки, переходы и темп решает код), "
                "tracks — сколько треков, только если человек назвал число (без числа длину решает код); "
                "action=stop — закончить сет и выключить музыку. Об успехе робот скажет сам, когда музыка "
                "реально заиграет; ok=false — музыка не заиграла.")

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="action", type="string", description="start — начать сет, stop — закончить",
                             enum=["start", "stop"]),
            MCPToolParameter(name="theme", type="string", description="Тема сета словами человека (например «космос»)",
                             required=False),
            MCPToolParameter(name="persona", type="string", description="Имя диджея для реплик", required=False),
            MCPToolParameter(name="style", type="string", required=False, enum=list(STYLE_CHOICES),
                             description="Стиль сета (один на весь сет); auto — по словам темы, иначе клубный"),
            MCPToolParameter(name="tracks", type="integer", required=False, minimum=1, maximum=MAX_TRACKS,
                             description="Сколько треков в сете — только если человек назвал число"),
            MCPToolParameter(name="styles", type="array", required=False,
                             items=MCPToolParameter(name="style", type="string", description="стиль трека",
                                                    enum=list(kn.GEN_STYLES)),
                             description="Стили по трекам, если человек назвал несколько, — по порядку его слов"),
            MCPToolParameter(name="minutes", type="integer", required=False, minimum=1, maximum=180,
                             description="Сколько минут музыки — только если человек назвал время"),
        ]

    @property
    def slice(self) -> str:
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, action: str = "start", theme: Optional[str] = None,
                persona: Optional[str] = None, style: Optional[str] = None,
                tracks: Optional[int] = None, styles: Optional[List[str]] = None, minutes: Optional[int] = None,
                heard_tracks: Optional[int] = None, turn_id: Optional[str] = None,
                heard_text: Optional[str] = None) -> MCPToolResult:
        """``heard_tracks``, ``turn_id`` и ``heard_text`` — скрытые аргументы хода (``llm_adapter.TURN_CONTEXT_ARGS``):
        длина сета из слов человека по грамматике (``set_length_words``; есть — она решает, а не ``tracks`` от LLM,
        ADR-0148), ход, запустивший сет (:meth:`started_in_turn`), и реплика хода: тема LLM не из неё (взята из
        истории диалога, 06.10 18:04) — тема реплики (``theme_grounding``). ``styles``/``minutes`` LLM — подсказка:
        стили и время из слов человека решает код (``GenSetMaker.plan``)."""
        tracks = heard_tracks or tracks
        refusal = set_refusal(heard_text, self.running) if action == "start" else None
        if refusal is not None:
            return tool_result(refusal, "сет не начался")
        if action == "start":  # «диджей X» одним видом: и от роутера, и от LLM
            theme, persona = self._set_slots(theme, persona_title(persona) or None, heard_text)
        with self._lock:
            if action == "stop":
                return self._stop()
            if action == "start":
                try:
                    key = set_style(style, theme or "", heard_text)
                    named = None if tracks is None else set_tracks(tracks)
                except ValueError as exc:
                    return MCPToolResult(success=False, error=str(exc))
                # окно стиля словами человека («гранж» → rock/grunge, ADR-0153 S5); нет слов — окно выбирает план
                window = match_window_text(key, heard_text, theme)
                result = self._start(theme or "", persona, key, named, window, heard_text,
                                     hints=(styles, minutes))
            else:
                return MCPToolResult(success=False, error=f"action={action!r}: есть только start и stop")
        result = confirmed(result, self._confirm)  # ждём started вне замка
        self._session_turn = turn_id if result.get("ok") else None
        return tool_result(result, "сет не начался")

    def _set_slots(self, theme: Optional[str], persona: Optional[str],
                   heard_text: Optional[str]) -> Tuple[str, Optional[str]]:
        """``(тема, персона)`` сета по реплике хода и последнему сету (``theme_grounding.set_slots``, #3581)."""
        return set_slots(theme, persona, heard_text, self._last_dj,
                         self.node.get_logger() if self.node is not None else _LOG)

    @property
    def running(self) -> bool:
        """Сет идёт: начат и не остановлен, не доиграл сам (между ``started`` треков дека может быть пуста)."""
        session = self._session
        return session is not None and session.active

    def started_in_turn(self, turn_id: Optional[str]) -> Optional[str]:
        """``set_id`` сета, который заиграл в ходе ``turn_id`` и идёт сейчас; иначе ``None`` (хода нет — ``None``)."""
        session = self._session
        if not turn_id or session is None or not session.active or self._session_turn != turn_id:
            return None
        return session.set_id

    def _hits(self, theme: str, log: Any, find: Optional[ThemeFinder] = None) -> ThemeHits:
        """Находки по словам темы (поиск, связи ``theme_links``); поиск упал — пусто, причина в лог."""
        try:
            return (find or self._find)(theme)
        except Exception as exc:  # noqa: BLE001 — поиск не держит звук: сет играет пул по хешу темы
            log.warning(f"⚠️ [dj_set] поиск мелодий темы «{theme}» упал: {type(exc).__name__}: {exc}")
            return ThemeHits()

    def work_hits(self, work: str) -> ThemeHits:
        """Находки RTTTL по словам произведения (ADR-0157 §2.3) — тот же поиск и те же связи ``theme_links``, что у
        темы сета, но без строки таблицы тем (она про тему, а не про пьесу) и без состояния сета; партитуры
        произведения — ``ScoreLibrary.work_candidates`` (``engine.piece``)."""
        log = self.node.get_logger() if self.node is not None else _LOG
        hits = self._hits(work, log, self._find_work)
        log.info(f"🎼 [classic] «{work}»: хуки={list(hits.names)}; партитуры: {self._scores.state}")
        return hits

    def theme_profile(self, theme: str, style: str = kn.DEFAULT_STYLE,
                      find: Optional[ThemeFinder] = None) -> ThemeProfile:
        """Seeded-профиль темы с мелодиями по её словам; поиск упал — профиль без находок, причина в лог."""
        log = self.node.get_logger() if self.node is not None else _LOG
        hits = self._hits(theme, log, find)
        materials, how = score_materials(self._scores, hits, theme)
        profile = seeded_profile(theme, style, found=hits.names, exact=hits.exact, parts=hits.parts,
                                 materials=materials)
        log.info(f"🎛️ [dj_set] тема «{theme}»: style={profile.style} source={profile.source} row={profile.row} "
                 f"хуки={list(profile.hook_ids)} материалы={list(materials)} ({how})")
        self._missing = hits.missing
        self._not_found = not_found(hits, materials, theme, log)
        return profile

    def plan_materials(self, plan: Any, logger: Any = None) -> Mapping[str, Any]:
        """Материалы треков плана (``TrackPlan.material``) для ``compose``: материал трека 1 читается здесь, при старте
        (замер M7 — в строку лога), остальные — при компоновке своего трека в фоне (:class:`PlanMaterials`)."""
        return track_materials(self._scores, plan, logger or _LOG)

    def set_length(self, named: Optional[int]) -> Tuple[int, str]:
        """``(длина, откуда)`` нового сета — решает код: число человека; без числа посреди идущего сета (смена
        темы, #3404/#3412) — сколько ему оставалось, считая играющий трек сыгранным (не меньше 1): «сет на 20»,
        сменивший тему на 8-м треке, доиграет 12; иначе :data:`DEFAULT_TRACKS`."""
        if named is not None:
            return named, "названо"
        session = self._session
        if session is not None and session.active:
            return max(1, session.tracks - session.track_no), f"остаток {session.set_id}"
        return DEFAULT_TRACKS, "по умолчанию"

    def _start(self, theme: str, persona: Optional[str], style: str = kn.DEFAULT_STYLE,
               tracks: Optional[int] = None, window: Optional[str] = None,
               heard_text: Optional[str] = None,
               hints: Tuple[Optional[List[str]], Optional[int]] = (None, None)) -> Dict[str, Any]:
        """Сет: путь каждого трека — кэш, генерация (:func:`generated_set`); синтез-сет (ADR-0149) — только запас
        (В13), тогда фраза кода ``source_note`` говорит почему."""
        length, why = self.set_length(tracks)
        generated, fallback = generated_set(self, theme, (style, window), persona, (length, why), heard_text, hints)
        if generated is not None:
            return generated
        style, window = fallback.style, fallback.window
        profile = self.theme_profile(theme, style)
        if self._not_found:  # названное не нашлось — не пул по хешу темы вместо него; идущий сет не трогаем
            return {"ok": False, "reason": NOT_FOUND, "theme": theme, "missing": list(self._not_found),
                    "message": set_not_found_text(self._not_found)}
        self._drop_session()
        set_seed = self._seed()
        set_id = f"set{set_seed % 100000:05d}"
        logger = self.node.get_logger() if self.node is not None else None
        plan = seed_plan(self._scores, profile, set_seed, length, set_id, self._memory.peek(), logger or _LOG,
                         genre=window, references=list(self._melodies(profile.theme_hooks).values()))
        (logger or _LOG).info(f"🎛️ [dj_set] {set_id} длина сета: {length} ({why}), "
                              f"энергия={[t.energy for t in plan.tracks]}")
        materials = self.plan_materials(plan, logger)
        box = SetPlanBox(plan, self._melodies, lines=self._transition_lines(persona, logger), logger=logger)
        base = plan_source(box.current, self._memory, materials)
        self._facts.forecast = getattr(base, "upcoming", None)  # «дальше будет» — из той же очереди, что компоновка
        session = SetSession(self._owner, lambda no, deck: box.compose_mark(base(no, deck), no), set_id=set_id,
                             bpm=plan.bpm, dj={"theme": theme, "persona": persona}, logger=logger,
                             on_track_started=box.on_started, tracks_dir=self._tracks_dir, tracks=length)
        result = session.start()
        self._session = session if result.get("ok") else None
        self._last = self._session or self._last
        reasoner = None
        if self._session is not None:  # звук уже поставлен seeded-планом; LLM — в фоне (§4.8)
            self._last_dj = (theme, persona)
            reasoner = self._reasoner.request(set_id, theme, plan, lambda ref: (box.apply(ref), session.replan()))
        return noted({**result, "theme": theme, "style": plan.style, "genre": plan.genre,
                      "theme_source": profile.source, "seed": set_seed, "tracks": length, "reasoner": reasoner},
                     fallback.note)

    def _drop_session(self) -> None:
        """Новый сет снимает идущий (под замком ``execute``)."""
        self._session_turn = None
        if self._session is not None:
            self._session.stop("new_set")

    def adopt(self, session: SetSession, theme: str, persona: Optional[str]) -> None:
        """Сет из генерации заиграл (ADR-0158 PR-8): он — идущий сет (стоп, «что играет», «продолжай»)."""
        self._session, self._last, self._facts, self._last_dj = session, session, None, (theme, persona)

    def status(self) -> Dict[str, Any]:
        """Сет для ``get_music_state``: идёт (трек N из M), окончен сам или его нет; фразу строит код (ADR-0148)."""
        session = self._last
        if session is None:
            return {"active": False, "message": "Диджей-сета не было."}
        info = {"set_id": session.set_id, "tracks": session.tracks, "track_no": session.track_no}
        now = self._facts.now() if self._facts is not None else {}
        if session.active and now:
            return {**info, **now, "active": True, "message": f"Идёт диджей-сет. {now_playing_text(now)}"}
        if session.active:
            return {**info, "active": True, "message": f"Идёт диджей-сет: трек {session.track_no} из {session.tracks}."}
        if session.ended:
            return {**info, "active": False, "ended": True,
                    "message": f"Диджей-сет закончился сам: сыграны все {session.tracks} из {session.tracks}."}
        return {**info, "active": False, "ended": False, "message": "Диджей-сет остановлен."}

    def _transition_lines(self, persona: Optional[str], logger: Any) -> TransitionLines:
        """Факты и реплики сета: факты (мелодия трека, план дальше, не найденное) — всегда, в снимок и ``status``;
        реплика — если включена параметром и есть чем говорить; раскраска — LLM ризонера (его breaker)."""
        speak = self._speak if self._lines else None
        ask = self._reasoner.ask if self._reasoner.enabled and speak is not None else None
        lines = TransitionLines(speak, titles=self._titles, ask=ask, persona=persona, missing=self._missing,
                                publish=getattr(self._owner, "update_dj", None), fold=latin_fold, logger=logger)
        self._facts = lines
        return lines

    def close_set(self, reason: str) -> None:
        """Закрыть идущий сет: деку занимает другой запрос (``request_music``)."""
        with self._lock:
            session, self._session, self._session_turn = self._session, None, None
        if session is not None:
            session.stop(reason)

    def _stop(self) -> MCPToolResult:
        """Стоп сета, а без сета — деки v2 (одиночный трек ``request_music``)."""
        session, self._session = self._session, None
        if session is None or not session.active:  # сета нет или он доиграл сам — стоп деки v2
            result = self._owner.stop("user_stop")
            return MCPToolResult(success=True, data={**result, "was_playing": result.get("track_id") is not None})
        return MCPToolResult(success=True, data={**session.stop("user_stop"), "was_playing": True})


class RequestMusicTool(MCPTool):
    """«Поставь клубный трек» v2: один club-трек из ``compose``/``render``, без LLM на пути звука."""

    def __init__(self, node: Any, owner: Any, dj: DjSetTool, melodies: Optional[MelodyLookup] = None, *,
                 seed: Callable[[], int] = lambda: int(time.time()), confirm: Optional[Confirm] = None,
                 classic: Optional[Callable[..., ClassicPick]] = None, history: Any = None,
                 records: Optional[RecordLookup] = None, words: Optional[WordHooks] = None,
                 tracks_dir: Optional[str] = None, chooser_mode: str = DETERMINISTIC_MODE,
                 slots_ask: Optional[Ask] = None, slots: Optional[RequestSlotParser] = None,
                 gen_library: Callable[[], Any] = lambda: None,
                 play_cached: Optional[Callable[[str], MCPToolResult]] = None, cover: Any = None,
                 speak: Optional[Callable[[str], Any]] = None, songs: Optional[SongStudio] = None,
                 instrumental: Any = None) -> None:
        super().__init__(node)
        self._owner = owner
        self._dj = dj
        library = getattr(dj, "library", None) or _shared(_rtttl_library)  # та же, что у сета: открывается один раз
        self._melodies = melodies or library_melodies(library)
        self._seed = seed
        self._confirm = confirm
        chooser = chooser_from_env(chooser_mode, log=self._log)
        # одиночная пьеса: кандидаты произведения, выбор, перебор (ADR-0157 §2.3, PR-2/PR-3)
        self._pieces = PieceChain(dj, records or library_records(library), words or word_hooks(library),
                                  classic or classic_picker(library), history, chooser=chooser, log=self._log)
        # путь музыки (ADR-0158 PR-3): синтез / кэш генераций / генерация / отказ — тот же выбиратель
        self._source = SourceRouter(chooser, gen_library, log=self._log, cover=cover, studio=songs,
                                    instrumental=instrumental)
        self._songs = songs  # песня голосом робота (ADR-0158 PR-7): фоновая генерация, пока играет мост
        self.gen_library = gen_library  # кэш генераций: результат ожидания встаёт с деки моста (BridgeHold, PR-8)
        self.hold: Any = None  # последний мост ожидания генерации (``BridgeHold``): песня входит вместо него
        self._song_order = SongOrder(songs, self._source.route, hold=lambda: self.hold)
        # одиночный инструментал («сгенерируй рок»): мост, вход по такту, фразы кода — ``InstrumentalOrder``
        self._instrumental = (InstrumentalOrder(instrumental, play_cached, speak, hold=lambda: self.hold, log=self._log)
                              if instrumental is not None else None)
        self._play_cached = play_cached  # трек кэша — play_library_track: дека владельца (ADR-0158 PR-2)
        # кавер пьесы (ADR-0158 PR-4): ``cover`` — gen.cover.CoverMaker; нет — кавер-кандидатов нет
        self._cover = PieceCover(cover, speak, self._log) if cover is not None else None
        self._tracks_dir = tracks_dir or None  # модели сыгранных пьес, как у треков сета (I24)
        # фраза заказа → слоты, проверенные каталогом (#3609); при подставном поиске (тесты) словаря архива нет —
        # судить слова нечем, разбора нет (название как от грамматики), если разборщик не передан явно
        self._slots = slots or (None if words is not None else RequestSlotParser(
            lambda text: terms(library(), text), Catalog(library, getattr(dj, "_scores", None)), ask=slots_ask,
            log=self._log))

    def warm(self) -> threading.Thread:
        """Прогреть путь одиночной пьесы в фоне (#3577: первый заказ после рестарта — 4.6 с при бюджете 2.5 с):
        индекс партитур, каталог RTTTL, история — до первого заказа, старт ноды не ждёт."""
        thread = threading.Thread(target=self._pieces.warm, name="rbx-piece-warm", daemon=True)
        thread.start()
        return thread

    @property
    def name(self) -> str:
        return "request_music"

    @property
    def description(self) -> str:
        return ("Поставить музыку по просьбе человека: intent=track — клубный трек (темп, тональность и "
                "аранжировку решает код), intent=melody — известная мелодия по названию из text. text — слова "
                "человека дословно. Об успехе робот скажет сам, когда музыка реально заиграет; ok=false — "
                "музыка не заиграла.")

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="intent", type="string", description="track — трек, melody — мелодия по названию",
                             enum=["track", "melody"]),
            MCPToolParameter(name="text", type="string", description="Слова человека дословно, без пересказа"),
            MCPToolParameter(name="mood", type="string", description="Настроение трека", required=False,
                             enum=sorted(kn.MOOD_ENERGY)),
            MCPToolParameter(name="genre", type="string", description="Жанр; auto — решает код", required=False,
                             enum=["auto", "club", "classical", "folk"]),
        ]

    @property
    def slice(self) -> str:
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, intent: str = "track", text: str = "", mood: Optional[str] = None,
                genre: Optional[str] = None, turn_id: Optional[str] = None,
                heard_text: Optional[str] = None) -> MCPToolResult:
        """``turn_id`` и ``heard_text`` — скрытые аргументы хода (``llm_adapter.TURN_CONTEXT_ARGS``): сет этого хода не
        снимается; инструмент («на балалайке») — из реплики хода (:func:`lead_for`), LLM его не выбирает."""
        return self.order(intent, text, mood, genre, turn_id, heard_text)

    def order(self, intent: str = "track", text: str = "", mood: Optional[str] = None, genre: Optional[str] = None,
              turn_id: Optional[str] = None, heard_text: Optional[str] = None,
              ask: Optional[SourceAsk] = None, gen_failed: str = "") -> MCPToolResult:
        """Заказ: путь музыки решает код (ADR-0158 PR-3) по просьбе ``ask`` (нет — слова реплики хода), затем
        синтез/пьеса как раньше, трек кэша (:meth:`_play_cache`) или честный отказ с фразой кода."""
        set_id = self._dj.started_in_turn(turn_id)
        if set_id is not None:
            return tool_result({"ok": False, "reason": SET_PLAYING, "set_id": set_id,
                                "detail": "сет запущен в этом ходе и играет — заказ его не заменяет"},
                               "музыка не заменена")
        classic = intent == "melody" or genre in CLASSIC_GENRES
        what = "мелодия не заиграла" if classic else "музыка не заиграла"
        # пьеса собирается до выбора пути: кавер (ADR-0158 PR-4) — по её произведению, мост и фолбэк — она сама
        ask = ask or source_ask(heard_text or text)
        piece = self._stage_piece(text, heard_text) if classic else None
        work, cover_track = cover_source(self._cover, piece, self._pieces, self._seed)
        route = self._source.route(ask, piece=classic, work=work, gen_failed=gen_failed)
        bridge = is_bridge(route, self._cover)
        if route.kind != SYNTH and not bridge:
            return self._off_synth(route, what, ask, (intent, text, mood, genre, turn_id, heard_text),
                                   gen_failed == "pending")
        staged = self._stage(route, piece, text, mood, heard_text, bridge_find(self._dj, gen_failed))
        if "program" not in staged:  # не нашлось / не собралось: идущий сет играет дальше
            return tool_result(staged, what)
        program, once, row, model = unstage(staged)
        self._dj.close_set("request_music")  # деку снимаем только под собранный заказ
        # мост ожидания генерации (кавер — PR-4, песня — PR-7) держит звук до результата (BridgeHold, PR-8)
        result, hold = play_bridged(self, program, staged, (model, once), bridge or gen_failed == "pending")
        if result.get("ok"):
            self._started(row, model)
            if bridge:  # мост звучит — кавер в фоне встанет вместо него кроссфейдом
                self._cover.start(route, cover_track or model, hold)
        return tool_result(result, what)

    def _started(self, row: Mapping[str, Any], model: Any) -> None:
        """Заказ зазвучал: пьеса — в память свежести, модель трека — в каталог моделей (I24)."""
        if row:
            self._pieces.memory.remember(row)
        if model is not None and self._tracks_dir:
            self._write_model(model)

    def _stage_piece(self, text: str, heard_text: Optional[str]) -> Dict[str, Any]:
        """Пьеса по названию с инструментом реплики — собрана, не запущена (мост/фолбэк кавера и сам синтез)."""
        lead = lead_for(heard_text, text)
        staged = self._stage_classic(text, lead, asks_more(heard_text))
        staged.update(lead_data(lead))
        return staged

    def _stage(self, route: Route, piece: Optional[Dict[str, Any]], text: str, mood: Optional[str],
               heard_text: Optional[str], find: Optional[ThemeFinder] = None) -> Dict[str, Any]:
        """Собрать синтез пути (пьеса ``piece`` или трек стиля ``route.style``) с фразой пути кода; ``find`` — поиск
        темы моста ожидания генерации (без LLM, :func:`bridge_find`)."""
        if piece is None:
            lead = lead_for(heard_text, text)
            staged = self._stage_club(text, mood, lead, route.style, find)
            staged.update(lead_data(lead))
        else:
            staged = piece
        note = route_note(route)
        if note:  # играет своё, а просили генерацию: фраза кода говорит об этом (ADR-0158 G3)
            staged["source_note"] = note
        return staged

    def _off_synth(self, route: Route, what: str, ask: SourceAsk, args: Tuple[Any, ...],
                   held: bool = False) -> MCPToolResult:
        """Путь не синтез: песня голосом робота (PR-7), трек кэша (есть шов проигрывания; ``held`` — мост ожидания
        песни) или честный отказ с фразой кода — без тихой подмены."""
        if self._instrumental is not None and self._instrumental.handles(route):
            return self._instrumental.start(route, ask, args, self.order, tool_result)
        if self._song_order.handles(route):
            return self._song_order.start(ask, args, self.order, tool_result)
        if route.kind == CACHE and route.track is not None and self._play_cached is not None:
            return play_cache(self._play_cached, route, what, hold_start(self) if held else None)
        return tool_result({"ok": False, "reason": SOURCE_DECLINED, "route": route.id,
                            "message": decline_text(route)}, what)

    def _write_model(self, model: Any) -> None:
        try:
            self._log().info(f"🎼 [classic] модель трека: {write_track_file(self._tracks_dir, model)}")
        except OSError as exc:
            self._log().warning(f"⚠️ [classic] модель трека не записана: {type(exc).__name__}: {exc}")

    def _stage_club(self, text: str, mood: Optional[str], lead: Optional[LeadChoice] = None,
                    style: str = kn.DEFAULT_STYLE, find: Optional[ThemeFinder] = None) -> Dict[str, Any]:
        """Трек плана с энергией настроения: ``compose`` → ``render`` (дека A) — собран, но не запущен; ``lead`` —
        синт инструмента заказа; ``style`` — стиль синтеза, который выбрал путь (ADR-0158)."""
        theme = club_theme(text)
        profile = (self._dj.theme_profile(theme, style, find) if style in kn.STYLES and style != kn.DEFAULT_STYLE
                   else self._dj.theme_profile(theme, find=find))
        seed = self._seed()
        plan = seeded_plan(profile, seed, set_id=f"req{seed % 100000:05d}")
        energy = kn.MOOD_ENERGY.get(mood or "", kn.ENERGY_WAVE[0])
        track_no = kn.ENERGY_WAVE.index(energy) + 1
        try:
            model = compose(plan, track_no, melodies=self._melodies(profile.hook_ids), deck="A",
                            materials=self._dj.plan_materials(plan), lead_synth=lead and lead.synth)
            program = render(model, "A")
        except Exception as exc:  # noqa: BLE001 — отказ громкий (I25), звука нет
            return self._owner.reject(f"{plan.set_id}:{track_no:02d}:A", "compose_error",
                                      f"{type(exc).__name__}: {exc}")
        title = f"{profile.theme or 'клубный трек'} · {plan.bpm} BPM"
        return {"program": program, "once": False, "bridge_model": model, "title": title,
                "name": club_name(text, profile.theme),
                "source": SOURCE_SYNTH, "style": plan.style, "bpm": plan.bpm, "energy": energy,
                "theme_source": profile.source}

    def _log(self) -> Any:
        return self.node.get_logger() if self.node is not None else _LOG

    def _request_slots(self, text: str, work: str, command: Any,
                       lead: Optional[LeadChoice]) -> Tuple[RequestSlots, Optional[LeadChoice]]:
        """Слоты заказа (#3609): непонятые слова — LLM по схеме, проверка каталогом; ``(слоты, лид инструмента от LLM —
        если реплика хода инструмент не назвала)``. Инструмент реплики хода («на органе») — слот грамматики."""
        if lead is not None:
            command = replace(command, instrument=command.instrument or lead.asked)
        slots = self._slots.parse(text, work, command) if self._slots else RequestSlots(work, work=work)
        return slots, (None if lead or not slots.instrument else lead_choice(slots.instrument))

    def _stage_classic(self, text: str, lead: Optional[LeadChoice] = None, more: bool = False) -> Dict[str, Any]:
        """Мелодия по названию: песня v2 (одна пьеса, один проход формы) — собрана, не запущена.

        Название берёт грамматика заказа по имени («поставь калинку» → «калинку»); не разобрала — слова целиком.
        Тональность («ре минор») — слот, не слова поиска (:func:`works.key_slot`). Кандидаты — ``engine.piece``;
        ``not_found`` — только когда кандидатов нет вовсе (ADR-0157 R5). ``more`` («продолжай играть Вагнера», #3599) —
        только то, чего недавно не играло; всё сыграно — ``no_more`` с фразой кода, не повтор и не подмена.
        """
        from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command

        command = parse_media_command(text)
        query = command.name if command.intent is MediaIntent.PLAY_NAMED else text
        work, key = key_slot(query)
        slots, slot_lead = self._request_slots(text, work, command, lead)
        work, lead = slots.query, lead or slot_lead
        seed = self._seed()
        try:
            pick, played, reasons, count = self._pieces.play(work, key, seed, lead, fresh=more,
                                                             confirmed=slots.work_confirmed)
        except Exception as exc:  # noqa: BLE001 — отказ громкий (I25), звука нет
            return self._owner.reject(f"classic:{work}", "compose_error", f"{type(exc).__name__}: {exc}")
        self._log().info(f"🎼 [classic] «{work}» тональность={key or '—'}: кандидатов {count}, недавние "
                         f"{self._pieces.memory.recent()} → {_piece_line(pick, played)}; "
                         f"отказы: {'; '.join(reasons) or '—'}")
        if pick is None and played.get("repeats"):  # просили «ещё», а всё подходящее уже играло
            from rob_box_voice.core.media_phrases import no_more_text
            return {"ok": False, "found": True, "reason": NO_MORE, "query": work,
                    "message": no_more_text(work, played["repeats"][0])}
        if pick is None and played.get(COMPOSER_MISSING):  # композитор назван, его пьес нет: фраза кода (#3604)
            from rob_box_voice.core.media_phrases import composer_missing_text
            return {"ok": False, "found": False, "reason": NOT_FOUND, "detail": "; ".join(reasons), "query": work,
                    "composer_missing": True,
                    "message": composer_missing_text(str(played.get("composer") or ""))}
        if pick is None and not count:
            return {"ok": False, "found": False, "reason": NOT_FOUND, "detail": "; ".join(reasons), "query": work}
        if pick is None:
            return self._owner.reject(f"classic:{work}", "compose_error", "; ".join(reasons))
        return {"program": pick.program, "once": True, "found": True, "title": pick.title, "melody_id": pick.melody_id,
                "bpm": pick.bpm, "key": pick.key, "played": played, "history": dict(pick.history), "track": pick.track,
                "work": pick.work, "public_domain": pick.public_domain, **lead_data(slot_lead)}


class GenerateMusicTool(MCPTool):
    """«Сгенерируй …» (ADR-0158 §3.3, PR-3): узкий тул-просьба. Становится слотами ``asked_generated`` (+ петь /
    своим голосом по ``kind``, жанр по ``style``) и идёт тем же путём, что ``request_music``: денег, лимита или
    исполнителя генерации нет — кандидатов генерации нет, играет своё или сочинённое раньше, либо честный отказ."""

    def __init__(self, node: Any, request: RequestMusicTool) -> None:
        super().__init__(node)
        self._request = request

    @property
    def name(self) -> str:
        return "generate_music"

    @property
    def description(self) -> str:
        return ("Просьба сгенерировать музыку (Suno): kind=instrumental — без слов, song — песня, robot_song — песня "
                "голосом робота; style — жанр, если назван. Это просьба, а не приказ: что играть (новую генерацию, "
                "сочинённое раньше или своё) и что сказать — решает робот сам по словам человека; ok=false — "
                "честный отказ, его фразу робот скажет сам.")

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="kind", type="string", description="instrumental, song или robot_song",
                             enum=list(GEN_KINDS)),
            MCPToolParameter(name="style", type="string", description="Жанр, если человек его назвал",
                             required=False, enum=list(GEN_STYLE_CHOICES)),
            MCPToolParameter(name="theme", type="string", description="Тема словами человека", required=False),
        ]

    @property
    def slice(self) -> str:
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    @property
    def starts_music(self) -> bool:
        return True

    def execute(self, kind: str = "instrumental", style: str = "", theme: str = "", turn_id: Optional[str] = None,
                heard_text: Optional[str] = None) -> MCPToolResult:
        """Слоты просьбы: слова реплики хода + то, что эта просьба — генерация (``kind``, ``style`` — из перечней)."""
        if kind not in GEN_KINDS or (style or "") not in GEN_STYLE_CHOICES:
            allowed = f"есть {list(GEN_KINDS)} и {list(GEN_STYLE_CHOICES)}"
            return MCPToolResult(success=False, error=f"kind={kind!r}/style={style!r}: {allowed}")
        heard = source_ask(heard_text or theme or "")
        ask = replace(heard, generated=True, vocal=heard.vocal or kind != "instrumental",
                      robot_voice=heard.robot_voice or kind == "robot_song", genre=heard.genre or style or "")
        return self._request.order("track", theme or heard_text or "", turn_id=turn_id, heard_text=heard_text,
                                   ask=ask)


def bridge_source(dj: DjSetTool, theme: str, set_id: str,
                  seed: int) -> Callable[[str], Tuple[Callable[[str], Any], int]]:
    """Мост сета из генерации (В4): ``style -> (deck -> Track, bpm)`` — трек 1 синтезом в сильном стиле."""
    def bridge(style: str) -> Tuple[Callable[[str], Any], int]:
        profile = dj.theme_profile(theme, style, dj.quick_find)  # мост — кодом, без вопроса LLM
        plan = seeded_plan(profile, seed, set_id=set_id)
        melodies, materials = dj._melodies(profile.hook_ids), dj.plan_materials(plan)  # noqa: SLF001
        return (lambda deck: compose(plan, 1, melodies=melodies, deck=deck, materials=materials)), plan.bpm

    return bridge


class SynthFallback(NamedTuple):
    """Синтез-сет запасом (В13): стиль, окно и фраза кода, почему не кэш и не генерация («» — молчать)."""

    style: str
    window: Optional[str]
    note: str = ""


def noted(result: Dict[str, Any], note: str) -> Dict[str, Any]:
    """Фраза пути перед фразой запуска (``media_phrases.launch_text``), если сет заиграл."""
    return {**result, "source_note": note} if note and result.get("ok") else result


def generated_set(dj: DjSetTool, theme: str, style: Tuple[str, Optional[str]], persona: Optional[str],
                  length: Tuple[int, str], heard_text: Optional[str],
                  hints: Tuple[Optional[List[str]], Optional[int]] = (None, None),
                  ) -> Tuple[Optional[Dict[str, Any]], SynthFallback]:
    """Сет из кэша и генерации (ADR-0158 PR-8, решение Шифу 14.10 — любой сет: тема, стиль, auto) →
    ``(результат, …)``; ни у одного трека нет пути кэша/генерации — ``(None, запас)``: синтез-сет первым стилем,
    который синтез играет, с фразой почему; исполнителя нет — ``(None, запас)`` стилем сета без фразы, как раньше.
    Синтез стили сета не играет (джаз, рок без денег и кэша) — отказ с фразой кода, идущий сет не снят.
    ``style`` — ``(стиль сета, окно)``, ``length`` — ``(длина, откуда)``."""
    from .gen.gen_session import synth_set_note

    key, window = style
    makers = dj.gen_sets
    if makers is None:
        return None, SynthFallback(key, window)
    genre = window if window in kn.GEN_STYLES else key
    plan = makers.plan(theme, genre, heard_text, length[0], named=length[1] == "названо", styles=hints[0] or (),
                       minutes=hints[1])
    fallback = plan.synth_style()
    if not plan.played() and fallback is not None:  # окно — у стиля сета, не у стиля запаса
        return None, SynthFallback(fallback, window if fallback == key else None,
                                   synth_set_note(plan.gen_reason(), fallback))
    seed = dj._seed()  # noqa: SLF001 — тот же сид, что у сета синтезом
    set_id = f"gen{seed % 100000:05d}"
    reasoner = dj._reasoner  # noqa: SLF001
    result = makers.start(plan, dj._owner, theme=theme, persona=persona, set_id=set_id,  # noqa: SLF001
                          bridge=bridge_source(dj, theme, set_id, seed), speak=dj._speak,  # noqa: SLF001
                          line_ask=reasoner.ask if reasoner.enabled else None, replace_set=dj._drop_session)  # noqa: SLF001
    session = result.pop("session", None)
    if session is not None:
        dj.adopt(session, theme, persona)
    return {**result, "seed": seed}, SynthFallback(key, window)


def bridge_find(dj: Any, gen_failed: str) -> Optional[ThemeFinder]:
    """Мост ожидания генерации (``gen_failed == "pending"``: песня пишется) — поиск темы без LLM (``quick_find``):
    мост не задерживает ответ тула; иначе — обычный поиск со связями."""
    return getattr(dj, "quick_find", None) if gen_failed == "pending" else None


def play_bridged(req: RequestMusicTool, program: Any, staged: Dict[str, Any], played: Tuple[Any, bool],
                 held: bool) -> Tuple[Dict[str, Any], Any]:
    """Запуск собранного заказа: ``(результат, мост ожидания | None)``. Мост ожидания генерации (``held``) — сет из
    двух треков ``BridgeHold``: мост держит звук до результата/провала, результат входит кроссфейдом (ADR-0158 PR-8)."""
    facts, bridge_model = track_facts(staged), staged.pop("bridge_model", None) or played[0]
    if not held or bridge_model is None:
        return confirmed({**req._owner.play(program, dj=facts, once=played[1]), **staged}, req._confirm), None  # noqa: SLF001
    result = confirmed({**hold_start(req)(bridge_model, program, facts), **staged}, req._confirm)  # noqa: SLF001
    if not result.get("ok"):
        req.hold = None
    return result, req.hold


def hold_start(req: RequestMusicTool) -> Callable[[Any, Any, Mapping[str, Any]], Dict[str, Any]]:
    """Запуск моста ожидания генерации (``BridgeHold``, одна реализация для синтеза-моста, кавера и трека кэша под
    песню): ``(модель моста, программа, факты) -> результат play``; мост — ``req.hold`` (песня/кавер входят вместо
    него кроссфейдом)."""
    from .gen.gen_session import BridgeHold

    def start(model: Any, program: Any, facts: Mapping[str, Any]) -> Dict[str, Any]:
        req.hold = BridgeHold(req._owner, model, program, req.gen_library, facts=dict(facts),  # noqa: SLF001
                              logger=req._log())  # noqa: SLF001
        return req.hold.start()

    return start


def play_cache(play: Callable[..., MCPToolResult], route: Route, what: str,
               hold: Optional[Callable[..., Dict[str, Any]]] = None) -> MCPToolResult:
    """Трек кэша — ``play_library_track`` на деке владельца (ADR-0158 PR-2): сет закрывает и деку заменяет он
    сам, ``ok`` — только по ``started`` запуска; ``track_id`` результата — id запуска на деке (его ``started`` ждёт
    роутер медиакоманд). ``hold`` (:func:`hold_start`) — трек кэша мостом ожидания песни: петля с кроссфейдом, пока
    песня пишется, а не один проход и тишина."""
    track = route.track
    result = play(track.id, hold=hold) if hold is not None else play(track.id)
    data = dict(result.data or {})
    if not result.success:
        return tool_result({"ok": False, "reason": "cache_play_failed", "route": route.id,
                            "detail": result.error or data.get("reason")}, what)
    return tool_result({**data, "ok": True, "source": CACHE, "route": route.id,
                        "title": track.title, "source_message": cache_text(route)}, what)


def is_bridge(route: Route, cover: Any) -> bool:
    """Путь — кавер пьесы, и исполнитель кавера есть: синтез пьесы играет мостом (ADR-0158 PR-4)."""
    return route.kind == GENERATE and route.work is not None and cover is not None


def cover_source(cover: Optional[PieceCover], piece: Optional[Dict[str, Any]], pieces: Any,
                 seed: Callable[[], int]) -> Tuple[Any, Any]:
    """Произведение кавера и трек-источник его мелодии (``PieceCover.source``); кавера нет — только произведение."""
    if piece is None or cover is None:
        return cover_work(piece), None
    return cover.source(piece, pieces, seed())


def unstage(staged: Dict[str, Any]) -> Tuple[Any, bool, Dict[str, Any], Any]:
    """Снять с собранного заказа ``(программа, once, строка истории, модель трека)``; у пьесы — фраза кода о ней
    из результата (ADR-0157 R6), её же скажет голос."""
    program, once, row = staged.pop("program"), staged.pop("once"), staged.pop("history", {})
    model = staged.pop("track", None)
    if "played" in staged:
        from rob_box_voice.core.media_phrases import launch_text
        staged["message"] = launch_text("", staged)
    return program, once, row, model


def _piece_line(pick: Optional[ClassicPick], played: Mapping[str, Any]) -> str:
    """Хвост строки лога ``[classic]``: что играет (источник, запись, название, замена) или «нет»."""
    if pick is None:
        return "нет"
    return f"{played['source']} {pick.melody_id} «{pick.title}» замена={played['replacement_for']}"


__all__ = ["AUTO_STYLE", "CLASSIC_GENRES", "NOT_A_SET_REQUEST", "NOT_FOUND", "NO_MORE", "SET_PLAYING", "STYLE_CHOICES", "Confirm",
           "DjSetTool", "MelodyLookup", "RequestMusicTool", "ThemeFinder", "club_theme", "confirmed", "lead_data",
           "lead_for", "library_melodies", "library_records", "music_busy", "not_found", "set_refusal", "set_style",
           "theme_finder", "theme_links", "tool_result", "word_hooks", "GEN_KINDS", "GEN_STYLE_CHOICES",
           "GenerateMusicTool", "SOURCE_DECLINED", "hold_start", "play_cache"]
