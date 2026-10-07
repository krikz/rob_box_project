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
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.dj_line import now_playing_text, persona_title, set_not_found_text
from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import DEFAULT_TRACKS, MAX_TRACKS, seeded_plan, set_tracks
from rob_box_music.theme import ThemeProfile, match_style_text, seeded_profile, style_for

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from .classic import ClassicPick, classic_picker
from .dj_lines import TransitionLines, Titles, latin_fold, library_titles
from .reasoner import SetPlanBox, SetReasoner
from .score_library import PlanMaterials, ScoreLibrary, seed_plan
from .search import ThemeHits, part_query, ThemeQuery, theme_search
from .session import SetMemory, SetSession, plan_source
from .theme_grounding import grounded_theme
from .theme_links import ThemeLinks

_LOG = logging.getLogger(__name__)

#: Мелодии по ``id`` для хука темы: ``ids -> {id: rtttl}``.
MelodyLookup = Callable[[Iterable[str]], Dict[str, str]]
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


def library_melodies(library_factory: Callable[[], Any]) -> MelodyLookup:
    """``{id: rtttl}`` из RTTTL-библиотеки; библиотека открывается при первом сете.

    Берётся только точное совпадение имени: ``RtttlLibrary.get`` иначе вернёт лучшую по словам
    запись — для хука темы это была бы чужая мелодия (честный пропуск лучше подмены).
    """
    box: Dict[str, Any] = {}

    def lookup(ids: Iterable[str]) -> Dict[str, str]:
        if "lib" not in box:
            box["lib"] = library_factory()
        found = {}
        for melody_id in ids:
            record = box["lib"].get(melody_id) or {}
            if str(record.get("name", "")).lower() == melody_id.lower() and record.get("rtttl"):
                found[melody_id] = record["rtttl"]
        return found

    return lookup


def theme_finder(library_factory: Callable[[], Any], links: Optional[ThemeLinks] = None) -> ThemeFinder:
    """``тема -> ThemeHits`` по её словам (``engine.search.theme_search``); библиотека — при первой теме. Часть без
    находок — через связи реестра и проверенные каталогом строки поиска LLM (``engine.theme_links``, #3493)."""
    box: Dict[str, Any] = {}

    def find(theme: str) -> ThemeHits:
        if not theme.strip():
            return ThemeHits()
        if "lib" not in box:
            box["lib"] = library_factory()
        hits = theme_search(box["lib"], theme)
        if links is None:
            return hits
        try:
            return links.expand(box["lib"], theme, hits)
        except Exception as exc:  # noqa: BLE001 — прямые находки не теряются из-за хранилища связей
            _LOG.warning(f"⚠️ [dj_set] связи темы «{theme}» упали: {type(exc).__name__}: {exc}")
            return hits

    return find


def _shared(factory: Callable[[], Any]) -> Callable[[], Any]:
    """Одна библиотека на хуки и поиск темы: открывается при первом обращении."""
    box: Dict[str, Any] = {}

    def get() -> Any:
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


def tool_result(data: Dict[str, Any], what: str) -> MCPToolResult:
    if data.get("ok"):
        return MCPToolResult(success=True, data=data)
    if data.get("reason") == NOT_FOUND and data.get("message"):  # отказ сета: фраза кода, её скажет голос
        return MCPToolResult(success=False, data=data, error=data["message"])
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
        library = _shared(_rtttl_library)
        self._melodies = melodies or library_melodies(library)
        self._find = finder or theme_finder(library, theme_links(node, self._reasoner))
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
        # треки прошлых сетов (``history`` — music_history в БД): разнообразие между сетами (A13, I17)
        self._memory = SetMemory(store=history)

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
                tracks: Optional[int] = None, heard_tracks: Optional[int] = None,
                turn_id: Optional[str] = None, heard_text: Optional[str] = None) -> MCPToolResult:
        """``heard_tracks``, ``turn_id`` и ``heard_text`` — скрытые аргументы хода (``llm_adapter.TURN_CONTEXT_ARGS``):
        длина сета из слов человека по грамматике (``set_length_words``; есть — она решает, а не ``tracks`` от LLM,
        ADR-0148), ход, запустивший сет (:meth:`started_in_turn`), и реплика хода: тема LLM не из неё (взята из
        истории диалога, 06.10 18:04) — тема реплики (``theme_grounding``)."""
        tracks = heard_tracks or tracks
        if action == "start":
            theme = self._heard_theme(theme, heard_text)
            persona = persona_title(persona) or None  # «диджей X» одним видом: и от роутера, и от LLM
        with self._lock:
            if action == "stop":
                return self._stop()
            if action == "start":
                try:
                    key = set_style(style, theme or "", heard_text)
                    named = None if tracks is None else set_tracks(tracks)
                except ValueError as exc:
                    return MCPToolResult(success=False, error=str(exc))
                result = self._start(theme or "", persona, key, named)
            else:
                return MCPToolResult(success=False, error=f"action={action!r}: есть только start и stop")
        result = confirmed(result, self._confirm)  # ждём started вне замка
        self._session_turn = turn_id if result.get("ok") else None
        return tool_result(result, "сет не начался")

    def _heard_theme(self, theme: Optional[str], heard_text: Optional[str]) -> Optional[str]:
        """Тема сета, опирающаяся на реплику хода; подмену код пишет в лог честной строкой."""
        grounded, replaced = grounded_theme(theme, heard_text)
        if not replaced:
            return theme
        logger = self.node.get_logger() if self.node is not None else None
        if not theme:  # команда роутера или LLM без темы: тему выделил код из слов реплики, подмены не было
            (logger or _LOG).info(f"🎛️ [dj_set] тема из слов реплики: {grounded!r}")
            return grounded
        (logger or _LOG).warning(f"🎛️ [dj_set] тема LLM не из текущей реплики → из реплики: LLM={theme!r} → {grounded!r}")
        return grounded

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

    def theme_profile(self, theme: str, style: str = kn.DEFAULT_STYLE) -> ThemeProfile:
        """Seeded-профиль темы с мелодиями по её словам; поиск упал — профиль без находок, причина в лог."""
        log = self.node.get_logger() if self.node is not None else _LOG
        try:
            hits = self._find(theme)
        except Exception as exc:  # noqa: BLE001 — поиск не держит звук: сет играет пул по хешу темы
            log.warning(f"⚠️ [dj_set] поиск мелодий темы «{theme}» упал: {type(exc).__name__}: {exc}")
            hits = ThemeHits()
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
        ids = [t.material for t in plan.tracks if t.material]
        if not ids:
            return {}
        materials = PlanMaterials(self._scores, ids, logger or _LOG)
        started = time.perf_counter()
        first = materials.get(ids[0])
        (logger or _LOG).info(f"🎼 [dj_set] {plan.set_id} материалы треков: {[t.material for t in plan.tracks]}; "
                              f"№1 {ids[0]} {'загружен' if first is not None else 'НЕ загружен'} за "
                              f"{(time.perf_counter() - started) * 1000:.1f} мс")
        return materials

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
               tracks: Optional[int] = None) -> Dict[str, Any]:
        length, why = self.set_length(tracks)
        profile = self.theme_profile(theme, style)
        if self._not_found:  # названное не нашлось — не пул по хешу темы вместо него; идущий сет не трогаем
            return {"ok": False, "reason": NOT_FOUND, "theme": theme, "missing": list(self._not_found),
                    "message": set_not_found_text(self._not_found)}
        self._session_turn = None
        if self._session is not None:
            self._session.stop("new_set")
        set_seed = self._seed()
        set_id = f"set{set_seed % 100000:05d}"
        logger = self.node.get_logger() if self.node is not None else None
        plan = seed_plan(self._scores, profile, set_seed, length, set_id, self._memory.peek(), logger or _LOG)
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
            reasoner = self._reasoner.request(set_id, theme, plan, lambda ref: (box.apply(ref), session.replan()))
        return {**result, "theme": theme, "style": plan.style, "genre": plan.genre, "theme_source": profile.source,
                "seed": set_seed, "tracks": length, "reasoner": reasoner}

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
                 classic: Optional[Callable[..., ClassicPick]] = None) -> None:
        super().__init__(node)
        self._owner = owner
        self._dj = dj
        self._melodies = melodies or library_melodies(_rtttl_library)
        self._seed = seed
        self._confirm = confirm
        self._classic = classic or classic_picker(_rtttl_library)

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
                genre: Optional[str] = None, turn_id: Optional[str] = None) -> MCPToolResult:
        """``turn_id`` — скрытый аргумент хода (``llm_adapter.TURN_CONTEXT_ARGS``): сет этого хода не снимается."""
        set_id = self._dj.started_in_turn(turn_id)
        if set_id is not None:
            return tool_result({"ok": False, "reason": SET_PLAYING, "set_id": set_id,
                                "detail": "сет запущен в этом ходе и играет — заказ его не заменяет"},
                               "музыка не заменена")
        classic = intent == "melody" or genre in CLASSIC_GENRES
        what = "мелодия не заиграла" if classic else "музыка не заиграла"
        staged = self._stage_classic(text) if classic else self._stage_club(text, mood)
        if "program" not in staged:  # не нашлось / не собралось: идущий сет играет дальше
            return tool_result(staged, what)
        program, once = staged.pop("program"), staged.pop("once")
        self._dj.close_set("request_music")  # деку снимаем только под собранный заказ
        result = self._owner.play(program, dj={"enabled": False, "title": staged["title"]}, once=once)
        return tool_result(confirmed({**result, **staged}, self._confirm), what)

    def _stage_club(self, text: str, mood: Optional[str]) -> Dict[str, Any]:
        """Трек плана с энергией настроения: ``compose`` → ``render`` (дека A) — собран, но не запущен."""
        profile = self._dj.theme_profile(text)
        seed = self._seed()
        plan = seeded_plan(profile, seed, set_id=f"req{seed % 100000:05d}")
        energy = kn.MOOD_ENERGY.get(mood or "", kn.ENERGY_WAVE[0])
        track_no = kn.ENERGY_WAVE.index(energy) + 1
        try:
            program = render(compose(plan, track_no, melodies=self._melodies(profile.hook_ids), deck="A",
                                     materials=self._dj.plan_materials(plan)), "A")
        except Exception as exc:  # noqa: BLE001 — отказ громкий (I25), звука нет
            return self._owner.reject(f"{plan.set_id}:{track_no:02d}:A", "compose_error",
                                      f"{type(exc).__name__}: {exc}")
        title = f"{profile.theme or 'клубный трек'} · {plan.bpm} BPM"
        return {"program": program, "once": False, "title": title, "bpm": plan.bpm, "energy": energy,
                "theme_source": profile.source}

    def _stage_classic(self, text: str) -> Dict[str, Any]:
        """Мелодия по названию: песня v2 (темп и тональность — из RTTTL), один проход формы — собрана, не запущена.

        Название берёт грамматика заказа по имени («поставь калинку» → «калинку»); не разобрала — ищутся
        слова целиком. Поиск — тот же, что у ``lookup_melody`` (``engine.classic.find_record``).
        """
        from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command

        command = parse_media_command(text)
        query = command.name if command.intent is MediaIntent.PLAY_NAMED else text
        seed = self._seed()
        try:
            pick = self._classic(query, seed=seed)
        except Exception as exc:  # noqa: BLE001 — отказ громкий (I25), звука нет
            return self._owner.reject(f"classic:{query}", "compose_error", f"{type(exc).__name__}: {exc}")
        if not pick.found:
            return {"ok": False, "found": False, "reason": "not_found", "detail": pick.reason, "query": query}
        return {"program": pick.program, "once": True, "found": True, "title": pick.title,
                "melody_id": pick.melody_id, "bpm": pick.bpm, "key": pick.key}


__all__ = ["AUTO_STYLE", "CLASSIC_GENRES", "NOT_FOUND", "SET_PLAYING", "STYLE_CHOICES", "Confirm", "DjSetTool",
           "MelodyLookup", "RequestMusicTool", "ThemeFinder", "confirmed", "library_melodies", "music_busy", "not_found",
           "set_style", "theme_finder", "theme_links", "tool_result"]
