"""Узкие MCP-тулы движка v2 (ADR-0149 §5.1): ``dj_set(start|stop)`` (PR-5), ``request_music`` (PR-6).

Регистрируются ``mcp_server._attach_player_owner_v2``; старый путь (``compose_music`` & Co.)
удалён вместе с флагом выбора движка (PR-13/PR-15).
Параметры трека (темп, тоника, синты, сид) тулы не принимают: тема → мелодии по её словам
(``engine.search.theme_hooks``, #3399) → ``theme.seeded_profile``
→ ``set_plan.seeded_plan`` → ``arrange.compose`` → ``render``. Вызывают их роутер медиакоманд
(без LLM) и LLM.

Результат ``ok: true`` — только когда плеер прислал ``started`` этого трека (``confirm``, A14:
ни одного ``ok:true`` без звука); ``rejected`` или тишина за время ожидания — ``ok: false``.
Classic («поставь Калинку», PR-11): ``request_music`` с ``intent=melody``/``genre=classical|folk`` играет
песню движка v2 (``engine.classic``: поиск и ``harmonize`` старой библиотеки → ``arrange.song`` → ``render``),
один проход формы, потом ``finished``. Не нашлась — ``ok: false, found: false`` (I16).

PR-10: после ``started`` сета ``dj_set`` в фоне спрашивает ``SetReasoner`` (LLM) профиль сета; ответ ок —
план подменяется со следующего несыгранного трека (``SetSession.replan``), иначе сет целиком seeded.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile, seeded_profile

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from .classic import ClassicPick, classic_picker
from .reasoner import SetPlanBox, SetReasoner
from .search import theme_hooks
from .session import SetMemory, SetSession, plan_source

_LOG = logging.getLogger(__name__)

#: Мелодии по ``id`` для хука темы: ``ids -> {id: rtttl}``.
MelodyLookup = Callable[[Iterable[str]], Dict[str, str]]
#: Мелодии по словам темы: ``тема -> (id, …)`` (лучшие первыми).
ThemeFinder = Callable[[str], Tuple[str, ...]]
#: ``track_id -> MusicEvent(started|rejected) | None`` — ждёт событие плеера (``MusicEventLog.wait``).
Confirm = Callable[[Optional[str]], Any]

#: ``request_music`` в classic-песню (PR-11).
CLASSIC_GENRES = ("classical", "folk")


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


def theme_finder(library_factory: Callable[[], Any]) -> ThemeFinder:
    """``тема -> id`` мелодий по её словам (``engine.search.theme_hooks``); библиотека — при первой теме."""
    box: Dict[str, Any] = {}

    def find(theme: str) -> Tuple[str, ...]:
        if not theme.strip():
            return ()
        if "lib" not in box:
            box["lib"] = library_factory()
        return theme_hooks(box["lib"], theme)

    return find


def _shared(factory: Callable[[], Any]) -> Callable[[], Any]:
    """Одна библиотека на хуки и поиск темы: открывается при первом обращении."""
    box: Dict[str, Any] = {}

    def get() -> Any:
        if "lib" not in box:
            box["lib"] = factory()
        return box["lib"]

    return get


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


def tool_result(data: Dict[str, Any], what: str) -> MCPToolResult:
    if data.get("ok"):
        return MCPToolResult(success=True, data=data)
    return MCPToolResult(success=False, data=data, error=f"{what}: {data.get('reason')}")


class DjSetTool(MCPTool):
    """«Ты диджей» v2: сет без LLM на пути звука; один сет за раз."""

    def __init__(self, node: Any, owner: Any, melodies: Optional[MelodyLookup] = None, *,
                 seed: Callable[[], int] = lambda: int(time.time()), confirm: Optional[Confirm] = None,
                 reasoner: Optional[SetReasoner] = None, speak: Optional[Callable[[str], None]] = None,
                 finder: Optional[ThemeFinder] = None, history: Any = None,
                 tracks_dir: Optional[str] = None) -> None:
        super().__init__(node)
        self._owner = owner
        self._reasoner = reasoner or SetReasoner(enabled=False)
        self._speak = speak
        library = _shared(_rtttl_library)
        self._melodies = melodies or library_melodies(library)
        self._find = finder or theme_finder(library)
        self._seed = seed
        self._tracks_dir = tracks_dir or None
        self._confirm = confirm
        self._lock = threading.Lock()
        self._session: Optional[SetSession] = None
        # треки прошлых сетов (``history`` — music_history в БД): разнообразие между сетами (A13, I17)
        self._memory = SetMemory(store=history)

    @property
    def name(self) -> str:
        return "dj_set"

    @property
    def description(self) -> str:
        return ("Диджей-сет: action=start — начать сет на тему theme (треки, переходы и темп решает код), "
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
                persona: Optional[str] = None) -> MCPToolResult:
        with self._lock:
            if action == "stop":
                return self._stop()
            if action == "start":
                result = self._start(theme or "", persona)
            else:
                return MCPToolResult(success=False, error=f"action={action!r}: есть только start и stop")
        return tool_result(confirmed(result, self._confirm), "сет не начался")  # ждём started вне замка

    def theme_profile(self, theme: str) -> ThemeProfile:
        """Seeded-профиль темы с мелодиями по её словам; поиск упал — профиль без находок, причина в лог."""
        log = self.node.get_logger() if self.node is not None else _LOG
        try:
            found = self._find(theme)
        except Exception as exc:  # noqa: BLE001 — поиск не держит звук: сет играет пул по хешу темы
            log.warning(f"⚠️ [dj_set] поиск мелодий темы «{theme}» упал: {type(exc).__name__}: {exc}")
            found = ()
        profile = seeded_profile(theme, found=found)
        log.info(f"🎛️ [dj_set] тема «{theme}»: source={profile.source} row={profile.row} "
                 f"хуки={list(profile.hook_ids)}")
        return profile

    def _start(self, theme: str, persona: Optional[str]) -> Dict[str, Any]:
        if self._session is not None:
            self._session.stop("new_set")
        profile = self.theme_profile(theme)
        set_seed = self._seed()
        set_id = f"set{set_seed % 100000:05d}"
        plan = seeded_plan(profile, set_seed, set_id=set_id)  # один план на сет = один темп
        logger = self.node.get_logger() if self.node is not None else None
        box = SetPlanBox(plan, self._melodies, speak=self._speak, logger=logger)
        base = plan_source(box.current, self._memory)
        session = SetSession(self._owner, lambda no, deck: box.compose_mark(base(no, deck)), set_id=set_id,
                             bpm=plan.bpm, dj={"theme": theme, "persona": persona}, logger=logger,
                             on_track_started=box.on_started, tracks_dir=self._tracks_dir)
        result = session.start()
        self._session = session if result.get("ok") else None
        reasoner = None
        if self._session is not None:  # звук уже поставлен seeded-планом; LLM — в фоне (§4.8)
            reasoner = self._reasoner.request(set_id, theme, plan, lambda ref: (box.apply(ref), session.replan()))
        return {**result, "theme": theme, "theme_source": profile.source, "seed": set_seed, "reasoner": reasoner}

    def close_set(self, reason: str) -> None:
        """Закрыть идущий сет: деку занимает другой запрос (``request_music``)."""
        with self._lock:
            session, self._session = self._session, None
        if session is not None:
            session.stop(reason)

    def _stop(self) -> MCPToolResult:
        """Стоп сета, а без сета — деки v2 (одиночный трек ``request_music``)."""
        session, self._session = self._session, None
        if session is None:
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
                genre: Optional[str] = None) -> MCPToolResult:
        self._dj.close_set("request_music")
        if intent == "melody" or genre in CLASSIC_GENRES:
            return tool_result(confirmed(self._play_classic(text), self._confirm), "мелодия не заиграла")
        return tool_result(confirmed(self._play_club(text, mood), self._confirm), "музыка не заиграла")

    def _play_club(self, text: str, mood: Optional[str]) -> Dict[str, Any]:
        """Трек плана с энергией настроения: ``compose`` → ``render`` → дека A."""
        profile = self._dj.theme_profile(text)
        seed = self._seed()
        plan = seeded_plan(profile, seed, set_id=f"req{seed % 100000:05d}")
        energy = kn.MOOD_ENERGY.get(mood or "", kn.ENERGY_WAVE[0])
        track_no = kn.ENERGY_WAVE.index(energy) + 1
        try:
            program = render(compose(plan, track_no, melodies=self._melodies(profile.hook_ids), deck="A"), "A")
        except Exception as exc:  # noqa: BLE001 — отказ громкий (I25), звука нет
            return self._owner.reject(f"{plan.set_id}:{track_no:02d}:A", "compose_error",
                                      f"{type(exc).__name__}: {exc}")
        title = f"{profile.theme or 'клубный трек'} · {plan.bpm} BPM"
        result = self._owner.play(program, dj={"enabled": False, "title": title})
        return {**result, "title": title, "bpm": plan.bpm, "energy": energy, "theme_source": profile.source}

    def _play_classic(self, text: str) -> Dict[str, Any]:
        """Мелодия по названию: песня v2 (темп и тональность — из RTTTL), один проход формы.

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
        result = self._owner.play(pick.program, dj={"enabled": False, "title": pick.title}, once=True)
        return {**result, "found": True, "title": pick.title, "melody_id": pick.melody_id, "bpm": pick.bpm,
                "key": pick.key}


__all__ = ["CLASSIC_GENRES", "Confirm", "DjSetTool", "MelodyLookup", "RequestMusicTool", "ThemeFinder", "confirmed",
           "library_melodies", "theme_finder", "tool_result"]
