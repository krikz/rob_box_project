"""Узкие MCP-тулы движка v2 (ADR-0149 §5.1): ``dj_set(start|stop)`` (PR-5), ``request_music`` (PR-6).

Регистрируются только при ``music_engine: v2`` (``mcp_server._attach_player_owner_v2``); в каталоге
LLM — только при v2 (``music_engine = "v2"``), старые ``compose_music`` & Co. при v2 скрыты.
Параметры трека (темп, тоника, синты, сид) тулы не принимают: тема → ``theme.seeded_profile``
→ ``set_plan.seeded_plan`` → ``arrange.compose`` → ``render``. Вызывают их роутер медиакоманд
(без LLM) и LLM.

Результат ``ok: true`` — только когда плеер прислал ``started`` этого трека (``confirm``, A14:
ни одного ``ok:true`` без звука); ``rejected`` или тишина за время ожидания — ``ok: false``.
Classic-мелодии («поставь Калинку») v2 пока не играет (PR-11, решение Шифу В5): ``request_music``
с ``intent=melody``/``genre=classical|folk`` отдаёт их старому пути (``classic`` — ``named_play``:
``lookup_melody`` → ``compose_music``), без второй реализации.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from .session import SetSession, compose_source

#: Мелодии по ``id`` для хука темы: ``ids -> {id: rtttl}``.
MelodyLookup = Callable[[Iterable[str]], Dict[str, str]]
#: ``track_id -> MusicEvent(started|rejected) | None`` — ждёт событие плеера (``MusicEventLog.wait``).
Confirm = Callable[[Optional[str]], Any]

#: ``request_music`` к старому пути (classic, В5).
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


def _rtttl_library() -> Any:
    from ..core.rtttl_library import RtttlLibrary

    return RtttlLibrary()


def named_play_classic(execute_tool: Callable[..., Any]) -> Callable[[str], Dict[str, Any]]:
    """Classic по просьбе LLM — тот же ``named_play``, что у роутера (``lookup_melody`` → ``compose_music``).

    ``execute_tool(name, **args) -> MCPToolResult`` — реестр ``mcp_server``. Название берёт грамматика
    заказа по имени («поставь калинку» → «калинку»); не разобрала — ищутся слова целиком.
    """
    import asyncio

    from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
    from rob_box_voice.core.named_play import NamedPlayStatus, run_named_play

    async def call(name: str, args: Dict[str, Any]) -> Any:
        result = execute_tool(name, **args)
        return bool(result.success), repr(result.data if result.data is not None else result.error)

    def play(text: str) -> Dict[str, Any]:
        command = parse_media_command(text)
        title = command.name if command.intent is MediaIntent.PLAY_NAMED else text
        outcome = asyncio.run(run_named_play(call, title))
        played = outcome.status is NamedPlayStatus.PLAYED
        return {"ok": played, "engine": "v1", "title": title,
                "reason": None if played else f"{outcome.status.value}: {outcome.reason}".rstrip(": ")}

    return play


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

    music_engine = "v2"

    def __init__(self, node: Any, owner: Any, melodies: Optional[MelodyLookup] = None, *,
                 seed: Callable[[], int] = lambda: int(time.time()), confirm: Optional[Confirm] = None) -> None:
        super().__init__(node)
        self._owner = owner
        self._melodies = melodies or library_melodies(_rtttl_library)
        self._seed = seed
        self._confirm = confirm
        self._lock = threading.Lock()
        self._session: Optional[SetSession] = None

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

    def _start(self, theme: str, persona: Optional[str]) -> Dict[str, Any]:
        if self._session is not None:
            self._session.stop("new_set")
        profile = seeded_profile(theme)
        set_seed = self._seed()
        set_id = f"set{set_seed % 100000:05d}"
        plan = seeded_plan(profile, set_seed, set_id=set_id)  # один план на сет = один темп
        source = compose_source(plan, self._melodies(profile.hook_ids))
        session = SetSession(self._owner, source, set_id=set_id, bpm=plan.bpm,
                             dj={"theme": theme, "persona": persona},
                             logger=self.node.get_logger() if self.node is not None else None)
        result = session.start()
        self._session = session if result.get("ok") else None
        return {**result, "theme": theme, "theme_source": profile.source, "seed": set_seed}

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

    music_engine = "v2"

    def __init__(self, node: Any, owner: Any, dj: DjSetTool, melodies: Optional[MelodyLookup] = None, *,
                 seed: Callable[[], int] = lambda: int(time.time()), confirm: Optional[Confirm] = None,
                 classic: Optional[Callable[[str], Dict[str, Any]]] = None) -> None:
        super().__init__(node)
        self._owner = owner
        self._dj = dj
        self._melodies = melodies or library_melodies(_rtttl_library)
        self._seed = seed
        self._confirm = confirm
        self._classic = classic

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
        if intent == "melody" or genre in CLASSIC_GENRES:
            return self._play_classic(text)
        self._dj.close_set("request_music")
        return tool_result(confirmed(self._play_club(text, mood), self._confirm), "музыка не заиграла")

    def _play_club(self, text: str, mood: Optional[str]) -> Dict[str, Any]:
        """Трек плана с энергией настроения: ``compose`` → ``render`` → дека A."""
        profile = seeded_profile(text)
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

    def _play_classic(self, text: str) -> MCPToolResult:
        """Classic — старый путь (В5): ``named_play`` через реестр, та же реализация, что у роутера."""
        if self._classic is None:
            return MCPToolResult(success=False, error="мелодии по названию сейчас недоступны")
        return tool_result(self._classic(text), "мелодия не заиграла")


__all__ = ["CLASSIC_GENRES", "Confirm", "DjSetTool", "MelodyLookup", "RequestMusicTool", "confirmed",
           "library_melodies", "named_play_classic", "tool_result"]
