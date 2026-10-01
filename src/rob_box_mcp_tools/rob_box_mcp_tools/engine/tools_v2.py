"""Узкие MCP-тулы движка v2 (ADR-0149 §5.1). PR-5: ``dj_set(start|stop)``.

Регистрируется только при ``music_engine: v2`` (``mcp_server._attach_player_owner_v2``).
Параметры трека (темп, тоника, синты, сид) тул не принимает: тема → ``theme.seeded_profile``
→ ``SetSession`` с источником ``compose_source``. LLM тул пока не видит (``llm_visible=False``):
вызывают его роутер медиакоманд и LLM с PR-6, а до того — оператор/харнесс по ``/mcp/execute``.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

from rob_box_music.theme import seeded_profile

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from .session import SetSession, compose_source

#: Мелодии по ``id`` для хука темы: ``ids -> {id: rtttl}``.
MelodyLookup = Callable[[Iterable[str]], Dict[str, str]]


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


class DjSetTool(MCPTool):
    """«Ты диджей» v2: сет без LLM на пути звука; один сет за раз."""

    def __init__(self, node: Any, owner: Any, melodies: Optional[MelodyLookup] = None, *,
                 seed: Callable[[], int] = lambda: int(time.time())) -> None:
        super().__init__(node)
        self._owner = owner
        self._melodies = melodies or library_melodies(_rtttl_library)
        self._seed = seed
        self._lock = threading.Lock()
        self._session: Optional[SetSession] = None

    @property
    def name(self) -> str:
        return "dj_set"

    @property
    def description(self) -> str:
        return ("Диджей-сет движка v2: action=start — начать сет на тему theme (треки, переходы и темп "
                "решает код), action=stop — закончить сет.")

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter("action", "string", "start — начать сет, stop — закончить", enum=["start", "stop"]),
            MCPToolParameter("theme", "string", "Тема сета словами человека (например «космос»)", required=False),
            MCPToolParameter("persona", "string", "Имя диджея для реплик", required=False),
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
    def llm_visible(self) -> bool:
        return False  # PR-6: роутер медиакоманд и видимость LLM при v2

    def execute(self, action: str = "start", theme: Optional[str] = None,
                persona: Optional[str] = None) -> MCPToolResult:
        with self._lock:
            if action == "stop":
                return self._stop()
            if action == "start":
                return self._start(theme or "", persona)
        return MCPToolResult(success=False, error=f"action={action!r}: есть только start и stop")

    def _start(self, theme: str, persona: Optional[str]) -> MCPToolResult:
        if self._session is not None:
            self._session.stop("new_set")
        profile = seeded_profile(theme)
        set_seed = self._seed()
        set_id = f"set{set_seed % 100000:05d}"
        source = compose_source(profile, set_seed=set_seed, set_id=set_id, melodies=self._melodies(profile.hook_ids))
        session = SetSession(self._owner, source, set_id=set_id, bpm=profile.bpm,
                             dj={"theme": theme, "persona": persona},
                             logger=self.node.get_logger() if self.node is not None else None)
        result = session.start()
        self._session = session if result.get("ok") else None
        data: Dict[str, Any] = {**result, "theme": theme, "theme_source": profile.source, "seed": set_seed}
        if not result.get("ok"):
            return MCPToolResult(success=False, data=data, error=f"сет не начался: {result.get('reason')}")
        return MCPToolResult(success=True, data=data)

    def _stop(self) -> MCPToolResult:
        session, self._session = self._session, None
        if session is None:
            return MCPToolResult(success=True, data={"ok": True, "track_id": None, "was_playing": False})
        return MCPToolResult(success=True, data={**session.stop("user_stop"), "was_playing": True})


__all__ = ["DjSetTool", "MelodyLookup", "library_melodies"]
