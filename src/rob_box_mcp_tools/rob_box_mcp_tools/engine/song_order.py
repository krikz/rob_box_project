"""Заказ песни голосом робота (ADR-0158 §2.3, В3/В4; PR-7): мост, фраза ожидания, фраза провала — всё кодом.

Вынесено из ``RequestMusicTool`` (бюджет класса, ADR-0145): тот решает путь, этот класс исполняет ветку ``generate``.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable, Dict, Optional, Tuple

from rob_box_music.source import DECLINE, GENERATE, Route, SourceAsk, decline_text

from .gen.song import BUSY_TEXT, SongFailed, SongRequest, SongStudio, failed_words, wait_text

#: Код отказа пути (``tools_v2.SOURCE_DECLINED``; равенство держит тест).
DECLINED = "music_source"


def _declined(message: str) -> Tuple[str, Dict[str, Any]]:
    return "data", {"ok": False, "reason": DECLINED, "message": message}


def bridge_hold(hold: Any, bridge: Any) -> Any:
    """Мост ожидания этого заказа (``BridgeHold``) или ``None``: мост не встал или это трек кэша (свой запуск на деке
    через ``gen_play_from_library``), а не мост ожидания."""
    if hold is None or not bridge.success or hold.bridge_id != (bridge.data or {}).get("track_id"):
        return None
    return hold


def held_play(hold: Any) -> Optional[Callable[[str], Any]]:
    """Песня готова: войдёт с деки моста кроссфейдом (``BridgeHold.deliver``); деку занял другой заказ — не заиграла.
    Моста ожидания нет — ``None`` (студия играет ``gen_play_from_library``)."""
    from ..base import MCPToolResult

    if hold is None:
        return None

    def play(track_id: str) -> Any:
        ok = hold.deliver(track_id)
        return MCPToolResult(success=ok, data={"track_id": hold.bridge_id}, error="" if ok else "мост снят другим заказом")

    return play


class SongOrder:
    """``route`` — ``SourceRouter.route`` (выбор заново без ``generate:*``, В3)."""

    def __init__(self, studio: SongStudio, route: Callable[..., Route], hold: Callable[[], Any] = lambda: None) -> None:
        self._studio = studio
        self._route = route
        self._hold = hold  # мост ожидания (``BridgeHold``, ADR-0158 PR-8): песня входит вместо него кроссфейдом

    def handles(self, route: Route) -> bool:
        """Путь выбрал генерацию песни и студия подключена."""
        return route.kind == GENERATE and self._studio is not None and route.work is None

    def start(self, ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any], wrap: Callable[..., Any]) -> Any:
        """``MCPToolResult`` заказа (ожидание или отказ; ``wrap`` — ``tools_v2.tool_result``). Мост не встал не из-за
        «нечем играть» (сет играет, не нашлось): песню не начинаем, отдаём его результат как есть. ``args`` — ``(intent, text, mood, genre, turn_id, heard_text)`` заказа; мост — тот же ``order`` без просьбы
        петь (кэш по жанру, иначе синтез)."""
        kind, data = self._begin(ask, args, order)
        return data if kind == "raw" else wrap(data, "песня не заиграла")

    def _begin(self, ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any]) -> Tuple[str, Any]:
        studio = self._studio
        try:
            provider, voice = studio.voice()
        except SongFailed as exc:
            return _declined(f"Спеть сейчас не могу: {failed_words(exc.reason)}.")
        if studio.ordered_in(args[4]):  # тот же ход: песня уже пишется, ответ тот же, что у первого вызова
            return "data", {"ok": True, "pending": "song", "source_message": wait_text(provider, voice, False)}
        if studio.busy():
            return _declined(BUSY_TEXT)
        bridge = order(*args, ask=replace(ask, vocal=False, robot_voice=False, generated=False), gen_failed="pending")
        if not bridge.success and (bridge.data or {}).get("reason") != DECLINED:
            return "raw", bridge
        request = SongRequest(str(args[5] or args[1] or ""), ask.genre, str(args[2] or ""))
        hold = bridge_hold(self._hold(), bridge)
        if not studio.launch(request, lambda why: self.failed_text(ask, why, bridge.success, hold), args[4],
                             play=held_play(hold)):
            return _declined(BUSY_TEXT)
        return "data", {**(bridge.data or {}), "ok": True, "pending": "song",
                        "source_message": wait_text(provider, voice, bridge.success)}

    def failed_text(self, ask: SourceAsk, reason: str, bridge: bool, hold: Any = None) -> str:
        """Фраза провала песни. Играет мост — он доиграет финалом (``BridgeHold.fail``); моста нет — выбор заново без
        ``generate:*``, отказ с причиной."""
        if hold is not None:
            hold.fail()
        if bridge:
            return f"Песню сделать не вышло: {failed_words(reason)}. Играет пока своё."
        route = self._route(ask, gen_failed=reason)
        if route.kind == DECLINE:
            return decline_text(replace(route, failed=True))
        return f"Песню сделать не вышло: {failed_words(reason)}."
