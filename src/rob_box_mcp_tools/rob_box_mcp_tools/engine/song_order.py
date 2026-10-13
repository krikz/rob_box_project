"""Заказ песни голосом робота (ADR-0158 §2.3, В3/В4; PR-7): мост, фраза ожидания, фраза провала — всё кодом.

Вынесено из ``RequestMusicTool`` (бюджет класса, ADR-0145): тот решает путь, этот класс исполняет ветку ``generate``.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable, Dict, Tuple

from rob_box_music.source import DECLINE, Route, SourceAsk, decline_text

from .gen.song import BUSY_TEXT, SongFailed, SongRequest, SongStudio, failed_words, wait_text

#: Код отказа пути (``tools_v2.SOURCE_DECLINED``; равенство держит тест).
DECLINED = "music_source"


def _declined(message: str) -> Tuple[str, Dict[str, Any]]:
    return "data", {"ok": False, "reason": DECLINED, "message": message}


class SongOrder:
    """``route`` — ``SourceRouter.route`` (выбор заново без ``generate:*``, В3)."""

    def __init__(self, studio: SongStudio, route: Callable[..., Route]) -> None:
        self._studio = studio
        self._route = route

    def start(self, ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any]) -> Tuple[str, Any]:
        """``("data", словарь)`` — результат заказа (ожидание или отказ); ``("raw", MCPToolResult)`` — мост не встал
        не из-за «нечем играть» (сет играет, не нашлось): песню не начинаем, отдаём его результат как есть.
        ``args`` — ``(intent, text, mood, genre, turn_id, heard_text)`` заказа; мост — тот же ``order`` без просьбы
        петь (кэш по жанру, иначе синтез)."""
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
        if not studio.launch(request, lambda why: self.failed_text(ask, why, bridge.success), args[4]):
            return _declined(BUSY_TEXT)
        return "data", {**(bridge.data or {}), "ok": True, "pending": "song",
                        "source_message": wait_text(provider, voice, bridge.success)}

    def failed_text(self, ask: SourceAsk, reason: str, bridge: bool) -> str:
        """Фраза провала песни. Играет мост — он и дальше; моста нет — выбор заново без ``generate:*``, отказ с причиной."""
        if bridge:
            return f"Песню сделать не вышло: {failed_words(reason)}. Играет пока своё."
        route = self._route(ask, gen_failed=reason)
        if route.kind == DECLINE:
            return decline_text(replace(route, failed=True))
        return f"Песню сделать не вышло: {failed_words(reason)}."
