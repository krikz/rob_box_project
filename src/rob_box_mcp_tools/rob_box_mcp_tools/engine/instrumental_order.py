"""Заказ одиночного инструментала (ADR-0158 §2.2, В3/В4): мост держит звук, новый трек врезается на такте.

Путь ``generate:instrumental:<модель>`` выбрал чойсер. Дальше — то же, что у песни (``song_order``) и кавера
(``piece_cover``), одной реализацией: мост — тот же ``request_music`` без генерации (кэш по жанру, иначе синтез;
жанр, который синтез не играет, — синтез ближайшего сильного стиля ``GEN_BRIDGE_STYLES``) под ``BridgeHold``;
готовый трек — ``BridgeHold.deliver`` (вход кроссфейдом на ближайшем такте), на его ``started`` — фраза кода с
названием и длительностью из кэша. Провал (деньги, сервис, таймаут) — мост доигрывает финалом, фраза причины от кода;
второго круга генерации нет (В3).
"""

from __future__ import annotations

import logging
from dataclasses import replace
from typing import Any, Callable, Dict, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.source import CACHE, GENERATE, Route, SourceAsk, gen_kind

from .gen.gen_session import bridge_style
from .gen.instrumental import BUSY_TEXT, InstrumentalResult, failed_text, ready_text, wait_text
from .song_order import DECLINED, bridge_hold
from .theme_grounding import order_theme

_LOG = logging.getLogger(__name__)
WHAT = "музыка не заиграла"


def bridge_label(data: Dict[str, Any]) -> str:
    """Что играет мостом — словами для фразы ожидания (факты результата моста, не от LLM)."""
    if data.get("source") == CACHE and data.get("title"):
        return f"сочинённое раньше «{data['title']}»"
    style = kn.GEN_STYLES.get(str(data.get("style") or ""))
    return f"своё — {style.name}" if style else "своё"


class InstrumentalOrder:
    """``maker`` — ``gen.instrumental.InstrumentalMaker``; ``play`` — ``play_library_track`` (моста нет);
    ``speak`` — реплика робота; ``hold`` — последний мост ожидания ``request_music`` (``BridgeHold``)."""

    def __init__(self, maker: Any, play: Optional[Callable[..., Any]], speak: Optional[Callable[[str], Any]],
                 hold: Callable[[], Any] = lambda: None, log: Callable[[], Any] = lambda: _LOG) -> None:
        self._maker, self._play, self._speak, self._hold, self._log = maker, play, speak, hold, log

    def handles(self, route: Route) -> bool:
        return route.kind == GENERATE and route.work is None and gen_kind(route.id) == "instrumental"

    def start(self, route: Route, ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any],
              wrap: Callable[..., Any]) -> Any:
        """``args`` — ``(intent, text, mood, genre, turn_id, heard_text)``; ``wrap`` — ``tools_v2.tool_result``."""
        maker, model, turn = self._maker, route.id.rsplit(":", 1)[-1], args[4]
        theme = order_theme(args[1], args[5])  # тема просьбы — в заказ Suno и во фразы (#3775)
        if maker.ordered_in(turn):  # тот же ход: трек уже пишется, ответ тот же, что у первого вызова
            return wrap({"ok": True, "pending": "instrumental",
                         "source_message": wait_text(ask.genre, model, "", theme)}, WHAT)
        if maker.busy():
            return wrap({"ok": False, "reason": DECLINED, "message": BUSY_TEXT}, WHAT)
        bridge = self._bridge(ask, args, order)
        if not bridge.success and (bridge.data or {}).get("reason") != DECLINED:
            return bridge  # мост не встал не из-за «нечем играть» (сет этого хода, движок): генерацию не начинаем
        hold = bridge_hold(self._hold(), bridge)
        data = dict(bridge.data or {}) if bridge.success else {}
        if not maker.launch(ask.genre, model, theme, turn, lambda result: self.done(result, ask.genre, hold)):
            return wrap({"ok": False, "reason": DECLINED, "message": BUSY_TEXT}, WHAT)
        self._log().info(f"🎛️ [gen instr] {route.id} заказан, тема={theme or '—'!r}, "
                         f"мост={data.get('track_id') or '—'}")
        label = bridge_label(data) if bridge.success else ""
        return wrap({**data, "ok": True, "pending": "instrumental", "route": route.id,
                     "source_message": wait_text(ask.genre, model, label, theme)}, WHAT)

    @staticmethod
    def _bridge(ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any]) -> Any:
        """Мост: тот же заказ без генерации (кэш по жанру, иначе синтез); нечем — синтез ближайшего сильного стиля."""
        bridge = order(*args, ask=replace(ask, generated=False, fresh=False, own=False), gen_failed="pending")
        if bridge.success or (bridge.data or {}).get("reason") != DECLINED:
            return bridge
        return order(*args, ask=SourceAsk(genre=bridge_style(ask.genre)), gen_failed="pending")

    def done(self, result: InstrumentalResult, genre: str, hold: Any) -> None:
        """Итог в фоне: врезать трек вместо моста, фраза — на его ``started``; провал — мост финалом и причина."""
        if result.reason:
            if hold is None or hold.fail():
                self._say(failed_text(result.reason, hold is not None))
            return
        if hold is not None:
            hold.on_live = lambda: self._say(ready_text(result, genre))
            if not hold.deliver(result.track_id, result.title):
                self._log().info(f"🎛️ [gen instr] {result.track_id} в кэше, деку не трогаю: на ней уже не мост "
                                 f"{hold.bridge_id}")
            return
        played = self._play(result.track_id) if self._play is not None else None
        ok = bool(getattr(played, "success", False))
        self._say(ready_text(result, genre) if ok else failed_text("track_play_failed", False))

    def _say(self, text: str) -> None:
        if text and self._speak is not None:
            try:
                self._speak(text)
            except Exception as exc:  # noqa: BLE001 — реплика не должна ронять поток
                self._log().warning(f"🎛️ [gen instr] реплика не сказана: {type(exc).__name__}: {exc}")


__all__ = ["InstrumentalOrder", "bridge_label"]
