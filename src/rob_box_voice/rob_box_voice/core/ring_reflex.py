"""Рефлексы кольца LED ReSpeaker по стадиям хода (ADR-0150 §5.5.2-5.5.3, PR-3b #3690).

Режим кольца на стадии - колонка ``ring`` таблицы :data:`rob_box_dialog.knowledge.REFLEXES`; здесь знание только о
железе: какой режим ReSpeaker включает ``RING_MODES[имя]`` и сколько держится вспышка. Решает код, не LLM; на каждое
включение кольцо отвечает событием (``started``, либо ``skipped{no_device}``, если USB-кольца нет): молчаливых
пропусков нет. Без ROS: кольцо, таймер и ответ приходят снаружи.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional, Tuple

from rob_box_dialog import knowledge as kn

#: Режим кольца -> метод ``PixelRingLite`` (режим вшит в ReSpeaker). ``doa_sector`` - ``listen``: кольцо подсвечивает
#: сторону говорящего; ``breath`` - ``speak`` (мягкая пульсация); ``spin_slow`` - ``think``; ``beat_pulse`` - ``spin``.
HARDWARE_MODES: Dict[str, str] = {
    "off": "off", "breath": "speak", "doa_sector": "listen", "spin_slow": "think", "beat_pulse": "spin",
}
#: Вспышки: режим -> цвет (R, G, B); после ``FLASH_S`` кольцо возвращается в покой.
FLASHES: Dict[str, Tuple[int, int, int]] = {"flash": (0, 255, 0), "flash_red": (255, 0, 0)}
FLASH_S = 0.7
#: Режим без следующей стадии не висит вечно: через ``HOLD_MAX_S`` кольцо возвращается в покой.
HOLD_MAX_S = 60.0
#: Покой кольца - слежение за направлением и голосом (ReSpeaker ``trace``).
REST = "trace"
#: Причина пропуска, когда USB-кольца нет или команда не ушла.
NO_DEVICE = "no_device"
#: Причина пропуска, когда робот в тишине (``silenced``): кольцо погашено, стадии не показывает.
SILENCED = "silenced"


class RingReflex:
    """Стадия хода -> режим кольца и ответ исполнителя.

    ``ring`` - ``PixelRingLite`` (методы возвращают ``True``, если команда ушла на устройство); ``call_later(сек,
    функция)`` отдаёт объект с ``cancel()``; ``answer(turn_id, started, reason)`` публикует ``ExpressionEvent``.
    """

    def __init__(self, ring: Any, call_later: Callable[[float, Callable[[], None]], Any],
                 answer: Callable[[str, bool, str], None]) -> None:
        self._ring, self._call_later, self._answer = ring, call_later, answer
        self._timer: Optional[Any] = None
        self._silenced = False
        self.mode = REST

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _apply(self, mode: str) -> bool:
        if mode in FLASHES:
            return bool(self._ring.mono(*FLASHES[mode]))
        return bool(getattr(self._ring, HARDWARE_MODES.get(mode, REST))())

    def _rest(self) -> None:
        self._timer = None
        self.mode = REST
        if self._silenced:
            self._ring.off()
        else:
            self._ring.trace()

    def on_turn_event(self, turn_id: str, stage: str, status: str) -> Optional[str]:
        """Событие стадии: включить режим кольца этой стадии; вернуть режим или ``None`` (кольцу нечего делать)."""
        row = kn.REFLEXES.get(stage)
        mode = row.ring if row is not None and stage in kn.TURN_STAGES and status != "skipped" else None
        if mode is None:
            return None
        if self._silenced:
            self._answer(turn_id, False, SILENCED)
            return None
        self._cancel_timer()
        ok = self._apply(mode)
        self.mode = mode if ok else REST
        self._answer(turn_id, ok, "" if ok else NO_DEVICE)
        self._timer = self._call_later(FLASH_S if mode in FLASHES else HOLD_MAX_S, self._rest)
        return mode

    def on_state(self, state: str) -> None:
        """Состояние диалога: ``silenced`` гасит кольцо, выход из тишины возвращает покой (остальное решают стадии)."""
        silenced = state.strip().lower() == "silenced"
        if silenced == self._silenced:
            return
        self._cancel_timer()
        self._silenced = silenced
        if silenced:
            self.mode = kn.REFLEXES["silenced"].ring or "off"
            self._ring.off()
        else:
            self._rest()
