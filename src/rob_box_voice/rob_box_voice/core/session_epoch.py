"""session_epoch.py — поколение диалоговой сессии (issue #2835).

«Новая сессия» (``DialogueNode._reset_dialogue_session``) отменяет
in-flight turn, но у отменённого хода остаются хвосты, которые живут
ПОСЛЕ сброса:

* синхронные ретраи music-/tool-гуардов из ``finally`` ``_run_turn``:
  отменённый ход приходит туда с ``result=None`` → ``tools_called=()`` →
  Bug C видит «музыку просили, тула нет» и шлёт ``[CRITICAL]``-ретрай
  (живой лог 23.09 14:18:08 — ретрай стартует в ту же секунду, что и
  ``session reset``);
* ретраи, уже поставленные в loop (``run_coroutine_threadsafe``), но ещё
  не начавшиеся — ``_cancel_run`` отменяет только текущий ``_run_task``;
* запоздалый ``set_dj_mode(enabled=true)`` из mcp_server (другой процесс,
  топик ``/voice/dj_mode``) от хода, начатого до сброса.

:class:`SessionEpoch` — счётчик поколений: сброс сессии его продвигает,
каждый ход несёт поколение, в котором родился, и ход/ретрай чужого
поколения отбрасывается. Модуль чистый (без ROS), чтобы тестировать
политику отдельно от ноды.
"""

from __future__ import annotations

import contextvars
import json
from typing import Optional

#: Поколение сессии, в котором родился ТЕКУЩИЙ ход. Выставляется в
#: ``_run_turn`` на время хода; asyncio-задачи копируют контекст, поэтому
#: синхронный ретрай, задиспатченный изнутри хода, наследует поколение
#: родителя, а не текущее (после сброса — уже новое). Вне хода (STT-колбэк,
#: DJ-тикер в ROS-потоке) переменная не выставлена.
TURN_EPOCH: contextvars.ContextVar[Optional[int]] = contextvars.ContextVar(
    "rob_box_turn_epoch", default=None
)


def _payload_enables_dj(payload: str) -> bool:
    """``True`` если JSON ``/voice/dj_mode`` включает DJ.

    Битый payload — ``False``: решать, что с ним делать (warning), будет
    ``DJModeController.handle_message``, а не забор.
    """
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return False
    return isinstance(data, dict) and bool(data.get("enabled", False))


class SessionEpoch:
    """Счётчик поколений диалоговой сессии."""

    def __init__(self) -> None:
        self._value = 0
        # «Забор» на включение DJ: поднимается сбросом сессии, снимается
        # первым ходом, родившимся в новом поколении. Пока забор стоит,
        # ``enabled=true`` может прийти только от хода старой сессии.
        self._dj_enable_fenced = False

    @property
    def current(self) -> int:
        return self._value

    def advance(self) -> int:
        """Сброс сессии: новое поколение + забор на включение DJ."""
        self._value += 1
        self._dj_enable_fenced = True
        return self._value

    def is_stale(self, epoch: Optional[int]) -> bool:
        """Поколение ``epoch`` устарело. ``None`` — «не знаю», не устарело."""
        return epoch is not None and epoch != self._value

    def epoch_for_dispatch(self) -> int:
        """Поколение для нового хода: родительское, если диспатч идёт
        изнутри хода (ретрай/дренаж очереди), иначе текущее."""
        parent = TURN_EPOCH.get()
        return self._value if parent is None else parent

    def note_turn_started(self, epoch: Optional[int]) -> None:
        """Ход текущего поколения начался — снимаем забор на DJ."""
        if not self.is_stale(epoch):
            self._dj_enable_fenced = False

    def retries_allowed(
        self, *, turn_epoch: Optional[int], cancelled: bool
    ) -> bool:
        """Можно ли ходу диспатчить post-turn ретраи (music/tool гуарды).

        Нельзя, если ход отменён (barge-in / сброс — результата нет, «тула
        не вызвали» тут ложь) или если за время хода сессию сбросили.
        """
        return not cancelled and not self.is_stale(turn_epoch)

    def admits_dj_payload(self, payload: str) -> bool:
        """Пропустить ли сообщение ``/voice/dj_mode`` в DJ-контроллер.

        Выключение проходит всегда. Включение режется, пока после сброса
        не начался ни один ход новой сессии: такой ``enabled=true`` может
        прийти только от хода, начатого до сброса.
        """
        return not (self._dj_enable_fenced and _payload_enables_dj(payload))
