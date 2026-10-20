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
  не начавшиеся — ``_cancel_run`` отменяет только текущий ``_run_task``.

(Забор на запоздалый ``set_dj_mode(enabled=true)`` удалён в ADR-0149 PR-13a
вместе с DJ-контроллером старого пути.)

:class:`SessionEpoch` — счётчик поколений: сброс сессии его продвигает,
каждый ход несёт поколение, в котором родился, и ход/ретрай чужого
поколения отбрасывается. Модуль чистый (без ROS), чтобы тестировать
политику отдельно от ноды.
"""

from __future__ import annotations

import contextvars
from typing import Optional

#: Поколение сессии, в котором родился ТЕКУЩИЙ ход. Выставляется в
#: ``_run_turn`` на время хода; asyncio-задачи копируют контекст, поэтому
#: синхронный ретрай, задиспатченный изнутри хода, наследует поколение
#: родителя, а не текущее (после сброса — уже новое). Вне хода (STT-колбэк)
#: переменная не выставлена.
TURN_EPOCH: contextvars.ContextVar[Optional[int]] = contextvars.ContextVar(
    "rob_box_turn_epoch", default=None
)


class SessionEpoch:
    """Счётчик поколений диалоговой сессии."""

    def __init__(self) -> None:
        self._value = 0

    @property
    def current(self) -> int:
        return self._value

    def advance(self) -> int:
        """Сброс сессии: новое поколение."""
        self._value += 1
        return self._value

    def is_stale(self, epoch: Optional[int]) -> bool:
        """Поколение ``epoch`` устарело. ``None`` — «не знаю», не устарело."""
        return epoch is not None and epoch != self._value

    def epoch_for_dispatch(self) -> int:
        """Поколение для нового хода: родительское, если диспатч идёт
        изнутри хода (ретрай/дренаж очереди), иначе текущее."""
        parent = TURN_EPOCH.get()
        return self._value if parent is None else parent

    def retries_allowed(
        self, *, turn_epoch: Optional[int], cancelled: bool
    ) -> bool:
        """Можно ли ходу диспатчить post-turn ретраи (tool-гуард).

        Нельзя, если ход отменён (barge-in / сброс — результата нет, «тула
        не вызвали» тут ложь) или если за время хода сессию сбросили.
        """
        return not cancelled and not self.is_stale(turn_epoch)
