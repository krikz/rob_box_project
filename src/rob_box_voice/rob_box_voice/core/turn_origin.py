"""turn_origin.py — чей ход: юзера или DJ-автоперехода (issue #3144).

Живой прогон 28.09.2026 18:48: DJ-автопереход (``is_dj_auto=True``) не
вызвал музыкальный тул, Bug E отправил ретрай — и ретрай ушёл БЕЗ
``is_dj_auto``. Ход превратился в «ход юзера» ``[Speaker:unknown]``,
промпт DJ_AUTO попал в «запрос юзера», Bug C (music_user) решил, что
юзер просит музыку, выжег бюджет ретраев и сказал «Музыка играет, а вот
эту просьбу я выполнить не смог» — хотя юзер ничего не просил.

Ретраев гуардов много (Bug D, Bug E, TurnGuards, universal/phantom
action, tool-skipped, …), и каждый сам зовёт ``_dispatch_turn``. Чинить
каждый вызов по отдельности — значит забыть следующий. Поэтому
происхождение хода хранится в :data:`TURN_IS_DJ_AUTO` (ContextVar, как
``TURN_EPOCH`` в :mod:`session_epoch`): ``_run_turn`` выставляет его на
время хода, asyncio-задача ретрая копирует контекст родителя, а
``_dispatch_turn`` через :func:`retry_is_dj_auto` наследует флаг для
любого синтетического ретрая. Модуль чистый (без ROS).
"""

from __future__ import annotations

import contextvars

#: ``True`` на время хода, запущенного DJ-автопереходом (или ретраем
#: такого хода). Вне хода (STT-колбэк, DJ-тикер в ROS-потоке) — ``False``.
TURN_IS_DJ_AUTO: contextvars.ContextVar[bool] = contextvars.ContextVar(
    "rob_box_turn_is_dj_auto", default=False
)

#: Issue #3247 — ``True`` на время DJ_AUTO-хода, идущего ПОСЛЕ промпта
#: «ФИНАЛЬНЫЙ ТРЕК» сета (``DJModeController.is_final_turn``): такой ход
#: сет только завершает, ``set_dj_mode(enabled=true)`` гард исполнителя
#: тулов не исполняет. Ретраи хода наследуют флаг через контекст задачи.
TURN_DJ_SET_FINAL: contextvars.ContextVar[bool] = contextvars.ContextVar(
    "rob_box_turn_dj_set_final", default=False
)


def retry_is_dj_auto(*, is_dj_auto: bool, is_synthetic: bool) -> bool:
    """Флаг ``is_dj_auto`` для хода, который сейчас диспатчится.

    * явный ``is_dj_auto=True`` (тик DJ, Bug B) — как есть;
    * синтетический ретрай гуарда (``is_synthetic=True``) наследует
      происхождение РОДИТЕЛЬСКОГО хода: ретрай DJ-перехода — тоже
      DJ-переход, а не реплика юзера;
    * несинтетический ход (реплика юзера, дренаж очереди) — никогда не
      DJ, даже если диспатчится изнутри DJ-хода.
    """
    if is_dj_auto:
        return True
    return is_synthetic and TURN_IS_DJ_AUTO.get()
