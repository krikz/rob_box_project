"""turn_reasoning.py — какой ход думает (thinking), а какой нет (issue #3220).

Договорённость с Шифу (#3136 п.3):

* мгновенное превью (seeded club роутера медиакоманд, #3153) и сами
  медиакоманды (#3134) идут БЕЗ LLM — сюда они не доходят вовсе;
* думает только ход, у которого есть запас времени: DJ-переход по тику
  (LLM выбирает или сочиняет следующий трек, а предыдущий ещё звучит);
* ход юзера — БЕЗ thinking, в том числе заказ музыки и старт DJ-сета по
  реплике (issue #3265): человек ждёт в тишине. Живой замер 01.10: ход
  ``[TG] Ты Диджей …`` с thinking — 76 с до пустого ответа + 39 с ретрая,
  первый тул через 115 с; те же заказы без thinking — 3-15 с до первого
  тула. Раньше (#3220) заказ музыки думал; это откатано.

Ретраи — без thinking. Синтетический ретрай гуарда (Bug D/E, TurnGuards, …)
и ретрай Bug B DJ-перехода (``_dispatch_dj_turn(from_tick=False)``) идут,
когда бюджет перехода уже частично съеден первой попыткой; ещё 10-20 с на
рассуждение увели бы новый трек за границу формы. Бюджет перехода на один
ход с thinking и быстрый ретрай — ``DJ_REASONING_BUDGET_S`` +
``DJ_TURN_BUDGET_S`` в :mod:`.dj_mode`.

Решение хода нода кладёт в ``TURN_REASONING``
(:mod:`rob_box_harness.providers.reasoning`) на время ``_run_turn``;
провайдер MiniMax читает флаг и включает thinking первому вызову хода.
Модуль чистый: без ROS и без I/O.
"""

from __future__ import annotations

__all__ = ["turn_wants_reasoning"]


def turn_wants_reasoning(
    *,
    is_dj_auto: bool,
    dj_transition: bool,
    is_synthetic: bool,
) -> bool:
    """Думать ли этому ходу.

    Args:
        is_dj_auto: ход DJ-автоперехода (или его ретрай).
        dj_transition: ход запущен тиком DJ — свежий переход, не ретрай Bug B.
        is_synthetic: синтетический ретрай гуарда.
    """
    if is_synthetic:
        return False
    return is_dj_auto and dj_transition
