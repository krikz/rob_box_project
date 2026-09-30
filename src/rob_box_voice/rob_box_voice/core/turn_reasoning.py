"""turn_reasoning.py — какой ход думает (thinking), а какой нет (issue #3220).

Договорённость с Шифу (#3136 п.3):

* мгновенное превью (seeded club роутера медиакоманд, #3153) и сами
  медиакоманды (#3134) идут БЕЗ LLM — сюда они не доходят вовсе;
* думает ход, у которого есть время: DJ-переход по тику (LLM выбирает или
  сочиняет следующий трек) и ход юзера с заказом музыки (сочинение, подбор);
* остальные голосовые ходы — без thinking, как раньше: там важна
  латентность ответа.

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

from typing import Optional

from .dialogue_guards import is_music_stop_command, user_wants_music

__all__ = ["is_music_request", "turn_wants_reasoning"]


def is_music_request(text: str) -> bool:
    """Реплика заказывает музыку (сочинить, подобрать, развить), а не стоп."""
    if not text:
        return False
    return user_wants_music(text) and not is_music_stop_command(text)


def turn_wants_reasoning(
    *,
    is_dj_auto: bool,
    dj_transition: bool,
    is_synthetic: bool,
    user_input: str,
    raw_user_command: Optional[str] = None,
) -> bool:
    """Думать ли этому ходу.

    Args:
        is_dj_auto: ход DJ-автоперехода (или его ретрай).
        dj_transition: ход запущен тиком DJ — свежий переход, не ретрай Bug B.
        is_synthetic: синтетический ретрай гуарда.
        user_input: текст хода (для DJ — промпт перехода).
        raw_user_command: реплика юзера как пришла из STT, если есть —
            по ней, а не по обёрнутому ``user_input``, узнаём заказ музыки.
    """
    if is_synthetic:
        return False
    if is_dj_auto:
        return dj_transition
    text = raw_user_command if raw_user_command is not None else user_input
    return is_music_request(text)
