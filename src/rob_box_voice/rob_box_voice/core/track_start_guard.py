"""track_start_guard.py — не больше одного запуска трека за ход (issue #2859).

Живой прогон 23.09.2026, DJ-переход #23: в ОДНОМ ходе LLM восемь раз
подряд вызвала ``compose_music`` + ``set_dj_mode``. Каждый вызов вернул
success, каждый начинал с ``Clock.clear()`` — музыка переключалась каждые
~3 с, пока AgentCore не упёрся в ``_MAX_TOOL_ITERATIONS=8``. Модель видит
«успех» и «продолжает сет» внутри хода; ограничивало её только общее число
итераций.

Этот модуль — чистый (без rclpy и I/O) счётчик на один ход:

* первый УСПЕШНЫЙ запуск трека в ходе проходит;
* повторный запуск в том же ходе не исполняется — модели уходит понятный
  отказ (:data:`REFUSAL_MESSAGE`), чтобы она закончила ход репликой;
* запуск, который УПАЛ, лимит не расходует — исправление после ошибки
  разрешено;
* :meth:`TrackStartGuard.reset` на границе хода снимает ограничение
  (следующий DJ-переход или просьба юзера сменить трек — новый ход).

Подключён в ``SchedulerToolExecutor``
(:mod:`rob_box_voice.scheduler.tool_executor`)
— это единственная точка, через которую проходят все вызовы тулов личности.
"""

from __future__ import annotations

import json
from typing import Optional

from rob_box_voice.core.dialogue_guards import (
    MUSIC_STARTING_TOOLS,
    USER_MUSIC_SATISFYING_TOOLS,
)

#: Тулы, запускающие трек (каждый начинает с ``Clock.clear()`` / смены mp3).
#: Оба набора выводятся из capability-флагов ``TOOL_CATALOG``, отдельного
#: рукописного списка здесь нет: ``compose_music`` / ``execute_music_code``
#: / ``gen_play_from_library`` / ``generate_music`` (``starts_music``) и
#: ``load_track`` (``satisfies_user_music``).
TRACK_STARTING_TOOLS: frozenset = (
    MUSIC_STARTING_TOOLS | USER_MUSIC_SATISFYING_TOOLS
)

#: Машиночитаемый код отказа в ответе тула.
REFUSAL_ERROR_CODE = "track_already_started_this_turn"

#: Текст отказа, который видит модель.
REFUSAL_MESSAGE = (
    "Трек уже запущен в этом ходе — повторный запуск НЕ выполнен, играет "
    "прежний трек. Заверши ход короткой репликой (speak_text); следующий "
    "трек — на следующем переходе."
)


class TrackStartGuard:
    """Per-turn лимит: один успешный запуск трека на ход.

    Вызовы идут последовательно (``AgentCore._execute_tool_batch`` ждёт
    каждый ``execute``), поэтому блокировка не нужна.
    """

    def __init__(self) -> None:
        self._started: Optional[str] = None

    @property
    def started_tool(self) -> Optional[str]:
        """Имя тула, успешно запустившего трек в этом ходе, или ``None``."""
        return self._started

    def reset(self) -> None:
        """Граница хода: следующий запуск снова разрешён."""
        self._started = None

    def should_refuse(self, tool_name: str) -> bool:
        """``True`` — это повторный запуск трека в ходе, исполнять нельзя."""
        return tool_name in TRACK_STARTING_TOOLS and self._started is not None

    def record(self, tool_name: str, *, is_error: bool) -> None:
        """Учесть результат исполненного вызова.

        Упавший запуск (``is_error``) лимит не расходует — модель вправе
        исправиться повтором в том же ходе.
        """
        if tool_name in TRACK_STARTING_TOOLS and not is_error:
            self._started = tool_name


def refusal_content(tool_name: str, started_tool: Optional[str]) -> str:
    """JSON-тело отказа для ``ToolResult.content``."""
    return json.dumps(
        {
            "success": False,
            "error": REFUSAL_ERROR_CODE,
            "tool": tool_name,
            "already_started_by": started_tool,
            "message": REFUSAL_MESSAGE,
        },
        ensure_ascii=False,
    )


__all__ = [
    "REFUSAL_ERROR_CODE",
    "REFUSAL_MESSAGE",
    "TRACK_STARTING_TOOLS",
    "TrackStartGuard",
    "refusal_content",
]
