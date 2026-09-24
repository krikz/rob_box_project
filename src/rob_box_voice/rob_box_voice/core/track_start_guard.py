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

Issue #2878 — тот же живой прогон, соседняя находка: в DJ-ходе модель
вызвала ``speak_text`` ШЕСТЬ раз подряд (длинные монологи поверх трека),
пока не упёрлась в тот же ``_MAX_TOOL_ITERATIONS=8``. Трек-гард выше её не
останавливал — ``speak_text`` не запускает трек. Этот же класс теперь
считает и реплики DJ-хода:

* «DJ-ход» — эвристика без пробрасывания ``is_dj_auto`` через AgentCore:
  ход уже запустил трек (:attr:`TrackStartGuard.started_tool`) или уже
  вызвал ``set_dj_mode`` в этом ходе (см. :meth:`record`);
* в DJ-ходе разрешено :data:`DEFAULT_DJ_SPEAK_LIMIT` (1) вызовов
  ``speak_text``, кроме самого первого хода сета («[DJ_AUTO — СТАРТ
  ВЕЧЕРИНКИ]», распознаётся по ``set_dj_mode(plan=...)``) — там
  :data:`PARTY_START_DJ_SPEAK_LIMIT` (2): один на представление диджея,
  один на трек;
* лишние вызовы не исполняются — отказ :data:`SPEAK_REFUSAL_MESSAGE`;
* обычные (не-DJ) ходы не ограничены (:attr:`TrackStartGuard.is_dj_turn`
  остаётся ``False``).

Подключён в ``SchedulerToolExecutor``
(:mod:`rob_box_voice.scheduler.tool_executor`)
— это единственная точка, через которую проходят все вызовы тулов личности.
"""

from __future__ import annotations

import json
import re
from typing import Any, Mapping, Optional

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

# ---------------------------------------------------------------------------
# Issue #2878 — DJ speak_text limit
# ---------------------------------------------------------------------------

#: Тул реплики диджея.
SPEAK_TOOL = "speak_text"

#: Тул, чьим появлением в ходе (независимо от успеха) мы тоже считаем ход
#: «DJ-ходом» — живой лог: модель вызывает ``set_dj_mode`` ДО того, как
#: успешный ``compose_music`` взводит :attr:`TrackStartGuard.started_tool`.
DJ_MODE_TOOL = "set_dj_mode"

#: Лимит ``speak_text`` в обычном DJ-ходе (переход между треками).
DEFAULT_DJ_SPEAK_LIMIT = 1

#: Лимит на самом первом ходе сета («[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ]») —
#: одна реплика на представление персоны, одна на анонс трека #1.
PARTY_START_DJ_SPEAK_LIMIT = 2

#: Машиночитаемый код отказа для лишнего ``speak_text`` в DJ-ходе.
SPEAK_REFUSAL_ERROR_CODE = "dj_speak_already_used_this_turn"

#: Текст отказа, который видит модель.
SPEAK_REFUSAL_MESSAGE = (
    "Реплика уже прозвучала в этом DJ-ходе — повторный speak_text НЕ "
    "выполнен. Заверши ход: следующая реплика — на следующем переходе."
)

#: ~140 символов / 1-2 предложения (issue #2878 acceptance).
DJ_SPEAK_MAX_CHARS = 140
DJ_SPEAK_MAX_SENTENCES = 2

_SENTENCE_SPLIT_RE = re.compile(r"(?<=[.!?…])\s+")


def trim_dj_speech(text: str, *, max_chars: int = DJ_SPEAK_MAX_CHARS) -> str:
    """Trim DJ-turn ``speak_text`` to ~*max_chars* / first 1-2 sentences.

    Live 23.09.2026: DJ monologues ran ~30-40 words each, ``speak_text``
    queued on the VOICE channel one after another on top of the track.
    Sentence-aware first (keeps the reply readable) with a hard
    word-boundary cutoff as a backstop for a single very long sentence.
    Empty/whitespace-only input is returned unchanged (nothing to trim).
    """
    stripped = text.strip()
    if not stripped:
        return stripped
    sentences = _SENTENCE_SPLIT_RE.split(stripped)
    trimmed = " ".join(sentences[:DJ_SPEAK_MAX_SENTENCES]).strip()
    if len(trimmed) <= max_chars:
        return trimmed
    cut = trimmed[:max_chars].rsplit(" ", 1)[0].rstrip(" ,.-—")
    return (cut or trimmed[:max_chars]).rstrip() + "…"


def _has_plan(args: Optional[Mapping[str, Any]]) -> bool:
    """``True`` if *args* is a ``set_dj_mode`` call carrying a set plan.

    Only the very first turn of a set ("[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ]",
    ``dj_mode.py``) calls ``set_dj_mode(plan=<...>)``; every later
    transition passes ``next_transition_sec`` without a plan. Cheap
    stand-in for threading ``is_dj_auto``/turn text through the executor.
    """
    if not args:
        return False
    plan = args.get("plan")
    return bool(plan) if isinstance(plan, str) else bool(plan)


def speak_refusal_content(speak_count: int, speak_limit: int) -> str:
    """JSON-тело отказа лишнего ``speak_text`` в DJ-ходе."""
    return json.dumps(
        {
            "success": False,
            "error": SPEAK_REFUSAL_ERROR_CODE,
            "tool": SPEAK_TOOL,
            "speak_count_this_turn": speak_count,
            "speak_limit_this_turn": speak_limit,
            "message": SPEAK_REFUSAL_MESSAGE,
        },
        ensure_ascii=False,
    )


class TrackStartGuard:
    """Per-turn лимит: один успешный запуск трека на ход.

    Вызовы идут последовательно (``AgentCore._execute_tool_batch`` ждёт
    каждый ``execute``), поэтому блокировка не нужна.
    """

    def __init__(self) -> None:
        self._started: Optional[str] = None
        # Issue #2878 — счётчик speak_text и признаки «это DJ-ход».
        self._speak_count: int = 0
        self._dj_mode_seen: bool = False
        self._dj_mode_plan_seen: bool = False

    @property
    def started_tool(self) -> Optional[str]:
        """Имя тула, успешно запустившего трек в этом ходе, или ``None``."""
        return self._started

    @property
    def is_dj_turn(self) -> bool:
        """``True`` — этот ход уже проявил себя как DJ-ход (issue #2878).

        Эвристика (без пробрасывания ``is_dj_auto`` через AgentCore, см.
        module docstring): ход уже запустил трек ИЛИ уже вызвал
        ``set_dj_mode`` — оба возможны только в DJ-оркестрации.
        """
        return self._started is not None or self._dj_mode_seen

    @property
    def speak_count(self) -> int:
        """Сколько ``speak_text`` уже исполнено в этом ходе."""
        return self._speak_count

    @property
    def speak_limit(self) -> int:
        """Лимит ``speak_text`` для текущего хода (issue #2878).

        :data:`PARTY_START_DJ_SPEAK_LIMIT` на самом первом ходе сета
        (``set_dj_mode(plan=...)`` в этом ходе), иначе
        :data:`DEFAULT_DJ_SPEAK_LIMIT`.
        """
        if self._dj_mode_plan_seen:
            return PARTY_START_DJ_SPEAK_LIMIT
        return DEFAULT_DJ_SPEAK_LIMIT

    def reset(self) -> None:
        """Граница хода: следующий запуск снова разрешён."""
        self._started = None
        self._speak_count = 0
        self._dj_mode_seen = False
        self._dj_mode_plan_seen = False

    def should_refuse(self, tool_name: str) -> bool:
        """``True`` — это повторный запуск трека в ходе, исполнять нельзя."""
        return tool_name in TRACK_STARTING_TOOLS and self._started is not None

    def should_refuse_speak(self) -> bool:
        """``True`` — DJ-ход уже израсходовал лимит ``speak_text``.

        Non-DJ ходы (:attr:`is_dj_turn` ``False``) никогда не отказывают —
        обычный диалог репликами не ограничен.
        """
        return self.is_dj_turn and self._speak_count >= self.speak_limit

    def record_speak(self) -> None:
        """Учесть один исполненный (не отказанный) ``speak_text``."""
        self._speak_count += 1

    def record(
        self,
        tool_name: str,
        *,
        is_error: bool,
        args: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """Учесть результат исполненного вызова.

        Упавший запуск трека (``is_error``) лимит не расходует — модель
        вправе исправиться повтором в том же ходе. ``set_dj_mode`` метит
        ход как DJ-ход независимо от успеха (issue #2878) — сам факт
        вызова уже говорит о контексте, а не об исходе побочного эффекта.
        """
        if tool_name in TRACK_STARTING_TOOLS and not is_error:
            self._started = tool_name
        if tool_name == DJ_MODE_TOOL:
            self._dj_mode_seen = True
            if _has_plan(args):
                self._dj_mode_plan_seen = True


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
    "DEFAULT_DJ_SPEAK_LIMIT",
    "DJ_MODE_TOOL",
    "DJ_SPEAK_MAX_CHARS",
    "DJ_SPEAK_MAX_SENTENCES",
    "PARTY_START_DJ_SPEAK_LIMIT",
    "REFUSAL_ERROR_CODE",
    "REFUSAL_MESSAGE",
    "SPEAK_REFUSAL_ERROR_CODE",
    "SPEAK_REFUSAL_MESSAGE",
    "SPEAK_TOOL",
    "TRACK_STARTING_TOOLS",
    "TrackStartGuard",
    "refusal_content",
    "speak_refusal_content",
    "trim_dj_speech",
]
