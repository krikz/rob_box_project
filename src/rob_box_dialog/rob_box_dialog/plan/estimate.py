"""Сколько ещё звучать реплике — для ``PlanEvent queued{eta_ms}`` и блока LLM «старт ≈ через N с» (ADR-0150 §5.6.1).

Оценка — по событиям владельца речи (``SpeechEvent``): длительность синтеза ``duration_ms``, если он её сообщил; иначе
длина текста на скорость речи. Скорость — замер по ``progress`` (``spoken_chars / seconds``) этого же процесса; пока
замера нет — ``knowledge.SPEECH_CHARS_PER_S_GUESS`` (оценка, не замер).

Прежний ``scheduler/pregen/estimator.py`` сюда не перенесён: это был гейт уверенности спекулятивного пред-синтеза
(разброс прошлых длительностей), а не оценка длительности; он удалён вместе с ``pregen`` (PR-7c ч.4а).
"""

from __future__ import annotations

from typing import Optional

from rob_box_dialog import knowledge as kn

#: Вес нового замера скорости (скользящее среднее): голос и провайдер меняются, старые замеры стареют.
_RATE_WEIGHT = 0.3
#: Замер короче этого не в счёт: первые доли секунды — задержка вывода, а не темп речи.
_MIN_SAMPLE_S = 1.0


class SpeechRate:
    """Скорость речи, символов в секунду: замер по ``SpeechEvent progress``, до замера — оценка из ``knowledge``."""

    def __init__(self) -> None:
        self._rate: Optional[float] = None

    @property
    def measured(self) -> bool:
        return self._rate is not None

    @property
    def chars_per_s(self) -> float:
        return self._rate if self._rate is not None else kn.SPEECH_CHARS_PER_S_GUESS

    def observe(self, spoken_chars: int, seconds: float) -> None:
        """Замер из ``progress``/``finished``: сколько символов прозвучало за сколько секунд от начала реплики."""
        if seconds < _MIN_SAMPLE_S or spoken_chars <= 0:
            return
        sample = spoken_chars / seconds
        self._rate = sample if self._rate is None else self._rate + _RATE_WEIGHT * (sample - self._rate)


def remaining_s(text_len: int, duration_ms: int, spoken_chars: int, seconds: float, rate: SpeechRate) -> Optional[float]:
    """Сколько ещё звучать реплике: по длительности синтеза минус прозвучавшее, иначе непрозвучавший текст на
    скорость; ни длительности, ни длины текста — ``None`` (неизвестно)."""
    if duration_ms > 0:
        return max(0.0, duration_ms / 1000.0 - seconds)
    if text_len > 0:
        return max(0, text_len - spoken_chars) / rate.chars_per_s
    return None


__all__ = ["SpeechRate", "remaining_s"]
