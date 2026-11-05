"""«Прогретый» голос: свой профиль без имени, с которым давно идёт беседа (решение Шифу 05.11, после #4427).

Подхват бэклога, где автор фразы не узнан, а «Робот» сказал другой (``pickup_parties``), раньше заканчивался отказом
памяти «Голос знакомый, но не уверен чей». Шифу: если в комнате голос без имени, который уже «прогрелся» — у
него свой профиль (биометрия завела его с кличкой, #3858) и он сказал несколько реплик за сессию, — робот не
отказывает молча, а одной фразой кода просит назваться (``knowledge.ASK_INTRO_REASON``). Короткий разговор —
как раньше.

Критерий — только здесь (числа — ``knowledge.WARM_VOICE_*``): профиль без имени прогрет, если за последние
``WARM_VOICE_WINDOW_S`` у него не меньше ``WARM_VOICE_MIN_PHRASES`` оценённых биометрией фраз и первая из них звучала
не позже чем за ``WARM_VOICE_MIN_SPAN_S`` до последней. Источник — результаты биометрии фраз (хост,
``dialog_node._on_speaker``); сброс — вместе с окном диалога (режим e2e).
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, List, Mapping, Tuple

from rob_box_core.speaker_wire import NAME_NAMELESS, name_state
from rob_box_dialog import knowledge as kn


def no_warm_voices() -> Tuple[str, ...]:
    """Хост без учёта голосов (стенды, тесты): прогретых нет — поведение как до решения 05.11."""
    return ()


class WarmVoices:
    """Сколько и когда звучали профили без имени: :meth:`note` — на каждый результат биометрии, :meth:`warm` —
    прогретые сейчас."""

    def __init__(self, clock: Callable[[], float] = time.time) -> None:
        self._clock = clock
        self._heard: Dict[str, List[float]] = {}

    def note(self, result: Mapping[str, Any]) -> None:
        """Результат биометрии фразы (словарь ``speaker_wire``): профиль без имени — ещё одна его фраза."""
        person = str(result.get("speaker_id") or "")
        if person and not result.get("inconclusive") and name_state(result) == NAME_NAMELESS:
            self._heard.setdefault(person, []).append(self._clock())

    def warm(self) -> Tuple[str, ...]:
        """Профили без имени, прогретые сейчас (критерий модуля), по порядку первой фразы."""
        now = self._clock()
        out = []
        for person, times in self._heard.items():
            recent = [t for t in times if now - t <= kn.WARM_VOICE_WINDOW_S]
            self._heard[person] = recent
            if len(recent) >= kn.WARM_VOICE_MIN_PHRASES and recent[-1] - recent[0] >= kn.WARM_VOICE_MIN_SPAN_S:
                out.append(person)
        return tuple(out)

    def reset(self) -> None:
        """Новое окно диалога (режим e2e): беседа начинается заново."""
        self._heard.clear()
