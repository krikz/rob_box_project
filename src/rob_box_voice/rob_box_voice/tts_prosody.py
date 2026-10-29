#!/usr/bin/env python3
"""
tts_prosody.py — скорость и высота голоса «на лету»: одна таблица «провайдер → параметр tts_node» (#4022).

Было: ``set_speed`` писал только ``yandex_speed``, а звучал MiniMax (он читает ``minimax_speed``, и ``tts_node`` не
принимал его на лету); ``set_pitch`` писал ``pitch_shift``, который слышен только при ``chipmunk_mode`` (в проде он
выключен). Оба тула отвечали «готово» по факту записи параметра, а голос не менялся (ADR-0148, capability-honest).

Здесь знание лежит в одном месте, его читают обе стороны:

* тулы ``set_speed`` / ``set_pitch`` (``rob_box_mcp_tools.tools.system``): какой параметр у АКТИВНОГО провайдера,
  новое значение шага, отказ для провайдера без регулировки; ответ строится из значения, прочитанного назад;
* ``tts_node``: какие параметры принимать на лету (``LIVE_AXES``) и какой ``voice_setting.pitch`` слать MiniMax.

Скорость — параметр провайдера: ``minimax_speed`` (T2A ``voice_setting.speed``) и ``yandex_speed`` (``Hints.speed``).
У Silero живой скорости нет (SSML ``prosody rate`` нода не строит) — отказ.

Тон — один коэффициент ``pitch_shift`` (1.0 = норма), а применяется он так, как умеет активный путь:

* ``chipmunk_mode=true`` — передискретизацией проигрыша, любой провайдер (как и было);
* ``chipmunk_mode=false`` — только там, где у провайдера есть голосовой параметр тона: MiniMax, ``voice_setting.pitch``
  в полутонах (``pitch_semitones``). У остальных тона нет — отказ.

Чистый Python без ROS: модуль импортируют и ``tts_node``, и ``rob_box_mcp_tools``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

#: Шаг тула «быстрее/медленнее», «выше/ниже» — на оси, в единицах самой оси.
STEP = 0.2

#: Предел ``voice_setting.pitch`` MiniMax T2A v2, полутонов в обе стороны (0 = исходный тон).
MAX_SEMITONES = 12


@dataclass(frozen=True)
class Axis:
    """Одна регулируемая ось: параметр ``tts_node`` (он же атрибут ноды) и границы шага тула."""

    param: str
    lo: float
    hi: float
    normal: float = 1.0


#: Скорость по провайдерам: у кого нет записи — живой скорости нет, тул отказывает (не «готово»).
SPEED_AXES: Dict[str, Axis] = {
    "minimax": Axis("minimax_speed", 0.5, 2.0),
    "yandex": Axis("yandex_speed", 0.5, 2.0),
}

#: Ось тона одна на всех провайдеров; кто её слышит — решает :func:`pitch_axis`.
PITCH_AXIS = Axis("pitch_shift", 0.5, 2.0)

#: Провайдеры с голосовым параметром тона (слышен без ``chipmunk_mode``).
PITCH_NATIVE = frozenset({"minimax"})

#: Живые параметры, которые ``tts_node`` принимает на лету: имя параметра → ось. Атрибут ноды называется так же.
LIVE_AXES: Dict[str, Axis] = {
    axis.param: axis for axis in (*SPEED_AXES.values(), PITCH_AXIS)
}


def speed_axis(provider: str) -> Optional[Axis]:
    """Ось скорости активного провайдера или ``None``: у него нет живой регулировки скорости."""
    return SPEED_AXES.get(provider)


def pitch_axis(provider: str, chipmunk: bool) -> Optional[Axis]:
    """Ось тона или ``None``: при выключенном ``chipmunk_mode`` тон слышен только у ``PITCH_NATIVE``."""
    return PITCH_AXIS if chipmunk or provider in PITCH_NATIVE else None


def target_value(axis: Axis, direction: int, current: float) -> float:
    """Значение оси после шага: ``direction`` −1/+1 — на ``STEP`` в границах оси, 0 — обычное значение оси."""
    if direction == 0:
        return axis.normal
    return round(min(axis.hi, max(axis.lo, current + direction * STEP)), 2)


def pitch_semitones(ratio: float) -> int:
    """Коэффициент тона → полутона ``voice_setting.pitch`` MiniMax (1.2 → +3, 0.8 → −4), в пределах ±12."""
    if ratio <= 0:
        return 0
    return max(-MAX_SEMITONES, min(MAX_SEMITONES, round(12 * math.log2(ratio))))


def minimax_pitch(base: Optional[int], ratio: float, chipmunk: bool) -> Optional[int]:
    """``voice_setting.pitch`` реплики MiniMax или ``None`` (поле не посылается).

    ``base`` — статический ``minimax_pitch`` из конфига; живой коэффициент ``ratio`` (``pitch_shift``) добавляется к нему
    полутонами. При ``chipmunk_mode`` коэффициент уже съела передискретизация проигрыша — второй раз он не применяется.
    """
    live = 0 if chipmunk else pitch_semitones(ratio)
    if base is None and live == 0:
        return None
    return max(-MAX_SEMITONES, min(MAX_SEMITONES, (base or 0) + live))


def semitones_ru(count: int) -> str:
    """``+3 полутона`` / ``-1 полутон`` / ``+5 полутонов`` — для ответа тула."""
    tail = abs(count) % 100
    if 11 <= tail <= 14:
        word = "полутонов"
    else:
        word = {1: "полутон", 2: "полутона", 3: "полутона", 4: "полутона"}.get(
            tail % 10, "полутонов"
        )
    return f"{count:+d} {word}"


def describe_pitch(ratio: float, chipmunk: bool) -> str:
    """Как назвать фактический тон в ответе: полутона у голосового параметра, коэффициент у чипмунка."""
    return f"×{ratio:g}" if chipmunk else semitones_ru(pitch_semitones(ratio))
